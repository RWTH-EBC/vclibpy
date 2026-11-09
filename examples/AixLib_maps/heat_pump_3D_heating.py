import dataclasses
import logging
import os
import pathlib
import shutil
from random import random
from typing import Literal, Optional

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from vclibpy import datamodels
from vclibpy import utils
from vclibpy import Inputs
from vclibpy.components.compressors import RotaryCompressor
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.heat_exchangers import heat_transfer
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU
from vclibpy.flowsheets import StandardCycle, VaporInjectionEconomizer, VaporInjectionPhaseSeparator
from vclibpy.utils.nominal_design import nominal_hp_design

logger = logging.getLogger(__name__)


class BoundedRotaryCompressor(RotaryCompressor):
    """
    Rotary compressor with bounded, physically plausible efficiencies.

      eta_is   = eta_is_max  * (1 - k_pi*ln(pi/pi_opt)^2) * (1 - k_n*(n - n_opt)^2)
      eta_mech = eta_mech_max * (1 - k_n_mech*(n - n_opt)^2)   (motor + inverter + mechanics, effective)
      lambda_h = Engelpracht regression, evaluated with the speed clipped to its fit range

    The plain Engelpracht regressions for eta_is and eta_mech were fitted for ~25-105 Hz
    and pi ~2-8. At N_max = 125 Hz and high lifts they extrapolate to eta_is < 0.1 and
    unrealistic discharge temperatures. n is the relative speed (0..1). All efficiencies
    are clipped to [min, max]. Defaults are calibrated against Viessmann datasheets
    (EN 14511, which includes fan/pump shares, so eta_mech is an effective value).
    """

    def __init__(self, N_max, V_h,
                 eta_is_max=0.70, pi_opt=2.5, k_pi=0.20, eta_is_min=0.40,
                 n_opt=0.6, k_n=0.25,
                 eta_mech_max=0.92, k_n_mech=0.15, eta_mech_min=0.80,
                 N_fit_range=(25.0, 105.0)):
        super().__init__(N_max=N_max, V_h=V_h)
        self.eta_is_max, self.pi_opt, self.k_pi, self.eta_is_min = eta_is_max, pi_opt, k_pi, eta_is_min
        self.n_opt, self.k_n = n_opt, k_n
        self.eta_mech_max, self.k_n_mech, self.eta_mech_min = eta_mech_max, k_n_mech, eta_mech_min
        self.N_fit_range = N_fit_range

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        pi = p_outlet / self.state_inlet.p
        f_pi = 1 - self.k_pi * np.log(pi / self.pi_opt) ** 2
        f_n = 1 - self.k_n * (inputs.n - self.n_opt) ** 2
        return float(np.clip(self.eta_is_max * f_pi * f_n, self.eta_is_min, self.eta_is_max))

    def get_eta_mech(self, inputs: Inputs) -> float:
        f_n = 1 - self.k_n_mech * (inputs.n - self.n_opt) ** 2
        return float(np.clip(self.eta_mech_max * f_n, self.eta_mech_min, self.eta_mech_max))

    def get_lambda_h(self, inputs: Inputs) -> float:
        N_clip = float(np.clip(self.get_n_absolute(inputs.n), *self.N_fit_range))
        n_orig = inputs.n
        inputs.set("n", N_clip / self.N_max)
        try:
            lam = super().get_lambda_h(inputs)
        finally:
            inputs.set("n", n_orig)
        return float(np.clip(lam, 0.5, 0.95))


@dataclasses.dataclass
class HXParams:
    """One physical heat exchanger. `A` is the SECONDARY-side (outer) area; vclibpy's
    NTU model references k to it, and ratio_outer_to_inner_area = A_outer / A_refrigerant."""
    A: float
    secondary_medium: str
    ratio_outer_to_inner_area: float
    alpha_two_phase: float
    alpha_gas: float
    alpha_liquid: float
    alpha_secondary: float
    wall_lambda: float
    wall_thickness: float

    def kwargs(self) -> dict:
        return dict(
            A=self.A,
            secondary_medium=self.secondary_medium,
            flow_type="counter",
            ratio_outer_to_inner_area=self.ratio_outer_to_inner_area,
            two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=self.alpha_two_phase),
            gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=self.alpha_gas),
            liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=self.alpha_liquid),
            secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=self.alpha_secondary),
            wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=self.wall_lambda,
                                                               thickness=self.wall_thickness),
        )

    def UA_two_phase(self) -> float:
        # Same formula as BasicNTU.calc_k
        r = self.ratio_outer_to_inner_area
        k = 1 / (r / self.alpha_two_phase + r * self.wall_thickness / self.wall_lambda + 1 / self.alpha_secondary)
        return k * self.A


def water_hx_for(Q_nominal: float) -> HXParams:
    """Brazed plate HX (stainless steel, 0.4 mm): ~0.21 m2 per kW, UA(two-phase) ~ 4.6 kW/K at 9.3 kW."""
    return HXParams(A=0.208 * Q_nominal / 1000, secondary_medium="water", ratio_outer_to_inner_area=1.0,
                    alpha_two_phase=4000, alpha_gas=800, alpha_liquid=1500, alpha_secondary=7000,
                    wall_lambda=15, wall_thickness=0.4e-3)


def air_hx_for(Q_nominal: float) -> HXParams:
    """Finned-tube coil (Cu tubes, Al fins): ~11 m2 finned area per kW, fin/tube area ratio 20,
    air-side alpha 55 W/m2K incl. fin efficiency -> UA(two-phase) ~ 4.1 kW/K at 9.3 kW."""
    return HXParams(A=11.0 * Q_nominal / 1000, secondary_medium="air", ratio_outer_to_inner_area=20.0,
                    alpha_two_phase=3000, alpha_gas=300, alpha_liquid=1000, alpha_secondary=55,
                    wall_lambda=390, wall_thickness=0.5e-3)


def _load_flowsheet(fluid: str, flowsheet: str, cooling: bool, V_h: float, Q_nominal: float):
    flowsheets = {
        "Standard": StandardCycle,
        "VaporInjectionPhaseSeparator": VaporInjectionPhaseSeparator,
        "VaporInjectionEconomizer": VaporInjectionEconomizer
    }
    N_max = 125
    compressor = BoundedRotaryCompressor(
        N_max=N_max,
        V_h=V_h
    )
    # Physical heat exchangers, sized with the nominal capacity. A 4-way reversing valve swaps
    # which physical HX rejects vs. absorbs heat, but does NOT change which vclibpy class
    # (MovingBoundaryNTUCondenser/-Evaporator) a role must be, since their `calc()` methods
    # hardcode the Inputs fields (T_con_in/m_flow_con vs. T_eva_in/m_flow_eva) and phase order
    # they read. So we swap the physical parameter sets between the fixed classes.
    water_hx, air_hx = water_hx_for(Q_nominal), air_hx_for(Q_nominal)
    if cooling:
        # Cooling: air side rejects heat (condenser), water side absorbs heat (evaporator)
        con_hx, eva_hx = air_hx, water_hx
    else:
        # Heating: water side rejects heat (condenser), air side absorbs heat (evaporator)
        con_hx, eva_hx = water_hx, air_hx

    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(**con_hx.kwargs())
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(**eva_hx.kwargs())
    economizer = VaporInjectionEconomizerNTU(
        A=2,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=50000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50000),
    )
    expansion_valve = Bernoulli(A=0.1)  # Should not matter

    kwargs = dict(
        evaporator=evaporator,
        condenser=condenser,
        fluid=fluid
    )
    if flowsheet == "Standard":
        kwargs.update(dict(
            expansion_valve=expansion_valve,
            compressor=compressor
        ))
    else:
        kwargs.update(dict(
            high_pressure_compressor=compressor,
            low_pressure_compressor=compressor,
            high_pressure_valve=expansion_valve,
            low_pressure_valve=expansion_valve
        ))
    if flowsheet == "VaporInjectionEconomizer":
        kwargs.update(dict(
            economizer=economizer
        ))
    return flowsheets[flowsheet](**kwargs)


@dataclasses.dataclass
class HeatPumpDesignSpec:
    """
    Design specification for one reversible heat pump (single compressor,
    one water-side and one air-side heat exchanger, roles swapped by mode)
    from which both a heating and a cooling characteristic map are generated.

    Only `design_mode` sizes the compressor exactly: a single fixed
    compressor cannot be forced to hit two independent, arbitrary capacity
    targets at once. The other mode's capacity at its own design point is
    therefore an emergent result of the sized machine, not an input.
    """
    fluid: str
    flowsheet: str
    design_mode: Literal["heating", "cooling"]

    # Design point of the mode that sizes the compressor:
    Q_nominal: float  # W
    dT_con: float  # K, secondary-side temperature rise across the condenser role
    dT_eva: float  # K, secondary-side temperature drop across the evaporator role
    T_con_in_design: float  # K
    T_eva_in_design: float  # K

    # Design point of the other mode (used for the map + reporting only, no capacity target):
    dT_con_other: float  # K
    dT_eva_other: float  # K
    T_con_in_design_other: float  # K
    T_eva_in_design_other: float  # K

    n_design: float = 1.0
    dT_eva_superheating: float = 5
    dT_con_subcooling: float = 0

    T_eva_in_ar_heating: Optional[np.ndarray] = None
    T_con_in_ar_heating: Optional[np.ndarray] = None
    T_eva_in_ar_cooling: Optional[np.ndarray] = None
    T_con_in_ar_cooling: Optional[np.ndarray] = None
    n_ar: Optional[np.ndarray] = None

    def __post_init__(self):
        if self.n_ar is None:
            self.n_ar = np.arange(0.3, 1.01, 0.1)
        if self.T_eva_in_ar_heating is None:
            self.T_eva_in_ar_heating = np.arange(-20, 20.1, 2.5) + 273.15
        if self.T_con_in_ar_heating is None:
            self.T_con_in_ar_heating = np.arange(20, 70.1, 2.5) + 273.15
        if self.T_eva_in_ar_cooling is None:
            self.T_eva_in_ar_cooling = np.arange(5, 22.6, 2.5) + 273.15
        if self.T_con_in_ar_cooling is None:
            self.T_con_in_ar_cooling = np.arange(15, 45.1, 2.5) + 273.15


def _nominal_design_point(
        fluid: str, flowsheet: str, cooling: bool, V_h: float, Q_nominal: float,
        dT_con: float, dT_eva: float,
        T_con_in_design: float, T_eva_in_design: float, n_design: float,
        dT_eva_superheating: float, dT_con_subcooling: float,
) -> dict:
    """Build a fresh flowsheet with the given V_h and converge the secondary
    mass flows to hit dT_con/dT_eva at the given design point."""
    heat_pump = _load_flowsheet(fluid=fluid, flowsheet=flowsheet, cooling=cooling, V_h=V_h, Q_nominal=Q_nominal)
    inputs = Inputs(
        T_con_in=T_con_in_design,
        T_eva_in=T_eva_in_design,
        n=n_design,
        dT_eva_superheating=dT_eva_superheating,
        dT_con_subcooling=dT_con_subcooling,
    )
    nominal_design = nominal_hp_design(
        heat_pump=heat_pump,
        fluid=fluid,
        inputs=inputs,
        dT_con=dT_con,
        dT_eva=dT_eva,
    )
    heat_pump.terminate()
    return nominal_design


def _size_compressor_for_capacity(
        fluid: str, flowsheet: str, cooling: bool,
        dT_con: float, dT_eva: float,
        T_con_in_design: float, T_eva_in_design: float,
        Q_target: float, n_design: float,
        dT_eva_superheating: float, dT_con_subcooling: float,
        V_h_start: float = 19e-6, tol: float = 0.005, max_iter: int = 30,
) -> dict:
    """
    Solve the compressor displacement volume V_h so the resulting capacity
    (Q_con for heating, Q_eva_outer for cooling) at the given design point
    matches Q_target.

    Refrigerant mass flow (and thus capacity) is ~monotonic in V_h, since the
    compressor's volumetric/isentropic/mechanical efficiencies only depend on
    pressure ratio and absolute speed, not on V_h (see RotaryCompressor). A
    secant search therefore converges in a handful of iterations.
    """
    Q_key = "Q_eva_outer" if cooling else "Q_con"

    def _capacity_for(V_h):
        design = _nominal_design_point(
            fluid=fluid, flowsheet=flowsheet, cooling=cooling, V_h=V_h, Q_nominal=Q_target,
            dT_con=dT_con, dT_eva=dT_eva,
            T_con_in_design=T_con_in_design, T_eva_in_design=T_eva_in_design,
            n_design=n_design, dT_eva_superheating=dT_eva_superheating,
            dT_con_subcooling=dT_con_subcooling,
        )
        return design, design[Q_key]

    V_h_0 = V_h_start
    design_0, Q_0 = _capacity_for(V_h_0)
    if abs(Q_0 - Q_target) / abs(Q_target) < tol:
        return {**design_0, "V_h": V_h_0}

    # Capacity scales ~linearly with V_h, so this is already a good 2nd guess:
    V_h_1 = V_h_0 * Q_target / Q_0
    design_1, Q_1 = _capacity_for(V_h_1)

    for _ in range(max_iter):
        if abs(Q_1 - Q_target) / abs(Q_target) < tol:
            return {**design_1, "V_h": V_h_1}
        slope = (Q_1 - Q_0) / (V_h_1 - V_h_0)
        V_h_next = V_h_1 + (Q_target - Q_1) / slope
        V_h_0, Q_0 = V_h_1, Q_1
        design_1, Q_1 = _capacity_for(V_h_next)
        V_h_1 = V_h_next

    raise RuntimeError(
        f"Compressor sizing did not converge within {max_iter} iterations "
        f"(target={Q_target:.1f} W, last achieved={Q_1:.1f} W)."
    )


def design_heat_pump(spec: HeatPumpDesignSpec) -> dict:
    """
    Size the compressor from spec.design_mode's nominal capacity, then derive
    the other mode's secondary mass flows (same physical machine, no capacity
    target) at its own design point.

    Returns a dict with keys "heating", "cooling" (each the nominal-design
    dict from `nominal_hp_design`, with "V_h" added) and "V_h".
    """
    driver_cooling = spec.design_mode == "cooling"
    driver_design = _size_compressor_for_capacity(
        fluid=spec.fluid, flowsheet=spec.flowsheet, cooling=driver_cooling,
        dT_con=spec.dT_con, dT_eva=spec.dT_eva,
        T_con_in_design=spec.T_con_in_design, T_eva_in_design=spec.T_eva_in_design,
        Q_target=spec.Q_nominal, n_design=spec.n_design,
        dT_eva_superheating=spec.dT_eva_superheating, dT_con_subcooling=spec.dT_con_subcooling,
    )
    V_h = driver_design["V_h"]
    logger.info(
        "Heat exchangers sized for Q_nominal=%.0f W: UA_two_phase water=%.0f W/K, air=%.0f W/K",
        spec.Q_nominal, water_hx_for(spec.Q_nominal).UA_two_phase(), air_hx_for(spec.Q_nominal).UA_two_phase(),
    )

    other_mode = "cooling" if not driver_cooling else "heating"
    other_design = _nominal_design_point(
        fluid=spec.fluid, flowsheet=spec.flowsheet, cooling=not driver_cooling, V_h=V_h,
        Q_nominal=spec.Q_nominal,
        dT_con=spec.dT_con_other, dT_eva=spec.dT_eva_other,
        T_con_in_design=spec.T_con_in_design_other, T_eva_in_design=spec.T_eva_in_design_other,
        n_design=spec.n_design, dT_eva_superheating=spec.dT_eva_superheating,
        dT_con_subcooling=spec.dT_con_subcooling,
    )
    other_design["V_h"] = V_h

    results = {
        spec.design_mode: driver_design,
        other_mode: other_design,
        "V_h": V_h,
    }
    driver_Q = driver_design["Q_eva_outer" if driver_cooling else "Q_con"]
    other_Q = other_design["Q_eva_outer" if not driver_cooling else "Q_con"]
    logger.info(
        "Sized compressor (V_h=%.3e m3) on %s target %.1f W (achieved %.1f W). "
        "Emergent %s capacity at its design point: %.1f W",
        V_h, spec.design_mode, spec.Q_nominal, driver_Q, other_mode, other_Q,
    )
    return results


def generate_maps(spec: HeatPumpDesignSpec, design_results: dict, working_dir: pathlib.Path) -> dict:
    """Generate both the heating and the cooling characteristic map for the sized heat pump."""
    working_dir = pathlib.Path(working_dir)
    results = {}
    for mode, cooling, T_eva_in_ar, T_con_in_ar in (
            ("heating", False, spec.T_eva_in_ar_heating, spec.T_con_in_ar_heating),
            ("cooling", True, spec.T_eva_in_ar_cooling, spec.T_con_in_ar_cooling),
    ):
        mode_working_dir = working_dir.joinpath("Cooling" if cooling else "Heating")
        os.makedirs(mode_working_dir, exist_ok=True)
        heat_pump = _load_flowsheet(
            fluid=spec.fluid, flowsheet=spec.flowsheet, cooling=cooling, V_h=design_results["V_h"],
            Q_nominal=spec.Q_nominal,
        )
        nominal_design = design_results[mode]
        results[mode] = utils.full_factorial_map_generation(
            heat_pump=heat_pump,
            save_path=mode_working_dir,
            T_con_in_ar=T_con_in_ar,
            T_eva_in_ar=T_eva_in_ar,
            n_ar=spec.n_ar,
            use_multiprocessing=False,
            save_plots=False,
            m_flow_con=nominal_design["m_flow_con"],
            m_flow_eva=nominal_design["m_flow_eva"],
            dT_eva_superheating=spec.dT_eva_superheating,
            dT_con_subcooling=spec.dT_con_subcooling,
            filename_suffix=f"_{mode}",
        )
    return results


def create_model(fluid, flowsheet, mode):
    model_name = f"VCLib{flowsheet}{fluid}{mode}"
    description = f"Map based on VCLib with {flowsheet} and {fluid} ({mode})"
    filename = f"modelica://AixLib/Resources/Data/Fluid/HeatPumps/ModularReversible/Data/VCLibMap/{mode}/{flowsheet}_{fluid}.sdf"

    model = f"""
        record {model_name}
          "{description}"
          extends Generic(
            refrigerant="{fluid}",
            flowsheet="{flowsheet}",
            filename="{filename}");
        end {model_name};
    """
    return model.strip()


def print_package(fluids, flowsheets):
    # Generate models for each combination
    models = []
    for fluid in FLUIDS:
        for flowsheet in FLOWSHEETS:
            for mode in ("Heating", "Cooling"):
                models.append(create_model(fluid, flowsheet, mode))

    # Print all generated models
    for model in models:
        print(model)
        print()  # Add an empty line between models for readability



if __name__ == "__main__":
    import logging
    logging.basicConfig(level="INFO")
    from vclibpy.media import set_global_media_properties
    from vclibpy.media import CoolProp
    set_global_media_properties(CoolProp)

    FLUIDS = [
        # "R410A",
        # "R152a",
        # "R134a",
        "Propane"
    ]
    FLOWSHEETS = [
        "Standard",
        # "VaporInjectionPhaseSeparator",
    ]
    print_package(FLUIDS, FLOWSHEETS)

    for FLUID in FLUIDS:
        for FLOWSHEET in FLOWSHEETS:
            # Example: size the compressor on the heating design point (a building
            # with a heating-dominated load). Switch design_mode to "cooling" and
            # swap which design point carries Q_nominal to size off cooling instead.
            spec = HeatPumpDesignSpec(
                fluid=FLUID,
                flowsheet=FLOWSHEET,
                design_mode="heating",
                Q_nominal=5000,  # W, nominal heating capacity at the heating design point
                dT_con=5,
                dT_eva=5,
                T_con_in_design=273.15 + 35,
                T_eva_in_design=273.15 - 3.2932,
                dT_con_other=5,
                dT_eva_other=3,
                T_con_in_design_other=273.15 + 35.2,
                T_eva_in_design_other=273.15 + 18,
            )
            design_results = design_heat_pump(spec)
            generate_maps(
                spec=spec,
                design_results=design_results,
                working_dir=pathlib.Path(r"D:\00_Temp\Testing\vclibpy\AixLibVCLibCreation"),
                # working_dir=pathlib.Path(r"T:\rha-jre\AixLibVCLibCreation"),
            )
