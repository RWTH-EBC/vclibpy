"""
Generate heating AND cooling characteristic maps (vclibpy -> .sdf/.csv for AixLib)
for one reversible air-to-water heat pump with physically plausible parameters.

Changes compared to heat_pump_3D_heating.py
-------------------------------------------
1. Heat exchangers: separate, realistic parameter sets for the brazed plate HX
   (water side) and the finned-tube coil (air side). Previously every alpha was
   5000 W/m2K, including the air side, so the air coil had UA ~ 37 kW/K and the
   plate HX ~ 12 kW/K (realistic for ~9 kW: 3-5 kW/K each). Areas now scale with Q_nominal.
2. Secondary mass flows are machine properties: one pump (fixed m_flow_water) and
   one fan (V_flow_air; optionally a different flow in cooling, V_flow_air_cooling_m3h,
   e.g. from the fan speed). Previously the cooling design used dT_air = 3 K, i.e. ~3.5x
   the air flow of the heating mode, which inflated the cooling EER.
3. Compressor: plain `RotaryCompressor` (Engelpracht regressions) as default. Estimated fit
   range of those regressions (from the normalisation constants of lambda_h, not documented):
   N ~ 28-100 Hz, pi ~ 1.7-7.4, T_1 ~ -24...+5 degC -> cooling is always an extrapolation.
   `CalibratedRotaryCompressor` keeps eta_is and lambda_h and only scales eta_mech and adds a
   constant auxiliary power per mode (EN 14511 COP/EER include fan/pump/control shares).
   `BoundedRotaryCompressor` (own eta_is/eta_mech curves) is still available via spec.compressor_cls.
4. No `nominal_hp_design` anymore (m_flows are fixed): V_h is sized directly
   by a secant search on the heating capacity at the design point.
5. Vitocal 251-A10 (`vitocal_251_a10_spec`, `calibrate`): V_h is sized so that all EN 14511 rating
   points of the datasheet are reachable with n <= 1, then eta_mech_factor, P_aux_heating and
   P_aux_cooling are fitted to the rated electrical powers (bounded linear least squares).
6. Map post-processing (`enforce_min_pressure_ratio`): points with pi < pi_min (source warmer than
   sink -> p_con < p_eva, negative compressor work) and non-converged points take the values of the
   nearest valid point along T_con_in (fallback: along dT_water, then T_eva_in, for lines without any
   valid point, e.g. cooling 5 degC water inlet with dT = 10 K). The raw map is kept as *_raw.sdf;
   the .csv stays raw.
7. Optional 4D maps (spec.dT_water_ar, `generate_map_4d`): 4th axis = water-side temperature difference
   (heating: dT_con, cooling: dT_eva), axes (n, T_con_in, T_eva_in, dT) as expected by AixLib
   TableData4DDeltaTCon (outOrd={3,1,2,4}, use_TConOutForTab=false). The water flow is solved per point,
   m_flow_water = Q/(cp*dT); air flow, compressor and calibration are identical to the 3D maps.
"""
import dataclasses
import json
import logging
import os
import pathlib
from typing import Optional

import numpy as np
import pandas as pd
from vclibpy import Inputs
from vclibpy import utils
from vclibpy.components.compressors import RotaryCompressor
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.heat_exchangers import heat_transfer, moving_boundary_ntu
from vclibpy.flowsheets import StandardCycle

logger = logging.getLogger(__name__)

CP_WATER = 4184.0  # J/(kg K)
RHO_AIR = 1.2  # kg/m3


# ---------------------------------------------------------------------------
# Compressor
# ---------------------------------------------------------------------------
class BoundedRotaryCompressor(RotaryCompressor):
    """
    Rotary compressor with bounded, physically plausible efficiencies.

      eta_is   = eta_is_max  * (1 - k_pi*ln(pi/pi_opt)^2) * (1 - k_n*(n - n_opt)^2)
      eta_mech = eta_mech_max * (1 - k_n_mech*(n - n_opt)^2)   (motor + inverter + mechanics, effective)
      lambda_h = Engelpracht regression, evaluated with the speed clipped to its fit range

    n is the relative speed (0..1). All efficiencies are clipped to [min, max].
    Defaults are the values calibrated against the Viessmann datasheets.
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


class CalibratedRotaryCompressor(RotaryCompressor):
    """
    Plain Engelpracht rotary compressor (eta_is and lambda_h unchanged) with two calibration terms
    to match EN 14511 datasheet values:

      eta_mech = min(eta_mech_factor * eta_mech_Engelpracht, eta_mech_max)
      P_el     = m_flow * (h_2 - h_1) / eta_mech + P_aux

    P_aux is a constant auxiliary power (fans, pump share, controls) of the respective operating mode.
    EN 14511 COP/EER include these shares, therefore the map P_el includes P_aux as well.
    With eta_mech_factor = 1 and P_aux = 0 this is identical to RotaryCompressor.
    """

    def __init__(self, N_max, V_h, eta_mech_factor: float = 1.0, eta_mech_max: float = 0.95, P_aux: float = 0.0):
        super().__init__(N_max=N_max, V_h=V_h)
        self.eta_mech_factor, self.eta_mech_max, self.P_aux = eta_mech_factor, eta_mech_max, P_aux

    def get_eta_mech(self, inputs: Inputs) -> float:
        return min(self.eta_mech_factor * super().get_eta_mech(inputs), self.eta_mech_max)

    def calc_electrical_power(self, inputs: Inputs, fs_state) -> float:
        P_el = super().calc_electrical_power(inputs=inputs, fs_state=fs_state) + self.P_aux
        fs_state.set(name="P_aux", value=self.P_aux, unit="W", description="Auxiliary power (fans, pump, controls)")
        return P_el


# ---------------------------------------------------------------------------
# Heat exchangers
# ---------------------------------------------------------------------------
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


def _load_flowsheet(spec: "HeatPumpDesignSpec", cooling: bool, V_h: float, water_hx: HXParams,
                    air_hx: HXParams) -> StandardCycle:
    """The 4-way valve swaps which physical HX is condenser/evaporator. vclibpy's condenser/evaporator
    classes are role-bound, so the physical parameter sets are swapped between them."""
    compressor = spec.compressor_cls(N_max=spec.N_max, V_h=V_h, **spec.compressor_kwargs_for(cooling))
    con, eva = (air_hx, water_hx) if cooling else (water_hx, air_hx)
    return StandardCycle(
        evaporator=moving_boundary_ntu.MovingBoundaryNTUEvaporator(**eva.kwargs()),
        condenser=moving_boundary_ntu.MovingBoundaryNTUCondenser(**con.kwargs()),
        fluid=spec.fluid,
        expansion_valve=Bernoulli(A=0.1),  # does not matter, superheat is prescribed
        compressor=compressor,
    )


# ---------------------------------------------------------------------------
# Design
# ---------------------------------------------------------------------------
@dataclasses.dataclass
class HeatPumpDesignSpec:
    fluid: str = "Propane"
    Q_nominal: float = 9300  # W, size class: scales HX areas, air flow and (by default) the design capacity
    Q_design: Optional[float] = None  # W, heating capacity at the design point and n_design (default: Q_nominal)
    T_eva_in_design: float = 273.15 - 2  # K, outdoor air
    T_con_in_design: float = 273.15 + 30  # K, water return (-> ~35 degC flow)
    dT_water_design: float = 5  # K, sets the (fixed) pump mass flow
    m_flow_water_fixed: Optional[float] = None  # kg/s, overrides dT_water_design if given
    V_flow_air_per_kW: float = 4500 / 9.3  # m3/h per kW Q_nominal, fixed fan air flow in heating mode
    V_flow_air_cooling_m3h: Optional[float] = None  # m3/h, fan air flow in cooling mode (default: as heating)
    n_design: float = 1.0
    N_max: float = 125  # Hz
    dT_eva_superheating: float = 5
    dT_con_subcooling: float = 0
    compressor_cls: type = RotaryCompressor  # plain Engelpracht regressions (default)
    compressor_kwargs: dict = dataclasses.field(default_factory=dict)  # extra kwargs for compressor_cls
    P_aux_heating: float = 0.0  # W, only used by CalibratedRotaryCompressor
    P_aux_cooling: float = 0.0  # W, only used by CalibratedRotaryCompressor

    T_eva_in_ar_heating: Optional[np.ndarray] = None
    T_con_in_ar_heating: Optional[np.ndarray] = None
    T_eva_in_ar_cooling: Optional[np.ndarray] = None
    T_con_in_ar_cooling: Optional[np.ndarray] = None
    n_ar: Optional[np.ndarray] = None
    # Optional 4th map dimension: hydraulic temperature difference of the WATER heat exchanger in K
    # (heating: condenser, dT_con; cooling: evaporator, dT_eva). The water flow is solved per point,
    # m_flow_water = Q / (cp * dT). None -> only the 3D maps with the fixed pump flow are generated.
    dT_water_ar: Optional[np.ndarray] = None

    def __post_init__(self):
        if self.n_ar is None:
            self.n_ar = np.round(np.arange(0.3, 1.01, 0.1), 2)
        if self.T_eva_in_ar_heating is None:  # outdoor air
            self.T_eva_in_ar_heating = np.arange(-20, 35.1, 2.5) + 273.15
        if self.T_con_in_ar_heating is None:  # water return
            self.T_con_in_ar_heating = np.arange(25, 70.1, 2.5) + 273.15
        if self.T_eva_in_ar_cooling is None:  # chilled water return
            self.T_eva_in_ar_cooling = np.arange(5, 30.1, 2.5) + 273.15
        if self.T_con_in_ar_cooling is None:  # outdoor air
            self.T_con_in_ar_cooling = np.arange(15, 45.1, 2.5) + 273.15

    @property
    def m_flow_water(self) -> float:
        if self.m_flow_water_fixed is not None:
            return self.m_flow_water_fixed
        return self.Q_nominal / (self.dT_water_design * CP_WATER)

    @property
    def m_flow_air(self) -> float:
        return self.V_flow_air_per_kW * self.Q_nominal / 1000 / 3600 * RHO_AIR

    @property
    def m_flow_air_cooling(self) -> float:
        if self.V_flow_air_cooling_m3h is None:
            return self.m_flow_air
        return self.V_flow_air_cooling_m3h / 3600 * RHO_AIR

    def compressor_kwargs_for(self, cooling: bool) -> dict:
        kwargs = dict(self.compressor_kwargs)
        if issubclass(self.compressor_cls, CalibratedRotaryCompressor):
            kwargs["P_aux"] = self.P_aux_cooling if cooling else self.P_aux_heating
        return kwargs


# ---------------------------------------------------------------------------
# Vitocal 251-A10
# ---------------------------------------------------------------------------
# EN 14511 rating points, Viessmann datasheet Vitocal 250-A (6195458), p. 9/10, type 251.A10 (400 V):
# (mode, T_air [degC], T_water_out [degC] (heating: flow, cooling: chilled water outlet), Q [W], P_el [W])
VITOCAL_251_A10_RATED = (
    ("heating", 2, 35, 5800, 1310),
    ("heating", 7, 35, 7300, 1380),
    ("heating", -7, 35, 9700, 3070),
    ("heating", -7, 55, 9180, 4310),
    ("cooling", 35, 7, 3900, 1180),
    ("cooling", 35, 18, 6500, 1230),
)
# Capacity ranges ("Leistungsregelung") in W: (mode, T_air, T_water_out, Q_min, Q_max). Used to derive the
# compressor speed range of the real unit; not part of the fit.
VITOCAL_251_A10_RANGES = (
    ("heating", 2, 35, 2200, 11000),
    ("heating", 7, 35, 2600, 12000),
    ("cooling", 35, 7, 3900, 6400),
    ("cooling", 35, 18, 6500, 13000),
)


def vitocal_251_a10_spec(**kwargs) -> HeatPumpDesignSpec:
    """Viessmann Vitocal 250-A / 251-A10 (400 V), datasheet 6195458:
    - size class: max. heating capacity at A7/W35 12.0 kW (HX areas scale with it)
    - twin rolling piston compressor (same compressor as the 251-A13) -> plain Engelpracht RotaryCompressor,
      N_max = 100 Hz (upper end of the estimated Engelpracht fit range); V_h is sized in the calibration so
      that every EN 14511 rating point is reachable with n <= 1
    - air flow 4045 m3/h at A7/W35 (fan 430 1/min); cooling: fan at 550 1/min -> 4045 * 550/430 m3/h
    - water flow: EN 14511 rating A7/W35, 7.3 kW, 5 K spread -> 0.349 kg/s (1256 l/h), kept for all heating points
    - initial design point A-7/W35 9.7 kW at n = 1 (only the start value for the calibration)"""
    m_w = 7300 / (5 * CP_WATER)
    defaults = dict(Q_nominal=12000, Q_design=9700, T_eva_in_design=273.15 - 7,
                    T_con_in_design=273.15 + 35 - 9700 / (m_w * CP_WATER),
                    V_flow_air_per_kW=4045 / 12.0, V_flow_air_cooling_m3h=4045 * 550 / 430,
                    m_flow_water_fixed=m_w, N_max=100,
                    T_eva_in_ar_heating=np.arange(-20, 40.1, 2.5) + 273.15)
    return HeatPumpDesignSpec(**{**defaults, **kwargs})


def _run(heat_pump, spec: HeatPumpDesignSpec, n, T_eva_in, T_con_in, m_flow_con, m_flow_eva):
    inputs = Inputs(n=n, T_eva_in=T_eva_in, T_con_in=T_con_in, m_flow_con=m_flow_con, m_flow_eva=m_flow_eva,
                    dT_eva_superheating=spec.dT_eva_superheating, dT_con_subcooling=spec.dT_con_subcooling)
    return heat_pump.calc_steady_state(fluid=spec.fluid, inputs=inputs)


def design_heat_pump(spec: HeatPumpDesignSpec, V_h_start: float = 35e-6, tol: float = 0.002,
                     max_iter: int = 20) -> dict:
    """Size V_h so that Q_con(design point, n_design) = Q_design (heating), with fixed m_flows."""
    water_hx, air_hx = water_hx_for(spec.Q_nominal), air_hx_for(spec.Q_nominal)
    Q_target = spec.Q_design if spec.Q_design is not None else spec.Q_nominal

    def Q_for(V_h):
        hp = _load_flowsheet(spec, False, V_h, water_hx, air_hx)
        fs = _run(hp, spec, spec.n_design, spec.T_eva_in_design, spec.T_con_in_design,
                  spec.m_flow_water, spec.m_flow_air)
        if fs is None:
            raise ValueError("Design point infeasible")
        return fs.get("Q_con").value, fs

    V0 = V_h_start
    Q0, _ = Q_for(V0)
    V1 = V0 * Q_target / Q0
    for _ in range(max_iter):
        Q1, fs = Q_for(V1)
        if abs(Q1 / Q_target - 1) < tol:
            break
        V0, Q0, V1 = V1, Q1, V1 + (Q_target - Q1) * (V1 - V0) / (Q1 - Q0)
    else:
        raise RuntimeError("Compressor sizing did not converge")

    design = dict(
        V_h=V1, m_flow_water=spec.m_flow_water, m_flow_air=spec.m_flow_air,
        m_flow_air_cooling=spec.m_flow_air_cooling,
        V_flow_air_m3h=spec.m_flow_air / RHO_AIR * 3600,
        V_flow_air_cooling_m3h=spec.m_flow_air_cooling / RHO_AIR * 3600,
        UA_water_two_phase=water_hx.UA_two_phase(), UA_air_two_phase=air_hx.UA_two_phase(),
        water_hx=dataclasses.asdict(water_hx), air_hx=dataclasses.asdict(air_hx),
        Q_con_design=fs.get("Q_con").value, COP_design=fs.get("COP").value,
    )
    logger.info("Sized V_h = %.2f cm3 -> Q_con = %.0f W, COP = %.2f at design point",
                V1 * 1e6, design["Q_con_design"], design["COP_design"])
    return design


def enforce_min_pressure_ratio(sdf_path: pathlib.Path, pi_min: float = 1.5) -> dict:
    """Post-process a vclibpy map (.sdf) for AixLib.

    Where the source is warmer than the sink (heating: warm air / cold water, cooling: warm water / cold air)
    the steady-state solver returns p_con < p_eva, i.e. negative compressor work and meaningless COP/EER.
    A real unit keeps a minimum pressure ratio there (fan / EEV control), so every point with
    pi < pi_min, and every non-converged point (NaN), takes all values of the nearest valid point
    along the T_con_in axis (same n and T_eva_in). The raw map is kept as *_raw.sdf."""
    import shutil
    import sdf
    sdf_path = pathlib.Path(sdf_path)
    raw_path = sdf_path.with_name(sdf_path.stem + "_raw.sdf")
    shutil.copyfile(sdf_path, raw_path)
    root = sdf.load(str(raw_path))
    fluid_group = root.groups[0].groups[0]
    ds = {d.name: d for d in fluid_group.datasets}
    pi = ds["p_con"].data / ds["p_eva"].data  # shape (n, T_con_in, T_eva_in[, dT_water])
    nd = [d for d in ds.values() if np.ndim(d.data) == pi.ndim]
    invalid = ~(pi >= pi_min)
    if "dT_water_converged" in ds:  # 4D maps: points where the water-side dT could not be reached
        invalid |= ~(ds["dT_water_converged"].data == 1)
    stats = dict(n_low_pi=int(np.sum(pi < pi_min)), n_nan=int(np.isnan(pi).sum()), n_fallback=0, n_unfixable=0)

    def fill_along(axis: int) -> int:
        """Fill invalid entries from the nearest valid entry along `axis`; returns the number filled."""
        filled = 0
        other = [s for i, s in enumerate(invalid.shape) if i != axis]
        for idx in np.ndindex(*other):
            sl = list(idx)
            sl.insert(axis, slice(None))
            sl = tuple(sl)
            bad = invalid[sl]
            valid = np.where(~bad)[0]
            if len(valid) == 0 or not bad.any():
                continue
            for i_b in np.where(bad)[0]:
                j = valid[np.argmin(np.abs(valid - i_b))]
                for d in nd:
                    v = d.data[sl]
                    v[i_b] = v[j]
            filled += int(bad.sum())
            invalid[sl] = False
        return filled

    fill_along(1)  # T_con_in: minimum pressure ratio held by fan/EEV control
    # Fallback for lines without any valid point, e.g. cooling with 5 degC water inlet and dT = 10 K
    # (outlet -5 degC, not feasible): nearest valid dT (4D only), then nearest valid T_eva_in.
    for axis in ([3] if pi.ndim == 4 else []) + [2]:
        stats["n_fallback"] += fill_along(axis)
    stats["n_unfixable"] = int(invalid.sum())
    sdf.save(str(sdf_path), root)
    logger.info("%s: replaced %d points with pi < %.2f and %d non-converged points "
                "(%d via fallback axes, %d left unfixed)", sdf_path, stats["n_low_pi"], pi_min, stats["n_nan"],
                stats["n_fallback"], stats["n_unfixable"])
    return stats


def _solve_dT_water_point(args):
    """Worker (top level for multiprocessing): steady state with the water flow solved for a given spread.
    Returns a FlowsheetState (empty if not converged) with the additional variables m_flow_water,
    dT_water (achieved spread) and dT_water_converged."""
    from vclibpy.datamodels import FlowsheetState
    hp, fluid, cooling, n, T_eva_in, T_con_in, m_flow_air, dT, m_start, sh, sc, tol, max_iter = args
    hx = hp.evaporator if cooling else hp.condenser
    m_w, fs, dT_act = m_start, None, np.nan
    try:
        for _ in range(max_iter):
            m_con, m_eva = (m_flow_air, m_w) if cooling else (m_w, m_flow_air)
            inputs = Inputs(n=n, T_eva_in=T_eva_in, T_con_in=T_con_in, m_flow_con=m_con, m_flow_eva=m_eva,
                            dT_eva_superheating=sh, dT_con_subcooling=sc)
            fs = hp.calc_steady_state(fluid=fluid, inputs=inputs)
            if fs is None:
                break
            Q = fs.get("Q_eva_outer" if cooling else "Q_con_outer").value
            cp = hx.m_flow_secondary_cp / hx.m_flow_secondary
            dT_act = Q / (m_w * cp)
            if abs(dT_act / dT - 1) < tol:
                break
            m_w = Q / (cp * dT)
    except Exception as e:  # keep the map generation alive, like vclibpy does
        logger.error("dT point failed (n=%s, T_eva_in=%s, T_con_in=%s, dT=%s): %s", n, T_eva_in, T_con_in, dT, e)
        fs = None
    if fs is not None:
        non_real = [k for k, v in fs.get_variables().items()
                    if isinstance(v.value, complex) or np.iscomplexobj(v.value)]
        if non_real:  # e.g. a regression evaluated far outside its range -> treat the point as failed
            logger.error("dT point discarded (n=%s, T_eva_in=%s, T_con_in=%s, dT=%s): non-real %s",
                         n, T_eva_in, T_con_in, dT, non_real)
            fs = None
    if fs is None:
        return FlowsheetState()
    fs.set(name="m_flow_water", value=m_w, unit="kg/s", description="Water mass flow rate (solved for dT)")
    fs.set(name="dT_water", value=dT_act, unit="K", description="Achieved water-side temperature difference")
    fs.set(name="dT_water_converged", value=float(abs(dT_act / dT - 1) < tol), unit="-",
           description="1 if the water-side dT matches the axis value")
    return fs


def generate_map_4d(spec: HeatPumpDesignSpec, design: dict, cooling: bool, save_dir: pathlib.Path,
                    use_multiprocessing: bool = False, tol: float = 0.005, max_iter: int = 10):
    """4D map over (n, T_con_in, T_eva_in, dT_water) in the AixLib order (outOrd={3,1,2,4}).
    Heating: 4th axis 'dT_con' (water condenser); cooling: 4th axis 'dT_eva' (water evaporator).
    Air flow, compressor and calibration are identical to the 3D maps; only the water flow varies."""
    import multiprocessing
    save_dir = pathlib.Path(save_dir)
    os.makedirs(save_dir, exist_ok=True)
    water_hx, air_hx = HXParams(**design["water_hx"]), HXParams(**design["air_hx"])
    hp = _load_flowsheet(spec, cooling, design["V_h"], water_hx, air_hx)
    T_con_in_ar = spec.T_con_in_ar_cooling if cooling else spec.T_con_in_ar_heating
    T_eva_in_ar = spec.T_eva_in_ar_cooling if cooling else spec.T_eva_in_ar_heating
    n_ar, dT_ar = np.asarray(spec.n_ar), np.asarray(spec.dT_water_ar, dtype=float)
    m_air = design["m_flow_air_cooling"] if cooling else design["m_flow_air"]
    dT_name = "dT_eva" if cooling else "dT_con"

    idx, tasks = [], []
    for i_n, n in enumerate(n_ar):
        for i_c, T_c in enumerate(T_con_in_ar):
            for i_e, T_e in enumerate(T_eva_in_ar):
                for i_d, dT in enumerate(dT_ar):
                    idx.append((i_n, i_c, i_e, i_d))
                    tasks.append((hp, spec.fluid, cooling, float(n), float(T_e), float(T_c), m_air, float(dT),
                                  design["m_flow_water"], spec.dT_eva_superheating, spec.dT_con_subcooling,
                                  tol, max_iter))
    logger.info("4D %s map: %d points", "cooling" if cooling else "heating", len(tasks))
    if use_multiprocessing:
        with multiprocessing.Pool(processes=multiprocessing.cpu_count()) as pool:
            states = list(pool.imap(_solve_dT_water_point, tasks, chunksize=8))
    else:
        states = [_solve_dT_water_point(t) for t in tasks]

    shape = (len(n_ar), len(T_con_in_ar), len(T_eva_in_ar), len(dT_ar))
    nd_data, info, rows = {}, {}, []
    for fs, (i_n, i_c, i_e, i_d), t in zip(states, idx, tasks):
        rows.append({"n": t[3], "T_eva_in": t[4], "T_con_in": t[5], dT_name: t[7],
                     **{k: v.value for k, v in fs.get_variables().items()}})
        for name, var in fs.get_variables().items():
            if name not in nd_data:
                nd_data[name] = np.full(shape, np.nan)
                info[name] = var
            try:
                nd_data[name][i_n, i_c, i_e, i_d] = var.value
            except (TypeError, ValueError):  # non-scalar / non-real value -> leave NaN (post-processing fills it)
                pass
    name = f"{hp.flowsheet_name}_{spec.fluid}_4D_{dT_name}"
    pd.DataFrame(rows).to_csv(save_dir.joinpath(name + ".csv"))
    scales = {"n": dict(data=n_ar, unit="-", comment="Relative compressor speed"),
              "T_con_in": dict(data=T_con_in_ar, unit="K", comment="Secondary side condenser inlet temperature"),
              "T_eva_in": dict(data=T_eva_in_ar, unit="K", comment="Secondary side evaporator inlet temperature"),
              dT_name: dict(data=dT_ar, unit="K", comment="Water-side temperature difference")}
    nd = {k: dict(data=v, unit=info[k].unit, comment=info[k].description) for k, v in nd_data.items()}
    params = {("m_flow_con" if cooling else "m_flow_eva"): dict(data=m_air, unit="kg/s", comment="Air mass flow rate"),
              "dT_eva_superheating": dict(data=spec.dT_eva_superheating, unit="K", comment="Superheating"),
              "dT_con_subcooling": dict(data=spec.dT_con_subcooling, unit="K", comment="Subcooling")}
    sdf_path = save_dir.joinpath(name + ".sdf")
    utils.save_to_sdf(data={hp.flowsheet_name: {spec.fluid: (scales, nd, params)}}, save_path=sdf_path)
    conv = nd_data.get("dT_water_converged", np.zeros(shape))
    logger.info("%s: %d of %d points solved, %d with dT not reached", sdf_path, int(np.sum(~np.isnan(conv))),
                conv.size, int(np.sum(conv == 0)))
    return sdf_path, save_dir.joinpath(name + ".csv")


def generate_maps(spec: HeatPumpDesignSpec, design: dict, working_dir: pathlib.Path,
                  use_multiprocessing: bool = False, modes=("heating", "cooling"),
                  pi_min: Optional[float] = 1.5) -> dict:
    """Heating and cooling maps of the same machine: same V_h, same HX, same fan and pump flows."""
    working_dir = pathlib.Path(working_dir)
    water_hx, air_hx = HXParams(**design["water_hx"]), HXParams(**design["air_hx"])
    results = {}
    for mode, cooling, T_eva_in_ar, T_con_in_ar in (
            ("heating", False, spec.T_eva_in_ar_heating, spec.T_con_in_ar_heating),
            ("cooling", True, spec.T_eva_in_ar_cooling, spec.T_con_in_ar_cooling),
    ):
        if mode not in modes:
            continue
        mode_dir = working_dir.joinpath("Cooling" if cooling else "Heating")
        os.makedirs(mode_dir, exist_ok=True)
        hp = _load_flowsheet(spec, cooling, design["V_h"], water_hx, air_hx)
        # heating: condenser = water, evaporator = air; cooling: condenser = air, evaporator = water
        m_con, m_eva = ((design["m_flow_air_cooling"], design["m_flow_water"]) if cooling
                        else (design["m_flow_water"], design["m_flow_air"]))
        try:
            results[mode] = utils.full_factorial_map_generation(
                heat_pump=hp, save_path=mode_dir,
                T_con_in_ar=T_con_in_ar, T_eva_in_ar=T_eva_in_ar, n_ar=spec.n_ar,
                m_flow_con=m_con, m_flow_eva=m_eva,
                dT_eva_superheating=spec.dT_eva_superheating, dT_con_subcooling=spec.dT_con_subcooling,
                use_multiprocessing=use_multiprocessing, save_plots=False,
            )
        except AttributeError:
            # vclibpy 0.1.2: with multiprocessing, heat_pump.terminate() fails after the files are written
            # because the parent process never initialised med_prop.
            name = f"{hp.flowsheet_name}_{spec.fluid}"
            results[mode] = (mode_dir.joinpath(name + ".sdf"), mode_dir.joinpath(name + ".csv"))
        if pi_min is not None:
            design.setdefault("map_postprocessing", {})[mode] = dict(
                pi_min=pi_min, **enforce_min_pressure_ratio(results[mode][0], pi_min))
        if spec.dT_water_ar is not None:
            results[mode + "_4D"] = generate_map_4d(spec, design, cooling, mode_dir, use_multiprocessing)
            if pi_min is not None:
                design.setdefault("map_postprocessing", {})[mode + "_4D"] = dict(
                    pi_min=pi_min, **enforce_min_pressure_ratio(results[mode + "_4D"][0], pi_min))
    with open(working_dir.joinpath("design.json"), "w") as f:
        json.dump({**design, "spec": {k: (v.tolist() if isinstance(v, np.ndarray) else
                                          v.__name__ if isinstance(v, type) else v)
                                      for k, v in dataclasses.asdict(spec).items()}}, f, indent=2)
    return results


# ---------------------------------------------------------------------------
# Calibration against EN 14511 datasheet points
# ---------------------------------------------------------------------------
def solve_rated_point(spec: HeatPumpDesignSpec, design: dict, mode: str, T_air: float, T_water_out: float,
                      Q_target: float, tol: float = 0.002, max_iter: int = 20):
    """Solve the relative speed n at which the useful capacity equals Q_target under EN 14511 conditions.

    Heating: fixed pump flow (set at the A7/W35 rating), return temperature = flow - Q/(m*cp).
    Cooling: 5 K chilled-water spread at the rated capacity (12/7, 23/18), fan flow of the cooling mode.
    Temperatures in degC. Returns (n, fs_state); n may exceed 1 if the point is not reachable."""
    cooling = mode == "cooling"
    hp = _load_flowsheet(spec, cooling, design["V_h"], HXParams(**design["water_hx"]), HXParams(**design["air_hx"]))
    if cooling:
        m_w = Q_target / (5 * CP_WATER)
        T_eva_in, T_con_in, m_con, m_eva = T_water_out + 5 + 273.15, T_air + 273.15, design["m_flow_air_cooling"], m_w
    else:
        m_w = design["m_flow_water"]
        T_eva_in, T_con_in = T_air + 273.15, T_water_out - Q_target / (m_w * CP_WATER) + 273.15
        m_con, m_eva = m_w, design["m_flow_air"]

    def Q_of(n):
        fs = _run(hp, spec, n, T_eva_in, T_con_in, m_con, m_eva)
        if fs is None:
            raise ValueError(f"{mode} A{T_air}/W{T_water_out}: infeasible at n = {n:.3f}")
        return fs.get("Q_eva_outer" if cooling else "Q_con").value, fs

    n0, n1 = 0.4, 0.8
    Q0, _ = Q_of(n0)
    for _ in range(max_iter):
        Q1, fs = Q_of(n1)
        if abs(Q1 / Q_target - 1) < tol:
            return n1, fs
        n0, Q0, n1 = n1, Q1, float(np.clip(n1 + (Q_target - Q1) * (n1 - n0) / (Q1 - Q0), 0.05, 2.0))
    raise RuntimeError(f"{mode} A{T_air}/W{T_water_out}: n for Q = {Q_target:.0f} W did not converge")


def calibrate(spec: HeatPumpDesignSpec, rated=VITOCAL_251_A10_RATED, ranges=VITOCAL_251_A10_RANGES,
              eta_mech_max: float = 0.95, P_aux_bounds=(0.0, 400.0), max_sizing_iter: int = 4):
    """Calibrate the machine against EN 14511 rating points (plain Engelpracht eta_is and lambda_h kept):

    1. V_h: scaled until the most demanding rated point needs exactly n = 1 (all points reachable).
    2. eta_mech_factor, P_aux_heating, P_aux_cooling: bounded linear least squares on the relative power
       error. This is exact, because eta_mech and P_aux only change P_el, not the cycle states:
           P_el,i = P_el,Engelpracht,i / eta_mech_factor + P_aux(mode_i)
    3. Speed range of the real unit from the datasheet capacity ranges (for the map grid / AixLib limits).

    Returns (calibrated spec, design, report dict)."""
    from scipy.optimize import lsq_linear

    plain = dataclasses.replace(spec, compressor_cls=RotaryCompressor, compressor_kwargs={})
    design = design_heat_pump(plain)
    for _ in range(max_sizing_iter):
        sols = [solve_rated_point(plain, design, *pt[:4]) for pt in rated]
        n_max = max(n for n, _ in sols)
        logger.info("V_h = %.2f cm3 -> max. required n over rated points = %.3f", design["V_h"] * 1e6, n_max)
        if abs(n_max - 1) < 0.01:
            break
        design["V_h"] *= n_max  # V_flow ~ V_h * n (lambda_h depends only weakly on N)
    else:
        sols = [solve_rated_point(plain, design, *pt[:4]) for pt in rated]

    P_engel = np.array([fs.get("P_el").value for _, fs in sols])
    eta_engel = np.array([fs.get("eta_mech").value for _, fs in sols])
    P_db = np.array([pt[4] for pt in rated], dtype=float)
    is_cool = np.array([pt[0] == "cooling" for pt in rated])
    # unknowns x = [1/eta_mech_factor, P_aux_heating, P_aux_cooling], rows scaled by 1/P_db (relative error)
    A = np.column_stack([P_engel, ~is_cool, is_cool]).astype(float) / P_db[:, None]
    a_min = eta_engel.max() / eta_mech_max  # keeps eta_mech <= eta_mech_max at all rated points
    res = lsq_linear(A, np.ones_like(P_db), bounds=([a_min, P_aux_bounds[0], P_aux_bounds[0]],
                                                    [np.inf, P_aux_bounds[1], P_aux_bounds[1]]))
    a, P_aux_h, P_aux_c = res.x
    cal_spec = dataclasses.replace(
        spec, compressor_cls=CalibratedRotaryCompressor,
        compressor_kwargs=dict(eta_mech_factor=1 / a, eta_mech_max=eta_mech_max),
        P_aux_heating=float(P_aux_h), P_aux_cooling=float(P_aux_c))

    points = []
    for pt, (n, fs), P_e, eta_e in zip(rated, sols, P_engel, eta_engel):
        mode, T_air, T_w, Q_db, P_d = pt
        P_cal = P_e * a + (P_aux_c if mode == "cooling" else P_aux_h)
        Q = Q_db  # capacity matched by construction
        points.append(dict(
            point=f"{mode} A{T_air}/W{T_w}", n=n, N=n * spec.N_max, Q_W=Q,
            P_datasheet_W=P_d, P_plain_W=float(P_e), P_calibrated_W=float(P_cal),
            COP_datasheet=Q_db / P_d, COP_plain=Q / P_e, COP_calibrated=Q / P_cal,
            dev_plain_pct=(P_d / P_e - 1) * 100, dev_calibrated_pct=(P_d / P_cal - 1) * 100,
            pi=fs.get("p_con").value / fs.get("p_eva").value, T_1_degC=fs.get("T_1").value - 273.15,
            T_2_degC=fs.get("T_2").value - 273.15, eta_is=fs.get("eta_is").value,
            eta_mech_plain=float(eta_e), eta_mech_calibrated=float(min(eta_e / a, eta_mech_max)),
            lambda_h=fs.get("lambda_h").value))

    speed_range = []
    for mode, T_air, T_w, Q_min, Q_max in ranges:
        n_lo, _ = solve_rated_point(plain, design, mode, T_air, T_w, Q_min)
        try:
            n_hi, _ = solve_rated_point(plain, design, mode, T_air, T_w, Q_max)
        except (ValueError, RuntimeError):
            n_hi = float("nan")
        speed_range.append(dict(point=f"{mode} A{T_air}/W{T_w}", Q_min_W=Q_min, Q_max_W=Q_max,
                                n_min=n_lo, n_max=n_hi))

    report = dict(V_h_cm3=design["V_h"] * 1e6, N_max_Hz=spec.N_max, eta_mech_factor=1 / a,
                  eta_mech_max=eta_mech_max, P_aux_heating_W=float(P_aux_h), P_aux_cooling_W=float(P_aux_c),
                  points=points, speed_range=speed_range)
    return cal_spec, design, report


def print_report(report: dict):
    print(f"\nV_h = {report['V_h_cm3']:.2f} cm3 at N_max = {report['N_max_Hz']:.0f} Hz | "
          f"eta_mech_factor = {report['eta_mech_factor']:.3f} (max {report['eta_mech_max']}) | "
          f"P_aux heating = {report['P_aux_heating_W']:.0f} W, cooling = {report['P_aux_cooling_W']:.0f} W")
    print(f"{'point':22s} {'n':>5s} {'N/Hz':>5s} {'COP_DB':>6s} {'plain':>6s} {'calib':>6s} {'dev_cal':>7s} "
          f"{'pi':>5s} {'T1/C':>5s} {'T2/C':>5s} {'eta_is':>6s} {'eta_m':>5s}")
    for p in report["points"]:
        print(f"{p['point']:22s} {p['n']:5.2f} {p['N']:5.0f} {p['COP_datasheet']:6.2f} {p['COP_plain']:6.2f} "
              f"{p['COP_calibrated']:6.2f} {p['dev_calibrated_pct']:+6.1f}% {p['pi']:5.2f} {p['T_1_degC']:5.1f} "
              f"{p['T_2_degC']:5.1f} {p['eta_is']:6.3f} {p['eta_mech_calibrated']:5.3f}")
    print("Speed range of the real unit (datasheet capacity range):")
    for s in report["speed_range"]:
        print(f"  {s['point']:22s} Q {s['Q_min_W']/1e3:4.1f}..{s['Q_max_W']/1e3:4.1f} kW -> "
              f"n {s['n_min']:.2f}..{s['n_max']:.2f}")


def create_model(fluid, flowsheet, mode):
    model_name = f"VCLib{flowsheet}{fluid}{mode}"
    description = f"Map based on VCLib with {flowsheet} and {fluid} ({mode})"
    filename = (f"modelica://AixLib/Resources/Data/Fluid/HeatPumps/ModularReversible/Data/VCLibMap/"
                f"{mode}/{flowsheet}_{fluid}.sdf")
    return f"""record {model_name}
  "{description}"
  extends Generic(
    refrigerant="{fluid}",
    flowsheet="{flowsheet}",
    filename="{filename}");
end {model_name};"""


def create_model_4d(fluid, flowsheet, mode):
    """AixLib record for the 4D heating map (TableData4DDeltaTCon, axes n, T_con_in, T_eva_in, dT_con).
    The temperature axis is the water INLET temperature -> use_TConOutForTab=false.
    For the 4D cooling map ('dT_eva' = water-side spread in cooling) AixLib has no matching chiller
    record; note that GenericVCLibPy reads Q_con_outer, while the useful cooling output is Q_eva_outer."""
    model_name = f"VCLib{flowsheet}{fluid}{mode}4D"
    dT_name = "dT_eva" if mode == "Cooling" else "dT_con"
    filename = (f"modelica://AixLib/Resources/Data/Fluid/HeatPumps/ModularReversible/Data/VCLibMap/"
                f"{mode}/{flowsheet}_{fluid}_4D_{dT_name}.sdf")
    return f"""record {model_name}
  "4D map based on VCLib with {flowsheet} and {fluid} ({mode}, water-side {dT_name})"
  extends AixLib.Fluid.HeatPumps.ModularReversible.Data.TableDataSDF.TableData4DDeltaTCon.VCLibPy.GenericVCLibPy4D(
    use_TConOutForTab=false,
    use_TEvaOutForTab=false,
    refrigerant="{fluid}",
    flowsheet="{flowsheet}",
    filename="{filename}");
end {model_name};"""


if __name__ == "__main__":
    logging.basicConfig(level="INFO")
    from vclibpy.media import set_global_media_properties, CoolProp
    set_global_media_properties(CoolProp)

    FLUID, FLOWSHEET = "Propane", "Standard"
    # Optional 4th dimension: water-side temperature difference in K (heating: condenser, cooling: evaporator).
    # None -> only 3D maps (fixed pump flow). With values, 4D maps are generated in addition.
    DT_WATER_AR = np.array([3.0, 5.0, 7.0, 10.0])
    for mode in ("Heating", "Cooling"):
        print(create_model(FLUID, FLOWSHEET, mode), end="\n\n")
        if DT_WATER_AR is not None:
            print(create_model_4d(FLUID, FLOWSHEET, mode), end="\n\n")

    WORKING_DIR = pathlib.Path(r"D:\00_Temp\Testing\vclibpy\Vitocal251A10")
    os.makedirs(WORKING_DIR, exist_ok=True)
    spec, design, report = calibrate(vitocal_251_a10_spec(fluid=FLUID, dT_water_ar=DT_WATER_AR))
    print_report(report)
    # map speed grid: from the lowest speed the real unit uses (rounded down to 0.1) up to n = 1
    n_min = min(s["n_min"] for s in report["speed_range"])
    spec.n_ar = np.round(np.arange(max(0.1, np.floor(n_min * 10) / 10), 1.001, 0.1), 2)
    design["calibration"] = report
    with open(WORKING_DIR.joinpath("calibration.json"), "w") as f:
        json.dump(report, f, indent=2)
    generate_maps(spec, design, working_dir=WORKING_DIR, use_multiprocessing=True)
