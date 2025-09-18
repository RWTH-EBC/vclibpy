# segmented_evaporator_external.py
from __future__ import annotations

import math
import warnings
from typing import Tuple, List

from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers import ExternalHeatExchanger

# Your segmented model:
from segmented_evaporator import SegmentedEvaporator

class SegmentedEvaporatorExternal(ExternalHeatExchanger):
    """
    Drop-in evaporator component for VcLibPy flowsheets that internally uses
    a segmented evaporator (marching in x) instead of the NTU method.

    External interface (unchanged):
      - calc(inputs, fs_state) -> (error_percent, dT_min)

    Internal model:
      - Quasi-isothermal wall: T_w taken from secondary inlet (air) temperature.
      - Two-phase HTC: user-provided function (e.g., Gungor–Winterton).
      - Single-phase HTC: optional function (fallback inside SegmentedEvaporator if None).
    """

    def __init__(
        self,
        # --- arguments required by ExternalHeatExchanger for compatibility ---
        A: float,
        wall_heat_transfer,
        secondary_heat_transfer,
        gas_heat_transfer,
        liquid_heat_transfer,
        two_phase_heat_transfer,
        secondary_medium: str,
        ratio_outer_to_inner_area: float = 1.0,
        flow_type: str = "counter",
        # --- geometry/segmentation for the internal model ---
        d_h: float = 0.010,
        n_cells: int = 20,
        # --- HTC functions for the internal model ---
        two_phase_htc_func=None,     # h(x, q_flux, p, m_flow, med_prop, d_h, orientation) -> float
        single_phase_htc_func=None,  # h(T, p, m_flow, med_prop, d_h) -> float
        orientation: str = "horizontal",
    ):
        super().__init__(
            A=A,
            wall_heat_transfer=wall_heat_transfer,
            secondary_heat_transfer=secondary_heat_transfer,
            gas_heat_transfer=gas_heat_transfer,
            liquid_heat_transfer=liquid_heat_transfer,
            two_phase_heat_transfer=two_phase_heat_transfer,
            secondary_medium=secondary_medium,
            ratio_outer_to_inner_area=ratio_outer_to_inner_area,
            flow_type=flow_type,
        )

        if A <= 0.0:
            raise ValueError("A must be > 0.")
        if d_h <= 0.0:
            raise ValueError("d_h must be > 0.")
        if n_cells < 1:
            raise ValueError("n_cells must be >= 1")

        self._A_total = float(A)
        self._d_h = float(d_h)
        self._n_cells = int(n_cells)
        self._orientation = orientation

        if two_phase_htc_func is None:
            raise ValueError("two_phase_htc_func must be provided.")
        self._two_phase_htc = two_phase_htc_func
        self._single_phase_htc = single_phase_htc_func

        # Internal segmented model; T_w will be set per calc()
        self._seg = SegmentedEvaporator(
            n_cells=self._n_cells,
            A_total=self._A_total,
            d_h=self._d_h,
            T_w=273.15,  # placeholder; overwritten in calc()
            h_two_phase=self._two_phase_htc,
            orientation=self._orientation,
            h_single_phase=self._single_phase_htc,
        )

    @staticmethod
    def _q_from_enthalpy(m_flow: float, h_in: float, h_out: float) -> float:
        """Primary-side heat transfer from enthalpy change [W]."""
        return m_flow * (h_out - h_in)

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> Tuple[float, float]:
        """
        One evaporator calculation step compatible with VcLibPy flowsheets.

        Returns
        -------
        error_percent : relative deviation between modeled Q and required Q (=m_flow*Δh) in percent
        dT_min        : pinch-like minimal driving temperature difference [K]
        """
        if any(v is None for v in [self.m_flow, self.med_prop, self.state_inlet, self.state_outlet]):
            raise RuntimeError("Component not initialized by flowsheet (m_flow/med_prop/states missing).")

        # Required heat from current refrigerant states (as NTU classes do)
        Q_required = self.calc_Q_flow()  # = m_flow * abs(h_in - h_out)

        # Ensure secondary cp is initialized (needed by get_all_inputs)
        try:
            self.start_secondary_med_prop()
        except Exception:
            pass
        try:
            # first guess: cp at secondary inlet temperature
            self.calc_secondary_cp(T=float(inputs.evaporator.T_in))
        except Exception:
            # fallback: use refrigerant outlet temp to at least set a cp
            self.calc_secondary_cp(T=self.state_outlet.T)

        # Secondary side inputs (we only use these for dT_min and consistency with framework)
        T_in_sec, T_out_sec, dT_sec, m_flow_sec = inputs.evaporator.get_all_inputs(
            cp=self.cp_secondary,
            Q=Q_required
        )
        self.m_flow_secondary = m_flow_sec  # keep framework consistency

        # Quasi-isothermal wall temperature (air side)
        try:
            T_wall = float(inputs.evaporator.T_in)
        except Exception:
            T_wall = float(T_in_sec)

        # Run internal segmented model for current step
        p = self.state_outlet.p  # consistent with MovingBoundaryNTUEvaporator
        h_in = self.state_inlet.h
        self._seg.T_w = T_wall

        seg_results = self._seg.simulate(
            p=p,
            h_in=h_in,
            m_flow=self.m_flow,
            med_prop=self.med_prop,
        )

        Q_model = sum(r.Q_segment for r in seg_results)
        h_out_model = seg_results[-1].h_outlet

        # Optional: expose details to flowsheet state
        if hasattr(fs_state, "set"):
            fs_state.set("eva_seg_total_Q_W", Q_model, "W", "Sum of segment heat rates (segmented model)")
            fs_state.set("eva_seg_h_out_model", h_out_model, "J/kg", "Outlet enthalpy predicted by segmented model")
            fs_state.set("eva_seg_x_out_model", seg_results[-1].x_outlet, "-", "Outlet vapor quality (segmented model)")

        # Error definition consistent with NTU classes: (Q_model/Q_required - 1)*100
        if abs(Q_required) > 1e-9:
            error_percent = (Q_model / Q_required - 1.0) * 100.0
        else:
            error_percent = 0.0

        # Pinch proxy like NTU evaporator:
        dT_min_in = T_in_sec - self.state_outlet.T
        dT_min_out = T_out_sec - self.state_inlet.T
        dT_min = min(dT_min_in, dT_min_out)

        return error_percent, dT_min