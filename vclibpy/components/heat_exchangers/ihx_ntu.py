import numpy as np
import logging

from vclibpy.components.heat_exchangers import InternalHeatExchanger, ntu
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer
from vclibpy import Inputs, FlowsheetState

logger = logging.getLogger(__name__)


class IHX_NTU(InternalHeatExchanger):
    """
    Logic for an internal heat exchanger in counter flow arrangement.
    The regime logics are depicted here: docs/source/ihx_logic.svg
    """

    def __init__(
            self,
            alpha_low_side: float,
            alpha_high_side: float,
            dT_pinch_min: float = 0,
            **kwargs):
        super().__init__(**kwargs)
        self.dT_pinch_min = dT_pinch_min
        assert self.flow_type == "counter", "Other types are not implemented"
        self.k = self.calc_k(
            alpha_pri=alpha_high_side,
            alpha_sec=alpha_low_side
        )

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        # First calculate the heat assuming q1 at inlet of lower side:
        state_low_q1 = self.med_prop.calc_state("PQ", self.state_outlet_low.p, 1)
        state_high_q0 = self.med_prop.calc_state("PQ", self.state_inlet_high.p, 0)

        dh_max_high = self.state_inlet_high.h - state_high_q0.h
        dh_max_low = self.state_outlet_low.h - state_low_q1.h
        if dh_max_low < 0:
            raise ValueError("This heat exchanger only allows low-pressure outlet states with superheat")

        # In case dh_max_high < 0, high side is already in subcooling,
        # no need for first regime in Option 1 (see svg)
        Q_high_tp_to_q0 = self.m_flow_high * max(0, dh_max_high)
        Q_low_sh_to_q1 = self.m_flow_low * dh_max_low

        # First part of the HX:
        m_flow_primary_cp = (
                (self.state_outlet_low.h - state_low_q1.h) /
                (self.state_outlet_low.T - state_low_q1.T)
        ) * self.m_flow_low
        m_flow_secondary_cp = self.m_flow_high * np.inf

        Q_first_regime = min(Q_high_tp_to_q0, Q_low_sh_to_q1)
        dT_max_first_regim = self.state_inlet_high.T - state_low_q1.T
        Q_ntu_first_regime, A_required_first_regime = ntu.calc_Q_with_available_area(
            heat_exchanger=self,
            k=self.k,
            Q_required=Q_first_regime,
            A_available=self.A,
            dT_max=dT_max_first_regim,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp
        )
        if Q_ntu_first_regime < Q_first_regime:
            # Area is only sufficient for first regime.
            self.set_missing_states(Q_ntu_first_regime)
            return
        state_low_first_regime = self.med_prop.calc_state(
            "PH",
            self.state_outlet_low.p,
            self.state_outlet_low.h - Q_ntu_first_regime / self.m_flow_low
        )
        state_high_first_regime = self.med_prop.calc_state(
            "PH",
            self.state_inlet_high.p,
            self.state_inlet_high.h - Q_ntu_first_regime / self.m_flow_high
        )
        if Q_ntu_first_regime < Q_low_sh_to_q1:
            Q_low_first_to_second_regime = self.m_flow_low * (
                state_low_first_regime.h - state_low_q1.h
            )
        else:
            Q_low_first_to_second_regime = np.inf
            m_flow_primary_cp = np.inf
        if Q_ntu_first_regime < Q_high_tp_to_q0:
            Q_high_first_to_second_regime = self.m_flow_high * (
                self.state_inlet_high.h - state_high_q0.h
            )
        else:
            Q_high_first_to_second_regime = np.inf
            state_artificial = self.med_prop.calc_state(
                "PT",
                state_high_first_regime.p,
                state_high_first_regime.T - 5  # Some dT to get cp
            )
            m_flow_secondary_cp = (
                (state_high_first_regime.h - state_artificial.h) /
                (state_high_first_regime.T - state_artificial.T)
            )
        Q_second_regime = min(Q_high_first_to_second_regime, Q_low_first_to_second_regime)
        dT_max_second_regime = state_high_first_regime.T - state_low_q1.T
        Q_ntu_second_regime, A_required_second_regime = ntu.calc_Q_with_available_area(
            heat_exchanger=self,
            k=self.k,
            Q_required=Q_second_regime,
            A_available=self.A - A_required_first_regime,
            dT_max=dT_max_second_regime,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp
        )
        state_high_second_regime = self.med_prop.calc_state(
            "PH",
            state_high_first_regime.p,
            state_high_first_regime.h - Q_ntu_second_regime / self.m_flow_high
        )
        if self.A - A_required_first_regime - A_required_second_regime < 0:
            raise ValueError("NTU calculation lead to area above 100 %")
        dT_max_third_regime = state_high_second_regime.T - state_low_q1.T
        Q_ntu_third_regime = ntu.calc_Q_ntu(
            k=self.k,
            A=self.A - A_required_first_regime - A_required_second_regime,
            dT_max=dT_max_third_regime,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp,
            flow_type=self.flow_type
        )
        self.set_missing_states(Q=Q_ntu_first_regime + Q_ntu_second_regime + Q_ntu_third_regime)
        return None, None  # Irrelevant for this heat exchanger for now.

    def set_missing_states(self, Q: float):
        self.state_inlet_low = self.med_prop.calc_state(
            "PH",
            self.state_outlet_low.p,
            self.state_outlet_low.h - Q / self.m_flow_low
        )
        self.state_outlet_high = self.med_prop.calc_state(
            "PH",
            self.state_inlet_high.p,
            self.state_inlet_high.h - Q / self.m_flow_high
        )
