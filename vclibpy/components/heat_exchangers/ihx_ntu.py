import numpy as np
import logging

from vclibpy.components.heat_exchangers import InternalHeatExchanger, ntu
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer
from vclibpy import Inputs, FlowsheetState

logger = logging.getLogger(__name__)


class IHX_NTU(InternalHeatExchanger):

    def __init__(
            self,
            gas_heat_transfer: HeatTransfer,
            two_phase_heat_transfer_high: TwoPhaseHeatTransfer,
            dT_pinch_min: float = 0,
            **kwargs):
        super().__init__(**kwargs)
        self.dT_pinch_min = dT_pinch_min
        assert self.flow_type == "counter", "Other types are not implemented"
        self._gas_heat_transfer = gas_heat_transfer
        self._two_phase_heat_transfer_high = two_phase_heat_transfer_high

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        # First calculate the heat assuming q1 at inlet of lower side:
        state_low_q1 = self.med_prop.calc_state("PQ", self.state_outlet_low.p, 1)
        state_high_q0 = self.med_prop.calc_state("PQ", self.state_inlet_high.p, 0)
        state_high_q1 = self.med_prop.calc_state("PQ", self.state_inlet_high.p, 1)

        dh_max_high = self.state_inlet_high.h - state_high_q0.h
        dh_max_low = self.state_outlet_low.h - state_low_q1.h

        primary_cp = ((self.state_outlet_low.h - state_low_q1.h) / (self.state_outlet_low.T - state_low_q1.T))
        transport_gas_low = self.med_prop.calc_mean_transport_properties(state_low_q1, self.state_outlet_low)
        alpha_gas_low = self._gas_heat_transfer.calc(transport_gas_low, self.m_flow_low)

        # Most of those settings have no effect, only if the twp-phase correlation actually includes these points
        alpha_two_phase_high = self._two_phase_heat_transfer_high.calc(
            state_q0=state_high_q0,
            state_q1=state_high_q1,
            inputs=inputs,
            fs_state=fs_state,
            m_flow=self.m_flow_high,
            med_prop=self.med_prop,
            state_inlet=self.state_inlet_high,
            state_outlet=state_high_q0
        )
        k = self.calc_k(
            alpha_pri=alpha_gas_low,
            alpha_sec=alpha_two_phase_high
        )
        Q_ntu = ntu.calc_Q_ntu(
            dT_max=self.state_inlet_high.T - self.state_outlet_low.T,
            k=k,
            A=self.A,
            m_flow_primary_cp=self.m_flow_low * primary_cp,
            m_flow_secondary_cp=self.m_flow_high * np.inf,
            flow_type=self.flow_type
        )

        dh = Q_ntu / self.m_flow_high
        # Both regimes are exceeded
        if dh > dh_max_low:
            remaining_dh_low = dh - dh_max_low
            logger.warning(
                f"High side still in two-phase region, but low side passed "
                f"to two-phase as well. Error in dh: %s", remaining_dh_low
            )
        if dh > dh_max_high:
            remaining_dh_high = dh - dh_max_high
            logger.warning(
                f"High side still in subcooling region, but low side still in gas region"
                f"Error in dh: %s", remaining_dh_high
            )
        self.state_inlet_low = self.med_prop.calc_state(
            "PH",
            self.state_outlet_low.p,
            self.state_outlet_low.h - dh
        )
        self.state_outlet_high = self.med_prop.calc_state(
            "PH",
            self.state_inlet_high.p,
            self.state_inlet_high.h - dh
        )
        return None, None  # Irrelevant for this heat exchanger for now.
