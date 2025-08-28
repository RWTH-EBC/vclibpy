import logging

from vclibpy.components.heat_exchangers import ExternalHeatExchanger
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)


class VaporInjectionEconomizerNTU(ExternalHeatExchanger):
    """
    Economizer heat exchanger which is NTU based.
    Used only for vapor injection cycles, as
    calculations are purely based for two-phase
    and liquid estimations.

    See parent class for more arguments.

    Assumptions:

    - Default `flow_type` is counter_flow.
    - Default `ratio_outer_to_inner_area` is 1, as
    - pipes are nearly same diameter and length
    - Secondary heat transfer for alpha is disabled; gas,
    liquid and two-phase models are used for both sides.
    """

    def __init__(self, **kwargs):
        kwargs.pop("secondary_heat_transfer", None)
        kwargs.pop("secondary_medium", None)
        self._state_two_phase_outlet = None
        self._state_two_phase_inlet = None
        super().__init__(
            flow_type=kwargs.pop("flow_type", "counter"),
            secondary_heat_transfer="None",
            secondary_medium="None",
            ratio_outer_to_inner_area=kwargs.pop("ratio_outer_to_inner_area", 1),
            **kwargs)

    @property
    def state_two_phase_inlet(self) -> ThermodynamicState:
        return self._state_two_phase_inlet

    @state_two_phase_inlet.setter
    def state_two_phase_inlet(self, state_inlet: ThermodynamicState):
        self._state_two_phase_inlet = state_inlet

    @property
    def state_two_phase_outlet(self) -> ThermodynamicState:
        return self._state_two_phase_outlet

    @state_two_phase_outlet.setter
    def state_two_phase_outlet(self, state_outlet: ThermodynamicState):
        self._state_two_phase_outlet = state_outlet

    def calc(self, inputs, fs_state) -> (float, float):
        raise NotImplementedError("Could be moved from VaporInjectionEconomizer")

    def set_secondary_cp(self, cp: float):
        """Set primary m_flow_cp"""
        self._secondary_cp = cp

    def start_secondary_med_prop(self):
        self.med_prop_sec = self.med_prop

    def terminate_secondary_med_prop(self):
        pass  # Not required as it is the central med_prop class

    def calc_alpha_secondary(self, transport_properties):
        raise NotImplementedError("Economizer does not use secondary heat transfer model.")

    def calc_transport_properties_secondary_medium(self, T, p=None):
        raise NotImplementedError("Economizer does not use this method")
