import logging
from vclibpy.components.heat_exchangers.ntu import BasicNTU
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)

class InternalHeatExchangerNTU(BasicNTU):
    """
    Internal heat exchanger (IHX) based on NTU.
    Both sides are flowed through by refrigerant:
    - One side: liquid (e.g. after condenser)
    - Other side: gaseous (e.g. after evaporator)

    See parent class for more arguments.
    Assumptions:

    - Default `flow_type` is counter_flow.
    - Default `ratio_outer_to_inner_area` is 1, as
    - pipes are nearly same diameter and length
    - Secondary heat transfer for alpha is disabled; gas,
    """

    def __init__(self, two_phase_heat_transfer=None, **kwargs):
        kwargs.pop("secondary_heat_transfer", None)
        kwargs.pop("secondary_medium", None)
        super().__init__(
            flow_type=kwargs.pop("flow_type", "counter"),
            secondary_heat_transfer="None",
            secondary_medium="None",
            ratio_outer_to_inner_area=kwargs.pop("ratio_outer_to_inner_area", 1),
            two_phase_heat_transfer=two_phase_heat_transfer,

            **kwargs
        )
        self._state_liquid_inlet = None
        self._state_liquid_outlet = None
        self._state_gas_inlet = None
        self._state_gas_outlet = None

    @property
    def state_liquid_inlet(self) -> ThermodynamicState:
        return self._state_liquid_inlet

    @state_liquid_inlet.setter
    def state_liquid_inlet(self, liquid_inlet_state: ThermodynamicState):
        self._state_liquid_inlet = liquid_inlet_state

    @property
    def state_liquid_outlet(self) -> ThermodynamicState:
        return self._state_liquid_outlet

    @state_liquid_outlet.setter
    def state_liquid_outlet(self, liquid_outlet_state: ThermodynamicState):
        self._state_liquid_outlet = liquid_outlet_state

    @property
    def state_gas_inlet(self) -> ThermodynamicState:
        return self._state_gas_inlet

    @state_gas_inlet.setter
    def state_gas_inlet(self, gas_inlet_state: ThermodynamicState):
        self._state_gas_inlet = gas_inlet_state

    @property
    def state_gas_outlet(self) -> ThermodynamicState:
        return self._state_gas_outlet

    @state_gas_outlet.setter
    def state_gas_outlet(self, gas_outlet_state: ThermodynamicState):
        self._state_gas_outlet = gas_outlet_state

    def calc(self, inputs, fs_state):
        raise NotImplementedError("Calculation method for IHX not yet implemented.")

    def set_secondary_cp(self, cp: float):
        self._secondary_cp = cp

    def start_secondary_med_prop(self):
        self.med_prop_sec = self.med_prop

    def terminate_secondary_med_prop(self):
        pass

    def calc_alpha_secondary(self, transport_properties):
        raise NotImplementedError("IHX does not use a secondary medium.")

    def calc_transport_properties_secondary_medium(self, T, p=None):
        raise NotImplementedError("IHX does not use a secondary medium.")