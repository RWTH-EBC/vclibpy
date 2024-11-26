import logging

from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.heat_exchangers.ihx_ntu import IHX_NTU

logger = logging.getLogger(__name__)


class IHX(BaseCycle):
    """
    Class for a IHX cycle with internal heat exchanger (ihx).

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after ihx low temperature side
    2. Before condenser, after compressor
    3. Before first EV, after condenser
    4. Before ihx high temperature side, after first EV
    5. Before second EV, after ihx high temperature side
    6. Before evaporator, after second EV
    7. Before idx low temperature side, after evaporator
    """

    flowsheet_name = "IHX"

    def __init__(
            self,
            compressor: Compressor,
            expansion_valve_high: ExpansionValve,
            expansion_valve_low: ExpansionValve,
            ihx: IHX_NTU,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valve_high = expansion_valve_high
        self.expansion_valve_low = expansion_valve_low
        self.ihx = ihx
        self.flowsheet_name = "IHX"

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.expansion_valve_high,
            self.expansion_valve_low,
            self.ihx
        ]

    def get_states_in_order_for_plotting(self):
        return [
            self.med_prop.calc_state("PQ", self.compressor.state_inlet.p, 1),
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.med_prop.calc_state("PQ", self.compressor.state_outlet.p, 1),
            self.med_prop.calc_state("PQ", self.compressor.state_outlet.p, 0),
            self.condenser.state_outlet,
            self.expansion_valve_high.state_outlet,
            # TODO: Case for phase change in IHX
            self.expansion_valve_low.state_inlet,
            self.evaporator.state_inlet,
            self.evaporator.state_outlet
            # TODO: Case for phase change in IHX
        ]

    def set_ihx_outlet_based_on_superheating(self, p_eva: float, inputs: Inputs):
        T_1 = self.med_prop.calc_state("PQ", p_eva, 1).T + inputs.control.dT_eva_superheating
        if inputs.control.dT_eva_superheating > 0:
            self.ihx.state_outlet_low = self.med_prop.calc_state("PT", p_eva, T_1)
        else:
            self.ihx.state_outlet_low = self.med_prop.calc_state("PQ", p_eva, 1)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        # State 1
        self.set_ihx_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.ihx.state_outlet_low
        # State 2
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet
        # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.condenser.m_flow = self.compressor.m_flow
        self.expansion_valve_high.m_flow = self.compressor.m_flow
        self.ihx.m_flow = self.compressor.m_flow
        self.expansion_valve_low.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.ihx.m_flow_high = self.compressor.m_flow
        self.ihx.m_flow_low = self.compressor.m_flow

        # State 3
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve_high.state_inlet = self.condenser.state_outlet
        # State 4
        if "opening" in inputs.control.get_variable_names():
            opening = inputs.control.opening
        else:
            opening = 1
        self.expansion_valve_high.calc_outlet_pressure_at_m_flow_and_opening(
            m_flow=self.compressor.m_flow,
            opening=opening
        )
        if self.expansion_valve_high.state_outlet.T - self.ihx.dT_pinch_min < self.ihx.state_outlet_low.T:
            T_min = self.ihx.state_outlet_low.T + self.ihx.dT_pinch_min
            self.expansion_valve_high.calc_outlet(p_outlet=self.med_prop.calc_state("TQ", T_min, 0).p)
            logger.info(
                "Pressure to low to at given opening to match dT_pinch_min=%s. "
                "Setting minimal required pressure."
            )
        self.ihx.state_inlet_high = self.expansion_valve_high.state_outlet
        # State 5
        self.ihx.calc(inputs=inputs, fs_state=fs_state)
        self.expansion_valve_low.state_inlet = self.ihx.state_outlet_high
        # State 6
        self.expansion_valve_low.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve_low.state_outlet
        # State 7
        self.evaporator.state_outlet = self.ihx.state_inlet_low

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
