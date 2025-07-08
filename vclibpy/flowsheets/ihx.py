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

    For the internal heat exchanger cycle, we have 7 possible states:

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
        first_states = [
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.condenser.state_inlet,
            self.med_prop.calc_state("PQ", self.condenser.state_outlet.p, 1),
            self.med_prop.calc_state("PQ", self.condenser.state_outlet.p, 0),
            self.condenser.state_outlet,
            self.expansion_valve_high.state_inlet,
            self.expansion_valve_high.state_outlet,
            self.ihx.state_inlet_high
        ]
        second_part_states = [
            self.ihx.state_outlet_high,
            self.expansion_valve_low.state_inlet,
            self.expansion_valve_low.state_outlet,
            self.evaporator.state_inlet,
            self.evaporator.state_outlet,
            self.ihx.state_inlet_low,
        ]
        third_part_states = [
            self.ihx.state_outlet_low
        ]
        # ihx_high in tp region:
        state_ihx_high_q0 = self.med_prop.calc_state("PQ", self.ihx.state_inlet_high.p, 0)
        if state_ihx_high_q0.h < self.ihx.state_inlet_high.h:
            states_until_low_side = first_states + [state_ihx_high_q0] + second_part_states
        else:
            states_until_low_side = first_states + second_part_states
        state_ihx_low_q1 = self.med_prop.calc_state("PQ", self.ihx.state_inlet_low.p, 1)
        if state_ihx_low_q1.h > self.ihx.state_inlet_low.h:
            return states_until_low_side + [state_ihx_low_q1] + third_part_states
        return states_until_low_side + third_part_states

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
        try:
            self.expansion_valve_high.calc_outlet_pressure_at_m_flow_and_opening(
                m_flow=self.compressor.m_flow,
                opening=opening
            )
        except ValueError as err:
            # During iteration, p_1 and p_2 can get very close together,
            # leading to high m_flows and then p_outlets below zero, depending on the EV model
            T_min = self.ihx.state_outlet_low.T + self.ihx.dT_pinch_min
            self.expansion_valve_high.calc_outlet(p_outlet=self.med_prop.calc_state("TQ", T_min, 0).p)
        dp_ev_high = self.expansion_valve_high.state_inlet.p - self.expansion_valve_high.state_outlet.p
        if self.expansion_valve_high.state_outlet.T - self.ihx.dT_pinch_min < self.ihx.state_outlet_low.T:
            T_min = self.ihx.state_outlet_low.T + self.ihx.dT_pinch_min
            self.expansion_valve_high.calc_outlet(p_outlet=self.med_prop.calc_state("TQ", T_min, 0).p)
            logger.info(
                "Pressure to low to at given opening to match dT_pinch_min=%s. "
                "Setting minimal required pressure."
            )
            # TODO: Re-calculate required opening
        self.ihx.state_inlet_high = self.expansion_valve_high.state_outlet
        # State 5
        self.ihx.calc(inputs=inputs, fs_state=fs_state)
        self.expansion_valve_low.state_inlet = self.ihx.state_outlet_high
        # State 6
        self.expansion_valve_low.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve_low.state_outlet
        # State 7
        self.evaporator.state_outlet = self.ihx.state_inlet_low
        fs_state.set(name="dp_ev_high", value=dp_ev_high / 1e5, unit="bar", description="Pressure difference due to first EV")
        for no, state in enumerate([
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.condenser.state_outlet,
            self.expansion_valve_high.state_outlet,
            self.ihx.state_outlet_high,
            self.expansion_valve_low.state_outlet,
            self.evaporator.state_outlet,
        ]):
            fs_state.set(name=f"T_{no+1}", value=state.T, unit="K", description=f"Temperature in state {no+1}")

        fs_state.set(name="p_con", value=p_2 / 1e5, unit="bar", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1 / 1e5, unit="bar", description="Evaporation pressure")

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
