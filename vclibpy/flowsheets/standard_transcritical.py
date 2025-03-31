from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve


class StandardCycleTranscritical(BaseCycle):
    """
    Class for a standard cycle with four components.

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after evaporator
    2. Before condenser, after compressor
    3. Before EV, after condenser
    4. Before Evaporator, after EV
    """

    flowsheet_name = "StandardTranscritical"

    def __init__(
            self,
            compressor: Compressor,
            expansion_valve: ExpansionValve,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valve = expansion_valve

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.expansion_valve
        ]

    def get_states_in_order_for_plotting(self):
        return [
            self.evaporator.state_inlet,
            self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),
            self.evaporator.state_outlet,
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.condenser.state_inlet,
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 1),
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 0),
            self.condenser.state_outlet,
            self.expansion_valve.state_inlet,
            self.expansion_valve.state_outlet,
        ]

    def set_condenser_outlet_based_on_q(self, p_con: float, inputs: Inputs, q_4, p_eva: float):
        h_4 = self.med_prop.calc_state("PQ", p_eva, q_4).h
        self.condenser.state_outlet = self.med_prop.calc_state("PH", p_con, h_4)
        #print(self.condenser.state_outlet)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        """
        This function calculates the states of a standard heat pump under
        specific conditions while adhering to several general assumptions.

        General Assumptions:
        ---------------------
        - Isenthalpic expansion valves:
          The enthalpy at the inlet equals the enthalpy at the outlet.
        - Input to the evaporator is always in the two-phase region.
        - Output of the evaporator and output of the condenser maintain
          a constant overheating or subcooling (can be set in Inputs).
        """


        last_cop = 1
        q_4_step = 0.1
        q_4 = 0.15

        while q_4_step > 0.0001:
            self.set_condenser_outlet_based_on_q(p_con=p_2, inputs=inputs, q_4=q_4, p_eva=p_1)
            self.expansion_valve.state_inlet = self.condenser.state_outlet
            self.expansion_valve.calc_outlet(p_outlet=p_1)
            self.evaporator.state_inlet = self.expansion_valve.state_outlet
            self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
            self.compressor.state_inlet = self.evaporator.state_outlet
            self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
            self.condenser.state_inlet = self.compressor.state_outlet
            # Mass flow rate:
            self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
            self.condenser.m_flow = self.compressor.m_flow
            self.evaporator.m_flow = self.compressor.m_flow
            self.expansion_valve.m_flow = self.compressor.m_flow
            Q_con = self.condenser.calc_Q_flow()
            P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)
            current_cop = Q_con / P_el
            #print("COP: ", current_cop)
            if current_cop < last_cop:
                q_4 += q_4_step
                q_4_step /= 10
                q_4 -= q_4_step
                if 0 > q_4 or q_4 > 1:
                    q_4 += q_4_step
                    q_4_step /= 10
            else:
                q_4 -= q_4_step
                if 0 > q_4 or q_4 > 1:
                    q_4 += q_4_step
                    q_4_step /= 10
            #print("q_4: ", q_4)
            last_cop = current_cop

        fs_state.set(
            name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
            unit="-", description="Expansion valve opening"
        )
        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="T_2", value=self.compressor.state_outlet.T,
            unit="K", description="Compressor outlet temperature"
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T, unit="K",
            description="Refrigerant temperature at condenser outlet"
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_inlet.T,
            unit="K", description="Refrigerant temperature at evaporator inlet"
        )
        fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)


