from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve


class StandardCycle(BaseCycle):
    """
    Class for a standard cycle with four components.

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after evaporator
    2. Before condenser, after compressor
    3. Before EV, after condenser
    4. Before Evaporator, after EV
    """

    flowsheet_name = "Standard"

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

    def get_states(self):

        return {"1": self.compressor.state_inlet,
               "1_q1":  self.med_prop.calc_state("PQ", self.compressor.state_inlet.p, 1),
               "2": self.compressor.state_outlet,
               "2_s": self.med_prop.calc_state("PS", self.compressor.state_outlet.p, self.compressor.state_inlet.s),
               "2_q1": self.med_prop.calc_state("PQ", self.compressor.state_outlet.p, 1),
               "3_q0": self.med_prop.calc_state("PQ", self.compressor.state_outlet.p, 0),
               "3": self.condenser.state_outlet,
               "4": self.evaporator.state_inlet
               }

    def get_state_keys(self):

        return ["1",
               "1_q1",
               "2",
               "2_s",
               "2_q1",
               "3_q0",
               "3",
               "4"
               ]


    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):


        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve.state_inlet = self.condenser.state_outlet
        self.expansion_valve.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve.state_outlet
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.evaporator.state_outlet
        if inputs.fix_speed == float(False):
            n_next = 0.01
            n_step = 0.1
            max_error = 0.00001
            n_iter = 0
            while True:
                n_iter += 1
                inputs.set(
                    name="n",
                    value=n_next,
                    unit="-",
                    description="Relative compressor speed"
                )
                self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
                self.condenser.state_inlet = self.compressor.state_outlet
                self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
                self.condenser.m_flow = self.compressor.m_flow
                error = self.condenser.calc_Q_flow() - inputs.Q_con

                if abs(error) < max_error:
                    break
                elif error < 0:
                    n_next += n_step
                    continue
                n_next -= n_step
                n_step /= 10
                n_next += n_step

        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet

        # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)

        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow

        inputs.set(
            name="Q_con",
            value=self.condenser.calc_Q_flow(),
            unit="W",
            description="heat flux condenser"
        )

        inputs.set(
            name="Q_eva",
            value=self.evaporator.calc_Q_flow(),
            unit="W",
            description="heat flux evaporator"
        )

        if inputs.fix_m_flow_eva == float(False):
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            m_flow_eva = inputs.Q_eva / (self.evaporator.secondary_cp * (inputs.T_eva_in - inputs.T_eva_out))
            inputs.set(
                name="m_flow_eva",
                value=m_flow_eva,
                unit="kg/s",
                description="Secondary side evaporator mass flow"
            )
        else:
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            T_eva_out = inputs.T_eva_in - (inputs.Q_eva / (inputs.Q_eva * inputs.m_flow_eva))
            inputs.set(
                name="T_eva_out",
                value=T_eva_out,
                unit="K",
                description="Secondary side evaporator outlet temperature"
            )
        if inputs.fix_m_flow_con == float(False):
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            m_flow_con = inputs.Q_con / (self.condenser.secondary_cp * (inputs.T_con_out - inputs.T_con_in))
            inputs.set(
                name="m_flow_con",
                value=m_flow_con,
                unit="kg/s",
                description="Secondary side condenser mass flow"
            )
        else:
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            T_con_out = inputs.T_con_in + (inputs.Q_con / (self.condenser.secondary_cp * inputs.m_flow_con))
            inputs.set(
                name="T_con_out",
                value=T_con_out,
                unit="K",
                description="Secondary side condenser outlet temperature"
            )


        #fs_state.set(
            #name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
            #unit="-", description="Expansion valve opening"
        #)
        fs_state.set(name="compressor_speed", value=inputs.n * self.compressor.N_max, unit="1/s",
                     description="Compressor Speed")
        fs_state.set(name="relative_compressor_speed", value=inputs.n, unit="1/s",
                     description="relative Compressor Speed")
        fs_state.set(name="Comp_dh",value=0.001*(self.compressor.state_outlet.h-self.compressor.state_inlet.h))

        h2_is = self.med_prop.calc_state("PS", self.compressor.state_outlet.p, self.compressor.state_inlet.s).h
        h4_is = self.med_prop.calc_state("PS", self.evaporator.state_inlet.p, self.condenser.state_outlet.s).h
        Comp_dh_is = 0.001*(h2_is-self.compressor.state_inlet.h)
        Ex_dh_is = 0.001 * (self.expansion_valve.state_inlet.h - h4_is)
        comp_dh_is_Ex_dh_is = Comp_dh_is/Ex_dh_is
        fs_state.set(name="Comp_dh_is", value=Comp_dh_is)
        fs_state.set(name="Exp_dh_is", value=Ex_dh_is)
        fs_state.set(name="Comp_dh_is_Exp_dh_is", value=comp_dh_is_Ex_dh_is)
        fs_state.set(name="Comp_dH_is", value=self.compressor.m_flow*Comp_dh_is)
        fs_state.set(name="Exp_dH_is", value=self.compressor.m_flow*Ex_dh_is)



    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
