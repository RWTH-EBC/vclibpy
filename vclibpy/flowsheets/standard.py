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

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve.state_inlet = self.condenser.state_outlet
        self.expansion_valve.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve.state_outlet
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.evaporator.state_outlet
        n_start = inputs.n
        if inputs.Q_con_set > 0 > inputs.n:
            n_next = 0.5
            n_step = 0.1
            max_rel_error = 0.0001
            bigger = False
            smaller = False
            n_iter = 0
            n_iter_max = 100000
            while n_iter <= n_iter_max:
                n_iter +=1
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
                Q_con = self.condenser.calc_Q_flow()
                rel_error = 100 * (Q_con - inputs.Q_con_set)/inputs.Q_con_set
                if abs(rel_error) < max_rel_error:
                    break
                elif rel_error < 0:
                    if n_next > 1.5:
                        n_next = 1.5
                        break
                    n_next+= n_step
                    bigger = True
                    if bigger and smaller:
                        n_next -= n_step
                        n_step/=10
                        n_next += n_step
                        bigger = False
                        smaller = False
                    continue
                elif rel_error > 0:
                    if n_next < 0.2:
                       n_next = 0.2
                       break
                    n_next -= n_step
                    smaller = True
                    if bigger and smaller:
                        n_next += n_step
                        n_step/=10
                        n_next -= n_step
                        bigger = False
                        smaller = False
                    continue
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
            description="heating power"
        )

        inputs.set(
            name="Q_eva",
            value=self.evaporator.calc_Q_flow(),
            unit="W",
            description="Evaporating power"
        )

        if inputs.T_eva_out_set > - 9999:
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            m_flow_eva = inputs.Q_eva / (self.evaporator.secondary_cp * (inputs.T_eva_in - inputs.T_eva_out_set))
            inputs.set(
                name="m_flow_eva",
                value=m_flow_eva,
                unit="kg/s",
                description="Secondary side evaporator mass flow rate"
            )
        else:
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            T_eva_out = inputs.T_eva_in - (inputs.Q_eva/(inputs.Q_eva*inputs.m_flow_eva))
            inputs.set(
                name="T_eva_out",
                value=T_eva_out,
                unit="K",
                description="Secondary side evaporator outlet temperature"
            )
        if inputs.T_con_out_set > - 9999:
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            m_flow_con = inputs.Q_con / (self.condenser.secondary_cp * (inputs.T_con_out_set - inputs.T_con_in))
            inputs.set(
                name="m_flow_con",
                value=m_flow_con,
                unit="kg/s",
                description="Secondary side condenser mass flow rate"
            )
        else:
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            T_con_out = inputs.T_con_in + (inputs.Q_con/self.condenser.secondary_cp*inputs.m_flow_con)
            inputs.set(
                name="T_con_out",
                value=T_con_out,
                unit="K",
                description="Secondary side condenser outlet temperature"
            )

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
        fs_state.set(
            name="d_1", value=self.compressor.state_inlet.d,
            unit="kg/m3", description="Refrigerant Density at compressor inlet"
        )
        fs_state.set(
            name="delta_h_con", value=(self.condenser.state_inlet.h-self.condenser.state_outlet.h),
            unit="J/kg", description="Enthalpie difference condenser"
        )
        fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")
        fs_state.set(name="compressor_speed_calc", value=inputs.n, unit="1/s", description="Compressor Speed")

        inputs.set(
            name="n",
            value=n_start,
            unit="-",
            description="Relative compressor speed"
        )

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
