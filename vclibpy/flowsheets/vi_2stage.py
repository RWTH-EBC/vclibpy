import numpy as np
from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.media import ThermodynamicState
from vclibpy.components.phase_separator import PhaseSeparator


class VaporInjection_TwoStage(BaseCycle):
    """
    Class for a standard cycle with four components.

    For the standard cycle, we have 4 possible states:

    1. Before compressor, after evaporator
    2. Before condenser, after compressor
    3. Before EV, after condenser
    4. Before Evaporator, after EV
    """

    def __init__(
            self,
            compressor_low: Compressor,
            compressor_high: Compressor,
            expansion_valve_low: ExpansionValve,
            expansion_valve_high: ExpansionValve,
            vi_pressure_fac=1,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor_low = compressor_low
        self.compressor_high = compressor_high
        self.expansion_valve_low = expansion_valve_low
        self.expansion_valve_high = expansion_valve_high
        self.vi_pressure_fac = vi_pressure_fac

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor_low,
            self.compressor_high,
            self.expansion_valve_low,
            self.expansion_valve_high
        ]

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):

        p_vi = np.sqrt(p_1 * p_2) * self.vi_pressure_fac
        # State 3
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve_high.state_inlet = self.condenser.state_outlet
        # state 4
        self.expansion_valve_high.calc_outlet(p_outlet=p_vi)

        # state 1
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor_low.state_inlet = self.evaporator.state_outlet

        if inputs.fix_speed == float(False):
            n_next = 0.5
            n_step = 0.1
            max_rel_error = 0.0001
            bigger = False
            smaller = False
            n_iter = 0
            n_iter_max = 100000
            while n_iter <= n_iter_max:
                n_iter += 1
                inputs.set(
                    name="n",
                    value=n_next,
                    unit="-",
                    description="Relative compressor speed"
                )
                self.compressor_low.calc_state_outlet(p_outlet=p_vi, inputs=inputs, fs_state=fs_state)

                x_vapor_injection, h_vapor_injection, state_low_ev_inlet = self.calc_injection()

                # State 4
                self.expansion_valve_low.state_inlet = state_low_ev_inlet
                self.expansion_valve_low.calc_outlet(p_1)
                self.evaporator.state_inlet = self.expansion_valve_low.state_outlet

                h_1_VI_mixed = (
                        (1 - x_vapor_injection) * self.compressor_low.state_outlet.h +
                        x_vapor_injection * h_vapor_injection
                )

                self.compressor_high.state_inlet = self.med_prop.calc_state("PH", p_vi, h_1_VI_mixed)
                self.compressor_high.calc_state_outlet(
                    p_outlet=p_2, inputs=inputs, fs_state=fs_state)
                self.condenser.state_inlet = self.compressor_high.state_outlet
                self.condenser.m_flow = self.compressor_high.calc_m_flow(inputs, fs_state)

                Q_con = self.condenser.calc_Q_flow()

                rel_error = 100 * (Q_con - inputs.Q_con) / inputs.Q_con
                if abs(rel_error) < max_rel_error:
                    break
                elif rel_error < 0:
                    if n_next > 1.5:
                        n_next = 1.5
                        inputs.set(
                            name="n",
                            value=n_next,
                            unit="-",
                            description="Relative compressor speed"
                        )
                        break
                    n_next += n_step
                    bigger = True
                    if bigger and smaller:
                        n_next -= n_step
                        n_step /= 10
                        n_next += n_step
                        bigger = False
                        smaller = False
                    continue
                elif rel_error > 0:
                    if n_next < 0.2:
                        n_next = 0.2
                        inputs.set(
                            name="n",
                            value=n_next,
                            unit="-",
                            description="Relative compressor speed"
                        )
                        break
                    n_next -= n_step
                    smaller = True
                    if bigger and smaller:
                        n_next += n_step
                        n_step /= 10
                        n_next -= n_step
                        bigger = False
                        smaller = False
                    continue

        self.compressor_low.calc_state_outlet(p_outlet=p_vi, inputs=inputs, fs_state=fs_state)
        x_vapor_injection, h_vapor_injection, state_low_ev_inlet = self.calc_injection()
        # State 4
        self.expansion_valve_low.state_inlet = state_low_ev_inlet
        self.expansion_valve_low.calc_outlet(p_1)
        self.evaporator.state_inlet = self.expansion_valve_low.state_outlet

        h_1_VI_mixed = (
                (1 - x_vapor_injection) * self.compressor_low.state_outlet.h +
                x_vapor_injection * h_vapor_injection
        )

        self.compressor_high.state_inlet = self.med_prop.calc_state("PH", p_vi, h_1_VI_mixed)
        self.compressor_high.calc_state_outlet(
            p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor_high.state_outlet

        self.condenser.m_flow = self.compressor_high.calc_m_flow(inputs, fs_state)
        self.evaporator.m_flow = (1 - x_vapor_injection) * self.condenser.m_flow
        self.compressor_low.m_flow = (1 - x_vapor_injection) * self.condenser.m_flow
        n_low = self.compressor_low.calc_n(inputs, fs_state)

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
            T_con_out = inputs.T_con_in + (inputs.Q_con / self.condenser.secondary_cp * inputs.m_flow_con)
            inputs.set(
                name="T_con_out",
                value=T_con_out,
                unit="K",
                description="Secondary side condenser outlet temperature"
            )

        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="T_2", value=self.compressor_high.state_outlet.T,
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
            name="d_1", value=self.compressor_low.state_inlet.d,
            unit="kg/m3", description="Refrigerant Density at compressor inlet"
        )
        fs_state.set(
            name="delta_h_con", value=(self.condenser.state_inlet.h - self.condenser.state_outlet.h),
            unit="J/kg", description="Enthalpie difference condenser"
        )
        fs_state.set(name="p_con", value=p_2, unit="Pa", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1, unit="Pa", description="Evaporation pressure")
        fs_state.set(name="p_vi", value=p_vi, unit="Pa", description="VI pressure")
        fs_state.set(name="x_vapor_injection", value=x_vapor_injection, unit="-", description="VI ratio")
        fs_state.set(name="eta_is_low", value=self.compressor_low.get_eta_isentropic(p_vi,inputs), unit="1/s",
                     description="Compressor Speed Low isentropic eff")
        fs_state.set(name="eta_is_high", value=self.compressor_high.get_eta_isentropic(p_2, inputs), unit="1/s",
                     description="Compressor Speed high isentropic eff")
        fs_state.set(name="eta_vol_low", value=self.compressor_low.get_lambda_h(inputs), unit="1/s",
                     description="Compressor Speed Low isentropic eff")
        fs_state.set(name="eta_vol_high", value=self.compressor_high.get_lambda_h(inputs), unit="1/s",
                     description="Compressor Speed high isentropic eff")
        fs_state.set(name="compressor_speed_low", value=n_low * self.compressor_low.N_max, unit="1/s",
                     description="Compressor Speed Low Pressure")
        fs_state.set(name="compressor_speed", value=inputs.n * self.compressor_high.N_max, unit="1/s",
                     description="Compressor Speed High Pressure")
        fs_state.set(name="relative_compressor_speed", value=inputs.n, unit="1/s",
                     description="relative Compressor Speed High Pressure")
        fs_state.set(name="relative_compressor_speed_low", value=n_low, unit="1/s",
                     description="relative Compressor Speed Low Pressure")

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor_high.calc_electrical_power(inputs=inputs,
                                                          fs_state=fs_state) + self.compressor_low.calc_electrical_power(
            inputs=inputs, fs_state=fs_state)

    def calc_injection(self) -> (float, float, ThermodynamicState):
        """
        Calculate the injection component, e.g. phase separator
        or heat exchanger.
        In this function, child classes must set inlets
        and calculate outlets of additional components.

        Returns:
            float: Portion of vapor injected (x)
            float: Enthalpy of vapor injected
            ThermodynamicState: Inlet state of low pressure expansion valve
        """
        raise NotImplementedError


class VaporInjection_TwoStageFlashTank(VaporInjection_TwoStage):

    def __init__(self,
                 **kwargs):
        self.flashtank = PhaseSeparator()
        super().__init__(**kwargs)

    def get_all_components(self):
        return super().get_all_components() + [self.flashtank]

    def calc_injection(self):
        self.flashtank.state_inlet = self.expansion_valve_high.state_outlet
        return self.flashtank.state_inlet.q, self.flashtank.state_outlet_vapor.h, self.flashtank.state_outlet_liquid


class VaporInjection_TwoStageECO(VaporInjection_TwoStage):

    def __init__(self,
                 ECO,
                 **kwargs):
        super().__init__(**kwargs)
        self.eco = ECO
