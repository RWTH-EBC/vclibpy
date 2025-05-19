import math
import logging
from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.phase_separator import PhaseSeparator


logger = logging.getLogger(__name__)


class VI_q2StageFLT(BaseCycle):
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
            vi_pressure_fac=None,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor_low = compressor_low
        self.compressor_high = compressor_high
        self.expansion_valve_low = expansion_valve_low
        self.expansion_valve_high = expansion_valve_high
        self.vi_pressure_fac = vi_pressure_fac
        self.flashtank = PhaseSeparator()

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor_low,
            self.compressor_high,
            self.expansion_valve_low,
            self.expansion_valve_high,
            self.flashtank
        ]

    def get_states(self):

        return {
            "1": self.compressor_low.state_inlet,
            "1_q1": self.med_prop.calc_state("PQ", self.compressor_low.state_inlet.p, 1),
            "1*": self.compressor_low.state_outlet,
            "1*s": self.med_prop.calc_state("PS", self.compressor_low.state_outlet.p,
                                            self.compressor_low.state_inlet.s),
            "1**": self.compressor_high.state_inlet,
            "2": self.compressor_high.state_outlet,
            "2s": self.med_prop.calc_state("PS", self.compressor_high.state_outlet.p,
                                           self.compressor_high.state_inlet.s),
            "2_q1": self.med_prop.calc_state("PQ", self.compressor_high.state_outlet.p, 1),
            "3_q0": self.med_prop.calc_state("PQ", self.compressor_high.state_outlet.p, 0),
            "3": self.condenser.state_outlet,
            "4": self.expansion_valve_high.state_outlet,
            "5": self.med_prop.calc_state("PQ", self.expansion_valve_high.state_outlet.p, 1),
            "6": self.med_prop.calc_state("PQ", self.expansion_valve_high.state_outlet.p, 0),
            "7": self.evaporator.state_inlet
        }


    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):

        # State 3
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve_high.state_inlet = self.condenser.state_outlet

        # state 1
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor_low.state_inlet = self.evaporator.state_outlet

        eta_is = self.compressor_low.get_eta_isentropic(p_outlet=p_2, inputs=inputs)
        lambda_h = self.compressor_low.get_lambda_h(p_outlet=p_2, inputs=inputs)


        if self.vi_pressure_fac is None:
            step_p_vi = 100000
            p_vi = p_1
        else:
            step_p_vi = 0
            p_vi = self.vi_pressure_fac*math.sqrt(p_1*p_2)

        while True:
            p_vi += step_p_vi
            # state 4
            self.expansion_valve_high.calc_outlet(p_outlet=p_vi)
            # state 1*
            self.compressor_low.calc_state_outlet(p_outlet=p_vi, inputs=inputs, fs_state=fs_state, eta_is=eta_is)
            # state 5

            # state 4
            self.flashtank.state_inlet = self.expansion_valve_high.state_outlet
            # state 6
            self.expansion_valve_low.state_inlet = self.flashtank.state_outlet_liquid
            # state 7
            self.expansion_valve_low.calc_outlet(self.evaporator.state_outlet.p)
            self.evaporator.state_inlet = self.expansion_valve_low.state_outlet


            # state 1**
            x_vapor_injection, h_vapor_injection = self.flashtank.state_inlet.q, self.flashtank.state_outlet_vapor.h
            h_1_VI_mixed = (
                    (1 - x_vapor_injection) * self.compressor_low.state_outlet.h +
                    x_vapor_injection * h_vapor_injection
            )
            self.compressor_high.state_inlet = self.med_prop.calc_state("PH", p_vi, h_1_VI_mixed)

            # state 2
            self.compressor_high.calc_state_outlet(
                p_outlet=p_2, inputs=inputs, fs_state=fs_state, eta_is=eta_is)
            # iterate compressor speed
            self.condenser.state_inlet = self.compressor_high.state_outlet
            if inputs.fix_speed == float(False):
                n_next = 0.01
                n_step = 0.1
                max_error = 0.001
                n_iter = 0
                while True:
                    n_iter += 1
                    inputs.set(
                        name="n",
                        value=n_next,
                        unit="-",
                        description="Relative compressor speed"
                    )

                    # m_flow condenser
                    self.condenser.m_flow = self.compressor_high.calc_m_flow(inputs, fs_state, lambda_h=lambda_h)
                    rel_error = 100 * (self.condenser.calc_Q_flow() - inputs.Q_con) / inputs.Q_con

                    if abs(rel_error) < max_error:
                        break
                    elif rel_error < 0:
                        n_next += n_step
                        continue
                    n_next -= n_step
                    n_step /= 10
                    n_next += n_step
                    continue

            # m_flow condenser
            self.condenser.m_flow = self.compressor_high.calc_m_flow(inputs, fs_state, lambda_h=lambda_h)
            self.compressor_low.m_flow = (1 - x_vapor_injection) * self.condenser.m_flow

            # m_flow evaporator
            self.evaporator.m_flow = self.compressor_low.m_flow

            # compressor speeds
            n_low = self.compressor_low.calc_n(inputs, fs_state, lambda_h=lambda_h)
            n_high = self.compressor_high.calc_n(inputs, fs_state, lambda_h=lambda_h)
            if self.vi_pressure_fac is not None:
                break
            n_ratio = n_low/n_high
            if n_ratio > 1:
                p_vi -= step_p_vi
                step_p_vi /= 10
                if step_p_vi < 1 or n_ratio < 1.001:
                    break

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

        fs_state.set(name="x_vapor_injection", value=x_vapor_injection, unit="-", description="VI ratio")
        fs_state.set(name="eta_is_low", value=eta_is, unit="1/s",
                     description="Compressor Speed Low isentropic eff")
        fs_state.set(name="eta_is_high", value=eta_is, unit="1/s",
                     description="Compressor Speed high isentropic eff")
        fs_state.set(name="eta_vol_low", value=lambda_h, unit="1/s",
                     description="Compressor Speed Low isentropic eff")
        fs_state.set(name="eta_vol_high", value=lambda_h, unit="1/s",
                     description="Compressor Speed high isentropic eff")
        fs_state.set(name="compressor_speed_low", value=n_low * self.compressor_low.N_max, unit="1/s",
                     description="Compressor Speed Low Pressure")
        fs_state.set(name="compressor_speed", value=inputs.n * self.compressor_high.N_max, unit="1/s",
                     description="Compressor Speed High Pressure")
        fs_state.set(name="relative_compressor_speed", value=inputs.n, unit="1/s",
                     description="relative Compressor Speed High Pressure")
        fs_state.set(name="relative_compressor_speed_low", value=n_low, unit="1/s",
                     description="relative Compressor Speed Low Pressure")

        fs_state.set(name="p_con", value=p_2 / 100000, unit="bar", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1 / 100000, unit="bar", description="Evaporation pressure")
        fs_state.set(name="p_vi", value=p_vi / 100000, unit="bar", description="VI pressure")
        fs_state.set(name="Comp_dh",
                     value=0.001*(self.compressor_high.state_outlet.h-self.compressor_high.state_inlet.h +
                                  (1-x_vapor_injection) *
                                  (self.compressor_low.state_outlet.h-self.compressor_low.state_inlet.h)))


        h1star_is = self.med_prop.calc_state("PS", self.compressor_low.state_outlet.p, self.compressor_low.state_inlet.s).h
        Comp_low_dh_is = 0.001 * (h1star_is - self.compressor_low.state_inlet.h)
        h2_is = self.med_prop.calc_state("PS", self.compressor_high.state_outlet.p, self.compressor_high.state_inlet.s).h
        Comp_high_dh_is = 0.001 * (h2_is - self.compressor_low.state_inlet.h)
        Comp_dh_is = Comp_high_dh_is + (1 - x_vapor_injection) * Comp_low_dh_is
        fs_state.set(name="Comp_low_dh_is", value=Comp_low_dh_is)
        fs_state.set(name="Comp_high_dh_is", value=Comp_high_dh_is)
        fs_state.set(name="Comp_dh_is", value=Comp_dh_is)
        fs_state.set(name="Comp_low_dH_is", value=self.compressor_low.m_flow*Comp_low_dh_is)
        fs_state.set(name="Comp_high_dH_is", value=self.compressor_high.m_flow*Comp_high_dh_is)
        fs_state.set(name="Comp_dH_is", value=self.compressor_high.m_flow*Comp_high_dh_is+self.compressor_low.m_flow*Comp_low_dh_is)

        h4_is = self.med_prop.calc_state("PS", p_vi, self.expansion_valve_high.state_inlet.s).h
        h7_is = self.med_prop.calc_state("PS", self.evaporator.state_inlet.p, self.expansion_valve_low.state_inlet.s).h
        Ex_high_dh_is = 0.001 * (self.expansion_valve_high.state_inlet.h - h4_is)
        Ex_low_dh_is = 0.001 * (self.expansion_valve_low.state_inlet.h - h7_is)
        Ex_dh_is = Ex_high_dh_is * x_vapor_injection + Ex_low_dh_is * (1-x_vapor_injection)

        fs_state.set(name="Exp_high_dh_is", value=Ex_high_dh_is)
        fs_state.set(name="Exp_low_dh_is", value=Ex_low_dh_is)
        fs_state.set(name="Ex_dh_is", value=Ex_dh_is)
        fs_state.set(name="Ex_dH_is", value=Ex_dh_is * self.compressor_high.m_flow)
        fs_state.set(name="Comp_dh_is_Exp_dh_is", value=Comp_dh_is/Ex_dh_is)


    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor_high.calc_electrical_power(inputs=inputs,
                                                          fs_state=fs_state) + self.compressor_low.calc_electrical_power(
            inputs=inputs, fs_state=fs_state)
