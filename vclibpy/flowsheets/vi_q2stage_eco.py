import math
import logging
from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.heat_exchangers import MVB_LMTD_IHX

logger = logging.getLogger(__name__)


class VI_q2StageECO(BaseCycle):
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
            eco: MVB_LMTD_IHX,
            vi_pressure_fac=1,
            y_inj=None,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor_low = compressor_low
        self.compressor_high = compressor_high
        self.expansion_valve_low = expansion_valve_low
        self.expansion_valve_high = expansion_valve_high
        self.vi_pressure_fac = vi_pressure_fac
        self.y_inj = y_inj
        self.eco = eco

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor_low,
            self.compressor_high,
            self.expansion_valve_low,
            self.expansion_valve_high,
            self.eco
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
            "5": self.eco.state_outlet_low,
            "6": self.eco.state_outlet_high,
            "7": self.evaporator.state_inlet
        }

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):

        # State 3
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve_high.state_inlet = self.condenser.state_outlet
        self.eco.state_inlet_high = self.condenser.state_outlet

        # state 1
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor_low.state_inlet = self.evaporator.state_outlet

        eta_is = self.compressor_low.get_eta_isentropic(p_outlet=p_2, inputs=inputs)
        lambda_h = self.compressor_low.get_lambda_h(p_outlet=p_2, inputs=inputs)

        if self.vi_pressure_fac is None:
            p_vi = p_1
            step_p_vi = 100000
        else:
            p_vi = self.vi_pressure_fac * math.sqrt(p_1 * p_2)
            step_p_vi = 0

        while True:
            p_vi += step_p_vi
            # State 4
            self.expansion_valve_high.calc_outlet(p_outlet=p_vi)
            self.eco.state_inlet_low = self.expansion_valve_high.state_outlet

            # state 1*
            self.compressor_low.calc_state_outlet(p_outlet=p_vi, inputs=inputs, fs_state=fs_state, eta_is=eta_is)

            # State 5
            self.eco.state_outlet_low = self.med_prop.calc_state("PQ", p_vi, 1)

            n_step = 0.1
            if inputs.fix_speed == float(False):
                inputs.set(
                    name="n",
                    value=0.2,
                    unit="-",
                    description="Relative compressor speed"
                )
            while True:
                if self.y_inj is None:
                    y_inj = 0.0001
                else:
                    y_inj = self.y_inj
                error = 10000
                y_inj_step = 0.01
                while abs(error) > 0.1 and y_inj < 0.19 and y_inj_step > 0.00001:
                    # State 1**
                    h_1_VI_mixed = (
                            (1 - y_inj) * self.compressor_low.state_outlet.h +
                            y_inj * self.eco.state_outlet_low.h
                    )
                    self.compressor_high.state_inlet = self.med_prop.calc_state("PH", p_vi, h_1_VI_mixed)

                    m_flow_ref = self.compressor_high.calc_m_flow(inputs=inputs, fs_state=fs_state, lambda_h=lambda_h)
                    self.eco.m_flow_low = y_inj * m_flow_ref
                    self.eco.m_flow_high = (1 - y_inj) * m_flow_ref

                    # State 6
                    Q_eco = self.eco.m_flow_low * (self.eco.state_outlet_low.h - self.eco.state_inlet_low.h)
                    h_6 = self.eco.state_inlet_high.h - Q_eco / self.eco.m_flow_high
                    self.eco.state_outlet_high = self.med_prop.calc_state("PH", p_2, h_6)
                    if self.y_inj is not None:
                        break
                    error, dT_min = self.eco.calc(inputs, fs_state)
                    if error > 0:
                        y_inj += y_inj_step
                    else:
                        y_inj -= y_inj_step
                        y_inj_step /= 10
                        y_inj += y_inj_step

                # State 2
                self.compressor_high.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state, eta_is=eta_is)
                self.condenser.state_inlet = self.compressor_high.state_outlet
                self.condenser.m_flow = self.compressor_high.m_flow
                Q_con = self.condenser.calc_Q_flow()
                if inputs.fix_speed == float(True):
                    break
                n = inputs.n
                if Q_con / inputs.Q_con < 1:
                    inputs.set(
                        name="n",
                        value=n + n_step,
                        unit="-",
                        description="Relative compressor speed"
                    )
                    continue
                elif Q_con / inputs.Q_con < 1.001:
                    break
                else:
                    n -= n_step
                    n_step /= 10
                    inputs.set(
                        name="n",
                        value=n + n_step,
                        unit="-",
                        description="Relative compressor speed"
                    )
                    continue

            # State 6 + 7
            self.expansion_valve_low.state_inlet = self.eco.state_outlet_high
            self.expansion_valve_low.calc_outlet(p_outlet=p_1)
            self.evaporator.state_inlet = self.expansion_valve_low.state_outlet
            self.evaporator.m_flow = self.eco.m_flow_high
            self.compressor_low.m_flow = self.eco.m_flow_high

            n_low = self.compressor_low.calc_n(inputs=inputs, fs_state=fs_state, lambda_h=lambda_h)
            n_high = self.compressor_high.calc_n(inputs=inputs, fs_state=fs_state, lambda_h=lambda_h)
            if self.vi_pressure_fac is not None:
                break
            if n_low / n_high < 1:
                continue
            elif n_low / n_high < 1.0001:
                break
            else:
                p_vi -= step_p_vi
                step_p_vi /= 10

        Q_eva = self.evaporator.calc_Q_flow()
        Q_con = self.condenser.calc_Q_flow()
        if inputs.fix_m_flow_eva == float(False):
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            m_flow_eva = Q_eva / (self.evaporator.secondary_cp * (inputs.T_eva_in - inputs.T_eva_out))
            inputs.set(
                name="m_flow_eva",
                value=m_flow_eva,
                unit="kg/s",
                description="Secondary side evaporator mass flow"
            )
        else:
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            T_eva_out = inputs.T_eva_in - (Q_eva / (Q_eva * inputs.m_flow_eva))
            inputs.set(
                name="T_eva_out",
                value=T_eva_out,
                unit="K",
                description="Secondary side evaporator outlet temperature"
            )
        if inputs.fix_m_flow_con == float(False):
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            m_flow_con = Q_con / (self.condenser.secondary_cp * (inputs.T_con_out - inputs.T_con_in))
            inputs.set(
                name="m_flow_con",
                value=m_flow_con,
                unit="kg/s",
                description="Secondary side condenser mass flow"
            )
        else:
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            T_con_out = inputs.T_con_in + (Q_con / self.condenser.secondary_cp * inputs.m_flow_con)
            inputs.set(
                name="T_con_out",
                value=T_con_out,
                unit="K",
                description="Secondary side condenser outlet temperature"
            )

        fs_state.set(name="x_vapor_injection", value=y_inj, unit="-", description="VI ratio")
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
        fs_state.set(name="Comp_dh", value=0.001 * (self.compressor_high.state_outlet.h -
                                                    self.compressor_high.state_inlet.h + (1 - y_inj) *
                                                    (self.compressor_low.state_outlet.h -
                                                     self.compressor_low.state_inlet.h)))

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor_high.calc_electrical_power(inputs=inputs,
                                                          fs_state=fs_state) + self.compressor_low.calc_electrical_power(
            inputs=inputs, fs_state=fs_state)
