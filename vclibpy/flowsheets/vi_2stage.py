import math
import logging
import numpy as np
import matplotlib.pyplot as plt
from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.media import ThermodynamicState
from vclibpy.components.phase_separator import PhaseSeparator

logger = logging.getLogger(__name__)


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
            maximise_cop=False,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor_low = compressor_low
        self.compressor_high = compressor_high
        self.expansion_valve_low = expansion_valve_low
        self.expansion_valve_high = expansion_valve_high
        self.vi_pressure_fac = vi_pressure_fac
        self.maximise_cop = maximise_cop
        self.p_vi = None

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor_low,
            self.compressor_high,
            self.expansion_valve_low,
            self.expansion_valve_high
        ]

    def get_states(self):

        return {
            "1": self.compressor_low.state_inlet,
            "1_q1": self.med_prop.calc_state("PQ", self.compressor_low.state_inlet.p, 1),
            "2": self.compressor_low.state_outlet,
            "2_s": self.med_prop.calc_state("PS", self.compressor_low.state_outlet.p,
                                            self.compressor_low.state_inlet.s),
            "3": self.compressor_high.state_inlet,
            "4": self.compressor_high.state_outlet,
            "4s": self.med_prop.calc_state("PS", self.compressor_high.state_outlet.p,
                                           self.compressor_high.state_inlet.s),
            "4_q1": self.med_prop.calc_state("PQ", self.compressor_high.state_outlet.p, 1),
            "5_q0": self.med_prop.calc_state("PQ", self.compressor_high.state_outlet.p, 0),
            "5": self.condenser.state_outlet,
            "6": self.expansion_valve_high.state_outlet,
            "7": self.med_prop.calc_state("PQ", self.expansion_valve_high.state_outlet.p, 1),
            "8": self.med_prop.calc_state("PQ", self.expansion_valve_high.state_outlet.p, 0),
            "9": self.evaporator.state_inlet
        }

    def calc_steady_state(self, inputs: Inputs, fluid: str = None, **kwargs):
        """
        Calculate the steady-state performance of a vapor compression cycle
        based on given inputs and assumptions.

        This function ensures consistent assumptions across different cycles.
        It calculates the performance of the heat pump under
        specific conditions while adhering to several general assumptions.

        General Assumptions:
        ---------------------
        - Isenthalpic expansion valves:
          The enthalpy at the inlet equals the enthalpy at the outlet.
        - No heat losses in any component:
          The heat input to the condenser equals the heat
          output of the evaporator plus the power input.
        - Input to the evaporator is always in the two-phase region.
        - Output of the evaporator and output of the condenser maintain
          a constant overheating or subcooling (can be set in Inputs).

        Args:
            inputs (Inputs):
                An instance of the Inputs class containing the
                necessary parameters to calculate the flowsheet state.
            fluid (str):
                The fluid to be used in the calculations.
                Required only if 'fluid' is not specified during the object's initialization.

        Keyword Arguments:
            min_iteration_step (int):
                The minimum step size for iterations (default: 1).
            save_path_plots (str or None):
                The path to save plots (default: None).
                If None, no plots are created.
            show_iteration (bool):
                Whether to display iteration progress (default: False).
            T_max (float):
                Maximum temperature allowed (default: 273.15 + 150).
            use_quick_solver (bool):
                Whether to use a quick solver (default: True).
            max_err_ntu (float):
                Maximum allowable error for the heat exchanger in percent (default: 0.5).
            max_err_dT_min (float):
                Maximum allowable error for minimum temperature difference in K (default: 0.1).
            max_num_iterations (int or None):
                Maximum number of iterations allowed (default: None).

        Returns:
            fs_state (FlowsheetState):
                An instance of the FlowsheetState class representing
                the calculated state of the vapor compression cycle.
        """
        # Settings
        min_iteration_step = kwargs.pop("min_iteration_step", 1)
        save_path_plots = kwargs.get("save_path_plots", None)
        input_name = ";".join([k + "=" + str(np.round(v.value, 3)).replace(".", "_")
                               for k, v in inputs.get_variables().items()])
        show_iteration = kwargs.get("show_iteration", False)
        use_quick_solver = kwargs.pop("use_quick_solver", True)
        err_ntu = kwargs.pop("max_err_ntu", 0.001)
        err_dT_min = kwargs.pop("max_err_dT_min", 0.1)

        p_1_history = []
        p_2_history = []



        step_p1_counter = 0

        # Setup fluid:
        if fluid is None:
            fluid = self.fluid
        self.setup_new_fluid(fluid)

        # First: Iterate with given conditions to get the 4 states and the mass flow rate:
        T_1_start = inputs.T_eva_in - inputs.dT_eva_superheating
        T_3_start = inputs.T_con_in + inputs.dT_con_subcooling
        p_1_start = self.med_prop.calc_state("TQ", T_1_start, 1).p
        p_2_start = self.med_prop.calc_state("TQ", T_3_start, 0).p

        last_cop = 1
        step_pvi = 100000
        self.p_vi = p_1_start
        trigger = True
        while True:
            max_num_iterations = kwargs.pop("max_num_iterations", 1e6)
            if use_quick_solver:
                step_p1 = kwargs.get("step_max", 100000)
                step_p2 = kwargs.get("step_max", 100000)
            else:
                step_p1 = min_iteration_step
                step_p2 = min_iteration_step

            p_1_next = p_1_start
            p_2_next = p_2_start
            self.p_vi += step_pvi

            fs_state = FlowsheetState()  # Always log what is happening in the whole flowsheet
            fs_state.set(name="Q_con", value=1, unit="W", description="Condenser heat flow rate")
            fs_state.set(name="COP", value=0, unit="-", description="Coefficient of performance")

            if show_iteration:
                fig_iterations, ax_iterations = plt.subplots(2)

            num_iterations = 0

            while True:
                if isinstance(max_num_iterations, (int, float)):
                    if num_iterations > max_num_iterations:
                        logger.warning("Maximum number of iterations %s exceeded. Stopping.",
                                       max_num_iterations)
                        return

                    if (num_iterations + 1) % (0.1 * max_num_iterations) == 0:
                        logger.info("Info: %s percent of max_num_iterations %s used",
                                    100 * (num_iterations + 1) / max_num_iterations, max_num_iterations)

                p_1 = p_1_next
                p_2 = p_2_next
                if not self.maximise_cop:
                    self.p_vi = self.vi_pressure_fac * math.sqrt(p_1 * p_2)

                p_1_history.append(p_1)
                p_2_history.append(p_2)
                if show_iteration:
                    ax_iterations[0].cla()
                    ax_iterations[1].cla()
                    ax_iterations[0].scatter(list(range(len(p_1_history))), p_1_history)
                    ax_iterations[1].scatter(list(range(len(p_2_history))), p_2_history)
                    plt.draw()
                    plt.pause(1e-5)

                # Increase counter
                num_iterations += 1
                # Check critical pressures:
                if p_2 >= self._p_max:
                    if step_p2 == min_iteration_step:
                        logger.error("Pressure too high. Configuration is infeasible.")
                        return
                    p_2_next = p_2 - step_p2
                    step_p2 /= 10
                    continue
                if p_1 <= self._p_min:
                    if p_1_next == min_iteration_step:
                        logger.error("Pressure too low. Configuration is infeasible.")
                        return
                    p_1_next = p_1 + step_p1
                    step_p1 /= 10
                    continue

                # Calculate the states based on the given flowsheet
                try:
                    self.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
                except ValueError as err:
                    logger.error("An error occurred while calculating states. "
                                 "Can't guess next pressures, thus, exiting: %s", err)
                    return

                if save_path_plots is not None and num_iterations == 1 and show_iteration:
                    self.plot_cycle(save_path=save_path_plots.joinpath(f"{input_name}_initialization.png"), inputs=inputs)

                # Check heat exchangers:
                error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)
                if not isinstance(error_eva, float):
                    print(error_eva)

                if error_eva < 0:
                    p_1_next = p_1 - step_p1
                    step_p1_counter += 1
                    if step_p1_counter > 8:
                        if step_p1 < 1000:
                            step_p1 *= 10
                        step_p1_counter = 0
                    continue
                else:
                    if step_p1 > min_iteration_step:
                        p_1_next = p_1 + step_p1
                        step_p1 /= 10
                        continue
                    elif error_eva > err_ntu and dT_min_eva > err_dT_min:
                        step_p1 = 1000
                        p_1_next = p_1 + step_p1
                        continue

                error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)
                if error_con < 0:
                    p_2_next = p_2 + step_p2
                    continue
                else:
                    if step_p2 > min_iteration_step:
                        p_2_next = p_2 - step_p2
                        step_p2 /= 10
                        continue
                    elif error_con > err_ntu and dT_min_con > err_dT_min:
                        p_2_next = p_2 - step_p2
                        step_p2 = 1000
                        continue

                # If still here, and the values are equal, we may break.
                if p_1 == p_1_next and p_2 == p_2_next:
                    # Check if solution was too far away. If so, jump back
                    # And decrease the iteration step by factor 10.
                    if step_p2 > min_iteration_step:
                        p_2_next = p_2 - step_p2
                        step_p2 /= 10
                        continue
                    if step_p1 > min_iteration_step:
                        p_1_next = p_1 + step_p1
                        step_p1 /= 10
                        continue
                    logger.info("Breaking: Converged")
                    break

                # Check if values are not converging at all:
                p_1_unique = set(p_1_history[-10:])
                p_2_unique = set(p_2_history[-10:])
                if len(p_1_unique) == 2 and len(p_2_unique) == 2 \
                        and step_p1 == min_iteration_step and step_p2 == min_iteration_step:
                    logger.critical("Breaking: not converging at all")
                    break

            if show_iteration:
                plt.close(fig_iterations)

            if self.flowsheet_name == "IHX":
                self.calc_missing_IHX_states(inputs, fs_state, **kwargs)

            # Calculate the heat flow rates for the selected states.
            Q_con = self.condenser.calc_Q_flow()
            Q_con_outer = self.condenser.calc_secondary_Q_flow(Q_con)
            Q_eva = self.evaporator.calc_Q_flow()
            Q_eva_outer = self.evaporator.calc_secondary_Q_flow(Q_eva)
            self.evaporator.calc(inputs=inputs, fs_state=fs_state)
            self.condenser.calc(inputs=inputs, fs_state=fs_state)
            P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)
            T_con_out = inputs.T_con_in + Q_con_outer / self.condenser.m_flow_secondary_cp
            T_eva_out = inputs.T_eva_in - Q_eva_outer / self.evaporator.m_flow_secondary_cp

            # COP based on P_el and Q_con:
            COP_inner = Q_con / P_el
            COP_outer = Q_con_outer / P_el
            # Calculate carnot quality as a measure of reliability of model:
            COP_carnot = (T_con_out / (T_con_out - inputs.T_eva_in))
            carnot_quality = COP_inner / COP_carnot
            if not self.maximise_cop:
                break
            n_comp_low = self.compressor_low.calc_n(inputs, fs_state)
            n_comp_high = self.compressor_high.calc_n(inputs, fs_state)
            if n_comp_low - n_comp_high < 0:
                continue
            else:
                self.p_vi -= step_pvi
                step_pvi /= 10
                if step_pvi < 1:
                    break

            #trigger = False
            #if (COP_outer < last_cop):
                #self.p_vi -= step_pvi
                #step_pvi /= 10
                #if step_pvi < 10:
                    #break
                #continue
            last_cop = COP_outer

        fs_state.set(
            name="P_el", value=P_el, unit="W",
            description="Power consumption"
        )
        fs_state.set(
            name="carnot_quality", value=carnot_quality,
            unit="-", description="Carnot Quality"
        )
        fs_state.set(
            name="COP", value=COP_inner,
            unit="-", description="Coefficient of Performance"
        )
        fs_state.set(
            name="Q_con", value=Q_con, unit="W",
            description="Condenser refrigerant heat flow rate"
        )

        fs_state.set(name="SEC_T_con_in", value=inputs.T_con_in - 273.15,
                     description="Condenser inlet temperature secondary")
        fs_state.set(name="SEC_T_con_out", value=T_con_out - 273.15,
                     description="Condenser outlet temperature secondary")
        fs_state.set(name="SEC_m_flow_con", value=self.condenser.m_flow_secondary,
                     description="Condenser mass flow secondary")
        fs_state.set(name="SEC_T_eva_in", value=inputs.T_eva_in - 273.15,
                     description="Evaporator inlet temperature secondary")
        fs_state.set(name="SEC_T_eva_out", value=T_eva_out - 273.15,
                     description="Evaporator outlet temperature secondary")
        fs_state.set(name="SEC_m_flow_eva", value=self.evaporator.m_flow_secondary,
                     description="Evaporator mass flow secondary")
        fs_state.set(name="REF_m_flow_con", value=self.condenser.m_flow)
        fs_state.set(name="REF_m_flow_eva", value=self.evaporator.m_flow)
        if save_path_plots is not None:
            self.plot_cycle(save_path=save_path_plots.joinpath(f"{COP_inner}_final_result.png"), inputs=inputs)
        all_states = self.get_states()
        for _state in all_states:
            fs_state.set(name="REF_T_"+_state, value=all_states[_state].T-273.15)
        for _state in all_states:
            fs_state.set(name="REF_p_"+_state, value=all_states[_state].p/100000)
        for _state in all_states:
            fs_state.set(name="REF_h_"+_state, value=all_states[_state].h/1000)

        return fs_state


    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):

        p_vi = self.p_vi
        if self.maximise_cop:
            step_p_vi = 100000
        else:
            step_p_vi = 0

        while True:
            p_vi += step_p_vi
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
            n_high = self.compressor_high.calc_n(inputs, fs_state)
            if not self.maximise_cop:
                break
            if n_low > n_high:
                p_vi -= step_p_vi
                step_p_vi /=10
                if step_p_vi < 1:
                    break

        #if not self.maximise_cop:
            #break
        #current_cop = self.condenser.calc_Q_flow() / (self.condenser.calc_Q_flow() - self.evaporator.calc_Q_flow())
        #if current_cop < last_cop:
            #p_vi -= dp
            #dp /= 10
            #if dp < 1:
             #   break
            #next_vi_pressure_fac -= vi_pressure_fac_step
            #vi_pressure_fac_step /= 10
            #if vi_pressure_fac_step < 10:
                #break
            #continue
            #last_cop = current_cop

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

        fs_state.set(name="x_vapor_injection", value=x_vapor_injection, unit="-", description="VI ratio")
        fs_state.set(name="eta_is_low", value=self.compressor_low.get_eta_isentropic(p_vi, inputs), unit="1/s",
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

        fs_state.set(name="p_con", value=p_2 / 100000, unit="bar", description="Condensation pressure")
        fs_state.set(name="p_eva", value=p_1 / 100000, unit="bar", description="Evaporation pressure")
        fs_state.set(name="p_vi", value=p_vi / 100000, unit="bar", description="VI pressure")

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
