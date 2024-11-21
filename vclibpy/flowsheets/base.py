import logging
from typing import List
import numpy as np
from scipy.optimize import fsolve

from abc import abstractmethod
import matplotlib.pyplot as plt
from vclibpy import media, Inputs
from vclibpy.datamodels import FlowsheetState
from vclibpy.components.heat_exchangers import HeatExchanger
from vclibpy.components.component import BaseComponent

logger = logging.getLogger(__name__)


class BaseCycle:
    """
    Base class for a heat pump. More complex systems may inherit from this class
    All HP have a compressor, two HE and a source and sink.
    Therefore, the parameters defined here are general parameters.

    Args:
        fluid (str): Name of the fluid
        evaporator (HeatExchanger): Instance of a heat exchanger used for the  evaporator
        condenser (HeatExchanger): Instance of a heat exchanger used for the condenser
     """

    flowsheet_name: str = "BaseCLass of all HP classes - not to use for map generation"

    def __init__(
            self,
            fluid: str,
            evaporator: HeatExchanger,
            condenser: HeatExchanger
    ):
        self.fluid: str = fluid
        self.evaporator = evaporator
        self.condenser = condenser
        # Instantiate dummy values
        self.med_prop = None
        self._p_min = 10000  # So that p>0 at all times
        self._p_max = None  # Is set by med-prop

    def __str__(self):
        return self.flowsheet_name

    def setup_new_fluid(self, fluid):
        # Only do so if new fluid is given
        if self.med_prop is not None:
            if self.med_prop.fluid_name == fluid:
                return
            self.med_prop.terminate()

        # Else create new instance of MedProp
        med_prop_class, med_prop_kwargs = media.get_global_med_prop_and_kwargs()
        self.med_prop = med_prop_class(fluid_name=fluid, **med_prop_kwargs)

        # Write the instance to the components
        for component in self.get_all_components():
            component.med_prop = self.med_prop
            component.start_secondary_med_prop()

        # Get max and min pressure
        _, self._p_max, _ = self.med_prop.get_critical_point()
        self.fluid = fluid

    def terminate(self):
        if self.med_prop is not None:
            self.med_prop.terminate()
            self.med_prop = None
        for component in self.get_all_components():
            component.terminate_secondary_med_prop()
            component.med_prop = None

    def get_all_components(self) -> List[BaseComponent]:
        return [self.condenser, self.evaporator]

    def get_start_condensing_pressure(self, inputs: Inputs):
        if inputs.uses_condenser_inlet:
            T_3_start = inputs.T_con_in + inputs.dT_con_subcooling
        else:
            try:
                dT_con_start = inputs.dT_con_start
            except AttributeError:
                dT_con_start = 10
            T_3_start = inputs.T_con_out - dT_con_start
        p_2_start = self.med_prop.calc_state("TQ", T_3_start, 0).p
        return p_2_start

    def improve_first_condensing_guess(self, inputs: Inputs, m_flow_guess, p_2_guess, dT_pinch_assumption=0):
        self.condenser.m_flow_secondary = inputs.m_flow_con  # [kg/s]
        self.condenser.calc_secondary_cp(T=inputs.T_con)

        def nonlinear_func(p_2, *args):
            _flowsheet, _inputs, _m_flow_ref, _dT_pinch = args
            state_q0 = _flowsheet.med_prop.calc_state("PQ", p_2, 0)
            state_q1 = _flowsheet.med_prop.calc_state("PQ", p_2, 1)
            state_3 = _flowsheet.med_prop.calc_state("PT", p_2, state_q0.T - _inputs.dT_con_subcooling)
            Q_water_till_q1 = (state_q1.h - state_3.h) * _m_flow_ref
            T_water_q1 = _inputs.T_con_in + Q_water_till_q1 / _flowsheet.condenser.m_flow_secondary_cp
            return T_water_q1 + _dT_pinch - state_q1.T

        p_2_guess_optimized = fsolve(
            func=nonlinear_func,
            x0=p_2_guess,
            args=(self, inputs, m_flow_guess, dT_pinch_assumption)
        )[0]
        return p_2_guess_optimized

    def get_start_evaporating_pressure(self, inputs: Inputs, dT_pinch: float = 0):
        T_1_start = inputs.T_eva_in - inputs.dT_eva_superheating - dT_pinch
        return self.med_prop.calc_state("TQ", T_1_start, 1).p

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
        show_iteration = kwargs.get("show_iteration", False)
        use_quick_solver = kwargs.pop("use_quick_solver", True)
        err_ntu = kwargs.pop("max_err_ntu", 0.5)
        err_dT_min = kwargs.pop("max_err_dT_min", 0.1)
        max_num_iterations = kwargs.pop("max_num_iterations", 1e5)
        dT_pinch_con_guess = kwargs.pop("dT_pinch_con_guess", 0)
        dT_pinch_eva_guess = kwargs.pop("dT_pinch_eva_guess", 0)
        improve_first_condensing_guess = kwargs.pop("improve_first_condensing_guess", False)
        p_1_history = []
        p_2_history = []
        error_con_history = []
        error_eva_history = []
        dT_eva_history = []
        dT_con_history = []
        error_con, dT_min_eva, dT_min_con, error_eva = np.nan, np.nan, np.nan, np.nan
        plot_last = -100

        if use_quick_solver:
            step_p1 = kwargs.get("step_max", 10000)
            step_p2 = kwargs.get("step_max", 10000)
        else:
            step_p1 = min_iteration_step
            step_p2 = min_iteration_step

        # Setup fluid:
        if fluid is None:
            fluid = self.fluid
        self.setup_new_fluid(fluid)

        # First: Iterate with given conditions to get the 4 states and the mass flow rate:
        p_1_next = self.get_start_evaporating_pressure(inputs=inputs, dT_pinch=dT_pinch_eva_guess)
        p_2_next = self.get_start_condensing_pressure(inputs=inputs)

        fs_state = FlowsheetState()  # Always log what is happening in the whole flowsheet
        fs_state.set(name="Q_con", value=1, unit="W", description="Condenser heat flow rate")
        fs_state.set(name="COP", value=0, unit="-", description="Coefficient of performance")

        if show_iteration:
            fig_iterations, ax_iterations = plt.subplots(3, 2, sharex=True)

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
            p_1_history.append(p_1 / 1e5)
            p_2_history.append(p_2 / 1e5)
            error_con_history.append(error_con)
            error_eva_history.append(error_eva)
            dT_con_history.append(dT_min_con)
            dT_eva_history.append(dT_min_eva)

            if show_iteration:
                for ax in ax_iterations.flatten():
                    ax.clear()
                iterations = list(range(len(p_1_history)))[plot_last:]
                ax_iterations[0, 0].set_ylabel("error_eva in %")
                ax_iterations[0, 1].set_ylabel("error_con in %")
                ax_iterations[1, 0].set_ylabel("$\Delta T_\mathrm{Min}$ in K")
                ax_iterations[1, 1].set_ylabel("$\Delta T_\mathrm{Min}$ in K")
                ax_iterations[2, 0].set_ylabel("$p_1$ in bar")
                ax_iterations[2, 1].set_ylabel("$p_2$ in bar")
                ax_iterations[0, 0].scatter(iterations, error_eva_history[plot_last:])
                ax_iterations[0, 1].scatter(iterations, error_con_history[plot_last:])
                ax_iterations[1, 0].scatter(iterations, dT_eva_history[plot_last:])
                ax_iterations[1, 1].scatter(iterations, dT_con_history[plot_last:])
                ax_iterations[2, 0].scatter(iterations, p_1_history[plot_last:])
                ax_iterations[2, 1].scatter(iterations, p_2_history[plot_last:])
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

            try:
                error_eva, dT_min_eva, error_con, dT_min_con = self.calculate_cycle_for_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state
                )
            except ValueError as err:
                logger.error("An error occurred while calculating states. "
                             "Can't guess next pressures, thus, exiting: %s", err)
                return

            if num_iterations == 1:
                if improve_first_condensing_guess:
                    p_2_next = self.improve_first_condensing_guess(
                       inputs=inputs,
                       m_flow_guess=self.condenser.m_flow,
                       p_2_guess=p_2,
                       dT_pinch_assumption=dT_pinch_con_guess
                    )

                if save_path_plots is not None and show_iteration:
                    input_name = inputs.get_name()
                    self.plot_cycle(
                        save_path=save_path_plots.joinpath(f"{input_name}_initialization.png"),
                        inputs=inputs
                    )

            if not isinstance(error_eva, float):
                print(error_eva)
            if error_eva < 0:
                p_1_next = p_1 - step_p1
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

        return self.calculate_outputs_for_valid_pressures(
            p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state,
            save_path_plots=save_path_plots
        )

    def calc_steady_state_fsolve(self, inputs: Inputs, fluid: str = None, **kwargs):
        # Settings
        max_err = kwargs.pop("max_err_ntu", 0.5)

        save_path_plots = kwargs.get("save_path_plots", None)
        dT_pinch_eva_guess = kwargs.pop("dT_pinch_eva_guess", 0)

        # Setup fluid:
        if fluid is None:
            fluid = self.fluid
        self.setup_new_fluid(fluid)

        # First: Iterate with given conditions to get the 4 states and the mass flow rate:
        p_1_next = self.get_start_evaporating_pressure(inputs=inputs, dT_pinch=dT_pinch_eva_guess)
        p_2_next = self.get_start_condensing_pressure(inputs=inputs)

        def nonlinear_func(x, *args):
            _flowsheet, _inputs, _fs_state, _max_err = args
            _p_1, _p_2 = x
            if not (_p_1 < _p_2 < self._p_max):
                return 1000, 1000
            if not (self._p_min < _p_1 < _p_2):
                return 1000, 1000
            _error_eva, dT_min_eva, _error_con, dT_min_con = _flowsheet.calculate_cycle_for_pressures(
                p_1=x[0], p_2=x[1], inputs=_inputs, fs_state=_fs_state
            )

            if 0 <= _error_eva < _max_err:
                _error_eva = 0
            if 0 <= _error_con < _max_err:
                _error_con = 0
            if 0 > _error_eva:
                _error_eva *= 5
            if 0 > _error_con:
                _error_con *= 5
            return _error_eva, _error_con

        fs_state = FlowsheetState()  # Always log what is happening in the whole flowsheet
        try:
            args = (self, inputs, fs_state, max_err)
            p_optimized = fsolve(
                func=nonlinear_func,
                x0=np.array([p_1_next, p_2_next]),
                args=args
            )
        except Exception as err:
            logger.error("An error occurred while calculating states using fsolve: %s", err)
            return
        error_con, error_eva = nonlinear_func(p_optimized, *args)
        print(f"{error_con=}, {error_eva=}")

        p_1, p_2 = p_optimized
        return self.calculate_outputs_for_valid_pressures(
            p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state,
            save_path_plots=save_path_plots
        )

    def calculate_cycle_for_pressures(self, p_1: float, p_2: float, inputs: Inputs, fs_state: FlowsheetState):
        # Calculate the states based on the given flowsheet
        self.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
        # Check heat exchangers:
        error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)
        error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)
        return error_eva, dT_min_eva, error_con, dT_min_con

    def calculate_outputs_for_valid_pressures(
            self,
            p_1,
            p_2,
            fs_state: FlowsheetState,
            inputs: Inputs,
            save_path_plots
    ):
        self.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
        # Calculate the heat flow rates for the selected states.
        Q_con = self.condenser.calc_Q_flow()
        Q_con_outer = self.condenser.calc_secondary_Q_flow(Q_con)
        Q_eva = self.evaporator.calc_Q_flow()
        Q_eva_outer = self.evaporator.calc_secondary_Q_flow(Q_eva)
        error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)
        error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)
        P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)
        if inputs.uses_condenser_inlet:
            T_con_in = inputs.T_con_in
            T_con_out = T_con_in + Q_con_outer / self.condenser.m_flow_secondary_cp
        else:
            T_con_out = inputs.T_con_out
            T_con_in = T_con_out - Q_con_outer / self.condenser.m_flow_secondary_cp

        # COP based on P_el and Q_con:
        COP_inner = Q_con / P_el
        COP_outer = Q_con_outer / P_el
        # Calculate carnot quality as a measure of reliability of model:
        COP_carnot = (T_con_out / (T_con_out - inputs.T_eva_in))
        carnot_quality = COP_inner / COP_carnot
        # Calc return temperature:
        if inputs.uses_condenser_inlet:
            fs_state.set(
                name="T_con_out", value=T_con_out, unit="K",
                description="Condenser outlet temperature"
            )
        else:
            fs_state.set(
                name="T_con_in", value=T_con_in, unit="K",
                description="Condenser inlet temperature"
            )

        fs_state.set(
            name="P_el", value=P_el, unit="W",
            description="Power consumption"
        )
        fs_state.set(
            name="carnot_quality", value=carnot_quality,
            unit="-", description="Carnot Quality"
        )
        fs_state.set(
            name="Q_con", value=Q_con, unit="W",
            description="Condenser refrigerant heat flow rate"
        )
        # COP based on P_el and Q_con:
        fs_state.set(
            name="Q_con_outer", value=Q_con_outer, unit="W",
            description="Secondary medium condenser heat flow rate"
        )
        fs_state.set(
            name="Q_eva_outer", value=Q_eva_outer, unit="W",
            description="Secondary medium evaporator heat flow rate"
        )
        fs_state.set(
            name="COP", value=COP_inner,
            unit="-", description="Coefficient of Performance"
        )
        fs_state.set(
            name="COP_outer", value=COP_outer,
            unit="-", description="Outer COP, including heat losses"
        )
        fs_state.set(
            name="error_con", value=error_con,
            unit="%", description="Error in condenser heat exchanger model"
        )
        fs_state.set(
            name="error_eva", value=error_eva,
            unit="%", description="Error in evaporator heat exchanger model"
        )
        fs_state.set(
            name="dT_min_eva", value=dT_min_eva,
            unit="K", description="Evaporator pinch temperature"
        )
        fs_state.set(
            name="dT_min_con", value=dT_min_con,
            unit="K", description="Condenser pinch temperature"
        )
        fs_state.set(
            name="eta_glob", value=fs_state.get("eta_is").value * fs_state.get("eta_mech").value,
            unit="%", description="Global compressor efficiency"
        )
        if save_path_plots is not None:
            input_name = inputs.get_name()
            self.plot_cycle(save_path=save_path_plots.joinpath(f"{input_name}_final_result.png"), inputs=inputs)

        return fs_state

    @abstractmethod
    def get_states_in_order_for_plotting(self):
        """
        Function to return all thermodynamic states of cycle
        in the correct order for plotting.
        Include phase change states to see if your simulation
        runs plausible cycles.

        Returns:
            - List with tuples, first entry being the state and second the mass flow rate
        """
        return []

    def set_evaporator_outlet_based_on_superheating(self, p_eva: float, inputs: Inputs):
        """
        Calculate the outlet state of the evaporator based on
        the required degree of superheating.

        Args:
            p_eva (float): Evaporation pressure
            inputs (Inputs): Inputs with superheating level
        """
        T_1 = self.med_prop.calc_state("PQ", p_eva, 1).T + inputs.dT_eva_superheating
        if inputs.dT_eva_superheating > 0:
            self.evaporator.state_outlet = self.med_prop.calc_state("PT", p_eva, T_1)
        else:
            self.evaporator.state_outlet = self.med_prop.calc_state("PQ", p_eva, 1)

    def set_condenser_outlet_based_on_subcooling(self, p_con: float, inputs: Inputs):
        """
        Calculate the outlet state of the evaporator based on
        the required degree of superheating.

        Args:
            p_con (float): Condensing pressure
            inputs (Inputs): Inputs with superheating level
        """
        T_3 = self.med_prop.calc_state("PQ", p_con, 0).T - inputs.dT_con_subcooling
        if inputs.dT_con_subcooling > 0:
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_con, T_3)
        else:
            self.condenser.state_outlet = self.med_prop.calc_state("PQ", p_con, 0)

    def plot_cycle(self, save_path: bool, inputs: Inputs, states: list = None):
        """Function to plot the resulting flowsheet of the steady state config."""
        if states is None:
            states = self.get_states_in_order_for_plotting()
            states.append(states[0])  # Plot full cycle
        # Unpack state var:
        h_T = np.array([state.h for state in states]) / 1000
        T = [state.T - 273.15 for state in states]
        p = np.array([state.p for state in states])
        h_p = h_T

        fig, ax = plt.subplots(2, 1, sharex=True)
        ax[0].set_ylabel("$T$ in °C")
        ax[1].set_xlabel("$h$ in kJ/kgK")
        # Two phase limits
        ax[0].plot(
            self.med_prop.get_two_phase_limits("h") / 1000,
            self.med_prop.get_two_phase_limits("T") - 273.15, color="black"
        )

        ax[0].plot(h_T, T, color="r", marker="s")
        self._plot_secondary_heat_flow_rates(ax=ax[0], inputs=inputs)
        ax[1].plot(h_p, np.log(p), marker="s", color="r")
        # Two phase limits
        ax[1].plot(
            self.med_prop.get_two_phase_limits("h") / 1000,
            np.log(self.med_prop.get_two_phase_limits("p")),
            color="black"
        )
        plt.plot()
        ax[1].set_ylabel("$log(p)$")
        ax[1].set_ylim([np.min(np.log(p)) * 0.9, np.max(np.log(p)) * 1.1])
        ax[0].set_ylim([np.min(T) - 5, np.max(T) + 5])
        ax[1].set_xlim([np.min(h_T) * 0.9, np.max(h_T) * 1.1])
        ax[0].set_xlim([np.min(h_T) * 0.9, np.max(h_T) * 1.1])
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_secondary_heat_flow_rates(self, ax, inputs):
        Q_con = self.condenser.calc_Q_flow()
        Q_eva = self.evaporator.calc_Q_flow()

        delta_H_con = np.array([
            self.condenser.state_outlet.h * self.condenser.m_flow,
            self.condenser.state_outlet.h * self.condenser.m_flow + Q_con
        ]) / self.condenser.m_flow
        delta_H_eva = np.array([
            self.evaporator.state_outlet.h * self.evaporator.m_flow,
            self.evaporator.state_outlet.h * self.evaporator.m_flow - Q_eva
        ]) / self.evaporator.m_flow
        self.condenser.m_flow_secondary = inputs.m_flow_con
        self.condenser.calc_secondary_cp(T=inputs.T_con)
        self.evaporator.m_flow_secondary = inputs.m_flow_eva
        self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
        if inputs.uses_condenser_inlet:
            ax.plot(delta_H_con / 1000, [
                inputs.T_con_in - 273.15,
                inputs.T_con_in + Q_con / self.condenser.m_flow_secondary_cp - 273.15
            ], color="b")
        else:
            ax.plot(delta_H_con / 1000, [
                inputs.T_con_out - Q_con / self.condenser.m_flow_secondary_cp - 273.15,
                inputs.T_con_out - 273.15
            ], color="b")
        ax.plot(delta_H_eva / 1000, [
            inputs.T_eva_in - 273.15,
            inputs.T_eva_in - Q_eva / self.evaporator.m_flow_secondary_cp - 273.15
        ], color="b")

    @abstractmethod
    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Function to calc the electrical power consumption based on the flowsheet used"""
        raise NotImplementedError

    @abstractmethod
    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        """
        Function to calculate the states and mass flow rates of the flowsheet
        and set these into each component based on the given pressure levels p_1 and p_2.

        Args:
            p_1 (float):
                Lower pressure level. If no pressure losses are assumed,
                this equals the evaporation pressure and the compressor inlet pressure.
            p_2 (float):
                Higher pressure level. If no pressure losses are assumed,
                this equals the condensing pressure and the compressor outlet pressure.
            inputs (Inputs): Inputs of calculation.
            fs_state (FlowsheetState): Flowsheet state to save important variables.
        """
        raise NotImplementedError
