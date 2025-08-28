import logging
from typing import Union

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

from vclibpy import Inputs, FlowsheetState
from vclibpy.algorithms.base import Algorithm
from vclibpy.flowsheets import BaseCycle

logger = logging.getLogger(__name__)


class Iteration(Algorithm):
    """
    Algorithm to calculate steady states with an iteration based approach.

    Args:
        min_iteration_step (int):
            The minimum step size for iterations (default: 1).
        show_iteration (bool):
            Whether to display iteration progress (default: False).
        use_quick_solver (bool):
            Whether to use a quick solver (default: True).
        min_allowed_dT_min (float):
            Allowable minimum temperature difference in K (default: 0.01).
            This setting is relevant in combination with `max_err`.
            If the error is bigger than the allowed error but the temperature difference
            is lower than this value, the pressure won't be adjusted further.
            If the heat exchanger is too big in some points the error
            values will be larger than `max_err`, but if the temperature difference
            approaches zero the area has no impact, as dT approaches zero.
        max_num_iterations (int or None):
            Maximum number of iterations allowed (default: None).
        step_max (int):
            Maximum step in pressure to take, used for use_quick_solver=True.
            (default: 10000)
    """

    def __init__(self, **kwargs):
        """Initialize class with kwargs"""
        self.min_iteration_step = kwargs.pop("min_iteration_step", 10)
        self.show_iteration = kwargs.get("show_iteration", False)
        self.use_quick_solver = kwargs.pop("use_quick_solver", True)
        self.max_err_dT_min = kwargs.pop("min_allowed_dT_min", 0.01)
        self.max_num_iterations = kwargs.pop("max_num_iterations", int(1e5))
        self.step_max = kwargs.pop("step_max", 10000)
        self.return_least_error_if_max_reached = kwargs.pop("return_least_error_if_max_reached", False)
        super().__init__(**kwargs)

    def calc_steady_state(
            self,
            flowsheet: BaseCycle,
            inputs: Inputs,
            fluid: str = None
    ) -> Union[FlowsheetState, None]:
        flowsheet.iteration_converged = False

        p_1_next, p_2_next, _p_max, fs_state = self.initial_setup(
            flowsheet=flowsheet, inputs=inputs, fluid=fluid
        )

        # Settings
        if self.use_quick_solver:
            step_p1 = self.step_max
            step_p2 = self.step_max
        else:
            step_p1 = self.min_iteration_step
            step_p2 = self.min_iteration_step

        p_1_history = []
        p_2_history = []
        error_con_history = []
        error_eva_history = []
        dT_eva_history = []
        dT_con_history = []
        p_1, p_2, error_con, dT_min_eva, dT_min_con, error_eva = np.nan, np.nan, np.nan, np.nan, np.nan, np.nan
        plot_last = -100
        if flowsheet.flowsheet_name == "StandardTranscritical":
            cop_history = []
            current_cop = np.nan
            differential_history = []
            num_p_2_iterations = 1

        # First: Iterate with given conditions to get the 4 states and the mass flow rate:
        if self.show_iteration:
            fig_iterations, ax_iterations = plt.subplots(3, 2, sharex=True)

        num_iterations = 0

        while True:
            if isinstance(self.max_num_iterations, (int, float)):
                if num_iterations > self.max_num_iterations:
                    logger.warning("Maximum number of iterations %s exceeded. Stopping.",
                                   self.max_num_iterations)
                    if not self.return_least_error_if_max_reached:
                        return
                    # Try to find the best available solution
                    df_history = pd.DataFrame({
                        "p_1": p_1_history,
                        "p_2": p_2_history,
                        "error_con": error_con_history,
                        "error_eva": error_eva_history,
                        "dT_con": dT_con_history,
                        "dT_eva": dT_eva_history,
                    })
                    # No errors left
                    df_history = df_history.loc[(df_history["error_con"] > 0) & (df_history["error_eva"] > 0)]
                    if df_history.empty:
                        logger.error("No iteration yielded pressures where heat transfer would be possible.")
                        return
                        # Take the pressures with the least error
                    least_error_arg = (df_history["error_con"] + df_history["error_eva"]).argmin()
                    fs_state.set(
                        name="converged", value=0, unit="-",
                        description="Algorithm converged (1 true, 0 false)"
                    )
                    return flowsheet.calculate_outputs_for_valid_pressures(
                        p_1=df_history.iloc[least_error_arg]["p_1"] * 1e5,
                        p_2=df_history.iloc[least_error_arg]["p_2"] * 1e5,
                        inputs=inputs, fs_state=fs_state,
                        save_path_plots=self.save_path_plots
                    )

                if (num_iterations + 1) % (0.1 * self.max_num_iterations) == 0:
                    logger.info("Info: %s percent of max_num_iterations %s used",
                                100 * (num_iterations + 1) / self.max_num_iterations, self.max_num_iterations)

            p_1_history.append(p_1 / 1e5)
            p_2_history.append(p_2 / 1e5)
            p_1 = p_1_next
            p_2 = p_2_next
            error_con_history.append(error_con)
            error_eva_history.append(error_eva)
            dT_con_history.append(dT_min_con)
            dT_eva_history.append(dT_min_eva)
            if flowsheet.flowsheet_name == "StandardTranscritical":
                cop_history.append(current_cop)

            if self.show_iteration:
                for ax in ax_iterations.flatten():
                    ax.clear()
                iterations = list(range(len(p_1_history)))[plot_last:]
                ax_iterations[0, 0].set_ylabel("error_eva in %")
                ax_iterations[0, 1].set_ylabel("cop in -")
                ax_iterations[1, 0].set_ylabel("$\Delta T_\mathrm{Min}$ in K")
                ax_iterations[1, 1].set_ylabel("$\Delta T_\mathrm{Min}$ in K")
                ax_iterations[2, 0].set_ylabel("$p_1$ in bar")
                ax_iterations[2, 1].set_ylabel("$p_2$ in bar")
                ax_iterations[0, 0].scatter(iterations, error_eva_history[plot_last:])
                ax_iterations[0, 1].scatter(iterations, cop_history[plot_last:])
                ax_iterations[1, 0].scatter(iterations, dT_eva_history[plot_last:])
                ax_iterations[1, 1].scatter(iterations, dT_con_history[plot_last:])
                ax_iterations[2, 0].scatter(iterations, p_1_history[plot_last:])
                ax_iterations[2, 1].scatter(iterations, p_2_history[plot_last:])
                plt.pause(1e-3)
                plt.draw()
                #plt.pause(1e-4)

            # Increase counter
            num_iterations += 1
            # Check critical pressures:
            if flowsheet.flowsheet_name != "StandardTranscritical" and p_2 >= _p_max:
                if step_p2 == self.min_iteration_step:
                    logger.error("Pressure too high. Inputs %s are infeasible.", inputs.get_name())
                    return
                p_2_next = p_2 - step_p2
                step_p2 /= 10
                continue
            if p_1 <= self._p_min:
                if p_1_next == self.min_iteration_step:
                    logger.error("Pressure too low. Inputs %s are infeasible.", inputs.get_name())
                    return
                p_1_next = p_1 + step_p1
                step_p1 /= 10
                continue

            try:
                error_eva, dT_min_eva, error_con, dT_min_con = flowsheet.calculate_cycle_for_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state
                )
            except ValueError as err:
                logger.error(
                    "An error occurred while calculating states. "
                    "Can't guess next pressures for inputs %s, thus, exiting: %s",
                    inputs.get_name(), err
                )
                if self.raise_errors:
                    raise err
                return

            Q_con = flowsheet.condenser.calc_Q_flow()
            P_el = flowsheet.calc_electrical_power(fs_state=fs_state, inputs=inputs)
            current_cop = Q_con / P_el

            if num_iterations == 1:
                if self.save_path_plots is not None and self.show_iteration:
                    input_name = inputs.get_name()
                    flowsheet.plot_cycle(
                        save_path=self.save_path_plots.joinpath(f"{input_name}_initialization.png"),
                        inputs=inputs
                    )

            if not isinstance(error_eva, float):
                print(error_eva)

            # print(f"Iteration {num_iterations}: p_1 = {p_1}, p_2 = {p_2}, error_eva = {error_eva}, error_con = {error_con}")

            # Check if last pressure adjustment caused a change of sign in error
            # If so, we went over the optimum and need to decrease the step size
            if (np.sign(error_eva) != np.sign(error_eva_history[-1])
                    and num_iterations > 1
                    and not np.isclose(p_1, p_1_history[-2])):
                step_p1 /= 10
                # continue
            if flowsheet.flowsheet_name != "StandardTranscritical" and (np.sign(error_con) != np.sign(error_con_history[-1])
                    and num_iterations > 1
                    and not np.isclose(p_2, p_2_history[-2]))\
                    and not step_p2 <= 0.5 * step_p1:
                step_p2 /= 10
                # continue

            # Check which heat exchanger has the bigger error. The corresponding pressure gets updated
            # if abs(error_eva) > abs(error_con):
            if error_eva < 0 and step_p1 > self.min_iteration_step:
                p_1_next = p_1 - step_p1
                num_p_2_iterations = 1
                step_p2 = self.step_max
                continue
            elif error_eva < -1*self.max_err and dT_min_eva > self.max_err_dT_min:
                step_p1 = 1000
                p_1_next = p_1 - step_p1
                num_p_2_iterations = 1
                step_p2 = self.step_max
                continue
            else:
                if step_p1 > self.min_iteration_step:
                    p_1_next = p_1 + step_p1
                    num_p_2_iterations = 1
                    step_p2 = self.step_max
                    continue
                elif error_eva > self.max_err and dT_min_eva > self.max_err_dT_min:
                    step_p1 = 1000
                    p_1_next = p_1 + step_p1
                    num_p_2_iterations = 1
                    step_p2 = self.step_max
                    continue
            # else:
            if flowsheet.flowsheet_name != "StandardTranscritical":  # "!=" meaning "unequal"
                if error_con < 0:
                    p_2_next = p_2 + step_p2
                    continue
                else:
                    if step_p2 > self.min_iteration_step:
                        p_2_next = p_2 - step_p2
                        continue
                    elif error_con > self.max_err and dT_min_con > self.max_err_dT_min:
                        p_2_next = p_2 - step_p2
                        step_p2 = 1000
                        continue
            else:
                if num_p_2_iterations == 1:
                    # print(f"Iteration: 1, COP: {current_cop}")
                    p_2_next = p_2 - step_p2
                    num_p_2_iterations += 1
                    continue
                else:
                    # print(f"Iteration: {num_p_2_iterations}, COP: {current_cop}")
                    local_differential = (current_cop - cop_history[-1]) / (p_2*1e-5 - p_2_history[-1])     #ToDO: COP calculation needs to take change of p_1 into account. Therefore error_eva should be iterated before this
                    print(f"Iteration: {num_p_2_iterations}, current_cop: {current_cop}, last_cop: {cop_history[-1]}, current_p_2: {p_2*1e-5}, last_p_2: {p_2_history[-1]}, local_differential: {local_differential}")
                    differential_history.append(local_differential)
                    if num_p_2_iterations > 2 and np.sign(differential_history[-1]) != np.sign(differential_history[-2]):
                        print("sign change in local_differential. Decreasing step_p2")
                        step_p2 /= 10
                    num_p_2_iterations += 1
                    if step_p2 >= self.min_iteration_step:
                        if local_differential > 0:
                            # print("local_differential: ", local_differential, "increasing pressure")
                            p_2_next = p_2 + step_p2
                            continue
                        else:
                            # print("local_differential: ", local_differential, "decreasing pressure")
                            p_2_next = p_2 - step_p2
                            continue

            # If still here, and the values are equal, we may break.
            if p_1 == p_1_next and p_2 == p_2_next:
                # Check if solution was too far away. If so, jump back
                # And decrease the iteration step by factor 10.
                if step_p2 > self.min_iteration_step:
                    p_2_next = p_2 - step_p2
                    step_p2 /= 10
                    continue
                if step_p1 > self.min_iteration_step:
                    p_1_next = p_1 + step_p1
                    step_p1 /= 10
                    continue
            logger.info("Breaking: Converged")
            break

            # Check if values are not converging at all:
            p_1_unique = set(p_1_history[-10:])
            p_2_unique = set(p_2_history[-10:])
            if len(p_1_unique) == 2 and len(p_2_unique) == 2 \
                    and step_p1 == self.min_iteration_step and step_p2 == self.min_iteration_step:
                logger.critical("Breaking: not converging at all")
                break

        if self.show_iteration:
            plt.close(fig_iterations)

        flowsheet.iteration_converged = True
        fs_state.set(name="converged", value=1, unit="-", description="Algorithm converged (1 true, 0 false)")
        final_fs_state = flowsheet.calculate_outputs_for_valid_pressures(
            p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state,
            save_path_plots=self.save_path_plots
        )

        p_1_history.append(p_1 / 1e5)
        p_2_history.append(p_2 / 1e5)
        cop_history.append(final_fs_state.get("COP").value)
        error_con_history.append(final_fs_state.get("error_con").value)
        error_eva_history.append(final_fs_state.get("error_eva").value)
        dT_con_history.append(final_fs_state.get("dT_min_con").value)
        dT_eva_history.append(final_fs_state.get("dT_min_eva").value)

        if self.save_path_plots is not None:
            pd.DataFrame({
                "p_1": p_1_history,
                "p_2": p_2_history,
                "error_con": error_con_history,
                "error_eva": error_eva_history,
                "dT_con": dT_con_history,
                "dT_eva": dT_eva_history,
                "cop": cop_history,
            }).to_excel(self.save_path_plots.joinpath(f"{inputs.get_name()}.xlsx"))

        return final_fs_state

