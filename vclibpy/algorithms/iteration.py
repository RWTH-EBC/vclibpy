import logging
from typing import Union

import numpy as np
import matplotlib.pyplot as plt

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
        max_err_dT_min (float):
            Maximum allowable error for minimum temperature difference in K (default: 0.1).
        max_num_iterations (int or None):
            Maximum number of iterations allowed (default: None).
        step_max (int):
            Maximum step in pressure to take, used for use_quick_solver=True.
            (default: 10000)
    """

    def __init__(self, **kwargs):
        """Initialize class with kwargs"""
        self.min_iteration_step = kwargs.pop("min_iteration_step", 1)
        self.show_iteration = kwargs.get("show_iteration", False)
        self.use_quick_solver = kwargs.pop("use_quick_solver", True)
        self.max_err_dT_min = kwargs.pop("max_err_dT_min", 0.1)
        self.max_num_iterations = kwargs.pop("max_num_iterations", int(1e5))
        self.step_max = kwargs.pop("step_max", 10000)
        super().__init__(**kwargs)

    def calc_steady_state(
            self,
            flowsheet: BaseCycle,
            inputs: Inputs,
            fluid: str = None
    ) -> Union[FlowsheetState, None]:
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
        error_con, dT_min_eva, dT_min_con, error_eva = np.nan, np.nan, np.nan, np.nan
        plot_last = -100

        # First: Iterate with given conditions to get the 4 states and the mass flow rate:
        if self.show_iteration:
            fig_iterations, ax_iterations = plt.subplots(3, 2, sharex=True)

        num_iterations = 0

        while True:
            if isinstance(self.max_num_iterations, (int, float)):
                if num_iterations > self.max_num_iterations:
                    logger.warning("Maximum number of iterations %s exceeded. Stopping.",
                                   self.max_num_iterations)
                    return

                if (num_iterations + 1) % (0.1 * self.max_num_iterations) == 0:
                    logger.info("Info: %s percent of max_num_iterations %s used",
                                100 * (num_iterations + 1) / self.max_num_iterations, self.max_num_iterations)

            p_1 = p_1_next
            p_2 = p_2_next
            p_1_history.append(p_1 / 1e5)
            p_2_history.append(p_2 / 1e5)
            error_con_history.append(error_con)
            error_eva_history.append(error_eva)
            dT_con_history.append(dT_min_con)
            dT_eva_history.append(dT_min_eva)

            if self.show_iteration:
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
            if p_2 >= _p_max:
                if step_p2 == self.min_iteration_step:
                    logger.error("Pressure too high. Configuration is infeasible.")
                    return
                p_2_next = p_2 - step_p2
                step_p2 /= 10
                continue
            if p_1 <= self._p_min:
                if p_1_next == self.min_iteration_step:
                    logger.error("Pressure too low. Configuration is infeasible.")
                    return
                p_1_next = p_1 + step_p1
                step_p1 /= 10
                continue

            try:
                error_eva, dT_min_eva, error_con, dT_min_con = flowsheet.calculate_cycle_for_pressures(
                    p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state
                )
            except ValueError as err:
                logger.error("An error occurred while calculating states. "
                             "Can't guess next pressures, thus, exiting: %s", err)
                return

            if num_iterations == 1:
                if self.save_path_plots is not None and self.show_iteration:
                    input_name = inputs.get_name()
                    flowsheet.plot_cycle(
                        save_path=self.save_path_plots.joinpath(f"{input_name}_initialization.png"),
                        inputs=inputs
                    )

            if not isinstance(error_eva, float):
                print(error_eva)
            if error_eva < 0:
                p_1_next = p_1 - step_p1
                continue
            else:
                if step_p1 > self.min_iteration_step:
                    p_1_next = p_1 + step_p1
                    step_p1 /= 10
                    continue
                elif error_eva > self.max_err and dT_min_eva > self.max_err_dT_min:
                    step_p1 = 1000
                    p_1_next = p_1 + step_p1
                    continue

            if error_con < 0:
                p_2_next = p_2 + step_p2
                continue
            else:
                if step_p2 > self.min_iteration_step:
                    p_2_next = p_2 - step_p2
                    step_p2 /= 10
                    continue
                elif error_con > self.max_err and dT_min_con > self.max_err_dT_min:
                    p_2_next = p_2 - step_p2
                    step_p2 = 1000
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

        return flowsheet.calculate_outputs_for_valid_pressures(
            p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state,
            save_path_plots=self.save_path_plots
        )
