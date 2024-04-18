import abc
import logging
import os

from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.heat_exchangers import HeatExchanger
import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)


class InternalHeatExchange(BaseCycle, abc.ABC):
    """
    Class for a ihx cycle with 6 components.

    For the ihx cycle, we have 6 possible states:

    1. Before compressor, after ihx_lt
    2. Before condenser, after compressor
    3. Before ihx_EV, after condenser
    4. Before ihx_ht, after ihx_EV
    5. Before EV, after ihx_ht
    6. Before Evaporator, after EV
    7. Before ihx_lt, after Evaporator
    """

    flowsheet_name = "InternalHeatExchange"

    def __init__(
            self,
            compressor: Compressor,
            expansion_valveIHX: ExpansionValve,  # Expansionsventil vor IHX
            expansion_valve: ExpansionValve,
            ihx: HeatExchanger,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valveIHX = expansion_valveIHX
        self.expansion_valve = expansion_valve
        self.ihx = ihx

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.expansion_valve,
            self.ihx
        ]

    def get_states_in_order_for_plotting(self):  # noch anpassen
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

    def set_evaporator_outlet_ihx(self, p_eva: float):
        """
        Calculate the outlet state of the evaporator in IHX Cycle (q=1)
        Args:
            p_eva (float): Evaporation pressure
        """
        self.evaporator.state_outlet = self.med_prop.calc_state("PQ", p_eva, 1)

    def set_ihx_outlet_based_on_superheating(self, p_eva: float, inputs: Inputs):
        """
        Calculate the outlet state of the ihx_lt based on
        the required degree of superheating.
        Args:
            p_eva (float): Evaporation pressure
            inputs (Inputs): Inputs with superheating level
        """
        T_1 = self.med_prop.calc_state("PQ", p_eva, 1).T + inputs.dT_eva_superheating
        if inputs.dT_eva_superheating > 0:
            self.ihx.state_outlet = self.med_prop.calc_state("PT", p_eva, T_1)
        else:
            self.ihx.state_outlet = self.med_prop.calc_state("PQ", p_eva, 1)

    def set_evaporator_inlet_IHX(self, p_eva: float):
        """
        Calculate the inlet state of the evaporator in ihx cycle based on
        the internal heat exchange
        Args:
            p_eva (float): Evaporation pressure
            inputs (Inputs): Inputs with superheating level
        """
        h_eva_in = self.condenser.state_outlet.h - (self.ihx.state_outlet.h - self.evaporator.state_outlet.h)
        self.evaporator.state_inlet = self.med_prop.calc_state("PH", p_eva, h_eva_in)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        """
        Calculate all required states of the ihx cycle for evaporating and
        condensing pressure iteration. calculation of the ihx-ht inlet and outlet state
        is implemented in "calc_steady_state"
        """
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)  # Subcooling nach Condenser, vor IHX
        self.expansion_valveIHX.state_inlet = self.condenser.state_outlet
        # self.expansion_valve.calc_outlet(p_outlet=p_1)
        self.set_evaporator_outlet_ihx(p_eva=p_1)
        self.ihx.state_inlet = self.evaporator.state_outlet
        self.set_ihx_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.ihx.state_outlet
        self.set_evaporator_inlet_IHX(p_eva=p_1)
        self.expansion_valve.state_outlet = self.evaporator.state_inlet

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
                Q_con = self.condenser.calc_Q_flow()
                rel_error = 100 * (Q_con - inputs.Q_con_set) / inputs.Q_con_set
                if abs(rel_error) < max_rel_error:
                    break
                elif rel_error < 0:
                    if n_next > 1.5:
                        n_next = 1.5
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
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet

        # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)

        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow
        self.ihx.m_flow = self.compressor.m_flow
        self.expansion_valveIHX.m_flow = self.compressor.m_flow

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

        inputs.set(
            name="Q_ihx",
            value=self.ihx.calc_Q_flow(),
            unit="W",
            description="IHX power"
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
            T_eva_out = inputs.T_eva_in - (inputs.Q_eva / (inputs.Q_eva * inputs.m_flow_eva))
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
            T_con_out = inputs.T_con_in + (inputs.Q_con / self.condenser.secondary_cp * inputs.m_flow_con)
            inputs.set(
                name="T_con_out",
                value=T_con_out,
                unit="K",
                description="Secondary side condenser outlet temperature"
            )

        # fs_state.set(
        #     name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
        #     unit="-", description="Expansion valve opening"
        # )
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

    def calc_steady_state(self, inputs: Inputs, fluid: str = None, **kwargs):
        """
        This function is modified for the ihx-cycle with two expansion valves.
        After the iteration of evaporating and condesing pressure, the pressure
        for the high temperature side of the ihx is determined.

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
        err_ntu = kwargs.pop("max_err_ntu", 0.5)
        err_dT_min = kwargs.pop("max_err_dT_min", 0.1)
        max_num_iterations = kwargs.pop("max_num_iterations", 1e5)
        p_1_history = []
        p_2_history = []

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
        T_1_start = inputs.T_eva_in - inputs.dT_eva_superheating
        T_3_start = inputs.T_con_in + inputs.dT_con_subcooling
        p_1_start = self.med_prop.calc_state("TQ", T_1_start, 1).p
        p_2_start = self.med_prop.calc_state("TQ", T_3_start, 0).p
        p_1_next = p_1_start
        p_2_next = p_2_start

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

        # Interal Heat Exchanger - Pressure Iteration

        p_ihx_next = p_2
        step_p_ihx = 10000
        num_iterations = 0
        p_ihx_history = []
        firstIteration = True
        success = True

        while True:
            if isinstance(max_num_iterations, (int, float)):
                if num_iterations > max_num_iterations:
                    logger.warning("Maximum number of iterations %s exceeded. Stopping.",
                                   max_num_iterations)
                    return

                if (num_iterations + 1) % (0.1 * max_num_iterations) == 0:
                    logger.info("Info: %s percent of max_num_iterations %s used",
                                100 * (num_iterations + 1) / max_num_iterations, max_num_iterations)

            p_ihx = p_ihx_next
            p_ihx_history.append(p_ihx)

            self.expansion_valveIHX.state_inlet = self.condenser.state_outlet
            self.expansion_valveIHX.state_outlet = self.med_prop.calc_state("PH", p_ihx,
                                                                            self.expansion_valveIHX.state_inlet.h)
            self.ihx.state_inlet_ht = self.expansion_valveIHX.state_outlet
            h_outlet_lt = self.ihx.state_inlet_ht.h - (self.ihx.state_outlet.h - self.ihx.state_inlet.h)
            self.ihx.state_outlet_ht = self.med_prop.calc_state("PH", p_ihx, h_outlet_lt)

            # Check IHX
            error_ihx, dT_min_ihx = self.ihx.calc(inputs=inputs, fs_state=fs_state)
            # starting pressure is p_2, so the pressure cannot be increased
            if firstIteration:
                firstIteration = False
                if error_ihx < 0:
                    success = False
                    logger.critical("Breaking: IXH-pressure is higher than pressure after condenser")
                    break

            if error_ihx > 0:
                p_ihx_next = p_ihx - step_p_ihx
                continue
            else:
                if step_p_ihx > min_iteration_step:
                    p_ihx_next = p_ihx + step_p_ihx
                    step_p_ihx /= 10
                    continue
                elif error_ihx > err_ntu and dT_min_ihx > err_dT_min:
                    step_p_ihx = 1000
                    p_ihx_next = p_ihx + step_p_ihx
                    continue

            if p_ihx == p_ihx_next:
                # Check if solution was too far away. If so, jump back
                # And decrease the iteration step by factor 10.
                if step_p_ihx > min_iteration_step:
                    p_ihx_next = p_ihx - step_p_ihx
                    step_p_ihx /= 10
                    continue
                logger.info("Breaking: Converged")
                if p_ihx < p_1:
                    logger.critical("Breaking: IHX-pressure is lower than evaporating pressure")
                break

        # iterations = list(range(len(p_ihx_history)))
        # plt.ylabel("$p$ in Pa")
        # plt.xlabel("$No$ in -")
        # plt.plot(iterations, p_ihx_history, marker=".", color="red")
        # savefolder = os.getcwd() + "\\iterations"
        # if not os.path.exists(savefolder):
        #     os.mkdir(savefolder)
        # savename = savefolder +"\\iteration_" + str(int(p_ihx)) + ".png"
        # plt.savefig(savename)
        # plt.close()
        # state_6 = self.evaporator.state_outlet
        # state_1 = self.ihx.state_outlet
        # state_2 = self.compressor.state_outlet
        # state_3 = self.condenser.state_outlet
        # state_4 = self.ihx.state_outlet_ht
        # state_5 = self.evaporator.state_inlet
        #
        # states_show = [state_1, state_2, state_3, state_4, state_5, state_6]
        # count = 1
        # for state in states_show:
        #     print("State " + str(count) + " - " + str(state.h) + "  J/kgK")
        #     count += 1
        # Set States for expansion Valves

        # self.expansion_valveIHX.calc_outlet(p_ihx)
        self.expansion_valve.state_inlet = self.ihx.state_outlet_ht
        self.expansion_valve.calc_outlet(p_1)

        # Calculate the heat flow rates for the selected states.
        Q_con = self.condenser.calc_Q_flow()
        Q_con_outer = self.condenser.calc_secondary_Q_flow(Q_con)
        Q_eva = self.evaporator.calc_Q_flow()
        Q_eva_outer = self.evaporator.calc_secondary_Q_flow(Q_eva)
        self.evaporator.calc(inputs=inputs, fs_state=fs_state)
        self.condenser.calc(inputs=inputs, fs_state=fs_state)
        P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)
        T_con_out = inputs.T_con_in + Q_con_outer / self.condenser.m_flow_secondary_cp

        # COP based on P_el and Q_con:
        COP_inner = Q_con / P_el
        COP_outer = Q_con_outer / P_el
        if not success:
            COP_inner = 0
            COP_outer = 0
        # Calculate carnot quality as a measure of reliability of model:
        COP_carnot = (T_con_out / (T_con_out - inputs.T_eva_in))
        carnot_quality = COP_inner / COP_carnot

        fs_state.set(name="p_ihx", value=p_ihx, unit="Pa", description="Evaporation pressure")

        fs_state.set(
            name="y_EV", value=self.expansion_valve.calc_opening_at_m_flow(m_flow=self.expansion_valve.m_flow),
            unit="-", description="Expansion valve opening"
        )
        if success:
            fs_state.set(
                name="y_EV_IHX", value=self.expansion_valveIHX.calc_opening_at_m_flow(m_flow=self.expansion_valveIHX.m_flow),
                unit="-", description="Expansion valve opening (IHX)"
            )
        else:
            fs_state.set(
                name="y_EV_IHX",
                value=0,
                unit="-", description="Expansion valve opening (IHX)"
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

        if save_path_plots is not None:
            self.plot_cycle(save_path=save_path_plots.joinpath(f"{COP_inner}_final_result.png"), inputs=inputs)

        return fs_state
