import logging
import os.path
from typing import List
import numpy as np
import time
from copy import deepcopy
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
            condenser: HeatExchanger,
            T2_max = np.inf,
            T_con_out_max = np.inf,
    ):
        self.fluid: str = fluid
        self.evaporator = evaporator
        self.condenser = condenser
        # Instantiate dummy values
        self.med_prop = None
        self._p_min = 0.1 * 1e5  # So that p>0 at all times
        self._p_max = None  # Is set by med-prop
        self.T2_max = T2_max
        self.T_con_out_max = T_con_out_max

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
        for component in self.get_all_components():
            component.terminate_secondary_med_prop()

    def get_all_components(self) -> List[BaseComponent]:
        return [self.condenser, self.evaporator]

    def calc_steady_state(self, inputs: Inputs, fluid: str = None, **kwargs):
        start_time_warning = time.time()
        start_time = time.time()

        min_iteration_step = kwargs.pop("min_iteration_step", 0.001)
        save_path_plots = kwargs.get("save_path_plots", None)
        err_ntu = kwargs.pop("max_err_ntu", 0.1)

        # Setup fluid:
        if fluid is None:
            fluid = self.fluid
        self.setup_new_fluid(fluid)


        fs_state = self.set_default_state(inputs,start_time)  # Always log what is happening in the whole flowsheet

        num_iterations = 0
        Tc, pc, dc = self.med_prop.get_critical_point()
        ### Start Temperature Iteration ####

        if self.flowsheet_name == "IHX":
            T_eva_start = inputs.T_eva_in
        else:
            T_eva_start = inputs.T_eva_in - inputs.dT_eva_superheating
        T_con_start = inputs.T_con_in + inputs.dT_con_subcooling

        n_min_tried = False
        n_min_try = False
        n_input, n_next = 0, 0
        n_max = 100
        if inputs.fix_speed == float(True):
            n_input = deepcopy(inputs.n)
            n_next = inputs.n
            n_max = 100 *n_input/inputs.n_rel
        history_inputs = []
        while True:
            if inputs.fix_speed == float(True):
                inputs.set(
                    name="n",
                    value=n_next,
                    unit="-",
                    description="Relative compressor speed"
                )
                fs_state.set(name="relative_compressor_speed_internal", value=n_next)
            T_con_next = T_con_start
            step_T_con = 1
            adjust_n = False
            temp_num_iteration = 0
            while True:
                T_eva_next = T_eva_start
                step_T_eva = 1
                while True:
                    num_iterations += 1
                    temp_num_iteration +=1

                    if inputs.fix_speed == float(True) and not n_min_try:
                        if [n_next, T_con_next, T_eva_next] in history_inputs:
                            return self.set_default_state(inputs, start_time, "LoopError")
                        history_inputs.append([n_next, T_con_next, T_eva_next])

                    if (time.time() - start_time_warning) > 60:
                        logger.error("RunTimeWarning")
                        start_time_warning = time.time()

                    if time.time() - start_time > 90:
                        logger.error("RunTimeError")
                        return self.set_default_state(inputs, start_time, "RunTimeError")

                    if T_con_next > Tc - 10:
                        if inputs.fix_speed == float(True):
                            adjust_n = True
                            break
                        else:
                            return self.set_fs_state_to_off(inputs, start_time, "Maximal Pressure reached")

                    p_2 = self.med_prop.calc_state("TQ", T_con_next, 0).p
                    p_1 = self.med_prop.calc_state("TQ", T_eva_next, 0).p

                    if p_1 < 0.01 *10**5:
                        if inputs.fix_speed == float(True):
                            adjust_n = True
                            break

                        else:
                            return self.set_fs_state_to_off(inputs, comment="Min Pressure reached", start_time=start_time)

                    try:
                        valid = self.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
                    except ValueError as err:
                        logger.error("An error occurred while calculating states. "
                                     "Can't guess next pressures, thus, exiting: %s", err)
                        return self.set_default_state(inputs, start_time, "State Calculation Error")

                    if valid is not None:
                        T_con_next += 1
                        continue

                    try:
                        error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)
                    except:
                        logger.error("An error occurred while calculating evaporator.")
                        return self.set_default_state(inputs, start_time, "Evaporator Error")

                    if dT_min_eva < 0:
                        T_eva_next -= step_T_eva
                        continue
                    if abs(error_eva) < err_ntu:
                        break
                    if error_eva < 0:
                        T_eva_next -= step_T_eva
                        continue
                    if error_eva > 0:
                        if dT_min_eva < 0.1*min_iteration_step:
                            break
                        T_eva_next += step_T_eva
                        step_T_eva /= 10
                        if step_T_eva < min_iteration_step:
                            break
                        T_eva_next -= step_T_eva
                        T_eva_next = min(T_eva_next,T_eva_start)
                        continue

                if inputs.fix_speed == float(True) and adjust_n:
                    if n_min_try:
                        return self.set_fs_state_to_off(inputs, comment="Min Compressor Speed reached", start_time=start_time)
                    break
                try:
                    error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)

                except:
                    logger.error(f"An error occurred while calculating condenser.")
                    return self.set_default_state(inputs, start_time,"Condenser Error")

                if dT_min_con < 0:
                    T_con_next += step_T_con
                    continue
                if abs(error_con) < err_ntu:
                    break
                if error_con < 0:
                    T_con_next += step_T_con
                    continue
                if error_con > 0:
                    if dT_min_con < 0.1*min_iteration_step:
                        break
                    T_con_next -= step_T_con
                    step_T_con /= 10
                    T_con_next += step_T_con
                    T_con_next = max(T_con_next, T_con_start)
                    if step_T_con < min_iteration_step:
                        break
                    continue

            if inputs.fix_speed == float(False):
                break
            if not adjust_n:
                if self.condenser.state_inlet.T <= self.T2_max and inputs.T_con_out <= self.T_con_out_max:
                    if n_min_try:
                        n_next = deepcopy(n_input)
                        n_next -= 0.25 * n_max
                        if n_next < 0.2:
                            inputs.set(
                                name="n",
                                value=n_input,
                                unit="-",
                                description="Relative compressor speed"
                            )
                            return self.set_fs_state_to_off(inputs, comment="Min Compressor Speed reached",
                                                            start_time=start_time)
                        n_min_try = False
                        n_min_tried = True
                        continue
                    break
                else:
                    if n_min_try:
                        inputs.set(
                            name="n",
                            value=n_input,
                            unit="-",
                            description="Relative compressor speed"
                        )
                        return self.set_fs_state_to_off(inputs, comment="Min Compressor Speed reached", start_time=start_time)
            if not n_min_tried:
                n_next = 0.2
                n_min_try = True
                continue
            n_next -= 0.25 * n_max
            if n_next < 0.2:
                inputs.set(
                    name="n",
                    value=n_input,
                    unit="-",
                    description="Relative compressor speed"
                )
                return self.set_fs_state_to_off(inputs, comment="Min Compressor Speed reached", start_time=start_time)
            continue
        if inputs.fix_speed == float(True):
            inputs.set(
                name="n",
                value=n_input,
                unit="-",
                description="Relative compressor speed"
            )
            fs_state.set(name="relative_compressor_speed", value=n_input)

        if self.flowsheet_name == "IHX":
            self.calc_missing_IHX_states(inputs, fs_state, **kwargs)

        # Calculate the heat flow rates for the selected states.
        Q_con = self.condenser.calc_Q_flow()
        Q_eva = self.evaporator.calc_Q_flow()
        self.evaporator.calc(inputs=inputs, fs_state=fs_state)
        self.condenser.calc(inputs=inputs, fs_state=fs_state)
        P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)

        # COP based on P_el and Q_con:
        COP_inner = Q_con / P_el
        # Calculate carnot quality as a measure of reliability of model:
        COP_carnot = (inputs.T_con_out / (inputs.T_con_out - inputs.T_eva_in))
        carnot_quality = COP_inner / COP_carnot
        fs_state.set("ErrorCon", value=error_con)
        fs_state.set("ErrorEva", value=error_eva)
        fs_state.set(
            name="P_el", value=P_el / 1000, unit="W",
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
        fs_state.set(name="COP_Carnot", value=COP_carnot,
                     unit="-", description="maximal Coefficient of performance")
        fs_state.set(
            name="Q_con", value=Q_con / 1000, unit="W",
            description="Condenser refrigerant heat flow rate"
        )
        fs_state.set(
            name="Q_eva", value=Q_eva / 1000, unit="W",
            description="Evaporator refrigerant heat flow rate"
        )

        fs_state.set(name="SEC_T_con_in", value=inputs.T_con_in - 273.15,
                     description="Condenser inlet temperature secondary")
        fs_state.set(name="SEC_T_con_out", value=inputs.T_con_out - 273.15,
                     description="Condenser outlet temperature secondary")
        fs_state.set(name="SEC_dT_con", value=inputs.T_con_out - inputs.T_con_in,
                     description="Condenser temperature difference secondary")
        fs_state.set(name="SEC_m_flow_con", value=self.condenser.m_flow_secondary,
                     description="Condenser mass flow secondary")
        fs_state.set(name="SEC_T_eva_in", value=inputs.T_eva_in - 273.15,
                     description="Evaporator inlet temperature secondary")
        fs_state.set(name="SEC_T_eva_out", value=inputs.T_eva_out - 273.15,
                     description="Evaporator outlet temperature secondary")
        fs_state.set(name="SEC_dT_eva", value=inputs.T_eva_out - inputs.T_eva_out,
                     description="Evaporator temperature difference secondary")
        fs_state.set(name="SEC_m_flow_eva", value=self.evaporator.m_flow_secondary,
                     description="Evaporator mass flow secondary")
        fs_state.set(name="REF_m_flow_con", value=self.condenser.m_flow)
        fs_state.set(name="REF_m_flow_eva", value=self.evaporator.m_flow)
        fs_state.set(name="REF_p_con", value=self.condenser.state_inlet.p / 100000)
        fs_state.set(name="REF_p_eva", value=self.evaporator.state_inlet.p / 100000)
        if save_path_plots is not None:
            self.plot_cycle(save_path=save_path_plots.joinpath(f"{COP_inner}_final_result.png"), inputs=inputs)
        all_states = self.get_states()
        for _state in all_states:
            fs_state.set(name="REF_T_" + _state, value=all_states[_state].T - 273.15)
        for _state in all_states:
            fs_state.set(name="REF_p_" + _state, value=all_states[_state].p / 100000)
        for _state in all_states:
            fs_state.set(name="REF_h_" + _state, value=all_states[_state].h / 1000)
        for _state in all_states:
            fs_state.set(name="REF_q_" + _state, value=all_states[_state].q)
        for _state in all_states:
            fs_state.set(name="REF_d_" + _state, value=all_states[_state].d)
        fs_state.set(name="NumberIterations", value=num_iterations )
        fs_state.set(name="CalcTime",
                     value=round(time.time() - start_time, 2))
        return fs_state


    def set_default_state(self, inputs: Inputs, start_time, comment=""):
        fs_state = FlowsheetState()

        for _var in inputs.get_variable_names():
            fs_state.set(name=_var,
                         value=inputs.get(_var).value)
        # Definieren Sie alle Namen in Listen
        basic_states = [
            "COP", "COP_Carnot", "Q_con", "Q_eva", "P_el",
            "carnot_quality", "SEC_T_con_in", "SEC_T_con_out",
            "SEC_dT_con", "SEC_m_flow_con", "SEC_T_eva_in",
            "SEC_T_eva_out", "SEC_dT_eva", "SEC_m_flow_eva",
            "REF_m_flow_con", "REF_m_flow_eva",
            "REF_p_con", "REF_p_eva"
        ]

        eva_states = [
            "Eva_dh", "Eva_A_sh", "Eva_A_lat",
            "Eva_A_sh_rel", "Eva_A_lat_rel",
            "Eva_Q_sh", "Eva_Q_lat",
            "Eva_Q_sh_rel", "Eva_Q_lat_rel", "Eva_Pinch",
            "Eva_alpha_lat", "Eva_U_lat", "Eva_alpha_gas",
            "Eva_U_gas","Eva_dT_secin","Eva_dT_seclatin","Eva_dT_secout","ErrorEva"
        ]

        con_states = [
            'Con_dh', 'Con_A_sh', 'Con_A_lat', 'Con_A_sc',
            'Con_A_sh_rel', 'Con_A_lat_rel', 'Con_A_sc_rel',
            'Con_Q_sh', 'Con_Q_lat', 'Con_Q_sc',
            'Con_Q_sh_rel', 'Con_Q_lat_rel', 'Con_Q_sc_rel',
            'Con_Pinch', "Con_alpha_sc", "Con_U_sc", "Con_alpha_lat",
            "Con_U_lat", "Con_alpha_sh", "Con_U_sh", "Con_dT_secout","Con_dT_seclatout","Con_dT_seclatin","Con_dT_secin","ErrorCon"
        ]

        compressor_states = [
            'compressor_speed', 'relative_compressor_speed', 'relative_compressor_speed_low',
            'Comp_dh', 'Comp_dh_is',
            'Exp_dh_is', 'Comp_dh_is_Exp_dh_is', 'Comp_dH_is', 'Exp_dH_is',
            'eta_is', 'lambda_h', 'REF_V_flow_comp', 'REF_m_flow_comp', "eta_mech",
            "spec_expansion_losses","relative_compressor_speed_internal","compressor_speed_internal"
        ]

        efficiency_states = [
            'm_low_m_high_ratio', 'eta_is_low', 'eta_is_high',
            'eta_vol_low', 'eta_vol_high', 'compressor_speed_low'
        ]

        pressure_states = [
            'p_con', 'p_eva', 'p_vi', 'p_ihx',
            'Comp_low_dh_is', 'Comp_high_dh_is',
            'Comp_low_dH_is', 'Comp_high_dH_is',
        ]

        expansion_states = ['Exp_high_dh_is', 'Exp_low_dh_is']


        for state in basic_states:
            fs_state.set(name=state)


        for state in eva_states:
            fs_state.set(name=state)


        for state in con_states:
            fs_state.set(name=state)


        for state in compressor_states:
            fs_state.set(name=state)


        for state in efficiency_states:
            fs_state.set(name=state)


        for state in pressure_states:
            fs_state.set(name=state)


        for state in expansion_states:
            fs_state.set(name=state)

        all_states = self.get_state_keys()
        for _state in all_states:
            for suffix in ["_T_", "_p_", "_h_", "_q_", "_d_"]:
                fs_state.set(name=("REF" + suffix + _state))
        fs_state.set(name="NumberIterations")
        fs_state.set(name="Comment",
                     value=comment)
        fs_state.set(name="CalcTime",
                     value=round(time.time()-start_time,2))

        return fs_state

    def set_fs_state_to_off(self, inputs: Inputs, start_time, comment=""):

        fs_state = self.set_default_state(inputs, start_time, comment)

        fs_state.set(
            name="P_el", value=0, unit="W",
            description="Power consumption"
        )
        fs_state.set(
            name="COP", value=0,
            unit="-", description="Coefficient of Performance"
        )
        fs_state.set(
            name="Q_con", value=0, unit="W",
            description="Condenser refrigerant heat flow rate"
        )
        fs_state.set(
            name="Q_eva", value=0, unit="W",
            description="Evaporator refrigerant heat flow rate"
        )
        fs_state.set(name="relative_compressor_speed_internal", value=0, unit="1/s",
                     description="Relative Compressor Speed Internal")
        fs_state.set(name="relative_compressor_speed", value=inputs.n, unit="1/s",
                     description="relative Compressor Speed")

        fs_state.set(name="SEC_T_eva_in", value=inputs.T_eva_in - 273.15,
                     description="Evaporator inlet temperature secondary")
        fs_state.set(name="SEC_T_con_in", value=inputs.T_con_in - 273.15,
                     description="Condenser inlet temperature secondary")
        fs_state.set(name="REF_T_2", value=0)
        if inputs.fix_m_flow_eva == float(False):
            m_flow_eva = 0
        else:
            m_flow_eva = inputs.m_flow_eva

        fs_state.set(name="SEC_m_flow_eva", value=m_flow_eva,
                     description="Evaporator mass flow secondary")

        if inputs.fix_m_flow_con == float(False):
            m_flow_con = 0
        else:
            m_flow_con = inputs.m_flow_con
        fs_state.set(name="SEC_m_flow_con", value=m_flow_con,
                     description="Condenser mass flow secondary")
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

    @abstractmethod
    def get_states(self):
        """
        Function to return all thermodynamic states of cycle
        in the correct order for plotting.
        Include phase change states to see if your simulation
        runs plausible cycles.

        Returns:
            - Dic
        """
        return {}

    @abstractmethod
    def get_state_keys(self):
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
        self.condenser.calc_secondary_cp(T=inputs.T_con_in)
        self.evaporator.m_flow_secondary = inputs.m_flow_eva
        self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
        ax.plot(delta_H_con / 1000, [
            inputs.T_con_in - 273.15,
            inputs.T_con_in + Q_con / self.condenser.m_flow_secondary_cp - 273.15
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

    def calc_missing_IHX_states(self, inputs: Inputs, fs_state: FlowsheetState, **kwargs):
        """


        Args:

            inputs (Inputs): Inputs of calculation.
            fs_state (FlowsheetState): Flowsheet state to save important variables.
        """
        raise NotImplementedError
