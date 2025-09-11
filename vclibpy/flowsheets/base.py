import logging
from typing import List
import numpy as np

from abc import abstractmethod
import matplotlib.pyplot as plt
from vclibpy import media, Inputs
from vclibpy.datamodels import FlowsheetState
from vclibpy.components.heat_exchangers import HeatExchanger, ExternalHeatExchanger
from vclibpy.components.component import BaseComponent

logger = logging.getLogger(__name__)


class BaseCycle:
    """
    Base class for a heat pump. More complex systems may inherit from this class
    All HP have a compressor, two HE and a source and sink.
    Therefore, the parameters defined here are general parameters.

    Args:
        fluid (str): Name of the fluid
        evaporator (ExternalHeatExchanger): Instance of a heat exchanger used for the  evaporator
        condenser (ExternalHeatExchanger): Instance of a heat exchanger used for the condenser
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
        # Add helper to improve log levels
        self.iteration_converged = False

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
            if isinstance(component, ExternalHeatExchanger):
                component.start_secondary_med_prop()

        self.fluid = fluid

    def terminate(self):
        if self.med_prop is not None:
            self.med_prop.terminate()
            self.med_prop = None
        for component in self.get_all_components():
            if isinstance(component, ExternalHeatExchanger):
                component.terminate_secondary_med_prop()
            component.med_prop = None

    def get_all_components(self) -> List[BaseComponent]:
        return [self.condenser, self.evaporator]

    def get_start_condensing_pressure(self, inputs: Inputs, dT_start_guess: float):
        """Calculates initial guess for condensing pressure based on inlet/outlet conditions.

        Args:
            inputs: Input parameters containing condenser and control settings
            dT_start_guess: Initial temperature difference guess for outlet-based calculation

        Returns:
            float: Initial guess for condensing pressure in Pa
        """
        if inputs.condenser.uses_inlet:
            T_3_start = inputs.condenser.T_in + inputs.control.dT_con_subcooling
        else:
            T_3_start = inputs.condenser.T_out - dT_start_guess
        p_2_start = self.med_prop.calc_state("TQ", T_3_start, 0).p
        return p_2_start

    def get_start_evaporating_pressure(self, inputs: Inputs, dT_start_guess: float, dT_pinch_guess: float = 0):
        """Calculates initial guess for evaporating pressure based on inlet/outlet conditions.

        Args:
            inputs: Input parameters containing evaporator and control settings
            dT_start_guess: Initial temperature difference guess for outlet-based calculation
            dT_pinch_guess: Initial pinch point temperature difference guess (default: 0)

        Returns:
            float: Initial guess for evaporating pressure in Pa
        """
        if inputs.evaporator.uses_inlet:
            T_eva_in = inputs.evaporator.T_in
        else:
            T_eva_in = inputs.evaporator.T_out + dT_start_guess
        T_1_start = T_eva_in - inputs.control.dT_eva_superheating - dT_pinch_guess
        return self.med_prop.calc_state("TQ", T_1_start, 1).p

    def calculate_cycle_for_pressures(self, p_1: float, p_2: float, inputs: Inputs, fs_state: FlowsheetState):
        self.evaporator.calc_secondary_cp(T=inputs.evaporator.T)
        self.condenser.calc_secondary_cp(T=inputs.condenser.T)
        # Calculate the states based on the given flowsheet
        self.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
        self.add_all_states_to_fs_state(inputs=inputs, fs_state=fs_state)
        # Check heat exchangers:
        error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)
        error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)
        return error_eva, dT_min_eva, error_con, dT_min_con

    def add_all_states_to_fs_state(self, inputs: Inputs, fs_state: FlowsheetState):
        # Calculate the heat flow rates for the selected states.
        Q_con = self.condenser.calc_Q_flow()
        Q_con_outer = self.condenser.calc_secondary_Q_flow(Q_con)
        Q_eva = self.evaporator.calc_Q_flow()
        Q_eva_outer = self.evaporator.calc_secondary_Q_flow(Q_eva)
        P_el = self.calc_electrical_power(fs_state=fs_state, inputs=inputs)

        T_con_in, T_con_out, dT_con, m_flow_con = inputs.condenser.get_all_inputs(
            Q=Q_con_outer, cp=self.condenser.cp_secondary
        )
        T_eva_in, T_eva_out, dT_eva, m_flow_eva = inputs.evaporator.get_all_inputs(
            Q=Q_eva_outer, cp=self.evaporator.cp_secondary
        )
        # In case dT_con is used
        inputs.condenser.set("m_flow", m_flow_con)
        inputs.evaporator.set("m_flow", m_flow_eva)

        # COP based on P_el and Q_con:
        COP_inner = Q_con / P_el
        COP_outer = Q_con_outer / P_el
        # Calculate carnot quality as a measure of reliability of model:
        COP_carnot = (T_con_out / (T_con_out - inputs.evaporator.T_in))
        carnot_quality = COP_inner / COP_carnot

        # Update input values as not all in/out/dT/m_flow values are provided by the user.
        fs_state = fill_fs_state_from_inputs(
            fs_state=fs_state,
            inputs=inputs,
            T_con_out=T_con_out,
            T_con_in=T_con_in,
            dT_con=dT_con,
            m_flow_con=m_flow_con,
            T_eva_out=T_eva_out,
            T_eva_in=T_eva_in,
            dT_eva=dT_eva,
            m_flow_eva=m_flow_eva,
        )

        # Set outputs
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
            name="eta_glob", value=fs_state.get("eta_is").value * fs_state.get("eta_mech").value,
            unit="-", description="Global compressor efficiency"
        )

    def calculate_outputs_for_valid_pressures(
            self,
            p_1,
            p_2,
            fs_state: FlowsheetState,
            inputs: Inputs,
            save_path_plots
    ):
        self.calc_states(p_1, p_2, inputs=inputs, fs_state=fs_state)
        self.add_all_states_to_fs_state(inputs=inputs, fs_state=fs_state)
        error_con, dT_min_con = self.condenser.calc(inputs=inputs, fs_state=fs_state)
        error_eva, dT_min_eva = self.evaporator.calc(inputs=inputs, fs_state=fs_state)

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
        T_1 = self.med_prop.calc_state("PQ", p_eva, 1).T + inputs.control.dT_eva_superheating
        if inputs.control.dT_eva_superheating > 0:
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
        T_3 = self.med_prop.calc_state("PQ", p_con, 0).T - inputs.control.dT_con_subcooling
        if inputs.control.dT_con_subcooling > 0:
            self.condenser.state_outlet = self.med_prop.calc_state("PT", p_con, T_3)
        else:
            self.condenser.state_outlet = self.med_prop.calc_state("PQ", p_con, 0)

    def plot_cycle(self, save_path: str, inputs: Inputs):
        """Function to plot the resulting flowsheet of the steady state config."""
        from vclibpy.utils.plotting import plot_cycle
        states = self.get_states_in_order_for_plotting()
        fig, ax = plot_cycle(
            states=states,
            med_prop=self.med_prop,
            save_path=None
        )
        self._plot_secondary_heat_flow_rates(ax=ax[0], inputs=inputs)
        fig.tight_layout()
        fig.savefig(save_path)
        plt.close(fig)

    def _plot_secondary_heat_flow_rates(self, ax, inputs):
        self.condenser.m_flow_secondary = inputs.condenser.m_flow
        self.evaporator.m_flow_secondary = inputs.evaporator.m_flow
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
        T_eva_in, T_eva_out, _, _ = inputs.evaporator.get_all_inputs(Q=-Q_eva, cp=self.evaporator.cp_secondary)
        T_con_in, T_con_out, _, _ = inputs.condenser.get_all_inputs(Q=Q_con, cp=self.condenser.cp_secondary)
        ax.plot(delta_H_con / 1000, [T_con_in - 273.15, T_con_out - 273.15], color="b")
        ax.plot(delta_H_eva / 1000, [T_eva_in - 273.15, T_eva_out - 273.15], color="b")

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


def fill_fs_state_from_inputs(
        T_con_out: float,
        T_con_in: float,
        dT_con: float,
        m_flow_con: float,
        T_eva_out: float,
        T_eva_in: float,
        dT_eva: float,
        m_flow_eva: float,
        inputs: Inputs,
        fs_state: FlowsheetState
):
    fs_state.set(
        name="T_con_out",
        value=T_con_out,
        unit=inputs.condenser.get('T_out').unit,
        description=f"Condenser {inputs.condenser.get('T_out').description}"
    )
    fs_state.set(
        name="T_con_in",
        value=T_con_in,
        unit=inputs.condenser.get('T_in').unit,
        description=f"Condenser {inputs.condenser.get('T_in').description}"
    )
    fs_state.set(
        name="dT_con",
        value=dT_con,
        unit=inputs.condenser.get('m_flow').unit,
        description=f"Condenser {inputs.condenser.get('m_flow').description}"
    )
    fs_state.set(
        name="m_flow_con",
        value=m_flow_con,
        unit=inputs.condenser.get('dT').unit,
        description=f"Condenser {inputs.condenser.get('dT').description}"
    )
    fs_state.set(
        name="T_eva_out",
        value=T_eva_out,
        unit=inputs.evaporator.get('T_out').unit,
        description=f"Evaporator {inputs.evaporator.get('T_out').description}"
    )
    fs_state.set(
        name="T_eva_in",
        value=T_eva_in,
        unit=inputs.evaporator.get('T_in').unit,
        description=f"Evaporator {inputs.evaporator.get('T_in').description}"
    )
    fs_state.set(
        name="dT_eva",
        value=dT_eva,
        unit=inputs.evaporator.get('m_flow').unit,
        description=f"Evaporator {inputs.evaporator.get('m_flow').description}"
    )
    fs_state.set(
        name="m_flow_eva",
        value=m_flow_eva,
        unit=inputs.evaporator.get('dT').unit,
        description=f"Evaporator {inputs.evaporator.get('dT').description}"
    )
    # Add control inputs
    for variable in inputs.control.get_variables().values():
        fs_state.set(**variable.__dict__)

    return fs_state
