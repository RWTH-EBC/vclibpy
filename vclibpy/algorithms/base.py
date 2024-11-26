import abc
from typing import Union
from pathlib import Path

from scipy.optimize import fsolve
import numpy as np

from vclibpy.flowsheets import BaseCycle
from vclibpy import Inputs, FlowsheetState


class Algorithm(abc.ABC):
    """
    Base class to define an algorithm which calculates
    steady FlowsheetStates for given flowsheets and inputs.

    Args:
        max_err (float):
            Maximum allowable error for the heat exchanger in percent (default: 0.5).
        save_path_plots (pathlib.Path, str or None):
            The path to save plots (default: None).
            If None, no plots are created.
        dT_eva_start_guess (float):
            Initial temperature difference guess for outlet-based calculation in evaporator
        dT_pinch_eva_guess (float):
            Initial pinch point temperature difference guess in evaporator (default: 0)
        dT_con_start_guess (float):
            Initial temperature difference guess for outlet-based calculation in condenser
        dT_pinch_con_guess (float):
            Initial pinch point temperature difference guess in condenser,
            required for `improve_first_condensing_guess` (default: 0)
        improve_first_condensing_guess (bool):
            Improve the guess for condensing pressure for inlet-based calculation
            using scipy fsolve. Requires a good guess of dT_pinch_con_guess.
            (default: False)
        p_min (float):
            Minimal pressure allowed to ensure p>0 at all times. (default: 10000)
    """

    def __init__(self, **kwargs):
        """Initialize class with kwargs"""
        if "save_path_plots" in kwargs:
            self.save_path_plots = Path(kwargs["save_path_plots"])
        else:
            self.save_path_plots = None
        self.max_err = kwargs.pop("max_err", 0.5)
        self.dT_eva_start_guess = kwargs.pop("dT_start_guess", 3)
        self.dT_pinch_eva_guess = kwargs.pop("dT_pinch_eva_guess", 0)
        self.dT_con_start_guess = kwargs.pop("dT_start_guess", 10)
        self.dT_pinch_con_guess = kwargs.pop("dT_pinch_con_guess", 0)
        self.improve_first_condensing_guess = kwargs.pop("improve_first_condensing_guess", False)
        self._p_min = kwargs.pop("p_min", 10000)

    @abc.abstractmethod
    def calc_steady_state(
            self,
            flowsheet: BaseCycle,
            inputs: Inputs,
            fluid: str = None
    ) -> Union[FlowsheetState, None]:
        """
        Calculate the steady-state performance of a vapor compression cycle
        based on given inputs and assumptions.

        Args:
            flowsheet (BaseCycle):
                The flowsheet to calculate the steady state for.
            inputs (Inputs):
                An instance of the Inputs class containing the
                necessary parameters to calculate the flowsheet state.
            fluid (str):
                The fluid to be used in the calculations.
                Required only if 'fluid' is not specified during the object's initialization.

        Returns:
            fs_state (FlowsheetState):
                An instance of the FlowsheetState class representing
                the calculated state of the vapor compression cycle.
            None: If an error occurs (which will be logged, not raised).
        """
        raise NotImplementedError

    def initial_setup(self, flowsheet: BaseCycle, fluid: str, inputs: Inputs) -> (float, float, float):
        """
        Calculate values required for all algorithms at the initial setup

        Args:
            flowsheet (BaseCycle):
                The flowsheet to calculate the steady state for.
            inputs (Inputs):
                An instance of the Inputs class containing the
                necessary parameters to calculate the flowsheet state.
            fluid (str):
                The fluid to be used in the calculations.
                Required only if 'fluid' is not specified during the object's initialization.

        Returns:
            p_1_start: Start guess for p_1
            p_2_start: Start guess for p_2
            p_max: Maximal pressure (critical point)
        """
        # Setup fluid:
        if fluid is None:
            fluid = flowsheet.fluid
        flowsheet.setup_new_fluid(fluid)

        # Get max and min pressure
        _, _p_max, _ = flowsheet.med_prop.get_critical_point()
        p_1_start = flowsheet.get_start_evaporating_pressure(
            inputs=inputs, dT_pinch_guess=self.dT_pinch_eva_guess,
            dT_start_guess=self.dT_eva_start_guess
        )
        p_2_start = flowsheet.get_start_condensing_pressure(
            inputs=inputs,
            dT_start_guess=self.dT_con_start_guess
        )
        if self.get_improved_start_condensing_pressure:
            p_2_start = self.get_improved_start_condensing_pressure(
                inputs=inputs,
                m_flow_guess=flowsheet.condenser.m_flow,
                p_2_guess=p_2_start
            )

        return p_1_start, p_2_start, _p_max

    def get_improved_start_condensing_pressure(
            self, flowsheet: BaseCycle,
            inputs: Inputs,
            m_flow_guess: float,
            p_2_guess: float
    ):
        flowsheet.condenser.m_flow_secondary = inputs.condenser.m_flow  # [kg/s]
        flowsheet.condenser.calc_secondary_cp(T=inputs.condenser.T)

        def nonlinear_func(p_2, *args):
            _flowsheet, _inputs, _m_flow_ref, _dT_pinch = args
            state_q0 = _flowsheet.med_prop.calc_state("PQ", p_2, 0)
            state_q1 = _flowsheet.med_prop.calc_state("PQ", p_2, 1)
            state_3 = _flowsheet.med_prop.calc_state("PT", p_2, state_q0.T - _inputs.control.dT_con_subcooling)
            Q_water_till_q1 = (state_q1.h - state_3.h) * _m_flow_ref
            T_water_q1 = _inputs.condenser.T_in + Q_water_till_q1 / _flowsheet.condenser.m_flow_secondary_cp
            return T_water_q1 + _dT_pinch - state_q1.T

        p_2_guess_optimized = fsolve(
            func=nonlinear_func,
            x0=np.array([p_2_guess]),
            args=(flowsheet, inputs, m_flow_guess, self.dT_pinch_con_guess)
        )[0]
        return p_2_guess_optimized
