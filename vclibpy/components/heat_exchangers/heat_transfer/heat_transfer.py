"""
Module with basic functions to calculate heat transfer coefficients.
"""
import abc

import numpy as np

from vclibpy.media import TransportProperties, ThermodynamicState, MedProp
from vclibpy.datamodels import FlowsheetState, Inputs


class HeatTransfer(abc.ABC):
    """
    Base class to implement possible heat transfer models.

    Methods:
        calc(transport_properties_callback: callable, m_flow: float) -> float:
            Abstract method to calculate heat transfer.

    """

    @abc.abstractmethod
    def calc(self, transport_properties_callback: callable, m_flow: float) -> float:
        """
        Calculate heat transfer.

        Args:
            transport_properties_callback (callable): function returning transport properties of the medium.
            m_flow (float): Mass flow rate.

        Returns:
            float: Calculated heat transfer coefficient.

        Raises:
            NotImplementedError: If the method is not implemented in the subclass.
        """
        raise NotImplementedError


class TwoPhaseHeatTransfer(abc.ABC):
    """
    Base class to implement possible heat transfer models
    """

    @abc.abstractmethod
    def calc(
            self,
            state_q0: ThermodynamicState,
            state_q1: ThermodynamicState,
            state_inlet: ThermodynamicState,
            state_outlet: ThermodynamicState,
            med_prop: MedProp,
            inputs: Inputs,
            fs_state: FlowsheetState,
            m_flow: float
    ) -> float:
        """
        Calculate two-phase heat transfer.

        Args:
            state_q0 (ThermodynamicState): Thermodynamic state at the beginning of the two-phase region.
            state_q1 (ThermodynamicState): Thermodynamic state at the end of the two-phase region.
            state_inlet (ThermodynamicState): Inlet thermodynamic state.
            state_outlet (ThermodynamicState): Outlet thermodynamic state.
            med_prop (MedProp): Medium properties class.
            inputs (Inputs): Input parameters.
            fs_state (FlowsheetState): Flowsheet state.
            m_flow (float): Mass flow rate.

        Returns:
            float: Calculated two-phase heat transfer coefficient.

        Raises:
            NotImplementedError: If the method is not implemented in the subclass.
        """
        raise NotImplementedError


def calc_reynolds_pipe(dynamic_viscosity: float, m_flow: float, characteristic_length: float) -> float:
    """
    Calculate Reynolds number for flow inside a pipe.

    Args:
        dynamic_viscosity (float): Dynamic viscosity of the fluid.
        m_flow (float): Mass flow rate.
        characteristic_length (float): Characteristic length (e.g., diameter) of the pipe.

    Returns:
        float: Reynolds number.

    """
    return 4 * m_flow / (np.pi * characteristic_length * dynamic_viscosity)
