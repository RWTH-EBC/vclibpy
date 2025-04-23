"""
Module with classes for EV models.
They are not used for the calculation.
They may be used to see if the output is correct.
"""
import abc

from vclibpy.components.component import BaseComponent


class ExpansionValve(BaseComponent, abc.ABC):
    """Base class for an expansion valve.

    Args:
        A (float): Cross-sectional area of the expansion valve.
    """

    def __init__(self, A):
        super().__init__()
        self.A = A  # Cross-sectional area of the expansion valve

    @abc.abstractmethod
    def calc_m_flow_at_opening(self, opening) -> float:
        """
        Calculate the mass flow rate for the given opening.

        Args:
            opening (float): Opening of valve between 0 and 1

        Returns:
            float: Mass flow rate in kg/s
        """
        raise NotImplementedError

    @abc.abstractmethod
    def calc_opening_at_m_flow(self, m_flow, **kwargs) -> float:
        """
        Calculate the opening for the given mass flow rate

        Args:
            m_flow (float): Mass flow rate in kg/s
            **kwargs: Possible keyword arguments for child classes

        Returns:
            float: Opening
        """
        raise NotImplementedError

    def calc_outlet(self, p_outlet: float):
        """
        Calculate isenthalpic expansion valve.

        Args:
            p_outlet (float): Outlet pressure level
        """
        self.state_outlet = self.med_prop.calc_state("PH", p_outlet, self.state_inlet.h)

    def  terminate_secondary_med_prop(self):
        if self.med_prop is not None:
            self.med_prop.terminate()
