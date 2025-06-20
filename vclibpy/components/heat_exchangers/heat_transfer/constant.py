"""
Module with constant heat transfer assumptions
"""
import abc

from vclibpy.media import TransportProperties


class ConstantHeatTransfer(abc.ABC):
    """
    Constant heat transfer assumption

    Args:
        alpha (float):
            Constant heat transfer coefficient in W/(m2*K)
    """

    def __init__(self, alpha: float):
        self.alpha = alpha

    def calc(self, transport_properties_callback: callable, m_flow: float) -> float:
        """
        Calculate constant heat transfer coefficient.

        Args:
            transport_properties_callback (callable): function returning transport properties of the medium (not used).
            m_flow (float): Mass flow rate (not used).

        Returns:
            float: Constant heat transfer coefficient in W/(m2*K).

        """
        return self.alpha


class ConstantTwoPhaseHeatTransfer(abc.ABC):
    """
    Constant heat transfer assumption.

    Args:
        alpha (float):
            Constant heat transfer coefficient in W/(m2*K).
    """

    def __init__(self, alpha: float):
        self.alpha = alpha

    def calc(self, **kwargs) -> float:
        """
        Calculate constant two-phase heat transfer coefficient.

        Args:
            **kwargs: Allows to set arguments for different heat transfer classes which are not used here.

        Returns:
            float: Constant two-phase heat transfer coefficient in W/(m2*K).

        """
        return self.alpha
