"""
Module with constant heat transfer assumptions
"""
import abc

from vclibpy.media import TransportProperties
from .heat_transfer import HeatTransfer, TwoPhaseHeatTransfer


class ConstantHeatTransfer(HeatTransfer):
    """
    Constant heat transfer assumption

    Args:
        alpha (float):
            Constant heat transfer coefficient in W/(m2*K)
    """

    def __init__(self, alpha: float):
        self.alpha = alpha

    def calc(self, transport_properties: TransportProperties, m_flow: float) -> float:
        """
        Calculate constant heat transfer coefficient.

        Args:
            transport_properties (TransportProperties): Transport properties of the medium (not used).
            m_flow (float): Mass flow rate (not used).

        Returns:
            float: Constant heat transfer coefficient in W/(m2*K).

        """
        return self.alpha


class ConstantTwoPhaseHeatTransfer(TwoPhaseHeatTransfer):
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
