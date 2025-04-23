"""
Module with constant heat transfer assumptions
"""
import abc

from vclibpy.media import TransportProperties
from .heat_transfer import HeatTransfer, TwoPhaseHeatTransfer


class MassFlowDependingHeatTransfer(HeatTransfer):
    """
    Constant heat transfer assumption

    Args:
        alpha (float):
            Constant heat transfer coefficient in W/(m2*K)
    """

    def __init__(self, alpha: float, m_ref=None):
        self.alpha = alpha
        self.m_ref = m_ref
    def calc(self, transport_properties: TransportProperties, m_flow: float) -> float:
        """
        Calculate constant heat transfer coefficient.

        Args:
            transport_properties (TransportProperties): Transport properties of the medium (not used).
            m_flow (float): Mass flow rate (not used).

        Returns:
            float: Constant heat transfer coefficient in W/(m2*K).

        """
        if self.m_ref is None:
            return self.alpha
        return self.alpha * (m_flow/self.m_ref)**0.8


class MassFlowDependingTwoPhaseHeatTransfer(TwoPhaseHeatTransfer):
    """
    Constant heat transfer assumption.

    Args:
        alpha (float):
            Constant heat transfer coefficient in W/(m2*K).
    """

    def __init__(self, alpha: float, m_ref=None):
        self.alpha = alpha
        self.m_ref = m_ref

    def calc(self, **kwargs) -> float:
        """
        Calculate constant two-phase heat transfer coefficient.

        Args:
            **kwargs: Allows to set arguments for different heat transfer classes which are not used here.

        Returns:
            float: Constant two-phase heat transfer coefficient in W/(m2*K).

        """
        if self.m_ref is None:
            return self.alpha
        m_flow = kwargs.get("m_flow", self.m_ref)

        return self.alpha * (m_flow/self.m_ref)**0.8
