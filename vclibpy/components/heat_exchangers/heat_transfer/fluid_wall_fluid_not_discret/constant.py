"""
Module with constant heat transfer assumptions
"""
import abc

from vclibpy.media import TransportProperties
from vclibpy.components.heat_exchangers.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer


class ConstantHeatTransfer(HeatTransfer):
    """
    Constant heat transfer assumption

    Args:
        alpha (float):
            Constant heat transfer coefficient in W/(m2*K)
    """

    def __init__(self, U: float):
        self.U = U

    def calc(self, **kwargs) -> float:
        """
        Calculate constant heat transfer coefficient.

        Args:
            transport_properties (TransportProperties): Transport properties of the medium (not used).
            m_flow (float): Mass flow rate (not used).

        Returns:
            float: Constant heat transfer coefficient in W/(m2*K).

        """
        return self.U
