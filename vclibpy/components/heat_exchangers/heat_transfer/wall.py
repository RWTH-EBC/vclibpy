import logging

from .heat_transfer import HeatTransfer
from vclibpy.media import TransportProperties


logger = logging.getLogger(__name__)


class WallTransfer(HeatTransfer):
    """
    Simple wall heat transfer

    Args:
        lambda_ (float):
            Heat conductivity of wall material in W/Km
        thickness (float):
            Thickness of wall in m^2
    """
    def __init__(self, lambda_: float, thickness: float):
        self.thickness = thickness
        self.lambda_ = lambda_

    def calc(self, transport_properties: TransportProperties, m_flow: float) -> float:
        """
        Heat transfer coefficient inside wall

        Returns:
            float: Heat transfer coefficient in W/(m^2*K)
        """
        return self.lambda_ / self.thickness
