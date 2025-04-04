import abc
import logging

from .heat_transfer import HeatTransfer
from vclibpy.media import TransportProperties


logger = logging.getLogger(__name__)


class AirToWallTransfer(HeatTransfer, abc.ABC):
    """
    Heat transfer model for air to wall.

    Args:
        A_cross (float):
            Cross-section area in m2.
        characteristic_length (float):
            Length in m to calculate the similitude approach for the
            heat transfer from secondary_medium -> wall.
    """

    def __init__(self, A_cross: float, characteristic_length: float):
        self.A_cross = A_cross
        self.characteristic_length = characteristic_length

    def calc(self, transport_properties: TransportProperties, m_flow: float) -> float:
        """
        Heat transfer coefficient from air to the wall of the heat exchanger.
        The flow is assumed to be always laminar.

        Returns:
            float: Heat transfer coefficient in W/(m^2*K).
        """
        Re = self.calc_reynolds(dynamic_viscosity=transport_properties.dyn_vis, m_flow=m_flow)
        alpha_sec = self.calc_laminar_area_nusselt(Re, transport_properties.Pr, lambda_=transport_properties.lam)
        return alpha_sec

    @abc.abstractmethod
    def calc_reynolds(self, dynamic_viscosity: float, m_flow: float):
        """
        Calculate the reynolds number of the given flow rate.

        Args:
            dynamic_viscosity (float): Dynamic viscosity.
            m_flow (float): Mass flow rate.

        Returns:
            float: Reynolds number
        """
        raise NotImplementedError

    @abc.abstractmethod
    def calc_laminar_area_nusselt(self, Re, Pr, lambda_):
        """
        Calculate the Nusselt number for laminar heat transfer
        on an area (Used for Air->Wall in the evaporator).

        Args:
            Re (float): Reynolds number
            Pr (float): Prandlt number
            lambda_ (float): Lambda of air

        Returns:
            float: Nusselt number
        """
        raise NotImplementedError


class WSUAirToWall(AirToWallTransfer):
    """
    Class to implement the heat transfer calculations
    based on the WSÜ-Script at the RWTH.
    """

    def calc_reynolds(self, dynamic_viscosity: float, m_flow: float) -> float:
        """
        Calculate the Reynolds number on air side.

        Args:
            dynamic_viscosity (float): Dynamic viscosity of air.
            m_flow (float): Mass flow rate of air.

        Returns:
            float: Reynolds number.
        """
        velocity_times_dens = m_flow / self.A_cross
        return (velocity_times_dens * self.characteristic_length) / dynamic_viscosity

    def calc_laminar_area_nusselt(self, Re, Pr, lambda_) -> float:
        """
        Calculate the Nusselt number for laminar heat transfer
        on an area (Used for Air->Wall in the evaporator).

        Args:
            Re (float): Reynolds number of air.
            Pr (float): Prandtl number of air.
            lambda_ (float): Lambda of air.

        Returns:
            float: Nusselt number of air.
        """
        const_fac = 0.664
        exp_rey = 0.5
        exp_pra = 1 / 3
        if Re > 2e5:
            raise ValueError(f"Given Re {Re} is outside of allowed bounds for WSÜ-Script")
        if Pr > 10 or Pr < 0.6:
            raise ValueError(f"Given Pr {Pr} is outside of allowed bounds for WSÜ-Script")
        Nu = const_fac * Re ** exp_rey * Pr ** exp_pra
        return Nu * lambda_ / self.characteristic_length
