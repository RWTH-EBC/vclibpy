import logging
from dataclasses import dataclass

import numpy as np

from .air_to_wall import AirToWallTransfer

logger = logging.getLogger(__name__)


@dataclass
class AirSourceHeatExchangerGeometry:
    """
    Geometry of a fin and tube heat exchanger with two rows of pipes in a shifted arrangement.

    Source: VDI-Wärmeatlas, Berechnungsblätter für den Wärmeübergang, 11. Auflage, S.1461

    As the source is in German, the names are kept in German as well.
    """
    t_l: float  # Achsabstand der Rohre in Luftrichtung in m
    t_q: float    # Achsabstand der Rohre quer zur Luftrichtung in m
    tiefe: float  # Tiefe der Rippe gesamt
    d_a: float  # Äußerer Rohrdurchmesser
    d_i: float  # Innener Rohrdurchmesser
    lambda_R: float  # Wärmeleitfähigkeit Material der Wand
    n_Rohre: int = 50
    n_Rippen: int = 500
    a: float = 1.95e-3
    dicke_rippe: float = 0.05e-3
    laenge: float = 1.040
    hoehe: float = 0.64

    @property
    def char_length(self) -> float:
        """Return the characteristic length d_a."""
        return self.d_a

    @property
    def A_Rippen(self) -> float:
        """Return the total surface area of the fins."""
        return 2 * self.n_Rippen * (self.tiefe * self.hoehe - self.n_Rohre * np.pi * 0.25 * self.d_a ** 2)

    @property
    def A(self) -> float:
        """Total air side heat transfer area."""
        return self.A_Rippen + self.A_FreieOberflaecheRohr

    @property
    def A_FreieOberflaecheRohr(self) -> float:
        """Free air side area of the tubes."""
        return self.n_Rippen * np.pi * self.d_a * self.a * self.n_Rohre

    @property
    def A_RohrInnen(self) -> float:
        """Total refrigerant heat transfer area."""
        return self.laenge * self.d_i * np.pi * self.n_Rohre

    @property
    def A_RohrUnberippt(self) -> float:
        """Total outside area of the tubes without fins"""
        return self.laenge * self.d_a * np.pi * self.n_Rohre

    @property
    def verjuengungsfaktor(self) -> float:
        """Reduction factor A_cross_free/A_cross_smallest"""
        return ((self.hoehe * self.laenge) /
                (self.hoehe * self.laenge -
                 self.hoehe * self.n_Rippen * self.dicke_rippe -
                 self.d_a * self.n_Rohre / 2 * (self.laenge - self.n_Rippen * self.dicke_rippe)))

    @property
    def phi(self) -> float:
        """Auxiliary variable for fin efficiency"""
        if self.t_l >= 0.5 * self.t_q:
            b_r = self.t_q
        else:
            b_r = 2 * self.t_l
        l_r = np.sqrt(self.t_l **2 + self.t_q **2 / 4)
        phi_strich = 1.27 * b_r / self.d_a * np.sqrt(l_r / b_r - 0.3)
        return (phi_strich - 1) * (1 + 0.35 * np.log(phi_strich))

    def eta_R(self, alpha_R) -> float:
        """
        Calculate fin efficiency.

        Args:
            alpha_R (float): Average heat transfer coefficient for tube and fin.

        Returns:
            float: Fin efficiency.
        """
        X = self.phi * self.d_a / 2 * np.sqrt(2 * alpha_R / (self.lambda_R * self.dicke_rippe))
        return 1 / X * (np.exp(X) - np.exp(-X)) / (np.exp(X) + np.exp(-X))

    def alpha_S(self, alpha_R) -> float:
        """
        Calculate apparent heat transfer coefficient.

        Args:
            alpha_R (float): Average heat transfer coefficient for tube and fin.

        Returns:
            float: Apparent heat transfer coefficient.
        """
        A_R_to_A = self.A_Rippen / self.A
        return alpha_R * (1 - (1 - self.eta_R(alpha_R)) * A_R_to_A)


class VDIAtlasAirToWallTransfer(AirToWallTransfer):
    """
    VDI-Atlas based heat transfer estimation.
    Check `AirToWallTransfer` for further argument options.

    This class assumes two pipes with a shifted arrangement.

    Args:
        A_cross (float): Free cross-sectional area.
        characteristic_length (float): Characteristic length d_a.
        geometry_parameters (AirSourceHeatExchangerGeometry):
            Geometry parameters for heat exchanger according to VDI-Atlas
    """

    def __init__(self,
                 A_cross: float, characteristic_length: float,
                 geometry_parameters: AirSourceHeatExchangerGeometry):
        super().__init__(A_cross=A_cross, characteristic_length=characteristic_length)
        self.geometry_parameters = geometry_parameters

    def calc_reynolds(self, dynamic_viscosity: float, m_flow: float) -> float:
        """
        Calculate Reynolds number.

        Args:
            dynamic_viscosity (float): Dynamic viscosity of the fluid.
            m_flow (float): Mass flow rate.

        Returns:
            float: Reynolds number.
        """
        velocity_times_dens = m_flow / self.A_cross * self.geometry_parameters.verjuengungsfaktor
        return (velocity_times_dens * self.characteristic_length) / dynamic_viscosity

    def calc_laminar_area_nusselt(self, Re: float, Pr: float, lambda_: float) -> float:
        """
        Calculate apparent heat transfer coefficient based on Nusselt correlation.

        Args:
            Re (float): Reynolds number.
            Pr (float): Prandtl number.
            lambda_ (float): Thermal conductivity of air.

        Returns:
            float: Apparent heat transfer coefficient.
        """
        C = 0.33  # Versetzte Anordnung, zwei Rohre
        if Re < 1e3 or Re > 1e5:
            logger.debug("Given Re %s is outside of allowed bounds for VDI-Atlas", Re)
        A_div_A_0 = self.geometry_parameters.A / self.geometry_parameters.A_RohrUnberippt
        if A_div_A_0 < 5 or A_div_A_0 > 30:
            logger.debug("Given A/A0 is outside of bounds for method VDI-Atlas")
        Nu = (
                C * Re ** 0.6 *
                A_div_A_0 ** (-0.15) *
                Pr ** (1 / 3)
        )
        alpha_R = Nu * lambda_ / self.characteristic_length
        alpha_S = self.geometry_parameters.alpha_S(alpha_R=alpha_R)
        return alpha_S
