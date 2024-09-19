import logging
from dataclasses import dataclass

import numpy as np

from .heat_transfer import HeatTransfer
from vclibpy.media import TransportProperties

logger = logging.getLogger(__name__)


@dataclass
class PlateHeatExchangerGeometry:
    """
    Geometry of a fin and tube heat exchanger with two rows of pipes in a shifted arrangement.

    Source: VDI-Wärmeatlas, Druckverlust und Wärmeübergang in Plattenwärmeübertragern, 11. Auflage, S.1687


    """
    wall_thickness: float   # thickness of a plate
    lambda_w: float         # thermal conductivity of plates
    amplitude: float        # amplitude of a wavy plate
    wave_length: float      # wavelength of a wavy plate
    phi: float              # embossing angle
    width: float            # width of a plate
    height: float           # height of a plate
    n_plates: float         # number of plates

    @property
    def X(self) -> float:
        """return the wave number"""
        return 2 * np.pi * self.amplitude / self.wave_length

    @property
    def enlargement_factor(self) -> float:
        """return surface enlargement factor"""
        return (1 + np.sqrt(1 + self.X ** 2) + 4 * np.sqrt(1 + self.X ** 2 / 2)) / 6

    @property
    def d_h(self) -> float:
        """return hydraulic diameter"""
        return 4 * self.amplitude / self.enlargement_factor

    @property
    def A(self) -> float:
        """return plate surface"""
        return self.height * self.width * self.enlargement_factor * self.n_plates


class VDIAtlasPlateHeatTransfer(HeatTransfer):
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
                 geometry_parameters: PlateHeatExchangerGeometry):
        super().__init__()
        self.geometry_parameters = geometry_parameters

    def calc(self, transport_properties: TransportProperties, m_flow: float) -> float:
        """
        Calculate heat transfer coefficient from refrigerant to the wall of the heat exchanger.

        The flow is assumed to be always turbulent and is based on a calibrated
        Nusselt correlation.

        Args:
            transport_properties (TransportProperties): Transport properties of the fluid.
            m_flow (float): Mass flow rate of the fluid.

        Returns:
            float: Heat transfer coefficient from refrigerant to HE in W/(m^2*K).
        """
        Re = self.calc_reynolds(
            dynamic_viscosity=transport_properties.dyn_vis,
            m_flow=m_flow
        )
        Nu = self.calc_turbulent_plate_nusselt(Re, transport_properties.Pr)
        return Nu * transport_properties.lam / self.geometry_parameters.d_h

    def calc_reynolds(self, dynamic_viscosity: float, m_flow: float) -> float:
        """
        Calculate Reynolds number for flow between two plates.

        Args:
            dynamic_viscosity (float): Dynamic viscosity of the fluid.
            m_flow (float): Mass flow rate.
            width (float): Characteristic length (e.g., width) of the plate.
            enlargement_factor (float): surface enlargement factor.

        Returns:
            float: Reynolds number.

        """

        return 2 * m_flow / (self.geometry_parameters.enlargement_factor * self.geometry_parameters.width * dynamic_viscosity)

    def calc_turbulent_plate_nusselt(self, Re: float, Pr: float) -> float:
        """
        Calculate Nusselt Number based on Nusselt correlation VDI Atlas.

        Args:
            Re (float): Reynolds number.
            Pr (float): Prandtl number.
            Zeta (float): pressure drop coefficient.
            eta_by_eta_w (float): dynamic viscosity temperature correction factor.

        Returns:
            float: Apparent heat transfer coefficient.
        """
        c_q = 0.122
        q = 0.374
        eta_by_eta_w = 1
        Zeta = 1

        return c_q * Pr ** (1/3) * eta_by_eta_w ** (1/6) * (Zeta * Re ** 2 * np.sin(2 * self.geometry_parameters.phi * np.pi / 180)) ** q
