"""
Module with models for pipe-to-wall heat transfer.
"""

from .heat_transfer import HeatTransfer, calc_reynolds_pipe
from vclibpy.media import TransportProperties


class TurbulentFluidInPipeToWallTransfer(HeatTransfer):
    """
    Class to model turbulent heat exchange in a pipe.

    Args:
        method (str):
            Equation to calc the nusselt number of turbulent flow for
            a given Re and Pr number.
            Note: Just for one-phase heat transfer!!
            Implemented Options are:

            - Taler2016
            - Domanski1989_sp_smooth
            - Amalfi2016
            - ScriptWSÜ. For turbulent regimes, eta_by_eta_w is assumed to be one.

            Refer to the paper / documents or the function in this class for more
            info on numbers and assumptions
        characteristic_length (float):
            Length to calculate the similitude approach for the
            heat transfer from ref -> wall. For heat pumps this is
            always the Diameter of the HE in m
    """

    def __init__(self, method: str, characteristic_length: float):
        self.method = method
        self.characteristic_length = characteristic_length

    def calc(self, transport_properties_callback: callable, m_flow: float) -> float:
        """
        Calculate heat transfer coefficient from refrigerant to the wall of the heat exchanger.

        The flow is assumed to be always turbulent and is based on a calibrated
        Nusselt correlation.

        Args:
            transport_properties_callback (callable): function returning transport properties of the fluid.
            m_flow (float): Mass flow rate of the fluid.

        Returns:
            float: Heat transfer coefficient from refrigerant to HE in W/(m^2*K).
        """
        transport_properties = transport_properties_callback()
        Re = calc_reynolds_pipe(
            characteristic_length=self.characteristic_length,
            dynamic_viscosity=transport_properties.dyn_vis,
            m_flow=m_flow
        )
        Nu = self.calc_turbulent_tube_nusselt(Re, transport_properties.Pr)
        return Nu * transport_properties.lam / self.characteristic_length

    def calc_turbulent_tube_nusselt(self, Re, Pr) -> float:
        """
        Calculate the Nuseelt number for turbulent heat transfer
        in a tube (used for ref/water->Wall in the evaporator and condernser).

        Args:
            Re (float): Reynolds number.
            Pr (float): Prandtl number.

        Returns:
            float: Nusselt number.
        """
        if self.method == "Taler2016":
            if Re < 3e3 or Re > 1e6:
                raise ValueError(f"Given Re {Re} is outside of allowed bounds for method {self.method}")
            if 0.1 <= Pr <= 1:
                return 0.02155 * Re ** 0.8018 * Pr ** 0.7095
            elif 1 < Pr <= 3:
                return 0.02155 * Re ** 0.8018 * Pr ** 0.7095
            elif 3 < Pr <= 1000:
                return 0.02155 * Re ** 0.8018 * Pr ** 0.7095
            else:
                raise ValueError(f"Given Pr {Pr} is outside of allowed bounds for method {self.method}")
        elif self.method == "ScriptWSÜ":
            if Re < 3000 or Re > 1e5:
                raise ValueError(f"Given Re {Re} is outside of allowed bounds for method {self.method}")
            eta_by_eta_w = 1
            return 0.027 * Re ** 0.8 ** Pr ** (1 / 3) * eta_by_eta_w ** 0.14
        elif self.method == "Amalfi2016":
            if Re <= 700:
                Nu = (0.0295 * Pr - 0.115) * Re ** 0.954
            else:
                Nu = (1.760 * Pr - 5.391) * Re ** 0.262
            if Nu < 0:
                raise ValueError(f"Given Pr {Pr} is outside of allowed bounds for method {self.method}")
            return Nu
        elif self.method == "Domanski1989_sp_smooth":
            # According to Domanski 1989 for singular phase and smooth surfaces
            return 0.023 * Re ** 0.8 * Pr ** 0.4
        else:
            raise TypeError(f"Method {self.method} not supported!")
