import abc

from typing import Tuple

from vclibpy import media
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.component import BaseComponent
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer


class HeatExchanger(BaseComponent, abc.ABC):
    """
    Class for a heat exchanger.

    Args:
        A (float):
            Area of HE in m^2 for NTU calculation
        secondary_medium (str):
            Name for secondary medium, e.g. `water` or `air`
        wall_heat_transfer (HeatTransfer):
            Model for heat transfer inside wall
        secondary_heat_transfer (HeatTransfer):
            Model for heat transfer from secondary medium to wall
        gas_heat_transfer (HeatTransfer):
            Model for heat transfer from refrigerant gas to wall
        liquid_heat_transfer (HeatTransfer):
            Model for heat transfer from refrigerant liquid to wall
        two_phase_heat_transfer (TwoPhaseHeatTransfer):
            Model for heat transfer from refrigerant two phase to wall
    """

    def __init__(
            self,
            A: float,
            wall_heat_transfer: HeatTransfer,
            secondary_heat_transfer: HeatTransfer,
            gas_heat_transfer: HeatTransfer,
            liquid_heat_transfer: HeatTransfer,
            two_phase_heat_transfer: TwoPhaseHeatTransfer,
            secondary_medium: str,
            ratio_outer_to_inner_area: float = 1,
    ):
        super().__init__()
        self.A = A
        self.secondary_medium = secondary_medium.lower()
        self.ratio_outer_to_inner_area = ratio_outer_to_inner_area

        self._wall_heat_transfer = wall_heat_transfer
        self._secondary_heat_transfer = secondary_heat_transfer
        self._gas_heat_transfer = gas_heat_transfer
        self._liquid_heat_transfer = liquid_heat_transfer
        self._two_phase_heat_transfer = two_phase_heat_transfer

        self.med_prop_sec = None  # Later start in start_secondary_med_prop
        self._m_flow_secondary = None
        self._secondary_cp = 0  # Allow initial calculation of _m_flow_secondary_cp if cp is not set
        self._m_flow_secondary_cp = 0

    def start_secondary_med_prop(self):
        """
        Set up the wrapper for the secondary medium's media properties.
        """
        # Set up the secondary_medium wrapper:
        med_prop_class, med_prop_kwargs = media.get_global_med_prop_and_kwargs()
        if self.secondary_medium == "air" and med_prop_class == media.RefProp:
            fluid_name = "AIR.PPF"
        else:
            fluid_name = self.secondary_medium
        if self.med_prop_sec is not None:
            if self.med_prop_sec.fluid_name == fluid_name:
                return
            self.med_prop_sec.terminate()
        self.med_prop_sec = med_prop_class(fluid_name=self.secondary_medium, **med_prop_kwargs)

    def terminate_secondary_med_prop(self):
        if self.med_prop_sec is not None:
            self.med_prop_sec.terminate()

    @abc.abstractmethod
    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> Tuple[float, float]:
        """
        Calculate the heat exchanger based on the given inputs.

        The flowsheet state can be used to save important variables
        during calculation for later analysis.

        Both return values are used to check if the heat transfer is valid or not.

        Args:
            inputs (Inputs): The inputs for the calculation.
            fs_state (FlowsheetState): The flowsheet state to save important variables.

        Returns:
            Tuple[float, float]:
                error: Error in percentage between the required and calculated heat flow rates.
                dT_min: Minimal temperature difference (can be negative).
        """
        raise NotImplementedError

    def calc_alpha_two_phase(self, state_q0, state_q1, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Calculate the two-phase heat transfer coefficient.

        Args:
            state_q0: State at vapor quality 0.
            state_q1: State at vapor quality 1.
            inputs (Inputs): The inputs for the calculation.
            fs_state (FlowsheetState): The flowsheet state to save important variables.

        Returns:
            float: The two-phase heat transfer coefficient.
        """
        return self._two_phase_heat_transfer.calc(
            state_q0=state_q0,
            state_q1=state_q1,
            inputs=inputs,
            fs_state=fs_state,
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_inlet=self.state_inlet,
            state_outlet=self.state_outlet
        )

    def calc_alpha_liquid(self, transport_properties) -> float:
        """
        Calculate the liquid-phase heat transfer coefficient.

        Args:
            transport_properties: Transport properties for the liquid phase.

        Returns:
            float: The liquid-phase heat transfer coefficient.
        """
        return self._liquid_heat_transfer.calc(
            transport_properties=transport_properties,
            m_flow=self.m_flow
        )

    def calc_alpha_gas(self, transport_properties) -> float:
        """
        Calculate the gas-phase heat transfer coefficient.

        Args:
            transport_properties: Transport properties for the gas phase.

        Returns:
            float: The gas-phase heat transfer coefficient.
        """
        return self._gas_heat_transfer.calc(
            transport_properties=transport_properties,
            m_flow=self.m_flow
        )

    def calc_alpha_secondary(self, transport_properties) -> float:
        """
        Calculate the secondary-medium heat transfer coefficient.

        Args:
            transport_properties: Transport properties for the secondary medium.

        Returns:
            float: The secondary-medium heat transfer coefficient.
        """
        return self._secondary_heat_transfer.calc(
            transport_properties=transport_properties,
            m_flow=self.m_flow_secondary
        )

    def calc_k(self, alpha_pri: float, alpha_sec: float) -> float:
        """
        Calculate the overall heat transfer coefficient (k) of the heat exchanger.

        Args:
            alpha_pri (float): Heat transfer coefficient for the primary medium.
            alpha_sec (float): Heat transfer coefficient for the secondary medium.

        Returns:
            float: Overall heat transfer coefficient (k).
        """
        k_wall = self.calc_wall_heat_transfer()
        k = (1 / (
                        (1 / alpha_pri) * self.ratio_outer_to_inner_area +
                        (1 / k_wall) * self.ratio_outer_to_inner_area +
                        (1 / alpha_sec)
                )
             )
        return k

    def calc_wall_heat_transfer(self) -> float:
        """
        Calculate the heat transfer coefficient inside the wall.

        Returns:
            float: The wall heat transfer coefficient.
        """
        # Arguments are not required
        return self._wall_heat_transfer.calc(
            transport_properties=media.TransportProperties(),
            m_flow=0
        )

    @property
    def m_flow_secondary(self) -> float:
        return self._m_flow_secondary

    @m_flow_secondary.setter
    def m_flow_secondary(self, m_flow: float):
        self._m_flow_secondary = m_flow
        self._m_flow_secondary_cp = self._m_flow_secondary * self._secondary_cp

    @property
    def m_flow_secondary_cp(self):
        return self._m_flow_secondary_cp

    def calc_secondary_cp(self, T: float, p=None):
        """
        Calculate and set the heat capacity rate m_flow_cp of the secondary medium.

        Args:
            T (float): Temperature of the secondary medium.
            p (float, optional): Pressure of the secondary medium. Defaults to None.
        """
        self._secondary_cp = self.calc_transport_properties_secondary_medium(T=T, p=p).cp
        self._m_flow_secondary_cp = self.m_flow_secondary * self._secondary_cp

    def calc_secondary_Q_flow(self, Q_flow: float) -> float:
        return Q_flow

    def calc_Q_flow(self) -> float:
        """
        Calculate the total heat flow rate.

        Returns:
            float: The total heat flow rate.
        """
        return self.m_flow * abs(self.state_inlet.h - self.state_outlet.h)

    def calc_transport_properties_secondary_medium(self, T, p=None) -> media.TransportProperties:
        """
        Calculate the transport properties for the selected secondary_medium.

        Args:
            T (float): Temperature in K.
            p (float, optional): Pressure to use. Defaults to None.

        Returns:
            media.TransportProperties: The calculated transport properties.
        """
        if p is None:
            if self.secondary_medium == "water":
                p = 2e5  # 2 bar (default hydraulic pressure)
            elif self.secondary_medium == "air":
                p = 101325  # 1 atm
            else:
                raise NotImplementedError(
                    "Default pressures for secondary_mediums aside from water and air are not supported yet."
                )
        # Calc state
        state = self.med_prop_sec.calc_state("PT", p, T)
        # Return properties
        return self.med_prop_sec.calc_transport_properties(state)
