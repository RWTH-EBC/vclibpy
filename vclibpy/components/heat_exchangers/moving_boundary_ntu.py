import abc
import logging

import numpy as np
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.ntu import BasicNTU

from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer

logger = logging.getLogger(__name__)


class MovingBoundaryNTU(BasicNTU, abc.ABC):
    """
    Moving boundary NTU based heat exchanger.

    See parent classe for arguments.
    """

    def __init__(self,
                 flow_type: str,
                 ratio_outer_to_inner_area: float,
                 gas_heat_transfer: HeatTransfer,
                 liquid_heat_transfer: HeatTransfer,
                 two_phase_heat_transfer: TwoPhaseHeatTransfer,
                 **kwargs):
        super(MovingBoundaryNTU, self).__init__(flow_type,
                                                ratio_outer_to_inner_area,
                                                **kwargs)

        self._gas_heat_transfer = gas_heat_transfer
        self._liquid_heat_transfer = liquid_heat_transfer
        self._two_phase_heat_transfer = two_phase_heat_transfer

        return

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


class MovingBoundaryNTUCondenser(MovingBoundaryNTU):
    """
    Condenser class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        """
        Calculate the heat exchanger with the NTU-Method based on the given inputs.

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
        self.m_flow_secondary = inputs.m_flow_con  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_con_in)

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_inlet,
            self.state_outlet,
            self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_con_ntu has to converge to Q_con (m_ref*delta_h), we can safely
        # calculate the output temperature.

        T_mean = inputs.T_con_in + self.calc_secondary_Q_flow(Q) / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # Calculate secondary_medium side temperatures:
        # Assumption loss is the same correlation for each regime
        T_sc = inputs.T_con_in + self.calc_secondary_Q_flow(Q_sc) / self.m_flow_secondary_cp
        T_sh = T_sc + self.calc_secondary_Q_flow(Q_lat) / self.m_flow_secondary_cp
        T_out = T_sh + self.calc_secondary_Q_flow(Q_sh) / self.m_flow_secondary_cp

        # 1. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            self.set_primary_cp((state_q0.h - self.state_outlet.h) / (state_q0.T - self.state_outlet.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)

            # Only use still available area:
            A_sc = self.iterate_area(dT_max=(state_q0.T - inputs.T_con_in),
                                     alpha_pri=alpha_ref_wall,
                                     alpha_sec=alpha_med_wall,
                                     Q=Q_sc)
            A_sc = min(self.A, A_sc)

            Q_sc_ntu, k_sc = self.calc_Q_ntu(dT_max=(state_q0.T - inputs.T_con_in),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sc)

        # 2. Regime: Latent heat exchange
        Q_lat_ntu, A_lat = 0, 0
        if Q_lat > 0:
            self.set_primary_cp(np.inf)
            # Get transport properties:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

            A_lat = self.iterate_area(dT_max=(state_q1.T - T_sc),
                                      alpha_pri=alpha_ref_wall,
                                      alpha_sec=alpha_med_wall,
                                      Q=Q_lat)
            # Only use still available area:
            A_lat = min(self.A - A_sc, A_lat)

            Q_lat_ntu, k_lat = self.calc_Q_ntu(dT_max=(state_q1.T - T_sc),
                                               alpha_pri=alpha_ref_wall,
                                               alpha_sec=alpha_med_wall,
                                               A=A_lat)
            logger.debug(f"con_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Superheat heat exchange
        Q_sh_ntu, A_sh = 0, 0
        if Q_sh and (self.state_inlet.T != state_q1.T):
            self.set_primary_cp((self.state_inlet.h - state_q1.h) / (self.state_inlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)

            # Only use still available area:
            A_sh = self.A - A_sc - A_lat

            Q_sh_ntu, k_sh = self.calc_Q_ntu(dT_max=(self.state_inlet.T - T_sh),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sh)
            logger.debug(f"con_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        Q_ntu = Q_sh_ntu + Q_sc_ntu + Q_lat_ntu
        error = (Q_ntu / Q - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - inputs.T_con_in
        dT_min_out = self.state_inlet.T - T_out
        dT_min_LatSH = state_q1.T - T_sh

        fs_state.set(name="A_con_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in condenser")
        fs_state.set(name="A_con_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in condenser")
        fs_state.set(name="A_con_sc", value=A_sc, unit="m2",
                     description="Area for subcooling heat exchange in condenser")

        return error, min(dT_min_in,
                          dT_min_LatSH,
                          dT_min_out)


class MovingBoundaryNTUEvaporator(MovingBoundaryNTU):
    """
    Evaporator class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        """
        Calculate the heat exchanger with the NTU-Method based on the given inputs.

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
        self.m_flow_secondary = inputs.m_flow_eva  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_eva_in)

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_outlet,
            self.state_inlet,
            self.state_inlet.p
        )

        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_eva_ntu has to converge to Q_eva (m_ref*delta_h), we can safely
        # calculate the output temperature.
        T_mean = inputs.T_eva_in + Q / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # Calculate secondary_medium side temperatures:
        T_sh = inputs.T_eva_in - Q_sh / self.m_flow_secondary_cp
        T_sc = T_sh - Q_lat / self.m_flow_secondary_cp
        T_out = T_sc - Q_sc / self.m_flow_secondary_cp

        # 1. Regime: Superheating
        Q_sh_ntu, A_sh = 0, 0
        if Q_sh and (self.state_outlet.T != state_q1.T):
            self.set_primary_cp((self.state_outlet.h - state_q1.h) / (self.state_outlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)

            if Q_lat > 0:
                A_sh = self.iterate_area(dT_max=(inputs.T_eva_in - state_q1.T),
                                         alpha_pri=alpha_ref_wall,
                                         alpha_sec=alpha_med_wall,
                                         Q=Q_sh)
            else:
                # if only sh is present --> full area:
                A_sh = self.A

            # Only use still available area
            A_sh = min(self.A, A_sh)

            Q_sh_ntu, k_sh = self.calc_Q_ntu(dT_max=(inputs.T_eva_in - state_q1.T),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sh)

            logger.debug(f"eva_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 2. Regime: Latent heat exchange
        Q_lat_ntu, A_lat = 0, 0
        if Q_lat > 0:
            self.set_primary_cp(np.inf)

            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

            if Q_sc > 0:
                A_lat = self.iterate_area(dT_max=(T_sh - self.state_inlet.T),
                                          alpha_pri=alpha_ref_wall,
                                          alpha_sec=alpha_med_wall,
                                          Q=Q_lat)
            else:
                A_lat = self.A - A_sh

            # Only use still available area:
            A_lat = min(self.A - A_sh, A_lat)
            Q_lat_ntu, k_lat = self.calc_Q_ntu(dT_max=(T_sh - self.state_inlet.T),
                                               alpha_pri=alpha_ref_wall,
                                               alpha_sec=alpha_med_wall,
                                               A=A_lat)
            logger.debug(f"eva_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        if Q_sc > 0 and (state_q0.T != self.state_inlet.T):
            self.set_primary_cp((state_q0.h - self.state_inlet.h) / (state_q0.T - self.state_inlet.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)

            # Only use still available area:
            A_sc = self.A - A_sh - A_lat

            Q_sc_ntu, k_sc = self.calc_Q_ntu(dT_max=(T_sc - self.state_inlet.T),
                                             alpha_pri=alpha_ref_wall,
                                             alpha_sec=alpha_med_wall,
                                             A=A_sc)

        Q_ntu = Q_sh_ntu + Q_sc_ntu + Q_lat_ntu
        error = (Q_ntu / Q - 1) * 100
        # Get dT_min
        dT_min_in = inputs.T_eva_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2",
                     description="Area for latent heat exchange in evaporator")

        return error, min(dT_min_out, dT_min_in)
