import logging

import numpy as np

from vclibpy.components.heat_exchangers import utils
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers import HeatExchanger

logger = logging.getLogger(__name__)


class MovingBoundaryTmCondenser(HeatExchanger):
    """
    Condenser class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        """
        Calculate the heat exchanger with the Tm-Method based on the given inputs.

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
        self.m_flow_secondary = inputs.condenser.m_flow  # [kg/s]

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = utils.separate_phases(
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_max=self.state_inlet,
            state_min=self.state_outlet,
            p=self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        T_in, T_sc, T_sh, T_out = utils.get_condenser_phase_temperatures_and_alpha(
            heat_exchanger=self, inputs=inputs,
            Q_sc=Q_sc, Q_lat=Q_lat, Q_sh=Q_sh
        )

        tra_prop_med = self.calc_transport_properties_secondary_medium((T_in + T_out) / 2)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # 1. Regime: Subcooling
        Q_sc_Tm, A_sc, A_sc_available = 0, 0, 0
        if not np.isclose(Q_sc, 0) and not np.isclose(state_q0.T, self.state_outlet.T):
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)

            # Only use still available area:
            k_sc = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sc = utils.calc_mean_temperature(
                T_hot_in=state_q0.T, T_hot_out=self.state_outlet.T,
                T_cold_in=T_in, T_cold_out=T_sc
            )
            A_sc = utils.calc_area(Q_sc, k_sc, T_m_sc)
            A_sc_available = min(self.A, A_sc)
            Q_sc_Tm = A_sc_available * k_sc * T_m_sc

        # 2. Regime: Latent heat exchange
        Q_lat_Tm, A_lat, A_lat_available = 0, 0, 0
        if not np.isclose(Q_lat, 0):
            # Get transport properties:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            k_lat = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_lat = utils.calc_mean_temperature(
                T_hot_in=state_q1.T, T_hot_out=state_q0.T,
                T_cold_in=T_sc, T_cold_out=T_sh
            )
            A_lat = utils.calc_area(Q_lat, k_lat, T_m_lat)
            # Only use still available area:
            A_lat_available = min(max(self.A - A_sc_available, 0), A_lat)
            Q_lat_Tm = A_lat_available * k_lat * T_m_lat

            logger.debug(f"con_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Superheat heat exchange
        Q_sh_Tm, A_sh = 0, 0
        if not np.isclose(Q_sh, 0) and not np.isclose(self.state_inlet.T, state_q1.T):
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)

            k_sh = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sh = utils.calc_mean_temperature(
                T_hot_in=self.state_inlet.T, T_hot_out=state_q1.T,
                T_cold_in=T_sh, T_cold_out=T_out
            )
            A_sh = utils.calc_area(Q_sh, k_sh, T_m_sh)
            # Only use still available area:
            A_sh_available = min(max(self.A - A_sc_available - A_lat_available, 0), A_sh)
            Q_sh_Tm = A_sh_available * k_sh * T_m_sh

            logger.debug(f"con_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        A_necessary = A_sh + A_lat + A_sc
        Q_Tm = Q_sh_Tm + Q_sc_Tm + Q_lat_Tm
        error_A = (1 - A_necessary / self.A) * 100
        error = (Q_Tm / Q - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - T_in
        dT_min_out = self.state_inlet.T - T_out
        dT_min_LatSH = state_q1.T - T_sh

        fs_state.set(name="A_con_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in condenser")
        fs_state.set(name="A_con_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in condenser")
        fs_state.set(name="A_con_sc", value=A_sc, unit="m2", description="Area for subcooling heat exchange in condenser")
        fs_state.set(name="error_A", value=error_A, unit="%", description="Area-percentage error for heat exchange in condenser")

        return error, min(dT_min_in,
                          dT_min_LatSH,
                          dT_min_out)


class MovingBoundaryTmEvaporator(HeatExchanger):
    """
    Evaporator class which implements the actual `calc` method.

    Assumptions:
    - No phase changes in secondary medium
    - cp of secondary medium is constant over heat-exchanger

    See parent classes for arguments.
    """

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        """
        Calculate the heat exchanger with the Tm-Method based on the given inputs.

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
        self.m_flow_secondary = inputs.evaporator.m_flow  # [kg/s]

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = utils.separate_phases(
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_max=self.state_outlet,
            state_min=self.state_inlet,
            p=self.state_outlet.p
        )

        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_eva_Tm has to converge to Q_eva (m_ref*delta_h), we can safely
        # calculate the output temperature.
        T_mean = inputs.evaporator.T_in - Q / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)
        # alpha_med_wall = 26

        # Calculate secondary_medium side temperatures:
        T_sh = inputs.evaporator.T_in - Q_sh / self.m_flow_secondary_cp
        T_sc = T_sh - Q_lat / self.m_flow_secondary_cp
        T_out = T_sc - Q_sc / self.m_flow_secondary_cp

        # 1. Regime: Superheating
        Q_sh_Tm, A_sh, A_sh_available = 0, 0, 0
        if not np.isclose(Q_sh, 0) and not np.isclose(self.state_outlet.T, state_q1.T):
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)

            k_sh = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sh = utils.calc_mean_temperature(
                T_hot_in=inputs.evaporator.T_in, T_hot_out=T_sh,
                T_cold_in=state_q1.T, T_cold_out=self.state_outlet.T
            )
            # Only use still available area:
            A_sh = utils.calc_area(Q_sh, k_sh, T_m_sh)
            A_sh_available = min(self.A, A_sh)
            Q_sh_Tm = A_sh_available * k_sh * T_m_sh

            logger.debug(f"eva_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 2. Regime: Latent heat exchange
        Q_lat_Tm, A_lat, A_lat_available = 0, 0, 0
        if not np.isclose(Q_lat, 0):
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            k_lat = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_lat = utils.calc_mean_temperature(
                T_hot_in=T_sh, T_hot_out=T_sc,
                T_cold_in=state_q0.T, T_cold_out=state_q1.T
            )
            A_lat = utils.calc_area(Q_lat, k_lat, T_m_lat)
            # Only use still available area:
            A_lat_available = min(max(self.A - A_sh_available, 0), A_lat)
            Q_lat_Tm = A_lat_available * k_lat * T_m_lat

            logger.debug(f"eva_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Subcooling
        Q_sc_Tm, A_sc = 0, 0
        if not np.isclose(Q_sc, 0) and not np.isclose(state_q0.T, self.state_inlet.T):
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)

            # Only use still available area:
            k_sc = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sc = utils.calc_mean_temperature(
                T_hot_in=T_sc, T_hot_out=T_out,
                T_cold_in=self.state_inlet.T, T_cold_out=state_q0.T
            )
            A_sc = utils.calc_area(Q_sc, k_sc, T_m_sc)
            A_sc_available = min(max(self.A - A_sh_available - A_lat_available, 0), A_sc)
            Q_sc_Tm = A_sc_available * k_sc * T_m_sc

        A_necessary = A_sc + A_lat + A_sh
        Q_Tm = Q_sh_Tm + Q_sc_Tm + Q_lat_Tm
        error_A = (A_necessary / self.A - 1) * 100
        error = (Q_Tm / Q - 1) * 100
        # Get dT_min
        dT_min_in = inputs.evaporator.T_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in evaporator")

        return error, min(dT_min_out, dT_min_in)
