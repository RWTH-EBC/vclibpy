import logging

import numpy as np
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers import HeatExchanger
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)


def calc_area(Q, k, Tm):
    if Tm * k == 0:
        return 1e10  # A really large number, but not np.inf to still be able to calculate with it
    else:
        return Q / (k * Tm)


def calc_mean_temperature(T_hot_in, T_hot_out, T_cold_in, T_cold_out):
    dT_A = T_hot_in - T_cold_out
    dT_B = T_hot_out - T_cold_in
    if dT_B < 0 or dT_A < 0:
        return 0  # Heat can't be transferred
    if np.isclose(dT_B, 0) or np.isclose(dT_A, 0) or np.isclose(dT_B, dT_A):
        return abs((dT_A + dT_B) / 2)
    return (dT_A - dT_B) / np.log(dT_A / dT_B)


def separate_phases(m_flow, med_prop, state_max: ThermodynamicState, state_min: ThermodynamicState, p: float):
    """
    Separates a flow with possible phase changes into three parts:
    subcooling (sc), latent phase change (lat), and superheating (sh)
    at the given pressure.

    Args:
        state_max (ThermodynamicState): State with higher enthalpy.
        state_min (ThermodynamicState): State with lower enthalpy.
        p (float): Pressure of phase change.

    Returns:
        Tuple[float, float, float, ThermodynamicState, ThermodynamicState]:
            Q_sc: Heat for subcooling.
            Q_lat: Heat for latent phase change.
            Q_sh: Heat for superheating.
            state_q0: State at vapor quality 0 and the given pressure.
            state_q1: State at vapor quality 1 and the given pressure.
    """
    # Get relevant states:
    state_q0 = med_prop.calc_state("PQ", p, 0)
    state_q1 = med_prop.calc_state("PQ", p, 1)
    Q_sc = max(0.0,
               min((state_q0.h - state_min.h),
                   (state_max.h - state_min.h))) * m_flow
    Q_lat = max(0.0,
                (min(state_max.h, state_q1.h) -
                 max(state_min.h, state_q0.h))) * m_flow
    Q_sh = max(0.0,
               min((state_max.h - state_q1.h),
                   (state_max.h - state_min.h))) * m_flow
    return Q_sc, Q_lat, Q_sh, state_q0, state_q1


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
        self.m_flow_secondary = inputs.m_flow_con  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_con_in)

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = separate_phases(
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_max=self.state_inlet,
            state_min=self.state_outlet,
            p=self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_con_Tm has to converge to Q_con (m_ref*delta_h), we can safely
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
        Q_sc_Tm, A_sc, A_sc_available = 0, 0, 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)

            # Only use still available area:
            k_sc = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sc = calc_mean_temperature(
                T_hot_in=state_q0.T, T_hot_out=self.state_outlet.T,
                T_cold_in=inputs.T_con_in, T_cold_out=T_sc
            )
            A_sc = calc_area(Q_sc, k_sc, T_m_sc)
            A_sc_available = min(self.A, A_sc)
            Q_sc_Tm = A_sc_available * k_sc * T_m_sc

        # 2. Regime: Latent heat exchange
        Q_lat_Tm, A_lat, A_lat_available = 0, 0, 0
        if Q_lat > 0:
            # Get transport properties:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            k_lat = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_lat = calc_mean_temperature(
                T_hot_in=state_q1.T, T_hot_out=state_q0.T,
                T_cold_in=T_sc, T_cold_out=T_sh
            )
            A_lat = calc_area(Q_lat, k_lat, T_m_lat)
            # Only use still available area:
            A_lat_available = min(max(self.A - A_sc_available, 0), A_lat)
            Q_lat_Tm = A_lat_available * k_lat * T_m_lat

            logger.debug(f"con_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Superheat heat exchange
        Q_sh_Tm, A_sh = 0, 0
        if Q_sh and (self.state_inlet.T != state_q1.T):
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)

            k_sh = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sh = calc_mean_temperature(
                T_hot_in=self.state_inlet.T, T_hot_out=state_q1.T,
                T_cold_in=T_sh, T_cold_out=T_out
            )
            A_sh = calc_area(Q_sh, k_sh, T_m_sh)
            # Only use still available area:
            A_sh_available = min(max(self.A - A_sc_available - A_lat_available, 0), A_sh)
            Q_sh_Tm = A_sh_available * k_sh * T_m_sh

            logger.debug(f"con_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        A_necessary = A_sh + A_lat + A_sc
        Q_Tm = Q_sh_Tm + Q_sc_Tm + Q_lat_Tm
        error_A = (1 - A_necessary / self.A) * 100
        error = (Q_Tm / Q - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - inputs.T_con_in
        dT_min_out = self.state_inlet.T - T_out
        dT_min_LatSH = state_q1.T - T_sh

        fs_state.set(name="A_con_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in condenser")
        fs_state.set(name="A_con_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in condenser")
        fs_state.set(name="A_con_sc", value=A_sc, unit="m2", description="Area for subcooling heat exchange in condenser")

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
        self.m_flow_secondary = inputs.m_flow_eva  # [kg/s]
        self.calc_secondary_cp(T=inputs.T_eva_in)

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = separate_phases(
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_max=self.state_outlet,
            state_min=self.state_inlet,
            p=self.state_outlet.p
        )

        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_eva_Tm has to converge to Q_eva (m_ref*delta_h), we can safely
        # calculate the output temperature.
        T_mean = inputs.T_eva_in - Q / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)
        # alpha_med_wall = 26

        # Calculate secondary_medium side temperatures:
        T_sh = inputs.T_eva_in - Q_sh / self.m_flow_secondary_cp
        T_sc = T_sh - Q_lat / self.m_flow_secondary_cp
        T_out = T_sc - Q_sc / self.m_flow_secondary_cp

        # 1. Regime: Superheating
        Q_sh_Tm, A_sh, A_sh_available = 0, 0, 0
        if Q_sh and (self.state_outlet.T != state_q1.T):
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)

            k_sh = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sh = calc_mean_temperature(
                T_hot_in=inputs.T_eva_in, T_hot_out=T_sh,
                T_cold_in=state_q1.T, T_cold_out=self.state_outlet.T
            )
            # Only use still available area:
            A_sh = calc_area(Q_sh, k_sh, T_m_sh)
            A_sh_available = min(self.A, A_sh)
            Q_sh_Tm = A_sh_available * k_sh * T_m_sh

            logger.debug(f"eva_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 2. Regime: Latent heat exchange
        Q_lat_Tm, A_lat, A_lat_available = 0, 0, 0
        if Q_lat > 0:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            k_lat = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_lat = calc_mean_temperature(
                T_hot_in=T_sh, T_hot_out=T_sc,
                T_cold_in=state_q0.T, T_cold_out=state_q1.T
            )
            A_lat = calc_area(Q_lat, k_lat, T_m_lat)
            # Only use still available area:
            A_lat_available = min(max(self.A - A_sh_available, 0), A_lat)
            Q_lat_Tm = A_lat_available * k_lat * T_m_lat

            logger.debug(f"eva_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Subcooling
        Q_sc_Tm, A_sc = 0, 0
        if Q_sc > 0 and (state_q0.T != self.state_inlet.T):
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)

            # Only use still available area:
            k_sc = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            T_m_sc = calc_mean_temperature(
                T_hot_in=T_sc, T_hot_out=T_out,
                T_cold_in=self.state_inlet.T, T_cold_out=state_q0.T
            )
            A_sc = calc_area(Q_sc, k_sc, T_m_sc)
            A_sc_available = min(max(self.A - A_sh_available - A_lat_available, 0), A_sc)
            Q_sc_Tm = A_sc_available * k_sc * T_m_sc

        A_necessary = A_sc + A_lat + A_sh
        Q_Tm = Q_sh_Tm + Q_sc_Tm + Q_lat_Tm
        error_A = (A_necessary / self.A - 1) * 100
        error = (Q_Tm / Q - 1) * 100
        # Get dT_min
        dT_min_in = inputs.T_eva_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2", description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in evaporator")

        return error, min(dT_min_out, dT_min_in)
