import logging

import numpy as np
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers import ntu, ExternalHeatExchanger
from vclibpy.components.heat_exchangers.utils import separate_phases, get_condenser_phase_temperatures_and_alpha

logger = logging.getLogger(__name__)


def iterate_area(heat_exchanger: ExternalHeatExchanger, dT_max, alpha_pri, alpha_sec, Q) -> float:
    """
    Iteratively calculates the required area for the heat exchange.

    Args:
        heat_exchanger (BasicNTU): An instance of the BasicNTU or children classes
        dT_max (float): Maximum temperature differential.
        alpha_pri (float): Heat transfer coefficient for the primary medium.
        alpha_sec (float): Heat transfer coefficient for the secondary medium.
        Q (float): Heat flow rate.

    Returns:
        float: Required area for heat exchange.
    """
    _accuracy = 1e-6  # square mm
    _step = 1.0
    m_flow_primary_cp = heat_exchanger.m_flow_secondary_cp
    m_flow_secondary_cp = heat_exchanger.m_flow_secondary_cp
    R = ntu.calc_R(m_flow_primary_cp, m_flow_secondary_cp)
    k = heat_exchanger.calc_k(alpha_pri, alpha_sec)
    m_flow_cp_min = ntu.calc_m_flow_cp_min(m_flow_primary_cp, m_flow_secondary_cp)
    # First check if point is feasible at all
    if dT_max <= 0:
        return heat_exchanger.A
    eps_necessary = Q / (m_flow_cp_min * dT_max)

    # Special cases:
    # ---------------
    # eps is equal or higher than 1, an infinite amount of area would be necessary.
    if eps_necessary >= 1:
        return heat_exchanger.A
    # eps is lower or equal to zero: No Area required (Q<=0)
    if eps_necessary <= 0:
        return 0

    area = 0.0
    while True:
        if heat_exchanger.flow_type == "cross" and area == 0.0:
            eps = 0.0
        else:
            NTU = ntu.calc_NTU(area, k, m_flow_cp_min)
            eps = ntu.calc_eps(R, NTU, heat_exchanger.flow_type)
        if eps >= eps_necessary:
            if _step <= _accuracy:
                break
            else:
                # Go back
                area -= _step
                _step /= 10
                continue
        if _step < _accuracy and area > heat_exchanger.A:
            break
        area += _step

    return min(area, heat_exchanger.A)


class MovingBoundaryNTUCondenser(ExternalHeatExchanger):
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
        self.m_flow_secondary = inputs.condenser.m_flow  # [kg/s]

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = separate_phases(
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_max=self.state_inlet,
            state_min=self.state_outlet,
            p=self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        T_in, T_sc, T_sh, T_out = get_condenser_phase_temperatures_and_alpha(
            heat_exchanger=self, inputs=inputs,
            Q_sc=Q_sc, Q_lat=Q_lat, Q_sh=Q_sh
        )

        tra_prop_med = self.calc_transport_properties_secondary_medium((T_in + T_out) / 2)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # 1. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        if not np.isclose(Q_sc, 0) and not np.isclose(state_q0.T, self.state_outlet.T):
            primary_cp = ((state_q0.h - self.state_outlet.h) / (state_q0.T - self.state_outlet.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)

            # Only use still available area:
            A_sc = iterate_area(heat_exchanger=self,
                                dT_max=(state_q0.T - T_in),
                                alpha_pri=alpha_ref_wall,
                                alpha_sec=alpha_med_wall,
                                Q=Q_sc)
            A_sc = min(self.A, A_sc)

            k_sc = self.calc_k(alpha_pri=alpha_ref_wall, alpha_sec=alpha_med_wall)
            Q_sc_ntu = ntu.calc_Q_ntu(
                dT_max=(state_q0.T - T_in),
                k=k_sc,
                m_flow_secondary_cp=self.m_flow_secondary_cp,
                m_flow_primary_cp=self.m_flow * primary_cp,
                flow_type=self.flow_type,
                A=A_sc
            )

        # 2. Regime: Latent heat exchange
        Q_lat_ntu, A_lat = 0, 0
        if not np.isclose(Q_lat, 0):
            primary_cp = np.inf
            # Get transport properties:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

            A_lat = iterate_area(heat_exchanger=self,
                                 dT_max=(state_q1.T - T_sc),
                                 alpha_pri=alpha_ref_wall,
                                 alpha_sec=alpha_med_wall,
                                 Q=Q_lat)
            # Only use still available area:
            A_lat = min(self.A - A_sc, A_lat)

            k_lat = self.calc_k(
                alpha_pri=alpha_ref_wall,
                alpha_sec=alpha_med_wall
            )
            Q_lat_ntu = ntu.calc_Q_ntu(
                dT_max=(state_q1.T - T_sc),
                k=k_lat,
                m_flow_secondary_cp=self.m_flow_secondary_cp,
                m_flow_primary_cp=self.m_flow * primary_cp,
                flow_type=self.flow_type,
                A=A_lat
            )
            logger.debug(f"con_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Superheat heat exchange
        Q_sh_ntu, A_sh = 0, 0
        if not np.isclose(Q_sh, 0) and not np.isclose(self.state_inlet.T, state_q1.T):
            primary_cp = ((self.state_inlet.h - state_q1.h) / (self.state_inlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)

            # Only use still available area:
            A_sh = self.A - A_sc - A_lat

            k_sh = self.calc_k(
                alpha_pri=alpha_ref_wall,
                alpha_sec=alpha_med_wall
            )
            Q_sh_ntu = ntu.calc_Q_ntu(
                dT_max=(self.state_inlet.T - T_sh),
                k=k_sh,
                m_flow_secondary_cp=self.m_flow_secondary_cp,
                m_flow_primary_cp=self.m_flow * primary_cp,
                flow_type=self.flow_type,
                A=A_sh
            )
            logger.debug(f"con_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        Q_ntu = Q_sh_ntu + Q_sc_ntu + Q_lat_ntu
        error = (Q_ntu / Q - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - T_in
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


class MovingBoundaryNTUEvaporator(ExternalHeatExchanger):
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
        self.m_flow_secondary = inputs.evaporator.m_flow  # [kg/s]

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = separate_phases(
            m_flow=self.m_flow,
            med_prop=self.med_prop,
            state_max=self.state_outlet,
            state_min=self.state_inlet,
            p=self.state_outlet.p
        )

        Q = Q_sc + Q_lat + Q_sh

        # Note: As Q_eva_ntu has to converge to Q_eva (m_ref*delta_h), we can safely
        # calculate the output temperature.
        T_mean = inputs.evaporator.T_in - Q / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        # Calculate secondary_medium side temperatures:
        T_sh = inputs.evaporator.T_in - Q_sh / self.m_flow_secondary_cp
        T_sc = T_sh - Q_lat / self.m_flow_secondary_cp
        T_out = T_sc - Q_sc / self.m_flow_secondary_cp

        # 1. Regime: Superheating
        Q_sh_ntu, A_sh = 0, 0
        if not np.isclose(Q_sh, 0) and not np.isclose(self.state_outlet.T, state_q1.T):
            primary_cp = ((self.state_outlet.h - state_q1.h) / (self.state_outlet.T - state_q1.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)

            if Q_lat > 0:
                A_sh = iterate_area(heat_exchanger=self,
                                    dT_max=(inputs.evaporator.T_in - state_q1.T),
                                    alpha_pri=alpha_ref_wall,
                                    alpha_sec=alpha_med_wall,
                                    Q=Q_sh)
            else:
                # if only sh is present --> full area:
                A_sh = self.A

            # Only use still available area
            A_sh = min(self.A, A_sh)

            k_sh = self.calc_k(
                alpha_pri=alpha_ref_wall,
                alpha_sec=alpha_med_wall
            )
            Q_sh_ntu = ntu.calc_Q_ntu(
                dT_max=(inputs.evaporator.T_in - state_q1.T),
                A=A_sh,
                k=k_sh,
                m_flow_secondary_cp=self.m_flow_secondary_cp,
                m_flow_primary_cp=self.m_flow * primary_cp,
                flow_type=self.flow_type
            )

            logger.debug(f"eva_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 2. Regime: Latent heat exchange
        Q_lat_ntu, A_lat = 0, 0
        if not np.isclose(Q_lat, 0):
            primary_cp = np.inf

            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

            if Q_sc > 0:
                A_lat = iterate_area(heat_exchanger=self, dT_max=(T_sh - self.state_inlet.T),
                                     alpha_pri=alpha_ref_wall,
                                     alpha_sec=alpha_med_wall,
                                     Q=Q_lat)
            else:
                A_lat = self.A - A_sh

            # Only use still available area:
            k_lat = self.calc_k(
                alpha_pri=alpha_ref_wall,
                alpha_sec=alpha_med_wall
            )
            A_lat = min(self.A - A_sh, A_lat)
            Q_lat_ntu = ntu.calc_Q_ntu(
                dT_max=(T_sh - self.state_inlet.T),
                k=k_lat,
                m_flow_secondary_cp=self.m_flow_secondary_cp,
                m_flow_primary_cp=self.m_flow * primary_cp,
                flow_type=self.flow_type,
                A=A_lat
            )
            logger.debug(f"eva_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Subcooling
        Q_sc_ntu, A_sc = 0, 0
        if not np.isclose(Q_sc, 0) and not np.isclose(state_q0.T, self.state_inlet.T):
            primary_cp = ((state_q0.h - self.state_inlet.h) / (state_q0.T - self.state_inlet.T))
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)

            # Only use still available area:
            A_sc = self.A - A_sh - A_lat

            k_sc = self.calc_k(
                alpha_pri=alpha_ref_wall,
                alpha_sec=alpha_med_wall
            )
            Q_sc_ntu = ntu.calc_Q_ntu(
                dT_max=(T_sc - self.state_inlet.T),
                A=A_sc,
                k=k_sc,
                m_flow_secondary_cp=self.m_flow_secondary_cp,
                m_flow_primary_cp=self.m_flow * primary_cp,
                flow_type=self.flow_type
            )

        Q_ntu = Q_sh_ntu + Q_sc_ntu + Q_lat_ntu
        error = (Q_ntu / Q - 1) * 100
        # Get dT_min
        dT_min_in = inputs.evaporator.T_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2",
                     description="Area for latent heat exchange in evaporator")

        return error, min(dT_min_out, dT_min_in)
