import abc
import logging

import numpy as np
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.ntu import BasicNTU

from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer

logger = logging.getLogger(__name__)


class SimpleNTU(BasicNTU, abc.ABC):
    """
    Moving boundary NTU based heat exchanger.

    See parent classe for arguments.
    """
    def __init__(self,
                 flow_type: str,
                 ratio_outer_to_inner_area: float,
                 primary_heat_transfer: HeatTransfer,
                 **kwargs):

        super(SimpleNTU, self).__init__(flow_type,
                                        ratio_outer_to_inner_area,
                                        **kwargs)
        self._primary_heat_transfer = primary_heat_transfer

        return

    def calc_alpha_primary(self, transport_properties) -> float:
        """
        Calculate the primary-medium heat transfer coefficient.

        Args:
            transport_properties: Transport properties for the secondary medium.

        Returns:
            float: The secondary-medium heat transfer coefficient.
        """
        return self._primary_heat_transfer.calc(
            transport_properties=transport_properties,
            m_flow=self.m_flow_secondary
        )

class SimpleNTUCondenser(SimpleNTU):
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

        self.set_primary_cp((self.state_inlet.h - self.state_outlet.h) / (self.state_inlet.T - self.state_outlet.T))
        tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
        alpha_ref_wall = self.calc_alpha_primary(tra_prop_ref_con)

        A = self.iterate_area(dT_max=(self.state_inlet.T - inputs.T_con_in),
                              alpha_pri=alpha_ref_wall,
                              alpha_sec=alpha_med_wall,
                              Q=Q)
        A = min(self.A, A)

        Q_ntu, k = self.calc_Q_ntu(dT_max=(self.state_inlet.T - inputs.T_con_in),
                                   alpha_pri=alpha_ref_wall,
                                   alpha_sec=alpha_med_wall,
                                   A=A)

        error = (Q_ntu / Q - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - inputs.T_con_in
        dT_min_out = self.state_inlet.T - T_out
        dT_min_LatSH = state_q1.T - T_sh

        A_sh = A * (Q_sh / Q)
        A_lat = A * (Q_lat / Q)
        A_sc = A * (Q_sc / Q)

        fs_state.set(name="A_con_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in condenser")
        fs_state.set(name="A_con_lat", value=A_lat, unit="m2", description="Area for latent heat exchange in condenser")
        fs_state.set(name="A_con_sc", value=A_sc, unit="m2",
                     description="Area for subcooling heat exchange in condenser")

        return error, min(dT_min_in,
                          dT_min_LatSH,
                          dT_min_out)


class SimpleNTUEvaporator(SimpleNTU):
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
        # Assumption loss is the same correlation for each regime
        T_sh = inputs.T_eva_in - Q_sh / self.m_flow_secondary_cp
        T_sc = T_sh - Q_lat / self.m_flow_secondary_cp
        T_out = T_sc - Q_sc / self.m_flow_secondary_cp

        self.set_primary_cp((self.state_inlet.h - self.state_outlet.h) / (self.state_inlet.T - self.state_outlet.T))
        tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
        alpha_ref_wall = self.calc_alpha_primary(tra_prop_ref_eva)

        A = self.iterate_area(dT_max=(inputs.T_eva_in - self.state_inlet.T),
                              alpha_pri=alpha_ref_wall,
                              alpha_sec=alpha_med_wall,
                              Q=Q)
        A = min(self.A, A)

        Q_ntu, k = self.calc_Q_ntu(dT_max=(inputs.T_eva_in - self.state_inlet.T),
                                   alpha_pri=alpha_ref_wall,
                                   alpha_sec=alpha_med_wall,
                                   A=A)

        error = (Q_ntu / Q - 1) * 100
        # Get dT_min

        A_sh = A * (Q_sh / Q)
        A_lat = A * (Q_lat / Q)
        A_sc = A * (Q_sc / Q)

        dT_min_in = inputs.T_eva_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

        fs_state.set(name="A_eva_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="A_eva_lat", value=A_lat, unit="m2",
                     description="Area for latent heat exchange in evaporator")

        return error, min(dT_min_out, dT_min_in)

