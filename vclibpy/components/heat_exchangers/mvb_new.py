import abc
import logging
import math
import numpy as np

from vclibpy.components.component import BaseComponent
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.heat_exchanger import HeatExchanger
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer
from vclibpy.media import ThermodynamicState, MedProp



class BasicHX(HeatExchanger, abc.ABC):

    def __init__(self,
                 flow_type: str,
                 ratio_outer_to_inner_area: float,
                 model_approach:str,
                 gas_heat_transfer: HeatTransfer,
                 liquid_heat_transfer: HeatTransfer,
                 two_phase_heat_transfer: TwoPhaseHeatTransfer,
                 **kwargs):
        """
        Initializes BasicLMTD.

        Args:
            flow_type (str): Type of flow: Counter, Cross, or Parallel.
            ratio_outer_to_inner_area (float):
                The NTU method uses the overall heat transfer coefficient `k`
                and multiplies it with the overall area `A`.
                However, depending on the heat exchanger type, the areas may
                differ drastically. For instance in an air-to-water heat exchanger.
                The VDI-Atlas proposes the ratio of outer area to inner pipe area
                to account for this mismatch in sizes.
                The calculation follows the code in the function `calc_k`.
            **kwargs: Additional keyword arguments passed to the parent class.
        """
        super(BasicHX, self).__init__(**kwargs)
        self.ratio_outer_to_inner_area = ratio_outer_to_inner_area

        self._gas_heat_transfer = gas_heat_transfer
        self._liquid_heat_transfer = liquid_heat_transfer
        self._two_phase_heat_transfer = two_phase_heat_transfer

        # Type of HE:
        self.flow_type = flow_type.lower()
        if self.flow_type not in ["counter", "cross", "parallel"]:
            raise TypeError("Given flow_type is not supported")
        self.model_approach = model_approach.lower()
        if self.model_approach not in ["ntu", "lmtd"]:
            raise TypeError("Given model approach is not supported")

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

    def calc_U(self, alpha_pri: float, alpha_sec: float) -> float:
        """
        Calculate the overall heat transfer coefficient (k) of the heat exchanger.

        Args:
            alpha_pri (float): Heat transfer coefficient for the primary medium.
            alpha_sec (float): Heat transfer coefficient for the secondary medium.

        Returns:
            float: Overall heat transfer coefficient (k).
        """
        k_wall = self.calc_wall_heat_transfer()
        return 1 / (
                (1 / alpha_pri) * self.ratio_outer_to_inner_area +
                (1 / k_wall) * self.ratio_outer_to_inner_area +
                (1 / alpha_sec))

    def calc_lmtd(self, T_prim_in, T_prim_out, T_sec_in, T_sec_out):

        dT_in = T_sec_in - T_prim_out
        dT_out = T_sec_out - T_prim_in

        if dT_in * dT_out <= 0:
            return 0.000001
        if dT_out == dT_in:
            return dT_out
        lmtd = (dT_in - dT_out) / math.log((dT_in / dT_out))

        return abs(lmtd)

    def calc_NTU(
            self,
            C_prim,
            C_sec,
            Q,
            T_prim_in,
            T_prim_out,
            T_sec_in,
            T_sec_out,
            flow_type):
        dT_in = abs(T_sec_in - T_prim_out)
        dT_out = abs(T_sec_out - T_prim_in)
        eps = Q / (min(C_prim, C_sec) * max(dT_in, dT_out))
        if np.isinf(C_prim):
            return -math.log(1 - eps)
        if C_sec / C_prim == 1:
            return eps / (1 - eps)
        if C_sec / C_prim < 1:
            Y = C_sec / C_prim
        else:
            Y = C_prim / C_sec
        return -math.log((1 - eps) / (1 - Y * eps)) / (1 - Y)


    def separate_phases(self, state_max: ThermodynamicState, state_min: ThermodynamicState, p: float):
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
        state_q0 = self.med_prop.calc_state("PQ", p, 0)
        state_q1 = self.med_prop.calc_state("PQ", p, 1)
        Q_sc = max(0.0,
                   min((state_q0.h - state_min.h),
                       (state_max.h - state_min.h))) * self.m_flow
        Q_lat = max(0.0,
                    (min(state_max.h, state_q1.h) -
                     max(state_min.h, state_q0.h))) * self.m_flow
        Q_sh = max(0.0,
                   min((state_max.h - state_q1.h),
                       (state_max.h - state_min.h))) * self.m_flow
        return Q_sc, Q_lat, Q_sh, state_q0, state_q1

class MVB_Condenser(BasicHX, abc.ABC):

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):

        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_inlet,
            self.state_outlet,
            self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        T_mean = inputs.T_con_in + self.calc_secondary_Q_flow(Q) / (self.m_flow_secondary_cp * 2)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        T_in_sec = inputs.T_con_in
        T_out_sec = inputs.T_con_out
        dT_sec =T_out_sec - T_in_sec
        dT_sec_sc = dT_sec * (Q_sc/Q)
        dT_sec_lat = dT_sec * (Q_lat/Q)
        dT_sec_sh = dT_sec *(Q_sh/Q)

        T_sec_sc_in = T_in_sec
        T_sec_sc_out = T_sec_sc_in + dT_sec_sc
        T_sec_lat_in = T_sec_sc_out
        T_sec_lat_out = T_sec_lat_in + dT_sec_lat
        T_sec_sh_in = T_sec_lat_out
        T_sec_sh_out = T_sec_lat_out + dT_sec_sh

        T_ref_sh_in = self.state_inlet.T
        T_ref_sh_out = state_q1.T
        T_ref_lat_in = state_q1.T
        T_ref_lat_out = state_q0.T
        T_ref_sc_in = state_q0.T
        T_ref_sc_out = self.state_outlet.T

        dT_ref_sh = T_ref_sh_in - T_ref_sh_out
        dT_ref_lat = T_ref_lat_in - T_ref_lat_out
        dT_ref_sc = T_ref_sc_in - T_ref_sc_out

        A_sc = 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)
            U = self.calc_U(alpha_pri=alpha_ref_wall,
                            alpha_sec=alpha_med_wall)

            if self.model_approach.lower() == "ntu":
                W_ref = Q_sc / dT_ref_sc
                W_sec = Q_sc / dT_sec_sc
                NTU = self.calc_NTU(
                    C_sec=W_sec,
                    C_prim=W_ref,
                    T_prim_in=T_ref_sc_in,
                    T_prim_out=T_ref_sc_out,
                    T_sec_in=T_sec_sc_in,
                    T_sec_out=T_sec_sc_out,
                    Q=Q_sh,
                    flow_type=self.flow_type)
                A_sc = min(W_ref,W_sec)*NTU/U
            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_sc_in,
                    T_prim_out=T_ref_sc_out,
                    T_sec_in=T_sec_sc_in,
                    T_sec_out=T_sec_sc_out)
                A_sc = Q_sc/(lmtd*U)
        A_lat = 0
        if Q_lat > 0:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            U = self.calc_U(alpha_pri=alpha_ref_wall,
                            alpha_sec=alpha_med_wall)

            if self.model_approach.lower() == "ntu":
                if dT_ref_lat >0.000001:
                    W_ref = Q_lat / dT_ref_lat
                else:
                    W_ref = np.inf
                W_sec = Q_lat / dT_sec_lat
                NTU = self.calc_NTU(
                    C_sec=W_sec,
                    C_prim=W_ref,
                    T_prim_in=T_ref_lat_in,
                    T_prim_out=T_ref_lat_out,
                    T_sec_in=T_sec_lat_in,
                    T_sec_out=T_sec_lat_out,
                    Q=Q_lat,
                    flow_type=self.flow_type)
                A_lat = min(W_ref,W_sec)*NTU/U
            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_lat_in,
                    T_prim_out=T_ref_lat_out,
                    T_sec_in=T_sec_lat_in,
                    T_sec_out=T_sec_lat_out)
                A_lat = Q_lat/(lmtd*U)
        A_sh = 0
        if Q_sh > 0 and (self.state_inlet.T != state_q1.T):
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)
            U = self.calc_U(alpha_pri=alpha_ref_wall,
                            alpha_sec=alpha_med_wall)

            if self.model_approach.lower() == "ntu":

                W_ref = Q_sh / dT_ref_sh
                W_sec = Q_sh / dT_sec_sh
                NTU = self.calc_NTU(
                    C_sec=W_sec,
                    C_prim=W_ref,
                    T_prim_in=T_ref_sh_in,
                    T_prim_out=T_ref_sh_out,
                    T_sec_in=T_sec_sh_in,
                    T_sec_out=T_sec_sh_out,
                    Q=Q_lat,
                    flow_type=self.flow_type)
                A_lat = min(W_ref,W_sec)*NTU/U

            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_sh_in,
                    T_prim_out=T_ref_sh_out,
                    T_sec_in=T_sec_sh_in,
                    T_sec_out=T_sec_sh_out)
                A_sh = Q_sh / (lmtd * U)

        A_lmtd = A_sh + A_lat + A_sc
        error = (self.A / A_lmtd - 1) * 100
        # Get possible dT_min:
        dT_min_lat = T_ref_lat_in - T_sec_lat_out
        dT_min_sc = T_ref_sc_out - T_sec_sc_in
        dT_min_sh = T_ref_sh_in - T_sec_sh_out
        dT_min_Lat_out = T_ref_lat_out - T_sec_lat_in

        return error, min(dT_min_lat,
                          dT_min_sc,
                          dT_min_sh,
                          dT_min_Lat_out)



















