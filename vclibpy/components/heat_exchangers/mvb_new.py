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
                 n_elemente = 1,
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

        self.n_elemente = n_elemente

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

    def detailed_epsNTU(self,
        dh,
        Qdot,
        dT_sec,
        state_in,
        T_sec_out,
        W_sec,
        U,
        n_elements=1):

        dh_element = dh / n_elements
        dT_sec_element = dT_sec / n_elements
        Qdot_element = Qdot / n_elements
        state_in_element = state_in
        T_sec_in_element = T_sec_out - dT_sec-273.15

        A = 0
        for i in range(n_elements):
            state_out_element = self.med_prop.calc_state("PH", state_in.p, state_in_element.h - dh_element)
            T_ref_in_element = state_in_element.T-273.15
            T_ref_out_element = state_out_element.T-273.15
            dT_ref_element = abs(T_ref_in_element - T_ref_out_element)

            if dT_ref_element < 0.00001:
                W_prim = np.inf
            else:
                W_prim = Qdot_element / dT_ref_element
            NTU = self.calc_NTU(
                W_prim=W_prim,
                W_sec=W_sec,
                Q=Qdot_element,
                T_prim_in=T_ref_in_element,
                T_sec_in=T_sec_in_element)
            A += min(W_sec, W_prim) * NTU / U
            state_in_element = state_out_element
            T_sec_in_element -= dT_sec_element
        return A

    def calc_NTU(
            self,
            W_prim,
            W_sec,
            Q,
            T_prim_in,
            T_sec_in):

        dT_max = abs(T_prim_in - T_sec_in)
        W_1 = min(W_prim, W_sec)
        W_2 = max(W_prim, W_sec)
        P = Q / (W_1 * dT_max)
        if np.isinf(W_2):
            return -math.log(1 - P)
        R = W_1 / W_2
        if self.flow_type.lower() == "counter":
            return math.log((1 - R * P) / (1 - P)) / (1 - R)
        if self.flow_type.lower() == "cross":
            return - 1 / R * math.log(1 + R * math.log(1 - P))

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
        self.m_flow_secondary = inputs.m_flow_con
        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_inlet,
            self.state_outlet,
            self.state_inlet.p
        )
        Q = Q_sc + Q_lat + Q_sh

        T_mean = 0.5*(inputs.T_con_in + inputs.T_con_out)
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

        dT_min_lat = T_ref_lat_in - T_sec_lat_out
        dT_min_sc = T_ref_sc_out - T_sec_sc_in
        dT_min_sh = T_ref_sh_in - T_sec_sh_out
        dT_min_Lat_out = T_ref_lat_out - T_sec_lat_in

        pinch = (
            min(
                dT_min_lat,
                dT_min_sc,
                dT_min_sh,
                dT_min_Lat_out)
        )
        if pinch < 0:
            return -100, -10
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
            fs_state.set(name="Con_alpha_lat", value=alpha_ref_wall)
            fs_state.set(name="Con_U_lat", value=U)
            if self.model_approach.lower() == "ntu":
                W_sec = Q_lat / dT_sec_lat
                A_lat = self.detailed_epsNTU(
                    dh=state_q1.h - state_q0.h,
                    Qdot=Q_lat,
                    dT_sec=dT_sec_lat,
                    state_in=state_q1,
                    T_sec_out=T_sec_lat_out,
                    W_sec=W_sec,
                    U=U
                )
            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_lat_in,
                    T_prim_out=T_ref_lat_out,
                    T_sec_in=T_sec_lat_in,
                    T_sec_out=T_sec_lat_out)
                A_lat = Q_lat/(lmtd*U)
            if A_lat > self.A:
                return -100, -10

        A_sc = 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)
            U = self.calc_U(alpha_pri=alpha_ref_wall,
                            alpha_sec=alpha_med_wall)
            fs_state.set(name="Con_alpha_sc", value=alpha_ref_wall)
            fs_state.set(name="Con_U_sc", value=U)
            if self.model_approach.lower() == "ntu":

                W_sec = Q_sc / dT_sec_sc

                A_sc = self.detailed_epsNTU(
                    dh= state_q0.h - self.state_outlet.h,
                    Qdot=Q_sc,
                    dT_sec=dT_sec_sc,
                    state_in=state_q0,
                    T_sec_out=T_sec_sc_out,
                    W_sec=W_sec,
                    U=U
                )

            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_sc_in,
                    T_prim_out=T_ref_sc_out,
                    T_sec_in=T_sec_sc_in,
                    T_sec_out=T_sec_sc_out)
                A_sc = Q_sc/(lmtd*U)

        A_sh = 0
        if Q_sh > 0 and (self.state_inlet.T != state_q1.T):
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)
            U = self.calc_U(alpha_pri=alpha_ref_wall,
                            alpha_sec=alpha_med_wall)
            fs_state.set(name="Con_alpha_sh", value=alpha_ref_wall)
            fs_state.set(name="Con_U_sh", value=U)
            if self.model_approach.lower() == "ntu":
                W_sec = Q_sh / dT_sec_sh
                A_sh = self.detailed_epsNTU(
                    dh=self.state_inlet.h - state_q1.h,
                    Qdot=Q_sh,
                    dT_sec=dT_sec_sh,
                    state_in=self.state_inlet,
                    T_sec_out=T_sec_sh_out,
                    W_sec=W_sec,
                    U=U,
                    n_elements=self.n_elemente
                )

            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_sh_in,
                    T_prim_out=T_ref_sh_out,
                    T_sec_in=T_sec_sh_in,
                    T_sec_out=T_sec_sh_out)
                A_sh = Q_sh / (lmtd * U)

        A_calc = A_sh + A_lat + A_sc
        error = (self.A / A_calc - 1) * 100
        # Get possible dT_min:

        fs_state.set(name="Con_dh", value=-0.001 * (self.state_outlet.h - self.state_inlet.h), unit="kJ/kg",
                     description="Enthalpy difference Evaporator")
        fs_state.set(name="Con_A_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in condenser")
        fs_state.set(name="Con_A_lat", value=A_lat, unit="m2",
                     description="Area for latent heat exchange in condenser")
        fs_state.set(name="Con_A_sc", value=A_sc, unit="m2",
                     description="Area for subcool heat exchange in condenser")
        fs_state.set(name="Con_A_sh_rel", value=A_sh / self.A, unit="",
                     description="relative Area for superheat heat exchange in condenser")
        fs_state.set(name="Con_A_lat_rel", value=A_lat / self.A, unit="",
                     description="relative Area for latent heat exchange in condenser")
        fs_state.set(name="Con_A_sc_rel", value=A_sc / self.A, unit="",
                     description="relative Area for subcool heat exchange in condenser")
        fs_state.set(name="Con_Q_sh", value=Q_sh, unit="",
                     description="superheat heat exchange in condenser")
        fs_state.set(name="Con_Q_lat", value=Q_lat, unit="",
                     description="latent heat exchange in condenser")
        fs_state.set(name="Con_Q_sc", value=Q_sc, unit="",
                     description="subcooled heat exchange in condenser")
        fs_state.set(name="Con_Q_sh_rel", value=Q_sh / Q, unit="",
                     description="superheat heat exchange in condenser")
        fs_state.set(name="Con_Q_lat_rel", value=Q_lat / Q, unit="",
                     description="latent heat exchange in condenser")
        fs_state.set(name="Con_Q_sc_rel", value=Q_sc / Q, unit="",
                     description="subcooled heat exchange in condenser")
        fs_state.set(name="Con_Pinch", value=pinch, unit="K")
        return error, pinch

class MVB_Evaporator(BasicHX, abc.ABC):

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        self.m_flow_secondary = inputs.m_flow_eva
        # First we separate the flow:
        Q_sc, Q_lat, Q_sh, state_q0, state_q1 = self.separate_phases(
            self.state_outlet,
            self.state_inlet,
            self.state_inlet.p
        )
        if Q_sc >0:
            return -100, -10

        Q = Q_sc + Q_lat + Q_sh


        T_mean = 0.5 * (inputs.T_eva_in +inputs.T_eva_out)
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)


        T_in_sec = inputs.T_eva_in
        T_out_sec = inputs.T_eva_out
        dT_sec = T_in_sec - T_out_sec
        dT_sec_lat = dT_sec * (Q_lat/Q)
        dT_sec_sh = dT_sec *(Q_sh/Q)

        T_sec_sh_in = T_in_sec
        T_sec_sh_out = T_sec_sh_in - dT_sec_sh
        T_sec_lat_in = T_sec_sh_out
        T_sec_lat_out = T_sec_lat_in - dT_sec_lat

        T_ref_lat_in = self.state_inlet.T
        T_ref_lat_out = state_q1.T
        T_ref_sh_in = T_ref_lat_out
        T_ref_sh_out = self.state_outlet.T

        # Get possible dT_min:
        dT_1 = T_sec_lat_out - T_ref_lat_in
        dT_2 = T_sec_lat_in - T_ref_lat_out
        dT_3 = T_sec_sh_in - T_ref_sh_out

        pinch = (
            min(dT_1,
                dT_2,
                dT_3)
        )
        if pinch < 0:
            return -100, -10

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
            fs_state.set(name="Eva_alpha_lat", value=alpha_ref_wall)
            fs_state.set(name="Eva_U_lat", value=U)
            if self.model_approach.lower() == "ntu":
                W_sec = Q_lat / dT_sec_lat
                A_lat = self.detailed_epsNTU(
                    dh=self.state_inlet.h - state_q1.h,
                    Qdot=Q_lat,
                    dT_sec=-dT_sec_lat,
                    state_in=self.state_inlet,
                    T_sec_out=T_sec_lat_out,
                    W_sec=W_sec,
                    U=U
                )
            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_lat_in,
                    T_prim_out=T_ref_lat_out,
                    T_sec_in=T_sec_lat_in,
                    T_sec_out=T_sec_lat_out)
                A_lat = Q_lat/(lmtd*U)
        A_sh = 0
        if Q_sh > 0:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)
            U = self.calc_U(alpha_pri=alpha_ref_wall,
                            alpha_sec=alpha_med_wall)
            fs_state.set(name="Eva_alpha_gas", value=alpha_ref_wall)
            fs_state.set(name="Eva_U_gas", value=U)
            if self.model_approach.lower() == "ntu":
                W_sec = Q_sh / dT_sec_sh
                A_sh = self.detailed_epsNTU(
                    dh=state_q1.h - self.state_outlet.h,
                    Qdot=Q_sh,
                    dT_sec=-dT_sec_sh,
                    state_in=state_q1,
                    T_sec_out=T_sec_sh_out,
                    W_sec=W_sec,
                    U=U)

            if self.model_approach.lower() == "lmtd":
                lmtd=self.calc_lmtd(
                    T_prim_in=T_ref_sh_in,
                    T_prim_out=T_ref_sh_out,
                    T_sec_in=T_sec_sh_in,
                    T_sec_out=T_sec_sh_out)
                A_sh = Q_sh / (lmtd * U)

        A_calc = A_sh + A_lat
        error = (self.A / A_calc - 1) * 100


        fs_state.set(name="Eva_dh", value=0.001 * (self.state_outlet.h - self.state_inlet.h), unit="kJ/kg",
                     description="Enthalpy difference Evaporator")
        fs_state.set(name="Eva_A_sh", value=A_sh, unit="m2",
                     description="Area for superheat heat exchange in evaporator")
        fs_state.set(name="Eva_A_lat", value=A_lat, unit="m2",
                     description="Area for latent heat exchange in evaporator")
        fs_state.set(name="Eva_A_sh_rel", value=A_sh / self.A, unit="",
                     description="relative Area for superheat heat exchange in evaporator")
        fs_state.set(name="Eva_A_lat_rel", value=A_lat / self.A, unit="",
                     description="relative Area for latent heat exchange in evaporator")
        fs_state.set(name="Eva_Q_sh", value=Q_sh, unit="",
                     description="superheat heat exchange in evaporator")
        fs_state.set(name="Eva_Q_lat", value=Q_lat, unit="",
                     description="latent heat exchange in evaporator")
        fs_state.set(name="Eva_Q_sh_rel", value=Q_sh / Q, unit="",
                     description="superheat heat exchange in evaporator")
        fs_state.set(name="Eva_Q_lat_rel", value=Q_lat / Q, unit="",
                     description="latent heat exchange in evaporator")
        fs_state.set(name="Eva_Pinch",
                     value=pinch,
                     unit="")
        return error, pinch











