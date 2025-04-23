import abc
import logging
import math
import numpy as np

from vclibpy.components.component import BaseComponent
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.heat_exchanger import HeatExchanger
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer
from vclibpy.media import ThermodynamicState, MedProp

logger = logging.getLogger(__name__)


class BasicLMTD(HeatExchanger, abc.ABC):
    """
    Moving boundary LMTD based heat exchanger.

    See parent class for more arguments.

    Args:
        flow_type (str):
            Counter, Cross or parallel flow
        ratio_outer_to_inner_area (float):
            The NTU method uses the overall heat transfer coefficient `k`
            and multiplies it with the overall area `A`.
            However, depending on the heat exchanger type, the areas may
            differ drastically. For instance in an air-to-water heat exchanger.
            The VDI-Atlas proposes the ratio of outer area to inner pipe area
            to account for this mismatch in sizes.
            The calculation follows the code in the function `calc_k`.
    """

    def __init__(self, flow_type: str, ratio_outer_to_inner_area: float, **kwargs):
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
        super(BasicLMTD, self).__init__(**kwargs)
        self.ratio_outer_to_inner_area = ratio_outer_to_inner_area

        # Type of HE:
        self.flow_type = flow_type.lower()
        if self.flow_type not in ["counter", "cross", "parallel"]:
            raise TypeError("Given flow_type is not supported")

    def calc_A(self, lmtd, alpha_pri, alpha_sec, Q):

        k = self.calc_k(alpha_pri=alpha_pri,
                        alpha_sec=alpha_sec)

        A = Q / (k * lmtd)

        return max(A, 0)

    def calc_UA(self, lmtd, Q):

        if lmtd < 0.001:
            return 0

        return Q / lmtd

    def calc_Q_lmtd(self, lmtd, alpha_pri, alpha_sec, A):
        k = self.calc_k(alpha_pri=alpha_pri,
                        alpha_sec=alpha_sec)

        Q_lmtd = A * k * lmtd

        return Q_lmtd

    def calc_lmtd(self, Tprim_in, Tprim_out, Tsec_in, Tsec_out):

        dT_in = Tsec_in - Tprim_out
        dT_out = Tsec_out - Tprim_in

        if dT_in * dT_out <= 0:
            return 0.0000001
        if dT_out == dT_in:
            return dT_out
        lmtd = (dT_in - dT_out) / math.log((dT_in / dT_out))

        return abs(lmtd)

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


class MVBLMTDSensibleSec(BasicLMTD, abc.ABC):
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
        super(MVBLMTDSensibleSec, self).__init__(flow_type,
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


class MVBLMTDSensibleSecCon(MVBLMTDSensibleSec):
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

        lmtd_sh, lmtd_lat, lmtd_sc = 0, 0, 0

        # 1. Regime: Subcooling
        A_sc = 0
        if Q_sc > 0 and (state_q0.T != self.state_outlet.T):
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(state_q0, self.state_outlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_con)

            # Only use still available area:

            lmtd_sc = self.calc_lmtd(Tprim_in=state_q0.T,
                                     Tprim_out=self.state_outlet.T,
                                     Tsec_in=inputs.T_con_in,
                                     Tsec_out=T_sc)

            A_sc = self.calc_A(lmtd=lmtd_sc,
                               alpha_pri=alpha_ref_wall,
                               alpha_sec=alpha_med_wall,
                               Q=Q_sc)

            A_sc = min(self.A, A_sc)
            fs_state.set(name="Con_alpha_liquid", value=alpha_ref_wall)

        # 2. Regime: Latent heat exchange
        A_lat = 0
        if Q_lat > 0:
            # Get transport properties:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )
            lmtd_lat = self.calc_lmtd(Tprim_in=state_q1.T,
                                      Tprim_out=state_q0.T,
                                      Tsec_in=T_sc,
                                      Tsec_out=T_sh)

            A_lat = self.calc_A(lmtd=lmtd_lat,
                                alpha_pri=alpha_ref_wall,
                                alpha_sec=alpha_med_wall,
                                Q=Q_lat)

            # Only use still available area:
            A_lat = min(self.A - A_sc, A_lat)
            fs_state.set(name="Con_alpha_lat", value=alpha_ref_wall)

            logger.debug(f"con_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Superheat heat exchange
        A_sh = 0
        if Q_sh and (self.state_inlet.T != state_q1.T):
            # Get transport properties:
            tra_prop_ref_con = self.med_prop.calc_mean_transport_properties(self.state_inlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_con)

            # Only use still available area:

            lmtd_sh = self.calc_lmtd(Tprim_in=self.state_inlet.T,
                                     Tprim_out=state_q1.T,
                                     Tsec_in=T_sh,
                                     Tsec_out=T_out)

            A_sh = self.calc_A(lmtd=lmtd_sh,
                               alpha_pri=alpha_ref_wall,
                               alpha_sec=alpha_med_wall,
                               Q=Q_sh)
            fs_state.set(name="Con_alpha_gas", value=alpha_ref_wall)
            logger.debug(f"con_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        A_lmtd = A_sh + A_lat + A_sc
        error = (self.A / A_lmtd - 1) * 100
        # Get possible dT_min:
        dT_min_in = self.state_outlet.T - inputs.T_con_in
        dT_min_out = self.state_inlet.T - T_out
        dT_min_LatSH = state_q1.T - T_sh

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
        fs_state.set(name="Con_lmtd_sh", value=lmtd_sh, unit="K",
                     description="logartihmic temperature difference sh in condenser")
        fs_state.set(name="Con_lmtd_lat", value=lmtd_lat, unit="K",
                     description="logartihmic temperature difference lat in condenser")
        fs_state.set(name="Con_lmtd_sc", value=lmtd_sc, unit="K",
                     description="logartihmic temperature difference sc in condenser")

        fs_state.set(name="dT_pinch_con",
                     value=min(dT_min_in,
                               dT_min_LatSH,
                               dT_min_out),
                     description="Pinch Condenser")

        fs_state.set(name="Con_UA_sh", value=self.calc_UA(lmtd_sh, Q_sh))
        fs_state.set(name="Con_UA_lat", value=self.calc_UA(lmtd_lat, Q_lat))
        fs_state.set(name="Con_UA_sc", value=self.calc_UA(lmtd_sc, Q_sc))

        return error, min(dT_min_in,
                          dT_min_LatSH,
                          dT_min_out)


class MVBLMTDSensibleSecEvap(MVBLMTDSensibleSec):
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

        lmtd_sh, lmtd_lat, lmtd_sc = 0, 0, 0

        # 1. Regime: Superheating
        A_sh = 0
        if Q_sh and (self.state_outlet.T != state_q1.T):
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(self.state_outlet, state_q1)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref_eva)

            lmtd_sh = self.calc_lmtd(Tprim_in=state_q1.T,
                                     Tprim_out=self.state_outlet.T,
                                     Tsec_in=inputs.T_eva_in,
                                     Tsec_out=T_sh)

            A_sh = self.calc_A(lmtd=lmtd_sh,
                               alpha_pri=alpha_ref_wall,
                               alpha_sec=alpha_med_wall,
                               Q=Q_sh)
            fs_state.set(name="Eva_alpha_gas", value=alpha_ref_wall)
            logger.debug(f"eva_sh: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 2. Regime: Latent heat exchange
        A_lat = 0
        if Q_lat > 0:
            alpha_ref_wall = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                fs_state=fs_state,
                inputs=inputs
            )

            lmtd_lat = self.calc_lmtd(Tprim_in=state_q0.T,
                                      Tprim_out=state_q1.T,
                                      Tsec_in=T_sh,
                                      Tsec_out=T_sc)

            A_lat = self.calc_A(lmtd=lmtd_lat,
                                alpha_pri=alpha_ref_wall,
                                alpha_sec=alpha_med_wall,
                                Q=Q_lat)

            fs_state.set(name="Eva_alpha_lat", value=alpha_ref_wall)

            logger.debug(f"eva_lat: pri: {round(alpha_ref_wall, 2)} sec: {round(alpha_med_wall, 2)}")

        # 3. Regime: Subcooling
        A_sc = 0
        if Q_sc > 0 and (state_q0.T != self.state_inlet.T):
            # Get transport properties:
            tra_prop_ref_eva = self.med_prop.calc_mean_transport_properties(state_q0, self.state_inlet)
            alpha_ref_wall = self.calc_alpha_liquid(tra_prop_ref_eva)

            # Only use still available area:
            lmtd_sc = self.calc_lmtd(Tprim_in=self.state_inlet.T,
                                     Tprim_out=state_q0.T,
                                     Tsec_in=T_sc,
                                     Tsec_out=T_out)

            A_sc = self.calc_A(lmtd=lmtd_sc,
                               alpha_pri=alpha_ref_wall,
                               alpha_sec=alpha_med_wall,
                               Q=Q_sc)
            fs_state.set(name="Eva_alpha_liquid", value=alpha_ref_wall)

        A_lmtd = A_sh + A_lat + A_sc
        error = (self.A / A_lmtd - 1) * 100
        # Get dT_min
        dT_min_in = inputs.T_eva_in - self.state_outlet.T
        dT_min_out = T_out - self.state_inlet.T

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
        fs_state.set(name="Eva_lmtd_sh", value=lmtd_sh, unit="K",
                     description="logartihmic temperature difference sh in evaporator")
        fs_state.set(name="Eva_lmtd_lat", value=lmtd_lat, unit="K",
                     description="logartihmic temperature difference lat in evaporator")

        fs_state.set(name="dT_pinch_eva",
                     value=min(dT_min_out,
                               dT_min_in),
                     description="Pinch Evaporator")

        fs_state.set(name="Eva_UA_sh", value=self.calc_UA(lmtd_sh, Q_sh))
        fs_state.set(name="Eva_UA_lat", value=self.calc_UA(lmtd_lat, Q_lat))

        return error, min(dT_min_out, dT_min_in)


class MVB_LMTD_IHX(BasicLMTD):

    def __init__(self,
                 flow_type: str,
                 gas_heat_transfer: HeatTransfer,
                 liquid_heat_transfer: HeatTransfer,
                 two_phase_heat_transfer_high: TwoPhaseHeatTransfer,
                 two_phase_heat_transfer_low: TwoPhaseHeatTransfer,
                 **kwargs):
        super(MVB_LMTD_IHX, self).__init__(flow_type,
                                           secondary_medium="None",
                                           secondary_heat_transfer=None,
                                           ratio_outer_to_inner_area=1,
                                           **kwargs)

        self._state_inlet_low: ThermodynamicState = None
        self._state_outlet_low: ThermodynamicState = None
        self._state_inlet_high: ThermodynamicState = None
        self._state_outlet_high: ThermodynamicState = None
        self._m_flow_high = None
        self._m_flow_low = None

        self._gas_heat_transfer = gas_heat_transfer
        self._liquid_heat_transfer = liquid_heat_transfer
        self._two_phase_heat_transfer_high = two_phase_heat_transfer_high
        self._two_phase_heat_transfer_low = two_phase_heat_transfer_low

    def start_secondary_med_prop(self):
        return

    def terminate_secondary_med_prop(self):
        return

    def calc_alpha_secondary(self, transport_properties):
        return

    def separte_phases(self,
                       p_low,
                       p_high,
                       state_high_min,
                       state_high_max,
                       state_low_min,
                       state_low_max):
        state_high_q0 = self.med_prop.calc_state("PQ", p_high, 0)
        state_high_q1 = self.med_prop.calc_state("PQ", p_high, 1)

        Q_high_sc = max(0.0,
                        min((state_high_q0.h - state_high_min.h),
                            (state_high_max.h - state_high_min.h))) * self.m_flow_high

        Q_high_lat = max(0.0,
                         (min(state_high_max.h, state_high_q1.h) -
                          max(state_high_min.h, state_high_q0.h))) * self.m_flow_high

        Q_high_sh = max(0.0,
                        min((state_high_max.h - state_high_q1.h),
                            (state_high_max.h - state_high_min.h))) * self.m_flow_high

        state_low_q0 = self.med_prop.calc_state("PQ", p_low, 0)
        state_low_q1 = self.med_prop.calc_state("PQ", p_low, 1)

        Q_low_sc = max(0.0,
                       min((state_low_q0.h - state_low_min.h),
                           (state_low_max.h - state_low_min.h))) * self.m_flow_low

        Q_low_lat = max(0.0,
                        (min(state_low_max.h, state_low_q1.h) -
                         max(state_low_min.h, state_low_q0.h))) * self.m_flow_low

        Q_low_sh = max(0.0,
                       min((state_low_max.h - state_low_q1.h),
                           (state_low_max.h - state_low_min.h))) * self.m_flow_low

        def _seprate_singel(h_sc,
                            h_lat,
                            h_sh,
                            low):
            Q_hsc_l = max(0.0, min(low, h_sc))
            low -= Q_hsc_l
            h_sc -= Q_hsc_l
            Q_hlat_l = max(0.0, min(low, h_lat))
            low -= Q_hlat_l
            h_lat -= Q_hlat_l
            Q_hsh_l = max(0.0, min(low, h_sh))
            low -= Q_hsh_l
            h_sh -= Q_hsh_l

            return [[Q_hsc_l, Q_hlat_l, Q_hsh_l], [h_sc, h_lat, h_sc]]

        low_sc = _seprate_singel(h_sc=Q_high_sc, h_lat=Q_high_lat, h_sh=Q_high_sh, low=Q_low_sc)
        low_lat = _seprate_singel(h_sc=low_sc[1][0], h_lat=low_sc[1][1], h_sh=low_sc[1][2], low=Q_low_lat)
        low_sh = _seprate_singel(h_sc=low_lat[1][0], h_lat=low_lat[1][1], h_sh=low_lat[1][2], low=Q_low_sh)

        regime = []
        for low in [low_sc[0], low_lat[0], low_sh[0]]:
            for _Q in low:
                regime.append(_Q)

        h_low = [state_low_min.h]
        h_high = [state_high_min.h]
        for i, _q in enumerate(regime):
            h_low.append(h_low[i] + _q / self.m_flow_low)
            h_high.append(h_high[i] + _q / self.m_flow_high)

        t_low, t_high = [], []
        for _h_low in h_low:
            t_low.append(self.med_prop.calc_state("PH", p_low, _h_low).T)
        for _h_high in h_high:
            t_high.append(self.med_prop.calc_state("PH", p_high, _h_high).T)
        return regime, t_low, t_high, state_low_q0, state_low_q1, state_high_q0, state_high_q1

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):

        Q_regime, T_low, T_high, state_low_q0, state_low_q1, state_high_q0, state_high_q1 = self.separte_phases(
            p_low=self.state_inlet_low.p,
            p_high=self.state_inlet_high.p,
            state_high_min=self.state_outlet_high,
            state_high_max=self.state_inlet_high,
            state_low_min=self.state_inlet_low,
            state_low_max=self.state_outlet_low)

        transport_gas_high = self.med_prop.calc_mean_transport_properties(state_high_q1, self.state_outlet_high)
        transport_gas_low = self.med_prop.calc_mean_transport_properties(state_low_q1, self.state_outlet_low)
        transport_liq_high = self.med_prop.calc_mean_transport_properties(state_high_q0, self.state_inlet_high)
        transport_liq_low = self.med_prop.calc_mean_transport_properties(state_low_q0, self.state_inlet_low)

        alpha_gas_high = self._gas_heat_transfer.calc(transport_gas_high, self.m_flow_high)
        alpha_liq_high = self._liquid_heat_transfer.calc(transport_liq_high, self.m_flow_high)
        alpha_gas_low = self._gas_heat_transfer.calc(transport_gas_low, self.m_flow_low)
        alpha_liq_low = self._liquid_heat_transfer.calc(transport_liq_low, self.m_flow_low)

        alpha_con = self._two_phase_heat_transfer_high.calc(
            state_q0=state_high_q0,
            state_q1=state_high_q1,
            inputs=inputs,
            fs_state=fs_state,
            m_flow=self.m_flow_high,
            med_prop=self.med_prop,
            state_inlet=self.state_inlet_high,
            state_outlet=self.state_outlet_high)

        alpha_eva = self._two_phase_heat_transfer_low.calc(
            state_q0=state_low_q0,
            state_q1=state_low_q1,
            inputs=inputs,
            fs_state=fs_state,
            m_flow=self.m_flow_low,
            med_prop=self.med_prop,
            state_inlet=self.state_inlet_low,
            state_outlet=self.state_outlet_low)

        alpha_low = [alpha_liq_low, alpha_eva, alpha_gas_low, alpha_liq_low, alpha_eva, alpha_gas_low, alpha_liq_low,
                     alpha_eva, alpha_gas_low]
        alpha_high = [alpha_liq_high, alpha_con, alpha_gas_high, alpha_liq_high, alpha_con, alpha_gas_high,
                      alpha_liq_high, alpha_con, alpha_gas_high]

        A_lmtd = 0
        dT = []
        for i, Q in enumerate(Q_regime):
            lmtd = self.calc_lmtd(
                Tprim_in=T_low[i],
                Tprim_out=T_low[i + 1],
                Tsec_in=T_high[i + 1],
                Tsec_out=T_high[i]
            )
            A_lmtd += self.calc_A(
                alpha_sec=alpha_high[i],
                alpha_pri=alpha_low[i],
                lmtd=lmtd,
                Q=Q
            )
            dT.append(T_high[i + 1] - T_low[i])
            dT.append(T_high[i] - T_low[i + 1])

        error = (self.A / A_lmtd - 1) * 100

        return error, min(dT)

    @property
    def state_inlet_high(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet_high

    @state_inlet_high.setter
    def state_inlet_high(self, state_inlet_high: ThermodynamicState):
        """
        Set the inlet state of the component.

        Args:
            state_inlet (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet_high = state_inlet_high

    @property
    def state_outlet_high(self) -> ThermodynamicState:
        """
        Get or set the outlet state of the component.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet_high

    @state_outlet_high.setter
    def state_outlet_high(self, state_outlet_high: ThermodynamicState):
        """
        Set the outlet state of the component.

        Args:
            state_outlet (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet_high = state_outlet_high

    @property
    def m_flow_high(self) -> float:
        """
        Get or set the mass flow rate through the component.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow_high

    @m_flow_high.setter
    def m_flow_high(self, m_flow_high: float):
        """
        Set the mass flow rate through the component.

        Args:
            m_flow (float): Mass flow rate to set.
        """
        self._m_flow_high = m_flow_high

    @property
    def state_inlet_low(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet_low

    @state_inlet_low.setter
    def state_inlet_low(self, state_inlet_low: ThermodynamicState):
        """
        Set the inlet state of the component.

        Args:
            state_inlet (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet_low = state_inlet_low

    @property
    def state_outlet_low(self) -> ThermodynamicState:
        """
        Get or set the outlet state of the component.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet_low

    @state_outlet_low.setter
    def state_outlet_low(self, state_outlet_low: ThermodynamicState):
        """
        Set the outlet state of the component.

        Args:
            state_outlet (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet_low = state_outlet_low

    @property
    def m_flow_low(self) -> float:
        """
        Get or set the mass flow rate through the component.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow_low

    @m_flow_low.setter
    def m_flow_low(self, m_flow_low: float):
        """
        Set the mass flow rate through the component.

        Args:
            m_flow (float): Mass flow rate to set.
        """
        self._m_flow_low = m_flow_low

    @BaseComponent.state_outlet.setter
    def state_outlet(self, state: ThermodynamicState):
        """
        This outlet is disabled for this component.

        Args:
            state (ThermodynamicState): Outlet state.
        """
        raise NotImplementedError("This outlet is disabled for this component")

    @BaseComponent.state_inlet.setter
    def state_outlet(self, state: ThermodynamicState):
        """
        This outlet is disabled for this component.

        Args:
            state (ThermodynamicState): Outlet state.
        """
        raise NotImplementedError("This outlet is disabled for this component")


class FV_LMTD_GC(BasicLMTD):

    def __init__(self,
                 flow_type: str,
                 ratio_outer_to_inner_area: float,
                 gas_heat_transfer: HeatTransfer,
                 n_regimes=50,
                 **kwargs):
        super(FV_LMTD_GC, self).__init__(flow_type,
                                         ratio_outer_to_inner_area,
                                         **kwargs)

        self._gas_heat_transfer = gas_heat_transfer
        self.n_regimes = n_regimes

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

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        dT_min_regimes = []
        A_lmtd = 0

        dh = self.state_inlet.h - self.state_outlet.h
        Q = self.m_flow * dh

        dh_step = dh / self.n_regimes
        dQ_step = Q / self.n_regimes
        p_con = self.state_inlet.p

        dT_step = (inputs.T_con_out - inputs.T_con_in) / self.n_regimes

        T_sec_in = inputs.T_con_in
        state_step_out = self.state_outlet

        T_mean = (inputs.T_con_in + inputs.T_con_out) / 2
        tra_prop_med = self.calc_transport_properties_secondary_medium(T_mean)
        alpha_med_wall = self.calc_alpha_secondary(tra_prop_med)

        while state_step_out.h < self.state_inlet.h:
            state_step_in = self.med_prop.calc_state("PH", p_con, state_step_out.h + dh_step)
            T_sec_out = T_sec_in + dT_step
            tra_prop_ref = self.med_prop.calc_mean_transport_properties(state_step_in, state_step_out)
            alpha_ref_wall = self.calc_alpha_gas(tra_prop_ref)

            lmtd = self.calc_lmtd(Tprim_in=state_step_in.T,
                                  Tprim_out=state_step_out.T,
                                  Tsec_in=T_sec_in,
                                  Tsec_out=T_sec_in)

            A_lmtd += self.calc_A(lmtd=lmtd,
                                  alpha_pri=alpha_ref_wall,
                                  alpha_sec=alpha_med_wall,
                                  Q=dQ_step)

            dT_min_regimes.append(state_step_in.T - T_sec_out)
            dT_min_regimes.append(state_step_out.T - T_sec_in)
            state_step_out = state_step_in
            T_sec_in = T_sec_out

        error = (self.A / A_lmtd - 1) * 100

        dT_min = min(dT_min_regimes)
        fs_state.set(name="dT_pinch_con",
                     value=dT_min,
                     description="Pinch Condenser")
        return error, dT_min
