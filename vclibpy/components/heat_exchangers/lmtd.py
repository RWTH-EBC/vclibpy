import logging
import abc
import math

import numpy as np
from vclibpy.components.heat_exchangers.heat_exchanger import HeatExchanger
from vclibpy.media import ThermodynamicState


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

        # Set primary cp:
        self._primary_cp = None

        # Type of HE:
        self.flow_type = flow_type.lower()
        if self.flow_type not in ["counter", "cross", "parallel"]:
            raise TypeError("Given flow_type is not supported")

    def set_primary_cp(self, cp: float):
        """
        Set the specific heat (cp) for the primary medium.

        Args:
            cp (float): Specific heat for the primary medium.
        """
        self._primary_cp = cp

    def calc_A(self, lmtd, alpha_pri, alpha_sec, Q):

        k = self.calc_k(alpha_pri=alpha_pri,
                        alpha_sec=alpha_sec)

        A = Q/(k*lmtd)

        return max(A,0)

    def calc_Q_lmtd(self, lmtd, alpha_pri, alpha_sec, A):
        k = self.calc_k(alpha_pri=alpha_pri,
                        alpha_sec=alpha_sec)

        Q_lmtd = A * k * lmtd

        return Q_lmtd


    def calc_lmtd(self, Tprim_in, Tprim_out, Tsec_in, Tsec_out):

        dT_in = Tsec_in-Tprim_out
        dT_out = Tsec_out-Tprim_in

        if dT_in*dT_out < 0:
            return 0.0000001

        lmtd = (dT_in-dT_out)/math.log((dT_in/dT_out))

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


    def calc_m_flow_cp_min(self) -> float:
        """
        Calculate the minimum value between the heat capacity rates (m_flow*cp) for the primary and secondary mediums.

        Returns:
            float: Minimum value.
        """
        return min(
            self.m_flow * self._primary_cp,
            self.m_flow_secondary_cp
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

