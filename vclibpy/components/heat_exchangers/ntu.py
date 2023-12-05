import logging
import abc

import numpy as np
from vclibpy.components.heat_exchangers.heat_exchanger import HeatExchanger
from vclibpy.media import ThermodynamicState


logger = logging.getLogger(__name__)


class BasicNTU(HeatExchanger, abc.ABC):
    """
    Moving boundary NTU based heat exchanger.

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
        Initializes BasicNTU.

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
        super(BasicNTU, self).__init__(**kwargs)
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

    def calc_eps(self, R: float, NTU: float) -> float:
        """
        Calculate the effectiveness (eps) of the heat exchanger based on NTU.

        Source of implementation: EBC Lecture SimModelle.

        Args:
            R (float): Ratio of heat capacity rates (m_flow*cp) of the primary to secondary medium.
            NTU (float): Number of Transfer Units.

        Returns:
            float: Effectiveness (eps) of the heat exchanger.
        """
        if R in (0, 1):
            return NTU / (NTU + 1)
        if self.flow_type == "counter":
            return (1 - np.exp(-NTU * (1 - R))) / (1 - R * np.exp(-NTU * (1 - R)))
        if self.flow_type == "cross":
            eta = NTU ** -0.22
            return 1 - np.exp((np.exp(- NTU * R * eta) - 1) / (R * eta))
        if self.flow_type == "parallel":
            return (1 - np.exp(-NTU * (1 + R))) / (1 + R)
        raise TypeError(f"Flow type {self.flow_type} not supported")

    def calc_R(self) -> float:
        """
        Calculate the R value, which is the ratio of heat capacity rates (m_flow*cp) of the primary to secondary medium.

        Returns:
            float: R value.
        """
        m_flow_pri_cp = self.m_flow * self._primary_cp
        if m_flow_pri_cp > self.m_flow_secondary_cp:
            return self.m_flow_secondary_cp / m_flow_pri_cp
        return m_flow_pri_cp / self.m_flow_secondary_cp

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

    @staticmethod
    def calc_NTU(A: float, k: float, m_flow_cp: float) -> float:
        """
        Calculate the Number of Transfer Units (NTU) for the heat exchanger.

        Args:
            A (float): Area of the heat exchanger.
            k (float): Overall heat transfer coefficient.
            m_flow_cp (float): Minimal heat capacity rates (m_flow*cp) between primary and secondary side.

        Returns:
            float: Number of Transfer Units (NTU).
        """
        return k * A / m_flow_cp

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

    def calc_Q_ntu(self, dT_max: float, alpha_pri: float, alpha_sec: float, A: float) -> (float, float):
        """
        Calculate the heat transfer and overall heat transfer coefficient for the heat exchanger based on NTU.

        Args:
            dT_max (float): Maximum temperature differential.
            alpha_pri (float): Heat transfer coefficient for the primary medium.
            alpha_sec (float): Heat transfer coefficient for the secondary medium.
            A (float): Area of the heat exchanger.

        Returns:
            Tuple[float, float]: Heat transfer and overall heat transfer coefficient.
        """
        R = self.calc_R()
        k = self.calc_k(alpha_pri, alpha_sec)
        m_flow_cp_min = self.calc_m_flow_cp_min()
        NTU = self.calc_NTU(A, k, m_flow_cp_min)
        eps = self.calc_eps(R, NTU)

        # Get the maximal allowed heat flow
        Q_max = m_flow_cp_min * dT_max
        return Q_max * eps, k

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

    def iterate_area(self, dT_max, alpha_pri, alpha_sec, Q) -> float:
        """
        Iteratively calculates the required area for the heat exchange.

        Args:
            dT_max (float): Maximum temperature differential.
            alpha_pri (float): Heat transfer coefficient for the primary medium.
            alpha_sec (float): Heat transfer coefficient for the secondary medium.
            Q (float): Heat flow rate.

        Returns:
            float: Required area for heat exchange.
        """
        _accuracy = 1e-6  # square mm
        _step = 1.0
        R = self.calc_R()
        k = self.calc_k(alpha_pri, alpha_sec)
        m_flow_cp_min = self.calc_m_flow_cp_min()
        # First check if point is feasible at all
        if dT_max <= 0:
            return self.A
        eps_necessary = Q / (m_flow_cp_min * dT_max)

        # Special cases:
        # ---------------
        # eps is equal or higher than 1, an infinite amount of area would be necessary.
        if eps_necessary >= 1:
            return self.A
        # eps is lower or equal to zero: No Area required (Q<=0)
        if eps_necessary <= 0:
            return 0

        area = 0.0
        while True:
            NTU = self.calc_NTU(area, k, m_flow_cp_min)
            eps = self.calc_eps(R, NTU)
            if eps >= eps_necessary:
                if _step <= _accuracy:
                    break
                else:
                    # Go back
                    area -= _step
                    _step /= 10
                    continue
            if _step < _accuracy and area > self.A:
                break
            area += _step

        return min(area, self.A)
