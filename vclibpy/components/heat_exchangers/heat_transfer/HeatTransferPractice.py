import math
import numpy as np
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import TwoPhaseHeatTransfer

class GungorWintertonTwoPhaseHeatTransferForTubesAndAnnuli(TwoPhaseHeatTransfer)
    def __init__(
        self,
        d_h: float = None,
        *,
        is_annulus: bool = False,
        d_i: float = None,
        d_o: float = None,
        orientation: str ="horizontal",
        g: float = 9.80665,
        A_heat: float = None,
    ):
        self.orientation = orientation.lower()
        self.g = g
        self.A_heat = A_heat

        if is_annulus and (d_i is not None) and (d_o is not None):
            self.d_i = d_i
            self.d_o = d_o
            gap = d_o - d_i

            self.A_flow = math.pi * (d_o**2 - d_i**2) / 4.0

            P_i = math.pi * d_i
            P_o = math.pi * d_o

            if gap > 0.004:
                self.d_h = 4.0 * self.A_flow / (P_o + P_i)
            else:
                heated_perimeter = P_i
                self.d_h = 4.0 * self.A_flow / heated_perimeter
        else:
            if d_h is None:
                raise ValueError("d_h must be provided for circular tube configuration.")
            self.d_h = float(d_h)
            self.A_flow = math.pi * (self.d_h**2) / 4.0

    def calc(self, state_q0, state_q1, inputs, fs_state, m_flow, med_prop, state_inlet, state_outlet):
        p = state_q0.p
        h_lv = state_q1.h - state_q0.h
        tp_l = med_prop.calc_transport_properties(state_q0)
        tp_v = med_prop.calc_transport_properties(state_q1)
        mu_l = tp_l.dyn_vis
        mu_v = tp_v.dyn_vis
        k_l = tp_l.lam
        Pr_l = tp_l.Pr
        rho_l = state_q0.d
        rho_v = state_q1.d

        G = m_flow / max(self.A_flow, 1e-12)
        Re_l = G * self.d_h / max(mu_l,1e-12)
        h_lo = 0.023 * (Re_l**0.8) * (Pr_l**0.4) * (k_l / max(self.d_h, 1e-12))

