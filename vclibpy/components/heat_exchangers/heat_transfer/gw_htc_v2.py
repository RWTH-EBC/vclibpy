from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class GWInputs:
    """
    Immutable input bundle for the Gungor & Winterton HTC evaluation.

    Attributes
    ----------
    x : float
        Local vapor quality in [-]. Must be 0 < x < 1 for the two‑phase correlation.
    q_flux : float
        Local heat flux q'' in [W/m²], positive by convention (heat into the refrigerant).
    p : float
        Local pressure in [Pa].
    m_flow : float
        Refrigerant mass flow in [kg/s].
    d_h : float
        Hydraulic diameter in [m].
    orientation : str
        "horizontal" or "vertical". Only a small correction is applied for horizontal low‑Fr.
    """
    x: float
    q_flux: float
    p: float
    m_flow: float
    d_h: float
    orientation: str = "horizontal"


class GungorWintertonHTC:
    """
    Pure Gungor & Winterton (1986) flow‑boiling correlation.

    Usage
    -----
    h = model.htc(GWInputs(...), med_prop)

    This class does **not** cache state and does **not** know the segment geometry
    beyond the given hydraulic diameter. All fluid properties are pulled via the
    provided `med_prop` interface (your wrapper with calc_state / calc_transport_properties).
    """

    g = 9.80665  # [m/s²]

    def htc(self, inp: GWInputs, med_prop) -> float:
        """
        Compute two‑phase HTC in [W/m²K] using the final GW superposition.

        The method requires the local heat flux q'', which your segmented model
        must provide (e.g., by a fixed‑point iteration q'' = h(x, q'') * ΔT).

        Parameters
        ----------
        inp : GWInputs
            Correlation inputs (x, q'', p, m_flow, d_h, orientation).
        med_prop : MedProp
            Media property wrapper, must provide:
              - calc_state("PQ", p, 0/1)
              - calc_transport_properties(state) -> has dyn_vis [Pa·s], lam [W/mK], Pr [-]
              - get_critical_point() -> (Tc[K], pc[Pa], dc[kg/m³])
              - get_molar_mass() -> [kg/mol]

        Returns
        -------
        float
            Two‑phase heat transfer coefficient in [W/m²K].
        """
        # Validate inputs that can break the correlation
        if not (0.0 < inp.x < 1.0):
            raise ValueError(f"x={inp.x:.4f} outside two‑phase range (0<x<1).")
        if inp.q_flux <= 0.0:
            raise ValueError("q_flux must be > 0.")
        if inp.p <= 0.0 or inp.m_flow <= 0.0 or inp.d_h <= 0.0:
            raise ValueError("p, m_flow, d_h must be positive.")

        # Saturation states at local pressure
        st_l = med_prop.calc_state("PQ", inp.p, 0)  # saturated liquid
        st_v = med_prop.calc_state("PQ", inp.p, 1)  # saturated vapor
        h_lv = st_v.h - st_l.h
        if h_lv <= 0.0:
            raise ValueError(f"Latent heat invalid at p={inp.p:g} Pa (h_lv={h_lv:g}).")

        # Transport properties (liquid and vapor at saturation)
        tp_l = med_prop.calc_transport_properties(st_l)
        tp_v = med_prop.calc_transport_properties(st_v)
        mu_l = tp_l.dyn_vis
        mu_v = tp_v.dyn_vis
        k_l = tp_l.lam
        Pr_l = tp_l.Pr
        rho_l = st_l.d
        rho_v = st_v.d
        if any(val <= 0.0 for val in (mu_l, mu_v, k_l, Pr_l, rho_l, rho_v)):
            raise ValueError("Invalid transport properties from med_prop.")

        # Mass flux G based on circular tube
        A_flow = math.pi * (inp.d_h ** 2) / 4.0
        G = inp.m_flow / A_flow  # [kg/m²s]
        if G <= 0.0:
            raise ValueError("Computed mass flux G <= 0.")

        # Liquid‑only film coefficient by Dittus‑Boelter (heating of liquid)
        Re_l = G * inp.d_h / mu_l
        if Re_l <= 2300:
            warnings.warn(f"Re_l={Re_l:.0f} < 2300. G&W correlation designed for turbulent flow.")

        h_lo = 0.023 * (Re_l ** 0.8) * (Pr_l ** 0.4) * (k_l / inp.d_h)

        # Boiling number and Martinelli parameter X_tt
        Bo = inp.q_flux / (G * h_lv)
        x = max(1e-3, min(1.0 - 1e-3, inp.x))  # numerical safety
        X_tt = ((1.0 - x) / x) ** 0.9 * (rho_v / rho_l) ** 0.5 * (mu_l / mu_v) ** 0.1

        # Enhancement (E) and suppression (S) factors (GW final recommended form)
        E = 1.0 + 24000.0 * (Bo ** 1.16) + 1.37 * (X_tt ** -0.86)
        S = 1.0 / (1.0 + 1.15e-6 * (E ** 2) * (Re_l ** 1.17))

        # Pool‑boiling (Cooper‑type) contribution with reduced pressure & molar mass
        Tc, pc, _ = med_prop.get_critical_point()
        p_r = inp.p / pc
        if not (1e-3 < p_r < 0.99):
            raise ValueError(f"Reduced pressure p_r={p_r:.3f} outside [1e-3,0.99] for GW.")
        M = med_prop.get_molar_mass()  # [kg/mol]

        # Cooper correlation term (units as in GW paper adaptation)
        # h_pool ∝ p_r^0.12 * (-log10 p_r)^-0.55 * M^-0.5 * q''^0.67
        h_pool = 55.0 * (p_r ** 0.12) * ((-math.log10(p_r)) ** -0.55) * (M ** -0.5) * (inp.q_flux ** 0.67)

        # Horizontal low‑Froude correction (optional, mild)
        Fr_f = G ** 2 / (rho_l ** 2 * self.g * inp.d_h)
        if inp.orientation.lower() == "horizontal" and Fr_f <= 0.05 and Fr_f > 0.0:
            E *= Fr_f ** (0.1 - 2.0 * Fr_f)
            S *= Fr_f ** 0.5

        # Superposition
        h_tp = E * h_lo + S * h_pool
        if h_tp <= 0.0:
            raise ValueError(f"h_tp={h_tp:g} not positive.")

        return float(h_tp)