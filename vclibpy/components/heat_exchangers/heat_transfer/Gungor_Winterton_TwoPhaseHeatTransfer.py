import math
import numpy as np
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import TwoPhaseHeatTransfer

class GungorWintertonTwoPhaseHeatTransferForTubesAndAnnuli(TwoPhaseHeatTransfer):
    """
    Two-phase boiling heat transfer correlation following
    Gungor & Winterton (1986) for tubes and annuli.

    References
      - Gungor, K.E., & Winterton, R.H.S. (1986). A general correlation for
        flow boiling in tubes and annuli. Int. J. Heat Mass Transfer 29(3):351–358.
      - Cooper pool-boiling correlation as used within G&W superposition.
    """

    def __init__(
        self,
        d_h: float = None,
        *,
        is_annulus: bool = False,
        d_i: float = None,
        d_o: float = None,
        orientation: str = "horizontal",
        g: float = 9.80665,
        A_heat: float = None,
    ):
        """
        Args:
            d_h: Hydraulic diameter [m] when using a regular circular tube.
            is_annulus: Set True for an annular gap and supply d_i, d_o.
            d_i, d_o: Inner/outer diameters for annulus geometry [m].
            orientation: "horizontal" or "vertical" (affects low-Froude correction).
            g: Gravitational acceleration [m/s²] used in Fr_f.
            A_heat: Optional heated area [m²] to compute q'' for Cooper.
                    If not provided, will fall back to internal flow area (approximation).

        Notes on annuli:
          - The hydraulic diameter selection follows G&W Eq. (13):
            if gap > 4 mm, use 4*A_flow/(P_o + P_i); else use one-sided heated perimeter.
          - We also preserve the TRUE flow area to compute G = m_dot / A_flow
            (important: mass flux must use real cross-sectional area, not π(d_h)²/4).
        """
        self.orientation = orientation.lower()
        self.g = g
        self.A_heat = A_heat

        if is_annulus and (d_i is not None) and (d_o is not None):
            # Geometric primitives
            self.d_i = d_i
            self.d_o = d_o
            gap = d_o - d_i

            # True cross-sectional flow area (used for G)
            self.A_flow = math.pi * (d_o**2 - d_i**2) / 4.0

            # Perimeters
            P_i = math.pi * d_i
            P_o = math.pi * d_o

            # G&W Eq. (13): annulus equivalent hydraulic diameter selection
            if gap > 0.004:
                # Gap > 4 mm: use both perimeters (two-sided heating)
                self.d_h = 4.0 * self.A_flow / (P_o + P_i)
            else:
                # Gap <= 4 mm: one-sided heating → use heated perimeter (inner side)
                heated_perimeter = P_i
                self.d_h = 4.0 * self.A_flow / heated_perimeter
        else:
            # Circular tube: store d_h and its true flow area
            if d_h is None:
                raise ValueError("d_h must be provided for circular tube configuration.")
            self.d_h = float(d_h)
            self.A_flow = math.pi * (self.d_h**2) / 4.0

    def calc(self, state_q0, state_q1, inputs, fs_state, m_flow, med_prop, state_inlet, state_outlet):
        """
        Compute two-phase HTC h_tp [W/m²·K] using Gungor & Winterton (1986).

        Model structure (per G&W):
          1) Compute liquid-only convection HTC (Dittus–Boelter).
          2) Compute enhancement factor E(Bo, X_tt) and suppression factor S(E, Re_l).
          3) Compute pool-boiling term via Cooper correlation (with reduced pressure p_r).
          4) Superpose: h_tp = E*h_lo + S*h_pool.
          5) For horizontal tubes at very low liquid Froude number (Fr_f ≤ 0.05),
             apply G&W’s low-Froude corrections to E and S.

        Arguments follow VcLibPy conventions.
        """
        # Local saturation end states (quality 0 and 1) at the same p
        p = state_q0.p  # [Pa] — ensure consistent units with med_prop.p_crit
        h_lv = state_q1.h - state_q0.h  # latent heat [J/kg]

        # --- Transport properties (evaluated at saturated liquid/vapor states) ---
        # These property calls should return SI units:
        #   dyn_vis: Pa·s, lam: W/m·K, Pr: -, density d: kg/m³
        tp_l = med_prop.calc_transport_properties(state_q0)
        tp_v = med_prop.calc_transport_properties(state_q1)
        mu_l = tp_l.dyn_vis
        mu_v = tp_v.dyn_vis
        k_l = tp_l.lam
        Pr_l = tp_l.Pr
        rho_l = state_q0.d
        rho_v = state_q1.d

        # --- Mass flux based on TRUE flow area (critical for annuli) ---
        G = m_flow / max(self.A_flow, 1e-12)  # [kg/m²·s]

        # --- Reynolds number using liquid-only flow (standard in G&W) ---
        Re_l = G * self.d_h / max(mu_l, 1e-12)

        # --- Liquid-only convective HTC: Dittus–Boelter (as in G&W base) ---
        h_lo = 0.023 * (Re_l**0.8) * (Pr_l**0.4) * (k_l / max(self.d_h, 1e-12))

        # --- Heat flux q'' and Boiling number Bo ---
        # Properly, q'' should be based on HEATED area, not flow area.
        # If you can supply a local heated area (segment area) via A_heat or inputs.A, use it.
        # Otherwise we fall back to A_flow as an approximation.
        A_heat = self.A_heat
        if A_heat is None and hasattr(inputs, "A") and inputs.A:
            A_heat = inputs.A  # VcLibPy components often carry a total/segment area field
        if A_heat is None:
            A_heat = self.A_flow  # approximation if nothing better is available

        # Segment heat rate estimate from inlet/outlet enthalpy change:
        # Q = m_dot * |Δh|  (consistent with segment-averaged modeling)
        Q = m_flow * abs(state_inlet.h - state_outlet.h)  # [W]
        q_dot = Q / max(A_heat, 1e-12)                    # [W/m²]

        # Boiling number Bo = q'' / (G * h_lv)
        Bo = q_dot / max(G * max(h_lv, 1e-12), 1e-12)

        # --- Mean vapor quality x for X_tt ---
        # Use average of inlet/outlet qualities when available; otherwise a safe fallback.
        if hasattr(state_inlet, "q") and hasattr(state_outlet, "q"):
            x = 0.5 * (max(0.0, min(1.0, state_inlet.q)) + max(0.0, min(1.0, state_outlet.q)))
        else:
            x = 0.5

        # --- Martinelli parameter X_tt (per G&W; DO NOT take square root) ---
        x_safe = max(x, 1e-6)
        X_tt = ((1.0 - x_safe) / x_safe)**0.9 * (max(rho_v, 1e-12) / max(rho_l, 1e-12))**0.5 * (max(mu_l, 1e-12) / max(mu_v, 1e-12))**0.1

        # --- Enhancement factor E and suppression factor S (G&W final recommended form) ---
        E = 1.0 + 24000.0 * (Bo**1.16) + 1.37 * (X_tt**-0.86)

        S = 1.0 / (1.0 + 1.15e-6 * (E**2) * (Re_l**1.17))

        # --- Cooper pool-boiling term (as used in G&W) ---
        # Use REDUCED pressure p_r = p/p_crit (not absolute p) in the Cooper formula.
        # h_pool = 55 * p_r^0.12 * [-log10(p_r)]^-0.55 * M^-0.5 * q''^0.67
        # Ensure units: p and p_crit both in Pa; M in kg/mol or g/mol consistently with the constant 55 (SI usage assumed).
        try:
            _, p_crit, _ = med_prop.get_critical_point()
            p_r = p / max(p_crit, 1e-12)
        except Exception:
            p_r = 0.3

        # Numerical safeguards for the logarithm and near-critical region
        p_r = min(max(p_r, 1e-6), 0.99)

        try:
            M = med_prop.get_molar_mass()
        except Exception:
            M = 0.018  # fallback

        h_pool = 55.0 * (p_r**0.12) * ((-np.log10(p_r))**-0.55) * (M**-0.5) * (q_dot**0.67)

        # --- Horizontal low-Froude correction (G&W recommendation) ---
        # For HORIZONTAL tubes at very low liquid Froude number (Fr_f <= 0.05), account for stratification effects:
        Fr_f = G**2 / (max(rho_l, 1e-12)**2 * self.g * max(self.d_h, 1e-12))
        if self.orientation == "horizontal" and Fr_f <= 0.05:
            # Apply gently; Fr_f^(0.1 - 2Fr_f) → approaches 1 as Fr_f→0.05, reduces E at very low Fr_f
            correction_E = Fr_f**(0.1 - 2.0 * Fr_f) if Fr_f > 0.0 else 0.0
            correction_S = Fr_f**0.5 if Fr_f > 0.0 else 0.0
            E *= correction_E
            S *= correction_S

        # --- Superposition principle (G&W): convection + suppressed pool boiling ---
        h_tp = E * h_lo + S * h_pool

        # --- Debug/diagnostics into flowsheet state for post-processing/validation ---
        # Saving intermediate nondimensional groups and components helps verify implementation
        fs_state.set(name="GW_h_lo", value=h_lo, unit="W/m2K", description="Liquid-only HTC (Dittus–Boelter)")
        fs_state.set(name="GW_Bo", value=Bo, unit="-", description="Boiling number q''/(G*h_lv)")
        fs_state.set(name="GW_X_tt", value=X_tt, unit="-", description="Martinelli parameter per G&W")
        fs_state.set(name="GW_E", value=E, unit="-", description="Enhancement factor per G&W final form")
        fs_state.set(name="GW_S", value=S, unit="-", description="Suppression factor per G&W final form")
        fs_state.set(name="GW_Fr_f", value=Fr_f, unit="-", description="Liquid Froude number (horizontal check)")
        fs_state.set(name="GW_h_pool", value=h_pool, unit="W/m2K", description="Cooper pool-boiling HTC")
        fs_state.set(name="GW_h_tp", value=h_tp, unit="W/m2K", description="Two-phase HTC by Gungor & Winterton (superposition)")

        return h_tp