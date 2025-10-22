"""
!!!!! Warning: ONLY USE FOR VERTICAL TUBE !!!!!
!!!!! OTHERWISE HIGH ERROR !!!!!
"""
import math

from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import (
    TwoPhaseHeatTransfer,
)


class ChenTwoPhase(TwoPhaseHeatTransfer):
    """
    Chen (1966) flow-boiling correlation integrated with VcLibPy.

    This class implements the TwoPhaseHeatTransfer interface and expects the
    *per-segment* quantities to be provided via FlowsheetState before `calc`
    is called.

    Reference:
    Chen, J.C. (1966). "Correlation for Boiling Heat Transfer to Saturated
    Fluids in Convective Flow." I&EC Process Design and Development,
    Vol. 5, No. 3, pp. 322-329.

    Required FlowsheetState variables (set by the segmented HX loop):
    - 'seg_x'       : representative vapor quality for the segment (0 < x < 1)
    - 'seg_q_flux'  : local heat flux q'' in W/m^2
    - 'seg_d_h'     : hydraulic diameter [m]

    Optional FlowsheetState variables:
    - 'seg_p'           : segment pressure [Pa]
                          (defaults to state_inlet.p)

    All fluid properties are obtained from `med_prop`.
    """

    def __init__(self):
        super().__init__()

    @staticmethod
    def _read_required(fs_state, name: str) -> float:
        """Read a required variable from fs_state or raise ValueError."""
        var = fs_state.get(name, None)
        if var is None or var.value is None:
            raise ValueError(f"fs_state missing required variable '{name}'")
        return float(var.value)

    @staticmethod
    def _read_optional(fs_state, name: str, default):
        """Read an optional variable from fs_state; return default if absent."""
        var = fs_state.get(name, None)
        return float(var.value) if (var and var.value is not None) else default

    @staticmethod
    def _reynolds_number_factor_F(x_tt_inv: float) -> float:
        """
        Calculate Reynolds number factor F from Figure 6 of Chen (1966).

        F = (Re_tp / Re_l)^0.8

        Approximated by piecewise power-law fits to the empirical curve.

        Args:
            x_tt_inv: 1/X_tt (reciprocal Martinelli parameter)

        Returns:
            F factor (dimensionless)
        """
        # Piecewise approximation based on Figure 6
        # Region 1: very low quality (1/X_tt < 0.1)
        if x_tt_inv < 0.1:
            return 1.0
        # Region 2: transition (0.1 <= 1/X_tt < 1.0)
        elif x_tt_inv < 1.0:
            return 1.0 + 2.35 * (x_tt_inv + 0.213) ** 0.736
        # Region 3: annular flow (1.0 <= 1/X_tt < 10)
        elif x_tt_inv < 10.0:
            return 2.35 * (x_tt_inv + 0.213) ** 0.736
        # Region 4: high quality (1/X_tt >= 10)
        else:
            return 4.9 * (x_tt_inv ** 0.5)

    @staticmethod
    def _suppression_factor_S(re_tp: float) -> float:
        """
        Calculate suppression factor S from Figure 7 of Chen (1966).

        S = (Î”T_effective / Î”T_wall)^0.99

        Approximated by empirical curve fit.

        Args:
            re_tp: Two-phase Reynolds number = Re_l * F^1.25

        Returns:
            S factor (dimensionless, 0 to 1)
        """
        # Empirical fit to Figure 7
        # S approaches 1 at low Re_tp and 0 at high Re_tp

        if re_tp < 32.5:
            return 1.0
        elif re_tp < 70.0:
            # Transition region
            return 1.0 - 0.1 * math.log10(re_tp / 32.5) / math.log10(70.0 / 32.5)
        else:
            # Asymptotic decay
            return 1.0 / (1.0 + 2.53e-6 * (re_tp ** 1.17))

    def calc(
            self,
            state_q0,
            state_q1,
            state_inlet,
            state_outlet,
            med_prop,
            inputs,
            fs_state,
            m_flow: float,
    ) -> float:
        """
        Compute two-phase HTC using Chen (1966) correlation with segment-local data.

        Expects the calling code to have inserted segment-local values into
        `fs_state` under the keys described in the class docstring.

        Returns
        -------
        float
            Two-phase heat transfer coefficient [W/mÂ²K]
        """
        if med_prop is None:
            raise ValueError("med_prop is required")
        if m_flow is None or m_flow <= 0.0:
            raise ValueError("m_flow must be > 0")

        # --- Segment-local inputs from FlowsheetState ---
        x = self._read_required(fs_state, "seg_x")
        q_flux = self._read_required(fs_state, "seg_q_flux")
        d_h = self._read_required(fs_state, "seg_d_h")

        if d_h <= 0.0:
            raise ValueError(f"hydraulic diameter must be > 0; got {d_h}")

        p_var = fs_state.get("seg_p", None)
        p = float(p_var.value) if (p_var and p_var.value is not None) else float(state_inlet.p)

        # --- Validation ---
        if not (0.0 <= x <= 1.0):
            raise ValueError(f"segment vapor quality must be in [0,1]; got {x}")
        if q_flux <= 0.0:
            raise ValueError(f"seg_q_flux must be > 0; got {q_flux}")
        if p <= 0.0:
            raise ValueError(f"pressure must be > 0; got {p}")

        # --- Saturation states and transport properties ---
        st_l = med_prop.calc_state("PQ", p, 0.0)  # saturated liquid
        st_v = med_prop.calc_state("PQ", p, 1.0)  # saturated vapor

        h_lv = st_v.h - st_l.h  # latent heat [J/kg]
        if h_lv <= 0.0:
            raise ValueError(f"latent heat <= 0 at p={p}")

        tp_l = med_prop.calc_transport_properties(st_l)
        tp_v = med_prop.calc_transport_properties(st_v)

        # Handle possible attribute name variants
        pr_l = getattr(tp_l, "Pr", getattr(tp_l, "pr", None))
        mu_l = tp_l.dyn_vis
        k_l = tp_l.lam
        cp_l = st_l.cp
        rho_l = st_l.d
        rho_v = st_v.d
        mu_v = tp_v.dyn_vis

        # Surface tension (approximation if not available)
        try:
            sigma = med_prop.calc_surface_tension(st_l.T)
        except:
            # Approximation for refrigerants: Ïƒ â‰ˆ 0.01 N/m
            sigma = 0.01

        for name, val in {"pr_l": pr_l, "mu_l": mu_l, "k_l": k_l,
                          "cp_l": cp_l, "rho_l": rho_l, "rho_v": rho_v,
                          "mu_v": mu_v, "sigma": sigma}.items():
            if val is None or val <= 0.0:
                raise ValueError(f"invalid property {name}={val}")

        # --- Chen correlation ---
        A = math.pi * (d_h ** 2) / 4.0
        G = m_flow / A  # mass flux [kg/mÂ²s]

        # Safe vapor quality (avoid division by zero)
        x_safe = max(1.0e-6, min(1.0 - 1.0e-6, x))

        # Martinelli parameter X_tt
        x_tt = (
                ((1.0 - x_safe) / x_safe) ** 0.9
                * (rho_v / rho_l) ** 0.5
                * (mu_l / mu_v) ** 0.1
        )

        if x_tt <= 0.0:
            raise ValueError(f"Martinelli parameter X_tt <= 0")

        x_tt_inv = 1.0 / x_tt

        # Reynolds number factor F (Figure 6)
        F = self._reynolds_number_factor_F(x_tt_inv)

        # Liquid Reynolds number
        re_l = G * d_h * (1.0 - x_safe) / mu_l

        if re_l <= 0.0:
            raise ValueError(f"Reynolds number Re_l <= 0")

        # Two-phase Reynolds number
        re_tp = re_l * (F ** 1.25)

        # Suppression factor S (Figure 7)
        S = self._suppression_factor_S(re_tp)

        # --- Macroconvective component ---
        # h_mac = 0.023 * Re_l^0.8 * Pr_l^0.4 * (k_l/D) * F
        h_mac = 0.023 * (re_l ** 0.8) * (pr_l ** 0.4) * (k_l / d_h) * F

        # --- Microconvective component (Forster-Zuber based) ---
        # h_mic = 0.00122 * [...] * Î”T^0.24 * Î”P^0.75 * S

        # Estimate wall superheat Î”T
        # For explicit calculation, approximate: Î”T â‰ˆ q / h_single_phase
        h_sp_approx = 0.023 * (re_l ** 0.8) * (pr_l ** 0.4) * (k_l / d_h)
        delta_T_approx = max(1.0, q_flux / h_sp_approx)  # minimum 1K

        # Vapor pressure difference corresponding to Î”T (Clausius-Clapeyron)
        # Î”P â‰ˆ (Ï_v * h_lv * Î”T) / T_sat
        T_sat = st_l.T
        delta_P = (rho_v * h_lv * delta_T_approx) / T_sat

        # Forster-Zuber microconvective coefficient
        g0 = 9.80665  # [m/sÂ²]

        h_mic_base = 0.00122 * (
                (k_l ** 0.79) * (cp_l ** 0.45) * (rho_l ** 0.49) * (g0 ** 0.25)
                / (
                        (sigma ** 0.5) * (mu_l ** 0.29)
                        * (h_lv ** 0.24) * (rho_v ** 0.24)
                )
        )

        h_mic = h_mic_base * (delta_T_approx ** 0.24) * (delta_P ** 0.75) * S

        # --- Total two-phase HTC ---
        h_tp = h_mac + h_mic

        if h_tp <= 0.0:
            raise ValueError("calculated h_tp <= 0")

        # Optional diagnostics
        try:
            fs_state.set("chen_htc", h_tp, "W/m2K", "Chen two-phase HTC")
            fs_state.set("chen_h_mac", h_mac, "W/m2K", "Macroconvective component")
            fs_state.set("chen_h_mic", h_mic, "W/m2K", "Microconvective component")
            fs_state.set("chen_F", F, "-", "Reynolds number factor")
            fs_state.set("chen_S", S, "-", "Suppression factor")
            fs_state.set("chen_x_tt", x_tt, "-", "Martinelli parameter")
        except Exception:
            pass

        return float(h_tp)