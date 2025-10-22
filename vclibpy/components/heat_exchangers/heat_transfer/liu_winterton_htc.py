import math

from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import (
    TwoPhaseHeatTransfer,
)


class LiuWintertonTwoPhase(TwoPhaseHeatTransfer):
    """
    Liu & Winterton (1991) flow-boiling correlation integrated with VcLibPy.

    This class implements the TwoPhaseHeatTransfer interface and expects the
    *per-segment* quantities to be provided via FlowsheetState before `calc`
    is called.

    Reference:
    Liu, Z. & Winterton, R.H.S. (1991). "A General Correlation for Saturated
    and Subcooled Flow Boiling in Tubes and Annuli, Based on a Nucleate Pool
    Boiling Equation." Int. J. Heat Mass Transfer, Vol. 34, No. 11,
    pp. 2759-2766.

    This correlation improves upon Gungor-Winterton (1986) by:
    - Using asymptotic combination: h_tp = sqrt((F*h_l)^2 + (S*h_pool)^2)
    - Including Prandtl number effect in enhancement factor F
    - Removing boiling number dependence
    - Better accuracy for subcooled boiling and cryogenic fluids

    Required FlowsheetState variables (set by the segmented HX loop):
    - 'seg_x'       : representative vapor quality for the segment (0 < x < 1)
    - 'seg_q_flux'  : local heat flux q'' in W/m^2
    - 'seg_d_h'     : hydraulic diameter [m]

    Optional FlowsheetState variables:
    - 'seg_orientation' : 'horizontal' or 'vertical'
                          (defaults to self.orientation)
    - 'seg_p'           : segment pressure [Pa]
                          (defaults to state_inlet.p)

    All fluid properties are obtained from `med_prop`.
    """

    def __init__(self, orientation: str = "horizontal"):
        """
        Initialize Liu & Winterton two-phase HTC model.

        Args:
            orientation: Flow orientation, either 'horizontal' or 'vertical'
        """
        super().__init__()
        self.orientation = orientation.lower()
        if self.orientation not in ("horizontal", "vertical"):
            raise ValueError(
                f"orientation must be 'horizontal' or 'vertical'; got '{orientation}'"
            )

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
        Compute two-phase HTC using Liu & Winterton (1991) correlation with
        segment-local data.

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

        orientation_var = fs_state.get("seg_orientation", None)
        orientation = (
            str(orientation_var.value).lower()
            if (orientation_var and orientation_var.value is not None)
            else self.orientation
        )

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

        # Handle possible attribute name variants
        pr_l = getattr(tp_l, "Pr", getattr(tp_l, "pr", None))
        mu_l = tp_l.dyn_vis
        k_l = tp_l.lam
        rho_l = st_l.d
        rho_v = st_v.d

        # Critical pressure for reduced pressure calculation
        _, p_c, _ = med_prop.get_critical_point()

        p_r = p / p_c  # reduced pressure

        # Molecular weight (for Cooper correlation)
        M = float(med_prop.get_molar_mass()) * 1000 # kg/mol -> kg/kmol

        for name, val in {"pr_l": pr_l, "mu_l": mu_l, "k_l": k_l,
                          "rho_l": rho_l, "rho_v": rho_v}.items():
            if val is None or val <= 0.0:
                raise ValueError(f"invalid property {name}={val}")

        # --- Mass flux ---
        A = math.pi * (d_h ** 2) / 4.0
        G = m_flow / A  # mass flux [kg/mÂ²s]

        # Safe vapor quality (avoid division by zero)
        x_safe = max(1.0e-6, min(1.0 - 1.0e-6, x))

        # --- Reynolds number (liquid phase) ---
        # !!! different definition than in Gungor & Winterton (1986): re_l = G * d_h * (1 - x) / mu_l
        re_l = G * d_h / mu_l

        if re_l <= 0.0:
            raise ValueError(f"Reynolds number Re_l <= 0")

        # --- Liquid-only heat transfer coefficient (Dittus-Boelter) ---
        h_lo = 0.023 * (k_l / d_h) * (re_l ** 0.8) * (pr_l ** 0.4)

        # --- Enhancement factor F (Equation 13) ---
        F = (1.0 + x_safe * pr_l * (rho_l / rho_v - 1.0)) ** 0.35

        # --- Pool boiling coefficient (Cooper correlation, Equation 4) ---
        if p_r >= 1.0:
            # Above critical pressure, set h_pool to zero
            h_pool = 0.0
        else:
            h_pool = (
                    55.0 * (p_r ** 0.12) * (q_flux ** (2/3)) * ((-math.log10(p_r)) ** -0.55) * (M ** -0.5)
            )

        # --- Suppression factor S (Equation 14) ---
        S = (1.0 + 0.055 * (F ** 0.1) * (re_l ** 0.16)) ** -1

        # --- Froude number correction for horizontal tubes (Equations 15-16) ---
        if orientation == "horizontal":
            g0 = 9.80665  # m/s²
            fr = G ** 2 / (rho_l ** 2 * g0 * d_h)

            if fr < 0.05:
                # Froude number corrections
                e_fr = fr ** (0.1 - 2.0 * fr)
                e_s = fr ** 0.5
                F *= e_fr
                S *= e_s

        # --- Asymptotic combination (Equation 2) ---
        h_tp = math.sqrt((F * h_lo) ** 2 + (S * h_pool) ** 2)

        if h_tp <= 0.0:
            raise ValueError("calculated h_tp <= 0")

        # Optional diagnostics
        try:
            fs_state.set("liu_winterton_htc", h_tp, "W/m2K", "Liu-Winterton two-phase HTC")
            fs_state.set("liu_winterton_F", F, "-", "Enhancement factor F")
            fs_state.set("liu_winterton_S", S, "-", "Suppression factor S")
            fs_state.set("liu_winterton_h_lo", h_lo, "W/m2K", "Liquid-only HTC")
            fs_state.set("liu_winterton_h_pool", h_pool, "W/m2K", "Pool boiling HTC")
            fs_state.set("liu_winterton_convective", F * h_lo, "W/m2K", "Convective contribution")
            fs_state.set("liu_winterton_nucleate", S * h_pool, "W/m2K", "Nucleate boiling contribution")
        except Exception:
            pass

        return float(h_tp)