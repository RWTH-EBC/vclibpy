import math
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import (
    TwoPhaseHeatTransfer,
)


class GungorWintertonTwoPhase87(TwoPhaseHeatTransfer):
    """
    Gungor & Winterton (1987) flow-boiling correlation integrated with VcLibPy.

    This class implements the TwoPhaseHeatTransfer interface and expects the
    *per-segment* quantities to be provided via FlowsheetState before `calc`
    is called:

    Required FlowsheetState variables (set by the segmented HX loop):
        - 'seg_x'       : representative vapor quality for the segment (0 < x < 1)
        - 'seg_q_flux'  : local heat flux q'' in W/m^2

    Optional FlowsheetState variables:
        - 'seg_d_h'         : hydraulic diameter [m] (defaults to self.d_h)
        - 'seg_orientation' : 'horizontal' or 'vertical'
                              (defaults to self.orientation)
        - 'seg_p'           : segment pressure [Pa]
                              (defaults to state_inlet.p)

    All fluid properties are obtained from `med_prop`.
    """

    def __init__(self, orientation: str = "horizontal"):
        super().__init__()
        self.orientation = orientation.lower()

        if self.orientation not in ("horizontal", "vertical"):
            raise ValueError("orientation must be 'horizontal' or 'vertical'")

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
        Compute two-phase HTC using Gungor & Winterton with segment-local data.

        Expects the calling code to have inserted segment-local values into
        `fs_state` under the keys described in the class docstring.
        """
        if med_prop is None:
            raise ValueError("med_prop is required")
        if m_flow is None or m_flow <= 0.0:
            raise ValueError("m_flow must be > 0")

        # --- segment-local inputs from FlowsheetState ---
        x = self._read_required(fs_state, "seg_x")
        q_flux = self._read_required(fs_state, "seg_q_flux")
        d_h = self._read_required(fs_state, "seg_d_h")

        if d_h <= 0.0:
            raise ValueError(f"hydraulic diameter must be > 0; got {d_h}")

        orientation = self.orientation.lower()
        if orientation not in ("horizontal", "vertical"):
            raise ValueError("orientation must be 'horizontal' or 'vertical'")

        p_var = fs_state.get("seg_p", None)
        p = float(p_var.value) if (p_var and p_var.value is not None) else float(
            state_inlet.p
        )

        # --- validation ---
        if not (0.0 <= x <= 1.0):
            raise ValueError(f"segment vapor quality must be in (0,1); got {x}")
        if q_flux <= 0.0:
            raise ValueError(f"seg_q_flux must be > 0; got {q_flux}")
        if p <= 0.0:
            raise ValueError(f"pressure must be > 0; got {p}")
        if d_h <= 0.0:
            raise ValueError(f"hydraulic diameter must be > 0; got {d_h}")

        # --- saturation states and transport properties ---
        st_l = med_prop.calc_state("PQ", p, 0.0)  # saturated liquid
        st_v = med_prop.calc_state("PQ", p, 1.0)  # saturated vapor
        h_lv = st_v.h - st_l.h
        if h_lv <= 0.0:
            raise ValueError(f"latent heat <= 0 at p={p}")

        tp_l = med_prop.calc_transport_properties(st_l)
        tp_v = med_prop.calc_transport_properties(st_v)

        # Handle possible attribute name variants (Pr vs. pr)
        pr_l = getattr(tp_l, "Pr", getattr(tp_l, "pr", None))
        mu_l = tp_l.dyn_vis
        mu_v = tp_v.dyn_vis
        k_l = tp_l.lam
        rho_l, rho_v = st_l.d, st_v.d

        for name, val in dict(pr_l=pr_l, mu_l=mu_l, mu_v=mu_v, k_l=k_l,
                              rho_l=rho_l, rho_v=rho_v).items():
            if val is None or val <= 0.0:
                raise ValueError(f"invalid property {name}={val}")

        # --- Gungor & Winterton correlation ---
        A = math.pi * (d_h ** 2) / 4.0
        G = m_flow / A  # mass flux [kg/m^2/s]

        x_safe = max(1.0e-3, min(1.0 - 1.0e-3, x))

        # Reynolds number for all mass flowing as liquid
        re_l = G * d_h * (1 - x_safe) / mu_l

        # Dittus–Boelter: liquid-only reference HTC
        h_lo = 0.023 * (re_l ** 0.8) * (pr_l ** 0.4) * (k_l / d_h)

        # Boiling number
        bo = q_flux / (G * h_lv)

        # Enhancement factor
        e = 1 + 3000 * (bo ** 0.86) + 1.12 * ((x_safe/(1 - x_safe)) ** 0.75) * (rho_l / rho_v) ** 0.41

        # Horizontal low-Froude correction
        if orientation == "horizontal":
            g0 = 9.80665
            # Froude number based on liquid phase
            fr = G ** 2 / (rho_l ** 2 * g0 * d_h)

            if fr < 0.05:
                e *= fr ** (0.1 - 2 * fr)

        # --- Final two-phase heat transfer coefficient ---
        h_tp = e * h_lo

        if h_tp <= 0.0:
            raise ValueError("calculated h_tp <= 0")

        # Optional diagnostics
        try:
            fs_state.set("gw1987_htc", h_tp, "W/m2K", "G&W local two-phase HTC")
            fs_state.set("gw1987_x", x, "-", "segment vapor quality used")
            fs_state.set("gw1987_q_flux", q_flux, "W/m2", "segment heat flux used")
        except Exception:
            pass

        return float(h_tp)