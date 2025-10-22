import math

from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import (
    TwoPhaseHeatTransfer,
)


class KandlikarTwoPhase(TwoPhaseHeatTransfer):
    """
    Kandlikar (1990) flow-boiling correlation integrated with VcLibPy.

    This class implements the TwoPhaseHeatTransfer interface and expects the
    *per-segment* quantities to be provided via FlowsheetState before `calc`
    is called.

    Reference:
    Kandlikar, S.G. (1990). "A General Correlation for Saturated Two-Phase
    Flow Boiling Heat Transfer Inside Horizontal and Vertical Tubes."
    Journal of Heat Transfer, Vol. 112, pp. 219-228.

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

    # Fluid-dependent parameter F_fl for different refrigerants
    # Table 4 from Kandlikar (1990)
    FLUID_PARAMETERS = {
        'Water': 1.00,
        'R11': 1.30,
        'R-11': 1.30,
        'R12': 1.50,
        'R-12': 1.50,
        'R13B1': 1.31,
        'R-13B1': 1.31,
        'R22': 2.20,
        'R-22': 2.20,
        'R113': 1.30,
        'R-113': 1.30,
        'R114': 1.24,
        'R-114': 1.24,
        'R134a': 1.63,
        'R-134a': 1.63,
        'R152a': 1.10,
        'R-152a': 1.10,
        'R32': 3.30,
        'R-32': 3.30,
        'R132': 3.30,
        'R-132': 3.30,
        'R141b':1.80,
        'R-141b': 1.80,
        'R124': 1.00,
        'R-124': 1.00,
        'Kerosene': 0.488,
        'Nitrogen': 4.70,
        'Neon': 3.50
    }

    # Constants from Table 3 of Kandlikar (1990)
    # Convective boiling region
    C1_CONV = 1.1360
    C2_CONV = -0.9
    C3_CONV = 667.2
    C4_CONV = 0.7
    C5_CONV = 0.3

    # Nucleate boiling region
    C1_NB = 0.6683
    C2_NB = -0.2
    C3_NB = 1058.0
    C4_NB = 0.7
    C5_NB = 0.3

    def __init__(self, orientation: str = "horizontal", fluid_name: str = None, f_fl: float = None):
        """
        Initialize Kandlikar correlation.

        Parameters
        ----------
        orientation : str, optional
            Tube orientation: 'horizontal' or 'vertical' (default: 'horizontal')
        fluid_name : str, optional
            Name of the fluid for looking up F_fl parameter
        f_fl : float, optional
            Explicit fluid-dependent parameter. If provided, overrides fluid_name lookup.
            If neither f_fl nor fluid_name is provided, defaults to 1.0 (water-like).
        """
        super().__init__()
        self.orientation = orientation.lower()
        if self.orientation not in ("horizontal", "vertical"):
            raise ValueError("orientation must be 'horizontal' or 'vertical'")

        # Determine fluid-dependent parameter F_fl
        if f_fl is not None:
            self.f_fl = float(f_fl)
        elif fluid_name is not None:
            self.f_fl = self.FLUID_PARAMETERS.get(fluid_name, 1.0)
        else:
            self.f_fl = 1.0  # Default to water

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
        Compute two-phase HTC using Kandlikar correlation with segment-local data.

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

        orientation = self._read_optional(fs_state, "seg_orientation", self.orientation)
        orientation = orientation.lower()
        if orientation not in ("horizontal", "vertical"):
            raise ValueError("orientation must be 'horizontal' or 'vertical'")

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

        for name, val in {"pr_l": pr_l, "mu_l": mu_l, "k_l": k_l,
                          "rho_l": rho_l, "rho_v": rho_v}.items():
            if val is None or val <= 0.0:
                raise ValueError(f"invalid property {name}={val}")

        # --- Kandlikar correlation ---
        A = math.pi * (d_h ** 2) / 4.0
        G = m_flow / A  # mass flux [kg/mÂ²s]

        # Safe vapor quality (avoid division by zero)
        x_safe = max(1.0e-6, min(1.0 - 1.0e-6, x))

        # Liquid Reynolds number (liquid-only flow)
        re_l = G * d_h * (1.0 - x_safe) / mu_l

        # Dittus-Boelter liquid-only heat transfer coefficient
        h_l = 0.023 * (re_l ** 0.8) * (pr_l ** 0.4) * (k_l / d_h)

        if h_l <= 0.0:
            raise ValueError(f"calculated h_l <= 0; Re_l={re_l}, Pr_l={pr_l}")

        # Boiling number
        bo = q_flux / (G * h_lv)

        # Convection number (Equation in paper)
        co = ((1.0 - x_safe) / x_safe) ** 0.8 * (rho_v / rho_l) ** 0.5

        # Froude number (liquid-only)
        g0 = 9.80665  # [m/sÂ²]
        fr_lo = G ** 2 / (rho_l ** 2 * g0 * d_h)

        # Froude correction: multiplier becomes unity for vertical or Fr_lo >= 0.04 (C5 = 0)
        if orientation == "vertical" or fr_lo >= 0.04:
            fr_factor_CONV = 1.0
            fr_factor_NB = 1.0
        else:
            # For horizontal tubes with Fr_lo < 0.04
            fr_factor_CONV = (25.0 * fr_lo) ** self.C5_CONV
            fr_factor_NB = (25.0 * fr_lo) ** self.C5_NB



        # Calculate HTC for convective boiling region
        h_conv = (self.C1_CONV * (co ** self.C2_CONV) * fr_factor_CONV +
                  (self.C3_CONV * (bo ** self.C4_CONV)) * self.f_fl) * h_l

        # Calculate HTC for nucleate boiling dominant region
        h_nb = (self.C1_NB * (co ** self.C2_NB) * fr_factor_NB +
                (self.C3_NB * (bo ** self.C4_NB)) * self.f_fl) * h_l

        # Two-phase HTC is the MAXIMUM of the two
        h_tp = max(h_conv, h_nb)

        if h_tp <= 0.0:
            raise ValueError("calculated h_tp <= 0")

        # Optional diagnostics
        try:
            fs_state.set("kandlikar_htc", h_tp, "W/m2K", "Kandlikar two-phase HTC")
            fs_state.set("kandlikar_h_conv", h_conv, "W/m2K", "Convective component")
            fs_state.set("kandlikar_h_nbd", h_nb, "W/m2K", "Nucleate boiling component")
            fs_state.set("kandlikar_co", co, "-", "Convection number")
            fs_state.set("kandlikar_bo", bo, "-", "Boiling number")
            fs_state.set("kandlikar_fr_lo", fr_lo, "-", "Froude number")
        except Exception:
            pass

        return float(h_tp)