import math
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import (
    TwoPhaseHeatTransfer,
)


class Shah2022TwoPhase(TwoPhaseHeatTransfer):
    """
    Shah (2022) flow-boiling correlation integrated with VcLibPy.

    Reference:
    Shah, M.M. (2022). New general correlation for heat transfer during
    saturated boiling in mini and macro channels. International Journal
    of Refrigeration, 137, 103-116.

    This class implements the TwoPhaseHeatTransfer interface and expects the
    *per-segment* quantities to be provided via FlowsheetState before `calc`
    is called:

    Required FlowsheetState variables (set by the segmented HX loop):
        - 'seg_x'       : representative vapor quality for the segment (0 < x < 1)
        - 'seg_q_flux'  : local heat flux q'' in W/m^2
        - 'seg_d_h'     : hydraulic diameter [m] (DHYD)

    Optional FlowsheetState variables:
        - 'seg_orientation' : 'horizontal' or 'vertical'
                              (defaults to self.orientation)
        - 'seg_p'           : segment pressure [Pa]
                              (defaults to state_inlet.p)

    All fluid properties are obtained from `med_prop`.

    Notes:
    ------
    - Shah (2022) defines two diameters (Eqs. 13, 14):
      * DHYD = 4 * Flow Area / Wetted Perimeter
      * DHP  = 4 * Flow Area / Heated Perimeter
    - For fully heated circular tubes (standard heat pump evaporators): DHP = DHYD
    - This implementation assumes fully heated circular tubes (DHP = DHYD)
    - DHYD is used in Weber and Froude number calculations
    - DHP is used in single-phase heat transfer coefficient calculation (Eq. 9)
    - Surface tension is obtained from TransportProperties.sur_ten calculated
      at the two-phase state (P, Q) using med_prop wrapper
    """

    def __init__(self, orientation: str = "horizontal", is_co2: bool = False):
        """
        Initialize Shah 2022 correlation.

        Parameters:
        -----------
        orientation : str
            Tube orientation, either 'horizontal' or 'vertical'
        is_co2 : bool
            Set to True if the fluid is CO2 (uses different correlation for psi_0)
        """
        super().__init__()
        self.orientation = orientation.lower()
        self.is_co2 = is_co2

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
        Compute two-phase HTC using Shah 2022 correlation with segment-local data.

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
        d_h = self._read_required(fs_state, "seg_d_h")  # hydraulic diameter (DHYD)

        if d_h <= 0.0:
            raise ValueError(f"hydraulic diameter must be > 0; got {d_h}")

        # For fully heated circular tubes: DHP = DHYD
        # (Shah 2022 defines DHYD in Eq. 13, DHP in Eq. 14)
        # In standard heat pump evaporators, the entire perimeter is heated
        d_hp = d_h  # DHP = DHYD for circular tubes

        orientation = self._read_optional(fs_state, "seg_orientation", self.orientation).lower()
        if orientation not in ("horizontal", "vertical"):
            raise ValueError("orientation must be 'horizontal' or 'vertical'")

        p_var = fs_state.get("seg_p", None)
        p = float(p_var.value) if (p_var and p_var.value is not None) else float(
            state_inlet.p
        )

        # --- validation ---
        if not (0.0 <= x <= 1.0):
            raise ValueError(f"segment vapor quality must be in [0,1]; got {x}")
        if q_flux <= 0.0:
            raise ValueError(f"seg_q_flux must be > 0; got {q_flux}")
        if p <= 0.0:
            raise ValueError(f"pressure must be > 0; got {p}")

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

        # Get surface tension at the actual segment vapor quality x
        # Surface tension is stored in TransportProperties.sur_ten
        # Calculate it at the two-phase state using med_prop wrapper
        try:
            # Calculate state at actual vapor quality
            st_x = med_prop.calc_state("PQ", p, x)

            # Calculate transport properties at this two-phase state
            # TransportProperties contains sur_ten (surface tension in N/m)
            tp_x = med_prop.calc_transport_properties(st_x)

            # Get surface tension from TransportProperties
            sigma = tp_x.sur_ten

            if sigma is None or sigma <= 0.0:
                raise ValueError(f"surface tension returned invalid value: {sigma}")

        except (AttributeError, Exception) as e:
            raise ValueError(
                f"Surface tension could not be determined at x={x}, p={p}: {e}. "
                "Surface tension is required for Shah 2022 correlation (Weber number calculation). "
                "Ensure your med_prop backend (CoolProp/REFPROP) supports surface tension calculation "
                "and that TransportProperties.sur_ten is available."
            )

        # --- Shah 2022 correlation (Section 3.2) ---

        # Calculate flow area and mass flux G
        A_flow = math.pi * (d_h ** 2) / 4.0
        G = m_flow / A_flow  # mass flux [kg/m^2/s]

        # Safe vapor quality to avoid division by zero
        x_safe = max(1.0e-3, min(1.0 - 1.0e-3, x))

        # Calculate single-phase liquid heat transfer coefficient hLS
        # Using DHP for single-phase HTC calculation (Eq. 9)
        # For circular tubes, DHP = DHYD = d_h
        re_ls = G * (1 - x_safe) * d_hp / mu_l
        h_ls = 0.023 * (re_ls ** 0.8) * (pr_l ** 0.4) * (k_l / d_hp)

        # Calculate dimensionless numbers
        # Bo: Boiling number
        bo = q_flux / (G * h_lv)

        # Co: Convection number (Eq. 7)
        co = ((1.0 - x_safe) / x_safe) ** 0.8 * (rho_v / rho_l) ** 0.5

        # FrLT: Froude number (Eq. 6) - using DHYD
        g0 = 9.80665  # m/s^2
        fr_lt = G ** 2 / (g0 * d_h * rho_l ** 2)

        # WeGT: Weber number (Eq. 11) - using DHYD
        we_gt = G ** 2 * d_h / (rho_v * sigma)

        # Calculate n for parameter J (Eq. 5)
        if orientation == "horizontal" and fr_lt < 0.04:
            n = 1.0
        else:
            n = 0.0

        # Calculate J parameter (Eq. 5)
        j = ((0.38 * (fr_lt ** -0.3)) ** n) * co

        # Calculate psi_0 (Eq. 21 for non-CO2, Eq. 22 for CO2)
        if self.is_co2:
            psi_0 = 1820.0 * (bo ** 0.68)  # Eq. 22
        else:
            psi_0 = 1.0 + 560.0 * (bo ** 0.65)  # Eq. 21

        # Calculate psi_cb (Eq. 23)
        psi_cb = 2.0 / (j ** 0.8)

        # Calculate psi_bs (Eq. 24)
        psi_bs = psi_0 * (1.0 + 0.16 / (j ** 0.87))

        # Take maximum of psi_0, psi_cb, and psi_bs
        psi = max(psi_0, psi_cb, psi_bs)

        # Calculate surface tension factor Fst (Eq. 12)
        # For horizontal channels with FrLT < 0.04, Fst = 1 (stratification)
        if orientation == "horizontal" and fr_lt < 0.04:
            f_st = 1.0
        else:
            f_st_calc = (2.1 - 0.008 * we_gt - 110 * bo)
            f_st = max(1.0, f_st_calc)

        # Calculate final two-phase heat transfer coefficient
        h_tp = f_st * psi * h_ls

        if h_tp <= 0.0:
            raise ValueError("calculated h_tp <= 0")

        # Optional diagnostics
        try:
            fs_state.set("shah2022_htc", h_tp, "W/m2K", "Shah 2022 local two-phase HTC")
            fs_state.set("shah2022_x", x, "-", "segment vapor quality used")
            fs_state.set("shah2022_q_flux", q_flux, "W/m2", "segment heat flux used")
            fs_state.set("shah2022_psi", psi, "-", "psi multiplier")
            fs_state.set("shah2022_fst", f_st, "-", "surface tension factor")
            fs_state.set("shah2022_bo", bo, "-", "Boiling number")
            fs_state.set("shah2022_fr", fr_lt, "-", "Froude number")
            fs_state.set("shah2022_we", we_gt, "-", "Weber number")
            fs_state.set("shah2022_hls", h_ls, "W/m2K", "single-phase liquid HTC")
            fs_state.set("shah2022_sigma", sigma, "N/m", "surface tension")
        except Exception:
            pass

        return float(h_tp)