import numpy as np
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import TwoPhaseHeatTransfer
from vclibpy.datamodels import FlowsheetState, Inputs
import logging

# --- Helper functions based on Thome (1995) ---

def bubble_point_T(w_local, P_sat, a0, a1, a2, a3, a4, b0, b1, b2, b3, b4):
    """
    Computes the bubble point temperature Tbub [K] using Thome's polynomial correlation.
    - Thome (1995), Equations (1)-(3)
    - Polynomial coefficients are default for R-22 but can be replaced for other refrigerant/oil pairs.
    - Valid only for w_local < 0.7 (mass fraction); above, the correlation is unreliable (see Thome p.113, 115).
    """
    A = a0 + a1 * w_local + a2 * w_local**3 + a3 * w_local**5 + a4 * w_local**7
    B = b0 + b1 * w_local + b2 * w_local**3 + b3 * w_local**5 + b4 * w_local**7
    delta_P = P_sat - B  # P_sat MUST be greater than B(w_local)
    if delta_P <= 0:
        # Physically invalid region: log not defined. See necessary check, Thome p.113 Table 1.
        raise ValueError("Invalid pressure difference (P_sat - B(w_local)) must be > 0.")
    return np.log(delta_P) / A

def cp_oil(T_C, spec_grav):
    """
    Computes specific heat capacity of the oil [kJ/kgK] as per Thome (1995) Eq. (6).
    - T_C: Oil temperature in Celsius
    - spec_grav: Specific gravity (dimensionless), measured at 15.6°C (see Table 2 and Eq.(6) Thome).

    Example:
        For HAF-68D1C oil with density = 0.9597 g/cm³ at 15°C,
        s = rho_oil / rho_water ≈ 0.9597 / 0.9991 ≈ 0.96
    """
    return 4.186 * (0.388 + 0.00045 * (1.8 * T_C + 32)) / spec_grav

def cp_mix(cp_ref, cp_oil_val, w_local):
    """
    Calculates the local mixture heat capacity [kJ/kgK] via mass-fraction weighted average.
    - Thome Eq. (7): cp_mix = w_local * cp_oil + (1 - w_local) * cp_ref
    - Used for the enthalpy balance, see Table 2 and Eq.(7) Thome.
    """
    return w_local * cp_oil_val + (1 - w_local) * cp_ref

# --- Main class: segmented Thome-based evaporator model ---

class ThomeSegmentedEvaporator(TwoPhaseHeatTransfer):
    """
    Segmented evaporator model using Thome's thermodynamic approach for refrigerant-lubricating oil mixtures.
    Divides the heat exchanger into vapor quality (= local x) zones and calculates all local thermodynamic and oil effects.
    """

    def __init__(self, d_h: float, A: float, n_segments: int = 20, correlation=None,
                 a_tuple=None, b_tuple=None, oil_sg=0.96):
        """
        Parameters & Assumptions:
        - d_h: Hydraulic diameter [m], typical for internal tube diameter of evaporator.
        - A: Total heat transfer surface area [m²]
        - n_segments: Number of vapor quality (x) zones. Typically 10–20 (Thome recommends 5–10+).
        - correlation: Two-phase HTC model, can be GungorWinterton, Shah, etc.
        - a_tuple/b_tuple: Thome's empirical polynomial coefficients for Eq.(2)&(3).
          Defaults for R-22, but should be replaced by fitted data for other refrigerants/oils, see Thome p.113/Table 1 and Appendix.
        - oil_sg: Specific gravity of oil at 15.6°C; measure from datasheet (see Thome p.116).
        """
        self.d_h = d_h
        self.A = A
        self.n_segments = n_segments
        self.correlation = correlation
        self.a_tuple = a_tuple or [-2394.5, 182.52, -724.21, 3868.0, -5268.9]
        self.b_tuple = b_tuple or [8.0736, -0.72212, 2.3914, -13.779, 17.066]
        self.oil_sg = oil_sg

    def calc(self, state_q0, state_q1, inputs: Inputs, fs_state: FlowsheetState,
             m_flow, med_prop, state_inlet, state_outlet) -> float:
        """
        Main calculation routine applying Thome's thermodynamic approach. (Thome 1995, Section "Heat Release Curves", p.115ff)
        - Divides the two-phase region into n_segments along vapor quality (x), up to the physically-correct maximum x.
        - Computes all local properties and heat transfer in each segment:
          * Local oil concentration, bubble point temperature (Tbub), mixture cp, latent and sensible enthalpy terms.
          * Local HTC (alpha) is called per segment from chosen correlation, always using Tbub as reference T (Thome Eq.9).
        - Strictly respects Thome's limits on x (1-w_inlet) and w_local (<0.7).
        """

        P_sat = state_q0.p / 1e6  # Saturation pressure in MPa (see Eq.(1))
        w_inlet = getattr(inputs, "w_oil", 0.03)  # Default 3% oil mass fraction, typical for refrigeration systems [Thome Table 2]

        x_max = 1.0 - w_inlet  # Per Thome, S.115: Maximum vapor quality, as oil is non-volatile (x_max = 1 - w_inlet)
        dx = x_max / self.n_segments  # Step size for segmentation

        a0, a1, a2, a3, a4 = self.a_tuple
        b0, b1, b2, b3, b4 = self.b_tuple

        total_Q = 0.0  # Will accumulate evaporator duty over all valid segments

        # Arrays for scientific profiles (for later plotting, validation, reporting)
        x_vals, w_vals, T_vals, alpha_vals, Q_vals = [], [], [], [], []

        for i in range(self.n_segments):
            x0 = i * dx
            x1 = x0 + dx
            x_mean = 0.5 * (x0 + x1)
            if x_mean > x_max:
                # Physically, cannot evaporate more than allowed by inlet oil concentration [Thome, p.115]
                break

            denominator = 1 - x_mean
            if denominator <= 0:
                # Just in case of numerical edge case; no physical solution for x_mean ≥ 1.
                break

            w_local = w_inlet / denominator  # Thome Eq.(5): Local oil mass fraction as function of vapor quality
            if w_local > 0.7:
                # As supported by Thome (p.113, Table 1): Bubble-point polynomial only valid up to 70% oil in liquid phase
                break

            try:
                Tbub_K = bubble_point_T(w_local, P_sat, a0, a1, a2, a3, a4,
                                        b0, b1, b2, b3, b4)  # Thome Eq.(1)-(3)
                # This integral step models the rise of Tbub along the evaporator, reflecting oil buildup [Thome, p.115/Table 2]
            except ValueError:
                # If polynomial fails, segment loop is aborted; this is recommended by Thome.
                import logging
                logging.warning(f"Aborting segment {i} due to invalid bubble-point calc: "
                                f"x_mean={x_mean:.3f}, w_local={w_local:.3f}")
                break

            Tbub_C = Tbub_K - 273.15  # Convert K → °C for cp_oil formula

            # Calculate reference cp for refrigerant at current Tbub.
            try:
                tp = med_prop.calc_transport_properties(T=Tbub_K)
                cp_ref = tp.cp  # [J/kgK] from property correlations
                cp_ref_kJ = cp_ref / 1000.0  # Convert to [kJ/kgK] for mass-fraction mixing
                h_lv_local = med_prop.h_lv(T=Tbub_K)  # Latent heat at local bubble point
            except Exception:
                # Fallback for property database error.
                cp_ref_kJ = 1.8  # Typical value for refrigerant cp [kJ/kgK]
                h_lv_local = state_q1.h - state_q0.h  # Approximate latent heat from inlet/outlet enthalpy

            cp_oil_val = cp_oil(Tbub_C, self.oil_sg)  # Oil cp, from Thome Eq.(6), using oil SG from datasheet or experiment. See Table 2.
            cp_mix_val = cp_mix(cp_ref_kJ, cp_oil_val, w_local)  # Mixture cp, Thome Eq.(7)
            cp_mix_val_J = cp_mix_val * 1000  # [J/kgK], for internal consistency

            # Compute bubble point temperature rise across this segment [Thome p.115/Table 2]
            Tbub_K0 = bubble_point_T(w_inlet / (1 - x0), P_sat, a0, a1, a2, a3, a4, b0, b1, b2, b3, b4)
            Tbub_K1 = bubble_point_T(w_inlet / (1 - x1), P_sat, a0, a1, a2, a3, a4, b0, b1, b2, b3, b4)
            dT_bub = Tbub_K1 - Tbub_K0

            # Calculate heat added in this segment (per Thome Eq.(4)): Latent + sensible heating contributions
            dH_latent = h_lv_local * dx                                 # Latent term: evaporation in Δx segment
            dH_sens_liq = (1 - x_mean) * cp_mix_val_J * dT_bub         # Sensible: heating liquid mixture (oil+refrigerant)
            dH_sens_vap = x_mean * cp_ref * dT_bub                     # Sensible: heating the vapor phase
            dH_zone = m_flow * (dH_latent + dH_sens_liq + dH_sens_vap)
            total_Q += dH_zone

            # Local HTC calculation: must use Tbub as reference temperature in all correlations [Thome Eq.(9)]
            if self.correlation:
                alpha = self.correlation.calc(
                    x=x_mean,
                    T_bub=Tbub_K,
                    cp=cp_mix_val_J,
                    h_lv=h_lv_local,
                    m_flow=m_flow,
                    d_h=self.d_h,
                )
            else:
                alpha = 1000  # fallback; update for real experimental data if needed

            # Store all local values for reporting, validation, charts, diagnostics.
            fs_state.set(name=f"x_zone_{i}", value=x_mean)
            fs_state.set(name=f"w_local_{i}", value=w_local)
            fs_state.set(name=f"T_bub_{i}", value=Tbub_K)
            fs_state.set(name=f"cp_mix_{i}", value=cp_mix_val_J)
            fs_state.set(name=f"htc_{i}", value=alpha)
            fs_state.set(name=f"Q_zone_{i}", value=dH_zone)

            x_vals.append(x_mean)
            w_vals.append(w_local)
            T_vals.append(Tbub_C)
            alpha_vals.append(alpha)
            Q_vals.append(dH_zone)

        # Save full arrays for scientific plots or postprocessing as needed for master thesis/experiments.
        self.x_profile = np.array(x_vals)
        self.w_profile = np.array(w_vals)
        self.Tbub_profile = np.array(T_vals)
        self.alpha_profile = np.array(alpha_vals)
        self.dQ_profile = np.array(Q_vals)

        return total_Q  # Return total heat transfer for the evaporator

