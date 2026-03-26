"""
Semi-empirical rolling-piston compressor model after Molinaroli et al. (2017),
extended by:
    1. Giuffrida-like viscosity-dependent friction-loss formulation
    2. Oil circulation path (Ansatz 1: oil does not participate in compression)

Oil path
--------
The oil is pumped from the high-pressure sump (p_dis, T_oil_sump) through
the crankshaft into the suction chamber at p_suc.

Suction side:
    - Throttling from p_dis to p_suc causes partial degassing at T_oil_sump.
    - The liquid stream exchanges heat with the fictitious wall (Q_suc_oil,
      computed via enthalpy balance). Back-dissolution is permitted.
    - Additional degassing/dissolution at T_oil_after is tracked separately.
    - Residuum 4 uses SPLIT degassing: throttle part at T_oil_sump,
      HT part at T_oil_after, each with its own enthalpy.

Discharge side (James Eq. 17-18 approach):
    - The total gas mass flow through the discharge valve and discharge HT
      is m_gas = m_suc + m_KM_degas_total (mass-consistent with R3).
    - After the discharge valve, the gas stream and the oil stream (at
      T_oil_after, p_suc composition) mix adiabatically in the discharge
      plenum. The discharge valve itself sees only gas (Variante A).
    - The combined stream undergoes discharge heat transfer with the wall using
      a single NTU-epsilon calculation with the total mass flow.
    - Oil cp uses calc_cp_mix (oil-KM mixture), not pure oil cp.

Residuum 3 follows the James/Molinaroli convention: the oil does NOT reduce
the effective gas volume in the cylinder.

Hydraulic oil recirculation loss W_dot_oil_recirc is included as a parasitic
loss (not thermodynamic compression work). Uses m_dot_fl and rho_fl consistently.

When the oil path calculation fails (e.g. solubility solver), the residuals
return large penalty values instead of silently falling back to a no-oil model.

A predictor-corrector scheme is used for T_dis: first a gas-only discharge
HT provides T_dis_est, then the oil path is computed, then a combined
discharge HT gives T_dis_final, and the oil path is recomputed with this
corrected T_dis. This closes the coupling without a full inner iteration.

m_flow is the EXTERNAL suction refrigerant mass flow rate (catalogue value).
m_dot_gas_discharge = m_suc + m_KM_degas_total is the gas flow through the
discharge valve and discharge HT (excludes internal leakage recirculation).

Outlet separation at (T_dis, p_dis) is computed as a DIAGNOSTIC only.

Fitting parameters:
    Molinaroli base:  Ua_suc_ref, Ua_dis_ref, Ua_amb, A_tot, A_dis,
                      V_IC, alpha_loss, W_dot_loss_ref
    Viscosity ext.:   alpha_fric_tot
    Oil path:         m_dot_oil_ref, Ua_suc_oil_ref

References:
    Molinaroli et al. (2017), doi:10.1016/j.ijrefrig.2017.04.015
    James et al. (2016), doi:10.1016/j.ijrefrig.2015.12.011
"""
from scipy.optimize import least_squares
import os
import numpy as np

from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs, FlowsheetState
from vclibpy.media import ThermodynamicState

from vclibpy.media.lubricant_fitting_shared_refprop import LubricantFitting as SharedLubricantFitting


ENABLE_TIMING = True


class Molinaroli_2017_Compressor_Oil_Path(Compressor):

    _LUBRICANT_MODEL_CACHE = {}

    # -----------------------------------------------------------------
    # Lubricant model management
    # -----------------------------------------------------------------
    @classmethod
    def _build_lubricant_model(cls, fluid_name, lub_name, shared_refprop=None):
        if shared_refprop is not None:
            try:
                model = SharedLubricantFitting(
                    fluid_name=fluid_name, lub_name=lub_name,
                    shared_refprop=shared_refprop)
            except TypeError:
                model = SharedLubricantFitting(fluid_name=fluid_name, lub_name=lub_name)
                if hasattr(model, "refrigerant_prop"):
                    model.refrigerant_prop = shared_refprop
        else:
            model = SharedLubricantFitting(fluid_name=fluid_name, lub_name=lub_name)
        if shared_refprop is not None and hasattr(model, "refrigerant_prop"):
            model.refrigerant_prop = shared_refprop
        return model

    @classmethod
    def _get_cached_lubricant_model(cls, fluid_name, lub_name, shared_refprop=None):
        key = (os.getpid(), str(fluid_name).strip().lower(),
               str(lub_name).strip().lower(),
               id(shared_refprop) if shared_refprop is not None else None)
        if key not in cls._LUBRICANT_MODEL_CACHE:
            cls._LUBRICANT_MODEL_CACHE[key] = cls._build_lubricant_model(
                fluid_name, lub_name, shared_refprop)
        return cls._LUBRICANT_MODEL_CACHE[key]

    def _get_lubricant_model(self):
        shared_refprop = getattr(self, "med_prop", None)
        model = self._get_cached_lubricant_model(
            self.fluid_name, self.lub_name, shared_refprop)
        if shared_refprop is not None and hasattr(model, "refrigerant_prop"):
            model.refrigerant_prop = shared_refprop
        self.lubricant_model = model
        return model

    # -----------------------------------------------------------------
    # Constructor
    # -----------------------------------------------------------------
    def __init__(self, N_max, V_h,
                 fluid_name="propane", lub_name="LPG 68", parameters=None):
        super().__init__(N_max=N_max, V_h=V_h)

        if parameters is None:
            parameters = {
                "Ua_suc_ref": 16.05,
                "Ua_dis_ref": 13.96,
                "Ua_amb": 0.36,
                "A_tot": 9.47e-9,
                "A_dis": 86.1e-6,
                "V_IC": 16.11e-6,
                "alpha_loss": 0.16,
                "W_dot_loss_ref": 83.0,
                "alpha_fric_tot": 0.0,
                "m_dot_ref": 0.0083,
                "f_ref": 50.0,
                "mu_fallback": 5.0,
                "m_dot_oil_ref": 0.001,
                "Ua_suc_oil_ref": 5.0,
            }

        self.parameters = dict(parameters)
        self.fluid_name = fluid_name
        self.lub_name = lub_name
        self.lubricant_model = None

        # Thermodynamic states
        self.state_c_1 = None
        self.state_c_3 = None
        self.state_c_4 = None
        self.state_c_5 = None

        # Solver results
        self.T_w = None
        self.state_outlet = None
        self.W_dot_comp = None
        self.P_el = None

        # Viscosity outputs
        self.T_oil_sump = None
        self.mu_oil = None
        self.mu_mix_eff = None

        # Loss outputs
        self.W_dot_int = None
        self.W_dot_loss = None
        self.W_dot_loss_load = None
        self.W_dot_loss_ref_term = None
        self.W_dot_loss_fric = None

        # Oil path outputs
        self.m_dot_oil = None
        self.Q_dot_suc_oil = None
        self.W_dot_oil_recirc = None
        self.m_dot_KM_degas_thr = None
        self.m_dot_KM_degas_ht = None
        self.m_dot_KM_degas_total = None
        self.m_dot_fl = None
        self.w_KM_sump = None
        self.w_KM_after = None
        self.T_oil_after = None
        # Diagnostic
        self.m_dot_KM_gas = None
        self.T_KM_gas = None
        self.m_dot_gas_discharge = None  # Gas through discharge valve (m_suc + m_degas)
        self.m_dot_KM_degas_ht_raw = None  # Raw HT degassing before limiting
        self.w_KM_after_raw = None  # Raw solubility after HT (before cap)
        self.Q_dis_total = None  # Total discharge HT (gas + oil combined)
        self.T_dis_est = None    # Predictor T_dis (gas-only)
        self.T_dis_corr = None   # Corrector T_dis (combined)

        # Per-residual-call storage
        self._current_oil_path = None
        self._dis_ht_result = None
        self._T_dis_est = None
        self._T_dis_corr = None

        # Solver caches
        self._state_cache = {}
        self._last_x = None
        self._cache_hits = 0
        self._cache_misses = 0
        self._cached_discharge_valve = {}
        self._oil_viscosity_cache = {}

        # Debug
        self.debug_enabled = False
        self._gamma4_min = None
        self._gamma4_max = None
        self._gamma4_n = 0
        self._corrector_fallback_count = 0
        self._solver_fallback_count = 0

    # -----------------------------------------------------------------
    # Helpers
    # -----------------------------------------------------------------
    @staticmethod
    def _kelvin_to_celsius(T_K):
        return T_K - 273.15

    @staticmethod
    def _celsius_to_kelvin(T_C):
        return T_C + 273.15

    @staticmethod
    def _milli_pas_to_pas(mu_mpas):
        return float(mu_mpas) * 1e-3

    def _get_ambient_temperature(self, inputs):
        return float(getattr(inputs, "T_amb", 25.0 + 273.15))

    def _calculate_oil_sump_temperature(self, T_dis_K, T_in_K, T_amb_K):
        T_dis_C = self._kelvin_to_celsius(T_dis_K)
        T_in_C = self._kelvin_to_celsius(T_in_K)
        T_amb_C = self._kelvin_to_celsius(T_amb_K)
        T_oil_C = 0.914227 * T_dis_C + 0.008136 * T_in_C + 0.006144 * T_amb_C
        return self._celsius_to_kelvin(T_oil_C)

    def _calculate_oil_viscosity(self, T_oil_K, p_oil):
        cache_key = (round(T_oil_K, 3), round(p_oil, 1))
        if cache_key in self._oil_viscosity_cache:
            return self._oil_viscosity_cache[cache_key]
        lubricant = self._get_lubricant_model()
        oil_state = ThermodynamicState(p=p_oil, T=T_oil_K)
        try:
            transport = lubricant.calc_transport_properties(state=oil_state, phase="liquid")
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in oil viscosity: {err}")
            transport = None
        mu_fb = float(self.parameters.get("mu_fallback", 5.0))
        if transport is None or transport.dyn_vis is None:
            mu = mu_fb
        else:
            mu = float(transport.dyn_vis)
            if not np.isfinite(mu) or mu <= 0.0:
                mu = mu_fb
        self._oil_viscosity_cache[cache_key] = mu
        if len(self._oil_viscosity_cache) > 1000:
            del self._oil_viscosity_cache[next(iter(self._oil_viscosity_cache))]
        return mu

    def _calculate_ntu_effectiveness(self, Ua_ref, m_dot, cp, m_dot_ref):
        if m_dot < 1e-10 or cp < 1e-10:
            return 0.0, 0.0
        NTU = (Ua_ref / (m_dot * cp)) * (m_dot / m_dot_ref) ** 0.8
        NTU = min(NTU, 100.0)
        return NTU, 1.0 - np.exp(-NTU)

    def _calculate_leakage_flow(self, p4, h4, s3, p_suc, gamma4):
        p_thr = p4 * (2.0 / (gamma4 + 1.0)) ** (gamma4 / (gamma4 - 1.0))
        p_thr = max(p_suc, p_thr)
        try:
            st = self.med_prop.calc_state("PS", p_thr, s3)
            dh = h4 - st.h
            return st.d * self.parameters["A_tot"] * np.sqrt(2.0 * dh) if dh > 0 else 0.0
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in leakage: {err}")
            return 0.0

    # =================================================================
    # DISCHARGE HEAT TRANSFER (James Eq. 17-18 approach)
    # =================================================================
    def _calculate_discharge_heat_transfer(self, m_dot_suc, T_w, h4, p_dis,
                                            oil_path):
        """
        James-style combined discharge heat transfer.

        After the discharge valve, the gas stream and the oil stream mix
        adiabatically in the discharge plenum. The combined stream then
        exchanges heat with the wall via NTU-eps. The discharge valve
        itself sees only gas (Variante A: oil mixes after valve).

        The gas mass flow includes both the suction refrigerant and the
        degassed KM from the oil path, ensuring mass consistency with R2/R3.
        The oil stream cp uses calc_cp_mix (oil-KM mixture at p_suc
        composition, consistent with Ansatz 1).

        Args:
            m_dot_suc: Suction refrigerant mass flow [kg/s]
            T_w: Wall temperature [K]
            h4: Gas enthalpy after compression [J/kg]
            p_dis: Discharge pressure [Pa]
            oil_path: dict from _calculate_oil_path, or None

        Returns:
            (h_dis_gas, T_dis, Q_dis_total)
        """
        lubricant = self._get_lubricant_model()

        try:
            state_5 = self.med_prop.calc_state("PH", p_dis, h4)
            T_gas = state_5.T
            cp_gas = self.med_prop.calc_transport_properties(state_5).cp
        except Exception:
            return h4, T_w, 0.0

        if oil_path is None:
            # Gas-only discharge HT (standard Molinaroli)
            _, eps = self._calculate_ntu_effectiveness(
                self.parameters["Ua_dis_ref"], m_dot_suc, cp_gas,
                self.parameters["m_dot_ref"])
            h_dis = h4 - eps * cp_gas * (T_gas - T_w)
            T_dis = T_gas - eps * (T_gas - T_w)
            Q_dis = m_dot_suc * (h4 - h_dis)
            return h_dis, T_dis, Q_dis

        # --- Adiabatic mixing of gas and oil in discharge plenum (after valve) ---
        # Gas mass flow includes degassed KM (mass-consistent with R2, R3)
        m_dot_gas = m_dot_suc + oil_path["m_dot_KM_degas_total"]
        m_dot_fl = oil_path["m_dot_fl"]
        T_oil = oil_path["T_oil_after"]
        # Oil-KM mixture cp (still at p_suc composition during Ansatz 1)
        p_suc = self.state_inlet.p
        cp_fl = lubricant.calc_cp_mix(T_oil, p_suc, oil_path["w_KM_after"])
        m_dot_total = m_dot_gas + m_dot_fl

        if m_dot_total < 1e-10:
            return h4, T_gas, 0.0

        # Mixing temperature
        T_mix = (m_dot_gas * cp_gas * T_gas + m_dot_fl * cp_fl * T_oil) / \
                (m_dot_gas * cp_gas + m_dot_fl * cp_fl)

        # Mass-weighted cp
        cp_mix = (m_dot_gas * cp_gas + m_dot_fl * cp_fl) / m_dot_total

        # --- NTU-epsilon with combined stream ---
        _, eps = self._calculate_ntu_effectiveness(
            self.parameters["Ua_dis_ref"], m_dot_total, cp_mix,
            self.parameters["m_dot_ref"])

        T_dis = T_mix - eps * (T_mix - T_w)
        Q_dis_total = m_dot_total * cp_mix * eps * (T_mix - T_w)

        # Gas-only discharge enthalpy at T_dis for state_outlet
        try:
            h_dis_gas = self.med_prop.calc_state("PT", p_dis, T_dis).h
        except Exception:
            h_dis_gas = h4 - eps * cp_gas * (T_gas - T_w)

        return h_dis_gas, T_dis, Q_dis_total

    # =================================================================
    # OIL PATH (suction side)
    # =================================================================
    def _calculate_oil_path(self, T_w, T_dis_exact, inputs, p_suc, p_dis):
        """
        Oil path with split degassing and enthalpy-balance Q_suc_oil.

        Returns dict with separate throttle and HT degassing, or None.
        """
        lubricant = self._get_lubricant_model()
        f = self.get_n_absolute(inputs.control.n)
        f_ref = self.parameters["f_ref"]

        # Step 1: Pure oil mass flow
        m_dot_oil = self.parameters["m_dot_oil_ref"] * (f / f_ref) ** 2

        # Step 2: Oil sump temperature
        T_amb = self._get_ambient_temperature(inputs)
        T_oil_sump = self._calculate_oil_sump_temperature(
            T_dis_exact, self.state_inlet.T, T_amb)

        # Step 3: Solubility at sump and after throttle
        w_KM_sump = lubricant.solve_w_KM(T_oil_sump, p_dis)
        if w_KM_sump is None:
            return None
        w_KM_suc = lubricant.solve_w_KM(T_oil_sump, p_suc)
        if w_KM_suc is None:
            return None

        # Step 4: Throttle degassing
        m_dot_KM_in_oil = m_dot_oil * w_KM_sump / (1.0 - w_KM_sump)
        m_dot_KM_suc = m_dot_oil * w_KM_suc / (1.0 - w_KM_suc)
        m_dot_KM_degas_thr = max(0.0, m_dot_KM_in_oil - m_dot_KM_suc)
        m_dot_fl_in = m_dot_oil / (1.0 - w_KM_suc)

        # h_degas_thr: superheated gas at p_suc, T_oil_sump
        try:
            h_degas_thr = self.med_prop.calc_state("PT", p_suc, T_oil_sump).h
        except Exception:
            h_degas_thr = self.med_prop.calc_state("PQ", p_suc, 1).h

        # Step 5: NTU-epsilon for T_oil_after
        cp_fl_in = lubricant.calc_cp_mix(T_oil_sump, p_suc, w_KM_suc)
        _, eps_oil = self._calculate_ntu_effectiveness(
            self.parameters["Ua_suc_oil_ref"], m_dot_fl_in, cp_fl_in,
            self.parameters["m_dot_oil_ref"])
        T_oil_after = T_oil_sump + eps_oil * (T_w - T_oil_sump)

        # Step 6: Solubility after HT (back-dissolution allowed)
        w_KM_after = lubricant.solve_w_KM(T_oil_after, p_suc)
        if w_KM_after is None:
            w_KM_after = w_KM_suc

        m_dot_KM_after = m_dot_oil * w_KM_after / (1.0 - w_KM_after)
        # Raw HT degassing: positive = extra degassing, negative = back-dissolution
        m_dot_KM_degas_ht_raw = m_dot_KM_suc - m_dot_KM_after
        # Effective HT degassing: back-dissolution limited to at most what the
        # throttle released. This ensures m_degas_total >= 0 without separate clamp
        # and provides a single consistent value for R3, R4, and Q_suc_oil.
        m_dot_KM_degas_ht_eff = max(-m_dot_KM_degas_thr, m_dot_KM_degas_ht_raw)
        m_dot_KM_degas_total = m_dot_KM_degas_thr + m_dot_KM_degas_ht_eff

        # Effective liquid composition: if the cap limits back-dissolution,
        # the liquid state must be consistent with the capped gas exchange.
        # m_KM_after_eff = what's actually dissolved after the capped exchange.
        m_dot_KM_after_eff = m_dot_KM_suc - m_dot_KM_degas_ht_eff
        if m_dot_KM_after_eff + m_dot_oil > 0:
            w_KM_after_eff = m_dot_KM_after_eff / (m_dot_oil + m_dot_KM_after_eff)
        else:
            w_KM_after_eff = 0.0

        # Use w_KM_after_eff for all liquid properties (consistent with capped gas)
        m_dot_fl_after = m_dot_oil / (1.0 - w_KM_after_eff) if w_KM_after_eff < 1.0 else m_dot_oil

        # h_degas_ht: gas at p_suc, T_oil_after
        try:
            h_degas_ht = self.med_prop.calc_state("PT", p_suc, T_oil_after).h
        except Exception:
            h_degas_ht = self.med_prop.calc_state("PQ", p_suc, 1).h

        # Step 7: Q_suc_oil via enthalpy balance (uses effective values throughout)
        h_fl_in = lubricant.calc_h_mix(T_oil_sump, p_suc, w_KM_suc)
        h_fl_after = lubricant.calc_h_mix(T_oil_after, p_suc, w_KM_after_eff)

        Q_suc_oil = (m_dot_fl_after * h_fl_after
                     + m_dot_KM_degas_ht_eff * h_degas_ht
                     - m_dot_fl_in * h_fl_in)

        # Step 8: Hydraulic oil recirculation loss
        rho_fl = lubricant.calc_rho_mix(T_oil_after, w_KM_after_eff)
        W_dot_oil_recirc = m_dot_fl_after * (p_dis - p_suc) / rho_fl if rho_fl > 0 else 0.0

        return {
            "m_dot_oil": m_dot_oil,
            "m_dot_fl_in": m_dot_fl_in,
            "m_dot_fl": m_dot_fl_after,
            "m_dot_KM_degas_thr": m_dot_KM_degas_thr,
            "m_dot_KM_degas_ht": m_dot_KM_degas_ht_eff,
            "m_dot_KM_degas_ht_raw": m_dot_KM_degas_ht_raw,
            "m_dot_KM_degas_total": m_dot_KM_degas_total,
            "h_degas_thr": h_degas_thr,
            "h_degas_ht": h_degas_ht,
            "h_fl_after": h_fl_after,
            "rho_fl": rho_fl,
            "Q_suc_oil": Q_suc_oil,
            "W_dot_oil_recirc": W_dot_oil_recirc,
            "T_oil_sump": T_oil_sump,
            "T_oil_after": T_oil_after,
            "w_KM_sump": w_KM_sump,
            "w_KM_suc": w_KM_suc,
            "w_KM_after": w_KM_after_eff,
            "w_KM_after_raw": w_KM_after,
            "epsilon_oil_suc": eps_oil,
        }

    # =================================================================
    # DIAGNOSTIC: Outlet separation
    # =================================================================
    def _calculate_outlet_separation(self, T_dis, p_dis, oil_path, m_dot_suc):
        """
        DIAGNOSTIC post-processing only. Does NOT feed into residuals.
        """
        lubricant = self._get_lubricant_model()
        m_dot_oil = oil_path["m_dot_oil"]

        w_KM_dis = lubricant.solve_w_KM(T_dis, p_dis)
        if w_KM_dis is None:
            return None

        m_dot_oil_KM_dis = m_dot_oil / (1.0 - w_KM_dis)
        m_dot_dis_total = (m_dot_suc + oil_path["m_dot_fl"]
                           + oil_path["m_dot_KM_degas_total"])
        m_dot_KM_gas = m_dot_dis_total - m_dot_oil_KM_dis
        if m_dot_KM_gas <= 0:
            return None

        h_dis = self.state_outlet.h
        h_oil_KM_dis = lubricant.calc_h_mix(T_dis, p_dis, w_KM_dis)
        h_KM_gas = (m_dot_dis_total * h_dis
                    - m_dot_oil_KM_dis * h_oil_KM_dis) / m_dot_KM_gas
        try:
            T_KM_gas = self.med_prop.calc_state("PH", p_dis, h_KM_gas).T
        except Exception:
            T_KM_gas = T_dis

        return {"m_dot_KM_gas": m_dot_KM_gas, "T_KM_gas": T_KM_gas,
                "w_KM_dis": w_KM_dis, "h_KM_gas": h_KM_gas}

    # =================================================================
    # Loss power
    # =================================================================
    def _calculate_loss_power(self, h4, h3, T_dis_exact, p_dis, inputs):
        f = self.get_n_absolute(inputs.control.n)
        f_ref = self.parameters["f_ref"]
        rho3 = self.state_c_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f
        W_dot_int = m_dot_3 * (h4 - h3)

        T_amb = self._get_ambient_temperature(inputs)
        T_oil_sump = self._calculate_oil_sump_temperature(
            T_dis_exact, self.state_inlet.T, T_amb)
        mu_oil = self._calculate_oil_viscosity(T_oil_sump, p_dis)
        mu_mix_eff = self._milli_pas_to_pas(mu_oil)

        W_loss_load = float(self.parameters["alpha_loss"]) * W_dot_int
        W_loss_ref = float(self.parameters.get("W_dot_loss_ref", 0.0)) * (f / f_ref) ** 2
        W_loss_fric = (float(self.parameters.get("alpha_fric_tot", 0.0))
                       * mu_mix_eff * self.V_h * (2.0 * np.pi * f) ** 2)

        return {
            "W_dot_int": W_dot_int,
            "W_dot_loss": W_loss_load + W_loss_ref + W_loss_fric,
            "W_dot_loss_load": W_loss_load,
            "W_dot_loss_ref_term": W_loss_ref,
            "W_dot_loss_fric": W_loss_fric,
            "T_oil_sump": T_oil_sump,
            "mu_oil": mu_oil,
            "mu_mix_eff": mu_mix_eff,
        }

    # =================================================================
    # RESIDUALS
    # =================================================================
    def _get_initial_guesses(self, inputs, p_outlet):
        h_suc = self.state_inlet.h
        f = self.get_n_absolute(inputs.control.n)
        st_is = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s)
        m0 = self.state_inlet.d * self.parameters["V_IC"] * f
        Tw0 = 0.5 * (self.state_inlet.T + st_is.T)
        cp_suc = self.med_prop.calc_transport_properties(self.state_inlet).cp
        h1_0 = h_suc + 0.5 * cp_suc * (Tw0 - self.state_inlet.T)
        return [m0, Tw0, h1_0, 1.1 * p_outlet, h1_0]

    def _clear_cache(self):
        self._state_cache.clear()
        self._oil_viscosity_cache.clear()
        self._cached_discharge_valve.clear()
        self._current_oil_path = None
        self._dis_ht_result = None
        self._T_dis_est = None
        self._T_dis_corr = None
        self._last_x = None
        self._cache_hits = 0
        self._cache_misses = 0
        self._corrector_fallback_count = 0
        self._solver_fallback_count = 0
        self._gamma4_min = None
        self._gamma4_max = None
        self._gamma4_n = 0

    def _get_cache_key(self, p_suc, h1, h3, p4, s3):
        return (round(p_suc, 2), round(h1, 1), round(h3, 1),
                round(p4, 2), round(s3, 4))

    def _calculate_residuals(self, x, inputs, p_outlet):
        m_dot_suc, T_w, h1, p4, h3 = x
        p_suc = self.state_inlet.p
        p_dis = p_outlet

        # --- State 3 ---
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        rho3 = self.state_c_3.d

        # --- State cache ---
        ck = self._get_cache_key(p_suc, h1, h3, p4, s3)
        if (self._last_x is not None
                and np.allclose(x, self._last_x, rtol=1e-6, atol=1e-8)
                and ck in self._state_cache):
            c = self._state_cache[ck]
            self.state_c_1, self.state_c_4 = c["s1"], c["s4"]
            h4, gamma4, m_dot_tot = c["h4"], c["g4"], c["mt"]
            self._cache_hits += 1
        else:
            self._cache_misses += 1
            self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)
            self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
            h4 = self.state_c_4.h
            tp4 = self.med_prop.calc_transport_properties(self.state_c_4)
            gamma4 = tp4.cp / tp4.cv
            m_dot_tot = self._calculate_leakage_flow(p4, h4, s3, p_suc, gamma4)
            self._state_cache[ck] = {"s1": self.state_c_1, "s4": self.state_c_4,
                                     "h4": h4, "g4": gamma4, "mt": m_dot_tot}
            if len(self._state_cache) > 1000:
                del self._state_cache[next(iter(self._state_cache))]
        self._last_x = x.copy()

        if self.debug_enabled:
            if self._gamma4_min is None:
                self._gamma4_min = self._gamma4_max = gamma4
            else:
                self._gamma4_min = min(self._gamma4_min, gamma4)
                self._gamma4_max = max(self._gamma4_max, gamma4)
            self._gamma4_n += 1

        # --- Gas-only discharge HT for T_dis estimate (predictor) ---
        h_dis_est, _, _ = self._calculate_discharge_heat_transfer(
            m_dot_suc, T_w, h4, p_dis, oil_path=None)
        try:
            T_dis_est = self.med_prop.calc_state("PH", p_dis, h_dis_est).T
        except Exception:
            T_dis_est = self.state_c_4.T

        # --- Oil path (predictor: uses gas-only T_dis) ---
        oil_pred = self._calculate_oil_path(
            T_w, T_dis_est, inputs, p_suc, p_dis)

        if oil_pred is None:
            return np.full(5, 1e6)

        # --- James-style combined discharge HT (with predictor oil path) ---
        _, T_dis_final, _ = self._calculate_discharge_heat_transfer(
            m_dot_suc, T_w, h4, p_dis, oil_path=oil_pred)

        # --- Oil path (corrector: uses combined T_dis_final) ---
        self._current_oil_path = self._calculate_oil_path(
            T_w, T_dis_final, inputs, p_suc, p_dis)

        if self._current_oil_path is None:
            return np.full(5, 1e6)

        oil = self._current_oil_path

        # --- Final James-style combined discharge HT (with corrected oil path) ---
        h_dis_final, T_dis_final, Q_dis_total = self._calculate_discharge_heat_transfer(
            m_dot_suc, T_w, h4, p_dis, oil_path=oil)
        self._dis_ht_result = (h_dis_final, T_dis_final, Q_dis_total)

        # --- Loss power ---
        self._loss = self._calculate_loss_power(h4, h3, T_dis_final, p_dis, inputs)

        # --- 5 Residuals ---
        r = np.zeros(5)
        r[0] = self._residual_suction_ht(m_dot_suc, T_w, h1)
        r[1] = self._residual_discharge_valve(m_dot_suc, p4, h4, s3, p_dis, gamma4)
        r[2] = self._residual_compressor_flow(m_dot_suc, rho3, m_dot_tot, inputs)
        r[3] = self._residual_mixing_energy(m_dot_suc, h1, h3, h4, m_dot_tot)
        r[4] = self._residual_wall_energy(m_dot_suc, T_w, h1, h4, inputs)
        return r

    # R1: Suction heat transfer (unchanged)
    def _residual_suction_ht(self, m_dot_suc, T_w, h1):
        h_suc = self.state_inlet.h
        T_suc = self.state_inlet.T
        cp_suc = self.med_prop.calc_transport_properties(self.state_inlet).cp
        _, eps = self._calculate_ntu_effectiveness(
            self.parameters["Ua_suc_ref"], m_dot_suc, cp_suc,
            self.parameters["m_dot_ref"])
        return h1 - h_suc - eps * cp_suc * (T_w - T_suc)

    # R2: Discharge valve flow (gas only — oil mixes after valve)
    # m_gas = m_suc + m_KM_degas_total = m_valve(p4, ...)
    # The valve sees only the gas phase; oil is added downstream
    # in the discharge plenum.
    def _residual_discharge_valve(self, m_dot_suc, p4, h4, s3, p_dis, gamma4):
        oil = self._current_oil_path
        m_dot_gas = m_dot_suc + oil["m_dot_KM_degas_total"]

        ck = (round(p4, 1), round(h4, 1), round(s3, 4),
              round(p_dis, 1), round(gamma4, 3))
        if ck in self._cached_discharge_valve:
            mv = self._cached_discharge_valve[ck]
        else:
            p_thr = p4 * (2.0 / (gamma4 + 1.0)) ** (gamma4 / (gamma4 - 1.0))
            p_thr = max(p_dis, p_thr)
            try:
                st = self.med_prop.calc_state("PS", p_thr, s3)
                dh = h4 - st.h
                mv = st.d * self.parameters["A_dis"] * np.sqrt(2.0 * dh) if dh > 0 else 0.0
                if len(self._cached_discharge_valve) > 500:
                    del self._cached_discharge_valve[next(iter(self._cached_discharge_valve))]
                self._cached_discharge_valve[ck] = mv
            except Exception:
                mv = 1e-6
        return m_dot_gas - mv

    # R3: Compressor flow (Molinaroli form, no V_oil)
    def _residual_compressor_flow(self, m_dot_suc, rho3, m_dot_tot, inputs):
        f = self.get_n_absolute(inputs.control.n)
        m_dot_3 = rho3 * self.parameters["V_IC"] * f
        oil = self._current_oil_path
        return m_dot_3 - (m_dot_suc + m_dot_tot + oil["m_dot_KM_degas_total"])

    # R4: Mixing energy with SPLIT degassing
    # m_degas_ht is the effective value (back-dissolution limited to throttle
    # release), so thr + ht >= 0 by construction — no conditional needed.
    def _residual_mixing_energy(self, m_dot_suc, h1, h3, h4, m_dot_tot):
        oil = self._current_oil_path

        m_degas_thr = oil["m_dot_KM_degas_thr"]
        m_degas_ht = oil["m_dot_KM_degas_ht"]
        m_degas_total = oil["m_dot_KM_degas_total"]

        m_3_gas = m_dot_suc + m_dot_tot + m_degas_total

        e_degas = m_degas_thr * oil["h_degas_thr"] + m_degas_ht * oil["h_degas_ht"]

        e_in = m_dot_suc * h1 + m_dot_tot * h4 + e_degas
        residual = m_3_gas * h3 - e_in
        if abs(m_3_gas) > 1e-10:
            residual /= m_3_gas
        return residual

    # R5: Wall energy balance
    def _residual_wall_energy(self, m_dot_suc, T_w, h1, h4, inputs):
        h_suc = self.state_inlet.h
        T_amb = self._get_ambient_temperature(inputs)

        loss = self._loss
        oil = self._current_oil_path

        Q_suc = m_dot_suc * (h1 - h_suc)
        Q_dis_total = self._dis_ht_result[2]
        Q_amb = np.sign(T_w - T_amb) * self.parameters["Ua_amb"] * abs(T_w - T_amb) ** 1.25
        Q_suc_oil = oil["Q_suc_oil"]
        W_recirc = oil["W_dot_oil_recirc"]

        residual = (loss["W_dot_loss"] + W_recirc) + Q_dis_total - Q_amb - Q_suc - Q_suc_oil
        if abs(loss["W_dot_int"]) > 1e-10:
            residual /= loss["W_dot_int"]
        return residual

    # =================================================================
    # SOLVER
    # =================================================================
    def simulate_operating_point(self, inputs, p_outlet, fs_state):
        self._clear_cache()
        x0 = self._get_initial_guesses(inputs, p_outlet)
        bounds = [
            (1e-6, 1.0),
            (self.state_inlet.T, 400.0),
            (self.state_inlet.h, self.state_inlet.h + 100e6),
            (p_outlet * 1.001, p_outlet * 15.0),
            (self.state_inlet.h, self.state_inlet.h + 200e6),
        ]
        max_nfev = int(getattr(inputs, "lsq_max_nfev", 20000))
        ftol = float(getattr(inputs, "lsq_ftol", 1e-8))
        xtol = float(getattr(inputs, "lsq_xtol", 1e-8))

        result = least_squares(
            lambda x: self._calculate_residuals(x, inputs, p_outlet),
            x0, bounds=([b[0] for b in bounds], [b[1] for b in bounds]),
            method="trf", ftol=ftol, xtol=xtol, max_nfev=max_nfev)
        if not result.success:
            print(f"  Solver warning: {result.message}")
            print("Trying to use solution anyway...")

        self._calculate_final_states(result.x, inputs, p_outlet, fs_state)
        return result.x

    # =================================================================
    # FINAL STATES
    # =================================================================
    def _calculate_final_states(self, solution, inputs, p_outlet, fs_state):
        m_dot_suc, T_w, h1, p4, h3 = solution
        p_suc = self.state_inlet.p
        p_dis = p_outlet

        self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
        h4 = self.state_c_4.h

        # Predictor: gas-only T_dis estimate
        h_dis_est, _, _ = self._calculate_discharge_heat_transfer(
            m_dot_suc, T_w, h4, p_dis, oil_path=None)
        try:
            T_dis_est = self.med_prop.calc_state("PH", p_dis, h_dis_est).T
        except Exception:
            T_dis_est = self.state_c_4.T

        oil_pred = self._calculate_oil_path(T_w, T_dis_est, inputs, p_suc, p_dis)

        # Corrector: combined T_dis -> recalculate oil path
        T_dis_corr = T_dis_est  # default if corrector is skipped
        if oil_pred is not None:
            _, T_dis_corr, _ = self._calculate_discharge_heat_transfer(
                m_dot_suc, T_w, h4, p_dis, oil_path=oil_pred)
            oil = self._calculate_oil_path(T_w, T_dis_corr, inputs, p_suc, p_dis)
            if oil is None:
                if ENABLE_TIMING:
                    print("  Warning: corrector oil path failed, using predictor result")
                self._corrector_fallback_count += 1
                oil = oil_pred
        else:
            oil = None

        # Fallback: if both predictor and corrector failed, use the last
        # valid oil path from the solver's converged residual evaluation.
        if oil is None:
            if self._current_oil_path is not None:
                print("  Warning: final-state oil path failed, using last solver result")
                self._solver_fallback_count += 1
                oil = self._current_oil_path
            else:
                raise RuntimeError(
                    "Oil path failed in _calculate_final_states and no valid "
                    "solver result available. Operating point is invalid."
                )

        # Store predictor/corrector T_dis for diagnostics
        self._T_dis_est = T_dis_est
        self._T_dis_corr = T_dis_corr

        # Final James-style combined discharge HT
        h_dis_final, T_dis, Q_dis_total = self._calculate_discharge_heat_transfer(
            m_dot_suc, T_w, h4, p_dis, oil_path=oil)

        self.state_c_5 = self.med_prop.calc_state("PH", p_dis, h_dis_final)
        # state_outlet is a PURE REFRIGERANT equivalent state at (p_dis, T_dis).
        # T_dis comes from the combined gas/oil discharge HT model, so it
        # represents the mixed stream equilibrium temperature. The enthalpy
        # h_dis_final is the gas-phase enthalpy at this temperature.
        # This is NOT a true two-phase mixture outlet state — it is the
        # gaseous refrigerant state that is thermally consistent with the
        # combined discharge process. eta_is and T_dis from this state are
        # therefore approximate when the oil mass flow is significant.
        self.state_outlet = self.state_c_5

        # Loss power
        loss = self._calculate_loss_power(h4, h3, T_dis, p_dis, inputs)
        self.T_oil_sump = loss["T_oil_sump"]
        self.mu_oil = loss["mu_oil"]
        self.mu_mix_eff = loss["mu_mix_eff"]
        self.W_dot_int = loss["W_dot_int"]
        self.W_dot_loss = loss["W_dot_loss"]
        self.W_dot_loss_load = loss["W_dot_loss_load"]
        self.W_dot_loss_ref_term = loss["W_dot_loss_ref_term"]
        self.W_dot_loss_fric = loss["W_dot_loss_fric"]

        # Oil is guaranteed non-None here (RuntimeError raised otherwise)
        W_recirc = oil["W_dot_oil_recirc"]
        self.P_el = self.W_dot_int + self.W_dot_loss + W_recirc
        self.W_dot_comp = self.P_el
        # m_flow is the EXTERNAL suction refrigerant mass flow rate — the quantity
        # reported in manufacturer catalogues and relevant for cycle simulation.
        # It does NOT include the degassed KM from the oil path, which is an
        # internal recirculation stream that returns to the oil after separation.
        self.m_flow = m_dot_suc
        self.T_w = T_w

        # Oil path results
        self.m_dot_oil = oil["m_dot_oil"]
        self.Q_dot_suc_oil = oil["Q_suc_oil"]
        self.W_dot_oil_recirc = W_recirc
        self.m_dot_KM_degas_thr = oil["m_dot_KM_degas_thr"]
        self.m_dot_KM_degas_ht = oil["m_dot_KM_degas_ht"]
        self.m_dot_KM_degas_total = oil["m_dot_KM_degas_total"]
        self.m_dot_fl = oil["m_dot_fl"]
        self.w_KM_sump = oil["w_KM_sump"]
        self.w_KM_after = oil["w_KM_after"]
        self.T_oil_after = oil["T_oil_after"]

        # Diagnostic quantities
        self.m_dot_KM_degas_ht_raw = oil["m_dot_KM_degas_ht_raw"]
        self.w_KM_after_raw = oil["w_KM_after_raw"]
        self.Q_dis_total = Q_dis_total
        self.T_dis_est = self._T_dis_est
        self.T_dis_corr = self._T_dis_corr

        # Gas mass flow through discharge valve and discharge HT
        # = m_suc + m_KM_degas_total (without internal leakage recirculation m_tot)
        self.m_dot_gas_discharge = m_dot_suc + self.m_dot_KM_degas_total

        # Diagnostic outlet separation (approximate — see class docstring)
        sep = self._calculate_outlet_separation(T_dis, p_dis, oil, m_dot_suc)
        self.m_dot_KM_gas = sep["m_dot_KM_gas"] if sep else m_dot_suc
        self.T_KM_gas = sep["T_KM_gas"] if sep else T_dis

        # Flowsheet state
        fs_state.set("m_flow", self.m_flow, "kg/s", "External suction refrigerant mass flow rate")
        fs_state.set("m_dot_gas_discharge", self.m_dot_gas_discharge, "kg/s", "Gas flow through discharge valve (m_suc + m_degas, excl. leakage)")
        fs_state.set("P_el", self.P_el, "W", "Electrical power input")
        fs_state.set("W_dot_int", self.W_dot_int, "W", "Internal compression power")
        fs_state.set("W_dot_loss", self.W_dot_loss, "W", "Compressor loss power")
        fs_state.set("W_dot_loss_load", self.W_dot_loss_load, "W", "Load-dependent loss")
        fs_state.set("W_dot_loss_ref_term", self.W_dot_loss_ref_term, "W", "Speed-dependent loss")
        fs_state.set("W_dot_loss_fric", self.W_dot_loss_fric, "W", "Viscous friction loss")
        fs_state.set("W_dot_oil_recirc", self.W_dot_oil_recirc, "W", "Hydraulic oil recirc. loss")
        fs_state.set("T_wall", T_w, "K", "Wall temperature")
        fs_state.set("T_dis", T_dis, "K", "Discharge temperature")
        fs_state.set("T_oil_sump", self.T_oil_sump, "K", "Oil sump temperature")
        fs_state.set("mu_oil", self.mu_oil, "mPa*s", "Dynamic viscosity")
        fs_state.set("mu_mix_eff", self.mu_mix_eff, "Pa*s", "Effective mixture viscosity")
        fs_state.set("pc4", p4, "Pa", "Internal discharge pressure")
        fs_state.set("hc1", h1, "J/kg", "Enthalpy after suction HT")
        fs_state.set("hc3", h3, "J/kg", "Enthalpy after mixing")
        fs_state.set("hc4", h4, "J/kg", "Enthalpy after compression")
        fs_state.set("p_2", self.state_outlet.p, "Pa", "Outlet pressure")
        fs_state.set("p_1", self.state_inlet.p, "Pa", "Inlet pressure")
        fs_state.set("T_1", self.state_inlet.T, "K", "Inlet temperature")
        fs_state.set("m_dot_oil", self.m_dot_oil, "kg/s", "Pure oil mass flow")
        fs_state.set("Q_suc_oil", self.Q_dot_suc_oil, "W", "Oil suction HT")
        fs_state.set("m_dot_KM_degas_thr", self.m_dot_KM_degas_thr, "kg/s", "Degassed KM (throttle)")
        fs_state.set("m_dot_KM_degas_ht", self.m_dot_KM_degas_ht, "kg/s", "Degassed KM (HT)")
        fs_state.set("m_dot_KM_degas_total", self.m_dot_KM_degas_total, "kg/s", "Total net degassed KM")
        fs_state.set("m_dot_fl", self.m_dot_fl, "kg/s", "Liquid oil stream")
        fs_state.set("w_KM_sump", self.w_KM_sump, "-", "KM fraction at sump")
        fs_state.set("w_KM_after", self.w_KM_after, "-", "KM fraction after HT (effective)")
        fs_state.set("T_oil_after", self.T_oil_after, "K", "Oil T after suction HT")
        fs_state.set("m_dot_KM_gas", self.m_dot_KM_gas, "kg/s", "Gaseous KM at outlet (diag.)")
        fs_state.set("T_KM_gas", self.T_KM_gas, "K", "Gaseous KM T at outlet (diag.)")
        # Diagnostic: predictor-corrector and back-dissolution details
        fs_state.set("w_KM_after_raw", self.w_KM_after_raw, "-", "KM fraction after HT raw (diag.)")
        fs_state.set("m_dot_KM_degas_ht_raw", self.m_dot_KM_degas_ht_raw, "kg/s", "Raw HT degassing before limiting (diag.)")
        fs_state.set("Q_dis_total", self.Q_dis_total, "W", "Total discharge HT gas+oil (diag.)")
        fs_state.set("T_dis_est", self.T_dis_est, "K", "Predictor T_dis gas-only (diag.)")
        fs_state.set("T_dis_corr", self.T_dis_corr, "K", "Corrector T_dis combined (diag.)")

    # =================================================================
    # INTERFACE (no clamps, NaN on errors)
    # =================================================================
    def get_eta_mech(self, inputs):
        if self.W_dot_comp is None or self.W_dot_comp <= 0.0:
            return float("nan")
        if (self.state_c_3 is None or self.state_c_4 is None
                or self.state_c_3.d is None or self.state_c_4.h is None
                or self.state_c_3.h is None):
            return float("nan")
        f = self.get_n_absolute(inputs.control.n)
        W_int = (self.state_c_3.d * self.parameters["V_IC"] * f
                 * (self.state_c_4.h - self.state_c_3.h))
        if W_int <= 0.0:
            return float("nan")
        return W_int / self.W_dot_comp

    def get_lambda_h(self, inputs):
        if (self.m_flow is None or self.state_inlet is None
                or self.state_inlet.d is None or self.m_flow <= 0.0):
            return float("nan")
        f = self.get_n_absolute(inputs.control.n)
        m_th = self.state_inlet.d * self.V_h * f
        if m_th <= 0.0:
            return float("nan")
        return self.m_flow / m_th

    def get_eta_isentropic(self, p_outlet, inputs, fs_state):
        if (self.state_inlet.T != fs_state.T_1
                or p_outlet != fs_state.p_2
                or self.state_inlet.p != fs_state.p_1):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        try:
            h_suc = self.state_inlet.h
            h_dis = self.state_outlet.h
            if h_dis <= h_suc:
                return float("nan")
            h_is = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s).h
            if h_is <= h_suc:
                return float("nan")
            w_act = h_dis - h_suc
            if w_act <= 0.0:
                return float("nan")
            return (h_is - h_suc) / w_act
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in eta_is: {err}")
            return float("nan")

    def calc_state_outlet(self, p_outlet, inputs, fs_state):
        self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

    def calc_m_flow(self, p_outlet, inputs, fs_state):
        if (self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2
                or self.state_inlet.p != fs_state.p_1):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        return self.m_flow

    def calc_electrical_power(self, p_outlet, inputs, fs_state):
        if (self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2
                or self.state_inlet.p != fs_state.p_1):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        return self.P_el

    def get_debug_report(self):
        if not self.debug_enabled or self._gamma4_n == 0:
            return "Debug disabled."
        return (
            f"Oil Path v5 Debug Report\n"
            f"  gamma4: {self._gamma4_min:.6f}..{self._gamma4_max:.6f} (N={self._gamma4_n})\n"
            f"  state_cache: hits={self._cache_hits}, misses={self._cache_misses}\n"
            f"  dis_valve_cache: {len(self._cached_discharge_valve)}\n"
            f"  visc_cache: {len(self._oil_viscosity_cache)}\n"
            f"  corrector_fallbacks: {self._corrector_fallback_count}\n"
            f"  solver_fallbacks: {self._solver_fallback_count}")
