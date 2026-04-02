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
    - ISENTHALPIC throttling from p_dis to p_suc causes partial degassing
      and cooling (T_throttle < T_oil_sump due to evaporation enthalpy).
    - The COMBINED liquid + gas stream exchanges heat with the fictitious
      wall via NTU-epsilon. Both liquid and degassed gas participate in the
      heat transfer. Back-dissolution is permitted.
    - Additional degassing/dissolution at T_oil_after is tracked separately.
    - Residuum 4 uses a UNIFIED degassing enthalpy: all degassed gas enters
      the cylinder at T_oil_after (after participating in suction HT).
    - Q_suc_oil is computed via full enthalpy balance across the HT zone.

Discharge side — 3-stage chain:
    Stage 1: Adiabatic mixing + back-dissolution
        After the discharge valve, the gas stream (m_suc + m_KM_degas_total)
        and the oil stream (at T_oil_after, w_KM_after_eff) mix in the
        discharge plenum at p_dis. The oil is far below its equilibrium
        solubility and absorbs KM from the gas phase. T_mix and w_KM_mix
        are found simultaneously via brentq (enthalpy conservation +
        solubility equilibrium). The discharge valve sees only gas
        (Variante A).
    Stage 2: NTU-epsilon discharge HT
        The combined stream (with updated gas/liquid fractions from Stage 1)
        exchanges heat with the wall. Uses Ua_dis_ref.
    Stage 3: Post-HT back-dissolution
        After cooling to T_dis, the equilibrium shifts (higher solubility
        at lower T). Additional KM dissolves; the released heat Q_dissolve_3
        goes to the wall (R5).

Oil sump energy balance:
    The oil returns at (T_dis, p_dis, w_KM_dis) and must reach
    (T_sump, p_dis, w_KM_sump). During cooling, KM from the gas phase
    in the housing dissolves into the oil. The rejected heat Q_oil_sump
    is included in R5. Uses correct compositions at both endpoints.

All mass and energy balances close exactly:
    - KM degas (suction) = KM dissolution (Stage 1 + Stage 3 + sump)
    - External: m_flow_in = m_flow_out = m_suc

Residuum 3 follows the James/Molinaroli convention: the oil does NOT reduce
the effective gas volume in the cylinder.

Hydraulic oil recirculation loss W_dot_oil_recirc is included as a parasitic
loss (not thermodynamic compression work). Uses m_dot_fl and rho_fl consistently.

When the oil path calculation fails (e.g. solubility solver), the residuals
return large penalty values instead of silently falling back to a no-oil model.

A predictor-corrector scheme is used for T_dis: first a gas-only discharge
HT provides T_dis_est, then the oil path is computed, then a combined
discharge HT gives T_dis_final, and the oil path is recomputed with this
corrected T_dis. Convergence is measured as |T_dis_final - T_dis_corr|.

m_flow is the EXTERNAL suction refrigerant mass flow rate (catalogue value).
m_dot_gas_discharge = m_suc + m_KM_degas_total is the gas flow through the
discharge valve (excludes internal leakage recirculation).

Fitting parameters:
    Molinaroli base:  Ua_suc_ref, Ua_dis_ref, Ua_amb, A_tot, A_dis,
                      V_IC, alpha_loss, W_dot_loss_ref
    Viscosity ext.:   alpha_fric_tot
    Oil path:         m_dot_oil_ref, Ua_suc_oil_ref

References:
    Molinaroli et al. (2017), doi:10.1016/j.ijrefrig.2017.04.015
    James et al. (2016), doi:10.1016/j.ijrefrig.2015.12.011
"""
from scipy.optimize import least_squares, brentq
import os
import numpy as np

from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs, FlowsheetState
from vclibpy.media import ThermodynamicState

from vclibpy.media.lubricant_fitting_shared_refprop import LubricantFitting as SharedLubricantFitting


ENABLE_TIMING = True


class Molinaroli_2017_Compressor_Oil_Path(Compressor):

    _LUBRICANT_MODEL_CACHE = {}

    @classmethod
    def _build_lubricant_model(cls, fluid_name, lub_name, shared_refprop=None):
        if shared_refprop is not None:
            try:
                model = SharedLubricantFitting(fluid_name=fluid_name, lub_name=lub_name, shared_refprop=shared_refprop)
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
        key = (os.getpid(), str(fluid_name).strip().lower(), str(lub_name).strip().lower(),
               id(shared_refprop) if shared_refprop is not None else None)
        if key not in cls._LUBRICANT_MODEL_CACHE:
            cls._LUBRICANT_MODEL_CACHE[key] = cls._build_lubricant_model(fluid_name, lub_name, shared_refprop)
        return cls._LUBRICANT_MODEL_CACHE[key]

    def _get_lubricant_model(self):
        shared_refprop = getattr(self, "med_prop", None)
        model = self._get_cached_lubricant_model(self.fluid_name, self.lub_name, shared_refprop)
        if shared_refprop is not None and hasattr(model, "refrigerant_prop"):
            model.refrigerant_prop = shared_refprop
        self.lubricant_model = model
        return model

    def __init__(self, N_max, V_h, fluid_name="propane", lub_name="LPG 68", parameters=None):
        super().__init__(N_max=N_max, V_h=V_h)
        if parameters is None:
            parameters = {"Ua_suc_ref": 16.05, "Ua_dis_ref": 13.96, "Ua_amb": 0.36,
                          "A_tot": 9.47e-9, "A_dis": 86.1e-6, "V_IC": 16.11e-6,
                          "alpha_loss": 0.16, "W_dot_loss_ref": 83.0, "alpha_fric_tot": 0.0,
                          "m_dot_ref": 0.0083, "f_ref": 50.0, "mu_fallback": 5.0,
                          "m_dot_oil_ref": 0.001, "Ua_suc_oil_ref": 5.0}
        self.parameters = dict(parameters)
        self.fluid_name = fluid_name
        self.lub_name = lub_name
        self.lubricant_model = None
        self.state_c_1 = self.state_c_3 = self.state_c_4 = self.state_c_5 = None
        self.T_w = self.state_outlet = self.W_dot_comp = self.P_el = None
        self.T_oil_sump = self.mu_oil = self.mu_mix_eff = None
        self.W_dot_int = self.W_dot_loss = None
        self.W_dot_loss_load = self.W_dot_loss_ref_term = self.W_dot_loss_fric = None
        self.m_dot_oil = self.Q_dot_suc_oil = self.W_dot_oil_recirc = None
        self.Q_oil_sump = self.Q_dissolve_3 = None
        self.m_dot_KM_degas_thr = self.m_dot_KM_degas_ht = self.m_dot_KM_degas_total = None
        self.m_dot_fl = self.w_KM_sump = self.w_KM_after = None
        self.T_oil_after = self.T_throttle = None
        self.m_dot_KM_gas = self.m_dot_gas_discharge = None
        self.m_dot_KM_degas_ht_raw = self.w_KM_after_raw = None
        self.w_KM_mix = self.w_KM_dis = None
        self.Q_dis_total = self.T_dis_est = self.T_dis_corr = None
        self.pc_convergence_gap = None
        self._current_oil_path = self._dis_ht_result = None
        self._T_dis_est = self._T_dis_corr = self._T_dis_final = None
        self._state_cache = {}
        self._last_x = None
        self._cache_hits = self._cache_misses = 0
        self._cached_discharge_valve = {}
        self._oil_viscosity_cache = {}
        self.debug_enabled = False
        self._gamma4_min = self._gamma4_max = None
        self._gamma4_n = 0
        self._corrector_fallback_count = self._solver_fallback_count = 0
        self._w_KM_after_fallback_count = self._throttle_fallback_count = 0
        self._stage1_fallback_count = 0

    # ----- helpers -----
    @staticmethod
    def _kelvin_to_celsius(T_K): return T_K - 273.15
    @staticmethod
    def _celsius_to_kelvin(T_C): return T_C + 273.15
    @staticmethod
    def _milli_pas_to_pas(mu_mpas): return float(mu_mpas) * 1e-3

    def _get_ambient_temperature(self, inputs):
        return float(getattr(inputs, "T_amb", 25.0 + 273.15))

    def _calculate_oil_sump_temperature(self, T_dis_K, T_in_K, T_amb_K):
        return self._celsius_to_kelvin(
            0.914227 * self._kelvin_to_celsius(T_dis_K)
            + 0.008136 * self._kelvin_to_celsius(T_in_K)
            + 0.006144 * self._kelvin_to_celsius(T_amb_K))

    def _calculate_oil_viscosity(self, T_oil_K, p_oil):
        ck = (round(T_oil_K, 3), round(p_oil, 1))
        if ck in self._oil_viscosity_cache:
            return self._oil_viscosity_cache[ck]
        lubricant = self._get_lubricant_model()
        try:
            tp = lubricant.calc_transport_properties(state=ThermodynamicState(p=p_oil, T=T_oil_K), phase="liquid")
        except Exception:
            tp = None
        mu_fb = float(self.parameters.get("mu_fallback", 5.0))
        mu = mu_fb
        if tp is not None and tp.dyn_vis is not None:
            mu = float(tp.dyn_vis)
            if not np.isfinite(mu) or mu <= 0.0:
                mu = mu_fb
        self._oil_viscosity_cache[ck] = mu
        if len(self._oil_viscosity_cache) > 1000:
            del self._oil_viscosity_cache[next(iter(self._oil_viscosity_cache))]
        return mu

    def _calculate_ntu_effectiveness(self, Ua_ref, m_dot, cp, m_dot_ref):
        if m_dot < 1e-10 or cp < 1e-10:
            return 0.0, 0.0
        NTU = (Ua_ref / (m_dot * cp)) * (m_dot / m_dot_ref) ** 0.8
        return min(NTU, 100.0), 1.0 - np.exp(-min(NTU, 100.0))

    def _calculate_leakage_flow(self, p4, h4, s3, p_suc, gamma4):
        p_thr = max(p_suc, p4 * (2.0 / (gamma4 + 1.0)) ** (gamma4 / (gamma4 - 1.0)))
        try:
            st = self.med_prop.calc_state("PS", p_thr, s3)
            dh = h4 - st.h
            return st.d * self.parameters["A_tot"] * np.sqrt(2.0 * dh) if dh > 0 else 0.0
        except Exception:
            return 0.0

    # =================================================================
    # DISCHARGE HEAT TRANSFER — 3-stage chain
    # =================================================================
    def _calculate_discharge_heat_transfer(self, m_dot_suc, T_w, h4, p_dis, oil_path):
        """3-stage discharge HT. Returns dict."""
        lubricant = self._get_lubricant_model()
        _zero = {"h_dis_gas": h4, "T_dis": T_w, "Q_dis_total": 0.0,
                 "Q_dissolve_3": 0.0, "Q_oil_sump": 0.0,
                 "w_KM_mix": None, "w_KM_dis": None, "m_dot_gas_exit": m_dot_suc}
        try:
            state_5 = self.med_prop.calc_state("PH", p_dis, h4)
            T_gas = state_5.T
            cp_gas = self.med_prop.calc_transport_properties(state_5).cp
        except Exception:
            return _zero

        if oil_path is None:
            _, eps = self._calculate_ntu_effectiveness(
                self.parameters["Ua_dis_ref"], m_dot_suc, cp_gas, self.parameters["m_dot_ref"])
            h_dis = h4 - eps * cp_gas * (T_gas - T_w)
            T_dis = T_gas - eps * (T_gas - T_w)
            return {"h_dis_gas": h_dis, "T_dis": T_dis,
                    "Q_dis_total": m_dot_suc * (h4 - h_dis),
                    "Q_dissolve_3": 0.0, "Q_oil_sump": 0.0,
                    "w_KM_mix": None, "w_KM_dis": None, "m_dot_gas_exit": m_dot_suc}

        # ===== STAGE 1: Adiabatic mixing + back-dissolution =====
        m_dot_gas_in = m_dot_suc + oil_path["m_dot_KM_degas_total"]
        m_dot_fl_after = oil_path["m_dot_fl"]
        m_dot_oil = oil_path["m_dot_oil"]
        T_oil = oil_path["T_oil_after"]
        w_KM_after = oil_path["w_KM_after"]
        m_dot_KM_in_oil = m_dot_oil * w_KM_after / (1.0 - w_KM_after)

        H_in = m_dot_gas_in * h4 + m_dot_fl_after * lubricant.calc_h_mix(T_oil, p_dis, w_KM_after)

        stage1_ok = False
        try:
            def _s1_res(T_trial):
                w_t = lubricant.solve_w_KM(T_trial, p_dis)
                if w_t is None:
                    return 1e6
                m_KM_t = m_dot_oil * w_t / (1.0 - w_t)
                m_dis = max(0.0, m_KM_t - m_dot_KM_in_oil)
                m_g = m_dot_gas_in - m_dis
                if m_g < 0:
                    return -1e6
                return (m_g * self.med_prop.calc_state("PT", p_dis, T_trial).h
                        + (m_dot_oil + m_KM_t) * lubricant.calc_h_mix(T_trial, p_dis, w_t)
                        - H_in)
            T_mix = brentq(_s1_res, min(T_oil, T_gas) - 5.0, max(T_oil, T_gas) + 5.0,
                           xtol=0.01, maxiter=50)
            w_KM_mix = lubricant.solve_w_KM(T_mix, p_dis)
            if w_KM_mix is not None:
                stage1_ok = True
        except Exception:
            pass

        if stage1_ok:
            m_dot_KM_mix = m_dot_oil * w_KM_mix / (1.0 - w_KM_mix)
            m_dissolved_1 = max(0.0, m_dot_KM_mix - m_dot_KM_in_oil)
            m_dot_gas_mix = m_dot_gas_in - m_dissolved_1
            m_dot_fl_mix = m_dot_oil + m_dot_KM_mix
        else:
            self._stage1_fallback_count += 1
            cp_fl_fb = lubricant.calc_cp_mix(T_oil, p_dis, w_KM_after)
            T_mix = ((m_dot_gas_in * cp_gas * T_gas + m_dot_fl_after * cp_fl_fb * T_oil)
                     / (m_dot_gas_in * cp_gas + m_dot_fl_after * cp_fl_fb))
            w_KM_mix = w_KM_after
            m_dot_gas_mix = m_dot_gas_in
            m_dot_fl_mix = m_dot_fl_after

        m_dot_total = m_dot_gas_mix + m_dot_fl_mix
        if m_dot_total < 1e-10:
            return _zero

        # ===== STAGE 2: NTU-epsilon discharge HT =====
        try:
            cp_gas_mix = self.med_prop.calc_transport_properties(
                self.med_prop.calc_state("PT", p_dis, T_mix)).cp
        except Exception:
            cp_gas_mix = cp_gas
        cp_fl_mix = lubricant.calc_cp_mix(T_mix, p_dis, w_KM_mix)
        cp_comb = (m_dot_gas_mix * cp_gas_mix + m_dot_fl_mix * cp_fl_mix) / m_dot_total

        _, eps = self._calculate_ntu_effectiveness(
            self.parameters["Ua_dis_ref"], m_dot_total, cp_comb, self.parameters["m_dot_ref"])
        T_dis = T_mix - eps * (T_mix - T_w)
        Q_dis_total = m_dot_total * cp_comb * eps * (T_mix - T_w)

        # ===== STAGE 3: Post-HT back-dissolution at (T_dis, p_dis) =====
        w_KM_dis = lubricant.solve_w_KM(T_dis, p_dis)
        Q_dissolve_3 = 0.0
        m_dot_gas_dis = m_dot_gas_mix

        if w_KM_dis is not None and w_KM_dis != w_KM_mix:
            m_dot_KM_dis = m_dot_oil * w_KM_dis / (1.0 - w_KM_dis)
            m_dot_KM_mix_mass = m_dot_oil * w_KM_mix / (1.0 - w_KM_mix)
            delta_m_3 = m_dot_KM_dis - m_dot_KM_mix_mass
            if delta_m_3 > 0:
                m_dot_gas_dis = m_dot_gas_mix - delta_m_3
            m_dot_fl_dis = m_dot_oil + m_dot_KM_dis

            try:
                h_gas_dis = self.med_prop.calc_state("PT", p_dis, T_dis).h
                H_b3 = m_dot_gas_mix * h_gas_dis + m_dot_fl_mix * lubricant.calc_h_mix(T_dis, p_dis, w_KM_mix)
                H_a3 = m_dot_gas_dis * h_gas_dis + m_dot_fl_dis * lubricant.calc_h_mix(T_dis, p_dis, w_KM_dis)
                Q_dissolve_3 = H_b3 - H_a3
            except Exception:
                Q_dissolve_3 = 0.0
        else:
            if w_KM_dis is None:
                w_KM_dis = w_KM_mix

        # ===== Q_OIL_SUMP: (T_dis, w_KM_dis) → (T_sump, w_KM_sump) =====
        T_oil_sump = oil_path["T_oil_sump"]
        w_KM_sump = oil_path["w_KM_sump"]
        m_dot_fl_dis_ret = m_dot_oil / (1.0 - w_KM_dis)
        m_dot_fl_sump = m_dot_oil / (1.0 - w_KM_sump)
        delta_m_KM_sump = (m_dot_oil * w_KM_sump / (1.0 - w_KM_sump)
                           - m_dot_oil * w_KM_dis / (1.0 - w_KM_dis))

        h_fl_dis_ret = lubricant.calc_h_mix(T_dis, p_dis, w_KM_dis)
        h_fl_sump_lv = lubricant.calc_h_mix(T_oil_sump, p_dis, w_KM_sump)
        try:
            h_gas_housing = self.med_prop.calc_state("PT", p_dis, T_dis).h
        except Exception:
            h_gas_housing = 0.0

        H_in_sump = m_dot_fl_dis_ret * h_fl_dis_ret
        if delta_m_KM_sump > 0:
            H_in_sump += delta_m_KM_sump * h_gas_housing
        Q_oil_sump = H_in_sump - m_dot_fl_sump * h_fl_sump_lv

        # Gas-only enthalpy at T_dis for state_outlet
        try:
            h_dis_gas = self.med_prop.calc_state("PT", p_dis, T_dis).h
        except Exception:
            h_dis_gas = h4 - eps * cp_gas * (T_gas - T_w)

        # Gas exiting compressor after all dissolution
        m_dot_gas_exit = m_dot_gas_dis
        if delta_m_KM_sump > 0:
            m_dot_gas_exit -= delta_m_KM_sump

        return {"h_dis_gas": h_dis_gas, "T_dis": T_dis, "Q_dis_total": Q_dis_total,
                "Q_dissolve_3": Q_dissolve_3, "Q_oil_sump": Q_oil_sump,
                "w_KM_mix": w_KM_mix, "w_KM_dis": w_KM_dis,
                "m_dot_gas_exit": m_dot_gas_exit}

    # =================================================================
    # OIL PATH (suction side) — Steps 1-8, no Q_oil_sump
    # =================================================================
    def _calculate_oil_path(self, T_w, T_dis_exact, inputs, p_suc, p_dis):
        """Suction-side oil path. Q_oil_sump is in discharge HT."""
        lubricant = self._get_lubricant_model()
        f = self.get_n_absolute(inputs.control.n)
        f_ref = self.parameters["f_ref"]

        m_dot_oil = self.parameters["m_dot_oil_ref"] * (f / f_ref) ** 2
        T_amb = self._get_ambient_temperature(inputs)
        T_oil_sump = self._calculate_oil_sump_temperature(T_dis_exact, self.state_inlet.T, T_amb)

        w_KM_sump = lubricant.solve_w_KM(T_oil_sump, p_dis)
        if w_KM_sump is None:
            return None

        m_dot_KM_in_oil = m_dot_oil * w_KM_sump / (1.0 - w_KM_sump)
        m_dot_fl_sump = m_dot_oil + m_dot_KM_in_oil
        h_before_throttle = m_dot_fl_sump * lubricant.calc_h_mix(T_oil_sump, p_dis, w_KM_sump)

        # Isenthalpic throttle
        T_throttle = T_oil_sump
        try:
            def _thr_res(T_t):
                w_t = lubricant.solve_w_KM(T_t, p_suc)
                if w_t is None: return 1e6
                m_KM_t = m_dot_oil * w_t / (1.0 - w_t)
                m_fl_t = m_dot_oil + m_KM_t
                m_dg = max(0.0, m_dot_KM_in_oil - m_KM_t)
                return (m_fl_t * lubricant.calc_h_mix(T_t, p_suc, w_t)
                        + m_dg * self.med_prop.calc_state("PT", p_suc, T_t).h
                        - h_before_throttle)
            T_throttle = brentq(_thr_res, T_oil_sump - 30.0, T_oil_sump + 1.0, xtol=0.01, maxiter=50)
        except Exception:
            self._throttle_fallback_count += 1

        w_KM_suc = lubricant.solve_w_KM(T_throttle, p_suc)
        if w_KM_suc is None:
            return None

        m_dot_KM_suc = m_dot_oil * w_KM_suc / (1.0 - w_KM_suc)
        m_dot_KM_degas_thr = max(0.0, m_dot_KM_in_oil - m_dot_KM_suc)
        m_dot_fl_in = m_dot_oil / (1.0 - w_KM_suc)

        try:
            h_degas_thr = self.med_prop.calc_state("PT", p_suc, T_throttle).h
        except Exception:
            h_degas_thr = self.med_prop.calc_state("PQ", p_suc, 1).h

        # Combined liquid + gas NTU-epsilon
        cp_fl_in = lubricant.calc_cp_mix(T_throttle, p_suc, w_KM_suc)
        m_dot_total_suc = m_dot_fl_in + m_dot_KM_degas_thr
        if m_dot_KM_degas_thr > 1e-12 and m_dot_total_suc > 1e-12:
            try:
                cp_gas_thr = self.med_prop.calc_transport_properties(
                    self.med_prop.calc_state("PT", p_suc, T_throttle)).cp
            except Exception:
                cp_gas_thr = 0.0
            cp_comb = (m_dot_fl_in * cp_fl_in + m_dot_KM_degas_thr * cp_gas_thr) / m_dot_total_suc
        else:
            m_dot_total_suc = m_dot_fl_in
            cp_comb = cp_fl_in

        _, eps_oil = self._calculate_ntu_effectiveness(
            self.parameters["Ua_suc_oil_ref"], m_dot_total_suc, cp_comb, self.parameters["m_dot_oil_ref"])
        T_oil_after = T_throttle + eps_oil * (T_w - T_throttle)

        # Solubility after HT
        w_KM_after = lubricant.solve_w_KM(T_oil_after, p_suc)
        if w_KM_after is None:
            if ENABLE_TIMING:
                print(f"  Warning: solve_w_KM failed at T_oil_after={T_oil_after:.1f} K, "
                      f"p_suc={p_suc:.0f} Pa, using w_KM_suc={w_KM_suc:.6f} as fallback")
            self._w_KM_after_fallback_count += 1
            w_KM_after = w_KM_suc

        m_dot_KM_after = m_dot_oil * w_KM_after / (1.0 - w_KM_after)
        m_dot_KM_degas_ht_raw = m_dot_KM_suc - m_dot_KM_after
        m_dot_KM_degas_ht_eff = max(-m_dot_KM_degas_thr, m_dot_KM_degas_ht_raw)
        m_dot_KM_degas_total = m_dot_KM_degas_thr + m_dot_KM_degas_ht_eff

        m_dot_KM_after_eff = m_dot_KM_suc - m_dot_KM_degas_ht_eff
        w_KM_after_eff = (m_dot_KM_after_eff / (m_dot_oil + m_dot_KM_after_eff)
                          if m_dot_KM_after_eff + m_dot_oil > 0 else 0.0)
        m_dot_fl_after = m_dot_oil / (1.0 - w_KM_after_eff) if w_KM_after_eff < 1.0 else m_dot_oil

        # Q_suc_oil enthalpy balance
        h_fl_in = lubricant.calc_h_mix(T_throttle, p_suc, w_KM_suc)
        h_fl_after = lubricant.calc_h_mix(T_oil_after, p_suc, w_KM_after_eff)
        try:
            h_degas_out = self.med_prop.calc_state("PT", p_suc, T_oil_after).h
        except Exception:
            h_degas_out = self.med_prop.calc_state("PQ", p_suc, 1).h

        Q_suc_oil = (m_dot_fl_after * h_fl_after + m_dot_KM_degas_total * h_degas_out
                     - m_dot_fl_in * h_fl_in - m_dot_KM_degas_thr * h_degas_thr)

        # Hydraulic recirculation loss
        rho_fl = lubricant.calc_rho_mix(T_oil_after, w_KM_after_eff)
        W_dot_oil_recirc = m_dot_fl_after * (p_dis - p_suc) / rho_fl if rho_fl > 0 else 0.0

        return {"m_dot_oil": m_dot_oil, "m_dot_fl_in": m_dot_fl_in, "m_dot_fl": m_dot_fl_after,
                "m_dot_KM_degas_thr": m_dot_KM_degas_thr, "m_dot_KM_degas_ht": m_dot_KM_degas_ht_eff,
                "m_dot_KM_degas_ht_raw": m_dot_KM_degas_ht_raw,
                "m_dot_KM_degas_total": m_dot_KM_degas_total,
                "h_degas_thr": h_degas_thr, "h_degas_out": h_degas_out,
                "Q_suc_oil": Q_suc_oil, "W_dot_oil_recirc": W_dot_oil_recirc,
                "T_oil_sump": T_oil_sump, "T_throttle": T_throttle, "T_oil_after": T_oil_after,
                "w_KM_sump": w_KM_sump, "w_KM_suc": w_KM_suc,
                "w_KM_after": w_KM_after_eff, "w_KM_after_raw": w_KM_after,
                "epsilon_oil_suc": eps_oil}

    # =================================================================
    # Loss power
    # =================================================================
    def _calculate_loss_power(self, h4, h3, T_dis_exact, p_dis, inputs):
        f = self.get_n_absolute(inputs.control.n)
        f_ref = self.parameters["f_ref"]
        m_dot_3 = self.state_c_3.d * self.parameters["V_IC"] * f
        W_dot_int = m_dot_3 * (h4 - h3)
        T_amb = self._get_ambient_temperature(inputs)
        T_oil_sump = self._calculate_oil_sump_temperature(T_dis_exact, self.state_inlet.T, T_amb)
        mu_oil = self._calculate_oil_viscosity(T_oil_sump, p_dis)
        mu_mix_eff = self._milli_pas_to_pas(mu_oil)
        W_loss_load = float(self.parameters["alpha_loss"]) * W_dot_int
        W_loss_ref = float(self.parameters.get("W_dot_loss_ref", 0.0)) * (f / f_ref) ** 2
        W_loss_fric = (float(self.parameters.get("alpha_fric_tot", 0.0))
                       * mu_mix_eff * self.V_h * (2.0 * np.pi * f) ** 2)
        return {"W_dot_int": W_dot_int, "W_dot_loss": W_loss_load + W_loss_ref + W_loss_fric,
                "W_dot_loss_load": W_loss_load, "W_dot_loss_ref_term": W_loss_ref,
                "W_dot_loss_fric": W_loss_fric, "T_oil_sump": T_oil_sump,
                "mu_oil": mu_oil, "mu_mix_eff": mu_mix_eff}

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
        return [m0, Tw0, h_suc + 0.5 * cp_suc * (Tw0 - self.state_inlet.T), 1.1 * p_outlet, h_suc + 0.5 * cp_suc * (Tw0 - self.state_inlet.T)]

    def _clear_cache(self):
        self._state_cache.clear(); self._oil_viscosity_cache.clear(); self._cached_discharge_valve.clear()
        self._current_oil_path = self._dis_ht_result = None
        self._T_dis_est = self._T_dis_corr = self._T_dis_final = self._last_x = None
        self._cache_hits = self._cache_misses = 0
        self._corrector_fallback_count = self._solver_fallback_count = 0
        self._w_KM_after_fallback_count = self._throttle_fallback_count = self._stage1_fallback_count = 0
        self._gamma4_min = self._gamma4_max = None; self._gamma4_n = 0

    def _get_cache_key(self, p_suc, h1, h3, p4, s3):
        return (round(p_suc, 2), round(h1, 1), round(h3, 1), round(p4, 2), round(s3, 4))

    def _calculate_residuals(self, x, inputs, p_outlet):
        m_dot_suc, T_w, h1, p4, h3 = x
        p_suc = self.state_inlet.p
        p_dis = p_outlet
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3, rho3 = self.state_c_3.s, self.state_c_3.d

        ck = self._get_cache_key(p_suc, h1, h3, p4, s3)
        if (self._last_x is not None and np.allclose(x, self._last_x, rtol=1e-6, atol=1e-8) and ck in self._state_cache):
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
            self._state_cache[ck] = {"s1": self.state_c_1, "s4": self.state_c_4, "h4": h4, "g4": gamma4, "mt": m_dot_tot}
            if len(self._state_cache) > 1000:
                del self._state_cache[next(iter(self._state_cache))]
        self._last_x = x.copy()

        if self.debug_enabled:
            if self._gamma4_min is None: self._gamma4_min = self._gamma4_max = gamma4
            else: self._gamma4_min = min(self._gamma4_min, gamma4); self._gamma4_max = max(self._gamma4_max, gamma4)
            self._gamma4_n += 1

        # Predictor (gas-only)
        dis_est = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis, None)
        try: T_dis_est = self.med_prop.calc_state("PH", p_dis, dis_est["h_dis_gas"]).T
        except Exception: T_dis_est = self.state_c_4.T

        oil_pred = self._calculate_oil_path(T_w, T_dis_est, inputs, p_suc, p_dis)
        if oil_pred is None: return np.full(5, 1e6)

        # Corrector
        T_dis_corr = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis, oil_pred)["T_dis"]
        self._current_oil_path = self._calculate_oil_path(T_w, T_dis_corr, inputs, p_suc, p_dis)
        if self._current_oil_path is None: return np.full(5, 1e6)

        # Final
        dis_final = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis, self._current_oil_path)
        self._dis_ht_result = dis_final
        self._T_dis_est = T_dis_est; self._T_dis_corr = T_dis_corr; self._T_dis_final = dis_final["T_dis"]
        self._loss = self._calculate_loss_power(h4, h3, dis_final["T_dis"], p_dis, inputs)

        r = np.zeros(5)
        r[0] = self._residual_suction_ht(m_dot_suc, T_w, h1)
        r[1] = self._residual_discharge_valve(m_dot_suc, p4, h4, s3, p_dis, gamma4)
        r[2] = self._residual_compressor_flow(m_dot_suc, rho3, m_dot_tot, inputs)
        r[3] = self._residual_mixing_energy(m_dot_suc, h1, h3, h4, m_dot_tot)
        r[4] = self._residual_wall_energy(m_dot_suc, T_w, h1, h4, inputs)
        return r

    def _residual_suction_ht(self, m_dot_suc, T_w, h1):
        h_suc, T_suc = self.state_inlet.h, self.state_inlet.T
        cp_suc = self.med_prop.calc_transport_properties(self.state_inlet).cp
        _, eps = self._calculate_ntu_effectiveness(self.parameters["Ua_suc_ref"], m_dot_suc, cp_suc, self.parameters["m_dot_ref"])
        return h1 - h_suc - eps * cp_suc * (T_w - T_suc)

    def _residual_discharge_valve(self, m_dot_suc, p4, h4, s3, p_dis, gamma4):
        oil = self._current_oil_path
        m_dot_gas = m_dot_suc + oil["m_dot_KM_degas_total"]
        ck = (round(p4, 1), round(h4, 1), round(s3, 4), round(p_dis, 1), round(gamma4, 3))
        if ck in self._cached_discharge_valve:
            mv = self._cached_discharge_valve[ck]
        else:
            p_thr = max(p_dis, p4 * (2.0 / (gamma4 + 1.0)) ** (gamma4 / (gamma4 - 1.0)))
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

    def _residual_compressor_flow(self, m_dot_suc, rho3, m_dot_tot, inputs):
        f = self.get_n_absolute(inputs.control.n)
        oil = self._current_oil_path
        return rho3 * self.parameters["V_IC"] * f - (m_dot_suc + m_dot_tot + oil["m_dot_KM_degas_total"])

    def _residual_mixing_energy(self, m_dot_suc, h1, h3, h4, m_dot_tot):
        oil = self._current_oil_path
        m_dg = oil["m_dot_KM_degas_total"]
        m_3 = m_dot_suc + m_dot_tot + m_dg
        res = m_3 * h3 - (m_dot_suc * h1 + m_dot_tot * h4 + m_dg * oil["h_degas_out"])
        return res / m_3 if abs(m_3) > 1e-10 else res

    def _residual_wall_energy(self, m_dot_suc, T_w, h1, h4, inputs):
        loss, oil, dis = self._loss, self._current_oil_path, self._dis_ht_result
        Q_suc = m_dot_suc * (h1 - self.state_inlet.h)
        T_amb = self._get_ambient_temperature(inputs)
        Q_amb = np.sign(T_w - T_amb) * self.parameters["Ua_amb"] * abs(T_w - T_amb) ** 1.25
        res = ((loss["W_dot_loss"] + oil["W_dot_oil_recirc"])
               + dis["Q_dis_total"] + dis["Q_dissolve_3"] + dis["Q_oil_sump"]
               - Q_amb - Q_suc - oil["Q_suc_oil"])
        return res / loss["W_dot_int"] if abs(loss["W_dot_int"]) > 1e-10 else res

    # =================================================================
    # SOLVER
    # =================================================================
    def simulate_operating_point(self, inputs, p_outlet, fs_state):
        self._clear_cache()
        x0 = self._get_initial_guesses(inputs, p_outlet)
        bounds = [(1e-6, 1.0), (self.state_inlet.T, 400.0),
                  (self.state_inlet.h, self.state_inlet.h + 100e6),
                  (p_outlet * 1.001, p_outlet * 15.0),
                  (self.state_inlet.h, self.state_inlet.h + 200e6)]
        max_nfev = int(getattr(inputs, "lsq_max_nfev", 20000))
        ftol = float(getattr(inputs, "lsq_ftol", 1e-8))
        xtol = float(getattr(inputs, "lsq_xtol", 1e-8))
        result = least_squares(lambda x: self._calculate_residuals(x, inputs, p_outlet), x0,
                               bounds=([b[0] for b in bounds], [b[1] for b in bounds]),
                               method="trf", ftol=ftol, xtol=xtol, max_nfev=max_nfev)
        if not result.success:
            print(f"  Solver warning: {result.message}\nTrying to use solution anyway...")
        self._calculate_final_states(result.x, inputs, p_outlet, fs_state)
        return result.x

    # =================================================================
    # FINAL STATES
    # =================================================================
    def _calculate_final_states(self, solution, inputs, p_outlet, fs_state):
        m_dot_suc, T_w, h1, p4, h3 = solution
        p_suc, p_dis = self.state_inlet.p, p_outlet
        self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
        h4 = self.state_c_4.h

        # Predictor
        dis_est = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis, None)
        try: T_dis_est = self.med_prop.calc_state("PH", p_dis, dis_est["h_dis_gas"]).T
        except Exception: T_dis_est = self.state_c_4.T

        oil_pred = self._calculate_oil_path(T_w, T_dis_est, inputs, p_suc, p_dis)
        T_dis_corr = T_dis_est
        if oil_pred is not None:
            T_dis_corr = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis, oil_pred)["T_dis"]
            oil = self._calculate_oil_path(T_w, T_dis_corr, inputs, p_suc, p_dis)
            if oil is None:
                if ENABLE_TIMING: print("  Warning: corrector oil path failed, using predictor result")
                self._corrector_fallback_count += 1; oil = oil_pred
        else:
            oil = None

        if oil is None:
            if self._current_oil_path is not None:
                print("  Warning: final-state oil path failed, using last solver result")
                self._solver_fallback_count += 1; oil = self._current_oil_path
            else:
                raise RuntimeError("Oil path failed in _calculate_final_states and no valid solver result available.")

        self._T_dis_est, self._T_dis_corr = T_dis_est, T_dis_corr

        dis_final = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis, oil)
        T_dis = dis_final["T_dis"]
        self._T_dis_final = T_dis

        pc_convergence_gap = abs(T_dis - T_dis_corr)
        if pc_convergence_gap > 1.0 and self.debug_enabled:
            print(f"  Warning: predictor-corrector convergence gap = {pc_convergence_gap:.2f} K "
                  f"(est={T_dis_est:.1f} K, corr={T_dis_corr:.1f} K, final={T_dis:.1f} K)")

        self.state_c_5 = self.med_prop.calc_state("PH", p_dis, dis_final["h_dis_gas"])
        self.state_outlet = self.state_c_5

        loss = self._calculate_loss_power(h4, h3, T_dis, p_dis, inputs)
        self.T_oil_sump = loss["T_oil_sump"]; self.mu_oil = loss["mu_oil"]; self.mu_mix_eff = loss["mu_mix_eff"]
        self.W_dot_int = loss["W_dot_int"]; self.W_dot_loss = loss["W_dot_loss"]
        self.W_dot_loss_load = loss["W_dot_loss_load"]; self.W_dot_loss_ref_term = loss["W_dot_loss_ref_term"]
        self.W_dot_loss_fric = loss["W_dot_loss_fric"]

        W_recirc = oil["W_dot_oil_recirc"]
        self.P_el = self.W_dot_int + self.W_dot_loss + W_recirc
        self.W_dot_comp = self.P_el; self.m_flow = m_dot_suc; self.T_w = T_w

        self.m_dot_oil = oil["m_dot_oil"]; self.Q_dot_suc_oil = oil["Q_suc_oil"]
        self.W_dot_oil_recirc = W_recirc
        self.m_dot_KM_degas_thr = oil["m_dot_KM_degas_thr"]; self.m_dot_KM_degas_ht = oil["m_dot_KM_degas_ht"]
        self.m_dot_KM_degas_total = oil["m_dot_KM_degas_total"]; self.m_dot_fl = oil["m_dot_fl"]
        self.w_KM_sump = oil["w_KM_sump"]; self.w_KM_after = oil["w_KM_after"]
        self.T_oil_after = oil["T_oil_after"]; self.T_throttle = oil["T_throttle"]

        self.Q_oil_sump = dis_final["Q_oil_sump"]; self.Q_dissolve_3 = dis_final["Q_dissolve_3"]
        self.w_KM_mix = dis_final["w_KM_mix"]; self.w_KM_dis = dis_final["w_KM_dis"]
        self.Q_dis_total = dis_final["Q_dis_total"]; self.m_dot_KM_gas = dis_final["m_dot_gas_exit"]

        self.m_dot_KM_degas_ht_raw = oil["m_dot_KM_degas_ht_raw"]; self.w_KM_after_raw = oil["w_KM_after_raw"]
        self.T_dis_est = T_dis_est; self.T_dis_corr = T_dis_corr; self.pc_convergence_gap = pc_convergence_gap
        self.m_dot_gas_discharge = m_dot_suc + self.m_dot_KM_degas_total

        fs_state.set("m_flow", self.m_flow, "kg/s", "External suction refrigerant mass flow rate")
        fs_state.set("m_dot_gas_discharge", self.m_dot_gas_discharge, "kg/s", "Gas through discharge valve")
        fs_state.set("m_dot_gas_exit", self.m_dot_KM_gas, "kg/s", "Gas exiting compressor (should = m_suc)")
        fs_state.set("P_el", self.P_el, "W", "Electrical power input")
        fs_state.set("W_dot_int", self.W_dot_int, "W", "Internal compression power")
        fs_state.set("W_dot_loss", self.W_dot_loss, "W", "Compressor loss power")
        fs_state.set("W_dot_loss_load", self.W_dot_loss_load, "W", "Load-dependent loss")
        fs_state.set("W_dot_loss_ref_term", self.W_dot_loss_ref_term, "W", "Speed-dependent loss")
        fs_state.set("W_dot_loss_fric", self.W_dot_loss_fric, "W", "Viscous friction loss")
        fs_state.set("W_dot_oil_recirc", self.W_dot_oil_recirc, "W", "Hydraulic oil recirc. loss")
        fs_state.set("Q_oil_sump", self.Q_oil_sump, "W", "Oil sump heat rejection")
        fs_state.set("Q_dissolve_3", self.Q_dissolve_3, "W", "Stage 3 dissolution heat")
        fs_state.set("T_wall", T_w, "K", "Wall temperature")
        fs_state.set("T_dis", T_dis, "K", "Discharge temperature")
        fs_state.set("T_oil_sump", self.T_oil_sump, "K", "Oil sump temperature")
        fs_state.set("T_throttle", self.T_throttle, "K", "Oil T after isenthalpic throttle")
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
        fs_state.set("m_dot_fl", self.m_dot_fl, "kg/s", "Liquid oil stream (suction side)")
        fs_state.set("w_KM_sump", self.w_KM_sump, "-", "KM fraction at sump")
        fs_state.set("w_KM_after", self.w_KM_after, "-", "KM fraction after suction HT (eff.)")
        fs_state.set("w_KM_mix", self.w_KM_mix, "-", "KM fraction after Stage 1 (diag.)")
        fs_state.set("w_KM_dis", self.w_KM_dis, "-", "KM fraction after Stage 3 (diag.)")
        fs_state.set("T_oil_after", self.T_oil_after, "K", "Oil T after suction HT")
        fs_state.set("w_KM_after_raw", self.w_KM_after_raw, "-", "KM fraction after HT raw (diag.)")
        fs_state.set("m_dot_KM_degas_ht_raw", self.m_dot_KM_degas_ht_raw, "kg/s", "Raw HT degassing (diag.)")
        fs_state.set("Q_dis_total", self.Q_dis_total, "W", "Discharge HT Stage 2 (diag.)")
        fs_state.set("T_dis_est", self.T_dis_est, "K", "Predictor T_dis gas-only (diag.)")
        fs_state.set("T_dis_corr", self.T_dis_corr, "K", "Corrector T_dis (diag.)")
        fs_state.set("pc_convergence_gap", self.pc_convergence_gap, "K", "|T_final-T_corr| (diag.)")

    # =================================================================
    # INTERFACE
    # =================================================================
    def get_eta_mech(self, inputs):
        if self.W_dot_comp is None or self.W_dot_comp <= 0.0: return float("nan")
        if self.state_c_3 is None or self.state_c_4 is None: return float("nan")
        if self.state_c_3.d is None or self.state_c_4.h is None or self.state_c_3.h is None: return float("nan")
        f = self.get_n_absolute(inputs.control.n)
        W_int = self.state_c_3.d * self.parameters["V_IC"] * f * (self.state_c_4.h - self.state_c_3.h)
        return W_int / self.W_dot_comp if W_int > 0.0 else float("nan")

    def get_lambda_h(self, inputs):
        if self.m_flow is None or self.state_inlet is None or self.state_inlet.d is None or self.m_flow <= 0.0:
            return float("nan")
        f = self.get_n_absolute(inputs.control.n)
        m_th = self.state_inlet.d * self.V_h * f
        return self.m_flow / m_th if m_th > 0.0 else float("nan")

    def get_eta_isentropic(self, p_outlet, inputs, fs_state):
        if self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2 or self.state_inlet.p != fs_state.p_1:
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        try:
            h_suc, h_dis = self.state_inlet.h, self.state_outlet.h
            if h_dis <= h_suc: return float("nan")
            h_is = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s).h
            w_act = h_dis - h_suc
            return (h_is - h_suc) / w_act if h_is > h_suc and w_act > 0.0 else float("nan")
        except Exception:
            return float("nan")

    def calc_state_outlet(self, p_outlet, inputs, fs_state):
        self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

    def calc_m_flow(self, p_outlet, inputs, fs_state):
        if self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2 or self.state_inlet.p != fs_state.p_1:
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        return self.m_flow

    def calc_electrical_power(self, p_outlet, inputs, fs_state):
        if self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2 or self.state_inlet.p != fs_state.p_1:
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        return self.P_el

    def get_debug_report(self):
        if not self.debug_enabled or self._gamma4_n == 0: return "Debug disabled."
        pc = self.pc_convergence_gap if self.pc_convergence_gap is not None else 0.0
        oe = abs(self.T_dis_corr - self.T_dis_est) if self.T_dis_corr is not None and self.T_dis_est is not None else 0.0
        return (f"Oil Path v7 Debug Report\n"
                f"  gamma4: {self._gamma4_min:.6f}..{self._gamma4_max:.6f} (N={self._gamma4_n})\n"
                f"  state_cache: hits={self._cache_hits}, misses={self._cache_misses}\n"
                f"  dis_valve_cache: {len(self._cached_discharge_valve)}\n"
                f"  visc_cache: {len(self._oil_viscosity_cache)}\n"
                f"  pc_convergence_gap: {pc:.3f} K (|T_final-T_corr|)\n"
                f"  oil_effect_magnitude: {oe:.1f} K (|T_corr-T_est|)\n"
                f"  corrector_fb: {self._corrector_fallback_count}  solver_fb: {self._solver_fallback_count}\n"
                f"  w_KM_after_fb: {self._w_KM_after_fallback_count}  throttle_fb: {self._throttle_fallback_count}\n"
                f"  stage1_fb: {self._stage1_fallback_count}")
