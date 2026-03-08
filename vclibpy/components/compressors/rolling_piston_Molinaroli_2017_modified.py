from scipy.optimize import least_squares
import numpy as np

from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs, FlowsheetState
from vclibpy.media import ThermodynamicState, LubricantFitting


ENABLE_TIMING = True


class Molinaroli_2017_Compressor(Compressor):
    """
    Semi-empirical rolling-piston compressor model after Molinaroli et al. (2017),
    extended by a viscosity-dependent friction-loss formulation.

    Extension:
        W_loss = alpha_loss * W_int
               + W_loss_ref * (f / f_ref)^2 * (mu / mu_ref)^n_fric

    The oil/refrigerant viscosity mu is obtained from the lubricant-fitting correlation,
    using:
        - oil sump temperature from the Zhang correlation
        - discharge pressure as approximation for the oil-sump pressure
    """

    def __init__(
        self,
        N_max: float,
        V_h: float,
        fluid_name: str = "propane",
        lub_name: str = "LPG 68",
        parameters=None,
    ):
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
                "m_dot_ref": 0.0083,
                "f_ref": 50.0,
                # new parameters for viscosity-dependent friction losses
                "mu_ref": 3.0,   # same unit returned by LubricantFitting (currently documented/commented as mPa*s)
                "n_fric": 1.0,
            }

        self.parameters = parameters
        self.fluid_name = fluid_name
        self.lub_name = lub_name
        self.lubricant_model = LubricantFitting(fluid_name=fluid_name, lub_name=lub_name)

        # Thermodynamic states
        self.state_c_suc: ThermodynamicState = None
        self.state_c_1: ThermodynamicState = None
        self.state_c_3: ThermodynamicState = None
        self.state_c_4: ThermodynamicState = None
        self.state_c_5: ThermodynamicState = None

        # Unknown variables / results
        self.T_w = None
        self.m_flow_suc = None
        self.state_outlet = None
        self.W_dot_comp = None
        self.P_el = None

        # additional oil-related outputs
        self.T_oil_sump = None
        self.mu_oil = None

        # caching for solver calls
        self._cached_m_dot_tot = None
        self._state_cache = {}
        self._last_x = None
        self._cache_hits = 0
        self._cache_misses = 0
        self._cached_discharge_valve = {}
        self._oil_viscosity_cache = {}

        # debug info
        self.debug_enabled = False
        self._gamma4_min = None
        self._gamma4_max = None
        self._gamma4_n = 0

    # ---------------------------------------------------------------------
    # helper functions for the viscosity extension
    # ---------------------------------------------------------------------
    @staticmethod
    def _kelvin_to_celsius(T_K: float) -> float:
        return T_K - 273.15

    @staticmethod
    def _celsius_to_kelvin(T_C: float) -> float:
        return T_C + 273.15

    def _get_ambient_temperature(self, inputs) -> float:
        return float(getattr(inputs, "T_amb", 25.0 + 273.15))

    def _calculate_oil_sump_temperature(self, T_dis_K: float, T_in_K: float, T_amb_K: float) -> float:
        """
        Unified Zhang correlation for oil sump temperature.

        Important:
            The regression is formulated in °C, so conversion is done explicitly here.

        Toil ≈ 0.914227*Tdis + 0.008136*Tin + 0.006144*Tamb
        """
        T_dis_C = self._kelvin_to_celsius(T_dis_K)
        T_in_C = self._kelvin_to_celsius(T_in_K)
        T_amb_C = self._kelvin_to_celsius(T_amb_K)

        T_oil_C = 0.914227 * T_dis_C + 0.008136 * T_in_C + 0.006144 * T_amb_C
        return self._celsius_to_kelvin(T_oil_C)

    def _calculate_oil_viscosity(self, T_oil_K: float, p_oil: float) -> float:
        """
        Calculate dynamic viscosity of the oil/refrigerant mixture.

        To reduce the cost inside the residual loop, the viscosity is cached with a
        rounded (T, p) key. That avoids repeated root-finding in LubricantFitting
        for nearly identical solver states.
        """
        cache_key = (round(T_oil_K, 3), round(p_oil, 1))
        if cache_key in self._oil_viscosity_cache:
            return self._oil_viscosity_cache[cache_key]

        oil_state = ThermodynamicState(p=p_oil, T=T_oil_K)

        try:
            transport = self.lubricant_model.calc_transport_properties(
                state=oil_state,
                phase="liquid",
            )
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in oil viscosity calculation: {err}")
            transport = None

        mu_ref = float(self.parameters.get("mu_ref", 1.0))

        if transport is None or transport.dyn_vis is None:
            mu = mu_ref
        else:
            mu = float(transport.dyn_vis)
            if not np.isfinite(mu) or mu <= 0.0:
                mu = mu_ref

        self._oil_viscosity_cache[cache_key] = mu
        if len(self._oil_viscosity_cache) > 1000:
            first_key = next(iter(self._oil_viscosity_cache))
            del self._oil_viscosity_cache[first_key]

        return mu

    def _calculate_viscosity_corrected_loss_power(
        self,
        m_dot_suc: float,
        T_w: float,
        h4: float,
        h3: float,
        p_dis: float,
        inputs,
        use_exact_discharge_state: bool = True,
    ):
        """
        Common helper used in the residual equation and in the final post-processing.

        Returns
        -------
        dict with:
            W_dot_int, W_dot_loss, h_dis, epsilon_dis, state_dis, T_oil_sump, mu_oil

        Notes
        -----
        This helper can be used in the residual loop and in the final post-processing.
        In this version, the exact discharge state is also used inside the residual
        loop, so T_dis for the Zhang correlation is always obtained from a PH-state
        call at (p_dis, h_dis).
        """
        n_abs = self.get_n_absolute(inputs.control.n)
        rho3 = self.state_c_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * n_abs
        W_dot_int = m_dot_3 * (h4 - h3)

        h_dis, epsilon_dis, T_dis_approx = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis)

        state_dis = None
        T_dis_for_viscosity = T_dis_approx
        if use_exact_discharge_state:
            state_dis = self.med_prop.calc_state("PH", p_dis, h_dis)
            T_dis_for_viscosity = state_dis.T
        else:
            # Kept for completeness, but the current model version uses the exact
            # discharge state in the residual loop as well.
            T_dis_for_viscosity = T_dis_approx

        T_amb = self._get_ambient_temperature(inputs)
        T_oil_sump = self._calculate_oil_sump_temperature(
            T_dis_K=T_dis_for_viscosity,
            T_in_K=self.state_inlet.T,
            T_amb_K=T_amb,
        )
        mu_oil = self._calculate_oil_viscosity(T_oil_K=T_oil_sump, p_oil=p_dis)

        alpha_loss = self.parameters["alpha_loss"]
        W_dot_loss_ref = self.parameters["W_dot_loss_ref"]
        f_ref = self.parameters["f_ref"]
        mu_ref = float(self.parameters.get("mu_ref", 1.0))
        n_fric = float(self.parameters.get("n_fric", 1.0))

        mu_ratio = max(mu_oil / mu_ref, 1e-12)
        W_dot_loss = (
            alpha_loss * W_dot_int
            + W_dot_loss_ref * (n_abs / f_ref) ** 2 * mu_ratio ** n_fric
        )

        return {
            "W_dot_int": W_dot_int,
            "W_dot_loss": W_dot_loss,
            "h_dis": h_dis,
            "epsilon_dis": epsilon_dis,
            "state_dis": state_dis,
            "T_dis_for_viscosity": T_dis_for_viscosity,
            "T_oil_sump": T_oil_sump,
            "mu_oil": mu_oil,
        }

    # ---------------------------------------------------------------------
    # original model structure
    # ---------------------------------------------------------------------
    def _get_initial_guesses(self, inputs, p_outlet):
        h_suc = self.state_inlet.h
        s_suc = self.state_inlet.s

        n_abs = self.get_n_absolute(inputs.control.n)
        p_dis = p_outlet

        state_dis_is = self.med_prop.calc_state("PS", p_dis, s_suc)
        T_dis_is = state_dis_is.T

        rho_suc = self.state_inlet.d
        m_dot_suc_0 = rho_suc * self.parameters["V_IC"] * n_abs
        T_w_0 = 0.5 * (self.state_inlet.T + T_dis_is)

        transport_suc = self.med_prop.calc_transport_properties(self.state_inlet)
        cp_suc = transport_suc.cp
        h1_0 = h_suc + 0.5 * cp_suc * (T_w_0 - self.state_inlet.T)

        p4_0 = 1.1 * p_dis
        h3_0 = h1_0
        return [m_dot_suc_0, T_w_0, h1_0, p4_0, h3_0]

    def _clear_cache(self):
        self._state_cache.clear()
        self._oil_viscosity_cache.clear()
        self._last_x = None
        self._cache_hits = 0
        self._cache_misses = 0
        self._gamma4_min = None
        self._gamma4_max = None
        self._gamma4_n = 0

    def _get_cache_key(self, p_suc, h1, h3, p4, s3):
        return (
            round(p_suc, 2),
            round(h1, 1),
            round(h3, 1),
            round(p4, 2),
            round(s3, 4),
        )

    def _calculate_residuals(self, x, inputs, p_outlet):
        m_dot_suc, T_w, h1, p4, h3 = x

        p_suc = self.state_inlet.p
        p_dis = p_outlet

        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        rho3 = self.state_c_3.d

        cache_key = self._get_cache_key(p_suc, h1, h3, p4, s3)

        if (
            self._last_x is not None
            and np.allclose(x, self._last_x, rtol=1e-6, atol=1e-8)
            and cache_key in self._state_cache
        ):
            cached = self._state_cache[cache_key]
            self.state_c_1 = cached["state_c_1"]
            self.state_c_4 = cached["state_c_4"]
            h4 = cached["h4"]
            gamma4 = cached["gamma4"]
            m_dot_tot = cached["m_dot_tot"]
            self._cache_hits += 1
        else:
            self._cache_misses += 1
            self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)
            self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
            h4 = self.state_c_4.h

            transport4 = self.med_prop.calc_transport_properties(self.state_c_4)
            gamma4 = transport4.cp / transport4.cv
            m_dot_tot = self._calculate_leakage_flow(p4, h4, s3, p_suc, gamma4)

            self._state_cache[cache_key] = {
                "state_c_1": self.state_c_1,
                "state_c_4": self.state_c_4,
                "h4": h4,
                "gamma4": gamma4,
                "m_dot_tot": m_dot_tot,
            }
            if len(self._state_cache) > 1000:
                first_key = next(iter(self._state_cache))
                del self._state_cache[first_key]

        self._last_x = x.copy()

        if self.debug_enabled:
            if self._gamma4_min is None:
                self._gamma4_min = gamma4
                self._gamma4_max = gamma4
            else:
                self._gamma4_min = min(self._gamma4_min, gamma4)
                self._gamma4_max = max(self._gamma4_max, gamma4)
            self._gamma4_n += 1

        residuals = np.zeros(5)
        residuals[0] = self._residual_suction_heat_transfer(m_dot_suc, T_w, h1)
        residuals[1] = self._residual_discharge_valve_flow(m_dot_suc, p4, h4, s3, p_dis, gamma4)
        residuals[2] = self._residual_compressor_flow(m_dot_suc, rho3, m_dot_tot, inputs)
        residuals[3] = self._residual_mixing_energy(m_dot_suc, h1, h3, h4, rho3, m_dot_tot, inputs)
        residuals[4] = self._residual_overall_energy(m_dot_suc, T_w, h1, h4, h3, inputs, p_dis)
        return residuals

    def _calculate_leakage_flow(self, p4, h4, s3, p_suc, gamma4):
        p_thr_leak_critical = p4 * (2.0 / (gamma4 + 1.0)) ** (gamma4 / (gamma4 - 1.0))
        p_thr_leak = max(p_suc, p_thr_leak_critical)

        try:
            state_thr_leak = self.med_prop.calc_state("PS", p_thr_leak, s3)
            h_thr_leak = state_thr_leak.h
            rho_thr_leak = state_thr_leak.d
            delta_h = h4 - h_thr_leak
            if delta_h <= 0.0:
                return 0.0
            return rho_thr_leak * self.parameters["A_tot"] * np.sqrt(2.0 * delta_h)
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in leakage calculation: {err}")
            return 0.0

    def _calculate_ntu_effectiveness(self, Ua_ref, m_dot, cp, m_dot_ref):
        if m_dot < 1e-10 or cp < 1e-10:
            return 0.0, 0.0
        m_dot_ratio = m_dot / m_dot_ref
        NTU = (Ua_ref / (m_dot * cp)) * (m_dot_ratio ** 0.8)
        NTU = min(NTU, 100.0)
        epsilon = 1.0 - np.exp(-NTU)
        return NTU, epsilon

    def _residual_suction_heat_transfer(self, m_dot_suc, T_w, h1):
        h_suc = self.state_inlet.h
        T_suc = self.state_inlet.T
        transport_suc = self.med_prop.calc_transport_properties(self.state_inlet)
        cp_suc = transport_suc.cp

        _, epsilon_suc = self._calculate_ntu_effectiveness(
            self.parameters["Ua_suc_ref"],
            m_dot_suc,
            cp_suc,
            self.parameters["m_dot_ref"],
        )
        return h1 - h_suc - epsilon_suc * cp_suc * (T_w - T_suc)

    def _residual_discharge_valve_flow(self, m_dot_suc, p4, h4, s3, p_dis, gamma4):
        cache_key = (round(p4, 1), round(h4, 1), round(s3, 4), round(p_dis, 1), round(gamma4, 3))

        if cache_key in self._cached_discharge_valve:
            m_dot_valve = self._cached_discharge_valve[cache_key]
        else:
            p_thr_dis_critical = p4 * (2.0 / (gamma4 + 1.0)) ** (gamma4 / (gamma4 - 1.0))
            p_thr_dis = max(p_dis, p_thr_dis_critical)
            try:
                state_thr_dis = self.med_prop.calc_state("PS", p_thr_dis, s3)
                h_thr_dis = state_thr_dis.h
                rho_thr_dis = state_thr_dis.d
                delta_h = h4 - h_thr_dis
                if delta_h <= 0.0:
                    m_dot_valve = 0.0
                else:
                    m_dot_valve = rho_thr_dis * self.parameters["A_dis"] * np.sqrt(2.0 * delta_h)

                if len(self._cached_discharge_valve) > 500:
                    first_key = next(iter(self._cached_discharge_valve))
                    del self._cached_discharge_valve[first_key]
                self._cached_discharge_valve[cache_key] = m_dot_valve
            except Exception as err:
                if ENABLE_TIMING:
                    print(f"Warning in discharge valve calculation: {err}")
                m_dot_valve = 1e-6

        return m_dot_suc - m_dot_valve

    def _residual_compressor_flow(self, m_dot_suc, rho3, m_dot_tot, inputs):
        n_abs = self.get_n_absolute(inputs.control.n)
        m_dot_3 = rho3 * self.parameters["V_IC"] * n_abs
        return m_dot_3 - (m_dot_suc + m_dot_tot)

    def _residual_mixing_energy(self, m_dot_suc, h1, h3, h4, rho3, m_dot_tot, inputs):
        n_abs = self.get_n_absolute(inputs.control.n)
        m_dot_3 = rho3 * self.parameters["V_IC"] * n_abs

        energy_in = m_dot_suc * h1 + m_dot_tot * h4
        energy_out = m_dot_3 * h3
        residual = energy_out - energy_in

        if abs(m_dot_3) > 1e-10:
            residual = residual / m_dot_3
        return residual

    def _residual_overall_energy(self, m_dot_suc, T_w, h1, h4, h3, inputs, p_dis):
        h_suc = self.state_inlet.h
        T_amb = self._get_ambient_temperature(inputs)

        loss_data = self._calculate_viscosity_corrected_loss_power(
            m_dot_suc=m_dot_suc,
            T_w=T_w,
            h4=h4,
            h3=h3,
            p_dis=p_dis,
            inputs=inputs,
            use_exact_discharge_state=True,
        )

        W_dot_int = loss_data["W_dot_int"]
        W_dot_loss = loss_data["W_dot_loss"]
        h_dis = loss_data["h_dis"]

        Q_dot_suc = m_dot_suc * (h1 - h_suc)
        Q_dot_dis = m_dot_suc * (h4 - h_dis)
        Q_dot_amb = np.sign(T_w - T_amb) * self.parameters["Ua_amb"] * abs(T_w - T_amb) ** 1.25

        residual = W_dot_loss + Q_dot_dis - Q_dot_suc - Q_dot_amb
        if abs(W_dot_int) > 1e-10:
            residual = residual / W_dot_int
        return residual

    def _calculate_discharge_heat_transfer(self, m_dot_suc, T_w, h4, p_dis):
        try:
            state_before_cooling = self.med_prop.calc_state("PH", p_dis, h4)
            T_before_cooling = state_before_cooling.T
            transport5 = self.med_prop.calc_transport_properties(state_before_cooling)
            cp5 = transport5.cp

            _, epsilon_dis = self._calculate_ntu_effectiveness(
                self.parameters["Ua_dis_ref"],
                m_dot_suc,
                cp5,
                self.parameters["m_dot_ref"],
            )
            h_dis = h4 - epsilon_dis * cp5 * (T_before_cooling - T_w)
            T_dis_approx = T_before_cooling - epsilon_dis * (T_before_cooling - T_w)
            return h_dis, epsilon_dis, T_dis_approx
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in discharge heat transfer calculation: {err}")
            return h4, 0.0, T_w

    def simulate_operating_point(self, inputs, p_outlet, fs_state):
        self._clear_cache()

        initial_guess = self._get_initial_guesses(inputs, p_outlet)
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
            initial_guess,
            bounds=([b[0] for b in bounds], [b[1] for b in bounds]),
            method="trf",
            ftol=ftol,
            xtol=xtol,
            max_nfev=max_nfev,
        )

        if not result.success:
            print(f"  Solver warning: {result.message}")
            print("Trying to use solution anyway...")

        solution = result.x
        self._calculate_final_states(solution, inputs, p_outlet, fs_state)
        return solution

    def _calculate_final_states(self, solution, inputs, p_outlet, fs_state):
        m_dot_suc, T_w, h1, p4, h3 = solution
        p_suc = self.state_inlet.p
        p_dis = p_outlet

        self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
        h4 = self.state_c_4.h

        loss_data = self._calculate_viscosity_corrected_loss_power(
            m_dot_suc=m_dot_suc,
            T_w=T_w,
            h4=h4,
            h3=h3,
            p_dis=p_dis,
            inputs=inputs,
            use_exact_discharge_state=True,
        )

        self.state_c_5 = loss_data["state_dis"]
        self.state_outlet = self.state_c_5
        self.T_oil_sump = loss_data["T_oil_sump"]
        self.mu_oil = loss_data["mu_oil"]

        W_dot_int = loss_data["W_dot_int"]
        W_dot_loss = loss_data["W_dot_loss"]
        self.P_el = W_dot_int + W_dot_loss
        self.W_dot_comp = self.P_el
        self.m_flow = m_dot_suc
        self.T_w = T_w

        fs_state.set("m_flow", self.m_flow, "kg/s", "Refrigerant mass flow rate")
        fs_state.set("P_el", self.P_el, "W", "Electrical power input")
        fs_state.set("W_dot_int", W_dot_int, "W", "Internal compression power")
        fs_state.set("W_dot_loss", W_dot_loss, "W", "Compressor loss power")
        fs_state.set("T_wall", T_w, "K", "Wall temperature")
        fs_state.set("T_dis", self.state_c_5.T, "K", "Discharge temperature")
        fs_state.set("T_oil_sump", self.T_oil_sump, "K", "Oil sump temperature")
        fs_state.set("mu_oil", self.mu_oil, "same-as-lubricant-fitting", "Dynamic oil viscosity")
        fs_state.set("pc4", p4, "Pa", "Internal discharge pressure")
        fs_state.set("hc1", h1, "J/kg", "Enthalpy after suction heat transfer")
        fs_state.set("hc3", h3, "J/kg", "Enthalpy after mixing")
        fs_state.set("hc4", h4, "J/kg", "Enthalpy after compression")
        fs_state.set("p_2", self.state_outlet.p, "Pa", "Outlet pressure")
        fs_state.set("p_1", self.state_inlet.p, "Pa", "Inlet pressure")
        fs_state.set("T_1", self.state_inlet.T, "K", "Inlet temperature")

    def get_eta_mech(self, inputs: Inputs) -> float:
        if self.W_dot_comp is None or self.W_dot_comp <= 0.0:
            return 0.0
        if (
            self.state_c_3 is None
            or self.state_c_4 is None
            or self.state_c_3.d is None
            or self.state_c_4.h is None
            or self.state_c_3.h is None
        ):
            return 0.0

        n_abs = self.get_n_absolute(inputs.control.n)
        rho3 = self.state_c_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * n_abs
        W_dot_int = m_dot_3 * (self.state_c_4.h - self.state_c_3.h)

        if W_dot_int <= 0.0:
            return 0.0

        eta_mech = W_dot_int / self.W_dot_comp
        return max(0.1, min(0.95, eta_mech))

    def get_lambda_h(self, inputs: Inputs) -> float:
        if (
            self.m_flow is None
            or self.state_inlet is None
            or self.state_inlet.d is None
            or self.m_flow <= 0.0
        ):
            return 0.0

        rho_suc = self.state_inlet.d
        n_abs = self.get_n_absolute(inputs.control.n)
        m_dot_theoretical = rho_suc * self.V_h * n_abs

        if m_dot_theoretical <= 0.0:
            return 0.0

        lambda_h = self.m_flow / m_dot_theoretical
        return max(0.5, min(0.98, lambda_h))

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState) -> float:
        if (
            self.state_inlet.T != fs_state.T_1
            or p_outlet != fs_state.p_2
            or self.state_inlet.p != fs_state.p_1
        ):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

        try:
            h_suc = self.state_inlet.h
            s_suc = self.state_inlet.s
            h_dis_actual = self.state_outlet.h

            if h_dis_actual <= h_suc:
                return 0.0

            state_dis_isen = self.med_prop.calc_state("PS", p_outlet, s_suc)
            h_dis_isen = state_dis_isen.h
            if h_dis_isen <= h_suc:
                return 0.0

            w_is = h_dis_isen - h_suc
            w_actual = h_dis_actual - h_suc
            if w_actual <= 0.0:
                return 0.0

            eta_is = w_is / w_actual
            if eta_is > 1.0 and ENABLE_TIMING:
                print(f"  Warning: Isentropic efficiency > 1.0: {eta_is:.3f}")
            return max(0.0, min(1.0, eta_is))
        except Exception as err:
            if ENABLE_TIMING:
                print(f"Warning in overall isentropic efficiency calculation: {err}")
            return 0.7

    def calc_state_outlet(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState):
        self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

    def calc_m_flow(self, p_outlet, inputs: Inputs, fs_state: FlowsheetState) -> float:
        if (
            self.state_inlet.T != fs_state.T_1
            or p_outlet != fs_state.p_2
            or self.state_inlet.p != fs_state.p_1
        ):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        return self.m_flow

    def calc_electrical_power(self, p_outlet, inputs: Inputs, fs_state: FlowsheetState) -> float:
        if (
            self.state_inlet.T != fs_state.T_1
            or p_outlet != fs_state.p_2
            or self.state_inlet.p != fs_state.p_1
        ):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)
        return self.P_el

    def get_debug_report(self) -> str:
        if not self.debug_enabled or self._gamma4_n == 0:
            return "Debug disabled (or no solver calls recorded)."
        return (
            f"Molinaroli2017 Debug Report\n"
            f"  gamma4 range: {self._gamma4_min:.6f} .. {self._gamma4_max:.6f}  (N={self._gamma4_n})\n"
            f"  state_cache: hits={self._cache_hits}, misses={self._cache_misses}, size={len(self._state_cache)}\n"
            f"  discharge_valve_cache size={len(self._cached_discharge_valve)}"
        )
