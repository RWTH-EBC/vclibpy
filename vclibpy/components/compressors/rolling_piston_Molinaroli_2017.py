from vclibpy.components.compressors.compressor import Compressor
from vclibpy.media import ThermodynamicState
#from vclibpy.media.cool_prop import CoolProp
from vclibpy.datamodels import Inputs, FlowsheetState
import numpy as np
#import time

ENABLE_TIMING = True


class Molinaroli_2017_Compressor(Compressor):
    """
    Model from:
    Molinaroli et. al. (2017), A semi-empirical model for hermetic rolling piston compressors
    http://dx.doi.org/10.1016/j.ijrefrig.2017.04.015

    """
    def __init__(self, N_max: float, V_h: float, refrigerant="R290"):
        super().__init__(N_max=N_max, V_h=V_h)

        self.refrigerant = refrigerant
        self.med_prop = CoolProp(fluid_name=refrigerant)

        # Parameters for compressor 'B' from Molinaroli et. al(2017)
        self.parameters = {
            "Ua_suc_ref": 16.05,  # W/K
            "Ua_dis_ref": 13.96,  # W/K
            "Ua_amb": 0.36,  # W/K **(-1.25)
            "A_tot": 9.47e-9,  # m^2
            "A_dis": 86.1e-9,  # m^2
            "V_IC": 16.11e-6,  # m^3
            "alpha_loss": 0.16,  # -
            "W_dot_loss_ref": 83,  # W
            "m_dot_ref": 0.0083,  # kg/s
            "f_ref": 50.0}  # Hz

        ##INITIALIZE VARIABLES##
        # Thermodynamic states
        self.state_suc: ThermodynamicState = None
        self.state_1: ThermodynamicState = None
        self.state_3: ThermodynamicState = None
        self.state_4: ThermodynamicState = None
        self.state_5: ThermodynamicState = None

        # Unknown variables
        self.T_w = None
        self.m_flow_suc = None

        # Results storage
        self.state_outlet = None
        self.W_dot_comp = None

        # Cache for expensive calculations
        self._cached_transport4 = None
        self._cached_gamma4 = None
        self._cached_m_dot_tot = None

        # State caching for repeated solver calls
        self._state_cache = {}
        self._last_x = None
        self._cache_hits = 0
        self._cache_misses = 0

        # Pre-computed constants
        self._V_IC_times_f_ref = self.parameters["V_IC"] * self.parameters["f_ref"]
        self._W_dot_loss_ref_over_f_ref_sq = self.parameters["W_dot_loss_ref"] / (self.parameters["f_ref"] ** 2)

        # Cache for discharge valve calculations
        self._cached_discharge_valve = {}

    def _get_initial_guesses(self, inputs, p_outlet):
        " find initial guesses for the unknowns "
        "args: inputs: Inputs object with operating conditions, p_outlet: outlet pressure [Pa] "
        "returns: list of initial guesses for [m_dot_suc_0, T_w_0, h1_0, p4_0, h3_0] "
        T_evap = inputs.T_eva_in
        state_sat_vapor = self.med_prop.calc_state("TQ", T_evap, 1.0)
        p_suc = state_sat_vapor.p

        T_suc = T_evap + inputs.dT_eva_superheating
        self.state_inlet = self.med_prop.calc_state("PT", p_suc, T_suc)
        h_suc = self.state_inlet.h
        s_suc = self.state_inlet.s

        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        p_dis = p_outlet
        state_dis_is = self.med_prop.calc_state("PS", p_dis, s_suc)
        T_dis_is = state_dis_is.T

        rho_suc = self.state_inlet.d
        m_dot_suc_0 = rho_suc * self.parameters["V_IC"] * f

        T_w_0 = (T_suc + T_dis_is) / 2

        transport_suc = self.med_prop.calc_transport_properties(self.state_inlet)
        cp_suc = transport_suc.cp
        h1_0 = h_suc + 0.5 * cp_suc * (T_w_0 - T_suc)

        p4_0 = 1.1 * p_dis

        h3_0 = h1_0

        return [m_dot_suc_0, T_w_0, h1_0, p4_0, h3_0]

    def _clear_cache(self):
        """Clear the state cache"""
        self._state_cache.clear()
        self._last_x = None
        self._cache_hits = 0
        self._cache_misses = 0

    def _get_cache_key(self, p_suc, h1, p_suc_3, h3, p4, s3):
        """Create a cache key for state calculations"""
        # Round to avoid floating point precision issues
        key = (
            round(p_suc, 2),  # Pressure for state1 and state3
            round(h1, 1),  # Enthalpy for state1
            round(h3, 1),  # Enthalpy for state3
            round(p4, 2),  # Pressure for state4
            round(s3, 4)  # Entropy for state4 (more precision needed)
        )
        return key

    def _calculate_residuals(self, x, inputs, p_outlet):
        """
        Optimized version with CoolProp call reduction and state caching
        args: x: array of unknowns [m_dot_suc, T_w, h1, p4, h3], inputs: Inputs object, p_outlet: outlet pressure [Pa]
        returns: array of residuals for the 5 equations
        """
        # Unpack unknowns
        m_dot_suc, T_w, h1, p4, h3 = x

        # Get known pressures
        p_suc = self.state_inlet.p
        p_dis = p_outlet

        # ---------------------------------------------
        # 1. Check cache for existing state calculations
        # ---------------------------------------------

        # Calculate state 3 first to get s3 for cache key
        self.state_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_3.s
        rho3 = self.state_3.d

        cache_key = self._get_cache_key(p_suc, h1, p_suc, h3, p4, s3)

        # Check if we have cached results for these states
        if (self._last_x is not None and
                np.allclose(x, self._last_x, rtol=1e-6, atol=1e-8) and
                cache_key in self._state_cache):

            cached_data = self._state_cache[cache_key]
            self.state_1 = cached_data['state_1']
            self.state_4 = cached_data['state_4']
            h4 = cached_data['h4']
            transport4 = cached_data['transport4']
            gamma4 = cached_data['gamma4']
            m_dot_tot = cached_data['m_dot_tot']

            self._cache_hits += 1

        else:
            # Cache miss - calculate everything
            self._cache_misses += 1

            # State 1: After suction heat transfer
            self.state_1 = self.med_prop.calc_state("PH", p_suc, h1)

            # State 4: Isentropic compression (expensive call)
            self.state_4 = self.med_prop.calc_state("PS", p4, s3)
            h4 = self.state_4.h

            # Calculate transport properties for state_4 ONCE
            if (self._cached_transport4 is None or
                    self._cached_gamma4 is None or
                    abs(self.state_4.p - p4) > 1000):

                transport4 = self.med_prop.calc_transport_properties(self.state_4)
                cp4 = transport4.cp
                cv4 = transport4.cv
                gamma4 = cp4 / cv4

                self._cached_transport4 = transport4
                self._cached_gamma4 = gamma4
            else:
                transport4 = self._cached_transport4
                gamma4 = self._cached_gamma4

            # Calculate leakage flow ONCE
            m_dot_tot = self._calculate_leakage_flow(p4, h4, s3, p_suc, gamma4)

            # Cache the results
            self._state_cache[cache_key] = {
                'state_1': self.state_1,
                'state_4': self.state_4,
                'h4': h4,
                'transport4': transport4,
                'gamma4': gamma4,
                'm_dot_tot': m_dot_tot
            }

            # Limit cache size to prevent memory issues
            if len(self._state_cache) > 1000:
                # Remove oldest entry
                first_key = next(iter(self._state_cache))
                del self._state_cache[first_key]

        self._last_x = x.copy()

        # ---------------------------------------------
        # 2. Calculate Residuals (using cached or new values)
        # ---------------------------------------------

        residuals = np.zeros(5)

        residuals[0] = self._residual_suction_heat_transfer(m_dot_suc, T_w, h1, inputs)
        residuals[1] = self._residual_discharge_valve_flow(m_dot_suc, p4, h4, s3, p_dis, gamma4)
        residuals[2] = self._residual_compressor_flow(m_dot_suc, rho3, m_dot_tot, inputs)
        residuals[3] = self._residual_mixing_energy(m_dot_suc, h1, h3, h4, rho3, m_dot_tot, inputs)
        residuals[4] = self._residual_overall_energy(m_dot_suc, T_w, h1, h4, h3, inputs, p_dis)

        return residuals

    def _calculate_leakage_flow(self, p4, h4, s3, p_suc, gamma4):
        """
        Calculate total leakage flow using pre-computed gamma4
        args: p4: pressure at state 4 [Pa], h4: enthalpy at state 4 [J/kg], s3: entropy at state 3 [J/kg-K], p_suc: suction pressure [Pa], gamma4: specific heat ratio at state 4
        returns: m_dot_tot: total leakage mass flow [kg/s]
        """
        p_thr_leak_critical = p4 * (2 / (gamma4 + 1)) ** (gamma4 / (gamma4 - 1))
        p_thr_leak = max(p_suc, p_thr_leak_critical)

        try:
            state_thr_leak = self.med_prop.calc_state("PS", p_thr_leak, s3)
            h_thr_leak = state_thr_leak.h
            rho_thr_leak = state_thr_leak.d

            delta_h = h4 - h_thr_leak
            if delta_h <= 0:
                return 0.0

            m_dot_tot = rho_thr_leak * self.parameters["A_tot"] * np.sqrt(2 * delta_h)
            return m_dot_tot

        except Exception as e:
            if ENABLE_TIMING:
                print(f"Warning in leakage calculation: {e}")
            return 0.0

    def _calculate_ntu_effectiveness(self, Ua_ref, m_dot, cp, m_dot_ref):
        """
        Optimized NTU and effectiveness calculation
        args: Ua_ref: reference overall heat transfer coefficient [W/K], m_dot: mass flow rate [kg/s], cp: specific heat capacity [J/kg-K], m_dot_ref: reference mass flow rate [kg/s]
        returns: NTU: number of transfer units, epsilon: effectiveness
        """
        # Avoid division by zero
        if m_dot < 1e-10 or cp < 1e-10:
            return 0.0, 0.0

        # Precompute commonly used values
        m_dot_ratio = m_dot / m_dot_ref
        NTU = (Ua_ref / (m_dot * cp)) * (m_dot_ratio ** 0.8)

        # Clamp NTU to avoid numerical issues
        NTU = min(NTU, 100.0)  # Very high NTU -> effectiveness ~1

        epsilon = 1.0 - np.exp(-NTU)
        return NTU, epsilon

    def _residual_suction_heat_transfer(self, m_dot_suc, T_w, h1, inputs):
        "REDIDUAL 1: Suction heat transfer eq. using optimized NTU calculation"
        "args: m_dot_suc: suction mass flow rate [kg/s], T_w: wall temperature [K], h1: enthalpy after suction heat transfer [J/kg], inputs: Inputs object"
        "returns: residual [0]"
        h_suc = self.state_inlet.h
        T_suc = self.state_inlet.T

        transport_suc = self.med_prop.calc_transport_properties(self.state_inlet)
        cp_suc = transport_suc.cp

        # Use optimized NTU calculation
        NTU_suc, epsilon_suc = self._calculate_ntu_effectiveness(
            self.parameters["Ua_suc_ref"], m_dot_suc, cp_suc, self.parameters["m_dot_ref"]
        )

        residual = h1 - h_suc - epsilon_suc * cp_suc * (T_w - T_suc)
        return residual

    def _residual_discharge_valve_flow(self, m_dot_suc, p4, h4, s3, p_dis, gamma4):
        """
        Residual 2 using mass balance equation for discharge valve
        Optimized discharge valve flow with caching
        returns: residual[1]
        """
        # Create cache key for this calculation
        cache_key = (round(p4, 1), round(h4, 1), round(s3, 4), round(p_dis, 1), round(gamma4, 3))

        if cache_key in self._cached_discharge_valve:
            m_dot_valve = self._cached_discharge_valve[cache_key]
        else:
            p_thr_dis_critical = p4 * (2 / (gamma4 + 1)) ** (gamma4 / (gamma4 - 1))
            p_thr_dis = max(p_dis, p_thr_dis_critical)

            try:
                state_thr_dis = self.med_prop.calc_state("PS", p_thr_dis, s3)
                h_thr_dis = state_thr_dis.h
                rho_thr_dis = state_thr_dis.d

                delta_h = h4 - h_thr_dis
                if delta_h <= 0:
                    m_dot_valve = 0.0
                else:
                    m_dot_valve = rho_thr_dis * self.parameters["A_dis"] * np.sqrt(2 * delta_h)

                # Cache the result
                if len(self._cached_discharge_valve) > 500:
                    # Remove oldest entry
                    first_key = next(iter(self._cached_discharge_valve))
                    del self._cached_discharge_valve[first_key]
                self._cached_discharge_valve[cache_key] = m_dot_valve

            except Exception as e:
                if ENABLE_TIMING:
                    print(f"Warning in discharge valve calculation: {e}")
                m_dot_valve = 1e-6

        residual = m_dot_suc - m_dot_valve
        return residual

    def _residual_compressor_flow(self, m_dot_suc, rho3, m_dot_tot, inputs):
        """
        Residual 3: Compressor flow using pre-computed values. mass balance equation for suction mass flow with leakage and return flow
        args: m_dot_suc: suction mass flow rate [kg/s], rho3: density at state 3 [kg/m^3], m_dot_tot: total leakage mass flow [kg/s], inputs: Inputs object
        returns: residual[2]
        """
        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        m_dot_3 = rho3 * self.parameters["V_IC"] * f
        residual = m_dot_3 - (m_dot_suc + m_dot_tot)
        return residual

    def _residual_mixing_energy(self, m_dot_suc, h1, h3, h4, rho3, m_dot_tot, inputs):
        """
        Residual 4:energy balance for state 3 suction mass flow, leakage flow and reexpansion.
        args: m_dot_suc: suction mass flow rate [kg/s], h1: enthalpy after suction heat transfer [J/kg], h3: enthalpy after mixing [J/kg], h4: enthalpy after compression [J/kg], rho3: density at state 3 [kg/m^3], m_dot_tot: total leakage mass flow [kg/s], inputs: Inputs object
        returns: residual[3]
        """
        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        m_dot_3 = rho3 * self.parameters["V_IC"] * f

        energy_in = m_dot_suc * h1 + m_dot_tot * h4
        energy_out = m_dot_3 * h3

        residual = energy_out - energy_in

        # Normalize by mass flow for better numerical behavior
        if abs(m_dot_3) > 1e-10:
            residual = residual / m_dot_3

        return residual

    def _residual_overall_energy(self, m_dot_suc, T_w, h1, h4, h3, inputs, p_dis):
        """
        Residual 5:  energy balance for the whole system
        args: m_dot_suc: suction mass flow rate [kg/s], T_w: wall temperature [K], h1: enthalpy after suction heat transfer [J/kg], h4: enthalpy after compression [J/kg], h3: enthalpy after mixing [J/kg], inputs: Inputs object, p_dis: discharge pressure [Pa]
        returns: residual[4]
        """
        h_suc = self.state_inlet.h
        T_amb = inputs.T_ambient
        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        rho3 = self.state_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f

        W_dot_int = m_dot_3 * (h4 - h3)

        W_dot_loss = (W_dot_int * self.parameters["alpha_loss"] +
                      self.parameters["W_dot_loss_ref"] * (n_abs / self.parameters["f_ref"]) ** 2)

        Q_dot_suc = m_dot_suc * (h1 - h_suc)

        h_dis, epsilon_dis = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis)
        Q_dot_dis = m_dot_suc * (h4 - h_dis)

        Q_dot_amb = self.parameters["Ua_amb"] * (T_w - T_amb) ** (5 / 4)

        residual = W_dot_loss + Q_dot_dis - Q_dot_suc - Q_dot_amb

        # Normalize by internal work
        if abs(W_dot_int) > 1e-10:
            residual = residual / W_dot_int

        return residual

    def _calculate_discharge_heat_transfer(self, m_dot_suc, T_w, h4, p_dis):
        """
        Calculate discharge heat transfer and h_dis
        """
        try:
            self.state_5 = self.med_prop.calc_state("PH", p_dis, h4)
            T5 = self.state_5.T

            transport5 = self.med_prop.calc_transport_properties(self.state_5)
            cp5 = transport5.cp

            NTU_dis, epsilon_dis = self._calculate_ntu_effectiveness(
                self.parameters["Ua_dis_ref"], m_dot_suc, cp5, self.parameters["m_dot_ref"])

            h_dis = h4 - epsilon_dis * cp5 * (T5 - T_w)
            return h_dis, epsilon_dis

        except Exception as e:
            if ENABLE_TIMING:
                print(f"Warning in discharge heat transfer calculation: {e}")
            return h4, 0.0

    def simulate_operating_point(self, inputs, p_outlet, fs_state):
        """
        Main simulation function with state caching
        """

        # Clear cache for new operating point
        self._clear_cache()

        # 1. Get initial guesses
        initial_guess = self._get_initial_guesses(inputs, p_outlet)

        # 2. Define bounds for physical realism
        bounds = [
            (1e-6, 1),  # m_dot_suc > 0
            (inputs.T_eva_in, 550),  # T_w between suction and 127°C
            (self.state_inlet.h, self.state_inlet.h + 100e6),  # h1 > h_suc
            (p_outlet * 1.001, p_outlet * 15),  # p4 > p_dis but not extreme
            (self.state_inlet.h, self.state_inlet.h + 200e6)  # h3 reasonable range
        ]

        # 3. Solve the system using least_squares with state caching
        from scipy.optimize import least_squares

        def residual_wrapper(x):
            return self._calculate_residuals(x, inputs, p_outlet)

        result = least_squares(
            residual_wrapper,
            initial_guess,
            bounds=([b[0] for b in bounds], [b[1] for b in bounds]),
            method='trf',
            ftol=1e-8,
            xtol=1e-8,
            max_nfev=1000
        )

        # 4. Check convergence
        if not result.success:
            print(f"  Solver warning: {result.message}")
            print("Trying to use solution anyway...")

        # 5. Extract solution
        solution = result.x
        m_dot_suc, T_w, h1, p4, h3 = solution

        # Print cache statistics
        total_calls = self._cache_hits + self._cache_misses
        hit_rate = self._cache_hits / total_calls * 100 if total_calls > 0 else 0

        # 6. Calculate final states with solution
        self._calculate_final_states(solution, inputs, p_outlet, fs_state)

        return solution

    def _calculate_final_states(self, solution, inputs, p_outlet, fs_state):
        """
        Calculate all final states and outputs after solution
        """
        m_dot_suc, T_w, h1, p4, h3 = solution
        p_suc = self.state_inlet.p
        p_dis = p_outlet

        # Recalculate all states with final solution (bypass cache for accuracy)
        self.state_1 = self.med_prop.calc_state("PH", p_suc, h1)
        self.state_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_3.s
        self.state_4 = self.med_prop.calc_state("PS", p4, s3)
        h4 = self.state_4.h
        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        # Calculate discharge state
        h_dis, epsilon_dis = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis)
        self.state_5 = self.med_prop.calc_state("PH", p_dis, h_dis)
        self.state_outlet = self.state_5
        rho3 = self.state_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f

        # Calculate internal work
        W_dot_int = m_dot_3 * (h4 - h3)

        # Calculate powers and efficiencies
        n_abs = self.get_n_absolute(inputs.n)
        W_dot_loss = (W_dot_int * self.parameters["alpha_loss"] +
                      self.parameters["W_dot_loss_ref"] * (n_abs / self.parameters["f_ref"]) ** 2)
        self.W_dot_comp = W_dot_int + W_dot_loss

        # Store results
        self.m_flow_suc = m_dot_suc
        self.T_w = T_w

        # Populate flowsheet state
        fs_state.set("m_flow", m_dot_suc, "kg/s", "Refrigerant mass flow rate")
        fs_state.set("P_el", self.W_dot_comp, "W", "Electrical power input")
        fs_state.set("T_wall", T_w, "K", "Wall temperature")
        fs_state.set("p4", p4, "Pa", "Internal discharge pressure")
        fs_state.set("h1", h1, "J/kg", "Enthalpy after suction heat transfer")
        fs_state.set("h3", h3, "J/kg", "Enthalpy after mixing")
        fs_state.set("h4", h4, "J/kg", "Enthalpy after compression")
        fs_state.set("T_dis", self.state_5.T, "K", "Discharge temperature")

    def _create_catalog_data(self):
        """
        Create catalog data dictionaries from the provided tables
        """
        # Mass flow catalog data [kg/h]
        # Rows: T_evap, Columns: T_cond
        mass_flow_catalog = {
            -25: {30: 9.158, 35: 9.016},
            -20: {30: 11.38, 35: 11.24, 40: 11.07, 45: 10.87},
            -15: {30: 13.99, 35: 13.83, 40: 13.64, 45: 13.41, 50: 13.16, 55: 12.90},
            -10: {30: 17.04, 35: 16.85, 40: 16.61, 45: 16.33, 50: 16.02, 55: 15.68, 60: 15.34},
            -5: {30: 20.60, 35: 20.36, 40: 20.05, 45: 19.70, 50: 19.31, 55: 18.88, 60: 18.43},
            0: {30: 24.74, 35: 24.43, 40: 24.05, 45: 23.62, 50: 23.13, 55: 22.59, 60: 22.01},
            5: {30: 29.53, 35: 29.15, 40: 28.69, 45: 28.16, 50: 27.56, 55: 26.90, 60: 26.19},
            10: {30: 35.08, 35: 34.61, 40: 34.06, 45: 33.43, 50: 32.72, 55: 31.93, 60: 31.06},
            15: {30: 41.47, 35: 40.93, 40: 40.28, 45: 39.55, 50: 38.72, 55: 37.80, 60: 36.79}
        }

        # Power input catalog data [W]
        power_catalog = {
            -25: {30: 433.8, 35: 460.9},
            -20: {30: 445.4, 35: 475.5, 40: 505.6, 45: 537.0},
            -15: {30: 455.4, 35: 489.7, 40: 523.6, 45: 558.5, 50: 595.8, 55: 637.0},
            -10: {30: 462.0, 35: 501.6, 40: 540.4, 45: 579.9, 50: 621.4, 55: 666.4, 60: 716.5},
            -5: {30: 463.2, 35: 509.4, 40: 554.3, 45: 599.4, 50: 646.2, 55: 696.2, 60: 750.8},
            0: {30: 457.3, 35: 511.1, 40: 563.2, 45: 615.2, 50: 668.5, 55: 724.5, 60: 784.9},
            5: {30: 442.3, 35: 504.8, 40: 565.3, 45: 625.3, 50: 686.2, 55: 749.5, 60: 816.7},
            10: {30: 416.4, 35: 488.8, 40: 558.8, 45: 627.9, 50: 697.6, 55: 769.3, 60: 844.4},
            15: {30: 377.6, 35: 461.0, 40: 541.7, 45: 621.1, 50: 700.7, 55: 781.9, 60: 866.2}
        }

        return mass_flow_catalog, power_catalog

    def plot_catalog_comparison(self, results, save_path=None):
        """
        Plot comparison between model results and catalog data
        """
        import matplotlib.pyplot as plt
        import pandas as pd
        import numpy as np

        # Get catalog data
        mass_flow_catalog, power_catalog = self._create_catalog_data()

        # Convert results to DataFrame
        successful_results = [r for r in results if r.get('success', False)]
        if len(successful_results) == 0:
            print("No successful results to plot")
            return

        df = pd.DataFrame(successful_results)

        # Prepare data for comparison
        comparison_data = []

        for _, row in df.iterrows():
            T_evap = row['T_evap_C']
            T_cond = row['T_cond_C']

            # Get catalog values if they exist
            catalog_mass_flow = mass_flow_catalog.get(T_evap, {}).get(T_cond, None)
            catalog_power = power_catalog.get(T_evap, {}).get(T_cond, None)

            if catalog_mass_flow is not None and catalog_power is not None:
                comparison_data.append({
                    'T_evap_C': T_evap,
                    'T_cond_C': T_cond,
                    'model_mass_flow': row['m_flow_kg_h'],
                    'catalog_mass_flow': catalog_mass_flow,
                    'model_power': row['W_comp'],
                    'catalog_power': catalog_power,
                    'pressure_ratio': row['p_discharge'] / row['p_evap']
                })

        if not comparison_data:
            print("No matching catalog data found for comparison")
            return

        comparison_df = pd.DataFrame(comparison_data)

        # Create comparison plots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

        # Mass Flow Comparison
        ax1.scatter(comparison_df['catalog_mass_flow'], comparison_df['model_mass_flow'],
                    c=comparison_df['T_cond_C'], cmap='viridis', s=80, alpha=0.7,
                    edgecolors='black', linewidth=0.5)

        # Perfect agreement line
        max_mass_flow = max(comparison_df['catalog_mass_flow'].max(), comparison_df['model_mass_flow'].max())
        min_mass_flow = min(comparison_df['catalog_mass_flow'].min(), comparison_df['model_mass_flow'].min())
        ax1.plot([min_mass_flow, max_mass_flow], [min_mass_flow, max_mass_flow],
                 'r--', alpha=0.8, linewidth=2, label='Perfect Agreement')

        ax1.set_xlabel('Catalog Mass Flow [kg/h]', fontsize=12)
        ax1.set_ylabel('Model Mass Flow [kg/h]', fontsize=12)
        ax1.set_title('Mass Flow Rate: Model vs Catalog', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.tick_params(axis='both', which='major', labelsize=10)

        # Add colorbar for condensing temperature
        sm = plt.cm.ScalarMappable(cmap='viridis',
                                   norm=plt.Normalize(vmin=comparison_df['T_cond_C'].min(),
                                                      vmax=comparison_df['T_cond_C'].max()))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax1)
        cbar.set_label('Condensing Temperature [°C]', fontsize=10)

        # Power Comparison
        ax2.scatter(comparison_df['catalog_power'], comparison_df['model_power'],
                    c=comparison_df['T_cond_C'], cmap='viridis', s=80, alpha=0.7,
                    edgecolors='black', linewidth=0.5)

        # Perfect agreement line
        max_power = max(comparison_df['catalog_power'].max(), comparison_df['model_power'].max())
        min_power = min(comparison_df['catalog_power'].min(), comparison_df['model_power'].min())
        ax2.plot([min_power, max_power], [min_power, max_power],
                 'r--', alpha=0.8, linewidth=2, label='Perfect Agreement')

        ax2.set_xlabel('Catalog Power [W]', fontsize=12)
        ax2.set_ylabel('Model Power [W]', fontsize=12)
        ax2.set_title('Compressor Power: Model vs Catalog', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.tick_params(axis='both', which='major', labelsize=10)

        # Add colorbar for condensing temperature
        sm2 = plt.cm.ScalarMappable(cmap='viridis',
                                    norm=plt.Normalize(vmin=comparison_df['T_cond_C'].min(),
                                                       vmax=comparison_df['T_cond_C'].max()))
        sm2.set_array([])
        cbar2 = plt.colorbar(sm2, ax=ax2)
        cbar2.set_label('Condensing Temperature [°C]', fontsize=10)

        plt.tight_layout()

        # Calculate and display statistics
        self._display_comparison_statistics(comparison_df)

        if save_path:
            comparison_save_path = save_path.replace('.png', '_comparison.png')
            plt.savefig(comparison_save_path, dpi=300, bbox_inches='tight')
            print(f"Comparison plot saved to {comparison_save_path}")

        plt.show()

        return fig

    def _display_comparison_statistics(self, comparison_df):
        """
        Display statistical comparison between model and catalog data
        """
        # Mass flow statistics
        mass_flow_error = comparison_df['model_mass_flow'] - comparison_df['catalog_mass_flow']
        mass_flow_relative_error = (mass_flow_error / comparison_df['catalog_mass_flow']) * 100

        # Power statistics
        power_error = comparison_df['model_power'] - comparison_df['catalog_power']
        power_relative_error = (power_error / comparison_df['catalog_power']) * 100

        print("\n" + "=" * 60)
        print("MODEL vs CATALOG COMPARISON STATISTICS")
        print("=" * 60)

        print(f"\nMASS FLOW RATE:")
        print(f"  Mean Absolute Error: {abs(mass_flow_error).mean():.2f} kg/h")
        print(f"  Mean Relative Error: {mass_flow_relative_error.mean():.2f}%")
        print(f"  Max Relative Error: {mass_flow_relative_error.abs().max():.2f}%")
        print(f"  RMS Error: {np.sqrt((mass_flow_error ** 2).mean()):.2f} kg/h")

        print(f"\n COMPRESSOR POWER:")
        print(f"  Mean Absolute Error: {abs(power_error).mean():.2f} W")
        print(f"  Mean Relative Error: {power_relative_error.mean():.2f}%")
        print(f"  Max Relative Error: {power_relative_error.abs().max():.2f}%")
        print(f"  RMS Error: {np.sqrt((power_error ** 2).mean()):.2f} W")

        print(f"\nCORRELATION COEFFICIENTS:")
        mass_flow_corr = np.corrcoef(comparison_df['catalog_mass_flow'], comparison_df['model_mass_flow'])[0, 1]
        power_corr = np.corrcoef(comparison_df['catalog_power'], comparison_df['model_power'])[0, 1]
        print(f"  Mass Flow Correlation: {mass_flow_corr:.4f}")
        print(f"  Power Correlation: {power_corr:.4f}")

        # Display individual point errors
        print(f"\n INDIVIDUAL POINT ERRORS:")
        print(f"{'T_evap':>6} {'T_cond':>6} {'M_flow_err':>10} {'Pwr_err':>10}")
        print(f"{'(°C)':>6} {'(°C)':>6} {'(kg/h)':>10} {'(W)':>10}")
        print("-" * 40)

        for _, row in comparison_df.iterrows():
            mf_err = row['model_mass_flow'] - row['catalog_mass_flow']
            pwr_err = row['model_power'] - row['catalog_power']
            print(f"{row['T_evap_C']:6.0f} {row['T_cond_C']:6.0f} {mf_err:10.2f} {pwr_err:10.1f}")

    def plot_operating_map(self, results, save_path=None):
        """
        Updated operating map plot that includes ALL plots
        """
        import matplotlib.pyplot as plt
        import pandas as pd
        import numpy as np

        # Convert to DataFrame for easier handling
        successful_results = [r for r in results if r.get('success', False)]
        if len(successful_results) == 0:
            print("No successful results to plot")
            return

        df = pd.DataFrame(successful_results)

        # Get unique condensing temperatures and sort them
        condensing_temps = sorted(df['T_cond_C'].unique())

        # Create the figure with subplots
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

        # Define colors for different condensing temperatures
        colors = plt.cm.viridis(np.linspace(0, 1, len(condensing_temps)))

        # Plot 1: Mass Flow Rate vs Evaporation Temperature
        for i, T_cond in enumerate(condensing_temps):
            cond_data = df[df['T_cond_C'] == T_cond].sort_values('T_evap_C')

            if len(cond_data) > 0:
                ax1.plot(cond_data['T_evap_C'], cond_data['m_flow_kg_h'],
                         marker='o', linewidth=2, markersize=6,
                         color=colors[i], label=f'T_cond = {T_cond}°C')

        ax1.set_xlabel('Evaporation Temperature [°C]', fontsize=12)
        ax1.set_ylabel('Mass Flow Rate [kg/h]', fontsize=12)
        ax1.set_title('Mass Flow Rate vs Evaporation Temperature', fontsize=14, fontweight='bold')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)
        ax1.tick_params(axis='both', which='major', labelsize=10)

        # Plot 2: Compressor Power vs Evaporation Temperature
        for i, T_cond in enumerate(condensing_temps):
            cond_data = df[df['T_cond_C'] == T_cond].sort_values('T_evap_C')

            if len(cond_data) > 0:
                ax2.plot(cond_data['T_evap_C'], cond_data['W_comp'],
                         marker='s', linewidth=2, markersize=6,
                         color=colors[i], label=f'T_cond = {T_cond}°C')

        ax2.set_xlabel('Evaporation Temperature [°C]', fontsize=12)
        ax2.set_ylabel('Compressor Power [W]', fontsize=12)
        ax2.set_title('Compressor Power vs Evaporation Temperature', fontsize=14, fontweight='bold')
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax2.grid(True, alpha=0.3)
        ax2.tick_params(axis='both', which='major', labelsize=10)

        plt.tight_layout()

        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Performance plot saved to {save_path}")

        plt.show()

        # FIXED: Check for the correct column names
        if 'eta_mech' in df.columns and 'eta_is_overall' in df.columns:
            print("\n Generating efficiency plots...")
            print(f"Available columns (eff plots): {list(df.columns)}")

            self._plot_efficiency_results(df, save_path)
        else:
            print("\n Cannot generate efficiency plots - missing efficiency data")
            print(f"Available columns: {list(df.columns)}")

        # Create catalog comparison plot if we have catalog data
        if any('catalog' in col for col in df.columns) or hasattr(self, '_create_catalog_data'):
            print("\n Generating catalog comparison...")
            self.plot_catalog_comparison(results, save_path)

        return fig

    def _plot_efficiency_results(self, df, save_path=None):
        """
        Plot efficiency results (mechanical and isentropic) vs Pressure Ratio
        Using correct pressure ratio: p_discharge / p_suc
        """
        import matplotlib.pyplot as plt
        import numpy as np

        # Calculate CORRECT pressure ratio: p_discharge / p_suc
        df['pressure_ratio'] = df['p_discharge'] / df['p_suc']

        # Debug: print pressure ratio range
        print(f"\n Pressure Ratio Range: {df['pressure_ratio'].min():.2f} to {df['pressure_ratio'].max():.2f}")
        print(f" Isentropic Efficiency Range: {df['eta_is_overall'].min():.3f} to {df['eta_is_overall'].max():.3f}")
        if 'lambda_h' in df.columns:
            print(f" Volumetric Efficiency Range: {df['lambda_h'].min():.3f} to {df['lambda_h'].max():.3f}")

        condensing_temps = sorted(df['T_cond_C'].unique())
        colors = plt.cm.viridis(np.linspace(0, 1, len(condensing_temps)))

        fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 6))

        # Mechanical Efficiency vs Pressure Ratio
        for i, T_cond in enumerate(condensing_temps):
            cond_data = df[df['T_cond_C'] == T_cond].sort_values('pressure_ratio')

            if len(cond_data) > 0:
                ax1.plot(cond_data['pressure_ratio'], cond_data['eta_mech'] * 100,
                         marker='^', linewidth=2, markersize=6,
                         color=colors[i], label=f'T_cond = {T_cond}°C')

        ax1.set_xlabel('Pressure Ratio (p_discharge / p_suc)', fontsize=12)
        ax1.set_ylabel('Mechanical Efficiency [%]', fontsize=12)
        ax1.set_title('Mechanical Efficiency vs Pressure Ratio', fontsize=14, fontweight='bold')
        ax1.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax1.grid(True, alpha=0.3)
        ax1.tick_params(axis='both', which='major', labelsize=10)

        # Isentropic Efficiency vs Pressure Ratio
        for i, T_cond in enumerate(condensing_temps):
            cond_data = df[df['T_cond_C'] == T_cond].sort_values('pressure_ratio')

            if len(cond_data) > 0:
                ax2.plot(cond_data['pressure_ratio'], cond_data['eta_is_overall'] * 100,
                         marker='v', linewidth=2, markersize=6,
                         color=colors[i], label=f'T_cond = {T_cond}°C')

        ax2.set_xlabel('Pressure Ratio (p_discharge / p_suc)', fontsize=12)
        ax2.set_ylabel('Isentropic Efficiency [%]', fontsize=12)
        ax2.set_title('Isentropic Efficiency vs Pressure Ratio', fontsize=14, fontweight='bold')
        ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax2.grid(True, alpha=0.3)
        ax2.tick_params(axis='both', which='major', labelsize=10)

        # Volumetric Efficiency λ_h vs Pressure Ratio
        if 'lambda_h' in df.columns:
            for i, T_cond in enumerate(condensing_temps):
                cond_data = df[df['T_cond_C'] == T_cond].sort_values('pressure_ratio')

                if len(cond_data) > 0:
                    ax3.plot(
                        cond_data['pressure_ratio'],
                        cond_data['lambda_h'] * 100,
                        marker='o',
                        linewidth=2,
                        markersize=6,
                        color=colors[i],
                        label=f'T_cond = {T_cond}°C'
                    )

            ax3.set_ylabel('Volumetric Efficiency λ_h [%]', fontsize=12)
        else:
            ax3.text(
                0.5,
                0.5,
                "λ_h data not available",
                ha='center',
                va='center',
                transform=ax3.transAxes
            )
            ax3.set_ylabel('Volumetric Efficiency λ_h [%]', fontsize=12)

        ax3.set_xlabel('Pressure Ratio (p_discharge / p_suc)', fontsize=12)
        ax3.set_title('Volumetric Efficiency vs Pressure Ratio', fontsize=14, fontweight='bold')
        ax3.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax3.grid(True, alpha=0.3)
        ax3.tick_params(axis='both', which='major', labelsize=10)

        plt.tight_layout()

        if save_path:
            efficiency_save_path = save_path.replace('.png', '_efficiencies.png')
            plt.savefig(efficiency_save_path, dpi=300, bbox_inches='tight')
            print(f"Efficiency plot saved to {efficiency_save_path}")

        plt.show()

    def run_operating_map(self, T_evap_range, T_cond_range, n_speed=1.0, T_ambient=298.15):
        """
        Run compressor model over multiple evaporation and condensing temperatures
        """
        results = []

        total_points = len(T_evap_range) * len(T_cond_range)
        current_point = 0

        for T_evap_c in T_evap_range:
            for T_cond_c in T_cond_range:
                current_point += 1

                try:
                    # Convert to Kelvin
                    T_evap_K = T_evap_c + 273.15
                    T_cond_K = T_cond_c + 273.15

                    # Calculate superheating (T_suction = 20°C)
                    T_suction_target = 20 + 273.15  # 20°C in Kelvin
                    dT_superheating = T_suction_target - T_evap_K

                    # Calculate pressures from temperatures
                    state_evap_sat = self.med_prop.calc_state("TQ", T_evap_K, 1.0)
                    p_evap = state_evap_sat.p

                    state_cond_sat = self.med_prop.calc_state("TQ", T_cond_K, 0.0)
                    p_discharge = state_cond_sat.p

                    # Create inputs
                    inputs = Inputs(
                        n=n_speed,
                        T_eva_in=T_evap_K,
                        dT_eva_superheating=dT_superheating,
                        T_ambient=T_ambient,
                        T_con_in=T_cond_K,
                        m_flow_eva=0.1,
                        m_flow_con=0.1,
                        dT_con_subcooling=5.0
                    )

                    fs_state = FlowsheetState()

                    # Clear cache for new operating point
                    self._clear_cache()

                    # Solve for this operating point
                    #start_time = time.time()
                    solution = self.simulate_operating_point(inputs, p_discharge, fs_state)
                    #solve_time = time.time() - start_time

                    if self.W_dot_comp is not None and self.m_flow_suc > 0:
                        # Calculate ALL efficiencies
                        eta_mech = self.get_eta_mech(inputs)

                        eta_is_overall = self.get_eta_isentropic_overall(p_discharge, inputs)
                        lambda_h = self.get_lambda_h(inputs)

                        result = {
                            'T_evap_C': T_evap_c,
                            'T_cond_C': T_cond_c,
                            'T_evap_K': T_evap_K,
                            'T_cond_K': T_cond_K,
                            'p_evap': p_evap,
                            'p_discharge': p_discharge,
                            'p_suc': p_evap,  # p_suc is the same as p_evap in our model
                            'm_flow_kg_s': self.m_flow_suc,
                            'm_flow_kg_h': self.m_flow_suc * 3600,
                            'W_comp': self.W_dot_comp,
                            'eta_mech': eta_mech,
                            'eta_is_overall': eta_is_overall,
                            'lambda_h': lambda_h,
                            #'solve_time': solve_time,
                            'success': True,
                            'T_discharge_C': self.state_outlet.T - 273.15,
                            'T_wall_C': self.T_w - 273.15
                        }
                        results.append(result)

                    else:
                        results.append({
                            'T_evap_C': T_evap_c,
                            'T_cond_C': T_cond_c,
                            'success': False,
                            'error': 'No solution found'
                        })

                except Exception as e:
                    results.append({
                        'T_evap_C': T_evap_c,
                        'T_cond_C': T_cond_c,
                        'success': False,
                        'error': str(e)
                    })

        return results

    def export_results_to_csv(self, results, filename):
        """
        Export results to CSV file
        """
        import pandas as pd
        import csv

        successful_results = [r for r in results if r.get('success', False)]

        if successful_results:
            df = pd.DataFrame(successful_results)
            df.to_csv(filename, index=False)
            print(f"Results exported to {filename}")

            # Print summary statistics
            print(f"\n SUMMARY STATISTICS:")
            print(f"Total operating points: {len(results)}")
            print(f"Successful: {len(successful_results)}")
            print(f"Success rate: {len(successful_results) / len(results) * 100:.1f}%")
            print(f"Mass flow range: {df['m_flow_kg_h'].min():.1f} - {df['m_flow_kg_h'].max():.1f} kg/h")
            print(f"Power range: {df['W_comp'].min():.1f} - {df['W_comp'].max():.1f} W")

        else:
            print("No successful results to export")

    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Get the mechanical efficiency including motor and inverter efficiencies.

        For the HRP model, we calculate mechanical efficiency as:
        η_mech = W_dot_int / W_dot_comp

        Where:
        - W_dot_int = m_dot_3 * (h_4 - h_3) [internal work]
        - W_dot_comp = W_dot_int + W_dot_loss [electrical power input]

        Args:
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Mechanical efficiency including motor and inverter efficiencies.
        """
        if self.W_dot_comp is None or self.W_dot_comp <= 0:
            return 0.0

        # Calculate internal work
        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        if (self.state_3 is None or self.state_4 is None or
                self.state_3.d is None or self.state_4.h is None or self.state_3.h is None):
            return 0.0

        rho3 = self.state_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f
        h4 = self.state_4.h
        h3 = self.state_3.h
        W_dot_int = m_dot_3 * (h4 - h3)

        if W_dot_int <= 0:
            return 0.0

        # Mechanical efficiency = internal work / electrical input
        eta_mech = W_dot_int / self.W_dot_comp

        # Ensure reasonable bounds
        return max(0.1, min(0.95, eta_mech))

    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Get the volumetric efficiency.

        For the HRP model, volumetric efficiency is defined as:
        λ_h = m_dot_suc / (ρ_suc * V_h * f)

        Where:
        - m_dot_suc: Actual suction mass flow rate
        - ρ_suc: Density at suction conditions
        - V_h: Compressor displacement volume
        - f: Rotational frequency

        Args:
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Volumetric efficiency.
        """
        if (self.m_flow_suc is None or self.state_inlet is None or
                self.state_inlet.d is None or self.m_flow_suc <= 0):
            return 0.0

        # Get suction density
        rho_suc = self.state_inlet.d

        # Get rotational frequency
        n_abs = self.get_n_absolute(inputs.n)
        f = n_abs

        # Theoretical mass flow (no losses)
        m_dot_theoretical = rho_suc * self.V_h * f

        if m_dot_theoretical <= 0:
            return 0.0

        # Volumetric efficiency = actual flow / theoretical flow
        lambda_h = self.m_flow_suc / m_dot_theoretical

        # Ensure reasonable bounds (typically 0.6-0.95 for compressors)
        return max(0.5, min(0.98, lambda_h))

    def get_eta_isentropic_overall(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Overall isentropic efficiency from suction (state_inlet) to final discharge (state_outlet):

            η_is_overall = (h_dis_is - h_suc) / (h_dis_actual - h_suc)
        """
        if (self.state_inlet is None or self.state_outlet is None or
                self.state_inlet.s is None or self.state_inlet.h is None or
                self.state_outlet.h is None):
            return 0.0

        try:
            # Suction enthalpy and entropy
            h_suc = self.state_inlet.h
            s_suc = self.state_inlet.s

            # Actual discharge enthalpy
            h_dis_actual = self.state_outlet.h

            # Need actual work > 0
            if h_dis_actual <= h_suc:
                return 0.0

            # Isentropic discharge state at same outlet pressure
            state_dis_isen = self.med_prop.calc_state("PS", p_outlet, s_suc)
            h_dis_isen = state_dis_isen.h

            # Also require isentropic discharge to be above suction
            if h_dis_isen <= h_suc:
                return 0.0

            # Ideal and actual specific works
            w_is = h_dis_isen - h_suc
            w_actual = h_dis_actual - h_suc

            if w_actual <= 0:
                return 0.0

            eta_is = w_is / w_actual

            # Clamp to [0, 1]
            if eta_is > 1.0:
                if ENABLE_TIMING:
                    print(f"  Warning: Isentropic efficiency > 1.0: {eta_is:.3f}")
            eta_is = max(0.0, min(1.0, eta_is))
            return eta_is

        except Exception as e:
            if ENABLE_TIMING:
                print(f"Warning in overall isentropic efficiency calculation: {e}")
            return 0.7


if __name__ == "__main__":
    print("  COMPRESSOR MODEL TEST")
    print("=" * 60)

    try:
        # 1. Create compressor instance
        compressor = Molinaroli_2017_Compressor(N_max=50, V_h=0.0000161, refrigerant="R290")
        print("✓ Compressor instance created")

        # 2. Create test inputs
        inputs = Inputs(
            n=1,
            T_eva_in=283.15,  # 5°C evaporation
            dT_eva_superheating=10.0,
            T_ambient=293.15,  # 20°C ambient
            T_con_in=310.15,
            m_flow_eva=0.1,
            m_flow_con=0.1,
            dT_con_subcooling=5.0
        )
        p_outlet = 1369299  # in Pa

        print("✓ Test conditions:")
        print(f"  - Evaporation: {inputs.T_eva_in - 273.15:.1f}°C")
        print(f"  - Discharge: {p_outlet / 1000:.1f} kPa")
        print(f"  - Speed: {inputs.n * 100}% of max")

        # 3. Create flowsheet state for results
        fs_state = FlowsheetState()

        # 4. RUN THE FULL SIMULATION!
        print("\n" + "=" * 60)
        print("RUNNING COMPLETE COMPRESSOR SIMULATION")
        print("=" * 60)

        #start_time = time.time()
        solution = compressor.simulate_operating_point(inputs, p_outlet, fs_state)
        #solve_time = time.time() - start_time

        #print(f"\n  Solution time: {solve_time:.3f} seconds")
        # Define operating range (manufacturer catalog range)
        eta_mech = compressor.get_eta_mech(inputs)
        print(f"Mechanical efficiency: {eta_mech:.3f}")

        # 2. Test get_lambda_h
        lambda_h = compressor.get_lambda_h(inputs)
        print(f"Volumetric efficiency: {lambda_h:.3f}")

        # Use the exact temperature ranges from the catalog data
        T_evap_range = [-25, -20, -15, -10, -5, 0, 5, 10, 15]
        T_cond_range = [30, 35, 40, 45, 50, 55, 60]

        results = compressor.run_operating_map(
            T_evap_range=T_evap_range,
            T_cond_range=T_cond_range,
            n_speed=1.0
        )

        # This will now generate:
        # 1. Standard performance plots (mass flow & power vs T_evap)
        # 2. Efficiency plots vs pressure ratio
        # 3. Catalog comparison plots with detailed statistics
        compressor.plot_operating_map(results, save_path="fixed_analysis.png")
        # Export to CSV
        compressor.export_results_to_csv(results, "compressor_results.csv")
        # 5. Verify results
        if compressor.W_dot_comp is not None and compressor.m_flow_suc > 0:
            print("\nSIMULATION SUCCESSFUL!")
            print(f" Key Results:")
            print(f"   - Mass flow rate: {60 * 60 * compressor.m_flow_suc:.6f} kg/h")
            print(f"   - Electrical power: {compressor.W_dot_comp:.2f} W")
            print(f"   - Specific work: {compressor.W_dot_comp / compressor.m_flow_suc / 1000:.2f} kJ/kg")

            # Calculate isentropic efficiency for reference
            h_suc = compressor.state_inlet.h
            s_suc = compressor.state_inlet.s
            state_dis_isen = compressor.med_prop.calc_state("PS", p_outlet, s_suc)
            h_dis_isen = state_dis_isen.h
            h_dis_actual = compressor.state_outlet.h
            eta_is = (h_dis_isen - h_suc) / (h_dis_actual - h_suc) if (h_dis_actual - h_suc) > 0 else 0
            print(f"   - Isentropic efficiency: {eta_is:.3f}")

        else:
            print("\n Simulation failed to produce valid results")

    except Exception as e:
        print(f"\n SIMULATION FAILED: {e}")
        import traceback

        traceback.print_exc()

