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
    def __init__(self, N_max: float, V_h: float, parameters = {"Ua_suc_ref": 16.05,
                                                               "Ua_dis_ref": 13.96,
                                                               "Ua_amb": 0.36,
                                                               "A_tot": 9.47e-9,
                                                               "A_dis": 86.1e-9,
                                                               "V_IC": 16.11e-6,
                                                               "alpha_loss": 0.16,
                                                               "W_dot_loss_ref": 83,
                                                               "m_dot_ref": 0.0083,
                                                               "f_ref": 50.0}):
        super().__init__(N_max=N_max, V_h=V_h)

        #self.refrigerant = refrigerant
        #self.med_prop = CoolProp(fluid_name=refrigerant)

        # Parameters for compressor 'B' from Molinaroli et. al(2017)
        self.parameters = parameters


        ##INITIALIZE VARIABLES##
        # Thermodynamic states
        self.state_c_suc: ThermodynamicState = None
        self.state_c_1: ThermodynamicState = None
        self.state_c_3: ThermodynamicState = None
        self.state_c_4: ThermodynamicState = None
        self.state_c_5: ThermodynamicState = None

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

        h_suc = self.state_inlet.h
        s_suc = self.state_inlet.s

        n_abs = self.get_n_absolute(inputs.control.n)
        f = n_abs

        p_dis = p_outlet
        state_dis_is = self.med_prop.calc_state("PS", p_dis, s_suc)
        T_dis_is = state_dis_is.T

        rho_suc = self.state_inlet.d
        m_dot_suc_0 = rho_suc * self.parameters["V_IC"] * f

        T_w_0 = (self.state_inlet.T + T_dis_is) / 2

        transport_suc = self.med_prop.calc_transport_properties(self.state_inlet)
        cp_suc = transport_suc.cp
        h1_0 = h_suc + 0.5 * cp_suc * (T_w_0 - self.state_inlet.T)

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
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        rho3 = self.state_c_3.d

        cache_key = self._get_cache_key(p_suc, h1, p_suc, h3, p4, s3)

        # Check if we have cached results for these states
        if (self._last_x is not None and
                np.allclose(x, self._last_x, rtol=1e-6, atol=1e-8) and
                cache_key in self._state_cache):

            cached_data = self._state_cache[cache_key]
            self.state_c_1 = cached_data['state_c_1']
            self.state_c_4 = cached_data['state_c_4']
            h4 = cached_data['h4']
            transport4 = cached_data['transport4']
            gamma4 = cached_data['gamma4']
            m_dot_tot = cached_data['m_dot_tot']

            self._cache_hits += 1

        else:
            # Cache miss - calculate everything
            self._cache_misses += 1

            # State 1: After suction heat transfer
            self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)

            # State 4: Isentropic compression (expensive call)
            self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
            h4 = self.state_c_4.h

            # Calculate transport properties for state_4 ONCE
            if (self._cached_transport4 is None or
                    self._cached_gamma4 is None or
                    abs(self.state_c_4.p - p4) > 1000):

                transport4 = self.med_prop.calc_transport_properties(self.state_c_4)
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
                'state_c_1': self.state_c_1,
                'state_c_4': self.state_c_4,
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
        n_abs = self.get_n_absolute(inputs.control.n)
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
        n_abs = self.get_n_absolute(inputs.control.n)
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
        T_amb = getattr(inputs, "T_amb" ,  25+273.15)
        n_abs = self.get_n_absolute(inputs.control.n)
        f = n_abs

        rho3 = self.state_c_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f

        W_dot_int = m_dot_3 * (h4 - h3)

        W_dot_loss = (W_dot_int * self.parameters["alpha_loss"] +
                      self.parameters["W_dot_loss_ref"] * (n_abs / self.parameters["f_ref"]) ** 2)

        Q_dot_suc = m_dot_suc * (h1 - h_suc)

        h_dis, epsilon_dis = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis)
        Q_dot_dis = m_dot_suc * (h4 - h_dis)


        Q_dot_amb = np.sign(T_w - T_amb) * self.parameters["Ua_amb"] * (abs(T_w - T_amb) ** 1.25)


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
            self.state_c_5 = self.med_prop.calc_state("PH", p_dis, h4)
            T5 = self.state_c_5.T

            transport5 = self.med_prop.calc_transport_properties(self.state_c_5)
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
            (self.state_inlet.T, 400),  # T_w between suction and 127 °C
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
            max_nfev=20000
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
        self.state_c_1 = self.med_prop.calc_state("PH", p_suc, h1)
        self.state_c_3 = self.med_prop.calc_state("PH", p_suc, h3)
        s3 = self.state_c_3.s
        self.state_c_4 = self.med_prop.calc_state("PS", p4, s3)
        h4 = self.state_c_4.h
        n_abs = self.get_n_absolute(inputs.control.n)
        f = n_abs

        # Calculate discharge state
        h_dis, epsilon_dis = self._calculate_discharge_heat_transfer(m_dot_suc, T_w, h4, p_dis)
        self.state_c_5 = self.med_prop.calc_state("PH", p_dis, h_dis)
        self.state_outlet = self.state_c_5
        rho3 = self.state_c_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f

        # Calculate internal work
        W_dot_int = m_dot_3 * (h4 - h3)

        # Calculate powers and efficiencies
        n_abs = self.get_n_absolute(inputs.control.n)
        W_dot_loss = (W_dot_int * self.parameters["alpha_loss"] +
                      self.parameters["W_dot_loss_ref"] * (n_abs / self.parameters["f_ref"]) ** 2)
        self.P_el = W_dot_int + W_dot_loss

        # Store results
        self.m_flow = m_dot_suc
        self.T_w = T_w

        # Populate flowsheet state
        fs_state.set("m_flow", self.m_flow, "kg/s", "Refrigerant mass flow rate")
        fs_state.set("P_el", self.P_el, "W", "Electrical power input")
        fs_state.set("T_wall", T_w, "K", "Wall temperature")
        fs_state.set("pc4", p4, "Pa", "Internal discharge pressure")
        fs_state.set("hc1", h1, "J/kg", "Enthalpy after suction heat transfer")
        fs_state.set("hc3", h3, "J/kg", "Enthalpy after mixing")
        fs_state.set("hc4", h4, "J/kg", "Enthalpy after compression")
        fs_state.set("T_dis", self.state_c_5.T, "K", "Discharge temperature")
        fs_state.set("p_2", self.state_outlet.p, "Pa", "Outlet pressure")
        fs_state.set("p_1", self.state_inlet.p, "Pa", "Inlet Pressure")
        fs_state.set("T_1", self.state_inlet.T, "K", "Inlet Temperature")

#

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
        n_abs = self.get_n_absolute(inputs.control.n)
        f = n_abs

        if (self.state_c_3 is None or self.state_c_4 is None or
                self.state_c_3.d is None or self.state_c_4.h is None or self.state_c_3.h is None):
            return 0.0

        rho3 = self.state_c_3.d
        m_dot_3 = rho3 * self.parameters["V_IC"] * f
        h4 = self.state_c_4.h
        h3 = self.state_c_3.h
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
        if (self.m_flow is None or self.state_inlet is None or
                self.state_inlet.d is None or self.m_flow <= 0):
            return 0.0

        # Get suction density
        rho_suc = self.state_inlet.d

        # Get rotational frequency
        n_abs = self.get_n_absolute(inputs.control.n)
        f = n_abs

        # Theoretical mass flow (no losses)
        m_dot_theoretical = rho_suc * self.V_h * f

        if m_dot_theoretical <= 0:
            return 0.0

        # Volumetric efficiency = actual flow / theoretical flow
        lambda_h = self.m_flow / m_dot_theoretical

        # Ensure reasonable bounds (typically 0.6-0.95 for compressors)
        return max(0.5, min(0.98, lambda_h))

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Overall isentropic efficiency from suction (state_inlet) to final discharge (state_outlet):

            η_is_overall = (h_dis_is - h_suc) / (h_dis_actual - h_suc)
        """
        if (self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2 or self.state_inlet.p != fs_state.p_1):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

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

    def calc_state_outlet(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState):
        """
        Calculate the output state based on the high pressure level and the provided inputs.
        The state is automatically set as the outlet state of this component.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for calculation.
            fs_state (FlowsheetState): Flowsheet state.
        """

        self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

    def calc_m_flow(self, p_outlet, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Calculate the refrigerant mass flow rate.

        Args:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Refrigerant mass flow rate.
        """
        if (self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2 or self.state_inlet.p != fs_state.p_1):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

        return self.m_flow

    def calc_electrical_power(self, p_outlet, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Calculate the electrical power consumed by the compressor based on an adiabatic energy balance.

        Args:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Electrical power consumed.
        """
        if (self.state_inlet.T != fs_state.T_1 or p_outlet != fs_state.p_2 or self.state_inlet.p != fs_state.p_1):
            self.simulate_operating_point(inputs=inputs, p_outlet=p_outlet, fs_state=fs_state)

        return self.P_el
#todo: add calc mass flow, add calc electrical power
