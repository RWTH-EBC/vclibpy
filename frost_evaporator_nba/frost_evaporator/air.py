from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import warnings
import math
import CoolProp.CoolProp as CP_HumidAir
from scipy.optimize import brentq
from scipy.interpolate import interp1d

class AirModel:

    def __init__(self, 
                 parameters: FrostEvaporatorParameters, 
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
        """
        self.params = parameters

        # Wang parameters validity check
        self._check_wang_validity()


        # Fan interpolation setup
        flow_data = np.array([
            0.0, 24.591265397536393, 39.50727883538634, 46.96528555431131, 50.190369540873455,
            52.40761478163493, 55.02799552071668, 58.0515117581187, 61.679731243001115,
            65.30795072788354, 68.93617021276596, 81.43337066069428, 105.21836506159013,
            118.11870100783874, 127.39081746920492, 136.46136618141097, 145.73348264277715,
            155.20716685330348, 164.88241881298993, 174.55767077267637
        ])
        pressure_data = np.array([
            61.15112994350283, 58.13559322033898, 54.971751412429384, 51.7090395480226, 48.44632768361582,
            45.183615819209045, 42.01977401129944, 38.75706214689266, 35.494350282485875,
            32.33050847457627, 29.06779661016949, 25.805084745762713, 22.641242937853107,
            19.37853107344633, 16.11581920903955, 13.05084745762712, 9.689265536723164,
            6.52542372881356, 3.26271186440678, 0.09887005649717515
        ])
        self.min_f = flow_data[0]
        self.max_f = flow_data[-1]
        self.interpolator = interp1d(flow_data, pressure_data, kind='cubic', fill_value="extrapolate")



    ####################################################################################
    # Calculate Fan-System Interaction for Air Flow
    ####################################################################################

    def solve_fan_system_equilibrium(self, states: list[FrostEvaporatorState], global_inputs: FrostEvaporatorInputs):
        """
        Solves the Hydraulic Circuit: Fan Curve vs System Resistance.
        
        Calculates the GLOBAL Mass Flow Rate that satisfies the pressure balance.
        Then calculates and sets the local Velocity for each layer based on that mass flow
        and the local density/frost blockage.
        """
        
        # Calculate Air Density at Fan Inlet
        T_in = global_inputs.air.T_in
        p_in = global_inputs.air.p_in
        W_in = global_inputs.air.W_in
        density_inlet = 1.0 / CP_HumidAir.HAPropsSI('V', 'T', T_in, 'P', p_in, 'W', W_in)

        # --- Solve ---
        max_m_dot = (self.max_f / 3600.0) * density_inlet
        
        try:
            m_dot_solved = brentq(
                f    = self._calculate_hydraulic_residual, 
                a    = 1e-4, 
                b    = max_m_dot,  
                args = (states, density_inlet),
                xtol = 1e-6)
        except ValueError:
            # If choked or out of range, default to 0 or min
            m_dot_solved = 0.0
            print("Warning: Fan solver failed to converge. Flow set to 0.")


        # --- Apply Results to States ---
        v_dot_fan_m3h = (m_dot_solved / density_inlet) * 3600.0
        total_pressure_drop_fan = self._get_pressure_from_flow(v_dot_fan_m3h)

        # Initialize running pressure with global inlet pressure
        current_static_pressure = global_inputs.air.p_in
        
        for state in states:
            density_local = state.air.density_avg

            # Re-calculate exact dP and Velocity for this layer using solved mass flow
            dp_layer, v_local = self._calculate_layer_dp(
                state         = state, 
                m_dot         = m_dot_solved, 
                density_local = density_local
            )

            state.air.set("velocity", v_local)
            state.air.set("m_dot_humid", m_dot_solved)
            state.air.set("p_out", current_static_pressure - dp_layer)
            state.air.set("pressure_drop", dp_layer)
            state.air.set("total_system_pressure_drop", total_pressure_drop_fan)

            # Decrement pressure for the next layer (Outlet of n is Inlet of n+1)
            current_static_pressure -= dp_layer


    ####################################################################################
    # Helpers for Air Flow
    ####################################################################################

    

    def _check_wang_validity(self) -> None:
        """
        Validates if the geometry lies within the limits of the Wang et al. (2000) correlation.

        This checks if the plain fin-and-tube exchanger parameters (tube layers,
        diameters, and pitches) are within the experimental ranges defined by the
        correlation. Emits a warning if parameters are out of bounds.

        Ranges Checked:
            - Tube Rows (N): 1 to 6
            - Tube Outer Diameter: 6.35 to 12.7 [mm]
            - Fin Pitch: 1.19 to 8.7 [mm]
            - Transverse Pitch: 17.7 to 31.75 [mm]
            - Longitudinal Pitch: 12.4 to 27.5 [mm]

        Raises:
            UserWarning: If any geometric parameter is outside the valid range.
        """
        # scaling factor: meters to millimeters
        m_to_mm = 1000.0

        # Extract and convert parameters to mm for comparison
        tube_rows = self.params.tube_layers
        tube_outer_diameter_mm = self.params.tube_outer_diameter * m_to_mm
        fin_pitch_mm = self.params.fin_pitch * m_to_mm
        transverse_pitch_mm = self.params.transverse_tube_pitch * m_to_mm
        longitudinal_pitch_mm = self.params.longitudinal_tube_pitch * m_to_mm

        # Define validation limits: (current_value, min_limit, max_limit)
        checks = {
            "Tube Rows (N)": (tube_rows, 1, 6),
            "Tube Outer Diameter (Do)": (tube_outer_diameter_mm, 6.35, 12.7),
            "Fin Pitch (Fp)": (fin_pitch_mm, 1.19, 8.7),
            "Transverse Pitch (Pt)": (transverse_pitch_mm, 17.7, 31.75),
            "Longitudinal Pitch (Pl)": (longitudinal_pitch_mm, 12.4, 27.5),
        }

        for name, (val, min_v, max_v) in checks.items():
            if not (min_v <= val <= max_v):
                warnings.warn(
                    f"[Wang Correlation Validity] {name} value {val:.2f} is outside "
                    f"the valid range [{min_v} - {max_v}]. Results may be inaccurate."
                )

    def _calculate_hydraulic_residual(self, m_dot_guess: float, states: list[FrostEvaporatorState], density_inlet: float)-> float:
        """
        Residual = Fan_Pressure(V_dot) - Sum(Layer_Pressure_Drops)
        """
        # A. Calculate Fan Pressure Available
        v_dot_fan_m3h = (m_dot_guess / density_inlet) * 3600.0
        dp_fan = self._get_pressure_from_flow(v_dot_fan_m3h)
        
        # B. Calculate System Pressure Drop (Sum of all layers)
        dp_system_total = 0.0
        
        for state in states:                
            dp_layer, _ = self._calculate_layer_dp(
                state         = state, 
                m_dot         = m_dot_guess,
                density_local = state.air.density_avg
            )
            dp_system_total += dp_layer
            
        return dp_fan - dp_system_total
    

    def _get_pressure_from_flow(self, flow_val: float) -> float:
        """
        Returns Static Pressure for a given Volume Flow.

        Args:
            flow_val: The volume flow rate [m^3/h].

        Returns:
            The static pressure [Pa].
        """
        # Check for out of bounds
        if flow_val < self.min_f or flow_val > self.max_f:
            print(
                f"--> WARNING: Flow input {flow_val:.2f} is outside valid range "
                f"({self.min_f:.2f} - {self.max_f:.2f}). Extrapolating..."
            )

        return float(self.interpolator(flow_val))


    def _calculate_layer_dp(self, state: list[FrostEvaporatorState], m_dot: float, density_local: float) -> tuple[float, float]:
        """Helper to calculate dP for a single layer to avoid code duplication"""
        # Calculate Local Velocity
        v_loc = m_dot / (state.air.density_avg * state.frost.flow_area_air)

        # Pressure Drop Calculation 
        if self.params.pressure_drop_correlation_choice == "Wang":
            hydraulic_diameter = 4 * (state.frost.flow_area_air * self.params.fin_length) / state.frost.A_frost_surface

            dp_raw = self._calculate_system_resistance_wang(
                velocity=v_loc, 
                density=state.air.density_avg, 
                dyn_viscosity=state.air.dyn_viscosity_avg, 
                longitudinal_tube_pitch=self.params.longitudinal_tube_pitch, 
                transverse_tube_pitch=self.params.transverse_tube_pitch, 
                fin_pitch=self.params.fin_pitch, 
                collar_diameter_w_frost=self.params.tube_outer_diameter + 2 * self.params.fin_thickness + 2 * state.frost.thickness, 
                tube_layers=self.params.tube_layers,
                flow_length=self.params.fin_length,
                hydraulic_diameter=hydraulic_diameter,
            )
        
        elif self.params.pressure_drop_correlation_choice == "Haaf":
            dp_raw = self._calculate_system_resistance_haaf(
                velocity                = v_loc, 
                density                 = state.air.density_avg, 
                dyn_viscosity           = state.air.dyn_viscosity_avg, 
                space_between_frost     = state.frost.space_between_frost, 
                longitudinal_tube_pitch = self.params.longitudinal_tube_pitch
            )
        
        else:
            raise ValueError(f"Unknown pressure drop correlation choice: {self.params.pressure_drop_correlation_choice}")

        return self.params.correction_factor_pressure_loss * dp_raw, v_loc




    def _calculate_system_resistance_wang(self, velocity: float, density: float, dyn_viscosity: float, longitudinal_tube_pitch: float, 
                                               transverse_tube_pitch: float, fin_pitch: float, collar_diameter_w_frost: float, tube_layers: int,
                                               flow_length: float,hydraulic_diameter: float) -> float:
        """
        Calculates Pressure Drop using Wang et al. (2000) Friction Factor.
        """
        if velocity <= 1e-5:
            return 0.0

        # 1. Reynolds (based on Collar Diameter Dc, NOT Hydraulic Diameter)
        reynolds_dc = (density * velocity * collar_diameter_w_frost) / dyn_viscosity
        
        # Clamp Re to prevent log errors
        reynolds_dc = max(reynolds_dc, 10.0)
        ln_re = math.log(reynolds_dc)
        
        Pl = longitudinal_tube_pitch
        Pt = transverse_tube_pitch
        Fp = fin_pitch
        Dc = collar_diameter_w_frost
        N = float(tube_layers)

        # 2. Coefficients (Eq 13, 14, 15)
        F1 = -0.764 + (0.739 * (Pt / Pl)) + (0.177 * (Fp / Dc)) - (0.00758 / N)
        F2 = -15.689 + (64.021 / ln_re)
        F3 = 1.696 - (15.695 / ln_re)

        # 3. Fanning Friction Factor (Eq 12)
        f = 0.0267 * (reynolds_dc ** F1) * ((Pt / Pl) ** F2) * ((Fp / Dc) ** F3)

        # 4. Calculate Pressure Drop (Fanning Equation)
        # Note: If you have Area Ratio (A_tot / A_min), use: f * (A_tot/A_min) * dynamic_pressure
        # Here we use the Hydraulic Diameter equivalent: 4 * f * (L/Dh) * dynamic_pressure
        
        dynamic_pressure = 0.5 * density * (velocity ** 2)
        
        # The factor 4 comes from conversion of Fanning f to Darcy-Weisbach context in non-circular ducts
        friction_term = 4.0 * f * (flow_length / hydraulic_diameter)
        
        return friction_term * dynamic_pressure





    
    def _calculate_system_resistance_haaf(self, velocity: float, density: float, dyn_viscosity: float, space_between_frost: float, longitudinal_tube_pitch: float) -> float:
        """
        Calculates the system pressure drop (resistance) based on Haaf correlation.
        
        Args:
            velocity: Air velocity [m/s].
            density: Air density [kg/m^3].
            dyn_viscosity: Dynamic viscosity [Pa·s].
            space_between_frost: Effective space between frost layers [m].
            longitudinal_tube_pitch: Space between pipes in flow direction [m].

        Returns:
            Pressure drop of the system [Pa].
        """
        if velocity <= 0:
            return 0.0

        reynolds = (density * velocity * (2 * space_between_frost)) / dyn_viscosity

        zeta = self._calculate_pressure_loss_coefficient_haaf(
            reynolds=reynolds,
            space_between_frost=space_between_frost,
            longitudinal_tube_pitch=longitudinal_tube_pitch
        )

        # Bernoulli / Darcy-Weisbach formulation
        return zeta * 0.5 * density * (velocity ** 2)
    


    def _calculate_pressure_loss_coefficient_haaf(self,reynolds: float,space_between_frost: float,longitudinal_tube_pitch: float) -> float:
        """
        Calculates the pressure loss coefficient (zeta) based on the Haaf correlation.

        Args:
            reynolds: The Reynolds number [-].
            space_between_frost: The effective space between frost layers [m].
            longitudinal_tube_pitch: The distance two tubes in air flow direction [m].

        Returns:
            The pressure loss coefficient [-].

        Raises:
            ValueError: If fin_spacing is less than or equal to zero.
        """

        length_ratio = space_between_frost / longitudinal_tube_pitch

        return 10.5 * (reynolds ** (-1.0 / 3.0)) * (length_ratio ** 0.6)





    ####################################################################################
    # Update Properties
    ####################################################################################

    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the air model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # ================= Calculate new Values =================

        # --- Average Air Properties ---
        pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, thermal_conductivity_avg, prandtl_avg, lewis_avg, rho_w_in, rho_w_out, h_in, h_out, R_in, R_out = self._calculate_air_averages(
            T_in  = inputs.air.T_in,
            p_in  = inputs.air.p_in,
            W_in  = inputs.air.W_in,
            T_out = state.air.T_out,
            p_out = state.air.p_out,
            W_out = state.air.W_out
        )

        # --- Water Vapor Densities (at surface and base) ---
        rho_w_frost_surface_sat, W_frost_surface_sat = self._get_water_vapor_density(
            T = state.hmt.T_frost_surface, 
            p = pressure_avg, 
            R = 1.0  # Saturated
        )
        
        rho_w_frost_base_sat, W_frost_base_sat = self._get_water_vapor_density(
            T = state.hmt.T_frost_base, 
            p = pressure_avg, 
            R = 1.0  # Saturated
        )

        # --- Geometry / Reynolds Number ---
        D_h     = 2 * state.frost.space_between_frost 
        D_c_eff = self.params.tube_outer_diameter + 2 * self.params.fin_thickness + 2 * state.frost.thickness

        Re_Dc = self._calculate_reynolds_number(
            density               = density_avg, 
            velocity              = state.air.velocity, 
            characteristic_length = D_c_eff, 
            dyn_viscosity         = dyn_viscosity_avg
        )

        # --- Heat Transfer Coefficient ---
        h_conv_raw = self._calculate_h_conv_wang(
            Re_Dc             = Re_Dc,
            N                 = self.params.tube_layers,
            F_p               = self.params.fin_pitch,
            D_c               = D_c_eff,
            D_h               = D_h,
            P_t               = self.params.transverse_tube_pitch,
            P_l               = self.params.longitudinal_tube_pitch,
            density_avg       = density_avg,
            velocity          = state.air.velocity,
            heat_capacity_avg = heat_capacity_avg,
            prandtl_avg       = prandtl_avg
        )

        h_conv = h_conv_raw * self.params.correction_factor_h_conv_air

        # --- Mass Transfer Coefficient ---
        betta_raw = self._calculate_mass_transfer_coefficient(
            h_conv        = h_conv,
            density       = density_avg,
            heat_capacity = heat_capacity_avg,
            lewis_number  = lewis_avg
        )

        betta = betta_raw * self.params.correction_factor_betta_air

        # --- Mass Flows ---
        m_dot_humid, m_dot_dry = self._calculate_mass_flows(
            density       = density_avg, 
            flow_area_air = state.frost.flow_area_air, 
            velocity      = state.air.velocity, 
            W_in          = inputs.air.W_in, 
            W_out         = state.air.W_out
        )

        # --- Enthalpy of Ice & Dew Point ---
        h_ice = self._get_ice_enthalpy(
            T = state.hmt.T_frost_surface
        )

        T_dew_point = self._get_inlet_dew_point(
            p_in = inputs.air.p_in,
            W_in = inputs.air.W_in,
            T_in = inputs.air.T_in
        )


        # ================= Write new values to state =================
        # Air Properties
        state.air.set("pressure_avg", pressure_avg)
        state.air.set("density_avg", density_avg)
        state.air.set("dyn_viscosity_avg", dyn_viscosity_avg)
        state.air.set("heat_capacity_avg", heat_capacity_avg)
        state.air.set("thermal_conductivity_avg", thermal_conductivity_avg)
        state.air.set("prandtl_avg", prandtl_avg)
        state.air.set("lewis_avg", lewis_avg)
        
        # Humid Air / Water Vapor
        state.air.set('rho_w_in', rho_w_in)
        state.air.set('rho_w_out', rho_w_out)
        state.air.set("rho_w_frost_surface_sat", rho_w_frost_surface_sat)
        state.air.set("W_frost_surface_sat", W_frost_surface_sat)
        state.air.set("rho_w_frost_base_sat", rho_w_frost_base_sat)
        state.air.set("W_frost_base_sat", W_frost_base_sat)
        state.air.set("T_dew_point", T_dew_point)
        state.air.set("R_in", R_in)
        state.air.set("R_out", R_out)

        # Flow & Heat Transfer
        # state.air.set("reynolds", 0.0)
        # state.air.set("nusselt", 0.0)
        state.air.set("h_conv", h_conv)
        state.air.set("betta", betta)
        
        # Mass Flow
        state.air.set("m_dot_humid", m_dot_humid)
        state.air.set("m_dot_dry", m_dot_dry)

        # Enthalpies
        state.air.set("h_in", h_in)
        state.air.set("h_out", h_out)
        state.air.set("h_ice", h_ice)


    ####################################################################################
    # Helper Functions
    ####################################################################################


    def _calculate_air_averages(self, T_in: float, p_in: float, W_in: float, T_out: float, p_out: float, W_out: float
                                ) -> tuple[float, float, float, float, float, float, float, float, float, float, float]:
        """
        Calculates average air properties between inlet and outlet states.

        Args:
            T_in: Inlet temperature [K].
            p_in: Inlet pressure [Pa].
            W_in: Inlet humidity ratio [kg_water/kg_dry_air].
            T_out: Outlet temperature [K].
            p_out: Outlet pressure [Pa].
            W_out: Outlet humidity ratio [kg_water/kg_dry_air].

        Returns:
            A tuple containing:
            0.  Average Pressure [Pa]
            1.  Average Density [kg/m^3]
            2.  Average Dynamic Viscosity [Pa*s]
            3.  Average Specific Heat Capacity [J/kg*K]
            4.  Average Thermal Conductivity [W/m*K]
            5.  Average Prandtl Number [-]
            6.  Average Lewis Number [-]
            7.  Inlet Water Vapor Density [kg/m^3]
            8.  Outlet Water Vapor Density [kg/m^3]
            9.  Inlet Specific Enthalpy [J/kg]
            10. Outlet Specific Enthalpy [J/kg]
        """
        # Unpack properties for inlet state
        (rho_in, mu_in, cp_in, k_in, pr_in, le_in, rho_w_in, h_in, R_in) = self._get_air_properties(T_in, p_in, W_in)

        # Unpack properties for outlet state
        (rho_out, mu_out, cp_out, k_out, pr_out, le_out, rho_w_out, h_out, R_out) = self._get_air_properties(T_out, p_out, W_out)

        # Calculate arithmetic averages
        pressure_avg = (p_in + p_out) / 2.0
        density_avg = (rho_in + rho_out) / 2.0
        viscosity_avg = (mu_in + mu_out) / 2.0
        heat_capacity_avg = (cp_in + cp_out) / 2.0
        conductivity_avg = (k_in + k_out) / 2.0
        prandtl_avg = (pr_in + pr_out) / 2.0
        lewis_avg = (le_in + le_out) / 2.0

        return (pressure_avg, density_avg, viscosity_avg, heat_capacity_avg, conductivity_avg, prandtl_avg, lewis_avg,
                rho_w_in, rho_w_out, h_in, h_out, R_in, R_out)

    def _get_air_properties(self, T: float, p: float, W: float) -> tuple[float, float, float, float, float, float, float, float]:
        """
        Calculates multiple thermophysical properties of moist air for a single state.

        Args:
            temperature: The dry bulb temperature [K].
            pressure: The absolute pressure [Pa].
            humidity_ratio: The humidity ratio (specific humidity) [kg_water/kg_dry_air].

        Returns:
            A tuple containing the following properties in order:
            0. Density [kg/m^3]
            1. Dynamic Viscosity [Pa*s]
            2. Specific Heat Capacity [J/kg*K]
            3. Thermal Conductivity [W/m*K]
            4. Prandtl Number [-]
            5. Lewis Number [-]
            6. Water Vapor Density [kg/m^3]
            7. Specific Enthalpy [J/kg_dry_air]
        """
        # Define property keys for CoolProp
        # Vha: Vol. per humid air, mu: Viscosity, cp_ha: Heat Cap., k: Conductivity, Hha: Specific Enthalpy per humid air basis
        prop_keys = ['Vha', 'mu', 'cp_ha', 'k', 'Enthalpy', 'R'] 
        
        props = {key: CP_HumidAir.HAPropsSI(key, 'T', T, 'P', p, 'W', W)for key in prop_keys}

        # Derived properties
        density = 1.0 / props['Vha']
        prandtl_number = (props['cp_ha'] * props['mu']) / props['k']
        
        # Lewis Number approximation (CoolProp lacks diffusivity for humid air)
        lewis_number = 0.85

        # Calculate water vapor density (utilizing internal helper)
        water_vapor_density, _ = self._get_water_vapor_density(T=T, p=p, W=W)

        return (
            density,
            props['mu'],
            props['cp_ha'],
            props['k'],
            prandtl_number,
            lewis_number,
            water_vapor_density,
            props['Enthalpy'],
            props['R']
        )
    

    def _get_water_vapor_density(self, T: float, p: float, W: float = None, R: float = None) -> tuple[float, float]:
        """
        Calculates the density of the water vapor in the moist air.

        Args:
            T: The dry bulb temperature [K].
            p: The ambient pressure [Pa].
            W: The humidity ratio [kg_w/kg_da].
            R: The relative humidity [0-1].

        Returns:
            The water vapor density [kg/m^3].
            The water humidity ratio  [kg_w/kg_da].

        Raises:
            ValueError: If neither W nor R is provided.
        """

        # Determine the humidity ratio W, if not given (e.g., at saturation)
        if W is not None:
            W_calc = W
        elif R is not None:
            # Calculate W at saturation (R=1.0)
            W_calc = CP_HumidAir.HAPropsSI('W', 'T', T, 'P', p, 'R', R)
        else:
            # Error if neither W nor R is provided
            raise ValueError("Must provide either W (humidity ratio) or R (relative humidity)")

        # Get the specific volume Vda (m^3 / kg_dry_air)
        Vda = CP_HumidAir.HAPropsSI('Vda', 'T', T, 'P', p, 'W', W_calc)
        
        # Calculate the water vapor density:
        rho_w = W_calc / Vda         
        return rho_w, W_calc


    def _calculate_reynolds_number(self, density: float, velocity: float, characteristic_length: float, dyn_viscosity: float) -> float:
        """
        Calculates the Reynolds number (Re = rho * v * L / mu).

        Args:
            density: The fluid density [kg/m^3].
            velocity: The fluid velocity [m/s].
            characteristic_length: The characteristic length [m].
            dyn_viscosity: The dynamic viscosity [Pa*s].

        Returns:
            The Reynolds number [dimensionless].        """
        return (density * velocity * characteristic_length) / dyn_viscosity



    def _calculate_h_conv_wang(self, Re_Dc: float, N: int, F_p: float, D_c: float, D_h: float, P_t: float, P_l: float,
                                    density_avg: float, velocity: float, heat_capacity_avg: float, prandtl_avg: float) -> float:
        """
        Calculates Colburn j-factor using Wang et al. (2000) correlations.

        Args:
            Re_Dc: Reynolds number based on Collar Diameter [dimensionless].
            N: Number of tube rows (tube_layers) [count].
            F_p: Fin Pitch (center-to-center) [m].
            D_c: Collar diameter (Tube OD + 2*fin_thickness + 2*frost_thickness) [m].
            D_h: Hydraulic diameter [m].
            P_t: Transverse tube pitch [m].
            P_l: Longitudinal tube pitch [m].
            density_avg: Average air density [kg/m^3].
            velocity: Air velocity [m/s].
            heat_capacity_avg: Average air specific heat capacity [J/(kg*K)].
            prandtl_avg: Average air Prandtl number [dimensionless].

        Returns:
            h_conv: The convective heat transfer coefficient [W/(m^2*K)].
        """
        import numpy as np

        # Safety clamps for Logarithms to prevent domain errors
        Re_Dc = max(Re_Dc, 10.0)
        ln_Re = np.log(Re_Dc)

        # --- Heat Transfer (j-factor) ---
        if N == 1:
            P1 = 1.9 - 0.23 * ln_Re
            P2 = -0.236 + 0.126 * ln_Re
            
            j = (0.108 * (Re_Dc**-0.29) * ((P_t / P_l)**P1) * ((F_p / D_c)**-1.084) * ((F_p / D_h)**-0.786) * ((F_p / P_t)**P2))
        else:
            # P3 = -0.361 - (0.042 * N / ln_Re) + 0.158 * np.log(N * (F_p / D_c)**0.41)
            # P4 = -1.224 - (0.076 * ((P_l / D_h)**1.42) / ln_Re)
            # P5 = -0.083 + (0.058 * N / ln_Re)
            # P6 = -5.735 + 1.21 * np.log(Re_Dc / N)

            # j = (0.086 * (Re_Dc**P3) * (N**P4) * ((F_p / D_c)**P5) * ((F_p / D_h)**P6) * ((F_p / P_t)**-0.93))
            print("ERRORRRRRR")
        

        h_conv = (j * density_avg * velocity * heat_capacity_avg) / (prandtl_avg ** (2/3))
        return h_conv



    def _calculate_mass_transfer_coefficient(self, h_conv: float, density: float, heat_capacity: float, lewis_number: float) -> float:
        """
        Calculates the mass transfer coefficient (betta) based on the Chilton-Colburn analogy.

        Args:
            h_conv: Convective heat transfer coefficient (alpha_air) [W/(m^2*K)].
            density: Average air density [kg/m^3].
            heat_capacity: Average air heat capacity (c_p) [J/(kg*K)].
            lewis_number: Average Lewis number (Le) [dimensionless].

        Returns:
            The mass transfer coefficient (betta) [m/s].

        Raises:
            ValueError: If the product of density and heat_capacity is <= 0.
        """
        denominator = density * heat_capacity
        if denominator <= 0:
            raise ValueError(f"Invalid denominator in betta calculation (density * c_p = {denominator}).")

        return (h_conv / denominator) * (lewis_number ** (-2.0 / 3.0))


    def _calculate_mass_flows(self, density: float, flow_area_air: float, velocity: float, W_in: float, W_out: float) -> tuple[float, float]:
        """
        Calculates both humid and dry air mass flow rates.

        Args:
            density: The humid air density [kg/m^3].
            flow_area_air: The effective flow cross-sectional area [m^2].
            velocity: The air velocity [m/s].
            W_in: Humidity ratio at inlet [kg_w/kg_da].
            W_out: Humidity ratio at outlet [kg_w/kg_da].

        Returns:
            A tuple containing:
            1. Humid air mass flow rate [kg/s]
            2. Dry air mass flow rate [kg/s]

        Raises:
            ValueError: If density, area, or velocity are negative.
        """
        if density < 0 or flow_area_air < 0 or velocity < 0:
            raise ValueError(f"Inputs must be non-negative: rho={density}, area={flow_area_air}, vel={velocity}.")

        # Calculate humid mass flow
        m_dot_humid = density * flow_area_air * velocity
        
        # Calculate average humidity ratio
        W_avg = (W_in + W_out) / 2.0
        
        # Calculate dry mass flow
        m_dot_dry = m_dot_humid / (1.0 + W_avg)

        return m_dot_humid, m_dot_dry


    def _get_ice_enthalpy(self, T: float) -> float:
        """
        Calculates the specific enthalpy of ice using ASHRAE Fundamentals formulation.

        Args:
            T: Temperature of the ice [K].

        Returns:
            The specific enthalpy of ice [J/kg].
        """
        import warnings
        
        T_celsius = T - 273.15

        # Robustness check for physical validity
        if T_celsius > 0.1:
            warnings.warn(f"Temperature ({T} K) is above freezing point for ice enthalpy calculation.", RuntimeWarning)

        # Constants derived from ASHRAE Fundamentals (SI Units)
        # Reference: Liquid water at 0.01°C = 0 J/kg.
        h_fusion_ref = -333400.0 
        cp_ice = 2060.0 

        return h_fusion_ref + (cp_ice * T_celsius)
    

    def _get_inlet_dew_point(self, p_in: float, W_in: float, T_in: float) -> float:
        """
        Calculates the dew point temperature of the inlet air using CoolProp.

        Args:
            p_in: The absolute pressure of the inlet air [Pa].
            W_in: The humidity ratio (specific humidity) [kg_water/kg_dry_air].
            T_in: The dry bulb temperature of the inlet air [K].

        Returns:
            The dew point temperature [K].
        """
        return CP_HumidAir.HAPropsSI('Tdp', 'P', p_in, 'W', W_in, 'T', T_in)































    # def _calculate_nusselt(self, Re_Dh: float, Pr: float, D_h:float) -> float:
    #     # Jonas Diss (4.1)
    #     return 0.31 * (Re_Dh**(5/8)) * (Pr**(1/3)) * ((D_h / self.P_l)**(1/3))

    

    # Backup of VDI Nusselt stuff

    
    # def _calculate_nusselt(self, Re_Dh: float, Pr: float, Dh: float, L: float) -> float:
    #     """
    #     Calculates the mean Nusselt number (Nu_m) for flow in a flat gap (ebener Spalt), 
    #     covering laminar, transition, and turbulent flow regimes.

    #     Assumes constant wall temperature. The characteristic length is the hydraulic 
    #     diameter Dh. Fluid properties must be evaluated at the mean fluid temperature ϑm.

    #     Sources: 
    #     - Laminar: VDI-Wärmeatlas, Chapter G2 2.3.2, Gl. (43).
    #     - Transition/Turbulent: VDI-Wärmeatlas, Chapter G1 4.1 (Gnielinski correlations).

    #     :param Re_Dh: Reynolds number based on the hydraulic diameter Dh.
    #     :param Pr: Prandtl number of the fluid.
    #     :param Dh: Hydraulic diameter [m] (Dh = 2 * s).
    #     :param L: Fin length in flow direction [m].
    #     :return: Mean Nusselt number (Nu_m).
    #     """
        
    #     # --- Critical Reynolds Numbers as per VDI G1 ---
    #     RE_CRIT = 2300        # Upper limit for Laminar regime (Re <= 2300)
    #     RE_TURB_START = 4000  # Lower limit for fully developed Turbulent regime (Re >= 4000)

    #     # --- LAMINAR REGIME (Re <= 2300) ---
    #     if Re_Dh <= RE_CRIT:
    #         # Calculate Graetz Number (Gz)
    #         Gz = Re_Dh * Pr * (Dh / L)

    #         # Use Stephan's correlation (VDI 2.3.2, Gl. 43)
    #         return self._stephan_laminar(Gz, Pr)

    #     # --- TURBULENT REGIME (Re >= 4000) ---
    #     elif Re_Dh >= RE_TURB_START:
    #         # Use Gnielinski correlation (VDI G1, Gl. 26/27)
    #         return self._gnielinski_turbulent(Re_Dh, Pr, Dh, L)

    #     # --- TRANSITION REGIME (2300 < Re < 4000) ---
    #     else:
    #         # Interpolation factor (γ) - VDI G1 4.1 Gl. 30
    #         gamma = (Re_Dh - RE_CRIT) / (RE_TURB_START - RE_CRIT)

    #         # Nu at Re=2300 (Nu_m,L,2300)
    #         Gz_crit = RE_CRIT * Pr * (Dh / L)
    #         Nu_m_L_2300 = self._stephan_laminar(Gz_crit, Pr)
            
    #         # Nu at Re=4000 (Nu_m,T,4000)
    #         Nu_m_T_4000 = self._gnielinski_turbulent(RE_TURB_START, Pr, Dh, L)
            
    #         # Interpolation - VDI G1 4.1 Gl. 29
    #         Num_transition = (1.0 - gamma) * Nu_m_L_2300 + gamma * Nu_m_T_4000
            
    #         return Num_transition
        
    # # --- Helper function for laminar flow (VDI G2 2.3.2, Gl. 43) ---
    # def _stephan_laminar(self, Gz: float, Pr: float) -> float:
    #     """
    #     Calculates Nu_m for laminar flow in a flat gap (constant wall temp.).
    #     Gz: Graetz Number (Re_Dh * Pr * Dh / L).
    #     """
    #     # Gl. (43) from VDI G2 2.3.2 for constant wall temperature (Nu_m,II)
    #     numerator = 0.024 * (Gz**1.14)
    #     denominator = 1 + 0.0358 * (Gz**0.64) * (Pr**0.17)
    #     Num_lam = 7.55 + (numerator / denominator)
    #     return Num_lam

    # # --- Helper function for turbulent flow (VDI G1 4.1 26/27) ---
    # def _gnielinski_turbulent(self, Re: float, Pr: float, Dh: float, L: float) -> float:
    #     """
    #     Calculates Nu_m for turbulent flow using the Gnielinski correlation (VDI G1 4.1).
    #     """
    #     # Gl. (27) - Friction factor xi (Druckverlustbeiwert)
    #     xi = (1.8 * np.log10(Re) - 1.5)**(-2)
        
    #     # Gl. (26) - Nusselt number (adapted for flat gap with Dh)
    #     numerator = (xi / 8) * (Re - 1000) * Pr
    #     denominator = 1 + 12.7 * np.sqrt(xi / 8) * (Pr**(2/3) - 1)
        
    #     # Entry length correction term: [1 + (Dh/L)**(2/3)]
    #     Num_turb = (numerator / denominator) * (1 + (Dh / L)**(2/3))
        
    #     return Num_turb






        # # Calculate h_conv
        # D_h = 2 * space_between_frost 

        # Re_Dh = self._calculate_reynolds_number(
        #     density=density_avg, 
        #     velocity=velocity, 
        #     characteristic_length=D_h, 
        #     dyn_viscosity=dyn_viscosity_avg
        # )

        # nusselt = self._calculate_nusselt(
        #     Re_Dh=Re_Dh, 
        #     Pr=prandtl_avg,
        #     D_h=D_h
        # )
        
        # h_conv_raw = self._calculate_heat_transfer_coefficient(
        #     nusselt=nusselt, 
        #     thermal_conductivity=thermal_conductivity_avg, 
        #     characteristic_length=D_h
        # )

        

    # def _calculate_heat_transfer_coefficient(self, nusselt: float, thermal_conductivity: float, characteristic_length: float):
    #     """Calculates the convective heat transfer coefficient (h = Nu * k / L)."""
    #     if characteristic_length == 0:
    #         return 0.0
    #     return (nusselt * thermal_conductivity) / characteristic_length





