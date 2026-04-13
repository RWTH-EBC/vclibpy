from ..datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import warnings
import CoolProp.CoolProp as CP_HumidAir

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


    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the air model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # ================= Calculate new Values =================

        # --- Average Air Properties ---
        (pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, thermal_conductivity_avg, 
         prandtl_avg, lewis_avg, rho_w_in, rho_w_out, h_in, h_out, R_in, R_out, T_avg, W_avg, p_avg) = self._calculate_air_averages(
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

        # --- Geometry Definition ---
        # Hydraulic Diameter (for VDI)
        D_h = 4 * state.frost.flow_area_air * self.params.fvm_fin_length / state.frost.A_frost_surface
        
        # Collar Diameter (Effective diameter for Wang)
        D_c_eff = self.params.tube_outer_diameter + 2 * self.params.fin_thickness + 2 * state.frost.thickness

        # --- Heat Transfer Coefficient Calculation ---
        h_conv_raw, reynolds = self.calculate_h_conv(
            velocity                 = state.air.velocity,
            density_avg              = density_avg,
            dyn_viscosity_avg        = dyn_viscosity_avg,
            heat_capacity_avg        = heat_capacity_avg,
            thermal_conductivity_avg = thermal_conductivity_avg,
            prandtl_avg              = prandtl_avg,
            D_c                      = D_c_eff,
            D_h                      = D_h
        )

        # --- Mass Transfer Coefficient ---
        betta_raw = self._calculate_mass_transfer_coefficient(
            h_conv        = h_conv_raw,
            density       = density_avg,
            heat_capacity = heat_capacity_avg,
            lewis_number  = lewis_avg
        )

        # Multiply with manual correction factor
        h_conv = h_conv_raw * self.params.correction_factor_h_conv_air

        # Bound Fin Spacing (Bounded by Experimental data at 1.75 and 7 mm):
        bound_spacing = np.clip(self.params.fin_pitch, 0.00175, 0.007)

        correction_factor_betta_air = self.params.correction_factor_betta_intercept_air + self.params.correction_factor_betta_slope_air * bound_spacing
        betta = betta_raw * correction_factor_betta_air

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
        
        # --- Air Properties ---
        state.air.set("pressure_avg", pressure_avg)
        state.air.set("density_avg", density_avg)
        state.air.set("dyn_viscosity_avg", dyn_viscosity_avg)
        state.air.set("heat_capacity_avg", heat_capacity_avg)
        state.air.set("thermal_conductivity_avg", thermal_conductivity_avg)
        state.air.set("prandtl_avg", prandtl_avg)
        state.air.set("lewis_avg", lewis_avg)
        state.air.set("reynolds", reynolds)

        state.air.set("T_avg", T_avg)
        state.air.set("W_avg", W_avg)
        state.air.set("p_avg", p_avg)

        # --- Humid Air / Water Vapor ---
        state.air.set('rho_w_in', rho_w_in)
        state.air.set('rho_w_out', rho_w_out)
        state.air.set("rho_w_frost_surface_sat", rho_w_frost_surface_sat)
        state.air.set("W_frost_surface_sat", W_frost_surface_sat)
        state.air.set("rho_w_frost_base_sat", rho_w_frost_base_sat)
        state.air.set("W_frost_base_sat", W_frost_base_sat)
        state.air.set("T_dew_point", T_dew_point)
        state.air.set("R_in", R_in)
        state.air.set("R_out", R_out)

        # --- Flow & Heat Transfer ---
        state.air.set("h_conv", h_conv)
        state.air.set("betta", betta)
        
        # --- Mass Flow ---
        state.air.set("m_dot_humid", m_dot_humid)
        state.air.set("m_dot_dry", m_dot_dry)

        # --- Enthalpies ---
        state.air.set("h_in", h_in)
        state.air.set("h_out", h_out)
        state.air.set("h_ice", h_ice)



    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _calculate_air_averages(self, T_in: float, p_in: float, W_in: float, T_out: float, p_out: float, W_out: float
                                ) -> tuple[float, float, float, float, float, float, float, float, float, float, float, float, float, float]:
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
            11. Inlet Relative Humidity [-]
            12. Outlet Relative Humidity [-]
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
        T_avg = (T_in + T_out) / 2.0
        W_avg = (W_in + W_out) / 2.0
        p_avg = (p_in + p_out) / 2.0

        return (pressure_avg, density_avg, viscosity_avg, heat_capacity_avg, conductivity_avg, prandtl_avg, lewis_avg,
                rho_w_in, rho_w_out, h_in, h_out, R_in, R_out, T_avg, W_avg, p_avg)
    

    def _get_air_properties(self, T: float, p: float, W: float) -> tuple:
        """
        Calculates thermophysical properties of moist air, handling Fog (W > W_sat) correctly.

        Args:
            T: The dry bulb temperature [K].
            p: The absolute pressure [Pa].
            W: The humidity ratio (specific humidity) [kg_water/kg_dry_air].
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
            8. Relative Humidity [-]
        """

        # Calculate Saturation Humidity Ratio for current Air
        W_sat = CP_HumidAir.HAPropsSI('W', 'T', T, 'P', p, 'R', 1.0)

        # Separate phases: Gas cannot hold more than W_sat
        W_gas = min(W, W_sat)           # The humidity actually dissolved in the air
        W_liquid = max(0.0, W - W_sat)  # The excess humidity (fog)

        # Calculate Gas Phase Properties
        prop_keys = ['Vha', 'mu', 'cp_ha', 'k', 'Enthalpy']
        props = {key: CP_HumidAir.HAPropsSI(key, 'T', T, 'P', p, 'W', W_gas) for key in prop_keys}

        # Calculate Prandtl and Lewis numbers for the gas phase
        prandtl_number = (props['cp_ha'] * props['mu']) / props['k']
        lewis_number = 0.9

        # Calculate air density and water vapor density
        density = (1.0 + W) / props['Vha']
        water_vapor_density, _ = self._get_water_vapor_density(T=T, p=p, W=W_gas)

        # Handle Fog Corrections
        if W_liquid > 0.0:           
            if T >= 273.15:
                # Get Liquid Water Properties at T
                h_condensed = CP_HumidAir.PropsSI('H', 'T', T, 'Q', 0, 'Water')  
                cp_condensed = CP_HumidAir.PropsSI('C', 'T', T, 'Q', 0, 'Water') 
            else:
                # Get Ice Properties at T
                h_condensed = self._get_ice_enthalpy(T)
                cp_condensed = 2060.0  # Approx specific heat capacity of ice [J/kg*K]

            # Correct the air enthalpy and specific heat capacity
            props['Enthalpy'] += (W_liquid * h_condensed)
            props['cp_ha'] += (W_liquid * cp_condensed)

            # Set the air relative humidity (saturation)
            rel_humidity = 1.0
            
        else:
            # Calculate air relative humidity
            rel_humidity = CP_HumidAir.HAPropsSI('R', 'T', T, 'P', p, 'W', W)
        
        return (
            density,
            props['mu'],
            props['cp_ha'],
            props['k'],
            prandtl_number,
            lewis_number,
            water_vapor_density,
            props['Enthalpy'],
            rel_humidity
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
            raise ValueError("Must provide either W (humidity ratio) or R (relative humidity)")

        # Get the specific volume Vda (m^3 / kg_dry_air)
        Vda = CP_HumidAir.HAPropsSI('Vda', 'T', T, 'P', p, 'W', W_calc)
        
        # Calculate the water vapor density:
        rho_w = W_calc / Vda
        return rho_w, W_calc

    def calculate_h_conv(self, velocity: float, density_avg: float, dyn_viscosity_avg: float, 
                         heat_capacity_avg: float, thermal_conductivity_avg: float, prandtl_avg: float, 
                         D_c: float, D_h: float) -> tuple[float, float]:
        # sourcery skip: inline-variable, switch
        """
        Calculates h_conv based on the correlation selected in self.params.h_conv_air_correlation_choice.
        
        Supported Choices:
            - 'Wang'
            - 'VDI'
        """
        choice = self.params.h_conv_air_correlation_choice

        if choice == "Wang":
            # Wang uses Reynolds based on Collar Diameter (D_c)
            Re = (density_avg * velocity * D_c) / dyn_viscosity_avg

            h_conv =  self._calculate_h_conv_wang(
                            Re_Dc             = Re,
                            D_c               = D_c,
                            D_h               = D_h,
                            density_avg       = density_avg,
                            velocity          = velocity,
                            heat_capacity_avg = heat_capacity_avg,
                            prandtl_avg       = prandtl_avg
                        )
            
        elif choice == "VDI":
            # VDI uses Reynolds based on Hydraulic Diameter (D_h) and Flow Length (L)
            Re = (density_avg * velocity * D_h) / dyn_viscosity_avg

            # Assuming Flow Length L is the depth of the tube bank
            L = self.params.fvm_tube_layers * self.params.fin_segment_length
            
            nu = self._calculate_nusselt_vdi(
                            Re_Dh=Re, 
                            Pr=prandtl_avg, 
                            Dh=D_h, 
                            L=L
                        )

            # Calculate the h_conv from Nusselt Number
            h_conv = (nu * thermal_conductivity_avg) / D_h

        else:
            raise ValueError(f"Unknown h_conv correlation choice: '{choice}'. Valid options: 'Wang', 'VDI'.")
        
        return h_conv, Re


    def _calculate_h_conv_wang(self, Re_Dc: float, D_c: float, D_h: float, density_avg: float, velocity: float, 
                               heat_capacity_avg: float, prandtl_avg: float ) -> float:
        """
        Calculates Colburn j-factor using Wang et al. (2000) correlations.

        Args:
            Re_Dc: Reynolds number based on Collar Diameter [dimensionless].
            D_c: Collar diameter (Tube OD + 2*fin_thickness + 2*frost_thickness) [m].
            D_h: Hydraulic diameter [m].
            density_avg: Average air density [kg/m^3].
            velocity: Air velocity [m/s].
            heat_capacity_avg: Average air specific heat capacity [J/(kg*K)].
            prandtl_avg: Average air Prandtl number [dimensionless].
        Returns:
            h_conv: The convective heat transfer coefficient [W/(m^2*K)].
        """
        
        # Safety clamps for Logarithms to prevent domain errors
        Re_Dc = max(Re_Dc, 10.0)
        ln_Re = np.log(Re_Dc)

        # The global tube layers has to be used, because of the influence of the entry region on the heat transfer.
        N   = self.params.global_tube_layers
        F_p = self.params.fin_pitch
        P_t = self.params.transverse_tube_pitch
        P_l = self.params.longitudinal_tube_pitch

        # --- Heat Transfer (j-factor) ---
        if N == 1:
            P1 = 1.9 - 0.23 * ln_Re
            P2 = -0.236 + 0.126 * ln_Re
            
            j = (0.108 * (Re_Dc**-0.29) * ((P_t / P_l)**P1) * ((F_p / D_c)**-1.084) * ((F_p / D_h)**-0.786) * ((F_p / P_t)**P2))
        else:
            # Equations for N >= 2 from Wang et al. (2000)
            P3 = -0.361 - (0.042 * N / ln_Re) + 0.158 * np.log(N * (F_p / D_c)**0.41)
            P4 = -1.224 - (0.076 * (P_l / D_h)**1.42) / ln_Re
            P5 = -0.083 + (0.058 * N / ln_Re)
            P6 = -5.735 + 1.21 * np.log(Re_Dc / N)
            
            j = 0.086 * (Re_Dc**P3) * (N**P4) * ((F_p / D_c)**P5) * ((F_p / D_h)**P6) * ((F_p / P_t)**-0.93)

        # Convert j-factor back to convective heat transfer coefficient
        h_conv = (j * density_avg * velocity * heat_capacity_avg) / (prandtl_avg ** (2/3))
        h_conv = max(1.0, h_conv) # Prevent unphysical low values
        
        return h_conv
    
    def _calculate_nusselt_vdi(self, Re_Dh: float, Pr: float, Dh: float, L: float) -> float:
        """
        Calculates the mean Nusselt number (Nu_m) for flow in a flat gap (ebener Spalt).

        Sources: 
        - Laminar: VDI-Wärmeatlas, Chapter G2 2.3.2, Gl. (43).
        - Transition/Turbulent: VDI-Wärmeatlas, Chapter G1 4.1 (Gnielinski).

        Args:
            Re_Dh: Reynolds number based on the hydraulic diameter Dh [-].
            Pr: Prandtl number of the fluid [-].
            Dh: Hydraulic diameter [m] (Dh = 2 * s).
            L: Fin length in flow direction [m].
        Returns:
            Mean Nusselt number (Nu_m) [-].
        """
        # Limits
        RE_CRIT = 2300.0        # Upper limit for Laminar regime
        RE_TURB_START = 4000.0  # Lower limit for fully developed Turbulent regime

        # --- LAMINAR REGIME ---
        if Re_Dh <= RE_CRIT:
            # Calculate Graetz Number (Gz)
            Gz = Re_Dh * Pr * (Dh / L)
            return self._stephan_laminar(Gz, Pr)

        # --- TURBULENT REGIME ---
        elif Re_Dh >= RE_TURB_START:
            return self._gnielinski_turbulent(Re_Dh, Pr, Dh, L)

        # --- TRANSITION REGIME ---
        else:
            # Interpolation factor (gamma) - VDI G1 4.1 Gl. 30
            gamma = (Re_Dh - RE_CRIT) / (RE_TURB_START - RE_CRIT)

            # Nu at Re=2300 (Laminar limit)
            Gz_crit = RE_CRIT * Pr * (Dh / L)
            Nu_m_L_2300 = self._stephan_laminar(Gz_crit, Pr)
            
            # Nu at Re=4000 (Turbulent limit)
            Nu_m_T_4000 = self._gnielinski_turbulent(RE_TURB_START, Pr, Dh, L)
            
            # Linear Interpolation - VDI G1 4.1 Gl. 29
            return (1.0 - gamma) * Nu_m_L_2300 + gamma * Nu_m_T_4000
        
    def _stephan_laminar(self, Gz: float, Pr: float) -> float:
        """
        Helper: Calculates Nu_m for laminar flow (VDI G2 2.3.2, Gl. 43).
        """
        # Constant wall temperature assumption (Nu_m,II)
        numerator = 0.024 * (Gz**1.14)
        denominator = 1 + 0.0358 * (Gz**0.64) * (Pr**0.17)
        return 7.55 + (numerator / denominator)

    def _gnielinski_turbulent(self, Re: float, Pr: float, Dh: float, L: float) -> float:
        """
        Helper: Calculates Nu_m for turbulent flow (VDI G1 4.1 26/27).
        """
        # Gl. (27) - Friction factor xi (Druckverlustbeiwert)
        xi = (1.8 * np.log10(Re) - 1.5)**(-2)
        
        # Gl. (26) - Nusselt number (adapted for flat gap with Dh)
        numerator = (xi / 8) * (Re - 1000) * Pr
        denominator = 1 + 12.7 * np.sqrt(xi / 8) * (Pr**(2/3) - 1)
        
        # Entry length correction term: [1 + (Dh/L)**(2/3)]
        return (numerator / denominator) * (1 + (Dh / L)**(2/3))



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
        T_celsius = T - 273.15

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