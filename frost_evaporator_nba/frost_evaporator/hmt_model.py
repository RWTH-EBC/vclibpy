from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import math

import CoolProp.CoolProp as CP_HumidAir


class HeatMassTransferModel:
    
    def __init__(self, 
                 parameters: FrostEvaporatorParameters, 
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
        """
        self.params = parameters
        self.R_tube = self._calculate_thermal_resistance_tube()


    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the heat-mass-transfer model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # ================= Calculate New Values =================
        
        # --- Geometry and Resistances ---
        A_effective, eta_fin = self._calculate_effective_area(
            h_conv_air      = state.air.h_conv,
            frost_thickness = state.frost.thickness,
            k_frost         = state.frost.k_frost
        )

        R_air = self._calculate_resistance_air(
            h_conv_air  = state.air.h_conv, 
            A_effective = A_effective
        )

        R_frost = self._calculate_resistance_frost(
            frost_thickness = state.frost.thickness,
            k_frost         = state.frost.k_frost,
            A_effective     = A_effective
        )

        # Total Thermal Resistance on the Air+Frost+Tube side (excluding refrigerant)
        R_ext_total = R_air + R_frost + self.R_tube

        # Calculate sensible Heat (splittet into 2phase and superheated zones)
        Q_dot_sens = self._calculate_zonal_sensible_heat(
            state = state,
            inputs = inputs,
            R_ext_total = R_ext_total,
            A_effective = A_effective
        )

        # --- Mass Transfer ---
        m_dot_frost_total_mass_transfer = self._calculate_mass_transfer_total(
            m_dot_air   = state.air.m_dot_humid,
            density_air = state.air.density_avg,
            betta_air   = state.air.betta,
            A_effective = A_effective,
            rho_w_in    = state.air.rho_w_in,
            rho_w_surf  = state.air.rho_w_frost_surface_sat
        )

        # --- Saturation Check & Enthalpy ---
        h_sublimation = self._calculate_enthalpy_sublimation(T_frost_surface=state.hmt.T_frost_surface)

        # Check that the outlet relative humidity does not exceed 100%
        # If it does, clamp humidity and recalculate mass/energy rates.
        m_dot_frost_total, Q_dot_sens = self.enforce_outlet_saturation_ceiling(
            h_in_air                        = state.air.h_in,
            W_in                            = inputs.air.W_in,
            m_dot_dry                       = state.air.m_dot_dry,
            Q_dot_sens                      = Q_dot_sens,
            m_dot_frost_total_mass_transfer = m_dot_frost_total_mass_transfer,
            p_avg                           = state.air.pressure_avg,
            h_sublimation                   = h_sublimation,
            h_ice                           = state.air.h_ice
        )

        # --- Mass Split (Densification vs Thickening) ---
        m_dot_dens, m_dot_thick, m_dot_thick_flux = self._calculate_mass_flow_split_Fick(
            frost_thickness = state.frost.thickness,
            frost_density   = state.frost.density,
            m_dot_total     = m_dot_frost_total,
            rho_w_surf      = state.air.rho_w_frost_surface_sat,
            rho_w_base      = state.air.rho_w_frost_base_sat,
            A_frost_surface = state.frost.A_frost_surface
        )

        # --- Energy Balance & Temperatures ---
        Q_dot_lat = m_dot_frost_total * h_sublimation
        Q_dot_total = Q_dot_sens + Q_dot_lat

        T_air_avg = 0.5 * (inputs.air.T_in + state.air.T_out)
        T_surface_new = T_air_avg - (Q_dot_sens * R_air)
        T_base_new = T_surface_new - (Q_dot_total * R_frost)



        # ================= Write new values to state =================
        state.hmt.set("m_dot_densification", m_dot_dens)
        state.hmt.set("m_dot_thickening", m_dot_thick)
        state.hmt.set("m_dot_thickening_flux", m_dot_thick_flux)
        state.hmt.set("m_dot_frost_total", m_dot_frost_total)
        
        state.hmt.set("Q_dot_total", Q_dot_total)
        state.hmt.set("Q_dot_sens", Q_dot_sens)
        
        state.hmt.set("T_frost_surface", T_surface_new)
        state.hmt.set("T_frost_base", T_base_new)
        
        state.hmt.set("A_effective", A_effective)
        state.hmt.set("eta_fin", eta_fin)
        state.hmt.set("R_tube", self.R_tube)
        state.hmt.set("R_frost", R_frost)
        state.hmt.set("R_air", R_air)





    ####################################################################################
    # Helper Functions
    ####################################################################################


    def _calculate_zonal_sensible_heat(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs, R_ext_total: float, A_effective: float) -> float:
        """
        Calculates the total sensible heat transfer by splitting the coil into 
        Two-Phase and Superheated zones.
        """
        # 1. Retrieve Zonal Data
        portion_2ph = state.refrigerant.portion_two_phase
        portion_sh  = state.refrigerant.portion_superheated
        
        h_conv_2ph = state.refrigerant.h_conv_two_phase
        h_conv_sh  = state.refrigerant.h_conv_superheated

        T_2ph_in = state.refrigerant.T_two_phase_in
        T_sh_in  = state.refrigerant.T_superheated_in
        
        T_air_in = inputs.air.T_in

        
        # Mass Flows
        m_dot_air_total = state.air.m_dot_humid
        cp_air = state.air.heat_capacity_avg
        m_dot_ref = inputs.refrigerant.m_dot
        cp_ref_sh = state.refrigerant.heat_capacity_avg_superheated
        

        Q_2ph = 0.0
        Q_sh = 0.0

        A_inner_total = np.pi * self.params.tube_inner_diameter * self.params.total_tube_length

        # --- ZONE 1: Two-Phase (Evaporation) ---
        if portion_2ph > 1e-6:
            """ Assuming Evaporation at Constant Temperature """

            # Refrigerant Resistance in this zone (We scale the global tube inner area by the portion)
            R_ref_2ph = 1.0 / (h_conv_2ph * A_inner_total * portion_2ph)
            
            # External Resistance (Air+Frost+Tube) scales inversely with Area fraction
            R_ext_2ph = R_ext_total / portion_2ph
            
            # Total R for Zone 1
            R_2ph = R_ext_2ph + R_ref_2ph
            
            # Air Capacity Rate for this zone
            C_air_2ph = m_dot_air_total * portion_2ph * cp_air
            
            # Effectiveness (C_min/C_max = 0 because T_ref is constant)
            NTU_2ph = 1.0 / (R_2ph * C_air_2ph)
            epsilon_2ph = 1.0 - math.exp(-NTU_2ph)
            
            # Heat Transfer
            Q_2ph = epsilon_2ph * C_air_2ph * (T_air_in - T_2ph_in)


        # --- ZONE 2: Superheated (Gas) ---
        if portion_sh > 1e-6:
            """ Crossflow with one tube row from VDI-Wärmeatlas C1 """


            # Refrigerant Resistance in this zone (We scale the global tube inner area by the portion)
            R_ref_sh = 1.0 / (h_conv_sh * A_inner_total * portion_sh)

            # External Resistance (Air+Frost+Tube) scales inversely with Area fraction
            R_ext_sh = R_ext_total / portion_sh

            # Total R for Zone 2
            R_sh = R_ext_sh + R_ref_sh
            
            # Capacity Rates for this zone          
            C_ref_sh = m_dot_ref * cp_ref_sh
            C_air_sh = m_dot_air_total * portion_sh * cp_air
            C_min = min(C_ref_sh, C_air_sh)
            C_max = max(C_ref_sh, C_air_sh)
            C_r = C_min / C_max

            # Effectiveness

            
            NTU_sh = 1 / (R_sh * C_min)
            
            # Cross-Flow Epsilon (Both fluids Unmixed) - Standard for Finned Tubes
            exponent_inner = -C_r * (NTU_sh**0.78)
            term_inner = math.exp(exponent_inner) - 1.0
            exponent_outer = (NTU_sh**0.22 / C_r) * term_inner
            epsilon_sh = 1.0 - math.exp(exponent_outer)

            # Heat Transfer
            Q_sh = epsilon_sh * C_min * (T_air_in - T_sh_in)

        return Q_2ph + Q_sh



    def _calculate_effective_area(self, h_conv_air: float, frost_thickness: float, k_frost: float) -> tuple[float, float]:
        """
        Calculates the effective heat transfer area of the finned tube.
        This function works for "Fluchtende Rohre" only.

        Args:
            h_conv_air: The convective heat transfer coefficient of the air [W/m²K].
            frost_thickness: The thickness of the frost layer [m].
            k_frost: The thermal conductivity of the frost layer [W/mK].
            
        Returns:
            - A_effective: The effective heat transfer area [m²].
            - eta_fin: The fin efficiency [-].
        """

        # Calculate the heat transfer Coefficient of air and frost in series
        h_effective = self._calculate_effective_heat_transfer_coefficient(h_conv_air, frost_thickness,  k_frost)

        # Calculate fin efficiency
        eta_fin_raw = self._calculate_fin_efficiency(h_effective)
        eta_fin = eta_fin_raw * self.params.correction_factor_eta_fin

        # Calculate area of one fin and one tube segment
        A_one_tube_segment = np.pi * self.params.tube_outer_diameter * (self.params.fin_pitch - self.params.fin_thickness)
        A_one_fin_segment  = 2 * ( (self.params.fin_segment_height * self.params.fin_segment_length) - (0.25 * np.pi * self.params.tube_outer_diameter**2) )

        A_effective = self.params.fin_segment_amount * (A_one_tube_segment + eta_fin * A_one_fin_segment)

        return A_effective, eta_fin

    
    def _calculate_effective_heat_transfer_coefficient(self, h_conv_air: float, frost_thickness: float, k_frost: float) -> float:
        """
        Calculates the combined heat transfer coefficient for convection from air through a layer of frost.
        Treats convective and conductive layers as thermal resistances in series.

        Args:
            h_conv_air: The convective heat transfer coefficient of the air [W/m²K].
            frost_thickness: The thickness of the frost layer [m].
            k_frost: The thermal conductivity of the frost layer [W/mK].

        Returns:
            The combined heat transfer coefficient of air and frost [W/m²K].
        """        
        # Handle no-frost case
        if np.isclose(frost_thickness, 0):
            return h_conv_air

        # Check for zero denominators
        if np.isclose(h_conv_air, 0):
            raise ValueError("h_conv_air cannot be zero.") 
        if np.isclose(k_frost, 0):
            raise ValueError("k_frost cannot be zero.") 

        return 1 / (1 / h_conv_air + frost_thickness / k_frost)
    
    def _calculate_fin_efficiency(self, h_effective: float) -> float:
        # sourcery skip: assign-if-exp, reintroduce-else
        """
        Calculates the efficiency of a rectangular fin.

        The calculation procedure is based on the VDI Wärmeatlas,
        "M1 Wärmeübergang an berippten Rohren".

        Args:
            h_effective: The combined heat transfer coefficient of air and frost [W/m²K].

        Returns:
            Fin efficiency (eta_fin), dimensionless.
        """
        
        if np.isclose(self.params.tube_outer_diameter, 0):
            raise ValueError("tube_outer_diameter cannot be zero.")

        if np.isclose(self.params.fin_thickness, 0):
            raise ValueError("fin_thickness cannot be zero.")
        
        if np.isclose(self.params.fin_thermal_conductivity, 0):
            raise ValueError("fin_thermal_conductivity cannot be zero.")
        
        if np.isclose(self.params.fin_segment_length, 0) or np.isclose(self.params.fin_segment_height, 0):
            raise ValueError("fin_segment_length and fin_segment_height cannot be zero.")
            
        # Equation (13) from VDI Wärmeatlas M1
        phi_dash = (1.28 * self.params.fin_segment_height / self.params.tube_outer_diameter *
                    np.sqrt((self.params.fin_segment_length / self.params.fin_segment_height) - 0.2))

        # Equation (12) from VDI Wärmeatlas M1
        phi = (phi_dash - 1) * (1 + 0.35 * np.log(phi_dash))

        # Equation (8) from VDI Wärmeatlas M1
        term_in_sqrt = (2 * h_effective) / (self.params.fin_thermal_conductivity * self.params.fin_thickness)
        X = phi * (self.params.tube_outer_diameter / 2) * np.sqrt(term_in_sqrt)

        # Equation (7) and (9) from VDI Wärmeatlas M1
        if X < 1e-6: return 1.0
        
        return np.tanh(X) / X

    def _calculate_resistance_air(self, h_conv_air: float, A_effective: float) -> float:
        """
        Calculates the convective thermal resistance on the air side.

        Args:
            h_conv_air: Air convective heat transfer coefficient [W/m^2K].
            A_effective: Effective heat transfer area [m^2].
        Returns:
            Thermal resistance [K/W].
        Raises:
            ValueError: If inputs are zero.
        """
        if np.isclose(h_conv_air, 0) or np.isclose(A_effective, 0):
             raise ValueError("Zero value in resistance calc")
        return 1 / (h_conv_air * A_effective)

    def _calculate_resistance_frost(self, frost_thickness: float, k_frost: float, A_effective: float) -> float:
        """
        Calculates the conductive thermal resistance of the frost layer.

        Args:
            frost_thickness: Thickness of the frost [m].
            k_frost: Thermal conductivity of frost [W/mK].
            A_effective: Effective heat transfer area [m^2].
        Returns:
            Thermal resistance [K/W].
        Raises:
            ValueError: If conductivity or area is zero.
        """
        if np.isclose(k_frost, 0) or np.isclose(A_effective, 0):
             raise ValueError("Zero value in resistance calc")
        return frost_thickness / (k_frost * A_effective)
    
    def _calculate_thermal_resistance_tube(self) -> float:
        """
        Calculates the conductive thermal resistance through the tube wall.
        
        Returns:
            The tube thermal resistance [K/W].
        Raises:
            ValueError: If tube dimensions or conductivity are invalid.
        """

        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        if np.isclose(self.params.tube_thermal_conductivity, 0):
            raise ValueError("Tube conductivity cannot be zero.")
        
        return ( np.log(self.params.tube_outer_diameter / self.params.tube_inner_diameter) / 
                 (2 * np.pi * self.params.total_tube_length * self.params.tube_thermal_conductivity) )

    def _calculate_thermal_resistance_refrigerant(self, h_conv_refrigerant: float) -> float:
        """
        Calculates the convective thermal resistance on the refrigerant side.
        
        Args:
            h_conv_refrigerant: Refrigerant heat transfer coefficient [W/m^2K].
        Returns:
            Refrigerant side resistance [K/W].
        """
        if np.isclose(h_conv_refrigerant, 0):
            raise ValueError("h_conv_refrigerant cannot be zero.")
        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        
        return 1 / (h_conv_refrigerant * np.pi * self.params.tube_inner_diameter * self.params.total_tube_length)

    def _calculate_mass_transfer_total(self, m_dot_air: float, density_air: float, betta_air: float, 
                                       A_effective: float, rho_w_in: float, rho_w_surf: float) -> float:
        """
        Calculates Total Mass Transfer using Modified Epsilon-NTU.

        Args:
            m_dot_air: Mass flow rate of humid air [kg/s].
            density_air: Average density of air [kg/m^3].
            betta_air: Mass transfer coefficient [m/s].
            A_effective: Effective surface area [m^2].
            rho_w_in: Water vapor density at inlet [kg/m^3].
            rho_w_surf: Water vapor density at frost surface [kg/m^3].

        Returns:
            The total mass transfer rate of frost [kg/s].
        """
        # Volume Flow Rate [m^3/s]
        V_dot_air = m_dot_air / density_air

        # NTU Mass
        # (Mass Transfer Coeff * Area) / Volume Flow
        NTU_mass = (betta_air * A_effective) / V_dot_air

        # Effectiveness Mass
        epsilon_mass = 1.0 - math.exp(-NTU_mass)

        # Mass Transfer
        m_dot_frost_max = V_dot_air * (rho_w_in - rho_w_surf)
        m_dot_frost_total = epsilon_mass * m_dot_frost_max

        # Clamp Negative Mass Transfer to 0 
        m_dot_frost_total = max(0.0, m_dot_frost_total)
        
        return m_dot_frost_total


    def _calculate_enthalpy_sublimation(self, T_frost_surface: float) -> float:
        """
        Calculates the latent heat of sublimation (Ice -> Vapor) 
        as a function of surface temperature.
        
        Args:
            T_frost_surface: Surface temperature in Kelvin.
            
        Returns:
            Enthalpy of sublimation [J/kg]
        """
        T_celsius = T_frost_surface - 273.15
        
        # Linear approximation for h_sublimation [J/kg]
        # h_sub is approx 2.834e6 J/kg at 0°C and rises slightly as T drops.
        return (2834.3 - 0.29 * T_celsius) * 1000.0


    def enforce_outlet_saturation_ceiling(self, h_in_air: float, W_in: float, m_dot_dry: float, Q_dot_sens: float, m_dot_frost_total_mass_transfer: float, 
                                          p_avg: float, h_sublimation: float, h_ice: float)-> tuple[float, float]:
        """
        Ensures the air outlet state does not exceed 100% Relative Humidity.
        If it does, the mass transfer and sensible heat are adjusted.

        Args:
            h_in_air: Inlet air enthalpy [J/kg].
            W_in: Inlet humidity ratio [kg_w/kg_da].
            m_dot_dry: Dry air mass flow rate [kg/s].
            Q_dot_sens: Initial sensible heat transfer rate [W].
            m_dot_frost_total_mass_transfer: Initial total mass transfer rate [kg/s].
            p_avg: Average air pressure [Pa].
            h_sublimation: Enthalpy of sublimation [J/kg].
            h_ice: Enthalpy of ice [J/kg].

        Returns:
            - m_dot_frost_total: Corrected total frost mass flow rate [kg/s].
            - Q_dot_sens: Corrected sensible heat transfer rate [W].
        """
        m_dot_frost_total = m_dot_frost_total_mass_transfer

        # Predict outlet state based on initial mass/energy balance
        energy_removed = Q_dot_sens + m_dot_frost_total * (h_sublimation + h_ice)
        h_out_predicted = h_in_air - (energy_removed / m_dot_dry)
        W_out_predicted = W_in - (m_dot_frost_total / m_dot_dry)

        # Determine saturation limit at the predicted enthalpy
        try:
            W_sat_at_h_out = CP_HumidAir.HAPropsSI('W', 'H', h_out_predicted, 'P', p_avg, 'R', 0.99)
        except ValueError:
            # Fallback for numerical instability near zero/limits
            W_sat_at_h_out = W_in

        # Check for supersaturation and clamp if necessary
        if W_out_predicted > W_sat_at_h_out:
            # Clamp outlet humidity to saturation
            W_out_new = W_sat_at_h_out

            # Back-calculate the new mass transfer rate
            m_dot_frost_total = m_dot_dry * (W_in - W_out_new)

            # Back-calculate Sensible Heat to conserve enthalpy
            # Q_sens = Total_Enthalpy_Change - Energy_Removed_by_Mass
            enthalpy_change_air = m_dot_dry * (h_in_air - h_out_predicted)
            energy_removed_by_mass = m_dot_frost_total * (h_sublimation + h_ice)
            
            Q_dot_sens = max(0.0, enthalpy_change_air - energy_removed_by_mass)
        return m_dot_frost_total, Q_dot_sens
        

    def _calculate_mass_flow_split_Fick(self, frost_thickness: float, frost_density: float, m_dot_total: float, 
                                   rho_w_surf: float, rho_w_base: float, A_frost_surface: float) -> tuple[float, float, float]:
        """
        Determines how much mass flow contributes to densification vs thickening.
        This Function is based on Fick's Law of Diffusion.

        Args:
            frost_thickness: Current frost thickness [m].
            frost_density: Current frost density [kg/m^3].
            m_dot_total: Total frost mass flow rate [kg/s].
            rho_w_surf: Vapor density at surface [kg/m^3].
            rho_w_base: Vapor density at base [kg/m^3].
            A_frost_surface: Surface area of frost [m^2].

        Returns:
            - m_dot_densification: Mass flow rate for densification [kg/s].
            - m_dot_thickening: Mass flow rate for thickening [kg/s].
            - m_dot_thickening_flux: Mass flux for thickening [kg/(m^2s)].
        """        

        # Check for zero total mass flow
        if m_dot_total > 0.0:
            if frost_thickness <= 1e-6:
                # Layer too thin for internal diffusion
                return 0.0, m_dot_total, (m_dot_total / A_frost_surface)

            # Porosity and Diffusivity
            porosity = 1.0 - (frost_density / self.params.ice_density)
            porosity = max(0.0, min(1.0, porosity))

            D_AB = self.params.diffusivity_w_vapor_in_air
            D_eff = D_AB * porosity

            # Densification Flux (Fick's Law)
            gradient_rho = (rho_w_surf - rho_w_base) / frost_thickness
            m_dot_densification_flux = D_eff * gradient_rho
            m_dot_densification = m_dot_densification_flux * A_frost_surface

            # Clamping
            m_dot_densification = max(0.0, m_dot_densification)
            m_dot_densification = min(m_dot_densification, m_dot_total)
            
            # Thickening
            m_dot_thickening = m_dot_total - m_dot_densification
            m_dot_thickening_flux = m_dot_thickening / A_frost_surface

            return m_dot_densification, m_dot_thickening, m_dot_thickening_flux
        else:
            return 0.0, 0.0, 0.0