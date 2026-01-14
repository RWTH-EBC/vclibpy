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

        # R_refrigerant = self._calculate_thermal_resistance_refrigerant(
        #     h_conv_refrigerant = state.refrigerant.h_conv
        # )
        
        R_downstream = R_frost # + self.R_tube + R_refrigerant (TODO: Add back)


        # --- Heat Transfer ---
        Q_dot_sens = self._calculate_heat_transfer_sensible(
            m_dot_air         = state.air.m_dot_humid,
            heat_capacity_air = state.air.heat_capacity_avg,
            R_air             = R_air,
            T_air_in          = inputs.air.T_in,
            T_frost_surface   = state.hmt.T_frost_surface
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
        T_refrigerant_avg = 0.5 * (state.refrigerant.T_in + state.refrigerant.T_out)
        
        Q_dot_total, T_surface_new, T_base_new = self._calculate_energy_balance_and_temps(
            m_dot_frost_total    = m_dot_frost_total,
            Q_dot_sens           = Q_dot_sens,
            T_refrigerant_avg    = T_refrigerant_avg,
            R_downstream         = R_downstream,
            R_frost              = R_frost,
            h_sublimation        = h_sublimation
        )


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
        state.hmt.set("R_downstream", R_downstream)
        # state.hmt.set("R_refrigerant", R_refrigerant)
        state.hmt.set("R_tube", self.R_tube)
        state.hmt.set("R_frost", R_frost)
        state.hmt.set("R_air", R_air)





    ####################################################################################
    # Helper Functions
    ####################################################################################
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


    def _calculate_heat_transfer_sensible(self, m_dot_air: float, heat_capacity_air: float, R_air: float, T_air_in: float, T_frost_surface: float) -> float:
        """
        Calculates Sensible Heat Transfer using the Epsilon-NTU method.

        Args:
            m_dot_air: Mass flow rate of humid air [kg/s].
            heat_capacity_air: Specific heat capacity of air [J/kgK].
            R_air: Thermal resistance of the air side [K/W].
            T_air_in: Inlet air temperature [K].
            T_frost_surface: Frost surface temperature [K].

        Returns:
            The sensible heat transfer rate [W].
        """       
        # Capacity Rate
        C_air = m_dot_air * heat_capacity_air

        # NTU
        UA_air = 1.0 / R_air
        NTU = UA_air / C_air

        # Effectiveness (surface at constant temp)
        epsilon = 1.0 - math.exp(-NTU)

        # Heat Transfer
        Q_max_sens = C_air * (T_air_in - T_frost_surface)
        Q_dot_sens = epsilon * Q_max_sens
        
        return Q_dot_sens

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


    def _calculate_energy_balance_and_temps(self, m_dot_frost_total: float, Q_dot_sens: float, T_refrigerant_avg: float, 
                                            R_downstream: float, R_frost: float, h_sublimation: float) -> tuple[float, float, float]:
        """
        Calculates Latent Heat, Total Energy, and updates Surface/Base Temperatures.

        Args:
            m_dot_frost_total: Total frost mass accumulation rate [kg/s].
            Q_dot_sens: Sensible heat transfer rate [W].
            T_refrigerant_avg: Average refrigerant temperature [K].
            R_downstream: Thermal resistance downstream of the frost surface [K/W].
            R_frost: Thermal resistance of the frost layer [K/W].
            h_sublimation: Enthalpy of sublimation [J/kg].

        Returns:
            - Q_dot_total: Total heat transfer rate [W].
            - T_surface_new: New frost surface temperature [K].
            - T_base_new: New frost base temperature [K].
        """
        
        # Calculate Total Energy
        Q_dot_lat = m_dot_frost_total * h_sublimation
        Q_dot_total = Q_dot_sens + Q_dot_lat

        # Calculate Temperatures
        T_surface_new = T_refrigerant_avg + (Q_dot_total * R_downstream)
        T_base_new = T_surface_new - (Q_dot_total * R_frost)

        return Q_dot_total, T_surface_new, T_base_new