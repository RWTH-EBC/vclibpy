from ..datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import math
from typing import Dict, Tuple, Callable, Any
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
        self.R_tube = self._calculate_resistance_tube()


    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Main function to update all properties related to heat and mass transfer based on the current state and inputs.
        """

        # ================= Calculate new Values =================

        # --- Efficiency Calculations ---
        A_effective, eta_surface = self._calculate_surface_efficiency_terms(state, state.air.h_conv)

        # --- Area Splits ---
        f_2ph, f_sh = self._calculate_area_splits(state, inputs, eta_surface)

        # --- Process Zones ---
        results_2ph = self._process_two_phase_zone(state, inputs, eta_surface, f_2ph)
        results_sh  = self._process_superheated_zone(state, inputs, eta_surface, f_sh)

        # Aggregate Results (Area-weighted averages)
        Q_total = results_2ph["Q"] + results_sh["Q"]
        m_dot_frost_total = results_2ph["m_dot"] + results_sh["m_dot"]

        # Calculate weighted average surface temperatures
        T_surf_avg = results_2ph["T_surf"] * f_2ph + results_sh["T_surf"] * f_sh
        T_base_avg = results_2ph["T_base"] * f_2ph + results_sh["T_base"] * f_sh

        # Frost Massflow Split (Densification vs Thickening)
        rho_w_surf = self._get_rho_w_sat(T_surf_avg, state.air.p_avg)
        rho_w_base = self._get_rho_w_sat(T_base_avg, state.air.p_avg)

        m_dens, m_thick, m_thick_flux, m_dens_flux = self._calculate_mass_flow_split_Fick(
            frost_thickness = state.frost.thickness, 
            frost_density   = state.frost.density, 
            m_dot_total     = m_dot_frost_total,
            rho_w_surf      = rho_w_surf, 
            rho_w_base      = rho_w_base, 
            A_frost_surface = state.frost.A_frost_surface,
            T_frost_surface = T_surf_avg
        )

        # Energy Split (Latent vs Sensible)
        h_sub = self._calculate_enthalpy_sublimation(T_surf_avg)
        Q_latent = m_dot_frost_total * h_sub
        Q_sensible = Q_total - Q_latent 



        # ================= Write new values to state =================
        
        state.hmt.set("m_dot_densification", m_dens)
        state.hmt.set("m_dot_thickening", m_thick)
        state.hmt.set("m_dot_thickening_flux", m_thick_flux)
        state.hmt.set("m_dot_densification_flux", m_dens_flux)
        state.hmt.set("m_dot_frost_total", m_dot_frost_total)

        state.hmt.set("Q_dot_total", Q_total)
        state.hmt.set("Q_dot_sens", Q_sensible)

        state.hmt.set("T_frost_surface", T_surf_avg)
        state.hmt.set("T_frost_base", T_base_avg)

        state.hmt.set("A_effective", A_effective)
        state.hmt.set("eta_surface", eta_surface)
        state.hmt.set("area_split_2phase", f_2ph)
        state.hmt.set("area_split_superheat", f_sh)


    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _calculate_area_splits(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs, eta_surface: float) -> tuple[float, float]:
        """
        Calculates the damped area fractions for the two-phase (boiling) and superheated zones.
        """
        # Calculate heat required to reach full evaporation
        Q_required_to_boil = inputs.refrigerant.m_dot * (state.refrigerant.h_sat_vap - inputs.refrigerant.h_in )
        
        # Capacity of the coil if it were 100% boiling
        capacity_result = self._process_two_phase_zone(state, inputs, eta_surface, area_fraction=1.0)
        Q_capacity_full_coil = max(1e-3, capacity_result["Q"])
        
        # Determine the "Ideal" fraction
        ratio = Q_required_to_boil / Q_capacity_full_coil
        
        if ratio >= 1.0:
            f_2ph_ideal = 1.0
        elif ratio <= 0.0:
            f_2ph_ideal = 0.0
        else:
            # Calculate the Thermodynamic Maximum (Infinite Area Limit)
            h_air_saturated_at_ref_temp = CP_HumidAir.HAPropsSI(
                'H', 'T', state.refrigerant.T_two_phase_in, 
                'P', state.air.p_avg, 'R', 1.0)
            Q_max_theoretical = state.air.m_dot_dry * (state.air.h_in - h_air_saturated_at_ref_temp)

            # Calculate epsilon for both the boiling and full coil scenarios
            epsilon_boiling   = Q_required_to_boil / Q_max_theoretical
            epsilon_full_coil = Q_capacity_full_coil / Q_max_theoretical
            
            # Calculate NTU using phase-change evaporator -> NTU = -ln(1 - epsilon)
            ntu_boiling   = -math.log(max(1e-5, 1.0 - epsilon_boiling))
            ntu_full_coil = -math.log(max(1e-5, 1.0 - epsilon_full_coil))
            
            # Since Area is directly proportional to NTU, the Area Fraction is:
            f_2ph_ideal = ntu_boiling / ntu_full_coil

        # APPLY DAMPING
        f_2ph_prev = state.hmt.area_split_2phase
        ALPHA_GEO = 0.15 
        f_2ph = (ALPHA_GEO * f_2ph_ideal) + ((1.0 - ALPHA_GEO) * f_2ph_prev)
        
        # Hard Clamp to avoid vanishing zones entirely during iterations
        f_2ph = max(0.001, min(0.999, f_2ph))
        f_sh = 1.0 - f_2ph

        return f_2ph, f_sh

    def _process_two_phase_zone(self, state, inputs, eta_surface, area_fraction) -> Dict:
        """
        Solves physics for the boiling section of the tube.
        Epsilon-NTU not necessary, because T_refrigerant constant in 2ph zone.
        """
        T_ref = state.refrigerant.T_two_phase_in
        h_conv_ref = state.refrigerant.h_conv_two_phase
        
        # Resistances
        A_eff_actual = state.frost.A_frost_surface * eta_surface
        R_frost = self._calculate_resistance_frost(state.frost.thickness, state.frost.k_frost, A_eff_actual)
        R_ref   = self._calculate_resistance_refrigerant(h_conv_ref)
        
        # Zone specific Resistance
        R_tot_zone = (R_frost + self.R_tube + R_ref) / area_fraction
        R_frost_zone = R_frost / area_fraction

        # Define Residual Function
        def evaluate_energy_balance(T_surf_guess: float) -> Tuple[float, float, float, float, float]:
            """Returns (Residual, Q_source, T_base, w_sat, m_dot_air)"""
            # Air Side Heat Transfer
            Q_source, m_dot_air, w_sat = self._calculate_heat_transfer_air_to_frost(
                state, inputs, T_surf_guess, eta_surface, area_fraction
            )

            # Refrigerant Side Heat Transfer
            Q_sink = (T_surf_guess - T_ref) / R_tot_zone
            
            # Calculate Base Temperature
            T_base = T_surf_guess - (Q_sink * R_frost_zone)
            
            return (Q_source - Q_sink), Q_source, T_base, w_sat, m_dot_air

        # Solve for Equilibrium
        Q_zone, T_s, T_b, w_sat, m_dot_zone = self._find_equilibrium_temperature(
            state, inputs, evaluate_energy_balance, T_ref_lower=T_ref
        )

        return {"Q": Q_zone, "m_dot": m_dot_zone, "T_surf": T_s, "T_base": T_b}

    def _process_superheated_zone(self, state, inputs, eta_surface, area_fraction) -> Dict:
        """
        Solves physics for the superheated section.
        Uses an epsilon-NTU approach.
        """
        T_ref_in = state.refrigerant.T_two_phase_in
        h_conv_ref = state.refrigerant.h_conv_superheated
        m_dot_ref = inputs.refrigerant.m_dot
        cp_ref = state.refrigerant.heat_capacity_avg_superheated 
        C_ref = m_dot_ref * cp_ref

        # Resistances
        A_eff_actual = state.frost.A_frost_surface * eta_surface
        R_frost = self._calculate_resistance_frost(state.frost.thickness, state.frost.k_frost, A_eff_actual)
        R_ref_conv = self._calculate_resistance_refrigerant(h_conv_ref)
        
        # Zone specific Resistance and UA
        R_tot_zone = (R_frost + self.R_tube + R_ref_conv) / area_fraction
        R_frost_zone = R_frost / area_fraction
        
        # Avoid division by zero for UA
        UA_zone = 1.0 / R_tot_zone if R_tot_zone > 1e-9 else 0.0

        # Pre-calculate efficiency if C_ref is valid (optimization)
        eff_ref = 0.0
        if C_ref > 1e-9 and UA_zone > 0:
            NTU_ref = UA_zone / C_ref
            eff_ref = 1.0 - np.exp(-NTU_ref)

        # Define Residual Function
        def evaluate_energy_balance(T_surf_guess: float) -> Tuple[float, float, float, float, float]:
            """Returns (Residual, Q_source, T_base, w_sat, m_dot_air)"""
            # Air Side Heat Transfer
            Q_source, m_dot_air, w_sat = self._calculate_heat_transfer_air_to_frost(
                state, inputs, T_surf_guess, eta_surface, area_fraction
            )

            # Refrigerant Side Heat Transfer - epsilon-NTU
            if C_ref <= 1e-9:
                Q_sink = 0.0
            else:
                Q_sink = eff_ref * C_ref * (T_surf_guess - T_ref_in)
            
            # Calculate Base Temperature
            T_base = T_surf_guess - (Q_sink * R_frost_zone)

            return (Q_source - Q_sink), Q_source, T_base, w_sat, m_dot_air

        # Solve for Equilibrium
        Q_zone, T_s, T_b, w_sat, m_dot_zone = self._find_equilibrium_temperature(
            state, inputs, evaluate_energy_balance, T_ref_lower=T_ref_in
        )

        return {"Q": Q_zone, "m_dot": m_dot_zone, "T_surf": T_s, "T_base": T_b}

    def _find_equilibrium_temperature(self, state, inputs, evaluate_energy_balance: Callable, T_ref_lower: float) -> Tuple[float, float, float, float, float]:
        """
        Secant Method solver with Step Damping and Hard Bounds.
        Finds T_surface where Q_source == Q_sink.
        """
        # Solver Constants
        MAX_STEP_SIZE = 2.0  
        MAX_ITER = 25
        TOLERANCE = 0.05
        SMALL_DENOMINATOR = 1e-9

        # If Refrigerant is hotter than Air (impossible for evaporator), set T_surf = T_air
        if T_ref_lower >= inputs.air.T_in:
             R, Q, T_base, w, m = evaluate_energy_balance(inputs.air.T_in)
             return Q, inputs.air.T_in, T_base, w, m

        # CoolProp Humid Air is invalid < 140K (approx) and cannot handle saturation > 373K (Boiling) at 1 atm.
        COOLPROP_MIN_T = 145.0
        COOLPROP_MAX_T = 370.0
        BOUND_BUFFER = 0.01     # [K]
        safe_ref_temp = max(T_ref_lower, COOLPROP_MIN_T)
        safe_air_temp = min(inputs.air.T_in, COOLPROP_MAX_T)

        # Set the absolute boundaries for the Secant solver search space
        T_min = safe_ref_temp + BOUND_BUFFER
        T_max = safe_air_temp - BOUND_BUFFER

        # Initialize Current Guess (previous solution)
        T_curr = state.hmt.T_frost_surface

        # Check for validity of Current Guess, otherwise set Guess at the midpoint
        if not (T_min <= T_curr <= T_max):
            T_curr = (T_min + T_max) / 2.0

        R_curr, Q_curr, T_base, w_sat, m_dot_val = evaluate_energy_balance(T_curr)

        # Create a "Previous" point to kickstart Secant
        T_prev = T_curr + 0.5
        T_prev = max(T_min, min(T_max, T_prev))
        R_prev, _, _, _, _ = evaluate_energy_balance(T_prev)

        # Iteration Loop
        for _ in range(MAX_ITER):
            
            # Secant Method: x_new = x_curr - f(x_curr) * (x_curr - x_prev) / (f(x_curr) - f(x_prev))
            denominator = R_curr - R_prev
            
            # Prevent division by zero - revert to a small nudge if the solver stalls
            if abs(denominator) < SMALL_DENOMINATOR:
                delta_T = MAX_STEP_SIZE * 0.1 * (-1 if R_curr > 0 else 1)
            else:
                delta_T = -R_curr * (T_curr - T_prev) / denominator

            # Apply Step Damping
            delta_T = max(-MAX_STEP_SIZE, min(MAX_STEP_SIZE, delta_T))
            T_new = T_curr + delta_T

            # Clamp to physical limits
            T_new = max(T_min, min(T_max, T_new))

            # Convergence Check: Value stability
            if abs(T_new - T_curr) < 1e-5:
                # Re-eval at T_new for final correct return values
                R_curr, Q_curr, T_base, w_sat, m_dot_val = evaluate_energy_balance(T_new)
                return Q_curr, T_new, T_base, w_sat, m_dot_val

            # Update State for next iteration
            T_prev = T_curr
            R_prev = R_curr
            T_curr = T_new
            
            # Calculate new residual
            R_curr, Q_curr, T_base, w_sat, m_dot_val = evaluate_energy_balance(T_curr)

            # Convergence Check: Residual threshold
            if abs(R_curr) < TOLERANCE:
                return Q_curr, T_curr, T_base, w_sat, m_dot_val

        # Fallback if max iterations reached
        return Q_curr, T_curr, T_base, w_sat, m_dot_val

    def _calculate_heat_transfer_air_to_frost(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs, T_surface_guess: float, eta_surface: float, area_fraction: float) -> tuple[float, float, float]:
        """
        Calculates the total heat transfer rate (Source) from the air to the frost surface.

        Args:
            state: Current state object.
            inputs: Input parameters.
            T_surface_guess: The estimated surface temperature [K].
            eta_surface: The surface efficiency (fin efficiency included).
            area_fraction: The fraction of the total area for this zone.
            
        Returns:
            - Q_air_total: Total Heat Transfer [W]
            - m_dot_frost: Total Mass Transfer [kg/s]
            - w_sat_surf: Surface saturation humidity [kg/kg]
        """
        p_avg = state.air.p_avg

        # Scaling mass flow and area by the zone fraction (area_fraction)
        m_dot_dry_zone = state.air.m_dot_dry * area_fraction
        A_frost_surface_fraction = state.frost.A_frost_surface * area_fraction
        
        w_sat_surf = CP_HumidAir.HAPropsSI('W', 'T', T_surface_guess, 'P', p_avg, 'R', 1.0)

        # Sensible Heat (Temperature Driven) - epsilon-NTU
        C_air = m_dot_dry_zone * state.air.heat_capacity_avg
        
        UA_h = state.air.h_conv * A_frost_surface_fraction * eta_surface
        ntu_h = UA_h / C_air
        epsilon_h = 1.0 - np.exp(-ntu_h)
        
        Q_sensible = epsilon_h * C_air * (inputs.air.T_in - T_surface_guess)

        # Mass Transfer (Concentration Driven)
        # betta_m [kg/m²s]
        betta_m = state.air.density_avg * state.air.betta
        UA_m = betta_m * A_frost_surface_fraction * eta_surface
        
        ntu_m = UA_m / m_dot_dry_zone
        epsilon_m = 1.0 - np.exp(-ntu_m)
        
        # Only the gas phase contributes to the humidity ratio gradient
        W_in_gas = min(inputs.air.W_in, CP_HumidAir.HAPropsSI('W', 'T', inputs.air.T_in, 'P', p_avg, 'R', 1.0))

        # Calculate Mass Flow (Only gas phase diffuses!)
        m_dot_frost_final = epsilon_m * m_dot_dry_zone * (W_in_gas - w_sat_surf)
        m_dot_frost_final = max(0.0, m_dot_frost_final)
        
        # Latent Heat
        h_sub = self._calculate_enthalpy_sublimation(T_surface_guess)
        Q_latent = m_dot_frost_final * h_sub

        # Total Heat
        Q_air_final = Q_sensible + Q_latent

        return Q_air_final, m_dot_frost_final, w_sat_surf

    def _calculate_surface_efficiency_terms(self, state: FrostEvaporatorState, h_conv_air: float) -> tuple[float, float]:
        """
        Calculates the efficiency terms using the current geometry from state.frost.
        Updated to include Frost Resistance in Fin Efficiency calculation.
        
        Args:
            state: The current state object (containing updated geometry).
            h_conv_air: Convective heat transfer coefficient [W/m^2K].
            
        Returns:
            - A_total: Total effective surface area [m^2].
            - eta_surface: The overall surface efficiency [-].
        """
        # Calculate Combined HTC for Fin Efficiency
        frost_thickness = state.frost.thickness
        k_frost = state.frost.k_frost

        h_effective_fin = self._calculate_effective_heat_transfer_coefficient(h_conv_air, frost_thickness, k_frost)
        
        # Calculate Fin Efficiency (eta_fin) using h_effective
        eta_fin = self._calculate_fin_efficiency(h_effective_fin)

        # Calculate Component Areas
        tube_diameter_w_frost = state.frost.tube_diameter_w_frost
        space_between_frost = state.frost.space_between_frost
        
        # Area of the tube part (between fins with frost)
        A_one_tube_segment = np.pi * tube_diameter_w_frost * space_between_frost
        
        # Area of the fin part (2 faces minus the tube hole)
        A_one_fin_segment = 2 * ((self.params.fin_segment_height * self.params.fin_segment_length) - (0.25 * np.pi * tube_diameter_w_frost**2))

        # Calculate Weighted Areas
        A_effective_segment = A_one_tube_segment + (eta_fin * A_one_fin_segment)
        A_effective = self.params.fin_segment_amount * A_effective_segment

        # Physical Frost Surface Area
        A_total = state.frost.A_frost_surface

        # Calculate Surface Efficiency
        eta_surface = A_effective / A_total

        return A_effective, eta_surface

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
        """
        Calculates the efficiency of a rectangular fin.

        The calculation procedure is based on the VDI Wärmeatlas,
        "M1 Wärmeübergang an berippten Rohren".

        Args:
            h_effective: The combined heat transfer coefficient of air and frost [W/m²K].

        Returns:
            Fin efficiency (eta_fin), dimensionless.
        """
            
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
    
    def _calculate_resistance_tube(self) -> float:
        """
        Calculates the conductive thermal resistance through the tube wall.
        
        Returns:
            The tube thermal resistance [K/W].
        Raises:
            ValueError: If tube dimensions or conductivity are invalid.
        """
        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.fvm_total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        if np.isclose(self.params.tube_thermal_conductivity, 0):
            raise ValueError("Tube conductivity cannot be zero.")
        
        return ( np.log(self.params.tube_outer_diameter / self.params.tube_inner_diameter) / 
                 (2 * np.pi * self.params.fvm_total_tube_length * self.params.tube_thermal_conductivity) )

    def _calculate_resistance_refrigerant(self, h_conv_refrigerant: float) -> float:
        """
        Calculates the convective thermal resistance on the refrigerant side.
        
        Args:
            h_conv_refrigerant: Refrigerant heat transfer coefficient [W/m^2K].
        Returns:
            Refrigerant side resistance [K/W].
        """
        if np.isclose(h_conv_refrigerant, 0):
            raise ValueError("h_conv_refrigerant cannot be zero.")
        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.fvm_total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        
        return 1 / (h_conv_refrigerant * np.pi * self.params.tube_inner_diameter * self.params.fvm_total_tube_length)

    def _get_rho_w_sat(self, T: float, p: float) -> float:
        """
        Calculates the water vapor density at saturation for a specific temperature.
        Used to determine the concentration gradient across the frost layer.
        """
        # Get saturated humidity ratio
        W_sat = CP_HumidAir.HAPropsSI('W', 'T', T, 'P', p, 'R', 1.0)
        
        # Get specific volume of dry air
        Vda = CP_HumidAir.HAPropsSI('Vda', 'T', T, 'P', p, 'W', W_sat)
        
        # rho_vapor = W / Vda
        return W_sat / Vda

    def _calculate_enthalpy_sublimation(self, T_frost_surface: float) -> float:
        """
        Calculates the latent heat of phase change.
        Sublimation (Ice -> Vapor) if below freezing.
        Vaporization (Liquid -> Vapor) if above freezing.
        
        Args:
            T_frost_surface: Surface temperature in Kelvin.
            
        Returns:
            Enthalpy of sublimation [J/kg]
        """
        T_celsius = T_frost_surface - 273.15
        
        if T_celsius <= 0.0:
            # Sublimation: approx 2834 kJ/kg at 0°C
            return (2834.3 - 0.29 * T_celsius) * 1000.0
        else:
            # Vaporization: approx 2501 kJ/kg at 0°C
            return (2501.0 - 2.36 * T_celsius) * 1000.0

    def _calculate_mass_flow_split_Fick(self, frost_thickness: float, frost_density: float, m_dot_total: float, 
                                        rho_w_surf: float, rho_w_base: float, A_frost_surface: float, T_frost_surface: float) -> tuple[float, float, float, float]:
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
            T_frost_surface: Frost Surface Temperature [K].

        Returns:
            - m_dot_densification: Mass flow rate for densification [kg/s].
            - m_dot_thickening: Mass flow rate for thickening [kg/s].
            - m_dot_thickening_flux: Mass flux for thickening [kg/(m^2s)].
        """        
        # Check for zero total mass flow
        if m_dot_total > 0.0:

            # Check for condensation
            if T_frost_surface < self.params.water_freezing_point:
                if frost_thickness <= 1e-6:
                    # Layer too thin for internal diffusion
                    return 0.0, m_dot_total, (m_dot_total / A_frost_surface), 0.0

                # Porosity and Diffusivity
                porosity = 1.0 - (frost_density / self.params.ice_density)
                porosity = max(0.001, min(0.999, porosity))

                # Standart Diffusivity for Water Vapor in Air
                D_AB = self.params.diffusivity_w_vapor_in_air
                
                # Apply porosity and correction factor
                D_eff = D_AB * porosity * self.params.correction_factor_frost_diffusion

                # Densification Flux (Fick's Law)
                gradient_rho = (rho_w_surf - rho_w_base) / frost_thickness
                m_dot_densification_flux = D_eff * gradient_rho
                m_dot_densification_calc = m_dot_densification_flux * A_frost_surface

                # Apply Clamps
                m_dot_densification = max(0.0, min(m_dot_densification_calc, m_dot_total))

                # Thickening
                m_dot_thickening = m_dot_total - m_dot_densification
                
                # Fluxes
                m_dot_thickening_flux = m_dot_thickening / A_frost_surface
                m_dot_densification_flux = m_dot_densification / A_frost_surface

                return m_dot_densification, m_dot_thickening, m_dot_thickening_flux, m_dot_densification_flux
            
        return 0.0, 0.0, 0.0, 0.0