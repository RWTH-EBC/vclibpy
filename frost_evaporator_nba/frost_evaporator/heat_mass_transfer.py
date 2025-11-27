from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import math


class HeatMassTransferModel:
    """
    # TODO Docstring
    """
    
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
        Calculates and updates ...
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # Calculate effective area (with fin efficiency) for total thermal resistance
        A_effective, eta_fin = self._calculate_effective_area(
            h_conv_air=state.air.h_conv,
            frost_thickness = state.frost.thickness,
            k_frost=state.frost.k_frost
        )

        R_air = self._calculate_resistance_air(
            h_conv_air=state.air.h_conv,
            A_effective=A_effective
        )

        R_frost = self._calculate_resistance_frost(
            frost_thickness=state.frost.thickness,
            k_frost=state.frost.k_frost,
            A_effective=A_effective
        )

        # R_refrigerant = self._calculate_thermal_resistance_refrigerant(
        #     h_conv_refrigerant = state.refrigerant.h_conv
        # )

        # Combine the "downstream" resistances (everything AFTER the frost surface)
        # TODO change back to original form when refrigerant is reactivated!!!
        R_downstream = R_frost # + self.R_tube + R_refrigerant


        T_refrigerant_avg = 0.5 * (state.refrigerant.T_in + state.refrigerant.T_out)

        T_frost_surface = state.hmt.T_frost_surface



        # --- 1. HEAT TRANSFER (Standard epsilon-NTU) ---
        # Get Air Stream Capacity Rate (C_air)
        m_dot_air = state.air.m_dot_humid 
        cp_air = state.air.heat_capacity_avg       
        C_air = m_dot_air * cp_air

        # Calculate NTU (Number of Transfer Units) on the air side
        UA_air = 1.0 / R_air
        NTU = UA_air / C_air

        # Calculate Effectiveness (epsilon) - surface at constant temperature
        epsilon = 1.0 - math.exp(-NTU)

        # Calculate Maximum Possible Sensible Heat
        Q_max_sens = C_air * (inputs.air.T_in - T_frost_surface)

        # Calculate Actual Sensible Heat
        Q_dot_sens = epsilon * Q_max_sens



        # --- 2. MASS TRANSFER (Modified epsilon-NTU) ---
        
        # Calculate Volume Flow Rate [m^3/s]
        # We need this because betta is usually in [m/s] and applies to volume concentration
        # m_dot [kg/s] / rho [kg/m^3] = V_dot [m^3/s]
        rho_air_avg = state.air.density_avg
        V_dot_air = m_dot_air / rho_air_avg

        # Calculate NTU_mass
        # Analogous to UA / C_min. 
        # Here: (Mass Transfer Coeff * Area) / Volume Flow
        betta = state.air.betta
        NTU_mass = (betta * A_effective) / V_dot_air

        # Calculate epsilon_mass
        epsilon_mass = 1.0 - math.exp(-NTU_mass)

        # D. Calculate Mass Transfer
        # Driving force: Density difference (Inlet Air vs Surface Saturation)
        rho_w_in = state.air.rho_w_in       # Vapor density at INLET
        rho_w_surf = state.air.rho_w_frost_sat # Vapor density at SURFACE (saturation)
        
        # Max possible mass transfer (if air reached surface saturation perfectly)
        # kg/s = V_dot [m3/s] * delta_rho [kg/m3]
        m_dot_frost_max = V_dot_air * (rho_w_in - rho_w_surf)
        
        # Actual mass transfer
        m_dot_frost_total = epsilon_mass * m_dot_frost_max

        # Back-calculate flux for your specific variable tracking
        m_dot_frost_flux = m_dot_frost_total / A_effective


        # --- 3. TOTAL ENERGY ---
        h_sublimation = self._calculate_enthalpy_sublimation(T_frost_surface=T_frost_surface)
        Q_dot_lat = m_dot_frost_total * h_sublimation
        Q_dot_total = Q_dot_sens + Q_dot_lat


        # Calculate total geometric frost surface area (no fin efficiency) for T_frost_surface
        A_frost_surface = self._calculate_frost_surface_area(
            tube_diameter_w_frost=state.frost.tube_diameter_w_frost,
            space_between_frost=state.frost.space_between_frost
        )
        
        # Calculate required T_frost_surface to push Q_total through downstream resistance
        T_frost_surface_new = T_refrigerant_avg + (Q_dot_total * R_downstream)

        # alpha is your relaxation factor (e.g., 0.2 to 0.5)
        alpha = 0.2
        T_frost_surface_update = (alpha * T_frost_surface_new) + ((1 - alpha) * T_frost_surface)


        state.hmt.set("A_effective", A_effective)
        state.hmt.set("A_frost_surface", A_frost_surface)
        state.hmt.set("R_downstream", R_downstream)
        state.hmt.set("T_frost_surface", T_frost_surface_update)
        state.hmt.set("Q_dot_total", Q_dot_total)
        state.hmt.set("Q_dot_sens", Q_dot_sens)
        state.hmt.set("m_dot_frost_flux", m_dot_frost_flux)
        state.hmt.set("m_dot_frost_total", m_dot_frost_total)
        state.hmt.set("eta_fin", eta_fin)
        




    def _calculate_heat_transfer_rate(self, delta_T_log: float, R_total: float) -> float:
        """
        Calculates the heat transfer rate between air and refrigerant.
        """
        return delta_T_log / R_total
    
    def _calculate_m_dot_frost_flux(self, betta: float, rho_w_avg: float, rho_w_frost_sat: float) -> float:
        """
        Calculates the mass flux of frost formation on the evaporator surface.
        """
        if betta < 0:
            raise ValueError("betta cannot be negative.")
        if rho_w_avg < rho_w_frost_sat:
            print("rho_w_avg must be greater than or equal to rho_w_frost_sat.") 

        return betta * (rho_w_avg - rho_w_frost_sat)
    

    # def _calculate_frost_surface_temperature(self, T_air_in: float, T_air_out:float, Q_dot_sensible: float, h_conv_air: float, A_frost_surface: float) -> float:
    #     """
    #     Calculates the frost surface temperature.
        
    #     NOTE: We must use Q_sensible here (convection only), not total heat.
    #     Latent heat is released ON the surface, it does not travel THROUGH the air boundary layer 
    #     in the same way to drive the temperature difference.
    #     """

    #     if np.isclose(h_conv_air, 0):
    #         raise ValueError("h_conv_air cannot be zero.")
    #     if np.isclose(A_frost_surface, 0):
    #         raise ValueError("A_frost_surface cannot be zero.")

    #     T_air_avg = 0.5 * (T_air_in + T_air_out)

    #     # T_surf = T_air - (SensibleHeat / (h * A))
    #     return T_air_avg - Q_dot_sensible / (h_conv_air * A_frost_surface)

    
    def _calculate_resistance_air(self, h_conv_air, A_effective):
        if np.isclose(h_conv_air, 0) or np.isclose(A_effective, 0):
             raise ValueError("Zero value in resistance calc")
        return 1 / (h_conv_air * A_effective)

    def _calculate_resistance_frost(self, frost_thickness, k_frost, A_effective):
        if np.isclose(k_frost, 0) or np.isclose(A_effective, 0):
             raise ValueError("Zero value in resistance calc")
        # R = L / (k * A)
        return frost_thickness / (k_frost * A_effective)

    
    def _calculate_thermal_resistance_tube(self) -> float:
        """
        Calculates the conductive thermal resistance through the tube wall.
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
        """

        if np.isclose(h_conv_refrigerant, 0):
            raise ValueError("h_conv_refrigerant cannot be zero.")
        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        
        return 1 / (h_conv_refrigerant * np.pi * self.params.tube_inner_diameter * self.params.total_tube_length)


    def _calculate_effective_area(self, h_conv_air: float, frost_thickness:float, k_frost:float) -> tuple[float, float]:
        """
        Calculates the effective heat transfer area of the finned tube.
        This function works for "Fluchtende Rohre" only.
        Args:
            h_conv_air (float): The convective heat transfer coefficient of the air [W/m²K].
            k_frost (float): The thermal conductivity of the frost layer [W/mK].
        Returns:
            float: The effective heat transfer area [m²].
            float: The effective heat transfer coefficient of air and frost [W/m²K].

        """

        # Calculate the heat transfer Coefficient of air and frost in series
        h_effective = self._calculate_effective_heat_transfer_coefficient(h_conv_air, frost_thickness,  k_frost)


        # Calculate fin efficiency
        eta_fin= self._calculate_fin_efficiency(h_effective)

        # Calculate area of one fin and one tube segment
        A_one_tube_segment = np.pi * self.params.tube_outer_diameter * (self.params.fin_pitch - self.params.fin_thickness)
        A_one_fin_segment  = 2 * ( (self.params.fin_segment_height * self.params.fin_segment_length) - (0.25 * np.pi * self.params.tube_outer_diameter**2) )


        A_effective = self.params.fin_segment_amount * (A_one_tube_segment + eta_fin * A_one_fin_segment)

        return A_effective, eta_fin

    def _calculate_frost_surface_area(self, tube_diameter_w_frost:float, space_between_frost:float)-> float:
        """
        Calculates the total frost surface area on the finned tube evaporator.

        Args:
            tube_diameter_w_frost (float): The effective tube outer diameter including frost [m].
            space_between_frost (float): The air flow channel width between frosted fins including frost [m].

        Returns:
            float: The total frost surface area [m²].
        """

        # Calculate area of one fin and one tube segment
        A_one_tube_segment_frost = np.pi * tube_diameter_w_frost * space_between_frost
        A_one_fin_segment_frost  = 2 * ( (self.params.fin_segment_height * self.params.fin_segment_length) - (0.25 * np.pi * tube_diameter_w_frost**2) )

        return self.params.fin_segment_amount * (A_one_tube_segment_frost + A_one_fin_segment_frost)

    def _calculate_fin_efficiency(self, h_effective: float) ->float:
        # sourcery skip: assign-if-exp, reintroduce-else
        """
        Calculates the efficiency of a rectangular fin.

        The calculation procedure is based on the VDI Wärmeatlas,
        "M1 Wärmeübergang an berippten Rohren".

        Args:
            h_effective (float): The combined heat transfer coefficient of air and frost [W/m²K].

        Returns:
            float: Fin efficiency (eta_fin), dimensionless.
        """
        
        if np.isclose(self.params.tube_outer_diameter, 0):
            raise ValueError("tube_outer_diameter cannot be zero.")

            
        if np.isclose(self.params.fin_thickness, 0):
            raise ValueError("fin_thickness cannot be zero.")
        
        if np.isclose(self.params.fin_thermal_conductivity, 0):
            raise ValueError("fin_thermal_conductivity cannot be zero.")
        
        if np.isclose(self.params.fin_segment_length, 0) or np.isclose(self.params.fin_segment_height, 0):
            raise ValueError("fin_segment_length and fin_segment_height cannot be zero.")
            

        # # Equation (13) from VDI Wärmeatlas M1
        # if self.params.fin_segment_height > self.params.fin_segment_length:
        #     raise ValueError("Fin segment height cannot be greater than fin segment length.")


        # Equation (13) from VDI Wärmeatlas M1
        phi_dash = (1.28 * self.params.fin_segment_height / self.params.tube_outer_diameter *
                    np.sqrt((self.params.fin_segment_length / self.params.fin_segment_height) - 0.2))

        # Equation (12) from VDI Wärmeatlas M1
        phi = (phi_dash - 1) * (1 + 0.35 * np.log(phi_dash))

        # Equation (8) from VDI Wärmeatlas M1
        term_in_sqrt = (2 * h_effective) / (self.params.fin_thermal_conductivity * self.params.fin_thickness)
        X = phi * (self.params.tube_outer_diameter / 2) * np.sqrt(term_in_sqrt)

        # Equation (7) and (9) from VDI Wärmeatlas M1
        # To avoid division by zero if X is very close to 0,
        # we use the limit of tanh(X)/X as X->0, which is 1.
        if X < 1e-6: return 1.0
        
        return np.tanh(X) / X


    def _calculate_effective_heat_transfer_coefficient(self, h_conv_air:float, frost_thickness:float, k_frost:float) -> float:
        """
        Calculates the combined heat transfer coefficient
        for convection from air through a layer of frost.

        This treats the convective and conductive layers as thermal
        resistances in series.

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
    

    def _calculate_enthalpy_sublimation(self, T_frost_surface: float) -> float:
        """
        Calculates the latent heat of sublimation (Ice -> Vapor) 
        as a function of surface temperature.
        
        Based on standard property data for water ice.
        h_sub is approx 2.834e6 J/kg at 0°C and rises slightly as T drops.
        
        Args:
            T_frost_surface: Surface temperature in Kelvin.
            
        Returns:
            float: Enthalpy of sublimation [J/kg]
        """
        T_celsius = T_frost_surface - 273.15
        
        # Linear approximation for h_sublimation [J/kg]
        # h_sub = 2834.3 kJ/kg - 0.29 * T_celsius (approximate slope)
        # Note: The slope is negative, meaning h_sub INCREASES as T DECREASES.
        # (Ice crystal lattice binding energy is higher at lower temps)
        
        return (2834.3 - 0.29 * T_celsius) * 1000.0