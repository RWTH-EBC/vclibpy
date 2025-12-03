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
        Calculates and updates the heat-mass-transfer model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # ================= Get State Values =================
        # Air Properties
        m_dot_air = state.air.m_dot_humid
        heat_capacity_air = state.air.heat_capacity_avg
        density_air = state.air.density_avg
        betta_air = state.air.betta
        h_conv_air = state.air.h_conv
        T_air_in = inputs.air.T_in
        
        # Vapor Properties
        rho_w_in = state.air.rho_w_in
        rho_w_surf = state.air.rho_w_frost_surface_sat
        rho_w_base = state.air.rho_w_frost_base_sat

        # Frost/Refrigerant Properties
        T_frost_surface = state.hmt.T_frost_surface
        T_refrigerant_avg = 0.5 * (state.refrigerant.T_in + state.refrigerant.T_out)
        frost_thickness = state.frost.thickness
        frost_density = state.frost.density
        k_frost = state.frost.k_frost


        # ================= Calculate New Values =================
        
        # Geometry and Resistances
        A_effective, eta_fin = self._calculate_effective_area(
            h_conv_air=h_conv_air,
            frost_thickness=frost_thickness,
            k_frost=k_frost
        )

        A_frost_surface = self._calculate_frost_surface_area(
            tube_diameter_w_frost=state.frost.tube_diameter_w_frost,
            space_between_frost=state.frost.space_between_frost
        )

        R_air = self._calculate_resistance_air(
            h_conv_air=h_conv_air, 
            A_effective=A_effective
        )

        R_frost = self._calculate_resistance_frost(
            frost_thickness=frost_thickness,
            k_frost=k_frost,
            A_effective=A_effective
        )

        # R_refrigerant = self._calculate_thermal_resistance_refrigerant(
        #     h_conv_refrigerant = state.refrigerant.h_conv
        # )
        
        R_downstream = R_frost # + self.R_tube + R_refrigerant (TODO: Add back)


        # Heat Transfer (Sensible)
        Q_dot_sens = self._calculate_heat_transfer_sensible(
            m_dot_air=m_dot_air,
            heat_capacity_air=heat_capacity_air,
            R_air=R_air,
            T_air_in=T_air_in,
            T_frost_surface=T_frost_surface
        )


        # Mass Transfer (Latent)
        m_dot_frost_total = self._calculate_mass_transfer_total(
            m_dot_air=m_dot_air,
            density_air=density_air,
            betta_air=betta_air,
            A_effective=A_effective,
            rho_w_in=rho_w_in,
            rho_w_surf=rho_w_surf
        )


        # Mass Split (Densification vs Thickening)
        m_dot_dens, m_dot_thick, m_dot_thick_flux = self._calculate_mass_flow_split(
            frost_thickness=frost_thickness,
            frost_density=frost_density,
            m_dot_total=m_dot_frost_total,
            rho_w_surf=rho_w_surf,
            rho_w_base=rho_w_base,
            A_frost_surface=A_frost_surface
        )


        # Energy Balance & Temperatures
        Q_dot_total, T_surface_new, T_base_new = self._calculate_energy_balance_and_temps(
            m_dot_frost_total=m_dot_frost_total,
            Q_dot_sens=Q_dot_sens,
            T_frost_surface_prev=T_frost_surface,
            T_refrigerant_avg=T_refrigerant_avg,
            R_downstream=R_downstream,
            R_frost=R_frost
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
        state.hmt.set("A_frost_surface", A_frost_surface)
        state.hmt.set("eta_fin", eta_fin)
        state.hmt.set("R_downstream", R_downstream)
        # state.hmt.set("R_refrigerant", R_refrigerant)
        state.hmt.set("R_tube", self.R_tube)
        state.hmt.set("R_frost", R_frost)
        state.hmt.set("R_air", R_air)
        

    def _calculate_heat_transfer_sensible(self, m_dot_air, heat_capacity_air, R_air, T_air_in, T_frost_surface):
        """Calculates Sensible Heat Transfer using Epsilon-NTU method."""
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

    def _calculate_mass_transfer_total(self, m_dot_air, density_air, betta_air, A_effective, rho_w_in, rho_w_surf):
        """Calculates Total Mass Transfer using Modified Epsilon-NTU."""
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

    def _calculate_energy_balance_and_temps(self, m_dot_frost_total, Q_dot_sens, T_frost_surface_prev, T_refrigerant_avg, R_downstream, R_frost):
        """Calculates Latent Heat, Total Energy, and updates Surface/Base Temps."""
        
        # 1. Calculate Total Energy
        h_sublimation = self._calculate_enthalpy_sublimation(T_frost_surface=T_frost_surface_prev)
        Q_dot_lat = m_dot_frost_total * h_sublimation
        Q_dot_total = Q_dot_sens + Q_dot_lat

        # 2. Calculate Temperatures
        T_surface_new_raw = T_refrigerant_avg + (Q_dot_total * R_downstream)
        T_base_new = T_surface_new_raw - (Q_dot_total * R_frost)

        # 3. Apply Relaxation
        alpha = 0.2
        T_surface_final = (alpha * T_surface_new_raw) + ((1 - alpha) * T_frost_surface_prev)

        return Q_dot_total, T_surface_final, T_base_new

    def _calculate_mass_flow_split(self, frost_thickness, frost_density, m_dot_total, rho_w_surf, rho_w_base, A_frost_surface):
        """Determines how much mass flow contributes to densification vs thickening."""
        
        if frost_thickness <= 1e-6:
            # Layer too thin for internal diffusion
            return 0.0, m_dot_total, (m_dot_total / A_frost_surface)

        # 1. Porosity and Diffusivity
        porosity = 1.0 - (frost_density / self.params.ice_density)
        porosity = max(0.0, min(1.0, porosity))
        
        D_AB = self.params.diffussivity_w_vapor_in_air
        D_eff = D_AB * porosity

        # 2. Densification Flux (Fick's Law)
        gradient_rho = (rho_w_surf - rho_w_base) / frost_thickness
        m_dot_densification_flux = D_eff * gradient_rho
        m_dot_densification = m_dot_densification_flux * A_frost_surface

        # 3. Clamping
        m_dot_densification = max(0.0, m_dot_densification)
        if m_dot_densification > m_dot_total:
            m_dot_densification = m_dot_total

        # 4. Thickening
        m_dot_thickening = m_dot_total - m_dot_densification
        m_dot_thickening_flux = m_dot_thickening / A_frost_surface

        return m_dot_densification, m_dot_thickening, m_dot_thickening_flux


    def _calculate_heat_transfer_rate(self, delta_T_log: float, R_total: float) -> float:
        """
        Calculates the heat transfer rate between air and refrigerant.
        """
        return delta_T_log / R_total

    
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
        eta_fin_raw = self._calculate_fin_efficiency(h_effective)

        eta_fin = eta_fin_raw * self.params.correction_factor_eta_fin


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