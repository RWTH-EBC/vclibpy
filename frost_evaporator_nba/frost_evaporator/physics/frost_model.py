from ..datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np

class FrostModel:
    
    def __init__(self, 
                 parameters: FrostEvaporatorParameters, 
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
        """
        self.params = parameters


    def step_forward(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Performs the one-time integration step for frost thickness and updates 
        the average density based on mass/volume conservation.
        """

        # ================= Get Previous Values =================
        prev_thickness = state.frost.thickness
        prev_frost_mass = state.frost.mass
        prev_avg_density = state.frost.density

        # ================= Calculate new Values =================

        if state.hmt.T_frost_surface < self.params.water_freezing_point:
        
            # --- Effective Flux Override for Early Stage ---
            MIN_THICKNESS_FOR_DENSIFICATION = 1e-5 
            if prev_thickness < MIN_THICKNESS_FOR_DENSIFICATION:
                m_dot_thick_eff = state.hmt.m_dot_thickening + state.hmt.m_dot_densification
                m_dot_dens_eff = 0.0
                m_dot_thick_flux_eff = state.hmt.m_dot_thickening_flux + state.hmt.m_dot_densification_flux
            else:
                m_dot_thick_eff = state.hmt.m_dot_thickening
                m_dot_dens_eff = state.hmt.m_dot_densification
                m_dot_thick_flux_eff = state.hmt.m_dot_thickening_flux

            # --- New Surface- and Average-Density ---
            new_density_surface_raw = self._calculate_surface_density(
                T_frost_surface    = state.hmt.T_frost_surface, 
                T_frost_base       = state.hmt.T_frost_base,
                T_dew_point        = state.air.T_dew_point,
                correlation_choice = self.params.frost_density_correlation_choice
            )

            new_density_surface = new_density_surface_raw * self.params.correction_factor_surface_density

            new_avg_density, new_frost_mass = self._calculate_average_density_and_frost_mass(
                m_dot_thickening    = m_dot_thick_eff,
                m_dot_densification = m_dot_dens_eff,
                prev_frost_mass     = prev_frost_mass,
                prev_avg_density    = prev_avg_density,
                new_density_surface = new_density_surface,
            )

            # --- New Thermal Conductivity ---
            new_k_frost_raw = self._calculate_k_frost(
                average_density    = new_avg_density,
                correlation_choice = self.params.frost_conductivity_correlation_choice
            )
            new_k_frost = new_k_frost_raw * self.params.correction_factor_k_frost

            # --- New Thickness ---
            new_thickness = self._calculate_thickness(
                prev_thickness                     = prev_thickness,
                m_dot_thickening_flux              = m_dot_thick_flux_eff,
                new_density_surface                = new_density_surface,
            )

            # --- New Geometric Properties ---
            new_tube_diameter_w_frost = self._calculate_tube_diameter_w_frost(
                frost_thickness     = new_thickness,
            )
            new_space_between_frost = self._calculate_space_between_frost(
                frost_thickness = new_thickness, 
            )
            new_collar_diameter_w_frost = self._calculate_collar_diameter_w_frost(
                frost_thickness = new_thickness
            )


            new_flow_area_air = self._calculate_flow_area_air(
                space_between_frost   = new_space_between_frost, 
                tube_diameter_w_frost = new_tube_diameter_w_frost, 
            )
            new_A_frost_surface = self._calculate_frost_surface_area(
                tube_diameter_w_frost = new_tube_diameter_w_frost,
                space_between_frost = new_space_between_frost,
            )



            # ================= Write new values to state =================
            state.frost.set("density", new_avg_density)
            state.frost.set("thickness", new_thickness)

            state.frost.set("mass", new_frost_mass)
            state.frost.set("k_frost", new_k_frost)

            state.frost.set("tube_diameter_w_frost", new_tube_diameter_w_frost)
            state.frost.set("space_between_frost", new_space_between_frost)
            state.frost.set("collar_diameter_w_frost", new_collar_diameter_w_frost)
            state.frost.set("flow_area_air", new_flow_area_air)
            state.frost.set("A_frost_surface", new_A_frost_surface)
        
        else:
            # T_surface >= freezing point (Condensation phase)
            # Assumption: Condensate immediately drops off. 
            # Existing frost state and geometry remain unchanged.
            pass

            


    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _calculate_surface_density(self, T_frost_surface: float, T_frost_base: float, T_dew_point: float, correlation_choice: str) -> float:
        """
        Calculates the new frost density at the surface.

        Args:
            T_frost_surface: The frost surface temperature  [K].
            T_frost_base: The frost base temperature at the base (wall temperature) [K].
            T_dew_point: The dew point temperature of the air [K].
            correlation_choice: The choice of correlation to use for frost density [-].
        Returns:
            The calculated new frost density [kg/m^3].
        """
        
        # --- Convert to Celsius (Common for these correlations) ---
        T_surface_C = T_frost_surface - 273.15
        T_base_C = T_frost_base - 273.15  # Wall/Base temperature
        T_dew_C = T_dew_point - 273.15

        rho_calc = 100.0 # Default fallback

        if correlation_choice in ["hayashi_1977"]:        
            # Hayashi et al. (1977): A classic correlation based solely on the frost surface temperature.
            rho_calc = 650.0 * np.exp(0.277 * T_surface_C)
            
        elif correlation_choice in ["nascimento_2013"]:
            # Nascimento et al. (2013) / Hermes: Empirically fitted for wall temps between -15°C and -5°C.
            rho_calc = 207.0 * np.exp(0.266 * T_surface_C - 0.0615 * T_base_C)

        elif correlation_choice in ["da_silva_2011"]:  
            # da Silva et al. (2011): Section 4 "Results", Eq. 9
            rho_calc = 494.0 * np.exp(0.11 * T_surface_C - 0.06 * T_dew_C)

        elif correlation_choice == "wang_2012":
            # Wang et al. (2012): Modifies Hayashi's correlation by adding a base temperature correction factor.
            c1 = 0.70132 - 0.11346 * T_base_C - 0.00203 * (T_base_C ** 2)
            rho_calc = c1 * 650.0 * np.exp(0.277 * T_surface_C)

        else:
            raise ValueError(f"Unknown frost density correlation: {correlation_choice}")
            
        return max(20.0, min(900.0, rho_calc))
        
    def _calculate_average_density_and_frost_mass(self, m_dot_thickening: float, m_dot_densification: float, prev_frost_mass: float, prev_avg_density: float, 
                                                  new_density_surface: float) -> tuple[float, float]:
        """
        Calculates the new average density and total frost mass.
        Integrates mass flow rates over the time step. Note that densification 
        adds mass but not volume (fills pores), while thickening adds both.

        Args:
            m_dot_thickening: Mass flow rate contributing to thickness increase [kg/s].
            m_dot_densification: Mass flow rate contributing to density increase [kg/s].
            prev_frost_mass: Total frost mass from the previous step [kg].
            prev_avg_density: Average frost density from the previous step [kg/m^3].
            new_density_surface: Density of the newly deposited surface layer [kg/m^3].
        Returns:
            - new_avg_density: The updated average frost density [kg/m^3].
            - new_frost_mass: The updated total frost mass [kg].
        """

        # Calculate total mass increase
        mass_added_total = (m_dot_thickening + m_dot_densification) * self.params.time_step
        new_frost_mass = prev_frost_mass + mass_added_total

        # Determine Previous Frost Volume
        if prev_frost_mass > 0:
            prev_vol = prev_frost_mass / prev_avg_density
        else:
            prev_vol = 0.0

        # Calculate new total volume
        if m_dot_thickening > 0:
            vol_added = (m_dot_thickening * self.params.time_step) / new_density_surface 
            new_total_vol = prev_vol + vol_added
        else:
            new_total_vol = prev_vol
        
        # Calculate new Average Density
        if new_total_vol > 0:
            new_avg_density = new_frost_mass / new_total_vol  
        else: 
            new_avg_density = new_density_surface
            

        return new_avg_density, new_frost_mass

    def _calculate_k_frost(self, average_density: float, correlation_choice: str) -> float:
        """
        Calculates the new thermal conductivity of the frost.

        Args:
            average_density: The average frost density [kg/m^3].
            correlation_choice: The choice of correlation to use for frost conductivity [-].
        Returns:
            The calculated frost thermal conductivity [W/(m*K)].
        Raises:
            ValueError: If the frost conductivity correlation choice is unknown.
        """
        # Safety clamp to avoid math domain errors or non-physical densities 
        rho = max(20.0, min(900.0, average_density))

        if correlation_choice in ["oneal_tree_1984"]:
            # O'Neal & Tree (1984) / Sanders (1974)
            return 1.202e-3 * rho ** 0.963
            
        elif correlation_choice in ["lee_1997"]:  
            # Lee et al. (1997)
            return 0.132 + (3.13e-4 * rho) + (1.6e-7 * rho**2)

        elif correlation_choice == "yonko_sepsy_1967":
            # Yonko and Sepsy (1967) - Pioneering quadratic fit
            return 0.02422 + (7.214e-4 * rho) + (1.1797e-6 * rho**2)
        else:
            raise ValueError(f"Unknown frost conductivity correlation: {correlation_choice}")

    def _calculate_thickness(self, prev_thickness: float, m_dot_thickening_flux: float, new_density_surface: float,) -> float:
        """
        Calculates the new frost thickness.

        Args:
            prev_thickness: Old thickness [m].
            m_dot_thickening_flux: Mass flux contributing specifically to thickness [kg/(m^2*s)].
            new_density_surface: Density of the new layer [kg/m^3].
        Returns:
            The new total frost thickness [m].
        """
        # Euler forward step
        delta_thickness = (m_dot_thickening_flux * self.params.time_step) / new_density_surface
        return prev_thickness + delta_thickness

    def _calculate_tube_diameter_w_frost(self, frost_thickness: float) -> float:
        """
        Calculates the effective tube outer diameter including frost.
        
        Args:
            frost_thickness: The current frost thickness [m].
        Returns:
            The tube outer diameter plus twice the frost thickness [m].
        """
        return self.params.tube_outer_diameter + 2 * frost_thickness

    def _calculate_space_between_frost(self, frost_thickness: float) -> float:
        """
        Calculates the air flow channel width between fins, considering frost on both sides.

        Args:
            frost_thickness: The current frost thickness [m].
        Returns:
            The reduced space between the frosted fins [m].
        """
        return self.params.fin_pitch - self.params.fin_thickness - 2 * frost_thickness
    
    def _calculate_collar_diameter_w_frost(self, frost_thickness: float) -> float:
        """
        Calculates the diameter of the tube collar including fin thickness and frost.

        Args:
            frost_thickness: The current frost thickness [m].
        Returns:
            The outer diameter of the tube plus twice the fin thickness and 
            twice the frost thickness [m].
        """
        return self.params.tube_outer_diameter + 2 * self.params.fin_thickness + 2 * frost_thickness
        

    def _calculate_flow_area_air(self, space_between_frost: float, tube_diameter_w_frost: float) -> float:
        """
        Calculates the total cross-sectional area for air flow.

        Args:
            space_between_frost: The air flow channel width between frosted fins [m].
            tube_diameter_w_frost: The effective tube outer diameter including frost [m].
        Returns:
            The total air flow area [m^2].
        """
        return (self.params.fin_amount - 1) * (self.params.fvm_fin_height - tube_diameter_w_frost * self.params.fvm_tubes_per_layer) * space_between_frost
    
    def _calculate_frost_surface_area(self, tube_diameter_w_frost:float, space_between_frost:float)-> float:
        """
        Calculates the total frost surface area on the finned tube evaporator (area used for heat transfer from air to frost surface).

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