from .datamodels_nba import (
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

    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the frost model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values (like T_frost_surface).
        """
        
        # ================= Get State Values =================

        # ================= Calculate new Values =================
        
        # ================= Write new values to state =================
        


    def step_forward(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Performs the one-time integration step for frost thickness and updates 
        the average density based on mass/volume conservation.
        """

        # ================= Get Previous Values =================
        prev_thickness = state.frost.thickness
        prev_frost_mass = state.frost.mass
        prev_avg_density = state.frost.density

        if state.hmt.T_frost_surface > self.params.water_freezing_point:
            print("Warning: Calculated frost surface temperature is above freezing point. No frost growth will be calculated.")


        else: 
        
        
            # ================= Calculate new Values =================

            # --- New Surface- and Average-Density ---
            new_density_surface_raw = self._calculate_surface_density(
                T_frost_surface                  = state.hmt.T_frost_surface, 
                T_dew_point                      = state.air.T_dew_point,
                frost_density_correlation_choice = self.params.frost_density_correlation_choice
            )

            new_density_surface = new_density_surface_raw * self.params.correction_factor_surface_density

            new_avg_density, new_frost_mass = self._calculate_average_density_and_frost_mass(
                m_dot_thickening    = state.hmt.m_dot_thickening,
                m_dot_densification = state.hmt.m_dot_densification,
                prev_frost_mass     = prev_frost_mass,
                prev_avg_density    = prev_avg_density,
                new_density_surface = new_density_surface,
                prev_thickness      = prev_thickness,
                time_step           = self.params.time_step
            )

            # --- New Thermal Conductivity and Thickness ---
            new_k_frost_raw = self._calculate_k_frost(
                average_density                       = new_avg_density,
                frost_conductivity_correlation_choice = self.params.frost_conductivity_correlation_choice
            )

            new_k_frost = new_k_frost_raw * self.params.correction_factor_k_frost

            new_thickness = self._calculate_thickness(
                prev_thickness                     = prev_thickness,
                m_dot_thickening_flux              = state.hmt.m_dot_thickening_flux,
                new_density_surface                = new_density_surface,
                frost_thickness_correlation_choice = self.params.frost_thickness_correlation_choice,
                time_step                          = self.params.time_step
            )


            # --- New Geometric Properties ---
            new_tube_diameter_w_frost = self._calculate_tube_diameter_w_frost(
                frost_thickness     = new_thickness,
                tube_outer_diameter = self.params.tube_outer_diameter
            )

            new_space_between_frost = self._calculate_space_between_frost(
                frost_thickness = new_thickness, 
                fin_pitch       = self.params.fin_pitch, 
                fin_thickness   = self.params.fin_thickness
            )

            new_flow_area_air = self._calculate_flow_area_air(
                space_between_frost   = new_space_between_frost, 
                fin_height            = self.params.fin_height, 
                fin_amount            = self.params.fin_amount, 
                tube_diameter_w_frost = new_tube_diameter_w_frost, 
                tubes_per_layer       = self.params.tubes_per_layer
            )

            new_A_frost_surface = self._calculate_frost_surface_area(
                tube_diameter_w_frost = new_tube_diameter_w_frost,
                space_between_frost = new_space_between_frost,
                fin_segment_height = self.params.fin_segment_height,
                fin_segment_length = self.params.fin_segment_length,
                fin_segment_amount = self.params.fin_segment_amount
            )




            # ================= Write new values to state =================
            state.frost.set("density", new_avg_density)
            state.frost.set("thickness", new_thickness)

            state.frost.set("mass", new_frost_mass)
            state.frost.set("k_frost", new_k_frost)

            state.frost.set("tube_diameter_w_frost", new_tube_diameter_w_frost)
            state.frost.set("space_between_frost", new_space_between_frost)
            state.frost.set("flow_area_air", new_flow_area_air)
            state.frost.set("A_frost_surface", new_A_frost_surface)



    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _calculate_surface_density(self, T_frost_surface: float, T_dew_point: float, frost_density_correlation_choice: str) -> float:
        # sourcery skip: inline-variable, switch
        """
        Calculates the new frost density at the surface.

        Args:
            T_frost_surface: The frost surface temperature at the start of the step [K].
            T_dew_point: The dew point temperature of the air [K].
            frost_density_correlation_choice: The choice of correlation to use for frost density [-].
        Returns:
            The calculated new frost density [kg/m^3].
        Raises:
            ValueError: If the frost density correlation choice is unknown.
        """
        if frost_density_correlation_choice == "jonas_diss":        
            
            # Temperatur in Celsius
            T_s_C = T_frost_surface - 273.15
            rho_calc = 650.0 * np.exp(0.277 * T_s_C)
            
            # Safety Clamping           
            return max(80.0, min(900.0, rho_calc))
        
        elif frost_density_correlation_choice == "da_silva_paper":  
            # Coefficients from Section 4 "Results" of da Silva et al. (2011)
            a = 494.0
            b = 0.11
            c = -0.06
            
            # Convert K to Celsius
            T_s_C = T_frost_surface - 273.15
            T_dew_C = T_dew_point - 273.15

            # Eq. 9
            return a * np.exp(b * T_s_C + c * T_dew_C)
        
        else:
            raise ValueError(f"Unknown frost density correlation: {frost_density_correlation_choice}")
        
    def _calculate_average_density_and_frost_mass(self, m_dot_thickening: float, m_dot_densification: float, prev_frost_mass: float, prev_avg_density: float, 
                                                  new_density_surface: float, prev_thickness: float, time_step: float) -> tuple[float, float]:
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
            time_step: The simulation time step [s].
        Returns:
            - new_avg_density: The updated average frost density [kg/m^3].
            - new_frost_mass: The updated total frost mass [kg].
        """

        # Check for minimal frost Thickness for densitifaction. Otherwise there are numerical issues.
        MIN_THICKNESS_FOR_DENSIFICATION = 1e-4 
        if prev_thickness < MIN_THICKNESS_FOR_DENSIFICATION:
            # Early Stage: The layer is too thin for internal densification.
            # Treat ALL mass flow as thickening (crystal growth).
            m_dot_thickening_effective = m_dot_thickening + m_dot_densification
            m_dot_densification_effective = 0.0
        else:
            # Mature Stage: Allow separation of fluxes.
            m_dot_thickening_effective = m_dot_thickening
            m_dot_densification_effective = m_dot_densification

        if m_dot_thickening_effective > 0:
            # Integrate MASS (Total Mass increases by BOTH flows)
            mass_added_total = (m_dot_thickening_effective + m_dot_densification_effective) * time_step
            new_frost_mass = prev_frost_mass + mass_added_total
            
            # Integrate VOLUME (Volume increases ONLY by thickening flow)
            # Densification mass enters existing pores, so it adds 0 volume.
            prev_vol = prev_frost_mass / prev_avg_density
            vol_added = (m_dot_thickening_effective * time_step) / new_density_surface 
            new_total_vol = prev_vol + vol_added
            
            # Calculate New Average Density
            new_avg_density = new_frost_mass / new_total_vol

        else:
            new_avg_density = new_density_surface
            new_frost_mass = prev_frost_mass
            

        return new_avg_density, new_frost_mass

    def _calculate_k_frost(self, average_density: float, frost_conductivity_correlation_choice: str) -> float:
        """
        Calculates the new thermal conductivity of the frost.

        Args:
            average_density: The average frost density [kg/m^3].
            frost_conductivity_correlation_choice: The choice of correlation to use for frost conductivity [-].
        Returns:
            The calculated frost thermal conductivity [W/(m*K)].
        Raises:
            ValueError: If the frost conductivity correlation choice is unknown.
        """
        if frost_conductivity_correlation_choice == "A":
            return 1.202e-3 * average_density ** 0.963
        elif frost_conductivity_correlation_choice == "da_silva_paper":  
            return 0.132 + (3.13e-4 * average_density) + (1.6e-7 * average_density**2)

        else:
            raise ValueError(f"Unknown frost conductivity correlation: {frost_conductivity_correlation_choice}")

    def _calculate_thickness(self, prev_thickness: float, m_dot_thickening_flux: float, new_density_surface: float, 
                            frost_thickness_correlation_choice: str, time_step: float) -> float:
        """
        Calculates the new frost thickness.

        Args:
            prev_thickness: Old thickness [m].
            m_dot_thickening_flux: Mass flux contributing specifically to thickness [kg/(m^2*s)].
            new_density_surface: Density of the new layer [kg/m^3].
            frost_thickness_correlation_choice: The choice of correlation to use for frost thickness [-].
            time_step: The simulation time step [s].
        Returns:
            The new total frost thickness [m].
        Raises:
            ValueError: If the frost thickness correlation choice is unknown.
        """
        if frost_thickness_correlation_choice == "jonas_diss":
            # Euler forward step
            delta_thickness = (m_dot_thickening_flux * time_step) / new_density_surface
            return prev_thickness + delta_thickness
        else:
            raise ValueError(f"Unknown frost thickness correlation: {frost_thickness_correlation_choice}")

    def _calculate_tube_diameter_w_frost(self, frost_thickness: float, tube_outer_diameter: float) -> float:
        """
        Calculates the effective tube outer diameter including frost.
        
        Args:
            frost_thickness: The current frost thickness [m].
            tube_outer_diameter: The outer diameter of the clean tube [m].
        Returns:
            The tube outer diameter plus twice the frost thickness [m].
        """
        return tube_outer_diameter + 2 * frost_thickness

    def _calculate_space_between_frost(self, frost_thickness: float, fin_pitch: float, fin_thickness: float) -> float:
        """
        Calculates the air flow channel width between fins, considering frost on both sides.

        Args:
            frost_thickness: The current frost thickness [m].
            fin_pitch: The distance between the fins [m].
            fin_thickness: The thickness of a single fin [m].
        Returns:
            The reduced space between the frosted fins [m].
        """
        return fin_pitch - fin_thickness - 2 * frost_thickness

    def _calculate_flow_area_air(self, space_between_frost: float, fin_height: float, fin_amount: int, 
                                 tube_diameter_w_frost: float, tubes_per_layer: int) -> float:
        """
        Calculates the total cross-sectional area for air flow.

        Args:
            space_between_frost: The air flow channel width between frosted fins [m].
            fin_height: The height of the fins [m].
            fin_amount: The total number of fins [-].
            tube_diameter_w_frost: The effective tube outer diameter including frost [m].
            tubes_per_layer: The number of tubes per layer [-].
        Returns:
            The total air flow area [m^2].
        """
        return (fin_amount - 1) * (fin_height - tube_diameter_w_frost * tubes_per_layer) * space_between_frost
    
    def _calculate_frost_surface_area(self, tube_diameter_w_frost:float, space_between_frost:float, fin_segment_height:float, fin_segment_length:float, fin_segment_amount:int)-> float:
        """
        Calculates the total frost surface area on the finned tube evaporator (area used for heat transfer from air to frost surface).

        Args:
            tube_diameter_w_frost (float): The effective tube outer diameter including frost [m].
            space_between_frost (float): The air flow channel width between frosted fins including frost [m].
            fin_segment_height (float): The height of a single fin segment [m].
            fin_segment_length (float): The length of a single fin segment [m].
            fin_segment_amount (int): The total number of fin segments [-].

        Returns:
            float: The total frost surface area [m²].
        """

        # Calculate area of one fin and one tube segment
        A_one_tube_segment_frost = np.pi * tube_diameter_w_frost * space_between_frost
        A_one_fin_segment_frost  = 2 * ( (fin_segment_height * fin_segment_length) - (0.25 * np.pi * tube_diameter_w_frost**2) )

        return fin_segment_amount * (A_one_tube_segment_frost + A_one_fin_segment_frost)