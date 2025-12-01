from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np

class FrostModel:
    """
    Calculates the density, thickness, and thermal conductivity of the frost layer.
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

    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the frost model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values (like T_frost_surface).
        """
        
        # ================= Get State Values =================
        current_avg_density = state.frost.density

        # ================= Calculate new Values =================
        new_k_frost = self._calculate_k_frost(average_density=current_avg_density)

        # ================= Write new values to state =================
        state.frost.set("k_frost", new_k_frost)


    def step_forward(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Performs the one-time integration step for frost thickness and updates 
        the average density based on mass/volume conservation.
        """

        # ================= Get State Values =================
        prev_thickness = state.frost.thickness
        prev_frost_mass = state.frost.mass
        prev_avg_density = state.frost.density

        m_dot_thickening = state.hmt.m_dot_thickening
        m_dot_densification = state.hmt.m_dot_densification
        m_dot_thickening_flux = state.hmt.m_dot_thickening_flux
        T_frost_surface = state.hmt.T_frost_surface
        T_dew_point = state.air.T_dew_point
        
        if T_frost_surface > self.params.water_freezing_point:
            raise ValueError("Calculated frost surface temperature is above freezing point.")
        
        
        # ================= Calculate new Values =================
        new_density_surface = self._calculate_surface_density(
            T_frost_surface = T_frost_surface, 
            T_dew_point = T_dew_point
        )

        new_avg_density, new_frost_mass = self._calculate_average_density_and_frost_mass(
            m_dot_thickening = m_dot_thickening,
            m_dot_densification = m_dot_densification,
            prev_frost_mass = prev_frost_mass,
            prev_avg_density = prev_avg_density,
            new_density_surface = new_density_surface,
        )

        new_thickness = self._calculate_thickness(
            prev_thickness=prev_thickness,
            m_dot_thickening_flux=m_dot_thickening_flux,
            new_layer_density=new_density_surface 
        )

        new_tube_diameter_w_frost = self._calculate_tube_diameter_w_frost(
            frost_thickness=new_thickness,
            tube_outer_diameter=self.params.tube_outer_diameter
        )

        new_space_between_frost = self._calculate_space_between_frost(
            frost_thickness=new_thickness, 
            fin_pitch=self.params.fin_pitch, 
            fin_thickness=self.params.fin_thickness
        )

        new_flow_area_air = self._calculate_flow_area_air(
            space_between_frost=new_space_between_frost, 
            fin_height=self.params.fin_height, 
            fin_amount=self.params.fin_amount, 
            tube_diameter_w_frost=new_tube_diameter_w_frost, 
            tubes_per_layer=self.params.tubes_per_layer
        )


        # ================= Write new values to state =================
        state.frost.set("density", new_avg_density)
        state.frost.set("thickness", new_thickness)
        state.frost.set("tube_diameter_w_frost", new_tube_diameter_w_frost)
        state.frost.set("space_between_frost", new_space_between_frost)
        state.frost.set("flow_area_air", new_flow_area_air)
        state.frost.set("mass", new_frost_mass)




    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _calculate_surface_density(self, T_frost_surface: float, T_dew_point: float) -> float:
        """
        Calculates the new frost density at the surface.

        Args:
            T_frost_surface: The frost surface temperature at the start of the step [K].
            T_dew_point: The dew point temperature of the air [K].
        Returns:
            The calculated new frost density [kg/m^3].
        Raises:
            ValueError: If the frost density correlation choice is unknown.
        """
        if self.params.frost_density_correlation_choice == "jonas_diss":        
            return 650 * np.exp(0.277 * (T_frost_surface - 273.15))
        
        elif self.params.frost_density_correlation_choice == "da_silva_paper":  
            # Coefficients from Section 4 "Results" of da Silva et al. (2011)
            a = 494.0
            b = 0.11
            c = -0.06
            
            # Convert K to Celsius
            T_f_C = T_frost_surface - 273.15
            T_dew_C = T_dew_point - 273.15
            
            # Eq. 9
            return a * np.exp(b * T_f_C + c * T_dew_C)   
        
        else:
            raise ValueError(f"Unknown frost density correlation: {self.params.frost_density_correlation_choice}")

    def _calculate_average_density_and_frost_mass(self, m_dot_thickening: float, m_dot_densification: float, prev_frost_mass: float, 
                                                  prev_avg_density: float, new_density_surface: float) -> tuple[float, float]:
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
        # Integrate MASS (Total Mass increases by BOTH flows)
        mass_added_total = (m_dot_thickening + m_dot_densification) * self.params.time_step
        new_frost_mass = prev_frost_mass + mass_added_total
        
        # Integrate VOLUME (Volume increases ONLY by thickening flow)
        # Densification mass enters existing pores, so it adds 0 volume.
        prev_vol = prev_frost_mass / prev_avg_density
        vol_added = (m_dot_thickening * self.params.time_step) / new_density_surface 
        new_total_vol = prev_vol + vol_added
        
        # Calculate New Average Density
        new_avg_density = new_frost_mass / new_total_vol

        return new_avg_density, new_frost_mass

    def _calculate_thickness(self, prev_thickness: float, m_dot_thickening_flux: float, new_layer_density: float) -> float:
        """
        Calculates the new frost thickness.

        Args:
            prev_thickness: Old thickness [m].
            m_dot_thickening_flux: Mass flux contributing specifically to thickness [kg/(m^2*s)].
            new_layer_density: Density of the new layer [kg/m^3].
        Returns:
            The new total frost thickness [m].
        Raises:
            ValueError: If the frost thickness correlation choice is unknown.
        """
        if self.params.frost_thickness_correlation_choice == "jonas_diss":
            # Euler forward step
            delta_thickness = (m_dot_thickening_flux * self.params.time_step) / new_layer_density
            return prev_thickness + delta_thickness
        else:
            raise ValueError(f"Unknown frost thickness correlation: {self.params.frost_thickness_correlation_choice}")

    def _calculate_k_frost(self, average_density: float) -> float:
        """
        Calculates the new thermal conductivity of the frost.

        Args:
            average_density: The average frost density [kg/m^3].
        Returns:
            The calculated frost thermal conductivity [W/(m*K)].
        Raises:
            ValueError: If the frost conductivity correlation choice is unknown.
        """
        if self.params.frost_conductivity_correlation_choice == "A":
            return 1.202e-3 * average_density ** 0.963
        else:
            raise ValueError(f"Unknown frost conductivity correlation: {self.params.frost_conductivity_correlation_choice}")

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