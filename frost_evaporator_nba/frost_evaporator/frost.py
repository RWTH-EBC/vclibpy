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
        Calculates and updates density and k_frost based on the state.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values (like T_frost_surface).
        """
        
        # Get the current *guess* for T_frost_surface from the state
        T_frost = state.hmt.T_frost_surface
        T_dew_point = state.air.T_dew_point
        
        # Calculate the Target Density based on current T
        target_density = self._calculate_density(
            T_frost_surface=T_frost,
            T_dew_point=T_dew_point
        )

        # Retrieve the existing density (Previous Iteration)
        current_density = state.frost.density

        # Safety check for the very first step where density might be 0 or None
        if current_density is None or current_density <= 0.0:
            current_density = target_density

        # Apply Under-Relaxation
        relaxation_factor=0.4
        new_density = (relaxation_factor * target_density) + \
                      ((1.0 - relaxation_factor) * current_density)
        

        new_k_frost = self._calculate_k_frost(
            new_density=new_density
        )
        
        # Update the state with the new properties
        state.frost.set("density", new_density)
        state.frost.set("k_frost", new_k_frost)


    def step_forward(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Performs the one-time integration step for frost thickness.
        
        This is NOT safe to call in a loop. Call it ONCE after the
        iterative loop has converged.
        """

        if state.hmt.T_frost_surface > 273.15:
            raise ValueError("Calculated frost surface temperature is above freezing point.")
        
        # Get the *converged* values from the state
        prev_thickness = state.frost.thickness
        prev_frost_mass = state.frost.mass
        m_dot_frost_flux = state.hmt.m_dot_frost_flux
        m_dot_frost_total = state.hmt.m_dot_frost_total
        new_density = state.frost.density
        
        # Calculate the new thickness
        new_thickness = self._calculate_thickness(
            prev_thickness=prev_thickness,
            m_dot_frost_flux=m_dot_frost_flux,
            new_density=new_density 
        )

        # Calculate the tube diameter with frost
        new_tube_diameter_w_frost = self._calculate_tube_diameter_w_frost(
            frost_thickness=new_thickness,
            tube_outer_diameter=self.params.tube_outer_diameter
        )
 
        # Calculate the space between frost layers
        new_space_between_frost = self._calculate_space_between_frost(
            frost_thickness=new_thickness, 
            fin_pitch=self.params.fin_pitch, 
            fin_thickness=self.params.fin_thickness
        )

        # Calculate the new flow area for air
        new_flow_area_air = self._calculate_flow_area_air(
            space_between_frost=new_space_between_frost, 
            fin_height=self.params.fin_height, 
            fin_amount=self.params.fin_amount, 
            tube_diameter_w_frost=new_tube_diameter_w_frost, 
            tubes_per_layer=self.params.tubes_per_layer
        )

        # Calculate new frost mass
        new_frost_mass = prev_frost_mass + m_dot_frost_total * self.params.time_step
        
        # Update the state with the final new geometric values
        state.frost.set("thickness", new_thickness)
        state.frost.set("tube_diameter_w_frost", new_tube_diameter_w_frost)
        state.frost.set("space_between_frost", new_space_between_frost)
        state.frost.set("flow_area_air", new_flow_area_air)
        state.frost.set("mass", new_frost_mass)

    
    def _calculate_density(self, T_frost_surface: float, T_dew_point: float) -> float:
        """
        Determines the new frost density.
        
        Args:
            T_frost_surface: The frost surface temperature at the *start* of the step [K].
        
        Returns:
            The calculated new frost density [kg/m^3].
        """
        if self.params.frost_density_correlation_choice == "jonas_diss":        
            return 650 * np.exp(0.277 * (T_frost_surface - 273.15))
        if self.params.frost_density_correlation_choice == "da_silva_paper":  
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


    def _calculate_thickness(self, prev_thickness: float, m_dot_frost_flux: float, new_density: float) -> float:
        """
        Determines the new frost thickness.
        
        Args:
            prev_thickness: The frost thickness from the *previous* step [m].
            m_dot_frost_flux: The mass flux of frost [kg/(m^2*s)].
            new_density: The *newly* calculated frost density [kg/m^3].
            
        Returns:
            The calculated new frost thickness [m].
        """
        if self.params.frost_thickness_correlation_choice == "jonas_diss":
            # Euler forward step: new = old + delta
            return prev_thickness + (m_dot_frost_flux * self.params.time_step) / new_density
        else:
            raise ValueError(f"Unknown frost thickness correlation: {self.params.frost_thickness_correlation_choice}")
        

    def _calculate_k_frost(self, new_density: float) -> float:
        """
        Determines the new thermal conductivity.
        
        Args:
            new_density: The *newly* calculated frost density [kg/m^3].
            
        Returns:
            The calculated new frost thermal conductivity [W/(m*K)].
        """
        if self.params.frost_conductivity_correlation_choice == "A":
            return 1.202e-3 * new_density ** 0.963
        else:
            raise ValueError(f"Unknown frost conductivity correlation: {self.params.frost_conductivity_correlation_choice}")
    

    def _calculate_tube_diameter_w_frost(self, frost_thickness:float, tube_outer_diameter:float) -> float:
            """
            Calculates the effective tube outer diameter including frost.
            
            Args:
                frost_thickness: The current frost thickness [m].
                tube_outer_diameter: The outer diameter of the clean tube [m].
                
            Returns:
                The tube outer diameter plus twice the frost thickness [m].
            """
            return tube_outer_diameter + 2 * frost_thickness

    def _calculate_space_between_frost(self, frost_thickness:float, fin_pitch:float, fin_thickness:float) -> float:
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


    def _calculate_flow_area_air(self, space_between_frost:float, fin_height:float, fin_amount:int, tube_diameter_w_frost:float, tubes_per_layer:int) -> float:
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
        return (fin_amount-1) * (fin_height - tube_diameter_w_frost * tubes_per_layer) * space_between_frost