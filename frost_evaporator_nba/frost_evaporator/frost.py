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
                 delta_t: float
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
            delta_t: The simulation time step in seconds (e.g., 1.0).
        """
        self.params = parameters
        self.delta_t = delta_t

    def calculate(self, state: FrostEvaporatorState):
        """
        Runs the frost calculation for the current time step.
        
        This method reads all values from the *previous* state (t),
        calculates all *new* values (t + delta_t),
        and then updates the state object once at the end.
        """
        
        # Get all required values from the previous state
        prev_T_frost_surface = state.hmt.T_frost_surface
        prev_thickness = state.frost.thickness
        m_dot_frost_flux = state.hmt.m_dot_frost_flux # Assumed constant over delta_t
        
        # Calculate all new values for the next state (t + delta_t)
        
        # New density depends on the previous frost surface temperature
        new_density = self.calculate_density(
            T_frost_surface=prev_T_frost_surface
        )
        
        # New thickness depends on the previous thickness and the *new* density
        new_thickness = self.calculate_thickness(
            prev_thickness=prev_thickness,
            m_dot_frost_flux=m_dot_frost_flux,
            new_density=new_density 
        )

        # New k_frost depends on the *new* density
        new_k_frost = self.calculate_k_frost(
            new_density=new_density
        )
        
        # Update the state with all new values
        state.frost.set("density", new_density)
        state.frost.set("thickness", new_thickness)
        state.frost.set("k_frost", new_k_frost)

    
    def calculate_density(self, T_frost_surface: float) -> float:
        """
        Determines the new frost density.
        
        Args:
            T_frost_surface: The frost surface temperature at the *start* of the step [K].
        
        Returns:
            The calculated new frost density [kg/m^3].
        """
        if self.params.frost_density_correlation_choice == "jonas_diss":        
            return 650 * np.exp(0.277 * (T_frost_surface - 273.15))
        else:
            raise ValueError(f"Unknown frost density correlation: {self.params.frost_density_correlation_choice}")


    def calculate_thickness(self, prev_thickness: float, m_dot_frost_flux: float, new_density: float) -> float:
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
            return prev_thickness + (m_dot_frost_flux * self.delta_t) / new_density
        else:
            raise ValueError(f"Unknown frost thickness correlation: {self.params.frost_thickness_correlation_choice}")
        

    def calculate_k_frost(self, new_density: float) -> float:
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