from vclibpy.datamodels import VariableContainer

###################################################################################
# Parameters for the Frost Heat Exchanger
###################################################################################

class FrostEvaporatorParameters(VariableContainer):
    """
    Holds all constant geometric and material parameters 
    for the complex evaporator model.
    """
    def __init__(
        self,
        frost_density_correlation_choice: str,
        frost_thickness_correlation_choice: str,
        frost_conductivity_correlation_choice: str,

        # fin_pitch: float,
        # tube_diameter: float,
        # number_of_tubes: int,
        # ... add all other 50+ parameters
    ):
        super().__init__()
        self.set("frost_density_correlation_choice", frost_density_correlation_choice, "-", "Choice of correlation for frost density")
        self.set("frost_thickness_correlation_choice", frost_thickness_correlation_choice, "-", "Choice of correlation for frost thickness")
        self.set("frost_conductivity_correlation_choice", frost_conductivity_correlation_choice, "-", "Choice of correlation for frost thermal conductivity")

        # self.set("fin_pitch", fin_pitch, "m", "Spacing between fins")
        # self.set("tube_diameter", tube_diameter, "m", "Outer tube diameter")
        # self.set("number_of_tubes", number_of_tubes, "-", "Total tubes")
        # ... self.set(...) for all other parameters



###################################################################################
# Input Variables
###################################################################################

class AirInputs(VariableContainer):
    """Holds all external inputs for the air-side."""
    def __init__(self, T_in: float, m_flow_in: float, humidity_in: float):
        super().__init__()
        # self.set("T_in", T_in, "K", "Inlet air temperature")
        # self.set("m_flow_in", m_flow_in, "kg/s", "Inlet air mass flow rate")
        # self.set("humidity_in", humidity_in, "kg/kg", "Inlet air absolute humidity")
        # self.set("p_in", 101325.0, "Pa", "Inlet air pressure (default sea level)")

class CoolingFluidInputs(VariableContainer):
    """Holds all external inputs for the cooling-fluid-side."""
    def __init__(self, h_in: float, p_in: float, m_flow_in: float):
        super().__init__()
        # self.set("h_in", h_in, "J/kg", "Inlet fluid enthalpy")
        # self.set("p_in", p_in, "Pa", "Inlet fluid pressure")
        # self.set("m_flow_in", m_flow_in, "kg/s", "Inlet fluid mass flow rate")

class FrostEvaporatorInputs(VariableContainer):
    """
    Acts as the main container for all external inputs
    for the complex frost-evaporator model.
    """
    def __init__(self, air_inputs: AirInputs, fluid_inputs: CoolingFluidInputs):
        """
        Initializes by taking the pre-built sub-input objects.
        """
        super().__init__() 
        
        self.air = air_inputs
        self.fluid = fluid_inputs
        
        # List of sub-inputs for easier iteration
        self._sub_inputs = [self.air, self.fluid]

    # --- You can add override methods for logging ---
    
    def get_all_variables(self) -> dict:
        """
        Gathers all variables from all sub-inputs into one flat 
        dictionary. This is useful for logging.
        """
        all_vars = {}
        for input_container in self._sub_inputs:
            # Add a prefix (e.g., "air_T_in", "fluid_p_in")
            prefix = input_container.__class__.__name__.replace("Inputs", "").lower()
            for name, var in input_container.get_variables().items():
                all_vars[f"{prefix}_{name}"] = var
        return all_vars

    def convert_to_str_value_format(self, with_unit_and_description: bool) -> dict:
        """
        Overrides the base method to correctly log all nested variables.
        """
        all_vars = self.get_all_variables()
        if with_unit_and_description:
            return {f"{k} in {v.unit} ({v.description})": v.value 
                    for k, v in all_vars.items() if v.value is not None}
        return {k: v.value for k, v in all_vars.items() if v.value is not None}

    def get_name(self):
        """Overrides the base method for a unique name."""
        all_vars = self.get_all_variables()
        return ";".join([f"{k}={str(round(v.value, 3)).replace('.', '_')}" 
                         for k, v in all_vars.items() if v.value is not None])



###################################################################################
# Calculated Output Variables
####################################################################################

class FrostState(VariableContainer):
    """Holds all variables calculated by the frost subprogram."""
    def __init__(self):
        super().__init__()
        self.set("density", 100.0, "kg/m^3", "Frost density")
        self.set("thickness", 0.0, "m", "Frost layer thickness")
        self.set("k_frost", 0.1, "W/m/K", "Frost thermal conductivity")
        # ... add all other frost-specific variables

class AirState(VariableContainer):
    """Holds all variables calculated by the air subprogram."""
    def __init__(self):
        super().__init__()
        # Define your variables with defaults
        # self.set("h_conv", 0.0, "W/m^2/K", "Air-side convective heat transfer coeff.")
        # self.set("m_flow_air_out", 0.0, "kg/s", "Outlet air mass flow")
        # self.set("T_air_out", 273.15, "K", "Outlet air temperature")
        # self.set("p_air_out", 101325.0, "Pa", "Outlet air pressure")
        # ... add all other air-specific variables
        
class CoolingFluidState(VariableContainer):
    """Holds all variables calculated by the cooling fluid subprogram."""
    def __init__(self):
        super().__init__()
        # self.set("T_fluid_out", 270.15, "K", "Outlet fluid temperature")
        # self.set("dp_fluid", 0.0, "Pa", "Fluid-side pressure drop")
        # ... add all other fluid-specific variables

# --- You would do the same for the other two ---
class HeatMassTransferState(VariableContainer):
    """Holds all variables for the HMT subprogram."""
    def __init__(self):
        super().__init__()
        # self.set("Q_sensible", 0.0, "W", "Sensible heat transfer")
        # self.set("Q_latent", 0.0, "W", "Latent heat transfer (frost/condensation)")
        self.set("m_dot_frost_flux", 0.0, "kg/s/m^2", "Mass flux rate of frost growth")
        self.set("T_frost_surface", 0.0, "K", "Frost surface temperature")
        # ...
        
class ThermodynamicsState(VariableContainer):
    """Holds overall thermodynamic properties and wall temperatures."""
    def __init__(self):
        super().__init__()
        # self.set("T_wall_avg", 270.15, "K", "Average external wall temperature")
        # self.set("Q_total", 0.0, "W", "Total heat transfer to refrigerant")
        # ...

# TODO, I HAVE NOT PLAN WHAT THIS DOES
class FrostEvaporatorState(VariableContainer):
    """
    Acts as the main container for all sub-states of the
    complex frost-evaporator model.
    
    This object is passed to all subprograms, which read from
    and write to their dedicated state objects (e.g., state.air, state.frost).
    """
    def __init__(self):
        # We still call super() to get the base functionality like .copy()
        # but we won't add variables directly to this top-level object.
        super().__init__() 
        
        # --- Compose the state from its sub-states ---
        self.frost = FrostState()
        self.air = AirState()
        self.fluid = CoolingFluidState()
        self.hmt = HeatMassTransferState()
        self.thermo = ThermodynamicsState()

        # List of sub-states for easier iteration
        self._sub_states = [self.frost, self.air, self.fluid, self.hmt, self.thermo]

    # --- IMPORTANT: Override logging methods ---
    
    def get_all_variables(self) -> dict:
        """
        Gathers all variables from all sub-states into one flat 
        dictionary. This is crucial for logging.
        """
        all_vars = {}
        for state_container in self._sub_states:
            # Add a prefix to avoid name collisions (e.g., "air_T_out")
            prefix = state_container.__class__.__name__.replace("State", "").lower()
            for name, var in state_container.get_variables().items():
                all_vars[f"{prefix}_{name}"] = var
        return all_vars

    def convert_to_str_value_format(self, with_unit_and_description: bool) -> dict:
        """
        Overrides the base method to correctly log all nested variables.
        """
        all_vars = self.get_all_variables()
        if with_unit_and_description:
            return {f"{k} in {v.unit} ({v.description})": v.value 
                    for k, v in all_vars.items() if v.value is not None}
        return {k: v.value for k, v in all_vars.items() if v.value is not None}

    def get_name(self):
        """Overrides the base method for a unique name."""
        all_vars = self.get_all_variables()
        return ";".join([f"{k}={str(round(v.value, 3)).replace('.', '_')}" 
                         for k, v in all_vars.items() if v.value is not None])
    