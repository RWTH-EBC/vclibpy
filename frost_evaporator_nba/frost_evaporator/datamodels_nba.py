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

        # Time
        time_step: float,

        gravity: float,

        # Correlations Choices
        frost_density_correlation_choice: str,
        frost_thickness_correlation_choice: str,
        frost_conductivity_correlation_choice: str,

        # Fan and Air Side Pressure Loss Parameters
        hydraulic_fan_power: float,
        pressure_loss_fit_factor: float,

        # Geometrie Parameters
        fin_pitch: float,
        fin_height: float,
        fin_length: float,
        fin_thickness: float,
        fin_amount: int,
        fin_thermal_conductivity: float,
        tube_outer_diameter: float,
        tube_inner_diameter: float,
        tube_layers: int,
        tubes_per_layer: int,
        tube_thermal_conductivity: float,

        alpha_0: float,



    ):
        super().__init__()

        tube_length = fin_pitch * (fin_amount-1)
        tube_amount = tube_layers * tubes_per_layer
        total_tube_length = fin_length * tube_layers * tubes_per_layer
        fin_segment_amount = fin_amount * tube_amount
        fin_segment_height = fin_height / tubes_per_layer
        fin_segment_length = fin_length / tube_layers



        self.set("time_step", time_step, "s", "Simulation time step for evaporator model")

        self.set("gravity", gravity, "m/s^2", "Gravitational acceleration")

        self.set("frost_density_correlation_choice", frost_density_correlation_choice, "-", "Choice of correlation for frost density")
        self.set("frost_thickness_correlation_choice", frost_thickness_correlation_choice, "-", "Choice of correlation for frost thickness")
        self.set("frost_conductivity_correlation_choice", frost_conductivity_correlation_choice, "-", "Choice of correlation for frost thermal conductivity")

        self.set("hydraulic_fan_power", hydraulic_fan_power, "W", "Hydraulic power of the fan used for air-side calculations")
        self.set("pressure_loss_fit_factor", pressure_loss_fit_factor, "-", "Fit factor for air-side pressure loss calculations")
        self.set("ambient_pressure", 101325.0, "Pa", "Ambient pressure for air-side calculations")

        self.set("fin_pitch", fin_pitch, "m", "Spacing between fins (from center to center)")
        self.set("fin_height", fin_height, "m", "Height of each fin")
        self.set("fin_length", fin_length, "m", "Length of each fin")
        self.set("fin_thickness", fin_thickness, "m", "Thickness of each fin")
        self.set("fin_amount", fin_amount, "-", "Total number of fins")
        self.set("fin_thermal_conductivity", fin_thermal_conductivity, "W/m/K", "Thermal conductivity of fin material")

        self.set("tube_outer_diameter", tube_outer_diameter, "m", "Outer diameter of tubes")
        self.set("tube_inner_diameter", tube_inner_diameter, "m", "Inner diameter of tubes")
        self.set("tube_length", tube_length, "m", "Length of each tube (assumed equal to fin length)")
        self.set("tube_layers", tube_layers, "-", "Number of tube layers in the evaporator")
        self.set("tubes_per_layer", tubes_per_layer, "-", "Number of tubes per layer (for air flow calculations)")	
        self.set("tube_amount", tube_amount, "-", "Total number of tubes in the evaporator")
        self.set("total_tube_length", total_tube_length, "m", "Total length of all tubes in the evaporator")
        self.set("tube_thermal_conductivity", tube_thermal_conductivity, "W/m/K", "Thermal conductivity of tube material")

        self.set("fin_segment_amount", fin_segment_amount, "-", "Total number of fin-tube segments in the evaporator")
        self.set("fin_segment_height", fin_segment_height, "m", "Height of each fin segment")
        self.set("fin_segment_length", fin_segment_length, "m", "Length of each fin segment")

        

        self.set("alpha_0", alpha_0, "W/(m^2*K)", "Coefficient for two-phase htc (VDI Wärmeatlas H2 Tab.1)")
        self.set("q_dot_0", 20000, "W/m^2", "Normalized heat flux for propane two-phase htc (VDI Wärmeatlas H2 Tab.1)")



###################################################################################
# Input Variables
###################################################################################

class AirInputs(VariableContainer):
    """Holds all external inputs for the air-side."""
    def __init__(self, T_in: float, p_in: float, W_in: float):
        super().__init__()
        self.set("T_in",  T_in, "K", "Inlet air temperature")
        self.set("W_in",  W_in, "kg/kg", "Inlet air absolute humidity")

class RefrigerantInputs(VariableContainer):
    """Holds all external inputs for the refrigerant-side."""
    def __init__(self, h_in: float, p_eva: float, m_flow: float):
        super().__init__()
        self.set("h_in", h_in, "J/kg", "Inlet refrigerant enthalpy")
        self.set("p_eva", p_eva, "Pa", "Refrigerant pressure in Evaporator (constant)")
        self.set("m_flow", m_flow, "kg/s", "Inlet refrigerant mass flow rate")

class FrostEvaporatorInputs(VariableContainer):
    """
    Acts as the main container for all external inputs
    for the complex frost-evaporator model.
    """
    def __init__(self, air_inputs: AirInputs, refrigerant_inputs: RefrigerantInputs):
        """
        Initializes by taking the pre-built sub-input objects.
        """
        super().__init__() 
        
        self.air = air_inputs
        self.refrigerant = refrigerant_inputs
        
        # List of sub-inputs for easier iteration
        self._sub_inputs = [self.air, self.refrigerant]

    # --- You can add override methods for logging ---
    
    def get_all_variables(self) -> dict:
        """
        Gathers all variables from all sub-inputs into one flat 
        dictionary. This is useful for logging.
        """
        all_vars = {}
        for input_container in self._sub_inputs:
            # Add a prefix (e.g., "air_T_in", "refrigerant_p_in")
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
        self.set("space_between_frost", 0.0, "m", "Space between frost layers")
        self.set("tube_diameter_w_frost", 0.0, "m", "Tube outer diameter including frost")
        self.set("flow_area_air", 0.0, "m^2", "Free flow area for air through the evaporator")

class AirState(VariableContainer):
    """Holds all variables calculated by the air subprogram."""
    def __init__(self):
        super().__init__()

        # Output Air Definition
        self.set("T_out", 273.15, "K", "Outlet air temperature")
        self.set("W_out", 0.0, "kg/kg", "Outlet air absolute humidity")

        self.set("p_in",  101325.0, "Pa", "Inlet air pressure")

        # Average Air Properties (Inlet and Outlet)
        self.set("pressure_avg", 101325.0, "Pa", "Average air pressure")
        self.set("density_avg", 1.0, "kg/m^3", "Average air density")
        self.set("dyn_viscosity_avg", 1.8e-5, "Pa*s", "Average dynamic viscosity")
        self.set("heat_capacity_avg", 1005.0, "J/kg/K", "Average heat capacity")
        self.set("thermal_conductivity_avg", 0.025, "W/m/K", "Average thermal conductivity")
        self.set("prandtl_avg", 0.71, "-", "Average Prandtl number")
        self.set("lewis_avg", 0.85, "-", "Average Lewis number")
        self.set("rho_w_avg", 0.0 , "kg/m^3", "Average water vapor density")
    
        self.set("rho_w_frost_sat", 0.0 , "kg/m^3", "Saturation water vapor density at frost surface temperature")

        # Calculated Air Properties
        self.set("reynolds", 0.0, "-", "Air-side Reynolds number")
        self.set("nusselt", 0.0, "-", "Air-side Nusselt number")
        self.set("h_conv", 0.0, "W/m^2/K", "Air-side convective heat transfer coeff.")
        self.set("betta", 0.0, "m/s", "Air-side mass transfer coefficient")
        self.set("pressure_loss_coeff", 0.0, "-", "Air-side pressure loss coefficient")
        self.set("velocity", 0.0, "m/s", "Air velocity through the evaporator")
        self.set("m_flow", 0.0, "kg/s", "Outlet air mass flow")


        
class RefrigerantState(VariableContainer):
    """Holds all variables calculated by the refrigerant subprogram."""
    def __init__(self):
        super().__init__()
        self.set("h_out", 0.0, "J/kg", "Outlet refrigerant enthalpy")
        self.set("p_out", 0.0, "-", "Outlet refrigerant quality")
        self.set("h_conv", 0.0, "W/m^2/K", "Refrigerant-side convective heat transfer coeff.")
        self.set("T_in", 0.0, "K", "Inlet refrigerant temperature")
        self.set("T_out", 0.0, "K", "Outlet refrigerant temperature")

# --- You would do the same for the other two ---
class HeatMassTransferState(VariableContainer):
    """Holds all variables for the HMT subprogram."""
    def __init__(self):
        super().__init__()
        self.set("A_effective", 0.0, "m^2", "Effective heat transfer area")
        self.set("A_frost_surface", 0.0, "m^2", "Frost surface area for frost flux")
        self.set("R_total", 0.0, "K/W", "Total thermal resistance between air and refrigerant")
        self.set("delta_T_log", 0.0, "K", "Logarithmic mean temperature difference")
        self.set("T_frost_surface", 0.0, "K", "Frost surface temperature")
        self.set("Q_dot", 0.0, "W", "Heat transfer rate between air and refrigerant")
        self.set("m_dot_frost_flux", 0.0, "kg/s/m^2", "Mass flux rate of frost growth")
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
        self.refrigerant = RefrigerantState()
        self.hmt = HeatMassTransferState()
        self.thermo = ThermodynamicsState()

        # List of sub-states for easier iteration
        self._sub_states = [self.frost, self.air, self.refrigerant, self.hmt, self.thermo]

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
    