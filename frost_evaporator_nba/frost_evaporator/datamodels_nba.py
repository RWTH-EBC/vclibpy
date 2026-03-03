from vclibpy.datamodels import VariableContainer
import yaml
from pathlib import Path

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
        # General Parameters
        time_step: float,
        gravity: float,
        alpha_0: float,

        register_amount: int,
        layer_amount: int,
        fan_amount: int,
        refrigerant: str,

        # Correlations Choices
        frost_density_correlation_choice: str,
        frost_conductivity_correlation_choice: str,
        pressure_drop_correlation_choice: str,
        h_conv_air_correlation_choice: str,
        fan_selection: str,

        # Correction Factors
        correction_factor_h_conv_air: float,
        correction_factor_h_conv_ref_2ph: float,
        correction_factor_betta_air: float,
        correction_factor_surface_density: float,
        correction_factor_k_frost: float,
        correction_factor_pressure_loss: float,
        correction_factor_frost_diffusion: float,

        # Geometry Parameters
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
    ):
        super().__init__()

        # General
        self.set("time_step", time_step, "s", "Simulation time step for evaporator model")
        self.set("gravity", gravity, "m/s^2", "Gravitational acceleration")
        self.set("water_freezing_point", 273.15, "K", "The freezing point temperature of water.")
        self.set("ambient_pressure", 101325.0, "Pa", "Ambient pressure for air-side calculations")
        self.set("ice_density", 918, "kg/m^3", "Density of solid ice (for porosity calculation)")
        self.set("diffusivity_w_vapor_in_air", 2.12e-5, "m^2/s", "diffusivity of water vapor in air")
        self.set("alpha_0", alpha_0, "W/(m^2*K)", "Coefficient for two-phase htc (VDI Wärmeatlas H2 Tab.1)")
        self.set("q_dot_0", 20000, "W/m^2", "Normalized heat flux for propane two-phase htc (VDI Wärmeatlas H2 Tab.1)")
        self.set("refrigerant", refrigerant, "-", "Type of refrigerant used in the evaporator")
        
        # Structure
        self.set("register_amount", register_amount, "-", "Number of registers in the evaporator")
        self.set("layer_amount", layer_amount, "-", "Number of layers in the evaporator")
        self.set("fan_amount", fan_amount, "-", "Number of fans in the evaporator")

        # Correlations
        self.set("frost_density_correlation_choice", frost_density_correlation_choice, "-", "Choice of correlation for frost density")
        self.set("frost_conductivity_correlation_choice", frost_conductivity_correlation_choice, "-", "Choice of correlation for frost thermal conductivity")
        self.set("pressure_drop_correlation_choice", pressure_drop_correlation_choice, "-", "Choice of correlation for air-side pressure drop")
        self.set("h_conv_air_correlation_choice", h_conv_air_correlation_choice, "-", "Choice of correlation for air-side convective heat transfer coefficient")
        self.set("fan_selection", fan_selection, "-", "Fan selection for air-side pressure drop calculation")

        # Correction Factors
        self.set("correction_factor_h_conv_air", correction_factor_h_conv_air, "-", "Correction factor to scale h_conv of air")
        self.set("correction_factor_h_conv_ref_2ph", correction_factor_h_conv_ref_2ph, "-", "Scales HTC in the boiling zone")
        self.set("correction_factor_betta_air", correction_factor_betta_air, "-", "Correction factor to scale betta of air")
        self.set("correction_factor_surface_density", correction_factor_surface_density, "-", "Correction factor to scale the frost surface density")
        self.set("correction_factor_k_frost", correction_factor_k_frost, "-", "Correction factor to scale the frost heat transfer")
        self.set("correction_factor_pressure_loss", correction_factor_pressure_loss, "-", "Correction factor to scale the air-side pressure loss")
        self.set("correction_factor_frost_diffusion", correction_factor_frost_diffusion, "-", "Correction factor to scale the diffusivity (split of thickenig vs densifying)")

        # Geometry Inputs (Basiswerte, die sich ändern können)
        self.set("fin_pitch", fin_pitch, "m", "Spacing between fins (center to center)")
        self.set("fin_height", fin_height, "m", "Height of each fin")
        self.set("fin_length", fin_length, "m", "Length of each fin")
        self.set("fin_thickness", fin_thickness, "m", "Thickness of each fin")
        self.set("fin_amount", fin_amount, "-", "Total number of fins")
        self.set("fin_thermal_conductivity", fin_thermal_conductivity, "W/m/K", "Thermal conductivity of fin")

        self.set("tube_outer_diameter", tube_outer_diameter, "m", "Outer diameter of tubes")
        self.set("tube_inner_diameter", tube_inner_diameter, "m", "Inner diameter of tubes")
        self.set("tube_layers", tube_layers, "-", "Number of tube layers")
        self.set("tubes_per_layer", tubes_per_layer, "-", "Number of tubes per layer")
        self.set("tube_thermal_conductivity", tube_thermal_conductivity, "W/m/K", "Thermal conductivity of tube")

        # Initial geometry calculation
        self.recalculate_geometry()

    def recalculate_geometry(self):
        """
        Calculates the geometric parameters based on new evaporator Setups. Has to be calle
        if for example the pitch is changed.
        """
        # Read Input Parameters
        fin_pitch = self.fin_pitch
        fin_thickness = self.fin_thickness
        fin_amount = self.fin_amount
        fin_height = self.fin_height
        fin_length = self.fin_length
        tube_layers = self.tube_layers
        tubes_per_layer = self.tubes_per_layer
        
        # Calculate dependent Parameters
        fin_spacing = fin_pitch - fin_thickness
        tube_length = fin_pitch * (fin_amount - 1)
        tube_amount = tube_layers * tubes_per_layer
        total_tube_length = tube_length * tube_layers * tubes_per_layer
        
        fin_segment_amount = fin_amount * tube_amount
        
        fin_segment_height = fin_height / tubes_per_layer
        fin_segment_length = fin_length / tube_layers

        transverse_tube_pitch = fin_height / tubes_per_layer
        longitudinal_tube_pitch = fin_length / tube_layers


        # Write dependend Parameters to dict
        self.set("fin_spacing", fin_spacing, "m", "Distance between fin surfaces")
        self.set("tube_length", tube_length, "m", "Length of each tube")
        self.set("tube_amount", tube_amount, "-", "Total number of tubes")
        self.set("total_tube_length", total_tube_length, "m", "Total length of all tubes")
        
        self.set("fin_segment_amount", fin_segment_amount, "-", "Total number of fin-tube segments")
        self.set("fin_segment_height", fin_segment_height, "m", "Height of each fin segment")
        self.set("fin_segment_length", fin_segment_length, "m", "Length of each fin segment")
        
        self.set("longitudinal_tube_pitch", longitudinal_tube_pitch, "m", "Longitudinal pitch")
        self.set("transverse_tube_pitch", transverse_tube_pitch, "m", "Transverse pitch")

    @classmethod
    def from_yaml(cls, yaml_path: str):
        """
        Method to load parameters from a YAML file.
        Calculates geometric splits (registers/layers) automatically.
        """
        path = Path(yaml_path)
        if not path.exists():
            raise FileNotFoundError(f"Configuration file not found at: {path.absolute()}")

        with open(path, 'r') as f:
            cfg = yaml.safe_load(f)

        # Helper shortcuts to sections
        gen  = cfg.get('general', {})
        corr = cfg.get('correlations', {})
        fact = cfg.get('correction_factors', {})
        geo  = cfg.get('geometry', {})

        # --- Geometric Transformation ---
        # Convert Global Physical Dimensions -> Simulation Model Dimensions
        register_amount = float(geo.get('register_amount', 1.0))
        layer_amount    = int(geo.get('layer_amount', 1))
        fan_amount      = int(geo.get('fan_amount', 1))

        # 1. Fin Dimensions (Global -> Model)
        fin_height_model = geo['total_fin_height'] / register_amount
        fin_length_model = geo['total_fin_length'] / layer_amount

        # 2. Tube Counts (Global -> Model)
        # Using int() to ensure we pass integers to __init__
        tube_layers_model     = int(geo['total_tube_layers'] / layer_amount)
        tubes_per_layer_model = int(geo['total_tubes_per_layer'] / register_amount)

        return cls(
            # General
            time_step   = gen['time_step'],
            gravity     = gen['gravity'],
            alpha_0     = gen['alpha_0'],
            refrigerant = gen['refrigerant'],

            register_amount = register_amount,
            layer_amount    = layer_amount,
            fan_amount      = fan_amount,

            # Correlations
            frost_density_correlation_choice      = corr['frost_density_choice'],
            frost_conductivity_correlation_choice = corr['frost_conductivity_choice'],
            pressure_drop_correlation_choice      = corr['pressure_drop_choice'],
            h_conv_air_correlation_choice         = corr['h_conv_air_choice'],
            fan_selection                         = corr['fan_choice'],

            # Correction Factors
            correction_factor_h_conv_air         = fact['h_conv_air'],
            correction_factor_h_conv_ref_2ph     = fact['h_conv_ref_2ph'],
            correction_factor_betta_air          = fact['betta_air'],
            correction_factor_surface_density    = fact['surface_density'],
            correction_factor_k_frost            = fact['k_frost'],
            correction_factor_pressure_loss      = fact['pressure_loss'],
            correction_factor_frost_diffusion    = fact['frost_diffusion'],

            # Geometry (Calculated Model Segments)
            fin_height      = fin_height_model,
            fin_length      = fin_length_model,
            tube_layers     = tube_layers_model,
            tubes_per_layer = tubes_per_layer_model,

            # Geometry (Direct Pass-through)
            fin_pitch                 = geo['fin_pitch'],
            fin_thickness             = geo['fin_thickness'],
            fin_amount                = geo['fin_amount'],
            fin_thermal_conductivity  = geo['fin_thermal_conductivity'],
            tube_outer_diameter       = geo['tube_outer_diameter'],
            tube_inner_diameter       = geo['tube_inner_diameter'],
            tube_thermal_conductivity = geo['tube_thermal_conductivity'],
        )




###################################################################################
# Input Variables
###################################################################################

class AirInputs(VariableContainer):
    """Holds all external inputs for the air-side."""
    def __init__(self, T_in: float, p_in: float, W_in: float, fan_speed_rpm: float):
        super().__init__()
        self.set("T_in",  T_in, "K", "Inlet air temperature")
        self.set("W_in",  W_in, "kg/kg", "Inlet air absolute humidity")
        self.set("p_in",  p_in, "Pa", "Inlet air pressure")
        self.set("fan_speed_rpm", fan_speed_rpm, "rpm", "Fan speed in revolutions per minute")

class RefrigerantInputs(VariableContainer):
    """Holds all external inputs for the refrigerant-side."""
    def __init__(self, h_in: float, p_eva: float, m_dot: float):
        super().__init__()
        self.set("h_in", h_in, "J/kg", "Inlet refrigerant enthalpy")
        self.set("p_eva", p_eva, "Pa", "Refrigerant pressure in Evaporator (constant)")
        self.set("m_dot", m_dot, "kg/s", "Inlet refrigerant mass flow rate")

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

    def __str__(self):
        """
        Provides a user-friendly string representation (pretty print) 
        in the format: VariableName: Value [Unit].
        """
        all_vars = self.get_all_variables()
        
        # 1. Collect lines for all valid variables
        output_lines = []
        for name, var in all_vars.items():
            value = var.value
            unit = var.unit
            description = var.description
            
            if value is not None:
                # Format the value (round to 3 decimal places for readability)
                # Ensure the value is converted to a string before rounding
                try:
                    formatted_value = f"{value:.8f}"
                except (TypeError, ValueError):
                    formatted_value = str(value) # Fallback for non-numeric types

                # Determine the prefix/group for sorting and clarity
                # e.g., 'frost_T_surf' becomes 'frost' group
                group_name = name.split('_')[0].capitalize()
                
                # Create the final line
                line = f"{name}: {formatted_value} [{unit}]"
                output_lines.append((group_name, line))

        # 2. Sort lines by their group name (e.g., 'Air' then 'Frost')
        output_lines.sort(key=lambda x: x[0])
        
        # 3. Assemble the final output, adding sub-state headings
        final_str = f"--- {self.__class__.__name__} State ---\n"
        current_group = None

        for group, line in output_lines:
            if group != current_group:
                # Add a section header for the sub-state
                final_str += f"\n*** {group} State ***\n"
                current_group = group
            
            final_str += line + "\n"
            
        final_str += "--------------------------------------"
        
        return final_str


###################################################################################
# Calculated State Variables
####################################################################################

class FrostState(VariableContainer):
    """Holds all variables calculated by the frost subprogram."""
    def __init__(self):
        super().__init__()
        self.set("density", 100.0, "kg/m^3", "Frost density")
        self.set("thickness", 1e-6, "m", "Frost layer thickness")

        self.set("k_frost", 0.0, "W/m/K", "Frost thermal conductivity")
        self.set("mass", 1e-9, "kg", "Total mass of the accumulated frost")

        self.set("space_between_frost", 0.0, "m", "Space between frost layers")
        self.set("tube_diameter_w_frost", 0.0, "m", "Tube outer diameter including frost")
        self.set("collar_diameter_w_frost", 0.0, "m", "Collar Diameter of the tube with frost")
        self.set("flow_area_air", 0.0, "m^2", "Free flow area for air through the evaporator")
        self.set("A_frost_surface", 0.0, "m^2", "Frost surface area for frost flux")

class AirState(VariableContainer):
    """Holds all variables calculated by the air subprogram."""
    def __init__(self):
        super().__init__()

        # Output Air Definition
        self.set("T_out", 273.15, "K", "Outlet air temperature")
        self.set("W_out", 0.0, "kg/kg", "Outlet air absolute humidity")
        self.set("p_out",  101325.0, "Pa", "Outlet air pressure")

        # Average Air Properties (Inlet and Outlet)
        self.set("T_avg", 273.15, "K", "Average air temperature")
        self.set("W_avg", 0.0, "kg/kg", "Average air absolute humidity")
        self.set("p_avg", 101325.0, "Pa", "Average air pressure")
        self.set("pressure_avg", 101325.0, "Pa", "Average air pressure")
        self.set("density_avg", 1.0, "kg/m^3", "Average air density")
        self.set("dyn_viscosity_avg", 1.8e-5, "Pa*s", "Average dynamic viscosity")
        self.set("heat_capacity_avg", 1005.0, "J/kg/K", "Average heat capacity")
        self.set("thermal_conductivity_avg", 0.025, "W/m/K", "Average thermal conductivity")
        self.set("prandtl_avg", 0.71, "-", "Average Prandtl number")
        self.set("lewis_avg", 0.85, "-", "Average Lewis number")
        self.set("rho_w_in", 0.0 , "kg/m^3", "Water vapor density at inlet")
        self.set("rho_w_out", 0.0 , "kg/m^3", "Water vapor density at inlet")

        self.set("roughness_multiplier", 1.0, "-", "Multiplier to account for increased roughness due to frost")
    
        self.set("rho_w_frost_surface_sat", 0.0 , "kg/m^3", "Saturation water vapor density at frost surface temperature")
        self.set("W_frost_surface_sat", 0.0, "kg/kg, Frost Surface saturated air absolute humidity")
        self.set("rho_w_frost_base_sat", 0.0 , "kg/m^3", "Saturation water vapor density at frost base temperature")
        self.set("W_frost_base_sat", 0.0, "kg/kg, Frost Base saturated air absolute humidity")

        # Calculated Air Properties
        self.set("reynolds", 0.0, "-", "Air-side Reynolds number")
        self.set("nusselt", 0.0, "-", "Air-side Nusselt number")
        self.set("h_conv", 0.0, "W/m^2/K", "Air-side convective heat transfer coeff.")
        self.set("betta", 0.0, "m/s", "Air-side mass transfer coefficient")
        self.set("pressure_loss_coeff", 0.0, "-", "Air-side pressure loss coefficient")
        self.set("velocity", 0.0, "m/s", "Air velocity through the evaporator")
        self.set("m_dot_humid", 0.0, "kg/s", "Humid air mass flow")
        self.set("m_dot_dry", 0.0, "kg/s", "Dry air mass flow")

        self.set("h_in", 0.0, "J/kg", "Inlet air enthalpy per dry air kg")
        self.set("h_out", 0.0, "J/kg", "Outlet air enthalpy per dry air kg")
        self.set("h_ice", 0.0, "J/kg", "Enthalpy of ice formed from frost growth")
        self.set("T_dew_point", 265.0, "Dew Point Temperature of the inlet air")

        self.set("R_in", 0, "-", "Relative Humidity of Inlet air")
        self.set("R_out", 0, "-", "Relative Humidity of Outlet air")

        self.set("pressure_drop", 0.0, "Pa", "Air-side pressure drop through the current layer")
        self.set("v_dot_fan_m3h_segment", 0.0, "m^3/h", "Volume flow rate through one segment")

        self.set("total_system_pressure_drop", 0.0, "Pa", "Total air-side pressure loss through the FULL evaporator (ALL LAYERS)")
        self.set("total_fan_power", 0.0, "W", "Total power consumed by ALL fans to overcome the pressure drop")
        self.set("total_m_dot_humid", 0.0, "kg/s", "Total humid air mass flow through the FULL evaporator (ALL REGISTERS)")
        self.set("total_v_dot_fan_m3h", 0.0, "m^3/h", "Total volume flow rate through the FULL evaporator (ALL REGISTERS)")

        
class RefrigerantState(VariableContainer):
    """Holds all variables calculated by the refrigerant subprogram."""
    def __init__(self):
        super().__init__()
        self.set("h_out", 350_000.0, "J/kg", "Outlet refrigerant enthalpy")
        self.set("p_out", 0.0, "-", "Outlet refrigerant pressure")
        self.set("T_out", 250.0, "K", "Outlet refrigerant temperature")
        self.set("T_in", 250.0, "K", "Inlet refrigerant temperature")

        self.set("portion_two_phase", 0.0, "-", "Portion of refrigerant in two-phase state")
        self.set("portion_superheated", 0.0, "-", "Portion of refrigerant in superheated state")

        self.set("h_sat_vap", 0.0, "J/kg", "Saturation vapor enthalpy at evaporator pressure")

        self.set("h_conv_two_phase", 100.0, "W/m^2/K", "Two-phase convective heat transfer coefficient")
        self.set("h_conv_superheated", 100.0, "W/m^2/K", "Superheated convective heat transfer coefficient")

        self.set("T_two_phase_in", 0.0, "K", "Inlet temperature of two-phase region")
        self.set("T_superheated_in", 0.0, "K", "Inlet temperature of superheated region")

        self.set("heat_capacity_avg_superheated", 0.0, "J/kg/K", "Average heat capacity in superheated region")

        self.set("quality_avg", 0.0, "-", "Average refrigerant quality")
        

# --- You would do the same for the other two ---
class HeatMassTransferState(VariableContainer):
    """Holds all variables for the HMT subprogram."""
    def __init__(self):
        super().__init__()
        self.set("A_effective", 0.0, "m^2", "Effective heat transfer area")
        self.set("R_downstream", 0.0, "K/W", "Thermal resistance between frost and refrigerant")
        self.set("R_refrigerant", 0.0, "K/W", "Thermal resistance of refrigerant")
        self.set("R_tube", 0.0, "K/W", "Thermal resistance of tube")
        self.set("R_frost", 0.0, "K/W", "Thermal resistance of frost")
        self.set("R_air", 0.0, "K/W", "Thermal resistance of air")
        self.set("T_frost_surface", 268.0, "K", "Frost surface temperature")
        self.set("T_frost_base", 268.0, "K", "Frost Base temperature")
        self.set("Q_dot_total", 0.0, "W", "Total heat transfer rate (sensible + latent)")
        self.set("Q_dot_sens", 0.0, "W", "Sensible heat transfer rate")
        self.set("m_dot_thickening_flux", 0.0, "kg/s/m^2", "Mass flux rate of frost thickening")
        self.set("m_dot_densification_flux", 0.0, "kg/s/m^2", "Mass flux rate of frost densification")
        self.set("m_dot_densification", 0.0, "kg/s", "Mass flow rate of frost densification")
        self.set("m_dot_thickening", 0.0, "kg/s", "Mass flow rate of frost thickening")
        self.set("m_dot_frost_total", 0.0, "kg/s", "Mass flow rate of frost growth")
        self.set("eta_surface", 0.0, "-", "Efficiency of the full frost surface (averaged between pipe and fin surfaces)")

        self.set("area_split_2phase", 1.0, "-", "Portion of effective area in two-phase region")
        self.set("area_split_superheated", 0.0, "-", "Portion of effective area in superheated region")
        
class ThermodynamicsState(VariableContainer):
    """Holds overall thermodynamic properties and wall temperatures."""
    def __init__(self):
        super().__init__()

class FrostEvaporatorState(VariableContainer):
    """
    Acts as the main container for all sub-states of the
    complex frost-evaporator model.
    
    This object is passed to all subprograms, which read from
    and write to their dedicated state objects (e.g., state.air, state.frost).
    """
    def __init__(self):
        super().__init__() 
        
        # --- Compose the state from its sub-states ---
        self.frost = FrostState()
        self.air = AirState()
        self.refrigerant = RefrigerantState()
        self.hmt = HeatMassTransferState()
        self.thermo = ThermodynamicsState()

        # List of sub-states for easier iteration
        self._sub_states = [self.frost, self.air, self.refrigerant, self.hmt, self.thermo]
    
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
    
    def __str__(self):
        """
        Provides a user-friendly string representation (pretty print) 
        in the format: VariableName: Value [Unit].
        """
        all_vars = self.get_all_variables()
        
        # 1. Collect lines for all valid variables
        output_lines = []
        for name, var in all_vars.items():
            value = var.value
            unit = var.unit
            description = var.description
            
            if value is not None:
                # Format the value (round to 3 decimal places for readability)
                # Ensure the value is converted to a string before rounding
                try:
                    formatted_value = f"{value:.8f}"
                except (TypeError, ValueError):
                    formatted_value = str(value) # Fallback for non-numeric types

                # Determine the prefix/group for sorting and clarity
                # e.g., 'frost_T_surf' becomes 'frost' group
                group_name = name.split('_')[0].capitalize()
                
                # Create the final line
                line = f"{name}: {formatted_value} [{unit}]"
                output_lines.append((group_name, line))

        # 2. Sort lines by their group name (e.g., 'Air' then 'Frost')
        output_lines.sort(key=lambda x: x[0])
        
        # 3. Assemble the final output, adding sub-state headings
        final_str = f"--- {self.__class__.__name__} State ---\n"
        current_group = None

        for group, line in output_lines:
            if group != current_group:
                # Add a section header for the sub-state
                final_str += f"\n*** {group} State ***\n"
                current_group = group
            
            final_str += line + "\n"
            
        final_str += "--------------------------------------"
        
        return final_str
    