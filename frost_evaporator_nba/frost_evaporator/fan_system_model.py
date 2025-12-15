from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import warnings
import math
import CoolProp.CoolProp as CP_HumidAir
from scipy.optimize import brentq
from scipy.interpolate import interp1d

class FanSystemModel:

    def __init__(self, 
                 parameters: FrostEvaporatorParameters, 
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
        """
        self.params = parameters

        # --- Wang parameters validity check ---
        self._check_wang_validity()

        # --- Fan interpolation setup ---
        flow_data = np.array([
            0.0, 24.591265397536393, 39.50727883538634, 46.96528555431131, 50.190369540873455,
            52.40761478163493, 55.02799552071668, 58.0515117581187, 61.679731243001115,
            65.30795072788354, 68.93617021276596, 81.43337066069428, 105.21836506159013,
            118.11870100783874, 127.39081746920492, 136.46136618141097, 145.73348264277715,
            155.20716685330348, 164.88241881298993, 174.55767077267637
        ])
        pressure_data = np.array([
            61.15112994350283, 58.13559322033898, 54.971751412429384, 51.7090395480226, 48.44632768361582,
            45.183615819209045, 42.01977401129944, 38.75706214689266, 35.494350282485875,
            32.33050847457627, 29.06779661016949, 25.805084745762713, 22.641242937853107,
            19.37853107344633, 16.11581920903955, 13.05084745762712, 9.689265536723164,
            6.52542372881356, 3.26271186440678, 0.09887005649717515
        ])
        self.min_f = flow_data[0]
        self.max_f = flow_data[-1]
        self.interpolator = interp1d(flow_data, pressure_data, kind='cubic', fill_value="extrapolate")


    def solve_fan_system_equilibrium(self, states: list[FrostEvaporatorState], global_inputs: FrostEvaporatorInputs):
        """
        Solves the Hydraulic Circuit: Fan Curve vs System Resistance.
        
        Calculates the GLOBAL Mass Flow Rate that satisfies the pressure balance.
        Then calculates and sets the local Velocity for each layer based on that mass flow
        and the local density/frost blockage.
        
        Args:
            states: List of state objects for every spatial discretization layer.
            global_inputs: Global input parameters (inlet air conditions).
        """
        
        # ================= Get Inlet Conditions =================
        T_in = global_inputs.air.T_in
        p_in = global_inputs.air.p_in
        W_in = global_inputs.air.W_in

        # Calculate Air Density at Fan Inlet
        density_inlet = 1.0 / CP_HumidAir.HAPropsSI('V', 'T', T_in, 'P', p_in, 'W', W_in)

        # ================= Solve Mass Flow =================
        max_m_dot = (self.max_f / 3600.0) * density_inlet
        
        try:
            m_dot_solved = brentq(
                f    = self._calculate_hydraulic_residual, 
                a    = 1e-4, 
                b    = max_m_dot,  
                args = (states, density_inlet),
                xtol = 1e-6
            )
        except ValueError:
            # If choked or out of range, default to 0 or min
            m_dot_solved = 0.0
            print("Warning: Fan solver failed to converge. Flow set to 0.")


        # ================= Update States =================
        v_dot_fan_m3h = (m_dot_solved / density_inlet) * 3600.0
        total_pressure_drop_fan = self._get_pressure_from_flow(v_dot_fan_m3h)

        # Initialize running pressure with global inlet pressure
        current_static_pressure = global_inputs.air.p_in
        
        for state in states:
            density_local = state.air.density_avg

            # Re-calculate exact dP and Velocity for this layer using solved mass flow
            dp_layer, v_local = self._calculate_layer_dp(
                state         = state, 
                m_dot         = m_dot_solved, 
                density_local = density_local
            )

            state.air.set("velocity", v_local)
            state.air.set("m_dot_humid", m_dot_solved)
            state.air.set("p_out", current_static_pressure - dp_layer)
            state.air.set("pressure_drop", dp_layer)
            state.air.set("total_system_pressure_drop", total_pressure_drop_fan)

            # Decrement pressure for the next layer (Outlet of n is Inlet of n+1)
            current_static_pressure -= dp_layer


    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _check_wang_validity(self) -> None:
        """
        Validates if the geometry lies within the limits of the Wang et al. (2000) correlation.

        This checks if the plain fin-and-tube exchanger parameters (tube layers,
        diameters, and pitches) are within the experimental ranges defined by the
        correlation. Emits a warning if parameters are out of bounds.

        Ranges Checked:
            - Tube Rows (N): 1 to 6
            - Tube Outer Diameter: 6.35 to 12.7 [mm]
            - Fin Pitch: 1.19 to 8.7 [mm]
            - Transverse Pitch: 17.7 to 31.75 [mm]
            - Longitudinal Pitch: 12.4 to 27.5 [mm]
        """
        # scaling factor: meters to millimeters
        m_to_mm = 1000.0

        # Extract and convert parameters to mm for comparison
        tube_rows = self.params.tube_layers
        tube_outer_diameter_mm = self.params.tube_outer_diameter * m_to_mm
        fin_pitch_mm = self.params.fin_pitch * m_to_mm
        transverse_pitch_mm = self.params.transverse_tube_pitch * m_to_mm
        longitudinal_pitch_mm = self.params.longitudinal_tube_pitch * m_to_mm

        # Define validation limits: (current_value, min_limit, max_limit)
        checks = {
            "Tube Rows (N)": (tube_rows, 1, 6),
            "Tube Outer Diameter (Do)": (tube_outer_diameter_mm, 6.35, 12.7),
            "Fin Pitch (Fp)": (fin_pitch_mm, 1.19, 8.7),
            "Transverse Pitch (Pt)": (transverse_pitch_mm, 17.7, 31.75),
            "Longitudinal Pitch (Pl)": (longitudinal_pitch_mm, 12.4, 27.5),
        }

        for name, (val, min_v, max_v) in checks.items():
            if not (min_v <= val <= max_v):
                warnings.warn(
                    f"[Wang Correlation Validity] {name} value {val:.2f} is outside "
                    f"the valid range [{min_v} - {max_v}]. Results may be inaccurate."
                )

    def _calculate_hydraulic_residual(self, m_dot_guess: float, states: list[FrostEvaporatorState], density_inlet: float) -> float:
        """
        Calculates the residual of the hydraulic circuit equation.
        Residual = Fan_Pressure(V_dot) - Sum(Layer_Pressure_Drops)
        
        Args:
            m_dot_guess: The mass flow rate to test [kg/s].
            states: The list of layer states.
            density_inlet: The air density at the inlet [kg/m^3].
        Returns:
            The pressure difference residual [Pa].
        """
        # A. Calculate Fan Pressure Available
        v_dot_fan_m3h = (m_dot_guess / density_inlet) * 3600.0
        dp_fan = self._get_pressure_from_flow(v_dot_fan_m3h)
        
        # B. Calculate System Pressure Drop (Sum of all layers)
        dp_system_total = 0.0
        
        for state in states:                
            dp_layer, _ = self._calculate_layer_dp(
                state         = state, 
                m_dot         = m_dot_guess,
                density_local = state.air.density_avg
            )
            dp_system_total += dp_layer
            
        return dp_fan - dp_system_total
    
    def _get_pressure_from_flow(self, flow_val: float) -> float:
        """
        Returns Static Pressure for a given Volume Flow via interpolation.

        Args:
            flow_val: The volume flow rate [m^3/h].
        Returns:
            The static pressure [Pa].
        """
        # Check for out of bounds
        if flow_val < self.min_f or flow_val > self.max_f:
            print(
                f"--> WARNING: Flow input {flow_val:.2f} is outside valid range "
                f"({self.min_f:.2f} - {self.max_f:.2f}). Extrapolating..."
            )

        return float(self.interpolator(flow_val))

    def _calculate_layer_dp(self, state: FrostEvaporatorState, m_dot: float, density_local: float) -> tuple[float, float]:
        # sourcery skip: inline-variable, switch
        """
        Calculates pressure drop for a single layer based on the selected correlation.
        
        Args:
            state: The state object for the specific layer.
            m_dot: The mass flow rate through the system [kg/s].
            density_local: The average air density in this layer [kg/m^3].
        Returns:
            - dp_layer: The pressure drop across this layer [Pa].
            - v_loc: The local air velocity [m/s].
        """
        # Calculate Local Velocity
        v_loc = m_dot / (density_local * state.frost.flow_area_air)

        # Pressure Drop Calculation 
        if self.params.pressure_drop_correlation_choice == "Wang":
            hydraulic_diameter = 4 * (state.frost.flow_area_air * self.params.fin_length) / state.frost.A_frost_surface

            dp_raw = self._calculate_system_resistance_wang(
                velocity                  = v_loc, 
                density                   = density_local, 
                dyn_viscosity             = state.air.dyn_viscosity_avg, 
                longitudinal_tube_pitch   = self.params.longitudinal_tube_pitch, 
                transverse_tube_pitch     = self.params.transverse_tube_pitch, 
                fin_pitch                 = self.params.fin_pitch, 
                collar_diameter_w_frost   = self.params.tube_outer_diameter + 2 * self.params.fin_thickness + 2 * state.frost.thickness, 
                tube_layers               = self.params.tube_layers,
                flow_length               = self.params.fin_length,
                hydraulic_diameter        = hydraulic_diameter,
            )
        
        elif self.params.pressure_drop_correlation_choice == "Haaf":
            dp_raw = self._calculate_system_resistance_haaf(
                velocity                  = v_loc, 
                density                   = density_local, 
                dyn_viscosity             = state.air.dyn_viscosity_avg, 
                space_between_frost       = state.frost.space_between_frost, 
                longitudinal_tube_pitch   = self.params.longitudinal_tube_pitch
            )
        
        else:
            raise ValueError(f"Unknown pressure drop correlation choice: {self.params.pressure_drop_correlation_choice}")

        return self.params.correction_factor_pressure_loss * dp_raw, v_loc

    def _calculate_system_resistance_wang(self, velocity: float, density: float, dyn_viscosity: float, longitudinal_tube_pitch: float, 
                                          transverse_tube_pitch: float, fin_pitch: float, collar_diameter_w_frost: float, tube_layers: int,
                                          flow_length: float, hydraulic_diameter: float) -> float:
        """
        Calculates Pressure Drop using Wang et al. (2000) Friction Factor.
        
        Args:
            velocity: Air velocity [m/s].
            density: Air density [kg/m^3].
            dyn_viscosity: Dynamic viscosity of air [Pa*s].
            longitudinal_tube_pitch: Distance between tubes in flow direction (Pl) [m].
            transverse_tube_pitch: Distance between tubes perpendicular to flow (Pt) [m].
            fin_pitch: Distance between fins (Fp) [m].
            collar_diameter_w_frost: Effective collar diameter (Dc) [m].
            tube_layers: Number of tube rows (N) [-].
            flow_length: Length of the flow path [m].
            hydraulic_diameter: Hydraulic diameter of the flow channel [m].
        Returns:
            The calculated pressure drop [Pa].
        """
        if velocity <= 1e-5:
            return 0.0

        # 1. Reynolds (based on Collar Diameter Dc, NOT Hydraulic Diameter)
        reynolds_dc = (density * velocity * collar_diameter_w_frost) / dyn_viscosity
        
        # Clamp Re to prevent log errors
        reynolds_dc = max(reynolds_dc, 10.0)
        ln_re = math.log(reynolds_dc)
        
        # Map to Wang variables for readability
        Pl = longitudinal_tube_pitch
        Pt = transverse_tube_pitch
        Fp = fin_pitch
        Dc = collar_diameter_w_frost
        N = float(tube_layers)

        # 2. Coefficients (Eq 13, 14, 15 from paper)
        F1 = -0.764 + (0.739 * (Pt / Pl)) + (0.177 * (Fp / Dc)) - (0.00758 / N)
        F2 = -15.689 + (64.021 / ln_re)
        F3 = 1.696 - (15.695 / ln_re)

        # 3. Fanning Friction Factor (Eq 12 from paper)
        f = 0.0267 * (reynolds_dc ** F1) * ((Pt / Pl) ** F2) * ((Fp / Dc) ** F3)

        # 4. Calculate Pressure Drop (Fanning Equation)
        dynamic_pressure = 0.5 * density * (velocity ** 2)
        
        # The factor 4 converts Fanning f to Darcy-Weisbach context
        friction_term = 4.0 * f * (flow_length / hydraulic_diameter)
        
        return friction_term * dynamic_pressure
    
    def _calculate_system_resistance_haaf(self, velocity: float, density: float, dyn_viscosity: float, space_between_frost: float, longitudinal_tube_pitch: float) -> float:
        """
        Calculates the system pressure drop based on the Haaf correlation.
        
        Args:
            velocity: Air velocity [m/s].
            density: Air density [kg/m^3].
            dyn_viscosity: Dynamic viscosity [Pa·s].
            space_between_frost: Effective space between frost layers [m].
            longitudinal_tube_pitch: Space between pipes in flow direction [m].
        Returns:
            Pressure drop of the system [Pa].
        """
        if velocity <= 0:
            return 0.0

        reynolds = (density * velocity * (2 * space_between_frost)) / dyn_viscosity

        zeta = self._calculate_pressure_loss_coefficient_haaf(
            reynolds=reynolds,
            space_between_frost=space_between_frost,
            longitudinal_tube_pitch=longitudinal_tube_pitch
        )

        # Bernoulli / Darcy-Weisbach formulation
        return zeta * 0.5 * density * (velocity ** 2)

    def _calculate_pressure_loss_coefficient_haaf(self, reynolds: float, space_between_frost: float, longitudinal_tube_pitch: float) -> float:
        """
        Calculates the pressure loss coefficient (zeta) based on the Haaf correlation.

        Args:
            reynolds: The Reynolds number [-].
            space_between_frost: The effective space between frost layers [m].
            longitudinal_tube_pitch: The distance two tubes in air flow direction [m].
        Returns:
            The pressure loss coefficient (zeta) [-].
        """
        length_ratio = space_between_frost / longitudinal_tube_pitch

        return 10.5 * (reynolds ** (-1.0 / 3.0)) * (length_ratio ** 0.6)