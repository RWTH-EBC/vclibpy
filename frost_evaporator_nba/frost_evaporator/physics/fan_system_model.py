from ..datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import math
import pandas as pd
import warnings
import math
import CoolProp.CoolProp as CP_HumidAir
from scipy.optimize import brentq

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

        # --- Fan Setup ---
        # Pressure Curve Parameters
        self.q_peak_pressure = 0.0
        self.p_max_pressure = 0.0
        self.fan_curve_poly = None
        
        # Power Curve Parameters
        self.q_peak_power = 0.0
        self.p_max_power = 0.0
        self.fan_power_poly = None 

        self.min_f = 0.0
        self.max_f = 0.0
        
        # Read Fan Data once
        self.raw_fan_df = None 
        if self.params.fan_selection == "OptiAbt":
            self._load_optiabt_raw_data()
        elif self.params.fan_selection == "OptiHorst":
            self.load_optihorst_raw_data()


    def _load_optiabt_raw_data(self):
        """Loads the CSV data once during initialization."""
        file_path = r"D:\mbc_nba\OptiAbt_Daten\Ventilatorkennlinie_Daten_NB.csv"
        try:
            self.raw_fan_df = pd.read_csv(file_path, sep=';', decimal='.', skiprows=[1], encoding='latin1')
        except FileNotFoundError:
            raise FileNotFoundError(f"Could not find fan curve data at: {file_path}")
        except Exception as e:
            raise RuntimeError(f"Error loading OptiAbt fan data: {e}")

    def load_optihorst_raw_data(self):
        """Loads the CSV data once during initialization."""
        file_path = r"D:\mbc_nba\OptiHorst\Technische Informationen\Ventilatorkennlinie_Daten_NB.csv"
        try:
            self.raw_fan_df = pd.read_csv(file_path, sep=';', decimal='.', skiprows=[1], encoding='latin1')
        except FileNotFoundError:
            raise FileNotFoundError(f"Could not find fan curve data at: {file_path}")
        except Exception as e:
            raise RuntimeError(f"Error loading OptiHorst fan data: {e}")


    def _update_fan_curve(self, current_rpm: float):
        """
        Generates the fan curve polynomials for the CURRENT time step's RPM.
        
        Pressure & Power Curve: Use quadratic fit with peak detection for stall (Clamped).
        """
        if self.raw_fan_df is None:
            return

        # Scale data to current RPM using Fan Affinity Laws
        # Flow ~ n, Pressure ~ n^2, Power ~ n^3
        ratio = current_rpm / self.raw_fan_df['n']
        
        q_norm     = self.raw_fan_df['qV']   * (ratio**1)
        p_norm     = self.raw_fan_df['p_fs'] * (ratio**2)
        power_norm = self.raw_fan_df['P']    * (ratio**3)

        # Scale volume flow for specific register/fan count
        q_scaled = q_norm * (self.params.global_fan_amount / self.params.global_register_amount)

        # Scale total power for all fans in the system
        p_power_total_scaled = power_norm * self.params.global_fan_amount

        try:
            # ================= Pressure Fit =================
            coeffs_press = np.polyfit(q_scaled, p_norm, 2)
            self.fan_curve_poly = np.poly1d(coeffs_press)
            
            # Find the Pressure Peak (Stall Point)
            a, b, c = coeffs_press
            if a < 0: # Concave down
                self.q_peak_pressure = -b / (2.0 * a)
                self.p_max_pressure = self.fan_curve_poly(self.q_peak_pressure)
            else:
                self.q_peak_pressure = 0.0
                self.p_max_pressure = c

            # ================= Power Fit =================
            coeffs_power = np.polyfit(q_scaled, p_power_total_scaled, 2)
            self.fan_power_poly = np.poly1d(coeffs_power)
            
            # Find the Power Peak (Stall Point)
            ap, bp, cp = coeffs_power
            if ap < 0: # Concave down
                self.q_peak_power = -bp / (2.0 * ap)
                self.p_max_power = self.fan_power_poly(self.q_peak_power)
            else:
                self.q_peak_power = 0.0
                self.p_max_power = np.max(p_power_total_scaled)

            # Limits
            self.min_f = 0.0 
            self.max_f = q_scaled.max()

        except Exception:
            # Fallback
            self.fan_curve_poly = lambda x: 0.0
            self.fan_power_poly = lambda x: 0.0
            self.q_peak_pressure = 0.0
            self.p_max_pressure = 0.0
            self.q_peak_power = 0.0
            self.p_max_power = 0.0
            self.min_f = 0.0
            self.max_f = 0.0

    def solve_fan_system_equilibrium(self, states: list[FrostEvaporatorState], global_inputs: FrostEvaporatorInputs):
        """
        Solves the Hydraulic Circuit and calculates Power consumption.
        Calculates the GLOBAL Mass Flow Rate that satisfies the pressure balance.
        Then calculates and sets the local Velocity for each layer based on that mass flow
        and the local density/frost blockage.
        
        Args:
            states: List of state objects for every spatial discretization layer.
            global_inputs: Global input parameters (inlet air conditions).
        """
        
        # ================= Update Fan Curve for Current RPM =================
        current_rpm = global_inputs.air.fan_speed_rpm
        self._update_fan_curve(current_rpm)

        # ================= Get Inlet Conditions =================
        T_in = global_inputs.air.T_in
        p_in = global_inputs.air.p_in
        W_in = global_inputs.air.W_in

        # Calculate Air Density at Fan Inlet
        density_inlet = 1.0 / CP_HumidAir.HAPropsSI('V', 'T', T_in, 'P', p_in, 'W', W_in)

        # ================= Solve Mass Flow =================
        max_m_dot = (self.max_f / 3600.0) * density_inlet
        min_m_dot = (self.min_f / 3600.0) * density_inlet
        
        try:
            m_dot_solved = brentq(
                f    = self._calculate_hydraulic_residual, 
                a    = min_m_dot, 
                b    = max_m_dot,  
                args = (states, density_inlet),
                xtol = 1e-6
            )
        except ValueError:
            # The system is choked. Clamp to the minimum valid fan flow.
            m_dot_solved = min_m_dot
            print("[Fan System] Warning: No valid mass flow found within fan curve limits. Clamping to minimum flow.")


        # ================= Update States =================
        v_dot_fan_m3h = (m_dot_solved / density_inlet) * 3600.0
        total_pressure_drop_fan = self._get_pressure_from_flow(v_dot_fan_m3h)

        # Calculate Fan Power (With Safety Clamp)
        # If flow is to the left of the peak (stall region), use the Max Power (Clamped).
        # Otherwise, use the polynomial fit.
        if v_dot_fan_m3h < self.q_peak_power:
             total_power_watts = float(self.p_max_power)
        else:
             total_power_watts = max(0.0, float(self.fan_power_poly(v_dot_fan_m3h)))

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
            state.air.set("v_dot_fan_m3h_segment", v_dot_fan_m3h)

            state.air.set("total_fan_power", total_power_watts)
            state.air.set("total_system_pressure_drop", total_pressure_drop_fan)
            state.air.set("total_m_dot_humid", m_dot_solved * self.params.global_register_amount)
            state.air.set("total_v_dot_fan_m3h", v_dot_fan_m3h * self.params.global_register_amount)

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
        tube_rows = self.params.global_tube_layers
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

        # for name, (val, min_v, max_v) in checks.items():
        #     if not (min_v <= val <= max_v):
        #         warnings.warn(
        #             f"[Wang Correlation Validity] {name} value {val:.2f} is outside "
        #             f"the valid range [{min_v} - {max_v}]. Results may be inaccurate."
        #         )

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
            dp_layer, _= self._calculate_layer_dp(
                state         = state, 
                m_dot         = m_dot_guess,
                density_local = state.air.density_avg
            )
            dp_system_total += dp_layer
            
        return dp_fan - dp_system_total
    
    def _get_pressure_from_flow(self, flow_val: float) -> float:
        """
        Returns Static Pressure for a given Volume Flow using the active fan curve.

        Args:
            flow_val: The volume flow rate [m^3/h].
        Returns:
            The static pressure [Pa].
        """
        # Flat-Top Constraint: If flow is in stall region (left of peak), return Max Pressure
        if flow_val < self.q_peak_pressure:
            return float(self.p_max_pressure)
        
        # Standard curve evaluation
        return float(self.fan_curve_poly(flow_val))
    
    def _calculate_layer_dp(self, state: FrostEvaporatorState, m_dot: float, density_local: float) -> tuple[float, float]:
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

        hydraulic_diameter = 4 * (state.frost.flow_area_air * self.params.fvm_fin_length) / state.frost.A_frost_surface
        Re_Dh = (density_local * v_loc * hydraulic_diameter) / state.air.dyn_viscosity_avg

        # Pressure Drop Calculation 
        if self.params.pressure_drop_correlation_choice == "Wang":
            # Get the clean pressure drop
            dp_clean = self._calculate_system_resistance_wang(
                velocity                = v_loc, 
                density                 = density_local, 
                dyn_viscosity           = state.air.dyn_viscosity_avg, 
                hydraulic_diameter      = hydraulic_diameter,
                collar_diameter_w_frost = state.frost.collar_diameter_w_frost,
                space_between_frost     = state.frost.space_between_frost,
            )
            
            # Get the frost blockage multiplier
            frost_penalty = self._calculate_frost_blockage_scaling(
                space_between_frost = state.frost.space_between_frost
            )
            
            # Apply the scaling
            dp_raw = dp_clean * frost_penalty

        else:
            raise ValueError(f"Unknown pressure drop correlation choice: {self.params.pressure_drop_correlation_choice}")

        # --- Combine ---
        dp_final = dp_raw * self.params.correction_factor_pressure_loss 

        return dp_final, v_loc
    

    def _calculate_system_resistance_wang(self, velocity: float, density: float, dyn_viscosity: float, hydraulic_diameter: float, collar_diameter_w_frost: float, space_between_frost: float) -> float:
        """
        Calculates Pressure Drop using Wang et al. (2000) Friction Factor.
        
        Args:
            velocity: Air velocity [m/s].
            density: Air density [kg/m^3].
            dyn_viscosity: Dynamic viscosity of air [Pa*s].
            hydraulic_diameter: Hydraulic diameter of the flow channel [m].
            collar_diameter_w_frost: Collar Diameter of the tube with frost [m].
        Returns:
            The calculated pressure drop [Pa].
        """
        if velocity <= 1e-5:
            return 0.0
        
        # Map to Wang variables for readability
        Pl = self.params.longitudinal_tube_pitch
        Pt = self.params.transverse_tube_pitch
        Fp = space_between_frost
        Dc = collar_diameter_w_frost
        N = float(self.params.global_tube_layers)
        
        # 1. Reynolds (based on Collar Diameter Dc, NOT Hydraulic Diameter)
        reynolds_dc = (density * velocity * Dc) / dyn_viscosity
        
        # Clamp Re to prevent log errors
        ln_re = math.log(max(reynolds_dc, 10.0))

        # 2. Coefficients (Eq 13, 14, 15 from paper)
        F1 = -0.764 + (0.739 * (Pt / Pl)) + (0.177 * (Fp / Dc)) - (0.00758 / N)
        F2 = -15.689 + (64.021 / ln_re)
        F3 = 1.696 - (15.695 / ln_re)

        # 3. Fanning Friction Factor (Eq 12 from paper)
        f = 0.0267 * (reynolds_dc ** F1) * ((Pt / Pl) ** F2) * ((Fp / Dc) ** F3)

        # 4. Calculate Pressure Drop (Fanning Equation)
        dynamic_pressure = 0.5 * density * (velocity ** 2)
        
        # The factor 4 converts Fanning f to Darcy-Weisbach context
        friction_term = 4.0 * f * (self.params.fvm_fin_length / hydraulic_diameter)
        
        return friction_term * dynamic_pressure


    def _calculate_frost_blockage_scaling(self, space_between_frost: float) -> float:
        """
        Calculates a non-linear empirical blockage penalty multiplier to apply 
        to a clean pressure drop correlation.
        
        Args:
            space_between_frost: Space between frost layers [m].
            
        Returns:
            total_penalty: The pressure drop multiplier (> 1.0 when frosted).
        """
        Fp_clean = self.params.fin_spacing
        frost_thickness = max(0.0, (Fp_clean - space_between_frost) / 2.0)
        
        # Calculate the blocked ratio
        blocking_ratio = (2.0 * frost_thickness) / Fp_clean

        # Cap to prevent the model from diverging at extreme blockage levels
        max_allowed_blockage = 0.8
        blocking_ratio = min(blocking_ratio, max_allowed_blockage)

        # Tuning Parameters
        roughness_C = self.params.correction_factor_roughness_C 
        blocking_n  = self.params.correction_factor_roughness_n
        
        # Calculate penalty multiplier
        total_penalty = 1.0 + roughness_C * (blocking_ratio ** blocking_n)
        
        return total_penalty