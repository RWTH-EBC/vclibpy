from ..datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import pandas as pd
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

        # --- Fan Setup ---
        self.q_peak = 0.0
        self.p_max = 0.0
        self.min_f = 0.0
        self.max_f = 0.0
        self.fan_curve_poly = None
        
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
        Generates the fan curve polynomial for the CURRENT time step's RPM.
        Uses a standard quadratic fit (ax^2 + bx + c) but identifies the 
        peak/stall point to prevent pressure drop-off at low flows.
        """
        if self.raw_fan_df is None:
            return

        # Scale data to current RPM using Fan Affinity Laws
        ratio = current_rpm / self.raw_fan_df['n']
        q_norm = self.raw_fan_df['qV'] * ratio
        p_norm = self.raw_fan_df['p_fs'] * (ratio**2)

        # Scale volume flow for specific register/fan count
        q_scaled = q_norm * (self.params.fan_amount / self.params.register_amount)

        try:
            # Standard Quadratic Fit: P = ax^2 + bx + c
            coeffs = np.polyfit(q_scaled, p_norm, 2)
            self.fan_curve_poly = np.poly1d(coeffs)
            
            # Find the Peak (Stall Point)
            a, b, c = coeffs
            # Only calculate peak if curve is concave down (a < 0)
            if a < 0:
                # Vertex of parabola: x = -b / (2a)
                self.q_peak = -b / (2.0 * a)
                self.p_max = self.fan_curve_poly(self.q_peak)
            else:
                self.q_peak = 0.0
                self.p_max = c

            self.min_f = 0.0 
            self.max_f = q_scaled.max()
            
        except Exception:
            # Fallback
            self.fan_curve_poly = lambda x: 0.0
            self.q_peak = 0.0
            self.p_max = 0.0
            self.min_f = 0.0
            self.max_f = 0.0

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
            # If choked or out of range, default to 0 or min
            m_dot_solved = 0.0


        # ================= Update States =================
        v_dot_fan_m3h = (m_dot_solved / density_inlet) * 3600.0
        total_pressure_drop_fan = self._get_pressure_from_flow(v_dot_fan_m3h)

        # Initialize running pressure with global inlet pressure
        current_static_pressure = global_inputs.air.p_in
        
        for state in states:
            density_local = state.air.density_avg

            # Re-calculate exact dP and Velocity for this layer using solved mass flow
            dp_layer, v_local, roughness_multiplier = self._calculate_layer_dp(
                state         = state, 
                m_dot         = m_dot_solved, 
                density_local = density_local
            )

            state.air.set("velocity", v_local)
            state.air.set("m_dot_humid", m_dot_solved)
            state.air.set("p_out", current_static_pressure - dp_layer)
            state.air.set("pressure_drop", dp_layer)
            state.air.set("total_system_pressure_drop", total_pressure_drop_fan)
            state.air.set("v_dot_fan_m3h_segment", v_dot_fan_m3h)
            state.air.set("roughness_multiplier", roughness_multiplier)

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
            dp_layer, _, _ = self._calculate_layer_dp(
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
        if flow_val < self.q_peak:
            return float(self.p_max)
        
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

        hydraulic_diameter = 4 * (state.frost.flow_area_air * self.params.fin_length) / state.frost.A_frost_surface
        Re_Dh = (density_local * v_loc * hydraulic_diameter) / state.air.dyn_viscosity_avg

        # Pressure Drop Calculation 
        if self.params.pressure_drop_correlation_choice == "Wang":
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
                hydraulic_diameter        = hydraulic_diameter, 
                longitudinal_tube_pitch   = self.params.longitudinal_tube_pitch,
                Re_Dh                     = Re_Dh
            )
        
        else:
            raise ValueError(f"Unknown pressure drop correlation choice: {self.params.pressure_drop_correlation_choice}")
        
        # --- Roughness Multiplier ---
        frost_sand_grain_roughness = self._calculate_frost_sand_grain_roughness(
            thickness = state.frost.thickness,
            absolute_humidity = state.air.W_avg,
            Re_Dh = Re_Dh,
            T_frost_surface = state.hmt.T_frost_surface,
            T_frost_base = state.hmt.T_frost_base,
            T_air = state.air.T_avg,
        )

        f_rough  = self._calculate_friction_from_roughness(
            Re_Dh = Re_Dh,
            Dh = hydraulic_diameter,
            ks = frost_sand_grain_roughness,
        )

        f_smooth = self._calculate_friction_from_roughness(
            Re_Dh = Re_Dh,
            Dh = hydraulic_diameter,
            ks = 1e-9,  # Smooth surface approximation
        )

        # avoid division by zero
        if f_smooth <= 1e-12:
            roughness_multiplier = 1.0
        else:
            roughness_multiplier = f_rough / f_smooth

        # --- Combine ---
        dp_final = dp_raw * roughness_multiplier * self.params.correction_factor_pressure_loss

        return dp_final, v_loc, roughness_multiplier


    def _calculate_frost_sand_grain_roughness(self, thickness: float, absolute_humidity: float, Re_Dh: float, T_frost_surface: float, T_frost_base: float, T_air: float) -> float:
        """
        Calculates equivalent sand-grain roughness using Zhang et al. (2021). Eq. 13
        DOI: 10.2514/1.C036066

        Args:
            thickness: Frost thickness [m].
            absolute_humidity: Air humidity ratio [kg/kg].
            Re_Dh: Reynolds number based on hydraulic diameter [-].
            T_frost_surface: Frost surface temperature [K].
            T_frost_base: Wall/Base temperature [K].
            T_air: Bulk air temperature [K].
        Returns:
            The calculated equivalent sand-grain roughness [m].
        """
        import numpy as np

        # Physical Constant: Triple point of water [K]
        T_o = 273.15

        # Safety checks for negligible frost or invalid temp gradients
        delta_T_total = T_air - T_frost_base
        if thickness <= 1e-9 or delta_T_total <= 1e-5:
            return 0.0

        # Temperature terms
        raw_ratio = (T_air - T_frost_surface) / delta_T_total
        temp_ratio = max(0.0, min(1.0, raw_ratio))

        delta_T_frost = max(0.0, T_frost_surface - T_frost_base)

        # Calculate dimensionless roughness (ks / hf) using Zhang et al. Eq 13
        ks_dimensionless = (
            0.029 
            * (absolute_humidity**0.367) 
            * (Re_Dh**0.150) 
            * ((delta_T_frost / T_o)**0.342) 
            * np.exp(6.716 * temp_ratio)
        )

        return thickness * ks_dimensionless
        

    def _calculate_friction_from_roughness(self, Re_Dh: float, Dh: float, ks: float) -> float:
        """
        Calculates the Darcy friction factor using the Haaland Equation (1983).
        https://doi.org/10.1115/1.3240948

        Args:
            Re_Dh: Reynolds number based on hydraulic diameter [-].
            Dh: Hydraulic diameter [m].
            ks: Equivalent sand-grain roughness [m].

        Returns:
            The Darcy friction factor [-].

        Calculates the Darcy friction factor with a smooth transition between 
        Laminar and Turbulent flow to prevent numerical spikes.
        
        - Laminar (Re < 1000): f = 64/Re
        - Transition (1000 <= Re <= 4000): Linear interpolation
        - Turbulent (Re > 4000): Haaland Equation
        """
        
        # --- Calculate Laminar Candidate ---
        # Prevent division by zero
        if Re_Dh <= 1e-5:
            return 0.0
        f_laminar = 64.0 / Re_Dh

        # --- Calculate Turbulent Candidate (Haaland) ---
        # We calculate this even if Re is low, to use for interpolation
        if Dh <= 1e-9:
             f_turbulent = 0.0
        else:
             rel_roughness = ks / Dh
             
             # Haaland Equation terms
             term_roughness = (rel_roughness / 3.7)**1.11
             term_reynolds = 6.9 / Re_Dh
             
             inv_sqrt_f = -1.8 * math.log10(term_roughness + term_reynolds)
             f_turbulent = (1.0 / inv_sqrt_f)**2

        # --- Determine Regime & Interpolate ---
        Re_laminar_limit = 300.0
        Re_turbulent_limit = 2000.0
        
        # Pure Laminar (Roughness has NO effect here in standard theory)
        if Re_Dh < Re_laminar_limit:
            return f_laminar
            
        # Pure Turbulent (Roughness has FULL effect here)
        elif Re_Dh > Re_turbulent_limit:
            return f_turbulent
            
        # Transition Zone: Linearly blend between Laminar and Turbulent
        else:
            # Alpha goes from 0.0 to 1.0
            alpha = (Re_Dh - Re_laminar_limit) / (Re_turbulent_limit - Re_laminar_limit)
            
            return (1.0 - alpha) * f_laminar + alpha * f_turbulent



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

    
    def _calculate_system_resistance_haaf(self, velocity: float, density: float, dyn_viscosity: float, hydraulic_diameter: float, longitudinal_tube_pitch: float, Re_Dh: float) -> float:
        """
        Calculates the system pressure drop based on the Haaf correlation.
        
        Args:
            velocity: Air velocity [m/s].
            density: Air density [kg/m^3].
            dyn_viscosity: Dynamic viscosity [Pa·s].
            hydraulic_diameter: Hydraulic diameter of the flow channel [m].
            longitudinal_tube_pitch: Space between pipes in flow direction [m].
            Re_Dh: Reynolds number based on hydraulic diameter [-].
        Returns:
            Pressure drop of the system [Pa].
        """
        zeta = self._calculate_pressure_loss_coefficient_haaf(
            reynolds=Re_Dh,
            hydraulic_diameter=hydraulic_diameter,
            longitudinal_tube_pitch=longitudinal_tube_pitch
        )

        # Bernoulli / Darcy-Weisbach formulation
        return zeta * 0.5 * density * (velocity ** 2)

    def _calculate_pressure_loss_coefficient_haaf(self, reynolds: float, hydraulic_diameter: float, longitudinal_tube_pitch: float) -> float:
        """
        Calculates the pressure loss coefficient (zeta) based on the Haaf correlation.

        Args:
            reynolds: The Reynolds number [-].
            hydraulic_diameter: Hydraulic diameter of the flow channel [m].
            longitudinal_tube_pitch: The distance two tubes in air flow direction [m].
        Returns:
            The pressure loss coefficient (zeta) [-].
        """
        length_ratio = hydraulic_diameter / longitudinal_tube_pitch

        return 10.5 * (reynolds ** (-1.0 / 3.0)) * (length_ratio ** 0.6)