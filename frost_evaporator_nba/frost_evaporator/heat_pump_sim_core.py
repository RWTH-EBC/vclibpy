import numpy as np
from scipy.optimize import brentq, newton
from tqdm import tqdm

from vclibpy import FlowsheetState, Inputs, RelativeCompressorSpeedControl, HeatExchangerInputs
from vclibpy.media import RefProp
from vclibpy.components.compressors import ConstantEffectivenessCompressor
from vclibpy.components.heat_exchangers import MovingBoundaryNTUCondenser
from vclibpy.components.heat_exchangers.heat_transfer.constant import ConstantHeatTransfer, ConstantTwoPhaseHeatTransfer
from vclibpy.components.heat_exchangers.heat_transfer.wall import WallTransfer

# Import your existing Frost Evaporator Simulation
from frost_evaporator import FrostEvaporatorSimulation


###################################################################################
# Heat Pump Simulation Engine Class
###################################################################################

class HeatPumpSimulation:
    def __init__(self, 
                 evap_config_path: str, 
                 external_refprop: RefProp, 
                 time_step_size: int, 
                 simulation_duration: int,
                 t_air_in: float,
                 rh_in: float,
                 t_water_in: float,
                 q_cond_target: float,
                 m_flow_water: float,
                 fixed_compressor_speed: float = None
                 ):

        """
        Initializes the entire Heat Pump System, bringing together the high-side 
        hardware (Compressor, Condenser) and the low-side Frost Evaporator.
        """



        # =============================================================================================================================
        # Cycle Targets 
        self.target_subcooling = 3.0 # Jonas Klingebiel. Bedarfsgesteuerte Abtauung von Luft-Wärmepumpen durch Reinforcement Learning.
        self.target_superheat  = 8.0 # Jonas Klingebiel. Bedarfsgesteuerte Abtauung von Luft-Wärmepumpen durch Reinforcement Learning.

        # Defrost Trigger
        self.min_cop_drop_ratio = 0.935  # Trigger defrost at 93.5% of initial COP - https://doi.org/10.1016/j.energy.2021.120542
        # =============================================================================================================================
        
        # --- Time Configuration ---
        self.time_step_size = time_step_size
        self.simulation_duration = simulation_duration
        self.time_steps = np.arange(0, self.simulation_duration, self.time_step_size)

        # --- Boundary Conditions ---
        self.t_air_in      = t_air_in
        self.rh_in         = rh_in
        self.t_water_in    = t_water_in
        self.q_cond_target = q_cond_target
        self.m_flow_water  = m_flow_water
        self.fixed_compressor_speed = fixed_compressor_speed # Only used for preliminary study
        
        # --- Media Properties ---
        # We pass REFPROP in from the outside so you only have to configure the DLL path once in main.py
        self.med_prop = external_refprop

        self.fs_state = FlowsheetState()

        # --- Instantiate Sub-Models (Hardware) ---
        # Evaporator - Defined by .yaml File
        self.evap_sim = FrostEvaporatorSimulation(evap_config_path, external_refprop=self.med_prop)

        # --- OPTIHORST ---
        if evap_config_path == "config_OptiHorst.yaml":

            # Set Individual Fan RPM for OptiHorst configuration
            self.fan_rpm = 900

            # Bounds for R410A
            self.p_cond_min = 10e5   # 10 Bar
            self.p_cond_max = 48e5   # 48 Bar (Safely under 49 Bar Critical)
            self.p_cond_safe = 18e5  # Safe fallback
            self.p_evap_min = 1e5  # 1.0 Bar
            self.p_evap_max = 16e5   # 16 Bar
            self.p_evap_guess = 6e5  # Starting guess

            # Evaporator Priming (R410A)
            self.priming_p_ref = 6e5
            self.priming_m_dot = 0.02
            self.h2_penalty_limit = 600_000.0
            h_liq = self.med_prop.calc_state("PQ", self.priming_p_ref, 0.0).h
            h_vap = self.med_prop.calc_state("PQ", self.priming_p_ref, 1.0).h
            self.priming_h_ref = (h_liq + h_vap) / 2.0

            # Data from "https://areacooling.com/wp-content/uploads/product-documentation/C-SDP205H02B%20R410A%20Specification.pdf"
            # -> Scroll Compressor
            self.compressor = ConstantEffectivenessCompressor(
                N_max=90,             # Datasheet: 90 Hz
                V_h=42.3e-6,          # Datasheet: 42.3 cm³
                eta_isentropic=0.68,  # Modeled and Measured Effects of Compressor Downsizing in an Existing Air Conditioner/Heat Pump in the Cooling Mode - William P. Levins
                eta_mech=0.9,         # https://doi.org/10.3390/en17225729
                lambda_h=0.95         # Modeled and Measured Effects of Compressor Downsizing in an Existing Air Conditioner/Heat Pump in the Cooling Mode - William P. Levins
            )
            self.compressor.med_prop = self.med_prop
            self.comp_min_rel_speed = 0.3333 # Datasheet: min Speed is 30 Hz

            # Condenser
            self.condenser = MovingBoundaryNTUCondenser(
                A=3.53,                                                            # Optimal Defrost Initiation for Air-Source Heat Pumps: Evaluating the
                                                                                   # Improvement Potential of Common Defrosting Controllers - Klingebiel
                secondary_medium="Water",                                          # Standard medium
                flow_type="counter",                                               # Typical for Plate Heat Exchangers
                ratio_outer_to_inner_area=1.0,                                     # Symmetric plates
                two_phase_heat_transfer=ConstantTwoPhaseHeatTransfer(alpha=4000),  # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
                gas_heat_transfer=ConstantHeatTransfer(alpha=250),                 # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
                liquid_heat_transfer=ConstantHeatTransfer(alpha=1000),             # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
                wall_heat_transfer=WallTransfer(lambda_=16, thickness=0.0004),     # Wärme- und Stoffübertragung            - Baehr   - Tabelle 1.1
                secondary_heat_transfer=ConstantHeatTransfer(alpha=4000)           # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
            )

        # --- OPTIABT CASE ---
        elif evap_config_path == "config_OptiAbt.yaml":

            # Set Individual Fan RPM for OptiHorst configuration
            self.fan_rpm = 1300

            # Bounds for R134a
            self.p_cond_min = 5.0e5  # 5.0 Bar
            self.p_cond_max = 38e5   # 38 Bar (Safely under 40.6 Bar Critical)
            self.p_cond_safe = 10e5  # Safe fallback
            self.p_evap_min = 1.0e5  # 1.0 Bar (R134a evaporates much lower)
            self.p_evap_max = 8.0e5  # 8.0 Bar
            self.p_evap_guess = 2e5  # Starting guess

            # Evaporator Priming (R134a)
            self.priming_p_ref = 2e5        # Start lower!
            self.priming_m_dot = 0.02
            self.h2_penalty_limit = 500_000.0  # Lower penalty threshold for R134a
            h_liq = self.med_prop.calc_state("PQ", self.priming_p_ref, 0.0).h
            h_vap = self.med_prop.calc_state("PQ", self.priming_p_ref, 1.0).h
            self.priming_h_ref = (h_liq + h_vap) / 2.0

            # Data from file: "N:\Forschung\EBC1078_BMWK_Avoid_KAP\Students\Project-Exchange\Experimental_Data\OptiAbt\Technische Informationen\Verflüssigungssatz\Kompressor\96170_HG34e_D.pdf"
            # Model: SHG34e/380-4SL
            # Recipricating Compressor (Piston
            self.compressor = ConstantEffectivenessCompressor(
                N_max=2030.0 / 60.0,  # Datasheet: 1450 RPM (for 50Hz) - Frequenzumrichter: 25-70Hz - (1450.0 / 50.0) * 70.0 = 2030 RPM max
                V_h=380.0e-6,         # Datasheet: 380 cm^3
                eta_isentropic=0.70,  # Modeled and Measured Effects of Compressor Downsizing in an Existing Air Conditioner/Heat Pump in the Cooling Mode - William P. Levins
                eta_mech=0.9,         # https://doi.org/10.3390/en17225729
                lambda_h=0.77         # Modeled and Measured Effects of Compressor Downsizing in an Existing Air Conditioner/Heat Pump in the Cooling Mode - William P. Levins
            )
            self.compressor.med_prop = self.med_prop
            self.comp_min_rel_speed = 25.0 / 70.0  # Minimum speed based on 25 Hz limit from datasheet

            # Condenser
            self.condenser = MovingBoundaryNTUCondenser(
                A=3.53,                                                            # Copied Size of OptiHorst, because Heat Output is very similar 
                secondary_medium="Water",                                          # Standard medium
                flow_type="counter",                                               # Typical for Plate Heat Exchangers
                ratio_outer_to_inner_area=1.0,                                     # Symmetric plates
                two_phase_heat_transfer=ConstantTwoPhaseHeatTransfer(alpha=4000),  # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
                gas_heat_transfer=ConstantHeatTransfer(alpha=250),                 # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
                liquid_heat_transfer=ConstantHeatTransfer(alpha=1000),             # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
                wall_heat_transfer=WallTransfer(lambda_=16, thickness=0.0004),     # Wärme- und Stoffübertragung            - Baehr   - Tabelle 1.1
                secondary_heat_transfer=ConstantHeatTransfer(alpha=4000)           # Fundamentals of Heat and Mass Transfer - Bergman - Table 1.1
            )

        else:
            raise ValueError(f"Unsupported evaporator configuration: {evap_config_path}")
        
        
        self.condenser.med_prop = self.med_prop
        
        # Initialize Secondary Side (Water)
        self.condenser.start_secondary_med_prop()

    def update_and_prime_evaporator(self, fin_pitch: float, fin_amount: int):
        self.evap_sim.update_fin_pitch_and_amount(new_pitch=fin_pitch, new_amount=fin_amount)

        # B. Prime Evaporator Model
        init_inputs = self.evap_sim._create_inputs(
            T_air_C=self.t_air_in, 
            RH_pct=self.rh_in, 
            fan_rpm=self.fan_rpm,
            h_ref=self.priming_h_ref, 
            p_ref_Pa=self.priming_p_ref, 
            m_dot_ref=self.priming_m_dot
        )
        
        self.evap_sim.prime_model(init_inputs)
    

    # =============================================================================
    # COMPONENT HELPERS & RESIDUAL FUNCTIONS
    # =============================================================================

    def _calc_compressor_step(self, n_speed, p_discharge):
        """Runs compressor model. Returns: m_dot and h2"""
        ctrl = RelativeCompressorSpeedControl(n=n_speed)
        inputs = Inputs(control=ctrl)
        
        m_dot = self.compressor.calc_m_flow(inputs=inputs, fs_state=self.fs_state)
        self.compressor.calc_state_outlet(p_outlet=p_discharge, inputs=inputs, fs_state=self.fs_state)
        
        return m_dot, self.compressor.state_outlet.h

    def _calc_condenser_step(self, m_dot, h_inlet, p_pressure, water_inputs):
        """Runs condenser hardware model. Returns: h3, balance_error"""
        # Set Condenser Inlet State
        self.condenser.m_flow = m_dot
        self.condenser.state_inlet = self.med_prop.calc_state("PH", p_pressure, h_inlet)
        
        # Set Target Outcome (Subcooled Liquid)
        t_sat = self.med_prop.calc_state("PQ", p_pressure, 0).T
        self.condenser.state_outlet = self.med_prop.calc_state("PT", p_pressure, t_sat - self.target_subcooling)
        
        # Calculate Hardware Performance
        error, _ = self.condenser.calc(inputs=water_inputs, fs_state=self.fs_state)
        return self.condenser.state_outlet.h, error

    def _compression_residual(self, n_guess, p_cond_current, h3_target, target_q_cond):
        # Run Compressor Physics
        m_dot, h2 = self._calc_compressor_step(n_guess, p_cond_current)
        
        # Calculate theoretical thermodynamic heat capacity
        q_calc = m_dot * (h2 - h3_target)
        error = q_calc - target_q_cond

        if h2 > self.h2_penalty_limit: 
            # Preserve the gradient by adding the penalty to the actual error
            penalty = (h2 - self.h2_penalty_limit) * 10.0
            error += penalty
            
        return error

    # ---------------------------------------------------------------------------------------------------------------------
    # LEVEL 1 SOLVER: The "Compression Loop"
    # ---------------------------------------------------------------------------------------------------------------------
    # OBJECTIVE: Find Compressor Speed (n) to meet Heating Demand.
    # LOGIC:
    #   1. GUESS Speed (n).
    #   2. PHYSICS: Calculate mass flow (m_dot) and discharge enthalpy (h2).
    #   3. CHECK: Does Q_generated (m_dot * delta_h) match Target (TARGET_Q_COND)?
    #   4. RESIDUAL: Q_generated - Target. (Adjust 'n' to fix).
    # ---------------------------------------------------------------------------------------------------------------------

    def _run_compression_loop(self, p_cond_current, target_q_cond, solver_cache):

        # --- ADDED WORKAROUND FOR PRELIMINARY STUDY ---
        if self.fixed_compressor_speed is not None:
            solver_cache["last_speed"] = self.fixed_compressor_speed
            return self.fixed_compressor_speed
        # --------------------------------------

        # Calculate Target Enthalpy for Point 3 (Subcooled) for the current p_cond guess
        t_sat = self.med_prop.calc_state("PQ", p_cond_current, 0).T
        h3_target = self.med_prop.calc_state("PT", p_cond_current, t_sat - self.target_subcooling).h

        # Setup the residual function with the extra parameters
        residual_func = lambda n_guess: self._compression_residual(n_guess, p_cond_current, h3_target, target_q_cond)
        
        # Before solving, check if Max or Min Speed is even enough.
        r_min = residual_func(self.comp_min_rel_speed)
        r_max = residual_func(1.0)
        
        if r_min * r_max > 0:
            if r_min > 0 and r_max > 0:
                # Even at min speed, Q_calc > Target. 
                # P_cond is likely wrong. Clamp to min speed so L2 sees the error.
                solver_cache["last_speed"] = self.comp_min_rel_speed
                return self.comp_min_rel_speed
            elif r_min < 0 and r_max < 0:
                # Even at max speed, Q_calc < Target.
                # Clamp to max speed.
                solver_cache["last_speed"] = 1.0
                return 1.0

        # Solve for Compressor Speed
        try:
            n_solved = brentq(f=residual_func, a=self.comp_min_rel_speed, b=1.0, xtol=0.005)
            solver_cache["last_speed"] = n_solved
            return n_solved
        except ValueError:
            # --- DEBUGGING ---
            print(f"\n[L1 FAIL] Speed Loop. Target Q: {target_q_cond}")
            print(f"  At Min Speed ({self.comp_min_rel_speed:.2f}): Q_error = {r_min:.1f} (If positive, min speed is too high)")
            print(f"  At Max Speed (1.0): Q_error = {r_max:.1f} (If negative, compressor is too small)")
            # --- DEBUGGING INSERT END ---
            
            # Prevent the "silent freeze" by falling back to the best physical limit
            best_speed = self.comp_min_rel_speed if abs(r_min) < abs(r_max) else 1.0
            solver_cache["last_speed"] = best_speed
            return best_speed

    # ---------------------------------------------------------------------------------------------------------------------
    # LEVEL 2 SOLVER: The "Condensation Loop"
    # ---------------------------------------------------------------------------------------------------------------------
    # OBJECTIVE: Find Condenser Pressure (p_cond) to enable heat rejection.
    # LOGIC:
    #   1. GUESS Pressure (p_cond).
    #   2. DEMAND: Calculate heat (Q_req) needed to cool from Inlet -> Subcooled.
    #   3. CAPABILITY: Ask Hardware (UA, LMTD) how much heat (Q_hard) it 
    #      CAN actually transfer at this pressure/temp difference (with its area etc.).
    #   4. RESIDUAL: Q_hard - Q_req. (Adjust 'p_cond' to fix).
    # ---------------------------------------------------------------------------------------------------------------------

    def _condensation_residual(self, p_cond_guess, target_q_cond, water_inputs, solver_cache):
        solver_cache["calls"] += 1
        
        # Run Level 1 (Compression Loop) to get speed for the current p_cond guess
        n_speed = self._run_compression_loop(p_cond_guess, target_q_cond, solver_cache)
        
        # Run the Compressor Physics with that speed
        m_dot, h2 = self._calc_compressor_step(n_speed, p_cond_guess)
        
        # Check if Hardware matches Thermodynamics
        _, error = self._calc_condenser_step(m_dot, h2, p_cond_guess, water_inputs)
        
        return error

    # =============================================================================
    # SOLVER LOGIC (High Side)
    # =============================================================================

    def solve_high_side(self, p_evap, h1_target, target_q_cond, water_inputs):
        """
        Solves the High Pressure side (Compressor + Condenser).
        
        This function contains the LEVEL 1 (Compression) and LEVEL 2 (Condensation) loops.
        
        Args:
            p_evap (float): Evaporation Pressure [Pa]
            h1_target (float): Target Suction Enthalpy [J/kg]
        
        Returns:
            Tuple: (p_cond, m_flow, h2, h3, h4, speed, iterations)
        """
        
        # Set Compressor Inlet State (Point 1)
        self.compressor.state_inlet = self.med_prop.calc_state("PH", p_evap, h1_target)

        # Variable to store the last calculated speed to speed up next guess
        solver_cache = {"last_speed": 0.6, "calls": 0}

        # Setup the Level 2 residual function wrapper
        level_2_residual = lambda p_cond_guess: self._condensation_residual(p_cond_guess, target_q_cond, water_inputs, solver_cache)

        # Execute Level 2 Solver
        try:
            p_cond_final = brentq(f=level_2_residual, a=self.p_cond_min, b=self.p_cond_max, xtol=500)
        except ValueError:
            # --- DEBUGGING INSERT START ---
            r_a = level_2_residual(self.p_cond_min)
            r_b = level_2_residual(self.p_cond_max)
            print(f"\n[L2 FAIL] Condenser Pressure Loop.")
            print(f"  At {self.p_cond_min/1e5:.1f} bar: Error = {r_a:.1f} (If positive, pressure needs to be LOWER)")
            print(f"  At {self.p_cond_max/1e5:.1f} bar: Error = {r_b:.1f} (If negative, pressure needs to be HIGHER)")
            # --- DEBUGGING INSERT END ---
            p_cond_final = self.p_cond_safe  # Dynamic safe fallback

        # ---------------------------------------------------------------------------------------------------------------------
        # FINALIZE (Re-run physics to get final points)
        # ---------------------------------------------------------------------------------------------------------------------
        final_speed = self._run_compression_loop(p_cond_final, target_q_cond, solver_cache)
        
        # Final Compressor Run
        m_flow_final, h2_final = self._calc_compressor_step(final_speed, p_cond_final)
        
        # Final Condenser Run
        h3_final, _ = self._calc_condenser_step(m_flow_final, h2_final, p_cond_final, water_inputs)
        
        # Expansion (Isenthalpic)
        h4_final = h3_final

        return p_cond_final, m_flow_final, h2_final, h3_final, h4_final, final_speed, solver_cache["calls"]


    # ---------------------------------------------------------------------------------------------------------------------
    # LEVEL 3 SOLVER: The "Evaporation Loop"
    # ---------------------------------------------------------------------------------------------------------------------
    # OBJECTIVE: Find Evaporator Pressure (P_evap) to match the heat source and superheat target.
    # LOGIC:
    #   1. GUESS Pressure (P_evap).
    #   2. TARGET: Define required evaporator outlet state (Sat + 5K Superheat).
    #   3. SYSTEM: Solve High Side to get mass flow (m_dot) and inlet (h4) to meet (TARGET_Q_COND).
    #   4. PHYSICS: Run Evaporator model to see actual outlet (h1_actual).
    #   5. RESIDUAL: h1_actual - h1_target. (Adjust P to balance).
    # ---------------------------------------------------------------------------------------------------------------------

    def _evaporation_residual(self, p_evap_trial, step_counters, current_water_inputs, pbar, initial_cop, last_cop):
        step_counters[0] += 1 # Count Level 3 call
        
        # --- Gurad for unphysical Pressures ---
        if p_evap_trial < self.p_evap_min:
            # Return a soft penalty to guide solver back up
            return 1e5 * (self.p_evap_min - p_evap_trial)
            
        if p_evap_trial > self.p_evap_max:
            return 1e5 * (p_evap_trial - self.p_evap_max)
        # ------------------------------

        try:
            # Calculate Target h1
            t_sat_ev = self.med_prop.calc_state("PQ", p_evap_trial, 0).T
            h1_target = self.med_prop.calc_state("PT", p_evap_trial, t_sat_ev + self.target_superheat).h
            
            # Solve High Side (Extract trial_speed instead of ignoring it)
            _, m_f, _, _, h4_out, trial_speed, l2_iters = self.solve_high_side(
                p_evap_trial, h1_target, self.q_cond_target, current_water_inputs
            )
            step_counters[1] += l2_iters

            # --- ADD THIS SAFEGUARD ---
            if m_f <= 0 or h4_out <= 0:
                return 1e8 # Penalize invalid high-side results immediately
            # --------------------------

            
            # Call Evaporator
            h1_actual = self.evap_sim.calculate_performance(
                T_air_C=self.t_air_in,
                RH_pct=self.rh_in,
                fan_rpm=self.fan_rpm,
                h_ref=h4_out,
                p_ref_Pa=p_evap_trial,
                m_dot_ref=m_f,
                time_step_s=self.time_step_size
            )
            current_error = h1_actual - h1_target

            # Update Progress Bar
            if initial_cop is not None and last_cop is not None:
                cop_ratio = (last_cop / initial_cop) * 100
                cop_str = f"{cop_ratio:.1f}%"
            else:
                # First step: No previous COP available yet
                cop_str = "-"

            # Update the TQDM bar
            pbar.set_postfix({
                "P_evap": f"{p_evap_trial/1e5:.2f}bar",
                "  |  dH_Err": f"{current_error:.1f} J/kg",
                "  |  Full Iterations": step_counters[0],
                "  |  Speed": f"{trial_speed:.2f}",
                "  |  COP Ratio": cop_str
            }, refresh=True)

            return current_error

        except RuntimeError as e:
            # 1. Check if this is equilibrium trap
            if "Equilibrium not reached" in str(e):
                # Silent, smooth penalty: Guide the solver back toward the initial guess
                # This provides a continuous derivative for the Newton solver
                if p_evap_trial < self.p_evap_guess:
                    return 1e5 * (self.p_evap_guess - p_evap_trial) + 1e4
                else:
                    return -1e5 * (p_evap_trial - self.p_evap_guess) - 1e4
            else:
                # 2. It's a REAL crash (e.g., RefProp bounds error). Keep the loud warning.
                print(f"\n[CRITICAL FAILURE] Evap Model crashed at P={p_evap_trial:.0f} Pa")
                print(f" ---> EXACT ERROR: {repr(e)}") 
                
                if p_evap_trial < self.p_evap_guess:
                    return 1e7
                else:
                    return -1e7
                    
        except ValueError as e:
            # Catch other physics/math domain errors loudly
            print(f"\n[CRITICAL FAILURE] Math Domain Error at P={p_evap_trial:.0f} Pa")
            print(f" ---> EXACT ERROR: {repr(e)}") 
            return 1e7 if p_evap_trial < self.p_evap_guess else -1e7

    # =============================================================================
    # MAIN SIMULATION LOOP
    # =============================================================================

    def run_simulation(self):
        """
        Main time-stepping loop.
        Contains the LEVEL 3 Solver ("Evaporation Loop").
        """
        # Result Containers
        res = {
            "time": [], "m_flow": [], "p_evap": [], "p_cond": [], "total_frost_area": [],
            "Q_evap": [], "Q_cond": [], "compressor_power": [], "COP_wo_defrost": [], "speed": [],
            "h1": [], "h2": [], "h3": [], "h4": [],
            "dp_air": [], "fan_power": [], "m_frost": [], "cycles": [], 
            "v_dot_air": [], "space_between_frost": [],
            "iterations": {"Level 2": [], "Level 3": []}, 'fin_pitch': [], "overall_UA": [], "T_evap": [],
        }
        
        # 1. Update Condenser Physics for this specific water temp
        # We must recalculate Cp for the new water temperature
        self.condenser.calc_secondary_cp(T=273.15 + self.t_water_in, p=1e5)
        
        # Create the input object for this specific run
        current_water_inputs = Inputs(
            condenser=HeatExchangerInputs(T_in=273.15 + self.t_water_in, m_flow=self.m_flow_water)
        )

        # --- DYNAMIC SOLVER BOUNDS ---
        # Calculate saturation pressure at T_air (Absolute max possible P_evap)
        p_sat_max = self.med_prop.calc_state("TQ", 273.15 + self.t_air_in, 1.0).p
        
        # Calculate saturation pressure at T_air - 30K (Reasonable min possible P_evap)
        p_sat_min = self.med_prop.calc_state("TQ", 273.15 + self.t_air_in - 30.0, 1.0).p
        
        # Override the hardcoded bounds safely
        # Subtract/Add a small buffer to ensure we don't hit a 0 Kelvin delta_T
        dynamic_p_max = min(self.p_evap_max, p_sat_max - 0.1e5) 
        dynamic_p_min = max(self.p_evap_min, p_sat_min + 0.1e5)

        self.p_evap_guess = max(min(self.p_evap_guess, dynamic_p_max), dynamic_p_min)
        # ----------------------------------

        initial_cop = None
        
        # Default end reason if the loop finishes without breaking
        end_reason = "max_time_reached"

        try:
            with tqdm(self.time_steps, unit="step") as pbar:

                for i, t in enumerate(pbar):

                    # Update the description (Left side of bar)
                    pbar.set_description(f"Sim Time {t/60:.0f}min")

                    # Iteration Counters for this time step
                    step_counters = [0, 0]

                    last_cop = res["COP_wo_defrost"][-1] if res["COP_wo_defrost"] else None
                    evaporation_residual_wrapper = lambda p_evap_trial: self._evaporation_residual(
                        p_evap_trial, step_counters, current_water_inputs, pbar, initial_cop, last_cop
                    )

                    # Execute Level 3 Solver
                    p_evap_final = self.p_evap_guess
                    
                    if i >= 3:
                        # Second-order polynomial extrapolation
                        p_n1 = res["p_evap"][-1]
                        p_n2 = res["p_evap"][-2]
                        p_n3 = res["p_evap"][-3]
                        p_guess = 3 * p_n1 - 3 * p_n2 + p_n3

                        # Define the "Safe Physical Window" (Thermal Inertia)
                        max_dp = 0.2e5  # Max 0.2 bar change per 10s
                        search_min = max(dynamic_p_min, p_n1 - max_dp)
                        search_max = min(dynamic_p_max, p_n1 + max_dp)

                        newton_success = False
                        
                        # 1. The Fast Path: Let Newton try first
                        try:
                            p_newton = newton(
                                func=evaporation_residual_wrapper, 
                                x0=p_guess, 
                                x1=p_n1, 
                                tol=100, 
                                maxiter=15
                            )
                            # VERIFY: Did Newton stay within physical reality?
                            if search_min <= p_newton <= search_max:
                                p_evap_final = p_newton
                                newton_success = True
                            else:
                                pass # Newton solved, but jumped too far (violates inertia). Reject it.
                        except (RuntimeError, ValueError):
                            pass # Newton crashed into a fluid property boundary. Reject it.

                        # 2. The Safe Path: Newton failed or lied, use Clamped Brent
                        if not newton_success:
                            try: 
                                p_evap_final = brentq(f=evaporation_residual_wrapper, a=search_min, b=search_max, xtol=50)
                            except ValueError:
                                # 3. The Panic Path: The system is moving faster than 0.2 bar, use full bounds
                                try:
                                    p_evap_final = brentq(f=evaporation_residual_wrapper, a=dynamic_p_min, b=dynamic_p_max, xtol=100)
                                except ValueError:
                                    print(f"\n[L3 FAIL] Evaporator Loop (Time: {t}). All solvers failed.")
                                    break
                    else:
                        # First few steps: System is initializing, use robust full-bounds solver
                        try:
                            p_evap_final = brentq(f=evaporation_residual_wrapper, a=dynamic_p_min, b=dynamic_p_max, xtol=200)
                        except ValueError:
                            print(f"\n[L3 FAIL] Initial Evaporator Loop (Time: {t}).")
                            break

                    # Final Calculation
                    # Now that we have P_evap, we run the full chain one last time to save data.
                    
                    # Get Final h1
                    t_sat_ev = self.med_prop.calc_state("PQ", p_evap_final, 0).T
                    
                        
                    h1 = self.med_prop.calc_state("PT", p_evap_final, t_sat_ev + self.target_superheat).h
                    
                    # Get High Side results
                    p_cond, m_flow, h2, h3, h4, speed, _ = self.solve_high_side(
                        p_evap_final, h1, self.q_cond_target, current_water_inputs
                        )
                    
                    # Calculate Electrical Power (requires specific compressor call)
                    fs_final = FlowsheetState()
                    inputs_final = Inputs(control=RelativeCompressorSpeedControl(n=speed))
                    self.compressor.state_inlet = self.med_prop.calc_state("PH", p_evap_final, h1)
                    compressor_power = self.compressor.calc_electrical_power(inputs=inputs_final, fs_state=fs_final)

                    # Store Data
                    res["time"].append(t / 60.0)
                    res["m_flow"].append(m_flow)
                    res["p_evap"].append(p_evap_final)
                    res["p_cond"].append(p_cond)
                    
                    res["h1"].append(h1)
                    res["h2"].append(h2)
                    res["h3"].append(h3)
                    res["h4"].append(h4)
                    
                    res["Q_evap"].append(m_flow * (h1 - h4))
                    res["Q_cond"].append(m_flow * (h2 - h3))
                    res["compressor_power"].append(compressor_power)
                
                    res["speed"].append(speed)

                    res["iterations"]["Level 3"].append(step_counters[0])
                    res["iterations"]["Level 2"].append(step_counters[1])

                    # ADVANCE FROST SIMULATION (After convergence for this timestep)
                    current_states = self.evap_sim.advance_simulation_step() 
                    
                    # 1. Calculate Total Frost Mass (Sum over all layers)
                    reg_amount = self.evap_sim.params.global_register_amount

                    total_frost_mass = 0.0
                    total_frost_area = 0.0
                    total_UA = 0.0
                    
                    for j, layer_state in enumerate(current_states):
                        
                        m_layer_val = layer_state.frost.mass * reg_amount
                        total_frost_mass += m_layer_val
                        
                        key_mass = f"m_frost_L{j}"
                        if key_mass not in res:
                            res[key_mass] = []
                        res[key_mass].append(m_layer_val)

                        t_avg_val = (layer_state.hmt.T_frost_surface + layer_state.hmt.T_frost_base) / 2.0
                        key_temp = f"T_frost_avg_L{j}"
                        if key_temp not in res:
                            res[key_temp] = []
                        res[key_temp].append(t_avg_val)

                        total_frost_area += (layer_state.frost.A_frost_surface * reg_amount)

                        q_layer = layer_state.hmt.Q_dot_total * reg_amount
                        t_ref_local = (layer_state.refrigerant.T_in + layer_state.refrigerant.T_out) / 2.0
                        delta_t_layer = layer_state.air.T_avg - t_ref_local
                        
                        if delta_t_layer > 1e-3:
                            total_UA += (q_layer / delta_t_layer)
                    
                    res["m_frost"].append(total_frost_mass)
                    res["total_frost_area"].append(total_frost_area)
                    res["overall_UA"].append(total_UA)
                    res["dp_air"].append(current_states[-1].air.total_system_pressure_drop)
                    res["fan_power"].append(current_states[-1].air.total_fan_power)
                    res["space_between_frost"].append(current_states[-1].frost.space_between_frost)
                    current_cop = res["Q_cond"][-1] / (res["compressor_power"][-1] + res["fan_power"][-1])
                    res["COP_wo_defrost"].append(current_cop)
                    res["v_dot_air"].append(current_states[-1].air.total_v_dot_fan_m3h)
                    res["T_evap"].append(t_sat_ev - 273.15)

                    # Capture baseline COP after 10 time steps to bypass initial instability
                    if i == 10:
                        initial_cop = current_cop

                    # --------------------------
                    # --- Stopping Logic ---
                    stop_simulation = False
                    
                    # 1. Stoppage for COP Drop (Defrost Trigger)
                    if initial_cop is not None and current_cop <= (initial_cop * self.min_cop_drop_ratio):
                        print(f"\n[STOP] COP dropped to {current_cop/initial_cop*100:.1f}% of baseline value (step 10). Defrost triggered.")
                        end_reason = "defrost_triggered"
                        stop_simulation = True

                        
                    # 2. Early Stoppage if frost there is no frost growth
                    if i == 30:
                        m_frost_5  = res["m_frost"][5]
                        m_frost_30 = res["m_frost"][30]
                        
                        if abs(m_frost_30 - m_frost_5) < 1e-15:
                            print(f"\n[STOP] Early Stoppage: No Frost Growth detected.")
                            end_reason = "no_frost"
                            stop_simulation = True

                    # ---------------------------------------------------------
                    # 3. Early Stoppage if Heating Demand is not met
                    # ---------------------------------------------------------
                    q_cond_actual = res["Q_cond"][-1]
                    current_speed = res["speed"][-1]
                    
                    # Allow a tiny numerical tolerance (e.g., 2% off target)
                    if self.q_cond_target is not None:
                        if current_speed >= 0.99 or q_cond_actual < (self.q_cond_target * 0.98):
                            print(f"\n[STOP] Capacity Limit Reached. Cannot meet heating demand.")
                            print(f"      Speed: {current_speed:.2f} | Actual Q_cond: {q_cond_actual:.1f} W (Target: {self.q_cond_target:.1f} W)")
                            end_reason = "defrost_triggered"
                            stop_simulation = True
                    # ---------------------------------------------------------

                    # Capture Cycle
                    if i == 0 or i == len(self.time_steps) - 1 or stop_simulation:
                        cycle = {
                            "name": f"{'Start' if i == 0 else 'End'} (t={t/60:.0f}min)",
                            "p": [p_evap_final, p_cond, p_cond, p_evap_final, p_evap_final],
                            "h": [h1, h2, h3, h4, h1]
                        }
                        res["cycles"].append(cycle)

                    # Break loop
                    if stop_simulation:
                        break
            
            # Record final metadata
            total_duration_min = -1 if end_reason == "no_frost" else (res["time"][-1] if res["time"] else 0.0)

            return res, end_reason, total_duration_min

        except Exception as e:
            print(f"An error occurred during the simulation: {e}")
            
            # Make sure we still assign something if it crashes out
            end_reason = "crash"
            total_duration_min = res["time"][-1] if res["time"] else 0.0
            
            return res, end_reason, total_duration_min