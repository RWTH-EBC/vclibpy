import copy
from tqdm import tqdm
import CoolProp.CoolProp as CP

from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
    AirInputs,
    RefrigerantInputs,
)

from .physics.frost_model import FrostModel
from .physics.air_model import AirModel
from .physics.fan_system_model import FanSystemModel
from .physics.refrigerant_model import RefrigerantModel, R410a_RP, R134a_RP
from .physics.hmt_model import HeatMassTransferModel
from .physics.thermo_model import ThermoModel



###################################################################################
# Simulation Engine Class
###################################################################################


class FrostEvaporatorSimulation:
    def __init__(self, config_path: str, external_refprop=None):
        self.params = FrostEvaporatorParameters.from_yaml(config_path)

        
        if external_refprop is not None:
            print(" [Evap] Attaching to external RefProp instance.")
            refprop = external_refprop
        else:
            if self.params.refrigerant == 'R134a':
                refprop = R134a_RP
            elif self.params.refrigerant == 'R410a':
                refprop = R410a_RP
        
        # Instantiate Models
        self.frost_model       = FrostModel(self.params)
        self.air_model         = AirModel(self.params)
        self.refrigerant_model = RefrigerantModel(self.params, refprop)
        self.fan_system_model  = FanSystemModel(self.params)
        self.hmt_model         = HeatMassTransferModel(self.params)
        self.thermo_model      = ThermoModel(self.params)

        self.refprop = refprop

        # Persistent State (Needed for external API access later)
        self.states = []
        self.layer_inputs = []
    
    
    def update_fin_pitch_and_amount(self, new_pitch: float, new_amount: int):  
        """
        Updates the fin pitch parameter and propagates the change to all relevant models.
        """
        # Update Values
        self.params.set("fin_pitch", new_pitch)
        self.params.set("fin_amount", new_amount)

        # Recalculate dependent Values
        self.params.recalculate_geometry()

        # Pass the updated params to the models
        self.frost_model       = FrostModel(self.params)
        self.air_model         = AirModel(self.params)
        self.refrigerant_model = RefrigerantModel(self.params, self.refprop)
        self.fan_system_model  = FanSystemModel(self.params)
        self.hmt_model         = HeatMassTransferModel(self.params)
        self.thermo_model      = ThermoModel(self.params)


    
    def update_correction_factors(self, new_factors: dict, new_model_choices: dict):
        """
        Updates the internal correction factors and correlation choices dynamically.
        """

        # Set correction factors
        self.params.set("correction_factor_h_conv_air", new_factors["h_conv_air"])
        self.params.set("correction_factor_surface_density", new_factors["surface_density"])
        self.params.set("correction_factor_betta_intercept_air", new_factors["betta_intercept_air"])
        self.params.set("correction_factor_betta_slope_air", new_factors["betta_slope_air"])
        self.params.set("correction_factor_k_frost", new_factors["k_frost"])
        self.params.set("correction_factor_frost_diffusion", new_factors["frost_diffusion"])
        self.params.set("correction_factor_pressure_loss", new_factors["pressure_loss"])

        # Set correlations choices
        self.params.set("frost_density_correlation_choice", new_model_choices["frost_density_choice"])
        self.params.set("frost_conductivity_correlation_choice", new_model_choices["frost_conductivity_choice"])
        self.params.set("h_conv_air_correlation_choice", new_model_choices["h_conv_air_choice"])

        # Pass the updated params to the models
        self.frost_model       = FrostModel(self.params)
        self.air_model         = AirModel(self.params)
        self.refrigerant_model = RefrigerantModel(self.params, self.refprop)
        self.fan_system_model  = FanSystemModel(self.params)
        self.hmt_model         = HeatMassTransferModel(self.params)
        self.thermo_model      = ThermoModel(self.params)

    # -------------------------------------------------------------------------
    #  General Helper Functions (Used by Validation & Heat Pump)
    # -------------------------------------------------------------------------

    def reset_simulation(self):
        """Resets the internal state to a clean, initial condition."""
        # Init state objects
        self.states = [FrostEvaporatorState() for _ in range(self.params.global_layer_amount)]
        self.layer_inputs = [None] * self.params.global_layer_amount 

        # Reset Frost Thickness to starting epsilon (1e-8)
        initial_thickness = 1e-8
        for s in self.states:
            s.frost.set("thickness", initial_thickness)
            s.frost.set("mass", s.frost.A_frost_surface * initial_thickness * s.frost.density)

    def _create_inputs(self, T_air_C, RH_pct, fan_rpm, h_ref, p_ref_Pa, m_dot_ref) -> FrostEvaporatorInputs:
        """
        Generic function to create inputs from raw values. 
        Validation uses this via _get_boundary_conditions.
        HeatPump will use this directly.
        """
        # Physics Conversions
        T_air_K = 273.15 + T_air_C
        p_air   = 101325.0

        # Refrigerant Massflow per Register
        m_dot_ref_per_register = m_dot_ref / self.params.global_register_amount
        
        # Calculate Humidity Ratio (W) dynamically
        W_air = CP.HAPropsSI('W', 'T', T_air_K, 'P', p_air, 'R', RH_pct / 100.0)

        return FrostEvaporatorInputs(
            air_inputs = AirInputs(
                T_in=T_air_K, p_in=p_air, W_in=W_air, fan_speed_rpm=fan_rpm
            ),
            refrigerant_inputs = RefrigerantInputs(
                h_in=h_ref, p_eva=p_ref_Pa, m_dot=m_dot_ref_per_register
            )
        )

    def _step_frost_growth(self):
        """
        Advances the physical frost growth by one time step.
        """
        for k in range(self.params.global_layer_amount):
            self.frost_model.step_forward(self.states[k], self.layer_inputs[k])

    # -------------------------------------------------------------------------
    #  Internal Solvers
    # -------------------------------------------------------------------------

    def prime_model(self, initial_inputs: FrostEvaporatorInputs):
        """
        Primes the model state. This MUST be called before the first simulation step.
        It runs the stabilization routine (Initial blind step -> Equilibrium loops -> Reset Thickness)
        to ensure the numerical model starts on the same footing as the validation cases.
        """
        print("--- Priming Evaporator Model ---")

        self.reset_simulation()

        # 1. Set the initial inputs
        self.layer_inputs = [copy.deepcopy(initial_inputs) for _ in range(self.params.global_layer_amount)]

        # 2. Initialization Sequence
        self._step_frost_growth() # First blind step

        # Stabilize
        # for i in range(2):
        # self.solve_equilibrium(self.states, self.layer_inputs, initial_inputs, max_iter=200)
        # self._step_frost_growth()

        # Reset Frost Thickness
        initial_thickness = 1e-8
        for s in self.states:
            s.frost.set("thickness", initial_thickness)
            s.frost.set("mass", s.frost.A_frost_surface * initial_thickness * s.frost.density)        

    def _get_boundary_conditions_from_exp(self, t_min: float, exp_data) -> FrostEvaporatorInputs:
        """
        Retrieves experimental trends for time t and creates input object.
        """
        # Retrieve trends
        T_air_C  = exp_data.trends['temp'](t_min)
        RH_pct   = exp_data.trends['rh'](t_min)
        fan_rpm  = exp_data.trends['fan'](t_min)
        
        h_ref    = exp_data.trends['h'](t_min)
        p_ref    = exp_data.trends['p'](t_min) * 1e5      # [bar -> Pa]
        m_dot_ref = exp_data.trends['mass'](t_min)

        # Use the generic helper
        return self._create_inputs(T_air_C, RH_pct, fan_rpm, h_ref, p_ref, m_dot_ref)

    def _run_air_sweep(self, states, layer_inputs, global_air_input):
        """Forward pass (0 -> N): Solves Air side"""
        current_air = copy.deepcopy(global_air_input)
        
        for k in range(self.params.global_layer_amount):
            state = states[k]

            # Update Layer Inputs
            layer_inputs[k].air = current_air
            
            # Update Models
            self.air_model.update_properties(state, layer_inputs[k])
            self.hmt_model.update_properties(state, layer_inputs[k])
            self.thermo_model.update_properties(state, layer_inputs[k])

            # Pass output to next layer
            current_air = AirInputs(
                T_in  = state.air.T_out,
                p_in  = state.air.p_out, 
                W_in  = state.air.W_out,
                fan_speed_rpm = global_air_input.fan_speed_rpm
            )

    def _run_refrigerant_sweep(self, states, layer_inputs, global_ref_input):
        """Reverse pass (N -> 0): Solves Refrigerant side"""
        current_ref = copy.deepcopy(global_ref_input)
        
        for k in reversed(range(self.params.global_layer_amount)):
            state = states[k]

            # Update Layer Inputs
            layer_inputs[k].refrigerant = current_ref
            
            # Update Models
            self.refrigerant_model.update_properties(state, layer_inputs[k])
            self.hmt_model.update_properties(state, layer_inputs[k])
            self.thermo_model.update_properties(state, layer_inputs[k])

            # Pass output to next layer (upstream)
            current_ref = RefrigerantInputs(
                m_dot = global_ref_input.m_dot,
                p_eva = global_ref_input.p_eva, 
                h_in  = state.refrigerant.h_out
            )

    def solve_equilibrium(self, states, layer_inputs, global_inputs, max_iter=50, tolerance=1e-3) -> bool:
        """
        Iteratively solves the Heat & Mass Transfer balance between Air and Refrigerant.
        Returns True if converged, False otherwise.
        """
        
        for i in range(max_iter):
            # Solve Fan/System Flow
            self.fan_system_model.solve_fan_system_equilibrium(states, global_inputs)

            # Air Loop (Forward)
            for _ in range(1):
                self._run_air_sweep(states, layer_inputs, global_inputs.air)
            
            T_surf_after_air = [s.hmt.T_frost_surface for s in states]

            # Refrigerant Loop (Backward)
            for _ in range(1):
                self._run_refrigerant_sweep(states, layer_inputs, global_inputs.refrigerant)
                
            T_surf_after_ref = [s.hmt.T_frost_surface for s in states]

            # =========================================================
            # Convergence Check
            # =========================================================
            deltas = [abs(ref - air) for ref, air in zip(T_surf_after_ref, T_surf_after_air)]
            max_drift = max(deltas)
            
            if max_drift < tolerance:
                 return True

            # =========================================================
            # Relaxation / Averaging
            # =========================================================
            for k, state in enumerate(states):
                t_air = T_surf_after_air[k]
                t_ref = T_surf_after_ref[k]

                ALPHA = 0.5
                
                # Weighted Average
                t_mixed = (ALPHA * t_ref) + ((1.0 - ALPHA) * t_air)
                
                # Update the state for the next loop
                state.hmt.set("T_frost_surface", t_mixed)

            #! Wieder einschalten, ist cool
            # if max_drift > 0.5:
            #     layer_info = " - ".join([
            #         f"L{k+1}: {d:.4f}K (sh-air: {sha:.4f} | sh-ref: {shr:.4f})" 
            #         for k, (d, sha, shr) in enumerate(zip(deltas, sh_after_air, sh_after_ref))
            #     ])
            #     print(f"Iter {i}: Max Gap {max_drift:.4f} K (Alpha: {ALPHA:.3f}) | {layer_info}")

        print(f"Warning: Equilibrium not reached. Max residual: {max_drift:.4f}")
        return False

    def run_validation(self, exp_data):
        case_name = str(exp_data.id)
        print(f"--- Simulation of Experiment {case_name} ---")

        # ---------------------------------------------------------
        # 1. Initialization Phase
        # ---------------------------------------------------------
        
        # Get t=0 boundary conditions
        t0_inputs = self._get_boundary_conditions_from_exp(0.0, exp_data)

        # Prime the model directly with the object
        self.prime_model(t0_inputs)

        # ---------------------------------------------------------
        # 2. Main Time Loop
        # ---------------------------------------------------------
        print("--- Starting Simulation ---")
        
        states_history = []
        inputs_history = []
        m_flow_at_t0 = self.states[0].air.m_dot_humid
        
        duration_mins = exp_data.duration
        # simulation_steps = int(duration_mins * 60 / self.params.time_step + 1)

        simulation_steps = 20 #! Später Ändern
        self.params.set("time_step", duration_mins * 60 / simulation_steps)

        try:
            # Assign tqdm to a variable so we can update its postfix text
            pbar = tqdm(range(simulation_steps), desc=f"Sim {case_name}")
            for j in pbar:
                current_t_min = (j * self.params.time_step) / 60.0

                # A. Update Boundary Conditions
                global_inputs = self._get_boundary_conditions_from_exp(current_t_min, exp_data)

                # B. Solve Equilibrium
                self.solve_equilibrium(self.states, self.layer_inputs, global_inputs, max_iter=200)
                
                # Check for Air Choke
                if self.states[0].air.m_dot_humid < 0.1 * m_flow_at_t0:
                    print(f"!!!Air Choke in {case_name} at step {j} (t={current_t_min:.4f}min) !!!")
                    
                    # Save current state before breaking so we can see the crash
                    states_history.append([s.copy() for s in self.states])
                    inputs_history.append([inp.copy() for inp in self.layer_inputs])
                    break 

                # Collect affected layers and update the progress bar postfix
                affected_layers = [
                    str(i + 1) for i, s in enumerate(self.states) 
                    if s.hmt.T_frost_surface > self.params.water_freezing_point
                ]
                
                if affected_layers:
                    pbar.set_postfix_str(f"Frost Surface Temp > 0°C in Layers {', '.join(affected_layers)}")
                else:
                    pbar.set_postfix_str("") # Clears the message if the condition is no longer met

                # D. Store History
                states_history.append([s.copy() for s in self.states])
                inputs_history.append([inp.copy() for inp in self.layer_inputs])

                # E. Physical Time Step (Frost Growth)
                self._step_frost_growth()
        
        except Exception as e:
            import traceback
            print(f"CRASH: Simulation failed with exception: {e}")
            traceback.print_exc()

        return states_history, inputs_history


    # -------------------------------------------------------------------------
    #  Heat Pump Interface API
    # -------------------------------------------------------------------------

    def calculate_performance(self, T_air_C, RH_pct, fan_rpm, h_ref, p_ref_Pa, m_dot_ref, time_step_s) -> float:
        """
        Calculates the current evaporator state (Outlet Enthalpy) for a specific operating point.
        
        NOTE: This does NOT advance frost growth. It only solves the thermodynamic equilibrium.
        It can be called iteratively by the Heat Pump solver.
        """
        # 1. Update the timestep (stored for the eventual advance_simulation_step call)
        self.params.set("time_step", time_step_s)

        # 2. Create Input Object
        global_inputs = self._create_inputs(T_air_C, RH_pct, fan_rpm, h_ref, p_ref_Pa, m_dot_ref)

        # 3. Solve Equilibrium (Air/Ref balance)
        # We use the existing self.states as the starting guess (hot start) for speed.
        is_converged = self.solve_equilibrium(self.states, self.layer_inputs, global_inputs, max_iter=50)

        if not is_converged:
            # You might want to log this or raise a warning, depending on how robust the HP solver is
            pass

        # 4. Return Outlet Enthalpy
        # Based on reverse sweep (N->0), the outlet of the evaporator is the outlet of Layer 0.
        return self.states[0].refrigerant.h_out

    def advance_simulation_step(self):
        """
        freezes the current thermodynamic state and advances the frost growth by 'time_step'.
        Returns the full state object for logging purposes.
        """
        # 1. Calculate Frost Growth based on the equilibrium found in calculate_performance
        self._step_frost_growth()

        # 2. Return deep copies of states for external Logging
        # We copy them so the logger doesn't hold references to objects that change in the next step.
        return [s.copy() for s in self.states]