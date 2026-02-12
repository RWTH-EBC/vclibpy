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
from .physics.refrigerant_model import RefrigerantModel, R134a_RP, R410a_RP
from .physics.hmt_model import HeatMassTransferModel
from .physics.thermo_model import ThermoModel



###################################################################################
# Simulation Engine Class
###################################################################################


class FrostEvaporatorSimulation:
    def __init__(self, config_path: str):
        self.params = FrostEvaporatorParameters.from_yaml(config_path)

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
    
    def update_correction_factors(self, new_factors: dict, new_model_choices: dict):
        """
        Updates the internal correction factors dynamically.
        """

        # If correction_factors is a dictionary/Pydantic model:
        self.params.set("correction_factor_h_conv_air", new_factors["h_conv_air"])
        self.params.set("correction_factor_h_conv_ref_2ph", new_factors["h_conv_ref_2ph"])
        self.params.set("correction_factor_surface_density", new_factors["surface_density"])
        self.params.set("correction_factor_betta_air", new_factors["betta_air"])
        self.params.set("correction_factor_k_frost", new_factors["k_frost"])
        self.params.set("correction_factor_pressure_loss", new_factors["pressure_loss"])

        # self.params.set("frost_density_correlation_choice", new_model_choices["frost_density_choice"])
        # self.params.set("frost_conductivity_correlation_choice", new_model_choices["frost_conductivity_choice"])
        # self.params.set("h_conv_air_correlation_choice", new_model_choices["h_conv_air_choice"])

        self.frost_model       = FrostModel(self.params)
        self.air_model         = AirModel(self.params)
        self.refrigerant_model = RefrigerantModel(self.params, self.refprop)
        self.fan_system_model  = FanSystemModel(self.params)
        self.hmt_model         = HeatMassTransferModel(self.params)
        self.thermo_model      = ThermoModel(self.params)

    def _get_boundary_conditions(self, t_min: float, exp_data) -> FrostEvaporatorInputs:
        """
        Retrieves experimental trends for time t, calculates physics (CoolProp), 
        and returns a structured input object.
        """
        # Retrieve trends
        T_air_C  = exp_data.trends['temp'](t_min)
        RH_pct   = exp_data.trends['rh'](t_min)
        fan_rpm  = exp_data.trends['fan'](t_min)
        
        h_ref    = exp_data.trends['h'](t_min)
        p_ref    = exp_data.trends['p'](t_min) * 1e5      # [bar -> Pa]
        m_dot_ref = exp_data.trends['mass'](t_min) / self.params.register_amount

        # Physics Conversions
        T_air_K = 273.15 + T_air_C
        p_air   = 101325.0
        
        # Calculate Humidity Ratio (W) dynamically
        W_air = CP.HAPropsSI('W', 'T', T_air_K, 'P', p_air, 'R', RH_pct / 100.0)

        return FrostEvaporatorInputs(
            air_inputs = AirInputs(
                T_in=T_air_K, p_in=p_air, W_in=W_air, fan_speed_rpm=fan_rpm
            ),
            refrigerant_inputs = RefrigerantInputs(
                h_in=h_ref, p_eva=p_ref, m_dot=m_dot_ref
            )
        )

    def _run_air_sweep(self, states, layer_inputs, global_air_input):
        """Forward pass (0 -> N): Solves Air side"""
        current_air = copy.deepcopy(global_air_input)
        
        for k in range(self.params.layer_amount):
            state = states[k]
            
            # Update Layer Inputs
            layer_inputs[k].air = current_air
            
            # Update Models relevant to Air/Frost interface
            self.frost_model.update_properties(state, layer_inputs[k])
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
        
        for k in reversed(range(self.params.layer_amount)):
            state = states[k]

            # Update Layer Inputs
            layer_inputs[k].refrigerant = current_ref
            
            # Update Models relevant to Refrigerant/Frost interface
            self.frost_model.update_properties(state, layer_inputs[k])
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
            # Snapshot previous surface temps for convergence check
            old_T_surfaces = [s.hmt.T_frost_surface for s in states]

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
            # Robust Logic: Oscillation & Stagnation Detection
            # =========================================================

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

    def run(self, exp_data):
        case_name = str(exp_data.id)
        print(f"--- Simulation of Experiment {case_name} ---")

        # ---------------------------------------------------------
        # 1. Initialization Phase
        # ---------------------------------------------------------
        
        # Set up t=0
        global_inputs = self._get_boundary_conditions(0.0, exp_data)
        
        # Init state objects
        states = [FrostEvaporatorState() for _ in range(self.params.layer_amount)]
        layer_inputs = [copy.deepcopy(global_inputs) for _ in range(self.params.layer_amount)]

        # Prime the states (First blind step)
        for k in range(self.params.layer_amount):
            self.frost_model.step_forward(states[k], global_inputs)

        # Stabilize Initial Conditions (Run solver a few times to settle)
        print("--- Initializing Model ---")
        for _ in range(2): 
            self.solve_equilibrium(states, layer_inputs, global_inputs, max_iter=200)
            
            # Step physics forward to update internal model states
            next_step_input = copy.deepcopy(global_inputs)
            for k in range(self.params.layer_amount):
                self.frost_model.step_forward(states[k], next_step_input)

        # Reset Frost Thickness to starting epsilon (1e-8)
        initial_thickness = 1e-8
        for s in states:
            s.frost.set("thickness", initial_thickness)
            s.frost.set("mass", s.frost.A_frost_surface * initial_thickness * s.frost.density)

        # ---------------------------------------------------------
        # 2. Main Time Loop
        # ---------------------------------------------------------
        print("\n\n--- Starting Simulation ---")
        
        states_history = []
        inputs_history = []
        m_flow_at_t0 = states[0].air.m_dot_humid
        
        duration_mins = exp_data.duration
        # simulation_steps = int(duration_mins * 60 / self.params.time_step + 1)

        simulation_steps = 20 #! Später Ändern
        self.params.set("time_step", duration_mins * 60 / simulation_steps)

        try:
            for j in tqdm(range(simulation_steps), desc=f"Sim {case_name}"):
                current_t_min = (j * self.params.time_step) / 60.0

                # A. Update Boundary Conditions
                global_inputs = self._get_boundary_conditions(current_t_min, exp_data)

                # B. Solve Equilibrium
                converged = self.solve_equilibrium(states, layer_inputs, global_inputs, max_iter=200)
                
                # Check for Air Choke
                if states[0].air.m_dot_humid < 0.1 * m_flow_at_t0:
                    print(f"!!!Air Choke in {case_name} at step {j} (t={current_t_min:.4f}min) !!!")
                    
                    # Save current state before breaking so we can see the crash
                    states_history.append([s.copy() for s in states])
                    inputs_history.append([inp.copy() for inp in layer_inputs])
                    break 

                for s in states:
                    if s.hmt.T_frost_surface > self.params.water_freezing_point:
                        print(f"Warning: Frost Surface Temperature > 0°C at step {j} in L{states.index(s)+1}")

                # D. Store History
                states_history.append([s.copy() for s in states])
                inputs_history.append([inp.copy() for inp in layer_inputs])

                # E. Physical Time Step (Frost Growth)
                # This prepares the geometric state for the NEXT time step
                next_step_input = copy.deepcopy(global_inputs)
                for k in range(self.params.layer_amount):
                    self.frost_model.step_forward(states[k], next_step_input)
        
        except Exception as e:
            import traceback
            print(f"CRASH: Simulation failed with exception: {e}")
            traceback.print_exc()

        return states_history, inputs_history
