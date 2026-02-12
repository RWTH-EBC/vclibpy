"""
simulation_workflow.py

This module acts as the 'driver' for the Frost Evaporator simulation.
It orchestrates data loading, simulation execution, and result visualization.
"""

# --- 1. Imports ---
import os
import sys
import copy
from tqdm import tqdm
import CoolProp.CoolProp as CP
import matplotlib.pyplot as plt 
import sys
from dataclasses import dataclass
from typing import Callable, Dict, Tuple, List, Union
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.stats import binned_statistic
from scipy.interpolate import interp1d
import matplotlib.cm as cm

from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
    AirInputs,
    RefrigerantInputs,
)

from scipy.stats import linregress

from .frost_model import FrostModel
from .air_model import AirModel
from .fan_system_model import FanSystemModel
from .refrigerant_model import RefrigerantModel, R134a_RP, R410a_RP
from .hmt_model import HeatMassTransferModel
from .thermo_model import ThermoModel

import sys
import numpy as np
import pandas as pd
from pathlib import Path
from dataclasses import dataclass
# Add 'Optional' to this line:
from typing import List, Tuple, Dict, Callable, Union, Optional 
from scipy.interpolate import interp1d
from scipy.stats import binned_statistic



###################################################################################
# Data Loader Class
###################################################################################



@dataclass
class ExperimentInputs:
    id: str
    duration: float
    averages: Tuple[float, float]  # (Temp, Humidity)
    trends: Dict[str, Callable[[float], float]]

    def __str__(self):
        """Generates a clean dashboard-style text report."""
        return (
            f" Experiment ID  : {self.id}\n"
            f" Duration       : {self.duration:.2f} min\n"
            f" Avg Temp/RH    : {self.averages[0]:.2f}°C / {self.averages[1]:.2f}%\n"
        )

# ==============================================================================
#  ANALYZER CLASS
# ==============================================================================

class MultiExperimentAnalyzer:
    
    # --- CONFIGURATION SCHEMAS ---
    
    COL_MAP_OPTIABT = {
        'time':       ['time'],
        'modus':      ['modus_val', 'modus'],
        'fan':        ['VD_n2'],                                  # Needs * 60
        'mass':       ['MSS_rMassenstrom'],                       # Usually g/s
        'p_in':       ['VD_p_out'],
        'h_in':       ['VD_h_in_korr', 'VD_isenthalp_h_in'],
        'temp_kk':    ['TempKK'],
        'rh_kk':      ['KK_rlFeuchteKK', 'rlFeuchteKK']
    }

    COL_MAP_OPTIHORST = {
        'time':       ['time'],
        'modus':      ['modus_val'],                              # Often missing -> calculated
        'fan':        ['StateMachine_rps_EvapFan'],               # Needs * 60
        'mass':       ['StateMachine_m_CompOut'],                 # Usually kg/s
        'p_in':       ['StateMachine_p_EvapOut'],
        'h_in':       ['StateMachine_h_EvapIn'],
        'temp_kk':    ['KK_Temp_KK'],
        'rh_kk':      ['KK_rlFeuchte_KK']
    }

    def __init__(self, project_root_rel: str = '..', experiment_type: str = 'OptiAbt'):
        """
        Initialize configuration and setup paths.
        
        :param project_root_rel: Relative path to project root.
        :param experiment_type: 'OptiAbt' or 'OptiHorst'. Determines column mapping and logic.
        """
        # Select Configuration
        if experiment_type == "OptiHorst":
            self.COLS = self.COL_MAP_OPTIHORST
        elif experiment_type == "OptiAbt":
            self.COLS = self.COL_MAP_OPTIABT
        else:
            raise ValueError(f"Unknown experiment_type: {experiment_type}")
            
        self.exp_type = experiment_type

        # Ensure project root is in path
        self.project_root = Path(project_root_rel).resolve()
        if str(self.project_root) not in sys.path:
            sys.path.append(str(self.project_root))

    # ===========================================================================
    #  STATIC HELPER METHODS (Math & Data Util)
    # ===========================================================================

    @staticmethod
    def get_col(df: pd.DataFrame, df_fallback: pd.DataFrame, keys: List[str]) -> Optional[pd.Series]:
        """
        Retrieves column checking multiple keys and two DataFrames (Main and KK).
        Returns None if not found.
        """
        for k in keys:
            if k in df.columns: return df[k]
            if k in df_fallback.columns: return df_fallback[k]
        return None

    @staticmethod
    def polynomial_fit(x: np.ndarray, y: np.ndarray, degree: int = 1) -> Callable[[float], float]:
        if len(x) == 0: return lambda t: 0.0
        coeffs = np.polyfit(x, y, degree)
        return np.poly1d(coeffs)

    @staticmethod
    def hard_step_avg(x: np.ndarray, y: np.ndarray, interval_minutes: float = 3.0) -> Callable[[float], float]:
        if len(x) == 0: return lambda t: 0.0
        
        t_max = np.max(x)
        bins = np.arange(0, t_max + interval_minutes, interval_minutes)
        
        # Calculate statistics
        bin_means, bin_edges, _ = binned_statistic(x, y, statistic='mean', bins=bins)
        
        # Fill NaNs (for empty bins)
        bin_means = pd.Series(bin_means).ffill().bfill().to_numpy()
        
        # --- MODIFICATION: Always overwrite the last bin ---
        if len(bin_means) > 1:
            bin_means[-1] = bin_means[-2]
        # ---------------------------------------------------
        
        # Zero-Order Hold Interpolator
        return interp1d(
            bin_edges[:-1], bin_means, kind='zero', 
            fill_value="extrapolate", bounds_error=False
        )

    def _calculate_implicit_mode(self, df: pd.DataFrame, df_kk: pd.DataFrame) -> pd.Series:
        """
        Logic from Plotting Script: Calculates valid mode based on Fan RPM
        if explicit 'modus' column is missing or invalid.
        """
        # Try to get Fan Speed
        fan_raw = self.get_col(df, df_kk, self.COLS['fan'])
        
        if fan_raw is None:
            # Fallback: Assume everything is valid if no fan data exists
            return pd.Series(1, index=df.index)

        fan_vals = pd.to_numeric(fan_raw, errors='coerce').fillna(0)
        
        # Check if running (Threshold > 1 to account for noise/rps/rpm differences)
        is_running = fan_vals > 1
        
        if not is_running.any():
            return pd.Series(0, index=df.index)

        # Find longest continuous group
        group_ids = (is_running != is_running.shift()).cumsum()
        active_groups = group_ids[is_running]
        
        if active_groups.empty:
             return pd.Series(0, index=df.index)

        longest_group = active_groups.value_counts().idxmax()
        modus_series = (group_ids == longest_group).astype(int)
        
        # Trim Start/End (cleanup artifacts)
        valid_indices = modus_series[modus_series == 1].index
        if len(valid_indices) > 20:
            modus_series.loc[valid_indices[:10]] = 0 
            modus_series.loc[valid_indices[-10:]] = 0
            
        return modus_series

    # ===========================================================================
    #  CORE ANALYSIS LOGIC
    # ===========================================================================

    def analyze(self, exp_ids: List[int], data_path: Path, cutoff_pct: float, time_step: float) -> Optional[ExperimentInputs]:
        
        combined_data = {
            'time': [], 'time_stable': [],
            'fan': [], 'mass': [], 'p': [], 'h': [], 'temp': [], 'rh': []
        }
        
        durations = []
        t_avgs = []
        rh_avgs = []

        print(f"--- Aggregating Data ({self.exp_type}) for Experiments: {exp_ids} ---")

        for exp_id in exp_ids:
            try:
                p = Path(data_path)
                df = pd.read_csv(p / f"{exp_id}_data.csv", sep=';', decimal='.', on_bad_lines='skip', low_memory=False)
                
                # Check for KK file
                kk_path = p / f"{exp_id}_data_KK.csv"
                if kk_path.exists():
                    df_kk = pd.read_csv(kk_path, sep=';', decimal='.', low_memory=False)
                else:
                    df_kk = pd.DataFrame(index=df.index)
                    
            except FileNotFoundError:
                print(f"Warning: Skipping {exp_id}, file not found.")
                continue

            # --- 1. Determine Mode / Mask ---
            modus_col = self.get_col(df, df_kk, self.COLS['modus'])
            
            if modus_col is not None and modus_col.max() > 0:
                # Explicit column exists
                mask_mode = pd.to_numeric(modus_col, errors='coerce') == 1
            else:
                # Implicit calculation (OptiHorst Logic)
                mask_mode = self._calculate_implicit_mode(df, df_kk) == 1

            # Intersect Indices
            valid_idx = df.index[mask_mode]
            if not df_kk.empty:
                valid_idx = valid_idx.intersection(df_kk.index)

            df = df.loc[valid_idx]
            df_kk = df_kk.loc[valid_idx]

            # --- 2. Time Handling ---
            t_raw = self.get_col(df, df_kk, self.COLS['time'])
            
            if t_raw is not None:
                t_arr = pd.to_numeric(t_raw, errors='coerce').values
            else:
                # Fallback to Index if time is missing
                t_arr = df.index.values * 1.0

            x_time = (t_arr - t_arr[0]) / 60.0 # Minutes
            
            total_dur = np.max(x_time)
            durations.append(total_dur)
            mask_stable = x_time >= (total_dur * cutoff_pct)

            # --- 3. Data Extraction & Unit Normalization ---
            
            # Helper to extract and ensure numeric
            def get_val(keys):
                s = self.get_col(df, df_kk, keys)
                return pd.to_numeric(s, errors='coerce').fillna(0).values if s is not None else np.zeros(len(df))

            # FAN (RPM)
            # Both systems typically require * 60 (RPS -> RPM or scaling)
            # If standard OptiAbt is already RPM, this might need adjustment, 
            # but based on prompt OptiAbt was `VD_n2` * 60 in the original code? 
            # (Note: Original code said `self.get_col(...) * 60.0`). Assuming valid for both.
            val_fan = get_val(self.COLS['fan']) * 60.0

            # MASS FLOW (kg/s)
            val_mass = get_val(self.COLS['mass'])
            # Heuristic from plotting script: 
            # If mean < 0.5, it's likely already kg/s. If > 0.5, it's g/s.
            if np.mean(val_mass) > 0.5:
                val_mass = val_mass / 1000.0
            
            val_p = get_val(self.COLS['p_in'])
            val_h = get_val(self.COLS['h_in'])

            # Convert h to J/kg if needed
            if self.exp_type == "OptiAbt":
                val_h = val_h * 1000.0

            val_temp = get_val(self.COLS['temp_kk'])
            val_rh = get_val(self.COLS['rh_kk'])

            # Store Data
            combined_data['time'].append(x_time)
            combined_data['time_stable'].append(x_time[mask_stable])
            
            combined_data['fan'].append(val_fan[mask_stable])
            combined_data['mass'].append(val_mass[mask_stable])
            combined_data['p'].append(val_p[mask_stable])
            combined_data['h'].append(val_h[mask_stable])
            combined_data['temp'].append(val_temp[mask_stable])
            combined_data['rh'].append(val_rh[mask_stable])

            # Meta Stats
            t_avgs.append(np.mean(val_temp[mask_stable]))
            rh_avgs.append(np.mean(val_rh[mask_stable]))

        # --- Concatenate & Fit ---
        if not combined_data['time']:
            print("No valid data found for any experiment.")
            return None

        X_all = np.concatenate(combined_data['time'])
        X_stable = np.concatenate(combined_data['time_stable'])
        
        # Generate Trends
        trends = {
            'fan':  self.hard_step_avg(X_stable, np.concatenate(combined_data['fan']), interval_minutes=time_step/60.0),
            'mass': self.polynomial_fit(X_stable, np.concatenate(combined_data['mass']), degree=1),
            'p':    self.polynomial_fit(X_stable, np.concatenate(combined_data['p']), degree=1),
            'h':    self.polynomial_fit(X_stable, np.concatenate(combined_data['h']), degree=1),
            'temp': self.polynomial_fit(X_stable, np.concatenate(combined_data['temp']), degree=0),
            'rh':   self.polynomial_fit(X_stable, np.concatenate(combined_data['rh']), degree=0),
        }

        # --- Final Aggregation ---
        max_duration = np.max(durations)

        return ExperimentInputs(
            id=f"Combined_{len(exp_ids)}_{self.exp_type}", 
            duration=max_duration, 
            averages=(np.mean(t_avgs), np.mean(rh_avgs)), 
            trends=trends
        )
    


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
        Expects a dictionary like: {'h_conv_air': 1.3, 'k_frost': 0.9}
        """

        # If correction_factors is a dictionary/Pydantic model:
        self.params.set("correction_factor_h_conv_air", new_factors["h_conv_air"])
        self.params.set("correction_factor_surface_density", new_factors["surface_density"])
        self.params.set("correction_factor_betta_air", new_factors["betta_air"])
        self.params.set("correction_factor_k_frost", new_factors["k_frost"])
        self.params.set("correction_factor_eta_fin", new_factors["eta_fin"])
        self.params.set("correction_factor_roughness_exponent", new_factors["roughness_exponent"])
        self.params.set("correction_factor_pressure_loss", new_factors["pressure_loss"])

        self.params.set("frost_density_correlation_choice", new_model_choices["frost_density_choice"])
        self.params.set("frost_conductivity_correlation_choice", new_model_choices["frost_conductivity_choice"])
        self.params.set("h_conv_air_correlation_choice", new_model_choices["h_conv_air_choice"])


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
            # 1. Snapshot previous surface temps for convergence check
            old_T_surfaces = [s.hmt.T_frost_surface for s in states]

            # 2. Solve Fan/System Flow
            self.fan_system_model.solve_fan_system_equilibrium(states, global_inputs)

            # 3. Air Loop (Forward)
            for _ in range(1):
                self._run_air_sweep(states, layer_inputs, global_inputs.air)
            
            T_surf_after_air = [s.hmt.T_frost_surface for s in states]
            # Capture SH portion immediately after Air sweep (before Ref sweep changes it)
            sh_after_air = [s.refrigerant.portion_superheated for s in states]

            # 4. Refrigerant Loop (Backward)
            for _ in range(1):
                self._run_refrigerant_sweep(states, layer_inputs, global_inputs.refrigerant)
                
            T_surf_after_ref = [s.hmt.T_frost_surface for s in states]
            # Capture SH portion after Ref sweep
            sh_after_ref = [s.refrigerant.portion_superheated for s in states]

            # =========================================================
            # 5. Convergence Check (The "Gap" Check)
            # =========================================================
            # We calculate how far apart the two physics models are.
            deltas = [abs(ref - air) for ref, air in zip(T_surf_after_ref, T_surf_after_air)]
            max_drift = max(deltas)
            
            if max_drift < tolerance:
                # OPTIONAL: One final check to ensure we aren't drifting globally 
                avg_movement = max(abs(s.hmt.T_frost_surface - old_t) for s, old_t in zip(states, old_T_surfaces))
                
                if avg_movement < tolerance:
                    return True

            # =========================================================
            # 6. Relaxation / Averaging (Prepare for NEXT step)
            # =========================================================
            
            for k, state in enumerate(states):
                t_air = T_surf_after_air[k]
                t_ref = T_surf_after_ref[k]

                ALPHA = 0.5
                
                # Weighted Average
                t_mixed = (ALPHA * t_ref) + ((1.0 - ALPHA) * t_air)
                
                # Update the state for the next loop
                state.hmt.set("T_frost_surface", t_mixed)

            #! TURN BACK ON, THIS IS NICE
            # if max_drift > 0.5:
            #     # Format: L1: 3.12K (sh-air: 0.32 | sh-ref: 1.00) - ...
            #     layer_info = " - ".join([
            #         f"L{k+1}: {d:.2f}K (sh-air: {sha:.2f} | sh-ref: {shr:.2f})" 
            #         for k, (d, sha, shr) in enumerate(zip(deltas, sh_after_air, sh_after_ref))
            #     ])
            #     print(f"Iter {i}: Max Gap {max_drift:.2f} K | {layer_info}")

        print(f"Warning: Equilibrium not reached. Max residual: {max_drift:.2f}")
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
        
        duration_mins = exp_data.duration
        simulation_steps = int(duration_mins * 60 / self.params.time_step + 1)

        try:
            for j in tqdm(range(simulation_steps), desc=f"Sim {case_name}"):
                current_t_min = (j * self.params.time_step) / 60.0

                # A. Update Boundary Conditions
                global_inputs = self._get_boundary_conditions(current_t_min, exp_data)

                # B. Solve Equilibrium
                converged = self.solve_equilibrium(states, layer_inputs, global_inputs, max_iter=200)

                # C. Safety Checks
                if not converged and j % 100 == 0:
                    print(f"Warning: Step {j} did not strictly converge.")
                
                # Check for Air Choke
                if states[0].air.m_dot_humid < 1e-3:
                    print(f"!!!Air Choke in {case_name} at step {j} (t={current_t_min:.2f}min) !!!")
                    
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


####################################################################################
# Visualization Class
####################################################################################

import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as path_effects
import numpy as np
import os
import pandas as pd
import matplotlib.gridspec as gridspec
import CoolProp.CoolProp as CP 

# ==============================================================================
#  CONFIGURATION: DATA MAPPING
# ==============================================================================
COLUMN_MAPPING_OptiAbt = {
    'time':       ['time'],
    'modus':      ['modus_val', 'modus'],
    'n_fan':      ['VD_n2'],                                  # Will be * 60
    'T_in':       ['TempKK'],
    'rh_in':      ['KK_rlFeuchteKK', 'rlFeuchteKK'],
    'm_ref':      ['MSS_rMassenstrom'],                       # Will be / 1000 check
    'p_ref_in':   ['VD_p_out'],
    'h_ref_in':   ['VD_h_in_korr', 'VD_isenthalp_h_in'],
    'h_ref_out':  ['VD_h_out'],
    'dp':         ['Delta_P_VD'],
    'mass_raw':   ['WAAGEN_Waage1_Masse_smooth', 'WAAGEN_Waage2_Masse']
}

COLUMN_MAPPING_ObtiHorst = {
    'time':       ['time'], 
    'modus':      ['modus_val'],
    'n_fan':      ['StateMachine_rps_EvapFan'],               # Will be * 60
    'T_in':       ['KK_Temp_KK'],
    'rh_in':      ['KK_rlFeuchte_KK'],
    'm_ref':      ['StateMachine_m_CompOut'],
    'p_ref_in':   ['StateMachine_p_EvapOut'],
    'h_ref_in':   ['StateMachine_h_EvapIn'],
    'h_ref_out':  ['StateMachine_h_EvapOut'],
    'dp':         ['StateMachine_p_PresLos'],
    'mass_raw':   ['WAAGEN_Waage2_Masse']
}

class SimulationVisualizer:
    """Handles all plotting and visualization for Frost Evaporator simulations."""
    
    def __init__(self, params):
        self.params = params

    def _get_data_col(self, df_main, df_kk, mapping, key_name):
        """Helper to find columns based on mapping."""
        possible_names = mapping.get(key_name, [])
        for col in possible_names:
            if col in df_main.columns:
                return df_main[col]
            if col in df_kk.columns:
                return df_kk[col]
        return None


    def plot_layer_temperatures(self, states_history):
        """
        Plots the evaporation temperature and the outlet temperatures of all layers
        in a single consolidated plot.
        """
        steps = len(states_history)
        num_layers = len(states_history[0])
        
        # Create Time Axis (Minutes)
        t_sim = [i * self.params.time_step / 60.0 for i in range(steps)]

        # Setup Plot: Single plot for all data
        fig, ax = plt.subplots(figsize=(12, 7))
        
        # 1. Plot Evaporation Temperature (Ref from Layer 0 as the baseline)
        # We use a thicker, dashed line to make it stand out as the reference
        t_evap = [step[0].refrigerant.T_two_phase_in - 273.15 for step in states_history]
        ax.plot(t_sim, t_evap, label='T Evaporation (Saturation)', 
                color='black', linestyle='--', linewidth=2.5, zorder=5)

        # 2. Plot Outlet Temperatures for each layer
        # Using a colormap to differentiate layers nicely
        colors = cm.viridis([i / max(1, num_layers - 1) for i in range(num_layers)])

        for k in range(num_layers):
            t_out = [step[k].refrigerant.T_out - 273.15 for step in states_history]
            ax.plot(t_sim, t_out, label=f'Layer {k} Outlet', color=colors[k], alpha=0.8)

        # Styling
        ax.set_title("Refrigerant Temperature Evolution by Layer", fontsize=14, pad=15)
        ax.set_ylabel("Temperature [°C]", fontsize=12)
        ax.set_xlabel("Time [min]", fontsize=12)
        
        # Put legend outside the plot if there are many layers
        ax.legend(loc='center left', bbox_to_anchor=(1, 0.5), frameon=True, shadow=True)
        
        ax.grid(True, linestyle=':', alpha=0.7)
        plt.tight_layout()
        plt.show()
    

    def plot_heat_transfer_coefficients(self, states_history):
        """
        Debug Plotter: Visualizes the convective heat transfer coefficients 
        (two-phase and superheated) per layer over time.
        """
        steps = len(states_history)
        num_layers = len(states_history[0])
        
        # Create Time Axis (Minutes)
        t_sim = [i * self.params.time_step / 60.0 for i in range(steps)]

        # Setup Plot: One row per layer
        fig, axes = plt.subplots(num_layers, 1, figsize=(12, 4 * num_layers), sharex=True)
        if num_layers == 1: 
            axes = [axes]

        for k in range(num_layers):
            ax = axes[k]
            
            # Extract Data
            # h_conv_two_phase: coefficient during evaporation
            # h_conv_superheated: coefficient once the refrigerant is fully gaseous
            h_tp = [step[k].refrigerant.h_conv_two_phase for step in states_history]
            h_sh = [step[k].refrigerant.h_conv_superheated for step in states_history]
            
            # Plotting both on the same axis
            ax.plot(t_sim, h_tp, label='h_conv Two-Phase', color='teal', linewidth=2)
            ax.plot(t_sim, h_sh, label='h_conv Superheated', color='darkorange', linestyle='-.')

            # Styling
            ax.set_title(f"Layer {k} - Convective Heat Transfer Coefficients")
            ax.set_ylabel(r"HTC [W/(m²·K)]")
            ax.legend(loc='upper right')
            ax.grid(True, linestyle=':', alpha=0.7)

        axes[-1].set_xlabel("Time [min]")
        plt.tight_layout()
        plt.show()
    
    def plot_comparison(self, states_history, inputs_history, experiment_ids, path_exp, 
                        cutoff_pct=0.02, save_fig=False, group_name=None, experiment_name="OptiAbt"):
        """
        Master Dashboard: Comparison of N Experiments vs 1 Simulation.
        Supports 'OptiAbt' and 'OptiHorst' data structures.
        """

        # =========================================================================
        # 0. CONFIGURATION & COLORS
        # =========================================================================
        
        # Select Mapping
        if experiment_name == "OptiHorst":
            MAPPING = COLUMN_MAPPING_ObtiHorst
        elif experiment_name == "OptiAbt":
            MAPPING = COLUMN_MAPPING_OptiAbt
        else:
            raise ValueError(f"Unknown experiment_name: {experiment_name}")

        BLUE_GRADIENT = ['#89CFF0', '#4682B4', '#0047AB', '#000080', '#04043A']
        RED_GRADIENT  = ['#FFB3B3', '#FF8080', '#FF4D4D', '#E60000', '#8B0000']
        
        c_sim = 'black'
        c_sim_ref = '#0C00A4' 

        if len(experiment_ids) > 5:
            raise ValueError(f"Too many experiments ({len(experiment_ids)}). Maximum allowed is 5.")

        # =========================================================================
        # 1. DATA PREPARATION
        # =========================================================================
        steps = len(states_history)
        t_sim = np.array([i * self.params.time_step / 60.0 for i in range(steps)])
        t_total = t_sim[-1]

        # Cutoff timestamps
        t_cut_start = t_total * cutoff_pct
        t_cut_end = t_total * (1.0 - cutoff_pct)
        mask_sim_stable = (t_sim >= t_cut_start) & (t_sim <= t_cut_end)

        # --- SIMULATION DATA ---
        # We collect everything into a single dict for easy access
        sim_data = {
            # Inputs
            'n_fan': [], 'T_in': [], 'rh_in': [], 
            'm_ref': [], 'p_ref_in': [], 'h_ref_in': [],
            # Outputs
            'dp': [], 'h_ref_out': [], 'Q': [], 'm_frost': []
        }

        num_layers = len(states_history[0])
        reg_amount = self.params.register_amount

        for t in range(steps):
            # -- Inputs --
            inp_air = inputs_history[t][0].air
            sim_data['T_in'].append(inp_air.T_in - 273.15)
            sim_data['n_fan'].append(inp_air.fan_speed_rpm)
            
            try:
                rh = CP.HAPropsSI('R', 'T', inp_air.T_in, 'P', inp_air.p_in, 'W', inp_air.W_in)
                sim_data['rh_in'].append(rh * 100.0)
            except:
                sim_data['rh_in'].append(0.0)

            ref_in_idx = num_layers - 1
            inp_ref = inputs_history[t][ref_in_idx].refrigerant
            sim_data['m_ref'].append(inp_ref.m_dot * reg_amount)
            sim_data['p_ref_in'].append(inp_ref.p_eva / 1e5)
            sim_data['h_ref_in'].append(inp_ref.h_in / 1e3)

            # -- Outputs --
            dp_total = 0; q_total = 0; mass_frost_total = 0
            sim_data['h_ref_out'].append(states_history[t][0].refrigerant.h_out / 1e3)

            for k in range(num_layers):
                s = states_history[t][k]
                dp_total += s.air.pressure_drop
                q_total += s.hmt.Q_dot_total
                mass_frost_total += (s.frost.mass * 1000.0 * reg_amount)
            
            sim_data['dp'].append(dp_total)
            sim_data['Q'].append(q_total * reg_amount)
            sim_data['m_frost'].append(mass_frost_total)

        # Convert to Numpy
        for k in sim_data:
            sim_data[k] = np.array(sim_data[k])

        # --- EXPERIMENT DATA ---
        experiments = []
        
        # Scaling Calculator: Collect ALL stable data points here to determine Y-limits
        scaling_data = {k: [] for k in sim_data.keys()}
        
        # Add Sim stable data to scaling
        for k in scaling_data:
            if np.any(mask_sim_stable):
                scaling_data[k].extend(sim_data[k][mask_sim_stable])
        
        print(f"--- Visualizing {len(experiment_ids)} Experiments ({experiment_name}) ---")
        
        for exp_id in experiment_ids:
            entry = {'id': exp_id, 'avail': False, 'time': None, 'data': {}}
            try:
                f_data = os.path.join(path_exp, f"{exp_id}_data.csv")
                f_kk = os.path.join(path_exp, f"{exp_id}_data_KK.csv")

                if os.path.exists(f_data):
                    # Load Data
                    df = pd.read_csv(f_data, sep=';', decimal='.', on_bad_lines='skip', low_memory=False)
                    if os.path.exists(f_kk):
                        df_kk = pd.read_csv(f_kk, sep=';', decimal='.', low_memory=False)
                    else:
                        df_kk = pd.DataFrame(index=df.index)

                    # =========================================================
                    # 1. ROBUST MODE DETECTION (Ported from Analyzer)
                    # =========================================================
                    modus_col = self._get_data_col(df, df_kk, MAPPING, 'modus')
                    mask_mode = None

                    # A: Explicit Modus Column exists and has data
                    if modus_col is not None:
                        modus_num = pd.to_numeric(modus_col, errors='coerce').fillna(0)
                        if modus_num.max() > 0:
                            mask_mode = (modus_num == 1)

                    # B: Fallback -> Implicit Calculation (Fan Logic)
                    if mask_mode is None or mask_mode.sum() == 0:
                        fan_raw = self._get_data_col(df, df_kk, MAPPING, 'n_fan')
                        if fan_raw is not None:
                            fan_vals = pd.to_numeric(fan_raw, errors='coerce').fillna(0)
                            is_running = fan_vals > 1 # Threshold
                            
                            # Find longest continuous run
                            group_ids = (is_running != is_running.shift()).cumsum()
                            if is_running.any():
                                active_groups = group_ids[is_running]
                                longest_group = active_groups.value_counts().idxmax()
                                mask_mode = (group_ids == longest_group)
                            else:
                                mask_mode = pd.Series(False, index=df.index)
                        else:
                            # If no Fan data, assume all valid
                            mask_mode = pd.Series(True, index=df.index)

                    # =========================================================
                    # 2. CUT & PROCESS
                    # =========================================================
                    valid_idx = df.index[mask_mode]
                    if not df_kk.empty:
                        valid_idx = valid_idx.intersection(df_kk.index)

                    # Safety: If cut results in empty, revert to full (Prevent Empty Plots)
                    if len(valid_idx) < 10:
                        print(f"Warning: Filter removed all data for {exp_id}. Showing full dataset.")
                        valid_idx = df.index

                    df_cut = df.loc[valid_idx].copy()
                    df_kk_cut = df_kk.loc[valid_idx].copy()

                    # Time Handling
                    t_raw = self._get_data_col(df_cut, df_kk_cut, MAPPING, 'time')
                    if t_raw is not None:
                        t_arr = pd.to_numeric(t_raw, errors='coerce').values
                    else:
                        t_arr = df_cut.index.values * 1.0
                    
                    # Normalize Time
                    t_exp = (t_arr - t_arr[0]) / 60.0
                    entry['time'] = t_exp

                    # --- EXTRACT VARIABLES ---
                    d = {}
                    
                    # Helper
                    def get_val(key, default_val=0.0):
                        v = self._get_data_col(df_cut, df_kk_cut, MAPPING, key)
                        return pd.to_numeric(v, errors='coerce').fillna(default_val).values if v is not None else np.zeros(len(df_cut))

                    d['n_fan']    = get_val('n_fan') * 60.0 # Ensure RPM
                    d['T_in']     = get_val('T_in')
                    d['rh_in']    = get_val('rh_in')
                    d['p_ref_in'] = get_val('p_ref_in')
                    d['dp']       = get_val('dp')

                    # Mass Flow Heuristic (g/s vs kg/s)
                    m_raw = get_val('m_ref')
                    if np.mean(m_raw) > 0.5: # Likely g/s
                         d['m_ref'] = m_raw / 1000.0
                    else: # Likely kg/s
                         d['m_ref'] = m_raw

                    # Enthalpy Handling
                    h_in = get_val('h_ref_in')
                    h_out = get_val('h_ref_out')
                    # OptiAbt usually needs kJ conversion if raw is J, checks magnitude
                    if np.mean(h_in) > 10000: # Likely J/kg
                        d['h_ref_in'] = h_in / 1000.0
                        d['h_ref_out'] = h_out / 1000.0
                    else:
                        d['h_ref_in'] = h_in
                        d['h_ref_out'] = h_out

                    # Q Calculation
                    q_kw = d['m_ref'] * (d['h_ref_out'] - d['h_ref_in'])
                    d['Q'] = q_kw * 1000.0

                   # =========================================================
                    # FROST MASS: SMART ZEROING (Regression 10-15%)
                    # =========================================================
                    from scipy.stats import linregress
                    
                    m_frost_raw = get_val('mass_raw')
                    m_frost_min = 0.0 # Initialize safety default

                    if len(m_frost_raw) > 0:
                        # 1. Define Window (10-15% of SIMULATION time)
                        #    t_total comes from the simulation time array at the top of the function
                        mask_reg = (t_exp >= t_total * 0.10) & (t_exp <= t_total * 0.15)
                        
                        # 2. Calculate Intercept (Virtual Zero)
                        if np.sum(mask_reg) > 10:
                            res = linregress(t_exp[mask_reg], m_frost_raw[mask_reg])
                            m_frost_min = res.intercept 
                        else:
                            # Fallback to min if window is empty/too small
                            m_frost_min = m_frost_raw.min()

                        d['m_frost'] = (m_frost_raw - m_frost_min) * 1000.0
                    else:
                        d['m_frost'] = np.zeros(len(df_cut))

                    # =========================================================
                    # OPTIHORST: FULL DATA LOADING (For Defrost/Plateau Check)
                    # =========================================================
                    if experiment_name == "OptiHorst":
                        t_full_raw = self._get_data_col(df, df_kk, MAPPING, 'time')
                        m_full_raw = self._get_data_col(df, df_kk, MAPPING, 'mass_raw')

                        if t_full_raw is not None and m_full_raw is not None:
                            t_full_vals = pd.to_numeric(t_full_raw, errors='coerce').fillna(0).values
                            m_full_vals = pd.to_numeric(m_full_raw, errors='coerce').fillna(0).values

                            t_start_offset = t_arr[0]
                            d['time_full'] = (t_full_vals - t_start_offset) / 60.0
                            
                            # CRITICAL: Use the SAME 'm_frost_min' calculated above!
                            d['m_frost_full'] = (m_full_vals - m_frost_min) * 1000.0

                    entry['data'] = d
                    entry['avail'] = True

                    # Update Scaling
                    mask_exp_stable = (t_exp >= t_cut_start) & (t_exp <= t_cut_end)
                    for k in d:
                        if k in scaling_data and np.any(mask_exp_stable):
                            scaling_data[k].extend(d[k][mask_exp_stable])

            except Exception as e:
                print(f"Error loading {exp_id}: {e}")
            
            experiments.append(entry)
        
        # =========================================================================
        # 2. PLOTTING INFRASTRUCTURE
        # =========================================================================
        fig = plt.figure(figsize=(24, 13))
        if group_name:
            title_text = f"Unified Dashboard ({experiment_name}): {group_name}"
        else:
            title_text = f"Unified Dashboard ({experiment_name}): {len(experiment_ids)} Experiments vs Simulation"

        fig.suptitle(title_text, fontsize=20, fontweight='bold', y=0.96)

        # Top Half: Inputs 
        gs_top = gridspec.GridSpec(2, 3, figure=fig, 
                                top=0.90, bottom=0.45, hspace=0.35, wspace=0.20)
        
        # Bottom Half: Outputs
        gs_bot = gridspec.GridSpec(2, 4, figure=fig, height_ratios=[3, 1.2], 
                                top=0.38, bottom=0.05, hspace=0.0, wspace=0.20)

        # Helper: Cutoff Visualization
        def add_cutoff_vis(ax, show_labels=False):
            ax.axvspan(0, t_cut_start, color='gray', alpha=0.15, lw=0)
            ax.axvline(t_cut_start, color='gray', linestyle=':', linewidth=1)
            ax.axvspan(t_cut_end, t_total, color='gray', alpha=0.15, lw=0)
            ax.axvline(t_cut_end, color='gray', linestyle=':', linewidth=1)
            if show_labels:
                ax.text(t_cut_start/2, ax.get_ylim()[1], "Start", ha='center', va='bottom', fontsize=8, color='gray', fontstyle='italic')

        # Helper: Apply Smart Scaling (For Main Plots)
        def apply_smart_scaling(ax, key):
            # 1. SIMULATION
            sim_vals = sim_data.get(key, [])
            sim_vals = sim_vals[~np.isnan(sim_vals)]

            if len(sim_vals) > 0:
                final_min = np.min(sim_vals)
                final_max = np.max(sim_vals)
            else:
                final_min = np.inf
                final_max = -np.inf

            # 2. EXPERIMENTS
            all_exp_vals = []
            for exp in experiments:
                if exp.get('avail') and key in exp.get('data', {}):
                    all_exp_vals.extend(exp['data'][key])
            
            all_exp_vals = np.array(all_exp_vals)
            all_exp_vals = all_exp_vals[~np.isnan(all_exp_vals)]

            if len(all_exp_vals) > 0:
                exp_p_min = np.percentile(all_exp_vals, 1.0) 
                exp_p_max = np.percentile(all_exp_vals, 99.0)

                if final_min == np.inf:
                    final_min = exp_p_min
                    final_max = exp_p_max
                else:
                    final_min = min(final_min, exp_p_min)
                    final_max = max(final_max, exp_p_max)

            # 3. APPLY
            if final_min != np.inf and final_max != -np.inf:
                data_range = final_max - final_min
                padding = 1.0 if data_range == 0 else data_range * 0.2
                ax.set_ylim(final_min - padding, final_max + padding)

        # =========================================================================
        # 3. PLOT INPUTS (TOP HALF)
        # =========================================================================
        input_map = [
            (0, 0, 'n_fan',      'Fan Speed',           '[rpm]'),
            (0, 1, 'T_in',       'Air Inlet Temp',     '[°C]'),
            (0, 2, 'rh_in',      'Air Inlet RH',       '[%]'),
            (1, 0, 'm_ref',      'Ref. Mass Flow',     '[kg/s]'),
            (1, 1, 'h_ref_in',   'Ref. Inlet Enthalpy','[kJ/kg]'),
            (1, 2, 'p_ref_in',   'Evap. Inlet Pressure','[bar]')
        ]

        for r, c, key, title, unit in input_map:
            ax = fig.add_subplot(gs_top[r, c])
            
            # Plot Experiments
            for idx, exp in enumerate(experiments):
                if exp['avail'] and key in exp['data']:
                    lbl = f"Exp {exp['id']}" if (r==0 and c==0) else ""
                    ax.plot(exp['time'], exp['data'][key], color=BLUE_GRADIENT[idx], lw=1.5, alpha=0.6, label=lbl)
            
            # Plot Sim
            if key == 'n_fan':
                ax.plot(t_sim, sim_data[key], color=c_sim, lw=2.5, drawstyle='steps-post',label="SIM" if (r==0 and c==0) else "")
            else:
                ax.plot(t_sim, sim_data[key], color=c_sim, lw=2.5, label="SIM" if (r==0 and c==0) else "")

            add_cutoff_vis(ax, show_labels=(r==0 and c==0))
            apply_smart_scaling(ax, key)
            ax.set_title(title, fontsize=11, fontweight='bold', color='#333333')
            ax.set_ylabel(unit, fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)
            
            if r==0 and c==0:
                ax.legend(fontsize=8, loc='best', framealpha=0.9)

        # =========================================================================
        # 4. PLOT OUTPUTS (BOTTOM HALF)
        # =========================================================================
        output_map = [
            ('dp',        'Air Pressure Drop',    'Pa'),
            ('h_ref_out', 'Ref. Outlet Enthalpy', 'kJ/kg'),
            ('Q',         'Total Heat Flow',      'W'),
            ('m_frost',   'Total Frost Mass',     'g')
        ]

        for col_idx, (key, title, unit) in enumerate(output_map):
            ax_main = fig.add_subplot(gs_bot[0, col_idx])
            ax_err  = fig.add_subplot(gs_bot[1, col_idx], sharex=ax_main)

            # Collection list for Smart Scaling of Errors
            all_err_values = []

            # --- 1. PLOT EXPERIMENTS ---
            for idx, exp in enumerate(experiments):
                if exp['avail'] and key in exp['data']:
                    c_exp = RED_GRADIENT[idx]
                    
                    # Special Handling for OptiHorst Frost Mass (Full Data + Plateau Error)
                    if experiment_name == "OptiHorst" and key == 'm_frost' and 'm_frost_full' in exp['data']:
                        
                        # --- A. PLOT THE DATA (The missing part) ---
                        t_full = exp['data']['time_full']
                        m_full = exp['data']['m_frost_full']
                        
                        # Plot the full curve (including the defrost/plateau part)
                        ax_main.plot(t_full, m_full, color=c_exp, lw=1.5, alpha=0.7)

                        # --- B. ERROR CALCULATION (Single Point at Cutoff) ---
                        t_marker = exp['time'][-1] 
                        t_extended = t_total * 1.5 
                        mask_plateau = (t_full >= t_marker) & (t_full <= t_extended)
                        
                        val_exp = np.max(m_full[mask_plateau]) if np.any(mask_plateau) else exp['data'][key][-1]
                        val_sim = np.interp(t_marker, t_sim, sim_data[key])
                        
                        if val_exp != 0:
                            err_val = (val_sim - val_exp) / val_exp * 100.0
                        else:
                            err_val = 0.0
                            
                        lbl_err = f"Exp {exp['id']} ({err_val:+.1f}%)"
                        ax_err.scatter(t_marker, err_val, color=c_exp, marker='X', s=80, 
                                     edgecolor='black', linewidth=0.5, zorder=10, label=lbl_err)
                        
                        ax_err.vlines(t_marker, 0, err_val, color=c_exp, linestyle=':', alpha=0.5)
                        all_err_values.append(err_val)

                    else:
                        # --- STANDARD PLOTTING (Continuous Line) ---
                        t_plot = exp['time']
                        y_plot = exp['data'][key]
                        
                        ax_main.plot(t_plot, y_plot, color=c_exp, lw=1.5, alpha=0.7)

                        # --- ERROR CALCULATION (Continuous) ---
                        # Interpolate SIM to EXP timestamps
                        sim_interp = np.interp(exp['time'], t_sim, sim_data[key])
                        abs_err = sim_interp - exp['data'][key]
                        
                        with np.errstate(divide='ignore', invalid='ignore'):
                            rel_err = (abs_err / exp['data'][key]) * 100.0
                        rel_err = np.nan_to_num(rel_err, nan=0.0, posinf=0.0, neginf=0.0)
                        
                        mask_exp_stable_err = (exp['time'] >= t_cut_start) & (exp['time'] <= t_cut_end)
                        if np.sum(mask_exp_stable_err) > 0:
                            avg_rel = np.mean(np.abs(rel_err[mask_exp_stable_err]))
                            all_err_values.extend(rel_err[mask_exp_stable_err])
                        else:
                            avg_rel = 0.0
                        
                        lbl_err = f"Exp {exp['id']} (Ø{avg_rel:.1f}%)"
                        ax_err.plot(exp['time'], rel_err, color=c_exp, lw=1.2, label=lbl_err)

            # --- 2. PLOT SIMULATION ---
            lbl_sim = "SIM Out"
            ax_main.plot(t_sim, sim_data[key], color=c_sim, lw=2.5, zorder=10, label=lbl_sim)

            # --- 3. FORMATTING & OPTIHORST VISUALS ---
            apply_smart_scaling(ax_main, key)
            
            if experiment_name == "OptiHorst" and key == 'm_frost':
                t_extended = t_total * 1.5
                ax_main.set_xlim(0, t_extended)
                
                ax_main.axvspan(t_total, t_extended, color='#444444', alpha=0.15, hatch='//', edgecolor='gray')
                ax_main.axvline(t_total, color='black', linestyle='-', linewidth=1.5)
                
                for idx, exp in enumerate(experiments):
                    if exp['avail'] and 'm_frost_full' in exp['data']:
                        t_end_mode1 = exp['time'][-1]
                        
                        t_full = exp['data']['time_full']
                        m_full = exp['data']['m_frost_full']
                        mask_plateau = (t_full >= t_end_mode1) & (t_full <= t_extended)
                        
                        if np.any(mask_plateau):
                            final_val = np.max(m_full[mask_plateau])
                            
                            ax_main.plot([t_end_mode1, t_end_mode1], [0, final_val], 
                                       color='darkgray', linestyle='-', linewidth=0.8, zorder=20)
                            ax_main.plot([t_end_mode1, t_extended], [final_val, final_val], 
                                       color='darkgray', linestyle='-', linewidth=0.8, zorder=20)
                            
                            ax_main.scatter(t_end_mode1, final_val, 
                                          marker='X', s=120, color=RED_GRADIENT[idx], 
                                          edgecolor='black', zorder=25,
                                          label=f'Plateau Exp {exp["id"]}')

            ax_main.set_title(title, fontsize=11, fontweight='bold', color='#333333')
            ax_main.set_ylabel(f"[{unit}]", fontsize=9)
            ax_main.grid(True, linestyle=':', alpha=0.6)
            plt.setp(ax_main.get_xticklabels(), visible=False)
            
            if col_idx == 0: 
                ax_main.legend(fontsize=8, loc='best')

            # --- 4. FORMATTING ERROR AXIS ---
            ax_err.axhline(0, color='gray', linestyle='--', linewidth=1)
            
            if len(all_err_values) > 0:
                vals = np.array(all_err_values)
                vals = vals[~np.isnan(vals) & ~np.isinf(vals)]
                
                if len(vals) > 0:
                    p_min = np.percentile(vals, 1.0)
                    p_max = np.percentile(vals, 99.0)
                    
                    if p_min > -5.0: p_min = -5.0
                    if p_max < 5.0: p_max = 5.0
                    
                    rng = p_max - p_min
                    pad = rng * 0.15
                    ax_err.set_ylim(p_min - pad, p_max + pad)

            ax_err.set_ylabel("Err [%]", fontsize=8)
            ax_err.set_xlabel("Time [min]", fontsize=10)
            ax_err.grid(True, linestyle=':', alpha=0.6)
            
            if ax_err.get_legend_handles_labels()[0]:
                ax_err.legend(fontsize=7, loc='best', framealpha=0.9)

        # =========================================================================
        # 5. SAVE / SHOW
        # =========================================================================
        if save_fig:
            if group_name:
                safe_name = str(group_name).replace("°", "deg").replace(" ", "_").replace("(", "").replace(")", "")
                out_name = f"Unified_Dashboard_{experiment_name}_{safe_name}.png"
            else:
                out_name = f"Unified_Dashboard_{experiment_name}_{len(experiment_ids)}_Experiments.png"

            out_path = os.path.join(path_exp, "graphics", "Final_Comparison_Unified", out_name)
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            plt.savefig(out_path, dpi=300)
            print(f"Saved unified figure to {out_path}")
            plt.close()
        else:
            plt.show()

    def calculate_errors(self, states_history, inputs_history, experiment_id, path_exp, 
                         cutoff_pct=0.02, experiment_name="OptiAbt"):
        """
        Calculates error values between 1 Simulation and 1 Experiment.
        Returns a dictionary of error percentages.
        """

        # =========================================================================
        # 0. CONFIGURATION
        # =========================================================================
        if experiment_name == "OptiHorst":
            MAPPING = COLUMN_MAPPING_ObtiHorst # Ensure these are defined in class/global
        elif experiment_name == "OptiAbt":
            MAPPING = COLUMN_MAPPING_OptiAbt
        else:
            raise ValueError(f"Unknown experiment_name: {experiment_name}")

        # =========================================================================
        # 1. DATA PREPARATION (SIMULATION)
        # =========================================================================
        steps = len(states_history)
        t_sim = np.array([i * self.params.time_step / 60.0 for i in range(steps)])
        t_total = t_sim[-1]

        # Cutoff timestamps for stability calculation
        t_cut_start = t_total * cutoff_pct
        t_cut_end = t_total * (1.0 - cutoff_pct)
        mask_sim_stable = (t_sim >= t_cut_start) & (t_sim <= t_cut_end)

        sim_data = {
            'dp': [], 'h_ref_out': [], 'Q': [], 'm_frost': []
        }

        num_layers = len(states_history[0])
        reg_amount = self.params.register_amount

        for t in range(steps):
            # Calculate total outputs for this timestep
            dp_total = 0; q_total = 0; mass_frost_total = 0
            
            for k in range(num_layers):
                s = states_history[t][k]
                dp_total += s.air.pressure_drop
                q_total += s.hmt.Q_dot_total
                mass_frost_total += (s.frost.mass * 1000.0 * reg_amount)
            
            sim_data['dp'].append(dp_total)
            sim_data['Q'].append(q_total * reg_amount)
            sim_data['m_frost'].append(mass_frost_total)
            sim_data['h_ref_out'].append(states_history[t][0].refrigerant.h_out / 1e3)

        # Convert to Numpy
        for k in sim_data:
            sim_data[k] = np.array(sim_data[k])

        # =========================================================================
        # 2. EXPERIMENT DATA LOADING (SINGLE ID)
        # =========================================================================
        exp_data = {}
        
        f_data = os.path.join(path_exp, f"{experiment_id}_data.csv")
        f_kk = os.path.join(path_exp, f"{experiment_id}_data_KK.csv")

        if not os.path.exists(f_data):
            return {"error": f"File not found for {experiment_id}"}

        # Load Data
        df = pd.read_csv(f_data, sep=';', decimal='.', on_bad_lines='skip', low_memory=False)
        if os.path.exists(f_kk):
            df_kk = pd.read_csv(f_kk, sep=';', decimal='.', low_memory=False)
        else:
            df_kk = pd.DataFrame(index=df.index)

        # --- A. Robust Mode Detection ---
        modus_col = self._get_data_col(df, df_kk, MAPPING, 'modus')
        mask_mode = None

        if modus_col is not None:
            modus_num = pd.to_numeric(modus_col, errors='coerce').fillna(0)
            if modus_num.max() > 0:
                mask_mode = (modus_num == 1)

        if mask_mode is None or mask_mode.sum() == 0:
            fan_raw = self._get_data_col(df, df_kk, MAPPING, 'n_fan')
            if fan_raw is not None:
                fan_vals = pd.to_numeric(fan_raw, errors='coerce').fillna(0)
                is_running = fan_vals > 1 
                # Find longest continuous run
                group_ids = (is_running != is_running.shift()).cumsum()
                if is_running.any():
                    active_groups = group_ids[is_running]
                    longest_group = active_groups.value_counts().idxmax()
                    mask_mode = (group_ids == longest_group)
                else:
                    mask_mode = pd.Series(False, index=df.index)
            else:
                mask_mode = pd.Series(True, index=df.index)

        # --- B. Cut & Process ---
        valid_idx = df.index[mask_mode]
        if not df_kk.empty:
            valid_idx = valid_idx.intersection(df_kk.index)

        # Fallback if filter destroys data
        if len(valid_idx) < 10:
             valid_idx = df.index

        df_cut = df.loc[valid_idx].copy()
        df_kk_cut = df_kk.loc[valid_idx].copy()

        # Time Normalization
        t_raw = self._get_data_col(df_cut, df_kk_cut, MAPPING, 'time')
        if t_raw is not None:
            t_arr = pd.to_numeric(t_raw, errors='coerce').values
        else:
            t_arr = df_cut.index.values * 1.0
        
        t_exp = (t_arr - t_arr[0]) / 60.0 # Normalized time axis
        exp_data['time'] = t_exp

        # Helper to extract and clean columns
        def get_val(key, default_val=0.0):
            v = self._get_data_col(df_cut, df_kk_cut, MAPPING, key)
            return pd.to_numeric(v, errors='coerce').fillna(default_val).values if v is not None else np.zeros(len(df_cut))

        # Extract Standard Variables
        exp_data['dp'] = get_val('dp')
        
        # Enthalpy Unit Check
        h_out = get_val('h_ref_out')
        exp_data['h_ref_out'] = h_out / 1000.0 if np.mean(h_out) > 10000 else h_out

        # Q Calculation
        m_ref_raw = get_val('m_ref')
        m_ref = m_ref_raw / 1000.0 if np.mean(m_ref_raw) > 0.5 else m_ref_raw
        h_in_raw = get_val('h_ref_in')
        h_in = h_in_raw / 1000.0 if np.mean(h_in_raw) > 10000 else h_in_raw
        
        q_kw = m_ref * (exp_data['h_ref_out'] - h_in)
        exp_data['Q'] = q_kw * 1000.0

        # =========================================================
        # FROST MASS: SMART ZEROING (Regression 10-15%)
        # =========================================================
        from scipy.stats import linregress
        
        m_frost_raw = get_val('mass_raw')
        m_frost_min = 0.0 # Initialize safety default

        if len(m_frost_raw) > 0:
            # 1. Define Window (10-15% of SIMULATION time)
            #    t_total comes from the simulation time array at the top of the function
            mask_reg = (t_exp >= t_total * 0.10) & (t_exp <= t_total * 0.15)
            
            # 2. Calculate Intercept (Virtual Zero)
            if np.sum(mask_reg) > 10:
                res = linregress(t_exp[mask_reg], m_frost_raw[mask_reg])
                m_frost_min = res.intercept 
            else:
                # Fallback to min if window is empty/too small
                m_frost_min = m_frost_raw.min()

            d['m_frost'] = (m_frost_raw - m_frost_min) * 1000.0
        else:
            d['m_frost'] = np.zeros(len(df_cut))

        # =========================================================
        # OPTIHORST: FULL DATA LOADING (For Defrost/Plateau Check)
        # =========================================================
        if experiment_name == "OptiHorst":
            t_full_raw = self._get_data_col(df, df_kk, MAPPING, 'time')
            m_full_raw = self._get_data_col(df, df_kk, MAPPING, 'mass_raw')

            if t_full_raw is not None and m_full_raw is not None:
                t_full_vals = pd.to_numeric(t_full_raw, errors='coerce').fillna(0).values
                m_full_vals = pd.to_numeric(m_full_raw, errors='coerce').fillna(0).values

                t_start_offset = t_arr[0]
                d['time_full'] = (t_full_vals - t_start_offset) / 60.0
                
                # CRITICAL: Use the SAME 'm_frost_min' calculated above!
                d['m_frost_full'] = (m_full_vals - m_frost_min) * 1000.0

        # =========================================================================
        # 3. CALCULATE ERRORS (Interpolating Simulation to Experiment Time)
        # =========================================================================
        error_results = {}
        target_keys = ['dp', 'h_ref_out', 'Q', 'm_frost']
        
        # Define stable mask for the experimental timeline
        t_exp = exp_data['time']
        t_exp_total = t_exp[-1]
        mask_exp_stable = (t_exp >= t_exp_total * cutoff_pct) & \
                          (t_exp <= t_exp_total * (1.0 - cutoff_pct))

        for key in target_keys:
            # --- SPECIAL CASE: OptiHorst Frost Mass Plateau Check ---
            if experiment_name == "OptiHorst" and key == 'm_frost' and 'm_frost_full' in exp_data:
                t_marker = t_exp[-1]
                t_full = exp_data['time_full']
                m_full = exp_data['m_frost_full']
                t_extended = t_total * 1.5 
                
                mask_plateau = (t_full >= t_marker) & (t_full <= t_extended)
                val_exp = np.max(m_full[mask_plateau]) if np.any(mask_plateau) else exp_data[key][-1]
                
                # Interpolate simulation to the specific marker point
                val_sim = np.interp(t_marker, t_sim, sim_data[key])
                
                err_val = ((val_sim - val_exp) / val_exp * 100.0) if val_exp != 0 else 0.0
                error_results[key] = err_val

            # --- STANDARD CASE: Time-Series Average Error ---
            else:
                # INTERPOLATE SIMULATION TO EXPERIMENT TIME
                # sim_data[key] is projected onto t_exp
                sim_interp = np.interp(t_exp, t_sim, sim_data[key])
                
                # Actual values from experiment
                val_actual = exp_data[key]
                
                abs_err = sim_interp - val_actual
                
                with np.errstate(divide='ignore', invalid='ignore'):
                    # Error relative to experimental ground truth
                    rel_err = (abs_err / val_actual) * 100.0
                
                rel_err = np.nan_to_num(rel_err, nan=0.0, posinf=0.0, neginf=0.0)
                
                # Calculate Mean Absolute Percentage Error (MAPE) over experimental stable region
                if np.sum(mask_exp_stable) > 0:
                    avg_rel = np.mean(np.abs(rel_err[mask_exp_stable]))
                else:
                    avg_rel = 0.0
                
                error_results[key] = avg_rel

        return error_results



    def plot_detailed(self, states_history, inputs_history, experiment_ids):
        """
        Creates a detailed 3x5 Subplot (Landscape) based on simulation data.
        
        Refinements:
        - Plots System Inlet once ('Sys In').
        - Plots Layer Outlets explicitly labeled 'L{k} Out'.
        - distinct legends for Flow variables vs Layer properties.
        """
        
        # --- 1. Data Pre-processing ---
        
        time_steps = len(states_history)
        time_axis = np.array([i * self.params.time_step / 60.0 for i in range(time_steps)])
        num_layers = len(states_history[0])
        
        # --- COLOR SETUP ---
        if num_layers <= 10:
            cmap = plt.get_cmap('tab10')
            colors = [cmap(i) for i in range(num_layers)]
        else:
            cmap = plt.get_cmap('turbo')
            colors = cmap(np.linspace(0, 1, num_layers))
        
        # Initialize Data Containers
        data = {
            'air_dp_layer': [], 'air_T_in': [], 'air_T_out': [], 'air_RH_in': [], 'air_RH_out': [], 'air_h_conv': [],
            'ref_h_in': [], 'ref_h_out': [], 'ref_T_in': [], 'ref_T_out': [], 
            'ref_Q': [], 'ref_portion_sh': [], 'ref_h_conv_eff': [],
            'frost_T_surf': [], 'frost_density': [], 'frost_mass': [], 'frost_thickness': [],
            'Q_sens_layer': [], 'Q_lat_layer': [], 'Q_tot_layer': []
        }

        # Extract Data
        for k in range(num_layers):
            dp, T_a_in, T_a_out, rh_in, rh_out, h_c_air = [], [], [], [], [], []
            h_r_in, h_r_out, T_r_in, T_r_out, h_c_ref = [], [], [], [], []
            T_f_surf, rho_f, m_f, th_f = [], [], [], []
            q_s, q_l, q_t = [], [], []
            port_sh = []
            
            for t in range(time_steps):
                s = states_history[t][k]
                inp = inputs_history[t][k]
                
                # --- Air ---
                dp.append(s.air.pressure_drop)
                T_a_in.append(inp.air.T_in - 273.15)
                T_a_out.append(s.air.T_out - 273.15)
                rh_in.append(s.air.R_in * 100)
                rh_out.append(s.air.R_out * 100)
                h_c_air.append(s.air.h_conv)
                
                # --- Refrigerant ---
                h_r_in.append(inp.refrigerant.h_in / 1000.0)
                h_r_out.append(s.refrigerant.h_out / 1000.0)
                T_r_in.append(s.refrigerant.T_in - 273.15)
                T_r_out.append(s.refrigerant.T_out - 273.15)

                port_sh.append(s.refrigerant.portion_superheated)

                # h_conv weighted
                h_eff = (s.refrigerant.portion_two_phase * s.refrigerant.h_conv_two_phase + 
                        s.refrigerant.portion_superheated * s.refrigerant.h_conv_superheated)
                h_c_ref.append(h_eff)
                
                # --- Frost / Heat ---
                T_f_surf.append(s.hmt.T_frost_surface - 273.15)
                rho_f.append(s.frost.density)
                m_f.append(s.frost.mass * 1000.0 * self.params.register_amount)
                th_f.append(s.frost.thickness * 1000.0)
                
                q_t.append(s.hmt.Q_dot_total * self.params.register_amount)
                q_s.append(s.hmt.Q_dot_sens * self.params.register_amount)
                q_l.append((s.hmt.Q_dot_total - s.hmt.Q_dot_sens) * self.params.register_amount)

            data['air_dp_layer'].append(dp)
            data['air_T_in'].append(T_a_in)
            data['air_T_out'].append(T_a_out)
            data['air_RH_in'].append(rh_in)
            data['air_RH_out'].append(rh_out)
            data['air_h_conv'].append(h_c_air)
            data['ref_h_in'].append(h_r_in)
            data['ref_h_out'].append(h_r_out)
            data['ref_T_in'].append(T_r_in)
            data['ref_T_out'].append(T_r_out)
            data['ref_portion_sh'].append(port_sh)
            data['ref_h_conv_eff'].append(h_c_ref)
            data['frost_T_surf'].append(T_f_surf)
            data['frost_density'].append(rho_f)
            data['frost_mass'].append(m_f)
            data['frost_thickness'].append(th_f)
            data['Q_sens_layer'].append(q_s)
            data['Q_lat_layer'].append(q_l)
            data['Q_tot_layer'].append(q_t)

        # Global Calculations
        air_dp_sum = np.sum(data['air_dp_layer'], axis=0)
        
        air_vol_flow = []
        for t in range(time_steps):
            s0 = states_history[t][0]
            vol = s0.air.v_dot_fan_m3h_segment* self.params.register_amount / self.params.fan_amount
            air_vol_flow.append(vol)
            
        km_m_dot = inputs_history[0][0].refrigerant.m_dot * self.params.register_amount
        frost_mass_sum = np.sum(data['frost_mass'], axis=0) 
        Q_sens_sum = np.sum(data['Q_sens_layer'], axis=0)
        Q_lat_sum = np.sum(data['Q_lat_layer'], axis=0)
        Q_tot_sum = np.sum(data['Q_tot_layer'], axis=0)


        # --- 2. Plotting Setup ---
        
        plt.rcParams.update({'font.size': 10})
        fig, axs = plt.subplots(3, 5, figsize=(32, 18))
        fig.suptitle(f"Simulation Results: {experiment_ids}", fontsize=20, fontweight='bold')
        
        # Helper to standardize formatting and force legend
        def format_ax(ax, title, y_label, x_label="Time [min]"):
            ax.set_title(title, fontsize=11, fontweight='bold')
            ax.set_ylabel(y_label, fontsize=10)
            ax.set_xlabel(x_label, fontsize=10)
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.tick_params(axis='both', labelsize=9)
            ax.legend(fontsize=8, loc='best', framealpha=0.8)

        # --- ROW 1: AIR (axs[0, :]) ---
        
        # 1.1 Air Pressure Drop
        ax = axs[0, 0]
        ax.plot(time_axis, air_dp_sum, label='Total', color='black', linestyle='--', linewidth=1.5)
        for k in range(num_layers):
            ax.plot(time_axis, data['air_dp_layer'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Air Pressure Drop", "$\Delta p$ [Pa]")

        # 1.2 Air Volume Flow
        ax = axs[0, 1]
        ax.plot(time_axis, air_vol_flow, label=None, color='black', linewidth=1.5)
        format_ax(ax, "Air Volume Flow", "$\dot{V}$ [m³/h]")
        ax.set_ylim(0, max(air_vol_flow)*1.2)

        # 1.3 Air Temp (In & Out)
        ax = axs[0, 2]
        ax.plot(time_axis, data['air_T_in'][0], label='Sys In', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        for k in range(num_layers):
            # Explicit Label: L1 Out, L2 Out...
            ax.plot(time_axis, data['air_T_out'][k], label=f'L{k+1} Out', color=colors[k])
        format_ax(ax, "Air Temperatures", "T [°C]")

        # 1.4 Air RH
        ax = axs[0, 3]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['air_RH_out'][k], label=f'L{k+1} Out', color=colors[k])
        ax.plot(time_axis, data['air_RH_in'][0], label='Sys In', color='black', linestyle='--', linewidth=1.5, alpha=0.7)

        format_ax(ax, "Air Rel. Humidity (Out)", "RH [%]")

        # 1.5 Air HTC
        ax = axs[0, 4]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['air_h_conv'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Air HTC ($h_{conv}$)", "h [W/m²K]")


        # --- ROW 2: REFRIGERANT (axs[1, :]) ---

        # 2.1 Ref Mass Flow
        ax = axs[1, 0]
        ax.axhline(y=km_m_dot, label=None, color='black', linestyle='-', linewidth=1.5)
        format_ax(ax, "Refrigerant Mass Flow", "$\dot{m}$ [kg/s]")
        ax.set_ylim(0, 0.08)

        # 2.2 Ref Enthalpy
        ax = axs[1, 1]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_h_out'][k], label=f'L{k+1} IN (reverse)', linestyle='-', color=colors[k])
        ax.plot(time_axis, data['ref_h_in'][-1], label='Sys In (KM-side)', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        format_ax(ax, "Refrigerant Enthalpy", "h [kJ/kg]")
        
        # 2.3 Ref Temperature
        ax = axs[1, 2]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_T_out'][k], linestyle='-', color=colors[k], label=f'L{k+1} Out')
        ax.plot(time_axis, data['ref_T_in'][-1], label='Sys In (KM-side)', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        format_ax(ax, "Refrigerant Temperature", "T [°C]")

        # 2.4 Ref Portion Superheated (Replaces Quality)
        ax = axs[1, 3]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_portion_sh'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Ref. Portion Superheated", "Portion [-]")
        ax.set_ylim(-0.05, 1.05)

        # 2.5 Ref HTC
        ax = axs[1, 4]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_h_conv_eff'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Ref. HTC (Weighted)", "h [W/m²K]")


        # --- ROW 3: FROST & ENERGY (axs[2, :]) ---

        # 3.1 Heat Flows
        ax = axs[2, 0]
        ax.plot(time_axis, Q_tot_sum, label='Total', color='black', linewidth=1.5)
        ax.plot(time_axis, Q_sens_sum, label='Sensible', color='gray', linestyle='--')
        ax.plot(time_axis, Q_lat_sum, label='Latent', color='gray', linestyle='-.')
        format_ax(ax, "Heat Flows (Sum)", "$\dot{Q}$ [W]")

        # 3.2 Frost Surf Temp
        ax = axs[2, 1]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['frost_T_surf'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Surf. Temp.", "T [°C]")

        # 3.3 Frost Density
        ax = axs[2, 2]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['frost_density'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Density", "$\\rho$ [kg/m³]")

        # 3.4 Frost Mass
        ax = axs[2, 3]
        ax.plot(time_axis, frost_mass_sum, label='Sum', color='black', linestyle='--', linewidth=1.5)
        for k in range(num_layers):
            ax.plot(time_axis, data['frost_mass'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Mass", "m [g]")

        # 3.5 Frost Thickness
        ax = axs[2, 4]
        for k in range(num_layers):
            ax.plot(time_axis, data['frost_thickness'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Thickness", "s [mm]")

        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        plt.show()

    def plot_roughness(self, states_history, experiment_id):
        """
        Plots the Air Roughness Multiplier and its Square Root side-by-side.
        """
        # 1. Setup Time and Dimensions
        time_steps = len(states_history)
        num_layers = len(states_history[0])
        time_axis = np.array([i * self.params.time_step / 60.0 for i in range(time_steps)])

        # 2. Initialize Subplots (1 row, 2 columns)
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # 3. Plotting Logic
        for k in range(num_layers):
            # Extract base data for the current layer
            raw_data = [states_history[t][k].air.roughness_multiplier for t in range(time_steps)]
            
            # Left Plot: Pressure Loss (Original)
            ax1.plot(time_axis, raw_data, label=f'Layer {k+1}')
            
            # Right Plot: HMT (Square Root)
            hmt_data = [val**0.5 for val in raw_data]
            ax2.plot(time_axis, hmt_data, label=f'Layer {k+1}')

        # 4. Formatting Left Plot
        ax1.set_title(f"Air Roughness Multiplier\nPressure Loss", fontsize=12, fontweight='bold')
        ax1.set_xlabel("Time [min]")
        ax1.set_ylabel("Multiplier [-]")
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend(loc='best', fontsize='small')

        # 5. Formatting Right Plot
        ax2.set_title(f"Air Roughness Multiplier\nHMT", fontsize=12, fontweight='bold')
        ax2.set_xlabel("Time [min]")
        ax2.set_ylabel("Multiplier [-]")
        ax2.grid(True, linestyle=':', alpha=0.6)
        ax2.legend(loc='best', fontsize='small')

        plt.tight_layout()
        plt.show()

    def plot_frost_distribution(self, states_history, time_step_index=-1, num_fins_to_show=6):
        """
        Generates a scaled, top-down/side-section visualization of frost growth 
        on the heat exchanger.
        """
        
        # --- 1. Retrieve Data and Setup Dimensions ---
        if not states_history:
            print("No data to visualize.")
            return

        # Get the list of states for the requested time step (default: last step)
        current_states = states_history[time_step_index]
        current_time_min = time_step_index * self.params.time_step / 60.0
        
        mm = 1000.0
        
        # Dimensions from self.params (converted to mm)
        layer_count = len(current_states)
        
        # "fin_length" in self.params is the depth of one layer in the flow direction
        layer_depth = self.params.fin_length * mm 
        
        # Tube dimensions
        tube_width = self.params.tube_outer_diameter * mm
        
        # Fin dimensions
        fin_pitch = self.params.fin_pitch * mm
        fin_thickness_mm = self.params.fin_thickness * mm
        
        # Calculate total plot dimensions
        total_length = layer_depth * layer_count
        total_height = num_fins_to_show * fin_pitch
        
        # Create Figure
        fig, ax = plt.subplots(figsize=(16, 8), dpi=500)
        ax.set_aspect('equal') # Crucial for physically correct proportions
        ax.set_facecolor('white')
        
        # --- Colors ---
        c_tube = '#C88A45'          # Copper
        c_tube_border = '#8B5A2B'   # Darker copper outline
        c_fin = '#101010'           # Almost Black for fins
        c_frost_fin = '#81D4FA'     # Light Blue for Fin Frost
        c_frost_tube = '#4FC3F7'    # Slightly darker Blue for Tube Frost
        c_frost_edge = '#0288D1'    # Darker blue edge for definition
        
        # --- 2. Draw Geometry Loop ---
        
        # Y-positions for fins (centered vertically)
        y_start = (total_height / 2) - (num_fins_to_show * fin_pitch / 2) + (fin_pitch/2)
        y_positions = [y_start + i*fin_pitch for i in range(num_fins_to_show)]
        
        x_cursor = 0.0
        
        for i in range(layer_count):
            state = current_states[i]
            th_frost_mm = state.frost.thickness * mm
            
            # Center of the tube within this layer segment
            tube_center_x = x_cursor + (layer_depth / 2.0)
            
            # --- A. Draw Fins & Fin Frost ---
            for y in y_positions:
                # 1. Fin Frost (Background Layer)
                # Total height = fin + 2 * frost
                frost_h = fin_thickness_mm + (2 * th_frost_mm)
                
                # UPDATED: Added edgecolor and linewidth here
                rect_frost_fin = patches.Rectangle(
                    (x_cursor, y - frost_h/2), 
                    layer_depth, frost_h,
                    linewidth=0.8,              # Added width
                    edgecolor=c_frost_edge,     # Added color
                    facecolor=c_frost_fin, 
                    alpha=0.6, 
                    zorder=1
                )
                ax.add_patch(rect_frost_fin)
                
                # 2. The Fin Metal (Foreground Lines)
                # Drawn at Z=4 to ensure it sits ON TOP of the tube frost (Z=2)
                rect_fin = patches.Rectangle(
                    (x_cursor, y - fin_thickness_mm/2),
                    layer_depth, fin_thickness_mm,
                    linewidth=0, facecolor=c_fin, zorder=4
                )
                ax.add_patch(rect_fin)

            # --- B. Draw Tubes & Tube Frost ---
            
            # 1. Tube Frost (Vertical Bar)
            tube_frost_w = tube_width + (2 * th_frost_mm)
            
            # UPDATED: Added edgecolor and linewidth here
            rect_tube_frost = patches.Rectangle(
                (tube_center_x - tube_frost_w/2, -fin_pitch), 
                tube_frost_w, total_height + fin_pitch*2,
                linewidth=0.8,              # Added width
                edgecolor=c_frost_edge,     # Added color
                facecolor=c_frost_tube, 
                alpha=0.5, 
                zorder=2
            )
            ax.add_patch(rect_tube_frost)
            
            # Optional: Add dashed outline for tube frost
            # (Kept this as is, but you might find you don't need it now that the main block has an edge)
            rect_tube_frost_outline = patches.Rectangle(
                (tube_center_x - tube_frost_w/2, -fin_pitch), 
                tube_frost_w, total_height + fin_pitch*2,
                linewidth=0.5, edgecolor=c_frost_edge, facecolor='none', linestyle='--', zorder=3
            )
            ax.add_patch(rect_tube_frost_outline)
            
            # 2. The Tube Metal (Copper Block)
            rect_tube = patches.Rectangle(
                (tube_center_x - tube_width/2, -fin_pitch),
                tube_width, total_height + fin_pitch*2,
                linewidth=1.0, edgecolor=c_tube_border, facecolor=c_tube, zorder=5
            )
            ax.add_patch(rect_tube)
            
            # --- C. Annotations ---
            # Layer Number
            # Positioned slightly higher (+4) to prevent overlap with frost
            ax.text(x_cursor + layer_depth/2, 50, 
                    f"L{i+1}", 
                    ha='center', va='bottom', fontsize=25, fontweight='bold', color='#555', zorder=100)

            # Frost Thickness Value on the tube
            txt = ax.text(tube_center_x, total_height / 2.0, 
                        f"{th_frost_mm:.2f}\nmm", 
                        ha='center', va='center', fontsize=9, fontweight='bold', color='white', zorder=20)
            txt.set_path_effects([path_effects.Stroke(linewidth=2, foreground='#333'),
                                path_effects.Normal()])

            # Advance cursor
            x_cursor += layer_depth

        # --- 3. Draw Air Flow Arrow ---
        arrow_x_start = -18
        arrow_x_end = -3
        arrow_y = total_height / 2
        
        ax.annotate("", 
                    xy=(arrow_x_end, arrow_y), xycoords='data',
                    xytext=(arrow_x_start, arrow_y), textcoords='data',
                    arrowprops=dict(facecolor='#0277BD', edgecolor='none', width=10, headwidth=25, headlength=20),
                    zorder=30)
        
        ax.text((arrow_x_start+arrow_x_end)/2, arrow_y + 10, "AIR", 
                ha='center', va='bottom', color='#0277BD', fontweight='bold', fontsize=10, zorder=31)


        # --- 4. Formatting ---
        
        # FIX 1: Set Y-Limit higher to create headroom for labels so Title doesn't overlap
        ax.set_ylim(0, total_height + 15) 
        
        # FIX 2: Set View Limit to include the negative Arrow area...
        ax.set_xlim(-25, 150)
        
        # ...BUT restrict the visual Axis Line (Spine) to 0-150 only
        ax.spines['bottom'].set_bounds(0, 150)
        
        # ...AND ensure ticks start strictly at 0
        # Create ticks every 25mm up to 150mm
        ax.set_xticks(np.arange(0, 151, 25))

        ax.set_title(f"Frost Distribution (Schematic Top-Down View)", 
                    fontsize=14, fontweight='bold', pad=15)
        ax.set_xlabel("Depth along Airflow path [mm]", fontsize=11)
        
        # Clean up axes
        ax.set_yticks([]) 
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(False)
        
        # Legend
        legend_elements = [
            patches.Patch(facecolor=c_tube, edgecolor=c_tube_border, label='Tube (Refrigerant)'),
            patches.Patch(facecolor=c_fin, label='Fin (Aluminium)'),
            patches.Patch(facecolor=c_frost_fin, edgecolor=c_frost_edge, alpha=0.6, label='Frost Layer')
        ]
        ax.legend(handles=legend_elements, loc='upper left', frameon=True, fontsize=9)

        plt.tight_layout()
        plt.show()

    def plot_debug(self, states_history, inputs_history, experiment_id):
        """
        Erstellt einen massiven 8x4 Subplot (32 Graphen), um JEDE Variable zu visualisieren.
        Ziel: Identifikation von unsinnigen Werten oder Instabilitäten.
        """
        
        # --- 1. Datenaufbereitung ---
        time_steps = len(states_history)
        self.params.layer_amount = len(states_history[0])
        
        # Zeitachse in Minuten
        t_axis = np.array([i * self.params.time_step / 60.0 for i in range(time_steps)])
        
        # Farbschema für Layer (L1 bis LN)
        cmap = plt.get_cmap('jet') # 'jet' ist gut um Kontraste zwischen vielen Layern zu sehen
        colors = [cmap(i) for i in np.linspace(0, 1, self.params.layer_amount)]
        
        # Initialisiere Dictionary für Listen
        # Wir sammeln Listen von Arrays (Zeit x Layer)
        d = {
            # AIR
            'air_T_in': [], 'air_T_out': [], 'air_dp': [], 'air_vol': [],
            'air_RH_out': [], 'air_W_out': [], 'air_h_conv': [], 'air_Re': [],
            'air_v': [], 'air_m_dot_humid': [],
            
            # REFRIGERANT
            'ref_T_out': [], 'ref_p_out': [], 'ref_h_out': [], 'ref_x': [],
            'ref_h_conv': [], 'ref_portion_2p': [], 'ref_portion_sh': [],
            
            # FROST
            'fr_thick': [], 'fr_dens': [], 'fr_mass': [], 'fr_T_surf': [], 
            'fr_T_base': [], 'fr_k': [],
            
            # ENERGY / HMT
            'Q_tot': [], 'Q_sens': [], 'Q_lat': [], 'm_flux_thick': [], 'm_flux_dens': [],
            'eta_fin': [],
            
            # RESISTANCES (Wichtig für Instabilitäten!)
            'R_frost': [], 'R_air': [], 'R_ref': [], 'R_total': []
        }

        # Schleife über Zeit und Layer zum Extrahieren
        # Wir bauen Listen: d['key'][layer_index][time_index]
        
        # Vorbelegen der Listen pro Layer
        for key in d:
            d[key] = [[] for _ in range(self.params.layer_amount)]

        sys_vol_flow = [] # Globaler Volumenstrom
        sys_m_dot_ref = [] # Globaler Massenstrom KM
        
        for t in range(time_steps):
            # Global Values
            sys_vol_flow.append(states_history[t][0].air.v_dot_fan_m3h_segment * self.params.register_amount)
            sys_m_dot_ref.append(inputs_history[t][0].refrigerant.m_dot * self.params.register_amount)
            
            for k in range(self.params.layer_amount):
                s = states_history[t][k]
                inp = inputs_history[t][k]
                
                # --- Air ---
                d['air_T_in'][k].append(inp.air.T_in - 273.15)
                d['air_T_out'][k].append(s.air.T_out - 273.15)
                d['air_dp'][k].append(s.air.pressure_drop)
                d['air_vol'][k].append(s.air.v_dot_fan_m3h_segment) # Per segment/layer
                d['air_RH_out'][k].append(s.air.R_out * 100)
                d['air_W_out'][k].append(s.air.W_out * 1000) # g/kg
                d['air_h_conv'][k].append(s.air.h_conv)
                d['air_Re'][k].append(s.air.reynolds)
                d['air_v'][k].append(s.air.velocity)
                d['air_m_dot_humid'][k].append(s.air.m_dot_humid)

                # --- Refrigerant ---
                d['ref_T_out'][k].append(s.refrigerant.T_out - 273.15)
                d['ref_p_out'][k].append(s.refrigerant.p_out / 1e5) # Bar
                d['ref_h_out'][k].append(s.refrigerant.h_out / 1000) # kJ/kg
                d['ref_x'][k].append(s.refrigerant.quality_avg)
                
                # Weighted HTC
                h_eff = (s.refrigerant.portion_two_phase * s.refrigerant.h_conv_two_phase + 
                        s.refrigerant.portion_superheated * s.refrigerant.h_conv_superheated)
                d['ref_h_conv'][k].append(h_eff)
                d['ref_portion_2p'][k].append(s.refrigerant.portion_two_phase)

                d['ref_portion_sh'][k].append(s.refrigerant.portion_superheated)
                
                # --- Frost ---
                d['fr_thick'][k].append(s.frost.thickness * 1000) # mm
                d['fr_dens'][k].append(s.frost.density)
                d['fr_mass'][k].append(s.frost.mass * 1000 * self.params.register_amount) # g (scaled)
                d['fr_T_surf'][k].append(s.hmt.T_frost_surface - 273.15)
                d['fr_T_base'][k].append(s.hmt.T_frost_base - 273.15)
                d['fr_k'][k].append(s.frost.k_frost)
                
                # --- HMT & Energy ---
                d['Q_tot'][k].append(s.hmt.Q_dot_total * self.params.register_amount)
                d['Q_sens'][k].append(s.hmt.Q_dot_sens * self.params.register_amount)
                d['Q_lat'][k].append((s.hmt.Q_dot_total - s.hmt.Q_dot_sens) * self.params.register_amount)
                d['m_flux_thick'][k].append(s.hmt.m_dot_thickening_flux * 3600) # kg/m2h (visual scale)
                d['m_flux_dens'][k].append(s.hmt.m_dot_densification * 1000) # g/s
                d['eta_fin'][k].append(0)
                
                # --- Resistances (Debug Critical) ---
                d['R_frost'][k].append(s.hmt.R_frost)
                d['R_air'][k].append(s.hmt.R_air)
                d['R_ref'][k].append(s.hmt.R_refrigerant)
                d['R_total'][k].append(s.hmt.R_downstream + s.hmt.R_air) # Approx total

        # --- 2. Plotting Setup (8 Rows x 4 Cols) ---
        # Rows:
        # 0: Global / System
        # 1: Air State (T, RH, W, p)
        # 2: Air Flow (v, Re, h_conv, dp)
        # 3: Refrigerant State (T, p, h, x)
        # 4: Refrigerant Flow (HTC, Flow Pattern, m_dot)
        # 5: Frost Properties (s, rho, m, k)
        # 6: Temperatures & Surfaces (T_surf, T_base, eta_fin)
        # 7: Heat Transfer & Fluxes (Q, m_flux, Resistances)
        
        fig, axs = plt.subplots(8, 4, figsize=(24, 40), sharex=True)
        
        def plot_layer(ax, data_key, title, ylabel, multiplier=1.0):
            """Helper to plot all layers in one subplot"""
            for k in range(self.params.layer_amount):
                # Sortierung umkehren falls nötig für Visualisierung (Air flow vs Ref flow)
                # Hier: Air flow direction (0->N)
                ax.plot(t_axis, np.array(d[data_key][k]) * multiplier, 
                        label=f'L{k+1}', color=colors[k], linewidth=1.0)
            ax.set_title(title, fontsize=10, weight='bold')
            ax.set_ylabel(ylabel, fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.7)
            if self.params.layer_amount < 8: # Legend only if readable
                ax.legend(fontsize=6, loc='best')

        # === ROW 0: SYSTEM GLOBALS ===
        # 0,0: Total Heat Flow
        ax = axs[0,0]
        q_sum = np.sum(d['Q_tot'], axis=0)
        ax.plot(t_axis, q_sum, 'k-', lw=1.5, label='Q_tot')
        ax.plot(t_axis, np.sum(d['Q_sens'], axis=0), 'r--', lw=1, label='Q_sens')
        ax.plot(t_axis, np.sum(d['Q_lat'], axis=0), 'b--', lw=1, label='Q_lat')
        ax.set_title("System: Heat Flow", weight='bold'); ax.set_ylabel("Q [W]"); ax.legend(fontsize=8)
        ax.grid(True)
        
        # 0,1: Total Frost Mass
        ax = axs[0,1]
        ax.plot(t_axis, np.sum(d['fr_mass'], axis=0), 'k-', lw=1.5)
        ax.set_title("System: Total Frost Mass", weight='bold'); ax.set_ylabel("Mass [g]"); ax.grid(True)
        
        # 0,2: System Air Flow
        ax = axs[0,2]
        ax.plot(t_axis, sys_vol_flow, 'k-', lw=1.5)
        ax.set_title("System: Air Vol Flow", weight='bold'); ax.set_ylabel("V_dot [m3/h]"); ax.grid(True)
        
        # 0,3: System Total Pressure Drop
        ax = axs[0,3]
        ax.plot(t_axis, np.sum(d['air_dp'], axis=0), 'k-', lw=1.5)
        ax.set_title("System: Total Air DP", weight='bold'); ax.set_ylabel("dp [Pa]"); ax.grid(True)


        # === ROW 1: AIR THERMO ===
        plot_layer(axs[1,0], 'air_T_out', "Air Temp Out (per Layer)", "T [°C]")
        # Add Inlet T to 1,0
        axs[1,0].plot(t_axis, d['air_T_in'][0], 'k--', alpha=0.5, label='Inlet')
        
        plot_layer(axs[1,1], 'air_RH_out', "Air RH Out", "RH [%]")
        plot_layer(axs[1,2], 'air_W_out', "Air Humidity Ratio (W)", "W [g/kg]")
        # 1,3 Unused -> Air Pressure absolute per layer (if interesting) or Air density
        # Let's plot Air Density indirectly via m_dot_humid if it chokes
        plot_layer(axs[1,3], 'air_m_dot_humid', "Air Mass Flow (Local)", "m_dot [kg/s]")


        # === ROW 2: AIR FLOW ===
        plot_layer(axs[2,0], 'air_v', "Air Velocity (Core)", "v [m/s]")
        plot_layer(axs[2,1], 'air_Re', "Air Reynolds", "Re [-]")
        plot_layer(axs[2,2], 'air_h_conv', "Air HTC (alpha)", "h [W/m2K]")
        plot_layer(axs[2,3], 'air_dp', "Air DP (per Layer)", "dp [Pa]")


        # === ROW 3: REFRIGERANT THERMO ===
        plot_layer(axs[3,0], 'ref_T_out', "Ref Temp Out", "T [°C]")
        plot_layer(axs[3,1], 'ref_p_out', "Ref Pressure", "p [bar]")
        plot_layer(axs[3,2], 'ref_h_out', "Ref Enthalpy Out", "h [kJ/kg]")
        plot_layer(axs[3,3], 'ref_x', "Ref Quality (x)", "x [-]")
        axs[3,3].set_ylim(-0.1, 1.1)


        # === ROW 4: REFRIGERANT FLOW ===
        plot_layer(axs[4,0], 'ref_h_conv', "Ref HTC (Effective)", "h [W/m2K]")
        plot_layer(axs[4,1], 'ref_portion_2p', "Ref Portion 2-Phase", "Portion [-]")
        # 4,2: Global Ref Mass Flow
        axs[4,2].plot(t_axis, sys_m_dot_ref, 'k-')
        axs[4,2].set_title("System: Ref Mass Flow"); axs[4,2].set_ylabel("m_dot [kg/s]"); axs[4,2].grid(True)
        # 4,3: Empty or Ref Portion Superheated
        plot_layer(axs[4,3], 'ref_portion_sh', "Ref Portion Superheated", "Portion [-]")


        # === ROW 5: FROST PROPERTIES ===
        plot_layer(axs[5,0], 'fr_thick', "Frost Thickness", "s [mm]")
        plot_layer(axs[5,1], 'fr_dens', "Frost Density", "rho [kg/m3]")
        plot_layer(axs[5,2], 'fr_mass', "Frost Mass (per Layer)", "m [g]")
        plot_layer(axs[5,3], 'fr_k', "Frost Conductivity", "k [W/mK]")


        # === ROW 6: SURFACE & TEMPS ===
        plot_layer(axs[6,0], 'fr_T_surf', "Frost Surface Temp", "T [°C]")
        plot_layer(axs[6,1], 'fr_T_base', "Frost Base (Wall) Temp", "T [°C]")
        # Check Delta T Frost
        axs[6,2].set_title("Delta T Frost (Surf - Base)", fontsize=10, weight='bold')
        for k in range(self.params.layer_amount):
            dt = np.array(d['fr_T_surf'][k]) - np.array(d['fr_T_base'][k])
            axs[6,2].plot(t_axis, dt, color=colors[k])
        axs[6,2].set_ylabel("dT [K]"); axs[6,2].grid(True)
        
        plot_layer(axs[6,3], 'eta_fin', "Fin Efficiency", "eta [-]")


        # === ROW 7: HEAT TRANSFER & RESISTANCES ===
        # Very important for debugging!
        plot_layer(axs[7,0], 'Q_tot', "Heat Flow per Layer", "Q [W]")
        
        # Mass Fluxes (Debug Growth Logic)
        plot_layer(axs[7,1], 'm_flux_thick', "Flux: Thickening", "m'' [kg/m2h]")
        
        # Resistances
        plot_layer(axs[7,2], 'R_frost', "Resistance: Frost", "R [K/W]")
        plot_layer(axs[7,3], 'R_air', "Resistance: Air", "R [K/W]")
        
        
        # Final Layout self.params
        plt.tight_layout()
        plt.subplots_adjust(top=0.96) # Platz für Titel
        
        # Save option?
        # plt.savefig(f"{case_name}_debug_full.png", dpi=300)
        
        plt.show()
