import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as path_effects
import matplotlib.cm as cm
import numpy as np
import os
import pandas as pd
import matplotlib.gridspec as gridspec
import CoolProp.CoolProp as CP 

from .exp_data_processing import MultiExperimentAnalyzer, ComparisonData

class SimulationVisualizer:
    """Handles all plotting and visualization for Frost Evaporator simulations."""
    
    def __init__(self, params):
        self.params = params
    
    def _extract_sim_data(self, states_history, inputs_history, steps, reg_amount):
        """Helper: Extracts and calculates simulation metrics from history objects."""
        data = {
            'n_fan': [], 'T_in': [], 'rh_in': [], 
            'm_ref': [], 'p_ref_in': [], 'h_ref_in': [],
            'dp': [], 'h_ref_out': [], 'Q': [], 'm_frost': []
        }
        
        num_layers = len(states_history[0])
        ref_in_idx = num_layers - 1

        for t in range(steps):
            # --- Inputs ---
            inp_air = inputs_history[t][0].air
            inp_ref = inputs_history[t][ref_in_idx].refrigerant
            
            data['T_in'].append(inp_air.T_in - 273.15)
            data['n_fan'].append(inp_air.fan_speed_rpm)
            
            try:
                rh = CP.HAPropsSI('R', 'T', inp_air.T_in, 'P', inp_air.p_in, 'W', inp_air.W_in)
                data['rh_in'].append(rh * 100.0)
            except:
                data['rh_in'].append(0.0)

            data['m_ref'].append(inp_ref.m_dot * reg_amount)
            data['p_ref_in'].append(inp_ref.p_eva / 1e5)
            data['h_ref_in'].append(inp_ref.h_in / 1e3)

            # --- Outputs ---
            dp_total = 0
            q_total = 0
            mass_frost_total = 0
            
            for k in range(num_layers):
                s = states_history[t][k]
                dp_total += s.air.pressure_drop
                q_total += s.hmt.Q_dot_total
                mass_frost_total += (s.frost.mass * 1000.0 * reg_amount)
            
            data['h_ref_out'].append(states_history[t][0].refrigerant.h_out / 1e3)
            data['dp'].append(dp_total)
            data['Q'].append(q_total * reg_amount)
            data['m_frost'].append(mass_frost_total)

        return {k: np.array(v) for k, v in data.items()}

    def _prepare_comparison_data(self, states_history, inputs_history, experiment_ids, path_exp, cutoff_pct, experiment_name, use_melted_mass=False):
        """
        Refactored Helper: Loads Simulation and Experiment data common to both plotting and table generation.
        """
        # Time & Stability Setup
        steps = len(states_history)
        t_sim = np.array([i * self.params.time_step / 60.0 for i in range(steps)])
        t_total = t_sim[-1]
        
        t_cut_start = t_total * cutoff_pct
        t_cut_end = t_total * (1.0 - cutoff_pct)
        mask_sim_stable = (t_sim >= t_cut_start) & (t_sim <= t_cut_end)

        # Extract Data
        sim_data = self._extract_sim_data(states_history, inputs_history, steps, self.params.global_register_amount)
        
        analyzer = MultiExperimentAnalyzer(experiment_type=experiment_name)
        # Pass the flag to the analyzer
        clean_data_list = analyzer.get_comparison_data(experiment_ids, path_exp, sim_duration=t_total, use_melted_mass=use_melted_mass)
        data_map = {d.id: d for d in clean_data_list}

        # Prepare Experiments & Scaling
        experiments = []
        scaling_data = {k: [] for k in sim_data.keys()}

        # Add Sim stable data to scaling accumulator
        for k, v in sim_data.items():
            if np.any(mask_sim_stable):
                scaling_data[k].extend(v[mask_sim_stable])

        for exp_id in experiment_ids:
            entry = {'id': exp_id, 'avail': False, 'time': None, 'data': {}}
            if exp_id in data_map:
                cdata = data_map[exp_id]
                entry['avail'] = True
                entry['time'] = cdata.time
                
                d = {
                    'n_fan': cdata.n_fan, 'T_in': cdata.T_in, 'rh_in': cdata.rh_in,
                    'm_ref': cdata.m_ref, 'p_ref_in': cdata.p_ref_in, 'h_ref_in': cdata.h_ref_in,
                    'h_ref_out': cdata.h_ref_out, 'dp': cdata.dp, 'Q': cdata.Q, 'm_frost': cdata.m_frost
                }
                if cdata.time_full is not None: d['time_full'] = cdata.time_full
                if cdata.m_frost_full is not None: d['m_frost_full'] = cdata.m_frost_full
                
                entry['data'] = d

                # Update Scaling Accumulator
                mask_exp_stable = (cdata.time >= t_cut_start) & (cdata.time <= t_cut_end)
                for k in d:
                    if k in scaling_data and np.any(mask_exp_stable):
                        scaling_data[k].extend(d[k][mask_exp_stable])
            experiments.append(entry)
            
        return {
            'sim_data': sim_data,
            'experiments': experiments,
            'scaling_data': scaling_data,
            't_sim': t_sim,
            't_cuts': (t_cut_start, t_cut_end, t_total)
        }

    def _calculate_single_error(self, key, sim_data_arr, t_sim, exp, t_cuts, experiment_name):
        """
        Refactored Helper: Calculates error for a single key/experiment pair.
        Returns: (avg_error_scalar, error_series_time, error_series_values, abs_val_scalar)
        """
        t_cut_start, t_cut_end, t_total = t_cuts
        
        # Generalize the check: if 'm_frost_full' is populated, it's a melted mass measurement
        is_melted_frost = (key == 'm_frost' and 'm_frost_full' in exp['data'])

        if is_melted_frost:
            # --- Plateau Error Calculation ---
            t_full, m_full = exp['data']['time_full'], exp['data']['m_frost_full']
            t_marker = exp['time'][-1]
            t_extended = t_total * 1.5
            mask_plateau = (t_full >= t_marker) & (t_full <= t_extended)
            
            # Determine the Plateau Value (Max in the extended region)
            val_exp = np.max(m_full[mask_plateau]) if np.any(mask_plateau) else exp['data'][key][-1]
            
            # Calculate Scalar Error
            val_sim = np.interp(t_marker, t_sim, sim_data_arr)
            err_val_signed = (val_sim - val_exp) / val_exp * 100.0 if val_exp != 0 else 0.0
            err_val_abs = abs(err_val_signed)

            # Return the absolute error for the optimizer, but the signed array for plots
            return err_val_abs, np.array([t_marker]), np.array([err_val_signed]), val_exp

        else:
            # --- Continuous Error Calculation ---
            sim_interp = np.interp(exp['time'], t_sim, sim_data_arr)
            residual = sim_interp - exp['data'][key] # renamed from abs_err
            
            mask_err = (exp['time'] >= t_cut_start) & (exp['time'] <= t_cut_end)
            
            # Normalize by the MAX (or MEAN) of the experimental data in the valid window
            # This prevents division-by-near-zero at the start of the experiment
            if np.sum(mask_err) > 0:
                norm_factor = np.max(np.abs(exp['data'][key][mask_err]))
                if norm_factor == 0:
                    norm_factor = 1.0 # Fallback to prevent div by zero
                    
                # Calculate percentage error relative to the peak value of this run
                rel_err = (residual / norm_factor) * 100.0 
                avg_rel = np.mean(np.abs(rel_err[mask_err]))
            else:
                rel_err = np.zeros_like(residual)
                avg_rel = 0.0

            return avg_rel, exp['time'], rel_err, None

    def get_relative_error_table(self, states_history, inputs_history, experiment_ids, path_exp, 
                                 cutoff_pct=0.02, experiment_name="OptiAbt", use_melted_mass=False):
        """
        Returns a Pandas DataFrame containing the averaged relative errors [%].
        """
        ctx = self._prepare_comparison_data(states_history, inputs_history, experiment_ids, path_exp, cutoff_pct, experiment_name, use_melted_mass=use_melted_mass)
        
        output_keys = ['dp', 'h_ref_out', 'Q', 'm_frost']
        results = {}

        for exp in ctx['experiments']:
            if not exp['avail']:
                continue
                
            exp_results = {}
            for key in output_keys:
                if key in exp['data']:
                    # Unpack 4 values, ignore the last 3 for the table
                    avg_err, _, _, _ = self._calculate_single_error(
                        key, ctx['sim_data'][key], ctx['t_sim'], exp, ctx['t_cuts'], experiment_name
                    )
                    exp_results[key] = avg_err
                else:
                    exp_results[key] = np.nan
            
            results[exp['id']] = exp_results

        df = pd.DataFrame.from_dict(results, orient='index')
        df.index.name = 'Experiment_ID'
        return df

    def _apply_smart_scaling(self, ax, key, sim_data, scaling_data):
        """Calculates and sets Y-limits based on stable SIM and EXP data."""
        # 1. Simulation Limits
        sim_vals = sim_data.get(key, [])
        sim_vals = sim_vals[~np.isnan(sim_vals)]
        
        f_min = np.min(sim_vals) if len(sim_vals) > 0 else np.inf
        f_max = np.max(sim_vals) if len(sim_vals) > 0 else -np.inf

        # 2. Experiment Limits (Percentile based)
        exp_vals = np.array(scaling_data.get(key, []))
        exp_vals = exp_vals[~np.isnan(exp_vals)]

        if len(exp_vals) > 0:
            p_min, p_max = np.percentile(exp_vals, [1.0, 99.0])
            f_min = min(f_min, p_min) if f_min != np.inf else p_min
            f_max = max(f_max, p_max) if f_max != -np.inf else p_max

        if f_min != np.inf and f_max != -np.inf:
            rng = f_max - f_min
            padding = 1.0 if rng == 0 else rng * 0.2
            ax.set_ylim(f_min - padding, f_max + padding)

    def _add_cutoff_vis(self, ax, t_start, t_end, t_total, show_labels=False):
        """Draws vertical bands for stability regions."""
        ax.axvspan(0, t_start, color='gray', alpha=0.15, lw=0)
        ax.axvline(t_start, color='gray', linestyle=':', linewidth=1)
        ax.axvspan(t_end, t_total, color='gray', alpha=0.15, lw=0)
        ax.axvline(t_end, color='gray', linestyle=':', linewidth=1)
        if show_labels:
            ax.text(t_start/2, ax.get_ylim()[1], "Start", ha='center', va='bottom', fontsize=8, color='gray', fontstyle='italic')

    def plot_comparison(self, states_history, inputs_history, experiment_ids, path_exp, 
                        cutoff_pct=0.02, save_fig=False, group_name=None, experiment_name="OptiAbt", use_melted_mass=False):
        """
        Master Dashboard: Comparison of N Experiments vs 1 Simulation.
        """
        # =========================================================================
        # 1. CONFIGURATION
        # =========================================================================
        BLUE_GRADIENT = ['#89CFF0', '#4682B4', '#0047AB', '#000080', '#04043A']
        RED_GRADIENT  = ['#FFB3B3', '#FF8080', '#FF4D4D', '#E60000', '#8B0000']
        C_SIM = 'black'

        if len(experiment_ids) > 5:
            raise ValueError(f"Too many experiments ({len(experiment_ids)}). Maximum allowed is 5.")

        # =========================================================================
        # 2. DATA PROCESSING
        # =========================================================================
        ctx = self._prepare_comparison_data(states_history, inputs_history, experiment_ids, path_exp, cutoff_pct, experiment_name, use_melted_mass=use_melted_mass)

        sim_data = ctx['sim_data']
        experiments = ctx['experiments']
        t_sim = ctx['t_sim']
        scaling_data = ctx['scaling_data']
        t_cut_start, t_cut_end, t_total = ctx['t_cuts']

        print(f"--- Visualizing {len(experiment_ids)} Experiments ({experiment_name}) ---")

        # =========================================================================
        # 3. FIGURE SETUP
        # =========================================================================
        fig = plt.figure(figsize=(24, 13))
        title_suffix = group_name if group_name else f"{len(experiment_ids)} Experiments vs Simulation"
        fig.suptitle(f"Unified Dashboard ({experiment_name}): {title_suffix}", fontsize=20, fontweight='bold', y=0.96)

        gs_top = gridspec.GridSpec(2, 3, figure=fig, top=0.90, bottom=0.45, hspace=0.35, wspace=0.20)
        gs_bot = gridspec.GridSpec(2, 4, figure=fig, height_ratios=[3, 1.2], top=0.38, bottom=0.05, hspace=0.0, wspace=0.20)

        # =========================================================================
        # 4. PLOT INPUTS (Top Half)
        # =========================================================================
        input_map = [
            (0, 0, 'n_fan',      'Fan Speed',           '[rpm]'),
            (0, 1, 'T_in',       'Air Inlet Temp',      '[°C]'),
            (0, 2, 'rh_in',      'Air Inlet RH',        '[%]'),
            (1, 0, 'm_ref',      'Ref. Mass Flow',      '[kg/s]'),
            (1, 1, 'h_ref_in',   'Ref. Inlet Enthalpy', '[kJ/kg]'),
            (1, 2, 'p_ref_in',   'Evap. Inlet Pressure','[bar]')
        ]

        for r, c, key, title, unit in input_map:
            ax = fig.add_subplot(gs_top[r, c])
            is_main = (r == 0 and c == 0)

            # Plot Experiments
            for idx, exp in enumerate(experiments):
                if exp['avail'] and key in exp['data']:
                    ax.plot(exp['time'], exp['data'][key], color=BLUE_GRADIENT[idx], lw=1.5, alpha=0.6, 
                            label=f"Exp {exp['id']}" if is_main else "")
            
            # Plot Simulation
            draw_style = 'steps-post' if key == 'n_fan' else 'default'
            ax.plot(t_sim, sim_data[key], color=C_SIM, lw=2.5, drawstyle=draw_style, label="SIM" if is_main else "")

            # Format
            self._add_cutoff_vis(ax, t_cut_start, t_cut_end, t_total, show_labels=is_main)
            self._apply_smart_scaling(ax, key, sim_data, scaling_data)
            
            ax.set_title(title, fontsize=11, fontweight='bold', color='#333333')
            ax.set_ylabel(unit, fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.6)
            if is_main: ax.legend(fontsize=8, loc='best', framealpha=0.9)

        # =========================================================================
        # 5. PLOT OUTPUTS (Bottom Half)
        # =========================================================================
        output_map = [
            ('dp',        'Air Pressure Drop',    'Pa'),
            ('h_ref_out', 'Ref. Outlet Enthalpy', 'kJ/kg'),
            ('Q',         'Total Heat Flow',      'W'),
            ('m_frost',   'Total Frost Mass',     'g')
        ]
        
        BLUE_GRADIENT = ['#89CFF0', '#4682B4', '#0047AB', '#000080', '#04043A']
        RED_GRADIENT  = ['#FFB3B3', '#FF8080', '#FF4D4D', '#E60000', '#8B0000']
        C_SIM = 'black'

        for col_idx, (key, title, unit) in enumerate(output_map):
            ax_main = fig.add_subplot(gs_bot[0, col_idx])
            ax_err  = fig.add_subplot(gs_bot[1, col_idx], sharex=ax_main)
            all_err_values = []

            # --- A. Plot Experiments & Calc Errors ---
            for idx, exp in enumerate(experiments):
                if not (exp['avail'] and key in exp['data']): continue
                
                c_exp = RED_GRADIENT[idx]
                
                # 1. Calculate Error (using shared helper)
                err_val, t_err, y_err, val_abs_marker = self._calculate_single_error(
                    key, sim_data[key], t_sim, exp, ctx['t_cuts'], experiment_name
                )
                
                # Check if this requires the plateau visualization
                is_melted_frost = (key == 'm_frost' and 'm_frost_full' in exp['data'])

                # Add to error scaling list
                if not is_melted_frost and len(y_err) > 0:
                     mask_valid = (t_err >= t_cut_start) & (t_err <= t_cut_end)
                     all_err_values.extend(y_err[mask_valid])
                elif is_melted_frost:
                     # 1. CHANGE: Append the signed error so the Y-axis scales correctly below 0
                     all_err_values.append(y_err[0])

                # 2. Plotting Logic
                if is_melted_frost:
                    # Plateau Special Visualization
                    t_full, m_full = exp['data']['time_full'], exp['data']['m_frost_full']
                    t_marker = t_err[0]
                    t_extended = t_total * 1.5

                    ax_main.plot(t_full, m_full, color=c_exp, lw=1.5, alpha=0.7)
                    
                    # 2. CHANGE: Extract and use the signed error for the markers and lines
                    signed_err = y_err[0]
                    lbl = f"Exp {exp['id']} ({signed_err:+.1f}%)"
                    ax_err.scatter(t_marker, signed_err, color=c_exp, marker='X', s=80, edgecolor='black', lw=0.5, zorder=10, label=lbl)
                    ax_err.vlines(t_marker, 0, signed_err, color=c_exp, linestyle=':', alpha=0.5)

                    # Extended Axis Vis (Main Plot)
                    ax_main.set_xlim(0, t_extended)
                    ax_main.axvspan(t_total, t_extended, color='#444444', alpha=0.15, hatch='//', edgecolor='gray')
                    ax_main.axvline(t_total, color='black', linestyle='-', linewidth=1.5)
                    
                    # --- FIX: USE val_abs_marker returned from helper ---
                    ax_main.plot([t_marker, t_marker], [0, val_abs_marker], color='darkgray', ls='-', lw=0.8, zorder=20)
                    ax_main.plot([t_marker, t_extended], [val_abs_marker, val_abs_marker], color='darkgray', ls='-', lw=0.8, zorder=20)
                    ax_main.scatter(t_marker, val_abs_marker, marker='X', s=120, color=c_exp, edgecolor='black', zorder=25)

                else:
                    # Standard Visualization
                    ax_main.plot(exp['time'], exp['data'][key], color=c_exp, lw=1.5, alpha=0.7)
                    ax_err.plot(t_err, y_err, color=c_exp, lw=1.2, label=f"Exp {exp['id']} (Ø{err_val:.1f}%)")

            # --- B. Plot Simulation ---
            ax_main.plot(t_sim, sim_data[key], color=C_SIM, lw=2.5, zorder=10, label="SIM Out")

            # ... (Rest of formatting logic same as before) ...
            self._apply_smart_scaling(ax_main, key, sim_data, scaling_data)
            ax_main.set_title(title, fontsize=11, fontweight='bold', color='#333333')
            ax_main.set_ylabel(f"[{unit}]", fontsize=9)
            ax_main.grid(True, linestyle=':', alpha=0.6)
            plt.setp(ax_main.get_xticklabels(), visible=False)
            if col_idx == 0: ax_main.legend(fontsize=8, loc='best')

            # Error Axis Formatting
            ax_err.axhline(0, color='gray', linestyle='--', linewidth=1)
            ax_err.set_ylabel("Err [%]", fontsize=8)
            ax_err.set_xlabel("Time [min]", fontsize=10)
            ax_err.grid(True, linestyle=':', alpha=0.6)
            
            valid_errs = [e for e in all_err_values if not (np.isnan(e) or np.isinf(e))]
            if valid_errs:
                p1, p99 = np.percentile(valid_errs, [1.0, 99.0])
                p1 = min(p1, -5.0)
                p99 = max(p99, 5.0)
                pad = (p99 - p1) * 0.15
                ax_err.set_ylim(p1 - pad, p99 + pad)

            if ax_err.get_legend_handles_labels()[0]:
                ax_err.legend(fontsize=7, loc='best', framealpha=0.9)

        # =========================================================================
        # 6. SAVE / SHOW
        # =========================================================================
        if save_fig:
            safe_name = str(group_name).replace("°", "deg").replace(" ", "_").replace("(", "").replace(")", "") if group_name else f"{len(experiment_ids)}_Experiments"
            out_name = f"Unified_Dashboard_{experiment_name}_{safe_name}.png"
            out_path = os.path.join(path_exp, "graphics", "Final_Comparison_Unified", out_name)
            
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            plt.savefig(out_path, dpi=300)
            print(f"Saved unified figure to {out_path}")
            plt.close()
        else:
            plt.show()




    ###################################################################################
    # ADDITIONAL PLOTS FOR DEEP DIVE ANALYSIS
    # ONLY DEPENDENT ON SIMULATION DATA
    ###################################################################################

    def plot_layer_temperatures(self, states_history):
        """
        Plots the evaporation temperature and the outlet temperatures of all layers
        in a single consolidated plot.
        """
        steps = len(states_history)
        num_layers = len(states_history[0])

        t_sim = [i * self.params.time_step / 60.0 for i in range(steps)]

        fig, ax = plt.subplots(figsize=(12, 7))

        # Evaporation Temperature (Baseline)
        t_evap = [step[0].refrigerant.T_two_phase_in - 273.15 for step in states_history]
        ax.plot(t_sim, t_evap, label='T Evaporation (Saturation)',
                color='black', linestyle='--', linewidth=2.5, zorder=5)

        # Outlet Temperatures per layer
        colors = cm.viridis([i / max(1, num_layers - 1) for i in range(num_layers)])

        for k in range(num_layers):
            t_out = [step[k].refrigerant.T_out - 273.15 for step in states_history]
            ax.plot(t_sim, t_out, label=f'Layer {k} Outlet', color=colors[k], alpha=0.8)

        ax.set_title("Refrigerant Temperature Evolution by Layer", fontsize=14, pad=15)
        ax.set_ylabel("Temperature [°C]", fontsize=12)
        ax.set_xlabel("Time [min]", fontsize=12)

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

        t_sim = [i * self.params.time_step / 60.0 for i in range(steps)]

        fig, axes = plt.subplots(num_layers, 1, figsize=(12, 4 * num_layers), sharex=True)
        if num_layers == 1:
            axes = [axes]

        for k in range(num_layers):
            ax = axes[k]
            h_tp = [step[k].refrigerant.h_conv_two_phase for step in states_history]
            h_sh = [step[k].refrigerant.h_conv_superheated for step in states_history]

            ax.plot(t_sim, h_tp, label='h_conv Two-Phase', color='teal', linewidth=2)
            ax.plot(t_sim, h_sh, label='h_conv Superheated', color='darkorange', linestyle='-.')

            ax.set_title(f"Layer {k} - Convective Heat Transfer Coefficients")
            ax.set_ylabel(r"HTC [W/(m²·K)]")
            ax.legend(loc='upper right')
            ax.grid(True, linestyle=':', alpha=0.7)

        axes[-1].set_xlabel("Time [min]")
        plt.tight_layout()
        plt.show()

    def plot_detailed(self, states_history, inputs_history, experiment_ids):
        """
        Creates a detailed 3x5 Subplot (Landscape) based on simulation data.
        """
        time_steps = len(states_history)
        time_axis = np.array([i * self.params.time_step / 60.0 for i in range(time_steps)])
        num_layers = len(states_history[0])

        if num_layers <= 10:
            cmap = plt.get_cmap('tab10')
            colors = [cmap(i) for i in range(num_layers)]
        else:
            cmap = plt.get_cmap('turbo')
            colors = cmap(np.linspace(0, 1, num_layers))

        data = {
            'air_dp_layer': [], 'air_T_in': [], 'air_T_out': [], 'air_RH_in': [], 'air_RH_out': [], 'air_h_conv': [],
            'ref_h_in': [], 'ref_h_out': [], 'ref_T_in': [], 'ref_T_out': [],
            'ref_Q': [], 'ref_portion_sh': [], 'ref_h_conv_eff': [],
            'frost_T_surf': [], 'frost_density': [], 'frost_mass': [], 'frost_thickness': [],
            'Q_sens_layer': [], 'Q_lat_layer': [], 'Q_tot_layer': []
        }

        for k in range(num_layers):
            dp, T_a_in, T_a_out, rh_in, rh_out, h_c_air = [], [], [], [], [], []
            h_r_in, h_r_out, T_r_in, T_r_out, h_c_ref = [], [], [], [], []
            T_f_surf, rho_f, m_f, th_f = [], [], [], []
            q_s, q_l, q_t = [], [], []
            port_sh = []

            for t in range(time_steps):
                s = states_history[t][k]
                inp = inputs_history[t][k]

                # Air
                dp.append(s.air.pressure_drop)
                T_a_in.append(inp.air.T_in - 273.15)
                T_a_out.append(s.air.T_out - 273.15)
                rh_in.append(s.air.R_in * 100)
                rh_out.append(s.air.R_out * 100)
                h_c_air.append(s.air.h_conv)

                # Refrigerant
                h_r_in.append(inp.refrigerant.h_in / 1000.0)
                h_r_out.append(s.refrigerant.h_out / 1000.0)
                T_r_in.append(s.refrigerant.T_in - 273.15)
                T_r_out.append(s.refrigerant.T_out - 273.15)
                port_sh.append(s.refrigerant.portion_superheated)

                h_eff = (s.refrigerant.portion_two_phase * s.refrigerant.h_conv_two_phase +
                         s.refrigerant.portion_superheated * s.refrigerant.h_conv_superheated)
                h_c_ref.append(h_eff)

                # Frost / Heat
                T_f_surf.append(s.hmt.T_frost_surface - 273.15)
                rho_f.append(s.frost.density)
                m_f.append(s.frost.mass * 1000.0 * self.params.global_register_amount)
                th_f.append(s.frost.thickness * 1000.0)

                q_t.append(s.hmt.Q_dot_total * self.params.global_register_amount)
                q_s.append(s.hmt.Q_dot_sens * self.params.global_register_amount)
                q_l.append((s.hmt.Q_dot_total - s.hmt.Q_dot_sens) * self.params.global_register_amount)

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
            vol = s0.air.v_dot_fan_m3h_segment* self.params.global_register_amount / self.params.global_fan_amount
            air_vol_flow.append(vol)

        km_m_dot = inputs_history[0][0].refrigerant.m_dot * self.params.global_register_amount
        frost_mass_sum = np.sum(data['frost_mass'], axis=0)
        Q_sens_sum = np.sum(data['Q_sens_layer'], axis=0)
        Q_lat_sum = np.sum(data['Q_lat_layer'], axis=0)
        Q_tot_sum = np.sum(data['Q_tot_layer'], axis=0)

        # Plotting Setup
        plt.rcParams.update({'font.size': 10})
        fig, axs = plt.subplots(3, 5, figsize=(32, 18))
        fig.suptitle(f"Simulation Results: {experiment_ids}", fontsize=20, fontweight='bold')

        def format_ax(ax, title, y_label, x_label="Time [min]"):
            ax.set_title(title, fontsize=11, fontweight='bold')
            ax.set_ylabel(y_label, fontsize=10)
            ax.set_xlabel(x_label, fontsize=10)
            ax.grid(True, linestyle=':', alpha=0.6)
            ax.tick_params(axis='both', labelsize=9)
            ax.legend(fontsize=8, loc='best', framealpha=0.8)

        # ROW 1: AIR
        ax = axs[0, 0]
        ax.plot(time_axis, air_dp_sum, label='Total', color='black', linestyle='--', linewidth=1.5)
        for k in range(num_layers):
            ax.plot(time_axis, data['air_dp_layer'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Air Pressure Drop", "$\Delta p$ [Pa]")

        ax = axs[0, 1]
        ax.plot(time_axis, air_vol_flow, label=None, color='black', linewidth=1.5)
        format_ax(ax, "Air Volume Flow", "$\dot{V}$ [m³/h]")
        ax.set_ylim(0, max(air_vol_flow)*1.2)

        ax = axs[0, 2]
        ax.plot(time_axis, data['air_T_in'][0], label='Sys In', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        for k in range(num_layers):
            ax.plot(time_axis, data['air_T_out'][k], label=f'L{k+1} Out', color=colors[k])
        format_ax(ax, "Air Temperatures", "T [°C]")

        ax = axs[0, 3]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['air_RH_out'][k], label=f'L{k+1} Out', color=colors[k])
        ax.plot(time_axis, data['air_RH_in'][0], label='Sys In', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        format_ax(ax, "Air Rel. Humidity (Out)", "RH [%]")

        ax = axs[0, 4]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['air_h_conv'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Air HTC ($h_{conv}$)", "h [W/m²K]")

        # ROW 2: REFRIGERANT
        ax = axs[1, 0]
        ax.axhline(y=km_m_dot, label=None, color='black', linestyle='-', linewidth=1.5)
        format_ax(ax, "Refrigerant Mass Flow", "$\dot{m}$ [kg/s]")
        ax.set_ylim(0, 0.08)

        ax = axs[1, 1]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_h_out'][k], label=f'L{k+1} IN (reverse)', linestyle='-', color=colors[k])
        ax.plot(time_axis, data['ref_h_in'][-1], label='Sys In (KM-side)', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        format_ax(ax, "Refrigerant Enthalpy", "h [kJ/kg]")

        ax = axs[1, 2]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_T_out'][k], linestyle='-', color=colors[k], label=f'L{k+1} Out')
        ax.plot(time_axis, data['ref_T_in'][-1], label='Sys In (KM-side)', color='black', linestyle='--', linewidth=1.5, alpha=0.7)
        format_ax(ax, "Refrigerant Temperature", "T [°C]")

        ax = axs[1, 3]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_portion_sh'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Ref. Portion Superheated", "Portion [-]")
        ax.set_ylim(-0.05, 1.05)

        ax = axs[1, 4]
        for k in range(num_layers):
            ax.plot(time_axis, data['ref_h_conv_eff'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Ref. HTC (Weighted)", "h [W/m²K]")

        # ROW 3: FROST & ENERGY
        ax = axs[2, 0]
        ax.plot(time_axis, Q_tot_sum, label='Total', color='black', linewidth=1.5)
        ax.plot(time_axis, Q_sens_sum, label='Sensible', color='gray', linestyle='--')
        ax.plot(time_axis, Q_lat_sum, label='Latent', color='gray', linestyle='-.')
        format_ax(ax, "Heat Flows (Sum)", "$\dot{Q}$ [W]")

        ax = axs[2, 1]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['frost_T_surf'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Surf. Temp.", "T [°C]")

        ax = axs[2, 2]
        for k in reversed(range(num_layers)):
            ax.plot(time_axis, data['frost_density'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Density", "$\\rho$ [kg/m³]")

        ax = axs[2, 3]
        ax.plot(time_axis, frost_mass_sum, label='Sum', color='black', linestyle='--', linewidth=1.5)
        for k in range(num_layers):
            ax.plot(time_axis, data['frost_mass'][k], label=f'L{k+1}', color=colors[k])
        format_ax(ax, "Frost Mass", "m [g]")

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
        time_steps = len(states_history)
        num_layers = len(states_history[0])
        time_axis = np.array([i * self.params.time_step / 60.0 for i in range(time_steps)])

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))

        for k in range(num_layers):
            raw_data = [states_history[t][k].air.roughness_multiplier for t in range(time_steps)]

            # Left: Pressure Loss (Original)
            ax1.plot(time_axis, raw_data, label=f'Layer {k+1}')

            # Right: HMT (Square Root)
            hmt_data = [val**0.5 for val in raw_data]
            ax2.plot(time_axis, hmt_data, label=f'Layer {k+1}')

        ax1.set_title(f"Air Roughness Multiplier\nPressure Loss", fontsize=12, fontweight='bold')
        ax1.set_xlabel("Time [min]")
        ax1.set_ylabel("Multiplier [-]")
        ax1.grid(True, linestyle=':', alpha=0.6)
        ax1.legend(loc='best', fontsize='small')

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
        if not states_history:
            print("No data to visualize.")
            return

        current_states = states_history[time_step_index]
        mm = 1000.0

        # Dimensions from params
        layer_count = len(current_states)
        layer_depth = self.params.fvm_fin_length * mm
        tube_width = self.params.tube_outer_diameter * mm
        fin_pitch = self.params.fin_pitch * mm
        fin_thickness_mm = self.params.fin_thickness * mm

        total_length = layer_depth * layer_count
        total_height = num_fins_to_show * fin_pitch

        fig, ax = plt.subplots(figsize=(16, 8), dpi=500)
        ax.set_aspect('equal')
        ax.set_facecolor('white')

        # Colors
        c_tube = '#C88A45'
        c_tube_border = '#8B5A2B'
        c_fin = '#101010'
        c_frost_fin = '#81D4FA'
        c_frost_tube = '#4FC3F7'
        c_frost_edge = '#0288D1'

        # Geometry Loop
        y_start = (total_height / 2) - (num_fins_to_show * fin_pitch / 2) + (fin_pitch/2)
        y_positions = [y_start + i*fin_pitch for i in range(num_fins_to_show)]

        x_cursor = 0.0

        for i in range(layer_count):
            state = current_states[i]
            th_frost_mm = state.frost.thickness * mm
            tube_center_x = x_cursor + (layer_depth / 2.0)

            # Draw Fins & Fin Frost
            for y in y_positions:
                frost_h = fin_thickness_mm + (2 * th_frost_mm)

                # Fin Frost (Background)
                rect_frost_fin = patches.Rectangle(
                    (x_cursor, y - frost_h/2),
                    layer_depth, frost_h,
                    linewidth=0.8, edgecolor=c_frost_edge,
                    facecolor=c_frost_fin, alpha=0.6, zorder=1
                )
                ax.add_patch(rect_frost_fin)

                # Fin Metal (Foreground)
                rect_fin = patches.Rectangle(
                    (x_cursor, y - fin_thickness_mm/2),
                    layer_depth, fin_thickness_mm,
                    linewidth=0, facecolor=c_fin, zorder=4
                )
                ax.add_patch(rect_fin)

            # Draw Tubes & Tube Frost
            tube_frost_w = tube_width + (2 * th_frost_mm)

            # Tube Frost
            rect_tube_frost = patches.Rectangle(
                (tube_center_x - tube_frost_w/2, -fin_pitch),
                tube_frost_w, total_height + fin_pitch*2,
                linewidth=0.8, edgecolor=c_frost_edge,
                facecolor=c_frost_tube, alpha=0.5, zorder=2
            )
            ax.add_patch(rect_tube_frost)

            # Tube Frost Outline
            rect_tube_frost_outline = patches.Rectangle(
                (tube_center_x - tube_frost_w/2, -fin_pitch),
                tube_frost_w, total_height + fin_pitch*2,
                linewidth=0.5, edgecolor=c_frost_edge, facecolor='none', linestyle='--', zorder=3
            )
            ax.add_patch(rect_tube_frost_outline)

            # Tube Metal
            rect_tube = patches.Rectangle(
                (tube_center_x - tube_width/2, -fin_pitch),
                tube_width, total_height + fin_pitch*2,
                linewidth=1.0, edgecolor=c_tube_border, facecolor=c_tube, zorder=5
            )
            ax.add_patch(rect_tube)

            # Annotations
            ax.text(x_cursor + layer_depth/2, 50, f"L{i+1}",
                    ha='center', va='bottom', fontsize=25, fontweight='bold', color='#555', zorder=100)

            txt = ax.text(tube_center_x, total_height / 2.0, f"{th_frost_mm:.2f}\nmm",
                          ha='center', va='center', fontsize=9, fontweight='bold', color='white', zorder=20)
            txt.set_path_effects([path_effects.Stroke(linewidth=2, foreground='#333'), path_effects.Normal()])

            x_cursor += layer_depth

        # Draw Air Flow Arrow
        arrow_x_start = -18
        arrow_x_end = -3
        arrow_y = total_height / 2

        ax.annotate("", xy=(arrow_x_end, arrow_y), xycoords='data',
                    xytext=(arrow_x_start, arrow_y), textcoords='data',
                    arrowprops=dict(facecolor='#0277BD', edgecolor='none', width=10, headwidth=25, headlength=20),
                    zorder=30)
        ax.text((arrow_x_start+arrow_x_end)/2, arrow_y + 10, "AIR",
                ha='center', va='bottom', color='#0277BD', fontweight='bold', fontsize=10, zorder=31)

        # Formatting
        ax.set_ylim(0, total_height + 15)
        ax.set_xlim(-25, 150)
        ax.spines['bottom'].set_bounds(0, 150)
        ax.set_xticks(np.arange(0, 151, 25))

        ax.set_title(f"Frost Distribution (Schematic Top-Down View)", fontsize=14, fontweight='bold', pad=15)
        ax.set_xlabel("Depth along Airflow path [mm]", fontsize=11)

        ax.set_yticks([])
        ax.spines['top'].set_visible(False)
        ax.spines['right'].set_visible(False)
        ax.spines['left'].set_visible(False)

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
        Creates a massive 8x4 Subplot (32 graphs) to visualize EVERY variable.
        Goal: Identify nonsensical values or instabilities.
        """
        time_steps = len(states_history)
        self.params.global_layer_amount = len(states_history[0])

        t_axis = np.array([i * self.params.time_step / 60.0 for i in range(time_steps)])

        cmap = plt.get_cmap('jet')
        colors = [cmap(i) for i in np.linspace(0, 1, self.params.global_layer_amount)]

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
            # RESISTANCES
            'R_frost': [], 'R_air': [], 'R_ref': [], 'R_total': []
        }

        # Pre-allocate lists per layer
        for key in d:
            d[key] = [[] for _ in range(self.params.global_layer_amount)]

        sys_vol_flow = []
        sys_m_dot_ref = []

        for t in range(time_steps):
            sys_vol_flow.append(states_history[t][0].air.v_dot_fan_m3h_segment * self.params.global_register_amount)
            sys_m_dot_ref.append(inputs_history[t][0].refrigerant.m_dot * self.params.global_register_amount)

            for k in range(self.params.global_layer_amount):
                s = states_history[t][k]
                inp = inputs_history[t][k]

                # Air
                d['air_T_in'][k].append(inp.air.T_in - 273.15)
                d['air_T_out'][k].append(s.air.T_out - 273.15)
                d['air_dp'][k].append(s.air.pressure_drop)
                d['air_vol'][k].append(s.air.v_dot_fan_m3h_segment)
                d['air_RH_out'][k].append(s.air.R_out * 100)
                d['air_W_out'][k].append(s.air.W_out * 1000)
                d['air_h_conv'][k].append(s.air.h_conv)
                d['air_Re'][k].append(s.air.reynolds)
                d['air_v'][k].append(s.air.velocity)
                d['air_m_dot_humid'][k].append(s.air.m_dot_humid)

                # Refrigerant
                d['ref_T_out'][k].append(s.refrigerant.T_out - 273.15)
                d['ref_p_out'][k].append(s.refrigerant.p_out / 1e5)
                d['ref_h_out'][k].append(s.refrigerant.h_out / 1000)
                d['ref_x'][k].append(s.refrigerant.quality_avg)

                h_eff = (s.refrigerant.portion_two_phase * s.refrigerant.h_conv_two_phase +
                         s.refrigerant.portion_superheated * s.refrigerant.h_conv_superheated)
                d['ref_h_conv'][k].append(h_eff)
                d['ref_portion_2p'][k].append(s.refrigerant.portion_two_phase)
                d['ref_portion_sh'][k].append(s.refrigerant.portion_superheated)

                # Frost
                d['fr_thick'][k].append(s.frost.thickness * 1000)
                d['fr_dens'][k].append(s.frost.density)
                d['fr_mass'][k].append(s.frost.mass * 1000 * self.params.global_register_amount)
                d['fr_T_surf'][k].append(s.hmt.T_frost_surface - 273.15)
                d['fr_T_base'][k].append(s.hmt.T_frost_base - 273.15)
                d['fr_k'][k].append(s.frost.k_frost)

                # HMT
                d['Q_tot'][k].append(s.hmt.Q_dot_total * self.params.global_register_amount)
                d['Q_sens'][k].append(s.hmt.Q_dot_sens * self.params.global_register_amount)
                d['Q_lat'][k].append((s.hmt.Q_dot_total - s.hmt.Q_dot_sens) * self.params.global_register_amount)
                d['m_flux_thick'][k].append(s.hmt.m_dot_thickening_flux * 3600)
                d['m_flux_dens'][k].append(s.hmt.m_dot_densification * 1000)
                d['eta_fin'][k].append(0)

                # Resistances
                d['R_frost'][k].append(s.hmt.R_frost)
                d['R_air'][k].append(s.hmt.R_air)
                d['R_ref'][k].append(s.hmt.R_refrigerant)
                d['R_total'][k].append(s.hmt.R_downstream + s.hmt.R_air)

        fig, axs = plt.subplots(8, 4, figsize=(24, 40), sharex=True)

        def plot_layer(ax, data_key, title, ylabel, multiplier=1.0):
            """Helper to plot all layers in one subplot"""
            for k in range(self.params.global_layer_amount):
                ax.plot(t_axis, np.array(d[data_key][k]) * multiplier,
                        label=f'L{k+1}', color=colors[k], linewidth=1.0)
            ax.set_title(title, fontsize=10, weight='bold')
            ax.set_ylabel(ylabel, fontsize=9)
            ax.grid(True, linestyle=':', alpha=0.7)
            if self.params.global_layer_amount < 8:
                ax.legend(fontsize=6, loc='best')

        # ROW 0: SYSTEM GLOBALS
        ax = axs[0,0]
        q_sum = np.sum(d['Q_tot'], axis=0)
        ax.plot(t_axis, q_sum, 'k-', lw=1.5, label='Q_tot')
        ax.plot(t_axis, np.sum(d['Q_sens'], axis=0), 'r--', lw=1, label='Q_sens')
        ax.plot(t_axis, np.sum(d['Q_lat'], axis=0), 'b--', lw=1, label='Q_lat')
        ax.set_title("System: Heat Flow", weight='bold'); ax.set_ylabel("Q [W]"); ax.legend(fontsize=8)
        ax.grid(True)

        ax = axs[0,1]
        ax.plot(t_axis, np.sum(d['fr_mass'], axis=0), 'k-', lw=1.5)
        ax.set_title("System: Total Frost Mass", weight='bold'); ax.set_ylabel("Mass [g]"); ax.grid(True)

        ax = axs[0,2]
        ax.plot(t_axis, sys_vol_flow, 'k-', lw=1.5)
        ax.set_title("System: Air Vol Flow", weight='bold'); ax.set_ylabel("V_dot [m3/h]"); ax.grid(True)

        ax = axs[0,3]
        ax.plot(t_axis, np.sum(d['air_dp'], axis=0), 'k-', lw=1.5)
        ax.set_title("System: Total Air DP", weight='bold'); ax.set_ylabel("dp [Pa]"); ax.grid(True)

        # ROW 1: AIR THERMO
        plot_layer(axs[1,0], 'air_T_out', "Air Temp Out (per Layer)", "T [°C]")
        axs[1,0].plot(t_axis, d['air_T_in'][0], 'k--', alpha=0.5, label='Inlet')

        plot_layer(axs[1,1], 'air_RH_out', "Air RH Out", "RH [%]")
        plot_layer(axs[1,2], 'air_W_out', "Air Humidity Ratio (W)", "W [g/kg]")
        plot_layer(axs[1,3], 'air_m_dot_humid', "Air Mass Flow (Local)", "m_dot [kg/s]")

        # ROW 2: AIR FLOW
        plot_layer(axs[2,0], 'air_v', "Air Velocity (Core)", "v [m/s]")
        plot_layer(axs[2,1], 'air_Re', "Air Reynolds", "Re [-]")
        plot_layer(axs[2,2], 'air_h_conv', "Air HTC (alpha)", "h [W/m2K]")
        plot_layer(axs[2,3], 'air_dp', "Air DP (per Layer)", "dp [Pa]")

        # ROW 3: REFRIGERANT THERMO
        plot_layer(axs[3,0], 'ref_T_out', "Ref Temp Out", "T [°C]")
        plot_layer(axs[3,1], 'ref_p_out', "Ref Pressure", "p [bar]")
        plot_layer(axs[3,2], 'ref_h_out', "Ref Enthalpy Out", "h [kJ/kg]")
        plot_layer(axs[3,3], 'ref_x', "Ref Quality (x)", "x [-]")
        axs[3,3].set_ylim(-0.1, 1.1)

        # ROW 4: REFRIGERANT FLOW
        plot_layer(axs[4,0], 'ref_h_conv', "Ref HTC (Effective)", "h [W/m2K]")
        plot_layer(axs[4,1], 'ref_portion_2p', "Ref Portion 2-Phase", "Portion [-]")

        axs[4,2].plot(t_axis, sys_m_dot_ref, 'k-')
        axs[4,2].set_title("System: Ref Mass Flow"); axs[4,2].set_ylabel("m_dot [kg/s]"); axs[4,2].grid(True)

        plot_layer(axs[4,3], 'ref_portion_sh', "Ref Portion Superheated", "Portion [-]")

        # ROW 5: FROST PROPERTIES
        plot_layer(axs[5,0], 'fr_thick', "Frost Thickness", "s [mm]")
        plot_layer(axs[5,1], 'fr_dens', "Frost Density", "rho [kg/m3]")
        plot_layer(axs[5,2], 'fr_mass', "Frost Mass (per Layer)", "m [g]")
        plot_layer(axs[5,3], 'fr_k', "Frost Conductivity", "k [W/mK]")

        # ROW 6: SURFACE & TEMPS
        plot_layer(axs[6,0], 'fr_T_surf', "Frost Surface Temp", "T [°C]")
        plot_layer(axs[6,1], 'fr_T_base', "Frost Base (Wall) Temp", "T [°C]")

        axs[6,2].set_title("Delta T Frost (Surf - Base)", fontsize=10, weight='bold')
        for k in range(self.params.global_layer_amount):
            dt = np.array(d['fr_T_surf'][k]) - np.array(d['fr_T_base'][k])
            axs[6,2].plot(t_axis, dt, color=colors[k])
        axs[6,2].set_ylabel("dT [K]"); axs[6,2].grid(True)

        plot_layer(axs[6,3], 'eta_fin', "Fin Efficiency", "eta [-]")

        # ROW 7: HEAT TRANSFER & RESISTANCES
        plot_layer(axs[7,0], 'Q_tot', "Heat Flow per Layer", "Q [W]")
        plot_layer(axs[7,1], 'm_flux_thick', "Flux: Thickening", "m'' [kg/m2h]")
        plot_layer(axs[7,2], 'R_frost', "Resistance: Frost", "R [K/W]")
        plot_layer(axs[7,3], 'R_air', "Resistance: Air", "R [K/W]")

        plt.tight_layout()
        plt.subplots_adjust(top=0.96)
        plt.show()