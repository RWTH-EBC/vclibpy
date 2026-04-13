import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.patheffects as path_effects
import matplotlib.cm as cm
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.ticker as ticker
import numpy as np
import os
import pandas as pd
import matplotlib.gridspec as gridspec
import CoolProp.CoolProp as CP 
import matplotlib.ticker as ticker
import matplotlib.lines as mlines
from matplotlib.legend_handler import HandlerTuple
from scipy.ndimage import gaussian_filter1d, maximum_filter1d, minimum_filter1d

from .exp_data_processing import MultiExperimentAnalyzer, ComparisonData

class SimulationVisualizer:
    """Handles all plotting and visualization for Frost Evaporator simulations."""
    
    def __init__(self, params):
        self.params = params
    
    def set_plot_style(self):
        """
        Setzt die globalen, publikationsfähigen Matplotlib-Parameter für die Masterarbeit.
        """
        cm_to_inch = 1 / 2.54
        width_cm = 15.5
        height_cm = width_cm * (2/3) 

        plt.rcParams.update({
            "figure.figsize": (width_cm * cm_to_inch, height_cm * cm_to_inch),
            "figure.dpi": 300,
            
            "text.usetex": True,
            "text.latex.preamble": r"\usepackage{icomma} \usepackage{amsmath}",
            "font.family": "serif",
            
            "font.size": 11,
            "axes.labelsize": 11,
            "axes.titlesize": 11,
            "legend.fontsize": 10,
            "xtick.labelsize": 10,
            "ytick.labelsize": 10,
            
            # Deutsches Zahlenformat für Achsen
            "axes.formatter.use_locale": True,
            
            "axes.linewidth": 0.8,
            "grid.color": "#D9D9D9",
            "grid.linestyle": "--",
            "grid.alpha": 0.5, # Subtiles Raster
            
            "legend.frameon": False,
            "savefig.bbox": "tight",
            "savefig.format": "pdf"
        })

        return {
            "blue": "#00549F",
            "red": "#DD402D",
            "green": "#70AD47",
            "dark_grey": "#4E4F50",
            "mid_grey": "#9D9EA0",
            "light_grey": "#D9D9D9"
        }
    
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
        stable_vals = np.array(scaling_data.get(key, []))
        stable_vals = stable_vals[~np.isnan(stable_vals)]

        if len(stable_vals) > 0:
            f_min, f_max = np.percentile(stable_vals, [1.0, 99.0])
            
            if f_min != np.inf and f_max != -np.inf:
                rng = f_max - f_min
                padding = 1.0 if rng == 0 else rng * 0.15
                
                lower_limit = f_min - padding
                
                if np.min(stable_vals) >= 0:
                    lower_limit = max(0, lower_limit)
                    
                ax.set_ylim(lower_limit, f_max + padding)

    def _add_cutoff_vis(self, ax, t_start, t_end, t_total, show_labels=False):
        """Draws vertical bands for stability regions."""
        ax.axvspan(0, t_start, color='gray', alpha=0.15, lw=0)
        ax.axvline(t_start, color='gray', linestyle=':', linewidth=1)
        ax.axvspan(t_end, t_total, color='gray', alpha=0.15, lw=0)
        ax.axvline(t_end, color='gray', linestyle=':', linewidth=1)

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

    def extract_parity_data(self, states_history, inputs_history, experiment_ids, path_exp, 
                            cutoff_pct=0.02, experiment_name="OptiAbt", use_melted_mass=False):
        """
        Pure data extraction: Calculates regressions and evaluates the Experiment, Regression, 
        and Simulation over the shared interval. Returns parity plot data.
        """
        import numpy as np
        
        # 1. Prepare data
        ctx = self._prepare_comparison_data(
            states_history, inputs_history, experiment_ids, 
            path_exp, cutoff_pct, experiment_name, use_melted_mass=use_melted_mass
        )

        sim_data = ctx['sim_data']
        experiments = ctx['experiments']
        t_sim = ctx['t_sim']
        _, _, t_total = ctx['t_cuts']

        # Determine global max time for evaluating experimental bounds
        global_t_max = t_total
        for exp in experiments:
            if exp.get('avail') and 'm_frost_full' in exp.get('data', {}):
                _, t_err, _, _ = self._calculate_single_error(
                    'm_frost', sim_data['m_frost'], t_sim, exp, ctx['t_cuts'], experiment_name
                )
                global_t_max = max(global_t_max, t_err[0])
        global_t_max = global_t_max * 1.02 

        # Output structure
        output_keys = ['dp', 'Q', 'm_frost']
        parity_data = {k: {'regr_vals': [], 'exp_vals': [], 'sim_vals': []} for k in output_keys}

        # 2. Evaluate Data per Output
        for key in output_keys:
            all_t_exp, all_y_exp, t_markers, y_markers = [], [], [], []
            is_melted_frost = (key == 'm_frost') and any(
                'm_frost_full' in exp.get('data', {}) for exp in experiments if exp.get('avail')
            )

            # A. Collect raw data to build the baseline Regression
            for exp in experiments:
                if not (exp['avail'] and key in exp['data']):
                    continue
                
                if is_melted_frost:
                    t_full, m_full = exp['data']['time_full'], exp['data']['m_frost_full']
                    mask_full = (t_full >= 0) & (t_full <= global_t_max)
                    
                    if len(m_full[mask_full]) == 0 or np.all(m_full[mask_full] == 0):
                        continue
                        
                    _, t_err, _, val_abs_marker = self._calculate_single_error(
                        key, sim_data[key], t_sim, exp, ctx['t_cuts'], experiment_name
                    )
                    t_markers.append(t_err[0])
                    y_markers.append(val_abs_marker)
                else:
                    mask = (exp['time'] >= 0) & (exp['time'] <= global_t_max)
                    t_clean = exp['time'][mask]
                    y_clean = exp['data'][key][mask]
                    
                    if len(y_clean) == 0 or np.all(y_clean == 0):
                        continue
                    
                    all_t_exp.extend(t_clean)
                    all_y_exp.extend(y_clean)

            # B. Fit the Regression function
            regr_func = None
            if is_melted_frost and t_markers:
                t_arr, y_arr = np.array(t_markers), np.array(y_markers)
                sum_sq = np.sum(t_arr**2)
                m_slope = np.sum(t_arr * y_arr) / sum_sq if sum_sq != 0 else 0
                regr_func = lambda t: m_slope * t
            elif all_t_exp:
                regr_func = np.poly1d(np.polyfit(all_t_exp, all_y_exp, 5))

            # C. Evaluate Data on Shared Interval
            scale = 1000.0 if key in ['Q', 'm_frost'] else 1.0
            t_sim_max = t_sim[-1] 

            for exp in experiments:
                if not (exp['avail'] and key in exp['data']) or regr_func is None:
                    continue

                if is_melted_frost:
                    _, t_err, _, val_abs_marker = self._calculate_single_error(
                        key, sim_data[key], t_sim, exp, ctx['t_cuts'], experiment_name
                    )
                    t_m = t_err[0]
                    
                    exp_val = val_abs_marker / scale
                    regr_val = regr_func(t_m) / scale
                    sim_val = np.interp(t_m, t_sim, sim_data[key]) / scale
                    
                else:
                    mask = (exp['time'] >= 0) & (exp['time'] <= global_t_max)
                    t_clean = exp['time'][mask]
                    y_clean = exp['data'][key][mask]
                    
                    if len(t_clean) == 0: continue
                    
                    t_end_exp = t_clean[-1]
                    t_end_eval = min(t_end_exp, t_sim_max)
                    t_start_eval = 0.95 * t_end_eval 
                    
                    mask_eval = (t_clean >= t_start_eval) & (t_clean <= t_end_eval)
                    t_window = t_clean[mask_eval]
                    
                    if len(t_window) == 0: continue
                    
                    exp_val = np.mean(y_clean[mask_eval]) / scale
                    regr_val = np.mean(regr_func(t_window)) / scale
                    sim_interp = np.interp(t_window, t_sim, sim_data[key])
                    sim_val = np.mean(sim_interp) / scale
                
                # Append to extraction dictionary
                parity_data[key]['regr_vals'].append(regr_val)
                parity_data[key]['exp_vals'].append(exp_val)
                parity_data[key]['sim_vals'].append(sim_val)

        return parity_data



    ####################################################################################################################
    ####################################################################################################################

    def plot_error_to_experiment_combined(self, states_history, inputs_history, experiment_ids, path_exp, 
                                          cutoff_pct=0.02, save_fig=False, group_name=None, 
                                          experiment_name="OptiAbt", use_melted_mass=False, 
                                          target_axes=None, is_first_col=True, is_title_col=True, meas_errors=None):
        
        # --- CONFIGURATION & STYLING ---
        if meas_errors is None:
            meas_errors = {'dp': 10.0, 'Q': 10.0, 'm_frost_dyn': 15.0, 'm_frost_melt': 50.0}

        Z_ORDER = {
            'grid': 0,
            'background_fill': 1,
            'boundary_lines': 2,
            'zero_line': 3,
            'experimental_data': 4,
            'simulation_line': 5,
            'text_overlay': 6
        }

        COLORS = {
            'green': '#2ECC71',
            'orange': '#F39C12',
            'red': '#E74C3C',
            'gray_bg': '#F8F9FA',
            'gray_line': '#9D9EA0',
            "dark_grey": "#4E4F50",
            "mid_grey": "#9D9EA0",
            "light_grey": "#D9D9D9",
            'default_frame': '#4E4F50', 
            'no_data_text': '#9D9EA0',
            'red_gradient': ['#FFB3B3', '#FF8080', '#FF4D4D', '#E60000', '#8B0000']
        }

        # Alpha-Werte für die Flächen
        ALPHA_FILL = 0.20
        ALPHA_RED = 0.15

        status_counts = {'green': 0, 'orange': 0, 'red': 0, 'gray': 0}
        
        # 1. DATA PREPARATION
        ctx = self._prepare_comparison_data(
            states_history, inputs_history, experiment_ids, 
            path_exp, cutoff_pct, experiment_name, use_melted_mass=use_melted_mass
        )
        
        sim_data = ctx['sim_data']
        experiments = ctx['experiments']
        t_sim = ctx['t_sim']
        
        calc_rel_err = lambda y, base: np.divide(y - base, base, out=np.zeros_like(base, dtype=float), where=base != 0) * 100

        output_map = [
            ('dp',      'Luftdruckverlust'),
            ('Q',       'Wärmestrom'),
            ('m_frost', 'Reifmasse')
        ]

        if target_axes is None:
            fig, axes = plt.subplots(3, 1, figsize=(15.5/2.54, 10/2.54), sharex=True)
            owns_figure = True
        else:
            axes = target_axes
            owns_figure = False

        # 2. MAIN PLOTTING LOOP
        for row_idx, (key, row_title) in enumerate(output_map):
            ax = axes[row_idx]
            frame_color = COLORS['default_frame']
            
            is_melted_frost = (key == 'm_frost') and any(
                'm_frost_full' in exp.get('data', {}) for exp in experiments if exp.get('avail')
            )

            if key == 'dp':
                meas_tol = meas_errors['dp']
            elif key == 'Q':
                meas_tol = meas_errors['Q']
            else:
                meas_tol = meas_errors['m_frost_melt'] if is_melted_frost else meas_errors['m_frost_dyn']

            all_y_exp_check = [y for exp in experiments if exp['avail'] and key in exp.get('data', {}) for y in exp['data'][key]]
            has_valid_data = len(all_y_exp_check) > 0 and not np.all(np.array(all_y_exp_check) == 0)

            if not has_valid_data:
                # --- CASE X: NO DATA ---
                ax.set_facecolor(COLORS['gray_bg'])
                ax.text(0.5, 0.5, 'Keine\nDaten', transform=ax.transAxes, 
                        ha='center', va='center', fontsize=11, 
                        color=COLORS['no_data_text'], zorder=Z_ORDER['text_overlay'])
                frame_color = COLORS['light_grey']

            elif is_melted_frost:
                # --- CASE A: DISCONTINUOUS MELTED FROST ---
                t_markers, y_markers, valid_exps = [], [], []
                
                for idx, exp in enumerate(experiments):
                    if exp['avail'] and key in exp['data'] and 'm_frost_full' in exp['data']:
                        _, t_err, _, val_abs_marker = self._calculate_single_error(
                            key, sim_data[key], t_sim, exp, ctx['t_cuts'], experiment_name
                        )
                        t_m, y_m = t_err[0], val_abs_marker
                        t_markers.append(t_m)
                        y_markers.append(y_m)
                        valid_exps.append((idx, exp, t_m, y_m))
                
                if t_markers:
                    t_arr, y_arr = np.array(t_markers), np.array(y_markers)
                    sum_sq = np.sum(t_arr**2)
                    m_slope = np.sum(t_arr * y_arr) / sum_sq if sum_sq != 0 else 0
                    
                    regr_baseline_sim = m_slope * t_sim
                    rel_err_sim = calc_rel_err(sim_data[key], regr_baseline_sim)
                    
                    t_full_width = np.array([t_sim.min(), t_sim.max()])
                    
                    t_min_m, t_max_m = np.min(t_arr), np.max(t_arr)
                    valid_mask = (t_sim >= t_min_m) & (t_sim <= t_max_m)
                    t_sim_valid = t_sim[valid_mask]
                    rel_err_sim_valid = rel_err_sim[valid_mask]

                    rel_errors = []
                    for idx, exp, t_m, y_m in valid_exps:
                        baseline_at_t = m_slope * t_m
                        err_m = ((y_m - baseline_at_t) / baseline_at_t * 100) if baseline_at_t != 0 else 0
                        rel_errors.append(err_m)
                        ax.scatter(t_m, err_m, marker='X', s=45, 
                                color=COLORS['red_gradient'][idx % len(COLORS['red_gradient'])], 
                                edgecolor='black', zorder=Z_ORDER['experimental_data'], label=f"Versuch {exp['id']}")

                    if rel_errors:
                        min_err, max_err = min(rel_errors), max(rel_errors)
                        G_min, G_max = np.array([min_err, min_err]), np.array([max_err, max_err])
                        O_min, O_max = np.array([-meas_tol, -meas_tol]), np.array([meas_tol, meas_tol])

                        # Höhere Alpha-Werte angewendet
                        ax.fill_between(t_full_width, G_min, G_max, color=COLORS['green'], alpha=ALPHA_FILL, zorder=Z_ORDER['background_fill'])
                        ax.axhline(max_err, color=COLORS['green'], lw=1.5, alpha=0.8, zorder=Z_ORDER['boundary_lines'])
                        ax.axhline(min_err, color=COLORS['green'], lw=1.5, alpha=0.8, zorder=Z_ORDER['boundary_lines'])

                        ax.fill_between(t_full_width, np.maximum(G_max, O_min), np.maximum(G_max, O_max), color=COLORS['orange'], alpha=ALPHA_FILL, zorder=Z_ORDER['background_fill'])
                        ax.fill_between(t_full_width, np.minimum(G_min, O_min), np.minimum(G_min, O_max), color=COLORS['orange'], alpha=ALPHA_FILL, zorder=Z_ORDER['background_fill'])
                        ax.axhline(meas_tol,  color=COLORS['orange'], lw=1.5, ls='-', alpha=0.7, zorder=Z_ORDER['boundary_lines'])
                        ax.axhline(-meas_tol, color=COLORS['orange'], lw=1.5, ls='-', alpha=0.7, zorder=Z_ORDER['boundary_lines'])

                        C_max, C_min = np.maximum(G_max, O_max), np.minimum(G_min, O_min)
                        ax.fill_between(t_full_width, C_max, 1000, color=COLORS['red'], alpha=ALPHA_RED, zorder=Z_ORDER['background_fill']-1)
                        ax.fill_between(t_full_width, -1000, C_min, color=COLORS['red'], alpha=ALPHA_RED, zorder=Z_ORDER['background_fill']-1)

                        is_green = np.mean((rel_err_sim_valid >= min_err) & (rel_err_sim_valid <= max_err)) >= 0.90 if len(rel_err_sim_valid) > 0 else False
                        is_orange = np.mean((rel_err_sim_valid >= -meas_tol) & (rel_err_sim_valid <= meas_tol)) >= 0.90 if len(rel_err_sim_valid) > 0 else False
                        frame_color = COLORS['green'] if is_green else (COLORS['orange'] if is_orange else COLORS['red'])

                    ax.axhline(0, color=COLORS['gray_line'], lw=1.5, alpha=0.8, zorder=Z_ORDER['zero_line'])
                    ax.plot(t_sim_valid, rel_err_sim_valid, color='black', lw=1.5, zorder=Z_ORDER['simulation_line'])

            else:
                # --- CASE B: CONTINUOUS DATA ---
                all_t_exp = [t for exp in experiments if exp['avail'] and key in exp['data'] for t in exp['time']]
                all_y_exp = [y for exp in experiments if exp['avail'] and key in exp['data'] for y in exp['data'][key]]
                
                if all_t_exp:
                    poly_func = np.poly1d(np.polyfit(all_t_exp, all_y_exp, 5))
                    t_envelope = np.linspace(min(all_t_exp), max(all_t_exp), 500)
                    exp_errors_interp = []

                    for idx, exp in enumerate(experiments):
                        if not (exp['avail'] and key in exp['data']): continue
                        rel_err = calc_rel_err(exp['data'][key], poly_func(exp['time']))
                        ax.plot(exp['time'], rel_err, color=COLORS['red_gradient'][idx % len(COLORS['red_gradient'])], 
                                lw=1.0, alpha=0.25, zorder=Z_ORDER['experimental_data'])
                        exp_errors_interp.append(np.interp(t_envelope, exp['time'], rel_err))

                    if exp_errors_interp:
                        err_stack = np.array(exp_errors_interp)
                        upper_smooth = gaussian_filter1d(maximum_filter1d(np.max(err_stack, axis=0), size=40), sigma=7)
                        lower_smooth = gaussian_filter1d(minimum_filter1d(np.min(err_stack, axis=0), size=40), sigma=7)
                        O_min_arr, O_max_arr = np.full_like(t_envelope, -meas_tol), np.full_like(t_envelope, meas_tol)

                        # Höhere Alpha-Werte angewendet
                        ax.fill_between(t_envelope, lower_smooth, upper_smooth, color=COLORS['green'], alpha=ALPHA_FILL, zorder=Z_ORDER['background_fill'])
                        ax.plot(t_envelope, upper_smooth, color=COLORS['green'], lw=1.5, alpha=0.8, zorder=Z_ORDER['boundary_lines'])
                        ax.plot(t_envelope, lower_smooth, color=COLORS['green'], lw=1.5, alpha=0.8, zorder=Z_ORDER['boundary_lines'])

                        ax.fill_between(t_envelope, np.maximum(upper_smooth, O_min_arr), np.maximum(upper_smooth, O_max_arr), color=COLORS['orange'], alpha=ALPHA_FILL, zorder=Z_ORDER['background_fill'])
                        ax.fill_between(t_envelope, np.minimum(lower_smooth, O_min_arr), np.minimum(lower_smooth, O_max_arr), color=COLORS['orange'], alpha=ALPHA_FILL, zorder=Z_ORDER['background_fill'])
                        ax.axhline(meas_tol, color=COLORS['orange'], lw=1.5, alpha=0.7, zorder=Z_ORDER['boundary_lines'])
                        ax.axhline(-meas_tol, color=COLORS['orange'], lw=1.5, alpha=0.7, zorder=Z_ORDER['boundary_lines'])

                        C_max, C_min = np.maximum(upper_smooth, O_max_arr), np.minimum(lower_smooth, O_min_arr)
                        ax.fill_between(t_envelope, C_max, 1000, color=COLORS['red'], alpha=ALPHA_RED, zorder=Z_ORDER['background_fill']-1)
                        ax.fill_between(t_envelope, -1000, C_min, color=COLORS['red'], alpha=ALPHA_RED, zorder=Z_ORDER['background_fill']-1)

                        valid_mask = (t_sim >= min(all_t_exp)) & (t_sim <= max(all_t_exp))
                        t_sim_v, rel_sim_v = t_sim[valid_mask], calc_rel_err(sim_data[key], poly_func(t_sim))[valid_mask]
                        
                        if len(t_sim_v) > 0:
                            env_up = np.interp(t_sim_v, t_envelope, upper_smooth)
                            env_lo = np.interp(t_sim_v, t_envelope, lower_smooth)
                            is_green = np.mean((rel_sim_v >= env_lo) & (rel_sim_v <= env_up)) >= 0.80
                            is_orange = np.mean((rel_sim_v >= -meas_tol) & (rel_sim_v <= meas_tol)) >= 0.80
                            frame_color = COLORS['green'] if is_green else (COLORS['orange'] if is_orange else COLORS['red'])

                    ax.axhline(0, color=COLORS['gray_line'], lw=1.5, alpha=0.8, zorder=Z_ORDER['zero_line'])
                    ax.plot(t_sim, calc_rel_err(sim_data[key], poly_func(t_sim)), color='black', lw=1.5, zorder=Z_ORDER['simulation_line'])

            # --- 3. DYNAMIC FORMATTING ---
            ax.margins(x=0)
            
            # RAHMEN KOMPLETT SICHTBAR MACHEN (Alle Spines)
            for spine in ax.spines.values():
                spine.set_visible(True)
                spine.set_color(frame_color)
                spine.set_linewidth(1.8)

            # Labels & Titles
            if row_idx == 0:
                ax.set_title(f"{group_name}", fontsize=11)
            
            if is_first_col:
                ax.set_ylabel(f"{row_title}\nFehler [\\%]", fontsize=11)
            
            # ---> NEUER ABSCHNITT FÜR DIE X-ACHSE (Stunden & 2 Ticks) <---
            if row_idx == 2: 
                ax.set_xlabel("Zeit [h]", fontsize=11)
                if has_valid_data:
                    # Maximale Zeit in dieser Spalte (in Minuten) abgreifen
                    t_max_min = ax.get_xlim()[1] 
                    
                    # Berechne den größten 0.5h-Schritt
                    # (z.B. 258 min -> 4.3 h -> 4.0 h)
                    max_tick_h = np.floor((t_max_min / 60.0) / 0.5) * 0.5
                    
                    if max_tick_h > 0:
                        # Setze exakt zwei Ticks (Koordinaten in Minuten)
                        ax.set_xticks([0, max_tick_h * 60.0])
                        # String formatieren: ':g' entfernt Nullen (4.0 -> 4), replace sorgt für das deutsche Komma
                        label_str = f"{max_tick_h:g}".replace('.', ',')
                        ax.set_xticklabels(["0", label_str])
                    else:
                        ax.set_xticks([0])
                        ax.set_xticklabels(["0"])

            if has_valid_data:
                limits = {0: (-50, 50), 1: (-40, 40), 2: (-60, 60)}
                ax.set_ylim(limits.get(row_idx, (-100, 100)))
                ax.grid(True, axis='y', linestyle='--', alpha=0.5, color='#D9D9D9', zorder=Z_ORDER['grid'])
                ax.grid(False, axis='x')
            
            status_key = next((k for k, v in COLORS.items() if v == frame_color), 'gray')
            status_counts[status_key if status_key in status_counts else 'red'] += 1

        if owns_figure:
            plt.tight_layout()
            if save_fig: 
                plt.savefig('standalone_plot.pdf', bbox_inches='tight')
                plt.close()
            else: 
                plt.show()
                    
        return status_counts



    def plot_comparison_thesis(self, states_history, inputs_history, experiment_ids, path_exp, 
                               y_limits_dict=None, cutoff_pct=0.02, save_fig=False, 
                               experiment_name="OptiAbt", use_melted_mass=False, output_filename="Comparison_Plot.pdf"):
        """
        Erstellt ein 3x3 Dashboard für die Masterarbeit mit manuellen Achsengrenzen.
        """
        # 1. Style & Farben initialisieren
        self.set_plot_style()
        colors = {
            "blue_grad":  ['#04043A', '#000080', '#0047AB', '#4682B4', '#89CFF0'],
            "red_grad":  ['#8B0000', '#E60000', '#FF4D4D', '#FF8080', '#FFB3B3'],
            "sim": 'black',
            "grid": '#D9D9D9'
        }

        # 2. Datenaufbereitung
        ctx = self._prepare_comparison_data(states_history, inputs_history, experiment_ids, path_exp, 
                                            cutoff_pct, experiment_name, use_melted_mass=use_melted_mass)
        
        sim_data = ctx['sim_data']
        experiments = ctx['experiments']
        t_sim = ctx['t_sim']
        scaling_data = ctx['scaling_data']
        _, _, t_total = ctx['t_cuts']
        global_t_max = t_total * 1.02

        # 3. Figure & GridSpec
        width_in = 15.2 / 2.54
        height_in = 13.0 / 2.54 
        fig = plt.figure(figsize=(width_in, height_in), layout='constrained')
        gs = gridspec.GridSpec(5, 3, figure=fig, height_ratios=[1, 0.1, 1, 0.1, 1])

        # Plot Matrix Definition
        plot_map = [
            (0, 0, 'n_fan',    'Lüfter Drehzahl',      '[rpm]',    True),
            (0, 1, 'T_in',     'Luft Temperatur',      '[°C]',     True),
            (0, 2, 'rh_in',    'Luft rel. Feuchte',    '[\%]',      True),
            (2, 0, 'm_ref',    'KM Massenstrom',       '[g/s]',    True),
            (2, 1, 'h_ref_in', 'KM Enthalpie Ein',     '[kJ/kg]',  True),
            (2, 2, 'p_ref_in', 'KM Druck',             '[bar]',    True),
            (4, 0, 'dp',       'Luft Druckverlust',    '[Pa]',     False),
            (4, 1, 'Q',        'Wärmestrom',           '[kW]',     False),
            (4, 2, 'm_frost',  'Reifmasse',           '[kg]',     False)
        ]

        for r, c, key, title, unit, is_input in plot_map:
            ax = fig.add_subplot(gs[r, c])
            
            # Achsen-Design
            ax.spines['top'].set_visible(False)
            ax.spines['right'].set_visible(False)
            ax.grid(axis='y', zorder=0)

            marker_max = -np.inf

            # A. Experimente Plotten
            for idx, exp in enumerate(experiments):
                if not (exp['avail'] and key in exp['data']): continue
                c_exp = colors["blue_grad"][idx % 4] if is_input else colors["red_grad"][idx % 4]
                
                # Sonderfall Frostmasse Marker
                if not is_input and key == 'm_frost' and 'm_frost_full' in exp['data']:
                    _, t_err, _, val_abs = self._calculate_single_error(key, sim_data[key], t_sim, exp, ctx['t_cuts'], experiment_name)
                    ax.scatter(t_err[0], val_abs, marker='X', s=30, color=c_exp, edgecolor='black', lw=0.5, zorder=5)
                    marker_max = max(marker_max, val_abs) 
                else:
                    mask = (exp['time'] >= 0) & (exp['time'] <= global_t_max)
                    ax.plot(exp['time'][mask], exp['data'][key][mask], color=c_exp, lw=0.8, alpha=0.7, zorder=2)
            
            # B. Simulation Plotten
            mask_sim = (t_sim >= 0) & (t_sim <= t_total)
            ax.plot(t_sim[mask_sim], sim_data[key][mask_sim], color=colors["sim"], 
                    lw=1.5, drawstyle='steps-post' if key=='n_fan' else 'default', zorder=4)

            # C. Manuelle Skalierung & 3-Tick-Logik
            if y_limits_dict and key in y_limits_dict:
                y_min, y_max = y_limits_dict[key]
                y_mid = (y_min + y_max) / 2.0
                ax.set_ylim(y_min, y_max)
                ax.set_yticks([y_min, y_mid, y_max])
            else:
                self._apply_smart_scaling(ax, key, sim_data, scaling_data)
                y_min, y_max = ax.get_ylim()

                scale_factor = 1.0
                if key == 'm_ref': scale_factor = 1000.0
                elif key in ['Q', 'm_frost']: scale_factor = 1 / 1000.0

                if key in ['m_frost', 'dp']:
                    # Zero-anchored accumulating variables
                    sim_max = np.max(sim_data[key][mask_sim]) if np.any(mask_sim) else -np.inf
                    exp_max = -np.inf
                    for exp in experiments:
                        if exp['avail'] and key in exp['data']:
                            mask_exp = (exp['time'] >= 0) & (exp['time'] <= global_t_max)
                            if np.any(mask_exp):
                                exp_max = max(exp_max, np.max(exp['data'][key][mask_exp]))
                    
                    true_max = max(sim_max, exp_max)
                    if key == 'm_frost' and marker_max > -np.inf:
                        true_max = max(true_max, marker_max)
                        
                    target_max_scaled = max(y_max, true_max * 1.05) * scale_factor
                    min_step = target_max_scaled / 2.0
                    
                    if min_step <= 0:
                        step_scaled = 1.0
                    else:
                        power = np.floor(np.log10(min_step))
                        frac = min_step / (10**power)
                        for nice in [1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]:
                            if nice >= frac:
                                step_scaled = nice * (10**power)
                                break
                                
                    nice_ticks_scaled = [0.0, step_scaled, 2.0 * step_scaled]
                    nice_ticks_unscaled = [t / scale_factor for t in nice_ticks_scaled]

                    ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                    ax.set_yticks(nice_ticks_unscaled)

                elif key == 'T_in':
                    # FIX: Force Air Temperature to strict integer bounds
                    t_min = np.floor(y_min)
                    t_max = np.ceil(y_max)
                    
                    # Prevent a flat line from collapsing the axis
                    if t_min == t_max:
                        t_max += 1.0
                        
                    t_mid = (t_min + t_max) / 2.0
                    
                    nice_ticks_unscaled = [t_min, t_mid, t_max]
                    ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                    ax.set_yticks(nice_ticks_unscaled)

                elif key == 'rh_in':
                    # FIX: Force rel. Feuchte to bounds and ticks divisible by 5
                    t_min = np.floor(y_min / 5.0) * 5.0
                    t_max = np.ceil(y_max / 5.0) * 5.0
                    
                    # Prevent a flat line from collapsing the axis
                    if t_min == t_max:
                        t_max += 10.0
                        
                    # To ensure the middle tick is also divisible by 5, the difference must be a multiple of 10
                    if (t_max - t_min) % 10 != 0:
                        t_max += 5.0
                        
                    t_mid = (t_min + t_max) / 2.0
                    
                    nice_ticks_unscaled = [t_min, t_mid, t_max]
                    ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                    ax.set_yticks(nice_ticks_unscaled)

                else:
                    # Floating variables (like rh_in, Q, etc.)
                    y_min_scaled = y_min * scale_factor
                    y_max_scaled = y_max * scale_factor
                    
                    locator = ticker.MaxNLocator(nbins=2, steps=[1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10])
                    ticks_scaled = locator.tick_values(y_min_scaled, y_max_scaled)
                    
                    t_min = ticks_scaled[0]
                    t_max = ticks_scaled[-1]
                    t_mid = (t_min + t_max) / 2.0
                    
                    nice_ticks_scaled = [t_min, t_mid, t_max]
                    nice_ticks_unscaled = [t / scale_factor for t in nice_ticks_scaled]

                    ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                    ax.set_yticks(nice_ticks_unscaled)

            # D. Formatter (Einheitenumrechnung & Deutsches Komma)
            def german_formatter(x, pos, k=key):
                val = x
                if k == 'm_ref': val *= 1000  
                elif k in ['Q', 'm_frost']: val /= 1000 
                
                # The 'g' format specifier cleanly drops trailing zeros (e.g. 10.0 becomes 10, 10.5 stays 10.5)
                val = np.round(val, 1)
                s = f"{val:g}".replace('.', ',')
                return f"${s}$"

            ax.yaxis.set_major_formatter(ticker.FuncFormatter(german_formatter))
            ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, p: f"${x:g}$".replace('.', ',')))
            
            # Beschriftung
            ax.set_title(f"{title} {unit}")
            ax.set_xlim(0, global_t_max)
            
            if r == 4:
                ax.set_xlabel("Zeit [min]")
            else:
                ax.tick_params(labelbottom=False)

        # 4. Globale Legende unter dem Plot
        sim_line = mlines.Line2D([], [], color=colors["sim"], lw=1.5, label='Simulation')
        exp_handles = [sim_line]
        exp_labels = ['SIM']
        
        for idx, exp in enumerate(experiments):
            if not exp.get('avail'): continue
            c_in = colors["blue_grad"][idx % 4]
            c_out = colors["red_grad"][idx % 4]
            h = (mlines.Line2D([], [], color=c_in, lw=1), mlines.Line2D([], [], color=c_out, lw=1))
            exp_handles.append(h)
            exp_labels.append(f"Exp. {exp['id']}")

        fig.legend(exp_handles, exp_labels, loc='outside lower center', ncol=len(exp_labels), 
                   bbox_to_anchor=(0.5, -0.08), columnspacing=1.2, frameon=False, handlelength=1.0,
                   handler_map={tuple: HandlerTuple(ndivide=None)})

        # 5. Speichern
        if save_fig:
            output_dir = r"J:\Masterarbeit_Nils_Baumeister\02_Schriftliche_Ausarbeitung\Graphiken"
            os.makedirs(output_dir, exist_ok=True)
            out_path = os.path.join(output_dir, output_filename)
            plt.savefig(out_path, bbox_inches='tight')
            print(f"Grafik gespeichert: {out_path}")

        plt.show()


    def plot_comparison_powerpoint(self, states_history, inputs_history, experiment_ids, path_exp, 
                                   y_ticks_dict=None, cutoff_pct=0.02, save_fig=False, 
                                   experiment_name="OptiAbt", use_melted_mass=False, output_filename="Comparison_Plot.pdf"):
        """
        Erstellt ein 3x3 Dashboard für die Masterarbeit mit manuellen Achsengrenzen und Ticks.
        """
        # 1. Style & Farben initialisieren
        cm_to_inch = 1 / 2.54
        width_cm = 21
        height_cm = 14.0  # <-- Höhe leicht erhöht für die zusätzlichen X-Ticks

        FONT_FAMILY = "Arial"
        FONT_SIZE = 14

        plt.rcParams.update({
            "figure.figsize": (width_cm * cm_to_inch, height_cm * cm_to_inch),
            "figure.dpi": 300,
            "font.family": FONT_FAMILY,
            "font.size": FONT_SIZE,
            "axes.labelsize": FONT_SIZE,
            "legend.fontsize": FONT_SIZE,
            "xtick.labelsize": FONT_SIZE,
            "ytick.labelsize": FONT_SIZE,
            "axes.formatter.use_locale": True,
            "axes.linewidth": 0.8,
            "grid.color": "#D9D9D9",
            "grid.linestyle": "--",
            "grid.alpha": 0.5,
            "legend.frameon": False,
            "savefig.bbox": "tight",
            "savefig.format": "svg"
        })

        colors = {
            "blue_grad":  ['#04043A', '#000080', '#0047AB', '#4682B4', '#89CFF0'],
            "red_grad":  ['#8B0000', '#E60000', '#FF4D4D', '#FF8080', '#FFB3B3'],
            "sim": 'black',
            "grid": '#D9D9D9'
        }

        # 2. Datenaufbereitung
        ctx = self._prepare_comparison_data(states_history, inputs_history, experiment_ids, path_exp, 
                                            cutoff_pct, experiment_name, use_melted_mass=use_melted_mass)
        
        sim_data = ctx['sim_data']
        experiments = ctx['experiments']
        t_sim = ctx['t_sim']
        scaling_data = ctx['scaling_data']
        _, _, t_total = ctx['t_cuts']
        global_t_max = t_total * 1.02

        # STAGE LOOP HINZUGEFÜGT
        for stage in [1, 2, 3]:
            # 3. Figure & GridSpec
            width_in = width_cm / 2.54
            height_in = height_cm / 2.54 
            fig = plt.figure(figsize=(width_in, height_in), layout='constrained')
            gs = gridspec.GridSpec(5, 3, figure=fig, height_ratios=[1, 0.1, 1, 0.1, 1])

            # Plot Matrix Definition
            plot_map = [
                (0, 0, 'n_fan',    'Lüfter Drehzahl',      '(1/min)',    True),
                (0, 1, 'T_in',     'Temperatur',      '(°C)',    True),
                (0, 2, 'rh_in',    'Relative Feuchte',    '(%)',      True),
                (2, 0, 'm_ref',    'Massenstrom',       '(g/s)',    True),
                (2, 1, 'h_ref_in', 'Enthalpie Ein',     '(kJ/kg)',  True),
                (2, 2, 'p_ref_in', 'Druck',             '(bar)',    True),
                (4, 0, 'dp',       'Luft Druckverlust',    '(Pa)',     False),
                (4, 1, 'Q',        'Wärmestrom',           '(kW)',     False),
                (4, 2, 'm_frost',  'Reifmasse',           '(kg)',     False)
            ]

            for r, c, key, title, unit, is_input in plot_map:
                ax = fig.add_subplot(gs[r, c])
                
                # Achsen-Design
                ax.spines['top'].set_visible(False)
                ax.spines['right'].set_visible(False)
                ax.grid(axis='y', zorder=0)

                marker_max = -np.inf

                # A. Experimente Plotten
                for idx, exp in enumerate(experiments):
                    if not (exp['avail'] and key in exp['data']): continue
                    c_exp = colors["blue_grad"][idx % 4] if is_input else colors["red_grad"][idx % 4]
                    
                    # Sonderfall Frostmasse Marker
                    if not is_input and key == 'm_frost' and 'm_frost_full' in exp['data']:
                        _, t_err, _, val_abs = self._calculate_single_error(key, sim_data[key], t_sim, exp, ctx['t_cuts'], experiment_name)
                        
                        marker_max = max(marker_max, val_abs) # Immer berechnen, damit die Achsen konstant bleiben
                        
                        if stage == 1 and idx > 0: continue # Überspringen, wenn Stage 1 und nicht das erste Experiment
                        ax.scatter(t_err[0], val_abs, marker='X', s=30, color=c_exp, edgecolor='black', lw=0.5, zorder=5)
                    else:
                        if stage == 1 and idx > 0: continue # Überspringen, wenn Stage 1 und nicht das erste Experiment
                        mask = (exp['time'] >= 0) & (exp['time'] <= global_t_max)
                        ax.plot(exp['time'][mask], exp['data'][key][mask], color=c_exp, lw=0.8, alpha=0.7, zorder=2)
                
                # B. Simulation Plotten
                mask_sim = (t_sim >= 0) & (t_sim <= t_total)
                if stage == 3: # Nur in Stage 3 anzeigen
                    ax.plot(t_sim[mask_sim], sim_data[key][mask_sim], color=colors["sim"], 
                            lw=2.5, drawstyle='steps-post' if key=='n_fan' else 'default', zorder=4)

                # C. Manuelle Skalierung & 3-Tick-Logik (Unverändert)
                if y_ticks_dict and key in y_ticks_dict:
                    # Apply explicit user ticks
                    ticks = y_ticks_dict[key]
                    ax.set_ylim(ticks[0], ticks[-1])
                    ax.set_yticks(ticks)
                else:
                    self._apply_smart_scaling(ax, key, sim_data, scaling_data)
                    y_min, y_max = ax.get_ylim()

                    scale_factor = 1.0
                    if key == 'm_ref': scale_factor = 1000.0
                    elif key in ['Q', 'm_frost']: scale_factor = 1 / 1000.0

                    if key in ['m_frost', 'dp']:
                        # Zero-anchored accumulating variables
                        sim_max = np.max(sim_data[key][mask_sim]) if np.any(mask_sim) else -np.inf
                        exp_max = -np.inf
                        for exp in experiments:
                            if exp['avail'] and key in exp['data']:
                                mask_exp = (exp['time'] >= 0) & (exp['time'] <= global_t_max)
                                if np.any(mask_exp):
                                    exp_max = max(exp_max, np.max(exp['data'][key][mask_exp]))
                        
                        true_max = max(sim_max, exp_max)
                        if key == 'm_frost' and marker_max > -np.inf:
                            true_max = max(true_max, marker_max)
                            
                        target_max_scaled = max(y_max, true_max * 1.05) * scale_factor
                        min_step = target_max_scaled / 2.0
                        
                        if min_step <= 0:
                            step_scaled = 1.0
                        else:
                            power = np.floor(np.log10(min_step))
                            frac = min_step / (10**power)
                            for nice in [1.0, 1.2, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0]:
                                if nice >= frac:
                                    step_scaled = nice * (10**power)
                                    break
                                    
                        nice_ticks_scaled = [0.0, step_scaled, 2.0 * step_scaled]
                        nice_ticks_unscaled = [t / scale_factor for t in nice_ticks_scaled]

                        ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                        ax.set_yticks(nice_ticks_unscaled)

                    elif key == 'T_in':
                        # FIX: Force Air Temperature to strict integer bounds
                        t_min = np.floor(y_min)
                        t_max = np.ceil(y_max)
                        
                        # Prevent a flat line from collapsing the axis
                        if t_min == t_max:
                            t_max += 1.0
                            
                        t_mid = (t_min + t_max) / 2.0
                        
                        nice_ticks_unscaled = [t_min, t_mid, t_max]
                        ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                        ax.set_yticks(nice_ticks_unscaled)

                    elif key == 'rh_in':
                        # FIX: Force rel. Feuchte to bounds and ticks divisible by 5
                        t_min = np.floor(y_min / 5.0) * 5.0
                        t_max = np.ceil(y_max / 5.0) * 5.0
                        
                        # Prevent a flat line from collapsing the axis
                        if t_min == t_max:
                            t_max += 10.0
                            
                        # To ensure the middle tick is also divisible by 5, the difference must be a multiple of 10
                        if (t_max - t_min) % 10 != 0:
                            t_max += 5.0
                            
                        t_mid = (t_min + t_max) / 2.0
                        
                        nice_ticks_unscaled = [t_min, t_mid, t_max]
                        ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                        ax.set_yticks(nice_ticks_unscaled)

                    else:
                        # Floating variables (like rh_in, Q, etc.)
                        y_min_scaled = y_min * scale_factor
                        y_max_scaled = y_max * scale_factor
                        
                        locator = ticker.MaxNLocator(nbins=2, steps=[1, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10])
                        ticks_scaled = locator.tick_values(y_min_scaled, y_max_scaled)
                        
                        t_min = ticks_scaled[0]
                        t_max = ticks_scaled[-1]
                        t_mid = (t_min + t_max) / 2.0
                        
                        nice_ticks_scaled = [t_min, t_mid, t_max]
                        nice_ticks_unscaled = [t / scale_factor for t in nice_ticks_scaled]

                        ax.set_ylim(nice_ticks_unscaled[0], nice_ticks_unscaled[-1])
                        ax.set_yticks(nice_ticks_unscaled)

                # D. Formatter (Einheitenumrechnung & Deutsches Komma) (Unverändert)
                def german_formatter(x, pos, k=key):
                    val = x
                    if k == 'm_ref': val *= 1000  
                    elif k in ['Q', 'm_frost']: val /= 1000 
                    
                    # The 'g' format specifier cleanly drops trailing zeros
                    val = np.round(val, 1)
                    s = f"{val:g}".replace('.', ',')
                    return f"${s}$"

                ax.yaxis.set_major_formatter(ticker.FuncFormatter(german_formatter))
                ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda x, p: f"${x:g}$".replace('.', ',')))
                
                # Beschriftung
                ax.set_title(f"{title} {unit}")
                ax.set_xlim(0, global_t_max)
                
                # <-- X-Achsen Label für JEDEN Subplot aktivieren
                ax.set_xlabel("Zeit (min)")

            # 4. Globale Legende unter dem Plot (Stage-Abhängig angepasst)
            exp_handles = []
            exp_labels = []
            
            if stage == 3:
                sim_line = mlines.Line2D([], [], color=colors["sim"], lw=1.5, label='Simulation')
                exp_handles.append(sim_line)
                exp_labels.append('SIM')

            for idx, exp in enumerate(experiments):
                if not exp.get('avail'): continue
                if stage == 1 and idx > 0: continue # Nur erstes Experiment für Stage 1 in der Legende

                c_in = colors["blue_grad"][idx % 4]
                c_out = colors["red_grad"][idx % 4]
                h = (mlines.Line2D([], [], color=c_in, lw=1), mlines.Line2D([], [], color=c_out, lw=1))
                exp_handles.append(h)
                exp_labels.append(f"Exp. {exp['id']}")

            # fig.legend(exp_handles, exp_labels, loc='outside lower center', ncol=len(exp_labels), 
            #            bbox_to_anchor=(0.5, -0.08), columnspacing=1.2, frameon=False, handlelength=1.0,
            #            handler_map={tuple: HandlerTuple(ndivide=None)})

            # 5. Speichern (Angepasst für die 3 Stages und "PP_" Präfix)
            if save_fig:
                output_dir = r"J:\Masterarbeit_Nils_Baumeister\02_Schriftliche_Ausarbeitung\Graphiken"
                os.makedirs(output_dir, exist_ok=True)
                
                # Base Namen extrahieren und anpassen
                base_name = os.path.splitext(output_filename)[0]
                if not base_name.startswith("PP_"):
                    base_name = "PP_BIG_" + base_name
                
                # Dateinamen zusammensetzen (z.B. PP_Comparison_Plot_1.svg)
                stage_filename = f"{base_name}_{stage}.svg"
                out_path = os.path.join(output_dir, stage_filename)
                
                plt.savefig(out_path, bbox_inches='tight')
                print(f"Grafik gespeichert: {out_path}")

        # Zeigt alle 3 generierten Figures am Ende gebündelt an
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



    def _draw_evaporator_state(self, ax, current_states, colors, for_presentation=False, num_fins_to_show=None):
        mm = 1000.0

        # Dimensionen
        layer_count = self.params.global_layer_amount
        layer_depth = self.params.fvm_fin_length * mm
        fin_pitch = self.params.fin_pitch * mm
        fin_thickness_mm = self.params.fin_thickness * mm
        total_length = layer_depth * layer_count

        # Daten für Step-Plot
        x_edges = np.linspace(0, total_length, layer_count + 1)
        frost_thicknesses = [state.frost.thickness * mm for state in current_states]
        frost_plot_values = np.array(frost_thicknesses + [frost_thicknesses[-1]])

        # --- DYNAMISCHE LAMELLENANZAHL ---
        target_ratio = 0.25 
        target_height = total_length * target_ratio
        
        if num_fins_to_show == None: 
            num_fins_to_show = max(2, int(round(target_height / fin_pitch)))

        total_height = num_fins_to_show * fin_pitch

        # --- FARBPALETTE ---
        c_fin = colors["dark_grey"]  
        c_frost = '#74c0fc'          
        c_frost_edge = '#1971c2'     
        c_air = 'black'          
        c_text = 'black'         
        c_grid = colors["light_grey"]

        # Y-Positionen der Lamellen
        y_start = (total_height / 2) - (num_fins_to_show * fin_pitch / 2) + (fin_pitch / 2)
        y_positions = [y_start + i * fin_pitch for i in range(num_fins_to_show)]

        ax.clear() 
        ax.set_aspect('equal')
        ax.set_facecolor('white')

        # --- ZEICHNEN ---
        # 1. Frost & Lamellen
        for y_center in y_positions:
            y_top_frost = y_center + (fin_thickness_mm / 2) + frost_plot_values
            y_bottom_frost = y_center - (fin_thickness_mm / 2) - frost_plot_values
            
            ax.fill_between(x_edges, y_bottom_frost, y_top_frost, 
                            step='post', facecolor=c_frost, edgecolor=c_frost_edge, 
                            linewidth=0.5, alpha=0.9, zorder=3)

            ax.add_patch(patches.Rectangle((0, y_center - fin_thickness_mm/2), total_length, fin_thickness_mm,
                                            linewidth=0, facecolor=c_fin, zorder=4))

        # 2. Fortsetzungspunkte & Schicht-Beschriftung (NUR THESIS)
        if not for_presentation:
            for i in range(layer_count):
                x_mid = (i * layer_depth) + (layer_depth / 2.0)
                for y_offset in [0.8, 1.2]:
                    ax.plot(x_mid, max(y_positions) + fin_pitch * y_offset, 'o', color='black', markersize=1.2, zorder=20)
                    ax.plot(x_mid, min(y_positions) - fin_pitch * y_offset, 'o', color='black', markersize=1.2, zorder=20)

            label_y = max(y_positions) + fin_pitch * 1.9
            for i in range(layer_count):
                x_mid = (i * layer_depth) + (layer_depth / 2.0)
                if i > 0:
                    ax.axvline(i * layer_depth, color=c_grid, linewidth=0.8, linestyle=(0, (5, 5)), zorder=1)
                ax.text(x_mid, label_y, f"S{i+1}", ha='center', va='bottom', color=c_text)

        # 3. Luftstrom (NUR THESIS)
        if not for_presentation:
            core_ratio = 0.80 
            total_display_range = total_length / core_ratio
            arrow_length = total_display_range * 0.08 
            arrow_tip_gap = total_display_range * 0.012 
            text_gap = total_display_range * 0.02

            y_air_positions = [y + (fin_pitch / 2) for y in y_positions[:-1]]
            y_air_positions.insert(0, min(y_positions) - (fin_pitch / 2))
            y_air_positions.append(max(y_positions) + (fin_pitch / 2))

            for y_air in y_air_positions:
                ax.annotate("", xy=(-arrow_tip_gap, y_air), xytext=(-(arrow_tip_gap + arrow_length), y_air),
                            arrowprops=dict(arrowstyle='-|>', color=c_air, lw=1.0, mutation_scale=10), zorder=10)
            
            text_x_pos = -(arrow_tip_gap + arrow_length + text_gap)
            ax.text(text_x_pos, total_height / 2, "Luftstrom", color=c_air, rotation=90, va='center', ha='right')

        # --- FORMATIERUNG & RÄNDER ---
        ax.set_yticks([]) 
        for spine in ['top', 'right', 'left', 'bottom']:
            ax.spines[spine].set_visible(False)

        if not for_presentation:
            # Thesis-spezifische Ränder und Achsen
            margin_left = total_display_range * 0.16
            margin_right = total_display_range * 0.04
            ax.set_xlim(-margin_left, total_length + margin_right)
            
            margin_bottom = fin_pitch * 1.5 
            margin_top = fin_pitch * 1.5
            ax.set_ylim(min(y_positions) - margin_bottom, max(y_positions) + margin_top)

            ax.spines['bottom'].set_visible(True)
            ax.set_xticks(np.arange(0, total_length + 1, layer_depth))
            import matplotlib.ticker as ticker
            ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda val, pos: f"{val:g}".replace('.', ',')))
            ax.set_xlabel("Strömungsweg [mm]", labelpad=2) 
            ax.spines['bottom'].set_bounds(0, total_length)

            legend_elements = [
                patches.Patch(facecolor=c_fin, label='Lamelle'),
                patches.Patch(facecolor=c_frost, edgecolor=c_frost_edge, linewidth=0.85, label='Reifschicht')
            ]
            ax.legend(handles=legend_elements, loc='upper center', bbox_to_anchor=(0.5, -0.35), ncol=2)
            
        else:
            # Präsentations-spezifische Gehäuse-Box und enge Ränder
            ax.set_xticks([])
            
            box_x = 0
            box_y = min(y_positions) - (fin_pitch * 0.8)
            box_width = total_length
            box_height = (max(y_positions) - min(y_positions)) + (fin_pitch * 1.6)
            
            # Ganz enge innere Limits, damit die Box das Bild dominiert (2% Puffer, damit die Linie nicht abgeschnitten wird)
            ax.set_xlim(-box_width * 0.02, box_width * 1.02)
            ax.set_ylim(box_y - (box_height * 0.02), box_y + box_height * 1.02)


    def plot_frost_distribution(self, states_history, file_name, time_step_index=-1):
        # Plot-Style laden und global 'tight' Layout sicherstellen
        colors = self.set_plot_style()
        import matplotlib as mpl
        mpl.rcParams['savefig.bbox'] = 'tight'

        output_dir = r"J:\Masterarbeit_Nils_Baumeister\02_Schriftliche_Ausarbeitung\Graphiken"
        os.makedirs(output_dir, exist_ok=True)

        cm_to_inch = 1 / 2.54
        fig_width = 15.5 * cm_to_inch
        target_ratio = 0.25 
        fig_height = (fig_width * target_ratio) + 0.5 

        fig, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=300)
        
        current_states = states_history[time_step_index]
        # UPDATE: colors hier übergeben!
        self._draw_evaporator_state(ax, current_states, colors, for_presentation=False)

        file_path = os.path.join(output_dir, file_name)
        plt.savefig(file_path) # bbox_inches='tight' weggelassen, da es nun in den rcParams steht
        plt.show()
        plt.close(fig)



    def animate_frost_distribution(self, states_history, file_name):
        import matplotlib.animation as animation
        import matplotlib as mpl
        import os
        import matplotlib.pyplot as plt

        plt.rcParams['animation.ffmpeg_path'] = r'C:\Users\mbc-nba\Downloads\ffmpeg-8.0.1-essentials_build\ffmpeg-8.0.1-essentials_build\bin\ffmpeg.exe'
        
        colors = self.set_plot_style()
        mpl.rcParams['savefig.bbox'] = None

        output_dir = r"J:\Masterarbeit_Nils_Baumeister\02_Schriftliche_Ausarbeitung\Graphiken"
        os.makedirs(output_dir, exist_ok=True)

        cm_to_inch = 1 / 2.54
        fig_width = 15.5 * cm_to_inch
        target_ratio = 0.25 
        fig_height = (fig_width * target_ratio) + 0.5 

        # 300 DPI is already very sharp for video. 
        fig, ax = plt.subplots(figsize=(fig_width, fig_height), dpi=300)
        fig.subplots_adjust(left=0.01, right=0.99, bottom=0.01, top=0.99)

        num_frames = len(states_history)
        target_duration_s = 8.0
        calculated_fps = num_frames / target_duration_s

        # --- FPS MANAGEMENT ---
        # If calculated FPS is too high for video players (>60), we skip frames to keep it smooth
        if calculated_fps > 60:
            print(f"FPS ({calculated_fps:.1f}) is too high for smooth playback. Downsampling to 24 FPS.")
            fps = 24
            step = max(1, int(num_frames / (fps * target_duration_s)))
            frames_to_plot = list(range(0, num_frames, step))
            # Ensure the absolute final state is still included
            if frames_to_plot[-1] != num_frames - 1:
                frames_to_plot.append(num_frames - 1)
        else:
            fps = calculated_fps
            frames_to_plot = list(range(num_frames))

        # --- ADD A PAUSE AT THE END ---
        # Hold the final state for 2 seconds so it doesn't abruptly vanish
        hold_seconds = 2.0
        hold_frames = [frames_to_plot[-1]] * int(fps * hold_seconds)
        frames_to_plot.extend(hold_frames)

        def update(frame_idx):
            current_states = states_history[frame_idx]
            self._draw_evaporator_state(ax, current_states, colors, for_presentation=True, num_fins_to_show=10)

        # Added cache_frame_data=False to prevent memory limits dropping the final frames
        ani = animation.FuncAnimation(
            fig, update, frames=frames_to_plot, cache_frame_data=False
        )

        file_path = os.path.join(output_dir, file_name)
        
        # --- MP4 EXPORT ---
        writer = animation.FFMpegWriter(fps=fps, bitrate=5000)
        ani.save(file_path, writer=writer, dpi=500)
            
        print(f"Video erfolgreich gespeichert: {file_path}")
        plt.close(fig)

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