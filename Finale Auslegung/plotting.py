import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.path as mplPath
import seaborn as sns
import pandas as pd
import numpy as np
import os
import CoolProp.CoolProp as CP

# --- KONFIGURATION ---
FLUID = "Propane"

LIMITS = {
    'MAX_T_DISCHARGE': 110.0,
    'MAX_P_CON_BAR': 27.0,  # Dein Limit (ggf. 35.0 testen)
    'MAX_P_EL_KW': 5.0,
    'MAX_RPM': 6600
}

# --- PHYSIKALISCHE RANDBEDINGUNGEN ---
OFFSET_SOURCE = 8.0
PINCH_COND = 3.0
SPREAD_COND = 5.0

# Verdichter-Envelope
envelope_points_sat = [
    (25, 70), (25, 25), (10, 10), (-35, 10),
    (-35, 65), (-10, 80), (-5, 80), (15, 80), (25, 70)
]
envelope_points_real = [(x + OFFSET_SOURCE, y - PINCH_COND) for x, y in envelope_points_sat]
envelope_path_real = mplPath.Path(envelope_points_real)


# --- TEIL 1: DATA LOADING & MAPPING ---

def load_and_map_data(filepath):
    if not os.path.exists(filepath):
        print(f"FEHLER: Datei '{filepath}' nicht gefunden.")
        return None

    print(f"[Plotting] Lade Daten aus {filepath}...")
    df_raw = pd.read_excel(filepath, na_values=['NA', 'n/a', 'nan', '-'])
    df_raw.columns = df_raw.columns.str.strip()

    # Mapping Dictionary
    col_map = {
        'T_eva_in in K (Evaporator Secondary side inlet temperature)': 'T_source_in_K',
        'T_con_in in K (Condenser Secondary side inlet temperature)': 'T_sink_in_K',
        'COP in - (Coefficient of Performance)': 'COP',
        'P_el in W (Power consumption)': 'P_el_W',
        'Q_con in W (Condenser refrigerant heat flow rate)': 'Q_con_W',
        'n in Hz (None)': 'n_Hz',
        'dT_eva_superheating in K (None)': 'SH',
        'm_flow_ref in kg/s (Refrigerant mass flow rate)': 'm_flow_kg_s',
        'opening in - (Opening High-Side EV)': 'y_EV',


        # Zustände 1-7 (T, p, h, rho)
        'T_1 in K (Temperature in state 1)': 'T_1_K', 'H_1 in J/kg (Enthalpy in state 1)': 'h_1_Jkg',
        'p_1 in Pa (Pressure in state 1)': 'p_1_Pa', 'rho_1 in kg/m3 (Density in state 1)': 'rho_1_kgm3',
        'T_2 in K (Temperature in state 2)': 'T_2_K', 'H_2 in J/kg (Enthalpy in state 2)': 'h_2_Jkg',
        'p_2 in Pa (Pressure in state 2)': 'p_2_Pa', 'rho_2 in kg/m3 (Density in state 2)': 'rho_2_kgm3',
        'T_3 in K (Temperature in state 3)': 'T_3_K', 'H_3 in J/kg (Enthalpy in state 3)': 'h_3_Jkg',
        'p_3 in Pa (Pressure in state 3)': 'p_3_Pa', 'rho_3 in kg/m3 (Density in state 3)': 'rho_3_kgm3',
        'T_4 in K (Temperature in state 4)': 'T_4_K', 'H_4 in J/kg (Enthalpy in state 4)': 'h_4_Jkg',
        'p_4 in Pa (Pressure in state 4)': 'p_4_Pa', 'rho_4 in kg/m3 (Density in state 4)': 'rho_4_kgm3',
        'T_5 in K (Temperature in state 5)': 'T_5_K', 'H_5 in J/kg (Enthalpy in state 5)': 'h_5_Jkg',
        'p_5 in Pa (Pressure in state 5)': 'p_5_Pa', 'rho_5 in kg/m3 (Density in state 5)': 'rho_5_kgm3',
        'T_6 in K (Temperature in state 6)': 'T_6_K', 'H_6 in J/kg (Enthalpy in state 6)': 'h_6_Jkg',
        'p_6 in Pa (Pressure in state 6)': 'p_6_Pa', 'rho_6 in kg/m3 (Density in state 6)': 'rho_6_kgm3',
        'T_7 in K (Temperature in state 7)': 'T_7_K', 'H_7 in J/kg (Enthalpy in state 7)': 'h_7_Jkg',
        'p_7 in Pa (Pressure in state 7)': 'p_7_Pa', 'rho_7 in kg/m3 (Density in state 7)': 'rho_7_kgm3',
    }

    df = df_raw.rename(columns=col_map)

    # Erzwinge numerische Werte
    for col in df.columns:
        try:
            df[col] = pd.to_numeric(df[col], errors='coerce')
        except:
            pass

    df = df.copy()

    # Einheiten & Basiswerte
    if 'P_el_W' in df.columns: df['P_el_kW'] = df['P_el_W'] / 1000
    if 'Q_con_W' in df.columns: df['Q_con_kW'] = df['Q_con_W'] / 1000
    if 'n_Hz' in df.columns: df['n_rpm'] = df['n_Hz'] * 110
    if 'm_flow_kg_s' in df.columns: df['m_flow_ref'] = df['m_flow_kg_s']

    # Scatter Achsen
    if 'T_source_in_K' in df.columns: df['Source_C'] = df['T_source_in_K'] - 273.15
    if 'T_sink_in_K' in df.columns: df['Sink_C'] = df['T_sink_in_K'] - 273.15
    if 'Sink_C' in df.columns: df['Flow_C'] = df['Sink_C'] + SPREAD_COND

    # Zustandsgrößen aufbereiten (p, T, h, rho)
    for i in range(1, 8):
        # Druck Pa -> Bar
        if f'p_{i}_Pa' in df.columns:
            df[f'p{i}'] = df[f'p_{i}_Pa'] / 100000.0
            df[f'p_{i}_bar'] = df[f'p{i}']
        # Temp K -> C
        if f'T_{i}_K' in df.columns:
            df[f'T{i}'] = df[f'T_{i}_K'] - 273.15
        # Enthalpie J -> kJ
        if f'h_{i}_Jkg' in df.columns:
            df[f'h_{i}_kJkg'] = df[f'h_{i}_Jkg'] / 1000.0
        # Dichte (Mapping auf rho{i})
        if f'rho_{i}_kgm3' in df.columns:
            df[f'rho{i}'] = df[f'rho_{i}_kgm3']

    # --- DRUCKDIFFERENZEN BERECHNEN (NEU) ---
    # dp_High_EV = p4 - p5
    #if 'p4' in df.columns and 'p5' in df.columns:
        #df['dp_High_EV'] = df['p4'] - df['p5']
    # dp_Low_EV = p5 - p6
    #if 'p5' in df.columns and 'p6' in df.columns:
        #df['dp_Low_EV'] = df['p5'] - df['p6']

    # Envelope Check & Limits
    limit_t = float(LIMITS['MAX_T_DISCHARGE'])
    limit_p = float(LIMITS['MAX_P_CON_BAR'])

    if 'T2' in df.columns:
        df['T_dis_C'] = df['T2']
    elif 'T_2_K' in df.columns:
        df['T_dis_C'] = df['T_2_K'] - 273.15
    else:
        df['T_dis_C'] = 0

    df['p_con_check'] = df.get('p3', df.get('p2', 0))

    def get_fail_reason(row):
        reasons = []
        if row['T_dis_C'] > limit_t: reasons.append("T_max")
        if row['p_con_check'] > limit_p: reasons.append("p_max")
        if not reasons: return "OK"
        return "+".join(reasons)

    df['Fail_Reason'] = df.apply(get_fail_reason, axis=1)
    df['Limit_OK'] = (df['Fail_Reason'] == "OK")

    def check_env(row):
        if pd.isna(row.get('Source_C')) or pd.isna(row.get('Flow_C')): return False
        return envelope_path_real.contains_point((row['Source_C'], row['Flow_C']), radius=0.001) or \
            envelope_path_real.contains_point((row['Source_C'], row['Flow_C']))

    df['In_Envelope'] = df.apply(check_env, axis=1)

    conditions = [
        (df['Limit_OK'] & df['In_Envelope']),
        (df['Limit_OK'] & ~df['In_Envelope']),
        (~df['Limit_OK'])
    ]
    df['Status_Code'] = np.select(conditions, ['OK', 'Out_Env', 'Limit_Fail'], default='Error')

    # Aufräumen für Scatter
    df['Plot_Status'] = np.where(df['Limit_OK'], 'OK', 'Fail')

    return df.dropna(subset=['COP'])


# --- TEIL 2: BOXPLOTS & VIOLINPLOTS (NUR GÜLTIGE!) ---

def create_boxplots_internal(df, output_folder):
    c_p, c_t = 'skyblue', 'salmon'

    # 1. FILTERN: Nur gültige Punkte nutzen!
    df_valid = df[df['Limit_OK']].copy()

    if df_valid.empty:
        print("WARNUNG: Keine gültigen Daten für Boxplots vorhanden.")
        return

    # Sortierschlüssel für Spalten
    def sort_key(name):
        if name.startswith('dp'): return 100  # dp am Ende
        try:
            return int(''.join(filter(str.isdigit, name)))
        except:
            return 50

    # A) DRUCK (p1..p7 UND dp_...)
    p_cols = [c for c in df_valid.columns if
              (c.startswith('p') and len(c) <= 3 and c[1:].isdigit())]
    p_cols = sorted(p_cols, key=sort_key)

    if p_cols:
        df_p = df_valid.melt(value_vars=p_cols, var_name='Pos', value_name='Bar')
        fig, axes = plt.subplots(1, 2, figsize=(16, 6))

        # Boxplot
        sns.boxplot(data=df_p, x='Pos', y='Bar', ax=axes[0], color=c_p)
        axes[0].tick_params(axis='x', rotation=45)

        # Violinplot (cut=0 !)
        sns.violinplot(data=df_p, x='Pos', y='Bar', ax=axes[1], color=c_p, inner="quartile", cut=0)
        axes[1].tick_params(axis='x', rotation=45)

        axes[0].set_title('Druck Verteilung (Nur gültige BPs)');
        axes[1].set_title('Druck Dichte (cut=0)')
        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, "Statistik_Druck.png"), dpi=150)
        plt.close()

    # B) TEMPERATUR (T1..T7)
    t_cols = sorted([c for c in df_valid.columns if c.startswith('T') and len(c) <= 3 and c[1:].isdigit()],
                    key=lambda x: int(x[1:]))
    if t_cols:
        df_t = df_valid.melt(value_vars=t_cols, var_name='Pos', value_name='Grad_C')
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))

        sns.boxplot(data=df_t, x='Pos', y='Grad_C', ax=axes[0], color=c_t)
        axes[0].axhline(y=LIMITS['MAX_T_DISCHARGE'], color='red', ls='--', label='Max T_dis')

        sns.violinplot(data=df_t, x='Pos', y='Grad_C', ax=axes[1], color=c_t, inner="quartile", cut=0)

        axes[0].set_title('Temperatur Verteilung (Nur gültige BPs)');
        axes[1].set_title('Temperatur Dichte (cut=0)')
        plt.tight_layout()
        plt.savefig(os.path.join(output_folder, "Statistik_Temperatur.png"), dpi=150)
        plt.close()


# --- TEIL 3: EXCEL EXPORT (NUR GÜLTIGE!) ---

def export_sensor_data(df, output_folder):
    # 1. FILTERN: Nur gültige Punkte für die Sensor-Auslegung!
    df_valid = df[df['Limit_OK']].copy()

    if df_valid.empty:
        print("WARNUNG: Keine gültigen Daten für Sensor-Excel.")
        return

    summary_data = []

    # Globale Werte (Massenstrom) aus gültigen Daten
    if 'm_flow_ref' in df_valid.columns:
        m_min, m_max = df_valid['m_flow_ref'].min(), df_valid['m_flow_ref'].max()
    else:
        m_min, m_max = None, None

    for i in range(1, 8):
        row = {'Zustandspunkt': str(i)}

        # Temperatur
        if f'T{i}' in df_valid.columns:
            row['T_min [°C]'] = df_valid[f'T{i}'].min()
            row['T_max [°C]'] = df_valid[f'T{i}'].max()

        # Druck
        if f'p{i}' in df_valid.columns:
            row['p_min [bar]'] = df_valid[f'p{i}'].min()
            row['p_max [bar]'] = df_valid[f'p{i}'].max()

        # Dichte
        if f'rho{i}' in df_valid.columns:
            row['Dichte_min [kg/m³]'] = df_valid[f'rho{i}'].min()
            row['Dichte_max [kg/m³]'] = df_valid[f'rho{i}'].max()

        # Massenstrom
        row['m_flow_min [kg/s]'] = m_min
        row['m_flow_max [kg/s]'] = m_max

        summary_data.append(row)

    out_df = pd.DataFrame(summary_data)
    cols = ['Zustandspunkt', 'T_min [°C]', 'T_max [°C]', 'p_min [bar]', 'p_max [bar]',
            'm_flow_min [kg/s]', 'm_flow_max [kg/s]', 'Dichte_min [kg/m³]', 'Dichte_max [kg/m³]']

    final_cols = [c for c in cols if c in out_df.columns]
    out_df = out_df[final_cols]

    out_df.to_excel(os.path.join(output_folder, "Sensorauslegung_Werte.xlsx"), index=False)
    print(f" -> Sensor-Excel erstellt (Basierend auf {len(df_valid)} gültigen Punkten).")


# --- TEIL 4: LOG-P-H DIAGRAMME (MIT UNTERORDNERN) ---

def create_logph_diagrams(df, output_folder):
    base_dir = os.path.join(output_folder, "LogPH_Diagramme")
    valid_dir = os.path.join(base_dir, "Gueltig")
    invalid_dir = os.path.join(base_dir, "Ungueltig")
    os.makedirs(valid_dir, exist_ok=True);
    os.makedirs(invalid_dir, exist_ok=True)

    print("[Plotting] Berechne CoolProp Hintergrund...")
    try:
        T_crit = CP.PropsSI('Tcrit', FLUID)
        T_dome = np.linspace(273.15 - 60, T_crit - 0.5, 300)
        P_sat_dome = CP.PropsSI('P', 'T', T_dome, 'Q', 0, FLUID) / 1e5
        H_liq_dome = CP.PropsSI('H', 'T', T_dome, 'Q', 0, FLUID) / 1000.0
        H_vap_dome = CP.PropsSI('H', 'T', T_dome, 'Q', 1, FLUID) / 1000.0

        iso_temps = np.arange(-40, 151, 20)
        isotherms = []
        p_range_pa = np.logspace(np.log10(0.8e5), np.log10(95e5), 150)
        for t_c in iso_temps:
            t_k = t_c + 273.15
            h_iso, p_iso = [], []
            for p_pa in p_range_pa:
                try:
                    h = CP.PropsSI('H', 'P', p_pa, 'T', t_k, FLUID) / 1000.0
                    h_iso.append(h)
                    p_iso.append(p_pa / 1e5)
                except:
                    pass
            if h_iso: isotherms.append((t_c, h_iso, p_iso))
    except:
        print("CoolProp Fehler.");
        return

    print(f"[Plotting] Erstelle LogPH Diagramme...")
    count = 0
    for idx, row in df.iterrows():
        try:
            h_vals = [row[f'h_{i}_kJkg'] for i in range(1, 8)]
            p_vals = [row[f'p_{i}_bar'] for i in range(1, 8)]
            h_vals.append(h_vals[0]);
            p_vals.append(p_vals[0])
        except:
            continue

        fig, ax = plt.subplots(figsize=(10, 7))
        ax.plot(H_liq_dome, P_sat_dome, 'k-', lw=1, alpha=0.5)
        ax.plot(H_vap_dome, P_sat_dome, 'k-', lw=1, alpha=0.5)

        xmin, xmax, ymin, ymax = min(h_vals), max(h_vals), min(p_vals), max(p_vals)
        ax.set_xlim(xmin - 100, xmax + 100);
        ax.set_ylim(ymin * 0.7, ymax * 1.5)
        cur_xlim, cur_ylim = ax.get_xlim(), ax.get_ylim()

        for t, h_iso, p_iso in isotherms:
            h_arr, p_arr = np.array(h_iso), np.array(p_iso)
            if np.any((p_arr > cur_ylim[0]) & (p_arr < cur_ylim[1])):
                ax.plot(h_iso, p_iso, color='gray', lw=0.5, alpha=0.3)
                visible = (p_arr >= cur_ylim[0]) & (p_arr <= cur_ylim[1]) & (h_arr >= cur_xlim[0]) & (
                            h_arr <= cur_xlim[1])
                if np.any(visible):
                    idx_lbl = np.where(visible)[0][-1]
                    ax.text(h_arr[idx_lbl], p_arr[idx_lbl], f"{t}°C", color='gray', fontsize=7, clip_on=True)

        col = 'forestgreen' if row['Limit_OK'] else 'crimson'
        ax.plot(h_vals, p_vals, color=col, marker='o', markersize=5, lw=2)

        lbls = ['1', '2', '3', '4', '5', '6', '7']
        offs = [(10, -10), (-10, 5), (10, 5), (10, 5), (10, -10), (-15, -10), (10, -10)]
        for i, txt in enumerate(lbls):
            ax.annotate(txt, (h_vals[i], p_vals[i]), xytext=offs[i], textcoords='offset points', color=col,
                        fontweight='bold')

        ax.set_yscale('log')
        ax.set_yticks([1, 2, 5, 10, 20, 30, 40, 50, 60])
        ax.set_yticklabels(["1", "2", "5", "10", "20", "30", "40", "50", "60"])
        ax.set_xlabel("Enthalpie [kJ/kg]");
        ax.set_ylabel("Druck [bar]")

        t_str = (
            f"BP {idx:03d} | Src: {row.get('Source_C', 0):.1f}°C | Flow: {row.get('Flow_C', 0):.1f}°C | n: {row.get('n_rpm', 0):.0f}\n"
            f"SH: {row.get('SH', 0):.1f}K | EV: {row.get('y_EV', 0):.2f} | Status: {row.get('Status_Code', '')}")
        ax.set_title(t_str, color='black' if row['Limit_OK'] else 'red', fontweight='bold', fontsize=9)
        ax.grid(True, which='major', alpha=0.3)

        target_dir = valid_dir if row['Limit_OK'] else invalid_dir
        plt.savefig(os.path.join(target_dir, f"LogPH_BP{idx:03d}.png"), dpi=100)
        plt.close(fig)
        count += 1
    print(f"[Plotting] {count} LogPH erstellt.")


# --- TEIL 5: SCATTERPLOTS (BEST-OF LOGIK) ---

def create_scatter_plots(df, output_folder):
    scatter_dir = os.path.join(output_folder, "Scatter_Maps")
    os.makedirs(scatter_dir, exist_ok=True)

    df_plot = df.dropna(subset=['COP'])
    if df_plot.empty: return

    print("[Plotting] Gruppiere Daten für Scatterplots (Best-Of Wahl)...")
    summary_eco = []
    summary_power = []

    for (src, flow), group in df_plot.groupby(['Source_C', 'Flow_C']):
        # Gültige Punkte (Limit OK)
        valid_group = group[group['Limit_OK'] & (group['COP'] > 0)]

        if not valid_group.empty:
            best_cop = valid_group.loc[valid_group['COP'].idxmax()].to_dict()
            best_cop['Plot_Status'] = 'OK'
            summary_eco.append(best_cop)

            max_pwr = valid_group.loc[valid_group['Q_con_kW'].idxmax()].to_dict()
            max_pwr['Plot_Status'] = 'OK'
            summary_power.append(max_pwr)
        else:
            fail_row = group.iloc[0].to_dict()
            fail_row['Plot_Status'] = 'Fail'
            summary_eco.append(fail_row);
            summary_power.append(fail_row)

    df_eco = pd.DataFrame(summary_eco)
    df_power = pd.DataFrame(summary_power)
    if df_power.empty: return

    plots = [
        {'data': df_eco, 'col': 'COP', 'title': 'Map_COP', 'cmap': 'viridis', 'label': 'COP [-]'},
        {'data': df_power, 'col': 'Q_con_kW', 'title': 'Map_Heizleistung', 'cmap': 'magma', 'label': 'Q_con [kW]'},
        {'data': df_power, 'col': 'm_flow_ref', 'title': 'Map_Massenstrom', 'cmap': 'plasma', 'label': 'm_dot [kg/s]'},
        {'data': df_power, 'col': 'y_EV', 'title': 'Map_Ventil', 'cmap': 'cividis', 'label': 'Opening [-]'}
    ]

    fail_markers = {
        'T_max': {'color': 'red', 'marker': 'x', 'label': 'Fail: T_dis > Limit'},
        'p_max': {'color': 'darkorange', 'marker': 'x', 'label': 'Fail: p_max > Limit'},
        'T_max+p_max': {'color': 'purple', 'marker': 'x', 'label': 'Fail: T & p > Limit'}
    }

    for cfg in plots:
        current_df = cfg['data']
        col = cfg['col']
        if col not in current_df.columns: continue

        fig, ax = plt.subplots(figsize=(11, 8.5))
        ax.add_patch(patches.Polygon(envelope_points_real, closed=True, fill=False, ec='black', lw=2, ls='-',
                                     label='Verdichter Envelope'))

        line_min = min(current_df['Source_C'].min(), current_df['Flow_C'].min()) - 10
        line_max = max(current_df['Source_C'].max(), current_df['Flow_C'].max()) + 10
        ax.plot([line_min, line_max], [line_min, line_max], color='gray', ls='-.', lw=1.5, label='Grenze Tv=Tamb')

        mask_ok = (current_df['Plot_Status'] == 'OK')
        mask_env = current_df['In_Envelope'].fillna(False).astype(bool)

        m_ok_env = mask_ok & mask_env
        if m_ok_env.any():
            sc = ax.scatter(current_df.loc[m_ok_env, 'Source_C'], current_df.loc[m_ok_env, 'Flow_C'],
                            c=current_df.loc[m_ok_env, col], cmap=cfg['cmap'],
                            s=120, marker='o', edgecolors='k', label='Gültig (Best-Of)', zorder=5)
            plt.colorbar(sc, label=cfg['label'], fraction=0.046, pad=0.04)

        m_ok_out = mask_ok & ~mask_env
        if m_ok_out.any():
            ax.scatter(current_df.loc[m_ok_out, 'Source_C'], current_df.loc[m_ok_out, 'Flow_C'],
                       facecolors='none', edgecolors='orange', s=140, lw=2, marker='o', label='Außerh. Env.', zorder=6)
            ax.scatter(current_df.loc[m_ok_out, 'Source_C'], current_df.loc[m_ok_out, 'Flow_C'],
                       c=current_df.loc[m_ok_out, col], cmap=cfg['cmap'], s=40, zorder=6)

        mask_fail = (current_df['Plot_Status'] == 'Fail')
        if mask_fail.any():
            fails = current_df[mask_fail]
            if 'Fail_Reason' in fails.columns:
                for reason, grp in fails.groupby('Fail_Reason'):
                    style = fail_markers.get(reason, {'color': 'black', 'marker': 'x', 'label': f'Fail: {reason}'})
                    ax.scatter(grp['Source_C'], grp['Flow_C'],
                               c=style['color'], marker=style['marker'], s=100, lw=2,
                               label=style['label'], zorder=7)
            else:
                ax.scatter(fails['Source_C'], fails['Flow_C'], c='red', marker='x', s=100, label='Limit Fail', zorder=7)

        ax.set_xlabel("Außenlufttemperatur [°C]")
        ax.set_ylabel("Vorlauftemperatur [°C]")
        ax.set_title(cfg['title'] + " (Best-Of)")
        ax.grid(True, alpha=0.5)
        ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.1), ncol=3)
        plt.subplots_adjust(bottom=0.2)
        plt.savefig(os.path.join(scatter_dir, f"{cfg['title']}.png"), dpi=150)
        plt.close(fig)
    print("[Plotting] Scatterplots erstellt.")


# --- HAUPTFUNKTION ---

def create_all_plots(excel_file, output_folder="Plots_Ergebnisse"):
    print(f"\n--- STARTE PLOTTING MODUL ---")
    os.makedirs(output_folder, exist_ok=True)
    df = load_and_map_data(excel_file)
    if df is not None:
        export_sensor_data(df, output_folder)
        create_boxplots_internal(df, output_folder)
        create_logph_diagrams(df, output_folder)
        create_scatter_plots(df, output_folder)
        print(f"Plotting abgeschlossen. Ordner: '{output_folder}'")
    else:
        print("Keine Daten.")