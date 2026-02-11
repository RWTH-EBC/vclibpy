import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.path import Path
import CoolProp.CoolProp as CP
import pathlib
import os
import sys

# =============================================================================
# 1. KONFIGURATION
# =============================================================================

filename = "SC_Propane_Auslegung.xlsx"
FLUID = "Propane"

# Output Ordner
OUTPUT_FOLDER = "SC_Propane_Auslegung"
SUBFOLDER_VALID = "Gueltig"
SUBFOLDER_INVALID = "Ungueltig"
SUBFOLDER_SCATTER = "Scatter_Plots_Test"

# --- PHYSIKALISCHE OFFSETS ---
OFFSET_SOURCE = 8.0
PINCH_COND = 3.0
TARGET_SPREAD = 7.0

# --- LIMITS ---
LIMITS = {
    'MAX_T_DISCHARGE': 110.0,
    'MAX_P_CON_BAR': 27.0,
    'MAX_P_EL_KW': 5.0,
    'MAX_RPM': 6600
}

# Verdichterkennfeld
envelope_points = [
    (25, 70), (25, 25), (10, 10), (-35, 10),
    (-35, 65), (-10, 80), (-5, 80), (15, 80), (25, 70)
]
envelope_path = Path(envelope_points)

# =============================================================================
# 2. DATEN LADEN & MAPPING
# =============================================================================

file_path = pathlib.Path(filename)
if not file_path.exists():
    files = list(pathlib.Path(".").glob("*.xlsx"))
    if files:
        file_path = files[0]
        print(f"Warnung: '{filename}' nicht gefunden. Nutze: {file_path.name}")
    else:
        sys.exit("FEHLER: Keine Excel-Datei gefunden.")

print(f"Lade Daten aus: {file_path} ...")
df_raw = pd.read_excel(file_path)
df_raw.columns = df_raw.columns.str.strip()  # Leerzeichen entfernen

# --- MAPPING ---
col_map = {
    # Inputs
    'T_eva_in in K (Evaporator Secondary side inlet temperature)': 'T_source_in_K',
    'T_con_in in K (Condenser Secondary side inlet temperature)': 'T_sink_in_K',
    'T_ambient in K (Ambient temperature)': 'T_amb_K',

    # Performance & Parameter
    'COP in - (Coefficient of Performance)': 'COP',
    'P_el in W (Power consumption)': 'P_el_W',
    'n in Hz (Compressor speed)': 'n_Hz',
    'dT_eva_superheating in K (Evaporator superheating)': 'SH',

    # --- NEUE PARAMETER FÜR PLOTS ---
    'm_flow_ref in kg/s (Refrigerant mass flow rate)': 'm_flow_kg_s',
    'Q_con in W (Condenser refrigerant heat flow rate)': 'Q_con_W',
    'y_EV in - (Expansion valve opening)': 'y_EV',
    # --------------------------------

    # Drücke
    'p_con in Pa (Condensation pressure)': 'p_con_Pa',
    'p_eva in Pa (Evaporation pressure)': 'p_eva_Pa',
    'p_1 in Pa (Pressure in state 1)': 'P_1_Pa',
    'p_2 in Pa (Pressure in state 2)': 'P_2_Pa',
    'p_3 in Pa (Pressure in state 3)': 'P_3_Pa',
    'p_4 in Pa (Pressure in state 4)': 'P_4_Pa',

    # Temperaturen
    'T_1 in K (Refrigerant temperature at evaporator outlet)': 'T_1_K',
    'T_2 in K (Compressor outlet temperature)': 'T_2_K',
    'T_3 in K (Refrigerant temperature at condenser outlet)': 'T_3_K',
    'T_4 in K (Refrigerant temperature at evaporator inlet)': 'T_4_K',

    # Enthalpien
    'H_1 in J/K (Refrigerant enthalpy at evaporator outlet)': 'H_1_J',
    'H_2 in J/K (Compressor outlet enthalpy)': 'H_2_J',
    'H_3 in J/K (Refrigerant enthalpy at condenser outlet)': 'H_3_J',
    'H_4 in J/K (Refrigerant enthalpy at evaporator inlet)': 'H_4_J',
}

# Mapping anwenden
df = df_raw.rename(columns={k: v for k, v in col_map.items() if k in df_raw.columns})

# Fallback COP
if 'COP' not in df.columns:
    cop_candidates = [c for c in df.columns if "COP" in c]
    if cop_candidates: df = df.rename(columns={cop_candidates[0]: 'COP'})

# Filterung
df = df.dropna(subset=['COP'])
if 'H_1_J' in df.columns:
    df = df.dropna(subset=['H_1_J', 'H_2_J'])

print(f" -> {len(df)} Simulationen werden ausgewertet.")

# =============================================================================
# 3. BERECHNUNGEN & VALIDIERUNG
# =============================================================================

# Hilfsberechnungen
df['p_con_bar'] = df['p_con_Pa'] / 1e5
df['p_eva_bar'] = df['p_eva_Pa'] / 1e5

# Sättigung
def get_sat_temp_C(p_pa):
    try:
        return CP.PropsSI('T', 'P', p_pa, 'Q', 0.5, FLUID) - 273.15
    except:
        return np.nan

df['T_sat_con'] = df['p_con_Pa'].apply(get_sat_temp_C)
df['T_sat_eva'] = df['p_eva_Pa'].apply(get_sat_temp_C)

df['T_dis_C'] = df['T_2_K'] - 273.15
df['P_el_kW'] = df['P_el_W'] / 1000
df['n_rpm'] = df['n_Hz'] * 60

if 'Q_con_W' in df.columns:
    df['Q_con_kW'] = df['Q_con_W'] / 1000.0
else:
    df['Q_con_kW'] = 0.0

# Temperaturen Quelle/Senke
if 'T_source_in_K' not in df.columns:
    try:
        df['T_source_in_K'] = df[[c for c in df.columns if "T_eva_in" in c][0]]
        df['T_sink_in_K'] = df[[c for c in df.columns if "T_con_in" in c][0]]
    except:
        df['T_source_in_K'] = 273.15
        df['T_sink_in_K'] = 273.15

df['Source_C'] = df['T_source_in_K'] - 273.15
df['Sink_C'] = df['T_sink_in_K'] - 273.15
df['Flow_C'] = df['Sink_C'] + TARGET_SPREAD

# Limits Check
df['Limit_OK'] = (
        (df['T_dis_C'] <= LIMITS['MAX_T_DISCHARGE']) &
        (df['p_con_bar'] <= LIMITS['MAX_P_CON_BAR']) &
        (df['P_el_kW'] <= LIMITS['MAX_P_EL_KW']) &
        (df['n_rpm'] <= LIMITS['MAX_RPM'])
)

def check_envelope(row):
    if pd.isna(row['T_sat_eva']) or pd.isna(row['T_sat_con']): return False
    return envelope_path.contains_point((row['T_sat_eva'], row['T_sat_con']))

df['In_Envelope'] = df.apply(check_envelope, axis=1)
df['Is_Valid'] = df['Limit_OK']

def get_status_info(row):
    if not row['Limit_OK']:
        reasons = []
        if row['T_dis_C'] > LIMITS['MAX_T_DISCHARGE']: reasons.append(f"T_dis")
        if row['p_con_bar'] > LIMITS['MAX_P_CON_BAR']: reasons.append(f"p_max")
        if row['P_el_kW'] > LIMITS['MAX_P_EL_KW']: reasons.append("P_el")
        return "FEHLER: " + ",".join(reasons)
    elif not row['In_Envelope']:
        return "OK (Außerhalb Kennfeld)"
    else:
        return "OK"

df['Status_Text'] = df.apply(get_status_info, axis=1)
df['Reason'] = df['Status_Text']

# =============================================================================
# 4. PLOTTING VORBEREITUNG (Hintergrund-Daten)
# =============================================================================

print("\nBerechne Hintergrund (CoolProp)...")
try:
    T_crit = CP.PropsSI('Tcrit', FLUID)
    T_dome = np.linspace(273.15 - 60, T_crit - 2.0, 300)
    P_sat = CP.PropsSI('P', 'T', T_dome, 'Q', 0, FLUID)
    H_liq = CP.PropsSI('H', 'T', T_dome, 'Q', 0, FLUID)
    H_vap = CP.PropsSI('H', 'T', T_dome, 'Q', 1, FLUID)

    iso_temps_c = np.arange(-40, 141, 20)
    isotherms_data = []
    p_range_iso = np.logspace(np.log10(0.5e5), np.log10(90e5), 150)

    for t_c in iso_temps_c:
        t_k = t_c + 273.15
        h_iso, p_iso = [], []
        if t_k < T_crit:
            p_sat_t = CP.PropsSI('P', 'T', t_k, 'Q', 0, FLUID)
        else:
            p_sat_t = -1
        for p in p_range_iso:
            try:
                if t_k < T_crit and abs(p - p_sat_t) < 5000: continue
                h_val = CP.PropsSI('H', 'P', p, 'T', t_k, FLUID)
                h_iso.append(h_val)
                p_iso.append(p)
            except:
                pass
        if h_iso: isotherms_data.append((t_c, np.array(h_iso), np.array(p_iso)))

except Exception as e:
    print(f"Fehler bei CoolProp Background Berechnung: {e}")
    sys.exit()

# =============================================================================
# 5. TEIL A: LOG-P-H DIAGRAMME
# =============================================================================

count_valid = 0
print(f"Erstelle log-p-h Diagramme in '{OUTPUT_FOLDER}'...")

for idx, row in df.iterrows():
    try:
        h_1 = row['H_1_J'] / 1000.0
        h_2 = row['H_2_J'] / 1000.0
        h_3 = row['H_3_J'] / 1000.0
        h_4 = row['H_4_J'] / 1000.0

        p_1 = row.get('P_1_Pa', row['p_eva_Pa']) / 1e5
        p_2 = row.get('P_2_Pa', row['p_con_Pa']) / 1e5
        p_3 = row.get('P_3_Pa', row['p_con_Pa']) / 1e5
        p_4 = row.get('P_4_Pa', row['p_eva_Pa']) / 1e5

        cycle_h = np.array([h_1, h_2, h_3, h_4, h_1])
        cycle_p = np.array([p_1, p_2, p_3, p_4, p_1])

        status_folder = SUBFOLDER_VALID if row['Is_Valid'] else SUBFOLDER_INVALID
        water_folder = f"Vorlauf_{row['Flow_C']:.0f}C"
        target_dir = os.path.join(OUTPUT_FOLDER, status_folder, water_folder)
        os.makedirs(target_dir, exist_ok=True)
        fname = f"BP_{idx:03d}_n{row['n_rpm']:.0f}_SH{row['SH']:.1f}.png"

        fig, ax = plt.subplots(figsize=(10, 7))

        col = 'forestgreen' if row['Is_Valid'] else 'crimson'
        ax.plot(cycle_h, cycle_p, color=col, lw=2.0, marker='o', markersize=5, zorder=10)

        plot_xlim = [min(cycle_h) * 0.8, max(cycle_h) * 1.2]
        plot_ylim = [min(cycle_p) * 0.5, max(cycle_p) * 1.5]
        ax.set_ylim(plot_ylim)
        ax.set_xlim(plot_xlim)
        cur_xlim = ax.get_xlim()
        cur_ylim = ax.get_ylim()

        ax.plot(H_liq / 1000, P_sat / 1e5, 'k-', lw=1.5, alpha=0.6)
        ax.plot(H_vap / 1000, P_sat / 1e5, 'k-', lw=1.5, alpha=0.6)

        for t_c, h_iso, p_iso in isotherms_data:
            p_bar = p_iso / 1e5
            h_kj = h_iso / 1000
            if np.any((p_bar > cur_ylim[0]) & (p_bar < cur_ylim[1])):
                ax.plot(h_kj, p_bar, color='gray', lw=0.5, alpha=0.3)
                visible_mask = (p_bar >= cur_ylim[0]) & (p_bar <= cur_ylim[1]) & \
                               (h_kj >= cur_xlim[0]) & (h_kj <= cur_xlim[1])
                visible_indices = np.where(visible_mask)[0]
                if len(visible_indices) > 0:
                    top_idx = visible_indices[-1]
                    ax.text(h_kj[top_idx], p_bar[top_idx], f"{t_c}°C",
                            color='gray', fontsize=7, ha='left', va='bottom', clip_on=True)

        labels = ['1', '2', '3', '4']
        offs = [(10, -5), (-10, 5), (10, 5), (10, -10)]
        for i, txt in enumerate(labels):
            ax.annotate(txt, (cycle_h[i], cycle_p[i]), xytext=offs[i], textcoords='offset points',
                        fontweight='bold', color=col, fontsize=10)

        ax.set_yscale('log')
        ax.set_yticks([1, 2, 5, 10, 20, 30, 40, 50, 60])
        ax.set_yticklabels(["1", "2", "5", "10", "20", "30", "40", "50", "60"])
        ax.set_xlabel('Enthalpie [kJ/kg]')
        ax.set_ylabel('Druck [bar]')
        title_str = (f"BP {idx:03d} | T_source: {row['Source_C']:.1f}°C | T_flow: {row['Flow_C']:.1f}°C\n"
                     f"n: {row['n_rpm']:.0f} rpm | SH: {row['SH']:.1f} K | {row['Status_Text']} | COP: {row['COP']:.2f}")
        ax.set_title(title_str, fontweight='bold', fontsize=10, color='black' if row['Is_Valid'] else 'red')
        ax.grid(True, which="major", ls="-", alpha=0.3)
        ax.grid(True, which="minor", ls=":", alpha=0.1)

        plt.savefig(os.path.join(target_dir, fname), dpi=100)
        plt.close(fig)
        if row['Is_Valid']: count_valid += 1
    except Exception as e:
        print(f"Fehler bei Punkt {idx}: {e}")

print(f"Log-p-h Diagramme fertig. ({count_valid} gültige)")

# =============================================================================
# 6. TEIL B: SCATTERPLOTS (MIT STRIKTEM SANITY-CHECK)
# =============================================================================

print("\nErstelle intelligente Übersichts-Scatterplots...")
scatter_dir = os.path.join(OUTPUT_FOLDER, SUBFOLDER_SCATTER)
os.makedirs(scatter_dir, exist_ok=True)

summary_eco = []
summary_power = []

for (src, flow), group in df.groupby(['Source_C', 'Flow_C']):
    # 1. Filter: Muss im technischen Limit sein (Druck, Temp, Kennfeld)
    # 2. Filter (NEU): Muss physikalisch sinnvoll sein (Sanity Check)
    #    Wir fordern: COP > 0, Leistung > 0, Stromaufnahme > 0
    is_sane = (group['COP'] > 0) & (group['Q_con_kW'] > 0) & (group['P_el_W'] > 0)

    # Nur Gruppen betrachten, die BEIDES erfüllen
    valid_group = group[group['Limit_OK'] & is_sane]

    if not valid_group.empty:
        # A) Eco (Bester COP)
        best_cop = valid_group.sort_values('COP', ascending=False).iloc[0].to_dict()
        best_cop['Status'] = 'OK'
        best_cop['Reason'] = 'Kennfeld' if not best_cop['In_Envelope'] else ''
        summary_eco.append(best_cop)

        # B) Power (Max Q_con)
        max_power = valid_group.sort_values('Q_con_kW', ascending=False).iloc[0].to_dict()
        max_power['Status'] = 'OK'
        max_power['Reason'] = 'Kennfeld' if not max_power['In_Envelope'] else ''
        summary_power.append(max_power)
    else:
        # Kein gültiger Punkt gefunden (Entweder Limit verletzt ODER physikalischer Unsinn)

        # Grund finden: War es Limit oder Physik?
        if group['Limit_OK'].any():
            # Limits waren ok, aber Physik nicht (z.B. negativer COP)
            short_reason = "Physik (COP<0)"
        else:
            # Klassische Limit-Verletzung
            reason = group['Status_Text'].mode()[0] if not group['Status_Text'].mode().empty else "Fail"
            short_reason = reason.replace('FEHLER: ', '')

        fail_row = group.iloc[0].to_dict()
        fail_row['Status'] = 'Fail'
        fail_row['Reason'] = short_reason
        fail_row['COP'] = 0
        fail_row['Q_con_kW'] = 0
        summary_eco.append(fail_row)
        summary_power.append(fail_row)

df_eco = pd.DataFrame(summary_eco)
df_power = pd.DataFrame(summary_power)

if not df_eco.empty and not df_power.empty:

    # --- PLOT KONFIGURATION ---
    # Wir brauchen 'mask_negatives' jetzt eigentlich nicht mehr zwingend,
    # da negative Werte gar nicht mehr in 'ok' landen. Wir lassen es zur Sicherheit drin.
    plot_configs = [
        {'data': df_eco, 'col': 'COP', 'label': 'COP [-]', 'title': 'Maximal möglicher COP (Eco-Mode)',
         'cmap': 'viridis', 'mask_negatives': True},

        {'data': df_power, 'col': 'Q_con_kW', 'label': 'Heizleistung [kW]',
         'title': 'Maximale Heizleistung (Power-Mode)',
         'cmap': 'magma', 'mask_negatives': True},
        {'data': df_power, 'col': 'm_flow_kg_s', 'label': 'Massenstrom [kg/s]', 'title': 'Maximaler Massenstrom',
         'cmap': 'plasma', 'mask_negatives': True},

        {'data': df_power, 'col': 'T_dis_C', 'label': 'Heißgastemp. [°C]', 'title': 'T2 bei maximaler Leistung',
         'cmap': 'inferno', 'mask_negatives': False},

        {'data': df_power, 'col': 'y_EV', 'label': 'Ventilöffnung [-]', 'title': 'Ventil bei maximaler Leistung',
         'cmap': 'cividis', 'mask_negatives': True}
    ]

    for cfg in plot_configs:
        print(f" -> Erstelle Plot für: {cfg['title']}")

        current_df = cfg['data']
        fig, ax = plt.subplots(figsize=(11, 8))

        # 1. Envelope
        envelope_app = [(x + OFFSET_SOURCE, y - PINCH_COND) for x, y in envelope_points]
        ax.add_patch(
            patches.Polygon(envelope_app, closed=True, fill=False, edgecolor='gray', ls='--', lw=2,
                            label='Maximaler Vorlauf'))

        # 2. DIAGONALE LINIE (T_flow = T_source)
        line_min = min(current_df['Source_C'].min(), current_df['Flow_C'].min()) - 10
        line_max = max(current_df['Source_C'].max(), current_df['Flow_C'].max()) + 10
        ax.plot([line_min, line_max], [line_min, line_max], color='gray', linestyle='-.', linewidth=1.5, zorder=1,
                label=r'Grenze $T_{vl} = T_{amb}$')

        ok = current_df[current_df['Status'] == 'OK']
        fail = current_df[current_df['Status'] == 'Fail']

        # 3. GÜLTIGE PUNKTE PLOTTEN
        if not ok.empty:
            # Daten holen & bereinigen (Komplexe Zahlen fix)
            if cfg['col'] in ok.columns:
                raw_vals = ok[cfg['col']]
                try:
                    vals = raw_vals.astype(float)
                except (ValueError, TypeError):
                    vals = raw_vals.apply(lambda x: abs(x) if isinstance(x, complex) else x)
                    vals = pd.to_numeric(vals, errors='coerce').fillna(0)
            else:
                vals = np.zeros(len(ok))

            # --- LOGIK FÜR FARBSKALA ---
            if cfg.get('mask_negatives', False):
                mask_neg = vals <= 0
                mask_pos = vals > 0

                # Limits NUR basierend auf positiven Werten berechnen
                if mask_pos.any():
                    vmin_val = vals[mask_pos].min()
                    vmax_val = vals[mask_pos].max()

                    sc = ax.scatter(ok.loc[mask_pos, 'Source_C'], ok.loc[mask_pos, 'Flow_C'],
                                    c=vals[mask_pos], cmap=cfg['cmap'],
                                    vmin=vmin_val, vmax=vmax_val,
                                    s=150, marker='o', edgecolors='k', label='Machbar', zorder=5)
                    cbar = plt.colorbar(sc, ax=ax)
                    cbar.set_label(cfg['label'], fontsize=12)

                # Falls durch Rundungsfehler doch noch was <= 0 ist
                if mask_neg.any():
                    ax.scatter(ok.loc[mask_neg, 'Source_C'], ok.loc[mask_neg, 'Flow_C'],
                               c='darkgray', s=150, marker='o', edgecolors='k',
                               label=r'Wert $\leq$ 0 (ausgeblendet)', zorder=4)
            else:
                # Standard (Temperatur)
                sc = ax.scatter(ok['Source_C'], ok['Flow_C'], c=vals, cmap=cfg['cmap'],
                                s=150, marker='o', edgecolors='k', label='Machbar', zorder=5)
                cbar = plt.colorbar(sc, ax=ax)
                cbar.set_label(cfg['label'], fontsize=12)

            # Warnung Kennfeld
            warn = ok[ok['Reason'] == 'Kennfeld']
            if not warn.empty:
                ax.scatter(warn['Source_C'], warn['Flow_C'], s=150, facecolors='none', edgecolors='orange', lw=2,
                           label='Außerh. Kennfeld', zorder=6)

        # 4. UNGÜLTIGE PUNKTE
        if not fail.empty:
            ax.scatter(fail['Source_C'], fail['Flow_C'], c='crimson', s=150, marker='X', edgecolors='k',
                       label='Nicht machbar', zorder=5)
            for _, row in fail.iterrows():
                reason_txt = str(row['Reason'])
                if len(reason_txt) > 15: reason_txt = reason_txt[:12] + ".."
                ax.text(row['Source_C'], row['Flow_C'] + 1.5, reason_txt,
                        ha='center', va='bottom', color='darkred', fontsize=8, fontweight='bold', zorder=7)

        # Achsen & Layout
        ax.set_title(f"{cfg['title']}", fontsize=14)
        ax.set_xlabel('T_source [°C]', fontsize=12)
        ax.set_ylabel('T_flow [°C]', fontsize=12)
        ax.legend(loc='upper left')
        ax.grid(True, ls='--', alpha=0.7)
        ax.set_xlim(-40, 45)
        ax.set_ylim(0, 95)

        safe_title = cfg['title'].replace(" ", "_").replace(".", "").replace("/", "-").replace("(", "").replace(")", "")
        plt.tight_layout()
        plt.savefig(os.path.join(scatter_dir, f"Scatter_{safe_title}.png"), dpi=150)
        plt.close(fig)

print("\nFertig! Alle Diagramme erstellt.")