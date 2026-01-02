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
# 1. EINSTELLUNGEN
# =============================================================================

filename = "IHX_Propane_VarFlowHum_ValveStudy.xlsx"
FLUID = "Propane"
OUTPUT_FOLDER = "IHX_Propane_VarFlowHum_ValveStudy"
SUBFOLDER_VALID = "Gueltig"
SUBFOLDER_INVALID = "Ungueltig"
SUBFOLDER_SCATTER = "Scatter_Plots"

# Physikalische Offsets
OFFSET_SOURCE = 8.0
TARGET_SPREAD = 7.0
PINCH_COND = 3.0

# Technische Grenzen
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
# 2. DATEN LADEN & EXAKTES MAPPING
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

# Whitespace entfernen, um Matching-Fehler zu vermeiden
df_raw.columns = df_raw.columns.str.strip()

# --- NEUES MAPPING BASIEREND AUF DEINEN SPALTEN ---
col_map = {
    # Allgemeine Infos / Temperaturen Sekundärseite
    'T_eva_in in K (Evaporator Secondary side inlet temperature)': 'T_source_in_K',
    'T_con_in in K (Condenser Secondary side inlet temperature)': 'T_sink_in_K',

    # Leistungsdaten
    'COP in - (Coefficient of Performance)': 'COP',
    'P_el in W (Power consumption)': 'P_el_W',
    'Q_con in W (Condenser refrigerant heat flow rate)': 'Q_con_W',

    # Betriebsparameter
    'n in Hz (None)': 'n_Hz',
    'dT_eva_superheating in K (None)': 'SH',
    'm_flow_ref in kg/s (Refrigerant mass flow rate)': 'm_flow_kg_s',
    'opening in - (Opening High-Side EV)': 'y_EV',  # Valve Opening
    'phi in - (None)': 'phi', # <--- NEU: Feuchte

    # Zustand 1
    'T_1 in K (Temperature in state 1)': 'T_1_K',
    'H_1 in J/kg (Enthalpy in state 1)': 'h_1_Jkg',
    'p_1 in Pa (Pressure in state 1)': 'p_1_Pa',

    # Zustand 2
    'T_2 in K (Temperature in state 2)': 'T_2_K',
    'H_2 in J/kg (Enthalpy in state 2)': 'h_2_Jkg',
    'p_2 in Pa (Pressure in state 2)': 'p_2_Pa',

    # Zustand 3
    'T_3 in K (Temperature in state 3)': 'T_3_K',
    'H_3 in J/kg (Enthalpy in state 3)': 'h_3_Jkg',
    'p_3 in Pa (Pressure in state 3)': 'p_3_Pa',

    # Zustand 4
    'T_4 in K (Temperature in state 4)': 'T_4_K',
    'H_4 in J/kg (Enthalpy in state 4)': 'h_4_Jkg',
    'p_4 in Pa (Pressure in state 4)': 'p_4_Pa',

    # Zustand 5
    'T_5 in K (Temperature in state 5)': 'T_5_K',
    'H_5 in J/kg (Enthalpy in state 5)': 'h_5_Jkg',
    'p_5 in Pa (Pressure in state 5)': 'p_5_Pa',

    # Zustand 6
    'T_6 in K (Temperature in state 6)': 'T_6_K',
    'H_6 in J/kg (Enthalpy in state 6)': 'h_6_Jkg',
    'p_6 in Pa (Pressure in state 6)': 'p_6_Pa',

    # Zustand 7
    'T_7 in K (Temperature in state 7)': 'T_7_K',
    'H_7 in J/kg (Enthalpy in state 7)': 'h_7_Jkg',
    'p_7 in Pa (Pressure in state 7)': 'p_7_Pa',
}

# Spalten umbenennen
df = df_raw.rename(columns=col_map)

# Sicherheitscheck: Wurde COP gefunden?
if 'COP' not in df.columns:
    print("FEHLER: Konnte die Spalte 'COP' nicht mappen. Bitte Spaltennamen prüfen:")
    print(df_raw.columns.tolist())
    sys.exit()

df = df.dropna(subset=['COP'])

# Prüfen ob Mapping für Enthalpien geklappt hat
required_h = ['h_1_Jkg', 'h_2_Jkg']
if not all(col in df.columns for col in required_h):
    print("FEHLER: Konnte die H_... Spalten nicht finden. Mapping fehlgeschlagen.")
    sys.exit()

print(f" -> {len(df)} Zeilen bereit.")

# =============================================================================
# 3. UMRECHNUNG & BERECHNUNG
# =============================================================================

# 1. Enthalpie: J/kg -> kJ/kg
for i in range(1, 8):
    col_j = f'h_{i}_Jkg'
    col_kj = f'h_{i}_kJkg'
    if col_j in df.columns:
        df[col_kj] = df[col_j] / 1000.0

# 2. Druck: Pa -> Bar
for i in range(1, 8):
    col_pa = f'p_{i}_Pa'
    col_bar = f'p_{i}_bar'
    if col_pa in df.columns:
        df[col_bar] = df[col_pa] / 100000.0

# 3. Temperaturen & Leistung
df['T_dis_C'] = df['T_2_K'] - 273.15
df['P_el_kW'] = df['P_el_W'] / 1000
df['n_rpm'] = df['n_Hz'] * 60

# --- Q_con in kW umrechnen für Plots ---
if 'Q_con_W' in df.columns:
    df['Q_con_kW'] = df['Q_con_W'] / 1000.0
else:
    df['Q_con_kW'] = 0.0

# Falls 'phi' nicht existiert (z.B. alte Datei), mit 0 auffüllen
if 'phi' not in df.columns:
    df['phi'] = 0.0

# 4. Limits Check
df['p_con_check'] = df['p_3_bar']
df['p_eva_check'] = df['p_7_bar']
# Fallback falls p_3 fehlt
if 'p_3_bar' not in df.columns and 'p_2_bar' in df.columns:
    df['p_con_check'] = df['p_2_bar']


def get_sat_temp(p_bar):
    try:
        return CP.PropsSI('T', 'P', p_bar * 1e5, 'Q', 0.5, FLUID) - 273.15
    except:
        return np.nan


df['T_sat_con'] = df['p_con_check'].apply(get_sat_temp)
df['T_sat_eva'] = df['p_eva_check'].apply(get_sat_temp)

# Quell- und Senkentemperaturen (Celius)
df['Source_C'] = df['T_source_in_K'] - 273.15
df['Sink_C'] = df['T_sink_in_K'] - 273.15
df['Flow_C'] = df['Sink_C'] + TARGET_SPREAD

# VALIDIERUNG
df['Limit_OK'] = (
        (df['T_dis_C'] <= LIMITS['MAX_T_DISCHARGE']) &
        (df['p_con_check'] <= LIMITS['MAX_P_CON_BAR']) &
        (df['P_el_kW'] <= LIMITS['MAX_P_EL_KW']) &
        (df['n_rpm'] <= LIMITS['MAX_RPM'])
)


def check_env(row):
    if pd.isna(row['T_sat_eva']) or pd.isna(row['T_sat_con']): return False
    return envelope_path.contains_point((row['T_sat_eva'], row['T_sat_con']))


df['In_Envelope'] = df.apply(check_env, axis=1)
df['Is_Valid'] = df['Limit_OK']


def get_status(row):
    if not row['Limit_OK']:
        errs = []
        if row['T_dis_C'] > LIMITS['MAX_T_DISCHARGE']: errs.append(f"T_dis")
        if row['p_con_check'] > LIMITS['MAX_P_CON_BAR']: errs.append(f"p_max")
        if row['P_el_kW'] > LIMITS['MAX_P_EL_KW']: errs.append("P_el")
        return "FEHLER: " + ",".join(errs)
    if not row['In_Envelope']: return "OK (Außerh. Kennfeld)"
    return "OK"


df['Status_Text'] = df.apply(get_status, axis=1)

# =============================================================================
# 4. PLOTTING LOG-P-H
# =============================================================================

print("\nBerechne Hintergrund (CoolProp)...")
try:
    T_crit = CP.PropsSI('Tcrit', FLUID)
    T_dome = np.linspace(273.15 - 60, T_crit - 0.5, 300)
    P_sat_dome = CP.PropsSI('P', 'T', T_dome, 'Q', 0, FLUID) / 1e5
    H_liq_dome = CP.PropsSI('H', 'T', T_dome, 'Q', 0, FLUID) / 1000.0
    H_vap_dome = CP.PropsSI('H', 'T', T_dome, 'Q', 1, FLUID) / 1000.0

    iso_temps = np.arange(-40, 151, 20)
    isotherms = []
    p_range = np.logspace(np.log10(0.8e5), np.log10(95e5), 150)

    for t_c in iso_temps:
        t_k = t_c + 273.15
        h_i, p_i = [], []
        for p in p_range:
            try:
                h = CP.PropsSI('H', 'P', p, 'T', t_k, FLUID) / 1000.0
                h_i.append(h)
                p_i.append(p / 1e5)
            except:
                pass
        if h_i: isotherms.append((t_c, h_i, p_i))
except Exception as e:
    print(f"Fehler bei CoolProp: {e}")
    sys.exit()

print(f"Erstelle Diagramme in '{OUTPUT_FOLDER}'...")

count_valid = 0
for idx, row in df.iterrows():
    # Daten Zyklus
    h_vals, p_vals = [], []
    try:
        for i in range(1, 8):
            h_vals.append(row[f'h_{i}_kJkg'])
            p_vals.append(row[f'p_{i}_bar'])
        h_vals.append(h_vals[0])  # Schließen
        p_vals.append(p_vals[0])
    except:
        continue

    # Ordner
    status_dir = SUBFOLDER_VALID if row['Is_Valid'] else SUBFOLDER_INVALID
    path_dir = os.path.join(OUTPUT_FOLDER, status_dir, f"Vorlauf_{row['Flow_C']:.0f}C")
    os.makedirs(path_dir, exist_ok=True)

    # --- ANPASSUNG DATEINAME: Jetzt mit PHI und EV ---
    phi_val = row.get('phi', 0)
    y_ev_val = row.get('y_EV', 0)
    fname = f"BP_{idx:03d}_n{row['n_rpm']:.0f}_SH{row['SH']:.1f}_PHI{phi_val:.2f}_EV{y_ev_val:.2f}.png"

    fig, ax = plt.subplots(figsize=(10, 7))

    # Zyklus plotten
    col = 'forestgreen' if row['Is_Valid'] else 'crimson'
    ax.plot(h_vals, p_vals, color=col, marker='o', markersize=5, lw=2, zorder=5)

    # Limits
    xmin, xmax = min(h_vals), max(h_vals)
    ymin, ymax = min(p_vals), max(p_vals)
    ax.set_xlim(xmin - 100, xmax + 100)
    ax.set_ylim(ymin * 0.7, ymax * 1.5)

    # Hintergrund
    ax.plot(H_liq_dome, P_sat_dome, 'k-', lw=1, alpha=0.5)
    ax.plot(H_vap_dome, P_sat_dome, 'k-', lw=1, alpha=0.5)

    cur_xlim = ax.get_xlim()
    cur_ylim = ax.get_ylim()

    # Isothermen
    for t, h_iso, p_iso in isotherms:
        h_arr = np.array(h_iso)
        p_arr = np.array(p_iso)

        if np.any((p_arr > cur_ylim[0]) & (p_arr < cur_ylim[1])):
            ax.plot(h_iso, p_iso, color='gray', lw=0.5, alpha=0.3)

            visible_mask = (p_arr >= cur_ylim[0]) & (p_arr <= cur_ylim[1]) & \
                           (h_arr >= cur_xlim[0]) & (h_arr <= cur_xlim[1])
            visible_indices = np.where(visible_mask)[0]

            if len(visible_indices) > 0:
                top_idx = visible_indices[-1]
                ax.text(h_arr[top_idx], p_arr[top_idx], f"{t}°C",
                        color='gray', fontsize=7, ha='left', va='bottom', clip_on=True)

    # Labels 1-7
    lbls = ['1', '2', '3', '4', '5', '6', '7']
    offs = [(10, -10), (-10, 5), (10, 5), (10, 5), (10, -10), (-15, -10), (10, -10)]
    for i, txt in enumerate(lbls):
        ax.annotate(txt, (h_vals[i], p_vals[i]), xytext=offs[i], textcoords='offset points',
                    color=col, fontweight='bold')

    ax.set_yscale('log')
    ax.set_yticks([1, 2, 5, 10, 20, 30, 40, 50, 60])
    ax.set_yticklabels(["1", "2", "5", "10", "20", "30", "40", "50", "60"])
    ax.set_xlabel("Enthalpie [kJ/kg]")
    ax.set_ylabel("Druck [bar]")

    # --- ANPASSUNG TITEL: Jetzt mit phi und EV ---
    t_str = (f"BP {idx:03d} | T_source: {row['Source_C']:.1f}°C | T_flow: {row['Flow_C']:.1f}°C\n"
             f"n: {row['n_rpm']:.0f} rpm | SH: {row['SH']:.1f} K | phi: {phi_val:.2f} | EV: {y_ev_val:.2f}\n"
             f"{row['Status_Text']} | COP: {row['COP']:.2f}")

    ax.set_title(t_str, color='black' if row['Is_Valid'] else 'red', fontweight='bold', fontsize=10)
    ax.grid(True, which='major', alpha=0.3)

    plt.savefig(os.path.join(path_dir, fname), dpi=100)
    plt.close(fig)
    if row['Is_Valid']: count_valid += 1

print(f"Log-p-h Diagramme fertig ({count_valid} gültige).")

# =============================================================================
# 5. SCATTERPLOTS (OPTIMIERT, ECO vs POWER)
# =============================================================================

print("\nErstelle intelligente Übersichts-Scatterplots...")
scatter_dir = os.path.join(OUTPUT_FOLDER, SUBFOLDER_SCATTER)
os.makedirs(scatter_dir, exist_ok=True)

summary_eco = []
summary_power = []

for (src, flow), group in df.groupby(['Source_C', 'Flow_C']):
    # SANITY CHECK: COP, Leistung und Strom müssen positiv sein
    is_sane = (group['COP'] > 0) & (group['Q_con_kW'] > 0) & (group['P_el_W'] > 0)

    # Nur Gruppen betrachten, die Limit OK und Physikalisch OK sind
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
        # Kein gültiger Punkt
        if group['Limit_OK'].any():
            short_reason = "Physik (COP<0)"
        else:
            raw_reason = group['Status_Text'].mode()[0] if not group['Status_Text'].mode().empty else "Fail"
            short_reason = raw_reason.replace('FEHLER: ', '')

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

    # --- PLOT KONFIGURATION (Erweitert um phi) ---
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
         'cmap': 'cividis', 'mask_negatives': True},

        # NEU: Feuchte
        {'data': df_power, 'col': 'phi', 'label': 'Rel. Feuchte [-]', 'title': 'Feuchte bei maximaler Leistung',
         'cmap': 'Blues', 'mask_negatives': False}
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

                # Limits berechnen
                if mask_pos.any():
                    vmin_val = vals[mask_pos].min()
                    vmax_val = vals[mask_pos].max()

                    sc = ax.scatter(ok.loc[mask_pos, 'Source_C'], ok.loc[mask_pos, 'Flow_C'],
                                    c=vals[mask_pos], cmap=cfg['cmap'],
                                    vmin=vmin_val, vmax=vmax_val,
                                    s=150, marker='o', edgecolors='k', label='Machbar', zorder=5)
                    cbar = plt.colorbar(sc, ax=ax)
                    cbar.set_label(cfg['label'], fontsize=12)

                if mask_neg.any():
                    ax.scatter(ok.loc[mask_neg, 'Source_C'], ok.loc[mask_neg, 'Flow_C'],
                               c='darkgray', s=150, marker='o', edgecolors='k',
                               label=r'Wert $\leq$ 0 (ausgeblendet)', zorder=4)
            else:
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