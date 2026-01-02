import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import logging
import CoolProp.CoolProp as CP
from time import time

# Logging konfigurieren
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')

# --- vclibpy Imports ---
from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import ConstantEffectivenessCompressor
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs
from vclibpy.algorithms.fsolve import FSolve
from vclibpy.algorithms.iteration import Iteration
from vclibpy.utils.automation import calc_multiple_states

# =============================================================================
# 1. KONFIGURATION
# =============================================================================

EXCEL_FILE = 'betriebspunkte.xlsx'
TARGET_ID = 14  # Der Test-Punkt (-4.5 Grad)
FLUID = "Propane"

# Drehzahl 24 rpm -> 0.4 Hz
n_rpm = 24.0
n_hz_converted = n_rpm / 60.0

FIXED_PARAMS = {
    'n': n_hz_converted,
    'superheat': 5.0,  # K
    'valve_open': 0.7  # -
}

# Dateinamen für den Output
FILE_FSOLVE = f"Debug_FSolve_Punkt{TARGET_ID}.xlsx"
FILE_ITER = f"Debug_Iteration_Punkt{TARGET_ID}.xlsx"


# =============================================================================
# 2. PLOTTING FUNKTION (Liest aus Excel)
# =============================================================================

def create_log_ph_diagram(excel_path, title_suffix):
    """Erstellt das Diagramm basierend auf der generierten Excel-Datei."""
    if not os.path.exists(excel_path):
        print(f"Fehler: Datei {excel_path} nicht gefunden.")
        return

    print(f"   -> Erstelle Diagramm für {excel_path}...")
    df = pd.read_excel(excel_path)
    df.columns = df.columns.str.strip()

    # Umrechnung und Check
    if 'h_1_Jkg' not in df.columns:
        # Fallback falls Namen anders sind (manchmal generiert vclibpy "H_1 in J/kg")
        # Wir versuchen ein Mapping
        map_cols = {
            'H_1 in J/kg (Enthalpy in state 1)': 'h_1_Jkg',
            'p_1 in Pa (Pressure in state 1)': 'p_1_Pa'
            # (Mapping wird hier vereinfacht angenommen, da vclibpy meist Standard nutzt)
        }
        df = df.rename(columns=map_cols)

    # Check erneut
    found_h = False
    for c in df.columns:
        if 'h_1' in c or 'H_1' in c: found_h = True

    if not found_h:
        print("      Warnung: Keine Enthalpie-Daten gefunden (Simulation leer/gecrasht?).")
        return

    # Umrechnung J/kg -> kJ/kg und Pa -> bar
    # Wir suchen flexibel nach Spalten
    for i in range(1, 8):
        # Enthalpie
        col_h = next((c for c in df.columns if f"h_{i}" in c.lower() or f"H_{i}" in c), None)
        if col_h: df[f'h_{i}_kJkg'] = df[col_h] / 1000.0

        # Druck
        col_p = next((c for c in df.columns if f"p_{i}" in c.lower() and "pa" in c.lower()), None)
        if col_p: df[f'p_{i}_bar'] = df[col_p] / 100000.0

    # Letzte Zeile nutzen
    if df.empty:
        print("      Warnung: Excel-Datei ist leer.")
        return
    row = df.iloc[-1]

    # --- PLOT START ---
    fig, ax = plt.subplots(figsize=(10, 7))

    # CoolProp Hintergrund
    try:
        T_crit = CP.PropsSI('Tcrit', FLUID)
        T_dome = np.linspace(273.15 - 60, T_crit - 0.5, 300)
        P_sat_dome = CP.PropsSI('P', 'T', T_dome, 'Q', 0, FLUID) / 1e5
        H_liq_dome = CP.PropsSI('H', 'T', T_dome, 'Q', 0, FLUID) / 1000.0
        H_vap_dome = CP.PropsSI('H', 'T', T_dome, 'Q', 1, FLUID) / 1000.0

        ax.plot(H_liq_dome, P_sat_dome, 'k-', lw=1, alpha=0.5)
        ax.plot(H_vap_dome, P_sat_dome, 'k-', lw=1, alpha=0.5)

        # Isothermen
        iso_temps = np.arange(-40, 151, 20)
        p_range = np.logspace(np.log10(0.5), np.log10(60), 100)

        for t_c in iso_temps:
            t_k = t_c + 273.15
            h_i, p_i = [], []
            for p in p_range:
                try:
                    h = CP.PropsSI('H', 'P', p * 1e5, 'T', t_k, FLUID) / 1000.0
                    h_i.append(h)
                    p_i.append(p)
                except:
                    pass

            if h_i:
                ax.plot(h_i, p_i, color='gray', lw=0.5, alpha=0.3)
                # Beschriftung
                valid_idx = [i for i, h in enumerate(h_i) if h < 900]
                if valid_idx:
                    idx = valid_idx[-1]
                    ax.text(h_i[idx], p_i[idx], f"{t_c}°C", color='gray', fontsize=8, ha='left', va='center')

    except Exception as e:
        print(f"      Fehler CoolProp: {e}")

    # Zyklus Plotten
    h_vals, p_vals = [], []
    try:
        for i in range(1, 8):
            h_vals.append(row[f'h_{i}_kJkg'])
            p_vals.append(row[f'p_{i}_bar'])
        h_vals.append(h_vals[0])  # Schließen
        p_vals.append(p_vals[0])

        # COP suchen (Spalte kann variieren)
        col_cop = next((c for c in df.columns if "COP" in c), None)
        cop_val = row[col_cop] if col_cop else 0

        col = 'forestgreen' if cop_val > 0 else 'crimson'
        ax.plot(h_vals, p_vals, color=col, marker='o', markersize=6, lw=2, zorder=5)

        # Labels 1-7
        lbls = ['1', '2', '3', '4', '5', '6', '7']
        offs = [(10, -10), (-10, 5), (10, 5), (10, 5), (10, -10), (-15, -10), (10, -10)]
        for i, txt in enumerate(lbls):
            ax.annotate(txt, (h_vals[i], p_vals[i]), xytext=offs[i], textcoords='offset points',
                        color=col, fontweight='bold')

        ax.set_title(f"Log-p-h: {title_suffix}\n(COP: {cop_val:.2f})", fontweight='bold')

        if h_vals:
            x_span = max(h_vals) - min(h_vals)
            ax.set_xlim(min(h_vals) - x_span * 0.2, max(h_vals) + x_span * 0.2)
            ax.set_ylim(min(p_vals) * 0.7, max(p_vals) * 1.3)

    except Exception as e:
        print(f"      Fehler beim Zeichnen des Zyklus: {e}")

    ax.set_yscale('log')
    ax.set_xlabel("Enthalpie [kJ/kg]")
    ax.set_ylabel("Druck [bar]")
    ax.grid(True, which='major', alpha=0.3)

    plot_name = excel_path.replace(".xlsx", ".png")
    plt.savefig(plot_name, dpi=120)
    print(f"   -> Diagramm gespeichert: {plot_name}")


# =============================================================================
# 3. DATEN VORBEREITUNG & SETUP
# =============================================================================

if not os.path.exists(EXCEL_FILE):
    logging.error(f"Datei '{EXCEL_FILE}' nicht gefunden.")
    exit()

df_bp = pd.read_excel(EXCEL_FILE)
df_bp.columns = df_bp.columns.str.strip()

# Flexible Spaltenauswahl für Massenströme (Falls abgeschnitten in Excel)
col_m_flow_e = next((c for c in df_bp.columns if c.startswith('m_flow_e')), 'm_flow_eva')
col_m_flow_k = next((c for c in df_bp.columns if c.startswith('m_flow_k')), 'm_flow_kond')

print(f"Spalten erkannt: Luft={col_m_flow_e}, Wasser={col_m_flow_k}")

target_row = df_bp[df_bp['Nr'] == TARGET_ID]
if target_row.empty:
    logging.error(f"Punkt {TARGET_ID} nicht gefunden.")
    exit()
row = target_row.iloc[0]

print(f"\n--- GEPICKTER BETRIEBSPUNKT (Nr. {TARGET_ID}) ---")
print(f"T_eva_in: {row['T_eva_in']} °C | T_kond_in: {row['T_kond_in']} °C")

# --- Komponenten ---
condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=2.2, secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=15, thickness=0.4e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
)
evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=108.0, secondary_medium="air", flow_type="cross", ratio_outer_to_inner_area=105,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=1e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50)
)
ihx_component = ihx_ntu.IHX_NTU(
    A=0.15, alpha_low_side=1000, alpha_high_side=2000, dT_pinch_min=0.5,
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=15, thickness=1e-3)
)
expansion_valve_high = Bernoulli(A=0.000002)
expansion_valve_low = Bernoulli(A=0.000002)
compressor = ConstantEffectivenessCompressor(
    N_max=110, V_h=54.8e-6, eta_isentropic=0.7, eta_mech=0.95 ** 3, lambda_h=0.9
)
heat_pump = ihx.IHX(
    evaporator=evaporator, condenser=condenser, fluid="Propane",
    compressor=compressor, expansion_valve_high=expansion_valve_high,
    expansion_valve_low=expansion_valve_low, ihx=ihx_component
)

# Inputs
inputs = Inputs(
    evaporator=HeatExchangerInputs(T_in=row['T_eva_in'] + 273.15, m_flow=row[col_m_flow_e]),
    condenser=HeatExchangerInputs(T_in=row['T_kond_in'] + 273.15, m_flow=row[col_m_flow_k]),
    control=ControlInputs()
)
inputs.control.set(name="n", value=FIXED_PARAMS['n'], unit="Hz")
inputs.control.set(name="T_ambient", value=row['T_ambient'] + 273.15, unit="K")
inputs.control.set(name="dT_eva_superheating", value=FIXED_PARAMS['superheat'], unit="K")
inputs.control.set(name="dT_con_subcooling", value=0, unit="K")
inputs.control.set(name="opening", value=FIXED_PARAMS['valve_open'], unit="-")

# Wir packen das in eine Liste, weil calc_multiple_states das so will
input_list = [inputs]

save_directory = pathlib.Path(".")


# =============================================================================
# 4. SIMULATION START (NUR calc_multiple_states)
# =============================================================================

def run_sim_wrapper(algo_name, algo_obj, target_filename):
    print(f"\n--- Simulation: {algo_name} ---")

    # 1. Clean up: Alte potentielle Output-Dateien löschen
    default_names = ["IHX_Propane.xlsx", "results.xlsx", "Propane.xlsx"]
    for name in default_names:
        if (save_directory / name).exists():
            os.remove(save_directory / name)

    try:
        # 2. Rechnen (Wrapper nutzt intern seine Logik)
        calc_multiple_states(
            save_path=save_directory,
            flowsheet=heat_pump,
            inputs=input_list,  # Muss Liste sein!
            algorithm=algo_obj,
            use_multiprocessing=False,
            raise_errors=True,  # Damit wir sehen, wenn was knallt
            with_unit_and_description=True
        )
        print("   Berechnung abgeschlossen.")

        # 3. Datei finden und umbenennen
        renamed = False
        for name in default_names:
            src = save_directory / name
            dst = save_directory / target_filename
            if src.exists():
                if dst.exists(): os.remove(dst)
                os.rename(src, dst)
                print(f"   Ergebnis verschoben nach: {dst}")
                renamed = True
                break

        if not renamed:
            print("   WARNUNG: Keine Output-Datei gefunden! (Hat Iteration abgebrochen?)")
            # Bei Iteration ohne Konvergenz speichert er manchmal nix.
            # Dann haben wir kein Excel für den Plot.
            return

        # 4. Plotten aus der Excel
        create_log_ph_diagram(target_filename, title_suffix=f"{algo_name}")

    except Exception as e:
        print(f"!!! Fehler im Wrapper: {e}")


# 1. FSOLVE
run_sim_wrapper("FSolve", FSolve(), FILE_FSOLVE)

# 2. ITERATION
# Hinweis: Iteration speichert nur, wenn es konvergiert oder wir es zwingen.
# Da calc_multiple_states eine Blackbox ist, hoffen wir, dass er was schreibt.
# show_iteration=True ist wichtig zum Debuggen!
print("\nStarte Iteration... (bitte Grafik-Fenster beobachten)")
run_sim_wrapper("Iteration", Iteration(show_iteration=True, max_num_iterations=500), FILE_ITER)

print("\nFertig. Ergebnisse prüfen.")
plt.show()