# main.py
import logging
import pathlib
import os
from vclibpy.utils.automation import calc_multiple_states
from vclibpy.algorithms.iteration import Iteration

import model
import inputs
import optimization
import plotting
import eev_auslegung

logging.basicConfig(level=logging.INFO)

# =============================================================================
# KONFIGURATION & STEUERUNG
# =============================================================================
# MANUELLE PARAMETER (Startwerte bzw. optimierte Werte nach Iteration)
CURRENT_A_COND = 2.88    # Kondensatorfläche in m²
CURRENT_A_IHX = 0.01534  # Geschätzte Fläche für Danfoss IHX HE1.5

# Boolesche Flags zur selektiven Ausführung der Programmbausteine
RUN_FULL_SIMULATION = False
RUN_OPTIMIZER_COND = False
RUN_OPTIMIZER_IHX = False
RUN_PLOTTING = True
RUN_EEV_SIZING = False

# Dateipfade
EXCEL_BP = 'betriebspunkte_final.xlsx'
OUTPUT_FILE = 'IHX_Propane_Auslegung_IHX_1_5_con_2_88.xlsx'

# Referenz-Ergebnisse für das EEV-Sizing Tool
EXCEL_MIT_IHX = 'IHX_Propane_Auslegung_IHX_1_5_con_2_88.xlsx'
EXCEL_OHNE_IHX = 'SC_Propane_Auslegung_con_2_88.xlsx'

def run_simulation(A_cond, A_ihx, full_run=True):
    """
    Führt den kompletten vclibpy-Simulationszyklus aus.
    Erstellt das Modell, lädt die Inputs und nutzt die Iteration-Klasse
    zur Lösung der Systemgleichungen.
    """
    # 1. Systemarchitektur der Wärmepumpe aufbauen
    heat_pump = model.create_heat_pump(A_cond, A_ihx)

    # 2. Permutationsmatrix aus Input-Excel generieren
    df_bp = inputs.load_betriebspunkte(EXCEL_BP)
    input_list = inputs.generate_full_input_list(df_bp)

    # Limitierung für schnelles Testen (falls full_run = False)
    if not full_run:
        input_list = input_list[:5]

    logging.info(f"Starte Simulation mit A_cond={A_cond}, A_ihx={A_ihx}")

    # Berechnung über vclibpy Automation
    calc_multiple_states(
        save_path=pathlib.Path("."),
        flowsheet=heat_pump,
        inputs=input_list,
        algorithm=Iteration(),
        raise_errors=False, # Verhindert Abbruch bei nicht konvergierenden Punkten
        with_unit_and_description=True
    )

    # Renaming der Output-Datei zur sauberen Versionierung
    if os.path.exists("IHX_Propane.xlsx"):
        if os.path.exists(OUTPUT_FILE): os.remove(OUTPUT_FILE)
        os.rename("IHX_Propane.xlsx", OUTPUT_FILE)


# =============================================================================
# HAUPTPROGRAMM (PIPELINE)
# =============================================================================
if __name__ == "__main__":

    # SCHRITT 1: Führt die Zustandsgleichungs-Löser für alle Punkte aus
    if RUN_FULL_SIMULATION:
        run_simulation(CURRENT_A_COND, CURRENT_A_IHX)

    # SCHRITT 2: Pinch-Point Analyse und Kondensator-Geometrie anpassen
    if RUN_OPTIMIZER_COND:
        new_A = optimization.optimize_condenser(OUTPUT_FILE, CURRENT_A_COND, CURRENT_A_IHX)
        print(f"!!! BITTE 'CURRENT_A_COND' OBEN AUF {new_A} ÄNDERN !!!")

    # SCHRITT 3: IHX Optimierung (Platzhalter)
    if RUN_OPTIMIZER_IHX:
        new_A_ihx = optimization.optimize_ihx(OUTPUT_FILE, CURRENT_A_COND, CURRENT_A_IHX)
        print(f"!!! BITTE 'CURRENT_A_IHX' OBEN AUF {new_A_ihx} ÄNDERN !!!")

    # SCHRITT 4: Erstellung sämtlicher Visualisierungen für die Thesis
    if RUN_PLOTTING:
        print("Erstelle Plots...")
        plot_folder_name = f"Plots_Cond{CURRENT_A_COND:.2f}_IHX{CURRENT_A_IHX:.2f}"
        plotting.create_all_plots(OUTPUT_FILE, plot_folder_name)

    # SCHRITT 5: Parameterextraktion für Danfoss Coolselector (Ventilauslegung)
    if RUN_EEV_SIZING:
        print("\n=============================================")
        print("   SCHRITT 5: EEV AUSLEGUNG (Kv-Proxy)")
        print("=============================================")
        eev_auslegung.finde_eev_randpunkte(EXCEL_MIT_IHX, EXCEL_OHNE_IHX)