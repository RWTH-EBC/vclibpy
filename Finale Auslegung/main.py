# main.py
import logging
import pathlib
import os
from vclibpy.utils.automation import calc_multiple_states
from vclibpy.algorithms.iteration import Iteration

# Importiere unsere Module
import model
import inputs
import optimization
import plotting

# import plotting (dein Plotting-Skript als Modul)

logging.basicConfig(level=logging.INFO)

# =============================================================================
# KONFIGURATION & STEUERUNG
# =============================================================================

# MANUELLE PARAMETER (Hier trägst du die Ergebnisse der Optimierung ein)
CURRENT_A_COND = 2.2  # Startwert
CURRENT_A_IHX = 0.0275  # Startwert

# Flags steuern, was passiert
RUN_FULL_SIMULATION = False
RUN_OPTIMIZER_COND = True  # Erst True setzen, wenn Full Sim fertig
RUN_OPTIMIZER_IHX = False
RUN_PLOTTING = False

EXCEL_BP = 'betriebspunkte_final.xlsx'
OUTPUT_FILE = 'IHX_Propane_Auslegung_angepasste_IHX_Fläche_angepasst.xlsx'


def run_simulation(A_cond, A_ihx, full_run=True):
    """Führt die Simulation aus."""

    # 1. Modell bauen
    heat_pump = model.create_heat_pump(A_cond, A_ihx)

    # 2. Inputs generieren
    df_bp = inputs.load_betriebspunkte(EXCEL_BP)
    input_list = inputs.generate_full_input_list(df_bp)

    if not full_run:
        # Nur zum Testen vielleicht nur 5 Punkte
        input_list = input_list[:5]

    logging.info(f"Starte Simulation mit A_cond={A_cond}, A_ihx={A_ihx}")

    calc_multiple_states(
        save_path=pathlib.Path("."),
        flowsheet=heat_pump,
        inputs=input_list,
        algorithm=Iteration(),
        raise_errors=False,
        with_unit_and_description=True
    )

    # Datei umbenennen (VCLibPy Output Handling)
    # ... (Dein Rename Code hier) ...
    if os.path.exists("IHX_Propane.xlsx"):  # Standardname
        if os.path.exists(OUTPUT_FILE): os.remove(OUTPUT_FILE)
        os.rename("IHX_Propane.xlsx", OUTPUT_FILE)


# =============================================================================
# HAUPTPROGRAMM
# =============================================================================

if __name__ == "__main__":

    # SCHRITT 1: Simulation
    if RUN_FULL_SIMULATION:
        run_simulation(CURRENT_A_COND, CURRENT_A_IHX)

    # SCHRITT 2: Optimierung Kondensator
    if RUN_OPTIMIZER_COND:
        # Analysiert das Ergebnis von Schritt 1 und schlägt neues A vor
        new_A = optimization.optimize_condenser(OUTPUT_FILE, CURRENT_A_COND, CURRENT_A_IHX)
        print(f"!!! BITTE 'CURRENT_A_COND' OBEN AUF {new_A} ÄNDERN !!!")

    # SCHRITT 3: Optimierung IHX
    if RUN_OPTIMIZER_IHX:
        new_A_ihx = optimization.optimize_ihx(OUTPUT_FILE, CURRENT_A_COND, CURRENT_A_IHX)
        print(f"!!! BITTE 'CURRENT_A_IHX' OBEN AUF {new_A_ihx} ÄNDERN !!!")

    # SCHRITT 4: Plots
    if RUN_PLOTTING:
        print("Erstelle Plots...")
        plot_folder_name = f"Plots_Cond{CURRENT_A_COND:.2f}_IHX{CURRENT_A_IHX:.2f}"
        plotting.create_all_plots(OUTPUT_FILE, plot_folder_name)