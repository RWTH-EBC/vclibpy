import pandas as pd
import numpy as np
import os
import pathlib
import logging

# vclibpy Imports (Exakt wie in deiner main)
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs
from vclibpy.algorithms.iteration import Iteration
from vclibpy.utils.automation import calc_multiple_states

import model

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def optimize_condenser(results_excel, current_A_cond, current_A_ihx):
    print("\n=============================================")
    print("   KONDENSATOR OPTIMIERUNG (ORIGINAL INPUTS)")
    print("=============================================")

    TARGET_PINCH = 2.0
    TOLERANCE = 0.1
    MAX_ITER = 20
    STEP_SIZE = 0.5  # m²

    if not os.path.exists(results_excel):
        logger.error(f"Ergebnisdatei {results_excel} nicht gefunden.")
        return current_A_cond

    # ---------------------------------------------------------
    # 1. ERGEBNISSE LADEN & FILTERN
    # ---------------------------------------------------------
    df_res = pd.read_excel(results_excel)

    # Filtere ungültige COPs (Negativ oder NaN)
    col_cop = [c for c in df_res.columns if 'cop' in c.lower()][0]
    df_res = df_res.dropna(subset=[col_cop])
    df_res = df_res[df_res[col_cop] > 0]

    # Finde Pinch Spalte
    col_pinch = [c for c in df_res.columns if 'dt_min_con' in c.lower()][0]
    df_res = df_res.dropna(subset=[col_pinch])

    # === NEUER FILTER: Nur Außentemperaturen <= 7 °C (280.15 K) ===
    col_tamb = [c for c in df_res.columns if 'ambient' in c.lower() or 'evaporator secondary side inlet' in c.lower()][
        0]
    df_res = df_res[df_res[col_tamb] <= 280.15]

    if df_res.empty:
        print("WARNUNG: Keine Punkte unter 7°C gefunden. Checke den Filter!")
        return current_A_cond
    # ==============================================================

    # Finde den schlechtesten Betriebspunkt (Maximaler Pinch)
    worst_idx = df_res[col_pinch].idxmax()
    start_pinch = df_res.loc[worst_idx, col_pinch]

    direction = 1.0 if start_pinch > TARGET_PINCH else -1.0

    print(f"-> Kritischer Punkt bei Original-Index: {worst_idx}")
    print(f"-> Start-Pinch: {start_pinch:.3f} K (Ziel: {TARGET_PINCH} K)")
    print(f"-> Strategie: Kondensatorfläche {'VERGRÖSSERN' if direction > 0 else 'VERKLEINERN'}")

    # ---------------------------------------------------------
    # 2. INPUT-OBJEKT ÜBER DEINE ORIGINALLOGIK REKONSTRUIEREN
    # ---------------------------------------------------------
    # Wir laden die Basis-Tabelle neu
    excel_filename = 'betriebspunkte_final_bivalent_seriell.xlsx'
    if not os.path.exists(excel_filename):
        excel_filename = 'betriebspunkte_final.xlsx'  # Fallback

    df_betriebspunkte = pd.read_excel(excel_filename)

    inputs_list = []
    speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
    superheats = [5, 10, 15]
    air_vol_flows = [1.4]
    hpev_openings = [1.0]

    # DEIN EXAKTER CODE ZUR GENERIERUNG DER MATRIX
    for index, row in df_betriebspunkte.iterrows():
        T_amb_K = row['T_ambient'] + 273.15
        for v_flow in air_vol_flows:
            rho_air = 101325 / (287 * T_amb_K)
            m_flow_air_calc = v_flow * rho_air
            for valve_pos in hpev_openings:
                for sh in superheats:
                    for n_rel in speeds_rel:
                        evap_inputs = HeatExchangerInputs(
                            T_in=T_amb_K,
                            m_flow=m_flow_air_calc
                        )
                        cond_inputs = HeatExchangerInputs(
                            T_in=row['T_kond_in'] + 273.15,
                            m_flow=row['m_flow_kond']
                        )

                        control_inputs = ControlInputs()
                        control_inputs.set(name="n", value=n_rel, unit="Hz")
                        control_inputs.set(name="T_ambient", value=T_amb_K, unit="K")
                        control_inputs.set(name="V_flow_air", value=v_flow, unit="m3/s")
                        control_inputs.set(name="dT_eva_superheating", value=sh, unit="K")
                        control_inputs.set(name="dT_con_subcooling", value=2, unit="K")
                        control_inputs.set(name="opening", value=valve_pos, unit="-")

                        current_input = Inputs(
                            control=control_inputs,
                            evaporator=evap_inputs,
                            condenser=cond_inputs
                        )
                        inputs_list.append(current_input)

    # Hier ist der Magie-Moment: Da worst_idx der Index aus pandas ist,
    # entspricht er exakt der Position in dieser generierten Liste!
    target_input = inputs_list[worst_idx]

    print("-> Input-Parameter erfolgreich aus Original-Logik geladen!")

    # ---------------------------------------------------------
    # 3. OPTIMIERUNGS-SCHLEIFE
    # ---------------------------------------------------------
    current_area = current_A_cond
    temp_dir = pathlib.Path("Temp_Opt")
    temp_dir.mkdir(exist_ok=True)

    algo = Iteration(max_iter=300, tol=1e-4, damping=0.6)

    for i in range(MAX_ITER):
        print(f"\n--- Durchgang {i + 1}/{MAX_ITER} ---")
        print(f"   Simuliere A_cond = {current_area:.4f} m²")

        hp_model = model.create_heat_pump(current_area, current_A_ihx)

        # Alte Datei löschen, damit wir sicher das NEUE Ergebnis lesen
        excel_out = temp_dir / "IHX_Propane.xlsx"
        if excel_out.exists():
            excel_out.unlink()

        # Wir ignorieren den Rückgabewert, da die Daten in die Datei geschrieben werden
        calc_multiple_states(
            save_path=temp_dir,
            flowsheet=hp_model,
            inputs=[target_input],
            algorithm=algo,
            raise_errors=False,
            use_multiprocessing=False
        )

        # ---------------------------------------------------------
        # ERGEBNIS AUS DER EXCEL-DATEI AUSLESEN
        # ---------------------------------------------------------
        if not excel_out.exists():
            print("   -> Simulation abgebrochen (Keine Datei erstellt). Ändere Fläche weiter in Sicherheitsrichtung.")
            current_area += (STEP_SIZE * direction)
            continue

        try:
            df_temp = pd.read_excel(excel_out)

            # Check ob die Simulation wirklich ein gültiges Ergebnis hat (z.B. COP existiert)
            col_cop = [c for c in df_temp.columns if 'cop' in c.lower()]
            if not col_cop or pd.isna(df_temp.loc[0, col_cop[0]]):
                print("   -> Simulation nicht konvergiert (Werte leer). Ändere Fläche weiter in Sicherheitsrichtung.")
                current_area += (STEP_SIZE * direction)
                continue

            # Pinch auslesen
            col_pinch = [c for c in df_temp.columns if 'dt_min_con' in c.lower()][0]
            new_pinch = df_temp.loc[0, col_pinch]

            if pd.isna(new_pinch):
                print("   -> Simulation nicht konvergiert (Pinch ist NaN). Ändere Fläche weiter.")
                current_area += (STEP_SIZE * direction)
                continue

            print(f"   -> Resultierender Pinch = {new_pinch:.3f} K")

            # --- ZIEL ERREICHT? ---
            if abs(new_pinch - TARGET_PINCH) <= TOLERANCE:
                print(f"\n✅ ZIEL ERREICHT! Optimale Fläche: {current_area:.4f} m²")
                return current_area

            # --- FLÄCHE ANPASSEN ---
            if (direction == 1.0 and new_pinch < TARGET_PINCH) or \
                    (direction == -1.0 and new_pinch > TARGET_PINCH):
                print("   -> Zielwert gekreuzt. Verkleinere Anpassungsschritt.")
                STEP_SIZE *= 0.5
                direction *= -1.0

            current_area += (STEP_SIZE * direction)

        except Exception as e:
            print(f"   -> Fehler beim Auslesen der Ergebnisse ({e}). Ändere Fläche weiter.")
            current_area += (STEP_SIZE * direction)
            continue

    print("\nMaximale Durchgänge erreicht.")
    return current_area


def optimize_ihx(results_excel, A_cond, current_A_ihx):
    return current_A_ihx