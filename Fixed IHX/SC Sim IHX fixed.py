import numpy as np
import pandas as pd
import pathlib
import os
import logging

logging.basicConfig(level=logging.INFO)

from vclibpy.flowsheets import StandardCycle
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor, ConstantEffectivenessCompressor

from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs

from vclibpy.algorithms.base import Algorithm
from vclibpy.algorithms.fsolve import FSolve
from vclibpy.utils.automation import calc_multiple_states

# =============================================================================
# 1. Vorbereitung: Daten laden
# =============================================================================

# WICHTIG: Stelle sicher, dass dies die NEUE Excel-Datei aus dem Generator ist,
# die die Spalte "phi" enthält!
excel_filename = 'betriebspunkte_final_bivalent_seriell.xlsx'
if not os.path.exists(excel_filename):
    raise FileNotFoundError(f"Datei {excel_filename} nicht gefunden!")

df_betriebspunkte = pd.read_excel(excel_filename)

# Prüfen, ob die Spalte 'phi' existiert (Sicherheitscheck)
if 'phi' not in df_betriebspunkte.columns:
    raise ValueError("Die Excel-Datei hat keine Spalte 'phi'. Bitte erst den neuen Generator ausführen!")

# =============================================================================
# 2. Initialisierung des Kältekreises
# =============================================================================

# Kondensator
condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=5, secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
)

# Verdampfer
evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=43.50, secondary_medium="air", flow_type="cross", ratio_outer_to_inner_area=105,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50)
)

expansion_valve = Bernoulli(A=0.000002)

compressor = ConstantEffectivenessCompressor(
    N_max=110,
    V_h=54.8e-6,
    eta_isentropic=0.7,
    eta_mech=0.95 * 0.95 * 0.95,
    lambda_h=0.9
)

heat_pump = StandardCycle(
    evaporator=evaporator, condenser=condenser, fluid='propane',
    compressor=compressor, expansion_valve=expansion_valve,
)

algorithm = FSolve()

# =============================================================================
# 3. Iteration & Parameter-Definition
# =============================================================================

inputs_list = []

# --- Listen für die Variation ---
speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
superheats = [5, 10, 15]

# Luftvolumenströme in m³/s
air_vol_flows = [1.4]


logging.info(f"Generiere Input-Liste. Variation über:")
logging.info(f" -> {len(df_betriebspunkte)} Zeilen (Betriebspunkte inkl. Feuchte)")
logging.info(f" -> {len(speeds_rel)} Drehzahlen")
logging.info(f" -> {len(superheats)} Überhitzungen")
logging.info(f" -> {len(air_vol_flows)} Volumenströme")

for index, row in df_betriebspunkte.iterrows():
    # Temperatur in Kelvin für Dichteberechnung
    T_amb_K = row['T_ambient'] + 273.15

    # Schleife über Volumenströme
    for v_flow in air_vol_flows:

        # Umrechnung m³/s in kg/s (Massenstrom = Dichte * Volumenstrom)
        rho_air = 101325 / (287 * T_amb_K)
        m_flow_air_calc = v_flow * rho_air

        for sh in superheats:
            for n_rel in speeds_rel:
                # Verdampfer Inputs
                evap_inputs = HeatExchangerInputs(
                    T_in=T_amb_K,
                    m_flow=m_flow_air_calc,  # Nutze berechneten Massenstrom
                )

                cond_inputs = HeatExchangerInputs(
                    T_in=row['T_kond_in'] + 273.15,
                    m_flow=row['m_flow_kond'],
                )

                control_inputs = ControlInputs()

                control_inputs.set(
                    name="n",
                    value=n_rel,
                    unit="Hz",
                    description="Compressor speed"
                )
                control_inputs.set(
                    name="dT_eva_superheating",
                    value=sh,
                    unit="K",
                    description="Evaporator superheating"
                )
                control_inputs.set(
                    name="dT_con_subcooling",
                    value=0,  # Ggf. auf 2 oder 3 setzen für Stabilität
                    unit="K",
                    description="Condenser subcooling"
                )
                control_inputs.set(
                    name="T_ambient",
                    value=T_amb_K,
                    unit="K",
                    description="Ambient temperature"
                )

                # Volumenstrom speichern (nur für Doku)
                control_inputs.set(
                    name="V_flow_air",
                    value=v_flow,
                    unit="m3/s",
                    description="Air Volume Flow (Input Reference)"
                )

                # --- ERSTELLEN DES HAUPT-INPUTS-OBJEKTS ---
                current_input = Inputs(
                    control=control_inputs,
                    evaporator=evap_inputs,
                    condenser=cond_inputs,
                )

                inputs_list.append(current_input)

logging.info(f"Fertig. Insgesamt {len(inputs_list)} Simulationen vorbereitet")

# =============================================================================
# 4. Ausführung
# =============================================================================

save_directory = pathlib.Path(".")
output_filename = "SC_Propane_Auslegung.xlsx"

logging.info("Starte Simulation (calc_multiple_states)...")

try:
    calc_multiple_states(
        save_path=save_directory,
        flowsheet=heat_pump,
        inputs=inputs_list,
        algorithm=algorithm,
        use_multiprocessing=False,  # Stabil laufen lassen
        raise_errors=False,
        with_unit_and_description=True
    )

    res_file = save_directory / "Standard_propane.xlsx"
    target_file = save_directory / output_filename

    # Workaround für Dateinamen (manchmal heißt es results.xlsx, manchmal Fluidname)
    if not res_file.exists():
        res_file = save_directory / "results.xlsx"

    if res_file.exists():
        if target_file.exists():
            os.remove(target_file)
        os.rename(res_file, target_file)
        logging.info(f"\nSimulation beendet. Datei gespeichert als: {target_file}")
    else:
        logging.warning("Simulation fertig, aber Output-Datei nicht automatisch gefunden.")

except Exception as e:
    logging.error(f"Ein schwerwiegender Fehler ist aufgetreten: {e}")