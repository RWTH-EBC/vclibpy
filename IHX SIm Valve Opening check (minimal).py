import numpy as np
import pandas as pd
import pathlib
import os
import logging

# Konfiguriere das Logging
logging.basicConfig(level=logging.INFO)

# --- vclibpy Imports ---
from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor, ConstantEffectivenessCompressor
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs
from vclibpy.algorithms.fsolve import FSolve
from vclibpy.utils.automation import calc_multiple_states

# =============================================================================
# 1. VORBEREITUNG: DATEN LADEN
# =============================================================================

# Anpassung: Dateiname direkt 'betriebspunkte.xlsx'
excel_filename = 'betriebspunkte.xlsx'

if not os.path.exists(excel_filename):
    logging.error(f"Datei '{excel_filename}' nicht gefunden. Bitte erst Betriebspunkte generieren.")
    exit()

df_betriebspunkte = pd.read_excel(excel_filename)

# Anpassung: Nur die ersten 17 Betriebspunkte für den Test
logging.info("Reduziere Betriebspunkte auf die ersten 17 Stück.")
df_betriebspunkte = df_betriebspunkte.iloc[:17]

# =============================================================================
# 2. INITIALISIERUNG DES KÄLTEKREISES (IHX)
# =============================================================================

condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=2.2,
    secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
)

evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=108, secondary_medium="air", flow_type="cross", ratio_outer_to_inner_area=105,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50)
)

ihx_component = ihx_ntu.IHX_NTU(
    A=0.15,
    alpha_low_side=1000,
    alpha_high_side=2000,
    dT_pinch_min=0.5,
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3)
)

# Ventile (Größen aus deinem letzten Snippet übernommen)
expansion_valve_high = Bernoulli(A=0.000008)
expansion_valve_low = Bernoulli(A=0.000001)

compressor = ConstantEffectivenessCompressor(
    N_max=110,
    V_h=54.8e-6,
    eta_isentropic=0.7,
    eta_mech=0.95 * 0.95 * 0.95,
    lambda_h=0.9
)

heat_pump = ihx.IHX(
    evaporator=evaporator,
    condenser=condenser,
    fluid="Propane",
    compressor=compressor,
    expansion_valve_high=expansion_valve_high,
    expansion_valve_low=expansion_valve_low,
    ihx=ihx_component
)

algorithm = FSolve()

# =============================================================================
# 3. ERSTELLEN DER EINGABE-LISTE
# =============================================================================

inputs_list = []
speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
superheats = [5, 10, 15]

# Anpassung: Liste der Ventilöffnungen
valve_openings = [0.3, 0.6, 0.9]

logging.info(f"Generiere Input-Liste für {len(df_betriebspunkte)} Betriebspunkte...")
logging.info(f"Variiere Ventilöffnung über: {valve_openings}")

for index, row in df_betriebspunkte.iterrows():
    # Neue Schleife: Variation der Ventilöffnung
    for valve_pos in valve_openings:
        for sh in superheats:
            for n_rel in speeds_rel:
                evap_inputs = HeatExchangerInputs(
                    T_in=row['T_eva_in'] + 273.15,
                    m_flow=row['m_flow_eva'],
                )
                cond_inputs = HeatExchangerInputs(
                    T_in=row['T_kond_in'] + 273.15,
                    m_flow=row['m_flow_kond'],
                )

                control_inputs = ControlInputs()
                control_inputs.set(name="n", value=n_rel, unit="Hz", description="Compressor speed")
                control_inputs.set(name="dT_eva_superheating", value=sh, unit="K",
                                   description="Evaporator outlet superheat")
                control_inputs.set(name="dT_con_subcooling", value=0, unit="K", description="Condenser subcooling")
                control_inputs.set(name="T_ambient", value=row['T_ambient'] + 273.15, unit="K",
                                   description="Ambient temperature")

                # Setze die variable Ventilöffnung
                control_inputs.set(name="opening", value=valve_pos, unit="-", description="Opening High-Side EV")

                current_input = Inputs(
                    control=control_inputs,
                    evaporator=evap_inputs,
                    condenser=cond_inputs,
                )
                inputs_list.append(current_input)

logging.info(f"Fertig. {len(inputs_list)} Simulationen vorbereitet.")

# =============================================================================
# 4. AUSFÜHREN UND SPEICHERN
# =============================================================================

save_directory = pathlib.Path(".")
# Anpassung: Ziel-Dateiname
target_filename = "IHX_Propane_Valve_Check.xlsx"
# Anpassung: Der Name, den vclibpy standardmäßig erzeugt
default_filename = "IHX_Propane.xlsx"

logging.info("Starte Simulation...")

try:
    calc_multiple_states(
        save_path=save_directory,
        flowsheet=heat_pump,
        inputs=inputs_list,
        algorithm=algorithm,
        use_multiprocessing=False,
        raise_errors=False,
        with_unit_and_description=True
    )

    # Datei umbenennen
    source_file = save_directory / default_filename
    target_file = save_directory / target_filename

    if source_file.exists():
        if target_file.exists():
            os.remove(target_file)
            logging.info(f"Alte '{target_filename}' überschrieben.")

        os.rename(source_file, target_file)
        logging.info(f"Erfolg! Datei gespeichert als: {target_file}")
    else:
        # Fallback, falls vclibpy doch "results.xlsx" nutzt
        fallback_source = save_directory / "results.xlsx"
        if fallback_source.exists():
            if target_file.exists():
                os.remove(target_file)
            os.rename(fallback_source, target_file)
            logging.info(f"Erfolg! (Aus results.xlsx) Datei gespeichert als: {target_file}")
        else:
            logging.warning(f"Simulation fertig, aber '{default_filename}' nicht gefunden. Bitte Ordner prüfen.")

except Exception as e:
    logging.error(f"Fehler: {e}")