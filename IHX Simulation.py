import numpy as np
import pandas as pd
import pathlib
import os
import logging

# Konfiguriere das Logging
logging.basicConfig(level=logging.INFO)

# --- vclibpy Imports ---
# Flowsheet und Komponenten
from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor, ConstantEffectivenessCompressor

# Datenmodelle
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs

# Algorithmus
from vclibpy.algorithms.fsolve import FSolve
from vclibpy.utils.automation import calc_multiple_states

# =============================================================================
# 1. VORBEREITUNG: DATEN LADEN
# =============================================================================

excel_filename = 'betriebspunkte.xlsx'
if not os.path.exists(excel_filename):
    logging.error(f"Datei '{excel_filename}' nicht gefunden. Bitte erst Betriebspunkte generieren.")
    exit()

df_betriebspunkte = pd.read_excel(excel_filename)

# =============================================================================
# 2. INITIALISIERUNG DES KÄLTEKREISES (IHX)
# =============================================================================

# A. Kondensator & Verdampfer (Bleiben gleich)
condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=8,  # Hatten wir im vorherigen Schritt erhöht
    secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
)

evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=24, secondary_medium="air", flow_type="cross", ratio_outer_to_inner_area=24,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50)
)

# B. NEU: Interner Wärmeübertrager (IHX)
ihx_component = ihx_ntu.IHX_NTU(
    A=2.0,  # Fläche des IHX
    alpha_low_side=1000,  # Gas-Seite (Niederdruck)
    alpha_high_side=2000,  # Flüssigkeits-Seite (Hochdruck)
    dT_pinch_min=0.5,  # Minimaler Pinch
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3)
)

# C. NEU: Zwei Expansionsventile
# Im IHX Cycle gibt es ein High-Side und ein Low-Side Ventil
expansion_valve_high = Bernoulli(A=0.000008)
expansion_valve_low = Bernoulli(A=0.000001)

compressor = ConstantEffectivenessCompressor(
    N_max=110,
    V_h=54.8e-6,
    eta_isentropic=0.7,
    eta_mech=0.95*0.95*0.95,
    lambda_h=0.9
)

# D. Flowsheet Zusammenbau (IHX statt StandardCycle)
heat_pump = ihx.IHX(
    evaporator=evaporator,
    condenser=condenser,
    fluid="Propane",
    compressor=compressor,
    expansion_valve_high=expansion_valve_high,  # <--- NEU
    expansion_valve_low=expansion_valve_low,  # <--- NEU
    ihx=ihx_component  # <--- NEU
)

algorithm = FSolve()

# =============================================================================
# 3. ERSTELLEN DER EINGABE-LISTE
# =============================================================================

inputs_list = []
speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
superheats = [5, 10, 15]

# NEU: Öffnungsgrad des ersten Ventils (High-Pressure EV)
# Das IHX Flowsheet nutzt diesen Wert, um den Zwischendruck zu bestimmen.
# 1.0 bedeutet voll offen. Wir nehmen hier 0.5 als Startwert an oder iterieren.
# Für einfache Kompatibilität setzen wir ihn fest.
valve_opening = 0.5

logging.info("Generiere Input-Liste...")

for index, row in df_betriebspunkte.iterrows():
    for sh in superheats:
        for n_rel in speeds_rel:
            # HX Inputs (Wie gehabt)
            evap_inputs = HeatExchangerInputs(
                T_in=row['T_eva_in'] + 273.15,
                m_flow=row['m_flow_eva'],
            )
            cond_inputs = HeatExchangerInputs(
                T_in=row['T_kond_in'] + 273.15,
                m_flow=row['m_flow_kond'],
            )

            # Control Inputs
            control_inputs = ControlInputs()
            control_inputs.set(name="n", value=n_rel, unit="Hz", description="Compressor speed")
            control_inputs.set(name="dT_eva_superheating", value=sh, unit="K", description="Evaporator outlet superheat")
            control_inputs.set(name="dT_con_subcooling", value=0, unit="K", description="Condenser subcooling")
            control_inputs.set(name="T_ambient", value=row['T_ambient'] + 273.15, unit="K",
                               description="Ambient temperature")

            # NEU: Parameter für das erste Expansionsventil
            control_inputs.set(name="opening", value=valve_opening, unit="-", description="Opening High-Side EV")

            current_input = Inputs(
                control=control_inputs,
                evaporator=evap_inputs,
                condenser=cond_inputs,
            )
            inputs_list.append(current_input)

logging.info(f"Fertig. {len(inputs_list)} Simulationen vorbereitet.")

# =============================================================================
# 4. AUSFÜHREN
# =============================================================================

save_directory = pathlib.Path(".")
desired_filename = "IHX_Propane_1x0.xlsx"  # Ihr Wunschname

logging.info("Starte Simulation...")
#gh
try:
    # 1. Simulation ausführen (ohne file_name Argument!)
    calc_multiple_states(
        save_path=save_directory,
        flowsheet=heat_pump,
        inputs=inputs_list,
        algorithm=algorithm,
        use_multiprocessing=False,
        raise_errors=False,
        with_unit_and_description=True
        # file_name=... WURDE ENTFERNT
    )

    # 2. Datei umbenennen (Workaround)
    # vclibpy erstellt standardmäßig "results.xlsx" im Zielordner
    default_file = save_directory /"IHX_Propane.xlsx"
    target_file = save_directory / desired_filename

    if default_file.exists():
        # Falls die Zieldatei schon existiert, vorher löschen (sonst Fehler bei rename)
        if target_file.exists():
            os.remove(target_file)

        os.rename(default_file, target_file)
        logging.info(f"Simulation beendet. Datei gespeichert als: {target_file}")
    else:
        logging.warning("Simulation lief durch, aber 'results.xlsx' wurde nicht gefunden. Prüfen Sie den Ordner.")

except Exception as e:
    logging.error(f"Fehler: {e}")



