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

# 1. Vorbereitung: Daten laden

excel_filename = 'betriebspunkte.xlsx'
df_betriebspunkte = pd.read_excel(excel_filename)


# 2. Initialisierung des Kältekreises
condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=5, secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
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

expansion_valve = Bernoulli(A=0.00001)

compressor = ConstantEffectivenessCompressor(
    N_max=110,
    V_h=54.8e-6,
    eta_isentropic=0.7,
    eta_mech=0.95*0.95*0.95,
    lambda_h=0.9
)

heat_pump = StandardCycle(
    evaporator=evaporator, condenser=condenser, fluid='propane',
    compressor=compressor, expansion_valve=expansion_valve,
)

algorithm = FSolve()


# 3. Iteration

inputs_list = []
speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
superheats = [5, 10, 15]

logging.info("Generiere Input-Liste durch verschachtelte Schleifen...")
for index, row in df_betriebspunkte.iterrows():
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
                value=0,
                unit="K",
                description="Condenser subcooling"
            )
            control_inputs.set(
                name="T_ambient",
                value=row['T_ambient'] + 273.15,
                unit="K",
                description="Ambient temperature"
            )

            # --- 2. ERSTELLEN DES HAUPT-INPUTS-OBJEKTS ---
            current_input = Inputs(
                control=control_inputs,
                evaporator=evap_inputs,
                condenser=cond_inputs,
            )

            inputs_list.append(current_input)
logging.info(f"Fertig. Insgesamt {len(inputs_list)} Simulationen vorbereitet")


# 4. Ausführung

save_directory = pathlib.Path(".")

logging.info("Starte Simulation (calc_multiple_states)...")

try:
    calc_multiple_states(
        save_path=save_directory,
        flowsheet=heat_pump,
        inputs=inputs_list,
        algorithm=algorithm,
        use_multiprocessing=False,  # Nutze alle Kerne für Geschwindigkeit
        raise_errors=False,
        with_unit_and_description=True
    )
    logging.info(f"\nSimulation beendet. Ergebnisse in '{save_directory}' als Excel-Datei gespeichert.")

except Exception as e:
    logging.error(f"Ein schwerwiegender Fehler ist aufgetreten: {e}")
