import numpy as np
import pandas as pd
import pathlib
import os
import logging

logging.basicConfig(level=logging.INFO)

# --- vclibpy Imports ---
from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor, ConstantEffectivenessCompressor
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs
from vclibpy.algorithms.fsolve import FSolve
from vclibpy.algorithms.iteration import Iteration
from vclibpy.utils.automation import calc_multiple_states

# =============================================================================
# 1. VORBEREITUNG: DATEN LADEN
# =============================================================================

# WICHTIG: Nutze die Datei aus dem neuen Generator (mit Spalte 'phi')
excel_filename = 'betriebspunkte_final_bivalent_seriell.xlsx'
if not os.path.exists(excel_filename):
    raise FileNotFoundError(f"Datei '{excel_filename}' nicht gefunden.")

df_betriebspunkte = pd.read_excel(excel_filename)

# =============================================================================
# 2. INITIALISIERUNG DES KÄLTEKREISES (IHX)
# =============================================================================

condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=2.2,
    secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3500),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=500),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000), # Kupfer
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4000)
)

# Verdampfer (108 m² Außenfläche, Ratio 105 -> ~1m² Innen)
evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=43.50,
    secondary_medium="air", flow_type="cross",
    ratio_outer_to_inner_area=105,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=400),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3), # Kupfer
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=64)
)

ihx_component = ihx_ntu.IHX_NTU(
    A=0.15,
    alpha_low_side=500,
    alpha_high_side=2000,
    dT_pinch_min=0.5,
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3) # Kupfer
)

expansion_valve_high = Bernoulli(A=0.2)
expansion_valve_low = Bernoulli(A=0.000002)

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

algorithm = Iteration()

# =============================================================================
# 3. ERSTELLEN DER EINGABE-LISTE
# =============================================================================

inputs_list = []

# Iterations-Listen
speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
superheats = [5, 10, 15]
air_vol_flows = [1.4]#[1.3,1.4,1.5]  # m³/s

# IHX Spezifisch: Variation der Ventilöffnung (Hochdruck-Seite)
hpev_openings = [1.0] #[0.5, 0.7, 1.0]

logging.info(f"Generiere Input-Liste. Variation über:")
logging.info(f" -> {len(df_betriebspunkte)} Zeilen (BP + Feuchte)")
logging.info(f" -> {len(air_vol_flows)} Volumenströme")
logging.info(f" -> {len(hpev_openings)} Ventilöffnungen (HPEV)")

for index, row in df_betriebspunkte.iterrows():
    T_amb_K = row['T_ambient'] + 273.15

    for v_flow in air_vol_flows:
        # Dichte berechnen für Massenstrom
        rho_air = 101325 / (287 * T_amb_K)
        m_flow_air_calc = v_flow * rho_air
        for valve_pos in hpev_openings:
            for sh in superheats:
                for n_rel in speeds_rel:
                    # HX Inputs
                    evap_inputs = HeatExchangerInputs(
                        T_in=T_amb_K,
                        m_flow=m_flow_air_calc,  # Berechneter Massenstrom
                    )
                    cond_inputs = HeatExchangerInputs(
                        T_in=row['T_kond_in'] + 273.15,
                        m_flow=row['m_flow_kond'],
                    )

                    control_inputs = ControlInputs()

                    # Standard Controls
                    control_inputs.set(name="n", value=n_rel, unit="Hz")
                    control_inputs.set(name="T_ambient", value=T_amb_K, unit="K")

                    #Volumenstrom (Referenz)
                    control_inputs.set(name="V_flow_air", value=v_flow, unit="m3/s")

                    # LPEV Regelung (Überhitzung)
                    control_inputs.set(name="dT_eva_superheating", value=sh, unit="K")
                    control_inputs.set(name="dT_con_subcooling", value=0, unit="K")

                    # HPEV Regelung
                    control_inputs.set(
                        name="opening",
                        value=valve_pos,
                        unit="-",
                        description="Opening High-Side EV",
                    )

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
output_filename = "IHX_Propane_Auslegung.xlsx"

logging.info("Starte Simulation...")

try:
    calc_multiple_states(
        save_path=save_directory,
        flowsheet=heat_pump,
        inputs=inputs_list,
        algorithm=algorithm,
        use_multiprocessing=False,  # Stabil lassen oder True um schneller zu rechnen
        raise_errors=False,
        with_unit_and_description=True
    )

    # Datei umbenennen
    res_file = save_directory / "IHX_Propane.xlsx"  # Standardname bei diesem Flowsheet checken

    target_file = save_directory / output_filename

    if res_file.exists():
        if target_file.exists():
            os.remove(target_file)
        os.rename(res_file, target_file)
        logging.info(f"Simulation beendet. Datei gespeichert als: {target_file}")
    else:
        logging.warning("Simulation durchgelaufen, aber Output-Datei nicht gefunden.")

except Exception as e:
    logging.error(f"Fehler: {e}")