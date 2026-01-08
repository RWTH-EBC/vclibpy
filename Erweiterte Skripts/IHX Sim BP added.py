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
excel_filename = 'betriebspunkte.xlsx'
if not os.path.exists(excel_filename):
    raise FileNotFoundError(f"Datei '{excel_filename}' nicht gefunden.")

df_betriebspunkte = pd.read_excel(excel_filename)

#if 'phi' not in df_betriebspunkte.columns:
    #raise ValueError("Excel-Datei fehlt die Spalte 'phi'. Bitte Generator neu ausführen.")

# =============================================================================
# 2. INITIALISIERUNG DES KÄLTEKREISES (IHX)
# =============================================================================

# Kondensator (2.2 m² PWT - realistisch für EFH)
condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=2.2,
    secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=15, thickness=0.4e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
)

# Verdampfer (108 m² Außenfläche, Ratio 105 -> ~1m² Innen)
evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=108.0,
    secondary_medium="air", flow_type="cross",
    ratio_outer_to_inner_area=105,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=1e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50)
)

# IHX (Klein, Rohr-in-Rohr)
ihx_component = ihx_ntu.IHX_NTU(
    A=0.15,
    alpha_low_side=1000,
    alpha_high_side=2000,
    dT_pinch_min=0.5,
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=15, thickness=1e-3)
)

# Ventile: Angepasst auf 6e-6 für Stabilität bei Variation
expansion_valve_high = Bernoulli(A=0.000002)
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

algorithm = Iteration(show_iteration=True, max_num_iterations=100)

# =============================================================================
# 3. ERSTELLEN DER EINGABE-LISTE
# =============================================================================

inputs_list = []

# Iterations-Listen
speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]
superheats = [5, 10, 15]
air_vol_flows = [0.6]#[0.5, 0.6, 0.7]  # m³/s

# IHX Spezifisch: Variation der Ventilöffnung (Hochdruck-Seite)
# 3 Stufen wie gewünscht
hpev_openings = [0.5] #[0.5, 0.7, 1.0]

logging.info(f"Generiere Input-Liste. Variation über:")
logging.info(f" -> {len(df_betriebspunkte)} Zeilen (BP + Feuchte)")
logging.info(f" -> {len(air_vol_flows)} Volumenströme")
logging.info(f" -> {len(hpev_openings)} Ventilöffnungen (HPEV)")

for index, row in df_betriebspunkte.iterrows():
    T_amb_K = row['T_ambient'] + 273.15
    #current_phi = row['phi']  # Feuchte aus Excel

    # Schleife: Luftvolumenstrom
    for v_flow in air_vol_flows:
        # Dichte berechnen für Massenstrom
        rho_air = 101325 / (287 * T_amb_K)
        m_flow_air_calc = v_flow * rho_air

        # Schleife: Ventilöffnung (IHX Control)
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

                    # Feuchte & Volumenstrom (Referenz)
                    #control_inputs.set(name="phi", value=current_phi, unit="-")
                    control_inputs.set(name="V_flow_air", value=v_flow, unit="m3/s")

                    # LPEV Regelung (Überhitzung)
                    control_inputs.set(name="dT_eva_superheating", value=sh, unit="K")
                    control_inputs.set(name="dT_con_subcooling", value=0, unit="K")

                    # HPEV Regelung (Feste Öffnung)
                    # WICHTIG: KEIN Subcooling vorgeben, da Ventilposition fix ist!
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
output_filename = "IHX_Propane_VarFlowHum_ValveStudy_noHumidity.xlsx"

logging.info("Starte Simulation...")

try:
    calc_multiple_states(
        save_path=save_directory,
        flowsheet=heat_pump,
        inputs=inputs_list,
        algorithm=algorithm,
        use_multiprocessing=False,  # Stabil lassen
        raise_errors=False,
        with_unit_and_description=True
    )

    # Datei umbenennen
    res_file = save_directory / "IHX_Propane.xlsx"  # Standardname bei diesem Flowsheet checken
    if not res_file.exists():
        res_file = save_directory / "results.xlsx"

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