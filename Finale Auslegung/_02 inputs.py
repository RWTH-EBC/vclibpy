import pandas as pd
import numpy as np
import os
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs


def load_betriebspunkte(filename):
    """Lädt die grundlegenden Umgebungstemperaturen aus der Excel-Vorgabe."""
    if not os.path.exists(filename):
        raise FileNotFoundError(f"Datei '{filename}' fehlt. Bitte erst BP Generator ausführen.")
    return pd.read_excel(filename)


def generate_full_input_list(df_betriebspunkte):
    """
    Erstellt die vollständige Liste der zu simulierenden Inputs, indem
    verschiedene Anlagenparameter permutiert werden (Drehzahl, Überhitzung etc.).
    Gibt eine Liste von vclibpy 'Inputs'-Objekten zurück.
    """
    inputs_list = []

    # Permutations-Parameter für die Kennfelder
    speeds_rel = [round(x, 1) for x in np.arange(0.3, 1.1, 0.1)]  # Verdichter-Drehzahlen (30% bis 100%)
    superheats = [5, 10, 15]  # Überhitzungen am Verdampferaustritt
    air_vol_flows = [1.4]  # Luftvolumenstrom
    hpev_openings = [1.0]  # Öffnungsgrade des Ventils (vereinfacht auf 1.0)

    for index, row in df_betriebspunkte.iterrows():
        T_amb_K = row['T_ambient'] + 273.15

        for v_flow in air_vol_flows:
            # Korrektur des Luftmassenstroms über die temperaturabhängige Dichte
            rho_air = 101325 / (287 * T_amb_K)
            m_flow_air_calc = v_flow * rho_air

            for valve_pos in hpev_openings:
                for sh in superheats:
                    for n_rel in speeds_rel:
                        inp = create_single_input(
                            T_amb_K, m_flow_air_calc, v_flow,
                            row['T_kond_in'] + 273.15, row['m_flow_kond'],
                            n_rel, sh, valve_pos
                        )
                        inputs_list.append(inp)
    return inputs_list


def create_single_input(T_amb_K, m_flow_air, v_flow_air, T_kond_in_K, m_flow_kond, n_rel, sh, valve_pos):
    """
    Kapselt die Initialisierung eines einzelnen vclibpy-Datenmodells (Inputs).
    Übergibt Randbedingungen getrennt für Steuerung, Verdampfer und Kondensator.
    """
    evap_inputs = HeatExchangerInputs(T_in=T_amb_K, m_flow=m_flow_air)
    cond_inputs = HeatExchangerInputs(T_in=T_kond_in_K, m_flow=m_flow_kond)

    control_inputs = ControlInputs()
    control_inputs.set(name="n", value=n_rel, unit="Hz")
    control_inputs.set(name="T_ambient", value=T_amb_K, unit="K")
    control_inputs.set(name="V_flow_air", value=v_flow_air, unit="m3/s")
    control_inputs.set(name="dT_eva_superheating", value=sh, unit="K")
    control_inputs.set(name="dT_con_subcooling", value=2, unit="K")  # Feste Unterkühlung
    control_inputs.set(name="opening", value=valve_pos, unit="-")

    return Inputs(control=control_inputs, evaporator=evap_inputs, condenser=cond_inputs)