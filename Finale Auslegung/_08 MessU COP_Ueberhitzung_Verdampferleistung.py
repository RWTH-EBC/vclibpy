import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# Bibliotheken zur Messunsicherheitsberechnung nach GUM
from GTC import ureal
from pyfluids import Fluid, FluidsList, Input

# Eigene Hardware-Modelle zur Fehlerfortpflanzung (Sensoren und Klemmen)
from gum_ebc.Measurement import Measurement
from gum_ebc.configuration import *

from gum_ebc.Sensors import *
from gum_ebc.Terminals import *

from gum_ebc.Sensors.Pressure import Pressure
from gum_ebc.Sensors.Temperature import Temperature
from gum_ebc.Sensors.Krohne_OptiMass import Krohne_OptiMass
from gum_ebc.Sensors.EnergyMeter import EnergyMeter

from gum_ebc.Terminals.EL3154 import EL3154
from gum_ebc.Terminals.EL3182 import EL3182
from gum_ebc.Terminals.EL3202_0010 import EL3202_0010
from gum_ebc.Terminals.EL3443_0010 import EL3443_0010


# =============================================================================
# 0) USER SETTINGS & KONFIGURATION
# =============================================================================

# Pfad zu den thermodynamischen Betriebspunkten
EXCEL_PATH = r"C:\Users\mbc-asz\PycharmProjects\vclibpy\Finale Simulation\mbc-asz Simulation\pythonProject\Finale Auslegung\Auswahl_Punkte_Vergleich.xlsx"

# Ausgabeverzeichnis für diese erweiterte Analyse
OUTPUT_DIR = Path(
    r"C:\Users\mbc-asz\PycharmProjects\vclibpy\Finale Simulation\mbc-asz Simulation\pythonProject\Finale Auslegung\Messunsicherheit_Qevap_Ueberhitzung_COP_svg"
)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

PLOT_DIR = OUTPUT_DIR / "Plots"
PLOT_DIR.mkdir(parents=True, exist_ok=True)

# Erweiterungsfaktor für die GUM Unsicherheit (k=2 -> ~95% Konfidenzintervall)
K_EXPANSION = 2

# -----------------------------------------------------------------------------
# Stoffwert-/Sättigungsannahmen (Numerische Stabilisierung)
# -----------------------------------------------------------------------------
# WICHTIG:
# PyfluidsEnthalpy.py muss intern auf FluidsList.nPropane angepasst sein.
#
# Vor dem EEV liegt ohne IHX häufig Unterkühlung = 0 K vor.
# Daher wird für h(T,p) vor dem EEV eine leichte künstliche Unterkühlung
# angenommen, damit die Enthalpieberechnung numerisch stabil bleibt und
# keine Fehler am Phasenübergang (Siedelinie) entstehen.
BEFORE_EEV_SUBCOOLING_SHIFT_K = 1.0

# Für die COP-Berechnung:
# Falls der Kondensatoraustritt laut Simulation praktisch auf der
# Sättigungslinie liegt (dT_subcooling ~ 0), wird auch dort ein kleiner
# Shift genutzt.
COND_OUTLET_SUBCOOLING_SHIFT_K = 1.0
COND_OUTLET_ZERO_SC_THRESHOLD_K = 0.1

# -----------------------------------------------------------------------------
# Definition der Messketten (Hardware)
# -----------------------------------------------------------------------------

# Temperatursensoren
TEMP_SENSOR_TYPE = "PT100A"
TEMP_TERMINAL_CLASS = EL3202_0010

# Drucksensoren
PRESSURE_TERMINAL_CLASS = EL3154

# Verdampferseite: Fokus hier auf den 10 bar Sensor (für höhere Genauigkeit)
EVAP_PRESSURE_SENSOR_TYPE = "ICS_Schneider_IMP331-p10"
EVAP_PRESSURE_FULL_SCALE = 10 * 1e5

# Vor EEV / Hochdruckseite: 40 bar Sensor
HIGH_PRESSURE_SENSOR_TYPE = "ICS_Schneider_IMP331-p40"
HIGH_PRESSURE_FULL_SCALE = 40 * 1e5

# Massenstromsensor (Coriolis)
MASSFLOW_SENSOR_TYPE = "6400c_DN08"
MASSFLOW_TERMINAL_CLASS = EL3182
MASSFLOW_FULL_SCALE = 40 / 1000  # 40 g/s

# Leistungsmessung (Elektrische Wirkleistung des Verdichters)
POWER_SENSOR_TYPE = "iEM_3155"
POWER_TERMINAL_CLASS = EL3443_0010

# Falls kein Stromwert separat vorliegt, wird hier ein Platzhalter > 0.5 A
# gesetzt, damit im EnergyMeter-Modell der 1%-Fehlerpfad gewählt wird.
POWER_SENSOR_CURRENT_A = [1.0]


# =============================================================================
# 1) LABELS FÜR PLOTS UND EXPORTE
# =============================================================================

BP_LABELS = {
    "BP967_Kv max SC": "Kv max",
    "BP40_Kv min SC": "Kv min",
    "BP1007_max Q": "max Q",
    "BP568_min Q": "min Q",
    "BP551_Nenn": "Nenn",
}

STATE_LABELS = {
    # Q_evap (Verdampferleistung)
    "m_ref": "m_ref",
    "T_out_evap": "T_out",
    "p_out_evap": "p_out",
    "T_before_EEV": "T_vor EEV",
    "p_before_EEV_40bar": "p_vor EEV\n40 bar",

    # Superheat (Überhitzung)
    "T_sat_out": "T_sat(p_out)",

    # COP
    "T_comp_out": "T_nach Verd.",
    "p_comp_out": "p_nach Verd.",
    "T_con_out": "T_nach Kond.",
    "p_con_out": "p_nach Kond.",
    "P_el": "P_el",
}

METRIC_LABELS = {
    "Q_evap": "Verdampferleistung",
    "superheat": "Überhitzung",
    "COP": "COP",
}

METRIC_SHORT_LABELS = {
    "Q_evap": r"$Q_{evap}$",
    "superheat": r"$\Delta T_{ÜH}$",
    "COP": r"$COP$",
}

# Reihenfolge der Balken in den Diagrammen, sortiert nach Metrik
STATE_ORDER_BY_METRIC = {
    "Q_evap": [
        "m_ref",
        "T_out_evap",
        "p_out_evap",
        "T_before_EEV",
        "p_before_EEV_40bar",
    ],
    "superheat": [
        "T_out_evap",
        "p_out_evap",
    ],
    "COP": [
        "m_ref",
        "T_comp_out",
        "p_comp_out",
        "T_con_out",
        "p_con_out",
        "P_el",
    ]
}


def short_bp_name(bp):
    """Kürzt den Betriebspunkt-String."""
    return BP_LABELS.get(bp, bp.replace("BP", "").replace("_", " "))


def short_state_name(state):
    """Holt die lesbare Kurzform einer Zustandsgröße."""
    return STATE_LABELS.get(state, state)


# =============================================================================
# 2) HELPER-FUNKTIONEN ZUR SENSITIVITÄTSANALYSE (LEAVE-ONE-OUT)
# =============================================================================

def zero_uncertainty(obj):
    """
    Setzt die Unsicherheit eines Objekts auf Null, um den isolierten
    Einfluss anderer Komponenten berechnen zu können.
    """
    if hasattr(obj, "a"):
        obj.a = []
    return obj


def flags_with_disabled_component(disabled_component=None):
    """
    Erstellt ein Steuer-Dictionary. Wenn 'disabled_component' übergeben wird,
    wird genau diese Messkomponente auf 'False' gesetzt (Unsicherheit = 0).
    """
    flags = {
        # Verdampferbilanz / Superheat
        "m_sensor": True,
        "m_terminal": True,

        "T_out_sensor": True,
        "T_out_terminal": True,

        "p_out_sensor": True,
        "p_out_terminal": True,

        "T_before_EEV_sensor": True,
        "T_before_EEV_terminal": True,

        "p_before_EEV_40bar_sensor": True,
        "p_before_EEV_40bar_terminal": True,

        # COP
        "T2_sensor": True,
        "T2_terminal": True,

        "p2_sensor": True,
        "p2_terminal": True,

        "T3_sensor": True,
        "T3_terminal": True,

        "p3_sensor": True,
        "p3_terminal": True,

        "P_el_sensor": True,
        "P_el_terminal": True,
    }

    if disabled_component is None:
        return flags

    if disabled_component not in flags:
        raise ValueError(f"Unbekannte Komponente: {disabled_component}")

    flags[disabled_component] = False
    return flags


# =============================================================================
# 3) MESSOBJEKTE (GUM-INITIALISIERUNG)
# =============================================================================

def create_temperature(T_K, active_sensor=True, active_terminal=True):
    """Erstellt GUM-Objekt für Temperatur."""
    values = [float(T_K)]

    sensor = Temperature(
        type=TEMP_SENSOR_TYPE,
        values=values
    )

    terminal = TEMP_TERMINAL_CLASS(
        type="PT100"
    )

    if not active_sensor:
        sensor = zero_uncertainty(sensor)

    if not active_terminal:
        terminal = zero_uncertainty(terminal)

    return Measurement(
        values=values,
        sensor=sensor,
        terminal=terminal
    )()


def create_pressure(
    p_Pa,
    pressure_sensor_type,
    pressure_full_scale,
    active_sensor=True,
    active_terminal=True
):
    """Erstellt GUM-Objekt für Druck."""
    values = [float(p_Pa)]

    sensor = Pressure(
        type=pressure_sensor_type,
        values=values,
        full_scale=pressure_full_scale
    )

    terminal = PRESSURE_TERMINAL_CLASS(
        full_scale=pressure_full_scale
    )

    if not active_sensor:
        sensor = zero_uncertainty(sensor)

    if not active_terminal:
        terminal = zero_uncertainty(terminal)

    return Measurement(
        values=values,
        sensor=sensor,
        terminal=terminal
    )()


def create_massflow(
    m_kg_s,
    p_for_sensor,
    T_for_sensor,
    active_sensor=True,
    active_terminal=True
):
    """
    Erstellt GUM-Objekt für den Massenstrom (Coriolis).
    Das Modell nutzt Druck und Temperatur für interne Dichtekorrekturen.
    """
    values = [float(m_kg_s)]

    sensor = Krohne_OptiMass(
        type=MASSFLOW_SENSOR_TYPE,
        values=values,
        pressure=p_for_sensor.x,
        temperature=T_for_sensor.x
    )

    terminal = MASSFLOW_TERMINAL_CLASS(
        full_scale=MASSFLOW_FULL_SCALE
    )

    if not active_sensor:
        sensor = zero_uncertainty(sensor)

    if not active_terminal:
        terminal = zero_uncertainty(terminal)

    return Measurement(
        values=values,
        sensor=sensor,
        terminal=terminal
    )()


def create_electric_power(
    P_el_kW,
    active_sensor=True,
    active_terminal=True
):
    """Erstellt GUM-Objekt für elektrische Leistungsaufnahme."""
    values = [float(P_el_kW)]

    sensor = EnergyMeter(
        type=POWER_SENSOR_TYPE,
        values=values,
        current=POWER_SENSOR_CURRENT_A
    )

    terminal = POWER_TERMINAL_CLASS(
        type="Power",
        values=values
    )

    if not active_sensor:
        sensor = zero_uncertainty(sensor)

    if not active_terminal:
        terminal = zero_uncertainty(terminal)

    return Measurement(
        values=values,
        sensor=sensor,
        terminal=terminal
    )()


def enthalpy_refrigerant(p, T):
    """
    Enthalpieberechnung über PyfluidsEnthalpy (GTC-kompatibel).
    Voraussetzung: PyfluidsEnthalpy.py nutzt intern FluidsList.nPropane.
    """
    return PyfluidsEnthalpy(
        temperature=T,
        pressure=p
    )()


def saturation_temperature_nominal_K(p_Pa):
    """
    Sättigungstemperatur von R290 bei gegebenem Druck.
    Für reines Propan ist bubble-/dew-point temperaturgleich.
    """
    try:
        fluid = Fluid(FluidsList.nPropane).with_state(
            Input.pressure(float(p_Pa)),
            Input.quality(1.0)
        )
        return float(fluid.temperature) + 273.15
    except Exception:
        # Fallback, falls die pyfluids-Version anders aufgebaut ist
        raise RuntimeError(
            "Sättigungstemperatur konnte nicht mit pyfluids berechnet werden. "
            "Bitte die Funktion saturation_temperature_nominal_K an deine "
            "pyfluids-Version anpassen."
        )


def saturation_temperature_from_pressure(p_measurement):
    """
    Erzeugt eine ureal für T_sat(p) auf Basis der Unsicherheit des Drucks.
    Die Standardunsicherheit wird per numerischer Sensitivität (Differenz) bestimmt.
    """
    p0 = float(p_measurement.x)
    up = abs(float(p_measurement.u))

    T0 = saturation_temperature_nominal_K(p0)

    if up == 0:
        return ureal(T0, 0.0)

    p_plus = max(p0 + up, 1.0)
    p_minus = max(p0 - up, 1.0)

    T_plus = saturation_temperature_nominal_K(p_plus)
    T_minus = saturation_temperature_nominal_K(p_minus)

    # Lineare Approximation mit +/- 1 sigma des Drucks
    u_Tsat = abs(T_plus - T_minus) / 2.0

    return ureal(T0, u_Tsat)


# =============================================================================
# 4) EXCEL EINLESEN
# =============================================================================

def normalize_column_name(col):
    """Entfernt Umbrüche zur sicheren Erkennung von Spaltennamen."""
    return str(col).strip().replace("\n", " ")


def find_column(df, startswith_text):
    for col in df.columns:
        col_norm = normalize_column_name(col)
        if col_norm.startswith(startswith_text):
            return col

    raise KeyError(
        f"Spalte nicht gefunden. Gesucht wurde Anfang: '{startswith_text}'\n"
        f"Vorhandene Spalten:\n{list(df.columns)}"
    )


def find_optional_column(df, startswith_text):
    for col in df.columns:
        col_norm = normalize_column_name(col)
        if col_norm.startswith(startswith_text):
            return col
    return None


def optional_float(row, col):
    if col is None:
        return np.nan

    value = row[col]

    if pd.isna(value):
        return np.nan

    return float(value)


def load_operating_points(excel_path):
    """Lädt alle simulierten Betriebspunkte für die Gesamtsystem-Analyse."""
    df = pd.read_excel(excel_path)

    col_state = find_column(df, "State Number")
    col_desc = find_column(df, "Punktbeschreibung")

    col_m = find_column(df, "m_flow_ref in kg/s")

    col_T1 = find_column(df, "T_1 in K")
    col_p1 = find_column(df, "p_1 in Pa")

    col_T2 = find_column(df, "T_2 in K")
    col_p2 = find_column(df, "p_2 in Pa")

    col_T3 = find_column(df, "T_3 in K")
    col_p3 = find_column(df, "p_3 in Pa")

    col_T4 = find_optional_column(df, "T_4 in K")
    col_p4 = find_optional_column(df, "p_4 in Pa")

    col_H1 = find_optional_column(df, "H_1")
    col_H2 = find_optional_column(df, "H_2")
    col_H3 = find_optional_column(df, "H_3")
    col_H4 = find_optional_column(df, "H_4")

    col_superheat = find_optional_column(df, "dT_eva_superheating in K")
    col_subcooling = find_optional_column(df, "dT_con_subcooling in K")

    col_Pel = find_column(df, "P_el in W")
    col_Qcon = find_optional_column(df, "Q_con in W")
    col_COP = find_optional_column(df, "COP in -")

    points = []

    for _, row in df.iterrows():
        if pd.isna(row[col_state]):
            continue

        state_number = int(row[col_state])
        description = str(row[col_desc])

        point = {
            "state": state_number,
            "description": description,
            "name": f"BP{state_number}_{description}",

            "m_ref_kg_s": float(row[col_m]),

            # Zustand 1: Verdampferaustritt
            "T_out_evap_K": float(row[col_T1]),
            "p_out_evap_Pa": float(row[col_p1]),

            # Zustand 2: Verdichteraustritt
            "T_comp_out_K": float(row[col_T2]),
            "p_comp_out_Pa": float(row[col_p2]),

            # Zustand 3: Kondensatoraustritt / vor EEV
            "T_before_EEV_K": float(row[col_T3]),
            "p_before_EEV_Pa": float(row[col_p3]),

            # optional Zustand 4
            "T_evap_in_K": optional_float(row, col_T4),
            "p_evap_in_Pa": optional_float(row, col_p4),

            "h_out_evap_sim_Jkg": optional_float(row, col_H1),
            "h_comp_out_sim_Jkg": optional_float(row, col_H2),
            "h_before_EEV_sim_Jkg": optional_float(row, col_H3),
            "h_evap_in_sim_Jkg": optional_float(row, col_H4),

            "dT_superheat_K": optional_float(row, col_superheat),
            "dT_subcooling_K": optional_float(row, col_subcooling),

            "P_el_W": float(row[col_Pel]),
            "Q_con_sim_W": optional_float(row, col_Qcon),
            "COP_sim": optional_float(row, col_COP),
        }

        points.append(point)

    return points


# =============================================================================
# 5) METRIK-BERECHNUNGEN (FEHLERFORTPFLANZUNG)
# =============================================================================

def calculate_evaporator_balance(bp, disabled_component=None, expand=False):
    """
    Berechnet die Kälteleistung des Verdampfers.
    Q_evap = m_ref * (h_out - h_in)
    Dabei pflanzen sich alle Sensoren-Unsicherheiten durch h(T,p) fort.
    """
    flags = flags_with_disabled_component(disabled_component)

    # Zustand 1: Verdampferaustritt
    T_out = create_temperature(
        bp["T_out_evap_K"],
        active_sensor=flags["T_out_sensor"],
        active_terminal=flags["T_out_terminal"]
    )

    p_out = create_pressure(
        bp["p_out_evap_Pa"],
        pressure_sensor_type=EVAP_PRESSURE_SENSOR_TYPE,
        pressure_full_scale=EVAP_PRESSURE_FULL_SCALE,
        active_sensor=flags["p_out_sensor"],
        active_terminal=flags["p_out_terminal"]
    )

    h_out = enthalpy_refrigerant(
        p=p_out,
        T=T_out
    )

    # Zustand 3: vor EEV
    # Um numerische Instabilitäten auf der Sättigungslinie zu vermeiden,
    # wird eine künstliche Unterkühlung eingeführt.
    T_before_EEV_for_h_K = bp["T_before_EEV_K"] - BEFORE_EEV_SUBCOOLING_SHIFT_K

    T_before_EEV = create_temperature(
        T_before_EEV_for_h_K,
        active_sensor=flags["T_before_EEV_sensor"],
        active_terminal=flags["T_before_EEV_terminal"]
    )

    p_before_EEV = create_pressure(
        bp["p_before_EEV_Pa"],
        pressure_sensor_type=HIGH_PRESSURE_SENSOR_TYPE,
        pressure_full_scale=HIGH_PRESSURE_FULL_SCALE,
        active_sensor=flags["p_before_EEV_40bar_sensor"],
        active_terminal=flags["p_before_EEV_40bar_terminal"]
    )

    h_before_EEV = enthalpy_refrigerant(
        p=p_before_EEV,
        T=T_before_EEV
    )

    # Isenthalpe Drosselung im EEV
    h_in_evap = h_before_EEV

    m_ref = create_massflow(
        bp["m_ref_kg_s"],
        p_for_sensor=p_out,
        T_for_sensor=T_out,
        active_sensor=flags["m_sensor"],
        active_terminal=flags["m_terminal"]
    )

    Q_evap = (m_ref * (h_out - h_in_evap)) / 1000

    if expand:
        Q_evap = expand_uncertainty(Q_evap, k=K_EXPANSION)

    return {
        "Q_evap": Q_evap,
        "m_ref": m_ref,
        "T_out": T_out,
        "p_out": p_out,
        "h_out": h_out,
        "T_before_EEV": T_before_EEV,
        "p_before_EEV": p_before_EEV,
        "h_before_EEV": h_before_EEV,
        "delta_h": h_out.x - h_in_evap.x,
    }


def calculate_superheat(bp, disabled_component=None, expand=False):
    """Berechnet die Überhitzung (Isoliert betrachtet)."""
    flags = flags_with_disabled_component(disabled_component)

    T_out = create_temperature(
        bp["T_out_evap_K"],
        active_sensor=flags["T_out_sensor"],
        active_terminal=flags["T_out_terminal"]
    )

    p_out = create_pressure(
        bp["p_out_evap_Pa"],
        pressure_sensor_type=EVAP_PRESSURE_SENSOR_TYPE,
        pressure_full_scale=EVAP_PRESSURE_FULL_SCALE,
        active_sensor=flags["p_out_sensor"],
        active_terminal=flags["p_out_terminal"]
    )

    T_sat_out = saturation_temperature_from_pressure(p_out)

    dT_ueh = T_out - T_sat_out

    if expand:
        dT_ueh = expand_uncertainty(dT_ueh, k=K_EXPANSION)

    return {
        "dT_ueh": dT_ueh,
        "T_out": T_out,
        "p_out": p_out,
        "T_sat_out": T_sat_out,
    }


def calculate_condenser_heat_flow(bp, disabled_component=None):
    """
    Berechnet die Heizleistung am Kondensator für den COP.
    Q_H = m_ref * (h2 - h3) / 1000
    h2: Verdichteraustritt
    h3: Kondensatoraustritt
    """
    flags = flags_with_disabled_component(disabled_component)

    # Zustand 2: Verdichteraustritt
    T2 = create_temperature(
        bp["T_comp_out_K"],
        active_sensor=flags["T2_sensor"],
        active_terminal=flags["T2_terminal"]
    )

    p2 = create_pressure(
        bp["p_comp_out_Pa"],
        pressure_sensor_type=HIGH_PRESSURE_SENSOR_TYPE,
        pressure_full_scale=HIGH_PRESSURE_FULL_SCALE,
        active_sensor=flags["p2_sensor"],
        active_terminal=flags["p2_terminal"]
    )

    h2 = enthalpy_refrigerant(
        p=p2,
        T=T2
    )

    # Zustand 3: Kondensatoraustritt
    T3_for_h_K = bp["T_before_EEV_K"]
    used_shift = 0.0

    if (
        not pd.isna(bp["dT_subcooling_K"])
        and bp["dT_subcooling_K"] <= COND_OUTLET_ZERO_SC_THRESHOLD_K
    ):
        T3_for_h_K = T3_for_h_K - COND_OUTLET_SUBCOOLING_SHIFT_K
        used_shift = COND_OUTLET_SUBCOOLING_SHIFT_K

    T3 = create_temperature(
        T3_for_h_K,
        active_sensor=flags["T3_sensor"],
        active_terminal=flags["T3_terminal"]
    )

    p3 = create_pressure(
        bp["p_before_EEV_Pa"],
        pressure_sensor_type=HIGH_PRESSURE_SENSOR_TYPE,
        pressure_full_scale=HIGH_PRESSURE_FULL_SCALE,
        active_sensor=flags["p3_sensor"],
        active_terminal=flags["p3_terminal"]
    )

    h3 = enthalpy_refrigerant(
        p=p3,
        T=T3
    )

    # Massenstrom
    m_ref = create_massflow(
        bp["m_ref_kg_s"],
        p_for_sensor=p2,
        T_for_sensor=T2,
        active_sensor=flags["m_sensor"],
        active_terminal=flags["m_terminal"]
    )

    Q_H = (m_ref * (h2 - h3)) / 1000

    return {
        "Q_H": Q_H,
        "m_ref": m_ref,
        "T2": T2,
        "p2": p2,
        "h2": h2,
        "T3": T3,
        "p3": p3,
        "h3": h3,
        "T3_shift_used_K": used_shift,
    }


def calculate_cop(bp, disabled_component=None, expand=False):
    """Berechnet den COP als Verhältnis von Q_H und P_el."""
    flags = flags_with_disabled_component(disabled_component)

    cond = calculate_condenser_heat_flow(
        bp=bp,
        disabled_component=disabled_component
    )

    P_el = create_electric_power(
        bp["P_el_W"] / 1000.0,   # P_el in kW für Konsistenz
        active_sensor=flags["P_el_sensor"],
        active_terminal=flags["P_el_terminal"]
    )

    COP = cond["Q_H"] / P_el

    if expand:
        COP = expand_uncertainty(COP, k=K_EXPANSION)

    cond["P_el"] = P_el
    cond["COP"] = COP

    return cond


# =============================================================================
# 6) METADATA ZUR BEITRAGSANALYSE (Für dynamische Auswertung)
# =============================================================================

METRIC_CONFIG = {
    "Q_evap": {
        "label": "Verdampferleistung",
        "short_label": r"$Q_{evap}$",
        "value_key": "Q_evap",
        "unit": "kW",
        "components": [
            ("m_ref", "Sensor", "m_sensor"),
            ("m_ref", "Klemme", "m_terminal"),

            ("T_out_evap", "Sensor", "T_out_sensor"),
            ("T_out_evap", "Klemme", "T_out_terminal"),

            ("p_out_evap", "Sensor", "p_out_sensor"),
            ("p_out_evap", "Klemme", "p_out_terminal"),

            ("T_before_EEV", "Sensor", "T_before_EEV_sensor"),
            ("T_before_EEV", "Klemme", "T_before_EEV_terminal"),

            ("p_before_EEV_40bar", "Sensor", "p_before_EEV_40bar_sensor"),
            ("p_before_EEV_40bar", "Klemme", "p_before_EEV_40bar_terminal"),
        ],
    },
    "superheat": {
        "label": "Überhitzung",
        "short_label": r"$\Delta T_{ÜH}$",
        "value_key": "dT_ueh",
        "unit": "K",
        "components": [
            ("T_out_evap", "Sensor", "T_out_sensor"),
            ("T_out_evap", "Klemme", "T_out_terminal"),

            ("p_out_evap", "Sensor", "p_out_sensor"),
            ("p_out_evap", "Klemme", "p_out_terminal"),
        ],
    },
    "COP": {
        "label": "COP",
        "short_label": r"$COP$",
        "value_key": "COP",
        "unit": "-",
        "components": [
            ("m_ref", "Sensor", "m_sensor"),
            ("m_ref", "Klemme", "m_terminal"),

            ("T_comp_out", "Sensor", "T2_sensor"),
            ("T_comp_out", "Klemme", "T2_terminal"),

            ("p_comp_out", "Sensor", "p2_sensor"),
            ("p_comp_out", "Klemme", "p2_terminal"),

            ("T_con_out", "Sensor", "T3_sensor"),
            ("T_con_out", "Klemme", "T3_terminal"),

            ("p_con_out", "Sensor", "p3_sensor"),
            ("p_con_out", "Klemme", "p3_terminal"),

            ("P_el", "Sensor", "P_el_sensor"),
            ("P_el", "Klemme", "P_el_terminal"),
        ],
    }
}


def calculate_metric(metric_name, bp, disabled_component=None, expand=False):
    """Wrapper zum Ausführen der spezifizierten GUM-Berechnung."""
    if metric_name == "Q_evap":
        return calculate_evaporator_balance(
            bp=bp,
            disabled_component=disabled_component,
            expand=expand
        )
    elif metric_name == "superheat":
        return calculate_superheat(
            bp=bp,
            disabled_component=disabled_component,
            expand=expand
        )
    elif metric_name == "COP":
        return calculate_cop(
            bp=bp,
            disabled_component=disabled_component,
            expand=expand
        )
    else:
        raise ValueError(f"Unbekannte Metrik: {metric_name}")


# =============================================================================
# 7) BEITRAGSANALYSE (Leave-one-out)
# =============================================================================

def contribution_analysis(metric_name, bp):
    """
    Berechnet die prozentualen Varianz-Anteile aller Komponenten für die
    ausgewählte Metrik (Q_evap, Superheat, COP).
    """
    config = METRIC_CONFIG[metric_name]
    value_key = config["value_key"]

    full = calculate_metric(
        metric_name=metric_name,
        bp=bp,
        disabled_component=None,
        expand=False
    )

    y_full = full[value_key]
    u_full = abs(y_full.u)
    variance_full = u_full ** 2

    y_expanded = expand_uncertainty(y_full, k=K_EXPANSION)

    contribution_rows = []

    # Systemvarianz iterativ ohne jeweilige Komponente i berechnen
    for state_variable, source, disabled_component in config["components"]:
        without_i = calculate_metric(
            metric_name=metric_name,
            bp=bp,
            disabled_component=disabled_component,
            expand=False
        )

        u_without_i = abs(without_i[value_key].u)
        variance_without_i = u_without_i ** 2

        variance_i = variance_full - variance_without_i
        if variance_i < 0:
            variance_i = 0.0

        u_i = variance_i ** 0.5

        contribution_rows.append({
            "state": bp["state"],
            "description": bp["description"],
            "bp": bp["name"],
            "bp_short": short_bp_name(bp["name"]),

            "metric": metric_name,
            "metric_label": config["label"],
            "metric_short_label": config["short_label"],

            "state_variable": state_variable,
            "state_variable_short": short_state_name(state_variable),
            "source": source,

            "u_i": u_i,
            "variance_i": variance_i,

            "u_full": u_full,
            "u_without_i": u_without_i,
        })

    # Gewichtung
    variance_sum = sum(row["variance_i"] for row in contribution_rows)

    for row in contribution_rows:
        if variance_sum > 0:
            row["share_percent"] = row["variance_i"] / variance_sum * 100
        else:
            row["share_percent"] = 0.0

    dominant = max(contribution_rows, key=lambda r: r["share_percent"])

    result_row = {
        "state": bp["state"],
        "description": bp["description"],
        "bp": bp["name"],
        "bp_short": short_bp_name(bp["name"]),

        "metric": metric_name,
        "metric_label": config["label"],
        "metric_short_label": config["short_label"],
        "unit": config["unit"],

        "value": y_full.x,
        "u": y_full.u,
        "U": y_expanded.u,
        "rel_u_percent": abs(y_full.u / y_full.x) * 100 if y_full.x != 0 else np.nan,
        "rel_U_percent": abs(y_expanded.u / y_expanded.x) * 100 if y_expanded.x != 0 else np.nan,

        "dominant_variable": dominant["state_variable"],
        "dominant_variable_short": dominant["state_variable_short"],
        "dominant_source": dominant["source"],
        "dominant_share_percent": dominant["share_percent"],

        "variance_full": variance_full,
        "variance_sum_contributions": variance_sum,
    }

    # Zusätzliche Debug-/Info-Werte zur Plausibilisierung in den Tabellen
    if metric_name == "Q_evap":
        result_row["delta_h_Jkg"] = full["delta_h"]
        result_row["m_ref_kg_s"] = full["m_ref"].x

    if metric_name == "superheat":
        result_row["T_sat_out_K"] = full["T_sat_out"].x

    if metric_name == "COP":
        result_row["Q_H_kW"] = full["Q_H"].x
        result_row["P_el_kW"] = full["P_el"].x
        result_row["m_ref_kg_s"] = full["m_ref"].x
        result_row["T3_shift_used_K"] = full["T3_shift_used_K"]

    return result_row, contribution_rows


# =============================================================================
# 8) PLOTS UND EXPORTE
# =============================================================================

def result_box_text(result_row):
    """Erzeugt den kompakten Ergebnistext für die Bar-Charts."""
    metric = result_row["metric"]
    value = result_row["value"]
    U = result_row["U"]
    relU = result_row["rel_U_percent"]

    if metric == "Q_evap":
        return f"Q = {value:.2f} kW\nU = ±{U:.2f} kW\nrel. U = {relU:.1f} %"
    elif metric == "superheat":
        return f"ΔT = {value:.2f} K\nU = ±{U:.2f} K\nrel. U = {relU:.1f} %"
    elif metric == "COP":
        return f"COP = {value:.2f}\nU = ±{U:.2f}\nrel. U = {relU:.1f} %"

    return f"Wert = {value:.2f}\nU = ±{U:.2f}\nrel. U = {relU:.1f} %"


def plot_one_matrix_per_bp(contrib_df, results_df, output_dir):
    """
    Erstellt für jeden Betriebspunkt einen Multi-Plot mit den
    drei Hauptmetriken (Q, Superheat, COP) nebeneinander.
    """
    metrics_order = ["Q_evap", "superheat", "COP"]

    for bp in contrib_df["bp"].drop_duplicates():
        fig, axes = plt.subplots(1, 3, figsize=(17, 5), sharey=True)

        for ax, metric_name in zip(axes, metrics_order):
            df = contrib_df[
                (contrib_df["bp"] == bp)
                & (contrib_df["metric"] == metric_name)
            ].copy()

            rows = []

            for state in STATE_ORDER_BY_METRIC[metric_name]:
                sensor_share = df[
                    (df["state_variable"] == state)
                    & (df["source"] == "Sensor")
                ]["share_percent"].sum()

                terminal_share = df[
                    (df["state_variable"] == state)
                    & (df["source"] == "Klemme")
                ]["share_percent"].sum()

                rows.append({
                    "state": state,
                    "Sensor": sensor_share,
                    "Klemme": terminal_share,
                })

            plot_df = pd.DataFrame(rows)
            x = np.arange(len(plot_df))

            ax.bar(
                x,
                plot_df["Sensor"],
                label="Sensor"
            )

            ax.bar(
                x,
                plot_df["Klemme"],
                bottom=plot_df["Sensor"],
                label="Klemme"
            )

            ax.set_title(METRIC_LABELS[metric_name])
            ax.set_xticks(x)
            ax.set_xticklabels(
                [short_state_name(s) for s in plot_df["state"]],
                rotation=30,
                ha="right"
            )
            ax.set_ylim(0, 100)
            ax.grid(axis="y", alpha=0.3)

            r = results_df[
                (results_df["bp"] == bp)
                & (results_df["metric"] == metric_name)
            ]

            if not r.empty:
                text = result_box_text(r.iloc[0])

                ax.text(
                    0.02,
                    0.98,
                    text,
                    transform=ax.transAxes,
                    va="top",
                    ha="left",
                    bbox=dict(boxstyle="round", alpha=0.15)
                )

        axes[0].set_ylabel("Anteil an Gesamtvarianz in %")
        axes[0].legend(loc="upper right")

        fig.suptitle(f"Unsicherheitsbeiträge: {short_bp_name(bp)} (10 bar am Verdampfer)")

        plt.tight_layout()

        filename = f"01_matrix_{bp.replace(' ', '_').replace('/', '_')}.svg"
        file = output_dir / filename
        plt.savefig(file, dpi=300)
        plt.close(fig)


def export_dominant_contributions(contrib_df, output_dir):
    """Speichert eine CSV mit den Top-3 Unsicherheitsquellen pro Punkt und Metrik."""
    rows = []

    grouped = contrib_df.groupby(["bp", "metric"])

    for (bp, metric), group in grouped:
        top = group.sort_values("share_percent", ascending=False).head(3)

        for rank, (_, row) in enumerate(top.iterrows(), start=1):
            rows.append({
                "bp": bp,
                "bp_short": short_bp_name(bp),
                "metric": metric,
                "metric_label": METRIC_LABELS.get(metric, metric),
                "rank": rank,
                "state_variable": row["state_variable"],
                "state_variable_short": row["state_variable_short"],
                "source": row["source"],
                "share_percent": row["share_percent"],
            })

    out = pd.DataFrame(rows)

    out.to_csv(
        output_dir / "dominante_unsicherheitsquellen_top3.csv",
        sep=";",
        index=False
    )


# =============================================================================
# 9) MAIN (Programmablauf)
# =============================================================================

def main():
    """Führt die komplette Unsicherheitsanalyse über alle Metriken durch."""
    print("\nLese Betriebspunkte aus Excel...")
    operating_points = load_operating_points(EXCEL_PATH)

    print(f"Anzahl Betriebspunkte: {len(operating_points)}")
    for bp in operating_points:
        print(f"- {bp['name']}")

    print(
        "\nHinweis:\n"
        f"- Vor EEV wird für h(T,p) T3 - {BEFORE_EEV_SUBCOOLING_SHIFT_K} K verwendet.\n"
        f"- Am Kondensatoraustritt wird bei dT_subcooling <= {COND_OUTLET_ZERO_SC_THRESHOLD_K} K "
        f"für h3 zusätzlich T3 - {COND_OUTLET_SUBCOOLING_SHIFT_K} K verwendet."
    )

    result_rows = []
    contribution_rows_all = []

    metrics_order = ["Q_evap", "superheat", "COP"]

    for bp in operating_points:
        print("\n" + "=" * 100)
        print(f"Betriebspunkt: {bp['name']}")
        print("=" * 100)

        for metric_name in metrics_order:
            result_row, contribution_rows = contribution_analysis(
                metric_name=metric_name,
                bp=bp
            )

            result_rows.append(result_row)
            contribution_rows_all.extend(contribution_rows)

            print(
                f"{METRIC_LABELS[metric_name]}: "
                f"Wert = {result_row['value']:.4f} {result_row['unit']} | "
                f"U = ±{result_row['U']:.4f} {result_row['unit']} | "
                f"rel. U = {result_row['rel_U_percent']:.3f} %"
            )

            print(
                f"Dominant: {result_row['dominant_variable_short']} "
                f"({result_row['dominant_source']}, "
                f"{result_row['dominant_share_percent']:.1f} %)"
            )

    results_df = pd.DataFrame(result_rows)
    contrib_df = pd.DataFrame(contribution_rows_all)

    results_file = OUTPUT_DIR / "ergebnisse_Qevap_superheat_COP.csv"
    contrib_file = OUTPUT_DIR / "beitraege_unsicherheit_sensor_klemme_Qevap_superheat_COP.csv"

    results_df.to_csv(
        results_file,
        index=False,
        sep=";"
    )

    contrib_df.to_csv(
        contrib_file,
        index=False,
        sep=";"
    )

    print("\nErgebnisdateien gespeichert:")
    print(results_file)
    print(contrib_file)

    print("\nErstelle Plots...")

    plot_one_matrix_per_bp(
        contrib_df=contrib_df,
        results_df=results_df,
        output_dir=PLOT_DIR
    )

    export_dominant_contributions(
        contrib_df=contrib_df,
        output_dir=PLOT_DIR
    )

    print("\nFertig.")
    print("CSV-Ausgabeordner:")
    print(OUTPUT_DIR)
    print("Plot-Ausgabeordner:")
    print(PLOT_DIR)


if __name__ == "__main__":
    main()