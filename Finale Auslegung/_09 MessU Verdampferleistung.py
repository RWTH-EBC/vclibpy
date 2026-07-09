import os
from pathlib import Path

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from gum_ebc.Measurement import Measurement
from gum_ebc.configuration import *

# Wichtig:
# PyfluidsEnthalpy wird bei dir über diesen Sternimport verfügbar.
from gum_ebc.Sensors import *
from gum_ebc.Terminals import *

from gum_ebc.Sensors.Pressure import Pressure
from gum_ebc.Sensors.Temperature import Temperature
from gum_ebc.Sensors.Krohne_OptiMass import Krohne_OptiMass

from gum_ebc.Terminals.EL3154 import EL3154
from gum_ebc.Terminals.EL3182 import EL3182
from gum_ebc.Terminals.EL3202_0010 import EL3202_0010


# =============================================================================
# 0) USER SETTINGS & KONFIGURATION
# =============================================================================

# Pfad zu den thermodynamischen Betriebspunkten
EXCEL_PATH = r"C:\Users\mbc-asz\PycharmProjects\vclibpy\Finale Simulation\mbc-asz Simulation\pythonProject\Finale Auslegung\Auswahl_Punkte_Vergleich.xlsx"

# Ausgabeverzeichnisse
OUTPUT_DIR = Path(
    r"C:\Users\mbc-asz\PycharmProjects\vclibpy\Finale Simulation\mbc-asz Simulation\pythonProject\Finale Auslegung\Messunsicherheit_Ergebnisse_feedback_svg"
)
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

PLOT_DIR = OUTPUT_DIR / "Plots_ueberarbeitet"
PLOT_DIR.mkdir(parents=True, exist_ok=True)

DEBUG_DIR = OUTPUT_DIR / "Debug"
DEBUG_DIR.mkdir(parents=True, exist_ok=True)

# Erweiterungsfaktor für erweiterte Unsicherheit (k=2 entspricht ca. 95% Vertrauensniveau)
K_EXPANSION = 2

# -------------------------------------------------------------------------
# WICHTIG ZUR STOFFWERTBERECHNUNG
# -------------------------------------------------------------------------
# PyfluidsEnthalpy sollte in deiner Package-Datei auf FluidsList.nPropane
# geändert sein. nPropane entspricht R290/Propan.
#
# In PyfluidsEnthalpy.py sollte also nicht mehr FluidsList.R134a stehen,
# sondern FluidsList.nPropane.
# -------------------------------------------------------------------------

# -------------------------------------------------------------------------
# SENSITIVITÄTSANNAHME VOR DEM EEV (Numerische Stabilisierung)
# -------------------------------------------------------------------------
# Ohne IHX ist in der Simulation eine Unterkühlung von 0 K vorgegeben.
# Dadurch liegt der Punkt vor dem EEV numerisch exakt auf der Sättigungslinie.
# Die Enthalpieberechnung h(T,p) ist dort mathematisch problematisch, da
# leichte Messunsicherheiten ins Zweiphasengebiet führen könnten.
#
# Für diese Sensitivitätsvariante wird die Temperatur vor dem EEV für die
# Enthalpieberechnung künstlich um 1.0 K abgesenkt.
# Dadurch wird eine leicht unterkühlte Flüssigkeit angenommen.
#
# Vorteil:
# T_before_EEV bleibt als Eingangsgröße in der Unsicherheitsanalyse erhalten
# und das Stoffwertmodell liefert stabile Gradienten für die Fehlerfortpflanzung.
# -------------------------------------------------------------------------
BEFORE_EEV_SUBCOOLING_SHIFT_K = 1

# Definition der grundlegenden Messketten
TEMP_SENSOR_TYPE = "PT100A"
TEMP_TERMINAL_CLASS = EL3202_0010

PRESSURE_TERMINAL_CLASS = EL3154

MASSFLOW_SENSOR_TYPE = "6400c_DN08"
MASSFLOW_TERMINAL_CLASS = EL3182

# Beispiel: 40 g/s = 0.04 kg/s
# Prüfen, ob das zum realen Ausgangsbereich des Massenstromsensors passt.
MASSFLOW_FULL_SCALE = 40 / 1000

# -------------------------------------------------------------------------
# Varianten für eine zusätzliche Sensitivitätsanalyse der Massenstrommesskette
# -------------------------------------------------------------------------
# Die normale Verdampferbilanz verwendet weiterhin MASSFLOW_SENSOR_TYPE,
# MASSFLOW_TERMINAL_CLASS und MASSFLOW_FULL_SCALE.
#
# Für den zusätzlichen Vergleich wird untersucht, wie stark der parametrierte
# Messbereichsendwert (Full-Scale) des 4-20-mA-Ausgangs die Unsicherheit beeinflusst.
# Das ist für die Auslegung relevant, da die Unsicherheit der EL3182 Klemme
# prozentual auf diesen Endwert bezogen ist (Überdimensionierung = höherer absoluter Fehler).
# -------------------------------------------------------------------------
MASSFLOW_CHAIN_VARIANTS = [
    {
        "variant": "DN08_FS40",
        "label": "DN08, FS 40 g/s",
        "sensor_type": MASSFLOW_SENSOR_TYPE,
        "terminal_class": MASSFLOW_TERMINAL_CLASS,
        "full_scale": 40 / 1000,
    },
    {
        "variant": "DN08_FS60",
        "label": "DN08, FS 60 g/s",
        "sensor_type": MASSFLOW_SENSOR_TYPE,
        "terminal_class": MASSFLOW_TERMINAL_CLASS,
        "full_scale": 60 / 1000,
    },
    {
        "variant": "DN08_FS80",
        "label": "DN08, FS 80 g/s",
        "sensor_type": MASSFLOW_SENSOR_TYPE,
        "terminal_class": MASSFLOW_TERMINAL_CLASS,
        "full_scale": 80 / 1000,
    },
]

# Für den Massenstrommesskettenvergleich wird standardmäßig die 10-bar-Variante
# am Verdampfer verwendet, da diese die geplante genauere Auswertung darstellt.
MASSFLOW_COMPARISON_PRESSURE_VARIANT = "p10"

# Vergleichsvarianten am Verdampfer / Niederdruckseite
EVAP_PRESSURE_RANGE_VARIANTS = [
    {
        "variant": "p25",
        "label": "25 bar Messbereich",
        "pressure_sensor_type": "ICS_Schneider_IMP331-p25",
        "pressure_full_scale": 25 * 1e5,
    },
    {
        "variant": "p10",
        "label": "10 bar Messbereich",
        "pressure_sensor_type": "ICS_Schneider_IMP331-p10",
        "pressure_full_scale": 10 * 1e5,
    },
]

# Druckmessung vor dem EEV / Flüssigkeitsleitung (Hochdruck)
BEFORE_EEV_PRESSURE_SENSOR_TYPE = "ICS_Schneider_IMP331-p40"
BEFORE_EEV_PRESSURE_FULL_SCALE = 40 * 1e5


# =============================================================================
# 1) LABELS (Für Plotbeschriftungen und Exporte)
# =============================================================================

STATE_LABELS = {
    "m_ref": "m_ref",
    "T_out_evap": "T_out",
    "p_out_evap": "p_out",
    "T_before_EEV": "T_vor EEV",
    "p_before_EEV_40bar": "p_vor EEV\n40 bar",
}

VARIANT_LABELS = {
    "p25": "25 bar",
    "p10": "10 bar",
}

BP_LABELS = {
    "BP967_Kv max SC": "Kv max",
    "BP40_Kv min SC": "Kv min",
    "BP1007_max Q": "max Q",
    "BP568_min Q": "min Q",
    "BP551_Nenn": "Nenn",
}

STATE_ORDER = [
    "m_ref",
    "T_out_evap",
    "p_out_evap",
    "T_before_EEV",
    "p_before_EEV_40bar",
]


def short_bp_name(bp):
    return BP_LABELS.get(bp, bp.replace("BP", "").replace("_", " "))


def short_state_name(state):
    return STATE_LABELS.get(state, state)


def default_massflow_chain_variant():
    """Gibt die Standardkonfiguration der Massenstrommesskette zurück."""
    return {
        "variant": "base",
        "label": f"{MASSFLOW_SENSOR_TYPE}, FS {MASSFLOW_FULL_SCALE * 1000:.0f} g/s",
        "sensor_type": MASSFLOW_SENSOR_TYPE,
        "terminal_class": MASSFLOW_TERMINAL_CLASS,
        "full_scale": MASSFLOW_FULL_SCALE,
    }


def get_pressure_variant(variant_name):
    for variant in EVAP_PRESSURE_RANGE_VARIANTS:
        if variant["variant"] == variant_name:
            return variant
    raise ValueError(f"Unbekannte Drucksensorvariante: {variant_name}")


# =============================================================================
# 2) HELPER: EINZELNE UNSICHERHEITSQUELLEN AUSSCHALTEN
# =============================================================================

def zero_uncertainty(obj):
    """
    Entfernt die in Sensor/Klemme hinterlegten Unsicherheitsanteile.

    In der Leave-one-out-Methode wird immer nur eine Quelle ausgeschaltet.
    Alle anderen Quellen bleiben aktiv, um isolierte Varianzen zu berechnen.
    """
    if hasattr(obj, "a"):
        obj.a = []
    return obj


def flags_with_disabled_component(disabled_component=None):
    """Generiert Steuer-Flags zum gezielten Deaktivieren einer Messkette."""
    flags = {
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
    }

    if disabled_component is None:
        return flags

    if disabled_component not in flags:
        raise ValueError(f"Unbekannte Komponente: {disabled_component}")

    flags[disabled_component] = False
    return flags


# =============================================================================
# 3) MESSOBJEKTE (GUM-Implementierungen)
# =============================================================================

def create_temperature(T_K, active_sensor=True, active_terminal=True):
    """Erzeugt ein Temperatur-Messobjekt inkl. GUM Fehlerfortpflanzung."""
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
    """Erzeugt ein Druck-Messobjekt inkl. GUM Fehlerfortpflanzung."""
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
    active_terminal=True,
    massflow_chain_variant=None
):
    """
    Erzeugt ein Massenstrom-Messobjekt (Coriolis).
    Druck und Temperatur werden für das Sensormodell (Kompensation) übergeben.
    """
    values = [float(m_kg_s)]

    if massflow_chain_variant is None:
        massflow_chain_variant = default_massflow_chain_variant()

    sensor_type = massflow_chain_variant.get("sensor_type", MASSFLOW_SENSOR_TYPE)
    terminal_class = massflow_chain_variant.get("terminal_class", MASSFLOW_TERMINAL_CLASS)
    full_scale = massflow_chain_variant.get("full_scale", MASSFLOW_FULL_SCALE)

    sensor = Krohne_OptiMass(
        type=sensor_type,
        values=values,
        pressure=p_for_sensor.x,
        temperature=T_for_sensor.x
    )

    terminal = terminal_class(
        full_scale=full_scale
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
    Berechnet die Enthalpie über PyfluidsEnthalpy (GTC-kompatibel).
    Voraussetzung: PyfluidsEnthalpy.py verwendet intern FluidsList.nPropane.
    """
    return PyfluidsEnthalpy(
        temperature=T,
        pressure=p
    )()


# =============================================================================
# 4) EXCEL EINLESEN
# =============================================================================

def normalize_column_name(col):
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
    """Lädt die notwendigen Zustandsgrößen für die Verdampferbilanz."""
    df = pd.read_excel(excel_path)

    col_state = find_column(df, "State Number")
    col_desc = find_column(df, "Punktbeschreibung")

    col_m = find_column(df, "m_flow_ref in kg/s")

    col_T1 = find_column(df, "T_1 in K")
    col_p1 = find_column(df, "p_1 in Pa")

    col_T3 = find_column(df, "T_3 in K")
    col_p3 = find_column(df, "p_3 in Pa")

    col_T4 = find_optional_column(df, "T_4 in K")
    col_p4 = find_optional_column(df, "p_4 in Pa")

    col_H1 = find_optional_column(df, "H_1")
    col_H3 = find_optional_column(df, "H_3")
    col_H4 = find_optional_column(df, "H_4")

    # Optional: Falls in der Excel vorhanden (für Plausibilitätschecks).
    col_superheat = find_optional_column(df, "dT_eva_superheating in K")
    col_subcooling = find_optional_column(df, "dT_con_subcooling in K")

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

            "T_out_evap_K": float(row[col_T1]),
            "p_out_evap_Pa": float(row[col_p1]),

            "T_before_EEV_K": float(row[col_T3]),
            "p_before_EEV_Pa": float(row[col_p3]),

            "T_evap_in_K": optional_float(row, col_T4),
            "p_evap_in_Pa": optional_float(row, col_p4),

            "h_out_evap_sim_Jkg": optional_float(row, col_H1),
            "h_before_EEV_sim_Jkg": optional_float(row, col_H3),
            "h_evap_in_sim_Jkg": optional_float(row, col_H4),

            "dT_superheat_K": optional_float(row, col_superheat),
            "dT_subcooling_K": optional_float(row, col_subcooling),
        }

        points.append(point)

    return points


# =============================================================================
# 5) VERDAMPFERBILANZ (GUM)
# =============================================================================

def calculate_evaporator_balance(
    bp,
    evap_pressure_variant,
    disabled_component=None,
    expand=False,
    massflow_chain_variant=None
):
    """
    Berechnet die Kälteleistung des Verdampfers inkl. Fehlerfortpflanzung.
    Q_evap = m_ref * (h_out - h_before_EEV)
    """
    flags = flags_with_disabled_component(disabled_component)

    # -------------------------------------------------------------------------
    # Verdampferaustritt: Zustand 1
    # -------------------------------------------------------------------------
    T_out = create_temperature(
        bp["T_out_evap_K"],
        active_sensor=flags["T_out_sensor"],
        active_terminal=flags["T_out_terminal"]
    )

    p_out = create_pressure(
        bp["p_out_evap_Pa"],
        pressure_sensor_type=evap_pressure_variant["pressure_sensor_type"],
        pressure_full_scale=evap_pressure_variant["pressure_full_scale"],
        active_sensor=flags["p_out_sensor"],
        active_terminal=flags["p_out_terminal"]
    )

    h_out = enthalpy_refrigerant(
        p=p_out,
        T=T_out
    )

    # -------------------------------------------------------------------------
    # Vor EEV: Zustand 3
    #
    # Simulation ohne IHX: Unterkühlung = 0 K -> Sättigungslinie.
    # Um numerische Instabilitäten im Gradienten (GUM) zu vermeiden, wird
    # T_before_EEV künstlich um den definierten Shift abgesenkt.
    # -------------------------------------------------------------------------
    T_before_EEV_for_h_K = (
        bp["T_before_EEV_K"] - BEFORE_EEV_SUBCOOLING_SHIFT_K
    )

    T_before_EEV = create_temperature(
        T_before_EEV_for_h_K,
        active_sensor=flags["T_before_EEV_sensor"],
        active_terminal=flags["T_before_EEV_terminal"]
    )

    p_before_EEV = create_pressure(
        bp["p_before_EEV_Pa"],
        pressure_sensor_type=BEFORE_EEV_PRESSURE_SENSOR_TYPE,
        pressure_full_scale=BEFORE_EEV_PRESSURE_FULL_SCALE,
        active_sensor=flags["p_before_EEV_40bar_sensor"],
        active_terminal=flags["p_before_EEV_40bar_terminal"]
    )

    h_before_EEV = enthalpy_refrigerant(
        p=p_before_EEV,
        T=T_before_EEV
    )

    # Isenthalpe Drosselung:
    h_in_evap = h_before_EEV

    # -------------------------------------------------------------------------
    # Kältemittelmassenstrom
    # -------------------------------------------------------------------------
    m_ref = create_massflow(
        bp["m_ref_kg_s"],
        p_for_sensor=p_out,
        T_for_sensor=T_out,
        active_sensor=flags["m_sensor"],
        active_terminal=flags["m_terminal"],
        massflow_chain_variant=massflow_chain_variant
    )

    # -------------------------------------------------------------------------
    # Verdampferleistung (kW)
    # -------------------------------------------------------------------------
    Q_evap = (m_ref * (h_out - h_in_evap)) / 1000

    if expand:
        Q_evap = expand_uncertainty(
            Q_evap,
            k=K_EXPANSION
        )

    return {
        "Q_evap": Q_evap,

        "m_ref": m_ref,
        "massflow_chain_variant": massflow_chain_variant or default_massflow_chain_variant(),

        "T_out": T_out,
        "p_out": p_out,
        "h_out": h_out,

        "T_before_EEV": T_before_EEV,
        "p_before_EEV": p_before_EEV,
        "h_before_EEV": h_before_EEV,

        "T_before_EEV_original_K": bp["T_before_EEV_K"],
        "T_before_EEV_for_h_K": T_before_EEV_for_h_K,
        "before_EEV_subcooling_shift_K": BEFORE_EEV_SUBCOOLING_SHIFT_K,

        "h_in_evap": h_in_evap,
        "delta_h": h_out.x - h_in_evap.x,
    }


# =============================================================================
# 6) CONTRIBUTION ANALYSIS (VARIANZANALYSE)
# =============================================================================

def contribution_analysis(bp, evap_pressure_variant, massflow_chain_variant=None):
    """Ermittelt den Einfluss der einzelnen Sensoren/Klemmen auf Q_evap (Leave-one-out)."""
    full = calculate_evaporator_balance(
        bp=bp,
        evap_pressure_variant=evap_pressure_variant,
        disabled_component=None,
        expand=False,
        massflow_chain_variant=massflow_chain_variant
    )

    Q_full = full["Q_evap"]
    u_full = abs(Q_full.u)
    variance_full = u_full ** 2

    Q_expanded = expand_uncertainty(
        Q_full,
        k=K_EXPANSION
    )

    components = [
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
    ]

    contribution_rows = []

    for state_variable, source, disabled_component in components:
        without_i = calculate_evaporator_balance(
            bp=bp,
            evap_pressure_variant=evap_pressure_variant,
            disabled_component=disabled_component,
            expand=False,
            massflow_chain_variant=massflow_chain_variant
        )

        u_without_i = abs(without_i["Q_evap"].u)
        variance_without_i = u_without_i ** 2

        variance_i = variance_full - variance_without_i

        if variance_i < 0:
            variance_i = 0.0

        u_i = variance_i ** 0.5

        contribution_rows.append({
            "state": bp["state"],
            "description": bp["description"],
            "bp": bp["name"],

            "pressure_variant": evap_pressure_variant["variant"],
            "pressure_label": evap_pressure_variant["label"],

            "state_variable": state_variable,
            "state_variable_short": short_state_name(state_variable),
            "source": source,

            "u_i_kW": u_i,
            "variance_i": variance_i,

            "u_full_kW": u_full,
            "u_without_i_kW": u_without_i,
        })

    variance_sum = sum(row["variance_i"] for row in contribution_rows)

    for row in contribution_rows:
        if variance_sum > 0:
            row["share_percent"] = row["variance_i"] / variance_sum * 100
        else:
            row["share_percent"] = 0.0

    dominant = max(
        contribution_rows,
        key=lambda r: r["share_percent"]
    )

    result_row = {
        "state": bp["state"],
        "description": bp["description"],
        "bp": bp["name"],
        "bp_short": short_bp_name(bp["name"]),

        "pressure_variant": evap_pressure_variant["variant"],
        "pressure_label": evap_pressure_variant["label"],

        "Q_evap_kW": Q_full.x,
        "u_Q_evap_kW": Q_full.u,
        "U_Q_evap_kW": Q_expanded.u,

        "rel_u_percent": abs(Q_full.u / Q_full.x) * 100 if Q_full.x != 0 else np.nan,
        "rel_U_percent": abs(Q_expanded.u / Q_expanded.x) * 100 if Q_expanded.x != 0 else np.nan,

        "h_out_Jkg": full["h_out"].x,
        "u_h_out_Jkg": full["h_out"].u,

        "h_before_EEV_Jkg": full["h_before_EEV"].x,
        "u_h_before_EEV_Jkg": full["h_before_EEV"].u,

        "delta_h_Jkg": full["delta_h"],

        "m_ref_kg_s": full["m_ref"].x,
        "u_m_ref_kg_s": full["m_ref"].u,

        "massflow_chain_variant": full["massflow_chain_variant"]["variant"],
        "massflow_chain_label": full["massflow_chain_variant"]["label"],
        "massflow_full_scale_kg_s": full["massflow_chain_variant"]["full_scale"],

        "dominant_variable": dominant["state_variable"],
        "dominant_variable_short": short_state_name(dominant["state_variable"]),
        "dominant_source": dominant["source"],
        "dominant_share_percent": dominant["share_percent"],

        "variance_full": variance_full,
        "variance_sum_contributions": variance_sum,

        "h_out_evap_sim_Jkg": bp["h_out_evap_sim_Jkg"],
        "h_before_EEV_sim_Jkg": bp["h_before_EEV_sim_Jkg"],
        "h_evap_in_sim_Jkg": bp["h_evap_in_sim_Jkg"],

        "dT_superheat_K": bp["dT_superheat_K"],
        "dT_subcooling_K": bp["dT_subcooling_K"],

        "T_before_EEV_original_K": full["T_before_EEV_original_K"],
        "T_before_EEV_for_h_K": full["T_before_EEV_for_h_K"],
        "before_EEV_subcooling_shift_K": full["before_EEV_subcooling_shift_K"],
    }

    return result_row, contribution_rows


# =============================================================================
# 7) DEBUG-FUNKTIONEN (Plausibilitätsprüfung Skript vs. Simulation)
# =============================================================================

def export_debug_plausibility_table(results_df, output_dir):
    debug = results_df.copy()

    debug["delta_h_script_Jkg"] = debug["h_out_Jkg"] - debug["h_before_EEV_Jkg"]
    debug["delta_h_sim_Jkg"] = debug["h_out_evap_sim_Jkg"] - debug["h_before_EEV_sim_Jkg"]

    debug["Q_script_kW"] = debug["Q_evap_kW"]
    debug["Q_from_sim_h_kW"] = (
        debug["m_ref_kg_s"]
        * debug["delta_h_sim_Jkg"]
        / 1000
    )

    debug["h_out_diff_Jkg"] = debug["h_out_Jkg"] - debug["h_out_evap_sim_Jkg"]
    debug["h_before_EEV_diff_Jkg"] = debug["h_before_EEV_Jkg"] - debug["h_before_EEV_sim_Jkg"]

    debug["h_out_diff_percent"] = (
        debug["h_out_diff_Jkg"]
        / debug["h_out_evap_sim_Jkg"]
        * 100
    )

    debug["h_before_EEV_diff_percent"] = (
        debug["h_before_EEV_diff_Jkg"]
        / debug["h_before_EEV_sim_Jkg"]
        * 100
    )

    debug["Q_diff_kW"] = debug["Q_script_kW"] - debug["Q_from_sim_h_kW"]

    debug["Q_diff_percent"] = (
        debug["Q_diff_kW"]
        / debug["Q_from_sim_h_kW"]
        * 100
    )

    debug_columns = [
        "bp",
        "bp_short",
        "pressure_variant",

        "Q_script_kW",
        "Q_from_sim_h_kW",
        "Q_diff_kW",
        "Q_diff_percent",

        "h_out_Jkg",
        "h_out_evap_sim_Jkg",
        "h_out_diff_Jkg",
        "h_out_diff_percent",
        "u_h_out_Jkg",

        "h_before_EEV_Jkg",
        "h_before_EEV_sim_Jkg",
        "h_before_EEV_diff_Jkg",
        "h_before_EEV_diff_percent",
        "u_h_before_EEV_Jkg",

        "delta_h_script_Jkg",
        "delta_h_sim_Jkg",

        "m_ref_kg_s",
        "u_m_ref_kg_s",
        "massflow_chain_label",
        "massflow_full_scale_kg_s",

        "dT_superheat_K",
        "dT_subcooling_K",
        "T_before_EEV_original_K",
        "T_before_EEV_for_h_K",
        "before_EEV_subcooling_shift_K",

        "U_Q_evap_kW",
        "rel_U_percent",
    ]

    debug = debug[debug_columns]

    debug_file = output_dir / "debug_plausibilitaet_enthalpie.csv"
    debug.to_csv(debug_file, sep=";", index=False)

    print("\nDebug-Tabelle gespeichert:")
    print(debug_file)

    return debug


def print_debug_summary(result_row):
    """Kompakte Konsolenausgabe für Plausibilitätschecks."""
    Q_from_sim_h = (
        result_row["m_ref_kg_s"]
        * (
            result_row["h_out_evap_sim_Jkg"]
            - result_row["h_before_EEV_sim_Jkg"]
        )
        / 1000
    )

    print(
        f"h_out Skript = {result_row['h_out_Jkg']:.1f} J/kg | "
        f"h_out Sim = {result_row['h_out_evap_sim_Jkg']:.1f} J/kg"
    )

    print(
        f"h_vor_EEV Skript = {result_row['h_before_EEV_Jkg']:.1f} J/kg | "
        f"h_vor_EEV Sim = {result_row['h_before_EEV_sim_Jkg']:.1f} J/kg"
    )

    print(
        f"Q aus Skript-h = {result_row['Q_evap_kW']:.3f} kW | "
        f"Q aus Sim-h = {Q_from_sim_h:.3f} kW"
    )

    print(
        f"T_vor_EEV original = {result_row['T_before_EEV_original_K']:.3f} K | "
        f"T_vor_EEV für h(T,p) = {result_row['T_before_EEV_for_h_K']:.3f} K | "
        f"Shift = {result_row['before_EEV_subcooling_shift_K']:.3f} K"
    )

    if not pd.isna(result_row["dT_superheat_K"]):
        print(f"Überhitzung laut Simulation = {result_row['dT_superheat_K']:.3f} K")

    if not pd.isna(result_row["dT_subcooling_K"]):
        print(f"Unterkühlung laut Simulation = {result_row['dT_subcooling_K']:.3f} K")


def plot_debug_q_comparison(debug_df, output_dir):
    """Plot: Q_evap berechnet durch Skript-Enthalpien vs. Simulationsenthalpien."""
    df = debug_df.copy()
    df = df[df["pressure_variant"] == "p25"].copy()

    labels = df["bp_short"].tolist()
    x = np.arange(len(df))
    width = 0.36

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.bar(
        x - width / 2,
        df["Q_script_kW"],
        width,
        label="Q aus Skript-h"
    )

    ax.bar(
        x + width / 2,
        df["Q_from_sim_h_kW"],
        width,
        label="Q aus Sim-h"
    )

    ax.axhline(0, linewidth=1)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right")

    ax.set_ylabel("Verdampferleistung in kW")
    ax.set_title("Plausibilitätscheck: Skript-Enthalpien vs. Simulationsenthalpien")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()

    file = output_dir / "debug_Q_script_vs_Q_sim_h.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)


def plot_debug_enthalpy_difference(debug_df, output_dir):
    """Plot: Prozentuale Abweichungen der Enthalpien."""
    df = debug_df.copy()
    df = df[df["pressure_variant"] == "p25"].copy()

    labels = df["bp_short"].tolist()
    x = np.arange(len(df))
    width = 0.36

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.bar(
        x - width / 2,
        df["h_out_diff_percent"],
        width,
        label="h_out Abweichung"
    )

    ax.bar(
        x + width / 2,
        df["h_before_EEV_diff_percent"],
        width,
        label="h_vor EEV Abweichung"
    )

    ax.axhline(0, linewidth=1)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right")

    ax.set_ylabel("Abweichung von Simulation in %")
    ax.set_title("Plausibilitätscheck der Enthalpieberechnung")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()

    file = output_dir / "debug_h_script_vs_h_sim_percent.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)


# =============================================================================
# 8) PLOTS (Standardauswertung)
# =============================================================================

def plot_one_matrix_per_bp(contrib_df, results_df, output_dir):
    """Plot: Anteilige Messunsicherheit aufgeschlüsselt nach Sensoren und Klemmen."""
    for bp in contrib_df["bp"].drop_duplicates():
        fig, axes = plt.subplots(1, 2, figsize=(11, 5), sharey=True)

        for ax, variant in zip(axes, ["p25", "p10"]):
            df = contrib_df[
                (contrib_df["bp"] == bp)
                & (contrib_df["pressure_variant"] == variant)
            ].copy()

            rows = []

            for state in STATE_ORDER:
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
                    "Total": sensor_share + terminal_share,
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

            ax.set_title(VARIANT_LABELS.get(variant, variant))
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
                & (results_df["pressure_variant"] == variant)
            ]

            if not r.empty:
                Q = r["Q_evap_kW"].iloc[0]
                U = r["U_Q_evap_kW"].iloc[0]
                relU = r["rel_U_percent"].iloc[0]

                text = f"Q = {Q:.2f} kW\nU = ±{U:.2f} kW\nrel. U = {relU:.1f} %"

                ax.text(
                    0.02,
                    0.98,
                    text,
                    transform=ax.transAxes,
                    va="top",
                    ha="left",
                    bbox=dict(boxstyle="round", alpha=0.15)
                )

        axes[0].set_ylabel("Anteil an Gesamtvarianz von Q_evap in %")
        axes[0].legend(loc="upper right")

        fig.suptitle(f"Unsicherheitsbeiträge Verdampferbilanz: {short_bp_name(bp)}")

        plt.tight_layout()

        filename = f"01_matrix_{bp.replace(' ', '_').replace('/', '_')}.svg"
        file = output_dir / filename
        plt.savefig(file, dpi=300)
        plt.close(fig)


def plot_sensor_range_comparison(results_df, output_dir):
    """Plot: Gegenüberstellung der erweiterten Unsicherheit (25 bar vs 10 bar)."""
    p25 = results_df[results_df["pressure_variant"] == "p25"].set_index("bp")
    p10 = results_df[results_df["pressure_variant"] == "p10"].set_index("bp")

    common_bps = list(p25.index.intersection(p10.index))

    labels = [short_bp_name(bp) for bp in common_bps]

    U25_kW = np.array([p25.loc[bp, "U_Q_evap_kW"] for bp in common_bps])
    U10_kW = np.array([p10.loc[bp, "U_Q_evap_kW"] for bp in common_bps])

    U25_W = U25_kW * 1000
    U10_W = U10_kW * 1000

    relU25 = np.array([p25.loc[bp, "rel_U_percent"] for bp in common_bps])
    relU10 = np.array([p10.loc[bp, "rel_U_percent"] for bp in common_bps])

    improvement = (U25_kW - U10_kW) / U25_kW * 100

    x = np.arange(len(common_bps))
    width = 0.36

    fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)

    axes[0].bar(x - width / 2, U25_W, width, label="25 bar")
    axes[0].bar(x + width / 2, U10_W, width, label="10 bar")

    axes[0].set_ylabel("U(Q_evap) in W")
    axes[0].set_title("Erweiterte Unsicherheit der Verdampferbilanz")
    axes[0].legend()
    axes[0].grid(axis="y", alpha=0.3)

    axes[1].bar(x - width / 2, relU25, width, label="25 bar")
    axes[1].bar(x + width / 2, relU10, width, label="10 bar")

    axes[1].set_ylabel("rel. U(Q_evap) in %")
    axes[1].set_title("Relative erweiterte Unsicherheit bezogen auf Q_evap")
    axes[1].legend()
    axes[1].grid(axis="y", alpha=0.3)

    axes[2].bar(x, improvement)
    axes[2].axhline(0, linewidth=1)

    axes[2].set_ylabel("Reduktion durch 10 bar in %")
    axes[2].set_title("Verbesserung 10 bar gegenüber 25 bar")
    axes[2].grid(axis="y", alpha=0.3)

    axes[2].set_xticks(x)
    axes[2].set_xticklabels(labels, rotation=30, ha="right")

    for i, value in enumerate(improvement):
        axes[2].text(
            i,
            value,
            f"{value:.2f} %",
            ha="center",
            va="bottom" if value >= 0 else "top"
        )

    plt.tight_layout()

    file = output_dir / "02_vergleich_U_und_verbesserung.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)

    improvement_df = pd.DataFrame({
        "bp": common_bps,
        "bp_short": labels,
        "U_25_kW": U25_kW,
        "U_10_kW": U10_kW,
        "U_25_W": U25_W,
        "U_10_W": U10_W,
        "rel_U_25_percent": relU25,
        "rel_U_10_percent": relU10,
        "improvement_percent": improvement,
    })

    improvement_df.to_csv(
        output_dir / "verbesserung_10bar_gegen_25bar.csv",
        sep=";",
        index=False
    )

def plot_clean_heatmap(results_df, output_dir):
    """Plot: Heatmap zur kompakten Darstellung der relativen Messunsicherheiten."""
    df = results_df.copy()

    df["bp_short"] = df["bp"].map(short_bp_name)
    df["variant_short"] = df["pressure_variant"].map(VARIANT_LABELS)

    heat = df.pivot_table(
        index="bp_short",
        columns="variant_short",
        values="rel_U_percent",
        aggfunc="mean"
    )

    cols = [c for c in ["25 bar", "10 bar"] if c in heat.columns]
    heat = heat[cols]

    fig, ax = plt.subplots(figsize=(7, 5))

    im = ax.imshow(heat.values, aspect="auto")

    ax.set_xticks(np.arange(len(heat.columns)))
    ax.set_xticklabels(heat.columns)

    ax.set_yticks(np.arange(len(heat.index)))
    ax.set_yticklabels(heat.index)

    ax.set_xlabel("Messbereich Drucksensor am Verdampfer")
    ax.set_ylabel("Betriebspunkt")
    ax.set_title("Relative erweiterte Unsicherheit der Verdampferbilanz")

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label("rel. U(Q_evap) in %")

    for i in range(len(heat.index)):
        for j in range(len(heat.columns)):
            value = heat.values[i, j]
            ax.text(
                j,
                i,
                f"{value:.1f} %",
                ha="center",
                va="center"
            )

    plt.tight_layout()

    file = output_dir / "03_heatmap_rel_U_clean.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)


# =============================================================================
# MASSENSTROM-MESSKETTE: SENSITIVITÄTSANALYSE FÜR FULL-SCALE VARIANTEN
# =============================================================================

def run_massflow_chain_comparison(operating_points, output_dir):
    """
    Zusätzlicher Vergleich der Massenstrommesskette.

    Ziel:
    Bewertung, wie stark der gewählte Messbereichsendwert (Full-Scale) des
    Massenstromsignals die Unsicherheit der Verdampferbilanz beeinflusst.
    Da die Unsicherheit der EL3182 Klemme prozentual auf den Messbereichsendwert
    bezogen ist, führt eine Überdimensionierung des Sensors zu einem höheren
    absoluten Fehler.

    Hinweis:
    Diese Funktion ersetzt keine Recherche realer alternativer Sensoren.
    Sie ist zunächst eine Sensitivitätsanalyse der vorhandenen Messkette.
    """
    pressure_variant = get_pressure_variant(MASSFLOW_COMPARISON_PRESSURE_VARIANT)
    rows = []

    for chain in MASSFLOW_CHAIN_VARIANTS:
        for bp in operating_points:
            result = calculate_evaporator_balance(
                bp=bp,
                evap_pressure_variant=pressure_variant,
                disabled_component=None,
                expand=False,
                massflow_chain_variant=chain
            )

            Q = result["Q_evap"]
            Q_expanded = expand_uncertainty(Q, k=K_EXPANSION)

            rows.append({
                "bp": bp["name"],
                "bp_short": short_bp_name(bp["name"]),
                "pressure_variant": pressure_variant["variant"],
                "pressure_label": pressure_variant["label"],
                "massflow_chain_variant": chain["variant"],
                "massflow_chain_label": chain["label"],
                "massflow_full_scale_kg_s": chain["full_scale"],
                "massflow_full_scale_g_s": chain["full_scale"] * 1000,
                "m_ref_kg_s": result["m_ref"].x,
                "m_ref_g_s": result["m_ref"].x * 1000,
                "u_m_ref_kg_s": result["m_ref"].u,
                "u_m_ref_g_s": result["m_ref"].u * 1000,
                # Warn-Indikator, ob die reale Strömung den Sensor übersteigt
                "covers_operating_point": result["m_ref"].x <= chain["full_scale"],
                "Q_evap_kW": Q.x,
                "U_Q_evap_kW": Q_expanded.u,
                "U_Q_evap_W": Q_expanded.u * 1000,
                "rel_U_percent": abs(Q_expanded.u / Q_expanded.x) * 100 if Q_expanded.x != 0 else np.nan,
            })

    df = pd.DataFrame(rows)

    out_file = output_dir / "massenstrom_messkettenvergleich.csv"
    df.to_csv(out_file, sep=";", index=False)

    not_covered = df[~df["covers_operating_point"]]
    if not not_covered.empty:
        print("\nWARNUNG: Einige Massenstrom-Messbereichsendwerte decken Betriebspunkte nicht ab.")
        print(not_covered[["bp_short", "massflow_chain_label", "m_ref_g_s", "massflow_full_scale_g_s"]])

    plot_massflow_chain_comparison(df, output_dir)

    print("\nMassenstrommesskettenvergleich gespeichert:")
    print(out_file)

    return df


def plot_massflow_chain_comparison(df, output_dir):
    """
    Erstellt eine Grafik zum Vergleich der Massenstrommesskettenvarianten.
    Die Darstellung zeigt absolute Unsicherheit in W und relative Unsicherheit in %.
    """
    bp_order = list(df["bp_short"].drop_duplicates())
    chain_order = list(df["massflow_chain_label"].drop_duplicates())

    x = np.arange(len(bp_order))
    width = 0.8 / max(len(chain_order), 1)

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)

    for i, chain_label in enumerate(chain_order):
        sub = df[df["massflow_chain_label"] == chain_label].set_index("bp_short")
        values_W = np.array([sub.loc[bp, "U_Q_evap_W"] for bp in bp_order])
        values_rel = np.array([sub.loc[bp, "rel_U_percent"] for bp in bp_order])

        offset = (i - (len(chain_order) - 1) / 2) * width

        axes[0].bar(
            x + offset,
            values_W,
            width,
            label=chain_label
        )

        axes[1].bar(
            x + offset,
            values_rel,
            width,
            label=chain_label
        )

    axes[0].set_ylabel("U(Q_evap) in W")
    axes[0].set_title("Vergleich der Massenstrommesskette: absolute Unsicherheit")
    axes[0].grid(axis="y", alpha=0.3)
    axes[0].legend()

    axes[1].set_ylabel("rel. U(Q_evap) in %")
    axes[1].set_title("Vergleich der Massenstrommesskette: relative Unsicherheit")
    axes[1].grid(axis="y", alpha=0.3)
    axes[1].legend()

    axes[1].set_xticks(x)
    axes[1].set_xticklabels(bp_order, rotation=30, ha="right")

    fig.suptitle(
        f"Massenstrommesskettenvergleich bei {VARIANT_LABELS.get(MASSFLOW_COMPARISON_PRESSURE_VARIANT, MASSFLOW_COMPARISON_PRESSURE_VARIANT)}-Drucksensor am Verdampfer"
    )

    plt.tight_layout()

    file = output_dir / "04_massenstrom_messkettenvergleich.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)


def export_dominant_contributions(contrib_df, output_dir):
    """CSV-Export der maßgeblichen Fehlerquellen für die Evaluation."""
    rows = []

    grouped = contrib_df.groupby(["bp", "pressure_variant"])

    for (bp, variant), group in grouped:
        top = group.sort_values("share_percent", ascending=False).head(3)

        for rank, (_, row) in enumerate(top.iterrows(), start=1):
            rows.append({
                "bp": bp,
                "bp_short": short_bp_name(bp),
                "pressure_variant": variant,
                "pressure_variant_short": VARIANT_LABELS.get(variant, variant),
                "rank": rank,
                "state_variable": row["state_variable"],
                "state_variable_short": short_state_name(row["state_variable"]),
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
# 9) MAIN (Programmsteuerung)
# =============================================================================

def main():
    """Steuert den kompletten Ablauf der Sensitivitäts- und Unsicherheitsanalyse."""
    print("\nLese Betriebspunkte aus Excel...")
    operating_points = load_operating_points(EXCEL_PATH)

    print(f"Anzahl Betriebspunkte: {len(operating_points)}")
    for bp in operating_points:
        print(f"- {bp['name']}")

    print(
        "\nHinweis: Für h_before_EEV wird in dieser Variante "
        f"T_before_EEV - {BEFORE_EEV_SUBCOOLING_SHIFT_K} K verwendet."
    )

    result_rows = []
    contribution_rows_all = []

    for bp in operating_points:
        for variant in EVAP_PRESSURE_RANGE_VARIANTS:
            print("\n" + "=" * 100)
            print(f"Berechne {bp['name']} mit {variant['label']}")
            print("=" * 100)

            result_row, contribution_rows = contribution_analysis(
                bp=bp,
                evap_pressure_variant=variant
            )

            result_rows.append(result_row)
            contribution_rows_all.extend(contribution_rows)

            print(
                f"Q_evap = {result_row['Q_evap_kW']:.4f} kW "
                f"± {result_row['U_Q_evap_kW']:.4f} kW "
                f"(k={K_EXPANSION}, rel. U = {result_row['rel_U_percent']:.3f} %)"
            )

            print(
                f"Dominant: {result_row['dominant_variable_short']} "
                f"({result_row['dominant_source']}, "
                f"{result_row['dominant_share_percent']:.1f} %)"
            )

            print_debug_summary(result_row)

    results_df = pd.DataFrame(result_rows)
    contrib_df = pd.DataFrame(contribution_rows_all)

    results_file = OUTPUT_DIR / "ergebnisse_verdampferbilanz.csv"
    contrib_file = OUTPUT_DIR / "beitraege_unsicherheit_sensor_klemme.csv"

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

    print("\nErstelle Debug-Auswertung...")

    debug_df = export_debug_plausibility_table(
        results_df=results_df,
        output_dir=DEBUG_DIR
    )

    plot_debug_q_comparison(
        debug_df=debug_df,
        output_dir=DEBUG_DIR
    )

    plot_debug_enthalpy_difference(
        debug_df=debug_df,
        output_dir=DEBUG_DIR
    )

    print("\nErstelle überarbeitete Plots...")

    plot_one_matrix_per_bp(
        contrib_df=contrib_df,
        results_df=results_df,
        output_dir=PLOT_DIR
    )

    plot_sensor_range_comparison(
        results_df=results_df,
        output_dir=PLOT_DIR
    )

    plot_clean_heatmap(
        results_df=results_df,
        output_dir=PLOT_DIR
    )

    export_dominant_contributions(
        contrib_df=contrib_df,
        output_dir=PLOT_DIR
    )

    print("\nErstelle zusätzlichen Massenstrommesskettenvergleich...")

    run_massflow_chain_comparison(
        operating_points=operating_points,
        output_dir=PLOT_DIR
    )

    print("\nFertig.")
    print("CSV-Ausgabeordner:")
    print(OUTPUT_DIR)
    print("Plot-Ausgabeordner:")
    print(PLOT_DIR)
    print("Debug-Ausgabeordner:")
    print(DEBUG_DIR)


if __name__ == "__main__":
    main()