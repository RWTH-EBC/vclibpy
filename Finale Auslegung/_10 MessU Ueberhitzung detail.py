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

from gum_ebc.Terminals.EL3154 import EL3154
from gum_ebc.Terminals.EL3202_0010 import EL3202_0010


# =============================================================================
# 0) USER SETTINGS
# =============================================================================

# Pfad zu den thermodynamischen Betriebspunkten (Simulationsergebnisse)
EXCEL_PATH = r"C:\Users\mbc-asz\PycharmProjects\vclibpy\Finale Simulation\mbc-asz Simulation\pythonProject\Finale Auslegung\Auswahl_Punkte_Vergleich.xlsx"

# Ausgabeverzeichnisse
OUTPUT_DIR = Path(
    r"C:\Users\mbc-asz\PycharmProjects\vclibpy\Finale Simulation\mbc-asz Simulation\pythonProject\Finale Auslegung\Messunsicherheit_Ueberhitzung_Vergleich_svg"
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
# Für die Berechnung der Sättigungstemperatur wird pyfluids verwendet.
# R290 entspricht FluidsList.nPropane.
# -------------------------------------------------------------------------

# Temperaturmesskette
TEMP_SENSOR_TYPE = "PT100A"
TEMP_TERMINAL_CLASS = EL3202_0010

# Druckmesskette
PRESSURE_TERMINAL_CLASS = EL3154

# Vergleichsvarianten am Verdampfer / Niederdruckseite zur Untersuchung
# des Einflusses der Sensor-Messbereiche auf die Gesamtunsicherheit
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


# =============================================================================
# 1) LABELS
# =============================================================================

STATE_LABELS = {
    "T_out_evap": "T_out",
    "p_out_evap": "p_out",
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
    "T_out_evap",
    "p_out_evap",
]


def short_bp_name(bp):
    """Kürzt den Namen des Betriebspunkts für Plot-Beschriftungen."""
    return BP_LABELS.get(bp, bp.replace("BP", "").replace("_", " "))


def short_state_name(state):
    """Gibt eine lesbarere Bezeichnung für den Zustand zurück."""
    return STATE_LABELS.get(state, state)


def get_pressure_variant(variant_name):
    """Sucht die Metadaten zur gewünschten Drucksensor-Variante heraus."""
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
    Alle anderen Quellen bleiben aktiv. So lässt sich die Sensitivität (der Beitrag
    zur Gesamtvarianz) jeder einzelnen Komponente bestimmen.
    """
    if hasattr(obj, "a"):
        obj.a = []
    return obj


def flags_with_disabled_component(disabled_component=None):
    """
    Gibt ein Dictionary (Flags) zurück, in dem exakt eine Messkomponente
    auf 'False' (ohne Unsicherheit) gesetzt ist.
    """
    flags = {
        "T_out_sensor": True,
        "T_out_terminal": True,

        "p_out_sensor": True,
        "p_out_terminal": True,
    }

    if disabled_component is None:
        return flags

    if disabled_component not in flags:
        raise ValueError(f"Unbekannte Komponente: {disabled_component}")

    flags[disabled_component] = False
    return flags


# =============================================================================
# 3) MESSOBJEKTE
# =============================================================================

def create_temperature(T_K, active_sensor=True, active_terminal=True):
    """
    Erstellt ein GUM-Messobjekt für die Temperaturmesskette inkl.
    statistischer Unsicherheiten von Sensor und Klemme.
    """
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
    """
    Erstellt ein GUM-Messobjekt für die Druckmesskette (Sensor + Analogklemme).
    """
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


def saturation_temperature_nominal_K(p_Pa):
    """
    Sättigungstemperatur von R290 (Propan) bei gegebenem Druck.

    Für reines Propan ist die Sättigungstemperatur bei gegebener Qualität
    eindeutig. Falls die pyfluids-Version die Temperatur bereits in K
    zurückgibt, muss die +273.15-Anpassung entfernt werden.
    """
    try:
        fluid = Fluid(FluidsList.nPropane).with_state(
            Input.pressure(float(p_Pa)),
            Input.quality(1.0)
        )
        return float(fluid.temperature) + 273.15
    except Exception:
        raise RuntimeError(
            "Sättigungstemperatur konnte nicht mit pyfluids berechnet werden. "
            "Bitte die Funktion saturation_temperature_nominal_K an deine "
            "pyfluids-Version anpassen."
        )


def saturation_temperature_from_pressure(p_measurement):
    """
    Erzeugt eine ureal-Größe (GTC) für T_sat(p) auf Basis der Druckunsicherheit.

    Vorgehen (numerische Sensitivitätsanalyse zur Fehlerfortpflanzung):
    - T_sat wird beim Nominaldruck p0 berechnet.
    - Danach wird T_sat bei p0 + u(p) und p0 - u(p) berechnet.
    - Daraus wird numerisch (Differenzenquotient) die Standardunsicherheit
      von T_sat angenähert.
    """
    p0 = float(p_measurement.x)
    up = abs(float(p_measurement.u))

    T0 = saturation_temperature_nominal_K(p0)

    if up == 0:
        return ureal(T0, 0.0)

    # Absicherung: Druck darf für Stoffwertroutinen physikalisch nicht < 0 fallen
    p_plus = max(p0 + up, 1.0)
    p_minus = max(p0 - up, 1.0)

    T_plus = saturation_temperature_nominal_K(p_plus)
    T_minus = saturation_temperature_nominal_K(p_minus)

    # Lineare Approximation der Unsicherheit
    u_Tsat = abs(T_plus - T_minus) / 2.0

    return ureal(T0, u_Tsat)


# =============================================================================
# 4) EXCEL EINLESEN
# =============================================================================

def normalize_column_name(col):
    """Entfernt Zeilenumbrüche und Leerzeichen aus Spaltenüberschriften."""
    return str(col).strip().replace("\n", " ")


def find_column(df, startswith_text):
    """Findet eine notwendige Spalte anhand des Präfixes."""
    for col in df.columns:
        col_norm = normalize_column_name(col)
        if col_norm.startswith(startswith_text):
            return col

    raise KeyError(
        f"Spalte nicht gefunden. Gesucht wurde Anfang: '{startswith_text}'\n"
        f"Vorhandene Spalten:\n{list(df.columns)}"
    )


def find_optional_column(df, startswith_text):
    """Findet eine optionale Spalte anhand des Präfixes, andernfalls None."""
    for col in df.columns:
        col_norm = normalize_column_name(col)
        if col_norm.startswith(startswith_text):
            return col
    return None


def optional_float(row, col):
    """Liest einen float-Wert aus, falls die Spalte existiert, sonst NaN."""
    if col is None:
        return np.nan

    value = row[col]

    if pd.isna(value):
        return np.nan

    return float(value)


def load_operating_points(excel_path):
    """
    Lädt die relevanten Betriebspunkte aus der Excel-Tabelle und formatiert
    sie für die GUM-Auswertung.
    """
    df = pd.read_excel(excel_path)

    col_state = find_column(df, "State Number")
    col_desc = find_column(df, "Punktbeschreibung")

    col_T1 = find_column(df, "T_1 in K")
    col_p1 = find_column(df, "p_1 in Pa")

    # Optional: Simulationswert zum Plausibilitätscheck (Skript vs. Sim)
    col_superheat = find_optional_column(df, "dT_eva_superheating in K")

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

            # Zustand 1: Verdampferaustritt / vor Verdichter
            "T_out_evap_K": float(row[col_T1]),
            "p_out_evap_Pa": float(row[col_p1]),

            "dT_superheat_sim_K": optional_float(row, col_superheat),
        }

        points.append(point)

    return points


# =============================================================================
# 5) ÜBERHITZUNG
# =============================================================================

def calculate_superheat(
    bp,
    evap_pressure_variant,
    disabled_component=None,
    expand=False
):
    """
    Hauptfunktion zur Berechnung der thermodynamischen Überhitzung (Superheat)
    inklusive der GUM-Fehlerfortpflanzung durch alle beteiligten Messketten.
    """
    flags = flags_with_disabled_component(disabled_component)

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

    # Bestimmung der T_sat basierend auf p_out (mit p_out Fehlerfortpflanzung)
    T_sat_out = saturation_temperature_from_pressure(p_out)

    # Definition der Überhitzung:
    # ΔT_ÜH = T_out,evap - T_sat(p_out,evap)
    dT_superheat = T_out - T_sat_out

    if expand:
        dT_superheat = expand_uncertainty(
            dT_superheat,
            k=K_EXPANSION
        )

    return {
        "dT_superheat": dT_superheat,

        "T_out": T_out,
        "p_out": p_out,
        "T_sat_out": T_sat_out,
    }


# =============================================================================
# 6) CONTRIBUTION ANALYSIS
# =============================================================================

def contribution_analysis(bp, evap_pressure_variant):
    """
    Berechnet, wie stark jede einzelne Komponente (z.B. Drucksensor, Klemme)
    zur gesamten Messunsicherheit (Varianz) des Ergebnisses beiträgt.
    Nutzt dafür das Leave-one-out Verfahren.
    """
    # 1. Gesamte Systemvarianz berechnen (Alle Komponenten aktiv)
    full = calculate_superheat(
        bp=bp,
        evap_pressure_variant=evap_pressure_variant,
        disabled_component=None,
        expand=False
    )

    dT_full = full["dT_superheat"]
    u_full = abs(dT_full.u)
    variance_full = u_full ** 2

    dT_expanded = expand_uncertainty(
        dT_full,
        k=K_EXPANSION
    )

    components = [
        ("T_out_evap", "Sensor", "T_out_sensor"),
        ("T_out_evap", "Klemme", "T_out_terminal"),

        ("p_out_evap", "Sensor", "p_out_sensor"),
        ("p_out_evap", "Klemme", "p_out_terminal"),
    ]

    contribution_rows = []

    # 2. Iterativ jede Komponente isoliert ausschalten
    for state_variable, source, disabled_component in components:
        without_i = calculate_superheat(
            bp=bp,
            evap_pressure_variant=evap_pressure_variant,
            disabled_component=disabled_component,
            expand=False
        )

        u_without_i = abs(without_i["dT_superheat"].u)
        variance_without_i = u_without_i ** 2

        # 3. Differenz bildet den isolierten Beitrag der ausgeschalteten Komponente
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

            "u_i_K": u_i,
            "variance_i": variance_i,

            "u_full_K": u_full,
            "u_without_i_K": u_without_i,
        })

    # 4. Prozentuale Gewichtung (Dominanz)
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

    # 5. Zusammenfassung des Ergebnisses für diesen Betriebspunkt
    result_row = {
        "state": bp["state"],
        "description": bp["description"],
        "bp": bp["name"],
        "bp_short": short_bp_name(bp["name"]),

        "pressure_variant": evap_pressure_variant["variant"],
        "pressure_label": evap_pressure_variant["label"],

        "dT_superheat_K": dT_full.x,
        "u_dT_superheat_K": dT_full.u,
        "U_dT_superheat_K": dT_expanded.u,

        "rel_u_percent": abs(dT_full.u / dT_full.x) * 100 if dT_full.x != 0 else np.nan,
        "rel_U_percent": abs(dT_expanded.u / dT_expanded.x) * 100 if dT_expanded.x != 0 else np.nan,

        "T_out_evap_K": full["T_out"].x,
        "u_T_out_evap_K": full["T_out"].u,

        "p_out_evap_Pa": full["p_out"].x,
        "u_p_out_evap_Pa": full["p_out"].u,

        "T_sat_out_K": full["T_sat_out"].x,
        "u_T_sat_out_K": full["T_sat_out"].u,

        "dominant_variable": dominant["state_variable"],
        "dominant_variable_short": short_state_name(dominant["state_variable"]),
        "dominant_source": dominant["source"],
        "dominant_share_percent": dominant["share_percent"],

        "variance_full": variance_full,
        "variance_sum_contributions": variance_sum,

        "dT_superheat_sim_K": bp["dT_superheat_sim_K"],
    }

    return result_row, contribution_rows


# =============================================================================
# 7) DEBUG-FUNKTIONEN
# =============================================================================

def export_debug_superheat_table(results_df, output_dir):
    """Exportiert Tabellen zur Validierung der GUM Ergebnisse vs. Simulation."""
    debug = results_df.copy()

    debug["dT_diff_K"] = (
        debug["dT_superheat_K"]
        - debug["dT_superheat_sim_K"]
    )

    debug["dT_diff_percent"] = (
        debug["dT_diff_K"]
        / debug["dT_superheat_sim_K"]
        * 100
    )

    debug_columns = [
        "bp",
        "bp_short",
        "pressure_variant",

        "dT_superheat_K",
        "dT_superheat_sim_K",
        "dT_diff_K",
        "dT_diff_percent",

        "T_out_evap_K",
        "u_T_out_evap_K",

        "p_out_evap_Pa",
        "u_p_out_evap_Pa",

        "T_sat_out_K",
        "u_T_sat_out_K",

        "U_dT_superheat_K",
        "rel_U_percent",
    ]

    debug = debug[debug_columns]

    debug_file = output_dir / "debug_plausibilitaet_ueberhitzung.csv"
    debug.to_csv(debug_file, sep=";", index=False)

    print("\nDebug-Tabelle gespeichert:")
    print(debug_file)

    return debug


def plot_debug_superheat_comparison(debug_df, output_dir):
    """Bar-Chart für den optischen Vergleich Überhitzung (Skript) vs. Simulation."""
    df = debug_df.copy()
    df = df[df["pressure_variant"] == "p10"].copy()

    labels = df["bp_short"].tolist()
    x = np.arange(len(df))
    width = 0.36

    fig, ax = plt.subplots(figsize=(10, 5))

    ax.bar(
        x - width / 2,
        df["dT_superheat_K"],
        width,
        label="Überhitzung aus Skript"
    )

    ax.bar(
        x + width / 2,
        df["dT_superheat_sim_K"],
        width,
        label="Überhitzung aus Simulation"
    )

    ax.axhline(0, linewidth=1)

    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right")

    ax.set_ylabel("Überhitzung in K")
    ax.set_title("Plausibilitätscheck: berechnete Überhitzung vs. Simulation")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()

    file = output_dir / "debug_ueberhitzung_script_vs_sim.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)


# =============================================================================
# 8) PLOTS
# =============================================================================

def plot_one_matrix_per_bp(contrib_df, results_df, output_dir):
    """
    Plottet für jeden Betriebspunkt die prozentualen Unsicherheitsbeiträge
    aufgeteilt nach 25 bar und 10 bar Drucksensor.
    """
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
                dT = r["dT_superheat_K"].iloc[0]
                U = r["U_dT_superheat_K"].iloc[0]
                relU = r["rel_U_percent"].iloc[0]

                text = f"ΔT = {dT:.2f} K\nU = ±{U:.2f} K\nrel. U = {relU:.1f} %"

                ax.text(
                    0.02,
                    0.98,
                    text,
                    transform=ax.transAxes,
                    va="top",
                    ha="left",
                    bbox=dict(boxstyle="round", alpha=0.15)
                )

        axes[0].set_ylabel("Anteil an Gesamtvarianz der Überhitzung in %")
        axes[0].legend(loc="upper right")

        fig.suptitle(f"Unsicherheitsbeiträge Überhitzung: {short_bp_name(bp)}")

        plt.tight_layout()

        filename = f"01_matrix_SH_{bp.replace(' ', '_').replace('/', '_')}.svg"
        file = output_dir / filename
        plt.savefig(file, dpi=300)
        plt.close(fig)


def plot_sensor_range_comparison(results_df, output_dir):
    """
    Visualisiert direkt den Performance-Gewinn (Reduktion der Unsicherheit)
    wenn ein 10 bar anstelle eines 25 bar Sensors genutzt wird.
    """
    p25 = results_df[results_df["pressure_variant"] == "p25"].set_index("bp")
    p10 = results_df[results_df["pressure_variant"] == "p10"].set_index("bp")

    common_bps = list(p25.index.intersection(p10.index))

    labels = [short_bp_name(bp) for bp in common_bps]

    U25_K = np.array([p25.loc[bp, "U_dT_superheat_K"] for bp in common_bps])
    U10_K = np.array([p10.loc[bp, "U_dT_superheat_K"] for bp in common_bps])

    relU25 = np.array([p25.loc[bp, "rel_U_percent"] for bp in common_bps])
    relU10 = np.array([p10.loc[bp, "rel_U_percent"] for bp in common_bps])

    improvement = (U25_K - U10_K) / U25_K * 100

    x = np.arange(len(common_bps))
    width = 0.36

    fig, axes = plt.subplots(3, 1, figsize=(10, 11), sharex=True)

    axes[0].bar(x - width / 2, U25_K, width, label="25 bar")
    axes[0].bar(x + width / 2, U10_K, width, label="10 bar")

    axes[0].set_ylabel("U(ΔT_ÜH) in K")
    axes[0].set_title("Erweiterte Unsicherheit der Überhitzung")
    axes[0].legend()
    axes[0].grid(axis="y", alpha=0.3)

    axes[1].bar(x - width / 2, relU25, width, label="25 bar")
    axes[1].bar(x + width / 2, relU10, width, label="10 bar")

    axes[1].set_ylabel("rel. U(ΔT_ÜH) in %")
    axes[1].set_title("Relative erweiterte Unsicherheit bezogen auf die Überhitzung")
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

    file = output_dir / "02_vergleich_U_und_verbesserung_ueberhitzung.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)

    improvement_df = pd.DataFrame({
        "bp": common_bps,
        "bp_short": labels,
        "U_25_K": U25_K,
        "U_10_K": U10_K,
        "rel_U_25_percent": relU25,
        "rel_U_10_percent": relU10,
        "improvement_percent": improvement,
    })

    improvement_df.to_csv(
        output_dir / "verbesserung_10bar_gegen_25bar_ueberhitzung.csv",
        sep=";",
        index=False
    )


def plot_clean_heatmap(results_df, output_dir):
    """Erstellt eine Heatmap zur kompakten Darstellung der relativen Unsicherheit."""
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
    ax.set_title("Relative erweiterte Unsicherheit der Überhitzung")

    cbar = plt.colorbar(im, ax=ax)
    cbar.set_label("rel. U(ΔT_ÜH) in %")

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

    file = output_dir / "03_heatmap_rel_U_ueberhitzung.svg"
    plt.savefig(file, dpi=300)
    plt.close(fig)


def export_dominant_contributions(contrib_df, output_dir):
    """Schreibt die Top-3 Fehlerquellen jedes Betriebspunkts in eine CSV."""
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
        output_dir / "dominante_unsicherheitsquellen_top3_ueberhitzung.csv",
        sep=";",
        index=False
    )


# =============================================================================
# 9) MAIN (Programmablauf)
# =============================================================================

def main():
    """Führt die GUM-Analyse durch, exportiert Tabellen und erzeugt Plots."""
    print("\nLese Betriebspunkte aus Excel...")
    operating_points = load_operating_points(EXCEL_PATH)

    print(f"Anzahl Betriebspunkte: {len(operating_points)}")
    for bp in operating_points:
        print(f"- {bp['name']}")

    result_rows = []
    contribution_rows_all = []

    for bp in operating_points:
        for variant in EVAP_PRESSURE_RANGE_VARIANTS:
            print("\n" + "=" * 100)
            print(f"Berechne Überhitzung für {bp['name']} mit {variant['label']}")
            print("=" * 100)

            result_row, contribution_rows = contribution_analysis(
                bp=bp,
                evap_pressure_variant=variant
            )

            result_rows.append(result_row)
            contribution_rows_all.extend(contribution_rows)

            print(
                f"ΔT_ÜH = {result_row['dT_superheat_K']:.4f} K "
                f"± {result_row['U_dT_superheat_K']:.4f} K "
                f"(k={K_EXPANSION}, rel. U = {result_row['rel_U_percent']:.3f} %)"
            )

            print(
                f"Dominant: {result_row['dominant_variable_short']} "
                f"({result_row['dominant_source']}, "
                f"{result_row['dominant_share_percent']:.1f} %)"
            )

            if not pd.isna(result_row["dT_superheat_sim_K"]):
                print(
                    f"Überhitzung Simulation = {result_row['dT_superheat_sim_K']:.4f} K | "
                    f"Skript = {result_row['dT_superheat_K']:.4f} K"
                )

    results_df = pd.DataFrame(result_rows)
    contrib_df = pd.DataFrame(contribution_rows_all)

    results_file = OUTPUT_DIR / "ergebnisse_ueberhitzung.csv"
    contrib_file = OUTPUT_DIR / "beitraege_unsicherheit_sensor_klemme_ueberhitzung.csv"

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

    debug_df = export_debug_superheat_table(
        results_df=results_df,
        output_dir=DEBUG_DIR
    )

    plot_debug_superheat_comparison(
        debug_df=debug_df,
        output_dir=DEBUG_DIR
    )

    print("\nErstelle Plots...")

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

    print("\nFertig.")
    print("CSV-Ausgabeordner:")
    print(OUTPUT_DIR)
    print("Plot-Ausgabeordner:")
    print(PLOT_DIR)
    print("Debug-Ausgabeordner:")
    print(DEBUG_DIR)


if __name__ == "__main__":
    main()