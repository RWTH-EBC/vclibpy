#!/usr/bin/env python3
"""
test_liu_winterton_htc.py

Test script for Liu & Winterton (1991) two-phase flow boiling heat transfer correlation.

Features:
- Calculates HTC vs vapor quality using Liu & Winterton (1991) model
- Optional comparison with Gungor & Winterton (1986) and Kandlikar (1990) models
- Validates against experimental data from CSV files
- Calculates MAE and MAPE error metrics
- Generates publication-quality plots

Reference:
    Liu, Z. & Winterton, R.H.S. (1991). "A General Correlation for Saturated
    and Subcooled Flow Boiling in Tubes and Annuli, Based on a Nucleate Pool
    Boiling Equation." Int. J. Heat Mass Transfer, Vol. 34, No. 11, pp. 2759-2766.
"""

import math
import os
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

# =============================================================================
# Configuration
# =============================================================================
SAVE_DIR = "/Users/wooseok/Desktop/Abbildungen"
os.makedirs(SAVE_DIR, exist_ok=True)

# =============================================================================
# Experimental data paths
# =============================================================================
# Format: (label, csv_path, marker_style)
EXPERIMENT_DATASETS = [
    ("Wang2014",
     "/Users/wooseok/Desktop/Abbildungen/plotdigitizer/CSV_aus_Paper/Greco_2005_R22_with_ID_6mm/R22/p_sat4_45_q18_4_G363.csv",
     "o"),
]

# =============================================================================
# Output control flags
# =============================================================================
SHOW_FIG = True  # Show figure in PyCharm/IDE
SAVE_SVG = False  # Save as SVG file
SAVE_PNG = False  # Save as PNG file
EXPORT_TIKZ = False  # Export as TikZ (LaTeX)
EXPORT_PGF = False  # Export as PGF (LaTeX)

# =============================================================================
# Operating conditions
# =============================================================================
FLUID = "R22"  # Or "Propane", "R22", "R134a", etc.
USE_PSAT = True
T_SAT_C_OR_K = -35.0  # Saturation temperature in degC (if USE_PSAT=False)
P_SAT_VALUE = 4.45  # Saturation pressure
P_SAT_UNIT = "bar"  # bar, Pa, kPa, mPa

G = 363  # Mass flux in kg/m2s
Q_FLUX = 18.4  # Heat flux in kW/m2
D_I = 6.0  # Inner diameter in mm

X_MIN, X_MAX, N_X = 0.1, 0.99, 41  # Vapor quality range

# =============================================================================
# Model comparison options
# =============================================================================
# Select which models to plot
COMPARE_WITH_GW = True  # Compare with Gungor-Winterton (1986)
COMPARE_WITH_KANDLIKAR = True  # Compare with Kandlikar (1990)

# Kandlikar model parameters (only used if COMPARE_WITH_KANDLIKAR = True)
KANDLIKAR_FLUID_NAME = "R290"  # For F_fl lookup
KANDLIKAR_F_FL_VALUE = None  # Or explicit value, e.g., 1.5

# =============================================================================
# VcLibPy imports
# =============================================================================
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.heat_transfer.liu_winterton_htc import (
    LiuWintertonTwoPhase
    )

if COMPARE_WITH_GW:
    from vclibpy.components.heat_exchangers.heat_transfer.gw1986_htc import (
        GungorWintertonTwoPhase86
    )

if COMPARE_WITH_KANDLIKAR:
    from vclibpy.components.heat_exchangers.heat_transfer.kandlikar_htc import (
        KandlikarTwoPhase
    )

from vclibpy.media import CoolProp, set_global_media_properties

set_global_media_properties(CoolProp, use_high_level_api=True)


# =============================================================================
# Helper functions
# =============================================================================
def slugify(s: str) -> str:
    """
    Convert string to filesystem-safe format.

    Args:
        s: Input string

    Returns:
        Sanitized string safe for filenames
    """
    s = re.sub(r"[^\w\-,.]+", "-", s, flags=re.ASCII)
    return re.sub(r"-{2,}", "-", s).strip("-._")


def format_number(val: float, digits: int = 1) -> str:
    """
    Format number with comma as decimal separator.

    Args:
        val: Number to format
        digits: Number of decimal places

    Returns:
        Formatted string (e.g., 71.5 -> "71,5")
    """
    return f"{val:.{digits}f}".replace(".", ",")


def unique_path(base_dir: str, stem: str, ext: str) -> str:
    """
    Generate unique file path by appending counter if file exists.

    Args:
        base_dir: Directory path
        stem: Base filename without extension
        ext: File extension

    Returns:
        Unique file path
    """
    path = os.path.join(base_dir, f"{stem}.{ext}")
    if not os.path.exists(path):
        return path
    k = 1
    while True:
        alt = os.path.join(base_dir, f"{stem}-{k}.{ext}")
        if not os.path.exists(alt):
            return alt
        k += 1


def to_kelvin(T: float) -> float:
    """
    Convert Celsius to Kelvin if needed.

    Args:
        T: Temperature in degC or K

    Returns:
        Temperature in K
    """
    return T + 273.15 if -150.0 < T < 200.0 else T


def pressure_to_pascal(value: float, unit: str) -> float:
    """
    Convert pressure to Pascal.

    Args:
        value: Pressure value
        unit: Unit string (bar, kPa, MPa, Pa)

    Returns:
        Pressure in Pascal
    """
    u = unit.lower()
    conversion_factors = {
        "pa": 1.0,
        "kpa": 1e3,
        "mpa": 1e6,
        "bar": 1e5
    }
    return value * conversion_factors[u]


def get_saturation_states(med, use_psat, T_sat_input, P_value, P_unit):
    """
    Calculate saturation states for given conditions.

    Args:
        med: CoolProp media object
        use_psat: If True, use pressure; if False, use temperature
        T_sat_input: Saturation temperature in degC
        P_value: Saturation pressure value
        P_unit: Pressure unit

    Returns:
        Tuple of (saturated_liquid_state, saturated_vapor_state, pressure_Pa)
    """
    if use_psat:
        p_sat = pressure_to_pascal(P_value, P_unit)
        st_liq = med.calc_state("PQ", p_sat, 0.0)
        st_vap = med.calc_state("PQ", p_sat, 1.0)
        return st_liq, st_vap, p_sat
    else:
        T_sat = to_kelvin(T_sat_input)
        st_liq = med.calc_state("TQ", T_sat, 0.0)
        st_vap = med.calc_state("TQ", T_sat, 1.0)
        return st_liq, st_vap, st_liq.p


def load_csv_experimental_data(path: str):
    """
    Load experimental data from CSV file with flexible formatting.

    Attempts to parse CSV with various separator and decimal combinations.

    Args:
        path: Path to CSV file

    Returns:
        Tuple of (x_values, htc_values) as numpy arrays

    Raises:
        ValueError: If CSV parsing fails
    """
    import pandas as pd

    # Try different separator and decimal combinations
    parse_attempts = [
        (";", ","),  # European format
        (",", "."),  # US format
        ("\t", ","),  # Tab-separated with comma decimal
        (r"\s+", ",")  # Whitespace-separated
    ]

    for sep, dec in parse_attempts:
        try:
            df = pd.read_csv(
                path,
                header=None,
                sep=sep,
                decimal=dec,
                engine="python"
            )
            if df.shape[1] >= 2:
                x = pd.to_numeric(df.iloc[:, 0], errors="coerce").to_numpy(float)
                y = pd.to_numeric(df.iloc[:, 1], errors="coerce").to_numpy(float)
                mask = np.isfinite(x) & np.isfinite(y)
                if mask.any():
                    print(f"[CSV] Loaded {os.path.basename(path)}: N={mask.sum()} points")
                    return x[mask], y[mask]
        except Exception:
            pass

    raise ValueError(f"Failed to parse CSV file: {path}")


# =============================================================================
# Main execution
# =============================================================================
if __name__ == "__main__":
    # Convert units to SI
    q_flux_si = Q_FLUX * 1000.0  # kW/m2 -> W/m2
    d_i_si = D_I / 1000.0  # mm -> m

    # Initialize media
    med = CoolProp(FLUID, use_high_level_api=True)
    st_liq, st_vap, p_sat = get_saturation_states(
        med, USE_PSAT, T_SAT_C_OR_K, P_SAT_VALUE, P_SAT_UNIT
    )

    print(f"\n{'=' * 70}")
    print(f"Operating Conditions")
    print(f"{'=' * 70}")
    print(f"Fluid: {FLUID}")
    print(f"Saturation pressure: {p_sat / 1e5:.2f} bar")
    print(f"Saturation temperature: {st_liq.T - 273.15:.2f} degC")
    print(f"Mass flux: {G} kg/m2s")
    print(f"Heat flux: {Q_FLUX} kW/m2")
    print(f"Inner diameter: {D_I} mm")

    # Setup FlowsheetState
    fs = FlowsheetState()
    inputs = Inputs()
    fs.set("fluid", FLUID)

    # Set diameter in all possible keys
    for key in ("d_i", "d_h", "seg_d_i", "seg_d_h"):
        fs.set(key, d_i_si)

    fs.set("mass_flux", G)
    fs.set("seg_mass_flux", G)
    fs.set("q_flux", q_flux_si)
    fs.set("seg_q_flux", q_flux_si)
    fs.set("p_sat", p_sat)
    fs.set("seg_p_sat", p_sat)
    fs.set("seg_p", p_sat)

    # Initialize models
    print(f"\n{'=' * 70}")
    print(f"Model Initialization")
    print(f"{'=' * 70}")

    # Liu-Winterton model (main)
    lw = LiuWintertonTwoPhase(orientation="horizontal")
    print("[Liu-Winterton] Model initialized")

    # Gungor-Winterton model (optional)
    if COMPARE_WITH_GW:
        gw = GungorWintertonTwoPhase86(orientation="horizontal")
        print("[Gungor-Winterton] Model initialized for comparison")

    # Kandlikar model (optional)
    if COMPARE_WITH_KANDLIKAR:
        if KANDLIKAR_F_FL_VALUE is not None:
            kandlikar = KandlikarTwoPhase(orientation="horizontal", f_fl=KANDLIKAR_F_FL_VALUE)
            print(f"[Kandlikar] Using explicit F_fl = {KANDLIKAR_F_FL_VALUE}")
        elif KANDLIKAR_FLUID_NAME is not None:
            kandlikar = KandlikarTwoPhase(orientation="horizontal", fluid_name=KANDLIKAR_FLUID_NAME)
            print(f"[Kandlikar] Using fluid_name = '{KANDLIKAR_FLUID_NAME}', F_fl = {kandlikar.f_fl}")
        else:
            kandlikar = KandlikarTwoPhase(orientation="horizontal")
            print(f"[Kandlikar] Using default F_fl = {kandlikar.f_fl}")

    # Calculate mass flow rate
    m_flow = G * (math.pi * (d_i_si ** 2) / 4.0)

    # Generate vapor quality grid
    x_grid = np.linspace(X_MIN, X_MAX, N_X)

    # Calculate HTC using Liu-Winterton model
    print(f"\n{'=' * 70}")
    print(f"Calculating HTC vs vapor quality")
    print(f"{'=' * 70}")

    alpha_lw = []
    for x in x_grid:
        fs.set("seg_x", float(x))
        try:
            htc = lw.calc(st_liq, st_vap, st_liq, st_vap, med, inputs, fs, m_flow)
            alpha_lw.append(float(htc))
        except Exception as e:
            print(f"[Warning] Liu-Winterton calculation failed at x={x:.2f}: {e}")
            alpha_lw.append(np.nan)

    # Calculate HTC using Gungor-Winterton model if enabled
    if COMPARE_WITH_GW:
        alpha_gw = []
        for x in x_grid:
            fs.set("seg_x", float(x))
            try:
                htc = gw.calc(st_liq, st_vap, st_liq, st_vap, med, inputs, fs, m_flow)
                alpha_gw.append(float(htc))
            except Exception as e:
                print(f"[Warning] G&W calculation failed at x={x:.2f}: {e}")
                alpha_gw.append(np.nan)

    # Calculate HTC using Kandlikar model if enabled
    if COMPARE_WITH_KANDLIKAR:
        alpha_kandlikar = []
        for x in x_grid:
            fs.set("seg_x", float(x))
            try:
                htc = kandlikar.calc(st_liq, st_vap, st_liq, st_vap, med, inputs, fs, m_flow)
                alpha_kandlikar.append(float(htc))
            except Exception as e:
                print(f"[Warning] Kandlikar calculation failed at x={x:.2f}: {e}")
                alpha_kandlikar.append(np.nan)

    # ==========================================================================
    # Plotting
    # ==========================================================================
    mpl.rcParams.update({
        "axes.grid": True,
        "grid.linestyle": "dotted",
        "font.size": 10,
        "axes.labelsize": 10,
        "lines.linewidth": 1.4,
        "legend.fontsize": 9,
    })

    fig, ax = plt.subplots(figsize=(7, 5))

    # Plot Liu-Winterton model (main - solid line)
    ax.plot(
        x_grid,
        alpha_lw,
        lw=2.5,
        color="C2",
        label=r"Liu & Winterton (1991)",
        linestyle="-"
    )

    # Plot Gungor-Winterton model if enabled (dashed)
    if COMPARE_WITH_GW:
        ax.plot(
            x_grid,
            alpha_gw,
            lw=2.0,
            color="C0",
            label=r"Gungor & Winterton (1986)",
            linestyle="--"
        )

    # Plot Kandlikar model if enabled (dash-dot)
    if COMPARE_WITH_KANDLIKAR:
        ax.plot(
            x_grid,
            alpha_kandlikar,
            lw=2.0,
            color="C1",
            label=r"Kandlikar (1990)",
            linestyle="-."
        )

    # ==========================================================================
    # Load and plot experimental data with error calculation
    # ==========================================================================
    if EXPERIMENT_DATASETS:
        print(f"\n{'=' * 70}")
        print(f"Experimental Data Comparison")
        print(f"{'=' * 70}")

    for label, path, marker in EXPERIMENT_DATASETS:
        if not os.path.exists(path):
            print(f"[Warning] CSV file not found: {path}")
            continue

        try:
            x_exp, h_exp = load_csv_experimental_data(path)

            # Sort by vapor quality
            order = np.argsort(x_exp)
            x_exp, h_exp = x_exp[order], h_exp[order]

            # Plot experimental data
            ax.scatter(
                x_exp,
                h_exp,
                s=50,
                color="C3",
                marker=marker,
                label=label,
                zorder=5,
                edgecolors='black',
                linewidths=0.5
            )

            # Calculate error metrics for Liu-Winterton model
            h_lw_interp = np.interp(
                x_exp, x_grid, alpha_lw, left=np.nan, right=np.nan
            )
            valid_mask = np.isfinite(h_lw_interp)

            if np.any(valid_mask):
                diff = h_lw_interp[valid_mask] - h_exp[valid_mask]
                mae = float(np.mean(np.abs(diff)))
                mape = float(np.mean(np.abs(diff) / np.maximum(1e-9, h_exp[valid_mask]))) * 100.0
                print(f"[{label}] Liu-Winterton: N={valid_mask.sum()} MAE={mae:.0f} W/m2K MAPE={mape:.1f}%")

            # Calculate error metrics for Gungor-Winterton model if enabled
            if COMPARE_WITH_GW:
                h_gw_interp = np.interp(
                    x_exp, x_grid, alpha_gw, left=np.nan, right=np.nan
                )
                valid_mask_gw = np.isfinite(h_gw_interp)

                if np.any(valid_mask_gw):
                    diff_gw = h_gw_interp[valid_mask_gw] - h_exp[valid_mask_gw]
                    mae_gw = float(np.mean(np.abs(diff_gw)))
                    mape_gw = float(
                        np.mean(np.abs(diff_gw) / np.maximum(1e-9, h_exp[valid_mask_gw]))
                    ) * 100.0
                    print(f"[{label}] G&W: N={valid_mask_gw.sum()} MAE={mae_gw:.0f} W/m2K MAPE={mape_gw:.1f}%")

            # Calculate error metrics for Kandlikar model if enabled
            if COMPARE_WITH_KANDLIKAR:
                h_kandlikar_interp = np.interp(
                    x_exp, x_grid, alpha_kandlikar, left=np.nan, right=np.nan
                )
                valid_mask_kandlikar = np.isfinite(h_kandlikar_interp)

                if np.any(valid_mask_kandlikar):
                    diff_kandlikar = h_kandlikar_interp[valid_mask_kandlikar] - h_exp[valid_mask_kandlikar]
                    mae_kandlikar = float(np.mean(np.abs(diff_kandlikar)))
                    mape_kandlikar = float(
                        np.mean(np.abs(diff_kandlikar) / np.maximum(1e-9, h_exp[valid_mask_kandlikar]))
                    ) * 100.0
                    print(
                        f"[{label}] Kandlikar: N={valid_mask_kandlikar.sum()} MAE={mae_kandlikar:.0f} W/m2K MAPE={mape_kandlikar:.1f}%")

        except Exception as e:
            print(f"[Error] Failed to process {label}: {e}")
            continue

    # Format plot
    ax.set_xlabel(r"Vapor quality $x$ [-]")
    ax.set_ylabel(r"Heat transfer coefficient $\alpha$ [W/m$^2$K]")
    ax.legend(loc="best")
    fig.tight_layout()

    # ==========================================================================
    # Generate filename
    # ==========================================================================
    if EXPERIMENT_DATASETS:
        dataset_tag = "multi" if len(EXPERIMENT_DATASETS) > 1 else EXPERIMENT_DATASETS[0][0]
    else:
        dataset_tag = "no-exp"

    # Create model tag based on which models are plotted
    model_tags = ["LW"]
    if COMPARE_WITH_GW:
        model_tags.append("GW")
    if COMPARE_WITH_KANDLIKAR:
        model_tags.append("Kand")
    model_tag = "-vs-".join(model_tags)

    file_stem = slugify(
        f"fig_{FLUID}_{model_tag}_T{format_number(T_SAT_C_OR_K)}_G{format_number(G)}_"
        f"q{format_number(Q_FLUX)}_d{format_number(D_I)}_exp-{dataset_tag}"
    )

    # ==========================================================================
    # Save figures (controlled by flags)
    # ==========================================================================
    if SAVE_SVG or SAVE_PNG:
        print(f"\n{'=' * 70}")
        print(f"Saving figures")
        print(f"{'=' * 70}")

    # Configure font embedding for vector graphics
    mpl.rcParams['svg.fonttype'] = 'none'
    mpl.rcParams['pdf.fonttype'] = 42

    # Save SVG if enabled
    if SAVE_SVG:
        svg_path = unique_path(SAVE_DIR, file_stem, "svg")
        fig.savefig(svg_path, bbox_inches="tight")
        print(f"SVG saved: {svg_path}")

    # Save PNG if enabled
    if SAVE_PNG:
        png_path = unique_path(SAVE_DIR, file_stem, "png")
        fig.savefig(png_path, dpi=300, bbox_inches="tight")
        print(f"PNG saved: {png_path}")

    # Display figure in IDE or save and close
    if SHOW_FIG:
        plt.show()
    else:
        plt.close(fig)

    print(f"\n{'=' * 70}")
    print(f"Test completed successfully")
    print(f"{'=' * 70}\n")