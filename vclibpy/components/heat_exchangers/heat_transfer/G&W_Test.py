#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
run_gw_htc_vclibpy_coolprop_compare_simplecsv_latex_minimal_full.py
- G&W(1986) 모델 계산 + 헤더 없는 2열 실험 CSV 비교
- 소수점 → , / 자동 짧은 파일명 / MAE, MAPE 계산 포함
"""

import math, os, re, numpy as np, matplotlib.pyplot as plt, matplotlib as mpl

# ===== 저장 폴더 =====
SAVE_DIR = "/Users/wooseok/Desktop/Abbildungen"
os.makedirs(SAVE_DIR, exist_ok=True)

# --- 플래그 ---
SHOW_FIG = True
SAVE_PNG = True
EXPORT_TIKZ = False
EXPORT_PGF = False

# ===== 사용자 입력 =====
FLUID = "R22"
USE_PSAT = True
T_SAT_C_OR_K = -35.0    # Saturation Temperature
P_SAT_VALUE = 6.04         # Saturation Pressure
P_SAT_UNIT = "bar"      # bar, Pa, kPa, mPa
G = 363                # kg/m2s
Q_FLUX = 18.3           # kW/m2s
D_I = 6.0               # in mm
X_MIN, X_MAX, N_X = 0.1, 1, 41

EXPERIMENT_DATASETS = [
    ("Greco2005", "/Users/wooseok/Desktop/Abbildungen/plotdigitizer/CSV_aus_Paper/Greco_2005_R22_with_ID_6mm/R22/p_sat6_04_q18_3.csv", "o"),
]

# ===== VcLibPy =====
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers.heat_transfer.gw1986_htc import GungorWintertonTwoPhase86
from vclibpy.media import CoolProp, set_global_media_properties
set_global_media_properties(CoolProp, use_high_level_api=True)

# ===== 도우미 =====
def _slugify(s: str) -> str:
    s = re.sub(r"[^\w\-,.]+", "-", s, flags=re.ASCII)
    return re.sub(r"-{2,}", "-", s).strip("-._")

def _fmt_num(val: float, digits: int = 1) -> str:
    """예: 71.5 → 71,5"""
    return f"{val:.{digits}f}".replace(".", ",")

def _unique_path(base_dir: str, stem: str, ext: str) -> str:
    path = os.path.join(base_dir, f"{stem}.{ext}")
    if not os.path.exists(path):
        return path
    k = 1
    while True:
        alt = os.path.join(base_dir, f"{stem}-{k}.{ext}")
        if not os.path.exists(alt):
            return alt
        k += 1

def to_K(T: float) -> float:
    return T + 273.15 if -150.0 < T < 200.0 else T

def psat_to_Pa(v, unit):
    u = unit.lower()
    return v * {"pa":1, "kpa":1e3, "mpa":1e6, "bar":1e5}[u]

def get_sat_states(med, use_psat, T_sat_input, P_value, P_unit):
    if use_psat:
        p_sat = psat_to_Pa(P_value, P_unit)
        st_liq = med.calc_state("PQ", p_sat, 0.0)
        st_vap = med.calc_state("PQ", p_sat, 1.0)
        return st_liq, st_vap, p_sat
    else:
        T_sat = to_K(T_sat_input)
        st_liq = med.calc_state("TQ", T_sat, 0.0)
        st_vap = med.calc_state("TQ", T_sat, 1.0)
        return st_liq, st_vap, st_liq.p

def load_simple_csv(path):
    import pandas as pd
    tries = [(";", ","), (",", "."), ("\t", ","), (r"\s+", ",")]
    for sep, dec in tries:
        try:
            df = pd.read_csv(path, header=None, sep=sep, decimal=dec, engine="python")
            if df.shape[1] >= 2:
                x = pd.to_numeric(df.iloc[:, 0], errors="coerce").to_numpy(float)
                y = pd.to_numeric(df.iloc[:, 1], errors="coerce").to_numpy(float)
                m = np.isfinite(x) & np.isfinite(y)
                if m.any():
                    print(f"[CSV] {os.path.basename(path)} → N={m.sum()}")
                    return x[m], y[m]
        except Exception:
            pass
    raise ValueError(f"CSV 파싱 실패: {path}")

# ===== 실행 =====
if __name__ == "__main__":
    Q_FLUX_SI = Q_FLUX * 1000.0
    D_I_SI = D_I / 1000.0

    med = CoolProp(FLUID, use_high_level_api=True)
    st_liq, st_vap, p_sat = get_sat_states(med, USE_PSAT, T_SAT_C_OR_K, P_SAT_VALUE, P_SAT_UNIT)

    fs = FlowsheetState(); inputs = Inputs()
    fs.set("fluid", FLUID)
    for key in ("d_i", "d_h", "seg_d_i", "seg_d_h"): fs.set(key, D_I_SI)
    fs.set("mass_flux", G); fs.set("seg_mass_flux", G)
    fs.set("q_flux", Q_FLUX_SI); fs.set("seg_q_flux", Q_FLUX_SI)
    fs.set("p_sat", p_sat); fs.set("seg_p_sat", p_sat); fs.set("seg_p", p_sat)

    gw = GungorWintertonTwoPhase86(orientation="horizontal")
    m_flow = G * (math.pi * (D_I_SI ** 2) / 4.0)

    x_grid = np.linspace(X_MIN, X_MAX, N_X)
    alpha_vals = []
    for x in x_grid:
        fs.set("seg_x", float(x))
        alpha_vals.append(float(gw.calc(st_liq, st_vap, st_liq, st_vap, med, inputs, fs, m_flow)))

    mpl.rcParams.update({
        "axes.grid": True, "grid.linestyle": "dotted",
        "font.size": 10, "axes.labelsize": 10,
        "lines.linewidth": 1.4, "legend.fontsize": 9,
    })
    fig, ax = plt.subplots(figsize=(6,4))
    ax.plot(x_grid, alpha_vals, lw=2.0, color="C0", label=r"G&W")

    # ---- 실험 데이터 비교 + 오차 계산 ----
    for label, path, mark in EXPERIMENT_DATASETS:
        if not os.path.exists(path):
            print(f"[경고] CSV 파일을 찾지 못함: {path}")
            continue
        x_exp, h_exp = load_simple_csv(path)
        order = np.argsort(x_exp)
        x_exp, h_exp = x_exp[order], h_exp[order]
        ax.scatter(x_exp, h_exp, s=40, color="C3", marker=mark, label=label)

        # 모델 보간 + 오차 계산
        h_model = np.interp(x_exp, x_grid, alpha_vals, left=np.nan, right=np.nan)
        ok = np.isfinite(h_model)
        if np.any(ok):
            diff = h_model[ok] - h_exp[ok]
            mae = float(np.mean(np.abs(diff)))
            mape = float(np.mean(np.abs(diff) / np.maximum(1e-9, h_exp[ok]))) * 100.0
            print(f"[{label}] N={ok.sum()}  MAE={mae:.0f} W/m²K  MAPE={mape:.1f} %")

    ax.set_xlabel(r"Vapor quality $x$ in --")
    ax.set_ylabel(r"Heat transfer coefficient $\alpha$ in W/m$^2$K")
    ax.legend(loc="upper left")
    fig.tight_layout()

    # ---- 짧고 정확한 파일명 ----
    ds_tag = "multi" if len(EXPERIMENT_DATASETS) > 1 else EXPERIMENT_DATASETS[0][0]
    FILE_STEM = _slugify(
        f"fig_{FLUID}_T{_fmt_num(T_SAT_C_OR_K)}_G{_fmt_num(G)}_q{_fmt_num(Q_FLUX)}_d{_fmt_num(D_I)}_exp-{ds_tag}"
    )

    # ===== 저장 =====
    mpl.rcParams['svg.fonttype'] = 'none'
    svg_path = _unique_path(SAVE_DIR, FILE_STEM, "svg")
    fig.savefig(svg_path, bbox_inches="tight")
    print(f"SVG 저장: {svg_path}")

    if SAVE_PNG:
        png_path = _unique_path(SAVE_DIR, FILE_STEM, "png")
        fig.savefig(png_path, dpi=300, bbox_inches="tight")
        print(f"PNG 저장: {png_path}")

    if SHOW_FIG:
        plt.show()
    else:
        plt.close(fig)