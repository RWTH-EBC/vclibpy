# segmented_evaporator_improved.py
"""
VcLibPy-compatible segmented evaporator implementation.

This module provides a segmented evaporator model that follows VcLibPy
conventions and includes:
- Gungor & Winterton-compatible two-phase HTC hook (via callable)
- Dynamic secondary-side HTC calculation (or constant fallback)
- Fixed-point iteration for two-phase q'' with under-relaxation
- Phase-boundary protection per segment
- Energy conservation check and diagnostics into FlowsheetState
"""

from __future__ import annotations
import math
import warnings
from typing import Dict, Optional, Tuple
import numpy as np

from vclibpy.components.heat_exchangers.heat_exchanger import ExternalHeatExchanger
from vclibpy.datamodels import FlowsheetState, Inputs


class SegmentedEvaporatorImproved(ExternalHeatExchanger):
    """
    Segmented evaporator with forward-marching segment balance.

    Assumptions
    -----------
    - Constant refrigerant pressure (no pressure drop model; TODO for future).
    - Segment-wise heat transfer: dQ = k * dA * ΔT_local.
    - Two-phase regime: ΔT = T_sec - T_sat(p).
    - Single-phase regimes: ΔT = T_sec - T_ref (refrigerant bulk temperature).
    - Optional dynamic secondary-side HTC; otherwise a constant is used.
    - Phase-boundary protection so a single segment does not cross regimes.
    - Two-phase q'' solved by fixed-point iteration with adaptive relaxation.

    Parameters
    ----------
    n_segments : int, default 20
        Number of segments across the total area `A`.
    d_h : float, default 0.01
        Hydraulic diameter [m]. Used for Reynolds/Db HTC and two-phase callable.
    alpha_sec_const : float, default 50.0
        Constant secondary-side HTC [W/m²K] if dynamic calc is disabled.
    relax_omega : float, default 0.4
        Base under-relaxation factor (0 < omega < 1).
    tol : float, default 1e-6
        Fixed-point convergence tolerance for q''.
    max_iter : int, default 50
        Maximum iterations in two-phase q'' solve.
    use_dynamic_secondary : bool, default True
        If True compute secondary HTC each segment from `calc_alpha_secondary`.

    **kwargs :
        Forwarded to `ExternalHeatExchanger.__init__` (e.g., A, models, media, etc.).
    """

    def __init__(
            self,
            *,
            n_segments: int = 20,
            d_h: float = 0.01,
            alpha_sec_const: float = 50.0,
            relax_omega: float = 0.4,
            tol: float = 1e-6,
            max_iter: int = 50,
            use_dynamic_secondary: bool = True,
            x_vap_min: float = 0.99,
            **kwargs,
    ):
        super().__init__(**kwargs)

        tp = kwargs.get("two_phase_heat_transfer", None)

        if tp is None:
            raise ValueError(
                "two_phase_heat_transfer must be provided (e.g., GungorWintertonTwoPhase86 instance)."
            )
        self.two_phase_heat_transfer = tp

        # --- input validation ---
        if n_segments < 1:
            raise ValueError("n_segments must be >= 1")
        if d_h <= 0:
            raise ValueError("d_h must be > 0")
        if not (0 < relax_omega < 1):
            raise ValueError("relax_omega must be between 0 and 1")

        # --- model parameters ---
        self.n_segments = int(n_segments)
        self.d_h = float(d_h)

        self.alpha_sec_const = float(alpha_sec_const)
        self.use_dynamic_secondary = bool(use_dynamic_secondary)

        self.relax_omega = float(relax_omega)
        self.tol = float(tol)
        self.max_iter = int(max_iter)

        self.x_vap_min = float(x_vap_min)

        self._sat_props_current = None
        self._st_f = None
        self._st_g = None
    # --------------------------------------------------------------------- #
    # Utilities
    # --------------------------------------------------------------------- #
    def _get_saturation_properties_current(self) -> Tuple[float, float, float]:
        """
        Return saturation properties for the current run.

        Values are set once per calc() call and reused for all segments.

        Returns
        -------
        (T_sat, h_f, h_g)
            T_sat : saturation temperature [K]
            h_f   : enthalpy of saturated liquid [J/kg]
            h_g   : enthalpy of saturated vapor [J/kg]
        """
        if self._sat_props_current is None:
            raise RuntimeError(
                "Saturation properties have not been initialized. "
                "Call calc() before using this method."
            )
        return self._sat_props_current

    def _dq_to_boundary(self, h_now: float) -> float:
        """Return heat needed to reach the next phase boundary from h_now (no margin)."""
        _, h_f, h_g = self._get_saturation_properties_current()
        if h_now < h_f:  # subcooled → two-phase
            return self.m_flow * (h_f - h_now)
        elif h_now < h_g:  # two-phase → superheated
            return self.m_flow * (h_g - h_now)
        else:  # superheated
            return float("inf")  # no boundary ahead within this phase

    def _adaptive_relaxation(self, iteration: int) -> float:
        """
        Adaptive under-relaxation factor as iterations increase.
        """
        if iteration < 10:
            return self.relax_omega
        if iteration < 30:
            return self.relax_omega * 0.8
        return self.relax_omega * 0.6

    def calc_alpha_two_phase(
            self,
            *,
            state_q0,
            state_q1,
            inputs,
            fs_state,
    ):
        """
        Override of ExternalHeatExchanger.calc_alpha_two_phase.
        """
        model = getattr(self, "two_phase_heat_transfer", None)
        if model is None:
            raise RuntimeError("two_phase_heat_transfer model is not set")

        return model.calc(
            state_q0=state_q0,
            state_q1=state_q1,
            state_inlet=self.state_inlet,
            state_outlet=getattr(self, "state_outlet", None),
            med_prop=self.med_prop,
            inputs=inputs,
            fs_state=fs_state,
            m_flow=self.m_flow,
        )

    # --------------------------------------------------------------------- #
    # Two-phase segment solve
    # --------------------------------------------------------------------- #
    def _solve_qpp_two_phase(
            self,
            *,
            x: float,
            p: float,
            T_sec_in: float,
            da: float,
            qpp_guess: Optional[float] = None,
            inputs: Inputs,
            fs_state: FlowsheetState,
            return_diagnostics: bool = False,
    ) -> Tuple[float, float, bool, int, Optional[float], Optional[float]]:
        """
        Fixed-point solve for two-phase heat flux q'' (per area) in one segment.

        The solver iterates on:
            q''_{n+1} = k(alpha_in(q'', x, ...), alpha_sec) * ΔT,
        and returns the converged q'' and resulting segment heat. Optionally,
        it also returns (and can log) the final inner HTC and overall U.

        Args:
            x (float): Segment representative vapor quality (can be out-of-bounds; will be clamped).
            p (float): Segment pressure [Pa] (assumed constant within the segment).
            T_sec_in (float): Secondary-side bulk temperature [K] for this segment.
            da (float): Segment area [m^2].
            qpp_guess (Optional[float]): Initial guess for q'' [W/m^2].
            inputs (Inputs): Flowsheet/cycle inputs.
            fs_state (FlowsheetState): Diagnostic container for logging.
            return_diagnostics (bool): If True, also return alpha_in_final and k_final,
                and store generic keys for the current segment ("seg_*_final").
                The caller may still want to store per-segment indexed keys.

        Returns:
            Tuple[
                float,      # qpp: converged heat flux [W/m^2]
                float,      # dQ: segment heat [W]
                bool,       # converged flag
                int,        # iterations
                Optional[float],  # alpha_in_final [W/m^2/K] if return_diagnostics else None
                Optional[float],  # k_final [W/m^2/K]         if return_diagnostics else None
            ]
        """
        # --- saturation props for this run (constant over the segment) ---
        T_sat, h_f, h_g = self._get_saturation_properties_current()

        # Driving ΔT (evaporator): secondary hotter than refrigerant saturation
        dT = T_sec_in - T_sat
        if dT <= 0.0:
            # No driving force; nothing to transfer in this segment
            if return_diagnostics:
                try:
                    fs_state.set("seg_dT_final", float(dT), "K", "two-phase ΔT in segment (no driving force)")
                except Exception:
                    pass
            return 0.0, 0.0, True, 0, (0.0 if return_diagnostics else None), (0.0 if return_diagnostics else None)

        # Secondary-side HTC (dynamic or constant)
        if self.use_dynamic_secondary:
            tra_sec = self.calc_transport_properties_secondary_medium(T=T_sec_in)
            alpha_sec = self.calc_alpha_secondary(tra_sec)
        else:
            alpha_sec = self.alpha_sec_const

        # Physics-based initial guess via liquid-only correlation (Dittus–Boelter at sat. liquid)
        st_l = self._st_f
        tp_l = self.med_prop.calc_transport_properties(st_l)
        mu_l = float(tp_l.dyn_vis)
        k_l = float(tp_l.lam)
        Pr_l = float(tp_l.Pr)

        A_flow = math.pi * (self.d_h ** 2) / 4.0
        G = self.m_flow / A_flow
        Re_l = G * self.d_h / mu_l
        alpha_lo = 0.023 * (Re_l ** 0.8) * (Pr_l ** 0.4) * (k_l / self.d_h)

        k0 = self.calc_k(alpha_pri=alpha_lo, alpha_sec=alpha_sec)
        qpp = float(qpp_guess) if (qpp_guess is not None and qpp_guess > 0.0) else k0 * dT

        # Reference states for the two-phase HTC implementation
        state_q0 = st_l
        state_q1 = self._st_g

        # Log once-per-segment items
        try:
            fs_state.set("seg_p", p, "Pa", "segment pressure (assumed const per segment)")
            if fs_state.get("seg_d_h") is None:
                fs_state.set("seg_d_h", self.d_h, "m", "hydraulic diameter (constant)")
        except Exception:
            pass

        # --- Physically scaled tolerance per segment ---
        Q_scale_seg = max(1.0, abs(self.m_flow * (h_g - h_f) / self.n_segments))  # [W]
        qpp_scale = Q_scale_seg / da  # [W/m^2]
        tol_rel = self.tol
        tol_abs = 1e-6 * qpp_scale

        alpha_in_last: Optional[float] = None

        for it in range(1, self.max_iter + 1):
            # Provide segment-local data for the two-phase model via fs_state
            x_used = max(1.0e-6, min(1.0 - 1.0e-6, x))
            try:
                fs_state.set("seg_x", x_used, "-", "segment vapor quality (clamped)")
                fs_state.set("seg_q_flux", qpp, "W/m2", "segment heat flux (iteration guess)")
            except Exception:
                pass

            # Get inner HTC from the user's TwoPhaseHeatTransfer implementation
            alpha_in = self.calc_alpha_two_phase(
                state_q0=state_q0,
                state_q1=state_q1,
                inputs=inputs,
                fs_state=fs_state,
            )
            alpha_in_last = float(alpha_in)

            # Fixed-point update: q'' = k * ΔT (k from inner & secondary sides)
            k = self.calc_k(alpha_pri=alpha_in_last, alpha_sec=alpha_sec)
            qpp_new = k * dT

            # Convergence: relative + absolute tolerance
            if abs(qpp_new - qpp) <= tol_rel * abs(qpp_new) + tol_abs:
                alpha_in_final = alpha_in_last if alpha_in_last is not None else float(alpha_in)
                k_final = self.calc_k(alpha_pri=alpha_in_final, alpha_sec=alpha_sec)

                # Optional generic diagnostics (caller may additionally store per-segment keys)
                if return_diagnostics:
                    try:
                        fs_state.set("seg_alpha_in_final", float(alpha_in_final), "W/m2K",
                                     "final inner HTC (two-phase)")
                        fs_state.set("seg_k_final", float(k_final), "W/m2K", "final overall U (two-phase)")
                        fs_state.set("seg_q_flux_final", float(qpp_new), "W/m2", "converged heat flux q''")
                        fs_state.set("seg_dT_final", float(dT), "K", "two-phase ΔT in segment")
                    except Exception:
                        pass

                return float(qpp_new), float(qpp_new * da), True, it, float(alpha_in_final), float(k_final)

            # Under-relaxed update to damp oscillations
            omega = self._adaptive_relaxation(it)
            qpp = (1.0 - omega) * qpp + omega * qpp_new

            # Keep q'' within reasonable physical bounds
            qpp = max(0.0, min(qpp, 1.0e6))

        # Not converged → record and raise
        try:
            fs_state.set("tp_convergence_failed", 1, "-", "two-phase solver did not converge in this segment")
        except Exception:
            pass
        raise RuntimeError(
            f"Two-phase q'' iteration did not converge in {self.max_iter} iterations. "
            f"(p={p:.3f} Pa, T_sec={T_sec_in:.3f} K, dT={dT:.6g} K, x={x:.6f}, last_qpp={qpp:.6g} W/m^2)"
        )

    # --------------------------------------------------------------------- #
    # Main calc
    # --------------------------------------------------------------------- #
    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> Tuple[float, float]:
        """
        Calculate the segmented evaporator.
        """
        self._validate_inputs()
        self._sat_props_current = None  # reset for this run
        self._initialize_secondary_medium(inputs)

        # Refrigerant inlet
        p = float(self.state_inlet.p)
        h = float(self.state_inlet.h)
        state = self.med_prop.calc_state("PH", p, h)
        x = float(state.q)

        # --- compute saturation properties once for this run ---
        st_f = self.med_prop.calc_state("PQ", p, 0.0)
        st_g = self.med_prop.calc_state("PQ", p, 1.0)
        self._st_f, self._st_g = st_f, st_g
        self._sat_props_current = (float(st_f.T), float(st_f.h), float(st_g.h))

        # --- Secondary-side inlet via standardized interface (ensure consistency) ---
        #     We fetch inlet temperature and mass flow again to ensure consistency with flowsheet.
        #     If cp was not set in _initialize_secondary_medium, compute it here at T_in.
        T_sec_in, _, _, m_sec = inputs.evaporator.get_all_inputs(
            cp=self.cp_secondary,
            Q=0.0
        )

        # Update mass flow and capacity rate robustly (idempotent)
        self.m_flow_secondary = float(m_sec)
        if not self.cp_secondary:
            self.calc_secondary_cp(T=T_sec_in)

        # --- Cross-flow single-row assumption: keep secondary bulk temperature constant per row ---
        T_sec = float(T_sec_in)

        # --- Segment marching initializations ---
        da = self.A / self.n_segments
        Q_sum = 0.0
        dT_min = float("inf")
        last_qpp: Optional[float] = None
        n_fail = 0
        seg_count = {"subcooled": 0, "two_phase": 0, "superheated": 0}
        tp_iters_total = 0

        for seg_idx in range(self.n_segments):
            state = self.med_prop.calc_state("PH", p, h)
            T_ref = float(state.T)

            h_f = float(self._st_f.h)
            h_g = float(self._st_g.h)
            x_raw = (state.h - h_f) / (h_g - h_f)
            x01 = max(0.0, min(1.0, x_raw))

            # Determine regime by quality
            if x_raw < 0.0:
                # Subcooled
                dT = T_sec - T_ref
                if dT <= 0:
                    dT_min = min(dT_min, dT)
                    fs_state.set("pinch_reached", 1, "-", "No driving ΔT in subcooled")
                    #break

                if self.use_dynamic_secondary:
                    tp_sec = self.calc_transport_properties_secondary_medium(T=T_sec)
                    alpha_sec = self.calc_alpha_secondary(tp_sec)
                else:
                    alpha_sec = self.alpha_sec_const

                tp_ref = self.med_prop.calc_transport_properties(state)
                alpha_i = self.calc_alpha_liquid(tp_ref)
                k = self.calc_k(alpha_pri=alpha_i, alpha_sec=alpha_sec)
                dQ_trial = k * da * dT
                dQ_to = self._dq_to_boundary(h)  # heat to reach h_f exactly

                if dQ_trial <= dQ_to or not math.isfinite(dQ_to):
                    # 경계를 넘지 않음: 전 면적을 liquid로 사용
                    dQ = dQ_trial
                    q_flux = dQ / da
                else:
                    # 경계까지 쓴 뒤, 남은 면적은 두상으로 즉시 진행
                    da_to = dQ_to / (k * dT)  # liquid로 경계에 닿는 데 필요한 부분면적
                    dQ1 = dQ_to  # liquid 부분에서 소비된 열량
                    h_tmp = h + dQ1 / self.m_flow  # == h_f

                    da_rem = max(0.0, da - da_to)  # 남은 면적

                    dQ2 = 0.0
                    if da_rem > 0.0:
                        # 두상 시작은 x≈0에서 시작 (함수 내부에서 안전하게 clamp함)
                        qpp, dQ2_trial, converged, iters, alpha_in_final, k_final = self._solve_qpp_two_phase(
                            x=0.0, p=p, T_sec_in=T_sec, da=da_rem, qpp_guess=last_qpp,
                            inputs=inputs, fs_state=fs_state, return_diagnostics=False
                        )
                        last_qpp = qpp

                        # 두상에서 상한: h_g까지
                        dQ2_to = self._dq_to_boundary(h_tmp)  # m*(h_g - h_f)
                        dQ2 = min(dQ2_trial, dQ2_to)

                        # 두상 메타데이터(선택): k_final/alpha_in_final 있으면 기록
                        if k_final is not None:
                            fs_state.set(f"seg_k_{seg_idx}_tp", float(k_final), "W/m2K", "U in two-phase (partial)")
                        if alpha_in_final is not None:
                            fs_state.set(f"seg_alpha_in_{seg_idx}_tp", float(alpha_in_final), "W/m2K",
                                         "inner HTC (G&W, partial)")

                        # 만약 dQ2_trial > dQ2_to 여서 superheated로도 넘어가야 할 면적이 남는다면,
                        # 그 처리는 아래 3) 두상 블록에서와 동일한 패턴으로 구현할 수 있지만,
                        # 여기서는 '서브쿨드→두상'까지만 split 하고, superheated는 다음 세그먼트에서 처리되도록 놔둬도 충분히 안정적임.

                    dQ = dQ1 + dQ2
                    q_flux = dQ / da

                seg_count["subcooled"] += 1

                fs_state.set(f"seg_regime_{seg_idx}", "subcooled", "-", "segment regime")
                fs_state.set(f"seg_k_{seg_idx}", float(k), "W/m2K", "overall U in segment")
                fs_state.set(f"seg_q_flux_{seg_idx}", float(q_flux), "W/m2", "effective heat flux after capping")
                fs_state.set(f"seg_dT_{seg_idx}", float(dT), "K", "driving ΔT in segment")
                fs_state.set(f"seg_alpha_in_{seg_idx}", float(alpha_i), "W/m2K", "inner HTC (liquid)")
                fs_state.set(f"seg_alpha_sec_{seg_idx}", float(alpha_sec), "W/m2K", "secondary-side HTC")
                fs_state.set(f"seg_x_{seg_idx}", float(x01), "-", "segment vapor quality (clamped to [0,1])")

            elif x_raw <= self.x_vap_min:    # Two-phase (x_raw in (0,x_vap_min))
                qpp, dQ_trial, converged, iters, alpha_in_final, k_final = (
                    self._solve_qpp_two_phase(x=x_raw, p=p, T_sec_in=T_sec, da=da, qpp_guess=last_qpp, inputs=inputs,
                    fs_state=fs_state, return_diagnostics=False))
                last_qpp = qpp
                dQ_to = self._dq_to_boundary(h)  # heat to reach h_g exactly

                if dQ_trial <= dQ_to or not math.isfinite(dQ_to):
                    # 경계를 넘지 않음: 전 면적을 두상으로 사용
                    dQ = dQ_trial
                    qpp_eff = dQ / da
                else:
                    # 두상 면적 중 일부만 써서 h_g에 정확히 도달
                    da_tp = dQ_to / qpp  # 두상에서 경계까지 필요한 부분면적 (qpp는 W/m2)
                    dQ_tp = dQ_to  # 두상에서 소비된 열량
                    h_tmp = h + dQ_tp / self.m_flow  # == h_g

                    # 남은 면적은 superheated로 즉시 진행
                    da_rem = max(0.0, da - da_tp)
                    dQ_sh = 0.0
                    if da_rem > 0.0:
                        state_sh = self.med_prop.calc_state("PH", p, h_tmp)
                        T_ref_sh = float(state_sh.T)
                        dT_sh = T_sec - T_ref_sh
                        if dT_sh > 0.0:
                            if self.use_dynamic_secondary:
                                tp_sec = self.calc_transport_properties_secondary_medium(T=T_sec)
                                alpha_sec = self.calc_alpha_secondary(tp_sec)
                            else:
                                alpha_sec = self.alpha_sec_const

                            tp_ref_sh = self.med_prop.calc_transport_properties(state_sh)
                            alpha_i_sh = self.calc_alpha_gas(tp_ref_sh)
                            k_sh = self.calc_k(alpha_pri=alpha_i_sh, alpha_sec=alpha_sec)
                            dQ_sh = k_sh * da_rem * dT_sh
                            # 필요하면 진단값 기록:
                            fs_state.set(f"seg_k_{seg_idx}_sh", float(k_sh), "W/m2K", "U in superheated (partial)")
                            fs_state.set(f"seg_alpha_in_{seg_idx}_sh", float(alpha_i_sh), "W/m2K",
                                         "inner HTC (gas, partial)")
                        else:
                            fs_state.set("pinch_reached", 1, "-", "No driving ΔT in superheated (partial)")

                    dQ = dQ_tp + dQ_sh
                    qpp_eff = dQ / da

                # 두상일 때의 ΔT는 포화온도 기준
                T_sat, _, _ = self._get_saturation_properties_current()
                dT = T_sec - T_sat

                seg_count["two_phase"] += 1
                tp_iters_total += iters

                if self.use_dynamic_secondary:
                    tp_sec = self.calc_transport_properties_secondary_medium(T=T_sec)
                    alpha_sec = self.calc_alpha_secondary(tp_sec)
                else:
                    alpha_sec = self.alpha_sec_const

                fs_state.set(f"seg_regime_{seg_idx}", "two_phase", "-", "segment regime")
                if k_final is not None:
                    fs_state.set(f"seg_k_{seg_idx}", float(k_final), "W/m2K", "overall U in segment (two-phase)")
                fs_state.set(f"seg_q_flux_{seg_idx}", float(qpp_eff), "W/m2", "effective heat flux after capping")
                fs_state.set(f"seg_dT_{seg_idx}", float(dT), "K", "driving ΔT = T_sec - T_sat")
                fs_state.set(f"seg_alpha_sec_{seg_idx}", float(alpha_sec), "W/m2K", "secondary-side HTC")
                if alpha_in_final is not None:
                    fs_state.set(f"seg_alpha_in_{seg_idx}", float(alpha_in_final), "W/m2K", "inner HTC (G&W)")
                fs_state.set(f"seg_x_{seg_idx}", float(x01), "-", "segment vapor quality (clamped to [0,1])")

            else:
                # Superheated
                dT = T_sec - T_ref
                if dT <= 0:
                    dT = 0
                    dT_min = min(dT_min, dT)
                    fs_state.set("pinch_reached", 1, "-", "No driving ΔT in superheated")
                    #break

                if self.use_dynamic_secondary:
                    tp_sec = self.calc_transport_properties_secondary_medium(T=T_sec)
                    alpha_sec = self.calc_alpha_secondary(tp_sec)
                else:
                    alpha_sec = self.alpha_sec_const

                tp_ref = self.med_prop.calc_transport_properties(state)
                alpha_i = self.calc_alpha_gas(tp_ref)
                k = self.calc_k(alpha_pri=alpha_i, alpha_sec=alpha_sec)
                dQ = k * da * dT
                q_flux = dQ / da
                seg_count["superheated"] += 1

                fs_state.set(f"seg_regime_{seg_idx}", "superheated", "-", "segment regime")
                fs_state.set(f"seg_k_{seg_idx}", float(k), "W/m2K", "overall U in segment")
                fs_state.set(f"seg_q_flux_{seg_idx}", float(q_flux), "W/m2", "effective heat flux")
                fs_state.set(f"seg_dT_{seg_idx}", float(dT), "K", "driving ΔT in segment")
                fs_state.set(f"seg_alpha_in_{seg_idx}", float(alpha_i), "W/m2K", "inner HTC (gas)")
                fs_state.set(f"seg_alpha_sec_{seg_idx}", float(alpha_sec), "W/m2K", "secondary-side HTC")
                fs_state.set(f"seg_x_{seg_idx}", float(x01), "-", "segment vapor quality (clamped to [0,1])")

            # Cross-flow single-row: keep secondary bulk temperature constant per row
            # T_sec remains equal to T_sec_in within this calculation

            # Refrigerant enthalpy/state update
            h += dQ / self.m_flow
            Q_sum += dQ
            dT_min = min(dT_min, dT)

            # Simple sanity check for secondary temp
            if T_sec <= T_ref:
                warnings.warn(
                    f"Secondary is not hotter than refrigerant bulk in seg {seg_idx} "
                    f"(T_sec={T_sec:.2f} K, T_ref={T_ref:.2f} K) — no driving ΔT."
                )

        # Final outlet refrigerant state (after last segment update)
        self.state_outlet = self.med_prop.calc_state("PH", p, h)


        # Cross-flow single-row: compute secondary outlet once from the total row heat
        T_sec_out = T_sec_in - Q_sum / self.m_flow_secondary_cp

        print(f"Debug: Q_sum={Q_sum:.1f}W, C_sec={self.m_flow_secondary_cp:.3f}W/K")
        #print(f"Debug: T_sec_in={T_sec_in:.2f}K, T_sec_out={T_sec_out:.2f}K")
        #print(f"Debug: dT_sec={T_sec_in - T_sec_out:.2f}K")

        # Record results in fs_state
        self._record_results(
            fs_state=fs_state,
            q_sum=Q_sum,
            t_sec_out=T_sec_out,
            convergence_failures=n_fail,
            segment_regimes=seg_count,
            total_iterations=tp_iters_total,
        )

        dT_min_out = float(dT_min) if np.isfinite(dT_min) else 0.0

    # --------------------------------------------------------------------- #
    # Helpers
    # --------------------------------------------------------------------- #
    def _validate_inputs(self) -> None:
        """Validate required inputs before calculation."""
        if self.med_prop is None:
            raise ValueError("med_prop must be set before calculation")
        if self.m_flow <= 0:
            raise ValueError("Refrigerant mass flow rate must be > 0")
        if self.A <= 0:
            raise ValueError("Heat exchanger area A must be > 0")
        if not hasattr(self, "state_inlet") or self.state_inlet is None:
            raise ValueError("state_inlet must be set before calculation")

    def _initialize_secondary_medium(self, inputs: Inputs) -> None:
        """
        Initialize or verify secondary medium properties and capacity rate.

        - Ensures the secondary medium property backend is available.
        - Ensures cp_secondary is evaluated at a meaningful reference temperature.
        - Retrieves mass flow and temperature consistently via get_all_inputs.
        - Updates m_flow_secondary and m_flow_secondary_cp accordingly.

        Args:
            inputs (Inputs): Global Inputs object containing evaporator data.

        Raises:
            ValueError: If required evaporator inputs are missing.
        """
        if inputs.evaporator is None:
            raise ValueError("inputs.evaporator is required")

        # Ensure the secondary media wrapper is ready
        if self.med_prop_sec is None:
            self.start_secondary_med_prop()

        # Retrieve inlet temperature and mass flow via standardized interface
        # cp is set to self.cp_secondary if already available, otherwise a dummy (1.0) is passed
        T_sec_in, _, _, m_sec = inputs.evaporator.get_all_inputs(
            cp=self.cp_secondary, Q=0.0
        )

        if m_sec is None:
            raise ValueError(
                "SegmentedEvaporatorImproved requires 'inputs.evaporator.m_flow' "
                "on the secondary side. Please provide the air mass flow (kg/s)."
            )

        # Compute cp_secondary if not available yet
        if not self.cp_secondary:  # covers None or 0.0
            self.calc_secondary_cp(T=T_sec_in)

        # Update mass flow and derived capacity rate
        self.m_flow_secondary = float(m_sec)

    def _record_results(
        self,
        fs_state: FlowsheetState,
        q_sum: float,
        t_sec_out: float,
        convergence_failures: int,
        segment_regimes: Dict[str, int],
        total_iterations: int,
    ) -> None:
        """
        Store summary values into the flowsheet state.
        """
        fs_state.set("Q_eva_seg", q_sum, "W", "Segmented evaporator total heat")
        fs_state.set("T_sec_out_seg", t_sec_out, "K", "Secondary outlet temperature")
        fs_state.set("n_segments_used", self.n_segments, "-", "Number of segments marched")

        if convergence_failures > 0:
            fs_state.set(
                "tp_convergence_failures",
                convergence_failures,
                "-",
                "Number of two-phase convergence failures",
            )

        total = sum(segment_regimes.values())
        if total > 0:
            fs_state.set(
                "regime_subcooled_frac",
                segment_regimes["subcooled"] / total,
                "-",
                "Fraction of segments classified as subcooled",
            )
            fs_state.set(
                "regime_twophase_frac",
                segment_regimes["two_phase"] / total,
                "-",
                "Fraction of segments classified as two-phase",
            )
            fs_state.set(
                "regime_superheated_frac",
                segment_regimes["superheated"] / total,
                "-",
                "Fraction of segments classified as superheated",
            )

        if segment_regimes["two_phase"] > 0:
            avg_iters = total_iterations / segment_regimes["two_phase"]
            fs_state.set(
                "avg_tp_iterations",
                avg_iters,
                "-",
                "Average iterations per two-phase segment",
            )