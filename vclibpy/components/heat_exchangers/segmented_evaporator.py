from __future__ import annotations


from dataclasses import dataclass
from typing import Callable, List, Optional, Dict, Tuple
import math
import warnings


@dataclass
class SegmentResult:
    """
    Per-segment results of the evaporator march.

    regime:
        One of "subcooled_liquid", "two_phase", or "superheated_vapor".
        Classification follows the medium property's vapor quality convention:
          x < 0       -> subcooled_liquid
          0 <= x <= 1 -> two_phase
          x > 1       -> superheated_vapor
    """
    segment_id: int
    regime: str
    x_inlet: float
    x_outlet: float
    h_inlet: float
    h_outlet: float
    T_sat: Optional[float]          # reported only in two-phase regime
    delta_T: float                  # driving ΔT used in this segment
    q_flux: float                   # local heat flux q'' [W/m²]
    htc: float                      # segment HTC [W/m²K]
    Q_segment: float                # segment heat rate [W]
    converged: bool                 # fixed-point convergence (two-phase)
    iterations: int                 # number of fixed-point iterations (two-phase)


class SegmentedEvaporator:
    """
    Segmented evaporator with fixed-point iteration for q'' in the two-phase region.

    Assumptions
    -----------
    - Pressure is constant along the evaporator (no pressure drop model).
    - Wall/air side is quasi-isothermal: constant wall temperature T_w [K].
    - In two-phase, the driving temperature difference per segment is ΔT = T_w − T_sat(p).
    - Vapor quality x is advanced per segment by an energy balance without artificial clamping.

    Heat transfer models
    --------------------
    two_phase_htc:
        Callable with signature
          h = two_phase_htc(x, q_flux, p, m_flow, med_prop, d_h, orientation)
        returning h_tp in [W/m²K], e.g., a Gungor & Winterton implementation.

    single_phase_htc:
        Callable with signature
          h = single_phase_htc(T, p, m_flow, med_prop, d_h)
        returning h_sp in [W/m²K] for both subcooled-liquid and superheated-vapor regimes.
        If None, a simple Dittus–Boelter fallback is used.
    """

    def __init__(
        self,
        n_cells: int,
        A_total: float,
        d_h: float,
        T_w: float,
        two_phase_htc: Callable,
        orientation: str = "horizontal",
        single_phase_htc: Optional[Callable] = None,
        max_iter: int = 50,
        tol: float = 1e-6,
    ):
        if n_cells < 1:
            raise ValueError("n_cells must be >= 1.")
        if A_total <= 0.0:
            raise ValueError("A_total must be > 0.")
        if d_h <= 0.0:
            raise ValueError("d_h must be > 0.")

        self.n_cells = int(n_cells)
        self.A_cell = A_total / n_cells
        self.d_h = float(d_h)
        self.T_w = float(T_w)
        self.two_phase_htc = two_phase_htc
        self.single_phase_htc = single_phase_htc or self._default_single_phase_htc
        self.orientation = orientation
        self.max_iter = int(max_iter)
        self.tol = float(tol)

        # Cache for saturation states at constant pressure:
        # p -> (sat_liq_state, sat_vap_state, h_lv, T_sat)
        self._sat_cache: Dict[float, Tuple] = {}

    # ------------------------- internal utilities -------------------------

    def _area_flow(self) -> float:
        """Circular flow cross-sectional area from hydraulic diameter."""
        return math.pi * (self.d_h ** 2) / 4.0

    def _mass_flux(self, m_flow: float) -> float:
        """Mass flux G [kg/m²s]."""
        return m_flow / self._area_flow()

    def _get_saturation_states(self, p: float, med_prop):
        """
        Obtain saturation states and latent heat at pressure p (cached).
        Returns (st_liq, st_vap, h_lv, T_sat).
        """
        if p not in self._sat_cache:
            st_l = med_prop.calc_state("PQ", p, 0.0)
            st_v = med_prop.calc_state("PQ", p, 1.0)
            h_lv = st_v.h - st_l.h
            if h_lv <= 0.0:
                raise RuntimeError(f"Latent heat h_lv={h_lv} J/kg invalid at p={p}.")
            T_sat = st_l.T
            self._sat_cache[p] = (st_l, st_v, h_lv, T_sat)
        return self._sat_cache[p]

    def _default_single_phase_htc(self, T: float, p: float, m_flow: float, med_prop, d_h: float) -> float:
        """
        Simple single-phase Dittus–Boelter fallback used when no external model is provided.
        Evaluates properties at (p, T). Does not switch to a laminar correlation.
        """
        state = med_prop.calc_state("PT", p, T)
        tp = med_prop.calc_transport_properties(state)

        mu = tp.dyn_vis
        k = tp.lam
        Pr = tp.Pr

        A_flow = math.pi * (d_h ** 2) / 4.0
        G = m_flow / A_flow
        Re = G * d_h / mu

        h = 0.023 * (Re ** 0.8) * (Pr ** 0.4) * (k / d_h)
        return max(h, 100.0)

    def _solve_qflux_two_phase(
        self, x: float, p: float, m_flow: float, med_prop, dT: float
    ) -> Tuple[float, bool, int]:
        """
        Solve the fixed point q'' = h_tp(x, q'') * dT using under-relaxed iteration.

        Update scheme:
            q_{k+1} = (1 - ω) * q_k + ω * (h_tp(x, q_k) * dT)
        with a constant relaxation factor ω in (0,1).

        Returns
        -------
        (q_flux, converged, iterations)
        """
        if dT <= 0.0:
            return 0.0, True, 0

        # Initial guess based on liquid-only convection, scaled for two-phase
        G = self._mass_flux(m_flow)
        st_l, st_v, h_lv, T_sat = self._get_saturation_states(p, med_prop)
        tp_l = med_prop.calc_transport_properties(st_l)
        mu_l, k_l, Pr_l = tp_l.dyn_vis, tp_l.lam, tp_l.Pr
        Re_l = G * self.d_h / mu_l
        h_lo = 0.023 * (Re_l ** 0.8) * (Pr_l ** 0.4) * (k_l / self.d_h)
        q_guess = max(100.0, 3.0 * h_lo * dT)

        omega = 0.4  # under-relaxation factor

        for it in range(1, self.max_iter + 1):
            try:
                h_tp = float(self.two_phase_htc(
                    x=x,
                    q_flux=q_guess,
                    p=p,
                    m_flow=m_flow,
                    med_prop=med_prop,
                    d_h=self.d_h,
                    orientation=self.orientation,
                ))
                q_new = h_tp * dT

                # Convergence test
                if abs(q_new - q_guess) <= self.tol * max(1.0, abs(q_new)):
                    return float(q_new), True, it

                # Under-relaxed update
                q_guess = (1.0 - omega) * q_guess + omega * q_new

                # Hard bounds to avoid runaway values
                q_guess = max(1.0, min(q_guess, 1e7))

            except Exception as err:
                warnings.warn(f"Two-phase HTC evaluation failed: {err}")
                return float(q_guess), False, it

        warnings.warn(f"Two-phase q'' fixed point did not converge in {self.max_iter} iterations.")
        return float(q_guess), False, self.max_iter

    def _advance_energy(
        self, h_old: float, Q: float, m_flow: float, p: float, med_prop
    ) -> Tuple[float, float, str, float]:
        """
        Advance the thermodynamic state using the energy balance:
            h_new = h_old + Q / m_flow

        Returns (h_new, x_new, regime_new, T_new). No artificial clamping is applied.
        """
        h_new = h_old + Q / m_flow
        state_new = med_prop.calc_state("PH", p, h_new)
        x_new = float(state_new.q)
        T_new = float(state_new.T)

        if x_new < 0.0:
            regime = "subcooled_liquid"
        elif x_new <= 1.0:
            regime = "two_phase"
        else:
            regime = "superheated_vapor"

        return h_new, x_new, regime, T_new

    # ------------------------------- simulation -------------------------------

    def simulate(
        self,
        p: float,         # [Pa]
        h_in: float,      # [J/kg]
        m_flow: float,    # [kg/s]
        med_prop,         # media property wrapper
    ) -> List[SegmentResult]:
        """
        March through the evaporator segments and return per-segment results.
        The method has no side effects (no printing/logging).
        """
        if p <= 0.0 or m_flow <= 0.0:
            raise ValueError("p and m_flow must be positive.")

        # Clear saturation cache for a fresh run
        self._sat_cache.clear()

        # Inlet state
        st_in = med_prop.calc_state("PH", p, h_in)
        x = float(st_in.q)
        h = float(h_in)
        T = float(st_in.T)

        # Early diagnostic: warn if latent heat is very small (near-critical behavior)
        st_l0, st_v0, h_lv0, T_sat0 = self._get_saturation_states(p, med_prop)
        if h_lv0 < 1.0e2:  # 100 J/kg is extremely small for typical refrigerants away from the critical point
            warnings.warn(
                f"Very small latent heat h_lv={h_lv0:.1f} J/kg at p={p:.0f} Pa. "
                "Near-critical behavior likely; two-phase assumptions may degrade."
            )

        results: List[SegmentResult] = []

        for i in range(1, self.n_cells + 1):
            st_l, st_v, h_lv, T_sat = self._get_saturation_states(p, med_prop)

            # Regime classification by quality
            if x < 0.0:
                regime_now = "subcooled_liquid"
            elif x <= 1.0:
                regime_now = "two_phase"
            else:
                regime_now = "superheated_vapor"

            converged = True
            iterations = 0

            if regime_now == "two_phase":
                dT = self.T_w - T_sat
                qpp, converged, iterations = self._solve_qflux_two_phase(x, p, m_flow, med_prop, dT)
                htc = (qpp / dT) if (dT > 0.0 and qpp > 0.0) else 0.0
            else:
                dT = self.T_w - T
                if dT > 0.0:
                    try:
                        htc = float(self.single_phase_htc(T, p, m_flow, med_prop, self.d_h))
                    except Exception as err:
                        warnings.warn(f"Single-phase HTC failed in segment {i}: {err}")
                        htc = self._default_single_phase_htc(T, p, m_flow, med_prop, self.d_h)
                    qpp = htc * dT
                else:
                    htc = 0.0
                    qpp = 0.0

            Q = qpp * self.A_cell

            # Advance state by energy balance
            h_out, x_out, regime_out, T_out = self._advance_energy(h, Q, m_flow, p, med_prop)

            # Store result
            results.append(
                SegmentResult(
                    segment_id=i,
                    regime=regime_now,
                    x_inlet=x,
                    x_outlet=x_out,
                    h_inlet=h,
                    h_outlet=h_out,
                    T_sat=T_sat if regime_now == "two_phase" else None,
                    delta_T=dT,
                    q_flux=qpp,
                    htc=htc,
                    Q_segment=Q,
                    converged=converged,
                    iterations=iterations,
                )
            )

            # Prepare next segment
            x, h, T = x_out, h_out, T_out

        return results

    # ------------------------------- summarization -------------------------------

    @staticmethod
    def summarize(results: List[SegmentResult]) -> Dict:
        """
        Produce a compact summary dictionary from per-segment results.
        """
        if not results:
            return {"error": "no results"}

        total_Q = sum(r.Q_segment for r in results)
        tp = [r for r in results if r.regime == "two_phase"]
        conv_rate = 100.0 * (sum(1 for r in tp if r.converged) / len(tp)) if tp else 100.0

        return {
            "segments": len(results),
            "total_heat_W": total_Q,
            "x_in": results[0].x_inlet,
            "x_out": results[-1].x_outlet,
            "h_in_J_per_kg": results[0].h_inlet,
            "h_out_J_per_kg": results[-1].h_outlet,
            "regime_distribution": {
                "subcooled_liquid": sum(1 for r in results if r.regime == "subcooled_liquid"),
                "two_phase": sum(1 for r in results if r.regime == "two_phase"),
                "superheated_vapor": sum(1 for r in results if r.regime == "superheated_vapor"),
            },
            "two_phase_convergence_rate_percent": conv_rate,
            "avg_htc_W_m2K": sum(r.htc for r in results) / len(results),
            "min_htc_W_m2K": min(r.htc for r in results),
            "max_htc_W_m2K": max(r.htc for r in results),
        }