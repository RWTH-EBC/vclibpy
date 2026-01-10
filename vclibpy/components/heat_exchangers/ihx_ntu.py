import numpy as np
import logging

from vclibpy.components.heat_exchangers import InternalHeatExchanger, ntu
from vclibpy.components.heat_exchangers.heat_transfer.heat_transfer import HeatTransfer, TwoPhaseHeatTransfer
from vclibpy import Inputs, FlowsheetState

logger = logging.getLogger(__name__)


class IHX_NTU(InternalHeatExchanger):
    """
    Logic for an internal heat exchanger in counter flow arrangement.
    The regime logics are depicted here: docs/source/ihx_logic.svg
    """

    def __init__(
            self,
            alpha_low_side: float,
            alpha_high_side: float,
            dT_pinch_min: float = 0,
            **kwargs):
        super().__init__(**kwargs)
        self.dT_pinch_min = dT_pinch_min
        assert self.flow_type == "counter", "Other types are not implemented"
        self.k = self.calc_k(
            alpha_pri=alpha_high_side,
            alpha_sec=alpha_low_side
        )

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):
        # 1. Vorbereitung und Grenzen bestimmen
        # -------------------------------------
        # Low-Side Referenz (Taupunkt / Sättigung Dampf)
        state_low_q1 = self.med_prop.calc_state("PQ", self.state_outlet_low.p, 1)
        # High-Side Referenz (Siedepunkt / Sättigung Flüssigkeit)
        state_high_q0 = self.med_prop.calc_state("PQ", self.state_inlet_high.p, 0)

        dh_max_high = self.state_inlet_high.h - state_high_q0.h
        dh_max_low = self.state_outlet_low.h - state_low_q1.h

        if dh_max_low < 0:
            raise ValueError("This heat exchanger only allows low-pressure outlet states with superheat")

        # Maximale Teilwärmeströme für Phasenwechsel
        Q_high_tp_to_q0 = self.m_flow_high * max(0, dh_max_high)
        Q_low_sh_to_q1 = self.m_flow_low * dh_max_low

        # ---------------------------------------------------------
        # REGIME 1: Kondensation (High) vs. Gas (Low)
        # ---------------------------------------------------------
        m_flow_primary_cp = (
                                    (self.state_outlet_low.h - state_low_q1.h) /
                                    (self.state_outlet_low.T - state_low_q1.T)
                            ) * self.m_flow_low
        m_flow_secondary_cp = self.m_flow_high * np.inf

        Q_first_regime = min(Q_high_tp_to_q0, Q_low_sh_to_q1)
        dT_max_first_regim = self.state_inlet_high.T - state_low_q1.T

        Q_ntu_first_regime, A_required_first_regime = ntu.calc_Q_with_available_area(
            heat_exchanger=self,
            k=self.k,
            Q_required=Q_first_regime,
            A_available=self.A,
            dT_max=dT_max_first_regim,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp
        )

        # Wenn Fläche nicht mal für R1 reicht, aufhören
        if Q_ntu_first_regime < Q_first_regime:
            self.set_missing_states(Q_ntu_first_regime)
            return

        # Zwischenzustände nach Regime 1 berechnen
        state_low_first_regime = self.med_prop.calc_state(
            "PH",
            self.state_outlet_low.p,
            self.state_outlet_low.h - Q_ntu_first_regime / self.m_flow_low
        )
        state_high_first_regime = self.med_prop.calc_state(
            "PH",
            self.state_inlet_high.p,
            self.state_inlet_high.h - Q_ntu_first_regime / self.m_flow_high
        )

        # ---------------------------------------------------------
        # REGIME 2: Unterkühlung (High) vs. Gas (Low)
        # ---------------------------------------------------------
        # Bestimmen der Grenzen für Regime 2
        if Q_ntu_first_regime < Q_low_sh_to_q1:
            Q_low_first_to_second_regime = self.m_flow_low * (
                    state_low_first_regime.h - state_low_q1.h
            )
        else:
            Q_low_first_to_second_regime = np.inf
            m_flow_primary_cp = np.inf

        if Q_ntu_first_regime < Q_high_tp_to_q0:
            Q_high_first_to_second_regime = self.m_flow_high * (
                    self.state_inlet_high.h - state_high_q0.h
            )
        else:
            Q_high_first_to_second_regime = np.inf
            # cp Berechnung für Flüssigkeit (näherungsweise über 5K Differenz)
            state_artificial = self.med_prop.calc_state(
                "PT",
                state_high_first_regime.p,
                state_high_first_regime.T - 5
            )
            m_flow_secondary_cp = (
                    (state_high_first_regime.h - state_artificial.h) /
                    (state_high_first_regime.T - state_artificial.T)
            )

        Q_second_regime = min(Q_high_first_to_second_regime, Q_low_first_to_second_regime)
        dT_max_second_regime = state_high_first_regime.T - state_low_q1.T

        Q_ntu_second_regime, A_required_second_regime = ntu.calc_Q_with_available_area(
            heat_exchanger=self,
            k=self.k,
            Q_required=Q_second_regime,
            A_available=self.A - A_required_first_regime,
            dT_max=dT_max_second_regime,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp
        )

        state_high_second_regime = self.med_prop.calc_state(
            "PH",
            state_high_first_regime.p,
            state_high_first_regime.h - Q_ntu_second_regime / self.m_flow_high
        )

        # ---------------------------------------------------------
        # REGIME 3: Unterkühlung (High) vs. Nassdampf/Sättigung (Low)
        # ---------------------------------------------------------
        if self.A - A_required_first_regime - A_required_second_regime < 0:
            # Sollte durch calc_Q_with_available_area abgefangen sein,
            # aber sicherheitshalber keine Fläche mehr nutzen.
            A_regime_3 = 0
        else:
            A_regime_3 = self.A - A_required_first_regime - A_required_second_regime

        dT_max_third_regime = state_high_second_regime.T - state_low_q1.T

        Q_ntu_third_regime = ntu.calc_Q_ntu(
            k=self.k,
            A=A_regime_3,
            dT_max=dT_max_third_regime,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp,
            flow_type=self.flow_type
        )

        # ---------------------------------------------------------
        # SAFETY CHECK: Crossover Prevention
        # ---------------------------------------------------------
        # Wir summieren alles auf und prüfen gegen das physikalische Limit.
        Q_total = Q_ntu_first_regime + Q_ntu_second_regime + Q_ntu_third_regime

        # Das Limit ist erreicht, wenn T_high_out == T_low_in (state_outlet_low.T)
        T_limit_cold = self.state_outlet_low.T

        # Nur prüfen, wenn wir kühlen (High > Low)
        if T_limit_cold < self.state_inlet_high.T:
            try:
                # Berechne hypothetische Enthalpie bei T_limit
                state_limit_high = self.med_prop.calc_state("PT", self.state_inlet_high.p, T_limit_cold)

                # Maximale Enthalpiedifferenz
                dh_max_phys = self.state_inlet_high.h - state_limit_high.h

                # Maximaler Wärmestrom (mit 0.1% Sicherheitsabstand zur Vermeidung numerischer Fehler)
                Q_max_phys = self.m_flow_high * dh_max_phys * 0.999

                if Q_total > Q_max_phys:
                    # Skaliere die Wärmeströme proportional herunter
                    factor = Q_max_phys / Q_total
                    Q_ntu_first_regime *= factor
                    Q_ntu_second_regime *= factor
                    Q_ntu_third_regime *= factor
                    Q_total = Q_max_phys
            except Exception:
                # Fallback: Wenn Stoffdatenberechnung fehlschlägt, akzeptieren wir das Risiko
                pass

        # ---------------------------------------------------------
        # Finales Setzen der Zustände
        # ---------------------------------------------------------
        self.set_missing_states(Q=Q_total)
        return None, None

    def set_missing_states(self, Q: float):
        self.state_inlet_low = self.med_prop.calc_state(
            "PH",
            self.state_outlet_low.p,
            self.state_outlet_low.h - Q / self.m_flow_low
        )
        self.state_outlet_high = self.med_prop.calc_state(
            "PH",
            self.state_inlet_high.p,
            self.state_inlet_high.h - Q / self.m_flow_high
        )
