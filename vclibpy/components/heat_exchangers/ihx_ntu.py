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
        # ---------------------------------------------------------
        # DEBUG START A: IST DER MASSENSTROM STABIL?
        # ---------------------------------------------------------
        T_high_in = self.state_inlet_high.T
        # Wir holen uns auch den echten Eingangswert der Low-Side (vom Verdampfer kommend)
        # Hinweis: state_outlet_low ist im Code oft der Startpunkt für Rückrechnungen,
        # aber physikalisch kommt das Fluid vom 'inlet_low' (was wir hier meist nicht direkt als state haben,
        # aber wir sehen es am Massenstrom und den Werten).

        print(f"\n=== [DEBUG IHX START] ===")
        print(f"Massenstrom High: {self.m_flow_high:.6f} kg/s")
        print(f"Input High: {T_high_in:.2f}°C, {self.state_inlet_high.p / 1e5:.2f} bar")
        print(f"Input Low (Ref Out): {self.state_outlet_low.T:.2f}°C, {self.state_outlet_low.p / 1e5:.2f} bar")

        if self.m_flow_high < 0.001:  # Grenzwert z.B. 1 Gramm/Sekunde
            print("!!! ALARM: Massenstrom ist immer noch fast NULL. Ventil-Fixierung prüfen! !!!")
        else:
            print(">>> Massenstrom ist im gesunden Bereich.")
        # ---------------------------------------------------------
        # DEBUG ENDE A
        # ---------------------------------------------------------
        # First calculate the heat assuming q1 at inlet of lower side:
        state_low_q1 = self.med_prop.calc_state("PQ", self.state_outlet_low.p, 1)
        state_high_q0 = self.med_prop.calc_state("PQ", self.state_inlet_high.p, 0)

        dh_max_high = self.state_inlet_high.h - state_high_q0.h
        dh_max_low = self.state_outlet_low.h - state_low_q1.h
        if dh_max_low < 0:
            raise ValueError("This heat exchanger only allows low-pressure outlet states with superheat")

        # In case dh_max_high < 0, high side is already in subcooling,
        # no need for first regime in Option 1 (see svg)
        Q_high_tp_to_q0 = self.m_flow_high * max(0, dh_max_high)
        Q_low_sh_to_q1 = self.m_flow_low * dh_max_low

        # First part of the HX:
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
        if Q_ntu_first_regime < Q_first_regime:
            # Area is only sufficient for first regime.
            self.set_missing_states(Q_ntu_first_regime)
            return
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

        # ... (Original Code bis zur Berechnung von state_high_first_regime)

        # ---------------------------------------------------------
        # DEBUG START B: ZWISCHENSTAND NACH REGIME 1
        # ---------------------------------------------------------
        T_high_R1 = state_high_first_regime.T
        T_low_sat = state_low_q1.T  # Sättigungstemperatur

        print(f"--- Regime 1 Report ---")
        print(f"Q übertragen: {Q_ntu_first_regime:.1f} W")
        #print(f"High-Side Zustand nach R1: T={T_high_R1:.2f}°C, x={state_high_first_regime.x}") #.x ist nicht die richtige Systax
        print(f"High-Side T nach R1: {T_high_R1:.2f} K ({T_high_R1 - 273.15:.2f}°C)")
        print(f"Low-Side Saturation: {T_low_sat:.2f} K ({T_low_sat - 273.15:.2f}°C)")

        # Check: Ist High-Side schon kälter als Low-Side Sättigung?
        if T_high_R1 < T_low_sat:
            print(f"!!! CROSSOVER in R1: High ({T_high_R1:.2f}) < Low Sat ({T_low_sat:.2f}) !!!")
        # ---------------------------------------------------------
        # DEBUG ENDE B
        # ---------------------------------------------------------

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
            state_artificial = self.med_prop.calc_state(
                "PT",
                state_high_first_regime.p,
                state_high_first_regime.T - 5  # Some dT to get cp
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
        # DEBUG START C: ENDERGEBNIS REGIME 2
        # ---------------------------------------------------------
        T_high_R2 = state_high_second_regime.T
        h_high_R2 = state_high_second_regime.h
        T_low_sat = state_low_q1.T

        print(f"--- Regime 2 Report ---")
        print(f"Q übertragen: {Q_ntu_second_regime:.1f} W")
        print(f"High-Side T nach R2: {T_high_R2:.2f} K ({T_high_R2-273.15:.2f}°C)")
        print(f"High-Side h nach R2: {h_high_R2:.0f} J/kg")

        pinch = T_high_R2 - T_low_sat
        print(f"Pinch (Abstand HighOut - LowSat): {pinch:.2f} K")
        if pinch < 0:
            print(f"!!! CROSSOVER FEHLER !!! High-Side ist {abs(pinch):.2f} K KÄLTER als Low-Side Saturation.")

        if T_high_R2 < T_low_sat:
            print(f"!!! CROSSOVER FEHLER !!! High-Side ist {abs(pinch):.2f} K KÄLTER als Low-Side.")
            print("Mögliche Ursachen: Zu viel Q abgezogen oder Enthalpie-Crash.")
        else:
            print(">>> Alles OK. Physik wurde eingehalten.")

        print("=========================\n")
        # ---------------------------------------------------------
        # DEBUG ENDE C
        # ---------------------------------------------------------

        if self.A - A_required_first_regime - A_required_second_regime < 0:
            raise ValueError("NTU calculation lead to area above 100 %")
        dT_max_third_regime = state_high_second_regime.T - state_low_q1.T
        Q_ntu_third_regime = ntu.calc_Q_ntu(
            k=self.k,
            A=self.A - A_required_first_regime - A_required_second_regime,
            dT_max=dT_max_third_regime,
            m_flow_primary_cp=m_flow_primary_cp,
            m_flow_secondary_cp=m_flow_secondary_cp,
            flow_type=self.flow_type
        )

        # ... (nach der Berechnung von Q_ntu_third_regime)

        # ---------------------------------------------------------
        # DEBUG START D: REGIME 3 (DER CRASH REPORT)
        # ---------------------------------------------------------
        print(f"--- Regime 3 Report ---")
        # Zeig mir, was hier berechnet wurde
        print(f"Q übertragen (R3): {Q_ntu_third_regime:.1f} W")
        print(f"Rest-Fläche für R3: {A_required_third_regime:.4f} m²")

        # Welches dT treibt Regime 3 an?
        dT_R3_max = state_high_second_regime.T - state_low_q1.T
        print(f"Treibendes dT (HighIn - LowSat): {dT_R3_max:.2f} K")

        # Gesamtsumme
        Q_total = Q_ntu_first_regime + Q_ntu_second_regime + Q_ntu_third_regime
        print(f"=== GESAMT Q: {Q_total:.1f} W ===")

        # Manuelle Berechnung des Endzustands (High Side)
        h_out_high_check = self.state_inlet_high.h - Q_total / self.m_flow_high
        print(f"Enthalpie High-Out (berechnet): {h_out_high_check:.0f} J/kg")

        try:
            # Versuch Rückrechnung auf Temperatur
            state_check = self.med_prop.calc_state("PH", self.state_inlet_high.p, h_out_high_check)
            print(f"-> Resultierende Temperatur T5: {state_check.T:.2f} K ({state_check.T - 273.15:.2f}°C)")
        except:
            print("-> Resultierende Temperatur: CRASH (Enthalpie zu niedrig für Stoffdaten)")

        print("-----------------------------------------------------------")
        # ---------------------------------------------------------

        self.set_missing_states(Q=Q_ntu_first_regime + Q_ntu_second_regime + Q_ntu_third_regime)
        return None, None  # Irrelevant for this heat exchanger for now.

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
