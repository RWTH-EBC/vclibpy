import numpy as np

from vclibpy.flowsheets.vapor_injection import BaseVaporInjection
from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU
from vclibpy.components.heat_exchangers import ntu


class VaporInjectionEconomizerDownstream(BaseVaporInjection):
    """
    Cycle with vapor injection using an economizer (DOWN-STREAM configuration).

    For this cycle, we have 9 relevant states:

    MAIN LOOP (same as upstream):
    - 1: Before compressor, after evaporator
    - 2: Before condenser, after compressor
    - 3: Before economizer (primary inlet), after condenser
    - 7: After economizer (primary), before low-pressure expansion valve
    - 4: Before evaporator, after low-pressure expansion valve

    INJECTION LOOP (DOWN-STREAM):
    - 3 → Economizer primary → 7 → High-pressure valve → 5
    - 5 → Economizer secondary → 6 (sat. vapour)
    - 6 → High-pressure compressor (injection vapour)

    Additional Assumptions:
    -----------------------
    - No heat losses in economizer
    - No pressure loss in economizer
    - No losses for splitting of streams
    - Isenthalpic high-pressure EV
    """

    flowsheet_name = "VaporInjectionEconomizerDownstream"

    def __init__(self, economizer: VaporInjectionEconomizerNTU, **kwargs):
        self.economizer = economizer
        super().__init__(**kwargs)

    def get_all_components(self):
        return super().get_all_components() + [self.economizer]

    def get_hp_valve_inlet_state(self):
        return getattr(self.economizer, "state_outlet", None) or self.condenser.state_outlet

    def calc_injection(self):
        """
        Same logic as upstream:
        Iterate injected vapour fraction x_vi so that
        economizer has exactly enough capacity to fully vaporize
        the injected stream → state 5 → state 6 must have q = 1.
        """

        # PRIMARY INLET = condenser outlet (state 3)
        self.economizer.state_inlet = self.condenser.state_outlet

        # SECONDARY INLET = HP valve outlet (state 5)
        self.economizer.state_two_phase_inlet = self.high_pressure_valve.state_outlet

        # SECONDARY OUTLET = saturated vapour (state 6)
        self.economizer.state_two_phase_outlet = self.med_prop.calc_state(
            "PQ",
            self.high_pressure_valve.state_outlet.p,
            1
        )

        m_flow_evap = self.evaporator.m_flow

        # Needed enthalpy rise for VI vapour generation (5 → 6)
        dh_ihe_goal = (
            self.economizer.state_two_phase_outlet.h -
            self.economizer.state_two_phase_inlet.h
        )

        # Transport properties (liquid on primary side)
        props_liquid = self.med_prop.calc_transport_properties(
            self.economizer.state_inlet
        )
        alpha_liquid = self.economizer.calc_alpha_liquid(props_liquid)

        # Transport properties (two-phase on secondary side)
        props_two_phase = self.med_prop.calc_mean_transport_properties(
            self.economizer.state_two_phase_inlet,
            self.economizer.state_two_phase_outlet
        )
        alpha_two_phase = self.economizer.calc_alpha_liquid(props_two_phase)

        # Effective cp on secondary side (for NTU method)
        dT_secondary = (
            self.economizer.state_two_phase_outlet.T -
            self.economizer.state_two_phase_inlet.T
        )
        if dT_secondary == 0:
            cp_4 = np.inf
        else:
            cp_4 = dh_ihe_goal / dT_secondary

        self.economizer.set_secondary_cp(cp=cp_4)
        primary_cp = props_liquid.cp

        # Iterate x_vi until Q_real ≈ Q_goal
        _x_vi_step = 0.1
        _x_vi_min_step = 0.0001

        x_vi_next = _x_vi_min_step
        while True:
            x_vi = x_vi_next
            x_eva = 1 - x_vi

            # mass flow relations
            m_flow_vi = (x_vi / (1 - x_vi)) * m_flow_evap
            Q_flow_goal = dh_ihe_goal * m_flow_vi

            self.economizer.m_flow = x_eva * m_flow_evap
            self.economizer.m_flow_secondary = m_flow_vi

            # Overall heat transfer coefficient
            k = self.economizer.calc_k(alpha_liquid, alpha_two_phase)

            Q_flow = ntu.calc_Q_ntu(
                k=k,
                dT_max=(
                    self.economizer.state_inlet.T -
                    self.economizer.state_two_phase_inlet.T
                ),
                A=self.economizer.A,
                flow_type=self.economizer.flow_type,
                m_flow_primary_cp=self.economizer.m_flow * primary_cp,
                m_flow_secondary_cp=self.economizer.m_flow_secondary_cp
            )

            if Q_flow > Q_flow_goal:
                if _x_vi_step <= _x_vi_min_step:
                    break
                x_vi_next = x_vi + _x_vi_step
            else:
                x_vi_next = x_vi - 0.9 * _x_vi_step
                _x_vi_step /= 10

        # Energy balance for primary outlet (state 7)
        h_7 = self.economizer.state_inlet.h - dh_ihe_goal * self.economizer.m_flow
        state_7 = self.med_prop.calc_state("PH", self.economizer.state_inlet.p, h_7)
        self.economizer.state_outlet = state_7

        return x_vi, self.economizer.state_two_phase_outlet.h, state_7

    def get_states_in_order_for_plotting(self):
        """
        Relevant states for DOWN-STREAM VI-Economizer
        for drawing p-h / T-s diagrams.

        MAIN LOOP:
            4 → 1 → 2 → 3 → 7 → 4

        INJECTION LOOP:
            7 → 5 → 6 → (compressor)
        """
        return [

            # --- MAIN LOOP ---
            self.low_pressure_valve.state_inlet,         # 7
            self.low_pressure_valve.state_outlet,        # 4
            self.evaporator.state_inlet,                 # 4
            self.med_prop.calc_state(
                "PQ", self.evaporator.state_inlet.p, 1
            ),
            self.evaporator.state_outlet,                # 1
            self.low_pressure_compressor.state_inlet,    # 1
            self.low_pressure_compressor.state_outlet,   # 1_VI
            self.high_pressure_compressor.state_inlet,   # 1_VI_mixed
            self.high_pressure_compressor.state_outlet,  # 2
            self.condenser.state_inlet,                 # 2
            self.med_prop.calc_state(
                "PQ", self.condenser.state_inlet.p, 1
            ),
            self.med_prop.calc_state(
                "PQ", self.condenser.state_inlet.p, 0
            ),
            self.condenser.state_outlet,                # 3

            # --- DOWN-STREAM INJECTION LOOP ---
            self.economizer.state_inlet,                # 3
            self.economizer.state_outlet,               # 7
            self.high_pressure_valve.state_inlet,       # 7
            self.high_pressure_valve.state_outlet,      # 5
            self.economizer.state_two_phase_inlet,      # 5
            self.economizer.state_two_phase_outlet,     # 6
        ]
