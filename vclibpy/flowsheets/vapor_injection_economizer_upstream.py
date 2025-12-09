import numpy as np

from vclibpy.components.heat_exchangers import ntu
from vclibpy.media import ThermodynamicState
from vclibpy.flowsheets.vapor_injection_economizer import VaporInjectionEconomizer


class VaporInjectionEconomizerUpstream(VaporInjectionEconomizer):
    """
    Cycle with vapor injection using an economizer (UPSTREAM configuration).

    Topology:
    ---------
        3 → HP EV → 5 → economizer secondary → 6
        3 → economizer primary → 7 → LP EV → evaporator

    Notes
    -----
    This class keeps the original upstream mass-flow formulation:
        m_inj = x_vi / (1 - x_vi) * m_eva
        m_primary = (1 - x_vi) * m_eva
        m_secondary = m_inj
    """

    flowsheet_name = "VaporInjectionEconomizerUpstream"


    # ------------------------------------------------------------------
    # Injection calculation — UPSTREAM topology
    # ------------------------------------------------------------------
    def calc_injection(self):
        # Primary inlet (state 3)
        self.economizer.state_inlet = self.condenser.state_outlet

        # Secondary inlet = HP EV outlet (state 5)
        self.economizer.state_two_phase_inlet = self.high_pressure_valve.state_outlet

        # Secondary outlet (state 6) = saturated vapor
        self.economizer.state_two_phase_outlet = self.med_prop.calc_state(
            "PQ",
            self.high_pressure_valve.state_outlet.p,
            1
        )

        m_flow_evaporator = self.evaporator.m_flow

        # Enthalpy rise needed for vaporization
        dh_ihe_goal = (
            self.economizer.state_two_phase_outlet.h -
            self.economizer.state_two_phase_inlet.h
        )

        # Transport properties
        tra_properties_liquid = self.med_prop.calc_transport_properties(
            self.economizer.state_inlet
        )
        alpha_liquid = self.economizer.calc_alpha_liquid(tra_properties_liquid)

        tra_properties_two_phase = self.med_prop.calc_mean_transport_properties(
            self.economizer.state_two_phase_inlet,
            self.economizer.state_two_phase_outlet
        )
        alpha_two_phase = self.economizer.calc_alpha_liquid(tra_properties_two_phase)

        # ------------------------------------------------------------------
        # Secondary effective cp (two-phase region) for NTU calculation
        # ------------------------------------------------------------------
        dT_secondary = (
            self.economizer.state_two_phase_outlet.T -
            self.economizer.state_two_phase_inlet.T
        )
        if dT_secondary == 0:
            cp_4 = np.inf
        else:
            cp_4 = dh_ihe_goal / dT_secondary

        self.economizer.set_secondary_cp(cp=cp_4)

        primary_cp = tra_properties_liquid.cp

        # ------------------------------------------------------------------
        # Iterate injection mass fraction x_vi
        # ------------------------------------------------------------------
        _x_vi_step = 0.1
        _min_step_x_vi = 0.0001

        x_vi_next = _min_step_x_vi  # Don't start from zero

        t= 0
        while True:

            t+=1

            x_vi = x_vi_next
            x_eva = 1 - x_vi

            # UPSTREAM mass-flow definitions
            m_flow_vapor_injection = (x_vi / (1 - x_vi)) * m_flow_evaporator
            Q_flow_goal = dh_ihe_goal * m_flow_vapor_injection

            # Primary = (1 - x_vi)*m_eva   , Secondary = m_inj
            self.economizer.m_flow = x_eva * m_flow_evaporator
            self.economizer.m_flow_secondary = m_flow_vapor_injection

            # NTU heat transfer
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
                if _x_vi_step <= _min_step_x_vi:
                    break
                x_vi_next = x_vi + _x_vi_step
            else:
                x_vi_next = x_vi - _x_vi_step * 0.9
                _x_vi_step /= 10

            print("\n------------------- UPSTREAM ECONOMIZER DEBUG -------------------")

            print("ECONOMIZER INLET (state 3):")
            print(f"  T = {self.economizer.state_inlet.T}")
            print(f"  p = {self.economizer.state_inlet.p}")
            print(f"  h = {self.economizer.state_inlet.h}")

            print("\nHP VALVE OUTLET (state 5):")
            print(f"  T = {self.high_pressure_valve.state_outlet.T}")
            print(f"  p = {self.high_pressure_valve.state_outlet.p}")
            print(f"  h = {self.high_pressure_valve.state_outlet.h}")

            print("\nECONOMIZER TWO-PHASE OUTLET (state 6):")
            print(f"  T = {self.economizer.state_two_phase_outlet.T}")
            print(f"  p = {self.economizer.state_two_phase_outlet.p}")
            print(f"  h = {self.economizer.state_two_phase_outlet.h}")

            print("\nMass-flow definitions (UPSTREAM):")
            print(f"  m_flow_evaporator         = {m_flow_evaporator}")
            print(f"  current x_vi              = {x_vi}")
            print(f"  m_flow_vapor_injection    = {m_flow_vapor_injection}")
            print(f"  m_flow_primary (3→7)      = {self.economizer.m_flow}")
            print(f"  m_flow_secondary (5→6)    = {self.economizer.m_flow_secondary}")

            print("\nEnthalpy differences:")
            print(f"  dh_ihe_goal               = {dh_ihe_goal}")
            print(f"  dT_secondary              = {dT_secondary}")
            print(f"  cp_secondary (two-phase)  = {cp_4}")
            print(f"  primary_cp                = {primary_cp}")

            print("\nHeat transfer (NTU):")
            print(f"  k                         = {k}")
            print(f"  A                         = {self.economizer.A}")
            print(f"  flow_type                 = {self.economizer.flow_type}")
            print(
                f"  dT_max                    = {self.economizer.state_inlet.T - self.economizer.state_two_phase_inlet.T}")
            print(f"  Q_flow                    = {Q_flow}")
            print(f"  Q_flow_goal               = {Q_flow_goal}")
            print(f"  |Q_flow - Q_goal|         = {abs(Q_flow - Q_flow_goal)}")

            print("\nIteration parameters:")
            print(f"  step number t             = {t}")
            print(f"  x_vi_next                 = {x_vi_next}")
            print(f"  x_vi_step                 = {_x_vi_step}")

            print("\n---------------------------------------------------------------\n")

        # ------------------------------------------------------------------
        # Primary-side energy balance → state 7
        # ------------------------------------------------------------------
        h_7 = self.economizer.state_inlet.h - dh_ihe_goal * self.economizer.m_flow
        state_7_ihx = self.med_prop.calc_state(
            "PH",
            self.economizer.state_inlet.p,
            h_7
        )

        self.economizer.state_outlet = state_7_ihx

        # Return:
        #   x_vi         → injection ratio
        #   h_6 (state6) → enthalpy of injected vapor
        #   state_7_ihx  → LP EV inlet
        return x_vi, self.economizer.state_two_phase_outlet.h, state_7_ihx

    # ------------------------------------------------------------------
    # Upstream-specific plotting states
    # ------------------------------------------------------------------
    def get_states_in_order_for_plotting(self):
        return[


            self.high_pressure_valve.state_inlet,  # state 7
            self.high_pressure_valve.state_outlet,  # state 7 (path to HP EV)
            self.high_pressure_valve.state_inlet,
            self.condenser.state_outlet,
            self.economizer.state_inlet,  # state 3
            self.economizer.state_outlet,  # state 5
            self.low_pressure_valve.state_inlet,  # state 7  (econ primary outlet → LP valve)
            self.low_pressure_valve.state_outlet,  # state 4  (LP valve outlet → evaporator inlet)
            self.evaporator.state_inlet,  # state 4  (evaporator inlet, identical)
            self.med_prop.calc_state("PQ", self.evaporator.state_outlet.p, 1),  # saturated vapor line in evaporator
            self.evaporator.state_outlet,  # state 1  (evaporator outlet)
            self.low_pressure_compressor.state_inlet,  # state 1
            self.low_pressure_compressor.state_outlet,  # state 1_VI (LP compressor discharge)
            self.low_pressure_valve.state_inlet,
            self.economizer.state_two_phase_outlet,  # state 6
            self.economizer.state_two_phase_inlet,  # state 5
            self.economizer.state_two_phase_outlet,
            self.high_pressure_compressor.state_inlet,
            # Go back to the condenser outlet
            self.economizer.state_two_phase_outlet,  # state 6
            self.economizer.state_two_phase_inlet,
        ]
