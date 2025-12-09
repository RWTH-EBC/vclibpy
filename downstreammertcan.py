import numpy as np

from vclibpy.components.heat_exchangers import ntu
from vclibpy.media import ThermodynamicState

from vclibpy.flowsheets.vapor_injection_economizer import VaporInjectionEconomizer


class VaporInjectionEconomizerDownstream(VaporInjectionEconomizer):
    """
    Cycle with vapor injection using an economizer (DOWNSTREAM configuration).

    Topology (conceptual):
    ----------------------
    - Primary side:
        3 (condenser outlet) → economizer primary → 7 (economizer outlet)

    - From state 7, the flow is split:
        • (1 - x_vi) → low-pressure expansion valve → evaporator → 1
        • x_vi       → high-pressure expansion valve → 5 → economizer secondary → 6 → injection line

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "VaporInjectionEconomizerDownstream"

    # ------------------------------------------------------------------
    # For downstream, HP-valve inlet should be at IHX/economizer outlet
    # (state 7). BaseVaporInjection must call this instead of hardcoding
    # condenser.state_outlet.
    # ------------------------------------------------------------------
    def get_hp_valve_inlet_state(self) -> ThermodynamicState:
        return self.evaporator.state_outlet   # state 7

    def calc_injection(self):
        """
        This calculation assumes that the heat transfer
        of the higher temperature liquid is always in the subcooling
        region, while the vapor injection is always a two-phase heat
        transfer.
        In reality, you would need to achieve a certain degree of superheat.
        Thus, a moving boundary approach would be more fitting. For simplicity,
        we assume no superheat.

        Downstream-specific mass flow picture:
        --------------------------------------
        Let m_eva   = evaporator mass flow (low-pressure compressor mass flow)
        Let x_vi    = injection mass fraction (m_inj / m_high)
        Then:
            m_inj   = x_vi / (1 - x_vi) * m_eva
            m_high  = m_eva + m_inj = m_eva / (1 - x_vi)

        For downstream:
            - The FULL high-side mass flow (m_high) passes the economizer primary.
            - Only m_inj passes the economizer secondary.
        """

        # Primary inlet is still condenser outlet (state 3)
        self.economizer.state_inlet = self.condenser.state_outlet

        # Secondary two-phase inlet: HP-valve outlet (state 5)
        self.economizer.state_two_phase_inlet = self.high_pressure_valve.state_outlet

        # Secondary outlet: saturated vapor (state 6, q = 1)
        self.economizer.state_two_phase_outlet = self.med_prop.calc_state(
            "PQ", self.high_pressure_valve.state_outlet.p, 1
        )

        m_flow_evaporator = self.evaporator.m_flow

        # Required enthalpy rise for complete vaporization on secondary side
        dh_ihe_goal = (
            self.economizer.state_two_phase_outlet.h -
            self.economizer.state_two_phase_inlet.h
        )

        # -----------------------------
        # Transport properties:
        # -----------------------------
        tra_properties_liquid = self.med_prop.calc_transport_properties(
            self.economizer.state_inlet
        )
        alpha_liquid = self.economizer.calc_alpha_liquid(tra_properties_liquid)

        tra_properties_two_phase = self.med_prop.calc_mean_transport_properties(
            self.economizer.state_two_phase_inlet,
            self.economizer.state_two_phase_outlet
        )
        alpha_two_phase = self.economizer.calc_alpha_liquid(tra_properties_two_phase)

        # Effective cp on secondary side (two-phase)
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

        # ---------------------------------------
        # Iterate injection mass fraction x_vi
        # ---------------------------------------
        _x_vi_step = 0.1
        _min_step_x_vi = 0.0001

        x_vi_next = _min_step_x_vi  # Don't start with zero!
        while True:
            x_vi = x_vi_next

            # Mass flows based on downstream picture:
            # m_inj  = x_vi / (1 - x_vi) * m_eva
            # m_high = m_eva + m_inj = m_eva / (1 - x_vi)
            m_flow_vapor_injection = (x_vi / (1 - x_vi)) * m_flow_evaporator
            m_flow_high = m_flow_evaporator + m_flow_vapor_injection

            Q_flow_goal = dh_ihe_goal * m_flow_vapor_injection

            # For downstream, the economizer primary sees the full high-side mass flow
            self.economizer.m_flow = m_flow_high
            self.economizer.m_flow_secondary = m_flow_vapor_injection

            # This dT_max is always valid, as the primary inlet is cooled
            # and the secondary inlet (the vapor) is either heated
            # or isothermal for pure fluids
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
                # Heat flow that can be transferred > heat flow that is transferred at current step
                if _x_vi_step <= _min_step_x_vi:
                    break
                # We can increase x_vi_next further, as more heat can be extracted
                x_vi_next = x_vi + _x_vi_step
            else:
                # When heat flow at current step is too high, step size is reduced
                # to not bounce back too far
                x_vi_next = x_vi - _x_vi_step * 0.9
                _x_vi_step /= 10

        # ---------------------------------------
        # Solve primary side energy balance
        # → state 7 (economizer outlet)
        # ---------------------------------------
        h_7 = self.economizer.state_inlet.h - dh_ihe_goal * self.economizer.m_flow
        state_7_ihx = self.med_prop.calc_state(
            "PH", self.economizer.state_inlet.p, h_7
        )
        self.economizer.state_outlet = state_7_ihx

        # Return:
        #   x_vi                        : injection mass fraction
        #   self.economizer.state_two_phase_outlet.h : enthalpy of injected vapor (state 6)
        #   state_7_ihx                 : inlet state of low-pressure expansion valve (state 7)
        return x_vi, self.economizer.state_two_phase_outlet.h, state_7_ihx

    def get_states_in_order_for_plotting(self):
        """
        Downstream-specific states to append to the base two-stage cycle:

        - economizer.state_outlet           (state 7)
        - high_pressure_valve.state_inlet   (state 7, path to HP EV)
        - high_pressure_valve.state_outlet  (state 5)
        - economizer.state_two_phase_outlet (state 6)
        """
        return super().get_states_in_order_for_plotting() + [
            self.economizer.state_outlet,             # state 7
            self.high_pressure_valve.state_inlet,     # state 7 (path to HP EV)
            self.high_pressure_valve.state_outlet,    # state 5
            self.economizer.state_two_phase_outlet    # state 6
        ]
