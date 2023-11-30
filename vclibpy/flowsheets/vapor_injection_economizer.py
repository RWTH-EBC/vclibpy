import numpy as np

from vclibpy.flowsheets.vapor_injection import BaseVaporInjection
from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU


class VaporInjectionEconomizer(BaseVaporInjection):
    """
    Cycle with vapor injection using an economizer.

    For this cycle, we have 9 relevant states:
    1: Before compressor, after evaporator
    2: Before condenser, after compressor
    3: Before ihx, after condenser
    4: Before Evaporator, after ihx
    5_ihx: Before ihx, after condenser and EV
    6_ihx: Before Mixing with 1_VI, after ihx
    7_ihx: Before second EV, after ihx
    1_VI: Before Mixing with 6_ihx, After first stage
    1_VI_mixed: Before second stage of compressor, after mixing with 6_ihx

    Additional Assumptions:
    -----------------------
    - No heat losses in ihx
    - No pressure loss in ihx
    - No losses for splitting of streams
    - Isenthalpic second EV

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "VaporInjectionEconomizer"

    def __init__(self, economizer: VaporInjectionEconomizerNTU, **kwargs):
        self.economizer = economizer
        super().__init__(**kwargs)

    def get_all_components(self):
        return super().get_all_components() + [
            self.economizer
        ]

    def calc_injection(self):
        """
        This calculation assumes that the heat transfer
        of the higher temperature liquid is always in the subcooling
        region, while the vapor injection is always a two-phase heat
        transfer.
        In reality, you would need to achieve a certain degree of superheat.
        Thus, a moving boundary approach would be more fitting. For simplicity,
        we assume no superheat.

        This function iterates the amount of vapor injected.
        The iteration starts with close to no injection and increases
        the amount of injection as long as enough hotter sub-cooled liquid is
        present to fully vaporize the injected part.
        """
        self.economizer.state_inlet = self.condenser.state_outlet
        self.economizer.state_two_phase_inlet = self.high_pressure_valve.state_outlet
        self.economizer.state_two_phase_outlet = self.med_prop.calc_state(
            "PQ", self.high_pressure_valve.state_outlet.p, 1
        )
        m_flow_evaporator = self.evaporator.m_flow

        dh_ihe_goal = (
                self.economizer.state_two_phase_outlet.h -
                self.economizer.state_two_phase_inlet.h
        )

        # Get transport properties:
        tra_properties_liquid = self.med_prop.calc_transport_properties(
            self.economizer.state_inlet
        )
        alpha_liquid = self.economizer.calc_alpha_liquid(tra_properties_liquid)
        tra_properties_two_phase = self.med_prop.calc_mean_transport_properties(
            self.economizer.state_two_phase_inlet,
            self.economizer.state_two_phase_outlet
        )
        alpha_two_phase = self.economizer.calc_alpha_liquid(tra_properties_two_phase)

        # Set cp based on transport properties
        dT_secondary = (
                self.economizer.state_two_phase_outlet.T -
                self.economizer.state_two_phase_inlet.T
        )
        if dT_secondary == 0:
            cp_4 = np.inf
        else:
            cp_4 = dh_ihe_goal / dT_secondary
        self.economizer.set_secondary_cp(cp=cp_4)
        self.economizer.set_primary_cp(cp=tra_properties_liquid.cp)

        # We have to iterate to ensure the correct fraction of mass is
        # used to ensure state5 has q=1
        _x_vi_step = 0.1
        _min_step_x_vi = 0.0001

        x_vi_next = _min_step_x_vi  # Don't start with zero!
        while True:
            x_vi = x_vi_next
            x_eva = 1 - x_vi
            m_flow_vapor_injection = x_vi * m_flow_evaporator
            Q_flow_goal = dh_ihe_goal * m_flow_vapor_injection

            self.economizer.m_flow = x_eva * m_flow_evaporator
            self.economizer.m_flow_secondary = m_flow_vapor_injection

            # This dT_max is always valid, as the primary inlet is cooled
            # and the secondary inlet (the vapor) is either heated
            # or isothermal for pure fluids
            Q_flow, k = self.economizer.calc_Q_ntu(
                dT_max=(
                        self.economizer.state_inlet.T -
                        self.economizer.state_two_phase_inlet.T
                ),
                alpha_pri=alpha_liquid,
                alpha_sec=alpha_two_phase,
                A=self.economizer.A
            )
            if Q_flow > Q_flow_goal:
                if _x_vi_step <= _min_step_x_vi:
                    break
                # We can increase x_vi_next further, as more heat can be extracted
                x_vi_next = x_vi + _x_vi_step
            else:
                x_vi_next = x_vi - _x_vi_step * 0.9
                _x_vi_step /= 10

        # Solve Energy Balance
        h_7 = self.economizer.state_inlet.h - dh_ihe_goal * self.economizer.m_flow
        state_7_ihx = self.med_prop.calc_state("PH", self.economizer.state_inlet.p, h_7)
        self.economizer.state_outlet = state_7_ihx
        return x_vi, self.economizer.state_two_phase_outlet.h, state_7_ihx

    def get_states_in_order_for_plotting(self):
        return super().get_states_in_order_for_plotting() + [
            self.economizer.state_two_phase_inlet,
            self.economizer.state_two_phase_outlet,
            self.high_pressure_compressor.state_inlet,
            # Go back to the condenser outlet
            self.economizer.state_two_phase_outlet,
            self.economizer.state_two_phase_inlet,
            self.high_pressure_valve.state_outlet,
            self.high_pressure_valve.state_inlet,
            self.condenser.state_outlet,
            self.economizer.state_inlet,
            self.economizer.state_outlet
        ]
