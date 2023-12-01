import logging

from vclibpy.flowsheets.vapor_injection import BaseVaporInjection
from vclibpy.components.phase_separator import PhaseSeparator


logger = logging.getLogger(__name__)


class VaporInjectionPhaseSeparator(BaseVaporInjection):
    """
    Cycle with vapor injection using an adiabatic ideal phase seperator.

    For this cycle, we have 9 relevant states:

    - 1: Before compressor, after evaporator
    - 2: Before condenser, after compressor
    - 3: Before PS, after condenser
    - 4: Before Evaporator, after PS
    - 5_vips: Before PS, after first EV
    - 6_vips: Before Mixing with 1_VI, after PS
    - 7_vips: Before second EV, after PS
    - 1_VI: Before Mixing with 6_vips, After first stage
    - 1_VI_mixed. Before second stage of compressor, after mixing with 6_vips

    Additional Assumptions:
    -----------------------
    - Ideal mixing in compressor of state 5 and state 4

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "VaporInjectionPhaseSeparator"

    def __init__(self, **kwargs):
        self.phase_separator = PhaseSeparator()
        super().__init__(**kwargs)

    def get_all_components(self):
        return super().get_all_components() + [
            self.phase_separator
        ]

    def calc_injection(self):
        # Phase separator
        self.phase_separator.state_inlet = self.high_pressure_valve.state_outlet
        x_vapor_injection = self.phase_separator.state_inlet.q
        h_vapor_injection = self.phase_separator.state_outlet_vapor.h
        return x_vapor_injection, h_vapor_injection, self.phase_separator.state_outlet_liquid

    def get_states_in_order_for_plotting(self):
        return super().get_states_in_order_for_plotting() + [
            self.phase_separator.state_inlet,
            self.phase_separator.state_outlet_vapor,
            self.high_pressure_compressor.state_inlet,
            # Go back to separator for clear lines
            self.phase_separator.state_outlet_vapor,
            self.phase_separator.state_outlet_liquid
        ]
