import numpy as np

from vclibpy.flowsheets.vapor_injection import BaseVaporInjection
from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU


class VaporInjectionEconomizer(BaseVaporInjection):
    """
    Base class for vapor injection using an economizer.
    This class holds the economizer component but contains
    no injection topology. Upstream / Downstream configurations
    must be implemented in subclasses.

    Notes
    -----
    Subclasses must implement:
        - calc_injection()
        - get_hp_valve_inlet_state()
        - (optional) get_states_in_order_for_plotting()
    """

    flowsheet_name = "VaporInjectionEconomizerBase"

    def __init__(self, economizer: VaporInjectionEconomizerNTU, **kwargs):
        self.economizer = economizer
        super().__init__(**kwargs)

    def get_all_components(self):
        return super().get_all_components() + [
            self.economizer
        ]

    # ------------------------------------------------------------------
    # Injection is topology-dependent → must be implemented downstream.
    # ------------------------------------------------------------------
    def calc_injection(self):
        raise NotImplementedError(
            "calc_injection() must be implemented in a subclass "
            "(e.g., VaporInjectionEconomizerUpstream or "
            "VaporInjectionEconomizerDownstream)."
        )

    # ------------------------------------------------------------------
    # HP EV inlet state is topology-dependent → must be overridden.
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # Base plotting only contains two-stage cycle states.
    # Economizer-specific states must be appended in subclasses.
    # ------------------------------------------------------------------
    def get_states_in_order_for_plotting(self):
        return super().get_states_in_order_for_plotting()
