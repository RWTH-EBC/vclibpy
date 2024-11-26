"""
Module with simple bernoulli expansion valve
"""
from vclibpy.components.expansion_valves import ExpansionValve


class Bernoulli(ExpansionValve):
    """
    Simple Bernoulli model.

    Args:
        A (float): Cross-sectional area of the expansion valve.
    """

    def calc_m_flow_at_opening(self, opening):
        return opening * self.A * (2 * self.state_inlet.d * (self.state_inlet.p - self.state_outlet.p)) ** 0.5

    def calc_opening_at_m_flow(self, m_flow, **kwargs):
        return (
                m_flow /
                (self.A * (2 * self.state_inlet.d * (self.state_inlet.p - self.state_outlet.p)) ** 0.5)
        )

    def calc_outlet_pressure_at_m_flow_and_opening(self, m_flow, opening):
        p_outlet = (
            self.state_inlet.p - 1 / (2 * self.state_inlet.d) * (m_flow / (self.A * opening)) ** 2
        )
        self.calc_outlet(p_outlet)
