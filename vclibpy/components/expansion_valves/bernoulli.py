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
