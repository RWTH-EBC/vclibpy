"""
Module with a simple phase separator model.
"""

from vclibpy.media import ThermodynamicState
from vclibpy.components.component import TwoPortComponent


class PhaseSeparator(TwoPortComponent):
    """
    A simple phase separator model.
    """

    def __init__(self):
        super().__init__()
        self._state_outlet_liquid: ThermodynamicState = None
        self._state_outlet_vapor: ThermodynamicState = None

    @TwoPortComponent.state_inlet.setter
    def state_inlet(self, state_inlet: ThermodynamicState):
        """
        Set the state of the inlet and calculate the outlet states for liquid and vapor phases.

        Args:
            state_inlet (ThermodynamicState): Inlet state.
        """
        self._state_inlet = state_inlet
        self.state_outlet_vapor = self.med_prop.calc_state("PQ", self.state_inlet.p, 1)
        self.state_outlet_liquid = self.med_prop.calc_state("PQ", self.state_inlet.p, 0)

    @TwoPortComponent.state_outlet.setter
    def state_outlet(self, state: ThermodynamicState):
        """
        This outlet is disabled for this component.

        Args:
            state (ThermodynamicState): Outlet state.
        """
        raise NotImplementedError("This outlet is disabled for this component")

    @property
    def state_outlet_vapor(self) -> ThermodynamicState:
        """
        Getter for the outlet state of the vapor phase.

        Returns:
            ThermodynamicState: Outlet state for the vapor phase.
        """
        return self._state_outlet_vapor

    @state_outlet_vapor.setter
    def state_outlet_vapor(self, state: ThermodynamicState):
        """
        Setter for the outlet state of the vapor phase.

        Args:
            state (ThermodynamicState): Outlet state for the vapor phase.
        """
        self._state_outlet_vapor = state

    @property
    def state_outlet_liquid(self) -> ThermodynamicState:
        """
        Getter for the outlet state of the liquid phase.

        Returns:
            ThermodynamicState: Outlet state for the liquid phase.
        """
        return self._state_outlet_liquid

    @state_outlet_liquid.setter
    def state_outlet_liquid(self, state: ThermodynamicState):
        """
        Setter for the outlet state of the liquid phase.

        Args:
            state (ThermodynamicState): Outlet state for the liquid phase.
        """
        self._state_outlet_liquid = state
