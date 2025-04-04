"""
Module with a simple phase separator model.
"""

from vclibpy.media import ThermodynamicState
from vclibpy.components.component import BaseComponent


class PhaseSeparator(BaseComponent):
    """
    A simple phase separator model.
    """

    def __init__(self):
        super().__init__()
        self._state_outlet_liquid: ThermodynamicState = None
        self._state_outlet_vapor: ThermodynamicState = None
        self._state_inlet_low: ThermodynamicState = None
        self.massflowratio: float = None

    @BaseComponent.state_inlet.setter
    def state_inlet(self, state_inlet: ThermodynamicState):
        """
        Set the state of the inlet and calculate the outlet states for liquid and vapor phases.

        Args:
            state_inlet (ThermodynamicState): Inlet state.
        """
        self._state_inlet = state_inlet
        self.state_outlet_vapor = self.med_prop.calc_state("PQ", self.state_inlet.p, 1)
        self.state_outlet_liquid = self.med_prop.calc_state("PQ", self.state_inlet.p, 0)


    @BaseComponent.state_outlet.setter
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


    @property
    def state_inlet_low(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet_low

    @state_inlet_low.setter
    def state_inlet_low(self, state_inlet: ThermodynamicState):
        """
        Set the inlet state of the component.

        Args:
            state_inlet (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet_low = state_inlet

    def get_mflow_ratio(self):

        self.massflowratio = ((self.state_outlet_vapor.h - self.state_inlet.h)
                              / max(0.0001, self.state_inlet_low.h-self.state_outlet_liquid.h))
        return self.massflowratio