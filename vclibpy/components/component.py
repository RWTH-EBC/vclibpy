from abc import ABC
from vclibpy.media import ThermodynamicState, MedProp


class BaseComponent(ABC):
    """
    Abstract base class for defining interfaces of components in the vapor compression cycle.

    Methods:
        start_secondary_med_prop():
            To use multiprocessing, MedProp can't start in the main thread, as the object can't be pickled.
            This function starts possible secondary MedProp classes, for instance in heat exchangers.
            The default component does not have to override this function.

    Properties:
        state_inlet (ThermodynamicState):
            Property for accessing and setting the inlet state of the component.
        state_outlet (ThermodynamicState):
            Property for accessing and setting the outlet state of the component.
        m_flow (float):
            Property for accessing and setting the mass flow rate through the component.
        med_prop (MedProp):
            Property for accessing and setting the property wrapper for the working fluid.
    """

    def __init__(self):
        """
        Initialize the BaseComponent.
        """
        self._state_inlet: ThermodynamicState = None
        self._state_outlet: ThermodynamicState = None
        self._m_flow: float = None
        self._med_prop: MedProp = None

    def start_secondary_med_prop(self):
        """
        Start secondary MedProp classes for multiprocessing.

        To use multiprocessing, MedProp can't start in the main thread, as the object can't be pickled.
        This function starts possible secondary MedProp classes, for instance in heat exchangers.
        The default component does not have to override this function.
        """
        pass

    def terminate_secondary_med_prop(self):
        """
        To use multi-processing, MedProp can't start
        in the main thread, as the object can't be pickled.

        This function terminates possible secondary med-prop
        classes, for instance in heat exchangers.
        The default component does not have to override
        this function.
        """
        pass

    @property
    def state_inlet(self) -> ThermodynamicState:
        """
        Get or set the inlet state of the component.

        Returns:
            ThermodynamicState: Inlet state of the component.
        """
        return self._state_inlet

    @state_inlet.setter
    def state_inlet(self, state_inlet: ThermodynamicState):
        """
        Set the inlet state of the component.

        Args:
            state_inlet (ThermodynamicState): Inlet state to set.
        """
        self._state_inlet = state_inlet

    @property
    def state_outlet(self) -> ThermodynamicState:
        """
        Get or set the outlet state of the component.

        Returns:
            ThermodynamicState: Outlet state of the component.
        """
        return self._state_outlet

    @state_outlet.setter
    def state_outlet(self, state_outlet: ThermodynamicState):
        """
        Set the outlet state of the component.

        Args:
            state_outlet (ThermodynamicState): Outlet state to set.
        """
        self._state_outlet = state_outlet

    @property
    def m_flow(self) -> float:
        """
        Get or set the mass flow rate through the component.

        Returns:
            float: Mass flow rate through the component.
        """
        return self._m_flow

    @m_flow.setter
    def m_flow(self, m_flow: float):
        """
        Set the mass flow rate through the component.

        Args:
            m_flow (float): Mass flow rate to set.
        """
        self._m_flow = m_flow

    @property
    def med_prop(self) -> MedProp:
        """
        Get or set the property wrapper for the working fluid.

        Returns:
            MedProp: Property wrapper for the working fluid.
        """
        return self._med_prop

    @med_prop.setter
    def med_prop(self, med_prop: MedProp):
        """
        Set the property wrapper for the working fluid.

        Args:
            med_prop (MedProp): Property wrapper to set.
        """
        self._med_prop = med_prop
