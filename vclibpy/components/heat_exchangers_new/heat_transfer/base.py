import abc
class BaseHeatTransfer(abc.ABC):

    @abc.abstractmethod
    def calc(self, **kwargs) -> float:
        """
        Calculate heat transfer.

        Args:
            transport_properties (TransportProperties): Transport properties of the medium.
            m_flow (float): Mass flow rate.

        Returns:
            float: Calculated heat transfer coefficient.

        Raises:
            NotImplementedError: If the method is not implemented in the subclass.
        """
        raise NotImplementedError