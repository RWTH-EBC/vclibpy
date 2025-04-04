import abc
import math
from typing import Tuple
from vclibpy.datamodels import FlowsheetState, Inputs

class BaseHeatExchanger(abc.ABC):
    """
    Class for a heat exchanger.

    Args:
        A (float):
            Area of HE in m^2 for NTU calculation
        secondary_medium (str):
            Name for secondary medium, e.g. `water` or `air`
        wall_heat_transfer (HeatTransfer):
            Model for heat transfer inside wall
        secondary_heat_transfer (HeatTransfer):
            Model for heat transfer from secondary medium to wall
        gas_heat_transfer (HeatTransfer):
            Model for heat transfer from refrigerant gas to wall
        liquid_heat_transfer (HeatTransfer):
            Model for heat transfer from refrigerant liquid to wall
        two_phase_heat_transfer (TwoPhaseHeatTransfer):
            Model for heat transfer from refrigerant two phase to wall
    """

    def __init__(
            self,
            A: float,
    ):
        self.A = A
        self.Q_flow = None
        self.m_flow_warm = None
        self.m_flow_cold = None


    @abc.abstractmethod
    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> Tuple[float, float]:
        """
        Calculate the heat exchanger based on the given inputs.

        The flowsheet state can be used to save important variables
        during calculation for later analysis.

        Both return values are used to check if the heat transfer is valid or not.

        Args:
            inputs (Inputs): The inputs for the calculation.
            fs_state (FlowsheetState): The flowsheet state to save important variables.

        Returns:
            Tuple[float, float]:
                error: Error in percentage between the required and calculated heat flow rates.
                dT_min: Minimal temperature difference (can be negative).
        """
        raise NotImplementedError


class Regmine():

    def __init__(self,
                 T_1_in,
                 T_1_out,
                 T_2_in,
                 T_2_out,
                 U,
                 Q=None,
                 A=None):

        self.T_1_in = T_1_in
        self.T_1_out = T_1_out
        self.T_2_in = T_2_in
        self.T_2_out = T_2_out

        self.dT1 = abs(T_1_in - T_1_out)
        self.dT2 = abs(T_2_in - T_2_out)

        self.dT_in = abs(T_1_in-T_2_in)

        self.dT_out = abs(T_1_out-T_2_out)
        self.dT_pinch = min( self.dT_out, self.dT_in)

        self.U = U
        self.A = A
        self.Q = Q

    def calc_lmtd(self):

        if self.dT_out*self.dT_in < 0:
            return 0.00000001
        return (self.dT_in-self.dT_out)/(math.log((self.dT_in/self.dT_out)))

    def calc_A(self):
        if self.Q is None:
            raise TypeError("Could not calc A without a Q")
        lmtd = self.calc_lmtd()
        return self.Q/(lmtd*self.U)
    def calc_Q(self):
        if self.A is None:
            raise TypeError("Could not calc Q without a A")
        lmtd = self.calc_lmtd()
        return self.A * lmtd * self.U

