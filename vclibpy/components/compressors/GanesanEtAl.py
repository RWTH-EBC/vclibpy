from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs
import math


class GanesanEtAl(Compressor):
    """
    Compressor based on Mateu-Royo et al. 2020:
    "Advanced high temperature heat pump configurations using low GWP
    refrigerants for industrial waste heat recovery: A comprehensive study"

    Inherits from the Compressor class, which defines the basic properties and behavior of a compressor in a vapor
    compression cycle.

    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        eta_isentropic (float):
        eta_mech (float):
        lambda_h (float):

    Args:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        eta_isentropic (float): Constant isentropic efficiency of the compressor.
        eta_inverter (float): Constant inverter efficiency of the compressor.
        eta_motor (float): Constant motor efficiency of the compressor.
        eta_mech (float): Constant mechanical efficiency of the compressor including motor and inverter efficiencies.
        lambda_h (float): Constant volumetric efficiency.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the constant volumetric efficiency of the compressor.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the constant isentropic efficiency of the compressor.

        get_eta_mech(inputs: Inputs) -> float:
            Returns the constant mechanical efficiency including motor and inverter efficiencies.

    """

    def __init__(self,
                 N_max: float, V_h: float,
                 eta_mech: float,
                 ):
        """
        Initialize the ConstantEffectivenessCompressor.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
            eta_isentropic (float): Constant isentropic efficiency of the compressor.
            eta_inverter (float): Constant inverter efficiency of the compressor.
            eta_motor (float): Constant motor efficiency of the compressor.
            eta_mech (float): Constant mechanical efficiency of the compressor.
            lambda_h (float): Constant volumetric efficiency.
        """
        super(GanesanEtAl, self).__init__(N_max=N_max, V_h=V_h)
        self.eta_mech = eta_mech


    def get_lambda_h(self, p_outlet, inputs: Inputs) -> float:
        """
        Returns the constant volumetric efficiency of the compressor.

        Args:
            p_outlet:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Constant volumetric efficiency.
        """

        if p_outlet is None and self.state_outlet is None:
            print("p_outlet is missing or lambda_h")
            exit()
        if p_outlet is None:
            p_outlet = self.state_outlet.p



        rp = p_outlet / self.state_inlet.p
        eta_vol = 0.0011 * (rp ** 2) + 0.0487 * rp + 0.9979


        return eta_vol

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the constant isentropic efficiency of the compressor.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Constant isentropic efficiency.

        """
        rp = p_outlet / self.state_inlet.p
        eta_is = -0.00000461 * (rp ** 6) + 0.00027131 * (rp ** 5) - 0.00628605 * (rp ** 4) + 0.07370258 * (
                    rp ** 3) - 0.46054399 * (rp ** 2) + 1.40653347 * rp - 0.87811477
        return eta_is

    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the product of the constant mechanical, motor, and inverter efficiencies
        as the effective mechanical efficiency of the compressor.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Effective mechanical efficiency.
        """
        return self.eta_mech
