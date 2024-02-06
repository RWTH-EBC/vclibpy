from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class ScrewBitzer(Compressor):
    """
    Compressor based data from Bitzer-tool for screw compressors:
    Bitzer data from compact screw compressor series. Calculation of efficiencies with polynomials for P_el and m_flow.
    Boundary conditions: T_SH = 10 K, T_SC = 0 K, n_max = 50 Hz, eta_mech = 0.92
    Mean curve for eta_s and lambda_h approximated based on 5 refrigerants (R134a, R290, R1234yf, R1234ze, R407C).

    Inherits from the Compressor class, which defines the basic properties and behavior of a compressor in a vapor
    compression cycle.

    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        eta_isentropic (float): Solely a function of the pressure ratio.
        eta_mech (float): Constant mechanical efficiency of the compressor including motor and inverter efficiencies.
        lambda_h (float): Solely a function of the pressure ratio.

    Args:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        eta_isentropic (float): Solely a function of the pressure ratio.
        eta_inverter (float): Constant inverter efficiency of the compressor.
        eta_motor (float): Constant motor efficiency of the compressor.
        eta_mech (float): Constant mechanical efficiency of the compressor including motor and inverter efficiencies.
        lambda_h (float): Solely a function of the pressure ratio.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the volumetric efficiency of the compressor for the corresponding pressure ratio.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the isentropic efficiency of the compressor for the corresponding pressure ratio.

        get_eta_mech(inputs: Inputs) -> float:
            Returns the constant mechanical efficiency including motor and inverter efficiencies.

    """

    def __init__(self,
                 N_max: float, V_h: float,
                 eta_mech: float):
        """
        Initialize the ConstantEffectivenessCompressor.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
            eta_isentropic (float): Solely a function of the pressure ratio.
            eta_inverter (float): Constant inverter efficiency of the compressor.
            eta_motor (float): Constant motor efficiency of the compressor.
            eta_mech (float): Constant mechanical efficiency of the compressor.
            lambda_h (float): Solely a function of the pressure ratio.
        """
        super(ScrewBitzer, self).__init__(N_max=N_max, V_h=V_h)
        self.eta_mech = eta_mech


    def get_lambda_h(self, inputs: Inputs,
                     a0 = 0.995712787, a1 = -0.021768086) -> float:
        """
        Returns the volumetric efficiency of the compressor for the corresponding pressure ratio.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Volumetric efficiency for corresponding pressure ratio.
        """

        rp = self.state_outlet.p/self.state_inlet.p

        lambda_h = a0 + a1 * rp

        return lambda_h

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs,
                           transition_point = 5.3,
                           a0 = -0.428288911, a1 = 0.905243389, a2 = -0.218644813, a3 = 0.016393308,
                           b0 = 0.821171862, b1 = -0.028999017) -> float:
        """
        Returns the isentropic efficiency of the compressor for the corresponding pressure ratio.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Isentropic efficiency for corresponding pressure ratio.
        """

        rp = p_outlet/self.state_inlet.p

        if rp <= transition_point:
            eta_s = a0 + a1 * rp + a2 * rp**2 + a3 * rp**3
        else:
            eta_s = b0 + b1 * rp

        return eta_s

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
