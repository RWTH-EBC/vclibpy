from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class RotaryCompressor(Compressor):
    """
    Compressor model based on the thesis of Mirko Engelpracht.

    This compressor is characterized by using regressions provided by Mirko Engelpracht for a family of rotary
    compressors. The coefficients used in the regressions are sourced from his Master's thesis.

    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the volumetric efficiency based on the regressions of Mirko Engelpracht.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the isentropic efficiency based on the regressions of Mirko Engelpracht.

        get_eta_mech(inputs: Inputs) -> float:
            Returns the mechanical efficiency based on the regressions of Mirko Engelpracht.

    """

    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Returns the volumetric efficiency based on the regressions of Mirko Engelpracht.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Volumetric efficiency.
        """
        p_outlet = self.get_p_outlet()
        # If not constant value is given, eta_is is calculated based on the regression of Mirko Engelpracht
        n = self.get_n_absolute(inputs.n)
        T_1 = self.state_inlet.T

        a_1 = 0.80179
        a_2 = -0.05210
        sigma_pi = 1.63495
        pi_ave = 4.54069
        a_3 = 3.21616e-4
        sigma_T_1 = 8.43797
        T_1_ave = 263.86428
        a_4 = -0.00494
        a_5 = 0.04981
        sigma_n = 20.81378
        n_ave = 64.41071
        a_6 = -0.02190

        pi = p_outlet / self.state_inlet.p
        return (
                a_1 +
                a_2 * (pi - pi_ave) / sigma_pi +
                a_3 * (T_1 - T_1_ave) / sigma_T_1 * (pi - pi_ave) / sigma_pi +
                a_4 * (T_1 - T_1_ave) / sigma_T_1 +
                a_5 * (n - n_ave) / sigma_n +
                a_6 * ((n - n_ave) / sigma_n) ** 2
        )

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the isentropic efficiency based on the regressions of Mirko Engelpracht.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Isentropic efficiency.
        """
        # If not constant value is given, eta_is is calculated based on the regression of Mirko Engelpracht
        n = self.get_n_absolute(inputs.n)

        a_1 = 0.5816
        a_2 = 0.002604
        a_3 = -1.515e-7
        a_4 = -0.00473
        pi = p_outlet / self.state_inlet.p
        eta = (
                a_1 +
                a_2 * n +
                a_3 * n ** 3 +
                a_4 * pi ** 2
        )
        if eta <= 0:
            raise ValueError("Efficiency is lower or equal to 0")
        return eta

    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the mechanical efficiency based on the regressions of Mirko Engelpracht.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Mechanical efficiency.
        """
        p_outlet = self.get_p_outlet()
        n = self.get_n_absolute(inputs.n)
        # If not constant value is given, eta_is is calculated based on the regression of Mirko Engelpracht
        a_00 = 0.2199
        a_10 = -0.0193
        a_01 = 0.02503
        a_11 = 8.817e-5
        a_20 = -0.001345
        a_02 = -0.0003382
        a_21 = 1.584e-5
        a_12 = -1.083e-6
        a_22 = -5.321e-8
        a_03 = 1.976e-6
        a_13 = 4.053e-9
        a_04 = -4.292e-9
        pi = p_outlet / self.state_inlet.p
        return (
                a_00 +
                a_10 * pi + a_01 * n + a_11 * pi * n +
                a_20 * pi ** 2 + a_02 * n ** 2 + a_21 * pi ** 2 * n + a_12 * pi * n ** 2 + a_22 * pi ** 2 * n ** 2 +
                a_03 * n ** 3 + a_13 * pi * n ** 3 +
                a_04 * n ** 4
        )
