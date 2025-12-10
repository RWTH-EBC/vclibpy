from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class PiCorrelationCompressor(Compressor):
    """
    Compressor model based on the dissertation from [1] Christoph Höges, 2025, https://publications.rwth-aachen.de/record/1018275.

    Correlations depending on the pressure ratio for the isentropic and volumetric efficiency are used.

    Correlation for isentropic efficiency from [2] Xu et al., 2021, https://www.sciencedirect.com/science/article/pii/S0360544220323392
    Correlation for volumetric efficiency from [3] Bai et al., 2019, https://www.sciencedirect.com/science/article/pii/S0360544219308308?via%3Dihub

    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the volumetric efficiency based on the data from Winandy et al. [1].

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the isentropic efficiency based on the data from Winandy et al. [1].

        get_eta_mech(inputs: Inputs) -> float:
            Returns the mechanical efficiency (assumed constant).

    """

    def __init__(self,
                 N_max: float, V_h: float,
                 eta_mech: float = 1):
        """
        Initialize the PiCorrelationCompressor.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
        """
        super().__init__(N_max=N_max, V_h=V_h)
        self.eta_mech = eta_mech # assumed constant

    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Get the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Volumetric efficiency.
        """
        p_outlet = self.get_p_outlet()
        pi = p_outlet / self.state_inlet.p
        eta_vol = 0.9 - 0.035 * pi
        return eta_vol

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Get the isentropic efficiency.

        Args:
            p_outlet (float): Outlet pressure of the compressor in Pa.
            inputs (Inputs): Input parameters.

        Returns:
            float: Isentropic efficiency.
        """
        pi = p_outlet / self.state_inlet.p
        eta_is = (35.23 * pi ** 3 - 108.5 * pi ** 2 + 141.2 * pi - 70.85) / max(1e-5, pi ** 4 + 54.71 * pi ** 3 - 189.9 * pi ** 2 + 263.7 * pi - 132.5)
        return eta_is

    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Get the mechanical efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Mechanical efficiency.
        """
        return self.eta_mech