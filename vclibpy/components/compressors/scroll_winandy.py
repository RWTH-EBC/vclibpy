from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class ScrollCompressorWinandy(Compressor):
    """
    Compressor model based on the Paper from [1] Winandy et al., 2002, https://doi.org/10.1016/S1359-4311(01)00083-7.
    This model may not be used for vapor injection cycles, since the state calculations of the compressor are not
    conform with the calculation of the isentropic efficiency in this model.

    Compressor modeling used in [2]:

    [2] Mateu Royo, 2021, Advanced high temperature heat pump configurations using low GWP refrigerants for industrial
        waste heat recovery: A comprehensive study
        (https://www.sciencedirect.com/science/article/pii/S0196890420312759).


    Additional Literature:
    [3] Cuevas et al., 2009, http://dx.doi.org/10.1016/j.applthermaleng.2009.11.005
    [4] Tello-Oquendo et al., 2019, https://doi.org/10.1016/j.ijrefrig.2019.06.031

    This compressor is characterized by using regressions derived from experimental data for scroll compressors.
    For further details on the model, please refer to the original publications.

    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        v_ratio (float): Built in volume ratio of the compressor (is 2.45 in the work of Mateu-Royo [2]).

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
                 v_ratio: float = 2.45,
                 eta_mech: float = 1):
        """
        Initialize the ScrollCompressorWinandy.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
            v_ratio (float): Built in volume ratio of the compressor (is 2.45 in the work of Mateu-Royo [2]).
        """
        super().__init__(N_max=N_max, V_h=V_h)
        self.eta_mech = eta_mech # assumed constant
        self.v_ratio = v_ratio

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
        eta_vol = 1.0 - 0.0184 * pi - 0.0011 * pi ** 2 # Höges used 1.0 instead of 1.0455 (otherwise eta_vol > 1 possible)
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
        s_outlet = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s)
        v_ad = self.state_inlet.v / self.v_ratio
        d_ad = 1 / v_ad
        state_ad = self.med_prop.calc_state("DS", d_ad, self.state_inlet.s)
        eta_is = (s_outlet.h - self.state_inlet.h) / max(1e-5, state_ad.h - self.state_inlet.h + state_ad.v * (p_outlet - state_ad.p))
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