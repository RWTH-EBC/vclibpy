from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class RotaryCompressor(Compressor):
    """
     Reciprocating compressor semi-empirical model.

    Sources:
    --------
    [1] Giuffrida, Antonio; 2016, A semi-empirical method for assessing the performance of an open-drive screw
    refrigeration compressor. In: Applied Thermal Engineering 93 (2016) 813–823, DOI:https://doi.org/10.1016/j.applthermaleng.2015.10.023


    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the volumetric efficiency.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the isentropic efficiency.

        get_eta_mech(inputs: Inputs) -> float:
            Returns the mechanical efficiency.

    """

    def __init__(
            self,
            m_flow_nom: str,
            a_tl_1: str,
            a_tl_2: str,
            A_leak: str,
            AU_su_nom: str,
            AU_ex_nom: str,
            b_hl: str,
            BVR: str,
            V_sw: str
    ):
        """

        :param m_flow_nom:
        :param a_tl_1:
        :param a_tl_2:
        :param A_leak:
        :param AU_su_nom:
        :param AU_ex_nom:
        :param b_hl:
        :param BVR:
        :param V_sw:
        """

        super().__init__()
        self.m_flow_nom = m_flow_nom
        self.a_tl_1 = a_tl_1
        self.a_tl_2 = a_tl_2
        self.A_leak = A_leak
        self.AU_su_nom = AU_su_nom
        self.AU_ex_nom = AU_ex_nom
        self.b_hl = b_hl
        self.BVR = BVR
        self.V_sw = V_sw

    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Returns the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Volumetric efficiency.
        """




    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the isentropic efficiency.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Isentropic efficiency.
        """


    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the mechanical efficiency.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Mechanical efficiency.
        """
