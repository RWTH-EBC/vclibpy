from typing import Dict, Tuple
from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs
from enum import Enum

class Refrigerant(Enum):
    PROPANE = "Propane"
    PROPYLENE = "Propylene"
    R152a = "R152a"
    R1243zf = "R1243zf"

    @classmethod
    def parse(cls, value) -> "Refrigerant":
        if isinstance(value, Refrigerant):
            return value
        if isinstance(value, str):
            key = value.strip().lower()
            mapping = {
                "propane": cls.PROPANE,
                "r290": cls.PROPANE,
                "c3h8": cls.PROPANE,
                "propylene": cls.PROPYLENE,
                "propen": cls.PROPYLENE,
                "r1270": cls.PROPYLENE,
                "c3h6": cls.PROPYLENE,
            }
            return mapping.get(key, cls.PROPANE)
        return cls.PROPANE


class MolinaroliCorrelationCompressor(Compressor):
    """
    Compressor model based on the dissertation from [1] Luca Molinaroli, 2017, https://www.sciencedirect.com/science/article/pii/S0140700717301512?via%3Dihub.

    Model from Molinaroli is semi-physical. Christoph Höges fitted the model with the LOGIN testbench (rotary compressor
    and IHX cycle). This fit has been used to simulate in CrisPy for R290, R1290, R152a, R1243zf
    in the Range of -20 °C to 0 °C (source) and 35 °C to 75 °C (sink).

    Correlations depending on evaporation and condensation pressure for
    the isentropic and volumetric efficiency are used.

    Parameters:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the volumetric efficiency based on model from Molinaroli et al. [1] and fit from CrisPy.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the volumetric efficiency based on model from Molinaroli et al. [1] and fir from CrisPy.

        get_eta_mech(inputs: Inputs) -> float:
            Returns the mechanical efficiency (assumed constant).

    """

    # coefficients for different refrigerants (eta_is and eta_vol)
    # Form of the correlations: z = a0 + a1*p_in + a2*p_out + a3*p_in*p_out
    _COEFFS: Dict["Refrigerant", Dict[str, Tuple[float, float, float, float]]] = {
        # Propane (R290):
        # eta_vol:
        #   0.71318 + 5.2114e-07 * p_inlet + -3.1971e-08 * p_outlet + -3.8984e-14 * (p_inlet * p_outlet)
        # eta_is:
        #   0.46914 + 4.6869e-07 * p_inlet + -1.0874e-08 * p_outlet + -1.9902e-14 * (p_inlet * p_outlet)
        Refrigerant.PROPANE: {
            "vol": (0.71318, 5.2114e-07, -3.1971e-08, -3.8984e-14),
            "is": (0.46914, 4.6869e-07, -1.0874e-08, -1.9902e-14),
        },
        # Propylene (R1270):
        # eta_vol:
        #   0.0.6685 + 5.0486e-07 * p_inlet + -1.8515e-08 * p_outlet + -5.8768e-14 * (p_inlet * p_outlet)
        # eta_is:
        #   0.46529 + 4.2297e-07 * p_inlet + -6.0647e-09 * p_outlet + -3.6248e-14 * (p_inlet * p_outlet)
        Refrigerant.PROPYLENE: {
            "vol": (0.6685, 5.0486e-07, -1.8515e-08, -5.8768e-14),
            "is": (0.46529, 4.2297e-07, -6.0647e-09, -3.6248e-14),
        },
        # R152a:
        # eta_vol:
        #   0.64257 + 9.3218e-07 * p_inlet + -4.5755e-08 * p_outlet + -1.0489e-13 * (p_inlet * p_outlet)
        # eta_is:
        #   0.35267 + 9.3874e-07 * p_inlet + 1.0846e-09 * p_outlet + -7.38e-14 * (p_inlet * p_outlet)
        Refrigerant.R152a: {
            "vol": (0.64257, 9.3218e-07, -4.5755e-08, -1.0489e-13),
            "is": (0.35267, 9.3874e-07, 1.0846e-09, -7.38e-14),
        },
        # R1243zf::
        # eta_vol:
        #   0.64257 + 9.3218e-07 * p_inlet + -4.5755e-08 * p_outlet + -1.0489e-13 * (p_inlet * p_outlet)
        # eta_is:
        #   0.68838 + 8.7336e-07 * p_inlet + -4.5558e-08 * p_outlet + -9.547e-14 * (p_inlet * p_outlet)
        Refrigerant.R1243zf: {
            "vol": (0.64257, 9.3218e-07, -4.5755e-08, -1.0489e-13),
            "is": (0.68838, 8.7336e-07, -4.5558e-08, -9.547e-14),
        },
    }

    def __init__(self,
                 N_max: float,
                 V_h: float,
                 eta_mech: float = 1,
                 refrigerant=None):
        """
        Initialize the PiCorrelationCompressor.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
            refrigerant: Used refrigerant for the correlation fit.
                            Selection between Propane, Propylene, R152a, R1243zf
                            (None/unvalid: Propane)
        """
        super().__init__(N_max=N_max, V_h=V_h)
        self.refrigerant: Refrigerant = Refrigerant.parse(refrigerant)
        self.eta_mech = eta_mech # assumed constant

    def _eval_correlation(self, kind: str, p_inlet: float, p_outlet: float) -> float:
        a0, a1, a2, a3 = self._COEFFS[self.refrigerant][kind]
        return a0 + a1 * p_inlet + a2 * p_outlet + a3 * (p_inlet * p_outlet)

    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Get the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Volumetric efficiency.
        """
        p_outlet = self.get_p_outlet()
        p_inlet = self.state_inlet.p
        eta_vol = self._eval_correlation("vol", p_inlet, p_outlet)
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
        p_inlet = self.state_inlet.p
        eta_is = self._eval_correlation("is", p_inlet, p_outlet)
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