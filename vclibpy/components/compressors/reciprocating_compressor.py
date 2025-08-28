from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class ReciprocatingCompressor(Compressor):
    """
    Compressor model based on the paper of Ortiz et al. (2003), which can't be accessed directly but is cited in Sarkar (2008).

    This compressor class is characterized by using a semi-empiric greybox model provided by Ortiz et al.
    The coefficients used in the model are sourced from Sarkar (2008). Other coefficients can be found in the paper of Okasha (2018).

    Parameters:
        Still to be determined
        (
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        )
    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Returns the volumetric efficiency based on the regressions of Mirko Engelpracht.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Returns the isentropic efficiency based on the regressions of Mirko Engelpracht.

        get_eta_mech(inputs: Inputs) -> float:
            Returns the mechanical efficiency based on the regressions of Mirko Engelpracht.

    """
    def __init__(self, N_max: float, V_h: float, eta_mech: float = 0.85,
                 # Default coefficients from Sarkar (2008) citing Ortiz et al. (2003)
                 lambda_a0: float = 0.9207,
                 lambda_a1: float = -0.0756,
                 lambda_a2: float = 0.0018,
                 eta_is_b0: float = -0.26,
                 eta_is_b1: float = 0.7952,
                 eta_is_b2: float = -0.2803,
                 eta_is_b3: float = 0.0414,
                 eta_is_b4: float = -0.0022):
        super().__init__(N_max=N_max, V_h=V_h)
        # Store all parameters as instance variables
        self.eta_mech_const = eta_mech
        self.lambda_a0 = lambda_a0
        self.lambda_a1 = lambda_a1
        self.lambda_a2 = lambda_a2
        self.eta_is_b0 = eta_is_b0
        self.eta_is_b1 = eta_is_b1
        self.eta_is_b2 = eta_is_b2
        self.eta_is_b3 = eta_is_b3
        self.eta_is_b4 = eta_is_b4


    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Returns the volumetric efficiency based on the semi-empirical model of Ortiz et al. (2003).

        Args:
            inputs (Inputs): Not directly used, but necessary for method signature.

        Returns:
            float: volumetric efficiency.
        """
        pi = self.get_p_outlet() / self.state_inlet.p
        lambda_h = (
                self.lambda_a0 +
                self.lambda_a1 * pi +
                self.lambda_a2 * pi ** 2
        )
        return lambda_h

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the isentropic efficiency based on the semi-empirical model of Ortiz et al. (2003).

        Args:
            p_outlet (float): Outlet pressure.
            inputs (Inputs): Not directly used, but necessary for method signature.

        Returns:
            float: Isentropic efficiency.
        """
        pi = p_outlet / self.state_inlet.p

        eta_is = (
                self.eta_is_b0 +
                self.eta_is_b1 * pi +
                self.eta_is_b2 * pi ** 2 +
                self.eta_is_b3 * pi ** 3 +
                self.eta_is_b4 * pi ** 4
        )

        # Plausibility check
        if eta_is <= 0 <= 1:
            raise ValueError(f"Calculated isentropic efficiency ({eta_is:.3f}) is outside the plausible range (0.1-1.0) for pi={pi:.2f}.")
        return eta_is

    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the mechanical efficiency of the compressor.
        """
        # Returns the constant value defined during initialization
        return self.eta_mech_const
