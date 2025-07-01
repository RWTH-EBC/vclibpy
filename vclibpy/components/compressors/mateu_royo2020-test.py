from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


class MateuRoyo2020_2(Compressor):
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
                 eps_vol: float,
                 eta_mech: float,
                 a0=1.0455,
                 a1=-0.0184,
                 a2=-0.0011,
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
        super(MateuRoyo2020_2, self).__init__(N_max=N_max, V_h=V_h)
        self.eps_vol = eps_vol
        self.eta_mech = eta_mech
        self.a0 = a0
        self.a1 = a1
        self.a2 = a2

    def calc_state_outlet(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState,
                          eta_is=None):
        """
        Calculate the output state based on the high pressure level and the provided inputs.
        The state is automatically set as the outlet state of this component.

        Args:
            eta_is:
            p_outlet_for_eta:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for calculation.
            fs_state (FlowsheetState): Flowsheet state.
        """
        state_outlet_isentropic = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s)
        if eta_is is None:
            eta_is = max(0.01,self.get_eta_isentropic(p_outlet=p_outlet, inputs=inputs))
        h_outlet = (
                self.state_inlet.h + (state_outlet_isentropic.h - self.state_inlet.h) /
                eta_is
        )
        fs_state.set(name="eta_is", value=eta_is, unit="-", description="Isentropic efficiency")
        self.state_outlet = self.med_prop.calc_state("PH", p_outlet, h_outlet)
        return eta_is

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

        return self.a0 + self.a1 * rp + self.a2 * rp ** 2

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the constant isentropic efficiency of the compressor.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Constant isentropic efficiency.

        """
        v_suc = self.state_inlet.v
        v_ad = v_suc / self.eps_vol
        d_ad = 1 / v_ad
        state_outlet_isentropic = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s)
        state_adaptet = self.med_prop.calc_state("DS", d_ad, self.state_inlet.s)
        h_disch_is = state_outlet_isentropic.h
        h_suc = self.state_inlet.h
        h_ad = state_adaptet.h
        P_disch = p_outlet
        P_ad = state_adaptet.p
        return (h_disch_is - h_suc) / ((h_ad - h_suc) + v_ad * abs(P_disch - P_ad))

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
