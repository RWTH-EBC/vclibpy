from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs
import numpy as np

class RotaryCompressor(Compressor):
    """
     Reciprocating compressor semi-empirical model.

    Sources:
    --------
    [1] Giuffrida, Antonio; 2016, A semi-empirical method for assessing the performance of an open-drive screw
    refrigeration compressor. In: Applied Thermal Engineering 93 (2016) 813–823, DOI:https://doi.org/10.1016/j.applthermaleng.2015.10.023


    Parameters:


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
        """Initialization function

        Parameters:


        Compressor specific parameters:

        :param m_flow_nom:  Nominal mass flow rate, [kg/s]
        :param a_tl_1:      Coefficient for internal load losses, [-]
        :param a_tl_2:      Coefficient for viscous friction losses, [-]
        :param A_leak:      Leakage area, [mm^2]
        :param AU_su_nom:   Supply heat transfer coefficient, [W/K]
        :param AU_ex_nom:   Exhaust heat transfer coefficient, [W/K]
        :param b_hl:        Coefficient for ambient heat losses, [W/K^(5/4)]
        :param BVR:         Built-in volume ratio, [-]
        :param V_sw:        Swept volume, [cm^3]
        """
        # Super init function
        super().__init__()


        # Model parameters for Bitzer OSN5361-K  118 / 142 (https://www.bitzer.de/ch/de/produkte/schraubenverdichter/offen/fuer-standardkaeltemittel/os-serie/#!OSN5361)
        # Parameters taken from Giuffrida
        self.m_flow_nom     = 0.988
        self.a_tl_1         = 0.265
        self.a_tl_2         = 134.5
        self.A_leak         = 3.32
        self.AU_su_nom      = 60.5
        self.AU_ex_nom      = 35.6
        self.b_hl           = 1.82
        self.BVR            = 3.26
        self.V_sw           = 676.8


        #  state of fluid at in/out
        # param_state:
        # index in: suction
        # index out: discharge
        #       T_in                temperature, [°C]
        #       p_in                pressure, [kPa]
        #       v_in                specific volume, [m³/kg]
        #       u_in                specific internal energy, [kJ/kg]
        #       h_in                specific enthalpy, [kJ/kg]
        #       s_in                specific entropy, [kJ/kg/K]
        #       p_out               pressure, [kPa]

        # param_cycle: values that stay constant during one cycle/are iterated over cycles
        #       A_eff_suction       effective flow area of suction, [m²]
        #       A_eff_discharge     effective flow area of discharge, [m²]

        # z_iteration: stores the following values for every iteration step
        #       teta                crank angle, [rad]
        #       x                   piston position, [m]
        #       V                   cylinder volume, [m³]
        #       surface_heat        surface of heat transfer, [m²]
        #       cycle_step          0=compression, 1=discharge, 2=expansion, 3=suction, [-]
        #       T                   temperature inside cylinder, [°C]
        #       p                   pressure inside cylinder, [kPa]
        #       v                   specific volume inside cylinder, [m³/kg]
        #       u                   specific internal energy inside cylinder, [kJ/kg]
        #       h                   specific enthalpy inside cylinder, [kJ/kg]
        #       s                   specific entropy inside cylinder, [kJ/kg/K]
        #       m                   mass inside cylinder, [kg]
        #       T_thermal_mass      temperature of thermal mass, [°C]
        #       alpha               heat transfer coefficient inside, [W/m²/K]
        #       dm                  mass flowed in/out in current iteration step, [kg]
        #       q                   specific transferred heat [kJ/kg]

        # data for each cycle
        self.iteration_steps = iteration_steps = 5000  # amount of differential iteration steps for each cycle
        # self.param_compressor = param_compressor
        self.param_compressor = {"piston_diameter": 34e-3, "eng_dis_length": 34e-3, "ratio": 3.5,
                                 "A_cycle_surface": .04, "c1": .06071, "p_friction": 48.916, "el_comp_freq": 50.,
                                 "rot_speed_comp": 50. / 2., "amount_cyl": 2.}
        self.param_state = {"T_in": 0.0, "p_in": 0.0, "v_in": 0.0, "u_in": 0.0, "h_in": 0.0, "s_in": 0.0, "p_out": 0.0}
        self.param_cycle = {"A_eff_suction": 0.0, "A_eff_discharge": 0.0}
        self.z_iteration = {"teta": np.zeros(iteration_steps, float), "x": np.zeros(iteration_steps, float),
                            "V": np.zeros(iteration_steps, float), "surface_heat": np.zeros(iteration_steps, float),
                            "cycle_step": np.zeros(iteration_steps, float), "T": np.zeros(iteration_steps, float),
                            "p": np.zeros(iteration_steps, float), "v": np.zeros(iteration_steps, float),
                            "u": np.zeros(iteration_steps, float), "h": np.zeros(iteration_steps, float),
                            "s": np.zeros(iteration_steps, float), "m": np.zeros(iteration_steps, float),
                            "T_thermal_mass": np.zeros(iteration_steps, float),
                            "alpha": np.zeros(iteration_steps, float), "dm": np.zeros(iteration_steps, float),
                            "q": np.zeros(iteration_steps, float)}
        self.energies = {"W_compressor": np.zeros(iteration_steps, float),
                         "W_friction": np.zeros(iteration_steps, float),
                         "Q_compressor": np.zeros(iteration_steps, float)}

        # fitting compressor
        self.fitting_compressor = {"piston_diameter": 34e-3, "piston_height": 34e-3, "amount_cyl": 2.,
                                   "cylinder_surface": .04}

        self.T_env = 25.  # temperature of environment



        # thermodynamic substance data
        self.v_fit = None
        self.u_fit = None
        self.T_fit = None
        self.p_fit = None
        self.h_fit = None

    def _iterate(self):
        """ Iterate process until error is smaller than tolerance

        Return:
        :return float isen_eff:
            Isentropic compressor efficiency
        :return float vol_eff:
            Volumetric compressor efficiency
        """
        p_out = self.get_p_outlet()
        p_in  = self.state_inlet.p
        T_in  = self.state_inlet.T
        s_in  = self.state_inlet.s


        T_out_start = self.med_prop.calc_state('PS', p_out, s_in)

        number_of_iterations  = 0
        m_flow_history = []
        m_flow_start = self.m_flow_nom
        m_flow_next = m_flow_start
        m_flow_leak_history = []
        m_flow_leak_start = 0
        m_flow_leak_next = m_flow_leak_start




        while True:
            number_of_iterations +=1
            m_flow = m_flow_next
            m_flow_leak = m_flow_leak_next
            m_flow_ges = m_flow + m_flow_leak






    def calc_compression(self, inputs: Inputs):


        raise NotImplementedError("Re-implement this function to use it")


    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Returns the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Volumetric efficiency.
        """
        raise NotImplementedError("Re-implement this function to use it")




    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the isentropic efficiency.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Isentropic efficiency.
        """
        raise NotImplementedError("Re-implement this function to use it")


    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the mechanical efficiency.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Mechanical efficiency.
        """
        raise NotImplementedError("Re-implement this function to use it")
