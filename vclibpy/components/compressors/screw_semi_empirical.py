from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs
import numpy as np
import math

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
            n: float,
            my: float, # dynamic viscosity of the lubricant
            N_max,
            V_h,
            m_flow_nom: float,
            a_tl_1: float,
            a_tl_2: float,
            A_leak: float,
            AU_su_nom: float,
            AU_ex_nom: float,
            b_hl: float,
            BVR: float,
            V_sw: float
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
        self.n              = n
        self.my             = my


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

        self.T_amb = 25.  # temperature of ambience



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
        # boundary conditions
        p_out = self.get_p_outlet()
        p_in  = self.state_inlet.p
        T_in  = self.state_inlet.T
        s_in  = self.state_inlet.s


        R = 8.314 #ideal gas constant [J/(K*mol)]
        state_in = self.state_inlet

        T_out_start = self.med_prop.calc_state('PS', p_out, s_in).T
        T_out_next = T_out_start

        number_of_iterations  = 0
        m_flow_history = []
        m_flow_start = self.m_flow_nom
        m_flow_next = m_flow_start
        T_w_start = self.T_amb
        T_w_next = T_w_start
        T_w_history = []

        while True:
            number_of_iterations +=1
            m_flow = m_flow_next
            m_flow_history.append(m_flow)


            T_out = T_out_next
            T_w = T_w_next
            T_w_history.append(T_w)

            # Dertermination of state 2

            state_6 = self.med_prop.calc_state("PT", p_out, T_out)
            state_1 = self.med_prop.calc_state('PT', p_in, T_in)
            gamma = transport_properties.cp / transport_properties.cv
            p_crit_leak = p_out *((2/(gamma+1)) **(gamma/(gamma+1)))
            p_leak = max (p_crit_leak, state_1.p)
            state_leak = self.med_prop.calc_state('PS', p_leak, state_6.s)
            m_flow_leak = (1/state_leak.d) * self.A_leak * np.sqrt(2 * (state_6.h - state_leak.h))
            m_flow_ges = m_flow + m_flow_leak
            transport_properties = self.med_prop.calc_mean_transport_properties(state_1, state_6)
            h_2 = (m_flow * state_1.h + m_flow_leak * state_6.h) / m_flow_ges

            # Determination of state 2 via Volumenkonstanz


            v_2 = m_flow / (m_flow_ges * state_1.d) + m_flow_leak / (m_flow_ges * state_6.d)
            d_2 = 1/ v_2
            state_2 = self.med_prop.calc_state('DH', d_2, h_2)


            ## Poytropenverhältnis
            #ny = (state_6.s-state_1.s)/R * math.log(state_6.p/state_1.p)
            #p_2_array = np.linspace(p_in, p_out, 20)
            #T_2_array = []
            #h_2_array = []
            #for p_2 in p_2_array:
            #    T_2 = T_in *(p_2/p_in) ** (ny * R /transport_properties.cp)
            #    h_2 = self.med_prop.calc_state('PT', p_2, T_2).h
            #    T_2_array.append(T_2)
            #    h_2_array.append(h_2)

            #p_2 = np.interp(h_2, h_2_array, p_2_array)
            #state_2 = self.med_prop.calc_state('PH', p_2, h_2)



            # Determination of state 3

            AU_su = self.AU_su_nom * (m_flow_ges/self.m_flow_nom) ** 0.8
            Q_flow_23 = (m_flow_ges * (T_w - state_2.T) * transport_properties.cp *
                        (1 - np.exp(-AU_su / (m_flow_ges * transport_properties.cp))))
                        # todo: Check, if T_2 is correct (assumption in determination of state 2) evtl: keine Entropieproduktion

            h_3 = h_2 + Q_flow_23 / m_flow_ges
            v_3 = self.V_h * self.n / m_flow_ges
            d_3 = 1 / v_3
            state_3 = self.med_prop.calc_state('DH', d_3, h_3)

            # Determination of state 4

            v_4 = v_3/self.BVR
            d_4 = 1/v_4
            s_4 = state_3.s
            state_4 = self.med_prop.calc_state('DS', d_4, s_4)
            state_5 = self.med_prop.calc_state('PD', p_out, state_4.d)
            w_in = (state_4.h - state_3.h) + (1 / state_4.d) /(state_5.p-state_4.p)

            # Determination of state 6

            AU_ex = self.AU_ex_nom * (m_flow_ges / self.m_flow_nom) ** 0.8
            Q_flow_56 = (m_flow_ges * (T_w - state_5.T) * transport_properties.cp *
                        (1 - np.exp(-AU_ex / (m_flow_ges * transport_properties.cp))))
            h_6 = state_5.h + Q_flow_56 / m_flow_ges
            state_6 = self.med_prop.calc_state('PH', p_out, h_6)
            T_out_next = state_6.T

            P_in = w_in * m_flow_ges
            P_loss_1 = P_in * self.a_tl_1
            P_loss_2 = self.a_tl_2 * self.V_h * ((np.pi * self.n / 30) ** 2) * self.my
            P_sh = P_in + P_loss_1 + P_loss_2

            Q_flow_amb = self.b_hl * (T_w - self.T_amb) ** 1.25

            h_out= state_in.h + (P_sh - Q_flow_amb) / m_flow
            T_out_next = self.med_prop.calc_state('PH', p_out, h_out)

            T_w_next = self.T_amb + ((P_loss_1 + P_loss_2 - Q_flow_23 - Q_flow_56)/self.b_hl) ** (4/5)





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
