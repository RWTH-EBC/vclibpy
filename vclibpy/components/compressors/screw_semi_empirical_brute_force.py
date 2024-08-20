from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs, FlowsheetState
import numpy as np
import math
from scipy.optimize import minimize

import logging
logger = logging.getLogger(__name__)


class ScrewCompressorSemiEmpiricalBruteForce(Compressor):
    """
    Class of semi-empirical screw compressor model

    Sources:
    --------
    [1] Giuffrida, Antonio; 2016, A semi-empirical method for assessing the performance of an open-drive screw
    refrigeration compressor. In: Applied Thermal Engineering 93 (2016) 813–823, DOI:https://doi.org/10.1016/j.applthermaleng.2015.10.023
        Model parameters for Bitzer OSN5361-K  118 / 142 (https://www.bitzer.de/ch/de/produkte/schraubenverdichter/offen/fuer-standardkaeltemittel/os-serie/#!OSN5361)
        Fitting Parameters taken from Giuffrida

    Args:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.

    Methods:
        get_lambda_h(inputs: Inputs) -> float:
            Get the volumetric efficiency.

        get_eta_isentropic(p_outlet: float, inputs: Inputs) -> float:
            Get the isentropic efficiency.

        get_eta_mech(inputs: Inputs) -> float:
            Get the mechanical efficiency.

        get_p_outlet() -> float:
            Get the outlet pressure.

        get_n_absolute(n: float) -> float:
            Return the absolute compressor frequency based on the relative speed.

        calc_state_outlet(p_outlet: float, inputs: Inputs, fs_state: FlowsheetState):
            Calculate the outlet state based on the high pressure level and provided inputs.

        calc_m_flow(inputs: Inputs, fs_state: FlowsheetState) -> float:
            Calculate the refrigerant mass flow rate.

        calc_electrical_power(inputs: Inputs, fs_state: FlowsheetState) -> float:
            Calculate the electrical power consumed by the compressor based on an adiabatic energy balance.
    """

    def __init__(self,
                 N_max: float,
                 V_h: float,
                 eta_el: float,
                 my_40 = 170 * 1e-6,
                 my_100 = 18e-6,
                 max_num_iterations=2000):
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
        super().__init__(N_max=N_max, V_h=V_h)

        # Model parameters for Bitzer OSN5361-K  118 / 142 (https://www.bitzer.de/ch/de/produkte/schraubenverdichter/offen/fuer-standardkaeltemittel/os-serie/#!OSN5361)
        # Parameters taken from Giuffrida
        self.m_flow_nom = 0.988
        self.a_tl_1 = 0.265
        self.a_tl_2 = 134.5
        self.A_leak = 3.32e-6
        self.AU_su_nom = 60.5
        self.AU_ex_nom = 35.6
        self.b_hl = 1.82
        self.BVR = 3.26
        self.eta_el = eta_el

        self.my_40 = my_40  # dynamic viscosity of the lubricant at 40 °C
        self.my_100 = my_100  # dynamic viscosity of the lubricant at 40 °C

    def normalize(self, x):
        """ Normalize optimization parameters

        Parameters
        :param list x:
            Not-normalized optimization parameters
        Return:
        -------
        :return ndarray x:
            Normalized optimization parameters
        """
        x_norm = [(val - self.limits[idx][0]) / max((self.limits[idx][1] - self.limits[idx][0]), 1e-10)
                  for idx, val in enumerate(x)]
        return np.array(x_norm)

    #
    #
    def denormalize(self, x_norm):
        """ Denormalize optimization parameters

        Parameters
        :param list x_norm:
            Normalized optimization parameters
        Return:
        -------
        :return list x:
            Not-normalized optimization parameters
        """
        # Pressure in evaporator, pressure in condenser, dT super heating, dT subcooling
        x = [self.limits[idx][0] + val * (self.limits[idx][1] - self.limits[idx][0])
             for idx, val in enumerate(x_norm)]
        return x
    def calc_state_outlet(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState):
        """
        Calculate the output state based on the high pressure level and the provided inputs.
        The state is automatically set as the outlet state of this component.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for calculation.
            fs_state (FlowsheetState): Flowsheet state.
        """


        if inputs.T_ambient is not None:
            self.T_amb = inputs.T_amb
        else:
            self.T_amb = 25 + 273.15
        self.state_is = self.med_prop.calc_state('PS', p_outlet, self.state_inlet.s)


        self.p_outlet = p_outlet
        self.inputs = inputs
        self.limits = ((self.state_inlet.T, self.state_is.T + 100),     # T_3
                       (self.state_inlet.T, self.state_is.T + 100))     # T_w

        x_0 = [np.random.rand(), np.random.rand()]
        x = self.denormalize(x_0)

        T_3 = x[0]
        #p_3 = x[1]
        T_w = x[1]
        p_out = self.p_outlet

        inputs = self.inputs
        n_abs = self.get_n_absolute(inputs.n)

        p_3 = self.state_inlet.p
        state_in = self.state_inlet
        state_out_is = self.state_is
        state_3 = self.med_prop.calc_state("PT", p_3, T_3)

        rho_4 = self.BVR * state_3.d

        m_flow_ges = self.V_h * n_abs * state_3.d
        state_4 = self.med_prop.calc_state("DS", rho_4, state_3.s)

        # state_5 = self.med_prop.calc_state('PD', p_out, state_4.d)

        w_t_34 = state_4.h - state_3.h
        w_t_45 = (1 / state_4.d) * (p_out - state_4.p)
        w_t = w_t_34 + w_t_45

        state_5 = self.med_prop.calc_state("PH", p_out, state_4.h + w_t_45)

        # Determination of state_6
        cp_5 = self.med_prop.calc_transport_properties(state_5).cp
        AU_ex = self.AU_ex_nom * (m_flow_ges / self.m_flow_nom) ** 0.8  # Derived from Eq. (4)
        Q_flow_ex = (m_flow_ges * (T_w - state_5.T) * cp_5 *
                     (1 - np.exp(-AU_ex / (m_flow_ges * cp_5))))  # Derived from Eq. (3)
        h_6 = state_5.h + Q_flow_ex / m_flow_ges  # Derived from Eq. (2)
        state_6 = self.med_prop.calc_state('PH', p_out, h_6)

        # Dertermination of leakage mass flow
        state_1 = state_in

        transport_properties_6 = self.med_prop.calc_transport_properties(state_6)
        gamma = transport_properties_6.cp / transport_properties_6.cv
        p_crit_leak = p_out * ((2 / (gamma + 1)) ** (gamma / (gamma + 1)))  # Eq. (9)
        p_leak = max(p_crit_leak, state_1.p)
        state_leak = self.med_prop.calc_state('PS', p_leak, state_6.s)
        m_flow_leak = state_leak.d * self.A_leak * np.sqrt(2 * (state_6.h - state_leak.h))

        # state_2 (Isobaric, isenthalp mixing)
        m_flow = m_flow_ges - m_flow_leak
        h_2 = (m_flow * state_1.h + m_flow_leak * state_6.h) / m_flow_ges  # Eq. (1)
        state_2 = self.med_prop.calc_state('PH', state_1.p, h_2)
        cp_2 = self.med_prop.calc_transport_properties(state_2).cp

        # Enthalpy_3 --> as mimnimization variable
        AU_su = self.AU_su_nom * (m_flow_ges / self.m_flow_nom) ** 0.8  # Eq. (4)
        Q_flow_su = (m_flow_ges * (T_w - state_2.T) * cp_2 *
                     (1 - np.exp(-AU_su / (m_flow_ges * cp_2))))  # Eq. (3)

        P_t = w_t * m_flow_ges
        # mechanical losses
        my = self.my_40 + (self.my_100 - self.my_40) * (T_w - (273.15 + 40)) / 60
        P_loss_1 = P_t * self.a_tl_1  # Eq. (11)
        P_loss_2 = self.a_tl_2 * self.V_h * ((2 * np.pi * n_abs) ** 2) * my
        P_mech = P_t + P_loss_1 + P_loss_2

        delta_T_abs = abs(T_w - self.T_amb)
        Q_flow_amb = self.b_hl * math.pow(delta_T_abs, 1.25) * np.sign(T_w - self.T_amb)  # Eq. (16)

        # error definition
        h_3_tilde = h_2 + Q_flow_su / m_flow_ges
        delta_h3 = (h_3_tilde - state_3.h)
        if state_3.h == state_in.h:
            err_h3 = delta_h3 / 1e-10
        else:
            err_h3 = delta_h3 / (state_3.h - state_in.h)

        m_flow_tilde = (P_mech - Q_flow_amb) / (state_6.h - state_1.h)
        delta_m_flow = (m_flow_tilde - m_flow)
        err_m_flow = delta_m_flow / m_flow

        err = (delta_h3, err_m_flow)

        state_3_tilde = self.med_prop.calc_state("PH", p_3, h_3_tilde)

        # calculate efficiencies
        eta_is = (state_out_is.h - state_in.h) / (state_6.h - state_in.h)
        eta_mech = P_t / P_mech
        eta_vol = (m_flow * state_in.d) / (self.V_h * n_abs)
        self.eta_mech = eta_mech
        self.eta_vol = eta_vol

        result = [x, delta_h3, err_m_flow, state_6]
        return result

