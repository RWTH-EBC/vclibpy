from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs, FlowsheetState
import numpy as np
import math

import logging
logger = logging.getLogger(__name__)








class ScrewCompressorSemiEmpirical(Compressor):
    """
    Class of semi-empirical screw compressor model

    Sources:
    --------
    [1] Giuffrida, Antonio; 2016, A semi-empirical method for assessing the performance of an open-drive screw
    refrigeration compressor. In: Applied Thermal Engineering 93 (2016) 813–823, DOI:https://doi.org/10.1016/j.applthermaleng.2015.10.023



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
                 eta_mech: float,
                 my=68,
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
        self.A_leak = 3.32
        self.AU_su_nom = 60.5
        self.AU_ex_nom = 35.6
        self.b_hl = 1.82
        self.BVR = 3.26
        self.eta_mech = eta_mech

        self.my = my  # dynamic viscosity of the lubricant
        self.T_amb = 25.  # temperature of ambience
        self.max_num_iterations = max_num_iterations

    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Get the volumetric efficiency.

        Args:
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Volumetric efficiency.
        """
        raise NotImplementedError("Re-implement this function to use it")

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Get the isentropic efficiency.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Isentropic efficiency.
        """
        raise NotImplementedError("Re-implement this function to use it")

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

    def calc_state_outlet(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState):
        """
        Calculate the output state based on the high pressure level and the provided inputs.
        The state is automatically set as the outlet state of this component.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for calculation.
            fs_state (FlowsheetState): Flowsheet state.
        """
        """ Iterate process until error is smaller than tolerance

        Return:
        :return float isen_eff:
            Isentropic compressor efficiency
        :return float vol_eff:
            Volumetric compressor efficiency
        """
        accuracy = 0.001 # Maximale Abweichung: der Iterationsschritte: 0,1 %
        self.n_abs = self.get_n_absolute(self.n)
        # boundary conditions
        p_out = self.get_p_outlet()
        p_in  = self.state_inlet.p
        T_in  = self.state_inlet.T
        s_in  = self.state_inlet.s


        R = 8.314 #ideal gas constant [J/(K*mol)]
        state_in = self.state_inlet
        state_out_is = self.med_prop.calc_state('PS', p_out, s_in)
        T_out_start = state_out_is.T
        T_out_next = T_out_start

        number_of_iterations = 0
        m_flow_history = []
        m_flow_start = self.m_flow_nom
        m_flow_next = m_flow_start
        T_w_start = inputs.T_ambient
        T_w_next = T_w_start
        T_w_history = []

        while True:
            number_of_iterations += 1
            m_flow = m_flow_next
            m_flow_history.append(m_flow)
            T_out = T_out_next
            T_w = T_w_next
            T_w_history.append(T_w)



            # Dertermination of state 2

            state_6 = self.med_prop.calc_state("PT", p_out, T_out)
            state_1 = self.med_prop.calc_state('PT', p_in, T_in)
            transport_properties = self.med_prop.calc_mean_transport_properties(state_1, state_6)
            gamma = transport_properties.cp / transport_properties.cv
            p_crit_leak = p_out * ((2/(gamma+1)) **(gamma/(gamma+1)))
            p_leak = max(p_crit_leak, state_1.p)
            state_leak = self.med_prop.calc_state('PS', p_leak, state_6.s)
            m_flow_leak = (1/state_leak.d) * self.A_leak * np.sqrt(2 * (state_6.h - state_leak.h))
            m_flow_ges = m_flow + m_flow_leak
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
            v_3 = self.V_h * self.n_abs / m_flow_ges
            d_3 = 1 / v_3
            state_3 = self.med_prop.calc_state('DH', d_3, h_3)

            # Determination of state 4

            v_4 = v_3/self.BVR
            d_4 = 1/v_4
            s_4 = state_3.s
            state_4 = self.med_prop.calc_state('DS', d_4, s_4)
            state_5 = self.med_prop.calc_state('PD', p_out, state_4.d)
            w_t = (state_4.h - state_3.h) + (1 / state_4.d) /(state_5.p-state_4.p)

            # Determination of state 6

            AU_ex = self.AU_ex_nom * (m_flow_ges / self.m_flow_nom) ** 0.8
            Q_flow_56 = (m_flow_ges * (T_w - state_5.T) * transport_properties.cp *
                        (1 - np.exp(-AU_ex / (m_flow_ges * transport_properties.cp))))
            h_6 = state_5.h + Q_flow_56 / m_flow_ges
            state_6 = self.med_prop.calc_state('PH', p_out, h_6)
            T_out_next = state_6.T

            P_t = w_t * m_flow_ges
            P_loss_1 = P_t * self.a_tl_1
            P_loss_2 = self.a_tl_2 * self.V_h * ((np.pi * self.n_abs / 30) ** 2) * self.my
            P_sh = P_t + P_loss_1 + P_loss_2

            Q_flow_amb = self.b_hl * (T_w - inputs.T_ambient) ** 1.25

            h_out= state_in.h + (P_sh - Q_flow_amb) / m_flow
            state_out = self.med_prop.calc_state('PH', p_out, h_out)
            T_out_next = state_out.T
            T_w_next = inputs.T_ambient + ((P_loss_1 + P_loss_2 - Q_flow_23 - Q_flow_56)/self.b_hl) ** (4/5)

            if self.max_num_iterations <= number_of_iterations:
                logger.critical("Breaking: exceeded maximum number of iterations")
                break

            if np.abs((T_w_next-T_w)/T_w) <= accuracy:
                if np.abs((T_out_next-T_out)/T_out) >= accuracy:
                    logger.info("Breaking: Converged")
                    break

        eta_is = (state_out_is.h - state_in.h) / (state_out.h - state_in.h)
        eta_mech = 1
        eta_vol = 1



        state_outlet_isentropic = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s)

        h_outlet = (
                self.state_inlet.h + (state_outlet_isentropic.h - self.state_inlet.h) /
                eta_is
        )
        fs_state.set(name="eta_is", value=eta_is, unit="%", description="Isentropic efficiency")        #todo: check unit
        fs_state.set(name="m_flow", value=m_flow, unit="kg/s", description="Refrigerant mass flow")     #todo: check unit
        fs_state.set(name="eta_mech", value=eta_mech, unit="-", description="Mechanical efficiency")    #todo: check unit
        fs_state.set(name="eta_vol", value=eta_vol, unit="-", description="Volumetric efficiency")      #todo: check unit

        self.state_outlet = self.med_prop.calc_state("PH", p_outlet, state_out.h)
        return self.state_outlet

    def calc_m_flow(self, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Calculate the refrigerant mass flow rate.

        Args:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Refrigerant mass flow rate.
        """


        return self.m_flow

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Calculate the electrical power consumed by the compressor based on an adiabatic energy balance.

        Args:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Electrical power consumed.
        """
        # Heat flow in the compressor
        P_t = self.m_flow * (self.state_outlet.h - self.state_inlet.h)
        # Electrical power consumed
        eta_mech = self.get_eta_mech(inputs=inputs)
        P_el = P_t / eta_mech
        fs_state.set(name="eta_mech", value=eta_mech, unit="-", description="Mechanical efficiency")
        return P_el


########################################################################################################################
########################################################################################################################
########################################################################################################################
################################################################################################################################################################################################################################################
################################################################################################################################################################################################################################################
########################################################################################################################
class ScrewCompressorSemiEmpirical(Compressor):
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

    def __init__(self,
                 N_max: float,
                 V_h: float,
                 eta_mech: float,
                 my = 68,
                 max_num_iterations = 2000):

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
        self.m_flow_nom     = 0.988
        self.a_tl_1         = 0.265
        self.a_tl_2         = 134.5
        self.A_leak         = 3.32
        self.AU_su_nom      = 60.5
        self.AU_ex_nom      = 35.6
        self.b_hl           = 1.82
        self.BVR            = 3.26
        self.eta_mech = eta_mech

        self.my             = my # dynamic viscosity of the lubricant
        self.T_amb = 25.                # temperature of ambience
        self.max_num_iterations = max_num_iterations

    def calc_state_outlet(self, p_outlet: float, inputs: Inputs, fs_state: FlowsheetState):
        """
        Calculate the output state based on the high pressure level and the provided inputs.
        The state is automatically set as the outlet state of this component.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for calculation.
            fs_state (FlowsheetState): Flowsheet state.
        """
        state_outlet_isentropic = self.med_prop.calc_state("PS", p_outlet, self.state_inlet.s)
        eta_is = self.get_eta_isentropic(p_outlet=p_outlet, inputs=inputs)
        h_outlet = (
                self.state_inlet.h + (state_outlet_isentropic.h - self.state_inlet.h) /
                eta_is
        )
        fs_state.set(name="eta_is", value=eta_is, unit="%", description="Isentropic efficiency")
        self.state_outlet = self.med_prop.calc_state("PH", p_outlet, h_outlet)
    def iterate(self, inputs: Inputs, n_abs):
        """ Iterate process until error is smaller than tolerance

        Return:
        :return float isen_eff:
            Isentropic compressor efficiency
        :return float vol_eff:
            Volumetric compressor efficiency
        """
        accuracy = 0.001 # Maximale Abweichung: der Iterationsschritte: 0,1 %

        # boundary conditions
        p_out = self.get_p_outlet()
        p_in  = self.state_inlet.p
        T_in  = self.state_inlet.T
        s_in  = self.state_inlet.s


        R = 8.314 #ideal gas constant [J/(K*mol)]
        state_in = self.state_inlet
        state_out_is = self.med_prop.calc_state('PS', p_out, s_in)
        T_out_start = state_out_is.T
        T_out_next = T_out_start

        number_of_iterations  = 0
        m_flow_history = []
        m_flow_start = self.m_flow_nom
        m_flow_next = m_flow_start
        T_w_start = inputs.T_ambient
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
            transport_properties = self.med_prop.calc_mean_transport_properties(state_1, state_6)
            gamma = transport_properties.cp / transport_properties.cv
            p_crit_leak = p_out * ((2/(gamma+1)) **(gamma/(gamma+1)))
            p_leak = max(p_crit_leak, state_1.p)
            state_leak = self.med_prop.calc_state('PS', p_leak, state_6.s)
            m_flow_leak = (1/state_leak.d) * self.A_leak * np.sqrt(2 * (state_6.h - state_leak.h))
            m_flow_ges = m_flow + m_flow_leak
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
            v_3 = self.V_h * n_abs / m_flow_ges
            d_3 = 1 / v_3
            state_3 = self.med_prop.calc_state('DH', d_3, h_3)

            # Determination of state 4

            v_4 = v_3/self.BVR
            d_4 = 1/v_4
            s_4 = state_3.s
            state_4 = self.med_prop.calc_state('DS', d_4, s_4)
            state_5 = self.med_prop.calc_state('PD', p_out, state_4.d)
            w_t = (state_4.h - state_3.h) + (1 / state_4.d) /(state_5.p-state_4.p)

            # Determination of state 6

            AU_ex = self.AU_ex_nom * (m_flow_ges / self.m_flow_nom) ** 0.8
            Q_flow_56 = (m_flow_ges * (T_w - state_5.T) * transport_properties.cp *
                        (1 - np.exp(-AU_ex / (m_flow_ges * transport_properties.cp))))
            h_6 = state_5.h + Q_flow_56 / m_flow_ges
            state_6 = self.med_prop.calc_state('PH', p_out, h_6)
            T_out_next = state_6.T

            P_t = w_t * m_flow_ges
            P_loss_1 = P_t * self.a_tl_1
            P_loss_2 = self.a_tl_2 * self.V_h * ((np.pi * n_abs / 30) ** 2) * self.my
            P_sh = P_t + P_loss_1 + P_loss_2

            Q_flow_amb = self.b_hl * (T_w - inputs.T_ambient) ** 1.25

            h_out= state_in.h + (P_sh - Q_flow_amb) / m_flow
            state_out = self.med_prop.calc_state('PH', p_out, h_out)
            T_out_next = state_out.T
            T_w_next = inputs.T_ambient + ((P_loss_1 + P_loss_2 - Q_flow_23 - Q_flow_56)/self.b_hl) ** (4/5)

            if self.max_num_iterations <= number_of_iterations:
                logger.critical("Breaking: exceeded maximum number of iterations")
                break

            if np.abs((T_w_next-T_w)/T_w) <= accuracy:
                if np.abs((T_out_next-T_out)/T_out) >= accuracy:
                    logger.info("Breaking: Converged")
                    break

        eta_s = (state_out_is.h - state_in.h) / (state_out.h - state_in.h)
        eta_mech = 1
        eta_vol = 1
        return state_out, eta_s, eta_vol, eta_mech, m_flow










    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Returns the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: volumetric efficiency.
        """
        n_abs = self.get_n_absolute(inputs.n)

        state_in, state_out, m_flow, h_out_s, P_sh = self.iterate(n_abs)

        lambda_h = m_flow / (n_abs * state_in.d * self.V_h)
        return lambda_h





    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float:
        """
        Returns the isentropic efficiency.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Isentropic efficiency.
        """
        n_abs = self.get_n_absolute(inputs.n)

        state_in, state_out, m_flow, h_out_s, P_sh = self.iterate(n_abs)
        eta_s = (h_out_s - state_in.h) / (state_out.h - state_in.h)

        return eta_s





    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the mechanical efficiency.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Mechanical efficiency.
        """

        n_abs = self.get_n_absolute(inputs.n)

        state_in, state_out, m_flow, h_out_s, P_sh = self.iterate(n_abs)
        eta_mech = m_flow * (state_out.h - state_in.h)/P_sh

        return eta_mech
