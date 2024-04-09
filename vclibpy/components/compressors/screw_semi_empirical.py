from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs, FlowsheetState
import numpy as np
import math
from scipy.optimize import minimize

import logging
logger = logging.getLogger(__name__)


class ScrewCompressorSemiEmpirical(Compressor):
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
        self.A_leak = 3.32e-6
        self.AU_su_nom = 60.5
        self.AU_ex_nom = 35.6
        self.b_hl = 1.82
        self.BVR = 3.26
        self.eta_el = eta_el

        self.my = my  # dynamic viscosity of the lubricant
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
        """ Iterate process until error is smaller than tolerance

        Return:
        :return float isen_eff:
            Isentropic compressor efficiency
        :return float vol_eff:
            Volumetric compressor efficiency
        """
        self.p_outlet = p_outlet
        self.state_suc = state_in
        self.state_dis_is = self.rp.calc_state("PS", self.p_dis, self.state_suc.s)
        self.f = 50

        self.limits = ((self.p_dis + 1, self.p_dis + 1000),  # state4.p
                       (self.state_suc.s, self.state_suc.s + 500))  # state4.s

        x0 = np.random.random(2)

        res = minimize(fun=self._objective,
                       x0=x0,
                       bounds=((0, 1), (0, 1)),
                       constraints={"type": "ineq", "fun": self._constraint},
                       method="SLSQP")

        self.state_outlet = self._iterate(x, mode="state_out")



        return self.state_outlet

    def _objective(self, x):
        return self._iterate(x, mode="err")

    def _constraint(self, x):
        return self._iterate(x, mode="constr")

    def _iterate(self, x, mode, inputs: Inputs):

        m_flow = x[0]
        T_out = x[1]
        T_w = x[2]

        p_out = self.p_outlet

        state_in = self.state_inlet
        state_out_is = self.med_prop.calc_state('PS', p_out, state_in.s)




        if inputs.T_ambient is not None:
            T_amb = inputs.T_amb
        else:
            T_amb = 25 + 273.15

        # Dertermination of state 2

        state_out = self.med_prop.calc_state("PT", p_out, T_out)
        state_1 = self.med_prop.calc_state('PT', state_in.p, state_in.T)
        transport_properties = self.med_prop.calc_mean_transport_properties(state_1, state_out)
        gamma = transport_properties.cp / transport_properties.cv
        p_crit_leak = p_out * ((2/(gamma+1)) **(gamma/(gamma+1)))
        p_leak = max(p_crit_leak, state_1.p)
        state_leak = self.med_prop.calc_state('PS', p_leak, state_out.s)
        m_flow_leak = (1/state_leak.d) * self.A_leak * np.sqrt(2 * (state_out.h - state_leak.h))
        m_flow_ges = m_flow + m_flow_leak
        h_2 = (m_flow * state_1.h + m_flow_leak * state_out.h) / m_flow_ges

        # Determination of state 2 --> Isobaric, isenthalp mixing
        state_2 = self.med_prop.calc_state('PH', state_1.p, h_2)

        # Determination of state 3

        AU_su = self.AU_su_nom * (m_flow_ges/self.m_flow_nom) ** 0.8
        Q_flow_23 = (m_flow_ges * (T_w - state_2.T) * transport_properties.cp *
                     (1 - np.exp(-AU_su / (m_flow_ges * transport_properties.cp))))


        h_3 = h_2 + Q_flow_23 / m_flow_ges
        v_3 = self.V_h * n_abs / m_flow_ges
        d_3 = 1 / v_3
        state_3 = self.med_prop.calc_state('DH', d_3, h_3)

        # Determination of state 4

        d_4 = self.BVR * d_3
        s_4 = state_3.s
        state_4 = self.med_prop.calc_state('DS', d_4, s_4)
        state_5 = self.med_prop.calc_state('PD', p_out, d_4)
        w_t = (state_4.h - state_3.h) + (1 / state_4.d) / (state_5.p-state_4.p)

        # Determination of state 6

        AU_ex = self.AU_ex_nom * (m_flow_ges / self.m_flow_nom) ** 0.8
        Q_flow_56 = (m_flow_ges * (T_w - state_5.T) * transport_properties.cp *
                    (1 - np.exp(-AU_ex / (m_flow_ges * transport_properties.cp))))
        h_6 = state_5.h + Q_flow_56 / m_flow_ges
        state_6 = self.med_prop.calc_state('PH', p_out, h_6)

        P_t = w_t * m_flow_ges
        P_loss_1 = P_t * self.a_tl_1
        P_loss_2 = self.a_tl_2 * self.V_h * ((np.pi * n_abs / 30) ** 2) * self.my
        P_mech = P_t + P_loss_1 + P_loss_2

        Q_flow_amb = self.b_hl * (T_w - T_amb) ** 1.25

        h_out = state_in.h + (P_mech - Q_flow_amb) / m_flow
        state_out = self.med_prop.calc_state('PH', p_out, h_out)

        T_w = T_amb + ((P_loss_1 + P_loss_2 - Q_flow_23 - Q_flow_56)/self.b_hl) ** (4/5)



        eta_is = (state_out_is.h - state_in.h) / (state_out.h - state_in.h)
        eta_mech = P_t / P_mech
        eta_vol = (m_flow * state_in.d) / (self.V_h * n_abs)




        self.eta_mech = eta_mech
        self.eta_vol = eta_vol



        if mode == "err":
            return np.linalg.norm(np.array(err))
        elif mode == "constr":
            return [Q_flow_amb, -Q_flow_23, m_flow_leak, m_flow]
        elif mode == "state_out":
            return self.state_outlet
    def get_lambda_h(self, inputs: Inputs) -> float:
        """
        Get the volumetric efficiency.

        Args:
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Volumetric efficiency.
        """
        assert self.eta_vol is not None, "You have to calculate the outlet state first."
        return self.eta_vol

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs) -> float: #todo: implement claculation of eta_isentropic
        """
        Get the isentropic efficiency.

        Args:
            p_outlet (float): High pressure value.
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Isentropic efficiency.
        """
        assert inputs.eta_is is not None, "You have to calculate the outlet state first."
        return inputs.eta_is

    def get_eta_mech(self, inputs: Inputs) -> float:
        """
        Returns the product of the constant mechanical, motor, and inverter efficiencies
        as the effective mechanical efficiency of the compressor.

        Args:
            inputs (Inputs): Input parameters for the calculation.

        Returns:
            float: Effective mechanical efficiency.
        """
        assert self.eta_mech is not None, "You have to calculate the outlet state first."
        return self.eta_mech

    def calc_m_flow(self, inputs: Inputs, fs_state: FlowsheetState) -> float:
        """
        Calculate the refrigerant mass flow rate.

        Args:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Refrigerant mass flow rate.
        """
        assert self.m_flow is not None, "You have to calculate the outlet state first."
        return self.m_flow

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState) -> float: #todo: an Modell anpassen

        """
        Calculate the electrical power consumed by the compressor based on an adiabatic energy balance.

        Args:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Electrical power consumed.
        """


        # Electrical power consumed
        P_el = fs_state.P_mech / self.eta_el
        fs_state.set(name="eta_el", value=self.eta_el, unit="-", description="Electrical efficiency")
        fs_state.set(name="P_el", value=self.P_el, unit="W", description="Electrical power")
        return P_el



















