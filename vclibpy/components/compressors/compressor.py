"""
Module for different compressor models
"""

from vclibpy.components.component import BaseComponent
from vclibpy.datamodels import Inputs, FlowsheetState


class Compressor(BaseComponent):
    """
    Base compressor class to be extended for specific compressor models.

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

    def __init__(self, N_max: float, V_h: float):
        """
        Initialize the compressor.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
        """
        super().__init__()
        self.N_max = N_max
        self.V_h = V_h

    def get_lambda_h(self, p_outlet, inputs: Inputs) -> float:
        """
        Get the volumetric efficiency.

        Args:
            p_outlet:
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
        Get the mechanical efficiency including motor and inverter efficiencies.

        Args:
            inputs (Inputs): Inputs for the calculation.

        Returns:
            float: Mechanical efficiency including motor and inverter efficiencies.
        """
        raise NotImplementedError("Re-implement this function to use it")

    def get_p_outlet(self) -> float:
        """
        Get the outlet pressure.

        Returns:
            float: Outlet pressure.
        """
        assert self.state_outlet is not None, "You have to calculate the outlet state first."
        return self.state_outlet.p

    def get_n_absolute(self, n: float) -> float:
        """
        Return given relative n as absolute rounds/sec based on self.N_max.

        Args:
            n (float): Relative compressor speed between 0 and 1.

        Returns:
            float: Absolute compressor frequency in rounds/sec.
        """
        return self.N_max * n

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

    def calc_m_flow(self, inputs: Inputs, fs_state: FlowsheetState,
                    lambda_h=None) -> float:
        """
        Calculate the refrigerant mass flow rate.

        Args:
            lambda_h:
            inputs (Inputs): Inputs for the calculation.
            fs_state (FlowsheetState): Flowsheet state.

        Returns:
            float: Refrigerant mass flow rate.
        """
        if lambda_h is None:
            lambda_h = max(0.01,self.get_lambda_h(inputs=inputs,
                                         p_outlet=None))

        V_flow_ref = (
                lambda_h *
                self.V_h *
                self.get_n_absolute(inputs.n)
        )
        self.m_flow = self.state_inlet.d * V_flow_ref
        fs_state.set(name="lambda_h", value=lambda_h, unit="-", description="Volumetric efficiency")
        fs_state.set(name="REF_V_flow_comp", value=V_flow_ref, unit="m3/s", description="Refrigerant volume flow rate")
        fs_state.set(name="REF_m_flow_comp", value=self.m_flow, unit="kg/s", description="Refrigerant mass flow rate")
        return self.m_flow

    def calc_n(self, inputs: Inputs, fs_state: FlowsheetState,
               lambda_h=None):
        if lambda_h is None:
            lambda_h =  max(0.01,self.get_lambda_h(inputs=inputs,
                                         p_outlet=None))
        V_flow_ref = self.m_flow/self.state_inlet.d
        n_abs = V_flow_ref/(lambda_h * self.V_h)
        return n_abs/self.N_max

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

    def terminate_secondary_med_prop(self):
        if self.med_prop is not None:
            self.med_prop.terminate()

