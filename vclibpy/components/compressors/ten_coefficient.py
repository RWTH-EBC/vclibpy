import warnings
from abc import ABC
import numpy as np
import pandas as pd

from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs


def calc_ten_coefficients(T_eva, T_con, coef_list):
    """
    Calculate the result using the ten-coefficient method.

    Args:
        T_eva (float): Evaporator temperature in Celsius.
        T_con (float): Condenser temperature in Celsius.
        coef_list (list): List of coefficients.

    Returns:
        float: Result of the calculation.
    """
    # Formula for the ten-coefficient method according to the datasheet
    z = coef_list[0] + coef_list[1] * T_eva + coef_list[2] * T_con + coef_list[3] * T_eva ** 2 + \
        coef_list[4] * T_eva * T_con + coef_list[5] * T_con ** 2 + coef_list[6] * T_eva ** 3 + \
        coef_list[7] * T_eva ** 2 * T_con + coef_list[8] * T_con ** 2 * T_eva + coef_list[9] * T_con ** 3
    return z


class BaseTenCoefficientCompressor(Compressor, ABC):
    """
    Base class for compressors using the ten-coefficient method.

    Used table has to be in this format
    (order of the columns is not important).
    The values must be the same as in the example tabel.
    The column names can be different but must
    then be matched with argument parameter_names.
    (All typed in numbers are fictional placeholders)

          Capacity(W)     Input Power(W)      Flow Rate(kg/h)     Capacity(W)     ...     Flow Rate(kg/h)
    n     n1              n1                  n1                  n2              ...     n_last
    P1    42              12                  243                 32              ...     412
    ...   ...             ...                 ...                 ...             ...     ...
    P10   10              23                  21                  41              ...     2434

    Args:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        datasheet (str): Path of the datasheet file.
        **kwargs:
            parameter_names (dict, optional):
                Dictionary to match internal parameter names (keys) to the names used in the table values.
                Default
                {
                    "m_flow": "Flow Rate(kg/h)",
                    "capacity": "Capacity(W)",
                    "input_power": "Input Power(W)",
                    "eta_s": "Isentropic Efficiency(-)",
                    "lambda_h": "Volumentric Efficiency(-)",
                    "eta_mech": "Mechanical Efficiency(-)"
                }
            sheet_name (str, optional): Name of the sheet in the datasheet. Defaults to None.
    """

    def __init__(self, N_max, V_h, datasheet, **kwargs):
        """
        Initialize the BaseTenCoefficientCompressor.

        Args:
            N_max (float): Maximal rotations per second of the compressor.
            V_h (float): Volume of the compressor in m^3.
            datasheet (str): Path of the datasheet file.
            parameter_names (dict, optional): Dictionary of parameter names. Defaults to None.
            sheet_name (str, optional): Name of the sheet in the datasheet. Defaults to None.
        """

        super().__init__(N_max, V_h)
        sheet_name = kwargs.get('sheet_name', None)
        self.md = pd.read_excel(datasheet, sheet_name=sheet_name)
        parameter_names = kwargs.get('parameter_names', None)
        if parameter_names is None:
            self.parameter_names = {
                "m_flow": "Flow Rate(kg/h)",
                "capacity": "Capacity(W)",
                "input_power": "Input Power(W)",
                "eta_s": "Isentropic Efficiency(-)",
                "lambda_h": "Volumentric Efficiency(-)",
                "eta_mech": "Mechanical Efficiency(-)"
            }
        else:
            self.parameter_names = parameter_names

    def get_parameter(self, T_eva, T_con, n, type_):
        """
        Get a parameter based on temperatures, rotations, and parameter type from the datasheet.

        Args:
            T_eva (float): Evaporator temperature in Celsius.
            T_con (float): Condenser temperature in Celsius.
            n (float): Rotations per second.
            type_ (str): Parameter type in parameter_names.

        Returns:
            float: Interpolated parameter value.
        """
        param_list = []
        n_list = []

        sampling_points = sum(
            self.parameter_names[type_] in s for s in list(self.md.columns.values))  # counts number of sampling points

        for i in range(sampling_points):
            if i == 0:
                coefficients = self.md[self.parameter_names[type_]].tolist()
            else:
                coefficients = self.md[str(self.parameter_names[type_] + "." + str(i))].tolist()
            n_list.append(coefficients.pop(0))
            param_list.append(calc_ten_coefficients(T_eva, T_con, coefficients))

        return np.interp(self.get_n_absolute(n), n_list, param_list)  # linear interpolation


class TenCoefficientCompressor(BaseTenCoefficientCompressor):
    """
    Compressor based on the ten coefficient method.

    Used table has to be in this format
    (order of the columns is not important).
    The values must be the same as in the example tabel.
    The column names can be different but must
    then be matched with the keyword argument parameter_names.
    (All typed in numbers are fictional placeholders)

          Capacity(W)     Input Power(W)      Flow Rate(kg/h)     Capacity(W)     ...     Flow Rate(kg/h)
    n     n1              n1                  n1                  n2              ...     n_last
    P1    42              12                  243                 32              ...     412
    ...   ...             ...                 ...                 ...             ...     ...
    P10   10              23                  21                  41              ...     2434

    T_sh and T_sc have to be set according to the data sheet of your compressor. capacity_definition defines the
    parameter "capacity" in the datasheet. If capacity is the specific cooling capacity (h1-h4), set it on "cooling".
    If capacity is the specific heating capacity (h2-h3), set it on "heating".
    In the case of cooling capacity, the mechanical efficiency of the compressor has to be assumed (assumed_eta_mech)
    as h2 can't be calculated otherwise. Summary:
        - For the case heating capacity: h2 = h3 + capacity / m_flow
        - For the case cooling capacity:  h2 = h3 + (capacity + p_el * assumed_eta_mech) / m_flow

    Args:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        T_sc (float): Subcooling according to datasheet in K.
        T_sh (float): Superheating according to datasheet in K.
        capacity_definition (str): Definition of "capacity" in the datasheet. "cooling" or "heating".
        assumed_eta_mech (float): Assumed mechanical efficiency of the compressor (only needed if cooling).
        datasheet (str): Path of the modified datasheet.
        **kwargs:
            parameter_names (dict, optional):
                Dictionary to match internal parameter names (keys) to the names used in the table values.
                Default
                {
                    "m_flow": "Flow Rate(kg/h)",
                    "capacity": "Capacity(W)",
                    "input_power": "Input Power(W)"
                }
            sheet_name (str, optional): Name of the sheet in the datasheet. Defaults to None.
    """

    def __init__(self, N_max, V_h, T_sc, T_sh, capacity_definition, assumed_eta_mech, datasheet, **kwargs):
        super().__init__(N_max=N_max, V_h=V_h, datasheet=datasheet, **kwargs)
        self.T_sc = T_sc
        self.T_sh = T_sh
        if capacity_definition not in ["cooling", "heating"]:
            raise ValueError("capacity_definition has to be either 'heating' or 'cooling'")
        self._capacity_definition = capacity_definition
        self.assumed_eta_mech = assumed_eta_mech
        self.datasheet = datasheet

    def get_lambda_h(self, inputs: Inputs):
        """
        Get the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Volumetric efficiency.
        """
        p_outlet = self.get_p_outlet()

        n_abs = self.get_n_absolute(inputs.n)
        T_eva = self.med_prop.calc_state("PQ", self.state_inlet.p, 1).T - 273.15  # [°C]
        T_con = self.med_prop.calc_state("PQ", p_outlet, 0).T - 273.15  # [°C]

        if round((self.state_inlet.T - T_eva - 273.15), 2) != round(self.T_sh, 2):
            warnings.warn("The superheating of the given state is not "
                          "equal to the superheating of the datasheet. "
                          "State1.T_sh= " + str(round((self.state_inlet.T - T_eva - 273.15), 2)) +
                          ". Datasheet.T_sh = " + str(self.T_sh))
        # The datasheet has a given superheating temperature which can
        # vary from the superheating of the real state 1
        # which is given by the user.
        # Thus a new self.state_inlet_datasheet has to
        # be defined for all further calculations
        state_inlet_datasheet = self.med_prop.calc_state("PT", self.state_inlet.p, T_eva + 273.15 + self.T_sh)

        m_flow = self.get_parameter(T_eva, T_con, inputs.n, "m_flow") / 3600  # [kg/s]

        lambda_h = m_flow / (n_abs * state_inlet_datasheet.d * self.V_h)
        return lambda_h

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs):
        """
        Get the isentropic efficiency.

        Args:
            p_outlet (float): Outlet pressure in Pa.
            inputs (Inputs): Input parameters.

        Returns:
            float: Isentropic efficiency.
        """
        T_eva, T_con, state_inlet_datasheet, m_flow, capacity, p_el = self._calculate_values(
            p_2=p_outlet, inputs=inputs
        )

        h3 = self.med_prop.calc_state("PT", p_outlet, T_con + 273.15 - self.T_sc).h  # [J/kg]
        h2s = self.med_prop.calc_state("PS", p_outlet, state_inlet_datasheet.s).h  # [J/kg]

        if self._capacity_definition == "heating":
            h2 = h3 + capacity / m_flow  # [J/kg]
        else:
            h2 = h3 + (capacity + p_el * self.assumed_eta_mech) / m_flow  # [J/kg]

        if h2s > h2:
            raise ValueError("The calculated eta_s is above 1. You probably chose the wrong capacity_definition")

        eta_s = (h2s - state_inlet_datasheet.h) / (h2 - state_inlet_datasheet.h)
        return eta_s

    def get_eta_mech(self, inputs: Inputs):
        """
        Get the mechanical efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Mechanical efficiency.
        """
        p_outlet = self.get_p_outlet()

        if self._capacity_definition == "cooling":
            return self.assumed_eta_mech
        # Else heating
        T_eva, T_con, state_inlet_datasheet, m_flow, capacity, p_el = self._calculate_values(
            p_2=p_outlet, inputs=inputs
        )

        h3 = self.med_prop.calc_state("PT", p_outlet, T_con + 273.15 - self.T_sc).h  # [J/kg]
        h2 = h3 + capacity / m_flow  # [J/kg]

        eta_mech = p_el / (m_flow * (h2 - state_inlet_datasheet.h))
        return eta_mech

    def _calculate_values(self, p_2: float, inputs: Inputs):
        """
        Calculate intermediate values for efficiency calculations.

        Args:
            p_2 (float): Outlet pressure in Pa.
            inputs (Inputs): Input parameters.

        Returns:
            Tuple[float, float, State, float, float, float]: Intermediate values.
        """
        T_eva = self.med_prop.calc_state("PQ", self.state_inlet.p, 1).T - 273.15  # [°C]
        T_con = self.med_prop.calc_state("PQ", p_2, 0).T - 273.15  # [°C]

        state_inlet_datasheet = self.med_prop.calc_state("PT", self.state_inlet.p, T_eva + 273.15 + self.T_sh)

        m_flow = self.get_parameter(T_eva, T_con, inputs.n, "m_flow") / 3600  # [kg/s]
        capacity = self.get_parameter(T_eva, T_con, inputs.n, "capacity")  # [W]
        p_el = self.get_parameter(T_eva, T_con, inputs.n, "input_power")  # [W]
        return T_eva, T_con, state_inlet_datasheet, m_flow, capacity, p_el


class DataSheetCompressor(BaseTenCoefficientCompressor):
    """
    Compressor based on the ten coefficient method.

    Used table has to be in this format
    (order of the columns is not important).
    The values must be the same as in the example tabel.
    The column names can be different but must
    then be matched with the keyword argument parameter_names.
    (All typed in numbers are fictional placeholders)

          Isentropic      Volumetric        Mechanical      Isentropic              Mechanical
          Efficiency(-)   Efficiency(-)     Efficiency(-)   Efficiency(-)   ...     Efficiency(-)
    n     n1              n1                n1              n2              ...     n_last
    P1    42              12                243             32              ...     412
    ...   ...             ...               ...             ...             ...     ...
    P10   10              23                21              41              ...     2434

    Args:
        N_max (float): Maximal rotations per second of the compressor.
        V_h (float): Volume of the compressor in m^3.
        datasheet (str): Path of the datasheet file.
        **kwargs:
            parameter_names (dict, optional):
                Dictionary to match internal parameter names (keys) to the names used in the table values.
                Default
                {
                    "eta_s": "Isentropic Efficiency(-)",
                    "lambda_h": "Volumetric Efficiency(-)",
                    "eta_mech": "Mechanical Efficiency(-)"
                }
            sheet_name (str, optional): Name of the sheet in the datasheet. Defaults to None.
    """

    def __init__(self, N_max, V_h, datasheet, **kwargs):
        super().__init__(N_max=N_max, V_h=V_h, datasheet=datasheet, **kwargs)

    def get_lambda_h(self, inputs: Inputs):
        """
        Get the volumetric efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Volumetric efficiency.
        """
        p_outlet = self.get_p_outlet()
        T_eva = self.med_prop.calc_state("PQ", self.state_inlet.p, 0).T
        T_con = self.med_prop.calc_state("PQ", p_outlet, 0).T
        return self.get_parameter(T_eva, T_con, inputs.n, "lambda_h")

    def get_eta_isentropic(self, p_outlet: float, inputs: Inputs):
        """
        Get the isentropic efficiency.

        Args:
            p_outlet (float): Outlet pressure in Pa.
            inputs (Inputs): Input parameters.

        Returns:
            float: Isentropic efficiency.
        """
        T_eva = self.med_prop.calc_state("PQ", self.state_inlet.p, 0).T
        T_con = self.med_prop.calc_state("PQ", p_outlet, 0).T
        return self.get_parameter(T_eva, T_con, inputs.n, "eta_s")

    def get_eta_mech(self, inputs: Inputs):
        """
        Get the mechanical efficiency.

        Args:
            inputs (Inputs): Input parameters.

        Returns:
            float: Mechanical efficiency.
        """
        p_outlet = self.get_p_outlet()
        T_eva = self.med_prop.calc_state("PQ", self.state_inlet.p, 0).T
        T_con = self.med_prop.calc_state("PQ", p_outlet, 0).T
        return self.get_parameter(T_eva, T_con, inputs.n, "eta_mech")
