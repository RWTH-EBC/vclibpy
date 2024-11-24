import logging
from abc import ABC
from typing import Union

import numpy as np
import pandas as pd
from vclibpy.components.compressors.compressor import Compressor
from vclibpy.datamodels import Inputs

logger = logging.getLogger(__name__)


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
                    "eta_is": "Isentropic Efficiency(-)",
                    "lambda_h": "Volumentric Efficiency(-)",
                    "eta_mech": "Mechanical Efficiency(-)"
                }
            sheet_name (str, optional): Name of the sheet in the datasheet. Defaults to None.
            extrapolate (str, optional):
                Method to handle extrapolation of data.
                Default "hold" means no extrapolation
    """

    def __init__(self, N_max, V_h, datasheet, **kwargs):
        """
        Initialize the BaseTenCoefficientCompressor.
        """

        super().__init__(N_max, V_h)
        sheet_name = kwargs.get('sheet_name', None)
        if str(datasheet).endswith(".xlsx"):
            self.md = pd.read_excel(datasheet, sheet_name=sheet_name)
        else:
            self.md = pd.read_csv(datasheet)
        parameter_names = kwargs.get('parameter_names', None)
        if parameter_names is None:
            self.parameter_names = {
                "m_flow": "Flow Rate(kg/h)",
                "capacity": "Capacity(W)",
                "input_power": "Input Power(W)",
                "eta_is": "Isentropic Efficiency(-)",
                "lambda_h": "Volumentric Efficiency(-)",
                "eta_mech": "Mechanical Efficiency(-)"
            }
        else:
            self.parameter_names = parameter_names
        self.extrapolate = kwargs.get("extrapolate", "hold")

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

        return self._interpolate(self.get_n_absolute(n), n_list, param_list)  # linear interpolation

    def _interpolate(self, x_new, x, y):
        if self.extrapolate == "hold":
            # linear interpolation, no extrapolation
            return np.interp(x_new, x, y)
        if self.extrapolate == "linear":
            return linear_interpolate_extrapolate(x_new, x, y)
        raise KeyError(f"Given extrapolate option '{self.extrapolate}' is not supported!")


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
        assumed_eta_mech (float, callable): Assumed mechanical efficiency of the compressor (only needed if cooling).
            If you pass a funtion, it must have this signature `eta_mech(self, p_outlet, inputs)`
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

    def __init__(
            self,
            N_max,
            V_h,
            T_sh,
            capacity_definition,
            datasheet,
            T_sc: float = None,
            assumed_eta_mech: Union[float, callable] = None,
            scaling_factor: float = 1,
            **kwargs
    ):
        super().__init__(N_max=N_max, V_h=V_h, datasheet=datasheet, **kwargs)
        if capacity_definition == "cooling" and assumed_eta_mech is None:
            raise ValueError("capacity_definition cooling requires an assumption for eta_mech")
        if capacity_definition == "heating" and T_sc is None:
            raise ValueError("capacity_definition heating requires an assumption for T_sc")
        self.T_sc = T_sc
        self.T_sh = T_sh
        if capacity_definition not in ["cooling", "heating"]:
            raise ValueError("capacity_definition has to be either 'heating' or 'cooling'")
        self._capacity_definition = capacity_definition
        # Don't use lambda function to cast float as a function,
        # as local functions are not pickable for multiprocessing
        self.assumed_eta_mech = assumed_eta_mech
        self.scaling_factor = scaling_factor

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

        # if round((self.state_inlet.T - T_eva - 273.15), 2) != round(self.T_sh, 2):
        #     logger.warning("The superheating of the given state is not "
        #                    "equal to the superheating of the datasheet. "
        #                    "State1.T_sh= %s. Datasheet.T_sh = %s",
        #                    round((self.state_inlet.T - T_eva - 273.15), 2), self.T_sh)

        # The datasheet has a given superheating temperature which can
        # vary from the superheating of the real state 1
        # which is given by the user.
        # Thus a new self.state_inlet_datasheet has to
        # be defined for all further calculations

        if self.T_sh != 0:
            state_inlet_datasheet = self.med_prop.calc_state("PT", self.state_inlet.p, T_eva + 273.15 + self.T_sh)
        else:
            state_inlet_datasheet = self.med_prop.calc_state("PQ", self.state_inlet.p, 1)

        m_flow = self.get_parameter(T_eva, T_con, inputs.n, "m_flow") / 3600 * self.scaling_factor  # [kg/s]

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
        T_con, state_inlet_datasheet, m_flow, capacity, p_el = self._calculate_values(
            p_2=p_outlet, inputs=inputs
        )

        h2s = self.med_prop.calc_state("PS", p_outlet, state_inlet_datasheet.s).h  # [J/kg]

        if callable(self.assumed_eta_mech):
            eta_mech = self.assumed_eta_mech(self=self, p_outlet=p_outlet, inputs=inputs)
        else:
            eta_mech = self.assumed_eta_mech

        if self._capacity_definition == "heating":
            if self.T_sc != 0:
                h3 = self.med_prop.calc_state("PT", p_outlet, T_con + 273.15 - self.T_sc).h  # [J/kg]
            else:
                h3 = self.med_prop.calc_state("PQ", p_outlet, 0).h  # [J/kg]
            h2 = h3 + capacity / m_flow  # [J/kg]
        else:
            h2 = state_inlet_datasheet.h + (p_el * eta_mech) / m_flow  # [J/kg]

        eta_is = (h2s - state_inlet_datasheet.h) / (h2 - state_inlet_datasheet.h)
        if eta_is > 0.8:
            logger.warning(
                f"Calculated eta_is is {eta_is * 100} %, which is higher than "
                f"typical maximal values of up to, e.g., 80 %. "
                "You either chose the wrong capacity_definition, "
                "or your assumed eta_mech is also not realistic.",
            )
        return eta_is

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
            if callable(self.assumed_eta_mech):
                return self.assumed_eta_mech(self=self, p_outlet=p_outlet, inputs=inputs)
            return self.assumed_eta_mech

        # Else heating
        T_con, state_inlet_datasheet, m_flow, capacity, p_el = self._calculate_values(
            p_2=p_outlet, inputs=inputs
        )
        if self.T_sc != 0:
            h3 = self.med_prop.calc_state("PT", p_outlet, T_con + 273.15 - self.T_sc).h  # [J/kg]
        else:
            h3 = self.med_prop.calc_state("PQ", p_outlet, 0).h  # [J/kg]

        h2 = h3 + capacity / m_flow  # [J/kg]

        eta_mech = (m_flow * (h2 - state_inlet_datasheet.h)) / p_el
        return eta_mech

    def _calculate_values(self, p_2: float, inputs: Inputs):
        """
        Calculate intermediate values for efficiency calculations.

        Args:
            p_2 (float): Outlet pressure in Pa.
            inputs (Inputs): Input parameters.

        Returns:
            Tuple[float, State, float, float, float]: Intermediate values.
        """
        T_eva = self.med_prop.calc_state("PQ", self.state_inlet.p, 1).T - 273.15  # [°C]
        T_con = self.med_prop.calc_state("PQ", p_2, 0).T - 273.15  # [°C]

        if self.T_sh != 0:
            state_inlet_datasheet = self.med_prop.calc_state("PT", self.state_inlet.p, T_eva + 273.15 + self.T_sh)
        else:
            state_inlet_datasheet = self.med_prop.calc_state("PQ", self.state_inlet.p, 1)

        m_flow = self.get_parameter(T_eva, T_con, inputs.n, "m_flow") / 3600 * self.scaling_factor  # [kg/s]
        capacity = self.get_parameter(T_eva, T_con, inputs.n, "capacity") * self.scaling_factor  # [W]
        p_el = self.get_parameter(T_eva, T_con, inputs.n, "input_power") * self.scaling_factor  # [W]
        return T_con, state_inlet_datasheet, m_flow, capacity, p_el


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
                    "eta_is": "Isentropic Efficiency(-)",
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
        return self.get_parameter(T_eva, T_con, inputs.n, "eta_is")

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


def linear_interpolate_extrapolate(x_new, x, y):
    """
    Util function for linear 1D extrapolation or interpolation.
    Used to avoid scipy requirement.

    x_new: points where to interpolate/extrapolate
    x: known x values
    y: known y values
    """
    y_new = np.interp(x_new, x, y)

    # Handle left extrapolation
    left_mask = x_new < x[0]
    if x_new < x[0]:
        slope = (y[1] - y[0]) / (x[1] - x[0])
        y_new = y[0] + slope * (x_new - x[0])

    # Handle right extrapolation
    if x_new > x[-1]:
        slope = (y[-1] - y[-2]) / (x[-1] - x[-2])
        y_new = y[-1] + slope * (x_new - x[-1])

    return y_new
