"""
Module which contains datamodels which are used in this library.
"""
import abc
from dataclasses import dataclass
from typing import Dict, Any
from copy import deepcopy


@dataclass
class Variable:
    """
    Class for a variable used in analysis.

    Args:
        name (str): The name of the variable.
        value (float): The numerical value of the variable.
        unit (str): The unit of measurement for the variable (optional).
        description (str): A description of the variable (optional).
    """
    name: str
    value: float
    unit: str = None
    description: str = None


class VariableContainer:
    """
    Class which holds Variables to be used anywhere in the models.

    This class enables dynamic addition of Variables.
    """

    def __init__(self):
        self._variables: Dict[str, Variable] = {}

    def __str__(self):
        return f"{self.__class__.__name__}:\n" + "\n".join(
            [f"{var.name}={var.value} {var.unit} ({var.description})"
             for var in self._variables.values()]
        )

    def set(self, name: str, value: float, unit: str = None, description: str = None):
        """
        Add or update a Variable in the container.

        Args:
            name (str): The name of the variable.
            value (float): The numerical value of the variable.
            unit (str): The unit of measurement for the variable (optional).
            description (str): A description of the variable (optional).
        """
        if name in self._variables:
            self._variables[name].value = value
        else:
            self._variables[name] = Variable(
                name=name, value=value, unit=unit, description=description
            )

    def __getattr__(self, item):
        """
        Overwrite the dunder method to enable usage of e.g.
        fs_state.COP
        """
        if item in {'__getstate__', '__setstate__'}:
            return super().__getattr__(self, item)
        if item in self._variables:
            return self._variables.get(item).value
        raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{item}'")

    def get_variable_names(self) -> list:
        """
        Get the names of all variables in the container.

        Returns:
            list: A list of variable names.
        """
        return list(self._variables.keys())

    def get_variables(self):
        """
        Get all variables in the container.

        Returns:
            Dict[str, Variable]: A dictionary of variable names and Variable objects.
        """
        return self._variables

    def items(self):
        """
        Get items from the container.

        Returns:
            Dict[str, Variable]: A dictionary of variable names and Variable objects.
        """
        return self._variables.items()

    def get(self, name: str, default: Any = None):
        """
        Get the Variable with the specified name.

        Args:
            name (str): The name of the variable.
            default (Any): Default value to return if the variable is not found.

        Returns:
            Variable: The Variable object.

        """
        return self._variables.get(name, default)

    def copy(self):
        """
        Return a deepcopy of the instance as the variable dict is mutable.

        Returns:
            VariableContainer: A deepcopy of the VariableContainer instance.
        """
        return deepcopy(self)

    def convert_to_str_value_format(self, with_unit_and_description: bool) -> Dict[str, float]:
        """
        Returns a dictionary with a str: float format which is handy when storing results
        in files like .csv or .xlsx.

        Args:
            with_unit_and_description (bool): When False, only the name is in the string.

        Returns:
            dict: Containing all variables and values.
        """
        if with_unit_and_description:
            return {f"{k} in {v.unit} ({v.description})": v.value for k, v in self.items_not_none()}
        return {k: v.value for k, v in self.items_not_none()}

    def get_name(self):
        """Get the name based on variable names and rounded values"""
        return ";".join([k + "=" + str(round(v.value, 3)).replace(".", "_")
                         for k, v in self.items_not_none()])

    def items_not_none(self):
        """Get all items where the value is not None"""
        return {k: v for k, v in self.items() if v.value is not None}.items()


class FlowsheetState(VariableContainer):
    """
    This class is used to define the unique states of the flowsheet
    in the heat pump.

    The class is dynamic in the sense that attributes may be
    added during calculation of new flowsheet. This enables
    the easy adding of custom values to analyze the whole flowsheet
    and not restrict to a certain naming convention.
    """


class HeatExchangerInputs(VariableContainer):
    """
    Class defining inputs for a heat exchanger.

    Note that some inputs presuppose each other.
    You may only specify values which follow this equation:
    Q = m_flow * cp * abs(T_out - T_in) = m_flow * cp * dT

    Moreover, the amount of required inputs depend on the
    control inputs you specify.
    For example, when providing dT_superheating, dT_subcooling, and n,
    both at least two arguments must be free.
    However, you always have to specify either T_in or T_out to define
    the absolute temperature levels.

    Args:
        T_in (float):
            Secondary side inlet temperature in [K].
        T_out (float):
            Secondary side outlet temperature in [K].
        dT (float):
            Secondary side temperature difference in between inlet and outlet[K].
            May be provided instead of m_flow or both T_in and T_out.
        m_flow (float):
            Secondary side mass flow rate in [kg/s].
            May be provided instead of m_flow or both T_in and T_out.
        Q (float):
            Heat flow rate in [W].
            May be provided
        T_ambient (float):
            Ambient temperature of the heat exchanger in [K].
            Only used if the heat exchnager models account for
            heat losses.
    """

    def __init__(
            self,
            T_in: float = None,
            T_out: float = None,
            dT: float = None,
            m_flow: float = None,
            T_ambient: float = None
    ):
        """
        Initializes object with parameters representing external conditions
        for the heat exchanger.
        """
        super().__init__()
        # Handle over specified cases m_flow and dT
        if T_in is None and T_out is None:
            raise ValueError("You either have to specify T_in or T_out")
        elif T_in is None and T_out is not None:
            self.uses_outlet = True
            if dT is not None:
                T_in = T_out - dT
                self.uses_inlet = True
            else:
                self.uses_inlet = False
        elif T_in is not None and T_out is None:
            self.uses_inlet = True
            if dT is not None:
                T_out = T_in + dT
                self.uses_outlet = True
            else:
                self.uses_outlet = False
        else:
            self.uses_outlet = True
            self.uses_inlet = True
            if dT is None:
                dT = T_out - T_in
            else:
                if dT != T_out - T_in:
                    raise ValueError(f"Given {dT=} does not match {T_in=} and {T_out=}")
        if m_flow is not None and dT is not None:
            raise ValueError("You can either specify the temperature difference or m_flow, not both")
        self.set(
            name="T_in",
            value=T_in,
            unit="K",
            description="Secondary side inlet temperature"
        )
        self.set(
            name="T_out",
            value=T_out,
            unit="K",
            description="Secondary side outlet temperature"
        )
        self.set(
            name="dT",
            value=dT,
            unit="K",
            description="Secondary side temperature difference"
        )
        self.set(
            name="m_flow",
            value=m_flow,
            unit="kg/s",
            description="Secondary side mass flow rate"
        )
        # Handle optional T_ambient
        if T_ambient is None:
            T_ambient = self.T
        self.set(
            name="T_ambient",
            value=T_ambient,
            unit="K",
            description="Ambient temperature of machine"
        )

    @property
    def T(self):
        """Returns either T_in, T_out, or the mean, depending on what is specified."""
        if self.uses_inlet and self.uses_outlet:
            return (self.T_in + self.T_out) / 2
        if self.uses_inlet:
            return self.T_in
        return self.T_out

    def get_all_inputs(self, cp: float, Q: float):
        # Case with both inlet and outlet
        if self.uses_inlet and self.uses_outlet:
            T_in = self.T_in
            T_out = self.T_out
            m_flow = Q / cp / self.dT
        elif self.uses_inlet:
            T_in = self.T_in
            T_out = self.T_in + Q / cp / self.m_flow
            m_flow = self.m_flow
        else:  # self.uses_outlet is always True in this case
            T_out = self.T_out
            T_in = T_out - Q / cp / self.m_flow
            m_flow = self.m_flow
        dT = T_out - T_in
        if m_flow is None:
            raise ValueError("Provided inputs are insufficient to calculate m_flow")
        return T_in, T_out, dT, m_flow


class ControlInputs(VariableContainer, abc.ABC):
    """
    Abstract class defining inputs to control the
    vapor compression machine.
    """


class RelativeCompressorSpeedControl(ControlInputs):
    """
    Class defining inputs to control the
    vapor compression machine using the relative compressor
    speed.
    Furthermore, superheating and subcooling levels must be
    specified.

    Args:
        n (float): Relative compressor speed between 0 and 1 (unit: -).
        dT_eva_superheating (float): Super-heating after evaporator (unit: K).
        dT_con_subcooling (float): Subcooling after condenser (unit: K).
    """

    def __init__(
            self,
            n: float,
            dT_eva_superheating: float,
            dT_con_subcooling: float,
    ):
        """
        Initializes a ControlInputs object with parameters representing external conditions
        to control the vapor compression cycle.
        """
        super().__init__()
        self.set(
            name="n",
            value=n,
            unit="-",
            description="Relative compressor speed"
        )
        self.set(
            name="dT_eva_superheating",
            value=dT_eva_superheating,
            unit="K",
            description="Super-heating after evaporator"
        )
        self.set(
            name="dT_con_subcooling",
            value=dT_con_subcooling,
            unit="K",
            description="Subcooling after condenser"
        )


class CondenserPowerControl(ControlInputs):
    """
    Class defining inputs to control the
    vapor compression machine using the condenser heat flow rate.
    Furthermore, superheating and subcooling levels must be
    specified.

    Args:
        Q_con (float): Condenser heat flow rate (unit: W).
        dT_eva_superheating (float): Super-heating after evaporator (unit: K).
        dT_con_subcooling (float): Subcooling after condenser (unit: K).
    """

    def __init__(
            self,
            Q_con: float,
            dT_eva_superheating: float,
            dT_con_subcooling: float,
    ):
        """
        Initializes a ControlInputs object with parameters representing external conditions
        to control the vapor compression cycle.
        """
        super().__init__()
        self.set(
            name="Q_con",
            value=Q_con,
            unit="-",
            description="Relative compressor speed"
        )
        self.set(
            name="dT_eva_superheating",
            value=dT_eva_superheating,
            unit="K",
            description="Super-heating after evaporator"
        )
        self.set(
            name="dT_con_subcooling",
            value=dT_con_subcooling,
            unit="K",
            description="Subcooling after condenser"
        )


class Inputs:
    """
    Class defining inputs to calculate the FlowsheetState.

    Args:
        control (ControlInputs):
            Input parameters for control.
            Depending on your choice of control inputs,
            you may have to adjust the required values
            in the heat exchanger inputs.
            Also, based on the control setting, the steady state
            calculation will use different algorithms.
        evaporator (HeatExchangerInputs):
            Input parameters for evaporator
        condenser (HeatExchangerInputs):
            Input parameters for condenser
    """

    def __init__(
            self,
            control: ControlInputs = None,
            evaporator: HeatExchangerInputs = None,
            condenser: HeatExchangerInputs = None,
    ):
        """
        Initializes an Inputs object with parameters representing external conditions
        of the vapor compression cycle.
        """
        self.control = control
        self.evaporator = evaporator
        self.condenser = condenser


if __name__ == '__main__':
    print(HeatExchangerInputs(T_in=273.15, dT=5))
    print("-----")
    print(HeatExchangerInputs(T_out=273.15, dT=5))
    print("-----")
    print(HeatExchangerInputs(T_in=273.15, T_out=278.15, dT=5))
    print("-----")
    print(HeatExchangerInputs(T_in=273.15, m_flow=5))
    print("-----")
    print(HeatExchangerInputs(T_out=273.15, m_flow=5))
    try:
        HeatExchangerInputs(dT=5)
    except ValueError as err:
        print("-----")
        print(err)
    try:
        HeatExchangerInputs(m_flow=5)
    except ValueError as err:
        print("-----")
        print(err)
    try:
        HeatExchangerInputs(T_in=273.15, T_out=278.15, m_flow=5)
    except ValueError as err:
        print("-----")
        print(err)
    print(HeatExchangerInputs(T_out=273.15, dT=5).get_all_inputs(cp=4184, Q=1000))
