"""
Module which contains datamodels which are used in this library.
"""

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
            return {f"{k} in {v.unit} ({v.description})": v.value for k, v in self.items()}
        return {k: v.value for k, v in self.items()}


class FlowsheetState(VariableContainer):
    """
    This class is used to define the unique states of the flowsheet
    in the heat pump.

    The class is dynamic in the sense that attributes may be
    added during calculation of new flowsheet. This enables
    the easy adding of custom values to analyze the whole flowsheet
    and not restrict to a certain naming convention.
    """


class Inputs(VariableContainer):
    """
    Class defining inputs to calculate the FlowsheetState.

    While the inputs are pre-defined, you may add further ones
    using the `set` method.

    Args:
        n (float): Relative compressor speed between 0 and 1.
        T_eva_in (float): Secondary side evaporator inlet temperature.
        T_con_in (float): Secondary side condenser inlet temperature.
        m_flow_eva (float): Secondary side evaporator mass flow rate.
        m_flow_con (float): Secondary side condenser mass flow rate.
        dT_eva_superheating (float): Super-heating after evaporator.
        dT_con_subcooling (float): Subcooling after condenser.
        T_ambient (float): Ambient temperature of the machine.
        k_vapor_injection (float): Vapor injection coefficient.
    """

    def __init__(
            self,
            n: float = None,
            T_eva_in: float = None,
            T_con_in: float = None,
            m_flow_eva: float = None,
            m_flow_con: float = None,
            dT_eva_superheating: float = None,
            dT_con_subcooling: float = None,
            T_ambient: float = None,
            k_vapor_injection=None
    ):
        """
        Initializes an Inputs object with parameters representing external conditions
        for the vapor compression cycle.

        Args:
            n (float): Relative compressor speed between 0 and 1 (unit: -).
            T_eva_in (float): Secondary side evaporator inlet temperature (unit: K).
            T_con_in (float): Secondary side condenser inlet temperature (unit: K).
            m_flow_eva (float): Secondary side evaporator mass flow rate (unit: kg/s).
            m_flow_con (float): Secondary side condenser mass flow rate (unit: kg/s).
            dT_eva_superheating (float): Super-heating after evaporator (unit: K).
            dT_con_subcooling (float): Subcooling after condenser (unit: K).
            T_ambient (float): Ambient temperature of the machine (unit: K).
            k_vapor_injection (float): Vapor injection coefficient (unit: -).
        """
        super().__init__()
        self.set(
            name="n",
            value=n,
            unit="-",
            description="Relative compressor speed"
        )
        self.set(
            name="T_eva_in",
            value=T_eva_in,
            unit="K",
            description="Secondary side evaporator inlet temperature"
        )
        self.set(
            name="T_con_in",
            value=T_con_in,
            unit="K",
            description="Secondary side condenser inlet temperature"
        )
        self.set(
            name="m_flow_con",
            value=m_flow_con,
            unit="kg/s",
            description="Secondary side condenser mass flow rate"
        )
        self.set(
            name="m_flow_eva",
            value=m_flow_eva,
            unit="kg/s",
            description="Secondary side evaporator mass flow rate"
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
        if T_ambient is None:
            T_ambient = T_eva_in
        self.set(
            name="T_ambient",
            value=T_ambient,
            unit="K",
            description="Ambient temperature of machine"
        )
        self.set(
            name="k_vapor_injection",
            value=k_vapor_injection,
            unit="-",
            description="Vapor injection coefficient"
        )
