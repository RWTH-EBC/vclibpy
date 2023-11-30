"""
Module containing classes for thermodynamic state and transport properties.
"""
from vclibpy.datamodels import VariableContainer


__all__ = [
    'ThermodynamicState',
    'TransportProperties',
]


class ThermodynamicState:
    """
    Represents a thermodynamic state within a cycle.

    Notes:
        Does not necessarily need to have all state variables defined!

    Attributes:
        p (float): Pressure at the state in Pa.
        T (float): Temperature at the state in K.
        u (float): Inner energy at the state in J/kg.
        h (float): Enthalpy at the state in J/kg.
        s (float): Entropy at the state in J/(kg * K).
        v (float): Specific volume at the state in m^3/kg.
        q (float): Quality at the state (between 0 and 1).
        d (float): Density at the state in kg/m^3.

    Methods:
        __init__: Initializes the state class.
        __str__: Provides a string representation of the state.
        get_pretty_print: Formats the state with names, units, and descriptions.
    """

    def __init__(self,
                 p=None,
                 T=None,
                 u=None,
                 h=None,
                 s=None,
                 v=None,
                 q=None,
                 d=None):
        """
        Initializes a thermodynamic state.

        Args:
            p (float): Pressure at the state in Pa.
            T (float): Temperature at the state in K.
            u (float): Inner energy at the state in J/kg.
            h (float): Enthalpy at the state in J/kg.
            s (float): Entropy at the state in J/(kg * K).
            v (float): Specific volume at the state in m^3/kg.
            q (float): Quality at the state (between 0 and 1).
            d (float): Density at the state in kg/m^3.

        Notes:
            If only v or d is provided, the other attribute will be calculated. If both are given and they are similar,
            an error will be raised.
        """
        self.p = p
        self.T = T
        self.u = u
        self.h = h
        self.s = s
        self.v = v
        self.q = q
        self.d = d
        # Define density
        if v and d:
            if not round(1/v, 4) == round(d, 4):
                raise ValueError("At current state d and v do not match", d, v)
        elif v:
            self.d = 1/v
        elif d:
            self.v = 1/d

    def __str__(self):
        """
        Returns a string representation of the state.
        """
        return ";".join([f"{k}={v}" for k, v in self.__dict__.items()])

    def get_pretty_print(self):
        """
        Provides a formatted representation of the state with names, units, and descriptions.
        """
        _container = VariableContainer()
        _container.__class__.__name__ = self.__class__.__name__
        _container.set(name="p", value=self.p, unit="Pa", description="Pressure")
        _container.set(name="T", value=self.T, unit="K", description="Temperature")
        _container.set(name="u", value=self.u, unit="J/kg", description="Inner energy")
        _container.set(name="h", value=self.h, unit="J/kg", description="Enthalpy")
        _container.set(name="s", value=self.s, unit="J/(kg*K)", description="Entropy")
        _container.set(name="v", value=self.v, unit="m^3/kg", description="Specific volume")
        _container.set(name="q", value=self.q, unit="-", description="Quality")
        _container.set(name="d", value=self.d, unit="kg/m^3", description="Density")
        return str(_container)


class TransportProperties:
    """
    Represents transport properties at a specific thermodynamic state.

    Attributes:
        lam (float): Thermal conductivity in W/(m*K).
        dyn_vis (float): Dynamic viscosity in Pa*s.
        kin_vis (float): Kinematic viscosity in m^2/s.
        Pr (float): Prandtl number.
        cp (float): Isobaric specific heat capacity in J/(kg*K).
        cv (float): Isochoric specific heat capacity in J/(kg*K).
        beta (float): Thermal expansion coefficient in 1/K.
        sur_ten (float): Surface tension in N/m.
        ace_fac (float): Acentric factor.
        state (ThermodynamicState): The state the transport properties belong to.

    Methods:
        __init__: Initializes the transport properties class.
        __str__: Provides a string representation of the transport properties.
        get_pretty_print: Formats the properties with names, units, and descriptions.
    """

    def __init__(self,
                 lam=None,
                 dyn_vis=None,
                 kin_vis=None,
                 pr=None,
                 cp=None,
                 cv=None,
                 beta=None,
                 sur_ten=None,
                 ace_fac=None,
                 state=None):
        """
        Initializes transport properties.

        Args:
            lam (float): Thermal conductivity in W/(m*K).
            dyn_vis (float): Dynamic viscosity in Pa*s.
            kin_vis (float): Kinematic viscosity in m^2/s.
            pr (float): Prandtl number.
            cp (float): Isobaric specific heat capacity in J/(kg*K).
            cv (float): Isochoric specific heat capacity in J/(kg*K).
            beta (float): Thermal expansion coefficient in 1/K.
            sur_ten (float): Surface tension in N/m.
            ace_fac (float): Acentric factor.
            state (ThermodynamicState): The state the transport properties belong to.
        """
        self.lam = lam
        self.dyn_vis = dyn_vis
        self.kin_vis = kin_vis
        self.Pr = pr
        self.cp = cp
        self.cv = cv
        self.beta = beta
        self.sur_ten = sur_ten
        self.ace_fac = ace_fac
        self.state = state

    def __str__(self):
        """
        Returns a string representation of the transport properties.
        """
        return ";".join([f"{k}={v}" for k, v in self.__dict__.items()])

    def get_pretty_print(self):
        """
        Provides a formatted representation of the properties with names, units, and descriptions.
        """
        _container = VariableContainer()
        _container.__class__.__name__ = self.__class__.__name__
        _container.set(name="lam", value=self.lam, unit="W/(m*K)", description="Thermal conductivity")
        _container.set(name="dyn_vis", value=self.dyn_vis, unit="Pa*s", description="Dynamic viscosity")
        _container.set(name="kin_vis", value=self.kin_vis, unit="m^2/s", description="Kinematic viscosity")
        _container.set(name="pr", value=self.Pr, unit="-", description="Prandtl number")
        _container.set(name="cp", value=self.cp, unit="J/(kg*K)", description="Isobaric specific heat capacity")
        _container.set(name="cv", value=self.cv, unit="J/(kg*K)", description="Isochoric specific heat capacity")
        _container.set(name="beta", value=self.beta, unit="1/K", description="Thermal expansion coefficient")
        _container.set(name="sur_ten", value=self.sur_ten, unit="N/m", description="Surface tension")
        _container.set(name="ace_fac", value=self.ace_fac, unit="-", description="Acentric factor")
        return str(_container)
