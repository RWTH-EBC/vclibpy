"""Module with wrappers to access and handle media property databases.

This module provides interfaces to load media properties using various wrappers and
handle calculations related to media properties.

Classes:
    MedProp: Base class for all media property interfaces.

Functions:
    get_two_phase_limits: Return the states of the boundaries of the two-phase section for a given fluid.

"""
import abc
import logging
import math
import warnings
from typing import List
import numpy as np
from scipy.optimize import brentq as _brentq


from vclibpy.media import ThermodynamicState, TransportProperties, MedProp, OilProp, RefProp


logger = logging.getLogger(__name__)


class LubricantFitting(OilProp):
    """

    Base class for Lubricant-Refrigerant-Mixtures

    This class serves as the base for defining interfaces to access and compute media properties.

    Methods:
        calc_state: Calculate the thermodynamic state based on mode and state variables.
        calc_transport_properties: Calculate transport properties for a given state.
        get_critical_point: Retrieve critical point information.
        get_molar_mass: Retrieve molar mass information.
        get_two_phase_limits: Retrieve the two-phase limits for plotting.
        calc_mean_transport_properties: Calculate the average transport properties for given states.
    """
    _fluid_mapper = {}
    _oil_mapper = {}
    def __init__(self, fluid_name: str, lub_name: str): #, lub_frac: float):
        """Initialize the MedProp class instance.

        Args:
            fluid_name (str): The name of the refrigerant.
            lub_name (str): The name of the lubricant.
        """
        # Check if better internal names exist (e.g. air is modelled as air.ppf)
        super().__init__(fluid_name, lub_name)
        self.fluid_name = self._fluid_mapper.get(fluid_name, fluid_name)
        self.lub_name = self._oil_mapper.get(lub_name, lub_name)
        #self.lub_frac = lub_frac
        self._two_phase_limits: dict = None
        self.refrigerant_prop = RefProp(fluid_name=fluid_name)
        if fluid_name == "propane":
            if lub_name == "LPG 68": # Parameters for LPG 68
                self.T_crit = 369.89  # K (for reduced temperature)
                self.M_rfg = 43.01
                self.M_oil = 400.0

                # Parameters for calculation of saturation pressure
                self.sat_p_a = 7.11
                self.sat_p_b = -2.34
                self.sat_p_c = -4.96
                self.sat_p_d = -4.02
                self.sat_p_e = -0.436
                self.sat_p_f = 5.05

                # parameters for calculation of dynamic viscosity
                self.dyn_visc_a = 9.1741E+00
                self.dyn_visc_b = -3.9327E+00
                self.dyn_visc_c = 1.4495E-01
                self.dyn_visc_d = -3.6884E-01
                self.dyn_visc_e = -7.2671E-01
                self.dyn_visc_f = 7.5201E-03
                self.dyn_visc_g = 1.3094E+00
                self.dyn_visc_h = 1.5980E+00
                self.dyn_visc_i = -7.9215E-01

                #Parameters for calculation of density
                self.rho_a = 1.1736E+03
                self.rho_b = -4.3812E-01
                self.rho_c = -6.2498E-04
                self.rho_d = 1.7379E+02
                self.rho_e = -5.8711E+00
                self.rho_f = 9.1282E-03
                self.rho_g = 3.0804E+01
                self.rho_h = 2.6090E+00
                self.rho_i = -5.9097E-03

                #parameters for calculation of kinematic viscosity
                self.kin_visc_a = 1.3893E+01
                self.kin_visc_b = -7.8106E+00
                self.kin_visc_c = 9.4155E-01
                self.kin_visc_d = -5.2681E-02
                self.kin_visc_e = -1.6857E+00
                self.kin_visc_f = 3.5412E-01
                self.kin_visc_g = -4.0291E-02
                self.kin_visc_h = -1.2629E-01
                self.kin_visc_i = 2.0426E-01

            elif lub_name == "LPG 100": # Parameters for LPG 100
                self.T_crit = 369.89  # K (for reduced temperature)
                self.M_rfg = 43.01
                self.M_oil = 480.0

                # Parameters for calculation of saturation pressure
                self.sat_p_a = 6.87
                self.sat_p_b = -2.64
                self.sat_p_c = -4.80
                self.sat_p_d = -18.9
                self.sat_p_e = 34.4
                self.sat_p_f = -14.5

                # parameters for calculation of dynamic viscosity
                self.dyn_visc_a = 1.5002E+01
                self.dyn_visc_b = -8.8595E+00
                self.dyn_visc_c = 1.1898E+00
                self.dyn_visc_d = 1.0678E+00
                self.dyn_visc_e = -1.3895E+00
                self.dyn_visc_f = 9.5803E-03
                self.dyn_visc_g = -1.3753E+00
                self.dyn_visc_h = -1.7613E+00
                self.dyn_visc_i = 1.2581E+00

                #Parameters for calculation of density
                self.rho_a = 1.1829E+03
                self.rho_b = -5.23432E-01
                self.rho_c = -4.3074E-04
                self.rho_d = 1.3561E+02
                self.rho_e = -5.2478E+00
                self.rho_f = 6.5347E-03
                self.rho_g = 1.8586E+01
                self.rho_h = 6.8091E+00
                self.rho_i = 5.5917E-03

                #parameters for calculation of kinematic viscosity
                self.kin_visc_a = 1.9289E+01
                self.kin_visc_b = -1.2371E+01
                self.kin_visc_c = 1.9087E+00
                self.kin_visc_d = 1.0470E+00
                self.kin_visc_e = -2.3514E+00
                self.kin_visc_f = 4.1481E-01
                self.kin_visc_g = -1.7803E+00
                self.kin_visc_h = -2.3293E+00
                self.kin_visc_i = 1.6092E+00
            else:
                available_lubricants = ["LPG 68", "LPG 100"]
                assert lub_name in available_lubricants, f'Given lubricant {lub_name} is not in available phases'
        else:
            available_refrigerants = ["propane"]
            assert fluid_name in available_refrigerants, f'Given lubricant {fluid_name} is not in available phases'


 
    def calc_state(self, mode: str, phase: str, var1: float, var2: float, lub_frac=0):
        """Calculate the thermodynamic state based on the specified mode and state variables.

        This function calculates the thermodynamic state based on the chosen mode and provided state variables.
        The input state variables need to be in SI units.
        available phases:
            - vapor
            - liquid

        Notes:
            - PT does not work when the state might fall within the two-phase region.
            - Only functions for density are implemented. In cases where you know the specific volume, use the density
              functions with the inverse value.
            - Quality (q) may have values outside the 'physical' scope:
                - q = -998: Subcooled liquid
                - q = 998: Superheated vapor
                - q = 999: Supercritical state

        Possible modes include:
            - "PD": Pressure, Density
            - "PH": Pressure, Enthalpy
            - "PQ": Pressure, Quality
            - "PS": Pressure, Entropy
            - "PT": Pressure, Temperature
            - "PU": Pressure, Internal Energy
            - "TD": Temperature, Density
            - "TH": Temperature, Enthalpy
            - "TQ": Temperature, Quality
            - "TS": Temperature, Entropy
            - "TU": Temperature, Internal Energy
            - "DH": Density, Enthalpy
            - "DS": Density, Entropy
            - "DU": Density, Internal Energy

        Args:
            mode (str): Defines the given input state variables (see possible modes above).
            var1 (float): Value of the first state variable (as specified in the mode) in SI units.
            var2 (float): Value of the second state variable (as specified in the mode) in SI units.
            lub_frac (float): Mass fraction of lubricant in the mixture.

        Returns:
            ThermodynamicState: A ThermodynamicState instance with state variables.

        Raises:
            AssertionError: If the given mode is not within the available options.
        """
        if phase == "vapor":
            state = self.refrigerant_prop.calc_state(mode=mode, var1=var1, var2=var2)

            available_options = ['PD', 'PH', 'PQ', 'PS', 'PT',
                             'PU', 'TD', 'TH', 'TQ', 'TS',
                             'TU', 'DH', 'DS', 'DU', ]
        elif phase == "liquid":
            available_options = []
        available_phases = ['vapor']
        assert phase in available_phases, f'Given phase {phase} is not in available phases'
        assert mode in available_options, f'Given mode {mode} is not in available options'

        return state

    def calc_transport_properties(self, state: ThermodynamicState, phase: str, lub_frac=0):
        """Calculate the transport properties for the given state.

               Args:
                   state (ThermodynamicState): The current thermodynamic state.
                   lub_frac (float): Mass fraction of lubricant in the mixture.

               Returns:
                   TransportProperties: An instance of TransportProperties.
               """
        if phase == "vapor":
            props = self.refrigerant_prop.calc_transport_properties(state=state)


        elif phase == "liquid":
            T = state.T
            logT = math.log10(T)

            def calc_pressure(w, T):
                """
                Mixture pressure fitting equation (Pa), using saturation pressure from CoolProp if available.
                """
                Tr = T / self.T_crit
                # mass fraction -> mole fraction (refrigerant basis)
                x = w / (w + (1 - w) * (self.M_rfg / self.M_oil))
                psat = self.refrigerant_prop.calc_state("TQ", T, 1).p  # Pa
                if not np.isfinite(psat):
                    # Psat unavailable (e.g., no CoolProp or supercritical) → return NaN so solver will fail gracefully
                    return float('nan')

                term = self.sat_p_a + self.sat_p_b * Tr + self.sat_p_c * Tr ** 2 + self.sat_p_d * x + self.sat_p_e * x * Tr + self.sat_p_f * x * Tr ** 2
                p = x * psat + x * (1 - x) * term * psat  # bar
                return p

            # calculate refrigerant mass fraction
            """
                Given pressure (bar) and temperature (°C), solve for propane mass fraction w in (0,1).
                Uses SciPy brentq if available; otherwise bisection fallback.
                """

            def objective(x):
                return calc_pressure(x, state.T) - state.p

            # Try a safe bracket in [0.001, 0.99]
            a, b = 0.001, 0.999
            try:
                fa, fb = objective(a), objective(b)
                if not (np.isfinite(fa) and np.isfinite(fb)):
                    return None
                if fa * fb > 0:
                    # No sign change → cannot guarantee a root
                    return None
                w = _brentq(objective, a, b, maxiter=400)

            except Exception:
                return None


            #claiculate density
            rho = ((self.rho_a + self.rho_b*T + self.rho_c*T**2) +
                   w*(self.rho_d + self.rho_e*T + self.rho_f*T**2) +
                   w**2*(self.rho_g + self.rho_h*T + self.rho_i*T**2))

            #calculate dynamic viscosity
            y = ((self.dyn_visc_a + self.dyn_visc_b * logT + self.dyn_visc_c * logT ** 2) +
                 w * (self.dyn_visc_d + self.dyn_visc_e * logT + self.dyn_visc_f * logT ** 2) +
                 w ** 2 * (self.dyn_visc_g + self.dyn_visc_h * logT + self.dyn_visc_i * logT ** 2))
            dyn_vis = math.pow(10.0, math.pow(10.0, y)) - 0.7

            # calculate kinematic viscosity

            y = ((self.kin_visc_a + self.kin_visc_b * logT + self.kin_visc_c * logT ** 2) +
                 w * (self.kin_visc_d + self.kin_visc_e * logT + self.kin_visc_f * logT ** 2) +
                 w ** 2 * (self.kin_visc_g + self.kin_visc_h * logT + self.kin_visc_i * logT ** 2))

            kin_vis = math.pow(10, math.pow(10, y)) - 0.7

            state.d = rho
            state.v = 1/rho
            props = TransportProperties(lam = float('nan'),
                dyn_vis = dyn_vis, # in mPas
                kin_vis = kin_vis,
                pr = float('nan'),
                cp = float('nan'),
                cv = float('nan'),
                beta = float('nan'),
                sur_ten = float('nan'),
                ace_fac = float('nan'),
                state = state)

        return props

    def get_critical_point(self):
        """
        Minimal compatibility implementation required by the abstract base class.
        For the current use in the compressor model, the refrigerant critical point
        is sufficient.
        """
        return self.refrigerant_prop.get_critical_point()

    def get_molar_mass(self):
        """
        Minimal compatibility implementation required by the abstract base class.
        For the current use in the compressor model, return the refrigerant molar mass.
        """
        return self.refrigerant_prop.get_molar_mass()