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
import warnings
from typing import List
import numpy as np


from vclibpy.media import ThermodynamicState, TransportProperties, MedProp


logger = logging.getLogger(__name__)


class OilProp(MedProp):
    """Base class for all media property interfaces.

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
            lub_frac (float): The weight fraction of the lubricant between 0 and 1.
        """
        # Check if better internal names exist (e.g. air is modelled as air.ppf)
        super(OilProp, self).__init__(fluid_name)
        self.fluid_name = self._fluid_mapper.get(fluid_name, fluid_name)
        self.lub_name = self._oil_mapper.get(lub_name, lub_name)
        #self.lub_frac = lub_frac
        self._two_phase_limits: dict = None

 
    def calc_state(self, mode: str, var1: float, var2: float, lub_frac: float):
        """Calculate the thermodynamic state based on the specified mode and state variables.

        This function calculates the thermodynamic state based on the chosen mode and provided state variables.
        The input state variables need to be in SI units.

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
        available_options = ['PD', 'PH', 'PQ', 'PS', 'PT',
                             'PU', 'TD', 'TH', 'TQ', 'TS',
                             'TU', 'DH', 'DS', 'DU', ]
        assert mode in available_options, f'Given mode {mode} is not in available options'


    def calc_transport_properties(self, state: ThermodynamicState, lub_frac: float):
        """Calculate the transport properties for the given state.

        Args:
            state (ThermodynamicState): The current thermodynamic state.
            lub_frac (float): Mass fraction of lubricant in the mixture.

        Returns:
            TransportProperties: An instance of TransportProperties.
        """
        pass