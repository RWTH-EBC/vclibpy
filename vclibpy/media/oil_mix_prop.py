# -*- coding: utf-8 -*-
"""
Created on 17.07.2025

@author: Anna Halle, Roman Provolotskyi, Bogdan Bykov


"""

import abc
import logging
from vclibpy.media import ThermodynamicState, TransportProperties, OilProp, RefProp


logger = logging.getLogger(__name__)


class OilMixProp(OilProp):
    """
    Class to connect to OilMixProp package.

    Args:
    :param string fluid_name:
        Fluid name for RefProp use
    :param list or None z:
        Fluid composition. Should only be used, when a self-design mixture shall be used. Further information
        see notes.
        When you want to use a self-design mixture to as follows:
        - Fluid-name needs to contain all component names within mixture: "R32.FLD|R125.FLD"
        - z needs to be a list with molar fractions: [0.697, 0.303]
        - Used example would be similar to R410A

    :param string dll_path:
        Specifier for the dll path used for RefProp,
        e.g. dll_path='C:\\path_to_dll\\RefProp64.dll'.
        If None, the `ref_prop_path` and function `get_dll_path` are
        used to determine the dll path
    :param boolean use_error_check:
        Specifier whether errors and warnings shall be checked when calling RefProp or not
    :param boolean use_warnings:
        Specifier whether warnings shall be used
    :param str ref_prop_path:
        Path to RefProp package. Default is the ENV variable `RPPREFIX`.
    :param bool copy_dll:
        If True (not the default), a copy of the dll is created to enable simultaneous use of
        multiple fluids in multiprocessing.
    :param str copy_dll_directory:
        If `copy_dll` is True, the DLL is copied to this directory.
        If None (default), the current working directory is used.

    Note:
        - You need to install package ctREFPROP Package
          https://github.com/usnistgov/REFPROP-wrappers/tree/master/wrappers/python
        - In case you use a self-defined mixture: Entropy reference state might deviate from GUI!!

    Functionality:
        - It does not work to have multiple instances simultaneously. When calculating values the last fluid name
            somehow will be used even though instance includes "new" name. Need to be fixed at some point.


    How to use RefProp-Wrapper:
    ---------------------------
    1.) Create RefProp instance: rp = RefProp("R32")
    2.) In case you want to calculate fluid properties (state variables) for a specific state: Use calc_state() function
            Multiple inputs can be given (but you need to now two in order to define the values). For further
            information see function header.
    3.) Further get-Functions implemented
        - get_gwp(): Global warming potential
        - get_odp(): Ozone depletion potential
        - get_safety(): Safety class
        - get_mass_fraction(): Mass fractions of pure substances in fluid
        - get_molar_mass(): Molar mass of fluid
        - get_mol_fraction(): Mol fractions of pure substances in fluid
        - get_comp_names(): Pure substances names in fluid
        - get_longname(): Long name of fluid within RefProp
        - get_critical_point(): Crit. Temperature and Pressure
        - get_version(): Version of wrapper and of RefProp dll



    """
    # General information
    #

    __author__ = "Anna Halle"

    _fluid_mapper = {}

    def __init__(self,
                 fluid_name):


        super().__init__(fluid_name=fluid_name)



    def terminate(self):
        if self._delete_dll_path is not None:
            self._delete_dll()



    def calc_state(self, mode: str, var1: float, var2: float, lub_frac: float):
        """ Calculate state. Depending on mode, different function will be chosen. Input state variables need to be in
        SI units!

        Notes:
        ------
        1.) PT does not work when state might be within the two-phase region!
        2.) Only functions for density are implemented. In case you know the specific volume instead use density
                functions with inverse value!
        3.) Quality can have values outside of 'physical' scope:
                q = -998: Subcooled liquid
                q = 998: Superheated vapor
                q = 999: Supercritical state

        Possible modes are currently:
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
            - "HS": Enthalpy, Entropy

        Parameters:
        -----------
        :param string mode:
            Defines which input state variables are given (see possible modes above)
        :param float var1:
            Value of state variable 1 (first one in name) - use SI units!
        :param float var2:
            Value of state variable 2 (second one in name) - use SI units!
        :param int kr:
            phase flag (kr=1: lower density, kr=2: higher density)
            relevant for "TH", "TS", "TU"

        Return:
        -------
        :return ThermodynamicState state:
            Thermodynamic state with state variables
        """

        state = ThermodynamicState(p=p, T=T, u=u, h=h, s=s, d=d, q=q)
        return state


    def calc_transport_properties(self, state: ThermodynamicState):
        """ Calculate transport properties of RefProp fluid at given state

        Parameters:
        -----------
        :param ThermodynamicState state:
            Current thermodynamic state
        Return:
        -------
        :return TransportProperties props:
            Instance of TransportProperties
        """
        # Get properties

        props = TransportProperties(lam=None,
                                    dyn_vis=None,
                                    kin_vis=None,
                                    pr=None,
                                    cp=None,
                                    cv=None,
                                    beta=None,
                                    sur_ten=None,
                                    ace_fac=None,
                                    state=state)

        #Return props
        return props
