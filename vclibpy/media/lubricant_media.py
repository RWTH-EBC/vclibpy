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
    def __init__(self, fluid_name: str, lub_name: str, lub_frac: float):
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
        self.lub_frac = lub_frac
        self._two_phase_limits: dict = None

    def calc_state(self, mode: str, var1: float, var2: float):
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

        Returns:
            ThermodynamicState: A ThermodynamicState instance with state variables.

        Raises:
            AssertionError: If the given mode is not within the available options.
        """
        available_options = ['PD', 'PH', 'PQ', 'PS', 'PT',
                             'PU', 'TD', 'TH', 'TQ', 'TS',
                             'TU', 'DH', 'DS', 'DU', ]
        assert mode in available_options, f'Given mode {mode} is not in available options'

    def terminate(self):
        """
        Terminate the class.
        Default behaviour does nothing.
        """
        pass

    @abc.abstractmethod
    def calc_transport_properties(self, state: ThermodynamicState):
        """Calculate the transport properties for the given state.

        Args:
            state (ThermodynamicState): The current thermodynamic state.

        Returns:
            TransportProperties: An instance of TransportProperties.
        """
        pass

    @abc.abstractmethod
    def get_critical_point(self):
        """Retrieve critical point information for the fluid.

        Returns:
            Tuple[float, float, float]: A tuple containing critical point information
            (Temperature Tc [K], Pressure pc [Pa], Density dc [kg/m^3]).
        """
        pass

    @abc.abstractmethod
    def get_molar_mass(self):
        """Retrieve the molar mass of the current fluid.

        Returns:
            float: The molar mass M of the current fluid in kg/mol.
        """
        pass

    def get_two_phase_limits(self, quantity: str, p_min: int = 100000, p_step: int = 5000):
        """
        Retrieve the two-phase limits for plotting a specified quantity.

        This method returns the two-phase limits for a specified quantity (T, h, s, or p) in an array used for
        plotting purposes. It calculates the limits within the pressure range from p_min and quality (q) 0 to the
        critical pressure (pc), and then from the critical pressure to the pressure p_min and quality 1.

        Args:
            quantity (str): The specified quantity (T, h, s, or p).
            p_min (int, optional): The minimum pressure value to start iteration. Default is 100000 Pa.
            p_step (int, optional): The step size for pressure variation. Default is 5000 Pa.

        Returns:
            numpy.ndarray: An array containing the two-phase limits for the specified quantity.

        Raises:
            ValueError: If the given quantity is not supported (T, h, s, or p).
        """
        if self._two_phase_limits is not None:
            # Check existing two-phase limits
            p_min_old = self._two_phase_limits['p'][0]
            p_step_old = self._two_phase_limits['p'][1] - p_min_old
            if not np.isclose(p_min_old, p_min, 0, 10) or not np.isclose(p_step_old, p_step, 0, 10):
                warnings.warn(f"Overwriting previously calculated two-phase limits with "
                              f"p_min={p_min_old} and p_step={p_step_old}. This might take a few seconds.\n"
                              f"The quantity might not match with the previously calculated quantities.")
                self._two_phase_limits = None

        if self._two_phase_limits is None:
            # Calculate new two-phase limits for plotting
            _two_phase_limits = get_two_phase_limits(self, p_step=p_step, p_min=p_min)
            self._two_phase_limits = {
                "T": np.array([state.T for state in _two_phase_limits]),
                "h": np.array([state.h for state in _two_phase_limits]),
                "s": np.array([state.s for state in _two_phase_limits]),
                "p": np.array([state.p for state in _two_phase_limits]),
            }

        if quantity not in self._two_phase_limits:
            raise ValueError("The given quantity is not supported. T, h, s, or p are supported.")
        return self._two_phase_limits[quantity]

    def calc_mean_transport_properties(self, state_in, state_out):
        """
        Calculate the average transport properties for the given states.

        Args:
            state_in (ThermodynamicState): First state
            state_out (ThermodynamicState): Second state

        Returns:
            TransportProperties: Average transport properties

        Notes:
            The TransportProperties does not contain a state, as an average
            state is not possible to calculate.
        """
        tr_pr_in = self.calc_transport_properties(state_in)
        tr_pr_out = self.calc_transport_properties(state_out)

        return TransportProperties(
            lam=0.5 * (tr_pr_in.lam + tr_pr_out.lam),
            dyn_vis=0.5 * (tr_pr_in.dyn_vis + tr_pr_out.dyn_vis),
            kin_vis=0.5 * (tr_pr_in.kin_vis + tr_pr_out.kin_vis),
            pr=0.5 * (tr_pr_in.Pr + tr_pr_out.Pr),
            cp=0.5 * (tr_pr_in.cp + tr_pr_out.cp),
            cv=0.5 * (tr_pr_in.cv + tr_pr_out.cv),
            state=None)


def get_two_phase_limits(med_prop: MedProp, p_step: int = 1000, p_min: int = int(1e3)) -> List[ThermodynamicState]:
    """
    Return the states representing the boundaries of the two-phase section for the given fluid.

    This function is primarily used for visualizing the two-phase section and validating the accuracy of calculations.

    Args:
        med_prop (MedProp): An instance of a valid MedProp-Class.
        p_step (int): The step size for pressure variation in Pa. Default is 1000 Pa.
        p_min (int): The minimum pressure in Pa from where to start calculation. Default is 1000 Pa.

    Returns:
        List[ThermodynamicState]: A list of ThermodynamicState instances representing the two-phase limits.

    Notes:
        The two-phase limits are computed by iterating over a range of pressures from the minimum pressure up to the
        critical point pressure (exclusive) with a specified step size. States at quality 0 (saturated liquid)
        and quality 1 (saturated vapor) are appended to form the two-phase boundary. The list is reversed to
        maintain the correct order for visualization purposes.
    """
    _, _p_max, _ = med_prop.get_critical_point()
    q0, q1 = [], []
    for _p in range(p_min, int(_p_max), p_step):
        try:
            q0.append(med_prop.calc_state("PQ", _p, 0))
            q1.append(med_prop.calc_state("PQ", _p, 1))
        except ValueError as err:
            logger.info("Could not calculate two-phase limits for p=%s: %s",
                        _p, err)
    # Reverse list for correct order
    return q0 + q1[::-1]
