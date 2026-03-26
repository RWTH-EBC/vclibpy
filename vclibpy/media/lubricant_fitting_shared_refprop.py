"""
LubricantFitting — thermophysical property model for oil-refrigerant mixtures.

Provides standalone methods for:
    - Pure oil properties: cp_oil, lambda_oil, h_oil, s_oil, rho_oil_pure
    - Refrigerant solubility: solve_w_KM(T, p)
    - Mixture properties: cp_mix, h_mix, rho_mix

All process-level calculations (throttling, mass balances, heat transfer,
outlet separation) belong in the compressor model, not here.

Reference state for oil enthalpy/entropy:
    T_ref = 273.15 K,  h_oil(T_ref) = 0,  s_oil(T_ref) = 0
"""
import logging
import math
from typing import Optional

import numpy as np
from scipy.optimize import brentq as _brentq

from vclibpy.media import ThermodynamicState, TransportProperties, MedProp, OilProp, RefProp


logger = logging.getLogger(__name__)

# Reference temperature for oil enthalpy and entropy integration
T_REF = 273.15  # K (0 °C)


class LubricantFitting(OilProp):
    """
    Lubricant-refrigerant mixture property model.

    Provides all thermophysical properties needed for the oil-path
    extension of the Molinaroli (2017) rolling piston compressor model.
    """

    _fluid_mapper = {}
    _oil_mapper = {}

    def __init__(self, fluid_name: str, lub_name: str, shared_refprop=None):
        """
        Args:
            fluid_name: Name of the refrigerant (currently only "propane").
            lub_name: Name of the lubricant ("LPG 68" or "LPG 100").
            shared_refprop: Optional RefProp instance to inject immediately.
        """
        super().__init__(fluid_name, lub_name)

        self.fluid_name = self._fluid_mapper.get(fluid_name, fluid_name)
        self.lub_name = self._oil_mapper.get(lub_name, lub_name)
        self._two_phase_limits: dict = None

        # RefProp instance — injected from outside to avoid DLL copy issues
        self.refrigerant_prop: Optional[RefProp] = shared_refprop

        available_refrigerants = ["propane"]
        if fluid_name not in available_refrigerants:
            raise ValueError(
                f"Refrigerant '{fluid_name}' not supported. "
                f"Available: {available_refrigerants}"
            )

        if fluid_name == "propane":
            available_lubricants = ["LPG 68", "LPG 100"]
            if lub_name not in available_lubricants:
                raise ValueError(
                    f"Lubricant '{lub_name}' not supported. "
                    f"Available: {available_lubricants}"
                )
            self._init_propane_parameters(lub_name)

    # -----------------------------------------------------------------
    # Parameter initialization
    # -----------------------------------------------------------------
    def _init_propane_parameters(self, lub_name: str):
        """Load all fitting coefficients for a given propane-oil pair."""

        self.T_crit = 369.89  # K
        self.M_rfg = 43.01
        self.T_ref = T_REF

        if lub_name == "LPG 68":
            self.M_oil = 400.0
            self._init_lpg68()
        elif lub_name == "LPG 100":
            self.M_oil = 480.0
            self._init_lpg100()

    def _init_lpg68(self):
        """Coefficients for propane / LPG 68."""

        # --- Saturation pressure ---
        self.sat_p_a, self.sat_p_b, self.sat_p_c = 7.11, -2.34, -4.96
        self.sat_p_d, self.sat_p_e, self.sat_p_f = -4.02, -0.436, 5.05

        # --- Dynamic viscosity (log-log fit) ---
        self.dyn_visc_a, self.dyn_visc_b, self.dyn_visc_c = 9.1741e0, -3.9327e0, 1.4495e-1
        self.dyn_visc_d, self.dyn_visc_e, self.dyn_visc_f = -3.6884e-1, -7.2671e-1, 7.5201e-3
        self.dyn_visc_g, self.dyn_visc_h, self.dyn_visc_i = 1.3094e0, 1.5980e0, -7.9215e-1

        # --- Mixture density ---
        self.rho_a, self.rho_b, self.rho_c = 1.1736e3, -4.3812e-1, -6.2498e-4
        self.rho_d, self.rho_e, self.rho_f = 1.7379e2, -5.8711e0, 9.1282e-3
        self.rho_g, self.rho_h, self.rho_i = 3.0804e1, 2.6090e0, -5.9097e-3

        # --- Kinematic viscosity ---
        self.kin_visc_a, self.kin_visc_b, self.kin_visc_c = 1.3893e1, -7.8106e0, 9.4155e-1
        self.kin_visc_d, self.kin_visc_e, self.kin_visc_f = -5.2681e-2, -1.6857e0, 3.5412e-1
        self.kin_visc_g, self.kin_visc_h, self.kin_visc_i = -4.0291e-2, -1.2629e-1, 2.0426e-1

        # --- Pure oil cp: cp_oil [kJ/(kg*K)] = _cp_a + _cp_b * T [K] ---
        #     Fitted from manufacturer data (Fuchs Reniso LPG 68)
        self._cp_a = 0.764277
        self._cp_b = 0.00335165

        # --- Pure oil thermal conductivity: lam_oil [W/(m*K)] = _lam_a + _lam_b * T ---
        self._lam_a = 0.187875
        self._lam_b = -0.00012560

        # --- Tabulated cp for interpolation (fallback / validation) ---
        self.T_cp = [-20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
        self.cp_tab = [1.63, 1.66, 1.68, 1.71, 1.73, 1.77, 1.80,
                       1.84, 1.88, 1.91, 1.95, 1.99, 2.03]

    def _init_lpg100(self):
        """Coefficients for propane / LPG 100."""

        # --- Saturation pressure ---
        self.sat_p_a, self.sat_p_b, self.sat_p_c = 6.87, -2.64, -4.80
        self.sat_p_d, self.sat_p_e, self.sat_p_f = -18.9, 34.4, -14.5

        # --- Dynamic viscosity ---
        self.dyn_visc_a, self.dyn_visc_b, self.dyn_visc_c = 1.5002e1, -8.8595e0, 1.1898e0
        self.dyn_visc_d, self.dyn_visc_e, self.dyn_visc_f = 1.0678e0, -1.3895e0, 9.5803e-3
        self.dyn_visc_g, self.dyn_visc_h, self.dyn_visc_i = -1.3753e0, -1.7613e0, 1.2581e0

        # --- Mixture density ---
        self.rho_a, self.rho_b, self.rho_c = 1.1829e3, -5.23432e-1, -4.3074e-4
        self.rho_d, self.rho_e, self.rho_f = 1.3561e2, -5.2478e0, 6.5347e-3
        self.rho_g, self.rho_h, self.rho_i = 1.8586e1, 6.8091e0, 5.5917e-3

        # --- Kinematic viscosity ---
        self.kin_visc_a, self.kin_visc_b, self.kin_visc_c = 1.9289e1, -1.2371e1, 1.9087e0
        self.kin_visc_d, self.kin_visc_e, self.kin_visc_f = 1.0470e0, -2.3514e0, 4.1481e-1
        self.kin_visc_g, self.kin_visc_h, self.kin_visc_i = -1.7803e0, -2.3293e0, 1.6092e0

        # --- Pure oil cp: cp_oil [kJ/(kg*K)] = _cp_a + _cp_b * T [K] ---
        self._cp_a = 0.752456
        self._cp_b = 0.00340659

        # --- Pure oil thermal conductivity: lam_oil [W/(m*K)] = _lam_a + _lam_b * T ---
        self._lam_a = 0.188353
        self._lam_b = -0.00012423

        # --- Tabulated cp for interpolation (fallback / validation) ---
        self.T_cp = [-20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100]
        self.cp_tab = [1.64, 1.66, 1.68, 1.71, 1.74,
                       1.77, 1.81, 1.84, 1.88, 1.92,
                       1.96, 2.00, 2.04]

    # -----------------------------------------------------------------
    # RefProp injection
    # -----------------------------------------------------------------
    def set_refrigerant_prop(self, refrigerant_prop: RefProp):
        """Inject an existing RefProp instance (avoids DLL copy issues)."""
        self.refrigerant_prop = refrigerant_prop

    def _require_refrigerant_prop(self) -> RefProp:
        if self.refrigerant_prop is None:
            raise RuntimeError(
                "LubricantFitting.refrigerant_prop is not set. "
                "Call set_refrigerant_prop(med_prop) first."
            )
        return self.refrigerant_prop

    # =================================================================
    #  PURE OIL PROPERTIES
    # =================================================================

    def calc_cp_oil(self, T: float) -> float:
        """
        Specific heat capacity of pure oil.

        Args:
            T: Temperature [K]

        Returns:
            cp_oil [J/(kg*K)]
        """
        cp_kJ = self._cp_a + self._cp_b * T  # kJ/(kg*K)
        return cp_kJ * 1e3  # J/(kg*K)

    def calc_lambda_oil(self, T: float) -> float:
        """
        Thermal conductivity of pure oil.

        Args:
            T: Temperature [K]

        Returns:
            lambda_oil [W/(m*K)]
        """
        return self._lam_a + self._lam_b * T

    def calc_h_oil(self, T: float) -> float:
        """
        Specific enthalpy of pure oil relative to T_ref = 273.15 K.

        h_oil(T) = integral from T_ref to T of cp_oil(T') dT'
                 = a*(T - T_ref) + b/2*(T^2 - T_ref^2)

        where cp_oil = a + b*T in [kJ/(kg*K)]

        Args:
            T: Temperature [K]

        Returns:
            h_oil [J/kg]
        """
        dT = T - self.T_ref
        h_kJ = self._cp_a * dT + self._cp_b / 2.0 * (T ** 2 - self.T_ref ** 2)
        return h_kJ * 1e3  # J/kg

    def calc_s_oil(self, T: float) -> float:
        """
        Specific entropy of pure oil relative to T_ref = 273.15 K.

        s_oil(T) = integral from T_ref to T of cp_oil(T')/T' dT'
                 = a * ln(T/T_ref) + b * (T - T_ref)

        where cp_oil = a + b*T in [kJ/(kg*K)]

        Args:
            T: Temperature [K]

        Returns:
            s_oil [J/(kg*K)]
        """
        if T <= 0:
            return 0.0
        s_kJ = self._cp_a * math.log(T / self.T_ref) + self._cp_b * (T - self.T_ref)
        return s_kJ * 1e3  # J/(kg*K)

    def calc_rho_oil_pure(self, T: float) -> float:
        """
        Density of pure oil (w_KM = 0).

        Uses the mixture density correlation with w = 0.

        Args:
            T: Temperature [K]

        Returns:
            rho_oil [kg/m^3]
        """
        return self.rho_a + self.rho_b * T + self.rho_c * T ** 2

    # =================================================================
    #  REFRIGERANT SOLUBILITY
    # =================================================================

    def _calc_mixture_pressure(self, w_KM: float, T: float) -> float:
        """
        Mixture pressure [Pa] as function of refrigerant mass fraction and temperature.

        Args:
            w_KM: Refrigerant mass fraction in liquid phase [-]
            T: Temperature [K]

        Returns:
            Pressure [Pa]
        """
        refprop = self._require_refrigerant_prop()

        Tr = T / self.T_crit
        x = w_KM / (w_KM + (1.0 - w_KM) * (self.M_rfg / self.M_oil))

        psat = refprop.calc_state("TQ", T, 1).p
        if not np.isfinite(psat):
            return float("nan")

        term = (self.sat_p_a + self.sat_p_b * Tr + self.sat_p_c * Tr ** 2
                + self.sat_p_d * x + self.sat_p_e * x * Tr
                + self.sat_p_f * x * Tr ** 2)

        p = x * psat + x * (1.0 - x) * term * psat
        return p

    def solve_w_KM(self, T: float, p: float) -> Optional[float]:
        """
        Solve for the refrigerant mass fraction w_KM in the liquid phase
        at given temperature and pressure.

        Args:
            T: Temperature [K]
            p: Pressure [Pa]

        Returns:
            w_KM [-] or None if no solution found.
        """
        def objective(w):
            return self._calc_mixture_pressure(w, T) - p

        a, b = 0.001, 0.999
        try:
            fa, fb = objective(a), objective(b)
            if not (np.isfinite(fa) and np.isfinite(fb)):
                return None
            if fa * fb > 0:
                return None
            return _brentq(objective, a, b, maxiter=400)
        except Exception:
            return None

    # =================================================================
    #  MIXTURE PROPERTIES (oil + dissolved refrigerant)
    # =================================================================

    def calc_rho_mix(self, T: float, w_KM: float) -> float:
        """
        Density of the oil-refrigerant liquid mixture.

        Args:
            T: Temperature [K]
            w_KM: Refrigerant mass fraction [-]

        Returns:
            rho_mix [kg/m^3]
        """
        return (
            (self.rho_a + self.rho_b * T + self.rho_c * T ** 2)
            + w_KM * (self.rho_d + self.rho_e * T + self.rho_f * T ** 2)
            + w_KM ** 2 * (self.rho_g + self.rho_h * T + self.rho_i * T ** 2)
        )

    def calc_cp_mix(self, T: float, p: float, w_KM: float) -> float:
        """
        Specific heat capacity of the liquid mixture (oil + dissolved refrigerant).

        cp_mix = w_KM * cp_KM_liq + (1 - w_KM) * cp_oil

        Args:
            T: Temperature [K]
            p: Pressure [Pa]
            w_KM: Refrigerant mass fraction [-]

        Returns:
            cp_mix [J/(kg*K)]
        """
        refprop = self._require_refrigerant_prop()

        cp_oil = self.calc_cp_oil(T)

        # Liquid refrigerant cp at saturation
        try:
            state_liq = refprop.calc_state("PQ", p, 0)
            transport_liq = refprop.calc_transport_properties(state_liq)
            cp_KM_liq = transport_liq.cp
        except Exception:
            # Fallback: use saturated liquid cp at given T
            try:
                state_liq = refprop.calc_state("TQ", T, 0)
                transport_liq = refprop.calc_transport_properties(state_liq)
                cp_KM_liq = transport_liq.cp
            except Exception:
                cp_KM_liq = 2700.0  # typical propane liquid cp, J/(kg*K)

        return w_KM * cp_KM_liq + (1.0 - w_KM) * cp_oil

    def calc_h_mix(self, T: float, p: float, w_KM: float) -> float:
        """
        Specific enthalpy of the liquid mixture (oil + dissolved refrigerant).

        h_mix = w_KM * h_KM_liq(T, p) + (1 - w_KM) * h_oil(T)

        The reference states are:
            - Oil: h_oil(273.15 K) = 0  (internal reference)
            - Refrigerant: RefProp IIR reference (consistent across all RefProp calls)

        This is thermodynamically consistent because in all energy balances
        each substance is balanced individually — the absolute reference
        cancels out.

        Args:
            T: Temperature [K]
            p: Pressure [Pa]
            w_KM: Refrigerant mass fraction [-]

        Returns:
            h_mix [J/kg]
        """
        refprop = self._require_refrigerant_prop()

        h_oil = self.calc_h_oil(T)

        # Liquid refrigerant enthalpy
        try:
            state_liq = refprop.calc_state("PQ", p, 0)
            h_KM_liq = state_liq.h
        except Exception:
            try:
                state_liq = refprop.calc_state("TQ", T, 0)
                h_KM_liq = state_liq.h
            except Exception:
                h_KM_liq = 250e3  # rough fallback, J/kg

        return w_KM * h_KM_liq + (1.0 - w_KM) * h_oil

    # =================================================================
    #  EXISTING INTERFACE (calc_state, calc_transport_properties, etc.)
    # =================================================================

    def calc_state(self, mode: str, phase: str, var1: float, var2: float, lub_frac=0):
        """Calculate thermodynamic state (vapor phase only via RefProp)."""
        available_phases = ["vapor"]
        if phase not in available_phases:
            raise ValueError(f"Phase '{phase}' not supported. Available: {available_phases}")

        available_options = [
            "PD", "PH", "PQ", "PS", "PT",
            "PU", "TD", "TH", "TQ", "TS",
            "TU", "DH", "DS", "DU",
        ]
        if mode not in available_options:
            raise ValueError(f"Mode '{mode}' not supported. Available: {available_options}")

        if phase == "vapor":
            refprop = self._require_refrigerant_prop()
            return refprop.calc_state(mode=mode, var1=var1, var2=var2)

        raise NotImplementedError(f"Phase '{phase}' not implemented in calc_state.")

    def calc_transport_properties(self, state: ThermodynamicState, phase: str, lub_frac=0):
        """
        Calculate transport properties for the given state.

        For phase='liquid', also solves for w_KM and returns mixture properties
        including cp, rho, dyn_vis, kin_vis.
        """
        refprop = self._require_refrigerant_prop()

        if phase == "vapor":
            return refprop.calc_transport_properties(state=state)

        elif phase == "liquid":
            T = state.T
            logT = math.log10(T)

            # Solve for refrigerant mass fraction
            w = self.solve_w_KM(T, state.p)
            if w is None:
                return None

            # Mixture density
            rho = (
                (self.rho_a + self.rho_b * T + self.rho_c * T ** 2)
                + w * (self.rho_d + self.rho_e * T + self.rho_f * T ** 2)
                + w ** 2 * (self.rho_g + self.rho_h * T + self.rho_i * T ** 2)
            )

            # Dynamic viscosity [mPa*s]
            y = (
                (self.dyn_visc_a + self.dyn_visc_b * logT + self.dyn_visc_c * logT ** 2)
                + w * (self.dyn_visc_d + self.dyn_visc_e * logT + self.dyn_visc_f * logT ** 2)
                + w ** 2 * (self.dyn_visc_g + self.dyn_visc_h * logT + self.dyn_visc_i * logT ** 2)
            )
            dyn_vis = math.pow(10.0, math.pow(10.0, y)) - 0.7

            # Kinematic viscosity [mm^2/s]
            y = (
                (self.kin_visc_a + self.kin_visc_b * logT + self.kin_visc_c * logT ** 2)
                + w * (self.kin_visc_d + self.kin_visc_e * logT + self.kin_visc_f * logT ** 2)
                + w ** 2 * (self.kin_visc_g + self.kin_visc_h * logT + self.kin_visc_i * logT ** 2)
            )
            kin_vis = math.pow(10.0, math.pow(10.0, y)) - 0.7

            state.d = rho
            state.v = 1.0 / rho

            # Pure oil cp at this temperature
            cp_oil = self.calc_cp_oil(T)

            # Thermal conductivity of pure oil
            lam_oil = self.calc_lambda_oil(T)

            return TransportProperties(
                lam=lam_oil,
                dyn_vis=dyn_vis,
                kin_vis=kin_vis,
                pr=float("nan"),
                cp=cp_oil,
                cv=float("nan"),
                beta=float("nan"),
                sur_ten=float("nan"),
                ace_fac=float("nan"),
                state=state,
            )
        else:
            raise ValueError(f"Unsupported phase: {phase}")

    def get_critical_point(self):
        refprop = self._require_refrigerant_prop()
        return refprop.get_critical_point()

    def get_molar_mass(self):
        refprop = self._require_refrigerant_prop()
        return refprop.get_molar_mass()
