# -*- coding: utf-8 -*-
"""
Created on 25.07.2025

@author: Anna Halle, Roman Provolotskyi, Bogdan Bykov

"""
import abc
import logging
import matlab.engine
from vclibpy.media import ThermodynamicState, TransportProperties, OilProp, RefProp
eng = matlab.engine.start_matlab()

logger = logging.getLogger(__name__)


class OilMixProp(OilProp):
    """
    Class to connect to OilMixProp package.
    """

    __author__ = "Anna Halle"

    _fluid_mapper = {}

    def __init__(self, fluid_name, lub_name, OilMixProp_path):

        super().__init__(fluid_name=fluid_name, lub_name=lub_name)
        self.OilMixProp_path = OilMixProp_path


    def terminate(self):
        if self._delete_dll_path is not None:
            self._delete_dll()

    def get_critical_point(self):
        raise NotImplementedError("get_critical_point must be implemented.")

    def get_molar_mass(self):
        raise NotImplementedError("get_molar_mass must be implemented.")

    @staticmethod
    def get_field_safe(mat_struct, field):
        """
        Safely extracts a field from a MATLAB struct returned as a Python dictionary.
        Converts matlab.double to float or list.
        """
        if not isinstance(mat_struct, dict):
            print("[WARN] Result is not a dictionary.")
            return None

        if field not in mat_struct:
            return None

        val = mat_struct[field]

        # Convert matlab.double to float or list
        if isinstance(val, matlab.double):
            flat = list(val._data)
            if len(flat) == 1:
                return float(flat[0])
            return [float(x) for x in flat]

        # Return other types directly
        return val

    @staticmethod
    def translate_user_key_to_state(user_key: str) -> str:
        """
        Converts user input like 'PH' into internal format like 'P-H'.
        Raises ValueError if the combination is not supported.
        """

        # Unterstützte Zustandskombinationen im Programm
        supported_states = {
            "TP": "T-P",  # Temperature, Pressure
            "TD": "T-D",  # Temperature, Density
            "TS": "T-S",  # Temperature, Entropy
            "TH": "T-H",  # Temperature, Enthalpy
            "TQ": "T-Q",  # Temperature, Vapor quality
            "PD": "P-D",  # Pressure, Density
            "PS": "P-S",  # Pressure, Entropy
            "PH": "P-H",  # Pressure, Enthalpy
            "PQ": "P-Q",  # Pressure, Vapor quality
        }

        # Benutzereingabe in Großbuchstaben umwandeln
        key = user_key.upper()

        # Überprüfen, ob die Kombination unterstützt wird
        if key not in supported_states:
            raise ValueError(f"Unsupported combination '{key}'.")

        # Entsprechenden internen Zustandsschlüssel zurückgeben
        return supported_states[key]

    def calc_state(self, mode: str, var1: float, var2: float, lub_frac: float):
        """
    Calculates thermodynamic state and transport properties for a given input condition.

    Parameters:
    ----------
    mode : str
        Input variable mode
        Supported two-parameter combinations for state calculation:

        "TP": Temperature [K], Pressure [Pa]
        "TD": Temperature [K], Density [kg/m³]
        "TS": Temperature [K], Entropy [J/kg/K]
        "TH": Temperature [K], Enthalpy [J/kg]
        "TQ": Temperature [K], Vapor Quality [-]
        "PD": Pressure [Pa], Density [kg/m³]
        "PS": Pressure [Pa], Entropy [J/kg/K]
        "PH": Pressure [Pa], Enthalpy [J/kg]
        "PQ": Pressure [Pa], Vapor Quality [-]

    var1 : float
        Value of the first input variable (must be in SI units).
    var2 : float
        Value of the second input variable (must be in SI units).
    lub_frac : float
        Lubricant mass fraction (between 0 and 1).

    Returns:
    -------
    state : ThermodynamicState
        Contains the thermodynamic properties:
            - p : Pressure [Pa]
            - T : Temperature [K]
            - u : Internal Energy [J/kg]
            - h : Enthalpy [J/kg]
            - s : Entropy [J/kg/K]
            - d : Density [kg/m³]
            - q : Vapor Quality [-]
    props : TransportProperties
        Contains the transport properties:
            - lam : Thermal Conductivity [W/m/K]
            - dyn_vis : Dynamic Viscosity [Pa·s]
            - kin_vis : Kinematic Viscosity [m²/s]
            - pr : Prandtl Number [-]
            - cp : Specific Heat Capacity at Constant Pressure [J/kg/K]
            - cv : Specific Heat Capacity at Constant Volume [J/kg/K]
            - beta : Isobaric Thermal Expansion Coefficient [1/K]
            - sur_ten : Surface Tension [N/m] (not available)
            - ace_fac : Accommodation Factor [-] (not available)
    """

        ref = self.fluid_name
        lub = self.lub_name

        eng = matlab.engine.start_matlab()
        logger.info("MATLAB engine started")

        eng.cd(self.OilMixProp_path, nargout=0)
        eng.addpath(self.OilMixProp_path, nargout=0)

        refrigerants = [ref, lub]
        mass_fracs = [1.0 - lub_frac, lub_frac]

        if not 1 <= len(refrigerants) <= 3:
            raise ValueError('Only 1 to 3 components are allowed.')

        if len(mass_fracs) not in (1, 2):
            raise ValueError('mass_fracs must contain 1 or 2 values.')

        if len(mass_fracs) > len(refrigerants):
            raise ValueError('Too many mass fractions for the number of components.')

        if len(refrigerants) == 3 and len(mass_fracs) == 2:
            mass_fracs.append(1.0 - sum(mass_fracs))

        if any(mf < 0 for mf in mass_fracs) or sum(mass_fracs) > 1 + 1e-6:
            raise ValueError('Invalid mass fractions.')

        # EOS Auswahl: Einer Davon Auswählen: 'PR', 'SRK', 'PTV', 'YFR'
        eos_choice = 'YFR'

        # T P Guess(einfach als 0 lassen, falls keine gute Einschätzung nicht bekannt)
        T_K_guess = 0.0
        p_kPa_guess = 0.0
        state_key = self.translate_user_key_to_state(mode)
        mf_column = matlab.double([[mf] for mf in mass_fracs])  # Spaltenvektor


        result = eng.FluidCalc_1(
            mf_column,
            eos_choice,
            refrigerants,
            state_key,
            float(var1),
            float(var2),
            float(T_K_guess),
            float(p_kPa_guess),
            nargout=1
        )


        eng.quit()
        q = self.get_field_safe(result, 'FracV_mass')
        p = self.get_field_safe(result, 'p_Pa')
        T = self.get_field_safe(result, 'T_K')
        h = self.get_field_safe(result, 'hh_Jkg')

        s = self.get_field_safe(result, 'ss_JkgK')
        d = self.get_field_safe(result, 'rho_kgm3')
        v = self.get_field_safe(result, 'v_spez')
        if q is not None:
            h = h[0] * (1-q) + h[1]*q
            s = s[0] * (1 - q) + s[1] * q
            d = d[0] * (1 - q) + d[1] * q
            v = v[0] * (1 - q) + v[1] * q
        u = (h - p * v) if h is not None and p is not None and v is not None else None

        state = ThermodynamicState(
            p=p,
            T=T,
            u=u,
            h=h,
            s=s,
            d=d,
            q=q
        )
        cp = self.get_field_safe(result, 'cp_JkgK')
        cv = self.get_field_safe(result, 'cv_JkgK')
        dyn_vis = self.get_field_safe(result, 'vis_Pas')
        lam = self.get_field_safe(result, 'lambda_WmK')
        drho_dT = self.get_field_safe(result, 'drhokg_dT')
        if q is not None:
            cp = cp[0] * (1 - q) + cp[1] * q
            cp = cv[0] * (1 - q) + cv[1] * q
            dyn_vis = dyn_vis[0] * (1-q) + dyn_vis[1]*q
            lam = lam[0] * (1 - q) + lam[1] * q
            drho_dT = drho_dT[0] * (1 - q) + drho_dT[1] * q


        kin_vis = dyn_vis / d if dyn_vis is not None and d is not None else None
        pr = cp * dyn_vis / lam if cp is not None and dyn_vis is not None and lam is not None else None
        beta = - (1 / d) * drho_dT if d is not None and drho_dT is not None else None

        sur_ten = None
        ace_fac = None

        props = TransportProperties(
            lam=lam,
            dyn_vis=dyn_vis,
            kin_vis=kin_vis,
            pr=pr,
            cp=cp,
            cv=cv,
            beta=beta,
            sur_ten=sur_ten,
            ace_fac=ace_fac,
            state=state
        )

        return state, props
