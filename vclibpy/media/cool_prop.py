"""
Module with cool prop wrapper.
"""
import logging

import CoolProp.CoolProp as CoolPropInternal

from vclibpy.media.media import MedProp, Fluid
from vclibpy.media.states import ThermodynamicState, TransportProperties

logger = logging.getLogger(__name__)


class CoolProp(MedProp):
    """
    Class using the open-source CoolProp package
    to access the properties.

    Args:
        use_high_level_api (bool):
            True to use the high-level api, which is much slower,
            but you can use all modes in calc_state.
            Default is False.
    """

    _mode_map = {
        "PT": (CoolPropInternal.PT_INPUTS, True),
        "TQ": (CoolPropInternal.QT_INPUTS, False),
        "PS": (CoolPropInternal.PSmass_INPUTS, True),
        "PH": (CoolPropInternal.HmassP_INPUTS, False),
        "PQ": (CoolPropInternal.PQ_INPUTS, True)
    }

    _modes_mixtures = ["PQ", "TQ", "PT"]
    _modes_incomp = ["PT", "PH", "PD", "PS"]

    _fluid_mapper = {}

    # TODO: incompressible fluids (require INCOMP-Backend and can be mass- or volume-based)
    # TODO: mixtures only allow modes PQ, TQ, TP
    def __init__(self, fluid: Fluid, use_high_level_api: bool = False):
        super().__init__(fluid=fluid)

        self._backend = fluid.backend

        self._Ref = f"{self._backend}::" + "&".join([self._fluid_mapper.get(c, c)+"[{}]" for c in self.components]).format(*self._mol_frac)
        self._fluidnames = "&".join([self._fluid_mapper.get(c, c) for c in self.components])
        self._is_mixture = (len(self.components) > 1)

        # Set molar mass and trigger a possible fluid-name error if the fluid is not supported.
        self._state = CoolPropInternal.AbstractState(self._backend, self._fluidnames)
        if self._is_mixture:
            self._state.set_mole_fractions(self._mol_frac)
        self.use_high_level_api = use_high_level_api

    def calc_state(self, mode: str, var1: float, var2: float):
        super().calc_state(mode=mode, var1=var1, var2=var2)

        if self.use_high_level_api:
            _var1_str, _var2_str = mode[0], mode[1]
            # CoolProp returns Pa
            p = CoolPropInternal.PropsSI('P', _var1_str, var1, _var2_str, var2, self._Ref)
            # CoolProp returns kg/m^3
            d = CoolPropInternal.PropsSI('D', _var1_str, var1, _var2_str, var2, self._Ref)
            # CoolProp returns K
            T = CoolPropInternal.PropsSI('T', _var1_str, var1, _var2_str, var2, self._Ref)
            # CoolProp returns J/kg
            u = CoolPropInternal.PropsSI('U', _var1_str, var1, _var2_str, var2, self._Ref)
            # CoolProp returns J/kg
            h = CoolPropInternal.PropsSI('H', _var1_str, var1, _var2_str, var2, self._Ref)
            # CoolProp returns J/kg/K
            s = CoolPropInternal.PropsSI('S', _var1_str, var1, _var2_str, var2, self._Ref)
            # CoolProp returns mol/mol TODO: RefProp returns kg/kg (only different for mixtures)
            if self._backend == "INCOMP":
                q = 0
            else:
                q = CoolPropInternal.PropsSI('Q', _var1_str, var1, _var2_str, var2, self._Ref)
            # Return new state
            return ThermodynamicState(p=p, T=T, u=u, h=h, s=s, d=d, q=q)

        self._update_coolprop_heos(mode=mode, var1=var1, var2=var2)
        # Return new state
        return ThermodynamicState(
            p=self._state.p(),
            T=self._state.T(),
            u=self._state.umass(),
            h=self._state.hmass(),
            s=self._state.smass(),
            d=self._state.rhomass(),
            q=self._state.Q()
        )

    def calc_transport_properties(self, state: ThermodynamicState):
        if 0 <= state.q <= 1:
            mode = "PQ"
            var1, var2 = state.p, state.q
        else:
            # Get using p and T
            mode = "PT"
            var1, var2 = state.p, state.T

        if self.use_high_level_api:
            args = [mode[0], var1, mode[1], var2, self._Ref]
            # CoolProp returns -
            pr = CoolPropInternal.PropsSI('PRANDTL', *args)
            # CoolProp returns J/kg/K
            cp = CoolPropInternal.PropsSI('C', *args)
            # CoolProp returns J/kg/K
            cv = CoolPropInternal.PropsSI('CVMASS', *args)
            # CoolProp returns W/m/K
            lam = CoolPropInternal.PropsSI('CONDUCTIVITY', *args)
            # CoolProp returns Pa*s
            dyn_vis = CoolPropInternal.PropsSI('VISCOSITY', *args)
            # Internal calculation as kinematic vis is ration of dyn_vis to density
            # In m^2/s
            kin_vis = dyn_vis / state.d

            # Create transport properties instance
            return TransportProperties(lam=lam, dyn_vis=dyn_vis, kin_vis=kin_vis,
                                       pr=pr, cp=cp, cv=cv, state=state)
        # Low-level API
        self._update_coolprop_heos(mode=mode, var1=var1, var2=var2)
        # Create transport properties instance
        return TransportProperties(
            lam=self._state.conductivity(),
            dyn_vis=self._state.viscosity(),
            kin_vis=self._state.viscosity() / state.d,
            pr=self._state.Prandtl(),
            cp=self._state.cpmass(),
            cv=self._state.cvmass(),
            state=state
        )

    def _update_coolprop_heos(self, mode, var1, var2):
        if mode not in self._mode_map:
            raise KeyError(
                f"Given mode '{mode}' is currently not supported with the "
                f"faster low-level API to cool-prop. "
                f"Either use the high-level API or raise an issue. "
                f"Supported modes: {', '.join(self._mode_map.keys())}"
            )
        i_input, not_reverse_variables = self._mode_map[mode]
        if not_reverse_variables:
            self._state.update(i_input, var1, var2)
        else:
            self._state.update(i_input, var2, var1)

    def get_critical_point(self):
        Tc = CoolPropInternal.PropsSI("TCRIT", self._Ref)
        pc = CoolPropInternal.PropsSI("PCRIT", self._Ref)
        dc = CoolPropInternal.PropsSI("RHOCRIT", self._Ref)
        return Tc, pc, dc

    def get_molar_mass(self):
        try:
            return self._state.molar_mass()
        except ValueError as err:
            if "not implemented" in str(err):
                logger.error(str(err))
                raise NotImplementedError(str(err))
            else:
                raise err


if __name__ == '__main__':
    CoolProp("Propane")
