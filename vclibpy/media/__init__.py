from .states import ThermodynamicState, TransportProperties
from .media import get_two_phase_limits, MedProp
from .cool_prop import CoolProp
from .ref_prop import RefProp
from .lubricant_media import OilProp
from .oil_mix_prop import OilMixProp



__all__ = ['ThermodynamicState',
           'TransportProperties',
           'MedProp',
           'CoolProp',
           'RefProp',
           'OilProp',
           'OilMixProp']

USED_MED_PROP = (CoolProp, {})


def set_global_media_properties(med_prop_class: object, **kwargs):
    """
    Set the globally used MedProp class.

    Args:
        med_prop_class (object):
            Available MedProp children class.
        kwargs (dict):
            Additional settings for the MedProp class,
            e.g. {"use_high_level_api": True} for CoolProp.
    """
    global USED_MED_PROP
    USED_MED_PROP = (med_prop_class, kwargs)


def get_global_med_prop_and_kwargs():
    """
    Get the global MedProp class used
    for all calculations.
    Returns:
        MedProp: The class
    """
    global USED_MED_PROP
    return USED_MED_PROP[0], USED_MED_PROP[1]
