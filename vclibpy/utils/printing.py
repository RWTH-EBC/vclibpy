# -*- coding: utf-8 -*-
"""
Created on 24.04.2023

@author: Christoph Hoeges

Last Update: 24.04.2023
"""
import pandas as pd


def print_states(**kwargs):
    """
    Transforms given states to DataFrame and prints table layout.

    Returns:
    :return pandas.DataFrame df_states:
        DataFrame with states
    """
    # Remove keys in given data that are not label as "state"
    for key in kwargs.keys():
        if not key.startswith("state"):
            print(
                f"Given value {key} is removed from data transformation! "
                f"Value must be declared as 'state'"
            )
            kwargs.pop(key, None)

    # Extract data from ThermodynamicStates
    data_states = {key: {"state": key[5:], "T": data.T, "p": data.p, "h": data.h, "s": data.s, "d": data.d}
                   for key, data in kwargs.items()}
    df_states = pd.DataFrame.from_dict(data=data_states)

    # Similar to previous results - now with rounded data
    data_states_rounded = {key: {"State": key[5:],
                                 "T in °C": round(data.T - 273.15, 2),
                                 "p in bar": round(data.p * 1e-5, 3),
                                 "h in kJ/kg": round(data.h * 1e-3, 3),
                                 "s in kJ/kg": round(data.s * 1e-3, 3),
                                 "d in kg/m3": round(data.d, 3)}
                           for key, data in kwargs.items()}
    df_states_rounded = pd.DataFrame.from_dict(data=data_states_rounded)
    print(df_states_rounded.T)

    return df_states
