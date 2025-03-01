"""
Functions to generate HP Maps automatically
"""
import logging
import pathlib
import os
from typing import List, Union
import multiprocessing
import numpy as np
import pandas as pd
from vclibpy.datamodels import FlowsheetState, Inputs, RelativeCompressorSpeedControl, HeatExchangerInputs
from vclibpy.flowsheets import BaseCycle
from vclibpy.algorithms import Algorithm, Iteration
from vclibpy import utils, media

logger = logging.getLogger(__name__)


def calc_multiple_states(
        save_path: pathlib.Path,
        flowsheet: BaseCycle,
        inputs: List[Inputs],
        algorithm: Algorithm,
        use_multiprocessing: bool = False,
        raise_errors: bool = False,
        with_unit_and_description: bool = True
):
    """
    Function to calculate the flowsheet states for all given inputs.
    All results are stored as a .xlsx file in the given save-path

    Args:
        save_path (pathlib.Path): Location where to save the results as xlsx.
        flowsheet (BaseCycle): A valid flowsheet
        inputs (List[Inputs]): A list with all inputs to simulate
        algorithm (Algorithm): A supported algorithm to calculate a steady state.
        use_multiprocessing (bool): True to use all cores, default no multiprocessing
    """
    rel_infos = []
    fs_states = []
    i = 0
    global_med_prop_data = media.get_global_med_prop_and_kwargs()
    if use_multiprocessing:
        mp_inputs = [[algorithm, flowsheet, inputs_, raise_errors, global_med_prop_data] for inputs_ in inputs]
        pool = multiprocessing.Pool(processes=min(multiprocessing.cpu_count(), len(inputs)))
        for fs_state in pool.imap(_calc_single_state, mp_inputs):
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(inputs)} points")
    else:
        for inputs_ in inputs:
            fs_state = _calc_single_state([algorithm, flowsheet, inputs_, raise_errors, global_med_prop_data])
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(inputs)} points")

    for fs_state, single_inputs in zip(fs_states, inputs):
        hp_state_dic = {
            **fs_state.convert_to_str_value_format(with_unit_and_description)
        }
        rel_infos.append(hp_state_dic)
    df = pd.DataFrame(rel_infos)
    df.index.name = "State Number"
    if os.path.isdir(save_path):
        save_path = save_path.joinpath(f"{flowsheet}_{flowsheet.fluid}.xlsx")
    df.to_excel(save_path, sheet_name="HP_states", float_format="%.5f")


def full_factorial_map_generation(
        flowsheet: BaseCycle,
        T_eva_in: Union[list, np.ndarray, float],
        T_con: Union[list, np.ndarray, float],
        n: Union[list, np.ndarray, float],
        m_flow_eva: Union[list, np.ndarray, float],
        save_path: Union[pathlib.Path, str],
        m_flow_con: Union[list, np.ndarray, float] = None,
        dT_con: Union[list, np.ndarray, float] = None,
        algorithm: Algorithm = None,
        dT_eva_superheating: Union[list, np.ndarray, float] = 5,
        dT_con_subcooling: Union[list, np.ndarray, float] = 0,
        use_condenser_inlet: bool = True,
        use_multiprocessing: bool = False,
        save_plots: bool = False,
        raise_errors: bool = False,
        save_sdf: bool = True
) -> (pathlib.Path, pathlib.Path):
    """
    Run a full-factorial simulation to create performance maps
    used in other simulation tools like Modelica or to analyze
    the off-design of the flowsheet.
    The results are stored and returned as .sdf and .csv files.
    Currently, only varying T_eva_in, T_con_in (or T_con_out), and n is implemented.
    However, changing this to more dimensions or other variables
    is not much work. In this case, please raise an issue.

    Args:
        flowsheet (BaseCycle): The flowsheet to use
        T_eva_in (list):
            Array with inputs for T_eva_in
        T_con (list):
            Array with inputs for T_con_in or T_con_out, see `use_condenser_inlet`
        n (list):
            Array with inputs for n
        m_flow_eva (float):
            Evaporator mass flow rate
        save_path (Path):
            Where to save all results.
        m_flow_con (float):
            Condenser mass flow rate, required if dT_con is None. Default is None.
        dT_con (float):
            Condenser temperature spread, required if m_flow_con is None. Default is None.
        algorithm (Algorithm):
            A supported algorithm to calculate a steady state.
            If None, Iteration algorithm is used with default settings.
        dT_eva_superheating (float):
            Evaporator superheating
        dT_con_subcooling (float):
            Condenser subcooling
        use_condenser_inlet (bool):
            True to consider T_con as inlet, false for outlet.
        use_multiprocessing:
            True to use multiprocessing. May speed up the calculation. Default is False
        save_plots (bool):
            True to save plots of each steady state point. Default is False
        raise_errors (bool):
            True to raise errors if they occur.
        save_sdf (bool):
            = False to not save sdf files. Default is True

    Returns:
        tuple (pathlib.Path, pathlib.Path):
            Path to the created .sdf file and to the .csv file
    """
    # Convert single values to arrays
    def ensure_array(x):
        if isinstance(x, (list, np.ndarray)):
            return np.array(x)
        return np.array([x])

    T_eva_in = ensure_array(T_eva_in)
    T_con = ensure_array(T_con)
    n = ensure_array(n)
    m_flow_eva = ensure_array(m_flow_eva)
    dT_eva_superheating = ensure_array(dT_eva_superheating)
    dT_con_subcooling = ensure_array(dT_con_subcooling)
    if m_flow_con is not None and dT_con is not None:
        raise ValueError("Can only run m_flow_con or dT_con, not both")
    if m_flow_con is None and dT_con is None:
        raise ValueError("Either m_flow_con or dT_con are required")
    if m_flow_con is not None:
        con_array = ensure_array(m_flow_con)
        use_m_flow_con = True
    else:
        use_m_flow_con = False
        con_array = ensure_array(dT_con)

    all_arrays = [
        n,
        T_con,
        T_eva_in,
        con_array,
        m_flow_eva,
        dT_eva_superheating,
        dT_con_subcooling
    ]

    if isinstance(save_path, str):
        save_path = pathlib.Path(save_path)
    if algorithm is None:
        algorithm = Iteration()
    if save_plots:
        algorithm.save_path_plots = pathlib.Path(save_path).joinpath(
            f"plots_{flowsheet.flowsheet_name}_{flowsheet.fluid}"
        )
        os.makedirs(algorithm.save_path_plots, exist_ok=True)

    list_mp_inputs = []
    list_inputs = []
    idx_for_access_later = []
    global_med_prop_data = media.get_global_med_prop_and_kwargs()

    # Create meshgrid for all input combinations
    meshgrid = np.meshgrid(*all_arrays, indexing='ij')

    # Flatten arrays for iteration
    combinations = [arr.flatten() for arr in meshgrid]

    # Check which inputs are nd
    is_nd = np.array([len(arr) > 1 for arr in all_arrays])

    for i in range(len(combinations[0])):
        single_n = float(combinations[0][i])
        single_T_con = float(combinations[1][i])
        single_T_eva_in = float(combinations[2][i])
        single_con_val = float(combinations[3][i])
        single_m_flow_eva_val = float(combinations[4][i])
        single_dT_eva_sh = float(combinations[5][i])
        single_dT_con_sc = float(combinations[6][i])

        idx = np.unravel_index(i, [len(arr) for arr in all_arrays])
        idx_for_access_later.append(idx)

        control_inputs = RelativeCompressorSpeedControl(
            n=single_n,
            dT_eva_superheating=single_dT_eva_sh,
            dT_con_subcooling=single_dT_con_sc
        )

        evaporator_inputs = HeatExchangerInputs(
            T_in=single_T_eva_in,
            m_flow=single_m_flow_eva_val
        )
        if use_m_flow_con:
            con_kwargs = dict(m_flow=single_con_val)
        else:
            con_kwargs = dict(dT=single_con_val)

        if use_condenser_inlet:
            T_con_kwargs = dict(T_in=single_T_con)
        else:
            T_con_kwargs = dict(T_out=single_T_con)
        condenser_inputs = HeatExchangerInputs(**T_con_kwargs, **con_kwargs)

        inputs = Inputs(
            control=control_inputs,
            evaporator=evaporator_inputs,
            condenser=condenser_inputs
        )

        list_mp_inputs.append([algorithm, flowsheet, inputs, raise_errors, global_med_prop_data])
        list_inputs.append(inputs)

    fs_states = []
    i = 0

    if use_multiprocessing:
        pool = multiprocessing.Pool(processes=10)
        for fs_state in pool.imap(_calc_single_state, list_mp_inputs):
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(list_mp_inputs)} points")
    else:
        for single_input in list_mp_inputs:
            fs_state = _calc_single_state(single_input)
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(list_mp_inputs)} points")

    # Result shape based on input dimensions
    result_shape = tuple(len(arr) for arr in all_arrays if len(arr) > 1)

    _dummy = np.zeros(result_shape)
    if result_shape:
        _dummy[:] = np.nan
    # Get all possible values:
    all_variables = {}
    all_variables_info = {}
    variables_to_excel = []
    for fs_state, inputs in zip(fs_states, list_inputs):
        all_variables.update({var: _dummy.copy() for var in fs_state.get_variable_names()})
        all_variables_info.update({var: variable for var, variable in fs_state.get_variables().items()})
        variables_to_excel.append({
            **fs_state.convert_to_str_value_format(with_unit_and_description=False),
        })

    # Save to excel
    save_path_sdf = save_path.joinpath(f"{flowsheet.flowsheet_name}_{flowsheet.fluid}.sdf")
    save_path_csv = save_path.joinpath(f"{flowsheet.flowsheet_name}_{flowsheet.fluid}.csv")
    pd.DataFrame(variables_to_excel).to_csv(
        save_path_csv, sep=","
    )

    # Terminate heat pump med-props:
    flowsheet.terminate()
    if not save_sdf:
        return save_path_csv

    if not result_shape:
        raise IndexError("No inputs are varied, saving as sdf is not possible")

    for fs_state, idx in zip(fs_states, idx_for_access_later):
        idx_nd = tuple(i for i, i_is_nd in zip(idx, is_nd) if i_is_nd)
        for variable_name, variable in fs_state.get_variables().items():
            all_variables[variable_name][idx_nd] = variable.value

    # The order needs to be the same as all_arrays
    possible_scale_values = {
        "n": n,
        "T_con_in" if use_condenser_inlet else "T_con_out": T_con,
        "T_eva_in": T_eva_in,
        "m_flow_con" if use_m_flow_con else "dT_con": m_flow_con if use_m_flow_con else dT_con,
        "m_flow_eva": m_flow_eva,
        "dT_eva_superheating": dT_eva_superheating,
        "dT_con_subcooling": dT_con_subcooling,
    }

    _scale_values = {}
    for scale_name, values in possible_scale_values.items():
        if len(values) > 1:
            _scale_values[scale_name] = values

    # Use the first non-None entry, only relevant for unit and description
    for fs_state in fs_states:
        if fs_state == FlowsheetState():  # Empty means None
            fs_state_for_scales = fs_state
            break
    else:
        raise ValueError("Only empty flowsheet states, can't generate sdf.")
    _scales = {}
    for name, data in _scale_values.items():
        _scales[name] = {
            "data": data,
            "unit": fs_state_for_scales.get(name).unit,
            "comment": fs_state_for_scales.get(name).description
        }

    _nd_data = {}
    _parameters = {}
    for variable, nd_data in all_variables.items():
        if variable in _scales:
            continue
        if (nd_data == nd_data.min()).all():
            # All values are constant:
            _parameters[variable] = {
                "data": nd_data.min(),
                "unit": all_variables_info[variable].unit,
                "comment": all_variables_info[variable].description
            }
        else:
            _nd_data.update({
                variable: {
                    "data": nd_data,
                    "unit": all_variables_info[variable].unit,
                    "comment": all_variables_info[variable].description}
            })

    sdf_data = {
        flowsheet.flowsheet_name:
            {
                flowsheet.fluid: (_scales, _nd_data, _parameters)
            }
    }
    utils.save_to_sdf(data=sdf_data, save_path=save_path_sdf)

    return save_path_sdf, save_path_csv


def _calc_single_state(data):
    """Helper function for a single state to enable multiprocessing"""
    algorithm, flowsheet, inputs, raise_errors, global_med_prop_data = data
    fs_state = None
    media.set_global_media_properties(global_med_prop_data[0], **global_med_prop_data[1])
    try:
        fs_state = algorithm.calc_steady_state(
            flowsheet=flowsheet, inputs=inputs
        )
    except Exception as e:
        if raise_errors:
            raise e
        logger.error(f"An error occurred for input: {inputs.get_name()}: {e}")
    if fs_state is None:
        fs_state = FlowsheetState()
    # Append the data to the dataframe
    return fs_state
