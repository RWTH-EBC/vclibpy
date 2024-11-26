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
from vclibpy import utils

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
    if use_multiprocessing:
        mp_inputs = [[algorithm, flowsheet, inputs_, raise_errors] for inputs_ in inputs]
        pool = multiprocessing.Pool(processes=min(multiprocessing.cpu_count(), len(inputs)))
        for fs_state in pool.imap(_calc_single_state, mp_inputs):
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(inputs)} points")
    else:
        for inputs_ in inputs:
            fs_state = _calc_single_state([algorithm, flowsheet, inputs_, raise_errors])
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
        T_eva_in_ar: Union[list, np.ndarray],
        T_con_ar: Union[list, np.ndarray],
        n_ar: Union[list, np.ndarray],
        m_flow_con: float,
        m_flow_eva: float,
        save_path: Union[pathlib.Path, str],
        algorithm: Algorithm = None,
        dT_eva_superheating=5,
        dT_con_subcooling=0,
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
        T_eva_in_ar (list):
            Array with inputs for T_eva_in
        T_con_ar (list):
            Array with inputs for T_con_in or T_con_out, see `use_condenser_inlet`
        n_ar (list):
            Array with inputs for n_ar
        m_flow_con (float):
            Condenser mass flow rate
        m_flow_eva (float):
            Evaporator mass flow rate
        save_path (Path):
            Where to save all results.
        algorithm (Algorithm):
            A supported algorithm to calculate a steady state.
            If None, Iteration algorithm is used with default settings.
        dT_eva_superheating (float):
            Evaporator superheating
        dT_con_subcooling (float):
            Condenser subcooling
        use_condenser_inlet (bool):
            True to consider T_con_ar as inlet, false for outlet.
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
    for i_T_eva_in, T_eva_in in enumerate(T_eva_in_ar):
        for i_n, n in enumerate(n_ar):
            for i_T_con, T_con in enumerate(T_con_ar):
                idx_for_access_later.append([i_n, i_T_con, i_T_eva_in])
                control_inputs = RelativeCompressorSpeedControl(
                    n=n,
                    dT_eva_superheating=dT_eva_superheating,
                    dT_con_subcooling=dT_con_subcooling
                )
                evaporator_inputs = HeatExchangerInputs(
                    T_in=T_eva_in,
                    m_flow=m_flow_eva
                )
                if use_condenser_inlet:
                    condenser_inputs = HeatExchangerInputs(
                        T_in=T_con,
                        m_flow=m_flow_eva
                    )
                else:
                    condenser_inputs = HeatExchangerInputs(
                        T_out=T_con,
                        m_flow=m_flow_con
                    )
                inputs = Inputs(
                    control=control_inputs,
                    evaporator=evaporator_inputs,
                    condenser=condenser_inputs
                )
                list_mp_inputs.append([algorithm, flowsheet, inputs, raise_errors])
                list_inputs.append(inputs)
    fs_states = []
    i = 0
    if use_multiprocessing:
        # pool = multiprocessing.Pool(processes=multiprocessing.cpu_count())
        pool = multiprocessing.Pool(processes=10)
        for fs_state in pool.imap(_calc_single_state, list_mp_inputs):
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(list_mp_inputs)} points")
    else:
        for inputs in list_inputs:
            fs_state = _calc_single_state([algorithm, flowsheet, inputs, raise_errors])
            fs_states.append(fs_state)
            i += 1
            logger.info(f"Ran {i} of {len(list_mp_inputs)} points")

    # Save to sdf
    result_shape = (len(n_ar), len(T_con_ar), len(T_eva_in_ar))
    _dummy = np.zeros(result_shape)  # Use a copy to avoid overwriting of values of any sort.
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
        save_path_csv, sep=";"
    )

    # Terminate heat pump med-props:
    flowsheet.terminate()
    if not save_sdf:
        return save_path_csv

    for fs_state, idx_triple in zip(fs_states, idx_for_access_later):
        i_n, i_T_con, i_T_eva_in = idx_triple
        for variable_name, variable in fs_state.get_variables().items():
            all_variables[variable_name][i_n][i_T_con][i_T_eva_in] = variable.value

    _nd_data = {}
    for variable, nd_data in all_variables.items():
        _nd_data.update({
            variable: {
                "data": nd_data,
                "unit": all_variables_info[variable].unit,
                "comment": all_variables_info[variable].description}
        })

    _scale_values = {
        "n": n_ar,
        "T_con_in" if use_condenser_inlet else "T_con_out": T_con_ar,
        "T_eva_in": T_eva_in_ar
    }
    inputs: Inputs = list_inputs[0]
    # TODO: Automatically mark constant values are constant!
    _parameters = {}
    #for name, variable in inputs.items():
    #    if name not in list(_scale_values.keys()) + ["T_con_out", "T_con_in"]:
    #        _parameters[name] = {
    #            "data": variable.value,
    #            "unit": variable.unit,
    #            "comment": variable.description
    #        }
    _scales = {}
    for name, data in _scale_values.items():
        _scales[name] = {
            "data": data,
            "unit": inputs.get(name).unit,
            "comment": inputs.get(name).description
        }

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
    algorithm, flowsheet, inputs, raise_errors = data
    fs_state = None
    try:
        fs_state = algorithm.calc_steady_state(
            flowsheet=flowsheet, inputs=inputs
        )
    except Exception as e:
        if raise_errors:
            raise e
        logger.error(f"An error occurred for input: {inputs.__dict__}: {e}")
    if fs_state is None:
        fs_state = FlowsheetState()
    # Append the data to the dataframe
    return fs_state
