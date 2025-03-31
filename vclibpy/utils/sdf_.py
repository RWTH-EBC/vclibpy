import itertools
import pathlib

import pandas as pd
import sdf


def save_to_sdf(data: dict, save_path: pathlib.Path):
    """
    Save given input dictionary to a sdf file in the given save_path

    Args:
        data (dict):
            A dictionary with the following structure:
            Keys: Flowsheet_name
            Values: A dictionary with the following structure:

            - Keys: Fluids
            - Values: Data dictionaries with the following structure:
                A tuple with three values, in that order:

                - scales: Of the given nd_data, e.g. T_Amb, n
                - nd_data: More-dimensional data, e.g. COP
                - parameters: Scalar values, like m_flow_con or similar

        save_path (pathlib.Path): Where to store the data
        flowsheet_name (str): Name of the flowsheet. This is the top level group
    :return:
    """
    if isinstance(save_path, str):
        save_path = pathlib.Path(save_path)
    _all_groups = []

    for flowsheet_name, fluid_dict in data.items():
        _all_fluids = []
        for fluid, fluid_data in fluid_dict.items():
            # First write scales
            _scales = []
            for scale_name, scale_values in fluid_data[0].items():
                _scales.append(sdf.Dataset(scale_name,
                                           data=scale_values["data"],
                                           unit=scale_values["unit"],
                                           is_scale=True,
                                           display_name=scale_name,
                                           comment=scale_values.get("comment", "")))
            # Now the ND-Data:
            _nd_data = []
            for data_name, data_values in fluid_data[1].items():
                _nd_data.append(sdf.Dataset(data_name,
                                            data=data_values["data"],
                                            unit=data_values["unit"],
                                            scales=_scales,
                                            comment=data_values.get("comment", "")))
            # Now the constant parameters
            _paras = []
            for para_name, para_value in fluid_data[2].items():
                _paras.append(sdf.Dataset(para_name,
                                          data=para_value["data"],
                                          unit=para_value["unit"],
                                          comment=para_value.get("comment", "")))

            # Save everything
            fluid_group = sdf.Group(fluid, comment="Values for fluid", datasets=_scales + _nd_data + _paras)
            _all_fluids.append(fluid_group)

        flowsheet_group = sdf.Group(flowsheet_name,
                                    comment="Multiple fluids for the flowsheet",
                                    groups=_all_fluids)
        _all_groups.append(flowsheet_group)

    parent_group = sdf.Group("/", comment="Generated with VCLibPy", groups=_all_groups)
    sdf.save(save_path, group=parent_group)
    return save_path


def merge_sdfs(filepaths, save_path):
    """
    Merge given files and return a merged file.
    Be careful if both files contain the same combination.
    Then, the latter element of the list will overwrite the first one.

    Args:
        filepaths (list): List with paths to the files
        save_path (str): Save path for the new file
    """
    _all_flowsheets = {}
    # Merge to structure
    for fpath in filepaths:
        dataset = sdf.load(fpath)
        for flowsheet in dataset.groups:
            fluid_data = {fluid.name: fluid for fluid in flowsheet.groups}
            if flowsheet.name not in _all_flowsheets:
                _all_flowsheets.update({flowsheet.name: fluid_data})
            else:
                _all_flowsheets[flowsheet.name].update(fluid_data)

    # Write structure
    _all_groups = []
    for flowsheet_name, fluid_dict in _all_flowsheets.items():
        _all_fluids = []
        for fluid, data in fluid_dict.items():
            _all_fluids.append(data)
        flowsheet_group = sdf.Group(flowsheet_name,
                                    comment="Multiple fluids for the flowsheet",
                                    groups=_all_fluids)
        _all_groups.append(flowsheet_group)

    parent_group = sdf.Group("/", comment="Generated with python script", groups=_all_groups)
    sdf.save(save_path, group=parent_group)
    return save_path


def sdf_to_csv(filepath: pathlib.Path, save_path: pathlib.Path):
    """
    Convert a given .sdf file to multiple excel files,
    for each combination of flowsheet and refrigerant one file.

    Args:
        filepath (pathlib.Path): sdf file
        save_path (pathlib.Path): Directory where to store the csv files.
    """
    dataset = sdf.load(str(filepath))
    for flowsheet in dataset.groups:
        for fluid in flowsheet.groups:
            dfs = []
            for data in fluid.datasets:
                if _is_nd(data):
                    dfs.append(_unpack_nd_data(data))
            df = pd.concat(dfs, axis=1)
            df = df.loc[:, ~df.columns.duplicated()]
            df.to_csv(save_path.joinpath(f"{flowsheet.name}_{fluid.name}.csv"))


def _get_name(data):
    return f"{data.name} in {data.unit} ({data.comment})"


def _is_nd(data):
    if data.scales == [None]:
        return False
    return True


def _unpack_nd_data(data):
    column_name = _get_name(data)
    scale_names = [_get_name(scale) for scale in data.scales]
    scales_with_idx = [[(idx, value) for idx, value in enumerate(scale.data)] for scale in data.scales]
    all_data = []
    for scales in itertools.product(*scales_with_idx):
        indexer = tuple([scale[0] for scale in scales])
        values = [scale[1] for scale in scales]
        all_data.append({
            column_name: data.data[indexer],
            **{name: value for name, value in zip(scale_names, values)}
        })
    return pd.DataFrame(all_data)
