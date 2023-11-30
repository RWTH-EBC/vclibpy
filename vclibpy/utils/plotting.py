import pathlib
from typing import List

import numpy as np
import matplotlib.pyplot as plt
import sdf


def plot_sdf_map(
        filepath_sdf: pathlib.Path,
        nd_data: str,
        first_dimension: str,
        second_dimension: str,
        fluids: List[str] = None,
        flowsheets: List[str] = None,
        violin_plot_variable: str = None,
        third_dimension: str = None
):
    """
    Generate and display visualizations based on data from an SDF (Structured Data File) dataset.
    This function generates various types of visualizations based on the provided parameters,
    including 3D scatter plots, 3D surface plots, and violin plots, and displays them using Matplotlib.

    Args:
        filepath_sdf (pathlib.Path):
            The path to the SDF dataset file.
        nd_data (str):
            The name of the primary data to be plotted.
        first_dimension (str):
            The name of the first dimension for the visualization.
        second_dimension (str):
            The name of the second dimension for the visualization.
        fluids (List[str], optional):
            List of specific fluids to include in the visualization.
            Default is None, which includes all fluids.
        flowsheets (List[str], optional):
            List of specific flowsheets to include in the visualization.
            Default is None, which includes all flowsheets.
        violin_plot_variable (str, optional):
            The variable to be used for creating violin plots.
            Default is None, which disables violin plots.
        third_dimension (str, optional):
            The name of the third dimension for 4D visualizations.
            Default is None, which disables 4D plotting.

    Raises:
        KeyError: If the specified data or dimensions are not found in the dataset.

    Examples:
    >>> FILEPATH_SDF = r"HeatPumpMaps.sdf"
    >>> plot_sdf_map(
    >>>     filepath_sdf=FILEPATH_SDF,
    >>>     nd_data="COP",
    >>>     first_dimension="T_eva_in",
    >>>     second_dimension="n",
    >>>     fluids=["R410A"],
    >>>     flowsheets=["OptiHorn"],
    >>> )

    """
    if fluids is None:
        fluids = []
    if flowsheets is None:
        flowsheets = []
    if "T_" in second_dimension:
        offset_sec = -273.15
    else:
        offset_sec = 0
    if "T_" in first_dimension:
        offset_pri = -273.15
    else:
        offset_pri = 0
    offset_thi = 0
    plot_4D = False
    if third_dimension is not None:
        plot_4D = True
        if "T_" in third_dimension:
            offset_thi = -273.15

    dataset = sdf.load(str(filepath_sdf))
    plot_violin = True
    if violin_plot_variable is None:
        plot_violin = False
        violin_plot_variable = ""
    if plot_violin:
        if flowsheets:
            n_rows = len(flowsheets)
        else:
            n_rows = len(dataset.groups)
        fig_v, ax_v = plt.subplots(nrows=n_rows, ncols=1, sharex=True,
                                   squeeze=False)
        fig_v.suptitle(violin_plot_variable)
    i_fs = 0
    nd_str_plot = nd_data
    fac = 1
    if nd_data == "dT_eva_min":
        nd_data = "T_1"
        sub_str = "T_eva_in"
        fac = - 1
    elif nd_data == "dT_con":
        nd_data = "T_3"
        sub_str = "T_con_in"
    elif nd_data == "dT_sh":
        nd_data = "T_1"
        sub_str = "T_4"
    else:
        sub_str = ""

    if nd_str_plot.startswith("T_"):
        offset_nd = -273.15
    else:
        offset_nd = 0

    for flowsheet in dataset.groups:
        violin_data = {}
        if flowsheet.name not in flowsheets and len(flowsheets) > 0:
            continue
        for fluid in flowsheet.groups:
            if fluid.name not in fluids and len(fluids) > 0:
                continue
            nd, fd, sd, sub_data, td = None, None, None, None, None
            _other_scale = {}
            for ds in fluid.datasets:
                if ds.name == nd_data:
                    nd = ds
                elif ds.name == first_dimension:
                    fd = ds.data
                elif ds.name == second_dimension:
                    sd = ds.data
                if ds.name == sub_str:
                    sub_data = ds.data
                if ds.name == violin_plot_variable:
                    data = ds.data.flatten()
                    violin_data[fluid.name] = data[~np.isnan(data)]
                if plot_4D and ds.name == third_dimension:
                    td = ds.data

            if nd is None:
                raise KeyError("nd-String not found in dataset")

            if sub_data is None:
                sub_data = np.zeros(nd.data.shape)

            # Check other scales:
            for i, scale in enumerate(nd.scales):
                if scale.name not in [first_dimension, second_dimension]:
                    _other_scale[i] = scale

            if fd is None or sd is None or not _other_scale or (plot_4D and td is None):
                raise KeyError("One of the given strings was not found in dataset")

            if plot_4D:
                fig = plt.figure()
                figtitle = f"{flowsheet.name}_{fluid.name}_{nd_str_plot}"
                fig.suptitle(figtitle)
                ax = fig.add_subplot(111, projection='3d')
                ax.set_xlabel(first_dimension)
                ax.set_ylabel(second_dimension)
                ax.set_zlabel(third_dimension)
                fourth_dim = (nd.data - sub_data) * fac + offset_nd
                # Scale values for better sizes of circles:
                bounds = [fourth_dim.min(), fourth_dim.max()]
                _max_circle_size = 30
                fourth_dim_scaled = (fourth_dim - bounds[0]) / (bounds[1] - bounds[0]) * _max_circle_size
                inp = [fd + offset_pri, sd + offset_sec, td + offset_thi]
                import itertools
                scattergrid = np.array([c for c in itertools.product(*inp)])
                ax.scatter(scattergrid[:, 0],
                           scattergrid[:, 1],
                           scattergrid[:, 2],
                           c=fourth_dim_scaled,
                           s=fourth_dim_scaled)
            else:
                for index, scale in _other_scale.items():
                    for idx_data, value in enumerate(scale.data):
                        if index==0:
                            Z = nd.data[idx_data, :, :]
                            if sub_str in ["T_4", ""]:
                                sub_data_use = sub_data[idx_data, :, :]
                            else:
                                sub_data_use = sub_data
                        elif index == 1:
                            Z = nd.data[:, idx_data, :]
                            if sub_str in ["T_4", ""]:
                                sub_data_use = sub_data[:, idx_data, :]
                            else:
                                sub_data_use = sub_data
                        else:
                            Z = nd.data[:, :, idx_data]
                            if sub_str in ["T_4", ""]:
                                sub_data_use = sub_data[:, :, idx_data]
                            else:
                                sub_data_use = sub_data
                        if not plot_violin:
                            fig = plt.figure()
                            figtitle = f"{flowsheet.name}_{fluid.name}_{nd_str_plot}_{scale.name}={round(value, 3)}"
                            fig.suptitle(figtitle)
                            ax = fig.add_subplot(111, projection='3d')
                            ax.set_xlabel(first_dimension)
                            ax.set_ylabel(second_dimension)
                            X, Y = np.meshgrid(fd, sd)
                            ax.plot_surface(X + offset_pri, Y + offset_sec, (Z - sub_data_use)*fac + offset_nd)

        if plot_violin:
            for key, value in violin_data.items():
                print(f"{violin_plot_variable}: {flowsheet.name}_{key}")
                print(f"Min: {np.min(value)}")
                print(f"Max: {np.max(value)}")
                print(f"Mean: {np.mean(value)}")
                print(f"Median: {np.median(value)}\n")
            ax_v[i_fs][0].violinplot(
                                     list(violin_data.values()),
                                     showextrema=True,
                                     showmeans=True,
                                     showmedians=True
                                     )
            set_axis_style(ax_v[i_fs][0], list(violin_data.keys()))
            ax_v[i_fs][0].set_ylabel(flowsheet.name.replace("Flowsheet", ""))
        i_fs += 1
    plt.show()


def set_axis_style(ax, labels):
    """
    From: https://matplotlib.org/3.1.1/gallery/statistics/customized_violin.html#sphx-glr-gallery-statistics-customized-violin-py
    """
    ax.get_xaxis().set_tick_params(direction='out')
    ax.xaxis.set_ticks_position('bottom')
    ax.set_xticks(np.arange(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_xlim(0.25, len(labels) + 0.75)
