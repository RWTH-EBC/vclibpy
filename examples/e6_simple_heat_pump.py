# # Example for a heat pump with a standard cycle

def main():
    # Let's start the complete cycle simulation with the
    # most basic flowsheet, the standard-cycle. As all flowsheets
    # contain a condenser and an evaporator, we defined a common BaseCycle
    # to avoid code-repetition.
    # We can import this flowsheet and see how to use it. Note that
    # modern coding IDEs like PyCharm will tell you which arguments belong
    # to a class or function. If you don't have that at hand, you need
    # to look into the documentation.
    from vclibpy.flowsheets import BaseCycle, StandardCycle
    help(BaseCycle)
    help(StandardCycle)

    # We fist need to define the components in the cycle.
    # Here we are using the components developed in the previous examples.
    # Also, note again that the expansion valve model does not influence the results
    # for the current algorithm. But, you could size the expansion valve
    # using vclibpy, including off-design, but this is one for another example.
    from vclibpy.components.heat_exchangers import moving_boundary_ntu, simple_ntu, simple_lmtd, moving_boundary_lmtd
    from vclibpy.components.heat_exchangers import heat_transfer
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
    )

    condenser = simple_ntu.SimpleNTUCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        primary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
    )

    condenser = simple_lmtd.SimpleLMTDCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        primary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000)
    )

    condenser = moving_boundary_lmtd.MovingBoundaryLMTDCondenser(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
    )

    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=15,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )

    evaporator = simple_lmtd.SimpleLMTDEvaporator(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        primary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000)
    )

    evaporator = moving_boundary_lmtd.MovingBoundaryLMTDEvaporator(
        A=5,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    from vclibpy.components.expansion_valves import Bernoulli
    expansion_valve = Bernoulli(A=0.1)

    from vclibpy.components.compressors import RotaryCompressor
    compressor = RotaryCompressor(
        N_max=125,
        V_h=19e-6,
        #eps_vol=2.2,
        # eta_isentropic=0.92,
        # lambda_h=0.92,
        # eta_mech=1
    )

    # Now, we can plug everything into the flowsheet:
    heat_pump = StandardCycle(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        compressor=compressor,
        expansion_valve=expansion_valve,
    )
    # As in the other example, we can specify save-paths,
    # solver settings and inputs to vary:
    save_path = r"C:\users\cedri\00_python\00_temp\simple_heat_pump"
    T_eva_in_ar = [-10 + 273.15, 273.15, 10 + 273.15]
    T_con_in_ar = [30 + 273.15, 50 + 273.15, 70 + 273.15]
    n_ar = [0.7]

    # Now, we can generate the full-factorial performance map
    # using all inputs. The results will be stored under the
    # save-path. To see some logs, we can import the logging module
    # and get, for example, all messages equal or above the INFO-level
    import logging
    logging.basicConfig(level="INFO")

    from vclibpy import utils
    save_path_sdf, save_path_csv = utils.full_factorial_map_generation(
        heat_pump=heat_pump,
        save_path=save_path,
        T_con_in_ar=T_con_in_ar,
        T_eva_in_ar=T_eva_in_ar,
        n_ar=n_ar,
        use_multiprocessing=False,
        save_plots=False,
        m_flow_con=0.2,
        m_flow_eva=0.9,
        dT_eva_superheating=5,
        dT_con_subcooling=0,
    )
    # What did just happen? We can analyze all results by listing the
    # files in the save-path - or just open it in our default system explorer.
    import os
    print(os.listdir(save_path))
    # One file should be: `Standard_Propane.csv`. We can load this file and plot
    # the values using e.g. pandas. It is also the second return value of the function.
    import pandas as pd
    df = pd.read_csv(save_path_csv, index_col=0, sep= ";")
    df
    # Now, we can plot variables, for example as a scatter plot using matplotlib.
    # You have to know the names, which are the column headers.
    import matplotlib.pyplot as plt
    x_name = "n in - (Relative compressor speed)"
    y_name = "COP in - (Coefficient of performance)"
    plt.scatter(df[x_name], df[y_name])
    plt.ylabel(y_name)
    plt.xlabel(x_name)
    plt.show()
    # Looking at the results, we see that a higher frequency often leads to lower COP values.
    # However, other inputs (temperatures) have a greater impact on the COP.
    # We can also use existing 3D-plotting scripts in vclibpy to analyze the
    # dependencies. For this, we need the .sdf file. In the sdf, the field names are without
    # the unit and description, as those are accessible in the file-format in other columns.
    from vclibpy.utils.plotting import plot_sdf_map
    plot_sdf_map(
        filepath_sdf=save_path_sdf,
        nd_data="COP",
        first_dimension="T_eva_in",
        second_dimension="T_con_in",
        fluids=["Propane"],
        flowsheets=["Standard"]
    )


if __name__ == "__main__":
    main()
