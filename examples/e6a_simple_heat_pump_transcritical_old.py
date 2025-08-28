# # Example for a heat pump with a standard cycle

import os

def main(use_condenser_inlet: bool = True):
    # Let's start the complete cycle simulation with the
    # most basic flowsheet, the standard-cycle. As all flowsheets
    # contain a condenser/gas cooler and an evaporator, we defined a common BaseCycle
    # to avoid code-repetition.
    # We can import this flowsheet and see how to use it. Note that
    # modern coding IDEs like PyCharm will tell you which arguments belong
    # to a class or function. If you don't have that at hand, you need
    # to look into the documentation.
    from vclibpy.flowsheets import BaseCycle, StandardCycleTranscritical
    #help(BaseCycle)
    #help(StandardCycleTranscritical)

    # We fist need to define the components in the cycle.
    # Here we are using the components developed in the previous examples.
    # Also, note again that the expansion valve model does not influence the results
    # for the current algorithm. But, you could size the expansion valve
    # using vclibpy, including off-design, but this is one for another example.
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    condenser = moving_boundary_ntu.MovingBoundaryNTUGasCooler(
        A=80,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=30,
        secondary_medium="air",
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
        V_h=19e-6
    )

    # Now, we can plug everything into the flowsheet:
    flowsheet = StandardCycleTranscritical(
        evaporator=evaporator,
        condenser=condenser,
        fluid="CO2",
        compressor=compressor,
        expansion_valve=expansion_valve,
    )
    # As in the other example, we can specify save-paths,
    # solver settings and inputs to vary:
    # Note that T_con can either be inlet or outlet, depending on the setting
    # of `use_condenser_inlet`. Per default, we simulate the inlet, T_con_in
    save_path = r"D:\00_temp\simple_heat_pump"
    T_eva_in_ar = [10 + 273.15]
    T_con_ar = [20 + 273.15]
    n_ar = [0.7]

    # Now, we can generate the full-factorial performance map
    # using all inputs. The results will be stored under the
    # save-path. To see some logs, we can import the logging module
    # and get, for example, all messages equal or above the INFO-level
    import logging
    logging.basicConfig(level="INFO")

    from vclibpy import utils
    save_path_sdf, save_path_csv = utils.full_factorial_map_generation(
        flowsheet=flowsheet,
        save_path=save_path,
        T_con=T_con_ar,
        T_eva_in=T_eva_in_ar,
        n=n_ar,
        use_condenser_inlet=use_condenser_inlet,
        use_multiprocessing=False,
        save_plots=True,
        m_flow_con=1,
        m_flow_eva=1,
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
    df = pd.read_csv(save_path_csv, index_col=0)
    df
    # Now, we can plot variables, for example as a scatter plot using matplotlib.
    # You have to know the names, which are the column headers.
    import matplotlib.pyplot as plt
    x_name = "n in - (Relative compressor speed)"
    y_name = "COP in - (Coefficient of performance)"
    plt.scatter(df[x_name], df[y_name], s=20)
    plt.ylabel(y_name)
    plt.xlabel(x_name)
    plt.show()
    # Looking at the results, we see that a higher frequency often leads to lower COP values.
    # However, other inputs (temperatures) have a greater impact on the COP.
    # We can also use existing 3D-plotting scripts in vclibpy to analyze the
    # dependencies. For this, we need the .sdf file. In the sdf, the field names are without
    # the unit and description, as those are accessible in the file-format in other columns.
    # Depending on whether we varied the inlet or outlet, we have to specify the correct name.
    from vclibpy.utils.plotting import plot_sdf_map
    plot_sdf_map(
        filepath_sdf=save_path_sdf,
        nd_data="COP",
        first_dimension="T_eva_in",
        second_dimension="T_con_in" if use_condenser_inlet else "T_con_out",
        fluids=["CO2"],
        flowsheets=["StandardTranscritical"]
    )

def calculate_single_point():
    from vclibpy.flowsheets import StandardCycleTranscritical
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    from vclibpy.algorithms.iteration import Iteration

    condenser = moving_boundary_ntu.MovingBoundaryNTUGasCooler(
        A=30,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=30,
        secondary_medium="air",
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
        V_h=19e-6
    )

    # Now, we can plug everything into the flowsheet:
    flowsheet = StandardCycleTranscritical(
        evaporator=evaporator,
        condenser=condenser,
        fluid="CO2",
        compressor=compressor,
        expansion_valve=expansion_valve,
    )

    import logging
    logging.basicConfig(level="INFO")

    from vclibpy.datamodels import Inputs, RelativeCompressorSpeedControl, HeatExchangerInputs
    from vclibpy.algorithms.iteration import Iteration
    from vclibpy.utils.plotting import plot_cycle

    save_path = r"D:\00_temp\Standard_TC_SP"
    if not os.path.exists(save_path):
        os.makedirs(save_path)
        print(f"Info: Save path {save_path} has been created.")

    algorithm = Iteration(raise_errors=True, save_path_plots=save_path, show_iteration=True)
    speed_control = RelativeCompressorSpeedControl(0.2, 0.0, 0)
    eva_inputs = HeatExchangerInputs(T_in=18 + 273.15, m_flow=1)
    con_inputs = HeatExchangerInputs(T_in=28 + 273.15, m_flow=1)
    inputs = Inputs(control=speed_control, evaporator=eva_inputs, condenser=con_inputs)

    p_1_start, p_2_start, p_max, fs_state = algorithm.initial_setup(flowsheet, fluid=None, inputs=inputs)

    # flowsheet.calc_states(1e6, 6e6, inputs, fs_state)

    fs_state = algorithm.calc_steady_state(flowsheet, inputs, "CarbonDioxide")
    #plot_cycle(flowsheet.med_prop, flowsheet.get_states_in_order_for_plotting(), show=True)
    print(f"Compressor:")
    print(f"m_flow = {flowsheet.compressor.m_flow * 3600} kg/h")
    print(f"state_inlet = {flowsheet.compressor.state_inlet}")
    print(f"state_outlet = {flowsheet.compressor.state_outlet}")
    print(f"Condenser:")
    print(f"m_flow = {flowsheet.condenser.m_flow * 3600} kg/h")
    print(f"state_inlet = {flowsheet.condenser.state_inlet}")
    print(f"state_outlet = {flowsheet.condenser.state_outlet}")
    print(f"Temperature secondary_inlet = {flowsheet.condenser.T_in}")
    print(f"Temperature secondary_outlet = {flowsheet.condenser.T_out}")
    print(f"Evaporator:")
    print(f"m_flow = {flowsheet.evaporator.m_flow * 3600} kg/h")
    print(f"state_inlet = {flowsheet.evaporator.state_inlet}")
    print(f"state_outlet = {flowsheet.evaporator.state_outlet}")
    print(f"COP: {fs_state.get('COP').value}")



def test_gas_cooler():
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    from vclibpy.datamodels import Inputs, HeatExchangerInputs
    from vclibpy.media.ref_prop import RefProp
    from vclibpy.datamodels import FlowsheetState
    import numpy as np

    med_prop = RefProp(fluid_name="CarbonDioxide")

    condenser = moving_boundary_ntu.MovingBoundaryNTUGasCooler(
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
    condenser.med_prop = med_prop
    condenser.m_flow = 0.02
    condenser.start_secondary_med_prop()

    condenserInputs = HeatExchangerInputs(T_in=273.15+20, m_flow=0.1)
    inputs = Inputs(condenser=condenserInputs)
    fs_state = FlowsheetState()

    condenser.calc_secondary_cp(inputs.condenser.T)

    # p = 12e6
    # condenser.state_inlet = med_prop.calc_state("PH", p, 494670)
    # condenser.state_outlet = med_prop.calc_state("PT", p, 273.15+23)
    #
    # error, dT_min, q_ntu, q = condenser.calc(inputs=inputs, fs_state=fs_state)
    # print(f"Error: {error}")
    # print(f"dT_min: {dT_min}")
    # print(f"Q_ntu: {q_ntu}")
    # print(f"Q: {q}")
    # print(f"Condenser state inlet: {condenser.state_inlet}")
    # print(f"Condenser state outlet: {condenser.state_outlet}")
    # print(f"Condenser state inlet secondary: {condenser.T_in}")
    # print(f"Condenser state outlet secondary: {condenser.T_out}")



    pressures = np.arange(7.5e6, 17e6, 10000)
    errors = []
    Q_ntu = []
    Q = []

    for p in pressures:
        condenser.state_inlet = med_prop.calc_state("PH", p, 494670)
        condenser.state_outlet = med_prop.calc_state("PH", p, 289520)
        error, dT_min, q_ntu, q = condenser.calc(inputs=inputs, fs_state=fs_state)
        errors.append(error)
        Q_ntu.append(q_ntu)
        Q.append(q)

    # Plotting the results
    import matplotlib.pyplot as plt

    fig, ax1 = plt.subplots()

    # Plot errors on the primary y-axis
    ax1.plot(pressures / 1e5, errors, 'b-', label='Error')
    ax1.set_xlabel('Pressure (bar)')
    ax1.set_ylabel('Error', color='b')
    ax1.tick_params(axis='y', labelcolor='b')

    # Create a secondary y-axis for Q_ntu and Q
    ax2 = ax1.twinx()
    ax2.plot(pressures / 1e5, Q_ntu, 'r--', label='Q_ntu')
    ax2.plot(pressures / 1e5, Q, 'g-.', label='Q')
    ax2.set_ylabel('Heat Transfer (Q)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    # Add legends
    fig.legend(loc="upper right", bbox_to_anchor=(1, 1), bbox_transform=ax1.transAxes)

    # Add grid and title
    plt.title('Error and Heat Transfer vs Pressure')
    plt.grid()
    plt.show()




def test_condenser():
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer
    from vclibpy.datamodels import Inputs, HeatExchangerInputs
    from vclibpy.media.ref_prop import RefProp
    from vclibpy.datamodels import FlowsheetState
    import numpy as np

    med_prop = RefProp(fluid_name="CarbonDioxide")

    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=80,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=10,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    condenser.med_prop = med_prop
    condenser.m_flow = 0.01
    condenser.start_secondary_med_prop()

    condenserInputs = HeatExchangerInputs(T_in=273.15-50, m_flow=0.2)
    inputs = Inputs(condenser=condenserInputs)
    fs_state = FlowsheetState()

    condenser.calc_secondary_cp(inputs.condenser.T)

    pressures = np.arange(8e5, 7e6, 10000)
    errors = []
    dT_minimum = []
    Q_ntu = []
    Q = []
    Q_sc_ntu = []
    Q_lat_ntu = []
    Q_sh_ntu = []

    for p in pressures:
        condenser.state_inlet = med_prop.calc_state("PH", p, 454670)
        condenser.state_outlet = med_prop.calc_state("PH", p, 189520)
        error, dT_min, q_ntu, q, q_sc_ntu, q_lat_ntu, q_sh_ntu = condenser.calc(inputs=inputs, fs_state=fs_state)
        errors.append(error)
        dT_minimum.append(dT_min)
        Q_ntu.append(q_ntu)
        Q.append(q)
        Q_sc_ntu.append(q_sc_ntu)
        Q_lat_ntu.append(q_lat_ntu)
        Q_sh_ntu.append(q_sh_ntu)


    # Plotting the results
    import matplotlib.pyplot as plt

    fig, ax1 = plt.subplots()

    # Plot errors on the primary y-axis
    ax1.plot(pressures / 1e5, errors, 'b-', label='Error')
    ax1.set_xlabel('Pressure (bar)')
    ax1.set_ylabel('Error', color='b')
    ax1.tick_params(axis='y', labelcolor='b')

    # Create a secondary y-axis for Q_ntu and Q
    ax2 = ax1.twinx()
    ax2.plot(pressures / 1e5, Q_ntu, 'r--', label='Q_ntu')
    ax2.plot(pressures / 1e5, Q, 'g-.', label='Q')
    ax2.plot(pressures / 1e5, Q_sc_ntu, 'm:', label='Q_sc_ntu')
    ax2.plot(pressures / 1e5, Q_lat_ntu, 'c:', label='Q_lat_ntu')
    ax2.plot(pressures / 1e5, Q_sh_ntu, 'y:', label='Q_sh_ntu')
    ax2.set_ylabel('Heat Transfer (Q)', color='r')
    ax2.tick_params(axis='y', labelcolor='r')

    ax3 = ax1.twinx()
    ax3.plot(pressures / 1e5, dT_minimum, 'k--', label='dT_min')
    ax3.set_ylabel('dT_min', color='k')
    ax3.tick_params(axis='y', labelcolor='k')


    # Add legends
    fig.legend(loc="upper right", bbox_to_anchor=(1, 1), bbox_transform=ax1.transAxes)

    # Add grid and title
    plt.title('Error and Heat Transfer vs Pressure')
    plt.grid()
    plt.show()


if __name__ == "__main__":
    # test_gas_cooler()
    # test_condenser()
    # main(use_condenser_inlet=True)
    calculate_single_point()
