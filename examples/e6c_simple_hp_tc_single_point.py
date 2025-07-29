def calculate_single_point():
    import os
    import pandas as pd
    import time
    from vclibpy.flowsheets import StandardCycleTranscritical
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer

    condenser = moving_boundary_ntu.MovingBoundaryNTUGasCooler(
        A=0.8,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1200),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
    )
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=0.250,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1200),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
    )
    from vclibpy.components.expansion_valves import Bernoulli
    expansion_valve = Bernoulli(A=0.1)

    from vclibpy.components.compressors import RotaryCompressor
    compressor = RotaryCompressor(
        N_max=125,
        V_h=8.34e-6,
        eta_is_const = 0.75
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
    from vclibpy.algorithms.iteration_tc_dev import Iteration_TC
    from vclibpy.utils.plotting import plot_cycle
    from vclibpy.utils.automation import create_timestamped_folder

    base_save_path = r"D:\00_temp\Standard_TC_SP"
    timestamped_save_path = create_timestamped_folder(base_path=base_save_path, prefix="SinglePointRun")
    print(f"Info: Result-folder for this run created: {timestamped_save_path}")

    algorithm = Iteration_TC(raise_errors=True, save_path_plots=timestamped_save_path, show_iteration=False)
    speed_control = RelativeCompressorSpeedControl(0.2, 5.0, 0)
    eva_inputs = HeatExchangerInputs(T_in=5 + 273.15, m_flow=1)
    con_inputs = HeatExchangerInputs(T_in=35 + 273.15, m_flow=1)
    inputs = Inputs(control=speed_control, evaporator=eva_inputs, condenser=con_inputs)

    start_time = time.perf_counter()


    fs_state = algorithm.calc_steady_state(flowsheet, inputs, "CarbonDioxide")

    end_time = time.perf_counter()
    duration_seconds = end_time - start_time

    # Following code is for output and csv export of the results
    if fs_state is not None:
        print("\n--- Calculation successful! Results: ---")

        print(f"\nRuntime of code: {duration_seconds:.4f} seconds")
        try:
            runtime_save_path = os.path.join(timestamped_save_path, "runtime.txt")
            with open(runtime_save_path, "w") as f:
                f.write(f"Runtime in seconds: {duration_seconds}\n")
            print(f"Runtime successfully saved in: {runtime_save_path}")
        except Exception as e:
            print(f"Error while saving the runtime: {e}")

        try:
            print("\n--- Saving plot data (including secondary side) to CSV ---")

            cycle_states = flowsheet.get_states_in_order_for_plotting()
            all_plot_points = []

            for i, s in enumerate(cycle_states):
                all_plot_points.append(
                    {'label': f'cycle_point_{i}', 'h_kJ_kg': s.h / 1000, 'T_C': s.T - 273.15, 'p_bar': s.p / 1e5})

            h_sat = flowsheet.med_prop.get_two_phase_limits('h')
            T_sat = flowsheet.med_prop.get_two_phase_limits('T')
            p_sat = flowsheet.med_prop.get_two_phase_limits('p')
            split_idx = len(h_sat) // 2

            for i in range(split_idx):
                all_plot_points.append({'label': 'sat_liquid', 'h_kJ_kg': h_sat[i] / 1000, 'T_C': T_sat[i] - 273.15,
                                        'p_bar': p_sat[i] / 1e5})
            for i in range(split_idx, len(h_sat)):
                all_plot_points.append({'label': 'sat_vapor', 'h_kJ_kg': h_sat[i] / 1000, 'T_C': T_sat[i] - 273.15,
                                        'p_bar': p_sat[i] / 1e5})
            all_plot_points.append({'label': 'sec_condenser', 'h_kJ_kg': flowsheet.condenser.state_outlet.h / 1000,
                                    'T_C': flowsheet.condenser.T_in - 273.15, 'p_bar': None})
            all_plot_points.append({'label': 'sec_condenser', 'h_kJ_kg': flowsheet.condenser.state_inlet.h / 1000,
                                    'T_C': flowsheet.condenser.T_out - 273.15, 'p_bar': None})
            all_plot_points.append({'label': 'sec_evaporator', 'h_kJ_kg': flowsheet.evaporator.state_outlet.h / 1000,
                                    'T_C': flowsheet.evaporator.T_in - 273.15, 'p_bar': None})
            all_plot_points.append({'label': 'sec_evaporator', 'h_kJ_kg': flowsheet.evaporator.state_inlet.h / 1000,
                                    'T_C': flowsheet.evaporator.T_out - 273.15, 'p_bar': None})

            df_plot = pd.DataFrame(all_plot_points)
            csv_path = os.path.join(timestamped_save_path, "final_plot_data_full.csv")
            df_plot.to_csv(csv_path, sep=';', decimal=',', index=False)
            print(f"Plot data with secondary side successfully saved to: {csv_path}")

        except Exception as e:
            print(f"\n--- ERROR: Could not save plot data to CSV. Reason: {e} ---")

        # plot_cycle(flowsheet.med_prop, flowsheet.get_states_in_order_for_plotting(), show=True)
        print(f"Compressor:\n"
              f"m_flow = {flowsheet.compressor.m_flow * 3600} kg/h\n"
              f"state_inlet = {flowsheet.compressor.state_inlet}\n"
              f"state_outlet = {flowsheet.compressor.state_outlet}\n"
              )
        print(f"Evaporator:\n"
              f"m_flow = {flowsheet.evaporator.m_flow * 3600} kg/h\n"
              f"state_inlet = {flowsheet.evaporator.state_inlet}\n"
              f"state_outlet = {flowsheet.evaporator.state_outlet}\n"
              f"Temperature secondary_inlet = {flowsheet.evaporator.T_in}\n"
              f"Temperature secondary_outlet = {flowsheet.evaporator.T_out}\n"
              )
        print(f"Condenser:\n"
              f"m_flow = {flowsheet.condenser.m_flow * 3600} kg/h\n"
              f"state_inlet = {flowsheet.condenser.state_inlet}\n"
              f"state_outlet = {flowsheet.condenser.state_outlet}\n"
              f"Temperature secondary_inlet = {flowsheet.condenser.T_in}\n"
              f"Temperature secondary_outlet = {flowsheet.condenser.T_out}\n"
              )
        print(f"COP: {fs_state.get('COP').value}")
        print(f"Q_con: {fs_state.get('Q_con').value} W")
        print(f"P_el: {fs_state.get('P_el').value} W")
        print(f"Calculated pressure drop in gas cooler: {fs_state.get('delta_p_gas_cooler').value:.2f} Pa")
        print(f"{flowsheet.condenser.pinch_point_analysis()}")
    else:
        print(
            "\n--- Calculation NOT successfull. Algorithm couldn't find a solution for given Inputs. ---")

if __name__ == "__main__":
    calculate_single_point()
