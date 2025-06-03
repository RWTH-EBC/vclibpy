# # Example for a heat pump with vapor injection using a phase separator

def main():
    # Let's use a flowsheet which is more complex, e.g. the vapor injection
    # with a phase seperator.
    # We can import this flowsheet and how to use it like this:
    from vclibpy.flowsheets import VaporInjectionPhaseSeparator
    help(VaporInjectionPhaseSeparator)

    # As it needs the same heat exchanger model as a standard heat pump,
    # we will just use the ones from the standard cycle. Also, as
    # the expansion valve model does not influence the results for
    # the current algorithm, we will just use the same expansion-valve
    # twice. Note, that you could size the two expansion valves
    # using vclibpy, including off-design, but this is one for another
    # example.
    from vclibpy.components.heat_exchangers import moving_boundary_ntu
    from vclibpy.components.heat_exchangers import heat_transfer

    from vclibpy.components.heat_exchangers import mvb_new
    from vclibpy.components.heat_exchangers import heat_transfer

    condenser = mvb_new.MVB_Condenser(
        A=3,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        model_approach="ntu", # or use "lmtd"
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=250),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=2400),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4500)
    )
    # condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    #     A=2.4,
    #     secondary_medium="water",
    #     flow_type="counter",
    #     ratio_outer_to_inner_area=1,
    #     two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=2400),
    #     gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1200),
    #     wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=0.6e-3),
    #     liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
    #     secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500)
    # )

    evaporator = mvb_new.MVB_Evaporator(
        A=5,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        model_approach="ntu", # or use "lmtd"
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=150),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=100)
    )
    #
    # evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    #     A=15,
    #     secondary_medium="air",
    #     flow_type="counter",
    #     ratio_outer_to_inner_area=1,
    #     two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
    #     gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1200),
    #     wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=0.6e-3),
    #     liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
    #     secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500)
    # )
    from vclibpy.components.expansion_valves import Bernoulli
    high_pressure_valve = Bernoulli(A=0.1)
    low_pressure_valve = Bernoulli(A=0.1)
    # For the compressors, we need to specify low- and high-pressure
    # compressors. To achieve a somewhat similar heat pump as the
    # one in the standard-cycle example, we will  assume that we
    # use two smaller compressors instead of one larger one:
    from vclibpy.components.compressors import RotaryCompressor
    high_pressure_compressor = RotaryCompressor(
        N_max=120,
        V_h=30e-6 / 2
    )
    low_pressure_compressor = RotaryCompressor(
        N_max=120,
        V_h=30e-6 / 2
    )

    # Now, we can plug everything into the flowsheet:
    heat_pump = VaporInjectionPhaseSeparator(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        high_pressure_compressor=high_pressure_compressor,
        low_pressure_compressor=low_pressure_compressor,
        high_pressure_valve=high_pressure_valve,
        low_pressure_valve=low_pressure_valve                               #TODO: warum nicht der phase_separator?
    )
    # As in the other example, we can specify save-paths,
    # solver settings and inputs to vary:
    save_path = r"D:\00_temp\vapor_injection"
    T_eva_in_ar = [-20 + 273.15, 12 + 273.15]
    T_con_in_ar = [35 + 273.15, 75 + 273.15]
    n_ar = [1]
    k_vapor_injection_ar = [0.5, 1]

    # Now, we can generate the full-factorial performance map
    # using all inputs. The results will be stored under the
    # save-path. To see some logs, we can import the logging module
    # and get, for example, all messages equal or above the INFO-level
    import logging
    logging.basicConfig(level="INFO")

    from vclibpy import utils
    utils.full_factorial_map_generation(
        heat_pump=heat_pump,
        save_path=save_path,
        T_con_in_ar=T_con_in_ar,
        T_eva_in_ar=T_eva_in_ar,
        n_ar=n_ar,
        use_multiprocessing=False,
        save_plots=True,
        m_flow_con=0.75,
        m_flow_eva=2.7,
        dT_eva_superheating=5,
        dT_con_subcooling=3,
        k_vapor_injection_ar=k_vapor_injection_ar,
    )
    # As in the prior examples, feel free to load the plots,
    # .csv or .sdf result files and further analyze them
    # # Vapor injection with an economizer.
    # Aside from the phase-separator flowsheet, we have one with
    # an economizer (additional heat exchanger).
    # The assumptions are similar, and the usage as well:
    from vclibpy.flowsheets import VaporInjectionEconomizer
    help(VaporInjectionEconomizer)

    # We need an additional economizer, which can be found in the heat exchangers:
    from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU
    help(VaporInjectionEconomizerNTU)
    # Let's assume some dummy parameters:
    economizer = VaporInjectionEconomizerNTU(
        A=0.2,
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=200),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
    )
    # And create the heat pump, and run the map generation:
    heat_pump = VaporInjectionEconomizer(
        evaporator=evaporator,
        condenser=condenser,
        fluid="Propane",
        economizer=economizer,
        high_pressure_compressor=high_pressure_compressor,
        low_pressure_compressor=low_pressure_compressor,
        high_pressure_valve=high_pressure_valve,
        low_pressure_valve=low_pressure_valve
    )
    utils.full_factorial_map_generation(
        heat_pump=heat_pump,
        save_path=r"D:\00_temp\vapor_injection_economizer",
        T_con_in_ar=T_con_in_ar,
        T_eva_in_ar=T_eva_in_ar,
        n_ar=n_ar,
        use_multiprocessing=False,
        save_plots=True,
        m_flow_con=0.75,
        m_flow_eva=2.7,
        dT_eva_superheating=5,
        dT_con_subcooling=3,
        k_vapor_injection_ar=k_vapor_injection_ar,
    )


if __name__ == "__main__":
    main()
