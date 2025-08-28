import os
import pandas as pd
import time
import logging

from vclibpy.flowsheets import StandardCycleTranscritical
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components.heat_exchangers import heat_transfer
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor
from vclibpy.datamodels import Inputs, RelativeCompressorSpeedControl, HeatExchangerInputs
from vclibpy.algorithms.iteration_tc_dev import Iteration_TC

logging.basicConfig(level="INFO")


def run_analysis():
    steps_part1 = list(range(51, 150, 1))


    steps_to_test = steps_part1
    results_list = []

    base_save_path = r"D:\00_temp\Convergence_Analysis"
    if not os.path.exists(base_save_path):
        os.makedirs(base_save_path)

    print(f"Starte Konvergenzanalyse für {len(steps_to_test)} verschiedene Step-Anzahlen...")

    for step_count in steps_to_test:
        print("-" * 50)
        logging.info(f"Beginne Berechnung für steps = {step_count}")
        start_time = time.perf_counter()

        # Der Aufruf hier enthält nun alle notwendigen Argumente,
        # kopiert aus deinem e6c-Skript, plus den 'steps'-Parameter.
        condenser = moving_boundary_ntu.MovingBoundaryNTUGasCooler(
            A=80,
            secondary_medium="air",
            flow_type="counter",
            steps=step_count,  # Unser variabler Parameter
            ratio_outer_to_inner_area=10,
            two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
            gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
            wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
            liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
            secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
        )
        evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
            A=30, secondary_medium="air", flow_type="counter", ratio_outer_to_inner_area=10,
            two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000),
            gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
            wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
            liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
            secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25)
        )
        compressor = RotaryCompressor(N_max=125, V_h=19e-6, eta_is_const=0.75)
        expansion_valve = Bernoulli(A=0.1)

        flowsheet = StandardCycleTranscritical(
            evaporator=evaporator, condenser=condenser, fluid="CO2",
            compressor=compressor, expansion_valve=expansion_valve
        )

        algorithm = Iteration_TC(raise_errors=False)

        speed_control = RelativeCompressorSpeedControl(0.2, 5.0, 0)
        eva_inputs = HeatExchangerInputs(T_in=5 + 273.15, m_flow=1)
        con_inputs = HeatExchangerInputs(T_in=28 + 273.15, m_flow=1)
        inputs = Inputs(control=speed_control, evaporator=eva_inputs, condenser=con_inputs)

        fs_state = algorithm.calc_steady_state(flowsheet, inputs, "CarbonDioxide")
        runtime = time.perf_counter() - start_time

        if fs_state and fs_state.get('COP'):
            cop = fs_state.get('COP').value
            results_list.append({'steps': step_count, 'COP': cop, 'runtime_s': runtime})
            logging.info(f"Berechnung für steps = {step_count} erfolgreich. COP={cop:.4f}, Laufzeit={runtime:.2f}s")
        else:
            results_list.append({'steps': step_count, 'COP': None, 'runtime_s': runtime})
            logging.warning(f"Berechnung für steps = {step_count} NICHT erfolgreich.")

    df_results = pd.DataFrame(results_list)
    output_path = os.path.join(base_save_path, "convergence_analysis_results.csv")
    df_results.to_csv(output_path, sep=';', decimal=',', index=False)

    print("\n" + "=" * 50)
    print(f"Analyse abgeschlossen! Ergebnisse gespeichert in: {output_path}")
    print("Zusammenfassung der Ergebnisse:")
    print(df_results)


if __name__ == "__main__":
    run_analysis()