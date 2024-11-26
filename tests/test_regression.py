import shutil
import unittest
import logging
import pathlib
import os
from random import random

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from vclibpy import utils
from vclibpy import datamodels
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor
from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU
from vclibpy.flowsheets import StandardCycle, VaporInjectionEconomizer, VaporInjectionPhaseSeparator
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components.heat_exchangers import heat_transfer
from vclibpy.algorithms import Iteration


def _load_flowsheet(fluid: str, flowsheet: str = None):
    flowsheets = {
        "Standard": StandardCycle,
        "VaporInjectionPhaseSeparator": VaporInjectionPhaseSeparator,
        "VaporInjectionEconomizer": VaporInjectionEconomizer
    }
    N_max = 125
    V_h = 19e-6
    compressor = RotaryCompressor(
        N_max=N_max,
        V_h=V_h
    )
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
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=15,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=5000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000)
    )
    economizer = VaporInjectionEconomizerNTU(
        A=2,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=50000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50000),
    )
    expansion_valve = Bernoulli(A=0.1)  # Should not matter
    kwargs = dict(
        evaporator=evaporator,
        condenser=condenser,
        fluid=fluid
    )
    if flowsheet == "Standard":
        kwargs.update(dict(
            expansion_valve=expansion_valve,
            compressor=compressor
        ))
    else:
        kwargs.update(dict(
            high_pressure_compressor=compressor,
            low_pressure_compressor=compressor,
            high_pressure_valve=expansion_valve,
            low_pressure_valve=expansion_valve
        ))
    if flowsheet == "VaporInjectionEconomizer":
        kwargs.update(dict(
            economizer=economizer
        ))
    return flowsheets[flowsheet](**kwargs)


class TestRegressionWithAllFluidsAndFlowsheets(unittest.TestCase):
    """
    Return the settings used in the master thesis and following
    publication: https://doi.org/10.1016/j.enconman.2021.114888
    """

    def setUp(self) -> None:
        self.fs_state = datamodels.FlowsheetState()
        self.var = datamodels.Variable(name="test", value=random(), unit="K", description="dummy")
        import uuid
        self.working_dir = pathlib.Path(__file__).parent.joinpath(str(uuid.uuid4()))
        os.makedirs(self.working_dir, exist_ok=True)

        self.inputs_to_compare = [
            "n",
            "T_eva_in",
            "T_con_in",
        ]

    def _regression_of_examples(self, flowsheet, fluid):
        self.results_to_compare = [
            "Q_con",
            "COP",
            "m_flow_ref",
            "T_1",
            "T_2",
            "T_3",
            "T_4",
            "p_con",
            "p_eva",
            "A_eva_sh",
            "A_eva_lat",
            "A_con_sh",
            "A_con_lat",
            "A_con_sc",
            "carnot_quality",
        ]

        # Select the settings / parameters of the algorithm:
        algorithm = Iteration(
            max_err=0.5,
            max_err_dT_min=0.1,
            show_iteration=False,
            max_num_iterations=5000
        )

        # Just for quick study: Specify concrete points:
        T_eva_in_ar = [-10 + 273.15, 273.15]
        T_con_ar = [30 + 273.15, 70 + 273.15]
        n_ar = [0.3, 1]

        os.makedirs(self.working_dir, exist_ok=True)

        flowsheet = _load_flowsheet(
            fluid=fluid,
            flowsheet=flowsheet
        )
        _, path_csv = utils.full_factorial_map_generation(
            flowsheet=flowsheet,
            save_path=self.working_dir,
            T_con_ar=T_con_ar,
            T_eva_in_ar=T_eva_in_ar,
            n_ar=n_ar,
            use_multiprocessing=False,
            save_plots=False,
            raise_errors=True,
            m_flow_con=0.2,
            m_flow_eva=0.9,
            dT_eva_superheating=5,
            dT_con_subcooling=0,
            algorithm=algorithm
        )
        path_csv_regression = pathlib.Path(__file__).parent.joinpath(
            "regression_data", "reference_results", f"{flowsheet}_{fluid}.csv"
        )
        self.assertTrue(self._compare_results(path_csv, path_csv_regression))

    def test_standard_propane(self):
        self._regression_of_examples("Standard", "Propane")

    def test_standard_R410A(self):
        self._regression_of_examples("Standard", "R410A")

    def test_vi_ps_propane(self):
        self._regression_of_examples("VaporInjectionPhaseSeparator", "Propane")

    def test_evi_propane(self):
        #self.skipTest("EVI works locally, only CI fails.")
        self._regression_of_examples("VaporInjectionEconomizer", "Propane")

    @unittest.skip("not implemented")
    def test_opti_horst_regression(self):
        self.results_to_compare = [
            "Q_con",
            "COP",
            "m_flow_ref",
            "T_1",
            "T_2",
            "T_3",
            "T_4",
            "p_con",
            "p_eva",
            "p_2",
            "A_eva_sh",
            "A_eva_lat",
            "A_con_sh",
            "A_con_lat",
            "A_con_sc",
            "carnot_quality",
        ]
        path_csv = pathlib.Path(__file__).parent.joinpath(
            "regression_data", "temp", "OptiHorst_R410A.csv"
        )
        path_csv_regression = pathlib.Path(__file__).parent.joinpath(
            "regression_data", "publication", "OptiHorst_R410A.csv"
        )
        no_error = self._compare_results(path_csv, path_csv_regression)
        self.assertTrue(no_error)

    def _compare_results(self, path_csv, path_csv_regression):
        df = pd.read_csv(path_csv, index_col=0)
        df_regression = pd.read_csv(path_csv_regression, index_col=0)
        # Rename columns as with_unit_and_description=False in automation:
        df_regression.columns = [col.split(" ")[0] for col in df_regression.columns]
        # Compare if all inputs are present
        # Use this result object to pretty print any errors
        results_comparison = {col: {} for col in self.results_to_compare}
        for idx, row in df.iterrows():
            input_mask = np.ones(len(df_regression)) == 1  # All True
            for inp in self.inputs_to_compare:
                # We use is_close as floating errors may lead to 0.6 being 0.600000000002
                is_close = np.isclose(df_regression.loc[:, inp], row[inp])
                self.assertTrue(np.any(is_close))
                input_mask = input_mask & is_close
            for col in self.results_to_compare:
                new_value = row[col]
                old_value = df_regression.loc[input_mask, col].values[0]
                if np.isnan(new_value) and old_value in [np.nan, 1, 0]:
                    continue
                results_comparison[col][new_value] = old_value
        no_errors = []
        error_info = []
        for col in self.results_to_compare:
            res_new = np.array(list(results_comparison[col].keys()))
            res_old = np.array(list(results_comparison[col].values()))
            nan_mask = np.isnan(res_old)
            res_new = res_new[~nan_mask]
            res_old = res_old[~nan_mask]
            zero_mask = res_old == 0
            res_new = res_new[~zero_mask]
            res_old = res_old[~zero_mask]

            # Smaller 1 percent error
            deviation = (res_new - res_old) / res_old * 100
            deviation_ok = deviation < 1
            no_regression_error = np.all(deviation_ok)
            if not no_regression_error:
                error_info.append(f"{col}: {deviation[~deviation_ok]}")
                plt.figure()
                plt.plot(res_old[~deviation_ok], label="old results", marker="s")
                plt.plot(res_new[~deviation_ok], label="New results", marker="o")
                plt.ylabel(col)
                plt.legend()
                os.makedirs(self.working_dir.parent.joinpath("regression_errors"), exist_ok=True)
                col_clean = col.replace("/", "_")
                plt.savefig(self.working_dir.parent.joinpath("regression_errors", f"{path_csv.stem}_{col_clean}.png"))
                plt.close(plt.gcf())
            no_errors.append(no_regression_error)
        if not np.all(no_errors):
            print("Errors for the following variables:")
            for _error_info in error_info:
                print(_error_info)
        return np.all(no_errors)


def tearDown(self) -> None:
    try:
        shutil.rmtree(self.working_dir)
    except PermissionError:
        logging.error(
            "Could not delete folders due to PermissionError, delete them yourself: %s",
            self.working_dir
        )


if __name__ == "__main__":
    unittest.main()
