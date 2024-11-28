import logging
from typing import Union

import numpy as np
from scipy.optimize import fsolve

from vclibpy import Inputs, FlowsheetState
from vclibpy.algorithms.base import Algorithm
from vclibpy.flowsheets import BaseCycle


logger = logging.getLogger(__name__)


class FSolve(Algorithm):
    """
    Algorithm to calculate steady states using scipy's fsolve.
    """

    def calc_steady_state(
            self,
            flowsheet: BaseCycle,
            inputs: Inputs,
            fluid: str = None
    ) -> Union[FlowsheetState, None]:
        p_1_start, p_2_start, _p_max, fs_state = self.initial_setup(
            flowsheet=flowsheet, inputs=inputs, fluid=fluid
        )

        def nonlinear_func(x, *_args):
            _flowsheet, _inputs, _fs_state, _max_err = _args
            _p_1, _p_2 = x
            if not (_p_1 < _p_2 < _p_max):
                return 1000, 1000
            if not (self._p_min < _p_1 < _p_2):
                return 1000, 1000
            _error_eva, dT_min_eva, _error_con, dT_min_con = _flowsheet.calculate_cycle_for_pressures(
                p_1=x[0], p_2=x[1], inputs=_inputs, fs_state=_fs_state
            )

            if 0 <= _error_eva < _max_err:
                _error_eva = 0
            if 0 <= _error_con < _max_err:
                _error_con = 0
            if 0 > _error_eva:
                _error_eva *= 5
            if 0 > _error_con:
                _error_con *= 5
            return _error_eva, _error_con

        try:
            args = (flowsheet, inputs, fs_state, self.max_err)
            p_optimized = fsolve(
                func=nonlinear_func,
                x0=np.array([p_1_start, p_2_start]),
                args=args
            )
        except Exception as err:
            logger.error("An error occurred while calculating states using fsolve: %s", err)
            if self.raise_errors:
                raise err
            return
        error_con, error_eva = nonlinear_func(p_optimized, *args)
        logger.info(f"{error_con=}, {error_eva=}")

        if len(p_optimized) != 2:
            raise ValueError("Given optimal result contains more or less than 2 pressure levels")
        p_1, p_2 = p_optimized[0], p_optimized[1]
        return flowsheet.calculate_outputs_for_valid_pressures(
            p_1=p_1, p_2=p_2, inputs=inputs, fs_state=fs_state,
            save_path_plots=self.save_path_plots
        )
