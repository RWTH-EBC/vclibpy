from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.heat_exchangers_new.base import Regmine
class HX_MVB_LMTD():

    def __init__(self):
        return

    def calc(self, inputs: Inputs, fs_state: FlowsheetState) -> (float, float):

        regime = self.get_regime()

        A_calc = 0

        for _regime in regime:
            A_regime = _regime.calc_A()
            if A_regime > 0:
                A_calc += A_regime




    def get_regime(self):

        regime: [Regmine] = []

        return regime
