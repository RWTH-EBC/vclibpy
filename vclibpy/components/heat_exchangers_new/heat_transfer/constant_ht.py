from vclibpy.components.heat_exchangers_new.heat_transfer.base import BaseHeatTransfer


class ConstantHT(BaseHeatTransfer):
    """

    """
    def __init__(self,
                 discret: bool = False,
                 **kwargs):

        self.discret = discret
        if not discret:
            self.U = kwargs.get("U", 2400)
        else:
            self.alpha_cold = kwargs.get("alpha_cold", 2400)
            self.alpha_warm = kwargs.get("alpha_warm", 2400)
        return

    def calc(self, **kwargs) -> float:
        if not self.discret:
            return self.U
        else:
            lambda_wall = kwargs.get("lambda_wall")
            delta_wall = kwargs.get("wall_thickness")
            return 1/((1 / self.alpha_cold) + (1 / self.alpha_warm) + (delta_wall / lambda_wall))
