from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor, ConstantEffectivenessCompressor


def create_heat_pump(A_cond, A_ihx):
    """
    Erstellt das Wärmepumpen-Modell mit variablen Flächen.
    """

    # Kondensator (Variable Fläche!)
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=A_cond,  # <--- HIER VARIABLE
        secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3500),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4000)
    )

    # Verdampfer (Fix)
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=43.50,
        secondary_medium="air", flow_type="cross",
        ratio_outer_to_inner_area=105,
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=400),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=64)
    )

    # IHX (Variable Fläche!)
    ihx_component = ihx_ntu.IHX_NTU(
        A=A_ihx,  # <--- HIER VARIABLE
        alpha_low_side=500,
        alpha_high_side=2000,
        dT_pinch_min=0.5,
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3)
    )

    # Ventile & Verdichter (Fix)
    expansion_valve_high = Bernoulli(A=0.2)
    expansion_valve_low = Bernoulli(A=0.000002)

    compressor = ConstantEffectivenessCompressor(
        N_max=110, V_h=54.8e-6, eta_isentropic=0.7,
        eta_mech=0.95 * 0.95 * 0.95, lambda_h=0.9
    )

    heat_pump = ihx.IHX(
        evaporator=evaporator, condenser=condenser, fluid="Propane",
        compressor=compressor, expansion_valve_high=expansion_valve_high,
        expansion_valve_low=expansion_valve_low, ihx=ihx_component
    )

    return heat_pump