from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import RotaryCompressor, ConstantEffectivenessCompressor


def create_heat_pump(A_cond, A_ihx):
    """
    Erstellt das thermodynamische Flowsheet-Modell der Propan-Wärmepumpe.

    Die Flächen für den Kondensator und den internen Wärmeübertrager (IHX)
    werden als Parameter übergeben, um sie durch externe Skripte (wie
    optimization.py) variieren zu können.
    """

    # KONDENSATOR: Ausgelegt als MovingBoundary Modell mit variabler Fläche
    condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
        A=A_cond,  # Die zu optimierende Geometriegröße
        secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
        # Wärmeübergangskoeffizienten (Alphas) für die jeweiligen Zonen:
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3500),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4000)
    )

    # VERDAMPFER: Fixe Geometrie (auf die Anlage abgestimmt)
    evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
        A=43.50,
        secondary_medium="air", flow_type="cross",
        ratio_outer_to_inner_area=105,  # Berücksichtigung der berippten Rohrstruktur
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=400),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=2000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=64)  # Luftseite (niedrig)
    )

    # INTERNER WÄRMEÜBERTRAGER (IHX)
    ihx_component = ihx_ntu.IHX_NTU(
        A=A_ihx,  # Variable Fläche, basierend auf realen Modellen (z.B. Danfoss)
        alpha_low_side=525,
        alpha_high_side=818,
        dT_pinch_min=0.5,  # Numerische Stabilitätsgrenze für Pinch
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=400, thickness=1e-3)
    )

    # EXPANSIONSVENTILE (Modelliert nach Bernoulli)
    expansion_valve_high = Bernoulli(A=0.2)
    expansion_valve_low = Bernoulli(A=0.000002)

    # VERDICHTER: Vereinfachtes Modell mit konstanten Gütegraden
    compressor = ConstantEffectivenessCompressor(
        N_max=110, V_h=54.8e-6,  # Maximaldrehzahl und Hubvolumen
        eta_isentropic=0.7,  # Isentroper Gütegrad
        eta_mech=0.95 * 0.95 * 0.95,  # Mechanischer Wirkungsgrad
        lambda_h=0.9  # Liefergrad
    )

    # Zusammenbau der Einzelkomponenten zum zyklischen System (Flowsheet)
    heat_pump = ihx.IHX(
        evaporator=evaporator, condenser=condenser, fluid="Propane",  # Fluid-Wahl R290
        compressor=compressor, expansion_valve_high=expansion_valve_high,
        expansion_valve_low=expansion_valve_low, ihx=ihx_component
    )

    return heat_pump