import numpy as np
import matplotlib.pyplot as plt
from vclibpy.components.heat_exchangers import moving_boundary_ntu
from vclibpy.components.heat_exchangers import heat_transfer
from vclibpy.media import CoolProp

def test_gas_cooler():

    """
    Isolierter Test für den MovingBoundaryNTUGasCooler, um einen
    internen Pinch-Punkt gezielt zu erzeugen und zu analysieren.
    """
    print("--- Starte isolierten Gaskühler-Test ---")

    # 1. Komponente und Stoffdaten initialisieren
    condenser = moving_boundary_ntu.MovingBoundaryNTUGasCooler(
        A=80, secondary_medium="air", flow_type="counter",
        ratio_outer_to_inner_area=10,
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=2e-3),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=5000),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=25),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1000)  # Hinzugefügt
    )
    med_prop = CoolProp(fluid_name="CO2")
    condenser.med_prop = med_prop
    condenser.start_secondary_med_prop()

    # 2. Gezielt einen Fall mit internem Pinch definieren
    p2_bar = 97.78  # Druck nahe am kritischen Punkt, um "Bauch" zu erzeugen

    # Kältemittel-Zustände manuell setzen
    condenser.state_inlet = med_prop.calc_state("PT", p2_bar * 1e5, 120 + 273.15)
    condenser.state_outlet = med_prop.calc_state("PT", p2_bar * 1e5, 30 + 273.15)
    condenser.m_flow = 0.1  # Beispiel-Massenstrom

    # Sekundärmedium-Temperaturen so wählen, dass sie den "Bauch" schneiden
    condenser.T_in = 32 + 273.15  # Kaltes Ende
    condenser.T_out = 41.54 + 273.15  # Heißes Ende

    # 3. Die Analyse-Methode direkt aufrufen
    pinch_details = condenser.pinch_point_analysis()

    # 4. Ergebnisse ausgeben
    print("\n--- Analyse-Ergebnis ---")
    print(pinch_details)

    # 5. Visuelle Überprüfung mit einem T-h-Diagramm
    steps = 20
    h_steps = np.linspace(condenser.state_inlet.h, condenser.state_outlet.h, steps + 1)
    T_ref_steps = [med_prop.calc_state("PH", condenser.state_inlet.p, h).T for h in h_steps]
    T_sec_steps = np.linspace(condenser.T_in, condenser.T_out, steps + 1)

    plt.figure(figsize=(10, 6))
    plt.plot(np.array(h_steps) / 1000, np.array(T_ref_steps) - 273.15, 'r-', label='CO2 (Primärmedium)')
    plt.plot(np.array(h_steps) / 1000, np.array(T_sec_steps[::-1]) - 273.15, 'b-', label='Luft (Sekundärmedium)')
    plt.xlabel("Enthalpie h [kJ/kg]")
    plt.ylabel("Temperatur T [°C]")
    plt.title(f"T-h Diagramm für p = {p2_bar} bar")
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":
    test_gas_cooler()
