import logging
import numpy as np
from vclibpy.flowsheets import ihx
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer, ihx_ntu
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import ConstantEffectivenessCompressor
from vclibpy.datamodels import Inputs, ControlInputs, HeatExchangerInputs
from vclibpy.algorithms.fsolve import FSolve

# Logging aktivieren
logging.basicConfig(level=logging.INFO)

print("--- START DEBUGGING ---")

# 1. KOMPONENTEN (Deine Konfiguration)

# Kondensator
condenser = moving_boundary_ntu.MovingBoundaryNTUCondenser(
    A=2.2, secondary_medium="water", flow_type="counter", ratio_outer_to_inner_area=1,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=4000),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=15, thickness=0.4e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4000)
)

# Verdampfer
evaporator = moving_boundary_ntu.MovingBoundaryNTUEvaporator(
    A=108.0, secondary_medium="air", flow_type="cross", ratio_outer_to_inner_area=105,
    two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=1500),
    gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=236, thickness=1e-3),
    liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1000),
    secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=50)
)

# IHX
ihx_component = ihx_ntu.IHX_NTU(
    A=0.15, alpha_low_side=1000, alpha_high_side=2000, dT_pinch_min=0.5,
    wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=15, thickness=1e-3)
)

# --- WICHTIGE ÄNDERUNG: VENTILGRÖSSEN ---
# Wir gehen auf 2 mm² runter (2.0e-6).
# Das sorgt für den nötigen Staudruck!
expansion_valve_high = Bernoulli(A=2.0e-6)
expansion_valve_low = Bernoulli(A=2.0e-6)

compressor = ConstantEffectivenessCompressor(
    N_max=110, V_h=54.8e-6, eta_isentropic=0.7, eta_mech=0.9, lambda_h=0.9
)

# Cycle
heat_pump = ihx.IHX(
    evaporator=evaporator, condenser=condenser, fluid="Propane",
    compressor=compressor, expansion_valve_high=expansion_valve_high,
    expansion_valve_low=expansion_valve_low, ihx=ihx_component
)

# 2. EINZELNER TEST-PUNKT (A7 / W35)
# Wir definieren EINEN Input manuell, um zu sehen, ob die Physik stimmt.

inputs = Inputs(
    # Verdampfer: Luft 7°C
    evaporator=HeatExchangerInputs(T_in=273.15 + 7, m_flow=0.6),  # ca 0.5 m³/s
    # Kondensator: Wasser 30°C rein (für 35°C raus)
    condenser=HeatExchangerInputs(T_in=273.15 + 30, m_flow=0.2),  # Standard Fluss
    control=ControlInputs()
)

# Controls setzen
inputs.control.set(name="n", value=0.6, unit="Hz")  # 60% Drehzahl
inputs.control.set(name="T_ambient", value=280.15, unit="K")
inputs.control.set(name="phi", value=0.8, unit="-")
inputs.control.set(name="dT_eva_superheating", value=5.0, unit="K")
inputs.control.set(name="dT_con_subcooling", value=np.nan, unit="K")  # Dummy

# HPEV Öffnung: Wir testen 0.6
inputs.control.set(name="opening", value=0.6, unit="-")

# 3. AUSFÜHRUNG MIT FEHLERMELDUNG
print("Starte Simulation für A7/W35...")
solver = FSolve()

try:
    # Wir rufen solve() direkt auf, nicht calc_multiple_states
    # Das gibt uns sofortiges Feedback.
    heat_pump.solve(inputs, algorithm=solver)

    print("\n--- ERFOLG! ---")
    print(f"P_evap: {heat_pump.p_evaporator / 1e5:.2f} bar")
    print(f"P_cond: {heat_pump.p_condenser / 1e5:.2f} bar")
    print(f"COP: {heat_pump.COP:.2f}")
    print("Das System funktioniert mit A=2.0e-6!")

except Exception as e:
    print("\n--- FEHLER ---")
    print("Die Simulation ist abgestürzt. Hier ist der Grund:")
    print(e)
    # Manchmal hilft es, den Traceback zu sehen:
    import traceback

    traceback.print_exc()