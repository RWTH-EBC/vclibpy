#Verdampferbilanz ohne IHX
'''1. Imports
2. Sensor-/Klemmenfunktionen
3. Betriebspunktdefinition
4. Verdampferbilanz
5. Beitragsanalyse
6. Ausgabe als Tabelle'''

from gum_ebc.Measurement import Measurement
from gum_ebc.Sensors.Pressure import Pressure
from gum_ebc.Terminals import EL3182

p_test_10 = Measurement(
    values=[5e5],
    sensor=Pressure(type="ICS_Schneider_IMP331-p10", full_scale=10e5),
    terminal=EL3182(full_scale=10e5)
)()

p_test_25 = Measurement(
    values=[5e5],
    sensor=Pressure(type="ICS_Schneider_IMP331-p25", full_scale=25e5),
    terminal=EL3182(full_scale=25e5)
)()

print("10 bar:", p_test_10)
print("25 bar:", p_test_25)

from gum_ebc.Measurement import Measurement
from gum_ebc.Sensors.Pressure import Pressure
from gum_ebc.Terminals.EL3154 import EL3154

p_test = Measurement(
    values=[5e5],
    sensor=Pressure(type="ICS_Schneider_IMP331-p10", full_scale=10 * 1e5),
    terminal=EL3154(full_scale=10 * 1e5)
)()

print(p_test)
print("x =", p_test.x)
print("u =", p_test.u)

