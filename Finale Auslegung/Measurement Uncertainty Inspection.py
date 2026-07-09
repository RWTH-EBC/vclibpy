import inspect
from gum_ebc.configuration import PyfluidsEnthalpy

print("PyfluidsEnthalpy liegt hier:")
print(inspect.getsourcefile(PyfluidsEnthalpy))

print("\nSignatur:")
print(inspect.signature(PyfluidsEnthalpy.__init__))

print("\nQuellcode:")
print(inspect.getsource(PyfluidsEnthalpy))

import gum_ebc
from pathlib import Path
import re


package_path = Path(gum_ebc.__file__).parent

print("Package-Pfad:")
print(package_path)

pattern = re.compile(r'["\']([A-Za-z0-9_\-./]+)["\']')

interesting_files = list(package_path.rglob("*.py"))

found = set()

for file in interesting_files:
    text = file.read_text(encoding="utf-8", errors="ignore")

    for match in pattern.findall(text):
        if any(keyword in match.lower() for keyword in [
            "pt100",
            "krohne",
            "opti",
            "schneider",
            "sensirion",
            "dummy",
            "dn",
            "el3",
            "ics"
        ]):
            found.add((match, file.name))

for value, filename in sorted(found):
    print(f"{value:40s}  aus {filename}")

    import inspect
    from gum_ebc.Sensors import Pressure, Temperature, Krohne_OptiMass
    from gum_ebc.Terminals import EL3182, EL3202_0010


    def inspect_class(cls):
        print("\n" + "=" * 100)
        print("Klasse:", cls.__name__)
        print("Datei:", inspect.getsourcefile(cls))

        print("\nSignatur:")
        try:
            print(inspect.signature(cls.__init__))
        except Exception as e:
            print("Signatur nicht lesbar:", e)

        print("\nQuellcode:")
        try:
            print(inspect.getsource(cls))
        except Exception as e:
            print("Quellcode nicht lesbar:", e)


    inspect_class(Pressure)
    inspect_class(Temperature)
    inspect_class(Krohne_OptiMass)
    inspect_class(EL3182)
    inspect_class(EL3202_0010)