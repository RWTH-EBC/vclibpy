'''import CoolProp.CoolProp as CP

# --- Parameter definieren ---
fluid = 'R290'
T_celsius = 45
T_kelvin = T_celsius + 273.15

# Wir nehmen siedende Flüssigkeit an (Quality Q = 0).
# Alternativ könntest du über Druck (P) und Temperatur (T) gehen, falls es stark unterkühlt ist.
Q = 0

# --- Stoffwerte abrufen ---
# Dichte (rho) in kg/m^3
rho = CP.PropsSI('D', 'T', T_kelvin, 'Q', Q, fluid)

# Dynamische Viskosität (mu) in Pa*s
mu = CP.PropsSI('V', 'T', T_kelvin, 'Q', Q, fluid)

# Kinematische Viskosität (nu) in m^2/s (selbst berechnet aus mu / rho)
nu = mu / rho

# Wärmeleitfähigkeit (lambda) in W/(m*K)
lam = CP.PropsSI('L', 'T', T_kelvin, 'Q', Q, fluid)

# Prandtl-Zahl (Pr)
Pr = CP.PropsSI('PRANDTL', 'T', T_kelvin, 'Q', Q, fluid)

# --- Ausgabe ---
print(f"--- Stoffwerte für {fluid} bei {T_celsius} °C (Flüssig) ---")
print(f"Dichte (rho):           {rho:.2f} kg/m^3")
print(f"Kin. Viskosität (nu):   {nu:.3e} m^2/s")
print(f"Wärmeleitfähigkeit (L): {lam:.4f} W/(m*K)")
print(f"Prandtl-Zahl (Pr):      {Pr:.2f}")'''
import CoolProp.CoolProp as CP

# --- Parameter definieren ---
fluid = 'R290'
T_celsius = 74.8
T_kelvin = T_celsius + 273.15
p_bar = 29.50
p_pascal = p_bar * 100000

# Wir nehmen an dieser Stelle 100% Dampf an (Quality Q = 1)
Q = 0

# --- Stoffwerte abrufen ---
T = CP.PropsSI('T', 'P', p_pascal, 'Q', Q, fluid)

print(f"--- Stoffwerte für {fluid} bei {p_bar} bar (Hochdruckseite) ---")
print(f"Temperatur (T):           {T:.2f} K")
'''
p = CP.PropsSI('P', 'T', T_kelvin, 'Q', Q, fluid)
rho = CP.PropsSI('D', 'T', T_kelvin, 'Q', Q, fluid)
mu  = CP.PropsSI('V', 'T', T_kelvin, 'Q', Q, fluid)
nu  = mu / rho
lam = CP.PropsSI('L', 'T', T_kelvin, 'Q', Q, fluid)
Pr  = CP.PropsSI('PRANDTL', 'T', T_kelvin, 'Q', Q, fluid)

# --- Ausgabe ---
print(f"--- Stoffwerte für {fluid} bei {T_celsius} °C (Sauggas) ---")
print(f"Druck (p):           {p:.2f} Pa")
print(f"Dichte (rho):           {rho:.2f} kg/m^3")
print(f"Kin. Viskosität (nu):   {nu:.3e} m^2/s")
print(f"Wärmeleitfähigkeit (L): {lam:.4f} W/(m*K)")
print(f"Prandtl-Zahl (Pr):      {Pr:.2f}")
'''