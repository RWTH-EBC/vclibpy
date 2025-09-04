from vclibpy.media.oil_mix_prop import OilMixProp

fluid_name = "WATER"
lub_name = "R134A"

oilmix = OilMixProp(
    fluid_name=fluid_name,
    lub_name=lub_name,
    OilMixProp_path="D:\\OilMixProp-main" #todo: update
)
mode = "TP"
temperature = 300.0   # [K]
pressure = 800000.0      # [kPa]
lubricant_fraction = 0.2

state, props_ = oilmix.calc_state(mode, temperature, pressure, lubricant_fraction)

print("Thermodynamic State:")
print(f"Pressure [Pa]: {state.p}")
print(f"Temperature [K]: {state.T}")
print(f"Density [kg/m³]: {state.d}")
print(f"Specific Enthalpy [J/kg]: {state.h}")
print(f"Specific Entropy [J/kg/K]: {state.s}")
print(f"Internal Energy [J/kg]: {state.u}")
print(f"Vapor Quality [-]: {state.q}")

print("Transport Properties:")
print(f"Thermal conductivity [W/m/K]: {props_.lam}")
print(f"Dynamic viscosity [Pa·s]: {props_.dyn_vis}")
print(f"Kinematic viscosity [m²/s]: {props_.kin_vis}")
print(f"Prandtl number [-]: {props_.Pr}")
print(f"Specific heat cp [J/kg/K]: {props_.cp}")
print(f"Specific heat cv [J/kg/K]: {props_.cv}")
print(f"Thermal expansion coefficient beta [1/K]: {props_.beta}")
print(f"Surface tension [N/m]: {props_.sur_ten}")
print(f"Accommodation factor [-]: {props_.ace_fac}")