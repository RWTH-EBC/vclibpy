# # Expansion Valve Example
# This example demonstrates how to use the classes `ExpansionValve`,
# and its children.

def main():
    # First, let's import an exemplary valve from vclibpy's
    # `expansion_valve` package. The only one currently implemented
    # is Bernoulli:
    from vclibpy.components.expansion_valves import Bernoulli
    # The valves are not that important for the vapor compression simulation,
    # as we iterate the pressure levels directly. However, you can still check
    # if a given valve cross-section area is large enough for your required level of
    # superheat.
    help(Bernoulli)
    # Let's specify some dummy parameters:
    d = 5e-3  # 5 mm diameter
    area = 3.14 * d ** 2 / 4
    expansion_valve = Bernoulli(A=area)
    # Again, we have to start a med-prop, and give some input state:
    from vclibpy.media import CoolProp
    med_prop = CoolProp(fluid_name="Propane")
    expansion_valve.med_prop = med_prop
    # Let's use the inlet state as in the last example
    # Also, we will use the evaporation pressure level:
    state_condenser_outlet = med_prop.calc_state("TQ", 273.15 + 40, 0)
    expansion_valve.state_inlet = state_condenser_outlet
    p_evaporation = 3.149034617014494 * 1e5
    # Now, we can calculate the outlet:
    expansion_valve.calc_outlet(p_outlet=p_evaporation)
    # Note that the outlet has the same enthalpy as the inlet:
    print(f"{expansion_valve.state_inlet.h=}; {expansion_valve.state_outlet.h=}")
    # ## Valve opening:
    # Let's assume we want to match the mass flow rate of the last example:
    # What opening would we require for the given cross section area?
    # For this, we can use expansion_valve.ca
    m_flow_ref_goal = 0.01
    opening = expansion_valve.calc_opening_at_m_flow(m_flow=m_flow_ref_goal)
    print(f"Required opening: {opening}")
    # Not that much. Now, we can repeat the process with different diameters in mm.
    import numpy as np
    openings = []
    d_mm_array = np.arange(0.5, 5, 0.5)
    for d_mm in d_mm_array:
        expansion_valve.A = 3.14 * (d_mm * 1e-3) ** 2 / 4
        opening = expansion_valve.calc_opening_at_m_flow(m_flow=m_flow_ref_goal)
        print(f"Required opening for area={expansion_valve.A}: {opening * 100} %")
        openings.append(opening * 100)
    # Let's plot the result:
    import matplotlib.pyplot as plt
    plt.plot(d_mm_array, openings, marker="s")
    plt.ylabel("Opening in %")
    plt.xlabel("$d$ in mm")
    plt.show()
    # Looking only at this point, the diameter should not be smaller than 1 mm.
    # You can tweak the assumptions around, check for different mass flow rates
    # or different pressure levels.


if __name__ == '__main__':
    main()