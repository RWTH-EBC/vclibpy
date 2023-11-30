# # Refrigerant Data Example
# This example demonstrates how to use the classes `MedProp` (and its children),
# `ThermodynamicState`, and `TransportProperties`.
# Further, basic plotting is shown using this data.

def main():
    # First, let's import the important classes from vclibpy's
    # `media` module:
    from vclibpy.media import CoolProp, ThermodynamicState, TransportProperties
    # We have two media property classes, `CoolProp` and `RefProp`.
    # The latter requires a dll, which you have to purchase together with RefProp.
    # Thus, in this example, we will use `CoolProp`. Pass the `fluid_name` to
    # select the fluid you are going to use.
    cool_prop = CoolProp(fluid_name="Propane")
    # ## ThermodynamicState calculation
    # Let's start and show how the media property classes work. You always
    # call `calc_state()`. The documentation explains how to use it:
    help(cool_prop.calc_state)
    # Let's try and start with pressure of 2 bar (2e5 Pa) and 100 kJ/kg enthalpy:
    state = cool_prop.calc_state("PH", 2e5, 100e3)
    # The state is an instance of `ThermodynamicState`:
    print(type(state))
    # The state contains all important specific values:
    print(state.get_pretty_print())
    # For these values, we are outside of the two phase region, as q (quality) is -1.
    # You can play around with the possible options to get a better understanding.
    # ## TransportProperties calculation
    # With a given state, we can calculate the transport properties. Those include
    # relevant information for component models, e.g. heat conductivity.
    # For information on all properties, look at the documentation:
    help(cool_prop.calc_transport_properties)
    # You just have to pass a valid state:
    transport_properties = cool_prop.calc_transport_properties(state=state)
    print(transport_properties.get_pretty_print())

    # ## Plotting
    # To plot fluid data, we may plot the two phase limits.
    # While we have the function `get_two_phase_limits` in the `media` model,
    # we will define it here again so that you can further learn how to use `media`.
    # The idea is to loop over all pressure from some minimum value.
    # Let's use the pressure at -40 °C.
    # to the maximum, which is the critical point.
    # You can get the critical point using the function: `get_critical_point`:
    p_min = cool_prop.calc_state("TQ", 273.15 - 40, 0).p  # Pa
    T_crit, p_crit, d_crit = cool_prop.get_critical_point()
    p_max = p_crit
    # Let's create two lists, q0 and q1 for states with quality of 0 and 1. Further,
    # we loop only each 10000 Pa to reduce number of function calls.
    p_step = 10000  # Pa
    q0 = []
    q1 = []
    for p in range(int(p_min), int(p_max), p_step):
        q0.append(cool_prop.calc_state("PQ", p, 0))
        q1.append(cool_prop.calc_state("PQ", p, 1))
    # Now, we can plot these states, for example in a T-h Diagram.
    # Note: [::-1] reverts the list, letting it start from the critical point.
    # `[state.T for state in q0]` is a list comprehension, quite useful in Python.
    T = [state.T for state in q0 + q1[::-1]]
    h = [state.h for state in q0 + q1[::-1]]
    import matplotlib.pyplot as plt
    plt.ylabel("$T$ in K")
    plt.xlabel("$h$ in J/kg")
    plt.plot(h, T, color="black")

    # Now, without any component models, let's try to plot a closed vapor compression cycle:
    # Assumptions:
    # - No superheat nor subcooling
    # - isobaric heat exchange
    # - isentropic compression and expansion
    # - 0 °C evaporation and 40 °C condensation temperature
    state_1 = cool_prop.calc_state("TQ", 273.15, 1)
    state_3 = cool_prop.calc_state("TQ", 273.15 + 40, 0)
    state_4 = cool_prop.calc_state("PH", state_1.p, state_3.h)
    state_2 = cool_prop.calc_state("PS", state_3.p, state_1.s)
    # Now, let's plot them with some markers:
    plot_lines_h = [state_1.h, state_2.h, state_3.h, state_4.h, state_1.h]
    plot_lines_t = [state_1.T, state_2.T, state_3.T, state_4.T, state_1.T]
    plt.plot(plot_lines_h, plot_lines_t, marker="s", color="red")
    plt.show()

    # Try to use the skills you've learned in this example and tweak the assumptions
    # and the plot format: Plot log(p)-h, T-s, or similar. Assume some level
    # of superheat, non-isentropic compression etc.

    # After getting familiar with calling the refrigerant data module `media`, you will
    # learn how to use the `Compressor` classes in the next example.


if __name__ == '__main__':
    main()
