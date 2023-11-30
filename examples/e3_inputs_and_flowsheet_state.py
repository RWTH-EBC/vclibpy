# # Inputs and FlowsheetState
# This example demonstrates how to use the classes `Inputs`,
# and `FlowsheetState`

def main():
    # All Variables in the `Inputs` and `FlowsheetState` will be saved in
    # output formats like .csv or .sdf
    # Thus, the concept of these two classes is important to
    # understand and analyze simulations in VcLibPy.

    # ## Inputs
    # All external boundary conditions which act on the vapor compression
    # cycle may be inputs. This could be the compressor speed, ambient temperature,
    # or the inlet temperatures and mass flow rates of the secondary side
    # in the heat exchangers.
    # You can see all default options by just printing the empty instance:
    from vclibpy import Inputs
    print(Inputs())
    # Currently, the flowsheets need all of these parameters, except for
    # the ambient temperature. This is only required for heat loss estimations or
    # efficiency calculations in certain models.
    # Handling all the inputs in one object makes it easy for component models
    # to access any relevant data it needs. Also, you can add new inputs
    # as you like. For instance, let's say you want to control the pressure
    # level ratio, at which vapor is injected in the vapor-injection flowsheets.
    # Here, the flowsheets can act on the input `k_vapor_injection`. The default is 1.
    # You can set custom inputs like this:
    inputs = Inputs()
    inputs.set(name="k_vapor_injection", value=1.05)
    print(inputs)
    # Optional arguments of the `set` function are unit and description.
    # You should pass those along with the name and value to make
    # your results easier to analyze, for others and your future self:
    inputs.set(
        name="k_vapor_injection", value=1.05,
        unit="-",
        description="Calculates the injection pressure level according to "
                    "k_vapor_injection * np.sqrt(p_1 * p_2)"
    )
    # The `set` function registers a Variable in the `inputs` object.
    # You can get different information types like this:
    print(f"{inputs.get_variables()=}")
    print(f"{inputs.get_variable_names()=}")
    print(f"{inputs.get(name='k_vapor_injection')=}")
    print(f"{inputs.items()=}")  # To loop over the variable dict.

    # ## FlowsheetState
    # The class `FlowsheetState` is essentially the same as `Inputs`.
    # The only difference is its use, which is for outputs of the vapor
    # compression cycle like COP, heat flow rate, temperatures, etc.
    # Basically, anything a users might be interested in analyzing when
    # simulating a steady state vapor compression cycle.
    # As the outputs are flowsheet specific, we define no default
    # Variables in the `FlowsheetState` as with `Inputs`.
    # However, you can set variables the same way as with `Inputs`:
    from vclibpy import FlowsheetState
    fs_state = FlowsheetState()
    print(fs_state)
    fs_state.set(name="some_interesting_output", value=3.14,
                 unit="-", description="This is just an example")
    print(fs_state)
    # As the fs_state is passed to all components, it's important to
    # use distinct names. If two components set a variable `T_1`, the latter
    # one will override the first one.
    # As the `fs_state` and `inputs` are mutable, no history is preserved.
    # If you want to, for example, plot the history of the `fs_state`,
    # you have to store copies of the instance:
    fs_states = []
    for some_value in range(10):
        fs_state.set(name="some_interesting_output", value=some_value)
        fs_states.append(fs_state.copy())
    print([fs_state.get("some_interesting_output").value for fs_state in fs_states])
    # Without the copy, it would not work:
    fs_states = []
    for some_value in range(10):
        fs_state.set(name="some_interesting_output", value=some_value)
        fs_states.append(fs_state)
    print([fs_state.get("some_interesting_output").value for fs_state in fs_states])


if __name__ == '__main__':
    main()
