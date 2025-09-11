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
    # cycle may be inputs.
    # We separate the external inputs into `ControlInputs` and
    # `HeatExchangerInputs`.
    # Depending on the vapor compression cycle, you may have different
    # control options and may have multiple heat exchangers.
    # In most cases, each machine has some controls and has
    # an evaporator and condenser.
    # Thus, the `Inputs` class requires to specify
    # `control', 'evaporator', and 'condenser'.
    # However, if necessary, you could inherit `Inputs` and define
    # additional input requirements.
    from vclibpy import Inputs, RelativeCompressorSpeedControl, HeatExchangerInputs
    # ### HeatExchangerInputs
    # The heat exchanger inputs are necessary to define the secondary (outer) temperature
    # levels and to check if the heat can be transferred
    # given the inlet and outlet temperatures.
    #
    # During calculations, the equation
    # `Q = m_flow * cp * abs(T_out - T_in) = m_flow * cp * dT`
    # must be satisfied. For example, if you provide `T_in`, `T_out`, and `cp`, the code
    # will calculate `Q` and `dT`. If you provide `T_in`, `dT`, and `cp`, it will calculate
    # `T_out` and `Q`. `Q` and `cp` are automatically provided during cycle iteration.
    #
    # For the current calculations, you have to specify at least two other arguments (e.g., `T_in` and `T_out`).
    # You always have to specify either `T_in` or `T_out` to define
    # the absolute temperature and pressure levels.
    # The mass flow rate, temperature difference or other temperature presuppose each other.
    # Sometimes you have a mass flow rate at hand, sometimes you want to achieve a certain
    # temperature spread. Thus, all options are available.
    #
    # Moreover, you can add inputs (e.g. `T_ambient`), which may be required to perform calculations
    # of the heat exchangers. This concept is shown in the section "Adding new inputs".

    # #### Control inputs
    # The control inputs are strongly linked to the way the
    # cycle calculation works. For instance, most examples use
    # a `RelativeCompressorSpeedControl` class, controlling the
    # states via compressor speed `n`, evaporator superheat `dT_eva_superheating`
    # and condenser subcooling `dT_con_subcooling`.
    # Another example is `CondenserPowerControl`, which controls the states
    # according to a desired condenser heat flow rate `Q_con`. Here, the cycle
    # calculation has to iterate in a different manner to map the desired `Q_con`
    # to a compressor speed in order to calculate the compressor models.
    #
    # #### Adding new inputs
    # If you are required to add new inputs, you can in two ways.
    # For instance, let's say you want to control the pressure
    # level ratio, at which vapor is injected in the vapor-injection flowsheets.
    # Here, the flowsheets can act on the control-input `k_vapor_injection`.
    # You can set custom inputs like this to test out new implementations:
    control_inputs = RelativeCompressorSpeedControl(n=1)
    control_inputs.set(name="k_vapor_injection", value=1.05)
    print(control_inputs)
    # Optional arguments of the `set` function are unit and description.
    # You should pass those along with the name and value to make
    # your results easier to analyze, for others and your future self:
    control_inputs.set(
        name="k_vapor_injection", value=1.05,
        unit="-",
        description="Calculates the injection pressure level according to "
                    "k_vapor_injection * np.sqrt(p_1 * p_2)"
    )
    # The `set` function registers a Variable in the `inputs` object.
    # You can get different information types like this:
    print(f"{control_inputs.get_variables()=}")
    print(f"{control_inputs.get_variable_names()=}")
    print(f"{control_inputs.get(name='k_vapor_injection')=}")
    print(f"{control_inputs.items()=}")  # To loop over the variable dict.
    # If you decide a new flowsheet always requires this control option,
    # you can further extend any `ControlInputs` classes and add such an option:
    # To mimic the example above, we extend RelativeCompressorSpeedControl
    from vclibpy.datamodels import RelativeCompressorSpeedControl

    class VaporInjectionCompressorControl(RelativeCompressorSpeedControl):
        """
        Example on how to define custom controls as classes.
        """

        def __init__(
                self,
                k_vapor_injection: float = 1,
                **kwargs
        ):
            super().__init__(**kwargs)
            self.set(
                name="k_vapor_injection", value=k_vapor_injection,
                unit="-",
                description="Calculates the injection pressure level according to "
                            "k_vapor_injection * np.sqrt(p_1 * p_2)"
            )
    print(VaporInjectionCompressorControl(n=0.2, k_vapor_injection=0.9).get_variables())

    # ## FlowsheetState
    # The class `FlowsheetState` is essentially the same as `Inputs`.
    # The only difference is its use, which is for outputs of the vapor
    # compression cycle like COP, heat flow rate, temperatures, etc.
    # Basically, anything a users might be interested in analyzing when
    # simulating a steady state vapor compression cycle.
    # As the outputs are flowsheet specific, we define no default
    # Variables in the `FlowsheetState`.
    # However, you can set variables the same way as with `ControlInputs` or `HeatExchangerInputs`.
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
