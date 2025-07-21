
# Inputs and FlowsheetState
This example demonstrates how to use the classes `Inputs`,
and `FlowsheetState`

```python
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
```

## FlowsheetState
The class `FlowsheetState` is essentially the same as `Inputs`.
The only difference is its use, which is for outputs of the vapor
compression cycle like COP, heat flow rate, temperatures, etc.
Basically, anything a users might be interested in analyzing when
simulating a steady state vapor compression cycle.
As the outputs are flowsheet specific, we define no default
Variables in the `FlowsheetState`.
However, you can set variables the same way as with `ControlInputs` or `HeatExchangerInputs`.

```python
from vclibpy import FlowsheetState
fs_state = FlowsheetState()
print(fs_state)
fs_state.set(name="some_interesting_output", value=3.14,
             unit="-", description="This is just an example")
print(fs_state)
```

As the fs_state is passed to all components, it's important to
use distinct names. If two components set a variable `T_1`, the latter
one will override the first one.
As the `fs_state` and `inputs` are mutable, no history is preserved.
If you want to, for example, plot the history of the `fs_state`,
you have to store copies of the instance:

```python
fs_states = []
for some_value in range(10):
    fs_state.set(name="some_interesting_output", value=some_value)
    fs_states.append(fs_state.copy())
print([fs_state.get("some_interesting_output").value for fs_state in fs_states])
```

Without the copy, it would not work:

```python
fs_states = []
for some_value in range(10):
    fs_state.set(name="some_interesting_output", value=some_value)
    fs_states.append(fs_state)
print([fs_state.get("some_interesting_output").value for fs_state in fs_states])
```
