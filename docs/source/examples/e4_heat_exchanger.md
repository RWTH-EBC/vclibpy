
# Heat Exchanger Example
This example demonstrates how to use the heat exchanger
classes.

Contrary to the first examples, the heat exchanger is
more complex, in the sense that the only current
models follows a moving boundary epsNTU approach.
Thus, you need to assume heat transfer correlations
for different regimes.
We will use the evaporator for this example. Let's check the doc:

```python
from vclibpy.components.heat_exchangers import MovingBoundaryNTUEvaporator
help(MovingBoundaryNTUEvaporator)
```

As you can see, we have to define a lot of parameters.
Let's model a simple air-to-refrigerant heat exchanger with
constant heat transfer correlations. The areas are not that important
for this example.
For heat transfer, you can import models from the `heat_transfer` package
inside the `heat_exchangers` package.
You will find all options in the documentation of VcLibPy

```python
from vclibpy.components.heat_exchangers.heat_transfer.constant import (
    ConstantHeatTransfer, ConstantTwoPhaseHeatTransfer
)
from vclibpy.components.heat_exchangers.heat_transfer.wall import WallTransfer
```

Now, we can instantiate the class:

```python
evaporator = MovingBoundaryNTUEvaporator(
    A=15,
    secondary_medium="air",
    flow_type="counter",
    ratio_outer_to_inner_area=10,
    two_phase_heat_transfer=ConstantTwoPhaseHeatTransfer(alpha=1000),
    gas_heat_transfer=ConstantHeatTransfer(alpha=1000),
    wall_heat_transfer=WallTransfer(lambda_=236, thickness=2e-3),
    liquid_heat_transfer=ConstantHeatTransfer(alpha=5000),
    secondary_heat_transfer=ConstantHeatTransfer(alpha=25)
)
```

## Heat exchanger iterations
To understand the heat exchanger functions in VcLibPy,
you have to understand how we iterate to solve the closed
cycle simulation.
In reality, the expansion valve would
adjust its opening until a certain degree of superheat
is met. While this may oscillate in dynamic operation,
we assume that the control is able to meet a constant
degree of superheat in steady state.
This means, we have to find the pressure level which
ensures this superheat. So instead of iterating
the valve opening, we iterate the evaporation pressure
directly. The same holds for the condenser and subcooling.
However, levels of subcooling are not directly controlled in
real devices, at least in the standard cycle.
The assumption of keeping the degree of subcooling as an input
could be changed in future work.

For iteration, heat exchangers have the function `calc`.
In order for it to work, you have to assign both inlet and
outlet state, as well as the mass flow rate. Note that,
as we assume the levels of superheat and subcooling, we
will always have these states in our iterations.
Further, the inputs need to contain the evaporator inlet temperature T_eva_in and
the evaporator mass flow rate (secondary side):

```python
from vclibpy import FlowsheetState, Inputs
fs_state = FlowsheetState()
from vclibpy.media import CoolProp
med_prop = CoolProp(fluid_name="Propane")
evaporator.med_prop = med_prop  # We have to set it, same as in the second example.
evaporator.m_flow = 0.01
```

Also, we have to start the secondary med-prop. This is done for you
in the calculations, but required to be an extra function to enable multi-processing:

```python
evaporator.start_secondary_med_prop()
```

Let's assume  a superheat of 10 K and a condenser subcooling of 0 K.
With an isenthalp expansion valve, the inlet and outlet are given.
Further, let's assume a condensation temperature of 40 °C and
an air temperature of 2 °C, corresponding to the typical heat pump point A2W25

```python
T_eva_in = 273.15 + 2
dT_eva_superheating = 10
dT_con_subcooling = 0
inputs = Inputs(
    T_eva_in=T_eva_in, m_flow_eva=0.47,
    dT_eva_superheating=dT_eva_superheating, dT_con_subcooling=dT_con_subcooling
)
```

Let's start with a simple assumption, no temperature difference
at the evaporator outlet (or inlet of air):

```python
p_evaporation = med_prop.calc_state("TQ", T_eva_in - dT_eva_superheating, 1).p
```

Calculate the condenser outlet and expansion valve outlet, thus evaporator inlet

```python
state_condenser_outlet = med_prop.calc_state("TQ", 273.15 + 40, 0)
evaporator.state_inlet = med_prop.calc_state("PH", p_evaporation, state_condenser_outlet.h)
T_evaporation = med_prop.calc_state("PQ", p_evaporation, 1).T
evaporator.state_outlet = med_prop.calc_state("PT", p_evaporation, T_evaporation + dT_eva_superheating)
print(evaporator.calc_Q_flow())
```

What do they mean?
They are used in the iterative logic of VcLibPy and indicate that
the heat exchanger is valid, if the error is smaller than a pre-defined
margin and dT_min is greater than 0.

```python
error, dT_min = evaporator.calc(inputs=inputs, fs_state=fs_state)
print(f"{error=}, {dT_min=}")
```

dT_min is, as assumed, close to zero. However, the error is very large.
The error is calculated as follows: `(Q_ntu / Q - 1) * 100`.
`Q` is the amount of heat required to be transported, `Q_ntu` is
the amount possible to transfer according to NTU method.
This means, a negative value of 100 means we could transport 2 times
less heat than we want to.
Thus, we have to iterate the pressure assumptions and lower it,
as we need a higher temperature difference to the air.
For this, we will use a loop:

```python
import numpy as np
p_evaporations = np.linspace(p_evaporation - 1e4, p_evaporation, 1000)
errors, dT_mins = [], []
for p_evaporation in p_evaporations:
    evaporator.state_inlet = med_prop.calc_state("PH", p_evaporation, state_condenser_outlet.h)
    T_evaporation = med_prop.calc_state("PQ", p_evaporation, 1).T
    evaporator.state_outlet = med_prop.calc_state("PT", p_evaporation, T_evaporation + dT_eva_superheating)
    error, dT_min = evaporator.calc(inputs=inputs, fs_state=fs_state)
    errors.append(error)
    dT_mins.append(dT_min)
```

The iteration of VcLibPy is largely based on this method, so nothing fancy.
If the error is positive, we step back to the old value and decrease the
step-size by a factor of 10. At the end, we iterate with a `min_iteration_step`,
which is, by default 1 Pa.

Let's plot the result:

```python
import matplotlib.pyplot as plt
fig, ax = plt.subplots(2, 1, sharex=True)
ax[0].plot(p_evaporations / 1e5, errors)
ax[0].set_ylabel("Error in %")
ax[1].plot(p_evaporations / 1e5, dT_mins)
ax[1].set_ylabel("$\Delta T$ in K")
ax[1].set_xlabel("$p_\mathrm{Eva}$ in bar")
plt.show()
```

## Implement your own iteration
As the result is still not good, let's implement this really basic iteration logic.
For iterating, we use a while-loop. Let's define a max-iteration
counter to avoid an infinite iteration.

```python
max_iterations = 100
n_iteration = 0
p_eva_next = p_evaporation  # Start with simple assumption
p_step = 10000  # Pa
min_step = 1  # Pa
min_error = 0.1  # 0.1 % maximal error
errors, dT_mins, p_evaporations = [], [], []  # Store for later plotting
while n_iteration < max_iterations:
    evaporator.state_inlet = med_prop.calc_state("PH", p_eva_next, state_condenser_outlet.h)
    T_eva = med_prop.calc_state("PQ", p_eva_next, 1).T
    evaporator.state_outlet = med_prop.calc_state("PT", p_eva_next, T_eva + inputs.get("dT_eva_superheating").value)
    error, dT_min = evaporator.calc(inputs=inputs, fs_state=fs_state)
    # Store for later plotting
    errors.append(error)
    dT_mins.append(dT_min)
    p_evaporations.append(p_eva_next / 1e5)
    n_iteration += 1
    if error < min_error:
        p_eva_next -= p_step
        continue
    elif error > min_error:
        p_eva_next += p_step  # Go back
        if p_step <= min_step:
            print("Error: Can't solve any more accurate with given size of min_step")
            break
        p_step /= 10
        continue
    else:
        print(f"Converged")
else:
    print("Did not converged in the given max_iterations.")
print(f"Solution: {error=}, {dT_min=}, p_evaporation={p_evaporations[-1]}. Took {n_iteration=}")
```

Again, let's plot the iteration:

```python
import matplotlib.pyplot as plt
fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(range(n_iteration), errors, marker="s")
ax[0].set_ylabel("Error in %")
ax[1].plot(range(n_iteration), dT_mins, marker="s")
ax[1].set_ylabel("$\Delta T$ in K")
ax[2].plot(range(n_iteration), p_evaporations, marker="s")
ax[2].set_ylabel("$p_\mathrm{Eva}$ in bar")
ax[2].set_xlabel("Iterations in -")
plt.show()
```

You can see that the iterative process converges to an error
Close to zero. However, it's not really an efficient optimization.
This could, and should, be optimized using optimization techniques.

## Pinch plotting
What also helps to understand the heat exchanger better is to
plot the states and secondary media. We can use the
`get_two_phase_limits` function of med_prop to quickly plot
those:

```python
plt.plot(
    med_prop.get_two_phase_limits("h") / 1000,
    med_prop.get_two_phase_limits("T") - 273.15, color="black"
)
state_vapor = med_prop.calc_state("PQ", evaporator.state_inlet.p, 1)  # define intermediate state
states_to_plot = [evaporator.state_inlet, state_vapor, evaporator.state_outlet]
plt.plot(
    [state.h / 1000 for state in states_to_plot],
    [state.T - 273.15 for state in states_to_plot],
    marker="s", color="red"
)
Q_flow = evaporator.calc_Q_flow()
T_eva_in = T_eva_in - 273.15
T_eva_out = T_eva_in - Q_flow / inputs.get("m_flow_eva").value / 1000
plt.plot(
    [evaporator.state_outlet.h / 1000, evaporator.state_inlet.h / 1000],
    [T_eva_in, T_eva_out],
    color="blue"
)
plt.ylabel("$T$ in °C")
plt.xlabel("$h$ in kJ/kgK")
plt.ylim([evaporator.state_inlet.T - 275.15, T_eva_in + 2])
plt.show()
```
