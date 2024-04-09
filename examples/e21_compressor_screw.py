# # Compressor Example
# This example demonstrates how to use the classes `Compressor`,
# and its children.

def main():
    # First, let's import an exemplary compressor from vclibpy's
    # `compressor` package:
    from vclibpy.components.compressors import ScrewCompressorSemiEmpirical

    # Check out the documentation to see relevant parameters:
    #help(ScrewCompressorSemiEmpirical)
    # Let's specify some dummy parameters:
    screw_compressor = ScrewCompressorSemiEmpirical(
        N_max=120,
        V_h=676.8e-6,
        eta_el=0.9
    )
    # Before we can do any calculations, the compressor needs
    # access to refrigerant data. Each component in VcLibPy has
    # the property med_prop (media property) which you can set like this:
    from vclibpy.media import RefProp
    med_prop = RefProp(fluid_name="R134a")
    screw_compressor.med_prop = med_prop
    # Now, you have to define the input state of the compressor.
    # Each component has inlet and outlet states, which are, same as `med_prop`
    # properties of the component.
    # We assume a super-heated vapor at 1bar as an input state:
    p_inlet = 1e5
    T_superheat = med_prop.calc_state("PQ", p_inlet, 1).T + 10
    screw_compressor.state_inlet = med_prop.calc_state("PT", p_inlet, T_superheat)
    # Last but not least, most functions in VcLibPy require
    # the argument `inputs` and `fs_state`. The whole concept of the two
    # classes are explained in the third example. For now, we just instantiate
    # the classes and pass a relative compressor speed of 50 % (0.5) as an input.
    from vclibpy import FlowsheetState, Inputs
    fs_state = FlowsheetState()
    inputs = Inputs(n=0.5)
    # Now, we can calculate multiple things.
    # ### Outlet state
    # While the constant efficiency compressor does not rely on any
    # states to calculate the constant efficiencies, most other models do.
    # Thus, we first want to calculate the outlet state for the given input state.
    # We can do so by passing an outlet pressure to the function `calc_state_outlet`:
    p_outlet = 6e5
    print(f"{screw_compressor.state_outlet=}")  # still None
    screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state)
    print(f"{screw_compressor.state_outlet=}")  # now calculated
    # Also, relevant results are automatically added to the `fs_state`:
    print(fs_state)

    # You can play around with the compressor speed (which has no effect due to constant efficiencies)
    # or the compression ratio. Let's do the latter for the outlet temperature:
    import numpy as np
    ratios = np.arange(2, 10, 0.5)
    T_outlets = []
    for ratio in ratios:
        screw_compressor.calc_state_outlet(
            p_outlet=p_inlet * ratio, inputs=inputs, fs_state=fs_state
        )
        T_outlets.append(screw_compressor.state_outlet.T)
    # Let's plot the results:
    import matplotlib.pyplot as plt
    plt.plot(ratios, np.array(T_outlets) - 273.15)
    plt.ylabel("$T_\mathrm{Outlet}$ in °C")
    plt.xlabel("$\Pi$ in -")
    plt.show()

    # ### Mass flow rate
    # Now, let's continue with the mass flow rate. Again, each component has the property
    # `m_flow`, which always refers to the refrigerant mass flow rate.
    # The function `calc_m_flow` calculates and set's the mass flow rate to this property.
    # So, after calling `calc_m_flow`, the return `m_flow` equals the compressors `m_flow`:
    m_flow = screw_compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
    print(f"{m_flow=}, {screw_compressor.m_flow=}")
    # Again, some interesting results are automatically added to the `fs_state`:
    print(fs_state)
    # Now, we can check how the compressor will work for different compressor speeds:
    m_flows = []
    speeds = np.arange(0, 1, 0.1)
    for speed in speeds:
        m_flows.append(screw_compressor.calc_m_flow(Inputs(n=speed), fs_state=fs_state))
    # Let's plot the results:
    import matplotlib.pyplot as plt
    plt.plot(speeds, m_flows)
    plt.ylabel("$\dot{m}$ in kg/s")
    plt.xlabel("$n$ in -")
    plt.show()

    # ### Electrical power consumption
    # If mass flow rates and outlet states are calculated, we can calculate the
    # electrical power consumption of the compressor. Note, that
    # if you change input values here, you first have to calculate the
    # mass flow rate and outlet state again, as this may influence the result.
    screw_compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
    screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state)
    P_el = screw_compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)
    print(f"{P_el=}")
    # Again, important metrics are added to the fs_state:
    print(fs_state)

    # After learning the basics of each component and using `Inputs` and `FlowsheetState`
    # for the first time, we will go deeper into these classes in the third example.
    # You can alter the compressor in use by importing other compressors, such as
    # `RotaryCompressor` or `TenCoefficientCompressor`. Check if you can use these components as well.



if __name__ == '__main__':
    main()
