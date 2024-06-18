# # Compressor Example
# This example demonstrates how to use the classes `Compressor`,
# and its children.

def main():
    # First, let's import an exemplary compressor from vclibpy's
    # `compressor` package:
    from vclibpy.components.compressors import ScrewCompressorSemiEmpirical
    from vclibpy.media import RefProp
    from vclibpy import FlowsheetState, Inputs
    import matplotlib.pyplot as plt
    import numpy as np
    #Parametrisierung Verdichter
    screw_compressor = ScrewCompressorSemiEmpirical(
        N_max=120,
        V_h=676.8e-6,
        eta_el=0.9
    )

    med_prop = RefProp(fluid_name="R134a")
    screw_compressor.med_prop = med_prop

    #Definition of inlet state
    p_inlet = 1e5
    T_inlet = med_prop.calc_state("PQ", p_inlet, 1).T + 10
    screw_compressor.state_inlet = med_prop.calc_state("PT", p_inlet, T_inlet)

    fs_state = FlowsheetState()
    inputs = Inputs(n=0.5)

    p_outlet = 6e5
    n = 10
    T_out = np.zeros(shape=(n, n, n))
    iterations = np.linspace(0, 1, n)
    for i in range(n):
        for j in range(n):
            for k in range(n):

                fs_state.set(name="x0i", value=i/n)
                fs_state.set(name="x0j", value=j/n)
                fs_state.set(name="x0k", value=k/n)
                try:
                    T_out[i, j, k] = screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state).T
                except:
                    T_out[i, j, k] = 0
    x=1
    #plt.plot(iterations, T_out)
    #plt.show()
#




if __name__ == '__main__':
    main()
