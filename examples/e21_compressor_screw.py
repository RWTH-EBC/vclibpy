# # Compressor Example
# This example demonstrates how to use the classes `Compressor`,
# and its children.

def main():
    # First, let's import an exemplary compressor from vclibpy's
    # `compressor` package:
    from vclibpy.components.compressors import ScrewCompressorSemiEmpirical
    from vclibpy.media import RefProp
    from vclibpy import FlowsheetState, Inputs
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

    screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state)


if __name__ == '__main__':
    main()
