# # Compressor Example
# This example demonstrates how to use the classes `Compressor`,
# and its children.

def main():
    # First, let's import an exemplary compressor from vclibpy's
    # `compressor` package:
    from vclibpy.components.compressors import ScrewCompressorSemiEmpiricalBruteForce
    from vclibpy.media import RefProp
    from vclibpy import FlowsheetState, Inputs
    import pandas as pd

    import matplotlib.pyplot as plt
    import numpy as np
    #Parametrisierung Verdichter
    screw_compressor = ScrewCompressorSemiEmpiricalBruteForce(
        N_max=120,
        V_h=676.8e-6,
        eta_el=0.9
    )

    med_prop = RefProp(fluid_name="R134a")
    screw_compressor.med_prop = med_prop

    #Definition of inlet state
    p_inlet = 2e5
    T_inlet = med_prop.calc_state("PQ", p_inlet, 1).T + 10
    screw_compressor.state_inlet = med_prop.calc_state("PT", p_inlet, T_inlet)

    fs_state = FlowsheetState()
    inputs = Inputs(n=0.5)

    p_outlet = 10e5
    err_h_3 = []
    err_Q_amb = []
    err_m_flow = []
    T_out = []
    x = []
    error_counter = 0
    step_counter = 0
    for i in range(3):
        step_counter = step_counter + 1
        print("Step: ", step_counter)
        try:
            result = screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state)
            err_h_3.append(result[1])
            err_m_flow.append(result[3])
            err_Q_amb.append(result[2])
            T_out.append(result[4].T)
            x.append(result[0])
        except:
            err_h_3.append(0)
            err_m_flow.append(0)
            err_Q_amb.append(0)
            T_out.append(300)
            x.append([0,0,0])
            error_counter = error_counter + 1
            print(error_counter)

    results = {'err_h_3': err_h_3,'err_Q_amb': err_Q_amb,'err_m_flow': err_m_flow, 'T_out': T_out, 'x': x}
    #print(results)
    df = pd.DataFrame(data=results)
    import os
    SAVE_PATH = r"D:\00_temp\screw_compressor"
    os.makedirs(SAVE_PATH, exist_ok=True)
    df.to_csv(r"D:\00_temp\screw_compressor\results6.csv")



if __name__ == '__main__':
    import csv

    #os.chdir(SAVE_PATH)

    main()
