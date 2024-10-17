import numpy as np
# # Compressor Example
# This example demonstrates how to use the classes `Compressor`,
# and its children.
from vclibpy.components.compressors import ScrewCompressorSemiEmpiricalBruteForce
from vclibpy.media import RefProp
from vclibpy import FlowsheetState, Inputs
from vclibpy.components.compressors import ScrewCompressorSemiEmpirical

import pandas as pd

screw_compressor = ScrewCompressorSemiEmpirical(
    N_max=4500/60,
    V_h=676.8e-6,
    eta_el=0.9
)
med_prop = RefProp(fluid_name="R134a")
screw_compressor.med_prop = med_prop
T_SH = 10   # Superheat in in K
fs_state = FlowsheetState()

screw_compressor.med_prop= med_prop
T_cond = 40 + 273.15  # in K
T_eva = -15 + 273.15  # in K
n = 3500/4500
inputs = Inputs(n=n)
# definition of inlet/outlet state
p_out = med_prop.calc_state("TQ", T_cond, 0).p
p_in = med_prop.calc_state("TQ", T_eva, 0).p
T_in = T_eva + T_SH
state_in = med_prop.calc_state("PT", p_in, T_in)
screw_compressor.state_inlet = state_in
# calculation of outlet state
output_state, P_mech, m_flow, err = screw_compressor.calc_compressor(p_outlet=p_out, inputs=inputs, fs_state=fs_state)

errors = []

T_con_range = np.arange(20, 75, 5)
T_eva_range = np.arange(-20, 25, 5)

for c in T_con_range:
    for e in T_eva_range:
        T_cond = c + 273.15  # in K
        T_eva = e + 273.15  # in K
        p_out = med_prop.calc_state("TQ", T_cond, 0).p
        p_in = med_prop.calc_state("TQ", T_eva, 0).p
        T_in = T_eva + T_SH
        state_in = med_prop.calc_state("PT", p_in, T_in)
        screw_compressor.state_inlet = state_in
        # calculation of outlet state
        output_state, P_mech, m_flow, err = screw_compressor.calc_compressor(p_outlet=p_out, inputs=inputs,
                                                                             fs_state=fs_state)
        errors.append(err)


print(max(errors))
print("Ende")
# calculate COP
#state_con_out = med_prop.calc_state("PQ", output_state.p, 0)

#COP = (output_state.h - state_con_out.h) * m_flow / P_mech


#def main():
    # First, let's import an exemplary compressor from vclibpy's
    # `compressor` package:
    #from vclibpy.components.compressors import ScrewCompressorSemiEmpiricalBruteForce
    #from vclibpy.media import RefProp
    #from vclibpy import FlowsheetState, Inputs
    #import pandas as pd

    #import matplotlib.pyplot as plt
    #import numpy as np
    ##Parametrisierung Verdichter
    #screw_compressor = ScrewCompressorSemiEmpiricalBruteForce(
    #    N_max=120,
    #    V_h=676.8e-6,
    #    eta_el=0.9
    #)
#
    #med_prop = RefProp(fluid_name="R134a")
    #screw_compressor.med_prop = med_prop
#
    ##Definition of inlet state
    #T_eva = 273.15 -20
    #p_inlet = med_prop.calc_state("TQ", T_eva, 1).p
    #T_inlet = T_eva + 10
    #screw_compressor.state_inlet = med_prop.calc_state("PT", p_inlet, T_inlet)
#
    #T_con = 273.15 + 25
    #p_outlet = med_prop.calc_state("TQ", T_con, 1).p
#
    #fs_state = FlowsheetState()
    #inputs = Inputs(n=0.5)
    #result = screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state)
    #Result = result
#
#
    #err_h_3 = []
    #err_m_flow = []
    #T_out = []
    #x = []
    #error_counter = 0
    #step_counter = 0
    #for i in range(1000):
    #    step_counter = step_counter + 1
    #    print("Step: ", step_counter)
    #    try:
    #        result = screw_compressor.calc_state_outlet(p_outlet=p_outlet, inputs=inputs, fs_state=fs_state)
    #        err_h_3.append(result[1])
    #        err_m_flow.append(result[2])
    #        T_out.append(result[3].T)
    #        x.append(result[0])
    #    except:
    #        err_h_3.append(0)
    #        err_m_flow.append(0)
    #        T_out.append(300)
    #        x.append([0,0,0])
    #        error_counter = error_counter + 1
    #        print(error_counter)
#
    #results = {'err_h_3': err_h_3, 'err_m_flow': err_m_flow, 'T_out': T_out, 'x': x}
    ##print(results)
    #df = pd.DataFrame(data=results)
    #import os
    #SAVE_PATH = r"D:\00_temp\screw_compressor"
    #os.makedirs(SAVE_PATH, exist_ok=True)
    #df.to_csv(r"D:\00_temp\screw_compressor\results8.csv")
#
#
#
#if _#_name__ == '__main__':
    #import csv
#
    ##os.chdir(SAVE_PATH)
#
    #main()
#