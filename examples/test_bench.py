from vclibpy.components.compressors import ConstantEffectivenessCompressor
from vclibpy.components.heat_exchangers.heat_transfer.simple import *
from vclibpy.components.heat_exchangers import MVB_Evaporator, GasCooler
from vclibpy.flowsheets import StandardCycleTC
from vclibpy.datamodels import Inputs

my_inputs = Inputs(
    fix_m_flow_con=False,
    fix_m_flow_eva=False,
    T_con_in= 273.15+20,
    T_con_out=273.5+60,
    n_rel=1,
    n=100,
    
)



