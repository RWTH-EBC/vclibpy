"""Module to load specific real existing heat
pumps based on their parameter configuration"""
from pathlib import Path

import vclibpy.components.compressors.ten_coefficient
from vclibpy.components.heat_exchangers import HeatExchanger


class HPLoader_kbr_pwo:
    nominal_values = {
        "Propane_StandardFlowsheet":{
            #'Q_con': 5514.6429927053605,
            #'COP': 3.188828726670917,
            'm_flow_con': 1032*1.1644/3600, #Daten von RF-DL101F2H-035M060
            'm_flow_eva': 1000*1.2922/3600, #Daten von DV-1.1-400/355-7(+)-W48-P8-35G-D180/028-C5-211-1*16+1*16
            'delta_T_con': 8,
            'delta_T_eva': 5,
            'dT_eva_superheating': 6.1, #Daten von DV-1.1-400/355-7(+)-W48-P8-35G-D180/028-C5-211-1*16+1*16
            'dT_con_subcooling': 8}  #Daten von RF-DL101F2H-035M060
    }

    """
    "Propane_StandardFlowsheet": {
        'Q_con': 4407.787307356876,
        'COP': 2.1397221180625285,
        'm_flow_con': 0.13170689122288656,
        'm_flow_eva': 0.4698166163310231,
        'delta_T_con': 8,
        'delta_T_eva': 5,
        'dT_eva_superheating': 5,
        'dT_con_subcooling': 0}

    """

    eva_paras = {"A": 11.7, #Daten von DV-1.1-400/355-7(+)-W48-P8-35G-D180/028-C5-211-1*16+1*16
                 "wall": {"lambda": 236, "thickness": 1.8e-4},  #Daten von DV-1.1-400/355-7(+)-W48-P8-35G-D180/028-C5-211-1*16+1*16
                 "secondary_medium": "air",   #Daten von DV-1.1-400/355-7(+)-W48-P8-35G-D180/028-C5-211-1*16+1*16
                 "char_len_pri": 0,  # Without influence
                 "char_len_sec": 0,  # Without influence
                 "A_cross": 5,  # Without influence
                 "const_alpha_pri": 50000,
                 "const_alpha_sec": 5000
                 }

    con_paras = {"A": 6.1, #Daten von RF-DL101F2H-035M060
                 "wall": {"lambda": 236, "thickness": 2.1e-3}, #Daten von RF-DL101F2H-035M060
                 "secondary_medium": "water",  #Daten von RF-DL101F2H-035M060 -->eig water?
                 "char_len_pri": 0,  # Without influence
                 "char_len_sec": 0,  # Without influence
                 "A_cross": 5,  # Without influence
                 "const_alpha_pri": 50000,
                 "const_alpha_sec": 50000
                 }

    N_max = 120
    V_h = 20.9e-6
    HP_type = "AW"
    T_sh = 11.1  # [K]
    T_sc = 8.3  # [K]
    capacity_definition = "cooling"  # either "cooling" or "heating"
    assumed_eta_mech = 0.98  # only used if capacity_definition = "cooling"
    datasheet = Path(__file__).parent.joinpath("data", "coefficients_2.xlsx")
    com_type = vclibpy.components.compressors.ten_coefficient.TenCoefficientCompressor
    # com_type = compressors.DataSheetCompressor

    eva_type = HeatExchanger
    con_type = HeatExchanger
