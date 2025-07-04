# # Example for a heat pump where you can select the flowsheet
from vclibpy.flowsheets import StandardCycle, VaporInjectionPhaseSeparator, VaporInjectionEconomizer, InternalHeatExchangerCycle, IHX
from vclibpy.components.heat_exchangers import moving_boundary_ntu, heat_transfer,moving_boundary_lmtd
from vclibpy.components.heat_exchangers.economizer import VaporInjectionEconomizerNTU
from vclibpy.components.heat_exchangers.ihx_ntu import IHX_NTU
from vclibpy.components.expansion_valves import Bernoulli
from vclibpy.components.compressors import ConstantEffectivenessCompressor,RotaryCompressor, TenCoefficientCompressor
from vclibpy import utils
import os
import datetime

def create_heat_exchanger(model: str, hx_type: str, **kwargs):
    """
    Factory function for moving boundary heat exchanger.

    Args:
        model (str): "NTU" or "LMTD"
        hx_type (str): "condenser", "evaporator", "ihx", "economizer"
        kwargs: arguments for the designated class

    Returns:
        object of the specified heat exchanger.
    """
    if model == "NTU":
        if hx_type == "condenser":
            return moving_boundary_ntu.MovingBoundaryNTUCondenser(**kwargs)
        elif hx_type == "evaporator":
            return moving_boundary_ntu.MovingBoundaryNTUEvaporator(**kwargs)
        elif hx_type == "ihx":
            return IHX_NTU(**kwargs)
        elif hx_type == "economizer":
            return VaporInjectionEconomizerNTU(**kwargs)
    elif model == "LMTD":
        if hx_type == "condenser":
            return moving_boundary_lmtd.MovingBoundaryLMTDCondenser(**kwargs)
        elif hx_type == "evaporator":
            return moving_boundary_lmtd.MovingBoundaryLMTDEvaporator(**kwargs)
        elif hx_type == "ihx":
            return moving_boundary_lmtd.MovingBoundaryLMTDIHX(**kwargs) # TODO: Add IHX LMTD implementation, current version is from MA Jonas Matthies
        # TODO: Add economizer LMTD implementation
    else:
        raise ValueError(f"Unknown heat exchanger model: {model}")

def calculate_compressor_volumes(V_h, V_h_ratio):
    """
    Calculates the high- and low-pressure compressor volumes based on the total volume and the volume ratio.

    Args:
        V_h (float): Total compressor volume.
        V_h_ratio (float): Ratio between high- and low-pressure compressor volume (V_h_high / V_h_low).

    Returns:
        tuple: A tuple containing the low-pressure volume (V_h_low) and the high-pressure volume (V_h_high).
    """
    V_h_low = (V_h / (1 + V_h_ratio))
    V_h_high = V_h_low * V_h_ratio
    return V_h_low, V_h_high

def create_compressor(compressor_type, compressor_params):
    """
    Creates a compressor object based on the specified type and parameters.

    Args:
        compressor_type (str): The type of compressor to create
        ("RotaryCompressor", "ConstantEffectivenessCompressor", "TenCoefficientCompressor").
        compressor_params (dict): A dictionary of parameters for the compressor.

    Returns:
        object: An instance of the specified compressor type.
    """
    if compressor_type == "RotaryCompressor":
        return RotaryCompressor(
            N_max=compressor_params['N_max'],
            V_h=compressor_params['V_h']
        )
    elif compressor_type == "ConstantEffectivenessCompressor":
        return ConstantEffectivenessCompressor(
            N_max=compressor_params['N_max'],
            V_h=compressor_params['V_h'],
            eta_isentropic=compressor_params['eta_isentropic'],
            lambda_h=compressor_params['lambda_h'],
            eta_mech=compressor_params['eta_mech']
        )
    elif compressor_type == "TenCoefficientCompressor":
        return TenCoefficientCompressor(
            N_max=compressor_params['N_max'],
            V_h=compressor_params['V_h'],
            datasheet=compressor_params['datasheet'],
            parameter_names=compressor_params['parameter_names'],
            sheet_name=compressor_params['sheet_name']
        )
    else:
        raise ValueError("ERROR when selecting compressor. Unsupported compressor selected.")

def create_flowsheet(flowsheet_type, common_params, vip_params=None, vie_params=None):
    """
    Create the selected flowsheet-object based on type.

    Args:
        flowsheet_type (str): flowsheet type
        ("StandardCycle", "VaporInjectionPhaseSeparator", "VaporInjectionEconomizer", "IHX").
        common_params (dict): common parameters for all flowsheets (e.g. evaporator, condenser, fluid).
        vip_params (dict, optional): parameter specific for VaporInjectionPhaseSeparator.
        vie_params (dict, optional): parameter specific for VaporInjectionEconomizer.

    Returns:
        object: instance of flowsheet-object.
    """
    A_valve = common_params['A_valve']
    V_h_ratio = common_params.get('V_h_ratio', 1)  # default, if not specified

    if flowsheet_type in ["VaporInjectionPhaseSeparator", "VaporInjectionEconomizer"]:
        V_h = common_params['compressor_params']['V_h']
        V_h_low, V_h_high = calculate_compressor_volumes(V_h, V_h_ratio)

        compressor_params_low = common_params["compressor_params"].copy()
        compressor_params_low["V_h"] = V_h_low
        low_pressure_compressor = create_compressor(common_params["compressor_type"], compressor_params_low)

        compressor_params_high = common_params["compressor_params"].copy()
        compressor_params_high["V_h"] = V_h_high
        high_pressure_compressor = create_compressor(common_params["compressor_type"], compressor_params_high)

        high_pressure_valve = Bernoulli(A=A_valve)
        low_pressure_valve = Bernoulli(A=A_valve)

    if flowsheet_type == "StandardCycle":
        compressor_params = common_params["compressor_params"]
        compressor = create_compressor(common_params["compressor_type"], compressor_params)
        pressure_valve = Bernoulli(A=A_valve)
        return StandardCycle(
            evaporator=common_params['evaporator'],
            condenser=common_params['condenser'],
            fluid=common_params['fluid'],
            compressor=compressor,
            expansion_valve=pressure_valve
        )

    if flowsheet_type == "InternalHeatExchanger":
        compressor_params = common_params["compressor_params"]
        compressor = create_compressor(common_params["compressor_type"], compressor_params)

        # create main pressure valve
        pressure_valve = Bernoulli(A=common_params["A_valve"])

        # Always create the second pressure valve for the IHX to avoid passing None.
        # The internal logic of the cycle seems to require a valid object here.
        A_valve_ihx = common_params.get('A_valve_ihx', common_params['A_valve'])
        pressure_valve_ihx = Bernoulli(A=A_valve_ihx)

        return IHX(
            evaporator=common_params['evaporator'],
            condenser=common_params['condenser'],
            ihx=common_params['ihx'],
            fluid=common_params['fluid'],
            compressor=compressor,
            expansion_valve=pressure_valve,
            expansion_valveIHX=pressure_valve_ihx,
        )

    elif flowsheet_type == "VaporInjectionPhaseSeparator":
        return VaporInjectionPhaseSeparator(
            evaporator=common_params['evaporator'],
            condenser=common_params['condenser'],
            fluid=common_params['fluid'],
            high_pressure_compressor=high_pressure_compressor,
            low_pressure_compressor=low_pressure_compressor,
            high_pressure_valve=high_pressure_valve,
            low_pressure_valve=low_pressure_valve
        )
    elif flowsheet_type == "VaporInjectionEconomizer":
        return VaporInjectionEconomizer(
            evaporator=common_params['evaporator'],
            condenser=common_params['condenser'],
            fluid=common_params['fluid'],
            economizer=common_params['economizer'], # args-difference to VaporInjectionPhaseSeparator
            high_pressure_compressor=high_pressure_compressor,
            low_pressure_compressor=low_pressure_compressor,
            high_pressure_valve=high_pressure_valve,
            low_pressure_valve=low_pressure_valve
        )
    else:
        raise ValueError("ERROR when selecting flowsheet. Unsupported flowsheet selected.")

def main():
    # 1. choose heat exchanger model from:
        #"NTU",
        #"LMTD"                 # TODO: LMTD does not work since merge from development
    hx_model = "NTU"

    # 2. define heat exchanger parameters
    condenser = create_heat_exchanger(
        model=hx_model,
        hx_type="condenser",
        A=3,
        secondary_medium="water",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=250),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=2400),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=4500)
    )

    evaporator = create_heat_exchanger(
        model=hx_model,
        hx_type="evaporator",
        A=5,
        secondary_medium="air",
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=150),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
        secondary_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=100)
    )

    economizer = create_heat_exchanger(
        model=hx_model,
        hx_type="economizer",
        A=0.2,
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=200),
        two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000),
        liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
        secondary_medium=None,  # No secondary medium for economizer
        secondary_heat_transfer=None,
    )

    ihx = create_heat_exchanger(
        model=hx_model,
        hx_type="ihx",
        A=0.2,
        flow_type="counter",
        ratio_outer_to_inner_area=1,
        alpha_low_side=150, #gaseous
        alpha_high_side=1500, #liquid
        # gas_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=150),
        # two_phase_heat_transfer=heat_transfer.constant.ConstantTwoPhaseHeatTransfer(alpha=3000), #cold side always vapor
        # liquid_heat_transfer=heat_transfer.constant.ConstantHeatTransfer(alpha=1500),
        wall_heat_transfer=heat_transfer.wall.WallTransfer(lambda_=20, thickness=0.6e-3),
        # secondary_medium=None,  # No secondary medium for IHX
        # secondary_heat_transfer=None,

    )

    # 3. define common parameters for the flowsheet
    common_params = {
        'evaporator': evaporator,
        'condenser': condenser,
        'fluid': "Propane",  # Refrigerant selection
        'economizer': economizer,
        'ihx': ihx,
        'A_valve': 0.1,  # TODO: Maybe distinction between high- and low-pressure valve?
        # ---- parameters for IHX Valves ----
        'two_ev_ihx': False,  # True means two valves, auf False just one valve
        'A_valve_ihx': 0.1,  # Separate Größe für das zweite Ventil (optional)
        # ---- parameters for compressor ----
        'V_h_ratio': 1,  # Ratio between high-and low-pressure compressor volume (V_h_ratio = V_h_high / V_h_low)
        # Compressor Type selection between:
            # ConstantEffectivenessCompressor
            # RotaryCompressor
            # TenCoefficientCompressor
        'compressor_type': "ConstantEffectivenessCompressor",
        'compressor_params': {
            # General parameters
            'N_max': 120, # Maximal rotations per second of the compressor.
            'V_h': 30e-6, # Volume of the compressor in m^3.
            # ConstantEffectivenessCompressor parameters
            'eta_isentropic': 0.7, # Constant isentropic efficiency of the compressor.
            'lambda_h': 0.9, # Constant volumetric efficiency.
            'eta_mech': 0.8,  # Constant mechanical efficiency of the compressor.
            # TenCoefficientCompressor parameters
            'datasheet': "path/to/your/datasheet.csv", # Path of the datasheet file for ten-coefficient compressor.
            'parameter_names': { # Dictionary to match internal parameter names (keys) to the names used in the table values.
                "m_flow": "Flow Rate(kg/h)",
                "capacity": "Capacity(W)",
                "input_power": "Input Power(W)",
                "eta_s": "Isentropic Efficiency(-)",
                "lambda_h": "Volumentric Efficiency(-)",
                "eta_mech": "Mechanical Efficiency(-)"
            },
            'sheet_name': "Sheet1" # Name of the sheet in the datasheet.
        }
    }

    # 4. choose flowsheet from:
        # StandardCycle
        # VaporInjectionEconomizer      TODO: Distinction between up- and down-stream economizer implementation
        # VaporInjectionPhaseSeparator
        # InternalHeatExchanger TODO: Implementation pending
        # DirectInjection       TODO: Implementation pending
    flowsheet_type = "StandardCycle"

    # 5. create flowsheet object
    flowsheet = create_flowsheet(flowsheet_type, common_params)


    # 6. generate performance map (Study settings)
    base_output_dir = r"D:\00_temp\flowsheet_selection"

    flowsheet_output_dir = os.path.join(base_output_dir, flowsheet_type)

    timestamp =datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    run_save_path = os.path.join(flowsheet_output_dir, f"{flowsheet_type}_{timestamp}")

    os.makedirs(run_save_path, exist_ok=True)
    print(f"Results will be saved in: {run_save_path}")

    T_eva_in = [-20 + 273.15, 12 + 273.15]
    T_con = [35 + 273.15, 75 + 273.15]                # inlet/outlet temperature depends on use_condenser_inlet setting
    n = [1]
    k_vapor_injection = [1.3, 1]

    utils.full_factorial_map_generation(
        flowsheet=flowsheet,
        save_path=run_save_path,
        T_con=T_con,
        T_eva_in=T_eva_in,
        n=n,
        use_condenser_inlet=False,
        use_multiprocessing=False,
        save_plots=True,
        m_flow_con=0.75,
        m_flow_eva=2.7,
        dT_eva_superheating=5,
        dT_con_subcooling=3,
        k_vapor_injection=k_vapor_injection,
    )

if __name__ == "__main__":
    main()