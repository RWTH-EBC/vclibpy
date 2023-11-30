import time
import logging

from vclibpy import Inputs
from vclibpy.flowsheets import BaseCycle

logger = logging.getLogger(__name__)


def nominal_hp_design(
        heat_pump: BaseCycle, 
        inputs: Inputs, 
        fluid: str,
        dT_con: float = None,
        dT_eva: float = None,
        **kwargs) -> dict:
    """
    Function to calculate the heat pump design
    at a given nominal point. 
    Args:
        heat_pump (BaseCycle): A supported flowsheet
        inputs (Inputs):
            The input values at the nominal point.
            If the mass flow rates are not given, you
            can use dT_con and dT_eva to iteratively calculate
            the mass flow rates in order to achieve the required
            temperature differences at the nominal point.
        dT_con (float):
            Condenser temperature difference to calculate mass flow rate
        dT_eva (float):
            Evaporator temperature difference to calculate mass flow rate
        fluid (str): Fluid to be used.
        **kwargs: 
            m_flow_eva_start: Guess start-value for iteration. Default 0.2
            m_flow_con_start: Guess start-value for iteration. Default 1
            accuracy: Minimal accuracy for mass flow rate iteration (in kg/s). 
                Default 0.001 kg/s
             
    Returns:
        dict: A dictionary with all flowsheet states and inputs containing
              information about the nominal design.
    """
    t0 = time.time()
    # Define nominal values:
    m_flow_con_start = kwargs.get("m_flow_con_start", 0.2)
    m_flow_eva_start = kwargs.get("m_flow_eva_start", 1)
    accuracy = kwargs.get("accuracy", 0.001)
    calculate_m_flow = dT_eva is not None and dT_con is not None
    if calculate_m_flow:
        # Get nominal value:
        fs_state = heat_pump.calc_steady_state(fluid=fluid, inputs=inputs)
        if fs_state is None:
            raise ValueError("Given configuration is infeasible at nominal point.")

    else:
        # We have to iterate to match the m_flows to the Q_cons:
        m_flow_eva_next = m_flow_eva_start
        m_flow_con_next = m_flow_con_start
        while True:
            # Set values
            m_flow_eva = m_flow_eva_next
            m_flow_con = m_flow_con_next
            inputs.set("m_flow_con", m_flow_con)
            inputs.set("m_flow_eva", m_flow_eva)
            # Get nominal value:
            fs_state = heat_pump.calc_steady_state(fluid=fluid, inputs=inputs)
            if fs_state is None:
                raise ValueError("Given configuration is infeasible at nominal point.")
            cp_eva = heat_pump.evaporator._secondary_cp
            cp_con = heat_pump.condenser._secondary_cp
            m_flow_con_next = fs_state.get("Q_con") / (dT_con * cp_con)
            m_flow_eva_next = (fs_state.get("Q_con") * (1 - 1 / fs_state.get("COP"))) / (dT_eva * cp_eva)
            # Check convergence:
            if abs(m_flow_eva_next - m_flow_eva) < accuracy and abs(m_flow_con-m_flow_con_next) < accuracy:
                break

    nominal_design_info = {
        **inputs.convert_to_str_value_format(with_unit_and_description=False),
        **fs_state.convert_to_str_value_format(with_unit_and_description=False),
        "dT_con": dT_con,
        "dT_eva": dT_eva
    }
    logger.info("Auto-generation of nominal values took %s seconds", time.time()-t0)
    logger.info('Nominal values: %s', nominal_design_info)

    return nominal_design_info
