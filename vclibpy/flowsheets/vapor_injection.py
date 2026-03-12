import abc
import logging
from copy import deepcopy
import numpy as np

from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import Inputs, FlowsheetState
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.media import ThermodynamicState

logger = logging.getLogger(__name__)


class BaseVaporInjection(BaseCycle, abc.ABC):
    """
    Partial cycle with vapor injection, using
    two separated compressors and expansion valves.

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "VaporInjectionPhaseSeparator"

    def __init__(
            self,
            high_pressure_compressor: Compressor,
            low_pressure_compressor: Compressor,
            high_pressure_valve: ExpansionValve,
            low_pressure_valve: ExpansionValve,
            global_compressor_model: Compressor = None,
            **kwargs):
        super().__init__(**kwargs)
        self.high_pressure_compressor = high_pressure_compressor
        self.low_pressure_compressor = low_pressure_compressor
        self.high_pressure_valve = high_pressure_valve
        self.low_pressure_valve = low_pressure_valve
        self.global_compressor_model = global_compressor_model
        # Avoid nasty bugs for setting states
        if id(high_pressure_compressor) == id(low_pressure_compressor):
            self.high_pressure_compressor = deepcopy(low_pressure_compressor)
        if id(low_pressure_valve) == id(high_pressure_valve):
            self.high_pressure_valve = deepcopy(low_pressure_valve)

    def get_all_components(self):
        components = super().get_all_components() + [
            self.high_pressure_compressor,
            self.low_pressure_compressor,
            self.high_pressure_valve,
            self.low_pressure_valve,
        ]
        if self.global_compressor_model is not None:
            components.append(self.global_compressor_model)
        return components

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        k_vapor_injection_var = inputs.control.get("k_vapor_injection")

        # Extract the numerical value. If the variable is not present, use a default.
        # Default is 1 according to Xu, 2019
        if k_vapor_injection_var is not None:
            k_vapor_injection = k_vapor_injection_var.value
        else:
            k_vapor_injection = 1.0

        p_vapor_injection = k_vapor_injection * np.sqrt(p_1 * p_2) # TODO: are there other ways to set the mid pressure level?

        # Condenser outlet
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        # High pressure EV
        self.high_pressure_valve.state_inlet = self.condenser.state_outlet
        self.high_pressure_valve.calc_outlet(p_outlet=p_vapor_injection)

        # Calculate low compressor stage to already have access to the mass flow rates.
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        
        # INTERCEPT TO APPLY GLOBAL EFFICIENCIES
        if self.global_compressor_model is not None:
             # Point the global model at the total compression (from evaporator to condenser)
            self.global_compressor_model.state_inlet = self.evaporator.state_outlet
            
            # Create a hollow state purely to unblock empirical equations that check `p_outlet`
            self.global_compressor_model.state_outlet = ThermodynamicState(p=p_2)
            
            # Retrieve the correct efficiencies based on global pressure lift
            global_eta_is = self.global_compressor_model.get_eta_isentropic(p_outlet=p_2, inputs=inputs)
            global_eta_mech = self.global_compressor_model.get_eta_mech(inputs=inputs)
            global_lambda_h = self.global_compressor_model.get_lambda_h(inputs=inputs)
            
            # Overwrite the dummy constants on the ConstantEffectiveness models
            self.low_pressure_compressor.eta_isentropic = global_eta_is
            self.high_pressure_compressor.eta_isentropic = global_eta_is
            
            self.low_pressure_compressor.lambda_h = global_lambda_h
            self.high_pressure_compressor.lambda_h = global_lambda_h # does not do anything but for consistency and to avoid confusion
            
            self.low_pressure_compressor.eta_mech = global_eta_mech # only if the global model has a mechanical efficiency defined, otherwise it will be 1 as set in the constructor of the PiCorrelationCompressor
            self.high_pressure_compressor.eta_mech = global_eta_mech
            
            # Align physical traits just in case
            self.low_pressure_compressor.V_h = self.global_compressor_model.V_h
            self.high_pressure_compressor.V_h = self.global_compressor_model.V_h # does not do anything but for consistency and to avoid confusion
            self.low_pressure_compressor.N_max = self.global_compressor_model.N_max
            self.high_pressure_compressor.N_max = self.global_compressor_model.N_max
            
        self.low_pressure_compressor.state_inlet = self.evaporator.state_outlet
        self.low_pressure_compressor.calc_state_outlet(
            p_outlet=p_vapor_injection, inputs=inputs, fs_state=fs_state
        )

        # Calculate mass flow rate of lower stage based on the inlet state of the low pressure compressor
        m_flow_low = self.low_pressure_compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.evaporator.m_flow = self.low_pressure_compressor.m_flow

        # Injection component:
        x_vapor_injection, h_vapor_injection, state_low_ev_inlet = self.calc_injection()

        # Low pressure EV
        self.low_pressure_valve.state_inlet = state_low_ev_inlet
        self.low_pressure_valve.calc_outlet(p_outlet=p_1)
        # Evaporator
        self.evaporator.state_inlet = self.low_pressure_valve.state_outlet

        # Ideal Mixing of state_5 and state_1_VI:
        h_1_VI_mixed = (
                (1-x_vapor_injection) * self.low_pressure_compressor.state_outlet.h +
                x_vapor_injection * h_vapor_injection
        )
        self.high_pressure_compressor.state_inlet = self.med_prop.calc_state(
            "PH", p_vapor_injection, h_1_VI_mixed
        )
        self.high_pressure_compressor.calc_state_outlet(
            p_outlet=p_2, inputs=inputs, fs_state=fs_state
        )

        # Data continuity: apply the real physical outlet state to the global compressor
        if self.global_compressor_model is not None:
            self.global_compressor_model.state_outlet = self.high_pressure_compressor.state_outlet

        # Force mass conservation: calculate high stage mass flow based on low stage mass flow and the injected vapor amount.
        # The actual scroll compressor swept volume dictates the low stage flow. The high stage flow is simply the sum.
        m_flow_high_required = m_flow_low / (1 - x_vapor_injection)
        
        # Set the mass flow of the high-pressure compressor model directly
        self.high_pressure_compressor.m_flow = m_flow_high_required

        # Log these variables to the FlowsheetState so they appear in the result CSV
        m_flow_injection = m_flow_high_required - m_flow_low
        
        fs_state.set(
            name="m_flow_ref_injection", 
            value=m_flow_injection, 
            unit="kg/s", 
            description="Refrigerant mass flow rate injected between stages"
        )
        fs_state.set(
            name="m_flow_ref_eva", 
            value=m_flow_low, 
            unit="kg/s", 
            description="Refrigerant mass flow rate through the evaporator"
        )
        fs_state.set(
            name="m_flow_ref_con", 
            value=m_flow_high_required, 
            unit="kg/s", 
            description="Refrigerant mass flow rate through the condenser"
        )

        # Calculate volume flow rates
        rho_in_low = self.low_pressure_compressor.state_inlet.d
        V_flow_ref_eva = m_flow_low / rho_in_low
        
        # Calculate injection vapor state to get its density
        state_injection = self.med_prop.calc_state("PH", p_vapor_injection, h_vapor_injection)
        V_flow_ref_injection = m_flow_injection / state_injection.d
        
        rho_out_high = self.high_pressure_compressor.state_outlet.d
        V_flow_ref_hot_gas = m_flow_high_required / rho_out_high

        # Log Volume Flow Rates to FlowsheetState
        fs_state.set(
            name="V_flow_ref_suction",
            value=V_flow_ref_eva,
            unit="m^3/s",
            description="Refrigerant suction volume flow rate from evaporator"
        )
        fs_state.set(
            name="V_flow_ref_injection",
            value=V_flow_ref_injection,
            unit="m^3/s",
            description="Refrigerant injection volume flow rate"
        )
        fs_state.set(
            name="V_flow_ref_hot_gas",
            value=V_flow_ref_hot_gas,
            unit="m^3/s",
            description="Refrigerant hot gas volume flow rate to condenser"
        )

        # Log stage-specific efficiencies
        fs_state.set(
            name="eta_is_lp",
            value=self.low_pressure_compressor.get_eta_isentropic(p_outlet=p_vapor_injection, inputs=inputs),
            unit="-",
            description="Isentropic efficiency of the low-pressure compressor stage"
        )
        fs_state.set(
            name="lambda_h_lp",
            value=self.low_pressure_compressor.get_lambda_h(inputs=inputs),
            unit="-",
            description="Volumetric efficiency of the low-pressure compressor stage"
        )
        fs_state.set(
            name="eta_is_hp",
            value=self.high_pressure_compressor.get_eta_isentropic(p_outlet=p_2, inputs=inputs),
            unit="-",
            description="Isentropic efficiency of the high-pressure compressor stage"
        )        

        # Remove the variables logged by the primary compressor from the single stage cycle
        if "m_flow_ref" in fs_state.get_variables():
            del fs_state.get_variables()["m_flow_ref"]
        if "V_flow_ref" in fs_state.get_variables():
            del fs_state.get_variables()["V_flow_ref"]
        if "eta_is" in fs_state.get_variables():
            del fs_state.get_variables()["eta_is"]
        if "lambda_h" in fs_state.get_variables():
            del fs_state.get_variables()["lambda_h"]

        ## Check m_flow of both compressor stages to check if
        ## there would be an asymmetry of how much refrigerant is transported
        #m_flow_high = self.high_pressure_compressor.calc_m_flow(
        #    inputs=inputs, fs_state=fs_state
        #)
        #m_flow_low_should = m_flow_high * (1-x_vapor_injection)
        #percent_deviation = (m_flow_low - m_flow_low_should) / m_flow_low_should * 100
        #logger.debug("Deviation of mass flow rates is %s percent", percent_deviation)

        # Set states
        self.condenser.m_flow = self.high_pressure_compressor.m_flow
        self.condenser.state_inlet = self.high_pressure_compressor.state_outlet

        fs_state.set(
            name="T_1", value=self.evaporator.state_outlet.T,
            unit="K", description="Refrigerant temperature at evaporator outlet"
        )
        fs_state.set(
            name="T_2", value=self.high_pressure_compressor.state_outlet.T,
            unit="K", description="Compressor outlet temperature"
        )
        fs_state.set(
            name="T_3", value=self.condenser.state_outlet.T,
            unit="K", description="Refrigerant temperature at condenser outlet"
        )
        fs_state.set(
            name="T_4", value=self.evaporator.state_inlet.T,
            unit="K", description="Refrigerant temperature at evaporator inlet"
        )
        fs_state.set(
            name="p_con", value=p_2,
            unit="Pa", description="Condensation pressure"
        )
        fs_state.set(
            name="p_eva", value=p_1,
            unit="Pa", description="Evaporation pressure"
        )

    def calc_injection(self) -> (float, float, ThermodynamicState):
        """
        Calculate the injection component, e.g. phase separator
        or heat exchanger.
        In this function, child classes must set inlets
        and calculate outlets of additional components.

        Returns:
            float: Portion of vapor injected (x)
            float: Enthalpy of vapor injected
            ThermodynamicState: Inlet state of low pressure expansion valve
        """
        raise NotImplementedError

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        P_el_low = self.low_pressure_compressor.calc_electrical_power(
            inputs=inputs, fs_state=fs_state
        )
        # Capture LP mechanical efficiency
        eta_mech_lp = fs_state.get("eta_mech").value
        fs_state.set(name="eta_mech_lp", value=eta_mech_lp, unit="-", description="Mechanical efficiency of low stage")

        P_el_high = self.high_pressure_compressor.calc_electrical_power(
            inputs=inputs, fs_state=fs_state
        )
        # Capture HP mechanical efficiency
        eta_mech_hp = fs_state.get("eta_mech").value
        fs_state.set(name="eta_mech_hp", value=eta_mech_hp, unit="-", description="Mechanical efficiency of high stage")
        
        # Remove ambiguous mechanical efficiency from context
        if "eta_mech" in fs_state.get_variables():
            del fs_state.get_variables()["eta_mech"]
            
        # Calculate stage-specific global efficiencies
        eta_is_lp = fs_state.get("eta_is_lp")
        eta_is_hp = fs_state.get("eta_is_hp")
        if eta_is_lp is not None and eta_is_hp is not None:
            fs_state.set(
                name="eta_glob_lp", value=eta_is_lp.value * eta_mech_lp,
                unit="-", description="Global efficiency of low stage"
            )
            fs_state.set(
                name="eta_glob_hp", value=eta_is_hp.value * eta_mech_hp,
                unit="-", description="Global efficiency of high stage"
            )
            
        fs_state.set(
            name="P_el_low",
            value=P_el_low,
            unit="W",
            description="Electrical power consumption of low stage compressor"
        )
        fs_state.set(
            name="P_el_high",
            value=P_el_high,
            unit="W",
            description="Electrical power consumption of high stage compressor"
        )
        return P_el_low + P_el_high

    def get_states_in_order_for_plotting(self):
        """
        List with all relevant states of two-stage cycle
        except the intermediate component, e.g. phase separator
        or heat exchanger.
        """
        return [
            self.low_pressure_valve.state_inlet,                                # state 7
            self.low_pressure_valve.state_outlet,                               # state 4
            self.evaporator.state_inlet,                                        # state 4
            self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),   # state in evaporator at phase change
            self.evaporator.state_outlet,                                       # state 1
            self.low_pressure_compressor.state_inlet,                           # state 1
            self.low_pressure_compressor.state_outlet,                          # state 1_VI
            self.high_pressure_compressor.state_inlet,                          # state 1_VI_mixed
            self.high_pressure_compressor.state_outlet,                         # state 2
            self.condenser.state_inlet,                                         # state 2
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 1),    # state in condenser at phase change 1
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 0),    # state in condenser at phase change 0
            self.condenser.state_outlet,                                        # state 3
            self.high_pressure_valve.state_inlet,                               # state 3
            self.high_pressure_valve.state_outlet,                              # state 5
        ]
