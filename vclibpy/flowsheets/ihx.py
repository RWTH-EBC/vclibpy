import abc
import logging

from vclibpy.flowsheets import BaseCycle
from vclibpy.datamodels import FlowsheetState, Inputs
from vclibpy.components.compressors import Compressor
from vclibpy.components.expansion_valves import ExpansionValve
from vclibpy.components.heat_exchangers import MVB_LMTD_IHX


logger = logging.getLogger(__name__)


class IHX(BaseCycle, abc.ABC):
    """
    Class for a ihx cycle with


    """

    flowsheet_name = "IHX"

    def __init__(
            self,
            compressor: Compressor,
            expansion_valve1: ExpansionValve,
            expansion_valve2: ExpansionValve,
            ihx: MVB_LMTD_IHX,
            **kwargs
    ):
        super().__init__(**kwargs)
        self.compressor = compressor
        self.expansion_valve1 = expansion_valve1
        self.expansion_valve2 = expansion_valve2
        self.ihx = ihx
        self.flowsheet_name = "IHX"

    def get_all_components(self):
        return super().get_all_components() + [
            self.compressor,
            self.expansion_valve1,
            self.expansion_valve2,
            self.ihx
        ]

    def get_states_in_order_for_plotting(self):  # noch anpassen
        return [
            self.evaporator.state_inlet,
            self.med_prop.calc_state("PQ", self.evaporator.state_inlet.p, 1),
            self.evaporator.state_outlet,
            self.compressor.state_inlet,
            self.compressor.state_outlet,
            self.condenser.state_inlet,
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 1),
            self.med_prop.calc_state("PQ", self.condenser.state_inlet.p, 0),
            self.condenser.state_outlet,

        ]

    def set_ihx_outlet_based_on_superheating(self, p_eva: float, inputs: Inputs):
        """
        Calculate the outlet state of the evaporator based on
        the required degree of superheating.

        Args:
            p_eva (float): Evaporation pressure
            inputs (Inputs): Inputs with superheating level
        """
        T_1 = self.med_prop.calc_state("PQ", p_eva, 1).T + inputs.dT_eva_superheating
        if inputs.dT_eva_superheating > 0:
            self.ihx.state_outlet_low = self.med_prop.calc_state("PT", p_eva, T_1)
        else:
            self.ihx.state_outlet_low = self.med_prop.calc_state("PQ", p_eva, 1)

    def set_evaporator_outlet_no_superheating(self, p_eva: float, inputs: Inputs):
        """
        Calculate the outlet state of the evaporator based on
        the required degree of superheating.

        Args:
            p_eva (float): Evaporation pressure
            inputs (Inputs): Inputs with superheating level
        """

        self.evaporator.state_outlet = self.med_prop.calc_state("PQ", p_eva, 1)

    def set_evaporator_outlet_ihx(self, p_eva: float):
        """
        Calculate the outlet state of the evaporator in IHX Cycle (q=1)
        Args:
            p_eva (float): Evaporation pressure
        """
        self.evaporator.state_outlet = self.med_prop.calc_state("PQ", p_eva, 1)

    def set_evaporator_inlet_based_on_dh_ihx(self, p_eva: float, dh_ihx:float):

        h_eva_in = self.condenser.state_outlet.h - dh_ihx
        self.evaporator.state_inlet = self.med_prop.calc_state("PH", p_eva, h_eva_in)

    def set_ihx_outlet_high_based_on_dh_ihx(self, p_ihx: float, dh_ihx:float):
        self.ihx.state_outlet_high = self.med_prop.calc_state("PH", p_ihx, self.ihx.state_inlet_high.h - dh_ihx)

    def get_states(self):

        return {
            "1": self.compressor.state_inlet,
            "1_q1":  self.med_prop.calc_state("PQ", self.compressor.state_inlet.p, 1),
            "2": self.compressor.state_outlet,
            "2_s": self.med_prop.calc_state("PS", self.compressor.state_outlet.p, self.compressor.state_inlet.s),
            "2_q1": self.med_prop.calc_state("PQ", self.compressor.state_outlet.p, 1),
            "3_q0": self.med_prop.calc_state("PQ", self.compressor.state_outlet.p, 0),
            "3": self.condenser.state_outlet,
            "4": self.expansion_valve1.state_outlet,
            "5":  self.expansion_valve2.state_inlet,
            "6": self.evaporator.state_inlet,
            "7": self.evaporator.state_outlet,
            }

    def calc_missing_IHX_states(self, inputs: Inputs, fs_state: FlowsheetState, **kwargs):

        min_iteration_step = kwargs.pop("min_iteration_step", 1)
        max_num_iterations = kwargs.pop("max_num_iterations", 1e5)
        err_ntu = kwargs.pop("max_err_ntu", 0.5)
        err_dT_min = kwargs.pop("max_err_dT_min", 0.1)

        num_iterations = 0
        p_ihx_history = []
        firstIteration = True
        success = True

        p_ihx_next = self.condenser.state_outlet.p
        step_p_ihx = 1000

        while True:
            if isinstance(max_num_iterations, (int, float)):
                if num_iterations > max_num_iterations:
                    logger.warning("Maximum number of iterations %s exceeded. Stopping.",
                                   max_num_iterations)
                    return

                if (num_iterations + 1) % (0.1 * max_num_iterations) == 0:
                    logger.info("Info: %s percent of max_num_iterations %s used",
                                100 * (num_iterations + 1) / max_num_iterations, max_num_iterations)

            p_ihx = p_ihx_next
            p_ihx_history.append(p_ihx)

            # State 4
            self.expansion_valve1.calc_outlet(p_ihx)
            self.ihx.state_inlet_high = self.expansion_valve1.state_outlet
            # State 5

            dh_ihx = self.ihx.state_outlet_low.h - self.ihx.state_inlet_low.h
            self.set_ihx_outlet_high_based_on_dh_ihx(p_ihx, dh_ihx)
            self.expansion_valve2.state_inlet = self.ihx.state_outlet_high
            # Check IHX
            error_ihx, dT_min_ihx = self.ihx.calc(inputs=inputs, fs_state=fs_state)
            # starting pressure is p_2, so the pressure cannot be increased
            if firstIteration:
                firstIteration = False
                if error_ihx < 0:
                    success = False
                    logger.critical("Breaking: IXH-pressure is higher than pressure after condenser")
                    break

            if error_ihx > 0:
                p_ihx_next = p_ihx - step_p_ihx
                continue
            else:
                if step_p_ihx > min_iteration_step:
                    p_ihx_next = p_ihx + step_p_ihx
                    step_p_ihx /= 10
                    continue
                elif error_ihx > err_ntu and dT_min_ihx > err_dT_min:
                    step_p_ihx = 1000
                    p_ihx_next = p_ihx + step_p_ihx
                    continue

            if p_ihx == p_ihx_next:
                # Check if solution was too far away. If so, jump back
                # And decrease the iteration step by factor 10.
                if step_p_ihx > min_iteration_step:
                    p_ihx_next = p_ihx - step_p_ihx
                    step_p_ihx /= 10
                    continue
                logger.info("Breaking: Converged")
                if p_ihx < self.evaporator.state_inlet.p:
                    logger.critical("Breaking: IHX-pressure is lower than evaporating pressure")
                break


        inputs.set(
            name="Q_eva",
            value=self.evaporator.calc_Q_flow(),
            unit="W",
            description="heat flux evaporator"
        )
        if inputs.fix_m_flow_eva == float(False):
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            m_flow_eva = inputs.Q_eva / (self.evaporator.secondary_cp * (inputs.T_eva_in - inputs.T_eva_out))
            inputs.set(
                name="m_flow_eva",
                value=m_flow_eva,
                unit="kg/s",
                description="Secondary side evaporator mass flow"
            )
        else:
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            T_eva_out = inputs.T_eva_in - (inputs.Q_eva / (inputs.Q_eva * inputs.m_flow_eva))
            inputs.set(
                name="T_eva_out",
                value=T_eva_out,
                unit="K",
                description="Secondary side evaporator outlet temperature"
            )

        fs_state.set(name="p_ihx", value=p_ihx, unit="Pa", description="Intermediate pressure")
        h2_is = self.med_prop.calc_state("PS", self.compressor.state_outlet.p, self.compressor.state_inlet.s).h
        h4_is = self.med_prop.calc_state("PS", p_ihx, self.expansion_valve1.state_inlet.s).h
        h5_is = self.med_prop.calc_state("PS", self.evaporator.state_inlet.p, self.expansion_valve2.state_inlet.s).h

        Comp_dh_is = 0.001*(h2_is-self.compressor.state_inlet.h)
        Ex_dh_is = 0.001 * (self.expansion_valve1.state_inlet.h + self.expansion_valve2.state_inlet.h - h4_is - h5_is)
        comp_dh_is_Ex_dh_is = Comp_dh_is/Ex_dh_is
        fs_state.set(name="Comp_dh_is", value=Comp_dh_is)
        fs_state.set(name="Exp_dh_is", value=Ex_dh_is)
        fs_state.set(name="Comp_dh_is_Exp_dh_is", value=comp_dh_is_Ex_dh_is)
        fs_state.set(name="Comp_dH_is", value=self.compressor.m_flow*Comp_dh_is)
        fs_state.set(name="Exp_dH_is", value=self.compressor.m_flow*Ex_dh_is)

    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):

    # State 3
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.expansion_valve1.state_inlet = self.condenser.state_outlet
    # State 7
        self.set_evaporator_outlet_no_superheating(p_eva=p_1, inputs=inputs)
        self.ihx.state_inlet_low = self.evaporator.state_outlet
    # State 1
        self.set_ihx_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)
        self.compressor.state_inlet = self.ihx.state_outlet_low
    # State 6
        dh_ihx = self.ihx.state_outlet_low.h-self.ihx.state_inlet_low.h
        self.set_evaporator_inlet_based_on_dh_ihx(p_1, dh_ihx)
        self.expansion_valve2.state_outlet = self.evaporator.state_inlet
    # State 2
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet
    # Iterate compressor speed
        if inputs.fix_speed == float(False):
            n_next = 0.01
            n_step = 0.1
            max_error = 0.0001
            n_iter = 0
            while True:
                n_iter += 1
                inputs.set(
                    name="n",
                    value=n_next,
                    unit="-",
                    description="Relative compressor speed"
                )
                self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
                self.condenser.m_flow = self.compressor.m_flow
                rel_error = 100 * (self.condenser.calc_Q_flow() - inputs.Q_con) / inputs.Q_con
                if abs(rel_error) < max_error:
                    break
                elif rel_error < 0:
                    n_next += n_step
                    continue
                n_next -= n_step
                n_step /= 10
                n_next += n_step
                continue
    # State 2
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)
        self.condenser.state_inlet = self.compressor.state_outlet

    # Mass flow rate:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.condenser.m_flow = self.compressor.m_flow
        self.expansion_valve1.m_flow = self.compressor.m_flow
        self.ihx.m_flow = self.compressor.m_flow
        self.expansion_valve2.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.ihx.m_flow_high = self.compressor.m_flow
        self.ihx.m_flow_low = self.compressor.m_flow

        inputs.set(
            name="Q_con",
            value=self.condenser.calc_Q_flow(),
            unit="W",
            description="heat flux condenser"
        )

        inputs.set(
            name="Q_eva",
            value=self.evaporator.calc_Q_flow(),
            unit="W",
            description="heat flux evaporator"
        )

        if inputs.fix_m_flow_eva == float(False):
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            m_flow_eva = inputs.Q_eva / (self.evaporator.secondary_cp * (inputs.T_eva_in - inputs.T_eva_out))
            inputs.set(
                name="m_flow_eva",
                value=m_flow_eva,
                unit="kg/s",
                description="Secondary side evaporator mass flow"
            )
        else:
            self.evaporator.calc_secondary_cp(T=inputs.T_eva_in)
            T_eva_out = inputs.T_eva_in - (inputs.Q_eva / (inputs.Q_eva * inputs.m_flow_eva))
            inputs.set(
                name="T_eva_out",
                value=T_eva_out,
                unit="K",
                description="Secondary side evaporator outlet temperature"
            )

        if inputs.fix_m_flow_con == float(False):
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            m_flow_con = inputs.Q_con / (self.condenser.secondary_cp * (inputs.T_con_out - inputs.T_con_in))
            inputs.set(
                name="m_flow_con",
                value=m_flow_con,
                unit="kg/s",
                description="Secondary side condenser mass flow"
            )
        else:
            self.condenser.calc_secondary_cp(T=inputs.T_con_in)
            T_con_out = inputs.T_con_in + (inputs.Q_con / self.condenser.secondary_cp * inputs.m_flow_con)
            inputs.set(
                name="T_con_out",
                value=T_con_out,
                unit="K",
                description="Secondary side condenser outlet temperature"
            )
        fs_state.set(name="compressor_speed", value=inputs.n*self.compressor.N_max, unit="1/s",
                     description="Compressor Speed")
        fs_state.set(name="relative_compressor_speed", value=inputs.n , unit="1/s",
                     description="relative Compressor Speed")

        fs_state.set(name="Comp_dh", value=0.001 * (self.compressor.state_outlet.h - self.compressor.state_inlet.h))

    def calc_electrical_power(self, inputs: Inputs, fs_state: FlowsheetState):
        """Based on simple energy balance - Adiabatic"""
        return self.compressor.calc_electrical_power(inputs=inputs, fs_state=fs_state)

