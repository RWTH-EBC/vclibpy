import numpy as np

from vclibpy.components.heat_exchangers import ntu
from vclibpy.media import ThermodynamicState
from vclibpy.flowsheets.vapor_injection_economizer import VaporInjectionEconomizer
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




class VaporInjectionEconomizerDownstream(VaporInjectionEconomizer):
    """
    Cycle with vapor injection using an economizer (DOWNSTREAM configuration).

    Topology (conceptual):
    ----------------------
    - Primary side:
        3 (condenser outlet) → economizer primary → 7 (economizer outlet)

    - From state 7, the flow is split:
        • (1 - x_vi) → low-pressure expansion valve → evaporator → 1
        • x_vi       → high-pressure expansion valve → 5 → economizer secondary → 6 → injection line

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "VaporInjectionEconomizerDownstream"

    # ------------------------------------------------------------------
    # For downstream, HP-valve inlet should be at IHX/economizer outlet
    # (state 7). BaseVaporInjection must call this instead of hardcoding
    # condenser.state_outlet.
    # ------------------------------------------------------------------


    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        k_vapor_injection_var = inputs.control.get("k_vapor_injection")
        # Extract the numerical value. If the variable is not present, use a default.
        # Default is 1 according to Xu, 2019
        if k_vapor_injection_var is not None:
            k_vapor_injection = k_vapor_injection_var.value
        else:
            k_vapor_injection = 1.0

        p_vapor_injection = k_vapor_injection * np.sqrt(
            p_1 * p_2)  # TODO: are there other ways to set the mid pressure level?

        # -----------------------------------------------------------
        #  Condenser outlet (state 3)
        # -----------------------------------------------------------
        # Compute the condenser outlet thermodynamic state based on:
        #   • condenser pressure p_con = p_2
        #   • user-specified subcooling level
        #
        # The resulting state is a subcooled liquid, which serves as
        # the primary-side inlet of the economizer.
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.economizer.state_inlet = self.condenser.state_outlet

        # -----------------------------------------------------------
        #  Evaporator outlet (state 1)
        # -----------------------------------------------------------
        # Compute the evaporator outlet state using:
        #   • evaporating pressure p_eva = p_1
        #   • user-defined superheat
        #
        # This generates a superheated vapor state, which acts as the
        # suction (inlet) condition for the low-pressure compressor.
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)

        # -----------------------------------------------------------
        #  Low-pressure compressor inlet assignment
        # -----------------------------------------------------------
        # The evaporator outlet directly feeds the LP compressor,
        # ensuring thermodynamic continuity between components.
        self.low_pressure_compressor.state_inlet = self.evaporator.state_outlet

        # -----------------------------------------------------------
        #  Low-pressure compressor outlet (state 1_VI)
        # -----------------------------------------------------------
        # The LP compressor is evaluated up to the intermediate pressure
        # p_vapor_injection (injection pressure). This defines the LP-stage
        # discharge state before mixing with the injected vapor.
        self.low_pressure_compressor.calc_state_outlet(
            p_outlet=p_vapor_injection, inputs=inputs, fs_state=fs_state
        )

        # -----------------------------------------------------------
        #  Mass flow calculation for LP components
        # -----------------------------------------------------------
        # Determine the LP-side mass flow using the compressor model.
        # This mass flow is shared among:
        #   • evaporator
        #   • LP compressor
        #   • LP expansion valve
        #
        # Because these three components are in series, they must carry
        # the same refrigerant mass flow to satisfy mass conservation.
        self.low_pressure_compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        m_flow_low = self.low_pressure_compressor.m_flow
        self.evaporator.m_flow = m_flow_low
        self.low_pressure_valve.m_flow = self.evaporator.m_flow

        # -----------------------------------------------------------
        #  Economizer secondary outlet (state 6)
        # -----------------------------------------------------------
        # Define the economizer secondary-side outlet as saturated vapor
        # at the intermediate (injection) pressure:
        #
        #   • mode: "PQ" ⇒ pressure + vapor quality
        #   • q = 1      ⇒ saturated vapor
        #
        # This state represents the vapor that will be injected into the
        # compression process via the vapor-injection port.
        self.economizer.state_two_phase_outlet = self.med_prop.calc_state("PQ", p_vapor_injection, 1)

        print(f"Before Injection prozess def calc_states in class VaporInjectionEconomizerDownstream")

        # -----------------------------------------------------------
        # Vapor-injection subsystem
        # -----------------------------------------------------------
        # Compute the injection-related quantities:
        #   • x_vapor_injection   → mass fraction of injected vapor (dimensionless)
        #   • h_vapor_injection   → enthalpy of the injected vapor
        #   • state_economizer_outlet → primary-side outlet of the economizer (state 7)
        #
        # This function internally performs the economizer energy balance,
        # the quality search on the secondary side, and enforces h7 constraints.
        x_vapor_injection, h_vapor_injection, state_economizer_outlet = self.calc_injection()

        print(f"after Injection prozess def calc_states in class VaporInjectionEconomizerDownstream")

        # -----------------------------------------------------------
        # Economizer: primary-side outlet assignment
        # -----------------------------------------------------------
        # The economizer primary outlet (state 7) is the upstream condition
        # feeding both expansion valves (LP and HP). It is returned from
        # calc_injection() after satisfying the subcooling split and matching
        # the enthalpy constraints (h7 consistency)
        self.economizer.state_outlet = state_economizer_outlet

        print(f"state_economizer_outlet: {state_economizer_outlet}")
        print(f"economizer.state_outlet: {self.economizer.state_outlet}")
        print(f"after setting economizer.state_outlet def calc_states in class VaporInjectionEconomizerDownstream")

        # -----------------------------------------------------------
        # Expansion valves (LP and HP) – inlet assignment
        # -----------------------------------------------------------
        # Both expansion valves take the economizer primary outlet (state 7)
        # as their inlet condition:
        #
        #   • LP-valve  → expands down to evaporator pressure p_1
        #   • HP-valve  → expands down to injection/intermediate pressure p_vapor_injection
        #
        # These two throttling processes generate:
        #   • A two-phase mixture for the evaporator (LP path)
        #   • A two-phase mixture that vaporizes inside the economizer (HP path)
        self.low_pressure_valve.state_inlet = state_economizer_outlet
        self.high_pressure_valve.state_inlet = state_economizer_outlet

        # -----------------------------------------------------------
        # High-pressure expansion valve (HP valve → injection line)
        # -----------------------------------------------------------
        # Perform the isenthalpic expansion from p_cond to p_vapor_injection.
        # This defines the secondary-side inlet of the economizer (state 5).
        #
        # The HP expansion produces a low-quality two-phase mixture that
        # evaporates inside the economizer, generating saturated or superheated
        # vapor for injection into the compressor.
        print(f"high_pressure_valve.state_inlet: {self.high_pressure_valve.state_inlet}")
        print(f"economizer.state_two_phase_inlet before HP valve calc_outlet: {self.economizer.state_two_phase_inlet}")
        self.high_pressure_valve.calc_outlet(p_outlet=p_vapor_injection)
        print(f"high_pressure_valve.state_outlet after calc_outlet: {self.high_pressure_valve.state_outlet}")
        # Feed the economizer with the secondary-side inlet (state 5)
        self.economizer.state_two_phase_inlet = self.high_pressure_valve.state_outlet
        print(f"economizer.state_two_phase_inlet after HP valve calc_outlet: {self.economizer.state_two_phase_inlet}")

        # -----------------------------------------------------------
        # Low-pressure expansion valve (LP valve → evaporator line)
        # -----------------------------------------------------------
        # Isenthalpic throttling from condensation level (state 7) to the
        # evaporator pressure p_1. This generates a two-phase mixture that
        # enters the evaporator (state 4).
        self.low_pressure_valve.calc_outlet(p_outlet=p_1)

        # Assign the evaporator inlet state (state 4)
        self.evaporator.state_inlet = self.low_pressure_valve.state_outlet



        # -----------------------------------------------------------
        # Ideal mixing process (LP-compressor discharge + injected vapor)
        # -----------------------------------------------------------
        # Compute the mixed enthalpy at the injection pressure. The mixture
        # consists of:
        #   • (1 - x_vi) * LP compressor discharge stream  (state 1_VI)
        #   • x_vi       * injected vapor from economizer  (state 6)
        #
        # This is an isobaric, ideal mixing process at p_vapor_injection,
        # assuming no additional heat transfer or pressure loss.
        h_1_VI_mixed = (
                (1 - x_vapor_injection) * self.low_pressure_compressor.state_outlet.h +
                x_vapor_injection * h_vapor_injection
        )

        # -----------------------------------------------------------
        # High-pressure compressor inlet (state 1_VI_mix)
        # -----------------------------------------------------------
        # Construct the thermodynamic state at the injection pressure using:
        #   • state type: "PH"  (pressure + enthalpy)
        #   • p = p_vapor_injection
        #   • h = h_1_VI_mixed
        #
        # This defines the suction condition for the high-pressure stage,
        # after the injection has thermodynamically mixed with the LP-stage
        # discharge.
        self.high_pressure_compressor.state_inlet = self.med_prop.calc_state(
            "PH", p_vapor_injection, h_1_VI_mixed
        )

        # -----------------------------------------------------------
        # High-pressure compressor outlet (state 2)
        # -----------------------------------------------------------
        # Compress the mixed injection state up to the condenser pressure p_2.
        # This produces the high-pressure discharge state (state 2).
        self.high_pressure_compressor.calc_state_outlet(
            p_outlet=p_2, inputs=inputs, fs_state=fs_state
        )
        # Assign HP compressor discharge as the inlet to the condenser (state 2 → state 3)
        self.condenser.state_inlet = self.high_pressure_compressor.state_outlet

        # -----------------------------------------------------------
        # Mass flow consistency check between LP and HP compressors
        # -----------------------------------------------------------
        # Assumption:
        #   x_vi = m_inj / m_high        (injected mass fraction at HP inlet)
        # Mass balance:
        #   m_high = m_low + m_inj
        # Therefore:
        #   m_low_should = (1 - x_vi) * m_high
        #
        # Compare LP compressor mass flow vs. theoretical value.
        m_flow_high = self.high_pressure_compressor.calc_m_flow(
            inputs=inputs, fs_state=fs_state
        )
        m_flow_low_should = m_flow_high * (1 - x_vapor_injection)
        percent_deviation = (m_flow_low - m_flow_low_should) / m_flow_low_should * 100
        logger.debug("Deviation of mass flow rates is %s percent", percent_deviation)

        # -----------------------------------------------------------
        # Propagate mass flow to condenser and economizer
        # -----------------------------------------------------------
        # The condenser and the economizer operate at the high-side mass
        # flow rate, which equals the high-pressure compressor discharge flow.
        self.condenser.m_flow = self.high_pressure_compressor.m_flow

        # High-side mass flow: both the condenser and economizer operate at m_high
        self.economizer.m_flow = self.high_pressure_compressor.m_flow

        # -----------------------------------------------------------
        # Update flowsheet state variables (fs_state)
        # -----------------------------------------------------------
        # Expose key thermodynamic states to the flowsheet for monitoring,
        # logging, and UI output. The temperatures assigned correspond to:
        #
        #   T_1 → Evaporator outlet
        #   T_2 → HP compressor discharge
        #   T_3 → Condenser outlet
        #   T_4 → Evaporator inlet
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

    def calc_injection(self):

        # Step size for quality (Q) at economizer two-phase inlet (state 5)
        _Q_economizer_twophase_inlet_step = 0.0001
        _min_step_Q_economizer_twophase_inlet_step = 0.0000000001
        Q_economizer_twophase_inlet_next = _min_step_Q_economizer_twophase_inlet_step
        print(f"calc_injection başı _Q_economizer_twophase_inlet_step: {_Q_economizer_twophase_inlet_step}")

        # Initial guess for two-phase inlet (state 5) at the injection pressure
        self.economizer.state_two_phase_inlet = self.med_prop.calc_state("PQ", self.economizer.state_two_phase_outlet.p, Q_economizer_twophase_inlet_next)
        print(f"initial economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")

        # Initial guess for primary-side outlet (state 7): set equal to condenser outlet
        # self.economizer.state_outlet = self.economizer.state_inlet     # initial guess
        print(f"initial economizer.state_outlet:          {self.economizer.state_outlet}")

        t = 0
        # Outer loop: iterate over quality Q at secondary inlet (state 5)
        while True:
            t += 1
            print(f"Outer loop iteration: {t}")

            if _Q_economizer_twophase_inlet_step < _min_step_Q_economizer_twophase_inlet_step:
                print(f"Breaking outer loop")
                break

            # Step control for vapor injection mass fraction x_vi
            _x_vi_step = 0.01
            _min_step_x_vi = 0.00000001
            x_vi_next = _min_step_x_vi  # Don't start from zero

            # Fix current Q guess for this outer iteration
            Q_economizer_twophase_inlet = Q_economizer_twophase_inlet_next
            print(f"Q_economizer_twophase_inlet: {Q_economizer_twophase_inlet}")
            # Update state 5 = (p_injection, Q_guess)
            self.economizer.state_two_phase_inlet = self.med_prop.calc_state("PQ", self.economizer.state_two_phase_outlet.p, Q_economizer_twophase_inlet)
            print(f"after calculate state economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
            # Base mass flow on evaporator (LP side)
            m_flow_evaporator = self.evaporator.m_flow

            # Enthalpy lift across two-phase side of economizer (state 5 → 6)
            dh_ihe_goal = (
                    self.economizer.state_two_phase_outlet.h -
                    self.economizer.state_two_phase_inlet.h
            )

            # Transport properties on primary side (liquid)
            tra_properties_liquid = self.med_prop.calc_transport_properties(
                self.economizer.state_inlet
            )
            alpha_liquid = self.economizer.calc_alpha_liquid(tra_properties_liquid)

            # Mean transport properties on secondary (two-phase) side
            tra_properties_two_phase = self.med_prop.calc_mean_transport_properties(
                self.economizer.state_two_phase_inlet,
                self.economizer.state_two_phase_outlet
            )
            alpha_two_phase = self.economizer.calc_alpha_liquid(tra_properties_two_phase)

            # ---------------------------------------------------------------
            # Effective cp on secondary side (used in NTU method)
            # ---------------------------------------------------------------
            dT_secondary = (
                    self.economizer.state_two_phase_outlet.T -
                    self.economizer.state_two_phase_inlet.T
            )
            if dT_secondary == 0:
                cp_4 = np.inf
            else:
                cp_4 = dh_ihe_goal / dT_secondary
            self.economizer.set_secondary_cp(cp=cp_4)
            primary_cp = tra_properties_liquid.cp

            # -----------------------------------------------------------
            # Inner loop: iterate vapor-injection fraction x_vi
            # Goal: find x_vi such that NTU heat transfer (Q_flow)
            #       matches the required enthalpy lift (Q_flow_goal)
            # -----------------------------------------------------------
            while True:
                x_vi = x_vi_next
                x_eva = 1 - x_vi

                # Mass-flow definitions for the upstream configuration
                m_flow_evaporator = 0.3   #if you get h7<h5 at the beginning of iteration, then try to increase m_flow_evaporator here, for example 0.3
                m_flow_vapor_injection = (x_vi / (1-x_vi)) * m_flow_evaporator #vorsichtttttttttttt
                m_flow_sum = m_flow_evaporator + m_flow_vapor_injection

                # Target heat transfer on economizer secondary side
                Q_flow_goal = dh_ihe_goal * m_flow_vapor_injection

                # Assign primary & secondary mass flows
                self.economizer.m_flow = m_flow_sum
                self.economizer.m_flow_secondary = m_flow_vapor_injection

                # NTU heat-exchanger model
                k = self.economizer.calc_k(alpha_liquid, alpha_two_phase)
                Q_flow = ntu.calc_Q_ntu(
                    k=k,
                    dT_max=(
                            self.economizer.state_inlet.T -
                            self.economizer.state_two_phase_inlet.T  # Frage!
                    ),
                    A=self.economizer.A,
                    flow_type=self.economizer.flow_type,
                    m_flow_primary_cp=self.economizer.m_flow * primary_cp,
                    m_flow_secondary_cp=self.economizer.m_flow_secondary_cp
                )

                # print(f"Inner loop calc_injection x_vi: {x_vi}, Q_flow: {Q_flow}, Q_flow_goal: {Q_flow_goal}")
                # print(f"m_flow_primary: {self.economizer.m_flow}")
                # print(f"m_flow_secondary: {m_flow_vapor_injection}")
                # print(f"self.evaporator.m_flow: {self.evaporator.m_flow}")

                # Adjust x_vi until Q_flow ≈ Q_flow_goal
                if Q_flow > Q_flow_goal:
                    # Increase injection fraction
                    if abs(x_vi) >= 0.9 or x_vi < 0 :
                        break
                    if _x_vi_step <= _min_step_x_vi:
                        break
                    x_vi_next = x_vi + _x_vi_step


                else:
                    if abs(x_vi) >= 0.9 or x_vi <0 or x_eva <0:
                        break

                    # Decrease injection fraction and refine step
                    x_vi_next = x_vi - _x_vi_step * 0.9
                    _x_vi_step /= 10

            # -----------------------------------------------------------
            # Update economizer primary outlet (state 7)
            # Enthalpy balance: h7 = h3 - x_vi * (h6 - h5)
            # -----------------------------------------------------------
            h_7 = self.economizer.state_inlet.h - x_vi  * (self.economizer.state_two_phase_outlet.h - self.economizer.state_two_phase_inlet.h)
            self.economizer.state_outlet = self.med_prop.calc_state("PH", self.condenser.state_outlet.p, h_7)
            print(f"deneme")
            print(f"x_vi: {x_vi}")
            print(f"fark Q: {Q_flow - Q_flow_goal}")
            print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            # -----------------------------------------------------------
            # Check whether h7 matches h5 within tolerance.
            # If not, adjust the quality Q at state 5 (outer loop).
            # -----------------------------------------------------------
            if self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h > 0.001:
                # If Q-step becomes too small → stop
                if _Q_economizer_twophase_inlet_step <= _min_step_Q_economizer_twophase_inlet_step:
                    print(f"_Q_economizer_twophase_inlet_step: {_Q_economizer_twophase_inlet_step}")
                    print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
                    print(f"_min_step_Q_economizer_twophase_inlet_step: {_min_step_Q_economizer_twophase_inlet_step}")
                    print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                    print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                    break

                # Increase Q guess
                Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet + _Q_economizer_twophase_inlet_step
                print(f"before deneme2 Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
                print(f"deneme2")
            # elif (self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) <0: #sıkıntı şu direkt burada başlıyor ve Qstepi çok düşürüyor sonra minstepsınırını aşıyor break atıyor
            #     #while self.economizer.state_outlet.h <! self.economizer.state_two_phase_inlet.h:  x_vi azalt
            #
            #     print(f"economizer.state_outlet: {self.economizer.state_outlet}")
            #     print(f"economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
            #     print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #     Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet - _Q_economizer_twophase_inlet_step * 0.9
            #     _Q_economizer_twophase_inlet_step /= 10
            #     print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            #     print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            #     print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            #     print(f"_Q_economizer_twophase_inlet_step düzeltme sonrası: {_Q_economizer_twophase_inlet_step}")
            #     print(f"Düzeltme sonrası Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")



            else:
                # Close enough → refine Q step
                print(f"else block {_Q_economizer_twophase_inlet_step}")
                print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet - _Q_economizer_twophase_inlet_step * 0.9
                _Q_economizer_twophase_inlet_step /= 10

                # Stop if tolerance satisfied
                if abs(self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) <= 0.001:
                    print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
                    print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                    print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                    print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                    break
                if Q_economizer_twophase_inlet_next < 0:
                    print("Q < 0 hatası, sıfıra sabitliyorum1")
                    Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet + _Q_economizer_twophase_inlet_step

        print(
            f"break oluyor mu gerçekten? fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
        print(f"before return: ")
        print(f"self.economizer.state_outlet:          {self.economizer.state_outlet},")
        print(f"self.economizer.state_two_phase.inlet: {self.economizer.state_two_phase_inlet} ")
        print(f"now returning...")

            # Final return values:
            #   x_vi → optimal vapor injection fraction
            #   h6   → enthalpy of injected vapor
            #   state7 → economizer primary outlet
        return x_vi, self.economizer.state_two_phase_outlet.h, self.economizer.state_outlet



            # # Safety: ensure
            # # h7 >= h5
            # elif (self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) < -100:  # sıkıntı şu direkt burada başlıyor ve Qstepi çok düşürüyor sonra minstepsınırını aşıyor break atıyor
            #     print(f" elif <-100 bloğu çalıştı")
            #     print(f"economizer.state_outlet: {self.economizer.state_outlet}")
            #     print(f"economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
            #     print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #     Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet - _Q_economizer_twophase_inlet_step * 0.9
            #     _Q_economizer_twophase_inlet_step /= 10
            #     print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            #     print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            #     print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            #     print(f"_Q_economizer_twophase_inlet_step düzeltme sonrası: {_Q_economizer_twophase_inlet_step}")
            #     print(f"Düzeltme sonrası Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #     if Q_economizer_twophase_inlet_next < 0:
            #         print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #         print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            #         print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            #         print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            #         print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #         print("Q < 0 hatası, sıfıra sabitliyorum2")
            #         Q_economizer_twophase_inlet_next = 0

    def get_states_in_order_for_plotting(self):
        """
        Returns the thermodynamic states in cycle order for P–h / T–s plotting.
        The sequence follows the flow of the downstream vapor-injection cycle.
        """
        print(f"son mu?")
        print("\n==================== FLOW DEBUG (DOWNSTREAM VI CYCLE) ====================\n")

        # ---------------------------------------------------------
        # 1. CONDENSER OUTLET (STATE 3)
        # ---------------------------------------------------------
        print(f"[3] condenser_outlet: {self.condenser.state_outlet}")

        # ---------------------------------------------------------
        # 2. ECONOMIZER PRIMARY (3 → 7)
        # ---------------------------------------------------------
        print(f"[3] economizer_primary_inlet: {self.economizer.state_inlet}")
        print(f"[7] economizer_primary_outlet: {self.economizer.state_outlet}")

        # ---------------------------------------------------------
        # 3. ECONOMIZER SECONDARY (5 → 6)  *** <-- senin istediğin yer burası
        # ---------------------------------------------------------
        print(f"[5] economizer_twophase_inlet: {self.economizer.state_two_phase_inlet}")
        print(f"[6] economizer_twophase_outlet (injected vapor): {self.economizer.state_two_phase_outlet}")

        # ---------------------------------------------------------
        # 4. SPLITTER (state 7 → LP EV + HP EV)
        # ---------------------------------------------------------
        print(f"[7→4] lp_valve_inlet: {self.low_pressure_valve.state_inlet}")
        print(f"[7→5] hp_valve_inlet: {self.high_pressure_valve.state_inlet}")

        # ---------------------------------------------------------
        # 5. LP EXPANSION VALVE (7 → 4)
        # ---------------------------------------------------------
        print(f"[4] lp_valve_outlet (evaporator_inlet): {self.low_pressure_valve.state_outlet}")

        # ---------------------------------------------------------
        # 6. EVAPORATOR (4 → 1)
        # ---------------------------------------------------------
        print(f"[4] evaporator_inlet: {self.evaporator.state_inlet}")
        print(f"[1] evaporator_outlet: {self.evaporator.state_outlet}")

        # ---------------------------------------------------------
        # 7. LP COMPRESSOR (1 → 1_VI)
        # ---------------------------------------------------------
        print(f"[1] lp_compressor_inlet: {self.low_pressure_compressor.state_inlet}")
        print(f"[1_VI] lp_compressor_outlet (before mixing): {self.low_pressure_compressor.state_outlet}")

        # ---------------------------------------------------------
        # 8. INJECTION MIXING (1_VI + 6)
        # ---------------------------------------------------------
        print(f"[mix] injection_mixed_state (hp_compressor_inlet): {self.high_pressure_compressor.state_inlet}")

        # ---------------------------------------------------------
        # 9. HIGH-PRESSURE COMPRESSOR (mix → 2)
        # ---------------------------------------------------------
        print(f"[2] hp_compressor_outlet: {self.high_pressure_compressor.state_outlet}")


        # print(f"m_flow_low: {m_flow_low}")
        # print(f"m_flow_low_should: {m_flow_low_should}")
        # print(f"percent_deviation: {percent_deviation}")
        # print(f"m_flow_high: {m_flow_high}")
        # print(f"m_sum_flow_economizer: {self.economizer.m_flow}")
        # print(f"x_vapor_injection: {x_vapor_injection}")
        # print(f"check x_vapor_injection m_flow_low/m_flow_high: {m_flow_low / m_flow_high}")

        print("\n==================== END FLOW DEBUG ====================\n")

        return  [
            self.low_pressure_valve.state_inlet,  # state 7  (econ primary outlet → LP valve)
            self.low_pressure_valve.state_outlet, # state 4  (LP valve outlet → evaporator inlet)
            self.evaporator.state_inlet,                                        # state 4  (evaporator inlet, identical)
            self.med_prop.calc_state("PQ", self.evaporator.state_outlet.p, 1),  # saturated vapor line in evaporator
            self.evaporator.state_outlet,                                       # state 1  (evaporator outlet)
            self.low_pressure_compressor.state_inlet,  # state 1
            self.low_pressure_compressor.state_outlet,  # state 1_VI (LP compressor discharge)
            self.high_pressure_compressor.state_inlet,  # state 1_VI_mixed (after vapor injection)
            self.high_pressure_compressor.state_outlet,  # state 2
            self.condenser.state_inlet,  # state 2
            self.med_prop.calc_state("PQ", self.condenser.state_outlet.p, 1),  # saturated vapor in condenser
            self.med_prop.calc_state("PQ", self.condenser.state_outlet.p, 0),  # saturated liquid in condenser
            self.condenser.state_outlet,  # state 3
            self.economizer.state_inlet,  # state 3
            self.economizer.state_outlet, # state 5
            self.high_pressure_valve.state_inlet,             # state 7
            self.high_pressure_valve.state_outlet,     # state 7 (path to HP EV)
            self.economizer.state_two_phase_inlet,    # state 5
            self.economizer.state_two_phase_outlet,   # state 6
            self.high_pressure_compressor.state_inlet,
            # Go back to the condenser outlet
            self.economizer.state_two_phase_outlet,  # state 6
            self.economizer.state_two_phase_inlet,  # state 5
            self.high_pressure_valve.state_outlet,  # state 5
            self.high_pressure_valve.state_inlet,  # state 3
            self.economizer.state_inlet,  # state 3
            self.economizer.state_outlet,  # state 7 (path to the evaporator)
            self.condenser.state_outlet,  # state 3 (path back to the splitting point)
        ]


import numpy as np

from vclibpy.components.heat_exchangers import ntu
from vclibpy.media import ThermodynamicState
from vclibpy.flowsheets.vapor_injection_economizer import VaporInjectionEconomizer
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




class VaporInjectionEconomizerDownstream(VaporInjectionEconomizer):
    """
    Cycle with vapor injection using an economizer (DOWNSTREAM configuration).

    Topology (conceptual):
    ----------------------
    - Primary side:
        3 (condenser outlet) → economizer primary → 7 (economizer outlet)

    - From state 7, the flow is split:
        • (1 - x_vi) → low-pressure expansion valve → evaporator → 1
        • x_vi       → high-pressure expansion valve → 5 → economizer secondary → 6 → injection line

    Notes
    -----
    See parent docstring for info on further assumptions and parameters.
    """

    flowsheet_name = "VaporInjectionEconomizerDownstream"

    # ------------------------------------------------------------------
    # For downstream, HP-valve inlet should be at IHX/economizer outlet
    # (state 7). BaseVaporInjection must call this instead of hardcoding
    # condenser.state_outlet.
    # ------------------------------------------------------------------


    def calc_states(self, p_1, p_2, inputs: Inputs, fs_state: FlowsheetState):
        k_vapor_injection_var = inputs.control.get("k_vapor_injection")
        # Extract the numerical value. If the variable is not present, use a default.
        # Default is 1 according to Xu, 2019
        if k_vapor_injection_var is not None:
            k_vapor_injection = k_vapor_injection_var.value
        else:
            k_vapor_injection = 1.0

        p_vapor_injection = k_vapor_injection * np.sqrt(
            p_1 * p_2)  # TODO: are there other ways to set the mid pressure level?

        # -----------------------------------------------------------
        #  Condenser outlet (state 3)
        # -----------------------------------------------------------
        # Compute the condenser outlet thermodynamic state based on:
        #   • condenser pressure p_con = p_2
        #   • user-specified subcooling level
        #
        # The resulting state is a subcooled liquid, which serves as
        # the primary-side inlet of the economizer.
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        self.economizer.state_inlet = self.condenser.state_outlet

        # -----------------------------------------------------------
        #  Evaporator outlet (state 1)
        # -----------------------------------------------------------
        # Compute the evaporator outlet state using:
        #   • evaporating pressure p_eva = p_1
        #   • user-defined superheat
        #
        # This generates a superheated vapor state, which acts as the
        # suction (inlet) condition for the low-pressure compressor.
        self.set_evaporator_outlet_based_on_superheating(p_eva=p_1, inputs=inputs)

        # -----------------------------------------------------------
        #  Low-pressure compressor inlet assignment
        # -----------------------------------------------------------
        # The evaporator outlet directly feeds the LP compressor,
        # ensuring thermodynamic continuity between components.
        self.low_pressure_compressor.state_inlet = self.evaporator.state_outlet

        # -----------------------------------------------------------
        #  Low-pressure compressor outlet (state 1_VI)
        # -----------------------------------------------------------
        # The LP compressor is evaluated up to the intermediate pressure
        # p_vapor_injection (injection pressure). This defines the LP-stage
        # discharge state before mixing with the injected vapor.
        self.low_pressure_compressor.calc_state_outlet(
            p_outlet=p_vapor_injection, inputs=inputs, fs_state=fs_state
        )

        # -----------------------------------------------------------
        #  Mass flow calculation for LP components
        # -----------------------------------------------------------
        # Determine the LP-side mass flow using the compressor model.
        # This mass flow is shared among:
        #   • evaporator
        #   • LP compressor
        #   • LP expansion valve
        #
        # Because these three components are in series, they must carry
        # the same refrigerant mass flow to satisfy mass conservation.
        self.low_pressure_compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        m_flow_low = self.low_pressure_compressor.m_flow
        self.evaporator.m_flow = m_flow_low
        self.low_pressure_valve.m_flow = self.evaporator.m_flow

        # -----------------------------------------------------------
        #  Economizer secondary outlet (state 6)
        # -----------------------------------------------------------
        # Define the economizer secondary-side outlet as saturated vapor
        # at the intermediate (injection) pressure:
        #
        #   • mode: "PQ" ⇒ pressure + vapor quality
        #   • q = 1      ⇒ saturated vapor
        #
        # This state represents the vapor that will be injected into the
        # compression process via the vapor-injection port.
        self.economizer.state_two_phase_outlet = self.med_prop.calc_state("PQ", p_vapor_injection, 1)

        print(f"Before Injection prozess def calc_states in class VaporInjectionEconomizerDownstream")

        # -----------------------------------------------------------
        # Vapor-injection subsystem
        # -----------------------------------------------------------
        # Compute the injection-related quantities:
        #   • x_vapor_injection   → mass fraction of injected vapor (dimensionless)
        #   • h_vapor_injection   → enthalpy of the injected vapor
        #   • state_economizer_outlet → primary-side outlet of the economizer (state 7)
        #
        # This function internally performs the economizer energy balance,
        # the quality search on the secondary side, and enforces h7 constraints.
        x_vapor_injection, h_vapor_injection, state_economizer_outlet = self.calc_injection()

        print(f"after Injection prozess def calc_states in class VaporInjectionEconomizerDownstream")

        # -----------------------------------------------------------
        # Economizer: primary-side outlet assignment
        # -----------------------------------------------------------
        # The economizer primary outlet (state 7) is the upstream condition
        # feeding both expansion valves (LP and HP). It is returned from
        # calc_injection() after satisfying the subcooling split and matching
        # the enthalpy constraints (h7 consistency)
        self.economizer.state_outlet = state_economizer_outlet

        print(f"state_economizer_outlet: {state_economizer_outlet}")
        print(f"economizer.state_outlet: {self.economizer.state_outlet}")
        print(f"after setting economizer.state_outlet def calc_states in class VaporInjectionEconomizerDownstream")

        # -----------------------------------------------------------
        # Expansion valves (LP and HP) – inlet assignment
        # -----------------------------------------------------------
        # Both expansion valves take the economizer primary outlet (state 7)
        # as their inlet condition:
        #
        #   • LP-valve  → expands down to evaporator pressure p_1
        #   • HP-valve  → expands down to injection/intermediate pressure p_vapor_injection
        #
        # These two throttling processes generate:
        #   • A two-phase mixture for the evaporator (LP path)
        #   • A two-phase mixture that vaporizes inside the economizer (HP path)
        self.low_pressure_valve.state_inlet = state_economizer_outlet
        self.high_pressure_valve.state_inlet = state_economizer_outlet

        # -----------------------------------------------------------
        # High-pressure expansion valve (HP valve → injection line)
        # -----------------------------------------------------------
        # Perform the isenthalpic expansion from p_cond to p_vapor_injection.
        # This defines the secondary-side inlet of the economizer (state 5).
        #
        # The HP expansion produces a low-quality two-phase mixture that
        # evaporates inside the economizer, generating saturated or superheated
        # vapor for injection into the compressor.
        print(f"high_pressure_valve.state_inlet: {self.high_pressure_valve.state_inlet}")
        print(f"economizer.state_two_phase_inlet before HP valve calc_outlet: {self.economizer.state_two_phase_inlet}")
        self.high_pressure_valve.calc_outlet(p_outlet=p_vapor_injection)
        print(f"high_pressure_valve.state_outlet after calc_outlet: {self.high_pressure_valve.state_outlet}")
        # Feed the economizer with the secondary-side inlet (state 5)
        self.economizer.state_two_phase_inlet = self.high_pressure_valve.state_outlet
        print(f"economizer.state_two_phase_inlet after HP valve calc_outlet: {self.economizer.state_two_phase_inlet}")

        # -----------------------------------------------------------
        # Low-pressure expansion valve (LP valve → evaporator line)
        # -----------------------------------------------------------
        # Isenthalpic throttling from condensation level (state 7) to the
        # evaporator pressure p_1. This generates a two-phase mixture that
        # enters the evaporator (state 4).
        self.low_pressure_valve.calc_outlet(p_outlet=p_1)

        # Assign the evaporator inlet state (state 4)
        self.evaporator.state_inlet = self.low_pressure_valve.state_outlet



        # -----------------------------------------------------------
        # Ideal mixing process (LP-compressor discharge + injected vapor)
        # -----------------------------------------------------------
        # Compute the mixed enthalpy at the injection pressure. The mixture
        # consists of:
        #   • (1 - x_vi) * LP compressor discharge stream  (state 1_VI)
        #   • x_vi       * injected vapor from economizer  (state 6)
        #
        # This is an isobaric, ideal mixing process at p_vapor_injection,
        # assuming no additional heat transfer or pressure loss.
        h_1_VI_mixed = (
                (1 - x_vapor_injection) * self.low_pressure_compressor.state_outlet.h +
                x_vapor_injection * h_vapor_injection
        )

        # -----------------------------------------------------------
        # High-pressure compressor inlet (state 1_VI_mix)
        # -----------------------------------------------------------
        # Construct the thermodynamic state at the injection pressure using:
        #   • state type: "PH"  (pressure + enthalpy)
        #   • p = p_vapor_injection
        #   • h = h_1_VI_mixed
        #
        # This defines the suction condition for the high-pressure stage,
        # after the injection has thermodynamically mixed with the LP-stage
        # discharge.
        self.high_pressure_compressor.state_inlet = self.med_prop.calc_state(
            "PH", p_vapor_injection, h_1_VI_mixed
        )

        # -----------------------------------------------------------
        # High-pressure compressor outlet (state 2)
        # -----------------------------------------------------------
        # Compress the mixed injection state up to the condenser pressure p_2.
        # This produces the high-pressure discharge state (state 2).
        self.high_pressure_compressor.calc_state_outlet(
            p_outlet=p_2, inputs=inputs, fs_state=fs_state
        )
        # Assign HP compressor discharge as the inlet to the condenser (state 2 → state 3)
        self.condenser.state_inlet = self.high_pressure_compressor.state_outlet

        # -----------------------------------------------------------
        # Mass flow consistency check between LP and HP compressors
        # -----------------------------------------------------------
        # Assumption:
        #   x_vi = m_inj / m_high        (injected mass fraction at HP inlet)
        # Mass balance:
        #   m_high = m_low + m_inj
        # Therefore:
        #   m_low_should = (1 - x_vi) * m_high
        #
        # Compare LP compressor mass flow vs. theoretical value.
        m_flow_high = self.high_pressure_compressor.calc_m_flow(
            inputs=inputs, fs_state=fs_state
        )
        m_flow_low_should = m_flow_high * (1 - x_vapor_injection)
        percent_deviation = (m_flow_low - m_flow_low_should) / m_flow_low_should * 100
        logger.debug("Deviation of mass flow rates is %s percent", percent_deviation)

        # -----------------------------------------------------------
        # Propagate mass flow to condenser and economizer
        # -----------------------------------------------------------
        # The condenser and the economizer operate at the high-side mass
        # flow rate, which equals the high-pressure compressor discharge flow.
        self.condenser.m_flow = self.high_pressure_compressor.m_flow

        # High-side mass flow: both the condenser and economizer operate at m_high
        self.economizer.m_flow = self.high_pressure_compressor.m_flow

        # -----------------------------------------------------------
        # Update flowsheet state variables (fs_state)
        # -----------------------------------------------------------
        # Expose key thermodynamic states to the flowsheet for monitoring,
        # logging, and UI output. The temperatures assigned correspond to:
        #
        #   T_1 → Evaporator outlet
        #   T_2 → HP compressor discharge
        #   T_3 → Condenser outlet
        #   T_4 → Evaporator inlet
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

    def calc_injection(self):


        # Step size for quality (Q) at economizer two-phase inlet (state 5)
        _Q_economizer_twophase_inlet_step = 0.0001
        _min_step_Q_economizer_twophase_inlet_step = 0.0000000001
        Q_economizer_twophase_inlet_next = _min_step_Q_economizer_twophase_inlet_step
        print(f"calc_injection başı _Q_economizer_twophase_inlet_step: {_Q_economizer_twophase_inlet_step}")

        # Initial guess for two-phase inlet (state 5) at the injection pressure
        self.economizer.state_two_phase_inlet = self.med_prop.calc_state("PQ", self.economizer.state_two_phase_outlet.p, Q_economizer_twophase_inlet_next)
        print(f"initial economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")

        # Initial guess for primary-side outlet (state 7): set equal to condenser outlet
        # self.economizer.state_outlet = self.economizer.state_inlet     # initial guess
        print(f"initial economizer.state_outlet:          {self.economizer.state_outlet}")

        t = 0
        # Outer loop: iterate over quality Q at secondary inlet (state 5)
        while True:
            t += 1
            print(f"Outer loop iteration: {t}")

            if _Q_economizer_twophase_inlet_step < _min_step_Q_economizer_twophase_inlet_step:
                print(f"Breaking outer loop")
                break

            # Step control for vapor injection mass fraction x_vi
            _x_vi_step = 0.01
            _min_step_x_vi = 0.00000001
            x_vi_next = _min_step_x_vi  # Don't start from zero

            # Fix current Q guess for this outer iteration
            Q_economizer_twophase_inlet = Q_economizer_twophase_inlet_next
            print(f"Q_economizer_twophase_inlet: {Q_economizer_twophase_inlet}")
            # Update state 5 = (p_injection, Q_guess)
            self.economizer.state_two_phase_inlet = self.med_prop.calc_state("PQ", self.economizer.state_two_phase_outlet.p, Q_economizer_twophase_inlet)
            print(f"after calculate state economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
            # Base mass flow on evaporator (LP side)
            m_flow_evaporator = self.evaporator.m_flow

            # Enthalpy lift across two-phase side of economizer (state 5 → 6)
            dh_ihe_goal = (
                    self.economizer.state_two_phase_outlet.h -
                    self.economizer.state_two_phase_inlet.h
            )

            # Transport properties on primary side (liquid)
            tra_properties_liquid = self.med_prop.calc_transport_properties(
                self.economizer.state_inlet
            )
            alpha_liquid = self.economizer.calc_alpha_liquid(tra_properties_liquid)

            # Mean transport properties on secondary (two-phase) side
            tra_properties_two_phase = self.med_prop.calc_mean_transport_properties(
                self.economizer.state_two_phase_inlet,
                self.economizer.state_two_phase_outlet
            )
            alpha_two_phase = self.economizer.calc_alpha_liquid(tra_properties_two_phase)

            # ---------------------------------------------------------------
            # Effective cp on secondary side (used in NTU method)
            # ---------------------------------------------------------------
            dT_secondary = (
                    self.economizer.state_two_phase_outlet.T -
                    self.economizer.state_two_phase_inlet.T
            )
            if dT_secondary == 0:
                cp_4 = np.inf
            else:
                cp_4 = dh_ihe_goal / dT_secondary
            self.economizer.set_secondary_cp(cp=cp_4)
            primary_cp = tra_properties_liquid.cp

            # -----------------------------------------------------------
            # Inner loop: iterate vapor-injection fraction x_vi
            # Goal: find x_vi such that NTU heat transfer (Q_flow)
            #       matches the required enthalpy lift (Q_flow_goal)
            # -----------------------------------------------------------
            while True:
                x_vi = x_vi_next
                x_eva = 1 - x_vi

                # Mass-flow definitions for the upstream configuration
                m_flow_evaporator = self.evaporator.m_flow   #if you get h7<h5 at the beginning of iteration, then try to increase m_flow_evaporator here, for example 0.3
                m_flow_vapor_injection = (x_vi / (1-x_vi)) * m_flow_evaporator #vorsichtttttttttttt
                m_flow_sum = m_flow_evaporator + m_flow_vapor_injection

                # Target heat transfer on economizer secondary side
                Q_flow_goal = dh_ihe_goal * m_flow_vapor_injection

                # Assign primary & secondary mass flows
                self.economizer.m_flow = m_flow_sum
                self.economizer.m_flow_secondary = m_flow_vapor_injection

                # NTU heat-exchanger model
                k = self.economizer.calc_k(alpha_liquid, alpha_two_phase)
                Q_flow = ntu.calc_Q_ntu(
                    k=k,
                    dT_max=(
                            self.economizer.state_inlet.T -
                            self.economizer.state_two_phase_inlet.T  # Frage!
                    ),
                    A=self.economizer.A,
                    flow_type=self.economizer.flow_type,
                    m_flow_primary_cp=self.economizer.m_flow * primary_cp,
                    m_flow_secondary_cp=self.economizer.m_flow_secondary_cp
                )

                # print(f"Inner loop calc_injection x_vi: {x_vi}, Q_flow: {Q_flow}, Q_flow_goal: {Q_flow_goal}")
                # print(f"m_flow_primary: {self.economizer.m_flow}")
                # print(f"m_flow_secondary: {m_flow_vapor_injection}")
                # print(f"self.evaporator.m_flow: {self.evaporator.m_flow}")

                # Adjust x_vi until Q_flow ≈ Q_flow_goal
                if Q_flow > Q_flow_goal:
                    # Increase injection fraction
                    if abs(x_vi) >= 0.9 or x_vi < 0 :
                        break
                    if _x_vi_step <= _min_step_x_vi:
                        break
                    x_vi_next = x_vi + _x_vi_step


                else:
                    if abs(x_vi) >= 0.9 or x_vi <0 or x_eva <0:
                        break

                    # Decrease injection fraction and refine step
                    x_vi_next = x_vi - _x_vi_step * 0.9
                    _x_vi_step /= 10

            # -----------------------------------------------------------
            # Update economizer primary outlet (state 7)
            # Enthalpy balance: h7 = h3 - x_vi * (h6 - h5)
            # -----------------------------------------------------------
            h_7 = self.economizer.state_inlet.h - x_vi  * (self.economizer.state_two_phase_outlet.h - self.economizer.state_two_phase_inlet.h)
            self.economizer.state_outlet = self.med_prop.calc_state("PH", self.condenser.state_outlet.p, h_7)
            print(f"deneme")
            print(f"x_vi: {x_vi}")
            print(f"fark  Q Wärme: {Q_flow - Q_flow_goal}")
            print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            print(f"fark h7-h5: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")



#T iterationuna giden satırrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
            if self.economizer.state_outlet.h < self.economizer.state_two_phase_inlet.h:
                print(f" h7 < h5 Block, than h5 must be decreased, so Q = -1, calc methode f(p_inj, T")
                # self.economizer.state_two_phase_inlet.q = None
                _T_economizer_two_phase_inlet_step = 0.1
                _min_step_T_economizer_two_phase_inlet_step  = 0.0001
                T_initall_guess = self.economizer.state_two_phase_inlet.T - _min_step_T_economizer_two_phase_inlet_step
                print(f"economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
                print(f"Initial T_economizer_two_phase_inlet in h7<h5 block: {T_initall_guess}")
                T_economizer_two_phase_inlet_next = T_initall_guess
                self.economizer.state_two_phase_inlet = self.med_prop.calc_state("PT", self.economizer.state_two_phase_outlet.p, T_economizer_two_phase_inlet_next)
                print(f"initial economizer.state_two_phase_inlet in h7<h5 block: {T_economizer_two_phase_inlet_next}")
                print(f"economizer.state_two_phase_inlet after T-1 : {self.economizer.state_two_phase_inlet}")
                t_T_5 = 0
                while True:
                    t_T_5 +=1
                    print(f"h7 < h5 while loop iteration: {t_T_5}")
                    # if _Q_economizer_twophase_inlet_step < _min_step_Q_economizer_twophase_inlet_step:
                    #     print(f"Breaking outer loop")
                    #     break

                    # Step control for vapor injection mass fraction x_vi
                    _x_vi_step = 0.01
                    _min_step_x_vi = 0.00000001
                    x_vi_next = _min_step_x_vi  # Don't start from zero

                    T_economizer_two_phase_inlet = T_economizer_two_phase_inlet_next
                    print(f"T_economizer_two_phase_inlet: {T_economizer_two_phase_inlet}")
                    self.economizer.state_two_phase_inlet = self.med_prop.calc_state("PT", self.economizer.state_two_phase_outlet.p, T_economizer_two_phase_inlet)
                    print(f"after calculate state economizer.state_two_phase_inlet in h7<h5 block: {self.economizer.state_two_phase_inlet}")
                    # Base mass flow on evaporator (LP side)
                    m_flow_evaporator = self.evaporator.m_flow

                    # Enthalpy lift across two-phase side of economizer (state 5 → 6)
                    dh_ihe_goal = (
                            self.economizer.state_two_phase_outlet.h -
                            self.economizer.state_two_phase_inlet.h
                    )

                    # Transport properties on primary side (liquid)
                    tra_properties_liquid = self.med_prop.calc_transport_properties(
                        self.economizer.state_inlet
                    )
                    alpha_liquid = self.economizer.calc_alpha_liquid(tra_properties_liquid)

                    # Mean transport properties on secondary (two-phase) side
                    tra_properties_two_phase = self.med_prop.calc_mean_transport_properties(
                        self.economizer.state_two_phase_inlet,
                        self.economizer.state_two_phase_outlet
                    )
                    alpha_two_phase = self.economizer.calc_alpha_liquid(tra_properties_two_phase)

                    # ---------------------------------------------------------------
                    # Effective cp on secondary side (used in NTU method)
                    # ---------------------------------------------------------------
                    dT_secondary = (
                            self.economizer.state_two_phase_outlet.T -
                            self.economizer.state_two_phase_inlet.T
                    )
                    if dT_secondary == 0:
                        cp_4 = np.inf
                    else:
                        cp_4 = dh_ihe_goal / dT_secondary
                    self.economizer.set_secondary_cp(cp=cp_4)
                    primary_cp = tra_properties_liquid.cp

                    # -----------------------------------------------------------
                    # Inner loop: iterate vapor-injection fraction x_vi
                    # Goal: find x_vi such that NTU heat transfer (Q_flow)
                    #       matches the required enthalpy lift (Q_flow_goal)
                    # -----------------------------------------------------------
                    while True:
                        x_vi = x_vi_next
                        x_eva = 1 - x_vi

                        # Mass-flow definitions for the upstream configuration
                        m_flow_evaporator = self.evaporator.m_flow  # if you get h7<h5 at the beginning of iteration, then try to increase m_flow_evaporator here, for example 0.3
                        m_flow_vapor_injection = (x_vi / (1 - x_vi)) * m_flow_evaporator  # vorsichtttttttttttt
                        m_flow_sum = m_flow_evaporator + m_flow_vapor_injection

                        # Target heat transfer on economizer secondary side
                        Q_flow_goal = dh_ihe_goal * m_flow_vapor_injection

                        # Assign primary & secondary mass flows
                        self.economizer.m_flow = m_flow_sum
                        self.economizer.m_flow_secondary = m_flow_vapor_injection

                        # NTU heat-exchanger model
                        k = self.economizer.calc_k(alpha_liquid, alpha_two_phase)
                        Q_flow = ntu.calc_Q_ntu(
                            k=k,
                            dT_max=(
                                    self.economizer.state_inlet.T -
                                    self.economizer.state_two_phase_inlet.T  # Frage!
                            ),
                            A=self.economizer.A,
                            flow_type=self.economizer.flow_type,
                            m_flow_primary_cp=self.economizer.m_flow * primary_cp,
                            m_flow_secondary_cp=self.economizer.m_flow_secondary_cp
                        )

                        # Adjust x_vi until Q_flow ≈ Q_flow_goal
                        if Q_flow > Q_flow_goal:
                            # Increase injection fraction
                            if abs(x_vi) >= 0.9 or x_vi < 0:
                                break
                            if _x_vi_step <= _min_step_x_vi:
                                break
                            x_vi_next = x_vi + _x_vi_step


                        else:
                            if abs(x_vi) >= 0.9 or x_vi < 0 or x_eva < 0:
                                break

                            # Decrease injection fraction and refine step
                            x_vi_next = x_vi - _x_vi_step * 0.9
                            _x_vi_step /= 10


                    h_7 = self.economizer.state_inlet.h - x_vi * (self.economizer.state_two_phase_outlet.h - self.economizer.state_two_phase_inlet.h)
                    self.economizer.state_outlet = self.med_prop.calc_state("PH", self.condenser.state_outlet.p, h_7)
                    print(f"T iteration x_inj")
                    print(f"x_vi: {x_vi}")
                    print(f"fark Q Wärme: {Q_flow - Q_flow_goal}")
                    print(f"economizer.state_inlet.h: {self.economizer.state_inlet.h}")
                    print(f"economizer.state_two_phase_outlet.h: {self.economizer.state_two_phase_outlet.h}")
                    print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                    print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                    print(f"fark h7-h5: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                    print(f" h5 muss = {(self.economizer.state_inlet.h - 0.2 * self.economizer.state_two_phase_outlet.h) /(0.8) } olacak ki h7=h5 olsun")

                    if (self.economizer.state_two_phase_inlet.h - self.economizer.state_outlet.h) >  0.001:
                        # If Tstep becomes too small → stop
                        if _T_economizer_two_phase_inlet_step <= _min_step_T_economizer_two_phase_inlet_step:
                            print(f" minstep limit reached in T block")
                            print(f" returning...")
                            print(f"_T_economizer_twophase_inlet_step: {_T_economizer_two_phase_inlet_step}")
                            print(f"T_economizer_twophase_inlet_next: {T_economizer_two_phase_inlet_next}")
                            print(
                                f"_min_step_Q_economizer_twophase_inlet_step: {_min_step_T_economizer_two_phase_inlet_step}")
                            print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                            print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                            print(f" returning...")
                            return x_vi, self.economizer.state_two_phase_outlet.h, self.economizer.state_outlet

                        # decrease T guess
                        T_economizer_two_phase_inlet_next = T_economizer_two_phase_inlet - _T_economizer_two_phase_inlet_step
                        print(f"T iterationu T_economizer_twophase_inlet_next: {T_economizer_two_phase_inlet_next}")
                        print(f"deneme2 T block")
                    else:
                        # Close enough → refine Q step
                        print(f"else block {_T_economizer_two_phase_inlet_step} in T block")
                        print(f"fark h7-h5: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                        T_economizer_two_phase_inlet_next = T_economizer_two_phase_inlet + _T_economizer_two_phase_inlet_step * 0.9
                        _T_economizer_two_phase_inlet_step /= 10

                        # Stop if tolerance satisfied
                        if abs(self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) <= 0.001:
                            print(f"Breaking T block loop with h7>h5 condition met")
                            print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                            print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                            print(f"fark h7-h5: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                            print(f" returning...")
                            return x_vi, self.economizer.state_two_phase_outlet.h, self.economizer.state_outlet
            # -----------------------------------------------------------
            # -----------------------------------------------------------

            #T iteration ended here



            # -----------------------------------------------------------
            # Check whether h7 matches h5 within tolerance.
            # If not, adjust the quality Q at state 5 (outer loop).
            # -----------------------------------------------------------
            if self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h > 0.001:
                # If Q-step becomes too small → stop
                if _Q_economizer_twophase_inlet_step <= _min_step_Q_economizer_twophase_inlet_step:
                    print(f"_Q_economizer_twophase_inlet_step: {_Q_economizer_twophase_inlet_step}")
                    print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
                    print(f"_min_step_Q_economizer_twophase_inlet_step: {_min_step_Q_economizer_twophase_inlet_step}")
                    print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                    print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                    break

                # Increase Q guess
                Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet + _Q_economizer_twophase_inlet_step
                print(f"before deneme2 Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
                print(f"deneme2")
            # elif (self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) <0: #sıkıntı şu direkt burada başlıyor ve Qstepi çok düşürüyor sonra minstepsınırını aşıyor break atıyor
            #     #while self.economizer.state_outlet.h <! self.economizer.state_two_phase_inlet.h:  x_vi azalt
            #
            #     print(f"economizer.state_outlet: {self.economizer.state_outlet}")
            #     print(f"economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
            #     print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #     Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet - _Q_economizer_twophase_inlet_step * 0.9
            #     _Q_economizer_twophase_inlet_step /= 10
            #     print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            #     print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            #     print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            #     print(f"_Q_economizer_twophase_inlet_step düzeltme sonrası: {_Q_economizer_twophase_inlet_step}")
            #     print(f"Düzeltme sonrası Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")



            else:
                # Close enough → refine Q step
                print(f"else block {_Q_economizer_twophase_inlet_step}")
                print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet - _Q_economizer_twophase_inlet_step * 0.9
                _Q_economizer_twophase_inlet_step /= 10

                # Stop if tolerance satisfied
                if abs(self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) <= 0.001:
                    print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
                    print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
                    print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
                    print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
                    break
                if Q_economizer_twophase_inlet_next < 0:
                    print("Q < 0 hatası, sıfıra sabitliyorum1")
                    Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet + _Q_economizer_twophase_inlet_step

        print(
            f"break oluyor mu gerçekten? fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
        print(f"before return: ")
        print(f"self.economizer.state_outlet:          {self.economizer.state_outlet},")
        print(f"self.economizer.state_two_phase.inlet: {self.economizer.state_two_phase_inlet} ")
        print(f"now returning...")

            # Final return values:
            #   x_vi → optimal vapor injection fraction
            #   h6   → enthalpy of injected vapor
            #   state7 → economizer primary outlet
        return x_vi, self.economizer.state_two_phase_outlet.h, self.economizer.state_outlet



            # # Safety: ensure
            # # h7 >= h5
            # elif (self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h) < -100:  # sıkıntı şu direkt burada başlıyor ve Qstepi çok düşürüyor sonra minstepsınırını aşıyor break atıyor
            #     print(f" elif <-100 bloğu çalıştı")
            #     print(f"economizer.state_outlet: {self.economizer.state_outlet}")
            #     print(f"economizer.state_two_phase_inlet: {self.economizer.state_two_phase_inlet}")
            #     print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #     Q_economizer_twophase_inlet_next = Q_economizer_twophase_inlet - _Q_economizer_twophase_inlet_step * 0.9
            #     _Q_economizer_twophase_inlet_step /= 10
            #     print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            #     print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            #     print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            #     print(f"_Q_economizer_twophase_inlet_step düzeltme sonrası: {_Q_economizer_twophase_inlet_step}")
            #     print(f"Düzeltme sonrası Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #     if Q_economizer_twophase_inlet_next < 0:
            #         print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #         print(f"economizer.state_outlet.h: {self.economizer.state_outlet.h}")
            #         print(f"economizer.state_two_phase_inlet.h: {self.economizer.state_two_phase_inlet.h}")
            #         print(f"fark: {self.economizer.state_outlet.h - self.economizer.state_two_phase_inlet.h}")
            #         print(f"Q_economizer_twophase_inlet_next: {Q_economizer_twophase_inlet_next}")
            #         print("Q < 0 hatası, sıfıra sabitliyorum2")
            #         Q_economizer_twophase_inlet_next = 0

    def get_states_in_order_for_plotting(self):
        """
        Returns the thermodynamic states in cycle order for P–h / T–s plotting.
        The sequence follows the flow of the downstream vapor-injection cycle.
        """
        print(f"son mu?")
        print("\n==================== FLOW DEBUG (DOWNSTREAM VI CYCLE) ====================\n")

        # ---------------------------------------------------------
        # 1. CONDENSER OUTLET (STATE 3)
        # ---------------------------------------------------------
        print(f"[3] condenser_outlet: {self.condenser.state_outlet}")

        # ---------------------------------------------------------
        # 2. ECONOMIZER PRIMARY (3 → 7)
        # ---------------------------------------------------------
        print(f"[3] economizer_primary_inlet: {self.economizer.state_inlet}")
        print(f"[7] economizer_primary_outlet: {self.economizer.state_outlet}")

        # ---------------------------------------------------------
        # 3. ECONOMIZER SECONDARY (5 → 6)  *** <-- senin istediğin yer burası
        # ---------------------------------------------------------
        print(f"[5] economizer_twophase_inlet: {self.economizer.state_two_phase_inlet}")
        print(f"[6] economizer_twophase_outlet (injected vapor): {self.economizer.state_two_phase_outlet}")

        # ---------------------------------------------------------
        # 4. SPLITTER (state 7 → LP EV + HP EV)
        # ---------------------------------------------------------
        print(f"[7→4] lp_valve_inlet: {self.low_pressure_valve.state_inlet}")
        print(f"[7→5] hp_valve_inlet: {self.high_pressure_valve.state_inlet}")

        # ---------------------------------------------------------
        # 5. LP EXPANSION VALVE (7 → 4)
        # ---------------------------------------------------------
        print(f"[4] lp_valve_outlet (evaporator_inlet): {self.low_pressure_valve.state_outlet}")

        # ---------------------------------------------------------
        # 6. EVAPORATOR (4 → 1)
        # ---------------------------------------------------------
        print(f"[4] evaporator_inlet: {self.evaporator.state_inlet}")
        print(f"[1] evaporator_outlet: {self.evaporator.state_outlet}")

        # ---------------------------------------------------------
        # 7. LP COMPRESSOR (1 → 1_VI)
        # ---------------------------------------------------------
        print(f"[1] lp_compressor_inlet: {self.low_pressure_compressor.state_inlet}")
        print(f"[1_VI] lp_compressor_outlet (before mixing): {self.low_pressure_compressor.state_outlet}")

        # ---------------------------------------------------------
        # 8. INJECTION MIXING (1_VI + 6)
        # ---------------------------------------------------------
        print(f"[mix] injection_mixed_state (hp_compressor_inlet): {self.high_pressure_compressor.state_inlet}")

        # ---------------------------------------------------------
        # 9. HIGH-PRESSURE COMPRESSOR (mix → 2)
        # ---------------------------------------------------------
        print(f"[2] hp_compressor_outlet: {self.high_pressure_compressor.state_outlet}")


        # print(f"m_flow_low: {m_flow_low}")
        # print(f"m_flow_low_should: {m_flow_low_should}")
        # print(f"percent_deviation: {percent_deviation}")
        # print(f"m_flow_high: {m_flow_high}")
        # print(f"m_sum_flow_economizer: {self.economizer.m_flow}")
        # print(f"x_vapor_injection: {x_vapor_injection}")
        # print(f"check x_vapor_injection m_flow_low/m_flow_high: {m_flow_low / m_flow_high}")

        print("\n==================== END FLOW DEBUG ====================\n")

        return  [
            self.low_pressure_valve.state_inlet,  # state 7  (econ primary outlet → LP valve)
            self.low_pressure_valve.state_outlet, # state 4  (LP valve outlet → evaporator inlet)
            self.evaporator.state_inlet,                                        # state 4  (evaporator inlet, identical)
            self.med_prop.calc_state("PQ", self.evaporator.state_outlet.p, 1),  # saturated vapor line in evaporator
            self.evaporator.state_outlet,                                       # state 1  (evaporator outlet)
            self.low_pressure_compressor.state_inlet,  # state 1
            self.low_pressure_compressor.state_outlet,  # state 1_VI (LP compressor discharge)
            self.high_pressure_compressor.state_inlet,  # state 1_VI_mixed (after vapor injection)
            self.high_pressure_compressor.state_outlet,  # state 2
            self.condenser.state_inlet,  # state 2
            self.med_prop.calc_state("PQ", self.condenser.state_outlet.p, 1),  # saturated vapor in condenser
            self.med_prop.calc_state("PQ", self.condenser.state_outlet.p, 0),  # saturated liquid in condenser
            self.condenser.state_outlet,  # state 3
            self.economizer.state_inlet,  # state 3
            self.economizer.state_outlet, # state 5
            self.high_pressure_valve.state_inlet,             # state 7
            self.high_pressure_valve.state_outlet,     # state 7 (path to HP EV)
            self.economizer.state_two_phase_inlet,    # state 5
            self.economizer.state_two_phase_outlet,   # state 6
            self.high_pressure_compressor.state_inlet,
            # Go back to the condenser outlet
            self.economizer.state_two_phase_outlet,  # state 6
            self.economizer.state_two_phase_inlet,  # state 5
            self.high_pressure_valve.state_outlet,  # state 5
            self.high_pressure_valve.state_inlet,  # state 3
            self.economizer.state_inlet,  # state 3
            self.economizer.state_outlet,  # state 7 (path to the evaporator)
            self.condenser.state_outlet,  # state 3 (path back to the splitting point)
        ]
