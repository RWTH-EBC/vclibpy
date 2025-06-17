from vclibpy.flowsheets import StandardCycle

class InternalHeatExchangerCycle(StandardCycle):
    flowsheet_name = "InternalHeatExchanger"

    def __init__(
            self,
            compressor,
            expansion_valve,
            internal_heat_exchanger,
            **kwargs
    ):
        super().__init__(compressor=compressor, expansion_valve=expansion_valve, **kwargs)
        self.internal_heat_exchanger = internal_heat_exchanger

    def get_all_components(self):
        return super().get_all_components() + [
            self.internal_heat_exchanger
        ]

    def calc_states(self, p_1, p_2, inputs, fs_state):
        # 1. evaporator outlet: q=1, p=p_1 (no superheat in evaporator)
        state_eva_out = self.med_prop.calc_state("PQ", p_1, 1)
        self.evaporator.state_outlet = state_eva_out

        # 2. IHX cold side: superheat to assumed value
        T_ihx_gas_out = state_eva_out.T + inputs.dT_eva_superheating
        state_ihx_gas_out = self.med_prop.calc_state("PT", p_1, T_ihx_gas_out)
        self.internal_heat_exchanger.state_gas_inlet = state_eva_out
        self.internal_heat_exchanger.state_gas_outlet = state_ihx_gas_out

        # 3. compressor:
        self.compressor.state_inlet = state_ihx_gas_out
        self.compressor.calc_state_outlet(p_outlet=p_2, inputs=inputs, fs_state=fs_state)

        # 4. condenser (with assumed subcooling)
        self.condenser.state_inlet = self.compressor.state_outlet
        self.set_condenser_outlet_based_on_subcooling(p_con=p_2, inputs=inputs)
        state_con_out = self.condenser.state_outlet
        self.condenser.state_q1 = self.med_prop.calc_state("PQ", p_2, 1)
        self.condenser.state_q0 = self.med_prop.calc_state("PQ", p_2, 0)

        # 5. IHX hot side: additional subcooling by IHX
        # calculate specific enthalpy difference, required for superheat in IHX cold side
        delta_h_superheat = self.internal_heat_exchanger.state_gas_outlet.h - self.internal_heat_exchanger.state_gas_inlet.h
        # calculate liquid outlet state of IHX
        h_ihx_liq_out = state_con_out.h - delta_h_superheat
        state_ihx_liq_out = self.med_prop.calc_state("PH", p_2, h_ihx_liq_out)
        self.internal_heat_exchanger.state_liquid_inlet = state_con_out
        self.internal_heat_exchanger.state_liquid_outlet = state_ihx_liq_out

        # 6. expansionvalve: isenthalp to p_1
        self.expansion_valve.state_inlet = state_ihx_liq_out
        self.expansion_valve.calc_outlet(p_outlet=p_1)
        self.evaporator.state_inlet = self.expansion_valve.state_outlet

        # 7. massflow calculation:
        self.compressor.calc_m_flow(inputs=inputs, fs_state=fs_state)
        self.condenser.m_flow = self.compressor.m_flow
        self.evaporator.m_flow = self.compressor.m_flow
        self.expansion_valve.m_flow = self.compressor.m_flow
        self.internal_heat_exchanger.m_flow = self.compressor.m_flow

        # Logging to fs_state (optional, similar to StandardCycle)
        fs_state.set(
            name="T_ihx_gas_out", value=state_ihx_gas_out.T,
            unit="K", description="T_IHX cold side outlet"
        )
        fs_state.set(
            name="T_ihx_liq_out", value=state_ihx_liq_out.T,
            unit="K",description="T_IHX hot side outlet"
        )

    def get_states_in_order_for_plotting(self):
        return [
            self.evaporator.state_inlet,  # evaporator inlet
            self.evaporator.state_outlet,  # evaporator outlet
            self.internal_heat_exchanger.state_gas_outlet,  # cold side IHX outlet
            self.compressor.state_outlet,  # compressor outlet
            self.condenser.state_q1,  # condenser entirely vapor,
            self.condenser.state_q0,  # condenser entirely liquid
            self.condenser.state_outlet,  # condenser outlet
            self.internal_heat_exchanger.state_liquid_outlet,  # hot side IHX outlet
            self.expansion_valve.state_outlet,  # expansion valve outlet
        ]