from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)

import CoolProp.CoolProp as CP_HumidAir


class ThermoModel:
    
    def __init__(self, 
                 parameters: FrostEvaporatorParameters, 
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
        """
        self.params = parameters

    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the thermodynamics model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """


        # ================= Calculate new Values =================
        h_refrigerant_out = self._calculate_refrigerant_outlet_enthalpy(
            Q_dot_total=state.hmt.Q_dot_total,
            m_dot_refrigerant=inputs.refrigerant.m_dot,
            h_in_refrigerant=inputs.refrigerant.h_in,
        )

        W_out_air = self._calculate_outlet_humidity_ratio(
            W_in              = inputs.air.W_in,
            m_dot_dry_air     = state.air.m_dot_dry,
            m_dot_frost_total = state.hmt.m_dot_frost_total,
        )

        T_out_air = self._calculate_air_outlet_temperature(
            Q_dot_total       = state.hmt.Q_dot_total,
            m_dot_dry_air     = state.air.m_dot_dry,
            h_in_air          = state.air.h_in,
            m_dot_frost_total = state.hmt.m_dot_frost_total,
            h_ice             = state.air.h_ice,
            p_out_air         = state.air.p_out,
            W_out_air         = W_out_air,
            T_in_air          = inputs.air.T_in
        )


        # ================= Write new values to state =================
        state.refrigerant.set("h_out", h_refrigerant_out)
        state.air.set("W_out", W_out_air)
        state.air.set("T_out", T_out_air)






    ####################################################################################
    # Helper Functions
    ####################################################################################
    def _calculate_refrigerant_outlet_enthalpy(self, Q_dot_total: float, m_dot_refrigerant: float, h_in_refrigerant: float) -> float:
        """
        Calculates the outlet enthalpy of the refrigerant using an energy balance.
        
        Args:
            Q_dot_total: Heat transfer rate (W).
            m_dot_refrigerant: Mass flow rate of refrigerant (kg/s).
            h_in_refrigerant: Inlet enthalpy of refrigerant (J/kg).
        Returns:
            h_out_refrigerant: Outlet enthalpy of refrigerant (J/kg).
        """
        return h_in_refrigerant + Q_dot_total / m_dot_refrigerant

    def _calculate_outlet_humidity_ratio(self, W_in: float, m_dot_dry_air: float, m_dot_frost_total: float) -> float:
        """
        Calculates the outlet humidity ratio based on a water vapor mass balance.

        The mass balance is:
        m_dot_water_in = m_dot_water_out + m_dot_frost
        (m_dot_dry_air * W_in) = (m_dot_dry_air * W_out) + (m_dot_frost_total)

        Args:
            W_in: Inlet humidity ratio (kg_water/kg_dry_air).
            m_dot_dry_air: Mass flow rate of dry air (kg/s).
            m_dot_frost_total: Mass flow of frost (rate of water removal) (kg/s).
        Returns:
            float: Outlet humidity ratio (W_out) (kg_water/kg_dry_air).
        """
        # Calculate the change in humidity ratio
        delta_W = m_dot_frost_total / m_dot_dry_air 

        # Calculate the outlet humidity ratio
        W_out = W_in - delta_W

        return 0.0 if W_out < 0 else W_out

    def _calculate_air_outlet_temperature(self, Q_dot_total: float, m_dot_dry_air: float, h_in_air: float, 
                                          m_dot_frost_total: float, h_ice: float, p_out_air: float, 
                                          W_out_air: float, T_in_air: float) -> float:
        """
        Calculates the outlet air temperature based on the energy balance.
        
        Energy balance on air (control volume):
        m_da*h_in = m_da*h_out + Q_dot_total + m_frost*h_ice
        
        Args:
            Q_dot_total: Total heat transfer rate [W].
            m_dot_dry_air: Mass flow rate of dry air [kg/s].
            h_in_air: Inlet air enthalpy [J/kg].
            m_dot_frost_total: Total frost mass flow rate [kg/s].
            h_ice: Enthalpy of ice/frost [J/kg].
            p_out_air: Outlet air pressure [Pa].
            W_out_air: Outlet humidity ratio [kg/kg].
            T_in_air: Inlet air temperature (used for fallback/error context) [K].
        Returns:
            The calculated outlet air temperature [K].
        Raises:
            ValueError: If calculated enthalpy is physically impossible or CoolProp fails.
        """
        # Calculate outlet enthalpy [J/kg_dry_air]
        h_out_air = h_in_air - (Q_dot_total + m_dot_frost_total * h_ice) / m_dot_dry_air

        try:
            T_out_air = CP_HumidAir.HAPropsSI('T', 'H', h_out_air, 'P', p_out_air, 'W', W_out_air)
        except ValueError:
            # If CoolProp fails, we raise an error to stop the solver from using bad physics.
            # (Alternatively, you can implement the linear approximation fallback here if strictly required)
            raise ValueError(f"CoolProp failed to solve for T at H={h_out_air}, P={p_out_air}, W={W_out_air}")
            
        return T_out_air