from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)

import CoolProp.CoolProp as CP_HumidAir


class ThermoModel:
    """
    # TODO Docstring
    """
    
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

        # ================= Get State Values =================
        Q_dot_total = state.hmt.Q_dot_total
        m_dot_frost_total = state.hmt.m_dot_frost_total

        W_in = state.air.W_in
        m_dot_dry_air = state.air.m_dot_dry
        h_in_air = state.air.h_in
        h_ice = state.air.h_ice

        T_in_air = inputs.air.T_in
        p_out_air = self.params.ambient_pressure


        # ================= Calculate new Values =================
        # h_refrigerant_out = self._calculate_refrigerant_outlet_enthalpy(
        #     Q_dot_total=state.hmt.Q_dot_total,
        #     m_dot_refrigerant=inputs.refrigerant.m_dot,
        #     h_in_refrigerant=inputs.refrigerant.h_in,
        # )

        W_out_air = self._calculate_outlet_humidity_ratio(
            W_in=W_in,
            m_dot_dry_air=m_dot_dry_air,
            m_dot_frost_total=m_dot_frost_total,
        )

        T_out_air = self._calculate_air_outlet_temperature(
            Q_dot_total=Q_dot_total,
            m_dot_dry_air=m_dot_dry_air,
            h_in_air=h_in_air,
            m_dot_frost_total=m_dot_frost_total,
            h_ice=h_ice,
            p_out_air=p_out_air,
            W_out_air=W_out_air,
            T_in_air=T_in_air
        )


        # ================= Write new values to state =================
        # state.refrigerant.set("h_out", h_refrigerant_out)
        state.air.set("W_out", W_out_air)
        state.air.set("T_out", T_out_air)



    def _calculate_refrigerant_outlet_enthalpy(self, Q_dot_total: float, m_dot_refrigerant: float, h_in_refrigerant: float) -> float:
        """
        Calculates the outlet enthalpy of the refrigerant using an energy balance.
        Args:
            Q_dot_total: Heat transfer rate (W)
            m_dot_refrigerant: Mass flow rate of refrigerant (kg/s)
            h_in_refrigerant: Inlet enthalpy of refrigerant (J/kg)
        Returns:
            h_out_refrigerant: Outlet enthalpy of refrigerant (J/kg)
        """
        return h_in_refrigerant + Q_dot_total / m_dot_refrigerant
    

    def _calculate_air_outlet_temperature(self, Q_dot_total: float, m_dot_dry_air: float, h_in_air: float, m_dot_frost_total:float, h_ice:float, p_out_air:float, W_out_air, T_in_air: float) -> float:
        """
        Calculates the outlet air temperature based on the energy balance.
        
        Energy balance on air (control volume):
        m_da*h_in = m_da*h_out + Q_dot_total + m_frost*h_ice
        
        Solving for h_out:
        h_out = h_in - (Q_dot_total + m_frost*h_ice) / m_da

        Then, T_out is found from h_out, p_out, and W_out.
        """
        # Calculate outlet enthalpy [J/kg_dry_air]
        h_out_air = h_in_air - (Q_dot_total + m_dot_frost_total * h_ice) / m_dot_dry_air

        # Safety Check: Enthalpie darf nicht unrealistisch tief fallen
        # Überschlägige Min-Enthalpie bei -50°C trockener Luft ca -50000 J/kg
        if h_out_air < -100000: 
            # Fallback, um Crash zu verhindern, bis sich die Iteration fängt
            print("Error, Air outlet Enthalpy to small!")
            return T_in_air

        try:
            T_out_air = CP_HumidAir.HAPropsSI('T', 'H', h_out_air, 'P', p_out_air, 'W', W_out_air)
        except ValueError:
            # Fallback bei CoolProp Fehler (z.B. wenn State unmöglich ist)
            # Wir nehmen an, dass wir auf der Sättigungslinie liegen
            # Dies ist nur eine Rettungsmaßnahme für numerische Stabilität
            print(f"Warning: CoolProp fail at H={h_out_air}, W={W_out_air}. Using approx.")
            # Sehr grobe Näherung: cp_air ca 1006
            T_out_air = (h_out_air / 1006.0) + 273.15 
            
        return T_out_air

    

    def _calculate_outlet_humidity_ratio(self, W_in: float, m_dot_dry_air: float, m_dot_frost_total: float) -> float:
        """
        Calculates the outlet humidity ratio based on a water vapor mass balance.

        The mass balance is:
        m_dot_water_in = m_dot_water_out + m_dot_frost
        (m_dot_dry_air * W_in) = (m_dot_dry_air * W_out) + (m_dot_frost_total)

        Solving for W_out gives:
        W_out = W_in - (m_dot_frost_total) / m_dot_dry_air

        Args:
            W_in: Inlet humidity ratio (kg_water/kg_dry_air)
            m_dot_dry_air: Mass flow rate of dry air (kg/s)
            m_dot_frost_total: Mass flow of frost (rate of water removal) (kg/s)

        Returns:
            float: Outlet humidity ratio (W_out) (kg_water/kg_dry_air)
        """
        # Calculate the change in humidity ratio
        delta_W = m_dot_frost_total / m_dot_dry_air  # (kg_water/s) / (kg_dry_air/s) = kg_water/kg_dry_air

        # Calculate the outlet humidity ratio
        W_out = W_in - delta_W

        if W_out < 0:
            W_out = 0.0
        return W_out