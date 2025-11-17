from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np


class HeatMassTransferModel:
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
        Calculates and updates ...
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # Calculate effective area (with fin efficiency) for total thermal resistance
        A_effective, h_effective = self._calculate_effective_area_and_convection(
            h_conv_air=state.air.h_conv,
            frost_thickness = state.frost.thickness,
            k_frost=state.frost.k_frost
        )

        R_total = self._calculate_total_thermal_resistance(
            h_effective=h_effective,
            h_conv_refrigerant=state.refrigerant.h_conv,
            A_effective=A_effective
        )

        delta_T_log = self._calculate_log_temperature_difference(
            T_air_in=inputs.air.T_in,
            T_air_out=state.air.T_out,
            T_refrigerant_in=state.refrigerant.T_in,
            T_refrigerant_out=state.refrigerant.T_out
        )

        Q_dot = self._calculate_heat_transfer_rate(
            delta_T_log=delta_T_log,
            R_total=R_total
        )

        m_dot_frost_flux = self._calculate_m_dot_frost_flux(
            betta=state.air.betta,
            rho_w_avg=state.air.rho_w_avg,
            rho_w_frost_sat=state.air.rho_w_frost_sat
        )

        # Calculate total geometric frost surface area (no fin efficiency) for T_frost_surface
        A_frost_surface = self._calculate_frost_surface_area(
            tube_diameter_w_frost=state.frost.tube_diameter_w_frost,
            space_between_frost=state.frost.space_between_frost
        )

        T_frost_surface = self._calculate_frost_surface_temperature(
            T_air_in=inputs.air.T_in,
            T_air_out=state.air.T_out,
            Q_dot=Q_dot,
            h_conv_air=state.air.h_conv,
            A_frost_surface=A_frost_surface
        )





        state.hmt.set("A_effective", A_effective)
        state.hmt.set("A_frost_surface", A_frost_surface)
        state.hmt.set("R_total", R_total)
        state.hmt.set("delta_T_log", delta_T_log)
        state.hmt.set("T_frost_surface", T_frost_surface)
        state.hmt.set("Q_dot", Q_dot)
        state.hmt.set("m_dot_frost_flux", m_dot_frost_flux)
        




    def _calculate_heat_transfer_rate(self, delta_T_log: float, R_total: float) -> float:
        """
        Calculates the heat transfer rate between air and refrigerant.
        """
        return delta_T_log / R_total
    
    def _calculate_m_dot_frost_flux(self, betta: float, rho_w_avg: float, rho_w_frost_sat: float) -> float:
        """
        Calculates the mass flux of frost formation on the evaporator surface.
        """
        if betta < 0:
            raise ValueError("betta cannot be negative.")
        if rho_w_avg < rho_w_frost_sat:
            raise ValueError("rho_w_avg must be greater than or equal to rho_w_frost_sat.") 

        return betta * (rho_w_avg - rho_w_frost_sat)
    

    def _calculate_frost_surface_temperature(self, T_air_in: float, T_air_out:float, Q_dot: float, h_conv_air: float, A_frost_surface: float) -> float:
        """
        Calculates the frost surface temperature based on air temperature,
        heat transfer rate, convective heat transfer coefficient, and effective area.
        """

        if np.isclose(h_conv_air, 0):
            raise ValueError("h_conv_air cannot be zero.")
        if np.isclose(A_frost_surface, 0):
            raise ValueError("A_frost_surface cannot be zero.")

        T_air_avg = 0.5 * (T_air_in + T_air_out)

        return T_air_avg - Q_dot / (h_conv_air * A_frost_surface)

    def _calculate_log_temperature_difference(self, T_air_in: float, T_air_out: float, T_refrigerant_in: float, T_refrigerant_out: float) -> float:
        """
        Calculates the log mean temperature difference (LMTD) between air and refrigerant.
        This is a simplification, as LMTD is strictly valid for counterflow heat exchangers only.
        """

        delta_T1 = T_air_in - T_refrigerant_out
        delta_T2 = T_air_out - T_refrigerant_in

        if delta_T1 <= 0 or delta_T2 <= 0:
            print("T_air_in:", T_air_in)
            print("T_air_out:", T_air_out)
            print("T_refrigerant_in:", T_refrigerant_in)
            print("T_refrigerant_out:", T_refrigerant_out)
            raise ValueError("Temperature differences must be positive for LMTD calculation.")

        if np.isclose(delta_T1, delta_T2):
            return delta_T1
        
        return (delta_T1 - delta_T2) / np.log(delta_T1 / delta_T2)


    def _calculate_total_thermal_resistance(self, h_effective:float, h_conv_refrigerant:float, A_effective:float) -> float:
        """
        Calculates and returns the total thermal resistance.
        """
        R_air_frost = self._calculate_thermal_resistance_air_frost(h_effective, A_effective)
        R_tube = self._calculate_thermal_resistance_tube()
        R_refrigerant = self._calculate_thermal_resistance_refrigerant(h_conv_refrigerant)

        return R_air_frost + R_tube + R_refrigerant
    
    def _calculate_thermal_resistance_air_frost(self, h_effective: float, A_effective:float) -> float:
        """
        Calculates the convective thermal resistance on the air side + frost.
        """
        if np.isclose(h_effective, 0):
            raise ValueError("h_effective cannot be zero.")
        if np.isclose(A_effective, 0):
            raise ValueError("A_effective cannot be zero.")
        return 1 / (h_effective * A_effective)

    
    def _calculate_thermal_resistance_tube(self) -> float:
        """
        Calculates the conductive thermal resistance through the tube wall.
        """

        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        if np.isclose(self.params.tube_thermal_conductivity, 0):
            raise ValueError("Tube conductivity cannot be zero.")
        
        return ( np.log(self.params.tube_outer_diameter / self.params.tube_inner_diameter) / 
                 (2 * np.pi * self.params.total_tube_length * self.params.tube_thermal_conductivity) )
    
    def _calculate_thermal_resistance_refrigerant(self, h_conv_refrigerant: float) -> float:
        """
        Calculates the convective thermal resistance on the refrigerant side.
        """

        if np.isclose(h_conv_refrigerant, 0):
            raise ValueError("h_conv_refrigerant cannot be zero.")
        if np.isclose(self.params.tube_inner_diameter, 0) or np.isclose(self.params.total_tube_length, 0):
            raise ValueError("Tube dimensions cannot be zero.")
        
        return 1 / (h_conv_refrigerant * np.pi * self.params.tube_inner_diameter * self.params.total_tube_length)


    def _calculate_effective_area_and_convection(self, h_conv_air: float, frost_thickness:float, k_frost:float) -> tuple[float, float]:
        """
        Calculates the effective heat transfer area of the finned tube.
        This function works for "Fluchtende Rohre" only.
        Args:
            h_conv_air (float): The convective heat transfer coefficient of the air [W/m²K].
            k_frost (float): The thermal conductivity of the frost layer [W/mK].
        Returns:
            float: The effective heat transfer area [m²].
            float: The effective heat transfer coefficient of air and frost [W/m²K].

        """

        # Calculate the heat transfer Coefficient of air and frost in series
        h_effective = self._calculate_effective_heat_transfer_coefficient(h_conv_air, frost_thickness,  k_frost)


        # Calculate fin efficiency
        eta_fin= self._calculate_fin_efficiency(h_effective)

        # Calculate area of one fin and one tube segment
        A_one_tube_segment = np.pi * self.params.tube_outer_diameter * (self.params.fin_pitch - self.params.fin_thickness)
        A_one_fin_segment  = 2 * ( (self.params.fin_segment_height * self.params.fin_segment_length) - (0.25 * np.pi * self.params.tube_outer_diameter**2) )


        A_effective = self.params.fin_segment_amount * (A_one_tube_segment + eta_fin * A_one_fin_segment)

        return A_effective, h_effective

    def _calculate_frost_surface_area(self, tube_diameter_w_frost:float, space_between_frost:float)-> float:
        """
        Calculates the total frost surface area on the finned tube evaporator.

        Args:
            tube_diameter_w_frost (float): The effective tube outer diameter including frost [m].
            space_between_frost (float): The air flow channel width between frosted fins including frost [m].

        Returns:
            float: The total frost surface area [m²].
        """

        # Calculate area of one fin and one tube segment
        A_one_tube_segment_frost = np.pi * tube_diameter_w_frost * space_between_frost
        A_one_fin_segment_frost  = 2 * ( (self.params.fin_segment_height * self.params.fin_segment_length) - (0.25 * np.pi * tube_diameter_w_frost**2) )

        return self.params.fin_segment_amount * (A_one_tube_segment_frost + A_one_fin_segment_frost)

    def _calculate_fin_efficiency(self, h_effective: float) ->float:
        # sourcery skip: assign-if-exp, reintroduce-else
        """
        Calculates the efficiency of a rectangular fin.

        The calculation procedure is based on the VDI Wärmeatlas,
        "M1 Wärmeübergang an berippten Rohren".

        Args:
            h_effective (float): The combined heat transfer coefficient of air and frost [W/m²K].

        Returns:
            float: Fin efficiency (eta_fin), dimensionless.
        """
        
        if np.isclose(self.params.tube_outer_diameter, 0):
            raise ValueError("tube_outer_diameter cannot be zero.")

            
        if np.isclose(self.params.fin_thickness, 0):
            raise ValueError("fin_thickness cannot be zero.")
        
        if np.isclose(self.params.fin_thermal_conductivity, 0):
            raise ValueError("fin_thermal_conductivity cannot be zero.")
        
        if np.isclose(self.params.fin_segment_length, 0) or np.isclose(self.params.fin_segment_height, 0):
            raise ValueError("fin_segment_length and fin_segment_height cannot be zero.")
            

        # Equation (13) from VDI Wärmeatlas M1
        if self.params.fin_segment_height > self.params.fin_segment_length:
            raise ValueError("Fin segment height cannot be greater than fin segment length.")


        # Equation (13) from VDI Wärmeatlas M1
        phi_dash = (1.28 * self.params.fin_segment_height / self.params.tube_outer_diameter *
                    np.sqrt((self.params.fin_segment_length / self.params.fin_segment_height) - 0.2))

        # Equation (12) from VDI Wärmeatlas M1
        phi = (phi_dash - 1) * (1 + 0.35 * np.log(phi_dash))

        # Equation (8) from VDI Wärmeatlas M1
        term_in_sqrt = (2 * h_effective) / (self.params.fin_thermal_conductivity * self.params.fin_thickness)
        X = phi * (self.params.tube_outer_diameter / 2) * np.sqrt(term_in_sqrt)

        # Equation (7) and (9) from VDI Wärmeatlas M1
        # To avoid division by zero if X is very close to 0,
        # we use the limit of tanh(X)/X as X->0, which is 1.
        if X < 1e-6: return 1.0
        
        return np.tanh(X) / X


    def _calculate_effective_heat_transfer_coefficient(self, h_conv_air:float, frost_thickness:float, k_frost:float) -> float:
        """
        Calculates the combined heat transfer coefficient
        for convection from air through a layer of frost.

        This treats the convective and conductive layers as thermal
        resistances in series.

        Args:
            h_conv_air: The convective heat transfer coefficient of the air [W/m²K].
            frost_thickness: The thickness of the frost layer [m].
            k_frost: The thermal conductivity of the frost layer [W/mK].
            

        Returns:
            The combined heat transfer coefficient of air and frost [W/m²K].
        """        
            # Handle no-frost case
        if np.isclose(frost_thickness, 0):
            return h_conv_air

        # Check for zero denominators
        if np.isclose(h_conv_air, 0):
            raise ValueError("h_conv_air cannot be zero.") 
        if np.isclose(k_frost, 0):
            raise ValueError("k_frost cannot be zero.") 

        return 1 / (1 / h_conv_air + frost_thickness / k_frost)
    
