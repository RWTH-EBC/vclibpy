from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import CoolProp.CoolProp as CP_HumidAir
from scipy.optimize import brentq

class AirModel:
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

        # Conversion of relative humidity to absolute humidity
        W_in = CP_HumidAir.HAPropsSI(
            'W', 
            'T', inputs.air.T_in, 
            'P', self.params.ambient_pressure, 
            'R', inputs.air.R_in
        )

        # Calculate average air properties between inlet and outlet
        pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, thermal_conductivity_avg, prandtl_avg, lewis_avg, rho_w_in, rho_w_out, h_in, h_out = self._calculate_air_averages(
            T_in=inputs.air.T_in,
            p_in=state.air.p_in,
            W_in=W_in,
            T_out=state.air.T_out,
            p_out=self.params.ambient_pressure,
            W_out=state.air.W_out
        )

        # Calculate water vapor density at frost surface (saturated)
        rho_w_frost_sat, W_frost_sat = self._get_water_vapor_density(
            T=state.hmt.T_frost_surface, 
            p=pressure_avg, 
            R=1.0  # R=1.0 for fully saturated air
        )

        space_between_frost = state.frost.space_between_frost
        flow_area_air = state.frost.flow_area_air


        pressure_loss_coeff = self._calculate_pressure_loss_coefficient_haaf(
            reynolds=state.air.reynolds,
            space_between_frost=space_between_frost,
            fin_length=self.params.fin_length
        )

        velocity, delta_p = self._calculate_velocity_and_pressure_drop(
            flow_area_air=flow_area_air,
            density=density_avg,
            K=self.params.pressure_loss_fit_factor,
            dyn_viscosity=dyn_viscosity_avg,
            space_between_frost=space_between_frost,
            fin_length=self.params.fin_length
        )

        p_in = self.params.ambient_pressure + delta_p
        characteristic_length = 2*space_between_frost  # Hydraulic diameter Dh = 2 * s

        reynolds = self._calculate_reynolds_number(
            density=density_avg,
            velocity=velocity,
            characteristic_length=characteristic_length,
            dyn_viscosity=dyn_viscosity_avg
        )

        nusselt = self._calculate_nusselt(
            Re_Dh=reynolds, 
            Pr=prandtl_avg, 
            Dh=characteristic_length, 
            L=self.params.fin_length
        )

        h_conv = self._calculate_heat_transfer_coefficient(
            nusselt=nusselt,
            thermal_conductivity=thermal_conductivity_avg,
            characteristic_length=characteristic_length
        )

        betta = self._calculate_mass_transfer_coefficient(
            h_conv=h_conv,
            density=density_avg,
            heat_capacity=heat_capacity_avg,
            lewis_number=lewis_avg
        )

        m_dot_humid = self._calculate_humid_mass_flow(
            density=density_avg,
            flow_area_air=flow_area_air,
            velocity=velocity
        )

        m_dot_dry = self._calculate_dry_mass_flow(
            m_dot_humid=m_dot_humid,
            W_in=W_in,
            W_out=state.air.W_out
        )

        h_ice = self._get_ice_enthalpy(
            T=state.hmt.T_frost_surface
        )

        T_dew_point = self._get_inlet_dew_point(
            p_in = state.air.p_in,
            W_in = W_in,
            T_in = inputs.air.T_in
        )


        state.air.set("pressure_avg", pressure_avg)
        state.air.set("density_avg", density_avg)
        state.air.set("dyn_viscosity_avg", dyn_viscosity_avg)
        state.air.set("heat_capacity_avg", heat_capacity_avg)
        state.air.set("thermal_conductivity_avg", thermal_conductivity_avg)
        state.air.set("prandtl_avg", prandtl_avg)
        state.air.set("lewis_avg", lewis_avg)
        state.air.set('rho_w_in', rho_w_in)
        state.air.set('rho_w_out', rho_w_out)
        state.air.set("rho_w_frost_sat", rho_w_frost_sat)
        state.air.set("W_frost_sat", W_frost_sat)

        state.air.set("reynolds", reynolds)
        state.air.set("nusselt", nusselt)
        state.air.set("h_conv", h_conv)
        state.air.set("betta", betta)
        state.air.set("velocity", velocity)
        state.air.set("p_in", p_in)
        state.air.set("pressure_loss_coeff", pressure_loss_coeff)
        state.air.set("m_dot_humid", m_dot_humid)
        state.air.set("m_dot_dry", m_dot_dry)

        state.air.set("h_in", h_in)
        state.air.set("h_out", h_out)
        state.air.set("h_ice", h_ice)

        state.air.set("T_dew_point", T_dew_point)
        state.air.set("W_in", W_in)


    def _get_inlet_dew_point(self, p_in: float, W_in: float, T_in: float) -> float:
        """
        Calculates the Dew Point Temperature of the inlet air.
        
        We ask CoolProp for 'Tdp' given the current state (P, W, T).
        """
        return CP_HumidAir.HAPropsSI('Tdp', 'P', p_in, 'W', W_in, 'T', T_in)

    

    def _get_air_properties(self, T:float, p:float, W:float):
        """Calculates multiple moist air properties for a single state."""

        # List of properties to calculate
        # Vha: specific Volume per humid air, mu: Dynamic Viscosity, cp_ha: Heat Capacity, k: Thermal Conductivity
        # Enthalpy: Specific enthalpy [J/kg_dry_air]
        prop_list = ['Vha', 'mu', 'cp_ha', 'k', 'Enthalpy'] 
        
        # Calculate all properties
        properties = {prop: CP_HumidAir.HAPropsSI(prop, 'T', T, 'P', p, 'W', W) for prop in prop_list}

        properties['density'] = 1 / properties['Vha']  # Convert specific volume to density

        # Prandtl Number: Pr = (cp_ha * mu) / k
        properties['prandtl'] = (properties['cp_ha'] * properties['mu']) / properties['k']

        # Lewis Number cant be calculated by CoolProp (Diffusivity is missing)
        properties['lewis'] = 0.85

        # Calculate water vapor density
        properties['rho_w'], _ = self._get_water_vapor_density(T=T,p=p, W=W)
            
        # Return as a simple tuple for easy unpacking in the main function
        return (
            properties['density'], 
            properties['mu'], 
            properties['cp_ha'], 
            properties['k'], 
            properties['prandtl'], 
            properties['lewis'],
            properties['rho_w'],
            properties['Enthalpy']
        )

    def _calculate_air_averages(self, T_in:float, p_in:float, W_in:float, T_out:float, p_out:float, W_out:float):
        # Get all properties for inlet and outlet in two compact calls
        density_in,  mu_in,  cp_in,  k_in,  prandtl_in,  lewis_in,  rho_w_in,  h_in  = self._get_air_properties(T_in,  p_in,  W_in)   # <-- h_in added
        density_out, mu_out, cp_out, k_out, prandtl_out, lewis_out, rho_w_out, h_out = self._get_air_properties(T_out, p_out, W_out)  # <-- h_out added

        # Calculate averages
        pressure_avg = (p_in + p_out) / 2
        density_avg = (density_in + density_out) / 2
        dyn_viscosity_avg = (mu_in + mu_out) / 2
        heat_capacity_avg = (cp_in + cp_out) / 2
        thermal_conductivity_avg = (k_in + k_out) / 2
        prandtl_avg = (prandtl_in + prandtl_out) / 2
        lewis_avg = (lewis_in + lewis_out) / 2
        
        # Return averages AND specific enthalpies
        return (
            pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, 
            thermal_conductivity_avg, prandtl_avg, lewis_avg, rho_w_in, rho_w_out, h_in, h_out)
    

    def _get_water_vapor_density(self, T: float, p: float, W: float = None, R: float = None) -> float:
        """
        Calculates the density of the water vapor in the moist air.
        
        Definition: rho_w = Mass_Water / Volume_AirMixture [kg/m^3]
        
        Requires T, p and EITHER W (humidity ratio) OR R (relative humidity).

        Args:
            T: Temperature [K]
            p: Pressure [Pa]
            W: Humidity ratio [kg_w/kg_da] (optional)
            R: Relative humidity [0..1] (optional)

        Returns:
            Water vapor density [kg/m^3]
        """


        # Determine the humidity ratio W, if not given (e.g., at saturation)
        if W is not None:
            W_calc = W
        elif R is not None:
            # Calculate W at saturation (R=1.0)
            W_calc = CP_HumidAir.HAPropsSI('W', 'T', T, 'P', p, 'R', R)
        else:
            # Error if neither W nor R is provided
            raise ValueError("Must provide either W (humidity ratio) or R (relative humidity)")

        # Get the specific volume Vda (m^3 / kg_dry_air)
        try:
            Vda = CP_HumidAir.HAPropsSI('Vda', 'T', T, 'P', p, 'W', W_calc)
        except ValueError as e:
            print(f"CoolProp Error during Vda calculation: T={T}, p={p}, W={W_calc} -> {e}")
            return 0.0 # Safe return value on error

        # Calculate the water vapor density:
        rho_w = W_calc / Vda         
        return rho_w, W_calc


    def _calculate_pressure_loss_coefficient_haaf(self, reynolds: float, space_between_frost: float, fin_length: float) -> float:
        """
        Calculates the pressure loss coefficient (ζ) based on the Haaf correlation (Klingebiel Diss. Eq. 4.3).
        
        NOTE: The original source (Eq. 4.3) contains a dimensional inconsistency: (d_ae / rho_L).
        This implementation assumes a typo and uses the dimensionally consistent form (d_ae / L),
        where L is the fin length, which is standard practice for such correlations.
        """
        # ... (checks) ...

        if fin_length <= 0:
            print("Error: Fin length must be positive.")
            return 0.0

        length_ratio = space_between_frost / fin_length
        
        return 10.5 * (reynolds ** (-1.0/3.0)) * (length_ratio ** 0.6)


    
    def _calculate_velocity_and_pressure_drop(self, flow_area_air: float, density: float, K: float,dyn_viscosity: float,
                                              space_between_frost: float, fin_length: float) -> tuple[float, float]:
        """
        Calculates velocity by finding the intersection of:
        1. System Curve (Haaf correlation)
        2. Fan Curve (Hard-coded Polynomial)
        """

        # --- HARD CODED FAN CURVE COEFFICIENTS ---
        # Format: dp = c0 + c1*V_dot + c2*V_dot^2
        # Example: Small Axial Fan (approx 300mm)
        # c0 (Static Pressure at 0 flow): 150 Pa
        # Max Volume Flow: approx 2000 m3/h (~0.55 m3/s) -> dp=0
        
        FAN_C0 = 68.0  
        FAN_C1 = 0.0
        FAN_C2 = -30500.0 

        # ----------------------------------------

        def objective_function(u_guess):
            """
            Returns (Fan_Pressure - System_Pressure).
            We want to find u where this is 0.
            """
            if u_guess <= 1e-3:
                return FAN_C0 # Positive pressure difference at 0 flow
            
            # A. Calculate System Resistance (Physics)
            # We must update Re and Zeta at every guess step for accuracy
            re = (density * u_guess * (2*space_between_frost)) / dyn_viscosity
            
            zeta = self._calculate_pressure_loss_coefficient_haaf(
                reynolds=re, 
                space_between_frost=space_between_frost, 
                fin_length=fin_length
            )
            
            dp_system = K * zeta * 0.5 * density * (u_guess**2)

            # B. Calculate Fan Pressure (Polynomial)
            v_dot = flow_area_air * u_guess  # Volume flow [m3/s]
            dp_fan = FAN_C0 + (FAN_C1 * v_dot) + (FAN_C2 * (v_dot**2))

            # prevent negative fan pressure (backflow)
            dp_fan = max(0.0, dp_fan)

            return dp_fan - dp_system

        # --- SOLVER ---
        # Look for a solution between 0.01 m/s and 20.0 m/s
        try:
            velocity = brentq(objective_function, 0.001, 20.0, xtol=1e-7)
        except ValueError:
            # If no intersection found (e.g., frost is fully blocked), flow is 0
            velocity = 0.0
            
        # --- FINAL PRESSURE DROP ---
        # Use the found velocity to calculate the actual Delta P
        # (We use the system curve definition for the final reported Delta P)
        if velocity > 0:
            re_final = (density * velocity * (2*space_between_frost)) / dyn_viscosity
            zeta_final = self._calculate_pressure_loss_coefficient_haaf(re_final, space_between_frost, fin_length)
            delta_p = K * zeta_final * 0.5 * density * (velocity**2)
        else:
            delta_p = FAN_C0 # Static pressure of dead-headed fan

        return velocity, delta_p


    def _calculate_reynolds_number(self, density: float, velocity: float, characteristic_length: float, dyn_viscosity: float):
        """Calculates the Reynolds number (Re = rho * v * L / mu)."""
        if dyn_viscosity == 0:
            # The simulation cannot continue if viscosity is zero.
            raise ValueError("Dynamic viscosity cannot be zero in Reynolds calculation.")
        return (density * velocity * characteristic_length) / dyn_viscosity

    def _calculate_nusselt(self, Re_Dh: float, Pr: float, Dh: float, L: float) -> float:
        """
        Calculates the mean Nusselt number (Nu_m) for flow in a flat gap (ebener Spalt), 
        covering laminar, transition, and turbulent flow regimes.

        Assumes constant wall temperature. The characteristic length is the hydraulic 
        diameter Dh. Fluid properties must be evaluated at the mean fluid temperature ϑm.

        Sources: 
        - Laminar: VDI-Wärmeatlas, Chapter G2 2.3.2, Gl. (43).
        - Transition/Turbulent: VDI-Wärmeatlas, Chapter G1 4.1 (Gnielinski correlations).

        :param Re_Dh: Reynolds number based on the hydraulic diameter Dh.
        :param Pr: Prandtl number of the fluid.
        :param Dh: Hydraulic diameter [m] (Dh = 2 * s).
        :param L: Fin length in flow direction [m].
        :return: Mean Nusselt number (Nu_m).
        """
        
        # --- Critical Reynolds Numbers as per VDI G1 ---
        RE_CRIT = 2300        # Upper limit for Laminar regime (Re <= 2300)
        RE_TURB_START = 4000  # Lower limit for fully developed Turbulent regime (Re >= 4000)

        # --- LAMINAR REGIME (Re <= 2300) ---
        if Re_Dh <= RE_CRIT:
            # Calculate Graetz Number (Gz)
            Gz = Re_Dh * Pr * (Dh / L)

            # Use Stephan's correlation (VDI 2.3.2, Gl. 43)
            return self._stephan_laminar(Gz, Pr)

        # --- TURBULENT REGIME (Re >= 4000) ---
        elif Re_Dh >= RE_TURB_START:
            # Use Gnielinski correlation (VDI G1, Gl. 26/27)
            return self._gnielinski_turbulent(Re_Dh, Pr, Dh, L)

        # --- TRANSITION REGIME (2300 < Re < 4000) ---
        else:
            # Interpolation factor (γ) - VDI G1 4.1 Gl. 30
            gamma = (Re_Dh - RE_CRIT) / (RE_TURB_START - RE_CRIT)

            # Nu at Re=2300 (Nu_m,L,2300)
            Gz_crit = RE_CRIT * Pr * (Dh / L)
            Nu_m_L_2300 = self._stephan_laminar(Gz_crit, Pr)
            
            # Nu at Re=4000 (Nu_m,T,4000)
            Nu_m_T_4000 = self._gnielinski_turbulent(RE_TURB_START, Pr, Dh, L)
            
            # Interpolation - VDI G1 4.1 Gl. 29
            Num_transition = (1.0 - gamma) * Nu_m_L_2300 + gamma * Nu_m_T_4000
            
            return Num_transition
        
    # --- Helper function for laminar flow (VDI G2 2.3.2, Gl. 43) ---
    def _stephan_laminar(self, Gz: float, Pr: float) -> float:
        """
        Calculates Nu_m for laminar flow in a flat gap (constant wall temp.).
        Gz: Graetz Number (Re_Dh * Pr * Dh / L).
        """
        # Gl. (43) from VDI G2 2.3.2 for constant wall temperature (Nu_m,II)
        numerator = 0.024 * (Gz**1.14)
        denominator = 1 + 0.0358 * (Gz**0.64) * (Pr**0.17)
        Num_lam = 7.55 + (numerator / denominator)
        return Num_lam

    # --- Helper function for turbulent flow (VDI G1 4.1 26/27) ---
    def _gnielinski_turbulent(self, Re: float, Pr: float, Dh: float, L: float) -> float:
        """
        Calculates Nu_m for turbulent flow using the Gnielinski correlation (VDI G1 4.1).
        """
        # Gl. (27) - Friction factor xi (Druckverlustbeiwert)
        xi = (1.8 * np.log10(Re) - 1.5)**(-2)
        
        # Gl. (26) - Nusselt number (adapted for flat gap with Dh)
        numerator = (xi / 8) * (Re - 1000) * Pr
        denominator = 1 + 12.7 * np.sqrt(xi / 8) * (Pr**(2/3) - 1)
        
        # Entry length correction term: [1 + (Dh/L)**(2/3)]
        Num_turb = (numerator / denominator) * (1 + (Dh / L)**(2/3))
        
        return Num_turb


    def _calculate_heat_transfer_coefficient(self, nusselt: float, thermal_conductivity: float, characteristic_length: float):
        """Calculates the convective heat transfer coefficient (h = Nu * k / L)."""
        if characteristic_length == 0:
            return 0.0
        return (nusselt * thermal_conductivity) / characteristic_length

    def _calculate_mass_transfer_coefficient(self, h_conv: float, density: float, heat_capacity: float, lewis_number: float) -> float:
        """
        Calculates the mass transfer coefficient (betta) based on the Chilton-Colburn analogy
        and the heat transfer coefficient (h_conv, or alpha_air).
        Args:
            h_conv: Convective heat transfer coefficient (alpha_air) [W/(m^2*K)].
            density: Average air density [kg/m^3].
            heat_capacity: Average air heat capacity (c_p) [J/(kg*K)].
            lewis_number: Average Lewis number (Le) [dimensionless].

        Returns:
            Mass transfer coefficient (betta) [m/s].
        """
        
        denominator = density * heat_capacity
        if denominator <= 0:
            print(f"Error: Invalid denominator in betta calculation (density * c_p = {denominator}).")
            return 0.0
            
        if lewis_number <= 0:
            print(f"Error: Invalid Lewis number ({lewis_number}) in betta calculation. Must be positive.")
            # Use 1.0 as a fallback to avoid division/power errors, though 0.85 is the default
            lewis_number = 0.85 

        # Calculate betta
        
        return (h_conv / denominator) * (lewis_number ** (-2.0/3.0))


    def _calculate_humid_mass_flow(self, density: float, flow_area_air: float, velocity: float) -> float:
        """
        Calculates the mass flow rate (m_dot) based on the continuity equation:
        m_dot = ρL * A_flow * uL
        """
        if density < 0 or flow_area_air < 0 or velocity < 0:
            print("Error: Density, flow area, and velocity must be non-negative.")
            return 0.0
            
        return density * flow_area_air * velocity
    
    def _calculate_dry_mass_flow(self, m_dot_humid: float, W_in: float, W_out: float) -> float:
        """
        Calculates the dry air mass flow rate (m_dot_da) from the humid mass flow rate (m_dot_ha)
        using the humidity ratios at inlet and outlet.
        """
        W_avg = (W_in + W_out) / 2.0
        
        # Calculate dry air mass flow
        return m_dot_humid / (1.0 + W_avg)


    def _get_ice_enthalpy(self, T: float) -> float:
        """
        Calculates the specific enthalpy of ice [J/kg] using ASHRAE Fundamentals formulation.
        
        This method ensures consistency with CoolProp's HAPropsSI reference state:
        - Reference: Liquid water at 0.01°C = 0 J/kg.
        - Ice at 0°C is approx. -333,400 J/kg (Latent heat of fusion).
        
        This manual calculation is preferred over PropsSI because PropsSI often uses 
        different null-levels (e.g., IIR convention) which leads to massive offsets 
        in energy balances when mixed with HAPropsSI.

        Args:
            T (float): Temperature of the ice [K]. Must be <= 273.16 K.
            
        Returns:
            float: Specific enthalpy of ice [J/kg]. 
        """
        T_celsius = T - 273.15
        
        # Safety check: If T is above freezing, this physical model is invalid for ice.
        # However, for robustness near 0°C, we allow small deviations.
        if T_celsius > 0.1: 
             print("Warning: Temperature above freezing point for ice enthalpy calculation.")

        # Constants derived from ASHRAE Fundamentals (SI Units)
        h_fusion_ref = -333400.0  # Enthalpy of ice at 0°C relative to liquid water at 0°C
        cp_ice = 2060.0           # Average specific heat capacity for ice [J/(kg K)]
        
        return h_fusion_ref + (cp_ice * T_celsius)