from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import CoolProp.CoolProp as CP_HumidAir


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

        # Calculate average air properties between inlet and outlet
        pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, thermal_conductivity_avg, prandtl_avg, lewis_avg, rho_w_avg = self.calculate_air_averages(
            T_in=inputs.air.T_in,
            p_in=state.air.p_in,
            W_in=inputs.air.W_in,
            T_out=state.air.T_out,
            p_out=self.params.ambient_pressure,
            W_out=state.air.W_out
        )

        # Calculate water vapor density at frost surface (saturated)
        rho_w_frost_sat = self.get_water_vapor_density(
            T=state.hmt.T_frost_surface, 
            p=pressure_avg, 
            R=1.0  # R=1.0 for fully saturated air
        )

        space_between_frost = state.frost.space_between_frost
        flow_area_air = state.frost.flow_area_air


        pressure_loss_coeff = self.calculate_pressure_loss_coefficient_haaf(
            reynolds=state.air.reynolds,
            space_between_frost=space_between_frost,
            fin_length=self.params.fin_length
        )

        velocity, delta_p = self.calculate_velocity_and_pressure_drop(
            hydraulic_fan_power=self.params.hydraulic_fan_power,
            flow_area_air=flow_area_air,
            pressure_loss_coeff=pressure_loss_coeff,
            density=density_avg,
            K=self.params.pressure_loss_fit_factor
        )

        p_in = self.params.ambient_pressure + delta_p

        reynolds = self.calculate_reynolds_number(
            density=density_avg,
            velocity=velocity,
            characteristic_length=space_between_frost,
            dyn_viscosity=dyn_viscosity_avg
        )

        nusselt = self.calculate_nusselt(
            Re_Dh=reynolds, 
            Pr=prandtl_avg, 
            Dh=space_between_frost, 
            L=self.params.fin_length
        )

        h_conv = self.calculate_h_conv(
            nusselt=nusselt,
            thermal_conductivity=thermal_conductivity_avg,
            characteristic_length=space_between_frost
        )

        m_flow = self.calculate_mass_flow(
            density=density_avg,
            flow_area_air=flow_area_air,
            velocity=velocity
        )


        state.air.set("pressure_avg", pressure_avg)
        state.air.set("density_avg", density_avg)
        state.air.set("dyn_viscosity_avg", dyn_viscosity_avg)
        state.air.set("heat_capacity_avg", heat_capacity_avg)
        state.air.set("thermal_conductivity_avg", thermal_conductivity_avg)
        state.air.set("prandtl_avg", prandtl_avg)
        state.air.set("lewis_avg", lewis_avg)
        state.air.set("rho_w_avg", rho_w_avg)
        state.air.set("rho_w_frost_sat", rho_w_frost_sat)

        state.air.set("reynolds", reynolds)
        state.air.set("nusselt", nusselt)
        state.air.set("h_conv", h_conv)
        state.air.set("velocity", velocity)
        state.air.set("p_in", p_in)
        state.air.set("pressure_loss_coeff", pressure_loss_coeff)
        state.air.set("m_flow", m_flow)




    

    def get_air_properties(self, T:float, p:float, W:float):
        """Calculates multiple moist air properties for a single state."""
        
        properties = {}
        # List of properties to calculate
        # Vha: specific Volume per humid air, mu: Dynamic Viscosity, cp_ha: Heat Capacity, k: Thermal Conductivity
        prop_list = ['Vha', 'mu', 'cp_ha', 'k'] 
        
        # Calculate all properties in a loop
        for prop in prop_list:
            # One HAPropsSI call per property
            properties[prop] = CP_HumidAir.HAPropsSI(prop, 'T', T, 'P', p, 'W', W) 

        properties['density'] = 1 / properties['Vha']  # Convert specific volume to density

        # Prandtl Number: Pr = (cp_ha * mu) / k
        properties['prandtl'] = (properties['cp_ha'] * properties['mu']) / properties['k']

        # Lewis Number cant be calculated by CoolProp (Diffusivity is missing)
        properties['lewis'] = 0.85

        # Calculate water vapor density
        properties['rho_w'] = self.get_water_vapor_density(T=T,p=p, W=W)
            
        # Return as a simple tuple for easy unpacking in the main function
        return (
            properties['density'], 
            properties['mu'], 
            properties['cp_ha'], 
            properties['k'], 
            properties['prandtl'], 
            properties['lewis'],
            properties['rho_w']
        )

    def calculate_air_averages(self, T_in:float, p_in:float, W_in:float, T_out:float, p_out:float, W_out:float):
        # Get all properties for inlet and outlet in two compact calls
        density_in,  mu_in,  cp_in,  k_in,  prandtl_in,  lewis_in,  rho_w_in  = self.get_air_properties(T_in,  p_in,  W_in)
        density_out, mu_out, cp_out, k_out, prandtl_out, lewis_out, rho_w_out = self.get_air_properties(T_out, p_out, W_out)

        # Calculate averages
        pressure_avg = (p_in + p_out) / 2
        density_avg = (density_in + density_out) / 2
        dyn_viscosity_avg = (mu_in + mu_out) / 2
        heat_capacity_avg = (cp_in + cp_out) / 2
        thermal_conductivity_avg = (k_in + k_out) / 2
        prandtl_avg = (prandtl_in + prandtl_out) / 2
        lewis_avg = (lewis_in + lewis_out) / 2
        rho_w_avg = (rho_w_in + rho_w_out) / 2
        

        return pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, thermal_conductivity_avg, prandtl_avg, lewis_avg, rho_w_avg
    

    def get_water_vapor_density(self, T: float, p: float, W: float = None, R: float = None) -> float:
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
        # rho_w = W / Vha         
        return W_calc / Vda


    def calculate_pressure_loss_coefficient_haaf(self, reynolds: float, space_between_frost: float, fin_length: float) -> float:
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



    def calculate_velocity_and_pressure_drop(self, hydraulic_fan_power: float, flow_area_air: float, pressure_loss_coeff: float, density: float, K: float) -> tuple[float, float]:
        """
        Calculates the air velocity (uL) AND the pressure drop (Delta p) 
        simultaneously, based on the fan's hydraulic power.

        This function combines two relationships:
        1. uL = ( (2 * P_Hyd) / (A_flow * ζ * ρL * K) ) ^ (1/3)
        2. Δp = K * ζ * 0.5 * ρ * uL^2
        
        Args:
            hydraulic_fan_power (float): Hydraulic power from the fan [W].
            flow_area_air (float): Cross-sectional flow area [m^2].
            pressure_loss_coeff (float): Pressure loss coefficient (ζ) [dimensionless].
            density (float): Average air density (ρL) [kg/m^3].
            K (float): Proportionality constant [dimensionless].

        Returns:
            tuple[float, float]: (velocity [m/s], delta_p [Pa])
        """
        
        # --- Velocity Calculation ---
        if hydraulic_fan_power < 0:
            print("Error: Hydraulic fan power must be non-negative.")
            return 0.0, 0.0
        
        denominator = flow_area_air * pressure_loss_coeff * density * K
        if denominator <= 0:
            print("Error: The denominator (A_flow * ζ * ρL * K) must be positive.")
            return 0.0, 0.0

        # Calculate velocity
        velocity = ((2 * hydraulic_fan_power) / denominator) ** (1.0/3.0)
        
        # --- Pressure Drop Calculation ---
        # Use the velocity we just calculated.
        # Δp = K * ζ * 0.5 * ρ * uL^2
        delta_p = K * pressure_loss_coeff * 0.5 * density * (velocity ** 2)
        
        return velocity, delta_p


    def calculate_reynolds_number(self, density: float, velocity: float, characteristic_length: float, dyn_viscosity: float):
        """Calculates the Reynolds number (Re = rho * v * L / mu)."""
        if dyn_viscosity == 0:
            # The simulation cannot continue if viscosity is zero.
            raise ValueError("Dynamic viscosity cannot be zero in Reynolds calculation.")
        return (density * velocity * characteristic_length) / dyn_viscosity

    def calculate_nusselt(self, Re_Dh: float, Pr: float, Dh: float, L: float) -> float:
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

        # --- Helper function for laminar flow (VDI G2 2.3.2, Gl. 43) ---
        def stephan_laminar(Gz: float, Pr: float) -> float:
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
        def gnielinski_turbulent(Re: float, Pr: float, Dh: float, L: float) -> float:
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

        # Calculate Graetz Number (Gz)
        Gz = Re_Dh * Pr * (Dh / L)
        
        # --- Critical Reynolds Numbers as per VDI G1 ---
        RE_CRIT = 2300        # Upper limit for Laminar regime (Re <= 2300)
        RE_TURB_START = 4000  # Lower limit for fully developed Turbulent regime (Re >= 4000)

        # --- LAMINAR REGIME (Re <= 2300) ---
        if Re_Dh <= RE_CRIT:
            # Use Stephan's correlation (VDI 2.3.2, Gl. 43)
            return stephan_laminar(Gz, Pr)

        # --- TURBULENT REGIME (Re >= 4000) ---
        elif Re_Dh >= RE_TURB_START:
            # Use Gnielinski correlation (VDI G1, Gl. 26/27)
            return gnielinski_turbulent(Re_Dh, Pr, Dh, L)

        # --- TRANSITION REGIME (2300 < Re < 4000) ---
        else:
            # Interpolation factor (γ) - VDI G1 4.1 Gl. 30
            gamma = (Re_Dh - RE_CRIT) / (RE_TURB_START - RE_CRIT)

            # Nu at Re=2300 (Nu_m,L,2300)
            Gz_crit = RE_CRIT * Pr * (Dh / L)
            Nu_m_L_2300 = stephan_laminar(Gz_crit, Pr)
            
            # Nu at Re=4000 (Nu_m,T,4000)
            Nu_m_T_4000 = gnielinski_turbulent(RE_TURB_START, Pr, Dh, L)
            
            # Interpolation - VDI G1 4.1 Gl. 29
            Num_transition = (1.0 - gamma) * Nu_m_L_2300 + gamma * Nu_m_T_4000
            
            return Num_transition


    def calculate_h_conv(self, nusselt: float, thermal_conductivity: float, characteristic_length: float):
        """Calculates the convective heat transfer coefficient (h = Nu * k / L)."""
        if characteristic_length == 0:
            return 0.0
        return (nusselt * thermal_conductivity) / characteristic_length



    def calculate_mass_flow(self, density: float, flow_area_air: float, velocity: float) -> float:
        """
        Calculates the mass flow rate (m_flow) based on the continuity equation:
        m_flow = ρL * A_flow * uL

        :param density: Density of the fluid (ρL) [kg/m^3].
        :param flow_area_air: Cross-sectional area of the air flow (A_flow) [m^2].
        :param velocity: Air velocity (uL) [m/s].
        :return: Mass flow rate (m_flow) [kg/s].
        """
        if density < 0 or flow_area_air < 0 or velocity < 0:
            print("Error: Density, flow area, and velocity must be non-negative.")
            return 0.0
            
        return density * flow_area_air * velocity

