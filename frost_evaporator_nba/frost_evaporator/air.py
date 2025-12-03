from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
import numpy as np
import warnings
import CoolProp.CoolProp as CP_HumidAir
from scipy.optimize import brentq
from scipy.interpolate import interp1d

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

        # Wang parameters setup (transverse and longitudinal tube pitch)
        self.P_t = self.params.fin_height / self.params.tubes_per_layer
        self.P_l = self.params.fin_length / self.params.tube_layers

        # Wang parameters validity check
        self._check_wang_validity()


        # Fan interpolation setup
        flow_data = np.array([
            0.0, 24.591265397536393, 39.50727883538634, 46.96528555431131, 50.190369540873455,
            52.40761478163493, 55.02799552071668, 58.0515117581187, 61.679731243001115,
            65.30795072788354, 68.93617021276596, 81.43337066069428, 105.21836506159013,
            118.11870100783874, 127.39081746920492, 136.46136618141097, 145.73348264277715,
            155.20716685330348, 164.88241881298993, 174.55767077267637
        ])
        pressure_data = np.array([
            61.15112994350283, 58.13559322033898, 54.971751412429384, 51.7090395480226, 48.44632768361582,
            45.183615819209045, 42.01977401129944, 38.75706214689266, 35.494350282485875,
            32.33050847457627, 29.06779661016949, 25.805084745762713, 22.641242937853107,
            19.37853107344633, 16.11581920903955, 13.05084745762712, 9.689265536723164,
            6.52542372881356, 3.26271186440678, 0.09887005649717515
        ])
        self.min_f = flow_data[0]
        self.max_f = flow_data[-1]
        self.interpolator = interp1d(flow_data, pressure_data, kind='cubic', fill_value="extrapolate")



    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the air model.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """

        # ================= Get State Values =================
        # Inputs
        T_in = inputs.air.T_in
        R_in = inputs.air.R_in

        # State (Current Guesses)
        p_in_current = state.air.p_in
        T_out = state.air.T_out
        W_out = state.air.W_out
        
        T_frost_surface = state.hmt.T_frost_surface
        T_frost_base = state.hmt.T_frost_base
        
        space_between_frost = state.frost.space_between_frost
        flow_area_air = state.frost.flow_area_air

        # Parameters
        p_ambient = self.params.ambient_pressure

        fin_spacing = self.params.fin_spacing
        correction_factor_pressure_loss = self.params.correction_factor_pressure_loss


        # frost_thickness = state.frost.thickness
        # tube_outer_diameter = self.params.tube_outer_diameter
        # fin_thickness = self.params.fin_thickness
        # tube_layers = self.params.tube_layers
        # fin_pitch = self.params.fin_pitch
        # P_t = self.P_t
        # P_l = self.P_l


        # ================= Calculate new Values =================
        
        # Inlet Humidity Ratio (W_in)
        W_in = CP_HumidAir.HAPropsSI(
            'W', 
            'T', T_in, 
            'P', p_ambient, 
            'R', R_in
        )

        # Average Air Properties
        pressure_avg, density_avg, dyn_viscosity_avg, heat_capacity_avg, thermal_conductivity_avg, prandtl_avg, lewis_avg, rho_w_in, rho_w_out, h_in, h_out = self._calculate_air_averages(
            T_in=T_in,
            p_in=p_in_current,
            W_in=W_in,
            T_out=T_out,
            p_out=p_ambient,
            W_out=W_out
        )

        # Water Vapor Densities (at surface and base)
        rho_w_frost_surface_sat, W_frost_surface_sat = self._get_water_vapor_density(
            T=T_frost_surface, 
            p=pressure_avg, 
            R=1.0  # Saturated
        )
        
        rho_w_frost_base_sat, W_frost_base_sat = self._get_water_vapor_density(
            T=T_frost_base, 
            p=pressure_avg, 
            R=1.0  # Saturated
        )

        # Velocity and Pressure Drop
        velocity, delta_p = self._calculate_velocity_and_pressure_drop(
            flow_area_air=flow_area_air,
            density=density_avg,
            K=correction_factor_pressure_loss,
            dyn_viscosity=dyn_viscosity_avg,
            space_between_frost=space_between_frost,
            fin_spacing=fin_spacing
        )

        new_p_in = p_ambient + delta_p

        # Calculate h_conv
        D_h = 2 * space_between_frost 

        Re_Dh = self._calculate_reynolds_number(
            density=density_avg, 
            velocity=velocity, 
            characteristic_length=D_h, 
            dyn_viscosity=dyn_viscosity_avg
        )

        nusselt = self._calculate_nusselt(
            Re_Dh=Re_Dh, 
            Pr=prandtl_avg,
            D_h=D_h
        )
        
        h_conv_raw = self._calculate_heat_transfer_coefficient(
            nusselt=nusselt, 
            thermal_conductivity=thermal_conductivity_avg, 
            characteristic_length=D_h
        )
        
        h_conv = h_conv_raw * self.params.correction_factor_h_conv_air
        # h_conv=30

        # Mass Transfer Coefficient
        betta = self._calculate_mass_transfer_coefficient(
            h_conv=h_conv,
            density=density_avg,
            heat_capacity=heat_capacity_avg,
            lewis_number=lewis_avg
        )

        # Mass Flows
        m_dot_humid = self._calculate_humid_mass_flow(
            density=density_avg,
            flow_area_air=flow_area_air,
            velocity=velocity
        )

        m_dot_dry = self._calculate_dry_mass_flow(
            m_dot_humid=m_dot_humid,
            W_in=W_in,
            W_out=W_out
        )

        # Enthalpy of Ice & Dew Point
        h_ice = self._get_ice_enthalpy(
            T=T_frost_surface
        )

        T_dew_point = self._get_inlet_dew_point(
            p_in=p_in_current,
            W_in=W_in,
            T_in=T_in
        )


        # ================= Write new values to state =================
        # Air Properties
        state.air.set("pressure_avg", pressure_avg)
        state.air.set("density_avg", density_avg)
        state.air.set("dyn_viscosity_avg", dyn_viscosity_avg)
        state.air.set("heat_capacity_avg", heat_capacity_avg)
        state.air.set("thermal_conductivity_avg", thermal_conductivity_avg)
        state.air.set("prandtl_avg", prandtl_avg)
        state.air.set("lewis_avg", lewis_avg)
        
        # Humid Air / Water Vapor
        state.air.set('rho_w_in', rho_w_in)
        state.air.set('rho_w_out', rho_w_out)
        state.air.set("rho_w_frost_surface_sat", rho_w_frost_surface_sat)
        state.air.set("W_frost_surface_sat", W_frost_surface_sat)
        state.air.set("rho_w_frost_base_sat", rho_w_frost_base_sat)
        state.air.set("W_frost_base_sat", W_frost_base_sat)

        # Flow & Heat Transfer
        state.air.set("reynolds", Re_Dh)
        # state.air.set("nusselt", 0.0)
        state.air.set("h_conv", h_conv)
        state.air.set("betta", betta)
        state.air.set("velocity", velocity)
        state.air.set("p_in", new_p_in)
        
        # Mass Flow
        state.air.set("m_dot_humid", m_dot_humid)
        state.air.set("m_dot_dry", m_dot_dry)

        # Enthalpies
        state.air.set("h_in", h_in)
        state.air.set("h_out", h_out)
        state.air.set("h_ice", h_ice)

        # Inlet Conditions
        state.air.set("T_dew_point", T_dew_point)
        state.air.set("W_in", W_in)



    ####################################################################################
    # Helper Functions
    ####################################################################################

    def _check_wang_validity(self) -> None:
        """
        Validates if the geometry lies within the limits of the Wang et al. (2000) correlation.

        This checks if the plain fin-and-tube exchanger parameters (tube layers,
        diameters, and pitches) are within the experimental ranges defined by the
        correlation. Emits a warning if parameters are out of bounds.

        Ranges Checked:
            - Tube Rows (N): 1 to 6
            - Tube Outer Diameter: 6.35 to 12.7 [mm]
            - Fin Pitch: 1.19 to 8.7 [mm]
            - Transverse Pitch: 17.7 to 31.75 [mm]
            - Longitudinal Pitch: 12.4 to 27.5 [mm]

        Raises:
            UserWarning: If any geometric parameter is outside the valid range.
        """
        # scaling factor: meters to millimeters
        m_to_mm = 1000.0

        # Extract and convert parameters to mm for comparison
        tube_rows = self.params.tube_layers
        tube_outer_diameter_mm = self.params.tube_outer_diameter * m_to_mm
        fin_pitch_mm = self.params.fin_pitch * m_to_mm
        transverse_pitch_mm = self.P_t * m_to_mm
        longitudinal_pitch_mm = self.P_l * m_to_mm

        # Define validation limits: (current_value, min_limit, max_limit)
        checks = {
            "Tube Rows (N)": (tube_rows, 1, 6),
            "Tube Outer Diameter (Do)": (tube_outer_diameter_mm, 6.35, 12.7),
            "Fin Pitch (Fp)": (fin_pitch_mm, 1.19, 8.7),
            "Transverse Pitch (Pt)": (transverse_pitch_mm, 17.7, 31.75),
            "Longitudinal Pitch (Pl)": (longitudinal_pitch_mm, 12.4, 27.5),
        }

        for name, (val, min_v, max_v) in checks.items():
            if not (min_v <= val <= max_v):
                warnings.warn(
                    f"[Wang Correlation Validity] {name} value {val:.2f} is outside "
                    f"the valid range [{min_v} - {max_v}]. Results may be inaccurate."
                )


    def _get_inlet_dew_point(self, p_in: float, W_in: float, T_in: float) -> float:
        """
        Calculates the dew point temperature of the inlet air using CoolProp.

        Args:
            p_in: The absolute pressure of the inlet air [Pa].
            W_in: The humidity ratio (specific humidity) [kg_water/kg_dry_air].
            T_in: The dry bulb temperature of the inlet air [K].

        Returns:
            The dew point temperature [K].
        """
        return CP_HumidAir.HAPropsSI('Tdp', 'P', p_in, 'W', W_in, 'T', T_in)



    def _get_air_properties(self, T: float, p: float, W: float) -> tuple[float, float, float, float, float, float, float, float]:
        """
        Calculates multiple thermophysical properties of moist air for a single state.

        Args:
            temperature: The dry bulb temperature [K].
            pressure: The absolute pressure [Pa].
            humidity_ratio: The humidity ratio (specific humidity) [kg_water/kg_dry_air].

        Returns:
            A tuple containing the following properties in order:
            0. Density [kg/m^3]
            1. Dynamic Viscosity [Pa*s]
            2. Specific Heat Capacity [J/kg*K]
            3. Thermal Conductivity [W/m*K]
            4. Prandtl Number [-]
            5. Lewis Number [-]
            6. Water Vapor Density [kg/m^3]
            7. Specific Enthalpy [J/kg_dry_air]
        """
        # Define property keys for CoolProp
        # Vha: Vol. per humid air, mu: Viscosity, cp_ha: Heat Cap., k: Conductivity, Hha: Specific Enthalpy per humid air basis
        prop_keys = ['Vha', 'mu', 'cp_ha', 'k', 'Enthalpy'] 
        
        props = {key: CP_HumidAir.HAPropsSI(key, 'T', T, 'P', p, 'W', W)for key in prop_keys}

        # Derived properties
        density = 1.0 / props['Vha']
        prandtl_number = (props['cp_ha'] * props['mu']) / props['k']
        
        # Lewis Number approximation (CoolProp lacks diffusivity for humid air)
        lewis_number = 0.85

        # Calculate water vapor density (utilizing internal helper)
        water_vapor_density, _ = self._get_water_vapor_density(T=T, p=p, W=W)

        return (
            density,
            props['mu'],
            props['cp_ha'],
            props['k'],
            prandtl_number,
            lewis_number,
            water_vapor_density,
            props['Enthalpy']
        )
    

    def _calculate_air_averages(self, T_in: float, p_in: float, W_in: float, T_out: float, p_out: float, W_out: float
                                ) -> tuple[float, float, float, float, float, float, float, float, float, float, float]:
        """
        Calculates average air properties between inlet and outlet states.

        Args:
            T_in: Inlet temperature [K].
            p_in: Inlet pressure [Pa].
            W_in: Inlet humidity ratio [kg_water/kg_dry_air].
            T_out: Outlet temperature [K].
            p_out: Outlet pressure [Pa].
            W_out: Outlet humidity ratio [kg_water/kg_dry_air].

        Returns:
            A tuple containing:
            0.  Average Pressure [Pa]
            1.  Average Density [kg/m^3]
            2.  Average Dynamic Viscosity [Pa*s]
            3.  Average Specific Heat Capacity [J/kg*K]
            4.  Average Thermal Conductivity [W/m*K]
            5.  Average Prandtl Number [-]
            6.  Average Lewis Number [-]
            7.  Inlet Water Vapor Density [kg/m^3]
            8.  Outlet Water Vapor Density [kg/m^3]
            9.  Inlet Specific Enthalpy [J/kg]
            10. Outlet Specific Enthalpy [J/kg]
        """
        # Unpack properties for inlet state
        (rho_in, mu_in, cp_in, k_in, pr_in, le_in, rho_w_in, h_in) = self._get_air_properties(T_in, p_in, W_in)

        # Unpack properties for outlet state
        (rho_out, mu_out, cp_out, k_out, pr_out, le_out, rho_w_out, h_out) = self._get_air_properties(T_out, p_out, W_out)

        # Calculate arithmetic averages
        pressure_avg = (p_in + p_out) / 2.0
        density_avg = (rho_in + rho_out) / 2.0
        viscosity_avg = (mu_in + mu_out) / 2.0
        heat_capacity_avg = (cp_in + cp_out) / 2.0
        conductivity_avg = (k_in + k_out) / 2.0
        prandtl_avg = (pr_in + pr_out) / 2.0
        lewis_avg = (le_in + le_out) / 2.0

        return (
            pressure_avg,
            density_avg,
            viscosity_avg,
            heat_capacity_avg,
            conductivity_avg,
            prandtl_avg,
            lewis_avg,
            rho_w_in,
            rho_w_out,
            h_in,
            h_out
        )
    

    def _get_water_vapor_density(self, T: float, p: float, W: float = None, R: float = None) -> tuple[float, float]:
        """
        Calculates the density of the water vapor in the moist air.

        Args:
            T: The dry bulb temperature [K].
            p: The ambient pressure [Pa].
            W: The humidity ratio [kg_w/kg_da].
            R: The relative humidity [0-1].

        Returns:
            The water vapor density [kg/m^3].
            The water humidity ratio  [kg_w/kg_da].

        Raises:
            ValueError: If neither W nor R is provided.
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
        Vda = CP_HumidAir.HAPropsSI('Vda', 'T', T, 'P', p, 'W', W_calc)
        
        # Calculate the water vapor density:
        rho_w = W_calc / Vda         
        return rho_w, W_calc

    
    def _calculate_velocity_and_pressure_drop(self,flow_area_air: float,density: float,K: float,dyn_viscosity: float,space_between_frost: float,fin_spacing: float) -> tuple[float, float]:
        """
        Calculates velocity by finding the intersection of the System Curve and Fan Curve.

        This method solves for the operating point where the Fan Pressure equals
        the System Pressure (Haaf correlation).

        Args:
            flow_area_air: The cross-sectional flow area of the air [m^2].
            density: The density of the air [kg/m^3].
            K: The geometric resistance coefficient [-].
            dyn_viscosity: The dynamic viscosity of the air [Pa·s].
            space_between_frost: The effective space between frost layers [m].
            fin_spacing: The base spacing between fins [m].

        Returns:
            A tuple containing:
                - velocity: The calculated air velocity [m/s].
                - delta_p: The pressure drop at the operating point [Pa].
        """

        def _objective_function(u_guess: float) -> float:
            """
            Calculates the residual (Fan_Pressure - System_Pressure).
            We want to find u_guess where this returns 0.
            """
            # A. Calculate System Resistance (Physics)
            reynolds = (density * u_guess * (2 * space_between_frost)) / dyn_viscosity

            zeta = self._calculate_pressure_loss_coefficient_haaf(
                reynolds=reynolds,
                space_between_frost=space_between_frost,
                fin_spacing=fin_spacing
            )

            dp_system = K * zeta * 0.5 * density * (u_guess ** 2)

            # B. Calculate Fan Pressure (Polynomial/Interpolation)
            # Convert velocity [m/s] to volume flow [m^3/h]
            v_dot = flow_area_air * u_guess * 3600.0
            dp_fan = self._get_pressure_from_flow(v_dot)

            return dp_fan - dp_system

        # --- SOLVER ---
        # Look for a solution between 0.001 m/s and max capacity.
        try:
            # Max volume flow assumed 174.55 m^3/h converted to m/s
            max_velocity = (174.55 / 3600.0) / flow_area_air
            
            velocity = brentq(
                _objective_function,
                0.001,
                max_velocity,
                xtol=1e-7
            )
        except ValueError:
            # If no intersection found (e.g., frost is fully blocked), flow is 0
            velocity = 0.0

        # --- FINAL PRESSURE DROP ---
        # Use the found velocity to calculate the actual Delta P
        if velocity > 0:
            re_final = (density * velocity * (2 * space_between_frost)) / dyn_viscosity
            
            zeta_final = self._calculate_pressure_loss_coefficient_haaf(
                re_final,
                space_between_frost,
                fin_spacing
            )
            
            delta_p = K * zeta_final * 0.5 * density * (velocity ** 2)
        else:
            # Static pressure of dead-headed fan (0 flow)
            delta_p = self._get_pressure_from_flow(0.0)

        return velocity, delta_p
    
    
    def _get_pressure_from_flow(self, flow_val: float) -> float:
        """
        Returns Static Pressure for a given Volume Flow.

        Args:
            flow_val: The volume flow rate [m^3/h].

        Returns:
            The static pressure [Pa].
        """
        # Check for out of bounds
        if flow_val < self.min_f or flow_val > self.max_f:
            print(
                f"--> WARNING: Flow input {flow_val:.2f} is outside valid range "
                f"({self.min_f:.2f} - {self.max_f:.2f}). Extrapolating..."
            )

        return float(self.interpolator(flow_val))
    

    def _calculate_pressure_loss_coefficient_haaf(self,reynolds: float,space_between_frost: float,fin_spacing: float) -> float:
        """
        Calculates the pressure loss coefficient (zeta) based on the Haaf correlation.

        Args:
            reynolds: The Reynolds number [-].
            space_between_frost: The effective space between frost layers [m].
            fin_spacing: The distance between fins (fin length L) [m].

        Returns:
            The pressure loss coefficient [-].

        Raises:
            ValueError: If fin_spacing is less than or equal to zero.
        """
        if fin_spacing <= 0:
            raise ValueError(f"Fin spacing must be positive. Received: {fin_spacing}")

        length_ratio = space_between_frost / fin_spacing

        return 10.5 * (reynolds ** (-1.0 / 3.0)) * (length_ratio ** 0.6)


    def _calculate_reynolds_number(self, density: float, velocity: float, characteristic_length: float, dyn_viscosity: float) -> float:
        """
        Calculates the Reynolds number (Re = rho * v * L / mu).

        Args:
            density: The fluid density [kg/m^3].
            velocity: The fluid velocity [m/s].
            characteristic_length: The characteristic length [m].
            dyn_viscosity: The dynamic viscosity [Pa*s].

        Returns:
            The Reynolds number [dimensionless].        """
        return (density * velocity * characteristic_length) / dyn_viscosity


    def _calculate_mass_transfer_coefficient(self, h_conv: float, density: float, heat_capacity: float, lewis_number: float) -> float:
        """
        Calculates the mass transfer coefficient (betta) based on the Chilton-Colburn analogy.

        Args:
            h_conv: Convective heat transfer coefficient (alpha_air) [W/(m^2*K)].
            density: Average air density [kg/m^3].
            heat_capacity: Average air heat capacity (c_p) [J/(kg*K)].
            lewis_number: Average Lewis number (Le) [dimensionless].

        Returns:
            The mass transfer coefficient (betta) [m/s].

        Raises:
            ValueError: If the product of density and heat_capacity is <= 0.
        """
        denominator = density * heat_capacity
        if denominator <= 0:
            raise ValueError(f"Invalid denominator in betta calculation (density * c_p = {denominator}).")

        return (h_conv / denominator) * (lewis_number ** (-2.0 / 3.0))


    def _calculate_humid_mass_flow(self, 
                                   density: float, 
                                   flow_area_air: float, 
                                   velocity: float) -> float:
        """
        Calculates the humid air mass flow rate based on the continuity equation.

        Args:
            density: The humid air density [kg/m^3].
            flow_area_air: The effective flow cross-sectional area [m^2].
            velocity: The air velocity [m/s].

        Returns:
            The humid air mass flow rate [kg/s].

        Raises:
            ValueError: If density, area, or velocity are negative.
        """
        if density < 0 or flow_area_air < 0 or velocity < 0:
            raise ValueError(f"Inputs must be non-negative: rho={density}, area={flow_area_air}, vel={velocity}.")

        return density * flow_area_air * velocity

        
    def _calculate_dry_mass_flow(self, m_dot_humid: float, W_in: float, W_out: float) -> float:
        """
        Calculates the dry air mass flow rate from the humid mass flow rate.

        Args:
            m_dot_humid: The humid air mass flow rate [kg/s].
            W_in: Humidity ratio at inlet [kg_w/kg_da].
            W_out: Humidity ratio at outlet [kg_w/kg_da].

        Returns:
            The dry air mass flow rate [kg/s].
        """
        W_avg = (W_in + W_out) / 2.0
        return m_dot_humid / (1.0 + W_avg)


    def _get_ice_enthalpy(self, T: float) -> float:
        """
        Calculates the specific enthalpy of ice using ASHRAE Fundamentals formulation.

        Args:
            T: Temperature of the ice [K].

        Returns:
            The specific enthalpy of ice [J/kg].
        """
        import warnings
        
        T_celsius = T - 273.15

        # Robustness check for physical validity
        if T_celsius > 0.1:
            warnings.warn(f"Temperature ({T} K) is above freezing point for ice enthalpy calculation.", RuntimeWarning)

        # Constants derived from ASHRAE Fundamentals (SI Units)
        # Reference: Liquid water at 0.01°C = 0 J/kg.
        h_fusion_ref = -333400.0 
        cp_ice = 2060.0 

        return h_fusion_ref + (cp_ice * T_celsius)
    

    def _calculate_nusselt(self, Re_Dh: float, Pr: float, D_h:float) -> float:
        # Jonas Diss (4.1)
        return 0.31 * (Re_Dh**(5/8)) * (Pr**(1/3)) * ((D_h / self.P_l)**(1/3))
    

    

    def _calculate_heat_transfer_coefficient(self, nusselt: float, thermal_conductivity: float, characteristic_length: float):
        """Calculates the convective heat transfer coefficient (h = Nu * k / L)."""
        if characteristic_length == 0:
            return 0.0
        return (nusselt * thermal_conductivity) / characteristic_length

    # Backup of VDI Nusselt stuff

    
    # def _calculate_nusselt(self, Re_Dh: float, Pr: float, Dh: float, L: float) -> float:
    #     """
    #     Calculates the mean Nusselt number (Nu_m) for flow in a flat gap (ebener Spalt), 
    #     covering laminar, transition, and turbulent flow regimes.

    #     Assumes constant wall temperature. The characteristic length is the hydraulic 
    #     diameter Dh. Fluid properties must be evaluated at the mean fluid temperature ϑm.

    #     Sources: 
    #     - Laminar: VDI-Wärmeatlas, Chapter G2 2.3.2, Gl. (43).
    #     - Transition/Turbulent: VDI-Wärmeatlas, Chapter G1 4.1 (Gnielinski correlations).

    #     :param Re_Dh: Reynolds number based on the hydraulic diameter Dh.
    #     :param Pr: Prandtl number of the fluid.
    #     :param Dh: Hydraulic diameter [m] (Dh = 2 * s).
    #     :param L: Fin length in flow direction [m].
    #     :return: Mean Nusselt number (Nu_m).
    #     """
        
    #     # --- Critical Reynolds Numbers as per VDI G1 ---
    #     RE_CRIT = 2300        # Upper limit for Laminar regime (Re <= 2300)
    #     RE_TURB_START = 4000  # Lower limit for fully developed Turbulent regime (Re >= 4000)

    #     # --- LAMINAR REGIME (Re <= 2300) ---
    #     if Re_Dh <= RE_CRIT:
    #         # Calculate Graetz Number (Gz)
    #         Gz = Re_Dh * Pr * (Dh / L)

    #         # Use Stephan's correlation (VDI 2.3.2, Gl. 43)
    #         return self._stephan_laminar(Gz, Pr)

    #     # --- TURBULENT REGIME (Re >= 4000) ---
    #     elif Re_Dh >= RE_TURB_START:
    #         # Use Gnielinski correlation (VDI G1, Gl. 26/27)
    #         return self._gnielinski_turbulent(Re_Dh, Pr, Dh, L)

    #     # --- TRANSITION REGIME (2300 < Re < 4000) ---
    #     else:
    #         # Interpolation factor (γ) - VDI G1 4.1 Gl. 30
    #         gamma = (Re_Dh - RE_CRIT) / (RE_TURB_START - RE_CRIT)

    #         # Nu at Re=2300 (Nu_m,L,2300)
    #         Gz_crit = RE_CRIT * Pr * (Dh / L)
    #         Nu_m_L_2300 = self._stephan_laminar(Gz_crit, Pr)
            
    #         # Nu at Re=4000 (Nu_m,T,4000)
    #         Nu_m_T_4000 = self._gnielinski_turbulent(RE_TURB_START, Pr, Dh, L)
            
    #         # Interpolation - VDI G1 4.1 Gl. 29
    #         Num_transition = (1.0 - gamma) * Nu_m_L_2300 + gamma * Nu_m_T_4000
            
    #         return Num_transition
        
    # # --- Helper function for laminar flow (VDI G2 2.3.2, Gl. 43) ---
    # def _stephan_laminar(self, Gz: float, Pr: float) -> float:
    #     """
    #     Calculates Nu_m for laminar flow in a flat gap (constant wall temp.).
    #     Gz: Graetz Number (Re_Dh * Pr * Dh / L).
    #     """
    #     # Gl. (43) from VDI G2 2.3.2 for constant wall temperature (Nu_m,II)
    #     numerator = 0.024 * (Gz**1.14)
    #     denominator = 1 + 0.0358 * (Gz**0.64) * (Pr**0.17)
    #     Num_lam = 7.55 + (numerator / denominator)
    #     return Num_lam

    # # --- Helper function for turbulent flow (VDI G1 4.1 26/27) ---
    # def _gnielinski_turbulent(self, Re: float, Pr: float, Dh: float, L: float) -> float:
    #     """
    #     Calculates Nu_m for turbulent flow using the Gnielinski correlation (VDI G1 4.1).
    #     """
    #     # Gl. (27) - Friction factor xi (Druckverlustbeiwert)
    #     xi = (1.8 * np.log10(Re) - 1.5)**(-2)
        
    #     # Gl. (26) - Nusselt number (adapted for flat gap with Dh)
    #     numerator = (xi / 8) * (Re - 1000) * Pr
    #     denominator = 1 + 12.7 * np.sqrt(xi / 8) * (Pr**(2/3) - 1)
        
    #     # Entry length correction term: [1 + (Dh/L)**(2/3)]
    #     Num_turb = (numerator / denominator) * (1 + (Dh / L)**(2/3))
        
    #     return Num_turb







    # def _calculate_wang_correlations(self, Re_Dc: float, N: int, F_p: float, D_c: float, D_h: float, P_t: float, P_l: float) -> float:
    #     """
    #     Calculates Colburn j-factor using Wang et al. (2000) correlations.

    #     Args:
    #         Re_Dc: Reynolds number based on Collar Diameter [dimensionless].
    #         N: Number of tube rows (tube_layers) [count].
    #         F_p: Fin Pitch (center-to-center) [m].
    #         D_c: Collar diameter (Tube OD + 2*fin_thickness + 2*frost_thickness) [m].
    #         D_h: Hydraulic diameter [m].
    #         P_t: Transverse tube pitch [m].
    #         P_l: Longitudinal tube pitch [m].

    #     Returns:
    #         The Colburn j-factor [dimensionless].
    #     """
    #     import numpy as np

    #     # Safety clamps for Logarithms to prevent domain errors
    #     Re_Dc = max(Re_Dc, 10.0)
    #     ln_Re = np.log(Re_Dc)

    #     N=1

    #     # --- Heat Transfer (j-factor) ---
    #     if N == 1:
    #         P1 = 1.9 - 0.23 * ln_Re
    #         P2 = -0.236 + 0.126 * ln_Re
            
    #         j = (0.108 * (Re_Dc**-0.29) * ((P_t / P_l)**P1) * ((F_p / D_c)**-1.084) * ((F_p / D_h)**-0.786) * ((F_p / P_t)**P2))
    #     else:
    #         P3 = -0.361 - (0.042 * N / ln_Re) + 0.158 * np.log(N * (F_p / D_c)**0.41)
    #         P4 = -1.224 - (0.076 * ((P_l / D_h)**1.42) / ln_Re)
    #         P5 = -0.083 + (0.058 * N / ln_Re)
    #         P6 = -5.735 + 1.21 * np.log(Re_Dc / N)

    #         j = (0.086 * (Re_Dc**P3) * (N**P4) * ((F_p / D_c)**P5) * ((F_p / D_h)**P6) * ((F_p / P_t)**-0.93))

    #     return j



# HOW TO USE IT:

    
# # Geometry / Reynolds Number
# D_h = 2 * space_between_frost 
# D_c_eff = tube_outer_diameter + 2 * fin_thickness + 2 * frost_thickness

# Re_Dc = self._calculate_reynolds_number(
#     density=density_avg, 
#     velocity=velocity, 
#     characteristic_length=D_c_eff, 
#     dyn_viscosity=dyn_viscosity_avg
# )

# # Heat Transfer Coefficients (Wang correlations)
# j_factor = self._calculate_wang_correlations(
#     Re_Dc=Re_Dc,
#     N=tube_layers,
#     F_p=fin_pitch,
#     D_c=D_c_eff,
#     D_h=D_h,
#     P_t=P_t,
#     P_l=P_l
# )

# h_conv = (j_factor * density_avg * velocity * heat_capacity_avg) / (prandtl_avg ** (2/3))