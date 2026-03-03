from ..datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
from vclibpy.media import RefProp
import math
import os

# Get path from environment variable, fall back to a default if not set
REFPROP_DIR = os.environ.get("REFPROP_PATH", r"C:\Program Files (x86)\REFPROP")
REFPROP_DLL = os.path.join(REFPROP_DIR, "REFPRP64.DLL")

# Check if the path actually exists
if not os.path.exists(REFPROP_DLL):
    raise FileNotFoundError(
        f"Could not find REFPRP64.DLL. "
        f"Looked in: {REFPROP_DLL}. "
        "Please set the 'REFPROP_PATH' environment variable."
    )

os.environ["RPPREFIX"] = REFPROP_DIR

propane_RP = RefProp(
        fluid_name="Propane",
        dll_path=REFPROP_DLL,
        ref_prop_path=REFPROP_DIR
)
R134a_RP = RefProp(
        fluid_name="R134a",
        dll_path=REFPROP_DLL,
        ref_prop_path=REFPROP_DIR
)
R410a_RP = RefProp(
    fluid_name="R32.FLD|R125.FLD",       
    z=[0.697615, 0.302385],         # Exact molar composition for R410A
    dll_path=REFPROP_DLL, 
    ref_prop_path=REFPROP_DIR,
    copy_dll=False
)



class RefrigerantModel:
    
    def __init__(self, 
                 parameters: FrostEvaporatorParameters, 
                 refprop_instance: RefProp
                 ):
        """
        Initializes the model.
        
        Args:
            parameters: The (read-only) parameters object.
        """
        self.params = parameters
        self.RP     = refprop_instance  
        print(f"Model initialized with fluid: {self.RP.fluid_name}")

    def update_properties(self, state: FrostEvaporatorState, inputs: FrostEvaporatorInputs):
        """
        Calculates and updates the refrigerant model using a zonal approach.
        
        This method divides the evaporator into two zones:
        1. Two-Phase Zone
        2. Superheated Zone
        Depending on the inlet and outlet enthalpies, it determines which zones are present
        and calculates the heat transfer coefficients (HTC) for each zone accordingly.
        """
        
        # --- Extract Inputs ---
        h_in  = inputs.refrigerant.h_in
        h_out_current = state.refrigerant.h_out
        p_eva = inputs.refrigerant.p_eva
        m_dot = inputs.refrigerant.m_dot

        # Ensure h_out is at least the inlet enthalpy (prevent errors later on)
        h_out = max(h_out_current, h_in + 1e-3)
            
        # --- Update State Object ---
        # Global Inputs/Outputs
        props_global = self.calculate_refrigerant_props(h_in, h_out, p_eva)

        # --- Determine Saturation Boundary (h'') ---
        # Calculate saturated vapor enthalpy at current pressure (Quality = 1)
        sat_vapor_state = self.RP.calc_state("PQ", p_eva, 1.0)
        h_sat_vap = sat_vapor_state.h
        T_sat = sat_vapor_state.T

        # Initialize Variables
        portion_2ph = 0.0
        portion_sh = 0.0
        h_conv_2ph = 100.0 
        h_conv_sh  = 100.0 
        cp_avg_sh  = 1000.0
        
        T_in_sh  = props_global['temperature_in']
        T_in_2ph = props_global['temperature_in']

        # Get area split from HMT state
        f_2ph = state.hmt.area_split_2phase

        # Case A: Entirely Superheated (h_in > h'')
        if h_in >= h_sat_vap:
            portion_sh = 1.0
            h_conv_sh, cp_avg_sh = self._compute_zone_htc(h_in, h_out, p_eva, m_dot, area_fraction=1.0)

        # Case B: Entirely Two-Phase (h_out <= h'')
        elif h_out <= h_sat_vap:
            portion_2ph = 1.0
            h_conv_2ph, _ = self._compute_zone_htc(h_in, h_out, p_eva, m_dot, area_fraction=1.0)

        # Case C: Mixed / Transition (h_in < h'' < h_out)
        else:
            # Calculate Enthalpy Splits
            delta_h_2ph = h_sat_vap - h_in
            delta_h_sh  = h_out - h_sat_vap
            total_enthalpy_diff = h_out - h_in
            
            # Calculate Portions (Weighting by Heat Flow / Enthalpy change)
            portion_2ph = delta_h_2ph / total_enthalpy_diff
            portion_sh = delta_h_sh / total_enthalpy_diff
            
            # --- Two-Phase Zone (h_in -> h'') ---
            h_conv_2ph, _ = self._compute_zone_htc(h_in, h_sat_vap, p_eva, m_dot, area_fraction=f_2ph)
            
            # --- Superheated Zone (h'' -> h_out) ---
            h_conv_sh, cp_avg_sh = self._compute_zone_htc(h_sat_vap, h_out, p_eva, m_dot, area_fraction=(1.0 - f_2ph))
            T_in_sh = T_sat


        

        state.refrigerant.set("T_in", props_global['temperature_in'])
        state.refrigerant.set("T_out", props_global['temperature_out'])
        state.refrigerant.set("p_out", p_eva)

        # Zonal Outputs
        state.refrigerant.set("portion_two_phase", portion_2ph)
        state.refrigerant.set("portion_superheated", portion_sh)

        state.refrigerant.set("h_sat_vap", h_sat_vap)
        
        state.refrigerant.set("h_conv_two_phase", h_conv_2ph)
        state.refrigerant.set("h_conv_superheated", h_conv_sh)
        
        state.refrigerant.set("T_two_phase_in", T_in_2ph)
        state.refrigerant.set("T_superheated_in", T_in_sh)

        state.refrigerant.set("heat_capacity_avg_superheated", cp_avg_sh)

        state.refrigerant.set("quality_avg", props_global['quality_avg'])


    ####################################################################################
    # Helper Functions
    ####################################################################################


    def _compute_zone_htc(self, h_start: float, h_end: float, p_sys: float, m_dot: float, area_fraction: float = 1.0) -> tuple[float, float]:
        """
        Helper method to calculate HTC for a specific enthalpy range (zone).
        """
        # Calculate properties for this specific zone
        zone_props = self.calculate_refrigerant_props(
            h_in=h_start,
            h_out=h_end,
            p_eva=p_sys
        )
        
        # Calculate h_conv for this specific zone
        h_conv = self.calculate_heat_transfer_coefficient(
            refrigerant_props=zone_props,
            m_dot=m_dot,
            h_in=h_start,
            h_out=h_end,
            area_fraction=area_fraction
        )
    
        return h_conv, zone_props['heat_capacity_avg']


    def calculate_refrigerant_props(self, h_in: float, h_out: float, p_eva: float) -> dict:
        """
        Calculates all necessary refrigerant properties for an average 0D state.

        This function calculates properties for 5 states at the average pressure:
        1.  The average mixture state (at h_avg)
        2.  The saturated liquid state (Q=0)
        3.  The saturated vapor state (Q=1)
        4.  The inlet state (at h_in)
        5.  The outlet state (at h_out)

        This provides all values needed for common heat transfer correlations.

        Args:
            h_in: Refrigerant enthalpy at the inlet [J/kg]
            h_out: Refrigerant enthalpy at the outlet [J/kg]
            p_eva: Average evaporator pressure [Pa]

        Returns:
            A dictionary containing all calculated properties.
        """

        if h_out < 0 :
            h_out = h_in + 20_000
            print("Warning: h_out was negative in calculate_refrigerant_prop, setting h_out = h_in + 20kj/kg to avoid invalid calculation.")
        
        # --- Define Average State ---
        p_avg = p_eva
        h_avg = (h_in + h_out) / 2


        refrigerant_props = {'pressure_avg': p_avg, 
                             'enthalpy_avg': h_avg}     

        # --- Calculate Properties at Average State (P_avg, h_avg) ---
        avg_state = self.RP.calc_state("PH", p_avg, h_avg)
        avg_trans_prop = self.RP.calc_transport_properties(avg_state)


        refrigerant_props['quality_avg'] = avg_state.q
        refrigerant_props['density_avg'] = avg_state.d
        refrigerant_props['dyn_viscosity_avg'] = avg_trans_prop.dyn_vis
        refrigerant_props['heat_capacity_avg'] = avg_trans_prop.cp
        refrigerant_props['thermal_conductivity_avg'] = avg_trans_prop.lam
        refrigerant_props['prandtl_avg'] = avg_trans_prop.Pr

        # --- Calculate Saturated Phase Properties (at P_avg) ---
        # These are needed for two-phase correlations (e.g., X_tt)

        # Saturated Liquid (Q=0)
        liq_state = self.RP.calc_state("PQ", p_avg, 0)
        liq_trans_prop = self.RP.calc_transport_properties(liq_state)


        refrigerant_props['density_liquid'] = liq_state.d
        refrigerant_props['enthalpy_liquid'] = liq_state.h
        refrigerant_props['dyn_viscosity_liquid'] = liq_trans_prop.dyn_vis
        refrigerant_props['kin_viscosity_liquid'] = liq_trans_prop.kin_vis
        refrigerant_props['thermal_conductivity_liquid'] = liq_trans_prop.lam
        refrigerant_props['prandtl_liquid'] = liq_trans_prop.Pr

        # Saturated Vapor (Q=1)
        vap_state = self.RP.calc_state("PQ", p_avg, 1)
        vap_trans_prop = self.RP.calc_transport_properties(vap_state)

        refrigerant_props['density_vapor'] = vap_state.d
        refrigerant_props['enthalpy_vapor'] = vap_state.h
        refrigerant_props['dyn_viscosity_vapor'] = vap_trans_prop.dyn_vis
        refrigerant_props['kin_viscosity_vapor'] = vap_trans_prop.kin_vis


        # Critical Properties
        refrigerant_props['pressure_critical'] = self.RP.get_critical_point()[1]


        # --- Calculate Inlet and Outlet States (at P_avg) ---
        in_state = self.RP.calc_state("PH", p_avg, h_in)
        out_state = self.RP.calc_state("PH", p_avg, h_out)

        refrigerant_props['temperature_in'] = in_state.T
        refrigerant_props['temperature_out'] = out_state.T

        return refrigerant_props
    

    def calculate_heat_transfer_coefficient(self, refrigerant_props: dict, m_dot: float, h_in: float, h_out: float, area_fraction: float = 1.0) -> float:
        """
        Calculates the refrigerant-side heat transfer coefficient.

        Implements "AdvancedHeatTransfer Model" from Richter Dissertation:
        "Proposal of New Object-Oriented Equation-Based Model Libraries for Thermodynamic Systems"
        
        Dispatches to the correct correlation (single-phase or two-phase)
        based on the average vapor quality.
        """
        quality = refrigerant_props["quality_avg"]
        
        # Get common geometric parameters
        d_h = self.params.tube_inner_diameter 
        A_c = (math.pi * d_h**2) / 4.0  
        G = m_dot / A_c               # Mass flux [kg/m^2/s]

        # Calculate Single-Phase HTC (Pure Vapor) - We need this for the anchor point
        h_conv_1ph = self._calculate_single_phase_htc(refrigerant_props, G, d_h)

        # Dispatch based on flow regime
        # Pure Liquid or Pure Vapor
        if quality <= 0.0 or quality >= 1.0:
            return h_conv_1ph
            
        # Two-Phase Evaporation
        else:
            h_conv_2ph_raw = self._calculate_two_phase_htc(refrigerant_props, G, d_h, m_dot, h_in, h_out, area_fraction)
            
            # Smooth Dryout Transition
            x_dryout_start = 0.85
            if quality > x_dryout_start:
                w = (quality - x_dryout_start) / (1.0 - x_dryout_start)
                
                # Linear Interpolation: (1-w)*Boiling + w*Vapor
                h_conv_effective = (1.0 - w) * h_conv_2ph_raw + w * h_conv_1ph
                return h_conv_effective
            else:
                return h_conv_2ph_raw


    def _calculate_single_phase_htc(self, props: dict, G: float, d_h: float) -> float:
        """
        Calculates the heat transfer coefficient for single-phase (liquid or vapor) flow.
        """
        # 1. Get properties
        dyn_viscosity = props['dyn_viscosity_avg']
        Pr = props['prandtl_avg']
        thermal_conductivity = props['thermal_conductivity_avg']

        # Calculate Reynolds Number (Re) 
        Re = (G * d_h) / dyn_viscosity

        # Call the reusable utility function
        Nu = self._calculate_single_phase_nusselt(Re, Pr)
        
        # Calculate and return HTC
        return (Nu * thermal_conductivity) / d_h



    def _calculate_single_phase_nusselt(self, Re: float, Pr: float) -> float:
        """
        Calculates the Nusselt number for single-phase flow.
        
        - Uses constant Nu for Laminar flow (Eq 4.23)
        - Uses Gnielinski correlation for turbulent flow (Eq 4.24, 4.25)
        - Uses linear interpolation in the transition region (Re 2300-4000)
          to ensure a smooth, continuous function for the solver.
        """
        
        #  Define Transition Boundaries
        Re_lam_max = 2300.0  # End of fully laminar
        Re_turb_min = 4000.0 # Start of fully turbulent
        
        # Laminar Nu (Eq 4.23)
        Nu_lam = 3.6568

        if Re <= Re_lam_max:
            # Fully Laminar
            return Nu_lam

        # Calculate Turbulent Nu (Eq 4.24, 4.25)
        # Check Prandtl range validity
        if Pr < 0.5 or Pr > 2000:
            # raise ValueError(f"Prandtl number ({Pr}) out of range [0.5, 2000] for Gnielinski correlation.")
            print("Warning: Prandtl number out of range for Gnielinski correlation. Returning laminar Nusselt number as fallback.")
            return Nu_lam # Fallback to laminar value if Pr is out of range

        zeta = (0.79 * math.log(Re) - 1.64)**(-2.0)
        numerator = (zeta / 8.0) * (Re - 1000) * Pr
        denominator = 1.0 + 12.7 * math.sqrt(zeta / 8.0) * (Pr**(2.0/3.0) - 1.0)
        
        if denominator <= 1e-6: # Check for zero or negative denominator
            # Fallback to Dittus-Boelter (Eq 4.26)
            Nu_turb = 0.023 * Re**0.8 * Pr**(0.4)
        else:
            Nu_turb = numerator / denominator

        # --- 4. Select or Interpolate Nu ---
            
        if Re >= Re_turb_min:
            # Fully Turbulent
            return Nu_turb
            
        # Interpolation between Laminar and Turbulent
        w = (Re - Re_lam_max) / (Re_turb_min - Re_lam_max)
        return (1.0 - w) * Nu_lam + w * Nu_turb
    


    def _calculate_two_phase_htc(self, props: dict, G: float, d_h: float, m_dot: float, h_in: float, h_out: float, area_fraction: float = 1.0) -> float:
        """
        Dispatches to the correct evaporation correlation based on Froude number.
        """
        # --- 1. Get required properties ---
        rho_l = props['density_liquid']
        g = self.params.gravity
        
        # --- 2. Calculate Froude Number (Fr) (Eq 4.38) ---
        Fr = (G**2) / (rho_l**2 * g * d_h)

        # --- 3. Branch based on Froude number ---
        if Fr < 0.04:
            # Use Shah (1976) correlation
            return self._calculate_shah_htc(props, G, d_h, Fr)
        else:
            # Use Chen (1966) correlation
            L_tube = self.params.tube_length

            # Multiply total area by the zone's area fraction
            A_surface = math.pi * d_h * L_tube * max(1e-5, area_fraction)  
            
            Q_dot = m_dot * (h_out - h_in)   # Total heat transfer [W]
            q_dot = Q_dot / A_surface        # Heat flux [W/m^2]

            return self._calculate_chen_htc(props, G, d_h, q_dot)

    
    def _calculate_shah_htc(self, props: dict, G: float, d_h: float, Fr: float) -> float:
        """
        Implements Shah (1976) correlation (Eq 4.39, 4.40) for Fr < 0.04.
        """
        # Get properties
        q_avg = props['quality_avg']
        rho_l = props['density_liquid']
        rho_g = props['density_vapor']
        lambda_l = props['thermal_conductivity_liquid']
        dyn_vis_l = props['dyn_viscosity_liquid']
        Pr_l = props['prandtl_liquid']

        # Calculate h_conv_K (Eq 4.40)
        Re_l = G * (1.0 - q_avg) * d_h / dyn_vis_l
        h_conv_K = (lambda_l / d_h) * 0.023 * Re_l**0.8 * Pr_l**0.4
        
        # Calculate final h_conv (Eq 4.39)        
        return 3.9 * Fr**0.24 * (q_avg / (1.0 - q_avg))**0.64 * (rho_l / rho_g)**0.4 * h_conv_K


    def _calculate_chen_htc(self, props: dict, G: float, d_h: float, q_dot: float) -> float:
        """
        Implements the Standard Chen (1966) correlation.
        
        Corrections based on Chen & Fang (2014) paper:
        - F Factor: Uses Eq (6) (Best fit for Chen's graph)
        - S Factor: Uses Eq (9) (Corrects common literature typos)
        http://dx.doi.org/10.1016/j.ijrefrig.2014.09.008
        """
        # --- 1. Get properties ---
        q_avg = props['quality_avg']
        rho_l = props['density_liquid']
        rho_g = props['density_vapor']
        lambda_l = props['thermal_conductivity_liquid']
        dyn_vis_l = props['dyn_viscosity_liquid']
        dyn_vis_g = props['dyn_viscosity_vapor']
        Pr_l = props['prandtl_liquid']
        
        # --- 2. Calculate Reference Liquid HTC (h_conv_sp,l) ---
        Re_l = G * (1.0 - q_avg) * d_h / dyn_vis_l
        Nu_l = 0.023 * (Re_l**0.8) * (Pr_l**0.4)
        h_conv_K = (Nu_l * lambda_l) / d_h

        # --- 3. Calculate Martinelli Parameter (X_tt) ---

        # Eq 5a in paper
        X_tt = ((1.0 - q_avg) / q_avg)**0.9 * (rho_g / rho_l)**0.5 * (dyn_vis_l / dyn_vis_g)**0.1
        inv_X_tt = 1.0 / X_tt

        # --- 4. Calculate Reynolds Number Factor (F) - Eq (6) ---
        if inv_X_tt <= 0.1:
            F = 1.0
        else:
            F = 2.35 * ((inv_X_tt + 0.213)**0.736)

        # --- 5. Calculate Suppression Factor (S) - Eq (9) ---
        Re_tp = Re_l * (F**1.25)
        S = 1.0 / (1.0 + 2.53e-6 * (Re_tp**1.17))

        # --- 6. Calculate Boiling Component (h_conv_nb) ---
        h_conv_B = self._calculate_bulk_boiling_htc(props, q_dot)

        # --- 7. Final Summation (Eq 1) ---
        return (S * h_conv_B) + (F * h_conv_K)

    def _calculate_bulk_boiling_htc(self, props: dict, q_dot: float) -> float:
        """
        Implements Stephan (1988) correlation for h_conv_B (Eq 4.45, 4.46).
        """
        # --- 1. Get properties ---
        p_red = props["pressure_avg"] / props['pressure_critical'] # p / p_critical
        
        h_conv_0 = self.params.h_conv_0
        q_dot_0 = self.params.q_dot_0

        # Calculate F_pred (Eq 4.46)
        F_pred = 2.1 * p_red**0.27 + (4.4 + 1.8 / (1.0 - p_red)) * p_red
        
        # Calculate n (Eq 4.46)
        n = 0.9 - 0.3 * p_red**0.3

        if q_dot<=0:
            print("Warning: q_dot <= 0 in _calculate_bulk_boiling_htc, setting to small positive value to avoid invalid calculation.")
            q_dot = 1e-3
        
        # Calculate h_conv_B (Eq 4.45)
        return h_conv_0 * F_pred * (q_dot / q_dot_0)**n