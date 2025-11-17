from .datamodels_nba import (
    FrostEvaporatorParameters, 
    FrostEvaporatorInputs, 
    FrostEvaporatorState,
)
from vclibpy.media import RefProp
import numpy as np
import os

# At the top of your file
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



class RefrigerantModel:
    """
    # TODO Docstring
    """
    
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
        Calculates and updates density and k_frost based on the state.
        
        This IS safe to call inside an iterative loop, as it just recalculates
        properties based on the latest guessed values.
        """
        
        refrigerant_props = self.calculate_refrigerant_props(
            h_in  = inputs.refrigerant.h_in,
            h_out = state.refrigerant.h_out,
            p_eva  = inputs.refrigerant.p_eva,
        )

        h_conv = self.calculate_heat_transfer_coefficient(
            refrigerant_props = refrigerant_props,
            m_flow = inputs.refrigerant.m_flow,
            h_in = inputs.refrigerant.h_in,
            h_out = state.refrigerant.h_out
        )

        state.refrigerant.set("h_conv", h_conv)
        state.refrigerant.set("T_in", refrigerant_props['temperature_in'])
        state.refrigerant.set("T_out", refrigerant_props['temperature_out'])



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
        
        refrigerant_props = {}
        
        # --- Define Average State ---
        p_avg = p_eva
        h_avg = (h_in + h_out) / 2
        
        refrigerant_props['pressure_avg'] = p_avg
        refrigerant_props['enthalpy_avg'] = h_avg
        
        # --- Calculate Properties at Average State (P_avg, h_avg) ---
        avg_state = self.RP.calc_state("PH", p_avg, h_avg)
        avg_trans_prop = self.RP.calc_transport_properties(state=avg_state)
    
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
        liq_trans_prop = self.RP.calc_transport_properties(state=liq_state)

        refrigerant_props['density_liquid'] = liq_state.d
        refrigerant_props['enthalpy_liquid'] = liq_state.h
        refrigerant_props['dyn_viscosity_liquid'] = liq_trans_prop.dyn_vis
        refrigerant_props['kin_viscosity_liquid'] = liq_trans_prop.kin_vis
        refrigerant_props['thermal_conductivity_liquid'] = liq_trans_prop.lam
        refrigerant_props['prandtl_liquid'] = liq_trans_prop.Pr
        
        # Saturated Vapor (Q=1)
        vap_state = self.RP.calc_state("PQ", p_avg, 1)
        vap_trans_prop = self.RP.calc_transport_properties(state=vap_state)

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



    def calculate_heat_transfer_coefficient(self, refrigerant_props: dict, m_flow: float, h_in: float, h_out: float) -> float:
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
        A_c = (np.pi * d_h**2) / 4.0  
        G = m_flow / A_c               # Mass flux [kg/m^2/s]

        # Dispatch based on flow regime
        if quality <= 0.0 or quality >= 1.0:
            # Single-Phase (Liquid or Vapor)
            return self._calculate_single_phase_htc(refrigerant_props, G, d_h)
        
        else:
            # Two-Phase Evaporation
            return self._calculate_two_phase_htc(refrigerant_props, G, d_h, m_flow, h_in, h_out)
        


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
            raise ValueError(f"Prandtl number ({Pr}) out of range [0.5, 2000] for Gnielinski correlation.")

        zeta = (0.79 * np.log(Re) - 1.64)**(-2.0)
        numerator = (zeta / 8.0) * (Re - 1000) * Pr
        denominator = 1.0 + 12.7 * np.sqrt(zeta / 8.0) * (Pr**(2.0/3.0) - 1.0)
        
        if denominator <= 1e-6: # Check for zero or negative denominator
            # Fallback to Dittus-Boelter (Eq 4.26)
            Nu_turb = 0.023 * Re**0.8 * Pr**(0.4)
        else:
            Nu_turb = numerator / denominator

        # --- 4. Select or Interpolate Nu ---
            
        if Re >= Re_turb_min:
            # Fully Turbulent
            return Nu_turb
            
        else:
            # Interpolation between Laminar and Turbulent
            w = (Re - Re_lam_max) / (Re_turb_min - Re_lam_max)
            return (1.0 - w) * Nu_lam + w * Nu_turb
    


    def _calculate_two_phase_htc(self, props: dict, G: float, d_h: float, m_flow: float, h_in: float, h_out: float) -> float:
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

            A_surface = np.pi * d_h * L_tube  # Wetted surface area [m^2]
            Q_dot = m_flow * (h_out - h_in)   # Total heat transfer [W]
            q_dot = Q_dot / A_surface         # Heat flux [W/m^2]

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

        # Calculate alpha_K (Eq 4.40)
        Re_l = G * (1.0 - q_avg) * d_h / dyn_vis_l
        alpha_K = (lambda_l / d_h) * 0.023 * Re_l**0.8 * Pr_l**0.4
        
        # Calculate final alpha (Eq 4.39)        
        return 3.9 * Fr**0.24 * (q_avg / (1.0 - q_avg))**0.64 * (rho_l / rho_g)**0.4 * alpha_K


    def _calculate_chen_htc(self, props: dict, G: float, d_h: float, q_dot: float) -> float:
        """
        Implements Chen (1966) correlation (Eq 4.41 - 4.46) for Fr >= 0.04.
        """
        # --- 1. Get properties ---
        q_avg = props['quality_avg']
        rho_l = props['density_liquid']
        rho_g = props['density_vapor']
        lambda_l = props['thermal_conductivity_liquid']
        dyn_vis_l = props['dyn_viscosity_liquid']
        dyn_vis_g = props['dyn_viscosity_vapor']
        Pr_l = props['prandtl_liquid']
        h_l = props['enthalpy_liquid']
        h_g = props['enthalpy_vapor']

        # --- 2. Calculate alpha_K (Convective Boiling Term) ---
        Re_l = G * (1.0 - q_avg) * d_h / dyn_vis_l
        Nu_l = self._calculate_single_phase_nusselt(Re_l, Pr_l) # REUSE
        alpha_K = (Nu_l * lambda_l) / d_h

        # Calculate alpha_B (Bulk Boiling Term)
        alpha_B = self._calculate_bulk_boiling_htc(props, q_dot)

        # Calculate X_tt (Eq 4.44)
        X_tt = ((1.0 - q_avg) / q_avg)**0.9 * (dyn_vis_l / dyn_vis_g)**0.1 * (rho_g / rho_l)**0.5

        # Calculate Bo (Eq 4.44)
        Bo = q_dot / (G * (h_g - h_l))

        # Calculate F (Eq 4.42)
        F = 1.0 + (2.4e4 * Bo**1.16) + (1.37 * X_tt**(-0.86))
        
        # Calculate S (Suppression Factor) (Eq 4.43)
        S = (1.0 + 1.15e-6 * F**2 * Re_l**1.17)**(-1.0)

        # 8. Combine (Eq 4.41)
        return S * alpha_B + F * alpha_K


    def _calculate_bulk_boiling_htc(self, props: dict, q_dot: float) -> float:
        """
        Implements Stephan (1988) correlation for alpha_B (Eq 4.45, 4.46).
        """
        # --- 1. Get properties ---
        p_red = props["pressure_avg"] / props['pressure_critical'] # p / p_critical
        
        alpha_0 = self.params.alpha_0
        q_dot_0 = self.params.q_dot_0

        # Calculate F_pred (Eq 4.46)
        F_pred = 2.1 * p_red**0.27 + (4.4 + 1.8 / (1.0 - p_red)) * p_red
        
        # Calculate n (Eq 4.46)
        n = 0.9 - 0.3 * p_red**0.3
        
        # Calculate alpha_B (Eq 4.45)
        return alpha_0 * F_pred * (q_dot / q_dot_0)**n