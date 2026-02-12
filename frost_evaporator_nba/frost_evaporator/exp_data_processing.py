import numpy as np
import pandas as pd
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional, Any, Tuple
from scipy.interpolate import interp1d
from scipy.stats import binned_statistic, linregress

# ==============================================================================
#  CONFIGURATION & CONSTANTS
# ==============================================================================
# --- Regression Degrees ---
TREND_DEGREES = {
    'mass': 5,
    'p': 5,
    'h': 5,
    'temp': 0,
    'rh': 0
}

# --- Column Mappings ---
COLUMN_MAPPINGS = {
    'OptiAbt': {
        'time':    ['time'],
        'modus':   ['modus_val', 'modus'],
        'fan':     ['VD_n2'],
        'mass':    ['MSS_rMassenstrom'],
        'p_in':    ['VD_p_out'],
        'h_in':    ['VD_h_in_korr', 'VD_isenthalp_h_in'],
        'temp_kk': ['TempKK'],
        'rh_kk':   ['KK_rlFeuchteKK', 'rlFeuchteKK'],
        'h_out':   ['VD_h_out'],
        'dp':      ['Delta_P_VD'],
        'mass_raw':['WAAGEN_Waage1_Masse_smooth', 'WAAGEN_Waage2_Masse']
    },
    'OptiHorst': {
        'time':    ['time'],
        'modus':   ['modus_val'],
        'fan':     ['StateMachine_rps_EvapFan'],
        'mass':    ['StateMachine_m_CompOut'],
        'p_in':    ['StateMachine_p_EvapOut'],
        'h_in':    ['StateMachine_h_EvapIn'],
        'temp_kk': ['KK_Temp_KK'],
        'rh_kk':   ['KK_rlFeuchte_KK'],
        'h_out':   ['StateMachine_h_EvapOut'],
        'dp':      ['StateMachine_p_PresLos'],
        'mass_raw':['WAAGEN_Waage2_Masse']
    }
}


# ==============================================================================
# Data Loader Class
# ==============================================================================
@dataclass
class ExperimentInputs:
    id: str
    duration: float
    trends: Dict[str, Callable[[float], float]]


@dataclass
class ComparisonData:
    """Holds cleaned time-series data for reporting/plotting."""
    id: int
    time: np.ndarray             # Normalized time in minutes (0 to end)
    
    # Processed Data Arrays (aligned with time)
    n_fan: np.ndarray            # RPM
    T_in: np.ndarray             # °C
    rh_in: np.ndarray            # %
    m_ref: np.ndarray            # kg/s
    p_ref_in: np.ndarray         # bar
    h_ref_in: np.ndarray         # kJ/kg
    h_ref_out: np.ndarray        # kJ/kg
    dp: np.ndarray               # Pa
    Q: np.ndarray                # W
    m_frost: np.ndarray          # g (Zeroed)
    
    # Special fields for OptiHorst defrost/plateau detection
    time_full: Optional[np.ndarray] = None
    m_frost_full: Optional[np.ndarray] = None

# ==============================================================================
#  ANALYZER CLASS
# ==============================================================================

class MultiExperimentAnalyzer:
    def __init__(self, project_root_rel: str = '..', experiment_type: str = 'OptiAbt'):
        """
        Initialize configuration and setup paths.
        
        :param project_root_rel: Relative path to project root.
        :param experiment_type: 'OptiAbt' or 'OptiHorst'. Determines column mapping and logic.
        """
        if experiment_type not in COLUMN_MAPPINGS:
            raise ValueError(f"Unknown experiment type: {experiment_type}")
        
        self.exp_type = experiment_type
        self.cols = COLUMN_MAPPINGS[experiment_type]


    # ===========================================================================
    #  Helper Functions
    # ===========================================================================

    @staticmethod
    def get_col(df: pd.DataFrame, df_fallback: pd.DataFrame, keys: List[str]) -> Optional[pd.Series]:
        """
        Retrieves column checking multiple keys and two DataFrames (Main and KK).
        Returns None if not found.
        """
        for k in keys:
            if k in df.columns: return df[k]
            if k in df_fallback.columns: return df_fallback[k]
        return None

    @staticmethod
    def polynomial_fit(x: np.ndarray, y: np.ndarray, degree: int = 1) -> Callable[[float], float]:
        """
        Fits a polynomial of specified degree to the data (x, y).
        """
        if len(x) == 0: return lambda t: 0.0
        coeffs = np.polyfit(x, y, degree)
        return np.poly1d(coeffs)

    @staticmethod
    def hard_step_avg(x: np.ndarray, y: np.ndarray, interval_minutes: float = 3.0) -> Callable:
        """
        Calculates a hard step average (piecewise constant) of y over x with specified interval in minutes.
        """
        if len(x) == 0: return lambda t: 0.0
        
        t_max = np.max(x)
        # Ensure bins cover the full range
        bins = np.arange(0, t_max + interval_minutes + 1e-9, interval_minutes)
        
        # Calculate statistics
        bin_means, bin_edges, _ = binned_statistic(x, y, statistic='mean', bins=bins)
        
        # Clean NaNs
        s_means = pd.Series(bin_means).ffill().bfill()
        
        # Correction: Carry forward second-to-last bin to avoid edge drop-off
        if len(s_means) > 1:
            s_means.iloc[-1] = s_means.iloc[-2]

        return interp1d(
            bin_edges[:-1], 
            s_means.values, 
            kind='zero', 
            fill_value="extrapolate", 
            bounds_error=False
        )


    def calculate_implicit_mode(self, df: pd.DataFrame, df_kk: pd.DataFrame) -> pd.Series:
        """
        Calculates valid mode based on Fan RPM if explicit 'modus' column is missing or invalid.
        """
        # Try to get Fan Speed
        # Uses self.cols, so this must be an instance method
        fan_raw = self.get_col(df, df_kk, self.cols['fan']) 
        
        if fan_raw is None:
            # Fallback: Assume everything is valid if no fan data exists
            return pd.Series(1, index=df.index)

        fan_vals = pd.to_numeric(fan_raw, errors='coerce').fillna(0)
        
        # Check if running (Threshold > 1 to account for noise/rps/rpm differences)
        is_running = fan_vals > 1
        
        if not is_running.any():
            return pd.Series(0, index=df.index)

        # Find longest continuous group
        group_ids = (is_running != is_running.shift()).cumsum()
        active_groups = group_ids[is_running]
        
        if active_groups.empty:
             return pd.Series(0, index=df.index)

        longest_group = active_groups.value_counts().idxmax()
        modus_series = (group_ids == longest_group).astype(int)
        
        # Trim Start/End (cleanup artifacts)
        valid_indices = modus_series[modus_series == 1].index
        if len(valid_indices) > 20:
            modus_series.loc[valid_indices[:10]] = 0 
            modus_series.loc[valid_indices[-10:]] = 0
            
        return modus_series
    
    def _load_and_mask_experiment(self, path: Path, exp_id: int) -> Tuple[Optional[pd.DataFrame], Optional[pd.DataFrame], Optional[pd.DataFrame], Optional[pd.DataFrame]]:
        """
        Standardized loading logic. 
        Returns (df_cut, df_kk_cut, df_raw, df_kk_raw).
        df_cut is strictly the data where Mode == 1 (or implicit active).
        """
        f_data = path / f"{exp_id}_data.csv"
        f_kk = path / f"{exp_id}_data_KK.csv"

        if not f_data.exists():
            return None, None, None, None

        df = pd.read_csv(f_data, sep=';', decimal='.', on_bad_lines='skip', low_memory=False)
        if f_kk.exists():
            df_kk = pd.read_csv(f_kk, sep=';', decimal='.', low_memory=False)
        else:
            df_kk = pd.DataFrame(index=df.index)

        # 1. Mode Detection
        modus_col = self.get_col(df, df_kk, self.cols['modus'])
        mask_mode = None

        if modus_col is not None:
            modus_num = pd.to_numeric(modus_col, errors='coerce').fillna(0)
            if modus_num.max() > 0:
                mask_mode = (modus_num == 1)

        # Fallback to implicit calculation
        if mask_mode is None or mask_mode.sum() == 0:
            mask_mode = (self.calculate_implicit_mode(df, df_kk) == 1)

        # 2. Slice DataFrames
        valid_idx = df.index[mask_mode]
        if not df_kk.empty:
            valid_idx = valid_idx.intersection(df_kk.index)

        if len(valid_idx) < 10:
             print(f"Warning: Filter removed all data for {exp_id}. Using full dataset.")
             valid_idx = df.index

        df_cut = df.loc[valid_idx].copy()
        df_kk_cut = df_kk.loc[valid_idx].copy()

        return df_cut, df_kk_cut, df, df_kk


    # ===========================================================================
    #  CORE ANALYSIS LOGIC
    # ===========================================================================

    def analyze(self, exp_ids: List[int], data_path: Path, cutoff_pct: float, time_step: float) -> Optional[ExperimentInputs]:
        
        combined_data = {'time_stable': [], 'fan': [], 'mass': [], 'p': [], 'h': [], 'temp': [], 'rh': []}   
        durations = []

        print(f"--- Aggregating Data ({self.exp_type}) for Experiments: {exp_ids} ---")

        path = Path(data_path)

        for exp_id in exp_ids:
            # REFACTORED: Use the centralized loader
            df, df_kk, _, _ = self._load_and_mask_experiment(path, exp_id)


            # --- Time Handling ---
            t_raw = self.get_col(df, df_kk, self.cols['time'])
            t_seconds = pd.to_numeric(t_raw, errors='coerce').values

            # Convert to minutes and align to start at 0
            t_minutes = (t_seconds - t_seconds[0]) / 60.0 # Minutes
            
            total_dur = np.max(t_minutes)
            durations.append(total_dur)
            
            # Trend Generation Cutoff (e.g. ignore last 2%)
            mask_stable = t_minutes >= (total_dur * cutoff_pct)

            # --- Data Extraction & Unit Normalization ---
            def get_val(keys):
                s = self.get_col(df, df_kk, keys)
                return pd.to_numeric(s, errors='coerce').fillna(0).values if s is not None else np.zeros(len(df))
            
            val_fan =  get_val(self.cols['fan'])
            val_mass = get_val(self.cols['mass'])
            val_p =    get_val(self.cols['p_in'])
            val_h =    get_val(self.cols['h_in'])
            val_temp = get_val(self.cols['temp_kk'])
            val_rh =   get_val(self.cols['rh_kk'])

            # Unit Conversions
            if self.exp_type == "OptiAbt":
                val_mass = val_mass / 1000.0 # Convert from g/s to kg/s
            if self.exp_type == "OptiAbt":
                val_h = val_h * 1000.0 # Convert from kJ/kg to J/kg
            if self.exp_type == "OptiHorst" or self.exp_type == "OptiAbt":
                val_fan = val_fan * 60.0 # Convert from rps to rpm

            # Store Data
            combined_data['time_stable'].append(t_minutes[mask_stable])
            combined_data['fan'].append(val_fan[mask_stable])
            combined_data['mass'].append(val_mass[mask_stable])
            combined_data['p'].append(val_p[mask_stable])
            combined_data['h'].append(val_h[mask_stable])
            combined_data['temp'].append(val_temp[mask_stable])
            combined_data['rh'].append(val_rh[mask_stable])

        t_stable = np.concatenate(combined_data['time_stable'])
        
        # Generate Trends
        trends = {
            'fan':  self.hard_step_avg(t_stable,  np.concatenate(combined_data['fan']), interval_minutes=time_step/60.0),
            'mass': self.polynomial_fit(t_stable, np.concatenate(combined_data['mass']), TREND_DEGREES['mass']),
            'p':    self.polynomial_fit(t_stable, np.concatenate(combined_data['p']), TREND_DEGREES['p']),
            'h':    self.polynomial_fit(t_stable, np.concatenate(combined_data['h']), TREND_DEGREES['h']),
            'temp': self.polynomial_fit(t_stable, np.concatenate(combined_data['temp']), TREND_DEGREES['temp']),
            'rh':   self.polynomial_fit(t_stable, np.concatenate(combined_data['rh']), TREND_DEGREES['rh']),
        }

        return ExperimentInputs(
            id=f"Combined_{len(exp_ids)}_{self.exp_type}", 
            duration=np.max(durations), 
            trends=trends
        )
    
    # ===========================================================================
    #  REPORTING DATA EXTRACTION
    # ===========================================================================

    def get_comparison_data(self, exp_ids: List[int], data_path: Path, sim_duration: Optional[float] = None) -> List[ComparisonData]:
        """
        Loads clean data for plotting, applying robust unit conversion heuristics and 
        simulation-time-based regression for frost mass.
        """
        results = []
        path = Path(data_path)

        for exp_id in exp_ids:
            # Load both CUT (stable) and RAW (full) dataframes
            df_cut, df_kk_cut, df_raw, df_kk_raw = self._load_and_mask_experiment(path, exp_id)
            if df_cut is None: continue

            # --- Extraction Helper ---
            def extract(df_m, df_k, col_key, default=0.0):
                val = self.get_col(df_m, df_k, self.cols[col_key])
                return pd.to_numeric(val, errors='coerce').fillna(default).values if val is not None else np.zeros(len(df_m))

            # --- 1. Time Normalization (Stable) ---
            t_raw = extract(df_cut, df_kk_cut, 'time')
            if len(t_raw) == 0: continue
            
            t_start_offset = t_raw[0] # Used to align the full dataset later
            t_minutes = (t_raw - t_start_offset) / 60.0

            # --- Raw Values ---
            n_fan       = extract(df_cut, df_kk_cut, 'fan')
            m_ref       = extract(df_cut, df_kk_cut, 'mass')
            h_in        = extract(df_cut, df_kk_cut, 'h_in')
            h_out       = extract(df_cut, df_kk_cut, 'h_out')
            m_frost_raw = extract(df_cut, df_kk_cut, 'mass_raw')

            # --- Unit Conversions ---
            if (self.exp_type == "OptiHorst" or self.exp_type == "OptiAbt"):
                if np.max(n_fan) < 100 and np.max(n_fan) > 0:
                    n_fan = n_fan * 60.0
            if self.exp_type == "OptiAbt":
                m_ref = m_ref / 1000.0  # g/s -> kg/s
            if self.exp_type == "OptiHorst":
                h_in = h_in / 1000.0 # J/kg -> kJ/kg
                h_out = h_out / 1000.0 # J/kg -> kJ/kg

            # --- Derived Calculations ---
            # Q [W] = m [kg/s] * delta_h [kJ/kg] * 1000 [W/kW]
            q_watts = m_ref * (h_out - h_in) * 1000

            # Use simulation duration for the regression window if provided
            t_basis = sim_duration if sim_duration is not None else np.max(t_minutes)
            
            # Calculate zeroed mass and the offset used
            m_frost_zeroed, m_frost_offset = self._calculate_frost_mass(t_minutes, m_frost_raw, t_basis)

            # 6. OptiHorst Specific: Full Data Extraction (Defrost Plateau)
            time_full = None
            m_frost_full = None
            
            if self.exp_type == "OptiHorst" and df_raw is not None:
                t_raw_full = extract(df_raw, df_kk_raw, 'time')
                m_raw_full = extract(df_raw, df_kk_raw, 'mass_raw')
                
                if len(t_raw_full) > 0:
                    # Align full time axis to the start of the cut data (t=0)
                    t_start = t_raw[0] if len(t_raw) > 0 else t_raw_full[0]
                    time_full = (t_raw_full - t_start) / 60.0
                    
                    # Apply the SAME offset from the stable region to the full data
                    m_frost_full = (m_raw_full - m_frost_offset) * 1000.0

            results.append(ComparisonData(
                id=exp_id,
                time=t_minutes,
                n_fan=n_fan,
                T_in=extract(df_cut, df_kk_cut, 'temp_kk'),
                rh_in=extract(df_cut, df_kk_cut, 'rh_kk'),
                m_ref=m_ref,
                p_ref_in=extract(df_cut, df_kk_cut, 'p_in'),
                h_ref_in=h_in,
                h_ref_out=h_out,
                dp=extract(df_cut, df_kk_cut, 'dp'),
                Q=q_watts,
                m_frost=m_frost_zeroed,
                time_full=time_full,
                m_frost_full=m_frost_full
            ))

        return results

    def _calculate_frost_mass(self, t_minutes: np.ndarray, m_raw: np.ndarray, t_max_ref: float) -> Tuple[np.ndarray, float]:
        """
        Calculates frost mass zeroing offset based on a 15-25% window of REFERENCE time (Simulation time).
        Uses Linear Regression to find the intercept.
        """
        if len(t_minutes) == 0: return m_raw, 0.0

        # Window based on REFERENCE time
        mask_reg = (t_minutes >= t_max_ref * 0.15) & (t_minutes <= t_max_ref * 0.25)

        offset = 0.0
        
        # Linear regression on the window
        res = linregress(t_minutes[mask_reg], m_raw[mask_reg])
        offset = res.intercept 

        m_grams = (m_raw - offset) * 1000.0
        return m_grams, offset