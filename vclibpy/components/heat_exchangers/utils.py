import numpy as np

from vclibpy import Inputs
from vclibpy.components.heat_exchangers import HeatExchanger

from vclibpy.media import ThermodynamicState


def calc_area(Q, k, Tm):
    if Tm * k == 0:
        return 1e10  # A really large number, but not np.inf to still be able to calculate with it
    else:
        return Q / (k * Tm)


def calc_mean_temperature(
        T_hot_in: float, T_hot_out: float,
        T_cold_in: float, T_cold_out: float,
        flow_type: str = "counter"
):
    if T_hot_in < T_hot_out or T_cold_out < T_cold_in:
        raise ValueError(
            "Temperatures profiles are not valid for LTMD. "
            "Cold stream should heat up and hot stream should cool down."
        )

    if flow_type == "counter":
        dT_A = T_hot_in - T_cold_out
        dT_B = T_hot_out - T_cold_in
    elif flow_type == "parallel":
        dT_A = T_hot_in - T_cold_in
        dT_B = T_hot_out - T_cold_out
    else:
        raise TypeError("Given flow_type is not supported yet")
    if dT_B < 0 or dT_A < 0:
        return 0  # Heat can't be transferred

    # Handle special cases
    if np.isclose(dT_A, dT_B, rtol=1e-5):
        return dT_A

    if np.isclose(dT_A, 0, rtol=1e-2) or np.isclose(dT_B, 0, rtol=1e-2):
        return min(dT_A, dT_B)  # No heat transfer if either side is zero

    return (dT_A - dT_B) / np.log(dT_A / dT_B)


def separate_phases(m_flow, med_prop, state_max: ThermodynamicState, state_min: ThermodynamicState, p: float):
    """
    Separates a flow with possible phase changes into three parts:
    subcooling (sc), latent phase change (lat), and superheating (sh)
    at the given pressure.

    Args:
        state_max (ThermodynamicState): State with higher enthalpy.
        state_min (ThermodynamicState): State with lower enthalpy.
        p (float): Pressure of phase change.

    Returns:
        Tuple[float, float, float, ThermodynamicState, ThermodynamicState]:
            Q_sc: Heat for subcooling.
            Q_lat: Heat for latent phase change.
            Q_sh: Heat for superheating.
            state_q0: State at vapor quality 0 and the given pressure.
            state_q1: State at vapor quality 1 and the given pressure.
    """
    # Get relevant states:
    state_q0 = med_prop.calc_state("PQ", p, 0)
    state_q1 = med_prop.calc_state("PQ", p, 1)
    Q_sc = max(0.0,
               min((state_q0.h - state_min.h),
                   (state_max.h - state_min.h))) * m_flow
    Q_lat = max(0.0,
                (min(state_max.h, state_q1.h) -
                 max(state_min.h, state_q0.h))) * m_flow
    Q_sh = max(0.0,
               min((state_max.h - state_q1.h),
                   (state_max.h - state_min.h))) * m_flow
    return Q_sc, Q_lat, Q_sh, state_q0, state_q1


def get_condenser_phase_temperatures_and_alpha(
        inputs: Inputs,
        Q_sc: float, Q_lat: float, Q_sh: float,
        heat_exchanger: HeatExchanger
):
    cp = heat_exchanger.m_flow_secondary_cp
    Q = Q_sc + Q_lat + Q_sh

    # Calculate secondary_medium side temperatures:
    # Assumption loss is the same correlation for each regime

    # Calculate secondary_medium side temperatures:
    # Assumption loss is the same correlation for each regime
    if inputs.condenser.uses_inlet:
        T_in = inputs.condenser.T_in
        T_sc = inputs.condenser.T_in + heat_exchanger.calc_secondary_Q_flow(Q_sc) / cp
        T_sh = T_sc + heat_exchanger.calc_secondary_Q_flow(Q_lat) / cp
        T_out = T_sh + heat_exchanger.calc_secondary_Q_flow(Q_sh) / cp
    else:
        T_out = inputs.condenser.T_out
        T_sh = T_out - heat_exchanger.calc_secondary_Q_flow(Q_sh) / cp
        T_sc = T_sh - heat_exchanger.calc_secondary_Q_flow(Q_lat) / cp
        T_in = T_sc - heat_exchanger.calc_secondary_Q_flow(Q_sc) / cp
    return T_in, T_sc, T_sh, T_out


def plot_lmtd_vs_cold_out(T_hot_in, T_hot_out, dT_cold):
    """
    Plot LMTD over decreasing T_cold_out for counter flow.

    Args:
        T_hot_in: Hot fluid inlet temperature (°C)
        T_hot_out: Hot fluid outlet temperature (°C)
        dT_cold: Cold fluid temperature difference (K)
    """
    import matplotlib.pyplot as plt

    # Create array of T_cold_out values from T_hot_in down to below T_cold_in
    dT_min_values = np.arange(-5, 2, 0.0001)
    T_cold_out_values = T_hot_in + dT_min_values

    # Calculate LMTD for each T_cold_out
    lmtd_values = [
        calc_mean_temperature(T_hot_in, T_hot_out, T_cold_out - dT_cold, T_cold_out, "counter")
        for T_cold_out in T_cold_out_values
    ]

    # Create the plot
    plt.figure(figsize=(10, 6))
    plt.plot(dT_min_values, lmtd_values, 'b-', linewidth=2)

    # Add vertical lines for reference temperatures
    plt.axvline(x=T_hot_in, color='red', linestyle='--', label='T_hot_in')
    plt.axvline(x=T_hot_out, color='blue', linestyle='--', label='T_hot_out')

    # Customize plot
    plt.xlabel('$T_\mathrm{cold,out} - T_\mathrm{hot,in}$ in K')
    plt.ylabel('$\Delta T_\mathrm{LMTD}$ in K')
    plt.title('Log Mean Temperature Difference vs Cold Outlet Temperature\n'
              f'(dT_hot={T_hot_in-T_hot_out} K, dT_cold={dT_cold} K)')
    plt.legend()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    plot_lmtd_vs_cold_out(T_hot_in=0, T_hot_out=-1, dT_cold=10)
