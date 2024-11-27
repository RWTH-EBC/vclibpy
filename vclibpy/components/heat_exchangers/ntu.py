import numpy as np

from vclibpy.components.heat_exchangers import HeatExchanger


def calc_Q_ntu(
        m_flow_primary_cp: float,
        m_flow_secondary_cp: float,
        k: float,
        dT_max: float,
        A: float,
        flow_type: str
) -> float:
    """
    Calculate the heat transfer and overall heat transfer coefficient for the
    heat exchanger based on NTU.

    Args:
        m_flow_primary_cp (float): Primary heat capacity rate
        m_flow_secondary_cp (float): Secondary heat capacity rate
        k (float): Heat transfer coefficient
        dT_max (float): Maximum temperature differential.
        A (float): Area of the heat exchanger.
        flow_type (str): Flow direction. Supported are counter, cross, or parallel

    Returns:
        Tuple[float, float]: Heat transfer and overall heat transfer coefficient.
    """
    R = calc_R(m_flow_primary_cp, m_flow_secondary_cp)
    m_flow_cp_min = calc_m_flow_cp_min(m_flow_primary_cp, m_flow_secondary_cp)
    NTU = calc_NTU(A, k, m_flow_cp_min)
    eps = calc_eps(R, NTU, flow_type=flow_type)

    # Get the maximal allowed heat flow
    Q_max = m_flow_cp_min * dT_max
    return Q_max * eps


def calc_R(m_flow_primary_cp, m_flow_secondary_cp) -> float:
    """
    Calculate the R value, which is the ratio of heat capacity rates
    (m_flow*cp) of the primary to secondary medium.

    Returns:
        float: R value.
    """
    if m_flow_primary_cp > m_flow_secondary_cp:
        return m_flow_secondary_cp / m_flow_primary_cp
    return m_flow_primary_cp / m_flow_secondary_cp


def calc_m_flow_cp_min(m_flow_primary_cp, m_flow_secondary_cp) -> float:
    """
    Calculate the minimum value between the heat capacity rates
    (m_flow*cp) for the primary and secondary mediums.

    Returns:
        float: Minimum value.
    """
    return min(m_flow_secondary_cp, m_flow_primary_cp)


def calc_NTU(A: float, k: float, m_flow_cp: float) -> float:
    """
    Calculate the Number of Transfer Units (NTU) for the heat exchanger.

    Args:
        A (float): Area of the heat exchanger.
        k (float): Overall heat transfer coefficient.
        m_flow_cp (float): Minimal heat capacity rates (m_flow*cp) between primary and secondary side.

    Returns:
        float: Number of Transfer Units (NTU).
    """
    return k * A / m_flow_cp


def calc_eps(R: float, NTU: float, flow_type: str) -> float:
    """
    Calculate the effectiveness (eps) of the heat exchanger based on NTU.

    Source of implementation: EBC Lecture SimModelle.

    Args:
        R (float): Ratio of heat capacity rates (m_flow*cp) of the primary to secondary medium.
        NTU (float): Number of Transfer Units.
        flow_type (str): Flow direction. Supported are counter, cross, or parallel

    Returns:
        float: Effectiveness (eps) of the heat exchanger.
    """
    if R in (0, 1):
        return NTU / (NTU + 1)
    if flow_type == "counter":
        return (1 - np.exp(-NTU * (1 - R))) / (1 - R * np.exp(-NTU * (1 - R)))
    if flow_type == "cross":
        if NTU == 0:
            return 0
        eta = NTU ** -0.22
        return 1 - np.exp((np.exp(- NTU * R * eta) - 1) / (R * eta))
    if flow_type == "parallel":
        return (1 - np.exp(-NTU * (1 + R))) / (1 + R)
    raise TypeError(f"Flow type {flow_type} not supported")


def iterate_area(
        m_flow_primary_cp: float,
        m_flow_secondary_cp: float,
        heat_exchanger: HeatExchanger,
        dT_max: float,
        k: float,
        Q: float
) -> float:
    """
    Iteratively calculates the required area for the heat exchange.

    Args:
        m_flow_primary_cp (float): Primary heat capacity rate
        m_flow_secondary_cp (float): Secondary heat capacity rate
        heat_exchanger (HeatExchanger): An instance of the BasicNTU or children classes
        dT_max (float): Maximum temperature differential.
        k (float): Heat transfer coefficient.
        Q (float): Heat flow rate.

    Returns:
        float: Required area for heat exchange.
    """
    _accuracy = 1e-6  # square mm
    _step = 1.0
    R = calc_R(m_flow_primary_cp, m_flow_secondary_cp)
    m_flow_cp_min = calc_m_flow_cp_min(m_flow_primary_cp, m_flow_secondary_cp)
    # First check if point is feasible at all
    if dT_max <= 0:
        return heat_exchanger.A
    eps_necessary = Q / (m_flow_cp_min * dT_max)

    # Special cases:
    # ---------------
    # eps is equal or higher than 1, an infinite amount of area would be necessary.
    if eps_necessary >= 1:
        return heat_exchanger.A
    # eps is lower or equal to zero: No Area required (Q<=0)
    if eps_necessary <= 0:
        return 0

    area = 0.0
    while True:
        if heat_exchanger.flow_type == "cross" and area == 0.0:
            eps = 0.0
        else:
            NTU = calc_NTU(area, k, m_flow_cp_min)
            eps = calc_eps(R, NTU, heat_exchanger.flow_type)
        if eps >= eps_necessary:
            if _step <= _accuracy:
                break
            else:
                # Go back
                area -= _step
                _step /= 10
                continue
        if _step < _accuracy and area > heat_exchanger.A:
            break
        area += _step

    return min(area, heat_exchanger.A)


def calc_Q_with_available_area(
        heat_exchanger: HeatExchanger,
        m_flow_primary_cp: float,
        m_flow_secondary_cp: float,
        Q_required: float,
        k: float,
        dT_max: float,
        A_available: float,
) -> (float, float):
    A_required = iterate_area(
        heat_exchanger=heat_exchanger,
        k=k,
        Q=Q_required,
        dT_max=dT_max,
        m_flow_primary_cp=m_flow_primary_cp,
        m_flow_secondary_cp=m_flow_secondary_cp
    )
    # Only use available area
    A_required = min(A_available, A_required)
    Q_achievable = calc_Q_ntu(
        dT_max=dT_max,
        k=k,
        A=A_required,
        m_flow_primary_cp=m_flow_primary_cp,
        m_flow_secondary_cp=m_flow_secondary_cp,
        flow_type=heat_exchanger.flow_type
    )
    return Q_achievable, A_required
