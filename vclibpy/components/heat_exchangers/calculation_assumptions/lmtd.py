import math

def calc_lmtd(flow_type, T1_in, T1_out, T2_in, T2_out):

    if flow_type == "counter":
        dT_in = T1_in - T2_out
        dT_out = T1_out - T2_in
    else:
        dT_in = T1_in - T2_in
        dT_out = T1_out - T2_out

    if dT_in * dT_out < 0:
        return 0.0000001
    lmtd = (dT_in - dT_out) / math.log((dT_in / dT_out))
    return abs(lmtd)

def calc_A(lmtd, k, Q):

    return max(Q / (k * lmtd), 0)

def calc_Q_(lmtd, k, A):

    return A * k * lmtd

def calc_k(ratio_outer_to_inner_area, k_wall: float,alpha_1: float, alpha_2: float) -> float:
    """
    Calculate the overall heat transfer coefficient (k) of the heat exchanger.

    Args:
        alpha_pri (float): Heat transfer coefficient for the primary medium.
        alpha_sec (float): Heat transfer coefficient for the secondary medium.

    Returns:
        float: Overall heat transfer coefficient (k).
    """
    k = (1 / (
                    (1 / alpha_1) * ratio_outer_to_inner_area +
                    (1 / k_wall) * ratio_outer_to_inner_area +
                    (1 / alpha_2)
            )
         )
    return k