from vclibpy.media import ThermodynamicState


def separate_phases(self, state_max_cold: ThermodynamicState, state_min_cold: ThermodynamicState,
                    p_1: float, state_max_warm: ThermodynamicState, state_min_warm: ThermodynamicState, p_2: float):
    """

    Args:
        self:
        state_max_cold:
        state_min_cold:
        p_1:
        state_max_warm:
        state_min_warm:
        p_2:

    Returns:

    """
    # Get relevant states:
    state_q0_1 = self.med_prop.calc_state("PQ", p_1, 0)
    state_q1_1 = self.med_prop.calc_state("PQ", p_1, 1)

    state_q0_2 = self.med_prop.calc_state("PQ", p_1, 0)
    state_q1_2 = self.med_prop.calc_state("PQ", p_1, 1)

    is_sc_1 = True
    is_lat_1 = True
    is_sh_1 = True

    is_sc_2 = True
    is_lat_2 = True
    is_sh_2 = True

    if state_q0_1.h < state_min_cold.h < state_q1_1.h:
        is_sc_1 = False
    elif state_q1_1.h < state_min_cold.h:
        is_sc_1 = False
        is_lat_1 = False
    if state_q0_1.h < state_max_cold.h < state_q1_1.h:
        is_sh_1 = False
    elif state_max_cold.h < state_q0_1.h:
        is_sh_1 = False
        is_lat_1 = False

    if state_q0_2.h < state_min_warm.h < state_q1_2.h:
        is_sc_2 = False
    elif state_q1_1.h < state_min_warm.h:
        is_sc_2 = False
        is_lat_2 = False
    if state_q0_2.h < state_max_warm.h < state_q1_2.h:
        is_sh_2 = False
    elif state_max_warm.h < state_q0_2.h:
        is_sh_2 = False
        is_lat_2 = False

    Q1_regime = _seperate_single(is_sc_1, is_lat_1, is_sh_1,
                                 state_min_cold, state_max_cold, state_q0_1, state_q1_1, self.m_flow_1)
    Q2_regime = _seperate_single(is_sc_2, is_lat_2, is_sh_2,
                                 state_min_warm, state_max_warm, state_q0_2, state_q1_2, self.m_flow_2)
    Q_sc_sc = 0
    Q_sc_lat = 0
    Q_sc_sh = 0
    Q_lat_sc = 0
    Q_lat_lat = 0
    Q_lat_sh = 0
    Q_sh_sc = 0
    Q_sh_lat = 0
    Q_sh_sh = 0
    if self.flow_type == "counter":

        if is_sc_1 and not is_lat_1 and not is_sh_1:
            if is_sc_2 and not is_lat_2 and not is_sh_2:
                # print("sc/sc")
                Q_sc_sc = Q2_regime[0]
            elif not is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sc/lat")
                Q_sc_lat = Q2_regime[1]
            elif not is_sc_2 and not is_lat_2 and is_sh_2:
                # print("sc/sh")
                Q_sc_sh = Q2_regime[2]
            elif is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sc/sc+lat")
                Q_sc_sc = Q2_regime[0]
                Q_sc_lat = Q2_regime[1]
            elif not is_sc_2 and is_lat_2 and is_sh_2:
                # print("sc/lat+sh")
                Q_sc_lat = Q2_regime[1]
                Q_sc_sh = Q2_regime[2]
            elif is_sc_2 and is_lat_2 and is_sh_2:
                # print("sc/sc+lat+sh")
                Q_sc_sc = Q2_regime[0]
                Q_sc_lat = Q2_regime[1]
                Q_sc_sh = Q2_regime[2]

        elif not is_sc_1 and is_lat_1 and not is_sh_1:
            if is_sc_2 and not is_lat_2 and not is_sh_2:
                # print("lat/sc")
                Q_lat_sc = Q2_regime[0]
            elif not is_sc_2 and is_lat_2 and not is_sh_2:
                # print("lat/lat")
                Q_lat_lat = Q2_regime[1]
            elif not is_sc_2 and not is_lat_2 and is_sh_2:
                # print("lat/sh")
                Q_lat_sh = Q2_regime[2]
            elif is_sc_2 and is_lat_2 and not is_sh_2:
                # print("lat/sc+lat")
                Q_lat_sc = Q2_regime[0]
                Q_lat_lat = Q2_regime[1]
            elif not is_sc_2 and is_lat_2 and is_sh_2:
                # print("lat/lat+sh")
                Q_lat_lat = Q2_regime[1]
                Q_lat_sh = Q2_regime[2]
            elif is_sc_2 and is_lat_2 and is_sh_2:
                # print("lat/sc+lat+sh")
                Q_lat_sc = Q2_regime[0]
                Q_lat_lat = Q2_regime[1]
                Q_lat_sh = Q2_regime[2]

        elif not is_sc_1 and not is_lat_1 and is_sh_1:
            if is_sc_2 and not is_lat_2 and not is_sh_2:
                # print("sh/sc")
                Q_sh_sc = Q2_regime[0]
            elif not is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sh/lat")
                Q_sh_lat = Q2_regime[1]
            elif not is_sc_2 and not is_lat_2 and is_sh_2:
                # print("sh/sh")
                Q_sh_sh = Q2_regime[2]
            elif is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sh/sc+lat")
                Q_sh_sc = Q2_regime[0]
                Q_sh_lat = Q2_regime[1]
            elif not is_sc_2 and is_lat_2 and is_sh_2:
                # print("sh/lat+sh")
                Q_sh_lat = Q2_regime[1]
                Q_sh_sh = Q2_regime[2]
            elif is_sc_2 and is_lat_2 and is_sh_2:
                # print("sh/sc+lat+sh")
                Q_sh_sc = Q2_regime[0]
                Q_sh_lat = Q2_regime[1]
                Q_sh_sh = Q2_regime[2]

        elif is_sc_1 and is_lat_1 and not is_sh_1:
            if is_sc_2 and not is_lat_2 and not is_sh_2:
                # print("sc+lat/sc")
                Q_sc_sc = Q1_regime[0]
                Q_lat_sc = Q1_regime[1]
            elif not is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sc+lat/lat")
                Q_sc_lat = Q1_regime[0]
                Q_lat_lat = Q1_regime[1]
            elif not is_sc_2 and not is_lat_2 and is_sh_2:
                # print("sc+lat/sh")
                Q_sc_sh = Q1_regime[0]
                Q_lat_sh = Q1_regime[1]
            elif is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sc+lat/sc+lat")
                Q_sc_sc = min(Q1_regime[0], Q2_regime[0])
                Q_lat_lat = min(Q1_regime[1], Q2_regime[1])
                Q_sc_lat = max(0, Q1_regime[0] - Q2_regime[0])
                Q_lat_sc = max(0, Q2_regime[0] - Q1_regime[0])
            elif not is_sc_2 and is_lat_2 and is_sh_2:
                # print("sc+lat/lat+sh")
                Q_sc_lat = min(Q1_regime[0], Q2_regime[1])
                Q_lat_sh = min(Q1_regime[1], Q2_regime[2])
                Q_lat_lat = max(0, Q1_regime[1] - Q1_regime[0] - Q2_regime[2])
                Q_sc_sh = max(0, Q1_regime[0] - Q2_regime[1])
            elif is_sc_2 and is_lat_2 and is_sh_2:
                # print("sc+lat/sc+lat+sh")
                Q_sc_sc = min(Q1_regime[0], Q2_regime[0])
                Q_sc_lat = max(0, Q1_regime[0] - Q2_regime[0])
                Q_sc_sh = max(0, Q2_regime[2] - Q1_regime[1])
                Q_lat_sc = max(0, Q2_regime[0] - Q1_regime[0])
                Q_lat_lat = max(0, Q1_regime[1] - Q2_regime[2] - max(Q1_regime[0], Q2_regime[0]))
                Q_lat_sh = min(Q1_regime[1], Q2_regime[2])

        elif not is_sc_1 and is_lat_1 and is_sh_1:
            if is_sc_2 and not is_lat_2 and not is_sh_2:
                # print("lat+sh/sc")
                Q_lat_sc = Q1_regime[1]
                Q_sh_sc = Q1_regime[2]
            elif not is_sc_2 and is_lat_2 and not is_sh_2:
                # print("lat+sh/lat")
                Q_lat_lat = Q1_regime[1]
                Q_sh_lat = Q1_regime[2]
            elif not is_sc_2 and not is_lat_2 and is_sh_2:
                # print("lat+sh/sh")
                Q_lat_sh = Q1_regime[1]
                Q_sh_sh = Q1_regime[2]
            elif is_sc_2 and is_lat_2 and not is_sh_2:
                # print("lat+sh/sc+lat")
                Q_lat_sc = min(Q1_regime[1], Q2_regime[0])
                Q_lat_lat = max(0, Q2_regime[1] - Q1_regime[2] - Q2_regime[0])
                Q_sh_sc = max(0, Q1_regime[2] - Q1_regime[1] - Q2_regime[1])
                Q_sh_lat = min(Q1_regime[2], Q2_regime[1])
            elif not is_sc_2 and is_lat_2 and is_sh_2:
                # print("lat+sh/lat+sh")
                Q_lat_lat = min(Q1_regime[1], Q2_regime[1])
                Q_sh_sh = min(Q1_regime[2], Q2_regime[2])
                Q_lat_sh = max(0, Q1_regime[1] - Q2_regime[1])
                Q_sh_lat = max(0, Q2_regime[1] - Q1_regime[1])
            elif is_sc_2 and is_lat_2 and is_sh_2:
                # print("lat+sh/sc+lat+sh")
                Q_lat_sc = min(Q1_regime[1], Q2_regime[0])
                Q_sh_sh = min(Q1_regime[2], Q1_regime[2])
                Q_lat_lat = max(0, Q1_regime[1] - Q2_regime[0] - max(Q1_regime[2], Q2_regime[2]))
                Q_lat_sh = max(0, Q1_regime[1] - Q2_regime[0] - Q2_regime[1])
                Q_sh_lat = max(0, Q1_regime[2] - Q2_regime[2])
                Q_sh_sc = max(0, Q1_regime[2] - Q2_regime[2] - Q2_regime[1])

        elif is_sc_1 and is_lat_1 and is_sh_1:
            if is_sc_2 and not is_lat_2 and not is_sh_2:
                Q_sc_sc = Q1_regime[0]
                Q_lat_sc = Q1_regime[1]
                Q_sh_sc = Q1_regime[2]
            # print("sc+lat+sh/sc")
            elif not is_sc_2 and is_lat_2 and not is_sh_2:
                Q_sc_lat = Q1_regime[0]
                Q_lat_lat = Q1_regime[1]
                Q_sh_lat = Q1_regime[2]
                # print("sc+lat+sh/lat")
            elif not is_sc_2 and not is_lat_2 and is_sh_2:
                # print("sc+lat+sh/sh")
                Q_sc_sh = Q1_regime[0]
                Q_lat_sh = Q1_regime[1]
                Q_sh_sh = Q1_regime[2]
            elif is_sc_2 and is_lat_2 and not is_sh_2:
                # print("sc+lat+sh/sc+lat")
                Q_sc_sc = min(Q1_regime[0], Q2_regime[0])
                Q_sc_lat = max(0, Q1_regime[0] - Q2_regime[0])
                Q_lat_sc = max(0, Q2_regime[0] - Q1_regime[0])
                Q_lat_lat = max(0, Q2_regime[1] - Q1_regime[2] - max(Q1_regime[0], Q2_regime[0]))
                Q_sh_sc = max(0, Q1_regime[2] - Q2_regime[1])
                Q_sh_lat = min(Q1_regime[2], Q2_regime[1])
            elif not is_sc_2 and is_lat_2 and is_sh_2:
                # print("sc+lat+sh/lat+sh")
                Q_sc_lat = min(Q1_regime[0], Q2_regime[1])
                Q_sc_sh = max(0, Q1_regime[0] - Q2_regime[1])
                Q_lat_lat = max(0, Q2_regime[1] - Q1_regime[0] - max(Q1_regime[2], Q2_regime[2]))
                Q_lat_sh = max(0, Q2_regime[2] - Q1_regime[2])
                Q_sh_lat = max(0, Q1_regime[2] - Q2_regime[2])
                Q_sh_sh = min(Q1_regime[2], Q2_regime[2])
            elif is_sc_2 and is_lat_2 and is_sh_2:
                print("sc+lat+sh/sc+lat+sh")
                Q_sc_sc = min(Q1_regime[0], Q2_regime[0])
                Q_sc_lat = max(0, Q1_regime[0] - Q2_regime[0])
                Q_sc_sh = max(0, Q2_regime[2] - Q1_regime[2] - Q1_regime[1])
                Q_lat_sc = max(0, Q2_regime[0] - Q1_regime[0])
                Q_lat_lat = ((Q1_regime[0] + Q1_regime[1] + Q1_regime[2]) - max(Q1_regime[0], Q2_regime[0]) -
                             max(Q1_regime[2], Q2_regime[2]))
                Q_lat_sh = max(0, Q2_regime[2] - Q1_regime[2])
                Q_sh_sc = max(0, Q1_regime[2] - Q2_regime[2] - Q2_regime[1])
                Q_sh_lat = max(0, Q1_regime[2] - Q2_regime[2])
                Q_sh_sh = min(Q1_regime[2], Q2_regime[2])
    return Q_sc_sc, Q_sc_lat, Q_sc_sh, Q_lat_sc, Q_lat_lat, Q_lat_sh, Q_sh_sc, Q_sh_lat, Q_sh_sh


def _seperate_single(
        is_sc, is_lat, is_sh,
        state_min, state_max, state_q0, state_q1, m_flow
):
    q = state_max.h - state_min.h
    Q_regime = [0, 0, 0]

    if is_sc:
        Q_regime[0] = max(0.0,
                          min(q,
                              (state_q0.h - state_min.h))) * m_flow
    if is_lat:
        Q_regime[1] = max(0.0,
                          min(q,
                              (state_q1.h - state_q0.h),
                              (state_q1.h - state_min.h),
                              (state_max.h - state_q0.h))) * m_flow
    if is_sh:
        Q_regime[2] = max(0.0,
                          min(q,
                              (state_max.h - state_q1.h))) * m_flow

    return Q_regime
