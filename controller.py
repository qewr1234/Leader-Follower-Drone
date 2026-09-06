"""
controller.py — 거리 기반 모터 제어

주의: MAV_CMD_DO_MOTOR_TEST 기반 구조는 프로토타입/지상 테스트용이다.
실제 논문/비행용은 velocity setpoint 제어로 바꾸는 것을 권장.
"""

S_IDLE = "IDLE"
S_FAR = "APPROACH"
S_TARGET = "FOLLOW"
S_CLOSE = "BACK_OFF"
S_HOLD = "HOLD"
S_STOP = "STOP"

FRONT_MOTORS = [1, 3]
REAR_MOTORS = [2, 4]
RIGHT_MOTORS = [1, 4]
LEFT_MOTORS = [2, 3]
ALL_MOTORS = [1, 2, 3, 4]

ZONE_CLOSE_M = 2.5
ZONE_TARGET_M = 4.5
ZONE_CLOSE_RATIO = 0.15
ZONE_TARGET_RATIO = 0.05

MAX_THROTTLE = 50.0
MIN_THROTTLE = 5.0
THROTTLE_FAR = 42.0
THROTTLE_TARGET = 32.0
THROTTLE_CLOSE = 26.0

FORE_BIAS = 8.0
BACK_BIAS = 8.0
FORWARD_IDLE_BIAS = 2.0
CENTER_DEAD_ZONE = 0.12
MAX_LR_BIAS = 8.0


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def classify_zone(depth_m=None, h_ratio=None):
    if depth_m is not None:
        if depth_m <= ZONE_CLOSE_M:
            return S_CLOSE
        elif depth_m <= ZONE_TARGET_M:
            return S_TARGET
        else:
            return S_FAR
    if h_ratio is not None:
        if h_ratio >= ZONE_CLOSE_RATIO:
            return S_CLOSE
        elif h_ratio >= ZONE_TARGET_RATIO:
            return S_TARGET
        else:
            return S_FAR
    return S_STOP


def base_throttle_for_zone(zone):
    return {S_FAR: THROTTLE_FAR, S_TARGET: THROTTLE_TARGET, S_CLOSE: THROTTLE_CLOSE}.get(zone, 0.0)


def compute_throttles(zone, ex, uncertainty_slowdown=1.0):
    base = base_throttle_for_zone(zone) * uncertainty_slowdown
    t = {m: base for m in ALL_MOTORS}

    if zone == S_FAR:
        for m in REAR_MOTORS:
            t[m] += FORE_BIAS * uncertainty_slowdown
        for m in FRONT_MOTORS:
            t[m] -= FORE_BIAS * uncertainty_slowdown
    elif zone == S_CLOSE:
        for m in FRONT_MOTORS:
            t[m] += BACK_BIAS * uncertainty_slowdown
        for m in REAR_MOTORS:
            t[m] -= BACK_BIAS * uncertainty_slowdown
    else:
        for m in REAR_MOTORS:
            t[m] += FORWARD_IDLE_BIAS * uncertainty_slowdown

    lr_bias = clamp(abs(ex) * MAX_LR_BIAS, 0.0, MAX_LR_BIAS) * uncertainty_slowdown
    if ex < -CENTER_DEAD_ZONE:
        for m in RIGHT_MOTORS:
            t[m] += lr_bias
        for m in LEFT_MOTORS:
            t[m] -= lr_bias
    elif ex > CENTER_DEAD_ZONE:
        for m in LEFT_MOTORS:
            t[m] += lr_bias
        for m in RIGHT_MOTORS:
            t[m] -= lr_bias

    raw_throttles = {m: round(t[m], 1) for m in ALL_MOTORS}
    for m in ALL_MOTORS:
        t[m] = clamp(t[m], MIN_THROTTLE, MAX_THROTTLE) if zone != S_STOP else 0.0
    return t, raw_throttles
