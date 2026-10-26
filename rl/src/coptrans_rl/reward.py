"""Time-shaped reward with an exact successful-episode terminal correction."""
import math


def proxy_seconds(action, previous_mode, limits):
    rate = limits.omega_max if action.mode == 2 else limits.target_v
    hold = float(limits.mode_switch_duration or 0.) if action.mode != previous_mode else 0.
    return abs(action.amount)/rate+hold


def shaped_step(proxy_s, old_potential, new_potential, time_scale=10.,
                potential_weight=1.):
    if not all(math.isfinite(v) for v in (
            proxy_s, old_potential, new_potential, time_scale, potential_weight)):
        raise ValueError("nonfinite_reward_input")
    if time_scale <= 0 or proxy_s < 0:
        raise ValueError("invalid_reward_scale_or_time")
    return -proxy_s/time_scale+potential_weight*(new_potential-old_potential)


def success_correction(proxy_total, actual_seconds, time_scale=10., bonus=50.):
    if not all(math.isfinite(v) for v in (proxy_total, actual_seconds, time_scale, bonus)):
        raise ValueError("nonfinite_reward_input")
    if actual_seconds < 0 or proxy_total < 0 or time_scale <= 0:
        raise ValueError("invalid_execution_time")
    return bonus+(proxy_total-actual_seconds)/time_scale
