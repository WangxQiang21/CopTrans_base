"""One vehicle-level timing rule for every mode transition."""
import math


def mode_switch_event(delta, limits):
    """Describe one stationary mode-change event from wheel-angle delta.

    The vehicle configuration supplies a 0.3 s baseline. Under the interactive
    ``mode_switch_only`` profile, that is the complete event duration and
    steering demand remains diagnostic. Under the restricted profile, the
    event is lengthened only when setup plus rate/acceleration-limited steering
    cannot fit in the configured baseline.
    """
    delta = abs(float(delta))
    setup = float(limits.mode_setup_time)
    configured_base = limits.mode_switch_duration
    base = .0 if configured_base is None else float(configured_base)
    rate = float(limits.steer_rate_max)
    accel = float(limits.steer_accel_max)
    if not all(math.isfinite(x) for x in (delta, setup, base, rate, accel)):
        raise ValueError("nonfinite_mode_switch_timing")
    if (base < 0. or setup < 0. or rate <= 0. or accel <= 0. or
            (limits.translation_speed_policy == "mode_switch_only" and
             base <= 0.)):
        raise ValueError("invalid_mode_switch_timing")

    if limits.translation_speed_policy == "mode_switch_only":
        duration = base
        steering = duration
        setup = min(setup, duration)
        profile = "quintic_fixed_duration_kinematic"
        overlap = True
        steer_rate_peak = 1.875*delta/duration
        steer_accel_peak = (10/math.sqrt(3))*delta/duration**2
    else:
        if delta <= 1e-12:
            steering_min = 0.
        else:
            # Use the slower of the triangular/trapezoidal model used by the
            # continuous retimer and the quintic profile audited by v1.
            trapezoid = (2*math.sqrt(delta/accel)
                         if delta <= rate*rate/accel
                         else delta/rate+rate/accel)
            quintic = max(1.875*delta/rate,
                          math.sqrt((10/math.sqrt(3))*delta/accel))
            steering_min = max(trapezoid, quintic)
        duration = max(base, setup+steering_min)
        steering = duration-setup
        profile = "shared_rate_accel_bound"
        overlap = False
        steer_rate_peak = 1.875*delta/max(steering, 1e-12)
        steer_accel_peak = ((10/math.sqrt(3))*delta/
                            max(steering**2, 1e-12))

    return {"duration": duration, "setup_s": setup,
            "steering_s": steering, "steering_profile": profile,
            "setup_overlaps_steering": overlap,
            "steer_rate_peak": steer_rate_peak,
            "steer_accel_peak": steer_accel_peak,
            "max_wheel_angle_delta": delta}


def mode_switch_events_for_actions(actions, limits, initial_mode=0):
    """Return vehicle-timed events for an action sequence, starting in A mode."""
    from .core import Action
    from .retiming import wheel_configuration

    previous_mode = int(initial_mode)
    previous_angles, _ = wheel_configuration(
        Action(previous_mode, 0., 0.), limits)
    events = []
    for action in actions:
        if abs(action.amount) <= 1e-10:
            continue
        angles, _ = wheel_configuration(action, limits)
        if action.mode != previous_mode:
            event = mode_switch_event(
                max(abs(float(a)-float(b))
                    for a, b in zip(previous_angles, angles)), limits)
            event.update(from_mode=previous_mode, to_mode=action.mode)
            events.append(event)
        previous_mode = action.mode
        previous_angles = angles
    return events
