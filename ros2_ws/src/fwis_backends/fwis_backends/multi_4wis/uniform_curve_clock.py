"""Replacement common progress clock for an already audited C2 path."""

import math

from .formation_continuous_a import bounds
from .formation_continuous_a.contracts import request_from_payload
from .formation_continuous_a.timing import _BoundFailure, _SCALED_METRICS, _WHEEL_COORDINATES, _vehicle_bounds
from .uniform_progress_clock import JerkClock, reachability_clock


def _station_time_window(clock, length, first, last):
    # Pad in station space before a monotone inversion so roundoff cannot
    # exclude time near a lobe or curvature-segment boundary.
    margin = 1e-10 * max(1., length)

    def bracket(station):
        if station <= 0.:
            return 0., 0.
        if station >= length:
            return clock.duration_s, clock.duration_s
        low, high = 0., clock.duration_s
        for _ in range(45):
            mid = (low + high) / 2
            if clock.sample(mid, length)[0] < station:
                low = mid
            else:
                high = mid
        return low, high

    return (bracket(max(0., first - margin))[0],
            bracket(min(length, last + margin))[1])


def _motion_intervals(clock, length, first, last):
    low, high = _station_time_window(clock, length, first, last)
    r, h, duration = clock.ramp_s, clock.hold_s, clock.duration_s
    ends = (r, r + h, 2 * r + h)
    times = (low, high, *(t for e in ends for t in (e, duration - e) if low <= t <= high))
    sampled = [clock.sample(t, length) for t in times]
    speed = math.nextafter(max(v for _, v, _, _ in sampled), math.inf)
    accel = math.nextafter(max(abs(a) for _, _, a, _ in sampled), math.inf)
    lobes = ((0., r), (r + h, 2 * r + h),
             (duration - 2 * r - h, duration - r - h), (duration - r, duration))
    jerk = math.nextafter(clock.suprema(length)[2], math.inf) if any(
        low <= end and start <= high for start, end in lobes) else 0.
    return (0., speed), (-accel, accel), (-jerk, jerk)


def _curve_clock_bounds(curve, clock):
    request = request_from_payload(curve.to_model_dict()["request_inputs"])
    limits = request.limits
    names = tuple(name for name, _, _ in _SCALED_METRICS)
    result = dict.fromkeys((*names, "wheel_angle"), 0.)
    length = curve.total_S_m
    layout = [(x * limits.wheelbase, y * limits.track) for x, y in _WHEEL_COORDINATES]
    for piece, (start, end) in zip(curve.path.profile._pieces, curve.path.profile.domain_segments):
        for i in range(8):
            q0, q1 = i / 8, (i + 1) / 8
            jets = bounds.segment_jet_bounds(piece.c0, piece.c1, piece.c2, piece.c3, piece.h, q0, q1)
            w, a, j = _motion_intervals(clock, length, start + (end - start) * q0,
                                        start + (end - start) * q1)
            for offset in request.model.offsets:
                for xw, yw in layout:
                    values = _vehicle_bounds(jets, offset, limits, w, a, j, xw, yw)
                    for name in names:
                        result[name] = max(result[name], values[name])
                    result["wheel_angle"] = max(result["wheel_angle"], values["steer_angle"])
    return result, limits


def certify_curve_clock(curve, clock):
    """Rebuild the whole-domain bound from geometry and serialized durations."""
    if curve.kind != "motion" or clock.duration_s >= curve.duration_s:
        raise ValueError("replacement clock must improve a moving C2 plan")
    try:
        result, limits = _curve_clock_bounds(curve, clock)
    except _BoundFailure as error:
        raise ValueError(f"C2 clock bounds could not be proven: {error.reason}") from error
    if result["wheel_angle"] > limits.steer_angle_max:
        raise ValueError("C2 clock exceeds the steering angle limit")
    for name, field, _ in _SCALED_METRICS:
        if not math.isfinite(result[name]) or result[name] > getattr(limits, field):
            raise ValueError(f"C2 clock exceeds the {name} limit")
    return result


def choose_curve_clock(curve, limits):
    """Propose a rest-to-rest clock with a cruise instead of a speed peak."""
    if curve.total_S_m < 8.:
        return None
    best = None
    for speed, acceleration, jerk in ((1., .6, .4), (.68, .055, .045)):
        try:
            clock = reachability_clock(curve.total_S_m, speed * limits.v_max,
                                       acceleration * limits.a_max, jerk * limits.jerk_max)
            result, _ = _curve_clock_bounds(curve, clock)
            # Scaling the clock preserves the station boundaries of every lobe.
            scale = max((result[name] / getattr(limits, field)) ** (1 / order)
                        for name, field, order in _SCALED_METRICS)
            scale = max(1., 1.02 * scale)
            candidate = JerkClock(clock.ramp_s * scale, clock.hold_s * scale, clock.cruise_s * scale)
        except (_BoundFailure, ValueError, ArithmeticError):
            continue
        if candidate.duration_s < (curve.duration_s if best is None else best.duration_s):
            best = candidate
    return best
