"""Continuous speed and steering timing for continuous_motion_v2 blocks.

Translation blocks are time-scaled as one trajectory. Internal geometry and
control knots do not cause zero speed. A mode boundary creates one stationary
setup phase; a same-mode gear cusp reaches zero speed without a dwell.
"""
from dataclasses import asdict, dataclass
from collections import OrderedDict
from copy import deepcopy
import math

import numpy as np

from .continuous_geometry import ContinuousPlan, MotionBlock, sample_block
from .core import Action, Pose
from .retiming import (MotionLimits, bounded_duration, quintic, wheel_configuration,
                       steering_rest_to_rest_duration)


@dataclass
class ContinuousTimedPlan:
    samples: list
    phases: list
    duration: float
    moving_seconds: float
    spin_seconds: float
    mode_switch_hold_seconds: float
    mode_switch_count: int
    same_mode_hold_seconds: float
    gear_change_hold_seconds: float
    limits: MotionLimits
    speed_constraints: list = None
    low_speed_intervals: list = None
    interpolation_version: str = "continuous_motion_v2_piecewise_quintic_time_v3"

    def report(self):
        return {
            "interpolation_version": self.interpolation_version,
            "profile": asdict(self.limits),
            "translation_speed_policy": self.limits.translation_speed_policy,
            "duration_s": self.duration,
            "translation_s": self.moving_seconds,
            "spin_s": self.spin_seconds,
            "mode_switch_hold_seconds": self.mode_switch_hold_seconds,
            "mode_switch_count": self.mode_switch_count,
            "same_mode_hold_seconds": self.same_mode_hold_seconds,
            "gear_change_hold_seconds": self.gear_change_hold_seconds,
            "phases": self.phases,
            "speed_constraint_intervals": self.speed_constraints or [],
            "low_speed_intervals": self.low_speed_intervals or [],
            "low_speed_threshold_mps": .2,
            "low_speed_min_duration_s": .5,
        }


def _s3(tau):
    return 60. - 360.*tau + 360.*tau*tau


def _angles_and_factors(block, s, limits):
    if block.mode == 2:
        return wheel_configuration(Action(2, block.signed_angle, 0.), limits)
    control = block.control(s)[0]
    return wheel_configuration(Action(block.mode, block.direction, control), limits)


def _translation_time_limits(limits):
    """Limits that change translation timing under the selected policy.

    Wheel/steering/body-yaw demand is still measured in both policies. The
    interactive kinematic policy does not turn that demand into a speed cap.
    """
    bounds = {"speed": (limits.target_v, 1),
              "acceleration": (limits.a_max, 2),
              "jerk": (limits.jerk_max, 3)}
    if limits.translation_speed_policy == "restricted":
        bounds.update({
            "wheel_speed": (limits.wheel_speed_max, 1),
            "lateral_acceleration": (limits.lateral_a_max, 2),
            "angular_speed": (limits.translation_omega_max, 1),
            "angular_acceleration": (limits.translation_alpha_max, 2),
            "angular_jerk": (limits.translation_angular_jerk_max, 3),
            "steer_rate": (limits.steer_rate_max, 1),
            "steer_acceleration": (limits.steer_accel_max, 2),
            "oblique_normal_acceleration": (limits.lateral_a_max, 2)})
    return bounds


def _duration_and_peaks(block, limits):
    """Return a conservative time stretch from dense continuous derivatives."""
    if block.mode == 2:
        amount = abs(block.signed_angle)
        if amount <= 1e-12:
            return 0., {"angular_speed": 0., "angular_acceleration": 0.,
                        "angular_jerk": 0., "wheel_speed": 0.}
        angles, factors = _angles_and_factors(block, 0., limits)
        rate = min(limits.omega_max, limits.wheel_speed_max/max(factors))
        duration = bounded_duration(amount, rate, limits.alpha_max,
                                    limits.angular_jerk_max)
        return duration, {"angular_speed": 1.875*amount/duration,
                          "angular_acceleration": (10/math.sqrt(3))*amount/duration**2,
                          "angular_jerk": 60*amount/duration**3,
                          "wheel_speed": max(factors)*1.875*amount/duration}

    grid_n = max(201, int(math.ceil(block.length/.01))+1)
    ss = np.linspace(0., block.length, grid_n)
    controls = [block.control(float(s)) for s in ss]
    c = np.asarray([row[0] for row in controls])
    c1 = np.asarray([row[1] for row in controls])
    c2 = np.asarray([row[2] for row in controls])
    wheel = np.asarray([_angles_and_factors(block, float(s), limits)[0] for s in ss])
    factors = np.asarray([_angles_and_factors(block, float(s), limits)[1] for s in ss])
    _, factors, steer1, steer2 = _wheel_geometry_arrays(block, c, c1, c2, limits)

    duration = 1.
    for _ in range(12):
        peaks = {k: 0. for k in ("speed", "acceleration", "jerk", "wheel_speed",
                                 "lateral_acceleration", "angular_speed",
                                 "angular_acceleration", "angular_jerk",
                                 "steer_rate", "steer_acceleration",
                                 "oblique_normal_acceleration")}
        for tau in np.linspace(0., 1., 401):
            s_norm, ds, dds = quintic(float(tau))
            s = block.length*s_norm
            idx = min(len(ss)-2, max(0, int(np.searchsorted(ss, s)-1)))
            u = min(1., max(0., (s-ss[idx])/(ss[idx+1]-ss[idx])))
            k = c[idx]*(1-u)+c[idx+1]*u
            kp = c1[idx]*(1-u)+c1[idx+1]*u
            kpp = c2[idx]*(1-u)+c2[idx+1]*u
            d1 = steer1[idx]*(1-u)+steer1[idx+1]*u
            d2 = steer2[idx]*(1-u)+steer2[idx+1]*u
            fac = factors[idx]*(1-u)+factors[idx+1]*u
            v = block.length*ds/duration
            a = block.length*dds/duration**2
            j = block.length*_s3(float(tau))/duration**3
            if block.mode == 0:
                omega = abs(k*v)
                alpha = abs(k*a+kp*v*v)
                angular_jerk = abs(k*j+3*kp*v*a+kpp*v**3)
                lateral = v*v*abs(k)
                oblique_normal = 0.
            else:
                omega = 0.
                # O holds body yaw. Its changing velocity direction creates
                # path-normal acceleration; phi derivatives constrain steering
                # rate and acceleration below, not body angular acceleration.
                alpha = 0.
                angular_jerk = 0.
                lateral = 0.
                oblique_normal = v*v*abs(kp)
            values = {
                "speed": abs(v), "acceleration": abs(a), "jerk": abs(j),
                "wheel_speed": max(fac)*abs(v),
                "lateral_acceleration": lateral,
                "angular_speed": omega, "angular_acceleration": alpha,
                "angular_jerk": angular_jerk,
                "steer_rate": float(np.max(np.abs(d1*v))),
                "steer_acceleration": float(np.max(np.abs(d2*v*v+d1*a))),
                "oblique_normal_acceleration": oblique_normal,
            }
            for name, value in values.items():
                peaks[name] = max(peaks[name], value)
        limits_by_name = _translation_time_limits(limits)
        required = duration
        for name, (bound, power) in limits_by_name.items():
            if bound is None:
                continue
            ratio = peaks[name]/bound
            if ratio > 1.:
                required = max(required, duration*ratio**(1./power))
        if required <= duration*(1+1e-8):
            return duration, peaks
        duration = required*1.0005
    raise ValueError("continuous_time_scaling_did_not_converge")


def _quintic_coefficients(distance, start_speed, end_speed, duration,
                          start_acceleration=0., end_acceleration=0.):
    """Quintic position with shared endpoint speeds AND accelerations."""
    c1 = start_speed*duration
    c2 = .5*start_acceleration*duration**2
    tail = np.linalg.solve(
        np.asarray([[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]]),
        np.asarray([distance-c1-c2, end_speed*duration-c1-2*c2,
                    end_acceleration*duration**2-2*c2]))
    return np.asarray([0., c1, c2, tail[0], tail[1], tail[2]])


def _quintic_state(coefficients, tau, duration):
    c = np.asarray(coefficients, dtype=float)
    q = float(np.polynomial.polynomial.polyval(tau, c))
    v = float(np.polynomial.polynomial.polyval(
        tau, np.arange(1, 6)*c[1:])/duration)
    a = float(np.polynomial.polynomial.polyval(
        tau, np.arange(2, 6)*np.arange(1, 5)*c[2:])/duration**2)
    j = float(np.polynomial.polynomial.polyval(
        tau, np.arange(3, 6)*np.arange(2, 5)*np.arange(1, 4)*c[3:])/
        duration**3)
    return q, v, a, j


def _wheel_geometry_arrays(block, c, c1, c2, limits):
    """Exact chain rule for wheel angle derivatives; no finite differences."""
    c, c1, c2 = (np.asarray(v, dtype=float) for v in (c, c1, c2))
    if block.mode == 1:
        return (np.repeat(c[:, None], 4, axis=1), np.ones((len(c), 4)),
                np.repeat(c1[:, None], 4, axis=1),
                np.repeat(c2[:, None], 4, axis=1))
    x = np.asarray([limits.wheelbase/2, limits.wheelbase/2,
                    -limits.wheelbase/2, -limits.wheelbase/2])
    y = np.asarray([limits.track/2, -limits.track/2,
                    limits.track/2, -limits.track/2])
    vx, vy = 1.-c[:, None]*y, c[:, None]*x
    den = vx*vx+vy*vy
    dc = x/den
    ddc = 2.*x*(y-c[:, None]*(x*x+y*y))/(den*den)
    return (np.arctan2(vy, vx), np.sqrt(den), dc*c1[:, None],
            dc*c2[:, None]+ddc*c1[:, None]**2)


def _continuous_geometry_table(block, limits):
    count = max(201, int(math.ceil(block.length/.01))+1)
    # Resolve every short Hermite span as well as the regular metric grid.
    spans = [np.linspace(a.s, b.s, 9) for a, b in
             zip(block.knots, block.knots[1:])]
    ss = np.unique(np.concatenate([np.linspace(0., block.length, count)]+spans))
    c, c1, c2 = block.control_many(ss)
    _, factors, steer1, steer2 = _wheel_geometry_arrays(block, c, c1, c2, limits)
    return ss, c, c1, c2, factors, steer1, steer2


def _interp_rows(ss, *rows, s):
    index = min(len(ss)-2, max(0, int(np.searchsorted(ss, s)-1)))
    ratio = min(1., max(0., (s-ss[index])/(ss[index+1]-ss[index])))
    return tuple(row[index]*(1.-ratio)+row[index+1]*ratio for row in rows)


def _piece_peaks(block, phase, limits, table=None):
    """Sample the exact control spline and time polynomial in one batch."""
    d = phase["s1"]-phase["s0"]
    duration = phase["duration"]
    coeff = _quintic_coefficients(d, phase["start_speed"],
                                  phase["end_speed"], duration,
                                  phase.get("start_acceleration", 0.),
                                  phase.get("end_acceleration", 0.))
    if not np.allclose(coeff, phase["coefficients"], rtol=0., atol=1e-9):
        raise ValueError("continuous_time_profile_coefficients_mismatch")
    queries = np.concatenate([np.linspace(a.s, b.s, 9) for a, b in
                              zip(block.knots, block.knots[1:])])
    knots = np.asarray([k.s for k in block.knots])
    queries = np.unique(np.r_[queries, np.nextafter(knots, -np.inf),
                             np.nextafter(knots, np.inf)])
    queries = queries[(queries > phase["s0"])&(queries < phase["s1"])]-phase["s0"]
    lo, hi = np.zeros(len(queries)), np.ones(len(queries))
    for _ in range(36):
        mid = .5*(lo+hi)
        below = np.polynomial.polynomial.polyval(mid, coeff) < queries
        lo, hi = np.where(below, mid, lo), np.where(below, hi, mid)
    tau = np.r_[np.linspace(0., 1., 81), .5*(lo+hi)]
    q = np.polynomial.polynomial.polyval(tau, coeff)
    v = np.polynomial.polynomial.polyval(tau, np.arange(1, 6)*coeff[1:])/duration
    a = np.polynomial.polynomial.polyval(tau, np.arange(2, 6)*np.arange(1, 5)*coeff[2:])/duration**2
    j = np.polynomial.polynomial.polyval(tau, np.arange(3, 6)*np.arange(2, 5)*np.arange(1, 4)*coeff[3:])/duration**3
    if np.min(q) < -1e-8 or np.max(q) > d+1e-8 or np.min(v) < -1e-8:
        raise ValueError("continuous_nonmonotone_time_profile")
    local_s = phase["s0"]+np.clip(q, 0., d)
    # The Hermite curve is C1. At the right boundary its second derivative
    # belongs to the preceding span for this phase, and to the next span for
    # the following phase. Do not apply a tiny next turn to a straight piece.
    local_s = np.minimum(local_s, np.nextafter(phase["s1"], phase["s0"]))
    local_s[81:] = queries+phase["s0"]
    c, c1, c2 = block.control_many(local_s)
    wheel, factors, steer1, steer2 = _wheel_geometry_arrays(block, c, c1, c2, limits)
    zero = np.zeros_like(v)
    values = {
        "speed": v, "acceleration": a, "jerk": j,
        "wheel_speed": factors*v[:, None],
        "lateral_acceleration": c*v*v if block.mode == 0 else zero,
        "angular_speed": c*v if block.mode == 0 else zero,
        "angular_acceleration": c*a+c1*v*v if block.mode == 0 else zero,
        "angular_jerk": c*j+3.*c1*v*a+c2*v**3 if block.mode == 0 else zero,
        "steer_rate": steer1*v[:, None],
        "steer_acceleration": steer2*v[:, None]**2+steer1*a[:, None],
        "oblique_normal_acceleration": c1*v*v if block.mode == 1 else zero,
        "wheel_angle": wheel}
    return {name: float(np.max(np.abs(value))) for name, value in values.items()}


def _translation_phases(block, limits, block_index, start_time,
                        global_s_offset=0., time_step=.20):
    """Connect physical speed bottlenecks with jerk-limited S curves.

    Acceleration flows across geometry knots. Only speed extrema and reachable
    acceleration/cruise/braking boundaries define temporal phases; polynomial
    records retain continuous velocity and acceleration for independent audit.
    """
    if block.length <= 1e-10:
        return [], 0., [], []
    count = max(2, int(math.ceil(block.length/time_step))+1)
    if count == 2 and len(block.knots) == 2:
        duration, peaks = _duration_and_peaks(block, limits)
        if duration <= 1e-9:
            return [], 0., peaks, []
        phase = {"kind": "translation", "profile": "rest_to_rest",
                 "block_index": block_index, "segment_index": 0,
                 "mode": block.mode, "direction": block.direction,
                 "t0": start_time, "duration": duration,
                 "s0": 0., "s1": block.length,
                 "dynamic_peaks": peaks}
        return [phase], duration, peaks, [{
            "block_index": block_index, "mode": block.mode,
            "direction": block.direction,
            "global_s_interval_m": [global_s_offset,
                                     global_s_offset+block.length],
            "local_s_interval_m": [0., block.length],
            "cap_source": "rest_to_rest_dense_dynamic_peaks",
            "dynamic_peaks": peaks,
            "active_dynamic_limits": _active_peak_limits(peaks, limits)}]
    knots = np.asarray([k.s for k in block.knots])
    regular = np.linspace(0., block.length, count)
    # Prefer actual control boundaries. Nearly adjacent regular stations would
    # introduce artificial short time polynomials with zero endpoint accel.
    regular = regular[np.min(np.abs(regular[:, None]-knots), axis=1) >= .5*time_step]
    midpoints = [(a.s+b.s)/2. for a, b in zip(block.knots, block.knots[1:])
                 if abs(a.value-b.value) > 1e-8]
    stations = np.unique(np.r_[regular, knots, midpoints])
    stations = stations[np.r_[True, np.diff(stations) > 1e-9]]
    stations[-1] = block.length
    ds = np.diff(stations)
    table = _continuous_geometry_table(block, limits)
    ss, c, c1, c2, factors, steer1, steer2 = table
    interval_speed, interval_accel, interval_limits = [], [], []
    for left, right in zip(stations[:-1], stations[1:]):
        # Resolve spline interiors as well as the envelope station endpoints;
        # control knots are present in `stations` and each interval is sampled
        # densely enough to expose internal wheel-angle derivative peaks.
        query = np.linspace(left, right, 65)
        query[-1] = np.nextafter(right, left)
        values, first, second = block.control_many(query)
        _, wheel_ratios, d1, d2 = _wheel_geometry_arrays(
            block, values, first, second, limits)
        caps = {"target_speed": float(limits.target_v),
                "wheel_speed": float(limits.wheel_speed_max/
                                      max(float(np.max(wheel_ratios)), 1e-9)),
                "steer_rate": float(limits.steer_rate_max/
                                     max(float(np.max(np.abs(d1))), 1e-9)),
                "steer_acceleration_speed": float(math.sqrt(
                    limits.steer_accel_max/
                    max(float(np.max(np.abs(d2))), 1e-9)))}
        acceleration_caps = {"path_acceleration": float(limits.a_max),
                             "steer_acceleration": float(
                                 limits.steer_accel_max/
                                 max(float(np.max(np.abs(d1))), 1e-9))}
        if block.mode == 0:
            max_k = max(float(np.max(np.abs(values))), 1e-9)
            max_kp = max(float(np.max(np.abs(first))), 1e-9)
            caps["lateral_acceleration"] = float(
                math.sqrt(limits.lateral_a_max/max_k))
            if limits.translation_omega_max is not None:
                caps["translation_angular_speed"] = float(
                    limits.translation_omega_max/max_k)
            if limits.translation_alpha_max is not None:
                caps["translation_angular_acceleration_speed"] = float(
                    math.sqrt(limits.translation_alpha_max/max_kp))
                acceleration_caps["translation_angular_acceleration"] = float(
                    limits.translation_alpha_max/max_k)
        else:
            caps["oblique_normal_acceleration"] = float(math.sqrt(
                limits.lateral_a_max/
                max(float(np.max(np.abs(first))), 1e-9)))
        diagnostic_caps = {}
        if limits.translation_speed_policy == "mode_switch_only":
            diagnostic_caps = {name: value for name, value in caps.items()
                               if name != "target_speed"}
            caps = {"target_speed": float(limits.target_v)}
            acceleration_caps = {"path_acceleration": float(limits.a_max)}
        vmax = min(caps.values())
        amax = min(acceleration_caps.values())
        interval_speed.append(max(1e-4, vmax))
        interval_accel.append(max(1e-4, amax))
        near = [name for name,value in caps.items()
                if value <= vmax*(1.+1e-6)+1e-9]
        accel_near = [name for name,value in acceleration_caps.items()
                      if value <= amax*(1.+1e-6)+1e-9]
        interval_limits.append({
            "local_s_interval_m": [float(left),float(right)],
            "global_s_interval_m": [float(global_s_offset+left),
                                     float(global_s_offset+right)],
            "speed_caps_mps": caps,
            "diagnostic_cap_equivalents_mps": diagnostic_caps,
            "raw_minimum_speed_cap_mps": float(vmax),
            "active_speed_caps": near,
            "acceleration_caps_mps2": acceleration_caps,
            "active_acceleration_caps": accel_near,
            "peak_sampling_points": int(len(query)),
            "peak_sampling_step_m": float((right-left)/max(1,len(query)-1))})
    cap_margin = 1. if limits.translation_speed_policy == "mode_switch_only" else .98
    cap = cap_margin*np.asarray([interval_speed[0]]+
        [min(interval_speed[i-1], interval_speed[i]) for i in range(1, len(stations)-1)]+
        [interval_speed[-1]])
    acceleration_limit = .90*limits.a_max
    jerk_limit = .90*limits.jerk_max

    def transition(low, high):
        change = abs(high-low)
        if change < 1e-12:
            return 0., 0., 0.
        tj = min(math.sqrt(change/jerk_limit), acceleration_limit/jerk_limit)
        ta = max(0., change/acceleration_limit-tj)
        return tj, ta, .5*(low+high)*(2*tj+ta)

    # Minima are genuine speed constraints. Maxima in sufficiently wide free
    # regions get acceleration/cruise/braking anchors at physical reachability
    # distances, instead of resetting acceleration every 20 cm.
    tolerance = max(1e-5, .01*limits.target_v)
    groups = []
    i = 0
    while i < len(cap):
        j = i+1
        while j < len(cap) and abs(cap[j]-cap[i]) <= tolerance:
            j += 1
        groups.append((i, j-1, float(np.min(cap[i:j]))))
        i = j
    minima = [(0., 0.)]
    for gi in range(1, len(groups)-1):
        first, last, value = groups[gi]
        if value < groups[gi-1][2] and value < groups[gi+1][2]:
            minima.append((float(stations[first]), value))
            if last != first:
                minima.append((float(stations[last]), value))
    minima.append((block.length, 0.))
    anchors = []
    for (left, v0), (right, v1) in zip(minima, minima[1:]):
        anchors.append((left, v0))
        inside = np.flatnonzero((stations > left+1e-9)&(stations < right-1e-9))
        if not len(inside):
            continue
        peak_index = int(inside[np.argmax(cap[inside])])
        peak = float(cap[peak_index])
        if peak <= max(v0, v1)+tolerance:
            continue
        lo = hi = peak_index
        while lo > 0 and stations[lo-1] > left and cap[lo-1] >= peak-tolerance:
            lo -= 1
        while hi+1 < len(cap) and stations[hi+1] < right and cap[hi+1] >= peak-tolerance:
            hi += 1
        ramp = transition(v0, peak)[2]
        braking = transition(peak, v1)[2]
        at_left = max(float(stations[lo]), left+ramp)
        at_right = min(float(stations[hi]), right-braking)
        if at_left <= at_right and at_left > left+1e-8 and at_right < right-1e-8:
            anchors.append((at_left, peak))
            if at_right-at_left > 1e-8:
                anchors.append((at_right, peak))
        else:
            anchors.append((.5*(stations[lo]+stations[hi]), peak))
    anchors.append(minima[-1])
    anchors.sort()
    compact = []
    for position, speed in anchors:
        if compact and position-compact[-1][0] <= 1e-9:
            compact[-1] = (position, min(speed, compact[-1][1]))
        else:
            compact.append((position, speed))
    positions = np.asarray([item[0] for item in compact])
    speeds = np.asarray([item[1] for item in compact])
    distances = np.diff(positions)

    def reachable_speed(low, high, distance):
        if transition(low, high)[2] <= distance:
            return high
        lo, hi = low, high
        for _ in range(36):
            mid = .5*(lo+hi)
            if transition(low, mid)[2] <= distance:
                lo = mid
            else:
                hi = mid
        return lo

    def connect_speeds():
        for i in range(1, len(speeds)):
            if speeds[i] > speeds[i-1]:
                speeds[i] = reachable_speed(speeds[i-1], speeds[i], distances[i-1])
        for i in range(len(speeds)-2, -1, -1):
            if speeds[i] > speeds[i+1]:
                speeds[i] = reachable_speed(speeds[i+1], speeds[i], distances[i])

    connect_speeds()
    for row in interval_limits:
        row["envelope_endpoint_caps_mps"] = [
            float(cap[max(0,min(len(cap)-1,int(np.searchsorted(stations,row["local_s_interval_m"][0]))))]),
            float(cap[max(0,min(len(cap)-1,int(np.searchsorted(stations,row["local_s_interval_m"][1]))))])]
        row["scheduled_anchor_speed_mps"] = [
            float(np.interp(row["local_s_interval_m"][0],positions,speeds)),
            float(np.interp(row["local_s_interval_m"][1],positions,speeds))]
    for attempt in range(32):
        phases, elapsed = [], start_time
        overall = dict((name, 0.) for name in (
            "speed", "acceleration", "jerk", "wheel_speed", "lateral_acceleration",
            "angular_speed", "angular_acceleration", "angular_jerk",
            "steer_rate", "steer_acceleration", "oblique_normal_acceleration", "wheel_angle"))
        for interval, (left, right) in enumerate(zip(positions[:-1], positions[1:])):
            v0, v1 = float(speeds[interval]), float(speeds[interval+1])
            if max(v0, v1) <= 1e-8:
                raise ValueError("continuous_speed_envelope_zero")
            tj, ta, _ = transition(v0, v1)
            if tj <= 1e-10:
                segments = [((right-left)/max(v0, 1e-12), 0.)]
            else:
                duration = 2.*(right-left)/(v0+v1)
                stretch = max(1., duration/(2.*tj+ta))
                sign = 1. if v1 > v0 else -1.
                j = sign*jerk_limit/stretch**2
                segments = [(tj*stretch, j)]
                if ta > 1e-10:
                    segments.append((ta*stretch, 0.))
                segments.append((tj*stretch, -j))
            cursor, velocity, accel = float(left), v0, 0.
            for duration, jerk in segments:
                distance = velocity*duration+.5*accel*duration**2+jerk*duration**3/6.
                end_speed = velocity+accel*duration+.5*jerk*duration**2
                end_accel = accel+jerk*duration
                phase = {"kind": "translation", "profile": "piecewise_quintic",
                    "block_index": block_index, "segment_index": len(phases),
                    "speed_anchor_interval": interval,
                    "mode": block.mode, "direction": block.direction,
                    "t0": elapsed, "duration": duration,
                    "s0": cursor, "s1": min(block.length, cursor+distance),
                    "start_speed": velocity, "end_speed": max(0., end_speed),
                    "start_acceleration": accel, "end_acceleration": end_accel,
                    "coefficients": [0., velocity*duration, .5*accel*duration**2,
                                     jerk*duration**3/6., 0., 0.]}
                peaks = _piece_peaks(block, phase, limits, table)
                phase["dynamic_peaks"] = peaks
                phases.append(phase)
                for name, value in peaks.items():
                    overall[name] = max(overall[name], value)
                cursor += distance
                elapsed += duration
                velocity, accel = end_speed, end_accel
        powers = _translation_time_limits(limits)
        if overall["wheel_angle"] > limits.steer_angle_max+1e-8:
            raise ValueError("continuous_steering_angle_limit")
        correction = np.ones(len(speeds))
        for phase in phases:
            ratio = min([1.]+[(bound/phase["dynamic_peaks"][name])**(1./power)
                for name, (bound, power) in powers.items() if bound is not None and
                phase["dynamic_peaks"][name] > bound*(1.+1e-7)])
            if ratio < 1.-1e-7:
                i = phase["speed_anchor_interval"]
                correction[i:i+2] = np.minimum(correction[i:i+2], max(.5, .97*ratio))
        if np.min(correction) >= 1.-1e-7:
            for phase in phases:
                phase["speed_envelope_active_limits"] = sorted({
                    name for row in interval_limits
                    if row["local_s_interval_m"][0] < phase["s1"]+1e-9 and
                       row["local_s_interval_m"][1] > phase["s0"]-1e-9
                    for name in row["active_speed_caps"]})
            return phases, elapsed-start_time, overall, interval_limits
        speeds *= correction
        connect_speeds()
    raise ValueError("continuous_piecewise_time_scaling_did_not_converge")


def _active_peak_limits(peaks, limits):
    bounds = {name: value[0] for name, value in _translation_time_limits(limits).items()}
    ratios = {name: float(peaks[name])/float(bound)
              for name,bound in bounds.items()
              if bound is not None and peaks.get(name,0.) > 0.}
    if not ratios:
        return []
    best=max(ratios.values())
    return [name for name,ratio in ratios.items() if ratio >= best*(1.-.01)]


def _low_speed_intervals(samples, phases, threshold=.2, minimum_duration=.5,
                         endpoint_exclusion_m=.5):
    result=[]
    by_block={}
    for phase in phases:
        if phase.get("kind")=="translation":
            by_block.setdefault(phase.get("block_index"),[]).append(phase)
    for block_index,block_phases in by_block.items():
        block_phases.sort(key=lambda phase:phase["t0"])
        start=min(float(phase["t0"]) for phase in block_phases)
        end=max(float(phase["t0"])+float(phase["duration"])
                for phase in block_phases)
        rows=[row for row in samples if start-1e-9 <= row[0] <= end+1e-9]
        if len(rows)<2:
            continue
        run_start=None;run_end=None;run_max=0.
        groups=[]
        for left,right in zip(rows,rows[1:]):
            low=(abs(left[2])<threshold and abs(right[2])<threshold)
            if low:
                if run_start is None:
                    run_start=left[0];run_max=max(abs(left[2]),abs(right[2]))
                run_end=right[0];run_max=max(run_max,abs(left[2]),abs(right[2]))
            elif run_start is not None:
                if run_end-run_start>minimum_duration:
                    groups.append((run_start,run_end,run_max))
                run_start=run_end=None;run_max=0.
        if run_start is not None and run_end-run_start>minimum_duration:
            groups.append((run_start,run_end,run_max))
        for t0,t1,vmax in groups:
            def arc_at(t):
                phase=next((item for item in block_phases
                    if item["t0"]-1e-9 <= t <=
                       item["t0"]+item["duration"]+1e-9),block_phases[-1])
                tau=min(1.,max(0.,(t-phase["t0"])/max(phase["duration"],1e-12)))
                if phase.get("profile")=="piecewise_quintic":
                    progress=_quintic_state(phase["coefficients"],tau,
                                            phase["duration"])[0]
                else:
                    progress=(phase["s1"]-phase["s0"])*quintic(tau)[0]
                return float(phase["s0"]+progress)
            s0,s1=arc_at(t0),arc_at(t1)
            block_length=max(float(phase["s1"]) for phase in block_phases)
            global_offset=(float(block_phases[0].get("global_s0",0.))-
                           float(block_phases[0]["s0"]))
            start_distance=max(0.,s0)
            end_distance=max(0.,block_length-s1)
            near_start=start_distance<=endpoint_exclusion_m
            near_end=end_distance<=endpoint_exclusion_m
            if near_start and near_end:
                kind=("start_acceleration" if start_distance<=end_distance
                      else "end_braking")
            elif near_start:
                kind="start_acceleration"
            elif near_end:
                kind="end_braking"
            else:
                kind="ordinary_translation"
            active=sorted({name for phase in block_phases
                if phase["t0"] < t1+1e-9 and
                   phase["t0"]+phase["duration"] > t0-1e-9
                for name in phase.get("speed_envelope_active_limits",[])})
            result.append({"block_index":block_index,
                "mode":block_phases[0].get("mode"),
                "direction":block_phases[0].get("direction"),
                "classification":kind,"t0_s":float(t0),"t1_s":float(t1),
                "duration_s":float(t1-t0),"s_interval_m":[float(s0),float(s1)],
                "global_s_interval_m":[float(global_offset+s0),
                                        float(global_offset+s1)],
                "distance_m":float(max(0.,s1-s0)),
                "max_abs_speed_mps":float(vmax),
                "active_speed_limits":active})
    return result


def _lookup_pose(rows, s, distances=None):
    distances = distances if distances is not None else [row[0] for row in rows]
    i = min(len(rows)-2, max(0, int(np.searchsorted(distances, s)-1)))
    a, b = rows[i], rows[i+1]
    ratio = min(1., max(0., (s-a[0])/max(b[0]-a[0], 1e-12)))
    return Pose(a[1].x+(b[1].x-a[1].x)*ratio,
                a[1].y+(b[1].y-a[1].y)*ratio,
                a[1].th+(b[1].th-a[1].th)*ratio)


def _mode_endpoint_control(block, at_end):
    if block.mode == 2:
        return 0.
    return block.control(block.length if at_end else 0.)[0]


def _append_phase_samples(samples, phase, start_pose, block, dt, spatial_dt,
                          geometry_rows=None, geometry_table=None):
    duration = phase["duration"]
    n = max(2, int(math.ceil(duration/dt)))
    rows = (geometry_rows if geometry_rows is not None else
            sample_block(block, start_pose, spatial_dt) if block.mode != 2 else None)
    distances = (geometry_table if geometry_table is not None else
                 np.asarray([row[0] for row in rows]) if rows is not None else None)
    for i in range(1, n+1):
        tau = i/n
        if phase.get("profile") == "piecewise_quintic":
            q, speed, _, _ = _quintic_state(
                phase["coefficients"], tau, duration)
            pose = _lookup_pose(rows, phase["s0"]+q, distances)
            v = block.direction*speed
        else:
            progress, ds, _ = quintic(tau)
        if block.mode == 2:
            pose = Pose(start_pose.x, start_pose.y,
                        start_pose.th+block.signed_angle*progress)
            v = 0.
        elif phase.get("profile") != "piecewise_quintic":
            pose = _lookup_pose(rows, block.length*progress, distances)
            v = block.direction*block.length*ds/duration
        samples.append((phase["t0"]+duration*tau, pose, v, block.mode))


class SegmentTimingCache:
    """Bounded request-local reuse of mode-consistent segment time solves.

    The exact immutable block and complete dynamic profile form the key.
    Geometry in world coordinates, mode-event holds, sampling and collision
    certificates are never cached. Cached phases use local time and distance;
    each caller receives its own mutable copy rebased to the current route.
    """
    def __init__(self, max_entries=32):
        if max_entries < 1:
            raise ValueError("segment_timing_cache_invalid_capacity")
        self.max_entries = max_entries
        self._entries = OrderedDict()
        self.hits = self.misses = self.evictions = 0

    @property
    def stats(self):
        return {"hits": self.hits, "misses": self.misses,
                "evictions": self.evictions, "entries": len(self._entries),
                "scope": "one_mode_master_request",
                "collision_certificates_reused": False}

    def translation(self, block, limits, block_index, start_time, global_s):
        key = (block, limits)
        if key in self._entries:
            self.hits += 1
            self._entries.move_to_end(key)
            result = deepcopy(self._entries[key])
        else:
            self.misses += 1
            result = _translation_phases(block, limits, 0, 0., 0.)
            self._entries[key] = deepcopy(result)
            if len(self._entries) > self.max_entries:
                self._entries.popitem(last=False)
                self.evictions += 1
        phases, duration, peaks, constraints = result
        for phase in phases:
            phase["block_index"] = block_index
            phase["t0"] += start_time
        for interval in constraints:
            if "block_index" in interval:
                interval["block_index"] = block_index
            interval["global_s_interval_m"] = [
                global_s+s for s in interval["local_s_interval_m"]]
        return phases, duration, peaks, constraints


def retime_continuous(plan, limits=None, dt=.02, spatial_dt=.005,
                      segment_cache=None):
    """Time-scale a continuous plan; only a mode change inserts stationary setup."""
    if not isinstance(plan, ContinuousPlan):
        raise TypeError("continuous_plan_required")
    limits = limits or MotionLimits()
    if not math.isfinite(dt) or dt <= 0 or not math.isfinite(spatial_dt) or spatial_dt <= 0:
        raise ValueError("continuous_invalid_sampling_interval")
    blocks = plan.blocks
    samples = [(0., plan.start, 0., blocks[0].mode if blocks else 0)]
    phases = []
    elapsed = moving = spinning = switch_hold = 0.
    switch_count = 0
    start_pose = plan.start
    previous = None
    dynamic_peaks = []
    speed_constraints = []
    global_s = 0.
    for index, block in enumerate(blocks):
        if previous is not None and block.mode != previous.mode:
            old_action = Action(previous.mode,
                                previous.signed_angle if previous.mode == 2 else previous.direction,
                                _mode_endpoint_control(previous, True))
            new_action = Action(block.mode,
                                block.signed_angle if block.mode == 2 else block.direction,
                                _mode_endpoint_control(block, False))
            old_angles, _ = wheel_configuration(old_action, limits)
            new_angles, _ = wheel_configuration(new_action, limits)
            delta = float(np.max(np.abs(new_angles-old_angles)))
            steer = (steering_rest_to_rest_duration(delta, limits.steer_rate_max,
                                      limits.steer_accel_max) if delta > 1e-8 else 0.)
            setup = limits.mode_setup_time
            hold = setup+steer
            steering_profile = "triangular_trapezoidal_rate_accel"
            overlap = False
            steer_rate_peak = min(limits.steer_rate_max,
                                  math.sqrt(delta*limits.steer_accel_max))
            steer_accel_peak = limits.steer_accel_max if delta > 1e-8 else 0.
            if limits.translation_speed_policy == "mode_switch_only":
                # One complete mode event; do not append another setup delay.
                hold = steer = float(limits.mode_switch_duration)
                setup = min(setup, hold)
                overlap = True
                steering_profile = "quintic_fixed_duration_kinematic"
                steer_rate_peak = 1.875*delta/hold
                steer_accel_peak = (10/math.sqrt(3))*delta/hold**2
            phases.append({"kind": "mode_switch", "from_mode": previous.mode,
                           "to_mode": block.mode, "t0": elapsed,
                           "duration": hold, "setup_s": setup,
                           "steering_s": steer,
                           "steering_profile": steering_profile,
                           "setup_overlaps_steering": overlap,
                           "steer_rate_peak_radps": steer_rate_peak,
                           "steer_accel_peak_radps2": steer_accel_peak,
                           "max_wheel_angle_delta_rad": delta,
                           "wheel_angles_start_rad": old_angles.tolist(),
                           "wheel_angles_end_rad": new_angles.tolist(),
                           "pose": [start_pose.x, start_pose.y, start_pose.th]})
            n = max(1, int(math.ceil(hold/dt)))
            samples.extend((elapsed+hold*i/n, start_pose, 0., block.mode)
                           for i in range(1, n+1))
            elapsed += hold
            switch_hold += hold
            switch_count += 1
        if block.mode == 2:
            duration, peaks = _duration_and_peaks(block, limits)
            if duration <= 1e-9:
                previous = block
                continue
            phase = {"kind": "spin", "block_index": index, "mode": block.mode,
                     "direction": block.direction, "t0": elapsed,
                     "duration": duration,
                     "start_pose": [start_pose.x, start_pose.y, start_pose.th],
                     "length_m": 0., "signed_angle_rad": block.signed_angle,
                     "control_knots": [], "dynamic_peaks": peaks}
            phases.append(phase)
            _append_phase_samples(samples, phase, start_pose, block, dt, spatial_dt)
            elapsed += duration
            spinning += duration
            start_pose = Pose(start_pose.x, start_pose.y,
                              start_pose.th+block.signed_angle)
        else:
            rows = sample_block(block, start_pose, spatial_dt)
            lookup_stations = np.asarray([row[0] for row in rows])
            if segment_cache is None:
                block_phases, duration, peaks, block_constraints = _translation_phases(
                    block, limits, index, elapsed, global_s)
            else:
                block_phases, duration, peaks, block_constraints = segment_cache.translation(
                    block, limits, index, elapsed, global_s)
            speed_constraints.extend(block_constraints)
            for phase in block_phases:
                phase["start_pose"] = [start_pose.x, start_pose.y, start_pose.th]
                phase["length_m"] = phase["s1"]-phase["s0"]
                phase["global_s0"] = global_s+phase["s0"]
                phase["signed_angle_rad"] = 0.
                phase["control_knots"] = []
                phases.append(phase)
                _append_phase_samples(samples, phase, start_pose, block, dt,
                                     spatial_dt, rows, lookup_stations)
            elapsed += duration
            moving += duration
            start_pose = rows[-1][1]
            dynamic_peaks.extend(phase["dynamic_peaks"]
                                 for phase in block_phases)
        global_s += block.length
        if block.mode == 2:
            dynamic_peaks.append(peaks)
        previous = block
    if not blocks:
        samples.append((dt, plan.start, 0., 0))
        elapsed = dt
    low_speed = _low_speed_intervals(samples, phases)
    return ContinuousTimedPlan(samples, phases, elapsed, moving, spinning,
                               switch_hold, switch_count, 0., 0., limits,
                               speed_constraints, low_speed)
