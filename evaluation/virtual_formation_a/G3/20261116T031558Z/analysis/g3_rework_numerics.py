"""Independent numerical evidence for G3 rework 01.

This script never trusts the planner's or the audit's interval code as an
oracle.  It evaluates the rigid-offset spatial derivatives and the common-time
jerk from handwritten closed-form expressions, then checks containment against
the corresponding local interval-certificate bounds.  It also records the
Codex degenerate counterexample, narrow four-sign coverage, real positive
paths covering the front vehicle, a true jerk-overlimit negative, and a
fail-closed coarse-interval case.
"""
import json
import math
from pathlib import Path

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a import audit as audit_module
from fwis_v3_optimizer.formation_continuous_a import timing as timing_module
from fwis_v3_optimizer.formation_continuous_a.audit import audit_dynamics
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    DynamicAuditConfig,
    GoalPolicy,
    InitialAssumption,
    TimingConfig,
    VirtualARequest,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.timing import (
    quintic_progress,
    retime_path,
)
from fwis_v3_optimizer.formation_continuous_a.trajectory import (
    VirtualATimedPlan,
)

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "virtual_a"
REQUESTS = json.loads((FIXTURE_DIR / "open_requests.json").read_text())
BASE = REQUESTS["base"]
SUBDIVISIONS = 8


def model_from_fixture():
    return FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in BASE["model"]["offsets"]])


def shapes_from_fixture():
    return {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in BASE["model"]["vehicle_shapes"]}


def limits_from_fixture(**changes):
    values = dict(BASE["limits"])
    values.update(changes)
    return FormationTimingLimits(**values)


def request_for(path, **limit_changes):
    record = REQUESTS["requests"]["STRAIGHT_LINE"]
    goal = path.endpoint()
    return VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=model_from_fixture(),
        vehicle_shapes=shapes_from_fixture(),
        start_qf=FormationPose(**BASE["start_qf"]),
        start_headings={
            int(key): float(value)
            for key, value in BASE["start_headings"].items()},
        goal_qf=FormationPose(goal.x, goal.y, goal.psi),
        initial=InitialAssumption(**BASE["initial"]),
        goal_policy=GoalPolicy(record["goal_policy"]),
        limits=limits_from_fixture(**limit_changes),
    )


def bind(path, request):
    path.request_sha256 = request_fingerprint(request)
    return path


def profile_from_records(records):
    return C2CurvatureProfile.from_knots([
        ControlKnot(float(s), float(k), float(eta))
        for s, k, eta in records])


def h_derivatives(tau):
    """Independent textbook quintic time-law derivatives."""
    h1 = 30.0 * tau * tau * (1.0 - tau) ** 2
    h2 = 60.0 * tau - 180.0 * tau * tau + 120.0 * tau ** 3
    h3 = 60.0 - 360.0 * tau + 360.0 * tau * tau
    return h1, h2, h3


def independent_tau(fraction):
    """Bisection for h(tau)=fraction without calling production inverse code."""
    if fraction <= 0.0:
        return 0.0
    if fraction >= 1.0:
        return 1.0
    low, high = 0.0, 1.0
    for _ in range(200):
        middle = 0.5 * (low + high)
        if quintic_progress(middle) < fraction:
            low = middle
        else:
            high = middle
    tau = 0.5 * (low + high)
    if abs(quintic_progress(tau) - fraction) > 1e-14:
        raise RuntimeError("independent tau inverse failed")
    return tau


def coefficients_at(profile, station, side="right"):
    """Return exact Hermite coefficients for the segment containing station."""
    segments = profile.interval_bounds(0.0, profile.length_m)["segments"]
    if station <= 0.0:
        return segments[0]
    if station >= profile.length_m:
        return segments[-1]
    for index, record in enumerate(segments):
        s0, s1 = record["s0"], record["s1"]
        if s0 < station < s1:
            return record
        if station == s0:
            if side == "left" and index > 0:
                return segments[index - 1]
            return record
        if station == s1:
            if side == "right" and index + 1 < len(segments):
                return segments[index + 1]
            return record
    raise RuntimeError("station {} not locatable".format(station))


def eval_cubic(coefficients, q):
    value = 0.0
    for coefficient in reversed(coefficients):
        value = value * q + coefficient
    return value


def eval_derivatives(record, q):
    """Evaluate k, dk/ds, d²k/ds² and d³k/ds³ from Hermite coefficients."""
    c0, c1, c2, c3 = record["coefficients"]
    h = record["h"]
    k = ((c3 * q + c2) * q + c1) * q + c0
    eta = ((3.0 * c3 * q + 2.0 * c2) * q + c1) / h
    u = (6.0 * c3 * q + 2.0 * c2) / (h * h)
    g = 6.0 * c3 / (h * h * h)
    return k, eta, u, g


def spatial_derivatives(offset, k, eta, u):
    """Handwritten rigid-offset local derivatives."""
    aoff, boff = offset.x, offset.y
    p_s = (1.0 - boff * k, aoff * k)
    p_ss = (-boff * eta - aoff * k * k,
            k - boff * k * k + aoff * eta)
    p_sss = (-k * k + boff * k ** 3 - 3.0 * aoff * k * eta - boff * u,
             eta - 3.0 * boff * k * eta + aoff * u - aoff * k ** 3)
    return p_s, p_ss, p_sss


def local_jerk(offset, k, eta, u, w, a, j):
    p_s, p_ss, p_sss = spatial_derivatives(offset, k, eta, u)
    return (p_sss[0] * w ** 3 + 3.0 * p_ss[0] * w * a + p_s[0] * j,
            p_sss[1] * w ** 3 + 3.0 * p_ss[1] * w * a + p_s[1] * j)


def ratio(actual, bound):
    """Nonnegative ratio for a scalar upper bound."""
    if bound == 0.0:
        return 0.0 if actual == 0.0 else math.inf
    return abs(actual) / abs(bound)


def interval_ratio(actual, lower, upper):
    """Absolute-value ratio for a signed enclosing interval."""
    magnitude = max(abs(lower), abs(upper))
    if magnitude == 0.0:
        return 0.0 if actual == 0.0 else math.inf
    return abs(actual) / magnitude


def point_record(offset, k, eta, u, w, a, j):
    _p_s, _p_ss, p_sss = spatial_derivatives(offset, k, eta, u)
    jerk = local_jerk(offset, k, eta, u, w, a, j)
    return {
        "k": k, "eta": eta, "u": u,
        "k_times_eta": k * eta,
        "p_sss": list(p_sss),
        "p_sss_norm": math.hypot(*p_sss),
        "jerk": list(jerk),
        "jerk_norm": math.hypot(*jerk),
    }


def minimum_wheel_record(bounds_record, vehicle_id):
    wheels = bounds_record["vehicles"][vehicle_id]["wheels"]
    first = wheels[min(wheels)]
    result = dict(first)
    for wheel in wheels.values():
        for name in ("jerk", "jerk_component_norm_upper"):
            result[name] = min(result[name], wheel[name])
        for name in ("jerk_component_upper", "p_sss_component_upper"):
            result[name] = (
                min(result[name][0], wheel[name][0]),
                min(result[name][1], wheel[name][1]))
        for name in ("p_sss_norm_upper",):
            result[name] = min(result[name], wheel[name])
    return result


def compare_point_to_bounds(point, bounds_record):
    """Check every z3/jerk component and norm against the local certificate."""
    vehicles = {}
    all_contained = True
    max_ratios = {"jerk_norm": 0.0, "jerk_component": 0.0,
                  "p_sss_norm": 0.0, "p_sss_component": 0.0}
    for offset in BASE_MODEL.offsets:
        point_value = point[offset.vehicle_id]
        bound = minimum_wheel_record(bounds_record, offset.vehicle_id)
        z3_interval = bound["p_sss_local_interval"]
        jerk_interval = bound["jerk_local_interval"]
        checks = {}
        for axis in (0, 1):
            checks["p_sss_{}".format(axis)] = {
                "actual": point_value["p_sss"][axis],
                "lower": z3_interval[axis][0],
                "upper": z3_interval[axis][1],
                "contained": (z3_interval[axis][0]
                              <= point_value["p_sss"][axis]
                              <= z3_interval[axis][1]),
            }
            checks["jerk_{}".format(axis)] = {
                "actual": point_value["jerk"][axis],
                "lower": jerk_interval[axis][0],
                "upper": jerk_interval[axis][1],
                "contained": (jerk_interval[axis][0]
                              <= point_value["jerk"][axis]
                              <= jerk_interval[axis][1]),
            }
        checks["p_sss_norm"] = {
            "actual": point_value["p_sss_norm"],
            "upper": bound["p_sss_norm_upper"],
            "contained": point_value["p_sss_norm"]
                         <= bound["p_sss_norm_upper"],
        }
        checks["jerk_norm"] = {
            "actual": point_value["jerk_norm"],
            "upper": bound["jerk"],
            "contained": point_value["jerk_norm"] <= bound["jerk"],
        }
        checks["jerk_component_norm"] = {
            "actual": point_value["jerk_norm"],
            "upper": bound["jerk_component_norm_upper"],
            "contained": point_value["jerk_norm"]
                         <= bound["jerk_component_norm_upper"],
        }
        for name, record in checks.items():
            all_contained = all_contained and record["contained"]
        max_ratios["jerk_norm"] = max(
            max_ratios["jerk_norm"],
            ratio(checks["jerk_norm"]["actual"], checks["jerk_norm"]["upper"]))
        max_ratios["jerk_component"] = max(
            max_ratios["jerk_component"],
            max(interval_ratio(
                checks["jerk_{}".format(axis)]["actual"],
                checks["jerk_{}".format(axis)]["lower"],
                checks["jerk_{}".format(axis)]["upper"])
                for axis in (0, 1)))
        max_ratios["p_sss_norm"] = max(
            max_ratios["p_sss_norm"],
            ratio(checks["p_sss_norm"]["actual"],
                  checks["p_sss_norm"]["upper"]))
        max_ratios["p_sss_component"] = max(
            max_ratios["p_sss_component"],
            max(interval_ratio(
                checks["p_sss_{}".format(axis)]["actual"],
                checks["p_sss_{}".format(axis)]["lower"],
                checks["p_sss_{}".format(axis)]["upper"])
                for axis in (0, 1)))
        vehicles[offset.vehicle_id] = {
            "offset": [offset.x, offset.y],
            "checks": checks,
        }
    return {"vehicles": vehicles, "contained": all_contained,
            "max_ratios": max_ratios}


def point_map(profile, station, tau, duration, total_s, side="right"):
    record = coefficients_at(profile, station, side)
    q = (station - record["s0"]) / record["h"]
    k, eta, u, _g = eval_derivatives(record, q)
    h1, h2, h3 = h_derivatives(tau)
    w = total_s * h1 / duration
    a = total_s * h2 / (duration * duration)
    j = total_s * h3 / (duration ** 3)
    return {
        offset.vehicle_id: point_record(offset, k, eta, u, w, a, j)
        for offset in BASE_MODEL.offsets}, (k, eta, u, w, a, j)


def degenerate_evidence():
    expected_ps_sss = (-0.04, 0.099)
    expected_norm = math.hypot(*expected_ps_sss)
    offset = VehicleOffset(1, 1.0, 0.0)
    jets = {"k": (0.1, 0.1), "eta": (0.1, 0.1),
            "u": (0.0, 0.0), "g": (0.0, 0.0)}
    timing = timing_module._vehicle_bounds(
        jets, offset, limits_from_fixture(),
        (1.0, 1.0), (0.0, 0.0), (0.0, 0.0), 0.247, 0.182)
    audit = audit_module._audit_interval_vehicle(
        jets, offset, limits_from_fixture(),
        (1.0, 1.0), (0.0, 0.0), (0.0, 0.0), 0.247, 0.182)
    return {
        "input": {
            "offset": [1.0, 0.0], "k": 0.1, "eta": 0.1,
            "u": 0.0, "g": 0.0, "w": 1.0, "a_time": 0.0,
            "j_time": 0.0,
        },
        "expected_p_sss": list(expected_ps_sss),
        "expected_p_sss_norm": expected_norm,
        "old_defective_bounds": {
            "timing_p_sss_norm_upper": 0.10100000000000009,
            "timing_jerk": 0.10100000000000023,
            "audit_jerk": 0.10100000000000023,
        },
        "timing": {
            "p_sss_component_upper": list(timing["p_sss_component_upper"]),
            "p_sss_norm_upper": timing["p_sss_norm_upper"],
            "jerk": timing["jerk"],
            "jerk_component_norm_upper":
                timing["jerk_component_norm_upper"],
            "p_sss_local_interval":
                [list(value) for value in timing["p_sss_local_interval"]],
            "jerk_local_interval":
                [list(value) for value in timing["jerk_local_interval"]],
            "covers_expected_norm":
                timing["jerk"] >= expected_norm,
            "covers_expected_components": [
                timing["p_sss_local_interval"][axis][0]
                <= expected_ps_sss[axis]
                <= timing["p_sss_local_interval"][axis][1]
                for axis in (0, 1)],
        },
        "audit": {
            "p_sss_component_upper": list(audit["p_sss_component_upper"]),
            "p_sss_norm_upper": audit.get(
                "p_sss_norm_upper", math.hypot(
                    *[audit["p_sss_component_upper"][axis]
                      for axis in (0, 1)])),
            "jerk": audit["jerk"],
            "jerk_component_norm_upper":
                audit["jerk_component_norm_upper"],
            "p_sss_local_interval":
                [list(value) for value in audit["p_sss_local_interval"]],
            "jerk_local_interval":
                [list(value) for value in audit["jerk_local_interval"]],
            "covers_expected_norm": audit["jerk"] >= expected_norm,
            "covers_expected_components": [
                audit["p_sss_local_interval"][axis][0]
                <= expected_ps_sss[axis]
                <= audit["p_sss_local_interval"][axis][1]
                for axis in (0, 1)],
        },
    }


def narrow_evidence():
    width = 1e-5
    offsets = BASE_MODEL.offsets
    samples = ((0.12, 0.09, 0.05, 0.3),
               (0.12, -0.09, -0.05, -0.3),
               (-0.12, 0.09, -0.07, -0.2),
               (-0.12, -0.09, 0.07, 0.2))
    records = []
    for implementation, function in (
            ("timing", timing_module._vehicle_bounds),
            ("audit", audit_module._audit_interval_vehicle)):
        for offset in offsets:
            for k, eta, u, g in samples:
                jets = {
                    "k": (k - width, k + width),
                    "eta": (eta - width, eta + width),
                    "u": (u - width, u + width),
                    "g": (g - width, g + width),
                }
                bounds_record = function(
                    jets, offset, limits_from_fixture(),
                    (0.8, 1.2), (-0.3, 0.4), (-0.5, 0.6), 0.247, 0.182)
                actual_k = k + 0.37 * width
                actual_eta = eta - 0.41 * width
                actual_u = u + 0.53 * width
                actual_w = 0.8 + 0.31 * 0.4
                actual_a = -0.3 + 0.29 * 0.7
                actual_j = -0.5 + 0.61 * 1.1
                _p_s, _p_ss, p_sss = spatial_derivatives(
                    offset, actual_k, actual_eta, actual_u)
                actual_jerk = local_jerk(
                    offset, actual_k, actual_eta, actual_u,
                    actual_w, actual_a, actual_j)
                checks = {}
                for axis in (0, 1):
                    checks["p_sss_{}".format(axis)] = {
                        "actual": p_sss[axis],
                        "interval": list(
                            bounds_record["p_sss_local_interval"][axis]),
                        "contained": (
                            bounds_record["p_sss_local_interval"][axis][0]
                            <= p_sss[axis]
                            <= bounds_record["p_sss_local_interval"][axis][1]),
                    }
                    checks["jerk_{}".format(axis)] = {
                        "actual": actual_jerk[axis],
                        "interval": list(
                            bounds_record["jerk_local_interval"][axis]),
                        "contained": (
                            bounds_record["jerk_local_interval"][axis][0]
                            <= actual_jerk[axis]
                            <= bounds_record["jerk_local_interval"][axis][1]),
                    }
                checks["p_sss_norm"] = {
                    "actual": math.hypot(*p_sss),
                    "upper": bounds_record["p_sss_norm_upper"],
                    "contained": math.hypot(*p_sss)
                                 <= bounds_record["p_sss_norm_upper"],
                }
                checks["jerk_norm"] = {
                    "actual": math.hypot(*actual_jerk),
                    "upper": bounds_record["jerk"],
                    "contained": math.hypot(*actual_jerk)
                                 <= bounds_record["jerk"],
                }
                checks["jerk_component_norm"] = {
                    "actual": math.hypot(*actual_jerk),
                    "upper": bounds_record["jerk_component_norm_upper"],
                    "contained": math.hypot(*actual_jerk)
                                 <= bounds_record["jerk_component_norm_upper"],
                }
                records.append({
                    "implementation": implementation,
                    "vehicle_id": offset.vehicle_id,
                    "offset": [offset.x, offset.y],
                    "center": {"k": k, "eta": eta, "u": u, "g": g},
                    "point": {
                        "k": actual_k, "eta": actual_eta, "u": actual_u,
                        "w": actual_w, "a_time": actual_a,
                        "j_time": actual_j},
                    "checks": checks,
                    "all_contained": all(
                        record["contained"] for record in checks.values()),
                })
    failures = [record for record in records if not record["all_contained"]]
    return {
        "interval_half_width": width,
        "case_count": len(records),
        "failure_count": len(failures),
        "records": records,
    }


def plan_evidence(records, name, limit_changes=None):
    profile = profile_from_records(records)
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    request = request_for(path, **(limit_changes or {}))
    bind(path, request)
    result = retime_path(request, path, TimingConfig())
    if result.status != "DYNAMIC_FEASIBLE" or result.plan is None:
        raise RuntimeError("{} planning failed: {}".format(
            name, result.as_dict()))
    report = audit_dynamics(request, result.plan, DynamicAuditConfig())
    if not report.passed:
        raise RuntimeError("{} audit failed: {}".format(
            name, report.as_dict()))
    return {
        "name": name, "records": records, "profile": profile,
        "path": path, "request": request, "result": result,
        "report": report,
    }


def positive_path_evidence(sign, label):
    def curvature(s):
        return sign * (0.08 * s * s - 0.025 * s ** 3)

    def curvature_rate(s):
        return sign * (0.16 * s - 0.075 * s * s)

    records = [(float(s), curvature(s), curvature_rate(s))
               for s in range(0, 5)]
    evidence = plan_evidence(records, label)
    request = evidence["request"]
    result = evidence["result"]
    report = evidence["report"]
    path = evidence["path"]
    total_s = result.plan.total_S_m
    duration = result.duration_s
    bind_record = audit_module._audit_bind_plan(request, result.plan, None)
    pieces = bind_record["pieces"]
    profile_records = path.profile.interval_bounds(
        0.0, path.profile.length_m)["segments"]
    if report.metrics["leaves"] != SUBDIVISIONS * len(profile_records):
        raise RuntimeError("certificate refined cells; local-cell evidence "
                           "would not match the recorded leaves")
    samples = []
    node_samples = []
    front = {"sample_count": 0, "nonzero_k_times_eta": 0,
             "max_jerk_ratio": 0.0, "max_component_ratio": 0.0,
             "max_p_sss_ratio": 0.0, "sign_combinations": set()}
    for index, record in enumerate(profile_records):
        s0, s1, h = record["s0"], record["s1"], record["h"]
        piece = pieces[index]
        for q in (0.1, 0.3, 0.5, 0.7, 0.9):
            station = s0 + h * q
            tau = independent_tau(station / total_s)
            points, jets = point_map(
                path.profile, station, tau, duration, total_s)
            cell = min(SUBDIVISIONS - 1, int(math.floor(q * SUBDIVISIONS)))
            q0 = cell / float(SUBDIVISIONS)
            q1 = (cell + 1) / float(SUBDIVISIONS)
            tau0 = independent_tau((s0 + h * q0) / total_s)
            tau1 = independent_tau((s0 + h * q1) / total_s)
            bounds_record = audit_module._audit_interval_leaf(
                piece, q0, q1, tau0, tau1, request, total_s, duration)
            comparison = compare_point_to_bounds(points, bounds_record)
            entry = {
                "kind": "interior",
                "segment_index": index,
                "q": q,
                "cell": [q0, q1],
                "s": station,
                "tau": tau,
                "t": duration * tau,
                "k": jets[0], "eta": jets[1], "u": jets[2],
                "k_times_eta": jets[0] * jets[1],
                "w": jets[3], "a_time": jets[4], "j_time": jets[5],
                "signs": [math.copysign(1.0, jets[0])
                          if jets[0] != 0.0 else 0.0,
                          math.copysign(1.0, jets[1])
                          if jets[1] != 0.0 else 0.0],
                "comparison": comparison,
            }
            samples.append(entry)
            front_point = points[1]
            if abs(front_point["k_times_eta"]) > 1e-12:
                front["nonzero_k_times_eta"] += 1
                front["sign_combinations"].add(
                    (1 if jets[0] > 0.0 else -1,
                     1 if jets[1] > 0.0 else -1))
            front["sample_count"] += 1
            front["max_jerk_ratio"] = max(
                front["max_jerk_ratio"],
                comparison["max_ratios"]["jerk_norm"])
            front["max_component_ratio"] = max(
                front["max_component_ratio"],
                comparison["max_ratios"]["jerk_component"])
            front["max_p_sss_ratio"] = max(
                front["max_p_sss_ratio"],
                max(comparison["max_ratios"]["p_sss_norm"],
                    comparison["max_ratios"]["p_sss_component"]))
        if index + 1 < len(profile_records):
            station = s1
            tau = independent_tau(station / total_s)
            left_record = profile_records[index]
            right_record = profile_records[index + 1]
            left_points, left_jets = point_map(
                path.profile, station, tau, duration, total_s, "left")
            right_points, right_jets = point_map(
                path.profile, station, tau, duration, total_s, "right")
            q0 = 1.0 - 1.0 / SUBDIVISIONS
            tau0 = independent_tau(
                (left_record["s0"] + left_record["h"] * q0) / total_s)
            left_bounds = audit_module._audit_interval_leaf(
                pieces[index], q0, 1.0, tau0, tau, request, total_s,
                duration)
            q1 = 1.0 / SUBDIVISIONS
            tau1 = independent_tau(
                (right_record["s0"] + right_record["h"] * q1) / total_s)
            right_bounds = audit_module._audit_interval_leaf(
                pieces[index + 1], 0.0, q1, tau, tau1, request, total_s,
                duration)
            node_samples.append({
                "kind": "node",
                "node_index": index + 1,
                "s": station,
                "tau": tau,
                "t": duration * tau,
                "k_left": left_jets[0], "eta_left": left_jets[1],
                "u_left": left_jets[2],
                "k_right": right_jets[0], "eta_right": right_jets[1],
                "u_right": right_jets[2],
                "left_comparison": compare_point_to_bounds(
                    left_points, left_bounds),
                "right_comparison": compare_point_to_bounds(
                    right_points, right_bounds),
            })
    certificate = report.metrics["pointwise_interval_checks"]
    if certificate["leaf_count"] != report.metrics["leaves"]:
        raise RuntimeError("pointwise certificate did not cover all leaves")
    if any(not sample["comparison"]["contained"] for sample in samples):
        raise RuntimeError("{} positive pointwise containment failed".format(
            label))
    for node in node_samples:
        if not (node["left_comparison"]["contained"]
                and node["right_comparison"]["contained"]):
            raise RuntimeError("{} node containment failed".format(label))
    return {
        "name": label,
        "direction_sign": sign,
        "knot_records": [[s, k, eta] for s, k, eta in records],
        "timing": {
            "status": result.status,
            "T_s": result.duration_s,
            "rescalings": result.rescalings,
            "required_duration_s":
                result.bounds_summary["required_duration_s"],
            "selected_duration_s":
                result.bounds_summary["selected_duration_s"],
            "worst": result.bounds_summary["worst"],
        },
        "model_sha256": result.plan.model_sha256(),
        "request_sha256": result.plan.request_sha256,
        "audit": {
            "passed": report.passed,
            "reason": report.reason,
            "check_count": report.check_count,
            "leaves": report.metrics["leaves"],
            "max_utilization": report.metrics["max_utilization"],
            "pointwise_interval_checks": certificate,
        },
        "front_vehicle": {
            **front,
            "sign_combinations": sorted(front["sign_combinations"]),
        },
        "samples": samples,
        "node_samples": node_samples,
        "sample_count": len(samples),
        "node_sample_count": len(node_samples),
        "all_contained": True,
    }


G_JUMP_RECORDS = (
    (0.0, 0.0, 0.0),
    (1.0, 0.020, 0.050),
    (2.0, 0.105, 0.115),
    (3.0, 0.25166666666666666, 0.185),
)


def shortened_plan(request, result, factor):
    selection = json.loads(json.dumps(result.bounds_summary))
    duration = result.duration_s * factor
    required = duration / float(selection["safety_factor"])
    scale = required / float(selection["required_duration_s"])
    metrics = selection["metrics_at_T0_s"]
    for record in metrics.values():
        order = int(record["order"])
        record["upper_at_T0_s"] = float(record["upper_at_T0_s"]) * \
            scale ** order
        record["required_duration_s"] = (
            float(record["upper_at_T0_s"]) / float(record["limit"])) ** \
            (1.0 / order)
    selection["required_duration_s"] = max(
        record["required_duration_s"] for record in metrics.values())
    selection["selected_duration_s"] = duration
    return VirtualATimedPlan._build_motion(
        request, result.plan.path, duration, selection)


def dense_jerk_scan(request, plan, samples=20001):
    """Independent dense scan; reported as a witness, never as a proof."""
    total_s = plan.total_S_m
    duration = plan.duration_s
    offsets = request.model.offsets
    maximum = {"jerk": -1.0}
    maximum_interior = {"jerk": -1.0}
    maximum_away = {"jerk": -1.0}
    over = []
    over_away = []
    for index in range(samples):
        tau = index / float(samples - 1)
        station = quintic_progress(tau) * total_s
        record = coefficients_at(plan.path.profile, station)
        q = (station - record["s0"]) / record["h"]
        k, eta, u, _g = eval_derivatives(record, q)
        h1, h2, h3 = h_derivatives(tau)
        w = total_s * h1 / duration
        a = total_s * h2 / (duration * duration)
        j = total_s * h3 / (duration ** 3)
        for offset in offsets:
            jerk = local_jerk(offset, k, eta, u, w, a, j)
            norm = math.hypot(*jerk)
            record_value = {
                "jerk": norm, "vehicle_id": offset.vehicle_id,
                "t": duration * tau, "tau": tau, "s": station,
                "k": k, "eta": eta, "w": w, "a_time": a,
                "j_time": j, "components": list(jerk),
            }
            if norm > maximum["jerk"]:
                maximum = record_value
            if 0.0 < tau < 1.0 and norm > maximum_interior["jerk"]:
                maximum_interior = record_value
            if 0.01 <= tau <= 0.99 and norm > maximum_away["jerk"]:
                maximum_away = record_value
            if norm > request.limits.jerk_max:
                over.append({
                    "t": duration * tau, "s": station,
                    "vehicle_id": offset.vehicle_id, "jerk": norm,
                    "limit": request.limits.jerk_max,
                })
                if 0.01 <= tau <= 0.99:
                    over_away.append({
                        "t": duration * tau, "s": station,
                        "tau": tau, "vehicle_id": offset.vehicle_id,
                        "jerk": norm, "limit": request.limits.jerk_max,
                    })
    return {
        "sample_count": samples,
        "sample_method": "uniform_tau_scan_witness_not_whole_domain_proof",
        "maximum": maximum,
        "maximum_open_interval": maximum_interior,
        "maximum_away_from_endpoints": maximum_away,
        "over_limit_count": len(over),
        "first_over_limit": over[0] if over else None,
        "last_over_limit": over[-1] if over else None,
        "over_limit_count_away_from_endpoints": len(over_away),
        "first_over_limit_away_from_endpoints":
            over_away[0] if over_away else None,
        "last_over_limit_away_from_endpoints":
            over_away[-1] if over_away else None,
    }


def negative_evidence():
    profile = profile_from_records(G_JUMP_RECORDS)
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    high = 1e6
    changes = {
        "v_max": high, "a_max": high, "jerk_max": 0.2,
        "yaw_rate_max": high, "yaw_accel_max": high,
        "yaw_jerk_max": high, "lateral_accel_max": high,
        "wheel_speed_max": high, "steer_rate_max": high,
        "steer_accel_max": high,
    }
    request = request_for(path, **changes)
    bind(path, request)
    result = retime_path(request, path, TimingConfig())
    if result.status != "DYNAMIC_FEASIBLE":
        raise RuntimeError("negative-case base plan failed: {}".format(
            result.as_dict()))
    accepted = audit_dynamics(request, result.plan, DynamicAuditConfig())
    if not accepted.passed:
        raise RuntimeError("negative-case accepted plan failed audit")
    shortened = shortened_plan(request, result, 0.8)
    rejected = audit_dynamics(request, shortened, DynamicAuditConfig())
    if rejected.passed or rejected.reason != "DYNAMIC_LIMIT":
        raise RuntimeError("shortened plan was not rejected on a limit")
    scan_short = dense_jerk_scan(request, shortened)
    scan_accepted = dense_jerk_scan(request, result.plan)
    if not (scan_short["maximum"]["jerk"] > request.limits.jerk_max):
        raise RuntimeError("dense scan did not confirm the over-limit")
    if not (scan_short["maximum_away_from_endpoints"]["jerk"]
            > request.limits.jerk_max):
        raise RuntimeError(
            "dense scan found no over-limit away from the endpoints")
    return {
        "limits": {"jerk_max": request.limits.jerk_max},
        "knot_records": [[s, k, eta] for s, k, eta in G_JUMP_RECORDS],
        "accepted_plan": {
            "status": result.status,
            "T_s": result.duration_s,
            "worst": result.bounds_summary["worst"],
            "audit_passed": accepted.passed,
            "audit_reason": accepted.reason,
            "check_count": accepted.check_count,
            "scanned_maximum": scan_accepted["maximum"],
        },
        "shortened_plan": {
            "factor": 0.8,
            "T_s": shortened.duration_s,
            "audit_passed": rejected.passed,
            "audit_reason": rejected.reason,
            "first_failure": rejected.first_failure,
            "scanned_maximum": scan_short["maximum"],
            "scanned_maximum_open_interval":
                scan_short["maximum_open_interval"],
            "scanned_maximum_away_from_endpoints":
                scan_short["maximum_away_from_endpoints"],
            "scan_over_limit_count": scan_short["over_limit_count"],
            "scan_over_limit_count_away_from_endpoints":
                scan_short["over_limit_count_away_from_endpoints"],
            "first_over_limit": scan_short["first_over_limit"],
            "last_over_limit": scan_short["last_over_limit"],
            "first_over_limit_away_from_endpoints":
                scan_short["first_over_limit_away_from_endpoints"],
            "last_over_limit_away_from_endpoints":
                scan_short["last_over_limit_away_from_endpoints"],
        },
    }


def fail_closed_evidence():
    profile = profile_from_records(G_JUMP_RECORDS)
    path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), profile,
                        model_from_fixture())
    request = request_for(path)
    bind(path, request)
    result = retime_path(request, path, TimingConfig())
    if result.status != "DYNAMIC_FEASIBLE":
        raise RuntimeError("fail-closed base plan failed")
    coarse = audit_dynamics(
        request, result.plan,
        DynamicAuditConfig(interval_subdivisions_per_segment=1,
                           max_depth=0))
    refined = audit_dynamics(
        request, result.plan,
        DynamicAuditConfig(interval_subdivisions_per_segment=1,
                           max_depth=20))
    if coarse.passed or coarse.reason != "BOUND_UNCONFIRMED":
        raise RuntimeError("coarse bound did not fail closed")
    if not refined.passed or refined.metrics["leaves"] <= 3:
        raise RuntimeError("refinement did not confirm the bounds")
    return {
        "T_s": result.duration_s,
        "coarse": {
            "passed": coarse.passed,
            "reason": coarse.reason,
            "first_failure": coarse.first_failure,
            "leaves": coarse.metrics.get("leaves"),
        },
        "refined": {
            "passed": refined.passed,
            "reason": refined.reason,
            "leaves": refined.metrics["leaves"],
            "max_utilization": refined.metrics["max_utilization"],
        },
    }


BASE_MODEL = model_from_fixture()


def main():
    evidence = {
        "schema": "virtual_rigid_a.g3.rework_numerics.v1",
        "generated_by": "analysis/g3_rework_numerics.py",
        "degenerate_counterexample": degenerate_evidence(),
        "narrow_interval_coverage": narrow_evidence(),
        "positive_paths": [
            positive_path_evidence(1.0, "positive_curvature_cubic"),
            positive_path_evidence(-1.0, "negative_curvature_cubic"),
        ],
        "jerk_overlimit_negative": negative_evidence(),
        "broad_interval_fail_closed": fail_closed_evidence(),
    }
    if not all(value["all_contained"]
               for value in evidence["positive_paths"]):
        raise SystemExit("positive containment failed")
    if evidence["narrow_interval_coverage"]["failure_count"]:
        raise SystemExit("narrow interval coverage failed")
    for name in ("timing", "audit"):
        record = evidence["degenerate_counterexample"][name]
        if not (record["covers_expected_norm"]
                and all(record["covers_expected_components"])):
            raise SystemExit("degenerate counterexample coverage failed")
    path = RUN / "analysis" / "g3_rework_numerics.json"
    path.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    print("wrote", path)
    print("degenerate:", evidence["degenerate_counterexample"]["timing"][
        "jerk"], evidence["degenerate_counterexample"]["audit"]["jerk"])
    print("narrow cases:", evidence["narrow_interval_coverage"]["case_count"],
          "failures:", evidence["narrow_interval_coverage"]["failure_count"])
    for positive in evidence["positive_paths"]:
        print("positive", positive["name"], "T", positive["timing"]["T_s"],
              "samples", positive["sample_count"],
              "nodes", positive["node_sample_count"],
              "front signs", positive["front_vehicle"]["sign_combinations"],
              "front max jerk ratio",
              positive["front_vehicle"]["max_jerk_ratio"])
    negative = evidence["jerk_overlimit_negative"]["shortened_plan"]
    print("negative", negative["audit_reason"],
          negative["first_failure"])
    print("negative scan max", negative["scanned_maximum"])
    print("negative scan max away from endpoints",
          negative["scanned_maximum_away_from_endpoints"])
    fail_closed = evidence["broad_interval_fail_closed"]
    print("fail-closed", fail_closed["coarse"]["reason"],
          "refined leaves", fail_closed["refined"]["leaves"])


if __name__ == "__main__":
    main()