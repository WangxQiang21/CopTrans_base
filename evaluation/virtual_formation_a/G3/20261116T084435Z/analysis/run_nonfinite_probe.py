"""Independent direct probe of the G3 non-finite actual-witness check.

The probe builds the frozen straight-line request from the public test
fixture and calls the private audit functions directly.  It covers every
_DYNAMIC_METRICS entry with NaN, +inf and -inf across different vehicles and
wheels, the non-finite wheel angle, the non-finite raw curvature path, and the
finite pass / worst-overlimit behaviour.  All evidence is JSON-safe.
"""
import hashlib
import json
import math
from pathlib import Path

from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a import audit as audit_module
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GoalPolicy,
    InitialAssumption,
    VirtualARequest,
)

ROOT = Path(__file__).resolve().parents[5]
RUN = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures/virtual_a/open_requests.json"
NONFINITE = (("NaN", float("nan")),
             ("+Infinity", float("inf")),
             ("-Infinity", float("-inf")))


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def make_request():
    requests = json.loads(FIXTURES.read_text(encoding="utf-8"))
    base = requests["base"]
    record = requests["requests"]["STRAIGHT_LINE"]
    goal = record["goal_qf"]
    model = FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in base["model"]["offsets"]])
    shapes = {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in base["model"]["vehicle_shapes"]}
    limits = FormationTimingLimits(**base["limits"])
    request = VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=model,
        vehicle_shapes=shapes,
        start_qf=FormationPose(**base["start_qf"]),
        start_headings={
            int(key): float(value)
            for key, value in base["start_headings"].items()},
        goal_qf=FormationPose(goal["x"], goal["y"], goal["psi"]),
        initial=InitialAssumption(**base["initial"]),
        goal_policy=GoalPolicy(record["goal_policy"]),
        limits=limits,
    )
    return request


def actual_record(request, angle=0.0):
    metrics = [
        name for name, _limit, _order in audit_module._DYNAMIC_METRICS]
    wheel = {name: 0.0 for name in metrics}
    return {
        vehicle_id: {
            "wheels": {
                wheel_index: dict(wheel) for wheel_index in range(4)},
            "wheel_angle_rad": float(angle),
        }
        for vehicle_id in sorted(
            offset.vehicle_id for offset in request.model.offsets)
    }


def raw_dynamic(**changes):
    record = {
        "v_i": 0.5,
        "acceleration": (0.0, 0.0),
        "jerk": (0.0, 0.0),
        "omega_i": 0.0,
        "yaw_accel": 0.0,
        "yaw_jerk": 0.0,
        "lateral_accel": 0.0,
        "wheel_speeds": [0.0, 0.0, 0.0, 0.0],
        "steer_rates": [0.0, 0.0, 0.0, 0.0],
        "steer_accels": [0.0, 0.0, 0.0, 0.0],
        "C_i": 0.0,
    }
    record.update(changes)
    return record


def failure_fields(error):
    location = dict(error.location)
    json.dumps(location, allow_nan=False, sort_keys=True)
    return {
        "reason": error.reason,
        "detail": error.detail,
        "location": location,
    }


def check_nonfinite(error, vehicle_id, wheel_index, quantity, description):
    fields = failure_fields(error)
    require(fields["reason"] == "NONFINITE_AUDIT_VALUE", "reason")
    location = fields["location"]
    require(location["vehicle_id"] == vehicle_id, "vehicle")
    require(location["wheel_index"] == wheel_index, "wheel")
    require(location["quantity"] == quantity, "quantity")
    require(location["constraint"] == "NONFINITE_AUDIT_VALUE", "constraint")
    require("value" not in location, "nonfinite numeric location")
    require(description in fields["detail"], "detail description")
    require(quantity in fields["detail"], "detail quantity")
    return fields


def require(condition, label):
    if not condition:
        raise SystemExit("PROBE FAILED: " + label)


def main():
    request = make_request()
    metric_names = [
        name for name, _limit, _order in audit_module._DYNAMIC_METRICS]
    metric_cases = []
    covered_vehicles = set()
    covered_wheels = set()
    for metric_index, (name, _limit_name, _order) in enumerate(
            audit_module._DYNAMIC_METRICS):
        for kind_index, (description, value) in enumerate(NONFINITE):
            vehicle_id = 1 + (metric_index + kind_index) % 3
            wheel_index = (3 * metric_index + kind_index) % 4
            actual = actual_record(request)
            actual[vehicle_id]["wheels"][wheel_index][name] = value
            try:
                audit_module._audit_check_actual_metrics(actual, request)
            except audit_module._AuditFailure as error:
                fields = check_nonfinite(
                    error, vehicle_id, wheel_index, name, description)
            else:
                raise SystemExit(
                    "PROBE FAILED: {} {} was skipped".format(
                        name, description))
            metric_cases.append({
                "metric": name, "value": description,
                "vehicle_id": vehicle_id, "wheel_index": wheel_index,
                **fields})
            covered_vehicles.add(vehicle_id)
            covered_wheels.add(wheel_index)
    require(set(covered_vehicles) == {1, 2, 3}, "vehicle coverage")
    require(covered_wheels == {0, 1, 2, 3}, "wheel coverage")

    angle_cases = []
    for index, (description, value) in enumerate(NONFINITE):
        vehicle_id = 1 + index
        actual = actual_record(request)
        actual[vehicle_id]["wheel_angle_rad"] = value
        try:
            audit_module._audit_check_actual_metrics(actual, request)
        except audit_module._AuditFailure as error:
            fields = check_nonfinite(
                error, vehicle_id, None, "wheel_angle", description)
        else:
            raise SystemExit(
                "PROBE FAILED: wheel_angle {} was skipped".format(
                    description))
        angle_cases.append({
            "value": description, "vehicle_id": vehicle_id, **fields})

    curvature_cases = []
    for description, value in NONFINITE:
        try:
            audit_module._audit_actual_metrics(
                {2: raw_dynamic(C_i=value)}, request)
        except audit_module._AuditFailure as error:
            fields = check_nonfinite(error, 2, None, "C_i", description)
        else:
            raise SystemExit(
                "PROBE FAILED: C_i {} was hidden by atan2".format(
                    description))
        curvature_cases.append({
            "value": description, "vehicle_id": 2, **fields})

    source_cases = []
    source_specs = (
        ("v_i", float("nan"), "speed", 0, "NaN"),
        ("acceleration", (float("inf"), 0.0), "acceleration", 0,
         "+Infinity"),
        ("jerk", (0.0, float("nan")), "jerk", 0, "NaN"),
        ("omega_i", float("inf"), "yaw_rate", 0, "+Infinity"),
        ("yaw_accel", float("nan"), "yaw_acceleration", 0, "NaN"),
        ("yaw_jerk", float("-inf"), "yaw_jerk", 0, "+Infinity"),
        ("lateral_accel", float("inf"), "lateral_acceleration", 0,
         "+Infinity"),
        ("wheel_speeds", [0.0, float("nan"), 0.0, 0.0],
         "wheel_speed", 1, "NaN"),
        ("steer_rates", [0.0, 0.0, float("inf"), 0.0],
         "steer_rate", 2, "+Infinity"),
        ("steer_accels", [0.0, 0.0, 0.0, float("-inf")],
         "steer_acceleration", 3, "+Infinity"),
    )
    for field, value, metric, wheel_index, description in source_specs:
        actual = audit_module._audit_actual_metrics(
            {1: raw_dynamic(**{field: value})}, request)
        try:
            audit_module._audit_check_actual_metrics(actual, request)
        except audit_module._AuditFailure as error:
            fields = check_nonfinite(
                error, 1, wheel_index, metric, description)
        else:
            raise SystemExit(
                "PROBE FAILED: raw {} was sanitised".format(field))
        source_cases.append({
            "source": field, "metric": metric, "value": description,
            "vehicle_id": 1, "wheel_index": wheel_index, **fields})

    finite_actual = actual_record(request)
    require(audit_module._audit_check_actual_metrics(
        finite_actual, request) is True, "finite pass")

    limits = request.limits
    worst_actual = actual_record(request)
    # The larger ratio is visited after an earlier violation, so this
    # distinguishes worst selection from a first-violation implementation.
    worst_actual[1]["wheels"][0]["speed"] = 1.2 * limits.v_max
    worst_actual[3]["wheels"][2]["jerk"] = 2.0 * limits.jerk_max
    try:
        audit_module._audit_check_actual_metrics(worst_actual, request)
    except audit_module._AuditFailure as error:
        worst_fields = failure_fields(error)
    else:
        raise SystemExit("PROBE FAILED: finite overlimits passed")
    require(worst_fields["reason"] == "DYNAMIC_LIMIT", "finite reason")
    require(worst_fields["location"]["vehicle_id"] == 3, "worst vehicle")
    require(worst_fields["location"]["wheel_index"] == 2, "worst wheel")
    require(worst_fields["location"]["quantity"] == "jerk", "worst metric")
    require(worst_fields["location"]["value"] == 2.0 * limits.jerk_max,
            "worst value")
    require(worst_fields["location"]["limit"] == limits.jerk_max,
            "worst limit")

    last_actual = actual_record(request)
    # The larger ratio is visited first, and the smaller violation later,
    # so this distinguishes worst selection from last-violation overwrite.
    last_actual[1]["wheels"][0]["speed"] = 2.0 * limits.v_max
    last_actual[3]["wheels"][2]["jerk"] = 1.2 * limits.jerk_max
    try:
        audit_module._audit_check_actual_metrics(last_actual, request)
    except audit_module._AuditFailure as error:
        last_fields = failure_fields(error)
    else:
        raise SystemExit("PROBE FAILED: finite overlimits passed")
    require(last_fields["reason"] == "DYNAMIC_LIMIT", "last reason")
    require(last_fields["location"]["vehicle_id"] == 1, "last vehicle")
    require(last_fields["location"]["wheel_index"] == 0, "last wheel")
    require(last_fields["location"]["quantity"] == "speed", "last metric")
    require(last_fields["location"]["value"] == 2.0 * limits.v_max,
            "last value")
    require(last_fields["location"]["limit"] == limits.v_max,
            "last limit")

    dominate_actual = actual_record(request)
    dominate_actual[1]["wheels"][0]["speed"] = 1.2 * limits.v_max
    dominate_actual[3]["wheels"][2]["jerk"] = float("nan")
    try:
        audit_module._audit_check_actual_metrics(dominate_actual, request)
    except audit_module._AuditFailure as error:
        dominate_fields = check_nonfinite(error, 3, 2, "jerk", "NaN")
    else:
        raise SystemExit("PROBE FAILED: non-finite lost to finite maximum")

    mixed_angle_cases = []
    for vehicle_id, value, description in (
            (1, float("nan"), "NaN"),
            (3, float("inf"), "+Infinity")):
        mixed_actual = actual_record(request)
        mixed_actual[1]["wheels"][0]["speed"] = 1.2 * limits.v_max
        mixed_actual[vehicle_id]["wheel_angle_rad"] = value
        try:
            audit_module._audit_check_actual_metrics(mixed_actual, request)
        except audit_module._AuditFailure as error:
            fields = check_nonfinite(
                error, vehicle_id, None, "wheel_angle", description)
        else:
            raise SystemExit(
                "PROBE FAILED: angle non-finite lost to finite maximum")
        mixed_angle_cases.append({
            "value": description, "vehicle_id": vehicle_id, **fields})

    angle_limit = limits.steer_angle_max
    angle_actual = actual_record(request)
    angle_actual[2]["wheel_angle_rad"] = 1.5 * angle_limit
    try:
        audit_module._audit_check_actual_metrics(angle_actual, request)
    except audit_module._AuditFailure as error:
        angle_fields = failure_fields(error)
    else:
        raise SystemExit("PROBE FAILED: finite wheel angle passed")
    require(angle_fields["reason"] == "WHEEL_ANGLE_LIMIT",
            "finite angle reason")
    require(angle_fields["location"]["vehicle_id"] == 2,
            "finite angle vehicle")
    require(angle_fields["location"]["wheel_index"] is None,
            "finite angle wheel")
    require(angle_fields["location"]["quantity"] == "wheel_angle",
            "finite angle quantity")

    output = {
        "schema": "virtual_rigid_a.g3.nonfinite_probe.v1",
        "status": "PASS",
        "run_id": RUN.name,
        "fixture_path": str(FIXTURES.relative_to(ROOT)),
        "fixture_sha256": sha256(FIXTURES),
        "metric_names": metric_names,
        "metric_case_count": len(metric_cases),
        "metric_cases": metric_cases,
        "wheel_angle_case_count": len(angle_cases),
        "wheel_angle_cases": angle_cases,
        "curvature_case_count": len(curvature_cases),
        "curvature_cases": curvature_cases,
        "generation_source_case_count": len(source_cases),
        "generation_source_cases": source_cases,
        "mixed_angle_case_count": len(mixed_angle_cases),
        "mixed_angle_cases": mixed_angle_cases,
        "covered_vehicles": sorted(covered_vehicles),
        "covered_wheels": sorted(covered_wheels),
        "finite_cases": {
            "below_limit_passed": True,
            "two_overlimits_reason": worst_fields,
            "two_overlimits_later_smaller_reason": last_fields,
            "nonfinite_dominates_finite_overlimit": dominate_fields,
            "wheel_angle_overlimit_reason": angle_fields,
        },
        "note": (
            "Direct private-function probe.  Non-finite cases are "
            "synthetic inputs, not naturally occurring motion values."),
    }
    print(json.dumps(output, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()