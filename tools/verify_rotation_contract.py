#!/usr/bin/env python3
"""Independent differential verification of the rigid rotation contract.

For every candidate the centre derivative at ``lambda=0`` is recomputed from

    p_i(lambda) = q_F(lambda) + R(psi_F(lambda)) r_i
    dp_i/dlambda = dpsi/L R(psi_F) J (r_i - pivot)

and projected on the actual body heading ``theta_i``.  The signed speed is
``dot(dp_i/dlambda, u(theta_i))``; an A body may not have a lateral component
and its curvature must satisfy ``kappa = theta'/v``.  The stored
``direction``/``control`` values are only compared afterwards, so a wrong
constructor label cannot certify itself.

The tool covers CCW/CW rotation, tangent and opposite-tangent bodies,
forward/reverse travel, a non-zero pivot and an initial cargo heading that
crosses +/-pi.  It also tampers ``direction`` and ``control`` to prove the
independent audit rejects them.  This is S5 evidence only: passing here still
does not certify timing, collisions or publication.
"""
import argparse
import json
import math
from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "catkin_ws/src/fwis_v3_optimizer/src"))

from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_kinematics import (  # noqa: E402
    ACKERMANN,
    OBLIQUE,
    SPIN,
    FormationKinematicBlock,
    KinematicLimits,
    assign_formation_modes,
    audit_mode_assignment,
    wrap_angle,
)

WEDGE = (
    VehicleOffset(1, 1.0, 0.0),
    VehicleOffset(2, 0.0, 1.0),
    VehicleOffset(3, 0.0, -1.0),
)
TOL = 1e-12


def primitive(dpsi, pivot=(0.0, 0.0)):
    return SimpleNamespace(
        geometry_type=1, dx=0.0, dy=0.0, dpsi=dpsi,
        pivot_x=pivot[0], pivot_y=pivot[1])


def tangent_headings(offsets, pivot, start_psi):
    return {
        offset.vehicle_id: wrap_angle(
            math.atan2(offset.x - pivot[0],
                       -(offset.y - pivot[1])) + start_psi)
        for offset in offsets
    }


def opposite(headings):
    return {vehicle_id: wrap_angle(value + math.pi)
            for vehicle_id, value in headings.items()}


def differential(segment, block, offset):
    length = block.parameter_length
    slope = block.delta_psi / length
    rx = offset.x - block.pivot_x
    ry = offset.y - block.pivot_y
    cosine = math.cos(block.start_qf.psi)
    sine = math.sin(block.start_qf.psi)
    local_vx = -slope * ry
    local_vy = slope * rx
    velocity_x = cosine * local_vx - sine * local_vy
    velocity_y = sine * local_vx + cosine * local_vy
    signed_speed = (
        velocity_x * math.cos(segment.theta_start) +
        velocity_y * math.sin(segment.theta_start))
    lateral_speed = (
        -velocity_x * math.sin(segment.theta_start) +
        velocity_y * math.cos(segment.theta_start))
    theta_rate = (segment.theta_end - segment.theta_start) / length
    curvature = (theta_rate / signed_speed
                 if abs(signed_speed) > TOL else None)
    return {
        "signed_speed": signed_speed,
        "lateral_speed": lateral_speed,
        "velocity_magnitude": math.hypot(velocity_x, velocity_y),
        "theta_rate": theta_rate,
        "curvature_theta_over_v": curvature,
        "expected_direction": (1 if signed_speed > 0.0 else -1
                               if signed_speed < 0.0 else 0),
    }


def run_case(name, dpsi, start_psi=0.0, pivot=(0.0, 0.0),
             use_opposite=False, allow_reverse=False):
    block = FormationKinematicBlock.from_primitive(
        FormationPose(4.0, -2.0, start_psi), primitive(dpsi, pivot), WEDGE)
    headings = tangent_headings(WEDGE, pivot, start_psi)
    if use_opposite:
        headings = opposite(headings)
    assignment = assign_formation_modes(
        block, headings, allow_reverse=allow_reverse)
    result = {
        "name": name,
        "dpsi": dpsi,
        "start_psi": start_psi,
        "pivot": list(pivot),
        "opposite_headings": use_opposite,
        "allow_reverse": allow_reverse,
        "assignment_success": bool(assignment.success),
        "assignment_reason": assignment.reason,
        "assignment_detail": assignment.detail,
        "segments": [],
    }
    if not assignment.success:
        result["all_pass"] = False
        return result, assignment
    audit = audit_mode_assignment(
        assignment, allow_reverse=allow_reverse)
    result["audit_success"] = bool(audit.success)
    result["audit_reason"] = audit.reason
    result["audit_detail"] = audit.detail
    worst_lateral = 0.0
    worst_curvature = 0.0
    worst_direction = 0.0
    for segment in assignment.segments:
        offset = next(item for item in WEDGE
                      if item.vehicle_id == segment.vehicle_id)
        values = differential(segment, block, offset)
        direction_error = abs(
            values["expected_direction"] - segment.direction)
        curvature_error = (
            abs(values["curvature_theta_over_v"] - segment.control)
            if values["curvature_theta_over_v"] is not None else 0.0)
        lateral_error = abs(values["lateral_speed"]) if (
            segment.mode == ACKERMANN) else 0.0
        worst_direction = max(worst_direction, direction_error)
        worst_curvature = max(worst_curvature, curvature_error)
        worst_lateral = max(worst_lateral, lateral_error)
        result["segments"].append({
            "vehicle_id": segment.vehicle_id,
            "mode": {ACKERMANN: "A", OBLIQUE: "O", SPIN: "S"}[segment.mode],
            "body_heading": segment.theta_start,
            "stored_direction": segment.direction,
            "expected_direction": values["expected_direction"],
            "signed_speed": values["signed_speed"],
            "lateral_speed": values["lateral_speed"],
            "stored_control": segment.control,
            "curvature_theta_over_v": values["curvature_theta_over_v"],
            "direction_error": direction_error,
            "lateral_speed_error": lateral_error,
            "curvature_error": curvature_error,
        })
    result["max_direction_error"] = worst_direction
    result["max_lateral_speed"] = worst_lateral
    result["max_curvature_error"] = worst_curvature
    result["all_pass"] = bool(
        assignment.success and audit.success and
        worst_direction <= TOL and worst_lateral <= 1e-12 and
        worst_curvature <= 1e-12)
    return result, assignment


def tamper_case(name, source, change):
    segments = list(source.segments)
    index = next(
        index for index, segment in enumerate(segments)
        if segment.mode == ACKERMANN)
    segments[index] = change(segments[index])
    from dataclasses import replace
    tampered = replace(source, segments=tuple(segments))
    audit = audit_mode_assignment(tampered, allow_reverse=True)
    return {
        "name": name,
        "vehicle_id": segments[index].vehicle_id,
        "audit_success": bool(audit.success),
        "reason": audit.reason,
        "detail": audit.detail,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    cases = []
    assignments = {}
    for spec in (
            ("ccw_tangent_forward", 0.5, 0.0, (0.0, 0.0), False, False),
            ("cw_tangent_reverse", -0.5, 0.0, (0.0, 0.0), False, True),
            ("cw_opposite_forward", -0.5, 0.0, (0.0, 0.0), True, False),
            ("ccw_opposite_reverse", 0.5, 0.0, (0.0, 0.0), True, True),
            ("ccw_nonzero_pivot", 0.4, 0.0, (0.0, 0.1), False, False),
            ("cw_nonzero_pivot_reverse", -0.4, 0.0, (0.0, 0.1), False,
             True),
            ("ccw_cross_pi", 0.6, 2.9, (0.0, 0.0), False, False),
    ):
        name, dpsi, start_psi, pivot, opposite, allow_reverse = spec
        result, assignment = run_case(
            name, dpsi, start_psi=start_psi, pivot=pivot,
            use_opposite=opposite, allow_reverse=allow_reverse)
        cases.append(result)
        assignments[name] = assignment

    tampered = []
    ccw = assignments["ccw_tangent_forward"]
    cw = assignments["cw_tangent_reverse"]
    from dataclasses import replace as _replace
    tampered.append(tamper_case(
        "ccw_flipped_direction", ccw,
        lambda segment: _replace(segment, direction=-segment.direction)))
    tampered.append(tamper_case(
        "ccw_flipped_curvature", ccw,
        lambda segment: _replace(segment, control=-segment.control)))
    # A CW reverse segment has control +1/r; the forward-tangent value is
    # -1/r, so this is the classic fixed label error.
    tampered.append(tamper_case(
        "cw_fixed_tangent_curvature", cw,
        lambda segment: _replace(segment, control=-segment.control)))

    output = {
        "tool": "tools/verify_rotation_contract.py",
        "contract": {
            "centre_derivative":
                "dp_i/dlambda = dpsi/L R(psi_F) J (r_i - pivot)",
            "signed_speed": "dot(dp_i/dlambda, u(theta_i))",
            "ackermann_curvature": "theta'/v",
            "direction": "sign of the signed centre speed",
        },
        "limits": {
            "r_min_ackermann": KinematicLimits().r_min_ackermann,
            "heading_tolerance_rad":
                KinematicLimits().heading_tolerance_rad,
        },
        "cases": cases,
        "tamper": tampered,
        "all_pass": all(case["all_pass"] for case in cases) and all(
            not item["audit_success"] for item in tampered),
        "scope": "S5 candidate/audit only; no timing, collision or "
                 "publication certification",
    }
    Path(args.out).write_text(
        json.dumps(output, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "all_pass": output["all_pass"],
        "cases": len(cases),
        "tamper_rejected": sum(
            1 for item in tampered if not item["audit_success"]),
        "output": args.out,
    }, indent=2))
    return 0 if output["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())