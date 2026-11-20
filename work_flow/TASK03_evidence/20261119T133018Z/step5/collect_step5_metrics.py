#!/usr/bin/env python3
"""TASK03 Step 5 evidence: EDT soft-clearance ablation on one union scene.

The scene is a frozen straight-line request on a synthetic map with one thin
obstacle above the formation.  Every arm shares the same request, grid, model,
chain, allocation, template identity, planning-context SHA and solver budget.
Only ``corridor_clearance_weight`` changes inside a target pair; the separate
target pair keeps the weight fixed while changing the (test-only) preference
target.

The clearance field is the existing authoritative-grid EDT cache used by
``_clearance_proxy``; no second distance field is built, and no artificial
corridor edge enters it.  ``weight=0`` must reproduce the no-proxy NLP exactly;
the certificate is checked separately and never consumes the soft weight.

Run from the worktree root with the TASK03 plan's PYTHONPATH.
"""
import json
import math
import os
import sys
import traceback

ROOT = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)
TESTS = os.path.join(ROOT, "tests")
if TESTS not in sys.path:
    sys.path.insert(0, TESTS)

from fwis_v3_optimizer.formation_continuous_a.audit import (  # noqa: E402
    audit_dynamics,
)
from fwis_v3_optimizer.formation_continuous_a.collision import (  # noqa: E402
    audit_collision,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    CollisionConfig,
    CorridorChain,
    DynamicAuditConfig,
    ObstacleContext,
    TimingConfig,
    VirtualAMapRequest,
    grid_signature,
    planning_context_sha256,
)
from fwis_v3_optimizer.formation_continuous_a.corridor import (  # noqa: E402
    allocate_corridors,
    make_corridor_guide,
)
from fwis_v3_optimizer.formation_continuous_a.formation_certificate import (  # noqa: E402
    certify_formation_path,
    verify_formation_certificate,
)
from fwis_v3_optimizer.formation_continuous_a.formation_corridor import (  # noqa: E402
    single_region_template,
)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (  # noqa: E402
    solve_open_path_guided,
)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (  # noqa: E402
    _cached_clearance_field,
)
from fwis_v3_optimizer.formation_continuous_a.timing import (  # noqa: E402
    retime_path,
)
from tests.test_virtual_a_corridor_optimizer import _box  # noqa: E402
from tests.test_virtual_a_optimizer import (  # noqa: E402
    planner_config,
    request_from_fixture,
)
from virtual_a_g4_support import empty_grid  # noqa: E402

INTERVALS = 8
TARGET_DEFAULT_M = 0.05
TARGET_TEST_M = 0.6
MAX_SECONDS = 20.0
TIMING = TimingConfig()
COLLISION = CollisionConfig()


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _build_grid():
    """Authoritative grid with one thin obstacle above the formation."""
    grid = empty_grid()
    ix0 = int(round((2.0 - grid.ox) / grid.res))
    ix1 = int(round((4.0 - grid.ox) / grid.res))
    iy0 = int(round((1.6 - grid.oy) / grid.res))
    iy1 = int(round((2.2 - grid.oy) / grid.res))
    grid.occ[iy0:iy1, ix0:ix1] = True
    return grid, (2.0, 4.0, 1.6, 2.2)


def _reference_config():
    return planner_config(
        guidance_mode="corridor",
        formation_corridor_scope=(
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
        solver="ipopt", max_initials=1, max_seconds=MAX_SECONDS,
        corridor_clearance_weight=0.0,
        corridor_clearance_target_m=TARGET_DEFAULT_M)


def _config(weight, target):
    return planner_config(
        guidance_mode="corridor",
        formation_corridor_scope=(
            FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1),
        solver="ipopt", max_initials=1, max_seconds=MAX_SECONDS,
        corridor_clearance_weight=float(weight),
        corridor_clearance_target_m=float(target))


def _trajectory(path, stations=256):
    return tuple(
        path.state_at_s(path.length_m * index / float(stations)).qf
        for index in range(stations + 1))


def _trajectory_difference(first, second):
    position = max(
        math.hypot(a.x - b.x, a.y - b.y)
        for a, b in zip(first, second))
    heading = max(abs(_wrap(a.psi - b.psi)) for a, b in zip(first, second))
    return {"max_position_m": position, "max_heading_rad": heading}


def _max_abs_kappa(path, stations=512):
    return max(
        abs(path.profile.jet_at(path.length_m * index / float(stations)).kappa)
        for index in range(stations + 1))


def _edt_clearance(path, request, field, stations=512):
    """Smallest cached-EDT body clearance along the final path."""
    worst = None
    for index in range(stations + 1):
        s_m = path.length_m * index / float(stations)
        state = path.state_at_s(s_m)
        for vehicle in state.vehicles:
            shape = request.vehicle_shapes[vehicle.vehicle_id]
            clearance, direction, _gradient = field.direction_at(
                vehicle.x, vehicle.y)
            if direction is None or not math.isfinite(direction):
                direction = vehicle.theta
            support = (
                0.5 * shape.length_m * abs(math.cos(vehicle.theta - direction))
                + 0.5 * shape.width_m * abs(math.sin(
                    vehicle.theta - direction)))
            value = float(clearance) - support
            if worst is None or value < worst["clearance_m"]:
                worst = {
                    "clearance_m": value,
                    "s_m": s_m,
                    "vehicle_id": int(vehicle.vehicle_id),
                    "edt_m": float(clearance),
                    "support_m": support,
                }
    return worst


def _certificate_summary(certificate):
    if not certificate:
        return None
    return {
        "schema": certificate.get("schema"),
        "status": certificate.get("status"),
        "reason": certificate.get("reason"),
        "certificate_sha256": certificate.get("certificate_sha256"),
        "intervals_checked": certificate.get("intervals_checked"),
        "intervals_verified": certificate.get("intervals_verified"),
        "max_depth_reached": certificate.get("max_depth_reached"),
        "max_outward_residual_m":
            certificate.get("max_outward_residual_m"),
        "path_sha256":
            (certificate.get("identity") or {}).get("path_sha256"),
        "chain_digest":
            (certificate.get("identity") or {}).get("chain_digest"),
        "allocation_digest":
            (certificate.get("identity") or {}).get("allocation_digest"),
        "guide_sha256":
            (certificate.get("identity") or {}).get("guide_sha256"),
        "cut_check_count": len(certificate.get("cut_checks") or ()),
        # The optimizer certifies with the remaining wall budget, so this
        # max_seconds differs across arms and is part of the sealed limits.
        "limits": dict(certificate.get("limits") or {}),
        "timed_out": certificate.get("timed_out"),
        "interval_limit_reached":
            certificate.get("interval_limit_reached"),
        "elapsed_s": certificate.get("elapsed_s"),
    }


def _audit_summary(report):
    if report is None:
        return None
    return {
        "passed": bool(getattr(report, "passed", False)),
        "reason": getattr(report, "reason", ""),
        "detail": getattr(report, "detail", ""),
        "check_count": getattr(report, "check_count", None),
    }


def _three_layer(request, path, grid, context):
    timing = retime_path(
        request, path, TIMING, planning_context=context)
    record = {
        "timing_status": timing.status,
        "timing_reason": timing.reason,
        "timing_detail": timing.detail,
        "duration_s": getattr(timing, "T_s", None),
        "dynamics": None,
        "collision": None,
    }
    if timing.status in ("DYNAMIC_FEASIBLE", "NO_MOTION") and \
            getattr(timing, "plan", None) is not None:
        dynamics = audit_dynamics(
            request, timing.plan, DynamicAuditConfig(),
            expected_planning_context=context)
        collision = audit_collision(
            request, timing.plan, grid, COLLISION)
        record["dynamics"] = _audit_summary(dynamics)
        record["collision"] = _audit_summary(collision)
    return record


def main():
    evidence = {
        "schema": "task03_step5_soft_clearance_ablation_v1",
        "defaults": {
            "corridor_clearance_weight":
                float(planner_config().corridor_clearance_weight),
            "corridor_clearance_target_m":
                float(planner_config().corridor_clearance_target_m),
            "collision_baseline_margin_m":
                float(COLLISION.baseline_margin_m),
            "target_or_weight_relaxed": False,
        },
    }
    try:
        grid, obstacle = _build_grid()
        field = _cached_clearance_field(grid)
        request = request_from_fixture("STRAIGHT_LINE")
        grid_sha = grid_signature(grid)
        reference = _reference_config()
        reference_map = VirtualAMapRequest(
            request, grid, reference, TIMING, COLLISION)
        context = reference_map.context()
        context_sha = planning_context_sha256(context)
        chain = CorridorChain(
            regions=(_box(
                -2.0, -2.5, 8.0, 2.5,
                (0.0, 0.0), (6.0, 0.0)),),
            route_source="step5_fixture", grid_sha256=grid_sha)
        guide = make_corridor_guide(
            chain, (0.0, 0.0), (6.0, 0.0))
        allocation = allocate_corridors(chain, guide, INTERVALS)
        evidence["scene"] = {
            "request_sha256": reference_map.request_sha256,
            "context_sha256": context_sha,
            "grid_sha256": grid_sha,
            "obstacle_box_xyxy": obstacle,
            "corridor_box": [-2.0, -2.5, 8.0, 2.5],
            "field_downsample": int(field.downsample),
            "field_sha256": field.sha256(),
            "edt_at_body_centres": {
                "(3.0, 1.0)": field.value_at(3.0, 1.0),
                "(3.0, 0.0)": field.value_at(3.0, 0.0),
                "(3.0, -1.0)": field.value_at(3.0, -1.0),
            },
        }
        arms = (
            ("no_soft_context_default_target", None, 0.0, TARGET_DEFAULT_M),
            ("weight_zero_default_target", "context", 0.0, TARGET_DEFAULT_M),
            ("weight_one_default_target", "context", 1.0, TARGET_DEFAULT_M),
            ("weight_zero_test_target", "context", 0.0, TARGET_TEST_M),
            ("weight_one_test_target", "context", 1.0, TARGET_TEST_M),
        )
        results = {}
        trajectories = {}
        accepted = {}
        knots = {}

        def knot_vector(path):
            return tuple(
                (float(knot.value), float(knot.derivative))
                for knot in path.profile.knots)
        for name, context_mode, weight, target in arms:
            config = _config(weight, target)
            template = single_region_template(
                request, chain, allocation, guide,
                planning_context_sha256=context_sha, region_indices=0,
                margin_m=1e-5, margin_source="step5_ablation")
            obstacle_context = None if context_mode is None else \
                ObstacleContext(
                    grid_sha256=grid_sha,
                    source="step5_ablation_clearance_field",
                    clearance_field=field)
            result = solve_open_path_guided(
                request, config, route_guides=(guide,),
                obstacle_context=obstacle_context,
                corridor_chain=chain, corridor_allocation=allocation,
                formation_template=template,
                formation_planning_context_sha256=context_sha)
            certificate = result.formation_certificate or {}
            verification = None
            if result.path is not None and \
                    certificate.get("status") == "VERIFIED":
                verification = verify_formation_certificate(
                    certificate, result.path, request, chain, template,
                    allocation=allocation, guide=guide,
                    planning_context_sha256=context_sha)
            record = {
                "config": {
                    "corridor_clearance_weight": weight,
                    "corridor_clearance_target_m": target,
                    "max_seconds": MAX_SECONDS,
                    "max_initials": 1,
                    "solver": "ipopt",
                },
                "status": result.status,
                "reason": result.reason,
                "detail": result.detail,
                "solver_used": result.solver_used,
                "wall_time_s": result.wall_time_s,
                "attempts": [{
                    "solver": attempt.solver,
                    "solver_status": attempt.solver_status,
                    "status": attempt.status,
                    "iterations": attempt.iterations,
                    "objective": attempt.objective,
                    "constraint_residual": attempt.constraint_residual,
                    "wall_time_s": attempt.wall_time_s,
                    "rejection_reason": attempt.rejection_reason,
                    "detail": attempt.detail,
                } for attempt in result.attempts],
                "path_is_none": result.path is None,
                "certificate": _certificate_summary(certificate),
                "certificate_verification": verification,
                "geometry_audit": _audit_summary(result.audit),
                "corridor_halfspace_constraint_count":
                    result.corridor_halfspace_constraint_count,
                "formation_obb_constraint_count":
                    result.formation_obb_constraint_count,
                "formation_template_id": result.formation_template_id,
            }
            if result.path is not None:
                record["length_m"] = result.path.length_m
                record["max_abs_kappa"] = _max_abs_kappa(result.path)
                record["endpoint"] = {
                    "x": result.path.endpoint().x,
                    "y": result.path.endpoint().y,
                    "psi": result.path.endpoint().psi,
                }
                record["edt_body_clearance"] = _edt_clearance(
                    result.path, request, field)
                record["three_layer"] = _three_layer(
                    request, result.path, grid, context)
                trajectories[name] = _trajectory(result.path)
                knots[name] = knot_vector(result.path)
                accepted[name] = {
                    "path": result.path, "template": template}
            results[name] = record

        evidence["arms"] = results

        def difference(first, second):
            if first not in trajectories or second not in trajectories:
                return {"status": "NOT_COMPARABLE",
                        "missing": [name for name in (first, second)
                                    if name not in trajectories]}
            return _trajectory_difference(
                trajectories[first], trajectories[second])

        evidence["comparisons"] = {
            "zero_vs_no_context_default": difference(
                "weight_zero_default_target",
                "no_soft_context_default_target"),
            "one_vs_zero_default": difference(
                "weight_one_default_target", "weight_zero_default_target"),
            "one_vs_zero_test_target": difference(
                "weight_one_test_target", "weight_zero_test_target"),
            "zero_default_vs_zero_test_target": difference(
                "weight_zero_default_target", "weight_zero_test_target"),
        }
        # The zero-weight arm and the no-context arm compile the same NLP
        # (unit-tested bit-for-bit).  Two independent IPOPT runs may still
        # land on numerically different, equally valid optima; this records
        # whether the returned profiles are bit-identical in this run.
        if "weight_zero_default_target" in knots and \
                "no_soft_context_default_target" in knots:
            first = knots["weight_zero_default_target"]
            second = knots["no_soft_context_default_target"]
            evidence["zero_vs_no_context_profile"] = {
                "knot_count": len(first),
                "bit_identical": first == second,
                "max_abs_value_delta": max(
                    abs(a[0] - b[0]) for a, b in zip(first, second)),
                "max_abs_derivative_delta": max(
                    abs(a[1] - b[1]) for a, b in zip(first, second)),
            }
        # The same accepted path re-certified under two different soft configs
        # must carry a bit-identical certificate: the certificate function has
        # no soft-weight input at all.
        entry = accepted.get("weight_zero_default_target")
        if entry is None:
            evidence["certificate_config_independence"] = {
                "status": "NO_ACCEPTED_CANDIDATE"}
        else:
            re_certified = {}
            for tag, weight, target in (
                    ("zero_default", 0.0, TARGET_DEFAULT_M),
                    ("high_test_only", 5.0, TARGET_TEST_M)):
                certificate = certify_formation_path(
                    entry["path"], request, chain, entry["template"],
                    allocation=allocation, guide=guide,
                    planning_context_sha256=context_sha,
                    config=_config(weight, target))
                re_certified[tag] = {
                    "config": {
                        "corridor_clearance_weight": weight,
                        "corridor_clearance_target_m": target},
                    "status": certificate["status"],
                    "certificate_sha256": certificate["certificate_sha256"],
                }
            # Isolate why the per-arm certificates carry different hashes:
            # the optimizer seals the remaining wall budget into ``limits``,
            # and that differs from arm to arm.  Same path, explicit limits,
            # different limit -> hash must change.
            limited = certify_formation_path(
                entry["path"], request, chain, entry["template"],
                allocation=allocation, guide=guide,
                planning_context_sha256=context_sha,
                config=_config(0.0, TARGET_DEFAULT_M),
                max_seconds=5.0)
            evidence["certificate_config_independence"] = {
                "same_accepted_path": True,
                "configs": re_certified,
                "identical_certificate_sha256": (
                    re_certified["zero_default"]["certificate_sha256"]
                    == re_certified["high_test_only"][
                        "certificate_sha256"]),
                "same_path_five_second_limit": {
                    "config": {
                        "corridor_clearance_weight": 0.0,
                        "corridor_clearance_target_m": TARGET_DEFAULT_M},
                    "max_seconds": 5.0,
                    "status": limited["status"],
                    "certificate_sha256": limited["certificate_sha256"],
                },
                "hash_includes_sealed_limits":
                    limited["certificate_sha256"] != re_certified[
                        "zero_default"]["certificate_sha256"],
                "per_arm_limits": {
                    name: (record["certificate"] or {}).get("limits")
                    for name, record in results.items()},
                "unittest":
                    "SoftClearanceAblationTests."
                    "test_soft_config_never_changes_the_certificate",
            }
    except Exception as exc:  # evidence must show failures, never hide them
        evidence["error"] = "{}: {}".format(type(exc).__name__, exc)
        evidence["traceback"] = traceback.format_exc()
    print(json.dumps(evidence, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()