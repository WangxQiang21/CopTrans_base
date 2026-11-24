"""Audited zero-motion hold for a rigid formation.

A zero-motion request has identical start and goal formation poses and a seed
whose primitives are all zero length.  The only successful representation of
such a request is a static hold, never a fabricated non-zero path:

* exactly one common time sample at ``t = 0``;
* every vehicle centre equals ``q_F(0) + R(psi_F(0)) r_i``;
* every body heading equals the requested start heading;
* signed centre speed, yaw rate and wheel speed are zero;
* the mode is A with zero curvature (a motionless vehicle, not a fake Spin);
* the start formation passes an independent static OBB/map check with the S9
  margin and against the exact map signature used by the request.

The hold trajectory is still ``GEOMETRY_ONLY``; only
:func:`evaluate_formation_hold` may return ``FEASIBLE``, and the publication
gate still requires the current plan id.
"""
import math

from .core import Grid
from .formation import FormationModel, FormationPose, FormationValidationError
from .formation_collision import FormationCollisionChecker
from .formation_geometry import validate_zero_motion_seed
from .formation_kinematics import ACKERMANN
from .formation_quality import (
    FEASIBLE,
    FormationEvaluation,
    FormationEvaluationFailure,
    FormationQualityConfig,
    _DYNAMIC_TOL,
    _TIME_TOL,
    _angle_error,
    _status_for_code,
    grid_signature,
)
from .formation_retiming import (
    GEOMETRY_ONLY,
    FormationTimedPoint,
    FormationTimedTrajectory,
    FormationTimingReport,
    _DYNAMIC_PROFILE,
)

NO_MOTION = "NO_MOTION"
HOLD_CONTRACT = (
    "one common t=0 sample; fixed centres and body headings; zero signed "
    "speed, yaw rate and wheel speed; A/control=0; static OBB/map certificate")
_MAX_RIGIDITY_TOL_M = 1e-9


def build_hold_timed_trajectory(snapshot, dt_s=0.02):
    """Build the static hold trajectory for a validated request snapshot."""
    if not isinstance(dt_s, (int, float)) or isinstance(dt_s, bool) or not (
            math.isfinite(float(dt_s)) and float(dt_s) > 0.0):
        raise FormationValidationError("INVALID_HOLD_DT")
    try:
        model = snapshot.model
        if not isinstance(model, FormationModel):
            raise FormationValidationError("INVALID_REQUEST_MODEL")
        start = snapshot.start
        if not isinstance(start, FormationPose):
            raise FormationValidationError("INVALID_REQUEST_START")
        vehicle_ids = tuple(snapshot.vehicle_ids)
        headings = dict(snapshot.start_headings)
        if set(vehicle_ids) != set(headings):
            raise FormationValidationError("HOLD_HEADING_SET_MISMATCH")
    except (AttributeError, TypeError, ValueError) as exc:
        raise FormationValidationError(
            "INVALID_HOLD_SNAPSHOT {!r}".format(exc)) from exc
    centers = model.centers(start)
    points = {}
    for vehicle_id in vehicle_ids:
        points[vehicle_id] = (
            FormationTimedPoint(
                time_from_start=0.0,
                center_x=float(centers[vehicle_id][0]),
                center_y=float(centers[vehicle_id][1]),
                theta=float(headings[vehicle_id]),
                center_speed_mps=0.0,
                center_yaw_rate_radps=0.0,
                mode=ACKERMANN,
                mode_control=0.0,
                block_index=0,
                station=0.0,
                stationary=True,
                center_acceleration_mps2=0.0,
                center_jerk_mps3=0.0,
                wheel_speed_mps=0.0,
                lateral_acceleration_mps2=0.0),
        )
    report = FormationTimingReport(
        dynamic_profile=_DYNAMIC_PROFILE,
        dt=float(dt_s),
        duration_s=0.0,
        moving_seconds=0.0,
        stationary_seconds=0.0,
        block_durations=(),
        phases=(),
        stationary_events=(
            {"event": NO_MOTION, "duration_s": 0.0,
             "vehicle_ids": list(vehicle_ids)},),
        direction_changes=(),
        vehicle_peaks={
            vehicle_id: {
                "speed": 0.0, "acceleration": 0.0, "jerk": 0.0,
                "yaw_rate": 0.0, "yaw_acceleration": 0.0,
                "yaw_jerk": 0.0, "wheel_speed": 0.0,
                "lateral_acceleration": 0.0,
            } for vehicle_id in vehicle_ids},
        active_constraints=(),
        mode_intervals={vehicle_id: () for vehicle_id in vehicle_ids},
        limits={"hold_contract": HOLD_CONTRACT},
        certification=GEOMETRY_ONLY)
    return FormationTimedTrajectory(
        plan_id=int(snapshot.plan_id),
        formation_id=str(snapshot.formation_id),
        offsets=tuple(model.offsets),
        formation_poses=(FormationPose(start.x, start.y, start.psi),),
        times_from_start=(0.0,),
        trajectories=points,
        phases=(),
        timing_report=report)


def hold_failure(plan_id, formation_id, code, detail, checks=0,
                 metrics=None, **fields):
    """Build the non-feasible S9 evaluation used by the pipeline."""
    return FormationEvaluation(
        plan_id=int(plan_id),
        formation_id=str(formation_id),
        status=_status_for_code(code),
        feasible=False,
        reason=code,
        detail=detail,
        failure=FormationEvaluationFailure(code=code, detail=detail, **fields),
        metrics=dict(metrics or {"no_motion": True, "duration_s": 0.0}),
        checks=int(checks))


def evaluate_formation_hold(snapshot, timed, grid, config=None):
    """Independently audit a zero-motion hold; only ``FEASIBLE`` is a pass."""
    config = config or FormationQualityConfig()
    if not isinstance(config, FormationQualityConfig):
        raise FormationValidationError("INVALID_FORMATION_QUALITY_CONFIG")
    plan_id = getattr(snapshot, "plan_id", -1)
    formation_id = getattr(snapshot, "formation_id", "")
    checks = 0
    metrics = {"no_motion": True, "duration_s": 0.0,
               "moving_seconds": 0.0, "wheel_speed_max": 0.0,
               "max_rigidity_error_m": 0.0, "hold_contract": HOLD_CONTRACT}

    def fail(code, detail, **fields):
        return hold_failure(
            plan_id, formation_id, code, detail, checks=checks,
            metrics=metrics, **fields)

    if not isinstance(timed, FormationTimedTrajectory):
        return fail("INVALID_TIMED_TRAJECTORY",
                    "zero-motion hold must be a FormationTimedTrajectory",
                    field="trajectory")
    if (getattr(timed, "plan_id", None) != plan_id or
            getattr(timed, "formation_id", None) != formation_id):
        return fail("INVALID_REQUEST",
                    "hold identity differs from the active request",
                    field="plan_id")
    try:
        model = snapshot.model
        if not isinstance(model, FormationModel):
            return fail("INVALID_REQUEST", "request model is not a "
                        "FormationModel", field="vehicles")
        start = snapshot.start
        if not isinstance(start, FormationPose):
            return fail("INVALID_REQUEST", "request start is not a "
                        "FormationPose", field="formation_start")
        vehicle_ids = tuple(snapshot.vehicle_ids)
        start_headings = dict(snapshot.start_headings)
        shapes = dict(snapshot.vehicle_shapes)
    except (AttributeError, TypeError, ValueError) as exc:
        return fail("INVALID_REQUEST", "request fields are malformed: {!r}"
                    .format(exc), field="request")
    checks += 1
    if tuple(timed.offsets) != tuple(model.offsets):
        return fail("INVALID_TIMED_TRAJECTORY",
                    "hold offsets differ from the request model",
                    field="offsets")
    checks += 1
    times = tuple(float(value) for value in timed.times_from_start)
    if len(times) != 1 or abs(times[0]) > _TIME_TOL:
        return fail("INVALID_TIMED_TRAJECTORY",
                    "a hold must have exactly one common sample at t=0",
                    field="times_from_start")
    checks += 1
    if len(timed.formation_poses) != 1:
        return fail("INVALID_TIMED_TRAJECTORY",
                    "a hold must have exactly one formation pose",
                    field="formation_poses")
    pose = timed.formation_poses[0]
    pose_xy = math.hypot(pose.x - start.x, pose.y - start.y)
    pose_yaw = _angle_error(pose.psi, start.psi)
    if (pose_xy > config.position_tolerance_m or
            pose_yaw > config.heading_tolerance_rad):
        return fail("FORMATION_RIGIDITY_VIOLATION",
                    "hold formation pose differs from the request start "
                    "by {:.12g} m / {:.12g} rad".format(pose_xy, pose_yaw),
                    field="formation_poses")
    checks += 1
    expected_centers = model.centers(FormationPose(pose.x, pose.y, pose.psi))
    if set(timed.trajectories) != set(vehicle_ids):
        return fail("INVALID_TIMED_TRAJECTORY",
                    "hold vehicle ids differ from the request",
                    field="trajectories")
    checks += 1
    for vehicle_id in vehicle_ids:
        points = tuple(timed.trajectories[vehicle_id])
        if len(points) != 1:
            return fail("INVALID_TIMED_TRAJECTORY",
                        "vehicle {} hold has {} samples, expected 1".format(
                            vehicle_id, len(points)),
                        field="trajectories", vehicle_id=vehicle_id)
        point = points[0]
        if abs(point.time_from_start) > _TIME_TOL:
            return fail("INVALID_TIMED_TRAJECTORY",
                        "vehicle {} hold sample is not at t=0".format(
                            vehicle_id),
                        field="time_from_start", vehicle_id=vehicle_id)
        if (point.mode != ACKERMANN or
                abs(point.mode_control) > config.heading_tolerance_rad):
            return fail("KINEMATIC_INFEASIBLE",
                        "vehicle {} hold must be A with zero curvature, "
                        "got mode={} control={:.12g}".format(
                            vehicle_id, point.mode, point.mode_control),
                        field="mode", vehicle_id=vehicle_id)
        if (abs(point.center_speed_mps) > _DYNAMIC_TOL or
                abs(point.center_yaw_rate_radps) > _DYNAMIC_TOL or
                abs(point.wheel_speed_mps) > _DYNAMIC_TOL):
            return fail("KINEMATIC_INFEASIBLE",
                        "vehicle {} hold is not stationary: speed={:.12g} "
                        "yaw_rate={:.12g} wheel_speed={:.12g}".format(
                            vehicle_id, point.center_speed_mps,
                            point.center_yaw_rate_radps,
                            point.wheel_speed_mps),
                        field="center_speed_mps", vehicle_id=vehicle_id)
        if not bool(point.stationary):
            return fail("KINEMATIC_INFEASIBLE",
                        "vehicle {} hold point is not marked stationary"
                        .format(vehicle_id),
                        field="stationary", vehicle_id=vehicle_id)
        expected = expected_centers[vehicle_id]
        center_error = math.hypot(
            point.center_x - expected[0], point.center_y - expected[1])
        heading_error = _angle_error(
            point.theta, start_headings[vehicle_id])
        if center_error > config.position_tolerance_m:
            return fail("FORMATION_RIGIDITY_VIOLATION",
                        "vehicle {} hold centre differs from "
                        "q_F + R(psi_F) r_i by {:.12g} m".format(
                            vehicle_id, center_error),
                        field="center", vehicle_id=vehicle_id)
        if heading_error > config.heading_tolerance_rad:
            return fail("FORMATION_RIGIDITY_VIOLATION",
                        "vehicle {} hold heading differs from the start "
                        "heading by {:.12g} rad".format(
                            vehicle_id, heading_error),
                        field="theta", vehicle_id=vehicle_id)
        metrics["max_rigidity_error_m"] = max(
            metrics["max_rigidity_error_m"], center_error)
        checks += 1
    if not isinstance(grid, Grid):
        return fail("INVALID_REQUEST",
                    "an exact occupancy map is required for a hold audit",
                    field="map")
    try:
        signature = grid_signature(grid)
    except FormationValidationError as exc:
        return fail("INVALID_REQUEST", "invalid map: {}".format(exc),
                    field="map")
    if (config.expected_map_signature is not None and
            signature != config.expected_map_signature):
        return fail("INVALID_REQUEST",
                    "grid signature differs from the request map",
                    field="map_signature")
    checks += 1
    try:
        checker = FormationCollisionChecker(
            grid, model, shapes, config.collision_margin_m)
        report = checker.static_check(start, start_headings)
    except FormationValidationError as exc:
        return fail("INVALID_REQUEST",
                    "hold collision check failed to run: {}".format(exc),
                    field="collision")
    checks += 1
    if not report.success:
        return fail(
            report.reason or "START_FORMATION_COLLISION",
            "static hold is not clear: {}".format(report.detail or ""),
            field="collision", vehicle_id=report.vehicle_id,
            other_vehicle_id=report.other_vehicle_id)
    checks += 1
    return FormationEvaluation(
        plan_id=plan_id,
        formation_id=formation_id,
        status=FEASIBLE,
        feasible=True,
        reason=NO_MOTION,
        detail=("zero-motion request: audited static hold at the start pose; "
                "no non-zero path was fabricated"),
        failure=None,
        metrics=dict(metrics),
        checks=checks)


def zero_motion_seed(snapshot, seed, expected_map_signature=None):
    """Return the S6 seed report for a zero-motion request, else ``None``."""
    return validate_zero_motion_seed(
        snapshot, seed, expected_map_signature=expected_map_signature)