"""T4 internal body-spin events: fixed q_F and fixed vehicle centres.

A body-spin event expresses a heading reconfiguration that the S4 q_F-only
seed cannot express: every vehicle keeps its centre and the cargo pose stays
fixed while the body heading ``theta_i`` changes.  It is never encoded as a
zero-length translation and it is not an external seed primitive.

The event is internally represented by ``FormationPathControl`` with
``geometry_type=BODY_SPIN`` and ``start_qf == end_qf``.  Each vehicle segment
is either

* S with zero control, zero travel and the sign of its own heading change, or
* A with zero control, zero travel and no heading change (the stationary
  hold of a vehicle that does not need to rotate).

``parameter_length`` is the largest absolute heading change in the joint
event.  The common event duration is therefore driven by the slowest
per-vehicle yaw constraint and is computed later by S8, not here.

This module only builds geometry candidates.  It does not certify collision,
timing or execution.
"""
from dataclasses import dataclass
import itertools
import math
from typing import Mapping, Optional, Tuple

from .formation import FormationModel, FormationPose, FormationValidationError
from .formation_geometry import (
    GEOMETRY_VALID,
    FormationBlock,
    FormationContinuousPlan,
    FormationPathControl,
    _angle_error,
    _wrap,
)
from .formation_kinematics import (
    ACKERMANN,
    BODY_SPIN,
    SPIN,
    KinematicLimits,
    VehicleModeSegment,
)

_GEOMETRY_TOL = 1e-7


@dataclass(frozen=True)
class SpinScheme:
    """One joint heading target set for the pre-rotation spin event."""

    end_headings: Tuple[Tuple[int, float], ...]
    cost: float


def _checked_headings(headings, vehicle_ids, context):
    if not isinstance(headings, Mapping):
        raise FormationValidationError(
            "INVALID_BODY_HEADINGS context={} expected a mapping".format(
                context))
    result = {}
    expected = set(vehicle_ids)
    for vehicle_id, theta in headings.items():
        if vehicle_id not in expected:
            raise FormationValidationError(
                "UNKNOWN_VEHICLE_ID context={} vehicle_id={}".format(
                    context, vehicle_id))
        try:
            value = float(theta)
        except (TypeError, ValueError) as exc:
            raise FormationValidationError(
                "NONFINITE_VALUE context={} vehicle_id={}".format(
                    context, vehicle_id)) from exc
        if not math.isfinite(value):
            raise FormationValidationError(
                "NONFINITE_VALUE context={} vehicle_id={}".format(
                    context, vehicle_id))
        result[int(vehicle_id)] = value
    missing = sorted(expected - set(result))
    if missing:
        raise FormationValidationError(
            "MISSING_VEHICLE_HEADING context={} missing={}".format(
                context, missing))
    return result


def make_spin_block(qf, start_headings, target_headings, offsets,
                    block_index=0):
    """Build one joint body-spin block, or ``None`` if nothing rotates.

    ``qf`` and every vehicle centre are fixed.  A vehicle whose target equals
    its start heading is represented by a zero-control A hold; a vehicle that
    rotates is represented by S with the correct direction sign.  The block
    length is the largest absolute heading change, so every segment advances
    at most one radian per unit lambda.
    """
    if not isinstance(qf, FormationPose):
        raise FormationValidationError("INVALID_FORMATION_POSE qf")
    offsets = tuple(sorted(offsets, key=lambda item: item.vehicle_id))
    model = FormationModel(offsets)
    vehicle_ids = model.vehicle_ids
    headings = _checked_headings(
        start_headings, vehicle_ids, "spin.start_headings")
    targets = _checked_headings(
        target_headings, vehicle_ids, "spin.target_headings")
    deltas = {
        vehicle_id: _wrap(targets[vehicle_id] - headings[vehicle_id])
        for vehicle_id in vehicle_ids}
    length = max(abs(value) for value in deltas.values())
    if length <= _GEOMETRY_TOL:
        return None
    centers = model.centers(qf)
    segments = []
    for vehicle_id in vehicle_ids:
        delta = deltas[vehicle_id]
        start_heading = headings[vehicle_id]
        if abs(delta) <= _GEOMETRY_TOL:
            end_heading = start_heading
            mode = ACKERMANN
            direction = 1
        else:
            end_heading = start_heading + delta
            mode = SPIN
            direction = 1 if delta > 0.0 else -1
        center = centers[vehicle_id]
        segments.append(VehicleModeSegment(
            vehicle_id=vehicle_id,
            start_station=0.0,
            end_station=length,
            mode=mode,
            direction=direction,
            theta_start=start_heading,
            theta_end=end_heading,
            control=0.0,
            travel_distance=0.0,
            start_center_x=center[0],
            start_center_y=center[1],
            end_center_x=center[0],
            end_center_y=center[1],
            pivot_x=0.0,
            pivot_y=0.0))
    control = FormationPathControl(
        BODY_SPIN, length, qf, qf, 0.0, 0.0)
    block = FormationBlock(
        geometry_type=BODY_SPIN,
        parameter_length=length,
        formation_controls=(control,),
        vehicle_mode_segments=tuple(segments),
        block_index=block_index)
    end_headings = tuple(sorted(
        (segment.vehicle_id, segment.theta_end) for segment in segments))
    return block, end_headings, length


def heading_only_targets(snapshot):
    """Return per-vehicle targets for a cargo-stationary heading request.

    Explicit (non-free) goal headings are target headings.  A free goal
    heading stays at the current heading, so a request that changes only
    free placeholders is still a zero-motion hold.
    """
    vehicle_ids = tuple(snapshot.model.vehicle_ids)
    start = _checked_headings(
        snapshot.start_headings, vehicle_ids, "snapshot.start_headings")
    goal = _checked_headings(
        snapshot.goal_headings, vehicle_ids, "snapshot.goal_headings")
    free = dict(snapshot.goal_heading_free)
    targets = {
        vehicle_id: (start[vehicle_id] if free.get(vehicle_id, False)
                     else goal[vehicle_id])
        for vehicle_id in vehicle_ids}
    changes = {
        vehicle_id: _wrap(targets[vehicle_id] - start[vehicle_id])
        for vehicle_id in vehicle_ids}
    return targets, changes


def heading_reconfiguration_needed(snapshot):
    """True when an explicit goal heading requires a stationary body spin."""
    targets, changes = heading_only_targets(snapshot)
    del targets
    return any(abs(value) > _GEOMETRY_TOL for value in changes.values())


def build_heading_only_plan(snapshot, source_signature=()):
    """Build the single plan for ``q_F`` fixed and explicit heading targets."""
    vehicle_ids = tuple(snapshot.model.vehicle_ids)
    start_headings = _checked_headings(
        snapshot.start_headings, vehicle_ids, "snapshot.start_headings")
    targets, changes = heading_only_targets(snapshot)
    if not any(abs(value) > _GEOMETRY_TOL for value in changes.values()):
        return None
    built = make_spin_block(
        snapshot.start, start_headings, targets, snapshot.model.offsets, 0)
    if built is None:
        return None
    block, _end_headings, _length = built
    return FormationContinuousPlan(
        plan_id=int(snapshot.plan_id),
        formation_id=str(snapshot.formation_id),
        start=snapshot.start,
        goal=snapshot.goal,
        offsets=tuple(snapshot.model.offsets),
        vehicle_start_headings=tuple(sorted(start_headings.items())),
        vehicle_goal_headings=tuple(sorted(
            dict(snapshot.goal_headings).items())),
        blocks=(block,),
        allow_reverse=bool(snapshot.allow_reverse),
        goal_heading_free=tuple(sorted(
            dict(snapshot.goal_heading_free).items())),
        certification=GEOMETRY_VALID,
        dynamic_profile="static_geometry",
        source_signature=tuple(source_signature),
    )


def rotation_schemes(qf, headings, offsets, dpsi, limits=None,
                     goal_headings=None, goal_heading_free=None,
                     max_schemes=16):
    """Return joint tangent/anti-tangent heading schemes before a rotation.

    Every vehicle may be spun to the rotation tangent or to the opposite
    tangent.  When an explicit terminal body heading is present, the target
    before the rotation is forced to ``goal_heading - dpsi`` and is accepted
    only if it is one of those two tangents.  The returned schemes are
    unranked beyond their deterministic Cartesian order; the caller compares
    their A/O direction feasibility with the finite S5 budget.
    """
    if not isinstance(qf, FormationPose):
        raise FormationValidationError("INVALID_FORMATION_POSE qf")
    limits = limits or KinematicLimits()
    if not isinstance(limits, KinematicLimits):
        raise FormationValidationError(
            "INVALID_KINEMATIC_LIMITS expected KinematicLimits")
    offsets = tuple(sorted(offsets, key=lambda item: item.vehicle_id))
    model = FormationModel(offsets)
    vehicle_ids = model.vehicle_ids
    current = _checked_headings(headings, vehicle_ids, "spin.headings")
    goal = None if goal_headings is None else _checked_headings(
        goal_headings, vehicle_ids, "spin.goal_headings")
    free = {vehicle_id: False for vehicle_id in vehicle_ids}
    if goal_heading_free is not None:
        free.update({int(key): bool(value)
                     for key, value in dict(goal_heading_free).items()})
    if isinstance(max_schemes, bool) or not isinstance(max_schemes, int) or (
            max_schemes <= 0):
        raise FormationValidationError("INVALID_MAX_SPIN_SCHEMES")
    if abs(dpsi) <= _GEOMETRY_TOL:
        return ()
    options = []
    for offset in offsets:
        vehicle_id = offset.vehicle_id
        rx = offset.x
        ry = offset.y
        tangent = math.atan2(rx, -ry) + qf.psi
        candidates = (tangent, _wrap(tangent + math.pi))
        if goal is not None and not free.get(vehicle_id, False):
            required = _wrap(goal[vehicle_id] - dpsi)
            candidates = tuple(
                value for value in candidates
                if _angle_error(value, required) <= (
                    limits.heading_tolerance_rad + _GEOMETRY_TOL))
            if not candidates:
                return ()
        options.append((vehicle_id, candidates))
    schemes = []
    for combination in itertools.product(
            *(values for _, values in options)):
        targets = dict(zip((vehicle_id for vehicle_id, _ in options),
                           combination))
        deltas = {
            vehicle_id: _wrap(targets[vehicle_id] - current[vehicle_id])
            for vehicle_id in vehicle_ids}
        if all(abs(value) <= _GEOMETRY_TOL for value in deltas.values()):
            continue
        cost = sum(abs(value) for value in deltas.values()) + 0.5 * max(
            abs(value) for value in deltas.values())
        schemes.append(SpinScheme(
            end_headings=tuple(sorted(targets.items())),
            cost=cost))
        if max_schemes is not None and len(schemes) >= max_schemes:
            break
    schemes.sort(key=lambda item: (
        item.cost, tuple(round(value, 12) for _, value in item.end_headings)))
    return tuple(schemes)