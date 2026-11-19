"""ROS-free G5 hook surface for the ``virtual_rigid_a`` backend.

The planner node owns ROS admission, threading and publication.  This module
owns only the backend-specific pure logic:

* turning one normalised ``FormationRequestSnapshot`` and the planner's
  confirmed initial context into a G4 ``VirtualAMapRequest``;
* solving that request with the autonomous G4 pipeline;
* deriving one continuous A-only sample table on integer-nanosecond times;
* verifying the resulting atomic playback envelope against the authoritative
  timed model, the original request and the admitted planning context.

No ROS type is imported here.  Publication times are integer nanoseconds so the
planner can construct ``rospy.Duration`` values exactly; every sample is
evaluated at the actual quantised time (``secs + nsecs * 1e-9``), not at an
unquantised duration with only the header restamped.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass, field
import json
import math

from ..formation import FormationPose
from ..formation_request import FormationRequestSnapshot
from ..formation_retiming import FormationTimingLimits
from .contracts import (
    BACKEND,
    G4_PLANNING_SCHEMA,
    INITIAL_CURVATURE_TOLERANCE,
    INITIAL_HEADING_TOLERANCE_RAD,
    PLAYBACK_CERTIFICATION,
    PLAYBACK_SEMANTIC_VERSION,
    ROS_INITIAL_SOURCES,
    TIMED_MODEL_VERSION,
    CollisionConfig,
    GoalPolicy,
    InitialAssumption,
    PlannerConfig,
    TimingConfig,
    VirtualAContractError,
    VirtualAMapRequest,
    VirtualARequest,
    coerce_goal_policy,
    grid_summary_sha256,
    planning_context_sha256,
    request_fingerprint,
    request_from_payload,
    validate_collision_config,
    validate_planner_config,
    validate_timing_config,
    validate_virtual_a_request,
)
from .pipeline import plan_virtual_a
from .trajectory import VirtualATimedPlan


# Stable rejection reasons used by the planner's admission path.  The first
# two names are part of the G5 task; the others make the remaining admission
# rules explicit instead of raising an anonymous exception.
INITIAL_RESET_NOT_CONFIRMED = "INITIAL_RESET_NOT_CONFIRMED"
UNSUPPORTED_INITIAL_STATE = "UNSUPPORTED_INITIAL_STATE"
NONPARALLEL_INITIAL_HEADINGS = "NONPARALLEL_INITIAL_HEADINGS"
UNSUPPORTED_GOAL_HEADING = "UNSUPPORTED_GOAL_HEADING"
MAP_UNAVAILABLE = "MAP_UNAVAILABLE"
INVALID_SNAPSHOT = "INVALID_SNAPSHOT"
INVALID_SETTINGS = "INVALID_SETTINGS"
INVALID_INITIAL_CONTEXT = "INVALID_INITIAL_CONTEXT"
REQUEST_CONTEXT_MISMATCH = "REQUEST_CONTEXT_MISMATCH"
SOLVE_RESULT_MISMATCH = "SOLVE_RESULT_MISMATCH"
PUBLICATION_REQUEST_MISMATCH = "PUBLICATION_REQUEST_MISMATCH"
INVALID_PUBLICATION_STAMP = "INVALID_PUBLICATION_STAMP"
INVALID_PUBLICATION_DT = "INVALID_PUBLICATION_DT"
PUBLICATION_SAMPLE_BUDGET = "PUBLICATION_SAMPLE_BUDGET"
INVALID_ENVELOPE_FIELDS = "INVALID_ENVELOPE_FIELDS"

# F pose/terminal match tolerances.  Reset confirmation is a plane pose, not a
# physical measurement; the request heading check is the contract tolerance.
RESET_POSE_POSITION_TOL_M = 1e-6
RESET_POSE_HEADING_TOL_RAD = 1e-6
TERMINAL_POSE_POSITION_TOL_M = 1e-6
TERMINAL_POSE_HEADING_TOL_RAD = 1e-6
TERMINAL_HEADING_MATCH_TOL_RAD = 1e-9
SNAPSHOT_MATCH_TOL = 1e-12
GOAL_HEADING_COMPATIBILITY_RAD = 1e-9
TERMINAL_STOP_SPEED_TOL = 1e-9
TERMINAL_STOP_RATE_TOL = 1e-9
TERMINAL_STRAIGHT_TOL = INITIAL_CURVATURE_TOLERANCE
ENDPOINT_POSITION_TOL_M = 5e-3
ENDPOINT_HEADING_TOL_RAD = math.radians(0.2)

ENVELOPE_KEYS = frozenset((
    "plan_id", "formation_id", "backend", "semantic_version",
    "model_version", "model_sha256", "model_json", "certification",
    "trajectory_fields",
))
EXPECTED_ENVELOPE_KEYS = frozenset((
    "expected_request", "expected_context", "expected_grid_sha256",
    "expected_admission", "geometry_audit_config", "dynamic_audit_config",
    "position_tolerance", "angle_tolerance", "speed_tolerance",
))


def _finite(value, context):
    if isinstance(value, bool) or isinstance(value, (str, bytes)):
        raise VirtualAContractError(
            "INVALID_VALUE",
            "{} must be a finite number, got {!r}".format(context, value))
    try:
        number = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "INVALID_VALUE",
            "{} must be a finite number, got {!r}".format(
                context, value)) from exc
    if not math.isfinite(number):
        raise VirtualAContractError(
            "INVALID_VALUE",
            "{} must be finite, got {!r}".format(context, value))
    return number


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _context_id_text(value):
    if not isinstance(value, str) or not value.strip():
        raise VirtualAContractError(
            INVALID_INITIAL_CONTEXT,
            "context_id must be a non-empty string")
    return value.strip()


def _strict_json(value, context):
    try:
        return json.loads(json.dumps(
            value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
            allow_nan=False))
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_VALUE",
            "{} is not strict JSON: {}".format(context, exc)) from exc


def _json_safe(value):
    """Return a strict-JSON copy; non-finite numbers become text."""
    if isinstance(value, MappingABC):
        return {str(key): _json_safe(item)
                for key, item in sorted(value.items(), key=lambda pair: str(pair[0]))}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float):
        if math.isfinite(value):
            return value
        if math.isnan(value):
            return "NaN"
        return "+Infinity" if value > 0.0 else "-Infinity"
    if isinstance(value, (str, int, bool)) or value is None:
        return value
    return str(value)


@dataclass(frozen=True)
class VirtualAInitialContext:
    """Planner-owned confirmation of the experiment's initial state.

    ``rviz_reset`` stores the F pose from ``/initialpose`` only; it is a
    command/observation record, not a physical measurement.  A
    ``verified_terminal`` record is built from the previous published timed
    model's terminal state and additionally carries each vehicle's achieved
    heading.
    """

    source: str
    context_id: str
    qf: FormationPose
    headings: MappingABC = field(default_factory=dict)
    compatible: bool = True
    detail: str = ""
    terminal: MappingABC = field(default_factory=dict)

    def __post_init__(self):
        source = str(self.source)
        if source not in ROS_INITIAL_SOURCES:
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT,
                "source={!r} is not a ROS initial source {}".format(
                    source, list(ROS_INITIAL_SOURCES)))
        object.__setattr__(self, "source", source)
        object.__setattr__(self, "context_id", _context_id_text(
            self.context_id))
        if not isinstance(self.qf, FormationPose):
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT, "qf must be FormationPose")
        if not isinstance(self.headings, MappingABC):
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT,
                "headings must be a vehicle_id -> heading mapping")
        headings = {}
        for vehicle_id, heading in self.headings.items():
            if isinstance(vehicle_id, bool) or not isinstance(
                    vehicle_id, int):
                raise VirtualAContractError(
                    INVALID_INITIAL_CONTEXT,
                    "heading key={!r} is not an integer vehicle id".format(
                        vehicle_id))
            headings[int(vehicle_id)] = _finite(
                heading, "headings[{}]".format(vehicle_id))
        if source == "verified_terminal" and not headings:
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT,
                "a verified terminal requires every vehicle heading")
        if not isinstance(self.compatible, bool):
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT, "compatible must be bool")
        if not isinstance(self.detail, str):
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT, "detail must be a string")
        if not isinstance(self.terminal, MappingABC):
            raise VirtualAContractError(
                INVALID_INITIAL_CONTEXT, "terminal must be a mapping")
        object.__setattr__(self, "headings", dict(sorted(headings.items())))
        object.__setattr__(self, "terminal", dict(sorted(
            _strict_json(dict(self.terminal), "initial context terminal")
            .items())))

    def as_dict(self):
        return {
            "source": self.source,
            "context_id": self.context_id,
            "qf": {"x": self.qf.x, "y": self.qf.y, "psi": self.qf.psi},
            "headings": [
                {"vehicle_id": vehicle_id,
                 "heading_rad": self.headings[vehicle_id]}
                for vehicle_id in sorted(self.headings)],
            "compatible": self.compatible,
            "detail": self.detail,
            "terminal": dict(self.terminal),
        }


@dataclass(frozen=True)
class VirtualABackendSettings:
    """Backend settings frozen into one admitted planning context."""

    planner_config: PlannerConfig = field(default_factory=PlannerConfig)
    timing_config: TimingConfig = field(default_factory=TimingConfig)
    collision_config: CollisionConfig = field(default_factory=CollisionConfig)
    goal_policy: str = "free"
    frame_id: str = "map"
    max_publication_samples: int = 200001

    def __post_init__(self):
        for validator, config, name in (
                (validate_planner_config, self.planner_config,
                 "planner_config"),
                (validate_timing_config, self.timing_config, "timing_config"),
                (validate_collision_config, self.collision_config,
                 "collision_config")):
            validation = validator(config)
            if not validation.passed:
                raise VirtualAContractError(
                    INVALID_SETTINGS,
                    "{}: {}: {}".format(
                        name, validation.reason, validation.detail),
                    **validation.location)
        policy = self.goal_policy
        if not isinstance(policy, str):
            raise VirtualAContractError(
                INVALID_SETTINGS, "goal_policy must be a string")
        policy = policy.strip().lower()
        if policy not in ("free", "straight"):
            raise VirtualAContractError(
                INVALID_SETTINGS,
                "goal_policy={!r} must be 'free' or 'straight'".format(
                    self.goal_policy))
        object.__setattr__(self, "goal_policy", policy)
        if self.frame_id != "map":
            raise VirtualAContractError(
                INVALID_SETTINGS,
                "frame_id={!r}; the research transport contract is 'map'"
                .format(self.frame_id))
        if isinstance(self.max_publication_samples, bool) or not isinstance(
                self.max_publication_samples, int) or \
                self.max_publication_samples < 1:
            raise VirtualAContractError(
                INVALID_SETTINGS,
                "max_publication_samples must be a positive integer")

    @property
    def policy(self):
        return coerce_goal_policy(self.goal_policy.upper())


@dataclass(frozen=True)
class VerifiedPlayback:
    """A verified virtual envelope plus its authoritative plan identity."""

    trajectory: object
    plan: VirtualATimedPlan
    model_sha256: str
    backend: str
    semantic_version: str
    certification: str


def _choose_goal_policy(snapshot, settings):
    """Apply the launch-selected goal policy without editing the request."""
    if not isinstance(snapshot.goal_heading_free, MappingABC):
        raise VirtualAContractError(
            INVALID_SNAPSHOT, "goal_heading_free must be a mapping")
    free = {int(vehicle_id): bool(value) for vehicle_id, value in
            snapshot.goal_heading_free.items()}
    if set(free) != set(snapshot.vehicle_ids):
        raise VirtualAContractError(
            INVALID_SNAPSHOT,
            "goal_heading_free ids={} differ from vehicle ids={}".format(
                sorted(free), list(snapshot.vehicle_ids)))
    if not isinstance(snapshot.goal_headings, MappingABC):
        raise VirtualAContractError(
            INVALID_SNAPSHOT, "goal_headings must be a mapping")
    headings = {int(vehicle_id): _finite(
        value, "goal_headings[{}]".format(vehicle_id))
        for vehicle_id, value in snapshot.goal_headings.items()}
    if set(headings) != set(snapshot.vehicle_ids):
        raise VirtualAContractError(
            INVALID_SNAPSHOT,
            "goal_headings ids={} differ from vehicle ids={}".format(
                sorted(headings), list(snapshot.vehicle_ids)))
    policy = settings.policy
    if policy == GoalPolicy.FREE:
        if not all(free.values()):
            raise VirtualAContractError(
                UNSUPPORTED_GOAL_HEADING,
                "virtual_a_goal_policy=free requires goal_heading_free=true "
                "for every vehicle; got {}".format(free))
        return policy
    # STRAIGHT: the request must carry explicit, all-fixed headings that are
    # exactly compatible with a straight terminal formation (kT=etaT=0).
    if any(free.values()):
        raise VirtualAContractError(
            UNSUPPORTED_GOAL_HEADING,
            "virtual_a_goal_policy=straight requires explicit fixed headings "
            "for every vehicle; goal_heading_free={}".format(free))
    goal_psi = snapshot.goal.psi
    for vehicle_id in snapshot.vehicle_ids:
        error = abs(_wrap(headings[vehicle_id] - goal_psi))
        if error > GOAL_HEADING_COMPATIBILITY_RAD:
            raise VirtualAContractError(
                UNSUPPORTED_GOAL_HEADING,
                "vehicle_id={} fixed goal heading differs from goal psi by "
                "{:.6g} rad > {:.6g}; a straight terminal needs kT=etaT=0"
                .format(vehicle_id, error, GOAL_HEADING_COMPATIBILITY_RAD),
                vehicle_id=vehicle_id)
    return policy


def _validated_map_request(snapshot, grid, settings):
    if not isinstance(snapshot, FormationRequestSnapshot):
        raise VirtualAContractError(
            INVALID_SNAPSHOT, "expected FormationRequestSnapshot")
    if not isinstance(settings, VirtualABackendSettings):
        raise VirtualAContractError(
            INVALID_SETTINGS, "expected VirtualABackendSettings")
    if grid is None:
        raise VirtualAContractError(
            MAP_UNAVAILABLE, "the planner has not received an OccupancyGrid")
    return snapshot


def request_from_snapshot(snapshot, grid, settings, initial_context):
    """Adapt one RViz request to the G4 request using confirmed initial state.

    The snapshot is used exactly as received: real per-vehicle start headings,
    requested goal headings, shape flags and policy flags are never rewritten.
    Any mismatch with the planner's confirmed initial context is a structured
    rejection, not a silent correction.
    """
    _validated_map_request(snapshot, grid, settings)
    if initial_context is None:
        raise VirtualAContractError(
            INITIAL_RESET_NOT_CONFIRMED,
            "no /initialpose reset has been confirmed for this experiment")
    if not isinstance(initial_context, VirtualAInitialContext):
        raise VirtualAContractError(
            INVALID_INITIAL_CONTEXT,
            "expected VirtualAInitialContext")
    if not initial_context.compatible:
        raise VirtualAContractError(
            UNSUPPORTED_INITIAL_STATE,
            initial_context.detail or
            "the recorded initial state is not a supported continuation",
            source=initial_context.source,
            context_id=initial_context.context_id,
            terminal=initial_context.terminal)

    start = snapshot.start
    pose = initial_context.qf
    position_error = math.hypot(start.x - pose.x, start.y - pose.y)
    heading_error = abs(_wrap(start.psi - pose.psi))
    if (position_error > RESET_POSE_POSITION_TOL_M
            or heading_error > RESET_POSE_HEADING_TOL_RAD):
        raise VirtualAContractError(
            UNSUPPORTED_INITIAL_STATE,
            "request start F=({:.9g}, {:.9g}, {:.9g}) differs from the "
            "confirmed {} pose ({:.9g}, {:.9g}, {:.9g}) by {:.3g} m / "
            "{:.3g} rad".format(
                start.x, start.y, start.psi, initial_context.source,
                pose.x, pose.y, pose.psi, position_error, heading_error),
            source=initial_context.source,
            context_id=initial_context.context_id,
            position_error_m=position_error,
            heading_error_rad=heading_error)

    headings = {int(vehicle_id): _finite(
        value, "start_headings[{}]".format(vehicle_id))
        for vehicle_id, value in snapshot.start_headings.items()}
    if set(headings) != set(snapshot.vehicle_ids):
        raise VirtualAContractError(
            INVALID_SNAPSHOT,
            "start_headings ids={} differ from vehicle ids={}".format(
                sorted(headings), list(snapshot.vehicle_ids)))
    if initial_context.source == "rviz_reset":
        # The bridge supplies the real body headings.  A reset pose alone does
        # not authorise a non-parallel start, and the adapter never overwrites
        # the supplied headings with the formation heading.
        for vehicle_id in snapshot.vehicle_ids:
            error = abs(_wrap(headings[vehicle_id] - pose.psi))
            if error > INITIAL_HEADING_TOLERANCE_RAD:
                raise VirtualAContractError(
                    NONPARALLEL_INITIAL_HEADINGS,
                    "vehicle_id={} real start heading differs from the reset "
                    "F heading by {:.6g} rad > {:.6g}".format(
                        vehicle_id, error, INITIAL_HEADING_TOLERANCE_RAD),
                    vehicle_id=vehicle_id,
                    context_id=initial_context.context_id)
    else:
        if set(initial_context.headings) != set(snapshot.vehicle_ids):
            raise VirtualAContractError(
                UNSUPPORTED_INITIAL_STATE,
                "verified terminal headings={} differ from vehicle ids={}"
                .format(sorted(initial_context.headings),
                        list(snapshot.vehicle_ids)),
                source=initial_context.source,
                context_id=initial_context.context_id)
        for vehicle_id in snapshot.vehicle_ids:
            error = abs(_wrap(
                headings[vehicle_id]
                - initial_context.headings[vehicle_id]))
            if error > TERMINAL_HEADING_MATCH_TOL_RAD:
                raise VirtualAContractError(
                    UNSUPPORTED_INITIAL_STATE,
                    "vehicle_id={} request heading differs from the verified "
                    "terminal by {:.6g} rad > {:.6g}; the actual terminal is "
                    "kept and the experiment must be reset".format(
                        vehicle_id, error, TERMINAL_HEADING_MATCH_TOL_RAD),
                    vehicle_id=vehicle_id,
                    source=initial_context.source,
                    context_id=initial_context.context_id,
                    terminal=initial_context.terminal)

    policy = _choose_goal_policy(snapshot, settings)
    initial = InitialAssumption(
        stationary=True, mode="A", wheels_straight=True,
        source=initial_context.source,
        context_id=initial_context.context_id)
    request = VirtualARequest(
        plan_id=snapshot.plan_id,
        formation_id=snapshot.formation_id,
        model=snapshot.model,
        vehicle_shapes=dict(snapshot.vehicle_shapes),
        start_qf=snapshot.start,
        start_headings=dict(headings),
        goal_qf=snapshot.goal,
        initial=initial,
        goal_policy=policy,
        limits=FormationTimingLimits())
    validation = validate_virtual_a_request(request)
    if not validation.passed:
        raise VirtualAContractError(
            validation.reason, validation.detail, **validation.location)
    try:
        return VirtualAMapRequest(
            request=request, grid=grid,
            planner_config=settings.planner_config,
            timing_config=settings.timing_config,
            collision_config=settings.collision_config)
    except VirtualAContractError:
        raise
    except Exception as exc:
        raise VirtualAContractError(
            "INVALID_MAP_REQUEST",
            "cannot build the G4 map request: {}: {}".format(
                type(exc).__name__, exc)) from exc


def terminal_context_from_plan(plan, request, context_id):
    """Build the verified-terminal initial context from a published model.

    Every condition is checked on the authoritative terminal state of the
    plan: all vehicles stopped, each body heading parallel to the terminal
    formation heading, and all three ideal A wheel systems straight.  A
    zero formation curvature C_i=0 is not enough on its own: the ideal wheel
    angle of each wheel is also checked.
    """
    if not isinstance(plan, VirtualATimedPlan):
        raise VirtualAContractError(
            "INVALID_PLAN", "expected VirtualATimedPlan")
    if not isinstance(request, VirtualARequest):
        raise VirtualAContractError(
            "INVALID_REQUEST", "expected VirtualARequest")
    context_id = _context_id_text(context_id)
    state = plan.state_at(plan.duration_s)
    vehicles = []
    failures = []
    for vehicle in state.vehicles:
        heading_error = abs(_wrap(vehicle.theta - state.qf.psi))
        wheel_error = max([abs(value) for value in vehicle.wheel_angles]
                          or [0.0])
        checks = {
            "stopped_speed": abs(vehicle.v_i) <= TERMINAL_STOP_SPEED_TOL,
            "stopped_rate": abs(vehicle.omega_i) <= TERMINAL_STOP_RATE_TOL,
            "heading_parallel":
                heading_error <= INITIAL_HEADING_TOLERANCE_RAD,
            "formation_curvature":
                abs(vehicle.C_i) <= TERMINAL_STRAIGHT_TOL,
            "ideal_wheel_angles": wheel_error <= TERMINAL_STRAIGHT_TOL,
            "wheel_speeds_zero":
                max([abs(value) for value in vehicle.wheel_speeds] or [0.0])
                <= TERMINAL_STOP_SPEED_TOL,
            "steer_rates_zero":
                max([abs(value) for value in vehicle.steer_rates] or [0.0])
                <= TERMINAL_STOP_RATE_TOL,
        }
        record = {
            "vehicle_id": vehicle.vehicle_id,
            "x": vehicle.x, "y": vehicle.y, "theta": vehicle.theta,
            "C_i": vehicle.C_i, "v_i": vehicle.v_i, "omega_i": vehicle.omega_i,
            "heading_error_rad": heading_error,
            "wheel_angles_rad": list(vehicle.wheel_angles),
            "wheel_speeds_mps": list(vehicle.wheel_speeds),
            "steer_rates_radps": list(vehicle.steer_rates),
            "checks": checks,
            "eligible": all(checks.values()),
        }
        vehicles.append(record)
        if not all(checks.values()):
            failures.append(record)
    summary = {
        "t_s": state.t,
        "s_m": state.s,
        "region": state.region,
        "qf": {"x": state.qf.x, "y": state.qf.y, "psi": state.qf.psi},
        "vehicles": vehicles,
        "all_stopped": all(record["checks"]["stopped_speed"] and
                          record["checks"]["stopped_rate"]
                          for record in vehicles),
        "all_parallel": all(record["checks"]["heading_parallel"]
                            for record in vehicles),
        "all_wheels_straight": all(
            record["checks"]["formation_curvature"] and
            record["checks"]["ideal_wheel_angles"]
            for record in vehicles),
    }
    headings = {record["vehicle_id"]: record["theta"]
                for record in vehicles}
    if failures:
        detail = (
            "the previous terminal is not a supported continuation: " + "; "
            .join(
                "vehicle_id={} failed {}".format(
                    record["vehicle_id"],
                    sorted(name for name, passed in record["checks"].items()
                           if not passed))
                for record in failures))
        return VirtualAInitialContext(
            source="verified_terminal", context_id=context_id,
            qf=state.qf, headings=headings, compatible=False,
            detail=detail, terminal=summary)
    return VirtualAInitialContext(
        source="verified_terminal", context_id=context_id,
        qf=state.qf, headings=headings, compatible=True,
        detail="all vehicles stopped, parallel and wheel-straight at T",
        terminal=summary)


def solve(request, progress_callback=None):
    """Run the autonomous G4 pipeline and guard its executable identity.

    ``progress_callback`` is passed through unchanged to the ROS-free
    pipeline; the historical ``solve(request)`` call remains valid.
    """
    if not isinstance(request, VirtualAMapRequest):
        raise VirtualAContractError(
            "INVALID_MAP_REQUEST", "expected VirtualAMapRequest")
    result = plan_virtual_a(
        request, progress_callback=progress_callback)
    if not bool(getattr(result, "executable", False)):
        return result
    plan = getattr(result, "plan", None)
    problems = []
    if not isinstance(plan, VirtualATimedPlan):
        problems.append("executable result carries no VirtualATimedPlan")
    else:
        if plan.request_sha256 != request.request_sha256:
            problems.append("plan request digest differs from the map request")
        if plan.planning_context_sha256() != result.planning_context_sha256:
            problems.append("plan context digest differs from the result")
        if plan.model_sha256() != result.model_sha256:
            problems.append("plan model digest differs from the result")
    context = request.context()
    if result.planning_context_sha256 != planning_context_sha256(context):
        problems.append("result context digest differs from the map request")
    if result.grid_sha256 != request.grid_sha256:
        problems.append("result grid digest differs from the map request")
    if problems:
        raise VirtualAContractError(
            SOLVE_RESULT_MISMATCH, "; ".join(problems),
            plan_id=getattr(request.request, "plan_id", None))
    return result


def _publication_times(plan, dt, settings):
    """Return integer-nanosecond sample times and their exact seconds.

    A motion plan is sampled at the uniform ``dt`` grid plus its endpoint.  The
    endpoint is the smallest integer nanosecond time not before ``T``; a hold
    plan publishes exactly one sample at t=0.  Values are evaluated at the
    actual quantised times by the caller.
    """
    if plan.kind == "hold":
        return (0,), (0.0,)
    duration = plan.duration_s
    target = int(math.ceil(duration * 1e9))
    if target < 1:
        target = 1
    while target > 1 and (target - 1) / 1e9 >= duration:
        target -= 1
    while target / 1e9 < duration:
        target += 1
    step = int(round(_finite(dt, "publication dt") * 1e9))
    if step < 1:
        step = 1
    times = list(range(0, target, step))
    times.append(target)
    if len(times) > settings.max_publication_samples:
        raise VirtualAContractError(
            PUBLICATION_SAMPLE_BUDGET,
            "{} samples exceed the admitted budget {}".format(
                len(times), settings.max_publication_samples))
    seconds = tuple(secs + nsecs * 1e-9 for secs, nsecs in
                    (divmod(value, 1000000000) for value in times))
    return tuple(times), seconds


def _snapshot_matches_request(snapshot, request):
    """Compare the retained raw request with the model's original request."""
    if snapshot.plan_id != request.plan_id or \
            snapshot.formation_id != request.formation_id:
        return "plan/formation identity differs from the model request"
    errors = []
    for actual, expected, label in (
            (snapshot.start.x, request.start_qf.x, "start.x"),
            (snapshot.start.y, request.start_qf.y, "start.y"),
            (snapshot.start.psi, request.start_qf.psi, "start.psi"),
            (snapshot.goal.x, request.goal_qf.x, "goal.x"),
            (snapshot.goal.y, request.goal_qf.y, "goal.y"),
            (snapshot.goal.psi, request.goal_qf.psi, "goal.psi")):
        if abs(float(actual) - float(expected)) > SNAPSHOT_MATCH_TOL:
            errors.append("{}={!r} != {!r}".format(label, actual, expected))
    model_offsets = {offset.vehicle_id: (offset.x, offset.y)
                     for offset in request.model.offsets}
    snapshot_offsets = {offset.vehicle_id: (offset.x, offset.y)
                        for offset in snapshot.model.offsets}
    if set(model_offsets) != set(snapshot_offsets):
        errors.append("model vehicle ids differ")
    else:
        for vehicle_id in sorted(model_offsets):
            actual = snapshot_offsets[vehicle_id]
            expected = model_offsets[vehicle_id]
            if (abs(actual[0] - expected[0]) > SNAPSHOT_MATCH_TOL
                    or abs(actual[1] - expected[1]) > SNAPSHOT_MATCH_TOL):
                errors.append("offset vehicle {} differs".format(vehicle_id))
    if set(snapshot.vehicle_shapes) != set(request.vehicle_shapes):
        errors.append("vehicle shape ids differ")
    else:
        for vehicle_id in sorted(request.vehicle_shapes):
            actual = snapshot.vehicle_shapes[vehicle_id]
            expected = request.vehicle_shapes[vehicle_id]
            if (abs(actual.length_m - expected.length_m) > SNAPSHOT_MATCH_TOL
                    or abs(actual.width_m - expected.width_m)
                    > SNAPSHOT_MATCH_TOL):
                errors.append("shape vehicle {} differs".format(vehicle_id))
    if set(snapshot.start_headings) != set(request.start_headings):
        errors.append("start heading ids differ")
    else:
        for vehicle_id in sorted(request.start_headings):
            error = abs(_wrap(float(snapshot.start_headings[vehicle_id])
                              - float(request.start_headings[vehicle_id])))
            if error > SNAPSHOT_MATCH_TOL:
                errors.append(
                    "start heading vehicle {} differs by {:.3g}".format(
                        vehicle_id, error))
    policy = coerce_goal_policy(request.goal_policy)
    if set(snapshot.goal_heading_free) != set(request.vehicle_ids):
        errors.append("goal heading flag ids differ from the model")
    free = {int(vehicle_id): bool(value)
            for vehicle_id, value in snapshot.goal_heading_free.items()}
    if policy == GoalPolicy.FREE:
        if not all(free.values()):
            errors.append("FREE model request carries a fixed heading flag")
    else:
        if any(free.values()):
            errors.append("STRAIGHT model request carries a free heading flag")
    return "; ".join(errors)


def _validate_stamp(stamp):
    try:
        secs, nsecs = stamp
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            INVALID_PUBLICATION_STAMP,
            "stamp must be (secs, nsecs)") from exc
    if isinstance(secs, bool) or not isinstance(secs, int) or \
            isinstance(nsecs, bool) or not isinstance(nsecs, int):
        raise VirtualAContractError(
            INVALID_PUBLICATION_STAMP, "stamp must contain integers")
    if secs < 0 or secs > 2 ** 32 - 1 or nsecs < 0 or nsecs >= 1000000000:
        raise VirtualAContractError(
            INVALID_PUBLICATION_STAMP,
            "stamp=({}, {}) is outside ROS duration limits".format(secs, nsecs))
    if secs == 0 and nsecs == 0:
        raise VirtualAContractError(
            INVALID_PUBLICATION_STAMP, "stamp must be an explicit common time")
    return int(secs), int(nsecs)


def publication_fields(snapshot, result, stamp, dt, settings=None):
    """Build the envelope trajectory fields on integer-nanosecond times."""
    if settings is None:
        settings = VirtualABackendSettings()
    if not isinstance(settings, VirtualABackendSettings):
        raise VirtualAContractError(
            INVALID_SETTINGS, "expected VirtualABackendSettings")
    stamp = _validate_stamp(stamp)
    if dt is None or not math.isfinite(float(dt)) or float(dt) <= 0.0:
        raise VirtualAContractError(
            INVALID_PUBLICATION_DT, "dt must be a positive finite value")
    if not bool(getattr(result, "executable", False)):
        raise VirtualAContractError(
            "INVALID_RESULT", "only an executable result can be published")
    plan = getattr(result, "plan", None)
    if not isinstance(plan, VirtualATimedPlan):
        raise VirtualAContractError(
            "INVALID_RESULT", "executable result carries no timed plan")
    if result.status == "NO_MOTION" and plan.kind != "hold":
        raise VirtualAContractError(
            "INVALID_RESULT", "NO_MOTION result is not a hold plan")
    if result.status == "SUCCESS" and plan.kind != "motion":
        raise VirtualAContractError(
            "INVALID_RESULT", "SUCCESS result is not a motion plan")
    try:
        model_core = plan.to_model_dict()
        request = request_from_payload(model_core["request_inputs"])
    except VirtualAContractError:
        raise
    except Exception as exc:
        raise VirtualAContractError(
            "INVALID_MODEL_JSON",
            "cannot rebuild the model's original request: {}: {}".format(
                type(exc).__name__, exc)) from exc
    mismatch = _snapshot_matches_request(snapshot, request)
    if mismatch:
        raise VirtualAContractError(
            PUBLICATION_REQUEST_MISMATCH, mismatch)
    times_ns, times_s = _publication_times(plan, dt, settings)
    states = []
    for value in times_s:
        try:
            states.append(plan.state_at(value))
        except Exception as exc:
            raise VirtualAContractError(
                "INVALID_MODEL_JSON",
                "authoritative model cannot evaluate t={:.17g}: {}: {}"
                .format(value, type(exc).__name__, exc)) from exc
    start_centers = request.model.centers(request.start_qf)
    goal_centers = request.model.centers(request.goal_qf)
    vehicle_ids = tuple(request.model.vehicle_ids)
    vehicles = []
    for vehicle_id in vehicle_ids:
        shape = request.vehicle_shapes[vehicle_id]
        vehicles.append({
            "vehicle_id": vehicle_id,
            "offset": (float(next(
                offset.x for offset in request.model.offsets
                if offset.vehicle_id == vehicle_id)),
                float(next(
                    offset.y for offset in request.model.offsets
                    if offset.vehicle_id == vehicle_id))),
            "body_length_m": float(shape.length_m),
            "body_width_m": float(shape.width_m),
            "start_center": (
                float(start_centers[vehicle_id][0]),
                float(start_centers[vehicle_id][1]),
                float(request.start_headings[vehicle_id])),
            "goal_center": (
                float(goal_centers[vehicle_id][0]),
                float(goal_centers[vehicle_id][1]),
                float(request.goal_qf.psi)),
            "goal_heading_free": bool(snapshot.goal_heading_free[vehicle_id]),
        })
    trajectories = []
    for vehicle_id in vehicle_ids:
        points = []
        for time_ns, time_s, state in zip(times_ns, times_s, states):
            vehicle = state.vehicle(vehicle_id)
            points.append((
                float(time_s), float(vehicle.x), float(vehicle.y),
                float(vehicle.theta), float(vehicle.v_i),
                float(vehicle.omega_i), 0, float(vehicle.C_i)))
        trajectories.append({
            "vehicle_id": vehicle_id,
            "points": tuple(points),
            "point_times_ns": tuple(times_ns),
        })
    return {
        "frame_id": settings.frame_id,
        "stamp": stamp,
        "plan_id": int(request.plan_id),
        "formation_id": request.formation_id,
        "vehicles": tuple(vehicles),
        "formation_poses": tuple(
            (float(state.qf.x), float(state.qf.y), float(state.qf.psi))
            for state in states),
        "times_from_start": tuple(float(value) for value in times_s),
        "times_ns": tuple(int(value) for value in times_ns),
        "trajectories": tuple(trajectories),
    }


def _coerce_expected_request(expected_request):
    if expected_request is None:
        raise VirtualAContractError(
            "PLAYBACK_REQUEST_MISSING",
            "no normalized raw /formation/plan_request is retained for this "
            "plan; the envelope cannot be accepted")
    if isinstance(expected_request, VirtualARequest):
        request = expected_request
    elif isinstance(expected_request, MappingABC):
        try:
            request = request_from_payload(expected_request)
        except VirtualAContractError:
            raise
        except Exception as exc:
            raise VirtualAContractError(
                "INVALID_REQUEST",
                "expected_request payload is invalid: {}: {}".format(
                    type(exc).__name__, exc)) from exc
    else:
        raise VirtualAContractError(
            "INVALID_REQUEST",
            "expected_request must be a VirtualARequest or its payload")
    validation = validate_virtual_a_request(request)
    if not validation.passed:
        raise VirtualAContractError(
            validation.reason, validation.detail, **validation.location)
    return request


def _verify_admission(admission, request, context_sha, grid_sha):
    if not isinstance(admission, MappingABC):
        raise VirtualAContractError(
            "PLAYBACK_ADMISSION_MISSING",
            "expected_admission must be a mapping")
    problems = []
    request_sha = request_fingerprint(request)
    if admission.get("plan_id") != request.plan_id:
        problems.append("admission plan_id differs")
    if admission.get("formation_id") != request.formation_id:
        problems.append("admission formation_id differs")
    if admission.get("request_sha256") != request_sha:
        problems.append("admission request_sha256 differs")
    if admission.get("planning_context_sha256") != context_sha:
        problems.append("admission planning_context_sha256 differs")
    if admission.get("grid_sha256") != grid_sha:
        problems.append("admission grid_sha256 differs")
    if admission.get("initial_source") != request.initial.source:
        problems.append("admission initial_source differs from the request")
    if admission.get("initial_context_id") != request.initial.context_id:
        problems.append("admission initial_context_id differs from the request")
    if admission.get("goal_policy") != coerce_goal_policy(
            request.goal_policy).value:
        problems.append("admission goal_policy differs from the request")
    if problems:
        raise VirtualAContractError(
            "PLAYBACK_ADMISSION_MISMATCH", "; ".join(problems))


def verify_envelope_fields(envelope_fields):
    """Verify one envelope against the authoritative model and admission.

    The envelope is rebuilt from its canonical JSON with the G4 loader, the
    transmitted digest is recomputed from the transmitted bytes, the original
    physical request is compared with the retained raw request, the admitted
    planning context and its digests are matched, and geometry/dynamics are
    re-audited independently.  A missing raw request, context or admission
    record is an explicit rejection, never a default acceptance.
    """
    if not isinstance(envelope_fields, MappingABC):
        raise VirtualAContractError(
            INVALID_ENVELOPE_FIELDS, "envelope_fields must be a mapping")
    values = dict(envelope_fields)
    expected_request = values.pop("expected_request", None)
    expected_context = values.pop("expected_context", None)
    expected_grid_sha256 = values.pop("expected_grid_sha256", None)
    expected_admission = values.pop("expected_admission", None)
    geometry_audit_config = values.pop("geometry_audit_config", None)
    dynamic_audit_config = values.pop("dynamic_audit_config", None)
    position_tolerance = values.pop("position_tolerance", 1e-9)
    angle_tolerance = values.pop("angle_tolerance", 1e-9)
    speed_tolerance = values.pop("speed_tolerance", 1e-9)
    unknown = sorted(set(values) - ENVELOPE_KEYS)
    missing = sorted(ENVELOPE_KEYS - set(values))
    if unknown or missing:
        raise VirtualAContractError(
            INVALID_ENVELOPE_FIELDS,
            "envelope keys missing={} extra={}".format(missing, unknown))
    if values["backend"] != BACKEND:
        raise VirtualAContractError(
            "UNKNOWN_BACKEND",
            "backend={!r} expected {!r}".format(values["backend"], BACKEND))
    if values["semantic_version"] != PLAYBACK_SEMANTIC_VERSION:
        raise VirtualAContractError(
            "UNKNOWN_SEMANTIC_VERSION",
            "semantic_version={!r} expected {!r}".format(
                values["semantic_version"], PLAYBACK_SEMANTIC_VERSION))
    if values["model_version"] != TIMED_MODEL_VERSION:
        raise VirtualAContractError(
            "UNKNOWN_MODEL_VERSION",
            "model_version={!r} expected {!r}".format(
                values["model_version"], TIMED_MODEL_VERSION))
    if values["certification"] != PLAYBACK_CERTIFICATION:
        raise VirtualAContractError(
            "UNKNOWN_CERTIFICATION",
            "certification={!r} expected {!r}".format(
                values["certification"], PLAYBACK_CERTIFICATION))
    request = _coerce_expected_request(expected_request)
    if expected_context is None:
        raise VirtualAContractError(
            "PLAYBACK_CONTEXT_MISSING",
            "no admitted planning context is retained for this plan")
    if not isinstance(expected_context, MappingABC):
        raise VirtualAContractError(
            "INVALID_PLANNING_CONTEXT",
            "expected_context must be a mapping")
    expected_context = _strict_json(dict(expected_context),
                                    "expected planning context")
    if expected_context.get("schema") != G4_PLANNING_SCHEMA or \
            expected_context.get("stage") != "G4":
        raise VirtualAContractError(
            "INVALID_PLANNING_CONTEXT",
            "expected context schema/stage are not G4")
    context_sha = planning_context_sha256(expected_context)
    if expected_context.get("request_sha256") != request_fingerprint(request):
        raise VirtualAContractError(
            "PLAYBACK_CONTEXT_MISMATCH",
            "admitted context request_sha256 differs from the original "
            "request")
    grid = expected_context.get("grid")
    grid_sha = grid_summary_sha256(grid) if grid is not None else ""
    if expected_grid_sha256 is not None and expected_grid_sha256 != grid_sha:
        raise VirtualAContractError(
            "PLAYBACK_GRID_MISMATCH",
            "admitted grid digest differs from the expected grid")
    if expected_admission is not None:
        _verify_admission(expected_admission, request, context_sha, grid_sha)
    from ..formation_playback import verify_playback_plan
    trajectory_fields = values["trajectory_fields"]
    try:
        trajectory, plan, digest = verify_playback_plan(
            backend=values["backend"],
            plan_id=values["plan_id"],
            formation_id=values["formation_id"],
            semantic_version=values["semantic_version"],
            model_version=values["model_version"],
            model_sha256=values["model_sha256"],
            model_json=values["model_json"],
            certification=values["certification"],
            trajectory_fields=trajectory_fields,
            expected_request=request,
            expected_context=expected_context,
            expected_grid_sha256=grid_sha,
            geometry_audit_config=geometry_audit_config,
            dynamic_audit_config=dynamic_audit_config,
            position_tolerance=position_tolerance,
            angle_tolerance=angle_tolerance,
            speed_tolerance=speed_tolerance)
    except VirtualAContractError:
        raise
    except Exception as exc:
        from ..formation_playback import PlaybackValidationError
        if isinstance(exc, PlaybackValidationError):
            raise VirtualAContractError(
                "PLAYBACK_VERIFICATION_FAILED", str(exc)) from exc
        raise
    return VerifiedPlayback(
        trajectory=trajectory, plan=plan, model_sha256=digest,
        backend=BACKEND, semantic_version=PLAYBACK_SEMANTIC_VERSION,
        certification=PLAYBACK_CERTIFICATION)


def result_summary(result):
    """Strict-JSON summary of the G4 result, including the terminal state."""
    if result is None:
        return {"status": "NONE", "executable": False}
    summary = _json_safe(result.as_dict() if hasattr(result, "as_dict")
                         else {"status": str(getattr(result, "status", ""))})
    executable = bool(getattr(result, "executable", False))
    plan = getattr(result, "plan", None)
    terminal = None
    request_summary = None
    if isinstance(plan, VirtualATimedPlan):
        try:
            request = request_from_payload(
                plan.to_model_dict()["request_inputs"])
            request_summary = {
                "plan_id": request.plan_id,
                "formation_id": request.formation_id,
                "goal_policy": coerce_goal_policy(
                    request.goal_policy).value,
                "initial": request.initial.as_dict(),
                "start_qf": {"x": request.start_qf.x,
                             "y": request.start_qf.y,
                             "psi": request.start_qf.psi},
                "goal_qf": {"x": request.goal_qf.x,
                            "y": request.goal_qf.y,
                            "psi": request.goal_qf.psi},
                "request_sha256": result.request_sha256,
                "planning_context_sha256": result.planning_context_sha256,
                "grid_sha256": result.grid_sha256,
            }
            state = plan.state_at(plan.duration_s)
            terminal = {
                "t_s": state.t,
                "s_m": state.s,
                "region": state.region,
                "qf": {"x": state.qf.x, "y": state.qf.y, "psi": state.qf.psi},
                "vehicles": [
                    {"vehicle_id": vehicle.vehicle_id,
                     "x": vehicle.x, "y": vehicle.y,
                     "theta": vehicle.theta, "C_i": vehicle.C_i,
                     "v_i": vehicle.v_i, "omega_i": vehicle.omega_i,
                     "wheel_angles_rad": list(vehicle.wheel_angles)}
                    for vehicle in state.vehicles],
            }
        except Exception as exc:
            terminal = {"summary_error": "{}: {}".format(
                type(exc).__name__, exc)}
    summary.update({
        "executable": executable,
        "certification": PLAYBACK_CERTIFICATION if executable else "NONE",
        "model_summary": plan.model_summary()
        if isinstance(plan, VirtualATimedPlan) else None,
        "request": request_summary,
        "terminal": terminal,
        "audits": {
            "geometry": _report_summary(getattr(result, "geometry_audit", None)),
            "dynamics": _report_summary(getattr(result, "dynamic_audit", None)),
            "collision": _report_summary(
                getattr(result, "collision_audit", None)),
        },
        "virtual_a_playback": {
            "semantic_version": PLAYBACK_SEMANTIC_VERSION,
            "backend": BACKEND,
            "model_version": TIMED_MODEL_VERSION,
        },
        "collision_claim": (
            "planning context only; playback and this summary do not rerun "
            "map collision"),
    })
    return summary


def _report_summary(report):
    if report is None:
        return None
    return {
        "scope": getattr(report, "scope", None),
        "passed": bool(getattr(report, "passed", False)),
        "reason": getattr(report, "reason", ""),
        "detail": getattr(report, "detail", ""),
        "check_count": int(getattr(report, "check_count", 0) or 0),
        "request_sha256": getattr(report, "request_sha256", ""),
        "planning_context_sha256": getattr(
            report, "planning_context_sha256", ""),
        "grid_sha256": getattr(report, "grid_sha256", ""),
    }


class VirtualABackendHooks:
    """The exact hook surface consumed by the planner node."""

    backend = BACKEND
    model_version = TIMED_MODEL_VERSION
    semantic_version = PLAYBACK_SEMANTIC_VERSION
    certification = PLAYBACK_CERTIFICATION

    def __init__(self, settings=None):
        self.settings = settings or VirtualABackendSettings()

    def request_from_snapshot(self, snapshot, grid, settings,
                              initial_context):
        return request_from_snapshot(
            snapshot, grid, settings or self.settings, initial_context)

    def solve(self, request, progress_callback=None):
        return solve(request, progress_callback=progress_callback)

    def publication_fields(self, snapshot, result, stamp, dt):
        return publication_fields(
            snapshot, result, stamp, dt, self.settings)

    def verify_envelope_fields(self, envelope_fields):
        return verify_envelope_fields(envelope_fields)

    def result_summary(self, result):
        return result_summary(result)


__all__ = [
    "VirtualABackendHooks", "VirtualABackendSettings", "VirtualAInitialContext",
    "VerifiedPlayback", "request_from_snapshot", "publication_fields",
    "terminal_context_from_plan", "verify_envelope_fields", "result_summary",
    "solve", "INITIAL_RESET_NOT_CONFIRMED", "UNSUPPORTED_INITIAL_STATE",
    "NONPARALLEL_INITIAL_HEADINGS", "UNSUPPORTED_GOAL_HEADING",
]