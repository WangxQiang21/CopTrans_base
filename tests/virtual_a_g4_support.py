"""Shared G4 test support; this module is not a unittest test file."""
from pathlib import Path
import json

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import (
    FormationModel,
    FormationPose,
    VehicleOffset,
)
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a import optimizer as optimizer_module
from fwis_v3_optimizer.formation_continuous_a.audit import audit_geometry
from fwis_v3_optimizer.formation_continuous_a.collision import load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    CollisionConfig,
    GeometryAuditConfig,
    GoalPolicy,
    InitialAssumption,
    PlannerConfig,
    TimingConfig,
    VirtualAMapRequest,
    VirtualARequest,
    collision_config_from_dict,
    planner_config_from_dict,
    planning_context,
    request_fingerprint,
    request_from_payload,
    timing_config_from_dict,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.timing import retime_path


ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = ROOT / "tests" / "fixtures" / "virtual_a"
SYNTHETIC_MAPS = FIXTURE_DIR / "maps" / "synthetic"
SCENE_MAPS = FIXTURE_DIR / "maps" / "scenes"
SCENE_REQUESTS = FIXTURE_DIR / "scenes"

MODEL = FormationModel([
    VehicleOffset(1, 1.0, 0.0),
    VehicleOffset(2, 0.0, 1.0),
    VehicleOffset(3, 0.0, -1.0),
])
SHAPES = {
    1: VehicleShape(0.738, 0.500),
    2: VehicleShape(0.738, 0.500),
    3: VehicleShape(0.738, 0.500),
}
LIMITS = FormationTimingLimits()
INITIAL = InitialAssumption(True, "A", True, "offline_fixture")

# G4 reference/planning audit budget.  G2 rework 01 and G2/G3 acceptance fixed
# the independent audit at <= 5e-8 m; the exact per-panel M4 bound in
# integration_bounds lets the 12 m S2 profile confirm that threshold without
# reducing the G1 1e-7 m production contract, the floating/angle guard or the
# collision margin.
REFERENCE_GEOMETRY_AUDIT = GeometryAuditConfig(audit_position_budget_m=5e-8)
TIMING = TimingConfig(geometry_audit_config=REFERENCE_GEOMETRY_AUDIT)
PLANNER = PlannerConfig()
COLLISION = CollisionConfig()
CORNER_STATION_M = 2.625
CORNER_VEHICLE_ID = 1


def make_request(start_qf, goal_qf, *, plan_id=1,
                 limits=LIMITS, start_headings=None):
    return VirtualARequest(
        plan_id=plan_id,
        formation_id="synthetic_g4",
        model=MODEL,
        vehicle_shapes=SHAPES,
        start_qf=start_qf,
        start_headings=(
            {1: 0.0, 2: 0.0, 3: 0.0}
            if start_headings is None else dict(start_headings)),
        goal_qf=goal_qf,
        initial=INITIAL,
        goal_policy=GoalPolicy.FREE,
        limits=limits)


def straight_profile(length=10.0):
    return C2CurvatureProfile.from_knots([
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(float(length), 0.0, 0.0),
    ])


def ramp_profile(length=10.0, final_curvature=0.2):
    """C2 curvature ramp from an accepted zero-curvature initial state."""
    return C2CurvatureProfile.from_knots([
        ControlKnot(0.0, 0.0, 0.0),
        ControlKnot(float(length), float(final_curvature), 0.0),
    ])


def straight_path(length=10.0):
    return VirtualAPath(FormationPose(0.0, 0.0, 0.0),
                        straight_profile(length), MODEL)


def ramp_path(length=10.0, final_curvature=0.2):
    return VirtualAPath(FormationPose(0.0, 0.0, 0.0),
                        ramp_profile(length, final_curvature), MODEL)


def hold_path(start_qf=None):
    return VirtualAPath.hold(
        FormationPose(0.0, 0.0, 0.0) if start_qf is None else start_qf,
        MODEL)


def retime(path, grid, *, request=None, planner=None, timing=None,
           collision=None, expected=("DYNAMIC_FEASIBLE", "NO_MOTION")):
    request = request or make_request(path.start_qf, path.endpoint())
    planner = PLANNER if planner is None else planner
    timing = TIMING if timing is None else timing
    collision = COLLISION if collision is None else collision
    context = planning_context(request, grid, planner, timing, collision)
    result = retime_path(
        request, path, timing, planning_context=context)
    if result.status not in expected:
        raise AssertionError(
            "synthetic retime failed: {} {} {}".format(
                result.status, result.reason, result.detail))
    return request, result, context


def map_request(request, grid, *, planner=None, timing=None,
                collision=None):
    return VirtualAMapRequest(
        request=request,
        grid=grid,
        planner_config=PLANNER if planner is None else planner,
        timing_config=TIMING if timing is None else timing,
        collision_config=COLLISION if collision is None else collision)


def load_synthetic(name):
    return load_map(SYNTHETIC_MAPS / name / "map.yaml")


def empty_grid(width=320, height=120, resolution=0.05,
               origin_x=-2.0, origin_y=-3.0):
    import numpy as np
    return Grid(
        np.zeros((int(height), int(width)), dtype=bool),
        float(resolution), float(origin_x), float(origin_y),
        length=0.738, width=0.500)


def scene_inputs(name):
    """Return (grid, map_metadata, payload) for one frozen scene."""
    grid, metadata = load_map(SCENE_MAPS / (name + ".yaml"))
    payload = json.loads(
        (SCENE_REQUESTS / (name + ".request.json")).read_text(
            encoding="utf-8"))
    return grid, metadata, payload


def scene_map_request(name):
    grid, metadata, payload = scene_inputs(name)
    request = request_from_payload(payload["request"])
    planner = planner_config_from_dict(payload["planner_config"])
    timing = timing_config_from_dict(payload["timing_config"])
    collision = collision_config_from_dict(payload["collision_config"])
    return VirtualAMapRequest(request, grid, planner, timing, collision), \
        metadata, payload


def geometry_result_for(path, request=None, planner=None):
    """Build a real audited geometry-only result for pipeline injection."""
    request = request or make_request(path.start_qf, path.endpoint())
    planner = PLANNER if planner is None else planner
    audit = audit_geometry(request, path, REFERENCE_GEOMETRY_AUDIT)
    if not audit.passed:
        raise AssertionError(
            "injected path failed geometry audit: {} {}".format(
                audit.reason, audit.detail))
    return optimizer_module._result(
        "GEOMETRY_FEASIBLE",
        request=request,
        path=path,
        audit=audit,
        request_sha256=request_fingerprint(request),
        config=planner,
        solver_used="test_injection")