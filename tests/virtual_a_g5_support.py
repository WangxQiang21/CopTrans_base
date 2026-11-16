"""Shared G5 test support; this module is not a unittest test file.

All scenes are the frozen G4 test copies of the Codex-accepted scene inputs.
The support builds a real normalised ``FormationRequestSnapshot`` and runs the
real autonomous G4 pipeline; no reference trajectory is injected.
"""
import copy
import json
import math
from pathlib import Path

from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_request import (
    FormationRequestSnapshot,
    VehicleRequest,
)
from fwis_v3_optimizer.formation_continuous_a.collision import load_map
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    collision_config_from_dict,
    planner_config_from_dict,
    planning_context_sha256,
    request_fingerprint,
    request_from_payload,
    timing_config_from_dict,
)
from fwis_v3_optimizer.formation_continuous_a.ros_adapter import (
    VirtualABackendSettings,
    VirtualAInitialContext,
    publication_fields,
    request_from_snapshot,
    solve,
)

ROOT = Path(__file__).resolve().parents[1]
SCENE_ROOT = ROOT / "tests" / "fixtures" / "virtual_a" / "scenes"
MAP_ROOT = ROOT / "tests" / "fixtures" / "virtual_a" / "maps" / "scenes"
STAMP = (1234, 567000000)

_CACHE = {}


def load_scene(name):
    bundle = json.loads((SCENE_ROOT / (name + ".request.json")).read_text())
    request = request_from_payload(bundle["request"])
    grid, metadata = load_map(MAP_ROOT / (name + ".yaml"))
    return {
        "name": name,
        "bundle": bundle,
        "request": request,
        "grid": grid,
        "metadata": metadata,
        "planner": planner_config_from_dict(bundle["planner_config"]),
        "timing": timing_config_from_dict(bundle["timing_config"]),
        "collision": collision_config_from_dict(bundle["collision_config"]),
    }


def settings_for(scene, policy="free"):
    return VirtualABackendSettings(
        planner_config=scene["planner"],
        timing_config=scene["timing"],
        collision_config=scene["collision"],
        goal_policy=policy)


def snapshot_for(scene, *, policy="free", start=None, goal=None,
                 start_headings=None, goal_headings=None,
                 goal_heading_free=None, allow_reverse=False,
                 plan_id=None, formation_id=None):
    request = scene["request"]
    start = request.start_qf if start is None else start
    goal = request.goal_qf if goal is None else goal
    headings = (dict(request.start_headings)
                if start_headings is None else dict(start_headings))
    goal_values = ({vehicle_id: goal.psi
                    for vehicle_id in request.model.vehicle_ids}
                   if goal_headings is None else dict(goal_headings))
    if goal_heading_free is None:
        free = {vehicle_id: policy == "free"
                for vehicle_id in request.model.vehicle_ids}
    else:
        free = {int(k): bool(v) for k, v in goal_heading_free.items()}
    start_centers = request.model.centers(start)
    goal_centers = request.model.centers(goal)
    vehicles = tuple(
        VehicleRequest(
            vehicle_id=offset.vehicle_id,
            request_index=index,
            offset_x=offset.x,
            offset_y=offset.y,
            start_x=start_centers[offset.vehicle_id][0],
            start_y=start_centers[offset.vehicle_id][1],
            start_theta=headings[offset.vehicle_id],
            goal_x=goal_centers[offset.vehicle_id][0],
            goal_y=goal_centers[offset.vehicle_id][1],
            goal_theta=goal_values[offset.vehicle_id],
            goal_heading_free=free[offset.vehicle_id],
            body_length_m=request.vehicle_shapes[offset.vehicle_id].length_m,
            body_width_m=request.vehicle_shapes[offset.vehicle_id].width_m)
        for index, offset in enumerate(request.model.offsets))
    return FormationRequestSnapshot(
        plan_id=request.plan_id if plan_id is None else int(plan_id),
        formation_id=(request.formation_id if formation_id is None
                      else formation_id),
        header_stamp=(1, 0),
        start=start,
        goal=goal,
        model=request.model,
        vehicle_shapes=dict(request.vehicle_shapes),
        vehicles=vehicles,
        start_headings=headings,
        goal_headings=goal_values,
        goal_heading_free=free,
        allow_reverse=bool(allow_reverse))


def snapshot_from_request(request, *, allow_reverse=False):
    """Build the raw snapshot a bridge would derive from a G4 request.

    The request itself carries no per-vehicle goal headings or free flags, so
    the snapshot mirrors the request's goal policy: FREE uses every requested
    goal heading (the model heading), STRAIGHT uses those headings as fixed.
    """
    from fwis_v3_optimizer.formation_continuous_a.contracts import GoalPolicy

    policy = request.goal_policy
    if not isinstance(policy, GoalPolicy):
        policy = GoalPolicy[policy] if policy in GoalPolicy.__members__ \
            else GoalPolicy(str(policy))
    goal_headings = {vehicle_id: request.goal_qf.psi
                     for vehicle_id in request.model.vehicle_ids}
    free = {vehicle_id: policy == GoalPolicy.FREE
            for vehicle_id in request.model.vehicle_ids}
    start_centers = request.model.centers(request.start_qf)
    goal_centers = request.model.centers(request.goal_qf)
    vehicles = tuple(
        VehicleRequest(
            vehicle_id=offset.vehicle_id,
            request_index=index,
            offset_x=offset.x,
            offset_y=offset.y,
            start_x=start_centers[offset.vehicle_id][0],
            start_y=start_centers[offset.vehicle_id][1],
            start_theta=request.start_headings[offset.vehicle_id],
            goal_x=goal_centers[offset.vehicle_id][0],
            goal_y=goal_centers[offset.vehicle_id][1],
            goal_theta=goal_headings[offset.vehicle_id],
            goal_heading_free=free[offset.vehicle_id],
            body_length_m=request.vehicle_shapes[offset.vehicle_id].length_m,
            body_width_m=request.vehicle_shapes[offset.vehicle_id].width_m)
        for index, offset in enumerate(request.model.offsets))
    return FormationRequestSnapshot(
        plan_id=request.plan_id,
        formation_id=request.formation_id,
        header_stamp=(1, 0),
        start=request.start_qf,
        goal=request.goal_qf,
        model=request.model,
        vehicle_shapes=dict(request.vehicle_shapes),
        vehicles=vehicles,
        start_headings=dict(request.start_headings),
        goal_headings=goal_headings,
        goal_heading_free=free,
        allow_reverse=bool(allow_reverse))


def reset_context(scene, context_id="virtual-a-reset-test-1", qf=None):
    return VirtualAInitialContext(
        source="rviz_reset", context_id=context_id,
        qf=scene["request"].start_qf if qf is None else qf)


def solved(scene_name, policy="free"):
    """Solve one scene once per process; the pipeline is deterministic."""
    key = (scene_name, policy)
    if key not in _CACHE:
        scene = load_scene(scene_name)
        snapshot = snapshot_for(scene, policy=policy)
        settings = settings_for(scene, policy=policy)
        map_request = request_from_snapshot(
            snapshot, scene["grid"], settings, reset_context(scene))
        result = solve(map_request)
        _CACHE[key] = {
            "scene": scene,
            "snapshot": snapshot,
            "settings": settings,
            "map_request": map_request,
            "result": result,
            "context": map_request.context(),
            "context_sha256": planning_context_sha256(map_request.context()),
        }
    return _CACHE[key]


def admission_for(solved_case, request=None, context=None):
    map_request = solved_case["map_request"]
    request = map_request.request if request is None else request
    context_sha = (solved_case["context_sha256"] if context is None
                   else planning_context_sha256(context))
    return {
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "request_sha256": map_request.request_sha256,
        "planning_context_sha256": context_sha,
        "grid_sha256": map_request.grid_sha256,
        "initial_source": request.initial.source,
        "initial_context_id": request.initial.context_id,
        "goal_policy": request.goal_policy.value,
    }


def envelope_for(solved_case, *, stamp=STAMP, dt=0.02,
                 expected_request=None, expected_context=None,
                 expected_admission=None, digest=None, model_json=None,
                 fields=None):
    result = solved_case["result"]
    request = solved_case["map_request"].request
    return envelope_for_result(
        result=result, request=request, context=solved_case["context"],
        grid_sha256=solved_case["map_request"].grid_sha256,
        snapshot=solved_case["snapshot"], stamp=stamp, dt=dt,
        expected_request=expected_request, expected_context=expected_context,
        expected_admission=expected_admission, digest=digest,
        model_json=model_json, fields=fields)


def envelope_for_result(*, result, request, context, grid_sha256, snapshot,
                        stamp=STAMP, dt=0.02, expected_request=None,
                        expected_context=None, expected_admission=None,
                        digest=None, model_json=None, fields=None):
    """Build one complete atomic envelope for an executable G4 result."""
    fields = (publication_fields(snapshot, result, stamp, dt)
              if fields is None else fields)
    admission = {
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "request_sha256": request_fingerprint(request),
        "planning_context_sha256": planning_context_sha256(context),
        "grid_sha256": grid_sha256,
        "initial_source": request.initial.source,
        "initial_context_id": request.initial.context_id,
        "goal_policy": request.goal_policy.value,
    }
    return {
        "plan_id": request.plan_id,
        "formation_id": request.formation_id,
        "backend": "virtual_rigid_a",
        "semantic_version": "virtual_rigid_a.playback.v1",
        "model_version": "virtual_rigid_a.timed.v1",
        "model_sha256": result.model_sha256 if digest is None else digest,
        "model_json": (result.plan.to_model_json()
                       if model_json is None else model_json),
        "certification": "virtual_a_geometry_dynamics_collision/v1",
        "trajectory_fields": fields,
        "expected_request": (request if expected_request is None
                             else expected_request),
        "expected_context": (context if expected_context is None
                             else expected_context),
        "expected_grid_sha256": grid_sha256,
        "expected_admission": (admission if expected_admission is None
                               else expected_admission),
    }


def copy_json(value):
    return copy.deepcopy(value)


def plan_terminal(plan):
    return plan.state_at(plan.duration_s)


def angular_error(first, second):
    return abs(math.atan2(math.sin(first - second),
                          math.cos(first - second)))


def straight_vehicle_shape(vehicle_id=1):
    return VehicleShape(0.738, 0.500)


def pose_from_state(state):
    return FormationPose(state.qf.x, state.qf.y, state.qf.psi)