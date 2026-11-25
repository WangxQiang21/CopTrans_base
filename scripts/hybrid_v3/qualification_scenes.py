"""Geometry contracts and measured trajectory checks for the two V3 qualification worlds."""
import json
import math
from pathlib import Path

import numpy as np


SPECIAL_SCENES = {"multibend": "v3_multibend", "necessary_aux": "v3_auxiliary"}


def setup(family, task, config, *, scene=None):
    """Called after normal --scenario NAME --experiment full_route resolution."""
    if family not in SPECIAL_SCENES:
        return {}
    if config["scenario"] != SPECIAL_SCENES[family] or config["waypoints"]:
        raise ValueError("Qualification requires the declared scene and no supplied waypoints")
    from ranger_mini_gz.scene_generator import load_scene
    scene = scene or load_scene(config["scenario"])
    start, goal = (scene.experiments[task][key] for key in ("start", "goal"))
    if tuple(config["start"]) != start or tuple(config["goal"]) != goal:
        raise ValueError("Closed-loop request endpoints must match the fixture")
    obstacles = {o.name: o for o in scene.obstacles}
    if family == "multibend":
        gates = []
        for name in ("west", "middle", "east"):
            lower, upper = (obstacles[f"{name}_{part}"] for part in ("lower", "upper"))
            gates.append([lower.pose[0], lower.bounds[1][1], upper.bounds[0][1]])
        if not (gates[1][2] < min(gates[0][1], gates[2][1])
                and start[1] < gates[0][1] and goal[1] < gates[2][1]):
            raise ValueError("Gate openings do not force alternating lateral travel")
        return {"type": "ordered_high_low_high_crossings", "gates_x_ymin_ymax": gates}
    if task == "formation":
        headings = [p[2] for p in scene.spawn_poses(task).values()]
        matrix = [[-math.sin(h), math.cos(h), math.cos(h)*x+math.sin(h)*y]
                  for h, (x, y) in zip(headings, scene.offsets, strict=True)]
        sigma = float(np.linalg.svd(matrix, compute_uv=False)[-1])
        # Independent uncertainty of 1 degree in each vehicle yaw and in F yaw.
        perturbation = math.radians(1)*math.sqrt(3+4*sum(x*x+y*y for x, y in scene.offsets))
        if not sigma > perturbation:
            raise ValueError("Start mount tangents do not exclude A on a neighborhood")
        return {"type": "full_rank_start_mount_matrix", "body_headings_rad": headings,
                "smallest_singular_value": sigma, "uncertainty_bound": perturbation}
    width = obstacles["slot_east"].bounds[0][0] - obstacles["slot_west"].bounds[1][0]
    dx, dy = scene.vehicle_size[:2]
    overrides = config["planner_overrides"]
    margin = overrides["vehicle"].get("safety_margin", .04)
    dx, dy = dx + 2*margin, dy + 2*margin
    vmax = overrides["quality"]["limits"]["v_max"]
    lateral = abs(goal[1] - start[1])
    cap = overrides["task_duration_cap_s"]
    alpha = .22
    projected_at_barrier = dx*math.cos(alpha) + dy*math.sin(alpha)
    max_lateral_A = vmax*cap*math.sin(alpha)
    if not (abs(start[2])+math.radians(1) < alpha and projected_at_barrier > width
            and max_lateral_A < lateral-.04):
        raise ValueError("Pure-A lateral reach is not excluded at the resolved limits")
    # The body cannot pass +/-alpha in this strip. Integrating |v*sin(yaw)|
    # bounds lateral progress for any number of stopped gear changes.
    return {"type": "bounded_lateral_reach", "physical_width_m": width,
            "body_width_at_yaw_barrier_m": projected_at_barrier, "yaw_barrier_rad": alpha,
            "task_duration_cap_s": cap, "speed_limit_mps": vmax, "max_pure_A_lateral_m": max_lateral_A,
            "min_required_lateral_m": lateral-.04}


def measured(family, task, proof, spool):
    """Read the same measured feedback and delivered model used by the normal runner."""
    if family not in SPECIAL_SCENES:
        return {}
    spool = Path(spool)
    feedback = spool/"metrics/motion.jsonl"
    models = list((spool/"trajectories").rglob("model.json"))
    if not feedback.exists() or len(models) != 1:
        return dict(passed=False, reason="missing_delivered_model_or_measured_motion")
    plan = json.loads(models[0].read_text())
    motion_modes = [ph["mode"] for ph in plan["phases"] if ph["kind"] == 0]
    crossings, moving_modes = [], set()
    gates = proof.get("gates_x_ymin_ymax", [])
    previous = None
    with feedback.open() as stream:
        for line in stream:
            row = json.loads(line)
            ref = row["cargo_reference"] if task == "formation" else row["poses"]["1"]
            for car, twist in (row.get("odom_twist") or {}).items():
                if max(abs(x) for x in twist) > .015 and car in (row.get("mode_feedback") or {}):
                    moving_modes.add(row["mode_feedback"][car])
            if previous and len(crossings) < len(gates):
                gx = gates[len(crossings)][0]
                if previous[0] < gx <= ref[0]:
                    f = (gx - previous[0]) / (ref[0] - previous[0])
                    crossings.append(previous[1] + f*(ref[1]-previous[1]))
            previous = ref
    if family == "multibend":
        passed = (motion_modes == [0] and moving_modes == {0} and len(crossings) == len(gates)
                  and all(lo < y < hi for y, (_,lo,hi) in zip(crossings,gates)))
    else:
        passed = bool(set(motion_modes) & {1, 2} & moving_modes)
    return {"passed": bool(passed), "motion_modes": motion_modes,
            "moving_feedback_modes": sorted(moving_modes), "measured_gate_crossing_y_m": crossings}
