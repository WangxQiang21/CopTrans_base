#!/usr/bin/env python3
"""Independently audit a recorded FormationTimedTrajectory (T0/acceptance tool).

Checks are derived from the contract, not from the planner's own construction
formulas:

* rigid centres ``p_i = p_F + R(psi_F) r_i`` at every published sample;
* the same identity at NON-sample times using the actual display interpolation
  (``formation_playback.frame_at``), which is what RViz really shows;
* whether any sample exhibits ``theta_i != psi_F`` and whether the three body
  headings ever differ from each other (required by the new contract);
* pairwise centre distances d12/d13/d23 against the wedge geometry.

Usage: python3 tools/formation_trajectory_audit.py CASE_DIR/trajectory.json \
           --out report.json
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..",
    "catkin_ws", "src", "fwis_v3_optimizer", "src"))

from fwis_v3_optimizer.formation_playback import (  # noqa: E402
    frame_at, trajectory_from_fields)


def message_to_fields(message):
    """Adapt a serialized FormationTimedTrajectory to playback field values."""
    stamp = message["header"]["stamp"]
    if not isinstance(stamp, (list, tuple)):
        secs = int(stamp)
        stamp = (secs, int(round((float(stamp) - secs) * 1e9)))
    return {
        "frame_id": message["header"]["frame_id"],
        "stamp": stamp,
        "plan_id": message["plan_id"],
        "formation_id": message["formation_id"],
        "times_from_start": message["times_from_start"],
        "formation_poses": [[p["x"], p["y"], p["theta"]]
                            if isinstance(p, dict) else list(p)
                            for p in message["formation_poses"]],
        "vehicles": [{
            "vehicle_id": v["vehicle_id"],
            "offset": [v["offset_in_formation"]["x"],
                       v["offset_in_formation"]["y"]],
            "body_length_m": v["body_length_m"],
            "body_width_m": v["body_width_m"],
        } for v in message["vehicles"]],
        "trajectories": [{
            "vehicle_id": track["vehicle_id"],
            "points": [[p["time_from_start"], p["center_pose"]["x"],
                        p["center_pose"]["y"], p["center_pose"]["theta"],
                        p["center_speed_mps"], p["center_yaw_rate_radps"],
                        p["mode"], p["mode_control"]]
                       for p in track["points"]],
        } for track in message["trajectories"]],
    }


def rigid_centres(pose, offsets):
    x, y, psi = pose
    c, s = math.cos(psi), math.sin(psi)
    return {vid: (x + c * off[0] - s * off[1], y + s * off[0] + c * off[1])
            for vid, off in offsets.items()}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("trajectory")
    parser.add_argument("--out")
    parser.add_argument("--midpoints", type=int, default=3)
    args = parser.parse_args()

    message = json.load(open(args.trajectory))
    fields = message_to_fields(message)
    trajectory = trajectory_from_fields(fields)
    offsets = {v.vehicle_id: (v.offset_x, v.offset_y)
               for v in trajectory.vehicles}
    ids = sorted(offsets)
    times = trajectory.times_from_start
    poses = trajectory.formation_poses

    node_error = 0.0
    heading_vs_cargo = 0.0
    headings_differ = False
    theta_not_psi = False
    distance_error = 0.0
    pair_reference = None
    for index, pose in enumerate(poses):
        expected = rigid_centres(pose, offsets)
        thetas = {}
        for vid in ids:
            point = trajectory.points[vid][index]
            node_error = max(node_error, math.hypot(
                point.center_x - expected[vid][0],
                point.center_y - expected[vid][1]))
            thetas[vid] = point.theta
            heading_vs_cargo = max(heading_vs_cargo, abs(
                math.atan2(math.sin(point.theta - pose[2]),
                           math.cos(point.theta - pose[2]))))
            if abs(math.atan2(math.sin(point.theta - pose[2]),
                              math.cos(point.theta - pose[2]))) > 1e-6:
                theta_not_psi = True
        if len(set(round(t, 9) for t in thetas.values())) > 1:
            headings_differ = True
        pairs = {(a, b): math.hypot(expected[a][0] - expected[b][0],
                                    expected[a][1] - expected[b][1])
                 for i, a in enumerate(ids) for b in ids[i + 1:]}
        if pair_reference is None:
            pair_reference = pairs
        for key, value in pairs.items():
            distance_error = max(distance_error,
                                 abs(value - pair_reference[key]))

    # Display interpolation at non-sample times (arc vs chord).
    display_error = 0.0
    worst = None
    for index in range(len(times) - 1):
        span = times[index + 1] - times[index]
        for step in range(1, args.midpoints + 1):
            fraction = step / float(args.midpoints + 1)
            elapsed = times[index] + fraction * span
            frame = frame_at(trajectory, elapsed)
            psi = frame.formation_psi
            pose = (frame.formation_x, frame.formation_y, psi)
            expected = rigid_centres(pose, offsets)
            for state in frame.vehicles:
                error = math.hypot(state.center_x - expected[state.vehicle_id][0],
                                   state.center_y - expected[state.vehicle_id][1])
                if error > display_error:
                    display_error = error
                    worst = {"elapsed_s": elapsed, "vehicle_id": state.vehicle_id,
                             "segment": index}

    report = {
        "trajectory": os.path.abspath(args.trajectory),
        "plan_id": fields.get("plan_id"),
        "samples": len(times),
        "duration_s": times[-1] - times[0] if times else 0.0,
        "offsets": {str(k): list(v) for k, v in offsets.items()},
        "max_node_rigidity_error_m": node_error,
        "max_display_rigidity_error_m": display_error,
        "worst_display_error": worst,
        "max_pairwise_distance_drift_m": distance_error,
        "pairwise_distances_m": {"%d-%d" % k: v
                                 for k, v in (pair_reference or {}).items()},
        "max_abs_theta_minus_psi_rad": heading_vs_cargo,
        "any_theta_differs_from_cargo_psi": theta_not_psi,
        "any_two_vehicle_headings_differ": headings_differ,
    }
    text = json.dumps(report, indent=1, sort_keys=True)
    if args.out:
        with open(args.out, "w") as handle:
            handle.write(text + "\n")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
