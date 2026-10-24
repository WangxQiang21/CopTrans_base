#!/usr/bin/env python3
"""Probe one manually selected map goal and independently sweep the timed path."""
import argparse
import json
import math
import time
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"tools"))
from trajectory_direction_audit import audit_direction

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as SpatialPath
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from fwis_v3_msgs.msg import PlanSeed, TimedTrajectory

from ros_parking_goals_smoke import IndependentSweepChecker, Pose, wrap


def pose(x, y, theta):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x, msg.pose.position.y = x, y
    msg.pose.orientation.z = math.sin(theta / 2)
    msg.pose.orientation.w = math.cos(theta / 2)
    return msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-name", required=True)
    parser.add_argument("--start", nargs=3, type=float, required=True)
    parser.add_argument("--goal", nargs=3, type=float, required=True)
    parser.add_argument("--xy-tol", type=float, default=.16)
    parser.add_argument("--yaw-tol", type=float, default=.14)
    parser.add_argument("--timeout", type=float, default=60)
    parser.add_argument("--output")
    parser.add_argument("--expect-geometry-only", action="store_true")
    args = parser.parse_args()

    rospy.init_node("v3_map_probe", anonymous=True)
    grid = rospy.wait_for_message("/map", OccupancyGrid, timeout=15)
    checker = IndependentSweepChecker(
        grid,
        rospy.get_param("/fwis_v3_optimizer/vehicle/length", .738),
        rospy.get_param("/fwis_v3_optimizer/vehicle/width", .500),
        rospy.get_param("/fwis_v3_optimizer/collision/margin", .04),
    )
    data = {"seed": None, "search": None, "status": None, "traj": None,
            "quality": None, "corridors": None, "spatial_path": None, "hud": None}
    rospy.Subscriber("/plan/seed", PlanSeed,
                     lambda msg: data.update(seed=msg), queue_size=1)
    rospy.Subscriber("/plan/search_status", String,
                     lambda msg: data.update(search=msg.data), queue_size=1)
    rospy.Subscriber("/plan/status", String,
                     lambda msg: data.update(status=msg.data), queue_size=1)
    rospy.Subscriber("/plan/timed_trajectory", TimedTrajectory,
                     lambda msg: data.update(traj=msg), queue_size=1)
    rospy.Subscriber("/plan/corridor_markers", MarkerArray,
                     lambda msg: data.update(corridors=msg), queue_size=1)
    if args.expect_geometry_only:
        rospy.Subscriber("/plan/path", SpatialPath,
                         lambda msg: data.update(spatial_path=msg), queue_size=1)
        rospy.Subscriber("/robot/hud", Marker,
                         lambda msg: data.update(hud=msg), queue_size=1)
    def receive_quality(msg):
        try:
            data["quality"] = json.loads(msg.data)
        except ValueError:
            pass
    rospy.Subscriber("/plan/quality_report", String, receive_quality, queue_size=1)
    start_pub = rospy.Publisher("/start_pose", PoseStamped, queue_size=1)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    deadline = time.time() + 10
    while (start_pub.get_num_connections() == 0 or
           goal_pub.get_num_connections() == 0) and time.time() < deadline:
        rospy.sleep(.05)
    if not start_pub.get_num_connections() or not goal_pub.get_num_connections():
        raise RuntimeError("planner pose subscribers unavailable")

    prior_id = data["seed"].plan_id if data["seed"] else 0
    start_pub.publish(pose(*args.start))
    rospy.sleep(.25)
    goal_pub.publish(pose(*args.goal))
    expected_id = prior_id + 1
    deadline = time.time() + args.timeout
    while time.time() < deadline:
        search = data["search"] or ""
        if "id=%d" % expected_id in search:
            break
        rospy.sleep(.05)

    result = {
        "map": args.map_name, "start": args.start, "goal": args.goal,
        "plan_id": expected_id, "search": data["search"],
        "optimizer": None, "timed_sweep_clear": None,
    }
    seed_msg=data["seed"]
    if seed_msg and seed_msg.plan_id==expected_id:
        result["seed_actions"]=[
            {"mode":int(action.mode),"amount":float(action.amount),
             "control":float(action.control)}
            for action in seed_msg.primitives]
    if result["search"] and result["search"].startswith("search_success"):
        while time.time() < deadline:
            status = data["status"] or ""
            if "id=%d" % expected_id in status and (
                    status.startswith("success") or status.startswith("rejected") or
                    status.startswith("geometry_only")):
                break
            rospy.sleep(.05)
        result["optimizer"] = data["status"]
        if result["optimizer"] and result["optimizer"].startswith("success"):
            while time.time() < deadline:
                traj = data["traj"]
                if traj and traj.plan_id == expected_id and len(traj.points) > 1:
                    break
                rospy.sleep(.05)
            traj = data["traj"]
            if traj and traj.plan_id == expected_id and len(traj.points) > 1:
                points = traj.points
                poses = [Pose(pt.center_pose.x, pt.center_pose.y,
                              pt.center_pose.theta) for pt in points]
                clear = checker.pose_clear(poses[0])
                samples = 0
                for a, b in zip(poses, poses[1:]):
                    edge_clear, count, _ = checker.segment_clear(a, b, .005)
                    samples += count
                    if not edge_clear:
                        clear = False
                        break
                end = poses[-1]
                result.update(
                    trajectory_header={"frame_id": traj.header.frame_id,
                                       "stamp_s": traj.header.stamp.to_sec()},
                    timed_sweep_clear=clear,
                    sweep_samples=samples,
                    timed_points=len(points),
                    duration=points[-1].time_from_start.to_sec(),
                    position_error=math.hypot(end.x - args.goal[0],
                                              end.y - args.goal[1]),
                    heading_error=abs(wrap(end.th - args.goal[2])),
                    terminal_ok=(
                        math.hypot(end.x - args.goal[0],
                                   end.y - args.goal[1]) <= args.xy_tol and
                        abs(wrap(end.th - args.goal[2])) <= args.yaw_tol),
                    strictly_increasing_time=all(
                        b.time_from_start > a.time_from_start
                        for a, b in zip(points, points[1:])),
                )
                modes = [int(point.mode) for point in points]
                mode_runs = []
                for mode in modes:
                    if not mode_runs or mode_runs[-1]["mode"] != mode:
                        mode_runs.append({"mode": mode, "points": 1})
                    else:
                        mode_runs[-1]["points"] += 1
                result["timed_modes"] = modes
                result["mode_runs"] = mode_runs
                result["reverse_velocity_detected"] = any(
                    float(point.v) < -1e-8 for point in points)
                projections = []
                for a, b in zip(poses, poses[1:]):
                    heading = a.th+.5*wrap(b.th-a.th)
                    projections.append((b.x-a.x)*math.cos(heading)+(b.y-a.y)*math.sin(heading))
                result["reverse_body_projection_detected"] = any(value < -1e-6 for value in projections)
                if any(name in result["optimizer"] for name in
                       ("source=mode_path_v1", "source=mode_path_v2")):
                    while time.time() < deadline:
                        quality = data["quality"]
                        if (quality and quality.get("plan_id") == expected_id and
                                quality.get("status") == "FEASIBLE"):
                            break
                        rospy.sleep(.01)
                quality = data["quality"]
                if quality and quality.get("plan_id") == expected_id and quality.get("status") == "FEASIBLE":
                    result["quality_report"] = quality
                    limits = quality["selected"]["timing"]["profile"]
                else:
                    # Legacy uses its native smoothstep timing, with no common
                    # acceleration/steering contract. Measure those peaks but
                    # only certify the limits this backend actually promises.
                    limits = {
                        "v_max": rospy.get_param("/fwis_v3_optimizer/time/v_demo", .5),
                        "omega_max": rospy.get_param("/fwis_v3_optimizer/time/omega_demo", .5),
                        "a_max": None, "alpha_max": None,
                        "translation_omega_max": None,
                        "translation_alpha_max": None}
                result["dynamics_contract"] = (
                    "quality_report_profile" if "quality_report" in result
                    else "legacy_native_speed_only")
                t = [pt.time_from_start.to_sec() for pt in points]
                v = [float(pt.v) for pt in points]
                distances = [math.hypot(b.x-a.x, b.y-a.y)
                             for a, b in zip(poses, poses[1:])]
                total_distance = sum(distances)
                slow_distance = sum(d for d, va, vb in zip(distances, v, v[1:])
                                    if .5*(abs(va)+abs(vb)) < .05)
                result["speed_summary"] = {
                    "peak_mps": max(map(abs, v)),
                    "mean_moving_mps": total_distance/max(t[-1]-t[0], 1e-9),
                    "distance_below_0p05_mps_m": slow_distance,
                    "distance_below_0p05_fraction": slow_distance/max(total_distance, 1e-9),
                    "interior_zero_speed_samples": sum(
                        abs(value) < 1e-7 for value in v[1:-1])}
                corridors = data["corridors"]
                visible = [m for m in corridors.markers
                           if m.action == Marker.ADD and m.points] if corridors else []
                result["corridor_display"] = {
                    "markers": len(visible),
                    "line_points": sum(len(m.points) for m in visible),
                    "nonempty": bool(visible),
                    "markers_world": [
                        {"namespace": m.ns, "id": m.id,
                         "frame_id": m.header.frame_id,
                         "stamp_s": m.header.stamp.to_sec(),
                         "pose_position": [m.pose.position.x,m.pose.position.y,m.pose.position.z],
                         "pose_orientation_xyzw": [m.pose.orientation.x,m.pose.orientation.y,
                                                   m.pose.orientation.z,m.pose.orientation.w],
                         "points": [[p.x,p.y,p.z] for p in m.points]}
                        for m in visible]}
                result["timed_trace"] = [
                    [tt, p.x, p.y, p.th, vv, int(point.mode)]
                    for tt, p, vv, point in zip(t, poses, v, points)]
                acceleration = [(vb-va)/(tb-ta) for ta, tb, va, vb in zip(t,t[1:],v,v[1:])]
                mids = [(ta+tb)/2 for ta,tb in zip(t,t[1:])]
                angular_v = [wrap(b.th-a.th)/(tb-ta) for a,b,ta,tb in zip(poses,poses[1:],t,t[1:])]
                angular_a = [(wb-wa)/(tb-ta) for wa,wb,ta,tb in
                             zip(angular_v,angular_v[1:],mids,mids[1:])]
                peaks = {"speed": max(map(abs,v)), "acceleration": max(map(abs,acceleration),default=0),
                         "angular_speed": max(map(abs,angular_v),default=0),
                         "angular_acceleration": max(map(abs,angular_a),default=0)}
                yaw_ok = True
                for i, measured in enumerate(angular_v):
                    bound = (limits["omega_max"] if points[i].mode == 2 else
                             limits.get("translation_omega_max", limits["omega_max"]))
                    if points[i].mode != 2 and limits.get("translation_speed_policy") == "mode_switch_only":
                        bound = None
                    if bound is not None and bound > 0 and abs(measured) > bound+1e-5:
                        yaw_ok = False
                for i, measured in enumerate(angular_a):
                    bound = (limits["alpha_max"] if points[i].mode == 2 else
                             limits.get("translation_alpha_max", limits["alpha_max"]))
                    if points[i].mode != 2 and limits.get("translation_speed_policy") == "mode_switch_only":
                        bound = None
                    if bound is not None and bound > 0 and abs(measured) > bound+1e-5:
                        yaw_ok = False
                result["independent_dynamics"] = dict(peaks, pass_check=(
                    peaks["speed"] <= min(limits["v_max"], limits.get("target_v", limits["v_max"]))+1e-5 and
                    (limits["a_max"] is None or peaks["acceleration"] <= limits["a_max"]+1e-5) and yaw_ok))
                effective_direction_policy=rospy.get_param(
                    "/fwis_v3_optimizer/planning/direction_policy",None)
                if effective_direction_policy is None:
                    old_reverse=rospy.get_param(
                        "/fwis_v3_optimizer/planning/allow_reverse_fallback",True)
                    effective_direction_policy=("bidirectional" if old_reverse
                                                 else "forward_only")
                result["independent_direction"] = audit_direction(
                    [(pt.time_from_start.to_sec(), p.x, p.y, p.th, float(pt.v))
                     for pt, p in zip(points, poses)],
                    allow_reverse=(effective_direction_policy == "bidirectional"),
                    allow_shunting=rospy.get_param("/fwis_v3_optimizer/planning/allow_shunting",False),
                    report=quality)
    if data["quality"] and data["quality"].get("plan_id") == expected_id:
        result["quality_report"] = data["quality"]
    if args.expect_geometry_only:
        while time.time() < deadline:
            hud = data["hud"]
            if hud and "GEOMETRY ONLY" in hud.text and ("#%d " % expected_id) in hud.text:
                break
            rospy.sleep(.05)
        quality, spatial, traj, hud = data["quality"], data["spatial_path"], data["traj"], data["hud"]
        if quality and quality.get("plan_id") == expected_id:
            result["quality_report"] = quality
        poses = ([Pose(p.pose.position.x, p.pose.position.y,
                       2*math.atan2(p.pose.orientation.z, p.pose.orientation.w))
                  for p in spatial.poses] if spatial else [])
        clear = bool(poses) and checker.pose_clear(poses[0])
        for a, b in zip(poses, poses[1:]):
            if not checker.segment_clear(a, b, .005)[0]:
                clear = False
                break
        terminal = bool(poses) and math.hypot(poses[-1].x-args.goal[0],
                    poses[-1].y-args.goal[1]) <= args.xy_tol and abs(wrap(
                    poses[-1].th-args.goal[2])) <= args.yaw_tol
        result["geometry_only_check"] = {
            "quality_status": quality.get("status") if quality else None,
            "strict_geometry_retained": bool(quality and quality.get("plan_id") == expected_id and
                (quality.get("geometry_incumbent") or {}).get(
                    "geometry_audit", {}).get("geometry_status") == "PASS"),
            "no_executable_trajectory": bool(traj and traj.plan_id == expected_id and not traj.points),
            "spatial_sweep_clear": bool(clear), "spatial_terminal_ok": bool(terminal),
            "spatial_pose_count": len(poses),
            "hud_explains_uncertified_execution": bool(hud and "GEOMETRY ONLY" in hud.text and
                "EXECUTION NOT CERTIFIED" in hud.text and ("#%d " % expected_id) in hud.text),
            "hud_text": hud.text if hud else None,
        }
        check = result["geometry_only_check"]
        check["pass_check"] = (check["quality_status"] == "GEOMETRY_ONLY" and all(check[k] for k in
            ("strict_geometry_retained", "no_executable_trajectory", "spatial_sweep_clear",
             "spatial_terminal_ok", "hud_explains_uncertified_execution")))
    encoded = json.dumps(result, ensure_ascii=False, indent=2)
    print(encoded)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(encoded + "\n")
    if args.expect_geometry_only:
        raise SystemExit(0 if result["geometry_only_check"]["pass_check"] else 12)
    if not result["search"] or not result["search"].startswith("search_success"):
        raise SystemExit(2)
    if result["timed_sweep_clear"] is not True:
        raise SystemExit(3)
    if result.get("terminal_ok") is not True:
        raise SystemExit(4)
    if result.get("strictly_increasing_time") is not True:
        raise SystemExit(5)
    if result.get("independent_direction", {}).get("pass_check") is not True:
        raise SystemExit(6)
    if result.get("independent_dynamics", {}).get("pass_check", True) is not True:
        raise SystemExit(8)
    if any(source in str(result.get("optimizer")) for source in
           ("source=mode_path_v1", "source=mode_path_v2")) and "quality_report" not in result:
        raise SystemExit(9)
    if ("source=mode_path_v2" in str(result.get("optimizer")) and
            not result.get("corridor_display", {}).get("nonempty")):
        raise SystemExit(11)


if __name__ == "__main__":
    main()
