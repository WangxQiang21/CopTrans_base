#!/usr/bin/env python3
"""Offline verification of the captured V2 three-layer display payload.

This script reads only the saved evidence.  It does not start ROS and does not
trust the driver's summary: it rebuilds the virtual plan with the production
envelope verifier, then compares every displayed coordinate with the actual
A* route, the actual C2-A guide and the reconstructed final model.  It also
recomputes the payload hash from display_messages.jsonl and exercises the
negative display classifications on copies of the captured payload.

A successful run proves the display layers agree with the saved algorithm
data; it is not a new solver, audit or safety result.
"""
import copy
import hashlib
import json
import math
import sys
from pathlib import Path

from fwis_v3_optimizer.formation_continuous_a.contracts import (
    geometry_audit_config_from_dict,
    request_fingerprint,
)
from fwis_v3_optimizer.formation_playback import (
    CORRIDOR_EDGE_ALPHA,
    CORRIDOR_EDGE_Z_M,
    CORRIDOR_FILL_ALPHA,
    CORRIDOR_FILL_Z_M,
    GUIDE_ASTAR_CANDIDATE_NAMESPACE,
    GUIDE_ASTAR_COLOR,
    GUIDE_ASTAR_NAMESPACE,
    GUIDE_ASTAR_WIDTH_M,
    GUIDE_CENTERLINE_CANDIDATE_NAMESPACE,
    GUIDE_CENTERLINE_COLOR,
    GUIDE_CENTERLINE_NAMESPACE,
    GUIDE_CENTERLINE_WIDTH_M,
    GUIDE_UNVALIDATED_COLOR,
    GUIDE_Z_M,
    MODE_COLORS,
    PlaybackValidationError,
    build_guide_markers,
    frame_at,
    verify_virtual_a_playback_plan,
    virtual_a_request_from_raw_request,
)

RUN = Path(__file__).resolve().parent
TOL = 1e-9
COLOR_TOL = 1e-6
SAFETY_WORDS = (
    "SAFE", "CERTIFIED", "VERIFIED", "APPROVED", "COLLISION-FREE",
    "COLLISION_FREE", "EXECUTABLE", "FEASIBLE", "PASSED",
)


class Checks:
    def __init__(self):
        self.items = []

    def add(self, name, detail=None, passed=True):
        self.items.append({"check": name, "passed": bool(passed),
                           "detail": detail})
        if not passed:
            raise AssertionError("{}: {}".format(name, detail))
        print("PASS {} {}".format(name, "" if detail is None else detail))

    def close(self, name, actual, expected, tolerance=TOL, detail=None):
        error = abs(float(actual) - float(expected))
        effective = tolerance if math.isfinite(tolerance) else tolerance
        self.add(name, {"actual": actual, "expected": expected,
                        "error": error, "tolerance": effective},
                 error <= effective)
        return error


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def marker_content(document):
    copied = copy.deepcopy(document)
    for marker in copied["markers"]:
        marker["header"].pop("stamp", None)
    return copied


def payload_hash(debug_document, corridors, guides):
    return hashlib.sha256(canonical({
        "planning_debug": debug_document,
        "corridors": marker_content(corridors) if corridors else None,
        "guides": marker_content(guides) if guides else None,
    }).encode("utf-8")).hexdigest()


def max_error(pairs):
    return max((math.hypot(actual[0] - expected[0],
                           actual[1] - expected[1])
                for actual, expected in pairs), default=0.0)


def find_last(entries, topic):
    matches = [entry["document"] for entry in entries
               if entry["topic"] == topic]
    if not matches:
        raise AssertionError("no recorded message on {}".format(topic))
    return matches[-1]


def region_contains(region, point, tolerance=1e-9):
    for row, bound in zip(region["A"], region["b"]):
        if row[0] * point[0] + row[1] * point[1] > bound + tolerance:
            return False
    return True


def check_rviz_config(checks):
    import yaml
    document = yaml.safe_load(
        (RUN / "v2_display.rviz").read_text(encoding="utf-8"))
    displays = document["Visualization Manager"]["Displays"]
    by_topic = {item.get("Marker Topic"): item for item in displays
                if item.get("Marker Topic")}
    checks.add("rviz final layer", {
        "topic": "/formation/markers",
        "name": by_topic.get("/formation/markers", {}).get("Name")},
        by_topic.get("/formation/markers", {}).get("Name") ==
        "Final certified vehicle paths")
    guides = by_topic.get("/formation/guides")
    checks.add("rviz guide layer exists", {"present": guides is not None})
    if guides:
        namespaces = guides.get("Namespaces", {})
        checks.add("rviz A* default hidden",
                   {"formation/guides/astar":
                    namespaces.get("formation/guides/astar")},
                   namespaces.get("formation/guides/astar") is False)
        checks.add("rviz centerline default visible",
                   {"formation/guides/centerline":
                    namespaces.get("formation/guides/centerline")},
                   namespaces.get("formation/guides/centerline") is True)
        checks.add("rviz candidate namespaces default hidden",
                   {"astar_candidate":
                    namespaces.get("formation/guides/astar_candidate"),
                    "centerline_candidate":
                    namespaces.get("formation/guides/centerline_candidate")},
                   namespaces.get("formation/guides/astar_candidate")
                   is False and
                   namespaces.get("formation/guides/centerline_candidate")
                   is False)
    corridor = by_topic.get("/formation/corridors")
    checks.add("rviz corridor layer", {"present": corridor is not None,
                                       "name": corridor.get("Name")
                                       if corridor else None},
               corridor is not None)


def main():
    checks = Checks()
    result = json.loads((RUN / "run_result.json").read_text(encoding="utf-8"))
    entries = [json.loads(line) for line in
               (RUN / "display_messages.jsonl").read_text(
                   encoding="utf-8").splitlines() if line.strip()]
    capture = result["capture"]
    verification = result["verification"]
    job = result["job_state"]

    checks.add("schema", {"schema": result["schema"]},
               result["schema"] == "virtual_a_corridor.v2.real_ros_scene.v1")
    checks.add("private ROS master", {"uri": result["ros_master_uri"]},
               result["ros_master_uri"] == "http://127.0.0.1:11585")

    # The recorded one-shot topics must be the exact payload captured in
    # run_result.json; hashing the raw log then proves the saved hash.
    for topic, key in (("/formation/planning_debug", "planning_debug"),
                       ("/formation/corridors", "corridors"),
                       ("/formation/guides", "guides")):
        recorded = find_last(entries, topic)
        checks.add("captured payload equals recorded {}".format(topic),
                   {"same": recorded == capture[key]}, recorded == capture[key])

    recomputed = payload_hash(capture["planning_debug"],
                              capture["corridors"], capture["guides"])
    checks.add("payload sha256 from raw recorded messages",
               {"recomputed": recomputed,
                "recorded": capture["payload_sha256"]},
               recomputed == capture["payload_sha256"])
    checks.add("payload sha256 from run_result documents",
               {"same": payload_hash(capture["planning_debug"],
                                     capture["corridors"],
                                     capture["guides"]) ==
                capture["payload_sha256"]})

    debug = capture["planning_debug"]
    identity = {key: debug.get(key) for key in (
        "state", "plan_id", "formation_id", "backend", "generation",
        "grid_sha256", "planning_context_sha256", "model_sha256")}
    checks.add("planning debug identity", identity,
               identity == {
                   "state": "CANDIDATE_REFERENCE_POINT_ONLY",
                   "plan_id": result["plan_id"],
                   "formation_id": result["formation_id"],
                   "backend": verification["backend"],
                   "generation": 1,
                   "grid_sha256": verification["grid_sha256"],
                   "planning_context_sha256":
                       verification["planning_context_sha256"],
                   "model_sha256": verification["model_sha256"],
               })
    checks.add("planning debug has one used corridor route",
               {"routes": len(debug["corridors"])},
               len(debug["corridors"]) == 1)
    corridor = debug["corridors"][0]
    checks.add("route is algorithm-consumed current route", {
        "route_id": corridor.get("route_id"),
        "corridor_used": corridor.get("corridor_used"),
        "route_source": corridor.get("route_source"),
        "used_by_optimizer_present": "used_by_optimizer" in corridor,
    }, corridor.get("route_id") == 0 and
       corridor.get("corridor_used") is True and
       corridor.get("route_source") == "astar_primary" and
       "used_by_optimizer" not in corridor)
    checks.add("chain digest is also chain sha256", {
        "chain_digest": corridor.get("chain_digest"),
        "chain_sha256": corridor.get("chain_sha256")},
        corridor.get("chain_sha256") == corridor.get("chain_digest") and
        len(corridor.get("chain_sha256", "")) == 64 and
        all(ch in "0123456789abcdef"
            for ch in corridor.get("chain_sha256", "")))
    checks.add("corridor entry repeats document identity",
               {"same": all(corridor.get(key) == debug.get(key) for key in (
                   "plan_id", "formation_id", "backend", "generation",
                   "grid_sha256", "planning_context_sha256",
                   "model_sha256"))},
               all(corridor.get(key) == debug.get(key) for key in (
                   "plan_id", "formation_id", "backend", "generation",
                   "grid_sha256", "planning_context_sha256",
                   "model_sha256")))

    quality = corridor["guide_quality"]
    checks.add("guide label is PL_GUIDE", {
        "guide_label": corridor.get("guide_label"),
        "guide_is_executable_path": corridor.get(
            "guide_is_executable_path")},
        corridor.get("guide_label") == "PL_GUIDE" and
        corridor.get("guide_is_executable_path") is False)
    checks.add("guide quality contract", {
        "stage": quality.get("stage"),
        "rounding": quality.get("rounding"),
        "smoothness_claim": quality.get("smoothness_claim"),
        "is_executable_path": quality.get("is_executable_path"),
        "region_count": quality.get("region_count"),
        "warnings": quality.get("warnings"),
        "chain_digest_matches": quality.get("chain_digest") ==
                                corridor.get("chain_sha256"),
    }, quality.get("stage") == "C2_A" and
       quality.get("rounding") == "PL_ONLY_NO_BEZIER" and
       quality.get("smoothness_claim") == "NONE" and
       quality.get("is_executable_path") is False and
       quality.get("region_count") == len(corridor["regions"]) == 2 and
       quality.get("warnings") == [] and
       quality.get("chain_digest") == corridor.get("chain_sha256"))

    route_points = corridor["raw_route_points"]
    guide_points = corridor["guide_points"]
    checks.add("raw A* route point count", {"count": len(route_points)},
               len(route_points) == 246)
    checks.add("guide point count", {"count": len(guide_points)},
               len(guide_points) == 3)
    checks.add("A* route endpoints", {
        "first": route_points[0], "last": route_points[-1]},
        route_points[0] == [result["scene"]["start"][0],
                            result["scene"]["start"][1]] and
        route_points[-1] == [result["scene"]["goal"][0],
                             result["scene"]["goal"][1]])
    checks.add("guide endpoints equal chain endpoints", {
        "first": guide_points[0], "last": guide_points[-1]},
        guide_points[0] == corridor["regions"][0]["seed_start"] and
        guide_points[-1] == corridor["regions"][-1]["seed_end"])
    checks.add("guide portal lies inside both corridor regions", {
        "portal": guide_points[1],
        "region0": region_contains(corridor["regions"][0],
                                   guide_points[1]),
        "region1": region_contains(corridor["regions"][1],
                                   guide_points[1]),
    }, region_contains(corridor["regions"][0], guide_points[1]) and
       region_contains(corridor["regions"][1], guide_points[1]))

    # Rebuild the actual plan from the recorded envelope.  This is the same
    # production verifier used by the playback node, not a display-side
    # approximation.
    envelope = find_last(entries, "/formation/playback_plan")
    initial = job["initial_context"]
    request = virtual_a_request_from_raw_request(
        result["request"], initial_source=initial["source"],
        initial_context_id=initial["context_id"])
    fingerprint = request_fingerprint(request)
    checks.add("request fingerprint matches admission",
               {"request": fingerprint,
                "job": job.get("request_sha256")},
               fingerprint == job.get("request_sha256"))
    geometry_config = geometry_audit_config_from_dict(
        job["planning_context"]["timing_config"]["geometry_audit_config"])
    trajectory, plan, digest = verify_virtual_a_playback_plan(
        plan_id=int(envelope["plan_id"]),
        formation_id=envelope["formation_id"],
        backend=envelope["backend"],
        semantic_version=envelope["semantic_version"],
        model_version=envelope["model_version"],
        model_sha256=envelope["model_sha256"],
        model_json=envelope["model_json"],
        certification=envelope["certification"],
        trajectory_fields=envelope["trajectory_fields"],
        expected_request=request,
        expected_context=job["planning_context"],
        expected_grid_sha256=job["grid_sha256"],
        geometry_audit_config=geometry_config)
    checks.add("production envelope verifier", {
        "digest": digest,
        "message_digest": envelope["model_sha256"],
        "sample_count": len(trajectory.times_from_start),
        "duration_s": plan.duration_s},
        digest == envelope["model_sha256"] and
        len(trajectory.times_from_start) == 1982 and
        abs(plan.duration_s - 39.602577282743304) <= 1e-12)

    # Guide markers: actual route/guide polylines, verbatim, no smoothing.
    guides = capture["guides"]
    guide_markers = guides["markers"]
    checks.add("guide layer clear marker",
               {"markers": [(m["ns"], m["id"], m["action"])
                            for m in guide_markers[:1]]},
               guide_markers[0]["action"] == 3 and
               guide_markers[0]["ns"] == "formation")
    line_markers = [m for m in guide_markers if m["type"] == 4]
    text_markers = [m for m in guide_markers if m["type"] == 9]
    checks.add("guide layer has five markers",
               {"count": len(guide_markers)}, len(guide_markers) == 5)
    astar = [m for m in line_markers
             if m["ns"] == GUIDE_ASTAR_NAMESPACE]
    center = [m for m in line_markers
              if m["ns"] == GUIDE_CENTERLINE_NAMESPACE]
    checks.add("A* line namespace and point count", {
        "count": len(astar), "points": len(astar[0]["points"])
        if astar else None},
        len(astar) == 1 and len(astar[0]["points"]) == 246)
    checks.add("centerline namespace and point count", {
        "count": len(center), "points": len(center[0]["points"])
        if center else None},
        len(center) == 1 and len(center[0]["points"]) == 3)
    astar_error = max_error(
        (tuple(marker[:2]), tuple(actual))
        for marker, actual in zip(astar[0]["points"], route_points))
    center_error = max_error(
        (tuple(marker[:2]), tuple(actual))
        for marker, actual in zip(center[0]["points"], guide_points))
    checks.close("A* marker coordinates vs actual route", astar_error, 0.0)
    checks.close("centerline marker coordinates vs actual guide",
                 center_error, 0.0)
    checks.add("guide marker z values", {
        "astar_ok": all(abs(p[2] - GUIDE_Z_M) <= TOL
                        for p in astar[0]["points"]),
        "center_ok": all(abs(p[2] - GUIDE_Z_M) <= TOL
                         for p in center[0]["points"]),
    }, all(abs(p[2] - GUIDE_Z_M) <= TOL
           for p in astar[0]["points"] + center[0]["points"]))
    checks.add("guide line colors and widths", {
        "astar_color": astar[0]["color"],
        "center_color": center[0]["color"],
        "astar_scale": astar[0]["scale"],
        "center_scale": center[0]["scale"],
    }, all(abs(astar[0]["color"][i] - GUIDE_ASTAR_COLOR[i]) <= COLOR_TOL
           for i in range(4)) and
       all(abs(center[0]["color"][i] - GUIDE_CENTERLINE_COLOR[i])
           <= COLOR_TOL for i in range(4)) and
       abs(astar[0]["scale"][0] - GUIDE_ASTAR_WIDTH_M) <= TOL and
       abs(center[0]["scale"][0] - GUIDE_CENTERLINE_WIDTH_M) <= TOL)
    labels = {m["ns"]: m["text"] for m in text_markers}
    checks.add("guide labels", labels,
               labels.get(GUIDE_ASTAR_NAMESPACE) ==
               "A* ROUTE | REFERENCE_ONLY" and
               labels.get(GUIDE_CENTERLINE_NAMESPACE) ==
               "PL_GUIDE | REFERENCE_ONLY")
    all_text = " ".join(m["text"] for m in text_markers).upper()
    checks.add("guide labels carry no safety words", {"text": all_text},
               not any(word in all_text for word in SAFETY_WORDS))

    # Corridor layer: exact vertices and low-alpha fill.
    corridors = capture["corridors"]["markers"]
    checks.add("corridor layer clear marker",
               {"first": corridors[0]["action"]},
               corridors[0]["ns"] == "formation" and
               corridors[0]["action"] == 3)
    fill = [m for m in corridors if m["type"] == 11]
    edge = [m for m in corridors if m["type"] == 4]
    checks.add("corridor marker counts", {
        "fill": len(fill), "edge": len(edge)}, len(fill) == 2 and
        len(edge) == 2)
    fill_error = 0.0
    edge_error = 0.0
    for region in corridor["regions"]:
        region_id = region["region_id"]
        fill_marker = next((m for m in fill if m["id"] == region_id), None)
        edge_marker = next((m for m in edge if m["id"] == region_id), None)
        checks.add("corridor region {} markers exist".format(region_id),
                   {"fill": fill_marker is not None,
                    "edge": edge_marker is not None},
                   fill_marker is not None and edge_marker is not None)
        points = region["vertices"]
        expected_fill = []
        for index in range(1, len(points) - 1):
            for x, y in (points[0], points[index], points[index + 1]):
                expected_fill.append((x, y, CORRIDOR_FILL_Z_M))
        expected_edge = [(x, y, CORRIDOR_EDGE_Z_M) for x, y in points]
        expected_edge.append((points[0][0], points[0][1],
                              CORRIDOR_EDGE_Z_M))
        fill_error = max(fill_error, max_error(zip(
            (tuple(p) for p in fill_marker["points"]), expected_fill)))
        edge_error = max(edge_error, max_error(zip(
            (tuple(p) for p in edge_marker["points"]), expected_edge)))
        checks.add("corridor region {} fill style".format(region_id),
                   {"color": fill_marker["color"],
                    "scale": fill_marker["scale"]},
                   abs(fill_marker["color"][3] - CORRIDOR_FILL_ALPHA)
                   <= COLOR_TOL and
                   fill_marker["scale"] == [1.0, 1.0, 1.0])
        checks.add("corridor region {} edge style".format(region_id),
                   {"color": edge_marker["color"],
                    "scale": edge_marker["scale"]},
                   abs(edge_marker["color"][3] - CORRIDOR_EDGE_ALPHA)
                   <= COLOR_TOL and
                   edge_marker["scale"] == [0.03, 0.0, 0.0])
    checks.close("corridor fill vertices vs actual regions", fill_error,
                 0.0)
    checks.close("corridor edge vertices vs actual regions", edge_error,
                 0.0)

    # Final three-vehicle paths: current formation only, mode colors kept.
    final = capture["markers"]["markers"]
    final_namespaces = {m["ns"] for m in final}
    checks.add("final marker identity", {
        "namespaces": sorted(final_namespaces)},
        final_namespaces == {
            "formation/synthetic_g4/vehicle_1",
            "formation/synthetic_g4/vehicle_2",
            "formation/synthetic_g4/vehicle_3",
            "formation/synthetic_g4/links",
            "formation/synthetic_g4/reference",
        } and not any("candidate" in ns for ns in final_namespaces))
    path_error = 0.0
    path_color_error = 0.0
    state_error = 0.0
    frame = frame_at(trajectory, capture["clamped_elapsed_s"])

    # The captured /formation/markers array is the latest published tick, not
    # atomically bundled with the captured status string.  Identify the actual
    # status tick whose three centres the marker OBBs reproduce, then compare
    # the OBBs with frame_at at that tick.  The status used for the 1e-7
    # non-sample check above remains the capture tick.
    obb_centres = {}
    for marker in final:
        if (marker["type"] == 4 and len(marker["points"]) == 5 and
                "/vehicle_" in marker["ns"]):
            vehicle_id = int(marker["ns"].rsplit("_", 1)[1])
            points = marker["points"][:4]
            obb_centres[vehicle_id] = (
                sum(point[0] for point in points) / 4.0,
                sum(point[1] for point in points) / 4.0)
    marker_status = None
    marker_status_error = None
    for entry in entries:
        document = entry["document"]
        if (entry["topic"] != "/formation/playback_status" or
                document.get("plan_id") != result["plan_id"]):
            continue
        vehicles = document.get("vehicles")
        if (not isinstance(vehicles, list) or len(vehicles) != 3 or
                any(not isinstance(item.get("center"), (list, tuple))
                    for item in vehicles) or
                {item["vehicle_id"] for item in vehicles} !=
                set(obb_centres)):
            continue
        error = max(
            math.hypot(item["center"][0] -
                       obb_centres[item["vehicle_id"]][0],
                       item["center"][1] -
                       obb_centres[item["vehicle_id"]][1])
            for item in vehicles)
        if marker_status is None or error < marker_status_error:
            marker_status = document
            marker_status_error = error
    checks.add("marker OBB matches an actual status tick", {
        "error_m": marker_status_error,
        "status_elapsed_s": (marker_status or {}).get("clamped_elapsed_s"),
        "captured_elapsed_s": capture["clamped_elapsed_s"],
    }, marker_status is not None and marker_status_error <= 1e-7)
    marker_frame = frame_at(trajectory,
                            marker_status["clamped_elapsed_s"])
    for vehicle in trajectory.vehicles:
        namespace = "formation/synthetic_g4/vehicle_{}".format(
            vehicle.vehicle_id)
        vehicle_markers = [m for m in final if m["ns"] == namespace]
        path_marker = next(
            (m for m in vehicle_markers
             if m["type"] == 4 and len(m["points"]) == 1982), None)
        obb_marker = next(
            (m for m in vehicle_markers
             if m["type"] == 4 and len(m["points"]) == 5), None)
        checks.add("final path marker vehicle {}".format(
            vehicle.vehicle_id), {"path": path_marker is not None,
                                  "obb": obb_marker is not None},
            path_marker is not None and obb_marker is not None)
        samples = trajectory.points[vehicle.vehicle_id]
        state = marker_frame.vehicle_map()[vehicle.vehicle_id]
        for index, (marker_point, sample) in enumerate(
                zip(path_marker["points"], samples)):
            analytic = plan.state_at(trajectory.times_from_start[index])
            analytic_vehicle = {item.vehicle_id: item
                                for item in analytic.vehicles}[
                                    vehicle.vehicle_id]
            path_error = max(path_error, math.hypot(
                marker_point[0] - analytic_vehicle.x,
                marker_point[1] - analytic_vehicle.y))
            expected_color = MODE_COLORS[sample.mode]
            actual_color = path_marker["colors"][index]
            path_color_error = max(path_color_error, max(
                abs(actual_color[channel] - expected_color[channel])
                for channel in range(4)))
        checks.add("final path is bold mode color vehicle {}".format(
            vehicle.vehicle_id), {
            "scale": path_marker["scale"],
            "colors": sorted({tuple(round(value, 6)
                                    for value in color)
                              for color in path_marker["colors"]}),
        }, path_marker["scale"] == [0.05, 0.0, 0.0] and
           all(abs(color[channel] - MODE_COLORS[0][channel])
               <= COLOR_TOL for color in path_marker["colors"]
               for channel in range(4)))
        expected_obb = vehicle.obb_corners(state.center_x, state.center_y,
                                           state.theta)
        for marker_point, expected in zip(obb_marker["points"],
                                          expected_obb):
            state_error = max(state_error, math.hypot(
                marker_point[0] - expected[0],
                marker_point[1] - expected[1]))
    checks.close("final path coordinates vs analytic plan", path_error, 0.0)
    checks.close("final path mode colors", path_color_error, 0.0,
                 tolerance=COLOR_TOL)
    checks.close("current OBB vs frame_at", state_error, 0.0)

    # Playback status at the captured non-sample instant.
    status = next((entry["document"] for entry in entries
                   if entry["topic"] == "/formation/playback_status" and
                   entry["document"].get("plan_id") == result["plan_id"] and
                   entry["document"].get("clamped_elapsed_s") ==
                   capture["clamped_elapsed_s"]), None)
    checks.add("capture status exists", {"present": status is not None})
    checks.add("capture status certification", {
        "state": status.get("state"),
        "executable": status.get("executable"),
        "certification": status.get("certification"),
        "audit_status": status.get("audit_status"),
        "geometry_only": status.get("geometry_only"),
        "marker_count": status.get("marker_count"),
        "model_sha256": status.get("model_sha256"),
    }, status.get("state") == "PLAYING" and
       status.get("executable") is True and
       status.get("certification") == "FEASIBLE" and
       status.get("audit_status") == "FEASIBLE" and
       status.get("geometry_only") is False and
       status.get("marker_count") == 19 and
       status.get("model_sha256") == verification["model_sha256"])
    checks.add("status marker namespaces equal final layer", {
        "status": sorted(status["marker_namespaces"]),
        "final": sorted(final_namespaces)},
        sorted(status["marker_namespaces"]) == sorted(final_namespaces))
    checks.add("captured instant is not a published sample", {
        "nearest_sample_delta_s": capture["nearest_sample_delta_s"],
        "non_sample_time": capture["non_sample_time"],
        "progress": capture["progress"]},
        capture["non_sample_time"] is True and
        capture["nearest_sample_delta_s"] > 1e-9 and
        0.46 <= capture["progress"] < 0.47)
    live_position_error = 0.0
    live_heading_error = 0.0
    published_by_id = {item["vehicle_id"]: item
                       for item in status["vehicles"]}
    for vehicle_id, published in published_by_id.items():
        reference = frame.vehicle_map()[vehicle_id]
        live_position_error = max(
            live_position_error,
            math.hypot(published["center"][0] - reference.center_x,
                       published["center"][1] - reference.center_y))
        live_heading_error = max(
            live_heading_error,
            abs(math.atan2(
                math.sin(published["theta"] - reference.theta),
                math.cos(published["theta"] - reference.theta))))
    checks.add("live status frame vs authoritative frame_at", {
        "position_error_m": live_position_error,
        "heading_error_rad": live_heading_error,
        "standard_m": 1e-7},
        live_position_error <= 1e-7 and live_heading_error <= 1e-7)

    # Negative classifications on copies of the real captured payload.
    candidate = copy.deepcopy(debug)
    candidate["corridors"] = [copy.deepcopy(corridor)]
    candidate["corridors"][0]["corridor_used"] = False
    candidate_markers = build_guide_markers(candidate)
    candidate_lines = [m for m in candidate_markers
                       if m.marker_type == "LINE_STRIP"]
    candidate_text = [m for m in candidate_markers
                      if m.marker_type == "TEXT_VIEW_FACING"]
    candidate_namespaces = {m.namespace for m in candidate_lines}
    checks.add("unused route is candidate-only and default-off namespaces", {
        "namespaces": sorted(candidate_namespaces),
        "labels": [m.text for m in candidate_text],
    }, candidate_namespaces == {
        GUIDE_ASTAR_CANDIDATE_NAMESPACE,
        GUIDE_CENTERLINE_CANDIDATE_NAMESPACE,
    } and all("CANDIDATE" in m.text for m in candidate_text))

    unvalidated = copy.deepcopy(debug)
    unvalidated["corridors"] = [copy.deepcopy(corridor)]
    unvalidated["corridors"][0]["guide_quality"]["warnings"] = [
        "synthetic tamper for display classification"]
    unvalidated_markers = build_guide_markers(unvalidated)
    unvalidated_text = [m for m in unvalidated_markers
                        if m.marker_type == "TEXT_VIEW_FACING"]
    unvalidated_center = next(
        m for m in unvalidated_markers
        if m.namespace == GUIDE_CENTERLINE_NAMESPACE and
        m.marker_type == "LINE_STRIP")
    checks.add("tampered guide is UNVALIDATED_GUIDE", {
        "labels": [m.text for m in unvalidated_text],
        "color": unvalidated_center.color,
    }, any("UNVALIDATED_GUIDE" in m.text for m in unvalidated_text) and
       all(abs(unvalidated_center.color[i] -
               GUIDE_UNVALIDATED_COLOR[i]) <= COLOR_TOL for i in range(4)) and
       not any(word in " ".join(m.text for m in unvalidated_text).upper()
               for word in SAFETY_WORDS))

    mismatched = copy.deepcopy(debug)
    mismatched["corridors"] = [copy.deepcopy(corridor)]
    mismatched["corridors"][0]["plan_id"] += 1000
    try:
        build_guide_markers(mismatched)
    except PlaybackValidationError as exc:
        checks.add("mismatched route identity is rejected",
                   {"error": str(exc)})
    else:
        checks.add("mismatched route identity is rejected", None, False)

    screenshot = RUN / "images/screenshot_mid_curve.png"
    screenshot_hash = hashlib.sha256(screenshot.read_bytes()).hexdigest()
    checks.add("screenshot exists and size matches", {
        "bytes": screenshot.stat().st_size,
        "recorded_bytes": result["screenshot"]["bytes"],
        "sha256": screenshot_hash},
        screenshot.stat().st_size == result["screenshot"]["bytes"])
    checks.add("screenshot hash recorded", {"sha256": screenshot_hash})

    check_rviz_config(checks)

    output = {
        "schema": "virtual_a_corridor.v2.display_verification.v1",
        "verified": True,
        "run": str(RUN),
        "payload_sha256": capture["payload_sha256"],
        "markers_sha256": capture["markers_sha256"],
        "screenshot_sha256": screenshot_hash,
        "checks": checks.items,
        "max_errors_m": {
            "astar_marker_vs_route": astar_error,
            "centerline_marker_vs_guide": center_error,
            "corridor_fill_vs_regions": fill_error,
            "corridor_edge_vs_regions": edge_error,
            "final_path_vs_analytic_plan": path_error,
            "final_path_mode_color": path_color_error,
            "current_obb_vs_frame_at": state_error,
            "live_status_vs_frame_at_position": live_position_error,
            "live_status_vs_frame_at_heading_rad": live_heading_error,
        },
    }
    (RUN / "verification.json").write_text(
        json.dumps(output, sort_keys=True, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")
    print(canonical({"ok": True,
                     "payload_sha256": capture["payload_sha256"],
                     "max_errors_m": output["max_errors_m"]}))
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except AssertionError as exc:
        print("FAIL {}".format(exc))
        sys.exit(1)