#!/usr/bin/env python3
"""TASK02 Step F live ROS/RViz capture driver (task-private master only).

This script drives one already-running default ``./run_demo.sh`` session on
the task-private master and records only what the live nodes actually publish.
It reuses the TASK01 smoke helpers for the two RViz-style user topics
(``/initialpose`` and ``/move_base_simple/goal``), the job/request/envelope
recorders and the RViz screenshot function.  It adds Step E/F specific
recordings: ``/formation/planning_debug`` (full documents), the guide and
candidate marker arrays, the active A*/Guide path topics and the HUD text.

It never publishes a reference trajectory and never writes ``/formation/
playback_plan``.  If the request fails, that failure is recorded, not retried
with different parameters.

usage: ros_step_f_capture.py --run-dir DIR --label NAME \
       --reset X,Y,THETA --goal X,Y,THETA [--planning-timeout S]
"""
import argparse
import json
import math
import os
import sys
import threading
import time
from pathlib import Path

import rospy
from nav_msgs.msg import Path as NavPath
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray

ROOT = Path(__file__).resolve().parents[5]
sys.path.insert(0, str(ROOT / "tests"))
import ros_virtual_a_smoke as smoke  # noqa: E402

try:
    from jsk_rviz_plugins.msg import OverlayText
except ImportError:  # pragma: no cover - only in a broken ROS install
    OverlayText = None

PLANNING_DEBUG_TOPIC = "/formation/planning_debug"
QUALITY_TOPIC = "/formation/quality_report"
ASTAR_ACTIVE_TOPIC = "/formation/path/astar_active"
GUIDE_ACTIVE_TOPIC = "/formation/path/portal_guide_active"
CANDIDATES_TOPIC = "/formation/planning_candidates"
GUIDES_TOPIC = "/formation/guides"
HUD_TOPIC = "/formation/hud"
PROTECTED_PORTS = (11311, 11411)


def json_safe(value):
    return smoke.json_safe(value)


def marker_summary(message):
    """Compact, bounded view of one MarkerArray."""
    namespace_counts = {}
    namespace_types = {}
    texts = []
    colors = {}
    for marker in message.markers:
        namespace = str(marker.ns)
        namespace_counts[namespace] = namespace_counts.get(namespace, 0) + 1
        type_name = marker.type
        namespace_types.setdefault(namespace, set()).add(type_name)
        if marker.text:
            texts.append("{}: {}".format(namespace, marker.text))
        colors.setdefault(namespace, []).append([
            round(float(marker.color.r), 6),
            round(float(marker.color.g), 6),
            round(float(marker.color.b), 6),
            round(float(marker.color.a), 6)])
    return {
        "marker_count": len(message.markers),
        "namespace_counts": namespace_counts,
        "namespace_types": {
            name: sorted(values) for name, values in namespace_types.items()},
        "texts": texts[:80],
        "text_count": len(texts),
        "colors": {
            name: values[0] for name, values in colors.items()},
        "contains_certified_text": any(
            "CERTIFIED" in text.upper() for text in texts),
    }


def path_summary(message):
    points = [[float(pose.pose.position.x), float(pose.pose.position.y)]
              for pose in message.poses]
    return {
        "frame_id": str(message.header.frame_id),
        "stamp": [int(message.header.stamp.secs),
                  int(message.header.stamp.nsecs)],
        "point_count": len(points),
        "points": points,
        "first": points[0] if points else None,
        "last": points[-1] if points else None,
    }


def runtime_map_summary(message):
    width = int(message.info.width)
    height = int(message.info.height)
    resolution = float(message.info.resolution)
    origin = [float(message.info.origin.position.x),
              float(message.info.origin.position.y)]
    occupancy = [value >= 65 if value >= 0 else True for value in message.data]
    grid = smoke.Grid(
        [occupancy[index:index + width]
         for index in range(0, len(occupancy), width)],
        resolution, origin[0], origin[1], length=.738, width=.500)
    return {
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin": origin,
        "occupied_cells": sum(1 for value in occupancy if value),
        "runtime_grid_sha256": smoke.grid_signature(grid),
    }


def zoomed_screenshot(path, up=6):
    """Close-up of the formation for the Step F layer evidence screenshot."""
    import subprocess
    environment = dict(os.environ)
    environment.setdefault("DISPLAY", ":1")
    focused = smoke._focus_rviz(environment)
    if focused is None:
        return smoke.screenshot(path)
    window_id, width, height = focused
    target_x = int(max(1, width) * 0.55)
    target_y = int(max(1, height) * 0.62)
    smoke._xdotool(environment, "mousemove", "--window", window_id,
                   str(target_x), str(target_y))
    for _ in range(up):
        smoke._xdotool(environment, "click", "4")
        time.sleep(0.08)
    time.sleep(0.5)
    try:
        completed = subprocess.run(
            ["import", "-window", window_id, str(path)],
            env=environment, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, timeout=15.0)
    finally:
        for _ in range(up):
            smoke._xdotool(environment, "click", "5")
            time.sleep(0.08)
    return {
        "ok": completed.returncode == 0 and path.is_file(),
        "returncode": int(completed.returncode),
        "stderr": completed.stderr.decode("utf-8", "replace")[-400:],
    }


def parse_triple(value):
    parts = [float(item) for item in value.split(",")]
    if len(parts) != 3:
        raise SystemExit("pose must be x,y,theta")
    return tuple(parts)


def read_planner_params():
    names = (
        "backend", "virtual_a_guidance_mode",
        "virtual_a_corridor_enforcement", "virtual_a_goal_policy",
        "corridor_margin_m", "corridor_audit_min_width_m",
        "corridor_audit_position_margin_m", "intervals", "solver")
    document = {}
    for name in names:
        try:
            value = rospy.get_param(
                "/formation_uniform_planner/" + name, None)
        except Exception as exc:  # rosparam server failure is recorded
            value = "ERROR: {}: {}".format(type(exc).__name__, exc)
        document[name] = value
    try:
        document["planner_config"] = rospy.get_param(
            "/formation_uniform_planner/planner_config", None)
    except Exception as exc:
        document["planner_config"] = "ERROR: {}: {}".format(
            type(exc).__name__, exc)
    return document


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--reset", required=True)
    parser.add_argument("--goal", required=True)
    parser.add_argument("--planning-timeout", type=float, default=240.0)
    parser.add_argument("--screenshot-seconds", type=float, default=40.0)
    parser.add_argument("--node-name", default="task02_step_f_capture")
    args = parser.parse_args()

    master = os.environ.get("ROS_MASTER_URI", "")
    port = None
    if master:
        try:
            port = int(master.rstrip("/").rsplit(":", 1)[1])
        except (IndexError, ValueError):
            port = None
    if port is None or port in PROTECTED_PORTS:
        raise SystemExit(
            "refusing protected/default master {!r}".format(master))

    run_dir = Path(args.run_dir).resolve()
    images = run_dir / "images"
    images.mkdir(parents=True, exist_ok=True)
    reset = parse_triple(args.reset)
    goal = parse_triple(args.goal)

    rospy.init_node(args.node_name, anonymous=True)

    session = smoke.Session()
    recordings = {
        "planning_debug": [],
        "quality_reports": [],
        "astar_paths": [],
        "guide_paths": [],
        "candidate_markers": [],
        "guide_markers": [],
        "hud_texts": [],
        "received_at": {},
    }
    lock = threading.Lock()

    def stamp(name, payload):
        with lock:
            payload["_wall_s"] = rospy.Time.now().to_sec()
            recordings["received_at"].setdefault(name, []).append(
                payload["_wall_s"])
            return payload

    def on_debug(message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            document = {"invalid_json": True, "raw_length": len(message.data)}
        if isinstance(document, dict):
            stamp("planning_debug", document)
            with lock:
                recordings["planning_debug"].append(document)

    def on_quality(message):
        try:
            document = json.loads(message.data)
        except (TypeError, ValueError):
            document = {"invalid_json": True, "raw_length": len(message.data)}
        if not isinstance(document, dict):
            document = {"invalid_json": True, "raw_length": len(message.data)}
        stamp("quality", document)
        with lock:
            recordings["quality_reports"].append(document)

    def on_astar_path(message):
        summary = path_summary(message)
        stamp("astar_path", summary)
        with lock:
            recordings["astar_paths"].append(summary)

    def on_guide_path(message):
        summary = path_summary(message)
        stamp("guide_path", summary)
        with lock:
            recordings["guide_paths"].append(summary)

    def on_candidates(message):
        summary = marker_summary(message)
        stamp("candidates", summary)
        with lock:
            recordings["candidate_markers"].append(summary)

    def on_guides(message):
        summary = marker_summary(message)
        stamp("guides", summary)
        with lock:
            recordings["guide_markers"].append(summary)

    def on_hud(message):
        text = str(message.text or "")
        stamp("hud", {"text": text})
        with lock:
            recordings["hud_texts"].append(text)

    subscribers = [
        rospy.Subscriber(PLANNING_DEBUG_TOPIC, String, on_debug,
                         queue_size=200),
        rospy.Subscriber(QUALITY_TOPIC, String, on_quality, queue_size=20),
        rospy.Subscriber(ASTAR_ACTIVE_TOPIC, NavPath, on_astar_path,
                         queue_size=20),
        rospy.Subscriber(GUIDE_ACTIVE_TOPIC, NavPath, on_guide_path,
                         queue_size=20),
        rospy.Subscriber(CANDIDATES_TOPIC, MarkerArray, on_candidates,
                         queue_size=20),
        rospy.Subscriber(GUIDES_TOPIC, MarkerArray, on_guides,
                         queue_size=20),
    ]
    if OverlayText is not None:
        subscribers.append(
            rospy.Subscriber(HUD_TOPIC, OverlayText, on_hud, queue_size=5))

    report = {
        "schema": "task02.step_f.live_capture.v1",
        "label": args.label,
        "ros_master_uri": master,
        "reset_requested": list(reset),
        "goal_requested": list(goal),
        "checks": {},
        "errors": [],
    }

    def note(name, ok, detail):
        report["checks"][name] = {"ok": bool(ok), "detail": json_safe(detail)}

    try:
        params = read_planner_params()
        report["planner_params"] = params
        node_names = rospy.get_node_uri()
        report["capture_node_uri"] = node_names
        smoke.wait_ready(session, 60.0)
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not session.maps:
            rospy.sleep(0.05)
        note("map_received", bool(session.maps),
             "no /map message" if not session.maps else
             runtime_map_summary(session.maps[-1][1]))
        if session.maps:
            report["runtime_map"] = runtime_map_summary(
                session.maps[-1][1])
        smoke.screenshot(images / "00_idle.png")
        smoke.confirm_reset(session, reset, report["checks"], "step_f")
        smoke.screenshot(images / "01_reset.png")
        request_count = session.request_count()
        goal_published_at = time.monotonic()
        smoke.publish_goal(*goal)
        request = session.wait_request_after(request_count, 10.0)
        request_document = smoke.raw_request_document(request)
        report["request"] = request_document
        plan_id = int(request.plan_id)
        note("request_plan_id", plan_id > 0, plan_id)
        admitted = session.wait_job(
            lambda document: document.get("plan_id") == plan_id
            and document.get("state") == "PLANNING",
            20.0, "admitted PLANNING job")
        report["admitted_job"] = admitted

        captures = []

        def screenshot_loop():
            offsets = [0.8, 2.0, 3.5, 5.0, 7.0, 9.5, 13.0, 17.0, 22.0,
                       28.0, 34.0]
            for index, offset in enumerate(offsets):
                if offset > args.screenshot_seconds:
                    break
                wait = goal_published_at + offset - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                path = images / "live_{:02d}_{:04.1f}s.png".format(
                    index + 1, offset)
                captures.append({
                    "label": "live_{:02d}".format(index + 1),
                    "offset_s": offset,
                    "path": str(path),
                    "result": smoke.screenshot(path)})

        thread = threading.Thread(target=screenshot_loop,
                                  name="step-f-screenshots", daemon=True)
        thread.start()

        terminal = None
        deadline = time.monotonic() + args.planning_timeout
        while time.monotonic() < deadline:
            with session.lock:
                current = [document for _, document in session.job_states
                           if document.get("plan_id") == plan_id]
            if current and current[-1].get("state") in ("HOLD_END", "FAILED"):
                terminal = current[-1]
                break
            rospy.sleep(0.05)
        if terminal is None:
            raise RuntimeError(
                "timeout waiting for terminal state of plan {}".format(
                    plan_id))
        note("terminal_state", True,
             {"state": terminal.get("state"),
              "reason": terminal.get("reason")})
        # Allow the latched status/quality/display publishers to settle.
        rospy.sleep(1.5)
        thread.join(timeout=5.0)
        terminal_image = smoke.screenshot(images / "02_terminal.png")
        zoom_image = zoomed_screenshot(images / "03_terminal_zoom.png")
        report["terminal_images"] = {
            "terminal": terminal_image, "zoom": zoom_image}

        report["job_history"] = [
            {"t": round(received - goal_published_at, 6),
             "state": document.get("state"),
             "reason": document.get("reason"),
             "detail": document.get("detail"),
             "plan_id": document.get("plan_id"),
             "model_sha256": document.get("model_sha256"),
             "initial_source": document.get("initial_source"),
             "initial_context_id": document.get("initial_context_id"),
             "virtual_a_goal_policy": document.get("virtual_a_goal_policy")}
            for received, document in session.job_history(plan_id)]
        report["statuses"] = [
            {"t": round(received - goal_published_at, 6),
             "event": document.get("event"),
             "pipeline_status": document.get("pipeline_status"),
             "status": document.get("status"),
             "reason": document.get("reason"),
             "feasible": document.get("feasible"),
             "executable": document.get("executable"),
             "trajectory_published": document.get("trajectory_published"),
             "model_sha256": document.get("model_sha256"),
             "first_failure_layer": (
                 (document.get("virtual_result") or {})
                 .get("first_failure_layer"))}
            for received, document in session.statuses
            if document.get("plan_id") in (None, plan_id)][-40:]

        envelopes = []
        for _received, message in session.envelope_records:
            envelopes.append({
                "plan_id": int(message.plan_id),
                "formation_id": message.formation_id,
                "backend": message.backend,
                "semantic_version": message.semantic_version,
                "model_version": message.model_version,
                "model_sha256": message.model_sha256,
                "model_json_length": len(message.model_json or ""),
                "sample_count": len(message.trajectory.times_from_start),
                "executable": bool(message.model_json)
                and len(message.trajectory.times_from_start) > 0,
            })
        report["envelopes"] = envelopes
        executable = [entry for entry in envelopes
                      if entry["plan_id"] == plan_id
                      and entry["executable"]]
        report["executable_envelope_count"] = len(executable)
        report["non_executable_envelopes"] = [
            entry for entry in envelopes
            if entry["plan_id"] == plan_id and not entry["executable"]]
        success = terminal.get("state") == "HOLD_END"
        note("success_envelope_count",
             (success and len(executable) == 1) or
             (not success and len(executable) == 0),
             {"success": success, "count": len(executable)})

        with lock:
            debug_documents = list(recordings["planning_debug"])
            quality_reports = list(recordings["quality_reports"])
            astar_paths = list(recordings["astar_paths"])
            guide_paths = list(recordings["guide_paths"])
            candidate_markers = list(recordings["candidate_markers"])
            guide_markers = list(recordings["guide_markers"])
            hud_texts = list(recordings["hud_texts"])

        # Persist every planning_debug document (bounded/real, not rebuilt).
        with (run_dir / "planning_debug.jsonl").open(
                "w", encoding="utf-8") as handle:
            for document in debug_documents:
                handle.write(json.dumps(
                    json_safe(document), sort_keys=True) + "\n")
        report["planning_debug_events"] = [
            {
                "stage": document.get("stage"),
                "state": document.get("state"),
                "stage_index": document.get("stage_index"),
                "route_index": document.get("route_index"),
                "route_count": document.get("route_count"),
                "round_index": document.get("round_index"),
                "reason": document.get("reason"),
                "plan_id": document.get("plan_id"),
                "generation": document.get("generation"),
                "model_sha256": document.get("model_sha256"),
                "request_sha256": document.get("request_sha256"),
                "grid_sha256": document.get("grid_sha256"),
                "planning_context_sha256": document.get(
                    "planning_context_sha256"),
                "document_keys": sorted(document.keys()),
                "metrics": document.get("metrics"),
                "corridor_chain_digests": [
                    entry.get("chain_digest")
                    for entry in document.get("corridors") or ()
                    if isinstance(entry, dict)],
                "portal_diagnostics": [
                    entry.get("portal_diagnostics")
                    for entry in document.get("corridors") or ()
                    if isinstance(entry, dict)],
            }
            for document in debug_documents]

        if quality_reports:
            final_quality = quality_reports[-1]
            (run_dir / "quality_report.json").write_text(
                json.dumps(json_safe(final_quality), indent=1,
                           sort_keys=True) + "\n", encoding="utf-8")
            report["quality_summary"] = quality_report_summary(final_quality)
        else:
            note("quality_report_received", False, "no quality report")
        if astar_paths:
            (run_dir / "path_astar_active.json").write_text(
                json.dumps(astar_paths[-1], indent=1, sort_keys=True) + "\n",
                encoding="utf-8")
        if guide_paths:
            (run_dir / "path_portal_guide_active.json").write_text(
                json.dumps(guide_paths[-1], indent=1, sort_keys=True) + "\n",
                encoding="utf-8")
        if candidate_markers:
            (run_dir / "markers_planning_candidates.json").write_text(
                json.dumps(candidate_markers[-1], indent=1,
                           sort_keys=True) + "\n", encoding="utf-8")
        if guide_markers:
            (run_dir / "markers_guides.json").write_text(
                json.dumps(guide_markers[-1], indent=1,
                           sort_keys=True) + "\n", encoding="utf-8")
        (run_dir / "markers_sequence.json").write_text(
            json.dumps({"candidates": candidate_markers,
                        "guides": guide_markers},
                       indent=1, sort_keys=True) + "\n",
            encoding="utf-8")
        (run_dir / "hud_texts.json").write_text(
            json.dumps(hud_texts[-4:], indent=1) + "\n", encoding="utf-8")
        report["received_counts"] = {
            "planning_debug": len(debug_documents),
            "quality_report": len(quality_reports),
            "astar_path": len(astar_paths),
            "guide_path": len(guide_paths),
            "candidate_markers": len(candidate_markers),
            "guide_markers": len(guide_markers),
            "hud": len(hud_texts),
        }
        report["captures"] = captures
        report["screenshots_ok"] = bool(captures) and all(
            capture["result"].get("ok") for capture in captures)
        if candidate_markers:
            note("candidate_markers_no_certified",
                 not candidate_markers[-1]["contains_certified_text"],
                 candidate_markers[-1]["texts"][:20])
        if guide_markers:
            # Step E diagnostics are the new three namespaces only; the
            # historical A*/guide reference labels never claim certification.
            diagnostic_namespaces = (
                "formation/planning/portal_window",
                "formation/planning/map_collision_sample",
                "formation/planning/coverage_gap")
            diagnostic_texts = [
                text for text in guide_markers[-1]["texts"]
                if text.split(":", 1)[0] in diagnostic_namespaces]
            note("guide_diagnostics_no_certified",
                 not any("CERTIFIED" in text.upper()
                         for text in diagnostic_texts),
                 diagnostic_texts[:20])
            note("guide_diagnostics_reported",
                 any(namespace in guide_markers[-1]["namespace_counts"]
                     for namespace in diagnostic_namespaces) or
                 any(entry.get("counts", {}).get(
                         "map_collision_total", 0) == 0
                     for document in debug_documents
                     for corridor in document.get("corridors") or ()
                     if isinstance(corridor, dict)
                     for entry in (corridor.get("portal_diagnostics"),)
                     if isinstance(entry, dict)),
                 {"namespaces": guide_markers[-1]["namespace_counts"],
                  "texts": diagnostic_texts[:20]})
    except Exception as exc:  # record the real failure and exit non-zero
        report["errors"].append(
            "{}: {}".format(type(exc).__name__, exc))
        try:
            smoke.screenshot(images / "error.png")
        except Exception:
            pass
    finally:
        try:
            report["planner_params_final"] = read_planner_params()
        except Exception:
            pass
        (run_dir / "capture_summary.json").write_text(
            json.dumps(json_safe(report), indent=1, sort_keys=True) + "\n",
            encoding="utf-8")
        for subscriber in subscribers:
            try:
                subscriber.unregister()
            except Exception:
                pass
        try:
            session.shutdown()
        except Exception:
            pass
    return 0 if not report["errors"] else 1


def quality_report_summary(document):
    """Identity + route/guide/round evidence actually present in the report."""
    virtual = document.get("virtual_result") or {}
    search = virtual.get("search") or {}
    corridor = search.get("corridor") or {}
    rounds = corridor.get("rounds") or []
    routes = search.get("routes") or virtual.get("routes") or []
    solver_rounds = search.get("rounds") or []
    summary = {
        "plan_id": document.get("plan_id"),
        "request_sha256": document.get("request_sha256"),
        "planning_context_sha256": document.get("planning_context_sha256"),
        "grid_sha256": document.get("grid_sha256"),
        "model_sha256": document.get("model_sha256"),
        "status": virtual.get("status"),
        "reason": virtual.get("reason"),
        "first_failure_layer": virtual.get("first_failure_layer"),
        "route_count": search.get("route_count"),
        "routes": [
            {"length_m": route.get("length_m"),
             "point_count": len(route.get("points") or []),
             "source": route.get("source")}
            for route in routes],
        "corridor_rounds": [],
        "solver_rounds": [],
    }
    for entry in rounds:
        if not isinstance(entry, dict):
            continue
        selection = entry.get("guide_selection") or {}
        assessment = entry.get("formation_assessment") or {}
        summary["corridor_rounds"].append({
            "round_index": entry.get("round_index"),
            "route_index": entry.get("route_index"),
            "chain_digest": entry.get("chain_digest"),
            "guide_source": (entry.get("guide") or {}).get("source"),
            "guide_point_count": len(
                (entry.get("guide") or {}).get("points") or []),
            "guide_length_m": (entry.get("guide") or {}).get("length_m"),
            "allocation": entry.get("allocation"),
            "guide_selection": selection,
            "assessment": {
                "schema": assessment.get("schema"),
                "candidate_id": assessment.get("candidate_id"),
                "route_index": assessment.get("route_index"),
                "round_index": assessment.get("round_index"),
                "map_sample_status": assessment.get("map_sample_status"),
                "coverage_sample_status":
                    assessment.get("coverage_sample_status"),
                "candidate_certification":
                    assessment.get("candidate_certification"),
                "continuously_certified":
                    assessment.get("continuously_certified"),
                "formation_obb_hard_constraints":
                    assessment.get("formation_obb_hard_constraints"),
                "station_count": assessment.get("station_count"),
                "map_sample_counts": assessment.get("map_sample_counts"),
                "coverage_counts": {
                    str(vehicle): (value or {}).get("counts")
                    for vehicle, value in (
                        assessment.get("coverage_vehicles") or {}).items()},
                "transition_windows": [
                    {"window_index": window.get("window_index"),
                     "status": window.get("status"),
                     "map_sample_status": window.get("map_sample_status")}
                    for window in assessment.get("transition_windows") or []
                    if isinstance(window, dict)],
            },
        })
    for entry in solver_rounds:
        if not isinstance(entry, dict):
            continue
        summary["solver_rounds"].append({
            "round_index": entry.get("round_index"),
            "route_index": entry.get("route_index"),
            "corridor_round_index": entry.get("corridor_round_index"),
            "guide_length_m": entry.get("guide_length_m"),
            "guide_turn_rad": entry.get("guide_turn_rad"),
            "heading_branch": entry.get("heading_branch"),
            "winding_k": entry.get("winding_k"),
            "winding_candidate_rank": entry.get("winding_candidate_rank"),
            "S_solution_m": entry.get("S_solution_m"),
            "geometry_status": entry.get("geometry_status"),
        })
    return summary


if __name__ == "__main__":
    sys.exit(main())