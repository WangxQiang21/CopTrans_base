#!/usr/bin/env python3
"""TASK03 Step 6 live ROS/RViz capture driver (task-private master only).

One run drives one already-running ``./run_demo.sh`` session on the
task-private master.  It records only what the live nodes actually publish;
it never retries a failed request and never publishes a reference trajectory
or an envelope.

Modes
-----
``union``    the run must publish exactly one executable envelope whose
             admitted planner config declares ``three_vehicle_union_v1``.
             The driver rebuilds the expected request/planning context from
             the live topics, rebuilds the retained template/chain/certificate
             binding from the published quality report, and calls the real
             ``verify_virtual_a_playback_plan`` with ``expected_formation``.
             It also checks the live result evidence: OBB g count > 0, the
             continuous certificate VERIFIED, geometry/dynamics/original-map
             collision audits passed.
``f_only``   the historical comparison: success, one executable envelope,
             no formation binding in the admitted context and no union keys
             in the published result.
``negative``  E04: F is feasible but vehicles 2/3 hit a wall.  The run must
             fail, publish no executable envelope, leave no playable
             trajectory and keep the display clear of a stale certified path.

usage: ros_step6_capture.py --run-dir DIR --label NAME --mode union \
       --reset X,Y,THETA --goal X,Y,THETA [--planning-timeout S]
"""
import argparse
import json
import math
import os
import subprocess
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
PROTECTED_PORTS = (11311, 11312, 11320)
UNION_SCOPE = "three_vehicle_union_v1"
F_ONLY_SCOPE = "f_only"
DIAGNOSTIC_NAMESPACES = (
    "formation/planning/portal_window",
    "formation/planning/map_collision_sample",
    "formation/planning/coverage_gap",
)


def json_safe(value):
    return smoke.json_safe(value)


def hud_audit_state(texts):
    """Executable-audit claim read from the last HUD overlay text.

    The HUD calls a published, independently audited plan ``audit FEASIBLE``
    (or ``NO_MOTION``); the word ``CERTIFIED`` belongs to the final candidate
    marker layer, never to the HUD.  A pending or geometry-only audit shows
    ``AUDIT_PENDING``/``GEOMETRY_ONLY`` instead.
    """
    text = texts[-1] if texts else ""
    return {
        "executable_audit": ("audit FEASIBLE" in text or
                             "audit NO_MOTION" in text),
        "geometry_pass": "geometry=PASS" in text,
        "dynamics_pass": "dynamics=PASS" in text,
        "collision_pass": "collision=PASS" in text,
    }


def marker_summary(message):
    """Compact, bounded view of one MarkerArray."""
    namespace_counts = {}
    namespace_types = {}
    texts = []
    colors = {}
    for marker in message.markers:
        namespace = str(marker.ns)
        namespace_counts[namespace] = namespace_counts.get(namespace, 0) + 1
        namespace_types.setdefault(namespace, set()).add(marker.type)
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
        "colors": {name: values[0] for name, values in colors.items()},
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


def parse_triple(value):
    parts = [float(item) for item in value.split(",")]
    if len(parts) != 3:
        raise SystemExit("pose must be x,y,theta")
    return tuple(parts)


def read_planner_params():
    names = (
        "backend", "virtual_a_guidance_mode",
        "virtual_a_corridor_enforcement",
        "virtual_a_formation_corridor_scope",
        "virtual_a_goal_policy",
        "corridor_margin_m", "corridor_audit_min_width_m",
        "corridor_audit_position_margin_m", "intervals", "solver")
    document = {}
    for name in names:
        try:
            document[name] = rospy.get_param(
                "/formation_uniform_planner/" + name, None)
        except Exception as exc:  # rosparam server failure is recorded
            document[name] = "ERROR: {}: {}".format(
                type(exc).__name__, exc)
    try:
        document["planner_config"] = rospy.get_param(
            "/formation_uniform_planner/planner_config", None)
    except Exception as exc:
        document["planner_config"] = "ERROR: {}: {}".format(
            type(exc).__name__, exc)
    return document


def planner_scope(params):
    value = params.get("virtual_a_formation_corridor_scope")
    if value is None:
        config = params.get("planner_config")
        if isinstance(config, dict):
            value = config.get("formation_corridor_scope", F_ONLY_SCOPE)
        elif isinstance(config, str) and config:
            try:
                parsed = json.loads(config)
            except (TypeError, ValueError):
                parsed = None
            if isinstance(parsed, dict):
                value = parsed.get("formation_corridor_scope", F_ONLY_SCOPE)
    return str(value or F_ONLY_SCOPE).strip().lower()


def union_binding(quality):
    """Rebuild the retained binding from the published quality document."""
    virtual = quality.get("virtual_result") or {}
    audit = virtual.get("formation_corridor_audit") or {}
    search = virtual.get("search") or {}
    corridor = search.get("corridor") or {}
    rounds = corridor.get("rounds") or []
    return {
        "scope": UNION_SCOPE,
        "planning_context_sha256": virtual.get("planning_context_sha256"),
        "template": audit.get("template"),
        "chain": rounds[-1].get("chain") if rounds else None,
        "certificate": audit.get("certificate"),
    }


def binding_complete(binding):
    return (
        isinstance(binding, dict)
        and binding.get("scope") == UNION_SCOPE
        and isinstance(binding.get("planning_context_sha256"), str)
        and bool(binding["planning_context_sha256"])
        and isinstance(binding.get("template"), dict)
        and bool(binding["template"])
        and isinstance(binding.get("chain"), dict)
        and bool(binding["chain"])
        and isinstance(binding.get("certificate"), dict)
        and bool(binding["certificate"]))


def envelope_document(message):
    return {
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
        "formation_binding_present": bool(
            str(getattr(message, "formation_binding_json", "") or "")),
        "formation_binding_length": len(
            str(getattr(message, "formation_binding_json", "") or "")),
    }


def executable_envelope_record(records, plan_id):
    """Last captured message for the plan that carries an executable model.

    ``Session.envelopes`` keeps only the latest message per plan; a failed
    run overwrites the executable envelope with the explicit no-envelope
    message, so the executable record must be selected from the full history.
    """
    for _received, message in reversed(records):
        if int(message.plan_id) != plan_id:
            continue
        if message.model_json and message.trajectory.times_from_start:
            return message
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--mode", required=True,
                        choices=("union", "f_only", "negative"))
    parser.add_argument("--reset", required=True)
    parser.add_argument("--goal", required=True)
    parser.add_argument("--planning-timeout", type=float, default=240.0)
    parser.add_argument("--screenshot-seconds", type=float, default=45.0)
    parser.add_argument("--node-name", default="task03_step6_capture")
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
        "schema": "task03.step6.live_capture.v1",
        "label": args.label,
        "mode": args.mode,
        "ros_master_uri": master,
        "reset_requested": list(reset),
        "goal_requested": list(goal),
        "checks": {},
        "errors": [],
        "notes": {},
        "artifacts": {},
    }

    def check(name, ok, detail=""):
        report["checks"][name] = {"ok": bool(ok), "detail": json_safe(detail)}

    def note(name, detail):
        report["notes"][name] = json_safe(detail)

    def read_quality(plan_id, timeout, predicate=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with lock:
                candidates = [document for document in
                              recordings["quality_reports"]
                              if document.get("plan_id") == plan_id
                              and isinstance(document.get("virtual_result"),
                                             dict)]
            if predicate is not None:
                candidates = [document for document in candidates
                              if predicate(document)]
            if candidates:
                return candidates[-1]
            rospy.sleep(0.05)
        return None

    try:
        smoke.wait_ready(session, 60.0)
        # Params are read only after the node has loaded its launch args: a
        # read before wait_ready catches the defaults and reports the wrong
        # scope.
        params = read_planner_params()
        report["planner_params"] = params
        scope = planner_scope(params)
        note("planner_scope", scope)
        expected_scope = UNION_SCOPE if args.mode in ("union", "negative") \
            else F_ONLY_SCOPE
        check("scope_param", scope == expected_scope,
              {"expected": expected_scope, "actual": scope})

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and not session.maps:
            rospy.sleep(0.05)
        check("map_received", bool(session.maps),
              "no /map message" if not session.maps else
              runtime_map_summary(session.maps[-1][1]))
        if session.maps:
            report["runtime_map"] = runtime_map_summary(
                session.maps[-1][1])
        smoke.screenshot(images / "00_idle.png")
        smoke.confirm_reset(session, reset, report["checks"], "step6")
        smoke.screenshot(images / "01_reset.png")

        request_count = session.request_count()
        goal_published_at = time.monotonic()
        smoke.publish_goal(*goal)
        request = session.wait_request_after(request_count, 10.0)
        report["request"] = smoke.raw_request_document(request)
        plan_id = int(request.plan_id)
        check("request_plan_id", plan_id > 0, plan_id)
        admitted = session.wait_job(
            lambda document: document.get("plan_id") == plan_id
            and document.get("state") == "PLANNING", 20.0,
            "admitted PLANNING job")
        report["admitted_job"] = admitted
        context = admitted.get("planning_context") or {}
        context_scope = str(
            (context.get("planner_config") or {}).get(
                "formation_corridor_scope", F_ONLY_SCOPE)).strip().lower()
        check("admitted_context_scope", context_scope == expected_scope,
              {"expected": expected_scope, "actual": context_scope})

        captures = []

        def screenshot_loop():
            offsets = [0.8, 2.0, 3.5, 5.0, 7.0, 9.5, 13.0, 17.0, 22.0,
                       28.0, 34.0, 40.0]
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
                                  name="step6-screenshots", daemon=True)
        thread.start()

        terminal = None
        deadline = time.monotonic() + args.planning_timeout
        while time.monotonic() < deadline:
            with session.lock:
                current = [document for _, document in session.job_states
                           if document.get("plan_id") == plan_id]
            if current and current[-1].get("state") in (
                    "HOLD_END", "FAILED", "PLANNING_TIMEOUT"):
                terminal = current[-1]
                break
            rospy.sleep(0.05)
        if terminal is None:
            raise RuntimeError(
                "timeout waiting for terminal state of plan {}".format(
                    plan_id))
        check("terminal_state", True,
              {"state": terminal.get("state"),
               "reason": terminal.get("reason")})
        rospy.sleep(1.5)
        thread.join(timeout=5.0)
        report["terminal_images"] = {
            "terminal": smoke.screenshot(images / "02_terminal.png"),
            "zoom": smoke.screenshot(images / "03_terminal_zoom.png")}

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
            envelopes.append(envelope_document(message))
        report["envelopes"] = envelopes
        plan_envelopes = [entry for entry in envelopes
                          if entry["plan_id"] == plan_id]
        executable = [entry for entry in plan_envelopes
                      if entry["executable"]]
        report["executable_envelope_count"] = len(executable)
        report["non_executable_envelopes"] = [
            entry for entry in plan_envelopes if not entry["executable"]]

        with lock:
            debug_documents = list(recordings["planning_debug"])
            quality_reports = list(recordings["quality_reports"])
            astar_paths = list(recordings["astar_paths"])
            guide_paths = list(recordings["guide_paths"])
            candidate_markers = list(recordings["candidate_markers"])
            guide_markers = list(recordings["guide_markers"])
            hud_texts = list(recordings["hud_texts"])

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
                "metrics": document.get("metrics"),
            }
            for document in debug_documents]

        final_quality = None
        try:
            final_quality = read_quality(
                plan_id, 10.0,
                predicate=lambda document: (
                    document.get("executable") is True
                    if args.mode != "negative" else True))
        except Exception:
            final_quality = None
        if final_quality is None:
            with lock:
                matching = [document for document in
                            recordings["quality_reports"]
                            if document.get("plan_id") == plan_id]
            final_quality = matching[-1] if matching else None
        if final_quality is not None:
            (run_dir / "quality_report.json").write_text(
                json.dumps(json_safe(final_quality), indent=1,
                           sort_keys=True) + "\n", encoding="utf-8")
            report["artifacts"]["quality_report"] = str(
                run_dir / "quality_report.json")
        else:
            check("quality_report_received", False, "no quality report")

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
            json.dumps(hud_texts[-6:], indent=1) + "\n", encoding="utf-8")
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

        if guide_markers:
            # Guide/sample layers are reference-only: no Guide risk sample
            # may borrow the final path's CERTIFIED word.
            check("guide_markers_no_certified",
                  not guide_markers[-1]["contains_certified_text"],
                  guide_markers[-1]["texts"][:20])
            diagnostic_texts = [
                text for text in guide_markers[-1]["texts"]
                if text.split(":", 1)[0] in DIAGNOSTIC_NAMESPACES]
            check("guide_diagnostics_no_certified",
                  not any("CERTIFIED" in text.upper()
                          for text in diagnostic_texts),
                  diagnostic_texts[:20])
            check("guide_diagnostics_reported",
                  any(namespace in guide_markers[-1]["namespace_counts"]
                      for namespace in DIAGNOSTIC_NAMESPACES) or
                  any(entry.get("counts", {}).get(
                          "map_collision_total", 0) == 0
                      for document in debug_documents
                      for corridor in document.get("corridors") or ()
                      if isinstance(corridor, dict)
                      for entry in (corridor.get("portal_diagnostics"),)
                      if isinstance(entry, dict)),
                  {"namespaces": guide_markers[-1]["namespace_counts"],
                   "texts": diagnostic_texts[:20]})
        report["hud_texts"] = hud_texts[-6:]
        report["hud_contains_certified"] = any(
            "CERTIFIED" in text.upper() for text in hud_texts)
        report["hud_contains_audit_verified"] = any(
            "audit VERIFIED" in text for text in hud_texts)
        note("guide_namespaces", (
            guide_markers[-1]["namespace_counts"] if guide_markers else {}))
        note("candidate_namespaces", (
            candidate_markers[-1]["namespace_counts"]
            if candidate_markers else {}))
        note("marker_namespaces", report.get("marker_namespaces", {}))

        if args.mode in ("union", "f_only"):
            success = terminal.get("state") == "HOLD_END"
            check("success_terminal", success,
                  {"state": terminal.get("state"),
                   "reason": terminal.get("reason")})
            check("success_envelope", success and len(executable) == 1,
                  {"success": success, "count": len(executable)})
            published_documents = [
                document for document in debug_documents
                if document.get("plan_id") == plan_id
                and document.get("stage") == "PUBLISHED"]
            # A CERTIFIED candidate marker is the final accepted path's
            # claim and may exist only after the PUBLISHED stage, i.e. after
            # the atomic envelope was actually published.  It never stands
            # in for a Guide/sample-risk certification.
            check("candidate_certified_matches_published",
                  bool(candidate_markers)
                  and candidate_markers[-1]["contains_certified_text"]
                  == bool(published_documents),
                  {"candidate_certified": (
                      candidate_markers[-1]["contains_certified_text"]
                      if candidate_markers else None),
                   "published_documents": len(published_documents)})
            virtual = (final_quality or {}).get("virtual_result") or {}
            check("result_executable",
                  virtual.get("executable") is True, virtual.get("executable"))
            check("result_status", virtual.get("status") in
                  ("SUCCESS", "NO_MOTION"), virtual.get("status"))
            audits = virtual.get("audits") or {}
            for layer in ("geometry", "dynamics", "collision"):
                report_layer = audits.get(layer) or {}
                check("audit_" + layer,
                      report_layer.get("passed") is True,
                      {"passed": report_layer.get("passed"),
                       "reason": report_layer.get("reason"),
                       "scope": report_layer.get("scope"),
                       "checks": report_layer.get("check_count")})
            hud = hud_audit_state(hud_texts)
            check("hud_audit_executable",
                  all(hud.values()),
                  dict(hud, last_text=hud_texts[-1] if hud_texts else None))
            if args.mode == "union":
                check("result_scope_union",
                      virtual.get("formation_corridor_scope") == UNION_SCOPE,
                      virtual.get("formation_corridor_scope"))
                check("result_obb_count_positive",
                      isinstance(virtual.get(
                          "formation_obb_constraint_count"), int)
                      and int(virtual["formation_obb_constraint_count"]) > 0,
                      virtual.get("formation_obb_constraint_count"))
                certificate = (virtual.get("formation_corridor_audit") or {}
                               ).get("certificate") or {}
                check("certificate_verified",
                      certificate.get("status") == "VERIFIED",
                      {"status": certificate.get("status"),
                       "sha256": str(certificate.get(
                           "certificate_sha256", ""))[:16]})
                check("certificate_max_residual_zero",
                      certificate.get("max_outward_residual_m") is not None
                      and float(certificate["max_outward_residual_m"]) <= 0.0,
                      certificate.get("max_outward_residual_m"))
                binding = union_binding(final_quality or {})
                note("union_binding_summary", {
                    "complete": binding_complete(binding),
                    "planning_context_sha256": str(binding.get(
                        "planning_context_sha256", ""))[:16],
                    "template_id": str((binding.get("template") or {}).get(
                        "template_id", ""))[:16],
                    "chain_digest": str((binding.get("chain") or {}).get(
                        "digest", ""))[:16],
                    "certificate_sha256": str(
                        (binding.get("certificate") or {}).get(
                            "certificate_sha256", ""))[:16],
                })
                check("union_binding_complete", binding_complete(binding),
                      report["notes"].get("union_binding_summary"))
                if binding_complete(binding) and executable:
                    message = executable_envelope_record(
                        session.envelope_records, plan_id)
                    _verify_envelope(
                        request, message, admitted, plan_id, binding, report,
                        check, note)
            else:
                check("f_only_no_union_scope_key",
                      "formation_corridor_scope" not in virtual,
                      sorted(virtual.keys()))
                check("f_only_no_certificate_key",
                      "formation_certificate" not in virtual,
                      sorted(virtual.keys()))
                corridors = (virtual.get("search") or {}).get(
                    "corridor") or {}
                rounds = corridors.get("rounds") or []
                last = rounds[-1] if rounds else {}
                check("f_only_no_formation_templates",
                      "formation_templates" not in last,
                      sorted(last.keys()))
                check("f_only_context_has_no_scope",
                      "formation_corridor_scope" not in (
                          (context.get("planner_config") or {})),
                      sorted((context.get("planner_config") or {}).keys()))
                if executable:
                    message = executable_envelope_record(
                        session.envelope_records, plan_id)
                    _verify_envelope(
                        request, message, admitted, plan_id, None, report,
                        check, note)
        else:
            check("negative_terminal_failed",
                  terminal.get("state") == "FAILED",
                  {"state": terminal.get("state"),
                   "reason": terminal.get("reason")})
            check("negative_no_executable_envelope",
                  len(executable) == 0,
                  {"count": len(executable)})
            virtual = (final_quality or {}).get("virtual_result") or {}
            check("negative_result_not_executable",
                  virtual.get("executable") is False,
                  virtual.get("executable"))
            audit = virtual.get("formation_corridor_audit") or {}
            note("negative_evidence", {
                "status": virtual.get("status"),
                "reason": virtual.get("reason"),
                "detail": str(virtual.get("detail", ""))[:300],
                "formation_corridor_audit": {
                    key: audit.get(key) for key in (
                        "scope", "status", "reason", "detail")},
                "first_failure_layer": virtual.get("first_failure_layer"),
            })
            check("negative_scope_union",
                  virtual.get("formation_corridor_scope") == UNION_SCOPE,
                  virtual.get("formation_corridor_scope"))
            check("negative_status",
                  virtual.get("status") == "NO_ACCEPTED_CANDIDATE",
                  virtual.get("status"))
            check("negative_reasons",
                  virtual.get("reason") in (
                      "FORMATION_CORRIDOR_UNCONFIRMED",
                      "COVER_TEMPLATE_INSUFFICIENT",
                      "NO_ACCEPTED_CANDIDATE",
                      "GEOMETRY_REJECTED", "COLLISION_FAILED",
                      "BUDGET_EXHAUSTED"),
                  virtual.get("reason"))
            check("negative_no_certificate",
                  not bool(audit.get("certificate")),
                  bool(audit.get("certificate")))
            latest_marker = session.latest_marker()
            cleared = bool(latest_marker is not None and
                           latest_marker.delete_all)
            note("latest_marker_clear", cleared)
            playback_for_plan = [
                document for _received, document in
                session.playback_history(plan_id)]
            stale = [
                document for document in playback_for_plan
                if document.get("state") in ("PLAYING", "HOLD_END")]
            check("negative_no_stale_playback",
                  len(stale) == 0,
                  {"events": len(playback_for_plan),
                   "stale_states": [document.get("state")
                                    for document in stale]})
            hud = hud_audit_state(hud_texts)
            hud_ok = (not report["hud_contains_certified"]
                      and not hud["executable_audit"])
            check("negative_hud_not_certified", hud_ok,
                  dict(hud, last_text=hud_texts[-1] if hud_texts else None))
            if candidate_markers:
                check("negative_candidates_not_certified",
                      not candidate_markers[-1]["contains_certified_text"],
                      candidate_markers[-1]["texts"][:20])
            check("negative_no_playback_plan",
                  not any(entry["executable"] for entry in envelopes),
                  [entry["plan_id"] for entry in envelopes
                   if entry["executable"]])
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
        try:
            completed = subprocess.run(
                ["rostopic", "list", "-v"], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, timeout=10.0)
            report["topics"] = completed.stdout.decode(
                "utf-8", "replace").splitlines()
        except Exception as exc:
            report["topics_error"] = "{}: {}".format(
                type(exc).__name__, exc)
        try:
            completed = subprocess.run(
                ["rosnode", "list"], stdout=subprocess.PIPE,
                stderr=subprocess.PIPE, timeout=10.0)
            report["nodes"] = completed.stdout.decode(
                "utf-8", "replace").splitlines()
        except Exception as exc:
            report["nodes_error"] = "{}: {}".format(
                type(exc).__name__, exc)
        report["passed"] = bool(report["checks"]) and all(
            entry["ok"] for entry in report["checks"].values()) and \
            not report["errors"]
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
    return 0 if report.get("passed") else 1


def _verify_envelope(request_message, message, admitted, plan_id, binding,
                     report, check, note):
    """Rebuild the expected request/context and verify the live envelope."""
    if message is None:
        check("envelope_present", False, "no envelope message captured")
        return
    binding_text = str(
        getattr(message, "formation_binding_json", "") or "").strip()
    if binding is None:
        check("envelope_no_formation_binding", not binding_text,
              {"length": len(binding_text)})
    else:
        try:
            carried = json.loads(binding_text)
        except (TypeError, ValueError):
            carried = None
        check("envelope_carries_quality_binding",
              isinstance(carried, dict) and carried == binding,
              {"carried_object": isinstance(carried, dict),
               "length": len(binding_text)})
    try:
        raw = smoke.raw_request_document(request_message)
        initial = admitted.get("initial_context") or {}
        context = admitted.get("planning_context") or {}
        request = smoke.virtual_a_request_from_raw_request(
            raw, initial_source=initial.get("source"),
            initial_context_id=initial.get("context_id"))
        fingerprint = smoke.request_fingerprint(request)
        geometry_payload = (context.get("timing_config") or {}).get(
            "geometry_audit_config")
        geometry_config = smoke.geometry_audit_config_from_dict(
            geometry_payload)
        fields = smoke.fields_from_timed_message(message.trajectory)
        if binding is None:
            trajectory, plan, digest = smoke.verify_virtual_a_playback_plan(
                plan_id=int(message.plan_id),
                formation_id=message.formation_id,
                backend=message.backend,
                semantic_version=message.semantic_version,
                model_version=message.model_version,
                model_sha256=message.model_sha256,
                model_json=message.model_json,
                certification=message.certification,
                trajectory_fields=fields,
                expected_request=request,
                expected_context=context,
                expected_grid_sha256=admitted.get("grid_sha256"),
                geometry_audit_config=geometry_config)
        else:
            trajectory, plan, digest = smoke.verify_virtual_a_playback_plan(
                plan_id=int(message.plan_id),
                formation_id=message.formation_id,
                backend=message.backend,
                semantic_version=message.semantic_version,
                model_version=message.model_version,
                model_sha256=message.model_sha256,
                model_json=message.model_json,
                certification=message.certification,
                trajectory_fields=fields,
                expected_request=request,
                expected_context=context,
                expected_grid_sha256=admitted.get("grid_sha256"),
                expected_formation=binding,
                geometry_audit_config=geometry_config)
        check("envelope_verifier", True, {
            "plan_id": int(message.plan_id),
            "samples": len(fields["times_from_start"]),
            "model_sha256": str(digest)[:16],
            "expected_formation": binding is not None,
        })
        check("envelope_digest", digest == message.model_sha256,
              {"verified": str(digest), "message": message.model_sha256})
        check("envelope_request_identity", plan.request_sha256 == fingerprint,
              plan.request_sha256)
    except Exception as exc:
        check("envelope_verifier", False,
              "{}: {}".format(type(exc).__name__, exc))
        note("envelope_verifier_error", str(exc))


if __name__ == "__main__":
    sys.exit(main())