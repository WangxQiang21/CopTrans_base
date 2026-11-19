#!/usr/bin/env python3
"""Live U-turn evidence driver for the display-only RViz change.

This is an evidence driver, not production code.  It uses the existing G5
smoke helpers only: /initialpose and /move_base_simple/goal are the sole
inputs to the planner.  The 600003/600004 start-goal pair is copied from
evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z/SUMMARY.md.
"""
import json
import sys
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests"))

import rospy
import ros_virtual_a_smoke as smoke


EVIDENCE = Path(__file__).resolve().parent
IMAGES = EVIDENCE / "images"
SUCCESS_START = (10.841, 12.071, 0.000)
SUCCESS_GOAL = (10.153, 2.076, -3.126)
FAILURE_START = (-3.625, 11.771, -0.022)
FAILURE_GOAL = (-2.490, 2.309, 3.102)


def main():
    for directory in ("envelopes", "requests", "display", "images"):
        (EVIDENCE / directory).mkdir(parents=True, exist_ok=True)
    rospy.init_node("rviz_uturn_live_evidence", anonymous=True)
    session = smoke.Session()
    checks = {}
    smoke.wait_ready(session)
    args = types.SimpleNamespace(planning_timeout=180.0)

    started = time.time()
    first = smoke.run_verified_plan(
        session, None, SUCCESS_GOAL, checks, "uturn_success", args,
        EVIDENCE, reset=SUCCESS_START, capture_dir=IMAGES)
    reset_job = smoke.confirm_reset(
        session, FAILURE_START, checks, "uturn_failure")
    before = session.request_count()
    smoke.publish_goal(*FAILURE_GOAL)
    request = session.wait_request_after(before, 5.0)
    failed_id = int(request.plan_id)
    admitted = session.wait_job(
        lambda d: d.get("plan_id") == failed_id and
        d.get("state") in ("PLANNING", "FAILED"), 10.0,
        "U-turn failure plan admitted")
    failed = session.wait_job(
        lambda d: d.get("plan_id") == failed_id and d.get("state") == "FAILED",
        args.planning_timeout, "U-turn failure terminal FAILED")
    # Let the atomic DELETEALL/hold marker be delivered before the screenshot.
    rospy.sleep(1.0)
    failure_capture = smoke.screenshot(IMAGES / "uturn_failure_cleared.png")
    with session.lock:
        failed_envelope = session.envelopes.get(failed_id)
        playback = session.playback[-20:]
    envelope_summary = None
    if failed_envelope is not None:
        # FormationPlaybackPlan has no rejection_reason/status fields; the
        # observable failure is the empty model/envelope plus the planner and
        # playback status topics.  Keep this evidence serializer defensive so
        # a future message revision cannot abort after the screenshots exist.
        envelope_summary = {
            "plan_id": int(failed_envelope.plan_id),
            "model_json_length": len(failed_envelope.model_json),
            "model_sha256": str(failed_envelope.model_sha256),
            "certification": str(failed_envelope.certification),
            "sample_count": len(
                failed_envelope.trajectory.times_from_start),
            "reason": getattr(failed_envelope, "rejection_reason", None),
            "status": getattr(failed_envelope, "status", None),
        }
    document = {
        "schema": "rviz.uturn.live.evidence.v1",
        "started_epoch_s": started,
        "finished_epoch_s": time.time(),
        "success_start": list(SUCCESS_START),
        "success_goal": list(SUCCESS_GOAL),
        "failure_start": list(FAILURE_START),
        "failure_goal": list(FAILURE_GOAL),
        "success_plan_id": first["plan_id"],
        "success_hold_state": first["hold"].get("state"),
        "success_terminal_qf": first["terminal_qf"],
        "failure_plan_id": failed_id,
        "failure_reset_context": reset_job.get("initial_context_id"),
        "failure_admission": admitted.get("admission"),
        "failure_job": failed,
        "failure_envelope": envelope_summary,
        "failure_capture": failure_capture,
        "failure_playback_tail": [dict(d) for _, d in playback],
        "checks": checks,
        "failed_checks": [k for k, v in checks.items() if not v.get("ok")],
    }
    (EVIDENCE / "report.json").write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "success_plan_id": first["plan_id"],
        "failure_plan_id": failed_id,
        "failure_status": failed.get("status"),
        "failure_reason": failed.get("reason"),
        "failure_layer": failed.get("first_failure_layer"),
        "failed_checks": document["failed_checks"],
        "capture": failure_capture,
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
