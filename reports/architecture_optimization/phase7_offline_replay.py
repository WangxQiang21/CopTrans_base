#!/usr/bin/env python3
"""Run the offline local-handover prototype with 120 wall-clock 10 Hz polls."""

import argparse
from dataclasses import asdict, replace
import json
from pathlib import Path
import runpy
from time import monotonic_ns, sleep

import numpy as np
import yaml

from fwis_backends.multi_4wis.local_replan_protocol import (
    Acknowledgement,
    CertifiedPlan,
    LocalReplanCoordinator,
    MapRevision,
    stopped_window,
)
from fwis_backends.multi_4wis.uniform_pipeline import plan_uniform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    make_request = runpy.run_path(
        "ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_scene_matrix.py")["make_request"]
    offsets = yaml.safe_load(Path("ros2_ws/src/fwis_bringup/config/experiments.yaml").read_text())
    request = make_request(Path("ros2_ws/generated/scenes/factory_indoor_01"), "short_turn", offsets)
    outcome = plan_uniform(request)
    revision = MapRevision.observe(1, request)
    active = CertifiedPlan.capture(outcome, revision, monotonic_ns())
    window = stopped_window(
        active, monotonic_ns(), min_lead_ns=250_000_000, switch_dwell_ns=750_000_000,
        max_path_m=2.0, max_motion_s=35.0, max_lookahead_ns=20_000_000_000)
    assert window is not None

    coordinator = LocalReplanCoordinator(active)
    points = []
    proposal = None
    ack_states = []
    try:
        coordinator.begin(request, window, plan_id=request.plan_id + 1, computation_budget_s=2.0, max_expansions=40)
        start = monotonic_ns()
        for index in range(120):
            target = start + index * 100_000_000
            wait = (target - monotonic_ns()) / 1e9
            if wait > 0.0:
                sleep(wait)
            if index == 110:
                coordinator.observe_map(replace(revision, number=2))
            now = monotonic_ns()
            decision = coordinator.poll()
            duration = monotonic_ns() - now
            row = {
                "index": index, "time_from_epoch_ns": now - active.epoch_ns,
                "lateness_ns": now - target, "poll_compute_ns": duration,
                "action": decision.action, "reason": decision.reason,
            }
            if decision.preparation:
                row["plan_id"] = decision.preparation.plan_id
                row["activation_epoch_ns"] = decision.preparation.epoch_ns
            if (decision.action == "PREPARE" and now >= window.stop_ns and not ack_states):
                proposal = decision.preparation
                wrong = replace(proposal, epoch_ns=proposal.epoch_ns - 1)
                ack_states.append(coordinator.acknowledge(Acknowledgement(1, wrong, now, True)))
                due = monotonic_ns()
                for vehicle in (1, 2, 3):
                    ack_states.append(coordinator.acknowledge(Acknowledgement(vehicle, proposal, due, True)))
            points.append(row)
    finally:
        coordinator.close()

    assert ack_states == [False, True, True, True]
    switches = [row for row in points if row["action"] == "SWITCH"]
    assert len(switches) == 1 and switches[0]["activation_epoch_ns"] == window.epoch_ns
    assert any(row["action"] == "EXECUTE_ACTIVE" for row in points)
    assert all(row["action"] == "GROUP_STOP" and row["reason"] == "MAP_REVISION_CHANGED"
               for row in points[110:])
    metrics = {
        key: {f"p{q}_ms": float(np.percentile([row[key] / 1e6 for row in points], q))
              for q in (50, 95, 99, 100)}
        for key in ("lateness_ns", "poll_compute_ns")
    }
    output = {
        "mode": "offline protocol with real certified planning, no hardware commands",
        "nominal_poll_hz": 10,
        "global_plan_id": active.plan.plan_id,
        "global_model_sha256": active.model_sha256,
        "global_request_sha256": active.request_sha256,
        "local_preparation": asdict(proposal),
        "stationary_interval_ns": [window.stop_ns, window.stop_end_ns],
        "max_local_path_m": window.max_path_m,
        "max_local_motion_s": window.max_motion_s,
        "wrong_ack_rejected": not ack_states[0],
        "all_three_matching_acknowledgements": all(ack_states[1:]),
        "group_stop_on_revision_change": True,
        "metrics": metrics,
        "polls": points,
    }
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(json.dumps(metrics))


if __name__ == "__main__":
    main()
