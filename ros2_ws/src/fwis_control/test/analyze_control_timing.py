#!/usr/bin/env python3
"""Join native tracker, ROS middleware and Gazebo body timing without modifying messages."""

from collections import Counter, defaultdict
import argparse
from bisect import bisect_left, bisect_right
import csv
import json
import math
from pathlib import Path


SECOND_NS = 1_000_000_000


def stats_ns(values):
    values = sorted(values)
    def quantile(fraction):
        return values[max(0, math.ceil(len(values) * fraction) - 1)] / 1e6
    return dict(samples=len(values), **({
        "p50_ms": quantile(.50), "p95_ms": quantile(.95), "p99_ms": quantile(.99),
        "max_ms": quantile(1.)} if values else {}))


def pair_commands(controller_rows, applied_rows, window_ns=SECOND_NS//2):
    """Match only unique causal payloads in both directions within the explicit window."""
    source = defaultdict(list)
    for c in controller_rows:
        published = c["control_timing"]["command_publish_steady_ns"]
        values = tuple(c["command"])
        if published > 0:
            source[values].append(published)
    for values in source.values():
        values.sort()
    proposed = []
    counts = Counter()
    for g in applied_rows:
        received = int(g["observed_steady_ns"])
        payload = tuple(float(g[key]) for key in ("vx_mps", "vy_mps", "wz_radps"))
        candidates = source[payload]
        low = bisect_left(candidates, received - window_ns)
        high = bisect_right(candidates, received)
        if high - low == 1:
            proposed.append((candidates[low], g))
        else:
            counts["ambiguous_payload" if high > low else "outside_trace_or_window"] += 1
    reverse = Counter(published for published, _ in proposed)
    pairs = [(published, g) for published, g in proposed if reverse[published] == 1]
    counts["ambiguous_reverse"] = len(proposed) - len(pairs)
    counts["matched"] = len(pairs)
    return pairs, dict(counts)


def deadline_info(timing):
    """Count skipped releases and overruns beyond the latest release's deadline."""
    period = timing["timer_period_ns"]
    due = timing["timer_due_clock_ns"]
    lateness = max(0, timing["timer_called_clock_ns"] - due)
    skipped = lateness // period
    overruns = int(timing["callback_ended_clock_ns"] > due + (skipped + 1) * period)
    return lateness, skipped, overruns


def command_is_valid(c):
    values = c["command"]
    if len(values) != 3 or not all(isinstance(v, (int,float)) and math.isfinite(v)
                                  for v in values):
        return False
    vx, vy, wz = values
    if math.hypot(vx,vy) > .28 + 1e-10 or abs(wz) > .55 + 1e-10:
        return False
    mode = c["planned_mode"]
    if mode == 0:
        return vy == 0. and abs(wz) * .83 <= abs(vx) + 1e-10
    if mode == 1:
        return wz == 0.
    if mode == 2:
        return vx == vy == 0.
    return False


def rate_violation_pairs(tracking):
    count = 0
    for old, new in zip(tracking, tracking[1:]):
        dt = new["stamp"] - old["stamp"]
        if not 0 < dt <= .20 or old["planned_mode"] != new["planned_mode"]:
            continue
        if not command_is_valid(old) or not command_is_valid(new):
            continue
        vx1, vy1, wz1 = old["command"]
        vx2, vy2, wz2 = new["command"]
        changes_direction = ((new["planned_mode"] == 0 and vx1 * vx2 < 0.) or
                             (new["planned_mode"] == 1 and vy1 * vy2 < 0.))
        count += int(math.hypot(vx2 - vx1, vy2 - vy1) > .45 * dt + 1e-9 or
                     abs(wz2 - wz1) > .70 * dt + 1e-9 or changes_direction)
    return count


def report_tracker(control_path, plant_path):
    controllers = [json.loads(line) for line in control_path.read_text().splitlines()]
    all_timed = [c for c in controllers if "control_timing" in c]
    tracking = [c for c in all_timed if c["phase"] == "TRACKING"]
    timing = [c["control_timing"] for c in tracking if c["control_timing"]["timer_period_ns"] > 0]
    if not timing:
        raise ValueError(f"no tracking timer records in {control_path}")
    with plant_path.open(newline="") as stream:
        raw = list(csv.DictReader(stream))
    sensors = {int(row["identity"]): int(row["observed_steady_ns"])
               for row in raw if row["event"] == "odometry"}
    published = {c["control_timing"]["command_publish_steady_ns"] for c in tracking}
    pairs, pair_counts = pair_commands(all_timed, (r for r in raw if r["event"] == "command"))
    matching = [(begin, row) for begin,row in pairs if begin in published]
    deadlines = [deadline_info(t) for t in timing]
    period_ns = timing[0]["timer_period_ns"]
    recorded_due = [t["timer_due_clock_ns"] for t in timing]
    span_releases = round((max(recorded_due)-min(recorded_due))/period_ns) + 1
    distinct_odom = {t["odom_stamp_ns"]: t for t in timing if t["odom_stamp_ns"] > 0}
    tracked_sensor = list(distinct_odom.values())
    wall_intervals = [round(c["callback_interval_s"]*SECOND_NS) for c in tracking
                      if c["callback_interval_s"] > 0.]
    command_to_consumed = [int(r["consume_steady_ns"])-begin for begin,r in matching]

    def latency(rows, earlier, later):
        return stats_ns([b-a for a,b in ((x[earlier],x[later]) for x in rows) if 0 < a <= b])

    return {
        "tracking_cycles": len(timing),
        "core_tick": stats_ns([t["core_ns"] for t in timing]),
        "callback_to_status_queue": latency(timing, "callback_started_steady_ns", "callback_ended_steady_ns"),
        "timer_release_to_dispatch_ros": stats_ns([entry[0] for entry in deadlines]),
        "executor_dispatch_to_callback_steady": latency(timing, "timer_called_steady_ns", "callback_started_steady_ns"),
        "wall_interval": stats_ns(wall_intervals),
        "absolute_wall_interval_jitter": stats_ns([abs(interval-period_ns)
                                                   for interval in wall_intervals]),
        "wall_intervals_at_least_two_periods": sum(interval >= 2*period_ns
                                                    for interval in wall_intervals),
        "release_slots_in_between_first_and_last_trace": span_releases,
        "unaccounted_slots_in_trace": max(0, span_releases-len(timing)),
        "skipped_ros_timer_releases": sum(entry[1] for entry in deadlines),
        "callback_ros_deadline_overruns": sum(entry[2] for entry in deadlines),
        "status_queue_drops": max(t["status_drops"] for t in timing),
        "invalid_command_count": sum(not command_is_valid(c) for c in tracking),
        "invalid_command_rate_pairs": rate_violation_pairs(tracking),
        "sensor_distinct_messages_used": len(tracked_sensor),
        "sensor_middleware_source_to_receive": latency(
            tracked_sensor, "odom_source_realtime_ns", "odom_received_realtime_ns"),
        "sensor_receive_to_executor_callback": latency(
            tracked_sensor, "odom_received_realtime_ns", "odom_callback_realtime_ns"),
        "sensor_middleware_source_to_executor_callback": latency(
            tracked_sensor, "odom_source_realtime_ns", "odom_callback_realtime_ns"),
        "sensor_callback_to_tick_snapshot": latency(timing, "odom_callback_steady_ns", "callback_started_steady_ns"),
        "gazebo_sensor_publish_to_tracker_callback": stats_ns([
            t["odom_callback_steady_ns"]-sensors[t["odom_stamp_ns"]]
            for t in tracked_sensor if t["odom_stamp_ns"] in sensors
            and t["odom_callback_steady_ns"] >= sensors[t["odom_stamp_ns"]]]),
        "command_correlation": {**pair_counts, "matched_tracking": len(matching),
                                "tracking_publish_count": len(published)},
        "command_publish_to_gz_receive": stats_ns([
            int(r["observed_steady_ns"])-begin for begin,r in matching]),
        "gazebo_wait_before_consumption": stats_ns([
            int(r["consume_steady_ns"])-int(r["observed_steady_ns"]) for _,r in matching]),
        "gazebo_apply_call": stats_ns([
            int(r["applied_steady_ns"])-int(r["consume_steady_ns"]) for _,r in matching]),
        "command_publish_to_gazebo_consumed": stats_ns(command_to_consumed),
        "commands_consumed_after_one_wall_period": sum(delay > period_ns
                                                        for delay in command_to_consumed),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    probe = json.loads((args.capture/"probe_summary.json").read_text())
    trackers = {
        p.name.removesuffix("_controller.jsonl"): report_tracker(
            p, p.with_name(p.name.replace("_controller.jsonl", "_plant_timing.csv")))
        for p in sorted(args.capture.glob("*_controller.jsonl"))}
    result = {"probe": probe, "robots": trackers}
    args.output.write_text(json.dumps(result, indent=2)+"\n")


if __name__ == "__main__":
    main()
