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
PERIOD_NS = 40_000_000
MIN_WALL_S = 600.
MIN_TRACKING_CYCLES = 15_000


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


def overlap_gap(gaps, begin, end):
    return max((row for row in gaps if row["begin_steady_ns"] < end and begin < row["end_steady_ns"]),
               key=lambda row: row["wall_ns"], default=None)


def plant_gaps(plant_rows):
    odometry = sorted((int(row["observed_steady_ns"]), int(row["identity"]))
                      for row in plant_rows if row["event"] == "odometry")
    odom_gaps = [dict(begin_steady_ns=start, end_steady_ns=end, wall_ns=end-start,
                      sim_advance_ns=last-first)
                 for (start, first), (end, last) in zip(odometry, odometry[1:])
                 if end-start >= PERIOD_NS]
    physics_gaps = []
    for row in plant_rows:
        if row["event"] != "physics_gap":
            continue
        start, end = int(row["previous_steady_ns"]), int(row["observed_steady_ns"])
        physics_gaps.append(dict(begin_steady_ns=start, end_steady_ns=end, wall_ns=end-start,
                                 **{key: int(row[key]) for key in ("sim_advance_ns", "thread_cpu_delta_ns",
                                                                  "voluntary_switches", "involuntary_switches")}))
    return odom_gaps, physics_gaps


def delta(previous, current, key):
    a, b = previous.get(key), current.get(key)
    return b-a if a is not None and b is not None and 0 <= a <= b else None


def long_callback_gaps(tracking, odom_gaps, physics_gaps):
    rows = []
    for previous, current in zip(tracking, tracking[1:]):
        a, b = previous["control_timing"], current["control_timing"]
        wall = round(current["callback_interval_s"]*SECOND_NS)
        if wall < 2*b["timer_period_ns"]:
            continue
        end = b["callback_started_steady_ns"]
        begin = end - wall
        cpu = delta(a, b, "thread_cpu_ns")
        # Only use thread counters when the raw records span this exact callback interval.
        consecutive = abs(a["callback_started_steady_ns"] - begin) <= 5_000
        rows.append(dict(stamp_s=current["stamp"], wall_ns=wall, begin_steady_ns=begin,
                         end_steady_ns=end, ros_advance_ns=b["timer_called_clock_ns"]-a["timer_called_clock_ns"],
                         ros_dispatch_lateness_ns=deadline_info(b)[0],
                         control_thread_cpu_ns=cpu if consecutive else None,
                         control_thread_voluntary_switches=delta(a, b, "thread_voluntary_switches") if consecutive else None,
                         control_thread_involuntary_switches=delta(a, b, "thread_involuntary_switches") if consecutive else None,
                         gazebo_odom_gap=overlap_gap(odom_gaps, begin, end),
                         gazebo_physics_gap=overlap_gap(physics_gaps, begin, end)))
    return rows


def bridge_latency(row, key_a, key_b):
    a, b = int(row[key_a]), int(row[key_b])
    return b-a if 0 < a <= b else None


def slow_command(published, row, bridge, controller, odom_gaps, physics_gaps):
    received = int(row["observed_steady_ns"])
    consumed = int(row["consume_steady_ns"])
    bridge_taken = int(bridge["observed_steady_ns"]) if bridge else None
    # A matching repeated payload can travel twice. Never infer a causal stage
    # if the observed bridge take occurred after the body received the Twist.
    link = bridge and published <= bridge_taken <= received
    publish_end = controller["control_timing"].get("command_published_steady_ns", 0)
    return dict(stamp_s=controller["stamp"], published_steady_ns=published,
                ros_publish_call_ns=publish_end-published if published <= publish_end else None,
                publish_to_gazebo_receive_ns=received-published,
                gazebo_wait_ns=consumed-received,
                publish_to_consume_ns=consumed-published,
                publish_to_bridge_take_ns=bridge_taken-published if link else None,
                dds_source_to_receive_ns=bridge_latency(bridge, "source_realtime_ns", "received_realtime_ns") if link else None,
                rcl_receive_to_bridge_take_ns=bridge_latency(bridge, "received_realtime_ns", "taken_realtime_ns") if link else None,
                bridge_take_to_gazebo_receive_ns=received-bridge_taken if link else None,
                gazebo_odom_gap=overlap_gap(odom_gaps, published, consumed),
                gazebo_physics_gap=overlap_gap(physics_gaps, published, consumed))


def report_tracker(control_path, plant_path, bridge_rows=(), shared_physics_gaps=()):
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
    published = {c["control_timing"]["command_publish_steady_ns"]: c for c in tracking}
    pairs, pair_counts = pair_commands(all_timed, (r for r in raw if r["event"] == "command"))
    matching = [(begin, row) for begin,row in pairs if begin in published]
    bridge_pairs, bridge_counts = pair_commands(all_timed, bridge_rows)
    bridge_matching = {begin: row for begin, row in bridge_pairs if begin in published}
    odom_gaps, own_physics = plant_gaps(raw)
    physics_gaps = own_physics or shared_physics_gaps
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
        "timer_period_ns": period_ns if all(t["timer_period_ns"] == period_ns for t in timing) else None,
        "core_tick": stats_ns([t["core_ns"] for t in timing]),
        "callback_to_status_queue": latency(timing, "callback_started_steady_ns", "callback_ended_steady_ns"),
        "timer_release_to_dispatch_ros": stats_ns([entry[0] for entry in deadlines]),
        "executor_dispatch_to_callback_steady": latency(timing, "timer_called_steady_ns", "callback_started_steady_ns"),
        "wall_interval": stats_ns(wall_intervals),
        "absolute_wall_interval_jitter": stats_ns([abs(interval-period_ns)
                                                   for interval in wall_intervals]),
        "wall_intervals_at_least_two_periods": sum(interval >= 2*period_ns
                                                    for interval in wall_intervals),
        "wall_intervals_greater_than_one_period": sum(interval > period_ns for interval in wall_intervals),
        "long_callback_gaps": long_callback_gaps(tracking, odom_gaps, physics_gaps),
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
        "command_bridge_correlation": {**bridge_counts, "matched_tracking": len(bridge_matching),
                                       "trace_overflow": max((int(r["trace_overflow"]) for r in bridge_rows), default=0)},
        "command_ros_publish_call": latency(timing, "command_publish_steady_ns", "command_published_steady_ns")
        if timing[0].get("command_published_steady_ns") is not None else stats_ns([]),
        "command_dds_source_to_receive": stats_ns([
            v for row in bridge_matching.values()
            if (v := bridge_latency(row, "source_realtime_ns", "received_realtime_ns")) is not None]),
        "command_rcl_receive_to_bridge_take": stats_ns([
            v for row in bridge_matching.values()
            if (v := bridge_latency(row, "received_realtime_ns", "taken_realtime_ns")) is not None]),
        "command_bridge_take_to_gazebo_receive": stats_ns([
            int(r["observed_steady_ns"])-int(bridge_matching[begin]["observed_steady_ns"])
            for begin,r in matching if begin in bridge_matching
            and begin <= int(bridge_matching[begin]["observed_steady_ns"]) <= int(r["observed_steady_ns"])]),
        "command_publish_to_gz_receive": stats_ns([
            int(r["observed_steady_ns"])-begin for begin,r in matching]),
        "gazebo_wait_before_consumption": stats_ns([
            int(r["consume_steady_ns"])-int(r["observed_steady_ns"]) for _,r in matching]),
        "gazebo_apply_call": stats_ns([
            int(r["applied_steady_ns"])-int(r["consume_steady_ns"]) for _,r in matching]),
        "command_publish_to_gazebo_consumed": stats_ns(command_to_consumed),
        "commands_consumed_after_one_wall_period": sum(delay > period_ns
                                                        for delay in command_to_consumed),
        "slow_commands": [slow_command(begin, row, bridge_matching.get(begin), published[begin],
                                       odom_gaps, physics_gaps)
                          for begin, row in matching if int(row["consume_steady_ns"])-begin > period_ns],
    }


def assess_acceptance(probe, trackers):
    def status(checks):
        return "PASS" if all(checks.values()) else "NOT_MET"

    common = dict(probe_passed=probe["status"] == "passed",
                  required_robot_count=len(trackers) == probe["robots"],
                  wall_pressure_at_least_600s=probe.get("tracking_wall_s", 0.) >= MIN_WALL_S)
    simulation, wall = {}, {}
    for robot, row in trackers.items():
        commands = row["command_correlation"]
        simulation[robot] = dict(
            at_least_15000_tracking_cycles=row["tracking_cycles"] >= MIN_TRACKING_CYCLES,
            timer_period_is_25hz=row["timer_period_ns"] == PERIOD_NS,
            core_p99_at_most_0_5ms=row["core_tick"].get("p99_ms", math.inf) <= .50,
            core_max_at_most_1ms=row["core_tick"].get("max_ms", math.inf) <= 1.,
            callback_max_at_most_5ms=row["callback_to_status_queue"].get("max_ms", math.inf) <= 5.,
            no_ros_deadline_misses=row["skipped_ros_timer_releases"] == row["callback_ros_deadline_overruns"] == 0,
            no_trace_drops=row["unaccounted_slots_in_trace"] == row["status_queue_drops"] == 0,
            all_tracking_commands_matched=commands["matched_tracking"] == commands["tracking_publish_count"],
            no_invalid_commands=row["invalid_command_count"] == row["invalid_command_rate_pairs"] == 0,
        )
        wall[robot] = dict(no_full_40ms_release_missed=row["wall_intervals_at_least_two_periods"] == 0,
                           no_command_consumption_beyond_40ms=row["commands_consumed_after_one_wall_period"] == 0)
    sim_pass = status({**common, **{robot: all(checks.values()) for robot, checks in simulation.items()}})
    wall_pass = status({**common, **{robot: all(checks.values()) for robot, checks in wall.items()}})
    return dict(ros_clock_gate=sim_pass, strict_wall_gate=wall_pass,
                full_gate=status({"sim": sim_pass == "PASS", "wall": wall_pass == "PASS"}),
                common=common, simulation=simulation, wall=wall)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-contract", action="store_true",
                        help="Return exit status 1 unless both pressure gates pass")
    args = parser.parse_args()
    probe = json.loads((args.capture/"probe_summary.json").read_text())
    bridge_by_robot = defaultdict(list)
    for path in args.capture.glob("bridge_take_*.csv"):
        with path.open(newline="") as stream:
            for row in csv.DictReader(stream):
                bridge_by_robot[row["robot"]].append(row)
    with (args.capture/"robot1_plant_timing.csv").open(newline="") as stream:
        _, physics = plant_gaps(csv.DictReader(stream))
    trackers = {
        p.name.removesuffix("_controller.jsonl"): report_tracker(
            p, p.with_name(p.name.replace("_controller.jsonl", "_plant_timing.csv")),
            bridge_by_robot[p.name.removesuffix("_controller.jsonl")], physics)
        for p in sorted(args.capture.glob("*_controller.jsonl"))}
    acceptance = assess_acceptance(probe, trackers)
    result = {"probe": probe, "robots": trackers, "acceptance": acceptance}
    args.output.write_text(json.dumps(result, indent=2)+"\n")
    if args.require_contract and acceptance["full_gate"] != "PASS":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
