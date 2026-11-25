from analyze_control_timing import (
    PERIOD_NS, assess_acceptance, deadline_info, long_callback_gaps, pair_commands, rate_violation_pairs, slow_command,
)


def sample_source(ns, vx):
    return {"control_timing": {"command_publish_steady_ns": ns}, "command": (vx, 0., 0.)}


def sample_gazebo(ns, vx):
    return {"observed_steady_ns": str(ns), "vx_mps": str(vx), "vy_mps": "0", "wz_radps": "0"}


def test_match_only_unique_causal_twist_payload():
    sources = [sample_source(t, x) for t,x in ((100, .1), (101, .2), (110, .2), (120, .3), (200, .4))]
    gazebos = [sample_gazebo(t, x) for t,x in ((103, .1), (113, .2), (125, .3), (130, .3), (135, .4))]
    result, counter = pair_commands(sources, gazebos, window_ns=20)
    assert result == [(100, gazebos[0])]
    assert counter == {"matched": 1, "ambiguous_payload": 1, "ambiguous_reverse": 2,
                       "outside_trace_or_window": 1}


def test_release_miss_and_execution_overrun_counted_separately():
    timer = {"timer_period_ns": 40, "timer_due_clock_ns": 120, "timer_called_clock_ns": 167,
             "callback_ended_clock_ns": 167}
    assert deadline_info(timer) == (47, 1, 0)
    timer["callback_ended_clock_ns"] = 201
    assert deadline_info(timer) == (47, 1, 1)



def test_nominal_rate_and_reversal_checks():
    def ack(t, vx):
        return {"stamp": t, "planned_mode": 0, "command": (vx, 0., 0.)}
    assert rate_violation_pairs([ack(0., .02), ack(.04, .03), ack(.08, .04)]) == 0
    assert rate_violation_pairs([ack(0., .02), ack(.04, .05)]) == 1
    assert rate_violation_pairs([ack(0., .001), ack(.04, -.001)]) == 1


def test_wall_gap_does_not_become_a_ros_deadline_miss_or_a_compute_violation():
    probe = {"status": "passed", "robots": 1, "tracking_wall_s": 605.}
    tracker = dict(timer_period_ns=PERIOD_NS, tracking_cycles=15_000,
                   core_tick={"p99_ms": .1, "max_ms": .9}, callback_to_status_queue={"max_ms": 4.},
                   skipped_ros_timer_releases=0, callback_ros_deadline_overruns=0,
                   unaccounted_slots_in_trace=0, status_queue_drops=0, invalid_command_count=0,
                   invalid_command_rate_pairs=0, command_correlation={"matched_tracking": 15_000,
                                                                     "tracking_publish_count": 15_000},
                   wall_intervals_greater_than_one_period=1, commands_consumed_after_one_wall_period=0)
    result = assess_acceptance(probe, {"robot1": tracker})
    assert result["ros_clock_gate"] == "PASS"
    assert result["strict_wall_gate"] == result["full_gate"] == "NOT_MET"
    tracker.update(wall_intervals_greater_than_one_period=0)
    assert assess_acceptance(probe, {"robot1": tracker})["full_gate"] == "PASS"
    probe.update(tracking_wall_s=8.)
    assert assess_acceptance(probe, {"robot1": tracker})["full_gate"] == "NOT_MET"


def test_controller_wall_gap_correlates_sim_progress_and_plant_pause():
    def tick(wall_ns, sim_ns, cpu_ns, voluntary):
        return {"stamp": sim_ns * 1e-9, "callback_interval_s": .140, "control_timing": {
            "timer_period_ns": PERIOD_NS, "timer_due_clock_ns": sim_ns, "timer_called_clock_ns": sim_ns,
            "callback_ended_clock_ns": sim_ns, "callback_started_steady_ns": wall_ns,
            "thread_cpu_ns": cpu_ns, "thread_voluntary_switches": voluntary, "thread_involuntary_switches": 0}}

    physics = {"begin_steady_ns": 11_000_000, "end_steady_ns": 131_000_000,
               "wall_ns": 120_000_000, "sim_advance_ns": 2_000_000}
    rows = long_callback_gaps([tick(10_000_000, PERIOD_NS, 0, 1),
                              tick(150_000_000, PERIOD_NS*2, 100_000, 4)], [], [physics])
    record, = rows
    assert record["ros_advance_ns"] == PERIOD_NS and record["ros_dispatch_lateness_ns"] == 0
    assert record["control_thread_cpu_ns"] == 100_000 and record["control_thread_voluntary_switches"] == 3
    assert record["gazebo_physics_gap"] == physics


def test_consumption_attribution_keeps_dds_bridge_and_physics_boundaries():
    plant = {"observed_steady_ns": "140", "consume_steady_ns": "220"}
    bridge = {"observed_steady_ns": "135", "taken_realtime_ns": "1135",
              "source_realtime_ns": "1102", "received_realtime_ns": "1108"}
    controller = {"stamp": 1.0, "control_timing": {"command_published_steady_ns": 104}}
    row = slow_command(100, plant, bridge, controller, [], [])
    assert (row["ros_publish_call_ns"], row["dds_source_to_receive_ns"],
            row["rcl_receive_to_bridge_take_ns"], row["bridge_take_to_gazebo_receive_ns"],
            row["gazebo_wait_ns"], row["publish_to_consume_ns"]) == (4, 6, 27, 5, 80, 120)
    row = slow_command(100, plant, {**bridge, "observed_steady_ns": "150"}, controller, [], [])
    assert row["dds_source_to_receive_ns"] is None and row["bridge_take_to_gazebo_receive_ns"] is None
