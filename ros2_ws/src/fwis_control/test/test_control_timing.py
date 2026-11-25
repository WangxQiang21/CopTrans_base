from analyze_control_timing import deadline_info, pair_commands, rate_violation_pairs


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
