"""Compute trajectory metrics from recorded Gazebo observations and commands."""
import math
import statistics


def percentile(values, fraction):
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    left = int(index)
    return ordered[left] + (index - left) * (ordered[min(left + 1, len(ordered) - 1)] - ordered[left])


def summarize(values):
    return {"mean": statistics.fmean(values), "std": statistics.stdev(values) if len(values) > 1 else 0.,
            "median": statistics.median(values), "p95": percentile(values, .95), "worst": max(values)}


def rms(values):
    return math.sqrt(statistics.fmean(x*x for x in values)) if values else 0.


def score(rows, period, guard_radius, switch_count):
    if not rows:
        return {}
    references = [row for row in rows if row["position_error"] is not None]
    pairs = [(a, b, b["time_s"] - a["time_s"]) for a, b in zip(rows, rows[1:])]
    position = [row["position_error"] for row in references]
    heading = [abs(row["heading_error"]) for row in references]
    speed = [row["speed_error"] for row in references]
    yaw_rate = [row["yaw_rate_error"] for row in references]
    compute = [row["compute_s"] for row in rows]
    linear_delta = [math.hypot(b["cmd_vx"]-a["cmd_vx"], b["cmd_vy"]-a["cmd_vy"]) for a, b, _ in pairs]
    yaw_delta = [abs(b["cmd_wz"]-a["cmd_wz"]) for a, b, _ in pairs]
    linear_accel = [math.hypot(b["vx"]-a["vx"], b["vy"]-a["vy"]) / dt for a, b, dt in pairs]
    angular_accel = [abs(b["wz"]-a["wz"])/dt for a, b, dt in pairs]
    changes = [x for x in references if x["phase"] in ("STOPPING", "REQUEST_MODE_SWITCH",
                                                       "WAIT_MODE_CONFIRMATION", "SETTLE")]
    switch_time = []
    started = None
    for row in rows:
        if row["phase"] == "STOPPING" and started is None:
            started = row["time_s"]
        if started is not None and row["phase"] in ("TRACKING", "APPROACH_SWITCH", "COMPLETE"):
            switch_time.append(row["time_s"]-started)
            started = None

    results = {}
    for field, values in (("position_m", position), ("yaw_rad", heading),
                          ("speed_mps", speed), ("yaw_rate_radps", yaw_rate)):
        if values:
            results.update({f"{field}_rmse": rms(values), f"{field}_mae": statistics.fmean(values),
                            f"{field}_p95": percentile(values, .95), f"{field}_max": max(values)})
    if position:
        results.update(final_position_m=position[-1], final_yaw_rad=heading[-1])
    results.update(
        filter_fraction=sum(bool(r["reasons"]) for r in rows)/len(rows),
        saturation_fraction=sum(bool(set(r["reasons"].split("|")).intersection(
            {"speed", "yaw_rate", "acceleration", "angular_acceleration", "ackermann_radius"}))
            for r in rows)/len(rows),
        safety_stops=sum(r["phase"] == "STOPPED" for r in rows),
        timeouts=sum("timeout" in r["reasons"] or "stale" in r["reasons"]
                     or "overrun" in r["reasons"] for r in rows),
        mode_mismatches=sum(r["phase"] in ("TRACKING", "APPROACH_SWITCH") and
                            r["planned_mode"] != r["observed_mode"] for r in references),
        ack_command_guard_violations=sum(
            r["command_mode"] == 0 and (abs(r["cmd_vy"]) > 1e-10 or
                                        guard_radius * abs(r["cmd_wz"]) > abs(r["cmd_vx"])+1e-9)
            for r in rows),
        unintended_spin_commands=sum(r["command_mode"] == 0 and r["classified_command"] == 2 for r in rows),
        command_envelope_violations=sum(r["command_violation"] for r in rows),
        linear_accel_mps2_rms=rms(linear_accel),
        angular_accel_radps2_rms=rms(angular_accel),
        command_linear_variation_mps=sum(linear_delta),
        command_angular_variation_radps=sum(yaw_delta),
        command_max_linear_rate_mps2=max((change / pair[2] for change, pair in zip(linear_delta, pairs)), default=0.),
        command_max_angular_rate_radps2=max((change / pair[2] for change, pair in zip(yaw_delta, pairs)), default=0.),
        switches_requested=switch_count, switches_completed=len(switch_time),
        switch_success_rate=len(switch_time) / switch_count if switch_count else 1.,
        switch_time_s_mean=statistics.fmean(switch_time) if switch_time else 0.,
        switch_time_s_max=max(switch_time, default=0.),
        switch_position_m_max=max((r["position_error"] for r in changes), default=0.),
        switch_yaw_rad_max=max((abs(r["heading_error"]) for r in changes), default=0.),
        compute_s_mean=statistics.fmean(compute), compute_s_p99=percentile(compute, .99),
        compute_s_max=max(compute), compute_overruns=sum(x > period for x in compute),
        cycle_s_mean=statistics.fmean(p[2] for p in pairs) if pairs else 0.,
        cycle_s_p99=percentile([p[2] for p in pairs], .99) if pairs else 0.,
        cycle_s_max=max((p[2] for p in pairs), default=0.))
    return results
