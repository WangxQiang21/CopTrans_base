"""Replay both controllers with the same trajectories, feedback and faults."""
from dataclasses import replace
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import subprocess

import pytest

from fwis_control.config import load
from fwis_control.model import Mode, Observation, Twist, ZERO
from scenarios import A, O, S, Leg, sample_case, suite


CONFIG = load(Path(__file__).resolve().parent.parent / "config/ranger_tracking.yaml")


def write_fields(fields):
    return " ".join(str(x) for x in fields)


def replay(algorithm, cases, binary):
    lines, expected = [], []
    for case in cases:
        trajectory, start_mode, delta, future, axial, fault = case
        now = trajectory.stamp_ns * 1e-9 + delta
        engine = CONFIG.execution(algorithm)
        result = engine.admit(trajectory, start_mode, now, future_window=future,
                              allow_axial_oblique=axial)
        lines.append(write_fields([int(algorithm == "pid"), int(axial),
                                   int(start_mode), now, future]))
        lines.append(write_fields([trajectory.stamp_ns, trajectory.frame,
                                   trajectory.plan_id, len(trajectory.points)]))
        for p in trajectory.points:
            lines.append(write_fields([p.time_ns, p.pose.x, p.pose.y, p.pose.yaw,
                                       p.speed, int(p.mode)]))
        expected.append([result.result, engine.phase,
                         *(f"{issue.code}:{issue.level}" for issue in result.issues)])
        num_ticks = math.ceil((trajectory.duration + .4 + max(0., -delta)) / CONFIG.timing.period)
        lines.append(str(num_ticks))
        mode = start_mode
        for index in range(num_ticks):
            tick = now + CONFIG.timing.period * index
            inject_nan = fault == "nan" and index == 4
            if fault == "clock_jump" and index >= 4:
                tick -= .05
            ref = result.sampler.sample(tick - engine.start) if result.sampler is not None else None
            pose, measured = trajectory.points[0].pose, ZERO
            if ref is not None:
                pose, measured = ref.pose, ref.twist
                if measured != ZERO:
                    pose = replace(pose, x=pose.x + .06 * math.sin(tick * 4),
                                   y=pose.y - .05 * math.cos(tick * 4),
                                   yaw=pose.yaw + .10 * math.sin(tick * 2))
            stamp = tick - .3 if fault == "stale_odom" and index == 4 else tick
            mode_stamp = tick - .3 if fault == "stale_mode" and index == 4 else tick
            if fault == "unexpected_mode" and index == 4:
                mode = Mode.SPIN
            observation = Observation(pose, measured, stamp, mode, mode_stamp)
            if inject_nan:
                observation = replace(observation, pose=replace(pose, x=math.nan))
            has_feedback = not (fault == "missing" and index == 4)
            lines.append(write_fields([tick, int(has_feedback), pose.x, pose.y, pose.yaw,
                                      *measured.values(), stamp, int(mode), mode_stamp,
                                      int(inject_nan)]))
            output = engine.tick(tick, observation if has_feedback else None)
            if output.requested_mode is not None and fault != "missing_confirmation":
                mode = output.requested_mode
            values = output.reference
            reference = (int(values.mode), values.time, values.pose.x, values.pose.y,
                         values.pose.yaw, *values.twist.values()) if values else (-1, *([0.] * 7))
            expected.append((
                output.phase, int(output.requested_mode) if output.requested_mode is not None else -1,
                *output.raw.values(), *output.command.values(), *reference, *output.reasons))
    process = subprocess.run([binary], input="\n".join(lines) + "\n", text=True,
                             capture_output=True, check=True, timeout=10.)
    observed = process.stdout.splitlines()
    assert len(observed) == len(expected)
    differences = []
    ticks = 0
    for index, (text, want) in enumerate(zip(observed, expected)):
        tokens = text.split()
        if isinstance(want, list):
            assert tokens == want, index
            continue
        assert tokens[0] == want[0] and int(tokens[1]) == want[1], index
        numeric = tuple(float(x) for x in tokens[2:16])
        assert numeric == pytest.approx(want[2:16], rel=0., abs=1e-9), index
        assert all(math.isfinite(value) for value in numeric)
        differences.extend(abs(actual - target) for actual, target in zip(numeric[:6], want[2:8]))
        ticks += 1
        assert tuple(tokens[16:]) == want[16:], index
    evidence = os.environ.get("COPTRANS_PARITY_EVIDENCE")
    if evidence:
        folder = Path(evidence)
        folder.mkdir(parents=True, exist_ok=True)
        report = dict(algorithm=algorithm, ticks=ticks, matched_fsm_ticks=ticks,
                      compared_raw_and_applied_components=len(differences),
                      max_absolute_error=max(differences), mean_absolute_error=math.fsum(differences)/len(differences),
                      nonfinite_outputs=0, absolute_tolerance=1e-9, relative_tolerance=0.,
                      fixture_sha256=sha256(("\n".join(lines)+"\n").encode()).hexdigest(),
                      replay_sha256=sha256(Path(binary).read_bytes()).hexdigest())
        (folder / f"parity_{algorithm}.json").write_text(json.dumps(report,indent=2)+"\n")
    return expected


@pytest.mark.parametrize("algorithm", ["nonlinear", "pid"])
def test_cpp_ticks_match_python_oracle(algorithm):
    binary = os.environ.get("TRACKING_CPP_REPLAY")
    if not binary:
        pytest.skip("The C++ replay target is supplied by the fwis_control CTest configuration")
    cases = [(item.trajectory, A, 0., 0., False, None) for item in suite()]
    arc = cases[2][0]
    straight = cases[0][0]
    spin = next(item.trajectory for item in suite() if item.name == "spin_90")
    axial = sample_case("axial", [Leg(O, .40, .15)]).trajectory
    near_axial = sample_case("near_axial", [Leg(O, -.6, .21, parameter=math.radians(1.435))]).trajectory
    cases.extend([
        (axial, A, -.10, .5, True, None),
        (axial, S, 0., 0., True, None),
        (near_axial, A, 0., 0., True, None),
        (arc, A, .8, 0., False, None),
        (arc, A, -.4, 0., False, None),
        (spin, A, 0., 0., False, "missing_confirmation"),
    ])
    cases.extend((straight, A, 0., 0., False, fault) for fault in (
        "nan", "clock_jump", "stale_odom", "stale_mode", "unexpected_mode", "missing"))
    output = replay(algorithm, cases, binary)
    phases = {row[0] for row in output if isinstance(row, tuple)}
    reasons = {reason for row in output if isinstance(row, tuple) for reason in row[16:]}
    assert {"WAIT_START", "COMPLETE", "STOPPED", "REQUEST_MODE_SWITCH",
            "WAIT_MODE_CONFIRMATION", "SETTLE", "REJECTED", "STALE_TRAJECTORY"} <= phases
    assert {"missing_stale_or_invalid_feedback", "control_clock_jump_or_overrun",
            "unexpected_feedback_mode", "mode_confirmation_timeout",
            "oblique_reference_branch"} <= reasons
