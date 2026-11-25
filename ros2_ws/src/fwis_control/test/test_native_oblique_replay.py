"""Compare native model sampling, both tracking laws, mode retention and terminal rest."""
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import subprocess

import pytest

from fwis_control.config import load
from fwis_control.model import Mode, Observation, Pose, Reference, Twist, ZERO


@pytest.mark.parametrize("algorithm", ["pid", "nonlinear"])
@pytest.mark.parametrize("direction", [-1, 1])
def test_native_oblique_reference(algorithm, direction):
    binary = os.environ.get("TRACKING_NATIVE_OBLIQUE")
    if not binary:
        pytest.skip("CMake supplies the native oblique replay executable")
    rows = subprocess.check_output([binary, algorithm, str(direction)], text=True).splitlines()
    engine = load(Path(__file__).resolve().parents[1]/"config/ranger_tracking.yaml").execution(algorithm)
    previous = 0.
    max_err = 0.
    center_vy = None
    assert len(rows) >= 201
    for k, line in enumerate(rows):
        seq, phase, *data = line.split()
        assert int(seq) == k
        values = list(map(float, data))
        now = 1.+.04*k
        t = min(8., now-1.)
        u = t/8.
        s = (10 - 15*u + 6*u*u)*u**3
        ds = 30*u*u*(1-u)**2/8.
        pose = Pose(.8*direction*s, .125*s*(1-s), 0.)
        twist = Twist(.8*direction*ds, .125*(1-2*s)*ds, 0.)
        ref = Reference(now-1., pose, twist, Mode.OBLIQUE)
        feedback = Observation(Pose(pose.x-.012*twist.vx, pose.y+.008*twist.vy, 0.),
                               twist, now, Mode.OBLIQUE, now)
        dt = now-previous if k else .04
        previous = now
        if t >= 8. and engine.filter.previous == ZERO:
            want_phase, raw, cmd = "COMPLETE", ZERO, ZERO
        elif twist == ZERO:
            want_phase = "FINAL_STOP" if t >= 8. else "TRACKING"
            engine.controller.reset()
            raw = ZERO
            cmd = engine.filter.apply(ZERO, Mode.OBLIQUE, dt).command
        else:
            want_phase = "TRACKING"
            raw = engine.controller.update(ref, feedback, dt, 0.)
            cmd = engine.filter.apply(raw, Mode.OBLIQUE, dt, 0., twist.vy, continuous_oblique=True).command
            engine.controller.accept(cmd)
        expected = (twist.vx, twist.vy, *raw.values(), *cmd.values())
        max_err = max(max_err, max(abs(a-b) for a,b in zip(values, expected, strict=True)))
        assert all(math.isfinite(x) for x in values) and values == pytest.approx(expected, abs=1e-10, rel=0.)
        assert phase == want_phase
        if 96 <= k <= 104:
            assert phase == "TRACKING" and abs(values[-3]) > .15 and values[-2] != 0.
        if k == 100:
            assert values[1] == 0.
            center_vy = values[-2]
        if k in (0, 200):
            assert values[:2] == [0., 0.]
    assert rows[-1].split()[1] == "COMPLETE" and values[-3:] == [0.,0.,0.]
    assert center_vy is not None and float(rows[99].split()[3]) > 0. and float(rows[101].split()[3]) < 0.
    if folder := os.environ.get("COPTRANS_PARITY_EVIDENCE"):
        evidence=dict(algorithm=algorithm, direction=direction, ticks=len(rows), matched_fsm_ticks=len(rows),
                      max_absolute_error=max_err, zero_crossing_tick=100, geometric_reference_vy_at_crossing=0.,
                      applied_vy_at_crossing=center_vy, exact_terminal_rest=True,
                      replay_sha256=sha256(Path(binary).read_bytes()).hexdigest())
        (Path(folder)/f"native_oblique_{algorithm}_{direction}.json").write_text(json.dumps(evidence, indent=2)+"\n")
