"""Classify native timed samples; body spin is never a stationary dwell."""
import ast
from dataclasses import asdict
import hashlib
import inspect
import math
from pathlib import Path
import subprocess

from .kernel_adapter import PROJECT_ROOT, Action, Pose, replay, wrap
from fwis_v3_optimizer.core import time_sample
from fwis_v3_optimizer.retiming import retime
from fwis_v3_optimizer.continuous_geometry import plan_from_actions
from fwis_v3_optimizer.continuous_retiming import retime_continuous
from fwis_v3_optimizer.mode_switch_timing import mode_switch_events_for_actions


def classify_samples(samples):
    intervals = []
    interior_zero = []
    for i, (left, right) in enumerate(zip(samples, samples[1:])):
        t, p, v, m = left
        u, q, w, n = right
        dt = u-t
        if dt <= 0:
            raise ValueError("nonincreasing_native_timestamps")
        dxy = math.hypot(q.x-p.x, q.y-p.y)
        dyaw = abs(wrap(q.th-p.th))
        kind = ("stationary" if dxy <= 1e-11 and dyaw <= 1e-11 else
                "spin" if dxy <= 1e-11 else
                "slow_translation" if dxy/dt < .2 else "translation")
        if intervals and intervals[-1]["kind"] == kind:
            intervals[-1]["t1"] = u
        else:
            intervals.append({"kind": kind, "t0": t, "t1": u})
        if (0 < i < len(samples)-1 and m != 2 and abs(v) < 1e-10 and
                samples[i-1][3] == m == n and
                math.hypot(p.x-samples[i-1][1].x, p.y-samples[i-1][1].y) > 1e-10 and
                dxy > 1e-10):
            interior_zero.append(t)
    for interval in intervals:
        interval["duration_s"] = interval["t1"]-interval["t0"]
    return {"duration_s": samples[-1][0], "intervals": intervals,
            "stationary_holds_s": [x["duration_s"] for x in intervals
                                   if x["kind"] == "stationary"],
            "spin_motion_s": sum(x["duration_s"] for x in intervals if x["kind"] == "spin"),
            "slow_translation_s": sum(x["duration_s"] for x in intervals
                                      if x["kind"] == "slow_translation"),
            "interior_same_mode_zero_speed_timestamps": interior_zero,
            "sample_count": len(samples)}


def _function_ast(text, name):
    tree = ast.parse(text)
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.dump(node, include_attributes=False)


def function_identity(function, baseline):
    path = Path(inspect.getfile(function)).resolve()
    relative = str(path.relative_to(PROJECT_ROOT))
    current = _function_ast(path.read_text(), function.__name__)
    frozen = subprocess.run(["git", "show", baseline+":"+relative],
                            cwd=PROJECT_ROOT, text=True, capture_output=True, check=True).stdout
    return {"file": relative, "function": function.__name__, "baseline": baseline,
            "current_ast_sha256": hashlib.sha256(current.encode()).hexdigest(),
            "executed_ast_sha256": hashlib.sha256(_function_ast(frozen, function.__name__).encode()).hexdigest(),
            "matches_frozen_function": current == _function_ast(frozen, function.__name__)}


def frozen_function(function, baseline):
    """Execute exactly the versioned function, without switching the worktree."""
    relative = str(Path(inspect.getfile(function)).resolve().relative_to(PROJECT_ROOT))
    source = subprocess.check_output(["git", "show", baseline+":"+relative],
                                     cwd=PROJECT_ROOT, text=True)
    node = next(n for n in ast.parse(source).body
                if isinstance(n, ast.FunctionDef) and n.name == function.__name__)
    module = ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[]))
    namespace = dict(function.__globals__)
    exec(compile(module, baseline+":"+relative, "exec"), namespace)
    return namespace[function.__name__]


def run_timing_audit(contract, dt=.04):
    start = Pose(3., 3., 0.)
    r = contract.limits.r_min_ack
    cases = {"A-O-A": [Action(0, 2., 0.), Action(1, .738, math.pi/2), Action(0, 2., 0.)],
             "A-Spin-A": [Action(0, 2., 0.), Action(2, math.pi/2, 0.), Action(0, 2., 0.)],
             "A-controls": [Action(0, .738, 0.), Action(0, .738, .5/r), Action(0, .738, 0.)]}
    identities = [function_identity(time_sample, "a492235"),
                  function_identity(retime, "a415043"),
                  function_identity(retime_continuous, "c90e981")]
    native_legacy = frozen_function(time_sample, "a492235")
    native_v1 = frozen_function(retime, "a415043")
    native_v2 = frozen_function(retime_continuous, "c90e981")
    report = {"scope": "native_functions_direct_not_ROS_planner_or_wall_clock_playback",
              "sampling_dt_s": dt, "profile": asdict(contract.limits),
              "legacy_native_v_demo": 2., "legacy_native_omega_demo": 2.,
              "source_identity": identities, "cases": {}}
    for name, actions in cases.items():
        events = mode_switch_events_for_actions(actions, contract.limits)
        legacy = native_legacy(start, actions, v_demo=2., omega_demo=2., dt=dt,
                             mode_switch_durations=[e["duration"] for e in events])
        v1 = native_v1(start, actions, contract.limits, dt=dt, allow_reverse=True)
        v2 = native_v2(plan_from_actions(start, replay(start, actions)[-1], actions),
                               contract.limits, dt=dt)
        methods = {}
        for backend, samples in (("legacy", legacy), ("v1", v1.samples), ("v2", v2.samples)):
            stats = classify_samples(samples)
            holds = stats["stationary_holds_s"]
            stats["expected_event_holds_s"] = [e["duration"] for e in events]
            stats["event_holds_match"] = (len(holds) == len(events) and all(
                abs(a-b["duration"]) <= 1e-6 for a, b in zip(holds, events)))
            stats["timed_trace"] = [[t, p.x, p.y, p.th, v, m] for t, p, v, m in samples]
            methods[backend] = stats
        report["cases"][name] = {"actions": [asdict(a) for a in actions], "methods": methods}
    return report
