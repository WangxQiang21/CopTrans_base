"""Reproducible P0/P1 diagnostics; no trained-policy claims."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import platform
import subprocess
import time

import numpy as np

from .action_library import ActionLibrary
from .benchmark import BASELINES, engineering_tasks
from .environment import EnvironmentConfig, PlanningEnv
from .kernel_adapter import PROJECT_ROOT, Action, Grid, Pose, advance, replay, load_contract, stable_hash
from .timing_audit import run_timing_audit
from .action_mask import valid_actions
from .benchmark import sha256_file


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True,
                                    ensure_ascii=False, allow_nan=False)+"\n")


def smoke_tasks(contract, library):
    L = contract.length
    grid = Grid(np.zeros((120, 120), dtype=bool), .05, 0., 0.,
                contract.length, contract.width, contract.margin)
    start = Pose(2., 2., 0.)
    sequences = {"straight_A": [Action(0, L, 0.)]*3,
                 "lateral_O": [Action(1, L, np.pi/2)]*2,
                 "corner_A": [Action(0, L, 1./contract.limits.r_min_ack)]*2,
                 "A_Spin_A": [Action(0, L, 0.), Action(2, np.pi/2, 0.), Action(0, L, 0.)]}
    for name, actions in sequences.items():
        goal = replay(start, actions)[-1]
        if name == "lateral_O":
            occupied = np.ones((400, 200), dtype=bool)
            occupied[20:380, 57:143] = False
            narrow = Grid(occupied, .01, -1., -1., contract.length,
                          contract.width, contract.margin)
            narrow_start = Pose(0., 0., 0.)
            yield contract.problem(narrow, narrow_start, replay(narrow_start, actions)[-1],
                                   task_id=name), actions
        else:
            yield contract.problem(grid, start, goal, task_id=name), actions


def run_smoke(contract, library, seed=20261026):
    rng = np.random.default_rng(seed)
    results = []
    for problem, known in smoke_tasks(contract, library):
        for strategy in ("known_valid", "distance_greedy", "random"):
            env = PlanningEnv(lambda random: problem, library, EnvironmentConfig(max_decisions=24))
            begun = time.perf_counter()
            obs, _ = env.reset(seed=seed)
            phi0 = env.builder.potential(problem.start)
            reward_sum = 0.
            done = False
            ids = []
            for j in range(env.config.max_decisions):
                valid = np.flatnonzero(env.action_masks())
                if strategy == "known_valid":
                    a = known[j]
                    selected = next(i for i, b in enumerate(library.actions) if b == a)
                elif strategy == "random":
                    selected = int(rng.choice(valid))
                else:
                    def rank(i):
                        q = advance(env.pose, library[i])
                        distance = np.hypot(q.x-problem.goal.x, q.y-problem.goal.y)
                        angle = abs(np.arctan2(np.sin(q.th-problem.goal.th), np.cos(q.th-problem.goal.th)))
                        return distance+contract.limits.r_min_ack*angle
                    selected = int(min(valid, key=rank))
                ids.append(selected)
                obs, reward, done, truncated, info = env.step(selected)
                reward_sum += reward
                if done or truncated:
                    break
            row = {"task_id": problem.task_id, "strategy": strategy, "seed": seed,
                   "wall_s": time.perf_counter()-begun, "info": dict(info, **env.diagnostics()),
                   "reward_sum": reward_sum, "initial_potential": phi0,
                   "action_ids": ids, "actions": [asdict(a) for a in env.actions],
                   "start": asdict(problem.start), "goal": asdict(problem.goal)}
            if env.last_finalization:
                row["finalization"] = env.last_finalization.report(include_trace=True)
                if info["is_success"]:
                    row["reward_identity_error"] = reward_sum-(50.-info["execution_seconds"]/10.-phi0)
            results.append(row)
    return results


def run_mask_audit(contract, library, seed=20261026, samples_per_map=12):
    rng = np.random.default_rng(seed)
    results = []
    for problem, _ in engineering_tasks(contract):
        fast = reference = 0.
        valid_poses = allowed = 0
        for j in range(samples_per_map):
            p = problem.start if j == 0 else Pose(
                problem.start.x+rng.uniform(-.5, .5),
                problem.start.y+rng.uniform(-.5, .5), rng.uniform(-np.pi, np.pi))
            quick, a = valid_actions(problem, p, [], library)
            scalar, b = valid_actions(problem, p, [], library, reference=True)
            if not np.array_equal(quick, scalar):
                raise ValueError("mask_reference_mismatch:"+problem.task_id)
            fast += a["wall_s"]
            reference += b["wall_s"]
            valid_poses += int(not a.get("invalid_current_pose", False))
            allowed += int(quick.sum())
        preset, _ = valid_actions(problem, problem.start, [], library)
        finer_ids = list(np.flatnonzero(preset))[:2]
        for i in finer_ids:
            if not problem.grid.action_clear(problem.start, library[i], .0025):
                raise ValueError("finer_sweep_rejected_allowed_action:"+problem.task_id)
        warm_fast, warm_reference = [], []
        for _ in range(3):
            _, a = valid_actions(problem, problem.start, [], library)
            _, b = valid_actions(problem, problem.start, [], library, reference=True)
            warm_fast.append(a["wall_s"])
            warm_reference.append(b["wall_s"])
        results.append({"task_id": problem.task_id, "poses": samples_per_map,
                        "valid_current_poses": valid_poses,
                        "compared_action_ids": samples_per_map*len(library),
                        "allowed_actions": allowed, "fast_s": fast,
                        "reference_s": reference, "speedup": reference/max(fast, 1e-12),
                        "finer_sweep_checked_actions": len(finer_ids),
                        "warm_preset_fast_s": float(np.median(warm_fast)),
                        "warm_preset_reference_s": float(np.median(warm_reference)),
                        "warm_preset_speedup": float(np.median(warm_reference)/np.median(warm_fast))})
    return results


def plot_mask(results, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
    x = np.arange(len(results))
    ax.bar(x-.18, [1000*r["warm_preset_reference_s"] for r in results], .36,
           label="Scalar reference", color="#607d8b")
    ax.bar(x+.18, [1000*r["warm_preset_fast_s"] for r in results], .36,
           label="Batched SAT + conservative clearance", color="#0077b6")
    ax.set_xticks(x, [r["task_id"][:5] for r in results])
    ax.set(ylabel="Time for 80 actions [ms]", xlabel="Preset start (warm map cache)",
           title="Same 5 mm action mask; median of 3 runs per map")
    ax.legend(); ax.grid(axis="y", alpha=.25)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def plot_timing(report, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 2, figsize=(13, 11), constrained_layout=True)
    colors = {"legacy": "#607d8b", "v1": "#ef6c00", "v2": "#0077b6"}
    for row, (name, case) in enumerate(report["cases"].items()):
        ax, bx = axes[row]
        for method, stats in case["methods"].items():
            trace = np.asarray(stats["timed_trace"])
            ax.plot(trace[:, 0], trace[:, 4], label=method, color=colors[method])
            angular = np.unwrap(trace[:, 3])
            bx.plot(trace[:, 0], angular*180/np.pi, color=colors[method], label=method)
            for interval in stats["intervals"]:
                if interval["kind"] == "stationary":
                    ax.axvspan(interval["t0"], interval["t1"], color=colors[method], alpha=.12)
        ax.set(title=name+" — translation speed (shading = stationary hold)", xlabel="Time [s]", ylabel="Center speed [m/s]")
        bx.set(title=name+" — yaw (changing yaw is Spin, not a dwell)", xlabel="Time [s]", ylabel="Yaw [deg]")
        ax.legend(); bx.legend()
        ax.grid(alpha=.25); bx.grid(alpha=.25)
    fig.suptitle("Native timing functions: identical 0.3 s mode holds, different motion profiles")
    fig.savefig(output, dpi=150)
    plt.close(fig)


def plot_smoke(results, output, contract, library):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    fig, axes = plt.subplots(2, 2, figsize=(11, 9), constrained_layout=True)
    colors = ["#00a8e8", "#ff9400", "#b83cff"]
    known = [r for r in results if r["strategy"] == "known_valid"]
    tasks = {problem.task_id: problem for problem, _ in smoke_tasks(contract, library)}
    for ax, row in zip(axes.flat, known):
        grid = tasks[row["task_id"]].grid
        ax.imshow(grid.occ, origin="lower", cmap="Greys", vmin=0, vmax=1,
                  extent=[grid.ox, grid.ox+grid.width*grid.res,
                          grid.oy, grid.oy+grid.height*grid.res], interpolation="nearest")
        result = row.get("finalization", {})
        trace = np.asarray(result.get("timed_trace", []))
        if len(trace):
            for i in range(1, len(trace)):
                ax.plot(trace[i-1:i+1, 1], trace[i-1:i+1, 2], color=colors[int(trace[i, 5])], lw=3)
            for t, x, y, th, v, mode in trace[::max(1, len(trace)//14)]:
                c, s = np.cos(th), np.sin(th)
                hl, hw = contract.length/2, contract.width/2
                corners = np.array([[-hl,-hw], [hl,-hw], [hl,hw], [-hl,hw]])
                corners = corners@np.array([[c,s], [-s,c]])+np.array([x,y])
                ax.add_patch(Polygon(corners, fill=False, edgecolor=colors[int(mode)], alpha=.5))
        for key, color in (("start", "green"), ("goal", "red")):
            p = row[key]
            ax.arrow(p["x"], p["y"], .35*np.cos(p["th"]), .35*np.sin(p["th"]),
                     width=.035, color=color, length_includes_head=True)
        info = row["info"]
        ax.set(title=f'{row["task_id"]}: {info["reason"]}\nT={info.get("execution_seconds", 0):.3f} s; reward error={row.get("reward_identity_error", 0):.2g}',
               xlabel="x [m]", ylabel="y [m]", aspect="equal")
        if len(trace):
            ax.set_xlim(float(trace[:, 1].min())-.7, float(trace[:, 1].max())+.7)
            ax.set_ylim(float(trace[:, 2].min())-.7, float(trace[:, 2].max())+.7)
        ax.grid(alpha=.25); ax.margins(.25)
    fig.suptitle("P1 deterministic action replay + shared continuous finalization (not a learned policy)")
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    os.environ.setdefault("MPLCONFIGDIR", str(args.output/"matplotlib-cache"))
    contract = load_contract(args.config)
    library = ActionLibrary.from_contract(contract)
    manifest = {"schema": "rl_foundation_v1", "baselines": BASELINES,
                "python": platform.python_version(), "contract": asdict(contract),
                "training_environment_budget": asdict(EnvironmentConfig()),
                "smoke_environment_budget": asdict(EnvironmentConfig(max_decisions=24)),
                "contract_sha256": contract.identity, "action_library_sha256": library.identity,
                "commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True).strip(),
                "working_tree_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=PROJECT_ROOT)),
                "tasks": [entry for _, entry in engineering_tasks(contract)]}
    paths = list((PROJECT_ROOT/"rl/src/coptrans_rl").glob("*.py"))
    paths += list((PROJECT_ROOT/"catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer").glob("*.py"))
    paths += list((PROJECT_ROOT/"rl/configs").glob("*.yaml"))
    paths += [PROJECT_ROOT/"catkin_ws/src/fwis_common/config/ranger_mini_2.yaml",
              Path(args.config).resolve() if args.config else PROJECT_ROOT/"catkin_ws/src/fwis_v3_demo/config/params.yaml",
              PROJECT_ROOT/"rl/pyproject.toml", PROJECT_ROOT/"rl/requirements-foundation.lock",
              PROJECT_ROOT/"rl/run_checks.sh",
              PROJECT_ROOT/"catkin_ws/src/fwis_v3_playback/scripts/playback.py"]
    paths += list((PROJECT_ROOT/"rl/tests").glob("*.py"))
    paths += list((PROJECT_ROOT/"catkin_ws/src/fwis_v3_msgs/msg").glob("*.msg"))
    def display_source(path):
        try:
            return str(path.relative_to(PROJECT_ROOT))
        except ValueError:
            return str(path)
    manifest["source_files_sha256"] = {display_source(p): sha256_file(p) for p in sorted(set(paths))}
    manifest["manifest_sha256"] = stable_hash(manifest)
    write_json(args.output/"manifest.json", manifest)
    timing = run_timing_audit(contract)
    write_json(args.output/"native_timing.json", timing)
    plot_timing(timing, args.output/"native_timing.png")
    mask_audit = run_mask_audit(contract, library)
    write_json(args.output/"mask_audit.json", mask_audit)
    plot_mask(mask_audit, args.output/"mask_performance.png")
    smoke = run_smoke(contract, library)
    write_json(args.output/"environment_smoke.json", smoke)
    plot_smoke(smoke, args.output/"environment_smoke.png", contract, library)
    summary = {"valid_map_inputs": sum(e["start_footprint_valid"] and e["goal_footprint_valid"] for e in manifest["tasks"]),
               "total_maps": len(manifest["tasks"]),
               "native_mode_holds_all_match": all(s["event_holds_match"] for c in timing["cases"].values() for s in c["methods"].values()),
               "mask_compared_action_ids": sum(r["compared_action_ids"] for r in mask_audit),
               "mask_aggregate_speedup": sum(r["reference_s"] for r in mask_audit)/sum(r["fast_s"] for r in mask_audit),
               "smoke": [{"task": r["task_id"], "strategy": r["strategy"], **r["info"],
                          "wall_s": r["wall_s"]} for r in smoke]}
    write_json(args.output/"summary.json", summary)
    print(json.dumps(summary, indent=2))
    return 0 if (summary["valid_map_inputs"] == 11 and summary["native_mode_holds_all_match"] and all(
        r["info"]["is_success"] for r in smoke if r["strategy"] == "known_valid")) else 1


if __name__ == "__main__":
    raise SystemExit(main())
