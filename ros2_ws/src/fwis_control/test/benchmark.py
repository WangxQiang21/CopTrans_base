#!/usr/bin/env python3
"""Paired tracking comparison using stepped Harmonic servers and actual ECM pose."""
import argparse
from collections import Counter, defaultdict, deque
from concurrent.futures import ProcessPoolExecutor
import csv
from dataclasses import asdict, dataclass, replace
import gzip
import json
import math
import multiprocessing
import os
from pathlib import Path
import random
from tempfile import TemporaryDirectory
from time import perf_counter

import yaml

from fwis_control.config import load
from fwis_control.model import Mode, Observation, Pose, Twist
from fwis_control.trajectory import preflight
from metrics import score, summarize
from scenarios import from_document, suite


@dataclass(frozen=True)
class Trial:
    case: object
    algorithm: str
    condition: str
    repetition: int
    seed: int
    disturbance: dict


TERMINAL = ("COMPLETE", "STOPPED", "REJECTED", "STALE_TRAJECTORY")


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")


def disturbances(condition, seed, amplitude):
    if condition == "nominal":
        return dict(dx=0., dy=0., dyaw=0., initial_vx=0., initial_wz=0.,
                    response_scale=1., command_delay=0., feedback_delay=0.)
    rng = random.Random(seed)
    speed = rng.uniform(*amplitude["initial_vx"])
    return dict(dx=rng.uniform(*amplitude["initial_xy"]), dy=rng.uniform(*amplitude["initial_xy"]),
                dyaw=math.radians(rng.uniform(*amplitude["initial_yaw_deg"])), initial_vx=speed,
                initial_wz=abs(speed)*rng.uniform(*amplitude["initial_curvature"]),
                response_scale=rng.uniform(*amplitude["response_scale"]),
                command_delay=rng.uniform(*amplitude["command_delay"]),
                feedback_delay=rng.uniform(*amplitude["feedback_delay"]))


class SteppedGazebo:
    def __init__(self, config_path, plant_path, output):
        # Import transport / simulator in each independently spawned process.
        os.environ["GZ_PARTITION"] = f"coptrans_tracking_benchmark_{os.getpid()}"
        from gz.math7 import Pose3d
        from gz.sim8 import Model, TestFixture, World, world_entity
        from ranger_mini_gz.body_sim import GazeboBody, PlantSettings, classify_twist
        self.Pose3d, self.Model, self.World, self.world_entity = Pose3d, Model, World, world_entity
        self.GazeboBody, self.classify_twist = GazeboBody, classify_twist
        self.config = load(config_path)
        self.plant = PlantSettings(**yaml.safe_load(Path(plant_path).read_text())["plant"])
        self.output = Path(output)
        self.step_size = 1. / self.plant.frequency
        self.tmp = TemporaryDirectory(prefix="ranger-trial-")
        world = Path(self.tmp.name) / "stepped.sdf"
        world.write_text(f"""<sdf version="1.9">
  <world name="ranger_tracking_benchmark">
    <physics name="fast" type="ignored">
      <max_step_size>{self.step_size:.12g}</max_step_size>
      <real_time_factor>1000</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <model name="ranger1"><static>true</static><link name="base_link"/></model>
  </world>
</sdf>""")
        self.fixture = TestFixture(str(world))
        self.fixture.on_pre_update(self.pre_update)
        self.fixture.on_post_update(self.post_update)
        self.fixture.finalize()
        self.server = self.fixture.server()

    def run(self, trial):
        self.engine = self.config.execution(trial.algorithm)
        self.trial = trial
        self.dis = trial.disturbance
        self.start = None
        self.body = None
        self.rows = []
        self.commands = deque()
        self.states = deque()
        self.command, self.request = (0., 0., 0.), None
        self.done = False
        count = math.ceil((trial.case.trajectory.duration + self.config.timing.final_stop_timeout
                           + .20) / self.step_size) + 2
        error = None
        try:
            if not self.server.run(True, count, False):
                raise RuntimeError("Gazebo run returned false")
        except Exception as exc:
            error = f"{type(exc).__name__}: {exc}"
        return self.save(error)

    def pre_update(self, info, ecm):
        if self.done:
            return
        now = info.sim_time.total_seconds()
        if self.body is None:
            world = self.World(self.world_entity(ecm))
            entity = world.model_by_name(ecm, "ranger1")
            scale = self.dis["response_scale"]
            self.body = self.GazeboBody(entity, ecm, replace(
                self.plant, linear_tau=self.plant.linear_tau*scale, angular_tau=self.plant.angular_tau*scale))
            p = self.trial.case.trajectory.points[0].pose
            self.body.model.set_world_pose_cmd(ecm, self.Pose3d(
                p.x+self.dis["dx"], p.y+self.dis["dy"], .322, 0., 0., p.yaw+self.dis["dyaw"]))
        elif self.start is None:
            velocity = (self.dis["initial_vx"], 0., self.dis["initial_wz"])
            self.body.response.velocity = velocity
            self.body.pre_update(now, self.step_size, ecm, velocity)
            return
        while self.commands and self.commands[0][0] <= now + 1e-8:
            _, self.command, self.request = self.commands.popleft()
        self.body.pre_update(now, self.step_size, ecm, self.command, self.request)

    def post_update(self, info, ecm):
        if self.done:
            return
        now = info.sim_time.total_seconds()
        state = self.body.post_update(now, ecm)
        if state is None:
            return
        if self.start is None:
            if not self.states:
                self.states.append(state)
                return
            self.start = now
            self.next_tick = now
            self.engine.admit(self.trial.case.trajectory, Mode(state.mode), now, replay_start=now)
        self.states.append(state)
        if now + 1e-8 < self.next_tick:
            return
        self.next_tick += self.config.timing.period
        cutoff = now - self.dis["feedback_delay"] + 1e-8
        while len(self.states) > 1 and self.states[1].stamp <= cutoff:
            self.states.popleft()
        sensor = self.states[0]
        observed = self.observation(sensor)
        measured = self.observation(state)
        start = perf_counter()
        output = self.engine.tick(now, observed)
        cost = perf_counter() - start
        self.commands.append((now + self.dis["command_delay"], output.command.values(), output.requested_mode))
        self.rows.append(self.record(now, output, measured, observed, cost))
        self.done = output.phase in TERMINAL

    @staticmethod
    def observation(state):
        p = state.pose.pos()
        return Observation(Pose(p.x(), p.y(), state.pose.rot().yaw()), Twist(*state.velocity),
                           state.stamp, Mode(state.mode), state.stamp)

    def record(self, now, output, actual, feedback, cost):
        r = output.reference
        if r is None:
            error_pos = error_yaw = error_speed = error_wz = None
        else:
            ex, ey, error_yaw = actual.pose.error_to(r.pose)
            error_pos = math.hypot(ex, ey)
            c, s = math.cos(error_yaw), math.sin(error_yaw)
            ref_vx, ref_vy, ref_wz = r.twist.values()
            error_speed = math.hypot(c*ref_vx-s*ref_vy-actual.twist.vx, s*ref_vx+c*ref_vy-actual.twist.vy)
            error_wz = abs(ref_wz-actual.twist.wz)
        x, y, w = output.command.values()
        classified = self.classify_twist((x, y, w), self.plant.switch_radius)[0] if (x, y, w) != (0., 0., 0.) else None
        mode = self.engine.filter.mode
        limits = self.config.limits
        violation = (math.hypot(x, y) > limits.max_speed + 1e-9 or abs(w) > limits.max_yaw_rate+1e-9
                     or (mode == Mode.OBLIQUE and (w != 0 or classified == Mode.ACKERMANN))
                     or (mode == Mode.SPIN and (x != 0 or y != 0)))
        return dict(time_s=now-self.start, x=actual.pose.x, y=actual.pose.y, yaw=actual.pose.yaw,
                    vx=actual.twist.vx, vy=actual.twist.vy, wz=actual.twist.wz, phase=output.phase,
                    planned_mode=int(r.mode) if r else None, observed_mode=int(actual.mode),
                    feedback_mode=int(feedback.mode), feedback_age_s=now-feedback.stamp,
                    requested_mode=int(output.requested_mode) if output.requested_mode is not None else None,
                    command_mode=int(mode) if mode is not None else None,
                    classified_command=classified, command_violation=int(violation),
                    ref_x=r.pose.x if r else None, ref_y=r.pose.y if r else None, ref_yaw=r.pose.yaw if r else None,
                    ref_vx=r.twist.vx if r else None, ref_vy=r.twist.vy if r else None, ref_wz=r.twist.wz if r else None,
                    raw_vx=output.raw.vx, raw_vy=output.raw.vy, raw_wz=output.raw.wz,
                    cmd_vx=x, cmd_vy=y, cmd_wz=w, reasons="|".join(output.reasons),
                    position_error=error_pos, heading_error=error_yaw, speed_error=error_speed,
                    yaw_rate_error=error_wz, compute_s=cost)

    def save(self, error):
        t = self.trial
        key = f"{t.case.name}__{t.algorithm}__{t.condition}__{t.repetition:02d}"
        raw_path = f"runs/{key}.csv.gz"
        if self.rows:
            with gzip.open(self.output / raw_path, "wt", newline="") as file:
                writer = csv.DictWriter(file, fieldnames=self.rows[0].keys())
                writer.writeheader()
                writer.writerows(self.rows)
        terminal = self.rows[-1]["phase"] if self.rows else None
        measurements = score(self.rows, self.config.timing.period, self.config.limits.guard_radius,
                             len(self.engine.switches) if hasattr(self.engine, "switches") else 0)
        if error:
            outcome = "simulation_error"
        elif terminal == "COMPLETE":
            outcome = "complete"
            if (measurements["ack_command_guard_violations"] or measurements["mode_mismatches"]
                    or measurements["command_envelope_violations"] or measurements["unintended_spin_commands"]):
                outcome = "constraint_or_mode_violation"
        elif terminal in TERMINAL:
            outcome = (self.rows[-1]["reasons"] or terminal).lower()
        else:
            outcome = "benchmark_time_budget"
        result = dict(case=t.case.name, source=t.case.source, family=t.case.family,
                      plan_id=t.case.trajectory.plan_id, algorithm=t.algorithm, condition=t.condition,
                      repetition=t.repetition, seed=t.seed, disturbance=t.disturbance,
                      outcome=outcome, error=error, raw=raw_path if self.rows else None,
                      metrics=measurements)
        dump(self.output / f"runs/{key}.json", result)
        return result


def worker(task):
    config_path, plant_path, output, trials = task
    sim = SteppedGazebo(config_path, plant_path, output)
    return [sim.run(trial) for trial in trials]


def stats(rows):
    complete = [r for r in rows if r["outcome"] == "complete"]
    names = sorted({name for row in rows for name in row["metrics"]})
    return dict(count=len(rows), completed=len(complete), completion_rate=len(complete)/len(rows),
                outcomes=dict(Counter(row["outcome"] for row in rows)),
                metrics={name: summarize([r["metrics"][name] for r in rows if name in r["metrics"]])
                         for name in names})


def comparisons(results):
    by_group, by_source, by_mode, by_all, pairs = (defaultdict(list) for _ in range(5))
    for row in results:
        case, source, family, algorithm, condition = (
            row[key] for key in ("case", "source", "family", "algorithm", "condition"))
        by_group[(case, algorithm, condition)].append(row)
        by_source[(source, algorithm, condition)].append(row)
        by_mode[(source, family, algorithm, condition)].append(row)
        by_all[(algorithm, condition)].append(row)
        pairs[(case, condition, row["repetition"], row["seed"])].append(row)
    paired = defaultdict(list)
    for (case, condition, repetition, seed), rows in pairs.items():
        keyed = {row["algorithm"]: row for row in rows}
        if set(keyed) != {"nonlinear", "pid"} or any(row["outcome"] != "complete" for row in rows):
            continue
        for field in ("position_m_rmse", "yaw_rad_rmse", "speed_mps_rmse", "yaw_rate_radps_rmse",
                      "filter_fraction", "compute_s_p99"):
            delta = keyed["nonlinear"]["metrics"][field] - keyed["pid"]["metrics"][field]
            paired[(case, condition, field)].append(delta)
        assert keyed["nonlinear"]["disturbance"] == keyed["pid"]["disturbance"]
    return dict(per_case={"|".join(k): stats(v) for k, v in sorted(by_group.items())},
                per_source={"|".join(k): stats(v) for k, v in sorted(by_source.items())},
                per_mode={"|".join(k): stats(v) for k, v in sorted(by_mode.items())},
                overall={"|".join(k): stats(v) for k, v in sorted(by_all.items())},
                paired_nonlinear_minus_pid={"|".join(k): {**summarize(v), "n": len(v), "nonlinear_lower": sum(x < 0 for x in v)}
                                            for k,v in sorted(paired.items())})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    here = Path(__file__).resolve().parent
    parser.add_argument("--config", type=Path, default=here.parent / "config/ranger_tracking.yaml")
    parser.add_argument("--plant", type=Path, default=here.parent.parent / "ranger_mini_gz/config/body_twist.yaml")
    parser.add_argument("--conditions", type=Path, default=here / "disturbances.yaml")
    parser.add_argument("--captures", type=Path, nargs="*", default=[])
    parser.add_argument("--expect-rejection", nargs="*", default=[],
                        help="Native capture case names expected to fail preflight")
    parser.add_argument("--cases", nargs="*", help="Run only specified passing case names")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--workers", type=int, default=6)
    args = parser.parse_args()
    if min(args.repeats, args.workers) < 1:
        parser.error("workers and repeats must be positive")
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "trajectories").mkdir()
    (args.output / "runs").mkdir()
    settings = load(args.config)
    for path, name in ((args.config, "controller.yaml"), (args.plant, "plant.yaml"),
                       (args.conditions, "disturbances.yaml")):
        (args.output / name).write_bytes(path.read_bytes())
    condition_config = yaml.safe_load(args.conditions.read_text())
    scenarios = suite()
    scenarios.extend(from_document(json.loads(path.read_text())) for path in args.captures)
    names = {case.name for case in scenarios}
    if len(names) != len(scenarios):
        parser.error("duplicate scenario name")
    if set(args.expect_rejection) - names:
        parser.error("unknown case in --expect-rejection")
    cases, checks = [], []
    for case in scenarios:
        if case.name in args.expect_rejection:
            case = replace(case, expected_pass=False)
        dump(args.output / f"trajectories/{case.name}.json", case.document())
        result = preflight(case.trajectory, settings.limits, settings.preflight, settings.timing.minimum_hold)
        checks.append(dict(name=case.name, source=case.source, family=case.family,
                           plan_id=case.trajectory.plan_id, result=result.result,
                           duration_s=case.trajectory.duration, expected_pass=case.expected_pass,
                           switches=[asdict(switch) for switch in result.switches],
                           issues=[asdict(issue) for issue in result.issues]))
        if result.sampler is not None and (not args.cases or case.name in args.cases):
            cases.append(case)
    dump(args.output / "preflight.json", checks)
    unexpected = [item["name"] for item in checks if (item["result"] != "FAIL") != item["expected_pass"]]
    if unexpected:
        parser.error(f"unexpected preflight outcomes, see preflight.json: {unexpected}")
    if not cases:
        parser.error("no selected cases passed preflight")
    trials = []
    for index, case in enumerate(cases):
        for repetition in range(args.repeats):
            seed = condition_config["base_seed"] + index*10000 + repetition
            for condition in ("nominal", "perturbed"):
                delta = disturbances(condition, seed, condition_config["perturbed"])
                trials.extend(Trial(case, algorithm, condition, repetition, seed, delta)
                              for algorithm in ("nonlinear", "pid"))
    partitions = [trials[index::args.workers] for index in range(args.workers)]
    tasks = [(str(args.config), str(args.plant), str(args.output), part) for part in partitions if part]
    start = perf_counter()
    with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context("spawn")) as pool:
        results = [row for group in pool.map(worker, tasks) for row in group]
    results.sort(key=lambda r: (r["source"], r["case"], r["condition"], r["repetition"], r["algorithm"]))
    summary = comparisons(results)
    summary["elapsed_wall_s"] = perf_counter() - start
    summary["preflight_counts"] = dict(Counter(item["result"] for item in checks))
    summary["rejection_cases"] = {
        "expected_fail": sum(not case["expected_pass"] for case in checks),
        "rejected": sum(not case["expected_pass"] and case["result"] == "FAIL" for case in checks)}
    dump(args.output / "summary.json", summary)
    with (args.output / "trials.csv").open("w", newline="") as file:
        keys = ("case", "source", "family", "plan_id", "algorithm", "condition", "repetition", "seed", "outcome", "raw")
        metrics = sorted({name for row in results for name in row["metrics"]})
        writer = csv.DictWriter(file, fieldnames=[*keys, *metrics])
        writer.writeheader()
        for row in results:
            writer.writerow({**{key: row[key] for key in keys}, **row["metrics"]})
    print(f"preflight {summary['preflight_counts']}; {len(results)} trials in {summary['elapsed_wall_s']:.1f} s")
    for name, item in summary["overall"].items():
        print(name, item["completed"], "/", item["count"], item["outcomes"])


if __name__ == "__main__":
    main()
