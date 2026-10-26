"""Layout-isolated, certified 80-action curriculum data (not a blind benchmark)."""
import argparse
from dataclasses import asdict, replace
from functools import lru_cache
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .action_mask import valid_actions
from .configuration import load_settings
from .environment import PlanningEnv
from .kernel_adapter import PROJECT_ROOT, Grid, Pose, advance, replay, stable_hash, finalize_candidate

RANGES = {"train": (0, 10000), "val": (10000, 11000),
          "test": (20000, 21000), "ood": (30000, 31000)}
FAMILIES = ("open", "corridor", "corner", "parking")


def artifact_directory(path):
    path = Path(path).resolve()
    if path == PROJECT_ROOT or PROJECT_ROOT in path.parents:
        raise ValueError("training_artifacts_must_be_outside_repository")
    path.mkdir(parents=True, exist_ok=False)
    return path


def make_layout(seed, family):
    rng = np.random.default_rng(seed)
    size, resolution = 160, .05
    occupied = np.zeros((size, size), dtype=bool)
    if family == "corridor":
        occupied[:] = True
        y = int(rng.integers(35, 105)); width = int(rng.integers(19, 31))
        x = int(rng.integers(12, 28)); end = int(rng.integers(130, 150))
        occupied[y:y+width, x:end] = False
    elif family == "corner":
        occupied[:] = True
        x, y = int(rng.integers(16, 36)), int(rng.integers(16, 36))
        width = int(rng.integers(28, 43)); end = int(rng.integers(125, 145))
        occupied[y:y+width, x:end] = False
        occupied[y:145, end-width:end] = False
    elif family == "parking":
        for y in (35, 110):
            for x in (25, 55, 85, 115):
                width, height = int(rng.integers(8, 17)), int(rng.integers(15, 28))
                occupied[y:y+height, x:x+width] = True
    else:
        count = 3 if family == "open" else 15
        for _ in range(count):
            x, y = rng.integers(18, 135, size=2)
            width, height = rng.integers(5, 17, size=2)
            occupied[y:y+height, x:x+width] = True
    occupied[:2] = occupied[-2:] = True
    occupied[:, :2] = occupied[:, -2:] = True
    identity = stable_hash({"occupied_sha256": hashlib.sha256(occupied.tobytes()).hexdigest(),
                            "resolution": resolution, "origin": [0., 0.], "shape": list(occupied.shape)})
    return occupied, resolution, identity


def propose_task(settings, grid, seed):
    rng = np.random.default_rng(seed+9781)
    free = np.argwhere(~grid.occ)
    start = None
    for _ in range(400):
        y, x = free[int(rng.integers(len(free)))]
        heading = float(rng.choice([0., np.pi/2, np.pi, -np.pi/2]))
        candidate = Pose((x+.5)*grid.res, (y+.5)*grid.res, heading)
        if grid.pose_clear(candidate):
            start = candidate
            break
    if start is None:
        raise ValueError("no_valid_start_sample")
    provisional = settings.contract.problem(grid, start, start, **settings.problem_kwargs)
    steps = 1 if rng.random() < .4 else int(rng.integers(2, 7))
    ids, actions, pose = [], [], start
    for _ in range(steps):
        mask, _ = valid_actions(provisional, pose, actions, settings.library)
        legal = np.flatnonzero(mask)
        if not len(legal):
            raise ValueError("witness_no_legal_action")
        weights = []
        for i in legal:
            a = settings.library[i]
            weight = (3. if a.mode == 0 else 1. if a.mode == 1 else .5)
            weight *= .35 if a.mode != 2 and a.amount < 0 else 1.
            if actions and a.mode != actions[-1].mode:
                weight *= .6
            weights.append(weight)
        weights = np.asarray(weights); weights /= weights.sum()
        i = int(rng.choice(legal, p=weights))
        ids.append(i); actions.append(settings.library[i]); pose = advance(pose, actions[-1])
    goal = pose
    if (np.hypot(goal.x-start.x, goal.y-start.y) <= .02 and
            abs(np.arctan2(np.sin(goal.th-start.th), np.cos(goal.th-start.th))) <= np.pi/180):
        raise ValueError("trivial_witness_goal")
    # A teacher must end at the first goal occurrence, matching env termination.
    for j, q in enumerate(replay(start, actions)[1:]):
        if (np.hypot(q.x-goal.x, q.y-goal.y) <= .02 and
                abs(np.arctan2(np.sin(q.th-goal.th), np.cos(q.th-goal.th))) <= np.pi/180):
            ids, actions = ids[:j+1], actions[:j+1]
            break
    problem = replace(provisional, goal=goal)
    result = finalize_candidate(problem, actions, settings.finalization,
                                deadline=time.perf_counter()+settings.environment.finalization_timeout_s,
                                source="constructive_80_action_teacher")
    if not result.feasible:
        raise ValueError("teacher_finalization:"+str(result.reason))
    return problem, ids, result


def generate(output, counts, settings=None):
    settings = settings or load_settings()
    output = artifact_directory(output)
    (output/"layouts").mkdir()
    seen, tasks, attempts, layouts = set(), [], [], {}
    try:
        for split, count in counts.items():
            if split not in RANGES or not isinstance(count, int) or count < 0:
                raise ValueError("invalid_dataset_split_count")
            accepted = 0
            begin, end = RANGES[split]
            for seed in range(begin, end):
                if accepted == count:
                    break
                family = "islands" if split == "ood" else FAMILIES[(seed-begin) % len(FAMILIES)]
                occupied, resolution, layout_id = make_layout(seed, family)
                row = {"split": split, "seed": seed, "family": family, "layout_id": layout_id}
                if layout_id in seen:
                    attempts.append(dict(row, status="duplicate_layout"))
                    continue
                # Reserve every proposed layout in its first split, including
                # failed teachers, to avoid cross-split retry contamination.
                seen.add(layout_id)
                grid = Grid(occupied, resolution, 0., 0., settings.contract.length,
                            settings.contract.width, settings.contract.margin)
                try:
                    problem, ids, certificate = propose_task(settings, grid, seed)
                except ValueError as exc:
                    attempts.append(dict(row, status="rejected_curriculum_proposal", reason=str(exc)))
                    continue
                filename = "layouts/"+layout_id+".npz"
                np.savez_compressed(output/filename, occupied=occupied, resolution=resolution,
                                    origin=np.array([0., 0.]))
                layouts[layout_id] = dict(row, file=filename,
                    file_sha256=hashlib.sha256((output/filename).read_bytes()).hexdigest())
                task = dict(row, task_id=split+"_"+layout_id[:16],
                            start=asdict(problem.start), goal=asdict(problem.goal),
                            teacher_source="constructive_80_action_teacher",
                            action_ids=ids, certificate=certificate.report(),
                            connectivity="certified_vehicle_witness")
                tasks.append(task)
                attempts.append(dict(row, status="accepted"))
                accepted += 1
                if accepted % 25 == 0 or accepted == count:
                    print(split, accepted, "/", count, flush=True)
            if accepted != count:
                raise ValueError("seed_range_exhausted:"+split)
        text = "".join(json.dumps(row, sort_keys=True, allow_nan=False)+"\n" for row in tasks)
        (output/"tasks.jsonl").write_text(text)
        manifest = {"schema": "certified_curriculum_v1", "scope": "reachable_curriculum_not_blind_generalization",
                    "teacher": "constructive_same_library_not_three_classical_teachers",
                    "contract_sha256": settings.contract.identity,
                    "action_library_sha256": settings.library.identity,
                    "settings_sha256": settings.identity, "counts": counts,
                    "seed_ranges": RANGES, "layouts": layouts,
                    "tasks_sha256": hashlib.sha256(text.encode()).hexdigest(),
                    "attempt_count": len(attempts), "proposal_rejections": sum(
                        row["status"] != "accepted" for row in attempts)}
        manifest["dataset_sha256"] = stable_hash(manifest)
        (output/"manifest.json").write_text(json.dumps(manifest, indent=2, allow_nan=False)+"\n")
        return manifest
    finally:
        (output/"generation_attempts.jsonl").write_text("".join(
            json.dumps(row, sort_keys=True)+"\n" for row in attempts))


class Corpus:
    def __init__(self, directory, settings=None):
        self.directory = Path(directory).resolve()
        self.settings = settings or load_settings()
        self.manifest = json.loads((self.directory/"manifest.json").read_text())
        identity = {key: value for key, value in self.manifest.items() if key != "dataset_sha256"}
        if stable_hash(identity) != self.manifest.get("dataset_sha256"):
            raise ValueError("dataset_manifest_hash_mismatch")
        if self.manifest["schema"] != "certified_curriculum_v1":
            raise ValueError("unsupported_dataset_schema")
        if (self.manifest["contract_sha256"] != self.settings.contract.identity or
                self.manifest["action_library_sha256"] != self.settings.library.identity or
                self.manifest["settings_sha256"] != self.settings.identity):
            raise ValueError("dataset_vehicle_or_action_contract_mismatch")
        text = (self.directory/"tasks.jsonl").read_text()
        if hashlib.sha256(text.encode()).hexdigest() != self.manifest["tasks_sha256"]:
            raise ValueError("dataset_tasks_hash_mismatch")
        self.rows = [json.loads(row) for row in text.splitlines()]

    def split(self, name):
        return [row for row in self.rows if row["split"] == name]

    @lru_cache(maxsize=32)
    def grid(self, layout_id):
        meta = self.manifest["layouts"][layout_id]
        path = (self.directory/meta["file"]).resolve()
        if self.directory not in path.parents:
            raise ValueError("dataset_layout_path_escape")
        if hashlib.sha256(path.read_bytes()).hexdigest() != meta["file_sha256"]:
            raise ValueError("dataset_layout_file_hash_mismatch")
        with np.load(path, allow_pickle=False) as data:
            occupied = data["occupied"].copy()
            resolution = float(data["resolution"])
            ox, oy = (float(x) for x in data["origin"])
        c = self.settings.contract
        return Grid(occupied, resolution, ox, oy, c.length, c.width, c.margin)

    def problem(self, row):
        return self.settings.contract.problem(
            self.grid(row["layout_id"]), Pose(**row["start"]), Pose(**row["goal"]),
            task_id=row["task_id"], **self.settings.problem_kwargs)


def make_environment(directory, split="train", max_decisions=None):
    corpus = Corpus(directory)
    rows = corpus.split(split)
    if not rows:
        raise ValueError("empty_dataset_split:"+split)
    factory = lambda rng: corpus.problem(rows[int(rng.integers(len(rows)))])
    config = corpus.settings.environment
    if max_decisions is not None:
        config = replace(config, max_decisions=max_decisions)
    return PlanningEnv(factory, corpus.settings.library, config, corpus.settings.finalization)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--train-tasks", type=int, default=1000)
    parser.add_argument("--val-tasks", type=int, default=200)
    parser.add_argument("--test-tasks", type=int, default=500)
    parser.add_argument("--ood-tasks", type=int, default=100)
    args = parser.parse_args()
    result = generate(args.output, {"train": args.train_tasks, "val": args.val_tasks,
                                    "test": args.test_tasks, "ood": args.ood_tasks})
    print(json.dumps({key: result[key] for key in ("counts", "attempt_count", "proposal_rejections", "dataset_sha256")}, indent=2))


if __name__ == "__main__":
    main()
