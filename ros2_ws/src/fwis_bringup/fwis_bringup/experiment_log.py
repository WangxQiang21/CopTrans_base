"""Local CopTrans experiment archives and input replay, independent of ROS."""

import argparse
from datetime import datetime
import gzip
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import shutil
import tempfile

import yaml


TOTAL_BUDGET = 5_000_000_000
RUN_BUDGET = 4_000_000
BLOCK_SIZE = 4096
METADATA_HEADROOM = 32_768
SCHEMA = 1
ID_PATTERN = re.compile(r"\d{8}_\d{6}_[A-Za-z0-9_-]+\Z")


def read_config(path):
    text = path.read_text()
    # PyYAML's YAML 1.1 resolver treats JSON exponents such as 1e-05 as strings.
    if text.lstrip().startswith("{"):
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            pass
    return yaml.safe_load(text)


def allocated(path):
    if path.is_symlink():
        return 0
    try:
        used = path.stat().st_blocks * 512
        return used + sum(allocated(child) for child in path.iterdir()) if path.is_dir() else used
    except FileNotFoundError:
        return 0


def remaining_bytes(root):
    return max(0, TOTAL_BUDGET - allocated(root))


def _round_blocks(size):
    return ((size + BLOCK_SIZE - 1) // BLOCK_SIZE) * BLOCK_SIZE


class ArchiveWriter:
    def __init__(self, destination):
        self.destination = destination
        self.total_free = max(0, remaining_bytes(destination.parent) - METADATA_HEADROOM)
        self.run_used = allocated(destination)

    def write(self, name, data, *, essential=False):
        target = self.destination / name
        if target.is_file() and target.stat().st_size == len(data) and target.read_bytes() == data:
            return True
        missing = [parent for parent in target.parents if not parent.exists()]
        cost = _round_blocks(len(data)) + BLOCK_SIZE * len(missing)
        if cost > self.total_free or (not essential and self.run_used + cost + METADATA_HEADROOM > RUN_BUDGET):
            return False
        target.parent.mkdir(parents=True, exist_ok=True)
        old = allocated(target)
        # Write on the destination filesystem, then atomically replace the last checkpoint.
        with tempfile.NamedTemporaryFile(dir=target.parent, delete=False) as handle:
            path = Path(handle.name)
            try:
                handle.write(data)
                handle.flush()
                os.replace(path, target)
            finally:
                path.unlink(missing_ok=True)
        diff = allocated(target) - old + sum(parent.stat().st_blocks * 512 for parent in missing)
        self.total_free -= diff
        self.run_used += diff
        return True


def _digest_file(path):
    return sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def collect(spool, destination, *, final=False):
    """Checkpoint an experiment while its GUI can stay open, or after process exit."""
    writer = ArchiveWriter(destination)
    omissions = []

    def persist(name, *, essential=False, compressed=False):
        source = spool / name
        if not source.is_file():
            return False
        archived_name = name + (".gz" if compressed else "")
        if essential or source.stat().st_size <= 24_000_000:
            data = source.read_bytes()
            if compressed:
                data = gzip.compress(data, compresslevel=5, mtime=0)
            if writer.write(archived_name, data, essential=essential):
                return True
        omissions.append(archived_name)
        return False

    for name in ("result.yaml", "selected_endpoints.yaml", "resolved_config.yaml", "environment.txt", "command.txt",
                 "planner_parameters.yaml", "controller_parameters.yaml", "scene/body_twist_parameters.yaml",
                 "scene/map.yaml", "scene/scene_meta.yaml"):
        persist(name, essential=True)
    map_digest = _digest_file(spool / "scene" / "map.pgm")
    scene_digest = _digest_file(spool / "scene" / "scene_meta.yaml")
    persist("scene/map.pgm", essential=True, compressed=True)

    frozen_plan = None
    if persist("trajectories/planner.cdr", essential=True, compressed=True):
        frozen_plan = "trajectories/planner.cdr.gz"
    elif persist("trajectories/model.json", essential=True, compressed=True):
        frozen_plan = "trajectories/model.json.gz"

    for name in ("timing_trace.jsonl", "operator_events.jsonl", "planning_progress.jsonl", "launch.log",
                 "planner.log", "controller.log", "metrics/motion.jsonl", "metrics/fault_tail.jsonl", "build.log"):
        persist(name, compressed=name != "timing_trace.jsonl")

    # A full bag remains an explicit debug option and uses the same project quota.
    bag = spool / "bag"
    saved_bag = destination / "bag"
    if final and (bag / "metadata.yaml").is_file() and not saved_bag.exists():
        estimate = sum(_round_blocks(p.stat().st_size) if p.is_file() else BLOCK_SIZE for p in bag.rglob("*"))
        if estimate + BLOCK_SIZE + METADATA_HEADROOM <= writer.total_free:
            staged_bag = destination / ".bag.pending"
            try:
                shutil.copytree(bag, staged_bag)
                staged_bag.replace(saved_bag)
                writer.total_free -= allocated(saved_bag)
            finally:
                if staged_bag.is_dir():
                    shutil.rmtree(staged_bag)
        else:
            omissions.append("bag")

    old_manifest = destination / "experiment.json"
    initial = json.loads(old_manifest.read_text()) if old_manifest.is_file() else {}
    result_path = spool / "result.yaml"
    result = read_config(result_path) if result_path.is_file() else {}
    raw_config = spool / "resolved_config.yaml"
    config = read_config(raw_config) if raw_config.is_file() else {}
    environment_path = destination / "environment.txt"
    environment = dict(line.split("=", 1) for line in environment_path.read_text().splitlines()
                       if "=" in line) if environment_path.is_file() else {}
    manifest = {"schema": SCHEMA, "experiment_id": destination.name,
                "started_at": initial.get("started_at") or datetime.now().astimezone().isoformat(),
                "status": result.get("status", "started"), "reason": result.get("reason"),
                "detail": result.get("detail"), "scenario": result.get("scenario", config.get("scenario")),
                "scene_variant": config.get("scene_variant", "default"),
                "planner_backend": config.get("planner_backend"),
                "planner_source": result.get("planner_source"),
                "architecture_profile": config.get("architecture_profile"),
                "tracking_implementation": config.get("tracking_implementation"),
                "start": result.get("start") if config.get("interactive") else config.get("start"),
                "goal": result.get("goal") if config.get("interactive") else config.get("goal"),
                "plan_artifact": frozen_plan, "map_sha256": map_digest, "scene_sha256": scene_digest,
                "model_sha256": result.get("model_sha256"),
                "request_submitted": result.get("planner_wall_s") is not None,
                "replay_of": config.get("replay_of"),
                "git_commit": environment.get("git_commit"), "git_dirty": environment.get("git_dirty"),
                "omitted_by_budget": omissions}
    if result:
        manifest["finished_at"] = initial.get("finished_at") or datetime.now().astimezone().isoformat()
    content = json.dumps(manifest, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    if not writer.write("experiment.json", content.encode(), essential=True):
        raise ValueError("not enough local archive capacity to write experiment.json")
    if (spool / "trajectories/planner.cdr").is_file() and not frozen_plan:
        raise ValueError("not enough local archive capacity to retain the planned path")
    required = {"result.yaml", "selected_endpoints.yaml", "resolved_config.yaml", "scene/map.pgm.gz"}
    if required.intersection(omissions):
        raise ValueError("not enough local archive capacity to retain required experiment data")
    return manifest


def replay_config(root, experiment_id):
    if not ID_PATTERN.fullmatch(experiment_id):
        raise ValueError("invalid experiment ID")
    directory = root / experiment_id
    if directory.is_symlink() or not (directory / "experiment.json").is_file():
        raise ValueError(f"local experiment not found: {experiment_id}")
    manifest = json.loads((directory / "experiment.json").read_text())
    if manifest.get("schema") != SCHEMA or manifest.get("experiment_id") != experiment_id:
        raise ValueError("experiment archive metadata does not match the requested ID")
    if (not _pose(manifest.get("start")) or not _pose(manifest.get("goal"))
            or not isinstance(manifest.get("map_sha256"), str)):
        raise ValueError("this experiment has no complete endpoints and map to replay")
    if manifest.get("request_submitted") is False:
        raise ValueError("this experiment ended before the planning request was submitted")
    config = read_config(directory / "resolved_config.yaml")
    if config["scenario"] != manifest["scenario"] or config["mode"] != "sim":
        raise ValueError("archived scenario or simulation mode does not match")
    config.setdefault("scene_variant", "default")
    if config.get("interactive"):
        config["replay_endpoints"] = {key: manifest[key] for key in ("start", "goal")}
    else:
        config.update(start=manifest["start"], goal=manifest["goal"])
    config.update(replay_of=experiment_id, replay_map_sha256=manifest["map_sha256"],
                  replay_scene_sha256=manifest.get("scene_sha256"),
                  replay_git_commit=manifest.get("git_commit"), record="off")
    for key, filename in (("replay_planner", "planner_parameters.yaml"),
                          ("replay_controller", "controller_parameters.yaml"),
                          ("replay_plant", "scene/body_twist_parameters.yaml")):
        path = directory / filename
        if path.is_file():
            config[key] = read_config(path)
    return config


def _pose(value):
    return (isinstance(value, list) and len(value) == 3
            and all(type(v) in (int, float) and math.isfinite(v) for v in value))


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    commands = cli.add_subparsers(dest="command", required=True)
    capacity = commands.add_parser("reserve")
    capacity.add_argument("root", type=Path)
    archive = commands.add_parser("collect")
    archive.add_argument("spool", type=Path)
    archive.add_argument("destination", type=Path)
    archive.add_argument("--final", action="store_true")
    args = cli.parse_args()
    try:
        if args.command == "reserve":
            free = remaining_bytes(args.root)
            if free < RUN_BUDGET + METADATA_HEADROOM:
                raise ValueError(f"local experiment quota has {free:,} bytes remaining (limit 5 GB). "
                                 "Move older logs to other storage before starting a new experiment.")
        elif args.command == "collect":
            manifest = collect(args.spool, args.destination, final=args.final)
            if manifest["omitted_by_budget"]:
                print("Local archive omissions: " + ", ".join(manifest["omitted_by_budget"]))
    except (OSError, ValueError) as error:
        cli.exit(2, f"Experiment log: {error}\n")


if __name__ == "__main__":
    main()
