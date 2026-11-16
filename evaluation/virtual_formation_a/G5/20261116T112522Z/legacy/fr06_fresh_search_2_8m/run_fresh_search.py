#!/usr/bin/env python3
"""G5-C fresh 2.8 m no-seed FR06 autonomous search.

The frozen FR06 scene supplies only the corridor grid, start pose/headings,
goal pose, vehicle shapes, formation model and the fixed search/successor
configuration.  No reference path, edge, seed or trajectory is passed to
`plan_uniform`.  The original FR06 experiment runner object is imported
unmodified, but its output root is redirected to this new G5 run directory so
no protected historical artifact is written.
"""
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[5]
FR06 = ROOT / "evaluation" / "formation_refactor_v02" / "FR06"
sys.path.insert(0, str(FR06))
sys.path.insert(0, str(ROOT / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"))

import fr06_scene as scene  # noqa: E402
import run_experiments as runner  # noqa: E402
from fwis_v3_optimizer.uniform_search import GraphSearchConfig  # noqa: E402

# Redirect the original runner's writes into this new G5 run directory.
runner.HERE = HERE
(HERE / "plans").mkdir(exist_ok=True)
(HERE / "runs").mkdir(exist_ok=True)

LABEL = "g5_fresh_tight_2.8m_no_seed_attempt1"
PLAN_ID = 6204


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    successor = scene.successor_config()
    search_config = GraphSearchConfig()
    payload = runner._run(
        scene.TIGHT_WIDTH_M, PLAN_ID, LABEL)
    map_dir = FR06 / "maps" / "corridor_w2.8"
    map_files = {
        name: sha256_file(map_dir / name)
        for name in ("map.pgm", "map.yaml") if (map_dir / name).is_file()
    }
    record = {
        "schema": "virtual_rigid_a.g5.fr06_fresh_search.v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "scope": (
            "fresh autonomous search on the frozen FR06 2.8 m corridor; "
            "no reference path, edge, seed or trajectory supplied"),
        "host": {
            "platform": platform.platform(),
            "machine": platform.machine(),
            "python": platform.python_version(),
        },
        "scene": {
            "map_dir": str(map_dir.relative_to(ROOT)),
            "map_files_sha256": map_files,
            "corridor_width_m": scene.TIGHT_WIDTH_M,
            "start_qf": list(scene.START_QF),
            "goal_qf": list(scene.GOAL_QF),
            "formation_id": scene.FORMATION_ID,
        },
        "config": {
            "successor_config": asdict(successor),
            "search_config": asdict(search_config),
            "max_expansions": search_config.max_expansions,
            "max_seconds": search_config.max_seconds,
        },
        "attempt": payload,
        "reference_used_as_input": False,
        "seed_supplied": False,
    }
    (HERE / "result.json").write_text(
        json.dumps(record, sort_keys=True, indent=2, allow_nan=False),
        encoding="utf-8")
    print(json.dumps({
        "label": payload["label"],
        "status": payload["status"],
        "feasible": payload["feasible"],
        "executable": payload["executable"],
        "reason": payload["reason"],
        "wall_time_s": payload["wall_time_s"],
        "t_end_s": payload["t_end_s"],
        "plan_model_sha256": payload["plan_model_sha256"],
        "expansions": None if payload["search"] is None else payload["search"]["expansions"],
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())
