#!/usr/bin/env python3
"""Compare the Phase 1 single planner switch on fixed difficult ROS cases."""
from argparse import ArgumentParser
from contextlib import ExitStack
import json
from pathlib import Path
import sys
from tempfile import TemporaryDirectory

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import yaml

from bench_single_ros import FLAGS, PlannerCase, config, merge

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/fwis_backends/test"))
from test_state_lattice import LATTICE_CASES, make_grid

NAMES = {"reverse_only", "spin_in_place", "side_goal", "right_angle",
         "s_corridor", "disconnecting_wall"}


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--base-domain", type=int, default=170)
    args = parser.parse_args()
    base = config()
    share = Path(get_package_share_directory("fwis_backends"))
    prefix = Path(get_package_prefix("fwis_backends"))
    binaries = {role: prefix / "lib" / "fwis_backends" / f"single_continuous_mode_path_{role}"
                for role in ("planner", "optimizer")}
    vehicle_path = Path(get_package_share_directory("fwis_common")) / "config/ranger_mini_2.yaml"
    vehicle = yaml.safe_load(vehicle_path.read_text())
    defaults = yaml.safe_load((share / "config/single_continuous_mode_path/params.yaml").read_text())

    report = {}
    cases = [case for case in LATTICE_CASES if case[0] in NAMES]
    for index, (name, start, goal, obstacles, *_) in enumerate(cases):
        request = dict(base, start=[start.x, start.y, start.th], goal=[goal.x, goal.y, goal.th])
        cells = (100 * make_grid(obstacles).occ).astype("int8")
        report[name] = {}
        for variant, (label, flags) in enumerate(FLAGS.items()):
            with TemporaryDirectory(prefix="coptrans-single-fixtures-") as temp, ExitStack() as stack:
                directory = Path(temp)
                parameters = merge(vehicle, defaults, request["planner_overrides"], {
                    "search": {"adaptive_reverse_competition": flags["adaptive_reverse_competition"]},
                    "quality": {"seed_proposal_wait_s": flags["seed_proposal_wait_s"],
                                "report_dir": str(directory / "plans")},
                    "use_sim_time": False, "map_transient_local": True,
                })
                domain = args.base_domain + index * len(FLAGS) + variant
                case = PlannerCase(label, domain, binaries, request, parameters, directory, stack)
                case.ready(cells, .05)
                row = case.trial(16.)
                report[name][label] = row
                print(name, label, row["result"], row["wall_s"], row.get("mode_sequence"), flush=True)
        args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")


if __name__ == "__main__":
    main()
