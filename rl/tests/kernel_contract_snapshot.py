"""Serialize common-kernel results in each interpreter for parity comparison."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import platform
import sys

import numpy as np
import scipy

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"src"))
from coptrans_rl.kernel_adapter import load_contract, Grid, Pose, Action, replay, finalize_candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    contract = load_contract()
    grid = Grid(np.zeros((160, 160), dtype=bool), .05, 0., 0.,
                contract.length, contract.width, contract.margin)
    start = Pose(3., 3., 0.)
    cases = {"A": [Action(0, 2., 0.)],
             "O": [Action(1, .738, math.pi/2)],
             "Spin": [Action(2, math.pi/2, 0.)],
             "A_controls": [Action(0, .738, .5/contract.limits.r_min_ack)]*2,
             "A_Spin_A": [Action(0, .738, 0.), Action(2, math.pi/2, 0.), Action(0, .738, 0.)]}
    output = {"runtime": {"python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__},
              "contract_sha256": contract.identity, "cases": {}}
    for name, actions in cases.items():
        problem = contract.problem(grid, start, replay(start, actions)[-1])
        result = finalize_candidate(problem, actions)
        if not result.feasible:
            raise RuntimeError((name, result.reason, result.audit))
        output["cases"][name] = {"duration_s": result.timed.duration,
                                "mode_switch_hold_s": result.timed.mode_switch_hold_seconds,
                                "end_pose": asdict(result.timed.samples[-1][1]),
                                "score": result.audit["score"]}
    args.output.write_text(json.dumps(output, indent=2, allow_nan=False)+"\n")
    print(output["runtime"])


if __name__ == "__main__":
    main()
