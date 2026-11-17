"""Assemble ``results.json`` and ``git_state.txt`` from the C1 run.

The git state is taken before the C1 commit; this is recorded truthfully as
``uncommitted``.  The actual commit SHA is reported in the handoff message,
because a commit cannot contain its own hash.
"""
import json
import re
import subprocess
import sys


def command(text):
    return subprocess.check_output(
        text, shell=True, text=True, stderr=subprocess.STDOUT).strip()


def test_summary(path):
    with open(path) as handle:
        text = handle.read()
    match = re.search(r"Ran (\d+) tests? in", text)
    return {
        "tests": int(match.group(1)) if match else None,
        "ok": bool(re.search(r"^OK$", text, re.MULTILINE)),
        "log": path.rsplit("/", 1)[-1],
    }


def main(outdir):
    head = command("git rev-parse HEAD")
    branch = command("git branch --show-current")
    status = command("git status --porcelain=v1")
    diff_stat = command("git diff --stat")
    with open(outdir + "/git_state.txt", "w") as handle:
        handle.write("base_commit: 23fd16227e75a68f03f3ac7483555063c8189313\n")
        handle.write("current_head_at_evidence: {}\n".format(head))
        handle.write("branch: {}\n".format(branch))
        handle.write("worktree: uncommitted at evidence generation\n")
        handle.write("\nstatus --porcelain=v1:\n")
        for line in status.splitlines():
            handle.write(line + "\n")
        handle.write("\ndiff --stat:\n")
        handle.write(diff_stat + "\n")

    with open(outdir + "/measurements.json") as handle:
        measurements = json.load(handle)
    with open(outdir + "/sample_chain.json") as handle:
        sample = json.load(handle)

    tests = {}
    for name in (
            "test_virtual_a_corridor.log",
            "test_virtual_a_pipeline.log",
            "test_virtual_a_optimizer.log",
            "test_virtual_a_audit.log",
            "test_virtual_a_collision.log"):
        tests[name.split(".")[0]] = test_summary(outdir + "/" + name)

    results = {
        "stage": "C1",
        "run_utc": outdir.rsplit("/", 1)[-1],
        "base_commit": "23fd16227e75a68f03f3ac7483555063c8189313",
        "current_head_at_evidence": head,
        "branch": branch,
        "worktree_at_evidence": "uncommitted",
        "environment": measurements["environment"],
        "scope": {
            "implemented": [
                "route_processing.segment_is_free (exact closed segment/AABB)",
                "route_processing.simplify_route (verified greedy LOS)",
                "contracts.ConvexCorridor / contracts.CorridorChain",
                "corridor.build_corridor_chain (separating halfspaces)",
                "corridor.audit_corridor_chain (polygon-cell SAT)",
            ],
            "not_implemented": [
                "NLP/corridor guidance integration (C2)",
                "ROS/RViz publication",
                "three-vehicle corridor safety claim",
            ],
        },
        "tests": tests,
        "categories": measurements["categories"],
        "explicit_rejections": measurements["explicit_rejections"],
        "fifty_by_fifty": measurements["fifty_by_fifty"],
        "sample": {
            "file": "sample_chain.json",
            "case": sample["case"],
            "chain_digest": sample["chain"]["digest"],
            "route_points": len(sample["route"]["points"]),
            "region_count": len(sample["chain"]["regions"]),
            "portal_count": len(sample["audit"]["portals"]),
            "audit_passed": sample["audit"]["passed"],
            "hand_b_expected": sample["hand_check"]["b_expected"],
            "hand_b_rows": len(sample["hand_check"]["matching_A_b_rows"]),
        },
        "failures_and_not_run": [
            "real ROS master, planner node, RViz and publication: NOT_RUN",
            "full unittest discover: NOT_RUN (headers of historical evidence "
            "could be overwritten)",
            "NLP/corridor guidance and C2: NOT_RUN",
            "historical timing_continuous KeyError dynamic_peaks: unchanged, "
            "still a failure",
            "50x50 timing is one sample on this machine, not a performance "
            "promise",
        ],
        "claims": {
            "reference_point_geometry_only": True,
            "accepts_three_vehicle_safety_claim": False,
            "geometry_changed": False,
            "dynamics_changed": False,
            "collision_changed": False,
            "ros_config_changed": False,
        },
    }
    with open(outdir + "/results.json", "w") as handle:
        json.dump(results, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps({
        "results": outdir + "/results.json",
        "tests": tests,
        "sample_digest": results["sample"]["chain_digest"],
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise SystemExit("usage: assemble_results.py <evidence-dir>")
    main(sys.argv[1])