"""The public test runner must select and preserve scene layouts before launch."""

from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[4]


def dry_run(*args):
    return subprocess.run(
        ["bash", str(ROOT / "scripts/test.sh"), "--dry-run", *args],
        cwd=ROOT, check=False, capture_output=True, text=True,
    )


@pytest.mark.parametrize(("args", "variant"), [
    ((), "west_single_lane"),
    (("--scene-variant", "critical"), "critical"),
    (("--scene-variant", "too_narrow"), "too_narrow"),
    (("--scene-variant", "default"), "default"),
    (("--scenario", "port_01"), "default"),
])
def test_variant_is_part_of_resolved_experiment(args, variant):
    process = dry_run("--task", "multi", "--experiment", "full_route", *args)
    assert process.returncode == 0, process.stderr
    assert yaml.safe_load(process.stdout)["scene_variant"] == variant


def test_layout_from_another_scenario_is_rejected():
    process = dry_run("--scenario", "port_01", "--scene-variant", "critical")
    assert process.returncode != 0
    assert "is not available for port_01" in process.stderr


def test_replay_will_not_override_an_archived_layout():
    process = dry_run("--replay", "20260101_000000_replay", "--scene-variant", "critical")
    assert process.returncode != 0
    assert "uses the recorded scene layout" in process.stderr


@pytest.mark.parametrize("task", ["single", "multi"])
def test_new_experiments_use_qualified_backend_with_explicit_rollback(task):
    current = dry_run("--task", task)
    rollback = dry_run("--task", task, "--architecture-profile", "baseline",
                       "--tracker-implementation", "python")
    assert current.returncode == rollback.returncode == 0
    current, rollback = (yaml.safe_load(run.stdout) for run in (current, rollback))
    assert (current["architecture_profile"], current["tracking_implementation"]) == ("optimized", "cpp")
    assert (rollback["architecture_profile"], rollback["tracking_implementation"]) == ("baseline", "python")
    if task == "single":
        assert current["planner_backend"] == "single_state_lattice"
        assert current["planner_overrides"]["search"]["adaptive_reverse_competition"]
        assert rollback["planner_backend"] == "single_continuous_mode_path"
    else:
        assert current["planner_backend"] == rollback["planner_backend"] == "multi_4wis"
        for option in ("native_frontier", "convex_corridor", "reachability_retime"):
            assert current["planner_overrides"][option]
            assert not rollback["planner_overrides"].get(option, False)


def test_optimized_features_can_be_retested_in_isolation():
    process = dry_run("--task", "multi", "--no-multi-convex-corridor", "--no-multi-reachable-clock")
    assert process.returncode == 0, process.stderr
    overrides = yaml.safe_load(process.stdout)["planner_overrides"]
    assert overrides["native_frontier"]
    assert overrides["convex_corridor"] is False
    assert overrides["reachability_retime"] is False


def test_explicit_planner_override_still_works():
    process = dry_run("--task", "single", "--planner-backend", "continuous_mode_path")
    assert process.returncode == 0, process.stderr
    assert yaml.safe_load(process.stdout)["planner_backend"] == "single_continuous_mode_path"


def test_interactive_entry_gets_the_same_optimized_defaults():
    process = subprocess.run(
        ["bash", str(ROOT / "scripts/interactive.sh"), "--dry-run"],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )
    config = yaml.safe_load(process.stdout)
    assert config["interactive"] is True
    assert config["planner_backend"] == "multi_4wis"
    assert config["tracking_implementation"] == "cpp"
    assert all(config["planner_overrides"][feature]
               for feature in ("native_frontier", "convex_corridor", "reachability_retime"))
