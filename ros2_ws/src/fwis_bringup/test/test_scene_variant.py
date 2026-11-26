"""The public test runner must select and preserve scene layouts before launch."""

from pathlib import Path
import subprocess

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[4]
REGISTRY = ROOT / "ros2_ws/src/fwis_bringup/config/test_defaults.yaml"


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
    config = yaml.safe_load(process.stdout)
    assert config["scene_variant"] == variant
    assert config["interactive"] is False


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
    registry = yaml.safe_load(REGISTRY.read_text())
    default_alias = registry["defaults"]["planners"][task]
    default_backend = registry["planners"][task][default_alias]
    current = dry_run("--task", task)
    rollback = dry_run("--task", task, "--experiment", "full_route", "--planner-backend", "legacy",
                       "--architecture-profile", "baseline", "--tracker-implementation", "python")
    assert current.returncode == rollback.returncode == 0
    current, rollback = (yaml.safe_load(run.stdout) for run in (current, rollback))
    assert (current["architecture_profile"], current["tracking_implementation"]) == ("optimized", "cpp")
    assert (rollback["architecture_profile"], rollback["tracking_implementation"]) == ("baseline", "python")
    assert current["planner_backend"] == default_backend
    assert rollback["planner_backend"] == registry["planners"][task]["legacy"]
    if task == "multi":
        for option in ("native_frontier", "convex_corridor", "reachability_retime"):
            assert not rollback["planner_overrides"].get(option, False)


def test_optimized_features_can_be_retested_in_isolation():
    process = dry_run("--task", "multi", "--planner-backend", "legacy",
                      "--no-multi-convex-corridor", "--no-multi-reachable-clock")
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
    ordinary = dry_run("--task", "multi")
    assert ordinary.returncode == 0, ordinary.stderr
    default = yaml.safe_load(ordinary.stdout)
    assert default["interactive"] is True
    assert default["experiment"] == "interactive"
    assert "replay_endpoints" not in default
    process = subprocess.run(
        ["bash", str(ROOT / "scripts/interactive.sh"), "--dry-run"],
        cwd=ROOT, check=True, capture_output=True, text=True,
    )
    config = yaml.safe_load(process.stdout)
    assert config["interactive"] is True
    config["planner_overrides"].pop("publish_uniform_progress", None)
    for key in ("planner_backend", "planner_overrides", "tracking_implementation"):
        assert config[key] == default[key]
