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
