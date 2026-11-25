"""Regression checks for public build identity and portable V3 evidence."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys

import pytest

from source_fingerprint import RUNTIME_ARTIFACTS, installed_fingerprint


@pytest.mark.parametrize("merged", [False, True])
def test_installed_fingerprint_detects_replaced_or_missing_planner(tmp_path, merged):
    for package, names in RUNTIME_ARTIFACTS.items():
        prefix = tmp_path if merged else tmp_path / package
        for name in names:
            path = prefix / name
            path.parent.mkdir(exist_ok=True, parents=True)
            path.write_bytes(b"current")
    before = installed_fingerprint(tmp_path)
    prefix = tmp_path if merged else tmp_path / "fwis_hybrid_joint_ros"
    planner = prefix / "lib/fwis_hybrid_joint_ros/hybrid_joint_planner"
    original = planner.stat()
    planner.write_bytes(b"earlier")
    os.utime(planner, ns=(original.st_atime_ns, original.st_mtime_ns))
    assert installed_fingerprint(tmp_path) != before
    planner.unlink()
    with pytest.raises(FileNotFoundError):
        installed_fingerprint(tmp_path)


def test_missing_raw_inputs_preserve_published_measurements(tmp_path):
    script = Path(__file__).with_name("accept_hybrid_v3.py")
    published = tmp_path / "measurements.json"
    published.write_text('{"previous_measurement": true}\n')
    result = subprocess.run([sys.executable, script, "--batch", tmp_path / "absent",
                             "--report", tmp_path], capture_output=True, text=True)
    assert result.returncode != 0
    assert "--compact" in result.stderr
    assert published.read_text() == '{"previous_measurement": true}\n'


def test_failed_public_start_is_archived(tmp_path):
    root = Path(__file__).resolve().parents[1]
    scripts = tmp_path / "scripts"
    scripts.mkdir()
    entry = scripts / "test.sh"
    shutil.copy2(root / "scripts/test.sh", entry)
    (tmp_path / "ros2_ws").mkdir()
    (tmp_path / "ros2_ws/src").symlink_to(root / "ros2_ws/src", target_is_directory=True)
    env = dict(os.environ, DISPLAY="", WAYLAND_DISPLAY="")
    result = subprocess.run([entry, "--task", "single", "--no-build"], env=env,
                            capture_output=True, text=True)
    assert result.returncode == 2
    manifests = list((tmp_path / "reports/runs").glob("*/experiment.json"))
    assert len(manifests) == 1
    record = json.loads(manifests[0].read_text())
    assert record["status"] == "launch_failed"
    assert record["reason"] in ("display_missing", "ros_setup_missing")
