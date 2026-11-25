#!/usr/bin/env python3
"""Fingerprint build inputs without relying on Git HEAD or file timestamps."""

import hashlib
import json
import os
from pathlib import Path
import stat
import subprocess
import sys


RUNTIME_ARTIFACTS = {
    "fwis_hybrid_joint": ("lib/libfwis_hybrid_joint.so",),
    "fwis_hybrid_joint_ros": ("lib/libhybrid_codec.so", "lib/fwis_hybrid_joint_ros/hybrid_joint_planner",
                              "lib/fwis_hybrid_joint_ros/hybrid_joint_executor"),
    "fwis_control": ("lib/fwis_control/ranger_tracking_controller_cpp",),
    "fwis_bringup": ("lib/fwis_bringup/run_experiment", "share/fwis_bringup/config/test_defaults.yaml",
                     "share/fwis_bringup/launch/test.launch.py"),
}


def installed_receipt(install):
    artifacts = {}
    for package, names in RUNTIME_ARTIFACTS.items():
        prefix = install / package if (install / package).is_dir() else install
        for name in names:
            with (prefix / name).open("rb") as handle:
                file_digest = hashlib.sha256()
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    file_digest.update(chunk)
            artifacts[f"{package}/{name}"] = file_digest.hexdigest()
    return artifacts


def receipt_fingerprint(artifacts):
    digest = hashlib.sha256(b"coptrans-release-install-v1\0")
    for package, names in RUNTIME_ARTIFACTS.items():
        for name in names:
            key = f"{package}/{name}"
            digest.update(key.encode() + b"\0")
            digest.update(bytes.fromhex(artifacts[key]))
    return digest.hexdigest()


def installed_fingerprint(install):
    return receipt_fingerprint(installed_receipt(install))


def fingerprint(root):
    sources = subprocess.check_output(
        ("git", "-C", root, "ls-files", "-z", "--cached", "--others",
         "--exclude-standard", "--", "ros2_ws/src"))
    digest = hashlib.sha256(b"coptrans-ros-source-v1\0")
    for name in sorted(set(sources.split(b"\0")) - {b""}):
        path = root / os.fsdecode(name)
        if path.suffix.lower() in (".md", ".rst"):
            continue
        digest.update(name + b"\0")
        try:
            mode = path.lstat().st_mode
        except FileNotFoundError:
            digest.update(b"missing\0")
            continue
        if stat.S_ISLNK(mode):
            digest.update(b"symlink\0" + os.fsencode(path.readlink()) + b"\0")
        elif stat.S_ISREG(mode):
            contents = hashlib.sha256()
            with path.open("rb") as handle:
                for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                    contents.update(chunk)
            digest.update(b"file\0" + bytes([mode & 0o111]) + contents.digest())
        else:
            raise ValueError(f"unsupported source file type: {path}")
    return digest.hexdigest()


if __name__ == "__main__":
    if sys.argv[1:2] == ["--installed-receipt"]:
        files = installed_receipt(Path(sys.argv[2]))
        print(receipt_fingerprint(files))
        print(json.dumps(files, sort_keys=True, separators=(",", ":")))
    elif sys.argv[1:2] == ["--installed"]:
        print(installed_fingerprint(Path(sys.argv[2])))
    else:
        print(fingerprint(Path(sys.argv[1])))
