#!/usr/bin/env python3
"""Fingerprint build inputs without relying on Git HEAD or file timestamps."""

import hashlib
import os
from pathlib import Path
import stat
import subprocess
import sys


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
    print(fingerprint(Path(sys.argv[1])))
