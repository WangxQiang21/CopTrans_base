"""Record the host, package, git and run-directory environment."""
import datetime
import json
import os
import platform
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]
RUN = Path(__file__).resolve().parents[1]


def package_version(name):
    try:
        module = __import__(name)
    except Exception as exc:
        return {"import_error": "{}: {}".format(type(exc).__name__, exc)}
    return {
        "version": getattr(module, "__version__", None),
        "file": getattr(module, "__file__", None),
    }


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(ROOT), check=True,
        capture_output=True, text=True).stdout


def main():
    staged = git("diff", "--cached", "--binary")
    payload = {
        "schema": "virtual_rigid_a.g3.environment.v1",
        "run_id": RUN.name,
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "repo_root": str(ROOT),
        "host": {
            "platform": platform.platform(),
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "cpu_count": os.cpu_count(),
        },
        "python": {
            "version": sys.version,
            "executable": sys.executable,
            "implementation": platform.python_implementation(),
        },
        "packages": {
            name: package_version(name)
            for name in ("numpy", "scipy", "casadi", "yaml")},
        "environment": {
            key: os.environ.get(key)
            for key in ("PYTHONDONTWRITEBYTECODE", "OPENBLAS_NUM_THREADS",
                        "PYTHONPATH", "OMP_NUM_THREADS")},
        "git": {
            "head": git("rev-parse", "HEAD").strip(),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD").strip(),
            "status_short": git("status", "--short").splitlines(),
            "staged_bytes": len(staged.encode("utf-8")),
            "committed": "NO",
            "pushed": "NO",
            "branch_created": "NO",
        },
    }
    print(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False))


if __name__ == "__main__":
    main()