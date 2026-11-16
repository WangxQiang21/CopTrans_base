"""Record the environment in which the G4 evidence was produced."""
import datetime
import json
import os
import platform
import subprocess
import sys
from pathlib import Path

RUN = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[5]


def version(module_name):
    try:
        module = __import__(module_name)
    except Exception as exc:  # pragma: no cover - environment report only
        return {"available": False,
                "error": "{}: {}".format(type(exc).__name__, exc)}
    return {
        "available": True,
        "version": getattr(module, "__version__", None),
    }


def first_cpu_model():
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except OSError:
        pass
    return None


def git(*args):
    return subprocess.run(
        ["git"] + list(args), cwd=str(REPO_ROOT), check=True,
        capture_output=True, text=True).stdout


def main():
    import casadi
    packages = {}
    for name in ("numpy", "scipy", "casadi", "matplotlib", "yaml",
                 "catkin_pkg", "PIL"):
        key = name.lower().replace("catkin_pkg", "catkin_pkg").replace(
            "pil", "pillow")
        packages[key] = version(name)
    document = {
        "schema": "virtual_rigid_a.g4.environment.v1",
        "generated_at_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "run_id": RUN.name,
        "host": {
            "platform": platform.platform(),
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "processor": platform.processor(),
            "cpu_model": first_cpu_model(),
            "cpu_count": os.cpu_count(),
            "python_version": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "python_executable": sys.executable,
            "python_full_version": sys.version,
        },
        "packages": packages,
        "solver": {
            "casadi_version": casadi.__version__,
            "ipopt_plugin_available": bool(casadi.has_nlpsol("ipopt")),
            "selected_solver": "ipopt",
            "used_for": "G2 geometry path construction only",
        },
        "process_environment": {
            "cwd": os.getcwd(),
            "PYTHONDONTWRITEBYTECODE": os.environ.get(
                "PYTHONDONTWRITEBYTECODE"),
            "OPENBLAS_NUM_THREADS": os.environ.get("OPENBLAS_NUM_THREADS"),
            "PYTHONPATH": os.environ.get("PYTHONPATH"),
        },
        "git": {
            "head": git("rev-parse", "HEAD").strip(),
            "branch": git("rev-parse", "--abbrev-ref", "HEAD").strip(),
            "status_short": git("status", "--short").splitlines(),
        },
        "date_utc": datetime.datetime.now(
            datetime.timezone.utc).date().isoformat(),
        "notes": [
            "python3 -B and PYTHONDONTWRITEBYTECODE=1 were used for every "
            "test, benchmark and assembler command",
            "OPENBLAS_NUM_THREADS=1 reduces thread nondeterminism in the "
            "linear algebra used by CasADi/IPOPT",
            "this is an offline finite-map G4 evidence run; it is not a ROS, "
            "Gazebo or hardware validation",
            "the six frozen scenes are known-feasible A-only examples, not a "
            "general completeness or performance sample",
            "the collision layer re-uses the unmodified old OBB swept checker "
            "with a 0.05 m baseline margin and 0.01 m corner step",
        ],
    }
    target = RUN / "environment.json"
    target.write_text(
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    print(json.dumps({
        "status": "WRITTEN",
        "path": str(target.relative_to(REPO_ROOT)),
        "python": platform.python_version(),
        "ipopt_available": document["solver"]["ipopt_plugin_available"],
        "packages": {name: record.get("version")
                     for name, record in packages.items()},
    }, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()