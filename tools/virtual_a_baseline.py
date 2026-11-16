#!/usr/bin/env python3
"""G0 baseline protection and safe regression entry for virtual formation A.

This tool deliberately contains no formation geometry or optimizer.  Its only
G0 computation is a fresh audit of the frozen FR06 reference against an
independently reconstructed request.  Historical scripts are never executed.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib
import inspect
import json
import math
import os
from pathlib import Path
import platform
import shlex
import subprocess
import sys
import time
import traceback


BASELINE_HEAD = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
EXPECTED_BRANCH = "feature/virtual-rigid-formation-a"
SCHEMA = "virtual_rigid_a.g0.baseline.v1"
TOOL_PATH = "tools/virtual_a_baseline.py"
INTERFACE_PATH = "docs/virtual_formation_a/G0_INTERFACE_CHECK.md"
TECHNICAL_REVIEW = "docs/VIRTUAL_FORMATION_A_FR06_TECHNICAL_REVIEW_20261115.md"
KNOWN_DESIGN_DOCUMENTS = frozenset({
    TECHNICAL_REVIEW,
    "docs/virtual_formation_a/CLAUDE_G0_TASK.md",
    "docs/virtual_formation_a/CLAUDE_G1_TASK.md",
    "docs/virtual_formation_a/CLAUDE_G2_TASK.md",
    "docs/virtual_formation_a/CLAUDE_G3_TASK.md",
    "docs/virtual_formation_a/CLAUDE_G4_TASK.md",
    "docs/virtual_formation_a/CLAUDE_G5_TASK.md",
    "docs/virtual_formation_a/SOURCE_REVIEW_PLAN_FR06_BASE.md",
    "docs/virtual_formation_a/VIRTUAL_FORMATION_A_DECISIONS.md",
    "docs/virtual_formation_a/VIRTUAL_FORMATION_A_IMPLEMENTATION_PLAN_FOR_CLAUDE.md",
    "docs/virtual_formation_a/WORKSPACE_PREPARATION.md",
})
DEFAULT_PROTECTED_PATHS = (
    "catkin_ws/src",
    "tests",
    "tools",
    "maps",
    "run_demo.sh",
    "run_test.sh",
    "evaluation/formation_refactor_v02",
)
REDIRECT_VARIABLES = (
    "FR01_RESULTS_PATH",
    "FR02_TIMING_RESULTS_PATH",
    "FR02_TRAJECTORY_RESULTS_PATH",
    "FR02_QUALITY_RESULTS_PATH",
    "FR02_ARTIFACT_DIR",
    "FR03_RESULTS_PATH",
    "FR03_ARTIFACT_DIR",
    "FR04_RESULTS_PATH",
    "FR04_ARTIFACT_DIR",
)
REGRESSION_CASES = {
    "fr01_uniform_motion": ("FR01", "test_uniform_motion.py"),
    "geometry_continuous": ("G1-light", "test_continuous_geometry.py"),
    "geometry_formation": ("G1-light", "test_formation_geometry.py"),
    "timing_continuous": ("G3-light", "test_continuous_retiming.py"),
    "timing_formation": ("G3-light", "test_formation_retiming.py"),
    "fr02_a_timing": ("FR02-A", "test_uniform_timing.py"),
    "fr02_a_trajectory": ("FR02-A", "test_uniform_trajectory.py"),
    "fr02_b_quality": ("FR02-B", "test_uniform_quality.py"),
    "fr03_uniform_search": ("FR03", "test_uniform_search.py"),
    "fr04_uniform_pipeline": ("FR04", "test_uniform_pipeline.py"),
}
REGRESSION_LEVELS = {
    "geometry": (
        "geometry_continuous",
        "geometry_formation",
    ),
    "timing": (
        "timing_continuous",
        "timing_formation",
        "fr02_a_timing",
        "fr02_a_trajectory",
        "fr02_b_quality",
    ),
    "full": (
        "geometry_continuous",
        "geometry_formation",
        "timing_continuous",
        "timing_formation",
        "fr01_uniform_motion",
        "fr02_a_timing",
        "fr02_a_trajectory",
        "fr02_b_quality",
        "fr03_uniform_search",
        "fr04_uniform_pipeline",
    ),
}
REGRESSION_STAGES = {
    "baseline": "G0",
    "geometry": "G1",
    "timing": "G3",
    "full": "G5",
}
STAGE_ORDER = ("G1", "G2", "G3", "G4", "G5")
PACKAGE_ROOT = (
    "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/"
    "formation_continuous_a/")
SETUP_PATH = "catkin_ws/src/fwis_v3_optimizer/setup.py"
# These are path authorizations only. Codex must still review whether each
# actual diff is the minimal change allowed by the corresponding stage task.
STAGE_NEW_FILES = {
    "G1": frozenset({
        *(PACKAGE_ROOT + name for name in (
            "__init__.py", "contracts.py", "geometry.py", "bounds.py")),
        "tests/test_virtual_a_geometry.py",
        "tests/test_virtual_a_profile.py",
        "tools/virtual_a_benchmark.py",
    }),
    "G2": frozenset({
        PACKAGE_ROOT + "optimizer.py",
        PACKAGE_ROOT + "audit.py",
        "tests/test_virtual_a_optimizer.py",
        "tests/test_virtual_a_audit.py",
    }),
    "G3": frozenset({
        PACKAGE_ROOT + "trajectory.py",
        PACKAGE_ROOT + "timing.py",
        "tests/test_virtual_a_timing.py",
        "tests/test_virtual_a_trajectory.py",
    }),
    "G4": frozenset({
        PACKAGE_ROOT + "collision.py",
        PACKAGE_ROOT + "pipeline.py",
        "tools/virtual_a_scenes.py",
        "tests/test_virtual_a_collision.py",
        "tests/test_virtual_a_pipeline.py",
    }),
    "G5": frozenset({
        PACKAGE_ROOT + "ros_adapter.py",
        "tests/test_virtual_a_ros_adapter.py",
        "tests/test_virtual_a_playback.py",
        "tests/ros_virtual_a_smoke.py",
    }),
}
STAGE_NEW_PREFIXES = {
    "G4": ("tests/fixtures/virtual_a/maps/",),
}
STAGE_TRACKED_FILES = {
    "G1": frozenset({SETUP_PATH}),
    "G5": frozenset({
        "catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py",
        "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py",
        "catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py",
        "catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py",
        "catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch",
        "run_demo.sh",
        "catkin_ws/src/fwis_v3_optimizer/CMakeLists.txt",
    }),
}


class BaselineError(RuntimeError):
    """An expected, explicitly reported G0 failure."""


def _utc_now():
    return datetime.now(timezone.utc).isoformat()


def _json_default(value):
    if isinstance(value, Path):
        return str(value)
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, (set, frozenset)):
        return sorted(value)
    raise TypeError("not JSON serializable: {!r}".format(value))


def _json_bytes(value):
    return (json.dumps(
        value, indent=2, sort_keys=True, ensure_ascii=False,
        allow_nan=False, default=_json_default) + "\n").encode("utf-8")


def _canonical_sha256(value):
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False, default=_json_default).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _sha256_file(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while True:
            block = stream.read(1024 * 1024)
            if not block:
                break
            digest.update(block)
    return digest.hexdigest()


def _write_json(path: Path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(_json_bytes(value))


def _write_text(path: Path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _git(repo_root: Path, *args, check=True):
    completed = subprocess.run(
        ["git", "-C", str(repo_root), *args], text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
    if check and completed.returncode != 0:
        raise BaselineError(
            "git {} failed ({}): {}".format(
                " ".join(args), completed.returncode, completed.stderr.strip()))
    return completed


def _cumulative_stage_values(mapping, stage):
    if stage == "G0":
        return ()
    if stage not in STAGE_ORDER:
        raise BaselineError("unknown workspace authorization stage: {}".format(stage))
    values = []
    for current in STAGE_ORDER:
        values.extend(mapping.get(current, ()))
        if current == stage:
            break
    return tuple(values)


def _is_stage_addition(path, stage):
    if path in _cumulative_stage_values(STAGE_NEW_FILES, stage):
        return True
    return any(path.startswith(prefix) for prefix in _cumulative_stage_values(
        STAGE_NEW_PREFIXES, stage))


def _is_stage_tracked_change(status_line, stage):
    """Allow only A/M changes to explicit stage paths; never D/R/U/type changes."""
    if len(status_line) < 4 or status_line.startswith("??"):
        return False
    xy, path = status_line[:2], status_line[3:]
    if xy[0] not in (" ", "M", "A") or xy[1] not in (" ", "M"):
        return False
    authorized_paths = set(_cumulative_stage_values(STAGE_NEW_FILES, stage))
    authorized_paths.update(_cumulative_stage_values(STAGE_TRACKED_FILES, stage))
    return path in authorized_paths


def _classify_untracked(path, stage="G0"):
    if path in KNOWN_DESIGN_DOCUMENTS:
        return "known_design"
    if path in (TOOL_PATH, INTERFACE_PATH):
        return "allowed_g0_addition"
    if path.startswith("evaluation/virtual_formation_a/"):
        return "allowed_new_evidence"
    if _is_stage_addition(path, stage):
        return "allowed_stage_addition"
    return "unknown"


def _stage_unknown_modifications(manifest, stage):
    """Recompute authorization from raw status, not a caller-supplied verdict."""
    tracked = manifest.get("git", {}).get("tracked_changes", [])
    unknown_tracked = [
        line for line in tracked if not _is_stage_tracked_change(line, stage)]
    untracked = manifest.get("untracked", {})
    candidates = list(untracked.get("unknown", []))
    candidates += list(untracked.get("allowed_stage_addition", []))
    unknown_untracked = [
        path for path in candidates
        if _classify_untracked(path, stage) == "unknown"
    ]
    return unknown_tracked + unknown_untracked


def _is_cache_path(relative: Path):
    return (
        any(part in ("__pycache__", ".pytest_cache") for part in relative.parts)
        or relative.suffix in (".pyc", ".pyo")
    )


def _protected_entries(repo_root: Path, protected_paths):
    entries = {}
    missing = []
    for raw_path in sorted(set(protected_paths)):
        relative_root = Path(raw_path)
        absolute_root = repo_root / relative_root
        if not absolute_root.exists() and not absolute_root.is_symlink():
            missing.append(relative_root.as_posix())
            continue
        if absolute_root.is_file() or absolute_root.is_symlink():
            candidates = (absolute_root,)
        else:
            candidates = absolute_root.rglob("*")
        for absolute in candidates:
            relative = absolute.relative_to(repo_root)
            relative_posix = relative.as_posix()
            if relative_posix == TOOL_PATH or _is_cache_path(relative):
                continue
            if absolute.is_symlink():
                target = os.readlink(str(absolute))
                entries[relative_posix] = {
                    "path": relative_posix,
                    "type": "symlink",
                    "size_bytes": 0,
                    "sha256": hashlib.sha256(target.encode("utf-8")).hexdigest(),
                    "symlink_target": target,
                }
            elif absolute.is_file():
                entries[relative_posix] = {
                    "path": relative_posix,
                    "type": "file",
                    "size_bytes": absolute.stat().st_size,
                    "sha256": _sha256_file(absolute),
                }
    return [entries[key] for key in sorted(entries)], missing


def collect_manifest(
        repo_root: Path, protected_paths: tuple[str, ...], stage: str = "G0") -> dict:
    """Hash protected inputs and classify dirty paths for one explicit stage."""
    if stage != "G0" and stage not in STAGE_ORDER:
        raise BaselineError("unknown workspace authorization stage: {}".format(stage))
    repo_root = repo_root.resolve()
    head = _git(repo_root, "rev-parse", "HEAD").stdout.strip()
    branch = _git(repo_root, "branch", "--show-current").stdout.strip()
    status = _git(repo_root, "status", "--porcelain=v1", "--untracked-files=all").stdout
    status_lines = [line for line in status.splitlines() if line]
    tracked_changes = [line for line in status_lines if not line.startswith("??")]
    authorized_tracked = [
        line for line in tracked_changes
        if _is_stage_tracked_change(line, stage)
    ]
    unauthorized_tracked = [
        line for line in tracked_changes
        if not _is_stage_tracked_change(line, stage)
    ]
    untracked_output = _git(
        repo_root, "ls-files", "--others", "--exclude-standard").stdout
    classifications = {
        "known_design": [],
        "allowed_g0_addition": [],
        "allowed_new_evidence": [],
        "allowed_stage_addition": [],
        "unknown": [],
    }
    for path in sorted(line for line in untracked_output.splitlines() if line):
        classifications[_classify_untracked(path, stage)].append(path)
    entries, missing = _protected_entries(repo_root, protected_paths)
    design_documents = {}
    for relative in sorted(KNOWN_DESIGN_DOCUMENTS | {INTERFACE_PATH}):
        path = repo_root / relative
        if path.is_file():
            design_documents[relative] = _sha256_file(path)
    ancestor = _git(
        repo_root, "merge-base", "--is-ancestor", BASELINE_HEAD, "HEAD",
        check=False)
    if ancestor.returncode not in (0, 1):
        raise BaselineError(ancestor.stderr.strip())
    unstaged_diff = _git(repo_root, "diff", "--no-ext-diff", "--binary").stdout
    staged_diff = _git(
        repo_root, "diff", "--cached", "--no-ext-diff", "--binary").stdout
    return {
        "schema": "virtual_rigid_a.g0.manifest.v1",
        "generated_at_utc": _utc_now(),
        "repo_root": str(repo_root),
        "workspace_authorization_stage": stage,
        "git": {
            "head": head,
            "branch": branch,
            "dirty": bool(status_lines),
            "status_porcelain": status_lines,
            "tracked_changes": tracked_changes,
            "baseline_head": BASELINE_HEAD,
            "baseline_is_ancestor": ancestor.returncode == 0,
            "unstaged_diff_sha256": hashlib.sha256(
                unstaged_diff.encode("utf-8")).hexdigest(),
            "staged_diff_sha256": hashlib.sha256(
                staged_diff.encode("utf-8")).hexdigest(),
            "unstaged_diff_stat": _git(repo_root, "diff", "--stat").stdout,
            "staged_diff_stat": _git(repo_root, "diff", "--cached", "--stat").stdout,
        },
        "protected_paths": list(protected_paths),
        "protected_exclusions": [TOOL_PATH, "**/__pycache__/**", "*.pyc", "*.pyo"],
        "missing_protected_paths": missing,
        "protected_file_count": len(entries),
        "protected_files": entries,
        "protected_tree_sha256": _canonical_sha256({
            entry["path"]: {
                "type": entry["type"],
                "sha256": entry["sha256"],
                "size_bytes": entry["size_bytes"],
            } for entry in entries
        }),
        "design_document_sha256": design_documents,
        "untracked": classifications,
        "authorized_stage_tracked_changes": authorized_tracked,
        "unknown_modifications": unauthorized_tracked + classifications["unknown"],
    }


def inspect_environment() -> dict:
    """Record actual imports and solve a tiny independent CasADi/IPOPT problem."""
    dependencies = {}
    for import_name, distribution in (
        ("numpy", "numpy"),
        ("scipy", "scipy"),
        ("casadi", "casadi"),
        ("yaml", "PyYAML"),
        ("PIL", "Pillow"),
        ("rospy", "rospy"),
    ):
        try:
            module = importlib.import_module(import_name)
        except Exception as exc:
            dependencies[distribution] = {
                "import_name": import_name,
                "status": "UNAVAILABLE",
                "error_type": type(exc).__name__,
                "error": str(exc),
            }
        else:
            dependencies[distribution] = {
                "import_name": import_name,
                "status": "AVAILABLE",
                "version": getattr(module, "__version__", "unknown"),
                "path": getattr(module, "__file__", "unknown"),
            }
    ipopt = {"status": "NOT_RUN_DEPENDENCY", "reason": "casadi unavailable"}
    if dependencies["casadi"]["status"] == "AVAILABLE":
        try:
            import casadi as ca
            x = ca.MX.sym("x")
            solver = ca.nlpsol(
                "virtual_a_g0_probe", "ipopt", {"x": x, "f": (x - 2.0) ** 2},
                {"print_time": False, "ipopt.print_level": 0, "ipopt.sb": "yes"})
            solution = solver(x0=0.0, lbx=-10.0, ubx=10.0)
            value = float(solution["x"])
            stats = solver.stats()
            success = bool(stats.get("success", False)) and abs(value - 2.0) <= 1e-8
            ipopt = {
                "status": "PASS" if success else "FAIL",
                "problem": "minimize (x-2)^2, -10 <= x <= 10",
                "solution_x": value,
                "absolute_error": abs(value - 2.0),
                "solver_success": bool(stats.get("success", False)),
                "return_status": stats.get("return_status", "unknown"),
                "iterations": stats.get("iter_count", None),
            }
        except Exception as exc:
            ipopt = {
                "status": "FAIL",
                "problem": "minimize (x-2)^2, -10 <= x <= 10",
                "error_type": type(exc).__name__,
                "error": str(exc),
            }
    repo_root = Path(__file__).resolve().parents[1]
    ros_noetic = Path("/opt/ros/noetic/setup.bash")
    catkin_setup = repo_root / "catkin_ws" / "devel" / "setup.bash"
    return {
        "schema": "virtual_rigid_a.g0.environment.v1",
        "generated_at_utc": _utc_now(),
        "python": {
            "executable": sys.executable,
            "version": sys.version,
            "implementation": platform.python_implementation(),
        },
        "platform": {
            "system": platform.system(),
            "release": platform.release(),
            "machine": platform.machine(),
            "platform": platform.platform(),
            "logical_cpu_count": os.cpu_count(),
        },
        "environment_delta_recommended": {
            "PYTHONDONTWRITEBYTECODE": "1",
            "OPENBLAS_NUM_THREADS": "1",
        },
        "dependencies": dependencies,
        "casadi_ipopt_probe": ipopt,
        "g2_nlp_dependency_blocked": ipopt["status"] != "PASS",
        "ros": {
            "noetic_setup": str(ros_noetic),
            "noetic_setup_exists": ros_noetic.is_file(),
            "catkin_devel_setup": str(catkin_setup),
            "catkin_devel_setup_exists": catkin_setup.is_file(),
            "ros_launch_attempted": False,
            "gazebo_launch_attempted": False,
            "note": "Import and setup-file checks do not constitute ROS playback verification.",
        },
        "not_run": [
            {"item": "old unit-test regression", "status": "NOT_RUN_G0_BASELINE"},
            {"item": "FR06 autonomous searches", "status": "NOT_RUN_G0_EXCLUDED"},
            {"item": "ROS/Gazebo", "status": "NOT_RUN_G0_EXCLUDED"},
            {"item": "G1-G5 implementation and validation", "status": "NOT_RUN_STAGE_GATE"},
        ],
    }


def _add_package_to_sys_path(repo_root: Path):
    package_src = repo_root / "catkin_ws" / "src" / "fwis_v3_optimizer" / "src"
    if str(package_src) not in sys.path:
        sys.path.insert(0, str(package_src))
    return package_src


def reaudit_fr06_reference(repo_root: Path, width_m: float = 2.8) -> dict:
    """Re-audit frozen FR06 using constants from the original request contract."""
    repo_root = repo_root.resolve()
    package_src = _add_package_to_sys_path(repo_root)
    import numpy as np
    from PIL import Image
    from fwis_v3_optimizer.core import Grid
    from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
    from fwis_v3_optimizer.formation_collision import VehicleShape
    from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
    from fwis_v3_optimizer.uniform_quality import (
        UniformAuditRequest, audit_uniform_timed_plan)
    from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan

    fr06 = repo_root / "evaluation" / "formation_refactor_v02" / "FR06"
    model_path = fr06 / "reference" / "reference_model.json"
    summary_path = fr06 / "reference" / "reference_summary.json"
    acceptance_path = fr06 / "CODEX_ACCEPTANCE.json"
    map_path = fr06 / "maps" / ("corridor_w{:.1f}".format(width_m)) / "map.pgm"
    model_bytes = model_path.read_bytes()
    identity = json.loads(model_bytes.decode("utf-8"))
    frozen_summary = json.loads(summary_path.read_text(encoding="utf-8"))
    plan = UniformTimedPlan.from_model_json(model_bytes.decode("utf-8"))

    # Only plan_id and formation_id come from the frozen artifact.  Geometry,
    # endpoints, headings, limits and reverse permission are independently set.
    model = FormationModel((
        VehicleOffset(1, 1.0, 0.0),
        VehicleOffset(2, 0.0, 1.0),
        VehicleOffset(3, 0.0, -1.0),
    ))
    shapes = {vehicle_id: VehicleShape(0.738, 0.500)
              for vehicle_id in (1, 2, 3)}
    start = FormationPose(6.0, 6.0, 0.0)
    goal = FormationPose(10.0, 10.0, math.pi / 2.0)
    start_headings = {1: 0.0, 2: 0.0, 3: 0.0}
    terminal_headings = {
        1: math.pi / 2.0 + math.atan2(1.0, 2.0),
        2: math.pi / 2.0,
        3: math.pi / 2.0,
    }
    limits = FormationTimingLimits()
    request = UniformAuditRequest(
        plan_id=identity["plan_id"],
        formation_id=identity["formation_id"],
        model=model,
        vehicle_shapes=shapes,
        start_qf=start,
        start_headings=start_headings,
        goal_qf=goal,
        goal_headings=terminal_headings,
        timing_limits=limits,
        allow_reverse=True,
    )
    request_record = {
        "plan_id": identity["plan_id"],
        "plan_id_source": "frozen model identity only",
        "formation_id": identity["formation_id"],
        "formation_id_source": "frozen model identity only",
        "offsets_m": {"1": [1.0, 0.0], "2": [0.0, 1.0], "3": [0.0, -1.0]},
        "body_shapes_m": {str(key): [0.738, 0.500] for key in (1, 2, 3)},
        "start_qf": [6.0, 6.0, 0.0],
        "start_headings_rad": {str(key): value for key, value in start_headings.items()},
        "goal_qf": [10.0, 10.0, math.pi / 2.0],
        "terminal_headings_rad": {
            str(key): value for key, value in terminal_headings.items()},
        "timing_limits": asdict(limits),
        "allow_reverse": True,
        "construction": "independent G0 constants; no endpoint or limits from plan",
    }
    image = np.asarray(Image.open(map_path))
    occupied = np.flipud(image < 127).copy()
    grid = Grid(occupied, 0.05, 0.0, 0.0)
    sample_start = time.monotonic()
    projection = plan.sample()
    sample_wall_s = time.monotonic() - sample_start
    audit_start = time.monotonic()
    evaluation = audit_uniform_timed_plan(request, plan, projection, grid)
    audit_wall_s = time.monotonic() - audit_start
    model_sha256 = plan.model_sha256()
    model_json_sha256 = hashlib.sha256(model_bytes).hexdigest()
    duration_error = abs(plan.duration_s - 29.9552947533)
    identity_matches = (
        plan.plan_id == identity["plan_id"]
        and plan.formation_id == identity["formation_id"]
        and plan.plan_id == frozen_summary["plan_id"]
        and plan.formation_id == frozen_summary["formation_id"]
    )
    frozen_digest_matches = (
        model_sha256 == frozen_summary["model_sha256"]
        and model_json_sha256 == frozen_summary["model_json_sha256"]
    )
    passed = bool(
        evaluation.status == "FEASIBLE"
        and evaluation.feasible
        and duration_error <= 1e-9
        and identity_matches
        and frozen_digest_matches
    )
    return {
        "schema": "virtual_rigid_a.g0.reference_reaudit.v1",
        "generated_at_utc": _utc_now(),
        "scope": "historical FR06 mixed-mode reference only; not a new A-only planner result",
        "historical_context": {
            "tight_2_8m": "three historical autonomous mixed-mode successes",
            "wide_3_2m": "three historical autonomous searches exhausted budget",
            "evidence": str(acceptance_path.relative_to(repo_root)),
            "evidence_sha256": _sha256_file(acceptance_path),
            "this_run_performed_search": False,
        },
        "package_src": str(package_src),
        "reference_model": str(model_path.relative_to(repo_root)),
        "reference_model_file_sha256": model_json_sha256,
        "reference_summary": str(summary_path.relative_to(repo_root)),
        "reference_summary_file_sha256": _sha256_file(summary_path),
        "map": str(map_path.relative_to(repo_root)),
        "map_file_sha256": _sha256_file(map_path),
        "map_width_m": width_m,
        "map_image_shape": list(image.shape),
        "occupied_cell_count": int(occupied.sum()),
        "occupied_array_sha256": hashlib.sha256(occupied.tobytes()).hexdigest(),
        "grid": {
            "resolution_m": 0.05,
            "origin_x_m": 0.0,
            "origin_y_m": 0.0,
            "occupancy": "flipud(image < 127)",
        },
        "independent_request": request_record,
        "request_sha256": _canonical_sha256(request_record),
        "projection_source": "fresh plan.sample(); historical sample projection not read",
        "projection_sample_wall_s": sample_wall_s,
        "model_sha256": model_sha256,
        "frozen_model_digest_matches": frozen_digest_matches,
        "identity_matches": identity_matches,
        "duration_s": plan.duration_s,
        "expected_duration_s": 29.9552947533,
        "duration_absolute_error_s": duration_error,
        "audit_wall_s": audit_wall_s,
        "audit": evaluation.as_status(),
        "expected_status": "FEASIBLE" if abs(width_m - 2.8) <= 1e-12 else "RECORD_ONLY",
        "passed": passed,
        "new_a_only_capability_claimed": False,
    }


def _redirect_environment(output_dir: Path, case_id: str):
    case_root = output_dir / "regression_artifacts" / case_id
    paths = {
        "FR01_RESULTS_PATH": case_root / "FR01" / "primitive_results.json",
        "FR02_TIMING_RESULTS_PATH": case_root / "FR02" / "A" / "timing_results.json",
        "FR02_TRAJECTORY_RESULTS_PATH": case_root / "FR02" / "A" / "trajectory_results.json",
        "FR02_QUALITY_RESULTS_PATH": case_root / "FR02" / "B" / "uniform_quality_results.json",
        "FR02_ARTIFACT_DIR": case_root / "FR02" / "artifacts",
        "FR03_RESULTS_PATH": case_root / "FR03" / "successors.json",
        "FR03_ARTIFACT_DIR": case_root / "FR03",
        "FR04_RESULTS_PATH": case_root / "FR04" / "pipeline_results.json",
        "FR04_ARTIFACT_DIR": case_root / "FR04",
    }
    assert set(paths) == set(REDIRECT_VARIABLES)
    return {key: str(value) for key, value in paths.items()}


def _guard_regression_outputs(output_dir: Path, level: str):
    """Preflight every case before the first child process or output write."""
    conflicts = []
    for case_id in REGRESSION_LEVELS.get(level, ()):
        case_root = output_dir / "regression_artifacts" / case_id
        targets = (
            case_root,
            output_dir / "logs" / (case_id + ".stdout.log"),
            output_dir / "logs" / (case_id + ".stderr.log"),
        )
        for target in targets:
            resolved = target.resolve()
            try:
                resolved.relative_to(output_dir)
            except ValueError:
                raise BaselineError(
                    "REGRESSION_OUTPUT_ESCAPES_RUN: {}".format(target))
        if case_root.exists() or case_root.is_symlink():
            conflicts.append(str(case_root))
        for log_path in targets[1:]:
            if log_path.exists() or log_path.is_symlink():
                conflicts.append(str(log_path))
    if conflicts:
        raise BaselineError(
            "REGRESSION_OUTPUT_EXISTS: {}".format(sorted(conflicts)))


def run_regression(repo_root: Path, output_dir: Path, level: str) -> list[dict]:
    """Run explicit tests only after containment and no-overwrite preflight."""
    repo_root = repo_root.resolve()
    if level not in REGRESSION_STAGES:
        raise BaselineError("unknown regression level: {}".format(level))
    output_dir = _resolve_evidence_output(repo_root, output_dir)
    _guard_regression_outputs(output_dir, level)
    if level == "baseline":
        return []
    records = []
    for case_id in REGRESSION_LEVELS[level]:
        group, pattern = REGRESSION_CASES[case_id]
        env_delta = _redirect_environment(output_dir, case_id)
        env_delta.update({
            "PYTHONDONTWRITEBYTECODE": "1",
            "OPENBLAS_NUM_THREADS": "1",
        })
        environment = os.environ.copy()
        environment.update(env_delta)
        command = [
            sys.executable, "-B", "-m", "unittest", "discover",
            "-s", "tests", "-p", pattern, "-v",
        ]
        record = {
            "case_id": case_id,
            "group": group,
            "command": command,
            "command_shell_quoted": shlex.join(command),
            "cwd": str(repo_root),
            "interpreter": sys.executable,
            "environment_delta": env_delta,
            "started_at_utc": _utc_now(),
            "status": "NOT_RUN",
        }
        started = time.monotonic()
        try:
            completed = subprocess.run(
                command, cwd=str(repo_root), env=environment, text=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False)
        except PermissionError as exc:
            record.update({
                "status": "NOT_RUN_PERMISSION",
                "returncode": None,
                "stdout": "",
                "stderr": str(exc),
                "error_type": type(exc).__name__,
            })
        except (OSError, subprocess.SubprocessError) as exc:
            record.update({
                "status": "NOT_RUN",
                "returncode": None,
                "stdout": "",
                "stderr": str(exc),
                "error_type": type(exc).__name__,
            })
        else:
            record.update({
                "status": "PASS" if completed.returncode == 0 else "FAIL",
                "returncode": completed.returncode,
                "stdout": completed.stdout,
                "stderr": completed.stderr,
            })
        record["finished_at_utc"] = _utc_now()
        record["wall_s"] = time.monotonic() - started
        stdout_path = output_dir / "logs" / (case_id + ".stdout.log")
        stderr_path = output_dir / "logs" / (case_id + ".stderr.log")
        _write_text(stdout_path, record["stdout"])
        _write_text(stderr_path, record["stderr"])
        record["stdout_log"] = str(stdout_path.relative_to(output_dir))
        record["stderr_log"] = str(stderr_path.relative_to(output_dir))
        records.append(record)
    return records


def _verify_manifest_file(repo_root: Path, manifest_path: Path):
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != "virtual_rigid_a.g0.manifest.v1":
        raise BaselineError("unsupported manifest schema")
    recorded_root = Path(manifest["repo_root"]).resolve()
    if recorded_root != repo_root:
        raise BaselineError(
            "manifest repo_root mismatch: {} != {}".format(recorded_root, repo_root))
    stage = manifest.get("workspace_authorization_stage", "G0")
    current = collect_manifest(
        repo_root, tuple(manifest["protected_paths"]), stage=stage)
    mismatches = []
    if current["protected_tree_sha256"] != manifest["protected_tree_sha256"]:
        mismatches.append("protected_tree_sha256")
    recorded_files = {entry["path"]: entry for entry in manifest["protected_files"]}
    current_files = {entry["path"]: entry for entry in current["protected_files"]}
    for path in sorted(set(recorded_files) | set(current_files)):
        if recorded_files.get(path) != current_files.get(path):
            mismatches.append(path)
    if manifest["missing_protected_paths"] or current["missing_protected_paths"]:
        mismatches.append("missing_protected_paths")
    return {
        "schema": "virtual_rigid_a.g0.manifest_verification.v1",
        "manifest": str(manifest_path),
        "repo_root": str(repo_root),
        "status": "PASS" if not mismatches else "FAIL",
        "protected_file_count": len(current_files),
        "workspace_authorization_stage": stage,
        "mismatches": mismatches,
        "recorded_tree_sha256": manifest["protected_tree_sha256"],
        "current_tree_sha256": current["protected_tree_sha256"],
    }


def _workspace_gate(repo_root: Path, manifest, level="baseline"):
    if level not in REGRESSION_STAGES:
        raise BaselineError("unknown regression level: {}".format(level))
    stage = REGRESSION_STAGES[level]
    problems = []
    if manifest["git"]["branch"] != EXPECTED_BRANCH:
        problems.append("WORKSPACE_NOT_PREPARED: branch={}".format(manifest["git"]["branch"]))
    if not manifest["git"]["baseline_is_ancestor"]:
        problems.append("WORKSPACE_NOT_PREPARED: baseline is not an ancestor")
    unknown = _stage_unknown_modifications(manifest, stage)
    if unknown:
        problems.append("UNKNOWN_WORKSPACE_MODIFICATIONS: {}".format(unknown))
    if manifest["missing_protected_paths"]:
        problems.append("MISSING_PROTECTED_PATHS: {}".format(
            manifest["missing_protected_paths"]))
    if problems:
        raise BaselineError("; ".join(problems))


def _resolve_evidence_output(repo_root: Path, raw_output):
    """Shared CLI/public-runner containment rule; performs no writes."""
    output = Path(raw_output)
    if not output.is_absolute():
        output = repo_root / output
    if output.is_symlink():
        raise BaselineError("output directory must not be a symlink")
    output = output.resolve()
    evidence_root = (repo_root / "evaluation" / "virtual_formation_a").resolve()
    try:
        output.relative_to(evidence_root)
    except ValueError as exc:
        raise BaselineError(
            "output must be below {}".format(evidence_root)) from exc
    if output == evidence_root:
        raise BaselineError("output must name a unique run directory")
    if output.exists() and not output.is_dir():
        raise BaselineError("output path is not a directory: {}".format(output))
    return output


def _prepare_output(repo_root: Path, raw_output: str):
    output = _resolve_evidence_output(repo_root, raw_output)
    if output.exists() and any(output.iterdir()):
        raise BaselineError("OUTPUT_EXISTS_NONEMPTY: {}".format(output))
    output.mkdir(parents=True, exist_ok=True)
    (output / "logs").mkdir(exist_ok=True)
    return output


def _interface_snapshot(repo_root: Path):
    _add_package_to_sys_path(repo_root)
    from fwis_v3_optimizer.core import Grid
    from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
    from fwis_v3_optimizer.formation_collision import VehicleShape
    from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
    from fwis_v3_optimizer.uniform_quality import (
        UniformAuditRequest, audit_uniform_timed_plan)
    from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan
    objects = {
        "FormationModel": FormationModel,
        "FormationPose": FormationPose,
        "VehicleOffset": VehicleOffset,
        "VehicleShape": VehicleShape,
        "Grid": Grid,
        "FormationTimingLimits": FormationTimingLimits,
        "UniformAuditRequest": UniformAuditRequest,
        "UniformTimedPlan.from_model_json": UniformTimedPlan.from_model_json,
        "UniformTimedPlan.sample": UniformTimedPlan.sample,
        "audit_uniform_timed_plan": audit_uniform_timed_plan,
    }
    return {
        name: {
            "signature": str(inspect.signature(obj)),
            "module": getattr(obj, "__module__", ""),
            "source": inspect.getsourcefile(obj),
            "source_line": inspect.getsourcelines(obj)[1],
        } for name, obj in objects.items()
    }


def _regression_manifest():
    result = {}
    for level in ("baseline", "geometry", "timing", "full"):
        stage = REGRESSION_STAGES[level]
        authorization = {
            "stage": stage,
            "allowed_new_files": sorted(
                _cumulative_stage_values(STAGE_NEW_FILES, stage)),
            "allowed_new_prefixes": list(
                _cumulative_stage_values(STAGE_NEW_PREFIXES, stage)),
            "allowed_tracked_changes": sorted(
                _cumulative_stage_values(STAGE_TRACKED_FILES, stage)),
            "path_authorization_only": (
                "each actual diff still requires review for minimality and scope"),
        }
        if level == "baseline":
            result[level] = {
                "status": "NOT_RUN_OLD_TESTS_IN_G0",
                "cases": [],
                "reference_reaudit": True,
                "workspace_authorization": authorization,
            }
        else:
            result[level] = {
                "cases": [
                    {"case_id": case_id, "group": REGRESSION_CASES[case_id][0],
                     "pattern": REGRESSION_CASES[case_id][1]}
                    for case_id in REGRESSION_LEVELS[level]
                ],
                "redirect_variables": list(REDIRECT_VARIABLES),
                "per_case_artifact_root": "regression_artifacts/<case_id>/",
                "workspace_authorization": authorization,
            }
    result["full"]["not_included_in_g0_runner"] = [
        "fresh FR06 autonomous search (requires separately authorized G5 evidence)",
        "ROS/Gazebo (requires separately authorized G5 evidence)",
    ]
    return result


def _commands_markdown(repo_root, output, level, argv, regressions, exit_code):
    lines = [
        "# G0 Commands", "",
        "## Main invocation", "",
        "- cwd: `{}`".format(repo_root),
        "- interpreter: `{}`".format(sys.executable),
        "- Python: `{}`".format(sys.version.replace("\n", " ")),
        "- environment increments: `PYTHONDONTWRITEBYTECODE=1`, `OPENBLAS_NUM_THREADS=1`",
        "- regression level: `{}`".format(level),
        "- output: `{}`".format(output),
        "- exit code: `{}`".format(exit_code),
        "", "```bash",
        shlex.join([sys.executable, "-B", str(Path(__file__).resolve()), *argv]),
        "```", "",
        "The reference audit ran in this process. No historical verification or experiment script was executed.",
        "", "## Regression subprocesses", "",
    ]
    if not regressions:
        lines.append("None. G0 baseline does not run old unit tests, FR06 searches, ROS, or Gazebo.")
    for record in regressions:
        lines.extend([
            "### {}".format(record["case_id"]), "",
            "- cwd: `{}`".format(record["cwd"]),
            "- status / exit: `{}` / `{}`".format(record["status"], record["returncode"]),
            "- stdout: `{}`".format(record["stdout_log"]),
            "- stderr: `{}`".format(record["stderr_log"]),
            "- environment increment:", "",
            "```json", json.dumps(record["environment_delta"], indent=2, sort_keys=True), "```", "",
            "```bash", record["command_shell_quoted"], "```", "",
        ])
    return "\n".join(lines) + "\n"


def _report_markdown(reference, environment, before, after, regressions, level, overall):
    audit = reference["audit"]
    lines = [
        "# Claude G0 Report", "",
        "Stage: **G0 only**. Codex independent acceptance: **NOT_RUN / NOT_CLAIMED**.",
        "G1–G5 were not entered. No new A-only planning capability is claimed.", "",
        "## Implementation", "",
        "- Added `tools/virtual_a_baseline.py` for protected-file manifests, environment inspection, independent FR06 reference re-audit, and explicit safe regression mappings.",
        "- Added `docs/virtual_formation_a/G0_INTERFACE_CHECK.md` with actual API and output-path checks.",
        "- Added only this unique G0 evidence directory; no existing source, test, configuration, map, or historical evidence was edited.",
        "", "## Baseline and history protection", "",
        "- HEAD: `{}`".format(after["git"]["head"]),
        "- branch: `{}`".format(after["git"]["branch"]),
        "- protected files before / after: `{}` / `{}`".format(
            before["protected_file_count"], after["protected_file_count"]),
        "- protected tree before: `{}`".format(before["protected_tree_sha256"]),
        "- protected tree after: `{}`".format(after["protected_tree_sha256"]),
        "- protected files unchanged: **{}**".format(
            before["protected_tree_sha256"] == after["protected_tree_sha256"]),
        "", "## Independent FR06 reference re-audit", "",
        "The audit request uses independently written offsets, shapes, start/goal poses, terminal headings, default limits and `allow_reverse=True`. Only plan identity is read from the frozen model. This is a historical mixed-mode reference, not a new A-only result.", "",
        "- status / feasible: `{}` / `{}`".format(audit["status"], audit["feasible"]),
        "- reason: `{}`".format(audit["reason"]),
        "- checks: `{}` (informational, not a fixed acceptance count)".format(audit["checks"]),
        "- duration: `{:.14f}` s; absolute error from expected: `{:.3g}` s".format(
            reference["duration_s"], reference["duration_absolute_error_s"]),
        "- model SHA-256: `{}`".format(reference["model_sha256"]),
        "- audit wall time: `{:.6f}` s (informational)".format(reference["audit_wall_s"]),
        "- reference case passed: **{}**".format(reference["passed"]),
        "", "## Environment and blockers", "",
        "- interpreter: `{}`".format(environment["python"]["executable"]),
    ]
    for name, record in environment["dependencies"].items():
        lines.append("- {}: `{}` / `{}`".format(
            name, record["status"], record.get("version", record.get("error", ""))))
    lines.extend([
        "- CasADi/IPOPT scalar probe: `{}`; G2 dependency blocked: `{}`".format(
            environment["casadi_ipopt_probe"]["status"],
            environment["g2_nlp_dependency_blocked"]),
        "- ROS Noetic setup exists: `{}`; catkin devel setup exists: `{}`".format(
            environment["ros"]["noetic_setup_exists"],
            environment["ros"]["catkin_devel_setup_exists"]),
        "- ROS/Gazebo: `NOT_RUN_G0_EXCLUDED`; this is not G5 playback evidence.",
        "", "## Regression", "",
        "- requested level: `{}`".format(level),
        "- old test subprocesses executed: `{}`".format(len(regressions)),
    ])
    if not regressions:
        lines.append("- old unit tests: `NOT_RUN_G0_BASELINE`; no result is labeled PASS.")
    for record in regressions:
        lines.append("- {}: `{}`, exit `{}`".format(
            record["case_id"], record["status"], record["returncode"]))
    lines.extend([
        "", "## Result and stop", "",
        "Overall G0 implementation checks: **{}**. This is not Codex acceptance.".format(overall),
        "Stop here. Wait for Codex independent review and explicit user authorization before G1.",
        "",
    ])
    return "\n".join(lines)


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", help="unique new evidence run directory")
    parser.add_argument(
        "--regression", choices=("baseline", "geometry", "timing", "full"),
        default="baseline")
    parser.add_argument("--verify-manifest", help="verify a saved manifest without writing")
    args = parser.parse_args(argv)
    repo_root = Path(__file__).resolve().parents[1].resolve()
    if args.verify_manifest:
        if args.output:
            parser.error("--verify-manifest cannot be combined with --output")
        try:
            result = _verify_manifest_file(repo_root, Path(args.verify_manifest).resolve())
        except Exception as exc:
            print(json.dumps({
                "status": "FAIL", "error_type": type(exc).__name__, "error": str(exc)},
                indent=2), file=sys.stderr)
            return 2
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"] == "PASS" else 1
    if not args.output:
        parser.error("--output is required unless --verify-manifest is used")

    output = None
    before = None
    reference = None
    environment = None
    regressions = []
    try:
        workspace_stage = REGRESSION_STAGES[args.regression]
        before = collect_manifest(
            repo_root, DEFAULT_PROTECTED_PATHS, stage=workspace_stage)
        _workspace_gate(repo_root, before, level=args.regression)
        output = _prepare_output(repo_root, args.output)
        _write_json(output / "protected_before.json", before)
        environment = inspect_environment()
        _write_json(output / "environment.json", environment)
        _write_json(output / "regression_plan.json", _regression_manifest())
        _write_json(output / "api_interfaces.json", _interface_snapshot(repo_root))
        reference = reaudit_fr06_reference(repo_root)
        _write_json(output / "reference_reaudit.json", reference)
        regressions = run_regression(repo_root, output, args.regression)
        _write_json(output / "regression_results.json", {
            "schema": "virtual_rigid_a.g0.regression_results.v1",
            "level": args.regression,
            "status": "NOT_RUN_G0_BASELINE" if args.regression == "baseline" else (
                "PASS" if all(row["status"] == "PASS" for row in regressions) else "FAIL"),
            "records": regressions,
        })
        after = collect_manifest(
            repo_root, DEFAULT_PROTECTED_PATHS, stage=workspace_stage)
        _write_json(output / "protected_after.json", after)
        protected_unchanged = (
            before["protected_tree_sha256"] == after["protected_tree_sha256"])
        workspace_clean = (
            not _stage_unknown_modifications(after, workspace_stage)
            and after["git"]["branch"] == EXPECTED_BRANCH
            and after["git"]["baseline_is_ancestor"]
            and not after["missing_protected_paths"]
        )
        regressions_ok = all(row["status"] == "PASS" for row in regressions)
        overall = "PASS" if (
            reference["passed"] and protected_unchanged
            and workspace_clean and regressions_ok) else "FAIL"
        results = {
            "schema": SCHEMA,
            "stage": "G0",
            "generated_at_utc": _utc_now(),
            "overall_status": overall,
            "regression_level": args.regression,
            "workspace_authorization_stage": workspace_stage,
            "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
            "next_stage_entered": False,
            "cases": [
                {
                    "case": "workspace_and_baseline",
                    "expected": "feature branch, baseline ancestor, and no changes outside the declared workspace authorization stage",
                    "actual": "PASS" if workspace_clean else "FAIL",
                    "head": after["git"]["head"],
                    "branch": after["git"]["branch"],
                    "unknown_modifications": _stage_unknown_modifications(
                        after, workspace_stage),
                    "authorized_stage_tracked_changes": after.get(
                        "authorized_stage_tracked_changes", []),
                },
                {
                    "case": "fr06_reference_reaudit_2_8m",
                    "expected": "FEASIBLE, matching identity/digests, duration within 1e-9 s",
                    "actual": "PASS" if reference["passed"] else "FAIL",
                    "status": reference["audit"]["status"],
                    "reason": reference["audit"]["reason"],
                    "checks": reference["audit"]["checks"],
                    "duration_s": reference["duration_s"],
                    "duration_absolute_error_s": reference["duration_absolute_error_s"],
                },
                {
                    "case": "protected_history_unchanged",
                    "expected": "identical before/after protected tree",
                    "actual": "PASS" if protected_unchanged else "FAIL",
                    "before_sha256": before["protected_tree_sha256"],
                    "after_sha256": after["protected_tree_sha256"],
                },
                {
                    "case": "old_regression_tests",
                    "expected": "G0 baseline does not execute old tests",
                    "actual": "NOT_RUN_G0_BASELINE" if args.regression == "baseline" else (
                        "PASS" if regressions_ok else "FAIL"),
                    "subprocess_count": len(regressions),
                },
            ],
        }
        _write_json(output / "results.json", results)
        exit_code = 0 if overall == "PASS" else 1
        _write_text(output / "COMMANDS.md", _commands_markdown(
            repo_root, output, args.regression, argv, regressions, exit_code))
        _write_text(output / "CLAUDE_REPORT.md", _report_markdown(
            reference, environment, before, after, regressions,
            args.regression, overall))
        summary = {
            "stage": "G0", "overall_status": overall,
            "reference_status": reference["audit"]["status"],
            "reference_checks": reference["audit"]["checks"],
            "reference_duration_s": reference["duration_s"],
            "reference_audit_wall_s": reference["audit_wall_s"],
            "protected_unchanged": protected_unchanged,
            "regression_subprocesses": len(regressions),
            "output": str(output),
            "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
            "next_stage_entered": False,
        }
        _write_text(output / "logs" / "g0_summary.log", json.dumps(
            summary, indent=2, sort_keys=True) + "\n")
        new_files = {}
        for relative in (TOOL_PATH, INTERFACE_PATH):
            path = repo_root / relative
            if path.is_file():
                new_files[relative] = _sha256_file(path)
        for path in sorted(output.rglob("*")):
            if path.is_file() and path.name != "source_manifest.json":
                new_files[str(path.relative_to(repo_root))] = _sha256_file(path)
        source_manifest = {
            "schema": "virtual_rigid_a.g0.source_manifest.v1",
            "generated_at_utc": _utc_now(),
            "git": after["git"],
            "untracked": after["untracked"],
            "protected_before_sha256": before["protected_tree_sha256"],
            "protected_after_sha256": after["protected_tree_sha256"],
            "protected_files_unchanged": protected_unchanged,
            "protected_manifest": "protected_after.json",
            "new_file_sha256": new_files,
            "scope_note": "HEAD remains baseline; uncommitted additions are bound by the file hashes above.",
        }
        _write_json(output / "source_manifest.json", source_manifest)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return exit_code
    except Exception as exc:
        if output is not None and output.exists():
            _write_text(output / "logs" / "g0_error.log", traceback.format_exc())
            _write_json(output / "results.json", {
                "schema": SCHEMA,
                "stage": "G0",
                "overall_status": "FAIL",
                "error_type": type(exc).__name__,
                "error": str(exc),
                "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
                "next_stage_entered": False,
            })
        print("G0 FAILED: {}: {}".format(type(exc).__name__, exc), file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
