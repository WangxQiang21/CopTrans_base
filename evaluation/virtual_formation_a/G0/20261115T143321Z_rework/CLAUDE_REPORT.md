# Claude G0 Report

Stage: **G0 only**. Codex independent acceptance: **NOT_RUN / NOT_CLAIMED**.
G1–G5 were not entered. No new A-only planning capability is claimed.

## Implementation

- Added `tools/virtual_a_baseline.py` for protected-file manifests, environment inspection, independent FR06 reference re-audit, and explicit safe regression mappings.
- Added `docs/virtual_formation_a/G0_INTERFACE_CHECK.md` with actual API and output-path checks.
- Added only this unique G0 evidence directory; no existing source, test, configuration, map, or historical evidence was edited.

## Baseline and history protection

- HEAD: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- branch: `feature/virtual-rigid-formation-a`
- protected files before / after: `3120` / `3120`
- protected tree before: `ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091`
- protected tree after: `ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091`
- protected files unchanged: **True**

## Independent FR06 reference re-audit

The audit request uses independently written offsets, shapes, start/goal poses, terminal headings, default limits and `allow_reverse=True`. Only plan identity is read from the frozen model. This is a historical mixed-mode reference, not a new A-only result.

- status / feasible: `FEASIBLE` / `True`
- reason: `OK`
- checks: `13390` (informational, not a fixed acceptance count)
- duration: `29.95529475329162` s; absolute error from expected: `8.38e-12` s
- model SHA-256: `1da16fa1bdbe93e1bd3e55855b8fd900b6c4770f66a7ccbe8ae7eb368a5affde`
- audit wall time: `1.451380` s (informational)
- reference case passed: **True**

## Environment and blockers

- interpreter: `/usr/bin/python3`
- numpy: `AVAILABLE` / `1.24.4`
- scipy: `AVAILABLE` / `1.10.1`
- casadi: `AVAILABLE` / `3.7.2`
- PyYAML: `AVAILABLE` / `5.3.1`
- Pillow: `AVAILABLE` / `10.4.0`
- rospy: `AVAILABLE` / `unknown`
- CasADi/IPOPT scalar probe: `PASS`; G2 dependency blocked: `False`
- ROS Noetic setup exists: `True`; catkin devel setup exists: `False`
- ROS/Gazebo: `NOT_RUN_G0_EXCLUDED`; this is not G5 playback evidence.

## Regression

- requested level: `baseline`
- old test subprocesses executed: `0`
- old unit tests: `NOT_RUN_G0_BASELINE`; no result is labeled PASS.

## Rework scope

This directory is the G0 rework delivery for Codex REWORK items R1 and R2. The previous G0 run `20261115T140936Z` and Codex review `codex_review_20261115T141437Z` were preserved (15 original-run files re-hashed, zero drift).

### R1 — stage-aware workspace authorization

`_classify_untracked`, `collect_manifest`, `_workspace_gate`, and `main` now take an explicit authorization stage derived from the regression level (`baseline→G0`, `geometry→G1`, `timing→G3`, `full→G5`). The gate recomputes authorization from raw `git status` lines and does not trust the manifest's prior verdict. Authorization is a finite explicit path list per stage task book; only `A/M` changes are accepted and deletions/renames are rejected.

15 in-memory cases: approved G1 `setup.py` edit and new G1 package accepted at `geometry`; the same rejected at `baseline`; G2 file accepted only from `timing`; vehicle YAML, old algorithm, old test, deletion, unknown file rejected even at `full`; G5 node accepted at `full` but rejected at `timing`; G4 fixture map prefix active only from `full`. Actual repository files were not touched.

### R2 — public runner output guard

`run_regression` now shares `_resolve_evidence_output` with the CLI and runs `_guard_regression_outputs` before the first child process: output must be below `evaluation/virtual_formation_a/`, must not be the evidence root, must not be a symlink, and no target case `regression_artifacts/<case>/` or `<case>.stdout/stderr.log` may already exist. All cases are preflighted before any case starts.

5 mocked cases: historical FR06, external `/tmp`, evidence root and a pre-existing case artifact are rejected with zero child starts and zero writes; a clean run directory that already contains `main`'s metadata is accepted with only mocked children.

### Non-goals

No G1 package, algorithm, old test, configuration, map, ROS/Gazebo or Git action. `full` still does not include FR05 playback/feedback tests or real ROS, so it cannot be presented as complete G5 acceptance.

## Result and stop

Overall G0 rework checks: **PASS**. This is not Codex acceptance.
Stop here. Wait for Codex independent re-review and explicit user authorization before G1.
