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
- protected files before / after: `3183` / `3183`
- protected tree before: `de8e609c5cc348d996dbe3dd42a7eaede580b647fda190cf8af61adfa674b8a4`
- protected tree after: `de8e609c5cc348d996dbe3dd42a7eaede580b647fda190cf8af61adfa674b8a4`
- protected files unchanged: **True**

## Independent FR06 reference re-audit

The audit request uses independently written offsets, shapes, start/goal poses, terminal headings, default limits and `allow_reverse=True`. Only plan identity is read from the frozen model. This is a historical mixed-mode reference, not a new A-only result.

- status / feasible: `FEASIBLE` / `True`
- reason: `OK`
- checks: `13390` (informational, not a fixed acceptance count)
- duration: `29.95529475329162` s; absolute error from expected: `8.38e-12` s
- model SHA-256: `1da16fa1bdbe93e1bd3e55855b8fd900b6c4770f66a7ccbe8ae7eb368a5affde`
- audit wall time: `1.459991` s (informational)
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
- ROS Noetic setup exists: `True`; catkin devel setup exists: `True`
- ROS/Gazebo: `NOT_RUN_G0_EXCLUDED`; this is not G5 playback evidence.

## Regression

- requested level: `full`
- old test subprocesses executed: `10`
- geometry_continuous: `PASS`, exit `0`
- geometry_formation: `PASS`, exit `0`
- timing_continuous: `FAIL`, exit `1`
- timing_formation: `PASS`, exit `0`
- fr01_uniform_motion: `PASS`, exit `0`
- fr02_a_timing: `PASS`, exit `0`
- fr02_a_trajectory: `PASS`, exit `0`
- fr02_b_quality: `PASS`, exit `0`
- fr03_uniform_search: `PASS`, exit `0`
- fr04_uniform_pipeline: `PASS`, exit `0`

## Result and stop

Overall G0 implementation checks: **FAIL**. This is not Codex acceptance.
Stop here. Wait for Codex independent review and explicit user authorization before G1.
