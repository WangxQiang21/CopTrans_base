# FR02-B commands, environment and exit codes

All commands were run from the V0.2 worktree root:

```text
/home/wq/whut_thesis/expv3_multi_v02
```

Worktree branch / HEAD at execution time:

```text
branch: refactor/formation-v02
HEAD:   6227833011e2d92b711179bfdec979bb0b878b29
```

No Git commit was created. The working tree is intentionally dirty because
FR02-A/FR02-B sources and evidence are uncommitted, as required by the task.
The complete porcelain status is recorded in `evidence_manifest.json`.

## Python / platform

```text
Python 3.8.10
numpy  1.24.4
Linux-5.15.0-139-generic-x86_64-with-glibc2.29
```

## FR02-B acceptance suite (B01--B08)

```bash
FR02_QUALITY_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/uniform_quality_results.json \
FR02_ARTIFACT_DIR=$PWD/evaluation/formation_refactor_v02/FR02/B \
OPENBLAS_NUM_THREADS=1 \
python3 -m unittest discover -s tests -p 'test_uniform_quality.py' -v
```

Exit code: `0` (`Ran 8 tests ... OK`). This run writes:

- `uniform_quality_results.json` -- independent audit JSON for B01--B08,
  including source hashes, import paths, thresholds, all success metrics,
  machine-readable tamper before/after digests and diffs, dynamic candidate
  set/bounds, sweep policy/coverage/budget, and first-collision states.
- `collision_map_b06.json` -- B06 obstacle grids, changed cells, endpoint
  checks, first collision time/station/qf and the full analytic state.
- `models/*.json` -- complete authoritative plan model dicts used by the
  legal fixtures and B06's 6 m straight fixture.
- `tampers/*.json` -- complete before/after input payloads for every tamper
  (plan, projection or config), plus live edge payloads for the wheel-ID swap
  that the canonical model dict deliberately sorts away.

## Required regression suites

The same environment was used for all suites:

```bash
cd /home/wq/whut_thesis/expv3_multi_v02
export FR01_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/regression/uniform_motion_results.json
export FR02_TIMING_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/regression/uniform_timing_results.json
export FR02_TRAJECTORY_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/regression/uniform_trajectory_results.json
export FR02_ARTIFACT_DIR=$PWD/evaluation/formation_refactor_v02/FR02/B/regression/artifacts
export FR02_QUALITY_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/uniform_quality_results.json
export OPENBLAS_NUM_THREADS=1

for name in test_formation_collision test_formation_quality test_formation_hold \
            test_formation_geometry test_formation_kinematics \
            test_formation_retiming test_uniform_motion test_uniform_timing \
            test_uniform_trajectory test_uniform_quality; do
  python3 -m unittest discover -s tests -p "${name}.py" -v \
    > "evaluation/formation_refactor_v02/FR02/B/regression/logs/${name}.log" 2>&1
done
```

Machine-readable results are in `regression/exit_codes.tsv`; full logs are in
`regression/logs/`. All ten suites exited `0`:

| suite | exit | summary |
|---|---|---|
| test_formation_collision | 0 | Ran 17 tests, OK |
| test_formation_quality | 0 | Ran 22 tests, OK |
| test_formation_hold | 0 | Ran 14 tests, OK |
| test_formation_geometry | 0 | Ran 15 tests, OK |
| test_formation_kinematics | 0 | Ran 28 tests, OK |
| test_formation_retiming | 0 | Ran 14 tests, OK |
| test_uniform_motion | 0 | Ran 13 tests, OK |
| test_uniform_timing | 0 | Ran 5 tests, OK |
| test_uniform_trajectory | 0 | Ran 8 tests, OK |
| test_uniform_quality | 0 | Ran 8 tests, OK |

## Compile check

```bash
python3 -m py_compile \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_quality.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_collision.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_quality.py \
  tests/test_uniform_quality.py
```

Exit code: `0`.

## Diff evidence

`diff/B_changes_vs_pre_b.diff` is the unified diff against
`pre_b/sources/` and `pre_b/tests/` (accepted FR02-A baseline).
`diff/B_changes_vs_pre_b.stat` lists changed/new/byte-identical files.

```text
formation_collision.py         +259   -5
formation_quality.py           +52    -17
uniform_quality.py             +3248  -0     (new)
tests/test_uniform_quality.py  +1778  -0     (new)
```

The following are byte-identical to `pre_b` (verified with `cmp -s` and
SHA-256):

```text
formation_geometry.py, formation_kinematics.py, formation_retiming.py,
uniform_motion.py, uniform_timing.py, uniform_trajectory.py,
tests/test_formation_collision.py, tests/test_formation_quality.py,
tests/test_uniform_motion.py, tests/test_uniform_timing.py,
tests/test_uniform_trajectory.py
```

## Evidence hashes

`command_manifest.json` lists every executed command with its exact environment
and exit code (machine-readable). `evidence_manifest.json` records SHA-256 and
byte size for every artifact,
pre-B source/test hashes, current source hashes, the git status, and the exact
commands. `pre_b/baseline.json` is the accepted FR02-A baseline manifest.