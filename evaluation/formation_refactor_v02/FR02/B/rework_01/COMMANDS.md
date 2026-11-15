# FR02-B rework_01 commands, environment and exit codes

All commands were run from the V0.2 worktree root:

```text
/home/wq/whut_thesis/expv3_multi_v02
```

Worktree branch / HEAD at execution time:

```text
branch: refactor/formation-v02
HEAD:   6227833011e2d92b711179bfdec979bb0b878b29
```

No Git commit was created and no branch/stash/worktree operation was performed.
The working tree is intentionally dirty because the uncommitted FR02-A/B
sources and evidence are under review. `evidence_manifest.json` records the
complete porcelain status.

## Python / platform

```text
Python 3.8.10
numpy  1.24.4
Linux-5.15.0-139-generic-x86_64-with-glibc2.29
```

## Shared environment

```bash
cd /home/wq/whut_thesis/expv3_multi_v02
export FR01_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01/regression/uniform_motion_results.json
export FR02_TIMING_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01/regression/uniform_timing_results.json
export FR02_TRAJECTORY_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01/regression/uniform_trajectory_results.json
export FR02_QUALITY_RESULTS_PATH=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01/uniform_quality_results.json
export OPENBLAS_NUM_THREADS=1
```

The FR02-B suite was run twice with the same source/tests: once directly
(`logs/test_uniform_quality.log`, evidence written at the `rework_01` root)
and once as the final entry of the ten-suite loop
(`regression/logs/test_uniform_quality.log`), which regenerated the final
`uniform_quality_results.json`, `collision_map_b06.json` and `models/`.
Both runs exited `0` and used identical source hashes.

## FR02-B acceptance suite (B01--B09)

```bash
FR02_ARTIFACT_DIR=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01 \
python3 -m unittest discover -s tests -p 'test_uniform_quality.py' -v \
  > evaluation/formation_refactor_v02/FR02/B/rework_01/logs/test_uniform_quality.log 2>&1
```

Exit code: `0` (`Ran 9 tests ... OK`). This run writes:

- `uniform_quality_results.json` -- independent audit JSON for B01--B09,
  including source hashes, import paths, thresholds, per-constraint
  candidate/analytic/refined/adopted bounds with `h` and the derivative
  supremum, endpoint coverage, unconfirmed rejections, sweep
  policy/coverage/budget, and first-collision states.
- `collision_map_b06.json` -- B06 obstacle grids, changed cells, endpoint
  checks, first collision time/station/qf and the full analytic state,
  including the S-spin mid-sweep collision.
- `models/*.json` -- complete authoritative plan model dicts for the legal
  fixtures, B06's 6 m straight and S-spin fixtures, and B09's confirmed and
  unconfirmed plans.
- `tampers/*.json` -- complete before/after input payloads for every tamper.

## Ten targeted suites (final run)

```bash
for name in test_formation_collision test_formation_quality test_formation_hold \
            test_formation_geometry test_formation_kinematics \
            test_formation_retiming test_uniform_motion test_uniform_timing \
            test_uniform_trajectory test_uniform_quality; do
  if [ "$name" = test_uniform_quality ]; then
    export FR02_ARTIFACT_DIR=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01
  else
    export FR02_ARTIFACT_DIR=$PWD/evaluation/formation_refactor_v02/FR02/B/rework_01/regression/artifacts
  fi
  python3 -m unittest discover -s tests -p "${name}.py" -v \
    > "evaluation/formation_refactor_v02/FR02/B/rework_01/regression/logs/${name}.log" 2>&1
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
| test_uniform_quality | 0 | Ran 9 tests, OK |

Total: 145 tests.

## Codex probe rerun (copy, not in place)

```bash
mkdir -p evaluation/formation_refactor_v02/FR02/B/rework_01/codex_probe_rerun
cp evaluation/formation_refactor_v02/FR02/B/codex_acceptance_20261113T144933Z/independent_probes.py \
   evaluation/formation_refactor_v02/FR02/B/rework_01/codex_probe_rerun/independent_probes.py
python3 evaluation/formation_refactor_v02/FR02/B/rework_01/codex_probe_rerun/independent_probes.py \
  > evaluation/formation_refactor_v02/FR02/B/rework_01/codex_probe_rerun/stdout.json \
  2> evaluation/formation_refactor_v02/FR02/B/rework_01/codex_probe_rerun/stderr.txt
```

Exit code: `0`. The copied probe reports all three certificate fixtures as
`FEASIBLE` with `dynamic_continuous_certificate=true` and
`unexpected_unverified_feasible=false`; the S-spin map still rejects with
`FORMATION_SWEEP_COLLISION`; all four motion-phase projection checks remain
bit-identical. The original Codex directory was not modified.

## Compile check

```bash
python3 -m py_compile \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_quality.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_collision.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_quality.py \
  tests/test_uniform_quality.py
```

Exit code: `0` (`logs/py_compile.exit`).

## Diff and preservation evidence

`diff/rework_01_changes_vs_pre_rework.diff` is the unified diff against
`pre_rework/sources/` and `pre_rework/tests/` (snapshot taken before this
rework). `diff/rework_01_changes_vs_pre_rework.stat` lists changed and
byte-identical files.

```text
uniform_quality.py             +185   -17
tests/test_uniform_quality.py  +347   -12
formation_collision.py         identical to pre_rework
formation_quality.py           identical to pre_rework
```

The earlier FR02-B evidence in `evaluation/formation_refactor_v02/FR02/B/`
(including `codex_acceptance_20261113T144933Z/`) was not overwritten. All 206
artifacts listed in the old `evidence_manifest.json` still match their
recorded SHA-256.

## Evidence hashes

`command_manifest.json` lists every executed command with its exact environment
and exit code (machine-readable). `evidence_manifest.json` records SHA-256 and
byte size for every rework artifact, pre-rework source/test hashes, current
source hashes, the git status, and the exact commands.