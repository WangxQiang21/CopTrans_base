# G2 Rework 01 commands, exit codes and observed walls

- Run id: `20261115T165745Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / formal ancestor: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Interpreter: `/usr/bin/python3`, Python 3.8.10
- Scope: endpoint-driven continuous A geometry only; **GEOMETRY_FEASIBLE**
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- Git commit / push / branch: **NOT_RUN**
- ROS / Gazebo / hardware / map / timing / dynamics: **NOT_RUN**
- G3: **NOT_ENTERED**

All commands below were run from the worktree root. Raw command output is in
the named logs. No old evidence file was overwritten and no dependency was
installed.

## Environment preamble used for the rework run

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G2/20261115T165745Z"
set -o pipefail
```

`logs/run_context.log` records the exact environment, git state and start time
`2026-11-15T16:57:45Z`. `environment.json` records Python/CasADi/IPOPT and
package versions (`casadi 3.7.2`, `ipopt` plugin available).

## Mandatory test and benchmark sequence

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v \
  2>&1 | tee "$VA_RUN/logs/audit.log"
# audit_exit=0; Ran 31 tests in 0.293s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v \
  2>&1 | tee "$VA_RUN/logs/optimizer.log"
# optimizer_exit=0; Ran 17 tests in 48.186s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_geometry.log"
# g1_geometry_exit=0; Ran 20 tests in 1.070s; OK (G1 regression unchanged)

python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_profile.log"
# g1_profile_exit=0; Ran 22 tests in 1.273s; OK (G1 regression unchanged)

python3 -B tools/virtual_a_benchmark.py --stage G2 \
  --output "$VA_RUN/benchmark" 2>&1 | tee "$VA_RUN/logs/benchmark.log"
# benchmark_exit=0
# status=PASS; checks=124; failed=[]; codex_acceptance=NOT_RUN_NOT_CLAIMED
```

Exit codes are preserved in `logs/exit_codes.log`:

```text
audit_exit=0
optimizer_exit=0
g1_geometry_exit=0
g1_profile_exit=0
benchmark_exit=0
```

## Independent integration-budget cross-check

```bash
python3 -B -I "$VA_RUN/analysis/run_audit_budget_crosscheck.py" \
  2>&1 | tee "$VA_RUN/logs/audit_budget_crosscheck.log"
# FIRST attempt: exit 1, KeyError: 'max_abs_g'
#   while formatting a failed-audit record that had no C2 block.
#   The traceback was not retained because the successful rerun overwrote the
#   tee target; the exit code 1 remains in logs/exit_codes.log.
#
# Fix in the evidence-only script: use `report.c2 or {}` and
#   `.get("max_abs_g")` when recording failed audits.
#
# Rerun of the same command after the fix:
# exit 0; status=PASS; all 11 checks true
```

`logs/exit_codes.log` therefore contains, in order:

```text
audit_budget_crosscheck_exit=1
audit_budget_crosscheck_exit=0
```

The current `analysis/audit_budget_crosscheck.json` and its log are the
successful rerun. The first failure was a formatting bug in the new evidence
script, not a production-audit pass or a changed production result.

## Evidence assembly commands

These commands are read-only for protected source; they only add files below
this G2 evidence run.

```bash
python3 -B "$VA_RUN/analysis/protection_diff.py" \
  2>&1 | tee "$VA_RUN/logs/protection_diff.log"
# exit 0; 6 authorized additions, 2 authorized changes, 0 removals

python3 -B "$VA_RUN/analysis/freeze_requests.py" \
  2>&1 | tee "$VA_RUN/logs/freeze_requests.log"
# exit 0; byte-identical frozen fixtures; PlannerConfig(**fixture) == PlannerConfig()

python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  2>&1 | tee "$VA_RUN/logs/assemble_evidence.log"
# exit 0; 8 requests, 48 attempts, acceptance invariants verified

python3 -B "$VA_RUN/analysis/write_environment.py" \
  2>&1 | tee "$VA_RUN/logs/write_environment.log"
# exit 0; environment.json written
```

Old evidence preservation was checked independently against the old run's own
manifest:

```bash
python3 -B -I - <<'PY' 2>&1 | tee "$VA_RUN/logs/old_run_integrity.log"
import hashlib, json
from pathlib import Path
run = Path('evaluation/virtual_formation_a/G2/20261115T161608Z')
manifest = json.loads((run / 'source_manifest.json').read_text(encoding='utf-8'))
bad = []
for relative, record in manifest['run_files'].items():
    path = run / relative
    if not path.is_file():
        bad.append((relative, 'missing'))
        continue
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != record['sha256']:
        bad.append((relative, digest, record['sha256']))
print('old_run_files_checked={}'.format(len(manifest['run_files'])))
print('mismatches={}'.format(len(bad)))
print('old_manifest_sha256={}'.format(hashlib.sha256(
    (run / 'source_manifest.json').read_bytes()).hexdigest()))
print('status={}'.format('UNCHANGED' if not bad else 'CHANGED'))
PY
echo "old_run_integrity_exit=${PIPESTATUS[0]}" >> "$VA_RUN/logs/exit_codes.log"
# old_run_files_checked=30; mismatches=0
# old_manifest_sha256=9ee32fb676215e4be8016dce64520562890e0821f21fdd88ade8ec29bfdf56be
# status=UNCHANGED; old_run_integrity_exit=0
```

## Source-manifest generation and independent verification

These are the final commands; they run after `CLAUDE_REPORT.md`,
`COMMANDS.md` and all other run files are frozen. The generation and
verification logs are excluded from the manifest's own run-file set, as
documented in the generator.

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
# writes source_manifest.json; final generation exit 0

PYTHONDONTWRITEBYTECODE=1 python3 -B -I \
  "$VA_RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
# independently recomputes every source/evidence/protected/git hash
```

Verification-script iteration is recorded honestly: the first verifier run
against the first generated manifest exited `1` with `KeyError: 'location'`
when checking `negative.counterexample.default_config`; the serialized audit
report uses `first_failure`, not `location`. The verifier was corrected to
use `first_failure`, the manifest was regenerated, and the final verifier run
exited `0` with `status=OK`, `failures=[]`, 34 run files, 23 source files,
48 attempts, 124 benchmark checks, and the expected protected trees. The
first failure's traceback was overwritten by the final successful stdout, but
the exit code is preserved in `logs/exit_codes.log`.

The verifier uses `-I`, imports only the standard library, and recomputes the
source hashes, evidence hashes, attempt/test/benchmark aggregates,
negative-coverage count, protected-tree comparison and old-G1 manifest checks.

## Actual G2 solve walls

The table is recorded in `analysis/acceptance_summary.json` and
`benchmark/results.json`. Values are actual monotonic walls on this host; the
soft budget is 30 s, checked at initial/attempt boundaries. IPOPT receives
`max_wall_time`, but no deadline triggered and no hard real-time guarantee is
made.

| Request | Solver | Attempts | IPOPT iterations | Inner wall [s] | Outer wall [s] |
|---|---|---:|---|---:|---:|
| E1 | ipopt | 6 | 7, 7, 7, 6, 7, 7 | 3.4854 | 3.4855 |
| E2 | ipopt | 6 | 7, 7, 6, 7, 7, 7 | 3.2374 | 3.2375 |
| E3 | ipopt | 6 | 7, 6, 7, 7, 7, 7 | 3.1677 | 3.1678 |
| E4 | ipopt | 6 | 6, 7, 7, 7, 7, 7 | 3.1432 | 3.1433 |
| E5 | ipopt | 6 | 7, 6, 7, 7, 7, 7 | 3.1724 | 3.1725 |
| E6 | ipopt | 6 | 6, 7, 7, 7, 7, 7 | 3.1730 | 3.1731 |
| STRAIGHT_LINE | ipopt | 6 | 18, 18, 9, 9, 9, 9 | 3.5283 | 3.5284 |
| STRAIGHT_POLICY | ipopt | 6 | 6, 8, 7, 10, 8, 9 | 3.2453 | 3.2453 |

Deadline policy evidence is in `analysis/attempts.json`,
`benchmark/results.json` and the optimizer test log. The frozen run records
`timeout_point=NOT_TRIGGERED`; the tests trigger real IPOPT and SLSQP timeout
paths, not only mocks. A timed-out iterate is never audited or published.