# G2 Rework 02 commands, exit codes and observed walls

- Run id: `20261115T172337Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / formal ancestor: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Interpreter: `/usr/bin/python3`, Python 3.8.10
- Scope: endpoint-driven continuous A geometry only; **GEOMETRY_FEASIBLE**
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- Git commit / push / branch: **NOT_RUN**
- ROS / Gazebo / hardware / map / timing / dynamics: **NOT_RUN**
- G3: **NOT_ENTERED**

All commands below were run from the worktree root. Raw output for the
successful/final invocations is in the named logs. Two failed local evidence
invocations had their tee targets overwritten by later successful reruns: the
isolated auto-probe import (`exit=1`, described below) and the first manifest
verification (`exit=1`, described below). Their diagnostics and exit codes are
retained here and in `logs/exit_codes.log`. No old evidence file was
overwritten, no dependency was installed and no protected source was changed
by evidence assembly.

## Environment preamble

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G2/20261115T172337Z"
set -o pipefail
```

`logs/run_context.log` records the worktree, Python, HEAD, branch and initial
Git status. `environment.json` records Python/CasADi/IPOPT and package
versions (`casadi 3.7.2`, `ipopt` plugin available, `scipy 1.10.1`).

## Mandatory test sequence

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v \
  2>&1 | tee "$VA_RUN/logs/audit.log"
# audit_exit=0; Ran 31 tests in 0.308s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v \
  2>&1 | tee "$VA_RUN/logs/optimizer.log"
# optimizer_exit=0; Ran 22 tests in 48.608s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_geometry.log"
# g1_geometry_exit=0; Ran 20 tests in 1.057s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_profile.log"
# g1_profile_exit=0; Ran 22 tests in 1.292s; OK
```

`logs/exit_codes.log` preserves:

```text
audit_exit=0
optimizer_exit=0
g1_geometry_exit=0
g1_profile_exit=0
```

## Frozen benchmark

```bash
python3 -B tools/virtual_a_benchmark.py --stage G2 \
  --output "$VA_RUN/benchmark" 2>&1 | tee "$VA_RUN/logs/benchmark.log"
# benchmark_exit=0
# status=PASS; checks=124; failed=[]; codex_acceptance=NOT_RUN_NOT_CLAIMED
```

The benchmark output contains all eight frozen requests, 48 audited IPOPT
attempts, the six frozen endpoints, the straight line and the frozen
`STRAIGHT_POLICY` request. The maximum observed solve wall in this run was
`3.512377365 s`, below the 30 s soft budget; no deadline triggered.

## Freeze and independent checks

```bash
python3 -B "$VA_RUN/analysis/freeze_requests.py" \
  2>&1 | tee "$VA_RUN/logs/freeze_requests.log"
# freeze_requests_exit=0
# planner_config_is_default=true; both fixture hashes frozen

python3 -B "$VA_RUN/analysis/run_audit_budget_crosscheck.py" \
  2>&1 | tee "$VA_RUN/logs/audit_budget_crosscheck.log"
# audit_budget_crosscheck_exit=0; all 11 checks true
# audit_total_budget_m=1.514297925e-08; fixed threshold=5e-08
# independent minus quad euclidean=4.008989595e-11 m
# returned minus independent euclidean=8.950904183e-16 m
```

The cross-check is run with normal `python3 -B` rather than `-I`: SciPy is in
the user-site installation and isolated mode cannot import it. This is an
environment/invocation fact, not a production-solver change.

### Auto deadline probe

```bash
python3 -B "$VA_RUN/analysis/run_auto_deadline_probe.py" \
  2>&1 | tee "$VA_RUN/logs/auto_deadline_probe.log"
# auto_deadline_probe_exit=0; all five cases checks_pass=true
```

An initial reproduction used `python3 -B -I` and exited `1` at import time
with `ModuleNotFoundError: No module named 'scipy'`; the isolated-mode
traceback was overwritten by the successful rerun. `logs/exit_codes.log`
preserves both the isolated-first-attempt exit `1` and the successful exit
`0`. The successful probe was run without `-I` and with the worktree package
path. Probe results are recorded in `analysis/auto_deadline_probe.json`:

| Case | Status | Attempts |
|---|---|---|
| `auto_ipopt_timeout` | `BUDGET_EXHAUSTED` | `ipopt/Maximum_WallTime_Exceeded/SOLVER_TIMEOUT` |
| `auto_timeout_after_audited_candidate` | `GEOMETRY_FEASIBLE` | `ipopt/Solve_Succeeded/SOLVER_SUCCESS_AUDITED`, then `ipopt/Maximum_WallTime_Exceeded/SOLVER_TIMEOUT` |
| `auto_exception_budget_exhausted` | `BUDGET_EXHAUSTED` | `ipopt/IPOPT_EXCEPTION/SOLVER_FAILED` |
| `auto_ordinary_failure_real_fallback` | `GEOMETRY_FEASIBLE` | `ipopt/Solve_Failed/SOLVER_FAILED`, then `scipy_slsqp/Optimization terminated successfully/SOLVER_SUCCESS_AUDITED` |
| `auto_slsqp_timeout` | `BUDGET_EXHAUSTED` | `ipopt/Solve_Failed/SOLVER_FAILED`, then `scipy_slsqp/SLSQP_SOFT_DEADLINE/SOLVER_TIMEOUT` |

Every probe case has `checks_pass=true`; no timed-out iterate reached the
audit.

## Evidence assembly

```bash
python3 -B "$VA_RUN/analysis/protection_diff.py" \
  2>&1 | tee "$VA_RUN/logs/protection_diff.log"
# protection_diff_exit=0
# 6 authorized additions, 2 authorized changes, 0 removals
# only_authorized=true

python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  2>&1 | tee "$VA_RUN/logs/assemble_evidence.log"
# assemble_evidence_exit=0
# 8 requests, 48 attempts, 124 benchmark checks; all acceptance invariants pass

python3 -B "$VA_RUN/analysis/write_environment.py" \
  2>&1 | tee "$VA_RUN/logs/write_environment.log"
# write_environment_exit=0; environment.json written
```

### Old evidence preservation

The two prior G2 runs were checked against their own manifests, without
modifying them:

```bash
python3 -B -I - <<'PY' 2>&1 | tee "$VA_RUN/logs/old_run_integrity.log"
import hashlib, json
from pathlib import Path
runs = [
    Path('evaluation/virtual_formation_a/G2/20261115T161608Z'),
    Path('evaluation/virtual_formation_a/G2/20261115T165745Z'),
]
for run in runs:
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
    print('run={}'.format(run.name))
    print('run_files_checked={}'.format(len(manifest['run_files'])))
    print('mismatches={}'.format(len(bad)))
    print('manifest_sha256={}'.format(hashlib.sha256(
        (run / 'source_manifest.json').read_bytes()).hexdigest()))
    print('status={}'.format('UNCHANGED' if not bad else 'CHANGED'))
PY
# old_run_integrity_exit=0
# original: 30 files, 0 mismatches, manifest 9ee32fb6...; UNCHANGED
# rework01: 34 files, 0 mismatches, manifest 5e6131f9...; UNCHANGED
```

## Source-manifest generation and independent verification

These commands were run last, after `CLAUDE_REPORT.md`, this file and all
other run files were present. The generator and verifier logs, and
`source_manifest.json`, are excluded self-referential artifacts.

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
# writes source_manifest.json

PYTHONDONTWRITEBYTECODE=1 python3 -B -I \
  "$VA_RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
# independently recomputes every source/run-file/protected/git hash
```

The first verification attempt, against an intermediate manifest
`a903a20d…`, exited `1` with three failures:

```text
run file hash changed: logs/exit_codes.log
run file size changed: logs/exit_codes.log
auto deadline probe trigger changed: auto_ordinary_failure_real_fallback
```

The first two were caused by appending the verifier's exit code to
`logs/exit_codes.log` after the manifest had been generated, which made a
manifest-bound run file stale. The third was a verifier expectation error:
the no-deadline ordinary-failure/fallback case correctly has the production
empty-string trigger `""`, not `None`. The expectation was corrected to `""`,
the intermediate manifest `1c6464a5…` was generated, and a verifier run that
did not append to `exit_codes.log` returned `status=OK`, `failures=[]`, 37 run
files, 24 source files, 48 attempts, 45 negative coverage entries and 124
benchmark checks.

The docs were then updated with this history and `exit_codes.log` was given
the final attempt labels. The manifest was regenerated once more and the
final verifier was run without appending to any manifest-bound file. That
run also returns `status=OK` with the same counts; its output and exit `0`
marker are in the excluded
`logs/source_manifest_verification.log`. The final manifest hash is recorded
in the excluded `logs/source_manifest_generation.log` (the manifest cannot
hash itself). The verifier uses `-I`, imports only the standard library, and
independently recomputes the source hashes, evidence hashes, frozen config,
attempts, protected trees, G1 manifest and git state.

## Observed exit codes

`logs/exit_codes.log` contains, in order:

```text
audit_exit=0
optimizer_exit=0
g1_geometry_exit=0
g1_profile_exit=0
benchmark_exit=0
freeze_requests_exit=0
audit_budget_crosscheck_exit=0
auto_deadline_probe_isolated_first_attempt_exit=1
auto_deadline_probe_exit=0
protection_diff_exit=0
assemble_evidence_exit=0
write_environment_exit=0
old_run_integrity_exit=0
source_manifest_generation_exit=0
source_manifest_verification_attempt1_exit=1
source_manifest_verification_attempt2_exit=0
```

The final no-append verification ran after that log was frozen and therefore
does not add another line; its own exit `0` is recorded at the end of the
excluded verifier log.

No command changed a protected source file, created a branch or committed
Git. The isolated probe failure is recorded honestly and was an
environment/import issue only.