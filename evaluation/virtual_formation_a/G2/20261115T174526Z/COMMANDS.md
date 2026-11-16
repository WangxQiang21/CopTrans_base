# G2 Rework 03 commands, exit codes and observed walls

- Run id: `20261115T174526Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / baseline ancestor: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Interpreter: `/usr/bin/python3`, Python 3.8.10
- Scope: endpoint-driven continuous A geometry only; **GEOMETRY_FEASIBLE**
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- Git commit / push / branch creation: **NOT_RUN**
- ROS / Gazebo / hardware / map / timing / dynamics: **NOT_RUN**
- G3: **NOT_ENTERED**

All commands were run from the worktree root.  Raw output for every recorded
invocation is in the named logs; `logs/exit_codes.log` records the successful
test/benchmark/evidence exits in order.  No old evidence file was overwritten,
no dependency was installed, no protected source was changed by evidence
assembly, and no command created a Git commit, branch or G3 entry.

## Environment preamble

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G2/20261115T174526Z"
set -o pipefail
```

`logs/run_context.log` records the worktree, Python, HEAD, branch and initial
Git status.  `environment.json` records Python/CasADi/IPOPT and package
versions (`casadi 3.7.2`, `ipopt` plugin available, `scipy 1.10.1`).

## Mandatory test sequence

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v \
  2>&1 | tee "$VA_RUN/logs/audit.log"
# audit_exit=0; Ran 31 tests in 0.292s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v \
  2>&1 | tee "$VA_RUN/logs/optimizer.log"
# optimizer_exit=0; Ran 26 tests in 49.262s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_geometry.log"
# g1_geometry_exit=0; Ran 20 tests in 1.061s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_profile.log"
# g1_profile_exit=0; Ran 22 tests in 1.290s; OK
```

The G1 geometry/profile suites are unchanged and were rerun in this run rather
than only referenced from the prior evidence.

## Frozen benchmark

```bash
python3 -B tools/virtual_a_benchmark.py --stage G2 \
  --output "$VA_RUN/benchmark" 2>&1 | tee "$VA_RUN/logs/benchmark.log"
# benchmark_exit=0
# status=PASS; checks=124; failed=[]; codex_acceptance=NOT_RUN_NOT_CLAIMED
```

The benchmark contains the eight frozen requests, 48 audited IPOPT attempts,
the six frozen endpoints, the straight line and the frozen `STRAIGHT_POLICY`
request.  Maximum observed solve wall: `3.437642872 s`, below the 30 s soft
budget; no deadline triggered.  Every successful attempt records a finite,
nonnegative `constraint_residual`.

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

python3 -B "$VA_RUN/analysis/run_auto_deadline_probe.py" \
  2>&1 | tee "$VA_RUN/logs/auto_deadline_probe.log"
# auto_deadline_probe_exit=0; all five cases checks_pass=true
```

The cross-check and auto probe use normal `python3 -B`, not isolated mode,
because SciPy is in the user-site installation and isolated mode cannot import
it.  This is an environment/invocation fact, not a production-solver change.

The auto probe records residual evidence for every attempt:

| Case | Status | Attempts | Residuals |
|---|---|---|---|
| `auto_ipopt_timeout` | `BUDGET_EXHAUSTED` | `SOLVER_TIMEOUT` | null |
| `auto_timeout_after_audited_candidate` | `GEOMETRY_FEASIBLE` | `SOLVER_SUCCESS_AUDITED`, `SOLVER_TIMEOUT` | success `2.5757174171303632e-14`; timeout null |
| `auto_exception_budget_exhausted` | `BUDGET_EXHAUSTED` | `SOLVER_FAILED` | null |
| `auto_ordinary_failure_real_fallback` | `GEOMETRY_FEASIBLE` | `SOLVER_FAILED`, `SOLVER_SUCCESS_AUDITED` | primary null; real SLSQP fallback `1.3989396308033974e-09` |
| `auto_slsqp_timeout` | `BUDGET_EXHAUSTED` | `SOLVER_FAILED`, `SOLVER_TIMEOUT` | both null |

No timed-out iterate reached the geometry audit.

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
# 8 requests, 48 attempts, 48 finite non-null residuals, 0 null success residuals
# 124 benchmark checks; all acceptance invariants pass

python3 -B "$VA_RUN/analysis/write_environment.py" \
  2>&1 | tee "$VA_RUN/logs/write_environment.log"
# write_environment_exit=0; environment.json written
```

### Old evidence preservation

All three prior G2 runs were checked against their own manifests, without
modifying them:

```bash
python3 -B -I - <<'PY' 2>&1 | tee "$VA_RUN/logs/old_run_integrity.log"
import hashlib, json
from pathlib import Path
runs = [
    Path('evaluation/virtual_formation_a/G2/20261115T161608Z'),
    Path('evaluation/virtual_formation_a/G2/20261115T165745Z'),
    Path('evaluation/virtual_formation_a/G2/20261115T172337Z'),
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
    print('source_files_checked={}'.format(len(manifest['source_files'])))
    print('mismatches={}'.format(len(bad)))
    print('manifest_sha256={}'.format(hashlib.sha256(
        (run / 'source_manifest.json').read_bytes()).hexdigest()))
    print('status={}'.format('UNCHANGED' if not bad else 'CHANGED'))
PY
# old_run_integrity_exit=0
# original: 30 files, 0 mismatches, manifest 9ee32fb6...; UNCHANGED
# rework01: 34 files, 0 mismatches, manifest 5e6131f9...; UNCHANGED
# rework02: 37 files, 0 mismatches, manifest 76442fdd...; UNCHANGED
```

## Source-manifest generation and independent verification

These commands are run last, after `CLAUDE_REPORT.md`, this file and all other
run files are present.  `source_manifest.json`,
`logs/source_manifest_generation.log` and
`logs/source_manifest_verification.log` are excluded self-referential
artifacts.  The manifest binds `logs/exit_codes.log`, so the verifier is not
allowed to append its exit code to that file after generation.

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
# first-pass manifest generation; excluded log

PYTHONDONTWRITEBYTECODE=1 python3 -B -I \
  "$VA_RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
# preliminary independent verification of the first-pass manifest
```

After the preliminary verifier passes, its exit is appended to
`logs/exit_codes.log` as `source_manifest_verification_preliminary_exit=0`
and the manifest is regenerated so that `exit_codes.log` itself is bound:

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
# final manifest generation; excluded log

PYTHONDONTWRITEBYTECODE=1 python3 -B -I \
  "$VA_RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
# final independent verification; its full JSON output and exit marker are in
# the excluded log, not appended to any manifest-bound file
```

The verifier uses `-I`, imports only the standard library, and independently
recomputes source/run-file SHA-256 values, the frozen config/requests, the
attempt register and its residual coverage, the acceptance maxima, the 124
benchmark checks, the independent Simpson budgets, the protected trees, the
accepted G1 manifest and the Git state.  The final status and `failures` list
are in the excluded verification log.

## Observed exit codes

`logs/exit_codes.log` contains, in order:

```text
benchmark_exit=0
audit_exit=0
optimizer_exit=0
g1_geometry_exit=0
g1_profile_exit=0
freeze_requests_exit=0
audit_budget_crosscheck_exit=0
auto_deadline_probe_exit=0
protection_diff_exit=0
assemble_evidence_exit=0
write_environment_exit=0
old_run_integrity_exit=0
source_manifest_verification_preliminary_exit=0
```

The final manifest generation and final verification exits are recorded in
their excluded logs.  No command changed a protected source file, created a
branch or committed Git.