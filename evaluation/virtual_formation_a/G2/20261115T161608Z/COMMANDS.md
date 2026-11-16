# G2 commands, exit codes and observed walls

- Run id: `20261115T161608Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / formal ancestor: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Interpreter: `/usr/bin/python3`, Python 3.8.10
- Scope: endpoint-driven continuous A geometry only; **GEOMETRY_FEASIBLE**
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- Git commit / push / branch: **NOT_RUN**
- ROS / Gazebo / hardware / map / timing / dynamics: **NOT_RUN**

All commands below were run from the worktree root. Raw command output is in
the named logs. No old evidence file was overwritten and no dependency was
installed.

## Environment preamble used for the frozen run

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G2/20261115T161608Z"
set -o pipefail
```

`logs/run_context.log` records the exact environment, git state and start time
`2026-11-15T16:16:08Z`. `environment.json` records Python/CasADi/IPOPT and
package versions (`casadi 3.7.2`, `ipopt` plugin available).

## Mandatory frozen command sequence

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v \
  2>&1 | tee "$VA_RUN/logs/audit.log"
# audit_exit=0; Ran 20 tests in 0.220s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v \
  2>&1 | tee "$VA_RUN/logs/optimizer.log"
# optimizer_exit=0; Ran 12 tests in 46.827s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_geometry.log"
# g1_geometry_exit=0; Ran 20 tests in 1.075s; OK (G1 regression unchanged)

python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v \
  2>&1 | tee "$VA_RUN/logs/g1_profile.log"
# g1_profile_exit=0; Ran 22 tests in 1.289s; OK (G1 regression unchanged)

python3 -B tools/virtual_a_benchmark.py --stage G2 \
  --output "$VA_RUN/benchmark" 2>&1 | tee "$VA_RUN/logs/benchmark.log"
# benchmark_exit=0
# status=PASS; checks=97; failed=[]; codex_acceptance=NOT_RUN_NOT_CLAIMED
```

Exit codes are preserved verbatim in `logs/exit_codes.log`:

```text
audit_exit=0
optimizer_exit=0
g1_geometry_exit=0
g1_profile_exit=0
benchmark_exit=0
```

The benchmark writes `benchmark/summary.json` (97 checks, acceptance summary)
and `benchmark/results.json` (per-request request, endpoint, full profile, full
final audit, all six attempts and their candidate audits, walls).

## Evidence assembly commands

These commands are read-only for protected source; they only add files below
this G2 evidence run.

```bash
python3 -B "$VA_RUN/analysis/protection_diff.py" \
  2>&1 | tee "$VA_RUN/logs/protection_diff.log"
# exit 0; exactly 6 protected additions, 2 authorized changes, 0 removals

python3 -B "$VA_RUN/analysis/freeze_requests.py" \
  2>&1 | tee "$VA_RUN/logs/freeze_requests.log"
# exit 0; byte-identical frozen fixtures; PlannerConfig(**fixture) == PlannerConfig()

python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  2>&1 | tee "$VA_RUN/logs/assemble_evidence.log"
# exit 0; 8 requests, 48 attempts, acceptance invariants verified

python3 -B "$VA_RUN/analysis/write_environment.py" \
  2>&1 | tee "$VA_RUN/logs/write_environment.log"
# exit 0; environment.json written

python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
# exit 0; source_manifest.json written

PYTHONDONTWRITEBYTECODE=1 python3 -B -I \
  "$VA_RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
# exit 0; all source/evidence/protected/git checks independently recomputed
```

`-I` in the verifier isolates it from `PYTHONPATH`; the verifier uses only the
standard library and recomputes every hash itself.

## Actual G2 solve walls

The table is recorded in `analysis/acceptance_summary.json` and
`benchmark/results.json`. Values are actual monotonic walls on this host; the
soft budget is 30 s, checked at initial/attempt boundaries. IPOPT iterations
run internally under `max_iter=120` and are not individually polled. No
deadline was exhausted and no library-call overrun occurred.

| Request | Solver | Attempts | IPOPT iterations | Inner wall [s] | Outer wall [s] |
|---|---|---:|---|---:|---:|
| E1 | ipopt | 6 | 7, 7, 7, 6, 7, 7 | 3.1526 | 3.1527 |
| E2 | ipopt | 6 | 7, 7, 6, 7, 7, 7 | 3.1015 | 3.1016 |
| E3 | ipopt | 6 | 7, 6, 7, 7, 7, 7 | 3.0959 | 3.0960 |
| E4 | ipopt | 6 | 6, 7, 7, 7, 7, 7 | 3.1306 | 3.1307 |
| E5 | ipopt | 6 | 7, 6, 7, 7, 7, 7 | 3.1253 | 3.1254 |
| E6 | ipopt | 6 | 6, 7, 7, 7, 7, 7 | 3.1255 | 3.1256 |
| STRAIGHT_LINE | ipopt | 6 | 18, 18, 9, 9, 9, 9 | 3.5308 | 3.5309 |
| STRAIGHT_POLICY | ipopt | 6 | 6, 8, 7, 10, 8, 9 | 3.2178 | 3.2179 |

Maximum measured solve wall: `3.5308071420295164 s`; timeout point:
`NOT_TRIGGERED`.

## Frozen evidence commands: what not to infer

- The quadratic profiles `k(s)=alpha*(s/S)^2` and the clamped CubicSpline are
  independent existence/audit oracles. They are frozen in
  `frozen/existence_reference.json` and are never passed to
  `solve_open_path(request, config)`.
- The production isolation test opens no fixture, reference, `tests/` or
  `evaluation/` file during a solve. The benchmark reads the reference only
  for its independent checks.
- `benchmark/results.json` contains the six automatic initial guesses and the
  six attempts per request. `analysis/attempts.json` repeats them as the
  auditable attempt register.
- `GEOMETRY_FEASIBLE` is not an executable, timed, dynamic, collision, ROS or
  hardware claim.