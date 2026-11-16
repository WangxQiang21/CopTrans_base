# G2 Rework 02 report — auto solver timeout ordering and fallback budget boundary

- Stage: **G2 only**; this is the second rework of the rejected G2 delivery,
  not G3.
- Governing task: `docs/virtual_formation_a/CLAUDE_G2_TASK.md`.
- Rework instruction: `docs/virtual_formation_a/CLAUDE_G2_REWORK_02.md`
  (SHA-256 `b410e9b25ea821c86f52b9e32943cb4b832b78590ea9c8e58318182b4315c4ea`).
- Evidence run: `evaluation/virtual_formation_a/G2/20261115T172337Z`.
- Baseline / HEAD: `567bdecba9676313da3bc9a97b82f57b5b569a2b`.
- Branch: `feature/virtual-rigid-formation-a`.
- Run status: **PASS_PENDING_CODEX_RECHECK**.
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**.
- Next stage: **not entered**. No G3, no FR06 modification, no physical-limit
  relaxation, no negative-test deletion, no ROS/Gazebo/hardware/map/timing
  claim, and no Git commit, branch, push or worktree operation.

This remains a **geometry-feasibility** result (`GEOMETRY_FEASIBLE` only). It
does not certify execution, dynamics, timing, collision freedom, map validity,
global convergence or hardware behaviour.

## 1. Defect addressed

The rejected G2 build handled `solver='auto'` by checking
`not result["success"]` before checking `result["soft_deadline_triggered"]`.
An IPOPT `Maximum_WallTime_Exceeded` result could therefore start SLSQP; a
successful SLSQP result could overwrite the timeout, lose the original
solver status and return `GEOMETRY_FEASIBLE` with
`deadline_exhausted=False`. The same auto branch could also start SLSQP after
an ordinary IPOPT exception even when the whole-request soft budget was
already exhausted.

Rework 02 changes only the deadline/fallback control flow and its tests and
evidence. The mathematical model, vehicle/curvature/steer limits, initial
guesses, G1 expectations, frozen requests and old FR06 were not changed.

## 2. Control-flow change

`formation_continuous_a/optimizer.py` now:

1. Handles a solver-reported soft timeout **immediately after the primary
   solver call and before any fallback**. It records the primary solver,
   `SOLVER_TIMEOUT`, `SOFT_BUDGET_EXHAUSTED`, the true
   `deadline_trigger_stage` and solver status, and breaks. It never starts
   SLSQP and never passes the timed-out iterate to `audit_geometry`. A
   previously audited best candidate may still be returned and is marked
   `audited_candidate_preserved=True`.
2. Records an ordinary primary failure as `SOLVER_FAILED` **before** invoking
   a fallback, so a later fallback success cannot erase the original failure
   from the attempt history.
3. Rechecks the entire request's remaining soft budget immediately before
   starting SLSQP. If no budget remains, it marks
   `trigger_stage="pre_fallback_boundary"`, preserves the real primary solver
   status (for example `IPOPT_EXCEPTION`), and stops without starting a new
   solver.
4. Applies the same timeout-first handling to a fallback SLSQP result. A
   fallback timeout is recorded as `SOLVER_TIMEOUT`, its returned iterate is
   not audited, and `timeout_solver_status` / `timeout_iterations` name the
   SLSQP trigger.
5. Preserves both sides of a legitimate fallback: the IPOPT failure attempt
   and the audited SLSQP attempt remain in the attempt log, and the fallback
   detail explicitly contains `IPOPT <status>; SLSQP fallback used: ...`.
   The fallback result also carries `fallback_from_solver`,
   `fallback_from_status` and `fallback_from_detail` internally for evidence
   and diagnosis.

The 30 s budget remains **soft**. IPOPT receives `max_wall_time`; SLSQP uses
an iteration callback. No immediate interruption or hard real-time guarantee
is claimed; library shutdown/return overhead is recorded, not pre-empted.

## 3. Required tests added (optimizer suite 17 -> 22)

All five new tests use the auto branch and production `solve_open_path`
control flow:

| Test | What it proves |
|---|---|
| `test_auto_timeout_never_starts_slsqp_or_audits` | mock IPOPT timeout; any SLSQP call raises; zero SLSQP calls, zero audit calls, `BUDGET_EXHAUSTED`, `SOLVER_TIMEOUT`, `Maximum_WallTime_Exceeded`, trigger `ipopt_max_wall_time` |
| `test_auto_timeout_after_audited_candidate_never_falls_back` | a genuine audited candidate first, then IPOPT timeout; candidate and audit preserved, no SLSQP call, deadline true, attempts `SOLVER_SUCCESS_AUDITED` then `SOLVER_TIMEOUT` |
| `test_auto_exception_with_exhausted_budget_never_starts_slsqp` | deterministic controllable clock; IPOPT exception is recorded with its real reason, `pre_fallback_boundary` exhaustion is recorded, SLSQP is not started, `BUDGET_EXHAUSTED` |
| `test_auto_ordinary_failure_with_budget_uses_real_slsqp_and_audit` | ordinary IPOPT `Solve_Failed`, genuine SLSQP fallback and genuine independent audit; both attempts are retained and the original failure detail survives |
| `test_auto_slsqp_timeout_is_recorded_without_audit` | genuine SLSQP callback timeout fed through the auto fallback; zero audit calls, `SOLVER_TIMEOUT`, correct `slsqp_iteration_callback` source |

The existing genuine-solver tests remain unchanged and are not replaced by
mocks:

- `test_ipopt_max_wall_time_is_effective_and_truthful` executes real IPOPT
  with a `1e-9 s` wall limit and observes `Maximum_WallTime_Exceeded`;
- `test_slsqp_iteration_callback_stops_and_marks_timeout` executes real SLSQP
  and observes `SLSQP_SOFT_DEADLINE` with at least one callback iteration;
- `test_timed_out_success_flag_never_reaches_the_audit` proves a malicious
  `success=True` timeout is still rejected by the production wrapper;
- `test_timeout_preserves_only_a_previously_audited_candidate` proves a real
  audited candidate survives a later timeout without a path/audit change.

## 4. Deterministic auto-branch evidence

`analysis/auto_deadline_probe.json` (script
`analysis/run_auto_deadline_probe.py`, log `logs/auto_deadline_probe.log`)
records the actual result, status, attempt list and deadline summary for all
five branches. All internal checks pass:

| Case | Result status | Trigger stage | Attempts (solver, status, record) |
|---|---|---|---|
| `auto_ipopt_timeout` | `BUDGET_EXHAUSTED` | `ipopt_max_wall_time` | `ipopt, Maximum_WallTime_Exceeded, SOLVER_TIMEOUT` |
| `auto_timeout_after_audited_candidate` | `GEOMETRY_FEASIBLE` | `ipopt_max_wall_time` | `ipopt, Solve_Succeeded, SOLVER_SUCCESS_AUDITED`; `ipopt, Maximum_WallTime_Exceeded, SOLVER_TIMEOUT` |
| `auto_exception_budget_exhausted` | `BUDGET_EXHAUSTED` | `pre_fallback_boundary` | `ipopt, IPOPT_EXCEPTION, SOLVER_FAILED` |
| `auto_ordinary_failure_real_fallback` | `GEOMETRY_FEASIBLE` | `""` (no deadline) | `ipopt, Solve_Failed, SOLVER_FAILED`; `scipy_slsqp, Optimization terminated successfully, SOLVER_SUCCESS_AUDITED` |
| `auto_slsqp_timeout` | `BUDGET_EXHAUSTED` | `slsqp_iteration_callback` | `ipopt, Solve_Failed, SOLVER_FAILED`; `scipy_slsqp, SLSQP_SOFT_DEADLINE, SOLVER_TIMEOUT` |

The probe uses real IPOPT for the first audited candidate, real SLSQP and the
production audit for the ordinary-failure fallback, and a real SLSQP callback
timeout for the final case. Timeout boundary results are mocked only where
the test instruction requires deterministic control of the solver status.
Every timed-out attempt in the probe has `audit_present=false`; every
fallback-success path has a passing independent audit.

## 5. Regression, frozen solve and benchmark

All four required suites were rerun from the worktree:

| Suite | Result | Log |
|---|---:|---|
| `test_virtual_a_audit.py` | 31/31 pass | `logs/audit.log` |
| `test_virtual_a_optimizer.py` | 22/22 pass | `logs/optimizer.log` |
| `test_virtual_a_geometry.py` (G1) | 20/20 pass | `logs/g1_geometry.log` |
| `test_virtual_a_profile.py` (G1) | 22/22 pass | `logs/g1_profile.log` |

The full G2 benchmark ran all eight frozen requests with only the request and
frozen config: **124 checks, `failed=[]`, status `PASS`**. All 48 frozen
attempts use the default IPOPT solver and all are independently audited
successes. The frozen six endpoints, straight-line request and
`STRAIGHT_POLICY` configuration and thresholds are unchanged.

| Request | Endpoint pos [m] | Heading [rad] | Rigidity [m] | G1 budget [m] | Audit budget [m] | Combined [m] | max abs C | max wheel [rad] | IPOPT iterations | Solve wall [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|
| E1 | 4.800e-12 | 5.551e-17 | 6.661e-16 | 1.780e-08 | 1.780e-08 | 3.560e-08 | 0.1129 | 0.02848 | 7,7,7,6,7,7 | 3.1649 |
| E2 | 4.802e-12 | 5.551e-17 | 6.661e-16 | 1.778e-08 | 1.780e-08 | 3.558e-08 | 0.1129 | 0.02848 | 7,7,6,7,7,7 | 3.0948 |
| E3 | 1.951e-11 | 5.551e-17 | 8.882e-16 | 1.787e-08 | 1.789e-08 | 3.576e-08 | 0.2425 | 0.06258 | 7,6,7,7,7,7 | 3.0732 |
| E4 | 1.951e-11 | 5.551e-17 | 8.882e-16 | 1.781e-08 | 1.789e-08 | 3.569e-08 | 0.2425 | 0.06258 | 6,7,7,7,7,7 | 3.0908 |
| E5 | 3.197e-11 | 0.000e+00 | 1.110e-15 | 2.386e-08 | 2.387e-08 | 4.773e-08 | 0.4285 | 0.11429 | 7,6,7,7,7,7 | 3.2789 |
| E6 | 3.197e-11 | 0.000e+00 | 6.661e-16 | 2.372e-08 | 2.387e-08 | 4.759e-08 | 0.4285 | 0.11429 | 6,7,7,7,7,7 | 3.1636 |
| STRAIGHT_LINE | 8.882e-16 | 3.232e-20 | 4.441e-16 | 1.776e-08 | 1.776e-08 | 3.552e-08 | 0.0000 | 0.00000 | 18,18,9,9,9,9 | 3.5124 |
| STRAIGHT_POLICY | 8.316e-10 | 5.551e-17 | 6.661e-16 | 1.991e-08 | 2.291e-08 | 4.282e-08 | 0.5131 | 0.13890 | 6,8,7,10,8,9 | 3.1312 |

Acceptance limits remain endpoint position `0.005 m`, endpoint heading
`0.2 deg = 3.490659e-3 rad`, rigidity `1e-6 m`, G1 position budget `1e-7 m`,
angle budget `1e-9 rad`, and the frozen physical C/wheel limits. The maximum
observed solve wall was `3.512377365 s`; no frozen request triggered the soft
deadline. The independent Simpson cross-check remains passing:
`1.514297925e-08 m` total audit budget, `4.008989595e-11 m` independent-audit
minus SciPy quadrature, and `8.950904183e-16 m` returned-path minus independent
integration.

Negative coverage now has 45 named entries: the 40 prior entries plus the five
new auto-branch tests. No negative test was deleted or weakened.

## 6. Protection, old evidence and files

`analysis/protection_diff.py` reports exactly:

- 6 authorized additions: G2 `audit.py`, `optimizer.py`, both G2 test modules,
  both frozen request/reference fixtures;
- 2 authorized changes: `contracts.py` and `tools/virtual_a_benchmark.py`;
- 0 removals;
- G1 protected tree
  `910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519`
  unchanged;
- current full protected tree
  `2cf68d1b32d51887ffa0d1aefb7a4ddc50d15a7b0a32f15f3ae8ab309845c874`;
- `only_authorized=true`.

The two prior evidence runs were checked against their own manifests and are
unchanged:

| Prior run | Files checked | Mismatches | Manifest SHA-256 |
|---|---:|---:|---|
| `G2/20261115T161608Z` (original) | 30 | 0 | `9ee32fb676215e4be8016dce64520562890e0821f21fdd88ade8ec29bfdf56be` |
| `G2/20261115T165745Z` (rework 01) | 34 | 0 | `5e6131f9196557a432ad2cba91c081e1f776e59cac3591463482d5b0faa3b6ad` |

Source and test hashes of this delivery:

| Path | SHA-256 |
|---|---|
| `formation_continuous_a/optimizer.py` | `188c814ced25828d95e3c170220e5853818f72a6de86b7053eee831b69a9381d` |
| `formation_continuous_a/audit.py` | `2c7da5165498a4fd5273dea336bf51c4b652f4c1cd5b31615f78b34367a7a58d` |
| `formation_continuous_a/contracts.py` | `27b101f4a8143f549cab67794999a5fb7a9d9a66e253e3db0518bd0745a1ec69` |
| `tests/test_virtual_a_audit.py` | `979b5d69df56bb28751c6491971716e4d89b2e69d6476d0d18407c4f909c8503` |
| `tests/test_virtual_a_optimizer.py` | `eee7a0fa0240e5be400b54e4afb397fc680674789c2d9afb7340a3a1ef5be130` |
| `tools/virtual_a_benchmark.py` | `92368b1714fdee494fd31a2f57270148904ef87893b5b95cb5961647715a9cad` |
| `docs/virtual_formation_a/CLAUDE_G2_REWORK_02.md` | `b410e9b25ea821c86f52b9e32943cb4b832b78590ea9c8e58318182b4315c4ea` |
| `analysis/run_auto_deadline_probe.py` | `de995086ed9b49c3b800a88e60433431ccfc156cca83faf6a916bab715f6e4cd` |

`bounds.py` (`3c12cca5…`), `geometry.py` (`1254c334…`) and `__init__.py`
(`81d470f4…`), both G1 test modules, and `setup.py` remain unchanged. The
source manifest binds the full source and run-file set, the auto deadline
probe, logs, attempts, acceptance evidence and protected-tree comparison.

Independent manifest verification was attempted three times. The first
attempt (against intermediate manifest `a903a20d…`) exited `1`: its own exit
code had been appended to the manifest-bound `logs/exit_codes.log` after
generation, and it expected `None` instead of the production empty-string
trigger `""` for `auto_ordinary_failure_real_fallback`. The expectation was
corrected, manifest `1c6464a5…` was regenerated, and a verifier run without
post-generation log mutation returned `status=OK`, `failures=[]`, 37 run
files, 24 source files, 48 attempts, 45 negative coverage entries and 124
benchmark checks. After these reports and the final attempt-labelled
`exit_codes.log` were reconciled, the manifest was regenerated and the final
no-append verification again returned `status=OK`, `failures=[]` and exit
`0`. The final manifest hash is in the excluded generation log; the
verification log is also excluded self-referential evidence.

The probe was first invoked with `python3 -B -I`; isolated mode could not see
the user-site SciPy and exited `1` with
`ModuleNotFoundError: No module named 'scipy'`. Its tee target was overwritten
by the successful rerun, so the traceback is not retained; the exit code is
recorded in `logs/exit_codes.log`. The successful probe and cross-check were
then run with normal `python3 -B` and the worktree package path. This was an
evidence-run invocation issue, not a production result.

## 7. Non-claims and conclusion

- `GEOMETRY_FEASIBLE` is not execution, timing, dynamics, collision, map or
  ROS feasibility.
- The deadline is a soft budget; library shutdown latency is recorded rather
  than pre-empted. No hard real-time guarantee is made.
- Solver timeout boundary mocks prove control flow; they do not replace the
  retained genuine IPOPT and SLSQP tests.
- Passing tests, the auto deadline probe and 124 benchmark checks is **not**
  Codex acceptance.
- G3 was not started; no commit, branch, push or worktree/reset operation was
  performed.

**Conclusion:** the auto-branch timeout/fallback ordering and budget-boundary
defect is fixed, with explicit regression tests and a deterministic evidence
probe. Prior G2 evidence and accepted G1 source/protected trees are untouched.
The delivery stops here for independent Codex recheck.