# G2 Rework 03 — successful-candidate constraint residual evidence

- Run id: `20261115T174526Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / baseline ancestor: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Status in `results.json`: `PASS_PENDING_CODEX_RECHECK`
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- G3: **NOT_ENTERED**
- Git commit / push / branch creation: **NOT_RUN**
- ROS / Gazebo / hardware / map / timing / dynamics: **NOT_RUN**

This run is the rework-03 response to the finding that rework-02 fixed the
`auto` deadline/fallback control flow but recorded
`constraint_residual = null` for every successful primary or fallback solver
attempt.  The production change is limited to residual recording for
successful candidates plus the tests and evidence needed to prove it.  The
independent whole-segment geometry audit, mathematical model, vehicle limits,
soft-deadline semantics, old FR06, previous G2 evidence and accepted G1
sources/tests are unchanged.

## 1. Production change

`optimizer.py` now computes the constraint residual from the actual successful
decision vector and stores it on the attempt record before candidate
construction and before the independent audit:

- The existing `_constraint_residual` diagnostic itself is unchanged.  Its
  value remains the maximum of the NLP equality-row violation and the signed
  curvature-bound violation, and it is **not** a substitute for
  `audit_geometry`.
- `_constraint_residual_diagnostic(z, request, config, packet)` validates the
  vector shape `2 * intervals + 3`, converts it to a finite float vector,
  calls `_constraint_residual`, and requires a finite, nonnegative result.
- `_attach_failure_residual` records a best-effort residual for a failed
  non-timeout solver result when a vector exists.  A missing or malformed
  vector leaves the residual `null` and appends an explicit reason to the
  attempt detail; the planner does not raise.
- The primary and fallback failure paths use `_attach_failure_residual`.
  Timeout branches are unchanged: a timed-out iterate is not residual-audited,
  not geometry-audited and never published as a candidate.
- On a successful solver result, the diagnostic is evaluated on
  `candidate_result["z"]` before `_candidate_path` and before `audit_geometry`.
  If the vector is missing/non-finite or the diagnostic raises or returns a
  non-finite/negative value, the attempt is recorded as
  `CONSTRAINT_RESIDUAL_FAILURE` with `constraint_residual = null`; the
  candidate is not constructed, the geometry audit is not called, and no
  unhandled exception escapes the planner.
- Direct IPOPT success and legitimate SLSQP fallback success both reach the
  same success path and therefore both receive a finite residual.

Allowed null residuals are limited to failure/timeout/no-vector/non-finite
cases, and each such attempt still carries an explicit solver status, attempt
status, rejection reason and/or detail.

## 2. Tests

| Suite | Tests | Result | Log | Log SHA-256 |
|---|---:|---|---|---|
| `test_virtual_a_audit.py` | 31 | OK | `logs/audit.log` | `7f8f0e892524f5fb3ab5619416dc7a8d1b813388193f647b771dd8d2aca6568e` |
| `test_virtual_a_optimizer.py` | 26 | OK | `logs/optimizer.log` | `346a8a5a217215a0a21e2b29d04a96dacab63b994c99ed566d63aaa672a50041` |
| `test_virtual_a_geometry.py` (G1, unchanged) | 20 | OK | `logs/g1_geometry.log` | `089295dc20020c3e5ce8278cb407730359d17e02428aaf8d39cb551540acf6e1` |
| `test_virtual_a_profile.py` (G1, unchanged) | 22 | OK | `logs/g1_profile.log` | `e8089227fd4bca472d24a7cc529f12febe080f29745becde0a35edea9df25d57` |

The optimizer suite grew from 22 to 26 tests.  The four additions are:

1. `test_real_ipopt_success_residual_is_finite_and_recomputable` — a real
   IPOPT success is recorded with a finite nonnegative residual that exactly
   equals a separate `_constraint_residual` call on the actual returned
   vector.
2. `test_real_slsqp_fallback_success_residual_is_finite_and_recomputable` — a
   mocked primary `Solve_Failed` plus a real SLSQP fallback produces a finite
   nonnegative fallback residual exactly equal to a separate
   `_constraint_residual` call on the actual fallback vector; the primary
   failure keeps a null residual.
3. `test_missing_or_nonfinite_success_vector_never_reports_a_residual` —
   missing, NaN and infinite fake successful vectors are recorded as
   `CONSTRAINT_RESIDUAL_FAILURE` with null residual, no audit call and no path.
4. `test_residual_calculation_anomaly_fails_closed` — a diagnostic exception
   or a NaN return is recorded truthfully, never zero, with no audit call and
   no unhandled exception.

The existing timeout tests still pass unchanged.

## 3. Frozen benchmark and residual coverage

The benchmark was rerun on the same six frozen endpoints, the straight line
and the frozen `STRAIGHT_POLICY` request with the frozen default `STRAIGHT`
config and thresholds:

- `status = PASS`; 124 checks; `failed = []`
- 8 requests; 48 solver attempts; all 48 `ipopt` / `SOLVER_SUCCESS_AUDITED`
- **successful attempts: 48**
- **non-null finite residuals: 48**
- **null residuals among successful attempts: 0**
- residual minimum: `6.925808204141147e-16`
- residual maximum: `4.707345624410664e-14`
- maximum observed solve wall: `3.437642872100696 s`; no deadline triggered

Per request:

| Request | Attempts | Finite residuals | Minimum | Maximum |
|---|---:|---:|---|---|
| E1 | 6 | 6 | `2.5757174171303632e-14` | `2.7533531010703882e-14` |
| E2 | 6 | 6 | `2.5757174171303632e-14` | `2.7533531010703882e-14` |
| E3 | 6 | 6 | `3.8708619642946474e-14` | `4.566052397292353e-14` |
| E4 | 6 | 6 | `3.8708619642946474e-14` | `4.561195171559618e-14` |
| E5 | 6 | 6 | `3.4638958368304884e-14` | `4.707345624410664e-14` |
| E6 | 6 | 6 | `3.4638958368304884e-14` | `4.618527782440651e-14` |
| STRAIGHT_LINE | 6 | 6 | `1.7763568394002505e-15` | `2.6645352591003757e-15` |
| STRAIGHT_POLICY | 6 | 6 | `6.925808204141147e-16` | `3.552713678800501e-15` |

Every attempt row in `analysis/attempts.json` contains the original request,
the initial-guess vector and label, solver, solver status, attempt status,
iterations, objective, `constraint_residual`, a derived
`constraint_residual_finite` flag and the candidate audit.  The benchmark
attempt rows and the attempt register agree on every shared field, including
the residual value.

The published best path is not stored with every attempt profile, so the
analysis compared the published profile with the attempt selected by the
production best-objective rule (`objective < best - 1e-12`).  For all eight
requests the selected attempt is index 0, and recomputing the residual from
the published profile gives exactly the recorded residual (difference `0.0`).

## 4. Auto deadline probe (real success and fallback)

`analysis/auto_deadline_probe.json` exercises five control-flow cases:

| Case | Result | Attempts | Residual evidence |
|---|---|---|---|
| `auto_ipopt_timeout` | `BUDGET_EXHAUSTED` | `SOLVER_TIMEOUT` | null, no audit |
| `auto_timeout_after_audited_candidate` | `GEOMETRY_FEASIBLE` | `SOLVER_SUCCESS_AUDITED`, `SOLVER_TIMEOUT` | success `2.5757174171303632e-14`; timeout null |
| `auto_exception_budget_exhausted` | `BUDGET_EXHAUSTED` | `SOLVER_FAILED` | null, no audit |
| `auto_ordinary_failure_real_fallback` | `GEOMETRY_FEASIBLE` | `SOLVER_FAILED`, `SOLVER_SUCCESS_AUDITED` | primary null; real SLSQP fallback `1.3989396308033974e-09` |
| `auto_slsqp_timeout` | `BUDGET_EXHAUSTED` | `SOLVER_FAILED`, `SOLVER_TIMEOUT` | both null, no audit |

The probe records the residual fields for every attempt, and all five cases
have `checks_pass = true`.  No timed-out iterate reached `audit_geometry`.
The real SLSQP fallback result is a genuine solver result, not a fabricated
numeric residual.

## 5. Evidence artifacts (SHA-256)

| Artifact | SHA-256 |
|---|---|
| `results.json` | `1b655256f30f1bd00d124137416a06b6ea5a8bb8ee08d2f44f46f7aaef6e9e61` |
| `analysis/attempts.json` | `8556fc79f1b7e6bbc603644e14331b77b347673ffd67f8baf3275b08009670b0` |
| `analysis/acceptance_summary.json` | `5ad4d159b09aa23ba43a4592f8304454f5b2b58588880fdb3b831451d2ebaefd` |
| `benchmark/results.json` | `c50fe590a1cd622151e2c6e7f3d1484915e760aa0ea1836086a7c746e34b5018` |
| `benchmark/summary.json` | `4b2b10557aa5e18afa0a841c6f2a125c9525c72013dc65f1305dfbe03fb530d4` |
| `analysis/audit_budget_crosscheck.json` | `e3adac0a2ee32d0af9503a4e8818035a9832acf10dc8e35704ddf342651a48b2` |
| `analysis/auto_deadline_probe.json` | `0d69a6b6c21c3a87d35139f8708b47a90627596d67bfc4e28a48ebd4c3ef85e9` |
| `frozen/frozen_manifest.json` | `73dcf5a3803a719ebb9047f04ae989890b4a44b3df46d70c55ce0e32495a1523` |
| `analysis/protection_diff_vs_g1_accepted.json` | `1d5a11eaab4f02ea61cda1ba35eefbacc2a23eea8c464f82053c8d4d20c59400` |
| `optimizer.py` | `7ff9d1771c653aaeea9d71ee59ad34dfbf6be9e58bb1c6c71f4cdc5988baa132` |
| `tests/test_virtual_a_optimizer.py` | `d135c11ed4569f62a333bf4da05563377a20012a1e8e0945cfd599914ca66e2f` |

`source_manifest.json`, `logs/source_manifest_generation.log` and
`logs/source_manifest_verification.log` are the three excluded self-referential
artifacts.  The source manifest binds every source and run file and is checked
by the independent verifier after these documents are frozen.

## 6. Preservation and scope

- Only `formation_continuous_a/optimizer.py` and the G2 optimizer test file
  were modified by rework-03; both were already G2 files outside accepted G1.
- The accepted G1 source tree and tests are unchanged.  Both G1 suites were
  rerun in this run and passed (20 and 22 tests).  The G1 acceptance record and
  its manifest were read only, never rewritten.
- All three previous G2 evidence runs were rechecked against their own
  manifests without modification:
  - `20261115T161608Z`: 30 run files, 0 mismatches, manifest
    `9ee32fb676215e4be8016dce64520562890e0821f21fdd88ade8ec29bfdf56be`,
    `UNCHANGED`
  - `20261115T165745Z`: 34 run files, 0 mismatches, manifest
    `5e6131f9196557a432ad2cba91c081e1f776e59cac3591463482d5b0faa3b6ad`,
    `UNCHANGED`
  - `20261115T172337Z` (rework-02): 37 run files, 0 mismatches, manifest
    `76442fdd23f439e3f25c35d24ce494293d8f47e41abcdf3fbe887ea85719e8d7`,
    `UNCHANGED`
- FR06, the mathematical model, vehicle limits, the initial state, the
  independent audit, the collision code, the frozen config and all historical
  evidence remain untouched.  No new segment count, time budget, map or
  reference input was introduced.
- No dependency was installed, no ROS/Gazebo/hardware run was performed, no
  legacy regression was run, and no Git commit, push, branch creation or G3
  entry occurred.

## 7. Non-claims

- `GEOMETRY_FEASIBLE` is a geometry result only; it is not an executable,
  timed, dynamic, collision-certified, map-aware, ROS or hardware result.
- The 30 s budget is a soft deadline: IPOPT `max_wall_time` and the SLSQP
  iteration callback are checked, but library shutdown latency is recorded,
  not pre-empted; no hard-real-time guarantee is made.
- `constraint_residual` is an NLP collocation/equality/signed-curvature
  diagnostic.  The independent whole-segment geometry audit remains the
  feasibility authority.
- Passing tests and benchmark checks is not Codex acceptance and is not
  claimed as such.

The run is ready for independent Codex review.  No G3 work has started.