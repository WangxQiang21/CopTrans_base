# G2 Rework 01 report — fail-closed audit config, independent integration budget, soft deadline

- Stage: **G2 only**; this is a rework of the rejected G2 delivery, not G3.
- Governing task: `docs/virtual_formation_a/CLAUDE_G2_TASK.md`
  (SHA-256 `0d6e2f7f594a4b8fe66caa2099c7fb22805b13553dbf599512667e15f5aa0c61`).
- Rework instruction: `docs/virtual_formation_a/CLAUDE_G2_REWORK_01.md`
  (SHA-256 `dc200c01bb48616afb765f2b7e957ef8ed523f3547f870fbf90995b60ea391af`).
- Evidence run: `evaluation/virtual_formation_a/G2/20261115T165745Z`.
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

## 1. Rework outcome

| Item | Result | Primary evidence |
|---|---|---|
| P0: every `GeometryAuditConfig` field validated before use | implemented; invalid config returns `INVALID_AUDIT_CONFIG` | `contracts.py`, 6 config tests |
| P0: NaN/±inf/wrong-type/negative/illegal-count negatives over every field | implemented, including all C2, position, heading, rigidity, integration and angle limits | `test_virtual_a_audit.py` |
| P0: real 6 m path / 10 m goal / NaN tolerance counterexample | reproduced; NaN config rejected, default config rejects endpoint by 4.0 m, same path accepted for a 6 m goal | `RealCounterexampleTests`, benchmark |
| P0: solver-to-audit config conversion | PlannerConfig tolerances are converted and revalidated; NaN cannot silently enter audit | `_audit_config`, optimizer test |
| P1: independent integration remainder from raw knots | composite Simpson with independently rebuilt `m4`; 264 panels on cross-check path; total `1.5143e-08 m` against fixed `5e-8 m` threshold | `audit.py`, cross-check JSON |
| P1: numerical high-precision comparison | independent audit minus SciPy quadrature: `4.009e-11 m`; returned path minus independent: `8.951e-16 m` | `analysis/audit_budget_crosscheck.json` |
| P1: soft deadline | IPOPT `max_wall_time`, SLSQP iteration callback, boundary checks; timed-out iterate never audited; prior audited candidate preserved | `optimizer.py`, 5 deadline tests |
| Regression | 31 audit, 17 optimizer, 20 G1 geometry, 22 G1 profile tests; 124/124 G2 benchmark checks pass | logs and benchmark summary |

## 2. Fail-closed audit configuration (P0)

### 2.1 Validation before use

`validate_geometry_audit_config(config)` is called at the start of
`audit_geometry`, after the type check and **before any request, geometry,
hold or interval branch**. The optimizer also converts the three tolerances
supplied through `PlannerConfig` into `GeometryAuditConfig` and revalidates the
whole object in `_audit_config`; a failure raises
`VirtualAContractError("INVALID_AUDIT_CONFIG", ...)`. No tolerance is enlarged
and no default limit was relaxed.

All 17 configuration fields are checked:

- integer domains: `max_depth`, `max_leaves`,
  `sample_points_per_segment`, `audit_max_subintervals_per_segment`,
  `audit_max_total_subintervals`. A `bool`, a float such as `9.0`, zero where
  forbidden, a negative value, or an out-of-range count is rejected as
  `INVALID_AUDIT_CONFIG`. Integer fields are not silently truncated.
- non-negative finite domains: `c2_value_tol`, `c2_slope_tol`,
  `c2_second_tol`, `c2_third_abs_limit` (zero is legal).
- strictly positive finite domains: `position_tolerance_m`,
  `heading_tolerance_rad`, `rigidity_tolerance_m`, `integration_budget_m`,
  `angle_budget_rad`, `interval_curvature_tolerance`,
  `audit_panel_width_m`, `audit_position_budget_m`.
- cross-field rule: `audit_position_budget_m <= integration_budget_m`.
  An audit budget cannot exceed the accepted G1 position contract.
- every real field rejects `bool`, wrong types, NaN and both infinities; the
  error location names the exact field.

`_finite_audit` additionally guards intermediate sample, curvature, wheel,
integration and endpoint values. A non-finite intermediate produces
`NONFINITE_AUDIT_VALUE`, never a pass. This closes the Python comparison trap
where every ordered comparison with NaN is false.

### 2.2 Parameterised negative tests

`AuditConfigFailureClosedTests` covers every field, not only the originally
reported `position_tolerance_m`:

- each integer field with bool, float, zero where forbidden, and negative
  values;
- each of the 12 real fields with NaN, `+inf`, `-inf`, negative, wrong type
  (`"0.1"`), and bool; positive-only fields also with zero;
- the cross-field audit/G1 budget inequality;
- the valid zero endpoints of the four non-negative C2 limits;
- invalid config on an exact hold request: rejected before the hold branch;
- a monkeypatched non-finite independent integral: `NONFINITE_AUDIT_VALUE`.

The parameterisation explicitly includes `position_tolerance_m`,
`heading_tolerance_rad`, `rigidity_tolerance_m`, `integration_budget_m`,
`angle_budget_rad`, and all three C2 tolerances, plus every other audit field.

## 3. Finite-value counterexample

This is a real path/request object, not a mocked audit result. The path is the
legal 6 m straight cubic profile from raw records `(0,0,0)` to `(6,0,0)`;
the request goal is `(10,0,0)`.

| Case | Result |
|---|---|
| audit config `position_tolerance_m = NaN`, 10 m goal | `INVALID_AUDIT_CONFIG`, field `position_tolerance_m`, no hold/geometry claim |
| default audit config, 10 m goal | `ENDPOINT_POSITION_ERROR`; independent endpoint error `4.0 m`; tolerance `0.005 m`; constraint `ENDPOINT_POSITION` |
| same 6 m path with matching 6 m goal | audit passes |

The benchmark independently repeats the same three cases in
`negative.counterexample`: `INVALID_AUDIT_CONFIG`,
`ENDPOINT_POSITION_ERROR` with `4.0 m`, and a passing valid path. Thus the
counterexample cannot pass merely because an audit was skipped or mocked.

## 4. Independent integration error budget and numerical cross-check (P1)

### 4.1 Reconstruction and derivation

The audit rebuilds every cubic piece from the raw knots (`c0..c3`, `h`,
station domain) and reconstructs `k`, `eta`, `u`, `g` itself. It does not use
the optimizer residual and does not copy the path's self-reported
`position_error_bound_m`. The certified budget is:

```text
one composite-Simpson panel remainder:  H^5 * M4 / 2880
M4 upper bound:  G + 4*K*U + 3*E^2 + 6*K^2*E + K^4
```

where `K,E,U,G` are independently computed interval upper bounds for
`|k|,|eta|,|u|,|g|` from the generic G1 outward operators
(`segment_jet_bounds`). The initial panel count per segment is an even count
at least `h / audit_panel_width_m`; full panels and the local one-panel
remainder are kept separately. `position_budget` forms per-coordinate
remainders, the Euclidean bound is `sqrt(2)` times the per-coordinate bound,
and the documented G1 floating-point/angle guard is added. The fixed audit
threshold is **`5e-8 m`**, strictly below the accepted G1 `1e-7 m` position
contract. Failure to confirm a budget returns `BOUND_UNCONFIRMED`; it can
never return feasible.

The audit checks both independent and reported endpoint claims against the
**original request**:

- independent endpoint error plus audit budget against `position_tolerance_m`;
- independent heading error plus audit angle budget against
  `heading_tolerance_rad`;
- the path's reported endpoint plus its G1 position budget against the same
  tolerance;
- reported-vs-independent endpoint and heading disagreement against the sum
  of the G1 and audit budgets.

This prevents both a self-consistent false claim and a pass obtained only by
sampling a few points.

### 4.2 Cross-check path and actual numbers

`analysis/run_audit_budget_crosscheck.py` builds a legal four-segment C2
curvature profile at stations `(0, 1.1, 2.4, 3.9, 5.2)` m:

```text
k(s) = 0.012*s^2 - 0.0015*s^3
```

so the fourth derivative is nonzero. The request goal is the path endpoint.
The audit is compared with SciPy `quad` using `epsabs=epsrel=1e-13` and the
segment breakpoints. Actual recorded values:

| Quantity | Value |
|---|---:|
| audit method | `independent_composite_simpson_v1` |
| total subintervals / refinement steps | `264` / `0` |
| per-coordinate remainder | `8.412192079e-11 m` |
| Euclidean remainder | `1.189663613e-10 m` |
| roundoff/angle guard | `1.502401289e-08 m` |
| total audit budget | `1.514297925e-08 m` |
| fixed threshold | `5e-8 m` |
| independent audit minus SciPy quadrature (Euclidean) | `4.008989595e-11 m` |
| returned path minus independent audit (Euclidean) | `8.950904183e-16 m` |
| coarse-grid refinement steps | `12`; confirmed |
| resource-limited audit | `BOUND_UNCONFIRMED`, `AUDIT_INTEGRATION_TOTAL_LIMIT` |
| unreachable-budget audit | `BOUND_UNCONFIRMED`, `AUDIT_INTEGRATION_PANEL_LIMIT` |

The quadrature agreement is `4.0e-11 m`, inside the certified audit budget and
the reported `quad` absolute-error estimates. The returned path is consistent
with the independent endpoint to `8.95e-16 m`, far inside the combined
position budget. The cross-check is evidence only; neither its profile nor its
quadrature result feeds the production solver.

### 4.3 Frozen-run budgets

Across the eight frozen requests the maximum audit budget is
`2.386947092e-08 m`, the maximum G1 path budget is `2.385618175e-08 m`, and the
maximum combined budget is `4.772565267e-08 m`. Every audit budget is
confirmed, below `5e-8 m`, and therefore below the G1 `1e-7 m` contract. The
maximum audit angle bound is `1.600034727e-10 rad`, below `1e-9 rad`. No
tolerance was enlarged.

## 5. Soft deadline behaviour (P1)

The frozen `max_seconds=30 s` remains a **soft total budget**, checked at:

- the initial boundary before the first attempt;
- the post-attempt boundary after every attempt;
- the final boundary.

Two solver controls are implemented:

- IPOPT receives `ipopt.max_wall_time`; the wrapper tests the actual
  `Maximum_WallTime_Exceeded` return status and records
  `deadline_trigger_stage="ipopt_max_wall_time"`.
- SLSQP uses a Python iteration callback that raises `_SoftDeadline`; it
  records `SLSQP_SOFT_DEADLINE`,
  `deadline_trigger_stage="slsqp_iteration_callback"`, its iteration count and
  the remaining budget.

A timed-out solver result is **never** treated as a candidate and **never**
passed to `audit_geometry`. It is recorded as `SOLVER_TIMEOUT` with reason
`SOFT_BUDGET_EXHAUSTED`; a previously audited best candidate may still be
returned, in which case `audited_candidate_preserved=True` and its audit is
unchanged. The result/log records `deadline_summary` with soft budget, trigger
stage, trigger wall, solver status, final wall, and
`hard_real_time_guarantee=False`. No hard real-time or immediate interruption
claim is made: library shutdown/return overhead can exceed the trigger wall
and is recorded rather than pre-empted.

Tests execute real solver paths, not mocks of the timeout policy:

- real IPOPT with a `1e-9 s` wall limit returns
  `Maximum_WallTime_Exceeded`, is not `success`, and records the true trigger;
- real SLSQP with a tiny wall limit stops in the iteration callback and
  reports `SLSQP_SOFT_DEADLINE` with at least one iteration;
- a fake solver returning `success=True` plus a timeout is still rejected by
  the production wrapper: zero audit calls, `BUDGET_EXHAUSTED`,
  `path=None`, `audit=None`;
- a genuine audited candidate followed by a timeout preserves the audited
  path/audit and records both attempts;
- a NaN `PlannerConfig` tolerance, including a bypass through
  `object.__setattr__`, cannot enter the audit conversion and raises
  `INVALID_AUDIT_CONFIG`.

In the frozen run no deadline triggered (`timeout_point=NOT_TRIGGERED`); the
maximum solve wall was `3.528343322 s`.

## 6. Regression, frozen solve and benchmark

All four required suites were rerun from the worktree:

| Suite | Result | Log |
|---|---:|---|
| `test_virtual_a_audit.py` | 31/31 pass | `logs/audit.log` |
| `test_virtual_a_optimizer.py` | 17/17 pass | `logs/optimizer.log` |
| `test_virtual_a_geometry.py` (G1) | 20/20 pass | `logs/g1_geometry.log` |
| `test_virtual_a_profile.py` (G1) | 22/22 pass | `logs/g1_profile.log` |

The G2 benchmark ran all eight frozen requests with only the request and the
frozen config: **124 checks, `failed=[]`, status `PASS`**.
`G1 bounds.py`, `geometry.py`, `__init__.py`, both G1 test modules and
`setup.py` remain byte-identical to the accepted G1 hashes. No accepted G1
test expectation was changed.

| Request | Endpoint pos [m] | Heading [rad] | Rigidity [m] | G1 budget [m] | Audit budget [m] | Combined [m] | max abs C | max wheel [rad] | IPOPT iterations | Solve wall [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|
| E1 | 4.800e-12 | 5.551e-17 | 6.661e-16 | 1.780e-08 | 1.780e-08 | 3.560e-08 | 0.1129 | 0.02848 | 7,7,7,6,7,7 | 3.4854 |
| E2 | 4.802e-12 | 5.551e-17 | 6.661e-16 | 1.778e-08 | 1.780e-08 | 3.559e-08 | 0.1129 | 0.02848 | 7,7,6,7,7,7 | 3.2374 |
| E3 | 1.951e-11 | 5.551e-17 | 8.882e-16 | 1.788e-08 | 1.789e-08 | 3.576e-08 | 0.2425 | 0.06258 | 7,6,7,7,7,7 | 3.1677 |
| E4 | 1.951e-11 | 5.551e-17 | 8.882e-16 | 1.781e-08 | 1.789e-08 | 3.569e-08 | 0.2425 | 0.06258 | 6,7,7,7,7,7 | 3.1432 |
| E5 | 3.197e-11 | 0.000e+00 | 1.110e-15 | 2.386e-08 | 2.387e-08 | 4.773e-08 | 0.4285 | 0.11429 | 7,6,7,7,7,7 | 3.1724 |
| E6 | 3.197e-11 | 0.000e+00 | 6.661e-16 | 2.372e-08 | 2.387e-08 | 4.759e-08 | 0.4285 | 0.11429 | 6,7,7,7,7,7 | 3.1730 |
| STRAIGHT_LINE | 8.882e-16 | 3.232e-20 | 4.441e-16 | 1.776e-08 | 1.776e-08 | 3.552e-08 | 0.0000 | 0.00000 | 18,18,9,9,9,9 | 3.5283 |
| STRAIGHT_POLICY | 8.316e-10 | 5.551e-17 | 6.661e-16 | 1.991e-08 | 2.291e-08 | 4.282e-08 | 0.5131 | 0.13890 | 6,8,7,10,8,9 | 3.2453 |

Acceptance limits remain endpoint position `0.005 m`, endpoint heading
`0.2 deg = 3.490659e-3 rad`, rigidity `1e-6 m`, G1 position budget `1e-7 m`,
angle budget `1e-9 rad`, and the frozen physical C/wheel limits. All 48
attempts were independently audited successful attempts; six automatic
initial guesses were used per request as before.

Negative coverage remains intact: 40 named entries in the acceptance summary,
including non-C2 profiles, wrong initial heading, forged request digest,
forged goal, wrong vehicle shape, invalid limits, interior curvature violation,
unconfirmed enclosure, config failures, real counterexample, integration
resource failures and deadline cases. No negative test was deleted.

## 7. Protection, evidence and reproducibility

The new evidence run is unique. The previous run
`evaluation/virtual_formation_a/G2/20261115T161608Z` was not modified: all 30
files listed in its old manifest still match their recorded SHA-256, and its
`source_manifest.json` SHA-256 remains
`9ee32fb676215e4be8016dce64520562890e0821f21fdd88ade8ec29bfdf56be`.

`analysis/protection_diff.py` reports exactly:

- 6 authorized additions: G2 `audit.py`, `optimizer.py`, both G2 test modules,
  both frozen request/reference fixtures;
- 2 authorized changes: `contracts.py` and `tools/virtual_a_benchmark.py`;
- 0 removals;
- full protected tree
  `0c0efd3d0a984ee2e11c103acc2c5421fcbff7d5e05044dbf334ec60f31d33df`,
  G1 tree `910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519`,
  `only_authorized=true`.

Source and test hashes of this delivery:

| Path | SHA-256 |
|---|---|
| `formation_continuous_a/audit.py` | `2c7da5165498a4fd5273dea336bf51c4b652f4c1cd5b31615f78b34367a7a58d` |
| `formation_continuous_a/optimizer.py` | `9bad4d2acbf4d7a79d41f1c2fa1d93ddcf2b8e9c0223f92fa64125caa1d24a39` |
| `formation_continuous_a/contracts.py` | `27b101f4a8143f549cab67794999a5fb7a9d9a66e253e3db0518bd0745a1ec69` |
| `tests/test_virtual_a_audit.py` | `979b5d69df56bb28751c6491971716e4d89b2e69d6476d0d18407c4f909c8503` |
| `tests/test_virtual_a_optimizer.py` | `6d476847a41bf8bd485457f4b5eb6e5a628c3045ea6f5af0fc37b36a34a283e7` |
| `tools/virtual_a_benchmark.py` | `92368b1714fdee494fd31a2f57270148904ef87893b5b95cb5961647715a9cad` |

`bounds.py` (`3c12cca5…`), `geometry.py` (`1254c334…`) and `__init__.py`
(`81d470f4…`) were not modified. The source manifest binds the full file set,
hashes, logs, attempts, acceptance evidence and protected-tree comparison.

Two analysis/verifier-script failures are recorded honestly. First, the
initial `run_audit_budget_crosscheck.py` invocation exited `1` with a
`KeyError: 'max_abs_g'` while formatting a failed-audit record that had no C2
block. The script was corrected to use `report.c2 or {}` and
`.get("max_abs_g")`; the same command was rerun and exited `0`. Second, the
first source-manifest verifier run against the first generated manifest
exited `1` with `KeyError: 'location'` when checking the counterexample's
`default_config`; the serialized audit report uses `first_failure`. The
verifier was corrected, the manifest regenerated, and the final verification
exited `0` with `status=OK`, `failures=[]`, 34 run files, 23 source files, 48
attempts and 124 benchmark checks. Both first-attempt exit codes are preserved
in `logs/exit_codes.log`; the failed stdout traces were overwritten by the
successful reruns. These were evidence/verifier issues, not production-audit
passes or changed production results.

All commands, exit codes and logs are in `COMMANDS.md` and `logs/`. The full
audit budget (segments, panel counts, `m4` bounds, remainders, guard,
threshold, refinement) is present for every attempt in
`analysis/attempts.json`, `benchmark/results.json` and the per-request
`acceptance_summary.json`.

## 8. Floating-point assumptions and non-claims

The Simpson truncation remainder is derived analytically from the cubic knots
and the generic G1 fourth-derivative bound. The floating-point guard uses the
accepted G1 roundoff terms, plus the documented angle accumulation guard
(`8*EPSILON*(1+|psi|)`). This is **not** a formal proof of correctly rounded
libm/trig operations or of every IEEE-754 accumulation order; libm/trig values
are assumed to lie within those guards. No interval arithmetic library and no
hardware floating-point proof is claimed.

Other explicit non-claims:

- `GEOMETRY_FEASIBLE` is not execution, timing, dynamics, collision, map or
  ROS feasibility.
- The 30 s deadline is soft; IPOPT/SLSQP are asked to stop, but library
  shutdown latency is recorded, not pre-empted. No hard real-time guarantee.
- The quadratic/clamped-spline references are independent oracles, never
  solver seeds or production inputs.
- The cross-check quadrature is independent evidence but is not part of the
  production solve.
- Passing tests and 124 benchmark checks is **not** Codex acceptance.
- G3 was not started; no commit was made.

**Conclusion:** the rework addresses the P0 bypass and the missing independent
integration budget, implements testable soft-deadline semantics, preserves G1
and prior evidence, and stops here for independent Codex review.