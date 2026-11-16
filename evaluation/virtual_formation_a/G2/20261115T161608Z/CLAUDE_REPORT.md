# G2 report — endpoint-driven autonomous continuous A geometry

- Stage: **G2**
- Task: `docs/virtual_formation_a/CLAUDE_G2_TASK.md`
  (SHA-256 `0d6e2f7f594a4b8fe66caa2099c7fb22805b13553dbf599512667e15f5aa0c61`)
- Evidence run: `evaluation/virtual_formation_a/G2/20261115T161608Z`
- Baseline / HEAD: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Branch: `feature/virtual-rigid-formation-a`
- Run status: **PASS_PENDING_CODEX_RECHECK**
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- Next stage: **not entered**. No G3, no timing/dynamics/map model, no ROS or
  Gazebo execution, no collision certification, no hardware validation, no
  commit, push, branch, worktree reset or dependency installation.

The delivered result is a **geometry-feasibility** result only. The only
successful solve status is `GEOMETRY_FEASIBLE`; no result object contains an
`executable=True` field. This report deliberately does not infer speed,
acceleration, time, collision freedom, map validity, execution readiness or
global convergence.

## 1. What was implemented

Two production modules and their independent tests were added:

| Path | Role | SHA-256 |
|---|---|---|
| `P/formation_continuous_a/optimizer.py` | automatic initial guesses, endpoint NLP, candidate selection | `6a966c796e9c8015721dc0278ed9dfa14b83a884e861289a63523f25fc70be55` |
| `P/formation_continuous_a/audit.py` | independent geometry audit from raw knots | `71c817a8156eae2155d76e90f3b5a1a7ecdcebfdcda875822ca31aca3591b17f` |
| `P/formation_continuous_a/contracts.py` | G2 request/config/result/report contracts | `76078451f907edc5bd64811e5bb02950ff1b3dc636cc99f99ca964886180c4dc` |
| `tests/test_virtual_a_optimizer.py` | 12 optimizer/acceptance tests | `009700fda709e6508f56603ec391d95948fd2080207250a152e1e76d83b20694` |
| `tests/test_virtual_a_audit.py` | 20 independent-audit tests | `a47d57794474e6b33c69c8babeb694957ca352ff5960d4e2a9697cd2bc3479d5` |
| `tests/fixtures/virtual_a/open_requests.json` | frozen endpoint requests and default config | `5e7474ffd69aced189f4b0c8adaae6dc15f63742bc8f55400408e8c9f7f078fa` |
| `tests/fixtures/virtual_a/existence_reference.json` | independent existence/audit oracle | `217b3569663175d546e08b05d8c09e7983e78eacb2b9fc47e503b12be4528df8` |
| `tools/virtual_a_benchmark.py` | G2 benchmark (97 checks) | `2c4f4b60a208b13dc83fa9952453ad8c6c81ecaf6fb337a78e5cc92e6de164a2` |

`P = catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer`.

The following G1-accepted files are byte-identical to their accepted hashes:
`bounds.py` `3c12cca5…`, `geometry.py` `1254c334…`, `__init__.py`
`81d470f4…`, both G1 test modules, and `setup.py`. G1 limits, initial-state
validation, C2-space definition and G1 test expectations were not weakened.
The package `__init__.py` was not given G2 exports; the required functions live
in the named submodules.

Contracts added for this stage: `VirtualARequest`, `PlannerConfig`,
`validate_planner_config`, `InitialGuess`, `GeometryAuditConfig`,
`AuditReport`, `AttemptRecord`, `GeometryResult`, `request_payload`,
`request_fingerprint`, `validate_virtual_a_request`, `geometry_path_summary`.
The allowed status set is exactly:

```text
GEOMETRY_FEASIBLE / NO_MOTION / INVALID_REQUEST /
NO_ACCEPTED_CANDIDATE / BUDGET_EXHAUSTED / NUMERICAL_FAILURE
```

## 2. Endpoint-only solve and automatic initial guesses

The production entry point is:

```python
solve_open_path(request: VirtualARequest, config: PlannerConfig) -> GeometryResult
```

Only the physical request and the frozen planner config enter the solve.
There is no `S`, `alpha`, profile, fixture path, test id or reference-curve
argument. `test_solve_reads_no_reference_or_fixture_file` patches
`Path.open` during a live solve and requires **zero** opens; the source also
contains no lookup by `plan_id`, `formation_id`, test name, fixture or
evaluation path.

The endpoint NLP uses `N=8` equal intervals and variables
`S, k_0..k_N, eta_0..eta_N`; `k_0=eta_0=0`. Each interval is the G1 piecewise
cubic Hermite. C2 is imposed at internal nodes; the endpoint is integrated with
fixed 9-point Gauss collocation; the authoritative final path is rebuilt by
G1's bounded integration. The objective is

```text
J = S/l0 + .02*l0^3*integral(eta^2 ds) + .002*l0^5*integral(u^2 ds),
l0 = max(1, d).
```

Up to six automatic guesses are generated in fixed order
`(1.2,1.8,2.6) x (left,right)` from endpoint distance, heading change and
lateral need, then projected to C2 with zero endpoint curvature. No test-endpoint
table is read. Each guess is recorded in `analysis/attempts.json` with its
`S`, nodes, `kappas`, `etas`, label, shape sign, projection and source.

CasADi/IPOPT is preferred; SciPy SLSQP exists only as an explicitly labelled
fallback. Each attempt records solver, status, iterations, objective,
constraint residual, endpoint errors, wall time, rejection reason and its full
candidate audit. A solver success is never itself sufficient: every candidate
goes through `audit_geometry`. The best independently audited candidate (lowest
objective) is returned.

For the frozen run, **all six guesses solved and all six candidate audits
passed for every request**, i.e. 48 audited successful attempts. This is not a
claim that all future requests will do so.

## 3. Independent geometric audit

`audit_geometry(request, path, config, *, planning_context=None)` starts from
the raw knots. It independently rebuilds `c0..c3`, checks positive `h` and the
station domain, checks C0/C1/C2 left/right limits and the piecewise third
derivative, and checks the endpoint against the **original request**, not
against a goal carried only by the path. It records the request fingerprint,
request/path/limits summaries, first failure `(s, interval, vehicle,
constraint)`, C2 errors, integration budgets, whole-segment envelopes and a
check count. It does not call the optimizer residual as the sole verification.

Whole-segment checks:

- `A_i = 1 - b_i*k > 0`, `D_i = A_i^2 + (a_i*k)^2 > 0`;
- `C_i = (k*D_i + a_i*eta) / D_i^1.5` and `|C_i| <= 1/r_min_ackermann`;
- theoretical static wheel angle from the project ideal-wheel model:

```text
xw = +/- wheelbase/2, yw = +/- track/2
m = 1 - yw*C, n = xw*C
delta = atan2(n, m)
max |delta| over the four wheels <= steer_angle_max
```

For a closed C interval with fixed wheel, `delta(C)` is monotone and its
extremum is at an endpoint; the audit therefore evaluates both interval
endpoints. This correction replaces a coarser Ackermann-radius expression and
matches `fwis_v3_optimizer/retiming.py:wheel_configuration`. The audit does not
merely sample nodes. Interval subdivision defaults are max depth 20 and
200000 leaves; `BOUND_UNCONFIRMED` or `AUDIT_BUDGET_EXHAUSTED` cannot pass. In
the accepted runs each of the eight segments was confirmed with one leaf and
depth 0 because the interval envelope closed on the whole segment.

Rigidity, tangent heading, actual per-vehicle heading and `theta_s = C_i*Q_i`
are recomputed from the formation maps. The reported integration position
budget must be <= `1e-7 m` and the angle budget <= `1e-9 rad`, per the G1
contract.

## 4. Acceptance results

Frozen default config (verified equal to `PlannerConfig()`):
`intervals=8`, `max_initials=6`, `max_seconds=30.0`, `max_iterations=120`,
`solver='ipopt'`, `allow_scipy_fallback=True`, 9 collocation points,
`eta_limit=2.0`, `kappa_limit=1/(1+0.810)`, position tolerance `0.005 m`,
heading tolerance `0.2 deg`, rigidity tolerance `1e-6 m`. Config canonical
SHA-256: `cba9cd8cf53a4292f7adaa9725ba15c083426cc696d07a1d3aff4359e2e3d57b`
(see `frozen/frozen_manifest.json` for the exact canonicalisation).

All eight required requests solved with only `request` + frozen config:

| Request | Endpoint position error [m] | Heading error [rad] | Rigidity [m] | Position budget [m] | Angle budget [rad] | max abs C | max wheel angle [rad] | IPOPT iterations | Solve wall [s] |
|---|---:|---:|---:|---:|---:|---:|---:|---|---:|
| E1 | 4.800e-12 | 5.551e-17 | 6.66e-16 | 1.780e-08 | 1.600e-10 | 0.1129 | 0.02848 | 7,7,7,6,7,7 | 3.153 |
| E2 | 4.802e-12 | 5.551e-17 | 6.66e-16 | 1.778e-08 | 1.600e-10 | 0.1129 | 0.02848 | 7,7,6,7,7,7 | 3.101 |
| E3 | 1.951e-11 | 5.551e-17 | 8.88e-16 | 1.787e-08 | 1.600e-10 | 0.2425 | 0.06258 | 7,6,7,7,7,7 | 3.096 |
| E4 | 1.951e-11 | 5.551e-17 | 8.88e-16 | 1.781e-08 | 1.600e-10 | 0.2425 | 0.06258 | 6,7,7,7,7,7 | 3.131 |
| E5 | 3.197e-11 | 0.000e+00 | 1.11e-15 | 2.386e-08 | 1.600e-10 | 0.4285 | 0.11429 | 7,6,7,7,7,7 | 3.125 |
| E6 | 3.197e-11 | 0.000e+00 | 6.66e-16 | 2.372e-08 | 1.600e-10 | 0.4285 | 0.11429 | 6,7,7,7,7,7 | 3.126 |
| STRAIGHT_LINE `(6,0,0)` | 8.882e-16 | 3.232e-20 | 4.44e-16 | 1.776e-08 | 1.600e-10 | 0.0000 | 0.00000 | 18,18,9,9,9,9 | 3.531 |
| STRAIGHT_POLICY frozen | 8.316e-10 | 5.551e-17 | 6.66e-16 | 1.991e-08 | 1.600e-10 | 0.5131 | 0.13890 | 6,8,7,10,8,9 | 3.218 |

Acceptance limits: position `<= 5e-3 m`, heading `<= 3.490659e-3 rad`,
rigidity `<= 1e-6 m`, position budget `<= 1e-7 m`, angle budget
`<= 1e-9 rad`, `|C_i| <= 1.2345679012 1/m`,
wheel angle `<= 1.5707963268 rad`. All eight are far inside every limit.

Every request has 6 attempts, all `SOLVER_SUCCESS_AUDITED`, all candidate audits
passed, all solver statuses `Solve_Succeeded`, all iterations <= 120, all
`deadline_exhausted=False`. The 6 non-straight-policy requests and the straight
line all start at `(0,0,0)` with car headings `0`, stationary, A mode and
straight wheels.

`STRAIGHT_POLICY` terminal curvature and terminal eta are both frozen to 0
(`|k_N|,|eta_N| <= 1e-9`), and the goal is the independently integrated
clamped CubicSpline endpoint. `FREE` requests do not impose those two
equations. A separate real audit test constructs legal terminal nonzero `k`
and nonzero `eta` paths: both pass under FREE and are rejected under STRAIGHT,
so no hidden "return to straight" rule exists.

## 5. Time budget and deadline semantics

The 30 s value is a **soft total budget**, checked at initial and attempt
boundaries. Each IPOPT call runs internally with `max_iter=120`; individual
IPOPT iterations are not polled. A library-call overrun would be recorded
honestly, not pre-empted; this run did not hit one.

Actual maximum solve wall: `3.5308071420295164 s` (STRAIGHT_LINE). All per-request
walls are in the table and in `analysis/acceptance_summary.json`. Timeout
point: `NOT_TRIGGERED`. No budget was enlarged; intervals stayed at 8 and
initial count at 6.

## 6. Tests and required negative fixtures

| Suite | Result | Exit | Wall |
|---|---|---:|---:|
| `test_virtual_a_audit.py` | 20/20 OK | 0 | 0.220 s |
| `test_virtual_a_optimizer.py` | 12/12 OK | 0 | 46.827 s |
| `test_virtual_a_geometry.py` (G1 regression) | 20/20 OK | 0 | 1.075 s |
| `test_virtual_a_profile.py` (G1 regression) | 22/22 OK | 0 | 1.289 s |
| G2 benchmark | 97/97 PASS, 0 failed | 0 | solve walls above |

Required negative/failure checks, all passing:

- non-C2 profile -> `NON_C2_PROFILE` at `C2_SECOND`;
- car 1 measured initial heading `0.1 rad` -> `INITIAL_HEADING_MISMATCH`;
- forged request digest -> `FORGED_REQUEST_DIGEST`;
- forged goal is checked against the original request -> endpoint failure;
- wrong vehicle shape -> `VEHICLE_SHAPE_MISMATCH`;
- invalid limit (`r_min_ackermann=0`) -> `INVALID_LIMITS`;
- NaN goal -> `INVALID_REQUEST`;
- static-wheel whole-segment limit with `steer_angle_max=0.1` -> 
  `STATIC_WHEEL_LIMIT`, with the reported value recomputed from the
  ideal-wheel formula at the reported station/vehicle;
- segment-interior curvature violation with legal nodes: fixed fixture
  `(s,k,eta) = (0,0,0), (0.1,0,10)`, Hermite in `q=s/h` with
  `h=0.1, c0=0,c1=0,c2=-1,c3=1`, so `k=-q^2+q^3` and
  `eta=(-2q+3q^2)/h`; independently rejected at an interior station with
  `CURVATURE_LIMIT`;
- unconfirmable enclosure -> `CURVATURE_LIMIT` at depth 0 and
  `AUDIT_BUDGET_EXHAUSTED` with one leaf; neither is published as feasible;
- solver exception -> `NUMERICAL_FAILURE`, `IPOPT_EXCEPTION` attempt;
- very small total budget -> `BUDGET_EXHAUSTED` with no path;
- fake solver success with wrong endpoint -> every attempt
  `SOLVER_SUCCESS_AUDIT_REJECTED`, no feasible path;
- fake solver success with curvature violation -> no feasible path;
- exact `(0,0,0)` goal -> `NO_MOTION` hold; `(1e-9,0,0)` and
  `(0,0,0.1)` are not treated as hold;
- G1 full double-lane-change endpoint `(11.6445099698,0,0)` is accepted as a
  straight path; no test demands an autonomous double lane change;
- clamped CubicSpline STRAIGHT reference with stations 0..6 and curvature
  `[0,.10,.20,0,-.20,-.10,0]` passes independent audit;
- production solve reads no reference/fixture/evaluation file (zero opens);
- six initial guesses are fixed-order, reconstructible, C2 and request-only.

`benchmark/summary.json` also contains the real segment-interior negative
fixture, with first failure at `s=0.006856250762939453`, vehicle 1, value
`-1.2345687277601716 1/m` against limit
`1.2345679012345678 1/m`, depth 20. The benchmark result is PASS with 97
checks and 0 failures.

## 7. Frozen requests, independent existence reference and isolation

`frozen/open_requests.json` and `frozen/existence_reference.json` are
byte-identical copies of the fixture files; `frozen/frozen_manifest.json`
records both hashes, the default planner config canonical hash and proof that

```python
PlannerConfig(**open_requests["planner_config"]) == PlannerConfig()
```

is true (and the dict equals `PlannerConfig().as_dict()`). Each solved
request's `request_sha256` and `request_summary` match the frozen base and
per-request record, so the benchmark solved exactly the frozen requests, not a
different or rounded goal.

The independent existence oracle is:

- `k(s)=alpha*(s/S)^2` for `(S,alpha)` =
  `(6,.1),(6,-.1),(6,.2),(6,-.2),(8,.3),(8,-.3)`;
- a clamped `scipy.interpolate.CubicSpline` through stations `0..6` with
  curvature `[0,.10,.20,0,-.20,-.10,0]` and zero endpoint slopes.

The exact endpoints in the oracle match the frozen rounded request goals within
`1e-9 m` and `1e-12 rad`. The oracle is used only by tests/benchmark and audit;
it is never a warm start, seed, cache, lookup table or production input. The
`Path.open` isolation test and the production source scan are two independent
data-flow checks.

## 8. Protection and repository state

`analysis/protection_diff_vs_g1_accepted.json` compares the current protected
tree with the Codex-accepted G1 snapshot
`evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/protected_after.json`
(tree `910efe32…`, 3127 files). The accepted G1 evidence and source manifest
were rehashed first and all 47 manifest run files still match; G1 acceptance
remains `ACCEPTED` with `next_stage_authorized=false`.

Exactly six protected additions and two authorized tracked changes exist, with
no removals:

* additions: `audit.py`, `optimizer.py`, `tests/test_virtual_a_audit.py`,
  `tests/test_virtual_a_optimizer.py`,
  `tests/fixtures/virtual_a/open_requests.json`,
  `tests/fixtures/virtual_a/existence_reference.json`;
* changes: `contracts.py` (G1 `ae267d88…` -> G2 `76078451…`) and
  `tools/virtual_a_benchmark.py` (G1 `df804b04…` -> G2 `2c4f4b60…`);
* removals: none; the 3125 unchanged common protected entries reproduce an
  identical canonical tree.

All old FR06/single-vehicle/ROS/configuration/collision code and all historical
evidence under `evaluation/formation_refactor_v02` are unchanged. HEAD remains
the formal ancestor, there are no staged changes, and no commit or push was
made. `setup.py` was already part of the accepted G1 tree and is unchanged in
this stage.

## 9. Evidence map

| Evidence | Content |
|---|---|
| `benchmark/summary.json` | 97 checks, config, request names, compact results, negative fixture |
| `benchmark/results.json` | full request, endpoint, actual profile pieces, path summary, full final audit, all attempts and candidate audits, walls |
| `results.json` | assembled request/solution/audit/profile/endpoint table, acceptance metrics, tests, negative coverage, benchmark binding |
| `analysis/attempts.json` | all 48 attempts and the six reconstructible initial guesses per request |
| `analysis/acceptance_summary.json` | compact quantitative acceptance table, maximums, deadline record, test-log evidence, negative coverage |
| `analysis/protection_diff_vs_g1_accepted.json` | protected-tree diff and G1/G0 evidence preservation |
| `frozen/` | byte-identical requests/reference, config equality proof, request fingerprints |
| `environment.json` | host, Python 3.8.10, CasADi 3.7.2, IPOPT availability, package versions |
| `logs/` | mandatory tests, benchmark, protection diff, freeze, assembly, environment, source-manifest generation/verification |
| `source_manifest.json` | binds all source, fixtures, evidence and protected snapshots |

## 10. Explicit nonclaims and known limitations

- No timing, speed, acceleration, jerk, yaw-rate or dynamic-feasibility claim.
- No map, obstacle, collision-certification or free-space claim.
- No ROS, Gazebo, controller, hardware or calibration claim.
- No claim of global convergence or that every future endpoint is solvable.
- No claim that all possible homotopy classes are explored; the heading
  endpoint uses the recorded shortest-wrap branch only.
- `GEOMETRY_FEASIBLE` means the returned geometry passed this independent
  audit under the frozen request limits. It is not `EXECUTABLE`.
- The 30 s budget is soft and contains no hard real-time pre-emption.
- G2 tests and 97 benchmark checks are evidence for Codex review, not
  self-acceptance. **Codex acceptance remains NOT_RUN_NOT_CLAIMED.**

## 11. Suggested independent re-checks

1. Rebuild the six E1–E6 goals from `frozen/open_requests.json`, run
   `solve_open_path(request, PlannerConfig())`, and confirm `request_sha256`,
   endpoint errors and `GEOMETRY_FEASIBLE`.
2. Rerun a mirrored pair (E5/E6) and a rotated/translated E1; compare the knot
   profile and path length, not only the status.
3. Recompute `C_i` from the recorded raw knots and profile pieces with an
   independent fine quadrature/dense grid; confirm the whole-segment maxima
   and static-wheel values in `analysis/acceptance_summary.json`.
4. Rebuild a FREE nonzero terminal `k`/`eta` path and the STRAIGHT policy
   variant; confirm PASS/`GOAL_POLICY_VIOLATION`.
5. Replay the segment-interior fixture and the fake-solver-success tests;
   confirm rejection and the absence of any `GEOMETRY_FEASIBLE` path.
6. Rehash the protected tree against the accepted G1 snapshot; confirm only
   the six additions and two authorized changes, no removals.