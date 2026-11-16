# G1 implementation report — virtual rigid formation A

- Stage: **G1**
- Submitted run: `evaluation/virtual_formation_a/G1/20261115T150639Z`
- Superseded preliminary probe: `evaluation/virtual_formation_a/G1/20261115T150114Z`
  is preserved unchanged.  It predates the final mandatory non-C2 fixture test
  and the duplicate-knot / non-positive-segment split, so its source manifest
  binds an earlier source state and it must not be used for acceptance.
- Baseline / HEAD: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Branch: `feature/virtual-rigid-formation-a`
- Result of the commands in this run: **PASS** (20 geometry tests, 16 profile
  tests, 23 benchmark checks, geometry regression PASS)
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**.  This is a Claude implementation
  report, not an acceptance decision.
- Next stage: **not entered**.  No G2 code, no NLP, no ROS/Gazebo run, no
  commit or push.

## 1. Files added and the one authorized modification

New production package (the only new import surface):

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/__init__.py`
- `.../formation_continuous_a/contracts.py`
- `.../formation_continuous_a/bounds.py`
- `.../formation_continuous_a/geometry.py`

New tests and diagnostic tool:

- `tests/test_virtual_a_geometry.py` (20 tests)
- `tests/test_virtual_a_profile.py` (16 tests)
- `tools/virtual_a_benchmark.py` (analytic diagnostic only)

The only tracked file modified is
`catkin_ws/src/fwis_v3_optimizer/setup.py`: the package list now contains
`fwis_v3_optimizer` and `fwis_v3_optimizer.formation_continuous_a`.  No legacy
`uniform_*`, formation core, `continuous_geometry.py`, YAML, map, old test or
old evidence file was modified.  This is confirmed by
`protection_diff_vs_g0.json`: compared with the accepted G0 protected tree the
only additions are the seven G1 paths above and the only change is `setup.py`;
there are no unexpected additions, changes or removals.  Within this run the
protected tree hash is identical before and after
(`35ebb7fecc1a0dde7955b6218b2c21cf65d9bdb6705fc5921718bcad652c6634`);
the accepted G0 protected tree was
`ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091`
(3120 files, versus 3127 after the seven additions).

## 2. Implemented G1 contract

### 2.1 Point mapping and initial state

The A-only point map reuses `FormationModel`/`VehicleOffset`/`FormationPose`
and the authoritative offsets `1:(1,0), 2:(0,1), 3:(0,-1) m`:

- `A = 1 - b k`, `B = a k`, `D = A^2 + B^2`, `Q = sqrt(D)`
- `theta_i = psi + atan2(B, A)`
- `H_i = k + a eta / D`, `C_i = H_i / Q`
- `p_s = R(psi) (A, B)`, with the analytic `p_ss`/`p_sss` jets and
  `theta_s/theta_ss/theta_sss`, `C_s/C_ss`.

`A <= 0` or `D <= 0` is rejected as `NORMAL_BRANCH_VIOLATION` with
`vehicle_id`, `s`, `A` and `D`; it is never clipped.  `InitialAssumption`
requires `stationary=True`, `mode='A'`, `wheels_straight=True`; measured
headings are compared individually against `psi` with `1e-7 rad` tolerance and
are never first overwritten by `psi`.  ROS sources require a non-empty
`context_id`; G1 tests use only `offline_fixture`.

### 2.2 C2 profile and bounds

`C2CurvatureProfile.from_knots()` builds the cubic Hermite pieces
`c0 = k0`, `c1 = h eta0`,
`c2 = 3(k1-k0)-h(2 eta0+eta1)`,
`c3 = 2(k0-k1)+h(eta0+eta1)`, then cross-checks every piece against the reused
`MotionBlock(..., slope_policy="fixed")` polynomial.  It rejects a C0/C1/C2
discontinuity as `NON_C2_PROFILE` with `node`, `s` and the left/right second
derivatives.  A duplicate station is rejected as `NON_POSITIVE_SEGMENT`
(`h=0`, with `index`/`s`), a decreasing station as `INVALID_KNOT_ORDER`;
neither is clipped or silently reordered.

`bounds.py` implements outward-rounded interval Horner bounds with 16 ulp
coefficient widening, the fourth-derivative envelope
`M4 = G + 4KU + 3E^2 + 6K^2E + K^4`, and the composite-Simpson remainder
`(panels+1) H^5 M4 / 2880`.  The production integral uses fixed Simpson panels
that never cross a Hermite node; position and angle budgets are required to be
at most `1e-7 m` and `1e-9 rad`, otherwise `BOUND_UNCONFIRMED` is raised.
No sampled maximum is used as a supremum.

### 2.3 Analytic reference used for G1 only

The mandatory profile is `k = K sin^3(2 pi s / L)` on the first piece and the
mirrored second piece, with `K=0.30`, `L=6 m`; the mirror branch negates the
jets.  Its `eta`, `u`, `g` and closed-form `psi` are evaluated analytically.
At the middle node `s=L` the reference has `k=eta=u=0` but a genuine
third-derivative jump: `g(L^-)=+2.0670851120 1/m^3` and
`g(L^+)=-2.0670851120 1/m^3`.  This is expected for the two `sin^3` pieces and
is used to verify `side='left'/'right'/'auto'`; it is not rejected.  The
profile is a diagnostic/benchmark input (`serialization_ready=False`), is not
imported by the production package and is not a planner input or initial
guess.

## 3. Results

### 3.1 Mandatory endpoint and peak values

| Quantity | Expected | Measured |
|---|---|---|
| `p_F(L)` | `(5.8222549849, 1.1256889273)` | `(5.822254984922958, 1.1256889272858532)` |
| `p_F(2L)` | `(11.6445099698, 0)` | `(11.644509969845917, -2.22e-16)` |
| `psi(L)`, `psi(2L)` | `0` | `1.59e-17` (both) |
| curvature signs | `+,-,-,+` | `+,-,-,+` |
| mirror signs | `-,+,+,-` | `-,+,+,-` |
| `max eta` (analytic sup) | `0.3627598728` | sampled `0.36275987095` (below sup) |
| front `max |C|` | `0.526313` | `0.52631317048` |
| side `max |C|` | `0.428571` | `0.42857142857` |

Position endpoint checks use closed-form `psi` and strict adaptive
`scipy.integrate.quad`; production endpoint to quadrature is
`9.01e-10 m`.  Rigid pairwise distances are `sqrt(2), sqrt(2), 2` within
`1e-9 m`; the largest rigidity residual is `4.44e-16 m`.

### 3.2 Integration and derivative evidence

- `M4` per analytic piece: `16.0367717867`
- panels per piece: `300` (width `0.02 m`), total `600`
- Simpson Euclidean remainder: `1.5170e-8 m`
- floating/angle guard: `6.2000e-8 m`
- reported position budget: `7.7170e-8 m` (<= `1e-7`)
- reported angle budget: `1.00008e-10 rad` (<= `1e-9`)
- worst production position difference from quadrature on 201 stations:
  `9.00e-10 m`, below the reported budget.

Five-point differentiation of the independent quadrature positions at 121
interior stations (saved separately for each step) gives:

| `h` [m] | max heading error [rad] | max curvature error [1/m] | production position error [m] |
|---|---|---|---|
| `0.001` | `1.146e-10` | `3.690e-9` | `9.01e-10` |
| `0.0005` | `1.417e-11` | `1.314e-8` | `9.01e-10` |

The smaller step increases second-difference round-off, so the two rows are
reported separately and are not presented as a convergence assertion.  Both
are far inside the required `1e-6 rad` and `1e-5 1/m`.  The differential
positions come from closed-form `psi` plus quadrature, never from the
production position record.

### 3.3 Node, mirror, wheel and negative evidence

- At `s = 0, L/2, L, 3L/2, 2L` the benchmark records explicit left/right/auto
  jets and all three vehicle states.  `k/eta/u` are continuous; the middle
  `g` jump above is visible and `side='auto'` uses the right limit at internal
  nodes and the left limit at the terminal station.  Domain ends reject the
  non-existent side with `SIDE_DOMAIN`.
- The mandatory non-C2 fixture `(s,k,eta) = [(0,0,0),(1,.1,0),(2,.3,0)]` is
  rejected as `NON_C2_PROFILE` at `node=1, s=1.0` with `u_left=-0.6`,
  `u_right=1.2` (1/m^2).  A separate synthetic C2 profile with `g` jumping
  from `0.6` to `-1.8` at `s=1` is accepted, proving a `g` jump alone does not
  trigger `NON_C2_PROFILE`.
- Mirror mapping: formation centre `x`/`psi`, vehicle centres, headings,
  curvature and `p_s/p_ss/p_sss` mirror with the `2<->3` swap; checked at six
  stations within `1e-12`.
- Constant-curvature cross-check against the legacy `a_arc_edge` at
  `k=0.2` with cargo pivot `(0,5)` reproduces qf, all centres, headings and
  per-vehicle curvature (positions within `1e-6 m`, angles/curvatures within
  `1e-9`).
- Wheel diagnostic at `w=0.5 m/s`: the legacy `wheel_configuration` helper and
  the independent `m=1-yC`, `n=xC` formula agree to `0.0` angle and factor;
  `v_i = Q_i w`, `omega_i = H_i w` were checked as diagnostics only, not as a
  dynamic or hardware certification.
- Short negative case `L=0.5 m`: first failure is
  `CURVATURE_LIMIT, vehicle_id=1, s=0.02755,
  curvature=1.2364529650 > limit=1.2345679012`; the front peak is
  `4.3482216207`, side peaks are `0.4285714286`.  No state was clipped and no
  early failure was hidden.

All required structured rejections are covered by tests with the exact
reason and location: out-of-domain `s` (`S_DOMAIN`, never clipped), NaN/Inf,
zero-height duplicates (`NON_POSITIVE_SEGMENT`), decreasing knots
(`INVALID_KNOT_ORDER`), NaN knots, non-C2 node, invalid side, wrong formation
offsets/IDs (`FORMATION_MISMATCH`), `A=0` normal-branch violation, real
heading mismatch (0.1 rad on car 1), non-stationary/non-A/non-straight
declarations, non-zero initial curvature, unknown initial source and missing
ROS `context_id`.

Quadratic (`k=0.01 s^2`) and cubic (`k=0.02 s^2-0.001 s^3`) production
profiles are both integrated over 201 stations and compared with independent
quadrature; the actual error stays below each profile's reported budget.

## 4. Commands and evidence index

Exact commands, environment, `cwd` and exit codes are in `COMMANDS.md`;
`environment.json` records the interpreter and package versions.  Evidence:

- `logs/geometry.log`, `logs/profile.log`, `logs/benchmark.log`,
  `logs/legacy_geometry.log`, `logs/setup_registration.log`,
  `logs/git_state.log`
- `benchmark/summary.json` (23 checks and full node/mirror/differential data),
  `benchmark/samples.json`, `benchmark/double_lane_change.csv`,
  `benchmark/mirror.csv`
- `benchmark/figures/{double_lane_change,headings,curvature,integration_error}.png`
- `legacy/` (G0 runner outputs, `protected_before.json`,
  `protected_after.json`, regression plan/results)
- `protection_diff_vs_g0.json`, `results.json`, `source_manifest.json`
- `logs/source_manifest_verification.log`

The G0 runner's printed `"stage": "G0"` is a static field inside the already
accepted G0 tool; the geometry authorization block in
`legacy/regression_plan.json` is `G1` and exactly matches the seven additions
plus `setup.py`.  The G0 tool was not modified in G1 (it is outside the G1
whitelist).

## 5. Limitations and non-claims

- The analytic profile is a diagnostic; it is not a serialisable planner input
  and was not fed to any optimiser.
- Outward interval rounding relies on IEEE-754 successor/predecessor operations
  and a documented 16-ulp coefficient widening; this is a numerical design
  assumption, not a formal floating-point proof.
- The quadrature oracle is an independent numerical method with
  `1e-13` tolerances, not a symbolic proof; derivative recovery additionally
  carries finite-difference round-off documented above.
- No ROS/Gazebo execution, hardware calibration, timing profile, NLP solve or
  FR06 autonomous search was run.  Those remain `NOT_RUN` for later stages.
- The G0 acceptance follow-up `tests/fixtures/virtual_a/open_requests.json` is
  due before G3 and was intentionally not created in G1.
- No claim of `GEOMETRY_FEASIBLE` or `EXECUTABLE` is made for an experiment.
- **Stop point:** G1 is complete; this worktree is left uncommitted for Codex
  independent acceptance.  G2 must not start without the user's next-stage
  instruction.