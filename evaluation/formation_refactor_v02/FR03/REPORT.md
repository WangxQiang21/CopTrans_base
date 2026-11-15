# FR03 report: finite-label states, successors and proxy cost

Status: **complete, stopped for independent Codex acceptance**.
No Git commit, no FR04, no ROS/RViz, no messages, no NLP, no control or
dynamics work was performed.

## 1. Scope delivered

`uniform_search.py` implements exactly the FR03 contract of
`FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md` section 7 and
`FORMATION_REFACTOR_DECISIONS.md` section 7:

* **Exact state** `UniformSearchState`: `q_F` (x, y, psi), exact unwrapped
  body headings, current mode, previous canonical wheel/primitive signature,
  cumulative `g`, parent and incoming edge.  Nothing is snapped.
* **Finite-label key** `UniformStateKey`: XY bin 0.20 m, `psi` bin 10 deg,
  mode, per-vehicle wheel label (`mode`, signed direction and control bin
  0.01), per-vehicle heading label (10 deg bin plus direction sign).  It is a
  deduplication label only.  `RepresentativeTable` keeps the lower-`g` exact
  state per complete key; this is a bounded discrete approximation, **not** a
  strict dominance certificate over arbitrary exact states in one bin.
* **Successors**: single authoritative FR01 edges derived only through the
  FR01 factories.  A fixed library of pivots `c=(0,0),(0,+-2),(+-2,0)` with a
  10 deg step, plus the DECISIONS-permitted goal-cropped angle (the exact
  remaining cargo-heading difference; the fixed step is never changed).  O
  offers eight cargo-frame directions plus the goal direction; A candidates
  are generated only for already-tangent headings and otherwise produce an
  explicit S alignment successor.
* **Legality**: every candidate is checked with the real analytic-pose OBB
  sweep (`FormationCollisionChecker.swept_check_analytic` +
  `AnalyticSweepSegment`, exact `edge.state_at` per fraction, geometric
  per-fraction bounds, policy spacing 0.01 m / 200000 samples).  Full S8/S9
  is **not** run per expansion.
* **Seconds-dimension cost**: actual common FR02 setup demand
  (`setup_demand_s`, mirroring `_make_setup`) plus a first-order motion proxy
  `max(max centre travel / v_max, max heading change / yaw_rate_max)`.  It is
  explicitly not the S8 `T_end`, not admissible and not an optimality claim.

Not implemented (out of scope): whole-map graph search/open-closed/priority
queue (FR04), goal discovery experiments, ROS, NLP, control and dynamics.

## 2. Tests

`tests/test_uniform_search.py`, 10 cases, all passing:
`Ran 10 tests in 128.841s - OK`.  Evidence is written to
`successors.json`, `state_key_samples.json`, `arc_step_audit.json`,
`successor_fr02_audit.json` and `measure_200_states.json`.

### S01 exact state vs finite key

Four states share the same `q_F`; the four keys are pairwise distinct when
headings, mode (A vs S) or wheel/primitive signature differ.  The key bins are
integers while the representative keeps the literal pose
`(0.437, -0.313, 0.213)`; offering a worse `g=5` clone of the same key is
rejected and the `g=0` exact state remains representative.  A state with a
different heading label is kept as a second representative (2 entries).

### S02 tangent and S-before-A

With goal `(1.0, 0.6, 0)`, the direct A straight is not emitted: 88 candidate
schemes are rejected with `A_HEADING_REQUIRES_S_ALIGNMENT`.  The generated S
alignment edge reaches the tangent heading set exactly (setup 2.3432 s), and
from its successor the A straight is accepted.  The two-edge plan is audited
FEASIBLE by FR02, `T = 10.0312 s`.

### S03 same goal, forward and reverse

Goal `(1.0, 0, 0)`.  Forward A (gear `+++`) reaches
`[1.0, 0.0, 0.0]` and audits FEASIBLE, `T = 3.1228 s`.  Reverse A (gear
`---`) after an S alignment of pi reaches the **same** goal
`[1.0, 0.0, 0.0]` and audits FEASIBLE, `T = 15.2091 s` (the longer duration is
the S alignment, not a different endpoint).  `allow_reverse` is respected.

### S04 nonzero pivot, both directions, and the fixed-step audit limit

Pivot `(0, 2)`, cropped angle 90 deg:

| direction | FR02 result | duration | continuity |
| --- | --- | ---: | ---: |
| +90 deg | FEASIBLE | 18.585 s | 0.0 |
| -90 deg | FEASIBLE | 23.8735 s | 0.0 |

The fixed +-10 deg library also generates both signs.  Under the accepted
default limits their S8 duration is jerk-limited and the **conservative
analytic full-interval bound** cannot certify the FR02 dynamic audit:

* audit result: `REJECTED`, code `UNIFORM_DYNAMIC_UNCONFIRMED`, constraint
  `jerk`, vehicle 3;
* candidate peak 1.97029751862 (limit 2), adopted analytic bound
  2.16302284782 (> limit), refined bound `None`, max adjacent gap
  0.00314617372991 s;
* exact continuous model peaks over 20001 samples (the plan includes the S
  alignment): speed 0.390056 / 0.8, acceleration 0.477297 / 0.6, jerk
  1.970298 / 2.0 — all within limits;
* plan durations 10.0021 s (+10 deg) and 15.2906 s (-10 deg).

This is recorded as an audit-layer conservative-bound limitation, not a
demonstrated physical exceedance, and it is **not** worked around by changing
limits, step size, or the audit.  A certifiable nonzero-pivot rotation path
exists through the DECISIONS-permitted goal-cropped angle (>=90 deg shown
here; earlier probes found pivot `(0,2)` >=35 deg and pivot `(2,0)` >=60 deg
certifiable).

### S05 mid-sweep and mid-spin obstacles

* O translation 0.8 m with a 3x3 obstacle block on the middle of vehicle 1's
  centre path: both endpoint poses are statically clear, the analytic sweep
  fails `MAP_OBB_COLLISION` at station 0.15 (vehicle 1), and the generator
  rejects the same candidate.
* S spin by pi with an obstacle outside both endpoint rectangles: endpoints
  clear, sweep fails `MAP_OBB_COLLISION` at station 0.15894 (vehicle 1).

Neither negative case relies on a colliding endpoint.

### S06 wall edge: usable vs unusable angle

Vertical wall at `x = 2.3`, pivot `(0, 2)`, start clear, same start pose:

* +90 deg: sweep fails `MAP_OBB_COLLISION`, vehicle 1, station 0.31810;
* -90 deg: sweep passes.

The generator reproduces the same distinction: after S alignment the +90 deg
cropped arc is absent from the successors and appears in the rejections.

### S07 free terminal heading

With `goal_heading_free=True`, a state at the goal `q_F` with arbitrary body
headings is a goal; the placeholder goal heading is not used to exclude it.
With `goal_heading_free=False`, those headings are rejected and the matching
headings are accepted.

### S08 every legal successor class through FR02

| class | FR02 result | duration | continuity |
| --- | --- | ---: | ---: |
| A straight (after S) | FEASIBLE | 10.0312 s | 0.0 |
| O translation | FEASIBLE | 4.715 s | 0.0 |
| S alignment | FEASIBLE | 4.4546 s | 0.0 |
| cropped A arc (after S) | FEASIBLE | 18.585 s | 0.0 |

Cost checks: `cost_s == setup_s + motion_proxy_s` exactly; the S edge's setup
demand 2.3432099798544908 s equals the FR02 plan's `initial_setup` phase
duration to 1e-9; the proxy is demonstrably not the S8 duration (1.25 s vs
7.688 s FR02 motion for the straight-after-S plan); a no-wheel-change A
straight from the initial A configuration has zero setup demand, while the
S-to-A transition is charged 2.3432 s instead of being hidden.

### S09 fixed-seed 200-state measurement

Seed 20261114, goal `(5, 3, 0.4)`, empty grid, 200 exact states obtained by a
seeded random walk (200 walk states, 0 fallbacks; start `[0,0,0]`, final
sample `[7.3021, 0.7815, 0.4]`).

| metric | value |
| --- | ---: |
| candidates per state | mean 167.84, median 184, min 144, max 185 |
| candidates total | 33567 |
| accepted successors | 11663 (mean 58.3 per state) |
| rejections | 21904, all `A_HEADING_REQUIRES_S_ALIGNMENT` |
| generation time | 9.309 s total, 46.5 ms/state |
| collision time | 107.802 s total, 0.539 s/state |
| peak memory (`tracemalloc`) | 3.51 MB |

The rejection histogram is candidate-level on an empty map; no obstacle
rejection occurs there by construction (obstacle behaviour is S05/S06).  This
is a local per-state successor measurement, not a planning-success-rate
experiment and not a global performance claim.  The same seed reproduces the
same state sequence; the file records every state, key, count and timing.

### S10 no full S8/S9 per expansion

`uniform_timing.schedule_uniform_edges` and
`uniform_quality.audit_uniform_timed_plan` were patched to raise during one
full expansion (185 candidates, 57 accepted): zero calls were observed.  The
FR02 audit is exercised only by the separate acceptance cases above.

## 3. Regression and preservation

Ten targeted suites, `Ran 145 tests ... OK`, exit code 0 (9.7 s):
`uniform_motion`, `uniform_timing`, `uniform_trajectory`, `uniform_quality`,
`formation_collision`, `formation_quality`, `formation_retiming`,
`formation_geometry`, `formation_hold`, `formation_kinematics`.

All ten independently accepted FR01/FR02 source/test files and the three
frozen documents still match their Codex-recorded SHA-256 hashes
(`baseline_verification.json`, `all_accepted_match=true`).  FR03 added only
`uniform_search.py` and `tests/test_uniform_search.py`; no old test
expectation, limit, geometry, safety threshold, prior experiment or Codex
evidence was modified.

## 4. Known limitations to review

1. The fixed +-10 deg A arc cannot be dynamically certified by the current
   conservative S8/S9 boundary under default limits (S04).  This is an
   FR02 audit-layer property, not a success/failure of FR03; the search
   fail-closes and records the rejection.
2. The proxy cost is first-order and not admissible; it must not be used as
   an optimality or feasibility certificate in FR04.
3. Same-key selection is a bounded discrete approximation; it can discard a
   state with a different exact pose inside the same bin.
4. The 200-state timing was measured on an empty grid; obstacle-rich maps
   increase the sweep cost (bounded by the fixed sweep policy).
5. The goal-cropped arc is offered for every pivot and gear branch (subject to
   FR01 heading legality); FR04 may want to prune it, but FR03 does not.