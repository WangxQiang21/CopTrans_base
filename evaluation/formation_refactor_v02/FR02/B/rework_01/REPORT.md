# FR02-B rework_01: dynamic confirmation closure and test coverage

Date: 2026-11-13. Scope: FR02-B only. No FR03, ROS/RViz, message, NLP or
control work. No Git commit.

This rework answers `../CODEX_ACCEPTANCE.md` (CHANGES_REQUESTED). The sole
algorithmic blocker was that an unconfirmed dynamic interval still returned
`FEASIBLE / feasible=True / executable=True` and was published by
`FormationPublicationGate.offer()`. Both the closure and the requested test
coverage are implemented; the ten targeted suites and the copied Codex probe
pass.

## 1. What changed

Only the two files allowed by default were modified:

```text
catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_quality.py
tests/test_uniform_quality.py
```

`formation_collision.py` and `formation_quality.py` are byte-identical to the
pre-rework snapshot (`pre_rework/`). The accepted FR02-A sources/tests and the
FR01 uniform modules remain byte-identical to `../pre_b/baseline.json`.
Complete diff: `diff/rework_01_changes_vs_pre_rework.diff`.

## 2. Dynamic confirmation closure

### 2.1 Full-interval bound

The existing 803-point S9 candidate grid (`_TAUS`, endpoints 0 and 1,
`0.5`, `0.5 ± √3/6`) is unchanged. For each motion phase, vehicle and dynamic
constraint the audit now computes

```text
candidate_peak = max_k ||q(t_k)||
h              = max adjacent physical candidate time gap
                 = max(tau_{k+1} - tau_k) * phase_duration
refined_bound  = candidate_peak + J * h / 2
adopted_bound  = min(analytic_bound, refined_bound)
confirmed      = endpoints covered AND adopted_bound / limit <= 1 + 1e-6
```

`J` is the same-phase full-interval derivative supremum:

- acceleration -> `bounds["jerk"]`, the existing analytic quintic supremum;
- speed -> acceleration; yaw rate -> yaw acceleration; yaw acceleration ->
  yaw jerk; wheel speed / lateral acceleration -> acceleration (with the
  existing wheel-factor or curvature factor);
- jerk -> no same-phase derivative (snap) bound exists, so `J` is not
  fabricated and the analytic supremum is retained.

The validity argument is `| ||q(t)|| - ||q(t_k)|| | <= ||q(t)-q(t_k)|| <=
J |t - t_k|` with the nearest candidate at most `h/2` away. The method is
recorded as `candidate_plus_derivative_lipschitz` only when the refined bound
is tighter; otherwise `analytic_supremum` is recorded. The candidate grid
alone is never called a continuous certificate.

Every accepted result now stores, per constraint and vehicle:
`candidate_peak`, `candidate_peak_tau`, `limit`, `analytic_bound`,
`analytic_bound_exceeds_limit`, `derivative_constraint`, `derivative_bound`,
`lipschitz_error`, `refined_bound`, `adopted_bound`, `adopted_bound_ratio`,
`confirmed`, `method`, plus `max_adjacent_time_gap_s`, endpoint coverage and a
per-vehicle `continuous_certificate`. Metrics contain the global
`dynamic_bound_ratio` and `dynamic_confirmation`.

### 2.2 State semantics

- If any required constraint is unconfirmed, `_check_dynamic_peaks()` fails
  with `UNIFORM_DYNAMIC_UNCONFIRMED` (status `REJECTED`, `feasible=False`,
  `executable=False`).
- `UNIFORM_DYNAMIC_UNCONFIRMED` is distinct from
  `FORMATION_RETIMING_INFEASIBLE`: the candidate peak is below the limit;
  only the conservative full-interval bound exceeds it. The detail string
  states explicitly that this is not a demonstrated physical exceedance.
- `_uniform_success()` independently refuses to return success when the
  dynamic confirmation is incomplete for any motion phase/vehicle, so a
  future regression cannot silently restore the old behaviour.
- `FormationPublicationGate` now rejects because `feasible` is false; no
  gate code was changed.

### 2.3 Measured values

Non-zero-pivot A arc, vehicle 3, acceleration:

```text
candidate peak          0.5796584150289499 m/s^2
analytic bound          0.7893281464676393 m/s^2   (exceeds limit)
J (same-phase jerk)     1.9332470672747744 m/s^3
h                       0.005299804687500357 s
refined bound           0.5847813309635694 m/s^2   (< limit)
limit                   0.6 m/s^2
adopted / limit         0.9746355516059491
method                  candidate_plus_derivative_lipschitz
confirmed               true
```

All nine legal fixtures (A straight forward/reverse, non-zero-pivot A arc,
O forward/reverse, O same-mode wheel change, S spin with a zero-angle vehicle,
S-A-S-O complex, zero-motion hold) return `FEASIBLE` with
`dynamic_continuous_certificate=true` and no limitations. The maximum adopted
bound ratio is `0.9950248756218907` (A straight, B01).

Negative fixture: the same arc retimed by S8 with `jerk_max=1.5` (no limit,
geometry, timing model or test expectation was loosened; this is a genuine
S8 retiming at a tighter user limit) gives:

```text
candidate jerk peak     1.4170141912785232 m/s^3  (< 1.5)
analytic/adopted bound  1.9332470672747744 m/s^3  (> 1.5)
adopted / limit         1.2888313781831828
result                  REJECTED / UNIFORM_DYNAMIC_UNCONFIRMED
failure                 vehicle 3, constraint jerk, block 0
gate                    publish=False, reason=UNIFORM_DYNAMIC_UNCONFIRMED
```

## 3. Test coverage added

B01--B09 now contain 9 tests and 55 recorded subcases.

- `assert_legal()` (used by every complete-success test) asserts
  `dynamic_continuous_certificate`, empty limitations,
  `all_constraints_confirmed`, both endpoint markers, positive `h`, adopted
  bound within tolerance, candidate peak <= adopted bound, a valid method,
  and per-vehicle `continuous_certificate`/`confirmed`.
- B06 keeps the straight and A-arc mid-sweep collisions and adds an S-spin
  case: the start and goal formations are clear; one occupied cell
  `(row=209, col=221)` at resolution `0.05 m`, origin `(-10, -10)` is hit
  mid-sweep at `t=4.088966537921189 s`, station
  `0.47183098591549294`, vehicle 1, reason `FORMATION_SWEEP_COLLISION`.
- B07 keeps the three setup-phase common times (`0.137`, `0.419`, `0.777 s`,
  all `initial_setup`) and adds one common off-grid time inside every motion
  phase (`t0 + 0.371 * duration`): S `3.247646120824238 s`, A
  `9.384540677201464 s`, S `15.642577846647042 s`, O
  `21.301508521078738 s`. Both 0.02 s and 0.05 s projections are
  bit-identical to each other and to the analytic state at all seven times;
  each motion time asserts `phase_kind == "motion"` and the mode sequence is
  `S, A, S, O`. The chord-interpolation rejection is retained.
- B09 asserts the confirmation marker and the Lipschitz arithmetic on the
  positive arc, asserts every motion vehicle/constraint is confirmed in the
  complex plan, and proves the unconfirmed plan cannot enter the success or
  publication path (`REJECTED`, not feasible/executable, not
  `FORMATION_RETIMING_INFEASIBLE`, gate `publish=False`).

## 4. Reuse statement (actual calls)

- S8 timing: plans are built with the existing `UniformTimedPlan.from_edges`,
  `MotionTiming`/`SetupTiming` and `timing_motion_limits`; the audit consumes
  the existing `_analyze_block` motion data and motion limits. No second time
  planner was added.
- S9 dynamic core: the audit calls the existing `_dynamic_values`,
  `_TAUS` and `_DYNAMIC_LIMITS`. The new Lipschitz term reuses the existing
  analytic derivative supremum from `_dynamic_bounds`; it does not add a new
  interval/solver platform.
- Collision: B06 uses the existing `FormationCollisionChecker.static_check`
  and the analytic swept core (`swept_check_analytic`) through the B-era
  callback; no collision algorithm was rewritten in this rework.
- Limitation, stated explicitly: the B-added
  `_check_block_kinematics(..., uniform_v02=...)` extension is still not
  called by the new `_UniformAuditor` chain. It is not claimed as a validated
  new-chain capability.

## 5. Verification

All commands ran from `/home/wq/whut_thesis/expv3_multi_v02` with
`OPENBLAS_NUM_THREADS=1`. Exact command lines, logs and exit codes are in
`COMMANDS.md` and `command_manifest.json`.

| check | result |
|---|---|
| FR02-B suite B01--B09 | exit 0, 9 tests OK |
| ten targeted suites | exit 0, 145 tests total |
| copied Codex probe | exit 0, no unexpected unverified FEASIBLE |
| py_compile | exit 0 |
| old FR02-B evidence | 206/206 hashes still match |

The probe (run from a copy in `codex_probe_rerun/`) now reports straight,
non-zero-pivot arc and complex plans as `FEASIBLE` with
`dynamic_continuous_certificate=true` and
`unexpected_unverified_feasible=false`; the S-spin collision and all four
motion-time projection checks still pass.

## 6. Evidence and limits

Evidence root: `evaluation/formation_refactor_v02/FR02/B/rework_01/`.

- `uniform_quality_results.json` -- full B01--B09 machine results.
- `collision_map_b06.json` -- all three B06 maps and first collisions.
- `models/` -- authoritative plan inputs, including B09's unconfirmed plan.
- `tampers/` -- before/after payloads for every tamper.
- `regression/` -- ten suite logs, exit codes and ancillary results.
- `codex_probe_rerun/` -- copied probe source, stdout and exit code.
- `pre_rework/` -- pre-rework source/test copies and hashes.
- `diff/` -- complete diff and stat against `pre_rework/`.

The old `FR02/B/` evidence, including the Codex acceptance directory, was not
overwritten. No limits, thresholds, vehicle geometry/offsets, FR01 files, old
tests or frozen plans were changed. The candidate grid by itself is still not
a continuous certificate; confirmation depends on the full-interval
derivative bound. For jerk no same-phase derivative supremum is available, so
the analytic quintic bound remains the only conservative fallback.

Status: rework complete; stopped for Codex independent acceptance. No Git
commit.