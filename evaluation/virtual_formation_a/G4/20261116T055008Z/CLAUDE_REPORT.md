# G4 report — A-only virtual rigid formation, real maps

Run ID: `20261116T055008Z`
Stage: **G4**
Status: **PASS_PENDING_CODEX_RECHECK**
Codex acceptance: **NOT_RUN_NOT_CLAIMED**

Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
Branch: `feature/virtual-rigid-formation-a`
HEAD/ancestor: `567bdecba9676313da3bc9a97b82f57b5b569a2b` (unchanged)
Git: no branch, commit, push, reset or stash was created.

This delivery proves, on six frozen real-map A-only scenes, that the
production pipeline autonomously finds complete plans passing the independent
geometry, common-time dynamics and conservative swept-collision audits while
reading only `map.pgm`, `map.yaml` and `request.json`. It does not claim
general completeness, optimality, a success rate, real-time behavior, ROS or
hardware validation, and it does not claim Codex acceptance.

## 1. G4-A — scene construction, reference audit and freeze

The six scenes are `S1`, `S1_mirror`, `S2`, `S2_mirror`, `S3`, `S3_mirror`.
`S*_mirror` is the y-mirrored map with formed-vehicle IDs preserved and
per-vehicle offsets placed on the mirrored geometric sides; requests and
references were rebuilt, not copied.

Frozen manifest: [scenes/scene_manifest.json](scenes/scene_manifest.json)
(file SHA-256 `3cd821dfe78bf398c2fa84f855f3fb1a628f7c2e1014fc141c9adeb2cfd54516`,
self digest `e747868a01e505e82e6c4c0758b7ceb6951bd5f2ef2c6db1b2c26f474ec75c74`).
It records `frozen: true`, `reference_verified: true`,
`inputs_have_no_reference: true`, `failures: []`, base commit, resolution,
planner/timing/collision/dynamic configs, and per-scene map, request, planning
context, timed-model and reference digests.

Each formal input directory contains exactly `map.pgm`, `map.yaml` and
`request.json`; no reference, manifest or old audit is present in it, and the
pipeline test asserts that no file is read during autonomous planning.

| scene | grid W×H | res m | origin x,y | occupied / cells | map signature |
|---|---:|---:|---|---:|---|
| S1 | 409×302 | 0.05 | -6.80, -4.80 | 102358 / 123518 | `054cf5c7…0266a10e` |
| S1_mirror | 409×302 | 0.05 | -6.80, -10.30 | 102358 / 123518 | `fb485abf…e7c595` |
| S2 | 494×213 | 0.05 | -6.60, -4.65 | 85491 / 105222 | `55e3c5b8…af8782` |
| S2_mirror | 494×213 | 0.05 | -6.60, -6.00 | 85491 / 105222 | `e50be383…b4d57c` |
| S3 | 345×123 | 0.05 | -2.40, -3.00 | 1714 / 42435 | `d4ca8897…609ba1c3` |
| S3_mirror | 345×123 | 0.05 | -2.40, -3.15 | 1714 / 42435 | `84639455…e3b741` |

The grid signature is the canonical SHA-256 of
`{height,width,resolution_m,origin_x,origin_y,occupancy_sha256,cell_count,occupied_cells}`;
occupancy bytes are bottom-left row-major, free only when the ROS-style
probability is below `free_thresh`, so unknown/gray cells are conservatively
occupied. `analysis/g4_numerics.py` reparses every PGM and YAML from raw bytes
with the standard library and reproduces all six signatures.

All six references were built, retimed and audited before the freeze, then
reverified against the authoritative map/request copies. Every
`reference_verification` record has all twelve subchecks true, and
`inputs_have_no_reference` is true. Each reference has a passing three-layer
audit and an independently accepted reference centreline (Gauss–Legendre
integration, endpoint agreement 1.28e-10 m to 2.81e-13 m). Reference timing:

| scene | T (s) | geometry checks | dynamic checks | collision checks |
|---|---:|---:|---:|---:|
| S1 / S1_mirror | 28.347912 | 155 | 909 | 8204 |
| S2 / S2_mirror | 37.539035 | 183 | 937 | 15620 |
| S3 / S3_mirror | 37.539035 | 183 | 937 | 15620 |

Reference profiles use the accepted C2 Hermite path construction with the
common G3 quintic timing; no S-prepared mixed-mode result was used. The
reference is existence evidence only and is not an autonomous result.

## 2. G4-B — conservative sweep adaptation

The collision layer reuses the unmodified old `VehicleShape`,
`FormationCollisionChecker`, `static_check` and `swept_check_analytic`. The
G4 adapter:

- emits `AnalyticSweepSegment` with `Delta = s_b - s_a`, centre-speed bound
  `Delta`, formation-rate bound `Delta·sup|k|` and per-body rate bound
  `Delta·sup|H_i|`, where `H_i = k + a_i·eta/D_i`;
- takes those sup bounds from the accepted outward-rounded interval operators,
  never from collocation maxima, and records both the declared bound and the
  independently recomputed lower bound per λ interval;
- keeps `SweepPolicy(max_corner_step_m = 0.01, max_samples_per_segment =
  200000)`, the 0.05 m baseline margin and the unchanged vehicle shape;
- records the full trace (`s`, time, `qF`, actual body headings, the coverage
  bound and sample counts), runs `static_check` at start and goal before the
  swept check, and treats budget exhaustion as a failure that can never pass.
  Every scene retains this in `metrics.sweep_trace`, its per-λ `sweep_bounds`
  and its `checker_statistics` inside `benchmark/results.json`;
- computes the per-vehicle numerical envelope
  `eps_position + (|offset| + radius_upper)·eps_psi + radius_upper·eps_relative`
  with `eps_relative = 1e-10`, using the outward-rounded fixed-point radius,
  and adds the maximum envelope to the checker margin without reducing 0.05 m.

The final margins are 0.0500000327 m to 0.0500000755 m. In every scene
`margin_used >= 0.05 + maximum_envelope` in recorded binary64 arithmetic
(smallest excess 6.94e-18 m), the independently recomputed envelope equals the
recorded value exactly, and `radius_upper` equals the circumradius at the
recorded margin. Per-λ declared bounds dominate their independent lower bounds
for all segments and all three vehicles.

`NO_MOTION` runs the complete static map and three-vehicle hold check before
anything else; the exact start==goal request is never pre-empted by route
searching, and a blocked hold is rejected as `COLLISION_FAILED` with stage
`hold`.

Route guides are finite eight-neighbour A* paths under a distance-clearance
field (`clearance_weight = 2.0`, at most three routes, at most 8 s and 200000
expansions). They are labelled `is_executable_path: false`; they carry no
profile or curvature and cannot become a plan.

## 3. Mandatory negative evidence

All cases below ran in this run and are recorded as `... ok`; the evidence
assembler fails if any named test is absent.

| # | Required negative | Test(s) observed passing |
|---|---|---|
| 1 | thin obstacle between coarse samples | `test_thin_obstacle_between_coarse_samples_is_rejected` |
| 2 | corner collision with actual front heading | `test_corner_collision_uses_actual_front_heading` |
| 3 | changed map/limits/goal or stale context | `test_same_model_changed_map_limits_and_goal_are_rejected`, `test_missing_or_stale_planning_context_digest_is_rejected` |
| 4 | omitted Δ factor caught independently | `test_missing_delta_factor_is_caught_by_independent_bounds` |
| 5 | circumcircle blocked but true OBB passes | `test_whole_formation_circumcircle_blocked_but_obb_passes`, `test_circumcircle_blocked_door_is_solved_by_real_obb_audit` |
| 6 | tiny sweep budget fails | `test_sample_budget_is_a_failure_not_a_pass` |
| 7 | boundary/unknown/start/goal/hold/solver/no-candidate/budget | nine tests listed in `results.json` under `numbered_negative_coverage` |

Case 7 includes the explicit false-success path: a mocked solver reporting
`Solve_Succeeded` with a perturbed vector cannot become executable; every
attempt is retained as `SOLVER_SUCCESS_AUDIT_REJECTED` and no plan/path is
published. The tiny total budget returns `BUDGET_EXHAUSTED` and is
non-executable. The exact hold with a blocked start cell returns
`COLLISION_FAILED` after a nonzero hold static check rather than a route
error.

## 4. G4-C — autonomous benchmark

Command: `tools/virtual_a_benchmark.py --stage G4 --scene-manifest
scenes/scene_manifest.json`. Result: `PASS`, 72/72 checks, 6/6 executable.
The pipeline consumed only the frozen input bundles. Every result is
`SUCCESS`, executable, has a timed model and all three audits passed; the
three audits bind the same request, grid and planning context, and the model
digest equals the published model JSON.

| scene | status | checks (geo/dyn/col) | endpoint m | heading rad | rigidity m | margin m | T (s) | path m | wall s |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|
| S1 | SUCCESS | 10664 (163/917/9584) | 1.60e-10 | 0 | 1.55e-15 | 0.0500000415 | 30.305 | 10.473 | 12.1 |
| S1_mirror | SUCCESS | 9886 (161/915/8810) | 1.54e-10 | 2.89e-15 | 1.11e-15 | 0.0500000327 | 29.919 | 10.467 | 12.0 |
| S2 | SUCCESS | 16322 (181/935/15206) | 4.28e-10 | 4.72e-16 | 1.33e-15 | 0.0500000755 | 37.546 | 11.985 | 12.6 |
| S2_mirror | SUCCESS | 16298 (181/935/15182) | 4.13e-10 | 2.22e-16 | 1.33e-15 | 0.0500000737 | 37.695 | 11.982 | 12.8 |
| S3 | SUCCESS | 12956 (169/923/11864) | 4.37e-10 | 2.78e-17 | 1.33e-15 | 0.0500000472 | 37.280 | 11.865 | 24.3 |
| S3_mirror | SUCCESS | 13634 (175/929/12530) | 7.20e-10 | 8.33e-17 | 1.11e-15 | 0.0500000445 | 37.276 | 11.964 | 23.7 |

Endpoint limit 0.005 m and heading limit 0.2° are met by more than six orders
of magnitude; rigidity is below 1e-6 m by nine orders. Dynamic
`whole_domain_certificate` is true in all six, with 64 leaves per scene and
three pointwise interval ratios per leaf; maximum utilization is
0.9900990099 ≤ 1 for every limit component, and the pointwise component,
jerk and `p_sss` ratios are all ≤ 1.

Swept collision: 8,810–15,206 checks per scene, 1.13–1.78 s, with every static
start/goal check passing before the sweep and every per-λ bound marked
`verified`. S3/S3_mirror are the widest scenes and took 18.7 s/18.1 s of
geometry solve. Across all six scenes the attempt record contains **48
attempts, all retained**: 46 `SOLVER_SUCCESS_AUDITED` and 2
`SOLVER_SUCCESS_AUDIT_REJECTED` (one curvature-limit guide candidate in each
of S3 and S3_mirror). S3/S3_mirror also record one collision-audit retry round
after `COLLISION_NUMERICAL_FAILURE`, then pass in the retry round with relaxed
guidance weight 0.02; this is within `MAX_COLLISION_RETRIES = 2` and is
reported, not hidden.

Layer timings are recorded per scene under `timings`
(`route_generation_s`, `geometry_solve_s`, `retime_s`,
`dynamics_audit_s`, `collision_audit_s`, `result_assembly_s`) and their sum is
at most the measured wall time. `search.budget` records a 60 s soft total
budget (not a hard real-time guarantee), a 30 s planner soft limit and 2
collision retries. No soft deadline was triggered.

## 5. Independent numerical recheck

[analysis/g4_numerics.json](analysis/g4_numerics.json) is a
standard-library-only pass over the raw benchmark and scene files. It
independently reparses all PGM/YAML maps, rebuilds occupancy and grid
signatures, checks request/model/config immutability, rechecks every reference
manifest and input bundle, recomputes model digests, envelope arithmetic,
per-λ bound containment, static stage presence, dynamic utilization and all
pointwise ratios. Status `PASS` for all six scenes. Its maximum observed
values match the benchmark: endpoint 7.20e-10 m, rigidity 1.55e-15 m, margin
0.0500000755 m, utilization 0.9900990099.

This is numerical corroboration only, not a whole-domain proof by sampling:
the continuous claims rest on the recorded interval operators, declared
bounds and outward-rounded envelope, and sampled maxima are reported as
samples.

## 6. Protection and preservation

[analysis/protection_diff_vs_g3_accepted.json](analysis/protection_diff_vs_g3_accepted.json)
compares the current protected tree with the Codex-accepted G3 tree,
reconstructed from the accepted G1 snapshot plus the authorized G2 and G3
changes and checked against the G3 manifest digest
`bad6f55441bd34ca8d164cc954170336b56df398fcb0912dae10a4588279ceea` and G3
acceptance. Result:

- additions 38, exactly the G4 whitelist (2 package modules, 4 test/tool/
  support files, 32 fixtures; the G4 task document is outside the protected
  roots and is bound as a source file instead);
- changes 5, exactly `audit.py`, `contracts.py`, `optimizer.py`,
  `trajectory.py`, `tools/virtual_a_benchmark.py`;
- removals 0;
- unchanged common tree digest equal on both sides;
- current full protected tree
  `157686a3832cb0fd94241407866e33443a3b9188483a257fd16510916e78143d`
  over 3175 files.

Old FR06 source, old maps, the old collision engine, all G1–G3 tests and
fixtures, vehicle dimensions/offsets/limits and all old expectations are
unchanged. The approved G3 evidence run remains read-only and its non-self-
referential files were rehashed during the comparison.

## 7. Pre-final corrections and historical evidence

Two implementation concerns were found and corrected before the official run:

1. `collision.py` `radius_upper_note` said the radius was evaluated at
   baseline + 1e-6 m, while the implementation used the outward-rounded
   fixed-point margin. The note text was corrected to the implemented
   arithmetic; the arithmetic itself was independently reproduced.
2. An exact `NO_MOTION` request with an occupied A* start cell returned a
   route error before the mandatory hold static map/vehicle check. Route
   generation is now skipped for exact start==goal, the hold collision audit
   always runs, and a regression test covers the case.

The earlier `evaluation/virtual_formation_a/G4/20261116T053548Z/` directory is
left unchanged as read-only historical evidence of the pre-fix state. This
report and all official results come from `20261116T055008Z` after the fixes.

## 8. Evidence map

- [scenes/scene_manifest.json](scenes/scene_manifest.json) — frozen scene and reference manifest.
- [scenes/inputs/](scenes/inputs/) — reference-free autonomous inputs (3 files per scene).
- [scenes/reference/](scenes/reference/) — six independent references and audits.
- [benchmark/summary.json](benchmark/summary.json) — 72 checks, PASS (SHA-256 `d08f28ab82620e2a13f2325b766e776c2e3759f946a2304bde2d11ddd664f86f`).
- [benchmark/results.json](benchmark/results.json) — full results, attempts, audits, traces, models (SHA-256 `db474f2bcacd7e67a90899ff72171dd80491f0cda58e95e6dbbc88d1146590d6`).
- [results.json](results.json) — delivery index, negative coverage, tests, protection, nonclaims (SHA-256 `a48b6befabcc28c7b8987af67494477fec84acf8587ac2af2ec15c1f1e035640`).
- [analysis/protection_diff_vs_g3_accepted.json](analysis/protection_diff_vs_g3_accepted.json) — protected-tree comparison.
- [analysis/g4_numerics.json](analysis/g4_numerics.json) — independent recheck (SHA-256 `faf7826c1da9cf047dd1ebbcd034b8e8930d6a6dd5b9240c925b9d0e2b97fdd4`).
- [environment.json](environment.json) — host, packages, solver, git snapshot.
- [COMMANDS.md](COMMANDS.md) — exact commands, logs, exit codes.
- `source_manifest.json` and `logs/source_manifest_verification.log` — source/evidence binding written after this report.

G1/G2/G3 regression suites: geometry 20, optimizer 26, profile 22, audit 51,
timing 18, trajectory 23, all `OK`; G1 benchmark 26, G2 124, G3 151 checks,
all `PASS` with 0 failed.

## 9. Nonclaims and stop

- Six frozen known-feasible A-only scenes only; no general success rate,
  completeness, optimality or performance claim.
- No ROS, Gazebo, hardware or mixed-mode FR06 validation; no G5 work.
- Sampled maxima are not a whole-domain proof; the continuous evidence is the
  interval bounds and envelope, not the sampled values alone.
- Nominal clearance is not written as exact grid clearance; the recorded
  0.05 m baseline plus the numerical envelope is the collision margin, and
  the checker remains conservative on boundary/corner contact.
- This status is `PASS_PENDING_CODEX_RECHECK`, never `ACCEPTED`.

停止：完成G4报告后等待Codex ACCEPTED和用户下达G5。不启动ROS、不额外做优化平台、不提交Git。