# TASK03 Step 1 evidence — contract and single-convex-region OBB

UTC window: 2026-11-19T14:00Z–14:13Z (this directory).
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`.
HEAD unchanged: `a3c709e57560b1060dcc422777f4131a3e558acf`.

Step 1 implemented the versioned corridor scope, the immutable
`FormationCorridorTemplate` identity, the canonical whole/diagonal convex
decompositions, the deterministic finite row plan, and the complete-OBB corner
rows in both the numeric residual and CasADi `g`.  It does **not** claim a
continuous certificate; `is_three_vehicle_certificate` is still false and the
new result field is only `formation_obb_constraint_count` (finite appended
scalar rows).

## Production files

| File | SHA-256 at end of Step 1 |
|---|---|
| `formation_continuous_a/contracts.py` | `fea01faa2ee52baea49a697f99af2fba192bb24fb72a7271037f08e39b698014` |
| `formation_continuous_a/formation_corridor.py` (new) | `0fcb45ea2a92594501a924fc5ed1aa067eb650989647e0ba73db1dee3c86a52c` |
| `formation_continuous_a/optimizer.py` | `cb4825b55e194edfc969573402eecfad9ec0774d6680a9dad8c32b671fd85a6c` |
| `tests/test_virtual_a_formation_corridor.py` (new) | `de5d0516db8aa05589c893dfe91da38348e7a331f6e8deb549f96b79ec04fd1d` |

TASK01 dirty files were not touched and remain byte-identical:

- `formation_continuous_a/geometry.py` `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
- `tests/test_virtual_a_geometry.py` `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`

See `source_hashes_end_step1.txt` and `git_status_end_step1.txt`.

## Old identity preservation

Both real run identities still reproduce exactly after the Step 1 source
changes.  `sha256sum -c` over all frozen Step 0 inputs passes (`exit 0`).

| Run | request SHA-256 | grid SHA-256 | planning-context SHA-256 |
|---|---|---|---|
| S1 | `591b4100355ab991d3b7dd5afeb95e203343ec185bf7187d025e2662b4afdd3b` | `6e94c751a40ec94431b495fcbff002945f09c975429c493c89060f0e7e2aa2a4` | `8a545c3f4c794f9f04c57f453c454612c54ca2d3fe31e4c6b887037d2d36604b` |
| MAP05_straight | `e69d068b1cae75fa7205a7530b129f16bf731dddce705e6d2ef8042fec07aeaf` | `64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321` | `06aed480a711839483c94e42648563b53c48eb0080f3c9dfe4e16daebe9414dd` |

The historical `f_only` config payload omits `formation_corridor_scope`;
`three_vehicle_union_v1` is emitted explicitly.  Defaults, `from_dict`
round-trips and old hash inputs are covered by
`ScopeContractTests.test_default_and_corridor_payloads_keep_the_historical_hashes`.

Logs: `baseline_reproduce_s1.log`, `baseline_reproduce_map05.log`,
`baseline_frozen_inputs_sha_check.log`.

## Measured geometry and g metrics

Source: `step1_metrics.json`, produced by `collect_step1_metrics.py` on the
frozen straight-line fixture.  These are measured values, not assertions
restated.

| Metric | Measured | Step 1 bound |
|---|---|---|
| center/heading vs `map_spatial_state` (4 heading × sign-curvature cases) | `0.0 m` / `0.0 rad` | ≤ 1e-12 |
| OBB polygon vertices vs `obb_polygon` | `0.0 m` | ≤ 1e-12 |
| four-corner projection `max` vs support function | `1.776e-15 m` | ≤ 1e-10 m |
| compiled rows vs independent `map_spatial_state` recomputation | `4.619e-13 m` | ≤ 1e-8 m |
| numeric residual vs CasADi `g` | `0.0 m` | ≤ 1e-8 m |
| CasADi Jacobian vs central finite differences | max relative `1.667e-06`, max absolute `4.322e-09` | relative ≤ 1e-5; absolute for near-zero terms |
| actual sample count | `243` (81 per vehicle) | 8×9 Gauss + 9 boundaries per vehicle |
| actual OBB scalar rows | `3888` = `243 × 4 vertices × 4 halfspaces` | real appended rows only |
| rows per vehicle | `1296 / 1296 / 1296` | every vehicle present |
| formation rows in `g` | `g[274:4162]`, all finite; bounds `(-inf, 0]` | actual `g` append, not inferred |
| union-scope solve | `GEOMETRY_FEASIBLE`, IPOPT, `0.0 m` residual, audit pass | finite rows only |

The four cases are `(psi0, kappa) = (0.0, +0.1), (0.7, -0.1), (-1.1, +0.1),
(2.6, -0.1)`.  Each case produced the same row counts and per-vehicle split;
template IDs differ by case, e.g. `73b001aae99c…` for `(0.0, +0.1)`.

Degenerate A/D states (`kappa = 1, 2` for vehicle 2; `-1, -2` for vehicle 3)
are checked in `NormalBranchTests`: every affected numeric and CasADi row is
finite, numeric/symbolic agree within `1e-3`, and each affected row is
`> 0.5 × FORMATION_NORMAL_BRANCH_PENALTY`.  The row count does not change.

## Tests

`test_exit_codes.txt` records module exit codes and log mtimes.

| Command | Result |
|---|---|
| `python3 -m unittest test_virtual_a_formation_corridor` | 17 tests, OK, 14.8 s |
| `python3 -m unittest test_virtual_a_corridor_optimizer` | 27 tests, OK, 9.5 s |
| `python3 -m unittest test_virtual_a_collocation` | 6 tests, OK, 1.7 s |
| `python3 -m unittest test_virtual_a_optimizer test_virtual_a_pipeline test_virtual_a_timing` | 68 tests, OK, 189.7 s (background log copied as `regression_optimizer_pipeline_timing.log`) |

The new module includes complete-halfspace provenance per corner
(`test_each_corner_carries_every_assigned_region_halfspace`), strict version /
non-integral region-selector / margin-floor rejection
(`test_versions_margin_floor_and_region_selectors_fail_closed`), scope gating
and an actual union-scope IPOPT solve.  `py_compile` passes.

## Scope of the claim and open integration items

- The finite corner rows are a sufficient condition at sampled stations only.
  Continuous F+OBB certification, publication gating and ROS/HUD integration
  are Steps 4–6 and are not claimed here.
- The template margin is bound into the template digest and is checked to be
  no smaller than the active F corridor margin.  The production pipeline must
  still construct it with a margin no smaller than
  `CollisionConfig.baseline_margin_m` / final `margin_used_m`; that cross-module
  provenance check belongs to Step 3/4 integration.
- In endpoint-only recovery `route_guides=()`, the selected guide is `None`, so
  the template's `guide_sha256` is not cross-checked against a live guide.  The
  caller still passes the same template identity; Step 3 must preserve or
  explicitly re-validate the selected-guide identity in that path.
- No default switch: `run_demo.sh` and the launch default remain `f_only`.
  Union scope is exercised only by explicit tests at this step.