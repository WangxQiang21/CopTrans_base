# TASK03 Step 5 evidence — EDT soft-clearance ablation

UTC window: 2026-11-19T16:06Z–16:12Z (this directory).
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`.
HEAD unchanged: `a3c709e57560b1060dcc422777f4131a3e558acf`.

Step 5 uses the existing authoritative-grid EDT cache as a **soft** preference
and proves it can never change a final decision:

1. no second distance field: `pipeline._clearance_field` /
   `_cached_clearance_field` is the only clearance source, and
   `_clearance_proxy` samples it;
2. real obstacles only: the field comes from `grid.occ` and the map boundary.
   Artificial corridor edges are never walls;
3. the certificate path has no clearance parameter, so the soft weight/target
   cannot verify, violate or alter a certificate;
4. `weight=0` adds exactly no objective/NLP term (bit-for-bit unit test);
5. no default margin, target or weight was touched.

Production numerical files are unchanged from end of Step 4.  Only
`tests/test_virtual_a_corridor_optimizer.py` gained the `SoftClearanceAblationTests`
class and this collector was added.

| File | SHA-256 at end of Step 5 |
|---|---|
| `formation_continuous_a/contracts.py` | `600b17f76814ceafcd6102b27409814e5c341ae6513ae694c38a817fda856b77` |
| `formation_continuous_a/optimizer.py` | `c667388dd578dd435afbff4d785f48510ef3460d913b34672bfa7356f4296a2d` |
| `formation_continuous_a/pipeline.py` | `c697d5d6fe45e3ab3806d088a0a47a26edbab5ac2880e0f887bb44f465445e8d` |
| `formation_continuous_a/formation_certificate.py` | `27557c391df2dcd323fe20c6f5c829981a1c282e67a8a5d71dc46cf5d81ebf42` |
| `formation_continuous_a/formation_corridor.py` | `23db10a2b3d6201fd0c2029a5fab4a3425570fe54858575a5942ff78a2c8f809` |
| `formation_continuous_a/corridor.py` | `5b32983bf8b70e59de8416441efdd69af4eca422d6d46281057e37047c431210` |
| `formation_continuous_a/geometry.py` (TASK01 dirty, untouched) | `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d` |
| `tests/test_virtual_a_corridor_optimizer.py` | `8205af95e412ed58be402dd5f575a7f0990c5b992a3309cf84a5675ca7123d30` |
| `tests/test_virtual_a_geometry.py` (TASK01 dirty, untouched) | `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7` |

## Unit tests added this step

`SoftClearanceAblationTests` (3 tests, in
`tests/test_virtual_a_corridor_optimizer.py`):

| Test | Assertion |
|---|---|
| `test_zero_weight_is_the_exact_no_proxy_objective` | `_numeric_objective` with `weight=0` is **exactly** equal to the no-proxy value (no tolerance, no term added); a positive weight is strictly greater. With CasADi, the `f` and `g` functions of the proxy-None, proxy-zero and proxy-active NLPs are evaluated at two different `z` vectors: proxy-zero equals proxy-None exactly for both `f` and `g`, and proxy-active `f` is larger. |
| `test_clearance_field_has_no_corridor_wall_input` | `_cached_clearance_field` takes only `(grid, downsample)`; an occupied cell samples to `<= 1e-12`; free points on would-be corridor edges keep their true distance to occupancy/map boundary (`> 0.05 m`); no parameter name contains `corridor`. |
| `test_soft_config_never_changes_the_certificate` | Certifying the same path with weight/target `(0, 0)`, `(1, 0.05)` and `(5, 0.6)` gives the same status for the wide VERIFIED scene and the slanted VIOLATED scene, and the same `certificate_sha256` in both scenes. |

## Ablation scene (`collect_step5_metrics.py`)

Frozen straight-line request on `empty_grid()` with one thin obstacle at
`x in [2.0, 4.0], y in [1.6, 2.2]`; one union corridor region
`[-2.0, -2.5, 8.0, 2.5]`; `STRAIGHT_LINE` guide `(0,0)->(6,0)`; 8 Hermite
intervals; complete-OBB single-region template.  Every arm shares:

- request SHA `184f690b1c531f671e26907ad778fcd122eb90e67faa0102b948d6ab8a094816`
- grid SHA `9acdb54e4678c7a12e9b60562c4f62dbb5c586db5171a606aa1a6f64475c6d41`
- reference planning-context SHA `d78b3cd2bd67e60ce0b6348a8bf70c13fd4525f07d59b89841d89defaec532d7`
- template `d276bf4e1a6e99879016367ba190c7586cc00109faa12d9c90a057be95fa657b`
- corridor allocation, hard constraints, `max_initials=1`, `max_seconds=20`, solver `ipopt`.

The reference context is computed from one reference config and passed to
every arm and template.  The arms differ only in the solver's
`corridor_clearance_weight`/`target_m`; `planning_context` embeds the whole
`planner_config`, so in a real launch a changed soft weight would change the
context SHA and that would be a different, non-controlled request.  For this
controlled ablation the context is fixed on purpose and this paragraph is the
disclosure.

Field samples at body centres: `(3, 1) = 0.625 m`, `(3, 0) = 1.625 m`,
`(3, -1) = 2.0 m`; field SHA `0d70ae63b085511986bc5c28410b1bb5c3bd90757b1fdb4d689692e89fe32c27`,
downsample 4.

## Measured arms (`step5_metrics.json`)

| Arm | EDT context | weight / target | Result | Iters | Wall | Path length | max \|κ\| | Cert | min EDT body clearance |
|---|---|---|---|---|---|---|---|---|---|
| no soft context | none | 0 / 0.05 | `GEOMETRY_FEASIBLE` | 7 | 9.04 s | 6.0 m | `1.22e-15` | `VERIFIED` 8/8 | `0.296683125 m` |
| zero + default target | yes | 0 / 0.05 | `GEOMETRY_FEASIBLE` | 7 | 8.63 s | 6.0 m | `1.22e-15` | `VERIFIED` 8/8 | `0.296683125 m` |
| one + default target | yes | 1 / 0.05 | `GEOMETRY_FEASIBLE` | 7 | 8.63 s | 6.0 m | `1.53e-15` | `VERIFIED` 8/8 | `0.296683125 m` |
| zero + test target | yes | 0 / 0.6 | `GEOMETRY_FEASIBLE` | 7 | 8.71 s | 6.0 m | `1.22e-15` | `VERIFIED` 8/8 | `0.296683125 m` |
| one + test target | yes | 1 / 0.6 | `GEOMETRY_FEASIBLE` | 36 | 14.75 s | `6.0000000157 m` | `1.32e-3` | `VERIFIED` 8/8 | `0.296728492 m` |

All five arms: certificate `VERIFIED` with 8/8 intervals, depth 0, 16 two-sided
cuts, `max_outward=-0.499989997 m` (or `-0.499562620 m` for the test-target
active arm); `verify_formation_certificate` passed; geometry audit passed
(155 checks); timing `DYNAMIC_FEASIBLE` (14.203 s); independent dynamics audit
passed (909 checks); unchanged analytic collision sweep passed (3740 checks);
352 corridor halfspaces and 3888 complete-OBB rows in every arm.

Trajectory comparisons (max over 257 stations, `qf` x/y/psi):

| Pair | max position Δ | max heading Δ |
|---|---|---|
| zero-default vs no-context | `0.0 m` | `0.0 rad` |
| one-default vs zero-default | `5.23e-16 m` | `4.16e-16 rad` |
| zero-default vs zero-test-target | `0.0 m` | `0.0 rad` |
| one-test-target vs zero-test-target | `1.66e-4 m` | `1.84e-4 rad` |

The zero-default run and the no-context run return a **bit-identical profile**
(9 knots, max value/derivative delta `0.0`), which is the strongest form of
"weight 0 exactly restores no soft term".  The default weight/target arm moves
the path by `5.2e-16 m` (solver round-off scale) with the same 7 iterations:
**no measurable clearance benefit at the default setting on this scene**.

The `1 / 0.6` arm is a **test-only** probe used to make the soft term active;
it bows the path slightly (max `|κ| = 1.3e-3 m^-1`), raises iterations from 7
to 36 and wall time from 8.6 s to 14.8 s, and improves the minimum cached-EDT
body clearance from `0.296683125 m` to `0.296728492 m` (`+4.5e-5 m`).  That is
not an optimization claim and not a proposed default: the target `0.6 m` is
not reachable in this scene at the given corridor, and the default
`weight=1, target=0.05` and all margins are unchanged
(`target_or_weight_relaxed: false`).

## Certificate vs soft config

- Same accepted path re-certified with `(weight 0, target 0.05)` and
  `(weight 5, target 0.6)` at equal explicit limits: both `VERIFIED` with the
  **same** `certificate_sha256` `0f101dfb33226f40...`; the certificate has no
  soft term or weight field at all.
- Caveat recorded, not changed: the optimizer certifies with
  `max_seconds = min(30, remaining_budget)` and that limit is part of the
  sealed payload (`limits.max_seconds`), so per-arm certificates on the same
  path hash differently (`11.0 s … 11.4 s` in the table).  Re-certifying the
  same path at an explicit `5.0 s` limit changes the hash again
  (`6b52cdcc5f5eb1cf...`) and remains `VERIFIED`.  This is the sealed
  "limits actually used", not a soft-weight effect; the unit test and the
  equal-limit re-certification above isolate it.

## Test table

| Command | Result |
|---|---|
| `python3 -m py_compile` (production + test + collector) | exit 0 |
| `python3 -m unittest test_virtual_a_corridor_optimizer` | 30 tests OK, 12.578 s |
| `python3 -m unittest test_virtual_a_optimizer` | 37 tests OK, 67.914 s |
| `python3 -m unittest test_virtual_a_formation_corridor` | 50 tests OK, 38.213 s |
| `python3 -m unittest test_virtual_a_formation_assessment` | 40 tests OK, 0.715 s |
| `python3 -m unittest test_virtual_a_corridor` | 25 tests OK, 0.392 s |
| `python3 -m unittest test_virtual_a_geometry` | 24 tests OK, 1.142 s |
| `python3 -m unittest test_virtual_a_pipeline` | 13 tests OK, 121.250 s |
| `python3 -m unittest test_virtual_a_audit_evaluation_guard` | 4 tests OK, 0.689 s |
| `python3 -m unittest test_virtual_a_collision` | 17 tests OK, 2.790 s |
| `python3 -m unittest test_virtual_a_audit test_virtual_a_integration_bounds` | 73 tests OK, 5.831 s |
| `python3 collect_step5_metrics.py` | exit 0, 57.78 s |

All exit codes are in `test_exit_codes.txt`; unittest summaries are in the
`*.time.log` files (unittest writes to stderr).

## Self-review

- The EDT body-clearance number in the table is a **diagnostic** computed with
  `CachedClearanceField.direction_at` and a support-radius bound; the authority
  for safety remains the unchanged analytic `audit_collision` sweep (3740
  checks, all passed).  The diagnostic never feeds a decision.
- The soft arm with `target=0.6 m` is deliberately named "test target" and is
  not proposed as a default.  Defaults are asserted unchanged by the collector
  (`target_or_weight_relaxed: false`) and by the unchanged `PlannerConfig`
  defaults.
- The sealed certificate hash includes the actual `limits.max_seconds` used
  (the optimizer passes the remaining wall budget).  This is intentional
  evidence of "limits actually used" and was observed, not changed, in Step 5;
  the equal-limit re-certification isolates the soft config completely.
- No production file changed in Step 5, so there is no behavior change to
  review beyond the three new tests and the collector.

## Still open

Step 5 is evidence and tests only.  The explicit union-scope wiring through
launch → settings → pipeline → `g` → certificate → result → playback
verifier, the HUD/RViz namespaces, the catkin build, the real ROS/RViz run,
the E04 negative and the f_only regression (Step 6) are not in this
directory.  `run_demo.sh` still defaults to the historical scope and must not
switch before Codex acceptance.