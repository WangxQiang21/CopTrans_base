# TASK03 Step 4 evidence — independent continuous F+all-OBB certificate

UTC window: 2026-11-19T13:33Z–15:49Z (this directory; the final rerun at
15:45Z–15:49Z includes the Portal-cut test).
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`.
HEAD unchanged: `a3c709e57560b1060dcc422777f4131a3e558acf`.

Step 4 adds the independent continuous certificate over the full F domain
and all three complete OBBs:

1. outward-rounded interval arithmetic on every fixed piece between the
   required cuts (Hermite pieces, template/Portal segments, allocation
   boundaries, `[0, L]`);
2. two-sided concrete checks at every required cut;
3. a deterministic bisection with hard depth/width/interval/time limits;
4. an optimizer gate: only `VERIFIED` with a matching identity and canonical
   self-hash may reach the historical geometry audit, `best`, and
   `GEOMETRY_FEASIBLE`.

`VIOLATED` carries one concrete positive-evidence witness.  Everything that
does not close (`UNCONFIRMED`, `TIME_LIMIT`, `INTERVAL_LIMIT`, A/D
degeneration, non-finite interval data, stale identity) never publishes.

## Production files

| File | SHA-256 at end of Step 4 |
|---|---|
| `formation_continuous_a/contracts.py` | `600b17f76814ceafcd6102b27409814e5c341ae6513ae694c38a817fda856b77` |
| `formation_continuous_a/optimizer.py` | `c667388dd578dd435afbff4d785f48510ef3460d913b34672bfa7356f4296a2d` |
| `formation_continuous_a/formation_certificate.py` | `27557c391df2dcd323fe20c6f5c829981a1c282e67a8a5d71dc46cf5d81ebf42` |
| `tests/test_virtual_a_formation_corridor.py` | `8ffa807ad3c5844bfd6eca4fc6b8ffe141bdfcd171daed6aefd7cd2ba6367078` |

`corridor.py` and `formation_corridor.py` are unchanged from Step 2/3.  The
protected TASK01 dirty files remain byte-identical:

- `formation_continuous_a/geometry.py` `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
- `tests/test_virtual_a_geometry.py` `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`

See `source_hashes_end_step4.txt`, `git_status_end_step4.txt` and
`git_diff_stat_end_step4.txt`.

## Certificate semantics

Schema `virtual_a_formation_continuous_certificate_v1`, version
`virtual_a_formation_certificate.v1`.  The payload records the sealed
identity (request/grid/chain/model/context/template/allocation/guide/path)
both at top level and in an `identity` record, the exact limits actually
used, interval and cut counts, the maximum outward upper residual, at most
16 unclosed intervals with their reasons, all two-sided cut checks, the
complete vehicle coverage, a possible witness, and the old F-only comparison
(`virtual_a_f_only_corridor_audit_comparison`, never the gate).

The canonical SHA-256 excludes only the diagnostic `elapsed_s`, so identical
inputs produce an identical `certificate_sha256`.  `verify_formation_certificate`
fail-closes on a missing/non-mapping certificate, schema mismatch, body hash
mismatch, stale identity, non-`VERIFIED` status, a positive/non-finite
`max_outward_residual_m`, a non-zero unclosed count, a witness, inconsistent
interval counts, malformed cut records, or any cut not `CLOSED`.

### Interval arithmetic

For each closed sub-interval the certificate uses:

- `position_x/y = qf(a) ± (2 * position_error + width)` and
  `psi = qf.psi + kappa_interval * width ± 2 * angle_error`;
- `A = 1 - b * kappa`, `lateral = a * kappa`, `D = A^2 + lateral^2`;
- `phi = atan(lateral / A)` as a monotone interval, `theta = psi + phi`,
  and `cos/sin` via the angle's Lipschitz bound (clipped to `[-1, 1]`);
- every complete-OBB piece corner against every world halfspace with the
  template margin already subtracted.

`A_lo <= 0` or `D_lo <= 0` is `DEGENERATE`; non-finite kappa or a failed
state evaluation is `NONFINITE`; both are recorded unclosed and can only
yield `UNCONFIRMED`.  Only every closed interval upper `<= 0` plus every
cut `CLOSED` yields `VERIFIED`.

## Measured evidence (`step4_metrics.json`)

| Scenario | Result |
|---|---|
| Wide one-region scene | `VERIFIED`, 8/8 intervals, depth 0, max outward `-42.63098999720203 m`, 16 cuts, sha `e6c309797e0c42e495c1e0b07e64030402ef902352b64a06634de2673e639903` (identical on rerun) |
| Tight clear wall (`fine_max + 1e-3 m`) | `VERIFIED`, 1148 checked / 578 verified intervals, depth 11, max outward `-8.518289691394897e-7 m`, 16 cuts |
| E08: collocation max `4.824499611397091`, fine max `4.825172613203202`, wall `4.824836112300146` | `VIOLATED`, witness vehicle 3 at sigma `0.7777676582336426`, `residual=2.7779e-8`, `evidence=2.5717e-8 > 1e-9`; the old F-only comparison still passes with `max_bound_residual=-0.896 m` |
| Split template at sigma 0.4 | `VERIFIED`, 9/9 intervals, 18 cuts, both sides of `s=2.4 m` and every Hermite/allocation cut `CLOSED` |
| Real Portal template on a mismatched straight path | 1 template, segment boundaries `0.791083954 / 1.525665064 / 3.283694358 m`, both sides checked at each, 22 cuts; `VIOLATED DEFINITE_CUT_VIOLATION` at `3.283694358 m`, witness vehicle 2 in region 0, `evidence=0.006088738 > 1e-9` |
| `max_depth=0` | `UNCONFIRMED INTERVAL_NOT_CLOSED`, 4 unclosed intervals, no witness |
| `max_intervals=1` | `UNCONFIRMED INTERVAL_LIMIT`, 1 interval checked, no witness |
| `max_seconds=0` | `UNCONFIRMED TIME_LIMIT`, `timed_out=True`, 0 intervals, no witness |
| Normal-branch degeneration (`kappa=1.0`, `max_depth=4`) | `UNCONFIRMED INTERVAL_NOT_CLOSED`, 128 degenerate intervals, `A=0.0 D=0.0` for vehicle 2 |
| NaN kappa interval | `UNCONFIRMED INTERVAL_NOT_CLOSED`, 8 non-finite intervals, no witness |
| Tamper: path / request / grid / context / template / body / status / schema | identity mismatch, identity unavailable, template-id mismatch, `CERTIFICATE_HASH_MISMATCH`, `CERTIFICATE_SCHEMA_MISMATCH`; all fail closed |
| Tamper: only `elapsed_s` | still verifies; `elapsed_s` is diagnostic and excluded from the canonical hash |
| Signed malformed payload | `CERTIFICATE_COUNTS_UNAVAILABLE`, `CERTIFICATE_CUTS_MISSING`, `CERTIFICATE_CUT_NOT_CLOSED` |

## Optimizer gate (`optimizer_gate`)

The gate runs after the Step 3 refinement and before `audit_geometry`:

| Input | Result |
|---|---|
| Real certificate on a solved candidate (`_z(0.0)`) | `GEOMETRY_FEASIBLE`, attempt `SOLVER_SUCCESS_AUDITED`, certificate `VERIFIED` in the result and `as_dict()` |
| Mock `UNCONFIRMED` | `NO_ACCEPTED_CANDIDATE` / `FORMATION_CERTIFICATE_UNCONFIRMED`; audit not called; no path |
| Mock `VERIFIED` but verification fails | `NO_ACCEPTED_CANDIDATE` / `FORMATION_CERTIFICATE_VERIFICATION_FAILED`; audit not called |
| Mock `TIME_LIMIT` (`timed_out=True`) | `BUDGET_EXHAUSTED` / `SOFT_BUDGET_EXHAUSTED`; audit not called |

The f_only result key set is asserted exactly and still contains no
`formation_certificate` key; union results carry it.

## Test table

| Command | Result |
|---|---|
| `python3 -m py_compile` (contracts, optimizer, certificate, tests) | exit 0 |
| `python3 -m unittest test_virtual_a_formation_corridor` | 50 tests OK, 38.127 s (16 Step-4 certificate/gate tests) |
| `python3 -m unittest test_virtual_a_formation_assessment` | 40 tests OK, 0.747 s |
| `python3 -m unittest test_virtual_a_corridor test_virtual_a_corridor_optimizer` | 52 tests OK, 9.967 s |
| `python3 -m unittest test_virtual_a_optimizer` | 37 tests OK, 67.745 s |
| `python3 -m unittest test_virtual_a_geometry` | 24 tests OK, 1.113 s |
| `python3 -m unittest test_virtual_a_pipeline` | 13 tests OK, 121.189 s |
| `python3 -m unittest test_virtual_a_audit_evaluation_guard` | 4 tests OK, 0.628 s |
| `python3 -m unittest test_virtual_a_collision` | 17 tests OK, 2.833 s |
| `python3 -m unittest test_virtual_a_audit test_virtual_a_integration_bounds` | 73 tests OK, 6.051 s |
| `python3 collect_step4_metrics.py` | exit 0, 2.72 s |

All exit codes are in `test_exit_codes.txt`.

## Still open

Step 4 gates the optimizer only.  The explicit union-scope wiring through
launch → settings → pipeline → `g` → certificate → result → playback
verifier, the HUD/RViz namespaces, the real ROS run, the soft-clearance
ablation (Step 5), the E04 negative and the f_only regression (Step 6) are
not in this directory.  The default scope in `run_demo.sh` is unchanged and
must not switch before Codex acceptance.