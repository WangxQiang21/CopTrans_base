# TASK03 Step 2 evidence — finite whole/diagonal Portal templates

UTC window: 2026-11-19T14:30Z–14:39Z (this directory).
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`.
HEAD unchanged: `a3c709e57560b1060dcc422777f4131a3e558acf`.

Step 2 builds the bounded finite template candidates that Step 3 may consume
for one selected guide.  For each vehicle it tries one whole-OBB region
assignment plus the two canonical rectangle diagonals (each triangle assigned
to one of an adjacent region pair); no integer variables are introduced.  At
most three deterministic variants are emitted per selected guide.  The builder
uses the same diagnostic assessment candidate state (request/grid/context/
chain/guide/allocation), rechecks every cover against the actual OBB geometry,
and raises `COVER_TEMPLATE_INSUFFICIENT` when no finite cover exists.  This is
not a certificate: it is a finite row plan whose continuous behaviour is
verified separately in Step 4.

## Production files

| File | SHA-256 at end of Step 2 |
|---|---|
| `formation_continuous_a/contracts.py` | `fea01faa2ee52baea49a697f99af2fba192bb24fb72a7271037f08e39b698014` |
| `formation_continuous_a/corridor.py` | `5b32983bf8b70e59de8416441efdd69af4eca422d6d46281057e37047c431210` |
| `formation_continuous_a/formation_corridor.py` | `2bfbadf77615b84827dfc5cca3362cc1265fd69b2378ee6045b61e28c2ff049a` |
| `formation_continuous_a/optimizer.py` | `cb4825b55e194edfc969573402eecfad9ec0774d6680a9dad8c32b671fd85a6c` |
| `tests/test_virtual_a_formation_corridor.py` | `5bae92976ff10930600f2c4ba8e1320ffcaa5c343d9a06b926ede74e2c19c5d4` |

`contracts.py`, optimizer semantics and the TASK01 dirty geometry file are
unchanged from Step 1.  `corridor.py` only adds the bounded
`station_coverage` diagnostic view to the assessment payload; it changes no
allocation, winding, S-domain, margin or numeric result.  The protected
TASK01 files remain byte-identical:

- `formation_continuous_a/geometry.py` `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
- `tests/test_virtual_a_geometry.py` `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`

See `source_hashes_end_step2.txt`, `git_status_end_step2.txt` and
`git_diff_stat_end_step2.txt`.

## Finite cover semantics

`finite_cover_options(frame, center, theta, regions, candidates, margin)`
returns only covers whose every vertex lies inside every halfspace of its
assigned region with `limit - margin`:

- `whole`: all four exact rectangle corners in one convex region;
- `diagonal_0`: canonical triangles `(0,1,2)` and `(0,2,3)`;
- `diagonal_1`: canonical triangles `(1,2,3)` and `(1,3,0)`;
- region pairs are adjacent chain indices only, and the two diagonal
  assignment orders are both tested.

The finite predicate is a sufficient condition at a pose.  It is never a
general solution of the nonconvex union problem.

`build_formation_template_candidates` additionally:

- recomputes the assessment `candidate_id`, request/grid/context/chain/guide
  digests, guide length, station counts, per-station `sigma`/`s_m`, and
  rejects missing, capped, dropped, incomplete, non-clear-map or malformed
  assessments;
- requires `template margin_m >= corridor_effective_margin(chain, config)`;
- adds synthetic stations at allocation/Portal cuts, path endpoints and
  base-interval midpoints (recorded as `synthetic_station_count`);
- turns witness region changes into contiguous `sigma` segments at midpoints,
  and requires a non-empty common option set for every vehicle in every run;
- deduplicates templates by `template_id` and sorts them deterministically.

## Positive audited scene

`step2_metrics.json` records the real fixture: two overlapping rectangular
regions (`overlap_half=0.3`), an audited `CORRIDOR_CHAIN_VERIFIED` chain, and
a piecewise-linear guide whose heading places the rectangle body diagonal on
the overlap boundary.  Assessment: 47 samples, `CLEAR_AT_SAMPLES`,
`map_collision=0`, `vehicle_collision=0`, `candidate_certification=
DIAGNOSTIC_ONLY`.

| Quantity | Measured |
|---|---|
| chain audit | `CORRIDOR_CHAIN_VERIFIED` |
| assessment candidate id | `dd41b2a92f6d97423bbbbf9ea58d912533b0cd0c78ce3b7c9c582381786f1481` |
| templates | `1` |
| template id | `f8def307537555a752de8a0b55679b7cfae4cb5a3b16cb6d7539db74811676db` |
| cuts | `0.0, 0.19777098860641562, 0.3814162659269637, 0.8209235894179298, 1.0` |
| stations / synthetic | `57 / 10` |
| options per vehicle per segment (bounded) | `[[1,1,1],[1,1,1],[1,1,1],[1,1,1]]` |
| truncated options | `0` |
| samples / actual scalar rows | `312 / 4584` |
| rows per vehicle | `1536 / 1520 / 1528` |
| segment row counts | `952 / 1008 / 1728 / 896` |
| first internal cut samples | `8`, both segment indices, source `template_segment_boundary`, body regions `{0, 1}` |

The four segments use different whole/diagonal assignments, for example
vehicle 1 is `whole(0)` on `[0, 0.1978)`, `diagonal_0(1,0)` on
`[0.1978, 0.3814)`, `whole(1)` on `[0.3814, 0.8209)` and `whole(1)` on
`[0.8209, 1.0]`; vehicles 2 and 3 likewise switch between whole and diagonal
pieces.  At the first shared cut both neighbouring assignments are compiled
(8 samples at `sigma=0.19777098860641562`), so neither side of the boundary is
silently dropped.

## Insufficient finite-template scene

The same audited construction with `overlap_half=0.2` still has a verified
chain and `CLEAR_AT_SAMPLES` with zero map/vehicle collisions, but the
overlap is too narrow for any whole/diagonal cover.  The builder fails with:

```
COVER_TEMPLATE_INSUFFICIENT: station sigma=0.15726606955366723 vehicle 1
has no finite whole/diagonal cover in regions [0, 1]
```

`COVER_TEMPLATE_INSUFFICIENT` is an unknown finite-template result, not a
physical `MAP_OBB_COLLISION` or no-path claim.  Adding strips or other
templates is deferred until an E06 witness proves the need.

## Direct finite-cover checks

`step2_metrics.json` also records the E07 construction: four OBB corners lie
in one of two separated convex strips while the rectangle centre lies in the
nonconvex gap.  All four corners are in the union, the centre is not, and the
finite option set is empty.  With the rectangle diagonal aligned to a
straight artificial boundary, the only options are
`diagonal_0(1,0)` and `diagonal_1(0,1)` for the two opposing heading signs;
a rectangle crossing that boundary along its long axis has no option.

## Identity, determinism and rejection tests

`PortalTemplateBuilderTests` and `PortalAssessmentGateTests` cover:

- same selected state rebuilds the same template IDs and the same
  `as_dict()` payload;
- changing the actual guide source/points, chain source, grid occupancy or
  vehicle shape model changes the template id (and the model hash where
  applicable);
- changing any copied identity field (guide/allocation/chain/grid/model/
  request/margin/margin source) changes the template id;
- a fake assessment `candidate_id`, request/grid/context/chain/guide digest,
  selected guide source, allocation, schema, certification claim, hard-
  constraint/certificate claim, route length, station count, duplicate or
  inconsistent station view fails closed;
- a malformed decomposition with the wrong piece count, a sigma gap, an
  out-of-chain region and a fake template hash are rejected by the immutable
  template contract or the identity-aware validator.

## Tests

`test_exit_codes.txt` records module exit codes; result lines and wall times
are in `*.log` and `*.time.log`.

| Command | Result |
|---|---|
| `python3 -m unittest test_virtual_a_formation_corridor` | 23 tests, OK, 15.375 s |
| `python3 -m unittest test_virtual_a_formation_assessment` | 40 tests, OK, 0.720 s |
| `python3 -m unittest test_virtual_a_corridor test_virtual_a_corridor_optimizer` | 52 tests, OK, 9.811 s |
| `python3 collect_step2_metrics.py` | exit 0, 0.50 s |

## Scope of the claim and Step 3/4 items

- Template selection is deterministic and bounded (`<= 3` per guide), but
  the rows are sampled at finite stations; continuous F+OBB certification,
  gate migration and publication blocking are Step 4, not claimed here.
- Synthetic cut/midpoint stations use the selected piecewise-linear guide
  pose.  Step 3 must compile the same templates against the actual solved
  `q_F(s)` and Step 4 must independently verify all three OBBs over the full
  `s` domain.
- The builder is not yet wired into the production pipeline.  `run_demo.sh`
  and the launch default remain `f_only`; union scope is exercised only by
  these explicit tests until Codex accepts the integrated gate.
- The diagnostic assessment's union coverage status is still not a
  certificate and is only a witness/hint for choosing finite covers; the
  actual cover proof is the OBB geometry check in this module.