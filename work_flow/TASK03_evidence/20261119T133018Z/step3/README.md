# TASK03 Step 3 evidence — independent sample audit and at most one cut

UTC window: 2026-11-19T14:58Z–15:03Z (this directory).
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`.
HEAD unchanged: `a3c709e57560b1060dcc422777f4131a3e558acf`.

Step 3 adds two things on top of the Step 1/2 complete-OBB rows:

1. an independent finite-sample audit of the accepted candidate, evaluating
   the exact path away from the compiled stations; and
2. at most one audit-directed cut, rebuilt into the same NLP and solved once
   with the same solver family from the previous decision vector.

Neither record is a continuous certificate.  Step 4 performs the interval
certificate over the full F and all three OBBs; publication gating is also
Step 4.

## Production files

| File | SHA-256 at end of Step 3 |
|---|---|
| `formation_continuous_a/contracts.py` | `3010a925f5402347d4387c35ec0154451e527046754330af7a4f2550b9fb1b4f` |
| `formation_continuous_a/corridor.py` | `5b32983bf8b70e59de8416441efdd69af4eca422d6d46281057e37047c431210` |
| `formation_continuous_a/formation_corridor.py` | `23db10a2b3d6201fd0c2029a5fab4a3425570fe54858575a5942ff78a2c8f809` |
| `formation_continuous_a/optimizer.py` | `8416d55eeb12b7db05c051fb36c5ebb1d9c2b72d9066f923c8911825785244e9` |
| `tests/test_virtual_a_formation_corridor.py` | `9380f1725048c1941d7eaf5d7141b1e20bf062326f305d3713abd86ffca45b77` |

`corridor.py` is unchanged from Step 2.  The protected TASK01 dirty files
remain byte-identical:

- `formation_continuous_a/geometry.py` `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
- `tests/test_virtual_a_geometry.py` `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`

See `source_hashes_end_step3.txt`, `git_status_end_step3.txt` and
`git_diff_stat_end_step3.txt`.

## Audit semantics

`audit_formation_template_samples(path, template, chain)` samples the strict
interior of each fixed template segment at
`sigma = sigma_start + width * (k + 0.5) / count`,
`count = max(3, ceil(width / 0.005))`, capped at 2048 samples.  It uses the
candidate's own `state_at_s` and maps every complete-OBB corner through the
actual vehicle centre/heading.  Each corner is checked against every
halfspace of the region assigned by that template segment, with the template
margin subtracted.

The path's declared position/angle error bounds are propagated to the corner:

```
corner_error = position_error + 0.5 * hypot(length, width) * angle_error + 1e-12
bound        = norm(row) * corner_error
evidence     = residual - bound
```

- `SAMPLED_CLEAR`: no positive residual at any sample.
- `DEFINITE_VIOLATION`: at least one `evidence > 1e-9 m`; only this status
  carries a `cut_sigma` (the witness sample's sigma).
- `UNCONFIRMED`: numerical/budget/domain uncertainty, including a positive
  residual that lies inside the error budget.  It never supplies a cut.

## Refinement semantics

When a definite violation is found, the optimizer:

1. requires the witness sigma to be finite, inside `[0, 1]`, and not already
   a compiled station (a shared tolerance `1e-12`);
2. consumes the one allowed refinement for the whole solve;
3. rebuilds the same template with `extra_cuts_sigma = existing + (witness,)`
   so the cut becomes a `targeted_cut` station (priority above allocation,
   template-boundary, Gauss and Hermite stations);
4. re-solves exactly once with the same solver family as the accepted
   candidate, seeded from that candidate's 19-value decision vector; no
   fallback family is started on timeout;
5. rejects if the refined constraint residual or complete-OBB residual exceeds
   `1e-8 m`, and re-runs the independent audit.  Only `SAMPLED_CLEAR`
   becomes a candidate for the geometry audit.

A `TIMEOUT` marks the soft deadline and stops; `UNCONFIRMED` and a repeated
definite violation are recorded rejections without a new cut or solver.

## Measured results (`step3_metrics.json`)

### Real accepted union candidate

Real IPOPT solve on the Step 1 scene, union scope, 8 intervals:

| Quantity | Measured |
|---|---|
| status | `GEOMETRY_FEASIBLE` |
| audit status | `SAMPLED_CLEAR` |
| samples / planned | `200 / 200` |
| spacing sigma | `0.005` |
| max residual / evidence | `-42.64599 / -42.64599000143129 m` |
| format variables | `19` (`S` + 9 kappa + 9 eta) |
| Hermite segments | `8` |
| F halfspace count | `352` |
| OBB scalar rows | `3888` |
| accepted OBB max residual | `0.0 m` <= `1e-8 m` |
| refinement used | `False`, outcome `NOT_NEEDED` |

### Direct definite corner violation

Walls at `y = ±1.1 m`; complete vehicle-2/3 corners reach `±1.25 m`, while
the F reference path stays inside the old rows.

| Quantity | Measured |
|---|---|
| audit status | `DEFINITE_VIOLATION` |
| cut sigma | `0.0025` |
| witness vehicle / region / halfspace / vertex | `2 / 0 / 3 / 2` |
| residual / error budget / evidence | `0.15001 / 1.4313e-9 / 0.1500099986 m` |
| F halfspace max residual (same vector) | `-1.09999 m` (F-feasible) |
| complete-OBB max residual | `0.15001 m` |
| numeric `g` size | `4514` = signed `274` + F `352` + OBB `3888` |

### Near-boundary unconfirmed

Walls moved to leave a `5.0e-10 m` positive residual:

| Quantity | Measured |
|---|---|
| audit status | `UNCONFIRMED` |
| cut sigma | `None` |
| max residual | `5.0000004e-10 m` |
| max evidence | `-9.3129e-10 m` |

### Spy-proven one cut

Solver and audit are deterministic doubles; the constraint rebuild, numeric
vector, and CasADi `g` are production code.

| Quantity | Initial | Refined | Delta |
|---|---|---|---|
| targeted stations | `0` | `3` samples at sigma `0.3` | `+3` |
| OBB samples | `243` | `246` | `+3` |
| OBB scalar rows | `3888` | `3936` | `+48` |
| rows per vehicle | `1296 / 1296 / 1296` | `1312 / 1312 / 1312` | `+16 each` |
| variables | `19` | `19` | `0` |
| segments | `8` | `8` | `0` |

`solver_call_count = 2`; the refined guess source is `formation_refinement`
and its label ends with `|targeted_cut`.  `corridor_end = 4562`,
`numeric_size = 4562`, CasADi `g` size = `4562`, and all values match.  The
result is `GEOMETRY_FEASIBLE`, attempt statuses are
`SOLVER_SUCCESS_FORMATION_REFINEMENT` then `SOLVER_SUCCESS_AUDITED`, and the
refinement record is `used=True, attempts=1, cuts=[0.3], outcome=ACCEPTED`.

### Real F-feasible / complete-OBB-infeasible

Walls `2.2 m` apart cannot contain the `2.5 m` complete three-vehicle OBB span.
The real solver run returns `BUDGET_EXHAUSTED` after `SOLVER_FAILED`; no
candidate is accepted, `path=None`, and no `SOLVER_SUCCESS_AUDITED` attempt
exists.  IPOPT's max-wall-time is a soft option: the recorded wall time was
`9.76 s` against a `3.0 s` budget, and the evidence says so rather than
claiming a hard interrupt.

## Tests

`test_exit_codes.txt` records module exit codes; result lines and wall times
are in `*.log` and `*.time.log`.

| Command | Result |
|---|---|
| `python3 -m unittest test_virtual_a_formation_corridor` | 34 tests, OK, 36.421 s |
| `python3 -m unittest test_virtual_a_formation_assessment` | 40 tests, OK, 0.732 s |
| `python3 -m unittest test_virtual_a_corridor test_virtual_a_corridor_optimizer` | 52 tests, OK, 10.227 s |
| `python3 -m unittest test_virtual_a_optimizer` | 37 tests, OK, 68.357 s |
| `python3 -m unittest test_virtual_a_geometry` | 24 tests, OK, 1.165 s |
| `python3 -m unittest test_virtual_a_pipeline` | 13 tests, OK, 122.615 s |
| `python3 collect_step3_metrics.py` | exit 0 |

The new `FormationSampleAuditStep3Tests` cover: clear audit on a real accepted
candidate; direct definite/unconfirmed audits; the F-feasible-but-OBB-out
numeric rows and real non-acceptance; the one-cut re-entry into numeric and
CasADi `g`; a second definite violation stopping without a third solver;
refinement timeout; refinement solver failure; and UNCONFIRMED adding no cut
or solver.  Two later additions cover the scheduler regression and the
same-family fallback: a definite cut at a station that is already compiled
(`0.3125`) spends no refinement and adds no solver call; a SciPy-accepted
candidate refines with SLSQP only, seeded from its own 19-vector.  The protected TASK01 geometry suite still passes.

## Scope of the claim and Step 4 items

- The audit is finite samples, not an interval certificate.  A
  `SAMPLED_CLEAR` status can still miss a violation between samples.
- The refinement is limited to one cut and one same-family solve.  No
  infinite candidate/retry, no threshold relaxation, no extra solver on
  timeout.
- The union scope is still not the production default.  `run_demo.sh` and the
  launch default remain `f_only`; Step 6 integrates the explicit union path.
- Step 4 must independently certify all three OBBs over the full `s` domain
  with interval bounds and two-sided cuts before any gate or publication
  change.