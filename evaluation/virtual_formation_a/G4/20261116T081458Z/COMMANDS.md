# G4 rework 01 commands, exit codes and run record

All commands ran from `/home/wq/whut_thesis/expv3_multi_virtual_a` in the
`feature/virtual-rigid-formation-a` worktree at ancestor
`567bdecba9676313da3bc9a97b82f57b5b569a2b`. No branch, commit, push, reset,
stash or operation on another worktree was performed. The run directory
`evaluation/virtual_formation_a/G4/20261116T081458Z` was created fresh for this
rework; every earlier G4 run (including `20261116T055008Z` and
`20261116T053548Z`) and all G1/G2/G3 evidence were left untouched and read-only.

## Environment preamble

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G4/20261116T081458Z"
mkdir -p "$VA_RUN/logs"
set -o pipefail
```

Python `3.8.10`, NumPy `1.24.4`, SciPy `1.10.1`, CasADi `3.7.2` with the Ipopt
plugin, PyYAML `5.3.1`. Full host/package/solver/git data is in
[environment.json](environment.json).

## P0/P1 source changes before the run

- `formation_continuous_a/collision.py`: `error_envelope` now solves
  `margin >= baseline + 2*max(e_i)`, checks the actual checker binary64
  per-side growth and expanded half diagonal, records all five contexts and
  keeps the `0.01 m` sweep padding strictly as motion-sampling protection.
- `formation_continuous_a/integration_bounds.py` (new): exact per-panel
  interval bound of `|f''''|` for `f = exp(i psi)`, using
  `f''''/f = (k^4 - 4ku - 3e^2) + i(g - 6k^2 e)`, outward interval polynomial
  arithmetic, widened coefficients/panels and the unchanged Simpson remainder
  and floating/angle guard.
- `formation_continuous_a/audit.py`: the independent audit uses the exact-panel
  module and refines the worst segment in even increments rather than doubling
  it, because the guard grows with the panel count.
- `tools/virtual_a_scenes.py`, `tests/virtual_a_g4_support.py` and the six
  frozen request fixtures: `audit_position_budget_m = 5e-8`, not `1e-7`.
- `tools/virtual_a_benchmark.py`: G4 checks now include the factor-2 closure,
  per-side growth, expanded-radius enclosure and all five collision contexts.
- New `tests/test_virtual_a_integration_bounds.py`; six new P0 tests in
  `tests/test_virtual_a_collision.py`. No test was deleted or relaxed.

## Scene and reference freeze (G4-A, restored 5e-8 m)

```bash
python3 -B tools/virtual_a_scenes.py --output "$VA_RUN/scenes" \
  --verify-references > "$VA_RUN/logs/scenes.log" 2>&1
```

Exit `0`. Six scenes (`S1`, `S1_mirror`, `S2`, `S2_mirror`, `S3`,
`S3_mirror`) were built with only `map.pgm`, `map.yaml` and `request.json`,
each independently re-timed and audited at the restored independent geometry
threshold `5e-8 m`, and each reference verified against the authoritative map
and request. Log: six `REFERENCE_PASSED` records, six `verify <scene>: PASS`
lines and `manifest: frozen=True scenes=6 failures=0`.

## Focused G4 suites

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_collision.py' \
  -v > "$VA_RUN/logs/collision.log" 2>&1
python3 -B -m unittest discover -s tests \
  -p 'test_virtual_a_integration_bounds.py' -v \
  > "$VA_RUN/logs/integration_bounds.log" 2>&1
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' \
  -v > "$VA_RUN/logs/pipeline.log" 2>&1
```

Exits `0`: collision 17 tests, integration bounds 6 tests, pipeline 8 tests.
The collision suite now includes the five real-checker near-contact
counterexamples for static start/goal, NO_MOTION, map boundary and
vehicle-vehicle coverage, plus the sweep-padding role and closure/radius/
context checks. The integration suite verifies the exact panel bound against
dense actual derivatives, the S2 12 m reference at `5e-8 m`, fail-closed
behaviour and the frozen inputs/defaults.

## Official autonomous benchmark (G4-C)

```bash
python3 -B tools/virtual_a_benchmark.py --stage G4 \
  --scene-manifest "$VA_RUN/scenes/scene_manifest.json" \
  --output "$VA_RUN/benchmark" > "$VA_RUN/logs/benchmark.log" 2>&1
```

Exit `0`; `status=PASS`; 84 checks; 0 failed; 6/6 scenes executable SUCCESS.
Per-scene results, all attempts, audit payloads, sweep bounds, timed models and
the factor-2 envelope records are in `benchmark/results.json`; the check list is
in `benchmark/summary.json`.

## Preserved G1/G2/G3 regressions

The earlier accepted test files were not modified except for the G3 sources
listed in the protection diff; the suites were rerun against the current tree:

```bash
: > "$VA_RUN/logs/g1_g2_g3_regression.log"
: > "$VA_RUN/logs/regression_exit_codes.log"
for name in geometry optimizer profile audit timing trajectory; do
  echo "===== test_virtual_a_${name}.py =====" >> "$VA_RUN/logs/g1_g2_g3_regression.log"
  python3 -B -m unittest discover -s tests -p "test_virtual_a_${name}.py" \
    -v >> "$VA_RUN/logs/g1_g2_g3_regression.log" 2>&1
  code=$?
  echo "test_${name}=$code" >> "$VA_RUN/logs/regression_exit_codes.log"
done
```

All six exits `0`: geometry 20 tests, optimizer 26, profile 22, audit 51,
timing 18, trajectory 23; each suite ends `OK`.

```bash
python3 -B tools/virtual_a_benchmark.py --stage G1 \
  --output "$VA_RUN/regression/g1_benchmark" > "$VA_RUN/logs/regression_g1_benchmark.log" 2>&1
python3 -B tools/virtual_a_benchmark.py --stage G2 \
  --output "$VA_RUN/regression/g2_benchmark" > "$VA_RUN/logs/regression_g2_benchmark.log" 2>&1
python3 -B tools/virtual_a_benchmark.py --stage G3 \
  --output "$VA_RUN/regression/g3_benchmark" > "$VA_RUN/logs/regression_g3_benchmark.log" 2>&1
```

Exits `0`: G1 `PASS` 26 checks, G2 `PASS` 124 checks, G3 `PASS` 151 checks,
9 dynamic-feasible timed cases.

## Known G3 legacy timing failure (preserved, not fixed)

The pristine-HEAD probe was repeated from a `git archive` export without
loading the working tree:

```bash
rm -rf /tmp/g4_rework_baseline_probe_20261116
mkdir -p /tmp/g4_rework_baseline_probe_20261116
git archive 567bdecba9676313da3bc9a97b82f57b5b569a2b | \
  tar -x -C /tmp/g4_rework_baseline_probe_20261116
(cd /tmp/g4_rework_baseline_probe_20261116 && \
  env -u PYTHONPATH PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest \
  tests.test_continuous_retiming.ContinuousRetiming.test_short_translation_has_finite_acceleration_and_zero_end_speed -v) \
  > "$VA_RUN/logs/legacy_baseline_probe.log" 2>&1
echo "probe_exit=$?" >> "$VA_RUN/logs/legacy_baseline_probe.log"
```

Probe exit `1`, still failing with the pre-existing
`KeyError: 'dynamic_peaks'` at `tests/test_continuous_retiming.py:271`. No
legacy test or source was changed. The separately recorded G3 non-blocking
skip in `_audit_check_actual_metrics` for non-finite per-wheel values is still
an open non-blocking item and is not claimed fixed.

## Independent evidence analysis

```bash
python3 -B "$VA_RUN/analysis/protection_diff_vs_g3_accepted.py" \
  > "$VA_RUN/logs/protection_diff.log" 2>&1
python3 -B "$VA_RUN/analysis/write_environment.py" \
  > "$VA_RUN/logs/environment.log" 2>&1
python3 -B "$VA_RUN/analysis/g4_numerics.py" \
  > "$VA_RUN/logs/g4_numerics.log" 2>&1
python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  > "$VA_RUN/logs/assemble_evidence.log" 2>&1
```

Exits `0`. The protection diff reports 40 added and 5 changed protected paths,
0 removals, and the old G3 protected tree unchanged. `g4_numerics.json`
independently reparses the maps, rebuilds the Simpson budget from the
serialised knots and recorded panel bounds, densely samples the actual fourth
derivative below every recorded panel bound, and checks the factor-2 closure
with exact `Fraction` arithmetic.

## Source manifest

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  > "$VA_RUN/logs/source_manifest_generation.log" 2>&1
python3 -B "$VA_RUN/analysis/verify_source_manifest.py" \
  > "$VA_RUN/logs/source_manifest_verification.log" 2>&1
```

Both exit `0`; see the final logs for the bound counts and hashes.

## Historical evidence and stop

All earlier run directories were opened read-only. The previous G4 run
`evaluation/virtual_formation_a/G4/20261116T055008Z` is additionally rechecked
against its own source manifest without writing into it:

```bash
python3 -B "$VA_RUN/analysis/verify_historical_evidence.py" \
  > "$VA_RUN/logs/historical_evidence.log" 2>&1
```

Exit `0`: `status=UNCHANGED`, all 78 recorded old run files (29,709,240 bytes)
match their recorded sizes and SHA-256 digests, and the old manifest itself is
`353b1e1f8733e53a14969575ac086466edcf089ce65283d03c3f31ce886e79ef`. The
script's three self-referential exclusions are the old manifest and its
generation/verification logs.

The old `formation_collision.py`, old maps and old experiments are unchanged;
the protection diff binds this. This rework stops after the report and waits
for Codex re-review. It does not enter G5 and creates no Git commit, branch,
tag or push.