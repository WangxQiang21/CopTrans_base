# G4 commands, exit codes and run record

All commands ran from `/home/wq/whut_thesis/expv3_multi_virtual_a` in the
`feature/virtual-rigid-formation-a` worktree at ancestor
`567bdecba9676313da3bc9a97b82f57b5b569a2b`. No branch, commit, push, reset,
stash or operation on another worktree was performed. The run directory
`evaluation/virtual_formation_a/G4/20261116T055008Z` was created fresh for
this delivery; the earlier `G4/20261116T053548Z` directory was left untouched
and is historical, read-only evidence from before the two pre-final fixes.

## Environment preamble

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G4/20261116T055008Z"
mkdir -p "$VA_RUN/logs"
set -o pipefail
```

Python `3.8.10`, NumPy `1.24.4`, SciPy `1.10.1`, CasADi `3.7.2` with the
Ipopt plugin, PyYAML `5.3.1`. Full host/package/solver/git data is in
[environment.json](environment.json).

## Scene and reference freeze (G4-A)

```bash
python3 -B tools/virtual_a_scenes.py --output "$VA_RUN/scenes" \
  --verify-references 2>&1 | tee "$VA_RUN/logs/scenes.log"
```

Exit `0`. Six scenes (`S1`, `S1_mirror`, `S2`, `S2_mirror`, `S3`,
`S3_mirror`) were built from the authoritative maps, written with only
`map.pgm`, `map.yaml` and `request.json`, each independently re-timed and
audited, and each reference verified against the authoritative map and
request. Log: six `REFERENCE_PASSED` reference records and six
`verify <scene>: PASS` lines; `manifest: frozen=True scenes=6 failures=0`.

## Focused G4 suites

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_collision.py' \
  -v 2>&1 | tee "$VA_RUN/logs/collision.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' \
  -v 2>&1 | tee "$VA_RUN/logs/pipeline.log"
```

Exit `0` for both: collision `Ran 11 tests in 1.818s OK`; pipeline
`Ran 8 tests in 112.844s OK`.

The collision suite carries the mandatory G4 negative cases. The pipeline
suite runs only map/request inputs, patches `Path.read_text`, `Path.read_bytes`
and `json.load` to raise if the pipeline reads any file, and covers the true
OBB `door_circumcircle` case, the exact hold, blocked hold, fake solver
success, retained rejected evidence, tiny total budget, and route guides as
non-executable topology hints.

## Official autonomous benchmark (G4-C)

```bash
python3 -B tools/virtual_a_benchmark.py --stage G4 \
  --scene-manifest "$VA_RUN/scenes/scene_manifest.json" \
  --output "$VA_RUN/benchmark" 2>&1 | tee "$VA_RUN/logs/benchmark.log"
```

Exit `0`; `status=PASS`; 72 checks; 0 failed; 6/6 scenes executable SUCCESS.
Per-scene results, all attempts, audit payloads, sweep bounds and timed models
are in `benchmark/results.json`; the check list is in
`benchmark/summary.json`.

## Preserved G1/G2/G3 regressions

The accepted earlier test files were not modified and were rerun against the
current tree:

```bash
: > "$VA_RUN/logs/g1_g2_g3_regression.log"
for name in geometry optimizer profile audit timing trajectory; do
  echo "===== test_virtual_a_${name}.py =====" >> "$VA_RUN/logs/g1_g2_g3_regression.log"
  python3 -B -m unittest discover -s tests -p "test_virtual_a_${name}.py" \
    -v >> "$VA_RUN/logs/g1_g2_g3_regression.log" 2>&1
  code=${PIPESTATUS[0]}
  echo "test_${name}=$code" >> "$VA_RUN/logs/regression_exit_codes.log"
done
```

All six exits `0`: geometry 20 tests, optimizer 26, profile 22, audit 51,
timing 18, trajectory 23; each suite ends `OK`.

```bash
python3 -B tools/virtual_a_benchmark.py --stage G1 \
  --output "$VA_RUN/regression/g1_benchmark" 2>&1 | tee "$VA_RUN/logs/regression_g1_benchmark.log"
python3 -B tools/virtual_a_benchmark.py --stage G2 \
  --output "$VA_RUN/regression/g2_benchmark" 2>&1 | tee "$VA_RUN/logs/regression_g2_benchmark.log"
python3 -B tools/virtual_a_benchmark.py --stage G3 \
  --output "$VA_RUN/regression/g3_benchmark" 2>&1 | tee "$VA_RUN/logs/regression_g3_benchmark.log"
```

Exits `0`: G1 `PASS` 26 checks, G2 `PASS` 124 checks, G3 `PASS` 151 checks,
0 failed in each. `logs/exit_codes.log` records scenes/collision/pipeline/
benchmark; `logs/regression_exit_codes.log` records the nine regression
steps above.

## Protection comparison

```bash
python3 -B "$VA_RUN/analysis/protection_diff_vs_g3_accepted.py" \
  2>&1 | tee "$VA_RUN/logs/protection_diff.log"
```

Exit `0`. The current protected tree is compared with the Codex-accepted G3
tree reconstructed from the accepted G1 snapshot plus the authorized G2/G3
changes. Result: 38 additions, 5 changes, 0 removals, 3175 current files,
unchanged common-subtree digests equal, full protected tree
`157686a3832cb0fd94241407866e33443a3b9188483a257fd16510916e78143d`.
G3 acceptance, source manifest and every non-self-referential G3 run file were
rehashed before comparison. See
[analysis/protection_diff_vs_g3_accepted.json](analysis/protection_diff_vs_g3_accepted.json).

## Independent numerical recheck

```bash
python3 -B "$VA_RUN/analysis/g4_numerics.py" \
  2>&1 | tee "$VA_RUN/logs/g4_numerics.log"
```

Exit `0`. Standard-library-only reparse of every PGM/YAML from raw bytes,
conservative occupancy rebuild, canonical grid-signature recomputation,
request/config immutability, frozen-reference checks, benchmark thresholds,
timed-model digest, per-vehicle error-envelope arithmetic, per-λ sweep bound
containment and dynamic whole-domain certificates. Writes
[analysis/g4_numerics.json](analysis/g4_numerics.json).

## Evidence assembly and environment

```bash
python3 -B "$VA_RUN/analysis/write_environment.py" \
  2>&1 | tee "$VA_RUN/logs/environment.log"
python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  2>&1 | tee "$VA_RUN/logs/assemble_evidence.log"
```

Exit `0` for both. `assemble_evidence.py` requires every claimed negative test
to appear as `... ok` in this run's logs and writes
[results.json](results.json) with `status: PASS_PENDING_CODEX_RECHECK` and
`codex_acceptance: NOT_RUN_NOT_CLAIMED`.

## Source-and-evidence manifest

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
python3 -B -I "$VA_RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
```

The verifier was probed once before the manifest existed
(`logs/premanifest_verifier_probe.log`); it exits `1` with
`VERIFY FAILED: source manifest exists`, which is the expected
pre-generation result. The authoritative verification is the run after
manifest generation below.

Exit `0` for both. The manifest binds 70 source files: the 31 accepted G3
sources (5 changed by G4, 26 byte-identical), 7 new G4 code/test/tool/doc
files, and 32 new scene/synthetic map/request fixtures. It also binds the
accepted G1/G2/G3 evidence digests, the G4 protected comparison, all run files
except its own three self-referential artifacts, and the git state. The
verifier is stdlib-only and recomputes source, fixture, protection, results,
benchmark and log digests without importing the project.

## Timing note

Deterministic checks and digests are the evidence, not wall time. The recorded
per-scene G4 pipeline wall times are 12.1 s to 24.3 s (S3/S3_mirror include a
collision-audit retry round); the six-scene autonomous benchmark totals about
99.5 s. `search.budget.hard_real_time_guarantee` is `false` in every result.