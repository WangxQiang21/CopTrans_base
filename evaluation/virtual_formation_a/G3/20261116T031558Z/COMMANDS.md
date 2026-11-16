# G3 rework commands and exit codes

All commands ran from `/home/wq/whut_thesis/expv3_multi_virtual_a` in the
`feature/virtual-rigid-formation-a` worktree. No branch, commit, push, reset,
stash or operation on another worktree was performed.

## Environment preamble

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G3/20261116T031558Z"
set -o pipefail
```

## Focused G3 suites

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_timing.py' -v 2>&1 | tee "$VA_RUN/logs/timing.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_trajectory.py' -v 2>&1 | tee "$VA_RUN/logs/trajectory.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v 2>&1 | tee "$VA_RUN/logs/audit.log"
```

Exit codes: timing `0` (18 tests), trajectory `0` (23 tests), audit `0`
(51 tests). The added timing/audit tests include the Codex degenerate
counterexample and narrow intervals covering every `p_sss`/jerk component and
norm for all three offsets and both signs of curvature and curvature rate.

## Preserved G1/G2 regression suites

The G1 geometry, G2 optimizer and G2 profile test files were not modified;
they were rerun against the current tree:

```bash
: > "$VA_RUN/logs/g1_g2_regression.log"
for name in geometry optimizer profile; do
  echo "===== test_virtual_a_${name}.py =====" | tee -a "$VA_RUN/logs/g1_g2_regression.log"
  python3 -B -m unittest discover -s tests -p "test_virtual_a_${name}.py" -v \
    2>&1 | tee -a "$VA_RUN/logs/g1_g2_regression.log"
  code=${PIPESTATUS[0]}
  echo "test_virtual_a_${name}_exit=$code" | tee -a "$VA_RUN/logs/g1_g2_regression.log"
done
```

Exit codes: geometry `0` (20 tests), optimizer `0` (26 tests), profile `0`
(22 tests).

## Independent numerical interval-coverage evidence

```bash
python3 -B "$VA_RUN/analysis/g3_rework_numerics.py" \
  | tee "$VA_RUN/logs/g3_rework_numerics.log"
```

Exit `0`. The script writes `analysis/g3_rework_numerics.json`, which is bound
by `results.json` and the source manifest. Its `timing` and `audit` records
invoke the two production interval implementations; its pointwise formulas,
quintic inversion and dense scan are independent handwritten calculations.

## Benchmark

```bash
python3 -B tools/virtual_a_benchmark.py --stage G3 --output "$VA_RUN/benchmark"
```

Exit `0`. The benchmark wrote 151 checks, 0 failed, 9 timed models (8 motion
plus 1 exact hold). Each motion case now records 909 audit checks: the previous
845 checks plus 64 pointwise leaf-checks.

## Legacy timing regression through the G0 runner

```bash
# The unmodified G0 gate is expected to BLOCK on prior-stage documents and
# fixtures that accepted G1/G2 authorized; it is run unchanged first.
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN/legacy" --regression timing \
  2>&1 | tee "$VA_RUN/logs/legacy_unmodified_gate.log"
# Disclosed wrapper: adds only those prior-stage documents/fixtures to the G0
# in-memory allowlists and edits no workspace file. It now also allows the
# user-supplied G3 rework order in memory.
python3 -B "$VA_RUN/baseline_runner_rework_docs_and_fixtures.py" \
  --output "$VA_RUN/legacy" --regression timing \
  2>&1 | tee "$VA_RUN/logs/legacy.log"
```

Exit codes: unmodified gate `1` (BLOCKED:
`UNKNOWN_WORKSPACE_MODIFICATIONS`), disclosed-wrapper regression `1`
(overall FAIL). Four cases pass and `timing_continuous` fails with the
pre-existing `KeyError: 'dynamic_peaks'`. No legacy test or source was changed.

The pristine-HEAD probe used a `git archive` export:

```bash
rm -rf /tmp/g3_rework_baseline_probe_20261116
mkdir -p /tmp/g3_rework_baseline_probe_20261116
git archive 567bdecba9676313da3bc9a97b82f57b5b569a2b | tar -x -C /tmp/g3_rework_baseline_probe_20261116
(cd /tmp/g3_rework_baseline_probe_20261116 && \
  env -u PYTHONPATH PYTHONDONTWRITEBYTECODE=1 python3 -B -m unittest \
  tests.test_continuous_retiming.ContinuousRetiming.test_short_translation_has_finite_acceleration_and_zero_end_speed -v) \
  > "$VA_RUN/logs/legacy_baseline_probe.log" 2>&1
probe_exit=$?
echo "probe_exit=$probe_exit" >> "$VA_RUN/logs/legacy_baseline_probe.log"
```

Probe exit `1`, reproduced at `tests/test_continuous_retiming.py:271`
(`KeyError: 'dynamic_peaks'`). The failing test and source hashes are bound in
`results.json`; old tests were neither changed nor relaxed.

## Evidence analysis

```bash
python3 -B "$VA_RUN/analysis/protection_diff_vs_g2_accepted.py" \
  | tee "$VA_RUN/logs/protection_diff.log"
python3 -B "$VA_RUN/analysis/write_environment.py" \
  | tee "$VA_RUN/logs/write_environment.log"
python3 -B "$VA_RUN/analysis/assemble_evidence.py" \
  | tee "$VA_RUN/logs/assemble_evidence.log"
```

Exit codes: protection diff `0`, environment `0`, assemble evidence `0`.
`CLAUDE_REPORT.md` is written by the agent from these frozen records; it does
not re-solve or alter a model.

## Manifest and independent verification

```bash
python3 -B "$VA_RUN/analysis/write_source_manifest.py" 2>&1 | tee "$VA_RUN/logs/source_manifest_generation.log"
gen_status=${PIPESTATUS[0]}
echo "source_manifest_generation_exit=$gen_status" >> "$VA_RUN/logs/source_manifest_generation.log"

PYTHONDONTWRITEBYTECODE=1 python3 -B -I "$VA_RUN/analysis/verify_source_manifest.py" 2>&1 | tee "$VA_RUN/logs/source_manifest_verification.log"
verify_status=${PIPESTATUS[0]}
echo "source_manifest_verification_exit=$verify_status" >> "$VA_RUN/logs/source_manifest_verification.log"
```

The manifest binds every source and run file except its three self-referential
artifacts (`source_manifest.json` and the two manifest logs). The verifier
recomputes all hashes, the G2 protected-tree reconstruction, the G3 whitelist,
all nine model/request digests, the 151 benchmark checks, the 18/23/51 test
counts, the 909/16 audit check counts, 512 pointwise leaf certificates and
1536 vehicle containment ratios, the Codex degenerate counterexample, 24
narrow-interval cases, all four curvature/curvature-rate sign pairs, the
independent negative and fail-closed witnesses, the differential/inverse
tolerances, the legacy truth and the git state. The manifest SHA-256 is
reported in `logs/source_manifest_verification.log`.

## Recorded exits

`logs/exit_codes.log` records the exits of the commands above that run before
manifest generation, including the three preserved G1/G2 suites. The generator
and verifier exits are written to their own excluded logs to avoid a
self-referential hash cycle.