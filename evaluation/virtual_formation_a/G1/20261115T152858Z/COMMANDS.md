# G1 rework commands and exit codes

- Run id: `20261115T152858Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / baseline: `567bdecba9676313da3bc9a97b82f57b5b569a2b`
- Interpreter: `/usr/bin/python3` (Python 3.8.10)
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**
- Old G1 runs `20261115T150639Z` and `20261115T150114Z`: preserved unchanged

All commands below were run from the worktree root unless a different `cwd`
is stated.  The logs named here are direct command output.  No ROS node,
Gazebo session, FR06 autonomous search, NLP solve, G2 work, dependency
installation, Git commit or push was performed.

## Run directory creation

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
RUN="evaluation/virtual_formation_a/G1/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$RUN/logs"
printf 'VA_RUN=%s\n' "$RUN" > "$RUN/logs/run_id.txt"
# printed: evaluation/virtual_formation_a/G1/20261115T152858Z
```

## Environment preamble for the execution commands

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
RUN=evaluation/virtual_formation_a/G1/20261115T152858Z
set -o pipefail
```

`environment.json` was captured separately at 15:30:21 UTC from a shell without
the package `PYTHONPATH` prefix; it records the ambient value
`/opt/ros/noetic/lib/python3/dist-packages`.  Test and benchmark imports used
the package-source prefix shown above.

## Mandatory G1 tests

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v \
  2>&1 | tee "$RUN/logs/geometry.log"
# geometry_exit=0; Ran 20 tests in 1.055s; OK

python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v \
  2>&1 | tee "$RUN/logs/profile.log"
# profile_exit=0; Ran 22 tests in 1.280s; OK
```

The profile run contains six new `SimpsonRemainderReworkTests`; all sixteen
original profile tests remain.  Raw logs: `logs/geometry.log`,
`logs/profile.log`.

## Analytic benchmark, mirror and L=0.5 m negative case

```bash
python3 -B tools/virtual_a_benchmark.py --stage G1 \
  --output "$RUN/benchmark" 2>&1 | tee "$RUN/logs/benchmark.log"
# benchmark_exit=0
# status=PASS; 26 checks; 0 failed; position_error_bound_m=7.737011020460756e-08
```

The benchmark writes `summary.json`, `samples.json`, both CSVs and four AGG
figures.  Wall time was `NOT_MEASURED`; the deterministic outputs, check list
and exit code are the evidence.  Three checks are new rework checks:
`simpson_budget_fields_use_2d_full_panels`,
`simpson_budget_arithmetic_recomputed`, `partial_query_local_panel_bound`.

## Safe geometry regression: unmodified gate attempt

```bash
python3 -B tools/virtual_a_baseline.py --output "$RUN/legacy" \
  --regression geometry 2>&1 | tee "$RUN/logs/legacy_geometry.log"
# legacy_exit=1
# G0 FAILED: BaselineError: UNKNOWN_WORKSPACE_MODIFICATIONS:
# ['docs/virtual_formation_a/CLAUDE_G1_REWORK_01.md']
```

The failure was preserved verbatim as
`logs/legacy_geometry_unmodified_gate.log`.  It happens before any regression
test runs: the accepted G0 tool's static `KNOWN_DESIGN_DOCUMENTS` allowlist
predates the user-supplied rework task book.  This attempt is **FAILED**, not
PASS.

## Safe geometry regression: documented wrapper run

The wrapper below imports the unmodified `tools/virtual_a_baseline.py` and adds
only the rework document to the in-memory allowlist.  It changes no file and no
other authorization rule.  Its full source is preserved at
`baseline_runner_rework_doc.py`.

```bash
python3 -B "$RUN/baseline_runner_rework_doc.py" --output "$RUN/legacy" \
  --regression geometry 2>&1 | tee "$RUN/logs/legacy_geometry.log"
# legacy_wrapped_exit=0
# overall_status=PASS; protected_unchanged=true; reference_checks=13390;
# reference_status=FEASIBLE; regression_subprocesses=2;
# next_stage_entered=false
```

The old tests wrote only below `legacy/regression_artifacts/`; the protected
tree is identical before and after the run
(`910efe32972dcbcb3a0c2585c002916ede878f4295b0f26b4dc917faeee4f519`).
The accepted G0 tree was
`ff58f2b5c6acfbc207327202f875e5e9a20734ef09ebb6070ee07edce6ebf091` (3120
files; this run has 3127 protected files, the seven G1 additions).

**Qualification:** the regression subprocesses passed, but the unmodified gate
failed.  The wrapper is a documented workaround for Codex review; this run does
not self-accept it.  The accepted baseline tool itself was not edited.

## Package registration verification

```bash
# cwd = catkin_ws/src/fwis_v3_optimizer
python3 -B setup.py --name
# fwis_v3_optimizer; exit_code=0

# cwd = catkin_ws/src/fwis_v3_optimizer (needs package.xml beside it)
python3 -B -c "from catkin_pkg.python_setup import generate_distutils_setup as g; \
  a=g(packages=['fwis_v3_optimizer','fwis_v3_optimizer.formation_continuous_a'], \
  package_dir={'':'src'}); print(a['name']); print(a['packages'])"
# ['fwis_v3_optimizer', 'fwis_v3_optimizer.formation_continuous_a']; exit_code=0

# cwd = worktree root
python3 -B -c "import fwis_v3_optimizer.formation_continuous_a as m; \
  print(m.__name__); print(sorted(m.__all__))"
# exit_code=0
```

Authoritative log: `logs/setup_registration.log` (three checks, all exit 0).
The only tracked modification is this package registration
(`1 file changed, 4 insertions(+), 1 deletion(-)`).

## Read-only repository checks

```bash
git rev-parse HEAD
git rev-parse --abbrev-ref HEAD
git merge-base --is-ancestor 567bdecba9676313da3bc9a97b82f57b5b569a2b HEAD
git diff --stat -- catkin_ws/src/fwis_v3_optimizer/setup.py
git status --short
```

Each command exited 0; full output is in `logs/git_state.log`.  There is no
commit, push, rebase, reset, stash, checkout of another worktree, or dependency
installation.

## Environment capture

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B - <<'PY'
# imports platform, sys, os, numpy, scipy, matplotlib, catkin_pkg, casadi,
# yaml and PIL, then writes sorted JSON to environment.json
PY
# exit_code=0
```

The resulting file is `environment.json`; it records Python 3.8.10, numpy
1.24.4, scipy 1.10.1, matplotlib 3.1.2 (Agg), catkin_pkg 1.0.0, casadi 3.7.2,
PyYAML 5.3.1 and Pillow 10.4.0.

## Numerical evidence assembly

```bash
PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 \
  python3 -B "$RUN/analysis/budget_evidence.py" \
  > "$RUN/analysis/budget_evidence.json" 2>>"$RUN/logs/budget_evidence.log"
# exit_code=0; the log adds the command, exit code and a hash summary
```

The evidence script covers five hand-parameter cases `(h,n) =
(2,2),(1.7,4),(3,10),(0.8,200),(6,600)`, the analytic double-lane-change
budget/refinement/partial queries, and a legal C2 profile with non-zero `g`
compared against adaptive quadrature.  Output: `analysis/budget_evidence.json`;
log: `logs/budget_evidence.log`.

Evidence-assembly correction: the first invocation of this script exited 1
with `AttributeError: 'ControlKnot' object has no attribute 'kappa'` because
the evidence script used wrong field names.  It was corrected to
`.value`/`.derivative`; no production or test code was affected.  The preserved
log is the final successful run.

## Protection and results assembly

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B "$RUN/analysis/protection_diff.py"
# exit_code=0; added=7, removed=0, changed=1, only_authorized=true

PYTHONDONTWRITEBYTECODE=1 python3 -B "$RUN/analysis/assemble_results.py"
# exit_code=0; status=PASS_PENDING_CODEX_RECHECK
```

Outputs: `protection_diff_vs_g0.json` and `results.json`; logs:
`logs/protection_diff.log`, `logs/assemble_results.log`.

## Manifest generation and verification

After this document and `CLAUDE_REPORT.md` were finalized, the manifest was
generated by `analysis/write_source_manifest.py` and independently replayed by
`analysis/verify_source_manifest.py`:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -B \
  "$RUN/analysis/write_source_manifest.py" \
  2>&1 | tee "$RUN/logs/source_manifest_generation.log"
# exit_code=0; writes source_manifest.json; the log prints the manifest SHA-256,
# run-file count/bytes, source-file count and both old evidence tree hashes.

PYTHONDONTWRITEBYTECODE=1 python3 -B -I \
  "$RUN/analysis/verify_source_manifest.py" \
  2>&1 | tee "$RUN/logs/source_manifest_verification.log"
# exit_code=0 only if every rehash and every cross-check passes
```

The three documented self-referential exclusions are `source_manifest.json`
itself, `logs/source_manifest_verification.log` (written while the verifier is
checking) and `logs/source_manifest_generation.log` (written while the
generator is scanning).  The verification log records the manifest SHA-256,
run-file coverage and bytes, all source hashes, every protected-file rehash,
the G0 whitelist, budgets/quadrature comparisons, setup registration, git
state, old-evidence trees and the final exit code.

## Non-claims

- `1e-7 m` position budget and `1e-9 rad` angle budget were not relaxed.
- Sampled quadrature differences are numerical evidence, not a continuous-domain
  proof.
- No FR06 autonomous search, ROS/Gazebo run, hardware calibration, full legacy
  regression or G2 entry occurred.
- Codex acceptance is **NOT_RUN_NOT_CLAIMED**.  This run stops for independent
  recheck.