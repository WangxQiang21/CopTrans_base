# G1 commands and exit codes

- Run id: `20261115T150114Z`
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`
- HEAD / baseline: `567bdecba9676313da3bc9a97b82f57b5b569a2b` (unchanged; no commit, no push)
- Interpreter: `/usr/bin/python3` (Python 3.8.10)
- Codex acceptance: **NOT_RUN_NOT_CLAIMED**

All commands below were run from the worktree root unless a different `cwd` is
stated.  No ROS node, Gazebo session, FR06 autonomous search, G2 code or
dependency installation was executed.

## Environment preamble

```bash
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G1/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$VA_RUN/logs"
```

The resulting `VA_RUN` is `evaluation/virtual_formation_a/G1/20261115T150114Z`.
The exact environment variables observed by the run are in
`environment.json`.

## G1-A / G1-B mandatory tests

```bash
set -o pipefail
python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v \
  2>&1 | tee "$VA_RUN/logs/geometry.log"
# exit_code=0; Ran 20 tests in 1.081s; OK
python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v \
  2>&1 | tee "$VA_RUN/logs/profile.log"
# exit_code=0; Ran 15 tests in 1.231s; OK
```

## G1-C analytic benchmark, mirror, figures and negative case

```bash
python3 -B tools/virtual_a_benchmark.py --stage G1 --output "$VA_RUN/benchmark" \
  2>&1 | tee "$VA_RUN/logs/benchmark.log"
# exit_code=0; status PASS; 23 checks; 0 failed
```

The tool refuses a non-empty output directory, a symlink and any output outside
`evaluation/virtual_formation_a/`; it only writes a new run sub-directory.
It writes `summary.json`, `samples.json`, both CSVs and four AGG figures.
Benchmark wall time was not separately instrumented (`NOT_MEASURED`); the
deterministic outputs and exit code are the evidence.

## G0 baseline tool, geometry level only

```bash
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN/legacy" --regression geometry \
  2>&1 | tee "$VA_RUN/logs/legacy_geometry.log"
# exit_code=0
# overall_status=PASS; protected_unchanged=true; regression_subprocesses=2
# geometry cases: geometry_continuous, geometry_formation (both PASS)
```

`regression_plan.json` records the geometry authorization stage as `G1` with
exactly the seven new files and `setup.py` as the single authorized tracked
change.  The G0 runner tool's own summary prints a static `"stage": "G0"` field
(hard-coded in the accepted G0 tool, which G1 is not allowed to modify); this is
a tool label, not the authorization stage.  Old tests wrote only below
`legacy/regression_artifacts/`; the protected tree is identical before and
after (`a2ac71e186841f783c5433d474a64cb03b9d8d14ee5f3dddc798f2895bc0ca55`).

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

Authoritative log: `logs/setup_registration.log`.
`logs/setup_registration_initial_probe.log` preserves an earlier probe that
failed only because `generate_distutils_setup()` was invoked from the worktree
root instead of the package directory containing `package.xml`; it is not a
product failure and is not the authoritative registration result.

The only tracked modification is the package registration:

```bash
git diff --stat -- catkin_ws/src/fwis_v3_optimizer/setup.py
# 1 file changed, 4 insertions(+), 1 deletion(-)
```

## Read-only repository checks

```bash
git rev-parse HEAD
git rev-parse --abbrev-ref HEAD
git merge-base --is-ancestor 567bdecba9676313da3bc9a97b82f57b5b569a2b HEAD
git diff --stat -- catkin_ws/src/fwis_v3_optimizer/setup.py
git status --short
```

Exit code 0 for each check; full output in `logs/git_state.log`.  No commit,
push, rebase, reset, stash, checkout of other worktrees or dependency
installation was performed.

## Manifest verification

`logs/source_manifest_verification.log` records the independent SHA-256 replay
of `source_manifest.json` after the manifest was generated.  The verifier also
checks that every run file is listed, with the two documented self-referential
exclusions (`source_manifest.json` itself and the verification log it writes).