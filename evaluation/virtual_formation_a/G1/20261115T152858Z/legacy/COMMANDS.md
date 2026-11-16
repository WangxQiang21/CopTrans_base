# G0 Commands

## Main invocation

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- interpreter: `/usr/bin/python3`
- Python: `3.8.10 (default, Mar 18 2025, 20:04:55)  [GCC 9.4.0]`
- environment increments: `PYTHONDONTWRITEBYTECODE=1`, `OPENBLAS_NUM_THREADS=1`
- regression level: `geometry`
- output: `/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy`
- exit code: `0`

```bash
/usr/bin/python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tools/virtual_a_baseline.py --output evaluation/virtual_formation_a/G1/20261115T152858Z/legacy --regression geometry
```

The reference audit ran in this process. No historical verification or experiment script was executed.

## Regression subprocesses

### geometry_continuous

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/geometry_continuous.stdout.log`
- stderr: `logs/geometry_continuous.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_continuous/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_continuous_geometry.py -v
```

### geometry_formation

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/geometry_formation.stdout.log`
- stderr: `logs/geometry_formation.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G1/20261115T152858Z/legacy/regression_artifacts/geometry_formation/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_formation_geometry.py -v
```

