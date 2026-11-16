# G0 Commands

## Main invocation

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- interpreter: `/usr/bin/python3`
- Python: `3.8.10 (default, Mar 18 2025, 20:04:55)  [GCC 9.4.0]`
- environment increments: `PYTHONDONTWRITEBYTECODE=1`, `OPENBLAS_NUM_THREADS=1`
- regression level: `full`
- output: `/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy`
- exit code: `1`

```bash
/usr/bin/python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tools/virtual_a_baseline.py --output /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy --regression full
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
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_continuous/FR04/pipeline_results.json",
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
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/geometry_formation/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_formation_geometry.py -v
```

### timing_continuous

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `FAIL` / `1`
- stdout: `logs/timing_continuous.stdout.log`
- stderr: `logs/timing_continuous.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_continuous/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_continuous_retiming.py -v
```

### timing_formation

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/timing_formation.stdout.log`
- stderr: `logs/timing_formation.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/timing_formation/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_formation_retiming.py -v
```

### fr01_uniform_motion

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/fr01_uniform_motion.stdout.log`
- stderr: `logs/fr01_uniform_motion.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr01_uniform_motion/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_uniform_motion.py -v
```

### fr02_a_timing

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/fr02_a_timing.stdout.log`
- stderr: `logs/fr02_a_timing.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_timing/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_uniform_timing.py -v
```

### fr02_a_trajectory

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/fr02_a_trajectory.stdout.log`
- stderr: `logs/fr02_a_trajectory.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_a_trajectory/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_uniform_trajectory.py -v
```

### fr02_b_quality

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/fr02_b_quality.stdout.log`
- stderr: `logs/fr02_b_quality.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr02_b_quality/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_uniform_quality.py -v
```

### fr03_uniform_search

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/fr03_uniform_search.stdout.log`
- stderr: `logs/fr03_uniform_search.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr03_uniform_search/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_uniform_search.py -v
```

### fr04_uniform_pipeline

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- status / exit: `PASS` / `0`
- stdout: `logs/fr04_uniform_pipeline.stdout.log`
- stderr: `logs/fr04_uniform_pipeline.stderr.log`
- environment increment:

```json
{
  "FR01_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR01/primitive_results.json",
  "FR02_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR02/artifacts",
  "FR02_QUALITY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR02/B/uniform_quality_results.json",
  "FR02_TIMING_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR02/A/timing_results.json",
  "FR02_TRAJECTORY_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR02/A/trajectory_results.json",
  "FR03_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR03",
  "FR03_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR03/successors.json",
  "FR04_ARTIFACT_DIR": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR04",
  "FR04_RESULTS_PATH": "/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/regression_artifacts/fr04_uniform_pipeline/FR04/pipeline_results.json",
  "OPENBLAS_NUM_THREADS": "1",
  "PYTHONDONTWRITEBYTECODE": "1"
}
```

```bash
/usr/bin/python3 -B -m unittest discover -s tests -p test_uniform_pipeline.py -v
```

