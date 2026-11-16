# G0 Commands

## Main invocation

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- interpreter: `/usr/bin/python3`
- Python: `3.8.10 (default, Mar 18 2025, 20:04:55)  [GCC 9.4.0]`
- environment increments: `PYTHONDONTWRITEBYTECODE=1`, `OPENBLAS_NUM_THREADS=1`
- regression level: `baseline`
- output: `/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G0/20261115T143321Z_rework`
- exit code: `0`

```bash
/usr/bin/python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tools/virtual_a_baseline.py --output evaluation/virtual_formation_a/G0/20261115T143321Z_rework --regression baseline
```

The reference audit ran in this process. No historical verification or experiment script was executed.

## Regression subprocesses

None. G0 baseline does not run old unit tests, FR06 searches, ROS, or Gazebo.

## Rework verification (R1/R2)

cwd `/home/wq/whut_thesis/expv3_multi_virtual_a`, interpreter `/usr/bin/python3`, environment `PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1`.

```bash
python3 -B evaluation/virtual_formation_a/G0/20261115T143321Z_rework/verify_rework.py
```

- exit code: `0`
- stdout: `logs/rework_verification.stdout.log`
- stderr: `logs/rework_verification.stderr.log` (empty)
- method: pure in-memory manifests plus mocked `subprocess.run` and `_write_text`; zero real child starts, zero old test cases, zero writes outside this run directory.
- the script writes its JSON evidence with exclusive-create (`open("x")`), so re-running it in place refuses to overwrite; reproduce it from a copy placed in a fresh run directory rather than by clearing this one.

Generated evidence:

| file | content |
|---|---|
| `stage_gate_cases.json` | 15 R1 positive/negative gate simulations |
| `runner_guard_cases.json` | 5 R2 containment/overwrite/mock cases |
| `original_evidence_preserved.json` | 15 original-run files unchanged; intentional rework hashes |
| `hashes.json` | current tool/interface SHA-256 vs Codex-reviewed versions |

The temporary `guard_probe/` fixture directories remain as evidence that only mocked children ran.

## Manifest re-verification

```bash
python3 -B tools/virtual_a_baseline.py --verify-manifest evaluation/virtual_formation_a/G0/20261115T143321Z_rework/protected_after.json
```

- exit code: `0`
- output: `logs/manifest_verification.log`

```bash
python3 -B tools/virtual_a_baseline.py --verify-manifest evaluation/virtual_formation_a/G0/20261115T140936Z/protected_after.json
```

- exit code: `0`; the original G0 run's protected tree is unchanged.
- output: `logs/original_run_manifest_verification.log`
