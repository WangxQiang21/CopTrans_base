# G0 Commands

## Main invocation

- cwd: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- interpreter: `/usr/bin/python3`
- Python: `3.8.10 (default, Mar 18 2025, 20:04:55)  [GCC 9.4.0]`
- environment increments: `PYTHONDONTWRITEBYTECODE=1`, `OPENBLAS_NUM_THREADS=1`
- regression level: `baseline`
- output: `/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G0/20261115T140936Z`
- exit code: `0`

```bash
/usr/bin/python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tools/virtual_a_baseline.py --output evaluation/virtual_formation_a/G0/20261115T140936Z --regression baseline
```

The reference audit ran in this process. No historical verification or experiment script was executed.

## Regression subprocesses

None. G0 baseline does not run old unit tests, FR06 searches, ROS, or Gazebo.

## Post-run verification

All commands below used cwd `/home/wq/whut_thesis/expv3_multi_virtual_a` and `/usr/bin/python3`.

### Saved manifest verification — exit 0

```bash
python3 -B tools/virtual_a_baseline.py --verify-manifest evaluation/virtual_formation_a/G0/20261115T140936Z/protected_after.json
```

Output: `logs/manifest_verification.log`; status `PASS`, 3,120 protected files, no mismatches.

### Existing nonempty output refusal — expected exit 1

```bash
python3 -B tools/virtual_a_baseline.py --output evaluation/virtual_formation_a/G0/20261115T140936Z --regression baseline > /tmp/virtual_a_g0_overwrite_refusal.log 2>&1
```

Output: `logs/output_overwrite_refusal.log`; `OUTPUT_EXISTS_NONEMPTY` was returned and the existing run was not overwritten.

### Tampered manifest rejection — expected exit 1

A copy was made only under `/tmp`; its first protected-file hash and tree hash were replaced with zeroes. No repository evidence was altered.

```bash
python3 -B -c "import json; from pathlib import Path; src=Path('evaluation/virtual_formation_a/G0/20261115T140936Z/protected_after.json'); d=json.loads(src.read_text()); d['protected_files'][0]['sha256']='0'*64; d['protected_tree_sha256']='0'*64; Path('/tmp/virtual_a_g0_bad_manifest.json').write_text(json.dumps(d))"
python3 -B tools/virtual_a_baseline.py --verify-manifest /tmp/virtual_a_g0_bad_manifest.json > /tmp/virtual_a_g0_bad_manifest.log 2>&1
```

Output: `logs/bad_manifest_verification.log`; status `FAIL`, identifying the tree hash and `catkin_ws/src/CMakeLists.txt` mismatch.

### Added-file hash verification — exit 0

A read-only Python check recomputed every SHA-256 in `source_manifest.json:new_file_sha256`; initial result was 13 files checked with `mismatches: []`. After these three verification logs were added, the manifest was refreshed and the same check was rerun; the final console result was 16 files checked with no mismatches.
