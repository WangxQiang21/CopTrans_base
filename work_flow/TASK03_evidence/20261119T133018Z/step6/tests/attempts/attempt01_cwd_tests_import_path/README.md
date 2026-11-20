# Attempt 01 — offline checks run with cwd=`tests/`

UTC window: 2026-11-19T16:56:53Z–17:06:47Z.

`run_step6_checks.sh` (first version) ran every command with the working
directory set to `tests/` and
`PYTHONPATH=<root>/tests:<root>/catkin_ws/src/fwis_v3_optimizer/src`.

Groups 1–5, `catkin_make`, `bash -n` and the dry-run all exited 0.  The
combined regression group (`python3 -m unittest <all tests/test_virtual_a_*>`)
ran **539 tests with 2 import errors**:

```
ERROR: test_virtual_a_corridor_optimizer (unittest.loader._FailedTest)
ModuleNotFoundError: No module named 'tests'

ERROR: test_virtual_a_formation_corridor (unittest.loader._FailedTest)
ModuleNotFoundError: No module named 'tests'
```

Both modules import helpers as `tests.test_virtual_a_optimizer` /
`tests.test_virtual_a_corridor_optimizer`.  With cwd=`tests/` the repository
root was not on `sys.path`, so those two modules never loaded and their 80
tests did not run.  This is a harness working-directory/`PYTHONPATH` defect,
not a product or test-code failure; the same modules pass when the combined
suite runs from the repository root.

The first version of the script and all of its logs/exit codes/timestamps
are preserved byte-for-byte here.  The frozen script was then corrected to
run from the repository root with the root on `PYTHONPATH`; the corrected
run's artifacts live in the parent directory.