# FR03-R1 rework commands (all run from the V0.2 worktree root)

Worktree: `/home/wq/whut_thesis/expv3_multi_v02`
Python: system `python3` (not `-I`; `scipy` is required from the user site)
Environment: `OPENBLAS_NUM_THREADS=1`

```sh
# 1. Save pre-rework snapshot and hashes before editing (done first).
#    Result: pre_rework/ (both files + pre-rework evidence copies),
#            pre_rework_hashes.json.

# 2. Compile.
python3 -m py_compile \
  tests/test_uniform_search.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py
# exit 0

# 3. Full FR03 suite (new evidence kept out of the pre-rework top level).
FR03_RESULTS_PATH=evaluation/formation_refactor_v02/FR03/rework_01/successors.json \
FR03_ARTIFACT_DIR=evaluation/formation_refactor_v02/FR03/rework_01 \
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.test_uniform_search -v \
  > evaluation/formation_refactor_v02/FR03/rework_01/logs/test_uniform_search.log 2>&1
# exit 0; "Ran 12 tests in 157.173s - OK"

# 4. Codex probe, copied verbatim from
#    FR03/codex_acceptance_20261113T164549Z/key_compatibility_probe.py.
cd evaluation/formation_refactor_v02/FR03/rework_01
OPENBLAS_NUM_THREADS=1 python3 key_compatibility_probe.py \
  > logs/key_compatibility_probe.log 2>&1
# exit 0; both signs PASSED (same_key=false, kept_child=true)
cd /home/wq/whut_thesis/expv3_multi_v02

# 5. Ten targeted regression suites.
OPENBLAS_NUM_THREADS=1 python3 -m unittest -v \
  tests.test_uniform_motion tests.test_uniform_timing \
  tests.test_uniform_trajectory tests.test_uniform_quality \
  tests.test_formation_collision tests.test_formation_quality \
  tests.test_formation_retiming tests.test_formation_geometry \
  tests.test_formation_hold tests.test_formation_kinematics \
  > evaluation/formation_refactor_v02/FR03/rework_01/logs/regression_suites.log 2>&1
# exit 0; "Ran 145 tests in 9.696s - OK"

# 6. Baseline + pre/post rework hash verification.
python3 evaluation/formation_refactor_v02/FR03/rework_01/verify_baseline.py
# exit 0; all_accepted_match=true, all_frozen_match=true,
# unexpected_changes=[], both reworked files changed.

# 7. Evidence manifest (run last).
python3 evaluation/formation_refactor_v02/FR03/rework_01/make_manifest.py
```

The earlier one-off S11/S12 development runs are not part of the acceptance
evidence; their temporary `successors_dev.json` was removed before the final
suite.  No command above writes to the FR03 top-level pre-rework evidence.