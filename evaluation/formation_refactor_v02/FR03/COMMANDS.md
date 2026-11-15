# FR03 commands

Worktree: `/home/wq/whut_thesis/expv3_multi_v02`
Interpreter: system `python3` (not `-I`, see note), `OPENBLAS_NUM_THREADS=1`.

Every command below was run from the worktree root.  No command uses the
original repository `/home/wq/whut_thesis/expv3_multi`, no worktree was
created, and no `git commit`/`reset`/`stash`/`clean` was executed.

## 1. Byte-compile

```bash
python3 -m py_compile \
  tests/test_uniform_search.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py
```

exit code 0.

## 2. FR03 acceptance suite

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest tests.test_uniform_search -v \
  2>&1 | tee evaluation/formation_refactor_v02/FR03/logs/test_uniform_search.log
```

result: `Ran 10 tests ... OK`, exit code 0, wall time ~129 s.
Cases S01-S10 correspond to the implementation-plan section 7 acceptance
list.  The suite writes `successors.json`, `state_key_samples.json`,
`arc_step_audit.json`, `successor_fr02_audit.json` and
`measure_200_states.json`.

## 3. Targeted regression suites

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest -v \
  tests.test_uniform_motion tests.test_uniform_timing \
  tests.test_uniform_trajectory tests.test_uniform_quality \
  tests.test_formation_collision tests.test_formation_quality \
  tests.test_formation_retiming tests.test_formation_geometry \
  tests.test_formation_hold tests.test_formation_kinematics \
  2>&1 | tee evaluation/formation_refactor_v02/FR03/logs/regression_suites.log
```

result: `Ran 145 tests ... OK`, exit code 0, wall time ~9.7 s.

## 4. Baseline hashes (accepted FR01/FR02 sources and frozen docs)

```bash
python3 evaluation/formation_refactor_v02/FR03/verify_baseline.py
```

result: `all_accepted_match=true`, `all_frozen_match=true`,
`unexpected_changes=[]`.  Expected hashes are copied from the independent
Codex acceptance records, not recomputed from the current tree.

## 5. Evidence manifest

```bash
python3 evaluation/formation_refactor_v02/FR03/make_manifest.py
```

result: writes `evidence_manifest.json` for every FR03 evidence file.

## Notes

* `python3 -I` cannot be used: isolated mode hides the user site-packages and
  the suite then fails with `ModuleNotFoundError: scipy`.  This matches the
  FR02 run convention.
* The synthetic 200-state measurement is a local per-state successor
  measurement on an empty map.  It is **not** a planning-success experiment,
  a global performance claim, or a benchmark against FR04.