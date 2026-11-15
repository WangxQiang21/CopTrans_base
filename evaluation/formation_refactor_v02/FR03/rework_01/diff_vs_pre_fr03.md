# FR03-R1 rework diff against the pre-rework FR03 files

Worktree: `/home/wq/whut_thesis/expv3_multi_v02`
Base HEAD: `6227833011e2d92b711179bfdec979bb0b878b29` (unchanged; no commit)

Only the two FR03 files were modified.  FR01/FR02 accepted sources/tests,
frozen documents, old tests/thresholds, pivot/10 deg library and safety
parameters are byte-identical to the Codex-recorded hashes
(`baseline_verification.json`).

| Path | Pre-rework bytes / SHA-256 | Post-rework bytes / SHA-256 | Delta |
| --- | --- | --- | --- |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py` | 37230 / `cbfd5597abed18e899e7c18f327a1fc184fb0d45e952934abc7cb4f955e5f77a` | 42224 / `982b2f96e1ab6dfcb3008d1e7232432bd861ebe8c8b9376e65bfbd96f8f96e83` | +4994 |
| `tests/test_uniform_search.py` | 47550 / `65f2721c6660fe8a2acefe822aad9830cfa7e5934204facd37850e791ecef4cd` | 68203 / `87f3f032dc3732e59dd1c19282118d8ddbfb727a4d049609316ca6b462f997e5` | +20653 |

The pre-rework copies are preserved in `pre_rework/` with the hashes saved in
`pre_rework_hashes.json` before any edit.

## Semantic diff

`uniform_search.py`:

* module contract now documents the finite `a_labels` domain, input-state
  rule, goal dependence and the fact that the key never merges different
  capability sets;
* `UniformSearchState.a_labels` and `UniformStateKey.a_labels` (included in
  `as_dict()` and in `make_key`);
* new helpers `goal_direction`, `gear_label`, `a_arc_schemes` and
  `a_capability_labels`; the generator's `_goal_direction` and
  `_a_arc_schemes` now delegate to the same helpers so candidate generation
  and labels can never drift;
* `initial_state(..., goal_qf=None)` computes the labels; every accepted
  `consider()` successor computes the labels for the child exact headings;
* no change to FR01 edge construction, collision sweep, cost, bins, mode,
  wheel labels or any threshold.

`tests/test_uniform_search.py`:

* `state_from_edge` takes the goal (labels are goal-dependent) and computes
  `a_labels`; goal-aware `initial_state` calls;
* new S11 (positive + mirror small-angle S alignment, real generator,
  RepresentativeTable retention, subsequent A, 3-edge FR02 audit, duplicate /
  worse-`g` dedup) and S12 (finite label domain/consistency, reverse, nonzero
  pivot, nonzero psi, mode/wheel key distinction, no snapping);
* successors.json contract documents `a_labels`.

## Git working-tree note

The worktree still carries the uncommitted, independently accepted FR01/FR02
changes relative to HEAD.  This rework did not touch those paths, so
`git status --porcelain` shows the same tracked modifications plus the same
untracked FR01/FR02/FR03/evidence files as before, with only the two files
above changed in content and the new `FR03/rework_01/` directory added.
No commit, reset, stash, clean or branch operation was performed.