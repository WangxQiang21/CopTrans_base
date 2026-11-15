# FR03-R1 rework report: finite A-capability labels in state and key

Status: **rework complete, stopped for independent Codex acceptance.**
Only FR03-R1 was reworked.  No Git commit, no FR04, no ROS/messages/NLP, no
FR01/FR02 change, no library pivot/10 deg step or safety parameter change was
performed.

## 1. What was wrong (accepted finding)

The pre-rework key used only a 10 deg heading bin per vehicle (plus the gear
sign).  Two exact states in the same bin could therefore share a key even
when only one of them satisfied an A scheme's required heading set.  A legal
S alignment of 0.01 rad to 0 rad (all-S, same canonical wheel set, same mode
and direction) produced exactly the same key as its parent, and the lower-`g`
representative rule discarded the child plus the newly unlocked
`A_straight_goal_gear+++` capability.  Both mirror signs reproduced this.

## 2. Fix

`uniform_search.py` now records a finite **A-scheme capability label set**
`a_labels` on every exact state and includes it in `UniformStateKey`:

* A label is present **iff** the exact headings satisfy the FR01
  required-heading set of that configuration scheme under the current search
  goal, using the same helpers, the same `1e-9` tolerance, the same
  `allow_reverse` rule and the same gear combinations as the successor
  generator.  The label therefore corresponds to a candidate the generator
  really emits (verified in S12 against the real generator on an empty map).
* The label domain is finite and bounded: the goal-directed straight scheme
  (one per gear combination) plus the configured fixed/cropped arc schemes
  (`+/-10 deg` per pivot, plus the goal-cropped angle when it is non-zero;
  both signs only at exactly pi) times the gear combinations.  With the
  current library that is at most `8 + 20 x 8 = 168` labels after
  deduplication; the implementation returns a sorted, deduplicated tuple.
  No continuous angle, no finer bin and no unbounded retention is used.
* Labels are **goal-dependent** (the straight direction and the crop angle
  depend on the goal pose and the exact `q_F`), so a representative table is
  valid for one goal at a time.  This is documented in the module and in the
  report contract.
* Input-state rule: `initial_state(..., goal_qf=...)` computes the labels for
  that goal.  With `goal_qf=None` the goal-directed straight and cropped-arc
  labels are omitted (they cannot be evaluated); fixed arc labels remain
  exact.  Omission only **splits** keys, it never merges states with different
  capabilities.
* Geometric duplicates of the same `(pivot, delta)` scheme keep the first
  label, matching the generator's dedup rule.  Labels that correspond to
  different successor schemes (e.g. a `+10 deg gear+++` fixed arc and a
  `+90 deg gear+++` crop that share the same required heading set for a pure
  rotation) are kept separately because they are different candidates.
* Exact state is untouched: no pose or heading is snapped, the XY/psi bins
  are unchanged, and only the key gains the label field.  Mode and wheel
  labels are unchanged and still distinguish states (S12).

Public API stays compatible: `make_key(state, config)` and
`RepresentativeTable(config)` take the same arguments, so Codex's probe runs
verbatim.

## 3. FR03-R1 tests (S11) and evidence

`rework_r1_small_angle.json` (`fr03.rework_r1_small_angle.v1`).  Both signs:

1. Legal prefix S `0.02 -> 0.01` rad (mirror `-0.02 -> -0.01`), all-S,
   canonical S wheel set, direction `-1` (mirror `+1`).
2. The parent has `a_labels = []`; the real generator emits **no** direct
   `A_STRAIGHT` and rejects every straight with
   `A_HEADING_REQUIRES_S_ALIGNMENT`.
3. The real generator emits the S alignment to exactly `0` rad; its cost is
   `0.0125 s` (setup 0 + 0.01 rad / yaw rate, proxy).  The child is `MODE_S`
   with the same direction and `a_labels = ["A_straight_goal_gear+++"]`.
4. Parent and child keys have **identical xy/psi/heading bins** but differ in
   `a_labels`, so `parent_key != child_key`.
5. Offering the parent first and then the generator child: both accepted,
   `len(table) == 2`, and the child is still the representative for its own
   key.  An exact duplicate and a `g+5` worse state with the same labels are
   still rejected (bins and labels identical), so normal deduplication is
   intact.
6. The retained child generates `A_straight_goal_gear+++` (setup 2.3432 s for
   the S-to-A mode change); the 3-edge plan
   `(prefix S, alignment S, straight A)` audits **FEASIBLE** in FR02
   (duration 8.9846 s in both cases); continuity sup <= 1e-7.

## 4. Label domain / consistency test (S12) and evidence

`rework_r1_label_domain.json` (`fr03.rework_r1_label_domain.v1`) compares
`state.a_labels` with the labels of the A successors the real generator
accepts on an empty map, for four states:

| state | `a_labels` | generator accepted A |
| --- | --- | --- |
| initial, goal straight | `A_straight_goal_gear+++` | same |
| reverse-aligned (S to anti-tangent) | `A_straight_goal_gear---` | same |
| nonzero pivot crop, pivot `(0,2)`, +90 deg | `A_arc_crop_p0_2_pos_gear+++`, `A_arc_p0_2_pos_gear+++`, `A_arc_p0_2_neg_gear---` | same |
| nonzero cargo `psi=0.4`, yawed start | `A_straight_goal_gear+++` | same |

The several pivot labels reflect the authoritative FR01 rule that the
required heading set depends only on the sign of `delta_psi`: the `+90 deg`
crop, the `+10 deg` fixed arc forward and the `-10 deg` fixed arc reverse share
one required set for a pure rotation at this pivot.  They are kept as
distinct labels because they are distinct candidate schemes.

The test also checks that changing only the mode or only a wheel control by
`0.02` (two 0.01 bins) still changes the key, that `key.a_labels` equals the
state labels, that pose `(0.3, -0.2, 0.4)` and headings `0.5404 rad` survive
exactly (not a bin centre), and that a goal-absent input state has an empty
label set by the documented rule.

## 5. Codex probe re-run

Codex's `key_compatibility_probe.py` was copied **verbatim** into this
directory and run with the same worktree and Python.  Exit code 0; both signs
report `same_key=false`, `kept_child=true`, alignment cost `0.012499...`,
FR02 `FEASIBLE`, status `PASSED` (`key_compatibility_probe.json`,
`logs/key_compatibility_probe.log`).  The original failing
`codex_acceptance_20261113T164549Z/key_compatibility_probe.json`/`.log` were
not modified; their hashes are recorded in `baseline_verification.json`.

## 6. Regression, original suite, measurement

* Original S01–S10 still pass; the full suite is now **12 tests, all
  passing** (`Ran 12 tests in 157.173s - OK`, exit 0).
* Ten targeted suites: `Ran 145 tests in 9.696s - OK`, exit 0.
* Accepted FR01/FR02 sources/tests and the three frozen documents still match
  the Codex-recorded SHA-256 values (`all_accepted_match=true`,
  `all_frozen_match=true`, `unexpected_changes=[]`).
* Fixed-seed 200-state measurement (seed 20261114, same goal, empty map;
  `measure_200_states.json`, `fr03.measure_200_states.v1`): 200/200 walk
  states, 0 fallbacks, 33567 candidates (mean 167.835, median 184, min 144,
  max 185), 11663 accepted successors, 21904 rejections all
  `A_HEADING_REQUIRES_S_ALIGNMENT`.  Generation 32.573 s total
  (0.163 s/state), collision 108.756 s total, `tracemalloc` peak 3.62 MB
  (3792378 bytes).
  Generation time is higher than the pre-rework 9.309 s because every
  accepted successor now also computes its finite capability labels; the
  collision sweep is unchanged.  This remains a local per-state measurement
  and is not a planning-success or global-performance claim.

## 7. What this does and does not prove

* The fix closes the specific key/capability inconsistency: a generated S
  alignment that unlocks an A scheme is no longer deduplicated by a same-bin
  lower-`g` parent, while pure duplicates and worse-`g` states are still
  removed.
* `RepresentativeTable` remains a bounded discrete approximation, not a
  strict dominance theorem over arbitrary exact states in one bin.
* The capability label set covers the heading-dependent **A** schemes, which
  is what FR03-R1 demonstrated.  Under the default `allow_reverse=True` an O
  branch is feasible for every heading and an S spin is unconditional, so A
  is the only heading-dependent successor class.  Under the non-default
  `allow_reverse=False`, O branch legality would also depend on the exact
  heading; the minimal R1 label set does not cover that residual case and it
  is not exercised by the FR03 default.  It is recorded here rather than
  silently generalized.
* Successors are still **geometric/kinematic** candidates: the generator
  emits them after the FR01 legality check and the real analytic OBB sweep,
  but it does **not** run the full FR02 dynamic audit per expansion.  The
  three-edge FEASIBLE results above come from the separate acceptance audit;
  search "fail-closes" must always be read as *final FR02 rejection*, not as
  expansion-time dynamic rejection.
* The default `10 deg` fixed-arc FR02 limitation is unchanged and only
  recorded: under accepted default limits the conservative S8/S9 bound
  returns `UNIFORM_DYNAMIC_UNCONFIRMED` (jerk) for the fixed `+/-10 deg`
  nonzero-pivot arcs, while the exact model peaks and the goal-cropped angle
  path are fine.  This rework does not modify FR02, the limits or the step.
* The 20001-point exact-model sampling in S04 is evidence, not a strict
  continuous certificate.

## 8. Files, hashes, constraints

| file | pre-rework bytes / SHA-256 | post-rework bytes / SHA-256 |
| --- | --- | --- |
| `.../uniform_search.py` | 37230 / `cbfd5597...f77a` | 42224 / `982b2f96...6e83` |
| `tests/test_uniform_search.py` | 47550 / `65f2721c...f4cd` | 68203 / `87f3f032...f997e5` |

The pre-rework snapshot and hashes were saved before editing
(`pre_rework/`, `pre_rework_hashes.json`).  The FR03 top-level pre-rework
evidence was not overwritten (hash-checked in `baseline_verification.json`);
all new evidence is in this `rework_01/` directory.  No commit, reset, stash,
clean or branch operation was performed, and the original repository was not
touched.