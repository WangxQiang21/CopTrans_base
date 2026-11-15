# FR06 — narrow-corner single-scene autonomous search

Status: implemented and executed on 2026-11-14; evidence frozen below.
No Git commit was made. This report is handed to Codex for acceptance and
covers only the one frozen scene in §1.

## 1. Frozen scene, reference and inputs

The scene is a constant-width distance-band corridor around one cargo
centreline inside a 16 m x 16 m, 0.05 m grid (320 x 320 cells, origin
(0, 0)), independent of every historical map.

| element | value |
| --- | --- |
| formation | 3 vehicles, offsets (1, 0), (0, 1), (0, -1); body 0.738 m x 0.500 m |
| start | centre (6, 6), psi = 0; body headings 0, 0, 0 |
| goal | centre (10, 10), psi = pi/2; goal body headings left free for search |
| centreline | (4,6)->(8,6), quarter arc centre (8,8) r = 2 m, (10,8)->(10,12) |
| widths | wide 3.2 m, tight 2.8 m; 2.6 m analysed only, never searched |
| search budget | 10000 expansions / 30 s, `GraphSearchConfig()` default |
| collision margin | 0.05 m (`SuccessorConfig()` default) |

Frozen artefacts (`scene_frozen.json`, `make_scene.py` for
`maps/corridor_w{2.6,2.8,3.2}`):

| width | map.pgm sha256 | scene_meta sha256 | free / occupied cells | free section (h / v / arc) |
| --- | --- | --- | --- | --- |
| 3.2 | `da227385cb41afc8178d3a467deea56514816b6cc521234ab4d96b2643382f12` | `f809663cdd4955565ba99d0446d683396d3878edabad61636f176c50c2ddda02` | 17489 / 84911 | 3.20 / 3.20 / 3.19 m |
| 2.8 | `b59dc888714540e4eb04c2dec4a6a53f40f186d820048069e7458f138483362f` | `c1e85f2a53685569c05f9d3db96593065c6ff531c04d553c1e96c5f2b373ea7b` | 14954 / 87446 | 2.80 / 2.80 / 2.80 m |
| 2.6 | `45c2dc36f94c5e851aa234cb9084796f5e3980df6659ee2f9b0af2d46b1817d0` | `d0c69fb97e3db2f48f35c78f75f6ae78921da6ba242a267aa112d1a740c3cf9c` | 13717 / 88683 | 2.60 / 2.60 / 2.60 m |

The frozen reference sequence is built only from FR01 factories:
`O(2 m +x)` -> `S` (vehicle 1 aligns by atan2(1,2), vehicles 2 and 3 hold) ->
`A(+pi/2, pivot (0,2))` -> `O(2 m +y)`.
It is audited in `reference/` and is never imported by the search runner:
`run_experiments.py` reads only the grid, `START_QF`, `GOAL_QF`, the start
headings and (for bookkeeping) the frozen goal headings and envelope width.

Reference model: T_end = 29.955294753 s, model sha256
`1da16fa1bdbe93e1bd3e55855b8fd900b6c4770f66a7ccbe8ae7eb368a5affde`,
model JSON sha256 `65926c313ecc22b8bf97788c0887b00ca6c7247511b6d1b0dadb601208ea788e`,
dense projection (0.02 s, 1499 samples) sha256
`df7764139f9a2e08c2b216207c6fef520844827019322b26626f9c057bffecdb`.
Reference phases: O 2 + S 1 + A 1, 4 motion blocks, 4 setup dwells
(0.3 s initial + 7.02212723 s inter-edge), motion 22.633167523 s.

## 2. FR02 audit of the frozen reference

`freeze_reference.py` ran `audit_uniform_timed_plan` on the reference plan
over each frozen grid. The audit is the FR02 independent check; its result,
not the planner's own claim, is what is reported here.

| width | feasible | reason | checks | min clearance to band | after 0.05 m margin | width / 2.5 m envelope |
| --- | --- | --- | --- | --- | --- | --- |
| 3.2 | yes | OK | 13390 | 0.329119232 m | 0.279119232 m | 1.28 |
| 2.8 | yes | OK | 13390 | 0.129119232 m | 0.079119232 m | 1.12 |
| 2.6 | **no** | `FORMATION_SWEEP_COLLISION` | 11266 | 0.029119232 m | -0.020880768 m | 1.04 |

The 2.6 m result is a reference rejection by the 0.05 m collision margin,
not a proof that the environment is unsolvable; 2.6 m was never searched.
The tight reference clearance witness is at t = 19.27 s, vehicle 3 at
(10.728674799, 6.196393934).

## 3. Search protocol

* Inputs to `plan_uniform`: the frozen grid for the width, the frozen
  formation model and vehicle shapes, start pose (6, 6, 0) with headings
  (0, 0, 0), goal pose (10, 10, pi/2), and `goal_headings=None`
  (goal headings are free). No reference edge, waypoint, sample or schedule
  is passed. The only use of reference-derived values outside audit and
  bookkeeping is the static end-heading set for the deliberately
  unacceptable collision-goal probe in §8; that probe never reaches search.
* Three attempts per width, same process, same `SuccessorConfig()`,
  `GraphSearchConfig()`, timing limits and budget; plan ids 6101-6103
  (wide) and 6201-6203 (tight).
* `run_experiments.py` fix set: `config_sha256` =
  `3020fe9a3a08383839cd824a19e3fb64dc56383467e5b1473dcb201b44c26229`
  (canonical JSON of the successor config, search config, margin and budget).
* Every attempt is saved in `runs/`; successes also save the plan model in
  `plans/`. Nothing is discarded.

Host: Linux-5.15.0-139-generic x86_64, Python 3.8.10, AMD Ryzen 7 7840H.

## 4. Results — six autonomous attempts

| attempt | status | wall s | expansions | candidates | accepted | collision s | T_end s | plan clearance | ratio |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| wide 3.2 #1 | `BUDGET_EXHAUSTED` | 30.272 | 10 | 1832 | 476 | 29.543 | - | - | 1.28 (ref) |
| wide 3.2 #2 | `BUDGET_EXHAUSTED` | 30.315 | 10 | 1832 | 476 | 29.595 | - | - | 1.28 (ref) |
| wide 3.2 #3 | `BUDGET_EXHAUSTED` | 32.769 | 11 | 2000 | 476 | 31.990 | - | - | 1.28 (ref) |
| tight 2.8 #1 | `SUCCESS` | 8.637 | 6 | 1024 | 48 | 5.490 | 33.024494 | 0.129119232 m | 1.12 |
| tight 2.8 #2 | `SUCCESS` | 8.592 | 6 | 1024 | 48 | 5.436 | 33.024494 | 0.129119232 m | 1.12 |
| tight 2.8 #3 | `SUCCESS` | 8.644 | 6 | 1024 | 48 | 5.424 | 33.024494 | 0.129119232 m | 1.12 |

All three tight attempts returned the same status, route and metrics; only
wall time and request fingerprint differ (plan ids 6201, 6202, 6203).
This is one tight scenario succeeding autonomously, with an executable plan
(the same corridor was replayed in ROS in §7).

## 5. First-cause attribution

**Wide 3.2 — `first_failure_layer = budget`.** The search stopped at
`TIME_BUDGET_EXHAUSTED` (detail: "wall time >= 30 s"). In all three attempts
the collision sweep consumed the budget: 29.543 s, 29.595 s and 31.990 s of
the 30 s limit, with 10, 10 and 11 expansions and 476 accepted successors.
Rejection histograms: 1272/1272/1392 `A_HEADING_REQUIRES_S_ALIGNMENT` and
82/82/85 `MAP_OBB_COLLISION`; no terminal audit was reached. The extra
free space of the wide corridor increases the successor sweep cost per
expansion (1832-2000 candidates versus 1024 in the tight corridor), so the
30 s budget runs out. This is a search-budget outcome, not evidence that the
wide environment is physically infeasible. No budget, primitive family or
threshold was changed to force a different outcome.

**Tight 2.8 — success.** Six expansions, 48 accepted successors, 18698
audit checks, T_end 33.02449442722614 s. The returned route is
`O(1 m) -> O(1 m) -> S -> A(+pi/2) -> O(1 m) -> O(1 m)`, i.e. the same
centreline geometry as the corridor allows but a different block
decomposition from the 4-block reference (`O(2 m) -> S -> A -> O(2 m)`).
The search was not forced toward the reference result; the longer T_end
(33.024 vs 29.955 s, +10.2%) comes from the two extra 1 m blocks and their
inter-edge setup dwells.

## 6. Success-plan metrics (tight 2.8 m, attempt 1; #2/#3 identical apart from plan id and wall clock)

| quantity | value |
| --- | --- |
| model sha256 | `94b2d77e40aa48e3911d4b469e9eb91d15b35731352d7786b4af97533c7b35df` |
| request sha256 | `655819ad54932214c03333ab978c18ac7eef02d07477ef35a1d6b107fb02e377` |
| T_end | 33.02449442722614 s |
| modes | A 1, O 4, S 1 (6 motion blocks) |
| setup | 4 total: 0.3 s initial + 7.02212723 s inter-edge |
| motion | 25.702367197 s; S block 2.111407542 s |
| ordinary block stops | 5 (four O blocks + one A block; each motion block starts/ends at zero speed) |
| spin blocks | 1 |
| formation envelope | 2.500000000 m lateral; width ratio 1.12 |
| plan minimum clearance | 0.129119232 m; 0.079119232 m after the 0.05 m margin |
| clearance witness | t = 21.56 s, vehicle 3 at (10.991810349, 6.677981909) |
| audit checks | 18698, status FEASIBLE, margin 0.05 m, static collision checks 32 |
| max rigidity error | 1.5543122344752192e-15 m |
| max interior geometry error | 2.6645352591003757e-15 m or rad |
| plan model JSON | `plans/tight_2.8m_attempt1_plan6201.json` (6202/6203 same geometry) |

Phase table (attempt 1): setup 0-0.3; O 0.3-3.423; O 3.423-6.546; setup
6.546-8.889; S 8.889-11.000; setup 11.000-14.031; A 14.031-25.131; setup
25.131-26.779; O 26.779-29.902; O 29.902-33.024.

## 7. Real ROS/RViz synchronous replay

One replay was run on the tight 2.8 m map with the V0.2 uniform backend:
private master 11483, dedicated `ROS_HOME`, `rviz:=true`, `playback_rate:=100`.

* auto goal from `scene_meta`: start (6, 6, 0), goal (10, 10, 1.571);
* plan 600000 published at T_end = 33.024 s, 1659 shared samples,
  planner wall 9.231 s, model digest
  `a39818b8125ce7d3cd212e16108f7e91b849c952f534dfcf3b559e4bac0cc967`;
* playback state sequence: `PLAYING` at elapsed 0.261073 s ->
  `HOLD_END` at elapsed 33.045216 s, clamped 33.024494 s,
  `executable=True`, "busy released";
* 30 screenshots at 22:28:37-22:29:15 local show the three vehicles leaving
  the approach, vehicle 1 spinning, the formation sweeping the arc, and the
  formation parked at the exit goal; frame crops are in
  `rviz/round1/frames/`, metadata in `rviz/round1/run_meta.json`.

Caveats recorded in that metadata: the live node's plan carries plan_id
600000 and the launch-derived formation name `cargo_wedge`, so its model
digest differs from the offline plans 6201-6203 (`fr06_three_vehicle`);
equality of T_end and of the route is observed, hash equality is not
claimed. The `playback_rate:=100` parameter did not accelerate the observed
replay, which lasted 33.045 s wall. The RViz overlay's inherited caution
text ("no independent metrics reported") is the FR05 display note, not a
failure of this plan.

## 8. Explicitly unacceptable goal

Goal (12, 10, pi/2) with the reference end headings was rejected before
search: `REQUEST_REJECTED`, reason `MAP_OBB_COLLISION`,
`first_failure_layer = request`, detail "goal static check failed:
vehicle_id=1" (`runs/collision_goal.json`, plan id 6299). This is the
intentionally unacceptable collision target required by the task.

## 9. Scope and honesty constraints

* Results apply only to this 16 m x 16 m static corridor; no global or
  whole-map success rate is claimed, and no result from other maps is used.
* Wide 3.2 failure is a wall-clock budget outcome at the budget layer, not a
  feasibility result. Tight 2.8 success is one scenario, not a rate.
* The 2.6 m width is analysis only (reference rejected by 0.05 m margin);
  no search was claimed there.
* All six search attempts and the rejected goal are stored; nothing was
  hidden, overwritten across runs, or retro-fitted. The search never
  received a reference seed.
* No FR01/FR05 file, frozen plan/decision, old test expectation, vehicle
  parameter, safety threshold or Codex evidence was modified. No Git
  commit was made.

## 10. Evidence and hashes

| file | sha256 |
| --- | --- |
| `fr06_scene.py` | `3e7a2879d4938c048955adbe47b050c6a1a890646b6a2c9b005b238098697613` |
| `make_scene.py` | `ba679f8a4066d66c8f385393b1b3b9cb69795e2e1327c26e2037b9ded8561266` |
| `scene_frozen.json` | see `SOURCE_MANIFEST.sha256` |
| `experiment_summary.json` | see `SOURCE_MANIFEST.sha256` |
| `reference/reference_summary.json` | see `SOURCE_MANIFEST.sha256` |

`SOURCE_MANIFEST.sha256` lists every FR06 source, map, frozen JSON, run,
plan, log, node log, RViz metadata and screenshot; `sha256sum -c` passes
with exit 0 (106 entries, Chinese-locale "成功" output). The manifest
excludes itself, `logs/manifest_check.log` (which is its own verification
output) and the ROS caches/pids under `ros_home/`. Reproduction steps are
in `COMMANDS.md`.