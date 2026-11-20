# TASK03 Step 6 evidence — explicit union scope, HUD/RViz and live regression

UTC window: live runs 2026-11-19T16:45Z–16:56Z; offline checks
16:56Z–17:1xZ (this directory).
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`.
HEAD unchanged: `a3c709e57560b1060dcc422777f4131a3e558acf`
(`feature/virtual-rigid-formation-a`, in sync with origin, 27 dirty/untracked
entries inherited from TASK01/TASK02/TASK03).

Step 6 wires the versioned formation corridor scope explicitly through the
whole chain and proves it with a real ROS/RViz run, an E04 negative and an
f_only comparison.  The default scope stays `f_only` until Codex accepts the
delivery.

## Production files touched in Step 6

| File | SHA-256 at end of Step 6 |
|---|---|
| `formation_continuous_a/contracts.py` | `6261931e745e49fdca286b8d06c9c891b4d5a0b3523359e3d9e550189b49faa7` |
| `formation_continuous_a/corridor.py` | `5b32983bf8b70e59de8416441efdd69af4eca422d6d46281057e37047c431210` |
| `formation_continuous_a/formation_corridor.py` (new) | `ba8de6435004226ed3dad4372f0eb5b926c851409f0142e8dce087b961b3dc54` |
| `formation_continuous_a/formation_certificate.py` (new in Step 4, unchanged here) | `27557c391df2dcd323fe20c6f5c829981a1c282e67a8a5d71dc46cf5d81ebf42` |
| `formation_continuous_a/optimizer.py` | `c667388dd578dd435afbff4d785f48510ef3460d913b34672bfa7356f4296a2d` |
| `formation_continuous_a/pipeline.py` | `87f6ffe03517ea1776a857e822a40d682f799111c09dacc814a09ed4ae1468c8` |
| `formation_continuous_a/ros_adapter.py` | `9b38c846590fd018a857b564e618075d3ff20fa2a1846a4a440bdc639415d558` |
| `formation_playback.py` (optimizer verifier/display) | `9876c9066f02be55922a92244ddc9bbaacb4a1afccd21a95026baf1b7562c61c` |
| `scripts/formation_uniform_planner_node.py` | `9b37c6e4c25d6c0d740836f8b5acb30960a061ad5bf333c027a2ab3d314a54d1` |
| `fwis_v3_msgs/msg/FormationPlaybackPlan.msg` | `acc33896e7d734f8fbd0081c3928218fb48803a72fd2ad4dad65790be47b6f06` |
| `fwis_v3_playback/scripts/formation_playback.py` | `de53eef3fdd3b7ecda86b8178a84cc508e01d73318f87408c621de79ee5e286b` |
| `fwis_v3_demo/launch/formation_demo.launch` | `fe13c13b449239f27d34a0169350820e0b8acb74d591edd41e440fc9cfd473a5` |
| `run_demo.sh` | `d307b2fc039a658efcddf2ff8892ddd5d8afdde1a9f7352aaff4667e3cb0e8fa` |
| `tests/test_virtual_a_pipeline.py` | `cbacb8272faf3804ea7f809ba12abcef59aa9de1d886e407223d192020eaa98f` |
| `tests/test_virtual_a_playback.py` | `80a354768feaf6b5aef134d411b35bdbfe9e34ca611fd9a88d75a85dfdb266cd` |
| `tests/test_virtual_a_receiver.py` | `d17128f43f16bf0f48b1f28b28040f4981e7fb0df700eeacfb21d74e7fedfa26` |
| `tests/test_formation_demo_assets.py` | `136b43c6fe708e9baccc41c2934589f59cd81e7c005db308a2bba4087f419ad2` |
| `tests/test_virtual_a_corridor_optimizer.py` | `8205af95e412ed58be402dd5f575a7f0990c5b992a3309cf84a5675ca7123d30` |
| `tests/test_virtual_a_formation_corridor.py` (new) | `8ffa807ad3c5844bfd6eca4fc6b8ffe141bdfcd171daed6aefd7cd2ba6367078` (unchanged in Step 6) |

Protected TASK01 dirty files are byte-identical:
`geometry.py` `f31b20bb…`, `tests/test_virtual_a_geometry.py` `4f59e68f…`
(asserted again by `tools/make_step6_manifest.py`).

## Scope chain and binding transport

```
run_demo.sh (dedupe, default f_only, run_demo.sh:415-452)
  → formation_demo.launch arg/param (default f_only, L87/L130-131)
  → planner node ~virtual_a_formation_corridor_scope (L2482-2483)
  → PlannerConfig/planning_context_sha256 (contracts.py L847-882)
  → pipeline.plan_virtual_a() union_scope (L1332-1402)
  → finite templates + CasADi corner rows → solver → candidate
  → certify_formation_path() → optimizer gate → result certificate
  → envelope.formation_binding_json (planner node L1914-1941)
  → ros_adapter.verify_envelope_fields(expected_formation) (L916/L993)
  → playback formation_playback.py L732-763
  → verify_playback_plan(expected_formation)
  → _verify_union_formation_binding() (optimizer formation_playback.py L1017)
```

Binding keys are exactly
`{"scope","planning_context_sha256","template","chain","certificate"}`.  The
field travels inside the same atomic `FormationPlaybackPlan` envelope (new
msg field `formation_binding_json`); a second topic would race with the
latched envelope.  f_only/legacy/uniform envelopes carry an empty string.
The receiver parses it with `parse_constant` rejecting NaN/Infinity, requires
a JSON object, rebuilds the template and corridor chain, checks the grid and
re-verifies the certificate.

Planner-side `verify_envelope_fields()` performs the same binding
verification before publishing; only then is the envelope published.

## HUD/RViz vocabulary

- published, independently audited plan → HUD `audit FEASIBLE` / `audit
  NO_MOTION`; never `CERTIFIED`.
- `CERTIFIED` only on the final candidate markers after `PUBLISHED`.
- Guide layers (`formation/planning/astar_raw`, `formation/planning/portal_guide`)
  are `REFERENCE_ONLY`.
- diagnostics stay in separate namespaces:
  `formation/planning/portal_window`, `formation/planning/map_collision_sample`,
  `formation/planning/coverage_gap`.

## Live runs

Task-private master `http://127.0.0.1:11663`; each run has its own
`ROS_HOME`/`ROS_LOG_DIR`; RViz inputs are the same reset/goal publications a
user would make.  Driver exit code is in `capture_driver.exit`.

| Run | Launch arguments | Checks | Result |
|---|---|---|---|
| `union_map05_2625` | `--map map05 --scale 2.625 virtual_a_formation_corridor_scope:=three_vehicle_union_v1` | 29/29 | PASS |
| `negative_e04_map05_175` | `--map map05 --scale 1.75 virtual_a_formation_corridor_scope:=three_vehicle_union_v1` | 22/22 | PASS (negative) |
| `f_only_map05_2625` | `--map map05 --scale 2.625 virtual_a_formation_corridor_scope:=f_only` (explicit historical scope; the automatic default is proven by the dry-run's roslaunch line) | 28/28 | PASS |
| `union_map05_2625_attempt01_playback_binding_gap` | same as union | failed | preserved pre-fix failure |
| `union_map05_2625_attempt02_hud_word_only` | same as union | failed | preserved wrong-word assertion |

Union run `union_map05_2625`:

- plan `600000`, `SUCCESS`/`FEASIBLE`, `executable=true`, planning wall
  `11.460 s`, `T_end=14.2031250000003 s`, `S=6.0 m`; initial 1, routes 1,
  recoveries 0.
- request `e69d068b…`, context `9655b79f…`, grid `64eef089…`, playback model
  `e653238d…`.
- OBB rows `3888`; template `4e897ff7479e3a4c…`; chain `e7f8568f2397a95f…`.
- certificate `VERIFIED`: 24 checked / 16 verified intervals, depth 1,
  0 unclosed, `max_outward=-0.37498999716394005 m`, margin `1e-5`, guard
  `1e-12`, 16/16 cuts `CLOSED`, sha `86cce3e8e54c85e0…` (attempt01
  `888d9456…`, attempt02 `99ed13bf…`: the sealed payload includes the actual
  `limits.max_seconds` budget, so the hash legitimately differs between
  runs), F-only comparison `passed` with `max_bound_residual=-1.625 m`.
- audits geometry 155 / dynamics 909 / collision 3740 all passed.
- executable envelopes 1; binding length 11835; `union_binding_complete=true`;
  HUD `audit FEASIBLE`, `planning PUBLISHED | step 8/8`.
- screenshots `images/00_idle.png` … `images/live_11_34.0s.png`.

E04 negative `negative_e04_map05_175`:

- `NO_ACCEPTED_CANDIDATE` / `FORMATION_CORRIDOR_UNCONFIRMED`, layer
  `PLANNING`, corridor audit `TEMPLATE_BUILD_FAILED`, detail
  `assessment station sigma=0.5 map status='COLLISION_OBSERVED'`;
  5 samples / 3 map collisions / 12 unknown; wall `0.448 s`; initial 0.
- request `e5b6e715…`, context `0d87586b…`, grid `fcf003c3…`.
- no executable envelope, no playback plan for the id, no stale
  PLAYING/HOLD_END, no certificate, no certified candidate markers.
- guide diagnostics retain 6 red `MAP COLLISION (sample)` markers; HUD
  failure state; playback FAILED.

f_only comparison `f_only_map05_2625`:

- plan `600000`, `SUCCESS`, wall `6.434 s`, `T_end=14.203125000000316 s`,
  `S=6.0 m`; initial 5, routes 1, recoveries 0.
- request `e69d068b…` (same as union), context `06aed480…`, grid
  `64eef089…`, model `e6b6e55db62346d0…` — **byte-identical to the frozen
  Step 0 MAP05_straight historical model**, same terminal `(8,2,0)`.
- result has no `formation_certificate`, no union scope key; envelope
  binding length 0; audits 155/909/3740 passed.

## Offline checks

Each command has `.log`, `.exit`, `.start`, `.end`, `.wall_s.txt` in
`tests/`; aggregate in `tests/exit_codes.txt` and `tests/run_window.txt`.

| Group | Command | Result |
|---|---|---|
| 1 | `python3 -m unittest test_virtual_a_pipeline` | 15 tests OK, 138.979 s, exit 0 |
| 2 | `python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback` | 103 tests OK, 76.268 s, exit 0 |
| 3 | `python3 -m unittest test_virtual_a_corridor_display test_formation_hud` | 92 tests OK, 16.940 s, exit 0 |
| 4 | `python3 -m unittest test_formation_demo_assets` | 29 tests OK, 4.100 s, exit 0 |
| 5 | `python3 -m unittest test_virtual_a_receiver` | 24 tests OK, 17.591 s, exit 0 |
| 6 | `python3 -m unittest <all tests/test_virtual_a_*.py>` | 617 tests OK, 385.070 s, exit 0 |
| 7 | `catkin_make -C catkin_ws -j4` | all targets up to date/built, exit 0 |
| 8 | `bash -n run_demo.sh` | exit 0 |
| 9 | `./run_demo.sh --map map05 --scale 2.625 --dry-run` | exit 0; roslaunch line contains default `virtual_a_formation_corridor_scope:=f_only` |
| 10 | legacy/uniform: `test_uniform_playback_plan test_formation_playback test_cargo_transport test_transport_contract` | 65 tests OK, 1.431 s, exit 0 |
| 11 | legacy/uniform: `test_formation_regression test_uniform_pipeline` | 34 tests OK, 49.538 s, exit 0 |

### Harness attempt 01 (preserved)

The first script version ran with cwd=`tests/`.  Modules 1–5, build, syntax
and dry-run passed, but the combined group raised two
`ModuleNotFoundError: No module named 'tests'` import errors
(`test_virtual_a_corridor_optimizer`, `test_virtual_a_formation_corridor`
import helpers as `tests.test_virtual_a_*`), so 80 tests did not run.  The
failed artifacts are preserved under
`tests/attempts/attempt01_cwd_tests_import_path/`.  The script was corrected
to run from the repository root with the root on `PYTHONPATH`; the table
above is the corrected run.

## Deviations and known failures

- `FormationPlaybackPlan.msg` gained `formation_binding_json`; the message
  md5 changed and the workspace was rebuilt.  This is the only race-free
  transport for the sealed binding (see the chain above).  Legacy/uniform
  envelopes leave it empty and are covered by the supplementary groups.
- The capture driver itself was corrected during Step 6 and attempts 01/02
  preserve its pre-fix output: parameters are now read after `wait_ready`
  (attempt01 recorded `scope_param=f_only`), marker checks distinguish
  `REFERENCE_ONLY` guides from `CERTIFIED` candidates after `PUBLISHED`
  (attempt01's `candidate_markers_no_certified` no longer applies), and the
  executable envelope is selected from history rather than "latest".
- HUD assertions use the current vocabulary: an audited published plan shows
  `audit FEASIBLE`, not `CERTIFIED` (attempt02 preserved).
- All live runs use the task-private master `11663`, never `11411`/`11311`.
- The default scope remains `f_only` in both `run_demo.sh` and the launch
  file until Codex accepts the delivery.
- The live union vs f_only timing (`11.460 s` vs `6.434 s`) and initial
  counts (1 vs 5) are single-scene observations, not a performance claim.
- The supplementary legacy group `test_formation_regression` runs the FR04
  harness, which rewrites timing fields in
  `evaluation/formation_refactor_v02/FR04/cases/*.json` and
  `pipeline_results.json` (20 tracked files).  A recursive JSON comparison
  showed the only non-timing difference is `generated_at_utc`; the files were
  restored to HEAD and `git status --porcelain evaluation/` is empty.  See
  `tests/evaluation_side_effect_note.txt`.
- E09 (old F-only single-region gate failing while the audited union passes)
  was not constructed as a dedicated live or unit scenario; the migration
  branch is certificate-guarded and was not exercised in the live run
  (`f_only_gate_migrated=false`; the historical F-only containment audit
  passed as the comparison).  E02/E03 remain NOT_REPRODUCIBLE (missing
  original 600001/600002 inputs).

## Still open

Codex whole-delivery review.  No commit, no push, no TASK04, default scope
unchanged.