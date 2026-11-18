# Cargo HUD addendum — white Cargo hold text in the fixed corner

Run: `evaluation/virtual_a_corridor/V0/20261117T114406Z_cargo_hud`
UTC: 2026-11-17T11:44:06Z
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
HEAD: `9a19fd3e22d33188b13795f695757c3e4a36909f` (**not committed**)

This is a user-requested UI addendum to V0, not a new formal V0 acceptance
run.  The original V0 evidence in `../20261117T105425Z/` remains unchanged;
its real-HUD GUI gate was recorded NOT_RUN before `jsk_rviz_plugins` was
installed.

## 1. Request and behavior

Request: the white `Cargo` hold text should also appear in the fixed
top-right HUD.

- `hud_text(playback_status, job_state, quality, show_cargo_reason=True)`
  appends the cargo block only when the playback document state is `STATIC`
  (the static hold):
  - `Cargo` and, when the existing switch allows it, ` | reason`;
  - `STATIC / no executable trajectory`.
- The ROS playback node passes its existing `~show_cargo_reason` parameter to
  the formatter.  The interactive demo keeps that parameter `false`, so its
  HUD and its white map label both show `Cargo` plus the static warning
  without the reason.  The defaults of the library and the marker builder
  remain unchanged for old callers.
- Vehicle labels, plan/formation/backend identity, audit columns and the
  trajectory/corridor markers are untouched.
- The HUD default height is raised from 230 to 270 px so the two additional
  lines are fully visible.  Defaults are now 360x270 at left=900, top=20;
  launch/rviz/test assets use the same value.  Width/left/top are unchanged.

## 2. Files changed in this addendum

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py`
  — `hud_text` cargo block and optional `show_cargo_reason` argument.
- `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`
  — pass `self.show_cargo_reason`; default `~hud_height` 270.
- `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch` — default
  `hud_height` 270.
- `catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz` — HUD height 270.
- `tests/test_formation_hud.py` — Cargo block, reason switch and STATIC-only
  coverage (16 tests).
- `tests/test_formation_demo_assets.py` — HUD height assertions (21 tests).

## 3. Targeted tests (exit 0 for every module)

| module | tests | result |
| --- | --- | --- |
| test_formation_hud | 16 | PASS |
| test_formation_demo_assets | 21 | PASS |
| test_formation_playback | 22 | PASS |
| test_virtual_a_playback | 34 | PASS |

No full `unittest discover` was run.

## 4. Real ROS visual check (private master only)

- Private master `http://127.0.0.1:11585`; the user's session on 11411 was
  never contacted.
- Playback: `formation_playback_hudcheck`, backend `virtual_rigid_a`,
  `_hud_enabled:=true`, `_show_cargo_reason:=true`, `_transport_ui:=false`,
  `_publish_tf:=false`.  The direct C1 fixture published plan 92001, route 0,
  with the exact audited vertices.
- RViz: `catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz`, window
  1920x1043 (absolute upper-left 0,37); HUD render properties left=900,
  top=20, width=360, height=270.
- Captured `/formation/hud` message:
  `hud_message.yaml` has `height: 270` and text lines
  `Cargo | PLANNING` and `STATIC / no executable trajectory` between the
  compact vehicle labels and the three audit columns.  All 12 lines are
  visible in `screenshot_hud_cargo.png`; the two corridor regions and the
  three static bodies remain visible.
- `job PLANNING` in this fixture comes from the fixture's `/formation/job_state`
  record; there is no executable trajectory in this direct display session.
  In normal demo playback `_publish_tick` refreshes the same HUD while a
  trajectory exists.
- The white in-scene Cargo marker is deliberately retained; the HUD is an
  additional copy, not a replacement.

## 5. Caveats

- This addendum is not C2-B/V3 and does not create an autonomous
  planner-to-corridor display path.  The official `run_demo.sh` corridor
  layer stays empty until C2-B writes `result.search["corridor_displays"]`;
  the C1 saved-JSON fixture is the current way to see corridor geometry.
- No Git branch, commit or push was made.  The worktree remains uncommitted;
  `git_state.log` records the exact status at capture time.
- `jsk_rviz_plugins` is confirmed installed
  (`rospack find jsk_rviz_plugins` and
  `rosmsg show jsk_rviz_plugins/OverlayText` both exit 0), which is why this
  live HUD check could run.

## 6. Files

`hud_message.yaml`, `screenshot_hud_cargo.png`, `rviz_window.txt`,
`ros_topics.txt`, `show_cargo_reason_param.txt`, `test_logs/`,
`source_hashes.txt`, `git_state.log`, `cargo_hud.patch`, `commands.log`,
`results.json`.  `cargo_hud.patch` is the cumulative `git diff` from HEAD for
the touched files (it also contains the earlier uncommitted V0/V1 edits in
those same files), not an isolated addendum patch.