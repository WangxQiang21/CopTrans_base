# Stray GUI input during the visible RViz round (plan 600002)

While I was re-calibrating the zoomed RViz view with `xdotool` and taking
screenshots, a goal was published from RViz that I did not intend to click
(`/move_base_simple/goal` at wall 1794651452.01).  The goal bridge admitted it
as plan 600002 and the planner honestly failed it at the frozen 30 s budget.
This file preserves that observation; it is *not* one of the controlled
ACCEPTANCE probes.  The intended round was rejected while this job was
PLANNING (`refusing ... job 600002 is PLANNING`) and was redone afterwards as
plan 600003 once the job was FAILED.

Exact lines from `logs/run_demo_rviz.log` (this launch only):

```
110:[INFO] [1794651452.014616]: [formation goal bridge] plan 600002: start=(11.681, 9.670, 0.000) goal=(12.908, 6.172, -1.524) vehicles=3
111:[INFO] [1794651452.015212]: [uniform planner] plan 600002 admitted from cargo_wedge: start=(11.681, 9.670, 0.000) goal=(12.908, 6.172, -1.524) free_headings=True reverse=True
113:[WARN] [1794651464.991954]: [formation goal bridge] refusing 2D Pose Estimate: job 600002 is PLANNING; the start override and plan_id are not consumed and no request is published
114:[WARN] [1794651466.620645]: [formation goal bridge] refusing 2D Pose Estimate: job 600002 is PLANNING; the start override and plan_id are not consumed and no request is published
115:[WARN] [1794651478.237801]: [formation goal bridge] refusing 2D Pose Estimate: job 600002 is PLANNING; the start override and plan_id are not consumed and no request is published
116:[WARN] [1794651480.063994]: [formation goal bridge] refusing 2D Nav Goal: job 600002 is PLANNING; the start override and plan_id are not consumed and no request is published
117:[ERROR] [1794651482.197639]: [uniform planner] plan 600002 not executable: status=BUDGET_EXHAUSTED reason=TIME_BUDGET_EXHAUSTED layer=budget wall=30.180s
118:[WARN] [1794651482.200288]: [uniform planner] plan 600002 has no executable envelope (TIME_BUDGET_EXHAUSTED)
119:[WARN] [1794651482.200330]: [formation goal bridge] plan 600002 status=REJECTED reason=TIME_BUDGET_EXHAUSTED
120:[ERROR] [1794651482.200758]: [formation playback] rejecting playback plan 600002: model_sha256 must be 64 hex digits
```

Observed behaviour (all fail-closed, defaults unchanged):

* the accidental far goal (heading -1.524 rad) was admitted and honestly
  exhausted the frozen budget after 30.180 s: `status=BUDGET_EXHAUSTED
  reason=TIME_BUDGET_EXHAUSTED layer=budget`; no path was fabricated;
* no executable envelope was published; playback rejected the invalid
  placeholder (`model_sha256 must be 64 hex digits`) and cleared;
* while the job was PLANNING, the bridge refused further 2D Pose Estimate and
  2D Nav Goal clicks instead of silently consuming them (normal busy
  rejection);
* the search-level detail of this accidental round was only printed to the
  interactive terminal and is not preserved in a file, so it is not quoted
  here.  The controlled near-goal budget negative (`ros_near_goal/`, goal
  (10.1, 10.0, 0.1), 97 expansions / 17013 candidates) is the recorded one.

After the job reached FAILED the intended zoomed round was replayed with real
`xdotool` clicks and admitted as plan 600003 (see `run3_meta.json`,
`run3_08.png`, `run3_45.png`).