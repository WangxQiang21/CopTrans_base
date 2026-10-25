# Archived source

`exp_legacy_packages/` preserves the old `fwis_msgs`, `fwis_planner_hybrid_astar`, and `fwis_backend_qp` ROS packages from `exp`. They are retained for reference and comparison; the active `catkin_ws` does not build them.

`expv2_plan/` preserves the offline spatial-search prototype, its specifications, scripts, papers index, and historical results. It is not connected to `run_demo.sh`. Its S09 map and scenario defaults now point to this repository's `maps/` and `scenarios/`; PDF aliases point to the canonical copies in `papers/`.

Historical result files keep the original run metadata, including absolute paths from the machine where they were generated. They are evidence of those earlier runs, not inputs to the active demo.

The active ROS workspace is the repository's `catkin_ws/`, including the copied `fwis_common` and `fwis_map_server` dependencies. The original `exp` and `expv2` directories were left untouched.
