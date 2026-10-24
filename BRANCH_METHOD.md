# Branch method

- Branch: `mode-path-constant-primitive`
- Default backend: `mode_path_v1`
- Core idea: Constant-action mode-path search followed by a mode-consistent segmented trajectory and trapezoidal timing.
- Closest paper reference: Zeyu Yang et al., “A 4WIS Vehicle Trajectory Planning Approach Incorporating Multiple Motion Mode Sequence Decision-Making,” Automotive Innovation 9 (2026), 158–174. DOI: 10.1007/s42154-025-00369-5.

This is a nearest-concept mapping for development and comparison, not a claim that the current code fully reproduces the paper. The repository keeps all backends and shared ROS infrastructure on this branch. Select another backend explicitly with `planning_backend:=...`.

## Branch optimization

The selected constant-primitive route is also converted into mode-consistent continuous segments. An endpoint fit and independent full-footprint swept/dynamics audit gate this trajectory. It replaces exact-primitive timing only when it is certified and faster; otherwise the exact-primitive result remains selected. This removes stationary reconfiguration at same-mode primitive boundaries while preserving stops at actual mode switches.

The 2026-10-25 paired regression improved `P12_env1_default` from 113.75 s to 46.53 s and `P16_narrow_maze_default` from 85.84 s to 32.04 s. Both remained certified. The added trajectory generation took 0.94 s and 0.24 s; total optimizer callback time changed from 7.00 s to 8.49 s and from 6.09 s to 6.41 s, respectively. Raw runs are under `result/branch_optimization_20261025/` in the working copy.
