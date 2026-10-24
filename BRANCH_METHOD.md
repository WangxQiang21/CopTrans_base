# Branch method

- Branch: `multimodal-hybrid-astar`
- Default backend: `legacy`
- Core idea: Mode-augmented Hybrid A* search with mode-aware costs and terminal connections.
- Closest paper reference: Runjiao Bao et al., “Hybrid A* Path Planning with Multi-Modal Motion Extension for Four-Wheel Steering Mobile Robots,” arXiv:2509.06115.

This is a nearest-concept mapping for development and comparison, not a claim that the current code fully reproduces the paper. The repository keeps all backends and shared ROS infrastructure on this branch. Select another backend explicitly with `planning_backend:=...`.

## Optimization and validation (2026-10-25)

The planner configuration already selected `direction_policy: bidirectional`, but the legacy C++ frontend silently forced `forward_only`. The optimizer then independently rejected reverse seeds and any direction change, while legacy timed samples published positive speed for reverse primitives. This prevented the multimodal search extension from reaching execution.

This branch now applies the configured direction policy in the Hybrid A* frontend and legacy optimizer. When shunting is disabled, an initial reverse route remains eligible and one gear reversal is allowed only when an actual Spin lies between translation segments; repeated reversals remain forbidden. Legacy smoothstep timing now preserves the signed gear in published velocity samples. Other backends retain their prior candidate filters.

Final ROS regression used the frozen project preset starts/goals and the same legacy backend before and after. Every case below passed terminal pose, 5 mm swept-footprint, independent direction, and process-cleanup checks. Legacy timing remains `legacy_native_speed_only`; these durations should only be compared with legacy results under the same timing configuration.

| Case | Previous result | Updated result | Trajectory duration |
|---|---|---|---:|
| map02 (`P11_case_b_default`) | No path in discretized lattice | Pass; all-reverse route | 3.177 s |
| map09 (`P16_narrow_maze_default`) | No path in discretized lattice | Pass; one Spin-mediated reversal | 10.455 s |
| map04 (`P12_env1_default`) | Pass | Pass | 20.69 s → 20.543 s |
| map05 (`P12_env2_default`) | Pass | Pass | 10.04 s → 9.326 s |

The saved ROS logs and per-run JSON are in `result/branch_optimization_20261025/multimodal-hybrid-astar-final/`. Reproduce the final four-case run with:

```bash
python3 tools/run_quality_regression.py --cases P11_case_b_default P16_narrow_maze_default P12_env1_default P12_env2_default --cases-file tools/expv3_eval_cases.yaml --backend legacy --port-base 13500 --output-dir result/branch_optimization_20261025/multimodal-hybrid-astar-final
```
