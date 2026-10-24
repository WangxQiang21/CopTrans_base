# Branch method

- Branch: `continuous-mode-path`
- Default backend: `mode_path_v2`
- Core idea: Continuous mode-path candidates with continuous-motion retiming and independent certification.
- Closest paper reference: Runjiao Bao et al., “From Multi-Modal Paths to Executable Trajectories: A Trajectory Planning Framework for 4WIS Robots,” arXiv:2608.29108.

This is a nearest-concept mapping for development and comparison, not a claim that the current code fully reproduces the paper. The repository keeps all backends and shared ROS infrastructure on this branch. Select another backend explicitly with `planning_backend:=...`.

## Optimization and validation (2026-10-25)

The continuous-mode search already keeps the fastest certified incumbent, but it could expand three structural neighborhoods after a valid route existed. On map04, the third expansion did not improve the incumbent: it spent extra solve time evaluating another route family while retaining the same selected route. The post-feasibility structural expansion limit is now two; the empty-pool recovery path still has its full search budget.

The five-case ROS regression compared the committed baseline settings with this limit change. Each case retained the same selected source, score, and executable duration. All passed the 5 mm swept-footprint, terminal pose, direction, independent dynamics, and process-cleanup checks.

| Case | Search + optimization before | After | Certified trajectory before/after |
|---|---:|---:|---:|
| map04 (`P12_env1_default`) | 12.770 s; 3 neighborhoods | 9.252 s; 2 neighborhoods | 40.446 s / 40.446 s; score unchanged |
| map03 (`P11_case_c_default`) | 0.711 s; 3 neighborhoods | 0.664 s; 2 neighborhoods | 3.884 s / 3.884 s; score unchanged |
| map09 (`P16_narrow_maze_default`) | 3.788 s; 2 neighborhoods | 3.721 s; 2 neighborhoods | 29.766 s / 29.766 s; score unchanged |
| map05 (`P12_env2_default`) | 2.129 s; 2 neighborhoods | 2.120 s; 2 neighborhoods | 15.080 s / 15.080 s; score unchanged |
| map01 (`P11_case_a_default`) | 0.854 s; 2 neighborhoods | 0.795 s; 2 neighborhoods | 4.674 s / 4.674 s; score unchanged |

The main gain is map04: **3.518 s (27.6%) less optimizer time**, with the selected continuous route and its score unchanged. Runs and per-case audits are saved in `result/branch_optimization_20261025/continuous-mode-path-expansions2/`. Reproduce with:

```bash
python3 tools/run_quality_regression.py --cases P12_env1_default P11_case_c_default P16_narrow_maze_default P12_env2_default P11_case_a_default --cases-file tools/expv3_eval_cases.yaml --backend mode_path_v2 --port-base 13540 --output-dir result/branch_optimization_20261025/continuous-mode-path-expansions2
```
