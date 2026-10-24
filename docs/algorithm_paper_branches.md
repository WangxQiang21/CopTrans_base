# Algorithm branches and paper references

This repository keeps the three runnable backends together because they share the ROS front end, vehicle model, maps, collision checks, and parts of the optimizer. Each paper branch selects one backend as its default so it can be developed and compared independently.

| Branch | Default backend | Closest paper idea | Reference |
|---|---|---|---|
| `multimodal-hybrid-astar` | `legacy` | Mode-augmented Hybrid A* with mode-aware search costs and terminal connections | Runjiao Bao et al., “Hybrid A* Path Planning with Multi-Modal Motion Extension for Four-Wheel Steering Mobile Robots,” arXiv:2509.06115. |
| `mode-path-constant-primitive` | `mode_path_v1` | Multi-mode sequence decisions followed by trajectory refinement and timing | Zeyu Yang et al., “A 4WIS Vehicle Trajectory Planning Approach Incorporating Multiple Motion Mode Sequence Decision-Making,” *Automotive Innovation* 9 (2026), 158–174. DOI: 10.1007/s42154-025-00369-5. |
| `continuous-mode-path` | `mode_path_v2` | Connecting multi-modal paths to executable trajectories with mode-consistent continuous motion | Runjiao Bao et al., “From Multi-Modal Paths to Executable Trajectories: A Trajectory Planning Framework for 4WIS Robots,” arXiv:2608.29108. |

These are nearest-concept references, not claims of complete paper reproduction. In particular, the current `mode_path_v2` does not yet implement a global mode-path-time mixed-integer optimization. The source retains all three backend implementations on every branch; future branch-specific work should keep changes focused on its selected backend and record deviations from the cited paper.

## Selecting a backend

The branch default is used by `bash run_demo.sh` when `planning_backend:=...` is omitted. To compare another backend without changing branches, pass it explicitly:

```bash
bash run_demo.sh --map map01 planning_backend:=legacy
bash run_demo.sh --map map01 planning_backend:=mode_path_v1
bash run_demo.sh --map map01 planning_backend:=mode_path_v2
```

The project comparison on 2026-10-25 ran all three backends on the same 11 preset cases. `mode_path_v2` passed 9/11, `legacy` 6/11, and `mode_path_v1` 5/11. Treat this as a baseline snapshot, not a fully normalized paper reproduction: legacy uses a different timing/dynamics contract and its default direction policy differs from v1/v2. See [`../results/backend_comparison_20261025.md`](../results/backend_comparison_20261025.md).

The paper PDFs are not included in this repository. Public identifiers and bibliographic details above locate the cited works.
