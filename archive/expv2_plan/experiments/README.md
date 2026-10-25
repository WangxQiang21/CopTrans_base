# Spatial planning prototype and simulation runs

Current implementation follows the user-confirmed perfect-tracking planning scope. The detailed math and discretization are in [`specs/ranger_spatial_model.md`](specs/ranger_spatial_model.md). `src/spatial_model.py` defines ideal A (Ackermann), P (oblique/diagonal), L (lateral), and S (spin) motion, plus a raster swept-footprint collision checker. `src/lattice_planner.py` is a discrete-control A* prototype. It is an experimental baseline, not the Chang MIP implementation.

## Assumptions

- Body reference point is the rectangle center, with footprint 0.738 × 0.500 m from the Ranger manual.
- The manual's 0.810 m minimum Ackermann radius is applied at this planning reference point provisionally; the manual does not identify its reference point.
- A uses forward/reverse constant-curvature arcs with `|κ| ≤ 1/0.810 m⁻¹`; the steering command cap from the manual is recorded separately and is not converted to individual wheel angles.
- P translates at relative angles -45°, 0°, +45°; L is the ±90° fixed-heading translation; S rotates in place. Transitions are instantaneous under perfect control.
- Spatial primitive length is 0.25 m; XY search bins are 0.20 m and yaw bins are 15°. Occupied cells are inflated by one 8-connected cell neighborhood; swept collision samples translation at ≤1 cm and yaw at ≤1°, and rasterizes the footprint at roughly 3 cm spacing with 1.5 cm extra body margin for cache quantization. These discretizations need resolution sensitivity runs before research claims.
- `heuristic_weight=1` is ordinary A* with an admissible Euclidean lower bound (and yaw lower bound); weights above 1 speed search and give up optimality guarantees, consistent with coarse-search use.

## Run

```bash
python3 scripts/check_spatial_model.py
python3 scripts/run_geometry_validation.py
python3 scripts/run_s09_pair.py
python3 scripts/run_s09_goal_sweep.py --all-pairs
python3 scripts/run_fixed_sequence_refinement.py
```

Runs snapshot the scene and vehicle specification and write JSON/CSV plus figures under `results/raw/<run_id>/`. Deterministic propagation/collision sanity outputs are in `results/summary/spatial_model_checks.csv`.

The coarse-search pilot is documented in [`results/summary/phase1_spatial_pilot.md`](results/summary/phase1_spatial_pilot.md). The fixed-mode safe-corridor refinement and its role in the architecture are documented in [`results/summary/固定模式连续优化_阶段结果.md`](results/summary/固定模式连续优化_阶段结果.md); the latest comparison chart is [`coarse_vs_refined.png`](results/raw/s09_fixed_refine_20260930_202503/coarse_vs_refined.png).

The S09 map and preset definitions are read from this repository's `maps/S09_underground_garage/` and `scenarios/S09_underground_garage.yaml`. These scripts remain a standalone research prototype and are not wired into the active ROS demo. A successful simulation means only ideal geometric feasibility under the saved map and discretizations. It does not establish control tracking or physical vehicle feasibility.
