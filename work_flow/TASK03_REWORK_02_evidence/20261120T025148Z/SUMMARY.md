# TASK03 rework 02 evidence summary

**Outcome: `BLOCKED_DESIGN_DECISION` — E09 `NOT_REPRODUCIBLE`**
Baseline HEAD `a3c709e5…` on `feature/virtual-rigid-formation-a`.
No production or test source was modified (`source.diff` is empty); the
protected files and all historical evidence trees match the start hashes.

## 1. What was run

Thirty deterministic plan/grid/obstacle parameter combinations were frozen
before execution and each was run through the **full production**
`plan_virtual_a` pipeline with `formation_corridor_scope=three_vehicle_union_v1`.
The scanner only wrapped three production functions to record their real
inputs/outputs (`certify_formation_path`, `assess_formation_guide`,
`make_route_guides`); it never altered a returned object, template, allocation,
certificate or audit.

- `scan/combos.json` – frozen manifest of all 30 combinations
- `scan/batch_0{1..4}_*.json` – per-batch frozen parameters
- `scan/scan_e09.py` – production-pipeline scan driver
- `scan/s01..s30.log` – one `SUMMARY` JSON per run
- `scan/*.assessments.json` / `*.route_sets.json` – captured production data
- `scan/scan_matrix.json` / `.md` – consolidated 30-row matrix
- `scan/commands.json` – exact argv, exit code and elapsed time per run

## 2. Failure layer (Step 1)

| Layer | Combinations |
|---|---|
| Route generation (`NO_ROUTE_GUIDE`) | 1 (s01) |
| Single-region chain (no adjacent region) | 2 (s04, s06) |
| Two adjacent audited regions, then **assessment `COLLISION_OBSERVED`** | 27 |
| Template construction reached | **0** |
| NLP attempt / capture | **0** |
| Continuous certificate | **0** |
| Migration gate (`f_only_gate_migrated`) | **0** |

For all 27 two-region scenes the chain was built and `audit_corridor_chain`
passed; the allocation cut sat between 0.459 and 0.936. Every run then stopped
at `TEMPLATE_BUILD_FAILED` because at least one assessment station reported
`map status='COLLISION_OBSERVED'`. Instrumented runs recorded 13–190 colliding
stations each. No scene reached template candidates, so no template could be
accepted and no old-audit failure on a final optimized path exists to show.

## 3. Witness verification (independent)

`scan/witness_s30.json` recomputes one witness **without production collision
code**: at `sigma=0.3171` (`s_m=6.4077`) the formation pose is
`(6.3872, 0.5115, 0.0799)`; vehicle 1 (offset `(1.0, 0.0)`, OBB
`0.738 x 0.500`) penetrates the frozen 0.5 m wall by **0.1077 m** in bare-OBB
SAT. The assessment rejection is a genuine three-vehicle collision, not a
margin artefact. Vehicles 2 and 3 are clear at that station.

## 4. Why this path is blocked

The synthetic three-vehicle formation spans ±1.0 m laterally and +1.0 m
longitudinally around the F reference. Every obstacle layout that forced a
real two-corridor A* route also brought the leading vehicle's OBB into the
obstacle during the required turn — a real safety violation that the plan
forbids relaxing. With 30/30 combinations exhausted, the plan's stop condition
is reached: no contract-compliant E09 scene was found within the frozen
budget, and manufacturing one would require changing the objective, the
allocation cut definition, or the region semantics.

## 5. Required decision (not chosen by Claude)

1. Keep allocation, improve initial value / objective so a legal adjacent
   template can be found by the optimizer.
2. Define the switch time by actual optimizer-path region crossings.
3. Keep the legacy F-only hard gate and drop gate migration/E09 for now.

## 6. Integrity

- 19 TASK03 source files: `sha256sum -c` all OK against `start/`.
- `geometry.py` and `test_virtual_a_geometry.py`: protected SHAs unchanged.
- `evaluation/` git tree, TASK01/02/03 and REWORK_01 evidence trees unchanged.
- `git status --porcelain` still 32 pre-existing entries; nothing staged,
  committed or pushed.
- No ROS/RViz run was made in this rework because no positive E09 input exists
  to run; the earlier rework-01 live evidence is untouched.