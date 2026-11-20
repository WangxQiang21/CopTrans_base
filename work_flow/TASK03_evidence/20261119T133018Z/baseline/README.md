# TASK03 Step 0 baseline evidence

- UTC capture: 2026-11-19T13:30:18Z (directory name) through 13:39Z.
- Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
- Branch: `feature/virtual-rigid-formation-a`; HEAD `a3c709e57560b1060dcc422777f4131a3e558acf`; upstream `origin/feature/virtual-rigid-formation-a`.
- Starting status: two modified TASK01 files and four untracked TASK02/TASK03 review documents; no production source was changed in Step 0.
- TASK01 dirty files are byte-identical before and after Step 0:
  - `geometry.py` `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
  - `tests/test_virtual_a_geometry.py` `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`
  See `source_hashes_start.txt` and `source_hashes_end_step0.txt`.

## Old-default dry-run

`run_demo_dry_run.log` records `./run_demo.sh --map map05 --scale 2.625 --dry-run`, exit 0, with the old default launch still using `backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard`. No ROS master was started by the dry-run.

## Frozen run identities

`S1` is the TASK02 live S1 failure (plan 600000, `CORRIDOR_AUDIT_WIDTH_LIMIT`).
`MAP05_straight` is the TASK02 live map05 default success (plan 600000, start
`(2,2,0)`, goal `(8,2,0)`).

| Run | request SHA-256 | grid SHA-256 | planning-context SHA-256 |
|---|---|---|---|
| S1 | `591b4100355ab991d3b7dd5afeb95e203343ec185bf7187d025e2662b4afdd3b` | `6e94c751a40ec94431b495fcbff002945f09c975429c493c89060f0e7e2aa2a4` | `8a545c3f4c794f9f04c57f453c454612c54ca2d3fe31e4c6b887037d2d36604b` |
| MAP05_straight | `e69d068b1cae75fa7205a7530b129f16bf731dddce705e6d2ef8042fec07aeaf` | `64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321` | `06aed480a711839483c94e42648563b53c48eb0080f3c9dfe4e16daebe9414dd` |

Both identities were reconstructed from the retained raw ROS request fields
in each run's TASK02 `capture_summary.json`, the frozen runtime map copy, the
captured canonical planner context, and this worktree's current contracts.
All checks pass:

- request payload -> exact request SHA;
- runtime map files with ROS float32 resolution -> exact grid SHA;
- request + grid + old planner/timing/collision configs -> exact planning-context SHA;
- the captured context's own canonical digest -> exact planning-context SHA.

`S1_identity.json` / `MAP05_straight_identity.json` record the checks,
observed and reconstructed hashes, and source file SHA-256 values.
`*_request_reconstructed.json` is the exact canonical request payload that was
hashed. `*_planning_context.json` is the captured canonical context. Run
`reproduce_run_identity.py` (commands in `step0_s1_identity.log` and
`step0_map05_identity.log`) to repeat the reconstruction without ROS.

Notes:

- The runtime grid resolution is float32 (`0.05000000074505806` for S1,
  `0.06487162411212921` for map05). Reconstructing `Grid` with the plain
  YAML float64 resolution changes the canonical grid summary and explains the
  earlier naive hash mismatch; the script deliberately reproduces the ROS
  `OccupancyGrid` float32 value. The occupancy SHA and dimensions already
  matched.
- `frozen_inputs/G4_S1_offline_fixture.json` is the unrelated offline G4_S1
  plan-9001 fixture that was previously mis-copied as `S1_request.json`. It is
  retained under its correct name and is not used as S1 request identity.
- `frozen_inputs/S1_identity_naive_attempt.json` is the superseded naive
  reconstruction with float64 resolution and the wrong request fixture.

## Old results

`frozen_inputs/step0_old_results_summary.json` extracts, without rerunning:

- S1: `GEOMETRY_FAILED` / `CORRIDOR_AUDIT_WIDTH_LIMIT`, first failure at
  `s=7.757789718176485 m`, corridor 0, width `9.682634525276512e-06 m`,
  point residual `-4.691596815820098e-06 m`, bound residual
  `+2.061278807859711e-07 m`, max point residual `+2.850886833886124e-04 m`;
  both corridor rounds and both F-only containment records are retained.
- MAP05_straight: `SUCCESS` / executable, model
  `e6b6e55db62346d041e959b1955e031d33bd571995e53483b46c1c962635bcad`,
  geometry/dynamics/original-map OBB audits all passed, terminal at
  `(8,2,0)` with `S=6.0 m`, `T=14.203125 s`.
- Raw old quality reports, planning-debug JSONL and capture summaries are
  frozen byte-for-byte under `frozen_inputs/`.

## User plans 600001/600002

`user_plans_reproducibility.json` marks both `NOT_REPRODUCIBLE`. The current
user ROS master/log from 2026-11-19 records start/goal and failure summary,
but no raw `/formation/plan_request` message, request SHA, grid/context SHA,
runtime map copy, scale or matching capture was retained for those plans.
Plan numbers are not identities, and no replay or fabricated target run is
claimed. `user_plans_600001_600002_planner.log` is a byte copy of the observed
planner log used for the recorded lines.