#!/usr/bin/env python3
"""Read-only offline reproduction of the live map05 request (no code changes)."""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from PIL import Image

sys.path.insert(0, "/home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/lib/python3/dist-packages")
from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.uniform_pipeline import UniformPlanningRequest, plan_uniform
from fwis_v3_optimizer.uniform_search import GraphSearchConfig

map_dir = Path(sys.argv[1])
meta = yaml.safe_load((map_dir / "map.yaml").read_text())
img = np.asarray(Image.open(map_dir / meta["image"]).convert("L"))
img = np.flipud(img)  # map_publisher.py flips like ROS map_server
occ_thresh = float(meta.get("occupied_thresh", 0.65)) * 255.0
free_thresh = float(meta.get("free_thresh", 0.196)) * 255.0
data = np.full(img.shape, -1, dtype=np.int8)
data[img <= 255.0 - occ_thresh] = 100
data[img >= 255.0 - free_thresh] = 0
occupied = data != 0
origin = meta["origin"]
grid = Grid(occupied, float(meta["resolution"]), float(origin[0]), float(origin[1]))
print("grid %dx%d res=%.9f origin=(%.6f, %.6f) occupied=%d" % (
    grid.width, grid.height, grid.res, grid.ox, grid.oy, int(occupied.sum())))

model = FormationModel([VehicleOffset(1, 1.0, 0.0),
                        VehicleOffset(2, 0.0, 1.0),
                        VehicleOffset(3, 0.0, -1.0)])
shapes = {vid: VehicleShape(0.738, 0.500) for vid in model.vehicle_ids}

start = FormationPose(-5.7336816875, 9.638438875, 0.0)
goal = FormationPose(3.0888566875000008, 3.4107641875000003, 1.570796)
request = UniformPlanningRequest(
    grid=grid, model=model, vehicle_shapes=shapes,
    start_qf=start,
    start_headings={vid: 0.0 for vid in model.vehicle_ids},
    goal_qf=goal, goal_headings=None, plan_id=900001,
    formation_id="cargo_wedge",
    search_config=GraphSearchConfig(max_seconds=float(sys.argv[2]) if len(sys.argv) > 2 else 30.0))

from fwis_v3_optimizer.uniform_search import SuccessorGenerator, goal_heuristic_s
_orig = SuccessorGenerator.generate
def _logged(self, state, goal_qf):
    h = goal_heuristic_s(state, goal_qf, self.config,
                         GraphSearchConfig())
    print("POP qf=(%.2f, %.2f, %.3f) g=%.2f h=%.2f f=%.2f" % (
        state.qf.x, state.qf.y, state.qf.psi, state.g, h, state.g + 3.0 * h),
        flush=True)
    return _orig(self, state, goal_qf)
SuccessorGenerator.generate = _logged

t0 = time.perf_counter()
result = plan_uniform(request)
wall = time.perf_counter() - t0
search = result.search
out = {
    "status": result.status,
    "reason": result.reason,
    "first_failure_layer": result.first_failure_layer,
    "detail": result.detail,
    "wall_s": result.wall_time_s,
    "expansions": None if search is None else search.expansions,
    "candidate_count": None if search is None else search.candidate_count,
    "accepted_successors": None if search is None else search.accepted_successors,
    "collision_s": None if search is None else search.collision_s,
    "generation_s": None if search is None else search.generation_s,
    "open_size": None if search is None else search.open_size,
    "duplicate_key_rejections": None if search is None else search.duplicate_key_rejections,
    "rejection_histogram": None if search is None else dict(search.rejection_histogram),
    "terminal_audits": None if search is None else len(search.terminal_audits),
    "t_end_s": None if result.plan is None else result.plan.duration_s,
    "path": None if search is None or search.status != "SUCCESS" else [
        {"index": record["index"], "primitive": record["primitive"],
         "mode": record["mode"], "delta_psi": record["delta_psi"],
         "pivot": record["pivot"],
         "parameter_length": record["parameter_length"],
         "end_qf": record["end_qf"],
         "cumulative_proxy_g_s": record["cumulative_proxy_g_s"]}
        for record in search.as_dict(include_path=True)["path"]],
    "terminal_audit_summary": None if search is None or not search.terminal_audits else {
        "accepted": search.terminal_audits[-1].get("accepted"),
        "failure_code": search.terminal_audits[-1].get("failure_code"),
        "audit_wall_time_s": search.terminal_audits[-1].get("audit_wall_time_s")},
}
print(json.dumps(out, indent=2, sort_keys=True))