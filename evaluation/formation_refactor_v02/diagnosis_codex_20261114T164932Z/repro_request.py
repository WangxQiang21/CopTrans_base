#!/usr/bin/env python3
"""Read-only offline reproduction of one live formation request (no repo edits)."""
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
sx, sy, spsi = (float(x) for x in sys.argv[2:5])
gx, gy, gpsi = (float(x) for x in sys.argv[5:8])
max_seconds = float(sys.argv[8]) if len(sys.argv) > 8 else 30.0

meta = yaml.safe_load((map_dir / "map.yaml").read_text())
img = np.flipud(np.asarray(Image.open(map_dir / meta["image"]).convert("L")))
occ_thresh = float(meta.get("occupied_thresh", 0.65)) * 255.0
free_thresh = float(meta.get("free_thresh", 0.196)) * 255.0
data = np.full(img.shape, -1, dtype=np.int8)
data[img <= 255.0 - occ_thresh] = 100
data[img >= 255.0 - free_thresh] = 0
occupied = data != 0
origin = meta["origin"]
grid = Grid(occupied, float(meta["resolution"]), float(origin[0]), float(origin[1]))

model = FormationModel([VehicleOffset(1, 1.0, 0.0),
                        VehicleOffset(2, 0.0, 1.0),
                        VehicleOffset(3, 0.0, -1.0)])
shapes = {vid: VehicleShape(0.738, 0.500) for vid in model.vehicle_ids}
start = FormationPose(sx, sy, spsi)
goal = FormationPose(gx, gy, gpsi)
request = UniformPlanningRequest(
    grid=grid, model=model, vehicle_shapes=shapes,
    start_qf=start,
    start_headings={vid: spsi for vid in model.vehicle_ids},
    goal_qf=goal, goal_headings=None, plan_id=900002,
    formation_id="cargo_wedge",
    search_config=GraphSearchConfig(max_seconds=max_seconds))

t0 = time.perf_counter()
result = plan_uniform(request)
search = result.search
out = {
    "status": result.status,
    "reason": result.reason,
    "first_failure_layer": result.first_failure_layer,
    "wall_s": result.wall_time_s,
    "expansions": None if search is None else search.expansions,
    "candidate_count": None if search is None else search.candidate_count,
    "accepted_successors": None if search is None else search.accepted_successors,
    "collision_s": None if search is None else search.collision_s,
    "open_size": None if search is None else search.open_size,
    "rejection_histogram": None if search is None else dict(search.rejection_histogram),
    "t_end_s": None if result.plan is None else result.plan.duration_s,
}
if result.plan is not None:
    out["path"] = [
        {"index": record["index"], "primitive": record["primitive"],
         "mode": record["mode"], "delta_psi": record["delta_psi"],
         "pivot": record["pivot"], "parameter_length": record["parameter_length"],
         "start_qf": record["start_qf"], "end_qf": record["end_qf"],
         "cumulative_proxy_g_s": record["cumulative_proxy_g_s"]}
        for record in search.as_dict(include_path=True)["path"]]
    out["phases"] = [
        {"phase": row["phase_id"], "kind": row["kind"],
         "mode": row["mode"], "t0_s": round(row["t0_s"], 6),
         "duration_s": round(row["duration_s"], 6),
         "primitive": row.get("primitive")}
        for row in result.plan.phase_table()]
    out["phase_totals"] = result.plan.phase_totals()
print(json.dumps(out, indent=2, sort_keys=True))