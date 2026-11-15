"""FR06 narrow-corner scene: frozen geometry, grids, reference edges.

Everything here is a pure function of the constants below.  The scene is a
distance-band corridor of constant nominal width around one fixed cargo
centreline: a straight approach, a quarter circle of radius 2 m about
``(8, 8)``, and a straight exit.  The band is rasterised on a 16 m x 16 m,
0.05 m grid that is independent of every historical map.

The reference edge sequence is the candidate geometry from the implementation
plan.  It is *not* a search result: search gets only the grid, the start pose,
the start body headings and the goal pose.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
SRC = ROOT / "catkin_ws/src/fwis_v3_optimizer/src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from fwis_v3_optimizer.core import Grid  # noqa: E402
from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_collision import VehicleShape  # noqa: E402
from fwis_v3_optimizer.uniform_motion import (  # noqa: E402
    a_arc_edge, o_translation_edge, s_spin_edge)
from fwis_v3_optimizer.uniform_search import SuccessorConfig  # noqa: E402
from fwis_v3_optimizer.uniform_timing import (  # noqa: E402
    MotionTiming, SetupTiming, schedule_phase_totals)
from fwis_v3_optimizer.uniform_trajectory import UniformTimedPlan  # noqa: E402

SCHEMA = "fr06.narrow_corner.v1"
MAP_SIZE_M = 16.0
RESOLUTION_M = 0.05
CELLS = 320
ORIGIN = (0.0, 0.0)
MAP_CENTRE_Y = 6.0
ARC_CENTRE = (8.0, 8.0)
ARC_RADIUS_M = 2.0
APPROACH_START = (4.0, 6.0)
ARC_START = (8.0, 6.0)
ARC_END = (10.0, 8.0)
EXIT_END = (10.0, 12.0)
START_QF = (6.0, 6.0, 0.0)
GOAL_QF = (10.0, 10.0, math.pi / 2.0)
FORMATION_ID = "fr06_three_vehicle"

WIDE_WIDTH_M = 3.2
TIGHT_WIDTH_M = 2.8
EXPLORATORY_WIDTH_M = 2.6
ANALYSIS_WIDTHS = (WIDE_WIDTH_M, TIGHT_WIDTH_M, EXPLORATORY_WIDTH_M)

# The reference sequence is fixed by the plan; search never sees these edges.
_ALIGN = math.atan2(1.0, 2.0)


def formation_model():
    return FormationModel((
        VehicleOffset(1, 1.0, 0.0),
        VehicleOffset(2, 0.0, 1.0),
        VehicleOffset(3, 0.0, -1.0),
    ))


def vehicle_shapes():
    return {vehicle_id: VehicleShape(0.738, 0.500)
            for vehicle_id in formation_model().vehicle_ids}


def successor_config():
    return SuccessorConfig()


def reference_edges(model=None):
    """The frozen reference: O(2 m), S alignment, A(+90 deg, pivot 2 m), O."""
    model = formation_model() if model is None else model
    ids = model.vehicle_ids
    headings = {vehicle_id: 0.0 for vehicle_id in ids}
    forward = {vehicle_id: 1 for vehicle_id in ids}

    first = o_translation_edge(
        model, FormationPose(*START_QF), headings, (2.0, 0.0), forward,
        {vehicle_id: 0.0 for vehicle_id in ids})
    aligned = s_spin_edge(
        model, first.end_qf, dict(first.end_headings),
        {1: _ALIGN, 2: 0.0, 3: 0.0})
    arc = a_arc_edge(
        model, aligned.end_qf, dict(aligned.end_headings), math.pi / 2.0,
        (0.0, 2.0), forward)
    arc_headings = dict(arc.end_headings)
    last = o_translation_edge(
        model, arc.end_qf, arc_headings, (0.0, 2.0), forward,
        {vehicle_id: math.pi / 2.0 - arc_headings[vehicle_id]
         for vehicle_id in ids})
    return (first, aligned, arc, last)


def reference_plan(plan_id=1, model=None):
    model = formation_model() if model is None else model
    return UniformTimedPlan.from_edges(
        reference_edges(model), vehicle_shapes(), plan_id=plan_id,
        formation_id=FORMATION_ID,
        timing_limits=successor_config().timing_limits, model=model)


def _segment_distance(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    wx, wy = px - ax, py - ay
    denominator = vx * vx + vy * vy
    if denominator <= 0.0:
        return np.hypot(px - ax, py - ay)
    t = np.clip((wx * vx + wy * vy) / denominator, 0.0, 1.0)
    return np.hypot(px - (ax + t * vx), py - (ay + t * vy))


def centreline_distance(x, y):
    """Exact distance from points to the frozen centreline (scalars or arrays)."""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    approach = _segment_distance(
        x, y, APPROACH_START[0], APPROACH_START[1], ARC_START[0],
        ARC_START[1])
    exit_leg = _segment_distance(
        x, y, ARC_END[0], ARC_END[1], EXIT_END[0], EXIT_END[1])
    dx = x - ARC_CENTRE[0]
    dy = y - ARC_CENTRE[1]
    angle = np.arctan2(dy, dx)
    radius = np.hypot(dx, dy)
    inside_arc = (angle >= -math.pi / 2.0) & (angle <= 0.0)
    arc = np.where(
        inside_arc, np.abs(radius - ARC_RADIUS_M),
        np.minimum(np.hypot(x - ARC_START[0], y - ARC_START[1]),
                   np.hypot(x - ARC_END[0], y - ARC_END[1])))
    return np.minimum(np.minimum(approach, exit_leg), arc)


def cell_centres():
    values = ORIGIN[0] + (np.arange(CELLS) + 0.5) * RESOLUTION_M
    return np.meshgrid(values, values)


def corridor_occupancy(width_m):
    """True where the cell centre lies outside the corridor band."""
    if not (0.0 < width_m < 2.0 * ARC_RADIUS_M):
        raise ValueError("corridor width must be below the arc diameter")
    x, y = cell_centres()
    return centreline_distance(x, y) >= width_m / 2.0


def corridor_grid(width_m):
    return Grid(corridor_occupancy(width_m), RESOLUTION_M,
                ORIGIN[0], ORIGIN[1])


def free_width_measurements(width_m):
    """Free-cell counts on a straight and on the arc section, in metres."""
    half = width_m / 2.0
    x, y = cell_centres()
    column = int(np.argmin(np.abs(x[0] - 7.0)))
    horizontal = corridor_occupancy(width_m)[:, column]
    vertical_row = int(np.argmin(np.abs(y[:, 0] - 10.0)))
    vertical = corridor_occupancy(width_m)[vertical_row, :]
    # Radial free width on the 45 degree ray of the quarter turn.
    angle = -math.pi / 4.0
    radii = np.arange(0.0, 4.0, RESOLUTION_M / 4.0)
    points_x = ARC_CENTRE[0] + radii * math.cos(angle)
    points_y = ARC_CENTRE[1] + radii * math.sin(angle)
    arc_free = centreline_distance(points_x, points_y) < half
    return {
        "nominal_width_m": width_m,
        "horizontal_mid_section_free_m":
            round(float(np.count_nonzero(~horizontal)) * RESOLUTION_M, 6),
        "vertical_mid_section_free_m":
            round(float(np.count_nonzero(~vertical)) * RESOLUTION_M, 6),
        "arc_radial_free_m":
            round(float(np.count_nonzero(arc_free)) * RESOLUTION_M / 4.0, 6),
        "occupied_cells": int(np.count_nonzero(corridor_occupancy(width_m))),
        "free_cells": int(np.count_nonzero(~corridor_occupancy(width_m))),
    }


def _rectangle_corners(pose, shape):
    cosine, sine = math.cos(pose.psi), math.sin(pose.psi)
    half_l, half_w = shape.length_m / 2.0, shape.width_m / 2.0
    local = ((half_l, half_w), (half_l, -half_w),
             (-half_l, -half_w), (-half_l, half_w))
    return [(pose.x + cosine * lx - sine * ly,
             pose.y + sine * lx + cosine * ly) for lx, ly in local]


def _rectangle_boundary_points(pose, shape, spacing_m=0.01):
    corners = _rectangle_corners(pose, shape)
    points = []
    for first, second in zip(corners, corners[1:] + corners[:1]):
        length = math.hypot(second[0] - first[0], second[1] - first[1])
        count = max(1, int(math.ceil(length / spacing_m)))
        for step in range(count):
            ratio = step / float(count)
            points.append((first[0] + ratio * (second[0] - first[0]),
                           first[1] + ratio * (second[1] - first[1])))
    return points


def formation_envelope_and_clearance(plan, width_m, sample_dt=0.01):
    """Maximum lateral envelope width and minimum wall clearance.

    The clearance is measured analytically against the same distance band
    that the grid rasterises: ``half_width - distance(centreline)`` at every
    vehicle-boundary sample.  Collision margin is not subtracted here.
    """
    model = plan.model
    shapes = plan.vehicle_shapes
    half = width_m / 2.0
    times = np.arange(0.0, plan.duration_s + 1e-9, sample_dt)
    envelope = 0.0
    clearance = float("inf")
    worst = None
    for time_s in times:
        state = plan.state_at(float(min(time_s, plan.duration_s)))
        normal = (-math.sin(state.qf.psi), math.cos(state.qf.psi))
        projections = []
        for vehicle in state.vehicles:
            pose = FormationPose(vehicle.x, vehicle.y, vehicle.theta)
            shape = shapes[vehicle.vehicle_id]
            for px, py in _rectangle_corners(pose, shape):
                projections.append(px * normal[0] + py * normal[1])
            boundary = _rectangle_boundary_points(pose, shape)
            bx = np.array([point[0] for point in boundary])
            by = np.array([point[1] for point in boundary])
            values = half - centreline_distance(bx, by)
            value = float(np.min(values))
            if value < clearance:
                clearance = value
                index = int(np.argmin(values))
                worst = {
                    "time_s": float(state.time_s),
                    "vehicle_id": int(vehicle.vehicle_id),
                    "x": float(bx[index]),
                    "y": float(by[index]),
                    "clearance_m": value,
                }
        span = max(projections) - min(projections)
        envelope = max(envelope, float(span))
    return {
        "envelope_width_m": round(envelope, 9),
        "nominal_width_m": width_m,
        "net_width_to_envelope_ratio": round(width_m / envelope, 9),
        "minimum_clearance_m": round(clearance, 9),
        "minimum_clearance_after_margin_m": round(
            clearance - successor_config().collision_margin_m, 9),
        "minimum_clearance_witness": worst,
        "sample_dt_s": sample_dt,
        "formation_ids": sorted(int(value) for value in model.vehicle_ids),
    }


def phase_summary(plan):
    totals = schedule_phase_totals(plan.schedule)
    mode_counts = {"A": 0, "O": 0, "S": 0}
    mode_seconds = {"A": 0.0, "O": 0.0, "S": 0.0}
    setup_seconds = 0.0
    ordinary_motion_blocks = 0
    spin_blocks = 0
    for phase in plan.phases:
        if isinstance(phase, MotionTiming):
            mode = phase.edge.mode
            mode_counts[mode] += 1
            mode_seconds[mode] += float(phase.duration_s)
            if mode == "S":
                spin_blocks += 1
            else:
                ordinary_motion_blocks += 1
        elif isinstance(phase, SetupTiming):
            setup_seconds += float(phase.duration_s)
    return {
        "total_duration_s": round(float(plan.duration_s), 9),
        "initial_setup_s": round(totals["initial_setup_s"], 9),
        "inter_edge_setup_s": round(totals["inter_edge_setup_s"], 9),
        "motion_s": round(totals["motion_s"], 9),
        "initial_setup_count": totals["initial_setup_count"],
        "inter_edge_setup_count": totals["inter_edge_setup_count"],
        "setup_count": totals["initial_setup_count"]
                       + totals["inter_edge_setup_count"],
        "motion_count": totals["motion_count"],
        "mode_block_counts": mode_counts,
        "mode_motion_seconds": {key: round(value, 9)
                                for key, value in mode_seconds.items()},
        "spin_block_count": spin_blocks,
        "ordinary_motion_block_count": ordinary_motion_blocks,
        # Every motion block (A/O/S) starts and ends at zero speed in this
        # first-version timing; an ordinary block stop is one non-S motion
        # block boundary.  Setup phases are stationary dwell, not motion.
        "ordinary_block_stops": ordinary_motion_blocks,
        "stationary_motion_boundaries": totals["motion_count"],
        "phase_table": [
            {"phase_id": phase.phase_id,
             "kind": "setup" if isinstance(phase, SetupTiming) else "motion",
             "mode": (None if isinstance(phase, SetupTiming)
                      else phase.edge.mode),
             "t0_s": round(float(phase.t0_s), 9),
             "duration_s": round(float(phase.duration_s), 9)}
            for phase in plan.phases],
    }


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def sha256_file(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def occupancy_sha256(width_m):
    return sha256_bytes(np.ascontiguousarray(
        corridor_occupancy(width_m)).tobytes())


def write_map_dir(width_m, directory):
    """Write map.pgm/map.yaml/scene_meta.yaml for the ROS demo."""
    directory = Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    free = ~corridor_occupancy(width_m)
    gray = np.where(free, 254, 0).astype(np.uint8)
    Image.fromarray(np.flipud(gray), mode="L").save(directory / "map.pgm")
    (directory / "map.yaml").write_text(
        "image: map.pgm\n"
        "resolution: {}\n"
        "origin:\n- {}\n- {}\n- 0.0\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n".format(
            RESOLUTION_M, ORIGIN[0], ORIGIN[1]), encoding="utf-8")
    meta = {
        "name": "FR06 narrow corner {:.1f} m".format(width_m),
        "map_id": directory.name,
        "size_m": [MAP_SIZE_M, MAP_SIZE_M],
        "resolution": RESOLUTION_M,
        "origin": [ORIGIN[0], ORIGIN[1], 0.0],
        "border": True,
        "kind": "fr06_distance_band_corridor",
        "corridor_width_m": width_m,
        "centreline": {
            "approach": [list(APPROACH_START), list(ARC_START)],
            "arc": {"centre": list(ARC_CENTRE),
                    "radius_m": ARC_RADIUS_M,
                    "from_rad": -math.pi / 2.0, "to_rad": 0.0},
            "exit": [list(ARC_END), list(EXIT_END)],
        },
        "start": {"x": START_QF[0], "y": START_QF[1],
                  "theta": START_QF[2]},
        "goal": {"x": GOAL_QF[0], "y": GOAL_QF[1], "theta": GOAL_QF[2]},
        "description": "static L corridor, 90 degree arc of radius 2 m",
        "test_point_source": "project_defined_synthetic_test_case",
        "reference_is_not_a_search_seed": True,
    }
    import yaml
    (directory / "scene_meta.yaml").write_text(
        yaml.safe_dump(meta, sort_keys=False, allow_unicode=True),
        encoding="utf-8")
    return {
        "width_m": width_m,
        "directory": str(directory.relative_to(ROOT)),
        "map_pgm_sha256": sha256_file(directory / "map.pgm"),
        "map_yaml_sha256": sha256_file(directory / "map.yaml"),
        "scene_meta_sha256": sha256_file(directory / "scene_meta.yaml"),
        "occupancy_sha256": occupancy_sha256(width_m),
        "free_widths": free_width_measurements(width_m),
    }


def canonical_json(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      allow_nan=False)


def json_sha256(value):
    return sha256_bytes(canonical_json(value).encode("utf-8"))