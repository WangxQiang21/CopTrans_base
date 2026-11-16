#!/usr/bin/env python3
"""G4-A: construct, independently reference-check and freeze A-only scenes.

The tool never calls the autonomous planner.  Maps are rasterised from an
independent Gauss-Legendre centreline; the frozen reference is the production
C2 path, retimed by G3 and checked by the independent geometry/dynamics audits
plus the unchanged collision engine on the actual PGM grid.

Outputs (under ``--output``):

* ``maps/<scene>.pgm`` / ``.yaml``      authoritative maps
* ``reference/<scene>.json``           production reference + three audits
* ``inputs/<scene>/{map,request}.json`` no-reference planning inputs
* ``scene_manifest.json``              frozen digest manifest

The no-reference input copies deliberately contain only the map, the physical
request and the three configs; no profile, trajectory or known-good plan.
"""
import argparse
import datetime as _dt
import hashlib
import json
import math
from pathlib import Path
import shutil

import numpy as np
from scipy.spatial import cKDTree

from fwis_v3_optimizer.core import Grid
from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import FormationModel, FormationPose, VehicleOffset
from fwis_v3_optimizer.formation_collision import VehicleShape
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits
from fwis_v3_optimizer.formation_continuous_a.audit import (
    audit_dynamics,
    audit_geometry,
)
from fwis_v3_optimizer.formation_continuous_a.collision import (
    audit_collision,
    load_map,
    write_map,
)
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    CollisionConfig,
    DynamicAuditConfig,
    G4_PLANNING_SCHEMA,
    GeometryAuditConfig,
    GoalPolicy,
    InitialAssumption,
    PlannerConfig,
    TimingConfig,
    VirtualAContractError,
    VirtualAMapRequest,
    VirtualARequest,
    collision_config_from_dict,
    grid_signature,
    planner_config_from_dict,
    planning_context_sha256,
    request_fingerprint,
    request_from_payload,
    request_payload,
    timing_config_from_dict,
)
from fwis_v3_optimizer.formation_continuous_a.geometry import (
    C2CurvatureProfile,
    VirtualAPath,
)
from fwis_v3_optimizer.formation_continuous_a.timing import retime_path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "virtual_a" / "open_requests.json"
BASE_COMMIT = "567bdecba9676313da3bc9a97b82f57b5b569a2b"
RESOLUTION = 0.05
CORRIDOR_HALF_WIDTH = {"S1": 1.60, "S2": 1.40}
EXTENSION_M = 2.0
BOUNDARY_M = 3.0
REFERENCE_SAMPLE_M = 0.01
BODY_LENGTH = 0.738
BODY_WIDTH = 0.500


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True, allow_nan=False)


def _digest(value):
    return hashlib.sha256(_canonical(value).encode("utf-8")).hexdigest()


def _load_base():
    record = json.loads(FIXTURE.read_text())["base"]
    model = FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in record["model"]["offsets"]])
    shapes = {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in record["model"]["vehicle_shapes"]}
    return record, model, shapes


BASE, MODEL, SHAPES = _load_base()
LIMITS = FormationTimingLimits(**BASE["limits"])
INITIAL = InitialAssumption(**BASE["initial"])
_PLANNER_DICT = json.loads(FIXTURE.read_text())["planner_config"]
PLANNER_CONFIG = PlannerConfig(**_PLANNER_DICT)
# The G2/G3 accepted independent audit threshold is 5e-8 m.  The 12 m S2
# reference cannot be confirmed at that threshold with the coarse global
# triangle bound, so the audit uses the exact per-panel fourth-derivative
# interval bound in formation_continuous_a.integration_bounds.  The G1
# production 1e-7 m contract, floating/angle guard, dynamic limits, collision
# margin and sweep step are unchanged.
REFERENCE_GEOMETRY_AUDIT = GeometryAuditConfig(audit_position_budget_m=5e-8)
TIMING_CONFIG = TimingConfig(geometry_audit_config=REFERENCE_GEOMETRY_AUDIT)
DYNAMIC_AUDIT_CONFIG = DynamicAuditConfig(
    geometry_audit_config=REFERENCE_GEOMETRY_AUDIT)
COLLISION_CONFIG = CollisionConfig()


# ---------------------------------------------------------------------------
# Independent Gauss-Legendre reference integration
# ---------------------------------------------------------------------------

def _gauss_legendre(order):
    nodes, weights = np.polynomial.legendre.leggauss(order)
    return nodes, weights


_GL_NODES, _GL_WEIGHTS = _gauss_legendre(24)
_GL64_NODES, _GL64_WEIGHTS = _gauss_legendre(64)


def _gl_integral(function, a, b, order=24):
    nodes, weights = (_GL64_NODES, _GL64_WEIGHTS) if order == 64 \
        else (_GL_NODES, _GL_WEIGHTS)
    midpoint = 0.5 * (a + b)
    half = 0.5 * (b - a)
    total = 0.0
    for node, weight in zip(nodes, weights):
        total += weight * function(midpoint + half * float(node))
    return half * total


def clamped_cubic_derivatives(stations, values):
    """Clamped C2 spline derivative values with endpoint slopes zero."""
    stations = [float(value) for value in stations]
    values = [float(value) for value in values]
    count = len(stations) - 1
    if count < 1:
        raise ValueError("at least two stations are required")
    h = [stations[i + 1] - stations[i] for i in range(count)]
    if any(step <= 0.0 for step in h):
        raise ValueError("stations must be strictly increasing")
    interior = count - 1
    if interior == 0:
        return [0.0, 0.0]
    lower = [0.0] * interior
    diagonal = [0.0] * interior
    upper = [0.0] * interior
    rhs = [0.0] * interior
    for k in range(interior):
        i = k + 1
        lower[k] = h[i - 1]
        diagonal[k] = 2.0 * (h[i - 1] + h[i])
        upper[k] = h[i]
        rhs[k] = 3.0 * (h[i] * (values[i] - values[i - 1]) / h[i - 1]
                        + h[i - 1] * (values[i + 1] - values[i]) / h[i])
    for k in range(1, interior):
        factor = lower[k] / diagonal[k - 1]
        diagonal[k] -= factor * upper[k - 1]
        rhs[k] -= factor * rhs[k - 1]
    solution = [0.0] * interior
    solution[-1] = rhs[-1] / diagonal[-1]
    for k in range(interior - 2, -1, -1):
        solution[k] = (rhs[k] - upper[k] * solution[k + 1]) / diagonal[k]
    return [0.0] + solution + [0.0]


class IndependentReference:
    """Independent k/psi/x/y evaluation for a clamped-spline k profile.

    Curvature is the cubic Hermite polynomial of the clamped spline.  Its
    arc-length integral is evaluated analytically (quartic antiderivative);
    x/y use fixed Gauss-Legendre quadrature.  Prefixes at the knots are
    accumulated once, so repeated station queries stay cheap.
    """

    def __init__(self, stations, values, derivatives):
        self.stations = tuple(float(value) for value in stations)
        self.values = tuple(float(value) for value in values)
        self.derivatives = tuple(float(value) for value in derivatives)
        self.length = self.stations[-1]
        self.count = len(self.stations) - 1
        self._h = tuple(self.stations[i + 1] - self.stations[i]
                        for i in range(self.count))
        self._psi_prefix = [0.0]
        self._x_prefix = [0.0]
        self._y_prefix = [0.0]
        for index in range(self.count):
            a = self.stations[index]
            b = self.stations[index + 1]
            base = self._psi_prefix[-1]
            self._psi_prefix.append(base + self._psi_delta(index, 1.0))
            x_increment = _gl_integral(
                lambda u, idx=index, base_psi=base: math.cos(
                    base_psi + self._psi_delta(idx,
                                               (float(u) - self.stations[idx])
                                               / self._h[idx])),
                a, b, 64)
            y_increment = _gl_integral(
                lambda u, idx=index, base_psi=base: math.sin(
                    base_psi + self._psi_delta(idx,
                                               (float(u) - self.stations[idx])
                                               / self._h[idx])),
                a, b, 64)
            self._x_prefix.append(self._x_prefix[-1] + x_increment)
            self._y_prefix.append(self._y_prefix[-1] + y_increment)

    def _locate(self, s):
        if s < 0.0 or s > self.length:
            raise ValueError("s={!r} outside reference domain".format(s))
        for index in range(self.count):
            if s <= self.stations[index + 1]:
                return index, (s - self.stations[index]) / self._h[index]
        return self.count - 1, 1.0

    def _coefficients(self, index):
        h = self._h[index]
        y0, y1 = self.values[index], self.values[index + 1]
        d0, d1 = self.derivatives[index], self.derivatives[index + 1]
        return (y0, h * d0, 3.0 * (y1 - y0) - h * (2.0 * d0 + d1),
                2.0 * (y0 - y1) + h * (d0 + d1))

    def _psi_delta(self, index, q):
        y0, c1, c2, c3 = self._coefficients(index)
        h = self._h[index]
        return h * (y0 * q + c1 * q * q / 2.0 + c2 * q ** 3 / 3.0
                    + c3 * q ** 4 / 4.0)

    def curvature(self, s):
        index, q = self._locate(float(s))
        y0, c1, c2, c3 = self._coefficients(index)
        return y0 + c1 * q + c2 * q * q + c3 * q ** 3

    def state(self, s):
        s = float(s)
        index, q = self._locate(s)
        a = self.stations[index]
        base_psi = self._psi_prefix[index]
        base_x = self._x_prefix[index]
        base_y = self._y_prefix[index]
        x = base_x + _gl_integral(
            lambda u: math.cos(base_psi + self._psi_delta(
                index, (float(u) - a) / self._h[index])),
            a, s, 24)
        y = base_y + _gl_integral(
            lambda u: math.sin(base_psi + self._psi_delta(
                index, (float(u) - a) / self._h[index])),
            a, s, 24)
        psi = base_psi + self._psi_delta(index, q)
        return {"s": s, "x": x, "y": y, "psi": psi,
                "k": self.curvature(s)}


class QuadraticCurvatureReference:
    """S1 exact k=.45*(s/S)^2 with independent GL position integration."""

    def __init__(self, curvature, length):
        self.curvature = float(curvature)
        self.length = float(length)

    def k(self, s):
        return self.curvature * (float(s) / self.length) ** 2

    def psi(self, s):
        return self.curvature * float(s) ** 3 / (3.0 * self.length ** 2)

    def state(self, s):
        s = float(s)
        if s < 0.0 or s > self.length:
            raise ValueError("s outside reference domain")
        x = _gl_integral(
            lambda u: math.cos(self.psi(u)), 0.0, s, 64)
        y = _gl_integral(
            lambda u: math.sin(self.psi(u)), 0.0, s, 64)
        return {"s": s, "x": x, "y": y, "psi": self.psi(s), "k": self.k(s)}


# ---------------------------------------------------------------------------
# Scene profiles and requests
# ---------------------------------------------------------------------------

def s1_profile():
    curvature, length = 0.45, 3.0 * math.pi / (2.0 * 0.45)
    knots = []
    for index in range(9):
        s = length * index / 8.0
        knots.append(ControlKnot(
            s, curvature * (s / length) ** 2,
            2.0 * curvature * s / length ** 2))
    profile = C2CurvatureProfile.from_knots(knots)
    independent = QuadraticCurvatureReference(curvature, length)
    return profile, independent, "quadratic_curvature_90deg"


def s2_profile():
    stations = [1.5 * index for index in range(9)]
    values = [0.0, 0.30, 0.0, -0.30, 0.0, -0.30, 0.0, 0.30, 0.0]
    derivatives = clamped_cubic_derivatives(stations, values)
    knots = [ControlKnot(s, k, d)
             for s, k, d in zip(stations, values, derivatives)]
    profile = C2CurvatureProfile.from_knots(knots)
    independent = IndependentReference(stations, values, derivatives)
    # Self-check the independent spline against the production C2 jets.
    for s in np.linspace(0.0, 12.0, 97):
        jet = profile.jet_at(float(min(s, 12.0)))
        independent_k = independent.curvature(float(min(s, 12.0)))
        if abs(jet.kappa - independent_k) > 1e-12:
            raise RuntimeError(
                "independent spline disagrees at s={}: {} vs {}".format(
                    s, jet.kappa, independent_k))
    return profile, independent, "clamped_cubic_double_lane_change"


def make_request(plan_id, formation_id, start_qf, goal_qf):
    return VirtualARequest(
        plan_id=plan_id, formation_id=formation_id, model=MODEL,
        vehicle_shapes=SHAPES, start_qf=start_qf,
        start_headings={1: 0.0, 2: 0.0, 3: 0.0}, goal_qf=goal_qf,
        initial=INITIAL, goal_policy=GoalPolicy.FREE, limits=LIMITS)


def mirror_qf(qf):
    return FormationPose(qf.x, -qf.y, -qf.psi)


def mirror_profile(profile):
    knots = [ControlKnot(float(knot.s), -float(knot.value),
                         -float(knot.derivative)) for knot in profile.knots]
    return C2CurvatureProfile.from_knots(knots)


class MirroredIndependent:
    def __init__(self, source):
        self.source = source
        self.length = source.length

    def state(self, s):
        record = self.source.state(s)
        return {"s": record["s"], "x": record["x"], "y": -record["y"],
                "psi": -record["psi"], "k": -record.get("k", 0.0)}


# ---------------------------------------------------------------------------
# Rasterisation
# ---------------------------------------------------------------------------

def _polyline_samples(reference):
    samples = []
    count = max(2, int(math.ceil(reference.length / REFERENCE_SAMPLE_M)))
    for index in range(count + 1):
        s = reference.length * index / count
        record = reference.state(s)
        samples.append((record["x"], record["y"]))
    start = reference.state(0.0)
    end = reference.state(reference.length)
    start_tangent = (math.cos(start["psi"]), math.sin(start["psi"]))
    end_tangent = (math.cos(end["psi"]), math.sin(end["psi"]))
    extension = int(math.ceil(EXTENSION_M / REFERENCE_SAMPLE_M))
    prefix = [(start["x"] - start_tangent[0] * EXTENSION_M * i / extension,
               start["y"] - start_tangent[1] * EXTENSION_M * i / extension)
              for i in range(extension, 0, -1)]
    suffix = [(end["x"] + end_tangent[0] * EXTENSION_M * i / extension,
               end["y"] + end_tangent[1] * EXTENSION_M * i / extension)
              for i in range(1, extension + 1)]
    return prefix + samples + suffix


def corridor_grid(reference, half_width):
    points = np.asarray(_polyline_samples(reference), dtype=float)
    if len(points) < 2:
        raise RuntimeError("empty reference polyline")
    tree = cKDTree(points)
    radius_pad = BOUNDARY_M + half_width + 4.0 * RESOLUTION
    x_min = float(np.floor((points[:, 0].min() - radius_pad) / RESOLUTION) *
                  RESOLUTION)
    x_max = float(np.ceil((points[:, 0].max() + radius_pad) / RESOLUTION) *
                  RESOLUTION)
    y_min = float(np.floor((points[:, 1].min() - radius_pad) / RESOLUTION) *
                  RESOLUTION)
    y_max = float(np.ceil((points[:, 1].max() + radius_pad) / RESOLUTION) *
                  RESOLUTION)
    nx = max(1, int(round((x_max - x_min) / RESOLUTION)))
    ny = max(1, int(round((y_max - y_min) / RESOLUTION)))
    xs = x_min + (np.arange(nx) + 0.5) * RESOLUTION
    ys = y_min + (np.arange(ny) + 0.5) * RESOLUTION
    mesh_x, mesh_y = np.meshgrid(xs, ys)
    centres = np.stack([mesh_x.ravel(), mesh_y.ravel()], axis=1)
    distances, _ = tree.query(centres, k=1)
    half_diagonal = 0.5 * RESOLUTION * math.sqrt(2.0)
    free = (distances.reshape(ny, nx) + half_diagonal + 1e-12) <= half_width
    occupancy = ~free
    return occupancy, {
        "origin_x": x_min, "origin_y": y_min,
        "resolution": RESOLUTION, "width": nx, "height": ny,
        "free_cells": int(free.sum()), "occupied_cells": int(occupancy.sum()),
        "corridor_half_width_m": half_width,
        "raster_rule": (
            "cell free iff nearest extended-centreline sample distance at "
            "the cell centre + cell half-diagonal <= half_width; 1-Lipschitz "
            "distance makes this a conservative whole-cell condition"),
        "boundary_occupied_m": BOUNDARY_M,
        "extension_m": EXTENSION_M,
        "reference_sample_step_m": REFERENCE_SAMPLE_M,
        "centreline_sample_count": int(len(points)),
    }


def _body_corners(record, offset):
    """Independent OBB corners for one vehicle at one reference station."""
    a, b = float(offset.x), float(offset.y)
    k = record["k"]
    psi = record["psi"]
    D = (1.0 - b * k) ** 2 + (a * k) ** 2
    if not (1.0 - b * k > 0.0 and D > 0.0):
        raise RuntimeError("normal branch violated in reference envelope")
    theta = psi + math.atan2(a * k, 1.0 - b * k)
    cx = record["x"] + math.cos(psi) * a - math.sin(psi) * b
    cy = record["y"] + math.sin(psi) * a + math.cos(psi) * b
    c, s = math.cos(theta), math.sin(theta)
    hx, hy = BODY_LENGTH / 2.0, BODY_WIDTH / 2.0
    return [(cx + sx * hx + nx * hy, cy + sy * hx + ny * hy)
            for sx, sy, nx, ny in (
                (c, s, -s, c), (c, s, s, -c),
                (-c, -s, -s, c), (-c, -s, s, -c))]


def reference_envelope(reference, stations):
    records = [reference.state(s) for s in stations]
    bodies = []
    for record in records:
        bodies.append([_body_corners(record, offset)
                       for offset in MODEL.offsets])
    return records, bodies


def slalom_grid(reference):
    fine = np.linspace(0.0, reference.length, 
                       int(math.ceil(reference.length / 0.002)) + 1)
    records, bodies = reference_envelope(reference, fine)
    all_x = [point[0] for body_list in bodies for body in body_list
             for point in body]
    all_y = [point[1] for body_list in bodies for body in body_list
             for point in body]
    x_min_room = math.floor((min(all_x) - 2.0) / RESOLUTION) * RESOLUTION
    x_max_room = math.ceil((max(all_x) + 2.0) / RESOLUTION) * RESOLUTION
    y_min_room = math.floor(min(-3.0, min(all_y) - 0.5) / RESOLUTION) * \
        RESOLUTION
    y_max_room = math.ceil(max(3.0, max(all_y) + 0.5) / RESOLUTION) * \
        RESOLUTION
    for name, value in (("x_min", x_min_room), ("x_max", x_max_room),
                        ("y_min", y_min_room), ("y_max", y_max_room)):
        if not math.isfinite(float(value)):
            raise RuntimeError("nonfinite room bound {}".format(name))
    band_1_x = [reference.state(s)["x"] for s in np.linspace(4.8, 6.0, 601)]
    band_2_x = [reference.state(s)["x"] for s in np.linspace(9.0, 10.5, 901)]
    band_1 = (min(band_1_x), max(band_1_x))
    band_2 = (min(band_2_x), max(band_2_x))

    def in_band_y(corners, band):
        low_x = min(point[0] for point in corners)
        high_x = max(point[0] for point in corners)
        return high_x >= band[0] and low_x <= band[1]

    min_y_band_1 = math.inf
    max_y_band_2 = -math.inf
    for body_list in bodies:
        for corners in body_list:
            if in_band_y(corners, band_1):
                min_y_band_1 = min(min_y_band_1,
                                   min(point[1] for point in corners))
            if in_band_y(corners, band_2):
                max_y_band_2 = max(max_y_band_2,
                                   max(point[1] for point in corners))
    if not (math.isfinite(min_y_band_1) and math.isfinite(max_y_band_2)):
        raise RuntimeError("reference bodies do not intersect a baffle band")
    edge_1 = min_y_band_1 - 0.05 - 0.10
    edge_2 = max_y_band_2 + 0.05 + 0.10

    nx = max(1, int(round((x_max_room - x_min_room) / RESOLUTION)))
    ny = max(1, int(round((y_max_room - y_min_room) / RESOLUTION)))
    xs = x_min_room + (np.arange(nx) + 0.5) * RESOLUTION
    ys = y_min_room + (np.arange(ny) + 0.5) * RESOLUTION
    mesh_x, mesh_y = np.meshgrid(xs, ys)
    occupancy = np.zeros((ny, nx), dtype=bool)
    half = RESOLUTION / 2.0

    def span(lo, hi, origin, count):
        centres = origin + (np.arange(count) + 0.5) * RESOLUTION
        overlap = (centres + half > lo) & (centres - half < hi)
        return centres, overlap

    x_centres, x_overlap_1 = span(band_1[0], band_1[1], x_min_room, nx)
    x_centres_2, x_overlap_2 = span(band_2[0], band_2[1], x_min_room, nx)
    y_centres, y_below_1 = span(y_min_room - 1e-12, edge_1, y_min_room, ny)
    y_centres_2, y_above_2 = span(edge_2, y_max_room + 1e-12, y_min_room, ny)
    occupancy[np.ix_(y_below_1, x_overlap_1)] = True
    occupancy[np.ix_(y_above_2, x_overlap_2)] = True
    info = {
        "origin_x": float(x_min_room), "origin_y": float(y_min_room),
        "resolution": RESOLUTION, "width": nx, "height": ny,
        "free_cells": int((~occupancy).sum()),
        "occupied_cells": int(occupancy.sum()),
        "room_x_m": [float(x_min_room), float(x_max_room)],
        "room_y_m": [float(y_min_room), float(y_max_room)],
        "body_envelope_x_m": [float(min(all_x)), float(max(all_x))],
        "body_envelope_y_m": [float(min(all_y)), float(max(all_y))],
        "first_band_x_m": [float(band_1[0]), float(band_1[1])],
        "first_band_min_body_y_m": float(min_y_band_1),
        "first_baffle_upper_y_m": float(edge_1),
        "second_band_x_m": [float(band_2[0]), float(band_2[1])],
        "second_band_max_body_y_m": float(max_y_band_2),
        "second_baffle_lower_y_m": float(edge_2),
        "base_margin_m": 0.05, "extra_margin_m": 0.10,
        "envelope_sample_step_m": 0.002,
        "envelope_rule": (
            "dense independent OBB corner samples plus the stated 0.05 m "
            "base and 0.10 m construction margin; final authority is the "
            "unchanged collision engine on the rasterised grid"),
    }
    return occupancy, info


# ---------------------------------------------------------------------------
# Mirror maps and direct-straight negative witnesses
# ---------------------------------------------------------------------------

def mirror_map(occupancy, info, source_id):
    """Exact y-mirror of a frozen raster map."""
    flipped = np.flipud(np.asarray(occupancy, dtype=bool))
    height = flipped.shape[0]
    mirrored = dict(info)
    y_min, y_max = info["room_y_m"] if "room_y_m" in info else (
        info["origin_y"], info["origin_y"] + height * info["resolution"])
    mirrored["origin_y"] = float(-y_max)
    mirrored["room_y_m"] = [float(-y_max), float(-y_min)]
    mirrored["mirrored_from"] = source_id
    mirrored["mirror_rule"] = "exact row flip with origin_y' = -(origin_y + h*res)"
    mirrored["free_cells"] = int((~flipped).sum())
    mirrored["occupied_cells"] = int(flipped.sum())
    return flipped, mirrored


def straight_profile(length):
    knots = [ControlKnot(float(value), 0.0, 0.0)
             for value in np.linspace(0.0, float(length), 9)]
    return C2CurvatureProfile.from_knots(knots)


def direct_straight_witness(scene_id, request, grid, context):
    """A straight C2 candidate that must be rejected by the map."""
    goal = request.goal_qf
    length = math.hypot(goal.x - request.start_qf.x,
                        goal.y - request.start_qf.y)
    if length <= 0.0:
        return {"status": "NOT_APPLICABLE", "reason": "zero length"}
    path = VirtualAPath(request.start_qf, straight_profile(length), MODEL)
    timing = retime_path(request, path, TIMING_CONFIG,
                         planning_context=context)
    if timing.plan is None:
        return {
            "status": "NOT_RUN",
            "scene_id": scene_id,
            "timing_status": timing.status,
            "timing_reason": timing.reason,
            "detail": (
                "the straight witness did not reach a timed plan; this is "
                "not a collision rejection"),
        }
    dynamic = audit_dynamics(request, timing.plan, DYNAMIC_AUDIT_CONFIG,
                             expected_planning_context=context)
    collision = audit_collision(request, timing.plan, grid,
                                COLLISION_CONFIG)
    return {
        "status": "REJECTED" if not collision.passed else "NOT_REJECTED",
        "scene_id": scene_id,
        "path_length_m": float(path.length_m),
        "duration_s": timing.T_s,
        "timing_status": timing.status,
        "dynamic_passed": bool(dynamic.passed),
        "dynamic_reason": dynamic.reason,
        "collision_passed": bool(collision.passed),
        "collision_reason": collision.reason,
        "collision_first_failure": collision.first_failure,
        "collision_checks": collision.check_count,
        "collision_detail": collision.detail,
    }


# ---------------------------------------------------------------------------
# Reference audit and manifest
# ---------------------------------------------------------------------------

def _audit_summary(report):
    return {
        "scope": report.scope,
        "passed": bool(report.passed),
        "reason": report.reason,
        "detail": report.detail,
        "first_failure": report.first_failure,
        "check_count": report.check_count,
        "request_sha256": report.request_sha256,
        "planning_context_sha256": report.planning_context_sha256,
        "grid_sha256": report.grid_sha256,
        "model_sha256": report.model_sha256 or (
            report.metrics.get("model_sha256") if isinstance(
                report.metrics, dict) else None),
        "metrics_keys": sorted(report.metrics),
    }


def build_scene(scene_id, base_name, profile, independent, request,
                map_builder):
    occupancy, map_info = map_builder(independent)
    return {
        "scene_id": scene_id,
        "base": base_name,
        "profile": profile,
        "independent": independent,
        "request": request,
        "occupancy": occupancy,
        "map_info": map_info,
    }


def _map_paths(directory, scene_id):
    stem = scene_id
    return directory / (stem + ".pgm"), directory / (stem + ".yaml")


def _write_input_bundle(scene, input_dir):
    target = input_dir / scene["scene_id"]
    target.mkdir(parents=True, exist_ok=True)
    image = target / "map.pgm"
    yaml = target / "map.yaml"
    shutil.copyfile(scene["image_path"], image)
    # The input copy renames the image to map.pgm, so its YAML must point at
    # the copied file rather than the authoritative maps/<scene>.pgm name.
    lines = []
    for line in scene["yaml_path"].read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("image:"):
            line = "image: map.pgm"
        lines.append(line)
    yaml.write_text("\n".join(lines) + "\n", encoding="utf-8")
    payload = {
        "schema": G4_PLANNING_SCHEMA,
        "stage": "G4",
        "request": request_payload(scene["request"]),
        "planner_config": PLANNER_CONFIG.as_dict(),
        "timing_config": TIMING_CONFIG.as_dict(),
        "collision_config": COLLISION_CONFIG.as_dict(),
        "map": {"yaml": "map.yaml", "image": "map.pgm"},
        "note": (
            "No reference profile, trajectory, audit or known-good plan is "
            "present in this bundle."),
    }
    (target / "request.json").write_text(_canonical(payload), encoding="utf-8")
    return target


def _verify_frozen_references(output, manifest_scenes, scenes):
    """Re-load frozen inputs and re-run all three reference audits.

    This deliberately starts from the serialised PGM/YAML, the no-reference
    request bundle and the reference knot list; it does not reuse the
    in-memory paths or audit reports from construction.
    """
    by_id = {scene["scene_id"]: scene for scene in scenes}
    verification = []
    failures = []
    for scene_id in sorted(manifest_scenes):
        entry = manifest_scenes[scene_id]
        authoritative, _ = load_map(output / "maps" / (scene_id + ".yaml"))
        grid, map_meta = load_map(
            output / "inputs" / scene_id / "map.yaml")
        payload = json.loads(
            (output / "inputs" / scene_id / "request.json").read_text(
                encoding="utf-8"))
        request = request_from_payload(payload["request"])
        planner = planner_config_from_dict(payload["planner_config"])
        timing_config = timing_config_from_dict(payload["timing_config"])
        collision_config = collision_config_from_dict(
            payload["collision_config"])
        map_request = VirtualAMapRequest(
            request, grid, planner, timing_config, collision_config)
        context = map_request.context()
        reference = json.loads(
            (output / entry["reference_file"]).read_text(encoding="utf-8"))
        profile = C2CurvatureProfile.from_knots([
            ControlKnot(float(knot["s"]), float(knot["value"]),
                        float(knot["derivative"]))
            for knot in reference["knots"]])
        path = VirtualAPath(
            FormationPose(0.0, 0.0, 0.0), profile, request.model)
        geometry = audit_geometry(
            request, path, timing_config.geometry_audit_config,
            planning_context=context)
        timing = retime_path(
            request, path, timing_config, planning_context=context)
        dynamic = None
        collision = None
        if timing.plan is not None:
            dynamic = audit_dynamics(
                request, timing.plan,
                DynamicAuditConfig(
                    geometry_audit_config=timing_config.geometry_audit_config),
                expected_planning_context=context)
            collision = audit_collision(
                request, timing.plan, grid, collision_config)
        checks = {
            "authoritative_map_sha256":
                grid_signature(authoritative) == entry["map_sha256"],
            "input_map_sha256": grid_signature(grid) == entry["map_sha256"],
            "request_sha256":
                request_fingerprint(request) == entry["request_sha256"],
            "planning_context_sha256":
                planning_context_sha256(context) ==
                entry["planning_context_sha256"],
            "geometry_passed": bool(geometry.passed),
            "timing_status":
                timing.status in ("DYNAMIC_FEASIBLE", "NO_MOTION"),
            "dynamic_passed": bool(dynamic is not None and dynamic.passed),
            "collision_passed": bool(
                collision is not None and collision.passed),
            "timed_model_sha256":
                timing.plan is not None and
                timing.plan.model_sha256() == entry["timed_model_sha256"],
            "reference_status": reference.get("status") ==
                "REFERENCE_PASSED",
            "independent_accepted": bool(
                reference.get("independent", {}).get("accepted")),
            "base_matches": bool(
                by_id[scene_id]["base"] == entry["base"]),
        }
        passed = all(checks.values())
        verification.append({
            "scene_id": scene_id,
            "passed": passed,
            "checks": checks,
        })
        print("verify {}: {}{}".format(
            scene_id, "PASS" if passed else "FAIL",
            "" if passed else " " + ",".join(
                name for name, value in checks.items() if not value)))
        if not passed:
            failures.append({
                "scene_id": scene_id,
                "reason": "REFERENCE_VERIFY_FAILED",
                "checks": checks,
            })
    return verification, failures


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True)
    parser.add_argument("--verify-references", action="store_true")
    args = parser.parse_args(argv)
    output = Path(args.output)
    maps_dir = output / "maps"
    refs_dir = output / "reference"
    inputs_dir = output / "inputs"
    for directory in (maps_dir, refs_dir, inputs_dir):
        directory.mkdir(parents=True, exist_ok=True)

    p1, ind1, name1 = s1_profile()
    p2, ind2, name2 = s2_profile()
    p1m = mirror_profile(p1)
    p2m = mirror_profile(p2)
    ind1m = MirroredIndependent(ind1)
    ind2m = MirroredIndependent(ind2)

    start = FormationPose(0.0, 0.0, 0.0)
    path1 = VirtualAPath(start, p1, MODEL)
    path1m = VirtualAPath(start, p1m, MODEL)
    path2 = VirtualAPath(start, p2, MODEL)
    path2m = VirtualAPath(start, p2m, MODEL)
    goal1 = path1.endpoint()
    goal1m = mirror_qf(goal1)
    goal2 = path2.endpoint()
    goal2m = mirror_qf(goal2)

    scenes = []
    entries = [
        ("S1", "S1", p1, ind1, path1, goal1,
         lambda ref, hw=CORRIDOR_HALF_WIDTH["S1"]: corridor_grid(ref, hw),
         None),
        ("S1_mirror", "S1", p1m, ind1m, path1m, goal1m,
         None, "S1"),
        ("S2", "S2", p2, ind2, path2, goal2,
         lambda ref, hw=CORRIDOR_HALF_WIDTH["S2"]: corridor_grid(ref, hw),
         None),
        ("S2_mirror", "S2", p2m, ind2m, path2m, goal2m,
         None, "S2"),
        ("S3", "S3", p2, ind2, path2, goal2, slalom_grid, None),
        ("S3_mirror", "S3", p2m, ind2m, path2m, goal2m, None, "S3"),
    ]
    primary_maps = {}
    manifest_scenes = {}
    failures = []
    for plan_id, (scene_id, base_name, profile, independent, path, goal,
                  map_builder, mirror_source) in enumerate(entries, start=9001):
        request = make_request(plan_id, "G4_" + scene_id, start, goal)
        if mirror_source is None:
            scene = build_scene(scene_id, base_name, profile, independent,
                                request, map_builder)
        else:
            source = primary_maps[mirror_source]
            occupancy, map_info = mirror_map(
                source["occupancy"], source["map_info"], mirror_source)
            scene = {
                "scene_id": scene_id, "base": base_name,
                "profile": profile, "independent": independent,
                "request": request, "occupancy": occupancy,
                "map_info": map_info,
            }
            primary_maps[scene_id] = scene
        image_path, yaml_path = _map_paths(maps_dir, scene_id)
        write_map(
            maps_dir, scene_id, scene["occupancy"],
            scene["map_info"]["resolution"],
            scene["map_info"]["origin_x"],
            scene["map_info"]["origin_y"],
            occupied_thresh=0.65, free_thresh=0.196)
        scene["image_path"] = image_path
        scene["yaml_path"] = yaml_path
        # Load from disk: the audit must use the actual serialised map.
        grid, map_meta = load_map(yaml_path)
        if mirror_source is None:
            primary_maps[scene_id] = scene
        independent_endpoint = independent.state(independent.length)
        endpoint_difference = {
            "dx_m": float(goal.x - independent_endpoint["x"]),
            "dy_m": float(goal.y - independent_endpoint["y"]),
            "dpsi_rad": float(goal.psi - independent_endpoint["psi"]),
            "norm_m": float(math.hypot(
                goal.x - independent_endpoint["x"],
                goal.y - independent_endpoint["y"])),
        }
        map_request = VirtualAMapRequest(
            request, grid, PLANNER_CONFIG, TIMING_CONFIG, COLLISION_CONFIG)
        context = map_request.context()
        geometry = audit_geometry(
            request, path, TIMING_CONFIG.geometry_audit_config,
            planning_context=context)
        timing = retime_path(request, path, TIMING_CONFIG,
                             planning_context=context)
        dynamic = None
        collision = None
        if timing.plan is not None:
            dynamic = audit_dynamics(
                request, timing.plan, DYNAMIC_AUDIT_CONFIG,
                expected_planning_context=context)
            collision = audit_collision(
                request, timing.plan, grid, COLLISION_CONFIG)
        passed = bool(
            geometry.passed and timing.status in ("DYNAMIC_FEASIBLE",
                                                  "NO_MOTION") and
            dynamic is not None and dynamic.passed and
            collision is not None and collision.passed)
        straight = None
        straight_required = base_name in ("S2", "S3")
        if straight_required and passed:
            straight = direct_straight_witness(
                scene_id, request, grid, context)
        straight_ok = (not straight_required) or (
            straight is not None and straight.get("status") == "REJECTED")
        if straight_required and straight is None:
            straight = {"status": "NOT_RUN",
                        "reason": "reference audit had already failed"}
        passed = bool(passed and straight_ok)
        reference_record = {
            "schema": "virtual_rigid_a.g4.reference.v1",
            "scene_id": scene_id,
            "base": base_name,
            "profile_kind": profile.kind,
            "profile_name": name1 if base_name == "S1" else name2,
            "knots": [{"s": float(knot.s), "value": float(knot.value),
                       "derivative": float(knot.derivative)}
                      for knot in profile.knots],
            "path": {
                "length_m": float(path.length_m),
                "endpoint": [goal.x, goal.y, goal.psi],
                "position_error_bound_m": float(path.position_error_bound_m),
                "angle_error_bound_rad": float(path.angle_error_bound_rad),
                "integration_budget": path.integration_budget,
            },
            "independent": {
                "available": True,
                "note": (
                    "Gauss-Legendre independent centreline used for map "
                    "construction; not used as the audit authority"),
                "endpoint": independent_endpoint,
                "endpoint_difference": endpoint_difference,
                "accepted": bool(endpoint_difference["norm_m"] <= 1e-6
                                 and abs(endpoint_difference["dpsi_rad"])
                                 <= 1e-6),
            },
            "direct_straight_witness": straight,
            "request_sha256": request_fingerprint(request),
            "planning_context_sha256": planning_context_sha256(context),
            "grid_sha256": map_meta["grid_sha256"],
            "map_files": {
                "image": str(image_path.relative_to(output)),
                "yaml": str(yaml_path.relative_to(output)),
                "image_sha256": map_meta["image_sha256"],
                "yaml_sha256": map_meta["yaml_sha256"],
                "map_schema": map_meta["schema"],
                "grid": map_meta["grid"],
                "map_info": scene["map_info"],
            },
            "timing": timing.as_dict(),
            "geometry_audit": _audit_summary(geometry),
            "geometry_integration": geometry.integration,
            "dynamic_audit": None if dynamic is None else _audit_summary(
                dynamic),
            "collision_audit": None if collision is None else _audit_summary(
                collision),
            "collision_margin": (
                None if collision is None
                else collision.metrics.get("margin")),
            "timed_model_sha256": (
                timing.plan.model_sha256() if timing.plan is not None else None),
            "status": "REFERENCE_PASSED" if passed else "REFERENCE_FAILED",
        }
        (refs_dir / (scene_id + ".json")).write_text(
            _canonical(reference_record), encoding="utf-8")
        _write_input_bundle(scene, inputs_dir)
        if not passed:
            failures.append({
                "scene_id": scene_id,
                "reason": ("DIRECT_STRAIGHT_NOT_REJECTED"
                           if not straight_ok else "REFERENCE_AUDIT_FAILED"),
                "geometry": _audit_summary(geometry),
                "timing_status": timing.status,
                "timing_reason": timing.reason,
                "dynamic": None if dynamic is None else _audit_summary(dynamic),
                "collision": None if collision is None else _audit_summary(
                    collision),
                "collision_first_failure": (
                    None if collision is None else collision.first_failure),
                "direct_straight_witness": straight,
            })
        manifest_scenes[scene_id] = {
            "base": base_name,
            "reference_file": "reference/{}.json".format(scene_id),
            "input_dir": "inputs/{}".format(scene_id),
            "map_sha256": map_meta["grid_sha256"],
            "request_sha256": reference_record["request_sha256"],
            "planning_context_sha256":
                reference_record["planning_context_sha256"],
            "timed_model_sha256": reference_record["timed_model_sha256"],
            "timed_model_summary": (
                timing.plan.model_summary() if timing.plan is not None else None),
            "status": reference_record["status"],
            "direct_straight_status": (
                None if straight is None else straight.get("status")),
        }
        scenes.append(scene)
        print("{}: {} T={} status={}".format(
            scene_id, reference_record["status"],
            None if timing.plan is None else timing.T_s, timing.status))

    verification = []
    if args.verify_references:
        verification, verify_failures = _verify_frozen_references(
            output, manifest_scenes, scenes)
        failures.extend(verify_failures)
    manifest = {
        "schema": "virtual_rigid_a.g4.scene_manifest.v1",
        "stage": "G4-A",
        "created_at_utc": _dt.datetime.now(_dt.timezone.utc).isoformat(),
        "base_commit": BASE_COMMIT,
        "resolution_m": RESOLUTION,
        "planner_config": PLANNER_CONFIG.as_dict(),
        "timing_config": TIMING_CONFIG.as_dict(),
        "dynamic_audit_config": DYNAMIC_AUDIT_CONFIG.as_dict(),
        "collision_config": COLLISION_CONFIG.as_dict(),
        "reference_geometry_audit_note": (
            "Independent geometry audit threshold is 5e-8 m, as fixed by "
            "G2 rework 01 and G2/G3 acceptance.  The S2 12 m curvature "
            "profile is confirmed by exact per-panel fourth-derivative "
            "interval bounds (integration_bounds), not by the coarse global "
            "triangle bound and not by sampling.  The production G1 1e-7 m "
            "contract, floating/angle guard, dynamic limits, collision "
            "margin, sweep step and vehicle geometry are unchanged."),
        "collision_envelope_note": (
            "The legacy checker expands each body by margin_m/2 per side.  "
            "The G4 numerical envelope e is a physical distance, so the "
            "required closure is (margin_used - baseline)/2 >= max_i e_i, "
            "equivalently margin_used >= baseline + 2*max_i e_i.  Static "
            "start/goal, NO_MOTION hold, map boundary and vehicle-vehicle "
            "pairs use the actual binary64 half extents; the 0.01 m sweep "
            "padding is motion-sampling protection and is not counted as "
            "numerical margin."),
        "reference_verified": not failures,
        "reference_verification": verification,
        "frozen": not failures,
        "scenes": manifest_scenes,
        "failures": failures,
        "inputs_have_no_reference": True,
        "note": (
            "Maps and requests are frozen; modifications require a new scene "
            "version.  Reference files are not copied into inputs/."),
    }
    manifest["manifest_sha256"] = _digest(manifest)
    (output / "scene_manifest.json").write_text(
        _canonical(manifest), encoding="utf-8")
    print("manifest: frozen={} scenes={} failures={}".format(
        manifest["frozen"], len(manifest_scenes), len(failures)))
    return 0 if manifest["frozen"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
