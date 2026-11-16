"""G1 analytic benchmark and diagnostics for the virtual rigid formation A.

For G1 this tool builds the mandatory sin^3 double-lane-change reference (and
its mirror) plus the L=0.5 negative case.  For G2 it runs the frozen
endpoint-only requests through the production optimiser and audits every
candidate.  The G1 analytic profile lives here and in tests; it is never
imported by the production planning package and cannot become a G2 initial
guess.  The G2 existence-reference fixture is read by this benchmark only.

Usage::

    python3 -B tools/virtual_a_benchmark.py --stage G1 --output DIR
    python3 -B tools/virtual_a_benchmark.py --stage G2 --output DIR
    python3 -B tools/virtual_a_benchmark.py --stage G3 --output DIR
    python3 -B tools/virtual_a_benchmark.py --stage G4 \
        --scene-manifest MANIFEST --output DIR
"""
from pathlib import Path
import argparse
import csv
import json
import math
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "catkin_ws/src/fwis_v3_optimizer/src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from fwis_v3_optimizer.core import Action  # noqa: E402
from fwis_v3_optimizer.formation import (  # noqa: E402
    FormationModel, FormationPose, VehicleOffset)
from fwis_v3_optimizer.formation_collision import VehicleShape  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a import (  # noqa: E402
    C2CurvatureProfile, CurvatureJet, InitialAssumption, VirtualAContractError,
    VirtualAPath, check_vehicle_curvatures, validate_initial_state)
from fwis_v3_optimizer.formation_continuous_a.contracts import (  # noqa: E402
    DynamicAuditConfig, GeometryAuditConfig, GoalPolicy, PlannerConfig,
    TimingConfig, VirtualAMapRequest, VirtualARequest,
    collision_config_from_dict, grid_signature, planner_config_from_dict,
    request_fingerprint, request_from_payload, timing_config_from_dict)
from fwis_v3_optimizer.formation_continuous_a.collision import (  # noqa: E402
    load_map)
from fwis_v3_optimizer.formation_continuous_a.pipeline import (  # noqa: E402
    plan_virtual_a)
from fwis_v3_optimizer.formation_continuous_a.audit import (  # noqa: E402
    audit_dynamics, audit_geometry)
from fwis_v3_optimizer.formation_continuous_a.optimizer import (  # noqa: E402
    make_initial_guesses, solve_open_path)
from fwis_v3_optimizer.formation_continuous_a.timing import (  # noqa: E402
    retime_path)
from fwis_v3_optimizer.formation_continuous_a.trajectory import (  # noqa: E402
    VirtualATimedPlan)
from fwis_v3_optimizer.continuous_geometry import ControlKnot  # noqa: E402
from fwis_v3_optimizer.formation_continuous_a import bounds as bounds_module  # noqa: E402
from fwis_v3_optimizer.formation_retiming import FormationTimingLimits  # noqa: E402
from fwis_v3_optimizer.retiming import MotionLimits, wheel_configuration  # noqa: E402

SCHEMA = "virtual_rigid_a.g1.benchmark.v1"
G2_SCHEMA = "virtual_rigid_a.g2.benchmark.v2"
G3_SCHEMA = "virtual_rigid_a.g3.benchmark.v1"
G4_SCHEMA = "virtual_rigid_a.g4.benchmark.v1"
G4_SCENE_SCHEMA = "virtual_rigid_a.g4.scene_manifest.v1"
G2_FIXTURES = ROOT / "tests" / "fixtures" / "virtual_a"
K_REFERENCE = 0.30
L_REFERENCE = 6.0
EXPECTED_HALF_ENDPOINT = (5.8222549849, 1.1256889273)
EXPECTED_FULL_ENDPOINT = (11.6445099698, 0.0)
EXPECTED_ETA_MAX = 0.3627598728
EXPECTED_FRONT_CURVATURE_MAX = 0.526313
EXPECTED_SIDE_CURVATURE_MAX = 0.428571
EXPECTED_SHORT_FRONT_CURVATURE_MAX = 4.34822
DEFAULT_PROTECTED_EVIDENCE_ROOT = ROOT / "evaluation/virtual_formation_a"


def _research_model():
    return FormationModel((
        VehicleOffset(1, 1.0, 0.0),
        VehicleOffset(2, 0.0, 1.0),
        VehicleOffset(3, 0.0, -1.0)))


def _finite(value, context):
    result = float(value)
    if not math.isfinite(result):
        raise VirtualAContractError("NONFINITE_VALUE", context)
    return result


class DoubleLaneChangeProfile:
    """Analytic ``sin^3`` double-lane-change curvature profile (diagnostic).

    First piece: ``k = sigma K sin^3(2 pi s/L)``; second piece mirrors the
    first.  With ``mirror=True`` every jet and ``psi`` is negated.  Jet values
    are exact analytic expressions; no Hermite fit is substituted.
    """

    kind = "analytic_sin3"
    serialization_ready = False
    numerics_version = "virtual_rigid_a.numerics.analytic_sin3.v1"

    def __init__(self, K=K_REFERENCE, L=L_REFERENCE, mirror=False):
        self.K = _finite(K, "K")
        self.L = _finite(L, "L")
        if self.K <= 0.0 or self.L <= 0.0:
            raise VirtualAContractError(
                "INVALID_ANALYTIC_PROFILE", "K and L must be positive")
        self.c = 2.0 * math.pi / self.L
        self.mirror = bool(mirror)
        self._sign = -1.0 if self.mirror else 1.0

    @property
    def length_m(self):
        return 2.0 * self.L

    @property
    def segment_count(self):
        return 2

    @property
    def domain_segments(self):
        return ((0.0, self.L), (self.L, 2.0 * self.L))

    def _locate(self, s, side="auto"):
        if side not in ("auto", "left", "right"):
            raise VirtualAContractError(
                "INVALID_SIDE", "side={!r}".format(side))
        station = _finite(s, "analytic station")
        if station < 0.0 or station > 2.0 * self.L:
            raise VirtualAContractError(
                "S_DOMAIN",
                "s={!r} is outside [0, {!r}]; no clipping".format(
                    station, 2.0 * self.L), s=station)
        two_pi = 2.0 * math.pi
        if station == 0.0:
            if side == "left":
                raise VirtualAContractError(
                    "SIDE_DOMAIN", "no left limit exists at s=0", s=station)
            return 0, 0.0
        if station == 2.0 * self.L:
            if side == "right":
                raise VirtualAContractError(
                    "SIDE_DOMAIN",
                    "no right limit exists at the terminal station", s=station)
            return 1, two_pi
        if station == self.L:
            if side == "left":
                return 0, two_pi
            return 1, 0.0
        if station < self.L:
            return 0, self.c * station
        return 1, self.c * (station - self.L)

    def jet_at(self, s, side="auto"):
        index, t = self._locate(s, side)
        sigma = 1.0 if index == 0 else -1.0
        sine, cosine = math.sin(t), math.cos(t)
        kappa = sigma * self.K * sine ** 3
        eta = sigma * 3.0 * self.K * self.c * sine * sine * cosine
        u = sigma * 3.0 * self.K * self.c ** 2 * (
            2.0 * sine * cosine * cosine - sine ** 3)
        g = sigma * 3.0 * self.K * self.c ** 3 * (
            2.0 * cosine ** 3 - 7.0 * sine * sine * cosine)
        return CurvatureJet(
            self._sign * kappa, self._sign * eta,
            self._sign * u, self._sign * g)

    def psi_at(self, s):
        index, t = self._locate(s, "auto" if s < self.L or s > 0 else "right")
        base = (self.K / self.c) * (
            2.0 / 3.0 - math.cos(t) + math.cos(t) ** 3 / 3.0)
        value = base if index == 0 else -base
        return self._sign * value

    def interval_bounds(self, s0, s1):
        s0 = _finite(s0, "interval start")
        s1 = _finite(s1, "interval end")
        if s0 < 0.0 or s1 > self.length_m or s1 < s0:
            raise VirtualAContractError(
                "S_DOMAIN", "interval outside the analytic domain")
        kappa = self.K
        eta = 3.0 * self.K * self.c
        u = 9.0 * self.K * self.c * self.c
        g = 27.0 * self.K * self.c ** 3
        left = self.jet_at(s0, "left") if s0 > 0.0 else self.jet_at(0.0, "right")
        right = (self.jet_at(s1, "right") if s1 < self.length_m
                 else self.jet_at(self.length_m, "left"))
        return {
            "s0": s0, "s1": s1,
            "segments": [{
                "index": 0, "s0": s0, "s1": s1,
                "k": [-kappa, kappa], "eta": [-eta, eta],
                "u": [-u, u], "g": [-g, g],
                "bound_source": "analytic_trig_envelope",
            }],
            "k": [-kappa, kappa], "eta": [-eta, eta],
            "u": [-u, u], "g": [-g, g],
            "left_limit": left.as_dict(), "right_limit": right.as_dict(),
            "numerics_version": self.numerics_version,
        }

    def as_dict(self):
        return {
            "kind": self.kind,
            "serialization_ready": self.serialization_ready,
            "K": self.K, "L": self.L, "mirror": self.mirror,
            "length_m": self.length_m,
            "numerics_version": self.numerics_version,
            "note": "analytic diagnostic profile; never a planner input",
        }


def quadrature_position(profile, s):
    """Position of the formation reference from strict adaptive quadrature."""
    from scipy.integrate import quad

    station = _finite(s, "quadrature station")
    boundaries = []
    knots = getattr(profile, "knots", None)
    if knots:
        boundaries = [knot.s for knot in knots]
    elif hasattr(profile, "L"):
        boundaries = [profile.L]
    points = [value for value in boundaries if 0.0 < value < station]
    options = dict(epsabs=1e-13, epsrel=1e-13, limit=400)
    if points:
        options["points"] = points
    x = quad(lambda z: math.cos(profile.psi_at(z)), 0.0, station, **options)[0]
    y = quad(lambda z: math.sin(profile.psi_at(z)), 0.0, station, **options)[0]
    return x, y


def quadrature_vehicle_positions(profile, s, model):
    """World vehicle centres from quadrature; independent of production maps."""
    x, y = quadrature_position(profile, s)
    psi = profile.psi_at(s)
    cosine, sine = math.cos(psi), math.sin(psi)
    return {
        offset.vehicle_id: (
            x + cosine * offset.x - sine * offset.y,
            y + sine * offset.x + cosine * offset.y)
        for offset in model.offsets}


def _rigidity(model, state):
    centers = {vehicle.vehicle_id: (vehicle.x, vehicle.y)
               for vehicle in state.vehicles}
    residual = model.rigidity_residual(centers)
    distances = {}
    ids = model.vehicle_ids
    for first in range(len(ids)):
        for second in range(first + 1, len(ids)):
            pair = (ids[first], ids[second])
            distances["{}-{}".format(*pair)] = math.hypot(
                centers[pair[1]][0] - centers[pair[0]][0],
                centers[pair[1]][1] - centers[pair[0]][1])
    return residual, distances


def _check(name, category, expected, actual, passed, **detail):
    return {
        "name": name, "category": category,
        "expected": expected, "actual": actual,
        "pass": bool(passed), "detail": detail,
    }


def _relative_close(first, second, relative=1e-12):
    scale = max(abs(first), abs(second), 1e-300)
    return abs(first - second) <= relative * scale


def _integration_budget_audit(profile, path):
    """Recompute the v2 Simpson budget from raw panels without its helper."""
    budget = path.integration_budget
    rows = []
    per_coordinate = 0.0
    full_total = 0
    local_total = 0
    for index, (s0, s1) in enumerate(profile.domain_segments):
        h = s1 - s0
        n = budget["panels_per_segment"][index]
        m4 = budget["m4_per_segment"][index]
        d = h / n
        width = 2.0 * d
        full_count = n // 2
        full = full_count * width ** 5 * m4 / 2880.0
        local = width ** 5 * m4 / 2880.0
        per_coordinate += full + local
        full_total += full_count
        local_total += 1
        rows.append({
            "segment": index, "h_m": h, "subintervals": n,
            "subinterval_width_m": d, "full_panel_count": full_count,
            "full_panel_width_m": width, "m4": m4,
            "full_remainder_m": full, "local_remainder_m": local,
            "reported_full_remainder_m":
                budget["simpson_full_remainders_m"][index],
            "reported_local_remainder_m":
                budget["simpson_local_remainders_m"][index],
            "reported_panel_width_m": budget["simpson_panel_widths_m"][index],
        })
    euclidean = math.sqrt(2.0) * per_coordinate
    guard = bounds_module.POSITION_ROUNDOFF_GUARD_M * (
        full_total + local_total + 8)
    guard += path.length_m * budget["angle_error_bound_rad"]
    reported_total = budget["position_error_bound_m"]
    fields_ok = (
        budget["total_panels"] == sum(row["subintervals"] for row in rows)
        and budget["total_full_simpson_panels"] == full_total
        and budget["total_local_simpson_panels"] == local_total
        and all(_relative_close(row["reported_panel_width_m"],
                                row["full_panel_width_m"])
                and _relative_close(row["reported_full_remainder_m"],
                                    row["full_remainder_m"])
                and _relative_close(row["reported_local_remainder_m"],
                                    row["local_remainder_m"])
                for row in rows))
    totals_ok = (
        _relative_close(budget["simpson_per_coordinate_m"], per_coordinate)
        and _relative_close(budget["simpson_euclidean_m"], euclidean)
        and _relative_close(budget["floating_point_guard_m"], guard)
        and reported_total >= euclidean + guard * (1.0 - 1e-12)
        and _relative_close(reported_total, euclidean + guard))
    return {
        "rows": rows,
        "recomputed_per_coordinate_m": per_coordinate,
        "recomputed_euclidean_m": euclidean,
        "recomputed_guard_m": guard,
        "reported_total_m": reported_total,
        "fields_consistent": fields_ok,
        "totals_consistent": totals_ok,
        "initial_panels_per_segment": budget["initial_panels_per_segment"],
        "final_panels_per_segment": budget["panels_per_segment"],
        "refinement_steps": budget["refinement_steps"],
        "numerics_version": budget["numerics_version"],
    }


def _partial_query_evidence(profile, path):
    """Reconstruct the actual local Simpson panel for representative s.

    The station set covers a complete-panel interior, an odd grid node, a
    station very close to a curvature-segment end, and the segment boundary.
    """
    budget = path.integration_budget
    stations = []
    for index, (s0, s1) in enumerate(profile.domain_segments):
        n = budget["panels_per_segment"][index]
        d = (s1 - s0) / n
        stations.extend([s0 + 0.37 * d, s0 + 3.0 * d, s1 - 1e-6 * d])
        if index == 0:
            stations.append(s1)
    rows = []
    for s in stations:
        index = len(profile.domain_segments) - 1
        for candidate, (s0, s1) in enumerate(profile.domain_segments):
            if s <= s1:
                index = candidate
                break
        s0, s1 = profile.domain_segments[index]
        h = s1 - s0
        n = budget["panels_per_segment"][index]
        d = h / n
        m4 = budget["m4_per_segment"][index]
        local = min(max(0.0, s - s0), h)
        whole = int(math.floor(local / d))
        if whole >= n or local >= h - 32.0 * bounds_module.EPSILON * h:
            whole = n
        if whole % 2:
            whole -= 1
        start = s0 + whole * d
        remainder = s - start
        local_count = (
            1 if remainder > 32.0 * bounds_module.EPSILON * h else 0)
        local_width = max(0.0, remainder)
        local_remainder = local_width ** 5 * m4 / 2880.0
        x, y = quadrature_position(profile, s)
        state = path.state_at_s(s)
        error = math.hypot(state.qf.x - x, state.qf.y - y)
        rows.append({
            "s": s, "segment": index, "subintervals": n,
            "subinterval_width_m": d, "whole_even_grid_index": whole,
            "local_panel_count": local_count,
            "local_panel_width_m": local_width,
            "max_local_panel_width_m": 2.0 * d,
            "local_remainder_m": local_remainder,
            "reported_local_remainder_m":
                budget["simpson_local_remainders_m"][index],
            "quadrature_error_m": error,
            "reported_total_budget_m": path.position_error_bound_m,
        })
    ok = True
    for row in rows:
        ok = ok and row["local_panel_count"] in (0, 1)
        ok = ok and row["local_panel_width_m"] <= (
            2.0 * row["subinterval_width_m"] * (1.0 + 1e-12))
        ok = ok and row["local_remainder_m"] <= (
            row["reported_local_remainder_m"] * (1.0 + 1e-12))
        ok = ok and row["quadrature_error_m"] <= row["reported_total_budget_m"]
    return {"stations": rows, "all_within_bound": ok}


def _sample_profile(profile, path, count=40001):
    maximum_eta = 0.0
    maximum_curvature = {1: 0.0, 2: 0.0, 3: 0.0}
    for index in range(count):
        s = profile.length_m * index / (count - 1)
        jet = profile.jet_at(s)
        maximum_eta = max(maximum_eta, abs(jet.eta))
        state = path.state_at_s(s)
        for vehicle in state.vehicles:
            maximum_curvature[vehicle.vehicle_id] = max(
                maximum_curvature[vehicle.vehicle_id],
                abs(vehicle.curvature))
    return maximum_eta, maximum_curvature


def _differential_evidence(profile, path, model, width, count=121):
    stations = [profile.length_m * (index + 0.5) / count
                for index in range(count)]
    worst_theta = 0.0
    worst_curvature = 0.0
    worst_position = 0.0
    for s in stations:
        positions = {
            offset: quadrature_vehicle_positions(profile, s + offset, model)
            for offset in (-2 * width, -width, 0.0, width, 2 * width)}
        state = path.state_at_s(s)
        for vehicle in state.vehicles:
            vehicle_id = vehicle.vehicle_id
            minus2 = positions[-2 * width][vehicle_id]
            minus1 = positions[-1 * width][vehicle_id]
            zero = positions[0.0][vehicle_id]
            plus1 = positions[width][vehicle_id]
            plus2 = positions[2 * width][vehicle_id]
            first = (
                (minus2[0] - 8.0 * minus1[0] + 8.0 * plus1[0] - plus2[0])
                / (12.0 * width),
                (minus2[1] - 8.0 * minus1[1] + 8.0 * plus1[1] - plus2[1])
                / (12.0 * width))
            second = (
                (-minus2[0] + 16.0 * minus1[0] - 30.0 * zero[0]
                 + 16.0 * plus1[0] - plus2[0]) / (12.0 * width * width),
                (-minus2[1] + 16.0 * minus1[1] - 30.0 * zero[1]
                 + 16.0 * plus1[1] - plus2[1]) / (12.0 * width * width))
            speed = math.hypot(first[0], first[1])
            theta = math.atan2(first[1], first[0])
            curvature = (first[0] * second[1] - first[1] * second[0]) / speed ** 3
            theta_error = abs(math.atan2(
                math.sin(theta - vehicle.theta), math.cos(theta - vehicle.theta)))
            worst_theta = max(worst_theta, theta_error)
            worst_curvature = max(worst_curvature, abs(curvature - vehicle.curvature))
            oracle = positions[0.0][vehicle_id]
            worst_position = max(worst_position, math.hypot(
                vehicle.x - oracle[0], vehicle.y - oracle[1]))
    return {
        "h_m": width,
        "points": count,
        "max_heading_error_rad": worst_theta,
        "max_curvature_error_1_per_m": worst_curvature,
        "max_production_position_error_m": worst_position,
    }


def _wheel_diagnostic(profile, state, speed=0.5):
    limits = MotionLimits()
    rows = []
    worst_angle = 0.0
    worst_factor = 0.0
    for vehicle in state.vehicles:
        action = Action(0, 1, vehicle.curvature)
        angles, factors = wheel_configuration(action, limits)
        expected_angles = []
        expected_factors = []
        for x in (limits.wheelbase / 2.0, -limits.wheelbase / 2.0):
            for y in (limits.track / 2.0, -limits.track / 2.0):
                m = 1.0 - y * vehicle.curvature
                n = x * vehicle.curvature
                expected_angles.append(math.atan2(n, m))
                expected_factors.append(math.hypot(m, n))
        for actual, expected in zip(angles, expected_angles):
            worst_angle = max(worst_angle, abs(actual - expected))
        for actual, expected in zip(factors, expected_factors):
            worst_factor = max(worst_factor, abs(actual - expected))
        speed_factor = math.hypot(*vehicle.p_s)
        rows.append({
            "vehicle_id": vehicle.vehicle_id,
            "curvature": vehicle.curvature,
            "speed_factor_abs_p_s": speed_factor,
            "center_speed_m_per_s": speed_factor * speed,
            "yaw_rate_rad_per_s": vehicle.theta_s * speed,
            "wheel_angles": [float(value) for value in angles],
            "wheel_factors": [float(value) for value in factors],
        })
    return rows, worst_angle, worst_factor


def _write_csv(path, rows, columns):
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _profile_rows(profile, path, model, count=1201):
    rows = []
    for index in range(count):
        s = profile.length_m * index / (count - 1)
        side = "auto"
        jet = profile.jet_at(s, side)
        state = path.state_at_s(s)
        row = {
            "s": s,
            "psi": state.qf.psi,
            "x": state.qf.x, "y": state.qf.y,
            "k": jet.kappa, "eta": jet.eta, "u": jet.u, "g": jet.g,
        }
        for vehicle in state.vehicles:
            prefix = "vehicle{}_".format(vehicle.vehicle_id)
            row.update({
                prefix + "x": vehicle.x,
                prefix + "y": vehicle.y,
                prefix + "theta": vehicle.theta,
                prefix + "curvature": vehicle.curvature,
                prefix + "theta_s": vehicle.theta_s,
                prefix + "p_s_x": vehicle.p_s[0],
                prefix + "p_s_y": vehicle.p_s[1],
            })
        rows.append(row)
    return rows


def _save_figures(output, profile, mirror, path, mirror_path, model):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figures = output / "figures"
    figures.mkdir()
    stations = [profile.length_m * index / 1200 for index in range(1201)]
    positive = [_profile_rows(profile, path, model, 1201)]
    mirrored = [_profile_rows(mirror, mirror_path, model, 1201)]
    psi = [row["psi"] for row in positive[0]]
    curvature = [row["k"] for row in positive[0]]
    eta = [row["eta"] for row in positive[0]]

    figure, axis = plt.subplots(figsize=(7.0, 4.0))
    axis.plot([row["x"] for row in positive[0]],
              [row["y"] for row in positive[0]], label="formation F")
    for vehicle_id in (1, 2, 3):
        axis.plot([row["vehicle{}_x".format(vehicle_id)] for row in positive[0]],
                  [row["vehicle{}_y".format(vehicle_id)] for row in positive[0]],
                  label="vehicle {}".format(vehicle_id))
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.set_title("sin^3 double lane change and mirror")
    axis.legend(loc="best", fontsize=7)
    axis.set_aspect("equal", adjustable="box")
    figure.tight_layout()
    figure.savefig(str(figures / "double_lane_change.png"), dpi=130)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(7.0, 5.2), sharex=True)
    axes[0].plot(stations, psi, label="psi (positive)")
    axes[0].plot(stations, [row["psi"] for row in mirrored[0]],
                 label="psi (mirror)")
    axes[0].set_ylabel("psi [rad]")
    axes[0].legend(loc="best", fontsize=7)
    for vehicle_id in (1, 2, 3):
        axes[1].plot(stations,
                     [row["vehicle{}_theta".format(vehicle_id)]
                      for row in positive[0]],
                     label="vehicle {} theta".format(vehicle_id))
    axes[1].set_xlabel("s [m]")
    axes[1].set_ylabel("body heading [rad]")
    axes[1].legend(loc="best", fontsize=7)
    figure.tight_layout()
    figure.savefig(str(figures / "headings.png"), dpi=130)
    plt.close(figure)

    figure, axes = plt.subplots(2, 1, figsize=(7.0, 5.2), sharex=True)
    axes[0].plot(stations, curvature, label="kappa")
    axes[0].plot(stations, eta, label="eta")
    axes[0].set_ylabel("kappa, eta")
    axes[0].legend(loc="best", fontsize=7)
    for vehicle_id in (1, 2, 3):
        axes[1].plot(stations,
                     [row["vehicle{}_curvature".format(vehicle_id)]
                      for row in positive[0]],
                     label="vehicle {} curvature".format(vehicle_id))
    axes[1].axhline(1.0 / 0.810, linestyle=":", color="black",
                    label="1/r_min")
    axes[1].axhline(-1.0 / 0.810, linestyle=":", color="black")
    axes[1].set_xlabel("s [m]")
    axes[1].set_ylabel("curvature [1/m]")
    axes[1].legend(loc="best", fontsize=7)
    figure.tight_layout()
    figure.savefig(str(figures / "curvature.png"), dpi=130)
    plt.close(figure)

    errors = []
    for s in stations:
        x, y = quadrature_position(profile, s)
        state = path.state_at_s(s)
        errors.append((math.hypot(state.qf.x - x, state.qf.y - y), s))
    figure, axis = plt.subplots(figsize=(7.0, 3.6))
    axis.semilogy([item[1] for item in errors], [item[0] for item in errors])
    axis.set_xlabel("s [m]")
    axis.set_ylabel("|x/y error| vs quadrature [m]")
    axis.set_title("production Simpson position vs independent quadrature")
    figure.tight_layout()
    figure.savefig(str(figures / "integration_error.png"), dpi=130)
    plt.close(figure)


def _resolve_output(raw_output):
    output = Path(raw_output)
    if not output.is_absolute():
        output = ROOT / output
    if output.is_symlink():
        raise VirtualAContractError(
            "OUTPUT_SYMLINK", "output directory must not be a symlink")
    output = output.resolve()
    try:
        output.relative_to(DEFAULT_PROTECTED_EVIDENCE_ROOT.resolve())
    except ValueError as exc:
        raise VirtualAContractError(
            "OUTPUT_OUTSIDE_EVIDENCE",
            "output must be below {}".format(DEFAULT_PROTECTED_EVIDENCE_ROOT)
        ) from exc
    if output == DEFAULT_PROTECTED_EVIDENCE_ROOT.resolve():
        raise VirtualAContractError(
            "OUTPUT_IS_EVIDENCE_ROOT", "output must name a run sub-directory")
    if output.exists():
        if not output.is_dir():
            raise VirtualAContractError(
                "OUTPUT_NOT_DIRECTORY", "{} is not a directory".format(output))
        if any(output.iterdir()):
            raise VirtualAContractError(
                "OUTPUT_EXISTS_NONEMPTY",
                "{} already contains files; refusing to overwrite".format(output))
    return output


def run_g1(output):
    output = _resolve_output(output)
    output.mkdir(parents=True, exist_ok=True)
    model = _research_model()
    start = FormationPose(0.0, 0.0, 0.0)
    profile = DoubleLaneChangeProfile()
    mirror = DoubleLaneChangeProfile(mirror=True)
    short = DoubleLaneChangeProfile(L=0.5)
    path = VirtualAPath(start, profile, model)
    mirror_path = VirtualAPath(start, mirror, model)
    short_path = VirtualAPath(start, short, model)
    checks = []

    half_x, half_y = quadrature_position(profile, profile.L)
    full_x, full_y = quadrature_position(profile, 2.0 * profile.L)
    for name, expected, actual in (
            ("half_endpoint_x", EXPECTED_HALF_ENDPOINT[0], half_x),
            ("half_endpoint_y", EXPECTED_HALF_ENDPOINT[1], half_y),
            ("full_endpoint_x", EXPECTED_FULL_ENDPOINT[0], full_x)):
        checks.append(_check(
            name, "endpoint", expected, actual,
            abs(actual - expected) <= 1e-6,
            absolute_error=abs(actual - expected)))
    checks.append(_check(
        "full_endpoint_y", "endpoint", "0.0", full_y,
        abs(full_y) <= 1e-6, absolute_value=abs(full_y)))
    endpoint_half = path.state_at_s(profile.L).qf
    endpoint_full = path.endpoint()
    checks.append(_check(
        "production_endpoint_matches_quadrature", "endpoint",
        [half_x, half_y, full_x, full_y],
        [endpoint_half.x, endpoint_half.y, endpoint_full.x, endpoint_full.y],
        (math.hypot(endpoint_half.x - half_x, endpoint_half.y - half_y) <= 1e-6
         and math.hypot(endpoint_full.x - full_x,
                        endpoint_full.y - full_y) <= 1e-6),
        half_error_m=math.hypot(endpoint_half.x - half_x,
                                endpoint_half.y - half_y),
        full_error_m=math.hypot(endpoint_full.x - full_x,
                                endpoint_full.y - full_y)))
    psi_half = endpoint_half.psi
    psi_full = endpoint_full.psi
    checks.append(_check(
        "endpoint_psi_zero", "endpoint", [0.0, 0.0], [psi_half, psi_full],
        abs(psi_half) <= 1e-12 and abs(psi_full) <= 1e-12,
        absolute=[abs(psi_half), abs(psi_full)]))

    signs = [profile.jet_at(s).kappa for s in (
        profile.L / 4.0, 3.0 * profile.L / 4.0,
        5.0 * profile.L / 4.0, 7.0 * profile.L / 4.0)]
    expected_signs = [1, -1, -1, 1]
    actual_signs = [1 if value > 0 else -1 if value < 0 else 0
                    for value in signs]
    checks.append(_check(
        "formation_curvature_sign_sequence", "curvature",
        expected_signs, actual_signs, actual_signs == expected_signs,
        values=signs))
    mirror_signs = [mirror.jet_at(s).kappa for s in (
        profile.L / 4.0, 3.0 * profile.L / 4.0,
        5.0 * profile.L / 4.0, 7.0 * profile.L / 4.0)]
    mirror_actual = [1 if value > 0 else -1 if value < 0 else 0
                     for value in mirror_signs]
    checks.append(_check(
        "mirror_curvature_sign_sequence", "curvature",
        [-value for value in expected_signs], mirror_actual,
        mirror_actual == [-value for value in expected_signs],
        values=mirror_signs))

    maximum_eta, maximum_curvature = _sample_profile(profile, path)
    analytic_eta_sup = 3.0 * profile.K * profile.c * 2.0 / (
        3.0 * math.sqrt(3.0))
    checks.append(_check(
        "eta_sampled_max", "curvature",
        EXPECTED_ETA_MAX, maximum_eta,
        abs(maximum_eta - EXPECTED_ETA_MAX) <= 1e-6,
        analytic_sup=analytic_eta_sup,
        sampled_below_sup=maximum_eta <= analytic_eta_sup,
        note="sampled maximum is a diagnostic; the analytic sup is reported separately"))
    checks.append(_check(
        "front_vehicle_curvature_sampled_max", "curvature",
        EXPECTED_FRONT_CURVATURE_MAX, maximum_curvature[1],
        abs(maximum_curvature[1] - EXPECTED_FRONT_CURVATURE_MAX) <= 1e-3))
    checks.append(_check(
        "side_vehicle_curvature_sampled_max", "curvature",
        EXPECTED_SIDE_CURVATURE_MAX, maximum_curvature[2],
        abs(maximum_curvature[2] - EXPECTED_SIDE_CURVATURE_MAX) <= 1e-3
        and abs(maximum_curvature[3] - EXPECTED_SIDE_CURVATURE_MAX) <= 1e-3))

    node_rows = []
    for s in (0.0, profile.L / 2.0, profile.L, 1.5 * profile.L,
              2.0 * profile.L):
        row = {"s": s, "left": None, "right": None, "auto": None}
        for side in ("left", "right", "auto"):
            try:
                row[side] = profile.jet_at(s, side).as_dict()
            except VirtualAContractError as exc:
                row[side] = {"rejected": exc.reason}
        state = path.state_at_s(s)
        residual, distances = _rigidity(model, state)
        row["rigidity_residual_m"] = residual
        row["pair_distances_m"] = distances
        row["vehicles"] = [
            {"vehicle_id": vehicle.vehicle_id,
             "x": vehicle.x, "y": vehicle.y, "theta": vehicle.theta,
             "curvature": vehicle.curvature}
            for vehicle in state.vehicles]
        node_rows.append(row)
    continuity = True
    for row in node_rows:
        left, right = row["left"], row["right"]
        if not isinstance(left, dict) or "kappa" not in left:
            continue
        if not isinstance(right, dict) or "kappa" not in right:
            continue
        continuity = continuity and abs(left["kappa"] - right["kappa"]) <= 1e-9
        continuity = continuity and abs(left["eta"] - right["eta"]) <= 1e-9
        continuity = continuity and abs(left["u"] - right["u"]) <= 1e-8
    checks.append(_check(
        "node_continuity_and_g_jump", "node", "k/eta/u continuous",
        "continuous" if continuity else "discontinuous", continuity,
        g_left=node_rows[2]["left"]["g"], g_right=node_rows[2]["right"]["g"],
        g_jump_nonzero=(abs(node_rows[2]["left"]["g"]
                            - node_rows[2]["right"]["g"]) > 1e-6)))
    rigidity_ok = all(row["rigidity_residual_m"] <= 1e-6 for row in node_rows)
    distances_ok = True
    for row in node_rows:
        distances = row["pair_distances_m"]
        distances_ok = distances_ok and abs(
            distances["1-2"] - math.sqrt(2.0)) <= 1e-9
        distances_ok = distances_ok and abs(
            distances["1-3"] - math.sqrt(2.0)) <= 1e-9
        distances_ok = distances_ok and abs(distances["2-3"] - 2.0) <= 1e-9
    checks.append(_check(
        "rigid_pair_distances", "rigidity",
        {"1-2": math.sqrt(2.0), "1-3": math.sqrt(2.0), "2-3": 2.0},
        node_rows[-1]["pair_distances_m"], distances_ok,
        max_residual_m=max(row["rigidity_residual_m"] for row in node_rows)))
    checks.append(_check(
        "rigidity_residual_within_1e_6", "rigidity", "<=1e-6",
        max(row["rigidity_residual_m"] for row in node_rows), rigidity_ok))

    mirror_rows = []
    mirror_ok = True
    for s in (1.3, 5.0, 7.7, 11.2):
        positive_state = path.state_at_s(s)
        mirror_state = mirror_path.state_at_s(s)
        entry = {"s": s}
        mirror_ok = mirror_ok and abs(
            positive_state.qf.y + mirror_state.qf.y) <= 1e-12
        mirror_ok = mirror_ok and abs(
            positive_state.qf.psi + mirror_state.qf.psi) <= 1e-12
        for vehicle_id in (1, 2, 3):
            positive_vehicle = positive_state.vehicle(vehicle_id)
            swapped = {1: 1, 2: 3, 3: 2}[vehicle_id]
            mirror_vehicle = mirror_state.vehicle(swapped)
            entry["vehicle{}".format(vehicle_id)] = {
                "positive_theta": positive_vehicle.theta,
                "mirror_theta_swapped": mirror_vehicle.theta,
                "positive_curvature": positive_vehicle.curvature,
                "mirror_curvature_swapped": mirror_vehicle.curvature,
            }
            if vehicle_id == 1:
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.x - mirror_vehicle.x) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.theta + mirror_vehicle.theta) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.curvature
                    + mirror_vehicle.curvature) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.p_s[0] - mirror_vehicle.p_s[0]) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.p_s[1] + mirror_vehicle.p_s[1]) <= 1e-12
            else:
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.x - mirror_vehicle.x) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.y + mirror_vehicle.y) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.theta + mirror_vehicle.theta) <= 1e-12
                mirror_ok = mirror_ok and abs(
                    positive_vehicle.curvature
                    + mirror_vehicle.curvature) <= 1e-12
        mirror_rows.append(entry)
    checks.append(_check(
        "mirror_center_heading_curvature_mapping", "mirror",
        "centres/headings/curvatures mirrored with car2<->car3", mirror_rows,
        mirror_ok))

    differential = [_differential_evidence(profile, path, model, width)
                    for width in (0.001, 0.0005)]
    for record in differential:
        checks.append(_check(
            "derivative_recovery_h{}".format(record["h_m"]), "differential",
            {"heading_rad": 1e-6, "curvature_1_per_m": 1e-5},
            {"heading_rad": record["max_heading_error_rad"],
             "curvature_1_per_m": record["max_curvature_error_1_per_m"]},
            (record["max_heading_error_rad"] <= 1e-6
             and record["max_curvature_error_1_per_m"] <= 1e-5),
            points=record["points"],
            production_position_error_m=record["max_production_position_error_m"]))

    integration = []
    stations = [profile.length_m * index / 200 for index in range(201)]
    worst = 0.0
    for s in stations:
        x, y = quadrature_position(profile, s)
        state = path.state_at_s(s)
        worst = max(worst, math.hypot(state.qf.x - x, state.qf.y - y))
    checks.append(_check(
        "integration_error_within_budget", "integration",
        "<= {}".format(path.position_error_bound_m), worst,
        worst <= path.position_error_bound_m,
        bound=path.position_error_bound_m,
        points=len(stations)))
    checks.append(_check(
        "position_budget_within_1e_7", "integration", "<=1e-7",
        path.position_error_bound_m, path.position_error_bound_m <= 1e-7))
    checks.append(_check(
        "angle_budget_within_1e_9", "integration", "<=1e-9",
        path.angle_error_bound_rad, path.angle_error_bound_rad <= 1e-9))
    budget_audit = _integration_budget_audit(profile, path)
    checks.append(_check(
        "simpson_budget_fields_use_2d_full_panels", "integration",
        "every reported full/local width and remainder matches raw panel "
        "arithmetic",
        {"fields_consistent": budget_audit["fields_consistent"],
         "reported_total_m": budget_audit["reported_total_m"],
         "numerics_version": budget_audit["numerics_version"]},
        budget_audit["fields_consistent"],
        rows=budget_audit["rows"],
        initial_panels_per_segment=budget_audit["initial_panels_per_segment"],
        final_panels_per_segment=budget_audit["final_panels_per_segment"],
        refinement_steps=budget_audit["refinement_steps"]))
    checks.append(_check(
        "simpson_budget_arithmetic_recomputed", "integration",
        "recomputed sum of (n/2)*R(2d)+R(2d), guard and total match report",
        {"totals_consistent": budget_audit["totals_consistent"],
         "reported_total_m": budget_audit["reported_total_m"]},
        budget_audit["totals_consistent"],
        recomputed_per_coordinate_m=budget_audit[
            "recomputed_per_coordinate_m"],
        recomputed_euclidean_m=budget_audit["recomputed_euclidean_m"],
        recomputed_guard_m=budget_audit["recomputed_guard_m"],
        reported_total_m=budget_audit["reported_total_m"]))
    partial_queries = _partial_query_evidence(profile, path)
    checks.append(_check(
        "partial_query_local_panel_bound", "integration",
        "<=1 local panel of width <=2*d and quadrature error <= total budget",
        {"stations": len(partial_queries["stations"]),
         "all_within_bound": partial_queries["all_within_bound"]},
        partial_queries["all_within_bound"],
        stations=partial_queries["stations"]))
    integration.append({
        "kind": "analytic_sin3",
        "budget": path.integration_budget,
        "budget_audit": budget_audit,
        "partial_queries": partial_queries,
        "worst_quadrature_difference_m": worst,
        "stations": len(stations),
    })

    wheel_rows, worst_wheel_angle, worst_wheel_factor = _wheel_diagnostic(
        profile, path.state_at_s(1.3))
    checks.append(_check(
        "wheel_configuration_reference", "wheel_diagnostic",
        {"angle_rad": 1e-12, "factor": 1e-12},
        {"angle_rad": worst_wheel_angle, "factor": worst_wheel_factor},
        worst_wheel_angle <= 1e-12 and worst_wheel_factor <= 1e-12,
        speed_m_per_s=0.5,
        rows=wheel_rows,
        note="geometry diagnostic only; no dynamic certification"))

    limits = FormationTimingLimits()
    first_failure = None
    short_max = {1: 0.0, 2: 0.0, 3: 0.0}
    for index in range(20001):
        s = short.length_m * index / 20000
        state = short_path.state_at_s(s)
        for vehicle in state.vehicles:
            short_max[vehicle.vehicle_id] = max(
                short_max[vehicle.vehicle_id], abs(vehicle.curvature))
        if first_failure is None:
            result = check_vehicle_curvatures(state, limits)
            if not result.passed:
                first_failure = {"s": s, "result": result.as_dict()}
    peak_within = abs(short_max[1]
                      - EXPECTED_SHORT_FRONT_CURVATURE_MAX) <= 1e-3
    side_within = abs(short_max[2]
                      - EXPECTED_SIDE_CURVATURE_MAX) <= 1e-3
    negative_ok = (
        first_failure is not None
        and first_failure["result"]["reason"] == "CURVATURE_LIMIT"
        and first_failure["result"]["location"].get("vehicle_id") == 1
        and first_failure["result"]["location"].get("curvature", 0.0)
        > first_failure["result"]["location"].get("limit", 0.0)
        and peak_within and side_within)
    checks.append(_check(
        "short_profile_front_eta_curvature_limit", "negative",
        "CURVATURE_LIMIT vehicle_id=1 with s and value",
        first_failure, negative_ok,
        peak_front=short_max[1], peak_side=short_max[2],
        expected_peak_front=EXPECTED_SHORT_FRONT_CURVATURE_MAX,
        peak_within_1e_3=peak_within,
        side_within_1e_3=side_within))

    initial = validate_initial_state(
        start, {1: 0.0, 2: 0.0, 3: 0.0},
        InitialAssumption(True, "A", True, "offline_fixture"), model=model)
    checks.append(_check(
        "offline_initial_state_accepted", "initial", "passed",
        initial.as_dict(), initial.passed))

    source = {
        "tool": "tools/virtual_a_benchmark.py",
        "stage": "G1",
        "K": profile.K, "L": profile.L,
        "profile": profile.as_dict(),
        "mirror_profile": mirror.as_dict(),
        "short_profile": short.as_dict(),
        "start_qf": {"x": 0.0, "y": 0.0, "psi": 0.0},
        "initial_assumption": InitialAssumption(
            True, "A", True, "offline_fixture").as_dict(),
        "limits": {
            "r_min_ackermann": FormationTimingLimits().r_min_ackermann,
            "limit_1_per_m": 1.0 / FormationTimingLimits().r_min_ackermann},
        "note": "analytic diagnostic only; not a planner result and not Codex acceptance",
    }
    status = "PASS" if all(check["pass"] for check in checks) else "FAIL"
    summary = {
        "schema": SCHEMA,
        "stage": "G1",
        "status": status,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "checks": checks,
        "nodes": node_rows,
        "mirror": mirror_rows,
        "differential": differential,
        "integration": integration,
        "negative": {
            "first_failure": first_failure,
            "peak_abs_curvature_by_vehicle": {
                str(vehicle_id): value
                for vehicle_id, value in short_max.items()},
            "expected_peak_front": EXPECTED_SHORT_FRONT_CURVATURE_MAX,
        },
        "wheel_diagnostic": wheel_rows,
    }
    with (output / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")
    with (output / "samples.json").open("x", encoding="utf-8") as stream:
        json.dump({"schema": "virtual_rigid_a.g1.benchmark.samples.v1",
                   "source": source, "nodes": node_rows},
                  stream, indent=2, sort_keys=True)
        stream.write("\n")
    positive_rows = _profile_rows(profile, path, model)
    mirror_rows_csv = _profile_rows(mirror, mirror_path, model)
    columns = ["s", "psi", "x", "y", "k", "eta", "u", "g"] + [
        "vehicle{}_{}".format(vehicle_id, suffix)
        for vehicle_id in (1, 2, 3)
        for suffix in ("x", "y", "theta", "curvature", "theta_s", "p_s_x",
                       "p_s_y")]
    _write_csv(output / "double_lane_change.csv", positive_rows, columns)
    _write_csv(output / "mirror.csv", mirror_rows_csv, columns)
    _save_figures(output, profile, mirror, path, mirror_path, model)
    return summary




def _g2_load_fixtures():
    import json as _json
    requests = _json.loads(
        (G2_FIXTURES / "open_requests.json").read_text(encoding="utf-8"))
    references = _json.loads(
        (G2_FIXTURES / "existence_reference.json").read_text(encoding="utf-8"))
    return requests, references


def _g2_model(base):
    return FormationModel([
        VehicleOffset(row["vehicle_id"], row["x"], row["y"])
        for row in base["model"]["offsets"]])


def _g2_shapes(base):
    return {
        row["vehicle_id"]: VehicleShape(row["length_m"], row["width_m"])
        for row in base["model"]["vehicle_shapes"]}


def _g2_request(requests, name, base):
    record = requests["requests"][name]
    goal = record["goal_qf"]
    return VirtualARequest(
        plan_id=record["plan_id"],
        formation_id=record["formation_id"],
        model=_g2_model(base),
        vehicle_shapes=_g2_shapes(base),
        start_qf=FormationPose(**base["start_qf"]),
        start_headings={int(k): float(v)
                        for k, v in base["start_headings"].items()},
        goal_qf=FormationPose(goal["x"], goal["y"], goal["psi"]),
        initial=InitialAssumption(**base["initial"]),
        goal_policy=GoalPolicy(record["goal_policy"]),
        limits=FormationTimingLimits(**base["limits"]))


def _g2_profile_from_knots(records):
    return C2CurvatureProfile.from_knots([
        ControlKnot(float(s), float(k), float(eta))
        for s, k, eta in records])


def _g2_audit_integration_audit(result):
    """Recompute the audit-side Simpson budget from its recorded fields.

    This deliberately does not import the audit helper that produced the
    budget.  It checks the documented per-segment formula, the segment/panel
    bookkeeping, the floating-point guard and the fixed stricter threshold.
    The v2 method stores one exact interval M4 bound per Simpson panel; the
    coarse triangle bound remains a diagnostic and is recomputed separately.
    """
    if result is None or result.audit is None or result.path is None:
        return {"available": False, "ok": False, "reason": "no audit/path"}
    integration = getattr(result.audit, "integration", None)
    budget = (integration or {}).get("audit_integration")
    profile = result.path.profile
    if not isinstance(budget, dict) or profile is None:
        return {"available": False, "ok": False,
                "reason": "audit_integration budget missing"}
    method = budget.get("method")
    segments = budget.get("segments") or []
    rows = []
    ok = len(segments) == profile.segment_count
    per_coordinate = 0.0
    full_panels = 0
    local_panels = 0
    total_subintervals = 0
    for index, segment in enumerate(segments):
        h = float(segment["h"])
        panels = int(segment["subintervals"])
        k_upper = float(segment["k_upper"])
        eta_upper = float(segment["eta_upper"])
        u_upper = float(segment["u_upper"])
        g_upper = float(segment["g_upper"])
        coarse_recomputed = (g_upper + 4.0 * k_upper * u_upper
                             + 3.0 * eta_upper * eta_upper
                             + 6.0 * k_upper * k_upper * eta_upper
                             + k_upper ** 4)
        width = 2.0 * h / panels
        if method == "independent_composite_simpson_v2_exact_panel":
            panel_m4s = [float(value) for value in
                         segment.get("panel_m4_upper_m", [])]
            m4 = max(panel_m4s) if panel_m4s else math.nan
            full = sum(width ** 5 * value / 2880.0
                       for value in panel_m4s)
            local = width ** 5 * m4 / 2880.0
            segment_remainder = full + local
            row_ok = (
                panels >= 2 and panels % 2 == 0
                and h > 0.0
                and len(panel_m4s) == panels // 2
                and all(math.isfinite(value) and value >= 0.0
                        for value in panel_m4s)
                and math.isfinite(m4)
                and _relative_close(
                    float(segment["m4_upper"]), m4, 1e-12)
                and _relative_close(
                    float(segment["maximum_panel_m4_upper_m"]), m4,
                    1e-12)
                and _relative_close(
                    float(segment["coarse_triangle_m4_upper_m"]),
                    coarse_recomputed, 1e-12)
                and _relative_close(
                    float(segment["panel_width_m"]), width, 1e-12)
                and _relative_close(
                    float(segment["full_remainder_m"]), full, 1e-12)
                and _relative_close(
                    float(segment["local_remainder_m"]), local, 1e-12)
                and _relative_close(
                    float(segment["remainder_m"]), segment_remainder,
                    1e-12))
            row = {
                "segment_index": index,
                "subintervals": panels,
                "panel_count": len(panel_m4s),
                "panel_width_m": width,
                "maximum_panel_m4_upper_m": m4,
                "coarse_triangle_m4_upper_m": coarse_recomputed,
                "full_remainder_recorded_m": segment["full_remainder_m"],
                "full_remainder_recomputed_m": full,
                "local_remainder_recorded_m": segment["local_remainder_m"],
                "local_remainder_recomputed_m": local,
                "remainder_recorded_m": segment["remainder_m"],
                "remainder_recomputed_m": segment_remainder,
                "consistent": row_ok,
            }
        else:
            m4 = float(segment["m4_upper"])
            panel_remainder = width ** 5 * m4 / 2880.0
            segment_remainder = (panels // 2 + 1) * panel_remainder
            row_ok = (
                panels >= 2 and panels % 2 == 0
                and h > 0.0 and m4 >= 0.0
                and _relative_close(m4, coarse_recomputed, 1e-12)
                and _relative_close(float(segment["panel_width_m"]), width,
                                    1e-12)
                and _relative_close(float(segment["remainder_m"]),
                                    segment_remainder, 1e-12))
            row = {
                "segment_index": index,
                "subintervals": panels,
                "panel_width_m": width,
                "m4_upper_recorded": m4,
                "m4_upper_recomputed": coarse_recomputed,
                "remainder_recorded_m": segment["remainder_m"],
                "remainder_recomputed_m": segment_remainder,
                "consistent": row_ok,
            }
        ok = ok and row_ok
        per_coordinate += segment_remainder
        full_panels += panels // 2
        local_panels += 1
        total_subintervals += panels
        rows.append(row)
    euclidean = math.sqrt(2.0) * per_coordinate
    angle_bound = float(budget.get("angle_bound_rad", math.nan))
    guard = (1e-10 * (full_panels + local_panels + 8)
             + result.path.length_m * angle_bound)
    total = euclidean + guard
    ok = ok and _relative_close(
        float(budget["per_coordinate_remainder_m"]), per_coordinate, 1e-12)
    ok = ok and _relative_close(
        float(budget["euclidean_remainder_m"]), euclidean, 1e-12)
    ok = ok and _relative_close(
        float(budget["roundoff_and_angle_guard_m"]), guard, 1e-12)
    ok = ok and _relative_close(
        float(budget["total_budget_m"]), total, 1e-12)
    ok = ok and int(budget["total_subintervals"]) == total_subintervals
    ok = ok and int(budget["full_simpson_panels"]) == full_panels
    ok = ok and int(budget["local_simpson_panels"]) == local_panels
    threshold = float(budget.get("fixed_threshold_m", math.inf))
    strict = (
        budget.get("confirmed") is True
        and threshold <= 5e-8
        and float(budget["total_budget_m"]) <= threshold
        and angle_bound <= 1e-9
        and method in (
            "independent_composite_simpson_v1",
            "independent_composite_simpson_v2_exact_panel"))
    return {
        "available": True,
        "ok": bool(ok),
        "strict_threshold_ok": bool(strict),
        "method": method,
        "fixed_threshold_m": threshold,
        "total_budget_m": budget.get("total_budget_m"),
        "per_coordinate_remainder_m": budget.get(
            "per_coordinate_remainder_m"),
        "euclidean_remainder_m": budget.get("euclidean_remainder_m"),
        "guard_m": budget.get("roundoff_and_angle_guard_m"),
        "angle_bound_rad": angle_bound,
        "total_subintervals": budget.get("total_subintervals"),
        "refinement_steps": budget.get("refinement_steps"),
        "segments": rows,
    }


def _g2_result_record(name, request, result):
    record = {
        "name": name,
        "request": request.as_dict(),
        "request_sha256": request_fingerprint(request),
        "result": result.as_dict(),
        "path_summary": None,
        "endpoint": None,
        "profile": None,
        "audit": None,
        "initial_guesses": None,
    }
    try:
        record["initial_guesses"] = [
            guess.reconstruct() for guess in make_initial_guesses(
                request, PlannerConfig(**{
                    key: value for key, value in
                    _g2_load_fixtures()[0]["planner_config"].items()}))]
    except Exception as exc:
        record["initial_guesses"] = {
            "error": "{}: {}".format(type(exc).__name__, exc)}
    if result.path is not None:
        path = result.path
        endpoint = path.endpoint()
        record["path_summary"] = {
            "kind": path.kind,
            "length_m": path.length_m,
            "position_error_bound_m": path.position_error_bound_m,
            "angle_error_bound_rad": path.angle_error_bound_rad,
            "integration_budget": path.integration_budget,
        }
        record["endpoint"] = {
            "x": endpoint.x, "y": endpoint.y, "psi": endpoint.psi}
        if path.profile is not None:
            record["profile"] = path.profile.as_dict()
    if result.audit is not None:
        record["audit"] = result.audit.as_dict()
    return record


def run_g2(output):
    requests_doc, references = _g2_load_fixtures()
    base = requests_doc["base"]
    config_values = dict(requests_doc["planner_config"])
    config = PlannerConfig(**config_values)
    output = _resolve_output(output)
    output.mkdir(parents=True, exist_ok=True)
    names = ("E1", "E2", "E3", "E4", "E5", "E6",
             "STRAIGHT_LINE", "STRAIGHT_POLICY")
    results = {}
    records = {}
    checks = []
    for name in names:
        request = _g2_request(requests_doc, name, base)
        started = __import__("time").monotonic()
        result = solve_open_path(request, config)
        wall = __import__("time").monotonic() - started
        results[name] = result
        record = _g2_result_record(name, request, result)
        record["outer_wall_time_s"] = wall
        records[name] = record
        checks.append(_check(
            name + "_status", "production_solve", "GEOMETRY_FEASIBLE",
            result.status, result.status == "GEOMETRY_FEASIBLE"))
        checks.append(_check(
            name + "_independent_audit", "production_audit", "passed",
            result.audit.as_dict() if result.audit is not None else None,
            bool(result.audit is not None and result.audit.passed)))
        if result.path is None:
            checks.append(_check(
                name + "_attempt_count", "production_solve", 6,
                len(result.attempts), len(result.attempts) == 6))
            continue
        endpoint = result.path.endpoint()
        goal = request.goal_qf
        pos_error = math.hypot(endpoint.x - goal.x, endpoint.y - goal.y)
        heading_error = abs(math.atan2(
            math.sin(endpoint.psi - goal.psi),
            math.cos(endpoint.psi - goal.psi)))
        checks.append(_check(
            name + "_endpoint_position", "endpoint", 0.005,
            pos_error, pos_error <= 0.005))
        checks.append(_check(
            name + "_endpoint_heading", "endpoint",
            math.radians(0.2), heading_error,
            heading_error <= math.radians(0.2)))
        checks.append(_check(
            name + "_attempt_count", "production_solve", 6,
            len(result.attempts), len(result.attempts) == 6))
        attempts_ok = all(
            attempt.status == "SOLVER_SUCCESS_AUDITED"
            and attempt.audit is not None and attempt.audit.passed
            for attempt in result.attempts)
        checks.append(_check(
            name + "_all_attempts_audited", "production_solve",
            "all six audited", [attempt.status for attempt in result.attempts],
            attempts_ok))
        sample = result.audit.metrics.get("sample_audit", {})
        checks.append(_check(
            name + "_rigidity", "audit", 1e-6,
            sample.get("max_pairwise_rigidity_error_m"),
            sample.get("max_pairwise_rigidity_error_m", math.inf) <= 1e-6))
        checks.append(_check(
            name + "_integration_position_budget", "audit", 1e-7,
            result.audit.integration["position_budget_m"],
            result.audit.integration["position_budget_m"] <= 1e-7))
        checks.append(_check(
            name + "_integration_angle_budget", "audit", 1e-9,
            result.audit.integration["angle_budget_rad"],
            result.audit.integration["angle_budget_rad"] <= 1e-9))
        audit_integration = _g2_audit_integration_audit(result)
        checks.append(_check(
            name + "_audit_integration_budget_recomputed",
            "audit_integration", "segments and arithmetic consistent",
            audit_integration.get("ok"), bool(audit_integration.get("ok")),
            fixed_threshold_m=audit_integration.get("fixed_threshold_m"),
            total_budget_m=audit_integration.get("total_budget_m"),
            per_coordinate_remainder_m=audit_integration.get(
                "per_coordinate_remainder_m"),
            euclidean_remainder_m=audit_integration.get(
                "euclidean_remainder_m"),
            guard_m=audit_integration.get("guard_m"),
            total_subintervals=audit_integration.get("total_subintervals"),
            refinement_steps=audit_integration.get("refinement_steps"),
            segments=audit_integration.get("segments")))
        checks.append(_check(
            name + "_audit_budget_stricter_than_g1", "audit_integration",
            "confirmed and total < G1 1e-7 m with fixed threshold",
            {"method": audit_integration.get("method"),
             "fixed_threshold_m": audit_integration.get("fixed_threshold_m"),
             "total_budget_m": audit_integration.get("total_budget_m")},
            bool(audit_integration.get("strict_threshold_ok"))))
        deadline_summary = getattr(result, "deadline_summary", {}) or {}
        checks.append(_check(
            name + "_soft_deadline_policy_recorded", "soft_deadline",
            "not triggered, no hard-real-time claim",
            {"triggered": deadline_summary.get("triggered"),
             "hard_real_time_guarantee": deadline_summary.get(
                 "hard_real_time_guarantee"),
             "soft_budget_s": deadline_summary.get("soft_budget_s")},
            (deadline_summary.get("hard_real_time_guarantee") is False
             and deadline_summary.get("soft_budget_s", 0) > 0.0
             and deadline_summary.get("triggered") is False)))
        cmax = 1.0 / request.limits.r_min_ackermann
        max_c = 0.0
        max_wheel = 0.0
        for segment in result.audit.whole_segment_bounds:
            max_c = max(max_c, max(
                segment["max_abs_C_by_vehicle"].values()))
            max_wheel = max(max_wheel,
                            segment["max_static_wheel_angle_rad"])
        checks.append(_check(
            name + "_whole_segment_curvature", "whole_segment", cmax,
            max_c, max_c <= cmax))
        checks.append(_check(
            name + "_whole_segment_wheel_angle", "whole_segment",
            request.limits.steer_angle_max, max_wheel,
            max_wheel <= request.limits.steer_angle_max))
        if name == "STRAIGHT_POLICY":
            terminal = result.path.profile.knots[-1]
            checks.append(_check(
                name + "_terminal_kappa_zero", "goal_policy", 0.0,
                terminal.value, abs(terminal.value) <= 1e-9))
            checks.append(_check(
                name + "_terminal_eta_zero", "goal_policy", 0.0,
                terminal.derivative, abs(terminal.derivative) <= 1e-9))
    # Reference consistency is independently stored and is not used by the
    # production solver; this check only binds the request goals to it.
    for name, reference in references["quadratic_profiles"].items():
        goal = requests_doc["requests"][name]["goal_qf"]
        exact = reference["exact_endpoint"]
        checks.append(_check(
            name + "_reference_goal_consistency", "existence_reference",
            1e-9, "position",
            math.hypot(goal["x"] - exact["x"], goal["y"] - exact["y"])
            <= 1e-9 and abs(goal["psi"] - exact["psi"]) <= 1e-12))
    # Real segment-interior violation with legal nodes must be rejected.
    bad_request = _g2_request(requests_doc, "STRAIGHT_LINE", base)
    bad_profile = _g2_profile_from_knots([(0.0, 0.0, 0.0), (0.1, 0.0, 10.0)])
    bad_path = VirtualAPath(FormationPose(0.0, 0.0, 0.0), bad_profile,
                            _g2_model(base))
    bad_endpoint = bad_path.endpoint()
    bad_request = _g2_request(
        requests_doc, "STRAIGHT_LINE", base)
    bad_request = VirtualARequest(
        plan_id=bad_request.plan_id,
        formation_id=bad_request.formation_id,
        model=bad_request.model,
        vehicle_shapes=bad_request.vehicle_shapes,
        start_qf=bad_request.start_qf,
        start_headings=bad_request.start_headings,
        goal_qf=FormationPose(bad_endpoint.x, bad_endpoint.y,
                              bad_endpoint.psi),
        initial=bad_request.initial,
        goal_policy=GoalPolicy.FREE,
        limits=bad_request.limits)
    bad_path.request_sha256 = request_fingerprint(bad_request)
    bad_report = audit_geometry(bad_request, bad_path)
    checks.append(_check(
        "segment_interior_curvature_negative", "negative",
        "CURVATURE_LIMIT", bad_report.reason,
        bad_report.reason == "CURVATURE_LIMIT"))
    # The finite-value counterexample reported by Codex: a real legal 6 m
    # straight path bound to a 10 m goal must be rejected both when the audit
    # config contains a NaN tolerance and under the frozen default config.
    counter_base = _g2_request(requests_doc, "STRAIGHT_LINE", base)
    counter_request = VirtualARequest(
        plan_id=counter_base.plan_id,
        formation_id=counter_base.formation_id,
        model=counter_base.model,
        vehicle_shapes=counter_base.vehicle_shapes,
        start_qf=counter_base.start_qf,
        start_headings=counter_base.start_headings,
        goal_qf=FormationPose(10.0, 0.0, 0.0),
        initial=counter_base.initial,
        goal_policy=GoalPolicy.FREE,
        limits=counter_base.limits)
    counter_path = VirtualAPath(
        FormationPose(0.0, 0.0, 0.0),
        _g2_profile_from_knots([(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)]),
        _g2_model(base))
    counter_path.request_sha256 = request_fingerprint(counter_request)
    nan_report = audit_geometry(
        counter_request, counter_path,
        GeometryAuditConfig(position_tolerance_m=float("nan")))
    checks.append(_check(
        "counterexample_nan_tolerance_rejected", "negative",
        "INVALID_AUDIT_CONFIG", nan_report.reason,
        nan_report.reason == "INVALID_AUDIT_CONFIG"))
    default_report = audit_geometry(counter_request, counter_path)
    counter_error = default_report.location.get("error")
    checks.append(_check(
        "counterexample_default_endpoint_rejected", "negative",
        "ENDPOINT_POSITION_ERROR with 4 m error",
        {"reason": default_report.reason,
         "constraint": default_report.location.get("constraint"),
         "error_m": counter_error},
        (default_report.reason == "ENDPOINT_POSITION_ERROR"
         and default_report.location.get("constraint") == "ENDPOINT_POSITION"
         and counter_error is not None
         and abs(counter_error - 4.0) <= 1e-12)))
    endpoint_counter = counter_path.endpoint()
    valid_request = VirtualARequest(
        plan_id=counter_base.plan_id,
        formation_id=counter_base.formation_id,
        model=counter_base.model,
        vehicle_shapes=counter_base.vehicle_shapes,
        start_qf=counter_base.start_qf,
        start_headings=counter_base.start_headings,
        goal_qf=FormationPose(
            endpoint_counter.x, endpoint_counter.y, endpoint_counter.psi),
        initial=counter_base.initial,
        goal_policy=GoalPolicy.FREE,
        limits=counter_base.limits)
    valid_path = VirtualAPath(
        FormationPose(0.0, 0.0, 0.0),
        _g2_profile_from_knots([(0.0, 0.0, 0.0), (6.0, 0.0, 0.0)]),
        _g2_model(base))
    valid_path.request_sha256 = request_fingerprint(valid_request)
    valid_report = audit_geometry(valid_request, valid_path)
    checks.append(_check(
        "counterexample_valid_six_metre_path_passes", "negative",
        "passed", valid_report.reason, valid_report.passed))
    status = "PASS" if all(check["pass"] for check in checks) else "FAIL"
    summary = {
        "schema": G2_SCHEMA,
        "stage": "G2",
        "status": status,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "checks": checks,
        "config": config.as_dict(),
        "request_names": list(names),
        "results": {
            name: {
                "status": records[name]["result"]["status"],
                "wall_time_s": records[name]["result"]["wall_time_s"],
                "outer_wall_time_s": records[name]["outer_wall_time_s"],
                "solver_used": records[name]["result"]["solver_used"],
                "request_sha256": records[name]["request_sha256"],
                "attempts": records[name]["result"]["attempts"],
                "audit_integration": (
                    (records[name]["audit"] or {}).get("integration", {})
                    or {}).get("audit_integration"),
                "deadline_summary": records[name]["result"].get(
                    "deadline_summary"),
                "path_summary": records[name]["path_summary"],
                "endpoint": records[name]["endpoint"],
                "goal": requests_doc["requests"][name]["goal_qf"],
                "goal_policy": requests_doc["requests"][name]["goal_policy"],
            }
            for name in names
        },
        "reference": {
            "path": "tests/fixtures/virtual_a/existence_reference.json",
            "not_used_by_production_solve": True,
            "quadratic_profiles": references["quadratic_profiles"],
            "straight_spline": references["straight_spline"],
        },
        "negative": {
            "segment_interior": bad_report.as_dict(),
            "counterexample": {
                "request_goal": [10.0, 0.0, 0.0],
                "path_length_m": counter_path.length_m,
                "nan_tolerance": nan_report.as_dict(),
                "default_config": default_report.as_dict(),
                "valid_six_metre": valid_report.as_dict(),
            },
        },
        "nonclaims": [
            "GEOMETRY_FEASIBLE only",
            "no timing, dynamics, map, collision certification, ROS or hardware claim",
            "the reference profiles are not solver seeds",
        ],
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "results.json").write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def _g3_canonical(document):
    return json.dumps(document, sort_keys=True, separators=(",", ":"))


def _g3_tamper_reason(payload, mutate):
    document = json.loads(payload)
    mutate(document)
    try:
        VirtualATimedPlan.from_model_json(
            _g3_canonical(document))
    except VirtualAContractError as exc:
        return exc.reason
    return None


def _g3_shortened_plan(request, plan, factor=0.25):
    """A coherent-but-too-short model with understated T0 metric bounds.

    The selection arithmetic is kept internally consistent at the shortened
    duration so the model is not rejected for a self-contradictory table; the
    independent audit must still find the true overlimit by recomputing the
    raw-profile dynamics.
    """
    selection = json.loads(json.dumps(plan.to_model_dict()["timing"][
        "selection"]))
    duration = plan.duration_s * factor
    required = duration / float(selection["safety_factor"])
    scale = required / float(selection["required_duration_s"])
    metrics = selection["metrics_at_T0_s"]
    for record in metrics.values():
        order = int(record["order"])
        record["upper_at_T0_s"] = float(record["upper_at_T0_s"]) * \
            scale ** order
        record["required_duration_s"] = (
            float(record["upper_at_T0_s"]) / float(record["limit"])) ** \
            (1.0 / order)
    selection["required_duration_s"] = max(
        record["required_duration_s"] for record in metrics.values())
    selection["selected_duration_s"] = duration
    return VirtualATimedPlan._build_motion(
        request, plan.path, duration, selection)


def _g3_hold_record(requests_doc, base):
    import dataclasses
    template = _g2_request(requests_doc, "STRAIGHT_LINE", base)
    request = dataclasses.replace(
        template, goal_qf=FormationPose(0.0, 0.0, 0.0),
        goal_policy=GoalPolicy.FREE)
    path = VirtualAPath.hold(FormationPose(0.0, 0.0, 0.0), request.model)
    path.request_sha256 = request_fingerprint(request)
    timing = retime_path(request, path, TimingConfig())
    report = audit_dynamics(
        request, timing.plan, DynamicAuditConfig())         if timing.plan is not None else None
    payload = timing.plan.to_model_json() if timing.plan is not None else None
    rebuilt = VirtualATimedPlan.from_model_json(
        payload, expected_request=request) if payload is not None else None
    sample = timing.plan.sample([0.0]) if timing.plan is not None else None
    invalid_samples = []
    if timing.plan is not None:
        for times in ((), (0.0, 1.0), (1.0,), (-1.0,)):
            try:
                timing.plan.sample(times)
            except VirtualAContractError:
                invalid_samples.append({"times": list(times), "rejected": True})
            else:
                invalid_samples.append({"times": list(times),
                                        "rejected": False})
    record = {
        "name": "NO_MOTION_HOLD",
        "request": request.as_dict(),
        "request_sha256": request_fingerprint(request),
        "timing": timing.as_dict(),
        "model_json": payload,
        "model_sha256": timing.plan.model_sha256()
        if timing.plan is not None else None,
        "dynamic_audit": report.as_dict() if report is not None else None,
        "roundtrip": {
            "available": rebuilt is not None,
            "equal_json": rebuilt is not None and rebuilt.to_model_json() == payload,
            "equal_digest": rebuilt is not None and timing.plan is not None
            and rebuilt.model_sha256() == timing.plan.model_sha256(),
        },
        "sample": {
            "times": list(sample.times) if sample is not None else None,
            "authoritative": sample.authoritative if sample is not None else None,
            "invalid": invalid_samples,
        },
    }
    checks = [
        _check("NO_MOTION_HOLD_status", "hold_timing", "NO_MOTION",
               timing.status, timing.status == "NO_MOTION"),
        _check("NO_MOTION_HOLD_audit", "hold_dynamics", "passed",
               report.as_dict() if report is not None else None,
               bool(report is not None and report.passed)),
        _check("NO_MOTION_HOLD_zero_time", "hold_timing",
               {"S_m": 0.0, "T_s": 0.0},
               ({"S_m": timing.total_S_m, "T_s": timing.T_s}
                if timing.plan is not None else None),
               timing.plan is not None and timing.total_S_m == 0.0
               and timing.T_s == 0.0),
        _check("NO_MOTION_HOLD_one_sample", "hold_sample",
               {"times": [0.0], "authoritative": False},
               ({"times": list(sample.times),
                 "authoritative": sample.authoritative}
                if sample is not None else None),
               sample is not None and sample.times == (0.0,)
               and sample.authoritative is False),
        _check("NO_MOTION_HOLD_invalid_samples", "hold_sample",
               "all rejected", invalid_samples,
               bool(invalid_samples) and all(item["rejected"]
                                              for item in invalid_samples)),
        _check("NO_MOTION_HOLD_roundtrip", "serialization",
               {"equal_json": True, "equal_digest": True},
               record["roundtrip"],
               record["roundtrip"]["equal_json"]
               and record["roundtrip"]["equal_digest"]),
        _check("NO_MOTION_HOLD_no_execution_claim", "nonclaim",
               "no executable/collision claim",
               {"executable": report.metrics.get("executable")
                if report is not None else None,
                "collision_checked": report.metrics.get("collision_checked")
                if report is not None else None},
               report is not None
               and report.metrics.get("executable") is False
               and report.metrics.get("collision_checked") is False),
    ]
    return record, checks


def run_g3(output):
    requests_doc, _references = _g2_load_fixtures()
    base = requests_doc["base"]
    config_values = dict(requests_doc["planner_config"])
    config = PlannerConfig(**config_values)
    output = _resolve_output(output)
    output.mkdir(parents=True, exist_ok=True)
    names = ("E1", "E2", "E3", "E4", "E5", "E6",
             "STRAIGHT_LINE", "STRAIGHT_POLICY")
    records = {}
    checks = []
    result_overview = {}
    for name in names:
        request = _g2_request(requests_doc, name, base)
        started = __import__("time").monotonic()
        solve_result = solve_open_path(request, config)
        solve_wall = __import__("time").monotonic() - started
        timing = retime_path(request, solve_result.path, TimingConfig())             if solve_result.path is not None else None
        report = audit_dynamics(
            request, timing.plan, DynamicAuditConfig())             if timing is not None and timing.plan is not None else None
        record = {
            "name": name,
            "request": request.as_dict(),
            "request_sha256": request_fingerprint(request),
            "solve": {
                "status": solve_result.status,
                "solver_used": solve_result.solver_used,
                "wall_time_s": solve_result.wall_time_s,
                "outer_wall_time_s": solve_wall,
            },
            "path_summary": None,
            "timing": timing.as_dict() if timing is not None else None,
            "model_json": None,
            "model_sha256": None,
            "dynamic_audit": report.as_dict() if report is not None else None,
            "short_duration": None,
            "tamper": None,
            "roundtrip": None,
            "sample": None,
        }
        if solve_result.path is not None:
            path = solve_result.path
            endpoint = path.endpoint()
            record["path_summary"] = {
                "kind": path.kind,
                "length_m": path.length_m,
                "position_error_bound_m": path.position_error_bound_m,
                "angle_error_bound_rad": path.angle_error_bound_rad,
            }
            checks.append(_check(
                name + "_solve", "production_solve", "GEOMETRY_FEASIBLE",
                solve_result.status,
                solve_result.status == "GEOMETRY_FEASIBLE"))
            checks.append(_check(
                name + "_endpoint", "geometry",
                [request.goal_qf.x, request.goal_qf.y, request.goal_qf.psi],
                [endpoint.x, endpoint.y, endpoint.psi],
                math.hypot(endpoint.x - request.goal_qf.x,
                           endpoint.y - request.goal_qf.y) <= 0.005
                and abs(math.atan2(
                    math.sin(endpoint.psi - request.goal_qf.psi),
                    math.cos(endpoint.psi - request.goal_qf.psi))) <= 1e-3))
        if timing is not None and timing.plan is not None:
            plan = timing.plan
            payload = plan.to_model_json()
            record["model_json"] = payload
            record["model_sha256"] = plan.model_sha256()
            record["timing"]["selection"] = plan.to_model_dict()["timing"][
                "selection"]
            checks.append(_check(
                name + "_retime", "timing", "DYNAMIC_FEASIBLE",
                timing.status, timing.status == "DYNAMIC_FEASIBLE"))
            checks.append(_check(
                name + "_duration_selection", "timing",
                "T >= required * safety",
                {"T_s": timing.T_s,
                 "required_s": timing.bounds_summary.get(
                     "required_duration_s"),
                 "safety_factor": plan.to_model_dict()["timing"]["selection"][
                     "safety_factor"],
                 "rescalings": timing.rescalings},
                timing.T_s + 1e-12 * max(1.0, timing.T_s) >=
                timing.bounds_summary.get("required_duration_s", 0.0)
                * plan.to_model_dict()["timing"]["selection"][
                    "safety_factor"]))
            checks.append(_check(
                name + "_audit", "dynamic_audit", "passed",
                report.as_dict() if report is not None else None,
                bool(report is not None and report.passed)))
            checks.append(_check(
                name + "_audit_scope", "dynamic_audit", "DYNAMICS",
                report.scope if report is not None else None,
                report is not None and report.scope == "DYNAMICS"))
            checks.append(_check(
                name + "_audit_sha", "dynamic_audit", plan.model_sha256(),
                report.timing.get("model_sha256")
                if report is not None else None,
                report is not None
                and report.timing.get("model_sha256") == plan.model_sha256()))
            if report is not None:
                metric_names = {
                    "speed", "acceleration", "jerk", "yaw_rate",
                    "yaw_acceleration", "yaw_jerk", "lateral_acceleration",
                    "wheel_speed", "steer_rate", "steer_acceleration"}
                segment_count = len(report.whole_segment_bounds)
                certified = segment_count == plan.path.profile.segment_count                     and all(segment["certified"]
                            for segment in report.whole_segment_bounds)
                checks.append(_check(
                    name + "_whole_domain_certificate", "dynamic_audit",
                    {"segments": plan.path.profile.segment_count,
                     "all_certified": True},
                    {"segments": segment_count, "all_certified": certified,
                     "leaves": report.metrics.get("leaves")},
                    certified))
                checks.append(_check(
                    name + "_all_metrics_within_limits", "dynamic_audit",
                    "all ten metrics and wheel angle <= 1.0 utilization",
                    {"max_utilization": report.metrics.get("max_utilization"),
                     "max_wheel_angle_rad": report.metrics.get(
                         "max_wheel_angle_rad")},
                    bool(report.metrics.get("max_utilization"))
                    and set(report.metrics["max_utilization"]) ==
                    metric_names | {"wheel_angle"}
                    and all(value <= 1.0 for value in
                            report.metrics["max_utilization"].values())
                    and report.metrics.get("max_wheel_angle_rad", math.inf)
                    <= request.limits.steer_angle_max))
                nodes = report.metrics.get("nodes", {})
                checks.append(_check(
                    name + "_node_one_sided_motion", "dynamic_audit",
                    "every internal node is motion on both sides",
                    {"count": nodes.get("internal_node_count"),
                     "max_jump": nodes.get("max_jump_magnitude"),
                     "max_continuity_error": nodes.get(
                         "max_continuity_error")},
                    nodes.get("internal_node_count", -1) ==
                    plan.path.profile.segment_count - 1
                    and all(record_["left_region"] == "motion"
                            and record_["right_region"] == "motion"
                            and record_["auto_region"] == "motion"
                            for record_ in nodes.get("records", ()))
                    and nodes.get("max_continuity_error", math.inf) <=
                    DynamicAuditConfig().continuity_tolerance))
                inverse = report.metrics.get("time_inverse", {})
                checks.append(_check(
                    name + "_inverse_100", "dynamic_audit",
                    {"samples": 100, "max_error_s": 1e-9},
                    {"samples": inverse.get("sample_count"),
                     "max_error_s": inverse.get("max_error_s")},
                    inverse.get("sample_count") == 100
                    and inverse.get("max_error_s", math.inf) <= 1e-9))
                differential = report.metrics.get("differential", {})
                checks.append(_check(
                    name + "_differential", "dynamic_audit",
                    "two dt steps within tolerances",
                    {"steps": [step["dt_s"] for step in
                               differential.get("steps", ())],
                     "checked": differential.get("checked_sample_count"),
                     "max_errors": differential.get("max_errors")},
                    len(differential.get("steps", ())) == 2
                    and all(value <= differential.get("tolerances", {}).get(
                        key, math.inf)
                            for key, value in
                            differential.get("max_errors", {}).items())))
                checks.append(_check(
                    name + "_nonclaims", "nonclaim",
                    {"collision_checked": False, "executable": False,
                     "map_used": False, "ros_used": False},
                    {key: report.metrics.get(key) for key in (
                        "collision_checked", "executable", "map_used",
                        "ros_used")},
                    report.metrics.get("collision_checked") is False
                    and report.metrics.get("executable") is False
                    and report.metrics.get("map_used") is False
                    and report.metrics.get("ros_used") is False))
            # Independent short-duration counterexample: T/4 must be rejected
            # even though the plan's own selection block was made coherent.
            short_plan = _g3_shortened_plan(request, plan)
            short_report = audit_dynamics(
                request, short_plan, DynamicAuditConfig())
            record["short_duration"] = short_report.as_dict()
            checks.append(_check(
                name + "_short_duration_rejected", "counterexample",
                "DYNAMIC_LIMIT",
                {"passed": short_report.passed,
                 "reason": short_report.reason,
                 "quantity": (short_report.first_failure or {}).get(
                     "quantity"),
                 "location": short_report.first_failure},
                not short_report.passed
                and short_report.reason == "DYNAMIC_LIMIT"))
            # Serialization identity: canonical roundtrip and explicit tamper.
            rebuilt = VirtualATimedPlan.from_model_json(
                payload, expected_request=request)
            sample_times = (0.0, plan.duration_s / 2.0, plan.duration_s)
            projection = plan.sample(sample_times)
            record["roundtrip"] = {
                "equal_json": rebuilt.to_model_json() == payload,
                "equal_digest": rebuilt.model_sha256() == plan.model_sha256(),
                "equal_state_midpoint":
                    rebuilt.state_at(sample_times[1]).as_dict() ==
                    plan.state_at(sample_times[1]).as_dict(),
            }
            record["sample"] = {
                "times": list(projection.times),
                "authoritative": projection.authoritative,
                "states_equal_pointwise": all(
                    state.as_dict() == plan.state_at(t).as_dict()
                    for t, state in zip(projection.times,
                                        projection.states)),
            }
            checks.append(_check(
                name + "_serialization_roundtrip", "serialization",
                {"equal_json": True, "equal_digest": True,
                 "equal_state_midpoint": True},
                record["roundtrip"],
                all(record["roundtrip"].values())))
            checks.append(_check(
                name + "_sample_non_authoritative", "serialization",
                {"authoritative": False, "states_equal_pointwise": True},
                record["sample"],
                record["sample"]["authoritative"] is False
                and record["sample"]["states_equal_pointwise"]))
            checks.append(_check(
                name + "_model_no_self_digest", "serialization",
                "no model_sha256/audit/executable fields",
                "absent",
                ('"model_sha256"' not in payload
                 and '"dynamic_audit_passed"' not in payload
                 and '"executable"' not in payload)))
            tampered_t = _g3_tamper_reason(
                payload, lambda doc: doc["timing"].__setitem__(
                    "T_s", doc["timing"]["T_s"] * 2.0))
            tampered_extra = _g3_tamper_reason(
                payload, lambda doc: doc.__setitem__("extra", 1))
            tampered_self = _g3_tamper_reason(
                payload, lambda doc: doc.__setitem__(
                    "model_sha256", "0" * 64))
            record["tamper"] = {
                "timing_T": tampered_t,
                "extra_field": tampered_extra,
                "self_digest": tampered_self,
            }
            checks.append(_check(
                name + "_tamper_rejected", "tamper",
                {"timing_T": "MODEL_TIMING_INCONSISTENT",
                 "extra_field": "UNSUPPORTED_MODEL_FIELD",
                 "self_digest": "UNSUPPORTED_MODEL_FIELD"},
                record["tamper"],
                tampered_t == "MODEL_TIMING_INCONSISTENT"
                and tampered_extra == "UNSUPPORTED_MODEL_FIELD"
                and tampered_self == "UNSUPPORTED_MODEL_FIELD"))
            result_overview[name] = {
                "solve_status": solve_result.status,
                "timing_status": timing.status,
                "audit_passed": report.passed if report is not None else None,
                "audit_reason": report.reason if report is not None else None,
                "T_s": timing.T_s,
                "S_m": timing.total_S_m,
                "worst": timing.bounds_summary.get("worst"),
                "selected_duration_s": timing.bounds_summary.get(
                    "selected_duration_s"),
                "required_duration_s": timing.bounds_summary.get(
                    "required_duration_s"),
                "rescalings": timing.rescalings,
                "model_sha256": record["model_sha256"],
                "segments": len(report.whole_segment_bounds)
                if report is not None else None,
                "leaves": report.metrics.get("leaves")
                if report is not None else None,
                "max_utilization": report.metrics.get("max_utilization")
                if report is not None else None,
            }
        elif timing is not None:
            checks.append(_check(
                name + "_retime", "timing", "DYNAMIC_FEASIBLE",
                timing.status, timing.status == "DYNAMIC_FEASIBLE"))
            checks.append(_check(
                name + "_audit", "dynamic_audit", "passed", None, False))
            result_overview[name] = {
                "solve_status": solve_result.status,
                "timing_status": timing.status,
                "audit_passed": False,
                "audit_reason": timing.reason,
            }
        else:
            checks.append(_check(
                name + "_solve", "production_solve", "GEOMETRY_FEASIBLE",
                solve_result.status, False))
            result_overview[name] = {
                "solve_status": solve_result.status,
                "timing_status": None,
                "audit_passed": False,
            }
        records[name] = record
    hold_record, hold_checks = _g3_hold_record(requests_doc, base)
    records["NO_MOTION_HOLD"] = hold_record
    checks.extend(hold_checks)
    result_overview["NO_MOTION_HOLD"] = {
        "solve_status": None,
        "timing_status": hold_record["timing"]["status"],
        "audit_passed": (hold_record["dynamic_audit"] or {}).get("passed"),
        "T_s": 0.0, "S_m": 0.0,
        "model_sha256": hold_record["model_sha256"],
    }
    status = "PASS" if all(check["pass"] for check in checks) else "FAIL"
    summary = {
        "schema": G3_SCHEMA,
        "stage": "G3",
        "status": status,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "checks": checks,
        "results": result_overview,
        "nonclaims": [
            "DYNAMIC_FEASIBLE only when the independent dynamic audit passes",
            "collision_checked=false; executable=false",
            "no map, ROS, hardware, real-time or time-optimal claim",
            "retimer selection is not a certificate",
            "DYNAMIC_FEASIBLE is not Codex ACCEPTED",
        ],
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    (output / "results.json").write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return summary


def _g4_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _g4_check(name, category, expected, actual, passed, **detail):
    return _check(name, category, expected, actual, passed, **detail)


def run_g4(output, scene_manifest):
    """Run the autonomous pipeline on the frozen G4-A scene inputs.

    Only ``inputs/<scene>/map.yaml`` and ``inputs/<scene>/request.json`` are
    read.  Reference profiles, known-good trajectories and old audit records
    are never opened by the production call.
    """
    output = _resolve_output(output)
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = Path(scene_manifest)
    if not manifest_path.is_absolute():
        manifest_path = (ROOT / manifest_path).resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != G4_SCENE_SCHEMA or \
            manifest.get("stage") != "G4-A":
        raise VirtualAContractError(
            "INVALID_SCENE_MANIFEST",
            "schema={!r} stage={!r}".format(
                manifest.get("schema"), manifest.get("stage")))
    if not manifest.get("frozen") or not manifest.get(
            "inputs_have_no_reference"):
        raise VirtualAContractError(
            "UNFROZEN_SCENE_MANIFEST",
            "frozen/input-reference flags are not true")
    base_dir = manifest_path.parent
    checks = []
    records = {}
    overview = {}
    for name in sorted(manifest["scenes"]):
        entry = manifest["scenes"][name]
        input_dir = base_dir / entry["input_dir"]
        record = {
            "scene_id": name,
            "input_dir": str(input_dir),
            "status": None,
            "reason": "",
            "detail": "",
            "executable": False,
            "first_failure_layer": "",
            "checks": 0,
            "wall_time_s": 0.0,
            "timings": {},
            "metrics": {},
            "grid_sha256": "",
            "request_sha256": "",
            "planning_context_sha256": "",
            "model_sha256": "",
            "model_json": None,
            "model_summary": None,
            "audits": {"geometry": None, "dynamics": None,
                       "collision": None},
            "attempts": [],
            "routes": [],
            "search": {},
            "input_files": [],
            "reference_free_bundle": False,
            "manifest_map_match": False,
            "manifest_request_match": False,
            "endpoint_error_m": None,
            "heading_error_rad": None,
            "rigidity_error_m": None,
            "collision_margin_m": None,
            "collision_envelope": None,
            "failure": None,
        }
        try:
            files = sorted(
                str(path.relative_to(input_dir))
                for path in input_dir.rglob("*") if path.is_file())
            record["input_files"] = files
            record["reference_free_bundle"] = bool(
                set(files) == {"map.pgm", "map.yaml", "request.json"} and
                not any("reference" in value.lower() for value in files))
            checks.append(_g4_check(
                name + "_reference_free_bundle", "input_bundle",
                "only map.pgm/map.yaml/request.json", files,
                record["reference_free_bundle"]))
            grid, map_metadata = load_map(input_dir / "map.yaml")
            payload = json.loads(
                (input_dir / "request.json").read_text(encoding="utf-8"))
            request = request_from_payload(payload["request"])
            planner = planner_config_from_dict(payload["planner_config"])
            timing = timing_config_from_dict(payload["timing_config"])
            collision = collision_config_from_dict(
                payload["collision_config"])
            map_request = VirtualAMapRequest(
                request=request, grid=grid, planner_config=planner,
                timing_config=timing, collision_config=collision)
            record["grid_sha256"] = grid_signature(grid)
            record["request_sha256"] = request_fingerprint(request)
            record["manifest_map_match"] = (
                record["grid_sha256"] == entry.get("map_sha256"))
            record["manifest_request_match"] = (
                record["request_sha256"] == entry.get("request_sha256"))
            checks.append(_g4_check(
                name + "_manifest_map", "scene_manifest",
                entry.get("map_sha256"), record["grid_sha256"],
                record["manifest_map_match"]))
            checks.append(_g4_check(
                name + "_manifest_request", "scene_manifest",
                entry.get("request_sha256"), record["request_sha256"],
                record["manifest_request_match"]))
            started = time.monotonic()
            result = plan_virtual_a(map_request)
            elapsed = time.monotonic() - started
            record.update({
                "status": result.status,
                "reason": result.reason,
                "detail": result.detail,
                "executable": bool(result.executable),
                "first_failure_layer": result.first_failure_layer,
                "checks": int(result.checks),
                "wall_time_s": float(result.wall_time_s),
                "outer_wall_time_s": float(elapsed),
                "timings": dict(result.timings),
                "metrics": dict(result.metrics),
                "grid_sha256": result.grid_sha256 or record["grid_sha256"],
                "request_sha256":
                    result.request_sha256 or record["request_sha256"],
                "planning_context_sha256":
                    result.planning_context_sha256,
                "model_sha256": result.model_sha256,
                "audits": {
                    "geometry": None if result.geometry_audit is None
                    else result.geometry_audit.as_dict(),
                    "dynamics": None if result.dynamic_audit is None
                    else result.dynamic_audit.as_dict(),
                    "collision": None if result.collision_audit is None
                    else result.collision_audit.as_dict(),
                },
                "attempts": [
                    attempt.as_dict() if hasattr(attempt, "as_dict")
                    else attempt for attempt in result.attempts],
                "routes": [
                    route.as_dict() if hasattr(route, "as_dict")
                    else route for route in result.routes],
                "search": dict(result.search),
            })
            if result.plan is not None:
                record["model_json"] = result.plan.to_model_json()
                record["model_summary"] = result.plan.model_summary()
            if result.path is not None:
                endpoint = result.path.endpoint()
                record["endpoint_error_m"] = math.hypot(
                    endpoint.x - request.goal_qf.x,
                    endpoint.y - request.goal_qf.y)
                record["heading_error_rad"] = abs(_g4_wrap(
                    endpoint.psi - request.goal_qf.psi))
            if result.geometry_audit is not None and isinstance(
                    result.geometry_audit.metrics, dict):
                sample = result.geometry_audit.metrics.get(
                    "sample_audit") or {}
                record["rigidity_error_m"] = sample.get(
                    "max_pairwise_rigidity_error_m")
            if result.collision_audit is not None and isinstance(
                    result.collision_audit.metrics, dict):
                margin = result.collision_audit.metrics.get("margin") or {}
                record["collision_margin_m"] = margin.get("margin_used_m")
                record["collision_envelope"] = dict(margin)
            checks.append(_g4_check(
                name + "_status", "pipeline", "SUCCESS", result.status,
                result.status == "SUCCESS"))
            checks.append(_g4_check(
                name + "_executable", "pipeline", True, result.executable,
                bool(result.executable)))
            checks.append(_g4_check(
                name + "_audits", "pipeline", "three passed",
                [bool(result.geometry_audit is not None and
                      result.geometry_audit.passed),
                 bool(result.dynamic_audit is not None and
                      result.dynamic_audit.passed),
                 bool(result.collision_audit is not None and
                      result.collision_audit.passed)],
                bool(result.executable)))
            checks.append(_g4_check(
                name + "_checks", "pipeline", "> 0", result.checks,
                int(result.checks) > 0))
            checks.append(_g4_check(
                name + "_endpoint", "geometry", "<= 0.005 m",
                record["endpoint_error_m"],
                record["endpoint_error_m"] is not None and
                record["endpoint_error_m"] <= 0.005))
            checks.append(_g4_check(
                name + "_heading", "geometry", "<= 0.2 deg",
                record["heading_error_rad"],
                record["heading_error_rad"] is not None and
                record["heading_error_rad"] <= math.radians(0.2)))
            checks.append(_g4_check(
                name + "_rigidity", "geometry", "<= 1e-6 m",
                record["rigidity_error_m"],
                record["rigidity_error_m"] is not None and
                record["rigidity_error_m"] <= 1e-6))
            checks.append(_g4_check(
                name + "_collision_margin", "collision", ">= 0.05 m",
                record["collision_margin_m"],
                record["collision_margin_m"] is not None and
                record["collision_margin_m"] >= 0.05))
            envelope = record["collision_envelope"] or {}
            baseline = envelope.get("baseline_margin_m")
            maximum = envelope.get("maximum_envelope_m")
            margin_used = envelope.get("margin_used_m")
            minimum_required = envelope.get("margin_minimum_required_m")
            per_side = envelope.get("per_side_margin_m")
            contexts = envelope.get("contexts") or {}
            context_covered = bool(contexts) and all(
                bool(value.get("covered")) for value in contexts.values())
            checks.append(_g4_check(
                name + "_collision_envelope_closure", "collision",
                "margin >= baseline + 2*max(envelope) and per-side "
                "growth covers max(envelope)",
                {
                    "baseline_margin_m": baseline,
                    "maximum_envelope_m": maximum,
                    "margin_used_m": margin_used,
                    "margin_minimum_required_m": minimum_required,
                    "margin_growth_lower_m":
                        envelope.get("margin_growth_lower_m"),
                    "per_side_margin_m": per_side,
                    "closure_gap_m": envelope.get("closure_gap_m"),
                    "pair_envelope_required_m":
                        envelope.get("pair_envelope_required_m"),
                    "pair_closure_gap_m":
                        envelope.get("pair_closure_gap_m"),
                },
                baseline is not None and maximum is not None and
                margin_used is not None and
                minimum_required is not None and per_side is not None and
                baseline >= 0.05 and margin_used >= baseline and
                minimum_required >= baseline + 2.0 * maximum and
                margin_used >= minimum_required and per_side >= maximum))
            checks.append(_g4_check(
                name + "_collision_envelope_contexts", "collision",
                "static/no-motion/sweep/map/pairs covered; 0.01 m sweep "
                "padding kept out of the numerical envelope",
                {
                    "contexts": contexts,
                    "sweep_motion_padding_m":
                        envelope.get("sweep_motion_padding_m"),
                    "radius_upper_m": envelope.get("radius_upper_m"),
                    "checker_radius_upper_m":
                        envelope.get("checker_radius_upper_m"),
                    "checker_half_length_m":
                        envelope.get("checker_half_length_m"),
                    "checker_half_width_m":
                        envelope.get("checker_half_width_m"),
                },
                context_covered and
                set(("static_start_goal", "no_motion_hold", "motion_sweep",
                     "map_boundary", "vehicle_vehicle")).issubset(contexts)
                and envelope.get("sweep_motion_padding_m") == 0.01
                and envelope.get("radius_upper_m") is not None and
                envelope.get("checker_radius_upper_m") is not None and
                envelope["radius_upper_m"] >=
                envelope["checker_radius_upper_m"]))
            if result.executable:
                checks.append(_g4_check(
                    name + "_context", "binding",
                    "plan/request/context/model equal",
                    {
                        "plan_context":
                            result.plan.planning_context_sha256(),
                        "result_context": result.planning_context_sha256,
                        "plan_request": result.plan.request_sha256,
                        "result_request": result.request_sha256,
                        "plan_model": result.plan.model_sha256(),
                        "result_model": result.model_sha256,
                    },
                    result.plan.planning_context_sha256() ==
                    result.planning_context_sha256 and
                    result.plan.request_sha256 == result.request_sha256 and
                    result.plan.model_sha256() == result.model_sha256))
            overview[name] = {
                "status": result.status,
                "executable": bool(result.executable),
                "checks": int(result.checks),
                "wall_time_s": float(result.wall_time_s),
                "path_length_m": record["metrics"].get("path_length_m"),
                "duration_s": record["metrics"].get("duration_s"),
                "model_sha256": result.model_sha256,
                "planning_context_sha256":
                    result.planning_context_sha256,
                "collision_checks": record["metrics"].get(
                    "collision_checks"),
                "collision_margin_m": record["collision_margin_m"],
                "collision_envelope": {
                    "maximum_envelope_m":
                        (record["collision_envelope"] or {}).get(
                            "maximum_envelope_m"),
                    "margin_used_m":
                        (record["collision_envelope"] or {}).get(
                            "margin_used_m"),
                    "per_side_margin_m":
                        (record["collision_envelope"] or {}).get(
                            "per_side_margin_m"),
                    "closure_gap_m":
                        (record["collision_envelope"] or {}).get(
                            "closure_gap_m"),
                },
                "timings": record["timings"],
                "attempt_count": len(record["attempts"]),
                "route_count": len(record["routes"]),
            }
        except Exception as exc:
            record["failure"] = {
                "type": type(exc).__name__, "error": str(exc),
                "traceback": traceback.format_exc()}
            checks.append(_g4_check(
                name + "_exception", "pipeline", "no exception",
                "{}: {}".format(type(exc).__name__, exc), False))
            overview[name] = {"status": "EXCEPTION",
                              "executable": False,
                              "error": str(exc)}
        records[name] = record
    status = "PASS" if checks and all(check["pass"] for check in checks) \
        else "FAIL"
    summary = {
        "schema": G4_SCHEMA,
        "stage": "G4",
        "status": status,
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
        "manifest_path": str(manifest_path),
        "manifest_sha256": manifest.get("manifest_sha256"),
        "scene_count": len(records),
        "checks": checks,
        "results": overview,
        "nonclaims": [
            "autonomous success only for the finite frozen scene set",
            "no general completeness, optimality, real-time or ROS claim",
            "SUCCESS is not Codex ACCEPTED",
            "reference files are absent from the benchmark inputs",
        ],
    }
    (output / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    (output / "results.json").write_text(
        json.dumps(records, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("G1", "G2", "G3", "G4"),
                        required=True)
    parser.add_argument("--output", required=True,
                        help="new evidence directory below "
                             "evaluation/virtual_formation_a/")
    parser.add_argument("--scene-manifest", default="",
                        help="frozen G4-A scene manifest (required for G4)")
    args = parser.parse_args(argv)
    try:
        if args.stage == "G4":
            if not args.scene_manifest:
                parser.error("--scene-manifest is required for --stage G4")
            summary = run_g4(args.output, args.scene_manifest)
        elif args.stage == "G3":
            summary = run_g3(args.output)
        elif args.stage == "G2":
            summary = run_g2(args.output)
        else:
            summary = run_g1(args.output)
    except Exception as exc:
        output = None
        try:
            output = _resolve_output(args.output)
        except Exception:
            output = None
        if output is not None and output.exists():
            failure = {
                "schema": (G4_SCHEMA if args.stage == "G4"
                           else G3_SCHEMA if args.stage == "G3"
                           else G2_SCHEMA if args.stage == "G2" else SCHEMA),
                "stage": args.stage, "status": "FAIL",
                "error_type": type(exc).__name__, "error": str(exc),
                "traceback": traceback.format_exc(),
            }
            path = output / "failure.json"
            if not path.exists():
                path.write_text(
                    json.dumps(failure, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
        print(json.dumps({
            "status": "FAIL", "error_type": type(exc).__name__,
            "error": str(exc)}, indent=2), file=sys.stderr)
        return 2
    summary_view = {
        "stage": summary["stage"],
        "status": summary["status"],
        "checks": len(summary["checks"]),
        "failed": [check["name"] for check in summary["checks"]
                   if not check["pass"]],
        "output": str(Path(args.output)),
        "codex_acceptance": "NOT_RUN_NOT_CLAIMED",
    }
    if args.stage == "G4":
        summary_view.update({
            "executable": sum(
                1 for record in summary["results"].values()
                if record.get("executable") is True),
            "scene_count": summary["scene_count"],
        })
    elif args.stage == "G1":
        summary_view.update({
            "position_error_bound_m": summary["integration"][0]["budget"][
                "position_error_bound_m"],
            "differential": summary["differential"],
        })
    elif args.stage == "G3":
        summary_view.update({
            "dynamic_feasible": sum(
                1 for record in summary["results"].values()
                if record.get("audit_passed") is True),
            "timing_passed": sum(
                1 for record in summary["results"].values()
                if record.get("timing_status") in
                ("DYNAMIC_FEASIBLE", "NO_MOTION")),
        })
    print(json.dumps(summary_view, indent=2, sort_keys=True))
    return 0 if summary["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())