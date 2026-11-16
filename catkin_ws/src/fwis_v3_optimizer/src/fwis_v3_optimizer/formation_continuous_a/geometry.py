"""A-only analytic formation geometry: C2 curvature profile and spatial jets.

The formation reference moves along a curvature scalar ``kappa(s)`` with
``eta=kappa'``, ``u=kappa''`` and ``g=kappa'''``.  ``s`` is the formation
arc length in metres.  Vehicle centres stay on the rigid offsets
``p_i = p_F + R(psi) r_i``; their body headings, curvatures and spatial
derivatives are recovered from that offset curve, never optimised separately.

Positions follow ``exp(i psi)`` with the exact polynomial integral of the
curvature for ``psi`` and a fixed composite-Simpson rule with an outward
remainder budget for ``x``/``y``.  ``psi`` is not clipped and no queried
position is interpolated between samples.
"""
from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import math

from ..continuous_geometry import ControlKnot, MotionBlock
from ..formation import FormationModel, FormationPose
from . import bounds
from .contracts import (
    INITIAL_CURVATURE_TOLERANCE,
    INITIAL_HEADING_TOLERANCE_RAD,
    RESEARCH_VEHICLE_IDS,
    CurvatureJet,
    FormationSpatialState,
    InitialAssumption,
    ValidationResult,
    VehicleSpatialJet,
    VirtualAContractError,
    require_research_formation,
    validate_formation_model,
)

C2_VALUE_TOL = 1e-9          # 1/m
C2_SLOPE_TOL = 1e-9          # 1/m^2
C2_SECOND_TOL = 1e-8         # 1/m^3
HERMITE_REUSE_TOL = 1e-9
MAX_PANEL_WIDTH_M = 0.02  # maximum equal subinterval width d = h/n
MAX_PANELS = bounds.MAX_PANELS
SIDES = ("auto", "left", "right")


def _finite(value, context):
    if isinstance(value, (bool, str, bytes)):
        raise VirtualAContractError(
            "NONFINITE_VALUE", "{} value={!r}".format(context, value))
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "NONFINITE_VALUE", "{} value={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise VirtualAContractError(
            "NONFINITE_VALUE", "{} value={!r}".format(context, value))
    return result


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _require_side(side):
    if side not in SIDES:
        raise VirtualAContractError(
            "INVALID_SIDE", "side={!r} allowed={}".format(side, list(SIDES)))


@dataclass(frozen=True)
class _Piece:
    """One normalized cubic piece of the formation curvature."""

    s0: float
    s1: float
    h: float
    c0: float
    c1: float
    c2: float
    c3: float

    def value(self, q):
        return self.c0 + q * (self.c1 + q * (self.c2 + q * self.c3))

    def slope(self, q):
        return (self.c1 + q * (2.0 * self.c2 + 3.0 * q * self.c3)) / self.h

    def second(self, q):
        return (2.0 * self.c2 + 6.0 * q * self.c3) / (self.h * self.h)

    def third(self):
        return 6.0 * self.c3 / (self.h * self.h * self.h)

    def psi_increment(self, q):
        return self.h * q * (
            self.c0 + q * (
                self.c1 / 2.0 + q * (
                    self.c2 / 3.0 + q * self.c3 / 4.0)))

    def coefficients(self):
        return (self.c0, self.c1, self.c2, self.c3)


def _hermite_coefficients(left, right):
    h = right.s - left.s
    k0, k1 = left.value, right.value
    eta0, eta1 = left.derivative, right.derivative
    c0 = k0
    c1 = h * eta0
    c2 = 3.0 * (k1 - k0) - h * (2.0 * eta0 + eta1)
    c3 = 2.0 * (k0 - k1) + h * (eta0 + eta1)
    return c0, c1, c2, c3


class C2CurvatureProfile:
    """Production C2 cubic-Hermite formation curvature profile."""

    kind = "hermite_c2"
    serialization_ready = True
    numerics_version = bounds.NUMERICS_VERSION

    def __init__(self, knots, pieces, length_m, psi_prefix):
        self._knots = tuple(knots)
        self._pieces = tuple(pieces)
        self._length_m = float(length_m)
        self._psi_prefix = tuple(psi_prefix)

    @classmethod
    def from_knots(cls, knots):
        """Build and verify a C2 cubic-Hermite profile from shared knots."""
        try:
            supplied = tuple(knots)
        except TypeError as exc:
            raise VirtualAContractError(
                "INVALID_KNOTS", "knots must be an iterable of ControlKnot") from exc
        if len(supplied) < 2:
            raise VirtualAContractError(
                "INVALID_KNOTS", "at least two knots are required")
        for index, knot in enumerate(supplied):
            if not isinstance(knot, ControlKnot):
                raise VirtualAContractError(
                    "INVALID_KNOT", "index={} expected ControlKnot".format(index),
                    index=index)
            for name in ("s", "value", "derivative"):
                if not math.isfinite(getattr(knot, name)):
                    raise VirtualAContractError(
                        "NONFINITE_KNOT", "index={} {}".format(index, name),
                        index=index, field=name)
        if supplied[0].s != 0.0:
            raise VirtualAContractError(
                "INVALID_PROFILE_DOMAIN",
                "first knot must start at s=0; got s={!r}".format(supplied[0].s),
                index=0, s=supplied[0].s)
        for index in range(len(supplied) - 1):
            left, right = supplied[index], supplied[index + 1]
            if right.s < left.s:
                raise VirtualAContractError(
                    "INVALID_KNOT_ORDER",
                    "index={} s={!r} is before previous s={!r}".format(
                        index + 1, right.s, left.s),
                    index=index + 1, s=right.s, previous_s=left.s)
            if right.s == left.s:
                raise VirtualAContractError(
                    "NON_POSITIVE_SEGMENT",
                    "index={} h=0 between s={!r} and s={!r}".format(
                        index, left.s, right.s),
                    index=index, h=0.0, s=left.s)

        pieces = []
        for index in range(len(supplied) - 1):
            left, right = supplied[index], supplied[index + 1]
            h = right.s - left.s
            if h <= 0.0 or not math.isfinite(h):
                raise VirtualAContractError(
                    "NON_POSITIVE_SEGMENT", "index={} h={!r}".format(index, h),
                    index=index, h=h)
            c0, c1, c2, c3 = _hermite_coefficients(left, right)
            pieces.append(_Piece(
                left.s, right.s, h, c0, c1, c2, c3))

        # C0/C1 are shared-knot properties; verify the reconstructed limits.
        for index in range(len(pieces) - 1):
            piece, following = pieces[index], pieces[index + 1]
            if abs(piece.value(1.0) - following.c0) > C2_VALUE_TOL:
                raise VirtualAContractError(
                    "NON_C2_PROFILE",
                    "C0 discontinuity node={} s={!r}".format(
                        index + 1, following.s0),
                    node=index + 1, s=following.s0)
            if abs(piece.slope(1.0) - following.slope(0.0)) > C2_SLOPE_TOL:
                raise VirtualAContractError(
                    "NON_C2_PROFILE",
                    "C1 discontinuity node={} s={!r}".format(
                        index + 1, following.s0),
                    node=index + 1, s=following.s0)
            u_left, u_right = piece.second(1.0), following.second(0.0)
            if abs(u_left - u_right) > C2_SECOND_TOL:
                raise VirtualAContractError(
                    "NON_C2_PROFILE",
                    "C2 discontinuity node={} s={!r} u_left={!r} "
                    "u_right={!r} tolerance={!r}".format(
                        index + 1, following.s0, u_left, u_right,
                        C2_SECOND_TOL),
                    node=index + 1, s=following.s0,
                    u_left=u_left, u_right=u_right)

        profile = cls(supplied, pieces, supplied[-1].s,
                      cls._exact_psi_prefix(pieces))
        profile._verify_against_motion_block()
        return profile

    @staticmethod
    def _exact_psi_prefix(pieces):
        prefix = [0.0]
        for piece in pieces:
            prefix.append(prefix[-1] + piece.psi_increment(1.0))
        return prefix

    def _verify_against_motion_block(self):
        """Cross-check the independently generated coefficients on the reuse path."""
        block = MotionBlock(
            mode=0, direction=1, length=self._length_m,
            knots=self._knots, slope_policy="fixed")
        for piece in self._pieces:
            for q in (0.0, 0.37, 0.5, 1.0):
                s = piece.s0 + q * piece.h
                value, first, second = block.control(s)
                if (abs(value - piece.value(q)) > HERMITE_REUSE_TOL
                        or abs(first - piece.slope(q)) > HERMITE_REUSE_TOL
                        or abs(second - piece.second(q)) > HERMITE_REUSE_TOL):
                    raise VirtualAContractError(
                        "HERMITE_MISMATCH",
                        "MotionBlock disagrees with the independent C2 "
                        "coefficients at s={!r}".format(s), s=s)

    @property
    def knots(self):
        return self._knots

    @property
    def length_m(self):
        return self._length_m

    @property
    def segment_count(self):
        return len(self._pieces)

    @property
    def domain_segments(self):
        return tuple((piece.s0, piece.s1) for piece in self._pieces)

    def _check_station(self, s):
        s = _finite(s, "profile station")
        if s < 0.0 or s > self._length_m:
            raise VirtualAContractError(
                "S_DOMAIN",
                "s={!r} is outside [0, {!r}]; no clipping".format(
                    s, self._length_m), s=s, length_m=self._length_m)
        return s

    def _locate(self, s, side="auto"):
        _require_side(side)
        s = self._check_station(s)
        last = len(self._pieces) - 1
        if s == 0.0:
            if side == "left":
                raise VirtualAContractError(
                    "SIDE_DOMAIN", "no left limit exists at s=0", s=s)
            return 0, 0.0
        if s == self._length_m:
            if side == "right":
                raise VirtualAContractError(
                    "SIDE_DOMAIN",
                    "no right limit exists at the terminal station", s=s)
            return last, 1.0
        for index, piece in enumerate(self._pieces):
            if s == piece.s0:
                if side == "left":
                    return index - 1, 1.0
                return index, 0.0
            if s < piece.s1:
                return index, (s - piece.s0) / piece.h
        raise VirtualAContractError(
            "S_DOMAIN", "s={!r} not resolved".format(s), s=s)

    def jet_at(self, s, side="auto"):
        index, q = self._locate(s, side)
        piece = self._pieces[index]
        return CurvatureJet(piece.value(q), piece.slope(q),
                            piece.second(q), piece.third())

    def psi_at(self, s):
        s = self._check_station(s)
        if s == 0.0:
            return 0.0
        for index, piece in enumerate(self._pieces):
            if s <= piece.s1:
                local = min(s - piece.s0, piece.h)
                return self._psi_prefix[index] + piece.psi_increment(
                    local / piece.h)
        return self._psi_prefix[-1]

    def interval_bounds(self, s0, s1):
        """Interval k/eta/u/g bounds with piece coefficients and side limits."""
        s0 = _finite(s0, "interval start")
        s1 = _finite(s1, "interval end")
        if s0 < 0.0 or s1 > self._length_m or s1 < s0:
            raise VirtualAContractError(
                "S_DOMAIN",
                "interval [{!r}, {!r}] is outside [0, {!r}]".format(
                    s0, s1, self._length_m), s0=s0, s1=s1)
        records = []
        overall = {name: (math.inf, -math.inf) for name in ("k", "eta", "u", "g")}
        for index, piece in enumerate(self._pieces):
            a, b = max(s0, piece.s0), min(s1, piece.s1)
            if a > b:
                continue
            q0 = (a - piece.s0) / piece.h
            q1 = (b - piece.s0) / piece.h
            piece_bounds = bounds.segment_jet_bounds(
                piece.c0, piece.c1, piece.c2, piece.c3, piece.h, q0, q1)
            records.append({
                "index": index,
                "s0": a, "s1": b, "h": piece.h,
                "coefficients": list(piece.coefficients()),
                "k": list(piece_bounds["k"]),
                "eta": list(piece_bounds["eta"]),
                "u": list(piece_bounds["u"]),
                "g": list(piece_bounds["g"]),
                "bound_source": "widened_hermite_coefficients",
            })
            for name in overall:
                lower, upper = piece_bounds[name]
                overall[name] = (
                    min(overall[name][0], lower), max(overall[name][1], upper))
        if not records:
            index, q = self._locate(s0, "auto")
            piece = self._pieces[index]
            piece_bounds = bounds.segment_jet_bounds(
                piece.c0, piece.c1, piece.c2, piece.c3, piece.h, q, q)
            records.append({
                "index": index, "s0": s0, "s1": s1, "h": piece.h,
                "coefficients": list(piece.coefficients()),
                "k": list(piece_bounds["k"]),
                "eta": list(piece_bounds["eta"]),
                "u": list(piece_bounds["u"]),
                "g": list(piece_bounds["g"]),
                "bound_source": "widened_hermite_coefficients",
            })
            overall = {name: tuple(piece_bounds[name]) for name in overall}
        left = self.jet_at(s0, "left") if s0 > 0.0 else self.jet_at(0.0, "right")
        right = (self.jet_at(s1, "right") if s1 < self._length_m
                 else self.jet_at(self._length_m, "left"))
        return {
            "s0": s0, "s1": s1,
            "segments": records,
            "k": list(overall["k"]), "eta": list(overall["eta"]),
            "u": list(overall["u"]), "g": list(overall["g"]),
            "left_limit": left.as_dict(),
            "right_limit": right.as_dict(),
            "left_limit_source": "left" if s0 > 0.0 else "right-boundary",
            "right_limit_source": "right" if s1 < self._length_m else "terminal",
            "numerics_version": self.numerics_version,
        }

    def as_dict(self):
        return {
            "kind": self.kind,
            "serialization_ready": self.serialization_ready,
            "numerics_version": self.numerics_version,
            "length_m": self._length_m,
            "knots": [
                {"s": knot.s, "value": knot.value,
                 "derivative": knot.derivative}
                for knot in self._knots],
            "pieces": [
                {"s0": piece.s0, "s1": piece.s1, "h": piece.h,
                 "coefficients": list(piece.coefficients())}
                for piece in self._pieces],
        }


def _even_at_least(value):
    count = int(math.ceil(value))
    if count < 2:
        return 2
    return count if count % 2 == 0 else count + 1


class VirtualAPath:
    """Formation reference path: fixed position integral plus curvature jets.

    ``hold`` is the explicit zero-motion path: ``profile=None``, ``length_m=0``
    and only ``s=0`` is accepted.
    """

    def __init__(self, start_qf, profile, model):
        if not isinstance(start_qf, FormationPose):
            raise VirtualAContractError(
                "INVALID_FORMATION_POSE", "start_qf must be a FormationPose")
        if not isinstance(model, FormationModel):
            raise VirtualAContractError(
                "INVALID_FORMATION_MODEL", "model must be a FormationModel")
        require_research_formation(model)
        for attribute in ("length_m", "jet_at", "psi_at", "interval_bounds",
                          "domain_segments"):
            if not hasattr(profile, attribute):
                raise VirtualAContractError(
                    "INVALID_PROFILE",
                    "profile lacks {}".format(attribute))
        length = _finite(profile.length_m, "profile.length_m")
        if length <= 0.0:
            raise VirtualAContractError(
                "INVALID_PROFILE", "a motion path needs length_m > 0")
        self._start_qf = start_qf
        self._profile = profile
        self._model = model
        self._kernel = "motion"
        self._length_m = length
        self._plan_integration()

    @classmethod
    def hold(cls, start_qf, model):
        if not isinstance(start_qf, FormationPose):
            raise VirtualAContractError(
                "INVALID_FORMATION_POSE", "start_qf must be a FormationPose")
        if not isinstance(model, FormationModel):
            raise VirtualAContractError(
                "INVALID_FORMATION_MODEL", "model must be a FormationModel")
        require_research_formation(model)
        instance = cls.__new__(cls)
        instance._start_qf = start_qf
        instance._profile = None
        instance._model = model
        instance._kernel = "hold"
        instance._length_m = 0.0
        instance._grids = ()
        instance._prefix_x = ()
        instance._prefix_y = ()
        instance._position_error_bound_m = 0.0
        instance._angle_error_bound_rad = 0.0
        instance._budget = {
            "kind": "hold",
            "position_error_bound_m": 0.0,
            "angle_error_bound_rad": 0.0,
            "total_panels": 0,
        }
        return instance

    @property
    def start_qf(self):
        return self._start_qf

    @property
    def profile(self):
        return self._profile

    @property
    def model(self):
        return self._model

    @property
    def length_m(self):
        return self._length_m

    @property
    def kind(self):
        return self._kernel

    @property
    def is_hold(self):
        return self._kernel == "hold"

    @property
    def position_error_bound_m(self):
        return self._position_error_bound_m

    @property
    def angle_error_bound_rad(self):
        return self._angle_error_bound_rad

    @property
    def integration_budget(self):
        return dict(self._budget)

    def _plan_integration(self):
        records = []
        max_kappa = 0.0
        for s0, s1 in self._profile.domain_segments:
            piece_bounds = self._profile.interval_bounds(s0, s1)
            max_kappa = max(max_kappa, piece_bounds["k"][1])
            m4 = bounds.m4_upper(
                max(0.0, piece_bounds["k"][1]),
                max(0.0, piece_bounds["eta"][1]),
                max(0.0, piece_bounds["u"][1]),
                max(0.0, piece_bounds["g"][1]))
            h = s1 - s0
            records.append({
                "s0": s0, "s1": s1, "h": h, "m4": m4,
                "panels": _even_at_least(h / MAX_PANEL_WIDTH_M),
            })
        initial_panels = [record["panels"] for record in records]
        refinement_steps = 0
        angle_bound = bounds.angle_budget(
            self._profile.segment_count,
            abs(self._start_qf.psi) + self._length_m * max_kappa)
        if angle_bound > bounds.ANGLE_BUDGET_RAD:
            raise VirtualAContractError(
                "BOUND_UNCONFIRMED",
                "angle bound {!r} exceeds budget".format(angle_bound),
                angle_bound_rad=angle_bound)
        for _ in range(64):
            budget = bounds.position_budget(
                [(record["h"], record["panels"], record["m4"])
                 for record in records])
            # The guard counts Simpson accumulation panels: n/2 complete
            # panels per segment plus one local-query panel per segment.
            guard = bounds.position_guard(
                self._length_m,
                budget["full_panels"] + budget["local_panels"],
                angle_bound)
            total = bounds._next_up(budget["euclidean_m"] + guard)
            if total <= bounds.POSITION_BUDGET_M:
                break
            total_panels = budget["total_panels"]
            if total_panels * 2 > MAX_PANELS:
                raise VirtualAContractError(
                    "BOUND_UNCONFIRMED",
                    "Simpson refinement would exceed {} subintervals".format(
                        MAX_PANELS),
                    total_panels=total_panels)
            contributions = [
                bounds.simpson_segment_budget(
                    record["h"], record["panels"],
                    record["m4"])["segment_remainder_m"]
                for record in records]
            worst = contributions.index(max(contributions))
            records[worst]["panels"] *= 2
            refinement_steps += 1
        else:
            raise VirtualAContractError(
                "BOUND_UNCONFIRMED", "Simpson refinement did not converge")

        grids = []
        prefix_x = [self._start_qf.x]
        prefix_y = [self._start_qf.y]
        for record in records:
            h, panels = record["h"], record["panels"]
            width = h / panels
            psi = [
                self._profile.psi_at(
                    record["s1"] if index == panels
                    else record["s0"] + index * width)
                for index in range(panels + 1)]
            fx = [math.cos(value + self._start_qf.psi) for value in psi]
            fy = [math.sin(value + self._start_qf.psi) for value in psi]
            cumulative_x = [0.0] * (panels + 1)
            cumulative_y = [0.0] * (panels + 1)
            for index in range(1, panels, 2):
                factor = width / 3.0
                cumulative_x[index + 1] = cumulative_x[index - 1] + factor * (
                    fx[index - 1] + 4.0 * fx[index] + fx[index + 1])
                cumulative_y[index + 1] = cumulative_y[index - 1] + factor * (
                    fy[index - 1] + 4.0 * fy[index] + fy[index + 1])
            grids.append({
                "s0": record["s0"], "s1": record["s1"],
                "h": h, "panels": panels, "width": width,
                "cumulative_x": cumulative_x, "cumulative_y": cumulative_y,
            })
            prefix_x.append(prefix_x[-1] + cumulative_x[panels])
            prefix_y.append(prefix_y[-1] + cumulative_y[panels])
        self._grids = tuple(grids)
        self._prefix_x = tuple(prefix_x)
        self._prefix_y = tuple(prefix_y)
        self._position_error_bound_m = total
        self._angle_error_bound_rad = angle_bound
        segment_budgets = budget["segment_budgets"]
        self._budget = {
            "kind": "motion",
            "position_error_bound_m": total,
            "angle_error_bound_rad": angle_bound,
            # Number of equal Simpson subintervals d = h/n per segment.
            "total_panels": budget["total_panels"],
            # Actual Simpson accumulation panels: n/2 per segment plus one
            # local-query panel per segment in the conservative budget.
            "total_full_simpson_panels": budget["full_panels"],
            "local_simpson_panels_per_segment": 1,
            "total_local_simpson_panels": budget["local_panels"],
            "simpson_euclidean_m": budget["euclidean_m"],
            "simpson_per_coordinate_m": budget["per_coordinate_m"],
            "simpson_panel_widths_m": [
                record["full_panel_width_m"] for record in segment_budgets],
            "simpson_local_panel_widths_m": [
                record["local_panel_width_m"] for record in segment_budgets],
            "simpson_full_remainders_m": [
                record["full_remainder_m"] for record in segment_budgets],
            "simpson_local_remainders_m": [
                record["local_remainder_m"] for record in segment_budgets],
            "simpson_segment_remainders_m": [
                record["segment_remainder_m"] for record in segment_budgets],
            "floating_point_guard_m": guard,
            "panels_per_segment": [record["panels"] for record in records],
            "initial_panels_per_segment": initial_panels,
            "refinement_steps": refinement_steps,
            "m4_per_segment": [record["m4"] for record in records],
            # Full-panel widths are 2*d; subinterval widths are d.  Keep both
            # explicit so the v2 budget cannot be mistaken for a d-wide panel.
            "panel_widths_m": [
                record["full_panel_width_m"] for record in segment_budgets],
            "subinterval_widths_m": [
                record["h"] / record["panels"] for record in records],
            "numerics_version": bounds.NUMERICS_VERSION,
        }

    def _grid_index(self, s):
        for index, grid in enumerate(self._grids):
            if s <= grid["s1"]:
                return index
        return len(self._grids) - 1

    def _integrate_xy(self, s):
        if s <= 0.0:
            return self._start_qf.x, self._start_qf.y
        index = self._grid_index(s)
        grid = self._grids[index]
        x = self._prefix_x[index]
        y = self._prefix_y[index]
        local = min(s - grid["s0"], grid["h"])
        if local <= 0.0:
            return x, y
        width = grid["width"]
        panels = grid["panels"]
        whole = int(math.floor(local / width))
        if whole >= panels or local >= grid["h"] - 32.0 * bounds.EPSILON * grid["h"]:
            whole = panels
        if whole % 2:
            whole -= 1
        x += grid["cumulative_x"][whole]
        y += grid["cumulative_y"][whole]
        start = grid["s0"] + whole * width
        remainder = s - start
        if remainder > 32.0 * bounds.EPSILON * grid["h"]:
            middle = 0.5 * (start + s)
            factor = remainder / 6.0
            for station, weight in ((start, 1.0), (middle, 4.0), (s, 1.0)):
                psi = self._start_qf.psi + self._profile.psi_at(station)
                x += weight * factor * math.cos(psi)
                y += weight * factor * math.sin(psi)
        return x, y

    def state_at_s(self, s, side="auto"):
        """State at formation arc length ``s``; domain errors are never clipped."""
        if self._kernel == "hold":
            station = _finite(s, "path station")
            if station != 0.0:
                raise VirtualAContractError(
                    "S_DOMAIN",
                    "the hold path accepts only s=0; got {!r}".format(station),
                    s=station)
            jet = CurvatureJet(0.0, 0.0, 0.0, 0.0)
            return map_spatial_state(
                self._start_qf, jet, self._model, s=0.0,
                position_error_bound_m=0.0, angle_error_bound_rad=0.0)
        station = _finite(s, "path station")
        jet = self._profile.jet_at(station, side)
        x, y = self._integrate_xy(station)
        qf = FormationPose(
            x, y, self._start_qf.psi + self._profile.psi_at(station))
        return map_spatial_state(
            qf, jet, self._model, s=station,
            position_error_bound_m=self._position_error_bound_m,
            angle_error_bound_rad=self._angle_error_bound_rad)

    def endpoint(self):
        if self._kernel == "hold":
            return self._start_qf
        x, y = self._integrate_xy(self._length_m)
        return FormationPose(
            x, y, self._start_qf.psi + self._profile.psi_at(self._length_m))


def map_spatial_state(qf, jet, model, *, s=0.0, position_error_bound_m=0.0,
                      angle_error_bound_rad=0.0):
    """Recover all three vehicle centres, headings, curvatures and jets."""
    if not isinstance(qf, FormationPose):
        raise VirtualAContractError(
            "INVALID_FORMATION_POSE", "qf must be a FormationPose")
    if not isinstance(jet, CurvatureJet):
        raise VirtualAContractError(
            "INVALID_CURVATURE_JET", "jet must be a CurvatureJet")
    if not isinstance(model, FormationModel):
        raise VirtualAContractError(
            "INVALID_FORMATION_MODEL", "model must be a FormationModel")
    require_research_formation(model)
    station = _finite(s, "spatial state station")
    kappa, eta, u, g = jet.kappa, jet.eta, jet.u, jet.g
    cosine, sine = math.cos(qf.psi), math.sin(qf.psi)

    def rotate(local_x, local_y):
        return (cosine * local_x - sine * local_y,
                sine * local_x + cosine * local_y)

    vehicles = []
    for offset in model.offsets:
        a, b = offset.x, offset.y
        radius_factor = a * a + b * b
        forward = 1.0 - b * kappa
        lateral = a * kappa
        denominator = forward * forward + lateral * lateral
        if not (forward > 0.0 and denominator > 0.0):
            raise VirtualAContractError(
                "NORMAL_BRANCH_VIOLATION",
                "vehicle_id={} s={!r} A={!r} D={!r} requires A>0 and D>0".format(
                    offset.vehicle_id, station, forward, denominator),
                vehicle_id=offset.vehicle_id, s=station,
                A=forward, D=denominator)
        root = math.sqrt(denominator)
        theta = qf.psi + math.atan2(lateral, forward)
        heading_slope = kappa + a * eta / denominator
        curvature = heading_slope / root
        denominator_s = 2.0 * (radius_factor * kappa - b) * eta
        denominator_ss = 2.0 * (
            radius_factor * eta * eta + (radius_factor * kappa - b) * u)
        root_s = denominator_s / (2.0 * root)
        root_ss = denominator_ss / (2.0 * root) - (
            denominator_s * denominator_s) / (4.0 * root ** 3)
        heading_slope_s = eta + a * (
            u / denominator - eta * denominator_s / (denominator ** 2))
        heading_slope_ss = u + a * (
            g / denominator
            - 2.0 * u * denominator_s / (denominator ** 2)
            - eta * denominator_ss / (denominator ** 2)
            + 2.0 * eta * denominator_s ** 2 / (denominator ** 3))
        curvature_s = (heading_slope_s / root
                       - heading_slope * root_s / (root ** 2))
        curvature_ss = (
            heading_slope_ss / root
            - 2.0 * heading_slope_s * root_s / (root ** 2)
            - heading_slope * root_ss / (root ** 2)
            + 2.0 * heading_slope * root_s ** 2 / (root ** 3))
        z1 = (forward, lateral)
        z2 = (-a * kappa * kappa - b * eta,
              kappa - b * kappa * kappa + a * eta)
        z3 = (-kappa * kappa + b * kappa ** 3 - 3.0 * a * kappa * eta
              - b * u,
              eta - 3.0 * b * kappa * eta + a * u - a * kappa ** 3)
        p_s = rotate(*z1)
        p_ss = rotate(*z2)
        p_sss = rotate(*z3)
        center_x = qf.x + cosine * a - sine * b
        center_y = qf.y + sine * a + cosine * b
        vehicles.append(VehicleSpatialJet(
            vehicle_id=offset.vehicle_id,
            x=center_x, y=center_y, theta=theta,
            curvature=curvature,
            p_s=p_s, p_ss=p_ss, p_sss=p_sss,
            theta_s=heading_slope, theta_ss=heading_slope_s,
            theta_sss=heading_slope_ss,
            curvature_s=curvature_s, curvature_ss=curvature_ss))
    return FormationSpatialState(
        s=station, qf=qf, curvature=jet, vehicles=tuple(vehicles),
        position_error_bound_m=position_error_bound_m,
        angle_error_bound_rad=angle_error_bound_rad)


def validate_initial_state(start_qf, start_headings, initial_assumption,
                           kappa0=0.0, eta0=0.0, model=None):
    """Check the declared stationary straight-A initial state.

    Vehicle headings are read and compared individually; they are never
    rewritten to ``psi`` before validation.  ``model`` is optional so the
    exact vehicle-ID set can be checked when the caller has one.
    """
    if not isinstance(start_qf, FormationPose):
        return ValidationResult.fail(
            "INVALID_INPUT", "start_qf must be a FormationPose")
    if not isinstance(initial_assumption, InitialAssumption):
        return ValidationResult.fail(
            "INVALID_INPUT", "initial_assumption must be an InitialAssumption")
    if model is not None:
        formation = validate_formation_model(model)
        if not formation.passed:
            return formation
    if not initial_assumption.stationary:
        return ValidationResult.fail(
            "UNSUPPORTED_INITIAL_STATE", "stationary=False is not supported")
    if initial_assumption.mode != "A":
        return ValidationResult.fail(
            "UNSUPPORTED_INITIAL_STATE",
            "mode={!r}; only unified A is supported".format(
                initial_assumption.mode), mode=initial_assumption.mode)
    if not initial_assumption.wheels_straight:
        return ValidationResult.fail(
            "UNSUPPORTED_INITIAL_STATE",
            "wheels_straight=False is not supported")
    if not isinstance(start_headings, MappingABC):
        return ValidationResult.fail(
            "INVALID_INPUT", "start_headings must be a mapping")
    headings = {}
    for vehicle_id, value in start_headings.items():
        if isinstance(vehicle_id, bool) or not isinstance(vehicle_id, int):
            return ValidationResult.fail(
                "INVALID_VEHICLE_ID_SET",
                "heading key={!r} is not an integer vehicle_id".format(
                    vehicle_id))
        try:
            headings[vehicle_id] = _finite(
                value, "start_headings vehicle_id={}".format(vehicle_id))
        except VirtualAContractError as exc:
            return ValidationResult.fail(
                exc.reason, exc.detail, vehicle_id=vehicle_id)
    if not headings:
        return ValidationResult.fail(
            "INVALID_VEHICLE_ID_SET", "start_headings is empty")
    if model is not None:
        expected = set(model.vehicle_ids)
        if set(headings) != expected:
            return ValidationResult.fail(
                "INVALID_VEHICLE_ID_SET",
                "missing={} extra={}".format(
                    sorted(expected - set(headings), ),
                    sorted(set(headings) - expected)))
    maximum, worst = 0.0, None
    for vehicle_id in sorted(headings):
        error = abs(_wrap(headings[vehicle_id] - start_qf.psi))
        if error > maximum:
            maximum, worst = error, vehicle_id
    if maximum > INITIAL_HEADING_TOLERANCE_RAD:
        return ValidationResult.fail(
            "INITIAL_HEADING_MISMATCH",
            "vehicle_id={} heading error {!r} rad exceeds {!r}".format(
                worst, maximum, INITIAL_HEADING_TOLERANCE_RAD),
            vehicle_id=worst, heading_error_rad=maximum,
            tolerance_rad=INITIAL_HEADING_TOLERANCE_RAD)
    for name, value in (("kappa0", kappa0), ("eta0", eta0)):
        try:
            checked = _finite(value, name)
        except VirtualAContractError as exc:
            return ValidationResult.fail(exc.reason, exc.detail, quantity=name)
        if abs(checked) > INITIAL_CURVATURE_TOLERANCE:
            return ValidationResult.fail(
                "UNSUPPORTED_INITIAL_STATE",
                "{}={!r} must be zero for the stationary straight-A state".format(
                    name, checked), quantity=name, value=checked)
    return ValidationResult.ok(
        vehicle_count=len(headings),
        max_heading_error_rad=maximum,
        kappa0=0.0,
        eta0=0.0,
        source=initial_assumption.source,
        node_context_verified=False,
        initial_heading_tolerance_rad=INITIAL_HEADING_TOLERANCE_RAD)


def check_vehicle_curvatures(state, limits):
    """Compare every vehicle curvature with ``1/r_min_ackermann``.

    The front vehicle's curvature includes its ``eta`` term; checking the
    reference curvature alone is not sufficient.
    """
    if not isinstance(state, FormationSpatialState):
        return ValidationResult.fail(
            "INVALID_INPUT", "state must be a FormationSpatialState")
    try:
        r_min = float(limits.r_min_ackermann)
    except (AttributeError, TypeError, ValueError):
        return ValidationResult.fail(
            "INVALID_LIMITS", "limits must expose r_min_ackermann")
    if not math.isfinite(r_min) or r_min <= 0.0:
        return ValidationResult.fail(
            "INVALID_LIMITS", "r_min_ackermann must be positive and finite")
    maximum = 1.0 / r_min
    max_abs = 0.0
    expectation = {}
    for vehicle in state.vehicles:
        magnitude = abs(vehicle.curvature)
        expectation[vehicle.vehicle_id] = magnitude
        max_abs = max(max_abs, magnitude)
        if magnitude > maximum:
            return ValidationResult.fail(
                "CURVATURE_LIMIT",
                "CURVATURE_LIMIT,vehicle_id={} s={!r} curvature={!r} "
                "limit={!r}".format(
                    vehicle.vehicle_id, state.s, vehicle.curvature, maximum),
                vehicle_id=vehicle.vehicle_id, s=state.s,
                curvature=vehicle.curvature, limit=maximum,
                metrics={"max_abs_curvature": max_abs,
                         "limit": maximum,
                         "abs_curvature_by_vehicle": expectation})
    return ValidationResult.ok(
        max_abs_curvature=max_abs, limit=maximum,
        abs_curvature_by_vehicle=expectation)