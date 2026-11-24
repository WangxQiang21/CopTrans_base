"""TASK03 finite formation-corridor templates and scalar row plans.

This module owns only the *finite* pieces of the TASK03 hard-corridor
contract:

* canonical whole-OBB and two-diagonal convex decompositions of one vehicle
  rectangle, copied from the accepted research vehicle dimensions;
* the immutable :class:`FormationCorridorTemplate` identity (defined in
  :mod:`contracts`) bound to request/grid/context/chain/guide/allocation;
* the deterministic scalar row order shared by the numeric residual, the
  CasADi ``g`` vector and the SciPy fallback.

It does not certificate a path.  A row plan proves nothing outside the finite
sample stations; the independent continuous certificate is a separate
TASK03 Step 4 module/function.  Every template is a conservative sufficient
condition and must report ``TEMPLATE_INSUFFICIENT`` when it cannot express the
requested cover rather than claim physical infeasibility.
"""

from __future__ import annotations

from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
import hashlib
import json
import math
import numbers

from ..formation import FormationPose
from .contracts import (
    FORMATION_CORRIDOR_CERTIFICATE_VERSION,
    FORMATION_CORRIDOR_COMPILER_VERSION,
    FORMATION_CORRIDOR_TEMPLATE_SCHEMA,
    FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1,
    FORMATION_TOLERANCE_M,
    ConvexCorridor,
    CorridorChain,
    CurvatureJet,
    FormationCorridorPiece,
    FormationCorridorTemplate,
    FormationCorridorTemplateSegment,
    FormationCorridorVehicleCover,
    FormationCorridorVehicleFrame,
    GuideRoute,
    VirtualAContractError,
    request_fingerprint,
)
from .corridor import (
    ASSESSMENT_SCHEMA,
    CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY,
    MAP_CLEAR_AT_SAMPLES,
    corridor_effective_margin,
)
from .geometry import map_spatial_state


def _canonical_sha256(value):
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _formation_sha256_hex(value):
    return isinstance(value, str) and len(value) == 64 and all(
        character in "0123456789abcdef" for character in value)


def _region_index(value, context):
    """Reject booleans, floats and other non-integral region selectors."""
    if isinstance(value, bool) or not isinstance(value, numbers.Integral):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "{} must be a nonnegative integer, got {!r}".format(
                context, value))
    result = int(value)
    if result < 0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "{} must be a nonnegative integer, got {!r}".format(
                context, value))
    return result


def _finite_row(value, context):
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "{} value={!r}".format(context, value)) from exc
    if not math.isfinite(result):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "{} value={!r} is not finite".format(context, value))
    return result


def vehicle_frames(request):
    """Return the request's exact vehicle offsets and shapes in model order."""
    frames = []
    for offset in request.model.offsets:
        shape = request.vehicle_shapes.get(offset.vehicle_id)
        if shape is None:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "request has no shape for vehicle_id={!r}".format(
                    offset.vehicle_id))
        frames.append(FormationCorridorVehicleFrame(
            vehicle_id=int(offset.vehicle_id),
            offset_x=_finite_row(offset.x, "offset.x"),
            offset_y=_finite_row(offset.y, "offset.y"),
            length_m=_finite_row(shape.length_m, "shape.length_m"),
            width_m=_finite_row(shape.width_m, "shape.width_m")))
    if not frames:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "request model has no vehicle offsets")
    return tuple(frames)


def formation_model_sha256(request):
    """Canonical model identity used by a formation-corridor template."""
    frames = vehicle_frames(request)
    return _canonical_sha256({
        "model": [
            {
                "vehicle_id": int(frame.vehicle_id),
                "offset_x": float(frame.offset_x),
                "offset_y": float(frame.offset_y),
                "length_m": float(frame.length_m),
                "width_m": float(frame.width_m),
            }
            for frame in frames
        ],
    })


def formation_allocation_digest(allocation):
    """Stable identity of one normalized corridor allocation."""
    try:
        entries = [
            [float(left), float(right), int(index)]
            for left, right, index in allocation]
    except (TypeError, ValueError) as exc:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "allocation entries must be numeric triples") from exc
    return _canonical_sha256({"allocation": entries})


def formation_guide_digest(guide):
    """Stable identity of the exact guide points consumed by the solver."""
    if not isinstance(guide, GuideRoute):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "guide must be a GuideRoute")
    return _canonical_sha256({
        "source": str(guide.source),
        "grid_sha256": str(guide.grid_sha256),
        "points": [[float(x), float(y)] for x, y in guide.points],
    })


def whole_cover(frame, region_index):
    """One whole rectangle assigned to exactly one convex region."""
    return FormationCorridorVehicleCover(
        vehicle_id=frame.vehicle_id,
        decomposition="whole",
        pieces=(FormationCorridorPiece(
            region_index=_region_index(region_index, "whole region_index"),
            local_vertices=frame.local_corners),))


def diagonal_cover(frame, diagonal, first_region, second_region):
    """Two closed triangles covering the rectangle along one diagonal.

    ``diagonal=0`` splits corners ``(c0, c2)``; ``diagonal=1`` splits
    ``(c1, c3)``.  The first triangle is assigned ``first_region`` and the
    second ``second_region``.  Both regions must be the same or adjacent in
    the chain (checked by the template contract).
    """
    diagonal = _region_index(diagonal, "diagonal")
    if diagonal not in (0, 1):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "diagonal must be 0 or 1, got {!r}".format(diagonal))
    first_region = _region_index(first_region, "first_region")
    second_region = _region_index(second_region, "second_region")
    corners = frame.local_corners
    if diagonal == 0:
        decomposition = "diagonal_0"
        triangles = (
            (corners[0], corners[1], corners[2]),
            (corners[0], corners[2], corners[3]),
        )
    else:
        decomposition = "diagonal_1"
        triangles = (
            (corners[1], corners[2], corners[3]),
            (corners[1], corners[3], corners[0]),
        )
    return FormationCorridorVehicleCover(
        vehicle_id=frame.vehicle_id,
        decomposition=decomposition,
        pieces=(
            FormationCorridorPiece(
                region_index=int(first_region),
                local_vertices=triangles[0]),
            FormationCorridorPiece(
                region_index=int(second_region),
                local_vertices=triangles[1]),
        ))


def single_region_template(request, chain, allocation, guide, *,
                           planning_context_sha256, region_indices,
                           route_index=0, candidate_index=0,
                           margin_m, margin_source="unspecified",
                           f_region_index=None):
    """Build a whole-OBB template with one assigned region per vehicle.

    ``region_indices`` is either one integer for all vehicles or a mapping
    ``vehicle_id -> region index``.  This is the Step 1 single-convex-region
    compiler input; Portal diagonal templates are built by Step 2 from the
    same selected candidate state.  ``f_region_index`` assigns the F reference
    point; it defaults to the single integer region assignment and must be
    stated explicitly for a per-vehicle mapping.
    """
    frames = vehicle_frames(request)
    if isinstance(region_indices, int) and not isinstance(
            region_indices, bool):
        mapping = {
            frame.vehicle_id: int(region_indices) for frame in frames}
    else:
        try:
            mapping = dict(region_indices)
        except (TypeError, ValueError) as exc:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "region_indices must be an integer or vehicle mapping") from exc
    if set(mapping) != {frame.vehicle_id for frame in frames}:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "region_indices vehicles {} do not match request vehicles {}".format(
                sorted(mapping), sorted(frame.vehicle_id for frame in frames)))
    if f_region_index is None:
        if isinstance(region_indices, int) and not isinstance(
                region_indices, bool):
            f_region_index = int(region_indices)
        else:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "a per-vehicle region mapping needs an explicit "
                "f_region_index")
    covers = tuple(
        whole_cover(frame, mapping[frame.vehicle_id]) for frame in frames)
    template = FormationCorridorTemplate(
        request_sha256=request_fingerprint(request),
        grid_sha256=chain.grid_sha256,
        planning_context_sha256=planning_context_sha256,
        model_sha256=formation_model_sha256(request),
        chain_digest=chain.digest,
        route_index=int(route_index),
        candidate_index=int(candidate_index),
        guide_source=str(guide.source),
        guide_sha256=formation_guide_digest(guide),
        allocation_digest=formation_allocation_digest(allocation),
        margin_m=float(margin_m),
        margin_source=str(margin_source),
        vehicle_frames=frames,
        segments=(FormationCorridorTemplateSegment(
            sigma_start=0.0, sigma_end=1.0, covers=covers,
            f_region_index=int(f_region_index)),),
    )
    validate_formation_template(
        template, request, chain, allocation=allocation, guide=guide,
        planning_context_sha256=planning_context_sha256)
    return template


def _frame_mismatch(template_frame, request_frame):
    return any(
        abs(float(a) - float(b)) > FORMATION_TOLERANCE_M
        for a, b in (
            (template_frame.offset_x, request_frame.offset_x),
            (template_frame.offset_y, request_frame.offset_y),
            (template_frame.length_m, request_frame.length_m),
            (template_frame.width_m, request_frame.width_m)))


def validate_formation_template(template, request, chain, *, allocation=None,
                                guide=None, planning_context_sha256=None):
    """Fail closed on any stale or cross-wired template identity."""
    if not isinstance(template, FormationCorridorTemplate):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "expected FormationCorridorTemplate")
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "expected CorridorChain")
    request_sha = request_fingerprint(request)
    if template.request_sha256 != request_sha:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "template request_sha256 does not match the request")
    if template.grid_sha256 != chain.grid_sha256:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "template grid_sha256 does not match the chain grid")
    if template.chain_digest != chain.digest:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "template chain_digest does not match the chain")
    if template.model_sha256 != formation_model_sha256(request):
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "template model_sha256 does not match the request model")
    if planning_context_sha256 is not None \
            and template.planning_context_sha256 != planning_context_sha256:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "template planning_context_sha256 does not match")
    if allocation is not None:
        if template.allocation_digest != formation_allocation_digest(
                allocation):
            raise VirtualAContractError(
                "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
                "template allocation_digest does not match the allocation")
    if guide is not None:
        if template.guide_sha256 != formation_guide_digest(guide):
            raise VirtualAContractError(
                "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
                "template guide_sha256 does not match the selected guide")
    request_frames = vehicle_frames(request)
    template_frames = template.vehicle_frames
    if len(template_frames) != len(request_frames):
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "template frame count does not match the request model")
    for template_frame, request_frame in zip(
            template_frames, request_frames):
        if template_frame.vehicle_id != request_frame.vehicle_id \
                or _frame_mismatch(template_frame, request_frame):
            raise VirtualAContractError(
                "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
                "template vehicle frame does not match the request model")
    region_count = len(chain.regions)
    for segment_index, segment in enumerate(template.segments):
        f_region = segment.f_region_index
        if f_region >= region_count:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "segment {} f_region_index={} outside the chain".format(
                    segment_index, f_region))
        if not isinstance(chain.regions[f_region], ConvexCorridor):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "chain region {} assigned to F is not a ConvexCorridor".format(
                    f_region))
        if allocation is not None:
            # The F assignment may only stay in the allocation region or move
            # to a directly neighbouring Chain region.  The midpoint is the
            # deterministic witness for a segment that spans several
            # allocation entries; a shared allocation boundary admits both
            # adjacent indices as legal bases.
            middle = 0.5 * (
                float(segment.sigma_start) + float(segment.sigma_end))
            bases = sorted({
                int(index) for left, right, index in allocation
                if float(left) - FORMATION_TOLERANCE_M <= middle
                <= float(right) + FORMATION_TOLERANCE_M})
            if not bases:
                nearest = min(
                    allocation,
                    key=lambda entry: min(
                        abs(middle - float(entry[0])),
                        abs(middle - float(entry[1]))))
                bases = [int(nearest[2])]
            if not any(abs(f_region - base) <= 1 for base in bases):
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "segment {} F region {} is not the allocation region or "
                    "a directly adjacent region {} at sigma={!r}".format(
                        segment_index, f_region, bases, middle))
        for cover in segment.covers:
            for piece_index, piece in enumerate(cover.pieces):
                if piece.region_index >= region_count:
                    raise VirtualAContractError(
                        "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                        "segment {} vehicle {} piece {} region_index={} "
                        "outside the chain".format(
                            segment_index, cover.vehicle_id, piece_index,
                            piece.region_index))
                if not isinstance(chain.regions[piece.region_index],
                                  ConvexCorridor):
                    raise VirtualAContractError(
                        "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                        "chain region {} is not a ConvexCorridor".format(
                            piece.region_index))
    return template


# ---------------------------------------------------------------------------
# Deterministic scalar row plan
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class FormationCorridorHalfspace:
    """One outward halfspace already rotated into the start-local frame."""

    halfspace_id: int
    region_index: int
    row: tuple
    limit: float


@dataclass(frozen=True)
class FormationCorridorVertexRows:
    """One body-local piece vertex and every assigned region halfspace."""

    vertex_id: int
    local_vertex: tuple
    halfspaces: tuple


@dataclass(frozen=True)
class FormationCorridorSample:
    """One (sigma, vehicle, convex body piece) constraint station."""

    piece_index: int
    q: float
    sigma: float
    source: str
    template_segment_index: int
    vehicle_id: int
    vehicle_index: int
    offset_x: float
    offset_y: float
    decomposition: str
    body_piece_index: int
    body_region_index: int
    vertices: tuple

    @property
    def scalar_row_count(self):
        return sum(
            len(vertex.halfspaces) for vertex in self.vertices)


@dataclass(frozen=True)
class FormationCorridorFSample:
    """One (sigma, F reference point, template segment) row station."""

    piece_index: int
    q: float
    sigma: float
    source: str
    template_segment_index: int
    region_index: int
    halfspaces: tuple

    @property
    def scalar_row_count(self):
        return len(self.halfspaces)


@dataclass(frozen=True)
class FormationCorridorConstraint:
    """The complete deterministic finite row plan of one template.

    ``f_groups[j]`` holds the F reference-point samples of Hermite piece ``j``
    and ``groups[j]`` the complete-OBB samples.  The numeric residual, CasADi
    ``g`` and the SciPy fallback must iterate the historical allocation rows,
    then the template F rows, then the template OBB rows in this exact order.
    Row counts are real scalar inequalities only; they are never inferred from
    a polygon or a marker.
    """

    template_id: str
    margin_m: float
    margin_source: str
    quadrature_points: int
    groups: tuple
    sample_count: int
    scalar_row_count: int
    rows_by_vehicle: tuple
    rows_by_vehicle_region: tuple
    segment_row_counts: tuple
    segment_count: int
    compiler_version: str = FORMATION_CORRIDOR_COMPILER_VERSION
    scope: str = FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1
    f_groups: tuple = ()
    f_sample_count: int = 0
    f_scalar_row_count: int = 0
    segment_f_row_counts: tuple = ()
    f_rows_by_segment: tuple = ()
    f_region_by_segment: tuple = ()

    @property
    def formation_obb_constraint_count(self):
        return int(self.scalar_row_count)

    @property
    def formation_reference_constraint_count(self):
        return int(self.f_scalar_row_count)

    def f_row_references(self, formation_g_base_index=0):
        """Flat F provenance rows in the same order as the appended scalars."""
        references = []
        index = int(formation_g_base_index)
        for group in self.f_groups:
            for sample in group:
                for halfspace in sample.halfspaces:
                    references.append({
                        "g_index": index,
                        "template_id": self.template_id,
                        "source": sample.source,
                        "sigma": float(sample.sigma),
                        "curve_piece": int(sample.piece_index),
                        "q": float(sample.q),
                        "template_segment_index": int(
                            sample.template_segment_index),
                        "region_id": int(halfspace.region_index),
                        "halfspace_id": int(halfspace.halfspace_id),
                        "row": [float(halfspace.row[0]),
                                float(halfspace.row[1])],
                        "limit": float(halfspace.limit),
                    })
                    index += 1
        return tuple(references)

    def row_references(self, formation_g_base_index=0):
        """Flat OBB provenance rows in the same order as the appended scalars.

        ``formation_g_base_index`` is the ``g`` index of the first OBB row,
        i.e. the F rows are counted separately before this block.
        """
        references = []
        index = int(formation_g_base_index)
        for group in self.groups:
            for sample in group:
                for vertex in sample.vertices:
                    for halfspace in vertex.halfspaces:
                        references.append({
                            "g_index": index,
                            "template_id": self.template_id,
                            "source": sample.source,
                            "sigma": float(sample.sigma),
                            "curve_piece": int(sample.piece_index),
                            "q": float(sample.q),
                            "vehicle_id": int(sample.vehicle_id),
                            "vehicle_index": int(sample.vehicle_index),
                            "template_segment_index": int(
                                sample.template_segment_index),
                            "decomposition": sample.decomposition,
                            "body_piece_index": int(
                                sample.body_piece_index),
                            "region_id": int(halfspace.region_index),
                            "halfspace_id": int(halfspace.halfspace_id),
                            "vertex_id": int(vertex.vertex_id),
                            "row": [float(halfspace.row[0]),
                                    float(halfspace.row[1])],
                            "limit": float(halfspace.limit),
                        })
                        index += 1
        return tuple(references)

    def as_dict(self):
        return {
            "role": "three_vehicle_complete_obb_finite_scalar_rows",
            "scope": self.scope,
            "template_id": self.template_id,
            "compiler_version": self.compiler_version,
            "margin_m": float(self.margin_m),
            "margin_source": self.margin_source,
            "quadrature_points": int(self.quadrature_points),
            "sample_count": int(self.sample_count),
            "scalar_row_count": int(self.scalar_row_count),
            "formation_obb_constraint_count": int(
                self.formation_obb_constraint_count),
            "formation_f_sample_count": int(self.f_sample_count),
            "formation_f_constraint_count": int(
                self.formation_reference_constraint_count),
            "f_rows_by_segment": [
                {"template_segment_index": int(index),
                 "f_region_index": int(self.f_region_by_segment[index]),
                 "rows": int(count)}
                for index, count in enumerate(self.f_rows_by_segment)],
            "segment_count": int(self.segment_count),
            "rows_by_vehicle": [
                {"vehicle_id": int(vehicle_id), "rows": int(count)}
                for vehicle_id, count in self.rows_by_vehicle],
            "rows_by_vehicle_region": [
                {"vehicle_id": int(vehicle_id), "region_id": int(region),
                 "rows": int(count)}
                for vehicle_id, region, count in self.rows_by_vehicle_region],
            "segment_row_counts": [
                int(count) for count in self.segment_row_counts],
            "is_three_vehicle_certificate": False,
            "certificate_note": (
                "finite NLP rows only; continuous certificate is separate"),
        }


def _sigma_piece(sigma, n):
    """The existing optimizer mapping from normalized sigma to Hermite q."""
    if sigma <= 0.0:
        return 0, 0.0
    if sigma >= 1.0:
        return n - 1, 1.0
    scaled = float(sigma) * n
    piece = int(math.floor(scaled))
    q = scaled - piece
    if piece >= n:
        return n - 1, 1.0
    if q < 0.0:
        return max(0, piece - 1), 1.0
    if q > 1.0:
        return min(n - 1, piece + 1), 0.0
    return piece, q


def _gauss_nodes(count):
    import numpy as np
    nodes, _weights = np.polynomial.legendre.leggauss(int(count))
    return tuple(float(v) for v in 0.5 * (nodes + 1.0))


def _close(first, second):
    return abs(float(first) - float(second)) <= FORMATION_TOLERANCE_M


def _rotated_region(region, start, margin):
    cosine = math.cos(float(start.psi))
    sine = math.sin(float(start.psi))
    rows = tuple(
        (float(row[0] * cosine + row[1] * sine),
         float(-row[0] * sine + row[1] * cosine))
        for row in region.A)
    limits = tuple(
        float(limit) - float(margin)
        - (float(row[0]) * float(start.x) + float(row[1]) * float(start.y))
        for row, limit in zip(region.A, region.b))
    return rows, limits


# ---------------------------------------------------------------------------
# Step 3: independent finite sample audit of one solved candidate.
#
# This is deliberately *not* a continuous certificate.  It evaluates the exact
# 19-variable path away from the NLP collocation stations so the solver can
# make at most one audit-directed cut.  Step 4 owns the interval certificate.
# ---------------------------------------------------------------------------

FORMATION_SAMPLE_AUDIT_SCHEMA = "virtual_a_formation_sample_audit_v1"
FORMATION_SAMPLE_AUDIT_CLEAR = "SAMPLED_CLEAR"
FORMATION_SAMPLE_AUDIT_VIOLATED = "DEFINITE_VIOLATION"
FORMATION_SAMPLE_AUDIT_UNCONFIRMED = "UNCONFIRMED"
FORMATION_SAMPLE_AUDIT_SPACING_SIGMA = 0.005
FORMATION_SAMPLE_AUDIT_MAX_SAMPLES = 2048
FORMATION_SAMPLE_AUDIT_DEFINITE_M = 1e-9


def _audit_result(status, template, *, sample_count, spacing, length,
                  reason="", witness=None, max_residual=None,
                  max_evidence=None, planned_count=None):
    return {
        "schema": FORMATION_SAMPLE_AUDIT_SCHEMA,
        "role": "independent_finite_sample_audit_not_a_certificate",
        "template_id": template.template_id,
        "status": status,
        "reason": str(reason),
        "sample_count": int(sample_count),
        "planned_sample_count": (
            int(sample_count) if planned_count is None
            else int(planned_count)),
        "spacing_sigma": float(spacing),
        "max_samples": int(FORMATION_SAMPLE_AUDIT_MAX_SAMPLES),
        "path_length_m": (None if length is None else float(length)),
        "definite_margin_m": float(FORMATION_SAMPLE_AUDIT_DEFINITE_M),
        "max_residual_m": (
            None if max_residual is None else float(max_residual)),
        "max_evidence_m": (
            None if max_evidence is None else float(max_evidence)),
        "witness": witness,
        "cut_sigma": (
            None if witness is None or status != FORMATION_SAMPLE_AUDIT_VIOLATED
            else float(witness["sigma"])),
        "is_continuous_certificate": False,
        "certificate_note": (
            "finite independent samples only; Step 4 must certify intervals"),
    }


def audit_formation_template_samples(
        path, template, chain, *,
        spacing_sigma=FORMATION_SAMPLE_AUDIT_SPACING_SIGMA,
        max_samples=FORMATION_SAMPLE_AUDIT_MAX_SAMPLES,
        definite_margin_m=FORMATION_SAMPLE_AUDIT_DEFINITE_M):
    """Audit the exact candidate path away from the compiled stations.

    The returned status is one of ``SAMPLED_CLEAR`` (no positive residual at
    any sample), ``DEFINITE_VIOLATION`` (at least one residual exceeds the
    path's independent position/angle error budget by more than
    ``definite_margin_m``) or ``UNCONFIRMED`` (numerical/budget/domain
    uncertainty, including near-boundary residuals).  Only a definite
    witness may direct a cut; an unconfirmed audit never invents one.
    """
    if not isinstance(template, FormationCorridorTemplate):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "sample audit requires a FormationCorridorTemplate")
    if not isinstance(chain, CorridorChain):
        raise VirtualAContractError(
            "INVALID_CORRIDOR_CHAIN", "sample audit requires a chain")
    spacing = _finite_row(spacing_sigma, "audit spacing_sigma")
    if not spacing > 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "audit spacing_sigma must be positive")
    maximum = int(max_samples)
    if isinstance(max_samples, bool) or maximum < 1:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "audit max_samples must be positive")
    definite = _finite_row(definite_margin_m, "audit definite_margin_m")
    if definite < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "audit definite_margin_m must be nonnegative")

    length_value = getattr(path, "length_m", None)
    try:
        length = float(length_value)
    except (TypeError, ValueError, OverflowError):
        return _audit_result(
            FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
            sample_count=0, spacing=spacing, length=None,
            reason="candidate path has no finite length")
    if not math.isfinite(length) or length <= 0.0:
        return _audit_result(
            FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
            sample_count=0, spacing=spacing, length=None,
            reason="candidate path length is not positive")
    if not callable(getattr(path, "state_at_s", None)):
        return _audit_result(
            FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
            sample_count=0, spacing=spacing, length=length,
            reason="candidate path has no state_at_s evaluator")

    # Midpoint samples keep every evaluation in the strict interior of one
    # fixed template segment.  Shared boundaries are Step 4's two-sided
    # interval job; this audit only directs at most one interior cut.
    sample_plan = []
    planned = 0
    for segment_index, segment in enumerate(template.segments):
        width = float(segment.sigma_end) - float(segment.sigma_start)
        if not width > 0.0:
            return _audit_result(
                FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                sample_count=0, spacing=spacing, length=length,
                reason="template segment has non-positive width")
        count = max(3, int(math.ceil(width / spacing)))
        planned += count
        sample_plan.append((segment_index, segment, count))
    if planned > maximum:
        return _audit_result(
            FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
            sample_count=0, spacing=spacing, length=length,
            planned_count=planned,
            reason="sample cap would be exceeded: {} > {}".format(
                planned, maximum))

    frames = {frame.vehicle_id: frame
              for frame in template.vehicle_frames}
    maximum_residual = None
    maximum_evidence = None
    best_witness = None
    definite_witness = None
    uncertain = False
    evaluated = 0
    for segment_index, segment, count in sample_plan:
        width = float(segment.sigma_end) - float(segment.sigma_start)
        cover_by_vehicle = {
            cover.vehicle_id: cover for cover in segment.covers}
        for sample_index in range(count):
            sigma = float(segment.sigma_start) + width * (
                (sample_index + 0.5) / count)
            station = sigma * length
            try:
                state = path.state_at_s(station)
            except Exception as exc:
                return _audit_result(
                    FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                    sample_count=evaluated, spacing=spacing, length=length,
                    planned_count=planned,
                    reason="candidate state evaluation failed: {}: {}".format(
                        type(exc).__name__, exc))
            try:
                position_error = float(state.position_error_bound_m)
                angle_error = float(state.angle_error_bound_rad)
                s_value = float(state.s)
                qf = state.qf
                x_qf = float(qf.x)
                y_qf = float(qf.y)
            except (TypeError, ValueError, AttributeError) as exc:
                return _audit_result(
                    FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                    sample_count=evaluated, spacing=spacing, length=length,
                    planned_count=planned,
                    reason="candidate state lacks finite error bounds: "
                           "{}: {}".format(type(exc).__name__, exc))
            if not (math.isfinite(position_error)
                    and math.isfinite(angle_error)
                    and position_error >= 0.0 and angle_error >= 0.0
                    and math.isfinite(s_value)
                    and math.isfinite(x_qf) and math.isfinite(y_qf)):
                return _audit_result(
                    FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                    sample_count=evaluated, spacing=spacing, length=length,
                    planned_count=planned,
                    reason="candidate state has non-finite/invalid bounds")
            vehicles = {
                int(vehicle.vehicle_id): vehicle
                for vehicle in state.vehicles}
            for frame in template.vehicle_frames:
                cover = cover_by_vehicle.get(frame.vehicle_id)
                if cover is None:
                    return _audit_result(
                        FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                        sample_count=evaluated, spacing=spacing, length=length,
                        planned_count=planned,
                        reason="template segment lacks vehicle {}".format(
                            frame.vehicle_id))
                vehicle = vehicles.get(frame.vehicle_id)
                if vehicle is None:
                    return _audit_result(
                        FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                        sample_count=evaluated, spacing=spacing, length=length,
                        planned_count=planned,
                        reason="candidate state lacks vehicle {}".format(
                            frame.vehicle_id))
                try:
                    center_x = float(vehicle.x)
                    center_y = float(vehicle.y)
                    theta = float(vehicle.theta)
                except (TypeError, ValueError, AttributeError) as exc:
                    return _audit_result(
                        FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                        sample_count=evaluated, spacing=spacing, length=length,
                        planned_count=planned,
                        reason="vehicle {} state is invalid: {}".format(
                            frame.vehicle_id, exc))
                cosine = math.cos(theta)
                sine = math.sin(theta)
                # The mapped heading and centre share the path's declared
                # position/angle bounds.  A corner can therefore move by at
                # most position_error + half_diagonal * angle_error.
                corner_error = (
                    position_error + 0.5 * math.hypot(
                        frame.length_m, frame.width_m) * angle_error
                    + 1e-12)
                for body_piece_index, piece in enumerate(cover.pieces):
                    region_index = int(piece.region_index)
                    if not 0 <= region_index < len(chain.regions):
                        return _audit_result(
                            FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
                            sample_count=evaluated, spacing=spacing, length=length,
                            planned_count=planned,
                            reason="template piece region {} is outside the "
                                   "chain".format(region_index))
                    region = chain.regions[region_index]
                    for vertex_id, (local_x, local_y) in enumerate(
                            piece.local_vertices):
                        vertex_x = center_x + cosine * local_x - sine * local_y
                        vertex_y = center_y + sine * local_x + cosine * local_y
                        for halfspace_id, (row, limit) in enumerate(
                                zip(region.A, region.b)):
                            residual = (
                                float(row[0]) * vertex_x
                                + float(row[1]) * vertex_y
                                - (float(limit) - float(template.margin_m)))
                            row_norm = math.hypot(
                                float(row[0]), float(row[1]))
                            bound = row_norm * corner_error
                            evidence = residual - bound
                            if maximum_residual is None or \
                                    residual > maximum_residual:
                                maximum_residual = residual
                            if maximum_evidence is None or \
                                    evidence > maximum_evidence:
                                maximum_evidence = evidence
                            witness = {
                                "sigma": float(sigma),
                                "s_m": float(station),
                                "segment_index": int(segment_index),
                                "vehicle_id": int(frame.vehicle_id),
                                "decomposition": cover.decomposition,
                                "body_piece_index": int(body_piece_index),
                                "region_index": int(region_index),
                                "halfspace_id": int(halfspace_id),
                                "vertex_id": int(vertex_id),
                                "point_x": float(vertex_x),
                                "point_y": float(vertex_y),
                                "residual_m": float(residual),
                                "error_budget_m": float(bound),
                                "evidence_m": float(evidence),
                            }
                            if best_witness is None or \
                                    evidence > best_witness["evidence_m"]:
                                best_witness = witness
                            if evidence > definite:
                                if definite_witness is None or \
                                        evidence > definite_witness[
                                            "evidence_m"]:
                                    definite_witness = witness
                            elif residual > 0.0:
                                uncertain = True
            evaluated += 1

    if definite_witness is not None:
        return _audit_result(
            FORMATION_SAMPLE_AUDIT_VIOLATED, template,
            sample_count=evaluated, spacing=spacing, length=length,
            planned_count=planned,
            reason="definite OBB violation beyond the path error budget",
            witness=definite_witness,
            max_residual=maximum_residual,
            max_evidence=maximum_evidence)
    if uncertain:
        return _audit_result(
            FORMATION_SAMPLE_AUDIT_UNCONFIRMED, template,
            sample_count=evaluated, spacing=spacing, length=length,
            planned_count=planned,
            reason="positive residual inside the path error budget",
            witness=best_witness,
            max_residual=maximum_residual,
            max_evidence=maximum_evidence)
    return _audit_result(
        FORMATION_SAMPLE_AUDIT_CLEAR, template,
        sample_count=evaluated, spacing=spacing, length=length,
        planned_count=planned,
        max_residual=maximum_residual,
        max_evidence=maximum_evidence)


def build_formation_corridor_constraint(
        template, request, chain, config, *,
        allocation=None, guide=None, planning_context_sha256=None,
        extra_cuts_sigma=()):
    """Compile one template into the deterministic per-piece row plan.

    The initial stations are every Hermite endpoint, every current Gauss node,
    every template/allocation boundary and the selected candidate's extra
    cuts.  A cut exactly on a shared boundary is evaluated by both neighbouring
    template assignments.  Step 3 adds at most one independent-audit-directed
    cut per template through ``extra_cuts_sigma``.
    """
    if template is None:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "union scope requires one fixed FormationCorridorTemplate")
    validate_formation_template(
        template, request, chain, allocation=allocation, guide=guide,
        planning_context_sha256=planning_context_sha256)
    n = int(config.intervals)
    if n < 1:
        raise VirtualAContractError(
            "INVALID_CONFIG", "intervals must be positive")
    quadrature = int(config.corridor_quadrature_points)
    if quadrature < 3:
        raise VirtualAContractError(
            "INVALID_CONFIG", "corridor_quadrature_points must be after 3")
    margin = float(template.margin_m)
    if margin < 0.0 or not math.isfinite(margin):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template margin_m must be finite and nonnegative")
    # The OBB buffer may be more conservative than the F reference-point
    # inset, but it must never be smaller than the active corridor margin:
    # otherwise the new rows could be easier to satisfy than the historical
    # F-only rows under the same configuration.
    active_margin = corridor_effective_margin(chain, config)
    if margin + FORMATION_TOLERANCE_M < float(active_margin):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template margin_m={!r} is below the active corridor margin "
            "{!r}".format(margin, float(active_margin)))
    start = request.start_qf
    used_regions = sorted({
        piece.region_index
        for segment in template.segments
        for cover in segment.covers
        for piece in cover.pieces} | {
            int(segment.f_region_index) for segment in template.segments})
    rotated = {
        region_index: _rotated_region(
            chain.regions[region_index], start, margin)
        for region_index in used_regions}

    # Base stations and their provenance.  A station can be present for more
    # than one reason; the strongest/most explicit reason wins.
    sources = {}

    def add_station(sigma, source):
        value = float(sigma)
        if not (0.0 <= value <= 1.0) or not math.isfinite(value):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_CUT",
                "sigma={!r} outside [0, 1]".format(value))
        priority = {
            "targeted_cut": 5,
            "allocation_boundary": 4,
            "template_segment_boundary": 3,
            "gauss": 2,
            "hermite_endpoint": 1,
        }
        existing = sources.get(round(value, 15))
        if existing is None or priority[source] > priority[existing]:
            sources[round(value, 15)] = source

    for j in range(n + 1):
        add_station(float(j) / n, "hermite_endpoint")
    nodes = _gauss_nodes(quadrature)
    for j in range(n):
        for node in nodes:
            add_station((float(j) + node) / n, "gauss")
    if allocation is not None:
        for left, right, _index in allocation:
            add_station(left, "allocation_boundary")
            add_station(right, "allocation_boundary")
    for segment in template.segments:
        add_station(segment.sigma_start, "template_segment_boundary")
        add_station(segment.sigma_end, "template_segment_boundary")
    for sigma in extra_cuts_sigma:
        add_station(sigma, "targeted_cut")

    frame_by_id = {
        frame.vehicle_id: frame for frame in template.vehicle_frames}
    groups = [[] for _ in range(n)]
    f_groups = [[] for _ in range(n)]
    rows_by_vehicle = {}
    rows_by_vehicle_region = {}
    segment_row_counts = [0 for _ in template.segments]
    segment_f_row_counts = [0 for _ in template.segments]
    sample_count = 0
    f_sample_count = 0
    scalar_row_count = 0
    f_scalar_row_count = 0
    station_items = sorted(sources.items())
    for segment_index, segment in enumerate(template.segments):
        selected = [
            (sigma_rounded, sigma_rounded, source)
            for sigma_rounded, source in station_items
            if segment.sigma_start - FORMATION_TOLERANCE_M <= sigma_rounded
            <= segment.sigma_end + FORMATION_TOLERANCE_M]
        # Endpoints must always be present even if numerical rounding moved
        # them just outside the base stations' tolerance band.
        if not any(_close(sigma, segment.sigma_start)
                   for _rounded, sigma, _source in selected):
            selected.append((segment.sigma_start, segment.sigma_start,
                             "template_segment_boundary"))
        if not any(_close(sigma, segment.sigma_end)
                   for _rounded, sigma, _source in selected):
            selected.append((segment.sigma_end, segment.sigma_end,
                             "template_segment_boundary"))
        selected.sort(key=lambda entry: entry[1])
        deduped = []
        for item in selected:
            if deduped and _close(item[1], deduped[-1][1]):
                # Keep the stronger provenance for the shared station.
                if item[2] == "targeted_cut":
                    deduped[-1] = item
                continue
            deduped.append(item)
        for sigma_rounded, sigma, source in deduped:
            piece, q = _sigma_piece(sigma, n)
            f_region_index = int(segment.f_region_index)
            f_rows, f_limits = rotated[f_region_index]
            f_halfspaces = tuple(
                FormationCorridorHalfspace(
                    halfspace_id=int(halfspace_id),
                    region_index=int(f_region_index),
                    row=f_rows[halfspace_id],
                    limit=float(f_limits[halfspace_id]))
                for halfspace_id in range(len(f_rows)))
            f_groups[piece].append(FormationCorridorFSample(
                piece_index=int(piece),
                q=float(q),
                sigma=float(sigma),
                source=str(source),
                template_segment_index=int(segment_index),
                region_index=int(f_region_index),
                halfspaces=f_halfspaces))
            segment_f_row_counts[segment_index] += len(f_halfspaces)
            f_scalar_row_count += len(f_halfspaces)
            f_sample_count += 1
            for vehicle_index, cover in enumerate(segment.covers):
                frame = frame_by_id[cover.vehicle_id]
                for body_piece_index, piece_record in enumerate(cover.pieces):
                    region_index = piece_record.region_index
                    rows, limits = rotated[region_index]
                    vertex_rows = []
                    for vertex_id, local_vertex in enumerate(
                            piece_record.local_vertices):
                        halfspaces = tuple(
                            FormationCorridorHalfspace(
                                halfspace_id=int(halfspace_id),
                                region_index=int(region_index),
                                row=rows[halfspace_id],
                                limit=float(limits[halfspace_id]))
                            for halfspace_id in range(len(rows)))
                        vertex_rows.append(FormationCorridorVertexRows(
                            vertex_id=int(vertex_id),
                            local_vertex=(
                                float(local_vertex[0]),
                                float(local_vertex[1])),
                            halfspaces=halfspaces))
                        segment_row_counts[segment_index] += len(
                            halfspaces)
                        rows_by_vehicle[cover.vehicle_id] = (
                            rows_by_vehicle.get(cover.vehicle_id, 0)
                            + len(halfspaces))
                        key = (cover.vehicle_id, region_index)
                        rows_by_vehicle_region[key] = (
                            rows_by_vehicle_region.get(key, 0)
                            + len(halfspaces))
                        scalar_row_count += len(halfspaces)
                    sample = FormationCorridorSample(
                        piece_index=int(piece),
                        q=float(q),
                        sigma=float(sigma),
                        source=str(source),
                        template_segment_index=int(segment_index),
                        vehicle_id=int(cover.vehicle_id),
                        vehicle_index=int(vehicle_index),
                        offset_x=float(frame.offset_x),
                        offset_y=float(frame.offset_y),
                        decomposition=str(cover.decomposition),
                        body_piece_index=int(body_piece_index),
                        body_region_index=int(region_index),
                        vertices=tuple(vertex_rows))
                    groups[piece].append(sample)
                    sample_count += 1
    # Samples are appended per template segment.  Within a Hermite piece keep
    # them sorted by sigma and then by template segment/vehicle/piece so the
    # row order is stable and reviewable.
    for group in groups:
        group.sort(key=lambda sample: (
            float(sample.sigma), int(sample.template_segment_index),
            int(sample.vehicle_index), int(sample.body_piece_index)))
    for group in f_groups:
        group.sort(key=lambda sample: (
            float(sample.sigma), int(sample.template_segment_index),
            int(sample.region_index)))
    rows_by_vehicle_tuple = tuple(sorted(rows_by_vehicle.items()))
    rows_by_vehicle_region_tuple = tuple(sorted(
        (vehicle_id, region_id, count)
        for (vehicle_id, region_id), count in rows_by_vehicle_region.items()))
    return FormationCorridorConstraint(
        template_id=template.template_id,
        margin_m=margin,
        margin_source=str(template.margin_source),
        quadrature_points=quadrature,
        groups=tuple(tuple(group) for group in groups),
        sample_count=int(sample_count),
        scalar_row_count=int(scalar_row_count),
        rows_by_vehicle=rows_by_vehicle_tuple,
        rows_by_vehicle_region=rows_by_vehicle_region_tuple,
        segment_row_counts=tuple(segment_row_counts),
        segment_count=len(template.segments),
        f_groups=tuple(tuple(group) for group in f_groups),
        f_sample_count=int(f_sample_count),
        f_scalar_row_count=int(f_scalar_row_count),
        segment_f_row_counts=tuple(segment_f_row_counts),
        f_rows_by_segment=tuple(segment_f_row_counts),
        f_region_by_segment=tuple(
            int(segment.f_region_index) for segment in template.segments))


# ---------------------------------------------------------------------------
# Step 2: finite Portal template candidates from one selected assessment
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class FormationCoverOption:
    """One finite convex cover choice for a single vehicle."""

    decomposition: str
    region_indices: tuple

    @property
    def body_piece_count(self):
        return 1 if self.decomposition == "whole" else 2

    def as_dict(self):
        return {
            "decomposition": str(self.decomposition),
            "region_indices": [int(value) for value in self.region_indices],
            "body_piece_count": int(self.body_piece_count),
        }


def _world_vertices(local_vertices, center, theta):
    cosine, sine = math.cos(theta), math.sin(theta)
    return tuple(
        (float(center[0]) + cosine * float(x) - sine * float(y),
         float(center[1]) + sine * float(x) + cosine * float(y))
        for x, y in local_vertices)


def _piece_inside_region(region, world_vertices, margin_m, tol_m):
    for x, y in world_vertices:
        for row, limit in zip(region.A, region.b):
            value = row[0] * x + row[1] * y - (float(limit) - float(margin_m))
            if value > float(tol_m):
                return False
    return True


def finite_cover_options(frame, center, theta, regions, region_candidates,
                         margin_m=0.0, tol_m=1e-9):
    """Deterministically test the finite whole/diagonal covers at one pose.

    ``regions`` is the chain's convex-region sequence.  ``region_candidates``
    are the only regions the caller is willing to assign (the Task 2 witness
    region ids plus the local allocation neighbours).  A cover is accepted
    only when every vertex of every convex body piece lies in its assigned
    region halfspaces with the requested margin.  This is a sufficient
    condition for the piece and therefore for the complete vehicle rectangle.
    """
    if not isinstance(frame, FormationCorridorVehicleFrame):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "finite_cover_options needs a FormationCorridorVehicleFrame")
    center = (_finite_row(center[0], "cover.center.x"),
              _finite_row(center[1], "cover.center.y"))
    theta = _finite_row(theta, "cover.theta")
    margin = _finite_row(margin_m, "cover.margin_m")
    if margin < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "cover margin must be nonnegative")
    tolerance = _finite_row(tol_m, "cover.tol_m")
    if tolerance < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "cover tolerance must be nonnegative")
    candidates = sorted({
        _region_index(value, "region_candidates")
        for value in region_candidates})
    for index in candidates:
        if index >= len(regions):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "region_candidates index {} outside the chain".format(index))
    options = set()
    corners_world = _world_vertices(
        frame.local_corners, center, theta)
    for region_index in candidates:
        if _piece_inside_region(
                regions[region_index], corners_world, margin, tolerance):
            options.add(FormationCoverOption(
                decomposition="whole", region_indices=(region_index,)))
    pairs = []
    for index in candidates:
        if index + 1 in candidates:
            pairs.append((index, index + 1))
    for left, right in pairs:
        for diagonal in (0, 1):
            for first_region, second_region in (
                    (left, right), (right, left)):
                cover = diagonal_cover(
                    frame, diagonal, first_region, second_region)
                accepted = True
                for piece in cover.pieces:
                    vertices = _world_vertices(
                        piece.local_vertices, center, theta)
                    if not _piece_inside_region(
                            regions[piece.region_index], vertices, margin,
                            tolerance):
                        accepted = False
                        break
                if accepted:
                    options.add(FormationCoverOption(
                        decomposition="diagonal_{}".format(diagonal),
                        region_indices=(int(first_region),
                                        int(second_region))))
    return tuple(sorted(
        options,
        key=lambda option: (
            option.body_piece_count, option.decomposition,
            option.region_indices)))


@dataclass(frozen=True)
class FormationTemplateCandidateSet:
    """Bounded finite template candidates built from one diagnostic state."""

    assessment_candidate_id: str
    templates: tuple
    cuts: tuple
    per_segment_vehicle_option_counts: tuple
    truncated_option_count: int
    station_count: int
    synthetic_station_count: int

    def as_dict(self):
        return {
            "assessment_candidate_id": self.assessment_candidate_id,
            "template_ids": [
                template.template_id for template in self.templates],
            "template_count": len(self.templates),
            "cuts": [float(value) for value in self.cuts],
            "per_segment_vehicle_option_counts": [
                [int(value) for value in row]
                for row in self.per_segment_vehicle_option_counts],
            "truncated_option_count": int(self.truncated_option_count),
            "station_count": int(self.station_count),
            "synthetic_station_count": int(self.synthetic_station_count),
        }


def _guide_pose_at_sigma(guide, sigma):
    """Linear guide pose at a normalized arc position, for template cuts."""
    if not isinstance(guide, GuideRoute):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "guide must be a GuideRoute")
    points = tuple(guide.points)
    if len(points) < 2:
        raise VirtualAContractError(
            "COVER_TEMPLATE_INSUFFICIENT", "guide has fewer than two points")
    lengths = [
        math.hypot(points[index + 1][0] - points[index][0],
                   points[index + 1][1] - points[index][1])
        for index in range(len(points) - 1)]
    total = math.fsum(lengths)
    if total <= 0.0:
        raise VirtualAContractError(
            "COVER_TEMPLATE_INSUFFICIENT", "guide length is zero")
    target = min(max(float(sigma), 0.0), 1.0) * total
    running = 0.0
    for index, length in enumerate(lengths):
        if length <= 0.0:
            continue
        if target <= running + length or index == len(lengths) - 1:
            ratio = (target - running) / length
            ratio = min(max(ratio, 0.0), 1.0)
            x = points[index][0] + ratio * (
                points[index + 1][0] - points[index][0])
            y = points[index][1] + ratio * (
                points[index + 1][1] - points[index][1])
            heading = math.atan2(
                points[index + 1][1] - points[index][1],
                points[index + 1][0] - points[index][0])
            return {
                "x": float(x),
                "y": float(y),
                "heading_rad": float(heading),
                "kappa": 0.0,
            }
        running += length
    raise VirtualAContractError(
        "COVER_TEMPLATE_INSUFFICIENT", "guide pose could not be evaluated")


def _allocation_indices_at(allocation, sigma):
    indices = []
    tolerance = FORMATION_TOLERANCE_M
    for left, right, index in allocation:
        if float(left) - tolerance <= float(sigma) <= float(right) + tolerance:
            indices.append(int(index))
    if not indices:
        nearest = min(
            allocation,
            key=lambda entry: min(abs(float(sigma) - float(entry[0])),
                                 abs(float(sigma) - float(entry[1]))))
        indices.append(int(nearest[2]))
    return indices


def _assessment_identity_matches(assessment, request, chain, guide,
                                 allocation, planning_context_sha256):
    if not isinstance(assessment, dict):
        raise VirtualAContractError(
            "COVER_TEMPLATE_INSUFFICIENT",
            "a diagnostic formation assessment is required")
    if str(assessment.get("schema", "")) != ASSESSMENT_SCHEMA:
        raise VirtualAContractError(
            "INVALID_FORMATION_ASSESSMENT",
            "assessment schema={!r} expected {!r}".format(
                assessment.get("schema"), ASSESSMENT_SCHEMA))
    if str(assessment.get("candidate_certification", "")) != \
            CANDIDATE_CERTIFICATION_DIAGNOSTIC_ONLY:
        raise VirtualAContractError(
            "INVALID_FORMATION_ASSESSMENT",
            "assessment must remain DIAGNOSTIC_ONLY")
    if bool(assessment.get("formation_obb_hard_constraints", False)) or \
            bool(assessment.get("continuously_certified", False)):
        raise VirtualAContractError(
            "INVALID_FORMATION_ASSESSMENT",
            "diagnostic assessment falsely claims hard constraints/certificate")
    for field in ("route_index", "round_index"):
        value = assessment.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "assessment {} must be a nonnegative integer".format(field))
    context = str(planning_context_sha256 or "").strip().lower()
    checks = (
        ("request_sha256", request_fingerprint(request)),
        ("grid_sha256", chain.grid_sha256),
        ("chain_digest", chain.digest),
        ("planning_context_sha256", context),
    )
    for field, expected in checks:
        if str(assessment.get(field, "")).strip().lower() != str(
                expected).strip().lower():
            raise VirtualAContractError(
                "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
                "assessment {} does not match".format(field))
    if str(assessment.get("selected_guide_source", "")) != str(guide.source):
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "assessment selected_guide_source does not match")
    points_digest = _canonical_sha256(
        [[float(x), float(y)] for x, y in guide.points])
    if str(assessment.get("selected_guide_sha256", "")).strip().lower() != \
            points_digest:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "assessment selected_guide_sha256 does not match")
    expected_allocation = [
        [float(left), float(right), int(index)]
        for left, right, index in allocation]
    if assessment.get("legacy_f_allocation") != expected_allocation:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "assessment legacy_f_allocation does not match")
    if bool(assessment.get("sample_cap", False)) or int(
            assessment.get("dropped_required_stations", 0) or 0) > 0:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_UNCONFIRMED",
            "assessment dropped required stations; template coverage would "
            "not cover the full guide")
    if assessment.get("incomplete_reasons"):
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_UNCONFIRMED",
            "assessment incomplete: {}".format(
                ", ".join(str(value) for value in
                          assessment.get("incomplete_reasons"))))
    stations = assessment.get("station_coverage")
    if not isinstance(stations, list) or not stations:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_UNCONFIRMED",
            "assessment has no station coverage witnesses")
    guide_points = tuple(guide.points)
    guide_length = math.fsum(
        math.hypot(guide_points[index + 1][0] - guide_points[index][0],
                   guide_points[index + 1][1] - guide_points[index][1])
        for index in range(len(guide_points) - 1))
    route_length = assessment.get("route_length_m")
    try:
        route_length = float(route_length)
    except (TypeError, ValueError, OverflowError) as exc:
        raise VirtualAContractError(
            "INVALID_FORMATION_ASSESSMENT",
            "route_length_m value={!r}".format(route_length)) from exc
    if not math.isfinite(route_length) or route_length < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_ASSESSMENT",
            "route_length_m must be finite and nonnegative")
    if abs(route_length - guide_length) > max(
            1e-9, 1e-9 * max(route_length, guide_length)):
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "assessment route_length_m={!r} does not match the selected "
            "guide length {!r}".format(route_length, guide_length))
    station_count = assessment.get("station_count")
    sampled_station_count = assessment.get("sampled_station_count")
    for field, value in (
            ("station_count", station_count),
            ("sampled_station_count", sampled_station_count)):
        if isinstance(value, bool) or not isinstance(value, int) or \
                value != len(stations):
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "assessment {}={!r} does not match its {} station views"
                .format(field, value, len(stations)))
    seen_sigmas = set()
    for entry in stations:
        if not isinstance(entry, dict):
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "station coverage entries must be objects")
        try:
            sigma_value = float(entry.get("sigma"))
            s_value = float(entry.get("s_m"))
        except (TypeError, ValueError, OverflowError) as exc:
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "station sigma/s_m must be finite numbers") from exc
        if not math.isfinite(sigma_value) or not 0.0 <= sigma_value <= 1.0:
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "station sigma={!r} outside [0, 1]".format(sigma_value))
        if not math.isfinite(s_value) or s_value < 0.0:
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "station s_m={!r} must be finite and nonnegative".format(
                    s_value))
        if abs(s_value - sigma_value * route_length) > max(
                1e-9, 1e-9 * route_length):
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "station sigma={!r} and s_m={!r} are inconsistent with "
                "route_length_m={!r}".format(
                    sigma_value, s_value, route_length))
        key = round(sigma_value, 12)
        if key in seen_sigmas:
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "duplicate station sigma={!r}".format(sigma_value))
        seen_sigmas.add(key)
    expected_candidate_id = _canonical_sha256({
        "schema": ASSESSMENT_SCHEMA,
        "request_sha256": str(request_fingerprint(request)),
        "grid_sha256": str(chain.grid_sha256),
        "planning_context_sha256": str(context),
        "route_index": int(assessment.get("route_index")),
        "round_index": int(assessment.get("round_index")),
        "chain_digest": str(chain.digest),
        "guide_source": str(guide.source),
        "guide_points": [[float(x), float(y)] for x, y in guide.points],
    })
    if str(assessment.get("candidate_id", "")).strip().lower() != \
            expected_candidate_id:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "assessment candidate_id does not match its candidate state")


def _station_options(station, request, chain, allocation, frames,
                    margin_m, tol_m):
    """Per-vehicle finite cover options at one witness station."""
    pose = station.get("guide_pose") or {}
    if not bool(pose.get("pose_reliable", False)) or \
            pose.get("heading_rad") is None or pose.get("kappa") is None:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_UNCONFIRMED",
            "station sigma={!r} has no reliable guide pose".format(
                station.get("sigma")))
    qf = FormationPose(
        float(pose["x"]), float(pose["y"]), float(pose["heading_rad"]))
    jet = CurvatureJet(float(pose["kappa"]), 0.0, 0.0, 0.0)
    try:
        mapped = map_spatial_state(
            qf, jet, request.model, s=float(station.get("s_m", 0.0)))
    except VirtualAContractError as exc:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_UNCONFIRMED",
            "station sigma={!r} map state rejected: {}".format(
                station.get("sigma"), exc.reason))
    vehicles = {vehicle.vehicle_id: vehicle
                for vehicle in mapped.vehicles}
    sigma = float(station["sigma"])
    allocation_indices = _allocation_indices_at(allocation, sigma)
    local_candidates = []
    for index in allocation_indices:
        for neighbour in (index - 1, index, index + 1):
            if 0 <= neighbour < len(chain.regions):
                local_candidates.append(neighbour)
    result = {}
    for frame in frames:
        witness = (station.get("vehicles") or {}).get(str(frame.vehicle_id))
        region_candidates = list(local_candidates)
        if witness is not None:
            # The witness is a region hint, never the safety check.  All
            # covers below are verified against the actual OBB geometry, so
            # OUTSIDE_TRANSITION_WINDOW (no union predicate was sampled) does
            # not by itself make the whole-OBB option unsafe.
            region_candidates.extend(
                int(value) for value in witness.get("region_ids") or [])
        vehicle = vehicles.get(frame.vehicle_id)
        if vehicle is None:
            raise VirtualAContractError(
                "FORMATION_CORRIDOR_UNCONFIRMED",
                "station sigma={!r} has no vehicle {}".format(
                    sigma, frame.vehicle_id))
        options = finite_cover_options(
            frame, (vehicle.x, vehicle.y), vehicle.theta,
            chain.regions, sorted(set(region_candidates)),
            margin_m=margin_m, tol_m=tol_m)
        if not options:
            raise VirtualAContractError(
                "COVER_TEMPLATE_INSUFFICIENT",
                "station sigma={!r} vehicle {} has no finite whole/diagonal "
                "cover in regions {}".format(
                    sigma, frame.vehicle_id, sorted(set(region_candidates))))
        result[frame.vehicle_id] = options
    return result


def _choice_cover(frame, option):
    if option.decomposition == "whole":
        return whole_cover(frame, option.region_indices[0])
    diagonal = int(option.decomposition.rsplit("_", 1)[1])
    return diagonal_cover(
        frame, diagonal, option.region_indices[0], option.region_indices[1])


def _region_contains_point(region, point, margin_m, tol_m):
    """Outward-inset point test used to validate a witness F assignment."""
    return all(
        float(row[0]) * float(point[0])
        + float(row[1]) * float(point[1])
        - float(limit) + float(margin_m) <= float(tol_m)
        for row, limit in zip(region.A, region.b))


def _station_f_options(station, chain, allocation, margin_m, tol_m):
    """Legal single F regions at one witness station.

    The candidate set is the allocation region plus its directly neighbouring
    Chain regions, intersected with the regions whose outward inset actually
    contains the reliable guide F point.  No other region, Portal window or
    historical candidate is consulted.  An empty set is a safe template
    insufficiency, never a physical no-path claim.
    """
    pose = station.get("guide_pose") or {}
    if not bool(pose.get("pose_reliable", False)) \
            or pose.get("x") is None or pose.get("y") is None:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_UNCONFIRMED",
            "station sigma={!r} has no reliable guide F pose".format(
                station.get("sigma")))
    point = (float(pose["x"]), float(pose["y"]))
    sigma = float(station["sigma"])
    bases = sorted(set(_allocation_indices_at(allocation, sigma)))
    candidates = sorted({
        neighbour
        for base in bases
        for neighbour in (base - 1, base, base + 1)
        if 0 <= neighbour < len(chain.regions)})
    valid = tuple(
        index for index in candidates
        if _region_contains_point(
            chain.regions[index], point, margin_m, tol_m))
    if not valid:
        raise VirtualAContractError(
            "COVER_TEMPLATE_INSUFFICIENT",
            "station sigma={!r} has no legal F region among {} for the "
            "guide reference point".format(sigma, candidates))
    return bases, valid


def _f_choice(bases, valid, policy):
    """Deterministic F region for one station under one profile policy.

    ``allocation`` keeps the historical allocation region (the nearest
    allocation index, with the lower index winning a shared-boundary tie).
    ``forward`` takes the largest legal neighbouring region and ``backward``
    the smallest; both are still restricted to the station's allocation
    neighbourhood, so no region is skipped and no search is performed.
    """
    if policy == "forward":
        return int(valid[-1])
    if policy == "backward":
        return int(valid[0])
    return int(min(
        valid,
        key=lambda index: (
            min(abs(int(index) - int(base)) for base in bases),
            int(index))))


def _f_transition_cut(guide, left_sigma, right_sigma, left_region,
                      right_region, chain, margin_m, tol_m):
    """A sigma between the runs where the guide lies in both F regions.

    The two-sided template cut will be checked against both regions.  Placing
    it where the guide itself is inside both makes the cut a witness-backed
    transition; if the guide never passes through the shared portal, the
    finite template cannot express this assignment and the caller rejects it.
    """
    candidate_fractions = [0.5]
    candidate_fractions.extend(
        float(step) / 65.0 for step in range(1, 65))
    seen = set()
    for fraction in candidate_fractions:
        if fraction in seen:
            continue
        seen.add(fraction)
        sigma = float(left_sigma) + float(fraction) * (
            float(right_sigma) - float(left_sigma))
        pose = _guide_pose_at_sigma(guide, sigma)
        point = (float(pose["x"]), float(pose["y"]))
        if _region_contains_point(
                chain.regions[left_region], point, margin_m, tol_m) \
                and _region_contains_point(
                    chain.regions[right_region], point, margin_m, tol_m):
            return float(sigma)
    return None


def build_formation_template_candidates(
        request, chain, guide, allocation, assessment, config, *,
        planning_context_sha256, margin_m, margin_source,
        max_templates=3, tol_m=1e-9):
    """Build at most three fixed whole/diagonal templates for one guide.

    The assessment supplies the exact candidate identity, allocation and
    witness stations.  Region choices are always checked against the actual
    vehicle rectangle; a union-covered OBB that none of the finite whole /
    two-diagonal assignments can express raises
    ``COVER_TEMPLATE_INSUFFICIENT`` (never a physical no-path claim).

    The returned candidate set is the only finite template set Step 3 may
    consume for this selected guide.  It is not a certificate.
    """
    if isinstance(max_templates, bool) or not isinstance(
            max_templates, numbers.Integral) or not (1 <= int(max_templates) <= 3):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "max_templates must be in [1, 3]")
    max_templates = int(max_templates)
    context = str(planning_context_sha256 or "").strip().lower()
    if not _formation_sha256_hex(context):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "planning_context_sha256 must be 64 lowercase hex characters")
    if not isinstance(allocation, (tuple, list)) or not allocation:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "a normalized allocation is required")
    margin = _finite_row(margin_m, "template margin_m")
    if margin < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template margin_m must be nonnegative")
    active_margin = float(corridor_effective_margin(chain, config))
    if margin + FORMATION_TOLERANCE_M < active_margin:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template margin_m={!r} is below the active corridor margin "
            "{!r}".format(margin, active_margin))
    _assessment_identity_matches(
        assessment, request, chain, guide, allocation, context)
    frames = vehicle_frames(request)
    tolerance = _finite_row(tol_m, "template tol_m")
    if tolerance < 0.0:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template tolerance must be nonnegative")

    station_map = {}
    synthetic = 0
    for entry in assessment["station_coverage"]:
        if not isinstance(entry, dict):
            raise VirtualAContractError(
                "INVALID_FORMATION_ASSESSMENT",
                "station coverage entries must be objects")
        if str(entry.get("map_status", "")) != MAP_CLEAR_AT_SAMPLES:
            raise VirtualAContractError(
                "FORMATION_CORRIDOR_UNCONFIRMED",
                "assessment station sigma={!r} map status={!r}".format(
                    entry.get("sigma"), entry.get("map_status")))
        sigma = entry.get("sigma")
        if sigma is None:
            raise VirtualAContractError(
                "COVER_TEMPLATE_INSUFFICIENT",
                "assessment station has no normalized sigma")
        sigma = _finite_row(sigma, "station sigma")
        key = round(sigma, 12)
        station_map[key] = {
            "sigma": sigma,
            "s_m": float(entry.get("s_m", 0.0)),
            "guide_pose": dict(entry.get("guide_pose") or {}),
            "vehicles": dict(entry.get("vehicles") or {}),
            "synthetic": False,
        }

    # Every allocation/Portal cut and both path endpoints are template cuts.
    # A midpoint is added inside each resulting base interval so no finite
    # template segment is left without a witness.
    base_cuts = {0.0, 1.0}
    for left, right, _index in allocation:
        base_cuts.add(float(left))
        base_cuts.add(float(right))
    windows = assessment.get("transition_windows") or []
    for window in windows:
        if not isinstance(window, dict):
            continue
        bounds = window.get("sigma_window")
        if bounds is not None:
            base_cuts.add(float(bounds[0]))
            base_cuts.add(float(bounds[1]))
        hint = window.get("sigma_hint")
        if hint is not None:
            base_cuts.add(float(hint))
    base_cuts = sorted(base_cuts)
    route_length = float(assessment.get("route_length_m", 0.0))
    for sigma in base_cuts:
        key = round(float(sigma), 12)
        if key in station_map:
            continue
        pose = _guide_pose_at_sigma(guide, sigma)
        station_map[key] = {
            "sigma": float(sigma),
            "s_m": float(sigma) * route_length,
            "guide_pose": {
                "x": pose["x"], "y": pose["y"],
                "heading_rad": pose["heading_rad"],
                "kappa": pose["kappa"],
                "heading_source": "TEMPLATE_CUT",
                "kappa_source": "TEMPLATE_CUT",
                "pose_reliable": True,
            },
            "vehicles": {},
            "synthetic": True,
        }
        synthetic += 1
    # Pairwise base-interval midpoints provide a witness even when the
    # assessment's own sampling is sparse.
    for left, right in zip(base_cuts, base_cuts[1:]):
        if right - left <= FORMATION_TOLERANCE_M:
            continue
        middle = 0.5 * (left + right)
        key = round(middle, 12)
        if key in station_map:
            continue
        pose = _guide_pose_at_sigma(guide, middle)
        station_map[key] = {
            "sigma": middle,
            "s_m": middle * float(assessment.get("route_length_m", 0.0)),
            "guide_pose": {
                "x": pose["x"], "y": pose["y"],
                "heading_rad": pose["heading_rad"],
                "kappa": pose["kappa"],
                "heading_source": "TEMPLATE_MIDPOINT",
                "kappa_source": "TEMPLATE_MIDPOINT",
                "pose_reliable": True,
            },
            "vehicles": {},
            "synthetic": True,
        }
        synthetic += 1
    stations = sorted(station_map.values(), key=lambda item: item["sigma"])
    if not stations or abs(stations[0]["sigma"]) > FORMATION_TOLERANCE_M \
            or abs(stations[-1]["sigma"] - 1.0) > FORMATION_TOLERANCE_M:
        raise VirtualAContractError(
            "COVER_TEMPLATE_INSUFFICIENT",
            "template stations do not span sigma [0, 1]")

    options_per_station = []
    f_options_per_station = []
    for station in stations:
        options_per_station.append(_station_options(
            station, request, chain, allocation, frames, margin, tolerance))
        f_options_per_station.append(_station_f_options(
            station, chain, allocation, margin, tolerance))

    def greedy_runs(f_choices):
        """Contiguous witness runs under one fixed F profile.

        Every run keeps a non-empty common vehicle option and one identical
        F region.  A new run starts when either would otherwise lose its
        common value; the two neighbouring stations then belong to different
        contiguous template segments.
        """
        runs = []
        current = [0]
        common = {frame.vehicle_id: set(
            options_per_station[0][frame.vehicle_id]) for frame in frames}
        f_value = int(f_choices[0])
        for station_index in range(1, len(stations)):
            candidate = {}
            feasible = True
            for frame in frames:
                values = set(common[frame.vehicle_id]) & set(
                    options_per_station[station_index][frame.vehicle_id])
                if not values:
                    feasible = False
                    break
                candidate[frame.vehicle_id] = values
            if feasible and int(f_choices[station_index]) == f_value:
                current.append(station_index)
                common = candidate
                continue
            runs.append((current, common, f_value))
            current = [station_index]
            common = {frame.vehicle_id: set(
                options_per_station[station_index][frame.vehicle_id])
                for frame in frames}
            f_value = int(f_choices[station_index])
        runs.append((current, common, f_value))
        return runs

    def segment_bounds_for(runs):
        cuts = []
        for left_run, right_run in zip(runs, runs[1:]):
            left_index = left_run[0][-1]
            right_index = right_run[0][0]
            if right_index != left_index + 1:
                raise VirtualAContractError(
                    "COVER_TEMPLATE_INSUFFICIENT",
                    "witness runs are not contiguous")
            left_sigma = float(stations[left_index]["sigma"])
            right_sigma = float(stations[right_index]["sigma"])
            left_f = int(left_run[2])
            right_f = int(right_run[2])
            if left_f == right_f:
                cut = 0.5 * (left_sigma + right_sigma)
            else:
                if abs(right_f - left_f) != 1:
                    raise VirtualAContractError(
                        "COVER_TEMPLATE_INSUFFICIENT",
                        "F region assignment would skip from {} to {} (not "
                        "directly adjacent)".format(left_f, right_f))
                cut = _f_transition_cut(
                    guide, left_sigma, right_sigma, left_f, right_f,
                    chain, margin, tolerance)
                if cut is None:
                    raise VirtualAContractError(
                        "COVER_TEMPLATE_INSUFFICIENT",
                        "no sigma between {!r} and {!r} lies in both F "
                        "regions {} and {} for the guide".format(
                            left_sigma, right_sigma, left_f, right_f))
            cuts.append(float(cut))
        bounds = []
        for run_index in range(len(runs)):
            start = 0.0 if run_index == 0 else cuts[run_index - 1]
            end = 1.0 if run_index == len(runs) - 1 else cuts[run_index]
            if end <= start:
                raise VirtualAContractError(
                    "COVER_TEMPLATE_INSUFFICIENT",
                    "degenerate witness-derived template segment")
            bounds.append((float(start), float(end)))
        return bounds

    # Three deterministic profiles share the existing three-template budget:
    #   variant 0: keep the historical allocation region;
    #   variant 1: allow the next neighbouring region before the old gate would
    #              (early migration);
    #   variant 2: keep the previous neighbouring region late (late stay).
    # They are alternatives, not a Cartesian product with the vehicle covers.
    profile_by_variant = ("allocation", "forward", "backward")
    templates = []
    seen = set()
    profile_rank = {}
    variant_layouts = []
    truncated = 0
    for variant in range(max_templates):
        policy = profile_by_variant[min(variant, len(profile_by_variant) - 1)]
        f_choices = [
            _f_choice(bases, valid, policy)
            for bases, valid in f_options_per_station]
        runs = greedy_runs(f_choices)
        bounds = segment_bounds_for(runs)
        ranked = []
        for _indices, common, _f_value in runs:
            row = []
            for frame in frames:
                choices = sorted(
                    common[frame.vehicle_id],
                    key=lambda option: (
                        option.body_piece_count, option.decomposition,
                        option.region_indices))
                if not choices:
                    raise VirtualAContractError(
                        "COVER_TEMPLATE_INSUFFICIENT",
                        "a template segment has no finite cover for vehicle "
                        "{}".format(frame.vehicle_id))
                truncated += max(0, len(choices) - 3)
                row.append(tuple(choices))
            ranked.append(tuple(row))
        segments = []
        for (start, end), run, choices in zip(bounds, runs, ranked):
            covers = []
            for frame, options in zip(frames, choices):
                covers.append(_choice_cover(
                    frame, options[min(variant, len(options) - 1)]))
            segments.append(FormationCorridorTemplateSegment(
                sigma_start=start, sigma_end=end, covers=tuple(covers),
                f_region_index=int(run[2])))
        template = FormationCorridorTemplate(
            request_sha256=request_fingerprint(request),
            grid_sha256=chain.grid_sha256,
            planning_context_sha256=context,
            model_sha256=formation_model_sha256(request),
            chain_digest=chain.digest,
            route_index=int(assessment.get("route_index", 0)),
            candidate_index=int(assessment.get("round_index", 0)),
            guide_source=str(guide.source),
            guide_sha256=formation_guide_digest(guide),
            allocation_digest=formation_allocation_digest(allocation),
            margin_m=float(margin),
            margin_source=str(margin_source),
            vehicle_frames=frames,
            segments=tuple(segments))
        validate_formation_template(
            template, request, chain, allocation=allocation, guide=guide,
            planning_context_sha256=context)
        variant_layouts.append({
            "variant": int(variant),
            "f_policy": policy,
            "segment_count": len(segments),
            "f_region_indices": [
                int(segment.f_region_index) for segment in segments],
            "cut_sigmas": [float(start) for start, _end in bounds],
            "option_counts": tuple(
                tuple(min(len(choices), 3) for choices in row)
                for row in ranked),
            "template_id": template.template_id,
        })
        if template.template_id not in seen:
            seen.add(template.template_id)
            profile_rank[template.template_id] = int(variant)
            templates.append(template)
    if not templates:
        raise VirtualAContractError(
            "COVER_TEMPLATE_INSUFFICIENT",
            "no finite whole/diagonal template could express this candidate")
    templates.sort(key=lambda template: (
        profile_rank[template.template_id],
        sum(1 if cover.decomposition == "whole" else 2
            for segment in template.segments for cover in segment.covers),
        template.template_id))
    first_layout = variant_layouts[0]
    return FormationTemplateCandidateSet(
        assessment_candidate_id=str(assessment.get("candidate_id", "")),
        templates=tuple(templates),
        cuts=tuple(float(value) for value in first_layout["cut_sigmas"])
        + (1.0,),
        per_segment_vehicle_option_counts=first_layout["option_counts"],
        truncated_option_count=int(truncated),
        station_count=len(stations),
        synthetic_station_count=int(synthetic))


def formation_template_from_payload(payload):
    """Rebuild one immutable template from its canonical ``as_dict`` payload.

    Used by the playback verifier for union-scope envelopes.  Every identity
    field is reconstructed and the recomputed ``template_id`` must equal the
    transmitted one, so a modified payload cannot pass as the sealed template.
    """
    if not isinstance(payload, MappingABC):
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template payload must be a mapping")
    expected = {
        "schema_version", "compiler_version", "certificate_version",
        "request_sha256", "grid_sha256", "planning_context_sha256",
        "model_sha256", "chain_digest", "route_index", "candidate_index",
        "guide_source", "guide_sha256", "allocation_digest", "margin_m",
        "margin_source", "vehicle_frames", "segments", "template_id",
    }
    if set(payload) != expected:
        raise VirtualAContractError(
            "INVALID_FORMATION_CORRIDOR_TEMPLATE",
            "template payload keys missing={} extra={}".format(
                sorted(expected - set(payload)),
                sorted(set(payload) - expected)))
    frames = []
    for frame in payload["vehicle_frames"]:
        if not isinstance(frame, MappingABC):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "vehicle frame must be a mapping")
        frames.append(FormationCorridorVehicleFrame(
            vehicle_id=frame["vehicle_id"],
            offset_x=frame["offset_x"],
            offset_y=frame["offset_y"],
            length_m=frame["length_m"],
            width_m=frame["width_m"]))
    segments = []
    segment_keys = {
        "sigma_start", "sigma_end", "f_region_index", "covers"}
    for segment in payload["segments"]:
        if not isinstance(segment, MappingABC):
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "template segment must be a mapping")
        if set(segment) != segment_keys:
            raise VirtualAContractError(
                "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                "template segment keys missing={} extra={}".format(
                    sorted(segment_keys - set(segment)),
                    sorted(set(segment) - segment_keys)))
        covers = []
        for cover in segment["covers"]:
            if not isinstance(cover, MappingABC):
                raise VirtualAContractError(
                    "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                    "vehicle cover must be a mapping")
            pieces = []
            for piece in cover["pieces"]:
                if not isinstance(piece, MappingABC):
                    raise VirtualAContractError(
                        "INVALID_FORMATION_CORRIDOR_TEMPLATE",
                        "cover piece must be a mapping")
                pieces.append(FormationCorridorPiece(
                    region_index=piece["region_index"],
                    local_vertices=tuple(
                        (vertex[0], vertex[1])
                        for vertex in piece["local_vertices"])))
            covers.append(FormationCorridorVehicleCover(
                vehicle_id=cover["vehicle_id"],
                decomposition=cover["decomposition"],
                pieces=tuple(pieces)))
        segments.append(FormationCorridorTemplateSegment(
            sigma_start=segment["sigma_start"],
            sigma_end=segment["sigma_end"],
            covers=tuple(covers),
            f_region_index=segment["f_region_index"]))
    template = FormationCorridorTemplate(
        request_sha256=payload["request_sha256"],
        grid_sha256=payload["grid_sha256"],
        planning_context_sha256=payload["planning_context_sha256"],
        model_sha256=payload["model_sha256"],
        chain_digest=payload["chain_digest"],
        route_index=payload["route_index"],
        candidate_index=payload["candidate_index"],
        guide_source=payload["guide_source"],
        guide_sha256=payload["guide_sha256"],
        allocation_digest=payload["allocation_digest"],
        margin_m=payload["margin_m"],
        margin_source=payload["margin_source"],
        vehicle_frames=tuple(frames),
        segments=tuple(segments),
        schema_version=payload["schema_version"],
        compiler_version=payload["compiler_version"],
        certificate_version=payload["certificate_version"])
    if payload["template_id"] != template.template_id:
        raise VirtualAContractError(
            "FORMATION_CORRIDOR_TEMPLATE_IDENTITY_MISMATCH",
            "transmitted template_id differs from the rebuilt template")
    return template


__all__ = [
    "vehicle_frames", "formation_model_sha256",
    "formation_allocation_digest", "formation_guide_digest",
    "whole_cover", "diagonal_cover", "single_region_template",
    "validate_formation_template", "FormationCorridorHalfspace",
    "FormationCorridorVertexRows", "FormationCorridorSample",
    "FormationCorridorConstraint", "build_formation_corridor_constraint",
    "FormationCoverOption", "finite_cover_options",
    "FormationTemplateCandidateSet", "build_formation_template_candidates",
    "FORMATION_SAMPLE_AUDIT_SCHEMA", "FORMATION_SAMPLE_AUDIT_CLEAR",
    "FORMATION_SAMPLE_AUDIT_VIOLATED", "FORMATION_SAMPLE_AUDIT_UNCONFIRMED",
    "audit_formation_template_samples", "formation_template_from_payload",
]