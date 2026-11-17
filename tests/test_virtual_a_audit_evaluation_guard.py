"""C0-B: derived audit evaluation guard and the frozen U-turn candidate.

The C0-A run rejected every U-turn candidate with
``AUDIT_INTEGRATION_REMAINDER`` at ``9.95e-8..1.02e-7`` against the fixed
``5e-8`` audit threshold.  C0-B froze the same ``kappa/eta/S`` candidates,
decomposed the Simpson remainder, the reused G1 ``position_guard`` and the
angle bound, and found the guard to be the wrong term: ``1e-10 m`` per
Simpson panel made the audit budget grow with every refinement while the
measured float64 error of the audit's own Simpson sums stayed at ``1e-15 m``.
The audit now uses :func:`integration_bounds.evaluation_guard`, derived in
that function's docstring, and combines the per-coordinate evaluation and
angle terms with ``sqrt(2)`` into the Euclidean budget.  The fixed ``5e-8``
threshold, the production ``1e-7`` path budget and
``bounds.POSITION_ROUNDOFF_GUARD_M`` are unchanged.

Evidence: ``evaluation/virtual_a_corridor/C0-B/20261117T052817Z/``
(``candidates.json``, ``budget_analysis.json``, ``SUMMARY.md``).
"""
import math
import unittest

import numpy as np

from fwis_v3_optimizer.continuous_geometry import ControlKnot
from fwis_v3_optimizer.formation import FormationPose
from fwis_v3_optimizer.formation_continuous_a import audit
from fwis_v3_optimizer.formation_continuous_a import bounds
from fwis_v3_optimizer.formation_continuous_a import integration_bounds
from fwis_v3_optimizer.formation_continuous_a.contracts import (
    GeometryAuditConfig,
)

#: Capture 0 of ``candidates.json``: the first audited U-turn candidate of the
#: single C0-B targeted reproduction.  S = 15.163479941796252 m, 8 pieces.
FROZEN_UTURN_KNOTS = (
    (0.0, 0.0, 0.0),
    (1.8954349927245315, -0.3409110903299332, -0.24163458812033445),
    (3.790869985449063, -0.49147277425519387, 0.18865975024754295),
    (5.686304978173594, 0.06211521545210576, 0.1248855290100294),
    (7.581739970898126, -0.23484778051068608, -0.2820286215239922),
    (9.477174963622657, -0.5315389280076455, 0.06362277844339619),
    (11.372609956347189, -0.1136195658727009, 0.21941148143451084),
    (13.26804494907172, 0.057049956978602, -0.009679564096714994),
    (15.163479941796252, -0.21494819059223627, -0.34107112000338946),
)
FROZEN_UTURN_START = FormationPose(
    12.57, 10.32, -0.029733900000000004)
LD = np.longdouble


def _pieces():
    knots = [ControlKnot(*row) for row in FROZEN_UTURN_KNOTS]
    pieces, _stations, _values, _derivatives = audit._independent_pieces(
        knots)
    return pieces


def _plan(config=None):
    pieces = _pieces()
    start = FROZEN_UTURN_START
    return pieces, audit._integration_plan(
        pieces, start.psi, config or GeometryAuditConfig(),
        start_magnitude_m=max(abs(start.x), abs(start.y)))


def _psi_longdouble(piece, psi0, q):
    """The piece polynomial and psi increment in extended precision."""
    h = LD(piece.h)
    c0 = LD(piece.c0)
    c1 = LD(piece.c1)
    c2 = LD(piece.c2)
    c3 = LD(piece.c3)
    return LD(psi0) + h * q * (
        c0 + q * (c1 / LD(2) + q * (c2 / LD(3) + q * c3 / LD(4))))


def _piece_integral_longdouble(piece, psi0, qa, qb, subintervals):
    """Extended-precision composite Simpson on the same nodes and count."""
    count = int(subintervals)
    q = np.linspace(LD(qa), LD(qb), count + 1)
    psi = _psi_longdouble(piece, psi0, q)
    weights = np.full(count + 1, LD(2), dtype=LD)
    weights[0] = LD(1)
    weights[-1] = LD(1)
    weights[1:-1:2] = LD(4)
    cosine = np.sum(weights * np.cos(psi))
    sine = np.sum(weights * np.sin(psi))
    factor = LD(piece.h) * (LD(qb) - LD(qa)) / (LD(3) * LD(count))
    return factor * cosine, factor * sine


def _position_longdouble_same_panels(pieces, start, station, subintervals):
    """Mirror ``audit._integrate_xy`` in extended precision.

    The same subinterval counts, the same complete-panel/local-panel split and
    the same branch decisions are used, so the difference from the audit is
    the float64 evaluation error alone, not Simpson truncation.
    """
    if station <= 0.0:
        return LD(start.x), LD(start.y)
    index, q_end = audit._locate(pieces, station)
    prefixes = [LD(0)]
    for piece in pieces:
        prefixes.append(
            prefixes[-1] + _psi_longdouble(piece, LD(0), LD(1)))
    x = LD(start.x)
    y = LD(start.y)
    for piece_index in range(index + 1):
        piece = pieces[piece_index]
        psi0 = LD(start.psi) + prefixes[piece_index]
        if piece_index < index:
            dx, dy = _piece_integral_longdouble(
                piece, psi0, LD(0), LD(1), subintervals[piece_index])
        else:
            count = subintervals[piece_index]
            u = min(max(q_end, 0.0), 1.0)
            complete = min(count, max(0, int(math.floor(u * count))))
            if complete % 2:
                complete -= 1
            if complete / count > u:
                complete = max(0, complete - 2)
            if complete >= 2:
                dx, dy = _piece_integral_longdouble(
                    piece, psi0, LD(0), LD(complete / count), complete)
            else:
                dx, dy = LD(0), LD(0)
            if u > complete / count:
                local_x, local_y = _piece_integral_longdouble(
                    piece, psi0, LD(complete / count), LD(u), 2)
                dx += local_x
                dy += local_y
        x += dx
        y += dy
    return x, y


def _stations(pieces):
    stations = []
    for piece in pieces:
        for fraction in (0.13, 0.37, 0.5, 0.71, 0.89, 1.0):
            station = piece.s0 + piece.h * fraction
            if station > 0.0 and (
                    not stations or station != stations[-1]):
                stations.append(station)
    return stations


class AuditEvaluationGuardTests(unittest.TestCase):

    def test_frozen_candidate_confirms_5e8_with_the_derived_guard(self):
        pieces, plan = _plan()
        budget = plan["budget"]
        self.assertTrue(budget["confirmed"])
        self.assertLessEqual(budget["total_budget_m"], 5e-8)
        self.assertEqual(budget["fixed_threshold_m"], 5e-8)
        self.assertEqual(budget["refinement_steps"], 0)
        self.assertGreaterEqual(
            budget["total_budget_m"],
            budget["euclidean_remainder_m"]
            + budget["roundoff_and_angle_guard_m"])
        # The reused G1 guard rejected exactly this candidate in C0-A.
        old_guard = bounds.position_guard(
            pieces[-1].s1,
            budget["full_simpson_panels"] + budget["local_simpson_panels"],
            budget["angle_bound_rad"])
        old_total = bounds._next_up(
            budget["euclidean_remainder_m"] + old_guard)
        self.assertGreater(old_total, 5e-8)
        self.assertLess(old_guard, 4.3e-8)
        self.assertGreater(old_guard, 4.0e-8)
        # The derived evaluation and angle terms are per-coordinate; the
        # Euclidean guard is their sqrt(2) combination.
        per_coordinate = bounds._next_up(
            budget["evaluation_guard_m"] + budget["angle_guard_m"])
        self.assertEqual(
            budget["per_coordinate_roundoff_and_angle_guard_m"],
            per_coordinate)
        self.assertEqual(
            budget["roundoff_and_angle_guard_m"],
            bounds._next_up(bounds._next_up(math.sqrt(2.0))
                            * per_coordinate))
        self.assertLess(budget["evaluation_guard_m"], 1e-10)

    def test_guard_bounds_measured_float64_error_at_the_same_panels(self):
        pieces, plan = _plan()
        budget = plan["budget"]
        subintervals = plan["subintervals"]
        prefixes = audit._psi_prefixes(pieces)
        guard = budget["evaluation_guard_m"]
        worst_axis = 0.0
        worst_euclidean = 0.0
        for station in _stations(pieces):
            x, y = audit._integrate_xy(
                pieces, prefixes, FROZEN_UTURN_START, station, subintervals)
            ref_x, ref_y = _position_longdouble_same_panels(
                pieces, FROZEN_UTURN_START, station, subintervals)
            error_x = abs(x - float(ref_x))
            error_y = abs(y - float(ref_y))
            worst_axis = max(worst_axis, error_x, error_y)
            worst_euclidean = max(
                worst_euclidean, math.hypot(error_x, error_y))
        # The guard is a per-coordinate bound; the Euclidean error is bounded
        # by sqrt(2) times it.
        self.assertLessEqual(worst_axis, guard)
        self.assertLessEqual(
            worst_euclidean, bounds._next_up(math.sqrt(2.0)) * guard)
        # A real float64 error, not a bound measured against itself.
        self.assertGreater(worst_axis, 0.0)

    def test_guard_grows_with_the_panel_count(self):
        pieces, plan = _plan()
        record = [
            (piece.h, count,
             abs(piece.c0) + abs(piece.c1) + abs(piece.c2) + abs(piece.c3))
            for piece, count in zip(pieces, plan["subintervals"])]
        doubled = [
            (h, 2 * count, magnitude)
            for h, count, magnitude in record]
        first = integration_bounds.evaluation_guard(
            record, FROZEN_UTURN_START.psi,
            max(abs(FROZEN_UTURN_START.x), abs(FROZEN_UTURN_START.y)))
        second = integration_bounds.evaluation_guard(
            doubled, FROZEN_UTURN_START.psi,
            max(abs(FROZEN_UTURN_START.x), abs(FROZEN_UTURN_START.y)))
        self.assertGreater(
            second["total_evaluation_m"], first["total_evaluation_m"])

    def test_infeasible_threshold_still_fails_closed(self):
        pieces = _pieces()
        config = GeometryAuditConfig(audit_position_budget_m=1e-12)
        with self.assertRaises(audit._AuditFailure) as raised:
            audit._integration_plan(
                pieces, FROZEN_UTURN_START.psi, config,
                start_magnitude_m=max(
                    abs(FROZEN_UTURN_START.x),
                    abs(FROZEN_UTURN_START.y)))
        self.assertEqual(raised.exception.reason, "BOUND_UNCONFIRMED")
        constraint = str(raised.exception.location.get("constraint", ""))
        self.assertTrue(constraint.startswith("AUDIT_INTEGRATION"),
                        constraint)


if __name__ == "__main__":
    unittest.main()