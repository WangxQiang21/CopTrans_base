"""Continuously certify the swept blade and vehicle footprints in a blade scene."""

import math

import numpy as np
import shapely as sg


BODY_NAMES = ("blade", "robot1", "robot2")
SPACE_NAMES = ("road", "grid")
_GEOMETRY_TOLERANCE = 1e-8


def _rectangle(length, width):
    return np.array([[-length, -width], [length, -width],
                     [length, width], [-length, width]]) / 2


def _polygons(outline, centers, yaws):
    c, s = np.cos(yaws)[:, None], np.sin(yaws)[:, None]
    x = centers[:, :1] + c * outline[:, 0] - s * outline[:, 1]
    y = centers[:, 1:] + s * outline[:, 0] + c * outline[:, 1]
    return sg.polygons(np.stack((x, y), axis=-1))


def _grid_region(occupied, resolution):
    # Merge identical free runs between rows, then union the actual cell
    # rectangles. Distances are to the faces of occupied cells, not centers.
    free = ~occupied
    changes = np.diff(free, axis=1, prepend=False, append=False)
    runs = {}
    rectangles = []
    for row, line in enumerate(changes):
        current = {tuple(run) for run in np.flatnonzero(line).reshape(-1, 2)}
        for x0, x1 in runs.keys() - current:
            rectangles.append((x0, runs.pop((x0, x1)), x1, row))
        for run in current:
            runs.setdefault(run, row)
    rectangles.extend((x0, y0, x1, len(free)) for (x0, x1), y0 in runs.items())
    if not rectangles:
        raise ValueError("blade occupancy map has no free cells")
    return sg.union_all(sg.box(*(np.array(rectangles).T * resolution)))


class _FreeSpace:
    def __init__(self, polygon):
        self.polygon = polygon
        sg.prepare(self.polygon)
        edges = []
        for line in sg.get_parts(polygon.boundary):
            points = sg.get_coordinates(line)
            edges.append(np.stack((points[:-1], points[1:]), axis=1))
        self.index = sg.STRtree(sg.linestrings(np.concatenate(edges)))

    def covers(self, footprints):
        return sg.covers(self.polygon, footprints)

    def clearances(self, footprints):
        indices, distances = self.index.query_nearest(
            footprints.ravel(), return_distance=True, all_matches=False)
        ordered = np.empty(footprints.size)
        ordered[indices[0]] = distances
        return ordered.reshape(footprints.shape)


def _gate_interval(start, end, gate, error):
    first, last = 0., 1.
    for axis, name in enumerate("xy"):
        lower, upper = gate[name]
        lower += error
        upper -= error
        if lower > upper:
            return None
        delta = end[axis] - start[axis]
        if abs(delta) < 1e-12:
            if not lower <= start[axis] <= upper:
                return None
            continue
        u0, u1 = sorted(((lower - start[axis])/delta, (upper - start[axis])/delta))
        first, last = max(first, u0), min(last, u1)
        if first > last:
            return None
    return first, last


def _gate_visits(cargo, times, gates, errors):
    visited = []
    index, fraction = 0, 0.
    for gate in gates:
        while index < len(cargo) - 1:
            hit = _gate_interval(cargo[index], cargo[index + 1], gate, errors[index, 0])
            if hit is not None and hit[1] >= fraction:
                fraction = max(fraction, hit[0])
                time = None if times is None else float(
                    times[index] + fraction * (times[index + 1] - times[index]))
                position = (1 - fraction)*cargo[index, :2] + fraction*cargo[index + 1, :2]
                visited.append(dict(segment=index, fraction=float(fraction), time_s=time,
                                    xy_m=position.tolist()))
                break
            index += 1
            fraction = 0.
    return visited


class BladeCertifier:
    """Use exact blade and chassis outlines against both road and free cells."""

    def __init__(self, scene, occupied=None):
        if scene.blade_site is None or len(scene.offsets) != 2:
            raise ValueError("a blade scene with two supports is required")
        self.scene = scene
        self.cargo_outline = np.asarray(scene.cargo.outline)
        self.chassis_outline = _rectangle(*scene.vehicle_size[:2])
        self.offsets = np.asarray(scene.offsets)
        self.offset_radii = np.linalg.norm(self.offsets, axis=1)
        self.cargo_radius = float(np.max(np.linalg.norm(self.cargo_outline, axis=1)))
        self.chassis_radius = math.hypot(*scene.vehicle_size[:2]) / 2
        self.spacing = math.dist(*scene.offsets)
        if occupied is None:
            occupied = scene.blade_site.occupied(scene.size, scene.resolution)
        width, height = (round(size / scene.resolution) for size in scene.size)
        occupied = np.asarray(occupied)
        if occupied.shape != (height, width):
            raise ValueError(f"blade map must have shape {(height, width)}")
        grid_region = _grid_region(occupied != 0, scene.resolution)
        self._spaces = (_FreeSpace(scene.blade_site.road), _FreeSpace(grid_region))

    def mount_centers(self, cargo_poses):
        """The world XY coordinates of the two mounts for an array of poses."""
        cargo = np.asarray(cargo_poses, dtype=float)
        c, s = np.cos(cargo[:, 2:3]), np.sin(cargo[:, 2:3])
        dx, dy = self.offsets.T
        return np.stack((cargo[:, :1] + c*dx - s*dy, cargo[:, 1:2] + s*dx + c*dy), axis=-1)

    def footprints(self, cargo_poses, vehicle_yaws, vehicle_centers=None):
        """Return one column per body in blade, robot1, robot2 order."""
        cargo = np.asarray(cargo_poses, dtype=float)
        yaws = np.asarray(vehicle_yaws, dtype=float)
        centers = self.mount_centers(cargo) if vehicle_centers is None else np.asarray(vehicle_centers)
        return np.column_stack((
            _polygons(self.cargo_outline, cargo[:, :2], cargo[:, 2]),
            *(_polygons(self.chassis_outline, centers[:, i], yaws[:, i]) for i in range(2))))

    def check_poses(self, cargo_poses, vehicle_yaws=None, *, margin_m=0.):
        """Vectorized search filter, using chassis heading equal to blade by default."""
        cargo = np.atleast_2d(np.asarray(cargo_poses, dtype=float))
        yaws = np.repeat(cargo[:, 2:3], 2, axis=1) if vehicle_yaws is None else np.atleast_2d(vehicle_yaws)
        footprints = self.footprints(cargo, yaws)
        valid = np.logical_and.reduce([space.covers(footprints) for space in self._spaces])
        if margin_m > 0:
            for space in self._spaces:
                valid &= space.clearances(footprints) >= margin_m
        return np.all(valid, axis=1)

    def check_pose(self, cargo_pose, vehicle_yaws=None, *, margin_m=0.):
        return bool(self.check_poses(cargo_pose, vehicle_yaws, margin_m=margin_m)[0])

    def _geometry(self, cargo, yaws, centers):
        footprints = self.footprints(cargo, yaws, centers)
        covered = np.stack([space.covers(footprints) for space in self._spaces], axis=1)
        gaps = np.stack([space.clearances(footprints) for space in self._spaces], axis=1)
        return covered, np.where(covered, gaps, 0.)

    def _sweep_bounds(self, cargo, yaws, centers, error):
        translation = np.linalg.norm(np.diff(cargo[:, :2], axis=0), axis=1)
        rotation = np.abs(np.diff(cargo[:, 2]))
        chassis_rotation = np.abs(np.diff(yaws, axis=0))
        chassis_translation = translation[:, None] + rotation[:, None]*self.offset_radii
        chassis_error = error[:, :1] + error[:, 1:2]*self.offset_radii
        if centers is not None:
            chassis_translation = np.linalg.norm(np.diff(centers, axis=0), axis=2)
            chassis_error = 0.
        chord_bound = np.column_stack((
            translation + rotation*self.cargo_radius,
            chassis_translation + chassis_rotation*self.chassis_radius))
        remainder = np.column_stack((
            error[:, 0] + error[:, 1]*self.cargo_radius,
            chassis_error + error[:, 2:]*self.chassis_radius))
        return chord_bound, remainder

    def _mount_metrics(self, cargo, centers, error):
        metrics = dict(nominal_mount_spacing_m=self.spacing, max_mount_error_m=0.,
                       max_continuous_mount_error_bound_m=0., max_mount_spacing_error_m=0.)
        if centers is None:
            return metrics
        discrepancy = np.linalg.norm(centers - self.mount_centers(cargo), axis=2)
        dyaw = np.diff(cargo[:, 2:3], axis=0)
        # Chord error for the rigid offset uses the second derivative bound.
        offset_remainder = (dyaw**2 / 8 + error[:, 1:2]) * self.offset_radii
        between = np.maximum(discrepancy[:-1], discrepancy[1:]) + offset_remainder + error[:, :1]
        separation = centers[:, 1] - centers[:, 0]
        change = np.diff(separation, axis=0)
        square = np.einsum("ij,ij->i", change, change)
        station = np.divide(-np.einsum("ij,ij->i", separation[:-1], change), square,
                            out=np.zeros_like(square), where=square > 0.)
        closest = separation[:-1] + np.clip(station, 0., 1.)[:, None]*change
        sampled_lengths = np.linalg.norm(separation, axis=1)
        minimum = float(np.min(np.linalg.norm(closest, axis=1)))
        maximum = float(np.max(sampled_lengths))
        return dict(metrics, max_mount_error_m=float(np.max(discrepancy)),
                    max_continuous_mount_error_bound_m=float(np.max(between)),
                    max_mount_spacing_error_m=max(abs(self.spacing - minimum), abs(maximum - self.spacing)))

    def certify(self, cargo_poses, vehicle_yaws, times=None, vehicle_centers=None, *,
                interpolation_error=None, clearance_margin_m=0., mount_tolerance_m=1e-6,
                endpoint_position_tolerance_m=1e-6, endpoint_yaw_tolerance_rad=1e-6,
                check_endpoints=True):
        """Certify continuous interpolation between supplied trajectory samples.

        Angles are unwrapped. Without bounds, adjacent samples define linear
        cargo XY, cargo yaw and chassis yaw. Chassis centers follow the exact
        rigid transform or the measured center chords if supplied.

        interpolation_error bounds a smooth trajectory's deviation from that
        interpolation on each interval, in columns [XY, yaw, yaw1, yaw2].
        For example, use the norm bound on the second derivative times dt²/8.
        With measured centers the sweep is bounded for their interpolation.
        """
        cargo = np.asarray(cargo_poses, dtype=float)
        yaws = np.asarray(vehicle_yaws, dtype=float)
        n = len(cargo) if cargo.ndim else 0
        times = None if times is None else np.asarray(times, dtype=float)
        centers = None if vehicle_centers is None else np.asarray(vehicle_centers, dtype=float)
        errors = (np.zeros((max(0, n - 1), 4)) if interpolation_error is None
                  else np.asarray(interpolation_error, dtype=float))
        tolerances = (clearance_margin_m, mount_tolerance_m, endpoint_position_tolerance_m,
                      endpoint_yaw_tolerance_rad)
        if (n < 2 or cargo.shape != (n, 3) or yaws.shape != (n, 2) or errors.shape != (n - 1, 4)
                or (times is not None and (times.shape != (n,) or np.any(np.diff(times) <= 0.)))
                or (centers is not None and centers.shape != (n, 2, 2))
                or not all(np.isfinite(a).all() for a in (cargo, yaws, errors, times, centers) if a is not None)
                or not all(math.isfinite(x) and x >= 0. for x in tolerances)
                or np.any(errors < 0.)):
            raise ValueError("invalid blade samples, times, interpolation bounds or tolerances")

        exp = self.scene.experiments["blade_pair"]
        visits = _gate_visits(cargo, times, exp["ordered_gates"], errors)
        report = dict(passed=False, reason=None, samples=n, gate_visits=visits,
                      gates_required=len(exp["ordered_gates"]), clearance_margin_m=clearance_margin_m,
                      mount_tolerance_m=mount_tolerance_m, **self._mount_metrics(cargo, centers, errors))
        covered, gaps = self._geometry(cargo, yaws, centers)
        report["min_road_clearance_m"], report["min_grid_clearance_m"] = map(float, gaps.min(axis=(0, 2)))
        margin = clearance_margin_m + _GEOMETRY_TOLERANCE
        invalid = np.argwhere(~covered | (gaps < margin))
        if len(invalid):
            sample, space, body = map(int, invalid[0])
            return dict(report, reason=f"{SPACE_NAMES[space]}_sample_clearance",
                        first_failure=dict(sample=sample, body=BODY_NAMES[body],
                                           cargo_pose=cargo[sample].tolist(),
                                           clearance_m=float(gaps[sample, space, body])))

        if report["max_continuous_mount_error_bound_m"] > mount_tolerance_m:
            return dict(report, reason="mount_alignment")
        if report["max_mount_spacing_error_m"] > 2*mount_tolerance_m:
            return dict(report, reason="mount_spacing")
        if check_endpoints:
            for index, name in ((0, "start"), (-1, "goal")):
                target = exp[name]
                position = math.dist(cargo[index, :2], target[:2])
                yaw = abs(math.remainder(cargo[index, 2] - target[2], math.tau))
                if position > endpoint_position_tolerance_m or yaw > endpoint_yaw_tolerance_rad:
                    return dict(report, reason=f"{name}_pose", first_failure=dict(
                        position_error_m=position, yaw_error_rad=yaw, cargo_pose=cargo[index].tolist()))
        if len(visits) != len(exp["ordered_gates"]):
            return dict(report, reason="ordered_gates")

        chord_bound, remainder = self._sweep_bounds(cargo, yaws, centers, errors)
        certified, failure = self._certify_segments(cargo, yaws, centers, gaps, chord_bound, remainder, margin)
        if failure is not None:
            return dict(report, reason="swept_clearance_not_certified", first_failure=failure)
        return dict(report, passed=True, reason="certified",
                    swept_road_clearance_lower_bound_m=float(certified[0]),
                    swept_grid_clearance_lower_bound_m=float(certified[1]))

    def _certify_segments(self, cargo, yaws, centers, gaps, bounds, remainder, margin):
        indices = np.arange(len(bounds))
        fractions = np.tile((0., 1.), (len(bounds), 1))
        gap_start, gap_end = gaps[:-1], gaps[1:]
        best = np.full(2, np.inf)
        for depth in range(13):
            width = (fractions[:, 1] - fractions[:, 0])[:, None]
            travel = .5 * width*bounds[indices] + remainder[indices]
            lower = np.minimum(gap_start, gap_end) - travel[:, None, :]
            fine = np.all(lower >= margin, axis=(1, 2))
            if np.any(fine):
                best = np.minimum(best, lower[fine].min(axis=(0, 2)))
            if np.all(fine):
                return best, None
            indices, fractions = indices[~fine], fractions[~fine]
            gap_start, gap_end = gap_start[~fine], gap_end[~fine]
            midpoint = fractions.mean(axis=1)

            def interpolate_mid(data):
                weight = midpoint.reshape((-1,) + (1,) * (data.ndim - 1))
                return (1. - weight)*data[indices] + weight*data[indices + 1]

            mid_cargo = interpolate_mid(cargo)
            mid_yaws = interpolate_mid(yaws)
            mid_centers = None if centers is None else interpolate_mid(centers)
            _, mid_gaps = self._geometry(mid_cargo, mid_yaws, mid_centers)
            bad = np.argwhere(mid_gaps < (margin + remainder[indices, None, :]))
            if len(bad) or depth == 12:
                row, space, body = map(int, bad[0] if len(bad) else
                                       np.unravel_index(np.argmin(lower[~fine]), lower[~fine].shape))
                return best, dict(segment=int(indices[row]), fraction=float(midpoint[row]),
                                  cargo_pose=mid_cargo[row].tolist(), space=SPACE_NAMES[space],
                                  body=BODY_NAMES[body], midpoint_clearance_m=float(mid_gaps[row, space, body]))
            gap_start = np.concatenate((gap_start, mid_gaps))
            gap_end = np.concatenate((mid_gaps, gap_end))
            fractions = np.concatenate((np.column_stack((fractions[:, 0], midpoint)),
                                        np.column_stack((midpoint, fractions[:, 1]))))
            indices = np.tile(indices, 2)
        raise AssertionError("sweep refinement termination")
