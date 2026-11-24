"""Measure the simulated vehicle footprints and their independently sampled feedback."""

from itertools import combinations
import json
from math import degrees, dist, hypot, sqrt

from fwis_control.model import wrap
from ranger_mini_gz.scene_generator import model_footprints
from ranger_mini_gz.scene_geometry import clearance, convex_distance, local_pose, normals

from .formation_route import fit_reference


class RunMetrics:
    def __init__(self, directory, scene, task):
        self.scene, self.task = scene, task
        self.formation = task == "formation"
        self.motion = (directory / "metrics" / "motion.jsonl").open("w", buffering=1)
        self.minimum_clearance = {}
        self.first_tracking_s = {}
        self.errors = {}
        self.mount_error = None
        self.pair_distance = None
        self.pair_center_distance = None
        self.twist_separation = 0.
        self.samples = 0

    def controller(self, vehicle, row):
        if row.get("phase") in ("TRACKING", "APPROACH_SWITCH") and row.get("reference_time") is not None:
            self.first_tracking_s.setdefault(vehicle, row["stamp"])
        if row.get("body_error") is not None:
            dx, dy, dyaw = row["body_error"]
            count, squares, maximum, angle_squares, angle_max, compute_s = self.errors.get(
                vehicle, (0, 0., 0., 0., 0., 0.))
            position = hypot(dx, dy)
            self.errors[vehicle] = (count + 1, squares + position**2, max(maximum, position),
                                    angle_squares + dyaw**2, max(angle_max, abs(dyaw)),
                                    compute_s + row.get("compute_s", 0.))

    def sample(self, stamp, poses, commands):
        indices = self.scene.vehicle_ids if self.formation else (1,)
        positions = [poses[index] for index in indices]
        reference = fit_reference(self.scene.offsets, positions) if self.formation else positions[0]
        row = {"stamp_s": stamp, "poses": poses, "cmd_vel": commands,
               "cargo_reference": reference if self.formation else None, "clearance": {}}
        footprints = list(model_footprints(self.scene, self.task, reference, positions))
        for name, body in footprints:
            distance, obstacle = clearance(body, self.scene.obstacles, self.scene.size)
            row["clearance"][name] = distance
            if name not in self.minimum_clearance or distance < self.minimum_clearance[name]["m"]:
                self.minimum_clearance[name] = {"m": distance, "obstacle": obstacle, "stamp_s": stamp}
        if self.formation:
            mount_error = max(dist(p[:2], local_pose(reference, r)[:2])
                              for p, r in zip(positions, self.scene.offsets, strict=True))
            self.mount_error = max(self.mount_error or 0., mount_error)
            closest = min(convex_distance(a, b, normals(a), normals(b))
                          for (_, a), (_, b) in combinations(footprints[:-1], 2))
            self.pair_distance = closest if self.pair_distance is None else min(self.pair_distance, closest)
            centers = min(dist(a[:2], b[:2]) for a, b in combinations(positions, 2))
            self.pair_center_distance = (centers if self.pair_center_distance is None
                                         else min(self.pair_center_distance, centers))
            row.update(mount_error_m=mount_error, min_vehicle_gap_m=closest, min_vehicle_center_gap_m=centers)
            if len(commands) == len(indices):
                self.twist_separation = max(self.twist_separation, *(
                    abs(a[0] - b[0]) for a, b in combinations(commands.values(), 2)))
        self.motion.write(json.dumps(row, allow_nan=False) + "\n")
        self.samples += 1
        return reference

    def summary(self, initial, final, goals, expected_reference):
        vehicles = {}
        for vehicle, goal in goals.items():
            if vehicle not in final:
                continue
            count, squares, maximum, angle_squares, angle_max, compute_s = self.errors.get(
                vehicle, (0, 0., 0., 0., 0., 0.))
            actual = final[vehicle]
            vehicles[str(vehicle)] = {
                "travel_from_spawn_m": dist(actual[:2], initial[vehicle][:2]),
                "terminal_position_error_m": dist(actual[:2], goal[:2]),
                "terminal_yaw_error_deg": abs(degrees(wrap(actual[2] - goal[2]))),
                "position_rmse_m": sqrt(squares / count) if count else None,
                "max_position_error_m": maximum if count else None,
                "yaw_rmse_deg": degrees(sqrt(angle_squares / count)) if count else None,
                "max_yaw_error_deg": degrees(angle_max) if count else None,
                "mean_control_ms": 1000 * compute_s / count if count else None,
            }
        result = {"vehicles": vehicles, "actual_samples": self.samples,
                  "min_environment_clearance": self.minimum_clearance,
                  "first_tracking_s": self.first_tracking_s}
        if not self.formation and 1 in final and expected_reference is not None:
            actual = final[1]
            result.update(requested_goal_position_error_m=dist(actual[:2], expected_reference[:2]),
                          requested_goal_yaw_error_deg=abs(degrees(wrap(actual[2] - expected_reference[2]))))
        if self.formation:
            result.update(max_mount_error_m=self.mount_error,
                          min_vehicle_footprint_gap_m=self.pair_distance,
                          min_vehicle_center_gap_m=self.pair_center_distance,
                          max_linear_twist_difference_mps=self.twist_separation)
            if len(self.first_tracking_s) == len(self.scene.vehicle_ids):
                times = self.first_tracking_s.values()
                result["start_skew_s"] = max(times) - min(times)
            if expected_reference is not None and all(i in final for i in self.scene.vehicle_ids):
                actual = fit_reference(self.scene.offsets, [final[i] for i in self.scene.vehicle_ids])
                result.update(terminal_formation_pose=actual,
                              terminal_formation_position_error_m=dist(actual[:2], expected_reference[:2]),
                              terminal_formation_yaw_error_deg=abs(degrees(wrap(actual[2] - expected_reference[2]))))
        return result
