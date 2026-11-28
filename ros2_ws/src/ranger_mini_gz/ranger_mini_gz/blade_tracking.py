"""Track dual-mode plans using Gazebo pose feedback and rigid mounts."""

import argparse
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from threading import Condition
from time import monotonic

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node
import numpy as np
import shapely as sg
import yaml

from ranger_mini_gz.blade_geometry import local_footprint, world_footprint
from ranger_mini_gz.blade_system import (
    CARGO, FEEDBACK_MODES_KEY, MODE_KEY, PAIR_MODES, VEHICLES, stamp_ns, turn, write_stamp,
)
from ranger_mini_gz.body_sim import ACKERMANN, OBLIQUE, SPIN, PlantSettings
from ranger_mini_gz.scene_generator import installed_paths, load_scene, with_trial_road


TERMINAL_POSITION = .06
TERMINAL_YAW = .06
MOUNT_LIMIT = .015
TRACK_LIMIT = .30
MIN_ROAD_CLEARANCE = .02
STOP_WINDOW = .55
MIN_MODE_DWELL = .70
MOTION_LABELS = {ACKERMANN: "Ackermann", OBLIQUE: "Oblique", SPIN: "Spin"}
MODE_BY_NAME = {str(mode): mode for mode in PAIR_MODES}


def wrap(angle):
    return np.arctan2(np.sin(angle), np.cos(angle))


def relative_positions(a, b):
    return np.linalg.norm(a[..., :2] - b[..., :2], axis=-1)


@dataclass(frozen=True)
class Plan:
    time: np.ndarray
    poses: np.ndarray
    feedforward: np.ndarray
    motion_mode: np.ndarray | None = None
    switches: np.ndarray = field(init=False)
    releases: np.ndarray = field(init=False)

    def __post_init__(self):
        if self.motion_mode is None:
            object.__setattr__(self, "motion_mode", np.full(len(self.time), ACKERMANN, dtype=np.int8))
        if self.feedforward.shape[-1] == 2:
            values = np.insert(self.feedforward, 1, 0., axis=-1)
            object.__setattr__(self, "feedforward", values)
        indices = np.flatnonzero(np.r_[self.motion_mode[0] != ACKERMANN, np.diff(self.motion_mode) != 0])
        switches = self.time[indices]
        releases = []
        ends = np.r_[indices[1:], len(self.time)] if len(indices) else indices
        for begin, end in zip(indices, ends, strict=True):
            moving = (np.any(np.abs(self.feedforward[begin:end]) > 1e-8, axis=(1, 2)) |
                      np.any(np.abs(self.poses[begin:end]-self.poses[begin]) > 1e-8, axis=(1, 2)))
            moving_idx = np.flatnonzero(moving)
            stationary_end = begin + int(moving_idx[0]) - 1 if len(moving_idx) else end-1
            releases.append(max(self.time[begin]+MIN_MODE_DWELL, self.time[max(begin, stationary_end)]))
        object.__setattr__(self, "switches", switches)
        object.__setattr__(self, "releases", np.array(releases))

    @classmethod
    def read(cls, path):
        with np.load(path, allow_pickle=False) as data:
            time = np.asarray(data["time"], dtype=float)
            vehicles = np.asarray(data["vehicles"], dtype=float)
            cargo = np.asarray(data["cargo"], dtype=float)
            feedforward = np.asarray(data["feedforward"], dtype=float)
            modes = (data["motion_mode"] if "motion_mode" in data else
                     np.full(time.size, ACKERMANN, dtype=np.int8))
        n = time.size
        if (time.shape != (n,) or n < 2 or vehicles.shape != (n, 2, 3) or cargo.shape != (n, 3)
                or feedforward.shape not in ((n, 2, 2), (n, 2, 3))
                or modes.shape != (n,) or not np.isin(modes, PAIR_MODES).all()
                or not all(np.isfinite(a).all() for a in (time, cargo, vehicles, feedforward))):
            raise ValueError("invalid blade trajectory array dimensions or values")
        if abs(time[0]) > 1e-8 or np.any(np.diff(time) <= 0.) or np.max(np.abs(feedforward[[0, -1]])) > 1e-8:
            raise ValueError("blade tracker requires increasing sample times and stopped endpoints")
        if feedforward.shape[-1] == 2 and np.any(modes != ACKERMANN):
            raise ValueError("Non-Ackermann references require three-component body twists")
        poses = np.concatenate((cargo[:, None, :], vehicles), axis=1)
        poses[:, :, 2] = np.unwrap(poses[:, :, 2], axis=0)
        plan = cls(time, poses, feedforward, modes.astype(np.int8))
        plan.validate_motion()
        return plan

    def validate_motion(self):
        ackermann = self.feedforward[self.motion_mode == ACKERMANN]
        oblique = self.feedforward[self.motion_mode == OBLIQUE]
        spin = self.feedforward[self.motion_mode == SPIN]
        if np.any(ackermann[:, :, 0] < -1e-8) or np.any(np.abs(ackermann[:, :, 1]) > 1e-8):
            raise ValueError("Ackermann references require forward speed and zero lateral velocity")
        if np.any(np.abs(oblique[:, :, 2]) > 1e-8):
            raise ValueError("Oblique references require zero yaw rate")
        if np.any(np.abs(spin[:, :, :2]) > 1e-8):
            raise ValueError("Spin references require zero linear velocity")
        same_spin_step = (self.motion_mode[1:] == SPIN) & (self.motion_mode[:-1] == SPIN)
        spin_delta = np.diff(self.poses, axis=0)[same_spin_step]
        if np.any(np.abs(spin_delta[:, :, :2]) > 1e-8) or np.any(np.abs(spin_delta[:, 0, 2]) > 1e-8):
            raise ValueError("Spin references require stationary cargo and mount centers")
        same_oblique_step = (self.motion_mode[1:] == OBLIQUE) & (self.motion_mode[:-1] == OBLIQUE)
        yaw_delta = np.diff(self.poses[:, :, 2], axis=0)[same_oblique_step]
        if np.any(np.abs(yaw_delta) > 1e-8):
            raise ValueError("Oblique references require constant cargo and chassis headings")
        if np.any(np.diff(self.switches) < MIN_MODE_DWELL-1e-8):
            raise ValueError("consecutive mode changes overlap their stationary dwells")
        for start in self.switches:
            end = start + MIN_MODE_DWELL
            if end > self.duration + 1e-8:
                raise ValueError("mode change has no complete stationary dwell")
            mask = (self.time >= start) & (self.time <= end+1e-8)
            stopped = self.poses[mask][0]
            if (np.any(np.abs(self.feedforward[mask]) > 1e-8)
                    or np.any(np.abs(self.poses[mask] - stopped) > 1e-8)
                    or np.any(np.abs(self.interpolate(self.poses, end) - stopped) > 1e-8)):
                raise ValueError("mode change requires a 0.70 s stationary dwell at zero Twist")

    @property
    def duration(self):
        return float(self.time[-1])

    def interpolate(self, values, t):
        index = int(np.clip(np.searchsorted(self.time, t, side="right"), 1, len(self.time)-1))
        before, after = self.time[index-1:index+1]
        fraction = min(1., max(0., (t-before)/(after-before)))
        return values[index-1] + fraction*(values[index]-values[index-1])

    def phase(self, t):
        sample = max(0, np.searchsorted(self.time, t+1e-9, side="right")-1)
        next_switch = np.searchsorted(self.switches, t+1e-9, side="right")
        dwell = next_switch > 0 and t < self.releases[next_switch-1] - 1e-9
        end = self.switches[next_switch] if next_switch < len(self.switches) else self.duration
        return int(self.motion_mode[sample]), dwell, end


def controls(plan, t, observed, limits):
    mode, dwell, phase_end = plan.phase(t)
    if dwell:
        return np.zeros((2, 3))
    reference = plan.interpolate(plan.poses, t)
    advance = min(limits.linear_tau, limits.angular_tau)
    feedforward = plan.interpolate(plan.feedforward, min(t+advance, phase_end))
    if mode == SPIN:
        yaw_error = wrap(reference[1:, 2]-observed[1:, 2])
        yaw_rates = np.clip(feedforward[:, 2] + 2.*yaw_error, -limits.max_yaw_rate, limits.max_yaw_rate)
        yaw_rates[np.abs(yaw_rates) < 1e-6] = 0.
        return np.column_stack((np.zeros((2, 2)), yaw_rates))
    steering_tangent = math.tan(limits.high_steer_angle)
    if mode == OBLIQUE:
        world = np.mean([turn(*twist[:2], pose[2])
                         for twist, pose in zip(feedforward, reference[1:], strict=True)], axis=0)
        world += 1.35*(reference[0, :2] - observed[0, :2])
        speed = float(np.linalg.norm(world))
        if speed <= 1e-12:
            return np.zeros((2, 3))
        body = [turn(*world, -p[2]) for p in observed[1:]]
        cap = limits.max_speed
        if limits.high_steer_speed > 0. and any(abs(y) > steering_tangent*abs(x) for x, y in body):
            cap = min(cap, limits.high_steer_speed)
        scale = min(1., cap/speed)
        # Ranger's Twist classifier assigns any nonzero vy to oblique.
        # Keep its common world direction even at a longitudinal zero crossing.
        return np.array([(scale*x, scale*y if y else 1e-12, 0.) for x, y in body])
    high_steer_curvature = 2*steering_tangent / (limits.wheelbase + limits.track*steering_tangent)
    guarded_radius = max(.83, limits.switch_radius+.03)
    command = []
    for car, desired, (v, _, omega) in zip(observed[1:], reference[1:], feedforward, strict=True):
        ex, ey = turn(*(desired[:2]-car[:2]), -car[2])
        error = float(wrap(desired[2] - car[2]))
        # Same body-frame nonlinear Ackermann feedback law used by the single tracker.
        speed = min(limits.max_speed, max(0., v*math.cos(error) + 1.35*ex))
        yaw = omega + 9.*v*ey + 6.*v*math.sin(error)
        cap = min(limits.max_yaw_rate, speed/guarded_radius)
        yaw = min(cap, max(-cap, yaw))
        if limits.high_steer_speed > 0. and abs(yaw) > high_steer_curvature*speed:
            factor = min(1., limits.high_steer_speed/speed)
            speed, yaw = speed*factor, yaw*factor
        command.append((speed, 0., yaw))
    return np.array(command)


class Feedback:
    def __init__(self, scene):
        self.cv = Condition()
        self.last = None
        self.node = Node()
        self.tx = [self.node.advertise(f"/{name}/cmd_vel", Twist) for name in VEHICLES]
        self.reference = self.node.advertise(f"/world/{scene}/blade_pair/tracking_reference", Pose_V)
        self.node.subscribe(Pose_V, f"/world/{scene}/blade_pair/state", self.receive)

    def receive(self, msg):
        by_name = {p.name: p for p in msg.pose}
        names = (CARGO, *VEHICLES)
        if not all(name in by_name for name in names):
            return
        poses = []
        for name in names:
            p, q = by_name[name].position, by_name[name].orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            poses.append((p.x, p.y, yaw))
        values = {d.key: d.value for d in msg.header.data if d.value}
        raw_modes = values.get(FEEDBACK_MODES_KEY, ("0", "0"))
        modes = tuple(MODE_BY_NAME.get(m, -1) for m in raw_modes)
        fault = values.get("fault", ("",))[0]
        if len(modes) != len(VEHICLES) or not all(mode in PAIR_MODES for mode in modes):
            fault = "invalid Gazebo mode feedback"
        stamp = stamp_ns(msg.header)
        with self.cv:
            if self.last is None or stamp > self.last[0]:
                self.last = (stamp, np.asarray(poses), float(values["constraint_residual_mps"][0]), fault, modes)
                self.cv.notify()

    def next(self, last, wait):
        with self.cv:
            if not self.cv.wait_for(lambda: self.last is not None and self.last[0] > last, timeout=wait):
                raise RuntimeError("Gazebo pose feedback timeout")
            return self.last

    def send(self, stamp, commands, mode):
        for publisher, (x, y, yaw) in zip(self.tx, commands, strict=True):
            message = Twist()
            write_stamp(message.header, stamp)
            message.header.data.add(key=MODE_KEY, value=[str(mode)])
            message.linear.x, message.linear.y, message.angular.z = x, y, yaw
            publisher.publish(message)

    def publish_reference(self, stamp, trajectory_stamp, poses, observed, phase):
        if not self.reference.has_connections():
            return
        message = Pose_V()
        write_stamp(message.header, stamp)
        message.header.data.add(key="trajectory_stamp_ns", value=[str(trajectory_stamp)])
        message.header.data.add(key="phase", value=[phase])
        errors = []
        for desired, actual in zip(poses, observed, strict=True):
            ex, ey = turn(*(desired[:2]-actual[:2]), -actual[2])
            errors.extend((str(ex), str(ey), str(float(wrap(desired[2]-actual[2])))))
        message.header.data.add(key="body_error", value=errors)
        for name, (x, y, yaw) in zip((CARGO, *VEHICLES), poses, strict=True):
            item = message.pose.add(name=name)
            item.position.x, item.position.y = x, y
            item.orientation.z, item.orientation.w = math.sin(yaw*.5), math.cos(yaw*.5)
        self.reference.publish(message)


def mount_error(observed, offsets):
    cargo = observed[0]
    expected = np.array([turn(*offset, cargo[2]) for offset in offsets]) + cargo[:2]
    return np.linalg.norm(observed[1:, :2]-expected, axis=1)


def record(metrics_path, status, reason, plan, wall_time, samples, confirmations):
    output = dict(status=status, passed=status == "passed", reason=reason, planned_duration_s=plan.duration, wall_s=wall_time,
                  samples=len(samples), control="paired Ackermann, Oblique and Spin Twist tracking",
                  planned_mode_switches=len(plan.switches), mode_confirmations=confirmations,
                  plant="Gazebo body-level motion with ideal pin constraint at the two observed vehicle centers")
    if samples:
        time, poses, desired, cmd, mounts, residuals, gaps, requested, measured_modes = zip(*samples, strict=True)
        time, poses, desired, cmd, measured_modes = map(np.asarray, (time, poses, desired, cmd, measured_modes))
        position_error = relative_positions(poses, desired)
        yaw_error = np.abs(wrap(poses[:, :, 2] - desired[:, :, 2]))
        tracked = position_error.max(axis=0)
        terminal_pos = relative_positions(poses[-1], plan.poses[-1])
        terminal_yaw = np.abs(wrap(poses[-1, :, 2] - plan.poses[-1, :, 2]))
        speeds = relative_positions(poses[-1], poses[-2])/(time[-1]-time[-2]) if len(time) > 1 else np.zeros(3)
        mode_changes = np.r_[[[True, True]], np.diff(measured_modes, axis=0) != 0]
        output.update(transport_duration_s=float(time[-1]), max_feedback_gap_s=float(max(np.diff(time), default=0.)),
                      maximum_cargo_track_error_m=float(tracked[0]), maximum_vehicle_track_error_m=tracked[1:].tolist(),
                      maximum_heading_error_rad=yaw_error.max(axis=0).tolist(), final_error_m=terminal_pos.tolist(),
                      final_heading_error_rad=terminal_yaw.tolist(), final_speed_mps=speeds.tolist(),
                      maximum_mount_error_m=float(np.max(mounts)), maximum_constraint_residual_mps=float(max(residuals)),
                      minimum_observed_clearance_m=float(min(gaps)), feedback_order=[CARGO, *VEHICLES],
                      mode_switches_by_car=(mode_changes.sum(axis=0)-1).tolist(),
                      mode_sequence_by_car=[[MOTION_LABELS[mode] for mode in modes[changes]]
                                            for modes, changes in zip(measured_modes.T, mode_changes.T, strict=True)])
        metrics_path = Path(metrics_path)
        np.savez_compressed(metrics_path.with_suffix(".npz"), time=time, cargo=poses[:, 0], vehicles=poses[:, 1:],
                            command=cmd, reference=desired, mount_error=mounts, clearance=gaps,
                            constraint_residual=residuals, requested_motion_mode=requested, motion_mode=measured_modes)
    Path(metrics_path).write_text(json.dumps(output, indent=2) + "\n")
    return output


def track(plan, scene, settings, output, observation_timeout):
    envelope = local_footprint(scene)
    road = scene.blade_site.road
    sg.prepare(road)
    boundary = road.boundary
    feedback = Feedback(scene.name)
    started = monotonic()
    stamp = -1
    zero = np.zeros((2, 3))
    samples = []
    confirmations = []
    requested = ACKERMANN
    request_time = None
    status, reason = "failed", ""
    try:
        stamp, observed, residual, fault, actual_modes = feedback.next(stamp, observation_timeout)
        if fault or max(relative_positions(observed, plan.poses[0])) > .025 or np.max(
                np.abs(wrap(observed[:, 2] - plan.poses[0, :, 2]))) > .025:
            raise RuntimeError(fault or "Gazebo initial assembly does not match the plan")
        start, last = stamp, stamp - 1
        while True:
            feedback_gap = (stamp-last)*1e-9
            if feedback_gap > settings.command_timeout * .65:
                raise RuntimeError(f"Gazebo pose feedback gap {feedback_gap:.3f} s exceeded the tracking allowance")
            dt = (stamp-start)*1e-9
            if fault:
                raise RuntimeError(fault)
            mode, dwell, _ = plan.phase(dt)
            if mode != requested:
                requested, request_time = mode, dt
            if actual_modes[0] != actual_modes[1]:
                raise RuntimeError("vehicle mode feedback is desynchronized")
            confirmed = actual_modes[0] == requested
            if confirmed and request_time is not None:
                confirmations.append(dict(mode=MOTION_LABELS[requested], requested_s=request_time,
                                          confirmed_s=dt, latency_s=dt-request_time))
                request_time = None
            if not confirmed and not dwell:
                raise RuntimeError("both vehicles must confirm the new mode before the stationary dwell ends")
            desired = plan.interpolate(plan.poses, dt)
            poly = world_footprint(envelope, observed[0])
            gap = float(boundary.distance(poly)) if road.covers(poly) else 0.
            mounts = mount_error(observed, scene.offsets)
            command = zero if dwell or not confirmed or dt >= plan.duration else controls(plan, dt, observed, settings)
            samples.append((dt, observed, desired, command, mounts, residual, gap, requested, actual_modes))
            if gap < MIN_ROAD_CLEARANCE or max(mounts) > MOUNT_LIMIT or max(relative_positions(observed, desired)) > TRACK_LIMIT:
                raise RuntimeError(f"safety stop: road clearance {gap:.5f} m, mount error {max(mounts):.5f} m,"
                                   f" tracking error {max(relative_positions(observed, desired)):.5f} m")
            feedback.send(stamp, command, requested)
            if dt >= plan.duration + STOP_WINDOW:
                if (max(relative_positions(observed, plan.poses[-1])) > TERMINAL_POSITION or
                        np.max(np.abs(wrap(observed[:, 2] - plan.poses[-1, :, 2]))) > TERMINAL_YAW):
                    raise RuntimeError("terminal position or heading is outside tolerance")
                previous = samples[-2]
                elapsed = dt-previous[0]
                if (max(relative_positions(observed, previous[1]))/elapsed > settings.stopped_speed or
                        max(np.abs(wrap(observed[:, 2]-previous[1][:, 2])))/elapsed > settings.stopped_yaw_rate):
                    raise RuntimeError("vehicle did not settle at the goal")
                if len(confirmations) != len(plan.switches):
                    raise RuntimeError("paired mode transition count differs from the plan")
                status = "passed"
                feedback.publish_reference(stamp, start, desired, observed, "COMPLETE")
                break
            feedback.publish_reference(stamp, start, desired, observed, "TRACKING")
            last = stamp
            stamp, observed, residual, fault, actual_modes = feedback.next(last, 3.)
    except (RuntimeError, KeyboardInterrupt) as error:
        reason = str(error) or "operator interrupt"
        if samples:
            stopped_reference = plan.interpolate(plan.poses, (stamp-start)*1e-9)
            feedback.publish_reference(stamp, start, stopped_reference, observed, "STOPPED")
    finally:
        # Send a zero twist at a fresh stamp, including on cancellation.
        if stamp >= 0:
            next_stamp = stamp + 1
            with feedback.cv:
                if feedback.last:
                    next_stamp = max(next_stamp, feedback.last[0] + 1)
            feedback.send(next_stamp, zero, requested)
    return record(output, status, reason, plan, monotonic()-started, samples, confirmations)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    parser.add_argument("metrics", type=Path)
    parser.add_argument("--scene", required=True)
    parser.add_argument("--scene-package-dir", type=Path)
    parser.add_argument("--road", type=Path)
    parser.add_argument("--plant", type=Path, required=True)
    parser.add_argument("--observation-timeout", type=float, default=25.)
    args = parser.parse_args()
    paths = installed_paths()
    if args.scene_package_dir:
        paths = args.scene_package_dir, *paths[1:]
    scene = with_trial_road(load_scene(args.scene, paths=paths), args.road)
    settings = PlantSettings(**yaml.safe_load(args.plant.read_text())["plant"])
    result = track(Plan.read(args.plan), scene, settings, args.metrics, args.observation_timeout)
    print(json.dumps(result), flush=True)
    return int(result["status"] != "passed")


if __name__ == "__main__":
    raise SystemExit(main())
