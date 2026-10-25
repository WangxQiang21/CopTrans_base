#!/usr/bin/env python3

import math
import os
import shutil
import subprocess
import sys
import tempfile

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import OccupancyGrid
from fwis_msgs.msg import Trajectory


class SafeCorridorNode:
    def __init__(self):
        rospy.init_node("fwis_safe_corridor")

        self.robot_length = rospy.get_param("~vehicle/length", 0.738)
        self.robot_width = rospy.get_param("~vehicle/width", 0.500)
        self.wheelbase = rospy.get_param("~vehicle/wheelbase", 0.494)
        self.track_width = rospy.get_param("~vehicle/track", 0.364)
        self.max_wheel_steer = rospy.get_param(
            "~vehicle/max_wheel_steer", math.pi / 2.0)
        self.max_wheel_steer_rate = rospy.get_param(
            "~vehicle/max_wheel_steer_rate", 2.5)
        self.wheel_speed_epsilon = rospy.get_param("~wheel_speed_epsilon", 0.05)
        self.strict_wheel_check = rospy.get_param("~strict_wheel_check", True)
        self.auto_time_stretch = rospy.get_param("~auto_time_stretch", True)
        self.enforce_wheel_rate_in_ocp = rospy.get_param(
            "~enforce_wheel_rate_in_ocp", False)
        self.lock_verified_geometry = rospy.get_param(
            "~lock_verified_geometry", True)
        self.max_velocity = rospy.get_param("~max_velocity", 1.0)
        self.max_acceleration = rospy.get_param("~max_acceleration", 2.0)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 0.7)
        self.max_yaw_acceleration = rospy.get_param("~max_yaw_acceleration", 1.5)
        self.comfort_weight = rospy.get_param("~comfort_weight", 0.05)
        self.tracking_weight = rospy.get_param("~tracking_weight", 1.0)
        self.ipopt_max_iterations = int(
            rospy.get_param("~ipopt_max_iterations", 200))
        self.ipopt_tol = rospy.get_param("~ipopt_tol", 1e-4)
        self.infeasibility_tolerance = rospy.get_param(
            "~infeasibility_tolerance", 1e-4)

        self.corridor_max_distance = rospy.get_param(
            "~corridor_max_distance", 3.0)
        self.corridor_fast_step = rospy.get_param("~corridor_fast_step", 0.2)
        self.corridor_fine_step = rospy.get_param("~corridor_fine_step", 0.05)
        self.corridor_axis_aligned = rospy.get_param(
            "~corridor_axis_aligned", True)
        self.corridor_resample_distance = rospy.get_param(
            "~corridor_resample_distance", 0.1)
        self.safety_margin = rospy.get_param("~safety_margin", 0.04)

        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.casadi_script = rospy.get_param(
            "~casadi_script",
            os.path.join(script_dir, "casadi_safe_corridor_backend.py"))

        self._map = None
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber(
            "/plan/trajectory", Trajectory, self.on_trajectory, queue_size=1)
        self.pub = rospy.Publisher(
            "/plan/trajectory_optimized", Trajectory, queue_size=1, latch=True)

        rospy.loginfo("[safe_corridor] ready  backend=%s", self.casadi_script)

    def on_map(self, msg):
        self._map = msg

    def is_occupied(self, mx, my):
        if self._map is None:
            return False
        info = self._map.info
        res = info.resolution
        ox = info.origin.position.x
        oy = info.origin.position.y
        px = int((mx - ox) / res)
        py = int((my - oy) / res)
        if px < 0 or py < 0 or px >= info.width or py >= info.height:
            return True
        return self._map.data[py * info.width + px] > 50

    def obb_collides(self, x, y, yaw):
        """Dense full-vehicle OBB collision test against the occupancy map."""
        if self._map is None:
            return True
        step = max(self._map.info.resolution * 0.7, 0.01)
        # Match GridMap::obb_collides_dense: safety_margin inflates the
        # *overall* footprint, not each half extent independently.
        half_l = 0.5 * (self.robot_length + self.safety_margin)
        half_w = 0.5 * (self.robot_width + self.safety_margin)
        c, s = math.cos(yaw), math.sin(yaw)
        u = -half_l
        while u <= half_l + 1e-9:
            v = -half_w
            while v <= half_w + 1e-9:
                if self.is_occupied(x + u * c - v * s,
                                    y + u * s + v * c):
                    return True
                v += step
            u += step
        return False

    def trajectory_collision_free(self, poses):
        """Sweep the whole OBB between every pair of optimized states."""
        if len(poses) < 2:
            return False, -1
        half_diag = math.hypot(0.5 * (self.robot_length + self.safety_margin),
                               0.5 * (self.robot_width + self.safety_margin))
        for i in range(1, len(poses)):
            x0, y0, t0 = poses[i - 1]
            x1, y1, t1 = poses[i]
            ds = math.hypot(x1 - x0, y1 - y0)
            dt = (t1 - t0 + math.pi) % (2.0 * math.pi) - math.pi
            # Bound both translational and rotational corner sweep to 2 cm.
            count = max(1, int(math.ceil(max(ds, abs(dt) * half_diag) / 0.02)))
            for k in range(count + 1):
                alpha = k / count
                if self.obb_collides(x0 + alpha * (x1 - x0),
                                     y0 + alpha * (y1 - y0),
                                     t0 + alpha * dt):
                    return False, i
        return True, -1

    def strip_is_safe(self, x0, y0, dx, dy, L, step_size, N):
        length = math.hypot(dx, dy)
        if length < 1e-9:
            return True
        nx = -dy / length
        ny = dx / length
        for i in range(N + 1):
            t = i / max(N, 1)
            sx = x0 + t * dx
            sy = y0 + t * dy
            s = -L * 0.5
            while s <= L * 0.5 + 1e-9:
                px = sx + s * nx
                py = sy + s * ny
                if self.is_occupied(px, py):
                    return False
                s += step_size
        return True

    def build_one_corridor(self, points, idx, half_diag):
        cx, cy = points[idx]
        n = len(points)
        if idx + 1 < n and idx > 0:
            dx = points[idx + 1][0] - points[idx - 1][0]
            dy = points[idx + 1][1] - points[idx - 1][1]
        elif idx + 1 < n:
            dx = points[idx + 1][0] - cx
            dy = points[idx + 1][1] - cy
        elif idx > 0:
            dx = cx - points[idx - 1][0]
            dy = cy - points[idx - 1][1]
        else:
            dx, dy = 1.0, 0.0
        length = math.hypot(dx, dy)
        if length < 1e-9:
            dx, dy, length = 1.0, 0.0, 1.0
        tx = dx / length
        ty = dy / length

        if self.corridor_axis_aligned:
            if abs(tx) >= abs(ty):
                tx = 1.0 if tx >= 0 else -1.0
                ty = 0.0
            else:
                tx = 0.0
                ty = 1.0 if ty >= 0 else -1.0

        nx = -ty
        ny = tx

        bwd = fwd = right = left = half_diag

        def grow(bound, step, dx, dy, is_tan):
            while bound + step <= self.corridor_max_distance:
                x0 = cx + dx * bound
                y0 = cy + dy * bound
                ex = dx * step
                ey = dy * step
                strip_len = (left + right) if is_tan else (bwd + fwd)
                sd_x = nx if is_tan else tx
                sd_y = ny if is_tan else ty
                N = max(1, int(math.ceil(step / 0.4)))
                x0e = x0 + ex
                y0e = y0 + ey
                seg_dx = x0e - x0
                seg_dy = y0e - y0
                if not self.strip_is_safe(x0, y0, seg_dx, seg_dy,
                                           strip_len, 0.3, N):
                    return bound
                bound = bound + step
            return bound

        for pas in range(2):
            step = self.corridor_fast_step if pas == 0 else self.corridor_fine_step
            fwd = grow(fwd, step, tx, ty, True)
            bwd = grow(bwd, step, -tx, -ty, True)
            right = grow(right, step, nx, ny, False)
            left = grow(left, step, -nx, -ny, False)

        return (cx, cy, tx, ty, nx, ny, bwd, fwd, right, left)

    def resample(self, path_poses):
        if len(path_poses) < 2:
            return path_poses
        cum = [0.0]
        for i in range(1, len(path_poses)):
            cum.append(cum[-1] + math.hypot(
                path_poses[i][0] - path_poses[i - 1][0],
                path_poses[i][1] - path_poses[i - 1][1]))
        total = cum[-1]
        if total < self.corridor_resample_distance:
            return path_poses
        out = [path_poses[0]]
        d = self.corridor_resample_distance
        while d < total - 1e-6:
            j = next(i for i in range(len(cum) - 1) if cum[i + 1] >= d)
            t = (d - cum[j]) / max(cum[j + 1] - cum[j], 1e-9)
            x = path_poses[j][0] + t * (path_poses[j + 1][0] - path_poses[j][0])
            y = path_poses[j][1] + t * (path_poses[j + 1][1] - path_poses[j][1])
            dyaw = (path_poses[j + 1][2] - path_poses[j][2] + math.pi) % (2.0 * math.pi) - math.pi
            yaw = path_poses[j][2] + t * dyaw
            out.append((x, y, yaw))
            d += self.corridor_resample_distance
        out.append(path_poses[-1])
        return out

    def build_corridors(self, path_poses):
        resampled = self.resample(path_poses)
        half_wb = self.robot_length * 0.25
        front_axle = [(x + half_wb * math.cos(t), y + half_wb * math.sin(t))
                      for x, y, t in resampled]
        rear_axle = [(x - half_wb * math.cos(t), y - half_wb * math.sin(t))
                     for x, y, t in resampled]
        half_diag = math.sqrt(
            (self.robot_length * 0.5) ** 2 + (self.robot_width * 0.5) ** 2)
        front_corridors = [self.build_one_corridor(front_axle, i, half_diag)
                           for i in range(len(resampled))]
        rear_corridors = [self.build_one_corridor(rear_axle, i, half_diag)
                          for i in range(len(resampled))]
        return resampled, front_corridors, rear_corridors

    def write_input(self, path, raw_points, front_corridors, rear_corridors):
        with open(path, "w") as f:
            f.write("CONFIG\n")
            f.write(f"{self.robot_length} {self.wheelbase} {self.track_width} "
                    f"{self.max_wheel_steer_rate} {int(self.enforce_wheel_rate_in_ocp)} "
                    f"{int(self.lock_verified_geometry)} "
                    f"{self.max_velocity} {self.max_acceleration} "
                    f"{self.max_yaw_rate} {self.max_yaw_acceleration} "
                    f"{self.comfort_weight} {self.tracking_weight} "
                    f"{self.ipopt_max_iterations} "
                    f"{self.ipopt_tol} {self.infeasibility_tolerance} "
                    f"\n")
            f.write(f"POINTS {len(raw_points)}\n")
            for x, y, yaw in raw_points:
                f.write(f"{x:.10f} {y:.10f} {yaw:.10f}\n")
            f.write(f"FRONT_CORRIDORS {len(front_corridors)}\n")
            for c in front_corridors:
                f.write(" ".join(f"{v:.10f}" for v in c) + "\n")
            f.write(f"REAR_CORRIDORS {len(rear_corridors)}\n")
            for c in rear_corridors:
                f.write(" ".join(f"{v:.10f}" for v in c) + "\n")

    def wheel_timing(self, twists, total_time):
        """Return wheel-feasible local durations for a continuous body twist.

        Each wheel at (±wheelbase/2, ±track/2) receives the rigid-body
        velocity induced by (u, v, r).  Choosing the equivalent wheel heading
        modulo pi accounts for reversible wheel drive, then makes steering-rate
        validation independent of an arbitrary forward/reverse convention.
        """
        if len(twists) < 2:
            return True, 0.0, []
        base_dt = total_time / max(len(twists) - 1, 1)
        if base_dt <= 1e-6:
            return False, float("inf"), []
        locations = (( self.wheelbase * 0.5,  self.track_width * 0.5),
                     ( self.wheelbase * 0.5, -self.track_width * 0.5),
                     (-self.wheelbase * 0.5,  self.track_width * 0.5),
                     (-self.wheelbase * 0.5, -self.track_width * 0.5))
        previous = [None] * 4
        max_angle = 0.0
        required_dt = []
        for state_idx, (u, v, r) in enumerate(twists):
            step_dt = base_dt
            for i, (px, py) in enumerate(locations):
                vx = u - r * py
                vy = v + r * px
                # At near-zero wheel speed the wheel can retain its previous
                # orientation while the vehicle changes mode; atan2 is then
                # numerically undefined and must not manufacture a steering
                # rate spike from an equivalent +/- pi/2 representation.
                if math.hypot(vx, vy) < self.wheel_speed_epsilon:
                    angle = previous[i] if previous[i] is not None else 0.0
                else:
                    # Equivalent angle in [-pi/2, pi/2) with wheel-direction
                    # reversal handled by the drive controller.
                    angle = (math.atan2(vy, vx) + math.pi / 2.0) % math.pi - math.pi / 2.0
                max_angle = max(max_angle, abs(angle))
                if previous[i] is not None:
                    delta = (angle - previous[i] + math.pi / 2.0) % math.pi - math.pi / 2.0
                    step_dt = max(step_dt, abs(delta) / self.max_wheel_steer_rate)
                previous[i] = angle
            if state_idx > 0:
                required_dt.append(step_dt)
        return max_angle <= self.max_wheel_steer + 1e-4, max_angle, required_dt

    def on_trajectory(self, msg):
        if self._map is None:
            rospy.logwarn("[safe_corridor] no map yet")
            return
        n = len(msg.pose)
        if n < 3:
            rospy.logwarn("[safe_corridor] trajectory too short (n=%d)", n)
            return

        raw_poses = [(p.x, p.y, p.theta) for p in msg.pose]
        raw_modes = list(msg.mode)
        raw_phis = list(msg.phi)

        raw_clear, raw_bad_segment = self.trajectory_collision_free(raw_poses)
        if not raw_clear:
            rospy.logerr("[safe_corridor] REJECTED unsafe topology seed at segment %d; "
                           "optimizer is not invoked", raw_bad_segment)
            return

        t0 = rospy.Time.now()
        resampled, front_corridors, rear_corridors = self.build_corridors(raw_poses)
        build_ms = (rospy.Time.now() - t0).to_sec() * 1000.0

        tmpdir = tempfile.mkdtemp(prefix="fwis_sc_")
        try:
            inp = os.path.join(tmpdir, "input.txt")
            out = os.path.join(tmpdir, "output.txt")
            self.write_input(inp, resampled, front_corridors, rear_corridors)

            t1 = rospy.Time.now()
            proc = subprocess.run(
                [sys.executable, self.casadi_script, inp, out],
                capture_output=True, text=True, timeout=60)
            solve_ms = (rospy.Time.now() - t1).to_sec() * 1000.0

            if proc.returncode != 0:
                rospy.logwarn("[safe_corridor] CasADi crashed:\n%s", proc.stderr)
                return

            try:
                with open(out) as f:
                    text = f.read()
            except IOError:
                rospy.logwarn("[safe_corridor] missing output file")
                return

            lines = [l.strip() for l in text.splitlines() if l.strip()]
            if len(lines) < 2:
                rospy.logwarn("[safe_corridor] output too short")
                return

            parts = lines[0].split()
            ok = len(parts) >= 3 and bool(int(parts[1]))
            status = int(parts[2]) if len(parts) >= 3 else -1
            total_time = float(parts[3]) if len(parts) >= 4 else 0.0

            if not lines[1].startswith("STATES"):
                rospy.logwarn("[safe_corridor] missing STATES in output")
                return
            n_states = int(lines[1].split()[1])
            opt_poses = []
            opt_twist = []
            for i in range(n_states):
                vals = lines[2 + i].split()
                if len(vals) >= 3:
                    opt_poses.append((float(vals[0]), float(vals[1]), float(vals[2])))
                    opt_twist.append(tuple(float(v) for v in vals[3:6])
                                     if len(vals) >= 6 else (0.0, 0.0, 0.0))

            if not ok:
                # Never publish IPOPT's debug iterate.  The raw topology seed
                # has just passed the same dense full-OBB sweep, and includes
                # explicit Spin transitions, so it is the safe fallback.
                fallback = Trajectory()
                fallback.header = msg.header
                fallback.pose = list(msg.pose)
                fallback.mode = msg.mode
                fallback.phi = list(msg.phi)
                fallback.v = [0.0] * len(msg.pose)
                fallback.omega = [0.0] * len(msg.pose)
                fallback.d_switch = [
                    1.0 if i > 0 and int(msg.mode[i]) != int(msg.mode[i - 1]) else 0.0
                    for i in range(len(msg.pose))]
                self.pub.publish(fallback)
                rospy.logwarn("[safe_corridor] IPOPT not converged (status=%d); "
                              "published verified topology fallback", status)
                return

            collision_free, collision_segment = self.trajectory_collision_free(opt_poses)
            if not collision_free:
                rospy.logerr("[safe_corridor] REJECTED collision in optimized OBB sweep "
                               "at segment %d; no trajectory published", collision_segment)
                return

            wheel_ok, max_steer, segment_dt = self.wheel_timing(
                opt_twist, total_time)
            if not wheel_ok:
                rospy.logwarn("[safe_corridor] wheel feasibility failed: "
                              "max_delta=%.3f rad (limit %.3f)",
                              max_steer, self.max_wheel_steer)
                if self.strict_wheel_check:
                    return
            nominal_dt = total_time / max(len(opt_poses) - 1, 1)
            if self.auto_time_stretch and segment_dt:
                total_time = sum(segment_dt)
                max_steer_rate = max(
                    abs(0.0) if dt <= 1e-9 else self.max_wheel_steer_rate
                    for dt in segment_dt)
            else:
                segment_dt = [nominal_dt] * max(len(opt_poses) - 1, 0)
                max_steer_rate = self.max_wheel_steer_rate

            out_traj = Trajectory()
            out_traj.header = msg.header
            m_len = len(raw_modes)
            p_len = len(raw_phis)

            # Classify continuous body twist after optimization.  The optimiser
            # never receives a discrete mode label, eliminating primitive-level
            # mode flicker; labels are merely the executable steering regime.
            def classify(u, v, r):
                speed = math.hypot(u, v)
                if speed < 0.03 and abs(r) >= 0.05:
                    return 3, 0.0  # spin
                if abs(r) >= 0.05 or abs(v) <= max(0.04, 0.2 * abs(u)):
                    return 0, 0.0  # Ackermann / longitudinal
                if abs(u) <= max(0.04, 0.2 * abs(v)):
                    return 1, 0.0  # lateral
                return 2, math.atan2(v, u)  # crab / diagonal

            out_modes = []

            for i, (x, y, yaw) in enumerate(opt_poses):
                p = Pose2D()
                p.x = x
                p.y = y
                p.theta = yaw
                out_traj.pose.append(p)
                if i > 0:
                    ds = math.hypot(x - opt_poses[i - 1][0],
                                    y - opt_poses[i - 1][1])
                    local_dt = segment_dt[i - 1]
                else:
                    ds = 0.0
                    local_dt = segment_dt[0] if segment_dt else nominal_dt
                out_traj.v.append(ds / local_dt if local_dt > 1e-9 else 0.0)
                out_traj.omega.append(0.0)
                u, vbody, yaw_rate = opt_twist[i] if i < len(opt_twist) else (0., 0., 0.)
                mode, phi = classify(u, vbody, yaw_rate)
                out_modes.append(mode)
                out_traj.phi.append(phi)
                if i > 0:
                    dyaw = (yaw - opt_poses[i - 1][2] + math.pi) % (2.0 * math.pi) - math.pi
                    out_traj.omega[-1] = dyaw / local_dt if local_dt > 1e-9 else 0.0
                else:
                    out_traj.omega[-1] = 0.0
                out_traj.d_switch.append(
                    1.0 if i > 0 and mode != out_modes[i - 1] else 0.0)

            # Keep the ROS uint8[] representation as bytes.  A Python int
            # list serializes but breaks some rospy subscribers/tools that
            # expect the generated byte-string representation.
            out_traj.mode = bytes(out_modes)
            self.pub.publish(out_traj)
            rospy.loginfo(
                "[safe_corridor] corr=%dpts  build=%.0fms  solve=%.0fms  "
                "T=%.2fs  ok=%d  wheel=(%.2f rad, <=%.2f rad/s)",
                len(resampled), build_ms, solve_ms, total_time, ok,
                max_steer, max_steer_rate)
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)


def main():
    node = SafeCorridorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
