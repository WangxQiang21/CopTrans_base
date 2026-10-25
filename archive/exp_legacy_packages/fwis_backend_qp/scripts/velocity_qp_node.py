#!/usr/bin/env python3
"""ROS node: velocity-profile QP backend for the 4WIS Hybrid A* frontend.

Subscribes to /plan/trajectory (fwis_msgs/Trajectory) with geometry + mode
labels but v=0 everywhere (frontend does not solve velocity).

Runs the QP solver from fwis_backend_qp.velocity_qp, then re-publishes to
/plan/trajectory_optimized with populated v and omega fields.

Prints a summary of v at each mode-switch point.
"""
from __future__ import annotations

import sys
import os
import time

import numpy as np
import rospy
from fwis_msgs.msg import Trajectory

# Make the package importable when run standalone (rosrun)
_here = os.path.dirname(os.path.abspath(__file__))
_pkg_src = os.path.abspath(os.path.join(_here, os.pardir, "src"))
if _pkg_src not in sys.path:
    sys.path.insert(0, _pkg_src)

from fwis_backend_qp.velocity_qp import solve_velocity_qp, QPConfig, MODE_SPIN


class VelocityQPNode:
    def __init__(self):
        rospy.init_node("fwis_velocity_qp")

        # Config
        cfg = QPConfig()
        cfg.v_max_ack     = rospy.get_param("~v_max_ack",     1.50)
        cfg.v_max_ack_hi  = rospy.get_param("~v_max_ack_hi",  0.75)
        cfg.v_max_lat     = rospy.get_param("~v_max_lat",     0.75)
        cfg.v_max_dia     = rospy.get_param("~v_max_dia",     1.50)
        cfg.v_max_dia_hi  = rospy.get_param("~v_max_dia_hi",  0.75)
        cfg.v_max_spin    = rospy.get_param("~v_max_spin",    1.0)
        cfg.a_max         = rospy.get_param("~a_max",         2.0)
        cfg.a_lat_max     = rospy.get_param("~a_lat_max",     1.5)
        cfg.lambda_jerk   = rospy.get_param("~lambda_jerk",   0.1)
        cfg.ipopt_max_iter = rospy.get_param("~ipopt_max_iter", 200)
        self.cfg = cfg

        self.sub = rospy.Subscriber("/plan/trajectory", Trajectory,
                                    self.on_trajectory, queue_size=1)
        self.pub = rospy.Publisher("/plan/trajectory_optimized", Trajectory,
                                   queue_size=1, latch=True)

        rospy.loginfo("[velocity_qp] listening on /plan/trajectory  "
                      "cfg: a_max=%.1f  a_lat_max=%.1f  lambda_jerk=%.2f",
                      cfg.a_max, cfg.a_lat_max, cfg.lambda_jerk)

    def on_trajectory(self, msg: Trajectory):
        n = len(msg.pose)
        if n < 3:
            rospy.logwarn("[velocity_qp] trajectory too short (n=%d), skipping", n)
            return

        x = np.array([p.x for p in msg.pose])
        y = np.array([p.y for p in msg.pose])
        theta = np.array([p.theta for p in msg.pose])
        modes = np.array(list(msg.mode)) if msg.mode else np.zeros(n, dtype=int)
        phis  = np.array(list(msg.phi))  if msg.phi  else np.zeros(n)

        t_start = time.time()
        res = solve_velocity_qp(x, y, theta, modes.tolist(), phis, cfg=self.cfg)
        elapsed_ms = (time.time() - t_start) * 1000

        if not res.success:
            rospy.logwarn("[velocity_qp] solver did not fully converge: %s",
                          res.solver_status)

        # Re-publish trajectory with filled velocities
        out = Trajectory()
        out.header = msg.header
        out.pose = msg.pose
        out.v = res.v.tolist()
        # omega: from Spin segments (theta changes), 0 otherwise
        omega = np.zeros(n)
        for i in range(n - 1):
            if int(modes[i]) == MODE_SPIN:
                dt = res.t[i + 1] - res.t[i]
                if dt > 1e-6:
                    omega[i] = (theta[i + 1] - theta[i]) / dt
        out.omega = omega.tolist()
        out.mode = msg.mode
        out.phi  = msg.phi
        out.d_switch = msg.d_switch

        self.pub.publish(out)

        # Log summary — for each switch point, show (v_local_heuristic vs v_qp)
        # v_local_heuristic reconstruction: v_ref · min(1, dist_to_goal/d_slow)
        v_ref = float(rospy.get_param("/fwis_planner/cost/v_ref", 1.0))
        d_slow = float(rospy.get_param("/fwis_planner/cost/d_slow", 3.0))
        gx, gy = x[-1], y[-1]

        def v_local_heur(xi, yi):
            d = ((gx - xi) ** 2 + (gy - yi) ** 2) ** 0.5
            return v_ref * min(1.0, d / d_slow)

        switch_info = []
        for k in res.switch_indices[1:-1]:  # skip start/end (always 0)
            v_h = v_local_heur(x[k], y[k])
            switch_info.append(f"i={k}(v_heur={v_h:.2f},v_qp={res.v[k]:.3f})")

        rospy.loginfo("[velocity_qp] %.1fms  T=%.2fs  v_peak=%.2f  n_switch=%d",
                      elapsed_ms, res.total_time, float(res.v.max()),
                      max(0, len(res.switch_indices) - 2))
        if switch_info:
            rospy.loginfo("[velocity_qp] switches: %s", "  ".join(switch_info))


def main():
    node = VelocityQPNode()
    rospy.spin()


if __name__ == "__main__":
    main()
