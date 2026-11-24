"""Read ROS pose, clock and trajectory values in the tracker's planar convention."""

import math

from .model import Point, Pose, Trajectory


def stamp_ns(message):
    return message.sec * 10**9 + message.nanosec if message.nanosec < 10**9 else -1


def planar_pose(message):
    p, q = message.position, message.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not norm or not all(map(math.isfinite, (norm, p.x, p.y))):
        return None
    x, y, z, w = (component / norm for component in (q.x, q.y, q.z, q.w))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return Pose(p.x, p.y, yaw)


def read_trajectory(message):
    points = tuple(Point(stamp_ns(p.time_from_start),
                         Pose(p.center_pose.x, p.center_pose.y, p.center_pose.theta),
                         p.v, p.mode) for p in message.points)
    return Trajectory(stamp_ns(message.header.stamp), message.header.frame_id, message.plan_id, points)
