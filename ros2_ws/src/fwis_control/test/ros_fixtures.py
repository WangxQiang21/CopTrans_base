"""ROS message conversion for controller demos and integration tests."""
from fwis_v3_msgs.msg import TimedPoint, TimedTrajectory


def trajectory_message(trajectory, stamp):
    message = TimedTrajectory(plan_id=trajectory.plan_id)
    message.header.stamp = stamp
    message.header.frame_id = trajectory.frame
    for sample in trajectory.points:
        point = TimedPoint(v=sample.speed, mode=sample.mode)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(sample.time_ns, 10**9)
        point.center_pose.x = sample.pose.x
        point.center_pose.y = sample.pose.y
        point.center_pose.theta = sample.pose.yaw
        message.points.append(point)
    return message
