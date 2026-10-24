#!/usr/bin/env python3
"""Exercise a ROS Spin goal at the S09 starting vehicle center."""
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from fwis_v3_msgs.msg import PlanSeed,TimedTrajectory


def main():
    rospy.init_node("v3_spin_smoke",anonymous=True)
    try:baseline=rospy.wait_for_message("/plan/seed",PlanSeed,timeout=1).plan_id
    except rospy.ROSException:baseline=0
    data={"seed":None,"traj":None}
    rospy.Subscriber("/plan/seed",PlanSeed,lambda m:data.update(seed=m),queue_size=1)
    rospy.Subscriber("/plan/timed_trajectory",TimedTrajectory,lambda m:data.update(traj=m),queue_size=1)
    pub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    limit=time.time()+10
    while pub.get_num_connections()==0 and time.time()<limit:rospy.sleep(.05)
    goal=PoseStamped()
    goal.header.frame_id="map";goal.header.stamp=rospy.Time.now()
    goal.pose.position.x=13.2;goal.pose.position.y=2.25
    goal.pose.orientation.z=-math.sqrt(.5);goal.pose.orientation.w=math.sqrt(.5)
    pub.publish(goal)
    limit=time.time()+60
    while time.time()<limit:
        seed=data["seed"];traj=data["traj"]
        if seed and seed.plan_id>baseline and traj and traj.plan_id==seed.plan_id and traj.points:
            break
        rospy.sleep(.1)
    else:raise TimeoutError("spin plan not published")
    # The parking spot at the start is close to obstacles: a short O
    # reposition before/after Spin can be the lower-cost swept-safe solution.
    # Test that Spin is used and the requested center pose is still reached,
    # rather than requiring a scenario-specific all-Spin state machine.
    assert seed.primitives and any(a.mode==2 for a in seed.primitives)
    assert all(math.hypot(p.center_pose.x-13.2,p.center_pose.y-2.25)<.16
               for p in traj.points)
    for a,b in zip(traj.points,traj.points[1:]):
        if abs(b.center_pose.theta-a.center_pose.theta)>1e-5:
            assert math.hypot(b.center_pose.x-a.center_pose.x,
                              b.center_pose.y-a.center_pose.y)<1e-6
            assert a.v==0 and b.v==0
    assert all(b.time_from_start>a.time_from_start for a,b in zip(traj.points,traj.points[1:]))
    angles=[p.center_pose.theta for p in traj.points]
    assert angles[-1]>math.pi
    assert all(abs(b-a)<.2 for a,b in zip(angles,angles[1:]))
    assert abs(math.atan2(math.sin(traj.points[-1].center_pose.theta+math.pi/2),
                          math.cos(traj.points[-1].center_pose.theta+math.pi/2)))<.14
    print("PASS spin id=%d actions=%d points=%d duration=%.2fs"%
          (seed.plan_id,len(seed.primitives),len(traj.points),
           traj.points[-1].time_from_start.to_sec()))


if __name__=="__main__":main()
