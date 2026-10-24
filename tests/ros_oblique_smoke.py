#!/usr/bin/env python3
"""Exercise pure lateral motion as the ±90° case of O mode."""
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from fwis_v3_msgs.msg import PlanSeed,TimedTrajectory


def main():
    rospy.init_node("v3_oblique_smoke",anonymous=True)
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
    goal.pose.position.x=13.2;goal.pose.position.y=2.65
    goal.pose.orientation.z=1
    pub.publish(goal)
    limit=time.time()+60
    while time.time()<limit:
        seed=data["seed"];traj=data["traj"]
        if seed and seed.plan_id>baseline and traj and traj.plan_id==seed.plan_id and traj.points:break
        rospy.sleep(.1)
    else:raise TimeoutError("oblique plan not published")
    assert seed.primitives and all(a.mode==1 for a in seed.primitives)
    assert all(abs(math.atan2(math.sin(p.center_pose.theta-math.pi),
                              math.cos(p.center_pose.theta-math.pi)))<1e-4
               for p in traj.points)
    assert all(p.mode==p.OBLIQUE for p in traj.points)
    assert all(b.time_from_start>a.time_from_start for a,b in zip(traj.points,traj.points[1:]))
    assert abs(traj.points[-1].center_pose.y-2.65)<.16
    assert any(p.v>0 for p in traj.points)
    print("PASS oblique id=%d actions=%d points=%d"%
          (seed.plan_id,len(seed.primitives),len(traj.points)))


if __name__=="__main__":main()
