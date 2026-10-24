#!/usr/bin/env python3
"""Verify that an invalid goal clears a previously published safe plan."""
import math
import time
import rospy
from geometry_msgs.msg import PoseStamped
from fwis_v3_msgs.msg import TimedTrajectory


def goal(x,y,theta):
    msg=PoseStamped()
    msg.header.frame_id="map";msg.header.stamp=rospy.Time.now()
    msg.pose.position.x=x;msg.pose.position.y=y
    msg.pose.orientation.z=math.sin(theta/2)
    msg.pose.orientation.w=math.cos(theta/2)
    return msg


def wait_for(predicate,data,timeout=20):
    limit=time.time()+timeout
    while time.time()<limit:
        if data["traj"] is not None and predicate(data["traj"]):return data["traj"]
        rospy.sleep(.05)
    raise TimeoutError("trajectory state did not arrive")


def main():
    rospy.init_node("v3_failure_smoke",anonymous=True)
    data={"traj":None}
    rospy.Subscriber("/plan/timed_trajectory",TimedTrajectory,
                     lambda m:data.update(traj=m),queue_size=1)
    pub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    limit=time.time()+10
    while pub.get_num_connections()==0 and time.time()<limit:rospy.sleep(.05)
    prior=data["traj"].plan_id if data["traj"] else 0
    pub.publish(goal(13.2,2.65,math.pi))
    safe=wait_for(lambda m:m.plan_id>prior and len(m.points)>0,data)
    pub.publish(goal(.1,.1,0))  # Occupied/out of map-footprint bounds.
    cleared=wait_for(lambda m:m.plan_id>safe.plan_id and len(m.points)==0,data)
    assert cleared.plan_id==safe.plan_id+1
    print("PASS previous_id=%d invalid_id=%d old_trajectory_cleared"%
          (safe.plan_id,cleared.plan_id))


if __name__=="__main__":main()
