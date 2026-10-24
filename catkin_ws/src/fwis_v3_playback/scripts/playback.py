#!/usr/bin/env python3
"""RViz vehicle marker driven by TimedTrajectory; no independent retiming."""
import bisect
import json
import math
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from fwis_v3_msgs.msg import TimedTrajectory
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from fwis_v3_optimizer.planning_display import PlanningDisplayState


def angle_delta(a, b):
    return math.atan2(math.sin(b-a), math.cos(b-a))


class Playback:
    def __init__(self):
        self.length=rospy.get_param("~vehicle/length",.738)
        self.width=rospy.get_param("~vehicle/width",.500)
        self.traj=None
        self.static=None
        self.times=[]
        self.planning=PlanningDisplayState()
        self.pub=rospy.Publisher("/robot/marker",Marker,queue_size=1)
        self.heading=rospy.Publisher("/robot/heading_marker",Marker,queue_size=1)
        self.hud=rospy.Publisher("/robot/hud",Marker,queue_size=1)
        rospy.Subscriber("/plan/timed_trajectory",TimedTrajectory,self.on_traj,queue_size=1)
        rospy.Subscriber("/start_pose",PoseStamped,self.on_start,queue_size=1)
        rospy.Subscriber("/initialpose",PoseWithCovarianceStamped,self.on_init,queue_size=1)
        rospy.Subscriber("/plan/quality_report",String,self.on_quality,queue_size=1)
        rospy.Timer(rospy.Duration(.04),self.tick)

    def on_traj(self,msg):
        if msg.points:
            if not self.planning.accept_trajectory(int(msg.plan_id)):
                return
        elif not self.planning.accept_report({"plan_id":int(msg.plan_id),"status":"pending"}):
            return
        self.traj=msg if msg.points else None
        self.times=[p.time_from_start.to_sec() for p in msg.points]

    def on_quality(self,msg):
        try:
            report=json.loads(msg.data)
        except (ValueError,TypeError):
            return
        if self.planning.accept_report(report) and report["status"] in ("GEOMETRY_ONLY","REJECTED"):
            self.traj=None
            self.times=[]

    def on_start(self,msg):
        if msg.header.frame_id=="map":self.static=msg.pose

    def on_init(self,msg):
        if msg.header.frame_id=="map":self.static=msg.pose.pose

    def tick(self,_):
        if self.traj is None:
            if self.static is None:return
            pose=self.static
            v=0.0
            plan_id=self.planning.report["plan_id"]
            mode=-1
        else:
            now=max(0.0,(rospy.Time.now()-self.traj.header.stamp).to_sec())
            i=bisect.bisect_right(self.times,now)
            if i==0:i=1
            if i>=len(self.times):
                a=self.traj.points[-1]
                x,y,th=a.center_pose.x,a.center_pose.y,a.center_pose.theta
                v=a.v
                mode=a.mode
            else:
                a,b=self.traj.points[i-1],self.traj.points[i]
                alpha=max(0.0,min(1.0,(now-self.times[i-1])/(self.times[i]-self.times[i-1])))
                x=a.center_pose.x+alpha*(b.center_pose.x-a.center_pose.x)
                y=a.center_pose.y+alpha*(b.center_pose.y-a.center_pose.y)
                th=a.center_pose.theta+alpha*angle_delta(a.center_pose.theta,b.center_pose.theta)
                v=a.v+alpha*(b.v-a.v)
                mode=a.mode
            pose=PoseStamped().pose
            pose.position.x=x;pose.position.y=y
            q=quaternion_from_euler(0,0,th)
            pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=q
            plan_id=self.traj.plan_id
        m=Marker()
        m.header.frame_id="map";m.header.stamp=rospy.Time.now()
        m.ns="v3_vehicle";m.id=0;m.type=Marker.CUBE;m.action=Marker.ADD
        m.pose=pose
        m.scale.x=self.length;m.scale.y=self.width;m.scale.z=.10
        m.color.r=.04;m.color.g=.32;m.color.b=.95;m.color.a=.65
        self.pub.publish(m)
        arrow=Marker()
        arrow.header=m.header;arrow.ns="v3_vehicle_heading";arrow.id=0
        arrow.type=Marker.ARROW;arrow.action=Marker.ADD;arrow.pose=pose
        arrow.scale.x=max(.30,self.length*.62);arrow.scale.y=.075;arrow.scale.z=.075
        arrow.color.r=1.0;arrow.color.g=.12;arrow.color.b=.08;arrow.color.a=.98
        self.heading.publish(arrow)
        h=Marker()
        h.header=m.header;h.ns="v3_hud";h.id=0;h.type=Marker.TEXT_VIEW_FACING;h.action=Marker.ADD
        h.pose.position.x=pose.position.x;h.pose.position.y=pose.position.y
        h.pose.position.z=.48;h.pose.orientation.w=1
        h.scale.z=.18;h.color.r=h.color.g=h.color.b=h.color.a=1
        mode_names={-1:"IDLE",0:"ACKERMANN",1:"OBLIQUE",2:"SPIN"}
        h.text="v3 #%d  %s  center v=%.2f m/s"%(plan_id,mode_names.get(mode,"UNKNOWN"),v)
        status=self.planning.text()
        if status:
            h.text+="\n"+status
        self.hud.publish(h)


if __name__=="__main__":
    rospy.init_node("fwis_v3_playback")
    Playback()
    rospy.spin()
