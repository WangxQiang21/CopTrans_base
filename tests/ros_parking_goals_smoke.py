#!/usr/bin/env python3
"""Probe reported S09 middle-stall goals from the demo's default start."""
import math
import time
import sys
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from fwis_v3_msgs.msg import PlanSeed, TimedTrajectory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Pose, wrap


class IndependentSweepChecker:
    """Test-side OBB/cell SAT and dense interpolation, independent of planner status."""
    def __init__(self, grid_msg, length, width, margin):
        self.occupied=np.asarray(grid_msg.data,dtype=np.int16).reshape(
            (grid_msg.info.height,grid_msg.info.width)) != 0
        self.res=float(grid_msg.info.resolution)
        self.ox=float(grid_msg.info.origin.position.x)
        self.oy=float(grid_msg.info.origin.position.y)
        self.width=int(grid_msg.info.width);self.height=int(grid_msg.info.height)
        self.hl=(float(length)+float(margin))/2
        self.hw=(float(width)+float(margin))/2
        self.corner=math.hypot(self.hl,self.hw)

    def pose_clear(self, pose):
        hl=self.hl;hw=self.hw
        c,s=math.cos(pose.th),math.sin(pose.th)
        rx=hl*abs(c)+hw*abs(s)
        ry=hl*abs(s)+hw*abs(c)
        if (pose.x-rx<self.ox or pose.y-ry<self.oy or
            pose.x+rx>self.ox+self.width*self.res or
            pose.y+ry>self.oy+self.height*self.res):
            return False
        x0=max(0,math.floor((pose.x-rx-self.ox)/self.res))
        x1=min(self.width-1,math.floor((pose.x+rx-self.ox)/self.res))
        y0=max(0,math.floor((pose.y-ry-self.oy)/self.res))
        y1=min(self.height-1,math.floor((pose.y+ry-self.oy)/self.res))
        half=self.res/2
        for iy in range(y0,y1+1):
            for ix in np.flatnonzero(self.occupied[iy,x0:x1+1])+x0:
                dx=self.ox+(ix+.5)*self.res-pose.x
                dy=self.oy+(iy+.5)*self.res-pose.y
                if abs(dx*c+dy*s)>hl+half*(abs(c)+abs(s)):
                    continue
                if abs(-dx*s+dy*c)>hw+half*(abs(c)+abs(s)):
                    continue
                if abs(dx)>half+rx or abs(dy)>half+ry:
                    continue
                return False
        return True

    def segment_clear(self, a, b, max_corner_step=.005):
        dyaw=wrap(b.th-a.th)
        distance=math.hypot(b.x-a.x,b.y-a.y)
        # The configured 40mm footprint margin is much larger than the
        # maximum 2.5mm corner motion between these 5mm-spaced samples.
        n=max(1,math.ceil((distance+self.corner*abs(dyaw))/max_corner_step))
        for i in range(n+1):
            t=i/n
            p=Pose(a.x+t*(b.x-a.x),a.y+t*(b.y-a.y),wrap(a.th+t*dyaw))
            if not self.pose_clear(p):
                return False,n+1,p
        return True,n+1,None


def main():
    rospy.init_node("v3_parking_goals_smoke", anonymous=True)
    map_msg=rospy.wait_for_message("/map",OccupancyGrid,timeout=10)
    assert abs(map_msg.info.origin.orientation.z)<1e-7
    safety=IndependentSweepChecker(
        map_msg,rospy.get_param("/fwis_v3_optimizer/vehicle/length",.738),
        rospy.get_param("/fwis_v3_optimizer/vehicle/width",.500),
        rospy.get_param("/fwis_v3_optimizer/collision/margin",.04))
    state={"seed":None,"search":None,"status":None,"traj":None}
    rospy.Subscriber("/plan/seed",PlanSeed,lambda m:state.update(seed=m),queue_size=1)
    rospy.Subscriber("/plan/search_status",String,lambda m:state.update(search=m.data),queue_size=1)
    rospy.Subscriber("/plan/status",String,lambda m:state.update(status=m.data),queue_size=1)
    rospy.Subscriber("/plan/timed_trajectory",TimedTrajectory,
                     lambda m:state.update(traj=m),queue_size=1)
    pub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    deadline=time.time()+10
    while pub.get_num_connections()==0 and time.time()<deadline:rospy.sleep(.05)
    assert pub.get_num_connections()>0
    targets=((3.387,4.950,1.567),(1.921,4.982,1.594))
    presets=((.740,.500,1.5708),(5.860,.500,1.5708),(10.980,.500,1.5708),
             (6.580,4.000,1.5708),(2.700,4.900,-1.5708),
             (.814,9.214,-.7854),(6.014,9.214,-.7854))
    if len(sys.argv)>1 and sys.argv[1]=="presets":
        goals=presets
    elif len(sys.argv)>1 and sys.argv[1].startswith("p"):
        goals=(presets[int(sys.argv[1][1:])-1],)
    elif len(sys.argv)>1:
        goals=(targets[int(sys.argv[1])-1],)
    else:
        goals=targets
    failures=[]
    for gx,gy,gth in goals:
        base=state["seed"].plan_id if state["seed"] else 0
        goal=PoseStamped();goal.header.frame_id="map";goal.header.stamp=rospy.Time.now()
        goal.pose.position.x=gx;goal.pose.position.y=gy
        goal.pose.orientation.z=math.sin(gth/2);goal.pose.orientation.w=math.cos(gth/2)
        pub.publish(goal)
        expected=base+1;limit=time.time()+45
        while time.time()<limit:
            search=state["search"] or ""
            if "id=%d"%expected in search:break
            rospy.sleep(.05)
        else:raise TimeoutError("planner did not report goal id %d"%expected)
        if not search.startswith("search_success"):
            print("PROBE goal=(%.3f,%.3f,%.3f) %s"%(gx,gy,gth,search))
            failures.append("search failed: %s"%search)
            continue
        while time.time()<limit:
            status=state["status"] or ""
            if "id=%d"%expected in status and status.startswith("success"):break
            if "id=%d"%expected in status and status.startswith("rejected"):
                print("PROBE goal=(%.3f,%.3f,%.3f) %s search=%s"%
                      (gx,gy,gth,status,search))
                failures.append("optimizer rejected: %s"%status)
                break
            rospy.sleep(.05)
        else:raise TimeoutError("optimizer did not accept id %d: %s"%(expected,status))
        if status.startswith("rejected"):
            continue
        while time.time()<limit:
            traj=state["traj"]
            if traj and traj.plan_id==expected and len(traj.points)>1:
                break
            rospy.sleep(.02)
        if not traj or traj.plan_id!=expected or len(traj.points)<=1:
            print("PROBE goal=(%.3f,%.3f,%.3f) missing timed trajectory status=%s"%
                  (gx,gy,gth,status))
            failures.append("missing timed trajectory for id %d"%expected)
            continue
        pts=traj.points
        assert all(b.time_from_start>a.time_from_start for a,b in zip(pts,pts[1:]))
        timed_poses=[Pose(p.center_pose.x,p.center_pose.y,p.center_pose.theta)
                     for p in pts]
        blocked_pose=next((i for i,p in enumerate(timed_poses)
                           if not safety.pose_clear(p)),None)
        if blocked_pose is not None:
            failures.append("timed footprint collides at point %d for id %d"%
                            (blocked_pose,expected))
            print("FAIL timed pose SAT goal=(%.3f,%.3f) point=%d"%
                  (gx,gy,blocked_pose))
            continue
        blocked_segment=None;sweep_samples=0
        for i,(a,b) in enumerate(zip(timed_poses,timed_poses[1:])):
            clear,samples,collision_pose=safety.segment_clear(a,b,.005)
            sweep_samples+=samples
            if not clear:
                blocked_segment=i;break
        if blocked_segment is not None:
            pa=pts[blocked_segment];pb=pts[blocked_segment+1]
            failures.append("timed swept footprint collision at segment %d for id %d"%
                            (blocked_segment,expected))
            print("FAIL timed swept SAT goal=(%.3f,%.3f) segment=%d "
                  "mode=%d->%d t=%.3f..%.3f p=(%.4f,%.4f,%.4f) "
                  "from=(%.4f,%.4f,%.4f) to=(%.4f,%.4f,%.4f)"%
                  (gx,gy,blocked_segment,pa.mode,pb.mode,
                   pa.time_from_start.to_sec(),pb.time_from_start.to_sec(),
                   collision_pose.x,collision_pose.y,collision_pose.th,
                   a.x,a.y,a.th,b.x,b.y,b.th))
            continue
        end=pts[-1].center_pose
        if math.hypot(end.x-gx,end.y-gy)>=.16 or abs(wrap(end.theta-gth))>=.14:
            print("PROBE goal=(%.3f,%.3f,%.3f) terminal tolerance failed"%(gx,gy,gth))
            failures.append("terminal tolerance failed for id %d"%expected)
            continue
        print("PASS goal=(%.3f,%.3f,%.3f) independently_swept=%d_segments/%d_samples %s optimizer={%s}"%
              (gx,gy,gth,len(pts)-1,sweep_samples,search,status))
    if failures:
        raise RuntimeError("%d/%d goal probes failed: %s"%
                           (len(failures),len(goals),"; ".join(failures)))


if __name__=="__main__":
    main()
