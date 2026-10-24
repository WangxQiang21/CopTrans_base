#!/usr/bin/env python3
"""Run against `roslaunch fwis_v3_demo demo.launch rviz:=false`."""
import math
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from fwis_v3_msgs.msg import PlanSeed, TimedTrajectory

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import Grid, Pose, wrap


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--preset",type=int,default=2,
                        help="S09 preset (2=bottom-middle, 5=middle-B)")
    args=parser.parse_args()
    goals={1:(.740,.500,math.pi/2),
           2:(5.860,.500,math.pi/2),
           3:(10.980,.500,math.pi/2),
           4:(6.580,4.000,math.pi/2),
           5:(2.700,4.900,-math.pi/2),
           6:(.814,9.214,-math.pi/4),
           7:(6.014,9.214,-math.pi/4)}
    if args.preset not in goals:
        parser.error("preset must be in 1..7")
    goal_x,goal_y,goal_theta=goals[args.preset]
    rospy.init_node("v3_s09_smoke", anonymous=True)
    grid_msg=rospy.wait_for_message("/map",OccupancyGrid,timeout=10)
    try:
        baseline=rospy.wait_for_message("/plan/seed",PlanSeed,timeout=1).plan_id
    except rospy.ROSException:
        baseline=0
    data={"seed":None,"traj":None,"status":None,"search":None,"corridor":None,
          "modes_viz":None,"coarse_modes":None,"heading":None,"hud":None}
    rospy.Subscriber("/plan/seed",PlanSeed,lambda msg:data.update(seed=msg),queue_size=1)
    rospy.Subscriber("/plan/timed_trajectory",TimedTrajectory,lambda msg:data.update(traj=msg),queue_size=1)
    rospy.Subscriber("/plan/status",String,lambda msg:data.update(status=msg.data),queue_size=1)
    rospy.Subscriber("/plan/search_status",String,lambda msg:data.update(search=msg.data),queue_size=1)
    rospy.Subscriber("/plan/corridor_markers",MarkerArray,
                     lambda msg:data.update(corridor=msg),queue_size=1)
    rospy.Subscriber("/plan/mode_strip",MarkerArray,
                     lambda msg:data.update(modes_viz=msg),queue_size=1)
    rospy.Subscriber("/plan/mode_strip_coarse",MarkerArray,
                     lambda msg:data.update(coarse_modes=msg),queue_size=1)
    rospy.Subscriber("/robot/heading_marker",Marker,
                     lambda msg:data.update(heading=msg),queue_size=1)
    rospy.Subscriber("/robot/hud",Marker,
                     lambda msg:data.update(hud=msg),queue_size=1)
    pub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1)
    deadline=time.time()+10
    while pub.get_num_connections()==0 and time.time()<deadline:rospy.sleep(.05)
    goal=PoseStamped()
    goal.header.frame_id="map";goal.header.stamp=rospy.Time.now()
    goal.pose.position.x=goal_x;goal.pose.position.y=goal_y
    goal.pose.orientation.z=math.sin(goal_theta/2)
    goal.pose.orientation.w=math.cos(goal_theta/2)
    pub.publish(goal)
    deadline=time.time()+90
    while time.time()<deadline:
        seed=data["seed"];traj=data["traj"]
        if seed is not None and seed.plan_id>baseline and traj is not None and traj.plan_id==seed.plan_id and traj.points:
            break
        if (data["status"] and data["status"].startswith("rejected") and
            "id=%d"%(baseline+1) in data["status"]):
            raise RuntimeError(data["status"])
        rospy.sleep(.1)
    else:
        raise TimeoutError("no final timed trajectory; status=%r"%data["status"])
    modes=set(a.mode for a in seed.primitives)
    assert modes and modes.issubset({0,1,2})
    pts=traj.points
    assert len(pts)>2
    assert all(b.time_from_start>a.time_from_start for a,b in zip(pts,pts[1:]))
    assert abs(pts[0].center_pose.x-13.2)<1e-5
    assert abs(pts[0].center_pose.y-2.25)<1e-5
    end=pts[-1].center_pose
    assert math.hypot(end.x-goal_x,end.y-goal_y)<.16
    assert abs(wrap(end.theta-goal_theta))<.14
    spin_pairs=sum(abs(a.center_pose.x-b.center_pose.x)<1e-9 and
                   abs(a.center_pose.y-b.center_pose.y)<1e-9 and
                   abs(wrap(a.center_pose.theta-b.center_pose.theta))>1e-6 and
                   a.v==0 and b.v==0 for a,b in zip(pts,pts[1:]))
    if 2 in modes:
        assert spin_pairs>0
    grid=Grid(np.asarray(grid_msg.data,dtype=np.int8).reshape(grid_msg.info.height,grid_msg.info.width)!=0,
              grid_msg.info.resolution,grid_msg.info.origin.position.x,grid_msg.info.origin.position.y)
    p=[Pose(pt.center_pose.x,pt.center_pose.y,pt.center_pose.theta) for pt in pts]
    assert grid.pose_clear(p[0])
    assert all(grid.interpolated_clear(a,b) for a,b in zip(p,p[1:]))
    env=data["corridor"]
    assert env is not None and any(m.ns=="v3_vehicle_footprints" and
                                   m.type==m.LINE_LIST and len(m.points)>0 for m in env.markers)
    corridors=[m for m in env.markers if m.ns=="v3_vehicle_footprints" and
               m.action==Marker.ADD]
    assert corridors
    # At least one polygon edge must follow a non-cardinal reference heading;
    # this catches regressions to world-axis-aligned corridor visualization.
    oriented_edges=[(b.x-a.x,b.y-a.y)
                    for corridor in corridors
                    for a,b in zip(corridor.points[0::2],corridor.points[1::2])]
    assert any(abs(dx)>1e-3 and abs(dy)>1e-3 for dx,dy in oriented_edges), \
        "corridor polygons appear to be world-axis aligned"
    expected_colors={0:(0.0,.7,1.0),1:(1.0,.55,0.0),2:(.8,.2,1.0)}
    final_modes={pt.mode for pt in pts}
    assert all(marker.id in final_modes for marker in corridors)
    for marker in corridors:
        r,g,b=expected_colors[marker.id]
        assert abs(marker.color.r-r)<1e-5
        assert abs(marker.color.g-g)<1e-5
        assert abs(marker.color.b-b)<1e-5
        # Each LINE_LIST polygon has eight points: four physical body edges.
        for k in range(0,len(marker.points),8):
            a,b=marker.points[k:k+2]
            c,d=marker.points[k+2:k+4]
            assert abs(math.hypot(b.x-a.x,b.y-a.y)-.738)<1e-5
            assert abs(math.hypot(d.x-c.x,d.y-c.y)-.500)<1e-5
    mode_markers=data["modes_viz"]
    assert mode_markers is not None
    colored=[m for m in mode_markers.markers if m.ns=="v3_modes"]
    assert len(colored)==sum(a.mode!=b.mode for a,b in zip(pts,pts[1:]))+1
    assert all(len(m.points)>1 for m in colored)
    assert data["coarse_modes"] is not None and any(
        m.ns=="v3_coarse_modes" for m in data["coarse_modes"].markers)
    # MarkerArray DELETEALL markers need valid frames too; RViz otherwise
    # marks the associated display yellow even when the ADD markers look right.
    for marker_array in (env,mode_markers,data["coarse_modes"]):
        for marker in marker_array.markers:
            assert marker.header.frame_id=="map"
            if marker.action==Marker.ADD:
                q=marker.pose.orientation
                qnorm=math.sqrt(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w)
                assert abs(qnorm-1.0)<1e-6, (marker.ns,qnorm)
                assert marker.scale.x>0.0, marker.ns
                assert len(marker.points)>=2, (marker.ns,len(marker.points))
    assert all(pt.mode in (pt.ACKERMANN,pt.OBLIQUE,pt.SPIN) for pt in pts)
    deadline=time.time()+3
    while time.time()<deadline and (data["heading"] is None or data["hud"] is None):
        rospy.sleep(.05)
    assert data["heading"] is not None and data["heading"].type==Marker.ARROW
    assert data["hud"] is not None and any(
        mode_name in data["hud"].text for mode_name in
        ("ACKERMANN","OBLIQUE","SPIN"))
    translation=0.0;translation_dt=0.0
    for a,b in zip(pts,pts[1:]):
        if a.mode!=a.SPIN and b.mode!=b.SPIN:
            translation+=math.hypot(b.center_pose.x-a.center_pose.x,
                                    b.center_pose.y-a.center_pose.y)
            translation_dt+=(b.time_from_start-a.time_from_start).to_sec()
    average_translation_speed=translation/max(translation_dt,1e-9)
    assert average_translation_speed>1.0, average_translation_speed
    print("PASS preset=%d id=%d actions=%d modes=%s points=%d spin_pairs=%d duration=%.2fs avg_translation=%.2fm/s status=%s search=%s corridor_markers=%d colored_mode_runs=%d"%
          (args.preset,seed.plan_id,len(seed.primitives),sorted(modes),len(pts),spin_pairs,
           pts[-1].time_from_start.to_sec(),average_translation_speed,
           data["status"],data["search"],len(env.markers),len(colored)))


if __name__=="__main__":
    main()
