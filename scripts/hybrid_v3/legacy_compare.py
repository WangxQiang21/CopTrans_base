"""Execute the pre-V3 single portfolio and uniform_v02 against the frozen V3 requests.

Run from a sourced ROS workspace, in a free, private ROS_DOMAIN_ID. The single
portfolio includes its native seed challenger. Only map/start/goal topics are
published. The formation planner receives the same request in process.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
import yaml
from cases import LOCAL, POLICY, REPORT, ROOT, digest, indices, make, packed


def occupancy(grid):
    cells=np.zeros(grid["height"]*grid["width"],dtype=np.uint8)
    for off,count in grid["occupied_runs"]:
        cells[off:off+count]=100
    return cells.reshape(grid["height"],grid["width"])


def formation(r,budget):
    from fwis_backends.multi_4wis.core import Grid
    from fwis_backends.multi_4wis.formation import FormationModel, FormationPose, VehicleOffset
    from fwis_backends.multi_4wis.formation_collision import CargoFootprint, VehicleShape
    from fwis_backends.multi_4wis.formation_retiming import FormationTimingLimits
    from fwis_backends.multi_4wis.uniform_search import GraphSearchConfig, SuccessorConfig
    from fwis_backends.multi_4wis.uniform_pipeline import UniformPlanningRequest, plan_uniform
    g=r["grid"]; l=r["limits"]; c=r["cargo"]
    grid=Grid(occupancy(g)!=0,g["resolution"],*g["origin"])
    model=FormationModel([VehicleOffset(i,*f[:2]) for i,f in enumerate(r["vehicles"],1)])
    shapes={i:VehicleShape(*f[2:]) for i,f in enumerate(r["vehicles"],1)}
    limits=FormationTimingLimits(v_max=l["speed"],a_max=l["acceleration"],jerk_max=l["jerk"],
        yaw_rate_max=l["yaw_rate"],yaw_accel_max=l["yaw_acceleration"],yaw_jerk_max=l["yaw_jerk"],
        wheel_speed_max=l["wheel_speed"],steer_angle_max=l["steer_angle"],steer_rate_max=l["steer_rate"],
        steer_accel_max=l["steer_acceleration"],lateral_accel_max=l["lateral_acceleration"],
        wheelbase=l["wheelbase"],track=l["track"],r_min_ackermann=l["radius"],mode_switch_duration=l["prepare_time"])
    free=not all(r["goal"]["constrain_heading"])
    succ=SuccessorConfig(allow_reverse=r["allow_reverse"],collision_margin_m=0.,
        goal_heading_free=free,goal_formation_heading_free=not r["goal"]["constrain_reference_yaw"],timing_limits=limits)
    config=GraphSearchConfig(direct_goal_translation=True,max_seconds=budget,prefer_ackermann=True,
        native_frontier=True,convex_corridor=True,continuous_a_refinement=False)
    req=UniformPlanningRequest(grid,model,shapes,FormationPose(*r["start"]["reference"]),
        dict(enumerate(r["start"]["heading"],1)),FormationPose(*r["goal"]["reference"]),
        None if free else dict(enumerate(r["goal"]["heading"],1)),cargo_footprint=CargoFootprint(*c[2:],*c[:2]),
        successor_config=succ,search_config=config,reachability_retime=True,reuse_terminal_certificate=True)
    t=time.monotonic()
    result=plan_uniform(req)
    elapsed=time.monotonic()-t
    out=dict(solved=result.executable,status=result.status,reason=result.reason,wall_s=elapsed)
    if result.executable:
        out.update(duration_s=result.plan.duration_s,model_digest=result.plan.model_sha256(),
            modes=sorted(set(ph.mode for ph in result.plan.phases if ph.mode in {"A","O","S"})))
    return out


class PreviousPortfolio:
    def __init__(self,path,r,budget):
        import rclpy
        from rclpy.qos import DurabilityPolicy,QoSProfile
        from nav_msgs.msg import OccupancyGrid
        from geometry_msgs.msg import PoseStamped
        from fwis_v3_msgs.msg import TimedTrajectory
        from std_msgs.msg import String
        self.ros=rclpy
        rclpy.init()
        self.node=rclpy.create_node("legacy_hybrid_v3_benchmark")
        q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map=self.node.create_publisher(OccupancyGrid,"/map",q)
        self.start=self.node.create_publisher(PoseStamped,"/start_pose",1)
        self.goal=self.node.create_publisher(PoseStamped,"/move_base_simple/goal",1)
        self.status=[];self.trajectories={}
        self.node.create_subscription(String,"/plan/status",lambda m:self.status.append((time.monotonic(),m.data)),q)
        self.node.create_subscription(TimedTrajectory,"/plan/timed_trajectory",lambda m:self.trajectories.update({m.plan_id:m}),q)
        path.mkdir(parents=True,exist_ok=True)
        p=path/"params.yaml";l=r["limits"]
        # Grid in this old single backend adds 'margin' once to length/width.
        # Pass the already-inflated footprint at zero additional margin for the
        # very same per-side clearance as the frozen native request.
        settings={"vehicle":dict(length=r["vehicles"][0][2],width=r["vehicles"][0][3],safety_margin=0.,
                      r_min_ack=l["radius"],wheelbase=l["wheelbase"],track=l["track"],mode_switch_duration=l["prepare_time"]),
                  "quality":{"limits":dict(translation_speed_policy="restricted",target_v=l["speed"],v_max=l["speed"],
                      a_max=l["acceleration"],jerk_max=l["jerk"],omega_max=l["spin_speed"],alpha_max=l["spin_acceleration"],
                      angular_jerk_max=l["spin_jerk"],translation_omega_max=l["yaw_rate"],translation_alpha_max=l["yaw_acceleration"],
                      translation_angular_jerk_max=l["yaw_jerk"],wheel_speed_max=l["wheel_speed"],lateral_a_max=l["lateral_acceleration"],
                      steer_angle_max=l["steer_angle"],steer_rate_max=l["steer_rate"],steer_accel_max=l["steer_acceleration"])},
                  "lattice":{"contest_legacy":True,"contest_window_s":budget,"time_budget_s":budget}}
        p.write_text(yaml.safe_dump(settings))
        self.log=(path/"planner.log").open("a")
        self.process=subprocess.Popen(["ros2","launch","fwis_bringup","plan.launch.py","backend:=single_state_lattice",
            "rviz:=false","transport_ui:=false","overrides_file:="+str(p)],stdout=self.log,stderr=self.log,start_new_session=True)
        self.wait(lambda:self.map.get_subscription_count()>=2 and self.goal.get_subscription_count()>=1,20)
    def wait(self,done,timeout):
        end=time.monotonic()+timeout
        while not done() and time.monotonic()<end and self.process.poll() is None:
            self.ros.spin_once(self.node,timeout_sec=.01)
        return done()
    def pose(self,state):
        from geometry_msgs.msg import PoseStamped
        msg=PoseStamped();msg.header.frame_id="map"; msg.header.stamp=self.node.get_clock().now().to_msg()
        x,y,heading=state["reference"]
        msg.pose.position.x=float(x);msg.pose.position.y=float(y)
        msg.pose.orientation.w=math.cos(heading/2);msg.pose.orientation.z=math.sin(heading/2)
        return msg
    def run(self,r,budget):
        from nav_msgs.msg import OccupancyGrid
        from array import array
        g=r["grid"];m=OccupancyGrid();m.header.frame_id="map"; m.info.width=g["width"];m.info.height=g["height"]
        m.info.resolution=g["resolution"]; m.info.origin.position.x=g["origin"][0];m.info.origin.position.y=g["origin"][1]
        m.info.origin.orientation.w=1.
        m.data=array("b",occupancy(g).ravel().tobytes())
        self.status.clear();self.trajectories.clear()
        for _ in range(3):
            self.map.publish(m)
            if self.wait(lambda:any(s.startswith("map_ready") for _,s in self.status),5):
                break
        else:
            raise RuntimeError("legacy planner did not acknowledge map")
        self.start.publish(self.pose(r["start"]))
        self.wait(lambda:False,.08)
        self.status.clear();self.trajectories.clear()
        start=time.monotonic();self.goal.publish(self.pose(r["goal"]))
        if not self.wait(lambda:any(s.startswith(("success ","search_no_path ","rejected ")) for _,s in self.status),budget+3):
            return dict(solved=False,status="TIMEOUT",wall_s=time.monotonic()-start,reason="no terminal planner result")
        ended,status=next((t,s) for t,s in self.status if s.startswith(("success ","search_no_path ","rejected ")))
        out=dict(solved=False,status=status.split()[0],reason=status,wall_s=ended-start)
        if status.startswith("success "):
            pid=int(re.search(r"id=(\d+)",status).group(1))
            if not self.wait(lambda:pid in self.trajectories and bool(self.trajectories[pid].points),2):
                return dict(out,reason="missing matching certified projection")
            pts=self.trajectories[pid].points
            for point,bound in [(pts[0],r["start"]),(pts[-1],r["goal"])]:
                x,y,th=bound["reference"];p=point.center_pose
                if math.hypot(x-p.x,y-p.y)>.02 or abs(math.remainder(th-p.theta,2*math.pi))>math.pi/180:
                    return dict(out,reason="projection endpoint disagrees with request")
            last=pts[-1].time_from_start
            out.update(solved=True,modes=sorted({"AOS"[p.mode] for p in pts if abs(p.v)>1e-7}),
                duration_s=last.sec+last.nanosec/1e9,source=re.search(r"source=(\S+)",status).group(1))
        return out
    def close(self):
        if self.process.poll() is None:
            os.killpg(self.process.pid,signal.SIGINT)
            try:self.process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid,signal.SIGTERM); self.process.wait(timeout=5)
        self.log.close();self.node.destroy_node();self.ros.shutdown()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--task",choices=["single","formation"],required=True)
    ap.add_argument("--profile",choices=["baseline","standard"],default="standard")
    ap.add_argument("--split",choices=["core","development","holdout"],default="holdout")
    ap.add_argument("--ids");ap.add_argument("--output",type=Path,required=True)
    ap.add_argument("--timing",action="store_true",help="five warmups and thirty serial timed trials, C01 and C06")
    ap.add_argument("--cohort",type=int,default=0);ap.add_argument("--manifest",type=Path,default=REPORT/"case_manifest.json");a=ap.parse_args()
    from cases import MASTER_SEEDS
    MASTER_SEEDS["holdout"]+=a.cohort*1000003
    ids=[int(s) for s in a.ids.split(",")] if a.ids else range(len(indices(a.split)))
    if a.timing and not a.ids: ids=[indices(a.split).index(family) for family in (1,6)]
    trials=[(i,rep) for i in ids for rep in (range(-5,30) if a.timing else (0,))]
    budget=float(POLICY["budget_"+a.task]);a.output.parent.mkdir(parents=True,exist_ok=True)
    if a.task=="single" and not (os.getenv("ROS_LOCALHOST_ONLY")=="1" and os.getenv("ROS_DOMAIN_ID")):
        ap.error("single comparison requires a private ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")
    manifest=a.manifest
    if a.split=="holdout" and not manifest.exists(): raise SystemExit("freeze the holdout manifest first")
    doc=json.loads(manifest.read_text()) if manifest.exists() else {}
    if a.split=="holdout" and doc.get("cohort")!=a.cohort:
        raise SystemExit("Wrong frozen manifest cohort")
    frozen={x["id"]:x["request"] for x in doc.get("cases",[])}
    source_sha=subprocess.check_output(["git","rev-parse","HEAD"],cwd=ROOT,text=True).strip()
    service=None
    try:
        with a.output.open("x") as out:
            for i,repeat in trials:
                r,_,meta=make(a.split,i,a.task,a.profile)
                case_id=f"{a.split}/{a.task}/{i:03}"
                if a.split=="holdout" and frozen.get(case_id)!=digest(r):
                    raise RuntimeError("frozen request mismatch")
                if a.task=="single" and service is None:
                    service=PreviousPortfolio(a.output.with_suffix(".ros"),r,budget)
                try: result=service.run(r,budget) if service else formation(r,budget)
                except (ValueError,RuntimeError) as exc:
                    result=dict(solved=False,status="ERROR",reason=str(exc),wall_s=None)
                result["within_budget"]=result["wall_s"] is not None and result["wall_s"]<=budget
                result["certified_in_budget"]=bool(result["solved"] and result["within_budget"]
                    and result.get("duration_s",math.inf)<=r["task_duration_cap"])
                row=dict(case_id=case_id,cohort=a.cohort,task=a.task,profile=a.profile,budget_s=budget,source_sha=source_sha,
                    repeat=repeat,operation="timing" if a.timing else "solve",cpus=sorted(os.sched_getaffinity(0)),
                    backend="single_state_lattice_with_continuous_portfolio" if service else "uniform_v02_optimized",
                    request_hash=digest(r),map_hash=digest(r["grid"]),limits_hash=digest(r["limits"]),**meta,evidence=result)
                out.write(packed(row).decode()+"\n");out.flush()
    finally:
        if service:service.close()

if __name__=="__main__":main()
