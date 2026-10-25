#!/usr/bin/env python3
"""Deterministic ideal-kinematics sanity simulations and map collision probes."""
from __future__ import annotations

import csv
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE / "src"))
from spatial_model import Pose, RasterWorld, Vehicle, propagate


def main():
    vehicle = Vehicle()
    checks = []
    def check(name, pose, expected, xy_tol=1e-9, yaw_tol=1e-9):
        xy_error = math.hypot(pose.x-expected.x, pose.y-expected.y)
        yaw_error = abs((pose.yaw-expected.yaw+math.pi)%(2*math.pi)-math.pi)
        checks.append({"case":name,"status":"pass" if xy_error<=xy_tol and yaw_error<=yaw_tol else "fail",
                       "xy_error_m":xy_error,"yaw_error_rad":yaw_error,
                       "x":pose.x,"y":pose.y,"yaw":pose.yaw})

    p = Pose(0,0,0)
    check("ackermann_straight", propagate(p,"A",1,0,vehicle), Pose(1,0,0))
    k = 1/vehicle.min_turn_radius
    turn = propagate(p,"A",.25,k,vehicle)
    expected_yaw = .25*k
    check("ackermann_min_radius",turn,
          Pose(math.sin(expected_yaw)/k,(1-math.cos(expected_yaw))/k,expected_yaw))
    check("oblique_45_fixed_heading",propagate(p,"P",1,math.pi/4,vehicle),
          Pose(math.sqrt(.5),math.sqrt(.5),0))
    check("lateral_left_fixed_heading",propagate(p,"L",1,math.pi/2,vehicle),Pose(0,1,0))
    check("spin_fixed_position",propagate(Pose(2,3,.2),"S",math.pi/6,0,vehicle),Pose(2,3,.2+math.pi/6))

    world = RasterWorld(str(HERE.parents[2] / "maps/S09_underground_garage/map.yaml"),vehicle)
    probes = [("start_pose",Pose(13.2,2.25,math.pi),True),
              ("bottom_slot_goal",Pose(5.86,.5,-math.pi/2),True),
              ("preset_mid_a_is_occupied",Pose(7.25,4.0,math.pi/2),False),
              ("overlap_adjacent_parked_vehicle",Pose(5.30,.5,-math.pi/2),False)]
    for name,pose,expected in probes:
        actual=world.pose_clear(pose)
        checks.append({"case":f"collision_{name}","status":"pass" if actual==expected else "fail",
                       "expected_clear":expected,"actual_clear":actual})

    out=HERE/"results"/"summary"; out.mkdir(parents=True,exist_ok=True)
    with (out/"spatial_model_checks.csv").open("w",newline="") as f:
        fields=sorted(set().union(*(r.keys() for r in checks)))
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(checks)
    for row in checks: print(row)
    if any(r["status"]!="pass" for r in checks): raise SystemExit(1)


if __name__=="__main__": main()
