#!/usr/bin/env python3
"""ROS adapter: exact seed replay -> STC/IPOPT -> SAT -> timed center trajectory."""
import math
import hashlib
import json
import os
import re
import tempfile
import threading
import time
import traceback
from dataclasses import asdict

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray
from fwis_v3_msgs.msg import (PlanSeed, PlanSeedCandidates, TimedPoint,
                              TimedTrajectory)
from tf.transformations import quaternion_from_euler

from fwis_v3_optimizer.core import (Pose, Action, Grid, refine, time_sample,
                                    bounded_research, joint_refine,
                                    sequence_cost, anchor_mode_sequences,
                                    iterated_mode_geometry_refine,
                                    expand, replay, collapse_exact_runs,
                                    build_corridors, corridor_box_corners, footprint_corners,
                                    select_corridor_display_stations,
                                    fit_action_to_pose,
                                    mode_schedule_neighbor_candidates,
                                    _collapse_for_joint,
                                    _joint_seed_mode_gear_schedule,
                                    terminal_pose_graph_candidates,
                                    upstream_sos_replan_candidates,
                                    upstream_suffix_replan_candidates, wrap,
                                    has_reverse_motion)
from fwis_v3_optimizer.mode_master import ModeMasterConfig, ModePathMaster
from fwis_v3_optimizer.quality import (QualityProfile, TrajectoryEvaluator,
                                       action_shunting_violation)
from fwis_v3_optimizer.retiming import MotionLimits
from fwis_v3_optimizer.continuous_geometry import replay_continuous
from fwis_v3_optimizer.diagnostics import atomic_write_json


def pose_stamped(p, stamp):
    out = PoseStamped()
    out.header.frame_id = "map"
    out.header.stamp = stamp
    out.pose.position.x, out.pose.position.y = p.x, p.y
    q = quaternion_from_euler(0, 0, p.th)
    out.pose.orientation.x, out.pose.orientation.y = q[0], q[1]
    out.pose.orientation.z, out.pose.orientation.w = q[2], q[3]
    return out


def make_mode_strip(actions, poses, header, namespace, width, alpha, colors):
    erase=Marker();erase.header=header;erase.action=Marker.DELETEALL
    markers=[erase]
    i=0;marker_id=0
    while i<len(actions):
        mode=actions[i].mode
        j=i+1
        while j<len(actions) and actions[j].mode==mode: j+=1
        marker=Marker()
        marker.header=header;marker.ns=namespace;marker.id=marker_id;marker_id+=1
        marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD
        marker.pose.orientation.w=1.0
        marker.scale.x=width
        marker.color.r,marker.color.g,marker.color.b=colors[mode]
        marker.color.a=alpha
        if mode==2:
            p,q=poses[i],poses[j]
            turn=math.atan2(math.sin(q.th-p.th),math.cos(q.th-p.th))
            count=max(3,math.ceil(abs(turn)/math.radians(5)))
            radius=.14
            for n in range(count+1):
                angle=p.th+turn*n/count
                marker.points.append(Point(x=p.x+radius*math.cos(angle),
                                           y=p.y+radius*math.sin(angle),z=.075))
        else:
            marker.points=[Point(x=p.x,y=p.y,z=.075) for p in poses[i:j+1]]
        markers.append(marker)
        i=j
    return MarkerArray(markers=markers)


def make_timed_mode_strip(timed, header, namespace, width, alpha, colors):
    """Draw mode runs from the actual continuous timed trajectory samples."""
    erase=Marker();erase.header=header;erase.action=Marker.DELETEALL
    markers=[erase]
    i=0;marker_id=0
    while i<len(timed):
        mode=timed[i][3]
        j=i+1
        while j<len(timed) and timed[j][3]==mode: j+=1
        segment=timed[i:j]
        marker=Marker()
        marker.header=header;marker.ns=namespace;marker.id=marker_id;marker_id+=1
        marker.type=Marker.LINE_STRIP;marker.action=Marker.ADD
        marker.pose.orientation.w=1.0;marker.scale.x=width
        marker.color.r,marker.color.g,marker.color.b=colors[mode]
        marker.color.a=alpha
        if mode==2 and len(segment)>1:
            p,q=segment[0][1],segment[-1][1]
            turn=q.th-p.th
            count=max(3,math.ceil(abs(turn)/math.radians(5)))
            radius=.14
            for n in range(count+1):
                angle=p.th+turn*n/count
                marker.points.append(Point(x=p.x+radius*math.cos(angle),
                                           y=p.y+radius*math.sin(angle),z=.075))
        else:
            marker.points=[Point(x=row[1].x,y=row[1].y,z=.075) for row in segment]
        if marker.points:
            markers.append(marker)
        i=j
    return MarkerArray(markers=markers)


def safe_time_sample(start, actions, grid, v_demo, omega_demo, dt,
                     sweep_corner_step=.0025):
    """Refine temporal sampling until linear interpolation remains SAT-clear."""
    last_collision=False
    for level in range(7):
        sample_dt=dt/(2**level)
        timed=time_sample(start,actions,v_demo,omega_demo,sample_dt)
        if any(t2<=t1 for (t1,_,_,_), (t2,_,_,_) in zip(timed,timed[1:])):
            raise ValueError("non_increasing_timestamps")
        if all(grid.interpolated_clear(p1,p2,sweep_corner_step)
               for (_,p1,_,_),(_,p2,_,_) in zip(timed,timed[1:])):
            return timed,sample_dt
        last_collision=True
    if last_collision:
        raise ValueError("timed_sat_collision_after_adaptive_sampling")
    raise ValueError("empty_timed_trajectory")


def legacy_direction_allowed(actions, allow_reverse, allow_shunting):
    """Apply the configured gear policy and shared Spin-mediated reversal rule."""
    if has_reverse_motion(actions) and not allow_reverse:
        return False
    return allow_shunting or action_shunting_violation(actions) is None


def refine_mode_schedule_neighbors(start, goal, base_actions, grid,
                                   xy_tol, yaw_tol, max_candidates=2,
                                   max_window=6, time_limit=2.0,
                                   max_translation=3.0, max_knots=18,
                                   terminal_xy_tol=.015,
                                   terminal_yaw_tol=.02,
                                   terminal_xy_weight=100.0,
                                   terminal_yaw_weight=10.0,
                                   weights=None, collision_spacing=.75,
                                   adaptive_collision_refinement=False,
                                   corridor_clearance_slack=0.0,
                                   schedule_registry=None,
                                   profile=None,
                                   ackermann_merge_tolerance=1e-5,
                                   oblique_merge_tolerance=math.radians(.25),
                                   refine_base_schedule=True):
    """Refine a few exact swept-safe A/O schedule substitutions.

    The mode proposals are generated from a safe local incumbent and then
    solved as fixed-schedule continuous NLPs. This stage does not depend on
    Bonmin producing an integer incumbent.
    """
    base_score=sequence_cost(base_actions,weights)["score"]
    refined=[];rejected=[];bases=[("local",base_actions)]
    seen=set()
    if refine_base_schedule:
        fixed_begin=time.perf_counter()
        try:
            joint_actions,_=_collapse_for_joint(
                start,base_actions,max_translation,
                ackermann_merge_tolerance=ackermann_merge_tolerance,
                oblique_merge_tolerance=oblique_merge_tolerance)
            _,schedule=_joint_seed_mode_gear_schedule(joint_actions)
            key=canonical_joint_schedule_key(
                start,joint_actions,max_translation=max_translation,
                ackermann_merge_tolerance=ackermann_merge_tolerance,
                oblique_merge_tolerance=oblique_merge_tolerance)
            if schedule_registry is not None and key is not None:
                schedule_registry.add(key)
            poses,actions,boxes,meta=joint_refine(
                start,goal,base_actions,grid,xy_tol,yaw_tol,
                max_translation=max_translation,
                max_knots=max(max_knots,len(schedule)),time_limit=time_limit,
                terminal_xy_tol=terminal_xy_tol,
                terminal_yaw_tol=terminal_yaw_tol,
                terminal_xy_weight=terminal_xy_weight,
                terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                collision_spacing=collision_spacing,
                adaptive_collision_refinement=(
                    adaptive_collision_refinement),
                corridor_clearance_slack=corridor_clearance_slack,
                use_feasible_seed_warm_start=False,collision_retries=0,
                collision_repair_iterations=0,_fixed_schedule=schedule,
                _initial_actions=joint_actions)
            fixed_score=sequence_cost(actions,weights)["score"]
            if fixed_score<base_score-1e-6:
                meta["solver"]="ipopt_local_fixed_schedule"
                meta["fallback_reason"]="fixed_mode_schedule_refine"
                meta["bonmin_status"]="NOT_RUN_fixed_schedule"
                meta["local_fixed_schedule_refine"]=True
                meta["common_score"]=float(fixed_score)
                refined.append((fixed_score,"fixed_schedule_refine",poses,
                                actions,boxes,meta))
            bases.insert(0,("fixed_schedule",actions))
        except Exception as exc:
            rejected.append(({"first":0,"last":len(base_actions)},
                             "fixed_schedule_refine:"+
                             str(exc).splitlines()[-1]))
        if profile is not None:
            profile["pre_neighbor_base_schedule_s"]=(
                time.perf_counter()-fixed_begin)
    elif profile is not None:
        profile["pre_neighbor_base_schedule_s"]=0.0

    proposals=[]
    for base_name,actions in bases:
        for proposal in mode_schedule_neighbor_candidates(
                start,actions,grid,max_candidates=max_candidates,
                max_window=max_window,weights=weights,sweep_step=.005):
            key=canonical_joint_schedule_key(
                start,proposal[1],max_translation=max_translation,
                ackermann_merge_tolerance=ackermann_merge_tolerance,
                oblique_merge_tolerance=oblique_merge_tolerance)
            if key is None or key in seen:
                continue
            seen.add(key)
            proposals.append((proposal[0],proposal[1],proposal[2],
                              dict(proposal[3],base=base_name),key))
    proposals.sort(key=lambda item:item[0])
    if profile is not None:
        profile["pre_neighbor_schedules_generated"]=(
            profile.get("pre_neighbor_schedules_generated",0)+len(proposals))
    proposals=proposals[:int(max_candidates)]
    best_score=min([base_score]+[item[0] for item in refined])
    for (seed_score,seed_actions,_,neighbor_info,proposal_key) in proposals:
        try:
            if profile is not None:
                profile["pre_neighbor_schedules_solved"]=(
                    profile.get("pre_neighbor_schedules_solved",0)+1)
            if schedule_registry is not None:
                schedule_registry.add(proposal_key)
            joint_actions,_=_collapse_for_joint(
                start,seed_actions,max_translation,
                ackermann_merge_tolerance=ackermann_merge_tolerance,
                oblique_merge_tolerance=oblique_merge_tolerance)
            _,schedule=_joint_seed_mode_gear_schedule(joint_actions)
            poses,actions,boxes,meta=joint_refine(
                start,goal,seed_actions,grid,xy_tol,yaw_tol,
                max_translation=max_translation,
                max_knots=max(max_knots,len(schedule)),
                time_limit=time_limit,terminal_xy_tol=terminal_xy_tol,
                terminal_yaw_tol=terminal_yaw_tol,
                terminal_xy_weight=terminal_xy_weight,
                terminal_yaw_weight=terminal_yaw_weight,weights=weights,
                collision_spacing=collision_spacing,
                adaptive_collision_refinement=(
                    adaptive_collision_refinement),
                corridor_clearance_slack=corridor_clearance_slack,
                use_feasible_seed_warm_start=False,collision_retries=0,
                collision_repair_iterations=0,_fixed_schedule=schedule,
                _initial_actions=joint_actions)
            score=sequence_cost(actions,weights)["score"]
            if score>=best_score-1e-6:
                continue
            meta["solver"]="ipopt_mode_neighborhood"
            meta["fallback_reason"]="fixed_schedule_mode_neighbor"
            meta["bonmin_status"]="NOT_RUN_fixed_schedule_neighbor"
            meta["bonmin_rins_heuristic_enabled"]=False
            meta["mode_neighbor"]=neighbor_info
            meta["mode_neighbor_seed_score"]=float(seed_score)
            meta["common_score"]=float(score)
            refined.append((score,"mode_neighborhood",poses,actions,boxes,meta))
            if profile is not None:
                profile["pre_neighbor_schedules_accepted"]=(
                    profile.get("pre_neighbor_schedules_accepted",0)+1)
            best_score=score
        except Exception as exc:
            rejected.append((neighbor_info,str(exc).splitlines()[-1]))
    return refined,rejected


def canonical_joint_schedule_key(start, actions, max_translation=3.0,
                                 max_spin=math.pi,
                                 ackermann_merge_tolerance=1e-5,
                                 oblique_merge_tolerance=math.radians(.25)):
    """Stable key for a fixed MINLP mode/gear schedule and its knot layout."""
    if not actions:
        return None
    collapsed,_=_collapse_for_joint(
        start,actions,max_translation=max_translation,max_spin=max_spin,
        ackermann_merge_tolerance=ackermann_merge_tolerance,
        oblique_merge_tolerance=oblique_merge_tolerance)
    if not collapsed:
        return None
    _,schedule=_joint_seed_mode_gear_schedule(collapsed)
    boundary=0.0;key=[]
    for action,(mode,gear) in zip(collapsed,schedule):
        amount=abs(float(action.amount))
        control=float(action.control)
        if mode==1:
            control=wrap(control)
        # Quantized cumulative boundaries (5 cm), signed primitive direction,
        # and control (0.01 units) distinguish genuinely different fixed NLPs
        # while collapsing tiny changes from replay/solver roundoff.
        key.append((int(mode),int(gear),
                    1 if action.amount>=0 else -1,
                    int(round(amount*20.0)),
                    int(round(control*100.0)),
                    int(round(boundary*20.0))))
        if mode!=2:
            boundary+=amount
    return tuple(key)


def _mode_complexity(actions):
    """Return mode/steering complexity components used by WP2 scheduling."""
    events=collapse_exact_runs(actions)
    mode_switches=sum(a.mode!=b.mode for a,b in zip(events,events[1:]))
    oblique_updates=sum(
        a.mode==1 and b.mode==1 and
        abs(wrap(b.control-a.control))>math.radians(1.0)
        for a,b in zip(events,events[1:]))
    spin_runs=sum(a.mode==2 for a in events)
    return (int(mode_switches),int(oblique_updates),int(spin_runs))


class OptimizerNode:
    def __init__(self):
        self.grid = None
        self.plan_id = 0
        self._frontend_failures = {}
        self._seed_proposal_condition = threading.Condition()
        self._seed_proposals = {}
        self.length = rospy.get_param("~vehicle/length", .738)
        self.width = rospy.get_param("~vehicle/width", .5)
        self.margin = rospy.get_param("~collision/margin", .04)
        self.vehicle_corner = math.hypot(
            (self.length+self.margin)/2,(self.width+self.margin)/2)
        self.timed_sweep_corner_step = rospy.get_param(
            "~collision/timed_sweep_corner_step", .0025)
        self.v_demo = rospy.get_param("~time/v_demo", .5)
        self.omega_demo = rospy.get_param("~time/omega_demo", .5)
        self.dt = rospy.get_param("~time/dt", .1)
        self.xy_tol = rospy.get_param("~search/goal_xy_tol", .16)
        self.yaw_tol = rospy.get_param("~search/goal_yaw_tol", .14)
        self.planning_backend = rospy.get_param("~planning/backend", "legacy")
        if self.planning_backend not in ("legacy", "mode_path_v1", "mode_path_v2"):
            raise ValueError("unknown_planning_backend")
        project_root = os.path.abspath(os.path.join(
            os.path.dirname(__file__), "../../../.."))
        self.project_root = project_root
        default_report_dir = os.path.join(
            project_root, "relust/R12_planning_reliability_20261012/reports")
        self.report_dir = os.path.abspath(rospy.get_param(
            "~quality/report_dir", default_report_dir))
        direction_policy = rospy.get_param("~planning/direction_policy", None)
        legacy_reverse = rospy.get_param("~planning/allow_reverse_fallback", None)
        if direction_policy is not None and direction_policy not in (
                "bidirectional", "forward_only"):
            raise ValueError("unknown_direction_policy:%s" % direction_policy)
        if legacy_reverse is not None and not isinstance(legacy_reverse, bool):
            raise ValueError("legacy_reverse_policy_requires_boolean")
        if direction_policy is None:
            if legacy_reverse is None:
                direction_policy = "bidirectional"
            else:
                direction_policy = ("bidirectional" if legacy_reverse
                                    else "forward_only")
                rospy.logwarn("planning/allow_reverse_fallback is deprecated; "
                              "use planning/direction_policy=%s", direction_policy)
        elif legacy_reverse is not None:
            legacy_policy = ("bidirectional" if legacy_reverse else "forward_only")
            if legacy_policy != direction_policy:
                raise ValueError("conflicting_direction_policy_and_legacy_reverse")
            rospy.logwarn("planning/allow_reverse_fallback is deprecated; "
                          "use planning/direction_policy=%s", direction_policy)
        self.direction_policy = direction_policy
        self.allow_reverse = direction_policy == "bidirectional"
        self.allow_shunting = rospy.get_param("~planning/allow_shunting",False)
        self.show_feature_center_regions = rospy.get_param(
            "~visualization/show_feature_center_regions",False)
        limits_config = dict(rospy.get_param("~quality/limits", {}))
        for name, default in (("wheelbase", .494), ("track", .364), ("r_min_ack", .810)):
            limits_config[name] = float(rospy.get_param("~vehicle/"+name, default))
        self.quality_limits = MotionLimits(**limits_config)
        self.quality_profile = QualityProfile(**rospy.get_param("~quality/cost", {}))
        master_config = dict(rospy.get_param("~quality/search", {}))
        master_config["allow_reverse"] = self.allow_reverse
        master_config["allow_shunting"] = self.allow_shunting
        master_config["continuous_motion_v2"] = self.planning_backend == "mode_path_v2"
        self.mode_master_config = ModeMasterConfig(**master_config)
        self.repair_max_iter = rospy.get_param("~repair/max_iter", 5000)
        self.repair_padding = rospy.get_param("~repair/padding", .8)
        self.endpoint_replan_iter = rospy.get_param("~endpoint_replan/max_iter", 80)
        self.endpoint_replan_tail = rospy.get_param("~endpoint_replan/tail_length", 4.0)
        self.endpoint_sos_enabled = rospy.get_param(
            "~endpoint_replan/sos_enabled", True)
        self.endpoint_sos_min_hotspot_score = max(0.0,float(
            rospy.get_param("~endpoint_replan/sos_min_hotspot_score",1.0)))
        self.endpoint_sos_hotspot_lookahead = max(.5,float(rospy.get_param(
            "~endpoint_replan/sos_hotspot_lookahead",8.0)))
        self.endpoint_sos_hotspot_clearance = float(rospy.get_param(
            "~endpoint_replan/sos_hotspot_clearance",
            self.vehicle_corner+.10))
        self.endpoint_sos_upstream_clearance = float(rospy.get_param(
            "~endpoint_replan/sos_upstream_clearance",
            self.vehicle_corner+.065))
        self.endpoint_sos_replan_max_searches = max(0,int(rospy.get_param(
            "~endpoint_replan/sos_replan_max_searches",3)))
        self.endpoint_sos_upstream_windows = max(1,int(rospy.get_param(
            "~endpoint_replan/sos_upstream_windows",2)))
        self.endpoint_sos_max_iter = rospy.get_param(
            "~endpoint_replan/sos_max_iter", 800)
        self.endpoint_sos_upstream_distance = rospy.get_param(
            "~endpoint_replan/sos_upstream_distance", 8.0)
        self.endpoint_sos_score_slack = rospy.get_param(
            "~endpoint_replan/sos_score_slack", 1.1)
        self.endpoint_sos_switch_bonus = rospy.get_param(
            "~endpoint_replan/sos_switch_bonus", 1.0)
        self.optimizer_max_iter = rospy.get_param("~optimization/max_iter", 100)
        self.joint_enabled = rospy.get_param("~joint/enabled", False)
        self.joint_minlp_seed_source = str(rospy.get_param(
            "~joint/minlp_seed_source", "best_local"))
        self.joint_max_knots = rospy.get_param("~joint/max_knots", 24)
        self.joint_time_limit = rospy.get_param("~joint/time_limit", 1.5)
        self.joint_adaptive_time_limit = rospy.get_param(
            "~joint/adaptive_time_limit", True)
        self.joint_single_switch_time_limit = max(
            float(self.joint_time_limit),float(rospy.get_param(
                "~joint/single_switch_time_limit",4.0)))
        self.joint_single_switch_max_knots = max(1,int(rospy.get_param(
            "~joint/single_switch_max_knots",10)))
        self.joint_algorithm = rospy.get_param("~joint/algorithm", "B-BB")
        self.joint_variable_selection = rospy.get_param(
            "~joint/variable_selection", "most-fractional")
        self.joint_node_comparison = rospy.get_param(
            "~joint/node_comparison", "depth-first")
        self.joint_subproblem_time_limit = rospy.get_param(
            "~joint/subproblem_time_limit", 3.0)
        self.joint_subproblem_time_adaptive = rospy.get_param(
            "~joint/subproblem_time_adaptive", True)
        self.joint_subproblem_multi_switch_limit = max(.05,float(
            rospy.get_param("~joint/subproblem_time_multi_switch_limit",.75)))
        self.joint_bonmin_iteration_limit = rospy.get_param(
            "~joint/bonmin_iteration_limit", 0)
        self.joint_bonmin_subproblem_iteration_limit = rospy.get_param(
            "~joint/bonmin_subproblem_iteration_limit", 600)
        self.joint_bonmin_unsolved_nlp_retries = rospy.get_param(
            "~joint/bonmin_unsolved_nlp_retries", 0)
        self.joint_seed_cutoff = rospy.get_param("~joint/seed_cutoff", True)
        self.joint_use_cbc_seed_incumbent = rospy.get_param(
            "~joint/use_cbc_seed_incumbent", False)
        self.joint_rins_heuristic = rospy.get_param(
            "~joint/heuristic_rins", False)
        self.joint_mode_neighbor_candidates = max(0,int(rospy.get_param(
            "~joint/mode_neighborhood_candidates", 1)))
        self.joint_mode_neighbor_window = max(1,int(rospy.get_param(
            "~joint/mode_neighborhood_window", 6)))
        self.joint_mode_neighbor_time_limit = max(.1,float(rospy.get_param(
            "~joint/mode_neighborhood_time_limit", 1.0)))
        self.joint_mode_neighborhood_seed_hint = bool(rospy.get_param(
            "~joint/mode_neighborhood_seed_hint", False))
        self.joint_post_neighbor_skip_improvement = max(0.0,min(.05,float(
            rospy.get_param(
                "~joint/post_neighborhood_skip_relative_improvement",.005))))
        self.joint_global_rounds = max(0,min(2,int(rospy.get_param(
            "~joint/global_rounds", 0))))
        self.joint_collision_retries = rospy.get_param(
            "~joint/collision_retries", 0)
        self.joint_collision_repair_iterations = rospy.get_param(
            "~joint/collision_repair_iterations", 0)
        self.joint_collision_repair_time_limit = rospy.get_param(
            "~joint/collision_repair_time_limit", 2.0)
        self.joint_warm_start_time_limit = rospy.get_param(
            "~joint/warm_start_time_limit", 6.0)
        self.joint_warm_start_early_accept_improvement = rospy.get_param(
            "~joint/warm_start_early_accept_improvement", 0.0)
        self.joint_collision_spacing = rospy.get_param(
            "~joint/collision_spacing", .75)
        self.joint_adaptive_collision_refinement = rospy.get_param(
            "~joint/adaptive_collision_refinement", False)
        self.joint_corridor_clearance_slack = rospy.get_param(
            "~joint/corridor_clearance_slack", 0.0)
        self.joint_minlp_corridor_clearance_slack = rospy.get_param(
            "~joint/minlp_corridor_clearance_slack",
            self.joint_corridor_clearance_slack)
        # Seed tracking is only a MINLP search regularizer. Keep it separate
        # from the common trajectory score so matched experiments can measure
        # whether it helps Bonmin find a better integer schedule.
        self.joint_minlp_track_weight = max(0.0,float(rospy.get_param(
            "~joint/minlp_track_weight", .015)))
        self.joint_geometry_yaw_bounds = bool(rospy.get_param(
            "~joint/geometry_yaw_bounds", False))
        self.joint_geometry_spin_pruning = bool(rospy.get_param(
            "~joint/geometry_spin_pruning", False))
        self.joint_geometry_oblique_bounds = bool(rospy.get_param(
            "~joint/geometry_oblique_bounds", False))
        self.joint_minlp_ackermann_merge_tolerance = max(1e-5,float(
            rospy.get_param("~joint/minlp_ackermann_merge_tolerance",0.1)))
        self.joint_minlp_oblique_merge_tolerance = max(
            math.radians(.25),math.radians(float(rospy.get_param(
                "~joint/minlp_oblique_merge_tolerance_deg",7.5))))
        self.joint_feasibility_pump = rospy.get_param(
            "~joint/feasibility_pump", True)
        self.joint_minlp_pump = rospy.get_param("~joint/minlp_pump", True)
        self.joint_root_resolves = rospy.get_param(
            "~joint/root_resolves", 0)
        self.joint_dive_fractional = rospy.get_param(
            "~joint/dive_fractional", False)
        self.joint_random_seed = rospy.get_param("~joint/random_seed", 0)
        self.joint_seed_trace_file = str(rospy.get_param(
            "~joint/seed_trace_file", "")).strip()
        self.joint_seed_replay_file = str(rospy.get_param(
            "~joint/seed_replay_file", "")).strip()
        self.joint_feasible_seed_warm_start = rospy.get_param(
            "~joint/feasible_seed_warm_start", False)
        self.joint_warm_start_collision_retries = rospy.get_param(
            "~joint/warm_start_collision_retries", 2)
        self.joint_max_translation = rospy.get_param(
            "~joint/max_translation", 3.0)
        self.joint_terminal_xy_tol = rospy.get_param(
            "~joint/terminal_xy_tol", .015)
        self.joint_terminal_yaw_tol = rospy.get_param(
            "~joint/terminal_yaw_tol", .02)
        self.joint_terminal_xy_weight = rospy.get_param(
            "~joint/terminal_xy_weight", 100.0)
        self.joint_terminal_yaw_weight = rospy.get_param(
            "~joint/terminal_yaw_weight", 10.0)
        self.joint_alternating_iterations = rospy.get_param(
            "~joint/alternating_iterations", 2)
        self.joint_min_round_improvement = rospy.get_param(
            "~joint/min_round_improvement", .0025)
        self.mode_candidate_count = rospy.get_param("~mode_master/max_candidates", 4)
        self.mode_refine_slack = rospy.get_param("~mode_master/refine_slack", 0.25)
        self.mode_schedule_neighbor_candidates = max(0,int(rospy.get_param(
            "~mode_master/schedule_neighbor_candidates", 1)))
        self.mode_schedule_neighbor_window = max(1,int(rospy.get_param(
            "~mode_master/schedule_neighbor_window", 6)))
        self.mode_schedule_neighbor_time_limit = max(.1,float(rospy.get_param(
            "~mode_master/schedule_neighbor_time_limit", 2.0)))
        self.mode_schedule_neighbor_refine_seed = rospy.get_param(
            "~mode_master/refine_seed_schedule", False)
        self.cost_weights={
            "spin":rospy.get_param(
                "~cost/w_spin",self.v_demo/max(self.omega_demo,1e-6)),
            "switch":rospy.get_param("~cost/w_switch",2.00),
            "low_curvature_switch":rospy.get_param(
                "~cost/w_low_curvature_switch",.35),
            "switch_turn_threshold":rospy.get_param(
                "~cost/switch_turn_threshold",.35),
            "reverse":rospy.get_param("~cost/w_reverse",1.00),
            "direction":rospy.get_param("~cost/w_direction",20.0),
            "steer_tv":rospy.get_param("~cost/w_steer_tv",.40),
            "curvature":rospy.get_param("~cost/w_curvature",.01),
            "oblique_tv":rospy.get_param("~cost/w_oblique_tv",1.50),
            "oblique_event":rospy.get_param("~cost/w_oblique_event_nlp",1.50),
            "oblique_event_score":rospy.get_param("~cost/w_oblique_event_score",1.50),
            "oblique_cruise":rospy.get_param("~cost/w_oblique_cruise",.45),
            "oblique_use":rospy.get_param("~cost/w_oblique_use",.55),
            "terminal_xy":rospy.get_param("~cost/w_terminal_xy",8.0),
            "terminal_yaw":rospy.get_param("~cost/w_terminal_yaw",2.0)}
        self.path_pub = rospy.Publisher("/plan/path", Path, queue_size=1, latch=True)
        self.time_pub = rospy.Publisher("/plan/timed_trajectory", TimedTrajectory, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher("/plan/mode_strip", MarkerArray, queue_size=1, latch=True)
        self.coarse_mode_pub = rospy.Publisher("/plan/mode_strip_coarse", MarkerArray, queue_size=1, latch=True)
        self.corr_pub = rospy.Publisher("/plan/corridor_markers", MarkerArray, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher("/plan/status", String, queue_size=1, latch=True)
        self.quality_pub = rospy.Publisher("/plan/quality_report", String, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber("/plan/seed_candidates", PlanSeedCandidates,
                         self.on_seed_candidates, queue_size=2)
        rospy.Subscriber("/plan/seed", PlanSeed, self.on_seed, queue_size=1)
        rospy.Subscriber("/plan/search_status", String,
                         self.on_search_status, queue_size=2)

    def status(self, message):
        self.status_pub.publish(String(data=message))
        rospy.loginfo("[v3 optimizer] %s", message)

    def _joint_seed_map_signature(self):
        return {
            "sha256": hashlib.sha256(
                np.asarray(self.grid.occ, dtype=np.uint8).tobytes()).hexdigest(),
            "width": int(self.grid.width),
            "height": int(self.grid.height),
            "resolution": float(self.grid.res),
            "origin": [float(self.grid.ox), float(self.grid.oy)],
            "half_vehicle": [float(self.grid.half_l), float(self.grid.half_w)]
        }

    def _read_joint_seed_replay(self, start, goal):
        with open(self.joint_seed_replay_file, "r", encoding="utf-8") as stream:
            record = json.load(stream)
        if record.get("schema") != "fwis_v3_joint_seed_v1":
            raise ValueError("unsupported_seed_replay_schema")
        recorded_map = record.get("map")
        current_map = self._joint_seed_map_signature()
        if recorded_map != current_map:
            raise ValueError("seed_replay_map_mismatch")
        for label, actual in (("start", start), ("goal", goal)):
            expected = record.get(label)
            if (not isinstance(expected, list) or len(expected) != 3 or
                    max(abs(float(v)-a) for v, a in
                        zip(expected, (actual.x, actual.y, actual.th))) > 1e-7):
                raise ValueError("seed_replay_%s_mismatch" % label)
        raw_actions = record.get("actions")
        if not isinstance(raw_actions, list) or not raw_actions:
            raise ValueError("seed_replay_actions_missing")
        actions = []
        for row in raw_actions:
            if not isinstance(row, list) or len(row) != 3:
                raise ValueError("seed_replay_action_format_invalid")
            mode, amount, control = int(row[0]), float(row[1]), float(row[2])
            if mode not in (0, 1, 2) or not math.isfinite(amount) or not math.isfinite(control):
                raise ValueError("seed_replay_action_value_invalid")
            actions.append(Action(mode, amount, control))
        if has_reverse_motion(actions):
            raise ValueError("seed_replay_reverse_motion_forbidden")
        if sequence_cost(actions, self.cost_weights)["direction_switches"] != 0:
            raise ValueError("seed_replay_direction_switch_forbidden")
        poses = replay(start, actions)
        end = poses[-1]
        if (math.hypot(end.x-goal.x, end.y-goal.y) > self.xy_tol+1e-5 or
                abs(wrap(end.th-goal.th)) > self.yaw_tol+1e-5):
            raise ValueError("seed_replay_terminal_pose_invalid")
        if not self.grid.trajectory_clear(poses, actions, max_corner_step=.005):
            raise ValueError("seed_replay_swept_collision")
        return sequence_cost(actions, self.cost_weights)["score"], poses, actions

    def _audit_joint_seed(self, start, goal, actions):
        """Preflight the exact compressed trajectory that the MINLP will use."""
        compressed,_=_collapse_for_joint(
            start,actions,max_translation=self.joint_max_translation,
            max_spin=math.pi,
            ackermann_merge_tolerance=(
                self.joint_minlp_ackermann_merge_tolerance),
            oblique_merge_tolerance=(
                self.joint_minlp_oblique_merge_tolerance))
        reason="ok"
        poses=replay(start,compressed)
        end=poses[-1]
        reverse=has_reverse_motion(compressed)
        direction_switches=sequence_cost(
            compressed,self.cost_weights)["direction_switches"]
        position_error=math.hypot(end.x-goal.x,end.y-goal.y)
        heading_error=abs(wrap(end.th-goal.th))
        if not compressed:
            reason="empty_after_collapse"
        elif reverse or direction_switches:
            reason="direction_constraint"
        elif len(compressed)>self.joint_max_knots:
            reason="knot_limit"
        elif (position_error>self.xy_tol+1e-5 or
              heading_error>self.yaw_tol+1e-5):
            reason="terminal_after_collapse"
        elif not self.grid.trajectory_clear(
                poses,compressed,max_corner_step=.0025):
            reason="swept_collision_after_collapse"
        dense_poses,_=expand(start,compressed,max_translation=.10,
                             max_rotation=math.radians(5.0))
        clearance=min((self.grid.center_clearance_lower_bound(p.x,p.y)
                       for p in dense_poses),default=0.0)
        return {
            "valid":reason=="ok","reason":reason,
            "actions":compressed,"poses":poses,
            "score":sequence_cost(compressed,self.cost_weights)["score"],
            "knots":len(compressed),
            "mode_switches":sum(a.mode!=b.mode
                                for a,b in zip(compressed,compressed[1:])),
            "position_error":position_error,
            "heading_error":heading_error,
            "min_clearance":clearance,
        }

    def _mode_neighborhood_seed_hint(self, start, goal, seed_candidate):
        """Choose one exact safe A/O replacement for the existing Bonmin call.

        Generate from the actual selected/replayed compressed seed. No extra
        NLP or MINLP solve is added; the local incumbent stays in the outer
        candidate pool. Compression must preserve safety, terminal accuracy,
        lower common cost, and every mode/steering complexity component.
        """
        begin=time.perf_counter()
        seed_score,seed_source,_,seed_actions,_,_=seed_candidate
        seed_complexity=_mode_complexity(seed_actions)
        profile={"minlp_seed_hint_generated":0,
                 "minlp_seed_hint_audited":0,
                 "minlp_seed_hint_eligible":0,
                 "minlp_seed_hint_selected":0}
        hints=mode_schedule_neighbor_candidates(
            start,seed_actions,self.grid,
            max_candidates=max(1,self.joint_mode_neighbor_candidates),
            max_window=self.joint_mode_neighbor_window,
            weights=self.cost_weights,sweep_step=.0025)
        profile["minlp_seed_hint_generated"]=len(hints)
        eligible=[]
        diagnostics=[]
        for _,actions,_,info in hints:
            audit=self._audit_joint_seed(start,goal,actions)
            profile["minlp_seed_hint_audited"]+=1
            complexity=_mode_complexity(audit["actions"])
            reason=audit["reason"]
            if audit["valid"]:
                if audit["score"]>=seed_score-1e-6:
                    reason="no_common_score_gain_after_collapse"
                elif any(a>b for a,b in zip(complexity,seed_complexity)):
                    reason="complexity_increased_after_collapse"
                else:
                    eligible.append((audit["score"],audit,info))
            diagnostics.append(dict(
                info,reason=reason,score=audit["score"],
                knots=audit["knots"],complexity=list(complexity)))
        profile["minlp_seed_hint_eligible"]=len(eligible)
        selected=seed_candidate
        if eligible:
            _,audit,info=min(eligible,key=lambda item:item[0])
            selected=(audit["score"],seed_source+"_mode_hint",
                      audit["poses"],audit["actions"],[],{
                          "mode_neighbor_seed_hint":info,
                          "mode_neighbor_base_source":seed_source,
                          "mode_neighbor_base_score":float(seed_score)})
            profile["minlp_seed_hint_selected"]=1
        profile["minlp_seed_hint_time_s"]=time.perf_counter()-begin
        rospy.loginfo(
            "[v3 optimizer] MINLP mode seed hint selected=%s "
            "base_source=%s base_score=%.6f selected_score=%.6f "
            "complexity=%s->%s generated=%d eligible=%d data=%s elapsed=%.3fs",
            bool(eligible),seed_source,seed_score,selected[0],
            seed_complexity,_mode_complexity(selected[3]),len(hints),
            len(eligible),json.dumps(diagnostics,sort_keys=True,
                                     separators=(",",":")),
            profile["minlp_seed_hint_time_s"])
        return selected,profile

    def _write_joint_seed_trace(self, plan_id, start, goal, source, score, actions):
        if not self.joint_seed_trace_file:
            return
        record = {
            "schema": "fwis_v3_joint_seed_v1",
            "plan_id": int(plan_id),
            "source": str(source),
            "score": float(score),
            "start": [float(start.x), float(start.y), float(start.th)],
            "goal": [float(goal.x), float(goal.y), float(goal.th)],
            "map": self._joint_seed_map_signature(),
            "actions": [[int(a.mode), float(a.amount), float(a.control)]
                        for a in actions]
        }
        target = os.path.abspath(self.joint_seed_trace_file)
        parent = os.path.dirname(target) or "."
        temporary = None
        try:
            os.makedirs(parent, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                    mode="w", encoding="utf-8", dir=parent,
                    prefix=".joint_seed_", suffix=".tmp", delete=False) as stream:
                temporary = stream.name
                json.dump(record, stream, separators=(",", ":"))
                stream.write("\n")
            os.replace(temporary, target)
            rospy.loginfo(
                "[v3 optimizer] joint seed trace saved path=%s source=%s "
                "score=%.6f actions=%d",
                target, source, score, len(actions))
        except Exception as exc:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)
            rospy.logwarn("[v3 optimizer] joint seed trace failed path=%s: %s",
                          target, exc)

    def clear(self, plan_id):
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        self.path_pub.publish(Path(header=header))
        self.time_pub.publish(TimedTrajectory(header=header, plan_id=plan_id))
        erase = Marker()
        erase.header = header
        erase.action = Marker.DELETEALL
        self.mode_pub.publish(MarkerArray(markers=[erase]))
        self.coarse_mode_pub.publish(MarkerArray(markers=[erase]))
        self.corr_pub.publish(MarkerArray(markers=[erase]))
        self.quality_pub.publish(String(data=json.dumps({"plan_id": int(plan_id), "status": "pending"})))

    def _persist_plan_report(self, report, plan_id, start=None, goal=None,
                             seed=None):
        """Save a complete, versioned result report without interrupting planning."""
        payload=dict(report)
        payload.setdefault("schema_version","expv3_optimizer_run_report_v1")
        payload.setdefault("units",{"position":"m","angle":"rad",
            "time":"s","speed":"m/s","acceleration":"m/s^2",
            "arc_length":"m","sweep_spacing":"m"})
        payload["plan_id"]=int(plan_id)
        payload["backend"]=payload.get("backend",self.planning_backend)
        if start is not None:
            payload["start"]=[float(start.x),float(start.y),float(start.th)]
        if goal is not None:
            payload["goal"]=[float(goal.x),float(goal.y),float(goal.th)]
        if seed is not None:
            payload["seed_actions"]=[[int(a.mode),float(a.amount),float(a.control)]
                                     for a in seed]
        payload["vehicle_contract"]={"length_m":float(self.length),
            "width_m":float(self.width),"margin_total_dimension_m":float(self.margin),
            "sweep_spacing_m":float(self.quality_profile.sweep_spacing),
            "goal_xy_tolerance_m":.02,"goal_yaw_tolerance_rad":math.radians(1.)}
        payload["direction_contract"]={"policy":self.direction_policy,
            "allow_reverse":bool(self.allow_reverse),
            "allow_shunting":bool(self.allow_shunting)}
        payload["execution_profile"]=asdict(self.quality_limits)
        payload["quality_profile"]=asdict(self.quality_profile)
        payload["map_signature"]=(self._joint_seed_map_signature()
                                  if self.grid is not None else None)
        source_files=("catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/quality.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_geometry.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_optimizer.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/geometry_incumbent.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/planning_display.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/legacy_geometry_provider.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/mode_master.py",
                      "catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_retiming.py",
                      "catkin_ws/src/fwis_v3_planner/src/planner_node.cpp",
                      "catkin_ws/src/fwis_v3_demo/config/params.yaml")
        payload["source_manifest"]={}
        for relative in source_files:
            source_path=os.path.join(self.project_root,relative)
            try:
                with open(source_path,"rb") as stream:
                    payload["source_manifest"][relative]=hashlib.sha256(
                        stream.read()).hexdigest()
            except OSError:
                payload["source_manifest"][relative]=None
        filename="plan_%06d_%d.json"%(int(plan_id),time.time_ns())
        target=os.path.join(self.report_dir,filename)
        payload["diagnostic_file"]=target
        try:
            atomic_write_json(target,payload)
            return target
        except Exception as exc:
            rospy.logwarn("[v3 optimizer] persistent report failed plan=%d: %s",
                          int(plan_id),exc)
            return None

    def on_map(self, msg):
        if abs(msg.info.origin.orientation.z) > 1e-7:
            self.status("map_rotated_origin_unsupported")
            return
        occupied = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width) != 0
        self.grid = Grid(occupied, msg.info.resolution,
                         msg.info.origin.position.x, msg.info.origin.position.y,
                         self.length, self.width, self.margin)
        self.status("map_ready %dx%d" % (self.grid.width, self.grid.height))

    def on_seed_candidates(self, msg):
        """Cache provisional geometry until the matching primary seed arrives."""
        plan_id = int(msg.plan_id)
        if not msg.proposal_primitives:
            # The planner publishes an empty bundle when a new goal clears the
            # previous proposal. Do not let that clear marker satisfy the
            # matching primary-seed wait later in the same plan_id.
            with self._seed_proposal_condition:
                self._seed_proposals.pop(plan_id, None)
                self._seed_proposal_condition.notify_all()
            return
        record = {
            "source": str(msg.proposal_source),
            "actions": [Action(a.mode, a.amount, a.control)
                        for a in msg.proposal_primitives],
        }
        with self._seed_proposal_condition:
            self._seed_proposals[plan_id] = record
            # A latched topic retains only the newest useful record here.
            for plan_id in sorted(self._seed_proposals)[:-4]:
                self._seed_proposals.pop(plan_id, None)
            self._seed_proposal_condition.notify_all()

    def on_search_status(self, msg):
        reason = msg.data.split(" ", 1)[0]
        if reason not in ("search_budget_exhausted",
                "search_no_path_in_discretized_lattice", "action_policy_rejected",
                "invalid_goal_frame", "missing_map_or_start"):
            return
        match = re.search(r"\bid=(\d+)\b", msg.data)
        if match is None:
            return
        plan_id = int(match.group(1))
        if plan_id < self.plan_id:
            return
        self._frontend_failures[plan_id] = msg.data
        for old in sorted(self._frontend_failures)[:-4]:
            self._frontend_failures.pop(old, None)
        self._publish_frontend_failure(plan_id, msg.data)

    def _publish_frontend_failure(self, plan_id, search_status, seed_msg=None):
        report = {"plan_id": int(plan_id), "status": "REJECTED",
            "backend": self.planning_backend, "failure_stage": "frontend_search",
            "reason": search_status.split(" ", 1)[0],
            "search_status": search_status, "infeasibility_proof": False}
        start = (Pose(seed_msg.start_center.x, seed_msg.start_center.y,
                      seed_msg.start_center.theta) if seed_msg is not None else None)
        goal = (Pose(seed_msg.goal_center.x, seed_msg.goal_center.y,
                     seed_msg.goal_center.theta) if seed_msg is not None else None)
        self._persist_plan_report(report, plan_id, start, goal, [])
        self.quality_pub.publish(String(data=json.dumps(report)))
        self.status("frontend_rejected id=%d reason=%s" %
                    (plan_id, report["reason"]))

    def on_seed(self, msg):
        if msg.plan_id < self.plan_id:
            return
        self.plan_id = msg.plan_id
        self.clear(msg.plan_id)
        if not msg.primitives:
            if msg.plan_id in self._frontend_failures:
                self._publish_frontend_failure(msg.plan_id,
                    self._frontend_failures[msg.plan_id], msg)
                return
            self.status("search_pending_or_failed id=%d" % msg.plan_id)
            return
        if self.grid is None:
            self._persist_plan_report({"status":"REJECTED",
                "failure_stage":"input_validation","reason":"missing_map"},
                int(msg.plan_id),Pose(msg.start_center.x,msg.start_center.y,
                msg.start_center.theta),Pose(msg.goal_center.x,msg.goal_center.y,
                msg.goal_center.theta),[])
            self.status("missing_map id=%d" % msg.plan_id)
            return
        start = Pose(msg.start_center.x, msg.start_center.y, msg.start_center.theta)
        goal = Pose(msg.goal_center.x, msg.goal_center.y, msg.goal_center.theta)
        actions = [Action(a.mode, a.amount, a.control) for a in msg.primitives]
        auxiliary_seeds = []
        if self.planning_backend in ("mode_path_v1", "mode_path_v2"):
            proposal_deadline = time.monotonic()+.15
            with self._seed_proposal_condition:
                while (int(msg.plan_id) not in self._seed_proposals and
                       time.monotonic() < proposal_deadline):
                    self._seed_proposal_condition.wait(
                        max(0., proposal_deadline-time.monotonic()))
                proposal = self._seed_proposals.pop(int(msg.plan_id), None)
            if proposal and proposal["actions"]:
                auxiliary_seeds.append((proposal["actions"], proposal["source"]))
        if has_reverse_motion(actions) and not self.allow_reverse:
            self._persist_plan_report({"status":"REJECTED",
                "failure_stage":"direction_policy",
                "reason":"reverse_coarse_seed_rejected"},int(msg.plan_id),
                start,goal,actions)
            self.status("reverse_coarse_seed_rejected id=%d" % msg.plan_id)
            return
        # Keep all mode families eligible during refinement. The common score
        # allows a constant-angle O segment to replace costly A curvature,
        # while explicitly charging O steering-angle changes and mode events.
        allow_oblique_candidates=True
        coarse_poses=replay(start,actions)
        coarse_colors=((.0,.36,.42),(.48,.26,.08),(.38,.12,.48))
        self.coarse_mode_pub.publish(make_mode_strip(
            actions,coarse_poses,Header(stamp=rospy.Time.now(),frame_id="map"),
            "v3_coarse_modes",.035,.48,coarse_colors))
        begin = time.perf_counter()
        if self.planning_backend in ("mode_path_v1", "mode_path_v2"):
            self._persist_plan_report({"status":"IN_PROGRESS",
                "failure_stage":"mode_path_backend"},int(msg.plan_id),
                start,goal,actions)
            self.run_mode_path(msg.plan_id, start, goal, actions, begin,
                               auxiliary_seeds=auxiliary_seeds)
            return
        try:
            candidates=[]; errors=[]
            fixed_seconds=anchor_seconds=sample_seconds=0.0
            fixed_profile={};anchor_profile={}
            explored_mode_schedules=set()
            pre_neighbor_profile={
                "pre_neighbor_schedules_generated":0,
                "pre_neighbor_schedules_solved":0,
                "pre_neighbor_schedules_accepted":0}
            post_neighbor_profile={
                "post_neighbor_schedules_generated":0,
                "post_neighbor_schedules_solved":0,
                "post_neighbor_schedules_accepted":0,
                "post_neighbor_skipped_duplicate":0,
                "post_neighbor_skip_code":0,
                "post_neighbor_relative_improvement":0.0,
                "post_neighbor_time_s":0.0}
            anchor_best_score=float("inf");anchor_count=0;anchor_best_modes="none"
            anchor_refinement_attempted=False
            # Build safe local alternatives first. The global MINLP below uses
            # their best endpoint-checked path as its warm seed instead of the
            # raw lattice route, which can be a much poorer and longer NLP
            # initialization on the same map.
            fixed_seed_actions=actions
            fixed_seed_poses=None;fixed_seed_boxes=None;fixed_seed_meta=None
            try:
                fixed_begin=time.perf_counter()
                fixed_seed_poses,fixed_seed_actions,fixed_seed_boxes,\
                    fixed_seed_meta=refine(
                        start,goal,actions,self.grid,self.xy_tol,self.yaw_tol,
                        max_iter=self.optimizer_max_iter,
                        weights=self.cost_weights)
                fixed_seconds=time.perf_counter()-fixed_begin
                fixed_profile=fixed_seed_meta.get("profile",{})
                fixed_score=sequence_cost(
                    fixed_seed_actions,self.cost_weights)["score"]
                fixed_seed_meta["common_score"]=fixed_score
                candidates.append((fixed_score,"fixed",fixed_seed_poses,
                                   fixed_seed_actions,fixed_seed_boxes,
                                   fixed_seed_meta))
            except Exception as exc:
                fixed_seconds=time.perf_counter()-fixed_begin
                fixed_seed_actions=actions
                errors.append("fixed="+str(exc).splitlines()[-1])
            # A bounded anchor graph can merge adjacent coarse edges: each
            # A/O/Spin primitive exactly joins retained center anchors, so both
            # its geometry and the mode-switch location may differ from seed.
            try:
                anchor_begin=time.perf_counter()
                alternatives=anchor_mode_sequences(
                    start,actions,self.grid,self.mode_candidate_count,
                    beam_width=24,weights=self.cost_weights,profile=anchor_profile)
                def mode_runs(sequence):
                    return tuple(a.mode for i,a in enumerate(sequence)
                                 if i==0 or a.mode!=sequence[i-1].mode)
                original_modes=mode_runs(actions)
                original_score=sequence_cost(actions,self.cost_weights)["score"]
                for schedule,schedule_cost in alternatives:
                    anchor_count+=1
                    if schedule_cost<anchor_best_score:
                        anchor_best_score=schedule_cost
                        anchor_best_modes="".join(str(m) for m in mode_runs(schedule))
                    if (mode_runs(schedule)==original_modes and
                            schedule_cost>=original_score-1e-7):
                        continue
                    anchor_poses=replay(start,schedule)
                    if (math.hypot(anchor_poses[-1].x-goal.x,
                                   anchor_poses[-1].y-goal.y)>self.xy_tol+1e-5 or
                        abs(wrap(anchor_poses[-1].th-goal.th))>self.yaw_tol+1e-5):
                        continue
                    incumbent_score=min(item[0] for item in candidates)
                    anchor_meta={"solver":"anchor_graph",
                                 "corridor":"anchor_graph_only",
                                 "common_score":schedule_cost,
                                 "seed_cost":schedule_cost,
                                 "refinement":"skipped"}
                    # The anchor-graph path is already a swept-safe anytime
                    # candidate. Refine it only when its exact common score is
                    # competitive with the current incumbent; an inferior seed
                    # cannot win unless the NLP makes a large speculative jump.
                    candidates.append((schedule_cost,"anchor_graph",anchor_poses,
                                       schedule,[],anchor_meta))
                    if (anchor_refinement_attempted or
                            schedule_cost>=incumbent_score+self.mode_refine_slack):
                        continue
                    anchor_refinement_attempted=True
                    try:
                        poses,refined,boxes,meta=refine(
                            start,goal,schedule,self.grid,self.xy_tol,self.yaw_tol,
                            max_iter=self.optimizer_max_iter,weights=self.cost_weights)
                        meta["mode_master_seed_score"]=schedule_cost
                        score=sequence_cost(refined,self.cost_weights)["score"]
                        meta["common_score"]=score
                        candidates.append((score,
                                           "mode_master",poses,refined,boxes,meta))
                    except Exception as exc:
                        errors.append("mode_master="+str(exc).splitlines()[-1])
                anchor_seconds=time.perf_counter()-anchor_begin
            except Exception as exc:
                anchor_seconds=time.perf_counter()-anchor_begin
                errors.append("mode_master_generation="+str(exc).splitlines()[-1])
            # Re-search the terminal neighborhood when it has repeated mode
            # or gear changes. This complexity trigger is scene-agnostic: a
            # local A/O/Spin lattice proposes alternatives, and the same
            # full-plan cost plus swept-footprint check decides whether one
            # replaces the incumbent.
            try:
                endpoint_begin=time.perf_counter()
                events=collapse_exact_runs(actions)
                lengths=[abs(a.amount) if a.mode!=2 else 0.0
                         for a in events]
                total_length=sum(lengths)
                tail_start=max(0.0,total_length-self.endpoint_replan_tail)
                cumulative=[0.0]
                for length in lengths:
                    cumulative.append(cumulative[-1]+length)
                tail_idx=next((i for i,value in enumerate(cumulative[:-1])
                               if value>=tail_start),0)
                tail=events[tail_idx:]
                mode_events=sum(a.mode!=b.mode for a,b in zip(tail,tail[1:]))
                gear_events=sum(a.amount*b.amount<0
                                for a,b in zip(tail,tail[1:])
                                if a.mode!=2 and b.mode!=2)
                terminal_spin_without_reversal=(
                    any(a.mode==2 for a in tail) and mode_events>0 and
                    gear_events==0)
                # Find windows where maneuver complexity coincides with low
                # downstream clearance. Open sections remain cheap and do not
                # trigger an upstream suffix search by themselves.
                path_s=[0.0]
                for action in events:
                    path_s.append(path_s[-1]+(
                        abs(action.amount) if action.mode!=2 else 0.0))
                sos_hotspot_idx=None;sos_hotspot_score=0.0
                sos_hotspot_metrics={}
                event_poses=replay(start,events)
                for i in range(max(0,len(events)-1)):
                    if path_s[i]<.6:
                        continue
                    j=i+1
                    while (j<len(events) and
                           path_s[j]-path_s[i]<=self.endpoint_replan_tail):
                        j+=1
                    remaining_route=max(0.0,path_s[-1]-path_s[j])
                    if remaining_route>self.endpoint_sos_hotspot_lookahead:
                        continue
                    segment=events[i:j]
                    local_mode=sum(a.mode!=b.mode
                                   for a,b in zip(segment,segment[1:]))
                    if (i>0 and segment and
                            events[i-1].mode!=segment[0].mode):
                        local_mode+=1
                    local_gear=sum(a.amount*b.amount<0
                                   for a,b in zip(segment,segment[1:])
                                   if a.mode!=2 and b.mode!=2)
                    spin_total=sum(abs(a.amount) for a in segment
                                   if a.mode==2)
                    ackermann_tv=sum(abs(b.control-a.control)
                        for a,b in zip(segment,segment[1:])
                        if a.mode==b.mode==0)
                    oblique_tv=sum(abs(wrap(b.control-a.control))
                        for a,b in zip(segment,segment[1:])
                        if a.mode==b.mode==1)
                    oblique_updates=sum(
                        abs(wrap(b.control-a.control))>math.radians(1.0)
                        for a,b in zip(segment,segment[1:])
                        if a.mode==b.mode==1)
                    end_pose=event_poses[min(j,len(event_poses)-1)]
                    goal_yaw_demand=abs(wrap(goal.th-end_pose.th))
                    min_clearance=min(
                        (self.grid.center_clearance_lower_bound(p.x,p.y)
                         for p in event_poses[i:min(j+1,len(event_poses))]),
                        default=0.0)
                    clearance_pressure=max(0.0,min(1.0,
                        (self.endpoint_sos_hotspot_clearance-min_clearance)/
                        max(.05,self.endpoint_sos_hotspot_clearance)))
                    complexity=(2.0*(local_mode+local_gear)+
                        min(2.0,spin_total/math.pi)+
                        min(2.0,ackermann_tv/.45)+
                        min(2.0,oblique_tv/math.radians(45.0))+
                        .5*oblique_updates+
                        .5*min(1.0,goal_yaw_demand/math.pi))
                    score=complexity*math.sqrt(clearance_pressure)
                    if clearance_pressure<.05 or complexity<.5:
                        continue
                    if score>sos_hotspot_score:
                        sos_hotspot_idx=i;sos_hotspot_score=score
                        reasons=[]
                        if local_mode+local_gear:
                            reasons.append("mode_or_gear_events")
                        if spin_total>math.radians(8):
                            reasons.append("spin_angle")
                        if ackermann_tv>.45:
                            reasons.append("ackermann_curvature_tv")
                        if oblique_updates:
                            reasons.append("oblique_angle_updates")
                        if goal_yaw_demand>math.radians(15):
                            reasons.append("goal_yaw_demand")
                        reasons.append("low_downstream_clearance")
                        sos_hotspot_metrics={
                            "mode_events":local_mode,
                            "gear_events":local_gear,
                            "spin_total":spin_total,
                            "ackermann_curvature_tv":ackermann_tv,
                            "oblique_angle_tv":oblique_tv,
                            "oblique_updates":oblique_updates,
                            "goal_yaw_demand":goal_yaw_demand,
                            "remaining_route":remaining_route,
                            "minimum_clearance":min_clearance,
                            "clearance_pressure":clearance_pressure,
                            "complexity":complexity,
                            "reason":"+".join(reasons)}
                sos_hotspot_detected=(
                    sos_hotspot_idx is not None and
                    sos_hotspot_score>=self.endpoint_sos_min_hotspot_score)
                anchor_profile["sos_hotspot_detected"]=int(
                    sos_hotspot_detected)
                anchor_profile["sos_hotspot_score"]=float(
                    sos_hotspot_score)
                anchor_profile["sos_hotspot_threshold"]=float(
                    self.endpoint_sos_min_hotspot_score)
                anchor_profile["sos_hotspot_clearance_limit"]=float(
                    self.endpoint_sos_hotspot_clearance)
                anchor_profile["sos_hotspot_lookahead_limit"]=float(
                    self.endpoint_sos_hotspot_lookahead)
                for key,value in sos_hotspot_metrics.items():
                    if key!="reason":
                        anchor_profile["sos_hotspot_"+key]=float(value)
                anchor_profile["sos_hotspot_reason_code"]=float(
                    (1 if sos_hotspot_metrics.get("mode_events",0) else 0) +
                    (2 if sos_hotspot_metrics.get("spin_total",0)>0 else 0) +
                    (4 if sos_hotspot_metrics.get("ackermann_curvature_tv",0)>.45 else 0) +
                    (8 if sos_hotspot_metrics.get("oblique_updates",0) else 0) +
                    (16 if sos_hotspot_metrics.get("goal_yaw_demand",0)>math.radians(15) else 0) +
                    (32 if sos_hotspot_metrics else 0))
                anchor_profile["sos_clear_window_count"]=0
                if not self.endpoint_sos_enabled:
                    anchor_profile["sos_skip_reason_code"]=1
                    sos_skip_reason="disabled"
                elif sos_hotspot_idx is None:
                    anchor_profile["sos_skip_reason_code"]=2
                    sos_skip_reason="no_hotspot"
                elif not sos_hotspot_detected:
                    anchor_profile["sos_skip_reason_code"]=2
                    sos_skip_reason="below_complexity_clearance_threshold"
                else:
                    anchor_profile["sos_skip_reason_code"]=0
                    sos_skip_reason="candidate_search"
                if (mode_events+gear_events>=2 or
                        terminal_spin_without_reversal) and tail:
                    current_best=min((item[0] for item in candidates),
                                     default=sequence_cost(
                                         actions,self.cost_weights)["score"])
                    if terminal_spin_without_reversal:
                        pose_candidates=terminal_pose_graph_candidates(
                            start,goal,events,self.grid,current_best,
                            self.endpoint_replan_tail,
                            weights=self.cost_weights,
                            profile=anchor_profile)
                        for proposal_score,proposal,proposal_poses,details in pose_candidates:
                            endpoint_meta={
                                "solver":"terminal_local_pose_graph",
                                "corridor":"rebuild_after_replan",
                                "common_score":proposal_score,
                                "endpoint_target":"approach_graph"}
                            endpoint_meta.update(details)
                            candidates.append((
                                proposal_score,"endpoint_replan",
                                proposal_poses,proposal,[],endpoint_meta))
                            current_best=min(current_best,proposal_score)
                    tried=set()
                    for horizon in (4.0,):
                        split=max(0.0,total_length-horizon)
                        idx=next((i for i,value in enumerate(cumulative[:-1])
                                  if value>=split),0)
                        if idx in tried or idx>=len(events):
                            continue
                        tried.add(idx)
                        prefix=events[:idx]
                        local_seed=events[idx:]
                        local_start=replay(start,prefix)[-1]
                        initial_mode=prefix[-1].mode if prefix else 0
                        local_targets=[(goal,None)]
                        if terminal_spin_without_reversal:
                            # Sample a free approach pose on the goal axis.
                            # The search may reach it in any mode; only after
                            # search do A/O terminal fits compete. This is a
                            # pose-graph option, not a prescribed mode recipe.
                            distance=1.0
                            approach=Pose(
                                goal.x+distance*math.cos(goal.th),
                                goal.y+distance*math.sin(goal.th),goal.th)
                            local_targets.append((approach,"approach"))
                        for local_goal,target_kind in local_targets:
                            local=bounded_research(
                                local_start,local_goal,local_seed,self.grid,
                                self.xy_tol,self.yaw_tol,
                                self.endpoint_replan_iter,self.repair_padding,
                                initial_mode=initial_mode,anytime=True,
                                weights=self.cost_weights,
                                allow_oblique=allow_oblique_candidates)
                            if local is None:
                                continue
                            terminal_options=[[]]
                            if target_kind=="approach":
                                approach_end=replay(local_start,local[0])[-1]
                                terminal_options=[]
                                for terminal_mode in (0,1):
                                    connector=fit_action_to_pose(
                                        terminal_mode,approach_end,goal,
                                        tolerance=max(self.xy_tol,self.yaw_tol))
                                    if connector is not None:
                                        terminal_options.append([connector])
                            for terminal in terminal_options:
                                proposal=collapse_exact_runs(
                                    prefix+local[0]+terminal)
                                if self.planning_backend == "legacy":
                                    if not legacy_direction_allowed(
                                            proposal,self.allow_reverse,
                                            self.allow_shunting):
                                        continue
                                elif sequence_cost(
                                        proposal,self.cost_weights)[
                                            "direction_switches"]>0:
                                    continue
                                proposal_score=sequence_cost(
                                    proposal,self.cost_weights)["score"]
                                if proposal_score>=current_best-1e-6:
                                    continue
                                proposal_poses,proposal_dense=expand(
                                    start,proposal,max_translation=.25,
                                    max_rotation=math.radians(7.5))
                                if not self.grid.trajectory_clear(
                                        proposal_poses,proposal_dense):
                                    continue
                                if (math.hypot(proposal_poses[-1].x-goal.x,
                                               proposal_poses[-1].y-goal.y)>
                                        self.xy_tol+1e-5 or
                                        abs(wrap(proposal_poses[-1].th-goal.th))>
                                        self.yaw_tol+1e-5):
                                    continue
                                endpoint_meta={
                                    "solver":"terminal_local_pose_graph",
                                    "corridor":"rebuild_after_replan",
                                    "common_score":proposal_score,
                                    "endpoint_replan_nodes":local[1],
                                    "endpoint_replan_anchor":idx,
                                    "endpoint_target":target_kind or "goal"}
                                anchor_profile["endpoint_replan_nodes"]=local[1]
                                anchor_profile["endpoint_replan_anchor"]=idx
                                candidates.append((
                                    proposal_score,"endpoint_replan",
                                    proposal_poses,proposal,[],endpoint_meta))
                                current_best=proposal_score
                if self.endpoint_sos_enabled and sos_hotspot_detected:
                    sos_begin=time.perf_counter()
                    current_best=min((item[0] for item in candidates),
                                     default=sequence_cost(
                                         actions,self.cost_weights)["score"])
                    sos_candidates,sos_stats=upstream_sos_replan_candidates(
                        start,goal,events,self.grid,sos_hotspot_idx,current_best,
                        max_iter=self.endpoint_sos_max_iter,
                        padding=self.repair_padding,xy_tol=self.xy_tol,
                        yaw_tol=self.yaw_tol,
                        upstream_distance=self.endpoint_sos_upstream_distance,
                        score_slack=self.endpoint_sos_score_slack,
                        switch_bonus=self.endpoint_sos_switch_bonus,
                        weights=self.cost_weights,
                        min_upstream_clearance=
                            self.endpoint_sos_upstream_clearance,
                        max_upstream_windows=
                            self.endpoint_sos_upstream_windows)
                    anchor_profile.update(sos_stats)
                    clear_windows=int(sos_stats.get(
                        "sos_clear_window_count",0))
                    anchor_profile["sos_clear_window_count"]=clear_windows
                    if clear_windows==0:
                        anchor_profile["sos_skip_reason_code"]=3
                        sos_skip_reason="no_clear_upstream_window"
                    elif not sos_candidates:
                        anchor_profile["sos_skip_reason_code"]=4
                        sos_skip_reason="no_safe_improving_candidate"
                    else:
                        sos_skip_reason="candidate_available"
                    for proposal_score,proposal,proposal_poses,details in sos_candidates:
                        candidates.append((proposal_score,"upstream_sos",
                                           proposal_poses,proposal,[],details))
                    replan_incumbent=min((item[0] for item in candidates),
                        default=sequence_cost(
                            actions,self.cost_weights)["score"])
                    suffix_candidates,suffix_stats=(
                        upstream_suffix_replan_candidates(
                            start,goal,events,self.grid,sos_hotspot_idx,
                            replan_incumbent,
                            max_iter=self.endpoint_sos_max_iter,
                            padding=self.repair_padding,xy_tol=self.xy_tol,
                            yaw_tol=self.yaw_tol,
                            upstream_distance=self.endpoint_sos_upstream_distance,
                            score_slack=self.endpoint_sos_score_slack,
                            min_upstream_clearance=
                                self.endpoint_sos_upstream_clearance,
                            max_searches=self.endpoint_sos_replan_max_searches,
                            weights=self.cost_weights))
                    anchor_profile.update(suffix_stats)
                    for proposal_score,proposal,proposal_poses,details in suffix_candidates:
                        candidates.append((proposal_score,
                            "upstream_suffix_replan",proposal_poses,
                            proposal,[],details))
                    if suffix_stats.get("upstream_replan_safe",0):
                        sos_skip_reason="candidate_available"
                        anchor_profile["sos_skip_reason_code"]=0
                    anchor_profile["endpoint_sos_s"]=(
                        time.perf_counter()-sos_begin)
                    rospy.loginfo(
                        "[v3 optimizer] upstream SOS search hotspot=%d "
                        "score=%.2f threshold=%.2f attempts=%d suffix_searches=%d "
                        "suffix_ok=%d clear_windows=%d lower_local_complexity=%d "
                        "safe=%d accepted=%d suffix_replan_searches=%d "
                        "suffix_replan_safe=%d upstream_clearance=%.3f "
                        "hotspot_reason=%s reason=%s",
                        sos_hotspot_idx,sos_hotspot_score,
                        self.endpoint_sos_min_hotspot_score,
                        sos_stats.get("sos_attempts",0),
                        sos_stats.get("sos_searches",0),
                        sos_stats.get("sos_suffix_successes",0),
                        sos_stats.get("sos_clear_window_count",0),
                        sos_stats.get("sos_complexity_improved",0),
                        sos_stats.get("sos_safe",0),len(sos_candidates),
                        suffix_stats.get("upstream_replan_searches",0),
                        suffix_stats.get("upstream_replan_safe",0),
                        suffix_stats.get("upstream_replan_best_clearance",0.0),
                        sos_hotspot_metrics.get("reason","none"),
                        sos_skip_reason)
                else:
                    rospy.loginfo(
                        "[v3 optimizer] upstream SOS skipped reason=%s "
                        "hotspot_score=%.2f threshold=%.2f",
                        sos_skip_reason,sos_hotspot_score,
                        self.endpoint_sos_min_hotspot_score)
                anchor_profile["endpoint_replan_s"]=(
                    time.perf_counter()-endpoint_begin)
            except Exception as exc:
                errors.append("endpoint_replan="+str(exc).splitlines()[-1])
            # Bounded alternating discrete/continuous coordination: start from
            # the best independently safe candidate collected above, rebuild
            # the anchor-mode graph around its latest geometry, and refine one
            # competitive alternative per round. Monotone acceptance keeps the
            # existing incumbent available if every joint proposal fails.
            if candidates and self.joint_alternating_iterations>0:
                joint_begin=time.perf_counter()
                joint_profile={}
                try:
                    seed_score,seed_source,_,seed_actions,_,_=min(
                        candidates,key=lambda item:item[0])
                    joint_poses,joint_actions,joint_boxes,joint_meta=(
                        iterated_mode_geometry_refine(
                            start,goal,seed_actions,self.grid,
                            incumbent_actions=seed_actions,
                            xy_tol=self.xy_tol,yaw_tol=self.yaw_tol,
                            rounds=self.joint_alternating_iterations,
                            max_candidates=max(2,self.mode_candidate_count),
                            max_refinements_per_round=1,
                            proposal_slack=self.mode_refine_slack,
                            min_round_improvement=self.joint_min_round_improvement,
                            max_iter=self.optimizer_max_iter,
                            weights=self.cost_weights,profile=joint_profile))
                    joint_score=sequence_cost(
                        joint_actions,self.cost_weights)["score"]
                    joint_profile["joint_seed_source"]=seed_source
                    joint_profile["joint_seed_score"]=seed_score
                    if joint_score<seed_score-1e-7:
                        joint_meta["common_score"]=joint_score
                        candidates.append((joint_score,"iterated_joint",
                                           joint_poses,joint_actions,
                                           joint_boxes,joint_meta))
                        anchor_best_score=min(anchor_best_score,joint_score)
                        anchor_best_modes="".join(
                            str(a.mode) for i,a in enumerate(joint_actions)
                            if i==0 or a.mode!=joint_actions[i-1].mode)
                        anchor_count+=joint_profile.get(
                            "joint_graph_candidates",0)
                except Exception as exc:
                    errors.append("iterated_joint="+
                                  str(exc).splitlines()[-1])
                finally:
                    joint_profile["joint_integration_s"]=(
                        time.perf_counter()-joint_begin)
                    anchor_profile.update(joint_profile)
                    anchor_seconds+=joint_profile["joint_integration_s"]
            # Short exact A/O schedule substitutions are useful even when the
            # optional Bonmin MINLP is disabled or returns no better incumbent.
            if candidates and self.mode_schedule_neighbor_candidates>0:
                neighbor_begin=time.perf_counter()
                incumbent=min(candidates,key=lambda item:item[0])
                incumbent_score,_,_,incumbent_actions,_,_=incumbent
                incumbent_costs=sequence_cost(
                    incumbent_actions,self.cost_weights)
                if (not has_reverse_motion(incumbent_actions) and
                        incumbent_costs["direction_switches"]==0):
                    neighbor_candidates,neighbor_rejections=(
                        refine_mode_schedule_neighbors(
                            start,goal,incumbent_actions,self.grid,
                            self.xy_tol,self.yaw_tol,
                            max_candidates=(
                                self.mode_schedule_neighbor_candidates),
                            max_window=self.mode_schedule_neighbor_window,
                            time_limit=self.mode_schedule_neighbor_time_limit,
                            max_translation=self.joint_max_translation,
                            max_knots=self.joint_max_knots,
                            terminal_xy_tol=self.joint_terminal_xy_tol,
                            terminal_yaw_tol=self.joint_terminal_yaw_tol,
                            terminal_xy_weight=self.joint_terminal_xy_weight,
                            terminal_yaw_weight=self.joint_terminal_yaw_weight,
                            weights=self.cost_weights,
                            collision_spacing=self.joint_collision_spacing,
                            adaptive_collision_refinement=(
                                self.joint_adaptive_collision_refinement),
                            corridor_clearance_slack=(
                                self.joint_corridor_clearance_slack),
                            schedule_registry=explored_mode_schedules,
                            profile=pre_neighbor_profile,
                            ackermann_merge_tolerance=(
                                self.joint_minlp_ackermann_merge_tolerance),
                            oblique_merge_tolerance=(
                                self.joint_minlp_oblique_merge_tolerance),
                            refine_base_schedule=(
                                self.mode_schedule_neighbor_refine_seed or
                                not self.joint_enabled)))
                    for neighbor in neighbor_candidates:
                        candidates.append(neighbor)
                        score,source,_,_,_,meta=neighbor
                        info=meta.get("mode_neighbor")
                        if info is None:
                            rospy.loginfo(
                                "[v3 optimizer] local fixed-schedule "
                                "candidate score=%.6f incumbent=%.6f "
                                "solver=ipopt_fixed_schedule swept=checked",
                                score,incumbent_score)
                        else:
                            rospy.loginfo(
                                "[v3 optimizer] local mode-neighborhood "
                                "candidate score=%.6f incumbent=%.6f "
                                "from_mode=%d to_mode=%d window=%d:%d "
                                "solver=ipopt_fixed_schedule swept=checked",
                                score,incumbent_score,info["from_mode"],
                                info["to_mode"],info["first"],info["last"])
                    for info,error in neighbor_rejections:
                        rospy.logdebug(
                            "[v3 optimizer] local mode-neighborhood "
                            "rejected window=%d:%d: %s",
                            info["first"],info["last"],error)
                    anchor_profile["local_mode_neighbor_candidates"]=(
                        len(neighbor_candidates))
                    anchor_profile.update(pre_neighbor_profile)
                anchor_profile["local_mode_neighbor_s"]=(
                    time.perf_counter()-neighbor_begin)
            if self.joint_enabled and candidates:
                minlp_seed_score=min(item[0] for item in candidates)
                minlp_seeds=[];minlp_seen=set()
                coarse_fallback=None
                eligible_minlp_seeds=[]
                minlp_seed_audits=[]
                seed_reject_counts={
                    "swept_collision_after_collapse":0,
                    "terminal_after_collapse":0,"knot_limit":0,
                    "direction_constraint":0,"empty_after_collapse":0}
                coarse_actions=collapse_exact_runs(actions)
                coarse_poses,coarse_dense=expand(
                    start,coarse_actions,max_translation=.50,
                    max_rotation=math.radians(15))
                coarse_end=coarse_poses[-1]
                coarse_direction_ok=(
                    legacy_direction_allowed(
                        coarse_actions,self.allow_reverse,self.allow_shunting)
                    if self.planning_backend == "legacy" else
                    (not has_reverse_motion(coarse_actions) and
                     sequence_cost(coarse_actions,self.cost_weights)[
                        "direction_switches"]==0))
                coarse_valid=(
                    coarse_direction_ok and
                    self.grid.trajectory_clear(coarse_poses,coarse_dense) and
                    math.hypot(coarse_end.x-goal.x,
                               coarse_end.y-goal.y)<=self.xy_tol+1e-5 and
                    abs(wrap(coarse_end.th-goal.th))<=self.yaw_tol+1e-5)
                if coarse_valid:
                    coarse_score=sequence_cost(
                        coarse_actions,self.cost_weights)["score"]
                    coarse_fallback=(
                        coarse_score,"coarse_seed",coarse_poses,
                        coarse_actions,[],{})
                audit_inputs=list(sorted(candidates,key=lambda item:item[0]))
                if coarse_fallback is not None:
                    audit_inputs.append(coarse_fallback)
                for seed_candidate in audit_inputs:
                    seed_score,seed_source,_,seed_actions,seed_boxes,seed_meta=(
                        seed_candidate)
                    audit=self._audit_joint_seed(start,goal,seed_actions)
                    minlp_seed_audits.append((seed_source,audit))
                    if not audit["valid"]:
                        seed_reject_counts[audit["reason"]]=(
                            seed_reject_counts.get(audit["reason"],0)+1)
                        if seed_source=="coarse_seed":
                            coarse_fallback=None
                        rospy.logdebug(
                            "[v3 optimizer] MINLP seed preflight rejected "
                            "source=%s reason=%s knots=%d score=%.6f",
                            seed_source,audit["reason"],audit["knots"],
                            audit["score"])
                        continue
                    audited_candidate=(
                        audit["score"],seed_source+"_compressed",
                        audit["poses"],audit["actions"],seed_boxes,
                        dict(seed_meta,minlp_seed_audit={
                            "knots":audit["knots"],
                            "mode_switches":audit["mode_switches"],
                            "position_error":audit["position_error"],
                            "heading_error":audit["heading_error"],
                            "min_clearance":audit["min_clearance"]}))
                    if seed_source=="coarse_seed":
                        coarse_fallback=audited_candidate
                    else:
                        eligible_minlp_seeds.append(audited_candidate)
                anchor_profile["minlp_seed_candidates_audited"]=len(
                    minlp_seed_audits)
                anchor_profile["minlp_seed_candidates_eligible"]=len(
                    eligible_minlp_seeds)
                anchor_profile["minlp_seed_reject_swept_collision"]=(
                    seed_reject_counts["swept_collision_after_collapse"])
                anchor_profile["minlp_seed_reject_terminal"]=(
                    seed_reject_counts["terminal_after_collapse"])
                anchor_profile["minlp_seed_reject_knot_limit"]= (
                    seed_reject_counts["knot_limit"])
                anchor_profile["minlp_seed_reject_direction"]= (
                    seed_reject_counts["direction_constraint"])
                # Compare only preflighted compressed candidates. The coarse
                # route remains a single fallback option; Bonmin is still
                # launched at most once per planning request.
                preferred_seed=None
                if self.joint_minlp_seed_source=="coarse":
                    preferred_seed=coarse_fallback
                if preferred_seed is None and eligible_minlp_seeds:
                    preferred_seed=min(eligible_minlp_seeds,
                                       key=lambda item:item[0])
                if preferred_seed is None:
                    preferred_seed=coarse_fallback
                if preferred_seed is not None:
                    minlp_seed_score=preferred_seed[0]
                    minlp_seeds=[preferred_seed]
                    minlp_seen.add(tuple(
                        (a.mode,round(a.amount,4),round(a.control,4))
                        for a in preferred_seed[3]))
                    selected_audit=next((audit for source,audit in
                        minlp_seed_audits
                        if preferred_seed[1]==source+"_compressed"),None)
                    if selected_audit is not None:
                        anchor_profile["minlp_seed_selected_knots"]=(
                            selected_audit["knots"])
                        anchor_profile["minlp_seed_selected_switches"]=(
                            selected_audit["mode_switches"])
                        anchor_profile["minlp_seed_selected_clearance"]= (
                            selected_audit["min_clearance"])
                    rospy.loginfo(
                        "[v3 optimizer] MINLP seed selected source=%s "
                        "score=%.6f knots=%d mode_switches=%d "
                        "min_clearance=%.3f after compressed swept/terminal audit",
                        preferred_seed[1],preferred_seed[0],
                        (selected_audit or {}).get("knots",0),
                        (selected_audit or {}).get("mode_switches",0),
                        (selected_audit or {}).get("min_clearance",0.0))
                if self.joint_seed_replay_file:
                    try:
                        frozen_score,frozen_poses,frozen_actions=(
                            self._read_joint_seed_replay(start,goal))
                        frozen_audit=self._audit_joint_seed(
                            start,goal,frozen_actions)
                        if not frozen_audit["valid"]:
                            raise ValueError(
                                "compressed_seed_invalid:"+
                                frozen_audit["reason"])
                        minlp_seed_score=frozen_audit["score"]
                        minlp_seeds=[(
                            frozen_audit["score"],
                            "frozen_seed_replay_compressed",
                            frozen_audit["poses"],frozen_audit["actions"],[],{
                                "minlp_seed_audit":{
                                    "knots":frozen_audit["knots"],
                                    "mode_switches":frozen_audit[
                                        "mode_switches"],
                                    "position_error":frozen_audit[
                                        "position_error"],
                                    "heading_error":frozen_audit[
                                        "heading_error"],
                                    "min_clearance":frozen_audit[
                                        "min_clearance"]}})]
                        rospy.loginfo(
                            "[v3 optimizer] joint seed replay accepted after "
                            "compressed swept/terminal audit path=%s "
                            "score=%.6f knots=%d min_clearance=%.3f",
                            self.joint_seed_replay_file,frozen_audit["score"],
                            frozen_audit["knots"],
                            frozen_audit["min_clearance"])
                    except Exception as exc:
                        rospy.logwarn(
                            "[v3 optimizer] joint seed replay rejected "
                            "path=%s: %s; using selected local seed",
                            self.joint_seed_replay_file,exc)
                if self.joint_mode_neighborhood_seed_hint and minlp_seeds:
                    try:
                        hinted_seed,hint_profile=(
                            self._mode_neighborhood_seed_hint(
                                start,goal,minlp_seeds[0]))
                        minlp_seeds=[hinted_seed]
                        minlp_seed_score=hinted_seed[0]
                        anchor_profile.update(hint_profile)
                    except Exception as exc:
                        rospy.logwarn(
                            "[v3 optimizer] MINLP mode seed hint unavailable: "
                            "%s; using audited seed",exc)
                for (joint_seed_score,joint_seed_source,_,joint_seed_actions,
                     _,_) in minlp_seeds:
                    joint_begin=time.perf_counter()
                    self._write_joint_seed_trace(
                        msg.plan_id,start,goal,joint_seed_source,
                        joint_seed_score,joint_seed_actions)
                    joint_seed_mode_switches=sum(
                        a.mode!=b.mode for a,b in
                        zip(joint_seed_actions,joint_seed_actions[1:]))
                    joint_seed_knot_estimate=len(_collapse_for_joint(
                        start,joint_seed_actions,
                        max_translation=self.joint_max_translation,
                        max_spin=math.pi,
                        ackermann_merge_tolerance=(
                            self.joint_minlp_ackermann_merge_tolerance),
                        oblique_merge_tolerance=(
                            self.joint_minlp_oblique_merge_tolerance))[0])
                    joint_time_limit=float(self.joint_time_limit)
                    if (self.joint_adaptive_time_limit and
                            joint_seed_mode_switches==1 and
                            joint_seed_knot_estimate<=
                            self.joint_single_switch_max_knots):
                        joint_time_limit=max(
                            joint_time_limit,
                            self.joint_single_switch_time_limit)
                    joint_subproblem_time_limit=float(
                        self.joint_subproblem_time_limit)
                    if self.joint_subproblem_time_adaptive:
                        if joint_seed_mode_switches==0:
                            joint_subproblem_time_limit=min(
                                joint_subproblem_time_limit,2.0)
                        elif joint_seed_mode_switches>1:
                            joint_subproblem_time_limit=min(
                                joint_subproblem_time_limit,
                                self.joint_subproblem_multi_switch_limit)
                    try:
                        poses,refined,boxes,meta=joint_refine(
                            start,goal,joint_seed_actions,self.grid,
                            self.xy_tol,self.yaw_tol,
                            max_translation=self.joint_max_translation,
                            max_knots=self.joint_max_knots,
                            time_limit=joint_time_limit,
                            algorithm=self.joint_algorithm,
                            variable_selection=self.joint_variable_selection,
                            node_comparison=self.joint_node_comparison,
                            subproblem_time_limit=joint_subproblem_time_limit,
                            bonmin_iteration_limit=(
                                self.joint_bonmin_iteration_limit),
                            bonmin_subproblem_iteration_limit=(
                                self.joint_bonmin_subproblem_iteration_limit),
                            bonmin_unsolved_nlp_retries=(
                                self.joint_bonmin_unsolved_nlp_retries),
                            use_seed_cutoff=self.joint_seed_cutoff,
                            geometry_yaw_bounds=(
                                self.joint_geometry_yaw_bounds),
                            geometry_spin_pruning=(
                                self.joint_geometry_spin_pruning),
                            geometry_oblique_bounds=(
                                self.joint_geometry_oblique_bounds),
                            use_rins_heuristic=self.joint_rins_heuristic,
                            use_cbc_seed_incumbent=(
                                self.joint_use_cbc_seed_incumbent),
                            collision_spacing=self.joint_collision_spacing,
                            adaptive_collision_refinement=(
                                self.joint_adaptive_collision_refinement),
                            corridor_clearance_slack=(
                                self.joint_minlp_corridor_clearance_slack),
                            use_feasibility_pump=self.joint_feasibility_pump,
                            use_minlp_pump=self.joint_minlp_pump,
                            root_resolves=self.joint_root_resolves,
                            dive_fractional=self.joint_dive_fractional,
                            random_seed=self.joint_random_seed,
                            use_feasible_seed_warm_start=(
                                self.joint_feasible_seed_warm_start),
                            warm_start_collision_retries=(
                                self.joint_warm_start_collision_retries),
                            collision_retries=self.joint_collision_retries,
                            collision_repair_iterations=(
                                self.joint_collision_repair_iterations),
                            collision_repair_time_limit=(
                                self.joint_collision_repair_time_limit),
                            warm_start_time_limit=(
                                self.joint_warm_start_time_limit),
                            warm_start_early_accept_improvement=(
                                self.joint_warm_start_early_accept_improvement),
                            terminal_xy_tol=self.joint_terminal_xy_tol,
                            terminal_yaw_tol=self.joint_terminal_yaw_tol,
                            terminal_xy_weight=self.joint_terminal_xy_weight,
                            terminal_yaw_weight=self.joint_terminal_yaw_weight,
                            ackermann_merge_tolerance=(
                                self.joint_minlp_ackermann_merge_tolerance),
                            oblique_merge_tolerance=(
                                self.joint_minlp_oblique_merge_tolerance),
                            weights=dict(self.cost_weights,
                                         track=self.joint_minlp_track_weight))
                        score=sequence_cost(refined,self.cost_weights)["score"]
                        if score>=minlp_seed_score-1e-7:
                            rospy.loginfo(
                                "[v3 optimizer] joint seed did not improve "
                                "best local score seed=%.6f result=%.6f "
                                "source=%s solver=%s bonmin_status=%s "
                                "fallback=%s bonmin_returned=%s schedule_repair=%s "
                                "rins=%s "
                                "timing_s={bonmin:%.2f,warm:%.2f,total:%.2f} "
                                "root_feasible=%s root_violations=%d "
                                "root_max_violation=%.3g "
                                "debug_modes=%s debug_integer_gap=%.3g "
                                "debug_terminal_error=%.3g model_obj=%.6f "
                                "elapsed=%.2fs",
                                minlp_seed_score,score,joint_seed_source,
                                meta.get("solver","unknown"),
                                meta.get("bonmin_status",
                                         meta.get("solver_status","unknown")),
                                meta.get("fallback_reason","none"),
                                meta.get("bonmin_returned_modes",
                                         meta.get("bonmin_debug_last_mode_word","n/a")),
                                meta.get("bonmin_schedule_repair_error","none"),
                                bool(meta.get("bonmin_rins_heuristic_enabled",
                                              self.joint_rins_heuristic)),
                                float(meta.get("bonmin_solve_seconds",
                                               meta.get("solve_seconds",0.0))),
                                float(meta.get("bonmin_warm_start_wall_seconds",
                                               meta.get("warm_start_wall_seconds",0.0))),
                                time.perf_counter()-joint_begin,
                                meta.get("bonmin_root_initial_point_feasible","n/a"),
                                int(meta.get("bonmin_root_initial_violation_count",-1)),
                                float(meta.get("bonmin_root_initial_violation_max",-1.0)),
                                meta.get("bonmin_debug_last_mode_word","n/a"),
                                float(meta.get("bonmin_debug_mode_integrality_gap",
                                               -1.0) or 0.0),
                                float(meta.get("bonmin_debug_terminal_position_error",
                                               -1.0) or 0.0),
                                float(meta.get("objective",float("nan"))),
                                time.perf_counter()-joint_begin)
                        meta["common_score"]=score
                        modes="".join(str(a.mode) for i,a in enumerate(refined)
                                      if i==0 or a.mode!=refined[i-1].mode)
                        rospy.loginfo(
                            "[v3 optimizer] joint candidate solver=%s "
                            "score=%.6f model_obj=%.6f seed_model_obj=%.6f "
                            "model_residual={count:%s,max:%.3g} "
                            "seed_score=%.6f seed_source=%s "
                            "modes=%s bonmin_returned=%s rins=%s "
                            "bonmin_status=%s fallback=%s schedule_repair=%s "
                            "timing_s={bonmin:%.2f,warm:%.2f,total:%.2f} "
                            "budget_s={requested:%.2f,setup:%.2f,solver_limit:%.2f,"
                            "node_cap:%.2f,pump:%.2f} "
                            "knots=%d spacing=%.2f seed_mode_switches=%d "
                            "root_feasible=%s root_violations=%d "
                            "root_max_violation=%.3g root_records=%s cutoff=%s "
                            "unsolved_nlp_retries=%d "
                            "model_obj_recomputed=%.9g model_obj_delta=%.3g "
                            "model_terms=%s",
                            meta.get("solver","unknown"),score,
                            float(meta.get("objective",float("nan"))),
                            float(meta.get("seed_objective",float("nan"))),
                            meta.get("model_constraint_violation_count","n/a"),
                            float(meta.get("model_constraint_violation_max",
                                           float("nan")) or 0.0),
                            joint_seed_score,joint_seed_source,modes,
                            meta.get("bonmin_returned_modes",
                                     meta.get("bonmin_debug_last_mode_word","n/a")),
                            bool(meta.get("bonmin_rins_heuristic_enabled",
                                          self.joint_rins_heuristic)),
                            meta.get("bonmin_status",
                                     meta.get("solver_status","unknown")),
                            meta.get("fallback_reason","none"),
                            meta.get("bonmin_schedule_repair_error","none"),
                            float(meta.get("bonmin_solve_seconds",
                                           meta.get("solve_seconds",0.0))),
                            float(meta.get("bonmin_warm_start_wall_seconds",
                                           meta.get("warm_start_wall_seconds",0.0))),
                            time.perf_counter()-joint_begin,
                            float(meta.get("requested_time_limit",0.0)),
                            float(meta.get("model_setup_seconds",0.0)),
                            float(meta.get("effective_solver_time_limit",0.0)),
                            float(meta.get("subproblem_wall_time_limit",0.0)),
                            float(meta.get("minlp_pump_time_limit",0.0)),
                            int(meta.get("knots",0)),
                            self.joint_collision_spacing,
                            joint_seed_mode_switches,
                            meta.get("bonmin_root_initial_point_feasible",
                                     meta.get("initial_point_feasible","n/a")),
                            int(meta.get("bonmin_root_initial_violation_count",
                                         meta.get("initial_violation_count",-1))),
                            float(meta.get("bonmin_root_initial_violation_max",
                                           meta.get("initial_violation_max",-1.0))),
                            meta.get("bonmin_root_initial_violation_records",
                                     meta.get("initial_violation_records",[])),
                            meta.get("bonmin_root_seed_cutoff_enabled",
                                     meta.get("seed_cutoff_enabled","n/a")),
                            int(meta.get("bonmin_unsolved_nlp_retries",0)),
                            float(meta.get("model_objective_recomputed",
                                           float("nan"))),
                            float(meta.get("model_objective_delta",
                                           float("nan"))),
                            json.dumps(meta.get("model_objective_components",{}),
                                       sort_keys=True,separators=(",",":")))
                        rospy.loginfo(
                            "[v3 optimizer] joint geometry bounds enabled=%s data=%s",
                            bool(meta.get("geometry_derived_mode_bounds",{}).get(
                                "enabled",False)),
                            json.dumps(meta.get("geometry_derived_mode_bounds",{}),
                                       sort_keys=True,separators=(",",":")))
                        solver=meta.get("solver")
                        source=("joint_minlp" if solver=="bonmin_minlp" else
                                "joint_minlp_sat_repair" if
                                solver=="ipopt_fixed_schedule_repair" else
                                "joint_warm_start_refine" if
                                solver=="ipopt_fixed_schedule_early_accept" else
                                "joint_seed_fallback")
                        candidates.append((score,source,poses,refined,
                                           boxes,meta))
                        post_begin=time.perf_counter()
                        post_skip_reason="run_neighborhood"
                        seed_complexity=_mode_complexity(joint_seed_actions)
                        candidate_complexity=_mode_complexity(refined)
                        model_residual=meta.get(
                            "model_constraint_violation_max")
                        model_delta=meta.get("model_objective_delta")
                        model_objective=meta.get("objective")
                        audit_finite=(
                            model_objective is not None and
                            math.isfinite(float(model_objective)) and
                            model_delta is not None and
                            math.isfinite(float(model_delta)) and
                            abs(float(model_delta))<=1e-5 and
                            model_residual is not None and
                            math.isfinite(float(model_residual)) and
                            float(model_residual)<=1e-5)
                        relative_improvement=(
                            (minlp_seed_score-score)/
                            max(1e-9,abs(minlp_seed_score)))
                        complexity_no_worse=all(
                            candidate<=seed for candidate,seed in
                            zip(candidate_complexity,seed_complexity))
                        strong_minlp_improvement=(
                            solver=="bonmin_minlp" and audit_finite and
                            relative_improvement>=
                            self.joint_post_neighbor_skip_improvement and
                            complexity_no_worse and
                            not has_reverse_motion(refined))
                        if self.joint_mode_neighbor_candidates<=0:
                            post_skip_reason="disabled"
                            post_neighbor_profile["post_neighbor_skip_code"]=2
                        elif strong_minlp_improvement:
                            post_skip_reason="strong_minlp_improvement"
                            post_neighbor_profile["post_neighbor_skip_code"]=1
                        elif solver!="bonmin_minlp":
                            post_skip_reason="no_bonmin_candidate"
                        post_neighbor_profile[
                            "post_neighbor_relative_improvement"]=(
                                float(relative_improvement))
                        if (self.joint_mode_neighbor_candidates>0 and
                                not strong_minlp_improvement):
                            neighbor_bases=(("optimized",refined),
                                            ("minlp_seed",joint_seed_actions))
                            for (neighbor_base,neighbor_base_actions) in neighbor_bases:
                                neighbors=mode_schedule_neighbor_candidates(
                                    start,neighbor_base_actions,self.grid,
                                    max_candidates=self.joint_mode_neighbor_candidates,
                                    max_window=self.joint_mode_neighbor_window,
                                    weights=self.cost_weights,sweep_step=.005)
                                post_neighbor_profile[
                                    "post_neighbor_schedules_generated"]+=len(neighbors)
                                for (neighbor_seed_score,neighbor_seed_actions,
                                     _,neighbor_info) in neighbors:
                                    neighbor_joint_actions,_=_collapse_for_joint(
                                        start,neighbor_seed_actions,
                                        self.joint_max_translation,
                                        max_spin=math.pi,
                                        ackermann_merge_tolerance=(
                                            self.joint_minlp_ackermann_merge_tolerance),
                                        oblique_merge_tolerance=(
                                            self.joint_minlp_oblique_merge_tolerance))
                                    neighbor_key=canonical_joint_schedule_key(
                                        start,neighbor_joint_actions,
                                        max_translation=self.joint_max_translation,
                                        ackermann_merge_tolerance=(
                                            self.joint_minlp_ackermann_merge_tolerance),
                                        oblique_merge_tolerance=(
                                            self.joint_minlp_oblique_merge_tolerance))
                                    if neighbor_key is None:
                                        continue
                                    if neighbor_key in explored_mode_schedules:
                                        post_neighbor_profile[
                                            "post_neighbor_skipped_duplicate"]+=1
                                        continue
                                    explored_mode_schedules.add(neighbor_key)
                                    _,neighbor_schedule=(
                                        _joint_seed_mode_gear_schedule(
                                            neighbor_joint_actions))
                                    try:
                                        post_neighbor_profile[
                                            "post_neighbor_schedules_solved"]+=1
                                        (neighbor_poses,neighbor_actions,
                                         neighbor_boxes,neighbor_meta)=joint_refine(
                                            start,goal,neighbor_seed_actions,
                                            self.grid,self.xy_tol,self.yaw_tol,
                                            max_translation=(
                                                self.joint_max_translation),
                                            max_knots=self.joint_max_knots,
                                            time_limit=(
                                                self.joint_mode_neighbor_time_limit),
                                            terminal_xy_tol=(
                                                self.joint_terminal_xy_tol),
                                            terminal_yaw_tol=(
                                                self.joint_terminal_yaw_tol),
                                            terminal_xy_weight=(
                                                self.joint_terminal_xy_weight),
                                            terminal_yaw_weight=(
                                                self.joint_terminal_yaw_weight),
                                            weights=self.cost_weights,
                                            collision_spacing=(
                                                self.joint_collision_spacing),
                                            adaptive_collision_refinement=(
                                                self.joint_adaptive_collision_refinement),
                                            corridor_clearance_slack=(
                                                self.joint_corridor_clearance_slack),
                                            use_feasible_seed_warm_start=False,
                                            collision_retries=0,
                                            collision_repair_iterations=0,
                                            _fixed_schedule=neighbor_schedule,
                                            _initial_actions=neighbor_joint_actions)
                                        neighbor_score=sequence_cost(
                                            neighbor_actions,
                                            self.cost_weights)["score"]
                                        # Compare against the best safe incumbent
                                        # from the local stage. A weak/no-op MINLP
                                        # result must not hide a useful schedule
                                        # neighbor behind its own score.
                                        if (neighbor_score<minlp_seed_score-1e-6 and
                                                neighbor_score<neighbor_seed_score-1e-6):
                                            neighbor_meta["solver"]=(
                                                "ipopt_mode_neighborhood")
                                            neighbor_meta["fallback_reason"]=(
                                                "fixed_schedule_mode_neighbor")
                                            neighbor_meta["bonmin_status"]=(
                                                "NOT_RUN_fixed_schedule_neighbor")
                                            neighbor_meta["bonmin_rins_heuristic_enabled"]=(
                                                False)
                                            neighbor_meta["mode_neighbor"]=(
                                                dict(neighbor_info,
                                                     base=neighbor_base))
                                            neighbor_meta["mode_neighbor_seed_score"]=(
                                                float(neighbor_seed_score))
                                            neighbor_meta["common_score"]=(
                                                neighbor_score)
                                            candidates.append((
                                                neighbor_score,
                                                "joint_mode_neighborhood",
                                                neighbor_poses,neighbor_actions,
                                                neighbor_boxes,neighbor_meta))
                                            post_neighbor_profile[
                                                "post_neighbor_schedules_accepted"]+=1
                                            rospy.loginfo(
                                                "[v3 optimizer] mode-neighborhood "
                                                "candidate score=%.6f seed=%.6f model_obj=%.6f "
                                                "base=%s from_mode=%d to_mode=%d "
                                                "window=%d:%d knots=%d "
                                                "solver=ipopt_fixed_schedule swept=checked",
                                                neighbor_score,neighbor_seed_score,
                                                float(neighbor_meta.get(
                                                    "objective",float("nan"))),
                                                neighbor_base,
                                                neighbor_info["from_mode"],
                                                neighbor_info["to_mode"],
                                                neighbor_info["first"],
                                                neighbor_info["last"],
                                                int(neighbor_meta.get("knots",0)))
                                    except Exception as neighbor_exc:
                                        rospy.logdebug(
                                            "[v3 optimizer] mode-neighborhood "
                                            "rejected base=%s window=%d:%d: %s",
                                            neighbor_base,
                                            neighbor_info["first"],
                                            neighbor_info["last"],
                                            str(neighbor_exc).splitlines()[-1])
                        if post_neighbor_profile["post_neighbor_skip_code"]==0:
                            stage_result=(
                                "duplicate_schedules_only" if
                                post_neighbor_profile[
                                    "post_neighbor_schedules_solved"]==0 else
                                "neighborhood_evaluated")
                            if post_skip_reason!="run_neighborhood":
                                post_skip_reason+=";"+stage_result
                            else:
                                post_skip_reason=stage_result
                        post_neighbor_profile["post_neighbor_time_s"]=(
                            time.perf_counter()-post_begin)
                        anchor_profile.update(post_neighbor_profile)
                        rospy.loginfo(
                            "[v3 optimizer] post mode-neighborhood decision="
                            "%s relative_improvement=%.5f threshold=%.5f "
                            "complexity=%s->%s generated=%d solved=%d "
                            "accepted=%d duplicate=%d elapsed=%.3fs",
                            post_skip_reason,relative_improvement,
                            self.joint_post_neighbor_skip_improvement,
                            seed_complexity,candidate_complexity,
                            post_neighbor_profile[
                                "post_neighbor_schedules_generated"],
                            post_neighbor_profile[
                                "post_neighbor_schedules_solved"],
                            post_neighbor_profile[
                                "post_neighbor_schedules_accepted"],
                            post_neighbor_profile[
                                "post_neighbor_skipped_duplicate"],
                            post_neighbor_profile["post_neighbor_time_s"])
                        break
                    except Exception as exc:
                        errors.append("joint_seed=%s:%s" % (
                            joint_seed_source,str(exc).splitlines()[-1]))
                        rospy.logwarn(
                            "[v3 optimizer] joint candidate unavailable "
                            "seed_source=%s: %s",joint_seed_source,errors[-1])
                        if (not self.joint_seed_replay_file and
                                self.joint_minlp_seed_source!="coarse" and
                                joint_seed_source!="coarse_seed" and
                                coarse_fallback is not None and
                                "joint_seed_sat_collision" in str(exc)):
                            coarse_key=tuple(
                                (a.mode,round(a.amount,4),round(a.control,4))
                                for a in coarse_fallback[3])
                            if coarse_key not in minlp_seen:
                                minlp_seen.add(coarse_key)
                                minlp_seeds.append(coarse_fallback)
                                rospy.logwarn(
                                    "[v3 optimizer] retrying joint MINLP "
                                    "with validated coarse seed after swept "
                                    "collision in best-local seed score=%.6f",
                                    coarse_fallback[0])
            if self.joint_enabled and self.joint_global_rounds>0 and candidates:
                round_seed=min(candidates,key=lambda item:item[0])
                round_seen=set(minlp_seen)
                for round_index in range(self.joint_global_rounds):
                    (round_seed_score,round_seed_source,_,round_seed_actions,
                     _,_)=round_seed
                    if round_seed_score>=minlp_seed_score-1e-6:
                        break
                    if (has_reverse_motion(round_seed_actions) or
                            sequence_cost(round_seed_actions,self.cost_weights)[
                                "direction_switches"]>0):
                        rospy.logwarn(
                            "[v3 optimizer] iterated global MINLP skipped "
                            "unsafe seed_source=%s reverse=%s direction_switches=%d",
                            round_seed_source,
                            has_reverse_motion(round_seed_actions),
                            sequence_cost(round_seed_actions,self.cost_weights)[
                                "direction_switches"])
                        break
                    round_key=tuple((a.mode,round(a.amount,4),
                                     round(a.control,4))
                                    for a in round_seed_actions)
                    if round_key in round_seen:
                        break
                    round_seen.add(round_key)
                    round_switches=sum(
                        a.mode!=b.mode for a,b in
                        zip(round_seed_actions,round_seed_actions[1:]))
                    round_knot_estimate=len(_collapse_for_joint(
                        start,round_seed_actions,
                        max_translation=self.joint_max_translation,
                        max_spin=math.pi,
                        ackermann_merge_tolerance=(
                            self.joint_minlp_ackermann_merge_tolerance),
                        oblique_merge_tolerance=(
                            self.joint_minlp_oblique_merge_tolerance))[0])
                    round_time_limit=float(self.joint_time_limit)
                    if (self.joint_adaptive_time_limit and
                            round_switches==1 and
                            round_knot_estimate<=
                            self.joint_single_switch_max_knots):
                        round_time_limit=max(
                            round_time_limit,
                            self.joint_single_switch_time_limit)
                    round_subproblem_limit=float(
                        self.joint_subproblem_time_limit)
                    if self.joint_subproblem_time_adaptive:
                        if round_switches==0:
                            round_subproblem_limit=min(
                                round_subproblem_limit,2.0)
                        elif round_switches>1:
                            round_subproblem_limit=min(
                                round_subproblem_limit,
                                self.joint_subproblem_multi_switch_limit)
                    round_begin=time.perf_counter()
                    try:
                        (round_poses,round_actions,round_boxes,
                         round_meta)=joint_refine(
                            start,goal,round_seed_actions,self.grid,
                            self.xy_tol,self.yaw_tol,
                            max_translation=self.joint_max_translation,
                            max_knots=self.joint_max_knots,
                            time_limit=round_time_limit,
                            algorithm=self.joint_algorithm,
                            variable_selection=self.joint_variable_selection,
                            node_comparison=self.joint_node_comparison,
                            subproblem_time_limit=round_subproblem_limit,
                            bonmin_iteration_limit=(
                                self.joint_bonmin_iteration_limit),
                            bonmin_subproblem_iteration_limit=(
                                self.joint_bonmin_subproblem_iteration_limit),
                            bonmin_unsolved_nlp_retries=(
                                self.joint_bonmin_unsolved_nlp_retries),
                            use_seed_cutoff=self.joint_seed_cutoff,
                            use_rins_heuristic=self.joint_rins_heuristic,
                            use_cbc_seed_incumbent=(
                                self.joint_use_cbc_seed_incumbent),
                            collision_spacing=self.joint_collision_spacing,
                            adaptive_collision_refinement=(
                                self.joint_adaptive_collision_refinement),
                            corridor_clearance_slack=(
                                self.joint_minlp_corridor_clearance_slack),
                            use_feasibility_pump=self.joint_feasibility_pump,
                            use_minlp_pump=self.joint_minlp_pump,
                            root_resolves=self.joint_root_resolves,
                            dive_fractional=self.joint_dive_fractional,
                            random_seed=self.joint_random_seed,
                            use_feasible_seed_warm_start=(
                                self.joint_feasible_seed_warm_start),
                            warm_start_collision_retries=(
                                self.joint_warm_start_collision_retries),
                            collision_retries=self.joint_collision_retries,
                            collision_repair_iterations=(
                                self.joint_collision_repair_iterations),
                            collision_repair_time_limit=(
                                self.joint_collision_repair_time_limit),
                            warm_start_time_limit=(
                                self.joint_warm_start_time_limit),
                            warm_start_early_accept_improvement=(
                                self.joint_warm_start_early_accept_improvement),
                            terminal_xy_tol=self.joint_terminal_xy_tol,
                            terminal_yaw_tol=self.joint_terminal_yaw_tol,
                            terminal_xy_weight=self.joint_terminal_xy_weight,
                            terminal_yaw_weight=self.joint_terminal_yaw_weight,
                            ackermann_merge_tolerance=(
                                self.joint_minlp_ackermann_merge_tolerance),
                            oblique_merge_tolerance=(
                                self.joint_minlp_oblique_merge_tolerance),
                            weights=dict(self.cost_weights,
                                         track=self.joint_minlp_track_weight))
                        round_score=sequence_cost(
                            round_actions,self.cost_weights)["score"]
                        round_meta["common_score"]=round_score
                        round_meta["global_round"]=round_index+1
                        round_seed_modes="".join(
                            str(a.mode) for a in round_seed_actions)
                        round_result_modes="".join(
                            str(a.mode) for a in round_actions)
                        candidates.append((
                            round_score,"iterated_joint_minlp",round_poses,
                            round_actions,round_boxes,round_meta))
                        rospy.loginfo(
                            "[v3 optimizer] iterated global MINLP round=%d "
                            "seed=%.6f result=%.6f seed_model_obj=%.6f "
                            "model_obj=%.6f seed_modes=%s result_modes=%s "
                            "status=%s elapsed=%.2fs",
                            round_index+1,round_seed_score,round_score,
                            float(round_meta.get("seed_objective",float("nan"))),
                            float(round_meta.get("objective",float("nan"))),
                            round_seed_modes,round_result_modes,
                            round_meta.get("bonmin_status",
                                           round_meta.get("solver_status","unknown")),
                            time.perf_counter()-round_begin)
                        if round_score>=round_seed_score-1e-6:
                            break
                        round_seed=(round_score,"iterated_joint_minlp",
                                    round_poses,round_actions,round_boxes,
                                    round_meta)
                    except Exception as round_exc:
                        rospy.logwarn(
                            "[v3 optimizer] iterated global MINLP round=%d "
                            "seed_source=%s unavailable: %s",
                            round_index+1,round_seed_source,
                            str(round_exc).splitlines()[-1])
                        break
            repaired=False
            coarse_seed_needed=not candidates
            # Always retain the coarse route as an independently checked
            # fallback so local optimizer failures cannot discard a known
            # swept-safe incumbent.
            try:
                coarse_poses,coarse_actions=expand(
                    start,collapse_exact_runs(actions),max_translation=.50,
                    max_rotation=math.radians(15))
                if self.planning_backend == "legacy":
                    if not legacy_direction_allowed(
                            coarse_actions,self.allow_reverse,
                            self.allow_shunting):
                        raise ValueError("coarse_seed_direction_policy_forbidden")
                elif sequence_cost(
                        coarse_actions,self.cost_weights)[
                            "direction_switches"]>0:
                    raise ValueError("coarse_seed_direction_switch_forbidden")
                if not self.grid.trajectory_clear(coarse_poses,coarse_actions):
                    raise ValueError("coarse_seed_python_swept_sat_failed")
                if (math.hypot(coarse_poses[-1].x-goal.x,
                               coarse_poses[-1].y-goal.y)>self.xy_tol+1e-5 or
                    abs(wrap(coarse_poses[-1].th-goal.th))>self.yaw_tol+1e-5):
                    raise ValueError("coarse_seed_terminal_tolerance_failed")
                _,coarse_boxes,_=build_corridors(self.grid,coarse_poses)
                meta={"corridor":"coarse_seed_validated",
                      "solver":"none_safe_incumbent",
                      "common_score":sequence_cost(
                          coarse_actions,self.cost_weights)["score"]}
                candidates.append((meta["common_score"],"coarse_seed",
                                   coarse_poses,coarse_actions,coarse_boxes,meta))
                if coarse_seed_needed:
                    rospy.logwarn("[v3 optimizer] NLP candidates unavailable; "
                                  "using independently swept-SAT-validated coarse seed")
            except Exception as exc:
                errors.append("coarse_seed="+str(exc).splitlines()[-1])
            if not candidates:
                self.status("optimizer_retry_with_bounded_research id=%d reasons=%s" %
                            (msg.plan_id,";".join(errors)))
                candidate = bounded_research(
                    start, goal, actions, self.grid, self.xy_tol, self.yaw_tol,
                    self.repair_max_iter, self.repair_padding,
                    weights=self.cost_weights,
                    allow_oblique=allow_oblique_candidates)
                if candidate is None:
                    raise RuntimeError("bounded_research_failed_after: %s" % ";".join(errors))
                repaired_actions, repair_nodes = candidate
                poses, refined, boxes, meta = refine(
                    start, goal, repaired_actions, self.grid, self.xy_tol, self.yaw_tol,
                    max_iter=self.optimizer_max_iter,weights=self.cost_weights)
                meta["repair_nodes"] = repair_nodes
                repaired = True
                score=sequence_cost(refined,self.cost_weights)["score"]
                meta["common_score"]=score
                candidates.append((score,"repair",
                                   poses,refined,boxes,meta))
            original_candidate_count=len(candidates)
            if self.planning_backend == "legacy":
                candidates=[candidate for candidate in candidates
                            if legacy_direction_allowed(
                                candidate[3],self.allow_reverse,
                                self.allow_shunting)]
                if len(candidates)<original_candidate_count:
                    rospy.loginfo(
                        "[v3 optimizer] rejected %d direction-policy proposals",
                        original_candidate_count-len(candidates))
            else:
                candidates=[candidate for candidate in candidates
                            if sequence_cost(
                                candidate[3],self.cost_weights)[
                                    "direction_switches"]==0]
                if len(candidates)<original_candidate_count:
                    rospy.loginfo(
                        "[v3 optimizer] rejected %d direction-switching proposals",
                        original_candidate_count-len(candidates))
                original_candidate_count=len(candidates)
                candidates=[candidate for candidate in candidates
                            if not has_reverse_motion(candidate[3])]
                if len(candidates)<original_candidate_count:
                    rospy.loginfo(
                        "[v3 optimizer] rejected %d reverse-motion proposals",
                        original_candidate_count-len(candidates))
            candidates.sort(key=lambda item:
                item[0]-float(item[5].get("sos_selection_bonus",0.0)))
            chosen=None; sample_error=None
            sample_begin=time.perf_counter()
            for candidate in candidates:
                candidate_score,source,poses,refined,boxes,meta=candidate
                try:
                    timed,sample_dt=safe_time_sample(
                        start,refined,self.grid,self.v_demo,self.omega_demo,
                        self.dt,self.timed_sweep_corner_step)
                    chosen=(source,poses,refined,boxes,meta,timed,sample_dt)
                    break
                except Exception as exc:
                    sample_error=str(exc)
                    rospy.loginfo(
                        "[v3 optimizer] timed-sweep candidate rejected "
                        "source=%s solver=%s score=%.6f reason=%s",
                        source,meta.get("solver","unknown"),candidate_score,
                        sample_error.splitlines()[-1])
                    # A geometry refinement can be swept-clear at optimizer
                    # resolution yet graze an obstacle after time resampling.
                    # Try the same fixed mode sequence with a small extra
                    # footprint margin, then validate the published time path
                    # against the original grid. The extra margin is local to
                    # this recovery attempt; it never weakens acceptance.
                    if (source=="iterated_joint" and
                            sample_error.splitlines()[-1]==
                            "timed_sat_collision_after_adaptive_sampling"):
                        safe_score_limit=min(
                            (item[0] for item in candidates
                             if item is not candidate),
                            default=float("inf"))
                        recovery_seeds=[]
                        local_margin_schedule=(
                            .001,.00125,.0015,.00175,.002,.00225,
                            .0025,.005,.010)
                        target_mode_runs=tuple(
                            action.mode for i,action in enumerate(refined)
                            if i==0 or action.mode!=refined[i-1].mode)
                        # The local mode/geometry proposal can already sit in
                        # a narrow local basin. If it cannot gain timed
                        # clearance, retry the same mode-run schedule from
                        # Bonmin's independently safe incumbent geometry.
                        for alternate in candidates:
                            alt_score,alt_source,_,alt_actions,_,_=alternate
                            if (alternate is candidate or alt_source not in
                                    ("joint_minlp","joint_minlp_sat_repair")):
                                continue
                            alt_mode_runs=tuple(
                                action.mode for i,action in enumerate(alt_actions)
                                if i==0 or action.mode!=alt_actions[i-1].mode)
                            if alt_mode_runs==target_mode_runs:
                                # A small clearance increase often removes
                                # interpolation-only contacts. Probe a narrow
                                # band first, then retain the broad local ladder
                                # below as a geometry-independent fallback.
                                recovery_seeds.append((
                                    alt_actions,alt_source,(.00225,.0025)))
                                break
                        # Try Bonmin's independently swept-safe geometry first:
                        # it often needs fewer clearance projections than the
                        # lower-score local proposal. Keep the local proposal
                        # as a second recovery seed so a failed alternate does
                        # not discard its possible score gain.
                        recovery_seeds.append((
                            refined,source,local_margin_schedule))
                        # Probe modest extra margins first; the original-grid
                        # timed sweep remains the only acceptance check.
                        for recovery_actions,recovery_source,margin_schedule in recovery_seeds:
                            for extra_margin in margin_schedule:
                                clearance_grid=Grid(
                                    self.grid.occ,self.grid.res,self.grid.ox,
                                    self.grid.oy,self.length,self.width,
                                    self.margin+extra_margin)
                                try:
                                    repaired_poses,repaired_actions,repaired_boxes,repair_meta=refine(
                                        start,goal,recovery_actions,clearance_grid,
                                        self.xy_tol,self.yaw_tol,
                                        max_iter=max(self.optimizer_max_iter,300),
                                        weights=self.cost_weights,
                                        allow_initial_collision=True,
                                        acceptance_score_limit=safe_score_limit)
                                    repaired_score=sequence_cost(
                                        repaired_actions,self.cost_weights)["score"]
                                    if repaired_score>=safe_score_limit-1e-7:
                                        continue
                                    repaired_timed,repaired_dt=safe_time_sample(
                                        start,repaired_actions,self.grid,
                                        self.v_demo,self.omega_demo,self.dt,
                                        self.timed_sweep_corner_step)
                                    repair_meta.update({
                                        "solver":"ipopt_time_clearance_repair",
                                        "common_score":repaired_score,
                                        "time_clearance_extra_margin":extra_margin,
                                        "time_clearance_seed_source":recovery_source,
                                        "time_clearance_rejected_score":candidate_score})
                                    chosen=("time_clearance_repair",repaired_poses,
                                            repaired_actions,repaired_boxes,
                                            repair_meta,repaired_timed,repaired_dt)
                                    rospy.loginfo(
                                        "[v3 optimizer] time-clearance repair accepted "
                                        "score=%.6f extra_margin=%.4f m seed=%s",
                                        repaired_score,extra_margin,recovery_source)
                                    break
                                except Exception as repair_exc:
                                    rospy.loginfo(
                                        "[v3 optimizer] time-clearance repair rejected "
                                        "extra_margin=%.4f m seed=%s reason=%s",
                                        extra_margin,recovery_source,
                                        str(repair_exc).splitlines()[-1])
                            if chosen is not None:
                                break
                        if chosen is not None:
                            break
            sample_seconds=time.perf_counter()-sample_begin
            if chosen is None:
                # A locally optimized path can be spatially SAT-feasible but
                # fail after time resampling. Preserve the independently safe
                # coarse incumbent as the last timed-trajectory candidate.
                try:
                    coarse_poses,coarse_actions=expand(
                        start,collapse_exact_runs(actions),max_translation=.50,
                        max_rotation=math.radians(15))
                    if self.planning_backend == "legacy":
                        if not legacy_direction_allowed(
                                coarse_actions,self.allow_reverse,
                                self.allow_shunting):
                            raise ValueError("coarse_seed_direction_policy_forbidden")
                    elif sequence_cost(
                            coarse_actions,self.cost_weights)[
                                "direction_switches"]>0:
                        raise ValueError("coarse_seed_direction_switch_forbidden")
                    if not self.grid.trajectory_clear(coarse_poses,coarse_actions):
                        raise ValueError("coarse_seed_python_swept_sat_failed")
                    if (math.hypot(coarse_poses[-1].x-goal.x,
                                   coarse_poses[-1].y-goal.y)>self.xy_tol+1e-5 or
                        abs(wrap(coarse_poses[-1].th-goal.th))>self.yaw_tol+1e-5):
                        raise ValueError("coarse_seed_terminal_tolerance_failed")
                    timed,sample_dt=safe_time_sample(
                        start,coarse_actions,self.grid,self.v_demo,
                        self.omega_demo,self.dt,self.timed_sweep_corner_step)
                    _,coarse_boxes,_=build_corridors(self.grid,coarse_poses)
                    meta={"corridor":"coarse_seed_validated",
                          "solver":"none_safe_incumbent",
                          "common_score":sequence_cost(
                              coarse_actions,self.cost_weights)["score"]}
                    chosen=("coarse_seed",coarse_poses,coarse_actions,
                            coarse_boxes,meta,timed,sample_dt)
                    rospy.logwarn("[v3 optimizer] optimized candidates failed "
                                  "timed sweep; using SAT-validated coarse seed")
                except Exception as exc:
                    raise ValueError("all_candidate_time_samples_invalid: %s; "
                                     "coarse_seed=%s"%
                                     (sample_error,str(exc).splitlines()[-1]))
            source,poses,refined,boxes,meta,timed,sample_dt=chosen
            if source in ("anchor_graph","endpoint_replan","upstream_sos",
                          "upstream_suffix_replan") and not boxes:
                try:
                    _,boxes,covered=build_corridors(self.grid,poses)
                    meta["corridor"]="disc" if covered else "point_fallback"
                except Exception:
                    meta["corridor"]="anchor_graph_only"
            meta["anchor_best_score"]=anchor_best_score
            meta["anchor_candidates"]=anchor_count
            meta["anchor_modes"]=anchor_best_modes
            repaired=source=="repair"
            meta["selected_candidate"]=source
            if math.hypot(timed[-1][1].x-goal.x,timed[-1][1].y-goal.y)>self.xy_tol+1e-5:
                raise ValueError("timed_goal_violation")
            meta["visualization"] = self.publish(
                msg.plan_id, start, poses, refined, timed, boxes, meta)
            selected_score=sequence_cost(refined,self.cost_weights)
            # Capture the actual selected legacy route. The planner's input
            # seed and the MINLP warm seed can both differ from this geometry.
            # Native legacy playback has no restricted dynamic certificate;
            # its geometry is audited independently for same-profile replay.
            geometry_evaluator = TrajectoryEvaluator(
                self.grid, start, goal, self.quality_limits,
                self.length, self.width, .02, math.radians(1),
                self.quality_profile, allow_reverse=self.allow_reverse,
                allow_shunting=self.allow_shunting)
            geometry_audit = geometry_evaluator.evaluate(
                refined, timed=None, allow_reverse=self.allow_reverse)
            final_actions = [[int(a.mode), float(a.amount), float(a.control)]
                             for a in refined]
            legacy_report = {
                "status": "LEGACY_NATIVE_PUBLISHED",
                "geometry_status": geometry_audit.get("geometry_status"),
                "dynamic_status": "NOT_CHECKED_UNDER_RESTRICTED_PROFILE",
                "selected_source": source,
                "final_geometry": {
                    "representation": "legacy_final_actions_v1",
                    "actions": final_actions,
                    "actions_sha256": hashlib.sha256(json.dumps(
                        final_actions, separators=(",", ":")).encode()).hexdigest(),
                    "action_replay_poses": [[p.x, p.y, p.th]
                                            for p in replay(start, refined)],
                    "strict_geometry_audit": geometry_audit,
                },
                "legacy_native_timing": {
                    "contract": "legacy_native_speed_only",
                    "restricted_profile_certified": False,
                    "duration_s": float(timed[-1][0]),
                    "sample_dt_s": float(sample_dt),
                    "timed_trace": [[float(t), p.x, p.y, p.th,
                                     float(v), int(mode)]
                                    for t, p, v, mode in timed],
                },
                "execution_profile_role": "shared_replay_target_only",
                "visualization": meta["visualization"],
                "total_s": time.perf_counter()-begin,
            }
            legacy_report["diagnostic_file"] = self._persist_plan_report(
                legacy_report, int(msg.plan_id), start, goal, actions)
            self.quality_pub.publish(String(data=json.dumps(legacy_report)))
            if source=="upstream_sos":
                anchor_profile["sos_selected"]=1
                anchor_profile["sos_selected_score"]=selected_score["score"]
                for key in ("sos_upstream_distance",
                            "sos_hotspot_maneuvers_before",
                            "sos_hotspot_maneuvers_after",
                            "sos_hotspot_steering_tv_before",
                            "sos_hotspot_steering_tv_after",
                            "sos_selection_bonus"):
                    if key in meta:
                        anchor_profile["selected_"+key]=float(meta[key])
            elif source=="upstream_suffix_replan":
                anchor_profile["upstream_replan_selected"]=1
                anchor_profile["upstream_replan_selected_score"]=selected_score["score"]
                anchor_profile["upstream_replan_selected_pattern_code"]={
                    "pure_a":1,"a_o_a":2,"spin_a":3}.get(
                        meta.get("upstream_replan_pattern"),0)
            oblique_deltas=[abs(wrap(b.control-a.control))
                for a,b in zip(refined,refined[1:])
                if a.mode==1 and b.mode==1 and
                abs(wrap(b.control-a.control))>1e-3]
            cost_parts=",".join("%s:%.3f"%(name,value)
                                 for name,value in selected_score["cost_parts"].items())
            fixed_parts=",".join("%s:%.3f"%(k,v) for k,v in fixed_profile.items()
                                  if isinstance(v,(int,float)))
            fixed_solver="%s/%s"%(fixed_profile.get("nlp_iterations","?"),
                                   fixed_profile.get("nlp_return_status","?"))
            anchor_parts=",".join("%s:%.3f"%(k,v) for k,v in anchor_profile.items()
                                   if isinstance(v,(int,float)))
            self.status("success id=%d source=%s nodes=%d time=%.2fs fixed=%.2fs anchor=%.2fs timed_check=%.2fs fixed_profile={%s} solver=%s anchor_profile={%s} corridor=%s alpha=%.5f common_score=%.3f anchor_score=%.3f anchor_modes=%s anchor_candidates=%d oblique_updates=%d oblique_micro_le_5deg=%d cost_parts={%s} repaired=%s repair_nodes=%d sample_dt=%.5f" %
                        (msg.plan_id,source,len(poses),time.perf_counter()-begin,
                         fixed_seconds,anchor_seconds,sample_seconds,
                         fixed_parts,fixed_solver,anchor_parts,
                         meta["corridor"],meta.get("line_search_alpha",1.0),
                         selected_score["score"],anchor_best_score,
                         anchor_best_modes,anchor_count,len(oblique_deltas),
                         sum(delta<=math.radians(5) for delta in oblique_deltas),
                         cost_parts,repaired,
                         meta.get("repair_nodes",0),sample_dt))
        except Exception as exc:
            self._persist_plan_report({"status": "REJECTED",
                "failure_stage": "legacy_backend",
                "reason": str(exc).splitlines()[-1],
                "total_s": time.perf_counter()-begin}, int(msg.plan_id),
                start, goal, actions)
            rospy.logwarn("[v3 optimizer] rejected plan %d: %s",msg.plan_id,exc)
            rospy.logdebug(traceback.format_exc())
            self.status("rejected id=%d reason=%s" % (msg.plan_id,str(exc).splitlines()[-1]))

    def run_mode_path(self, plan_id, start, goal, seed, begin,
                      auxiliary_seeds=None):
        """Independent refactored backend; legacy stages are not chained here."""
        try:
            evaluator = TrajectoryEvaluator(
                self.grid, start, goal, self.quality_limits, self.length, self.width,
                .02, math.radians(1), self.quality_profile,
                allow_shunting=self.allow_shunting)
            solver_seed = seed
            solver_auxiliary_seeds = list(auxiliary_seeds or [])
            solver_seed_source = "planner_strict_primary"
            provisional = next((item for item in solver_auxiliary_seeds
                if str(item[1]) == "untrusted_shunting_search"), None)
            if provisional is not None:
                # Let the shared Python gate repair this provisional route as
                # an initializer, while retaining the C++ strict route as an
                # independently evaluated fallback candidate.
                solver_seed = provisional[0]
                solver_seed_source = str(provisional[1])
                solver_auxiliary_seeds = [
                    (seed, "planner_strict_primary")]+[
                        item for item in solver_auxiliary_seeds
                        if item is not provisional]
            winner, report = ModePathMaster(evaluator, self.mode_master_config).solve(
                solver_seed, auxiliary_seeds=solver_auxiliary_seeds)
            report["initial_seed_source"] = solver_seed_source
            report["input_seed_candidates"] = [
                {"source": solver_seed_source,
                 "actions": [[a.mode,a.amount,a.control] for a in solver_seed]}]
            report["input_seed_candidates"].extend(
                {"source": str(source),
                 "actions": [[a.mode,a.amount,a.control] for a in actions]}
                for actions,source in solver_auxiliary_seeds)
            actions, timed = winner["actions"], winner["timed"]
            continuous = winner.get("continuous_plan")
            stage_timings = report.setdefault("stage_timings_s", {})
            stage_timings["mode_master_total"] = float(report.get("total_s", 0.0))
            postprocess_begin = time.perf_counter()
            if continuous is not None:
                spatial_rows = replay_continuous(continuous, .10)
                corridor_rows = [spatial_rows[0]]
                for row in spatial_rows[1:-1]:
                    prior = corridor_rows[-1]
                    if (row[0]-prior[0] >= .35 or
                            abs(wrap(row[1].th-prior[1].th)) >= .25 or
                            row[3] != prior[3]):
                        corridor_rows.append(row)
                corridor_rows.append(spatial_rows[-1])
                poses = [row[1] for row in corridor_rows]
                corridor_records = []
                map_signature = self._joint_seed_map_signature()
                map_hash = hashlib.sha256(json.dumps(
                    map_signature, sort_keys=True).encode()).hexdigest()
                _, boxes, covered = build_corridors(
                    self.grid, poses, records=corridor_records,
                    plan_id=int(plan_id), map_hash=map_hash)
                corridor = "disc" if covered else "mixed_disc_point"
                meta = {"solver": "mode_path_v2_continuous_motion",
                        "corridor": corridor,
                        "common_score": winner["audit"]["score"],
                        "continuous_motion_v2": True,
                        "corridor_modes": [row[3] if row[3] is not None else
                                           continuous.blocks[0].mode
                                           for row in corridor_rows],
                        "corridor_records": corridor_records}
                report["corridor"] = {"type": corridor, "stations": len(boxes),
                                      "role": "feature_center_configuration_space_display",
                                      "used_in_continuous_nlp": False,
                                      "safety_certificate": "independent_5mm_sweep",
                                      "map_signature": map_signature,
                                      "map_hash": map_hash,
                                      "physical_vehicle_size_m": [self.length,self.width],
                                      "feature_center_regions_visible": self.show_feature_center_regions,
                                      "margin_contract": "total_dimension_increment_half_per_side",
                                      "records": [dict(record.to_dict(),
                                          mode=meta["corridor_modes"][record.station])
                                          for record in corridor_records],
                                      "footprints": [
                                          {"station": k, "reference_pose":
                                           {"x": p.x, "y": p.y, "th": p.th},
                                           "body_corners_world_m": footprint_corners(
                                               p, self.length, self.width),
                                           "margin_corners_world_m": footprint_corners(
                                               p, 2*self.grid.half_l, 2*self.grid.half_w)}
                                          for k,p in enumerate(poses)]}
                report["selected_continuous_plan"] = [
                    {"mode": block.mode, "direction": block.direction,
                     "slope_policy": block.slope_policy,
                     "length_m": block.length,
                     "signed_angle_rad": block.signed_angle,
                     "knots": [[k.s, k.value, k.derivative]
                               for k in block.knots]}
                    for block in continuous.blocks]
            else:
                poses = replay(start, actions)
                try:
                    _, boxes, covered = build_corridors(self.grid, poses)
                    corridor = "disc" if covered else "mixed_disc_point"
                except ValueError:
                    boxes, corridor = [], "independent_geometry"
                meta = {"solver": "mode_path_v1", "corridor": corridor,
                        "common_score": winner["audit"]["score"]}
            stage_timings["postprocess_and_corridor"] = (
                time.perf_counter()-postprocess_begin)
            source_counts = {}
            for candidate in report.get("candidates", []):
                source = str(candidate.get("source", "unknown"))
                source_counts[source] = source_counts.get(source, 0)+1
            report["candidate_source_counts"] = source_counts
            report.update(plan_id=int(plan_id), status="FEASIBLE",
                          selected_actions=[[a.mode, a.amount, a.control] for a in actions])
            publish_begin = time.perf_counter()
            report["visualization"] = self.publish(
                plan_id, start, poses, actions, timed.samples, boxes, meta)
            stage_timings["trajectory_publish_call"] = time.perf_counter()-publish_begin
            report["optimizer_callback_wall_s"] = time.perf_counter()-begin
            self._persist_plan_report(report,plan_id,start,goal,actions)
            encode_begin = time.perf_counter()
            quality_json = json.dumps(report, allow_nan=False, separators=(",", ":"))
            stage_timings["quality_report_encode"] = time.perf_counter()-encode_begin
            # Re-encode so the report contains the measured first-pass cost.
            quality_json = json.dumps(report, allow_nan=False, separators=(",", ":"))
            self.quality_pub.publish(String(data=quality_json))
            metrics = winner["audit"]["metrics"]
            self.status(
                "success id=%d source=%s nodes=%d time=%.2fs "
                "common_score=%.6f quality_profile=%s validity=FEASIBLE "
                "candidate_source=%s candidates=%d nlp_calls=%d "
                "min_body_clearance=%.4f oblique_updates=%d duration=%.3f" %
                (plan_id, self.planning_backend, len(poses), time.perf_counter()-begin,
                 winner["audit"]["score"],
                 self.quality_profile.name, winner["source"], len(report["candidates"]),
                 report["nlp_calls"], metrics["min_body_clearance_m"],
                 metrics["oblique_adjustments"], timed.duration))
        except Exception as exc:
            rospy.logwarn("[v3 optimizer] %s rejected plan %d: %s",
                          self.planning_backend, plan_id, exc)
            rospy.logdebug(traceback.format_exc())
            failure_report = getattr(exc, "report", {
                "status": "REJECTED", "reason": str(exc),
                "exception_type": type(exc).__name__})
            failure_report.update(
                plan_id=int(plan_id), backend=self.planning_backend,
                optimizer_callback_wall_s=time.perf_counter()-begin,
                start=[start.x,start.y,start.th], goal=[goal.x,goal.y,goal.th],
                seed_actions=[[a.mode,a.amount,a.control] for a in seed],
                map_signature=self._joint_seed_map_signature())
            if failure_report.get("status") == "GEOMETRY_ONLY":
                # Spatial display only. clear() already published an empty
                # TimedTrajectory for this plan; no uncertified time samples
                # are constructed or sent to playback.
                header = Header(stamp=rospy.Time.now(), frame_id="map")
                spatial = failure_report["geometry_incumbent"]["spatial_path"]
                path = Path(header=header)
                path.poses = [pose_stamped(Pose(*p), header.stamp) for p in spatial]
                self.path_pub.publish(path)
                failure_report["geometry_visualization"] = {
                    "role": "geometry_only_not_executable", "frame_id": "map",
                    "stamp_s": header.stamp.to_sec(), "pose_count": len(spatial),
                    "timed_trajectory_published": False}
            self._persist_plan_report(failure_report,plan_id,start,goal,seed)
            self.quality_pub.publish(String(data=json.dumps(
                failure_report, allow_nan=False, separators=(",", ":"))))
            self.status("%s id=%d backend=%s reason=%s" %
                        ("geometry_only" if failure_report.get("status") == "GEOMETRY_ONLY"
                         else "rejected", plan_id, self.planning_backend,
                         str(exc).splitlines()[-1]))

    def publish(self, plan_id, start, poses, actions, timed, boxes, meta):
        header = Header(stamp=rospy.Time.now(), frame_id="map")
        traj = TimedTrajectory(header=header, plan_id=plan_id)
        path = Path(header=header)
        for t, p, v, mode in timed:
            point = TimedPoint()
            point.time_from_start = rospy.Duration.from_sec(t)
            point.center_pose.x,point.center_pose.y,point.center_pose.theta = p.x,p.y,p.th
            point.v = v
            point.mode = mode
            traj.points.append(point)
            path.poses.append(pose_stamped(p, header.stamp + point.time_from_start))
        colors=((.0,.7,1.0),(1.0,.55,.0),(.8,.2,1.0))
        if meta.get("continuous_motion_v2"):
            self.mode_pub.publish(make_timed_mode_strip(
                timed,header,"v3_modes",.06,.95,colors))
        else:
            self.mode_pub.publish(make_mode_strip(
                actions,poses,header,"v3_modes",.06,.95,colors))
        corridor_poses = poses
        corridor_boxes = boxes
        display_sampling = {
            "max_spacing": .30,
            "max_yaw_step": math.radians(8),
            "narrow_half_width": .08,
            "narrow_spacing": .12,
            "extent_change": .06,
        }
        legacy_display_resampled = False
        if self.planning_backend == "legacy":
            # Legacy's published corridor used action endpoints, which are
            # much farther apart than the v1/v2 visualization stations. Build
            # a display-only fine replay here; leave the executable timed path,
            # mode strip and safety certificate unchanged.
            try:
                fine_poses, _ = expand(
                    start, actions, max_translation=.30,
                    max_rotation=math.radians(8))
                _, fine_boxes, _ = build_corridors(self.grid, fine_poses)
                corridor_poses, corridor_boxes = fine_poses, fine_boxes
                display_sampling = {
                    "max_spacing": .30,
                    "max_yaw_step": math.radians(8),
                    "narrow_half_width": .08,
                    "narrow_spacing": .12,
                    "extent_change": .06,
                }
                legacy_display_resampled = True
            except Exception as exc:
                rospy.logwarn("[v3 optimizer] legacy display resampling "
                              "failed; keeping original corridor: %s", exc)
        erase=Marker();erase.header=header;erase.action=Marker.DELETEALL
        corridor_markers=[erase]
        # Points are already in map coordinates: keep marker rotation identity.
        # The corridor row is paired with the actual displayed trajectory pose.
        display_poses=[]
        for k,row in enumerate(corridor_boxes):
            if (self.planning_backend == "legacy" and
                    k < len(corridor_poses)):
                display_poses.append(corridor_poses[k])
                continue
            if meta.get("continuous_motion_v2") and k<len(poses):
                display_poses.append(poses[k])
                continue
            if row:
                reference=Pose(sum(b[0] for b in row)/len(row),
                               sum(b[1] for b in row)/len(row),row[0][4])
            elif k<len(poses):
                display_poses.append(poses[k])
                continue
            else:
                display_poses.append(poses[-1] if poses else
                                      (timed[-1][1] if timed else Pose(0.,0.,0.)))
                continue
            candidates=timed or [(0.,p,0.,0) for p in poses]
            if candidates:
                display_poses.append(min(candidates,key=lambda item:
                    (item[1].x-reference.x)**2+(item[1].y-reference.y)**2+
                    (.45*wrap(item[1].th-reference.th))**2)[1])
            else:
                display_poses.append(reference)
        # Keep the display independent from optimization and 5 mm safety sampling.
        # Core corridor stations remain complete; only their rendered subset changes.
        indices=select_corridor_display_stations(
            display_poses, corridor_boxes, **display_sampling)
        edges=((0,1),(1,2),(2,3),(3,0))
        markers={}
        plan_suffix="_plan_%d"%int(plan_id)

        def append_polygon(namespace, marker_id, color, corners, width, z):
            key=(namespace,marker_id)
            if key not in markers:
                marker=Marker()
                marker.header=header;marker.ns=namespace;marker.id=marker_id
                marker.type=Marker.LINE_LIST;marker.action=Marker.ADD
                marker.pose.orientation.w=1.0;marker.scale.x=width
                marker.color.r,marker.color.g,marker.color.b=color
                marker.color.a=.8
                markers[key]=marker
            for i,j in edges:
                for px,py in (corners[i],corners[j]):
                    markers[key].points.append(Point(x=px,y=py,z=z))

        records={(r.station,r.feature_index):r
                 for r in meta.get("corridor_records",[])}
        for k in indices:
            row=corridor_boxes[k]
            if not row:
                continue
            body_pose=display_poses[k]
            mode=(meta["corridor_modes"][k] if
                  k<len(meta.get("corridor_modes",[])) else
                  actions[min(k,len(actions)-1)].mode if actions else 0)
            append_polygon("v3_vehicle_footprints"+plan_suffix,mode,colors[mode],
                           footprint_corners(body_pose,self.length,self.width),.025,.03)
            append_polygon("v3_vehicle_margin_footprints"+plan_suffix,0,(.7,.7,.7),
                           footprint_corners(body_pose,2*self.grid.half_l,
                                             2*self.grid.half_w),.009,.02)
            for feature_index,box in enumerate(row):
                if not self.show_feature_center_regions:
                    continue
                record=records.get((k,feature_index))
                certified=(record.certification == "disc_clearance_raster_guard"
                           if record else meta.get("corridor")=="disc")
                namespace=("v3_feature_center_disc_regions" if certified else
                           "v3_feature_center_point_or_uncertified_regions")+plan_suffix
                append_polygon(namespace,0,(.1,.85,.4) if certified else (1.,.48,.08),
                               corridor_box_corners(box),.008,.012)
        corridor_markers.extend(markers.values())
        self.corr_pub.publish(MarkerArray(markers=corridor_markers))
        self.time_pub.publish(traj)
        self.path_pub.publish(path)
        return {"schema":"corridor_display_sampling_v1",
                "plan_id":int(plan_id),
                "role":"display_only; does_not_change_optimizer_or_sweep",
                "all_corridor_stations":len(corridor_boxes),
                "displayed_stations":len(indices),
                "station_indices":indices,
                "sampling":{"max_arc_spacing_m":display_sampling["max_spacing"],
                             "max_accumulated_yaw_rad":display_sampling["max_yaw_step"],
                             "narrow_region_min_lateral_half_extent_m":display_sampling["narrow_half_width"],
                             "narrow_region_spacing_m":display_sampling["narrow_spacing"],
                             "max_region_half_extent_change_m":display_sampling["extent_change"]},
                "legacy_display_resampled":legacy_display_resampled,
                "vehicle_size_m":[self.length,self.width],
                "margin_size_m":[2*self.grid.half_l,2*self.grid.half_w],
                "margin_contract":"total_dimension_increment_half_per_side",
                "feature_center_regions_visible":self.show_feature_center_regions,
                "feature_center_regions_used_in_optimization":bool(
                    meta.get("corridor_records") and any(
                        r.used_in_optimization for r in meta["corridor_records"]))}


if __name__ == "__main__":
    rospy.init_node("fwis_v3_optimizer")
    OptimizerNode()
    rospy.spin()
