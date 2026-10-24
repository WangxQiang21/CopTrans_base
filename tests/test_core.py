import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] /
                       "catkin_ws/src/fwis_v3_optimizer/src"))
from fwis_v3_optimizer.core import (Action, Grid, Pose, advance, bounded_research,
                                    refine, replay, time_sample, wrap, joint_refine,
                                    sequence_cost, _collapse_for_joint,
                                    anchor_mode_sequences,
                                    collapse_exact_runs,
                                    equivalent_mode_sequences,
                                    fit_action_to_pose,
                                    build_corridors, corridor_box_corners, footprint_corners,
                                    select_corridor_display_stations,
                                    iterated_mode_geometry_refine,
                                    terminal_pose_graph_candidates,
                                    mode_schedule_neighbor_candidates)


class CoreTests(unittest.TestCase):
    def setUp(self):
        self.grid = Grid(np.zeros((100, 100), dtype=bool), .05, 0, 0)

    def test_center_kinematics(self):
        p = Pose(2, 2, 0)
        spin = advance(p, Action(2, math.pi/2, 0))
        self.assertEqual((spin.x, spin.y), (2, 2))
        self.assertAlmostEqual(spin.th, math.pi/2)
        oblique = advance(p, Action(1, 1, math.pi/2))
        self.assertAlmostEqual(oblique.x, 2)
        self.assertAlmostEqual(oblique.y, 3)
        k = 1/.81
        arc = advance(p, Action(0, .2, k))
        self.assertAlmostEqual(arc.x, 2+math.sin(.2*k)/k)

    def test_exact_run_compression_preserves_events_and_endpoint(self):
        start=Pose(2,2,.2)
        actions=[Action(0,.2,.3),Action(0,.2,.3),Action(0,.2,.4),
                 Action(0,-.1,.4),Action(2,.1,0),Action(2,.1,0)]
        collapsed=collapse_exact_runs(actions)
        self.assertEqual(len(collapsed),4)
        before=replay(start,actions)[-1]
        after=replay(start,collapsed)[-1]
        self.assertLess(math.hypot(before.x-after.x,before.y-after.y),1e-9)
        self.assertLess(abs(wrap(before.th-after.th)),1e-9)

    def test_swept_spin_collision(self):
        occ = np.zeros((100, 100), dtype=bool)
        occ[55, 55] = True
        grid = Grid(occ, .05, 0, 0)
        p = Pose(2.5, 2.5, 0)
        self.assertFalse(grid.action_clear(p, Action(2, math.pi/2, 0)))

    def test_corridors_follow_reference_heading_and_keep_seed_inside(self):
        heading=math.radians(37)
        pose=Pose(2.5,2.5,heading)
        feat,boxes,covered=build_corridors(self.grid,[pose])
        self.assertTrue(covered)
        self.assertEqual(len(boxes[0]),len(feat))
        c,s=math.cos(heading),math.sin(heading)
        for (ox,oy,_),(cx,cy,half_long,half_lat,box_heading) in zip(feat,boxes[0]):
            expected=(pose.x+ox*c-oy*s,pose.y+ox*s+oy*c)
            self.assertAlmostEqual(cx,expected[0])
            self.assertAlmostEqual(cy,expected[1])
            self.assertAlmostEqual(box_heading,heading)
            self.assertGreaterEqual(half_long,half_lat)
            corners=corridor_box_corners((cx,cy,half_long,half_lat,box_heading))
            for x,y in corners:
                self.assertGreaterEqual(x,0.0)
                self.assertLessEqual(x,5.0)
                self.assertGreaterEqual(y,0.0)
                self.assertLessEqual(y,5.0)
            # The first edge is the local longitudinal axis, rotated by ego yaw.
            dx=corners[1][0]-corners[0][0]
            dy=corners[1][1]-corners[0][1]
            if half_long>1e-8:
                self.assertAlmostEqual(math.atan2(dy,dx),heading)

    def test_corridor_records_identify_per_feature_fallback(self):
        occ = np.zeros((100, 100), bool)
        occ[:, 46] = True
        grid = Grid(occ, .05, 0, 0)
        records = []
        pose = Pose(2.7, 2.5, 0.)
        feat, boxes, covered = build_corridors(
            grid, [pose], records=records, plan_id=42, map_hash="test_map")
        self.assertFalse(covered)
        self.assertIn("disc_clearance_raster_guard", {r.certification for r in records})
        self.assertIn("point_free_cell_only", {r.certification for r in records})
        for index, record in enumerate(records):
            self.assertEqual(record.plan_id, 42)
            self.assertEqual(record.station, 0)
            self.assertEqual(record.feature_index, index)
            self.assertFalse(record.used_in_optimization)
            self.assertEqual(record.box, boxes[0][index])
            self.assertEqual(record.feature_offset, feat[index][:2])
            self.assertEqual(record.to_dict()["frame_id"], "map")

    def test_physical_footprint_size_heading_and_margin_across_pi(self):
        for yaw in (0., .6, math.pi-.01, math.pi+.01, -math.pi-.01):
            pose = Pose(2.5, 2.5, yaw)
            corners = footprint_corners(pose, .738, .500)
            expanded = footprint_corners(pose, .738, .500, .04)
            for polygon, length, width in ((corners,.738,.500), (expanded,.778,.540)):
                a,b,c,_ = polygon
                self.assertAlmostEqual(math.hypot(b[0]-a[0], b[1]-a[1]), length)
                self.assertAlmostEqual(math.hypot(c[0]-b[0], c[1]-b[1]), width)
                self.assertAlmostEqual(wrap(math.atan2(b[1]-a[1],b[0]-a[0])-yaw), 0.)
                self.assertAlmostEqual(sum(x for x,y in polygon)/4, pose.x)
                self.assertAlmostEqual(sum(y for x,y in polygon)/4, pose.y)

    def test_corridor_display_sampling_tracks_spacing_yaw_and_narrow_regions(self):
        poses=[];boxes=[]
        for i in range(41):
            x=.05*i
            width=.04 if .8-1e-9<=x<=1.2+1e-9 else .20
            poses.append(Pose(x,0.,0.))
            boxes.append([(x,0.,.25,width,0.)])
        selected=select_corridor_display_stations(poses,boxes)
        self.assertEqual(selected[0],0)
        self.assertEqual(selected[-1],len(poses)-1)
        narrow=[i for i in selected if .8-1e-9<=poses[i].x<=1.2+1e-9]
        self.assertGreaterEqual(len(narrow),3)
        self.assertLessEqual(max(poses[b].x-poses[a].x
                                 for a,b in zip(narrow,narrow[1:])),.16)
        for a,b in zip(selected,selected[1:]):
            self.assertLessEqual(poses[b].x-poses[a].x,.31)

        spin_poses=[Pose(1.,1.,math.radians(angle)) for angle in range(0,91,5)]
        spin_boxes=[[(1.,1.,.2,.12,p.th)] for p in spin_poses]
        spin_selected=select_corridor_display_stations(spin_poses,spin_boxes)
        self.assertEqual(spin_selected[0],0)
        self.assertEqual(spin_selected[-1],len(spin_poses)-1)
        for a,b in zip(spin_selected,spin_selected[1:]):
            self.assertLessEqual(abs(wrap(spin_poses[b].th-spin_poses[a].th)),
                                 math.radians(8)+1e-9)
        with self.assertRaisesRegex(ValueError,"station_mismatch"):
            select_corridor_display_stations(poses,boxes[:-1])

    def test_vectorized_pose_sat_matches_scalar_reference(self):
        rng=np.random.default_rng(24)
        occupied=rng.random((80,90))<.07
        grid=Grid(occupied,.05,0,0)
        half=grid.res/2
        def scalar_clear(p):
            c,s=math.cos(p.th),math.sin(p.th)
            rx=grid.half_l*abs(c)+grid.half_w*abs(s)
            ry=grid.half_l*abs(s)+grid.half_w*abs(c)
            if (p.x-rx<grid.ox or p.y-ry<grid.oy or
                p.x+rx>grid.ox+grid.width*grid.res or
                p.y+ry>grid.oy+grid.height*grid.res):
                return False
            x0=max(0,math.floor((p.x-rx-grid.ox)/grid.res))
            x1=min(grid.width-1,math.floor((p.x+rx-grid.ox)/grid.res))
            y0=max(0,math.floor((p.y-ry-grid.oy)/grid.res))
            y1=min(grid.height-1,math.floor((p.y+ry-grid.oy)/grid.res))
            for iy in range(y0,y1+1):
                for ix in np.flatnonzero(grid.occ[iy,x0:x1+1])+x0:
                    dx=grid.ox+(ix+.5)*grid.res-p.x
                    dy=grid.oy+(iy+.5)*grid.res-p.y
                    if abs(dx*c+dy*s)>grid.half_l+half*(abs(c)+abs(s)):continue
                    if abs(-dx*s+dy*c)>grid.half_w+half*(abs(s)+abs(c)):continue
                    if abs(dx)>half+rx or abs(dy)>half+ry:continue
                    return False
            return True
        for _ in range(400):
            p=Pose(float(rng.uniform(.5,4.0)),float(rng.uniform(.5,3.5)),
                   float(rng.uniform(-math.pi,math.pi)))
            self.assertEqual(grid.pose_clear(p),scalar_clear(p))

    def test_bounded_research_returns_sat_checked_aos_path(self):
        start=Pose(2,2,0)
        goal=Pose(2.7,2.4,math.radians(15))
        result=bounded_research(start,goal,[Action(0,.7,0)],self.grid,max_iter=100)
        self.assertIsNotNone(result)
        actions,iterations=result
        poses=replay(start,actions)
        self.assertGreaterEqual(iterations,1)
        self.assertTrue(self.grid.trajectory_clear(poses,actions))
        self.assertLessEqual(math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y),.16+1e-9)
        self.assertLessEqual(abs(wrap(poses[-1].th-goal.th)),.14+1e-9)

    def test_terminal_connector_compares_ackermann_with_oblique_for_reverse_entry(self):
        start=Pose(3.0,2.0,0.0)
        goal=Pose(2.4,2.0,0.0)
        result=bounded_research(start,goal,[Action(0,-.6,0.0)],self.grid,
                                max_iter=20)
        self.assertIsNotNone(result)
        actions,_=result
        self.assertEqual(actions[-1].mode,0)
        self.assertLess(actions[-1].amount,0.0)
        self.assertTrue(self.grid.trajectory_clear(replay(start,actions),actions))

    def test_terminal_pose_graph_discovers_spin_then_reverse_without_recipe(self):
        start=Pose(2.8,1.8,0.0)
        goal=Pose(2.8,2.8,-math.pi/2)
        seed=[Action(1,1.0,math.pi/2),Action(2,-math.pi/2,0.0)]
        candidates=terminal_pose_graph_candidates(
            start,goal,seed,self.grid,incumbent_score=100.0,
            tail_length=3.0,approach_offsets=(1.0,))
        self.assertTrue(candidates)
        score,actions,poses,_=candidates[0]
        self.assertLess(score,100.0)
        self.assertEqual([a.mode for a in actions],[2,0])
        self.assertLess(actions[-1].amount,0.0)
        spin_pose=poses[1]
        self.assertLess(spin_pose.y,goal.y)
        self.assertTrue(self.grid.trajectory_clear(poses,actions))

    def test_unwrapped_spin_output(self):
        timed=time_sample(Pose(2,2,math.pi-.02),[Action(2,.2,0)])
        self.assertTrue(all(mode==2 for _,_,_,mode in timed))
        angles=[p.th for _,p,_,_ in timed]
        self.assertGreater(angles[-1],math.pi)
        self.assertTrue(all(0 <= b-a < .1 for a,b in zip(angles,angles[1:])))
        self.assertTrue(all(v==0 for _,_,v,_ in timed))

    def test_time_sampling_preserves_internal_control_boundaries(self):
        start=Pose(2,2,0)
        actions=[Action(1,.7,0),Action(1,1.3,math.pi/2)]
        timed=time_sample(start,actions,v_demo=.5,dt=.5)
        self.assertTrue(all(mode==1 for _,_,_,mode in timed))
        corner=replay(start,actions)[1]
        self.assertTrue(any(math.hypot(p.x-corner.x,p.y-corner.y)<1e-9
                            for _,p,_,_ in timed))
        # No output chord should cut diagonally across the O-mode control
        # discontinuity: each neighboring pair lies on one primitive segment.
        for (_,a,_,_),(_,b,_,_) in zip(timed,timed[1:]):
            self.assertTrue(abs(a.x-b.x)<1e-9 or abs(a.y-b.y)<1e-9)

    def test_refinement_and_time(self):
        start = Pose(2, 2, 0)
        actions = [Action(0, .4, 0), Action(2, math.pi/2, 0),
                   Action(1, .4, 0)]
        goal = replay(start, actions)[-1]
        poses, refined, _, meta = refine(start, goal, actions, self.grid, max_iter=100)
        self.assertIn("nlp_solve_s",meta["profile"])
        self.assertTrue(self.grid.trajectory_clear(poses, refined))
        self.assertIn(meta["corridor"], ("disc", "point_fallback"))
        timed = time_sample(start, refined)
        self.assertTrue(all(b[0] > a[0] for a,b in zip(timed,timed[1:])))
        self.assertLess(math.hypot(timed[-1][1].x-goal.x,timed[-1][1].y-goal.y), .16)
        self.assertTrue(any(a[1].x==b[1].x and a[1].y==b[1].y and
                            abs(wrap(b[1].th-a[1].th))>1e-6 and a[2]==0 and b[2]==0
                            for a,b in zip(timed,timed[1:])))

    def test_refinement_accepts_sub_centimeter_reverse_primitive(self):
        start=Pose(2,2,0)
        actions=[Action(0,-.004,0),Action(0,.40,0)]
        goal=replay(start,actions)[-1]
        poses,refined,_,_=refine(start,goal,actions,self.grid,max_iter=100)
        self.assertTrue(self.grid.trajectory_clear(poses,refined))

    def test_joint_seed_collapse_respects_per_knot_limits(self):
        start=Pose(2,2,0)
        seed=[Action(0,1.0,.2),Action(2,math.radians(50),0)]
        collapsed,refs=_collapse_for_joint(start,seed)
        self.assertEqual(len(refs),len(collapsed)+1)
        self.assertTrue(all(abs(a.amount)<=2.0+1e-9 for a in collapsed if a.mode!=2))
        self.assertTrue(all(abs(a.amount)<=math.pi+1e-9
                            for a in collapsed if a.mode==2))
        end=replay(start,seed)[-1]
        self.assertLess(math.hypot(refs[-1].x-end.x,refs[-1].y-end.y),1e-5)
        self.assertLess(abs(wrap(refs[-1].th-end.th)),1e-5)

    def test_common_mode_cost_penalizes_unneeded_switches_and_spin(self):
        direct=[Action(1,.8,0)]
        spin_then_drive=[Action(2,math.pi/2,0),Action(0,.4,0),
                         Action(2,-math.pi/2,0)]
        self.assertLess(sequence_cost(direct)["score"],
                        sequence_cost(spin_then_drive)["score"])

    def test_mode_schedule_neighbors_offer_swept_safe_switch_reduction(self):
        start=Pose(1,1,0)
        seed=[Action(0,.5,0),Action(1,.5,0),Action(0,.5,0)]
        seed_end=replay(start,seed)[-1]
        neighbors=mode_schedule_neighbor_candidates(
            start,seed,self.grid,max_candidates=4,max_window=3,sweep_step=.005)
        reduced=next((item for item in neighbors
                      if sequence_cost(item[1])["switches"]<
                         sequence_cost(seed)["switches"]),None)
        self.assertIsNotNone(reduced)
        _,actions,poses,detail=reduced
        self.assertTrue(self.grid.trajectory_clear(poses,actions,.005))
        self.assertLess(math.hypot(poses[-1].x-seed_end.x,
                                   poses[-1].y-seed_end.y),.002)
        self.assertLess(abs(wrap(poses[-1].th-seed_end.th)),.002)
        self.assertEqual(detail["from_mode"],1)
        self.assertEqual(detail["to_mode"],0)

    def test_mode_schedule_switch_priority_can_keep_proxy_regression(self):
        start=Pose(1,1,0)
        seed=[Action(0,.5,0),Action(0,.5,0),Action(1,.5,-.5)]
        seed_score=sequence_cost(seed)["score"]
        neighbors=mode_schedule_neighbor_candidates(
            start,seed,self.grid,max_candidates=4,max_window=3,
            sweep_step=.005,require_proxy_improvement=False,
            prefer_switch_reduction=True)
        reduced=next((item for item in neighbors
                      if item[3]["mode_switch_count"]==0 and
                      item[3]["proxy_score"]>seed_score),None)
        self.assertIsNotNone(reduced)
        _,actions,poses,detail=reduced
        self.assertTrue(self.grid.trajectory_clear(poses,actions,.005))
        self.assertEqual(detail["mode_switch_count"],0)
        self.assertGreater(detail["proxy_score"],seed_score)

    def test_common_mode_cost_penalizes_sustained_oblique_travel(self):
        length=2.0;phi=math.pi/4
        straight=sequence_cost([Action(1,length,0.0)])["score"]
        diagonal=sequence_cost([Action(1,length,phi)])["score"]
        self.assertAlmostEqual(diagonal-straight,
                               length*(.55*abs(phi)+.45*(
                                   math.exp(-.5*(phi/math.radians(25))**2)-1)),
                               places=8)

    def test_ackermann_wins_oblique_mode_for_same_heading_aligned_cruise(self):
        ackermann=sequence_cost([Action(0,1.0,0.0)])["score"]
        oblique=sequence_cost([Action(1,1.0,0.0)])["score"]
        self.assertLess(ackermann,oblique)

    def test_common_cost_components_sum_to_reported_score(self):
        actions=[Action(0,.5,.2),Action(0,.4,-.3),Action(2,.6,0),
                 Action(1,-.7,math.pi/4),Action(1,.3,math.pi/3)]
        result=sequence_cost(actions)
        self.assertAlmostEqual(sum(result["cost_parts"].values()),
                               result["score"],places=10)
        self.assertGreater(result["cost_parts"]["spin"],0)
        self.assertGreater(result["cost_parts"]["mode_switch"],0)
        self.assertGreater(result["cost_parts"]["reverse"],0)
        self.assertGreater(result["cost_parts"]["steer_tv"],0)
        self.assertGreater(result["cost_parts"]["oblique_tv"],0)

    def test_mode_master_retains_ackermann_for_oblique_aligned_cruise(self):
        start=Pose(2,2,0)
        seed=[Action(0,.3,0),Action(1,.4,math.pi/2),Action(2,.2,0)]
        candidates=equivalent_mode_sequences(start,seed,max_candidates=8)
        target=replay(start,seed)[-1]
        lateral_first=next((seq for seq,_ in candidates
                            if [a.mode for a in seq]==[1,1,2]),None)
        self.assertIsNotNone(lateral_first)
        poses=replay(start,lateral_first)
        self.assertTrue(self.grid.trajectory_clear(poses,lateral_first))
        self.assertLess(math.hypot(poses[-1].x-target.x,poses[-1].y-target.y),1e-5)
        self.assertGreater(sequence_cost(lateral_first)["cost_parts"]["oblique_cruise"],
                           sequence_cost(seed)["cost_parts"]["oblique_cruise"])
        self.assertGreater(sequence_cost(lateral_first)["score"],
                           sequence_cost(seed)["score"])

    def test_anchor_mode_graph_can_move_switch_and_change_segment_geometry(self):
        start=Pose(2,2,0)
        seed=[Action(0,.3,0),Action(1,.4,math.pi/2),Action(2,.2,0)]
        goal=replay(start,seed)[-1]
        profile={}
        candidates=anchor_mode_sequences(start,seed,self.grid,max_candidates=8,
                                         profile=profile)
        self.assertGreaterEqual(profile["total_s"],0)
        merged=next((seq for seq,_ in candidates
                     if [a.mode for a in seq]==[1,2]),None)
        self.assertIsNotNone(merged)
        self.assertAlmostEqual(merged[0].amount,.5,places=5)
        self.assertAlmostEqual(merged[0].control,math.atan2(.4,.3),places=5)
        poses=replay(start,merged)
        self.assertTrue(self.grid.trajectory_clear(poses,merged))
        self.assertLess(math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y),1e-6)
        self.assertLess(sequence_cost(merged)["score"],
                        sequence_cost(seed)["score"])

    def test_iterated_mode_geometry_refine_is_safe_and_monotone(self):
        start=Pose(2,2,0)
        seed=[Action(0,.3,0),Action(1,.4,math.pi/2),Action(2,.2,0)]
        goal=replay(start,seed)[-1]
        profile={}
        poses,actions,_,meta=iterated_mode_geometry_refine(
            start,goal,seed,self.grid,incumbent_actions=seed,rounds=2,
            max_candidates=8,proposal_slack=.5,profile=profile)
        self.assertTrue(self.grid.trajectory_clear(poses,actions))
        self.assertLessEqual(sequence_cost(actions)["score"],
                             sequence_cost(seed)["score"]+1e-8)
        self.assertLess(math.hypot(poses[-1].x-goal.x,
                                   poses[-1].y-goal.y),.16)
        scores=[item["seed_score"] for item in meta["trace"]]
        scores.extend(item["result_score"] for item in meta["trace"])
        self.assertTrue(scores)
        self.assertEqual(meta["solver"],"iterated_mode_geometry")
        self.assertIn(meta["corridor"],("disc","point_fallback"))
        self.assertEqual(profile["joint_rounds_accepted"],
                         meta["rounds_accepted"])
        for previous,current in zip(meta["trace"],meta["trace"][1:]):
            self.assertLessEqual(current["seed_score"],
                                 previous["result_score"]+1e-8)

    def test_iterated_mode_geometry_stops_after_low_gain_but_keeps_safe_gain(self):
        start=Pose(2,2,0)
        seed=[Action(0,.3,0),Action(1,.4,math.pi/2),Action(2,.2,0)]
        goal=replay(start,seed)[-1]
        profile={}
        poses,actions,_,meta=iterated_mode_geometry_refine(
            start,goal,seed,self.grid,incumbent_actions=seed,rounds=2,
            max_candidates=8,proposal_slack=.5,
            min_round_improvement=1.0,profile=profile)
        self.assertTrue(self.grid.trajectory_clear(poses,actions))
        self.assertLess(sequence_cost(actions)["score"],
                        sequence_cost(seed)["score"])
        self.assertEqual(len(meta["trace"]),1)
        self.assertEqual(profile["joint_rounds_accepted"],1)
        self.assertEqual(profile["joint_early_stop_low_gain"],1.0)

    def test_mode_master_cost_reduces_switches_after_continuous_refinement(self):
        start=Pose(2,2,0)
        seed=[Action(0,.3,0),Action(1,.4,math.pi/2),Action(2,.2,0)]
        goal=replay(start,seed)[-1]
        alternatives=equivalent_mode_sequences(start,seed,max_candidates=4)
        schedule=next(seq for seq,_ in alternatives
                      if [a.mode for a in seq]==[1,1,2])
        _,base,_,_=refine(start,goal,seed,self.grid)
        poses,optimized,_,_=refine(start,goal,schedule,self.grid)
        self.assertTrue(self.grid.trajectory_clear(poses,optimized))
        self.assertLess(sequence_cost(optimized)["score"],
                        sequence_cost(base)["score"])
        self.assertEqual(sequence_cost(optimized)["switches"],1)

    def test_mode_fit_preserves_reverse_ackermann_arc(self):
        start=Pose(3,3,.4)
        original=Action(0,-.5,-.4)
        target=advance(start,original)
        fitted=fit_action_to_pose(0,start,target)
        self.assertIsNotNone(fitted)
        self.assertLess(fitted.amount,0)
        self.assertAlmostEqual(fitted.amount,original.amount,places=5)
        self.assertAlmostEqual(fitted.control,original.control,places=5)

    def test_joint_solver_can_select_oblique_for_lateral_goal(self):
        try:
            import casadi as ca
            if not ca.has_nlpsol("bonmin"):
                self.skipTest("CasADi Bonmin plugin unavailable")
        except Exception as exc:
            self.skipTest("CasADi/Bonmin unavailable: %s" % exc)
        start=Pose(2,2,0);goal=Pose(2,2.2,0)
        # Seed is deliberately Ackermann; one knot allows a direct mode swap.
        poses,actions,_,meta=joint_refine(
            start,goal,[Action(0,.2,0)],self.grid,max_knots=2,time_limit=1.0)
        self.assertTrue(any(a.mode==1 for a in actions))
        self.assertTrue(self.grid.trajectory_clear(poses,actions))
        self.assertLess(math.hypot(poses[-1].x-goal.x,poses[-1].y-goal.y),.02)
        self.assertIn(meta["solver"], ("bonmin_minlp",
                                        "joint_single_knot_exact_enumeration"))

    def test_joint_solver_handles_seed_yaw_crossing_pi(self):
        try:
            import casadi as ca
            if not ca.has_nlpsol("bonmin"):
                self.skipTest("CasADi Bonmin plugin unavailable")
        except Exception as exc:
            self.skipTest("CasADi/Bonmin unavailable: %s" % exc)
        start=Pose(3,3,3.13)
        goal=Pose(3,3,wrap(3.23))
        poses,actions,_,meta=joint_refine(
            start,goal,[Action(2,.10,0)],self.grid,max_knots=2,time_limit=1.0)
        self.assertIn(meta["solver"], ("bonmin_minlp",
                                        "joint_single_knot_exact_enumeration"))
        self.assertTrue(self.grid.trajectory_clear(poses,actions))
        self.assertLess(abs(wrap(poses[-1].th-goal.th)),.14)

    def test_joint_solver_scores_reverse_distance_consistently(self):
        try:
            import casadi as ca
            if not ca.has_nlpsol("bonmin"):
                self.skipTest("CasADi Bonmin plugin unavailable")
        except Exception as exc:
            self.skipTest("CasADi/Bonmin unavailable: %s" % exc)
        start=Pose(2,2,0);goal=Pose(1.8,2,0)
        poses,actions,_,meta=joint_refine(
            start,goal,[Action(0,-.2,0)],self.grid,max_knots=2,time_limit=1.0)
        score=sequence_cost(actions)
        self.assertTrue(self.grid.trajectory_clear(poses,actions))
        self.assertGreater(score["reverse_length"],.19)
        self.assertAlmostEqual(meta["score"],score["score"],places=7)


if __name__ == "__main__":
    unittest.main()
