"""Quantify display rigidity error of frame_at() during a cargo rotation.

Builds a synthetic but contract-correct rigid rotation (new wedge offsets,
radius 1 m) and measures the centre error introduced by the actual display
interpolation between published samples.  No ROS, no planner involved.
"""
import json, math, sys
sys.path.insert(0, 'catkin_ws/src/fwis_v3_optimizer/src')
from fwis_v3_optimizer.formation_playback import (
    FormationPlaybackTrajectory, PlaybackPoint, PlaybackVehicleSpec, frame_at)

OFFSETS = ((1, 1.0, 0.0), (2, 0.0, 1.0), (3, 0.0, -1.0))
DT = 0.02                      # publication_dt used by formation_demo.launch
results = []
for omega in (0.2, 0.5, 1.0):
    n = 400
    times, poses, points = [], [], {vid: [] for vid, _, _ in OFFSETS}
    for k in range(n + 1):
        t = k * DT
        psi = omega * t
        times.append(t)
        poses.append((5.0, 5.0, psi))
        for vid, rx, ry in OFFSETS:
            x = 5.0 + math.cos(psi) * rx - math.sin(psi) * ry
            y = 5.0 + math.sin(psi) * rx + math.cos(psi) * ry
            points[vid].append(PlaybackPoint(
                time_from_start=t, center_x=x, center_y=y,
                theta=math.pi / 2 + psi, center_speed_mps=omega, mode=0,
                mode_control=1.0))
    traj = FormationPlaybackTrajectory(
        plan_id=1, formation_id="probe", frame_id="map", stamp=(1, 0),
        vehicles=tuple(PlaybackVehicleSpec(
            vehicle_id=vid, offset_x=rx, offset_y=ry,
            body_length_m=0.738, body_width_m=0.5) for vid, rx, ry in OFFSETS),
        formation_poses=tuple(poses), times_from_start=tuple(times),
        points={k: tuple(v) for k, v in points.items()})
    worst = 0.0
    node = 0.0
    for k in range(n):
        frame = frame_at(traj, times[k] + DT / 2.0)
        for st in frame.vehicles:
            vid = st.vehicle_id
            rx, ry = [(r[1], r[2]) for r in OFFSETS if r[0] == vid][0]
            ex = frame.formation_x + math.cos(frame.formation_psi) * rx - math.sin(frame.formation_psi) * ry
            ey = frame.formation_y + math.sin(frame.formation_psi) * rx + math.cos(frame.formation_psi) * ry
            worst = max(worst, math.hypot(st.center_x - ex, st.center_y - ey))
    analytic = 1.0 - math.cos(omega * DT / 2.0)
    results.append({"omega_radps": omega, "dt_s": DT,
                    "delta_psi_per_sample_rad": omega * DT,
                    "analytic_midpoint_radial_error_m": analytic,
                    "measured_max_display_rigidity_error_m": worst,
                    "exceeds_1e-6_contract_suggestion": worst > 1e-6})
print(json.dumps(results, indent=1))
