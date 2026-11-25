"""Audit the actual mode transitions and final Twist of one completed Gazebo experiment."""
import argparse
import gzip
from hashlib import sha256
import json
import math
from pathlib import Path

import yaml
from cases import ROOT


def check_run(run, timing):
    from fwis_v3_msgs.msg import FormationPlaybackPlan
    from rclpy.serialization import deserialize_message
    from ranger_mini_gz.body_sim import classify_twist

    run, timing = Path(run), Path(timing)
    report = yaml.safe_load((run/"result.yaml").read_text())
    compressed = run/"trajectories/planner.cdr.gz"
    if (run/"trajectories/model.json.gz").is_file():
        raw = gzip.decompress((run/"trajectories/model.json.gz").read_bytes())
    elif report["task"]=="multi" and compressed.is_file():
        msg = deserialize_message(gzip.decompress(compressed.read_bytes()), FormationPlaybackPlan)
        raw = msg.model_json.encode()
    else:
        return dict(passed=False,issues=["archived_single_contains_only_sampled_projection"],run_id=run.name)
    model = json.loads(raw)
    model_hash = sha256(raw).hexdigest()
    epoch = model["epoch_ns"]*1e-9
    limits = yaml.safe_load((run/"controller_parameters.yaml").read_text())
    plant = yaml.safe_load((ROOT/"ros2_ws/src/ranger_mini_gz/config/body_twist.yaml").read_text())["plant"]
    stops = limits["timing"]
    phases = model["phases"]
    ends = []
    for phase in phases:
        ends.append((ends[-1] if ends else 0.) + phase["duration"])
    cars = sorted(report["routed_plan_keys"])
    motion = [json.loads(line) for line in gzip.open(run/"metrics/motion.jsonl.gz", "rt")]
    issues = []
    observed = {}
    zero_motion_request = {"WAIT_START","REQUEST_MODE_SWITCH","WAIT_MODE_CONFIRMATION","SETTLE","COMPLETE","STOPPED"}
    for car in cars:
        history = [json.loads(line) for line in gzip.open(timing/f"robot{car}_controller.jsonl.gz", "rt")]
        rows = [r for r in history if r.get("model_sha256")==model_hash and r["trajectory_stamp_ns"]==model["epoch_ns"]]
        if not rows:
            issues.append(f"robot{car}:missing_model_status")
            continue
        mode = rows[0]["feedback_mode"]
        records = []
        for i, phase in enumerate(phases):
            start, end = (ends[i-1] if i else 0.), ends[i]
            def in_phase(stamp):
                return start-1e-8 <= stamp-epoch < end-1e-8
            planned = phase["mode"]
            ctrls = [r for r in rows if in_phase(r["stamp"])]
            plant_rows = [r for r in motion if in_phase(r["stamp_s"]) and str(car) in r.get("odom_twist",{})]
            if not ctrls or not plant_rows:
                issues.append(f"robot{car}:phase{i}_missing_ticks")
            requests = [r for r in ctrls if r.get("requested_mode") is not None]
            if any(r["requested_mode"]!=planned or any(r["command"]) for r in requests):
                issues.append(f"robot{car}:phase{i}_moving_or_misdirected_request")
            if any(any(r["command"]) for r in ctrls if r["phase"] in zero_motion_request):
                issues.append(f"robot{car}:phase{i}_motion_during_transition")
            data = dict(mode=planned, kind=phase["kind"], duration_s=end-start, controller_ticks=len(ctrls),
                        odom_samples=len(plant_rows), requests=len(requests))
            if phase["kind"] != 0:
                if mode != planned and not requests:
                    issues.append(f"robot{car}:phase{i}_missing_mode_request")
                last = [r for r in ctrls if r["stamp"]-epoch >= end-stops["settle_time"]-1e-8]
                if not last or any(any(r["command"]) or r["feedback_mode"]!=planned for r in last):
                    issues.append(f"robot{car}:phase{i}_not_stopped_and_confirmed_before_motion")
            else:
                moving = [r for r in ctrls if any(r["command"])]
                if any(r["feedback_mode"]!=planned or
                       classify_twist(r["command"], plant["switch_radius"])[0]!=planned for r in moving):
                    issues.append(f"robot{car}:phase{i}_wrong_encoded_or_unconfirmed_mode")
                data["moving_command_ticks"] = len(moving)
                key=str(car)
                moving_odom = [r for r in plant_rows if math.hypot(*r["odom_twist"][key][:2])>stops["stop_speed"] or
                               abs(r["odom_twist"][key][2])>stops["stop_yaw_rate"]]
                if any(r["mode_feedback"][key]!=planned for r in moving_odom):
                    issues.append(f"robot{car}:phase{i}_wrong_observed_mode")
                data["moving_odom_samples"] = len(moving_odom)
                if planned == 2 and plant_rows:
                    turn = phase["heading1"][car-1]-phase["heading0"][car-1]
                    measured = sum(math.remainder(right["poses"][key][2]-left["poses"][key][2],2*math.pi)
                                   for left,right in zip(plant_rows,plant_rows[1:]))
                    data.update(planned_spin_rad=turn,measured_spin_rad=measured)
                    if abs(turn)>.01 and not (turn*measured>0 and moving_odom):
                        issues.append(f"robot{car}:phase{i}_spin_not_measured_in_planned_direction")
                if planned == 1 and plant_rows:
                    begin, finish = phase["pieces"][0], phase["pieces"][-1]
                    vector = [finish[k][-1]-begin[k][0] for k in ("x","y")]
                    disp = [plant_rows[-1]["poses"][key][j]-plant_rows[0]["poses"][key][j] for j in (0,1)]
                    data.update(planned_displacement_m=vector,measured_displacement_m=disp)
                    if math.hypot(*vector)>.02 and not (sum(v*d for v,d in zip(vector,disp))>0 and moving_odom):
                        issues.append(f"robot{car}:phase{i}_oblique_not_measured_in_planned_direction")
            records.append(data)
            mode=planned
        final_cmd=report["final_cmd_vel"][car]
        final_odom=report["last_odom_twist"][car]
        if any(final_cmd) or math.hypot(*final_odom[:2])>stops["stop_speed"] or abs(final_odom[2])>stops["stop_yaw_rate"]:
            issues.append(f"robot{car}:not_stopped_at_end")
        observed[car]=dict(phases=records,final_command=final_cmd,final_odom_twist=final_odom)
    if (report["status"]!="passed" or report["admitted"]!=cars or report["complete"]!=cars or
            any(key[1]!=model["epoch_ns"] for key in report["routed_plan_keys"].values())):
        issues.append("missing_common_epoch_completion")
    return dict(passed=not issues, issues=issues, run_id=run.name, epoch_ns=model["epoch_ns"],
                model_sha256=model_hash, vehicles=observed)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run",type=Path)
    parser.add_argument("timing",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    result=check_run(args.run,args.timing)
    args.output.write_text(json.dumps(result,indent=2)+"\n")
    print(args.run.name, result["passed"],result["issues"])
    return 0 if result["passed"] else 1


if __name__=="__main__":
    raise SystemExit(main())
