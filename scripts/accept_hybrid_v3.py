#!/usr/bin/env python3
"""Evaluate the frozen V3 gates. Missing cases, missing raw metrics and degraded plans cannot pass."""
import argparse
import collections
import csv
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parent/"hybrid_v3"))
from qualify import (LOCAL,NUMERIC,POLICY,PROFILES,REGISTRY,REPORT,ROOT,SPEC,TASKS,closed_rows,
                     archived_comparisons,collect,digest,lines,load,quant,setup_metrics,sha,xml_cases)
from cases import indices
from evidence_manifest import build_manifest, audit_rows, system_hashes, runtime_binaries, recorder_revision, RANKING_SOURCE


def write(path,obj):
    path.write_text(json.dumps(obj,ensure_ascii=False,indent=2,allow_nan=False)+"\n")


def main():
    release="c6"
    cohort="c3"
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--batch",type=Path,default=LOCAL/f"release_{release}")
    ap.add_argument("--control",type=Path,default=LOCAL/f"control_605_{release}")
    ap.add_argument("--gazebo",type=Path,default=LOCAL/f"gazebo_release_{release}")
    ap.add_argument("--headless",type=Path,default=LOCAL/f"gazebo_headless_{release}")
    ap.add_argument("--legacy",type=Path,default=NUMERIC/f"legacy_{cohort}",help="paired previous-backend results for this cohort")
    ap.add_argument("--numeric",type=Path,default=NUMERIC/release)
    ap.add_argument("--ros",type=Path,default=LOCAL/f"group_{release}")
    ap.add_argument("--witness",type=Path,default=LOCAL/f"witness_final_{release}")
    ap.add_argument("--system",type=Path,default=LOCAL/f"system_{release}",help="qualified ROS snapshot")
    ap.add_argument("--native",type=Path,default=LOCAL/f"native_{release}")
    ap.add_argument("--binary",type=Path,default=LOCAL/f"binary_{release}")
    ap.add_argument("--ctest",type=Path,default=LOCAL/f"release_{release}"/"ctest")
    ap.add_argument("--recorder",type=Path,default=LOCAL/f"recorder_stop_{release}"/"receipt.json")
    ap.add_argument("--report",type=Path,default=REPORT)
    ap.add_argument("--compact",action="store_true",help="re-evaluate from the checked-in measurements")
    args=ap.parse_args();out=args.report;out.mkdir(exist_ok=True,parents=True)
    spec=yaml.safe_load(SPEC.read_text())
    data=load(out/"measurements.json") if args.compact else None
    if not args.compact:
        if not any(args.batch.glob("*.jsonl")):
            raise SystemExit(f"No raw planning runs in {args.batch}. Use --compact to verify the published measurements.")
        casefile=REPORT/"case_manifest.json"
        stats=collect(args.batch)
        models=load(args.numeric/"model.json");collision=load(args.numeric/"collision.json")
        derivatives=load(args.numeric/"derivatives.json")
        defaults=yaml.safe_load(REGISTRY.read_text())
        native=load(args.native/"runtime.json")
        archive=args.ros
        fixtures={t:load(archive/f"{t}.json") for t in TASKS}
        legacy={f"{t}/{p}":[[x["case_id"],x.get("request_hash"),x["evidence"].get("certified_in_budget",False)]
                for x in lines(args.legacy/f"legacy_{t}_{p}.jsonl")] for t in TASKS for p in PROFILES}
        witness={}
        for split in ("core","holdout","development","pruning"):
            for profile in PROFILES:
                for r in lines(args.witness/f"witness_{split}_{profile}.jsonl"):
                    witness[r["case_id"]+"/"+profile]=[r["request_hash"],r["evidence"].get("executable",False),r.get("witness_hash")]
        ctest=xml_cases(args.ctest);san=xml_cases(args.numeric/"ASan_UBSan")
        data=dict(model=models,collision=collision,derivatives=derivatives,software_defaults=defaults["defaults"],
                  native=native,group_fixtures=fixtures,legacy=legacy,legacy_summary=load(args.legacy/"legacy_progress.json"),control=setup_metrics(args.control),
                  certificate_tests=xml_cases(args.numeric),ctest=ctest,san=san,necessary_aux=load(args.numeric/"necessary_aux.json"),
                  necessary_profiles=load(args.numeric/"necessary_corridor.json"),
                  asan_smoke=load(args.numeric/"sanitizer_summary.json"),native_faults=load(archive/"faults.json"),
                  admission=load(archive/"delivery.json"),cases=stats,witness=witness,gazebo=closed_rows(args.gazebo),headless=closed_rows(args.headless),
                  inputs_sha256={str(p.relative_to(ROOT)):sha(p) for p in [SPEC,REGISTRY,ROOT/RANKING_SOURCE,casefile,REPORT/"case_manifest_baseline.json"] if p.exists()},
                  default_runs=load(args.batch/"default_runs.json"),source_manifest=build_manifest(args.batch,args.witness,args.binary,args.ctest))
        data["paired_closed_loop"]=archived_comparisons(data["gazebo"],data["default_runs"])
        system_identity=args.system/"source_commit"
        system_commit=system_identity.read_text().strip() if system_identity.is_file() else None
        if data.get("default_runs"):
            from default_entries import collect as read_entry
            verified=[]
            for receipt in data["default_runs"]:
                if receipt["entry"] not in ("single","multi","interactive"):
                    raise ValueError("Unknown default entry")
                archive=ROOT/"reports/runs"/receipt["run_id"]
                checked=read_entry(receipt["entry"],archive)
                if checked!=receipt:
                    raise ValueError("Public entry receipt differs from its observed route")
                verified.append(checked)
            data["default_runs"]=verified
        data["source_manifest"]["system_commit"]=system_commit
        data["source_manifest"]["system_source_sha256"]=system_hashes(system_commit)
        data["source_manifest"]["system_release_tests"]=xml_cases(args.system/"ctest")
        data["source_manifest"]["runtime_binaries"]=runtime_binaries(args.system)
        data["source_manifest"]["recorder_revision"]=recorder_revision(args.recorder)
        data["source_manifest"]["system_snapshot"]=str(args.system.resolve())
        write(out/"measurements.json",data)
    if data is None:raise SystemExit("missing measurements")
    if not args.compact:
        from publish_evidence import export
        export(out,data,args.batch)
    cfg={g["id"]:g for g in spec["checks"]}
    checks={}
    def record(name,available,passed,measurement):
        checks[name]=dict(status=("PASS" if passed else "FAIL") if available else "NOT_RUN",
                          measured=measurement,required={k:v for k,v in cfg[name].items() if k not in ("id","mandatory")})
    m=data["model"] or {};d=data["derivatives"] or {};c=data["collision"] or {}
    record("GEOMETRY_KINEMATICS",bool(m),m.get("ordinary_per_task_per_mode",0)>=10000 and
           m.get("boundary_cases",0)>0 and m.get("max_position_error_m",1)<=1e-7 and m.get("max_angle_error_rad",1)<=1e-7,m)
    record("REINTEGRATION",bool(m),m.get("reintegrated_trajectories",0)>0 and
           max(m.get("max_reintegration_position_m",1),m.get("max_reintegration_angle_rad",1))<=1e-4,
           {k:v for k,v in m.items() if "reintegrat" in k})
    record("DERIVATIVES",bool(d),d.get("cases_per_row",0)>=100 and d.get("assembled_rows",0)>0 and
           len(d.get("step_sizes",[]))>=2 and d.get("failures",1)==d.get("nonfinite",1)==0 and
           d.get("max_relative_error",1)<=1e-4 and d.get("max_near_zero_absolute_error",1)<=1e-6,d)
    native=data["native"] or {}
    record("NATIVE_RUNTIME",bool(native),native.get("pybind_disabled") and native.get("forbidden_loads")==[]
           and len(native.get("traced_execs",[]))==1 and native.get("certified")==[True,True]
           and native.get("backend")==["single_hybrid_joint_cpp","formation_hybrid_joint_cpp"]
           and all(any(r.get("fault")=="success" and r.get("phase")=="COMPLETE" for r in (data["group_fixtures"].get(t) or [])) for t in TASKS),native)
    tests=data["certificate_tests"]
    record("COLLISION_DYNAMICS",bool(c) and bool(tests),c.get("random_collision_cases",0)>=20000 and
           c.get("false_negatives",1)==0 and all(tests.get(k) for k in (
           "RejectsHiddenCollisionAndCargoAndPairContact","RejectsInteriorSpinCornerContact",
           "EnforcesReversePolicyAndPreparationTargetAndWheelGeometry","GearAndSwitchRequireStoppedSteeringPreparation")),
           dict(collision=c,native_certificate_tests=tests))
    code_tests=data["ctest"];asan=data["asan_smoke"] or {}
    record("BUILD_TESTS_SANITIZERS",bool(code_tests),bool(code_tests) and all(code_tests.values()) and
           all(data["san"].values()) and bool(data["san"]) and bool((data["source_manifest"] or {}).get("system_release_tests")) and
           all(data["source_manifest"]["system_release_tests"].values()) and asan.get("native_solve_passed") is True
           and asan.get("sanitizers")==["address","undefined"] and bool(asan.get("tsan")),dict(ctest=code_tests,sanitizers=asan))
    rows=data["cases"];witness=data["witness"]
    def valid(r):
        return r["valid"] and r["has_plan"] and r["status"]=="CERTIFIED_POLICY_COMPLIANT" and r["actual_backend"]==r["task"]+"_hybrid_joint_cpp"
    def subset(task,profile,split,op):
        return [r for r in rows if r["task"]==task and r["profile"]==profile and r["dataset_split"]==split and r["operation"]==op]
    def full_set(group,split):
        return len(group)==len(indices(split)) and {r["index"] for r in group}==set(range(len(indices(split))))
    feasible=[r for r in rows if r["operation"]=="solve" and r["has_plan"] and r["valid"]]
    endpoints={k:max([r["metrics"].get(k,1e6) for r in feasible],default=None) for k in ("terminal_position_m","constrained_terminal_yaw_rad")}
    record("PLAN_ENDPOINTS",bool(feasible),bool(feasible) and endpoints["terminal_position_m"]<=.02 and
           endpoints["constrained_terminal_yaw_rad"]<=math.pi/180 and tests.get("RequestEndpointsFreeYawAndCancellation"),endpoints)
    joins={k:max([r["metrics"].get(k,1e6) for r in feasible],default=None) for k in ("join_position_m","join_yaw_rad","join_speed_mps","curvature_jump")}
    record("A_CONTINUITY",bool(feasible),bool(feasible) and max(joins[k] for k in joins if k!="curvature_jump")<=1e-6
           and joins["curvature_jump"]<=1e-5 and tests.get("ContinuousCurvatureAndIndependentFormationYaw"),joins)
    record("FORMATION_REFERENCE",bool(m),m.get("max_mount_error_m",1)<=1e-8 and tests.get("ContinuousCurvatureAndIndependentFormationYaw")
           and tests.get("EnforcesReversePolicyAndPreparationTargetAndWheelGeometry"),dict(max_mount_identity_error_m=m.get("max_mount_error_m")))
    necessary=data.get("necessary_aux") or {}
    guaranteed={r["task"]:r for r in necessary.get("cases",[])}
    extra_bounds={(r["task"],r["profile"]):r for r in (data.get("necessary_profiles") or {}).get("cases",[])}
    def proven_aux(task,profile):
        other=extra_bounds.get((task,profile),{})
        bounds=other.get("bound",{})
        if task=="single":
            physical=(bounds.get("projected_width_m",0)>bounds.get("width_m",1) and
                      bounds.get("max_A_lateral_m",1e6)<bounds.get("required_m",0)-.04)
        else:
            physical=bounds.get("smallest_singular_value",0)>bounds.get("endpoint_perturbation_bound",1)
        outcome=other.get("solve",{})
        if not (physical and other.get("witness_audit",{}).get("executable") and outcome.get("audit",{}).get("executable") and
                outcome.get("metrics",{}).get("aux_blocks",0)>0):
            return False
        r=guaranteed.get(task,{})
        bound=r.get("bound",{})
        proof=(bound.get("minimum_pure_A_duration_s",0)>bound.get("cap_s",1) if task=="single" else
               bound.get("certified_rank")==3 and bound.get("smallest_singular_value",0)>bound.get("endpoint_perturbation_Frobenius_max",1))
        plan=r.get("solve",{})
        return bool(proof and r.get("request_sha") and r.get("witness_sha") and
                    r.get("witness_audit",{}).get("executable") and plan.get("audit",{}).get("executable") and
                    plan.get("actual_backend")==task+"_hybrid_joint_cpp" and plan.get("metrics",{}).get("aux_blocks",0)>0)
    sets={};all_core=True;all_hold=True;all_prune=True;latencies={};timed_ok=True
    have_core=have_hold=have_prune=have_latency=False
    def is_witness(r):
        w=witness.get(r["case_id"]+"/"+r["profile"],[])
        return len(w)==3 and w[0]==r["request_hash"] and w[1] and bool(w[2])
    def pure(r):
        return valid(r) and r["metrics"].get("mode_sequence")=="A" and r["metrics"].get("internal_stops")==0
    for t in TASKS:
        for p in PROFILES:
            key=t+"/"+p;sets[key]={}
            for split in ("core","development","holdout"):
                rr=subset(t,p,split,"solve");a=[r for r in rr if r["label"]=="A"];f=[r for r in rr if r["label"]!="invalid"]
                paired=data["legacy"].get(key,[])
                rot=[sum(valid(r) for r in f if r["rotated"]==v)/max(1,sum(r["rotated"]==v for r in f)) for v in [False,True]]
                mm=dict(attempted=len(rr),expected=len(indices(split)),A=len(a),pure_A=sum(pure(r) for r in a),
                    feasible=len(f),success=sum(valid(r) for r in f),bad_invalid=sum(r["has_plan"] for r in rr if r["label"]=="invalid"),
                    certified_witnesses=sum(is_witness(r) for r in f),rotation_rate_drop_pp=100*(rot[0]-rot[1]))
                sets[key][split]=mm
                ready=full_set(rr,split) and len(f)>0 and all(is_witness(r) for r in f) and not mm["bad_invalid"]
                if split=="core":
                    have_core|=bool(rr);all_core &= ready and mm["success"]==len(f) and mm["pure_A"]==len(a)
                if split=="holdout":
                    by_case={r["case_id"]:r["request_hash"] for r in rr}
                    matched={i for i,h,_ in paired if by_case.get(i)==h}
                    mm["paired_legacy"]=len(matched);mm["paired_legacy_success"]=sum(bool(ok) for i,h,ok in paired if i in matched)
                    have_hold|=bool(rr);all_hold &= (ready and len(a)>=60 and len(f)-len(a)>=40 and len(matched)==len(rr)
                        and mm["success"]>=.95*len(f) and mm["pure_A"]>=.95*len(a) and mm["rotation_rate_drop_pp"]<=5.00000001)
            rr=subset(t,p,"pruning","ablation");have_prune|=bool(rr)
            edit=[r for r in rr if r["ablation"]=="full"]
            redundant=[r for r in edit if r["label"]=="A"]
            counter=[r for r in edit if r["label"]=="other"]
            # Require an actual structural edit in the trace: simply solving an unrelated pure-A seed is insufficient.
            removed=sum(pure(r) and any(("REPLACE_SUBSEQUENCE" in e or "REMOVE_BLOCK" in e) and "CERTIFIED" in e for e in r["trace"]) for r in redundant)
            ab={k:sum(valid(r) for r in rr if r["ablation"]==k) for k in ("fixed","movable","full")}
            sets[key]["pruning"]=dict(pairs=len(redundant),removed=removed,ablations=ab,other_fixtures=len(counter),proven_necessary_aux=proven_aux(t,p))
            all_prune &= len(redundant)>=10 and removed>=.9*len(redundant) and len(counter)>=1 and all(
                valid(r) for r in counter) and proven_aux(t,p) and all(full_set([r for r in rr if r["ablation"]==k],"pruning") for k in ab)
            for difficulty,family in (("easy","C01"),("difficult","C06")):
                rr=[r for r in rows if r["operation"]=="timing" and r["task"]==t and r["profile"]==p and r["family"]==family]
                have_latency|=bool(rr)
                trials=[r for r in rr if r["repeat"]>=0]; vals=[r["wall_s"] for r in trials if valid(r)]
                ceiling=cfg["PLANNING_LATENCY"][difficulty+"_"+t+"_p95_max"]
                latency=dict(trials=len(trials),success=len(vals),warmups=sum(r["repeat"]<0 for r in rr),
                             p50=quant(vals,.5),p95=quant(vals,.95),maximum=max(vals,default=None))
                latencies[key+"/"+difficulty]=latency
                timed_ok &= (len(rr)==35 and {r["repeat"] for r in rr}==set(range(-5,30)) and len(vals)==30 and latency["p95"]<=ceiling)
    record("A_WITNESS_CORE",have_core,all_core,{k:v["core"] for k,v in sets.items()})
    record("MODE_PRUNING",have_prune,all_prune,{k:v["pruning"] for k,v in sets.items()})
    record("HOLDOUT_GENERALIZATION",have_hold,all_hold and all(v["development"]["attempted"]>=60 for v in sets.values()),
           {k:v["holdout"] for k,v in sets.items()})
    record("PLANNING_LATENCY",have_latency,timed_ok,latencies)
    control=data["control"];four=set(control)=={"1/1","3/1","3/2","3/3"}
    count_ok=four and all(r["wall_s"]>=600 and r["valid"]==r["ticks"] and r["valid"]>=15000 and r["consecutive"]
                          and r["completed_planner_requests"]>0 and r["certified_planner_requests"]>0 for r in control.values())
    record("CONTROLLER_COMPUTE",bool(control),count_ok and all(r["p99_compute_ms"]<=.5 and r["max_compute_ms"]<=1.
           and r["max_callback_ms"]<=5 for r in control.values()),control)
    record("NATIVE_CONTROL_DEADLINES",bool(control),count_ok and all(r["deadline_misses"]==0 for r in control.values()),control)
    fixture_ok=all({r["fault"]:r["phase"] for r in (data["group_fixtures"].get(t) or [])} .items() >=
                   {"success":"COMPLETE","tamper":"STOPPED","missing_member":"STOPPED","stale_feedback":"STOPPED"}.items() for t in TASKS)
    extras=data["native_faults"] or {}
    record("CONTROLLER_PARITY_FAULTS",bool(code_tests and fixture_ok),fixture_ok and code_tests.get("tracking_cpp_parity") and
           extras.get("invalid_numeric_commands")==0 and all(extras.get(k) for k in ("old_epoch_rejected","clock_rollback_stopped","late_permit_stopped")),
           dict(groups=data["group_fixtures"],extra=extras))
    gaz=data["gazebo"];summary={};enough=True
    for t in TASKS:
        for p in PROFILES:
            subset_g=[r for r in gaz if r["task"]==t and r["profile"]==p]
            results=collections.defaultdict(list)
            for r in subset_g:
                count=1 if t=="single" else 3
                ids=list(range(1,count+1))
                met=r["metrics"] or {};vs=met.get("vehicles",{})
                fault=r["fault"] or {}
                stages=(r.get("publication_event") or {}).get("planning_stages",{})
                policy_ok=stages.get("policy_status")=="CERTIFIED_POLICY_COMPLIANT"
                if r["family"]=="fault":
                    good=(r["status"]=="stopped" and r["reason"]=="external_stop" and fault.get("triggered") and fault.get("moved")==ids and
                          len(fault.get("first_zero_after_s",{}))==count and not any(fault.get("last",{}).values()))
                else:
                    keys=set(tuple(k) for k in (r["routed_plan_keys"] or {}).values())
                    authentic=any(a.get("model_sha256")==r["model_sha256"] and (a.get("plan_id"),a.get("start_ns")) in keys
                                  for a in r["admissions"])
                    group_geometry=(t=="single" or
                                    met.get("min_vehicle_footprint_gap_m",-1)>0 and met.get("max_mount_error_m",1)<=.10 and
                                    met.get("terminal_formation_position_error_m",1)<=.10 and met.get("terminal_formation_yaw_error_deg",180)<=math.degrees(.10))
                    good=(r["status"]=="passed" and r["tracking_implementation"]=="cpp" and r["planner_source"]=="hybrid_joint" and authentic and r["plan_received"] and r["admitted"]==r["complete"]==ids and
                          bool(r["admissions"]) and len(keys)==1 and
                          len(vs)==count and all(v["terminal_position_error_m"]<=.10 and v["terminal_yaw_error_deg"]<=math.degrees(.1) for v in vs.values()) and
                          group_geometry and bool(met.get("min_environment_clearance")) and
                          all(v["m"]>0 for v in met["min_environment_clearance"].values()))
                if r["family"] in ("multibend","necessary_aux"):
                    good=good and r.get("scene_measurements",{}).get("passed") and bool(r.get("scene_proof"))
                results[r["family"]].append((r["trial"],bool(good and policy_ok and r["gui"])))
            summary[t+"/"+p]={f:[sum(ok for _,ok in v),len(v)] for f,v in results.items()}
            good={f for f,v in results.items() if len({trial for trial,_ in v})==len(v)>=3 and all(ok for _,ok in v)}
            enough &=len(good)>=8 and {"fault","long","multibend","necessary_aux"}<=good
    headless=data.get("headless") or []
    headless_tasks={r["task"] for r in headless if not r.get("gui",True) and r["status"]=="passed" and r["planner_source"]=="hybrid_joint" and r["admissions"]
                    and r.get("publication_event",{}).get("planning_stages",{}).get("policy_status")=="CERTIFIED_POLICY_COMPLIANT"}
    summary["headless_completed_tasks"]=sorted(headless_tasks)
    record("CLOSED_LOOP",bool(gaz),enough and headless_tasks==set(TASKS),summary)
    clocks=[c for r in gaz+headless for c in r["clocks"]]
    complete_clocks=all({c.get("vehicle") for c in r["clocks"]}==
                        ({"robot1"} if r["task"]=="single" else {"robot1","robot2","robot3"})
                        for r in gaz+headless)
    record("GAZEBO_CLOCK_SCOPE",bool(clocks),enough and complete_clocks and all(c["ros_misses"]==0 and c["stale_motion"]==0 and
           c["max_callback_ms"]<=5 and c["real_time_factor"] is not None and c["matched_commands"]>0 for c in clocks),
           dict(samples=len(clocks),stale_motion=sum(c["stale_motion"] for c in clocks),ros_misses=sum(c["ros_misses"] for c in clocks),
                matched_commands=sum(c["matched_commands"] for c in clocks),max_callback_ms=max([c["max_callback_ms"] for c in clocks],default=None)))
    admission=data["admission"] or {}
    delays_ok=all(len(admission.get(t+"/"+p,[]))>=30 and quant(admission[t+"/"+p],.95)<=ceiling for t,ceiling in
                  (("single",.2),("formation",.4)) for p in PROFILES)
    record("DELIVERY_AND_ADMISSION",bool(admission),delays_ok,{k:dict(samples=len(v),p95=quant(v,.95)) for k,v in admission.items()})
    default=data["software_defaults"];smoke=data["default_runs"] or []
    defaults_ok=default["planners"]==dict(single="hybrid_joint",multi="hybrid_joint") and default["tracking_implementation"]=="cpp"
    record("DEFAULT_INTEGRATION",True,defaults_ok and {r["entry"] for r in smoke if r["status"]=="passed" and r["native"]}=={"single","multi","interactive"},
           dict(test_default=default["planners"],tracker=default["tracking_implementation"],runs=smoke))
    hashes=data["inputs_sha256"];manifest=data["source_manifest"]
    provenance=audit_rows(rows,witness,{p:REPORT/("case_manifest.json" if p=="standard" else "case_manifest_baseline.json") for p in PROFILES},manifest)
    evidence_manifest=load(out/"manifest.json") or {}
    compact_hashes=evidence_manifest.get("compact_hashes",{})
    compact_integrity=all((out/p).is_file() and sha(out/p)==v for p,v in compact_hashes.items())
    has_scalars={"measurements.json","runs.csv"}<=set(compact_hashes)
    runtime_files=(manifest or {}).get("runtime_binaries",{})
    runtime_provenance=bool(runtime_files) and all(
        r.get("installed_binaries")==runtime_files and r.get("source_commit")==manifest["system_commit"]
        for r in gaz+headless)
    integrity=runtime_provenance and compact_integrity and has_scalars and all((ROOT/p).exists() and sha(ROOT/p)==v for p,v in hashes.items()) and len(hashes)>=3
    record("REPRODUCIBLE_EVIDENCE",True,provenance["verified"] and integrity and bool(rows) and all(
           c["status"]=="PASS" for k,c in checks.items() if k!="DEFAULT_INTEGRATION"),dict(input_hashes=hashes,source_commit=(manifest or {}).get("binary_commit"),provenance=provenance))
    if set(checks)!=set(cfg):raise ValueError("gate coverage differs from frozen specification")
    completed=all(g["status"]=="PASS" for g in checks.values())
    final={"schema":spec["schema"],"task_state":"COMPLETED" if completed else "INCOMPLETE","hardware":"HARDWARE_NOT_VERIFIED",
           "checks":[dict(id=k,**checks[k]) for k in cfg],"measurements_sha256":sha(out/"measurements.json")}
    write(out/"acceptance.json",final)
    if not args.compact:
        from report_summary import write_summary
        write_summary(out,final,data)
    print(final["task_state"],{k:v["status"] for k,v in checks.items() if v["status"]!="PASS"})
    return 0 if completed else 1

if __name__=="__main__":raise SystemExit(main())
