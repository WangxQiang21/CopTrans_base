"""Offline oracle diagnostic only: does NOT supply reference paths online."""
import argparse
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image
import yaml

from .benchmark import engineering_tasks
from .configuration import load_settings
from .dataset import artifact_directory
from .kernel_adapter import wrap,PROJECT_ROOT
from .rule_frontend import RuleFrontend,FrontendConfig


def check_reference(problem, directory, probe):
    if not np.allclose([problem.start.x,problem.start.y,problem.start.th],probe["start"],atol=1e-7,rtol=0.):
        raise ValueError("reference_start_mismatch")
    if not np.allclose([problem.goal.x,problem.goal.y,problem.goal.th],probe["goal"],atol=1e-7,rtol=0.):
        raise ValueError("reference_goal_mismatch")
    map_directory = directory/"runtime_map"
    if not map_directory.exists():
        run = json.loads((directory/"run.json").read_text())["case_data"]
        map_directory = PROJECT_ROOT/run["map_dir"]
        recorded = {r["path"]:r["sha256"] for r in run["map_files"]}
        config = yaml.safe_load((map_directory/"map.yaml").read_text())
        for name in ("map.yaml",config["image"]):
            if hashlib.sha256((map_directory/name).read_bytes()).hexdigest()!=recorded.get(name):
                raise ValueError("reference_source_map_file_changed:"+name)
    config = yaml.safe_load((map_directory/"map.yaml").read_text())
    with Image.open(map_directory/config["image"]) as image:
        pixels = np.flipud(np.asarray(image.convert("L")))
    occ = pixels <= 255.-255.*float(config.get("occupied_thresh",.65))
    free = pixels >= 255.-255.*float(config.get("free_thresh",.196))
    occ = occ | (~occ & ~free)
    grid = problem.grid
    if not np.array_equal(occ,grid.occ):
        raise ValueError("reference_occupancy_mismatch")
    if not np.allclose([grid.res,grid.ox,grid.oy],
                       [config["resolution"],*config["origin"][:2]],atol=1e-7,rtol=0.):
        raise ValueError("reference_grid_transform_mismatch")


class ReferenceOracle:
    """Monotone local reference matching; can use information unavailable online."""
    def __init__(self, trace, problem):
        poses = np.asarray(trace,dtype=float)[:,1:4]
        # Keep geometry/heading events, dropping exact timed holds.
        keep = np.r_[True,np.max(np.abs(np.diff(poses,axis=0)),axis=1)>1e-8]
        self.poses = poses[keep]
        xy = np.linalg.norm(np.diff(self.poses[:,:2],axis=0),axis=1)
        yaw = np.asarray([abs(wrap(x)) for x in np.diff(self.poses[:,2])])
        self.progress = np.r_[0.,np.cumsum(xy+.4*yaw)]
        self.cache = {():0}
        self.problem = problem

    def choose(self, env):
        choices = []
        for i,branch in enumerate(env.frontier):
            parent = branch.actions[:-len(branch.edge.actions)]
            prior = self.cache.get(parent,0)
            amount = sum(abs(a.amount)*(.4 if a.mode==2 else 1.) for a in branch.edge.actions)
            start = max(0,int(np.searchsorted(self.progress,self.progress[prior]-.15)))
            stop = min(len(self.poses),int(np.searchsorted(self.progress,self.progress[prior]+1.7*amount+.3))+1)
            section = self.poses[start:stop]
            end = branch.edge.end
            dxy = np.linalg.norm(section[:,:2]-[end.x,end.y],axis=1)
            dyaw = np.abs(np.arctan2(np.sin(section[:,2]-end.th),np.cos(section[:,2]-end.th)))
            error = dxy+.3*dyaw
            index = int(np.argmin(error))+start
            score = (12.*float(error[index-start])+.1*branch.proxy+
                     (self.progress[-1]-self.progress[index])/self.problem.limits.target_v)
            choices.append((score,i,index,float(error[index-start])))
        _,choice,index,error = min(choices)
        self.cache[env.frontier[choice].actions] = index
        return choice,index,error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference-root",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--maps",nargs="*")
    parser.add_argument("--budget",type=float,default=10.)
    parser.add_argument("--decisions",type=int,default=512)
    parser.add_argument("--collect",action="store_true",help="save successful oracle decisions for offline imitation")
    args = parser.parse_args()
    output = artifact_directory(args.output)
    from .rule_learning import source_identity
    (output/"manifest.json").write_text(json.dumps({
        "scope":"offline_reference_teacher_not_online_input",
        "feature_schema":"rule_frontier_features_27_v1",
        "contract":load_settings().contract.identity,
        "sources":source_identity(),
        "arguments":{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        "commit":subprocess.check_output(["git","rev-parse","HEAD"],cwd=PROJECT_ROOT,text=True).strip()
    },indent=2)+"\n")
    comparison = json.loads((args.reference_root/"comparison.json").read_text())
    available = {r["map_id"]:r for r in comparison["backends"]["mode_path_v2"] if r["status"]=="pass"}
    rows,examples,labels,example_tasks = [],[],[],[]
    for problem,inputs in engineering_tasks(load_settings().contract):
        short = problem.task_id.split("-")[0]
        if short not in available or (args.maps and short not in args.maps):
            continue
        reference = args.reference_root/"mode_path_v2"/available[short]["case"]
        path = reference/"probe_result.json"
        probe = json.loads(path.read_text())
        try:
            check_reference(problem,reference,probe)
        except (ValueError,OSError) as error:
            rows.append({"task_id":problem.task_id,"input_error":str(error)})
            print(short,str(error),flush=True)
            continue
        env = RuleFrontend(FrontendConfig(budget_s=args.budget,max_decisions=args.decisions))
        observation = env.reset(problem)
        oracle = ReferenceOracle(probe["timed_trace"],problem)
        matches,episode_x,episode_y = [],[],[]
        while not env.done:
            index,station,error = oracle.choose(env)
            if args.collect:
                episode_x.append(observation); episode_y.append(index)
            matches.append({"reference_station":station,"tracking_error":error})
            observation,_,_ = env.step(index)
        result = env.report(True)
        result.update(start=asdict(problem.start),goal=asdict(problem.goal),
                      reference=str(path),reference_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                      reference_matches=matches,inputs=inputs)
        rows.append(result)
        if args.collect and result["is_success"]:
            examples.extend(episode_x); labels.extend(episode_y)
            example_tasks.extend([problem.task_id]*len(episode_y))
        (output/"partial.json").write_text(json.dumps({"complete":False,"tasks":rows},indent=2,allow_nan=False)+"\n")
        print(short,result["is_success"],result["reason"],result["decisions"],flush=True)
    attempted = [r for r in rows if "input_error" not in r]
    summary = {"tasks":len(attempted),"successes":sum(r["is_success"] for r in attempted),
               "input_errors":len(rows)-len(attempted),"scope":"offline_reference_oracle_not_a_planner_benchmark"}
    report = {"method":"reference_oracle_diagnostic","summary":summary,"tasks":rows,
              "config":{"budget_s":args.budget,"max_decisions":args.decisions},
              "interpretation":"Success proves a constructive candidate chain under this oracle; failure does not prove absence of any chain. Reference paths are not available to the online policy."}
    (output/"evaluation.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n")
    if args.collect and labels:
        arrays = {key:np.stack([obs[key] for obs in examples]) for key in examples[0]}
        np.savez_compressed(output/"teacher_observations.npz",**arrays,
                            labels=np.asarray(labels,dtype=np.int64),task_ids=np.asarray(example_tasks))
    print(json.dumps(summary),flush=True)


if __name__=="__main__":
    main()
