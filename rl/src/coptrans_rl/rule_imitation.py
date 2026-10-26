"""Warm-start the candidate selector from explicitly offline oracle decisions.

This is supervised pretraining, not evidence of RL improvement or blind testing.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import torch

from .configuration import load_settings
from .dataset import artifact_directory
from .kernel_adapter import PROJECT_ROOT
from .rule_frontend import RuleFrontend
from .rule_learning import CandidateRanker,source_identity,tensors,save,json_write


def load_teachers(directory, holdout_prefixes=()):
    report = json.loads((directory/"evaluation.json").read_text())
    if report.get("method")!="reference_oracle_diagnostic":
        raise ValueError("not_an_offline_reference_teacher")
    with np.load(directory/"teacher_observations.npz",allow_pickle=False) as data:
        x = {key:data[key].copy() for key in ("candidates","rule_logits","mask","map")}
        y, tasks = data["labels"].copy(),data["task_ids"].copy()
    n = len(y)
    if (not n or x["candidates"].shape!=(n,64,RuleFrontend.feature_dim) or
            x["rule_logits"].shape!=(n,64) or x["mask"].shape!=(n,64) or
            x["map"].shape!=(n,2,32,32) or tasks.shape!=(n,) or
            not np.issubdtype(y.dtype,np.integer) or np.any(y<0) or np.any(y>=64)):
        raise ValueError("teacher_array_schema_mismatch")
    if not all(np.isfinite(a).all() for a in x.values()):
        raise ValueError("nonfinite_teacher_observation")
    if not np.all(x["mask"][np.arange(n),y]):
        raise ValueError("masked_teacher_label")
    certified = {r["task_id"] for r in report["tasks"] if r.get("is_success") and
                 r.get("finalization",{}).get("status")=="FEASIBLE" and
                 all(r["finalization"].get("audit",{}).get(key)=="PASS" for key in
                     ("geometry_status","dynamic_status","timing_status"))}
    if not set(tasks).issubset(certified):
        raise ValueError("teacher_task_lacks_terminal_certificate")
    validation = np.asarray([any(str(t).startswith(p) for p in holdout_prefixes) for t in tasks])
    return x,y,tasks,~validation,validation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--teachers",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--device",choices=("cpu","cuda"),default="cuda")
    parser.add_argument("--epochs",type=int,default=100)
    parser.add_argument("--batch-size",type=int,default=128)
    parser.add_argument("--learning-rate",type=float,default=1e-3)
    parser.add_argument("--holdout-prefix",action="append",default=[])
    parser.add_argument("--seed",type=int,default=7)
    args = parser.parse_args()
    if args.epochs<1 or args.batch_size<1 or not np.isfinite(args.learning_rate) or args.learning_rate<=0:
        parser.error("invalid training limits")
    torch.set_num_threads(2); torch.manual_seed(args.seed)
    if args.device=="cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    x,y,tasks,train_mask,val_mask = load_teachers(args.teachers,args.holdout_prefix)
    if not train_mask.any() or (args.holdout_prefix and not val_mask.any()):
        raise ValueError("empty_teacher_train_or_requested_holdout")
    output = artifact_directory(args.output)
    settings = load_settings()
    identity = {"schema":RuleFrontend.schema,"contract":settings.contract.identity,
                "sources":source_identity(),"features":RuleFrontend.feature_dim,
                "direction_policy":settings.problem_kwargs}
    json_write(output/"manifest.json",{"identity":identity,
        "scope":"engineering_reference_pretraining_not_blind_evaluation",
        "arguments":{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        "teacher_arrays_sha256":hashlib.sha256((args.teachers/"teacher_observations.npz").read_bytes()).hexdigest(),
        "teacher_audit_sha256":hashlib.sha256((args.teachers/"evaluation.json").read_bytes()).hexdigest(),
        "training_tasks":sorted(set(tasks[train_mask])),"heldout_tasks":sorted(set(tasks[val_mask])),
        "commit":subprocess.check_output(["git","rev-parse","HEAD"],cwd=PROJECT_ROOT,text=True).strip(),
        "working_tree_dirty":bool(subprocess.check_output(["git","status","--porcelain"],cwd=PROJECT_ROOT))})
    model = CandidateRanker().to(args.device)
    optimizer = torch.optim.Adam(model.parameters(),lr=args.learning_rate)
    observations = tensors(x,args.device,True)
    labels = torch.tensor(y,device=args.device)
    rng = np.random.default_rng(args.seed)
    train_indices,val_indices = np.flatnonzero(train_mask),np.flatnonzero(val_mask)
    history,best = [],float("inf")
    begun = time.perf_counter()
    def score(indices):
        if not len(indices):
            return None,None
        total,correct = 0.,0
        with torch.no_grad():
            for begin in range(0,len(indices),args.batch_size):
                ix = indices[begin:begin+args.batch_size]
                distribution,_ = model({k:v[ix] for k,v in observations.items()})
                total += float(-distribution.log_prob(labels[ix]).sum())
                correct += int((distribution.logits.argmax(dim=1)==labels[ix]).sum())
        return total/len(indices),correct/len(indices)
    print("Offline teacher pretraining",args.device,"rows",len(y),"train",len(train_indices),"holdout",len(val_indices),flush=True)
    for epoch in range(1,args.epochs+1):
        order = rng.permutation(train_indices)
        for begin in range(0,len(order),args.batch_size):
            ix = order[begin:begin+args.batch_size]
            distribution,_ = model({k:v[ix] for k,v in observations.items()})
            loss = -distribution.log_prob(labels[ix]).mean()
            optimizer.zero_grad(); loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(),1.)
            optimizer.step()
        train_nll,train_accuracy = score(train_indices)
        val_nll,val_accuracy = score(val_indices)
        row = {"epoch":epoch,"train_nll":train_nll,"train_accuracy":train_accuracy,
               "holdout_nll":val_nll,"holdout_accuracy":val_accuracy}
        history.append(row)
        criterion = val_nll if val_nll is not None else train_nll
        if criterion<best:
            best = criterion; save(model,optimizer,output/"best.pt",identity)
        if epoch==1 or epoch%10==0:
            print(json.dumps(row),flush=True)
    save(model,optimizer,output/"latest.pt",identity)
    json_write(output/"history.json",history)
    json_write(output/"training_summary.json",{"epochs":args.epochs,"rows":len(y),
        "train_rows":len(train_indices),"holdout_rows":len(val_indices),
        "scope":"teacher_forced_accuracy_not_planning_success", "wall_s":time.perf_counter()-begun,
        "gpu":torch.cuda.get_device_name(0) if args.device=="cuda" else None,
        "final":history[-1]})


if __name__=="__main__":
    main()
