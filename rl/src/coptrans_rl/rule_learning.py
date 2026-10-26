"""Residual candidate ranking with masked PPO; rules remain an explicit baseline."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import torch
from torch import nn
from torch.distributions import Categorical

from .dataset import Corpus, artifact_directory
from .kernel_adapter import PROJECT_ROOT, stable_hash
from .rule_frontend import RuleFrontend, FrontendConfig, solve
from .benchmark import engineering_tasks


class CandidateRanker(nn.Module):
    """Shared edge scorer: arbitrary slot order, rule score + learned residual."""
    schema = "rule_residual_candidate_ppo_v1"

    def __init__(self):
        super().__init__()
        self.map_encoder = nn.Sequential(nn.Conv2d(2,8,3,2,1),nn.ReLU(),
            nn.Conv2d(8,16,3,2,1),nn.ReLU(),nn.AdaptiveAvgPool2d((2,2)),
            nn.Flatten(),nn.Linear(64,32),nn.ReLU())
        self.edge_encoder = nn.Sequential(nn.Linear(RuleFrontend.feature_dim+32,64),nn.Tanh(),nn.Linear(64,32),nn.Tanh())
        self.actor = nn.Linear(32,1)
        self.critic = nn.Sequential(nn.Linear(64,64),nn.Tanh(),nn.Linear(64,1))
        nn.init.zeros_(self.actor.weight); nn.init.zeros_(self.actor.bias)

    def forward(self, obs):
        context = self.map_encoder(obs["map"])
        features = self.edge_encoder(torch.cat([obs["candidates"],
            context[:,None,:].expand(-1,obs["candidates"].shape[1],-1)],dim=-1))
        mask = obs["mask"].bool()
        if not torch.all(mask.any(dim=1)):
            raise ValueError("no_frontier_candidate")
        residual = self.actor(features).squeeze(-1)
        logits = (obs["rule_logits"]+residual).masked_fill(~mask,-1e9)
        pooled = (features*mask[:,:,None]).sum(dim=1)/mask.sum(dim=1,keepdim=True)
        value = self.critic(torch.cat([context,pooled],dim=1)).squeeze(-1)
        return Categorical(logits=logits),value


def tensors(obs, device, batched=False):
    return {k:torch.as_tensor(v if batched else v[None],device=device) for k,v in obs.items()}


def selector(model, device):
    def choose(obs):
        with torch.no_grad():
            distribution,_ = model(tensors(obs,device))
            return int(distribution.logits.argmax(dim=1).item())
    return choose


def source_identity():
    files = [PROJECT_ROOT/"rl/src/coptrans_rl"/name for name in
             ("rule_candidates.py","rule_frontend.py","rule_learning.py","local_connections.py")]
    files += [PROJECT_ROOT/"rl/native/local_connections.cpp",
              PROJECT_ROOT/"catkin_ws/src/fwis_common/include/fwis_common/reeds_shepp.hpp"]
    return {str(p.relative_to(PROJECT_ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}


def save(model, optimizer, path, identity):
    temporary = path.with_suffix(".partial.pt")
    torch.save({"schema":model.schema,"state_dict":model.state_dict(),
                "optimizer":optimizer.state_dict(),"identity":identity},temporary)
    temporary.replace(path)


def load(path, device, identity):
    checkpoint = torch.load(path,map_location=device,weights_only=True)
    if checkpoint["schema"] != CandidateRanker.schema or checkpoint["identity"] != identity:
        raise ValueError("rule_model_contract_or_source_mismatch")
    model = CandidateRanker().to(device)
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    return model


def evaluate(problems, config, model=None, device="cpu", trace=False):
    choose = selector(model,device) if model is not None else None
    results = []
    for problem in problems:
        row = solve(problem,config,choose,include_trace=trace)
        row["start"],row["goal"] = asdict(problem.start),asdict(problem.goal)
        results.append(row)
    passed = [r for r in results if r["is_success"]]
    return {"summary":{"tasks":len(results),"successes":len(passed),
             "success_rate":len(passed)/len(results) if results else 0.,
             "mean_execution_s":float(np.mean([r["execution_s"] for r in passed])) if passed else None,
             "mean_wall_s":float(np.mean([r["wall_s"] for r in results])) if results else None},
            "tasks":results}


def rank(report):
    s = report["summary"]
    return (s["successes"],-(s["mean_execution_s"] if s["mean_execution_s"] is not None else float("inf")))


def json_write(path, value):
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n")


def train(args, corpus, config, output, identity):
    torch.manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    model = CandidateRanker().to(args.device)
    initial_parameters = [p.detach().clone() for p in model.parameters()]
    print("Rule frontend PPO device:",args.device,
          torch.cuda.get_device_name(0) if args.device=="cuda" else "CPU",flush=True)
    optimizer = torch.optim.Adam(model.parameters(),lr=args.learning_rate)
    validation = [corpus.problem(r) for r in corpus.split("val")[:args.eval_episodes]]
    baseline = evaluate(validation,config)
    json_write(output/"rule_validation.json",baseline)
    json_write(output/"best_validation.json",baseline)
    best_report = baseline
    best = rank(baseline)
    save(model,optimizer,output/"best.pt",identity)
    # Zero actor residual is exactly the rule choice, tested independently.
    env = RuleFrontend(config)
    train_rows = corpus.split("train")
    reset_rejections = []
    def reset():
        for _ in range(100):
            task = train_rows[int(rng.integers(len(train_rows)))]
            observation = env.reset(corpus.problem(task))
            if not env.done:
                return observation
            reset_rejections.append(task["task_id"])
        raise RuntimeError("no_nonterminal_training_task")
    obs = reset()
    history, total, begun = [],0,time.perf_counter()
    while total < args.steps:
        batch, rewards, dones, values, actions, old_logs = [],[],[],[],[],[]
        completed = []
        for _ in range(min(args.rollout_steps,args.steps-total)):
            with torch.no_grad():
                distribution,value = model(tensors(obs,args.device))
                action = distribution.sample()
            batch.append(obs); values.append(float(value.item()))
            actions.append(int(action.item())); old_logs.append(float(distribution.log_prob(action).item()))
            obs,reward,done = env.step(actions[-1])
            rewards.append(reward); dones.append(done); total += 1
            if done:
                completed.append(env.report(False))
                obs = reset()
        with torch.no_grad():
            _,next_value = model(tensors(obs,args.device))
        advantages = np.zeros(len(rewards),dtype=np.float32)
        following,gae = float(next_value.item()),0.
        for i in reversed(range(len(rewards))):
            delta = rewards[i]+following*(not dones[i])-values[i]
            gae = delta+.95*(not dones[i])*gae
            advantages[i] = gae
            following = values[i]
        returns = advantages+np.asarray(values,dtype=np.float32)
        advantages = (advantages-advantages.mean())/(advantages.std()+1e-8)
        stacked = {k:np.stack([o[k] for o in batch]) for k in batch[0]}
        data = tensors(stacked,args.device,True)
        action_t = torch.tensor(actions,device=args.device)
        logs_t = torch.tensor(old_logs,device=args.device)
        adv_t = torch.tensor(advantages,device=args.device)
        return_t = torch.tensor(returns,device=args.device)
        kl, loss_number = 0.,0.
        for _ in range(args.ppo_epochs):
            distribution,value = model(data)
            log = distribution.log_prob(action_t)
            difference = log-logs_t
            kl = float(((difference.exp()-1)-difference).mean().detach())
            if kl > .03:
                break
            ratio = difference.exp()
            actor = -torch.minimum(ratio*adv_t,ratio.clamp(.8,1.2)*adv_t).mean()
            loss = actor+.5*(value-return_t).square().mean()-.01*distribution.entropy().mean()
            optimizer.zero_grad(); loss.backward()
            nn.utils.clip_grad_norm_(model.parameters(),.5)
            optimizer.step(); loss_number = float(loss.detach())
        row = {"steps":total,"episodes":len(completed),
               "successes":sum(r["is_success"] for r in completed),
               "loss":loss_number,"kl":kl,"wall_s":time.perf_counter()-begun}
        report = evaluate(validation,config,model,args.device)
        row["validation"] = report["summary"]
        if rank(report)>best:
            best = rank(report)
            best_report = report
            save(model,optimizer,output/"best.pt",identity)
            json_write(output/"best_validation.json",report)
        save(model,optimizer,output/"latest.pt",identity)
        history.append(row); json_write(output/"history.json",history)
        print(json.dumps(row),flush=True)
    json_write(output/"training_summary.json",{"steps":total,"device":args.device,
        "gpu":torch.cuda.get_device_name(0) if args.device=="cuda" else None,
        "wall_s":time.perf_counter()-begun,"baseline":baseline["summary"],
        "best_validation":best_report["summary"],"reset_rejected_tasks":reset_rejections,
        "gradient_is_cuda":all(p.grad.is_cuda for p in model.parameters() if p.grad is not None),
        "finite_gradients":all(bool(torch.isfinite(p.grad).all()) for p in model.parameters() if p.grad is not None),
        "parameter_l2_change":float(torch.sqrt(sum((p.detach()-before).square().sum()
               for p,before in zip(model.parameters(),initial_parameters))))})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage",choices=("train","evaluate"))
    parser.add_argument("--dataset",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--model",type=Path)
    parser.add_argument("--device",choices=("cpu","cuda"),default="cuda")
    parser.add_argument("--split",choices=("train","val","test","ood"),default="val")
    parser.add_argument("--episodes",type=int,default=20)
    parser.add_argument("--maps",action="store_true")
    parser.add_argument("--steps",type=int,default=512)
    parser.add_argument("--rollout-steps",type=int,default=128)
    parser.add_argument("--ppo-epochs",type=int,default=4)
    parser.add_argument("--learning-rate",type=float,default=5e-5)
    parser.add_argument("--eval-episodes",type=int,default=8)
    parser.add_argument("--seed",type=int,default=7)
    parser.add_argument("--budget",type=float,default=5.)
    parser.add_argument("--decisions",type=int,default=128)
    args = parser.parse_args()
    for key in ("episodes","steps","rollout_steps","ppo_epochs","eval_episodes"):
        if getattr(args,key)<1:
            parser.error(key+" must be positive")
    if not np.isfinite(args.learning_rate) or args.learning_rate<=0:
        parser.error("learning rate must be positive and finite")
    torch.set_num_threads(2)
    if args.device=="cuda" and (args.stage=="train" or args.model) and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable")
    corpus = Corpus(args.dataset)
    config = FrontendConfig(budget_s=args.budget,max_decisions=args.decisions)
    identity = {"schema":RuleFrontend.schema,"contract":corpus.settings.contract.identity,
                "sources":source_identity(),"features":RuleFrontend.feature_dim,
                "direction_policy":corpus.settings.problem_kwargs}
    output = artifact_directory(args.output)
    json_write(output/"manifest.json",{"identity":identity,"config":asdict(config),
        "arguments":{k:str(v) if isinstance(v,Path) else v for k,v in vars(args).items()},
        "dataset_sha256":corpus.manifest["dataset_sha256"],
        "commit":subprocess.check_output(["git","rev-parse","HEAD"],cwd=PROJECT_ROOT,text=True).strip(),
        "working_tree_dirty":bool(subprocess.check_output(["git","status","--porcelain"],cwd=PROJECT_ROOT))})
    if args.stage=="train":
        if not corpus.split("train") or not corpus.split("val"):
            raise ValueError("training_requires_train_and_val")
        train(args,corpus,config,output,identity)
    else:
        model = load(args.model,args.device,identity) if args.model else None
        problems = ([p for p,_ in engineering_tasks(corpus.settings.contract)] if args.maps else
                    [corpus.problem(r) for r in corpus.split(args.split)[:args.episodes]])
        report = evaluate(problems,config,model,args.device,trace=True)
        report["method"] = "rule_residual_rl" if model else "rules_only"
        json_write(output/"evaluation.json",report)
        print(json.dumps(report["summary"],indent=2))


if __name__=="__main__":
    main()
