"""Certified curriculum BC pretraining and masked PPO, with restartable models."""
import argparse
from dataclasses import replace
from functools import partial
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import subprocess
import time

import numpy as np

from .dataset import Corpus, artifact_directory, make_environment
from .evaluate import check_model_manifest, evaluate_rollouts
from .kernel_adapter import PROJECT_ROOT


def write_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(".tmp.json")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")
    temporary.replace(path)


def save_model(model, output, name):
    temporary = output/(name+".partial.zip")
    model.save(temporary)
    temporary.replace(output/(name+".zip"))
    meta = json.loads((output/"manifest.json").read_text())
    meta["models_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                            for p in output.glob("*.zip") if ".partial." not in p.name}
    write_json(output/"manifest.json", meta)


def model_rank(evaluation, validation_nll=0.):
    summary = evaluation["summary"]
    return (summary["success_rate"], -(summary["mean_execution_s"] or float("inf")), -validation_nll)


def run_manifest(args, corpus, output):
    dependencies = {d.metadata["Name"]: d.version for d in importlib.metadata.distributions()}
    meta = {"schema": "masked_planning_model_v1", "stage": args.stage,
            "contract_sha256": corpus.settings.contract.identity,
            "action_library_sha256": corpus.settings.library.identity,
            "settings_sha256": corpus.settings.identity,
            "dataset_sha256": corpus.manifest["dataset_sha256"],
            "dataset_scope": corpus.manifest["scope"],
            "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
            "dependencies": dependencies, "device": args.device,
            "code_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=PROJECT_ROOT, text=True).strip(),
            "working_tree_dirty": bool(subprocess.check_output(["git", "status", "--porcelain"], cwd=PROJECT_ROOT)),
            "source_files_sha256": {str(p.relative_to(PROJECT_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                                    for directory in (PROJECT_ROOT/"rl/src/coptrans_rl", PROJECT_ROOT/"catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer")
                                    for p in sorted(directory.glob("*.py"))}}
    if args.device == "cuda":
        import torch
        properties = torch.cuda.get_device_properties(0)
        meta["gpu"] = {"name": properties.name, "memory_bytes": properties.total_memory,
                       "cuda_runtime": torch.version.cuda,
                       "cudnn_version": torch.backends.cudnn.version()}
    if getattr(args, "initial_model", None):
        meta["initial_model_sha256"] = hashlib.sha256(args.initial_model.read_bytes()).hexdigest()
    write_json(output/"manifest.json", meta)
    return meta


def build_model(env, args):
    from sb3_contrib import MaskablePPO
    from .policy import PlanningFeatures
    return MaskablePPO("MultiInputPolicy", env, device=args.device, seed=args.seed,
                       n_steps=args.rollout_steps, batch_size=args.batch_size,
                       n_epochs=args.ppo_epochs, learning_rate=args.learning_rate,
                       gamma=1., gae_lambda=.95, clip_range=.2, ent_coef=.01,
                       target_kl=.02, verbose=1,
                       policy_kwargs={"features_extractor_class": PlanningFeatures,
                                      "normalize_images": False,
                                      "net_arch": {"pi": [128, 64], "vf": [128, 64]}})


def teacher_arrays(corpus, split, max_decisions):
    env = make_environment(corpus.directory, split, max_decisions)
    observations, labels = [], []
    for row in corpus.split(split):
        obs, _ = env.reset(options={"problem": corpus.problem(row)})
        for j, action in enumerate(row["action_ids"]):
            if not env.action_masks()[action]:
                raise ValueError("teacher_label_is_masked:"+row["task_id"])
            observations.append(obs); labels.append(action)
            obs, _, done, truncated, info = env.step(action)
            if done and j != len(row["action_ids"])-1:
                raise ValueError("teacher_terminates_early:"+row["task_id"])
        if not done or truncated or not info["is_success"]:
            raise ValueError("teacher_replay_not_certified:"+row["task_id"])
    env.close()
    if not labels:
        raise ValueError("no_teacher_labels:"+split)
    return {key: np.stack([obs[key] for obs in observations]) for key in observations[0]}, np.asarray(labels, dtype=np.int64)


def bc(model, corpus, args, output):
    import torch
    rng = np.random.default_rng(args.seed)
    preparation_begin = time.perf_counter()
    print("Replaying certified train/val teachers...", flush=True)
    train_x, train_y = teacher_arrays(corpus, "train", args.max_decisions)
    val_x, val_y = teacher_arrays(corpus, "val", args.max_decisions)
    print("BC transitions", len(train_y), "validation", len(val_y), flush=True)
    preparation_s = time.perf_counter()-preparation_begin
    history, best_rank, best_loss, stale = [], None, float("inf"), 0
    begun = time.perf_counter()

    def loss(x, y, indices):
        observations = {key: torch.as_tensor(values[indices], device=model.device) for key, values in x.items()}
        distribution = model.policy.get_distribution(
            observations, action_masks=observations["valid_action_mask"].bool())
        return -distribution.log_prob(torch.as_tensor(y[indices], device=model.device)).mean()

    for epoch in range(1, args.epochs+1):
        model.policy.set_training_mode(True)
        losses = []
        order = rng.permutation(len(train_y))
        for begin in range(0, len(order), args.batch_size):
            indices = order[begin:begin+args.batch_size]
            value = loss(train_x, train_y, indices)
            model.policy.optimizer.zero_grad()
            value.backward()
            torch.nn.utils.clip_grad_norm_(model.policy.parameters(), .5)
            model.policy.optimizer.step()
            losses.append(float(value.detach()))
        model.policy.set_training_mode(False)
        with torch.no_grad():
            validation_sum = 0.
            for i in range(0, len(val_y), args.batch_size):
                indices = np.arange(i, min(i+args.batch_size, len(val_y)))
                validation_sum += float(loss(val_x, val_y, indices))*len(indices)
            validation = validation_sum/len(val_y)
        row = {"epoch": epoch, "train_nll": float(np.mean(losses)), "val_nll": validation,
               "wall_s": time.perf_counter()-begun}
        if epoch == 1 or epoch % args.eval_every == 0 or epoch == args.epochs:
            evaluation = evaluate_rollouts(model, corpus.directory, episodes=args.eval_episodes,
                                           max_decisions=args.max_decisions)
            row.update(evaluation["summary"])
            rank = model_rank(evaluation, validation)
            if best_rank is None or rank > best_rank:
                best_rank = rank
                save_model(model, output, "best_model")
                write_json(output/"best_validation.json", evaluation)
        history.append(row)
        write_json(output/"bc_history.json", history)
        save_model(model, output, "latest_model")
        print(json.dumps(row), flush=True)
        if validation < best_loss-1e-4:
            best_loss, stale = validation, 0
        else:
            stale += 1
        if stale >= args.patience:
            break
    write_json(output/"training_summary.json", {"stage": "bc", "epochs": len(history),
               "train_transitions": len(train_y), "validation_transitions": len(val_y),
               "teacher_preparation_s": preparation_s,
               "wall_s": time.perf_counter()-begun,
               "total_prepare_and_train_s": preparation_s+time.perf_counter()-begun})


def ppo(model, corpus, args, output):
    from stable_baselines3.common.callbacks import BaseCallback
    from stable_baselines3.common.logger import configure

    class Progress(BaseCallback):
        def __init__(self):
            super().__init__()
            self.last_eval = self.last_checkpoint = 0
            self.best = None
            self.episodes, self.history = [], []
            self.begun = time.perf_counter()

        def _on_step(self):
            for done, info in zip(self.locals["dones"], self.locals["infos"]):
                if done:
                    self.episodes.append({"success": info.get("is_success", False),
                                          "execution_s": info.get("execution_seconds")})
            if self.num_timesteps-self.last_checkpoint >= args.checkpoint_every:
                save_model(model, output, "checkpoint_%d" % self.num_timesteps)
                self.last_checkpoint = self.num_timesteps
            if self.num_timesteps-self.last_eval >= args.eval_every:
                self.evaluate()
                self.last_eval = self.num_timesteps
            return True

        def _on_rollout_end(self):
            wall = time.perf_counter()-self.begun
            recent = self.episodes[-100:]
            self.logger.record("planning/success_last100", float(np.mean([r["success"] for r in recent])) if recent else 0.)
            self.logger.record("planning/wall_fps", (self.num_timesteps-initial_steps)/max(wall, 1e-9))

        def evaluate(self):
            report = evaluate_rollouts(model, corpus.directory, episodes=args.eval_episodes,
                                       max_decisions=args.max_decisions)
            row = dict(report["summary"], timesteps=self.num_timesteps,
                       wall_s=time.perf_counter()-self.begun)
            self.history.append(row)
            write_json(output/"ppo_validation.json", self.history)
            rank = model_rank(report)
            if self.best is None or rank > self.best:
                self.best = rank
                save_model(model, output, "best_model")
                write_json(output/"best_validation.json", report)
            print("Validation", json.dumps(row), flush=True)

    initial_steps = model.num_timesteps
    model.set_logger(configure(str(output), ["stdout", "csv"]))
    callback = Progress()
    begun = time.perf_counter()
    # Evaluate the initial policy so a worse PPO update never silently replaces
    # a stronger BC policy as the selected model.
    callback.init_callback(model)
    callback.num_timesteps = model.num_timesteps
    callback.evaluate()
    model.learn(total_timesteps=args.timesteps, callback=callback,
                reset_num_timesteps=False, log_interval=1)
    save_model(model, output, "latest_model")
    callback.evaluate()
    write_json(output/"training_summary.json", {"stage": "ppo",
               "requested_transitions": args.timesteps,
               "actual_new_transitions": model.num_timesteps-initial_steps,
               "total_transitions": model.num_timesteps,
               "wall_s": time.perf_counter()-begun})


def main():
    command_begin = time.perf_counter()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("bc", "ppo"))
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--initial-model", type=Path)
    parser.add_argument("--device", choices=("cuda", "cpu", "auto"), default="cuda")
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--envs", type=int, default=4)
    parser.add_argument("--torch-threads", type=int, default=2)
    parser.add_argument("--max-decisions", type=int, default=256)
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--learning-rate", type=float, default=3e-4)
    parser.add_argument("--rollout-steps", type=int, default=512)
    parser.add_argument("--ppo-epochs", type=int, default=10)
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--patience", type=int, default=8)
    parser.add_argument("--timesteps", type=int, default=200000)
    parser.add_argument("--eval-every", type=int)
    parser.add_argument("--eval-episodes", type=int, default=20)
    parser.add_argument("--checkpoint-every", type=int, default=50000)
    args = parser.parse_args()
    args.eval_every = args.eval_every or (5 if args.stage == "bc" else 50000)
    for name in ("envs", "torch_threads", "max_decisions", "batch_size", "rollout_steps",
                 "ppo_epochs", "epochs", "patience", "timesteps", "eval_every", "eval_episodes", "checkpoint_every"):
        if getattr(args, name) < 1:
            parser.error(name+" must be positive")
    if not math.isfinite(args.learning_rate) or args.learning_rate <= 0:
        parser.error("learning_rate must be positive and finite")
    if args.batch_size > args.rollout_steps*(args.envs if args.stage == "ppo" else 1):
        parser.error("batch_size exceeds rollout capacity")
    import torch
    from sb3_contrib import MaskablePPO
    from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
    torch.set_num_threads(args.torch_threads)
    if args.device == "auto":
        args.device = "cuda" if torch.cuda.is_available() else "cpu"
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable: use CUDA PyTorch in a GPU-visible host terminal; --device cpu is an explicit diagnostic option")
    print("Training device:", args.device,
          torch.cuda.get_device_name(0) if args.device == "cuda" else "CPU", flush=True)
    corpus = Corpus(args.dataset)
    if not corpus.split("train") or not corpus.split("val"):
        raise ValueError("training_requires_train_and_val_splits")
    output = artifact_directory(args.output)
    run_manifest(args, corpus, output)
    factories = [partial(make_environment, str(args.dataset), "train", args.max_decisions)
                 for _ in range(args.envs if args.stage == "ppo" else 1)]
    env = SubprocVecEnv(factories, start_method="forkserver") if len(factories) > 1 else DummyVecEnv(factories)
    try:
        if args.initial_model:
            check_model_manifest(args.initial_model, corpus)
            model = MaskablePPO.load(
                args.initial_model, env=env, device=args.device, seed=args.seed,
                n_steps=args.rollout_steps, batch_size=args.batch_size,
                n_epochs=args.ppo_epochs, learning_rate=args.learning_rate,
                gamma=1., gae_lambda=.95, ent_coef=.01, target_kl=.02)
        else:
            model = build_model(env, args)
        if args.stage == "bc":
            bc(model, corpus, args, output)
        else:
            ppo(model, corpus, args, output)
        manifest = json.loads((output/"manifest.json").read_text())
        manifest["main_wall_s"] = time.perf_counter()-command_begin
        manifest["models_sha256"] = {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.glob("*.zip")}
        write_json(output/"manifest.json", manifest)
    finally:
        env.close()


if __name__ == "__main__":
    main()
