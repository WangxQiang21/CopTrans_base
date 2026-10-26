"""Masked model rollout with final certificates, without classical full search."""
import argparse
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .dataset import Corpus, artifact_directory, make_environment


def evaluate_rollouts(model, directory, split="val", episodes=20,
                      max_decisions=256, seed=20261026, include_trace=False):
    corpus = Corpus(directory)
    rows = corpus.split(split)[:episodes]
    if not rows:
        raise ValueError("empty_evaluation_split")
    env = make_environment(directory, split, max_decisions)
    results = []
    for index, row in enumerate(rows):
        begun = time.perf_counter()
        obs, _ = env.reset(seed=seed+index, options={"problem": corpus.problem(row)})
        action_ids, reward_sum, policy_s = [], 0., 0.
        while True:
            tick = time.perf_counter()
            action, _ = model.predict(obs, deterministic=True, action_masks=env.action_masks())
            policy_s += time.perf_counter()-tick
            action = int(np.asarray(action).item())
            action_ids.append(action)
            obs, reward, terminated, truncated, info = env.step(action)
            reward_sum += reward
            if terminated or truncated:
                break
        item = {"task_id": row["task_id"], "family": row["family"],
                "seed": seed+index, "action_ids": action_ids, "reward_sum": reward_sum,
                "wall_s": time.perf_counter()-begun, "policy_wall_s": policy_s,
                "info": dict(info, **env.diagnostics()),
                "teacher_execution_s": row["certificate"]["timing"]["duration_s"]}
        if env.last_finalization:
            item["finalization"] = env.last_finalization.report(include_trace)
        results.append(item)
    env.close()
    success = [row for row in results if row["info"]["is_success"]]
    summary = {"split": split, "episodes": len(results), "successes": len(success),
               "success_rate": len(success)/len(results),
               "mean_execution_s": float(np.mean([r["info"]["execution_seconds"] for r in success])) if success else None,
               "mean_wall_s": float(np.mean([r["wall_s"] for r in results])),
               "scope": corpus.manifest["scope"],
               "dataset_sha256": corpus.manifest["dataset_sha256"]}
    return {"summary": summary, "episodes": results}


def check_model_manifest(model_path, corpus):
    model_path = Path(model_path)
    manifest_path = model_path.parent/"manifest.json"
    if not manifest_path.is_file():
        raise ValueError("missing_model_manifest")
    meta = json.loads(manifest_path.read_text())
    if (meta["contract_sha256"] != corpus.settings.contract.identity or
            meta["action_library_sha256"] != corpus.settings.library.identity or
            meta["settings_sha256"] != corpus.settings.identity):
        raise ValueError("model_environment_contract_mismatch")
    expected = meta.get("models_sha256", {}).get(model_path.name)
    if expected != hashlib.sha256(model_path.read_bytes()).hexdigest():
        raise ValueError("model_checkpoint_hash_mismatch")
    from .kernel_adapter import PROJECT_ROOT
    observation = PROJECT_ROOT/"rl/src/coptrans_rl/observation.py"
    if (meta.get("source_files_sha256", {}).get("rl/src/coptrans_rl/observation.py") !=
            hashlib.sha256(observation.read_bytes()).hexdigest()):
        raise ValueError("model_observation_schema_code_mismatch")
    return meta


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--split", choices=("train", "val", "test", "ood"), default="test")
    parser.add_argument("--episodes", type=int, default=500)
    parser.add_argument("--max-decisions", type=int, default=256)
    parser.add_argument("--torch-threads", type=int, default=2)
    parser.add_argument("--device", choices=("cuda", "cpu", "auto"), default="cuda")
    args = parser.parse_args()
    import torch
    from sb3_contrib import MaskablePPO
    torch.set_num_threads(args.torch_threads)
    if args.device == "auto":
        args.device = "cuda" if torch.cuda.is_available() else "cpu"
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA unavailable in this interpreter or execution environment")
    corpus = Corpus(args.dataset)
    check_model_manifest(args.model, corpus)
    output = artifact_directory(args.output)
    model = MaskablePPO.load(args.model, device=args.device)
    report = evaluate_rollouts(model, args.dataset, args.split, args.episodes,
                               args.max_decisions, include_trace=True)
    report["model_sha256"] = hashlib.sha256(args.model.read_bytes()).hexdigest()
    (output/"evaluation.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
