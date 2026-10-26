"""Plot curriculum examples, measured learning curves, and model rollouts."""
import argparse
from collections import Counter
import csv
import json
import math
import os
from pathlib import Path

import numpy as np

from .dataset import Corpus, artifact_directory
from .kernel_adapter import replay
from fwis_v3_optimizer.core import expand


def draw_problem(ax, problem):
    grid = problem.grid
    ax.imshow(grid.occ, cmap="Greys", origin="lower", vmin=0, vmax=1,
              extent=(grid.ox, grid.ox+grid.width*grid.res,
                      grid.oy, grid.oy+grid.height*grid.res), interpolation="nearest")
    for pose, color in ((problem.start, "green"), (problem.goal, "red")):
        ax.arrow(pose.x, pose.y, .5*math.cos(pose.th), .5*math.sin(pose.th),
                 color=color, width=.04, length_includes_head=True)
    ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal")


def dataset_figures(corpus, output):
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    colors = ["#00a8e8", "#ff9400", "#b83cff"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 9), constrained_layout=True)
    families = [f for f in ("open", "corridor", "corner", "parking", "islands")
                if any(row["family"] == f for row in corpus.rows)]
    for ax, family in zip(axes.flat, families):
        row = next(r for r in corpus.rows if r["family"] == family)
        problem = corpus.problem(row)
        draw_problem(ax, problem)
        actions = [corpus.settings.library[i] for i in row["action_ids"]]
        points, pieces = expand(problem.start, actions, .05, math.radians(5))
        for i, action in enumerate(pieces):
            a, b = points[i], points[i+1]
            ax.plot([a.x, b.x], [a.y, b.y], color=colors[action.mode], lw=2)
        for p in points[::max(1, len(points)//10)]:
            hl, hw = problem.length/2, problem.width/2
            box = np.array([[-hl,-hw], [hl,-hw], [hl,hw], [-hl,hw]])
            c, s = np.cos(p.th), np.sin(p.th)
            box = box@np.array([[c,s], [-s,c]])+np.array([p.x,p.y])
            ax.add_patch(Polygon(box, fill=False, edgecolor="#0077b6", alpha=.5))
        ax.set_title(f'{family} / {row["split"]}\ncertified teacher T={row["certificate"]["timing"]["duration_s"]:.2f}s')
    for ax in list(axes.flat)[len(families):5]:
        ax.axis("off")
    ax = axes.flat[-1]
    ax.axis("off")
    ax.text(.02, .9, "Certified curriculum\n\n"+"\n".join(
        f'{split}: {count}' for split, count in corpus.manifest["counts"].items())+
        "\n\nBlue: A; orange: O; purple: Spin\nGreen: start; red: goal\n\nRaw teacher actions shown.\nThese are not learned model outputs.", va="top")
    fig.savefig(output/"curriculum.png", dpi=140); plt.close(fig)
    summary = {"dataset_sha256": corpus.manifest["dataset_sha256"],
               "scope": corpus.manifest["scope"], "counts": corpus.manifest["counts"],
               "attempt_count": corpus.manifest["attempt_count"],
               "proposal_rejections": corpus.manifest["proposal_rejections"],
               "layout_unique_count": len({row["layout_id"] for row in corpus.rows}),
               "family_counts": {s: dict(Counter(r["family"] for r in corpus.split(s))) for s in corpus.manifest["counts"]},
               "training_action_modes": dict(Counter(str(corpus.settings.library[i].mode)
                    for row in corpus.split("train") for i in row["action_ids"]))}
    (output/"data_summary.json").write_text(json.dumps(summary, indent=2)+"\n")


def learning_curves(run, output):
    import matplotlib.pyplot as plt
    history_file = run/"bc_history.json"
    if history_file.exists():
        history = json.loads(history_file.read_text())
        fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
        axes[0].plot([r["epoch"] for r in history], [r["train_nll"] for r in history], label="train NLL")
        axes[0].plot([r["epoch"] for r in history], [r["val_nll"] for r in history], label="validation NLL")
        evaluated = [r for r in history if "success_rate" in r]
        axes[1].plot([r["epoch"] for r in evaluated], [r["success_rate"] for r in evaluated], marker="o")
        axes[0].set(xlabel="BC epoch", ylabel="Masked negative log likelihood")
        axes[1].set(xlabel="BC epoch", ylabel="Certified validation success rate", ylim=(-.02,1.02))
        axes[0].legend()
    else:
        rows = list(csv.DictReader((run/"progress.csv").open()))
        history = json.loads((run/"ppo_validation.json").read_text())
        fig, axes = plt.subplots(1, 2, figsize=(11, 4), constrained_layout=True)
        axes[0].plot([float(r["time/total_timesteps"]) for r in rows],
                     [float(r.get("planning/success_last100") or 0.) for r in rows], label="training last 100 episodes")
        axes[0].plot([r["timesteps"] for r in history], [r["success_rate"] for r in history], marker="o", label="fixed validation tasks")
        axes[1].plot([float(r["time/total_timesteps"]) for r in rows],
                     [float(r.get("planning/wall_fps") or 0.) for r in rows])
        axes[0].set(xlabel="PPO transitions", ylabel="Certified success rate", ylim=(-.02,1.02)); axes[0].legend()
        axes[1].set(xlabel="PPO transitions", ylabel="Measured wall transitions / s")
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle("Measured training diagnostics; curriculum evaluation")
    fig.savefig(output/"learning_curves.png", dpi=150); plt.close(fig)


def representative_episodes(episodes, limit=6):
    """Show both outcomes when available, retaining deterministic task order."""
    successful = [r for r in episodes if r["info"]["is_success"]]
    failed = [r for r in episodes if not r["info"]["is_success"]]
    selected = successful[:(limit+1)//2]
    selected += failed[:limit-len(selected)]
    selected_ids = {r["task_id"] for r in selected}
    selected += [r for r in episodes if r["task_id"] not in selected_ids][:limit-len(selected)]
    return selected


def rollout_figures(corpus, evaluation, output):
    import matplotlib.pyplot as plt
    rows = {r["task_id"]: r for r in corpus.rows}
    episodes = representative_episodes(evaluation["episodes"])
    fig, axes = plt.subplots(2, 3, figsize=(14, 9), constrained_layout=True)
    for ax, episode in zip(axes.flat, episodes):
        problem = corpus.problem(rows[episode["task_id"]])
        draw_problem(ax, problem)
        trace = episode.get("finalization", {}).get("timed_trace", [])
        success = episode["info"]["is_success"]
        if success and not trace:
            raise ValueError("model_plot_requires_full_timed_traces_from_coptrans_rl.evaluate")
        if success and trace:
            ax.plot([r[1] for r in trace], [r[2] for r in trace], color="#00a8e8", lw=2)
        else:
            points, _ = expand(problem.start, [corpus.settings.library[i] for i in episode["action_ids"]], .05, math.radians(5))
            ax.plot([p.x for p in points], [p.y for p in points], "--", color="#ef6c00", lw=1.5)
        label = f'PASS T={episode["info"]["execution_seconds"]:.2f}s' if success else "FAIL "+episode["info"]["reason"]
        ax.set_title(episode["family"]+" / "+episode["task_id"]+"\n"+label, fontsize=9)
    for ax in list(axes.flat)[len(episodes):]:
        ax.axis("off")
    fig.suptitle("Actual model rollouts: solid = certified trajectory; dashed = failed raw attempt")
    fig.savefig(output/"model_rollouts.png", dpi=140); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--run", type=Path)
    parser.add_argument("--evaluation", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    output = artifact_directory(args.output)
    os.environ.setdefault("MPLCONFIGDIR", str(output/"matplotlib-cache"))
    import matplotlib
    matplotlib.use("Agg")
    corpus = Corpus(args.dataset)
    dataset_figures(corpus, output)
    if args.run:
        learning_curves(args.run, output)
    if args.evaluation:
        rollout_figures(corpus, json.loads(args.evaluation.read_text()), output)


if __name__ == "__main__":
    main()
