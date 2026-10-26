"""Plot actual certified paths or explicit failed attempts from the new frontend."""
import argparse
import json
import math
import os
from pathlib import Path

import numpy as np

from .dataset import Corpus,artifact_directory
from .benchmark import engineering_tasks
from .configuration import load_settings
from .kernel_adapter import Action
from .training_figures import draw_problem
from fwis_v3_optimizer.core import expand


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evaluation",type=Path,required=True)
    parser.add_argument("--dataset",type=Path)
    parser.add_argument("--run",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args = parser.parse_args()
    output = artifact_directory(args.output)
    os.environ.setdefault("MPLCONFIGDIR",str(output/"matplotlib-cache"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    settings = load_settings()
    problems = {p.task_id:p for p,_ in engineering_tasks(settings.contract)}
    if args.dataset:
        corpus = Corpus(args.dataset)
        wanted = {r["task_id"] for r in json.loads(args.evaluation.read_text())["tasks"]}
        problems.update({r["task_id"]:corpus.problem(r) for r in corpus.rows if r["task_id"] in wanted})
    report = json.loads(args.evaluation.read_text())
    rows = [r for r in report["tasks"] if "is_success" in r]
    if not rows:
        raise ValueError("no_evaluated_tasks_to_plot")
    engineering = all(r["task_id"].startswith("map") for r in rows)
    if not engineering:
        rows = sorted(rows,key=lambda r:(r["is_success"],r["task_id"]))[:6]
    columns = 3
    height = math.ceil(len(rows)/columns)
    fig,axes = plt.subplots(height,columns,figsize=(14,4.4*height),squeeze=False,constrained_layout=True)
    colors = ["#00a8e8","#ff9400","#b83cff"]
    for ax,row in zip(axes.flat,rows):
        problem = problems[row["task_id"]]
        draw_problem(ax,problem)
        if row["is_success"]:
            trace = row["finalization"]["timed_trace"]
            xy = np.asarray([[r[1],r[2]] for r in trace])
            segments = np.stack([xy[:-1],xy[1:]],axis=1)
            ax.add_collection(LineCollection(segments,colors=[colors[int(r[5])] for r in trace[:-1]],linewidths=2))
            label = "PASS T=%.2fs" % row["execution_s"]
        else:
            points,_ = expand(problem.start,[Action(*a) for a in row.get("last_attempt",[])],.04,math.radians(4))
            ax.plot([p.x for p in points],[p.y for p in points],"--",color="#c95000",lw=1.5)
            label = "FAIL "+str(row["reason"])
        ax.set_title(row["task_id"]+"\n"+label,fontsize=9)
    for ax in list(axes.flat)[len(rows):]:
        ax.axis("off")
    fig.suptitle(report["method"]+" / solid: certified; dashed: failed attempt\nA: cyan; O: orange; Spin: purple; start: green; goal: red")
    fig.savefig(output/"paths.png",dpi=150)
    plt.close(fig)
    if args.run:
        history = json.loads((args.run/"history.json").read_text())
        baseline = json.loads((args.run/"rule_validation.json").read_text())["summary"]
        fig,axes = plt.subplots(1,2,figsize=(11,4),constrained_layout=True)
        x = [r["steps"] for r in history]
        axes[0].plot(x,[r["validation"]["success_rate"] for r in history],marker="o",label="trained policy")
        axes[0].axhline(baseline["success_rate"],ls="--",color="black",label="rules only")
        axes[0].set(ylabel="Validation success rate",ylim=(-.02,1.02))
        axes[1].plot(x,[r["validation"]["mean_execution_s"] for r in history],marker="o",label="trained policy")
        if baseline["mean_execution_s"] is not None:
            axes[1].axhline(baseline["mean_execution_s"],ls="--",color="black",label="rules only")
        axes[1].set(ylabel="Mean certified execution time [s]")
        for ax in axes:
            ax.set(xlabel="PPO decisions"); ax.grid(alpha=.25); ax.legend()
        fig.suptitle("GPU integration pilot / fixed 8 validation tasks")
        fig.savefig(output/"training.png",dpi=150)
        plt.close(fig)


if __name__=="__main__":
    main()
