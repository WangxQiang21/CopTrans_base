#!/usr/bin/env python3
"""Export reproducible plots from the saved comparison, without rerunning Gazebo."""
import argparse
import csv
import gzip
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


COLORS = {"nonlinear": "#087d80", "pid": "#bd561f"}
LABELS = {"nonlinear": "Nonlinear", "pid": "PID"}


def build_ecdf(directory, trials):
    sources = tuple(dict.fromkeys(trial["source"] for trial in trials if trial["outcome"] == "complete"))
    if not sources:
        raise ValueError("no completed trials to plot")
    fig, axes = plt.subplots(len(sources), 2, squeeze=False,
                             figsize=(10.1, 3.4*len(sources)), constrained_layout=True)
    for source, row in zip(sources, axes):
        for axis, (field, scale, label) in zip(row, (
                ("position_m_rmse", 100., "Position RMSE (cm)"),
                ("yaw_rad_rmse", 180. / math.pi, "Heading RMSE (deg)"))):
            for condition, style in (("nominal", "--"), ("perturbed", "-")):
                for algorithm, color in COLORS.items():
                    values = sorted(float(trial[field])*scale for trial in trials
                                    if trial["algorithm"] == algorithm and trial["source"] == source
                                    and trial["condition"] == condition and trial["outcome"] == "complete")
                    if not values:
                        continue
                    axis.step([0., *values], [i/len(values) for i in range(len(values)+1)],
                              where="post", ls=style, color=color,
                              label=f"{LABELS[algorithm]} · {condition}", lw=1.65)
            axis.set_title(f"{'CopTrans' if source == 'coptrans' else 'Synthetic'} trajectories",
                           loc="left", fontweight="bold", fontsize=10)
            axis.set_xlabel(label)
            axis.set_ylabel("Fraction of trials")
            axis.set_ylim(0., 1.02)
            axis.grid(alpha=.23)
    axes[0, 0].legend(loc="lower right", fontsize=8, frameon=False)
    fig.suptitle("Ranger Mini 2.0 body-Twist tracking in Gazebo", x=.02, ha="left",
                 fontweight="bold", fontsize=15)
    fig.savefig(directory / "comparison_errors.png", dpi=200)
    plt.close(fig)


def mixed_trace(directory, case, repetition):
    traces = {}
    for algorithm in COLORS:
        path = directory / f"runs/{case}__{algorithm}__perturbed__{repetition:02d}.csv.gz"
        if not path.is_file():
            print(f"Skipping {case} trace. Expected {path.name}")
            return
        with gzip.open(path, "rt", newline="") as file:
            traces[algorithm] = list(csv.DictReader(file))
    ref = [r for r in traces["nonlinear"] if r["ref_x"]]
    fig = plt.figure(figsize=(10.1, 5.8), layout="constrained")
    grid = fig.add_gridspec(3, 2, width_ratios=(.9, 1.38))
    path_axis = fig.add_subplot(grid[:, 0])
    pos = fig.add_subplot(grid[0, 1])
    yaw = fig.add_subplot(grid[1, 1], sharex=pos)
    mode = fig.add_subplot(grid[2, 1], sharex=pos)
    path_axis.plot([float(row["ref_x"]) for row in ref], [float(row["ref_y"]) for row in ref],
                   "--", color="#4b5358", lw=2, label="Reference", zorder=4)
    for algorithm, measurements in traces.items():
        points = [row for row in measurements if row["ref_x"]]
        time = [float(row["time_s"]) for row in points]
        color = COLORS[algorithm]
        path_axis.plot([float(row["x"]) for row in points], [float(row["y"]) for row in points],
                       color=color, lw=1.45, label=LABELS[algorithm])
        pos.plot(time, [float(row["position_error"])*100. for row in points], color=color, lw=1.45)
        yaw.plot(time, [math.degrees(float(row["heading_error"])) for row in points], color=color, lw=1.4)
        mode.step(time, [int(row["observed_mode"]) for row in points], where="post", color=color, lw=1.4)
    mode.step([float(row["time_s"]) for row in ref], [int(row["planned_mode"]) for row in ref],
              where="post", ls="--", color="#4b5358", lw=1., zorder=3)
    mode.set_yticks([0, 1, 2], ["Ackermann", "Oblique", "Spin"])
    path_axis.set_aspect("equal", adjustable="datalim")
    path_axis.set_xlabel("Map X (m)")
    path_axis.set_ylabel("Map Y (m)")
    path_axis.legend(loc="best", frameon=False)
    pos.set_ylabel("Position error (cm)")
    yaw.set_ylabel("Heading error (deg)")
    mode.set_xlabel("Simulation time (s)")
    for axis in (path_axis, pos, yaw, mode):
        axis.grid(alpha=.23)
    fig.suptitle(f"{case} · paired perturbed run {repetition+1}", x=.02, ha="left",
                 fontweight="bold", fontsize=15)
    fig.savefig(directory / "mixed_trace.png", dpi=200)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("comparison", type=Path)
    parser.add_argument("--case", default="mixed_aosa")
    parser.add_argument("--repetition", type=int, default=0)
    args = parser.parse_args()
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#69747a", "axes.labelcolor": "#26323a"})
    with (args.comparison / "trials.csv").open(newline="") as file:
        trials = list(csv.DictReader(file))
    build_ecdf(args.comparison, trials)
    mixed_trace(args.comparison, args.case, args.repetition)


if __name__ == "__main__":
    main()
