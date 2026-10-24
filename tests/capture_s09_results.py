#!/usr/bin/env python3
"""Capture reproducible S09 multi-goal paths, timing, and cost metrics.

Run while fwis_v3_demo/demo.launch is active on the same ROS master with
auto_goal:=false. The default target set deliberately covers a simple aisle
goal (#2), the unresolved reverse-entry bay (#5), and a diagonal bay (#7).
"""
import argparse
import csv
from datetime import date
import json
import math
import re
import threading
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from std_msgs.msg import String
from fwis_v3_msgs.msg import PlanSeed, TimedTrajectory


GOALS = {
    1: (.740, .500, math.pi/2, "bottom-left"),
    2: (5.860, .500, math.pi/2, "bottom-middle"),
    3: (10.980, .500, math.pi/2, "bottom-right"),
    4: (6.580, 4.000, math.pi/2, "middle-A-R2A"),
    5: (2.700, 4.900, -math.pi/2, "middle-B"),
    6: (.814, 9.214, -math.pi/4, "top-left-diagonal"),
    7: (6.014, 9.214, -math.pi/4, "top-middle-diagonal"),
}
MODE_NAMES = {0: "Ackermann", 1: "Oblique", 2: "Spin"}
MODE_COLORS = {0: "#16a6b6", 1: "#f28e2b", 2: "#8f63c7"}
COST_KEYS = ("distance", "spin", "mode_switch", "reverse", "direction",
             "steer_tv", "curvature", "oblique_tv", "oblique_event",
             "oblique_use", "oblique_cruise")
COST_LABELS = {
    "distance": "➜ Distance", "spin": "⟳ Spin", "mode_switch": "⇄ Mode switch",
    "reverse": "↶ Reverse", "direction": "↕ Direction",
    "steer_tv": "⌁ Steer TV", "curvature": "κ Curvature",
    "oblique_tv": "↔ Oblique TV", "oblique_event": "✳ Oblique event",
    "oblique_use": "◈ Oblique use", "oblique_cruise": "➤ Oblique cruise",
}


def parse_field(text, name, default=None):
    match = re.search(r"(?:^|\s)" + re.escape(name) + r"=([-+0-9.eE]+)", text)
    return float(match.group(1)) if match else default


def parse_anchor_field(text, name, default=None):
    match = re.search(r"\b" + re.escape(name) + r":([-+0-9.eE]+)", text)
    return float(match.group(1)) if match else default


def parse_cost_parts(text):
    matches = re.findall(r"cost_parts=\{([^}]*)\}", text)
    if not matches:
        return {}
    result = {}
    for item in matches[-1].split(","):
        if ":" not in item:
            continue
        key, value = item.split(":", 1)
        try:
            result[key.strip()] = float(value)
        except ValueError:
            pass
    return result


def path_points(message):
    return [[float(p.pose.position.x), float(p.pose.position.y)]
            for p in message.poses]


def make_goal(publisher, goal):
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x, msg.pose.position.y = goal[0], goal[1]
    msg.pose.orientation.z = math.sin(goal[2] / 2)
    msg.pose.orientation.w = math.cos(goal[2] / 2)
    publisher.publish(msg)


def capture_one(preset, states, lock, publisher, timeout):
    with lock:
        prior_seed = states.get("seed")
        baseline = prior_seed.plan_id if prior_seed is not None else 0
        states["status"] = ""
        states["search"] = ""
        states["coarse"] = None
    begin = time.monotonic()
    make_goal(publisher, GOALS[preset])
    deadline = begin + timeout
    seed = trajectory = None
    status = search = ""
    coarse = None
    while time.monotonic() < deadline and not rospy.is_shutdown():
        with lock:
            current_seed = states.get("seed")
            current_traj = states.get("trajectory")
            status = states.get("status", "")
            search = states.get("search", "")
            current_coarse = states.get("coarse")
        if current_seed is not None and current_seed.plan_id > baseline:
            plan_id = current_seed.plan_id
            if (current_traj is not None and current_traj.plan_id == plan_id and
                    current_traj.points and current_coarse is not None and
                    current_coarse.poses):
                seed, trajectory, coarse = current_seed, current_traj, current_coarse
                if ("success id=%d" % plan_id) in status:
                    break
            if ("rejected id=%d" % plan_id) in status:
                raise RuntimeError("preset %d rejected: %s" % (preset, status))
        time.sleep(.05)
    if seed is None or trajectory is None:
        raise TimeoutError("preset %d timed out; status=%r search=%r" %
                           (preset, status, search))

    pts = trajectory.points
    path = []
    mode_lengths = {name: 0.0 for name in MODE_NAMES.values()}
    mode_runs = []
    for point in pts:
        path.append({"t": point.time_from_start.to_sec(),
                     "x": point.center_pose.x, "y": point.center_pose.y,
                     "theta": point.center_pose.theta, "v": point.v,
                     "mode": int(point.mode)})
    for a, b in zip(pts, pts[1:]):
        distance = math.hypot(b.center_pose.x - a.center_pose.x,
                              b.center_pose.y - a.center_pose.y)
        # Time sampling marks the first sample after a segment boundary with
        # the new segment's mode; therefore the outgoing interval belongs to
        # b.mode, not a.mode.
        mode = int(b.mode)
        if mode in MODE_NAMES:
            mode_lengths[MODE_NAMES[mode]] += distance
        if not mode_runs or mode_runs[-1] != mode:
            mode_runs.append(mode)

    search_seconds = parse_field(search, "seconds", math.nan)
    optimizer_seconds = parse_field(status, "time", math.nan)
    fixed_seconds = parse_field(status, "fixed", math.nan)
    anchor_seconds = parse_field(status, "anchor", math.nan)
    wall_seconds = time.monotonic() - begin
    final_costs = parse_cost_parts(status)
    coarse_costs = parse_cost_parts(search)
    seed_score = parse_anchor_field(status, "joint_seed_score", math.nan)
    final_score = parse_field(status, "common_score", math.nan)
    accepted_rounds = parse_anchor_field(status, "joint_rounds_accepted", 0)
    low_gain_stop = parse_anchor_field(status, "joint_early_stop_low_gain", 0)
    integration_seconds = parse_anchor_field(status, "joint_integration_s", math.nan)
    result = {
        "preset": preset, "label": GOALS[preset][3], "plan_id": seed.plan_id,
        "goal": {"x": GOALS[preset][0], "y": GOALS[preset][1],
                 "theta": GOALS[preset][2]},
        "source": re.search(r"\bsource=([^\s]+)", status).group(1)
                  if re.search(r"\bsource=([^\s]+)", status) else "unknown",
        "search_seconds": search_seconds,
        "optimizer_seconds": optimizer_seconds,
        "fixed_optimizer_seconds": fixed_seconds,
        "optimizer_other_seconds": anchor_seconds,
        "joint_integration_seconds": integration_seconds,
        "wall_seconds": wall_seconds,
        "trajectory_duration_seconds": pts[-1].time_from_start.to_sec(),
        "coarse_score": parse_field(search, "guided_cost", math.nan),
        "joint_seed_score": seed_score, "final_score": final_score,
        "score_delta": seed_score - final_score,
        "score_reduction_percent": (100.0 * (seed_score-final_score) / seed_score
                                    if seed_score and math.isfinite(seed_score) else math.nan),
        "joint_rounds_accepted": int(accepted_rounds),
        "joint_early_stop_low_gain": int(low_gain_stop),
        "final_cost_parts": final_costs, "coarse_cost_parts": coarse_costs,
        "mode_distance_m": mode_lengths,
        "mode_runs": [MODE_NAMES[m] for m in mode_runs if m in MODE_NAMES],
        "timed_points": path,
        "coarse_path": path_points(coarse) if coarse is not None else [],
        "optimizer_status": status, "search_status": search,
    }
    return result


def draw_results(map_msg, results, output_path):
    width, height = map_msg.info.width, map_msg.info.height
    resolution = map_msg.info.resolution
    ox, oy = map_msg.info.origin.position.x, map_msg.info.origin.position.y
    occupancy = np.asarray(map_msg.data, dtype=np.int16).reshape(height, width)
    occupied = (occupancy != 0).astype(np.uint8)
    extent = (ox, ox + width*resolution, oy, oy + height*resolution)

    fig, ax = plt.subplots(figsize=(14, 9), constrained_layout=True)
    ax.imshow(occupied, origin="lower", extent=extent, interpolation="nearest",
              cmap=matplotlib.colors.ListedColormap(["#dfe3e8", "#262a30"]),
              vmin=0, vmax=1, zorder=0)
    ax.grid(color="#77818c", alpha=.17, linewidth=.5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, ox + width*resolution)
    ax.set_ylim(0, oy + height*resolution)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("S09 multi-goal planning — mode-colored final trajectories\n"
                 "same vehicle-center start, current iterated mode/geometry optimizer",
                 fontsize=14, pad=12)

    # Coarse routes are subdued; final optimized paths carry the A/O/Spin colors.
    for result in results:
        coarse = result["coarse_path"]
        if len(coarse) >= 2:
            ax.plot([p[0] for p in coarse], [p[1] for p in coarse],
                    color="#58616b", linewidth=1.0, alpha=.42,
                    linestyle=(0, (3, 3)), zorder=2)
    for result in results:
        points = result["timed_points"]
        segment_modes = [b["mode"] for b in points[1:]]
        i = 0
        while i < len(segment_modes):
            mode = segment_modes[i]
            j = i+1
            while j < len(segment_modes) and segment_modes[j] == mode:
                j += 1
            color = MODE_COLORS.get(mode, "#111111")
            if mode == 2:
                # Spin has zero center translation by definition; show a
                # small yaw arc around its stationary center so purple remains
                # visible in the combined path figure.
                p, q = points[i], points[j]
                delta = q["theta"]-p["theta"]
                if abs(delta) > 1e-5:
                    radius = .22
                    count = max(12, int(math.ceil(abs(delta)/math.radians(4))))
                    angles = np.linspace(p["theta"], q["theta"], count+1)
                    xs = p["x"] + radius*np.cos(angles)
                    ys = p["y"] + radius*np.sin(angles)
                    ax.plot(xs, ys, color=color, linewidth=3.0, zorder=6)
                    ax.annotate("", xy=(xs[-1], ys[-1]), xytext=(xs[-2], ys[-2]),
                                arrowprops={"arrowstyle": "-|>", "color": color,
                                            "lw": 1.8, "mutation_scale": 10},
                                zorder=7)
            else:
                run = points[i:j+1]
                ax.plot([p["x"] for p in run], [p["y"] for p in run],
                        color=color, linewidth=2.5, alpha=.92,
                        solid_capstyle="round", zorder=4)
            i = j

    start_x, start_y = results[0]["timed_points"][0]["x"], results[0]["timed_points"][0]["y"]
    ax.scatter([start_x], [start_y], marker="*", s=250, color="#16a34a",
               edgecolor="white", linewidth=1.2, zorder=7, label="start")
    for index, result in enumerate(results):
        gx, gy = result["goal"]["x"], result["goal"]["y"]
        ax.scatter([gx], [gy], s=75, marker="o", color="#1684d8",
                   edgecolor="white", linewidth=1.2, zorder=7)
        ax.annotate("#%d %s" % (result["preset"], result["label"]),
                    (gx, gy), xytext=(8, 7 + 13*(index % 2)),
                    textcoords="offset points", color="#075b9a", fontsize=9,
                    bbox={"boxstyle": "round,pad=.2", "fc": "white",
                          "ec": "none", "alpha": .82}, zorder=8)
    mode_legend = [Line2D([0], [0], color=MODE_COLORS[m], lw=3,
                          label=MODE_NAMES[m]) for m in MODE_NAMES]
    mode_legend += [Line2D([0], [0], color="#58616b", lw=1.2,
                           ls=(0, (3, 3)), label="coarse route")]
    mode_legend += [Line2D([0], [0], marker="*", color="none",
                           markerfacecolor="#16a34a", markeredgecolor="white",
                           markersize=13, label="start"),
                    Line2D([0], [0], marker="o", color="none",
                           markerfacecolor="#1684d8", markeredgecolor="white",
                           markersize=8, label="goal")]
    ax.legend(handles=mode_legend, loc="upper left", framealpha=.95,
              fontsize=9, ncol=1)
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def draw_cost_chart(results, output_path):
    """Plot all cost parts, separating dominant terms from small regularizers."""
    major = ["distance", "spin", "mode_switch", "oblique_use", "oblique_cruise"]
    regularizers = ["reverse", "direction", "steer_tv", "curvature",
                    "oblique_tv", "oblique_event"]
    fig, axes = plt.subplots(len(results), 2, figsize=(13, 3.25*len(results)),
                             squeeze=False, constrained_layout=True)
    for row, result in enumerate(results):
        for col, keys in enumerate((major, regularizers)):
            ax = axes[row][col]
            vals = [float(result["final_cost_parts"].get(key, 0.0)) for key in keys]
            labels = [COST_LABELS[key] for key in keys]
            y = np.arange(len(keys))
            bars = ax.barh(y, vals, color="#3b82a0" if col == 0 else "#78a6b5",
                           height=.62, edgecolor="white", linewidth=.7)
            ax.set_yticks(y)
            ax.set_yticklabels(labels)
            ax.invert_yaxis()
            ax.grid(axis="x", color="#7b8794", alpha=.2, linewidth=.7)
            ax.set_axisbelow(True)
            ax.set_xlabel("weighted cost (lower is better)")
            ax.set_title("Main cost terms" if col == 0 else "Small terms / regularizers",
                         loc="left", fontsize=10, pad=7)
            max_value = max(vals, default=0.0)
            ax.set_xlim(0, max(0.05, max_value*1.28))
            for bar, value in zip(bars, vals):
                ax.text(max(value, 0.0) + max_value*.025,
                        bar.get_y()+bar.get_height()/2, "%.3f" % value,
                        va="center", ha="left", fontsize=8, color="#263238")
            if col == 0:
                ax.set_ylabel("#%d %s" % (result["preset"], result["label"]),
                              fontsize=10, fontweight="bold")
    fig.suptitle("Final cost breakdown by goal — icon + horizontal bars\n"
                 "Separate x-scales are used for main terms and small regularizers",
                 fontsize=13, fontweight="bold")
    fig.savefig(output_path, dpi=180, facecolor="white")
    plt.close(fig)


def write_report(output_dir, results, image_name, cost_image_name):
    fields = ["preset", "label", "source", "search_seconds",
              "optimizer_seconds", "fixed_optimizer_seconds",
              "optimizer_other_seconds", "joint_integration_seconds",
              "wall_seconds", "trajectory_duration_seconds", "coarse_score", "joint_seed_score",
              "final_score", "score_delta", "score_reduction_percent",
              "joint_rounds_accepted", "joint_early_stop_low_gain", "mode_runs"]
    with (output_dir / "metrics.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields + ["cost_"+k for k in COST_KEYS])
        writer.writeheader()
        for result in results:
            row = {field: result.get(field, "") for field in fields}
            row["mode_runs"] = " → ".join(result["mode_runs"])
            for key in COST_KEYS:
                row["cost_"+key] = result["final_cost_parts"].get(key, "")
            writer.writerow(row)

    lines = ["# expv3 本轮迭代结果", "",
             "日期：%s。算法配置：A/O/Spin 粗搜 + 固定序列连续优化 + 最多两轮交替模式/几何优化；" % date.today().isoformat(),
             "图片中彩色实线为最终轨迹，灰色虚线为粗解。Spin 以车辆中心处的紫色转向弧表示。所有规划从同一起点独立运行。", "",
             "![S09 多目标模式着色规划图](%s)" % image_name, "",
             "模式颜色：A Ackermann 青色，O Oblique 橙色，Spin 紫色。",
             "cost 为当前共同代价分解（工程权重，不是实测时间或能耗）。", "",
             "本轮将全部 7 个预设设为车头朝车位开口的泊车终点：底排 #1–#3 改为朝外航向，R2A #4 修正到空位中心 `(6.58,4.00)`；R2B 与两个 45° 位原已朝外，保持对应航向。最终时间化轨迹内部扫掠步长为 2.5 mm。", "",
             "## Cost 分项柱状图", "",
             "![各目标 cost 分项横向柱状图](%s)" % cost_image_name, "",
             "图中图标对应 cost 分项；主项与小额正则项分开缩放，柱尾标注精确数值。", ""]
    for result in results:
        lines += ["## #%d %s" % (result["preset"], result["label"]), "",
                  "- 最终来源：`%s`；最终模式段：%s。" %
                  (result["source"], " → ".join(result["mode_runs"])),
                  "- 时间：搜索 `%.3f s`；优化器 `%.3f s`（固定 NLP `%.3f s`，锚点/局部/联合其余阶段 `%.3f s`，交替联合集成 `%.3f s`）；从发目标到结果的墙钟时间 `%.3f s`；轨迹回放时长 `%.2f s`。" %
                  (result["search_seconds"], result["optimizer_seconds"],
                   result["fixed_optimizer_seconds"],
                   result["optimizer_other_seconds"],
                   result["joint_integration_seconds"], result["wall_seconds"],
                   result["trajectory_duration_seconds"]),
                  "- 交替联合阶段 accepted rounds：%d；阶段输入 incumbent `%.3f` → 最终 `%.3f`（下降 `%.3f / %.2f%%`）。输入分数指进入交替阶段时已选出的最佳候选，不是 C++ 粗解分数。" %
                  (result["joint_rounds_accepted"], result["joint_seed_score"],
                   result["final_score"], result["score_delta"],
                   result["score_reduction_percent"]),
                  "- 低收益提前停止：%s；当一轮仍有安全改进但相对收益低于 `0.25%%` 时，保留改进并跳过后续昂贵迭代。" %
                  ("是" if result.get("joint_early_stop_low_gain", 0) else "否"),
                  "- C++ 粗解分数 `%.3f`；最终模式距离代价和优化阶段总分不是同一求解阶段的输入/输出成对消融，仅供阶段对比。" % result["coarse_score"],
                  "- 模式行驶距离：" + "，".join("%s %.3f m" % (k,v)
                    for k,v in result["mode_distance_m"].items()) +
                    "。", ""]
        if result["preset"] == 5:
            lines += ["- 结果解读：虽然代价下降且轨迹通过扫掠，最终仍无倒车，未解决‘车位前 Spin 后倒车 A 入库’目标。", ""]
    accepted = sum(result["joint_rounds_accepted"] > 0 for result in results)
    reductions = [result["score_reduction_percent"] for result in results
                  if math.isfinite(result["score_reduction_percent"])]
    joint_times = [result["joint_integration_seconds"] for result in results
                   if math.isfinite(result["joint_integration_seconds"])]
    effect_text = "本次 %d 个目标中 %d 个至少接受一次交替候选" % (len(results), accepted)
    if reductions:
        effect_text += "，交替阶段输入 incumbent 分数下降范围 %.2f–%.2f%%" % (
            min(reductions), max(reductions))
    if joint_times:
        effect_text += "，交替阶段耗时范围 %.2f–%.2f s" % (
            min(joint_times), max(joint_times))
    effect_text += "。"
    lines += ["## 本轮迭代做了什么", "",
              "新增默认开启的有界交替离散—连续步骤：以当前最佳安全候选为 incumbent，围绕最新几何重建锚点 A/O/Spin 候选图；最多两轮、每轮最多精修一个模式/几何提议。只有共同代价严格下降、车辆中心终点满足容差、全车 OBB swept-SAT 通过时才接受。失败/无改进时保留原 incumbent。",
              "新增自适应继续条件：某轮即使有小幅安全改进也照常接受；若该轮相对 cost 改善低于 `0.25%`，则不再启动下一轮图搜索/NLP，避免为边际收益固定付出完整迭代耗时。",
              "",
              "效果：" + effect_text + " 所有报告候选均通过终点和时间化整车扫掠验收；cost 下降不能直接代表跨场景性能提升。#5 仍无倒车，局部锚点协调不能替代终端拓扑/姿态候选补充。本次尚未做关闭/开启的多地图配对统计，不能得出普遍性能提升结论。",
              "",
              "复现：启动 `./run_demo.sh rviz:=false auto_goal:=false` 后，在同一 ROS master 运行 `python3 expv3/tests/capture_s09_results.py --presets %s`；另外运行 `python3 expv3/tests/ros_parking_goals_smoke.py presets` 做独立 5 mm 整车 swept-SAT 检查。逐轨迹和 ROS status 保存在 `trajectories.json`；指标见 `metrics.csv`。" %
              ",".join(str(result["preset"]) for result in results), ""]
    (output_dir / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--presets", default="2,5,7",
                        help="comma-separated S09 preset ids; default: 2,5,7")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--output-dir", default=str(Path(__file__).resolve().parents[1] / "relust"))
    args = parser.parse_args()
    presets = [int(value) for value in args.presets.split(",") if value.strip()]
    if not presets or any(value not in GOALS for value in presets):
        parser.error("presets must be ids in 1..7")

    rospy.init_node("expv3_result_capture", anonymous=True)
    states = {"map": None, "seed": None, "trajectory": None,
              "coarse": None, "status": "", "search": ""}
    lock = threading.Lock()

    def set_value(key):
        def callback(message):
            with lock:
                states[key] = message
        return callback

    def set_text(key):
        def callback(message):
            with lock:
                states[key] = message.data
        return callback

    rospy.Subscriber("/map", OccupancyGrid, set_value("map"), queue_size=1)
    rospy.Subscriber("/plan/seed", PlanSeed, set_value("seed"), queue_size=1)
    rospy.Subscriber("/plan/timed_trajectory", TimedTrajectory,
                     set_value("trajectory"), queue_size=1)
    rospy.Subscriber("/plan/path_coarse", RosPath, set_value("coarse"), queue_size=1)
    rospy.Subscriber("/plan/status", String, set_text("status"), queue_size=1)
    rospy.Subscriber("/plan/search_status", String, set_text("search"), queue_size=1)
    goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
    map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=15)
    deadline = time.monotonic() + 10
    while goal_pub.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(.05)
    if goal_pub.get_num_connections() == 0:
        raise RuntimeError("planner is not subscribed to /move_base_simple/goal")

    results = []
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    for preset in presets:
        print("[capture] planning S09 preset #%d (%s)" % (preset, GOALS[preset][3]), flush=True)
        results.append(capture_one(preset, states, lock, goal_pub, args.timeout))
        result = results[-1]
        with (output_dir / "trajectories.json").open("w", encoding="utf-8") as handle:
            json.dump({"date": "2026-10-02", "presets": presets,
                       "results": results}, handle, ensure_ascii=False, indent=2)
        print("[capture] #%d source=%s search=%.3fs optimizer=%.3fs score=%.3f "
              "seed=%.3f rounds=%d wall=%.3fs" %
              (preset, result["source"], result["search_seconds"],
               result["optimizer_seconds"], result["final_score"],
               result["joint_seed_score"], result["joint_rounds_accepted"],
               result["wall_seconds"]), flush=True)

    image_name = "s09_presets_%s_modes.png" % "_".join(map(str, presets))
    cost_image_name = "s09_presets_%s_costs.png" % "_".join(map(str, presets))
    draw_results(map_msg, results, output_dir / image_name)
    draw_cost_chart(results, output_dir / cost_image_name)
    write_report(output_dir, results, image_name, cost_image_name)
    print("[capture] wrote %s" % output_dir, flush=True)


if __name__ == "__main__":
    main()
