"""Matched full-route and first-failure views of the archived bend comparison."""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/ranger_mini_gz"))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np
import shapely as sg

from scenes import blade_span
from ranger_mini_gz.blade_certify import BladeCertifier
from ranger_mini_gz.scene_generator import load_scene


NAVY, TEAL, PURPLE, ORANGE, PINK, FAIL = "#194b68", "#007966", "#79459d", "#d57416", "#ba4472", "#bb263d"
BLADE, HULL, EDGE = "#c5deee", "#316382", "#34495c"


def font():
    candidates = [Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
                  Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf")]
    chosen = next((p for p in candidates if p.is_file()), None)
    if chosen:
        fontManager.addfont(str(chosen))
        plt.rcParams["font.family"] = FontProperties(fname=chosen).get_name()
    plt.rcParams.update({"font.size": 10, "axes.unicode_minus": False,
                         "axes.spines.top": False, "axes.spines.right": False, "axes.linewidth": .7})


def outline(ax, road, xlim, ylim):
    ax.set_facecolor("#e1e5e8")
    for poly in sg.get_parts(road):
        ax.fill(*poly.exterior.xy, facecolor="#fbfcfc", edgecolor="#61727b", lw=.95, zorder=0)
        for ring in poly.interiors:
            ax.fill(*ring.xy, facecolor="#e1e5e8", edgecolor="#61727b", lw=.8, zorder=0)
    ax.set(xlim=xlim, ylim=ylim, aspect="equal", xlabel="x (m)", ylabel="y (m)")
    ax.tick_params(colors="#4b5b64", labelsize=8, pad=2)


def read(candidate_file):
    result = json.loads((candidate_file.parent / "results.json").read_text())[candidate_file.stem]
    settings = json.loads((candidate_file.parent / "settings.json").read_text())
    with np.load(candidate_file, allow_pickle=False) as f:
        return settings, result, {k: f[k] for k in ("state", "mode", "gear")}


def sparse_selection(state, relevant, count):
    ids = np.flatnonzero(relevant)
    if not len(ids):
        return ids
    data = state[ids]
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(data[:, :2], axis=0), axis=1)
                                    + 2 * abs(np.diff(data[:, 2])))]
    return ids[np.unique(np.searchsorted(distance, np.linspace(0., distance[-1], count)))]


def add_envelopes(ax, checker, state, indices, active=False):
    if not len(indices):
        return
    footprints = checker.footprints(state[indices, :3], state[indices, 3:])
    for col, color in enumerate((HULL, "#375356", "#916626")):
        polys = [np.asarray(poly.exterior.coords) for poly in footprints[:, col]]
        ax.add_collection(PolyCollection(polys, facecolor=(BLADE if not col else color),
                                         edgecolor=color, alpha=(.65 if active else .29) if not col else .62,
                                         linewidth=(1.35 if active else .78), zorder=3 + col))


def pin_shapes(ax, checker, state, i, color):
    center = checker.mount_centers(state[i:i+1, :3])[0]
    for j, (px, py) in enumerate(center):
        yaw = state[i, 3 + j]
        ax.plot(px, py, marker="o", color=color, ms=3, zorder=9)
        ax.arrow(px, py, .65 * math.cos(yaw), .65 * math.sin(yaw), head_width=.15,
                 length_includes_head=True, color=color, lw=1.2, zorder=9)


def panel(ax, checker, data, report, xlim, ylim, title, margin):
    state, mode, gear = data["state"], data["mode"], data["gear"]
    road = checker.scene.blade_site.road
    outline(ax, road, xlim, ylim)
    ax.set_title(title, loc="left", fontsize=13, color="#162f40", pad=7)
    local = ((state[:, 0] >= xlim[0]) & (state[:, 0] <= xlim[1])
             & (state[:, 1] >= ylim[0]) & (state[:, 1] <= ylim[1]))
    near = (state[:, 0] >= xlim[0]-2) & (state[:, 0] <= xlim[1]+2)
    near &= (state[:, 1] >= ylim[0]-2) & (state[:, 1] <= ylim[1]+2)
    certificate = report["resolutions"]["0.05"]
    first = certificate.get("first_failure", {})
    bad = first.get("sample", first.get("segment", report["local_closest"]["index"]))
    point = report["local_closest"]
    limit = len(state) if report["certified"] else min(len(state), int(bad) + 1)
    ix = np.arange(len(state))
    show = sparse_selection(state, local & (ix < limit), 6)
    if report["certified"] and local[point["index"]]:
        show = np.unique([*show, point["index"]])
    add_envelopes(ax, checker, state, show)
    if not report["certified"]:
        ax.plot(*np.where(near[:, None], state[:, :2], np.nan).T, ls=":", color="#9aa6aa", lw=1.2)
    selected = near[:-1] & (ix[:-1] < limit)
    typ = mode.copy()
    typ[(mode == 0) & np.any(gear < 0, axis=1)] = 3
    colors = [TEAL, ORANGE, PINK, PURPLE]
    center_traces = checker.mount_centers(state[:, :3])
    for car in range(2):
        xy = np.where((near & (ix < limit))[:, None], center_traces[:, car], np.nan)
        ax.plot(*xy.T, lw=.7, color=("#73868c", "#a18465")[car], ls="--", zorder=2)
    segments = np.stack((state[:-1, :2], state[1:, :2]), axis=1)
    for category, color in enumerate(colors):
        ix_mode = selected & (typ == category)
        if ix_mode.any():
            ax.add_collection(LineCollection(segments[ix_mode], colors=color,
                linewidths=2.3 if category else 1.65, zorder=6))
    for i in np.flatnonzero(np.diff(np.r_[False,typ == 3].astype(int)) > 0):
        if selected[i]:
            ax.scatter(*state[i,:2],color=PURPLE,marker="v",s=31,zorder=8)
    events=report["connectors"]
    for num,event in enumerate(events):
        q = np.asarray(event["xy"])
        if not (xlim[0] < q[0] < xlim[1] and ylim[0] < q[1] < ylim[1]):
            continue
        index = (np.linalg.norm(state[:, :2] - q, axis=1)).argmin()
        color = ORANGE if event["mode"] == "Oblique" else PINK
        internal=(num>0 and events[num-1]["mode"]==event["mode"] and
                  np.allclose(events[num-1]["end_xy"],q,atol=1e-5))
        ax.scatter(*q,s=13 if internal else 42,facecolors="white",edgecolor=color,lw=1.4,zorder=9,
                   marker="o" if internal or event["mode"]=="Spin" else "D")
        if not internal:
            pin_shapes(ax, checker, state, index, color)
        end = np.asarray(event["end_xy"])
        if event["mode"] == "Oblique" and np.linalg.norm(end-q)>.65:
            ax.annotate("",q*.37+end*.63,q*.63+end*.37,arrowprops=dict(arrowstyle="->",color=color,lw=1.25),
                        zorder=9)
        if event["mode"]=="Oblique" and num==len(events)//2:
            ax.annotate("保持车头朝向\n斜移",q*.48+end*.52,xytext=(-61,26),
                        textcoords="offset points",fontsize=9,color=color,zorder=11,
                        arrowprops=dict(arrowstyle="-",color=color,lw=.7),
                        bbox=dict(boxstyle="round,pad=.18",fc="white",ec="none",alpha=.9))
        if event["mode"]=="Oblique" and num==len(events)-1 and np.linalg.norm(end-q)>.5:
            ax.scatter(*end,marker="D",s=32,facecolors="white",edgecolors=color,lw=1.5,zorder=9)
    marker = np.asarray(point["wall_xy"])
    if report["certified"]:
        ax.scatter(*marker, s=48, marker="x", color=FAIL, lw=1.6, zorder=10)
        label = f"最近 {point['gap_m']*100:.1f} cm"
        ax.annotate(label, marker, xytext=(10, -24), textcoords="offset points", fontsize=9, color=FAIL,
                    bbox=dict(boxstyle="round,pad=.18", fc="white", ec="none", alpha=.92),
                    arrowprops=dict(arrowstyle="-", color=FAIL, linewidth=.8), zorder=10)
    elif near[int(bad)]:
        add_envelopes(ax, checker, state, [int(bad)], active=True)
        b = int(bad)
        hulls = checker.footprints(state[b:b+1,:3],state[b:b+1,3:])[0]
        body = ("blade","robot1","robot2").index(first.get("body","blade"))
        space = checker._spaces[0 if certificate["reason"].startswith("road") else 1]
        wall = space.index.geometries[space.index.nearest(hulls[body])]
        failure = sg.get_coordinates(sg.shortest_line(hulls[body],wall))[-1]
        ax.scatter(*failure, color=FAIL, marker="X", s=40, zorder=10)
        offset=first.get("clearance_m",first.get("midpoint_clearance_m"))
        witness=report.get("first_rejected_footprint",{})
        along=witness.get("blade_longitudinal_coordinate_m",0.)
        who=("叶尖" if along>6.5 else "叶根" if along<-5.8 else "叶片") if body==0 else f"车 {body}"
        text=f"{who}首次低于净距"+(f"\n{100*offset:.2f} cm" if offset is not None else "")
        ax.annotate(text, failure, xytext=(10,-25), textcoords="offset points",fontsize=9,
                    color=FAIL, bbox=dict(fc="white", ec="none", alpha=.9), zorder=10)
    bound = certificate.get("swept_grid_clearance_lower_bound_m")
    detail = (f"全程扫掠净距 ≥ {100*bound:.2f} cm" if bound is not None and report["certified"]
              else f"候选最近 {100*point['gap_m']:.2f} cm")
    return detail + f"    要求 {margin*100:.0f} cm"


def figure(files, output, caption=None):
    rows = [read(file) for file in files]
    settings = rows[0][0]
    if any(s["start"] != settings["start"] or s["goal"] != settings["goal"] or
           (files[i].parent / "road.geojson").read_bytes() != (files[0].parent / "road.geojson").read_bytes()
           for i,(s, _, _) in enumerate(rows)):
        raise ValueError("two panels must use identical maps and endpoints")
    kind = "street" if "first_elbow_clear_width_m" in settings else "mountain"
    source = ROOT / "ros2_ws/src"
    scene = load_scene(settings["base"], paths=tuple(source / p for p in ("ranger_mini_gz", "fwis_common", "fwis_backends")))
    scene = blade_span(scene, settings["tip_extension_m"])
    road=sg.from_geojson((files[0].parent/"road.geojson").read_text())
    scene=replace(scene,blade_site=replace(scene.blade_site,road=road))
    checker = BladeCertifier(scene)
    xlim, ylim = ((31, 49), (17.3, 34.2)) if kind == "street" else ((49.3, 77.), (40.5, 60.2))
    xlim, ylim = settings.get("plot_bounds", [xlim, ylim])
    fig = plt.figure(figsize=(13.8, 6.7), layout="constrained", facecolor="white")
    grid = fig.add_gridspec(3, 3, height_ratios=[.38, 3.8, .66], width_ratios=[.78, 1, 1])
    header = fig.add_subplot(grid[0, :]); header.axis("off")
    name = "弓形街巷 · 首个直角" if kind == "street" else "山区回程 · 原 S 弯"
    if settings.get("mountain_bend") == "hairpin":
        name = "山区原回头弯"
    spec = (f"局部净宽 {settings['first_elbow_clear_width_m']:.2f} m" if kind == "street" else
            f"局部净宽 {settings.get('width_m', settings.get('s_bend_width_m')):.2f} m")
    if settings.get("geometry") == "tangent_reverse_curves":
        name = "山区回程 · S 弯线形对照"
        spec += f"  /  弯道半径 {settings['centerline_min_radius_m']:g} m"
    if "shelf_length_m" in settings:
        name = "山区回程 · 已撤下的历史方案"
    header.text(.002, .77, name, fontsize=19, color="#173245", va="center")
    header.text(.002, .07, f"{spec}     叶片 {settings['blade_length_m']:.3f} m     挂点间距 {settings['support_spacing_m']:g} m",
                fontsize=11, color="#4e616b")
    overview = fig.add_subplot(grid[1, 0])
    centerline = rows[0][2]["state"][:, :2]
    ox, oy = ((2, 50), (15, 57)) if kind == "street" else ((3, 121), (5, 62))
    outline(overview, scene.blade_site.road, ox, oy)
    overview.plot(*centerline.T, lw=1., ls=":" if not rows[0][1]["certified"] else "-", color=NAVY)
    for pose, label, marker in ((settings["start"], "起点", "o"), (settings["goal"], "终点", "s")):
        overview.scatter(*pose[:2], s=26, marker=marker, zorder=4, color=NAVY)
        overview.annotate(label, pose[:2], xytext=(-4, 8), textcoords="offset points", fontsize=9)
    overview.add_patch(Rectangle((xlim[0], ylim[0]), xlim[1]-xlim[0], ylim[1]-ylim[0],
                                 fc="none", ec="#b6562f", lw=1.2, ls="--"))
    overview.set_title("起终点与局部位置", loc="left", fontsize=12)
    for j, (_, report, data) in enumerate(rows):
        ax = fig.add_subplot(grid[1, j+1])
        pure_label=("纯 Ackermann · 直接转弯候选" if files[j].stem in ("sweep","direct")
                    else "纯 Ackermann · 允许倒车")
        titles = (pure_label, "多模式候选")
        status = "通过" if report["certified"] else "未通过"
        detail = panel(ax, checker, data, report, xlim, ylim, f"{titles[j]} · {status}", settings["required_clearance_m"])
        ax.set_xlabel("x (m)\n"+detail, fontsize=9)
    foot = fig.add_subplot(grid[2, :]);foot.axis("off")
    legend = [Line2D([], [], color=TEAL, lw=2.3, label="Ackermann 前进"),
              Line2D([], [], color=PURPLE, lw=2.3, label="Ackermann 倒车"),
              Line2D([], [], color=ORANGE, marker="D", ms=5, label="Oblique"),
              Line2D([], [], color=PINK, marker="o", ms=5, label="Spin"),
              Line2D([], [], color=HULL, lw=1.1, label="叶片包络"),
              Line2D([], [], color="#78888d", ls="--", label="两个挂点轨迹")]
    foot.legend(handles=legend, loc="upper left", bbox_to_anchor=(0,1), ncol=6, frameon=False, fontsize=10,
                handlelength=1.6, columnspacing=1.45)
    if caption:
        foot.text(.01,.10,caption,fontsize=11, color="#263f4e")
    fig.savefig(output, dpi=165, bbox_inches="tight");plt.close(fig)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("pure", type=Path)
    parser.add_argument("mixed", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--caption")
    args=parser.parse_args();font();figure((args.pure,args.mixed),args.output,args.caption)
