"""Compute a necessary yaw cross-section of the first street elbow."""
import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from shapely.affinity import rotate, translate
from shapely.geometry import Polygon

from plot import ROOT, FAIL, TEAL, font, outline
from scenes import elbow_slice, study_scene


def last_feasible(scene, angle):
    lower, upper = 0., .20
    positions = elbow_slice(scene, angle, lower)
    if positions.is_empty:
        return None
    while not elbow_slice(scene, angle, upper).is_empty:
        upper *= 2
    while upper-lower > .00005:
        middle = (lower+upper)/2
        region = elbow_slice(scene, angle, middle)
        if region.is_empty:
            upper = middle
        else:
            lower, positions = middle, region
    point = positions.representative_point()
    return dict(clearance_bracket_m=[lower, upper], center=[point.x, point.y])


def main(directory):
    directory.mkdir(parents=True, exist_ok=True)
    font()
    fig, axes = plt.subplots(1, 2, figsize=(9.7, 5.2), layout="constrained")
    result = {}
    for ax, width in zip(axes, (6.10,6.00)):
        _, scene, _, settings = study_scene(ROOT/"ros2_ws/src","street",width)
        threshold = settings["required_clearance_m"]
        areas = {str(a):elbow_slice(scene,a,threshold-.0001).area for a in (45,-45)}
        best = last_feasible(scene, 45)
        road = scene.blade_site.road
        outline(ax,road,(34,48.5),(17.7,32.5))
        blade = Polygon(scene.cargo.outline)
        placed = translate(rotate(blade,45,origin=(0,0)), *best["center"])
        clr = best["clearance_bracket_m"]
        color = TEAL if clr[0] >= threshold else FAIL
        ax.fill(*placed.exterior.xy,fc=color,ec=color,alpha=.36,zorder=3)
        cx, cy = best["center"]
        ax.scatter(cx,cy,s=22,c=color,zorder=4)
        ax.annotate("货物中心", (cx,cy), xytext=(-27,20), textcoords="offset points",fontsize=9)
        for x,y in scene.offsets:
            loc = [cx+(x-y)/math.sqrt(2),cy+(x+y)/math.sqrt(2)]
            ax.scatter(*loc,marker="o",s=17,c="#16374c",zorder=4)
        ax.set_title(f"首道口净宽 {width:.2f} m",loc="left")
        ax.set_xlabel(f"x (m)\n45° 最佳净距约 {100*np.mean(clr):.2f} cm  /  要求 6 cm",color=color)
        result[f"{width:.2f}"] = dict(required_clearance_m=threshold,relaxed_margin_m=threshold-.0001,
                    center_domain_y_max_m=36.,feasible_xy_area_at_relaxed_margin_m2=areas,
                    best_at_positive_45=best)
    fig.suptitle("弓形首弯  /  叶片转至 45° 时，路缘还剩多少空间？",fontsize=15)
    fig.savefig(directory/"street_width_boundary.png",dpi=165,bbox_inches="tight")
    plt.close(fig)
    (directory/"street_width_boundary.json").write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n")


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory",type=Path)
    main(parser.parse_args().directory)
