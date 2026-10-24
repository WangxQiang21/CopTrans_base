#!/usr/bin/env python3
"""Generate the retained N06 narrow-passage test map for expv3."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import binary_erosion, label


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "maps"
MAP_GEN = ROOT / "catkin_ws/src/fwis_map_server/scripts/gen_map.py"
SIZE = [12.0, 8.0]
RESOLUTION = 0.02
MAP_DIRS = {
    "N06_two_stage_offset_passage": "map15-n06-two-stage-offset-passage",
}


def rect(x0, y0, x1, y1):
    return {"type": "rect", "x": [x0, x1], "y": [y0, y1]}


SCENES = [
    {
        "name": "N06_two_stage_offset_passage",
        "title": "N06 Two-stage offset passage",
        "description": "前段窄直廊接近 90° 转向后进入第二窄廊，测试通道转换与目标航向精度。",
        "start": {"x": 1.0, "y": 2.70, "theta": 0.0},
        "goal": {"x": 10.95, "y": 6.85, "theta": 1.5708},
        "walls": [rect(0.08, 2.10, 5.30, 2.24), rect(0.08, 3.20, 3.30, 3.34),
                  rect(3.30, 3.20, 3.44, 5.20), rect(5.30, 4.40, 11.92, 4.54),
                  rect(5.30, 5.53, 10.25, 5.67), rect(5.30, 5.67, 5.44, 7.92),
                  rect(7.60, 0.08, 8.80, 1.35)],
    },
]


def load_map_generator():
    spec = importlib.util.spec_from_file_location("exp_gen_map", MAP_GEN)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.gen


def check_disk_clearance(scene, image):
    """Conservative 2-D connectivity check using a 0.45 m vehicle disk."""
    free = np.flipud(image) > 200
    radius_px = int(np.ceil(0.45 / RESOLUTION))
    yy, xx = np.mgrid[-radius_px:radius_px + 1, -radius_px:radius_px + 1]
    footprint = xx**2 + yy**2 <= (0.45 / RESOLUTION)**2
    safe = binary_erosion(free, structure=footprint, border_value=0)
    components, _ = label(safe, structure=np.ones((3, 3), dtype=np.uint8))

    def component(pose):
        ix = int(pose["x"] / RESOLUTION)
        iy = int(pose["y"] / RESOLUTION)
        return int(components[iy, ix])

    start_component = component(scene["start"])
    goal_component = component(scene["goal"])
    if start_component == 0 or start_component != goal_component:
        raise ValueError(f"{scene['name']}: 0.45 m disk-clearance map is disconnected")


def main():
    gen_map = load_map_generator()
    previews = []
    for scene in SCENES:
        payload = {
            "name": scene["name"], "map_id": MAP_DIRS[scene["name"]],
            "size_m": SIZE, "resolution": RESOLUTION,
            "origin": [0.0, 0.0, 0.0], "border": True,
            "border_thickness": 0.08, "walls": scene["walls"],
            "start": scene["start"], "goal": scene["goal"],
            "description": scene["description"],
            "test_point_source": "project_defined_synthetic_test_case",
        }
        source = OUT / MAP_DIRS[scene["name"]] / "scene_source.yaml"
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True), encoding="utf-8")
        gen_map(str(source), str(OUT))
        map_image = Image.open(source.parent / "map.pgm").convert("L")
        map_image.save(source.parent / "preview.png")
        image = np.asarray(map_image)
        check_disk_clearance(scene, image)
        previews.append((scene, image))

    fig, ax = plt.subplots(1, 1, figsize=(8, 5.5), constrained_layout=True)
    scene, image = previews[0]
    ax.imshow(image, cmap="gray", origin="upper", extent=[0, SIZE[0], 0, SIZE[1]], vmin=0, vmax=255)
    s, g = scene["start"], scene["goal"]
    ax.scatter([s["x"]], [s["y"]], color="#1677ff", s=36, zorder=4)
    ax.scatter([g["x"]], [g["y"]], color="#ef4444", s=36, zorder=4)
    for pose, color in ((s, "#1677ff"), (g, "#ef4444")):
        ax.quiver(pose["x"], pose["y"], np.cos(pose["theta"]), np.sin(pose["theta"]),
                  color=color, angles="xy", scale_units="xy", scale=1.7,
                  width=0.006, headwidth=4.5, zorder=4)
    ax.annotate("START", (s["x"], s["y"]), xytext=(5, 7), textcoords="offset points",
                color="#1677ff", fontsize=9, fontweight="bold")
    ax.annotate("GOAL", (g["x"], g["y"]), xytext=(5, -14), textcoords="offset points",
                color="#ef4444", fontsize=9, fontweight="bold")
    ax.set_title(scene["title"], fontsize=12)
    ax.set_xlim(0, SIZE[0]); ax.set_ylim(0, SIZE[1]); ax.set_aspect("equal")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)"); ax.grid(alpha=0.16)
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / "narrow_maps_contact_sheet.png", dpi=170)
    plt.close(fig)
    print(f"Wrote retained map15 and {OUT / 'narrow_maps_contact_sheet.png'}")


if __name__ == "__main__":
    main()
