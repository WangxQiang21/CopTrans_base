#!/usr/bin/env python3
"""Digitize approximate ROS occupancy maps from selected paper plot panels.

The papers provide rasterized plots rather than downloadable occupancy grids.
This script isolates the obstacle fill color inside each plot's calibrated
axis rectangle, then writes ROS map_server assets and per-map provenance.
"""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter, ImageOps, ImageDraw
from scipy.ndimage import label


ROOT = Path(__file__).resolve().parents[1]
PAPERS = ROOT / "papers"
OUT = ROOT / "maps"
CATALOG = OUT / "paper_digitized"
DPI = 360
REFERENCE_DPI = 180
ROBOT_WIDTH_M = 0.500
PAPER_VEHICLE_WIDTH_M = {
    "11_Chang_4WIS_Parking_Mode_Decision_2024.pdf": 1.675,
    "12_Bao_4WIS_Executable_Trajectories_2026.pdf": 0.620,
    "16_Zhu_Adaptive_Multi_Mode_4WIS_2026.pdf": 2.000,
}
MAP_DIRS = {
    "P11_case_a": "map01-p11-narrow-case-a",
    "P11_case_b": "map02-p11-goal-side-aisle",
    "P11_case_c": "map03-p11-narrow-case-c",
    "P12_env1": "map04-p12-maze-env1",
    "P12_env2": "map05-p12-parking-row-env2",
    "P12_env3": "map06-p12-bent-corridor-env3",
    "P12_env4": "map07-p12-obstacle-field-env4",
    "P16_complex_parking": "map08-p16-complex-parking",
    "P16_narrow_maze": "map09-p16-narrow-maze",
}

# plot_box is (left, top, right, bottom) in the 180-dpi PDF page render;
# bounds are the approximate world-coordinate limits read from plot ticks.
MAPS = [
    dict(name="P11_case_a", paper="11_Chang_4WIS_Parking_Mode_Decision_2024.pdf",
         page=7, plot_box=(192, 158, 622, 384), bounds=(-3.0, 20.0, 0.0, 12.0),
         fill="gray125", figure="Fig. 5(a)",
         caption="Narrow scene A; HA result shown in source figure.",
         min_component_area=250,
         boundary_median=5, boundary_close=7,
         obstacle_rectangles=[(114, 356, 337, 442), (558, 785, 342, 442)],
         cleanup_notes="Removed isolated raster traces smaller than 250 pixels and restored the two source-figure rectangular obstacle faces."),
    dict(name="P11_case_b", paper="11_Chang_4WIS_Parking_Mode_Decision_2024.pdf",
         page=7, plot_box=(663, 158, 914, 384), bounds=(0.0, 10.0, 0.0, 12.0),
         fill="gray125", figure="Fig. 5(b)",
         caption="Narrow scene B; HA result shown in source figure.", min_component_area=30,
         # Clear the small inward teeth and open the goal-side aisle. The
         # straight inner faces leave about 0.79 m between obstacles.
         cleanup_notes="Goal-side aisle widened and both inner faces straightened to about 0.79 m.",
         free_rectangles=[(186, 206, 255, 511), (280, 300, 255, 511)]),
    dict(name="P11_case_c", paper="11_Chang_4WIS_Parking_Mode_Decision_2024.pdf",
         page=7, plot_box=(973, 159, 1385, 384), bounds=(-3.0, 17.0, -2.5, 9.0),
         fill="gray125", figure="Fig. 5(c)",
         caption="Narrow scene C; HA result shown in source figure.", min_component_area=250,
         cleanup_notes="Removed isolated raster traces smaller than 250 pixels.",
         ignore_world_boxes=[(12.2, 16.0, 2.5, 4.8)]),
    dict(name="P12_env1", paper="12_Bao_4WIS_Executable_Trajectories_2026.pdf",
         page=9, plot_box=(226, 187, 463, 427), bounds=(0.0, 20.0, 0.0, 20.0),
         fill="gray166", figure="Fig. 5, Env1", caption="Four-method maze benchmark.",
         min_component_area=75, boundary_median=5, boundary_close=7,
         # This panel consists of axis-aligned walls. Reconstruct their faces
         # from the extracted bounds: a plotted trace had left a false notch
         # in the upper-right wall, and resizing left one-cell edge teeth.
         obstacle_rectangles_only=[
             (24, 450, 0, 23), (0, 23, 23, 444),
             (451, 473, 23, 444), (24, 450, 444, 467),
             (71, 403, 71, 94), (71, 95, 71, 397),
             (379, 403, 71, 397), (143, 403, 375, 397),
             (143, 189, 140, 327), (285, 331, 140, 187),
             (237, 284, 187, 234), (237, 331, 280, 327)],
         output_upsample=3,
         cleanup_notes="Reconstructed axis-aligned maze walls and obstacle faces; removed raster teeth and a false wall notch; exported at 3x grid resolution."),
    dict(name="P12_env2", paper="12_Bao_4WIS_Executable_Trajectories_2026.pdf",
         page=9, plot_box=(220, 450, 465, 613), bounds=(0.0, 15.0, 0.0, 10.0),
         fill="gray166", figure="Fig. 5, Env2", caption="Four-method parking-row benchmark.", min_component_area=30,
         # Parking-row obstacles are rectangles. Printed trajectories erased
         # parts of two teeth and left small bumps along the bottom wall.
         # Use their intact face coordinates, before any runtime map scaling.
         obstacle_rectangles_only=[
             (2, 489, 3, 15), (2, 16, 15, 325),
             (475, 489, 15, 325), (2, 489, 312, 325),
             (2, 376, 148, 163),
             (82, 98, 99, 148), (164, 180, 99, 148),
             (245, 261, 99, 148), (327, 376, 99, 148),
             (66, 81, 246, 312), (132, 147, 246, 312),
             (197, 213, 246, 312), (262, 277, 246, 312),
             (327, 343, 246, 312), (393, 409, 246, 312),
             (459, 475, 246, 312)],
         cleanup_notes="Reconstructed straight parking-row walls and teeth from intact rectangular faces; removed raster bumps and restored two trace-eroded teeth; original grid size and metric extent retained."),
    dict(name="P12_env3", paper="12_Bao_4WIS_Executable_Trajectories_2026.pdf",
         page=9, plot_box=(220, 638, 465, 802), bounds=(0.0, 9.0, 0.0, 5.0),
         fill="gray166", figure="Fig. 5, Env3", caption="Four-method bent-corridor benchmark.", min_component_area=30,
         # Straighten the rasterized vertical wall edge at the upper bend.
         cleanup_notes="Filled the upper-bend wall edge to remove raster teeth.",
         obstacle_rectangles=[(388, 408, 0, 122)]),
    dict(name="P12_env4", paper="12_Bao_4WIS_Executable_Trajectories_2026.pdf",
         page=9, plot_box=(220, 819, 465, 986), bounds=(3.0, 16.5, 0.0, 9.0),
         fill="gray166", figure="Fig. 5, Env4", caption="Four-method obstacle-field benchmark.",
         min_component_area=100, boundary_median=5, boundary_close=7,
         rectangularize_min_area=500, rectangularize_fill_ratio=.90,
         output_upsample=3,
         cleanup_notes="Removed small raster traces and reconstructed rectangular obstacle faces; exported at 3x grid resolution."),
    dict(name="P16_complex_parking", paper="16_Zhu_Adaptive_Multi_Mode_4WIS_2026.pdf",
         page=24, plot_box=(477, 505, 944, 973), bounds=(0.0, 30.0, 0.0, 30.0),
         fill="gray89", figure="Figure 22", caption="Complex parking benchmark.",
         map_scale_factor=0.1741,
         boundary_median=5,
         cleanup_notes="Applied a 5-pixel median filter to smooth digitized obstacle edges and remove small raster teeth.",
         scale_note="uniform scale makes the source 5.74 m vertical passage approximately 1.00 m (2x this robot width)"),
    dict(name="P16_narrow_maze", paper="16_Zhu_Adaptive_Multi_Mode_4WIS_2026.pdf",
         page=22, plot_box=(479, 534, 947, 1000), bounds=(0.0, 30.0, 0.0, 30.0),
         fill="gray89", figure="Figure 19", caption="Narrow-maze benchmark.",
         boundary_median=5,
         cleanup_notes="Applied a 5-pixel median filter to smooth digitized wall edges while preserving the corridor layout."),
]

def render_page(pdf: Path, page: int, tmp: Path) -> Image.Image:
    prefix = tmp / f"{pdf.stem}_{page}"
    subprocess.run([
        "pdftoppm", "-f", str(page), "-l", str(page), "-r", str(DPI),
        "-png", "-singlefile", str(pdf), str(prefix),
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return Image.open(prefix.with_suffix(".png")).convert("RGB")


def obstacle_mask(plot: Image.Image, kind: str, min_component_area: int = 30) -> Image.Image:
    """Extract solid obstacle fill; deliberately ignore paths/vehicle traces."""
    mask = Image.new("L", plot.size, 0)
    src = plot.load()
    dst = mask.load()
    if kind == "gray125":
        low, high = 116, 135
    elif kind == "gray166":
        low, high = 157, 174
    elif kind == "gray89":
        low, high = 83, 96
    else:
        raise ValueError(f"unknown fill class {kind}")
    for y in range(plot.height):
        for x in range(plot.width):
            r, g, b = src[x, y]
            if low <= r <= high and abs(r-g) <= 2 and abs(r-b) <= 2:
                dst[x, y] = 255
    # Remove thin plotted trajectories/vehicle outlines without reshaping the
    # filled obstacles. Metric scale adjustment happens only through map.yaml.
    mask = (mask.filter(ImageFilter.MinFilter(3))
                .filter(ImageFilter.MaxFilter(3))
                .filter(ImageFilter.MedianFilter(3)))
    components, _ = label(np.asarray(mask, dtype=np.uint8) > 0)
    sizes = np.bincount(components.ravel())
    small = sizes < min_component_area
    small[0] = False
    cleaned = (np.asarray(mask, dtype=np.uint8) > 0) & ~small[components]
    return Image.fromarray(cleaned.astype(np.uint8) * 255, mode="L")


def write_map(spec: dict, page: Image.Image) -> Image.Image:
    scale = DPI / REFERENCE_DPI
    left, top, right, bottom = (round(value*scale) for value in spec["plot_box"])
    plot = page.crop((left, top, right, bottom))
    mask = obstacle_mask(plot, spec["fill"],
                         int(spec.get("min_component_area", 30)))
    xmin, xmax, ymin, ymax = spec["bounds"]
    span_x, span_y = xmax-xmin, ymax-ymin
    # Some source panels draw a dashed target bay/pose envelope in the same
    # gray as the map walls. Explicitly omit that non-obstacle overlay.
    if spec.get("ignore_world_boxes"):
        draw=ImageDraw.Draw(mask)
        for bx0,bx1,by0,by1 in spec["ignore_world_boxes"]:
            x0=(bx0-xmin)/span_x*mask.width
            x1=(bx1-xmin)/span_x*mask.width
            y0=(ymax-by1)/span_y*mask.height
            y1=(ymax-by0)/span_y*mask.height
            draw.rectangle((x0,y0,x1,y1),fill=0)
    # Equal metric resolution is important: never stretch x and y differently.
    source_res = (span_x/mask.width + span_y/mask.height) / 2.0
    width, height = max(1, round(span_x/source_res)), max(1, round(span_y/source_res))
    mask = mask.resize((width, height), Image.Resampling.NEAREST)
    median_size=int(spec.get("boundary_median",3))
    close_size=int(spec.get("boundary_close",0))
    if median_size>=3 and median_size%2==1:
        mask=mask.filter(ImageFilter.MedianFilter(median_size))
    if close_size>=3 and close_size%2==1:
        mask=mask.filter(ImageFilter.MaxFilter(close_size))
        mask=mask.filter(ImageFilter.MinFilter(close_size))
    if "obstacle_rectangles_only" in spec:
        mask=Image.new("L",mask.size,0)
    elif "rectangularize_min_area" in spec:
        pixels=np.asarray(mask,dtype=np.uint8)>0
        components,count=label(pixels)
        boxes=[]
        min_area=int(spec["rectangularize_min_area"])
        min_fill=float(spec["rectangularize_fill_ratio"])
        for component_id in range(1,count+1):
            ys,xs=np.where(components==component_id)
            if len(xs)<min_area:
                continue
            x0,x1=int(xs.min()),int(xs.max())+1
            y0,y1=int(ys.min()),int(ys.max())+1
            if len(xs)/((x1-x0)*(y1-y0))>=min_fill:
                boxes.append((x0,x1,y0,y1))
        mask=Image.new("L",mask.size,0)
        draw=ImageDraw.Draw(mask)
        for x0,x1,y0,y1 in boxes:
            draw.rectangle((x0,y0,x1-1,y1-1),fill=255)
    # Apply small, explicit geometric cleanups in final occupancy-grid pixels.
    # These are limited to the reported aisle/corner regions; other extracted
    # obstacle geometry remains untouched.
    draw = ImageDraw.Draw(mask)
    for x0, x1, y0, y1 in spec.get("free_rectangles", []):
        draw.rectangle((x0, y0, x1-1, y1-1), fill=0)
    for x0, x1, y0, y1 in spec.get("obstacle_rectangles", []):
        draw.rectangle((x0, y0, x1-1, y1-1), fill=255)
    for x0, x1, y0, y1 in spec.get("obstacle_rectangles_only", []):
        draw.rectangle((x0, y0, x1-1, y1-1), fill=255)
    source_resolution = source_res
    upsample = max(1, int(spec.get("output_upsample", 1)))
    if upsample > 1:
        mask = mask.resize((mask.width*upsample, mask.height*upsample),
                           Image.Resampling.NEAREST)
        source_res /= upsample
    source_vehicle_width = PAPER_VEHICLE_WIDTH_M[spec["paper"]]
    scale_factor = float(spec.get(
        "map_scale_factor", ROBOT_WIDTH_M / source_vehicle_width))
    res = source_res * scale_factor
    center_x, center_y = (xmin+xmax)/2.0, (ymin+ymax)/2.0
    map_xmin = center_x - mask.width*res/2.0
    map_ymin = center_y - mask.height*res/2.0
    # The cropped page is y-up in world coordinates, while PGM row zero is top.
    pgm = ImageOps.invert(mask).convert("L")
    dest = OUT / MAP_DIRS[spec["name"]]
    dest.mkdir(parents=True, exist_ok=True)
    pgm.save(dest / "map.pgm")
    preview = pgm.copy()
    max_dimension = max(preview.size)
    if max_dimension > 1600:
        factor = 1600.0 / max_dimension
        preview = preview.resize(
            (max(1, round(preview.width * factor)),
             max(1, round(preview.height * factor))),
            Image.Resampling.NEAREST)
    preview.save(dest / "preview.png")
    (dest / "map.yaml").write_text(
        "image: map.pgm\n"
        f"resolution: {res:.6f}\n"
        f"origin: [{map_xmin:.4f}, {map_ymin:.4f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8")
    source = (
        f"name: {spec['name']}\n"
        f"source_pdf: {spec['paper']}\n"
        f"source_page: {spec['page']}\n"
        f"source_figure: \"{spec['figure']}\"\n"
        f"description: \"{spec['caption']}\"\n"
        f"paper_coordinate_bounds: [{xmin}, {xmax}, {ymin}, {ymax}]\n"
        f"map_coordinate_bounds: [{map_xmin:.6f}, {map_xmin+mask.width*res:.6f}, {map_ymin:.6f}, {map_ymin+mask.height*res:.6f}]\n"
        f"source_plot_pixels: [{left}, {top}, {right}, {bottom}]\n"
        f"source_resolution_m_per_pixel_approx: {source_resolution:.6f}\n"
        f"map_resolution_m_per_pixel: {res:.6f}\n"
        f"robot_scale_factor: {scale_factor:.6f}\n"
        f"paper_vehicle_width_m: {source_vehicle_width:.3f}\n"
        f"target_robot_width_m: {ROBOT_WIDTH_M:.3f}\n"
        f"scale_reference: \"{spec.get('scale_note', 'paper vehicle width mapped to target robot width')}\"\n"
        f"scale_center: [{center_x:.6f}, {center_y:.6f}]\n"
        f"obstacle_fill_class: {spec['fill']}\n"
        f"ignored_overlays: {spec.get('ignore_world_boxes', [])}\n"
        f"cleanup_notes: \"{spec.get('cleanup_notes', 'none')}\"\n"
        "pose_source: not encoded in paper; project-defined test poses are in scene_meta.yaml\n"
        "limitations: >-\n"
        "  Reconstructed from a printed simulation figure, not the authors'\n"
        "  occupancy grid. Boundaries and scale are approximate. Recheck all\n"
        "  paths with the vehicle swept-footprint collision checker.\n"
    )
    (dest / "scene_source.yaml").write_text(source, encoding="utf-8")
    return pgm


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    CATALOG.mkdir(parents=True, exist_ok=True)
    rendered = {}
    thumbnails = []
    with tempfile.TemporaryDirectory(prefix="paper_map_digitize_") as tmp_name:
        tmp = Path(tmp_name)
        for spec in MAPS:
            key = (spec["paper"], spec["page"])
            if key not in rendered:
                rendered[key] = render_page(PAPERS / spec["paper"], spec["page"], tmp)
            thumbnails.append((spec, write_map(spec, rendered[key])))

    cell_w, cell_h = 400, 350
    preview_pixels_per_m = 16.0
    sheet = Image.new("RGB", (1200, ((len(thumbnails)+2)//3)*cell_h), "white")
    draw = ImageDraw.Draw(sheet)
    for i, (spec, pgm) in enumerate(thumbnails):
        col, row = i % 3, i // 3
        x0, y0 = col*cell_w, row*cell_h
        source_res = ((spec["bounds"][1]-spec["bounds"][0])/pgm.width +
                      (spec["bounds"][3]-spec["bounds"][2])/pgm.height)/2.0
        map_res = source_res*float(spec.get(
            "map_scale_factor", ROBOT_WIDTH_M/PAPER_VEHICLE_WIDTH_M[spec["paper"]]))
        thumb_size = (max(1, round(pgm.width*map_res*preview_pixels_per_m)),
                      max(1, round(pgm.height*map_res*preview_pixels_per_m)))
        thumb = pgm.resize(thumb_size, Image.Resampling.NEAREST).convert("RGB")
        sheet.paste(thumb, (x0+15+(370-thumb.width)//2,
                            y0+42+(290-thumb.height)//2))
        draw.text((x0+12, y0+8), spec["name"], fill="#111111")
        car_w = max(2, round(0.738*preview_pixels_per_m))
        car_h = max(2, round(0.500*preview_pixels_per_m))
        draw.rectangle((x0+300, y0+8, x0+300+car_w, y0+8+car_h),
                       outline="#1677cc", width=2)
        draw.text((x0+car_w+306, y0+8), "car", fill="#1677cc")
    sheet.save(CATALOG / "digitized_maps_overview.png")

    lines = [
        "# 论文场景数字化地图（车体尺度校准）", "",
        "障碍栅格从论文图提取，不是作者原始 occupancy grid。P11_case_b 的目标通道做了局部加宽和直边处理，P11_case_c 删除了孤立细碎提取物，P12_env1/P12_env4 重建了矩形障碍边界并清理细碎图元，P12_env3 的弯道内侧墙边做了保守填平；其他障碍保持提取轮廓。P12_env1/P12_env4 的输出栅格分辨率提高到 3 倍，米制地图范围保持不变。为匹配本项目 0.500 m 车宽，基础地图按论文车辆宽度统一等比例缩放，P11/P12/P16 的比例因子约为 0.299/0.806/0.250。运行时还可用 `--map-scale` 改变整张地图的米制尺度。", "",
        "![数字化地图总览](digitized_maps_overview.png)", "",
        "| 地图 | 来源 | 论文坐标范围 | ROS 分辨率 | 全图比例因子 |", "|---|---|---|---:|---:|",
    ]
    for spec, pgm in thumbnails:
        xmin, xmax, ymin, ymax = spec["bounds"]
        source_res = ((xmax-xmin)/pgm.width + (ymax-ymin)/pgm.height)/2
        scale_factor = float(spec.get(
            "map_scale_factor", ROBOT_WIDTH_M / PAPER_VEHICLE_WIDTH_M[spec["paper"]]))
        res = source_res*scale_factor
        lines.append(
            f"| [{spec['name']}](../{MAP_DIRS[spec['name']]}/README.md) | "
            f"`{spec['paper']}` p.{spec['page']} {spec['figure']} | "
            f"source x=[{xmin:g},{xmax:g}], y=[{ymin:g},{ymax:g}] | "
            f"{res:.4f} m/px | scale ×{scale_factor:.3f} |")
    lines += [
        "", "## 使用", "",
        "地图障碍按论文图提取；P11_case_b 的目标通道做了局部加宽和直边处理，P11_case_c 去除了小型孤立提取碎片，P12_env1/P12_env4 重建了矩形边界并清理细碎图元，P12_env3 的弯道内侧墙边做了保守填平。P12_env1/P12_env4 输出分辨率提高 3 倍但米制范围不变；除此之外不对障碍做膨胀。原论文车辆比本项目车辆大，因此基础比例以论文车宽映射到本车 0.500 m，保持 x/y 等比例；P11/P12/P16 通常约为 ×0.299/×0.806/×0.250。Figure 22 按 1.0 m 纵向通道校准，比例为 ×0.174。总览统一使用 16 px/m，蓝色轮廓是 0.738×0.500 m 车体参照。", "",
        "运行时可用 `--map-scale` 按地图中心统一缩放米制尺寸；map01 默认 ×0.75、map03 ×0.7、map04 ×0.5、map05 ×0.7、map07 ×0.8、map08 ×1.2、map15 ×0.8，其余保留地图默认 ×1.0，均可在命令行覆盖。map08 和 map09 使用 5 像素中值滤波平滑数字化墙边。P12_env2 的停车排墙和齿形障碍已按原直边重建，去除了栅格毛刺及轨迹覆盖造成的缺口。可运行 `./run_demo.sh --list-maps` 查看地图名称。map01–map09 的项目测试位姿保存在各自 `scene_meta.yaml`，不是论文作者给出的原始位姿；启动时默认自动载入，可传 `auto_goal:=false` 后在 RViz 手动设置。", "",
        "脚本：`tools/digitize_paper_maps.py`。地图比例基于论文中车辆宽度和本车宽度的比值标定，车长/轴距比例仍存在不同论文车型之间的差异；这是为比较通行净宽所作的统一尺度映射，不是论文原始米制坐标复现。", "",
    ]
    (CATALOG / "README.md").write_text("\n".join(lines), encoding="utf-8")
    print(f"Wrote {len(MAPS)} approximate ROS maps to {OUT}")


if __name__ == "__main__":
    main()
