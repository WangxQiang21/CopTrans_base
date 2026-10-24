#!/usr/bin/env python3
"""Extract paper benchmark map/result panels for the local 4WIS planner."""
from __future__ import annotations

import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
PAPERS = ROOT / "papers"
OUT = ROOT / "maps" / "paper_candidates"
DPI = 180

# Crop boxes are pixel coordinates on pdftoppm's 180 dpi portrait page render.
PANELS = [
    {
        "id": "P01",
        "paper": "06_Wang_4WIS_HybridAstar_2025.pdf",
        "page": 5,
        "box": (1158, 155, 1405, 365),
        "figure": "Fig. 4, Env1-S3",
        "caption": "20 x 20 m maze; narrow passages and frequent turns; source reports 2 m corridor width.",
    },
    {
        "id": "P02",
        "paper": "08_Yang_4WIS_Motion_Mode_Sequence_2026.pdf",
        "page": 14,
        "box": (145, 550, 550, 1000),
        "figure": "Fig. 7(a), Case 1",
        "caption": "Narrow passage with complex obstacles; proposed 4WIS result.",
    },
    {
        "id": "P03",
        "paper": "08_Yang_4WIS_Motion_Mode_Sequence_2026.pdf",
        "page": 14,
        "box": (145, 1018, 550, 1395),
        "figure": "Fig. 7(d), Case 2",
        "caption": "Second narrow-passage layout; proposed 4WIS result.",
    },
    {
        "id": "P04",
        "paper": "12_Bao_4WIS_Executable_Trajectories_2026.pdf",
        "page": 9,
        "box": (770, 630, 1045, 805),
        "figure": "Fig. 5, Env3",
        "caption": "Compact bent corridor benchmark; 4WIS trajectory overlaid.",
    },
    {
        "id": "P05",
        "paper": "12_Bao_4WIS_Executable_Trajectories_2026.pdf",
        "page": 9,
        "box": (1050, 825, 1338, 1015),
        "figure": "Fig. 5, Env4",
        "caption": "Obstacle-field benchmark with a winding route; 4WIS trajectory overlaid.",
    },
    {
        "id": "P06",
        "paper": "16_Zhu_Adaptive_Multi_Mode_4WIS_2026.pdf",
        "page": 22,
        "box": (415, 520, 965, 1040),
        "figure": "Fig. 19",
        "caption": "30 x 30 m narrow maze; source labels it a severely constrained environment.",
    },
]

# Figures requested from the four paper figures discussed with the user.
# Coordinates are on the same 180-dpi page renders as PANELS above.
REQUESTED = [
    {
        "id": "R01_fig1_three_cases",
        "paper": "11_Chang_4WIS_Parking_Mode_Decision_2024.pdf",
        "page": 7,
        "box": (120, 125, 1400, 455),
        "figure": "Fig. 5(a)-(c)",
        "caption": "Three narrow-scene cases combined as one panel (HA results, as in the supplied reference).",
    },
    {
        "id": "R02_env1",
        "paper": "12_Bao_4WIS_Executable_Trajectories_2026.pdf",
        "page": 9,
        "box": (195, 175, 1340, 435),
        "figure": "Fig. 5, Env1",
        "caption": "Environment 1 comparison, separated from the four-environment overview.",
    },
    {
        "id": "R03_env2",
        "paper": "12_Bao_4WIS_Executable_Trajectories_2026.pdf",
        "page": 9,
        "box": (195, 435, 1340, 625),
        "figure": "Fig. 5, Env2",
        "caption": "Environment 2 comparison, separated from the four-environment overview.",
    },
    {
        "id": "R04_env3",
        "paper": "12_Bao_4WIS_Executable_Trajectories_2026.pdf",
        "page": 9,
        "box": (195, 625, 1340, 805),
        "figure": "Fig. 5, Env3",
        "caption": "Environment 3 comparison, separated from the four-environment overview.",
    },
    {
        "id": "R05_env4",
        "paper": "12_Bao_4WIS_Executable_Trajectories_2026.pdf",
        "page": 9,
        "box": (195, 805, 1340, 1000),
        "figure": "Fig. 5, Env4",
        "caption": "Environment 4 comparison, separated from the four-environment overview.",
    },
    {
        "id": "R06_complex_parking",
        "paper": "16_Zhu_Adaptive_Multi_Mode_4WIS_2026.pdf",
        "page": 24,
        "box": (380, 465, 975, 1035),
        "figure": "Figure 22",
        "caption": "Complex parking benchmark, proposed algorithm.",
    },
    {
        "id": "R07_narrow_maze",
        "paper": "16_Zhu_Adaptive_Multi_Mode_4WIS_2026.pdf",
        "page": 22,
        "box": (380, 475, 975, 1045),
        "figure": "Figure 19",
        "caption": "Narrow-maze benchmark, proposed algorithm.",
    },
]


def render_page(pdf: Path, page: int, tmp: Path) -> Image.Image:
    prefix = tmp / f"{pdf.stem}_{page}"
    subprocess.run([
        "pdftoppm", "-f", str(page), "-l", str(page), "-r", str(DPI),
        "-png", "-singlefile", str(pdf), str(prefix),
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return Image.open(prefix.with_suffix(".png")).convert("RGB")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rendered = {}
    with tempfile.TemporaryDirectory(prefix="paper_map_extract_") as temp:
        temp = Path(temp)
        all_panels = PANELS + REQUESTED
        for panel in all_panels:
            key = (panel["paper"], panel["page"])
            if key not in rendered:
                rendered[key] = render_page(PAPERS / panel["paper"], panel["page"], temp)
            page = rendered[key]
            crop = page.crop(panel["box"])
            if crop.size == (0, 0):
                raise ValueError(f"empty crop for {panel['id']}: page size {page.size}")
            panel["output"] = f"{panel['id']}.png"
            crop.save(OUT / panel["output"])

    requested_sheet = Image.new("RGB", (1440, len(REQUESTED) * 440), "white")
    draw = ImageDraw.Draw(requested_sheet)
    try:
        title_font = ImageFont.truetype("DejaVuSans.ttf", 20)
        note_font = ImageFont.truetype("DejaVuSans.ttf", 14)
    except OSError:
        title_font = note_font = ImageFont.load_default()
    for i, panel in enumerate(REQUESTED):
        img = Image.open(OUT / panel["output"]).convert("RGB")
        img.thumbnail((1400, 385), Image.Resampling.LANCZOS)
        y0 = i * 440
        draw.text((18, y0 + 8), f"{panel['id']}  |  {panel['figure']}",
                  fill="#111827", font=title_font)
        draw.text((18, y0 + 34), panel["paper"], fill="#374151", font=note_font)
        requested_sheet.paste(img, (18, y0 + 56))
    requested_sheet.save(OUT / "requested_paper_figures.png")

    thumb_w, thumb_h = 680, 400
    cell_w, cell_h = 720, 560
    sheet = Image.new("RGB", (cell_w * 2, cell_h * 3), "white")
    draw = ImageDraw.Draw(sheet)
    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 19)
        small = ImageFont.truetype("DejaVuSans.ttf", 14)
    except OSError:
        font = small = ImageFont.load_default()
    for i, panel in enumerate(PANELS):
        image = Image.open(OUT / panel["output"]).convert("RGB")
        image.thumbnail((thumb_w, thumb_h), Image.Resampling.LANCZOS)
        x0, y0 = (i % 2) * cell_w, (i // 2) * cell_h
        draw.text((x0 + 12, y0 + 8), f"{panel['id']}  {panel['figure']}", fill="#111827", font=font)
        draw.text((x0 + 12, y0 + 34), panel["paper"], fill="#374151", font=small)
        draw.text((x0 + 12, y0 + 54), f"PDF page {panel['page']}", fill="#374151", font=small)
        ix, iy = x0 + 12, y0 + 82
        sheet.paste(image, (ix, iy))
        draw.rectangle((ix, iy, ix + image.width - 1, iy + image.height - 1), outline="#9ca3af", width=1)
        caption_y = min(y0 + cell_h - 36, iy + image.height + 5)
        draw.text((x0 + 12, caption_y), panel["caption"], fill="#111827", font=small)
    sheet.save(OUT / "paper_map_candidates.png")

    lines = [
        "# Paper map candidates and requested figures",
        "",
        "以下是从本地论文 PDF 原页裁出的地图/仿真图面板，按源图保留了坐标轴、障碍和论文轨迹，用于挑选基准场景。它们不是原始 occupancy-grid 数据，也尚未转换成 ROS 地图；选定后再依据坐标轴和障碍边界数字化重建，并复核车辆扫掠安全。",
        "",
        "![论文地图候选面板](paper_map_candidates.png)",
        "",
        "| ID | 来源 PDF | PDF 页 / 图号 | 场景 |",
        "|---|---|---|---|",
    ]
    for p in PANELS:
        lines.append(f"| [{p['id']}]({p['output']}) | `{p['paper']}` | {p['page']} / {p['figure']} | {p['caption']} |")
    lines += [
        "",
        "## 本轮按建议拆出的论文图",
        "",
        "![本轮论文图](requested_paper_figures.png)",
        "",
        "| 文件 | 来源 PDF | 页码 / 图号 | 内容 |",
        "|---|---|---|---|",
    ]
    for p in REQUESTED:
        lines.append(f"| [{p['output']}]({p['output']}) | `{p['paper']}` | {p['page']} / {p['figure']} | {p['caption']} |")
    lines += [
        "",
        "注意：这些是论文仿真结果图的裁切版，保留了作者路径/车体轮廓以便对照；不能直接当作干净的占据栅格地图。要用于本项目算法对比，还需从图中数字化障碍边界、起终点与航向，并对原文缺失的地图精度作显式记录。",
        "",
        "原始候选面板图中的轨迹/车体轮廓也属于论文结果，不应当作障碍地图。",
    ]
    (OUT / "README.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Extracted {len(PANELS) + len(REQUESTED)} panels to {OUT}")


if __name__ == "__main__":
    main()
