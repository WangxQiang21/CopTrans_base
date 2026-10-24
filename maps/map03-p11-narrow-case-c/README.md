# map03-p11-narrow-case-c — P11 窄通道案例 C

![地图预览](preview.png)

- 原名称：`P11_case_c`。
- 分类：论文图数字化。
- 特点：移除了孤立的细碎栅格提取物。
- 论文来源：[《11_Chang_4WIS_Parking_Mode_Decision_2024.pdf》](../../papers/11_Chang_4WIS_Parking_Mode_Decision_2024.pdf)，PDF 第 7 页，Fig. 5(c)。 [论文图裁切](../paper_candidates/R01_fig1_three_cases.png)。
- 默认运行时缩放：×0.7。底图比例及数字化细节见 `scene_source.yaml`。
- 启动：`./run_demo.sh --map map03-p11-narrow-case-c`（旧名称 `P11_case_c` 仍作为别名接受）。

## 文件

- `preview.png`：地图预览。
- `map.pgm` 与 `map.yaml`：ROS occupancy map 栅格及分辨率/原点配置。
- `scene_source.yaml`：场景源配置与来源记录。
- 预设测试位姿（地图基础坐标）：起点 `(5.208, 3.298, 0.000)`，终点 `(7.632, 3.715, 0.000)`；按默认缩放后直线间距约 `1.72 m`。终点从截图位置向通道内调整到可容纳车体矩形 footprint 的最近栅格位置；规划可达性仍由运行结果确认。这些是项目测试位姿，不是论文原始位姿；启动时自动使用，显式传 `auto_goal:=false` 可改为 RViz 手动设置。
- `scene_meta.yaml`：项目固定测试起点与终点。
