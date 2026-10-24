# map01-p11-narrow-case-a — P11 窄通道案例 A

![地图预览](preview.png)

- 原名称：`P11_case_a`。
- 分类：论文图数字化。
- 特点：论文窄通道场景 A；地图来自论文图像提取。
- 论文来源：[《11_Chang_4WIS_Parking_Mode_Decision_2024.pdf》](../../papers/11_Chang_4WIS_Parking_Mode_Decision_2024.pdf)，PDF 第 7 页，Fig. 5(a)。 [论文图裁切](../paper_candidates/R01_fig1_three_cases.png)。
- 默认运行时缩放：×0.75。底图比例及数字化细节见 `scene_source.yaml`。
- 启动：`./run_demo.sh --map map01-p11-narrow-case-a`（旧名称 `P11_case_a` 仍作为别名接受）。

## 文件

- `preview.png`：地图预览。
- `map.pgm` 与 `map.yaml`：ROS occupancy map 栅格及分辨率/原点配置。
- `scene_source.yaml`：场景源配置与来源记录。
- 预设测试位姿（地图基础坐标）：起点 `(6.169, 6.537, 0.000)`，终点 `(8.914, 4.715, 0.000)`；按默认缩放后直线间距约 `2.47 m`。位置按截图标记换算，并在默认缩放下检查了车体矩形 footprint；规划可达性仍由运行结果确认。这些是项目测试位姿，不是论文原始位姿；启动时自动使用，显式传 `auto_goal:=false` 可改为 RViz 手动设置。
- `scene_meta.yaml`：项目固定测试起点与终点。
