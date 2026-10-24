# map04-p12-maze-env1 — P12 Env1 迷宫

![地图预览](preview.png)

- 原名称：`P12_env1`。
- 分类：论文图数字化。
- 特点：重建轴对齐墙体，清理假缺口和栅格锯齿；输出栅格分辨率提高 3 倍。
- 论文来源：[《12_Bao_4WIS_Executable_Trajectories_2026.pdf》](../../papers/12_Bao_4WIS_Executable_Trajectories_2026.pdf)，PDF 第 9 页，Fig. 5, Env1。 [论文图裁切](../paper_candidates/R02_env1.png)。
- 默认运行时缩放：×0.5。底图比例及数字化细节见 `scene_source.yaml`。
- 启动：`./run_demo.sh --map map04-p12-maze-env1`（旧名称 `P12_env1` 仍作为别名接受）。

## 文件

- `preview.png`：地图预览。
- `map.pgm` 与 `map.yaml`：ROS occupancy map 栅格及分辨率/原点配置。
- `scene_source.yaml`：场景源配置与来源记录。
- 预设测试位姿（地图基础坐标）：起点 `(5.424, 16.718, 0.000)`，终点 `(13.179, 14.024, 0.000)`；按默认缩放后直线间距约 `4.10 m`。起点从截图位置向通道内调整到可容纳车体矩形 footprint 的最近栅格位置；规划可达性仍由运行结果确认。这些是项目测试位姿，不是论文原始位姿；启动时自动使用，显式传 `auto_goal:=false` 可改为 RViz 手动设置。
- `scene_meta.yaml`：项目固定测试起点与终点。
