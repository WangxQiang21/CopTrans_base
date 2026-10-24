# map06-p12-bent-corridor-env3 — P12 Env3 弯折通道

![地图预览](preview.png)

- 原名称：`P12_env3`。
- 分类：论文图数字化。
- 特点：对弯道内侧墙边做了保守填平。
- 论文来源：[《12_Bao_4WIS_Executable_Trajectories_2026.pdf》](../../papers/12_Bao_4WIS_Executable_Trajectories_2026.pdf)，PDF 第 9 页，Fig. 5, Env3。 [论文图裁切](../paper_candidates/R04_env3.png)。
- 默认运行时缩放：×1。底图比例及数字化细节见 `scene_source.yaml`。
- 启动：`./run_demo.sh --map map06-p12-bent-corridor-env3`（旧名称 `P12_env3` 仍作为别名接受）。

## 文件

- `preview.png`：地图预览。
- `map.pgm` 与 `map.yaml`：ROS occupancy map 栅格及分辨率/原点配置。
- `scene_source.yaml`：场景源配置与来源记录。
- 预设测试位姿（地图基础坐标）：起点 `(1.864, 2.507, 0.000)`，终点 `(6.933, 2.534, 3.142)`；按默认缩放后直线间距约 `5.07 m`。位置按截图标记换算，并在默认缩放下检查了车体矩形 footprint；规划可达性仍由运行结果确认。这些是项目测试位姿，不是论文原始位姿；启动时自动使用，显式传 `auto_goal:=false` 可改为 RViz 手动设置。
- `scene_meta.yaml`：项目固定测试起点与终点。
