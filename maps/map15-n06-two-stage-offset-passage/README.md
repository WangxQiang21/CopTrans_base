# map15-n06-two-stage-offset-passage — N06 两段错位通道

![地图预览](preview.png)

- 原名称：`N06_two_stage_offset_passage`。
- 分类：合成压力测试图。
- 特点：窄直廊后接近 90° 转弯，再进入第二窄廊。
- 论文来源：无。这是项目内生成的合成压力测试地图，不属于论文复刻。生成器：[`tools/generate_narrow_maps.py`](../../tools/generate_narrow_maps.py)；任务基准：[`tools/expv3_eval_cases.yaml`](../../tools/expv3_eval_cases.yaml)。
- 默认运行时缩放：×0.8。
- 启动：`./run_demo.sh --map map15-n06-two-stage-offset-passage`（旧名称 `N06_two_stage_offset_passage` 仍作为别名接受）。
- 预设测试位姿（基础地图坐标）：起点 `(1.000, 2.700, 0.000)`，终点 `(10.950, 6.850, 1.571)`；默认缩放后直线间距约 `8.62 m`。

## 文件

- `preview.png`：地图预览。
- `map.pgm` 与 `map.yaml`：ROS occupancy map 栅格及分辨率/原点配置。
- `scene_source.yaml`：场景源配置与来源记录。
- `scene_meta.yaml`：地图预设测试起点与终点；启动 map15 时默认自动使用，也可传 `auto_goal:=false` 后手动指定。
