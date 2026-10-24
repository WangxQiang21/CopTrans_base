# 地图目录

每张可运行地图都放在一个 `mapNN-特点` 目录中，目录内包含地图栅格、ROS 配置、预览图、来源说明和测试位姿。当前运行列表保留 map01–map09、map15、map16；map10–map14 已移除。编号可用两位或三位写法，例如 `map01` / `map001`、`map15` / `map015`；也可传完整目录名或兼容旧名称。完整映射见 [`map_index.tsv`](map_index.tsv)。

## 当前可运行地图

| 编号 | 地图目录 | 特点 / 原名称 | 默认比例 | 默认缩放后的起终点直线间距 | 来源 |
|---|---|---|---:|---:|---|
| map01 | [`map01-p11-narrow-case-a/`](map01-p11-narrow-case-a/README.md) | P11 案例 A，窄通道 | 0.75 | 2.47 m | Chang 2024，Fig. 5(a) |
| map02 | [`map02-p11-goal-side-aisle/`](map02-p11-goal-side-aisle/README.md) | P11 案例 B，目标侧窄通道 | 1.0 | 1.36 m | Chang 2024，Fig. 5(b) |
| map03 | [`map03-p11-narrow-case-c/`](map03-p11-narrow-case-c/README.md) | P11 案例 C，窄通道 | 0.7 | 1.72 m | Chang 2024，Fig. 5(c) |
| map04 | [`map04-p12-maze-env1/`](map04-p12-maze-env1/README.md) | P12 Env1，迷宫 | 0.5 | 4.10 m | Bao 2026，Fig. 5 Env1 |
| map05 | [`map05-p12-parking-row-env2/`](map05-p12-parking-row-env2/README.md) | P12 Env2，停车排 | 0.7 | 2.82 m | Bao 2026，Fig. 5 Env2 |
| map06 | [`map06-p12-bent-corridor-env3/`](map06-p12-bent-corridor-env3/README.md) | P12 Env3，弯折通道 | 1.0 | 5.07 m | Bao 2026，Fig. 5 Env3 |
| map07 | [`map07-p12-obstacle-field-env4/`](map07-p12-obstacle-field-env4/README.md) | P12 Env4，障碍场 | 0.8 | 5.17 m | Bao 2026，Fig. 5 Env4 |
| map08 | [`map08-p16-complex-parking/`](map08-p16-complex-parking/README.md) | P16 复杂泊车 | 1.2 | 6.34 m | Zhu 2026，Figure 22 |
| map09 | [`map09-p16-narrow-maze/`](map09-p16-narrow-maze/README.md) | P16 窄迷宫 | 1.0 | 7.28 m | Zhu 2026，Figure 19 |
| map15 | [`map15-n06-two-stage-offset-passage/`](map15-n06-two-stage-offset-passage/README.md) | 两段错位通道压力图 | 0.8 | 8.62 m | 项目合成测试图 |
| map16 | [`map16-s09-underground-garage/`](map16-s09-underground-garage/README.md) | 地下车库演示图 | 1.0 | 7.53 m | 项目内置场景 |

## 地图目录包含什么

- `preview.png`：地图栅格预览。
- `map.pgm`、`map.yaml`：ROS 占据栅格、分辨率和原点配置。
- `scene_source.yaml`：论文图数字化来源或合成场景源配置。
- `scene_meta.yaml`：规划使用的预设起点和终点。论文地图中的位姿是项目选取的测试位姿，不是论文原文给定的位姿。
- `README.md`：地图特点、来源、默认比例和预设位姿说明。

使用 `./run_demo.sh --map mapNN` 选择地图时，脚本会使用 `scene_meta.yaml` 中的起点并自动发送终点。若要自行设置目标，可传 `auto_goal:=false`，再在 RViz 使用 **2D Pose Estimate** 和 **2D Nav Goal**。

表中距离是默认缩放后起点到终点的直线距离，不是规划路线长度。map01–map07、map09 的位姿按你提供的截图换算；map08 没有对应截图，保留此前选取的测试位姿。所有位姿均为项目测试设置，不是论文原文给定的位置；坐标与朝向保存在各地图的 `scene_meta.yaml`。

`paper_candidates/` 保留论文图裁切素材，不作为可运行地图；`paper_digitized/` 保留九张论文地图的总览和来源表；`archive/` 保留旧版改图，不进入运行列表。map15 是目前保留的合成压力测试图。

```bash
./run_demo.sh --list-maps
```
