# expv3

本文档列出 `run_demo.sh` 的完整用法。地图目录及预览见[地图总索引](maps/README.md)，可运行地图与旧名称的对应关系见 [`maps/map_index.tsv`](maps/map_index.tsv)。

## 快速开始

```bash
cd /home/wq/whut_thesis/expv3

# 启动默认地图（S09 地下车库），自动规划默认目标
bash run_demo.sh

# 查看所有地图及其旧名称
bash run_demo.sh --list-maps

# 查看命令帮助
bash run_demo.sh --help
```

启动脚本会加载 ROS Noetic、构建 `catkin_ws`，再启动 Demo。未指定规划后端时使用 `legacy`（本分支默认）。检测到图形桌面时打开 RViz；无图形桌面时自动关闭 RViz。

## 选择地图

`--map` 只能指定一次。它接受地图编号简写、地图目录名、旧地图名，或包含 `map.yaml` 的目录路径：

```bash
# 编号简写：map01–map09、map15、map16；三位补零形式 map001–map009、map015、map016 也兼容
bash run_demo.sh --map map01
bash run_demo.sh --map map05

# 也兼容编号前的分隔符写法
bash run_demo.sh --map - map01

# 完整目录名或原有地图名
bash run_demo.sh --map map05-p12-parking-row-env2
bash run_demo.sh --map P12_env2

# 传地图目录路径
bash run_demo.sh --map maps/map05-p12-parking-row-env2
bash run_demo.sh --map /absolute/path/to/map_directory
```

也可写成 `--map=map01` 或使用短选项 `-m map01`。选图后设置 `scene:=manual_map`，并默认读取地图 `scene_meta.yaml` 中的预设起点和终点自动规划。地图起终点表示车体中心位姿；若要手动指定，可加 `auto_goal:=false`，再在 RViz 中用 **2D Pose Estimate** 设置起点、用 **2D Nav Goal** 设置终点。

## 地图缩放

`--map-scale`、`--scale` 和 `-scale` 都接受一个有限且大于零的比例，地图围绕中心缩放，栅格尺寸不变。比例只影响本次启动生成的临时地图副本；退出 Demo 后副本会清理。

```bash
bash run_demo.sh --map map04 --map-scale 0.55
bash run_demo.sh --map map01 -scale 0.7
bash run_demo.sh --map P12_env2 --scale 1.0
bash run_demo.sh --map-scale=0.8 map_dir:=maps/map01-p11-narrow-case-a
```

未手动指定比例时，默认值为：map01 `0.75`、map03 `0.7`、map04 `0.5`、map05 `0.7`、map07 `0.8`、map08 `1.2`、map15 `0.8`；map02、map06、map09、map16 为 `1.0`。显式传入比例会覆盖地图默认值。缩放选项需要有效的 `--map` 或 `map_dir:=` 地图目录。

## 启动参数

地图选项解析后，其他参数会原样传给 `roslaunch fwis_v3_demo demo.launch`。下表列出 Demo 定义的常用参数：

| 参数 | 可用值/格式 | 作用与默认行为 |
|---|---|---|
| `planning_backend:=...` | `mode_path_v2`、`mode_path_v1`、`legacy` | 选择规划后端。通过 `run_demo.sh` 启动时默认 `legacy`（本分支）。 |
| `target_v:=...` | 正数，单位 m/s | 覆盖目标速度；同步设置速度上限。默认配置为 `2.0`。 |
| `direction_policy:=...` | `bidirectional`、`forward_only` | 新后端方向策略；默认允许正反向候选参与比较。 |
| `allow_reverse_fallback:=...` | `true`、`false` | 兼容旧参数：映射到双向或仅正向策略。与 `direction_policy` 同时指定时，两者必须一致。 |
| `config_file:=...` | YAML 配置文件路径 | 覆盖默认参数文件。受限动态对照配置为 `catkin_ws/src/fwis_v3_demo/config/params_restricted.yaml`。 |
| `rviz:=...` | `true`、`false` | 控制是否启动 RViz；有图形桌面时脚本默认开启，无图形桌面时默认关闭。 |
| `auto_goal:=...` | `true`、`false` | 控制是否使用地图预设目标。默认地图和带 `scene_meta.yaml` 的手动选图默认为 `true`；显式设为 `false` 可自行指定起终点。 |
| `scene:=...` | 场景名 | 选择场景标注和场景配置。默认场景为 S09；手动选图时脚本改为 `manual_map`。 |
| `map_dir:=...` | 含 `map.yaml` 的地图目录 | 直接指定地图。可替代 `--map`；二者不能同时使用。未指定时使用 S09 地图。 |
| `scene_yaml:=...` | 场景 YAML 路径 | 指定场景源配置；默认读取所选地图目录中的 `scene_source.yaml`。 |

示例：

```bash
# 选择 v1 后端
bash run_demo.sh --map map01 planning_backend:=mode_path_v1

# 选择 v2，覆盖目标速度和行驶方向策略
bash run_demo.sh --map map05 planning_backend:=mode_path_v2 target_v:=1.5 direction_policy:=forward_only

# 使用受限动态配置
bash run_demo.sh --map P12_env1 config_file:=/home/wq/whut_thesis/expv3/catkin_ws/src/fwis_v3_demo/config/params_restricted.yaml

# 不开 RViz；地图和场景也可以直接通过 roslaunch 参数传入
bash run_demo.sh map_dir:=maps/map16-s09-underground-garage scene:=S09_underground_garage auto_goal:=true rviz:=false
```

除上表参数外，也可传递 `demo.launch` 接受的其他 `name:=value` 参数。未知参数会交由 `roslaunch` 校验。

## 命令选项规则

- `--help` / `-h`：显示脚本帮助并退出。
- `--list-maps`：列出可运行地图编号、完整目录名和旧地图名，然后退出；不构建、不启动 ROS。
- `--map NAME` / `-m NAME` / `--map=NAME`：选择地图，只能指定一次。
- `--map - NAME`：`--map NAME` 的兼容分隔符写法，例如 `--map - map01`。
- `--map-scale VALUE` / `--scale VALUE` / `-scale VALUE`，以及对应的 `=VALUE` 写法：设置地图缩放，只能指定一次。
- 其他位置参数作为 ROS launch 参数传递，例如 `rviz:=false` 或 `target_v:=1.8`。

更多节点、话题和架构说明见 [`catkin_ws/README.md`](catkin_ws/README.md)；项目方案和实验记录集中在 [`plan/`](plan/README.md)。

## Algorithm branches

The checked out branch selects `legacy` by default. Branch purpose and its closest paper reference are documented in [`BRANCH_METHOD.md`](BRANCH_METHOD.md); the full branch-to-paper table is in [`docs/algorithm_paper_branches.md`](docs/algorithm_paper_branches.md). `main` is the shared project snapshot and defaults to `mode_path_v2`. All three backend implementations remain available for explicit comparisons.
