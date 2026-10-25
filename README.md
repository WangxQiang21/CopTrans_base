# expv3

## 本分支：模式一致的连续轨迹与段级时间求解

本分支 `continuous-mode-path` 运行 `mode_path_v2`。参考论文 12：Runjiao Bao 等，
*From Multi-Modal Paths to Executable Trajectories: A Trajectory Planning Framework for 4WIS Robots*
（[本地论文](papers/12_Bao_4WIS_Executable_Trajectories_2026.pdf)，
[arXiv:2608.29108](https://arxiv.org/abs/2608.29108)）。
主要借鉴前端模式路径与后端可执行轨迹衔接、按模式一致的连续段处理、切换时保持静止三个思路。
本分支用连续曲率/斜移角描述段内控制，连续速度与加速度可以跨过普通控制节点；
模式事件才添加静止重配置，候选最终按独立整车扫掠与动态审核结果竞争。

本次把重复的整路径时间计算拆成可复用的模式段局部求解。
[SegmentTimingCache](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_retiming.py)
以完整的不可变运动段和动态profile为精确键，在同一次规划请求中复用时间包络与多项式，
再根据新候选重置段索引、时间起点和累计弧长。局部控制、长度、方向或profile变化就重新求解；
每次取出都是独立副本，最多缓存32段。世界坐标轨迹、模式切换驻留和独立碰撞证书仍逐候选生成、审核。
诊断JSON的 `segment_timing_cache` 直接报告命中、求解和淘汰次数。

流程：有限模式结构候选 → 连续控制/末端投影 → 段级时间求解及复用 → 独立审核 → 最短可执行时间比较。
论文的迭代安全走廊优化方程没有逐式复现；当前局部连续NLP采用距离场引导，显示走廊并不等于NLP约束，
整车安全以独立扫掠为准。这是对可执行分段框架的工程扩展，不声称全局联合最优。

验证命令：`OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'`。
map04真实ROS配对中，重定时耗时从0.807秒降至0.298秒，执行时长均为40.446秒且独立审核通过；
12组共享段重复计时的重定时中位速度约提升2.4倍。这是本次用例结果，不代表全地图总求解速度。
完整 Python 回归151项通过，包含5项新增段级复用合同测试。
详见[指标、图片与测试范围](evaluation/paper12/REPORT.md)。

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

启动脚本会加载 ROS Noetic、构建 `catkin_ws`，再启动 Demo。三个算法分支各自锁定对应后端；切换算法请切换 Git 分支。检测到图形桌面时打开 RViz；无图形桌面时自动关闭 RViz。

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
| `planning_backend:=...` | 当前分支绑定的后端 | 三个算法分支不允许用参数切换到其他后端；传入与分支不符的值会报错。要运行其他后端，请切换到对应分支。 |
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
# 三个算法分支的 run_demo.sh 各自运行绑定后端
git switch mode-path-constant-primitive
bash run_demo.sh --map map01

git switch multimodal-hybrid-astar
bash run_demo.sh --map map01

git switch continuous-mode-path
bash run_demo.sh --map map05 target_v:=1.5 direction_policy:=forward_only

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

The three algorithm branches lock `run_demo.sh` to one backend each:

| Branch | Backend |
|---|---|
| `multimodal-hybrid-astar` | `legacy` |
| `mode-path-constant-primitive` | `mode_path_v1` |
| `continuous-mode-path` | `mode_path_v2` |

Switch branches to compare methods; a mismatched `planning_backend:=...` override is rejected. `main` remains the shared project snapshot. The branch and paper development plan is in [`plan/24_单仓库整合与四方法论文开发计划_20261024.md`](plan/24_单仓库整合与四方法论文开发计划_20261024.md).
