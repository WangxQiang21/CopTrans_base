# expv3

## 本分支：模式搜索与分段二次平滑

本分支 `mode-path-constant-primitive` 运行 `mode_path_v1`。参考论文 08：Zeyu Yang 等，
*A 4WIS Vehicle Trajectory Planning Approach Incorporating Multiple Motion Mode Sequence Decision-Making*，
Automotive Innovation 9:158–174 (2026)，[DOI](https://doi.org/10.1007/s42154-025-00369-5)，
[本地论文](papers/08_Yang_4WIS_Motion_Mode_Sequence_2026.pdf)。
借鉴“多模式图搜索 → 按模式分段优化 → 速度规划”的结构：搜索使用恒定原语提出模式/方向序列，
连续处理围绕已提出的模式段展开。

本次加入 [分段二次平滑](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/segmented_smoothing.py)：
对 Ackermann 段求解曲率偏差与二阶差分的凸二次问题，限制最大曲率、保持段两端轮配置及曲率积分，
然后逐段投影回原共享边界，再重定时和独立验碰。Spin/斜移的模式与方向保持原搜索结果。
粗路径已经满足搜索容差、但不满足发布末端精度时，先执行连续末端投影；通过完整审核后才进入候选池。
这使路径平滑与时间生成在发布前成为一个完整阶段，避免只因粗格点末端误差就丢弃候选。

已可执行的原候选继续参与比较；新平滑候选只有执行时间更短，或近乎等时且共同评分更低时才替换它。
投影、验碰或动态审核失败时保留原候选。默认模式切换时间仍为 0.3 秒。
论文优化的是路径节点，本分支优化曲率控制；速度生成保留现有加速度/jerk合同的连续 S 曲线，
不是对论文 QP 方程和梯形速度公式的逐式复现。

验证命令：`OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p test_segmented_smoothing.py -v`。
同一起终点的曲率振荡定向用例，在受限动态 profile 下执行时长由 8.63 秒降到 6.20 秒；
这是针对性收益，不能代替全地图结论。[指标、图片和真实 ROS 检查](evaluation/paper08/REPORT.md)。
map01 预设任务的真实 ROS 配对也已完成：基线因末端精度拒绝，本分支恢复成功；
投影后候选由 7.235 秒进一步平滑到 7.092 秒，独立扫掠、动态和方向审核通过。Python 全套 150 项测试通过。

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

模式切换时间是车辆配置，不属于某个规划后端：当前 Ranger Mini 2 仿真值统一设为 0.3 秒，来自 [`ranger_mini_2.yaml`](catkin_ws/src/fwis_common/config/ranger_mini_2.yaml)。三个后端在相同配置下使用同一切换事件；受限动态配置只有在转向执行时间更长时才延长事件。搜索评分中的换模惩罚与这段执行时间分开计算。

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
