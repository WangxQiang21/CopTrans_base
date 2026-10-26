# expv3

## 本分支：强化学习模式与路径规划（离线训练开发中）

本分支 rl-mode-path 从 continuous-mode-path 的 c90e981 派生，用于第四种方法的开发。
当前主线改为 **规则生成局部候选，RL 组织模式与路径分支**。独立离线前端、C++ 短程连接适配和残差 PPO 选择器已实现；35 项学习模块检查通过。
规则版完成 20/20 个短程验证任务，但工程地图仅 3/11；512 步 GPU 接入小试尚未超过规则版，完整 RL 规划效果未达标。操作命令见 [rl/README.md](rl/README.md)。
先前的 80 动作自由生成实验保留作历史对照；其最佳模型只有 2/20，不再作为当前优先训练路线。RL 在线前端尚未接入 ROS。

- [完整文献调研与算法架构](plan/rl/01_文献调研与算法架构.md)
- [分阶段执行方案与验收标准](plan/rl/02_执行方案与验收标准.md)
- [基础模块的安装、检查命令与代码说明](rl/README.md)
- [P0/P1 实测结果和图片](evaluation/rl/foundation/REPORT.md)
- [GPU 训练曲线与实际成功/失败路径图](evaluation/rl/training/REPORT.md)
- [规则与学习新前端：逐地图结果及路径图](evaluation/rl/rule_frontend/REPORT.md)

当前框架由规则生成带模式、方向、段长、曲率/角度的局部连接；学习策略选择待探索的路径分支。
候选按需展开，不要求先得到旧 C++ 前端的完整路径。之后复用连续化、边界拟合、重定时、车辆切换配置及整车审核。
旧研究方案与历史实验见调研报告；当前工程分工见执行方案顶部的主线调整。

**目前执行 run_demo.sh 仍运行继承的 mode_path_v2；它可以用于父版本对照，不能当成强化学习结果。**
执行方案 P5 完成后，本分支入口才会绑定拟定的 rl_mode_path 后端。
下方命令说明描述当前继承入口；离线基础检查使用 `bash rl/run_checks.sh --output 新目录 --ros`。
离线数据、训练与模型评估入口已实现；RL 在线推理仍待接入。

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

启动脚本会加载 ROS Noetic、构建 `catkin_ws`，再启动 Demo。三个已实现的算法分支各自锁定对应后端；rl-mode-path 当前继承 v2 入口，学习环境在独立目录中开发。检测到图形桌面时打开 RViz；无图形桌面时自动关闭 RViz。

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

The three implemented algorithm branches lock their launchers to their respective backends. The fourth branch currently contains the RL research and implementation plan:

| Branch | Backend | Status |
|---|---|---|
| multimodal-hybrid-astar | legacy | Implemented |
| mode-path-constant-primitive | mode_path_v1 | Implemented |
| continuous-mode-path | mode_path_v2 | Implemented |
| rl-mode-path | Planned: rl_mode_path | Research plan; current launcher still runs inherited v2 |

Switch branches to compare the implemented methods. The RL branch will become runnable after implementation stage P5. The shared project snapshot is main.
See the [RL research report](plan/rl/01_文献调研与算法架构.md) and [implementation plan](plan/rl/02_执行方案与验收标准.md).
