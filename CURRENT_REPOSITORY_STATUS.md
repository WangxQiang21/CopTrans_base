# expv3_multi 当前仓库状况、源码架构与 AI 优化交接说明

> 本文面向需要接手本仓库、拆解或执行后续优化任务的 AI 编程助手。
> 目标是把“当前源码真正实现了什么、怎样运行、哪些合同不能破坏、哪些文档已经过时”一次说明清楚。
> 本文不是新的算法设计方案，也不把历史实验记录当作本次重新验证的结果。

---

## 0. 文档基线与核验范围

| 项目 | 当前值 |
|---|---|
| 仓库目录 | `/home/wq/whut_thesis/expv3_multi` |
| Git 分支 | `continuous-mode-path` |
| 基线提交 | `956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee` |
| 基线提交标题 | `Unify vehicle execution and add human-labeled ROS data collection` |
| 远程仓库 | `git@github.com:WangxQiang21/CopTrans_base.git` |
| 当前算法后端 | `mode_path_v2` |
| 文档核对日期 | 2026-11-09 |
| 核验方式 | 静态阅读当前源码、消息、launch、YAML、测试清单和项目文档 |
| 本次是否重新构建 | 否 |
| 本次是否重新运行测试 | 否 |
| 本次是否启动 ROS/RViz | 否 |

撰写本文前，上述基线工作区是干净的。本文是新增的说明文档，不修改算法源码。

### 0.1 接手 AI 必须遵守的事实优先级

仓库中有大量历史交接、实验记录和方案文档，部分描述已经落后于当前源码。发生矛盾时，按以下优先级判断：

1. 当前提交中的实际源码和消息定义；
2. 当前 `launch`、车辆 YAML 和运行参数 YAML；
3. 当前 `run_demo.sh`；
4. 根目录 `README.md` 的当前分支说明；
5. `plan/`、`catkin_ws/README.md`、`BRANCH_METHOD.md` 中的历史架构描述；
6. 历史实验数据、截图、耗时和“已通过”结论。

**不要因为某个文档写着“支持”“默认”“已经通过”，就跳过源码核对。**

### 0.2 最重要的文档漂移警告

- 当前 `run_demo.sh` 会把本分支锁定为 `mode_path_v2`，传入 `legacy` 或 `mode_path_v1` 会报错。某些旧文档仍写着可直接通过 `planning_backend:=...` 切换后端，那不是当前一键脚本的实际行为。
- 旧架构文档中的部分搜索预算已经过时。例如当前 `params.yaml` 是 `search.max_iter=450000`、`guided_iter_without_incumbent=360000`、`max_iter_with_incumbent=5000`，不要继续引用旧文档中的 180000/1500 作为当前值。
- 很多文档中的命令仍使用 `/home/wq/whut_thesis/expv3`。当前工作副本是 **`/home/wq/whut_thesis/expv3_multi`**，在这里操作时必须替换路径，避免误改原单车目录。
- `plan/` 中大量“推荐架构”“待完成目标”和“历史修复记录”不是当前能力清单。
- `archive/` 是历史参考代码，不是默认构建代码。

---

## 1. 一句话认识这个仓库

这是一个 **ROS1 Noetic 的单车 4WIS 多模式运动规划实验工作区**：

```text
静态占据栅格 + 单车中心起终点
        ↓
C++ 前端生成多条拓扑路线和 A/O/Spin 粗规划种子
        ↓
Python mode_path_v2 后端生成有限结构候选
        ↓
连续几何处理、终点拟合、局部优化、重定时
        ↓
独立整车扫掠 + 终点 + 方向 + 时间/动态审核
        ↓
从完全通过审核的候选中选解
        ↓
发布车辆中心 Path 和 TimedTrajectory
        ↓
RViz 定时插值回放、诊断、实验采集
```

它当前**不是**：

- 三车或多车协同规划器；
- 实时定位、闭环跟踪或实车控制器；
- 轮速/转角/`cmd_vel` 生成器；
- 动态障碍物规划器；
- 完整的模式—路径—时间全局联合 MINLP；
- 对任意地图和任意起终点都能成功或严格实时的系统；
- 已完成实车全动态标定的规划器。

---

## 2. 名称和版本不要混淆

| 名称 | 含义 |
|---|---|
| `exp` / `expv2` / `expv3` | 历史项目阶段或目录名称 |
| `expv3_multi` | 当前准备用于多车扩展的新工作副本；当前源码仍是单车规划 |
| `fwis_v3_*` | 当前 ROS 包名前缀，不代表存在 `mode_path_v3` |
| `legacy` | 历史基线后端，代码仍保留在当前分支中 |
| `mode_path_v1` | 恒定原语的新主问题后端，代码保留作对照 |
| `mode_path_v2` | 当前默认连续模式路径后端 |
| `A` / `ACKERMANN` | 前后阿克曼运动，消息枚举值为 0 |
| `O` / `OBLIQUE` | 斜移/横移，消息枚举值为 1 |
| `S` / `Spin` / `SPIN` | 原地旋转，消息枚举值为 2 |
| `R01`…`R20` | 历史开发迭代或实验记录编号，不是 20 个后端 |
| `default` profile | 当前运动学演示时间策略，不是另一套规划后端 |
| `restricted` profile | 更严格的时间缩放对照配置，不等于实车完整标定 |

---

## 3. 顶层目录及实际作用

```text
expv3_multi/
├── CURRENT_REPOSITORY_STATUS.md  本文：当前源码与 AI 交接说明
├── README.md                     使用说明、当前分支说明、采集说明
├── BRANCH_METHOD.md              分支方法和部分历史性能记录
├── run_demo.sh                   当前唯一推荐的一键启动入口
├── catkin_ws/
│   ├── src/                      当前参与构建的 7 个 ROS 包
│   ├── build/                    catkin 生成物，不应手工修改
│   └── devel/                    构建生成的环境和入口，不应手工修改
├── maps/                         当前地图、场景来源、预设任务和地图索引
├── tests/                        Python 单元测试、ROS smoke/probe、结果捕获
├── tools/                        回归、探针、诊断、地图处理、数据复审
├── evaluation/                   精简指标、图片、报告模板和实验报告
├── plan/                         历史方案、交接、迭代记录；不参与运行
├── archive/                      旧 ROS 包和 expv2 原型；不参与默认构建
├── papers/                       论文资料
├── qna/                          研究问答资料
└── .gitignore
```

### 3.1 修改代码时的目录边界

- 正式功能修改通常只发生在 `catkin_ws/src/`、`tests/`、`tools/`、必要的 `maps/` 或 `run_demo.sh`。
- 不要直接修改 `catkin_ws/build/` 和 `catkin_ws/devel/`。
- 不要把 `archive/` 中的文件当作当前模块来修复或优化。
- 不要为了“整理结构”把历史文档、算法修改和大规模重命名混在同一任务里。
- 大规模采集结果默认写到仓库外，不应把大量实验产物提交进源码仓库。

---

## 4. 构建、运行和启动链

### 4.1 运行依赖

当前项目文档和包配置要求：

- Ubuntu 环境中的 ROS1 Noetic；
- `catkin_make`；
- C++17 编译环境；
- Python 3；
- NumPy；
- SciPy；
- 连续数值优化路径使用 CasADi/IPOPT，部分路径有 SciPy 回退；
- RViz 仅用于可视化，不是无界面规划的必要条件；
- legacy 的可选 Bonmin 功能有额外插件/求解器依赖，但当前 `joint.enabled=false`，不是 v2 默认主链依赖。

`fwis_v3_optimizer/package.xml` 明确声明了 NumPy 和 SciPy 运行依赖；CasADi 的实际导入位于连续优化代码中。接手环境仍应实际检查依赖版本，不要只根据文档假定环境完全一致。

### 4.2 推荐启动命令

在当前副本中运行：

```bash
cd /home/wq/whut_thesis/expv3_multi

# 默认 map16 地下车库，自动使用预设任务
bash run_demo.sh

# 列出可运行地图
bash run_demo.sh --list-maps

# 指定地图
bash run_demo.sh --map map04

# 指定地图比例，不开 RViz
bash run_demo.sh --map map04 --map-scale 0.5 rviz:=false

# 手动通过 RViz 设置起终点
bash run_demo.sh --map map04 auto_goal:=false

# 只允许正向运动
bash run_demo.sh --map map05 direction_policy:=forward_only

# 覆盖目标速度
bash run_demo.sh --map map05 target_v:=1.5
```

### 4.3 `run_demo.sh` 实际执行的步骤

1. 根据脚本自身位置确定仓库根目录，因此不依赖调用时的当前目录。
2. 解析 `--map`、`--map-scale`、`--collect-data`、`--data-root` 等选项。
3. 通过 `maps/map_index.tsv` 兼容地图编号、完整目录名和旧名称。
4. 如需缩放，在 `/tmp` 创建临时地图副本，围绕地图中心修改坐标配置，不重采样栅格。
5. 注册退出清理，删除临时缩放地图目录。
6. 强制检查后端：当前分支只允许 `mode_path_v2`。
7. 检查并加载 `/opt/ros/noetic/setup.bash`。
8. 清理已有 ROS/CMake/Python 相关环境变量，降低外部 overlay 污染。
9. 执行 `catkin_make -C catkin_ws -j4`。
10. 加载当前 `catkin_ws/devel/setup.bash`。
11. 如存在可选 Bonmin seed 插件，则设置 `CASADIPATH`。
12. 配置采集目录、地图自动目标、场景和 RViz。
13. 调用 `tools/describe_run_context.py` 记录本次运行上下文。
14. 最终执行 `roslaunch fwis_v3_demo demo.launch ...`。

### 4.4 手动构建和 launch

```bash
cd /home/wq/whut_thesis/expv3_multi
source /opt/ros/noetic/setup.bash
catkin_make -C catkin_ws -j1
source catkin_ws/devel/setup.bash
roslaunch fwis_v3_demo demo.launch
```

直接 `roslaunch` 不会经过 `run_demo.sh` 的分支后端锁定、地图便捷解析、默认比例、运行上下文打印等逻辑，因此两种启动方式的实际默认参数可能不同。实验记录必须说明启动方式。

### 4.5 launch 启动的节点

`catkin_ws/src/fwis_v3_demo/launch/demo.launch` 启动：

- `fwis_map_publisher`：`map_publisher.py`；
- `fwis_v3_planner`：唯一 C++ 前端可执行节点；
- `fwis_v3_optimizer`：Python 后端；
- `fwis_data_collector`：仅 `collect_data=true` 时启动，且为 `required=true`；
- `fwis_v3_playback`：轨迹回放；
- `fwis_scene_hints`：场景提示和预设目标；
- `fwis_v3_auto_goal`：仅 `auto_goal=true` 时启动；
- `fwis_parking_lines`：停车线显示；
- `rviz`：仅 `rviz=true` 时启动。

planner、optimizer、playback 都加载公共车辆 YAML 和算法参数 YAML。launch 中显式传入的覆盖参数在 YAML 加载后设置。

---

## 5. 七个活跃 ROS 包详细说明

## 5.1 `fwis_v3_msgs`：前后端接口合同

位置：`catkin_ws/src/fwis_v3_msgs/msg/`

当前只有五个核心消息，均没有 `vehicle_id` 或 `robot_id`。

### `ModePrimitive.msg`

```text
uint8 ACKERMANN=0
uint8 OBLIQUE=1
uint8 SPIN=2
uint8 mode
float64 amount
float64 control
```

含义：

- A：`amount` 是带方向的弧长/平移量，`control` 是中心曲率；正负 `amount` 区分前进和倒车。
- O：`amount` 是带方向的平移量，`control` 是斜移方向相关角度，允许范围由后端审核约束到 `[-π/2, π/2]` 附近。
- Spin：`amount` 是带符号旋转角，`control` 不承担平移控制含义。

消息本身不执行这些语义检查，真正的合法性检查在前端和 Python 审核器中。

### `PlanSeed.msg`

```text
std_msgs/Header header
uint32 plan_id
geometry_msgs/Pose2D start_center
geometry_msgs/Pose2D goal_center
ModePrimitive[] primitives
```

- 一个消息只包含一个起点、一个终点和一条主动作序列。
- 空 `primitives` 可表示前端尚未找到有效主种子或失败状态，后端会结合前端失败信息处理。
- 这是规划初值，不是最终可执行轨迹。

### `PlanSeedCandidates.msg`

```text
std_msgs/Header header
uint32 plan_id
ModePrimitive[] proposal_primitives
string proposal_source
```

- 用于传递辅助候选。
- 不含独立起终点，需要与同一 `plan_id` 的主请求上下文对应。
- `proposal_source` 标识来源。
- 注释和代码都明确：这些原语是 provisional initializer，必须经过共享动作策略修复和完整审核，不能直接当最终轨迹。

### `TimedPoint.msg`

```text
duration time_from_start
geometry_msgs/Pose2D center_pose
float64 v
uint8 ACKERMANN=0
uint8 OBLIQUE=1
uint8 SPIN=2
uint8 mode
```

- 每个点带相对轨迹起点的时间。
- `center_pose` 是车辆中心位姿，不是后轴中心。
- `v` 是车辆中心平移速度，带方向语义；Spin 时平移速度为 0。
- `mode` 与该时间样本对齐，用于显示和回放。

### `TimedTrajectory.msg`

```text
std_msgs/Header header
uint32 plan_id
TimedPoint[] points
```

- 一条消息只承载一条规划轨迹。
- 空点列用于清除/等待等状态语义的一部分，不能当成一条可执行轨迹。
- 不包含车辆尺寸、地图哈希或动态限值；这些信息在质量报告和采集记录中另行保存。

### 多车扩展直接相关的接口事实

- `plan_id` 是请求编号，不是车辆编号。
- 当前没有消息级车辆身份、优先级、协同请求 ID、其他车轨迹或时间占用约束。
- 若后续给消息增加字段，必须同步检查 C++ 构造、Python 序列化、采集、回放、测试和所有诊断工具。

---

## 5.2 `fwis_common`：公共 C++ 几何与车辆依赖

位置：`catkin_ws/src/fwis_common/`

主要文件：

| 文件 | 当前作用或注意事项 |
|---|---|
| `include/fwis_common/grid_map.hpp` | 占据栅格封装，是当前前端的重要公共依赖 |
| `include/fwis_common/reeds_shepp.hpp` | Reeds-Shepp 连接实现，用于 A 类候选连接 |
| `include/fwis_common/vehicle.hpp` | 车辆相关公共定义 |
| `include/fwis_common/kinematics.hpp` | 运动学相关定义 |
| `include/fwis_common/modes.hpp` | 模式相关定义，带历史代码性质 |
| `include/fwis_common/safe_corridor.hpp` | 走廊相关公共代码，带历史代码性质 |
| `include/fwis_common/velocity_heuristic.hpp` | 速度启发相关代码，不能据此认定当前 v2 使用该策略 |
| `config/ranger_mini_2.yaml` | 当前车辆尺寸和共享执行限值的关键配置 |

不要根据“头文件存在”推断对应功能在当前 v2 主链中一定启用。优化任务应先追踪实际 include 和调用点。

---

## 5.3 `fwis_map_server`：地图、场景和静态显示

位置：`catkin_ws/src/fwis_map_server/`

主要脚本：

| 脚本 | 作用 |
|---|---|
| `scripts/map_publisher.py` | 读取 PGM/YAML，发布 `/map`、`/start_pose` 和一次静态初始 TF |
| `scripts/scene_hints.py` | 场景文字、预设目标和 RViz 提示 |
| `scripts/parking_lines.py` | 根据场景配置发布停车线 Marker |
| `scripts/scale_scene.py` | 场景缩放辅助 |
| `scripts/gen_map.py` | 地图生成辅助 |

`map_publisher.py` 的关键行为：

- 输出 `nav_msgs/OccupancyGrid`；
- 地图 `frame_id` 固定为 `map`；
- `/map` 使用 latched 发布；
- `/start_pose` 来自地图预设；
- 发送一次静态 `map → base_link` TF，表示演示初始位姿。

**该 TF 不是实时定位，也不是车辆执行过程中的状态反馈。**

---

## 5.4 `fwis_v3_demo`：系统集成与配置

位置：`catkin_ws/src/fwis_v3_demo/`

```text
fwis_v3_demo/
├── launch/demo.launch          节点组合、YAML 加载、显式参数覆盖
├── config/params.yaml          当前默认算法和搜索配置
├── config/params_restricted.yaml  受限动态对照
├── scripts/auto_goal.py        读取 scene_meta.yaml 并自动发送目标
└── rviz/demo.rviz              固定 map 坐标系和单车显示配置
```

`auto_goal.py` 只负责自动目标输入，不负责路径搜索或协同决策。

RViz 当前围绕单车配置，显示地图、粗路径、最终路径、模式条、走廊、单车车体、朝向和场景标注。多车启动三套节点时，现有全局 topic 和 Marker 标识会发生冲突。

---

## 5.5 `fwis_v3_planner`：C++ 前端搜索

核心文件：`catkin_ws/src/fwis_v3_planner/src/planner_node.cpp`，当前约 2128 行。

构建：

- 使用 C++17；
- 生成唯一可执行文件 `fwis_v3_planner_node`；
- 依赖 `roscpp`、标准 ROS 消息、`tf`、`fwis_common` 和 `fwis_v3_msgs`。

### 输入

- `/map`：静态占据栅格；
- `/start_pose`：地图预设起点；
- `/initialpose`：RViz 2D Pose Estimate；
- `/move_base_simple/goal`：RViz 或自动目标；
- 其他场景预设目标输入经场景节点转换后进入同一主链。

起点输入要求 `map` 坐标系。节点只维护一张地图、一个当前起点、一个当前目标和当前请求编号等单车状态。

### 主要函数和职责

| 函数/区域 | 作用 |
|---|---|
| `PlannerNode::on_goal()` | 一次规划主入口，组织搜索、状态清理和结果发布 |
| `pose_clear()` | 对单个位姿进行整车 OBB 与占据格碰撞检查 |
| `sparse_mixed_route()` | 构造稀疏 A/O/Spin 混合路线图候选 |
| `oblique_route()` | 生成二维拓扑和斜移相关候选 |
| `search()` | 组织引导搜索、回退和方向竞争 |
| 结果发布区域 | 发布粗 Path、主 `PlanSeed`、辅助 `PlanSeedCandidates` 和诊断 Marker |

### 当前前端搜索的实质

前端不是只运行一次普通 Hybrid A*，而是组合：

1. 地图和起终点合法性检查；
2. 多个净空引导/二维拓扑路线；
3. 稀疏航点上的 A/O/Spin 候选；
4. 精确动作重放和整车扫掠检查；
5. 有 incumbent 后的限额改进搜索；
6. 没有可行 incumbent 时更大的恢复预算；
7. 正向、反向及特殊候选竞争；
8. 将一条主种子和可选辅助种子交给 Python。

前端动作代价使用 `cost` 下的换模、倒车、方向、曲率、斜移变化和 Spin 等工程权重。它与后端 `quality.cost` 有关联，但不是同一个配置节，也不能只看其中一个就声称理解完整选路目标。

### 当前重要搜索预算（以 `params.yaml` 为准）

| 参数 | 当前值 | 解释 |
|---|---:|---|
| `search.max_iter` | 450000 | 总体搜索预算相关上限 |
| `search.guided_iter_without_incumbent` | 360000 | 没有安全 incumbent 时的引导预算分配 |
| `search.reverse_competition_max_iter` | 90000 | 独立方向竞争预算 |
| `search.reverse_fallback_max_iter` | 30000 | 反向回退相关预算 |
| `search.reverse_fallback_budget_s` | 3.0 s | 反向回退时间预算 |
| `search.max_iter_with_incumbent` | 5000 | 已有安全解后的改进预算 |
| `search.route_graph_max_expansions` | 8000 | 路线图扩展上限 |
| `search.step_len` | 0.20 m | 搜索步长 |
| `search.xy_res` | 0.10 m | 位置离散分辨率 |
| `search.yaw_res` | 0.1745329 rad | 航向离散分辨率，约 10° |
| `search.goal_xy_tol` | 0.16 m | 前端目标容差，不是最终发布容差 |
| `search.goal_yaw_tol` | 0.14 rad | 前端目标容差，不是最终发布容差 |

这些是搜索控制参数，不是系统级严格实时保证。

### 前端不做什么

- 不负责最终连续时间轨迹；
- 不负责最终 2 cm/1° 终点审核；
- 不把辅助种子当成认证结果；
- 不输出真实车辆控制命令；
- 不处理其他车辆的动态占用。

---

## 5.6 `fwis_v3_optimizer`：Python 后端、审核与采集

这是当前最复杂、最关键的包。

```text
fwis_v3_optimizer/
├── scripts/
│   ├── optimizer_node.py          ROS 参数/消息适配、后端分流、发布
│   └── collect_node.py            人工采集会话和本地网页服务
├── src/fwis_v3_optimizer/
│   ├── core.py
│   ├── mode_master.py
│   ├── continuous_geometry.py
│   ├── continuous_optimizer.py
│   ├── retiming.py
│   ├── continuous_retiming.py
│   ├── vehicle_execution.py
│   ├── mode_switch_timing.py
│   ├── quality.py
│   ├── geometry_incumbent.py
│   ├── legacy_geometry_provider.py
│   ├── planning_display.py
│   ├── diagnostics.py
│   └── collection.py
├── patches/                       可选求解器补丁
└── tools/build_bonminseed_plugin.sh  可选 legacy 插件构建
```

### 代码规模与维护风险

当前大致行数：

| 文件 | 约行数 | 维护注意点 |
|---|---:|---|
| `core.py` | 4095 | 基础运动学、几何、碰撞、走廊和 legacy 算法混合 |
| `optimizer_node.py` | 2943 | ROS 适配、legacy 大流程、v1/v2 发布与诊断混合 |
| `mode_master.py` | 2317 | 新后端候选主问题，职责很多 |
| `quality.py` | 1267 | 审核、指标、缓存和安全检查 |
| `continuous_retiming.py` | 851 | 连续时间求解 |
| `continuous_geometry.py` | 798 | 连续几何表示和拟合 |
| `continuous_optimizer.py` | 795 | 固定结构局部 NLP |

**不能因文件名叫 `core.py` 就整体删除其中的“旧代码”；v2 仍复用其中大量基础函数。**

### 各模块的真实职责

| 模块 | 当前职责 |
|---|---|
| `core.py` | `Pose`、`Action`、`Grid`、精确动作传播、车体几何、SAT/距离场、走廊、旧局部优化、legacy `joint_refine` 等 |
| `mode_master.py` | v1/v2 候选生成、去重、预算、结构邻域、筛选、重定时、审核、incumbent 保留和最终选解 |
| `continuous_geometry.py` | `ControlKnot`、`MotionBlock`、`ContinuousPlan`；连续积分、控制过渡、边界/终点/末段拟合 |
| `continuous_optimizer.py` | 固定模式/方向/块结构内的局部连续控制 NLP；不是全局混合整数规划 |
| `retiming.py` | `MotionLimits`、恒定原语重定时、公共动态工具、`PrimitiveTimingCache` |
| `continuous_retiming.py` | v2 平移/Spin 时间相位、加减速、jerk、模式事件和 `SegmentTimingCache` |
| `vehicle_execution.py` | 三个分支共享的车辆执行合同、精确恒定控制接入、显示采样复核 |
| `mode_switch_timing.py` | 模式切换事件及驻留时间计算 |
| `quality.py` | 几何、终点、扫掠、方向、时间/动态和质量指标的独立审核 |
| `geometry_incumbent.py` | 保存“几何通过但执行未通过”的证据，防止把几何可行误报为时间轨迹可行 |
| `legacy_geometry_provider.py` | 有界借用 legacy 锚点、局部修复等几何候选，再进入共同审核；不等于运行完整 legacy 流程 |
| `planning_display.py` | 规划状态和显示年龄等辅助状态，不负责安全认证 |
| `diagnostics.py` | 诊断 JSON 原子写入 |
| `collection.py` | 采集记录、序列化等公共支持 |

### 后端分流

`OptimizerNode.on_seed()`：

1. 丢弃小于当前编号的旧 `plan_id`；
2. 接受当前或更新请求并清理旧显示；
3. 处理空种子和前端失败；
4. 检查地图是否存在；
5. 从消息恢复 `start`、`goal` 和 `Action[]`；
6. 对 v1/v2 最多等待约 0.15 s，尝试取得同 `plan_id` 的辅助候选；
7. 检查方向策略；
8. 发布粗模式条；
9. v1/v2 进入 `run_mode_path()` 并直接返回；
10. 只有 `legacy` 才继续后面的大段旧流程。

特殊来源 `untrusted_shunting_search` 可以作为受控初值交给共享 Python 修复，同时保留 C++ 严格主种子作为独立回退候选。这个机制不意味着未审核候选可以直接发布。

### v2 主入口

`run_mode_path()` 为每一次请求创建新的 `TrajectoryEvaluator`：

```python
TrajectoryEvaluator(
    self.grid, start, goal, self.quality_limits,
    self.length, self.width,
    .02, math.radians(1), self.quality_profile,
    allow_reverse=self.allow_reverse,
    allow_shunting=self.allow_shunting)
```

因此：

- 审核器绑定一张固定地图；
- 绑定一个起点和一个目标；
- 最终发布终点容差为 **0.02 m / 1°**；
- 缓存是单次请求/固定地图范围内的缓存；
- 当前没有多车共享一个请求或一个审核器的概念。

随后调用：

```python
ModePathMaster(evaluator, self.mode_master_config).solve(
    solver_seed,
    auxiliary_seeds=solver_auxiliary_seeds)
```

### `ModePathMaster.solve()` 的候选处理

v2 单个候选的典型路径：

```text
Action[] / 辅助 Action[]
        ↓
动作策略和揉库约束检查、候选签名去重
        ↓
plan_from_actions() 生成 ContinuousPlan
        ↓
fit_mode_boundaries() 或 fit_continuous_endpoint()
        ↓
连续几何扫掠初筛
        ↓
必要时保留安全前缀并进行末段修复
        ↓
retime_continuous()
        ↓
evaluate_continuous_geometry()
        ↓
几何、终点、方向、时间/动态全部通过才进入可执行候选池
```

在此之外，主问题还会按预算尝试有限结构邻域、锚点/模式替换、局部修复和少量 NLP 精修。当前不是无限搜索，也不是枚举所有可能同伦类。

### 当前选解目标

`quality.search.selection_objective=minimum_time`：

- 主目标是通过完整认证后的实际执行时间；
- 时间接近时再比较质量分数等次级指标；
- 已认证的最快 incumbent 始终保留资格；
- 没通过时间/动态审核的几何候选不能击败可执行 incumbent；
- 不应把某个候选的 NLP 求解器返回值直接当作系统最终输出。

### 当前后端预算

| 参数 | 当前值 | 注意事项 |
|---|---:|---|
| `quality.search.budget_s` | 8.0 s | 基础主问题预算 |
| `quality.search.validation_reserve_s` | 0.5 s | 最终验证预留 |
| `quality.search.candidate_evaluation_reserve_s` | 2.0 s | 避免在来不及审核时继续开新候选 |
| `quality.search.nlp_time_s` | 2.0 s | NLP 时间配置 |
| `quality.search.max_nlp_calls` | 1 | 新主问题 NLP 调用上限 |
| `quality.search.max_candidates` | 20 | 候选预算 |
| `quality.search.max_structural_expansions` | 2 | 有可行解后的结构邻域扩展上限 |
| `quality.search.max_stagnant_expansions` | 1 | 无收益扩展停滞控制 |
| `quality.search.mode_anchor_min_switches` | 12 | 高换模候选触发额外锚点预算的门槛 |
| `quality.search.mode_anchor_extra_budget_s` | 15.0 s | 满足触发条件时的额外预算 |
| `quality.search.legacy_geometry_fallback_s` | 1.5 s | 无可执行 incumbent 时的几何恢复预算 |
| `quality.search.sampling_dt` | 0.05 s | 主候选时间采样配置，不是最终所有发布的固定 dt |

这些预算是协作式搜索控制，不应写成“任何情况下后端严格在某毫秒内返回”的硬实时保证。正在执行的数值求解、审核和 Python 回调仍需实测。

### 连续 NLP 的真实边界

- 连续 NLP 优化固定结构内部的控制量；
- 它不负责全局选择所有模式序列；
- 当前默认 minimum time + `mode_switch_only` 下，某些固定结构 NLP 可以跳过；
- 跳过 NLP 不代表跳过几何构造、端点拟合、重定时和审核；
- `joint.enabled=false`，legacy Bonmin 不是当前 v2 默认流程；
- 不能将局部 NLP、候选邻域或旧 Bonmin 描述成“已经实现全局联合最优”。

---

## 5.7 连续几何与时间求解合同

### 连续几何表示

`ContinuousPlan` 由多个运动块组成。平移块通过控制节点描述控制随弧长的变化，Spin 块描述原地旋转。

关键对象：

- `ControlKnot`：弧长位置、控制值及导数；
- `MotionBlock`：模式、方向、长度、控制节点、Spin 角等；
- `ContinuousPlan`：起点、运动块序列和目标。

“连续”不表示每个几何控制节点都停车。相反，当前设计允许普通控制节点处速度和加速度连续跨段。

### 时间策略

`continuous_retiming.py` 的默认策略：

- 平移使用分段五次时间规律；
- 起点和终点速度为 0；
- 同一运动段内部根据速度、加速度和 jerk 限值求解；
- 普通控制节点不额外设置静止驻留；
- 模式变化插入静止重配置事件；
- 同模式换向可以到零速，但当前合同不允许额外驻留；
- Spin 使用独立角速度、角加速度和角 jerk 约束；
- 模式切换驻留包含设置时间，不是“0.3 s 驻留之外再加 0.15 s 设置”。

### `SegmentTimingCache`

当前分支的重要性能优化：

- 以不可变运动段和动态 profile 的完整内容为键；
- 在同一次 `ModePathMaster.solve()` 请求中复用段级时间包络和多项式；
- 取出时重置段索引、时间起点和累计弧长等上下文；
- 控制、长度、方向或 profile 变化就必须重新求解；
- 每次取出是独立副本；
- 最大缓存 32 段；
- 世界坐标轨迹、模式切换事件、独立碰撞与执行审核仍逐候选进行；
- 诊断 JSON 的 `segment_timing_cache` 记录命中、求解和淘汰情况。

不要在优化缓存时只使用段索引作为 key，也不要跨不同地图、车辆限值或请求盲目共享缓存。

### 显示采样不是安全捷径

`vehicle_execution.certify_display_sampling()`：

- 对实际将发布和插值显示的时间样本再次检查相邻位姿之间的扫掠；
- 必要时逐步细化空间和时间采样，最多 4 级；
- 不改变几何；
- 不允许改变执行总时长；
- 重采样后必要时重新执行完整审核；
- 无法通过则拒绝发布，而不是只关闭显示。

---

## 5.8 独立质量与安全审核

核心类：`quality.py` 中的 `TrajectoryEvaluator`。

### 审核层次

`evaluate_continuous_geometry()` 检查：

1. 输入确为 `ContinuousPlan`；
2. plan 的起点/目标与本次请求一致；
3. 非空且数值有效；
4. 模式、方向、曲率和斜移角合法；
5. 禁止的揉库/换向结构不存在；
6. 同模式换向边界没有不允许的转向跳变；
7. 起点无碰撞；
8. 各运动块进行连续离散化扫掠；
9. 实际终点满足严格容差；
10. 有时间轨迹时，再检查时间表示版本、动态 profile、样本单调性、起终速度、驻留合同、模式事件和几何一致性；
11. 需要时计算质量指标和软评分。

### 审核状态不能混用

- `GEOMETRY_VALID` / 几何通过：只说明空间几何满足相应检查；
- `FEASIBLE`：几何和相应时间/动态检查均通过；
- `INVALID` / `REJECTED`：不能发布为成功可执行轨迹；
- `GEOMETRY_ONLY`：可以显示空间路径作为诊断，但不发布未认证的时间轨迹；
- `score_status=DEFERRED`：严格几何和时间已经通过，但软质量分数延后计算，不等于没有审核。

### 终点容差有多层

| 层次 | 当前典型值 | 用途 |
|---|---|---|
| 前端 `search.goal_xy_tol` / `goal_yaw_tol` | 0.16 m / 0.14 rad | 粗搜索接受范围 |
| 连续端点拟合内部目标 | 常见为 0.015 m / 0.75°，部分修复更严格 | 给最终审核预留数值余量 |
| `run_mode_path()` 创建最终 evaluator | 0.02 m / 1° | 最终发布审核合同 |

**不要为了改善成功率，把前端容差、拟合容差和最终审核容差混为一谈或一起放宽。**

### 碰撞与扫掠

- 使用旋转整车 OBB 与占据格 SAT/距离索引，不是只检查车中心点；
- 物理车长宽与安全车长宽是不同概念；
- `collision.margin=0.04` 在报告中对应总尺寸增量，即约每侧 0.02 m，不是每侧额外 0.04 m；
- `quality.cost.sweep_spacing=0.005` 是 5 mm 扫掠相关配置；
- 为覆盖相邻采样位姿之间的运动，还会进行与角点运动相关的膨胀/加密；
- 质量净空指标使用较稀的 `metric_spacing=0.01`，安全扫掠仍使用更密的合同；
- 不能因为质量指标采样较稀，就把安全审核也改为 1 cm。

### 走廊的真实作用

当前报告明确写着：

- 走廊/特征圆中心区域主要用于显示和候选引导；
- `used_in_continuous_nlp=False`；
- 特征点走廊不提供整车安全保证；
- 最终安全证书是独立整车扫掠；
- RViz 中显示一个“安全走廊框”不等于该框已经成为连续 NLP 的硬约束。

---

## 5.9 结果发布、诊断和失败行为

成功时 `optimizer_node.py`：

1. 保存获选动作、连续计划、审核和时间报告；
2. 构造车辆执行合同及哈希；
3. 生成最终空间路径和走廊显示记录；
4. 复核实际显示采样；
5. 发布模式条、走廊 Marker、`TimedTrajectory` 和 `Path`；
6. 持久化质量报告；
7. 发布 `/plan/quality_report` JSON；
8. 发布成功状态摘要。

最终 `Path` 的每个 pose 使用对应时间样本的时间戳；`TimedTrajectory` 逐点写入：

```text
time_from_start, x, y, theta, v, mode
```

失败时：

- 保存失败阶段、原因、地图签名、输入种子和耗时等；
- 发布失败质量报告；
- 清理旧轨迹，避免新目标继续显示旧成功结果；
- 若只有几何 incumbent，可发布明确标注为 `geometry_only_not_executable` 的空间 Path；
- 不把几何证据包装成成功的可执行时间轨迹。

---

## 5.10 `fwis_v3_playback`：定时显示，不是控制器

核心文件：`catkin_ws/src/fwis_v3_playback/scripts/playback.py`。

输入：

- `/plan/timed_trajectory`；
- `/start_pose`；
- `/initialpose`；
- `/plan/quality_report`。

输出：

- `/robot/marker`；
- `/robot/heading_marker`；
- `/robot/hud`。

行为：

- 只保存一条当前轨迹；
- 使用消息 `header.stamp` 和每点 `time_from_start` 计算回放位置；
- 每 0.04 s 触发一次显示 Timer，约 25 Hz；
- 位置、速度和航向在相邻时间样本间插值；
- 航向插值处理角度环绕；
- 轨迹结束后保留最后位姿；
- 失败或仅几何报告可清除当前时间轨迹；
- 不重新计算速度，不重新规划，不做碰撞检查，不发布执行器命令。

单车假设非常直接：车辆 Marker 使用固定 `ns="v3_vehicle"`、`id=0`，HUD 和朝向也使用固定 namespace/ID。多车直接运行会相互覆盖。

---

## 6. 主要 ROS 话题与数据流

## 6.1 输入和规划话题

| Topic | 类型/来源 | 作用 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 当前唯一静态地图 |
| `/start_pose` | `geometry_msgs/PoseStamped` | 地图预设起点 |
| `/initialpose` | `PoseWithCovarianceStamped` | RViz 手动起点 |
| `/move_base_simple/goal` | `PoseStamped` | RViz/自动目标 |
| `/preset_goal` | `std_msgs/Int32` | 场景预设目标入口 |
| `/plan/seed` | `PlanSeed` | C++ 主规划种子 |
| `/plan/seed_candidates` | `PlanSeedCandidates` | 辅助初值，不是认证轨迹 |
| `/plan/path_coarse` | `nav_msgs/Path` | 粗路径显示 |
| `/plan/path` | `nav_msgs/Path` | 最终或明确标注的几何路径 |
| `/plan/timed_trajectory` | `TimedTrajectory` | 最终车辆中心时间轨迹 |
| `/plan/quality_report` | `std_msgs/String` | JSON 格式审核、选解和性能报告 |
| `/plan/search_status` | 状态字符串 | 前端搜索状态 |
| `/plan/status` | 状态字符串 | 后端状态和失败原因 |

## 6.2 显示话题

| Topic | 作用 |
|---|---|
| `/plan/mode_strip_coarse` | 粗解 A/O/Spin 模式色带 |
| `/plan/mode_strip` | 最终时间轨迹模式色带 |
| `/plan/corridor_markers` | 走廊、物理车体和安全尺寸显示 |
| `/robot/marker` | 单车车体回放 |
| `/robot/heading_marker` | 单车朝向箭头 |
| `/robot/hud` | 模式、速度、plan_id 和状态文字 |

## 6.3 当前不存在的输入

主规划链没有订阅：

- 实时 `/odom`；
- 实时定位位姿；
- 其他车辆轨迹；
- 动态障碍物；
- 多车优先级/让行状态；
- 控制跟踪误差；
- 执行器反馈。

因此当前“一次规划”本质上是给定静态地图和起终点后的离线/演示型参考轨迹生成。

---

## 7. 坐标、车辆和运动学合同

### 7.1 坐标合同

- 规划位姿是前后轴中心的中点，即车辆中心；
- RViz 起点、目标和消息中的 `start_center`、`goal_center` 都使用同一中心定义；
- 主坐标系为 `map`；
- 输出的 `theta` 按时间连续展开，用于避免跨 `±π` 的错误显示；
- 车体尺寸不是车中心点或圆点近似；
- 最终轨迹不是轮速、四轮转角或底盘控制命令。

### 7.2 默认 Ranger Mini 2 参数

来自 `fwis_common/config/ranger_mini_2.yaml`：

| 参数 | 当前默认值 |
|---|---:|
| 车长 | 0.738 m |
| 车宽 | 0.500 m |
| 车高 | 0.338 m |
| 轴距 | 0.494 m |
| 轮距 | 0.364 m |
| Ackermann 最小转弯半径 | 0.810 m |
| 支持 Spin | true |
| 模式切换总驻留 | 0.3 s |
| 其中设置时间字段 | 0.15 s，已包含在 0.3 s 内 |
| 目标/最大中心速度 | 2.0 m/s |
| 平移加速度上限 | 0.6 m/s² |
| 平移 jerk 上限 | 2.0 m/s³ |
| Spin 角速度上限 | 0.8 rad/s |
| Spin 角加速度上限 | 0.8 rad/s² |
| Spin 角 jerk 上限 | 3.0 rad/s³ |
| 初始模式 | Ackermann |
| 初始/最终速度 | 0 |

部分轮子转向速率、轮速等字段用于兼容、诊断或受限 profile，不应全部解释为默认演示策略下的主动速度限制。

### 7.3 默认 profile 与 restricted profile

默认：

```text
profile = kinematic_sim_mode_switch_only_v2
translation_speed_policy = mode_switch_only
```

含义：

- 平移速度、加速度和 jerk 是主要硬时间约束；
- Spin 使用独立角动态限值；
- 同模式内不因曲率主动降低目标速度；
- 转向、轮速、横向/法向等需求会计算和记录；
- 默认 profile 不把这些需求全部转成平移速度上限；
- 它不是实车完整动力学或执行器标定结果。

`params_restricted.yaml` 用于更严格时间缩放对照。两种 profile 的执行合同和结果不能直接混排为同一动态条件下的性能。

---

## 8. 配置文件的归属与易混点

## 8.1 配置加载

- `ranger_mini_2.yaml`：车辆几何、共享执行限值和默认 profile；
- `params.yaml`：后端、方向、搜索、候选、成本、碰撞、legacy 等；
- `params_restricted.yaml`：受限动态对照；
- launch 显式参数最后覆盖 YAML；
- planner、optimizer、playback 在各自私有节点命名空间读取自己需要的字段。

修改一个节点实际不读取的参数不会生效。接手任务应使用参数读取点或运行时 `rosparam` 核实，而不是只改 YAML 后假设全系统生效。

## 8.2 关键配置节

| 配置节 | 主要作用 |
|---|---|
| `planning` | 当前后端、方向策略、是否允许普通揉库 |
| `search` | C++ 前端离散搜索、恢复、路线图和预算 |
| `quality.search` | v1/v2 `ModePathMaster` 候选和审核预算 |
| `quality.cost` | 质量指标、软成本权重、扫掠/指标采样 |
| `quality.limits` | 速度、加速度、jerk、Spin 和执行 profile；当前主要由公共车辆 YAML 提供 |
| `cost` | 前端/旧动作序列代价，换模、倒车、曲率、斜移等 |
| `collision` | 车体 margin、扫掠保护和各种搜索采样 |
| `optimization` | 局部 IPOPT 迭代等 |
| `repair` / `endpoint_replan` | 旧流程和局部修复/末段重规划相关 |
| `joint` | legacy 可选 Bonmin MINLP；当前 disabled |
| `mode_master` | legacy 局部模式邻域相关，不等于 `quality.search` |
| `time` | 旧评分尺度兼容字段，不再直接决定当前实际发布时间 |
| `visualization` | 诊断区域显示开关 |

## 8.3 两套 cost 不能混淆

- `cost`：主要服务于前端动作搜索和旧序列评分；
- `quality.cost`：服务于新后端的质量评价、认证后指标和次级比较；
- 当前主选解目标是 `minimum_time`，所以并不是所有 `quality.cost` 权重都始终作为主目标直接比较；
- 改一个权重后，必须说明它影响了哪一层，以及最终候选是否真的发生变化。

## 8.4 当前方向与揉库策略

```text
planning.direction_policy = bidirectional
planning.allow_shunting = false
```

- 正向和反向候选可按共同规则竞争；
- 全程反向并不因“不是首选”而自动非法；
- 普通前后反复揉库和不允许的换向结构会被拒绝；
- `forward_only` 可做对照；
- `allow_reverse_fallback` 是兼容旧参数，会映射到方向策略；
- 同时传入新旧方向参数且冲突会报错。

---

## 9. 地图和实验任务

当前运行列表保留 map01–map09、map15、map16，共 11 张地图；map10–map14 已移除。

每张可运行地图通常包含：

```text
map.pgm            占据栅格
map.yaml           分辨率、原点等 ROS 地图配置
preview.png        预览
scene_source.yaml  论文图数字化或合成场景来源
scene_meta.yaml    项目选取的测试起终点
README.md          地图来源、比例和说明
```

重要边界：

- 论文地图中的起终点大多是项目测试设置，不是论文作者原始实验位姿；
- 默认比例是项目为当前车宽和场景测试设置的工程比例；
- 地图缩放围绕中心，栅格不重采样；
- 临时缩放地图在退出时清理；
- `paper_candidates/`、`paper_digitized/` 和 `maps/archive/` 不是默认运行地图入口；
- 比较算法时必须固定地图文件、比例、起终点、车辆合同和规划预算。

---

## 10. 测试、探针和回归体系

## 10.1 单元测试入口

项目 README 给出的完整 Python 回归命令：

```bash
cd /home/wq/whut_thesis/expv3_multi
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'
```

针对性测试示例：

```bash
python3 -m unittest discover -s tests -p 'test_continuous_geometry.py' -v
python3 -m unittest discover -s tests -p 'test_continuous_retiming.py' -v
python3 -m unittest discover -s tests -p 'test_segment_timing_cache.py' -v
```

主要测试文件：

| 文件 | 覆盖方向 |
|---|---|
| `tests/test_core.py` | 基础运动学、几何、碰撞和核心功能 |
| `tests/test_continuous_geometry.py` | 连续几何、控制块、边界和终点行为 |
| `tests/test_continuous_retiming.py` | 连续时间、动态和模式事件 |
| `tests/test_segment_timing_cache.py` | 段级时间缓存键、复制、复用和淘汰合同 |
| `tests/test_quality.py` | 独立审核、指标和失败合同 |
| `tests/test_geometry_incumbent.py` | 几何 incumbent 与执行可行性分离 |
| `tests/test_planning_display.py` | 规划显示状态和新旧报告处理 |
| `tests/test_diagnostics.py` | 诊断输出行为 |

`fwis_v3_planner/tests/test_policy_history.cpp` 还包含 C++ 策略/历史相关测试。

## 10.2 ROS smoke/probe

仓库保留：

- `ros_s09_smoke.py`；
- `ros_spin_smoke.py`；
- `ros_oblique_smoke.py`；
- `ros_failure_smoke.py`；
- `ros_parking_goals_smoke.py`；
- `ros_map_probe.py`；
- 结果捕获和其他探针脚本。

这些测试用于检查真实 ROS 发布链、终点、轨迹、碰撞清除、模式行为和失败状态，不应只用纯 Python 单测替代所有端到端验证。

## 10.3 固定案例和质量回归

- `tools/expv3_eval_cases.yaml`：冻结任务；
- `tools/run_expv3_probe.py`：隔离 ROS 环境并执行探针；
- `tools/run_quality_regression.py`：串行运行固定案例并汇总；
- `tools/audit_continuous_stages.py`、`trajectory_direction_audit.py`、`audit_mode_switch_speed.py` 等：专项审核。

回归至少应关注：

- 是否成功发布；
- 实际发布轨迹而不是搜索种子；
- 5 mm 级独立整车扫掠；
- 2 cm/1° 终点；
- 方向策略；
- 当前 profile 下的时间/动态；
- 实际显示插值扫掠；
- 总执行时间；
- 规划耗时；
- 模式切换和换向次数；
- 失败状态和旧结果清理；
- 测试结束后的 ROS 进程清理。

## 10.4 测试执行注意事项

- 低内存环境优先串行或分组运行；
- `catkin_ws/README.md` 提醒旧 `test_core.py` 曾有高内存问题，完整执行前需关注资源；
- 隔离 ROS 探针使用空闲非保护端口；
- 项目文档特别要求不要占用或清理 11311、11312、11320；
- 不要在未检查的情况下批量 `pkill` ROS/Python，可能误伤其他实验；
- 当前 README 报告“151 项 Python 回归通过”等是已有记录，**本文撰写时没有重新运行，不能写成本次新验证结果**。

---

## 11. 数据采集、复审和实验产物

### 11.1 启动采集

```bash
cd /home/wq/whut_thesis/expv3_multi
bash run_demo.sh --map map04 --collect-data
```

采集模式：

- 默认关闭自动目标；
- 打开 RViz 和本地标注网页；
- 每次规划保存独立 episode；
- 可标记优质、参考、剔除和理由；
- 保存预览、请求、最终几何、实际时间轨迹、实际走廊和审核信息；
- 不自动启动训练。

### 11.2 默认保存位置

当前副本默认是：

```text
/home/wq/whut_thesis/expv3_experiment_archive/teacher_collection/
```

注意这个目录与 `expv3_multi` 同级，名称仍是 `expv3_experiment_archive`，不是 `expv3_multi_experiment_archive`。

典型结构：

```text
teacher_collection/
├── maps/
│   ├── 地图哈希.json.gz
│   └── 地图哈希.png
├── sessions/启动时间_算法/
│   ├── session.json
│   ├── source.tar.gz
│   └── episodes/000001/
│       ├── episode.json
│       ├── preview.png
│       └── analysis.json
└── diagnostics/
```

`source.tar.gz` 可包含采集时的源代码快照和工作区修改，因此复现实验不能只看 Git commit，还应查看源码快照和哈希。

### 11.3 二次复审

```bash
python3 tools/review_collected_data.py --reanalyze
```

复审重新检查保存的最终几何和实际发布的时间轨迹，不用搜索种子替代最终结果，不自动修复几何，也不自动训练。

导出人工复审通过的引用清单：

```bash
python3 tools/review_collected_data.py --export-reviewed \
  /home/wq/whut_thesis/expv3_experiment_archive/teacher_collection/reviewed_selection.json
```

已有输出文件时不会直接覆盖。

---

## 12. 当前代码质量和优化时最容易踩的坑

### 12.1 大文件和混合职责

- `core.py` 同时包含公共基础和 legacy；
- `optimizer_node.py` 同时包含 ROS 适配、legacy 流程、新后端发布和诊断；
- `mode_master.py` 同时负责候选生成、预算、结构编辑、缓存、审核编排和选解；
- `planner_node.cpp` 将搜索、状态、消息和显示集中在单个大型 C++ 文件中。

优化时应先划定最小修改面。除非任务就是重构，不要顺带进行大范围文件拆分。

### 12.2 不要改变“实际发布轨迹”的验证对象

很多候选在搜索、几何拟合或 NLP 阶段看起来很好，但最终可能失败于：

- 终点；
- 连续扫掠；
- 模式边界；
- 时间/动态；
- 显示插值；
- 方向或揉库合同。

性能对比必须以**实际发布的最终轨迹**为准，不能用粗 seed、最佳中间候选或 GEOMETRY_ONLY 结果替代。

### 12.3 不要用放宽安全条件换取“优化提升”

以下变化必须作为算法合同变化单独说明，不能当作纯提速：

- 放大最终终点容差；
- 增大扫掠步长；
- 移除 margin；
- 跳过显示插值审核；
- 将特征点走廊当成整车安全证明；
- 接受 IPOPT/Bonmin 的可疑返回而不重放审核；
- 把失败候选改成 GEOMETRY_ONLY 后仍发布 timed trajectory；
- 修改默认 profile 后仍与旧 profile 的执行时间直接比较。

### 12.4 缓存必须绑定完整上下文

可能改变正确性的上下文包括：

- 地图内容和分辨率/原点；
- 起点、目标和运动块；
- 车辆尺寸和 margin；
- 模式、方向、控制和长度；
- 动态 profile；
- 扫掠间距；
- 时间采样与插值版本；
- plan_id 和请求状态。

当前许多缓存刻意限制在单次请求或固定 evaluator 内。扩展到跨候选、跨请求或跨车复用时必须重新证明 key 和生命周期。

### 12.5 时间预算不是结果正确性的替代品

减少预算可能提速，但也可能：

- 改变获选模式序列；
- 丢失更短轨迹；
- 增加失败率；
- 使困难地图退化为几何回退；
- 造成不同地图上的质量波动。

优化任务必须同时报告求解耗时、最终执行时间、成功率和安全审核，而不是只报 wall time。

### 12.6 历史耗时不能直接当当前保证

历史文档中的耗时与以下因素绑定：

- 当时的源码；
- 地图和比例；
- 起终点；
- 车辆和 profile；
- 搜索预算；
- 硬件和依赖；
- 是否包含前端、后端、审核和发布。

任何新任务都应重新记录这些上下文。

---

## 13. 当前明确的单车假设：多车任务必读

1. **全局 topic**：`/plan/*`、`/robot/*`、`/start_pose` 等没有车辆 namespace。
2. **无车辆 ID**：五个核心消息均不能区分车辆。
3. **单请求状态**：C++ planner 只保存一个起点、一个目标和当前 plan_id。
4. **单 evaluator**：Python 每次请求只构造一个起点/目标对应的单车审核器。
5. **静态单层地图**：没有多车动态占用或其他车时间轨迹输入。
6. **固定 `map` frame**：主输入和输出都围绕单一 `map`。
7. **无实时状态反馈**：没有 odom、实时定位或跟踪误差进入规划主链。
8. **唯一车辆合同**：当前 launch 给所有节点加载同一份 Ranger Mini 2 参数。
9. **单轨迹回放**：Playback 只保存一条轨迹。
10. **固定 Marker 标识**：车体、朝向和 HUD 都使用固定 namespace/ID，多车会覆盖。
11. **单车 RViz 配置**：topic 和显示项没有按车组织。
12. **单请求缓存和诊断**：缓存生命周期、质量报告和采集结构都以当前一个规划请求为中心。
13. **没有协同约束**：没有车车碰撞、交叉路口占用、优先级、让行、死锁或联合目标。
14. **没有多车任务输入**：当前一张 `PlanSeed` 只能表达一个中心起点和一个中心终点。

### 对后续多车设计的边界建议

以下是交接建议，不是当前已实现功能：

- 尽量先把现有 `ModePathMaster` 和 `TrajectoryEvaluator` 视为**单车规划与单车静态审核内核**；
- 在其外明确增加车辆身份、namespace、任务管理和车间时空冲突层；
- 不要误以为启动三个 planner 和三个 optimizer 就会自动产生协同；
- 不要只修改 RViz 就声称完成三车规划；
- 不要只把其他车当成静态障碍，因为这不能表达时间错开和协同让行；
- 多车安全审核必须检查每对车辆在实际时间轨迹上的相对占用，并保留每车原有的静态地图扫掠；
- 车辆 ID 与 plan_id 应是不同概念；
- 协同请求 ID 与各车局部 plan_id 也应明确区分；
- 先保留单车回归，再增加多车合同测试，避免多车改造破坏现有单车能力。

---

## 14. 给接手 AI 的任务执行规范

### 14.1 开始任务前

1. 确认当前目录是 `/home/wq/whut_thesis/expv3_multi`，不要误改 `/home/wq/whut_thesis/expv3`。
2. 执行 `git status --short --branch`，记录已有未提交修改。
3. 记录当前 commit、分支和远程。
4. 阅读任务涉及的当前源码，不只阅读本文。
5. 确认允许修改的文件范围。
6. 确认算法合同、车辆 profile、地图、起终点和预算是否允许改变。
7. 先运行与任务相关的最小基线测试；若未运行，明确说明原因。
8. 检查是否已有 ROS 进程、构建任务或采集会话，避免相互干扰。

### 14.2 修改过程中

- 将“纯重构”“性能优化”“算法行为变化”“多车接口变化”分开；
- 优先小步修改，保留可回归状态；
- 不在没有要求时重写整个规划器；
- 不修改生成目录和历史 archive；
- 不用全局替换盲目改变 topic/message；
- 新增缓存时写清 key、失效条件和生命周期；
- 新增状态字段时同步 C++、Python、采集、回放、诊断和测试；
- 修改时间轨迹后必须重新做实际发布链审核；
- 不把日志中的“求解成功”当成最终认证通过。

### 14.3 完成任务后必须报告

1. 修改了哪些文件和模块；
2. 改变了什么行为，什么行为保持不变；
3. 使用了哪份地图、比例、起终点、车辆配置和预算；
4. 运行了哪些测试，贴出真实结果；
5. 哪些测试没有运行以及原因；
6. 成功率、求解耗时、执行时间、终点、扫掠和动态审核结果；
7. 是否有失败案例、性能回退或遗留风险；
8. 当前 Git diff 是否包含无关文件；
9. 不得声称未实测的全地图提升、全局最优、严格实时或实车安全性。

### 14.4 推荐任务 Prompt 模板

把任务交给其他 AI 时，建议明确写成：

```text
任务目标：
  只优化【某个明确模块/指标】，例如减少 mode_path_v2 的段级重定时耗时。

基线：
  仓库 /home/wq/whut_thesis/expv3_multi
  分支 continuous-mode-path
  提交【填写实际 commit】
  固定地图、比例、起终点、params.yaml 和 ranger_mini_2.yaml。

允许修改：
  【列出文件或目录】

不得改变：
  最终 2 cm/1° 终点合同；
  独立整车扫掠和实际显示插值审核；
  默认方向/揉库规则；
  消息字段和 topic（除非本任务明确要求）；
  最终获选轨迹及其执行时间（若任务要求行为保持）。

必须验证：
  【列出单元测试、ROS 案例和独立审核】
  对比修改前后求解耗时、获选来源、执行时间、候选数和安全结果。

交付：
  修改摘要、真实测试输出、前后指标、未运行测试、遗留风险。
  不要修改 archive、build、devel 和历史实验产物。
```

---

## 15. 按任务快速定位源码

| 任务类型 | 首先查看 |
|---|---|
| 改启动、地图选择、默认后端 | `run_demo.sh`、`fwis_v3_demo/launch/demo.launch` |
| 改车辆尺寸和共享动态限值 | `fwis_common/config/ranger_mini_2.yaml`、各节点参数读取点 |
| 改 C++ 粗搜索 | `fwis_v3_planner/src/planner_node.cpp`、`params.yaml: search/cost` |
| 改 A/O/Spin 消息 | `fwis_v3_msgs/msg/`、C++ 发布、Python 接收与所有测试 |
| 改 v2 候选主流程 | `mode_master.py::ModePathMaster.solve()` |
| 改连续几何 | `continuous_geometry.py` |
| 改局部连续 NLP | `continuous_optimizer.py` |
| 改速度/加速度/模式切换时间 | `continuous_retiming.py`、`retiming.py`、`mode_switch_timing.py` |
| 改段级缓存 | `continuous_retiming.py::SegmentTimingCache`、`tests/test_segment_timing_cache.py` |
| 改最终安全审核 | `quality.py::TrajectoryEvaluator`、`vehicle_execution.py` |
| 改最终发布 | `optimizer_node.py::run_mode_path()`、`publish()` |
| 改失败和几何 incumbent | `geometry_incumbent.py`、`optimizer_node.py` 失败分支 |
| 改走廊显示 | `core.py` 走廊函数、`optimizer_node.py::publish()`；不要误当 NLP 约束 |
| 改回放 | `fwis_v3_playback/scripts/playback.py` |
| 改实验回归 | `tools/expv3_eval_cases.yaml`、`run_expv3_probe.py`、`run_quality_regression.py` |
| 改人工采集 | `collect_node.py`、`collection.py`、`tools/review_collected_data.py` |
| 多车 topic/身份改造 | 消息、planner、optimizer、playback、launch、RViz、采集全部联动检查 |
| 多车车车冲突 | 当前没有现成实现，需要新增协同与时空审核层 |

---

## 16. 当前能力的准确表述示例

在论文、报告或任务总结中，可以使用：

> 当前系统在静态占据栅格和给定单车中心起终点下，通过 C++ 多模式前端生成规划种子，由 Python `mode_path_v2` 后端构造有限连续结构候选，进行连续几何处理、时间参数化和独立整车扫掠/终点/方向/动态审核，并从完整认证候选中选择轨迹，最后由 RViz 按时间回放。

不应使用：

- “已经实现三车协同规划”；
- “已经实现全局模式—路径—时间联合最优”；
- “走廊保证了连续 NLP 的整车安全”；
- “输出可以直接作为实车控制命令”；
- “默认 profile 已完成实车全动态标定”；
- “所有地图均能保证成功或严格实时”；
- “历史测试全部在当前提交重新通过”。

---

## 17. 最短阅读路线

如果接手 AI 时间有限，至少按以下顺序阅读：

1. 本文第 0、1、12、13、14 节；
2. `run_demo.sh`；
3. `fwis_v3_demo/launch/demo.launch`；
4. `fwis_v3_demo/config/params.yaml`；
5. `fwis_common/config/ranger_mini_2.yaml`；
6. `fwis_v3_msgs/msg/`；
7. `fwis_v3_planner/src/planner_node.cpp::on_goal()`；
8. `optimizer_node.py::on_seed()` 和 `run_mode_path()`；
9. `mode_master.py::ModePathMaster.solve()`；
10. `quality.py::TrajectoryEvaluator.evaluate_continuous_geometry()`；
11. `continuous_retiming.py` 和 `vehicle_execution.py`；
12. 与任务直接相关的测试。

---

## 18. 总结

当前仓库已经具备一条较完整、可诊断、强调独立审核的**单车 4WIS 多模式规划与演示链**。它的主要价值不只是“能搜索一条路径”，还包括：

- A/O/Spin 多模式候选；
- 连续几何表示；
- 模式事件和动态限值下的时间求解；
- 候选级预算和缓存；
- 独立整车扫掠；
- 严格终点和方向合同；
- 实际显示插值复核；
- 失败/几何可行/可执行状态区分；
- 固定案例回归和人工数据采集。

同时，它仍是单体、静态、开环参考轨迹系统，代码中大型文件和历史后端并存，部分文档滞后。后续优化必须保护实际发布轨迹的安全与执行合同；后续三车扩展则必须新增车辆身份、隔离、协同决策和车车时空审核，而不是简单复制三个单车 Demo。
