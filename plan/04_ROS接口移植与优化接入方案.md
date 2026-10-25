# expv3：从 exp 移植 ROS1 接口并接入新规划器

## 1. 结论与迁移边界

采用现有 **ROS1 Noetic + catkin**。把 [exp/catkin_ws](../../exp/catkin_ws) 当作 underlay，在 `expv3/catkin_ws` 建 overlay；沿用地图、起终点交互和 RViz 话题约定，新增 `fwis_v3_msgs`、`fwis_v3_planner`、`fwis_v3_optimizer`、`fwis_v3_playback`。这些包使用新名字，避免在同一 catkin 环境中出现两个同名包；现有 `exp` 保持可独立运行，供结果对照。

ROS 接口保留：`/map`、`/initialpose`、`/start_pose`、`/move_base_simple/goal`、`/plan/path`、`/plan/mode_strip`、`/plan/markers`、`/plan/corridor_markers`、`/robot/marker`、`/robot/hud`。新轨迹发布到 `/plan/timed_trajectory`，其逐点内容为 `(t,[x,y,v,θ])`，位置与平动速度均对应车辆中心（前后轴中心中点）。`/plan/path_coarse` 专门显示粗解；`/plan/path` 只显示通过独立验碰的最终结果，所以原 RViz 配置里的 Path 显示可直接工作。

需区分“当前 overlay 实现”和“推荐算法目标”：现有 `/plan/seed` 携带固定的 A/O/S 原语，optimizer 只能优化给定模式序列；它是可运行 baseline，不是模式—轨迹联合决策。目标架构需要把前端输出改成一组 route/corridor 初值，把模式二进制变量和连续位姿/控制交给同一个 joint optimizer；成功后由 optimizer 生成最终模式原语及轨迹。ROS 起终点与 RViz 接线可原样复用，但 Planner—Optimizer 消息合同需要扩展/重定，当前 fixed-sequence ROS 闭环不能假定自动具备联合优化能力。

这不是把旧 planner + 旧 backend 的 launch 原样 include 进来。旧前端的 A 模式曲率映射与 Euler 步推进、旧后端的自由速度模型和旧回放器的自主计时分别需要适配；Spin 固定输入点和 OBB 中心坐标约定则与本方案一致，见第 2 节。

## 2. 现有组件逐项判定

| exp 文件/组件 | 可直接沿用 | v3 需要处理的差异 |
|---|---|---|
| [map_publisher.py](../../exp/catkin_ws/src/fwis_map_server/scripts/map_publisher.py) | `/map` latched `nav_msgs/OccupancyGrid`、地图加载 | `/start_pose` 是车辆中心预设，可直接作为 v3 起点，无坐标转换 |
| [parking_lines.py](../../exp/catkin_ws/src/fwis_map_server/scripts/parking_lines.py) | `/parking_lines` 可视化 | 无运动学依赖，可直接沿用 |
| [scene_hints.py](../../exp/catkin_ws/src/fwis_map_server/scripts/scene_hints.py) | 静态车位提示 Marker 与 `/preset_goal` | 预设目标按车辆中心解释，可与 RViz 手动目标共用 `/move_base_simple/goal`；先核对场景坐标与地图对齐 |
| [planner_node.cpp](../../exp/catkin_ws/src/fwis_planner_hybrid_astar/src/planner_node.cpp) | `/map`、`/initialpose`、`/start_pose`、`/move_base_simple/goal` 订阅方式；Path/Marker 发布方式 | 沿用 ROS 回调和可视化接口；新主线用二维 A* 产生拓扑种子，再构建走廊候选图；现有模式增广 Hybrid A* 留作 benchmark/fallback；A/O/S 候选使用车辆中心精确模型与 swept OBB |
| [GridMap 和 Reeds–Shepp](../../exp/catkin_ws/src/fwis_common/include/fwis_common/grid_map.hpp) | 栅格读取、距离场、RS 曲线工具 | OBB 输入和 RS 切向参考点均可直接用车辆中心；旧点采样 OBB 不作为 v3 的准入判碰，改用占据格 SAT；A 模式修正为 `κ=2tanδ_v/L` 并用精确圆弧传播 |
| [safe_corridor_node.py](../../exp/catkin_ws/src/fwis_backend_qp/scripts/safe_corridor_node.py) 与 [CasADi 后端](../../exp/catkin_ws/src/fwis_backend_qp/scripts/casadi_safe_corridor_backend.py) | ROS 节点订阅/发布、走廊构造的部分实现思路 | 默认 `lock_verified_geometry=true`，几何被冻结；优化器是自由车体速度/轮级时序模型，不固定 A/O/S 模式序列，不能直接得到本方案的固定模式几何优化结果 |
| [playback.py](../../exp/catkin_ws/src/fwis_planner_hybrid_astar/scripts/playback.py) | `/robot/marker`、`/robot/hud` Marker 风格及 Spin 同中心判定 | 旧版自行估计速度/停顿；新轨迹必须按消息时间戳播放，Cube 位置直接用车辆中心 `(x,y)` |
| [single_vehicle.rviz](../../exp/catkin_ws/src/fwis_planner_hybrid_astar/rviz/single_vehicle.rviz) | `/map`、`/plan/path`、`/plan/mode_strip`、`/robot/marker` 等显示与 RViz 起终点工具 | v3 新配置增加 `/plan/path_coarse` 和起终点显示；走廊订阅 `/plan/corridor_markers`，继续以 `map` 为 Fixed Frame |

现有 [fwis_msgs/Trajectory.msg](../../exp/catkin_ws/src/fwis_msgs/msg/Trajectory.msg) 有 `pose/v/omega/mode/phi/d_switch`，没有每点时间戳；旧 planner 发布时所有 `v=0`，旧 backend 发布 `/plan/trajectory_optimized`，旧 playback 再自行计时。这条链不符合 `(t,[x,y,v,θ])` 的交付合同，也不能靠只加一个 `time` 数组解决旧模式和坐标语义，所以 v3 使用新消息类型和新话题，不修改 exp 已有消息的 MD5。

## 3. 坐标合同：输入、输出和 S09 预设

v3 的规划状态、RViz 手动起终点、`/start_pose`、预设目标、`/plan/path` 及 timed trajectory 全部表示**车辆中心** `c=(x,y)`，即前后轴中心的中点；不引入后轴参考点转换。轴距 `L=0.494 m` 只用于车辆几何和双阿克曼曲率：前/后轴中心分别为 `c±0.247e(θ)`。

- `/initialpose` 与 `/move_base_simple/goal`：RViz 用户设置的车辆中心位姿，直接进入规划器。
- `/start_pose`：map publisher 从场景文件发布的车辆中心起点，直接进入规划器。
- `/preset_goal`：scene_hints 的车辆中心目标可直接发布到 `/move_base_simple/goal`，无专用坐标适配器。

具体到现有 [S09 scene_meta](../../exp/maps/map16-s09-underground-garage/scene_meta.yaml)，起点仍为 `c=(13.20,2.25), θ≈π`，目标仍为 `c=(5.86,0.55), θ≈−π/2`；不得误改为 `(13.447,2.25)` 或 `(5.86,0.797)`。地图的 S09 原点航向为 0，现有 map publisher 对该场景可直接使用。

旧 map publisher 还会在起点发布一次静态 `map→base_link`。v3 的车辆回放使用 `map` 系 Marker；不把这个静态 TF 当成车辆实时位姿，以免机器人 Marker 移动而 TF 停在起点。

## 4. ROS 节点与话题合同

```text
fwis_map_publisher ── /map, /start_pose(车辆中心) ──┐
RViz / scene_hints ── /initialpose, /move_base_simple/goal(车辆中心) ├→ fwis_v3_planner
                                                                  ├─ /plan/path_coarse
                                                                  ├─ /plan/seed
                                                                  └─ /plan/markers
                                                                            ↓
                                                      fwis_v3_optimizer ← /map
                                                        ├─ /plan/path (最终中心路径)
                                                        ├─ /plan/timed_trajectory
                                                        ├─ /plan/mode_strip
                                                        ├─ /plan/corridor_markers
                                                        └─ /plan/status
                                                                            ↓
                                                      fwis_v3_playback
                                                        ├─ /robot/marker
                                                        └─ /robot/hud
```

`fwis_v3_msgs` 定义两个明确的内部/外部接口：

```text
ModePrimitive.msg:
  uint8 mode           # 0=A, 1=O, 2=S
  float64 amount       # A/O: 车辆中心有符号弧长; S: 有符号航向增量
  float64 control      # A: κ; O: φ; S: 0

PlanSeed.msg:
  std_msgs/Header header           # frame_id=map
  uint32 plan_id
  geometry_msgs/Pose2D start_center
  geometry_msgs/Pose2D goal_center
  ModePrimitive[] primitives       # 可由第 2.3 节 F_A/F_O/F_S 精确重放

TimedPoint.msg:
  duration time_from_start
  geometry_msgs/Pose2D center_pose  # x,y,theta
  float64 v                        # 车辆中心平动速度大小；Spin 为 0

TimedTrajectory.msg:
  std_msgs/Header header          # 轨迹起始 ROS 时间, frame_id=map
  uint32 plan_id
  TimedPoint[] points             # 逐点 t 严格递增
```

模式和控制量留在 `PlanSeed` 内供优化与诊断，不增加到对外四维轨迹状态。`nav_msgs/Path` 中各 `PoseStamped.header.stamp=TimedTrajectory.header.stamp+time_from_start`。`/plan/status` 首版可用 `std_msgs/String` 发布 `plan_id/阶段/成功或拒绝原因`，不把失败候选发布成成功轨迹。每次新目标递增 `plan_id`，后端/回放丢弃旧 `plan_id` 的迟到结果；规划失败时发布空 Path/空 timed trajectory 清掉旧显示。全流程输入输出固定在 `map` frame，其他 frame 先经 TF 变换，不能把非 map 坐标默认为 map。

## 5. 在 ROS 链路中的算法接入点

1. **前端 `fwis_v3_planner`（C++）**：复用 exp 的 `GridMap` 和 ROS 回调约定；二维 4 邻接 A* 产生多个 `(x,y)` 拓扑初值，不输出已选定的最终模式序列。按每条初值构造 corridor/STC 和优化边界；模式增广 Hybrid A* 与稀疏候选图保留为 baseline/fallback。发布 corridor 初值与可选 incumbent，不能把未验收结果放入 `/plan/path`。
2. **后端 `fwis_v3_optimizer`（Python/CasADi + 混合整数/序列主问题）**：目标版本订阅 route/corridor 初值，在同一空间问题里优化车辆中心位姿、A/O/S 模式变量和相应控制；可先用 mode-sequence master + 固定模式 NLP subproblem 迭代开发，再评估 corridor 内 sequential MIQP/MINLP。expv2 fixed-sequence optimizer 仅是连续子问题/初值生成器，不可把前端首次给定序列锁死。Ranger 的 A/O/S 模型需自行复核和实现，不能直接移植论文中的不同参考点模型。
3. **验收与时间化**：复用 expv2 OBB 与占据格 SAT 判交方法，并在精确模式原语上密采样，验证搜索粗解和优化解。通过后按[第 8.2 节](03_expv3%E6%8E%A8%E8%8D%90%E7%AE%97%E6%B3%95%E6%A1%86%E6%9E%B6.md)生成时间与中心平动速度 `v`；Spin 段位置固定、`v=0`，但航向随时间变化。对重采样轨迹复验，再发布 `/plan/timed_trajectory` 与 `/plan/path`。
4. **回放 `fwis_v3_playback`（Python）**：保留旧 Marker/HUD 外观和 Spin 同位置识别方式；按 `header.stamp+time_from_start` 读取轨迹，不再自己分配速度或 dwell。车身矩形 Marker 直接放在轨迹中心 `(x,y)`；Spin 时中心 Marker 不平移，仅随 `θ(t)` 旋转。未收到轨迹时，对 `/initialpose` 或 `/start_pose` 的中心位姿直接显示静态车辆。

这条分工使 ROS 接口仍沿用 exp 的交互方式，而几何模型、优化器和轨迹消息可以独立迭代。规划器和优化器之间传**控制原语**，避免从稀疏 `nav_msgs/Path` 反推曲率、斜移角和 Spin 模式。

## 6. Overlay 目录和 launch 组合

```text
exp/catkin_ws/                  # 已有 ROS underlay，保留原样
expv3/catkin_ws/src/
  fwis_v3_msgs/                 # ModePrimitive / PlanSeed / TimedPoint / TimedTrajectory
  fwis_v3_planner/              # 复用订阅/Marker 接口 + 新搜索模型
  fwis_v3_optimizer/            # 固定模式 STC、OBB 验收、时间化
  fwis_v3_playback/             # 按时间戳播 Marker
  fwis_v3_demo/                 # launch、config、RViz 配置
```

运行时先 source Noetic 和 `exp/catkin_ws/devel/setup.bash`，再构建/source v3 overlay。实际入口为 `fwis_v3_demo/launch/demo.launch`，组合 `fwis_map_server/map_publisher.py`、`parking_lines.py`、`scene_hints.py`、v3 planner、v3 optimizer、v3 playback 和 v3 RViz 配置；地图目录参数仍指向 `exp/maps/map16-s09-underground-garage`。**不要 include** `single_vehicle_demo.launch`，它会同时启动旧 planner、旧 backend、旧 playback，产生重复发布和不兼容轨迹。scene hints 的预设目标可直接沿用中心坐标。

可按以下顺序逐步验收，避免 ROS 接线问题和算法问题混在一起：

1. 只启动地图、v3 输入节点和 RViz，确认 `/map`、手动 `/initialpose`、手动 goal、`/start_pose` 和预设目标均使用车辆中心且无需转换；在 S09 画出中心点和车身矩形，确认起点占据与 exp 旧演示相同。
2. 当前 baseline 可接入二维 A* 与候选图并分别可视化；目标 joint 版本需把 corridor seed 与优化后由 solver 决定的模式原语分开。特别验证从 `c=(0,0),θ=0` Spin `90°` 后仍是 `c=(0,0)`、`θ=π/2`，该段 `v=0`，且整个旋转 sweep 无碰。
3. 接入固定模式优化与独立 SAT 扫掠验证；只有成功解才进入 `/plan/path`。在 S09 揉库目标验证终点容差、曲率和整车净空。
4. 接入 `TimedTrajectory` 和 v3 playback，检查逐点时间严格递增、中心速度与位姿变化一致、RViz 车身 Marker 对齐碰撞 OBB。最后再加入局部失败修复。

当前 `expv3/catkin_ws` 已创建并通过 S09 默认目标、Spin、O 横移和失败清空的 headless ROS 验收，但该闭环仍是 Hybrid A* + fixed-sequence optimizer。走廊内模式—轨迹联合优化及其消息接口尚未实现。运行与已知边界见 [工作空间 README](../catkin_ws/README.md)；算法目标和实施次序见[联合优化分析](05_%E5%86%B3%E7%AD%96%E4%B8%8E%E8%BD%A8%E8%BF%B9%E8%81%94%E5%90%88%E4%BC%98%E5%8C%96%E7%AE%97%E6%B3%95%E5%88%86%E6%9E%90.md)。
