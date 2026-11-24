# CopTrans V1.1 规划与控制联调验收

运行环境为 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic 8.15.0。正式入口为仓库根目录的 `scripts/test.sh`。默认实验取场景的装载区起点，目标为相对起点前方 1.52 m、左侧 0.35 m 和逆时针 0.45 rad 的低速短弯。

三个场景的九组单车短弯与三组楔形短弯通过闭环验收。室内工厂还通过了三车 PID、录包和移动中单车反馈中断验收。

## 复现

```bash
cd /home/wq/Whut/CopTrans_base
./scripts/test.sh
./scripts/test.sh --task multi --formation wedge
./scripts/test.sh --task multi --controller-backend pid --record on
./scripts/test.sh --scenario port_01 --planner-backend hybrid_astar
./scripts/test.sh --list-scenarios
./scripts/test.sh --list-backends
./scripts/test.sh --task multi --experiment full_route --dry-run
```

参数与运行目录见 [工作空间说明](../../ros2_ws/README.md#统一仿真测试入口)。场景继续从 `ranger_mini_gz` 的注册表与 YAML 加载。实验使用 `fwis_bringup/config/experiments.yaml`，后端、控制器和低速约束使用 `test_defaults.yaml`。已注册三个工业场景，单车支持 Hybrid A*、恒定原语和连续模式路径三个后端，三车使用 `multi_4wis` 的 `virtual_rigid_a`。控制器可选非线性反馈和 PID，`auto` 使用非线性反馈。 `--mode real` 保留参数与适配配置，返回 2。

脚本为每次测试选择未占用的本机 DDS 端口范围，限制 ROS 发现范围为本机，并分配独立 Gazebo 分区。每次正常仿真启动两套 GUI。 `reports/runs/<run_id>/` 存放可复现命令、解析配置、环境、原生轨迹、控制日志、实际测量和 `result.yaml`，验收运行另有两窗截图。

## 接口矩阵

下表的 `N` 表示车辆 ID。单车入口使用 `robot1`，三车使用 `robot1`、`robot2`、`robot3`。 `map` 的方向和 Gazebo 世界坐标一致。

| Topic | 类型 | QoS | 坐标或时间 | 发布 → 订阅 |
| --- | --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | 发布 Reliable / Volatile，时间源订阅 Best Effort | Gazebo 仿真时钟 | 世界时钟桥 → 规划器、协调节点和控制器 |
| `/map` | `nav_msgs/OccupancyGrid` | Reliable / Transient Local | `map` | 场景栅格 → 规划器、协调节点、RViz2 |
| `/robotN/ground_truth_pose` | `geometry_msgs/PoseStamped` | Reliable / Volatile | `map`，模型中心 | Gazebo Pose 和 TF 适配器 → 起点提取、统计 |
| `/robotN/odom` | `nav_msgs/Odometry` | Reliable / Volatile | 父帧 `map`，子帧 `robotN/base_link` | 本车车体仿真和桥 → 本车控制器、协调节点 |
| `/robotN/motion_state` | `std_msgs/Int32` | Reliable / Volatile | 0 Ackermann、1 Oblique、2 Spin | 本车车体仿真和桥 → 本车控制器、协调节点 |
| `/start_pose`、`/move_base_simple/goal` | `geometry_msgs/PoseStamped` | Reliable / Volatile | `map` | 单车实验 → 所选单车规划器 |
| `/plan/timed_trajectory` | `fwis_v3_msgs/TimedTrajectory` | Reliable / Transient Local | `map`，发布时刻加相对时间 | 单车优化器 → robot1 控制器、记录 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Reliable / Volatile | `map` 中的编队参考 | 三车实测中心拟合 → 编队规划器 |
| `/formation/plan_request` | `fwis_v3_msgs/FormationPlanRequest` | Reliable / Volatile | `map`，共享目标和三车起点 | 实验 → 编队规划器、既有校验节点 |
| `/formation/playback_plan` | `fwis_v3_msgs/FormationPlaybackPlan` | Reliable / Transient Local | `map`，共同纳秒时间轴 | 编队规划器 → RViz 轨迹校验、分发 |
| `/formation/job_state` | `std_msgs/String` | Reliable / Transient Local | 规划 ID、模型摘要、任务状态 | 编队规划器 → 校验与分发 |
| `/robotN/trajectory` | `fwis_v3_msgs/TimedTrajectory` | Reliable / Transient Local | `map`，三车共同开始时刻加原始相对时间 | 分发 → 对应控制器 |
| `/formation/execution_permit` | `fwis_v3_msgs/FormationExecutionPermit` | Reliable / Volatile | 仿真时钟，规划 ID、开始时刻与许可 | 协调节点 → 三个控制器 |
| `/robotN/cmd_vel` | `geometry_msgs/Twist` | Reliable / Volatile | `robotN/base_link`，m/s 和 rad/s | 本车控制器 → 本车桥、Gazebo |
| `/robotN/sim_request_mode` | `std_msgs/Int32` | Reliable / Volatile | 仿真实验的停车模式请求 | 本车控制器 → 本车桥 |
| `/robotN/controller_status` | `std_msgs/String` | Reliable / Volatile | 规划 ID、仿真时刻、车体系误差和模式 | 本车控制器 → 协调与记录 |
| `/plan/path`、`/formation/path/vehicle_N` | `nav_msgs/Path` | Reliable / Transient Local | `map` | 规划输出与轨迹校验 → RViz2 |
| `/cargo/odom`、`/scene/markers` | `nav_msgs/Odometry`、`visualization_msgs/MarkerArray` | Reliable，Odometry 为 Volatile，Marker 为 Transient Local | `map` | 实测编队中心拟合 → Cargo 视觉与 RViz2 |

## 实现与验收口径

- 入口以 Gazebo 实测中心发起规划，等待单车搜索器与优化器分别确认地图接收，或等待编队规划器确认地图和起点。单车优化回调保持串行，时钟在独立回调组更新，优化器发布轨迹时取最新仿真时间。执行保护保留 0.70 s 轨迹接纳窗。
- 三车准备使用规划器认证过的原子消息。分发以 vehicle ID 匹配，检查规划 ID、模型摘要、起终点、共同纳秒网格、楔形 offsets 和 Ackermann 模式。分发只设置共同执行零时刻，逐点位姿、相对时刻、带符号速度和模式来自原生规划。启动前需要三台控制器接纳同一计划，运动中要求持续收到同一计划的许可。
- 楔形唯一来源是 `fwis_backends/multi_4wis/config/transport_formation.yaml`。在编队系下，robot1 为 `(1,0)`，robot2 为 `(0,1)`，robot3 为 `(0,-1)`。室内工厂的车辆实测起点为 `(5,5.6)`、`(4,6.6)`、`(4,4.6)` m。
- 测试规划使用每车 0.16 m/s、0.22 m/s²、0.40 rad/s 和 0.45 rad/s² 的低速限值，Ackermann 最小规划半径为 0.90 m。控制器原有的 0.83 m Ackermann 准入保护继续生效。 `virtual_rigid_a` 的 ROS 适配给规划和 RViz 校验传递同一套车辆运动限值。
- 通过条件包括收到真实规划、各车产生 Twist、Gazebo 位置实际变化、所有跟踪器完成、到点误差低于 0.10 m 和 0.10 rad，以及实际采样净空为正。三车还检查 Cargo 支座拟合残差低于 0.10 m 和车体间隙为正。
- Cargo 的显示位姿由三车实测中心拟合。使用实际车体和 Cargo 平面投影采样核查障碍净空和相互距离。这里的 Gazebo 车体响应为运动学仿真，Cargo 可视装配随实测队形更新。

## 变更位置

| 位置 | 变更 |
| --- | --- |
| `scripts/test.sh`、`.gitignore` | 参数入口、隔离域、运行目录、进程清理 |
| `fwis_bringup/config/`、`fwis_bringup/launch/` | 测试注册、低速实验、正式 Launch 及场景和规划选项 |
| `fwis_bringup/scripts/run_experiment`、`fwis_bringup/fwis_bringup/` | 原生任务提交、ID 路由、同步许可、独立碰撞及误差统计 |
| `fwis_control/scripts/ranger_tracking_controller`、`fwis_control/fwis_control/` | 可选同步执行接口和共用 ROS 状态读取，沿用控制律及安全限幅 |
| `ranger_mini_gz` | 每车独立状态与命令桥、模型真值、Cargo 可视跟随 |
| `fwis_ros/fwis_ros/node.py`、三个单车后端的 ROS 入口 | 地图就绪状态、独立仿真时钟、发布时间基准与进程退出 |
| `fwis_backends/multi_4wis` | 现有认证规划与显示校验接收同一套物理限值 |
| `fwis_v3_msgs`、`fwis_bringup/test` | 新增许可消息与时间轴、身份回归测试 |

## 单车实跑

九组使用同一非线性跟踪器，均启动 Gazebo 和 RViz2，收到各自后端的原生 `TimedTrajectory`，执行短弯并在 `COMPLETE` 后回到零 Twist。到点误差相对于发给规划器的目标。RMSE 与最大跟踪误差相对于各条已发布轨迹。

| 场景 | 规划器 | 规划墙钟 s | 轨迹仿真 s | 到点 mm | 到点航向 ° | RMSE mm | 最大跟踪 mm | 平均控制 ms | 记录 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 室内工厂 | 连续模式路径 | 4.06 | 12.53 | 5.09 | 0.25 | 2.86 | 6.07 | 0.21 | [结果](../../reports/runs/20261124_132819_single_factory_indoor_01_continuous_mode_path_HAUUzq/result.yaml) |
| 室内工厂 | Hybrid A* | 7.19 | 21.00 | 13.60 | 0.43 | 3.99 | 8.54 | 0.20 | [结果](../../reports/runs/20261124_130945_single_factory_indoor_01_hybrid_astar_MM6083/result.yaml) |
| 室内工厂 | 恒定原语 | 9.03 | 11.65 | 7.39 | 0.28 | 3.25 | 8.02 | 0.22 | [结果](../../reports/runs/20261124_131024_single_factory_indoor_01_constant_primitive_wEvWgl/result.yaml) |
| 港口维护区 | 连续模式路径 | 4.29 | 12.53 | 5.05 | 0.24 | 2.84 | 6.04 | 0.21 | [结果](../../reports/runs/20261124_131746_single_port_01_continuous_mode_path_fmyEMZ/result.yaml) |
| 港口维护区 | Hybrid A* | 6.36 | 19.02 | 4.79 | 0.58 | 2.78 | 6.16 | 0.20 | [结果](../../reports/runs/20261124_131813_single_port_01_hybrid_astar_w4qqNN/result.yaml) |
| 港口维护区 | 恒定原语 | 12.31 | 11.65 | 7.60 | 0.29 | 3.29 | 8.18 | 0.22 | [结果](../../reports/runs/20261124_131849_single_port_01_constant_primitive_zhuKrB/result.yaml) |
| 室外工厂 | 连续模式路径 | 4.37 | 12.53 | 5.06 | 0.25 | 2.84 | 6.02 | 0.20 | [结果](../../reports/runs/20261124_131923_single_factory_outdoor_01_continuous_mode_path_0tTYQf/result.yaml) |
| 室外工厂 | Hybrid A* | 8.19 | 21.69 | 16.10 | 1.08 | 4.48 | 9.27 | 0.20 | [结果](../../reports/runs/20261124_131950_single_factory_outdoor_01_hybrid_astar_3KkyJS/result.yaml) |
| 室外工厂 | 恒定原语 | 9.70 | 11.65 | 7.47 | 0.28 | 3.31 | 8.11 | 0.21 | [结果](../../reports/runs/20261124_132030_single_factory_outdoor_01_constant_primitive_XmJs4x/result.yaml) |

## 三车实跑

室内工厂完成约 26° 的楔形短弯。分发前后的 CDR 逐点核对位姿、速度、模式与相对时间，每车 1,418 点，三条轨迹同一 `plan_id` 和共同零时刻。非线性与 PID 两次分别测得最大单车纵向 Twist 差值为 0.1065 和 0.1066 m/s，三车均到点。开始偏差按控制器首次进入跟踪的仿真时刻统计，两次采样均为 0 ms。

| 场景 | 控制器 | 编队到点误差 mm / ° | 最大支座误差 mm | 最小车体间隙 m | Cargo 最小障碍净空 m | 记录 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| 室内工厂 | 非线性反馈 | 0.62 / 0.012 | 0.58 | 0.534 | 0.574 | [结果](../../reports/runs/20261124_130456_multi_factory_indoor_01_multi_4wis_qtawzz/result.yaml) |
| 室内工厂 | PID | 0.46 / 0.001 | 0.42 | 0.535 | 0.574 | [结果](../../reports/runs/20261124_132308_multi_factory_indoor_01_multi_4wis_cJsFyS/result.yaml) |
| 港口维护区 | 非线性反馈 | 0.63 / 0.012 | 0.59 | 0.534 | 0.805 | [结果](../../reports/runs/20261124_132905_multi_port_01_multi_4wis_zEp7ZJ/result.yaml) |
| 室外工厂 | 非线性反馈 | 0.62 / 0.012 | 1.19 | 0.534 | 1.605 | [结果](../../reports/runs/20261124_132958_multi_factory_outdoor_01_multi_4wis_gPmkL5/result.yaml) |

非线性实例按 robot1、robot2、robot3 的终点误差为 1.25、0.37 和 0.50 mm，航向误差为 0.309°、0.061° 和 0.021°。三车中心最小距离为 1.413 m。PID 到点截图和 56,025 条消息的录包保存在同一次运行目录，三个轨迹 Topic 各记录一条完整轨迹。 [Gazebo 到点](../../reports/runs/20261124_132308_multi_factory_indoor_01_multi_4wis_cJsFyS/screenshots/gazebo-final.png) · [RViz2 到点](../../reports/runs/20261124_132308_multi_factory_indoor_01_multi_4wis_cJsFyS/screenshots/rviz-final.png) · [录包元数据](../../reports/runs/20261124_132308_multi_factory_indoor_01_multi_4wis_cJsFyS/bag/metadata.yaml)

移动中停止了本次仿真的 robot2 车体桥，故障前各车线速度命令均超过 0.02 m/s。协调节点发现 robot2 反馈过期，撤销编队许可，三车均进入 `STOPPED`。三路 Twist 和退出前 Gazebo 真值位移步长均为零。 [失联结果](../../reports/runs/20261124_131717_multi_factory_indoor_01_multi_4wis_M9jtuf/result.yaml) · [控制日志](../../reports/runs/20261124_131717_multi_factory_indoor_01_multi_4wis_M9jtuf/controller.log)

## 入口检查

- `--help`、场景列表和后端列表通过。108 组 dry-run 覆盖场景、实验、控制器及各任务的合法后端组合。11 组拒绝用例覆盖任务与后端错配、单车指定队形、未知值、缺参、关闭 GUI 和实车占位。
- Ctrl+C、TERM 和超时分别返回 130、143、124。检查针对本次 Launch 进程的 PID 和环境，三种用例均确认退出时没有存活子进程。 [中断记录](../../reports/runs/20261124_131601_multi_factory_indoor_01_multi_4wis_oqG52D/result.yaml) · [TERM 记录](../../reports/runs/20261124_132404_multi_factory_indoor_01_multi_4wis_CBo7Ut/result.yaml) · [超时记录](../../reports/runs/20261124_131611_single_factory_indoor_01_continuous_mode_path_n9oQJ7/result.yaml)
- 路由、跟踪、车体响应和场景几何共 29 项 Python 回归通过。测试包含按 ID 重排、跨车时间戳不一致、错误模式和共同未来时刻准入。

## 完整路线与实车状态

室内工厂的 `--task multi --experiment full_route` 在 15.17 s 时由规划器以 `CORRIDOR_AUDIT_WIDTH_LIMIT` 拒绝。连续走廊认证未闭合，规划器未发布可执行编队轨迹，三车命令都保持为零。完整运输任务仍需通过该几何认证。 [完整路线记录](../../reports/runs/20261124_133055_multi_factory_indoor_01_multi_4wis_krqcNa/result.yaml)

实车适配配置保留设备 ID、按车辆命名空间映射的状态与命令 Topic、ROS1/ROS2 桥配置、反馈超时和急停 Topic。当前实车占位在 ROS 启动前返回 2。
