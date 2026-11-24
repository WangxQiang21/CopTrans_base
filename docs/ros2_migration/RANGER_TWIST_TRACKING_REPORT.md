# Ranger Mini 2.0 单车多模式 Twist 跟踪实施与验收

2026-11-23。按附件正文 V0.2 执行，附件文件名标记为 V0.1。实施环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Harmonic 8.15.0，工作分支为 `ros2-humble`，当前提交为 `956bc64`。现有未提交工作已保留，本轮代码和文档也留在工作区供检查。

## 验收结果

| 阶段 | 交付与结果 |
|---|---|
| C0 接口审查 | 核对 CopTrans 位姿、带符号速度、模式枚举和模式停车窗口，核对 Ranger 驱动的 Twist 选模优先级 |
| C1 单车接口 | Gazebo 单车独立 Twist、里程计及模式反馈通过直行、转弯、Spin、横移和断流停止测试 |
| C2 采样与安全过滤 | 非均匀轨迹按时间采样，发布前执行 Ackermann 半径和加速度约束。预检查接纳 30 条、拒绝 6 条 |
| C3 控制器 | 非线性反馈和前馈 PID 共用安全过滤与状态机。12 项控制核心测试与 3 项仿真响应测试通过 |
| C4 轨迹接入 | 合成轨迹覆盖三种单模式和混合模式。7 条原生 `TimedTrajectory` 进入对照组，实时 CopTrans O→S 闭环通过 |
| C5 重复对照 | 1,200 次 Gazebo 试验全部完成，1,040 次计划内切换全部确认。Ackermann 半径越界、模式不一致和被动 Spin 均为 0 |

**默认控制器为非线性反馈**。扰动组的位置 RMSE 均值为 1.28 cm，PID 为 1.35 cm。PID 的航向 RMSE 均值较低，为 1.20°，非线性反馈为 1.27°。两种方法均满足本轮模式与约束验收，继续保留 PID 对照。正式跟踪精度门槛待结合论文目标和实车平台确定，本轮暂未引入 NMPC。

## 接口与参数来源（C0）

| 接口 | 核对结果 |
|---|---|
| 规划输入 `/plan/timed_trajectory` | `fwis_v3_msgs/msg/TimedTrajectory` 含 Header、`uint32 plan_id` 和 `TimedPoint[]`。持续发布由 Planner 负责，控制器读取同一份消息 |
| 轨迹点 | `time_from_start` 为 `builtin_interfaces/Duration`，相对 `header.stamp`，秒与纳秒。原生样本存在非均匀时间间隔 |
| 速度与模式 | `v` 是 A/O 段的带符号平移速度，S 段为 0。枚举为 Ackermann 0、Oblique 1、Spin 2 |
| 参考位姿 | `center_pose` 是 `map` 下的位置与 yaw，规划模型以车体中心为参考。当前仿真中 `map` 与 Gazebo 世界的平面坐标重合，车体参考点映射到 `ranger1/base_link` |
| O/S 语义 | CopTrans O 为定航向平移，S 为中心固定的旋转。预检查会核对整段航向或中心漂移 |
| 实车参考 | Ranger Mini 2.0 先判定非零 `vy` 为 Parallel，随后用 `abs(vx)/abs(wz)` 选择 A 或 S。实车采集的 ROS 1 驱动记录阈值为 0.47644 m，上游固定 ROS 2 Mini V2 源码为 0.4764 m |
| 三车历史入口 | 原 `/ranger_sim/cmd_vel` 仍用于三车基础仿真。正式单车节点使用各自独立的命令、反馈和命名空间 |

字段和参考点依据 [TimedPoint](/home/wq/Whut/CopTrans_base/ros2_ws/src/fwis_v3_msgs/msg/TimedPoint.msg)、[TimedTrajectory](/home/wq/Whut/CopTrans_base/ros2_ws/src/fwis_v3_msgs/msg/TimedTrajectory.msg)、[CopTrans 采样实现](/home/wq/Whut/CopTrans_base/ros2_ws/src/fwis_backends/fwis_backends/single_continuous_mode_path/continuous_retiming.py:670)、[Ranger Twist 接口核查](/home/wq/Whut/CopTrans_base/docs/ros2_migration/RANGER_TWIST_CONTROL.md)。

| 参数 | 本轮取值与来源 |
|---|---|
| 外形、轴距与轮距 | 0.738 × 0.500 × 0.338 m，轴距 0.494 m，轮距 0.364 m。来自方案和厂家资料，与规划参数一致。控制核心使用等效车体模型 |
| Ackermann 机械半径参照 | 0.810 m，方案和厂家资料所列。手册最高速度 1.5 m/s 用作车型背景，本轮执行速度上限见下项 |
| 控制器工作半径 | `max(0.810, 0.47644) + 0.02 = 0.830 m`。0.02 m 为本轮仿真设计裕量。旧接口文档的 0.55 m 是软件切换限幅示例 |
| 控制器速度、加速度 | 0.28 m/s、0.55 rad/s、0.45 m/s²、0.70 rad/s²。25 Hz 控制，均为调试配置 |
| 仿真执行 | 50 Hz，一阶响应时间常数 0.075 s（平移）、0.095 s（偏航），变化率限值 0.65 m/s²、0.90 rad/s²。模式确认延迟 0.10 s，断流阈值 0.25 s |
| 切换与超时 | 速度停稳门槛 0.012 m/s 和 0.020 rad/s，稳定 0.08 s、确认超时 0.30 s、确认后等待 0.08 s。默认静止窗口下限 0.54 s。反馈过期阈值 0.20 s，计划接纳窗 0.70 s |

仿真时间常数、限幅、扰动幅度和停车门槛待实车标定。当前实车固件版本及其额外限制也待核实。 [AgileX Ranger Mini 2.0 参数表](https://new.agilex.ai/raw/upload/20230816/RANGER%20mini%202.0%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8CV1.0-20230316%28%E4%B8%AD%E8%8B%B1%E5%8F%8C%E8%AF%AD%29_71720.pdf) · [控制器配置](/home/wq/Whut/CopTrans_base/ros2_ws/src/fwis_control/config/ranger_tracking.yaml) · [车体仿真配置](/home/wq/Whut/CopTrans_base/ros2_ws/src/ranger_mini_gz/config/body_twist.yaml)

## 实现（C1–C3）

- **仿真接口。** `/ranger1/cmd_vel` 为 `geometry_msgs/msg/Twist`，只使用 `linear.x`、`linear.y`、`angular.z`。`/ranger1/odom` 为 `nav_msgs/msg/Odometry`，使用 Gazebo PostUpdate 读取到的真实模型位姿，两帧实际位姿差分得出车体速度。该反馈代表仿真状态。`/ranger1/motion_state` 为 `std_msgs/msg/Int32`，给出 0/1/2 模式。单车也可以独立于跟踪节点启动，直接通过 Twist 调试。
- **插值与预检查。** 以相邻样本的时间差和最短 yaw 增量恢复 SE(2) 车体速度，同模式内插值。异模式相邻点必须在停车容差内，保持原有切换标签、位置和时间。检查时间、速度一致性、半径、模式运动学、加速度和切换窗口，返回带时间区间和量值的 `PASS / WARN / FAIL`。WARN 目前用于声明速度与差分速度之间的小幅偏差，运动学越界按 FAIL 处理。拒绝结果通过 `/ranger1/controller_status` 发布。
- **控制器。** 两种算法都在车体系计算误差。非线性 A 段用 `v = vr·cos(eθ) + kx·ex` 和 `w = wr + ky·vr·ey + kθ·|vr|·sin(eθ)`。依当前体误差定义，对理想未限幅模型取 `V = (ex² + ey²)/2 + (1 − cos(eθ))/ky`，则 `Vdot = −kx·ex² − kθ·|vr|·sin²(eθ)/ky`。这是局部跟踪设计依据，停车点、混合切换和饱和下的性能由试验检查。O 段用平面前馈和位置反馈，S 段用偏航前馈和航向反馈。PID 对照使用积分限幅、抗饱和和滤波微分。 [Kanayama 等运动学跟踪方法](https://calhoun.nps.edu/bitstream/10945/40159/1/kanayama_a_stable.pdf)
- **最终 Twist 约束。** 速度幅值、模态约束和指令增量使用共同的过滤器。Ackermann 每个发布点满足 `vy = 0` 且 `0.830 × |wz| ≤ |vx|`，减速与换向同时收紧偏航包络。一个符号分支内采用共同插值系数限制加速度，反向通过零速。O 强制 `wz = 0`，S 强制 `vx = vy = 0`。过期与异常使用零 Twist 并记录原因。
- **执行状态机。** 进入预定停车窗口后，实测速度持续低于阈值且指令归零，才请求下一模式。仿真通过 `/ranger1/sim_request_mode` 请求零速模式握手，等待 `motion_state` 确认和稳定后继续原轨迹时间轴。零 Twist 本身无法区分三种模式，这条握手仅用于 Gazebo。模式或停车窗口超时触发停止。Gazebo 适配器负责模型位姿和仿真确认，ROS 适配层负责时间、消息与诊断，控制核心可脱离 ROS 和 Gazebo 执行。 [Gazebo Harmonic Python 接口](https://gazebosim.org/api/sim/8/python_interfaces.html)

## 试验与结果（C4–C5）

接受的 30 条轨迹由 23 条合成轨迹和 7 条 CopTrans 原生轨迹组成。合成集覆盖变速直线、倒车、左右转弯、不同速度的 S 曲线、近边界半径、多个斜移角、跨 ±π 和两圈旋转，以及包含三种模式的组合。原生轨迹覆盖 A 直行与转弯、O 横移、S 自旋、O→S。通过 Planner 已有启动参数使用低速和受限时序配置，记录原始 CDR 和同字段 JSON。规划算法、ROS 消息、时间戳、位姿与模式均沿用 Planner 输出。

6 条预期不可行轨迹全部拒绝。默认 CopTrans 直线峰值约 1.15 m/s，超出 0.28 m/s。早期低速圆弧约 0.81 m，超出本轮半径包络。另有两条 CopTrans 轨迹的初始停车窗口为 0.30 s。受限时序配置的再采集轨迹通过检查。合成集另有小半径和无停车窗口各一条拒绝用例。 [预检查明细](../../../reports/ranger_twist_tracking/comparison/preflight.json)

每条通过的轨迹按控制器与条件各重复 10 次。扰动组使用相同种子配对，范围含每轴 ±0.025 m、初始 yaw ±4°、小初速、响应时间倍率 0.75–1.45、命令延时 0–55 ms、反馈延时 0–25 ms。加速批量试验以 Harmonic TestFixture 步进简化静态车体实体，沿用在线的执行响应和 ECM 位姿差分。每次从实测模型状态闭环，规划文件保留原始时间戳，离线回放仅为试验显式指定仿真起点时间。批量计算耗时反映该机上控制核心，不包括进程间传输。

| 条件 | 控制器 | 完成 | 位置 RMSE 均值 / P95 / 最差 | 航向 RMSE 均值 / P95 / 最差 |
|---|---|---:|---:|---:|
| 标称 | 非线性 | 300/300 | 0.39 / 0.64 / 0.72 cm | 0.25 / 0.53 / 0.54° |
| 标称 | PID | 300/300 | 0.41 / 0.69 / 0.79 cm | 0.24 / 0.54 / 0.61° |
| 扰动 | 非线性 | 300/300 | 1.28 / 2.46 / 3.11 cm | 1.27 / 3.02 / 3.88° |
| 扰动 | PID | 300/300 | 1.35 / 2.46 / 3.11 cm | 1.20 / 3.02 / 3.88° |

P95 和最差值按单次试验的 RMSE 统计。全数据的单时刻最大位置误差 4.63 cm，最大航向误差 5.59°。覆盖 1,040 次模式切换，切换过程最长 0.56 s。指令最大变化率为 0.45 m/s² 和 0.70 rad/s²，未触发 Ackermann 越界及被动 Spin。两圈用例的实测累计转角为约 1.993–2.012 圈。扰动组中非线性的位置 RMSE 在 246/300 对较低，PID 的航向均值更低。核心计算时间的单次试验 P99 最差值分别约 0.078 ms（非线性）与 0.082 ms（扰动 PID），全组无 40 ms 周期超时。

首轮批量试验保留了 42 次停车窗口耗尽记录。根因是仿真初速度反向时，响应模型的指数衰减无法跨过原方向。最终版在单步加速度余量允许时归零，增加了有限时间换向测试，并使用同一组扰动种子重跑。

[原始和逐次指标](../../../reports/ranger_twist_tracking/comparison/trials.csv) · [分轨迹、分模式、分来源及成对统计](../../../reports/ranger_twist_tracking/comparison/summary.json) · [误差分布图](../../../reports/ranger_twist_tracking/comparison/comparison_errors.png) · [混合轨迹与模式确认时序](../../../reports/ranger_twist_tracking/comparison/mixed_trace.png) · [首轮记录](../../../reports/ranger_twist_tracking/comparison_initial/summary.json)

两种实时 ROS 2 闭环也已复测。非线性反馈完成合成 A→O→S→A，终点位置误差 0.91 cm。PID 从运行中的 CopTrans 直接接收原生 O→S 轨迹，完成两次模式确认，终点位置误差 0.51 cm。合成实时联调还验证了小半径轨迹被拒绝且发令保持为零。 [实时合成轨迹诊断](../../../reports/ranger_twist_tracking/live_synthetic_status.jsonl) · [实时原生 CopTrans 诊断](../../../reports/ranger_twist_tracking/live_native_status.jsonl) · [单车接口测试](../../../reports/ranger_twist_tracking/t0.log)

原三车入口启动后逐车更新位姿，三套 TF 与 Gazebo 的位置和航向保持同步。 [三车入口回归](../../../reports/ranger_twist_tracking/legacy_three_pose.log)

## 复现

当前系统已具备 Harmonic Python 包 `python3-gz-sim8`、`python3-gz-transport13`、`python3-gz-msgs10`。构建与单元测试从 ROS 2 工作空间执行。

```bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ranger_mini_description ranger_mini_gz fwis_control
source install/local_setup.bash
colcon test --packages-select ranger_mini_gz fwis_control
```

用同一隔离 ROS 环境启动单车，再从另一个加载了工作空间的终端发布测试轨迹或启动 Planner。

```bash
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1
GZ_PARTITION=whut_single_tracking ros2 launch fwis_control tracking.launch.py algorithm:=nonlinear
```

```bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export ROS_DOMAIN_ID=174 ROS_LOCALHOST_ONLY=1
python3 src/fwis_control/test/smoke_tracking_ros.py --output /tmp/ranger_synthetic_status.jsonl
```

原生实时联调时，控制启动传入 `initial_x:=3.0 initial_y:=4.0`。在同一 ROS 域另起 Planner，运行 smoke 脚本时添加 `--native-planner`，由脚本发布地图和目标。独立采集命令的参数见 `python3 src/fwis_control/test/capture_coptrans.py --help`。

```bash
export PYTHONPATH="/home/wq/Whut/.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
ros2 launch fwis_bringup plan.launch.py backend:=single_continuous_mode_path use_sim_time:=true rviz:=false \
  config_file:=/home/wq/Whut/reports/ranger_twist_tracking/coptrans_restricted_profile.yaml
```

重跑批量矩阵与对比图，输出使用新目录。

```bash
reports=/home/wq/Whut/reports/ranger_twist_tracking
python3 src/fwis_control/test/benchmark.py \
  --output /tmp/ranger_tracking_reproduction --repeats 10 --workers 6 \
  --captures "$reports"/coptrans_default_straight.json "$reports"/coptrans_low_turn.json \
    "$reports"/coptrans_tracking_{straight,turn,oblique,spin}.json "$reports"/coptrans_restricted_*.json \
  --expect-rejection coptrans_default_straight coptrans_low_turn coptrans_tracking_oblique coptrans_tracking_spin
python3 src/fwis_control/test/plot_tracking.py /tmp/ranger_tracking_reproduction
```

## 变更清单与边界

- 新增 `fwis_control`，包含采样与预检查、安全过滤、模式执行状态机、两种控制器、ROS 节点、配置、启动和试验脚本。
- 扩展 `ranger_mini_gz` 的单车模式和 Gazebo 车体适配器。扩展 `ranger_mini_description` 的仿真模型加载选项并增加单车 RViz 配置。
- 扩展工作空间 README 和 Ranger 接口文档，新增本报告及工作区报告目录下的配置快照、原始日志、轨迹、统计和图。
- 本轮验收范围为单车运动学仿真。实车配置、模式过渡、执行延时和控制参数待标定。长期规划耗时导致 Header 过期时，现有接入会拒绝执行，需要在实际部署流程中确认规划时间和接纳窗口的约定。
