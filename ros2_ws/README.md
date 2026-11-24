# CopTrans ROS 2 工作空间

本工作空间使用 Ubuntu 22.04、ROS 2 Humble 和 Gazebo Harmonic。四个来源提交、默认算法、接口和审查记录见 [迁移文档](../docs/ros2_migration/P0_AUDIT_REPORT.md)。

## 统一仿真测试入口

```bash
cd /home/wq/Whut/CopTrans_base
./scripts/test.sh
./scripts/test.sh --task multi --formation wedge
./scripts/test.sh --task multi --experiment full_route --scenario factory_indoor_01
./scripts/test.sh --scenario port_01 --planner-backend hybrid_astar
./scripts/test.sh --task multi --controller-backend pid --record on
```

默认在室内工厂跑单车低速短弯，使用连续模式路径规划与非线性反馈跟踪。多车默认使用楔形队形和 `uniform_v02` 组合运动搜索，三辆车各自跟踪同一时间轴上的轨迹。每次仿真同时打开 Gazebo 和 RViz2，运行完成后保留两窗五秒。按 Ctrl+C 可结束本次测试。

RViz2 上方显示三车与货物状态，下方显示各自的平面跟踪误差（cm）。货物误差按三车实测位姿拟合出的货物基准点与同一执行时刻的规划基准点计算，安装点约束误差单独显示。单车测试仅更新白色 V1 的反馈。

| 参数 | 可选值 |
| --- | --- |
| `--task` | `single`、`multi` |
| `--scenario` | `factory_indoor_01`、`port_01`、`factory_outdoor_01` |
| `--planner-backend` | 单车可选 `continuous_mode_path`、`hybrid_astar`、`constant_primitive`，多车用 `multi_4wis` |
| `--controller-backend` | `auto`、`nonlinear_feedback`、`pid` |
| `--experiment` | `short_turn`、`short_straight`、`full_route` |
| `--formation` | 多车使用 `wedge` |
| `--record` | `off`、`on`，开启后保存 rosbag |
| `--timeout` | 启动后的墙钟秒数，短程默认 180，`full_route` 默认 900 |
| `--no-build` | 使用已构建的工作空间 |
| `--dry-run`、`--list-scenarios`、`--list-backends`、`--help` | 查看配置与参数 |

每次运行的命令、解析配置、规划器日志、控制器日志、轨迹 CDR、观测数据和终态写入 `reports/runs/<run_id>/`。终端同时打印本次 ROS 域和 Gazebo 分区，便于连入只读的诊断工具。 `--mode real` 当前返回 2 并在加载 ROS 环境前退出。

实跑矩阵、接口和截图见 [CopTrans V1.1 联调验收](../docs/ros2_migration/COPTRANS_TEST_INTEGRATION_V1.1.md)。

## 内部调试与历史复现

以下命令用于定位单独的场景、显示、规划或控制模块。正式端到端测试使用上面的统一入口。

### 构建和载入

```bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
source /opt/ros/humble/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --parallel-workers 3
source install/local_setup.bash
# 本机验收使用的 CasADi 3.7.2 / IPOPT
export PYTHONPATH="/home/wq/Whut/.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
```

其他机器请在 ROS 2 使用的系统 Python 3.10 中提供 NumPy、SciPy、PyYAML 和 CasADi 3.7.2。恒定原语算法与多车默认算法的求解流程需要 CasADi。使用工作区内的 Harmonic 软件包时，在 `source install/local_setup.bash` 前执行 `source /home/wq/Whut/.gazebo-harmonic/setup.bash`。

## Ranger Mini 2.0

```bash
ros2 launch ranger_mini_gz display.launch.py robots:=1
ros2 launch ranger_mini_gz display.launch.py robots:=3
```

无图形测试可附加 `gui:=false rviz:=false`。模型为静态显示，Gazebo 模型 Pose 通过 `ros_gz_bridge` 进入对应 ROS 命名空间，根 TF 为 `world → robotN/base_link`。RViz 的 RobotModel 从各自的 `robot_description` 和 `robot_state_publisher` 固定轮端 TF 读取。三台车使用同一套原厂网格。

在启动的同一终端环境中打开另一终端，运行位姿测试或直接移动某一台车。

```bash
python3 src/ranger_mini_gz/test/smoke_pose_sync.py --robots 3
gz service -s /world/whut/set_pose/blocking \
  --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'name: "robot2" position {x: 1.35 y: 1.2 z: 0.322} orientation {w: 1}'
```

模型参考坐标为车体中心，前为 X、左为 Y、上为 Z。轮端固定在原厂 xacro 零转角，关节原点为 X ±0.23 m、Y ±0.206 m、Z −0.10 m，保留原厂视觉偏移和旋转。描述用 0.738 × 0.500 × 0.210 m 简化车体碰撞盒。Gazebo 加载后的轮胎最低点相对 `base_link` 约为 −0.32194 m，测试世界将根高度设为 0.322 m，使轮胎贴近地面。

### 三车 Twist 基础测试

三种模式的车体指令、实车切换边界和控制器限幅见 [Ranger Mini 2.0 车体 Twist 控制接口](../docs/ros2_migration/RANGER_TWIST_CONTROL.md)。

两个终端都加载工作空间，并设置相同的本机 ROS Domain。

```bash
source /opt/ros/humble/setup.bash
source /home/wq/Whut/CopTrans_base/ros2_ws/install/local_setup.bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
```

在第一个终端启动仿真。等待终端打印 `Ready for Twist on /ranger_sim/cmd_vel`。

```bash
GZ_PARTITION=whut_ranger_motion ros2 launch ranger_mini_gz display.launch.py robots:=3 motion_test:=true
```

在第二个终端依次发送直行、左转、原地自旋、左横移和停止。

```bash
send() {
  ros2 topic pub --rate 10 --times "$1" /ranger_sim/cmd_vel geometry_msgs/msg/Twist "$2"
}
send 50 '{linear: {x: 0.10}}'
send 50 '{linear: {x: 0.10}, angular: {z: 0.20}}'
send 60 '{angular: {z: 0.15}}'
send 50 '{linear: {y: 0.10}}'
send 1 '{}'
```

测试入口由 `motion_test:=true` 启用，使用 Harmonic 的 `python3-gz-transport13` 与 `python3-gz-msgs10`。它按实车记录的模式分支计算车体运动，批量更新 Gazebo 位姿。三车各自执行同一车体坐标系下的指令。收到全零 Twist 或 0.35 秒内未收到新命令时，模型停止更新。

### 单车多模式轨迹跟踪

`fwis_control` 订阅 `/plan/timed_trajectory`，执行已通过预检查的单车轨迹。单车仿真使用 `/ranger1/cmd_vel`、`/ranger1/odom` 和 `/ranger1/motion_state`。诊断在 `/ranger1/controller_status`，包含轨迹接纳、运行阶段、指令和过滤原因。

单车仿真使用 OSRF 的 Jammy Python 软件包 `python3-gz-sim8`、`python3-gz-transport13`、`python3-gz-msgs10`。

#### Gazebo 和 RViz 一键演示

在终端运行下列命令，脚本自动加载已构建的工作空间，打开 Gazebo 和 RViz。

```bash
/home/wq/Whut/CopTrans_base/ros2_ws/demo_ranger_tracking.sh
```

1. 摆好两个窗口，回到终端按 **Enter**。脚本发布一条合成 `TimedTrajectory`，车依次直行、左转、左横移、原地自旋 90°，最后再直行停车。整条轨迹约 26 个仿真秒，模式切换之间会短暂停稳。
2. Gazebo 和 RViz 使用白色 CopTransV1。RViz 中蓝线是参考轨迹，橙线是 Gazebo 里程计绘制的实际轨迹。两条线显示在车顶上方的同一高度附近，橙线额外抬高 3 mm 以区分重叠部分。车体坐标轴固定在控制中心 `ranger1/base_link`，X 朝前，Y 朝左，Z 朝上。原地自旋时可观察车体坐标轴转动。
3. 终端打印模式反馈和 `COMPLETE`，并报告终点位置与航向误差。两个窗口会保留，按 **Ctrl+C** 关闭。

用 PID 执行同一路线时，先结束当前演示，再加 `--algorithm pid`。加 `--auto-start` 可跳过回车提示。命令默认使用本机 ROS 域 177。

#### RViz 状态与误差

3D 视图顶部依次显示白色 V1、橙色 V2、蓝色 V3 和 Cargo。车辆状态读取各自命名空间下的 `odom` 和 `motion_state`。位置与航向在 `map` 中表示，速度分量在车体坐标系中表示。总速度是平面线速度的模，模式采用底盘反馈 0 Ackermann、1 Parallel、2 Spin。

底部四图读取对应命名空间下的 `controller_status`，纵轴绘制 `100 × hypot(body_error[0], body_error[1])`（cm），横轴为本次轨迹开始后的仿真秒数。位置误差首次有效时开始画线，完成后保留最后一条曲线。单车演示只有 `/ranger1` 提供反馈，其他卡片显示零值和「无数据」，误差图留空。

接入独立的 `/ranger2`、`/ranger3` 时沿用 V1 的消息格式。Cargo 的实测位姿使用 `/cargo/odom`（`nav_msgs/msg/Odometry`，`header.frame_id: map`），按默认编队尺寸显示半透明长方形。`/cargo/controller_status` 提供位置跟踪误差，`/cargo/constraint_error`（`std_msgs/msg/Float64`）接收编队控制器给出的距离约束误差，单位为米。

#### 独立启动跟踪节点

```bash
ros2 launch fwis_control tracking.launch.py algorithm:=nonlinear
ros2 topic echo /ranger1/controller_status
```

可选 `algorithm:=pid` 对照。需手动发单车 Twist 时，使用 `ros2 launch ranger_mini_gz display.launch.py robots:=1 body_twist:=true`。复现及数据见 [单车多模式 Twist 跟踪实施与验收](../docs/ros2_migration/RANGER_TWIST_TRACKING_REPORT.md)。

## 规划与回放

每次启动一个后端。默认 `/map` 使用可靠、transient local 的 OccupancyGrid，坐标原点不可旋转，帧为 `map`。地图由场景或上游发布。后端启动时可通过 `map_topic` 重映射。若地图发布器使用 volatile，传入 `map_transient_local:=false` 并在 RViz 的 Map 显示同步调整 Durability。

```bash
ros2 launch fwis_bringup plan.launch.py backend:=single_multimodal_hybrid_astar
ros2 launch fwis_bringup plan.launch.py backend:=single_constant_primitive
ros2 launch fwis_bringup plan.launch.py backend:=single_continuous_mode_path
ros2 launch fwis_bringup plan.launch.py backend:=multi_4wis
```

- 单车可在 RViz 用 **2D Pose Estimate** 和 **2D Goal Pose** 设定起终点，也可发布 `/start_pose` 和 `/move_base_simple/goal`。订阅 `/plan/path`、`/plan/timed_trajectory` 和 `/robot/marker` 查看结果与回放。
- 多车默认使用 `uniform_v02`。先在 `map` 中用 **2D Pose Estimate** 初始化，再用 **2D Goal Pose** 给出载荷目标。程序可发送 `/formation/plan_request`，输出包含 `/formation/playback_plan`、各车辆的 Path、MarkerArray 和状态。历史复现可通过 `formation_backend:=virtual_rigid_a` 或 `formation_backend:=legacy` 选择原后端。
- 三车回放使用与 Gazebo 一致的白、橙、蓝 Ranger 网格。半透明 **Cargo** 在车顶上方随编队位姿平移和转动。默认编队的显示尺寸为 1.40 × 2.18 × 0.05 m，不透明度为 28%。初始化后，在 **Panels → Views** 中双击 **Cargo overview** 查看全貌，双击 **Cargo side** 查看车顶和 Cargo 底面的连接。
- 三个连接机构分别位于车辆控制中心正上方。Cargo 上的固定安装点使用编队配置中的车体中心偏移 `offsets`，回放中车体绕各自安装点改变航向，安装点在 Cargo 坐标系内的平面位置保持固定。连接机构以 RViz 圆柱标记显示，尺寸用于示意。
- 在已有仿真时钟的会话中可附加 `use_sim_time:=true`。默认使用系统时间，便于独立验证规划接口。

另一终端在同一 ROS Domain 下运行固定用例。多车用例包含初始化请求和三车刚性距离断言。

```bash
python3 src/fwis_bringup/test/smoke_plan.py
python3 src/fwis_bringup/test/smoke_plan.py --multi
```

可加 `--blocked` 让地图在目标范围内包含障碍，并校验规划拒绝。每个用例在新启动的后端实例上运行。

### 多车带货长程规划

组合运动搜索使用 Ackermann 行驶、斜移和旋转调整。在候选边的搜索和最终碰撞检查中，货物包络按编队参考点的位姿旋转和平移，三辆车仍按各自航向检查车身。统一入口从场景读取货物长、宽和安装偏移。单独启动规划器时可通过 `overrides_file` 设置 `cargo_footprint.length_m`、`width_m`、`offset_x_m` 和 `offset_y_m`。

长距离直行或斜移按低速参数和单段时长上限划分，段间零速衔接，整条轨迹保留三车公共时间轴。用 `--experiment full_route` 验证场景的完整起终点，`--timeout` 控制整个仿真测试的墙钟预算。

搜索用解析运动段快速尝试连接目标，连接通过与普通候选相同的碰撞检查。货物横向及旋转扫掠结合二维占据栅格检查。设计参考了运动学搜索中的解析连接和连续碰撞检查。 [Dolgov 等人的路径搜索](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) · [Schulman 等人的连续扫掠检查](https://escholarship.org/uc/item/6km506db)

工作空间构建出 C++ 扩展后，规划器自动用它执行每个扫掠采样点的刚体坐标变换、车身及货物对栅格的碰撞检查，以及车身间检查。扩展通过 `pybind11-dev` 和 `python3-dev` 构建。缺少扩展的环境使用 Python 检查器。

规划器可独立运行。在载入同一 ROS 环境的第二个终端执行以下命令，会发布场景占据栅格和起终点，等待回放确认，并从已发布的轨迹独立检查车身与货物净空。独立启动规划器时，可复用统一测试目录中的 `planner_parameters.yaml` 来保持限速和货物尺寸一致。

```bash
python3 src/fwis_bringup/test/probe_scene_plan.py factory_indoor_01 formation --planner-only
```

## 工业运输场景

```bash
ros2 launch fwis_bringup simulation.launch.py scenario:=factory_indoor_01
```

也可选择 `port_01` 或 `factory_outdoor_01`。默认展示白、橙、蓝三辆 Ranger、半透明工业设备模块和车体中心的三个支座。加 `task:=single` 切换到单车，同一场景配置继续生成相同的环境地图。室内场景可用 `variant:=wide`、`critical` 或 `too_narrow` 切换瓶颈宽度，或加 `resolution:=0.025` 查看高分辨率地图。

场景文件、实际截图、ROS 接口、规划验收和复现命令见 [Gazebo 工业场景交付说明](generated/scenes/README.md)。
