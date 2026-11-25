# Ranger Mini 工业运输场景

三套场景由 `ranger_mini_gz/scenarios/` 中的 YAML 生成。场景 YAML 定义世界尺寸、障碍几何、相机和实验起终点。生成器用同一组碰撞几何写出 Gazebo SDF 和 ROS 地图。

| 场景 | 场地 | 默认栅格 | 布局 |
| --- | --- | --- | --- |
| [室内工厂](factory_indoor_01/) | 30 × 24 m | 600 × 480 | CNC、生产线、单侧凸出的控制柜、黄色围栏和 90° 转弯区 |
| [港口维护区](port_01/) | 42 × 30 m | 840 × 600 | 20/40 英尺集装箱通道、港机支腿、防撞柱、平板车和维修区 |
| [室外工厂](factory_outdoor_01/) | 44 × 32 m | 880 × 640 | 钢材堆放区、斜停平板车、架空管廊和设备安装区 |

室内场景的转弯区预留 5.5 × 5.6 m，货架之间另有一段 1.35 m 的单车通道。当前仿真入口选用 `west_single_lane`，在西墙与西侧货架之间新增 1.10 m 限宽口。此处的室内工厂工件和历史验收使用 `default` 版本。起终点坐标、参考转弯的位置和各车 Spawn 均保存在各场景的 `scene_meta.yaml`。

## 正式运输测试

```bash
cd /home/wq/Whut/CopTrans_base
./scripts/test.sh --task multi --scenario factory_indoor_01
```

统一入口会同时打开 Gazebo 和 RViz2，并用当前场景的地图、反馈和规划器输出执行短弯闭环。场景、后端和实验参数见 [ROS 2 工作空间说明](../../README.md)。

## 内部场景预览

```bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
ros2 launch fwis_bringup simulation.launch.py scenario:=factory_indoor_01 task:=formation
```

- `scenario` 可选 `factory_indoor_01`、`port_01`、`factory_outdoor_01`。用 `task:=single` 启动单车场景。
- 用 `resolution:=0.025` 加载室内精度对照网格。加 `gui:=false rviz:=false` 可只启动 Gazebo 服务端和 ROS 接口。
- RViz 的默认俯视图用于检查地图。选 **Panels → Views → Loading bay** 可近看装载区的三个车体中心支座。

## 瓶颈与货物

| 室内版本 | 几何净宽 | 用途 |
| --- | ---: | --- |
| `variant:=west_single_lane` | 南侧 3.20 m，西侧 1.10 m | 当前默认布局，西侧限宽口仅供单车通过 |
| `variant:=default` | 3.20 m | 历史布局和归档回放 |
| [`variant:=wide`](factory_indoor_01_wide/) | 3.60 m | 宽松通道 |
| [`variant:=critical`](factory_indoor_01_critical/) | 3.00 m | 临界净空实验 |
| [`variant:=too_narrow`](factory_indoor_01_too_narrow/) | 2.60 m | 固定朝向通过时，单车有净空，Cargo 与瓶颈相交 |

这些结果对应 YAML 中同一条参考线与朝向。几何检查分别对三辆车和 Cargo 计算净空，参考线采样平移步长上限为 0.05 m，转角步长上限为 0.01 rad。三个默认场景的 Cargo 参考线最小净空依次为 0.25、0.23 和 0.55 m，室内 3.00 m 版本为 0.05 m。2.60 m 是指定直线及朝向的碰撞样例。

车辆平面尺寸沿用 `fwis_common/config/ranger_mini_2.yaml` 的 0.738 × 0.500 m，与当前 URDF 中心碰撞盒相符。URDF 碰撞盒的高度为 0.210 m。车模以 `base_link` 为车体控制中心，Collada 单位由模型文件声明为毫米，Gazebo 根高度 0.322 m 使轮胎贴地。三车偏移与 `4wis_mutil_mode_plan01` 分支核对为 `(1,0)、(0,1)、(0,-1)`，初始朝向使用 `cargo_heading`。

Cargo 尺寸及偏移集中保存在 `ranger_mini_gz/config/cargo_geometry.yaml`。工业设备模块初值为 1.95 × 2.70 × 1.10 m，中心在编队参考点前方 0.50 m。三处支座放在车体中心正上方。Gazebo 里的设备有独立的 Cargo 碰撞盒、半透明外罩和示意支座。

## 地图与规划接口

世界 XY 与 `map` 同向。地图原点为 (0, 0)，yaw 为零。占据值为 0 和 100。旋转长方体、圆柱和凸多边形使用各自真实的 XY 投影，任一方格与障碍投影相交即设为占据。该栅格化在 0.05 m 和 0.025 m 下的保守边界容差分别为 0.071 m 和 0.035 m。水面、架空管廊横梁和地面标线仅作视觉提示，地图描述地面环境障碍物本体。

| 接口 | 数据来源或用途 |
| --- | --- |
| `/map` | 从生成的 PGM 发布 `OccupancyGrid`，Reliable、Transient Local、深度 1 |
| `/clock` | Gazebo 仿真时钟，经 ROS-GZ 桥接 |
| `/robotN/ground_truth_pose` | 由 Gazebo 模型位姿生成的 `PoseStamped`，父帧为 `map` |
| `/robotN/odom` | 同一实测位姿和仿真时间差分所得的车体速度，子帧为 `robotN/base_link` |
| `/scene/markers` | 场景中的 Cargo 和中心支座，采用与 SDF 相同的几何和位置 |

上述场景展示使用静态 Ranger 模型。规划起点来自实验的显式请求，验收脚本先核对该起点与 Gazebo Ground Truth 的一致性。

| 后端 | 地图 | 任务与状态输入 | 主要输出 |
| --- | --- | --- | --- |
| `single_continuous_mode_path` | `/map` | `/start_pose` 或 `/initialpose`，`/move_base_simple/goal` | `/plan/path`、`/plan/timed_trajectory`、`/plan/status` |
| `multi_4wis` 默认 `virtual_rigid_a` | `/map` | `/initialpose` 初始化，`/formation/plan_request` 提供编队参考位姿和车辆状态 | `/formation/playback_plan`、`/formation/path/vehicle_N`、`/formation/optimizer_status` |

规划器使用显式初始化和任务请求。场景验收用独立的 Cargo OBB 逐样本核查规划结果，编队后端内部认证使用各车辆 footprint。参考线净空、规划器认证与 Gazebo 动力学属于不同验收项。

## 早期场景规划验收复现

在启动场景的同一 ROS Domain 中，另开终端加载工作空间及已有的 CasADi 路径，再启动对应规划器。

```bash
source /opt/ros/humble/setup.bash
source /home/wq/Whut/CopTrans_base/ros2_ws/install/local_setup.bash
export PYTHONPATH="/home/wq/Whut/.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
ros2 launch fwis_bringup plan.launch.py backend:=multi_4wis use_sim_time:=true rviz:=false
```

在第三个终端加载同一工作空间并运行与当前场景匹配的请求。单车后端用 `backend:=single_continuous_mode_path`，将请求类型改为 `single`。

```bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
python3 src/fwis_bringup/test/probe_scene_plan.py factory_indoor_01 formation
python3 src/fwis_bringup/test/probe_scene_plan.py factory_indoor_01 formation --goal 7.8 5.6 0
```

每项用新启动的规划实例运行。RViz 场景视图可直接显示对应的规划 Path。生成某个场景的离线工件及运行场景几何回归测试的命令如下。

```bash
python3 -m ranger_mini_gz.scene_generator factory_indoor_01 generated/scenes/factory_indoor_01 --variant default
gz sdf -k generated/scenes/factory_indoor_01/world.sdf
python3 -m pytest -q src/ranger_mini_gz/test/test_scenes.py
```

`world.sdf` 包含 Cargo，`world_single.sdf` 为单车世界。仿真启动时按实验位姿加载 Ranger。各目录中的 `manifest.yaml` 记录生成文件的 SHA-256。室内目录另含 `map_0025.pgm` 和 `map_0025.yaml`。

## 实机软件验收

在本机 Gazebo Harmonic 和 ROS 2 Humble 验证。三张默认地图的实际 `/map` 与生成栅格逐格一致，Gazebo 初始位姿与实验起点一致。12 份场景 SDF 通过 `gz sdf -k`，场景几何、底盘响应与单车跟踪共 23 项回归测试通过。室内 0.025 m 地图测试了 1200 × 960 的晚加入订阅以及三辆车的里程计。

| 场景 | 单车完整任务长度 | 单车最小净空 | 三车 3.8 m 直行 Cargo 净空 | 三车完整任务 |
| --- | ---: | ---: | ---: | --- |
| 室内工厂 | 23.64 m | 0.135 m | 0.25 m | 走廊连续认证失败，`CORRIDOR_AUDIT_WIDTH_LIMIT` |
| 港口维护区 | 31.93 m | 0.202 m | 0.23 m | 55 秒观察窗口到期 |
| 室外工厂 | 34.06 m | 0.144 m | 0.55 m | 求解超过软预算，`SOFT_BUDGET_EXHAUSTED` |

三段单车完整轨迹和三段三车短程轨迹的逐样本净空均为正。三车短程均收到 `virtual_a_geometry_dynamics_collision/v1` 认证的轨迹，Cargo 净空由场景检查器独立复核。日志摘要、耗时、轨迹时间和最小净空位置分别保存在各默认场景的 `plan_single.json`、`plan_formation_short.json` 和 `plan_formation_full.json`。

| 场景 | Gazebo 截图 | RViz 截图 |
| --- | --- | --- |
| 室内工厂 | [Gazebo](factory_indoor_01/gazebo.png) | [RViz](factory_indoor_01/rviz.png) |
| 港口维护区 | [Gazebo](port_01/gazebo.png) | [RViz](port_01/rviz.png) |
| 室外工厂 | [Gazebo](factory_outdoor_01/gazebo.png) | [RViz](factory_outdoor_01/rviz.png) |
