# CopTrans ROS 2 迁移实施与验收

**实施日期** 2026-11-23。仓库位于 `/home/wq/Whut/CopTrans_base`，变更在本地 `ros2-humble` 分支，基线为 `continuous-mode-path@956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee`。四个 ROS1 源引用及基线 `catkin_ws` 均按来源保留。当前分支保留未提交的实施变更，以便检查。

| 阶段 | 结果 | 交付路径 |
|---|---|---|
| P0 源码审查 | 通过。对照四个提交、五种公共消息和八种多车消息，核对算法和 ROS1 接口 | 本目录五份审查文件 |
| P1 环境 | 通过。系统安装 Harmonic 8.15.0 和 ROS-GZ 0.244.12，启动空世界、RViz2 并桥接时钟 | `/home/wq/Whut/reports/gazebo_harmonic_install.md` |
| P2 模型 | 通过。单双窗口确认单车及三车网格可见；Gazebo 逐车修改位置和航向，TF 和 RViz2 更新 | `ranger_mini_description`、`ranger_mini_gz` |
| P3 公共基础 | 通过。Ament/Colcon 构建公共头文件、ROS2 适配和 13 种消息 | `fwis_common`、`fwis_ros`、`fwis_v3_msgs` |
| P4 算法 | 通过。四个默认后端分别运行。同一空地图的轨迹逐点一致，障碍用例拒绝原因一致 | `fwis_backends` |
| P5 启动与输出 | 通过。统一 Launch 可切换四个后端，单车和编队在 RViz2 显示实际地图、轨迹及 Marker，回放 smoke 通过 | `fwis_bringup`、`ros2_ws/README.md` |

## 迁移结果

- 新增独立 `ros2_ws/src`，7 个包整体通过 Colcon 构建。修改仓库的 `.gitignore`；Git 中无文件删除。完整增改清单在 `/home/wq/Whut/reports/coptrans_migration_files.tsv`。
- 保留独立的三套单车 C++ 搜索、Python 优化器及回放。多车采用指定分支自带的默认 `virtual_rigid_a` 流程。分支与目录见 [来源矩阵](BRANCH_BACKEND_MATRIX.md)。
- 地图接口为 `nav_msgs/msg/OccupancyGrid`，ROS1 消息时间字段换为 ROS2 内置类型。适配修改集中在构建、参数、QoS、ROS 时间、消息构造和 TF，算法权重和运动模型沿用各源分支。
- 官方 Ranger Mini V2 素材来自 AgileX `ugv_gazebo_sim@27633a956c845903ee630538afeb17fe70afdd84`。同套 Xacro 和网格供两个可视化程序使用。AgileX `ranger_ros2:humble@b6ea21a275ca5e7168130cc6470e61474681d679` 用于参考车辆消息。两个参考仓库位于 `/home/wq/Whut/third_party`。
- 三车试验把三台车从 (0, 0)、(0, 1.5)、(0, 3) m 分别移动到 (1.05, −0.30)、(1.35, 1.20)、(1.65, 2.70) m，设置航向 0.15、0.30、0.45 rad。每次移动时另外两车的 TF 保持试验预期。实际 TF、Gazebo 图和 RViz2 图保存在 `/home/wq/Whut/reports/p2`。系统版 Harmonic 三车试验复测通过。

## 规划回归

对每个源分支读取并运行未经修改的 ROS1 规划入口。ROS1 基线使用工作区内隔离的 Jammy ROS 1.15.14 组件和原始消息定义，ROS2 采用 Humble。两者使用同一张 200 × 160、0.1 m/格的空地图，同一起终点 (3.0, 4.0) 和 (5.8, 4.0) m，以及同一 CasADi 3.7.2 / IPOPT。比较每个采样的 X、Y、航向、运动模式、纳秒时间和速度。

| ROS2 后端 | 每车点数 | 末时刻（秒） | 结果 |
|---|---:|---:|---|
| `single_multimodal_hybrid_astar` | 55 | 4.851881814 | 与原版逐点相等，终点误差约 0.015 m |
| `single_constant_primitive` | 249 | 4.864070632 | 与原版逐点相等 |
| `single_continuous_mode_path` | 103 | 4.864070632 | 与原版逐点相等 |
| `multi_4wis` | 333 × 3 | 6.628125001 | 与原版逐点相等，三组车间刚性距离通过 |

在同一地图的目标区域加入占据单元，四个后端均拒绝目标且未输出轨迹。三套单车的 ROS1/ROS2 均返回 `search_no_path_in_discretized_lattice`；多车的 ROS1/ROS2 均返回 `NO_ROUTE_GUIDE`。

结果证据见 `/home/wq/Whut/reports/regression`。以 `python3 /home/wq/Whut/reports/regression/compare.py` 复核已保存数据。直线用例覆盖接口、回放、基本轨迹与刚性关系；复杂场景、历史数据采集 UI 以及多车可选旧流程的完整性能回归留待后续覆盖。原分支没有在真实车上运行。

## 复现

按 [ROS2 工作空间说明](../../ros2_ws/README.md) 构建。参考 checkout 采用部分克隆，基线 Catkin 文件已检出。其他历史目录可按需用 `git sparse-checkout add` 加载。

```bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash
export PYTHONPATH="/home/wq/Whut/.coptrans-python${PYTHONPATH:+:$PYTHONPATH}"
ros2 launch ranger_mini_gz display.launch.py robots:=3
# 单独终端运行规划
ros2 launch fwis_bringup plan.launch.py backend:=multi_4wis
# 在对应 Launch 已启动时运行验收
python3 src/ranger_mini_gz/test/smoke_pose_sync.py --robots 3
python3 src/fwis_bringup/test/smoke_plan.py --multi
```

仿真验收包含模型显示和位姿同步。场景生成、轮端动力学、闭环控制和实体车通信属于后续阶段。
