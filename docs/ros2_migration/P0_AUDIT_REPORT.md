# P0 源码审查

审查日期 2026-11-23。通过仓库四个远程引用和对应 Git 对象读取 Catkin 源文件、构建清单、消息、配置和启动文件。源提交见 [分支矩阵](BRANCH_BACKEND_MATRIX.md)。

- 四支都有 `fwis_common`、`fwis_map_server`、`fwis_v3_msgs`、`fwis_v3_planner`、`fwis_v3_optimizer`、`fwis_v3_playback`、`fwis_v3_demo`。`fwis_common`、`fwis_map_server` 的审查文件内容一致。
- C++ 通用运动学、Reeds–Shepp、车辆几何、速度启发式和安全走廊在公共头文件内。唯一的 ROS 消息依赖为 `grid_map.hpp` 的 OccupancyGrid 构造与重置。搜索代码与节点入口共享 C++ 翻译单元，保留数值代码并修改消息类型和运行接口。
- Python 数值核心通常独立于 ROS，在 `core.py`、`mode_master.py`、`continuous_geometry.py`、`continuous_optimizer.py`、`retiming.py`、`vehicle_execution.py`、`quality.py` 等文件内。分支间同名模块存在实质差异。
- 多车分支的 `uniform_*` 和 `formation_continuous_a/` 提供几何和轨迹流程，ROS1 默认启动 `virtual_rigid_a`。相同源分支还含 `uniform_v02` 和 `legacy` 路径。多车分析以指定分支的实际默认路径为准。
- 旧地图工具负责 PGM/YAML 加载、场景缩放、提示和泊车线。新入口可以直接接收 `nav_msgs/msg/OccupancyGrid`。归档实验包和历史场景不属于 ROS2 工作空间。
- 构建依赖包括 roscpp、rospy、各标准消息、tf、Catkin、消息生成；运行涉及 NumPy、SciPy、PyYAML，以及部分求解路径使用的 CasADi。源代码还有数据采集工具和历史诊断或依赖补丁。
- 现有回归包括每个单车分支的 `test_policy_history.cpp`、Python 数值测试、ROS1 `ros_*_smoke.py`；多车提供 `test_formation_geometry.cpp`、`ros_virtual_a_smoke.py`、编队及播放 smoke、公开的空场景与拒绝样例。
- Ranger Mini 资源取自 AgileX `ugv_gazebo_sim/ranger_mini/ranger_mini_v2`。Classic 启动与控制位于独立 `ranger_mini_v2_gazebo` 目录及模型 xacro 中，迁移使用原厂几何和网格。 `ranger_ros2:humble` 作为接口参考。 [AgileX Ranger Mini V2](https://github.com/agilexrobotics/ugv_gazebo_sim/tree/master/ranger_mini/ranger_mini_v2)

源代码审查期间原分支未写入。主题与消息见 [ROS1 接口清单](ROS1_API_INVENTORY.md) 和 [消息差异](MESSAGE_SCHEMA_DIFF.md)。
