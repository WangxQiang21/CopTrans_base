# CopTrans C++ 轨迹跟踪器

原有 `fwis_control` 包新增 `ranger_tracking_controller_cpp`。单车与三车统一测试的跟踪器实现由 `--tracker-implementation python|cpp` 选择，默认 `python`。`--controller-backend nonlinear_feedback|pid` 独立选择控制律。C++ 和 Python 都读取运行指定的跟踪器配置，基准参数来自同一份 [ranger_tracking.yaml](../../ros2_ws/src/fwis_control/config/ranger_tracking.yaml)。

## 代码和接口

- [C++ 类型与接口](../../ros2_ws/src/fwis_control/include/fwis_control/tracking_core.hpp) 定义地图系 `Pose`、车体系 `Twist` 与误差、A/O/S 枚举、参数、轨迹预检与执行接口。长度使用米，角度使用弧度，时间默认使用秒。
- [控制律与限幅](../../ros2_ws/src/fwis_control/src/tracking_model.cpp) 保持 Kanayama、O 模式前馈投影与 S 模式航向跟踪。PID 保留增益、微分低通、积分限幅和抗饱和。A 模式半径约束仍由 `max(mechanical_radius, switch_radius)+radius_margin` 计算，基准为 0.83 m。保留轴向 O 的横向编码、方向反转过零与线/角加速度限制。
- [预检和采样](../../ros2_ws/src/fwis_control/src/tracking_trajectory.cpp) 对完整轨迹校验采样间隔、初末速度、模式几何、停稳窗和动力学。当反馈初始模式与计划初始模式不同，预检也准备首个停稳切换事件。采样根据相对时间二分定位，并使用相邻同模式的速度中点做前馈插值。
- [执行状态机](../../ros2_ws/src/fwis_control/src/tracking_execution.cpp) 保持原有状态字符串，经过停稳、等待确认、稳定和再启动。反馈或模式失效、统一许可无效、轨迹替换和时钟异常按定义停止。终点停车需等待反馈验证。

ROS 适配器保留原有 `trajectory_topic`、`permit_topic`、`future_start_window_s`、`config_file` 与 `algorithm`。输入为 `TimedTrajectory`、车辆命名空间下的 `odom` 和 `motion_state`，协同时另接收 `FormationExecutionPermit`。输出是 `cmd_vel`、`sim_request_mode` 与 JSON `controller_status`。消息标识、里程计坐标系、控制命令及仿真模式编码保持兼容。 [ROS 节点](../../ros2_ws/src/fwis_control/src/tracking_controller_node.cpp)

## 调度和时间统计

控制线程独立运行 ROS 2 executor，处理反馈、许可、ROS 时钟 25 Hz timer 与 steady-clock watchdog。后台 executor 解析并预检到达的计划，新计划通过原子指针换入。大容器的回收和状态 JSON 序列化在后台完成。控制线程输出状态记录到 128 槽单生产者单消费者队列，后台每 10 ms 读取。队列饱和时常规状态记录可丢弃，候选接纳会等待至少两个槽位而保持已准入轨迹依照现有许可执行。旧计划通过 32 槽回收队列交给后台释放，避免控制 timer 释放大计划容器。

`compute_s` 在控制回调开始到发布 Twist、模式请求并组装定长状态时计时，包含核心计算和 ROS 发布开销，状态序列化发生在后台 executor。`callback_interval_s` 给出同一进程相邻回调进入的 steady-clock 间隔。现有守护将连续超过反馈超时窗口的 timer 停摆作为停车条件。严格统计大于 40 ms 的间隔和至少 80 ms 的间隔分别保留，便于区分周期边界抖动与跨越完整周期。原始合成试验只收集了这些外层指标。实际 Gazebo 阶段计时见后文。

## 构建和回退

在 ROS 2 Humble 工作区构建并测试 `fwis_control`，构建类型采用 `RelWithDebInfo`。C++ 节点依赖 `rclcpp`、`ament_index_cpp`、`yaml_cpp_vendor` 和旧消息包，纯 C++ 核心只依赖标准库。CMake 注册原有 Python 测试和 C++ 与 Python 逐周期差分测试。纯 C++ 也可用 ASan/UBSan 编译测试重放驱动。

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-up-to fwis_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
colcon test --packages-select fwis_control
colcon test-result --verbose
```

统一试验可选 `./scripts/test.sh --task multi --tracker-implementation cpp --no-build`。单车将任务设为 `single`。复现本文旧速度合同的闭环试验时，统一入口加 `--speed-profile baseline`。使用 `--tracker-implementation python` 切回原跟踪器。独立 Gazebo 跟踪演示支持 `ros2 launch fwis_control tracking.launch.py tracker_implementation:=cpp gui:=false rviz:=false`。

## 当前验证证据

- Python 和 C++ 对同样的 A/O/S、方向反转、初始 O 对齐、无效曲率、反馈超时及缺少模式确认样本进行逐周期比较。用同一输入对比了参考位姿、原始/实际 Twist、phase、requested mode、诊断原因与预检结果。数值比较阈值为绝对值 `1e-9`。ASan/UBSan 通过该差分测试，sandbox 环境内取消了 LSan 退出扫描。 [差分测试](../../ros2_ws/src/fwis_control/test/test_tracking_cpp_parity.py)
- 三 C++ 节点以 ROS 2 连接合成车体模型，nonlinear 与 PID 均通过 A/O/S 闭环、同行车辆反馈失效后组停和活动计划替换停车。故障烟测各车约 278 个跟踪回调，nonlinear P99 为 0.073–0.092 ms，PID P99 为 0.071–0.085 ms。 [故障注入入口](../../ros2_ws/src/fwis_control/test/smoke_tracking_cpp.py) · [nonlinear 证据](../../reports/control_cpp/nonlinear_fault_smoke.json) · [PID 证据](../../reports/control_cpp/pid_fault_smoke.json)
- Harmonic 独立单车 C++ 跟踪完成 A→O→S→A，拒绝了过紧 Ackermann 参考，合格轨迹终点位置误差 9.32 mm。 [Gazebo 烟测摘要](../../reports/control_cpp/gazebo_headless_smoke.json) · [Gazebo 原始跟踪状态](../../reports/control_cpp/gazebo_headless_status.jsonl)

同一组三车合成轨迹及故障脚本各运行一次。Python nonlinear 回调 P99 为 0.331–0.367 ms，C++ 为 0.073–0.092 ms。Python PID 回调 P99 为 0.289–0.310 ms，C++ 为 0.071–0.085 ms。统计均基于跟踪状态中的单周期样本。本组样本说明本机量级，测试轮次不足以评估多次独立启动间的尾部时延。 [同输入短程计算时间对照](../../reports/control_cpp/python_native_smoke_comparison.json)

### 610 秒 ROS 持续压力试验

在隔离本机 DDS 域用合成 Ackermann 车体持续运行，保留带 ROS 时钟、单调墙钟间隔和最终 Twist 的原始样本。压力运行的可执行文件与交付代码的控制数学、watchdog、发布与队列机制一致，用于压力采样的版本构建后，加固了进程退出的 executor 循环。新循环经上面的 PID 合成闭环与 Gazebo 混合模式复测验证。

| 测试 | TRACKING 样本 | P99 / 最大回调（ms） | 最大回调间隔（ms） | 间隔 ≥80 ms | 不合法命令 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 单车，611.65 s | 15,265 | 0.080 / 1.116 | 42.797 | 0 | 0 |
| 三车 robot1，611.75 s | 15,267 | 0.081 / 0.623 | 41.518 | 0 | 0 |
| 三车 robot2，611.75 s | 15,267 | 0.080 / 1.364 | 42.793 | 0 | 0 |
| 三车 robot3，611.75 s | 15,267 | 0.083 / 1.218 | 42.363 | 0 | 0 |

合成试验的整体回调耗时均小于 5 ms，P99 低于 0.5 ms，未观测到跨越 80 ms 的周期。该合成记录的峰值包含 ROS 发布。下文在实际 Gazebo 车体插件中独立测量了纯核心、传输、消费和调度。 [单车摘要](../../reports/control_cpp/single_610s_summary.json) · [单车原始数据](../../reports/control_cpp/single_610s_timing.csv.gz) · [三车摘要](../../reports/control_cpp/three_610s_summary.json) · [三车原始数据](../../reports/control_cpp/three_610s_timing.csv.gz)

实车控制状态为 `HARDWARE_NOT_VERIFIED`。当前生产默认继续使用 Python 跟踪器。

## 实际 Gazebo 插件的阶段计时

设置单次运行目录环境变量 `COPTRANS_TIMING_DIR` 可开启 C++ 跟踪器和 Gazebo 车体插件的可选计时。`timing_telemetry` 可显式覆盖 C++ 节点是否采集，默认在未指定目录时关闭。普通控制仍使用原有 Twist、里程计和模式话题，计时字段仅附在 C++ 的状态记录。控制线程及仿真物理更新均只形成有限大小的记录，现有控制节点的后台线程和独立车体记录线程负责序列化和磁盘写入。

| 指标 | 时间域与端点 |
| --- | --- |
| 纯控制计算 | 主机单调时钟，在已准备好的轨迹和反馈快照上进入与退出 `Execution::tick`。包含采样、状态机、控制律和安全过滤。 |
| 控制回调 | 主机单调时钟，从 timer 的用户回调入口到形成将要入队的状态记录。包含反馈快照、许可、Twist/切换请求发布和固定大小状态组装。 |
| 定时器 | Humble 的 RCL timer 时钟，使用 `rcl_timer_call` 前的 `time_until_trigger` 得到到期相对时间。对同一测量点记录从到期到派发的延迟、跳过的周期，以及单次回调退出时越过该次截止的次数。仿真模式采用 ROS 仿真时间，读取误差受仿真时钟推进粒度限制。 |
| 执行器派发和抖动 | 主机单调时钟，从 RCL timer 派发到进入控制函数。墙钟相邻回调间隔与 40 ms 的偏差另列。 |
| 里程计的 DDS 和回调 | 同机系统实时时钟，使用 RMW 的源和接收时间戳，再与 C++ 订阅回调入口比较。RMW 的接收时间戳具体落点由实现决定。另用仿真消息时间戳匹配 Gazebo 里程计发布时间，可观察从车体发布到跟踪器回调的完整主机单调时间。 |
| 命令从 ROS 到 Gazebo 实际使用 | 记录 ROS 侧发布前、Gazebo 传输回调收到时、首次将该命令送进车体 `pre_update` 时三个主机单调时间。以车号、精确的三个 Twist 分量和 500 ms 因果窗口做双向唯一匹配，重复及缺失记录单独计数。 |

实际 Gazebo 测量使用厂房内一条半径 1.12 m、峰值约 0.14 m/s 的开阔区域 A 模式重复圆弧。单车和三车在独立 DDS 域和 Gazebo 分区分别运行。三车各自经过共同起点提前登记和统一许可。这个持续压力用例衡量实际仿真车体命令消费，cargo 负载运动由前述完整协同运输集成用例覆盖。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 ros2_ws/src/fwis_control/test/gazebo_timing_probe.py \
    --robots 3 --dds-domain 131 --output /tmp/coptrans-timing-three
python3 ros2_ws/src/fwis_control/test/analyze_control_timing.py \
    /tmp/coptrans-timing-three --output /tmp/coptrans-timing-three/summary.json
```

选用独立输出目录和可用的本机 DDS 域。改用 `--robots 1` 可运行单车。完整压力默认至少运行 605 墙钟秒并收集每车 15,000 个跟踪样本。短时连通性验证可用 `--seconds 8 --min-samples 175`。 [本机探针](../../ros2_ws/src/fwis_control/test/gazebo_timing_probe.py) · [离线匹配与截止统计](../../ros2_ws/src/fwis_control/test/analyze_control_timing.py)

### 实际 Gazebo 的 605 秒压力结果

单车和三车各记录至少 605 墙钟秒，每台车超过 15,000 个 TRACKING 周期。处于正常跟踪的命令全部能按原始数值匹配到 Gazebo 插件的首次消费。仿真时钟的 40 ms 截止契约下，跳过的 Ticks、回调越过截止次数和未解释的周期槽均为 0。有限值、速度、模式、半径和相邻输出加速度的违规数均为 0。

| 测试 | 跟踪周期 | 纯核心 P99 / 最大值（ms） | 整体回调最大值（ms） | ROS 发布到 Gazebo 消费 P99 / 最大值（ms） | 墙钟间隔最大值（ms） / ≥80 ms 次数 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 单车 | 15,106 | 0.0119 / 0.483 | 1.496 | 5.805 / 9.845 | 139.386 / 6 |
| 三车 robot1 | 15,100 | 0.0095 / 0.539 | 1.948 | 5.876 / 106.308 | 141.331 / 7 |
| 三车 robot2 | 15,100 | 0.0119 / 0.558 | 2.210 | 5.872 / 102.107 | 140.526 / 7 |
| 三车 robot3 | 15,100 | 0.0095 / 0.423 | 3.859 | 5.872 / 101.994 | 141.580 / 7 |

DDS 里程计源到接收 P99 为 0.365–0.461 ms。从 RMW 接收标记到跟踪器回调 P99 为 0.246–0.298 ms。Gazebo 发布里程计到跟踪器回调的端到端 P99 为 1.096–1.209 ms。定时器派发到回调入口 P99 约 0.005 ms。命令从 Gazebo 收到到首次消费 P99 为 1.888–2.069 ms。

三车最大的墙钟控制回调间隙出现在同一仿真时间，此间 ROS 仿真时间仍只增加 40 ms。三车分别有 2、1、1 条命令超过 40 ms 才被插件消费。robot1 的最大延迟主要在 Gazebo 收到命令后等待 `pre_update`，robot2 和 robot3 的最大延迟主要发生在从 ROS 发布到 Gazebo 接收的路径上。该记录的 ROS 仿真时钟门槛为 `PASS`，严格墙钟 40 ms 的门槛为 `NOT_MET`。这是本机软实时测量，真机响应为 `HARDWARE_NOT_VERIFIED`，控制器默认仍为 Python。

原始归档包含控制周期、三轴 Twist 和 Gazebo 端事件，可解包到独立目录后用上面的分析脚本复算。 [单车指标](../../reports/control_cpp/actual_gazebo_single_605s_summary.json) · [单车原始计时](../../reports/control_cpp/actual_gazebo_single_605s_raw.tar.gz) · [三车指标](../../reports/control_cpp/actual_gazebo_three_605s_summary.json) · [三车原始计时](../../reports/control_cpp/actual_gazebo_three_605s_raw.tar.gz)
