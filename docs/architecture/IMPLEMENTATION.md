# CopTrans 规划控制优化实施和验收

以 ROS 2 Humble / Gazebo Harmonic 为本轮验证平台。原始系统和基准配置见 [架构基线](BASELINE.md)。新版本把仿真速度档和架构实验开关分开，运行架构 A/B 时显式传入 `--speed-profile baseline`。

## 仿真速度档

AgileX 手册的性能表和 CAN 指令表给出最高车速 5.4 km/h（1.50 m/s）、Ackermann 最小转弯半径 0.810 m，以及转向超过 20° 时的线速度指令上限 0.75 m/s。本轮给仿真跟踪器和车体保留统一的 0.72 m/s 上限。单车规划使用 0.60 m/s、0.42 m/s²，三车带货使用每车 0.50 m/s、0.36 m/s²。标准仿真档另给跟踪器 0.60 m/s² 的平动加速度上限，原仿真车体为 0.65 m/s²。最小半径保护、角速度、角加速度、模式确认和停车边界沿用既有参数。 [AgileX 官方用户手册，第 6 和 20 页](https://new.agilex.ai/raw/upload/20230816/RANGER%20mini%202.0%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8CV1.0-20230316%28%E4%B8%AD%E8%8B%B1%E5%8F%8C%E8%AF%AD%29_71720.pdf)

| 室内闭环用例 | 参考时长，旧档 → 标准档 | 峰值线速度指令，旧档 → 标准档 | 标准档结果 |
| --- | ---: | ---: | --- |
| 单车短弯 | 12.529 → 4.551 仿真秒 | 0.163 → 0.621 m/s | 通过，终端跟踪位置误差 26.1 mm |
| 三车带货短弯 | 30.683 → 14.466 仿真秒 | 0.160 → 0.507 m/s | 通过，货物采样净距 0.568 m，最大装载点偏差 0.89 mm |
| 三车带货全程 | 400.813 → 139.827 仿真秒 | 0.164 → 0.500 m/s | 通过，共同起步偏差 0 s，货物采样净距 0.126 m，最大装载点偏差 12.60 mm，编队终点误差 2.03 mm / 0.0626° |

表中的峰值是闭环 `TRACKING` 样本内的 `hypot(vx,vy)`，三车行取三台车最大值。时长是规划参考的仿真时间。实际测试的各车峰值、运行 ID 和终端记录见 [速度 A/B 数据](../../reports/architecture_optimization/speed_profiles_ab.json)。每次运行保存 `planner_parameters.yaml`、`controller_parameters.yaml` 和 `scene/body_twist_parameters.yaml`。 [速度档配置](../../ros2_ws/src/fwis_bringup/config/test_defaults.yaml)

## 提速后的数据流

1. 三车流水线先由独立审计器认证基础共同时间模型。请求内复用以地图、车体和货物几何、初末状态、运动和碰撞限值、认证版本及确切边序列为条件。接收端仍重建模型、核对摘要和采样，再生成三条共时控制路线。
2. `optimized` 用独立预算启动可选 C² worker。货物筛选用考虑积分数值误差的内缩包络作正碰撞见证，筛选通过的候选仍需完成连续动力学和扫掠认证。当前无在途切换入口，后台候选输出诊断状态，执行计划保持同一个 ID 和模型摘要。
3. `optimized` 的实验协调器以发布端的 `execution_verified` 和接收端的模型/ID/摘要检查确认准入。控制器继续报告预检，协调器在截止线前收齐三车回执，再发一条共用 permit。真实模型、路径显示、日志订阅彼此独立。统一实验入口按一轮一目标管理执行。这个入口可用 `--without-planning-playback` 省掉解析回放节点，交互式连续发目标继续用原生命周期。
4. 单车 `optimized` 是否缩小反向竞争预算取决于是否已有开阔、纯正向 A 候选。当前守卫考虑车体外接半径和路径长度，高风险或首解失败仍走原倒车预算。优化器立即获取已经收到的辅助种子，避免在同一 ROS 互斥回调组里空等。Phase 5 的 State Lattice 是独立选项。
5. `--tracker-implementation cpp` 切换到原生 rclcpp 跟踪器。YAML、安全限幅、状态机、25 Hz 和运行话题维持同一合同，参数 `--controller-backend pid` 继续切换控制律。

[三车求解与认证](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_pipeline.py) · [三车发布状态](../../ros2_ws/src/fwis_backends/multi_4wis/scripts/formation_uniform_planner_node.py) · [执行协调器和统一追踪](../../ros2_ws/src/fwis_bringup/scripts/run_experiment) · [单车搜索](../../ros2_ws/src/fwis_backends/single_continuous_mode_path/planner_node.cpp) · [控制接口与回退](CONTROLLER_CPP.md)

## 同输入性能证据

| 固定输入对照 | 样本数 / 配置 | 旧实现 | 实验选项 | 质量结果 |
| --- | ---: | ---: | ---: | --- |
| 三车室内短弯，纯内核总耗时 P50 / P95 | 预热 5 + 正式 30 | 同步 C²，2.459 / 2.737 s | 基础计划，0.178 / 0.208 s。基础计划加证书复用，0.096 / 0.112 s | 各档模型摘要完全相同，均有完整独立认证 |
| 单车室内短弯，ROS 2 请求到首个可执行结果 P50 / P95 | 预热 5 + 正式 30 | 1.548 / 1.682 s | 0.425 / 0.474 s | 轨迹内容摘要一致，均为 A、12.529 s，车体最小采样净距 1.711 m |
| 三车固定 2 次图扩展用例，完整搜索 P50 | 预热 5 + 正式 30 | 275.56 ms | C++ 候选、前沿、代价、扫掠和预算，106.18 ms | 同 SAT 内核、同模型摘要和认证，2.60 倍 |
| 三车室内短弯，ROS 2 请求到首条认证信封 P50 / P95 | 预热 5 + 正式 30 | 0.234 / 0.248 s | 0.228 / 0.263 s | 30 对模型与投影摘要一致。P50 下降 2.4%，P95 增加 5.9%，在 10% 门槛内 |
| C++ 跟踪器软实时压力，回调 P99 / 最大 | 单车及三车分别约 612 秒，每车超过 15,000 次 | 短时旧节点回调另见控制报告 | P99 0.080–0.083 ms，最大 1.364 ms | 合成反馈压力、非线性和 PID 逐周期对照及故障注入通过 |
| C++ 跟踪器接真实 Gazebo 车体，纯计算 P99 / 最大 | 单车及三车分别约 605 秒，每车超过 15,000 次 | — | P99 0.0095–0.0119 ms，最大 0.558 ms | ROS 仿真时钟漏周期 0，非法命令 0，回调最大 3.859 ms |

三车 C++ 搜索和跟踪的无回放 Gazebo 实测中，从请求到信封接收为 0.307 s，到本地复验和构造控制路线为 0.426 s，到共用 permit 为 0.478 s。该闭环执行 30.683 仿真秒的基线轨迹，旧版同一模型摘要保持一致。初始停稳和转向准备之后各车才出现非零 Twist。上表仅使用 30 次规划正式样本的 P95。墙钟规划、ROS 接收、仿真运动和控制回调分别统计。

[三车流水线逐次数据](../../reports/architecture_optimization/multi_short_turn_pipeline_ab.json) · [单车 ROS 逐次数据](../../reports/architecture_optimization/single_ros_short_turn_ab.json) · [规划 C++ 内核](PLANNING_CORE.md) · [控制压力](CONTROLLER_CPP.md) · [闭环运行和原始时序](../../reports/architecture_optimization/closed_loop_integration.json)

三车同配置 ROS 认证信封测量确认短弯使用直连候选，格点扩展两组均为 0。独立的格点扩展夹具评估搜索加速。相同地图和输入在真实 Gazebo 的单次配对还测到从请求出线到接收信封为 0.313 → 0.337 s，构造控制路线为 0.436 → 0.487 s，后者包含控制路由和独立重建。两个闭环均完成，模型摘要相同。 [30 次 ROS 信封采样](../../reports/architecture_optimization/multi_ros_short_turn_frontier_ab.json) · [Gazebo 配对](../../reports/architecture_optimization/multi_native_gazebo_pair.json)

最后一次构建加固了请求缓存的只读地图和输入快照。原速度短弯的 30 次复跑全部命中请求内证书并返回原模型，纯内核 P95 为 0.087 s。标准速度档同时通过最终无回放闭环，从请求到接收复验及路线构造用时 0.281 s，参考轨迹为 14.466 仿真秒。 [最终快照复测](../../reports/architecture_optimization/final_certificate_snapshot.json) · [最终闭环时序](../../reports/architecture_optimization/closed_loop_integration.json)

单车额外做了六组 ROS 2 单次配对烟测。倒车、Spin、横移、直角转角、S→A→O 通道均输出同摘要的认证轨迹，两种配置均拒绝封闭墙并保持可执行输出为空。尾时延采用固定短弯的重复测量。 [烟测数据](../../reports/architecture_optimization/single_ros_difficult_smoke.json) · [烟测命令入口](../../reports/architecture_optimization/smoke_single_difficult.py)

实际 Gazebo 车体消费的命令传输 P99 为 5.805–5.876 ms。真实进程墙钟出现最大 106.308 ms 的发布到消费延迟，三车最大回调间隔 141.580 ms。25 Hz 仿真时钟验收通过，严格 40 ms 墙钟指标未达标。 [分段测量、原始样本和复现](CONTROLLER_CPP.md)

### 规划和消息分段

三车标准档离线回放预热 5 次，每个场景正式运行 30 次。表中是墙钟 P50。搜索含终点审计，独立审计含连续碰撞，时间区间有嵌套。

| 测量区间 | 室内短弯 | 室内全程 |
| --- | ---: | ---: |
| 搜索和终点认证 | 81.43 ms | 286.88 ms |
| 独立轨迹审计 | 65.51 ms | 221.29 ms |
| 连续扫掠碰撞查询总时长 | 5.62 ms | 63.61 ms |
| 基础时间参数化 | 10.70 ms | 29.93 ms |
| 新共同 jerk 时间轴 | 5.22 ms | 12.27 ms |
| 内核总时长 | 91.12 ms | 305.90 ms |

同步 C² 精修的同输入试验见流水线对照。闭环执行的阶段追踪另外记录模型重建、路线构造、三车准入确认和共同许可。100 次 CDR 测量的全程信封编码和解码 P50 从 14.95 / 73.83 ms 降至 9.82 / 46.95 ms。两份信封分别有 2,803 和 1,752 个共同采样点，解码包括 Python 消息对象构造。 [离线分段原始样本](../../reports/architecture_optimization/final_formation_stage_timing.json) · [CDR 单独采样](../../reports/architecture_optimization/multi_cdr_timing.json) · [闭环准入时序](../../reports/architecture_optimization/formation_clock_closed_loop.json)

## 新算法和闭环验收

- 单车 State Lattice 纳入同一地图及起终点的连续路径候选，候选通过同一独立认证后才参加选择。10 个固定场景中，原后端与实验后端各通过 8 个可行场景并按合同拒绝两个不可行输入。八个共同可行场景的 ROS 请求到首条轨迹时延中位数从 1.518 s 降至 0.109 s。部分困难和长程场景增加了首解等待，靠认证候选竞争维持参考路径质量。最终安装版的室内标准档全程闭环参考为 91.548 仿真秒，终点位置误差 19.81 mm。五条固定纯 A 路径的 jerk 受限时间律同限值参考运动时长中位缩短 16.8%。 [单车研究验收](PHASE5_SINGLE.md)
- 三车凸走廊在保守带货栅格中提取矩形凸区，在门户链中用区域并集约束折线缩短，再经联合 A/O/S 边与货物和车体扫掠审计。六张生成场景均通过。交错窄门试验中新链路 30/30 次通过，原搜索 0/30 次通过。临界通道两组均 30/30 次通过，首条认证 P50 从 0.682 s 降至 0.578 s，轨迹长度从 45.59 m 降至 29.36 m。走廊失败时用剩余预算回退，封闭墙试验两组均拒绝发布。 [三车走廊和共同时间轴](PHASE6_FORMATION.md)
- 三车共同 jerk 时间轴通过连续动力学界和原有几何认证。五条固定纯 A 路径的参考时长中位缩短 31.4%。室内标准档全程闭环启用凸走廊和共同时间轴，参考时长从 139.827 s 降至 87.257 s，货物采样净距 0.125 m，最大装载点偏差 12.70 mm，共同起步偏差 0 s，编队终点误差 16.29 mm / 0.515°。 [闭环证据](../../reports/architecture_optimization/formation_clock_closed_loop.json)
- 可选局部重规划原型从已认证全局计划选取共同停稳交接点，后台生成并认证局部窗口。有效地图下的超时保留原计划，地图失效请求整组停车。新计划要求三车匹配同一模型与切换时刻。真实规划器参与的 120 次 10 Hz 离线轮询已验证共同切换，当前生产环继续使用现有共同准入和组停规则。 [局部重规划原型验收](../../reports/architecture_optimization/PHASE7.md)

## 复跑与回退

```bash
cd /home/wq/Whut/CopTrans_base
./scripts/test.sh --task single --scenario factory_indoor_01
./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment full_route

./scripts/test.sh --task multi --speed-profile baseline --architecture-profile optimized \
  --without-planning-playback --multi-native-frontier --tracker-implementation cpp --no-build
./scripts/test.sh --task multi --speed-profile baseline --architecture-profile optimized \
  --multi-native-frontier --multi-convex-corridor --multi-reachable-clock \
  --tracker-implementation cpp --experiment full_route --no-build
./scripts/test.sh --task single --planner-backend state_lattice --speed-profile baseline --no-build
```

可用 `--no-build` 复用与当前源码一致的安装。仿真默认使用 `optimized`、C++ nonlinear 跟踪器和标准速度，单车使用 State Lattice，三车使用 `uniform_v02`。三车优化配置启用原生搜索前沿、凸走廊引导和共同可达时间轴。搜索使用 A/O/S 组合模式策略，并保留 C² 同步和回放状态机。架构回退用 `--architecture-profile baseline --tracker-implementation python`，旧仿真速度另选 `--speed-profile baseline`。[切换验收](ACCEPTANCE.md)

从 `ros2_ws` 构建的目标是 `fwis_bringup` 及其依赖，共 11 个 ROS 2 包。新包为 `fwis_planning_core` 和 `fwis_multi_planner_cpp`。跟踪器保留在 `fwis_control` 并使用既有消息。新增时序探针使用 Humble 的 `rcl` 和 `rclcpp`，原生规划绑定需 pybind11。提交变更清单可用 `git diff --name-status 2c181ca..HEAD` 查看。 [构建和测量主机](../../reports/architecture_optimization/qualification_environment.json) · [规划核构建和测试](PLANNING_CORE.md) · [控制构建和测试](CONTROLLER_CPP.md) · [单车研究复现](PHASE5_SINGLE.md)

```bash
source /opt/ros/humble/setup.bash
cd /home/wq/Whut/CopTrans_base/ros2_ws
colcon --log-base /tmp/coptrans-verification/log build \
  --build-base /tmp/coptrans-verification/build --install-base /tmp/coptrans-verification/install \
  --packages-up-to fwis_bringup --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

最终独立安装的 11 个包全部通过 Release 构建。综合 pytest 为 128 项，通过运动学、候选与碰撞等价、ROS 投影、模型篡改、状态机、控制逐周期和安全回退。共享核和三车搜索核心均通过 ASan/UBSan。最终闭环在项目默认安装目录复跑，测试进程和窗口在完成后退出。

## 阶段验收状态

| Phase | 构建 | 功能与安全 | 性能与算法 | 状态 | 证据 |
| --- | --- | --- | --- | --- | --- |
| 0 基线 | `PASS` | `PASS` | `PASS` | `PASS` | [基线与输入](BASELINE.md) |
| 1 首解优先 | `PASS` | `PASS` | `PASS` | `PASS` | [单车 ROS 样本](../../reports/architecture_optimization/single_ros_short_turn_ab.json) · [三车流水线样本](../../reports/architecture_optimization/multi_short_turn_pipeline_ab.json) |
| 2 共享 C++ 核 | `PASS` | `PASS` | `PASS` | `PASS` | [原生几何、碰撞、ASan/UBSan 与批量速度](PLANNING_CORE.md) |
| 3 三车 C++ 搜索 | `PASS` | `PASS` | `PASS` | `PASS` | [候选、代价、预算、场景等价及 2.60 倍搜索](PLANNING_CORE.md) |
| 4 C++ 跟踪 | `PASS` | `PASS` | `PASS` | `PASS`（本机仿真） | [单车和三车分别独占 Gazebo 复测，原有时序门槛均通过](CONTROLLER_CPP.md) |
| 5 单车算法 | `PASS` | `PASS` | `PASS` | `PASS` | [10 场景、闭环和固定路径时律](PHASE5_SINGLE.md) |
| 6 三车算法 | `PASS` | `PASS` | `PASS` | `PASS` | [九场景、净距和共同运动时间轴](PHASE6_FORMATION.md) |
| 7 局部在线重规划 | `PASS` | `PASS`（离线） | `PASS`（离线 10 Hz） | ROS 2 在线执行 `NOT_RUN` | [原型和时序](../../reports/architecture_optimization/PHASE7.md) |
| 实际底盘 | — | — | — | `HARDWARE_NOT_VERIFIED` | 本轮在 ROS 2、Gazebo Harmonic 和 RViz 2 验证 |
