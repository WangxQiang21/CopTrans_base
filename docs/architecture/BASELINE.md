# CopTrans Phase 0 规划与控制基线

基线日期为 2026-11-24（Asia/Shanghai）。开始时 `ros2-humble` 和本地远端跟踪引用 `origin/ros2-humble` 均指向 `2c181ca7255b0287a278849c349096981da32cf6`，工作区干净。基线使用原有 `ros2_ws/install`，与实施阶段的独立 overlay 分开。已逐字节核对安装的三车规划节点、runner、跟踪器、规划 launch 和相关配置与基线提交一致。

| 项目 | 记录 |
| --- | --- |
| 主机 | AMD Ryzen 7 7840H，8 核 16 线程，约 30.5 GiB RAM，CPU governor `powersave` |
| 系统 | Ubuntu 22.04.5 LTS，ROS 2 Humble，Gazebo Harmonic 8.15.0 |
| 编译器 | GCC 11，原有 overlay 的 `CMAKE_BUILD_TYPE` 未显式设置 |
| 场景 | `factory_indoor_01`，统一 `test.sh` 入口启用 Gazebo GUI 和 RViz |
| 单车 | `continuous_mode_path`，控制器 `nonlinear_feedback`，另测 PID |
| 三车 | `multi_4wis / uniform_v02`，楔形编队、独立轨迹与共同许可 |

[环境和一致性记录](../../reports/architecture_baseline/environment.json) · [控制参数](../../reports/architecture_baseline/config/ranger_tracking.yaml) · [默认参数](../../reports/architecture_baseline/config/test_defaults.yaml) · [实验目标](../../reports/architecture_baseline/config/experiments.yaml)

## 闭环样本

| 任务 | 控制 | 请求至验证计划（墙钟 s） | 参考时长（仿真 s） | 终点位姿误差（m / rad） | 最小环境净距（m） | 判定 |
| --- | --- | ---: | ---: | --- | ---: | --- |
| 单车短弯 | nonlinear | 3.175 | 12.529 | 0.005009 / 0.004231 | 1.777843 | PASS |
| 单车直线 | nonlinear | 2.312 | 26.786 | 0.004016 / 0.000000 | 1.350000 | PASS |
| 单车全线路 | nonlinear | 9.913 | 213.647 | 0.011121 / 0.010687 | 0.181045 | PASS |
| 单车短弯 | PID | 3.092 | 12.529 | 0.005200 / 0.002358 | 1.777431 | PASS |
| 三车短弯 | nonlinear | 3.384 | 30.683 | 0.000428 / 0.000174 | 0.573627 | PASS |
| 三车全线路 | nonlinear | 7.566 | 400.813 | 0.000240 / 0.000055 | 0.125119 | PASS |

三车终点误差按编队参考 F 报告，环境净距取车体和货物最小值。三车短弯各车均经历 S→A，同步启动偏移为 0 仿真秒，最大实测货物支点误差 0.001397 m。表中已完成样本的 TRACKING 命令均通过逐条 A/O/S 模式约束复核。控制器 P50/P95/P99 在对应样本数充足时按每车 TRACKING 日志统计。原始日志、配置、CDR、地图和质量报告通过运行 ID 和 SHA-256 索引定位。 [实测索引](../../reports/architecture_baseline/live_runs.json)

## 时间边界

- `result.yaml` 的 `planner_wall_s` 是 runner 发请求到接收并验证计划的墙钟区间，三车还完成路线构造。Gazebo 启动、轨迹执行及退出观测窗口分别测量。
- `planner.log` 与 `trajectories/planner_reports` 记录规划进程的搜索、优化和认证耗时。基线未独立发出 `first_certified` 事件。
- `dds_trace.jsonl` 记录观察进程收到请求、轨迹、分车路线、准入、permit、首个非零 Twist 和控制状态的单调时钟。主题间的回调交付顺序可与发布顺序不同。追踪 ID 使用 `run_id`，消息有 `plan_id` 时一并记录。
- `capture.json` 每 200 ms 抽样子进程的累计 CPU 和 RSS。采样值用于同等负载下对比。
- 初次受限环境启动出现 Gazebo 日志目录访问和 DDS `getifaddrs` 错误。随后在允许本地 DDS 与显示的环境中复跑通过，该失败属于仿真启动。

三车长线只读监听在请求送达后 6.080 s 收到轨迹，7.976 s 收齐三车准入，8.924 s 收到三车首个非零 Twist。最终支点最大偏差 0.004191 m。 [DDS 事件偏移](../../reports/architecture_baseline/multi_full_route_event_offsets.json)

## 复现

以下命令复现归档用基线提交。在新版 CLI 上执行同一实验时追加 `--speed-profile baseline`，沿用 0.16 m/s 的旧速度合同。统一入口为各试验设置本机 DDS 域和 Gazebo 分区，仿真锁保护该入口的运行。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/test.sh --task single --scenario factory_indoor_01 --experiment short_turn --no-build
./scripts/test.sh --task single --scenario factory_indoor_01 --experiment short_straight --no-build
./scripts/test.sh --task single --scenario factory_indoor_01 --experiment full_route --no-build
./scripts/test.sh --task single --scenario factory_indoor_01 --experiment short_turn --controller-backend pid --no-build
./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment short_turn --no-build
./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment full_route --no-build
```

用 `capture.py` 包裹试验时记录 DDS 交付、事件单调时钟和资源采样。`summarize.py` 可索引同一版本的多个运行目录。三车短弯纯内核回放用固定的存档、配置和模型，默认 5 次预热和 30 次正式样本。优化 overlay 可用同一命令及同一存档，支持时追加 `--base-only --reuse`。闭环测量另外涵盖消息、控制跟踪和仿真时钟。

```bash
python3 reports/architecture_baseline/capture.py --task multi --experiment short_turn
python3 reports/architecture_baseline/summarize.py reports/runs/<run_id> -o /tmp/coptrans-run.json
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python3 reports/architecture_baseline/benchmark_multi.py reports/runs/20261124_213238_multi_factory_indoor_01_multi_4wis_Y37mPg --output /tmp/coptrans-kernel.json
```

短弯纯内核的 30 次样本全部可执行，模型摘要逐次匹配实测 ROS 轨迹。墙钟 P50/P95/max 为 2.777 / 2.971 / 3.075 s。采样期间主机同时运行基线闭环，记录用于同等负载的对照。 [逐次采样](../../reports/architecture_baseline/multi_short_turn_kernel.json)

| 验收项 | 状态 |
| --- | --- |
| 单车短弯、直线、全线路、PID 短弯 | PASS |
| 三车短弯 | PASS |
| 三车全线路 | PASS |
| CDR、日志、构型和运动质量索引 | PASS |
| 源码发布端和 Gazebo 消费端精确时戳 | NOT_RUN，已有只读 DDS 接收时间 |
| 单车端到端 30 次及任一规划指标 100 次有效采样 | NOT_RUN |
| 实车适配 | HARDWARE_NOT_VERIFIED |
