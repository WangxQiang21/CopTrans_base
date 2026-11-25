# 开发验收与复测

本机验证使用 ROS 2 Humble 和 Gazebo Harmonic。每轮 Gazebo 使用独立的 DDS 域和 Gazebo 分区。对比时固定地图、起终点、货物几何、速度限制和软件版本。涉及实车的时序和安全表现标记为 `HARDWARE_NOT_VERIFIED`。

## 构建和自动检查

```bash
bash scripts/check.sh
```

该入口在独立目录干净构建 Release 版本，确认共享规划核、三车搜索和编队碰撞三个原生 Python 扩展可加载，运行 Colcon 注册测试及规划器 pytest。每次保留构建与测试输出、JUnit、运行命令、Git SHA 和配置快照。GitHub Actions 在 `ros2-humble` 的推送和合并请求上调用同一入口。共享 CI 环境适合做功能和安全回归，真实 25 Hz 压力测试在固定主机单独复测。

## 保存可复算的证据

用 `scripts/record_acceptance.py` 包装验收命令。每轮使用一个新输出目录。可多次传入 `--input` 保存固定参数或原始输入的字节及 SHA-256。命令从仓库根目录运行，使用 `COPTRANS_EVIDENCE_DIR` 写入原始样本和结果。清单包含 Git 提交和树 SHA、工作区状态、原命令、依赖版本、CPU governor 和进程退出码。性能统计至少预热 5 次，正式采样至少 30 次。报告 P99 时保留至少 100 个正式样本。同次复测的环境、参数、墙钟原始采样和闭环指标应一并保留。

常规闭环由 `scripts/test.sh` 自动写入本地 `reports/runs`。验收基准、压力原始数据和 CI 复现输出放在本地 `reports/acceptance`。这两个目录由 Git 忽略。

## 2026-11-25 本机证据

| 证据 | 正式样本或闭环 | 本地目录 |
| --- | --- | --- |
| 矩形走廊与原搜索 | 6 个固定场景各 30 对 | `reports/acceptance/20261125_132054_algorithm_corridor30` |
| 同几何下的共同时间轴 | 5 条路径各 100 次 | `reports/acceptance/20261125_132106_algorithm_clock100` |
| C² 区间舍入优化 | 30 对内核、5 对完整请求，后者只报告中位数和最大值 | `reports/acceptance/planner-rounding-kernel-20261125`、`reports/acceptance/planner-c2-replay-20261125` |
| ROS CDR | 2 条归档轨迹各 100 次序列化和反序列化 | `reports/acceptance/20261125_132202_cdr100` |
| C++ 安全故障注入 | 三车 nonlinear、单车 PID | `reports/acceptance/20261125_132439_controller_faults` |
| Gazebo/RViz 复杂窄通道 | 3.00 m 直穿、2.60 m 绕行 | `reports/runs/20261125_132051_mGuofk`、`reports/runs/20261125_132257_XKwYLG` |

约 566 kB 的较长轨迹在同机 CDR 对照中，序列化 P99 为 20.91 ms，反序列化 P99 为 105.36 ms。该观测包含 Python 消息对象构造，计时范围截止于编码或解码完成。原始样本同时记录包尺寸、语义往返验证和模型 SHA。

## 生产门槛与回退

生产切换要求固定输入下的 Python/C++ 行为等价、A/O/S 安全故障注入、编队统一准入与停车、窄通道闭环、构建和时序门槛全部通过。25 Hz 控制的纯计算 P99 上限为 0.5 ms，最大值上限为 1 ms，完整回调最大值上限为 5 ms。单车和三车各持续至少 600 墙钟秒，每车至少 15,000 个跟踪周期。ROS 定时器截止违约和不安全命令要求为零。沿用原墙钟验收，回调间隔达到 80 ms 计为漏掉一个完整控制周期，命令从发布到 Gazebo 首次消费的上限为 40 ms。回调超过 40 ms 的次数和每段 DDS、桥接、仿真延迟同时记录。

常规入口上的完整回退使用同一条命令。交互测试支持单独回退到 Python 跟踪器。

```bash
./scripts/test.sh --task multi --architecture-profile baseline --tracker-implementation python --no-build
./scripts/interactive.sh --tracker-implementation python --no-build
```

场景验收、规划剖析和控制时序见 [三车规划与共同时间轴](PHASE6_FORMATION.md) · [规划热点](PLANNER_HOTSPOTS.md) · [C++ 跟踪器](CONTROLLER_CPP.md)。
