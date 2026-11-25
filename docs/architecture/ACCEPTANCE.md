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

## 生产门槛与回退

生产切换要求固定输入下的 Python/C++ 行为等价、A/O/S 安全故障注入、编队统一准入与停车、窄通道闭环、构建和时序门槛全部通过。25 Hz 控制的纯计算 P99 上限为 0.5 ms，最大值上限为 1 ms，完整回调最大值上限为 5 ms。单车和三车各持续至少 600 墙钟秒，每车至少 15,000 个跟踪周期。截止违约与不安全命令要求为零。ROS 仿真时钟的周期结果与墙钟回调、DDS 和 Gazebo 实际消费的结果分别报告。严格的墙钟违约仍属于生产门槛。

常规入口上的完整回退使用同一条命令。交互测试支持单独回退到 Python 跟踪器。

```bash
./scripts/test.sh --task multi --architecture-profile baseline --tracker-implementation python --no-build
./scripts/interactive.sh --tracker-implementation python --no-build
```

场景验收和算法定义见 [三车规划与共同时间轴](PHASE6_FORMATION.md)。控制器的故障注入和主机时序测量见 [C++ 跟踪器](CONTROLLER_CPP.md)。
