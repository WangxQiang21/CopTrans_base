# Phase 5 单车 State Lattice 与认证定时

`state_lattice` 是可选单车后端。只针对选中它的实验启用整套改动，`continuous_mode_path` 仍是默认后端。保留 `TimedTrajectory`、启动话题和跟踪器预检接口。 [测试入口](../../scripts/test.sh)

## 搜索、竞争与发布

- 格点包含车身航向、模式、行进方向、转向量和换向历史。A/O/S 原语库闭合在旋转后的四向格点上，终段允许解析连接。短直线、短弯、单段 O 或 S 可直接返回独立认证结果。未达到配置预算前可改进已认证 incumbent，新请求会协作终止旧格点搜索。
- 当端点直线距离达到 6 m，或首个候选含多个模式或超过三个连续原语时，启动最长 14 s 的发布前竞争。原有 C++ 前端走独立话题，经过起点应答、私有请求 ID、起点和终点、时间戳匹配才取得种子。`ModePathMaster` 在当前请求自己的地图快照和限制上重做轨迹优化、连续车体扫掠、动力学和质量认证。选择此阶段已认证且参考用时最短的候选。超时保留已取得的认证解。
- 每个请求最多发布一条非空控制轨迹。计时到期、取消或切换请求代际会舍弃落后的候选，地图或起点变化会使现有发布失效。背景改进无权替换实时跟踪中的计划。有限预算未找到候选时报告失败原因。

[搜索](../../ros2_ws/src/fwis_backends/fwis_backends/single_continuous_mode_path/state_lattice.py) · [竞争器](../../ros2_ws/src/fwis_backends/fwis_backends/single_continuous_mode_path/lattice_portfolio.py) · [ROS 适配器](../../ros2_ws/src/fwis_backends/single_continuous_mode_path/scripts/single_state_lattice_planner.py) · [启动和私有话题](../../ros2_ws/src/fwis_bringup/launch/plan.launch.py)

## 纯 A 定时

`parameterize` 以 `x = (ds/dt)^2` 按可控集合向后计算，再向前推进，支持区间速度、加减速以及多车约束的共同包络。`pure_ackermann_profile` 解析计算连续曲率块在 Hermite 区间上的曲率和空间导数极值。一般变化曲率的可达性解可用于速度上界研究。其加速度切换仍需 jerk 平滑和完整认证。

针对限制模式下的定曲率 A，已将解析 jerk 有界 S 曲线转成现有 `ContinuousTimedPlan` 的分段多项式。独立审计重新构造时间多项式、对其导数求极值并检查速度、加速度和 jerk，再校验车体扫掠、终点及下发样点。只有更快且独立认证通过才采用。模式切换和原有停车相位保持原合同。算法支持 ROS 普通调用链及原跟踪器预检，实车状态为 `HARDWARE_NOT_VERIFIED`。

相同限制下五个定曲率前进、倒行、直线和弯道样本全部通过认证，较原连续定时的运动时间缩短 **8.9%–18.9%**（中位 **16.8%**）。五个样本各运行 5 次预热及 30 次计时。故意篡改时间多项式均被独立审计拒绝。 [原始样本](../../reports/architecture_phase5/results.json) · [定时转换](../../ros2_ws/src/fwis_backends/fwis_backends/single_continuous_mode_path/continuous_retiming.py) · [独立审计](../../ros2_ws/src/fwis_backends/fwis_backends/single_continuous_mode_path/quality.py)

## 同配置 ROS 规划对照

使用相同地图、起点、终点和 0.16 m/s 基线速度，通过生产 `plan.launch.py` 启动各后端。十个固定场景覆盖直线、任意终端转弯、Spin、横移、仅倒行狭廊、直角弯、狭窄 S 廊、封闭隔断、地图外目标和室内完整线路。两端均 **8/10** 认证发布，封墙与地图外目标均未发可执行轨迹，每个请求最多一个非空输出。

| 场景 | 旧请求至轨迹 / 参考运动（s） | 新请求至轨迹 / 参考运动（s） |
| --- | ---: | ---: |
| 直线 | 1.449 / 8.929 | 0.078 / 7.252 |
| 任意终端短弯（30 次 P50） | 1.543 / 10.827 | 0.101 / 9.502 |
| Spin | 1.625 / 9.706 | 0.032 / 9.706 |
| 横移 | 1.494 / 12.420 | 0.080 / 12.420 |
| 仅倒行狭廊 | 0.306 / 15.306 | 0.117 / 13.502 |
| 直角弯 | 1.473 / 22.335 | 0.251 / 20.637 |
| 狭窄 S 廊 | 3.300 / 57.250 | 4.476 / 57.250 |
| 室内完整线路 | 6.258 / 152.409 | 6.830 / 152.409 |

八个共同可行场景的请求至 ROS 轨迹中位数 **1.518→0.109 s**，下降 92.8%，达到 20% 的中位目标。短弯预热 5 次、正式 30 次，P95 **1.911→0.116 s**。其他场景各一次对照，两个批次在同一主机的不同进程运行，复杂场景墙钟受负载影响。S 廊和完整线路增加了 1.176 s 和 0.572 s 的请求等待以保住当前连续后端的参考质量。完整线路首次纯 lattice 参考为 375.094 s，现以认证竞争恢复到当前连续后端的 152.409 s。Phase 0 旧闭环的参考是 213.647 s。

[汇总](../../reports/architecture_phase5/portfolio_summary.json) · [旧原始试验](../../reports/architecture_phase5/portfolio_matrix_legacy.json) · [新原始试验](../../reports/architecture_phase5/portfolio_matrix_best_of.json)

## 闭环、回退和复现

- 31 项单元测试通过，含随机进度包络、格点闭合、方向策略、A/O/S 等运动学、认证系数篡改、旧定时退路及协作取消。隔离 localhost DDS 域内验证了 A/O 轨迹通过现有跟踪器预检、错误坐标系拒绝和地图变化清空。
- 首版竞争器在 0.60 m/s 标准速度的 Gazebo/RViz 室内完整线路通过。最终安装版使用同一标准档和 C++ 跟踪器再次完成，规划请求至验证 **8.227 s**，参考 **91.548 仿真秒**，跟踪计算平均 **0.055 ms**，终点误差 **0.01981 m / 0.02230 rad**，最小现场净距 **0.19318 m**。五个困难场景完成真实 ROS 认证对照，其完整 Gazebo 闭环尚未分别运行。
- 可用 `--planner-backend continuous_mode_path` 回退至原后端。状态格点的 YAML 可将 `lattice.contest_legacy` 或 `lattice.analytic_ackermann` 分别设为 `false`，只保留格点或原定时。

[最终完整线路闭环记录](../../reports/architecture_phase5/gui_full_final_standard.json) · [首版完整线路闭环记录](../../reports/architecture_phase5/gui_full_standard.json) · [ROS 复现实验脚本](../../reports/architecture_phase5/portfolio_ros.py)

```bash
colcon build --base-paths ros2_ws/src --packages-up-to fwis_bringup
PYTHONPATH=ros2_ws/src/fwis_backends python3 -m pytest -q ros2_ws/src/fwis_backends/test/test_path_reachability.py ros2_ws/src/fwis_backends/test/test_state_lattice.py
./scripts/test.sh --task single --scenario factory_indoor_01 --experiment full_route --planner-backend state_lattice --speed-profile standard --no-build
python3 reports/architecture_phase5/portfolio_ros.py --backend single_state_lattice --repeat-case off_grid_turn --warmups 5 --repeat 30 --output reports/architecture_phase5/portfolio_matrix_best_of.json
```
