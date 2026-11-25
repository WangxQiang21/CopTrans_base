# 规划热点复测与 C++ 迁移范围

| 路径 | 已运行的 C++ | 仍在 Python 中执行 |
| --- | --- | --- |
| 单车 `continuous_mode_path` | ROS 规划前端和多模式粗搜索 | `ModePathMaster` 候选编排、连续几何、重定时、独立扫掠认证和发布适配。固定拓扑优化调用 IPOPT，数组计算调用 NumPy/SciPy。 |
| 三车 `uniform_v02`，启用 `native_frontier` | 状态键、代表表、优先队列、扩展预算、候选和代理代价，以及受支持的解析直线、转动和 Spin 联合扫掠 | ROS 任务编排、全局连接器和通道筛选、连续曲率候选与分段几何、共同时间律、FR02 独立审核。部分审核调用联合 SAT。 |
| 联合碰撞 | `_formation_collision_native` 通过 `fwis_planning_core` 调用车体、货物及互碰 SAT | 姿态及区间生成、连续曲线的保守扫掠组织、候选失败诊断和 Python 回退路径。 |
| 共享批量数学 API | `fwis_planning_core_native` 有几何推进、局部轨迹采样和批量碰撞 API | 此 Python 绑定目前在差分和基准脚本中使用。生产三车使用独立的 `_formation_collision_native` 接口。 |

复测固定车体及货物尺寸、起终点、地图、运动约束和搜索预算。正式墙钟计时在采样器关闭时采集，另用 5 ms CPU 时钟采样定位调用栈。经过 Python 包装的原生调用会计入包装函数的采样。以下窗口是离线规划，ROS 通信和消息消费时延另行验收。

## 定位结果

- 实验 `20261125_120655_6xtU9x` 保存的室内限宽地图和三车起终点，使用 native SAT 和 frontier。修补前的独立定位样本总耗时 4.700 s，首条认证 0.356 s，无格点扩展，连续曲率细化 4.324 s。区间下界、上界的向外舍入占 335/891 个自身栈样本。车辆和轮端的区间包络函数 `_vehicle_bounds` 覆盖 376/891 个累计栈样本。
- 固定室内短弯同样无格点扩展。区间向外舍入占 84/228 个自身栈样本。这个场景和近出口场景的优先项都是连续轨迹认证内的区间运算。
- 单车档案 `20261124_214810_single_factory_indoor_01_continuous_mode_path_NgEUdG` 重播 C++ 前端保存的短弯种子。一次定位样本的 Python 连续规划耗时 1.825 s，重定时 0.496 s，独立审核 0.491 s，IPOPT 0.152 s。第一项自身栈是 NumPy 的多项式求值，65/362 个样本。后续迁移优先验证是否能合并密集的分段求值与扫掠；保留已编译的求解器、独立认证和稳定的输入输出契约。

## 已实施的加速

区间舍入改用 Python 标准库的原生 `math.nextafter`。有限数仍前进或后退一个 IEEE-754 double，含符号零以及溢出到无穷大的边界。已有无穷大与 NaN 按历史契约返回原值。区间公式、分段、审核门槛和认证失败路径保持原有行为。

从归档 C² 轨迹提取共同时间律，固定几何与时间参数，重新计算整段车辆和轮端动力学区间。当前实现与原实现做 5 次预热、30 次交错配对，绑核到 CPU 11。30 对结果逐项浮点位模式一致。

| 共同时间律区间内核 | P50 | P95 | 最大 |
| --- | ---: | ---: | ---: |
| 原 struct 位操作 | 342.63 ms | 354.48 ms | 356.12 ms |
| 原生 nextafter | 192.03 ms | 196.03 ms | 216.51 ms |

同一内核的中位耗时下降 44.0%。100,000 个固定种子生成的 64 位输入样本和单独的非正规数、符号零、极值、NaN 负载测试均保持历史输出。轨迹时钟、货物碰撞及凸通道相关 26 项回归通过。

同一归档的完整规划做了 5 次预热、5 次配对复测。两组模型 SHA-256 在 5 对样本中完全一致，计划运动时间均为 114.984548346 s。规划中位耗时从 3.579 s 降至 2.574 s，最大从 3.920 s 降至 2.681 s。连续曲率细化阶段的中位耗时从 3.260 s 降至 2.266 s。正式样本和源码状态位于本地 `reports/acceptance/planner-rounding-kernel-20261125/` 与 `reports/acceptance/planner-c2-replay-20261125/`。

更新后的整条路径仍有 159/499 个累计栈样本在 `_vehicle_bounds`，独立碰撞审核占 105/499 个累计栈样本。若更广的窄弯用例有同样结果，下一处原生候选是成批计算车体、轮位和区间的包络上界，保留 Python 参考实现做逐项差分，包括浮点外扩、碰撞边界和失败分类。完整 C² 认证继续执行独立审核。

## 复测命令

从仓库根目录运行。更新构建并加载 ROS 2 Humble 工作区后，将输入、样本和源代码变更记录保存在本地 `reports/acceptance/`。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

archive=reports/runs/20261125_120655_6xtU9x
reference=$(mktemp)
git show 9ca13215:ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/formation_continuous_a/bounds.py > "$reference"
python3 scripts/record_acceptance.py --output "reports/acceptance/planner-clock-$(date +%Y%m%d_%H%M%S)" \
  --input "$reference" --input "$archive/trajectories/planner.cdr.gz" \
  --input reports/architecture_optimization/profile_planning_hotspots.py -- \
  bash -c 'python3 reports/architecture_optimization/profile_planning_hotspots.py \
    --clock-archive "$1" --rounding-reference "$2" --warmups 5 --samples 30 \
    --output "$COPTRANS_EVIDENCE_DIR/samples.json"' _ "$archive" "$reference"
```

`--formation-archive` 运行保存的三车端点、地图和运动约束的完整规划；`--generated short_turn` 运行提交中的固定室内短弯；`--single-archive` 使用带前端候选和运动契约的单车归档。CLI 分开记录每次计时、搜索扩展次数、模型或约束摘要和 CPU 采样。至少 30 个样本时报告 P95，至少 100 个样本时报告 P99。
