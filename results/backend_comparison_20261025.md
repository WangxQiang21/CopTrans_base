# 三后端预设地图对照结果

运行日期：2026-10-25。共 11 张地图、33 次独立 ROS 规划。逐例原始日志和 `run.json` 保留在运行工作区的 `result/20261025_backend_comparison/`，未纳入 Git；本仓库保存汇总表。

## 汇总

| 后端 | 通过 | 成功率 | 成功案例中位轨迹时长 | 成功案例中位搜索+优化时间 | 中位模式切换数 | 中位车体净空 |
|---|---:|---:|---:|---:|---:|---:|
| legacy | 6/11 | 54.5% | 6.71 s | 3.75 s | 1.0 | — |
| mode_path_v1 | 5/11 | 45.5% | 43.17 s | 6.53 s | 3.0 | 0.035 m |
| mode_path_v2 | 9/11 | 81.8% | 14.24 s | 2.57 s | 2.0 | 0.051 m |

注：legacy 的轨迹时长使用 `legacy_native_speed_only` 旧时间合同；它没有 v1/v2 共用的净空评分。因此 legacy 时长和净空栏不与新后端直接排名。legacy 的动态检查不等价于 v1/v2 的加速度、jerk、转向速率和换模时间限制。v1/v2 的轨迹时长均按 `kinematic_sim_mode_switch_only_v2` 计算。搜索+优化时间是两段日志计时之和，不包含 ROS 启动/关闭时间。

## 按地图逐例结果

| 地图 | legacy | mode_path_v1 | mode_path_v2 |
|---|---|---|---|
| map01 | 通过，1.95s | 未通过：optimizer_no_certified_candidate | 通过，4.67s |
| map02 | 未通过：search_no_path_in_discretized_lattice | 通过，32.25s | 通过，11.29s |
| map03 | 通过，1.25s | 未通过：optimizer_no_certified_candidate | 通过，3.88s |
| map04 | 通过，20.69s | 通过，113.75s | 通过，40.45s |
| map05 | 通过，10.04s | 通过，43.17s | 通过，15.08s |
| map06 | 通过，5.92s | 未通过：optimizer_no_certified_candidate | 通过，14.24s |
| map07 | 未通过：search_budget_exhausted | 未通过：search_budget_exhausted | 未通过：search_budget_exhausted |
| map08 | 未通过：search_no_path_in_discretized_lattice | 未通过：search_budget_exhausted | 未通过：search_budget_exhausted |
| map09 | 未通过：search_no_path_in_discretized_lattice | 通过，85.84s | 通过，29.77s |
| map15 | 通过，7.49s | 未通过：optimizer_no_certified_candidate | 通过，9.00s |
| map16 | 未通过：search_budget_exhausted | 通过，28.11s | 通过，15.69s |

## 结果解读

- **mode_path_v2 当前最适合作为开发基线候选**：严格通过 9/11，高于 legacy 的 6/11 和 mode_path_v1 的 5/11。map07、map08 仍未在默认预算内找到候选，失败是搜索预算耗尽，不是不可行性证明。
- 在 v1 和 v2 都成功的 5 张地图（map02、map04、map05、map09、map16）上，v2 的轨迹时长都更短；v1/v2 时长比中位数为 **2.86x**。v2 成功案例的中位模式切换数也更低。
- mode_path_v1 有 5 张通过；4 张是搜索成功后优化器没有认证出可行候选，另 2 张搜索预算耗尽。它当前更适合作为消融/历史对照，不适合做主开发基线。
- legacy 有 6 张通过，但默认前端只允许正向；map02、map08、map09 搜索未到达，map07 和 map16 搜索预算耗尽。legacy 的已通过时长较短不能直接视为更优，因为它没有采用新后端同一套模式切换/动态时间合同。
- 所有 33 次运行的 ROS master、launch 子进程均已停止，端口均已释放；所有通过项均通过 5 mm 整车扫掠碰撞、终点、时间单调性、方向和进程清理检查。v1/v2 还按共同运动 profile 通过动态审核；legacy 的通过状态只按其 `legacy_native_speed_only` 合同解释，不能视为通过同一套加速度/jerk 审核。

## 测试条件与复现

- 输入：`tools/expv3_eval_cases.yaml` 中带 `project_preset` 标签的 11 个固定案例，按默认地图缩放。
- 三种后端都使用同一个 `params.yaml`，仅覆盖 `planning.backend`；全部关闭 `joint` 和 CBC，避免 legacy 单独启用 Bonmin。
- 各后端的默认方向策略不同：legacy 是 forward-only；mode_path_v1/v2 是双向候选。因此这是各版本当前默认能力对比，不是强制统一方向策略的消融实验。
- 运行命令示例：
```bash
python3 tools/run_quality_regression.py --cases P11_case_a_default P11_case_b_default P11_case_c_default P12_env1_default P12_env2_default P12_env3_default P12_env4_default P16_complex_parking_default P16_narrow_maze_default N06_default S09_underground_garage_default --cases-file tools/expv3_eval_cases.yaml --backend mode_path_v2 --port-base 13100 --output-dir result/20261025_backend_comparison/mode_path_v2
```

## 文件

- `comparison.json`：结构化总表、逐例检查、失败诊断和统计。
- `legacy/summary.json`、`mode_path_v1/summary.json`、`mode_path_v2/summary.json`：每组回归工具摘要。
- 每个地图子目录含 `run.json`、输入快照、ROS 日志、探针结果和关闭记录。
