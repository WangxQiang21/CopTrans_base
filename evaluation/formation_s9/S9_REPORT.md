# S9 阶段审查报告：独立 FormationEvaluator、选解与事务式发布

## 结论

**S9 完成：新增 ROS 无关的 `formation_quality.py`，对 `map + request + FormationContinuousPlan + FormationTimedTrajectory` 独立重算请求/轨迹合同、刚性几何、碰撞、A/O/S 运动学、共同时间动态、模式驻留/静止换模与“实际发布采样”碰撞；只有全部子项通过才输出 `FEASIBLE`。同时新增按公共 `T_end` 的候选选解和事务式发布门，失败候选全部返回结构化失败码，旧 `plan_id` 的发布在新请求、失败、重复/过期种子和地图变化时被清除。`formation_optimizer_node.py` 从 S3 的“只验证不规划”节点升级为完整 S6→S7→S8→S9→发布管线；ROS 消息在发布前再次校验，校验失败则回滚为清除。** 18 项独立审核测试 + 18 项节点/管线/事务测试全部通过；S5–S8 定向测试 52/52；完整 Python 回归 312 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误，没有新增失败。ROS 消息转换探针与私有 master（端口 11399）上的节点 smoke（`enable_nlp=false` 与默认 NLP 两轮）均通过。没有修改 C++、单车控制器、NMPC、执行器、闭环状态仿真、既有 `.msg` 或单车 `quality.py`。

## 任务编号

S9：独立 FormationEvaluator、选解和发布。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L824-L866)，阶段表见同文档 [S9 行](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L404)。

## Git 检查点

- S8 已完成提交：`c9b5f0873fc04d5fcf427aadfe6531dc9c36c756`（`feat/formation-time` 分支 HEAD）
- S9 分支：`feat/formation-audit`（从 S8 提交创建）
- S9 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| S8 输出语义 | 已核对 `FormationTimedTrajectory` 的 `phases`、`times_from_start`、每车 `FormationTimedPoint`、`dynamic_profile`；S8 只输出 `GEOMETRY_ONLY`，`executable`/`trajectory_published`/`collision_certificate` 均为 false |
| S6/S7 计划 API | 已核对 `FormationContinuousPlan`、`block.qf_at`、`body_headings_at`、`segment_map`、`vehicle_mode_segments`、`direction`、`goal_heading_free` 与 `audit_continuous_plan` |
| 单车独立审核基线 | 只复用 `quality.py` 的 `FootprintDistance` 与 `core.py` 的 `Grid`/`Action`；未修改 `TrajectoryEvaluator` 或单车 `quality.py` |
| 既有消息定义 | 未修改任何 `.msg`；`FormationTimedTrajectory` 仍无 header 字段，ROS `frame_id` 由 adapter 的 `header.frame_id="map"` 负责 |
| S3 请求合同 | 继续使用 `validate_formation_request`：唯一 `plan_id`、`formation_id`、offset、车辆规格、`header.frame_id="map"`、非零时间戳；失败请求不进入规划 |
| S4 C++ 前端 | `formation_planner_node.cpp` 仍 latch `/formation/status`（`formation_frontend_status/v1`）；S9 Python 节点改用独立的 `/formation/optimizer_status`，避免两个 JSON schema 争用同一话题 |
| 地图合同 | S9 只接受 `core.Grid`；未知栅格 `-1` 保守视为占用；地图签名由 `grid_signature` 对占用栅格、分辨率、原点、尺寸做 SHA256；不支持旋转地图原点 |
| 发布事务性 | 新请求/失败/地图变化/重复或过期种子不会保留旧轨迹；只有当前 `plan_id` 的 `FEASIBLE` 轨迹可发布；发布前再走一遍 ROS 消息合同校验 |
| 既有行为 | 未修改单车 `ContinuousPlan`、`retime_continuous`、控制器、NMPC、执行器、闭环状态仿真、C++ 规划器和任何 launch/YAML |

## 修改文件

新增：

1. [formation_quality.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_quality.py)：`FormationQualityConfig`、`FormationEvaluationFailure`、`FormationEvaluation`、`FormationTimedCandidate`、`FormationSelection`、`PublicationDecision`、`FormationPublicationGate`、`grid_signature`、`evaluate_formation_trajectory`、`select_formation_candidate`；独立审核器 `_IndependentEvaluator` 及 OBB/线段几何 helper。
2. [test_formation_quality.py](../../tests/test_formation_quality.py)：18 项 ROS 无关独立审核/选解/发布门测试。
3. [test_formation_optimizer_node.py](../../tests/test_formation_optimizer_node.py)：18 项 ROS 无关管线、字段转换、地图转换和事务测试。
4. [logs/s9_targeted_tests.txt](logs/s9_targeted_tests.txt)
5. [logs/s5_s8_targeted_tests.txt](logs/s5_s8_targeted_tests.txt)
6. [logs/full_regression.txt](logs/full_regression.txt)
7. [logs/py_compile.txt](logs/py_compile.txt)
8. [logs/git_diff_check.txt](logs/git_diff_check.txt)
9. [logs/ros_conversion_probe.txt](logs/ros_conversion_probe.txt)
10. [logs/ros_node_smoke_nlp_off.txt](logs/ros_node_smoke_nlp_off.txt)
11. [logs/ros_node_process_nlp_off.txt](logs/ros_node_process_nlp_off.txt)
12. [logs/ros_master_nlp_off.txt](logs/ros_master_nlp_off.txt)
13. [logs/ros_node_smoke_nlp_on.txt](logs/ros_node_smoke_nlp_on.txt)
14. [logs/ros_node_process_nlp_on.txt](logs/ros_node_process_nlp_on.txt)
15. [logs/ros_master_nlp_on.txt](logs/ros_master_nlp_on.txt)
16. [logs/scenario_probe.txt](logs/scenario_probe.txt)
17. [logs/ros_smoke_summary.txt](logs/ros_smoke_summary.txt)
18. [S9_REPORT.md](S9_REPORT.md)

修改：

- [formation_optimizer_node.py](../../catkin_ws/src/fwis_v3_optimizer/scripts/formation_optimizer_node.py)：由 S3 的验证-only 节点升级为 S9 管线/发布节点。本阶段 diff 为 1563 行新增、55 行删除；新增 `FormationPipelineConfig`、`FormationPipelineRejection`、`FormationPipelineCandidate`、`FormationPipelineResult`、`FormationPlanningPipeline`、`FormationNodeEvent`、`FormationNodeCore`、`FormationOptimizerNode`、`grid_from_occupancy_grid`、`timed_trajectory_fields`。未修改既有 `.msg`、launch、C++ 或其他 Python 模块。

没有 C++ 或消息变更，因此本阶段不需要 catkin 重新构建；ROS 探针直接使用 `catkin_ws/devel` 中已有的消息包。

## 独立审核器边界

### 输入与不信任原则

`evaluate_formation_trajectory(snapshot, plan, timed, grid, config)` 的四个输入全部重新检查，不把以下内容当作证据：

- S4 前端候选的 `clear=True`；
- S6 `audit_continuous_plan` 的成功结论（只作为一次交叉核对，并计入 `plan_checks`）；
- S7 NLP 的内部 soft clearance 或求解器状态；
- S8 `timing_report`、`dynamic_profile` 或自审核峰值；
- ROS 节点的 `can_publish` 标记。

独立审核器是 ROS-free 的；`formation_optimizer_node.py` 只是外层边界。`FormationEvaluation` 的 `independent_audit` 恒为 true，`trajectory_published` 恒为 false；只有 `feasible=True` 时 `collision_certificate` 才为 true。

### 审核顺序

`_IndependentEvaluator.run()` 严格按以下顺序执行，任一步失败立即返回结构化失败，不会继续签发证书：

1. `_check_grid_and_signature`：`grid` 必须是 `core.Grid`；SHA256 签名与 `expected_map_signature` 一致；记录 `map_signature`。
2. `_check_request`：`plan_id` 在 uint32 内、`formation_id` 非空、`FormationModel` 合法、start/goal 为 `FormationPose`、header stamp 非零、`allow_reverse` 为 bool、start/goal 航向和 `goal_heading_free` 覆盖全部车辆、车辆形状与 ID 集合一致。
3. `_check_plan`：plan 类型/ID/formation/offset/start/goal/`goal_heading_free`/`allow_reverse` 与 request 一致；start/goal 航向逐车一致；block 索引 0..N−1；block 车辆集合一致；`audit_continuous_plan` 交叉核对；逐 block 重查 A 曲率、O 斜角与航向恒定、S 控制/中心/位移、倒车授权、旋转航向率、block 边界航向连续、首末航向与 request 一致；记录的 segment 起终中心必须等于 `q_F + R(ψ_F) r_i`。
4. `_check_timed_contract`：`FormationTimedTrajectory` 的 `plan_id`/`formation_id`/`offsets` 与 request/plan 一致；`times_from_start[0]=0`、严格递增、有限；每车轨迹点数/时间与共享网格完全一致；`phases` 非空；轨迹车辆 ID 集合一致。
5. `_parse_phases`：phase 必须是 `motion`/`stationary`，时间连续、duration>0，motion phase 按 block 顺序，stationary phase 的 `from_block_index` 正确且不能连续出现；phase 总时长与时间网格末尾一致。
6. `_assign_samples`：每个时间样本必须由恰好一个 phase 覆盖；用 `_quintic(τ)` 反算每个样本的 `station` 与 `q_F`；起点必须是 request.start；初始样本固定为第一个 block 的 `λ=0`。
7. `_check_samples`：逐样本重算 `q_F`、`FormationModel.centers`、`body_headings_at`；检查轨迹中的 `formation_poses`、每车中心、`block_index`、`station`、`stationary`、mode、control、`theta_i`、动态量；对每对车辆检查中心距与固定 offset 距离的残差；静止样本所有动态量必须为 0；终端 `q_F` 必须到 request.goal，非 `goal_heading_free` 的终端车体航向必须到显式目标。
8. `_check_mode_schedule`：逐车合并连续同 mode 区间，检查覆盖 0..T_end、区间连续、每段不小于 `min_mode_dwell_s`；模式或轮角变化必须有整编队 stationary hold，且 hold 时长不小于 `mode_switch_event` 的 `setup_s+steering_s`；方向变化必须发生在静止样本上。
9. `_check_dynamics_peaks`：对每个 motion phase 的每个车辆，在 `_TAUS`（801 个等分 τ + 三个关键 τ）上独立重算 8 项动态量峰值，与 `FormationTimingLimits` 的 `v_max`、`a_max`、`jerk_max`、`yaw_rate_max`、`yaw_accel_max`、`yaw_jerk_max`、`wheel_speed_max`、`lateral_accel_max` 比较；block duration 不得超过 `max_block_duration_s`。
10. `_check_static_collisions`：用 `FormationCollisionChecker` 独立检查起点和终点的每车 OBB vs 地图/边界、每两车 OBB；起点失败 `START_FORMATION_COLLISION`，终点失败 `GOAL_FORMATION_COLLISION`。
11. `_check_sweeps`：对 timed 采样状态做整段扫掠，失败 `FORMATION_SWEEP_COLLISION`；记录最小障碍物净空和最小车间净空。
12. `_check_publication`：把实际 timed 采样时间与 `publication_dt_s` 网格合并，按线性插值重建每个发布时刻的 `q_F` 与各车体航向，再独立扫掠；失败 `PUBLICATION_SWEEP_INVALID`。
13. `_success`：所有 metrics 必须有限，才返回 `FEASIBLE`，detail 明确写出独立审核范围。

### 失败码与状态映射

| 失败码 | 独立审核触发条件 | 结果状态 |
|---|---|---|
| `INVALID_REQUEST` | request/map 合同、ID、offset、时间戳、地图签名 | `INVALID` |
| `INVALID_TIMED_TRAJECTORY` | timed 结构、时间轴、phase、`formation_poses`、起终合同 | `INVALID` |
| `START_FORMATION_COLLISION` | 起点 OBB 撞地图/边界或车间重叠 | `REJECTED` |
| `GOAL_FORMATION_COLLISION` | 终点 OBB 碰撞 | `REJECTED` |
| `FORMATION_SWEEP_COLLISION` | 运动扫掠碰撞 | `REJECTED` |
| `KINEMATIC_INFEASIBLE` | A/O/S 模型、航向率、轮角、曲率、斜角、S 中心 | `REJECTED` |
| `MODE_SCHEDULE_INFEASIBLE` | 模式驻留、换模静止 hold、方向换向不在静止点 | `REJECTED` |
| `FORMATION_NLP_INFEASIBLE` | 管线中的 S7 优化异常/失败 | `REJECTED` |
| `FORMATION_RETIMING_INFEASIBLE` | 时间/动态/驻留、block 时长、静止动态非零 | `GEOMETRY_ONLY_NOT_EXECUTABLE` |
| `FORMATION_RIGIDITY_VIOLATION` | 中心不等于 `q_F + R(ψ_F) r_i`，或车间距漂移 | `REJECTED` |
| `PUBLICATION_SWEEP_INVALID` | 发布采样插值存在碰撞风险 | `REJECTED` |

`FEASIBLE` 是唯一可执行状态；`FormationEvaluation.executable` 仅在 `FEASIBLE` 时为 true，`trajectory_published` 始终为 false。节点层的 `executable` 更严格：只有 `FEASIBLE` 且通过事务门并成功发布 ROS 消息后，顶层状态才为 true。

### 选解

`select_formation_candidate` 先对全部候选逐一独立审核，绝不因某个候选成功而跳过其他候选；只有 `feasible=True` 的候选参与排序：

```text
(common T_end,
 -最小障碍物净空,
 -最小车间净空,
 模式切换数,
 动态峰值比,
 candidate_id)
```

因此第一目标是完整可行，第二目标是公共总时长 `T_end`；时间接近时优先更大净空、更少模式切换、更平滑的动态利用率。每个未选候选都进入 `rejections`，字段为 `candidate_id / feasibility_status / duration / rejection_reason / detail / failure_code`。`T_end` 取 `selected_timed.duration`，即所有车辆共享时间轴的末尾，不用某一辆车的时长代替。

## 事务式发布门与节点管线

### 发布门 `FormationPublicationGate`

- `begin_request(plan_id)`：记录当前请求、立即清除 `published_plan_id`、递增 `version`，返回 `CLEAR/NEW_REQUEST`。
- `offer(plan_id, timed, evaluation)`：只接受当前请求；评价必须为 `FormationEvaluation` 且 `feasible`；timed 必须与评价的 `plan_id`/`formation_id` 一致；成功返回 `PUBLISH`，重复发布当前 plan 返回 `PUBLISH`（refreshing），不产生额外的 clear。
- `fail(plan_id, reason, detail)`：如果失败属于当前请求，清除旧发布并返回 `REJECTED/clear=true`；过期 plan 的失败不会清除当前发布。
- `invalidate()`：无条件清除并递增版本，用于内部异常、消息转换失败和地图失效。

### 节点状态机

`FormationNodeCore` 维护请求快照、地图签名、一个 pending seed、已处理 seed key 和发布门：

- **请求先到**：`on_request` 验证 request，固定当前地图签名（若地图尚未到达则为空），清除旧发布，返回 `REQUEST_VALIDATED/VALIDATED`；此时 `feasible=false`、`executable=false`、`trajectory_published=false`，不宣称 `FEASIBLE`。若已有匹配的 pending seed，请求会重放该 seed；若 pending seed 属于更晚的 `plan_id`，则保留等待未来请求。
- **种子先到**：没有活动请求时缓存为 `SEED_CACHED/PENDING`；请求到达后若 plan/formation 匹配则重放。
- **匹配种子**：执行 `FormationPlanningPipeline.run`：S6 构建全部候选 → S6 master 碰撞审计 → 可选 S7 NLP → S8 公共时间 retiming → S9 独立评审和选解。任何阶段失败都变成结构化 rejection，不会被包装成成功。
- **没有候选通过时间**：若全部 rejection 都是 `FORMATION_RETIMING_INFEASIBLE` 且没有候选进入 S9，返回 `GEOMETRY_ONLY_NOT_EXECUTABLE`；其余失败返回 `REJECTED`/`INVALID`。
- **有候选**：全部候选送 S9；选解成功才 `FEASIBLE`，随后 `gate.offer`；S9 失败则 `gate.fail` 并清除旧轨迹。
- **重复/过期种子**：同一 `(plan_id, seed_stamp, map_signature, primitive_key)` 返回 `DUPLICATE_SEED`，不清除当前发布；旧 `plan_id` 返回 `SEED_STALE`，不清除；新 `plan_id` 缓存为 pending。
- **地图变化**：当前有发布且签名改变时，立即 `MAP_CHANGED_CLEARED` 并清除；请求与种子之间地图签名不一致时 `MAP_VERSION_MISMATCH`；无地图时 `MAP_UNAVAILABLE`；迟到的合法地图可以重放缓存 seed。
- **消息发布前校验**：`_publish_event` 在发布轨迹前调用 `validate_formation_timed_trajectory(message)`；任何字段/时间/车辆不一致都会回滚为 `PUBLICATION_REJECTED` 并发布空轨迹。

### 节点输出话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/formation/plan_request` | `FormationPlanRequest` | 订阅，输入请求 |
| `/formation/plan_seed` | `FormationPlanSeed` | 订阅，输入 S6 几何种子 |
| `/map` | `OccupancyGrid` | 订阅，精确栅格；要求 `frame_id="map"`、原点无旋转 |
| `/formation/timed_trajectory` | `FormationTimedTrajectory` | latch 发布；清空消息为空 vehicles，且带 plan/formation 标识 |
| `/formation/optimizer_status` | `std_msgs/String` | latch 发布；schema `formation_optimizer_status/v1` |
| `/formation/quality_report` | `std_msgs/String` | latch 发布；schema `formation_quality_report/v1`，含 pipeline/selection/publication |

**话题冲突处理**：C++ S4 前端仍使用 `/formation/status`（`formation_frontend_status/v1`）。S9 Python 节点不覆盖该话题，改用 `/formation/optimizer_status`。S3/S4 报告中的 `/formation/status` 记录保持历史原样；S10 的 smoke/RViz 必须读取 `/formation/optimizer_status`。

### 管线结果与 ROS 字段转换

`FormationPipelineResult.as_status` 明确区分：

- `status=FEASIBLE` 只在 S9 selection 为 `FEASIBLE` 且有 selected timed 时允许；非 FEASIBLE 不得携带 FEASIBLE selection（构造函数强制）；
- `trajectory_published` 恒为 false；
- `independent_audit = selection is not None`；`collision_certificate = status == FEASIBLE`；
- `candidates` 记录 origin（`s6_seed` / `s7_optimized` / `s7_seed_retained`）、duration、certification、optimization 元数据；
- `rejections` 记录 build/master/optimization/retiming 阶段以及 S9 的失败码。

`timed_trajectory_fields` 把库对象转换为纯 Python 字段，不依赖 ROS；adapter 再构造消息：

- `header.frame_id="map"`，stamp 必须非零；
- `times_from_start` 全车共享；每个 `FormationTimedPoint.time_from_start` 使用同一 float 秒；
- 车体航向 `theta` 独立于编队 `formation_poses[].theta`，不默认 `θ_i=ψ_F`；
- `rospy.Duration.from_sec` 与校验器的纳秒比较一致，避免“转换成功但校验失败”；
- 清空消息 `vehicles=[]`、`plan_id`/`formation_id` 对应当前请求。

## 本阶段自检发现的实现问题

以下问题在 S9 实现/联调中发现并修正，属于 S9 代码本身，不是既有模块回归：

1. **ROS 导入覆盖库类**：最初 `except ImportError` 把 `FormationTimedTrajectory` 赋值为 `None`，覆盖了 `formation_retiming` 的库类，导致 `isinstance` 抛 `TypeError` 并变成 `INTERNAL_ERROR`。改为只把 ROS 消息导入别名为 `FormationTimedTrajectoryMsg`；库类始终来自 `formation_retiming`。
2. **pending seed 被新请求误清除**：`on_request` 初稿在检查前清空 `pending_seed`，导致请求前到达的匹配种子丢失。改为先保存 pending seed/stamp，匹配则重放；更晚 `plan_id` 的种子保留给未来请求。
3. **请求事件错误宣称 `FEASIBLE`**：初稿在请求验证成功时返回 `status="FEASIBLE"` 但 `feasible=False`，违反“只有 S9 审核通过才 FEASIBLE”。改为 `VALIDATED`，并让所有非发布事件显式 `feasible=false`。
4. **quality 报告的 event 分类不完整**：地图/重复/缓存状态曾被误归为种子事件；改为显式区分 `request`、`map`、`plan_seed`。
5. **地图为空时的无意义判断**：删除 `if not occupied.any() and occupied.size == 0` 的不可达分支；明确未知栅格 `-1` 作为占用处理，符合“不把未检查的栅格当自由”的合同。
6. **`timed_trajectory_fields` 异常外泄**：库对象字段缺失时可能抛 `KeyError/IndexError`，adapter 会变成未捕获异常。改为捕获并重抛 `FormationValidationError("INVALID_TIMED_TRAJECTORY ...")`，同时补 plan/formation 身份、时间网格、pose 数、车辆 ID 集合检查。
7. **`/formation/status` 话题被两个 schema 争用**：C++ S4 前端与 Python 节点同时 latch 同一话题，会互相覆盖 JSON。改为 `/formation/optimizer_status`，C++ 保持 `/formation/status`。
8. **请求先于地图/种子先于地图**：增加“请求后第一张地图固定为 request map”的语义；seed 在无地图时保留，迟到 `/map` 可重放；地图变化且有发布时清除。
9. **独立审核早期缺陷**：修正 `plan.vehicle_start_headings` 的 mapping/tuple 两种输入解析；初始样本固定为 `stationary=false` 且只在 `t=0` 检查零动态；phase 端点用 `bisect_right` 包含终点；`_dynamic_values` 使用配置的轮系因子；模式区间合并使用完整 tuple；方向换向在边界与 event 终点两处检查静止；终点碰撞记录正确时间；共线 OBB 边求距补全；非法对象计划先做类型预检；smoke 地图碰撞格索引修正。
10. **未使用代码清理**：删除 `_UINT32_MAX`、`_last_selection`、`_decision_status`、`_matching_pending_seed` 等未使用状态/helper，避免两套状态来源。

上述修正不改变 S5–S8 的审核契约，只修正 S9 的独立审核与节点边界。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `python3 -m unittest tests.test_formation_quality tests.test_formation_optimizer_node -v` | 36/36 通过；3.147 s | 0 |
| `python3 -m unittest tests.test_formation_geometry tests.test_formation_kinematics tests.test_formation_optimizer tests.test_formation_retiming -v` | 52/52 通过；1.008 s | 0 |
| `python3 -m unittest discover -s tests -p 'test_*.py'` | 312 项，311 通过、1 个既有错误；12.620 s | 1 |
| `python3 -m py_compile formation_quality.py formation_optimizer_node.py test_formation_quality.py test_formation_optimizer_node.py` | 编译成功，无输出 | 0 |
| `git diff --check` | 无输出 | 0 |
| 源环境 ROS 消息转换探针（见下） | `message_valid=true`；133 采样；2 车；2 轨迹 | 0 |
| 私有 master 11399：`rosrun fwis_v3_optimizer formation_optimizer_node.py _enable_nlp:=false` + smoke | 两阶段均通过；`SMOKE_EXIT=0` | 0 |
| 私有 master 11399：`rosrun fwis_v3_optimizer formation_optimizer_node.py`（默认 NLP）+ smoke | 两阶段均通过；`SMOKE_EXIT=0` | 0 |
| `python3 s9_scenario_probe.py`（ROS 无关场景） | 12 个场景输出，JSON 严格序列化 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 312 tests in 12.620s
FAILED (errors=1)
```

312 = S8 的 276 项 + S9 新增 36 项。没有新增跳过、弱化或删除任何既有测试；未修改 `tests/test_continuous_retiming.py` 或相关重定时模块。

ROS 探针的实际命令为：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
python3 /tmp/s9_ros_probe.py

export ROS_MASTER_URI=http://127.0.0.1:11399
export ROS_HOME=/tmp/s9_ros_home
export ROS_LOG_DIR=/tmp/s9_ros_logs
roscore -p 11399
rosrun fwis_v3_optimizer formation_optimizer_node.py [_enable_nlp:=false]
python3 /tmp/s9_node_ros_smoke.py
```

第一次 `_enable_nlp:=false` smoke 使用 `setsid` 启动，外层 job 提前退出，清理时需按实际 PID 显式 SIGINT 节点和 master；随后确认端口 11399 无监听、无残留 `rosmaster/rosout/formation_optimizer_node`。第二遍默认 NLP 使用普通后台 PID，正常 SIGINT 清理。两轮均未使用受保护端口 11311/11312/11320；测试结束后 `ss -ltn` 确认这些端口无监听。

## 定向测试与验收对应

| 文档验收要求 | 实际测试 | 结果 |
|---|---|---|
| 请求合同：ID/车辆/offset/时间戳/地图签名 | `test_invalid_inputs_and_candidates_fail_closed`、`test_map_signature_and_missing_map_fail_closed`、`test_grid_conversion_fails_closed` | 通过 |
| 轨迹合同：严格递增、全车同网格、起终正确、θ 展开 | `test_nonmonotonic_shared_time_grid_is_rejected`、`test_inconsistent_vehicle_time_axis_is_rejected`、`test_tampered_formation_pose_is_invalid_trajectory`、`test_rotation_pipeline_fields_keep_formation_and_body_headings` | 通过 |
| 刚性几何：中心来自共同 `q_F`，offset 固定 | `test_tampered_center_is_rigidity_violation`、`test_tampered_formation_pose_is_invalid_trajectory`；每个 FEASIBLE 场景 `max_rigidity_error_m` ≤ 3.8e-15 | 通过 |
| 独立碰撞：起点/终点/运动扫掠/发布采样 | `test_start_goal_and_sweep_collisions_have_distinct_codes`、`test_publication_sample_sweep_is_recomputed`、`test_obstacle_is_rejected_before_feasible` | 通过 |
| 运动学：A 曲率、O 航向、S 中心 | `test_ackermann_curvature_uses_independent_limits`、`test_spin_center_motion_and_oblique_heading_change_rejected`、`test_tampered_heading_is_kinematically_infeasible` | 通过 |
| 时间/动态：全车包络、驻留、换向 | `test_short_mode_dwell_is_mode_schedule_infeasible`、`test_short_hold_is_mode_schedule_infeasible`、`test_valid_rotation_and_chain_are_feasible`（`mode_switch_count=2`、`direction_change_count=1`） | 通过 |
| 只有全部通过才 `FEASIBLE` | 所有 FEASIBLE 测试断言 `certification=FEASIBLE`、`collision_certificate=true`、`independent_audit=true`、`trajectory_published=false`；所有失败测试断言非 FEASIBLE | 通过 |
| 几何可行、时间不可行 → `GEOMETRY_ONLY_NOT_EXECUTABLE` | `test_retiming_failure_is_geometry_only_not_feasible` | 通过 |
| 选解按公共 `T_end` | `test_selection_evaluates_all_and_uses_common_t_end`：3 候选全评审，2 可行，选最快，记录 rigidity rejection | 通过 |
| 发布事务性 | `test_publication_gate_is_transactional` + 10 项节点事务测试（请求/种子/重复/过期/地图/缓存/重放） | 通过 |
| 旧轨迹不残留 | ROS smoke 的 invalid seed 阶段：空 `vehicles`、`published_plan_id=null`、`publication_version` 增加；节点单测覆盖新请求/无效请求/地图变化 | 通过 |
| 发布前消息合同 | `test_rotation_pipeline_fields_keep_formation_and_body_headings`、ROS 消息转换探针 `message_valid=true`、ROS smoke 发布前后校验 | 通过 |
| 既有单车回归 | 完整回归 312 项，只保留 S0 的 `dynamic_peaks` 错误 | 通过 |

### 程序化场景结果摘录

| 场景 | 结果 |
|---|---|
| 两车直线平移 2 m | `FEASIBLE`；16313 项检查；时长 4.7109375 s；237 采样/472 发布采样；最大刚性误差 3.77e-15 m；最小障碍净空 1.25 m、最小车间净空 0.462 m；动态峰值比 0.9950 |
| 纯旋转 0.5 rad，offset (0,0)、(1.2,0) | `FEASIBLE`；9616 项检查；时长 2.6338451 s；133/264；刚性误差 9.93e-16 m；净空 1.1037/0.5810 m；动态峰值比 0.9851 |
| 旋转 0.3 + 两段反向 O 平移链 | `FEASIBLE`；55925 项检查；时长 18.4455569 s；927/1849；刚性误差 2.16e-15 m；净空 1.131/0.581 m；模式切换 2、方向换向 1、静止 phase 2；动态峰值比 0.9901 |
| 起点障碍 | `REJECTED`；`START_FORMATION_COLLISION`；vehicle 0、time 0、detail `start MAP_OBB_COLLISION block/sample 0/0` |
| 终点障碍 | `REJECTED`；`GOAL_FORMATION_COLLISION`；vehicle 0、time 4.7109375 |
| 中段障碍 | `REJECTED`；`FORMATION_SWEEP_COLLISION`；vehicle 0、segment 68、sample 1、time 1.3573888 |
| 3 候选选解（快/慢/篡改） | `FEASIBLE`；时长 4.7109 / 18.8438 / 4.7109；评价状态 FEASIBLE/FEASIBLE/REJECTED；选中 1；`T_end=4.7109`；拒绝原因 `FORMATION_RIGIDITY_VIOLATION` |
| 发布门序列 | begin → clear；bad offer → REJECTED+clear；good offer → PUBLISH；重复 good → PUBLISH（不清除）；新请求 18 → clear；旧 plan 17 offer → STALE_PLAN、不发布 |
| 节点直线管线 | `FEASIBLE`；2 个 `s6_seed` 候选；选中 1；`T_end=4.7109`；`trajectory_published=false` |
| 节点旋转管线 | `FEASIBLE`；1 个候选；`T_end=2.6338` |
| 节点中段障碍管线 | `REJECTED`；0 admitted；2 个 master rejection，`COLLISION_PRECHECK_FAILED` |
| 节点紧时限管线 | `GEOMETRY_ONLY_NOT_EXECUTABLE`；0 candidates；2 个 retiming rejection，`FORMATION_RETIMING_INFEASIBLE` |
| ROS 消息转换探针 | `message_valid=true`；plan 77；2 车；133 采样；formation θ 0→0.5；vehicle 1 body θ 1.5707963→2.0707963；clear plan 77 |
| ROS smoke（NLP off） | 启动 → `REQUEST_VALIDATED`；seed → `FEASIBLE_PUBLISHED`，duration 2.6338451，`executable=true`，`trajectory_published=true`，publication version 1；invalid seed → `REJECTED`、`INVALID_PRIMITIVE`、published false、version 2、空轨迹 |
| ROS smoke（默认 NLP） | 同上；启动日志出现 Ipopt，仍为 `FEASIBLE_PUBLISHED`，说明 S7 路径与无 NLP 路径都走同一 S9 审核/发布门 |

## 未执行项目与边界

1. **只处理静态占据栅格**：没有动态障碍、行人预测、通信延迟、执行器带宽或真实轮系响应；S9 的碰撞证书只对输入 `Grid` 成立。
2. **数值采样而非区间证明**：动态峰值在 801 个等分 τ 与 3 个关键 τ 上采样；碰撞扫掠用 `sweep_spacing_m` 加密 + `max_samples_per_segment` 防护。它们不是连续区间算术证明；但发布采样额外加密，且任何 guard 触发都会失败闭合。
3. **发布插值合同**：`_check_publication` 按线性插值 `q_F` 与车体航向重建发布时刻；如果下游 RViz/控制器使用不同的角度插值方式，安全性不由本模块保证。S10 回放必须使用同一线性时间插值。
4. **`frame_id` 的检查边界**：ROS request 的 `header.frame_id` 由 `validate_formation_request` 强制为 `"map"`，`/map` 的 `frame_id` 由 `grid_from_occupancy_grid` 强制为 `"map"`；`FormationRequestSnapshot` 不保留 `header`，因此 ROS-free evaluator 不再重复检查该字符串。直接调用 evaluator 的代码必须自行保证坐标系语义。
5. **`executable` 的三层语义**：`FormationEvaluation.executable=true` 表示 S9 独立审核通过；`FormationSelection.as_status()["executable"]`/`FormationPipelineResult.as_status()["executable"]` 为 true 表示选出了可执行候选；两者都不等于已发布。只有节点顶层状态在 `FEASIBLE_PUBLISHED` 时才 `executable=true` 且 `trajectory_published=true`。下游必须读节点状态或 `trajectory_published`，不能只看 pipeline 字段。
6. **S6 计划审计复用**：`_check_plan` 调用 `audit_continuous_plan` 作为交叉核对，同时独立重算逐 block 动力学/中心/航向/时间采样；若 S6 与 S9 共享同一审计盲点，仍可能漏检。碰撞与时间曲线由 S9 独立重算，不复用 S6 的 `clear`。
7. **单 pending seed**：节点只缓存一个“比当前请求更新”的种子；同一 `plan_id` 的重复种子按 key 去重，不能用第二个种子替换已发布轨迹（除非它是新的 seed stamp 且重新走完 S9）。
8. **单线程回调**：`rospy.Subscriber(queue_size=1)` + 同步规划；长管线会阻塞回调队列，没有异步 worker、取消或超时。重规划期间新的 request/seed 可能排队，S10 需评估实时性。
9. **地图签名不含元数据**：签名覆盖占用栅格、分辨率、原点、尺寸；不覆盖 `frame_id`、`info.map_load_time` 或发布者。地图语义变化但栅格相同的情况不会被签名捕获。
10. **没有全局搜索/多候选调度**：S7 在 S6 admitted 候选上逐个优化，没有跨候选联合搜索；失败只按候选记录，不重新生成新几何。
11. **没有控制器/执行器接口**：S9 只发布 `FormationTimedTrajectory` 与状态 JSON；不调用控制器、NMPC、执行器，不改变闭环状态仿真。
12. **ROS smoke 不是实车实验**：只在私有 master 11399 上验证消息、事务和清空；没有 RViz、没有真实车辆、没有性能基准。

## 安全性审计

- **不信任上游结论**：S9 重新计算 request/plan/timed 的身份、时间轴、刚性中心、车体航向、A/O/S 模式、动态峰值、碰撞与发布采样；S6/S7/S8 的成功标记不是证书。
- **刚性合同唯一来源**：每个样本中心由 `q_F` 和固定 offset 重新派生，并再次与 `FormationModel.centers` 比较；车间距也做 pairwise 残差检查。
- **不默认 `θ_i=ψ_F`**：`body_headings_at` 独立重算；旋转 block 要求 A/S 的航向率等于 `dpsi/L`，平移 block 不允许改车体航向；O 模式要求航向恒定。
- **A/O/S 不混淆**：A 检查曲率/轮角，O 检查斜角/航向恒定，S 检查 control=0、中心不动、中心速度为 0；倒车方向必须由 request 的 `allow_reverse` 授权。
- **全停换模**：模式或轮角变化必须有 stationary hold，且 hold 时长不低于 `mode_switch_event` 的最大需求；静止样本所有动态量必须为零。
- **共同时间**：`times_from_start` 全车共享、严格递增、起点 0；`T_end` 取共同时间轴末尾，选解不按单车时长。
- **碰撞证书**：起点、终点、整个 timed 扫掠、以及按发布 `dt` 重新插值的发布采样分别扫掠；任一 OBB 接触地图/边界/另一车都失败。
- **失败闭合**：所有非法输入要么抛 `FormationValidationError`（公开 API），要么返回 `INVALID`/`REJECTED`/`GEOMETRY_ONLY_NOT_EXECUTABLE` 的结构化失败；不存在“审核失败但仍 `FEASIBLE`”的分支。
- **发布事务**：新请求立即清除旧发布；失败/重复/过期/地图变化不会保留旧轨迹；发布前 ROS 消息校验失败会回滚清除。
- **不污染既有系统**：未修改单车模块、C++、`.msg`、launch、控制器、NMPC、执行器或闭环仿真；未修改 `archive/`、`build/`、`devel/`。
- **测试可复现**：36 项定向测试的 offsets、几何、失败注入索引、地图障碍位置和时限全部写死；ROS smoke 使用私有端口和独立 ROS_HOME/日志目录，结束后确认无监听、无残留进程。

## 存在风险

1. **数值包络漏峰**：`_TAUS` 的 801 点 + 关键 τ 覆盖当前 quintic 的主要极值，但不是连续证明；组合范数的峰值可能落在采样点之间。S9 的 `dynamics_tolerance_ratio=1e-6` 仅用于数值一致性，不提供安全裕量。
2. **发布插值假设**：发布采样用 timed 样本之间的线性 `q_F`/航向插值；若消费者用不同的插值（例如对 `psi` 做最短角插值或多项式），实际轨迹可能偏离证书。
3. **共享 S6 审计盲点**：`_check_plan` 仍使用 S6 `audit_continuous_plan` 做一次交叉核对；如果同一个公式在 S6 与 S9 中都错，独立采样检查只能覆盖采样点。
4. **地图语义丢失**：`frame_id`/`map_load_time` 不在签名内；相同栅格的不同地图不会被识别。
5. **单线程长阻塞**：规划在 ROS 回调线程中同步执行；大地图/多候选时可能阻塞新请求，旧请求的取消语义未实现。
6. **只缓存一个 pending seed**：更高 `plan_id` 的多个种子先后到达时，后一个会覆盖前一个；S10 需要明确实验脚本的发布顺序。
7. **`GEOMETRY_ONLY_NOT_EXECUTABLE` 的语义**：它表示几何候选存在但公共时间不可行；不是“可执行但未认证”。下游必须把它当作不可执行。
8. **Str 状态 JSON 无版本协商**：`formation_optimizer_status/v1`、`formation_quality_report/v1` 是字符串 schema；S10 若新增字段必须保持向后兼容或升级 schema。
9. **未覆盖真实执行器限制**：轮角/轮速模型来自现有 `wheel_configuration` 与 `FormationTimingLimits`；没有转向速率实测、通信抖动或制动延迟。
10. **ROS smoke 只验证两个场景**：一个成功的旋转和一个非法 primitive；更复杂的地图变化、重复 seed、请求重放由 18 项 ROS 无关事务测试覆盖，但没有在真实 ROS 网络中逐项重跑。

## 完成判据与 S10 前置条件

- 独立审核重算 request/trajectory/rigidity/collision/kinematics/timing/publication：**已实现**。
- 只有全部子项通过才 `FEASIBLE`：**已实现并由代码不变量与测试确认**。
- `GEOMETRY_ONLY_NOT_EXECUTABLE` 与 `INVALID`/`REJECTED` 区分：**已实现并测试**。
- 结构化失败码与诊断字段：**已实现**；`FormationEvaluationFailure.as_dict` 覆盖 code/field/vehicle/other/pair/block/segment/sample/time/station/constraint/max_error/detail。
- 按公共 `T_end` 选解：**已实现并测试**。
- 事务式发布、失败/新请求/地图变化清除旧轨迹：**已实现并测试**；ROS smoke 验证 invalid seed 清空。
- 发布前 ROS 消息合同：**已实现**；探针与 smoke 均通过。
- 既有单车回归：**无新增失败；既有 `dynamic_peaks` 错误保留**。
- 未修改控制器/NMPC/执行器/闭环仿真/C++/消息：**已确认**。

S10 前置条件：

1. 回放必须使用 `FormationTimedTrajectory.header.stamp` 作为统一起点，所有车辆共享 `time_from_start`，不得为每车单独设起点。
2. Marker namespace 必须包含 `formation_id` 与 `vehicle_id`；TF frame 必须唯一，不能共享裸 `base_link`。
3. 失败/几何-only 轨迹必须明确标注 `GEOMETRY_ONLY`，不得以成功颜色/状态回放。
4. `ros_formation_smoke.py` 必须检查消息 schema、实际回放采样、空结果清理、全车 Marker、终端误差和独立审核报告，不能只看屏幕。
5. 固定 E01–E20 用例与指标、重复运行（≥3 次）、消融实验必须保留失败样本，不得删除失败。
6. smoke/RViz 读取 `/formation/optimizer_status`，不要与 C++ `/formation/status` 混用。