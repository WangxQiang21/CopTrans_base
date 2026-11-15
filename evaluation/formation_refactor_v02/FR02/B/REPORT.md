# FR02-B 独立审核、真实扫掠与模型/投影一致性报告

- 任务：`docs/CLAUDE_TASK_02.md` 第 5 节 **FR02-B**
- 工作区：`/home/wq/whut_thesis/expv3_multi_v02`（git worktree）
- 分支 / HEAD：`refactor/formation-v02` / `6227833011e2d92b711179bfdec979bb0b878b29`
- 前置：`FR02/A/CODEX_ACCEPTANCE.json` 记录 `ACCEPTED`，`a_allowed_to_enter_fr02_b=true`
- 状态：**B 已执行完毕，停止并交 Codex 独立验收**。
- 未进入 FR03/FR05，未接 ROS/RViz、消息、NLP、控制或动力学；未提交 Git；未创建 worktree；未访问原仓库。
- 证书边界：成功结论来自独立审核 + 解析扫掠碰撞，但**弧/复合场景的连续时间加速度证书不成立**（见第 8 节），不得把它解读为严格连续时间证书。

`P/` = `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

## 1. 准入基线与只读核验

执行 B 前已保存 FR02-A 的源码、测试和证据快照：

- `pre_b/sources/`：A 阶段 8 个源码文件；
- `pre_b/tests/`：A 阶段 5 个测试文件；
- `pre_b/evidence/`：`CODEX_ACCEPTANCE.md/json`、`REWORK_REPORT.md`、`evidence_manifest.json`；
- `pre_b/baseline.json`：上述文件的 SHA-256 与 A 阶段 git status。

A 的两个新模块在 B 期间只读，字节未变：

| 文件 | SHA256（A = B） |
|---|---|
| `P/uniform_motion.py` | `1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2` |
| `P/uniform_timing.py` | `91678441d589e48e5bb81c688bce8518df4bc2d4119a7de749fff3ba4dd9af18` |
| `P/uniform_trajectory.py` | `a93c2b2faba8bda7ac4592888cd502ea3850e6138505df851a12f44a5e86b579` |

B 未发现需要停止 A 验收的真实 A 缺陷；未修改 A 的两个新模块。

## 2. 文件变更

新增：

| 文件 | 作用 |
|---|---|
| `P/uniform_quality.py` | 独立审核入口 `audit_uniform_timed_plan`、`UniformAuditRequest`、`_UniformAuditor` |
| `tests/test_uniform_quality.py` | B01--B08 验收、篡改注入与机器证据生成 |

允许的定点扩展：

| 文件 | 变更 |
|---|---|
| `P/formation_collision.py` | `AnalyticSweepSegment`、`swept_check_analytic`、`analytic_segment_bound`；复用原 OBB/SAT、地图/车间碰撞、安全 margin 与采样预算 |
| `P/formation_quality.py` | `_check_block_kinematics(..., uniform_v02=False)`；仅允许零角 S 车的规范 S 轮系、方向占位和零实际轮速 |

以下文件与 `pre_b` 逐字节一致（`cmp -s` + SHA-256）：

```text
formation_geometry.py, formation_kinematics.py, formation_retiming.py,
uniform_motion.py, uniform_timing.py, uniform_trajectory.py,
tests/test_formation_collision.py, tests/test_formation_quality.py,
tests/test_uniform_motion.py, tests/test_uniform_timing.py,
tests/test_uniform_trajectory.py
```

完整 diff 见 `diff/B_changes_vs_pre_b.diff`，统计见
`diff/B_changes_vs_pre_b.stat`：

```text
formation_collision.py         +259   -5
formation_quality.py           +52    -17
uniform_quality.py             +3248  -0     (new)
tests/test_uniform_quality.py  +1778  -0     (new)
```

当前源码/测试 SHA-256：

| 文件 | SHA256 |
|---|---|
| `P/formation_collision.py` | `31ea1831b0fd4fe05ff9292e560ea11d8b7392787fba3f4971d157c961fd5fc5` |
| `P/formation_quality.py` | `95830366a57639cfb47329654fcaa2042dd252bb9a15c12b45178d203a6c7511` |
| `P/uniform_quality.py` | `9fd2144b3a0cb3daf26262ea8b6ba130c6ef64591cccf0aff7dc91679d45a3c3` |
| `tests/test_uniform_quality.py` | `0b0b8167764960a17155c59ac01e9a897552d7a39fd96f4bace5aebb889f1d3f` |

## 3. 独立审核设计

`audit_uniform_timed_plan(request, plan, projection, grid, config)` 的输入是
原始审核请求、权威 V0.2 模型、采样投影、真实地图、车辆形状与限值；它不读取
S8 峰值报告，也不以 FR01 校验成功或 JSON 摘要代替实值审核。审核顺序：

```text
grid/signature -> request -> plan identity -> edges -> phase axis ->
model nodes -> interior states -> dynamic peaks -> projection contract ->
static collisions -> analytic sweeps -> success certificate
```

每个失败都通过 `_EvalError` 变成结构化 `FormationEvaluation`：`status`、
`reason`、`field`、`vehicle_id`、`block_index`、`constraint`、`time_s`、
`station`、`max_error`、`detail`。失败路径不会返回成功证书或可用投影。

### 3.1 独立推导

- 复用 S9 `_IndependentEvaluator` 的独立几何/运动核和 `_dynamic_values`；
- 从模型 offsets、形状、边工厂参数、方向、曲率/斜角、控制量和五次时间律重新
  推导各车中心、速度/加速度/jerk、车体 yaw、轮角/轮速和横向加速度；
- 不读取 S8 `FormationTimingReport` 的峰值结论作为审核依据；
- A/O 中心速度按 V0.2 `σ||p_dot||`，S/setup 为 0；yaw 保留符号；轮速取绝对值；
- 零角 S 车保留 direction `+1` 占位、实际轮速为 0，但首末轮角/因子必须是规范 S
  轮系，不能替换为 A 零曲率轮系。

### 3.2 时间轴与状态

- 检查每个相位 `t0_s/duration_s/end_time`、初始/相邻 setup 时长、setup 需求
  （最慢车下界）、motion 最短时长和公共时间轴连续；
- 在每个相位边界两侧、模型节点、投影时刻和每个相位 ≥11 个内部时刻
  比较解析状态；
- 逐项比较 q_F、各车中心、body theta、mode、signed center speed、yaw/轮系字段；
- 不从投影点列插值反推轨迹；投影只是被审核对象。

### 3.3 动态峰值候选与保守界

动态审核不把 11 个内部样本当作极值证明。它复用 S9/S8 共用的 803 个候选
`_TAUS`（801 个均匀点 + 3 个五次临界点，去重），并补充 setup 转向
rate/acceleration 极值。每个块/车辆的候选峰值、峰值 τ、保守界、界比和
是否具备连续证书都写入 `dynamic_evidence` / `dynamic_limitations`。

五次时间律使用的上界常数：

```text
sup |lambda_dot|   = 1.875 * L / T
sup |lambda_ddot|  = (10/sqrt(3)) * L / T^2
sup |lambda_dddot| = 60 * L / T^3
```

候选峰值超过限值时按 S9 更严格的 `dynamics_tolerance_ratio=1e-6` 拒绝。
候选峰值未超限、但保守界超过限值时，审核结果仍可为 `FEASIBLE`，但
`dynamic_continuous_certificate=false`，并明确记录限制（见第 8 节）。

### 3.4 解析扫掠碰撞

碰撞使用真实解析状态，不在 q_F 端点间线性插值。`swept_check_analytic` 对每条
motion 边按 λ 细分：

```text
B_i <= (sup ||p_i'(lambda)|| + body_half_diagonal_i * sup |theta_i'(lambda)|) * Δlambda
```

- 每段 `B_i <= SweepPolicy.max_corner_step_m = 0.01 m`；
- 默认 `motion_padding_m = 0.01 m`，`collision_margin_m = 0.05 m`；
- 每段预算 `max_samples_per_segment = 200000`，超预算显式失败
  `UNIFORM_SWEEP_BUDGET_EXCEEDED`，不缩 margin/步长强行通过；
- A 弧/Spin/平移的导数范数为常数，使用解析上界；setup 位姿固定；
- 使用原地图、车间 OBB、静态边界和完整车体 Spin 扫掠；
- 成功后记录 `sweep_policy`、`sweep_coverage`（每边界的 `bound_m`、样本数、
  首末时间）和 `first_collision`/完整解析状态。

## 4. 阈值

`uniform_quality_results.json` 的 `acceptance_thresholds` 与测试断言一致：

| 项目 | 阈值 |
|---|---|
| 几何/内部状态误差 | `1e-7` m 或 rad |
| 车头误差 | `1e-7` rad |
| 投影刚性误差 | `1e-6` m |
| 归一化模式残差 | `1e-6` |
| 非端点/非零速有限差分 | `1e-4` |
| 时间连续 | `1e-9` s |
| 端点速度/加速度/进度加速度 | `1e-9` |
| 动态峰值比（任务） | `1e-3` |
| 动态峰值比（S9，采用更严格者） | `1e-6` |
| setup 位姿 | `1e-9` m/rad |

## 5. B01--B08 结果

完整机器结果见 `uniform_quality_results.json`；每条篡改的完整前后输入位于
`tampers/`，合法模型的完整 plan model dict 位于 `models/`。

| ID | 测试 | 结果 |
|---|---|---|
| B01 | 9 个合法 A/O/S 场景（含 A 直行正/倒、非零枢轴弧、O 正/倒、O 同模式换轮、S 零角车、S-A-S-O 复合、零运动 hold）+ 模型往返 | PASSED |
| B02 | 车头整体偏切线但保持中心路、方向、曲率、O signed speed 篡改 | PASSED |
| B03 | 全局/逐车 mode 冲突、S 零角换 A 轮、轮因子、轮 ID、NaN；合法 spin 正例 | PASSED |
| B04 | 缩短 motion、缩短/删除 setup、漏最慢车、时间轴 gap/overlap、提前切 mode | PASSED |
| B05 | 只改投影中心/货物姿态/展开车头/时间/模式/速度/yaw、整体刚性平移、弦插值 | PASSED |
| B06 | 直行与复合 A 弧的中途扫掠碰撞，端点均无碰撞 | PASSED |
| B07 | 0.02 s / 0.05 s 投影在共同非网格时刻一致；弦插值拒绝 | PASSED |
| B08 | 未知模型版本/投影语义、非法地图签名、plan/model/certification 身份不符、扫掠预算耗尽 | PASSED |

### 5.1 B01 合法审核数值

9 个合法场景的审计最大值：

| 指标 | 最大值 |
|---|---|
| `max_state_error_m_or_rad` | `1.7763568394002505e-15` |
| `max_projection_error_m_or_rad` | `1.1102230246251565e-16` |
| `max_rigidity_error_m` | `6.661338147750939e-16` |
| `max_finite_difference_error` | `9.129495415205469e-06` |
| `max_interior_geometry_error_m_or_rad` | `1.7763568394002505e-15` |
| `max_wheel_start_error_rad` | `0.0` |
| `max_wheel_end_error_rad` | `0.0` |
| `max_normalized_mode_residual` | `0.0` |
| `max_mode_residual_rad` | `0.0` |

每个 motion 相位有 17 个内部时刻，每个 setup 相位有 11 个内部时刻，均 ≥11。
所有边界跳变均为 `0.0`。hold 场景无运动相位且 `duration_s=0.0`，其 `hold` 计数
为 0，不存在可采样的内部区间。

模型往返：`plan.model_sha256()` 与
`UniformTimedPlan.from_model_dict` / `from_model_json` 后的哈希一致；权威状态在
5 个时间点（hold 为 0）的最大差 ≤`1e-12`。工厂元数据会把 `-0.17` 重建为
`-0.17000000000000004`，因此不使用 `to_model_dict()` 的逐位相等作为断言，
而使用模型哈希 + 权威状态实值相等（这是元数据重建，不是模型缺陷）。

### 5.2 B02--B05 拒绝位置摘要

| 子例 | 代码 | 字段 / 车辆 / 约束 |
|---|---|---|
| B02 coherent heading | `UNIFORM_MODEL_MISMATCH` | `edge0.start_heading[2]`，车辆 2，误差 0.01 |
| B02 spin direction | `UNIFORM_MODEL_MISMATCH` | `constraint=direction`，车辆 1 |
| B02 arc curvature | `UNIFORM_MODEL_MISMATCH` | `edge0.curvatures.2`，误差 0.01 |
| B02 O signed speed | `UNIFORM_MODEL_MISMATCH` | `edge0.signed_progress_speeds.3`，误差 0.1 |
| B03 global mode | `UNIFORM_MODEL_MISMATCH` | `mode`，block 0 |
| B03 segment mode | `UNIFORM_MODEL_MISMATCH` | `segments.mode`，车辆 1 |
| B03 zero-angle S -> A wheels | `UNIFORM_MODEL_MISMATCH` | `edge0.wheel_angles_start[2][0]`，误差 0.936 |
| B03 wheel factor | `UNIFORM_MODEL_MISMATCH` | `edge0.wheel_factors_start[1][0]` |
| B03 wheel ID | `UNIFORM_MODEL_MISMATCH` | `wheel_angles_start ids must be sorted exactly as (1, 2, 3)` |
| B03 NaN | `UNIFORM_MODEL_MISMATCH` | `edge0.signed_progress_speeds.1.actual` |
| B04 shorten motion | `UNIFORM_PHASE_SCHEDULE_INVALID` | `duration_s`，车辆 1，`constraint=speed` |
| B04 shorten initial setup | `UNIFORM_PHASE_SCHEDULE_INVALID` | `duration_s`，block 0 |
| B04 shorten adjacent setup | `UNIFORM_PHASE_SCHEDULE_INVALID` | `duration_s`，block 1 |
| B04 delete initial setup | `UNIFORM_PHASE_SCHEDULE_INVALID` | `phases`，block 1 |
| B04 omit slowest demand | `UNIFORM_PHASE_SCHEDULE_INVALID` | `demands`，车辆 1 |
| B04 time gap | `UNIFORM_PHASE_SCHEDULE_INVALID` | `t0_s`，`time_s=5e-10` |
| B04 time overlap | `UNIFORM_PHASE_SCHEDULE_INVALID` | `t0_s`，`time_s=2.343...` |
| B04 early mode switch | `UNIFORM_PHASE_SCHEDULE_INVALID` | `demands.to_mode`，车辆 2 |
| B05 projection only (12 子例) | `UNIFORM_PROJECTION_MISMATCH` | `projection.state[...]`、`timed.point/formation_poses/mode` 等 |
| B05 rigid translation | `UNIFORM_PROJECTION_MISMATCH` | `projection.state[0].qf.x`，即使刚性完全保持仍拒绝 |

B02 的 coherent heading 篡改保留全部车中心路径，只把车辆 2 的 start/end body
heading 与 segment theta 同移 0.01 rad；审核仍以 `edge0.start_heading[2]`
拒绝，证明“只查中心/切线”不足以通过。

轮 ID 篡改会被 `to_model_dict()` 按车辆 ID 规范排序而“看起来未变”；证据同时
保存 `tampers/b03_wheel_id_tamper.plan.live_before.json` /
`.live_after.json`，其中 `wheel_angles_start[1][0]` 从 2→3、
`[2][0]` 从 3→2，避免篡改因规范化而丢失。

### 5.3 B06 碰撞证据

`collision_map_b06.json` 保存两张真实地图、障碍 cell、端点检查、首次碰撞的
时间/station/qf 与完整解析状态。

| 场景 | 空地图 | 障碍地图 | 首次碰撞 | 车辆 | station |
|---|---|---|---|---|---|
| 6 m 直行 | FEASIBLE，sig `195f0faa...` | REJECTED，sig `2c4ffffd...`，9 cells | `t=6.6958125` | 1 | `0.4737777778` |
| S-A-S-O 的 A 弧 | FEASIBLE，sig `195f0faa...` | REJECTED，sig `1822789d...`，9 cells | `t=8.894607841496018` | 1 | `0.2554455446` |

两例起终点 OBB 均无碰撞（`endpoint_checks` 全部 `success=true`），碰撞只发生在
中途扫掠。首次碰撞 qf 由模型重新计算，误差分别为 `0.0` 与 `4.44e-16`；障碍
不在碰撞中点采样上（弧场景 `|station-0.5| > 0.05`）。

### 5.4 B07 采样率一致性

同一 `S_A_S_O_complex` 模型：

- 0.02 s 投影：1162 个样本；
- 0.05 s 投影：467 个样本；
- 共同非网格时刻 `0.137 / 0.419 / 0.777 s` 的两投影状态 `as_dict()` 逐位相同，
  且等于 `plan.state_at(t)`；
- 两投影均通过完整审核；
- 把 0.05 s 投影中 A 弧附近一个样本替换为前后邻点的弦中点，并整体重写该时刻
  的刚性阵型（`rigidity_error_m=0.0`），审核仍以
  `projection.state[202].qf.x` 拒绝（误差 `1.16e-4`）；
- 明确不比较两采样率下的离散净空数值（`discrete_clearance_values_compared=false`）。

### 5.5 B08 显式失败

| 子例 | 状态 | 代码 / 字段 |
|---|---|---|
| 未知模型版本 | `REJECTED` | `UNIFORM_PROJECTION_MISMATCH` / `projection.model_version` |
| 未知投影语义 | `REJECTED` | `UNIFORM_PROJECTION_MISMATCH` / `projection.semantic_version` |
| 非法地图签名 | `INVALID` | `INVALID_REQUEST` / `map_signature` |
| plan_id 不符 | `REJECTED` | `UNIFORM_PROJECTION_MISMATCH` / `projection.plan_id` |
| model hash 不符 | `REJECTED` | `UNIFORM_PROJECTION_MISMATCH` / `projection.model_sha256` |
| certification 不符 | `REJECTED` | `UNIFORM_PROJECTION_MISMATCH` / `projection.certification` |
| 扫掠预算耗尽 | `INVALID` | `UNIFORM_SWEEP_BUDGET_EXCEEDED` / `sweep_budget`，block 0，需要 375 > 1 样本 |

预算耗尽不被报告为 `FORMATION_RETIMING_INFEASIBLE`（“运动无解”），也不返回
审核通过。

## 6. 动态候选与扫掠覆盖原始值

以 S-A-S-O 复合场景为例：

| edge | phase | bound_m | sample_count | checked |
|---|---|---|---|---|
| 0 | motion_0000（S spin） | 0.642925 | 65 | 66 |
| 1 | motion_0001（A arc） | 5.040540 | 505 | 506 |
| 2 | motion_0002（S spin） | 0.153153 | 16 | 17 |
| 3 | motion_0003（O translation） | 1.560311 | 157 | 158 |

扫掠策略：

```text
max_corner_step_m=0.01
motion_padding_m=0.01
collision_margin_m=0.05
max_samples_per_segment=200000
```

动态候选覆盖率：`dynamic_candidate_count=803`；完整 `taus` 数组、
五次上界常数和每个块/车辆的 peaks/peak_taus/conservative_bounds/bound_ratios
均在 `uniform_quality_results.json` 中。

setup 转向曲线示例（A 弧 initial setup）：

```text
actual_duration_s=1.2124128118466029
minimum_duration_s=1.2124128118466029
steer_rate_peak=0.5175430224554335 <= 0.8
steer_accel_peak=1.5 <= 1.5
```

## 7. 回归与命令证据

`COMMANDS.md` 保存全部命令、环境变量、版本和退出码。`regression/exit_codes.tsv`
为机器可读退出码，`regression/logs/` 为完整日志。最终一次运行全部为 `0`：

```text
test_formation_collision   0  Ran 17 tests ... OK
test_formation_quality     0  Ran 22 tests ... OK
test_formation_hold        0  Ran 14 tests ... OK
test_formation_geometry    0  Ran 15 tests ... OK
test_formation_kinematics  0  Ran 28 tests ... OK
test_formation_retiming    0  Ran 14 tests ... OK
test_uniform_motion        0  Ran 13 tests ... OK
test_uniform_timing        0  Ran 5 tests ... OK
test_uniform_trajectory    0  Ran 8 tests ... OK
test_uniform_quality       0  Ran 8 tests ... OK
```

## 8. 明确限制：连续时间加速度证书

B01 的 `A_arc_nonzero_pivot` 与 `S_A_S_O_complex` 均返回 `FEASIBLE`，但：

```text
fixture: A_arc_nonzero_pivot, block_index=0
fixture: S_A_S_O_complex,     block_index=1
vehicle_id=3
constraint=acceleration
conservative_bound = 0.7893281464676393 m/s^2
limit              = 0.6 m/s^2
bound_ratio        = 1.315546910779399
candidate_peak     = 0.579658... m/s^2
candidate_peak_ratio = 0.9660973583815832
dynamic_continuous_certificate = false
```

含义：

- 803 点候选网格的加速度峰值未超限（0.966 < 1），这是 `FEASIBLE` 判定的
  动态数值依据；
- 但五次多项式加速度上界 `10/sqrt(3)*L/T^2` 在这个块/车辆上给出的保守界
  `0.7893 > 0.6`，因此**不能**宣称该连续区间的加速度已被严格证明；
- 这不是数值失败，也不是通过缩步长/margin 换取的成功；审核明确把
  `dynamic_continuous_certificate` 置为 `false` 并写入 `dynamic_limitations`；
- 若 Codex 要求严格连续证书，需要更密的候选集、区间分支定界或更紧的解析界，
  不能在 B 阶段用“候选点全过”冒充连续证明。

除此之外，所有合法场景的候选峰值与保守界均满足限值。

## 9. 复现与停止

- 复现审核与证据：按 `COMMANDS.md` 运行 `test_uniform_quality.py`；所有 fixture
  的模型在 `models/`，所有篡改前后输入在 `tampers/`，拒绝位置和指标在
  `uniform_quality_results.json`。
- 复现回归：按 `COMMANDS.md` 的 10 个套件循环运行；退出码见
  `regression/exit_codes.tsv`。
- 版本/哈希/命令清单：`evidence_manifest.json`。
- 未提交 Git；工作区保留 A/B 全部未提交修改与证据。

**B 阶段到此停止，等待 Codex 独立验收。**