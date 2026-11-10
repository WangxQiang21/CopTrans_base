# S8 阶段审查报告：全车同步时间参数化

## 结论

**S8 完成：新增 ROS 无关的 `formation_retiming.py`，在 S6 `FormationContinuousPlan` 上建立每个 block 一条单调进度 `λ(t)`，全部车辆在同一 `times_from_start` 网格上采样。** 每个 block 采用起终速度为零的五次进度规律；车辆中心由 `q_F(λ)` 与固定 offset 刚性派生，车体航向继续取 S5/S6 的 `θ_i(λ)`，不默认 `θ_i=ψ_F`。速度、加速度、jerk、yaw 率/角加速度/角 jerk、轮速与横向加速度由链式法则逐车计算，阶段时长取全车、全部限制的最大需求；A/O 的 signed speed 保留方向符号，Spin 中心速度为 0。block 边界若发生模式切换或轮角变化，插入整编队静止事件，其时长由现有 `mode_switch_timing` 的受限策略计算，并审核每车模式驻留时间。静态几何或时长预算不可行时返回 `FORMATION_RETIMING_INFEASIBLE`，并给出最紧约束、车辆与 block。13 项 S8 定向测试全部通过；S5 15/15、S6 15/15、S7 9/9；完整 Python 回归 276 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误，没有新增失败。本阶段输出仅为 `GEOMETRY_ONLY`，没有碰撞证书、独立时间审核、ROS 发布或 `FEASIBLE`；没有修改 C++、单车 `ContinuousPlan`/`retime_continuous`、控制器、NMPC、执行器或任何消息。**

## 任务编号

S8：全车统一 retiming。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L779-L820)，阶段表见同文档 [S8 行](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L403)。

## Git 检查点

- S7 已完成提交：`48a31d7c705ca0eee682e5e83ac98b8924794714`（`feat/formation-nlp` 分支 HEAD）
- S8 分支：`feat/formation-time`（从 S7 提交创建）
- S8 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| S6 连续计划 API | 已核对 `FormationContinuousPlan`/`FormationBlock`/`FormationPathControl` 的 `qf_at`、`body_heading_at`、`delta_psi`、`parameter_length`、`vehicle_mode_segments` 与 `start/goal`、`goal_heading_free` 语义 |
| 单车 retiming | 只参考 `continuous_retiming.py` 的 `quintic`、`_s3`、`_angles_and_factors`、`_wheel_geometry_arrays`、`_piece_peaks` 思路；未导入或修改单车 `ContinuousPlan`/`retime_continuous` 的结构、状态或公式 |
| 模式切换时序 | 复用 `mode_switch_timing.mode_switch_event` 与 `retiming.wheel_configuration`，构造独立的 `MotionLimits(translation_speed_policy="restricted")`；未修改两个既有模块 |
| 动态限制来源 | 使用 `FormationTimingLimits` 自身的字段（默认继承单车限制量级），不把编队参考点速度当作全车速度上限 |
| A/O/S 语义 | `_context_for` 静态检查平移不改车体航向、旋转 `θ_i` 率等于 `dpsi/L`、A 曲率、O 斜角、S 控制为零；逐段重新检查轮角上限 |
| `goal_heading_free` | 只影响终端车体航向审核；不释放中心、`ψ_F` 或中间航向，与 S7 契约一致 |
| 既有消息/节点 | 未修改任何 `.msg`、ROS 节点、launch、YAML 或 C++；S8 不发布话题 |
| 碰撞/地图 | S8 不建立碰撞证书、不检查地图版本；沿用 S6 的 `map_version_checked=false` 限制，`collision_certificate=false` |

## 修改文件

新增：

1. [formation_retiming.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_retiming.py)：`FormationTimingLimits`、`FormationTimedPoint`、`FormationTimingReport`、`FormationTimedTrajectory`、`FormationTimingAudit`、`FormationTimingFailure`、`FormationRetimingResult`、`retime_formation_plan`、`audit_formation_timing`。
2. [test_formation_retiming.py](../../tests/test_formation_retiming.py)：13 项 ROS 无关测试。
3. [logs/formation_retiming_tests.txt](logs/formation_retiming_tests.txt)
4. [logs/formation_geometry_tests.txt](logs/formation_geometry_tests.txt)
5. [logs/formation_kinematics_tests.txt](logs/formation_kinematics_tests.txt)
6. [logs/formation_optimizer_tests.txt](logs/formation_optimizer_tests.txt)
7. [logs/unit_tests.txt](logs/unit_tests.txt)
8. [logs/py_compile.txt](logs/py_compile.txt)
9. [logs/git_diff_check.txt](logs/git_diff_check.txt)
10. [S8_REPORT.md](S8_REPORT.md)

未修改任何既有文件；没有 C++ 或消息变更，因此本阶段不需要 catkin 构建。

## 公共时钟与进度规律

### 一条公共时钟

每个 block 有局部参数 `λ∈[0,L]`，其中 `L` 为 S6 block 的 `parameter_length`。对第 `k` 个运动 phase：

```text
t = t0_k + T_k * τ,           τ∈[0,1]
τ = 3τ² - 2τ³                （quintic 位置规律）
λ(t) = L * q(τ)
λ̇ = L * q'(τ) / T_k
λ̈ = L * q''(τ) / T_k²
λ⃛ = L * q'''(τ) / T_k³
q'(τ)  = 30τ² - 60τ³ + 30τ⁴
q''(τ) = 60τ - 180τ² + 120τ³
q'''(τ)= 60 - 360τ + 360τ²
```

`q(0)=q(1)=q'(0)=q'(1)=0`，因此每个运动 phase 起终速度为零。block 之间若有静止事件，先以 `Δt_event` 停住，再开始下一个运动 phase；`t0_k` 为各 phase 的绝对起点。所有车辆共享同一 `t0_k` 和 `T_k`，不存在每车独立时钟。

### 链式法则

编队参考位姿 `q_F(λ) = (x_F, y_F, ψ_F)` 与车辆 offset `r_i` 给出中心：

```text
p_i(λ) = q_F(λ) + R(ψ_F(λ)) r_i
p_i'   = q_F' + ψ_F' R J r_i
p_i''  = q_F'' - (ψ_F')² R r_i
p_i''' = q_F''' - (ψ_F')³ R J r_i
```

`_formation_derivatives` 是这些导数的精确解析式：

- 平移 block：`q_F'` 为常向量，`q_F''=q_F'''=0`，`ψ_F'=0`；
- 纯旋转 block：设 `s = dψ_F/L`，`q_F' = -s R J pivot`、`q_F'' = s² R pivot`、`q_F''' = s³ R J pivot`，`ψ_F'=s`。

`_vehicle_derivatives` 把 `p_i', p_i'', p_i'''` 写成 `q_F` 导数、`s` 与 offset 的闭式组合，没有对 `p_i'` 除零，也没有用中心差分代替解析导数。开发阶段用真实 S6 旋转 block 与 `h=1e-3` 的中心差分核对：解析速度、加速度、jerk 与差分结果在 `1e-5` 量级内一致。

### 采样与共享网格

`_build_timed` 先写 `t=0` 的起点样本，再按 phase 顺序拼接：

- 运动 phase 用 `_sample_motion` 按 `ceil(T_k/dt)` 个等分 τ 采样，包含 τ=1 端点；
- 静止 phase 用 `_sample_stationary` 按 `ceil(Δt_event/dt)` 采样，样本 `stationary=true`、全部动态量为 0；
- 每个采样点先按车辆 context 展开，再按“每车点数”分组，时间轴只取第一辆车的样本时间；所有车辆共享同一 `times_from_start`；
- 编队位姿由每个样本自己的 `block_index` 与 `station` 重新调用 `block.qf_at(station)` 得到（静止样本取 `qf_at(0)`），不从车辆中心反推。

`FormationTimedTrajectory.__post_init__` 强制：

- `times_from_start[0]=0`；
- 时间严格递增；
- 每车轨迹点数等于时间网格点数，且每个点时间与该网格一致；
- 只允许 `certification=GEOMETRY_ONLY`。

## 全车动态包络

### 逐车逐段指标

`_quantities` 对每个车辆 context 计算：

| 指标 | 计算 |
|---|---|
| 线速度 | `‖p_i˙‖` |
| 线加速度 | `‖p_i¨‖` |
| jerk | `‖p_i⃛‖` |
| yaw rate | `∣θ_i' λ̇∣` |
| yaw acceleration | `∣θ_i' λ̈∣` |
| yaw jerk | `∣θ_i' λ⃛∣` |
| 轮速 | A/O：`max(wheel_factor) * 线速度`；S：`max(wheel_factor) * yaw rate` |
| 横向加速度 | 仅 A：`∣κ∣ * 线速度²` |

`wheel_factor` 与轮角来自 `wheel_configuration(Action(mode, 0, control), MotionLimits)`；轮角上限在 `_context_for` 中单独静态检查。A/O 的 `center_speed_mps` 由 `direction` 乘上速度大小；Spin 强制为 0。反向 O 因此得到负 signed speed，而不是把方向藏到控制角里。

### 共同速度包络

`_peak_table` 在 `_TAUS` 上采样每个指标的“单位时长峰值”。`_TAUS` 为 401 个等分 τ，加上 quintic 的关键点 `0.5`、`0.5±√3/6`。对每个指标按量纲反解所需时长：

```text
order 1（速度、yaw rate、轮速）: T ≥ peak / limit
order 2（加速度、yaw 加速度、横向加速度）: T ≥ sqrt(peak / limit)
order 3（jerk、yaw jerk）: T ≥ cbrt(peak / limit)
```

取全部车辆、全部 8 个指标的最大需求，再乘安全系数 `1.005`，得到 `dynamic_duration`；`duration = max(dynamic_duration, min_mode_dwell_s)`。这就是所有车辆共同的速度/加速度/jerk 包络：不存在“只按编队参考点限速”的路径。若 `dynamic_duration > max_block_duration_s`，返回静态失败结果，最紧约束由最大 `required_duration` 的贡献项决定。

### 时长驱动与峰值报告

每个 block 报告：

- `duration_s`、`dynamic_duration_s`；
- `duration_driver`：实际决定时长的约束、车辆、峰值、限制、利用率与 τ；
- `limiting_constraints`：利用率不低于驱动项 95% 的前三项；
- `vehicle_peaks`：每车 8 个动态峰值，加上 steer rate/accel 与 wheel angle。

`mode_dwell` 驱动时 `duration_driver` 明确记录 `constraint="mode_dwell"`、`vehicle_id=None`，不伪装成某个车辆的速度约束。

## 静止事件、换向与驻留

### 边界事件

对相邻 block，`_boundary_event` 逐车比较前后 wheel angle（按最小角差）和 mode：

- 若任一车辆 mode 改变，或轮角差超过 `1e-9`，插入整编队静止 phase；
- 事件时长取所有车辆 `mode_switch_event(delta, motion_limits)` 的 `setup_s + steering_s` 最大值；
- `kind` 为 `mode_switch`（有模式改变）或 `wheel_reconfiguration`（仅轮角改变）；
- 事件记录起终轮角、每车 `setup_s`/`steering_s`/转向 profile/steer rate 峰值/steer accel 峰值、最大轮角差、下一 block 的位姿与每车航向。

静止样本全部动态量为 0，`mode`/`mode_control` 取事件后 block 的 segment 值。当前结构不改变 `q_F` 和 `θ_i`，因为 S6 计划审核保证 block 边界位姿与航向连续；因此静止期间车体航向保持常量，事件元数据记录每车航向起点。文档所说“必要时的每车航向插值”在当前 block 结构中没有被触发；若未来引入 block 内独立航向动作，必须新增显式的航向插值，不能只改事件时长。

### 方向换向

`_direction_changes` 在 block 边界比较 `segment.direction`；只要某车方向变化，就记录：

```text
kind=direction_reversal
vehicle_id, from_mode/to_mode, from_direction/to_direction
time_s = 静止事件起点（或运动 block 起点）
at_rest = true
```

因为换向只发生在 block 边界，且边界若有轮角/模式变化则先行静止，所以换向样本速度为 0。审核会用二分查找定位该时刻的样本并断言 `|speed|≤1e-9`。

### 模式驻留

`_mode_intervals` 按 phase 顺序合并同一车辆连续相同 mode 的区间；静止 phase 计入下一个 block 的 mode。审核检查：

- 每车区间从 0 覆盖到 `duration`；
- 区间连续；
- 每个区间时长不小于 `min_mode_dwell_s`。

`_analyze_block` 对每个 block 也取 `max(dynamic_duration, min_mode_dwell_s)`，这是每个 block 的保守驻留下限；两者共同保证模式驻留。

## S8 自审核设计

`audit_formation_timing(plan, timed, limits)` 是 S8 自洽审核，不是 S9 独立 evaluator。它消费 S8 自己的采样网格，不生成新网格，不做碰撞证书，不发布。检查项：

1. **元数据**：plan id、formation id、offsets、时间网格长度/起点/严格递增、报告时长；
2. **共享时间**：每车点数与时间网格一致、每点时间一致；
3. **编队位姿**：每个样本的 `FormationPose` 等于对应 block 的 `qf_at(station)`（静止样本取 `qf_at(0)`），位置误差与航向误差均不超过 `1e-7`；
4. **刚性中心**：`FormationModel.centers(pose)` 重新计算每车中心，与样本中心比较；误差记为 `max_rigidity_error_m`，超过 `1e-7` 报 `RIGIDITY_VIOLATION`；
5. **公共状态**：同一索引下所有车辆的 `block_index`、`station`、`stationary` 必须一致；
6. **车体航向**：等于 `block.body_heading_at(vehicle_id, station)`（静止时取 station 0），不是 `ψ_F`；
7. **模式/控制**：等于该 block 的 segment mode 与 control；
8. **静止样本**：速度、yaw rate、加速度、jerk、轮速、横向加速度全部为 0；
9. **Spin**：`center_speed_mps=0`；
10. **方向**：非 Spin 的非零速度符号必须等于 `direction`；
11. **采样动态上限**：速度、加速度、jerk、yaw rate、轮速、横向加速度逐点检查；
12. **端点**：起点/终点的编队位姿、每车中心、起点航向、非 `goal_heading_free` 的终点航向；
13. **驻留**：模式区间覆盖、连续、每段不少于 `min_mode_dwell_s`；
14. **换向**：换向时刻存在样本且速度为 0；
15. **报告峰值**：8 个动态峰值、steer rate/accel、wheel angle 均不超过对应限制；
16. **报告时长**：与时间网格末尾一致。

审核失败时 `retime_formation_plan` 返回 `REJECTED`/`TIMING_AUDIT_FAILED`，`timed=None`，不输出任何“已审核”的时间轨迹。审核成功也只返回 `TIMED_GEOMETRY_ONLY`，detail 明确写明“S9 independent audit and collision certification are still required”。

## 本阶段自检发现的实现问题

以下问题在写测试前的数值核查中发现并修正，属于 S8 代码本身的缺陷，不是既有模块回归：

1. **`_build_timed` 的 phase 分组初稿不完整**：最初用一个占位的 `_point_vehicle_id` shim 和重复的分组循环拼接每车样本，时间轴来源不清晰。现改为按 `count = len(new_points)//len(contexts)` 分组，时间只取第一辆车的样本时间，编队位姿由每个样本自己的 `station` 重新计算。
2. **`_mode_intervals` 含无效的初始化循环**：最初有一个不做任何事的循环；现改为先按 analyzed block 初始化每车区间，再按 phase 合并连续同 mode 段。
3. **`_wheel_data` 存在未使用参数**：签名中的 `limits` 从未使用，且与 `motion_limits` 混淆；已删除，只保留 `motion_limits`。
4. **限制自洽检查缺失**：`max_block_duration_s < min_mode_dwell_s` 会使 `max(dynamic_duration, min_mode_dwell_s)` 超过预算却不一定按预算失败；已在 `FormationTimingLimits.__post_init__` 增加 `INVALID_TIMING_LIMIT max_block_duration_s must be >= min_mode_dwell_s`。

上述修正不改变 S5/S6/S7 的审核契约，只修正 S8 的构造与限制校验。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `python3 -m unittest tests.test_formation_retiming -v` | 13/13 通过；0.216 s | 0 |
| `python3 -m unittest tests.test_formation_geometry -v` | 15/15 通过 | 0 |
| `python3 -m unittest tests.test_formation_kinematics -v` | 15/15 通过 | 0 |
| `python3 -m unittest tests.test_formation_optimizer -v` | 9/9 通过 | 0 |
| `python3 -m unittest discover -s tests -p 'test_*.py'` | 276 项，275 通过、1 个既有错误；8.453 s | 1 |
| `python3 -m py_compile formation_retiming.py tests/test_formation_retiming.py` | 编译成功，无输出 | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 276 tests in 8.453s
FAILED (errors=1)
```

276 = S7 的 263 项 + S8 新增 13 项。没有新增跳过、弱化或删除任何既有测试；未修改 `test_continuous_retiming.py` 或相关重定时模块。

## 定向测试与验收对应

| 文档验收要求 | 实际测试 | 结果 |
|---|---|---|
| 同一时刻所有 `p_i(t)` 满足刚性变换 | `test_translation_common_grid_and_rigid_centers` 对每个样本用 `FormationModel.centers(pose)` 重算中心并断言 `1e-9` 内相等；审核 `max_rigidity_error_m` 在成功场景为 0 | 通过 |
| 一条公共单调进度、共享 `times_from_start` | 同一测试断言时间起点为 0、严格递增、每车点数/点时间与网格一致；chain 测试断言三种 phase 拼接后仍共享网格 | 通过 |
| 全车速度包络而非编队参考点限速 | `test_speed_limit_sets_duration`：`v_max=0.2` 时 `duration_driver.constraint="speed"`、峰值不超过 0.2；直线两车同时受约束 | 通过 |
| 加速度/jerk 上限 | `test_jerk_limit_sets_duration`：`jerk_max=0.5` 时由 jerk 驱动，报告峰值 ≤0.5，速度 ≤0.8 | 通过 |
| Spin 中心不动、yaw dynamics 合法 | `test_pure_rotation_pivot_and_nonpivot`：pivot 中心范围 <1e-9、速度为 0、yaw rate >0.1；follower 为 A、速度 >0.1；mode intervals 分别只含 Spin 与 A | 通过 |
| A/O signed speed 与反向 | `test_chain_holds_dwell_and_reverse`：断言 follower 出现负速度、方向换向记录 `from_direction=1/to_direction=-1/at_rest=true`；车辆 0 不出现负速度 | 通过 |
| 模式切换全体停住 | 同一测试断言两个静止事件的 kind 依次为 `mode_switch`、`wheel_reconfiguration`，phase 序列为 motion/stationary/motion/stationary/motion | 通过 |
| 模式驻留 | 同一测试断言每个 mode interval 与每个静止 phase 不小于默认 `min_mode_dwell_s`；`test_dwell_limit_extends_block_duration` 用 5 s 下限断言时长被拉长且 `duration_driver.constraint="mode_dwell"` | 通过 |
| 失败报告最紧约束/车辆/时段 | `test_static_steering_angle_failure`：斜角 0.1 rad 限制下报 `steer_angle_limit`、vehicle 0、block 0；`test_duration_budget_failure`：`v_max=0.05` 时要求 75 s，超过 5 s 预算，报 `speed`、vehicle 0、block 0 | 通过 |
| 失败不得降低安全限制 | 两个失败测试都断言 `timed is None`、`audit is None`，失败码为 `FORMATION_RETIMING_INFEASIBLE`；没有用更长时长之外的“放宽限制”路径 | 通过 |
| 审核拒绝被篡改的中心/航向 | `test_audit_rejects_tampered_center` 报 `RIGIDITY_VIOLATION`/vehicle 1；`test_audit_rejects_tampered_heading` 报 `HEADING_MISMATCH`/vehicle 0 | 通过 |
| 时间轴不得倒退 | `test_constructor_rejects_nonmonotonic_times` 断言构造函数抛 `TIMES_NOT_STRICTLY_INCREASING` | 通过 |
| 非法输入 fail-closed | `test_invalid_inputs_fail_closed`：错误 plan/limits、`dt=0`、0/负限制、`steer_angle_max>π`、负驻留、`max_block_duration<min_mode_dwell` 全部抛 `FormationValidationError` | 通过 |
| 不输出 `FEASIBLE`/可执行声明 | `test_status_never_claims_feasible_or_executable` 断言结果与轨迹的 `executable`/`trajectory_published`/`feasible`/`independent_timing_audit`/`collision_certificate` 全为 false，certification 为 `GEOMETRY_ONLY`，detail 明确要求 S9 | 通过 |
| 单车版本仍运行 | 完整回归无新增失败；既有 `dynamic_peaks` 错误保留 | 通过 |

### 程序化场景结果摘录

| 场景 | 结果 |
|---|---|
| 两车直线平移 2 m，默认限制 | `TIMED_GEOMETRY_ONLY`；审核 6918 项通过；时长 4.7109375 s；237 个共享采样；driver=`speed`/vehicle 0/peak 0.7960199/limit 0.8/util 0.99502；采样最大加速度 0.5202983、jerk 1.1477815 |
| 纯旋转 0.5 rad，offset (0,0)、(1.2,0) | `TIMED_GEOMETRY_ONLY`；审核通过；时长 2.6338451 s；133 个采样；driver=`jerk`/vehicle 1/peak 1.9702975/limit 2.0；pivot 中心范围 0；pivot yaw rate 0.3559435、轮速 0.1092074、轮角 0.9357696；follower 速度 0.4271322、加速度 0.5002755、横向 0.1520349、轮速 0.4997088 |
| 旋转 0.3 + 两段反向 O 平移链 | `TIMED_GEOMETRY_ONLY`；审核通过；时长 18.4455569 s；927 个采样；静止事件 3.7063 s（mode_switch，最大轮角差 1.5173723）与 6.1469 s（wheel_reconfiguration，最大轮角差 2.5586791）；vehicle 1 在 9.1132325 s 发生 O 方向 +1→−1；模式区间 vehicle 0/1 分别为 S 2.221 s + O 16.224 s / A 2.221 s + O 16.224 s；全车最大速度 0.6145366 |
| 斜向 O 平移，`steer_angle_max=0.1` | `FORMATION_RETIMING_INFEASIBLE`；constraint=`steer_angle_limit`、vehicle 0、block 0；detail=`wheel angle 0.785398163397 rad exceeds 0.1` |
| `v_max=0.05`，`max_block_duration_s=5` | `FORMATION_RETIMING_INFEASIBLE`；constraint=`speed`、vehicle 0、block 0；detail=`speed requires 75.000000 s, exceeds max_block_duration_s=5.000000` |

## 未执行项目与边界

1. **每个 block 只有一个五次进度 phase**：没有 block 内多段速度 profile、没有时间最优或 jerk-limited S 曲线搜索；时长由单一 quintic 的峰值缩放，保守但非最优。
2. **没有 block 内换模/转轮动作**：模式切换与轮角变化只发生在 block 边界；block 内恒定控制。若未来允许 block 内换模，必须在运动 profile 中插入额外的静止事件与轮角插值。
3. **静止期间不插值轮角/航向**：事件元数据记录起终轮角、每车 `setup_s`/`steering_s`/profile 与航向，但 `FormationTimedPoint` 不带轮角字段，静止样本保持下一 block 的航向常量；S6 保证边界航向连续，因此当前不需要航向插值。这不是完整的“静止动作轨迹”，只是事件时长与动态峰值审核。
4. **自审核不是独立 evaluator**：`audit_formation_timing` 消费 S8 自己生成的网格与报告，只检查自洽和采样上限；不重新生成轨迹、不做碰撞扫掠、不证明所有连续时刻。S9 必须独立建立 evaluator。
5. **峰值来自 401 点采样**：`_TAUS` 包含 401 个等分点与 quintic 关键 τ，安全系数 1.005；这是数值包络，不是区间算术证明。若未来控制/几何更复杂，应改为解析极值或更密的区间界。
6. **yaw 加速度/jerk、steer rate/accel 的审核依赖报告峰值**：采样点检查覆盖速度、加速度、jerk、yaw rate、轮速、横向加速度；yaw 加速度/jerk、steer rate/accel 通过 `vehicle_peaks` 检查，静止事件的转向峰值来自 `mode_switch_event`，没有在时间轨迹中重新采样轮角。
7. **静态失败范围有限**：只检查平移不改航向、旋转航向率、A 曲率、O 斜角、S 零控制、轮角上限和时长预算；不检查障碍物、地图版本、通信延迟、执行器带宽或真实轮系约束。
8. **没有全局时间预算/多候选选择**：`max_block_duration_s` 是单 block 预算；没有编队总时长上限、没有在多个 S6/S7 候选之间按 `T_end` 选择。S9 负责选解。
9. **没有碰撞证书/发布**：`collision_certificate=false`、`trajectory_published=false`、`executable=false`、`independent_timing_audit=false`、`feasible=false`；S8 不写 ROS 话题、不调用控制器。
10. **没有地图/动态障碍/回放/RViz**：S8 是纯 Python 库；S10 才做回放、launch、smoke 与实验。
11. **失败只报告首个/最紧项**：与 S5/S6/S7 一致，未聚合所有车辆/所有 block 的失败。
12. **`allow_reverse` 未在 S8 重新推导**：反向合法性由 S5/S6 审核；S8 只按 segment.direction 生成 signed speed 并审核符号一致，不重新证明请求的 `allow_reverse` 语义。
13. **模式驻留按 block 下限保守处理**：连续同 mode 的跨 block 区间在审核时合并，但每个 block 仍单独取 `min_mode_dwell_s`，因此可能比真实驻留需求更保守。
14. **没有时间参数化的连续性证明**：block 之间速度为零，加速度在静止事件前后为 0，五次 profile 端点 `q''(0)=q''(1)=0`；但如果未来加入非零端点速度或 block 内多 phase，必须重新建立 C² 连续性审核。

## 安全性审计

- **中心唯一来源**：每个样本的中心由 `q_F(λ)` 与固定 offset 重新计算，`FormationTimedPoint` 不携带独立中心曲线；审核再用 `FormationModel.centers(pose)` 反算并逐点比对。
- **不默认 `θ_i=ψ_F`**：航向取 `block.body_heading_at(vehicle_id, station)`，旋转 block 的 `θ_i` 率必须等于 `dpsi/L`，平移 block 不允许改航向；审核逐点重算。
- **A/O/S 不混淆**：A 检查曲率和轮角，O 检查斜角，S 检查控制为零且中心速度为 0；A/O 的轮速按轮系因子映射，S 的轮速按 yaw rate 映射。
- **反向不伪装**：signed speed 由 `direction` 乘速度大小；审核在非零速度样本上检查符号等于 `direction`，换向事件必须在静止样本上。
- **全停换模**：任何模式或轮角变化都插入整编队静止 phase，静止样本所有动态量为 0；时长取全车最大换模需求，并审核每车模式驻留。
- **共同包络**：阶段时长取全部车辆、全部 8 个动态指标的最大需求；不存在只按编队参考点限速的分支。
- **失败闭合**：非法类型/限制/`dt` 抛 `FormationValidationError`；静态不可行、超时长预算返回 `FORMATION_RETIMING_INFEASIBLE` 且 `timed=None`；自审核失败返回 `REJECTED`/`TIMING_AUDIT_FAILED` 且 `timed=None`。
- **不冒充可执行**：结果最多 `TIMED_GEOMETRY_ONLY`；结果、轨迹与报告恒有 `executable=false`、`trajectory_published=false`、`feasible=false`、`independent_timing_audit=false`、`collision_certificate=false`；没有 `FEASIBLE` 状态。
- **不污染单车**：未修改 `core.py`、`continuous_geometry.py`、`continuous_retiming.py`、`retiming.py`、`mode_switch_timing.py`、单车 `.msg`、C++ 或 ROS 节点；只复用公开 helper。
- **测试可复现**：13 项测试的 offsets、heading、primitive、限制与篡改索引全部写死；成功场景先断言种子 S6 审核通过，再断言 S8 审核通过；失败场景断言具体约束/车辆/block。

## 存在风险

1. **采样包络可能低估峰值**：401 点加关键 τ 覆盖了当前 quintic 的主要极值，但不是连续区间证明；组合向量范数的极值可能落在采样点之间。安全系数 1.005 只是裕量，不是证书。
2. **单一 quintic 过于保守**：所有 block 起终速度为零，长链会累积大量静止/低速时间；这不影响安全，但会降低效率，S10 实验应记录时长。
3. **block 边界假设依赖 S6**：静止事件不改变 `q_F`/`θ_i`，依赖 S6 审核保证边界连续；若 S6 或未来计划允许边界航向跳变，S8 会产生不可执行的头向跳变而不自知。
4. **轮角在静止期间没有轨迹**：事件只给时长与峰值，真实执行器需要自己完成轮角插值；如果控制器要求逐点轮角，S8 输出不足以直接驱动。
5. **yaw accel/jerk 的采样审核不完整**：它们只通过报告峰值检查，报告峰值由 `_analyze_block` 计算；若 `_analyze_block` 与 `_sample_motion` 的公式不一致，审核可能漏掉。当前两者共享 `_quantities`，但 S9 应独立重算。
6. **失败路径覆盖有限**：`REJECTED`/`TIMING_AUDIT_FAILED` 由 `retime_formation_plan` 内部审核产生，测试通过直接调用 `audit_formation_timing` 覆盖篡改，但没有在公开 `retime_formation_plan` 上强制触发该分支；S9 的独立审核必须覆盖。
7. **时长预算不是总时长**：`max_block_duration_s` 只限制单 block；多 block 链的总时长可能很长，调用方需自行限制。
8. **没有碰撞/地图/动态障碍**：S8 完全不处理障碍物，S9 必须独立扫掠并签发证书。
9. **模式驻留保守**：每 block 至少 `min_mode_dwell_s`，可能把短 block 拉长；真实驻留需求若更细，需要 S9/S10 调整。
10. **没有多候选**：S8 只 retime 传入的一个 S6 plan；S9 需要按公共 `T_end` 在候选中选择，并重新审核。

## 完成判据与 S9 前置条件

- 公共时钟与单调进度：**已实现**；每个 block 一条 `λ(t)`，所有车辆共享 `times_from_start`。
- 链式法则速度/加速度/jerk：**已实现**；解析导数，开发阶段用有限差分核对。
- 全车共同动态包络：**已实现**；8 个指标逐车取最大需求，安全系数与时长预算显式记录。
- A/O signed speed、Spin 零中心速度：**已实现并测试**。
- 模式切换/轮角变化全停与静止事件：**已实现并测试**；事件时长、轮角差、峰值与每车明细可序列化。
- 模式驻留与方向换向审核：**已实现并测试**。
- 共享时间网格与刚性中心：**已实现并测试**；审核逐点比对 `FormationModel.centers`。
- 失败报告最紧约束/车辆/block：**已实现并测试**。
- 只输出 `GEOMETRY_ONLY`、无 `FEASIBLE`：**已由代码与状态测试确认**。
- 单车回归：**无新增失败；既有 `dynamic_peaks` 错误保留**。
- S9 前置条件：S9 必须独立重建 request/trajectory/rigidity/collision/kinematics/timing/publication-sample 审核，不得复用 S8 的自审核结论；只有全部独立检查通过才能输出 `FEASIBLE`；必须给出结构化失败码、事务式发布（失败不发布半成品），并按公共 `T_end` 在候选中选择；S8 的 `TIMED_GEOMETRY_ONLY` 不得被当作可执行或已发布轨迹。