# FR02-A 实现与验收报告

- 任务：`docs/CLAUDE_TASK_02.md` 的 **FR02-A：公共时间与解析相位**
- 工作区：`/home/wq/whut_thesis/expv3_multi_v02`
- 分支/HEAD：`refactor/formation-v02` / `6227833011e2d92b711179bfdec979bb0b878b29`
- 状态：**A 已完成，停止并交 Codex 验收**。未进入 FR02-B，未提交 Git，未创建 worktree，未访问原仓库。
- 证书边界：`GEOMETRY_ONLY_NO_S9_NO_COLLISION`。本报告不宣称碰撞通过、S9 独立审核通过或可发布。

## 1. 基线与输入核验

初始 tracked 工作区干净；`baseline/git_status_initial.txt`、`baseline/head_initial.txt` 与任务指定 HEAD 一致。初始输入文件哈希见 `baseline/initial_hashes.sha256`，逐项核验见 `baseline/input_verification.json`。

FR01 两个只读文件在启动和结束时哈希一致：

| 文件 | SHA256 | 结果 |
|---|---|---|
| `P/uniform_motion.py` | `1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2` | 未变 |
| `tests/test_uniform_motion.py` | `7c25e85b14d8c11ecb460daf76eead60d82270dd9ad74b0d2531f490b98f360a` | 未变 |

`P/` = `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

### 1.1 关于 `preparation/protected_source_files.json` 的 geometry 哈希

该清单记录的 `formation_geometry.py` 哈希为
`5f58e9bcf65b8fafc5b4c744ed50cb7ae1c2868d461edc0d9a2dff6cebd6b778`，
而本 worktree 修改前副本为
`3688aaf366e0904698615da385e88a2ae3994beaceded33cd3c01e64615a5bfb`。

已核对：

- `BASELINE_PROVENANCE.md` 明确说明 `protected_source_files.json` 是**原工作区 T5 三文件保护指纹**，供 Codex 审计原仓库使用；
- 本 worktree 的修改前副本与 HEAD blob 的 SHA256 完全一致（`git show HEAD:... | sha256sum` 亦为 `3688...`）；
- 因此这不是本工作区的 HEAD/输入哈希不符，不需要停止；未覆盖、未 reset/stash/clean 任何内容。

## 2. 文件变更

### 2.1 新增

| 文件 | 作用 |
|---|---|
| `P/uniform_timing.py` | FR01→S8 分析视图、公共运动时长复用、初始/相邻 setup 调度、导数适配、解码恢复 |
| `P/uniform_trajectory.py` | 权威解析时间模型、任意时刻求值、版本化模型、V0.2 采样投影 |
| `tests/test_uniform_timing.py` | A01–A05 定向测试与机器结果 |
| `tests/test_uniform_trajectory.py` | A06–A10 定向测试与机器结果 |

### 2.2 定点修改的共享文件

| 文件 | 修改点 | 必要性 |
|---|---|---|
| `P/formation_geometry.py` | `FormationBlock.__post_init__` 零角 segment 接受 `SPIN`（原先只接受 `ACKERMANN`） | 统一 S 边中某车 `delta_theta=0` 时，旧审计视图原会拒绝；中心固定、控制 0、行程 0 和非零 S 方向检查均保留 |
| `P/formation_kinematics.py` | `audit_mode_assignment` 不再一律拒绝零角 S；仅非零旋转检查方向 | 零角 S 是统一 S 边的合法静止成员，不是 A 保持；旧 A-hold、固定中心、S 控制 0/行程 0 检查不变 |
| `P/formation_retiming.py` | `_context_for` 不再对零角 S 抛 `body_spin_zero_yaw`；仅非零 `theta_rate` 检查方向 | 使 `_analyze_block` 能分析合法统一 S 边；`_wheel_data` 仍按 `segment.mode` 给出规范 S 轮系与因子，零角车实际轮速为 0 |

未修改 `P/formation_quality.py` 的第四个零角 S 拒绝点；完整 S9 适配属于 FR02-B。

### 2.3 未修改的只读文件

`P/uniform_motion.py`、`tests/test_uniform_motion.py`、`P/retiming.py`、`P/mode_switch_timing.py`、`P/formation_hold.py`、`P/formation_collision.py`、`P/formation_quality.py`、消息/ROS/控制/动力学相关文件均未修改。

## 3. 实现摘要

### 3.1 `uniform_timing.py`

- `edge_to_block(edge, block_index)` 直接由 FR01 `control`/`segments` 构造 `FormationBlock` 审计视图，不重新选模式、方向、车头或路线。
- `validate_uniform_edge_sequence()` 逐边调用 FR01 `validate_uniform_edge()`，核对：
  - vehicle IDs 与 offsets；
  - 相邻 `q_F` 位置接续 `<=1e-7 m`、原始 `psi` 与逐车展开车头接续 `<=1e-7 rad`（不随意加减 2π）；
  - FR01 `MotionLimits` 与 `FormationTimingLimits` 的 `wheelbase/track/r_min_ack/steer_angle_max` 一致；
  - `min_mode_dwell_s > 0`。
- `schedule_uniform_edges()` 对每条边调用现有 S8 `_analyze_block()`，直接复用其：
  - 401 个归一化时刻与加速度极值候选；
  - 全部车辆、八类动态约束的公共最大时长；
  - `1.005` 安全系数、`min_mode_dwell_s` 拉伸、`max_block_duration_s=120 s` 失败诊断。
  - 不调用 `retime_formation_plan`，不复制第二套峰值求解器。
- 设置调度：
  - 初始状态为全车静止 A、零曲率规范轮系；
  - 逐车逐轮比较真实规范首末轮角，带符号差用 `_wrap(end-start)`；
  - 每车调用 `mode_switch_event(delta, motion_limits)`；
  - 公共时长 `max(event_duration)`，公共开销 `max(setup_s)`，公共转向时间 `duration-overhead`；
  - 用同步五次曲线从各轮起角到终角，并重新计算公共实际 `rate_peak=1.875*max|delta|/steering_duration`、`accel_peak=(10/sqrt(3))*max|delta|/steering_duration^2`，核验不超过 `steer_rate_max/steer_accel_max`；
  - 同模式同配置且最大轮角差 `<=1e-9` 时不创建伪相位；
  - 相位左闭右开，setup 期间标签为旧模式。
- `motion_derivatives()` 通过现有 S8 `_formation_derivatives`/`_vehicle_derivatives` 返回 `q'`、`q''`、`q'''`、逐车一至三阶几何导数、`theta_rate`、方向、控制、规范轮角与因子；先拒绝非有限和越界 progress。
- `restore_motion_timing()` 只重建 block/context 视图，**不调用 `_analyze_block`**，用于解码时保留输入相位时长。
- `timing_motion_limits()` 把 S8 restricted `MotionLimits` 的构造集中在唯一适配边界。

### 3.2 `uniform_trajectory.py`

- `UniformTimedPlan.from_edges()`：接受人工指定边序列，或显式起终 `q_F`/车头完全一致的零运动请求。
- `state_at(t, side=...)`：
  - `[0,T_end]` 内任意有限时间精确解析；
  - 默认左闭右开，边界选择右相位；
  - `side='left'` 仅供审核查询边界处旧模式状态；
  - `t=T_end` 定义为最后一运动相位的终点；
  - 越界、NaN、非法边界侧明确拒绝。
- 运动相位：
  - `tau=(t-t0)/T`，`s=10τ³-15τ⁴+6τ⁵`，`λ=L s`；
  - `p_dot=p' λ_dot`、`p_ddot=p'' λ_dot²+p' λ_ddot`、`p_dddot=p''' λ_dot³+3p''λ_dotλ_ddot+p'λ_dddot`；
  - `theta_dot=theta' λ_dot`、`theta_ddot=theta' λ_ddot`、`theta_dddot=theta' λ_dddot`；
  - 端点 `tau` 显式取 0/1，避免浮点舍入使进度略超 L；
  - 中心/q_F 来自 FR01 `state_at(progress)`，导数来自 S8 闭式核；不独立插值三车中心，也不把非零枢轴弧变成弦插值。
- 设置相位：
  - 位姿、车头固定；线/角速度与加速度为 0；
  - 公共开销后执行同步五次轮角曲线；
  - 轮角、轮角速度、轮角加速度逐轮解析；
  - 三车同时从 `from_mode` 切到 `to_mode`。
- 投影：
  - `sample(times=None)` 默认含 0、所有相位边界、`T_end`；
  - 所有投影值来自同一 `state_at()`；
  - 输出显式 `uniform_v02_signed_center_speed.v1` 版本、`uniform_timed_plan.v1` 模型版本和 `GEOMETRY_ONLY_NO_S9_NO_COLLISION` 标记；
  - 使用旧 `FormationTimedPoint/FormationTimedTrajectory` 仅作数值载体，不作为第二条权威轨迹。
- 序列化：
  - 保存版本、`plan_id`、`formation_id`、offsets、形状、limits、初始 `q_F`/车头/模式/轮系、每条边工厂参数与冗余字段、相位时长、设置需求；
  - 解码通过 FR01 工厂重建边，并逐字段比对冗余值；
  - 拒绝未知版本、非有限值、缺失/重复 ID、非法枚举、重复 JSON key、冗余冲突、时间缝隙/重叠、非正相位时长、非法进度/时间；
  - 解码不重新定时；正时长缩短探测被原样恢复（`retimed=false`），时长可行性留给 FR02-B。
  - `model_sha256()` 明确为 **12 位有效数字的规范化语义摘要**。工厂重建会使冗余诊断字段出现约 1 ULP 差异，因此摘要用于身份辅助，不能替代 `from_model_dict` 的逐字段实值比较。

### 3.3 零角 S

`(π/2, 0, -π/4)` 场景中：

- 三车均为 S；
- car2 首末轮系均为规范 S 四轮角，未改回 A；
- car2 实际轮速为 0，但其设置需求 `max_wheel_angle_delta_rad=0.9357695914045828` 未被省略；
- 公共运动由 car1 `yaw_rate` 驱动，`T=3.6999616603801657 s`；
- 初始 A→S 公共 setup `T=2.3432099798544908 s`，公共 `rate_peak=0.8`、`accel_peak=1.1231761533`。

## 4. A01–A10 结果

机器结果：`timing_results.json`、`trajectory_results.json`；完整测试输出：`logs/new_uniform_timing.log`、`logs/new_uniform_trajectory.log`。

| ID | 判定 | 关键证据 |
|---|---|---|
| A01 | PASS | A 零曲率首段仅 1 个 motion、无伪 setup；O 横移/S 首段有真实 initial_setup，车头固定，三车同步。O 初始 setup `T=3.831553890925538 s`；S 初始 setup `T=2.3432099798544908 s` |
| A02 | PASS | `(π/2,0,-π/4)` 全 S；car2 首末规范 S 轮角、设置 delta `0.9357695914 rad`；公共运动 `T=3.6999616604 s`，driver=`yaw_rate` car1，utilization `0.9950248756` |
| A03 | PASS | 手工 S→A弧→S→O，4 边 4 motion 4 setup；`q_F`/展开车头逐边精确接续；总时长 `23.1490481180 s`；相位见 `artifacts/phase_table.json` |
| A04 | PASS | O→O 同模式轮角变化产生 setup（`T=3.831553890925538 s`）；第二次同模式同配置不产生 setup；端点线/角速度、进度加速度均在 `1e-9` 内 |
| A05 | PASS | O `+90°/-90°/反向` 的 `center_speed_mps=±0.5137149695`，`longitudinal_speed_mps=0.0`，`lateral_speed_mps=±0.5137149695`；V0.2 速度不是旧纵向投影 |
| A06 | PASS | 五次链式法则与独立中心差分最大误差：直线 `2.30e-8`、A 弧 `4.73e-8`、S `3.72e-8`；四组场景规范模式残差均为 `0.0`；分别由 speed/acceleration/yaw/wheel-speed 驱动的手工场景均成立 |
| A07 | PASS | setup 内部 12 个非均匀时刻位置误差 `0.0`、车头误差 `0.0`、实际轮角/角速度/角加速度逐轮吻合；采样最大轮速 `0.7998179146 <= 0.8`，最大轮加速度 `1.1085521712 <= 1.5`；初始 setup 与 S motion 分项计时，`6.0431716402=2.3432099799+3.6999616604` |
| A08 | PASS | 模型往返 126 个状态对比、105 个投影采样、12 个非零枢轴弧内部样本；最大状态往返误差 `6.66e-16`，投影误差 `6.66e-16`，弧 `q_F` 与独立闭式误差 `0.0`，刚性误差 `4.44e-16 m`；弧与货物首末弦最大偏离 `0.0876156570 m`，证明未按弦插值；规范化模型摘要相等 |
| A09 | PASS | 真零运动 `duration=0`、单采样、静止 A；位置相同车头不同按真实 S 处理（`T=4.3532099798544905 s`）；空序列位置/车头未满足、非空序列未达指定终点均明确拒绝 |
| A10 | PASS | 24 个故障注入子例全部按稳定 reason 拒绝：接续跳变、IDs/offsets/几何限值不一致、非法版本/非有限值、缺失/重复 ID、重复 JSON key、非法原语、冗余值冲突、非法 progress/time、零/负相位时长、时间缝隙/重叠；1000 m 平移超 120 s 返回 `MOTION_DURATION_LIMIT`，constraint=`speed`，vehicle_id=1 |

动态峰值：所有运动相位逐车逐约束 `peak <= limit*(1+1e-3)`；本组正例最大 utilization 为 `0.9950248756`。端点线/角速度与进度加速度绝对值均在 `1e-9` 内（A04/A06 测试检查）。

## 5. 回归

| 命令 | 退出码 | 结果 | 日志 |
|---|---:|---|---|
| `test_uniform_timing.py -v` | 0 | 5 tests OK | `logs/new_uniform_timing.log` |
| `test_uniform_trajectory.py -v` | 0 | 5 tests OK | `logs/new_uniform_trajectory.log` |
| `test_uniform_motion.py -v`（FR01，13 项） | 0 | 13 tests OK | `logs/fr01_uniform_motion.log` |
| `test_formation_geometry.py -v` | 0 | 15 tests OK | `logs/legacy_test_formation_geometry.log` |
| `test_formation_kinematics.py -v` | 0 | 28 tests OK | `logs/legacy_test_formation_kinematics.log` |
| `test_formation_retiming.py -v` | 0 | 14 tests OK | `logs/legacy_test_formation_retiming.log` |
| `test_formation_hold.py -v` | 0 | 14 tests OK | `logs/legacy_test_formation_hold.log` |

所有命令在 `OPENBLAS_NUM_THREADS=1` 下运行，导入路径来自本 worktree（见 `final/environment.json`、测试结果中的 `import_paths`）。FR01 结果写入 `fr01_primitive_results.json`，未覆盖历史 FR01 结果。

未运行、未宣称修复旧 `test_continuous_retiming.py:271 dynamic_peaks`；该已知旧套件失败不属于本轮通过项。

## 6. 数值与证据

- 中心/`q_F` 几何与接续误差：`<=1e-7 m` 校验；A08 实际最大投影/往返误差 `6.66e-16`。
- 展开车头接续/往返：`<=1e-7 rad` 校验；A08 实际最大 `6.66e-16`。
- 刚性：A08 最大 `4.44e-16 m`。
- 归一化模式残差：A06 直线/弧/S/O 场景与 A02 规范 S 轮系逐轮比较，最大 `0.0`（阈值 `1e-6`）。
- 独立有限差分：A06 最大 `4.73e-8`（阈值 `1e-4`，离端点/零速）。
- 时轴接续：`<=1e-9 s`；相位边界按左闭右开逐点检查。
- 端点线/角速度及进度加速度：`<=1e-9`（A04/A06）。
- setup 公共曲线：实际峰值 `rate=0.8 <= 0.8`，`accel=1.1231761533 <= 1.5`；A07 的 setup 段车体位置/车头误差均为 `0.0`。
- 非零 pivot 弧：`arc_internal_samples.json` 保存 12 个内部点的 `q_F`、独立闭式 `q_F`、误差和相对首末弦偏离；最大偏离 `0.0876156570 m`。

## 7. 证据索引

- `REPORT.md`：本报告。
- `baseline/`：修改前共享文件副本、初始哈希、Git 状态/HEAD/diff 统计、导入路径、输入核验。
- `final/shared/`：修改后共享文件与新增源码副本；`final/final_hashes.sha256`、`final/absolute_hashes.sha256`、`final/hashes_before_after.json`。
- `final/environment.json`、`final/branch_final.txt`、`final/head_final.txt`、`final/git_status_final.txt`、`final/git_diff_stat_final.txt`。
- `diff/`：三个共享文件的 `git diff`，四个新增源码/测试文件的 `/dev/null` diff；`.exit_code` 记录（新文件 diff 退出码 1 为预期，表示有差异）。
- `timing_results.json`、`trajectory_results.json`、`fr01_primitive_results.json`。
- `artifacts/model.json`、`artifacts/projection.json`、`artifacts/initial_state_limits.json`、`artifacts/phase_table.json`、`artifacts/setup_demands.json`、`artifacts/motion_peaks.json`、`artifacts/arc_internal_samples.json`、`artifacts/roundtrip_errors.json`、`artifacts/negative_cases.json`。
- `logs/`：正式测试完整输出与退出码；`logs/development_*` 为开发阶段首轮失败/修复记录，保留未隐藏。
- `command_manifest.json`、`evidence_manifest.json`。

## 8. 未验证项与 FR02-B 边界

- 未实现独立 S9 动力学审核，未运行 `evaluate_formation_trajectory` 的新模型入口。
- 未实现真实解析碰撞扫掠，未调用 `FormationCollisionChecker`，不宣称 OBB/地图/车辆间净空通过。
- 投影是数值载体，不是发布或安全证书；没有 `ACCEPTED`/可发布标记。
- 解码不重新定时，也不独立判定时长可行性；FR02-B 负责独立审核与篡改拒绝。
- `formation_quality.py` 的零角 S 第四个拒绝点仍未适配，属于 FR02-B。
- 未启动 ROS/RViz，未改消息定义、控制/动力学、FR03 或 FR05。
- 未创建 Git 提交；未操作原仓库 `/home/wq/whut_thesis/expv3_multi`；未创建 worktree。

## 9. 停止

FR02-A 的公共时间、初始/setup/motion 相位、任意时刻解析求值、版本化往返与采样投影已完成并通过上述定向测试及回归。**完成 A 即停止，交 Codex 独立验收；FR02-B 未获授权，不继续。**