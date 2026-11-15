# FR01 报告：统一模式解析原语

任务：FR01 / V0.2，日期：2026-11-12。状态：**实现与定向测试完成，等待 Codex 独立验收**。未开始 FR02，未恢复旧 T5/T6，未启动 ROS，未提交 Git。

## 1. 结论摘要

- 新增纯 Python 模块 `uniform_motion.py`，实现不可变 `UniformMotionEdge` 及 A 直线、A 固定枢轴圆弧、O 整体平移、全车 S 四个工厂。
- 新增 `tests/test_uniform_motion.py`，覆盖 P01–P12。最终结果 **12/12 PASS，退出码 0**。
- 每条边保存/校验：全局 mode、原语、几何控制、逐车初终展开航向、方向分支、曲率或斜移角、有符号几何进度速度、首末规范轮系，以及 `VehicleModeSegment` 数值载体。
- 所有派生字段在构造时由同一权威几何重新计算；篡改字段被 `INTERNAL_EDGE_FIELD_MISMATCH` 拒绝。独立审核函数 `validate_uniform_edge()` 再从原语定义重算，不信任边内派生字段。
- 未修改任何既有算法、测试、ROS 配置或消息文件；`git diff --name-only` 与 `git diff --cached --name-only` 均为空。

## 2. 工作区与冻结输入核对

| 项目 | 记录 |
|---|---|
| 实施目录 | `/home/wq/whut_thesis/expv3_multi_v02` |
| 分支 | `refactor/formation-v02` |
| 起始/结束 HEAD | `6227833011e2d92b711179bfdec979bb0b878b29` |
| 起始 tracked/index | 按 `READY.json`：clean |
| 结束 tracked/index | 无 tracked diff、无 staged diff；仅允许的准备输入与 FR01 新文件为 untracked |
| `git diff --check` | 退出码 0，无输出 |

冻结输入哈希按 `preparation/import_manifest.json` 逐一核对，全部一致；`import_manifest.json` 与 `BASELINE_PROVENANCE.md` 的 sha256 也与 `READY.json` 一致。完整输出见 `logs/provenance_check.log`。

| 文件 | sha256 | 结果 |
|---|---|---|
| `docs/FORMATION_REFACTOR_DECISIONS.md` | `8f2af8b8…f90a9` | 一致 |
| `docs/FORMATION_REFACTOR_IMPLEMENTATION_PLAN.md` | `88d190b9…6d93` | 一致 |
| `docs/CLAUDE_TASK_01.md` | `bea179cb…b07b` | 一致 |
| `docs/FORMATION_REFACTOR_REVIEW.md` | `9a4733e9…adc5` | 一致 |
| `docs/RESEARCH_HANDOFF_FOR_CHAT.md` | `1897831a…d8f2` | 一致 |
| `tools/configure_formation_view.py` | `0e910d4c…129b` | 一致 |

执行环境：`/usr/bin/python3`，Python 3.8.10，Linux x86_64；导入的包实际来自本工作区
`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/__init__.py`，未使用原仓库 devel。测试设置 `OPENBLAS_NUM_THREADS=1`。FR01 只使用 Python 标准库和既有包模块，无新增依赖。

## 3. 交付文件

| 文件 | 说明 | sha256 / 行数 |
|---|---|---|
| [uniform_motion.py](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py) | 新模块，2053 行 | `d75e4690…726a5` |
| [test_uniform_motion.py](tests/test_uniform_motion.py) | 定向测试，1482 行 | `a30b740b…4d4ba` |
| [primitive_results.json](evaluation/formation_refactor_v02/FR01/primitive_results.json) | 最终 P01–P12 机器可读结果 | `e9f2a558…48b7` |
| [primitive_results_round1.json](evaluation/formation_refactor_v02/FR01/primitive_results_round1.json) | 首轮 6 个失败的真实记录 | `5a9b67ce…8fa9` |
| [REPORT.md](evaluation/formation_refactor_v02/FR01/REPORT.md) | 本报告 | 运行时生成 |
| [new_files_manifest.json](evaluation/formation_refactor_v02/FR01/new_files_manifest.json) | 新文件哈希/字节/行数清单 | 运行时生成 |

完整新增文件 diff：

- [diff/uniform_motion.py.diff](evaluation/formation_refactor_v02/FR01/diff/uniform_motion.py.diff)
- [diff/test_uniform_motion.py.diff](evaluation/formation_refactor_v02/FR01/diff/test_uniform_motion.py.diff)

## 4. API

### 4.1 权威边类型

`UniformMotionEdge` 是 frozen dataclass，主要字段：

```text
mode, primitive, model, control,
start_headings, end_headings, directions,
curvatures, oblique_angles, signed_progress_speeds,
wheel_angles_start, wheel_factors_start,
wheel_angles_end, wheel_factors_end,
segments, limits, allow_reverse
```

- `mode ∈ {"A","O","S"}`；`primitive ∈ {A_STRAIGHT, A_ARC, O_TRANSLATION, S_SPIN}`；二者必须配对。
- 逐车字段均为按 `vehicle_id` 排序的 `(id, value)` 元组；轮系字段为 `(id, (4 个值))`。
- `segments` 为 `VehicleModeSegment`，只作数值载体；不调用旧混合模式逻辑替代新合同。
- `__post_init__` 验证输入并重新推导全部派生字段；任何内部字段被篡改即拒绝，不返回半有效对象。

主要查询方法：

```text
start_qf, end_qf, progress_interval, progress_unit,
position_derivative_unit, angle_derivative_unit, pivot_in_cargo_frame
state_at(progress) -> UniformMotionState(progress, qf, centers, headings)
derivative_at(progress) -> UniformMotionDerivative(
    progress, position_unit, angle_unit, dqf, dp, dtheta)
wheel_configurations(at_end=False)
wheel_configuration(vehicle_id, at_end=False)
wheel_configuration_at(progress)
angular_delta_from_zero_curvature_a()
as_dict()
```

`state_at` 与 `derivative_at` 严格接受 `[0, L]` 内的有限进度；越界/非有限值抛出 `PROGRESS_OUT_OF_RANGE` 或 `NONFINITE_VALUE`，不吸附、不裁剪。

### 4.2 工厂与辅助函数

```python
a_straight_edge(model, start_qf, headings, displacement, directions,
                limits=None, allow_reverse=False)
a_arc_edge(model, start_qf, headings, delta_psi, pivot, directions,
           limits=None, allow_reverse=False)
o_translation_edge(model, start_qf, headings, displacement, directions,
                   oblique_angles, limits=None, allow_reverse=False)
s_spin_edge(model, start_qf, headings, delta_thetas, limits=None)

a_straight_required_headings(model, displacement, directions)
a_arc_required_headings(model, start_qf, delta_psi, pivot, directions,
                        limits=None)
validate_uniform_edge(edge, sample_fractions=(0.0, 0.137, 0.413, 0.769, 1.0))
formation_model_from_contract(contract)
```

工厂接收的是**实际初始车头**，不会为了构造 A/O 边自动旋转或瞬移车体；不满足合同即拒绝。`a_arc_required_headings` 只生成候选，不修改状态。

错误类型 `UniformMotionError(FormationValidationError)` 带 `reason`、`detail`、可选 `vehicle_id`、`required_headings`。稳定错误码按用途分组：

- 输入/结构：`INVALID_MODEL`、`INVALID_POSE`、`INVALID_VEHICLE_ID_SET`、`INVALID_MODE`、`INVALID_PRIMITIVE`、`MODE_PRIMITIVE_MISMATCH`、`INTERNAL_EDGE_FIELD_MISMATCH`、`UNSORTED_PAIR_FIELD`、`NONFINITE_VALUE`。
- A：`ZERO_TRANSLATION`、`ZERO_ROTATION`、`A_HEADING_REQUIRES_S_ALIGNMENT`、`DIRECTION_MISMATCH`、`REVERSE_NOT_ALLOWED`、`PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED`、`INSUFFICIENT_TURN_RADIUS`、`CURVATURE_LIMIT`。
- O：`OBLIQUE_ANGLE_LIMIT`、`OBLIQUE_VECTOR_MISMATCH`。
- S：`ZERO_SPIN_NO_EDGE`、`SPIN_INCREMENT_OUT_OF_RANGE`、`SPIN_LENGTH_MISMATCH`。
- 轮系/进度/审核：`WHEEL_ANGLE_LIMIT`、`PROGRESS_OUT_OF_RANGE`、`EDGE_VALIDATION_FAILED`。

## 5. 公式与合同映射

### 5.1 参数域与单位

| 原语 | λ 含义 | L | `progress_unit` | 位置导数单位 | 角度导数单位 |
|---|---|---|---|---|---|
| A 直线/倒车 | 路径距离进度 | 共同平移距离 | `m` | `m/m` | `rad/m` |
| A 固定枢轴圆弧 | 编队旋转角进度 | `abs(Δpsi_F)` | `rad` | `m/rad` | `rad/rad` |
| O 平移 | 平移距离进度 | 共同平移距离 | `m` | `m/m` | `rad/m` |
| S 自旋 | 最大车体旋转角进度 | `max_i abs(Δtheta_i)` | `rad` | `m/rad` | `rad/rad` |

`λ` 始终是几何进度，不是时间。`as_dict()` 明确写出 `real_time_domain=false`、`time_seconds=null`。`dqf/dλ`、`dp_i/dλ`、`dtheta_i/dλ` 不是 m/s 或 rad/s；本任务不实现 `λ(t)`、真实速度、角速度或轮系设置时长。

### 5.2 A 直线

`λ∈[0,L]`，`L=||displacement||`，`d=displacement/L`：

```text
p_F(λ) = p_F0 + λ d
p_i(λ) = p_F(λ) + R(psi_0) r_i
psi(λ) = psi_0 ; theta_i(λ) = theta_i0
dp_i/dλ = d ; dtheta_i/dλ = 0 ; kappa_i = 0
v_lambda_i = sigma_i
```

每车车头必须与 `d` 平行或反平行，`sigma_i=+1` 表示车头沿 `d`，`-1` 表示反向；不一致返回 `A_HEADING_REQUIRES_S_ALIGNMENT`，不会自动转车体。

### 5.3 A 固定枢轴圆弧

`λ∈[0,|Δpsi|]`，`a=sign(Δpsi)`，`C=p_F0+R(psi_0)c`：

```text
psi(λ) = psi_0 + a λ
p_F(λ) = C - R(psi(λ)) c
p_i(λ) = p_F(λ) + R(psi(λ)) r_i
rho_i = ||r_i - c||
dp_i/dλ = a R(psi) J (r_i - c)
dtheta_i/dλ = a
v_lambda_i = sigma_i rho_i          # m/rad, 不是 m/s
kappa_i = a / (sigma_i rho_i)
```

- 初始车头沿实际导数（`sigma=+1`）或反向（`sigma=-1`），`theta_i(λ)=theta_i0+aλ`。
- 所有 `rho_i` 必须 `>= limits.r_min_ack`（默认 0.810 m）；枢轴落在车辆中心返回 `PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED`，半径不足返回 `INSUFFICIENT_TURN_RADIUS`。货物半径不能替代逐车检查。
- 首末轮系由实际曲率/方向计算，并检查四轮角不超过 `steer_angle_max`。

### 5.4 O 平移

`λ∈[0,L]`，`L=||displacement||`，`psi` 和所有 `theta_i` 固定；逐车 `phi_i` 常数，`|phi_i|<=π/2`：

```text
d = sigma_i [cos(theta_i + phi_i), sin(theta_i + phi_i)]
dp_i/dλ = d ; dtheta_i/dλ = 0
v_lambda_i = sigma_i                 # |d| = 1
```

检查完整二维向量，而不是仅检查沿车头的纵向投影；因此 ±90° 纯横移合法。每车 `phi_i`/方向可不同，但全局 mode 全为 O。首版不支持 O 编队转动。

`as_dict()` 逐车额外输出：

- `body_longitudinal_projection = sigma_i cos(phi_i)`；
- `body_lateral_projection = sigma_i sin(phi_i)`；
- `world_unit_displacement = d`。

这两个投影明确区分 V0.2 的 `center_speed_mps = sigma_i ||dp_i/dt||`（本边中即 `sigma_i`）与旧载体 `speed_signed`（身体纵向投影，纯横移时接近 0）。FR02 不得把旧 `speed_signed` 直接当作 V0.2 行驶速度。

### 5.5 S 自旋

`λ∈[0,L]`，`L=max_i|Δtheta_i|`，`q_F`/中心固定：

```text
theta_i(λ) = theta_i0 + Δtheta_i λ / L
dtheta_i/dλ = Δtheta_i / L
dp_i/dλ = 0
```

- 全部车辆 mode=S，包括 `Δtheta_i=0` 的车辆。
- `Δtheta_i∈[-π,π]`；恰为 ±π 时保留符号，不静默反转。
- 所有 `Δtheta_i=0` 返回 `ZERO_SPIN_NO_EDGE`，不伪造正 L、不自环。
- 包括零角车在内的每辆车，首末轮系都必须等于 `wheel_configuration(Action(SPIN,0), limits)`。由轮位置 `(x_w,y_w)=(±wheelbase/2,±track/2)`、单位车体自旋方向 `(-y_w,x_w)`、角 `atan2(x_w,-y_w)` 折入 `[-π/2,π/2]` 独立计算；本模型四轮角为 `(-0.9357695914, +0.9357695914, +0.9357695914, -0.9357695914)`，因子为 `0.3068110167513546`。
- 相对 A 零曲率配置的四轮角差绝对值均约 `0.9357695914 rad`；该值不是设置时长。

### 5.6 轮系

A/O/S 的规范轮系统一由既有 `retiming.wheel_configuration(Action(legacy_mode, 0.0, control), limits)` 生成：

- A：`control=kappa`；
- O：`control=phi`；
- S：`control=0`。

工厂和独立审核均检查轮角是否超过 `steer_angle_max`。轮系因子是单位车身运动率的几何因子，不是实际轮速；零角 S 车的实际轮速为 0，但其规范 S 轮角/因子不退化为 A。

## 6. 复用与不兼容点

### 6.1 已复用

- `formation.FormationModel / FormationPose / VehicleOffset`：构型、刚性中心映射。
- `formation_geometry.FormationPathControl`：平移/圆弧/旋转几何积分和 `qf_at`、`dqf_dlambda`。
- `formation_kinematics.VehicleModeSegment`：仅作逐车数值载体。
- `retiming.MotionLimits`、`retiming.wheel_configuration`：限值与规范轮系。
- `transport_contract.TransportContract`：`formation_model_from_contract()` 便利适配器。

### 6.2 已确认的 FR02 适配点（未在本项实现）

1. `formation_spin.make_spin_block()` 在车辆 `Δtheta=0` 时写入 `mode=ACKERMANN`（[formation_spin.py:117](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_spin.py#L117)）。新合同要求该车仍为 S；FR02 转换器必须显式处理。
2. `formation_geometry.FormationBlock` 在 `geometry_type=BODY_SPIN` 且某车 `delta≈0` 时，要求该段为 ACKERMANN，否则报 `BODY_SPIN_HOLD_MODE`（[formation_geometry.py:419](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_geometry.py#L419)）。
3. `formation_kinematics.audit_mode_assignment()` 拒绝 `Δtheta≈0` 的 SPIN 段，报 `ZERO_BODY_SPIN`（[formation_kinematics.py:1288](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py#L1288)）。
4. 旧 `_sample_motion()` 使用 `values["speed_signed"]`（[formation_retiming.py:1082](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_retiming.py#L1082)），该字段是身体纵向投影；纯 O 横移接近 0。FR02 必须按 V0.2 合同显式重算 `center_speed_mps = sigma_i ||dot(p_i)||`，不能静默沿用旧字段。
5. FR01 不把新边转换为 `FormationBlock`；上述零角 S 冲突留给 FR02 的适配/共享验证修正。

## 7. 验证方法与阈值

- 独立解析期望：测试用合同公式和固定 offsets 自行计算 `q_F(λ)`、中心、航向、半径、曲率、方向和轮系，不调用待测构造器证明自身正确。
- 独立数值差分：A 弧在非端点进度 `0.31, 0.73`、步长 `1e-5` 时用中心差分核对沿车头投影与侧向残差；A/O 平移在 `0.43L` 处核对 `dp_i/dλ` 和完整向量投影。P09 额外核对每车 `sigma_i cos(phi_i)` 与 `sigma_i sin(phi_i)`。
- `validate_uniform_edge()` 采样 5 个状态进度（含端点）并在另外 3 个非端点进度核对导数；`sample_count=8`。它独立重算 A 弧曲率、A/O 向量方程、S 长度/范围、轮角和因子。
- 构造后重推导：`UniformMotionEdge.__post_init__` 重算全部派生字段并与存储值比较，容差 `1e-9`；篡改即拒绝。
- 阈值：端点几何/航向 `<=1e-7 m/rad`；刚性 `<=1e-6 m`；归一化模式方程 `<=1e-6`；独立有限差分 `<=1e-4`；轮角不超过 `steer_angle_max + 1e-9`；曲率不超过合同范围。测试不使用比合同更宽的容差。

## 8. P01–P12 结果

完整输入、每个原语字段、半径/曲率/方向/首末轮角与因子、误差和拒绝原因见 [primitive_results.json](evaluation/formation_refactor_v02/FR01/primitive_results.json)。摘要：

| 用例 | 场景 | 关键独立期望/结果 | 状态 |
|---|---|---|---|
| P01 | A 直线，车头 `(0,π,0)`，方向 `(+,-,+)`，允许倒车 | 全 A；`L=3 m`；曲率 0；有符号进度速度 `(1,-1,1)`；端点 qf/航向误差 0；刚性 `2.22e-16 m`；模式残差 `1.22e-16` | PASS |
| P02 | `c=0`，正转，车头 `(π/2,π,0)` | `L=π/2 rad`；半径均 1；曲率均 `+1`；端点/航向误差 0；刚性 `2.22e-16 m`；最大轮角 0.2932503 rad | PASS |
| P03 | 保持 P02 车头，顺时针 `Δpsi=-π/2`，方向均 `-1`，允许倒车 | 全 A；半径 1；曲率均 **+1**（`a=-1`、`sigma=-1`）；有符号速度 `-1`；未重现旧“标正向但实际反向”错误 | PASS |
| P04 | `c=(0,2)`，正转 | 半径 `(√5,1,3)`；曲率 `(1/√5,1,1/3)`；相对车头 `(atan2(1,2),0,0)`；端点 qf/航向误差 0；刚性 `4.44e-16 m` | PASS |
| P05 | P04 从 `psi_0=0.37` 转 `π/2` | 世界货物/中心圆弧正确；`L=π/2 rad`；半径 `(√5,1,3)`；端点/航向误差 0；刚性 `6.66e-16 m` | PASS |
| P06 | `c=(0,1.5)` | 三车半径 `(1.8027756, 0.5, 1.8027756) m`；car2 的 0.5 m < 0.810 m；helper 与工厂均返回 `INSUFFICIENT_TURN_RADIUS`，`vehicle_id=2`；货物半径 1.5 m 未替代逐车检查 | PASS（预期拒绝） |
| P07 | 枢轴恰在 car1 中心 | 返回 `PIVOT_AT_VEHICLE_CENTER_UNSUPPORTED`，`vehicle_id=1`；无除零/NaN；明示“首版退化分支不支持”，不声称一般物理不可能 | PASS（预期拒绝） |
| P08 | `c=0` 的 A 弧输入车头全 0 | 返回 `A_HEADING_REQUIRES_S_ALIGNMENT`，`vehicle_id=1`；记录所需切向 `(π/2,π,0)`；不自动改车头 | PASS（预期拒绝） |
| P09 | O 横移/斜移：±90° 与不同初始 `theta`，3 条边 | 全 O；`psi/theta` 固定；逐车 `phi` 正确；±90° 时纵向投影约 `6.1e-17`、侧向 `±1`；完整向量独立核对误差 `<=1e-12`；三条边独立审核的模式残差 `<=2.22e-16`、刚性 `<=4.44e-16 m`、导数误差 0 | PASS |
| P10 | S 增量 `(π/2,0,-π/4)` | 全 S；`L=π/2 rad`；qf/中心固定；car2 `Δtheta=0` 仍 S；三车首末轮角均为规范 S，因子均 0.3068110167513546；相对 A 零曲率最大角差 0.9357695914 rad；篡改 A 轮角/因子被构造器拒绝，独立审核拒绝 | PASS |
| P11 | S 全零增量、明确 ±π | 全零返回 `ZERO_SPIN_NO_EDGE`，不伪造 L；`+π/-π` 分别保留为车1 `+π`、车2 `-π`，车3 零角；`Δ` 超出 `[-π,π]` 返回 `SPIN_INCREMENT_OUT_OF_RANGE` | PASS |
| P12 | 反转 direction/曲率、非法 mode/ids/非有限数/无效 phi/越界进度等 | 27 个负例子全部明确拒绝（含构造器与独立审核）：`DIRECTION_MISMATCH`、`A_HEADING_REQUIRES_S_ALIGNMENT`、`INTERNAL_EDGE_FIELD_MISMATCH`、`INVALID_MODE`、`MODE_PRIMITIVE_MISMATCH`、`INVALID_VEHICLE_ID_SET`、`NONFINITE_VALUE`、`OBLIQUE_ANGLE_LIMIT`、`OBLIQUE_VECTOR_MISMATCH`、`REVERSE_NOT_ALLOWED`、`PROGRESS_OUT_OF_RANGE`、`EDGE_VALIDATION_FAILED` 等 | PASS |

P10 的独立轮系证据：轮位置使用 `(±wheelbase/2, ±track/2)`，单位自旋速度方向 `(-y_w,x_w)`，角 `atan2(x_w,-y_w)` 折入 `[-π/2,π/2]`；结果与 `wheel_configuration(Action(SPIN,0), limits)` 一致。P10 的构造器负例与独立审核负例均记录在 `primitive_results.json` 的 `subcases`。

## 9. 命令、退出码与日志

在 `/home/wq/whut_thesis/expv3_multi_v02` 下执行：

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_uniform_motion.py' -v
```

结果：`Ran 12 tests in 0.019s`，`OK`，退出码 0。完整 stdout/stderr：[logs/fr01_final_test.log](evaluation/formation_refactor_v02/FR01/logs/fr01_final_test.log)。

```bash
git diff --check
```

结果：无输出，退出码 0。

相关既有定向测试（只读运行，未修改其文件）：

| 命令（`python3 -m unittest discover -s tests -p <pattern> -v`） | 结果 | 日志 |
|---|---|---|
| `test_formation_model.py` | 26 tests，OK，退出码 0 | `logs/legacy_formation_model.log` |
| `test_formation_geometry.py` | 15 tests，OK，退出码 0 | `logs/legacy_formation_geometry.log` |
| `test_formation_kinematics.py` | 28 tests，OK，退出码 0 | `logs/legacy_formation_kinematics.log` |
| `test_continuous_retiming.py` | 18 tests，1 error，退出码 1 | `logs/legacy_continuous_retiming.log` |

`test_continuous_retiming.py` 的既有错误是 `test_short_translation_has_finite_acceleration_and_zero_end_speed` 中的 `KeyError: 'dynamic_peaks'`（[test_continuous_retiming.py:271](tests/test_continuous_retiming.py#L271)）。FR01 未修改该测试或相关实现，未为获得 PASS 改旧日志/旧期望；该错误与 FR01 原语无调用关系。

## 10. 失败记录与修复

首轮（`primitive_results_round1.json`，日志 `logs/fr01_round1_failures.log`）为 6 失败：

1. **P02–P05**：测试用中心差分 `dtheta/projection` 核对曲率，容差 `1e-12` 过紧；实际数值差约 `1.6e-11`～`9.2e-12`。将测试侧比较容差改为 `1e-6`（符合合同“独立有限差分 `<=1e-4`”），未放宽审核器的合同阈值。
2. **P10**：测试期望方向错误地把三车都写成 `+1`；车3 `Δtheta=-π/4` 的正确方向是 `-1`。修正期望值。
3. **P12**：独立审核未拒绝 direction 篡改（`auditor_rejects_direction_tamper` 未抛错）。修复 `validate_uniform_edge()`，把已计算的 qf/航向/刚性/模式方程/导数/曲率指标显式与阈值比较，篡改方向现在返回 `EDGE_VALIDATION_FAILED`。这是实现缺陷修复，不是放宽阈值。

修复后第二轮 12/12 PASS（日志 `logs/fr01_round2_pre_serialization.log`）。随后在 `as_dict()` 增加半径/身体投影/世界单位位移的显式序列化字段，并在 P06 增加逐车半径记录、在 P09 增加独立审核指标；未改变实现公式或审核阈值，最终重跑 12/12 PASS。

## 11. 验证范围与未覆盖项

已验证：

- A 直线/圆弧、O 平移、S 自旋的解析几何、端点/航向、刚性、一阶导数、方向分支、半径/曲率/斜移角、轮角限值。
- 构造器与独立审核的双重验证；零角 S 的规范轮系及篡改拒绝。
- 进度域/单位、越界/非有限拒绝、全零 S 不生成边、±π 符号保留。

未覆盖（按任务边界）：

- 未实现 `λ(t)`、时间参数化、真实车速/角速度、轮系设置时长、动态峰值和碰撞审核。
- 未转换为旧 `FormationBlock`，未接入 ROS/RViz，未构建 catkin，未运行全量 427 项回归或优化器。
- 未实现一般 O 转动、退化枢轴驻车分支、跨模式时间链；这些属于 FR02 或后续任务。
- 未修改任何既有算法/测试/ROS/config 文件；未创建/清理 worktree；未访问原仓库；未提交 Git。

## 12. 停止声明

FR01 实现、定向测试、结果记录和本报告已完成。按任务停止条件：**不继续 FR02、不恢复旧 T5/T6、不启动 ROS、不提交 Git**。等待 Codex 独立验收及下一项任务。