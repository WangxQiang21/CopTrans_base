# Step 5 证据：版本化生产 v3 数值证书（保留 v2 重放）

日期：2026-11-19。本目录是 `work_flow/TASK01_Codex执行方案.md` Step 5 的
证据。未 commit/push；本文件不宣称 Codex PASS 或 TASK 01 验收。

## 1. 实现边界

- `bounds.py`：新增 `NUMERICS_V2="virtual_rigid_a.numerics.g1.v2"`、
  `NUMERICS_V3="virtual_rigid_a.numerics.g1.v3"`、
  `SUPPORTED_NUMERICS_VERSIONS=frozenset((V2, V3))`；
  `DEFAULT_NUMERICS_VERSION=V3`。`POSITION_BUDGET_M=1e-7`、
  `ANGLE_BUDGET_RAD=1e-9`、`POSITION_ROUNDOFF_GUARD_M=1e-10`、
  `MAX_PANELS=200000` 未改（tests 与 diff 可核对）。
- `geometry.py`：`C2CurvatureProfile.from_knots(..., numerics_version=None)` 与
  `VirtualAPath(..., numerics_version=None)` 只接受支持集合；`None` 时：
  可序列化 profile 必须声明受支持版本，诊断型（`serialization_ready=False`）
  profile 才默认为当前版本；**不存在“未知版本自动用最新”**。
  显式版本与 profile 声明冲突时报 `NUMERICS_VERSION_MISMATCH`。
  `VirtualAPath.hold(..., numerics_version=None)` 默认 v3；v2 hold 预算保持
  原字节形状（无 `numerics_version` 键），v3 hold 预算带版本键。
- `geometry.py::_plan_integration_v2()`：保留原实现与字节语义；仅把共享的
  Simpson 网格构建提取为 `_build_grids(records)`（算术逐项不变）。
- `geometry.py::_plan_integration_v3()`：
  - 每个 domain segment 由 `interval_bounds(s0, s1)` 取唯一正长度 Hermite
    piece（相邻 piece 在共享节点产生零长度记录被过滤），
    `integration_bounds.panel_m4_bounds(c0,c1,c2,c3,h,n)` 求**每个完整
    Simpson panel** 的 outward M4；诊断型 analytic profile 没有
    `coefficients`，退化为 `analytic_interval_envelope_per_panel`（每 panel
    用区间包络）。两种方法都记录在 `m4_method` 与每段 `m4_method`。
  - `integration_bounds.segment_budget(h, n, panel_m4s)`：完整 panel 的
    `H^5*M4/2880` 之和 + **一个任意前缀 local panel**（取该段最大 panel M4，
    宽度一个完整 panel）。`state_at_s()` 的任意查询因此被同一证书覆盖。
  - `integration_bounds.production_evaluation_guard(records, psi_start,
    start_magnitude_m)`：逐项对应生产 cached-grid 路径的
    Horner `psi_at`、缓存 `sin/cos`、Simpson weighted sum、piece position
    prefix 和每 piece 一个 local panel；每段单位
    `12*n_j + 64 + 16*Psi`，加
    `16*(segments+1)*EPSILON*L*Psi` 与
    `2*(segments+1)*EPSILON*(|start|+L)`；所有乘/加均 `_next_up` outward。
    它**不**复用 audit 的 `evaluation_guard`（audit 仍从 raw knots 独立重建）。
  - 证书公式（单坐标米）：
    `E_quad_coord = Σ_seg (Σ_full H^5*M4/2880 + one local panel)`
    `E_eval_coord = production float64/libm/prefix guard`
    `E_angle_coord = S * E_psi`
    `E_position = sqrt(2) * (E_quad_coord + E_eval_coord + E_angle_coord)`
    所有求和/乘积 outward；`total_budget_m` 是 Euclidean 界，
    `per_coordinate_*` 是各分项。
  - 细分：仅当 total > 1e-7 时，对**当前 quadrature contribution 最大**的
    segment 将 `n` 加倍并试算新总界；只有新 total 严格下降且细分前
    quadrature 仍是 `max(eval, angle)`（可改善主因）才接受。不可消除项
    （eval+angle ≥ 每坐标预算 `next_down(1e-7/sqrt2)`）或新界不降立即
    `BOUND_UNCONFIRMED`；不循环到 `MAX_PANELS`。总 subinterval 仍受
    `MAX_PANELS=200000` 与 64 次细分上限 fail-closed。
- `trajectory.py`：`numerics.version`、`path.profile.numerics_version`、
  `integration_budget.numerics_version` 都必须受支持且彼此一致；
  `from_model_json` 用文档声明版本重建（旧 v2 文档→v2，新文档→v3），
  不支持版本报 `UNKNOWN_NUMERICS_VERSION`，跨版本/不一致报
  `MODEL_NUMERICS_MISMATCH`。`_FLOATING_POINT_ASSUMPTION` 文本未改。
- `audit.py::_audit_bind_plan`：模型/路径/profile/预算版本都必须受支持且一致
  （`UNKNOWN_NUMERICS_VERSION` / `UNSUPPORTED_PROFILE` /
  `MODEL_NUMERICS_MISMATCH`）；独立 raw-knot 重建、5e-8 阈值与
  `evaluation_guard` 未改。
- 新规划默认 v3；v2 仅用于重建/核验旧模型。新 v3 model 的 SHA、planning
  context 与证书自然变化，旧 fixture 预期未被篡改。

## 2. 命令与环境

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
python3 work_flow/TASK01_evidence/05_numerics/probe_numerics.py \
  --map-yaml /tmp/expv3-task01-map.sax8HJ/map05-p12-parking-row-env2/map.yaml \
  --output work_flow/TASK01_evidence/05_numerics/numerics_probe.json
work_flow/TASK01_evidence/05_numerics/run_step5_tests.sh
python3 work_flow/TASK01_evidence/05_numerics/verify_numerics_evidence.py
```

- 探针退出码 0，耗时 61.15 s（`/usr/bin/time` 记录在
  `numerics_probe.stderr`）；stdout 见 `numerics_probe.stdout`，完整数据见
  `numerics_probe.json`。地图使用 Step 4 权威运行时缩放图
  （yaml `e4ee76a1…45e0`、image `0856ad0a…a718`、grid
  `55da861b…c457`；与 `00_baseline` 的 grid/request 输入同源）。
- §5.1 七个快速测试组逐组独立进程运行，原始 stdout/stderr、退出码与时长在
  `tests/`（`summary.txt`、`g*.stdout`、`g*.stderr`、`g*.result`）。
- 探针不启动 ROS；ROS/RViz 发布验证属于 Step 6（`07_ros_rviz/`），本文不
  冒充。

### 2.1 证书算术复核

`verify_numerics_evidence.py` 不导入 planner 结果，只读取
`numerics_probe.json` 并用同一 `_next_up` 重算每条记录的关系：版本集合、
`total = next_up(next_up(sqrt2)*per_coordinate)`、每段
`quad = full+local`、`angle = h*E_psi`、段总
`next_up(next_up(quad+eval)+angle)`、`total_panels`/full/local panel 计数、
per-segment M4 计数与最大值、version 声明与 tamper 原因。退出码 0
（`verify_numerics_evidence.stdout`）：

```text
verified 5 documents: replay byte-identical, 5 v3 certificates within 1e-7,
v2/v3 boundary and tamper reasons consistent
```

## 3. 旧 v2 model 可重建（门禁 1）

源：`evaluation/virtual_formation_a/G5/20261116T112522Z/final_main_free_5/
envelopes/envelope_00600000.json`（只读；未修改）。

| 项 | 值 |
|---|---|
| stored model sha256 | `df146b9bb4710132adeacd4d718d5a840c0f0e3bce353d06331a84142c2eaaf8` |
| rebuilt model sha256 | `df146b9bb4710132adeacd4d718d5a840c0f0e3bce353d06331a84142c2eaaf8` |
| byte-identical | `True`（9283/9283 chars） |
| declared numerics/profile/budget | `g1.v2` / `g1.v2` / `g1.v2` |
| replay budget 与文档预算结构一致 | `True` |
| v2 position bound | `3.3986020174706875e-08` m |
| v2 panels | 8×66 = 528 subintervals |
| 同一 knots 用 v3 重建 | `CONFIRMED`，bound `4.043312859891461e-09` m，528 subintervals |

结论：旧 v2 文档按其声明版本重建，输出字节不变；v2 路径未被新默认值改写。

## 4. 同几何 v2 ↔ v3 证书对照（门禁 2 的正例一侧）

对同一批 raw knots 分别显式选择 v2/v3（不是从结果“猜”版本）：

| 构造 | 长度 (m) | v3 bound (m) | v3 subintervals | v2 结果 |
|---|---:|---:|---:|---|
| 历史 600000 重建 | — | 4.0433128599e-09 | 528 | `CONFIRMED`（原文档） |
| 600003 实图路径 | 21.7016012085 | 5.1246799575e-09 | 1088 | `CONFIRMED`，1088 |
| 600004 实图路径 | 48.4362986116 | 1.1170049229e-08 | 2432 | `BOUND_UNCONFIRMED`，`total_panels=107008` |
| 50 m 直线 | 49.999999999999964 | 1.1376863043e-08 | 2512 | `BOUND_UNCONFIRMED`，`total_panels=101422` |
| 100 m 直线 | 100.0 | 2.2871226381e-08 | 5008 | `BOUND_UNCONFIRMED`，`total_panels=100160` |

v2 的失败位置就是其全局三角 M4 + 64 步 `MAX_PANELS` 边界；v3 用逐 panel
M4 在同一几何上一次通过（`refinement_steps=0`）。v3 bound 全部
≤ `1e-7 m`，v2 的历史 600000/600003 证书也仍可用（用于重建旧模型）。

## 5. 实图 600003/600004（v3）

冻结 Step 4 planner 策略：`guidance_mode=corridor, intervals=8,
max_initials=6, max_seconds=30, max_iterations=120, solver=ipopt`。

| 项 | 600003 | 600004 |
|---|---|---|
| status | `SUCCESS` | `SUCCESS` |
| 路径长度 (m) | 21.7016012085 | 48.4362986116 |
| guide 长度 (m) | 18.6258271507 | 45.7444252071 |
| v3 total bound (m) | 5.1246799575e-09 | 1.1170049229e-08 |
| angle bound (rad) | 1.6001183068e-10 | 1.6001183068e-10 |
| total subintervals | 1088 | 2432 |
| refinement steps / segments | 0 / `[]` | 0 / `[]` |
| geometry / dynamics / collision audit | True / True / True | True / True / True |
| model length (chars) | 27725 | 42814 |
| v3 model sha256 | `e830730ace79bf4e18f72b8072ee588a0314588f3826220f5deb39fc27d875ce` | `924beced4aebbc3a6e779d9d8a368eff47b24183796eab058ca37ebbacf01540` |
| planning context sha256 | `7449ee16690fd568726a5fc81c09cd9be162ff60adf8f55a207fe513852ab8cb` | `f5c759a8252e2eafc3396d8ff4f98c05559e4c62b36ffeb8226a890d8f7ab70e` |
| model 三处声明版本 | v3 / v3 / v3 | v3 / v3 / v3 |
| 模型 roundtrip 字节一致 | True | True |

600004 在 Step 4 中因 v2 证书边界报 `BOUND_UNCONFIRMED`（107008
subintervals），本步 v3 证书确认 `1.117e-8 < 1e-7` 后三项审核均过，
状态为 `SUCCESS`。600003 保持 `SUCCESS`。

Step 0 基线未记录 model/planning-context SHA（`repro_u_turn.json` 中
`context_sha256=null`，也无 model_sha256），因此不存在可对比的旧 v3 值；
上表是新的 v3 参考值，未改写任何旧证据或旧 fixture 预期。v3 model 文本
相对 v2 的变化来源是三处版本声明（`numerics.version`、
`path.profile.numerics_version`、`integration_budget.numerics_version`）
与新的证书字段。本次探针的 `context_id` 与 Step 0 复现脚本不同，所以
request SHA 不与基线做“未变”断言；起点/终点/limits/地图输入同源。

### 5.1 证书分项（单坐标米；Euclidean = sqrt2 × 总 per-coordinate）

600003（`m4_method=exact_interval_fourth_derivative_per_panel`，
`outward_rounding_method` 见 JSON，逐 product/sum `next_up`）：

| 量 | 值 |
|---|---:|
| E_quadrature_coord | 1.3846023171e-10 |
| E_evaluation_coord | 1.2940974490e-11 |
| E_angle_coord | 3.4722947431e-09 |
| per-coordinate total | 3.6236959493e-09 |
| Euclidean total (`total_budget_m`) | 5.1246799575e-09 |
| floating_point_guard_m（eval+angle，legacy 键） | 3.4852357176e-09 |

per-segment（600003；完整 panel M4 列表与 local panel 细节在 JSON）：

| seg | h (m) | n | full panels | max panel M4 | quad (m) | eval (m) | angle (m) | per-coord (m) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0 | 2.7127 | 136 | 68 | 1.52047e-02 | 3.29e-11 | 1.08e-12 | 4.34e-10 | 4.68e-10 |
| 1 | 2.7127 | 136 | 68 | 1.06001e-02 | 1.99e-11 | 1.08e-12 | 4.34e-10 | 4.55e-10 |
| 2 | 2.7127 | 136 | 68 | 1.08809e-02 | 2.38e-11 | 1.08e-12 | 4.34e-10 | 4.59e-10 |
| 3 | 2.7127 | 136 | 68 | 1.06383e-02 | 2.34e-11 | 1.08e-12 | 4.34e-10 | 4.59e-10 |
| 4 | 2.7127 | 136 | 68 | 8.14572e-03 | 1.68e-11 | 1.08e-12 | 4.34e-10 | 4.52e-10 |
| 5 | 2.7127 | 136 | 68 | 5.49698e-03 | 1.07e-11 | 1.08e-12 | 4.34e-10 | 4.46e-10 |
| 6 | 2.7127 | 136 | 68 | 3.61410e-03 | 7.36e-12 | 1.08e-12 | 4.34e-10 | 4.42e-10 |
| 7 | 2.7127 | 136 | 68 | 1.95685e-03 | 3.61e-12 | 1.08e-12 | 4.34e-10 | 4.39e-10 |

600004：

| 量 | 值 |
|---|---:|
| E_quadrature_coord | 9.0474841350e-11 |
| E_evaluation_coord | 5.7807579376e-11 |
| E_angle_coord | 7.7501351355e-09 |
| per-coordinate total | 7.8984175563e-09 |
| Euclidean total (`total_budget_m`) | 1.1170049229e-08 |
| floating_point_guard_m（eval+angle，legacy 键） | 7.8079427149e-09 |

600004 per-segment 的 max panel M4 在 seg 3/4 最大（8.18412e-03 /
8.13977e-03），quadrature 3.06e-11 / 3.05e-11；其余段更小。完整 8 段
subintervals（304×8）、panel M4 列表、quad/eval/angle/per-coordinate 均在
`numerics_probe.json` 的 `path.v3_certificate.segments`。

## 6. 50 m / 100 m 合成直线（v3）

solver-only（与 Step 4 相同的冻结初始值与策略；`legacy_route`，不查地图碰撞）：

| 构造 | status | 接受的 S (m) | v3 bound (m) | subintervals | 尝试数 | v2 对照 |
|---|---|---|---:|---:|---:|---|
| 50 m | `GEOMETRY_FEASIBLE` | 49.999999999999964（6 次接受） | 1.1376863043e-08 | 2512 | 6 | `BOUND_UNCONFIRMED`，101422 |
| 100 m | `GEOMETRY_FEASIBLE` | 100.0（前 4 次接受） | 2.2871226381e-08 | 5008 | 6 | `BOUND_UNCONFIRMED`，100160 |

100 m 的后两次求解返回 S=310.00000000973154 的候选并被审核拒绝
（`SOLVER_SUCCESS_AUDIT_REJECTED`，`BOUND_UNCONFIRMED`）；该 310 m 候选
不是被接受路径，最终几何可行路径仍为 100 m。这符合“证书不可确认即拒绝”，
没有把不可确认候选送进播放。50 m/100 m 的 v3 `E_quad` 接近 1e-24
（直线 M4≈0），主要误差项是角度项 `S*1e-9` 与求值 guard；v2 则在同一
几何上撞到 200000 上限。

## 7. 版本篡改拒绝（门禁 2 的负例一侧）

对 600003/600004 的 v3 model（结果一致，下表为 600003，600004 相同）：

| 篡改 | 读取/审核 | 结果 |
|---|---|---|
| `numerics.version = g1.v9` | `VirtualATimedPlan.from_model_json` | `UNKNOWN_NUMERICS_VERSION` |
| `numerics.version = g1.v2`（模型其余仍 v3） | 同上 | `MODEL_NUMERICS_MISMATCH` |
| `path.profile.numerics_version = g1.v9` | 同上 | `UNKNOWN_NUMERICS_VERSION` |
| `path.profile.numerics_version = g1.v2` | 同上 | `MODEL_NUMERICS_MISMATCH` |
| 内存 `_model_core` numerics → `"other"` | 独立 `audit_dynamics`（带 planning context） | `UNKNOWN_NUMERICS_VERSION`，scope `DYNAMICS`，location `constraint=MODEL_NUMERICS` |
| 内存 profile 版本 → `"other"` | 独立 `audit_dynamics` | `UNSUPPORTED_PROFILE`，scope `DYNAMICS`，location `constraint=PROFILE` |

未篡改的 600003/600004 v3 model 用同一 audit 与 planning context 会通过
（第 5 节三项 audit 均 True）。单元测试另覆盖 v3 声称 v2 的 numerics-only、
profile-only、both 与预算替换，见 `tests/test_virtual_a_trajectory.py`
`VersionedNumericsModelTests` 和 `tests/test_virtual_a_audit.py`
`VersionedAuditBindingTests`。

## 8. §5.1 快速测试

每条命令独立进程，产物在 `tests/`：

| 组 | 命令 | 退出码 | 时长 | 结果 |
|---|---|---:|---:|---|
| G1 | `python3 -m unittest tests.test_virtual_a_corridor_display tests.test_formation_hud` | 0 | 19.70 s | 67 tests OK (19.039 s) |
| G2 | `python3 -m unittest tests.test_virtual_a_receiver tests.test_virtual_a_playback` | 0 | 57.05 s | 49 tests OK (56.427 s) |
| G3 | `python3 -m unittest tests.test_formation_demo_assets` | 0 | 3.82 s | 28 tests OK (3.752 s) |
| G4 | `python3 -m unittest tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer` | 0 | 77.12 s | 64 tests OK (76.667 s) |
| G5 | `python3 -m unittest tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery` | 0 | 128.00 s | 38 tests OK (127.584 s) |
| G6 | `python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_profile` | 0 | 2.91 s | 49 tests OK (2.633 s) |
| G7 | `python3 -m unittest tests.test_virtual_a_integration_bounds tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit tests.test_virtual_a_trajectory` | 0 | 7.76 s | 103 tests OK (7.462 s) |

组内新增/保留的关键用例：G6 的 `V3NumericsTests`（v3 证书一致性与
fail-closed）以及显式 pin v2 的
`test_refinement_is_recorded_and_uses_the_corrected_remainder`；G7 的
`VersionedNumericsModelTests` 与 `VersionedAuditBindingTests`。
`test_virtual_a_profile.py` 中旧式 v2 guard 断言保留。

## 9. 边界和未完成项

- 本步没有运行 `catkin_make`、`run_demo.sh` 或 ROS 节点；ROS/RViz 原子发布
  验证仍属 Step 6（`07_ros_rviz/`）。
- `tools/virtual_a_benchmark.py` 的 `_integration_budget_audit` 仍是 v2 公式，
  本步未改；若后续把它作为 v3 证据运行，需要显式 pin v2 或更新公式（当前
  测试只用其 `_differential_evidence`/`quadrature_position`，不受影响）。
- 本步未修改历史证据（`00_baseline/`…`04_s_domain/`、`evaluation/`）与
  `TASK01_技术方案.md`；未覆盖用户更新的 `TASK01_Codex执行方案.md`。
- 未 commit/push；未进入 TASK 02；未宣称 Codex PASS 或 TASK 01 验收。