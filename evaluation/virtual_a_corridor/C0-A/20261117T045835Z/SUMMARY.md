# C0-A 证据摘要：段内前缀配点与失败摘要

- 阶段：**C0-A**（配点位置数学修复 + 失败摘要优先级）
- 证据目录：`evaluation/virtual_a_corridor/C0-A/20261117T045835Z/`
- 源码基线：`23fd16227e75a68f03f3ac7483555063c8189313`
- 当前 HEAD：`23fd16227e75a68f03f3ac7483555063c8189313`（同一提交）
- 分支：`feature/virtual-rigid-formation-a`
- 工作区状态：**未提交**。`optimizer.py`、`test_virtual_a_optimizer.py` 已修改；`test_virtual_a_collocation.py`、`tests/fixtures/virtual_a_corridor/`、`evaluation/` 为新增未跟踪文件。未建分支、未 commit、未 push。
- 停止状态：C0-A 完成后停止，等待 Codex / 用户验收；**未开始 C0-B**，未加走廊，未改 `5e5/100` 路线权重、段数或初值数量。

## 1. 前置 ACCEPTED 与基线说明

仓库内最近一份明确的 Codex 验收记录是：

- `evaluation/virtual_formation_a/G4/codex_acceptance_rework_01/CODEX_ACCEPTANCE.json`
- `status: ACCEPTED`
- `head: 567bdecba9676313da3bc9a97b82f57b5b569a2b`
- `source_manifest_sha256: caa929d09a4a346e910a13f8c1d5b4ad4a2bfe3b9dbf2e95b0609bd81994f8c6`

本阶段基线 `23fd...` 是 G5 实施提交。G5 报告
`evaluation/virtual_formation_a/G5/20261116T112522Z/CLAUDE_REPORT.md`
自述 Codex 验收为 `NOT_RUN / NOT_CLAIMED`；仓库内**没有找到 G5 的
`CODEX_ACCEPTANCE.json`**。本次执行依据是用户对
`plan/virtual_a_corridor_v01/TASK_C0.md` 的明确下达；此处如实记录该验收记录缺口，不自行补写 ACCEPTED。

## 2. 修改内容

### 2.1 配点位置：段内前缀积分

文件：`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py`

新增私有函数 `_piece_position_samples`（第 84 行）：

```text
p_j(q) = p_j(0) + h q sum_l w_l e(psi_j(q xi_l))
psi_j(q) = psi_j(0) + h q [c0 + q(c1/2 + q(c2/3 + q c3/4))]
```

三个生产调用点统一使用该函数：

- `_collocation_states`（第 460 行）：追加
  `(p_j(0)+dx, p_j(0)+dy, psi_j(q), kappa(q))`，不再把整段 Gauss
  全权重累计值当作 q 处位置；
- `_numeric_objective`（第 680 行）：route-collocation 与 clearance
  proxy 使用前缀样本位置；eta/u 积分仍为整段全 Gauss；
- `_build_casadi_nlp`（第 847 行）：同一数学语义，`sin/cos` 传入
  `ca.sin/ca.cos`，route/clearance 使用 `sample_x/sample_y`。

每段的段末位置仍由**独立的全 Gauss q=1 积分**推进，作为下一段起点；
`_numeric_constraint_vector`（第 624 行）本来就是段末积分，未改动。
未改 `_gauss`、`_coeffs`、车辆模型、路线权重、段数、初值数量或 solver 设置。

### 2.2 失败摘要优先级

`optimizer.py` 后处理块按任务书第 34 行顺序改为：

1. 已有 audited candidate：原样返回（未改）；
2. `deadline_exhausted`：`BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED`；
3. 存在 `SOLVER_SUCCESS_AUDIT_REJECTED`：`NO_ACCEPTED_CANDIDATE / AUDIT_REJECTED`，
   `detail` 保留最后一次被拒审计的具体原因；
4. 全部为数值异常：`NUMERICAL_FAILURE / ALL_ATTEMPTS_NUMERICAL_FAILURE`；
5. 全部普通不收敛：`NO_ACCEPTED_CANDIDATE / SOLVER_NOT_CONVERGED`；
6. 其它混合/未分类：保留既有行为。

沿用既有 status 枚举和 `attempts`，没有新增 Result 类型、wrapper 或
validator。solver 语义未改：`ipopt` 仅 IPOPT，`scipy/slsqp` 仅 SLSQP，
`auto + allow_scipy_fallback` 仅在非 timeout 失败且剩余预算 > 0 时每个
初值最多一次 SLSQP；timeout 不回退、不审计。

## 3. 量化门槛与实测

证据文件：`collocation_metrics.json`、`collocation_metrics.log`、
`test_collocation.log`。

| 检查 | 门槛 | 实测 | 结果 |
|---|---:|---:|---|
| 直线 72 个 Gauss 点的 x/y/psi/kappa 误差 | ≤ `1e-12 m/rad` | `8.8818e-16` | PASS |
| 旧累计法第一段 x 误差（负例） | > `1e-2 m` | `4.12799e-2 m` | PASS（复现旧错） |
| 修正后同一位置 x 误差 | ≤ `1e-12 m` | `0.0` | PASS |
| 旋转/平移等变 | ≤ `1e-9` | `2.2204e-16` | PASS |
| 镜像等变 | ≤ `1e-9` | `0.0` | PASS |
| N=8/S=6 合法 C2 曲线，72 个非网格点对 `VirtualAPath.state_at_s` | ≤ `1e-7 m`（含证书） | `4.3871e-11 + 1.79138e-8 = 1.79577e-8 m` | PASS |
| 同曲线段末等式残差 | ≤ `1e-9` | `4.3866e-11` | PASS |
| 同曲线轮式曲率约束 | ≤ `1.2098765` | `0.4285714` | PASS |
| numpy/CasADi base `f` 相对差 | ≤ `1e-9` | `0.0` | PASS |
| numpy/CasADi route `f` 相对差 | ≤ `1e-9` | `0.0` | PASS |
| numpy/CasADi clearance `f` 相对差 | ≤ `1e-9` | `5.0232e-16` | PASS |
| numpy/CasADi `g` 最大绝对差（base/route/clearance） | ≤ `1e-9` | `0.0` | PASS |
| 直线 route-collocation 目标增量 | ≤ `1e-9` | `0.0` | PASS |
| 旧累计法 route-collocation 惩罚（权重 1 的均值） | 仅记录 | `9.0260e-4`；按 `5e5` 权重为 `451.302` | 负例量级 |

`tests/test_virtual_a_collocation.py`：6 项全部通过。新增的
`test_straight_route_collocation_penalty_is_zero` 直接锁住“直线引导下
前缀配点不产生虚假目标”；旧累计法在同一算例产生 `451.302` 的目标项，
说明该修复不是只改文档。

## 4. 三组冻结 pipeline 单次运行

运行目录：`evaluation/virtual_a_corridor/C0-A/20261117T045835Z/`
运行脚本：`run_cases.py`；原始输出：`run_cases.log`；结构化结果：
`results.json`。

### 4.1 输入身份

| 算例 | 请求 SHA-256 | 期望匹配 | grid SHA-256 | 期望匹配 | context SHA-256 |
|---|---|---|---|---|---|
| A | `5f99cf2c...` | 是 | `c8da3454...` | 是 | `50018a88...` |
| B | `3fa8e46b...` | 是 | `33b3f0a6...` | 是 | `67a1f0e9...` |
| C | `88f3d15d...` | 是 | `33b3f0a6...` | 是 | `b9938730...` |

地图文件身份：

- A：YAML `888936d6...`，PGM `1524d82a...`，分辨率
  `0.05000000074505806`，原点 `(-1.5, -2.0)`，形状 `[100, 160]`。
- B/C：YAML `e6e33f34...`，PGM `0856ad0a...`，分辨率
  `0.04942600056529045`，原点 `(-4.5845285, -3.0563190000000002)`，
  形状 `[326, 489]`。

B/C 的 `33b3...` 只有按 ROS `OccupancyGrid.info.resolution` 的 float32
往返重建才成立；直接按 YAML 的 `0.049426` 会得到 `1fa8...`，不是历史
运行时 grid。该规则写在 `tests/fixtures/virtual_a_corridor/cases.json`，
runner 显式执行 `float(np.float32(res))`。B/C 源图与历史
`docs/virtual_formation_a/diagnostics_20261116/repro_uturn_summary.json`
一致。

### 4.2 结果表

| 算例 | status / reason | first layer | wall | attempts | path | executable |
|---|---|---|---:|---|---|---|
| A | `SUCCESS` | 空 | 20.666 s | 6 次，全部 `SOLVER_SUCCESS_AUDITED` | 4.000913 m，9 knots | true |
| B | `SUCCESS` | 空 | 34.244 s | 5 次；第 2 次 audited success，其余 3 次迭代上限、1 次 timeout | 3.952111 m，9 knots | true |
| C | `BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED` | `GEOMETRY` | 33.790 s | 6 次；3 次 audited 拒绝、2 次迭代上限、1 次 timeout | 无 | false |

A/B 路径证书：

- A：`position_error_bound_m = 4.6364e-8`，
  `angle_error_bound_rad = 1.6000e-10`，终点误差
  `1.8933e-9 m`，终点航向误差 `3.55e-15 rad`，max `|kappa| = 0.2843`，
  max `|eta| = 0.8760`。
- B：`position_error_bound_m = 8.1484e-8`，
  `angle_error_bound_rad = 1.6001e-10`，终点误差
  `8.2115e-9 m`，终点航向误差 `0.0`，max `|kappa| = 0.5475`，
  max `|eta| = 1.2075`。

A 的 pipeline 路径不是严格零曲率（`max |kappa| = 0.2843`，弦向局部
偏差存在），这是 route-guided NLP 的实际结果；**精确零曲率直线**由
`test_straight_line_prefix_positions_are_exact` 独立验证（误差
`8.88e-16`），两者不能混为同一断言。B 历史近直线包含约 `0.117 m`
局部横移，因此 `max |kappa|` 非零是预期行为，未要求零曲率。

C 的具体审计拒绝仍保留在 `results.json` 的 attempts 中：

| attempt | solver status | attempt status | 审计首错 |
|---:|---|---|---|
| 0 | `Solve_Succeeded` | `SOLVER_SUCCESS_AUDIT_REJECTED` | `AUDIT_INTEGRATION_REMAINDER = 9.95003e-8`（limit `5e-8`） |
| 1 | `Solve_Succeeded` | `SOLVER_SUCCESS_AUDIT_REJECTED` | `AUDIT_INTEGRATION_REMAINDER = 1.02156e-7` |
| 2 | `Maximum_Iterations_Exceeded` | `SOLVER_FAILED` | 无 |
| 3 | `Solve_Succeeded` | `SOLVER_SUCCESS_AUDIT_REJECTED` | `AUDIT_INTEGRATION_REMAINDER = 9.95054e-8` |
| 4 | `Maximum_Iterations_Exceeded` | `SOLVER_FAILED` | 无 |
| 5 | `Maximum_WallTime_Exceeded` | `SOLVER_TIMEOUT` | 无 |

C 的顶层是 `BUDGET_EXHAUSTED`，因为任务书规定 deadline 优先于
`AUDIT_REJECTED`；若没有 deadline，三个 audited 拒绝会按新规则给出
`AUDIT_REJECTED`（由 `test_audit_rejection_survives_later_solver_failure`
覆盖）。C 的审计预算本身仍未解决，属于 C0-B，不在本阶段擅自改
`5e-8`。

## 5. 分栏声明

### 5.1 求解器

- A/B：IPOPT 产生并通过独立几何审计的候选；B 在 timeout 后仍保留了
  之前已审计候选，符合既有“timeout 只保留已审计 best”语义。
- C：没有可接受候选；3 个 IPOPT success 被审计拒绝，2 个迭代上限，
  1 个 wall-time timeout。没有把 timeout 迭代当候选，也没有 timeout 后
  启动 SLSQP 回退。
- 定向测试 `test_virtual_a_optimizer.py` 29 项通过，覆盖：
  audit 拒绝不被后续 solver 失败覆盖、全不收敛归类、显式 ipopt 不启动
  SLSQP、auto 回退记录、timeout 不回退/不审计、fake success 仍被拒。
- 未做实时性、控制器、CAN、Gazebo 或实车声明。

### 5.2 几何

- A/B：pipeline 的 `geometry_audit.passed = true`，路径证书见上表。
- C：三个 success 的几何审计均拒绝，首错为
  `AUDIT_INTEGRATION_REMAINDER`，值约 `9.95e-8 ~ 1.02e-7`，limit
  `5e-8`；没有路径输出。
- 本阶段只修段内配点位置，没有放宽容度，没有改 `audit.py` /
  `integration_bounds.py` / `bounds.py`（见 `head_equality.txt`：这些文件
  与 HEAD 逐字节相同）。

### 5.3 动力学

- A/B：`dynamic_audit.passed = true`。
- C：无路径，动态审核 `NOT_RUN`。
- 未修改时间律、车辆限制或 retiming 配置。

### 5.4 碰撞（连续 OBB）

- A/B：`collision_audit.passed = true`，真实三车连续 OBB 扫掠在 offline
  adapter 中执行。
- C：无路径，碰撞审核 `NOT_RUN`。
- 本阶段未修改 `collision.py`，未改 margin / corner step / 采样预算。

### 5.5 发布 / 显示

- **NOT_RUN**：本阶段没有启动 ROS master、planner 节点、RViz 或任何
  发布链路；`ros_adapter.solve(VirtualAMapRequest)` 是离线路径。
- `result.executable = true` 只表示几何/动态/碰撞三层离线审核通过，
  不代表 ROS 回放、控制器跟踪或硬件可执行。
- `results.json` 的 `plan.is_executable_path` 是 runner 用
  `getattr(plan, "is_executable_path", False)` 得到的默认值；当前 timed
  plan 对象没有该属性，因此该字段为 `false` 并不与
  `result.executable = true` 矛盾。权威字段是 `result.executable`；
  plan 的 `request_sha256`、`planning_context_sha256`、`model_sha256`
  已记录并与结果绑定。

## 6. 失败、NOT_RUN 与未完成项

- C 仍未规划成功；`AUDIT_INTEGRATION_REMAINDER` 的 `5e-8` 预算问题
  未修复，留给 C0-B 做同候选预算分解。
- **C0-B 未开始**，未改 `audit.py` / `integration_bounds.py` / `bounds.py`。
- 未运行全量 `unittest discover`；只跑了任务书指定的三个定向模式。
- 真实 ROS / RViz / 发布 / 显示：**NOT_RUN**。
- 未做控制、CAN、Gazebo、实车、在线抢占或动态重规划。
- `timing_continuous` 历史 `KeyError: dynamic_peaks` 仍是既有失败，未修、
  未计为通过。
- 仓库内没有 G5 的 Codex `ACCEPTED` 文件；本报告不把 G5 声称为已验收。

## 7. 开放观察（未在本阶段修改）

1. **C 顶层 `deadline_exhausted` 字段与 reason 不一致**：
   `results.json` 中 C 的 `reason = SOFT_BUDGET_EXHAUSTED`，但
   `deadline_exhausted = false`。几何求解内部确实由 timeout 触发了
   deadline；顶层 `_empty_result` 在 `pipeline.py` 中没有传递该标志。
   status/reason/attempts 是正确的，`pipeline.py` 不在 C0-A 允许修改
   范围内，故未改；记录给 Codex 判定是否需要在后续阶段修正报告字段。
2. **clearance proxy 的坐标框架**：`_collocation_states` 返回
   start-local `(x,y,psi)`，而 `_clearance_proxy_angles` /
   `_proxy_numeric_penalty` 直接把该 `(x,y)` 传给世界坐标系的
   `CachedClearanceField`，没有应用 `start_qf` 的旋转/平移。
   route-collocation 的引导点本身是 start-local，因此只有 clearance
   proxy 存在这一框架疑问。该 proxy 只是软目标，不进入最终审计；A/B 的
   三层审计仍是独立通过。本阶段未改（超出 C0-A 的配点/失败摘要范围），
   需要后续独立确认其数值影响。
3. `_piece_position_samples` 的 Gauss 阶数由 `len(q_values)` 推断；当前
   三个生产调用点都传入完整 `_gauss(k)` 节点向量，端点另有独立全 Gauss
   循环，因此没有触发“单点探测降为一阶 Gauss”的陷阱。该私有函数的调用
   契约应保持为“传完整配点节点向量”。

## 8. 证据文件与复现

| 文件 | 内容 |
|---|---|
| `SUMMARY.md` | 本摘要 |
| `results.json` | A/B/C 三组单次运行的完整结构化记录 |
| `run_cases.log` | 三组运行的原始 stdout |
| `run_cases.py` | 从 `cases.json` 重建请求/grid 并调用离线 adapter 的 runner |
| `commands.log` | 精确命令、环境、退出码 |
| `test_collocation.log` | `test_virtual_a_collocation.py`，6 项 PASS |
| `test_optimizer.log` | `test_virtual_a_optimizer.py`，29 项 PASS |
| `test_integration_bounds.log` | `test_virtual_a_integration_bounds.py`，6 项 PASS |
| `collocation_metrics.py` / `.json` / `.log` | 配点、路线惩罚、f/g 的实测数值 |
| `source.diff` | 两个已跟踪文件的完整增量 diff |
| `git_state.txt` | base/current HEAD、branch、status、diff stat |
| `source_hashes.txt` | 源码、测试、fixture、runner 的 SHA-256 |
| `head_equality.txt` | 未改模块与 HEAD 的逐字节一致性 |
| `tests/fixtures/virtual_a_corridor/` | 两张地图、`cases.json` 和完整配置 |

基线与状态：

```text
base_commit:  23fd16227e75a68f03f3ac7483555063c8189313
current_head: 23fd16227e75a68f03f3ac7483555063c8189313
branch:       feature/virtual-rigid-formation-a
worktree:     uncommitted
diff:         2 tracked files changed, +209 / -29
              optimizer.py: +96 / -29
              test_virtual_a_optimizer.py: +113 / 0
```

复现命令见 `commands.log`。定向测试命令严格使用任务书给出的
`python3 -B -m unittest discover -s tests -p '...' -v`；三组 pipeline
只运行一次（另有一次在修正 fixture 字段名之前的失败启动，未写出
`results.json`，见 `commands.log` 说明）。

**停止**：C0-A 完成，等待 Codex 独立复核与用户明确验收；未提交 Git，
未推送，未开始 C0-B。