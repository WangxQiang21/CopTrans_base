# C0-B 证据摘要：同候选审计预算分解与过松 guard 修正

- 阶段：**C0-B**（同 κ/η/S 候选冻结、Simpson 预算分解、审计 guard 修正）
- 证据目录：`evaluation/virtual_a_corridor/C0-B/20261117T052817Z/`
- 源码基线：`23fd16227e75a68f03f3ac7483555063c8189313`
- 当前 HEAD：`23fd16227e75a68f03f3ac7483555063c8189313`（同一提交）
- 分支：`feature/virtual-rigid-formation-a`
- 工作区状态：**未提交**。`audit.py`、`integration_bounds.py` 为 C0-B 修改；
  `optimizer.py`、`test_virtual_a_optimizer.py` 仍是 C0-A 未提交修改
  （本轮 hash 与 C0-A 记录一致，C0-B 未再改）；`bounds.py`、`geometry.py`、
  `collision.py`、`pipeline.py`、ROS 与测试预期均与 HEAD 逐字节相同。新增
  `tests/test_virtual_a_audit_evaluation_guard.py`。未建分支、未 commit、未 push。
- 停止状态：C0-B 完成后停止，等待 Codex 独立复核与用户明确验收；**未开始
  C1/走廊**，未改 `5e5/100` 路线权重、段数、初值数量、车辆限制、地图、ROS
  或时间律。

## 1. 前置验收记录与本次依据

仓库内最近一份明确的 Codex 验收记录仍是：

- `evaluation/virtual_formation_a/G4/codex_acceptance_rework_01/CODEX_ACCEPTANCE.json`
- `status: ACCEPTED`，`head: 567bdecba9676313da3bc9a97b82f57b5b569a2b`

没有找到 G5 或 C0-A 的 `CODEX_ACCEPTANCE.json`。C0-A 报告自述
“C0-A 完成，等待 Codex 独立复核与用户明确验收”。本次执行依据是用户在
C0-A 完成后明确下达的 `执行c0-b`；此处如实记录验收缺口，不自行补写
ACCEPTED。C0-A 的证据目录 `evaluation/virtual_a_corridor/C0-A/20261117T045835Z/`
未被覆盖。

C0-B 的数学结论与 C0-A 的“成功规划”声明分开记录；本报告只把 C0-B
范围内实测到的预算确认、单次 pipeline 运行和分栏审核结果作为事实陈述。

## 2. 候选冻结：一次定向复现

C0-A 的 `results.json` 没有保存 solver 的 κ/η/S。按任务书第 36 行，C0-B
只做**一次**定向复现：用 `tests/fixtures/virtual_a_corridor/cases.json`
重建 C 请求（float32 ROS 分辨率，grid SHA `33b3f0a6...`），离线调用
`ros_adapter.solve(VirtualAMapRequest)` 一次，monkeypatch
`optimizer.audit_geometry` 捕获每个进入审计的候选，并回放**修复前**的
预算循环。原始记录：`capture.log`、`candidates.json`。

复现结果与 C0-A 的拒绝值逐项吻合：

| capture | C0-A 拒绝值 | C0-B 复现值 | S (m) | 初始总细分（full/local） | 64 步后 |
|---:|---:|---:|---:|---:|---:|
| 0 | `9.95003e-8` | `9.95002761269226e-8` | 15.163479941796252 | 768（384/8） | `REFINEMENT_LIMIT` |
| 1 | `1.02156e-7` | `1.0215595858990511e-7` | 15.573642704152004 | 784（392/8） | `REFINEMENT_LIMIT` |
| 2 | `9.95054e-8` | `9.950540375733431e-8` | 15.167666918790209 | 768（384/8） | `REFINEMENT_LIMIT` |

复现顶层结果为 `BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED / GEOMETRY`，
wall 33.801 s，与 C0-A 一致。该次运行只用于冻结候选；修复后的运行见第 6 节。

## 3. 同候选预算分解（修复前）

`candidates.json` 的 `decomposition` 是修复前预算的完整逐步回放；
`budget_analysis.py` 从同一冻结 profile 重建并输出
`budget_analysis.json`（schema v2）。capture 0 的分解：

| 项 | 初始（n=96/段） | 最小（第 1 步） | 64 步后 |
|---|---:|---:|---:|
| Simpson 余项（欧氏） | `7.893303e-9` | 略降 | `1.738727e-10` |
| `position_guard` 中 `1e-10*(panels+8)` | `4.000000e-8` | 增大 | `9.690000e-8` |
| `L*angle_bound` | `2.426403e-9` | 略变 | `2.426403e-9` |
| 旧总界 | `5.0319706714827974e-8` | `5.0266121896104534e-8` | `9.95002761269226e-8` |
| 全/局部 Simpson 面板 | 384 / 8 | — | 953 / 8 |

三个候选的旧总界都满足：初始已略高于 `5e-8`，第 1–2 步达到最小值
（`5.0266e-8`、`5.1363e-8`、`5.0446e-8`），之后**单调增大**，64 步后
`9.95e-8 ~ 1.02e-7`。这不是轨迹质量或余项不够小的问题：在旧 guard 下
`5e-8` 对同候选**数学上不可达**。

## 4. 根因与独立推导

### 4.1 错用的项

旧审计直接调用生产证书的

```text
position_guard(L, panels, angle_bound)
  = 1e-10 * (panels + 8) + L * angle_bound
```

其中 `1e-10 m/面板` 是 G1 **生产路径**的累积保护项：生产 `VirtualAPath`
通过自身的 profile/angle 证书计算位置，该项覆盖其嵌套求值的条件数。
独立审计的积分则是普通 float64 复合 Simpson，直接对冻结的 κ/η 系数求
`cos/sin`。把生产 guard 原样搬进审计，使审计预算每细化一步增加
`~1e-10 m`，而 Simpson 余项按 `~n^-4` 下降，因此细化到一定程度后总界
必增。这是“**实际算错/过松界**”，不是需要放宽 `5e-8` 的证据。

### 4.2 float64 求值误差的独立推导

`integration_bounds.evaluation_guard(records, psi_start, start_magnitude_m)`
逐项推导审计自身 Simpson 求值的 float64 误差（`u = EPSILON`，
`L = Σ h_j`，`Ψ = 1 + |ψ_start| + Σ_j h_j·Σ_i|c_i|`）：

```text
E_eval = Σ_j u·h_j·(8 n_j + 28 + 16 Ψ)
       + 16 (segments+1) u·L·Ψ
       + 2 (segments+1) u·(|start| + L)
```

- 第一项：采样点 `q`、`psi_increment` 的 13 次 Horner 舍入、libm 一次
  舍入、权重乘积与两个和的顺序累加。顺序求和 `n+1` 项、总权重 `3n`
  的经典界为 `(n+1)·u·3n`，经 Simpson 因子折算后为 `(n+1)` 个
  `u·h` 单位；`8n` 保留记录在案的安全因子。
- 第二项：ψ 前缀累积使整段积分核偏移 `dψ`，位置影响不超过
  `h_j·dψ`，逐段求和。
- 第三项：坐标前缀 `x += dx` 的累加舍入，`|start|+L` 为量级上界。

所有乘积/加法均用 `bounds._next_up` 向外舍入。该函数**只**替代审计
自身的 float64 保护，不替代任何截断余项；`bounds.POSITION_ROUNDOFF_GUARD_M`、
生产 `1e-7 m` 合同与审计 `5e-8 m` 阈值均未改。

### 4.3 同时修正的欧氏组合

`evaluation_guard` 与 `L*angle_bound` 都是**单坐标**界；旧代码把它们
未乘 `√2` 直接加到已乘 `√2` 的 Simpson 余项上，严格欧氏上界会被低估
（旧实现同样存在，只是在 `1e-10/面板` 的巨大项下被掩盖）。C0-B 在
`_integration_plan` 中改为

```text
guard = √2 · (E_eval + L·angle_bound)
total = euclidean_remainder + guard
```

并记录 `per_coordinate_roundoff_and_angle_guard_m`、
`guard_euclidean_scale`、`evaluation_guard_m`、`angle_guard_m` 及
`evaluation_guard_components`，便于 Codex 手算。

### 4.4 测量验证

`budget_analysis.py` 用 `numpy.longdouble`（80-bit，`eps≈1.084e-19`）
在**相同**面板数下镜像 `_integrate_xy`，隔离 float64 求值误差；另用
1024 细分/段的 longdouble Simpson 作为全误差参照。capture 0：

| 量 | 实测 | 界 | 比值 |
|---|---:|---:|---:|
| 同面板 float64 误差（8 段最大） | `1.110228e-15 m` | `E_eval = 1.270098e-11 m` | 1.1e4 |
| 审计 vs 1024 细分参照（初始面板） | `5.998624e-10 m` | 新总界 `1.134272e-8 m` | 18.9 |
| 审计 vs 1024 细分参照（64 步面板） | `2.012605e-11 m` | 新总界（同面板）`3.628706e-9 m` | 180 |
| 生产 `state_at_s` vs 同一参照 | `5.998624e-10 m` | — | — |

旧 `1e-10/面板` 项（初始 `4.0e-8`）是同面板实测 float64 误差的
`3.6e7` 倍；新推导界仍有 `1.1e4` 倍余量，是真正的上界而不是“贴着测量
调参”。三个冻结候选均得到同量级结论（`E_eval = 1.28e-11 ~ 1.36e-11`，
同面板实测 `1.1e-15 ~ 1.8e-15`）。

## 5. 修改内容

### 5.1 `integration_bounds.py`

新增 `evaluation_guard(records, psi_start, start_magnitude_m)`（含上述
推导、向外舍入和逐段分量），只返回界与分解，不做生产路径决策。模块
docstring 更新为“生产 1e-7 与 1e-10 guard 不动，审计改用推导 guard”。
`position_budget`、`segment_budget`、`panel_m4_bounds` **未改**。

### 5.2 `audit.py`

- `_integration_plan(pieces, start_psi, config, start_magnitude_m=0.0)`：
  用 `integration_bounds.evaluation_guard` 替代 `bounds.position_guard`，
  单坐标项乘 `√2` 后与欧氏余项相加；
- 两个调用点（几何审计、动态审计）传入
  `max(|start_qf.x|, |start_qf.y|)` 作为坐标前缀量级；
- 预算记录新增 `evaluation_guard_m`、`evaluation_guard_components`、
  `angle_guard_m`、`per_coordinate_roundoff_and_angle_guard_m`、
  `guard_euclidean_scale`，逐段新增 `evaluation_guard_m`；
  `method`、`m4_method`、`formula`（截断公式）、`fixed_threshold_m`、
  `total_subintervals`、`full/local_simpson_panels`、`refinement_steps`
  语义不变；`roundoff_and_angle_guard_m` 仍是“guard”字段，但内容改为
  推导求值项 + 角度项的欧氏组合，`floating_point_assumption` 已相应改写。

### 5.3 未改

- `bounds.py`：**0 行修改**（`POSITION_ROUNDOFF_GUARD_M=1e-10`、
  `position_guard` 保持生产语义）；
- `optimizer.py` / `test_virtual_a_optimizer.py`：与 C0-A 记录 hash 一致；
- `audit_position_budget_m=5e-8`、`integration_budget_m=1e-7`、
  `ANGLE_BUDGET_RAD=1e-9`、车辆限制、地图、ROS、时间律、路线权重、
  段数、初值数量：未改；
- `tests/test_virtual_a_audit.py`、`test_virtual_a_integration_bounds.py`、
  `test_virtual_a_profile.py`、`test_virtual_a_geometry.py`：与 HEAD
  逐字节相同（`head_equality.txt`），未改任何 5e-8 测试预期。

## 6. 修复后的量化验证与 pipeline

### 6.1 同候选：旧界 vs 新界

| capture | 旧初始总界 | 旧最小总界 | 新初始总界 | 新 `E_eval` | 新欧氏 guard | 确认 |
|---:|---:|---:|---:|---:|---:|:---:|
| 0 | `5.0319707e-8` | `5.0266122e-8` | `1.1342718e-8` | `1.2701e-11` | `3.4494e-9` | 是 |
| 1 | `5.1499826e-8` | `5.1362788e-8` | `1.1751220e-8` | `1.3553e-11` | `3.5434e-9` | 是 |
| 2 | `5.0492890e-8` | `5.0445600e-8` | `1.1516290e-8` | `1.2780e-11` | `3.4505e-9` | 是 |

三个冻结候选在**初始面板**即确认 `5e-8`，`refinement_steps=0`；即使在
旧循环的 64 步面板上，新总界也只有 `3.63e-9 ~ 3.71e-9`。因此不需要把
`5e-8` 改成 `1e-7`，也不需要为此提交碰撞误差包络放宽推导。

### 6.2 定向测试

| 测试 | 数量 | 结果 | 日志 |
|---|---:|---|---|
| `test_virtual_a_collocation.py` | 6 | OK | `test_collocation.log` |
| `test_virtual_a_optimizer.py` | 29 | OK | `test_optimizer.log` |
| `test_virtual_a_integration_bounds.py` | 6 | OK | `test_integration_bounds.log` |
| `test_virtual_a_audit.py` | 64 | OK | `test_audit.log` |
| `test_virtual_a_profile.py` | 22 | OK | `test_profile.log` |
| `test_virtual_a_audit_evaluation_guard.py`（新增） | 4 | OK | `test_audit_evaluation_guard.log` |

新增测试覆盖：冻结候选在推导 guard 下确认 `5e-8` 且旧 guard 总界仍
拒绝；同面板 longdouble 对照下 `E_eval` 是实测 float64 误差的上界；
guard 随面板数增大；`1e-12` 阈值仍 fail-closed。未运行全量 discover。

### 6.3 三组冻结 pipeline（修复后各单次）

运行脚本 `run_cases.py`，原始输出 `run_cases.log`，结构化
`results.json`。输入身份与 C0-A 完全一致（A/B/C 的 request/grid/context
SHA 全部匹配期望）。

| 算例 | status / reason | first layer | wall | attempts | path | executable |
|---|---|---|---:|---|---|---|
| A | `SUCCESS` | 空 | 20.018 s | 6 次，全部 audited success | 4.000913 m，9 knots | true |
| B | `SUCCESS` | 空 | 34.160 s | 5 次；第 3 次 audited success | 3.952111 m，9 knots | true |
| C | `SUCCESS` | 空 | 37.642 s | 6 次；3 次 audited success，2 次迭代上限，1 次 timeout | 15.167667 m，9 knots | true |

A/B 的路径长度、证书与 C0-A 相同（A `position_error_bound_m=4.6364e-8`，
B `8.1484e-8`），行为未被 C0-B 改变。C 的 3 个 audited success 分别为：

| C attempt | S (m) | 审计新总界 | 与冻结候选 |
|---:|---:|---:|---|
| 0 | 15.163479941796252 | `1.1342717821723744e-8` | capture 0，数值逐位一致 |
| 1 | 15.573642704152004 | `1.1751219643168802e-8` | capture 1，数值逐位一致 |
| 3 | 15.167666918790209 | `1.1516290370563627e-8` | capture 2，knots 逐位一致，最终被采纳 |

即：同一个冻结 κ/η/S 候选在 C0-A 被 `AUDIT_INTEGRATION_REMAINDER`
拒绝，在 C0-B 修复 guard 后以 `1.15e-8 ≤ 5e-8` 确认；pipeline 的
`geometry_audit.integration_budget.total_budget_m` 与
`budget_analysis.json` 的独立重算**完全相等**。C 最终路径的
`position_error_bound_m = 6.449e-8`（生产 1e-7 内），终点误差
`5.13e-10 m`，终点航向误差 `0`。

## 7. 分栏声明

### 7.1 求解器

- A：6 个 IPOPT success 全部通过审计，最终采纳 audited candidate；
- B：3 个迭代上限、1 个 audited success、1 个 wall-time timeout；timeout
  后未启动 SLSQP、未审计 timeout 迭代，保留了已有的 audited 候选；
- C：3 个 audited success，2 个迭代上限，1 个 timeout；最终采纳最后一个
  audited success（capture 2），而不是 timeout 迭代。
- 未做实时性、控制器、CAN、Gazebo 或实车声明。

### 7.2 几何

- A/B/C 的 `geometry_audit.passed = true`；审计预算分别为
  `1.8924e-8`、`4.3849e-8`（`refinement_steps=2`）、`1.1516e-8`
  （`refinement_steps=0`），均 ≤ `5e-8`；
- 审计的 `evaluation_guard_m` 分别为 `5.567e-13`、`1.075e-12`、
  `1.278e-11`，远小于角度项 `6.4e-10 ~ 2.43e-9`；没有隐藏的
  `1e-10/面板` 项；
- 生产路径证书未改：A/B/C `position_error_bound_m` 均 ≤ `1e-7`。

### 7.3 动力学

- A/B/C `dynamic_audit.passed = true`，使用未改的时间律、车辆限制和
  原始 profile 动态审计；C 首次有路径，因此该层从 C0-A 的 `NOT_RUN`
  变为实际运行并记录；
- 动态审计同样调用 `_integration_plan`，共享同一推导 guard；本阶段未改
  动力学公式、限制或 retiming。

### 7.4 碰撞（连续 OBB）

- A/B/C `collision_audit.passed = true`，真实三车连续 OBB 扫掠在离线
  adapter 中执行；C 首次实际运行；
- 未改 `collision.py`、margin、corner step 或采样预算。由于未把
  `5e-8` 放宽到 `1e-7`，本阶段不需要、也没有改动碰撞误差包络。

### 7.5 发布 / 显示

- **NOT_RUN**：没有启动 ROS master、planner 节点、RViz 或发布链路。
  `ros_adapter.solve(VirtualAMapRequest)` 是离线路径；
- `result.executable = true` 只表示离线几何/动态/碰撞三层审核通过，
  不代表 ROS 回放、控制器跟踪或硬件可执行；`plan.is_executable_path`
  的默认值 `false` 与 C0-A 的报告语义相同，权威字段是
  `result.executable`。

## 8. 失败、NOT_RUN 与已知不一致

1. **真实 ROS / RViz / 发布 / 显示：NOT_RUN**。未使用 ROS master，
   未触碰任何用户进程。
2. **未运行全量 `unittest discover`**；只跑了上表 6 个定向模式。
3. **`tools/virtual_a_benchmark.py` 的一致性检查未同步**：
   `_g2_audit_integration_audit` 仍硬编码旧审计 guard
   `1e-10*(panels+8) + L*angle`，而 C0-B 修改了
   `roundoff_and_angle_guard_m` 的构成。该工具不在任务书允许修改的
   文件内，本阶段**没有改它、也没有重跑 G2 工具**；若以后运行该检查，
   会得到 `ok=false`，必须先在获得授权的阶段把 formula 同步为
   `integration_bounds.evaluation_guard`。`_integration_budget_audit`
   检查的是生产预算，生产未改，不受影响。历史 G2 证据目录中的脚本
   同样硬编码旧公式，属历史证据，未修改、未重跑。
4. `timing_continuous` 历史 `KeyError: dynamic_peaks` 仍是既有失败，
   未修、未计为通过。
5. 仓库内**没有 G5 / C0-A 的 Codex `ACCEPTED` 文件**；本报告不把
   G5 或 C0-A 声称为已验收。
6. C0-A 的两个开放观察仍存在（`pipeline.py` 顶层
   `deadline_exhausted` 字段与 reason 不一致；clearance proxy 的
   start-local/world 坐标框架疑问）。本阶段未改 `pipeline.py` 或
   `optimizer.py`，只记录。
7. 推导的浮点假设：IEEE-754 double、libm `sin/cos` 在 guard 内正确
   舍入、无跨语句 FMA 融合；`integration_bounds` 模块 docstring 已
   记录。界相对实测有 `1e4` 余量，但这不是形式化证明。
8. 修复后的 C 成功是**单次**运行结果；本报告不把它外推为大规模
   稳定性或走廊阶段的成功声明。

## 9. 证据文件与复现

| 文件 | 内容 |
|---|---|
| `SUMMARY.md` | 本摘要 |
| `candidates.json` | 一次定向复现捕获的 3 个候选、完整 knots、修复前预算逐步回放 |
| `capture.log` | 复现的原始 stdout（含与 C0-A 相同的拒绝值） |
| `budget_analysis.py` / `.json` / `.log` | 同候选旧/新预算分解、同面板 longdouble 对照、生产误差 |
| `run_cases.py` / `run_cases.log` / `results.json` | 修复后 A/B/C 各单次运行及逐 attempt 审计预算 |
| `commands.log` | 精确命令、环境、退出码、单次运行说明 |
| `git_state.txt` | base/current HEAD、branch、status、diff stat |
| `source_diff_c0b.diff` | C0-B 生产增量（`audit.py`、`integration_bounds.py`） |
| `source_diff_c0a_context.diff` | C0-A 未提交增量（仅作上下文，C0-B 未改） |
| `new_test_diff.diff` | 新增测试文件 |
| `source_hashes.txt` | 源码、测试、fixture、runner、证据文件 SHA-256 |
| `head_equality.txt` | 受保护文件与 HEAD 的逐字节一致性；C0-A 文件 hash 未变 |
| `test_*.log` | 6 个定向测试模式，共 131 项全部 OK |

基线：

```text
base_commit:  23fd16227e75a68f03f3ac7483555063c8189313
current_head: 23fd16227e75a68f03f3ac7483555063c8189313
branch:       feature/virtual-rigid-formation-a
worktree:     uncommitted
tracked diff: audit.py +56/-13, integration_bounds.py +112/-2
              (C0-A optimizer.py +96/-29, test_virtual_a_optimizer.py +113/-0 保持不变)
new files:    tests/test_virtual_a_audit_evaluation_guard.py
```

**停止**：C0-B 完成，等待 Codex 独立复核数学与原始证据、用户明确验收；
未提交 Git，未推送，未开始 C1/走廊，未自动推进。