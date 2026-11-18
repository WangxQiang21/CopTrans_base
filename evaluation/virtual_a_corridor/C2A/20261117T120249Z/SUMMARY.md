# C2-A 阶段总结：Portal Chebyshev 初始化 QP、安全中线与 σ 分配

**状态：C2-A 代码、定向测试和证据已完成，工作区未提交，等待 Codex 独立验收。C2-B 未开始；不提交 Git、不推送、不启动 ROS/RViz。**

## 1. 基线、前置与工作区

| 项目 | 值 |
|---|---|
| 阶段 | C2-A |
| UTC run | `20261117T120249Z` |
| 任务书 | `plan/virtual_a_corridor_v01/TASK_C2.md` |
| 任务书源码基线 | `23fd16227e75a68f03f3ac7483555063c8189313` |
| 证据生成时 HEAD | `9a19fd3e22d33188b13795f695757c3e4a36909f`（C1 提交） |
| 分支 | `feature/virtual-rigid-formation-a` |
| 工作区 | 未提交；C2-A 改动 + 先前 V0/V1/Cargo HUD 改动同时存在 |
| Python / NumPy / SciPy | 3.8.10 / 1.24.4 / 1.10.1 |
| ROS | Noetic（仅用于 Python overlay；本阶段未启动 ROS 进程） |
| 源码目录 P | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a` |

### 1.1 C0/C1 ACCEPTED 前置缺口

`TASK_C2.md` 明确要求 `C0/C1 ACCEPTED`。本 run 在 `evaluation/` 与 `plan/` 中检索 `*accept*` / `*codex*` 文件，结果见 `prerequisite_search.log`：

- **未找到 C0/C1 的 Codex ACCEPTED 独立验收产物**。
- V1 的 `results.json` 明确记录 `c1_codex_accepted_artifact: null`。

因此，本报告**不声称 C0/C1 已获 Codex 验收**；C2-A 是在用户明确指示“开始推进c2”后继续实施。该前置缺口留给 Codex 在独立验收时判定。

### 1.2 Git 状态

C2-A 只修改/新增以下文件：

| 文件 | 状态 | C2-A 增量 | 相对 HEAD 的工作区 diff |
|---|---|---:|---:|
| `P/contracts.py` | 修改 | +16 / −2 | +16 / −2 |
| `P/corridor.py` | 修改 | +456 / −1 | +542 / −2（含 V1 显示改动） |
| `tests/test_virtual_a_corridor_optimizer.py` | 新增 | 344 行 / 13 tests | 344 行 / 13 tests |

`corridor.py` 在本阶段开始前已带有未提交的 V1 显示改动（`corridor_display_fields`）。为区分增量，本 run 用 `HEAD:corridor.py` 加 V1 `v0_v1_changes.patch` 中的 corridor.py 部分重建了 pre-C2A 基线，保存为 `pre_c2a_v1_corridor.py`，SHA-256 为 `c1c9142129a3102144a796b0b7229abef7423d82c3dc2dbcbc4146a32b4ba715`，与 V1 `source_hashes.txt` 完全一致。因此：

- `source_diff_c2a_corridor_only.diff`（SHA-256 `e04bf396...`）= pre-C2A V1 基线 → 最终 `corridor.py`，是 C2-A 的权威增量（+456 / −1）；
- `source_diff_corridor_from_head_including_v1.diff`（SHA-256 `9edd2eed...`）= `git diff HEAD` 的整份 corridor.py 差异（+542 / −2；其中 V1 为 +86 / −1，C2-A 为 +456 / −1），只用于说明工作区总量；
- `contracts.py` 没有 V1 改动，相对 HEAD 的 +16 / −2 全部属于 C2-A。

工作区还包含先前未提交的 V0/V1/Cargo HUD 改动（`formation_playback.py`、playback 脚本、demo launch/rviz、相关测试等）。这些不是本阶段 C2-A 改动；本阶段没有触碰 `optimizer.py`、`pipeline.py`、`trajectory.py`、`geometry.py`、`collision.py`、`timing.py`、`audit.py`、`bounds.py`、`ros_adapter.py` 或既有 C1/C0 测试。`head_equality.txt` 逐文件记录 `git diff --quiet HEAD` 的退出码 0。

`git_state.log` 记录完整 `HEAD/status/diff stat/name-status`。**本阶段没有新建分支、commit 或 push。**

## 2. 范围与非声明

### 2.1 本阶段实现

- `GuideRoute` 增加可选 `quality` 证据字段，旧的 A*/简化路线序列化与 equality/hash 语义不变。
- `corridor.make_corridor_guide(chain, start, goal)`：Chebyshev portal 中心初始化 + SciPy SLSQP QP，输出安全折线 `GuideRoute`。
- `corridor.allocate_corridors(chain, guide, intervals=8)`：按中线累计长度 `τ` 与固定 Hermite 分数并集分配 σ 区间。
- 新增 `tests/test_virtual_a_corridor_optimizer.py`，覆盖 M=1/3/12、非等距、U、hairpin、fallback、去重/缺口、序列化兼容。
- `sample_c2a.py/json` 给出 M=1、M=3 非等距、M=12、U 四组可复现小样例。

### 2.2 明确未实现/未声明

- **未接入 NLP、pipeline、PlannerConfig、guidance_mode、连续包含审核或三车最终审核**；这些属于 C2-B。
- **未实现局部二次 Bezier 圆角**；所有 guide 明确为 `PL_ONLY_NO_BEZIER`、`smoothness_claim=NONE`。
- **未启动 ROS/RViz**，没有真实地图、发布或显示证据。
- **未改三车运动学/动态/连续 OBB/时间律/审核物理门槛**。
- `GuideRoute` 仍不是可执行路径；`quality` 只记录构造证据。

## 3. 实现

### 3.1 `GuideRoute.quality` 兼容字段

`contracts.py` 中：

```python
quality: MappingABC = field(default=None, compare=False)
```

- 旧 A*/简化路线调用不传 `quality`，`as_dict()` 仍只有原来的五个键：`source`、`grid_sha256`、`points`、`length_m`、`is_executable_path`。
- `quality` 非空时才增加 `quality` 键，并复制为普通 `dict`。
- `compare=False` 保持旧 dataclass 的 equality/hash 语义；新增测试同时检查旧序列化键、相等性和 `hash()`，防止 C2 的 `dict` quality 让旧 `GuideRoute` 意外不可哈希。

### 3.2 Portal Chebyshev 中心与 QP

对每个内部 portal `P_j = C_{j-1} ∩ C_j`：

1. 用相邻走廊的 `A, b` 拼接，复用 C1 的 `_chebyshev_center`（HiGHS LP）求认证半径和中心。
2. 冻结 `ℓ_j = max(seed_length_j, 0.1 m)`；`L0 = max(||goal-start||, 1 m)`。
3. 初始化 `x0=start`、`xM=goal`、内部 `xj` 为 portal Chebyshev 中心。
4. 最小化

   `Σ ||x_{j+1}-x_j||² / L0²`
   `+ 0.1 Σ ||(x_{j+1}-x_j)/ℓ_j - (x_j-x_{j-1})/ℓ_{j-1}||²`
   `+ 0.01 Σ ||x_j-c_j||² / L0²`

   其中 `c_j` 是 portal Chebyshev 中心。
5. 线性约束为每个 `xj` 同时满足 `C_{j-1}` 与 `C_j` 的全部半空间；使用 SciPy `minimize(method="SLSQP")`、解析梯度、`ftol=1e-9`、`maxiter=300`。
6. 只有同时满足以下条件才接受 QP 结果：
   - solver `success`；
   - 结果有限；
   - 数值原始违约 `primal <= 1e-8 m`；
   - 每段长度 `> 1e-8 m`；
   - 每个顶点在分配走廊内，且最终 `_guide_metrics` 复核通过。
7. 否则保留已检查的 Chebyshev 折线，并在 `quality` 记录 `qp_attempted`、`qp_used`、`qp_status`、`qp_iterations`、`qp_objective`、`fallback_objective`、`fallback_reason`、`primal_violation_m`。没有新增 QP 框架、重试器或异常吞没。

### 3.3 安全折线与拓扑

- 每一段 `[x_j, x_{j+1}]` 分配给它所属的凸走廊 `C_j`。段的两端都在 `C_j` 内；由凸性，整段都在 `C_j` 内，因此不需要重新做 LOS 或碰撞证明。
- 没有可选 Bezier 圆角。`quality["rounding"]="PL_ONLY_NO_BEZIER"`，`quality["smoothness_claim"]="NONE"`，不把折线称为光滑路径。
- 负点积只在 `quality["negative_turn_indices"]` 和 `quality["warnings"]=["NEGATIVE_DOT_PRODUCT_PRESERVED"]` 中告警；不删除 V 点、不 shortcut、不改变真实 U/hairpin。
- 去重阈值 `1e-8 m` 用于拒绝近零长 guide 段。**没有通过删除重复 portal 来做 dedup**：删除一段会合并相邻走廊，导致某个 `C_j` 没有正区间，违背“不能跳过 C”。因此近零长段显式进入 `CORRIDOR_GUIDE_DEGENERATE`/fallback 路径，宁可不产出 guide，也不静默跳过走廊。

### 3.4 σ 分配

`allocate_corridors`：

1. 要求 `guide.points` 数量等于 `len(chain.regions)+1`，重新检查正段长和分配走廊包含。
2. `S = Σ ||x_{j+1}-x_j||`，累计边界 `τ_j = Σ_{i<j} ||x_{i+1}-x_i|| / S`，`τ_0=0`、`τ_M=1`。
3. 切点集合为 `{0,1} ∪ {τ_j} ∪ {k/8}`，排序并去掉 `<=1e-15` 的重复零长切点。
4. 每个区间用中点 `bisect_right(τ, midpoint)-1` 决定所属走廊；这样切点重合时不会因左右端点归属不同而跳过走廊。
5. 返回 tuple `(σ_a, σ_b, corridor_index)`；检查：
   - 覆盖 `[0,1]`；
   - 相邻区间连续、无缺口/倒序；
   - 所有走廊 0..M-1 均出现；
   - 每个区间 `σ_b > σ_a`。
6. 相邻不同 corridor 的区间在同一个 τ 切换点相接。C2-B 必须在该切换点同时用两侧走廊的半空间约束；C2-A 只给出切换分数和两侧索引，不声称已完成连续包含。

## 4. 小样例结果

`sample_c2a.py` 使用精确轴对齐盒（`A=(-1,0),(1,0),(0,-1),(0,1)`），不是真实地图。输出见 `sample_c2a.json`；下表数值来自最终代码。

| 样例 | guide 来源 | QP 状态 | QP 迭代 | QP 目标 | Chebyshev fallback 目标 | primal m | 区间数 | 覆盖 | 走廊索引 |
|---|---|---|---:|---:|---:|---:|---:|---|---|
| M=1 | endpoint PL | `NO_INTERNAL_PORTAL` | 0 | — | — | 0 | 8 | `[0,1]` | `{0}` |
| M=3 非等距 | QP PL | `SLSQP_SUCCESS` | 7 | `0.35605817153423364` | `0.403303454001592` | 0 | 10 | `[0,1]` | `{0,1,2}` |
| M=12 | QP PL | `SLSQP_SUCCESS` | 1 | `0.08333333333333333` | `0.08333333333333333` | 0 | 16 | `[0,1]` | `0..11` |
| U | QP PL | `SLSQP_SUCCESS` | 2 | `1.6274999999999997` | `3.4` | `2.220446049250313e-16` | 10 | `[0,1]` | `{0,1,2}` |

关键点：

- **M=3 非等距**：QP 把 internal portal 从 Chebyshev 中心移开，QP 目标低于 fallback；guide 点
  `[[0,0],[1.9760771982504293,-0.0],[3.4,0],[6,0]]`，段长
  `[1.9760772,1.4239228,2.6]`，冻结 seed 长度
  `[2.0,1.41421356,3.16227766]`。10 个区间由 2 个 τ 切点 + 7 个 Hermite 切点形成。
- **M=12**：13 个 guide 点、12 个走廊全部出现；16 个区间、无倒序/缺口、覆盖 `[0,1]`。直线场景中 12 个 τ 切点与 Hermite 切点部分重合，去重后仍不跳过任何走廊。
- **U**：4 个 guide 点保留 U 的两个内部拐点；QP 2 次迭代，目标 `1.6275 < 3.4`；没有 Bezier，未声称光滑。U 样例的转角点积不为负，因此告警为空；单独 hairpin 测试强制 fallback 并检查 `negative_turn_indices=[2]` 和 warning。
- **M=1**：退化为 `start→goal` 端点折线，8 个等分区间，全部属于 corridor 0。

## 5. 测试与回归

| 测试模式 | 结果 | 耗时 | 日志 |
|---|---:|---:|---|
| `test_virtual_a_corridor_optimizer.py` | **13 tests OK** | 0.023 s | `test_logs/test_virtual_a_corridor_optimizer.log` |
| `test_virtual_a_corridor.py`（C1 回归） | **25 tests OK** | 0.628 s | `test_logs/test_virtual_a_corridor_final.log` |
| `test_virtual_a_optimizer.py` | **29 tests OK** | 73.172 s | `test_logs/test_virtual_a_optimizer_final.log` |
| `test_virtual_a_trajectory.py` | **23 tests OK** | 0.510 s | `test_logs/test_virtual_a_trajectory_final.log` |
| `test_virtual_a_pipeline.py` | **8 tests OK** | 206.107 s | `test_logs/test_virtual_a_pipeline_final.log` |

所有测试均通过 `python3 -B -m unittest discover -s tests -p '<单文件模式>' -v` 定向执行，退出码 0；**没有执行全量 `unittest discover`**。C2-A 新增测试覆盖：

- M=1 端点折线 + 8 个区间；
- M=3 非等距 QP，目标等于 length + smoothness + centre 三项，seed 长度不等；
- M=12 对 N=8，13 点、所有 12 个走廊、无倒序/缺口；
- QP 强制异常 fallback 保留 Chebyshev 折线；
- endpoint 在第一走廊外显式拒绝；
- guide 数量与走廊数不匹配显式拒绝；
- τ 与 Hermite 切点重合的 M=8 精确算例不跳过走廊；
- 切换点位于 guide 累计边界；
- U 的 V 点不被删除/shortcut；
- hairpin 负点积只告警、顶点保留；
- sharp corner 不被静默圆角、不声称光滑；
- 旧 `GuideRoute` 无 `quality` 时序列化键不变，且 equality/hash 兼容；
- QP 解析梯度与中心差分一致（本机实测 `max_abs_gradient_error=5.292810634216e-11`，见 `gradient_check.log`）。

## 6. TASK_C2 C2-A 验收对照

| 要求 | 证据 |
|---|---|
| M 区域，`x0=start/xM=goal`，`xj∈Pj` | `make_corridor_guide` 校验端点、构造每个 portal 的 A/b，QP 约束两走廊半空间 |
| Chebyshev 中心初始化 | `_portal_bundle` → `_chebyshev_center`；M=3/M=12/U 的初值和 fallback 可复现 |
| 冻结 `ℓj=max(seed_length,.1m)`、`L0=max(端点距离,1m)` | `seed_lengths`、`l0`；`quality` 保存 `seed_lengths_m`、`segment_lengths_m` |
| QP 目标、SLSQP、primal≤1e-8 m | `_guide_qp_terms` 解析目标/梯度；SLSQP；`primal_violation_m` |
| 失败保留已验 Chebyshev 折线并记 quality | QP 异常/不成功/数值违约路径；`test_qp_exception...`；`fallback_reason` |
| 首版安全折线 | 每段两端在凸 `Cj` 内；`PL_ONLY_NO_BEZIER`；`is_executable_path=False` |
| 可选 Bezier 仅安全时使用，否则 PL，不声称光滑 | 本阶段未实现 Bezier；sharp corner 测试断言 PL 且 `smoothness_claim=NONE` |
| 去重阈值 1e-8 m | `_GUIDE_POINT_DEDUP_M=1e-8`；近零段显式拒绝，不以删除 portal 跳过走廊 |
| 负点积只告警，V 点/真实 U 不删除 | `negative_turn_indices`、`warnings`；U 与 hairpin 测试 |
| `allocate_corridors` τ + j/8 并集、区间的 corridor 索引 | `allocate_corridors`；M=1/3/12/U 样例与测试 |
| 去零长段同步索引，不能跳过 C | 排序切点去重；中点归属；`set(result indices)==range(M)` 检查 |
| M=1/3/12 对 N=8 全覆盖/无倒序/缺口 | `test_...` 三组；日志 `test_virtual_a_corridor_optimizer.log` |
| 不等距 QP、U 保留、不安全圆角拒绝 | M=3 非等距；U/hairpin；sharp corner 不圆角 |
| 证据目录、命令/退出码/哈希、未提交如实记录 | 本目录 `SUMMARY.md`、`results.json`、`commands.log`、`source_hashes.txt`、`git_state.log` 等 |

## 7. 限制、差异与遗留问题

1. **Bezier 圆角未实现**：任务中的 Bezier 是“可选”。本阶段保留 PL，不提供任何 fillet 接受路径；因此“不安全圆角拒绝”以“从不接受 fillet、sharp corner 保持 PL”体现。如果 Codex 要求显式的 fillet 候选/拒绝接口，需要在后续阶段返工，当前不能声称已实现该可选分支。
2. **去重语义**：`1e-8 m` 阈值用于拒绝近零 guide 段，而不是删除重复点。删除重复 portal 会合并相邻走廊并跳过某个 C，和“不能跳过 C”冲突；这里选择显式退化，不静默改拓扑。
3. **C2-A 未接入 pipeline**：`make_corridor_guide`/`allocate_corridors` 已是可调用接口，但 C2-B 才会构造 chain/guide/allocation、加入 NLP 半空间、连续包含审核和发布门。当前 `test_virtual_a_pipeline.py` 8/8 通过只证明既有 legacy 路径未回归，**不证明**新走廊引导已接入或三车可执行。
4. **没有真实地图/ROS/RViz**：样例是精确小盒；map05 等真实地图、RViz 显示和 ROS 发布均 **NOT_RUN**。
5. **未测量 QP 预算**：本阶段没有把 SLSQP 时间纳入规划请求预算；C2-B 接入时必须单独测量并纳入 context/预算。
6. **前置缺口**：未找到 C0/C1 Codex ACCEPTED 产物；本报告已如实记录，不把它解释为已验收。
7. **工作区有先前未提交改动**：V0/V1/Cargo HUD 改动与 C2-A 同时存在，且 `corridor.py` 本身已含 V1 显示改动；`git_state.log` 的整树 diff stat 和 +542 / −2 都包含它们。C2-A 的增量以 `source_diff_c2a_contracts.diff`（+16 / −2）、`source_diff_c2a_corridor_only.diff`（+456 / −1，相对可复现的 pre-C2A V1 基线）、`new_test_c2a.diff`（344 行新测试）为准；`source_diff_corridor_from_head_including_v1.diff` 只记录工作区总量。
8. **历史 `timing_continuous` `KeyError: dynamic_peaks` 未修复**：C2-A 没有触碰 timing；该历史问题保持原状。

## 8. 分栏状态

| 分栏 | C2-A 状态 |
|---|---|
| 求解成功 | C2-A guide QP 样例成功；三车 NLP/连续包含/新 corridor 发布 **NOT_RUN**（C2-B） |
| 几何 | 安全 PL、portal 包含、M=1/3/12、U/hairpin 定向测试通过；真实地图 **NOT_RUN** |
| 动态 | C2-A 未改动态代码；现有 optimizer/trajectory 回归通过，但没有新 corridor 动态结论 |
| 碰撞 | C2-A 未改三车连续 OBB/独立审核；没有把参考点安全当成车辆安全 |
| 发布/显示 | **NOT_RUN**：未启动 ROS master、规划节点、RViz、控制器或发布 |
| Codex 独立验收 | **PENDING**：本报告只是 Claude 侧证据，不构成 ACCEPTED |

## 9. 证据文件

| 文件 | 用途 |
|---|---|
| `SUMMARY.md` | 本文件 |
| `results.json` | 机器可读的阶段状态、测试、样例、差距和 NOT_RUN |
| `commands.log` | 精确命令、环境、退出码、日志路径 |
| `source_hashes.txt` | 最终源码/测试/样例/任务书/diff 的 SHA-256 |
| `git_state.log` | 最终 HEAD/branch/status/diff stat/numstat；明确未提交 |
| `head_equality.txt` | 保护文件相对 HEAD 未改（每行退出码 0） |
| `prerequisite_search.log` | C0/C1 ACCEPTED 产物检索结果（未找到） |
| `environment.txt` | Python/NumPy/SciPy/uname/ROS distro |
| `sample_c2a.py` / `sample_c2a.json` | 四组可复现小样例与完整 quality/allocation |
| `gradient_check.log` | QP 解析梯度中心差分实测值 |
| `pre_c2a_v1_corridor.py` | 重建的 C2-A 前 corridor.py 基线（HEAD + V1 显示改动，SHA-256 `c1c914...`） |
| `source_diff_c2a_contracts.diff` | `contracts.py` 的 C2-A 增量 |
| `source_diff_c2a_corridor_only.diff` | `corridor.py` 的 C2-A 权威增量（相对 pre-C2A V1 基线） |
| `source_diff_corridor_from_head_including_v1.diff` | `corridor.py` 相对 HEAD 的整份工作区 diff（V1 + C2-A，仅作总量参考） |
| `new_test_c2a.diff` | 新测试全文 diff |
| `test_logs/*_final.log` | 最终定向测试日志 |
| `source_hashes_pre_final.txt`、`git_state_pre_final.log`、`test_logs/*_pre_final.log`、`test_logs/*_pre_hash_fix.log` | 早期快照，**不作为最终证据**；保留用于审计过程 |

## 10. 停止点

C2-A 完成。**在此停止，等待 Codex 独立验收；不自动进入 C2-B，不提交 Git，不推送，不启动 ROS/RViz。**