# C3_REWORK_01 返工证据摘要

- 合同：`plan/virtual_a_corridor_v01/CLAUDE_C3_REWORK_01.md`
- 运行目录：`evaluation/virtual_a_corridor/C3_REWORK_01/20261117T185418Z/`
- 工作树：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- 分支 / HEAD：`feature/virtual-rigid-formation-a` / `9a19fd3e22d33188b13795f695757c3e4a36909f`
- 本阶段只处理 C3_REWORK_01 的两项阻断问题（R1、R2）。未进入 C4/C5，未启动 ROS/RViz，未发布，未 commit/push，也不声称 Codex ACCEPTED。

## 1. R1：整请求共享初值池

### 实现

- `pipeline.py` 在请求开始时建立唯一的 `used_initial_keys` 集合和
  `remaining_initial_slots()`；上限为 `min(MAX_INITIALS, config.max_initials)`，
  `MAX_INITIALS == 6`。
- 每轮调用 `solve_open_path_guided(..., excluded_initial_keys=..., max_new_initials=remaining_initial_slots())`。
  `make_initial_guesses` 仍是唯一初值来源；过滤发生在其输出之后、任何求解之前。
- `optimizer.py` 新增稳定身份函数 `initial_guess_key(guess)`：
  - 路由初值：`("map_route_guide", route_source, route_grid_sha256, route_index, label)`；
  - 端点形状初值：`("automatic_endpoint_shape", length_multiplier, shape_sign, label)`。
- `pipeline.py` 从真实 `attempts` 回吸本轮实际发起的初值身份：
  - `new_initial_count`：本轮首次出现、真正进入求解的 distinct key 数；
  - `initial_count_total`：整请求累计 distinct key 数；
  - `duplicate_initial_launch_count`：实际重复发起的 key 数（应为 0）；
  - `initial_keys`：整请求已用 key 列表。
- `initial_count` 保留历史语义：本轮交给 `solve_open_path_guided` 的构造候选数
  （不是全局证明）；实际是否进入求解由 `attempts` / `geometry_attempts` 证明。
- `solver=auto` 的 IPOPT→SLSQP 回退只增加 solver attempt，不增加 distinct initial；
  计数按初值 key，而不是按 attempt。
- 无未用初值或无剩余槽位时：不启动第二个 solver、不伪造 recovery，按原始失败原因
  fail closed；`reallocation` 记录 `attempted=false` 和具体 detail。
- 路由引导轮中，若路由初值已经通过独立几何审计，则在消耗端点形状族之前停止
  （`stop_after_route_audit`），把未用端点留给 recovery；若路由初值失败，仍会尝试
  全部端点形状，保持历史数值失败行为。该行为只影响调度，不改变求解模型。

### 负例与正向调度证据

- `test_numerical_failure_spends_one_endpoint_only_recovery`：
  第一轮 5 个初值，第二轮最多 1 个；实际 launch 6 次、identity 6 个；
  `initial_count_total==6`、`recovery_count==1`，无 path/plan。
- `test_solver_auto_fallback_is_one_initial_and_no_fake_recovery`：
  IPOPT+SLSQP 共 2 个 attempt，但只有 1 个 distinct initial；无恢复。
- `initial_pool_exhaustion` 重放（`max_initials=1`）：
  1 个 distinct initial、1 个 attempt、`recovery_count=0`，未生成重分配、未二次求解，
  无 path/plan。`reallocation.detail` 为
  `"no unused initial slot remains for a reallocated solve"`。

## 2. R2：固定 σ 相邻门重分配

### 实现

- `corridor.py` 新增唯一直接可测函数：

  ```python
  reallocate_adjacent_portal_once(path, chain, allocation, containment)
      -> tuple(new_allocation) | None
  ```

- 触发前置条件由 `pipeline.py` 执行：corridor 模式、硬连续包含失败、失败点/σ/
  corridor index/残差可用、真实相邻走廊半空间重叠可用、recovery 与 initial 槽位仍可用。
- 候选边界来自路径连续参数区间在两条相邻走廊原始半空间中的真实交集；使用直接
  半空间判据（不是稀疏碰撞近似），粗采样 513 点加 64 步二分定位连续重叠分量；
  要求区间宽度 ≥ 1e-6 m，严格位于重叠内部，生成一个确定性中间站点。
- 生成中点后再次对两走廊原始半空间做直接成员检查；若粗采样括号内隐藏窄的
  exit/re-entry gap，函数返回 `None`（fail closed），不会把间隙中的点当作边界。
- 只移动相邻走廊 j/j+1 之间的一个切换边界；其他边界、顺序、走廊数均不变。
- 候选必须通过 `validate_corridor_allocation`；失败返回 `None`，不会构造非法分配。
- `pipeline.py` 每个请求最多调用一次；成功后消耗唯一的 recovery，记录动作
  `adjacent_portal_reallocation`、旧/新边界 σ 与 s、相邻走廊、触发原因、失败 σ；
  下一轮复用同一 corridor chain/guide，只发起未用初值。第二次固定 σ 失败立即返回，
  不进行路由切换、局部避障或第二次恢复。
- 未改动 `min_width_m`、深度、区间数、margin、审计阈值、车辆参数、NLP 数学模型、
  碰撞/动力学/时间律审计或 ROS/launch/RViz。

## 3. 重放结果（逐项报告）

### 3.1 S1 corridor 固定 σ 失败（默认 `max_initials=6`）

证据文件：`replays/S1_corridor_reallocation.json`

- 结果：`GEOMETRY_FAILED` / `CORRIDOR_AUDIT_WIDTH_LIMIT`；无 path、无 plan、
  `executable=false`，`false_publications=0`。
- route count：**3**（≤3）。
- distinct initial count：**5**（`initial_limit=6`；不是用单轮 `initial_count` 证明）。
- solver attempts：**5** = 第一轮 1 个路由初值 + 第二轮 4 个端点形状初值；
  每轮 attempt 数与 distinct initial 数分别记录。
- duplicate initial launches：**0**。
- recovery count：**1**。
- 重分配动作：`adjacent_portal_reallocation`，相邻走廊 `[0, 1]`：
  - 旧边界 σ：`0.921342994303884`（s=`9.354375285372408`）；
  - 新边界 σ：`0.8856843077053926`（s=`8.992333419652475`）；
  - 只改动 entry 7/8 之间的一个边界，其余 entry 不变；新区间连续覆盖 `[0,1]`；
  - `validate_corridor_allocation` 通过后记录 `adjacent_portal_reallocation_applied`；
  - 触发时失败：σ=`0.7640891075134278`，s=`7.7577800095459395`，
    corridor index=`0`，点残差 `-4.893289648366661e-06`。
- 新分配后的第二次求解确实运行：第二轮 4 个 attempt，使用同一 corridor chain/guide
  和新的 allocation。
- 第二次失败：同一位置附近再次 `CORRIDOR_AUDIT_WIDTH_LIMIT`
  （s=`7.7577799663029445`，σ=`0.7640891075134277`，残差
  `-4.892450222282463e-06`），记录在
  `recoveries[0].second_failure`；立即返回，无第二次 recovery、无路由切换。
- geometry audit：**passed=true**（失败不在几何求解层，而在 corridor 连续包含宽度限）。
- continuous containment：**failed**，`CORRIDOR_AUDIT_WIDTH_LIMIT`，detail 为
  `interval width 9.682634438235027e-06 m cannot close the bound`。
- dynamics audit：**未运行（null）**；连续包含失败后未进入时间/动力学层。
- collision audit：**未运行（null）**；同上。
- publication eligibility：**不可发布**（无 plan；且本次证据完全不发布）。

结论：S1 上确实生成了一次合法的相邻门重分配，也确实用新分配重新求解；第二次仍在
同一固定 σ 宽度限制处失败，故按合同立即终止。没有通过放宽阈值或扩大走廊制造成功。

### 3.2 整请求初值耗尽负例（`max_initials=1`）

证据文件：`replays/initial_pool_exhaustion.json`

- 结果：`GEOMETRY_FAILED` / `CORRIDOR_AUDIT_WIDTH_LIMIT`；无 path、无 plan、
  `executable=false`。
- route count：3；distinct initial count：**1**；solver attempts：**1**；
- duplicate initial launches：0；recovery count：**0**；
- 重分配：`attempted=false`、`available=false`、
  `status=fixed_sigma_reallocation_unavailable`；
- detail：`no unused initial slot remains for a reallocated solve`；
- geometry audit true；continuous containment failed；dynamics/collision 未运行；
- publication eligibility：不可发布；`false_publications=0`。

该负例证明初值耗尽时不会启动第二次 solver，也不会伪造 recovery。

### 3.3 原有真实成功场景（S3，legacy route）

证据文件：`replays/S3_legacy_success.json`

- 结果：`SUCCESS`；path 长度 `12.093485960786976 m`；plan 存在；
  `executable=true`；`model_binding_ok=true`。
- route count：**1**；distinct initial count：**6**（`initial_limit=6`）；
- solver attempts：**6** = 1 个路由初值 + recovery 轮 5 个端点形状初值
  （其中两个 IPOPT `Maximum_Iterations_Exceeded`，但仍属于已发起的 distinct initial）；
- duplicate initial launches：**0**；recovery count：**1**，
  动作 `endpoint_only_initial`，原因 `COLLISION_NUMERICAL_FAILURE`；
- geometry audit：**passed=true**；
- continuous containment：**不适用**（该场景走 legacy route，不进入 corridor 包含门）；
- dynamics audit：**passed=true**；
- collision audit：**passed=true**；
- publication eligibility：**内存中可执行/可发布**，但本次证据未调用 ROS 发布；
  `false_publications=0` 表示没有发生发布调用，而不是发生过发布被拦截。

该场景证明 R1 的整请求 6 初值上限与既有 S3 成功路径兼容，且端点 recovery 未重跑
第一轮已经用过的端点形状。

## 4. 四组定向测试

完整日志在 `test_logs/`，时间与退出码在 `test_logs/run_meta.log`。

| 命令 | 结果 | 用时 |
|---|---|---|
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_recovery.py' -v` | 17 tests OK，exit 0 | 15.275 s |
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_optimizer.py' -v` | 24 tests OK，exit 0 | 6.048 s |
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v` | 29 tests OK，exit 0 | 64.500 s |
| `python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v` | 8 tests OK，exit 0 | 104.628 s |

回归覆盖：

- 单箱两侧、开放 dogleg 去重、超时无回退；
- OBB 一次 recovery、预算耗尽不启动 solver；
- C2-B clearance 世界坐标不变；
- 六个冻结 pipeline 场景全部 SUCCESS，并逐场景断言
  `initial_count_total <= min(6, max_initials)`、key 数量等于 total、
  duplicate=0、recovery<=1；
- 所有失败场景无 path/plan；
- 原 C3 数值失败测试不再以缺少身份唯一性的 `len(calls)==11` 作为断言，改为
  6 个真实 launch 且 6 个唯一 identity。

## 5. 变更范围与源码哈希

`source.diff` 只包含本次返工增量，涉及：

- `pipeline.py`（+239/-15）
- `optimizer.py`（+98/-2）
- `corridor.py`（+228/-1）
- `tests/test_virtual_a_corridor_recovery.py`（+521/-4）
- `tests/test_virtual_a_pipeline.py`（+11/-1）

`contracts.py` 与返工前快照逐字节相同，未进入 diff。完整前后 SHA-256 见
`source_hashes.txt`。未修改车辆参数、NLP 模型、碰撞/动力学/时间律审计、ROS、
launch、RViz。

## 6. 原始 C3 证据完整性

- 原目录：`evaluation/virtual_a_corridor/C3/20261117T155212Z/`
- `manifest.sha256` 自哈希：
  `bcce46dfd8ce9da9ce46ae18ca54f104a6008e2a670bcd5d3ab6815096a1a466`
- 目录内 26 个 manifest 条目全部校验通过，`sha256sum -c` exit 0；
- 28 个文件的排序哈希聚合摘要：
  `bb5558d4eda1083b07349d81f508eae483a982c283f25c9b2ae0214b2ee94640`；
- 详见 `original_c3_manifest_verification.log` 和 `original_c3_integrity.log`。
- 本返工只读取该目录，未写入、未覆盖、未移动。

## 7. 验收对照

`results.json` 中 `acceptance` 全部为 true：

- distinct initials ≤ `min(config.max_initials, 6)`；
- duplicate initial launches = 0；
- recovery ≤ 1；
- solver=auto 回退计数为 1 个 initial；
- route ≤ 3；
- S1 重分配只移动一个相邻边界，新分配连续覆盖 `[0,1]` 且通过严格校验；
- 未放宽任何容差/车辆参数；
- 不可执行场景均 `path is None`、`plan is None`；
- `false_publications = 0`；
- 四组定向测试均 exit 0。

## 8. 未运行与限制

- ROS、RViz、发布、真实车辆：**NOT_RUN**；本次仅为纯 Python 证据。
- C4/C5：**NOT_RUN**；按合同在 C3_REWORK_01 证据完成后停止。
- 未 commit、未 push、未创建分支。
- S1 仍是失败场景；本证据只证明失败路径符合合同（合法重分配、真实二次求解、
  第二次失败立即终止），不声称 S1 必须成功。
- 本报告不声称 Codex ACCEPTED；等待外部审核。