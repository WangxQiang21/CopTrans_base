# TASK02 Step D 证据：Route 尝试顺序与有界 Guide 候选进入真实求解

生成时间：2026-11-19（UTC）。目录：`work_flow/TASK02_evidence/20261119T054527Z/candidate/step_d/`。
本目录只记录 Step D 的实现、测试与真实调用证据；不是 Codex 审核结论，也不宣称 TASK01/TASK02 验收通过。TASK01 dirty 的 `geometry.py`、`tests/test_virtual_a_geometry.py` 未被修改（见 `git_status_porcelain.txt`）。

## 1. 实现范围

### 1.1 `corridor.py`：有界候选生成

- `corridor_guide_candidates(chain, start, goal, base_guide=None)`：
  - 首个候选始终是 base Guide：调用方传入的 `base_guide`（pipeline 已用真实 `make_corridor_guide()` 构造一次）或函数内部新构造的同一个 C2-A Guide。
  - 只有 base 点列满足 `start + 每个内部 Portal 一个点 + goal`（`len(points) == region_count + 1`）且 `region_count > 1` 时才生成变体；任意注入的 base Guide 不会派生无意义 Portal 变体。
  - 变体 = 全 Chebyshev 中心折线（`CHEBYSHEV_CENTRE_GUIDE`，最多 1 个）+ 单 Portal 变体（`ONE_PORTAL_VARIANT_{j}`，最多 3 个）。
  - 每个变体经 `_guide_region_violation()` 检查仍在该分配走廊的半空间内；零长度段、越界变体被丢弃，绝不放松。
  - 硬上限 `CORRIDOR_GUIDE_CANDIDATE_LIMIT=5`、`CORRIDOR_GUIDE_PORTAL_VARIANT_LIMIT=3`。
- 变体的 `quality` 保留 C2-A 字段，并写入 `candidate_origin` 与 `candidate_certification=DIAGNOSTIC_ONLY`；不产生可执行路径或连续认证声明。

### 1.2 `pipeline.py`：Route 尝试顺序

- A* 成功后，≤3 条 route 的真实粗评估写入 `search['corridor']['route_assessments']`；随后计算
  `route_attempt_order = stable sort(index, key=_route_assessment_score)`。
- `_route_assessment_score` 固定字典序：
  `(map_status_rank, collision_count, unconfirmed_count, route_length_m, original_route_index)`；
  `CLEAR_AT_SAMPLES < COLLISION_OBSERVED < UNCONFIRMED/缺失`，缺失评估永不视为 clear。
- `route_attempt_order` 只排序**原始 A* index**；`routes` 元组、A* topic ID、active route alias 全部保持原始编号。
- 起始 `route_index = route_attempt_order[0]`；`untried_route_index()` 按该顺序返回尚未尝试的原始 index，供既有单次 route recovery 使用。
- `search['corridor']['route_attempt_order']` 记录完整顺序；非 corridor 模式的顺序退化为原始 `0..n-1`，行为不变。
- 重新生成路线（collision local block 后）时同时清空 `simplified_by_route` 与 `route_attempt_order`，避免旧路线的简化缓存被新路线复用。

### 1.3 `pipeline.py`：Guide 候选、分配、评估与选择

- 在 chain audit 通过后，用真实 `make_corridor_guide()` 构造 base，再调用 `corridor_guide_candidates()`；候选数 ≤5。
- 每个候选先独立执行 `allocate_corridors()`：
  - 成功：记录该候选自己的 allocation，并对该候选执行真实 `assess_formation_guide()`（同一 checker、model、shapes、grid、request/context identity、共享 soft deadline）。
  - `allocate_corridors()` 抛 `VirtualAContractError`：记录 `status=ALLOCATION_INCOMPATIBLE`、`reason=FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE`、有界 detail；不进入评估、不进入 solver。
  - checker 不可用 / 评估异常：记录 `ASSESSMENT_UNAVAILABLE` 或 UNKNOWN 诊断，不阻塞。
- 选择只在 solver-compatible 候选中进行，固定字典序：
  `(assessment_available, has_collision, collision_count, has_invalid, invalid_count, gap_count, coverage_unknown_count, map_unknown_count, guide_rank, length_m, candidate_index)`；
  `assessment_available=0` 优先；不可用候选排在所有可用候选之后。`guide_rank` 依次为 base QP `0`、Chebyshev `1`、单 Portal 变体 `2`。
- 选中候选的 **Guide 点列、该候选自己的 allocation、原始 chain** 一起传给 `solve_open_path_guided()`；solver 仍收到 `corridor_chain`，hard 约束不因候选变化而移除。
- `corridor_round` 新增：
  - `guide_candidates`：≤5 条有界记录（序号、source、origin、长度、点/分配数、兼容性、状态、`selection_counts`、是否选中、认证声明固定为 `NONE`/`DIAGNOSTIC_ONLY`）。
  - `guide_selection`：候选数、兼容数、选中序号/source/origin、选择规则、base 是否兼容，以及固定 `certification_claim=NONE`、`continuously_certified=false`、`formation_obb_hard_constraints=false`。
- `PORTAL_GUIDE` progress 增加有界候选数、候选 source 列表、选中序号/origin、选择规则与选中候选的 assessment 摘要。
- 选择规则字符串：`SINGLE_BASE_GUIDE`（仅 1 个候选）、`DIAGNOSTIC_LEXICOGRAPHIC_BEST`（选中候选有可用评估）、`ASSESSMENT_UNAVAILABLE_BASE_FALLBACK`（无任何可用评估，按 rank/length/index 回退到 base）。

## 2. 语义与边界

| 项目 | 语义 |
|---|---|
| Route 排序 | 仅影响首次尝试与 recovery 的下一个**未尝试原始 index**；不重排 `routes`，不修改 A* topic/alias |
| Guide 选择 | 只使用有限静态采样的诊断计数；不产生连续认证 |
| 不可用评估 | 排在可用评估之后；全部不可用时回退 base Guide，仍调用真实 solver |
| 分配不兼容 | 记录 `FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE` 并拒绝该候选；base 不可用则保持既有 `CORRIDOR_ALLOCATION_FAILED` 失败 |
| 硬约束 | 任何候选路径都保留原始 chain + allocation；solver 必须收到 `corridor_chain` |
| 证据上限 | 候选 ≤5/route、单 Portal 变体 ≤3、候选记录字段为有界摘要 |

## 3. 测试

命令（仓库根目录，source ROS 后）：

```bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 -m unittest tests.test_virtual_a_formation_assessment -v
python3 -m unittest tests.test_virtual_a_corridor_recovery -v
python3 -m unittest tests.test_virtual_a_corridor_optimizer -v
python3 -m unittest tests.test_virtual_a_pipeline -v
```

| 日志 | 结果 | 耗时 | exit |
|---|---|---|---|
| `tests_formation_assessment.log` | Ran 32 tests, OK | 0.245 s | 0 |
| `tests_corridor_recovery.log` | Ran 29 tests, OK | 18.207 s | 0 |
| `tests_corridor_optimizer.log` | Ran 27 tests, OK | 9.322 s | 0 |
| `tests_pipeline.log` | Ran 13 tests, OK | 119.670 s | 0 |

`tests_pipeline.log` 在追加 solver/候选/assessment 同源 identity 断言后单独重跑；`tests_formation_assessment.log`、`tests_corridor_recovery.log`、`tests_corridor_optimizer.log` 来自同一最终源码的批次运行。四者 exit 均为 0。

`tests/test_virtual_a_pipeline.py::RouteAndGuideSelectionTests` 覆盖：

1. `test_route_attempt_order_and_guide_selection_reach_the_solver`：
   - 注入两条真实自由中心线 route（原始顺序 below, above），真实 checker 粗评估使
     `route_attempt_order=[1,0]`；
   - solver spy 记录两次真实调度：第一次 route 1（条目 `route_above`）选中 Chebyshev 变体（采样碰撞 23→16），第二次 route 0（`route_below`）选中 base QP（采样碰撞 25<38）；
   - 断言 solver 收到的 Guide 点列 == 对应 round 的选中候选点列、`corridor_chain.digest` == round chain digest、allocation 一致、`stop_after_route_audit=True`；
   - 断言 solver 点列/chain/allocation 与选中候选记录、round `formation_assessment` 的 `candidate_id`/`selected_guide_source`/`legacy_f_allocation` 完全同源；
   - 断言 `search['routes']` 仍为注入顺序，原始 index 未被排名污染。
2. `test_unavailable_diagnostics_keep_the_base_guide_and_solver`：
   - 真实 `FormationCollisionChecker` 构造失败 → route/Guide 诊断全 UNKNOWN；
   - 仍完成真实求解并 `SUCCESS`；base 候选被选中、`SINGLE_BASE_GUIDE`，无认证声明。
3. `test_incompatible_variant_is_rejected_with_hard_corridor_kept`：
   - 注入越界变体；`allocate_corridors()` 拒绝并记录 `FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE`；
   - base QP 被选为唯一兼容候选；solver 收到 base 点列、原始 chain 与 base allocation；最终 `SUCCESS`。

既有 `test_corridor_round_records_bounded_diagnostic_assessments` 追加 route 顺序、候选记录、选择 identity 与认证常量断言。

## 4. 真实调用结果

`selection_results.json`（完整 JSON，含 route/Guide 诊断与 solver spy 参数；已用最终源码重新生成，`started_utc=2026-11-19T07:06:00Z`、`finished_utc=2026-11-19T07:06:55Z`，对应 `source_hashes.txt` 中的最终文件哈希）：

### 4.1 空地图 corridor 成功请求

- `SUCCESS`；route 数 1，`route_attempt_order=[0]`，1 个候选、选择规则 `SINGLE_BASE_GUIDE`。
- Guide 21 stations：`CLEAR_AT_SAMPLES`（21/21 clear）、`SINGLE_REGION`、`VALID_HINT`；候选计数全 0。
- `corridor_guide_s=6.1e-05`、`corridor_allocation_s=0.0058`、`formation_assessment_s=0.0134`。

### 4.2 冻结 S1 corridor/hard

- 仍为 TASK01 既有 hard 失败：`GEOMETRY_FAILED / CORRIDOR_AUDIT_WIDTH_LIMIT`；未降低阈值、未改审核。
- 真实粗评估 3 条 route 均为 `COLLISION_OBSERVED`（各 8 个采样碰撞），长度决定 `route_attempt_order=[1,0,2]`，首次即尝试原始 route 1。
- round 0（原始 route 1）：base QP 42 个采样碰撞，Chebyshev 25；选中 Chebyshev（`DIAGNOSTIC_LEXICOGRAPHIC_BEST`），其 59 stations 为 `COLLISION_OBSERVED`、覆盖 `GAP`、1 个 Portal 窗口；连续 containment 通过。
- round 1（route recovery 到原始 route 0）：base 36，Chebyshev 33；选中 Chebyshev；该 round 连续 containment 以 `CORRIDOR_AUDIT_WIDTH_LIMIT` 失败，与 Step D 前的既有失败结论一致；route 2 因 recovery 已消耗未再尝试。
- `formation_assessment_s=0.937`；诊断字段保留在 `search['corridor']`，不改变失败结论。

### 4.3 合成双门 route 顺序演示

- 两条真实自由中心线由 `make_route_guides` 注入，checker/assess 全程真实；仅 solver 被 spy 替换为固定的 `ALL_ATTEMPTS_NUMERICAL_FAILURE`，使两次 route 尝试都可见。
- `route_attempt_order=[1,0]`；round 0（route 1）选中 Chebyshev（16<23），round 1（route 0）选中 base QP；两次 solver 调用的 Guide 点列、chain digest、allocation 均与对应 round 完全一致。
- 仅用于展示调度顺序，不作为真实场景的成功/失败结论。

## 5. 文件清单

| 文件 | 内容 |
|---|---|
| `step_d.diff` | `corridor.py`、`pipeline.py`、`tests/test_virtual_a_pipeline.py` 相对 HEAD 的累计 diff（含 Step C；不含 TASK01 dirty geometry/test） |
| `source_hashes.txt` | 上述三个文件的 SHA-256 |
| `git_status_porcelain.txt` | 生成证据时的工作区状态；TASK01 dirty 两项仍在 |
| `tests_formation_assessment.log` / `tests_corridor_recovery.log` / `tests_corridor_optimizer.log` / `tests_pipeline.log` | 各模块原始 stdout/stderr |
| `tests_*.exit` | 各模块 exit code（均为 0） |
| `selection_results.json` | 空地图、S1、双门演示的 route 顺序、候选记录、选择、solver spy 参数与 timings |
| `step_d_tests_started_utc.txt` / `step_d_tests_finished_utc.txt` | 测试命令起止时间 |

## 6. 边界与未完成项

- 本目录不包含 Step E（HUD/RViz 诊断图层）与 Step F（完整回归、catkin 构建、ROS/RViz 实跑）；这些在后续内部步骤完成。
- S1 仍按既有结论 hard 失败；Step D 只证明候选选择真实进入 solver 且不放松走廊/审核，不证明 S1 可通过。
- 所有选择计数来自有限静态采样；`candidate_certification=DIAGNOSTIC_ONLY`、`continuously_certified=false`、`formation_obb_hard_constraints=false` 始终成立。
- `step_d.diff` 为累计 diff；Step C 的独立快照仍保存在 `../step_c/step_c.diff`。