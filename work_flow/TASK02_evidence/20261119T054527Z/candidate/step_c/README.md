# TASK02 Step C 证据：三车姿态、地图静态样本与 Portal 过渡数据

生成时间：2026-11-19（UTC）。目录：`work_flow/TASK02_evidence/20261119T054527Z/candidate/step_c/`。
本目录只记录 Step C 的实现、测试与真实调用证据；不是 Codex 审核结论，也不宣称 TASK01/TASK02 验收通过。TASK01 dirty 的 `geometry.py`、`tests/test_virtual_a_geometry.py` 未被修改（`source_hashes.txt` 之外的 `git_status_porcelain.txt` 仍显示这两项）。

## 1. 实现范围

### 1.1 `corridor.py`（Step B 支持函数 + Step C 评估）

- 复用 `map_spatial_state()` 与 `FormationCollisionChecker.static_check()`，没有新增独立 raster/SAT 判定。
- `formation_transition_windows()`：Portal 窗口
  `s_j`、`σ_j=s_j/L_g`、`ρ=max_i(||r_i||+0.5*sqrt(L_i²+W_i²))`、
  `W_j=[max(0,(s_j-ρ)/L_g), min(1,(s_j+ρ)/L_g)]`；窗口端点、σ 点、窗口内 Guide 顶点和 ≤0.10 m 均匀点；缺 shape → `MISSING_VEHICLE_SHAPE` 且窗口不可确认。
- `assess_formation_guide()`：≤64 unique stations；端点、折线转角、窗口必需点、全局均匀点；required-first 截断；≤2 次窗口内二分。每站同时输出：
  - `map`：真实 `static_check()` 的 clear / `MAP_OBB_COLLISION` / `VEHICLE_OBB_COLLISION` / unconfirmed；
  - `coverage`：每车单区 + `C_j∪C_{j+1}` 两顺序完整 OBB 覆盖；
  - `normal_branch`：`map_spatial_state()` 的 `A_i>0, D_i>0` 与 `NORMAL_BRANCH_VIOLATION`。
- `assess_route_guide()`：≤32 stations；`coverage_sample_status` 固定 `UNCONFIRMED_NUMERICS`（`NO_CORRIDOR`），不提前声称 route chain 已构造。
- 所有结果固定 `candidate_certification=DIAGNOSTIC_ONLY`、`continuously_certified=false`、`formation_obb_hard_constraints=false`；`η` 未连续估计时车辆曲率始终 `CURVATURE_HINT_UNCONFIRMED`。
- `candidate_id` 由 schema + request/grid/context SHA-256 + route/round + chain digest + guide source + 点列 digest 规范化哈希得到，可复算。

### 1.2 `pipeline.py`（Step C 接线）

- A* 成功后、第一次 solver 调用前，用真实 `request.model`、`request.vehicle_shapes`、grid、`collision_config.baseline_margin_m` 构造一次 `FormationCollisionChecker`。
- 每条 route 只 `simplify_route()` 一次并缓存到 `simplified_by_route[original_index]`；粗评估与后续 `build_corridor_chain()` 共用该结果。
- `search['corridor']['route_assessments']` 记录 ≤3 条原始 route 索引的粗评估；失败/不可用记录 `map_sample_status="UNKNOWN"`，不阻塞规划。
- 已 audit 的 chain + `make_corridor_guide()` + `allocate_corridors()` 之后，`assess_formation_guide()` 结果写入对应 `corridor_round['formation_assessment']`；`PORTAL_GUIDE` progress 增加有界 `assessment` 摘要（样例数、碰撞数、union covered/gap/unknown、window 数、状态、`candidate_id`、`sample_cap`）。
- 评估异常被捕获为 `FORMATION_ASSESSMENT_UNAVAILABLE`/`CHECKER_UNAVAILABLE` 的 UNKNOWN 诊断，不改变 solver 调用、审核、预算或发布路径。

## 2. 状态语义

| 字段 | 取值 | 语义 |
|---|---|---|
| `map_sample_status` | `CLEAR_AT_SAMPLES` | 所有已采样 OBB 上真实 `static_check()` 通过 |
| | `COLLISION_OBSERVED` | 至少一个采样点观测到 `MAP_OBB_COLLISION` 或 `VEHICLE_OBB_COLLISION`，保留 vehicle ID/other ID/站点/reason |
| | `UNCONFIRMED` | 缺 shape、姿态 hint 不可用、正常分支违反、checker 异常、采样上限或超时；不伪造 clear |
| `coverage_sample_status` | `SINGLE_REGION` / `COVERED_BY_ADJACENT_UNION` | 所有站点的完整 OBB 均由单区或窗口内 union 覆盖 |
| | `GAP` | 所有采样均可数值判定且至少一车严格存在并集缺口；不是物理不可行证书 |
| | `UNCONFIRMED_NUMERICS` | 数值不明、无 chain（route 粗评估）、缺 shape、采样上限、退化 Guide 或超时 |
| `kinematic_hint_status` | `NORMAL_BRANCH_OK` / `NORMAL_BRANCH_INVALID` / `CURVATURE_HINT_UNCONFIRMED` | 正常分支逐站检查；`η` 未估计时曲率始终 unknown |
| `guide_hint_status` | `VALID_HINT` / `LOW_CONFIDENCE` / `DEGENERATE` | Guide 转角/曲率 hint 的可信度，仅诊断 |

采样上限/退化 Guide/缺 shape/超时时：coverage 与 transition window 状态降为 UNKNOWN，已观测的碰撞计数与 gap 计数仍保留；`map` 的碰撞观测不被降级，但也不能变成 clear。

## 3. 测试

命令（仓库根目录，source ROS 后）：

```bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 -m unittest tests.test_virtual_a_formation_assessment -v
python3 -m unittest tests.test_virtual_a_pipeline -v
```

| 日志 | 结果 | 耗时 | exit |
|---|---|---|---|
| `tests_formation_assessment.log` | Ran 32 tests, OK | 0.260 s | 0 |
| `tests_pipeline.log` | Ran 10 tests, OK | 112.883 s | 0 |

`tests/test_virtual_a_formation_assessment.py` 覆盖：支持函数对照（≤1e-12 m）、单区/相邻 union 正反顺序、diamond gap、窗口公式与钳制、缺 shape、`MAP_OBB_COLLISION` 与 `VEHICLE_OBB_COLLISION` 分离、map clear + union gap 并存、`NORMAL_BRANCH_INVALID` 不伪造 OBB、sample cap → window UNCONFIRMED + `SAMPLE_CAP`、退化 Guide、identity 确定性、route 粗评估上限与 `UNCONFIRMED_NUMERICS`。

`tests/test_virtual_a_pipeline.py::test_corridor_round_records_bounded_diagnostic_assessments` 验证真实 corridor 成功运行中：route assessments 与 round `formation_assessment` 的 request/grid/context/chain identity、`legacy_f_allocation == allocation`、station 上限、诊断常量，且 solver 仍成功。

## 4. 真实调用结果

`assessment_results.json`（完整 JSON，含 route/round 诊断）：

### 4.1 空地图 corridor 成功请求

- `SUCCESS`，`request_sha256`/`grid_sha256`/`planning_context_sha256` 均来自本次真实 request。
- route 0 粗评估：11 stations，`CLEAR_AT_SAMPLES`，coverage `UNCONFIRMED_NUMERICS`（`NO_CORRIDOR`），`VALID_HINT`。
- round 0 Guide 评估：21 stations，`CLEAR_AT_SAMPLES`（21/21 clear），`SINGLE_REGION`，`NORMAL_BRANCH_OK`，`CURVATURE_HINT_UNCONFIRMED`，无 transition window。
- `formation_assessment_s=0.0136`。

### 4.2 冻结 S1 corridor 模式

- 仍为 TASK01 既有 hard 失败：`GEOMETRY_FAILED / CORRIDOR_AUDIT_WIDTH_LIMIT`；未降低阈值、未改审核。
- 三条 route 粗评估均为 12 stations，`COLLISION_OBSERVED`（各 8 个 map collision），coverage `UNCONFIRMED_NUMERICS`。
- round 0 Guide 评估：57 stations，`COLLISION_OBSERVED`（36/57 map collision），coverage `GAP`，1 个 `PORTAL_TRANSITION_GAP` 窗口；`NORMAL_BRANCH_OK` 与 map collision 两个字段独立呈现。
- `formation_assessment_s=0.4024`；失败前后 `search['corridor']` 保留本 round 诊断，不改变失败结论。

## 5. 文件清单

| 文件 | 内容 |
|---|---|
| `step_c.diff` | `corridor.py`、`pipeline.py`、`tests/test_virtual_a_pipeline.py` 相对 HEAD 的完整 diff（不含 TASK01 dirty geometry/test） |
| `source_hashes.txt` | 上述文件与新增评估测试的 SHA-256 |
| `git_status_porcelain.txt` | 生成证据时的工作区状态；TASK01 dirty 两项仍在 |
| `tests_formation_assessment.log` / `tests_pipeline.log` | 原始 stdout/stderr 与 exit code |
| `assessment_results.json` | 两个真实 pipeline 运行的 identity、route/round assessment |
| `*_started_utc.txt` / `*_finished_utc.txt` | 命令起止时间 |

## 6. 边界与未完成项

- 本目录不包含 Step D（route/Guide 选择进入 solver）、Step E（HUD/RViz）和 Step F（回归/ROS 实跑）；这些在后续内部步骤完成。
- S1 的 corridor hard 失败按既有结论保留；本步骤只新增诊断字段，未证明 S1 可通过。
- 评估是有限静态样本诊断，不是连续认证；`continuously_certified=false` 永远成立。