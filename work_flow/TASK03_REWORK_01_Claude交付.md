# TASK03 返工 01 Claude 交付报告

任务：执行 `work_flow/TASK03_Codex返工执行方案.md`
工作树：`/home/wq/whut_thesis/expv3_multi_virtual_a`（git worktree，未切分支、未新建 worktree）
HEAD：`a3c709e57560b1060dcc422777f4131a3e558acf`，branch `feature/virtual-rigid-formation-a`
证据根目录：`work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/`（UTC 开始时间戳）
状态：**修复与证据收集完成，停止，等待 Codex 独立验收**。E09 依据方案 line 129 记为
`NOT_REPRODUCIBLE` 并已停止该步骤；E09 未通过、未伪造。

## 0. 结论摘要

| 项 | 状态 | 关键结果 |
|---|---|---|
| P0：heading increment 区间漏掉零增量 | **DONE** | `bounds.imul(kappa, (0.0, width))` 外舍入区间；修复前反例 2/3 失败、修复后全绿；`formation_corridor` 64 tests OK |
| P1-B：模板每段 F region 归属 | **DONE** | schema `formation_corridor_template.v2`，segment 强制 `f_region_index`；v1 fail-closed；payload 往返与 binding 重建通过 |
| P1-C：union NLP / certificate 使用模板 F 归属 | **DONE** | union rows 由 template 编译；numeric/CasADi/SciPy 同一决策向量最大差 ≤1e-8；四类测试通过；F rows 324、OBB rows 3888、实际 NLP `g` 总数 4486 |
| P1-D E04 拆分 | **DONE（拆分口径）** | live 负例仅证明 fail-closed/无旧轨迹；离线 solver probe 证明 OBB rows 真实存在且 F 可行 + OBB 正残差不可接受 |
| P1-D E09 | **NOT_REPRODUCIBLE** | 14+2 个确定性场景均未产生“证书 VERIFIED 且旧 F-only audit 失败”的路径；结构性阻碍与日志见 E09 报告 |
| 全量回归 | **PASS** | 7 个 Python 套件 357 tests OK；catkin_make、`bash -n`、dry-run 通过 |
| live ROS/RViz | **PASS** | union 正例 29/29 checks；失败清屏负例 22/22 checks；v2 binding 由 receiver 重建并验证 |
| `f_only` 兼容 | **PASS** | 冻结 request/context/grid hash 重建一致；默认 scope 未变；历史序列化未变 |
| 受保护文件/目录 | **PASS** | TASK01 两文件 SHA、`evaluation/`、TASK01/TASK02 证据树结束状态与开始完全一致 |
| Git 提交/推送/TASK04 | **未执行**（按要求） | 不 commit、不 push、不开始 TASK04 |

## 1. 工作树与 Git 状态

- 开始：`start/git_identity.txt`，UTC `2026-11-19T18:07:35Z`，HEAD `a3c709e5…`，status 29 entries。
- 结束：`end/git_identity.txt`（本报告写完后重跑，见 §10），HEAD/branch/远端不变。
- 本返工是在 TASK03 未提交工作区上的增量修复，**不是**从干净 HEAD 开始；开始快照
  `start/source_snapshot.tar` 与 `start/task03_sources.txt` 保存了 19 个 TASK03 源文件的原状，
  返工增量 diff 以此快照为基准，而不是以 HEAD 为基准。

## 2. P0 修复：heading increment 外包络（Step A）

缺陷：`_heading_interval()` 使用 `kappa * width` 作为整个区间的航向增量，等价于假定区间内
曲率为零；同号曲率时漏掉零增量，负曲率时漏掉起点增量，导致证书可能把真实越界判为
`VERIFIED`。

修复（[formation_certificate.py:206](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_certificate.py#L206)）：

```python
psi_interval = _inflate(
    _heading_interval(qf.psi, kappa, width),
    float(angle_error_rad) * 2.0)
```

`_heading_interval()` 现在对曲率区间做**区间乘法**：
`bounds.iadd((psi, psi), bounds.imul((k_lo, k_hi), (0.0, width)))`，即增量区间为
`[min(k_lo,k_hi)*0, max(k_lo,k_hi)*width]` 或反向，显式包含零增量；随后仍按
`2*angle_error_rad` 加宽。未修改 `bounds.py`，复用 `bounds.imul()`。

测试与证据：

- 新增 `FormationHeadingIntervalReworkTests`（[tests/test_virtual_a_formation_corridor.py:2845](tests/test_virtual_a_formation_corridor.py#L2845)）：
  正曲率、负曲率、跨零曲率、稠密部分积分 oracle、旧标量宽度反例、侧墙峰值/正间隙证书用例。
- 修复前反例日志：
  `tests/step_a_heading_interval_20261119T181431Z.log`（2 failures）、
  `tests/step_a_heading_old_interval_counterexample_20261119T181520Z.log`（3 failures）。
- 修复后：`tests/step_a_full_formation_corridor.log` 57 tests OK（当时套件规模）；
  后续加入 P1 测试后为 64 tests OK（`mandatory/test_virtual_a_formation_corridor.log`）。
- 证书判据未放宽：`VERIFIED` 仍要求闭合 interval 的 outward upper residual ≤0 且 unclosed=0；
  `VIOLATED` 需带 error budget 后仍为正的 witness；其余 `UNCONFIRMED`。

## 3. P1-B：模板逐段 F region 归属（Step B）

### 3.1 数据结构

- `FormationCorridorTemplateSegment`（[formation_corridor.py:220](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L220) 附近）
  新增**强制**字段 `f_region_index`；payload segment key 集合精确为
  `{sigma_start, sigma_end, f_region_index, covers}`。
- schema/compiler 版本：
  `FORMATION_CORRIDOR_TEMPLATE_SCHEMA = "formation_corridor_template.v2"`、
  `FORMATION_CORRIDOR_COMPILER_VERSION = "formation_corridor_rows.v2"`
  （[contracts.py:98-99](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py#L98-L99)）；
  certificate 仍为 `formation_corridor_certificate.v1`。
- `FormationCorridorFSample`（[formation_corridor.py:453](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L453)）：
  `piece_index, q, sigma, source, template_segment_index, region_index, halfspaces`。
- `FormationCorridorConstraint` 新增 `f_groups, f_sample_count, f_scalar_row_count,
  segment_f_row_counts, f_rows_by_segment, f_region_by_segment`，
  并提供 `formation_reference_constraint_count()` 与 `f_row_references()`
  （[formation_corridor.py:498-510](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L498-L510)）。

### 3.2 模板构建

- `_station_f_options()`（[formation_corridor.py:1624](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L1624)）：
  候选 = allocation ±1 相邻 region ∩ 其向外 inset 包含可靠 guide F 点的 region。
- `_f_choice()`：allocation / forward / backward 三种确定性选择。
- `_f_transition_cut()`：在 guide 上按确定性顺序（先 0.5，再 1/65…64/65）搜索同时位于两个
  F region 的点，作为段切点。
- `build_formation_template_candidates()`（[formation_corridor.py:1709](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L1709)）：
  > 段切点必须包含每条 allocation/Portal cut 与两个端点，并为每个 base interval 加中点
  > witness；`greedy_runs` 在车辆公共可行集或 F 归属变化处切段；最多 3 个确定性 profile
  > （allocation/forward/backward），按 profile rank 排序，预算未变。
- `validate_formation_template()`（[formation_corridor.py:292](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L292)）：
  segment 的 F region 必须存在、是 `ConvexCorridor`，且段中点处等于 allocation region 或与其
  直接相邻（`abs(f_region - base) <= 1`）；否则 `INVALID_FORMATION_CORRIDOR_TEMPLATE`。
  非相邻跳转、缺段、倒序、越界均拒绝。
- `formation_template_from_payload()`（[formation_corridor.py:2044](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_corridor.py#L2044)）：
  重建全部 identity 字段与 `template_id`；v1 payload（无 `f_region_index`）fail-closed。

### 3.3 测试

`FormationFRegionReworkTests`（[tests/test_virtual_a_formation_corridor.py:2991](tests/test_virtual_a_formation_corridor.py#L2991) 附近），6 tests：

- `test_f_profiles_adjacent_and_two_sided_on_audited_portal_chain`
- `test_template_validation_rejects_non_adjacent_and_unknown_f_region`
- `test_union_rows_compile_from_template_f_region_not_allocation`
- `test_f_rows_shared_by_numeric_casadi_and_scipy_fallback`
- `test_f_region_is_sealed_in_payload_and_rebuilt_by_verifier`
- `test_f_only_fixed_allocation_rows_remain_and_union_omits_them`

日志：`step_bc/formation_f_region_rework.log`（6 tests OK，2.11 s，
`step_bc/formation_f_region_rework.meta` exit=0）。

## 4. P1-C：union NLP 与 continuous certificate（Step C）

### 4.1 编译与数值链

- `build_formation_corridor_constraint()` 在 union scope 下按 `template.segments[].f_region_index`
  编译 F rows，不再使用 `corridor_allocation`；f_only 仍使用固定 allocation rows，union 不编译
  这些旧 rows（测试 `test_union_rows_compile_from_template_f_region_not_allocation` 证明：
  forward profile 下模板 F region=1 时 allocation 仍为 0）。
- 数值向量顺序：历史 corridor rows → template F rows → template OBB rows；
  `_constraint_value_offsets()` 返回 `(equality_end, signed_end, corridor_end)`，
  `corridor_end` 已含 F+OBB（[optimizer.py:1167](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py#L1167)）。
- optimizer 适配器（[optimizer.py:612](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py#L612)）新增
  `formation_f_constraint_count`，`total_halfspace_constraint_count = halfspace + F + OBB`。
- 接受前逐候选检查：
  `_formation_f_residual_diagnostic()`（[optimizer.py:1452](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py#L1452)）
  与 `_formation_obb_residual_diagnostic()`；F 或 OBB 最大硬 residual >1e-8 m 即拒绝
  （`FORMATION_F_RESIDUAL` / `FORMATION_OBB_RESIDUAL`），不进入 audit/publish。
- 同一固定决策向量下 numeric/CasADi/SciPy 的 F rows 最大差要求 ≤1e-8 m：
  `test_f_rows_shared_by_numeric_casadi_and_scipy_fallback`。

### 4.2 实测 rows 与 `g` 总数（方案 §6 要求）

证据脚本与输出（本返工新增只读探针，未改生产逻辑、阈值或预算）：

- `step_bc/union_pipeline_probe.py|.out|.meta`：运行与
  `G4PipelineTests::test_union_scope_certifies_the_f_obb_path_end_to_end`
  （[tests/test_virtual_a_pipeline.py:428](tests/test_virtual_a_pipeline.py#L428)）完全相同的
  empty-grid 直线 union 场景，结果为：

| 量 | 值 |
|---|---|
| status / executable / scope | `SUCCESS` / `true` / `three_vehicle_union_v1` |
| union F rows（`formation_f_constraint_count`） | **324** |
| OBB rows（`formation_obb_constraint_count`，仅 OBB） | **3888**（1296×3 车） |
| 两者合计 halfspace rows | 4212 |
| template / compiler / certificate | `formation_corridor_template.v2` / `formation_corridor_rows.v2` / `formation_corridor_certificate.v1` |
| segments | `[(0.0, 1.0, f_region_index=0)]` |
| allocation | 8 个区间全部 region 0 |
| template_id | `569150fcd22548799c54e3ada83d563271cfa404ee42e28b4d39efcefa0b9938` |
| chain_digest | `8945220f0e9521e77909af30e961fee4c7cede21333f53ef31da098432b07538` |
| certificate | `VERIFIED`，max_outward_residual `−0.3749899971639638`，unclosed 0，16/24 intervals |
| request / grid / context / path | `259b077d…` / `fd4a7d36…` / `354b9f86…` / `e603ac29…`，path_length 6.0 m |

- `step_bc/union_g_total_probe.py|.out|.meta`：用 `unittest.mock` 仅**观测**
  `optimizer._constraint_value_offsets`（始终调用原函数）记录求解器真实使用的切片：

| 切片 | rows |
|---|---|
| equality_end | 10 |
| signed_end（含 signed dynamics） | 274 |
| corridor_end（= equality + signed + F + OBB，即实际 NLP `g` 总数） | **4486** |
| 其中 F | 324 |
| 其中 OBB | 3888 |

`g_total_last_call = 4486`，`accepted_attempt_slice` 同上；attempts 仅 1 个：
`(0, ipopt, Solve_Succeeded, SOLVER_SUCCESS_AUDITED)`。该证据与 E04 离线 probe 的
`vector_size=4486`、`corridor_end_expected=4486` 相互一致（两者同为 1 region、8 intervals、
3 vehicles 的模板）。

### 4.3 continuous certificate

- `_heading_interval()` 修复见 §2。
- 证书 F 检查改为按 `template.segments[].f_region_index` 选择 region
  （[formation_certificate.py:406](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_certificate.py#L406)、
  [formation_certificate.py:527](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_certificate.py#L527)），
  不再按 allocation；`_allocations_at()` 已删除，allocation 仅保留为 f_only 对照诊断。
- 段边界与 F-region cuts 双侧检查：证书遍历 `_segments_at(template, sigma, tolerance)` 返回的
  所有段（边界处同时得到左右两段），对每段 F region 与 OBB piece 都检查；`knots` 作为显式
  cuts 加入；used region indices 含 F regions。
- 证书 identity 绑定 request/grid/context/path/chain/template/allocation/guide；
  `verify_formation_certificate()` 从 payload 重建并重新验证，任一 identity 被篡改即拒绝。

### 4.4 pipeline 门控迁移

- 迁移只可能在 union scope、`corridor_enforcement=hard`、且旧
  `audit_path_in_corridors()` 失败时发生；该分支位于 VERIFIED certificate gate 之后
  （未过 gate 会在之前 `return`），记录
  `f_only_gate_migrated=true`、`reason=three_vehicle_union_v1_continuous_certificate`、
  证书 status/sha 与旧 containment reason
  （[pipeline.py:2447](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py#L2447)）。
- 本次返工**没有**任何实际运行到达该分支：live union 为单 region，`f_only_gate_migrated=false`；
  E09 场景不可复现（§5.2）。因此该分支只有代码与门控顺序证据，没有端到端 true 证据，
  不把模板早期拒绝或直线路径包装为迁移成功。

## 5. P1-D：E04 / E09

### 5.1 E04 拆分结论

详见 [E04_SPLIT.md](work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e04/E04_SPLIT.md)。

**结论 1（live 负例，只证明 fail-closed）**：`live/negative_e04_map05_175_rework/`

- 22/22 checks ok，`executable_envelope_count=0`；`negative_no_playback_plan`、
  `negative_no_stale_playback`（0 events）、`negative_hud_not_certified`、
  `negative_candidates_not_certified`、`negative_no_certificate` 全为 true。
- quality report：`NO_ACCEPTED_CANDIDATE` / `executable=false` / reason
  `FORMATION_CORRIDOR_UNCONFIRMED`；`formation_corridor_audit.status=TEMPLATE_BUILD_FAILED`，
  detail `assessment station sigma=0.5 map status='COLLISION_OBSERVED'`；
  `formation_obb_constraint_count=null`、`checks=0`、`attempts=[]`、
  `trajectory_published=false`、`publication=null`。
- 该结果证明：Map collision 在模板构建/NLP 之前被拒绝，失败时不发布轨迹、不复用旧轨迹。
  **不证明** OBB NLP rows 拒绝了该场景。

**结论 2（离线 solver 集成测试，证明 OBB NLP 负例）**：
`step_d_e04/e04_offline_probe.py|.out|.meta`，场景与
`tests/test_virtual_a_formation_corridor.py::
FormationSampleAuditStep3Tests::test_e04_offline_obb_rows_are_real_and_reject_f_only_feasible`
（[tests/test_virtual_a_formation_corridor.py:1675](tests/test_virtual_a_formation_corridor.py#L1675) 的同类场景）相同，
通过生产 `solve_open_path_guided` 求解：

| 量 | 值 |
|---|---|
| F rows | 324 |
| OBB rows | 3888（`rows_by_vehicle` 每车 1296） |
| numeric vector / `g` 总数 | 4486（`corridor_end_expected=4486`） |
| max F row | `−1.09999`（F 参考点可行） |
| max OBB row | `+0.15001`（OBB 半空间正残差，违反） |
| solve | status `BUDGET_EXHAUSTED` / reason `SOFT_BUDGET_EXHAUSTED`，无 accepted attempt，path=None |
| solver entry | attempt 0 记录 `formation_obb_constraint_count=3888`，`SOLVER_FAILED` |

即：solver 确实带 OBB rows 进入 NLP；F-only 可行而 OBB 不可行的候选不会被接受。
该场景是离线集成测试，不是 live ROS 负例；live 版本仍是 `NOT_RUN`。

### 5.2 E09：NOT_REPRODUCIBLE

详见 [E09_OBSTRUCTION.md](work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e09/E09_OBSTRUCTION.md)。

- 方法：14 个确定性场景（`e09_matrix.sh`，覆盖 45°/60°/90°/120°、多腿长比、多 padding、
  三种模板 profile）+ 2 个障碍强制场景；`e09_profiling.py` 仅用 `unittest.mock` 过滤 profile
  并捕获输入输出，生产 chain/guide/allocation/pipeline 未改。完整日志、`matrix_summary.md`
  与逐行数据均在 `step_d_e09/`。
- 观测：11 个场景有候选路径，其 F 相对旧 allocation region 的最坏残差全部 ≤ **−1.255 m**
  （负值=在 region 内部），即从未离开旧 allocation region；这些路径旧
  `audit_path_in_corridors()` 全部 PASS。m08/m12/m14 未产生候选路径。所有场景的 union
  certificate 均未 `VERIFIED`（UNCONFIRMED 或 DEFINITE_VIOLATION），pipeline 全部
  `NO_ACCEPTED_CANDIDATE`。
- 唯一可复现的 union `SUCCESS` 均为**单 region chain**（扩展网格直线场景与 live
  `union_map05_2625_rework`），结构上不存在相邻 region/迁移窗口，
  `f_only_gate_migrated=false`。
- 结构性阻碍（详见报告 §4）：
  1. 模板约束是单侧包含，允许但从不强制迁移；F=(V2+V3)/2 的凸性意味着 F 离开旧 region
     需要至少一辆横向车 OBB 同时离开；
  2. allocation 切点由 guide portal 占比决定，而长度/曲率最优路径在拐角内切，到达旧 region
     边界晚于切点；
  3. 障碍半空间对两个相邻 region 等量收窄，无法只把旧 region 收窄到足以提前出口而不破坏
     模板覆盖/证书；
  4. layout 级 fixture 的 forward F cut 早于 allocation cut，但生产代价不会选择外摆路径。
- 结论：在当前“不改代价/参考/初值、不新增搜索、不改 allocation 语义、不放宽阈值”的固定
  契约内，E09 不可构造。按 line 129 记 `NOT_REPRODUCIBLE` 并停止；本目录不含任何
  `f_only_gate_migrated=true` 运行，未伪造旧 audit 失败或早期拒绝作为通过证据。

## 6. 测试矩阵

标准环境：repo 根目录，`source /opt/ros/noetic/setup.bash`、`source catkin_ws/devel/setup.bash`，
`PYTHONPATH="$PWD:$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"`。
全部日志在 `mandatory/`（脚本 `mandatory/run_mandatory.sh`）。

| 命令 | 结果 | 日志/耗时 |
|---|---|---|
| `python3 -m unittest tests.test_virtual_a_formation_corridor` | 64 tests OK | 53.876 s |
| `python3 -m unittest tests.test_virtual_a_corridor_optimizer` | 30 tests OK | 13.145 s |
| `python3 -m unittest tests.test_virtual_a_pipeline` | 16 tests OK | 139.878 s |
| `python3 -m unittest tests.test_virtual_a_audit tests.test_virtual_a_integration_bounds` | 73 tests OK | 6.156 s |
| `python3 -m unittest tests.test_virtual_a_collision` | 17 tests OK | 2.763 s |
| `python3 -m unittest tests.test_virtual_a_ros_adapter tests.test_virtual_a_playback tests.test_virtual_a_receiver` | 128 tests OK | 80.067 s |
| `python3 -m unittest tests.test_formation_demo_assets` | 29 tests OK | 4.035 s |
| `catkin_make` | exit 0 | 0.69 s |
| `bash -n run_demo.sh` | exit 0 | — |
| `./run_demo.sh --map map05 --scale 2.625 --dry-run` | exit 0，含 `virtual_a_formation_corridor_scope:=f_only` | — |

定向：`step_bc/formation_f_region_rework.log`（6 OK）、
`step_bc/f_only_historical_identity.log`（1 OK）、
`step_bc/playback_v2_binding_rebuild.log`（1 OK）。
按方案 §6 未重跑 617 项全套历史测试：返工只触及该调用链的 5 个生产模块与 3 个测试模块，
其余模块由上述 357 tests 与 live 审计覆盖。

## 7. live ROS/RViz

详见 [LIVE_SUMMARY.md](work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/live/LIVE_SUMMARY.md)。
任务私有 ROS master `11663`，`ROS_HOME`/`ROS_LOG_DIR` 在各自 run 目录；从未接触 `11411`。

### 7.1 union 正例 `union_map05_2625_rework`

- `./run_demo.sh --map map05 --scale 2.625 virtual_a_formation_corridor_scope:=three_vehicle_union_v1`
  （reset 2,2,0；goal 8,2,0），UTC 01:59:06→02:00:01。
- `capture_summary.json`：`passed=true`，29/29 checks，`executable_envelope_count=1`；
  statuses `IDLE→PLANNING→FEASIBLE→FEASIBLE`；HUD
  `planning PUBLISHED | audit FEASIBLE | geometry/dynamics/collision PASS`。
- quality report：`SUCCESS`/`executable=true`/union；F rows 324，OBB rows 3888；
  `metrics.formation_f_max_residual_m=0.0`、`metrics.formation_obb_max_residual_m=0.0`、
  accepted attempt `constraint_residual=1.7763568394002505e-15`、`corridor_residual_m=0.0`；
  certificate `VERIFIED`，max_outward `−0.37498999716394005`，unclosed 0，16/24；
  cert sha `a36ca37a…01b6c0a`，template_id `382adc91…1ff89b2`，chain_digest `e7f8568f…e4b618`；
  request/grid 与冻结 f_only 输入一致（`e69d…`/`64eef…`）；planning context 为
  `9655b79f…`（union 运行上下文，scope 属于 context，与 f_only 冻结值 `06aed4…` 不同，
  见 §8）；certificate path length 6.0 m（route/guide length 为 6.079790116 m），
  path sha `6775dda4…48ced`；`f_only_gate_migrated=false`。
- receiver 重建：`envelope_carries_quality_binding=true`（length 11989）、
  `envelope_verifier=true`（expected_formation=true，712 samples）、
  `union_binding_complete=true`、`envelope_digest`/`envelope_request_identity=true`；
  路径为生产 `formation_playback._verify_union_formation_binding`：重建
  `corridor_chain_from_payload`、`formation_template_from_payload`（v2）、
  `verify_formation_certificate`，并重跑 geometry/dynamics audit。
- 执行层：geometry 155、dynamics 909、collision 3740 checks 全 PASS；11 张 live 截图 +
  终端截图，`screenshots_ok=true`。

### 7.2 失败清屏负例 `negative_e04_map05_175_rework`

- `./run_demo.sh --map map05 --scale 1.75 virtual_a_formation_corridor_scope:=three_vehicle_union_v1`
  （reset 1.51,2.5,−π/2；goal 1.51,0.5,−π/2），UTC 02:00:33→02:01:00。
- 22/22 checks，无 envelope；HUD `FAILED`、`map_collisions=3`、`audit AUDIT_PENDING`；
  无旧轨迹残留。该负例口径见 §5.1，不是 OBB NLP 负例。

## 8. `f_only` 兼容性与序列化

- 默认 scope：Python `PlannerConfig` 默认 `f_only`；launch `formation_demo.launch:87` 默认
  `f_only`；`run_demo.sh:452` 在未指定时追加 `virtual_a_formation_corridor_scope:=f_only`。
  `mandatory/run_demo_dry_run.log` 含该参数。
- 历史 identity 重建测试
  `test_f_only_historical_model_and_context_hashes_are_unchanged`
  （[tests/test_virtual_a_pipeline.py:509](tests/test_virtual_a_pipeline.py#L509)）：
  从 TASK03 原始证据 `work_flow/TASK03_evidence/20261119T133018Z/baseline/` 重建冻结的
  MAP05 直线请求、context、grid，断言：
  - request payload/fingerprint：`e69d068b1cae75fa7205a7530b129f16bf731dddce705e6d2ef8042fec07aeaf`
  - planning context：`06aed480a711839483c94e42648563b53c48eb0080f3c9dfe4e16daebe9414dd`
  - grid：`64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321`
  - map pgm sha `0856ad0a…`、yaml `e4ee76a1…`（冻结输入）
  日志 `step_bc/f_only_historical_identity.log`：1 test OK。
- 序列化：v2 模板 payload 往返后 `template_id` 完全一致；`f_only` 结果不携带
  certificate/template 字段（`test_f_only_result_key_set_keeps_no_certificate`）；union
  binding key 集合保持精确 `{"scope","planning_context_sha256","template","chain","certificate"}`。
- live union 的 context `9655b79f…` 是 union 运行自己的 planning context（f_only 冻结值
  `06aed4…` 是历史 MAP05 f_only 运行值），两者不矛盾；f_only 历史值由上述重建测试保证。

## 9. 变更文件与返工增量 diff

相对返工开始快照（`end/rework_only_diff.patch`，2434 行，+1502/−177）：

| 文件 | +/− |
|---|---|
| `formation_continuous_a/contracts.py` | +37 / −3 |
| `formation_continuous_a/optimizer.py` | +230 / −34 |
| `formation_continuous_a/pipeline.py` | +21 / −0 |
| `formation_continuous_a/formation_certificate.py` | +83 / −59 |
| `formation_continuous_a/formation_corridor.py` | +396 / −77 |
| `tests/test_virtual_a_formation_corridor.py` | +642 / −4 |
| `tests/test_virtual_a_pipeline.py` | +64 / −0 |
| `tests/test_virtual_a_playback.py` | +29 / −0 |

全部位于返工方案允许的生产/测试文件范围；未改 `geometry.py`、`integration_bounds.py`、
`timing.py`、`collision.py`、旧 FR06、uniform_v02、legacy、single-vehicle planner、
`bounds.py`；未改 ROS message schema（`FormationPlaybackPlan.msg` 的既有
`formation_binding_json` 承载 v2 JSON）。

## 10. 受保护状态

开始/结束对比（`start/protected_*` vs `end/protected_*`）：

- `geometry.py` SHA `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`（不变）
- `tests/test_virtual_a_geometry.py` SHA `4f59e68f…7ddb4c7`（不变）
- `evaluation/` 树 `c2fa3efa…`（不变）
- `work_flow/TASK01_evidence/` 637 files 树 `a89ff48c…`（不变）
- `work_flow/TASK02_evidence/` 425 files 树 `b9f73b8b…`（不变）
- TASK03 原始证据 `work_flow/TASK03_evidence/20261119T133018Z/` 未写入；本返工全部证据在新目录。
- `end/git_identity.txt` 与 `start/git_identity.txt`：HEAD/branch/远端一致；status 条目从 29
  增至 30，唯一新增项是 `work_flow/TASK03_REWORK_01_Claude交付.md`（证据目录 `??` 条目在开始
  快照中已存在；`end/` 在报告完成后重跑，最终值以 `end/git_identity.txt` 与
  `manifest_summary.json` 为准）。

## 11. 预算与固定契约

未改变：3 templates、6 solver initials、3 routes、1 recovery、60 s soft budget、无隐藏重试；
未改阈值/边距/证书判据；`f_only` 默认未切换；collision/certificate/playback 审核未关闭；
union 只使用同一已审核 `CorridorChain` 相邻 region 的有限确定性 assignment，无组合枚举、
整数规划或新搜索。

## 12. 未完成 / NOT_RUN

- **E09：NOT_REPRODUCIBLE**（方案 line 129 允许并已停止）；无 `f_only_gate_migrated=true`
  的端到端运行；迁移分支未被实际触发。
- **live OBB NLP 负例：NOT_RUN**。live E04 负例在模板构建（NLP 之前）被地图碰撞拒绝；
  OBB NLP 负例只由离线 solver 集成 probe 提供（§5.1 结论 2）。
- 未重跑 617 项全套历史测试（方案 §6 允许按调用链范围测试）。
- 未提供 union 多 region 且 certificate VERIFIED 的 live 运行；live union 正例是单 region
  chain（合法但无迁移窗口）。

## 13. 停止状态

按返工方案 §7：**完成即停止，等待 Codex 独立验收**。未 commit、未 push、未开始 TASK04；
未切分支/新建 worktree；未 reset/checkout/clean；未覆盖或删除任何旧证据/冻结 fixture/
`evaluation/` 历史数据；`f_only` 默认保持。

复现入口：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash && source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD:$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
# 回归
bash work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/mandatory/run_mandatory.sh
# rows / g 总数
python3 work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_bc/union_g_total_probe.py
# E04 离线负例
python3 work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e04/e04_offline_probe.py
```