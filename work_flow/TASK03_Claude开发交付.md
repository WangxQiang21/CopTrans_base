# TASK03 Claude 开发交付

生成时间：2026-11-20（UTC）。任务：`work_flow/TASK03_技术方案_交Codex.md` + `work_flow/TASK03_Codex执行方案.md` 的 TASK03。
工作树：`/home/wq/whut_thesis/expv3_multi_virtual_a`（git worktree）。
基线 HEAD：`a3c709e57560b1060dcc422777f4131a3e558acf`（`TASK02 virtual rigid formation diagnostics and guide candidates`），分支 `feature/virtual-rigid-formation-a`，与 `origin/feature/virtual-rigid-formation-a` 同步（`0 0`）。

本文件是 Claude 的实现和证据说明，不是 Codex 审核结论；不宣称 Codex PASS。TASK03 完成后已停止：未 commit、未 push、未开始 TASK04。

## 1. 最终状态与保护约束核对

- 代码未提交；`git status --porcelain` 共 27 项：TASK03 自身为 16 个已跟踪文件 + 3 个新文件，TASK01 既有 dirty 2 项，另有 6 项 TASK02/TASK03 文档与证据目录（完整列表见 `TASK03_evidence/20261119T133018Z/final/git_status_end_step6.txt`）。
- TASK03 自身 diff：`16 files changed, 2575 insertions(+), 47 deletions(-)`，另加新文件 `formation_certificate.py`（1153 行）、`formation_corridor.py`（1837 行）、`tests/test_virtual_a_formation_corridor.py`（2648 行）。TASK01 两项 dirty 单列，不计入 TASK03 diff；含 TASK01 的总体 diff 为 `18 files changed, 2667 insertions(+), 48 deletions(-)`。
- `evaluation/` 最终无改动：legacy 回归运行时 FR04 harness 会重写结果文件里的计时字段（既有行为），已核验差异仅为 timing 键与 `generated_at_utc` 并恢复 HEAD，`git status --porcelain evaluation/` 为空（见 §9 第 9 条）。
- TASK01 dirty 文件逐字节未变（与 TASK02 交付时一致，见 `final/manifest.json` 的 `protected_task01_files`）：
  - `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py`
    `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
  - `tests/test_virtual_a_geometry.py`
    `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`
- 未修改 `TASK01_evidence/`、`TASK02_evidence/`、`evaluation/`、输入方案文档、冻结 fixture；未清理、移动或删除既有未跟踪文件。
- `run_demo.sh` 默认 scope 保持 `f_only`：`run_demo.sh:452` 只有未显式指定时才追加 `virtual_a_formation_corridor_scope:=f_only`；`formation_demo.launch:87` 的 arg 默认值也是 `f_only`。dry-run（未传 scope）打印的完整 roslaunch 参数行含 `virtual_a_formation_corridor_scope:=f_only`，证明自动默认路径（`step6/tests/run_demo_dry_run.log`）。默认入口的字面参数链仍为 `backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false`，只多了默认 `f_only`。**按方案，默认 scope 只有在 Codex 接受后才允许切换。**
- 真实 ROS 只使用任务私有 master `http://127.0.0.1:11663`；另一个会话的 `11411` 与用户此前 demo 的 `11311` 未被触碰。每次 live run 的 `ROS_HOME`/`ROS_LOG_DIR` 都在该 run 目录内。

## 2. 实际改动文件、函数与调用路径

| 文件 | 主要新增/修改 | 调用路径 |
|---|---|---|
| `formation_continuous_a/contracts.py` | 版本化 scope 常量 `FORMATION_CORRIDOR_SCOPE_THREE_VEHICLE_UNION_V1`（L86-88）；`PlannerConfig.formation_corridor_scope`（L847-853），非 `f_only` 时写入 planning-context 编码（L877-882）；scope 合法性与 `corridor`/`hard` 前置校验（L1041-1056）；默认工厂保持 `f_only`（L1126） | planner node 读 ROS param → `PlannerConfig` → `planning_context_sha256` |
| `formation_continuous_a/corridor.py` | 新增 `_station_coverage_view()`（L1315-1362）并附到 `assess_formation_guide()` 的 `station_coverage`（L2025）；提供每车 region/status 的有限 witness 供 union 模板构建读取，仅诊断，不改变旧 audit 门槛 | pipeline union 分支 → 模板构建 |
| `formation_continuous_a/formation_corridor.py`（新） | `vehicle_frames()`/`formation_model_sha256()`/`formation_allocation_digest()`/`formation_guide_digest()`；`whole_cover()`、`diagonal_cover()`、`single_region_template()`、`validate_formation_template()`；`finite_cover_options()` 与 `build_formation_template_candidates()`（整区 + 两个对角线模板，≤3 候选）；`FormationCorridorHalfspace`/`FormationCorridorVertexRows` 与 `build_formation_corridor_constraint()`；`audit_formation_template_samples()`；不足时抛 `COVER_TEMPLATE_INSUFFICIENT`（L1185-1239、L1449） | `pipeline.plan_virtual_a()` 的 union 分支生成/校验模板与 `g` rows |
| `formation_continuous_a/optimizer.py` | union scope 的完整 OBB corner hard rows 进入 numeric residual 与 CasADi `g`；模板感知的 `solve`/`solve_open_path_guided` 门（仅选中模板进入解算） | `pipeline` union 分支 → solver → candidate |
| `formation_continuous_a/formation_certificate.py`（新） | `certify_formation_path()`（L625）：全 F domain + 三车完整 OBB 的 outward-rounded interval certificate，必需切点双侧核对；`verify_formation_certificate()`（L1030）；状态 `VERIFIED`/`VIOLATED`/`UNCONFIRMED`；`_sealed_body()` 只排除 `elapsed_s` | optimizer gate → result → envelope binding → playback verifier |
| `formation_continuous_a/pipeline.py` | 显式 `formation_scope`（L1332-1402）；union 分支模板/证书（L2048-2111、L2259-2403）；`three_vehicle_union_v1_continuous_certificate` 迁移（L2436-2465，仅 union scope、且证书 VERIFIED 才替代旧 F-only 单区 gate）；result 装配（L2908-3001） | planner node → `plan_virtual_a()` |
| `formation_playback.py`（optimizer 显示/验证模块） | `verify_playback_plan()` 新增 `expected_formation`（L976-1259）；`_verify_union_formation_binding()`（L1017）精确键 `{"scope","planning_context_sha256","template","chain","certificate"}`，重建 chain/template、检查 grid、调用 `verify_formation_certificate`；`f_only` 不携带 | planner node envelope 验证；playback 接收验证 |
| `formation_continuous_a/ros_adapter.py` | `EXPECTED_ENVELOPE_KEYS` 加 `expected_formation`（L95-101）；`verify_envelope_fields()` 透传（L916、L993） | planner node `_finish_virtual_success()` |
| `formation_uniform_planner_node.py` | union 结果构造 sealed binding（L1914-1934）并写入 envelope 字段（L1935-1941），planner 侧先 `verify_envelope_fields`（L1942-1957）再发布；planner 读 `~virtual_a_formation_corridor_scope`，默认 `f_only`（L2482-2483） | `/formation/playback_plan` 原子 envelope |
| `fwis_v3_msgs/msg/FormationPlaybackPlan.msg` | 新增 `string formation_binding_json`（L24，空字符串表示 f_only/legacy/uniform） | 消息 schema（需重建） |
| `fwis_v3_playback/scripts/formation_playback.py` | `_reject_json_constant()`（L74-75）；`_verify_virtual_envelope()` 解析 `formation_binding_json`，拒绝非有限 JSON/非 object（L732-750），并传 `expected_formation`（L760-763） | playback 接收 → verifier |
| `formation_demo.launch` | `virtual_a_formation_corridor_scope` arg 默认 `f_only`（L87），仅 virtual_rigid_a 时下发 param（L130-131） | `run_demo.sh` |
| `run_demo.sh` | 帮助文本（L28）；scope 参数去重与默认 `f_only`（L415-452） | 入口 |
| `tests/test_virtual_a_pipeline.py` | +2 union scope/证书进入 pipeline 的测试 | 单元测试 |
| `tests/test_virtual_a_playback.py` | +union binding 传输与拒绝测试 | 单元测试 |
| `tests/test_virtual_a_receiver.py` | +3：binding 原样转发、f_only 无 binding、畸形 binding 拒绝 | 单元测试 |
| `tests/test_formation_demo_assets.py` | launch/run_demo 新 arg 与默认值断言 | 单元测试 |
| `tests/test_virtual_a_corridor_optimizer.py` | Step 5 soft-clearance 消融测试 | 单元测试 |
| `tests/test_virtual_a_formation_corridor.py`（新） | 模板/证书/gate/反例/tamper 50 tests | 单元测试 |

## 3. 算法与安全门语义

- **scope 是显式版本化输入**：`f_only`（历史 F 参考点单区 containment）与 `three_vehicle_union_v1`（F + 三车完整 OBB）二选一；未知 scope、union 缺 `corridor`/`hard` 均 fail closed。scope 进入 planning-context 编码，因此 union 与 f_only 的 context/model digest 必然不同，不能互认。
- **有限模板**：整区完整覆盖 + 两个对角线三角形覆盖（真实车体 0.738×0.500 m 的完整 OBB，不是四角/中心采样）。三种模板都表达不了时返回 `COVER_TEMPLATE_INSUFFICIENT`/UNKNOWN，安全拒绝，不称为物理不可行。
- **hard rows 真正进入 `g`**：选中模板的每车 corner rows 同时进入 numeric residual、CasADi `g` 与 SciPy fallback；本轮 live 结果为 `formation_obb_constraint_count=3888`。
- **独立连续证书**：`virtual_a_formation_certificate.v1`，覆盖全 F 与三车完整 OBB；必需切点双侧检查；只有所有闭合子区间 outward upper residual ≤ 0 且所有 cut `CLOSED` 才是 `VERIFIED`。`VIOLATED` 有可信正向 witness；`UNCONFIRMED`/`VIOLATED`/timeout/identity 不符一律不可执行、不可发布。
- **旧审核仍是必需**：union 的证书只在 F-only gate 位置替代旧单区门槛（scope 限定迁移）；geometry/dynamics/original-map OBB collision 三段独立 audit、sweep/margin、publication verifier 全部保留且未放宽。

## 4. Scope 贯通与传输

链路（每步均有源码位置，见第 2 节）：

```
run_demo.sh --... virtual_a_formation_corridor_scope:=three_vehicle_union_v1
  → formation_demo.launch arg/param
  → planner node ~virtual_a_formation_corridor_scope
  → PlannerConfig/planning_context_sha256
  → pipeline.plan_virtual_a() union_scope
  → 模板 → CasADi g → solver → candidate
  → certify_formation_path() → optimizer gate → result.formation_corridor_audit
  → envelope.formation_binding_json（原子 envelope 内）
  → ros_adapter.verify_envelope_fields(expected_formation)
  → playback _verify_virtual_envelope() → verify_playback_plan(expected_formation)
      → _verify_union_formation_binding()（重建 chain/template、检查 grid、再次 verify 证书）
```

- binding 精确键为 `{"scope","planning_context_sha256","template","chain","certificate"}`；键集合不符即拒绝。
- binding 必须走**同一个原子 envelope 字段**：`model_json` 有精确键校验，job_state 跨 topic 无顺序保证，`certification` 只是固定字符串。新增 msg 字段是唯一无消息顺序竞态的传输方式。f_only/legacy/uniform envelope 该字段为空，行为不变。
- planner 侧在发布前先调用同一 verifier；receiver 侧重建 chain/template 并重验证书，任何 stale/mismatch 都拒绝播放。

## 5. HUD / RViz 语义与命名空间

- 已发布、已独立审计的 plan：HUD 显示 `audit FEASIBLE`（或 `NO_MOTION`），不再使用 `CERTIFIED`。
- `CERTIFIED` 只属于**最终 candidate markers**，且只在 `PUBLISHED` 之后出现；Guide 图层（`formation/planning/astar_raw`、`formation/planning/portal_guide`）始终是 `REFERENCE_ONLY`。
- Guide sample 风险与最终 path failure 使用独立命名空间：`formation/planning/portal_window`、`formation/planning/map_collision_sample`、`formation/planning/coverage_gap`；负例不产生 certified path，也不残留旧轨迹 marker。
- 本轮 live 证据（union）：guide texts 为 `A* ROUTE | REFERENCE_ONLY`、`PL_GUIDE | REFERENCE_ONLY`；candidate 图层在 PUBLISHED 后含 `CERTIFIED`；负例 guide 层出现 `MAP COLLISION (sample)` 且 candidate 层无 `CERTIFIED`。

## 6. 真实 ROS/RViz 证据（`step6/live/`）

任务私有 master `11663`；每次 run 独立 `ROS_HOME`/`ROS_LOG_DIR`；RViz 输入与用户操作相同（reset/goal 由 capture driver 发布，脚本 `tools/ros_step6_capture.py`）。所有 run 的 params/topics/nodes/planning_debug/quality/markers/HUD/截图/节点日志均在各自目录。

### 6.1 union 正例 `union_map05_2625`（PASS，29/29 checks）

命令：`./run_demo.sh --map map05 --scale 2.625 virtual_a_formation_corridor_scope:=three_vehicle_union_v1`；reset `2,2,0`，goal `8,2,0`。

- plan `600000`，`SUCCESS`/`FEASIBLE`，`executable=true`，planning wall `11.460 s`，`T_end=14.2031250000003 s`，`S=6.0 m`；`initial_count=1`、route 1、recovery 0。
- request `e69d068b1cae75fa7205a7530b129f16bf731dddce705e6d2ef8042fec07aeaf`；context `9655b79fc587ab7251a4454a42e219bd44c1eb8dd0b2363ae666a90ff33680fc`；grid `64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321`；model（playback）`e653238d55ed644286c8ba3676a3b2ac992d8112f9f8672de0712bebbd622cc9`。
- OBB hard rows `3888`；template `4e897ff7479e3a4c8dcdfc0df564bcdf3f05166e726598b7f819bebd569a7a80`；chain `e7f8568f2397a95f44a0dc2693a1fea4e8f6bd50635dd80ce0e32c56eee4b618`。
- 本次场景 `template_count=1`：单段 **whole-region** 完整 OBB 覆盖（三车各 `decomposition=whole`、region 0，模板面积 0.369 m²/车），rank 0 首次尝试即接受；`not_tried=[]`，无模板候选被截断。
- certificate `VERIFIED`：`intervals_checked=24`、`intervals_verified=16`、`max_depth=1`、`unclosed=0`、`max_outward_residual_m=-0.37498999716394005 m`、margin `1e-5`、guard `1e-12`、16/16 cut `CLOSED`、`witness=null`；`certificate_sha256=86cce3e8e54c85e0038cc4c526f04748c76ffd546a1197302e4613e7152eb886`。三次 union run 都 `VERIFIED`，但证书 hash 互不相同（attempt01 `888d9456…`、attempt02 `99ed13bf…`），因为 sealed payload 含“实际使用的 `limits.max_seconds`”（剩余墙钟预算）；同 Step 5 已披露的口径，不是路径/模板不稳定。
- 旧审核全部通过：geometry `155`、dynamics `909`、original-map OBB collision `3740` checks。证书内 F-only 对照 `passed=true`，`max_bound_residual_m=-1.6249999486400133 m`，`certificate_scope=f_only_comparison_not_the_three_obb_gate`。
- envelope：1 条 executable；`formation_binding_json` 长度 `11835`，`union_binding_complete=true`；envelope verifier PASS；HUD `audit FEASIBLE`、`planning PUBLISHED | step 8/8`、elapsed `11.5 s`。
- 截图 `images/00_idle.png`…`live_11_34.0s.png`。

### 6.2 E04 负例 `negative_e04_map05_175`（PASS，22/22 checks）

命令：`./run_demo.sh --map map05 --scale 1.75 virtual_a_formation_corridor_scope:=three_vehicle_union_v1`；reset `1.51,2.5,-π/2`，goal `1.51,0.5,-π/2`。

- `NO_ACCEPTED_CANDIDATE` / `FORMATION_CORRIDOR_UNCONFIRMED`，`first_failure_layer=PLANNING`；corridor audit `TEMPLATE_BUILD_FAILED`，`detail=assessment station sigma=0.5 map status='COLLISION_OBSERVED'`。
- 5 samples / 3 map collisions / 12 unknown；`initial_count=0`；planning wall `0.448 s`。
- request `e5b6e715c8984c7cd488b64a8067011289cb13eccbe8bd65c2f39cc5fc82165d`；context `0d87586befc580ecffca3a1300a1c7b086f386a961182d7a9c738459ca177673`；grid `fcf003c36694b0907a7aefb26aa07205cb8a0c38e0e65fd2fda38f80b3d65215`。
- `negative_no_executable_envelope count=0`；`negative_no_playback_plan=[]`；`negative_no_stale_playback events=0, stale_states=[]`（无 PLAYING/HOLD_END 残留）；result 不含证书（`negative_no_certificate`）；candidate 层无 `CERTIFIED`（`negative_candidates_not_certified=[]`）。最新 marker 数组不是 DELETEALL，因为 guide 诊断层按设计保留：`formation/planning/map_collision_sample` 有 6 个红色 `MAP COLLISION (sample)`（车 1/2/3，含 `MAP_OBB_COLLISION`），不是 certified 轨迹。
- HUD 为失败态（`executable_audit=false`，`planning FAILED | step 3/8 | FORMATION_CORRIDOR_UNCONFIRMED`，`job FAILED | playback FAILED`）。

### 6.3 f_only 对照 `f_only_map05_2625`（PASS，28/28 checks）

命令：`./run_demo.sh --map map05 --scale 2.625 virtual_a_formation_corridor_scope:=f_only`（显式历史 scope；"未指定时自动追加 `f_only`" 由 dry-run 的完整 roslaunch 参数行证明）；reset/goal 与 union 相同。

- plan `600000`，`SUCCESS`/`FEASIBLE`，wall `6.434 s`，`T_end=14.203125000000316 s`，`S=6.0 m`；`initial_count=5`、route 1、recovery 0。
- request 与 union 相同 `e69d068b…`；context `06aed480a711839483c94e42648563b53c48eb0080f3c9dfe4e16daebe9414dd`；grid 相同 `64eef089…`；model `e6b6e55db62346d041e959b1955e031d33bd571995e53483b46c1c962635bcad`。
- **该 model hash 与 Step 0 冻结的 MAP05_straight 历史结果逐字相同**（`baseline/README.md`），terminal `(8,2,0)`、`S=6.0`、`T=14.203125`，证明默认入口的历史行为未被 TASK03 改变。
- result 无 `formation_certificate` 键、无 union scope 键；envelope `formation_binding_json` 长度 `0`；HUD `audit FEASIBLE`；旧三段 audit 155/909/3740 全过。

### 6.4 保留的失败 attempt

- `union_map05_2625_attempt01_playback_binding_gap/`：修复前 union envelope 无法携带 binding，playback 记录 `rejecting playback plan 600000: union-scope plan has no retained formation corridor binding`，随后 planner 的 `_publish_no_envelope` 发出空 certification 的无效 envelope（`unsupported certification ''`），playback 按预期清屏，`PLAYBACK_START_TIMEOUT`。这是真实失败证据，未删除；随后以 msg 字段修复（第 4 节）。
- `union_map05_2625_attempt02_hud_word_only/`：全部功能检查通过，仅 capture 的 `hud_certified` 断言错误（HUD 对已审计 plan 的正确词是 `audit FEASIBLE`）；修正断言后最终 run 通过。attempt02 保留为已知失败证据。

## 7. E01–E14 实验矩阵

| ID | 状态 | 证据 |
|---|---|---|
| E01 | **PASS（offline+live）** | union live 正例（§6.1）：既有 audit 全过 + 证书 `VERIFIED` 才发布；f_only live 复现冻结历史 model hash（§6.3） |
| E02/E03 | **NOT_REPRODUCIBLE** | `baseline/user_plans_reproducibility.json`：600001/600002 无原始 request/grid/context/map 输入，不伪造复现 |
| E04 | **PASS（offline+live）** | live 负例（§6.2）；`test_f_feasible_but_obb_infeasible_solve_is_never_accepted` |
| E05 | **PASS** | `test_real_audited_chain_builds_whole_and_diagonal_segments`（真实相邻 region，car1 在 Cj、car3 在 Cj+1） |
| E06 | **PASS（安全拒绝边界）** | `test_insufficient_template_is_unknown_not_physical_infeasibility`：`COVER_TEMPLATE_INSUFFICIENT`，不称物理无路；正例 `test_three_vehicles_whole_diagonal_and_nonconvex_gap` |
| E07 | **PASS** | `test_three_vehicles_whole_diagonal_and_nonconvex_gap`：四角属于非凸 union、矩形内部穿 gap → 拒绝/UNCONFIRMED（fixture 按方案 §6 固定） |
| E08 | **PASS** | `test_collocation_safe_but_continuous_corner_violation`（配点安全、中间角点越界被连续 audit 找到）；`test_one_definite_cut_reenters_actual_g_rows`（cut 进入真实 `g`）；`test_second_definite_violation_stops_without_a_third_solver` |
| E09 | **NOT_RUN（专门实验未构造）** | migration 分支存在且有证书守卫（`pipeline.py:2436-2465`）；仅 union scope 且证书 VERIFIED 才替代旧 F-only 单区 gate；本轮 live 未构造"旧 gate 失败但 union 可通过"的专门场景，`f_only_gate_migrated=false`（旧 F-only containment 仅作对照且通过） |
| E10 | **PASS** | `test_insufficient_template_is_unknown_not_physical_infeasibility`（TEMPLATE_INSUFFICIENT/UNCONFIRMED，不称物理无路） |
| E11 | **PASS** | `test_normal_branch_degeneration_is_unconfirmed`、`test_degenerate_vehicle_maps_are_infeasible_in_numeric_and_casadi`、`test_degenerate_guide_offers_only_the_shortest_wrap_branch`（正下界/退化 fail closed） |
| E12 | **PASS** | `test_u_turn_vertices_are_not_deleted_or_shortcut`、`test_s2_audit_confirms_5e8_with_exact_panel_arithmetic`、winding/heading 分支与 19 变量测试（regression_all） |
| E13 | **PASS（补充回归）** | f_only live 复现冻结 MAP05 model hash；legacy/uniform 回归见 §8.3；历史 `INSUFFICIENT_EVIDENCE` 如实保留 |
| E14 | **PASS** | identity/tamper 测试（path/request/grid/context/template/body）；live 负例无旧轨迹、无 certified 误标；attempt01/02 作为已披露失败 |

## 8. 测试、构建与 dry-run

环境（仓库根目录）：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
```

### 8.1 离线检查（`step6/tests/`，全部记录 stdout/stderr、exit、UTC start/end、wall）

| 命令 | 结果 |
|---|---|
| `python3 -m unittest test_virtual_a_pipeline` | 15 tests OK，138.979 s，exit 0 |
| `python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback` | 103 tests OK，76.268 s，exit 0 |
| `python3 -m unittest test_virtual_a_corridor_display test_formation_hud` | 92 tests OK，16.940 s，exit 0 |
| `python3 -m unittest test_formation_demo_assets` | 29 tests OK，4.100 s，exit 0 |
| `python3 -m unittest test_virtual_a_receiver` | 24 tests OK，17.591 s，exit 0 |
| `python3 -m unittest <全部 tests/test_virtual_a_*.py>` | 617 tests OK，385.070 s，exit 0 |
| `catkin_make -C catkin_ws -j4` | 全部 target built/up to date，exit 0 |
| `bash -n run_demo.sh` | exit 0 |
| `./run_demo.sh --map map05 --scale 2.625 --dry-run` | exit 0；roslaunch 参数行含默认 `virtual_a_formation_corridor_scope:=f_only` |

### 8.2 真实 ROS/RViz

见 §6；三次最终 run 的 `capture_summary.json` 分别 29/22/28 项检查全过，失败的 attempt01/02 保留。

### 8.3 legacy/uniform 补充回归

`tests/test_uniform_playback_plan test_formation_playback test_cargo_transport test_transport_contract` 与 `test_formation_regression test_uniform_pipeline`，日志在 `step6/tests/test_legacy_uniform_*.log`：

| 命令 | 结果 |
|---|---|
| `python3 -m unittest test_uniform_playback_plan test_formation_playback test_cargo_transport test_transport_contract` | 65 tests OK，1.431 s，exit 0 |
| `python3 -m unittest test_formation_regression test_uniform_pipeline` | 34 tests OK，49.538 s，exit 0 |

该 legacy 组会执行 FR04 harness 并重写 `evaluation/formation_refactor_v02/FR04/` 下结果文件的计时字段（`wall_time_s` 等）与 `generated_at_utc`；逐文件与 HEAD 递归比较，忽略 timing 键后无任何语义差异（状态、计数、路径、审计结果均未变）。已在核验后定向恢复 HEAD，最终 `git status --porcelain evaluation/` 为空；说明见 `step6/tests/evaluation_side_effect_note.txt`。

### 8.4 Step 5 消融（摘要）

`step5/README.md`：EDT soft-clearance 权重 0 精确恢复无 soft term（bit-identical profile）；certificate 没有 soft 参数，weight/target 不改变证书状态与 hash；默认 weight/target/margin 未改。详细数据在 `step5/step5_metrics.json`。

## 9. 已知失败、偏差与诚实披露

1. **msg schema 变更**：`FormationPlaybackPlan.msg` 新增字段是修复 attempt01 的必要改动；消息 md5 变化，整个 workspace 已重建。uniform_v02/legacy envelope 保持字段为空；补充的 legacy/uniform 回归两组共 99 tests OK（65 + 34，exit 0，见 §8.3），未因新字段改变旧行为。
2. **capture 驱动修正（attempt01/02 保留修正前输出）**：
   - attempt01 的参数在节点加载完成前读取，`scope_param` 误记为 `f_only`；修正为 `wait_ready` 之后再读，最终三次 run 的 `scope_param` 均正确。
   - attempt01 的 marker 断言不允许 PUBLISHED 之后的 `CERTIFIED`，且选错了 envelope（取最新而非最后一条 executable）；修正为 `candidate_certified_matches_published` + `guide_markers_no_certified` + `executable_envelope_record`。
   - attempt02 的失败是 HUD 断言用词错误（`CERTIFIED` ≠ 已审计 plan 的 HUD 词），不是产品缺陷；观察到的正确 HUD 是 `audit FEASIBLE`。
3. **任务私有 master**：全部 live run 用 `11663`，而不是默认 `11311`；`11411`（另一会话）与 `11311`（用户 demo）未被触碰。这是隔离要求，不影响单次 run 的算法行为。
4. **默认 scope 未切换**：`run_demo.sh` 与 launch 默认仍为 `f_only`，等待 Codex 整体验收后再按方案决定。
5. **E02/E03 NOT_REPRODUCIBLE**：缺 600001/600002 原始输入，未伪造。**E09 NOT_RUN**：未构造"旧 F-only gate 失败、audited union 可通过"的专门实验；migration 代码路径有证书守卫，但本轮 live 未触发。
6. **未验证的性能承诺**：union 与 f_only 的 wall time（11.460 s vs 6.434 s）只是同机单次实测，不作为优化/加速结论；initial_count（1 vs 5）也是该场景实测，不泛化。
7. **证书 hash 与预算**：sealed payload 含实际 `limits.max_seconds`（剩余墙钟预算），同一路径重跑可能得到不同 `certificate_sha256`；三次 union run 均 `VERIFIED` 但 hash 不同（§6.1）。Step 5 已用等预算重认证隔离该效应。
8. **组合全量回归**：正确的全量组合运行为 617 tests OK（385.070 s，exit 0）。首次运行脚本以 `tests/` 为 cwd，仓库根不在 `sys.path`，`test_virtual_a_corridor_optimizer`/`test_virtual_a_formation_corridor` 以 `tests.test_virtual_a_*` 导入 helper 时报 `ModuleNotFoundError: No module named 'tests'`，539 tests 中 2 errors，80 tests 未执行；该失败的完整日志保留在 `step6/tests/attempts/attempt01_cwd_tests_import_path/`。修正为仓库根目录运行并把根加入 `PYTHONPATH` 后重跑得到上表结果；这是 harness 缺陷，不是产品失败。全量组合不替代 §8.1 的分组结果。
9. **evaluation/ 计时副作用（已恢复）**：`test_formation_regression` 执行 FR04 harness 时会把当次计时写入 `evaluation/formation_refactor_v02/FR04/cases/F01..F19*.json` 和 `pipeline_results.json`（20 个已跟踪文件变 dirty）。逐文件与 HEAD 递归比较后，忽略 timing 键仅剩 `generated_at_utc` 差异，无语义变化；TASK03 基线中这些文件是干净的，因此定向 `git checkout -- evaluation/formation_refactor_v02/FR04` 恢复，恢复前确认无未跟踪文件，恢复后 `git status --porcelain evaluation/` 为空。说明保留在 `step6/tests/evaluation_side_effect_note.txt`，manifest 以恢复后的状态生成。

## 10. 停止点

TASK03 已完成并冻结：未 commit、未 push、未修改 TASK01/TASK02 历史证据、未开始 TASK04。等待 Codex 整体独立验收（方案 §9 清单）；默认 scope 保持在 `f_only`。

## 11. 复现与环境

- 证据根：`work_flow/TASK03_evidence/20261119T133018Z/`；入口与索引见 `final/README.md`、`final/manifest.json`、`final/SHA256SUMS`。
- 离线检查复跑：`bash step6/tools/run_step6_checks.sh`、`bash step6/tools/run_step6_legacy_checks.sh`；manifest：`python3 step6/tools/make_step6_manifest.py`。
- live 复跑需要 RViz 显示（本轮 `DISPLAY=:1`）；`step6/tools/run_step6_live.sh` 固定了私有 master 与每 run 的日志目录。