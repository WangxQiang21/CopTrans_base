# TASK02 Step E 证据：默认入口、HUD 与 RViz 有限诊断

生成时间：2026-11-19（UTC）。目录：`work_flow/TASK02_evidence/20261119T054527Z/candidate/step_e/`。
本目录只记录 Step E 的实现、离线测试和真实 progress/HUD 样本；不是 Codex 审核结论，也不宣称 TASK01/TASK02 验收通过。TASK01 dirty 的 `geometry.py` 与 `tests/test_virtual_a_geometry.py` 未被 Step E 修改。

> 2026-11-19 更新（Step F 真实 ROS 运行后）：`source_hashes.txt` 与 `step_e.diff` 已按最终工作树重新生成，
> 新增 `tests_display_after_queue_fix.log`（194 tests, OK, 80.160 s）。原因是 Step F 现场运行发现
> progress 文档的发送/接收队列过小，完整阶段的文档被丢弃；两处队列修正见第 7 节，均为发布/接收
> 队列参数和注释，不改变任何 admission、求解、审核或发布门控。本节其余内容保持 Step E 原记录。

## 1. 实现范围

### 1.1 `corridor.py`：只读 assessment 的有界诊断

- 新增 `corridor_portal_diagnostics(round_state, *, guide_points, guide_source=None)`：
  - 输入只来自当前 round 的真实 `formation_assessment`、实际选中的 Guide 点列与 Guide source；没有 assessment 时返回 `None`，不生成任何几何。
  - `map_collision_samples`：来自 `vehicle_map_collisions`，每个样本记录 `s_m`、`vehicle_id`、`station_index`、`reason`、`other_vehicle_id`；位置是 Guide 折线在评估弧长处的参考点（`GUIDE_REFERENCE_POINT_AT_ARC_LENGTH`），不是接触点或重建车体位姿。
  - `coverage_gaps`：来自各车 `coverage_vehicles[*].witnesses` 中 `status=GAP` 的真实 witness；位置优先用 witness 顶点，缺失时才回退到 Guide 弧长点；带 `region_ids`、`window_index`、`max_violation_m`。
  - `portal_windows`：来自 `transition_windows`，含起止弧长/端点、`portal_seed_point`、真实 `status`/`map_sample_status` 与逐车 union 计数。
  - 硬上限：`PORTAL_DIAGNOSTIC_COLLISION_LIMIT=5`、`PORTAL_DIAGNOSTIC_GAP_LIMIT=6`、`PORTAL_DIAGNOSTIC_WINDOW_LIMIT=3`。
  - `counts` 同时保留**完整** assessment 总数（map 碰撞、union covered/gap/unknown、Portal 窗口总数）与实际 marker 数；截断 marker 不会低报采样风险。
  - 固定 `certification_claim=NONE`、`candidate_certification=DIAGNOSTIC_ONLY`、`continuously_certified=false`、`formation_obb_hard_constraints=false`。
- `formation_uniform_planner_node.py::_display_from_corridor_round()` 在同一 `_display_from_corridor_round()` 中调用该函数，把 `portal_diagnostics` 附到 corridor display；不改变 admission/publish gate，也不改变最终 `/formation/playback_plan`。

### 1.2 `formation_playback.py`：独立图形、身份校验与失败清层

- `build_guide_display()` 在原有 A*/Guide path 与标签之后调用 `_append_portal_diagnostics()`：
  - `formation/planning/map_collision_sample`：洋红 `MAP_COLLISION_SAMPLE_COLOR=(236,72,153)/255`，`LINE_LIST` 十字 + `TEXT_VIEW_FACING`，标签固定含 `MAP COLLISION (sample) | carN | s=... | reason`。
  - `formation/planning/coverage_gap`：橙色 `COVERAGE_GAP_COLOR`，闭合 `LINE_STRIP` 菱形 + 文本，标签固定含 `CORRIDOR COVERAGE GAP (sample)`，不出现 `MAP COLLISION`。
  - `formation/planning/portal_window`：青色 `PORTAL_WINDOW_COLOR`，起止 `LINE_STRIP` + 可选 seed `SPHERE` + `PORTAL j->j+1 | status` 文本。
  - Marker ID 以 `1000+100*route_id` 为基址分区；每类标记数量受上述硬上限约束（单 route 最多 10/12/9 个标记）。
- `_portal_diagnostic_payload()` 对每个 diagnostics 块 fail closed：schema、`certification_claim=NONE`、`continuously_certified=false`、`formation_obb_hard_constraints=false`、`chain_digest == corridor.chain_sha256`、`guide_source` 与 corridor 一致、计数为非负整数、三个列表不超硬上限。
- 这些 marker 只存在于 `/formation/guides` MarkerArray；`build_guide_display()` 原 path 输出与 `/formation/playback_plan` 执行消息不变。

### 1.3 HUD 与 playback 节点

- `formation_playback.py::planning_diagnostics_view()` 只从 progress document 的 `metrics` 复制有界标量：
  `guide_source`、`selected_candidate_origin`、`selected_candidate_index`、`guide_candidate_count`、`guide_selection_rule`，以及 `assessment` 的 `available/station_count/map_sample_status/map_collision_count/union_covered_count/union_gap_count/union_unknown_count`。
  缺失字段保持 `None`/`unknown`，不计算任何百分比。
- `_hud_progress_line()` 在原有 `planning <stage> | step | route | round | elapsed` 后追加两行：
  `diagnostics | guide=... | origin=... | selected=i/n | <rule>` 与
  `samples=... | map_collisions=... | union covered=... gap=... unknown=... | status=...`；
  未完成 assessment 时打印 `samples=unknown | map_collisions=unknown | union covered=unknown gap=unknown unknown=unknown`。
- `fwis_v3_playback/scripts/formation_playback.py::_record_planning_progress()` 从同一 plan/generation/round 的 `metrics` 保留 diagnostics；同一 round 的后续 stage 不丢失已完成 assessment，新 plan/new round 不继承旧计数。
- 失败时保留本 plan 已发布的诊断图层，但不会把失败候选升级为 certified；`CLEARED`/pose reset/busy rejection 沿用 TASK01 lifecycle。

### 1.4 `formation_demo.rviz`

- 保持 TASK01 既有 `Guide labels (markers)` 显示项不变（仍 disabled，name 不变），新增第二个 `/formation/guides` MarkerArray：`Portal diagnostics (markers)`，仅开启
  `formation/planning/portal_window`、`formation/planning/map_collision_sample`、`formation/planning/coverage_gap`。
- 既有 Path 颜色未改：A* `249;115;22`、Portal Guide `34;197;94`、candidate `250;204;21`、certified `56;189;248`。

## 2. 语义与硬约束

| 项目 | 语义 |
|---|---|
| 诊断来源 | 仅当前 round 的真实 `formation_assessment` 与选中 Guide；不重新采样、不伪造 |
| map collision | 洋红 `(sample)`；仅为 Guide 参考点，不是碰撞证书 |
| corridor gap | 独立 namespace/颜色/文案；绝不显示为 map collision |
| Portal window | 仅当前 round 的有限窗口，青色起止线 + 真实 status |
| Marker 上限 | 5 collisions / 6 gap witnesses / 3 windows（单 route） |
| 认证 | `certification_claim=NONE`、`continuously_certified=false`、`formation_obb_hard_constraints=false` |
| 身份 | diagnostics 的 `chain_digest`/`guide_source` 不匹配时整层 Guide fail closed；admission 字段不变 |
| HUD | 只显示真实计数；缺失为 `unknown`，无百分比 |

## 3. 测试

命令（仓库根目录，source ROS 后）：

```bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 -m unittest \
  tests.test_virtual_a_corridor_display \
  tests.test_formation_hud \
  tests.test_virtual_a_receiver \
  tests.test_virtual_a_playback \
  tests.test_formation_playback \
  tests.test_planning_display \
  tests.test_formation_demo_assets
```

| 日志 | 结果 | 耗时 | exit |
|---|---|---|---|
| `tests_display.log` | Ran 194 tests, OK | 78.408 s | 0 |
| `tests_display_after_queue_fix.log` | Ran 194 tests, OK | 80.160 s | 0 |

新增/调整的测试（均为追加，不修改 TASK01 已验收预期）：

- `V2PortalDiagnosticsTest::test_round_export_carries_bounded_diagnostics_bound_to_the_assessment`
  - 真实 `_display_from_corridor_round()` 输出绑定 `chain_digest`/`guide_source`/`candidate_id`；
  - 碰撞样本位置与独立计算的 Guide 弧长插值误差 ≤`1e-9 m`；
  - `counts` 的 map/covered/gap/unknown/window 总数逐项等于真实 assessment 汇总；marker 数等于截断后列表长度，且 11 个 map 碰撞 > 5 个 marker。
- `V2PortalDiagnosticsTest::test_builder_truncates_at_the_marker_caps_but_keeps_full_totals`
  - 人为放大 assessment 列表后仍只输出 5/6/3 个样本，完整总数保持 11/窗口总数。
- `V2PortalDiagnosticsTest::test_diagnostic_markers_have_separate_namespaces_colors_and_labels`
  - 三类 namespace/颜色/图形；map 文本含 `MAP COLLISION (sample)`，gap 文本含 `CORRIDOR COVERAGE GAP (sample)` 且不含 `MAP COLLISION`；无 `CERTIFIED` 文本。
- `V2PortalDiagnosticsTest::test_bad_identity_cap_or_claim_fails_the_guide_layer_closed`
  - 错误 chain digest、guide source、`CERTIFIED` claim、`continuously_certified=true`、三类超上限、负计数均抛 `PlaybackValidationError`。
- `V2PortalDiagnosticsTest::test_missing_diagnostics_keeps_the_guide_layer_without_samples`
  - 旧 payload 无 `portal_diagnostics` 时仍输出原 Guide 文本层，不产生新 namespace。
- `V2GuideCallbackTest::test_invalid_portal_diagnostics_clear_only_the_guide_layer`
  - 真实 ROS callback 路径下，diagnostics 身份错误只清 Guide 层，corridor 层保留。
- `V2PlannerDebugPublicationTest::test_real_progress_document_keeps_route_guide_source_and_counts`
  - 真实 PORTAL_GUIDE event 经 `_progress_document()`/`_on_virtual_progress()` 发布为 JSON 后，仍含原始 route index、Guide source、选中候选、选择规则和真实 assessment 计数；corridor display 的 `portal_diagnostics.chain_digest` 与 `chain_sha256` 一致。
- `V2PlanningProgressDiagnosticsTest::test_completed_portal_guide_event_carries_selected_route_and_samples`
  - 真实 pipeline event 与 HUD view 的 source/候选/计数逐项一致。
- `PlanningProgressHUDTest` 新增 4 例：真实计数 HUD 文本、缺失 assessment 的 `unknown`、CORRIDOR_BUILD 无诊断时的 `unknown`、非对象 diagnostics fail closed。
- `ProgressiveLayerLifecycleTest::test_progress_diagnostics_are_retained_only_for_the_same_round`
  - 同 round 后续 stage 保留，新 round 清空计数。

## 4. 真实样本

`progress_diagnostics_sample.json` 来自 `tests/test_virtual_a_corridor_display._actual_corridor_case()` 的真实 corridor/hard 求解：

- 选中 route 0（`route_count=3`）、Guide `corridor_candidate_chebyshev`、候选 1/2、规则 `DIAGNOSTIC_LEXICOGRAPHIC_BEST`；
- 真实 assessment：`samples=64`、`map_collisions=11`、`union covered=133 gap=37 unknown=22`、`COLLISION_OBSERVED`；
- 诊断 marker 上限内实际输出：5 个 map collision 样本、2 个 gap witness、1 个 Portal 窗口；
- `certification_claim=NONE`，无连续认证或 OBB 硬约束声明。

`hud_sample.txt` 为同一真实 event 经 `hud_text()` 生成的固定角 HUD 文本，包含 route/round、Guide source、候选来源、样本数与三类 union 计数。

## 5. 默认入口证据

### 5.1 dry-run（无 DISPLAY，默认补 `rviz:=false`）

`dryrun_default.txt`（默认 map16）与 `dryrun_map05.txt`（`--map map05`）均只有一条 `roslaunch` 命令，且各参数只出现一次：

```
backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor
virtual_a_corridor_enforcement:=hard auto_goal:=false
```

无图形桌面时脚本打印 `未检测到图形桌面，自动使用 rviz:=false` 并追加 `rviz:=false`；未在测试时偷偷补 backend/guidance 参数。

### 5.2 launch dump

- `launch_dump_virtual_a.txt`：`/formation_uniform_planner/backend=virtual_rigid_a`，且
  `/formation_uniform_planner/virtual_a_guidance_mode=corridor`、
  `/formation_uniform_planner/virtual_a_corridor_enforcement=hard` 实际进入节点参数。
- `launch_dump_uniform_v02.txt`：仅 `backend=uniform_v02`，无任何 `virtual_a_guidance_mode`/`virtual_a_corridor_enforcement` 参数。
- `launch_dump_legacy.txt`：仅 `backend=legacy`，无新 virtual 参数；也不启动 planner 节点。

## 6. 未运行项

- Step E 本身未启动真实 ROS master / RViz；真实 GUI 图层与截图在 Step F 执行并另存于 `../step_f/`（`map05_default/`、`s1_history/` 及两次队列修正前的重跑目录）。本目录的 `hud_sample.txt`、`progress_diagnostics_sample.json` 为真实求解数据经真实 display/HUD 代码生成的离线样本，不是 GUI 截图。
- 未修改 `run_demo.sh` 或 `formation_demo.launch`：dry-run 与 dump 证明默认链路已满足 `virtual_rigid_a/corridor/hard`。

## 7. Step F 现场运行发现并修正的队列问题

| 位置 | 原值 | 新值 | 原因与证据 |
|---|---|---|---|
| `formation_uniform_planner_node.py` 的 `/formation/planning_debug` publisher | `queue_size=1` | `queue_size=20` | 每个阶段先发 `running`、完成后立即发 `complete`，随后下一阶段 `running` 紧跟；单槽发送队列丢 complete 文档。修正前 map05 run1（`../step_f/map05_default_run1_pre_queue_fix/`）的 playback/HUD 从未收到 ASTAR/CORRIDOR/PORTAL_GUIDE complete。 |
| `fwis_v3_playback/scripts/formation_playback.py` 的 `/formation/planning_debug` subscriber | `queue_size=1` | `queue_size=50` | 即使发布端已发全部文档，接收回调还要重建并发布各图层（10 Hz `tick` 持同一把 `@synchronized` 锁），单槽订阅队列同样丢 complete 文档。修正前 map05 run2（`../step_f/map05_default_run2_planner_queue_only/`）playback 日志仍只有 14 次 stage 记录。 |

两处修正后的 `../step_f/map05_default/`：playback 日志收到 16 次 stage 记录（stage 1–7 各 running+complete，stage 0 与 8 各一次），HUD 在 `PUBLISHED` 显示
`guide=corridor_endpoint_polyline | selected=0/1 | SINGLE_BASE_GUIDE` 与
`samples=13 | map_collisions=0 | union covered=39 gap=0 unknown=0 | status=CLEAR_AT_SAMPLES`。
这些值来自同一 progress document，与 `quality_report.json` 的 route 0 / chain `e7f8568f…` / round 0 一致。
队列大小只是本地 ROS 传输参数，不是新的认证或发布门控。