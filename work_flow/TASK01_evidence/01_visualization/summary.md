# Step 1 可视化证据（Progress / 五图层 / HUD / 失败冻结）

- 日期：2026-11-18
- BASE HEAD：`77805cd46849ebadc374187f9b00fce0f7151f84`（未提交、未 push）
- 工作目录：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- Python：3.8.10；ROS Noetic；CasADi 3.7.2

## 1. 运行环境说明

执行方案 §5.1 的字面命令只设：

```bash
export PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src
```

在本机该赋值会覆盖 ROS 与测试 helper 路径，导致 `visualization_msgs`、`fwis_v3_msgs`、
`virtual_a_g4_support` 等无法导入，测试在收集阶段即失败。实际运行采用增量前置：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

除路径设置外，命令与测试选择与 §5.1 完全一致。每条命令均单独保存 stdout/stderr、
退出码和墙钟时长（见 `g*.log` / `g*.exit_code` / `g*.duration`）。

## 2. §5.1 七组结果

| 组 | 命令（测试模块） | 用例数 | 时长 | 退出码 | 日志 |
|---|---|---:|---:|---:|---|
| G1 | `tests.test_virtual_a_corridor_display tests.test_formation_hud` | 66 | 21.044 s | 0 | `g1_corridor_display_hud.log` |
| G2 | `tests.test_virtual_a_receiver tests.test_virtual_a_playback` | 49 | 59.966 s | 0 | `g2_receiver_playback.log` |
| G3 | `tests.test_formation_demo_assets` | 21 | 1.811 s | 0 | `g3_demo_assets.log` |
| G4 | `tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer` | 53 | 80.500 s | 0 | `g4_corridor_optimizer_optimizer.log` |
| G5 | `tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery` | 26 | 133.772 s | 0 | `g5_pipeline_recovery.log` |
| G6 | `tests.test_virtual_a_geometry tests.test_virtual_a_profile` | 42 | 2.852 s | 0 | `g6_geometry_profile.log` |
| G7 | `tests.test_virtual_a_integration_bounds tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit tests.test_virtual_a_trajectory` | 97 | 7.105 s | 0 | `g7_integration_audit_trajectory.log` |

补充（不在 §5.1 七组内，但本次修改了同文件的旧 `_publish_corridor_debug` 测试）：

| 组 | 命令 | 用例数 | 时长 | 退出码 | 日志 |
|---|---|---:|---:|---:|---|
| G8 | `tests.test_virtual_a_ros_adapter` | 60 | 17.336 s | 0 | `g8_ros_adapter_supplemental.log` |

七组合计 354 个用例；含补充共 414 个用例，全部 `OK`。

## 3. Step 1 定向验收对应

| 验收项 | 证据（测试） |
|---|---|
| 真实 stage 顺序、允许回跳、pipeline 不发 `PUBLISHED` | `ProgressCallbackContractTest.test_real_success_events_are_monotone_read_only_and_identifiable` |
| callback 异常只记录一次且不影响规划 | `ProgressCallbackContractTest.test_callback_exception_is_recorded_and_disables_later_events` |
| 错误 plan/generation/grid/context/request/schema/state/stage/stage_index 零发布 | `ProgressiveDocumentTest.test_wrong_identity_or_generation_publishes_nothing` |
| 失败冻结已完成图层（A*/corridor/candidate 保留，stage_index 为最后真实阶段） | `ProgressiveDocumentTest.test_failed_document_freezes_completed_layers`；`ProgressiveLayerLifecycleTest.test_heartbeat_failed_and_raw_request_never_clear_layers` |
| 新 admission 清空三诊断层；busy/heartbeat/raw request/`_clear` 不清 | `ProgressiveLayerLifecycleTest.test_new_planning_admission_clears_all_three_layers`；`test_heartbeat_failed_and_raw_request_never_clear_layers` |
| 图层渐进与 `null` 语义（A*-only、corridor null、candidate null + failure、candidate null 清理） | `ProgressiveLayerLifecycleTest` 其余 4 个用例 |
| candidate 点逐点等于真实 `state_at_s` | `ProgressiveDocumentTest.test_candidate_is_pointwise_from_the_real_state_at_s`（F/中心点误差 ≤ 1e-12 m） |
| 三车刚性距离误差 ≤ 1e-10 m | 同上（各站两两中心距 max−min ≤ 1e-10 m） |
| OBB 角等于实际车辆 heading | 同上（首角与 `jet.theta` 公式误差 ≤ 1e-12 m） |
| 未认证 candidate 不得升级为 CERTIFIED、不得触发 playback | `CandidateMarkerTest.test_certified_payload_requires_a_published_document`；`test_unverified_candidate_uses_stable_namespaces_and_labels` |
| `PUBLISHED` 仅在原子信封实际发布后 | `test_virtual_a_ros_adapter.VirtualEnvelopePublicationTest` 记录发布顺序 `["envelope", "planning"]`，且失败/审核未通过路径无 `PUBLISHED` |
| 错误/畸形 corridor 显示只清该诊断层，不改变规划结果 | `test_virtual_a_ros_adapter.CorridorDisplayConstructionTest`；`V2PlannerDebugPublicationTest.test_missing_corridor_round_clears_only_that_layer` |
| RViz 固定名称/命名空间、A* 默认开启、新 candidate display | `tests.test_formation_demo_assets`（G3） |

## 4. 结论边界

本文档只记录 Claude 自测通过，不声明 Codex PASS，也不声明 TASK 01 整体验收通过。
`PUBLISHED` 语义按用户勘误执行：由 ROS 节点在 `playback_plan_pub.publish(envelope)`
之后发出；pipeline 只发出 `ASTAR_SEARCH`…`OBB_COLLISION_AUDIT` 的真实阶段事件与
`FAILED`（`REQUEST_ADMITTED` 由节点在 admission 后发出，`PUBLISHED` 由节点在信封
发布后发出）。未 commit、未 push。