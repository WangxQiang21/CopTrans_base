# FR05 最小返工报告（rework_20261114T095659Z）

- 依据：`evaluation/formation_refactor_v02/FR05/codex_acceptance_20261114T093751Z/ACCEPTANCE.md` §5
- 代码基线：git `6227833011e2d92b711179bfdec979bb0b878b29`（worktree `6227833-dirty`）
- 本轮目录：`evaluation/formation_refactor_v02/FR05/rework_20261114T095659Z/`
- 原始 FR05 证据目录未被改写；本轮**未提交 Git**，**未做 FR06**。

## 0. 结论

| 项 | 状态 | 证据 |
|---|---|---|
| R1 冻结默认值 + 节点/launch 统一 + 删除 0.16 冒烟验收逻辑 | 完成 | `formation_demo.launch` 52–59 行；`formation_uniform_planner_node.py` 336–358 行；`grep 0.16/50000/20.0` 无命中 |
| R1 发布目标可实现终端，但仍按原始请求的冻结容差严格校验 | 完成 | main 报告 `*_achieved_goal_within_frozen_request_tolerance`（4 个实验全 true） |
| R2 模型摘要贯穿审计标签 / Marker+状态 / 作业反馈 | 完成 | 代码 + 7 个 `AuditIdentityBindingTest` + 6 个 `FeedbackIdentityTest` + main 的 `playback_status_carries_envelope_identity` 等 |
| 同 plan_id 不同/缺失摘要：不得 FEASIBLE/可执行，HOLD_END 不得释放作业 | 完成 | `test_same_plan_id_other_digest_never_certifies_the_display`、`test_wrong_digest_terminal_does_not_release_busy` 等 |
| 定向正/负例、相关单元回归 | 完成 | `logs/units_related_full.log`：Ran 134 tests，OK |
| 私有 master + 专用 ROS_HOME 的 main / near-goal / failure / tamper 实跑 | 完成 | `ros/`、`ros_near_goal/`、`ros_failure/`、`ros_tamper/` 全部 `ok=True` |
| 至少一次可见 RViz 往返（100 Hz 定量、GUI 定性） | 完成 | `rviz/round1/`（round1/2/3 + 意外远目标 NOTE） |
| 失败尝试、Codex 探针全部保留 | 完成 | `ros_attempts/main_attempt1|2`、`near_goal_attempt1`、`ros_probes/` |
| 冻结源码 + manifest | 完成 | `SOURCE_MANIFEST.sha256`（174 条，`sha256sum -c` 全部 OK） |

主场景（empty_map，100 Hz，冻结默认值）**112/112 检查为 true**；
near-goal 负例在**完整** 30 s 预算下**诚实失败**（未伪造可行）；
failure 场景 57/57；tamper 场景 14/14。

## 1. R1：冻结契约与统一入口

### 1.1 launch 与节点使用同一组冻结默认值

`catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch`：

```
max_expansions=10000
uniform_max_seconds=30.0
uniform_goal_xy_tol=0.02
uniform_goal_psi_tol=0.017453292519943295   # 1 deg
uniform_translation_step=1.0
uniform_collision_margin=0.05
uniform_sweep_spacing=0.01
```

节点 `formation_uniform_planner_node.py` 在参数缺失时回退到 FR03/FR04 的
`SuccessorConfig` / `GraphSearchConfig` 默认值（不再重复字面量），因此
launch 与直接 rosrun 两条路径的默认值一致。`formation_demo_assets` 的
`test_launch_defaults_match_the_frozen_contract` 与
`test_uniform_planner_defaults_reference_frozen_config` 锁定这两点。

### 1.2 删除“0.16 当冻结值”的冒烟验收逻辑

`grep -n "0\.16\|50000\|20\.0" tests/ros_uniform_playback_smoke.py
catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py
catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch` 无任何命中；
冒烟脚本使用 `FROZEN_GOAL_XY_TOL_M=0.02` /
`FROZEN_GOAL_PSI_TOL_RAD=radians(1)` / 10000 / 30 s，并在
`check_frozen_parameters` 中核对 ROS 参数确为冻结值（见各报告中的
`frozen_*` 检查）。

### 1.3 发布目标 = 可实现终端，但仍严格对原始请求

`run_experiment()` 计算 `achieved = summary["goal_qf"]` 与
**原始请求** `result["goal"]` 的 XY / 航向误差，并要求同时满足冻结容差：

- main 的 4 个实验中该检查均为 true；`rviz_style` 的十进制请求
  `(12.0, 12.0, 0.018)` 实测误差为 **0.000169 m / 0.005000 rad**
  （`rviz_style_achieved_goal_inside_frozen_tolerance` 亦为 true）。
- 这意味着“把消息目标改成实际到达点”不会被当成达成原目标。

## 2. R2：摘要身份贯穿与 fail-closed

### 2.1 数据流

`envelope.model_sha256`（`model_sha256()` 由 model_json 计算）
→ playback 的 `status_document(...)` / Marker 文本 / 审计标签
→ `/formation/optimizer_status` 的 `model_sha256`、`formation_id`、`backend`
→ planner 的 `on_playback_status` 身份门。

- `fwis_v3_optimizer/formation_playback.py::_status_label()`：当显示的是
  verified uniform envelope（摘要存在）时，状态里的
  `model_sha256` / `formation_id` / `backend` 必须与之完全一致才可能得到
  `FEASIBLE` / `NO_MOTION`；不一致只保留 `GEOMETRY_ONLY` 并给出
  “identity does not match” 原因。legacy 轨迹传 `None`，保持历史
  plan_id-only 规则。
- `formation_uniform_planner_node.py::on_playback_status()`（1070–1137 行）：
  `HOLD_START/PLAYING/HOLD_END/CLEARED/INVALID` 之外的状态忽略；
  摘要/formation/backend 任一不匹配即 `logwarn_throttle` 并返回——不设置
  `_playback_seen`、不升级状态、不释放 busy。
- 复用既有字段，没有新增并行状态机。

### 2.2 定向用例（全部通过）

`tests/test_uniform_playback_plan.py::AuditIdentityBindingTest`（7 个）：

- `test_same_plan_id_other_digest_never_certifies_the_display`：两个**各自
  自洽**的 envelope（同 plan_id、不同路径/摘要）互不认证——正是指南要求的
  “两次自洽但身份不符”负例，而不是拿一个自相矛盾的 envelope 充数；
- `test_missing_identity_fields_never_certify_the_display`；
- `test_matching_identity_certifies_the_display`（正例）；
- `test_status_document_rejects_a_digest_for_another_envelope`；
- `test_verified_envelope_carries_the_authoritative_digest`；
- `test_zero_motion_hold_still_certifies_with_matching_identity`；
- `test_legacy_trajectory_keeps_the_plan_id_only_audit`。

`tests/test_uniform_feedback_identity.py::FeedbackIdentityTest`（6 个）直接
驱动真实的 `UniformPlannerNode.on_playback_status`（`__new__` + stub
publish，patch `logwarn_throttle`）：

- 正确摘要 HOLD_END 释放 busy 并升级为 certified terminal（正例）；
- 错误摘要 / 缺失身份 / 错误 formation/backend 的 HOLD_END **不**释放；
- 错误摘要的 HOLD_START **不**设置 `_playback_seen`。

另外 main 实跑中的 `playback_status_carries_envelope_identity`、
`playback_terminal_is_certified_for_that_digest`、
`status_model_digest_matches_envelope`、`job_model_digest_matches_envelope`
为 true；`final_job_hold_end` 的 plan 600003 摘要为
`fdfd0e5c560b39ea3fa368af4bdefe99c1b85b871596322d568714d6a368d6c6`。

## 3. ROS 实跑结果（私有 master 11472，专用 ROS_HOME）

### 3.1 main（empty_map，`playback_rate:=100`）

`ros/main_report.json`：`ok=True`，112 项检查全部 true。4 个实验：

| 实验 | plan | 请求 (x,y,psi) | T_end (s) | 严格请求误差 (m/rad) | 显示内部时刻最小值（tf/marker/status） |
|---|---|---|---|---|---|
| forward | 600000 | (12,10,0) | 6.2455 | 0 / 0 | 293 / 291 / 292 |
| lateral | 600001 | (10,12,0) | 15.4291 | 0 / 0 | 96 / 92 / 111（motion_0003 / setup_0002） |
| rviz_style | 600002 | (12,12,0.018) | 7.0119 | 0.000169 / 0.005 | 25 / 25 / 25（initial_setup_0000） |
| s_to_a | 600003 | (12,11,0.6) | 21.4436 | 1.1e-16 / 0 | 13 / 11 / 13（initial_setup_0000） |

- 每段相位、每类来源（TF / Marker / `/formation/playback_status`）均
  ≥11 个内部显示时刻（要求 11，未下调）；s_to_a 的 marker
  `initial_setup_0000` 恰好 11。
- 共享时间轴 3 车均 1078 个样本（`shared_time_axis_lengths`），s_to_a 的
  `T_end=21.4436 s`、planner wall 15.29 s；S 相位中仅 1 号车航向变化
  `0.4636476 rad`，其余为 0（`unified_S_phase_has_zero_angle_vehicle`）。
- 消息顺序：`REQUEST_VALIDATED → PUBLISHED → PLAYING → HOLD_END`；
  busy 期间拒绝新的 goal / initial pose；终止后的第二次请求从 playback
  终点出发（`second_request_started_from_playback_terminal = [12,10,0]`）。
- 起始/终止位姿与角度检查、模型 vs 样本、Marker/TF/status 三者一致性、
  envelope 版本/后端/认证字段全部为 true。

### 3.2 near-goal 负例（Codex 探针的真实请求）

`ros_near_goal/near_goal_report.json`：`ok=True`，plan 600005，
`(10,10,0) → (10.1,10,0.1)`（偏离 0.1 m / 0.1 rad，超出冻结容差）。
结果 **诚实失败**：`FAILED / TIME_BUDGET_EXHAUSTED / layer=budget`，
`wall=30.0479 s`，97 expansions、17013 candidates、collision 22.56 s，
`A_HEADING_REQUIRES_S_ALIGNMENT=11468`，`goal_state=null`。
没有出现历史上“贴近目标却判 NO_MOTION”的误判；也没有为通过而放宽搜索/加宽
阈值（指南允许诚实失败）。8/8 检查为 true，包括冻结容差与预算核对。

### 3.3 failure（blocked_map）

`ros_failure/failure_report.json`：`ok=True`，57/57。基线 plan 600000
成功；被阻挡目标 plan 600001 在 30.764 s 诚实 `TIME_BUDGET_EXHAUSTED`；
playback 收到无效占位 envelope 后 `CLEARED`、`executable=False`、
`marker_count=0`；旧动画停止（marker/TF 位移 0.0 m）；冻结保持位置误差
0.0 m；重试 plan 600002 从 `(11,10)` 出发并到达 HOLD_END（T_end 3.123 s）。

### 3.4 tamper（empty_map，`planner:=false goal_bridge:=false`）

`ros_tamper/tamper_report.json`：`ok=True`，14/14。以 main 的
`envelope_00600000.json` 为基线，6 类篡改
（unknown_semantic_version / unknown_model_version / digest_mismatch /
tampered_model / tampered_sample / stamp_mismatch）全部 `CLEARED`、
`executable=False`、`marker_count=0`、原因明确；随后晚到的 legacy 轨迹
（79 条文档，同一 plan 600000 的 PLAYING）被忽略
（`late_legacy_trajectory_is_ignored`）。

### 3.5 状态探针

`ros_probes/status_probe.py` + `status_probe_1.json`：plan 600001 的
`REQUEST_VALIDATED → PUBLISHED → HOLD_END`（optimizer）与 playback
`HOLD_END executable=True`，摘要 `b0c895…` 全程一致，
`hold_end_certified=[True×6]`。该探针在私有 master、同一套代码上运行，
时间早于四个受控场景，作为补充证据保留，不计入上面的场景统计。

## 4. 可见 RViz 往返（GUI 定性）

`rviz/round1/`，真实 `xdotool` 点击 2D Pose Estimate / 2D Nav Goal
（世界坐标由 RViz 自身发布的 `/initialpose`、`/move_base_simple/goal`
记录）：

| 轮次 | 输入（世界） | plan | 结果 |
|---|---|---|---|
| round 1 | (9.987,10.007)→(12.016,10.007) | 600000 | T_end 12.762 s，PLAYING/HOLD_END `executable=True`，界面 “FEASIBLE and published” |
| round 2（缩放后） | (11.401,9.670)→(11.681,9.670) | 600001 | T_end 2.042 s，HOLD_END FEASIBLE，文字可读 |
| 意外远目标 | (11.681,9.670)→(12.908,6.172,-1.524) | 600002 | 诚实 `BUDGET_EXHAUSTED ... wall=30.180 s`；playback CLEARED；忙时拒绝后续点击（`NOTE_stray_round.md`） |
| round 3（重做缩放轮） | (10.003,9.999)→(12.002,9.999) | 600003 | T_end 6.244 s、316 样本、摘要 `f8d0717c…`；PLAYING/HOLD_END FEASIBLE；`run3_08.png`（PLAYING）、`run3_45.png`（HOLD_END，终端 0.000 m） |

截图：`00_initial.png`、`01_resized.png`、`02_pose_set.png`、
`play_01..44.png`、`zoom_01..60.png`、`run3_01..60.png`、
`zoom_after_round2.png`、`zoom_mid_round1.png`；输入记录 `clicks.json`、
`run3_meta.json`。100 Hz 的定量检查在 3.1（GUI 只作定性）。

## 5. 保留的失败尝试与缺陷（未隐藏、未覆盖）

1. `ros_attempts/main_attempt1/`：`initial_setup_0000` 每个来源只有 9 个
   内部显示时刻（阈值 11），`note()` 抛 `SmokeError`，退出码 1。保留其
   报告、日志、envelope。
2. `ros_attempts/main_attempt2/`：`playback_terminal_is_certified_for_that_digest`
   得到 `[False, True, True]`——playback 缓存的审计状态在发布瞬间仍是
   `REQUEST_VALIDATED`（无摘要），首个 HOLD_END tick 因此 fail-closed，
   随后 planner 应答 HOLD_END 的认证状态到达，后续 tick 为 True。检查改为
   “2 s 内必须对**该**摘要到达认证”，未削弱跨身份负例（由单元测试覆盖）。
   退出码 1；NOTE 保留。
3. `ros_attempts/near_goal_attempt1/`：smoke 手工轮询把 planner 在诚实预算
   失败后发布的**无效占位** envelope 当成可执行计划，`envelope_certification`
   为空串而抛错（退出码 1）。已修正为“先看 FAILED，且要求 model_json 与
   时间轴非空”，失败报告保留。
4. `rviz/round1/NOTE_stray_round.md`：缩放假光标定期间的一次意外 GUI 远目标
   （heading −1.524 rad）被受理为 plan 600002，30.180 s 后诚实失败；这不
   属于受控探针，但原样保留，且当时的搜索细节没有落盘（文中明确说明未引用）。
5. 早前 85 项回归日志缺少的三个模块（`test_uniform_playback_plan`、
   `test_uniform_feedback_identity`、`test_formation_demo_assets`）已并入
   本轮 134 项回归。

## 6. 预算声明

- ROS 默认预算始终是冻结的 **10000 expansions / 30 s**，没有改动。
- 本轮所有实跑负例（near-goal 600005、failure 600001、意外远目标 600002）
  都使用**完整** 30 s 预算并因真实预算耗尽而失败；没有为了制造负例而调小
  默认预算。
- 单元测试 `test_uniform_pipeline` 的 `F11` 在**测试自身**的配置里故意设置
  极小扩张预算以触发 `EXPANSION_BUDGET_EXHAUSTED`（0.297 s 返回），这属于
  单元级受控负例，不是 ROS 默认值；在此显式记录。

## 7. 未改动项

- 未改 FR01 两个已验收文件、FR02/FR03/FR04 实现、冻结计划/决策、旧测试
  期待、车辆尺寸/offset/r_min/轮角/轮速/安全阈值、旧实验与 Codex 证据。
- 未新增 NLP、抢占、消息架构或 FR06。
- 未 `reset/stash/clean`，未操作原仓库 `expv3_multi`，未创建 worktree，
  未提交 Git。
- 本轮只改了：统一 planner 节点、统一 launch 参数、playback 库/节点的
  身份绑定分支、对应的 FR05 测试（含 `test_formation_demo_assets` 的
  launch 默认值检查）以及本目录证据。所有冻结阈值保持不变。

## 8. 冻结清单

`SOURCE_MANIFEST.sha256` 覆盖 `catkin_ws/src`、`tests`、`tools` 与
`run_demo.sh` 共 174 个文件（排除 `__pycache__`/`.pyc`），
`sha256sum -c` 全部通过。关键摘要：

```
3175e62d22ede7d7b0620b81910e254435c19ff333f8d4a9bd415f46a9843d24  catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py
bbdd6fab86729899d59fa41c470390c005625431df45bc70e6766a25a893d3ed  catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py
b9a34bfb57526f713987e28d1083d7cb566d82d1668eea9703831f07eddc03cd  catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch
fedef53f8fa6819b71861b0dcf319ed38c8a38b3c2df211b9f8dee1cfb66256f  tests/ros_uniform_playback_smoke.py
a4ebb3103c5ac878a70e06c551b8cf30cf961f1cca7aafe45e03b3cfc5dc051f  tests/test_uniform_playback_plan.py
4bf36e7c6a54d14dbb9646dc02d290e68394dca8745e26dbb73b779a0c71c440  tests/test_uniform_feedback_identity.py
51e82d67802da1a23791f81f6643c8b396bf85520d402c7e4ef34849b7fa5447  tests/test_formation_demo_assets.py
```

## 9. 诚实边界

- 显示时刻检查对启动阶段负载敏感：attempt 1 在同一主机上只有 9 个 tick，
  attempt 3 在空闲主机上通过（s_to_a 最低为 11）。阈值没有下调；失败尝试
  原样保留，说明该检查存在余量很小的情况。
- “终止认证”是**最终**性质（≤2 s 到达），不是“每个 HOLD_END tick 都必须
  已认证”；身份不符永不认证由单元测试锁定。
- RViz 轮次是定性证据；定量证据来自 100 Hz demo 的 report。
- near-goal 与 blocked 目标均诚实失败；本轮没有新增搜索能力，也没有放宽
  任何阈值来换取成功。