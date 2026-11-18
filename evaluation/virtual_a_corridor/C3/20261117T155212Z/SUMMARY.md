# C3 多通道选择与有限失败恢复 — 交付摘要

运行目录：`evaluation/virtual_a_corridor/C3/20261117T155212Z/`
日期：2026-11-17（UTC）；本文件生成于 2026-11-18（UTC）。
阶段：仅 C3。未进入 C4，未启动 ROS/显示，未提交 Git。

## 1. 结论（先看这里）

- C3 源码改动仅在允许范围内：`pipeline.py`、`optimizer.py`、`contracts.py`、
  `tests/test_virtual_a_pipeline.py`、新增 `tests/test_virtual_a_corridor_recovery.py`。
  未改模型、审核器、时间律、车辆参数/容差，未增加全局搜索框架。
- 三条定向测试命令全部通过（exit 0）：10 + 29 + 8 = 47 tests，无失败。
- 调度上限在全部证据中成立：A* 候选 ≤3、单次 solve 初值 ≤6、恢复 ≤1。
- 预算耗尽后不启动新 solver：专项 fake-solver 计数测试通过；S3 corridor
  第 2 轮在几何可行后、containment 之前按预算停止，未再进入 audit。
- 0 次误发布/false publication：所有非 executable 结果均无 plan/path；
  唯一成功结果（补充 legacy S3）三层审核全部通过。
- 同输入 C2→C3 对比：S1、S2 最终状态/原因不变；S3 由
  `COLLISION_FAILED/MAP_OBB_COLLISION` 变为
  `BUDGET_EXHAUSTED/SOFT_BUDGET_EXHAUSTED`。两者都不可执行、都未发布。
  corridor 模式 C3 成功率为 0/3；任务不要求成功率提升。
- 有限恢复的正例由补充 legacy S3 实测给出：首轮
  `COLLISION_NUMERICAL_FAILURE`，唯一一次 `endpoint_only_initial` 恢复后，
  第二轮以 6 个真正 endpoint-only 初值成功，三层审核通过，plan 可执行。

## 2. 基线与前置

| 项 | 值 |
|---|---|
| 工作区 | `/home/wq/whut_thesis/expv3_multi_virtual_a` |
| HEAD | `9a19fd3e22d33188b13795f695757c3e4a36909f` |
| 分支 | `feature/virtual-rigid-formation-a` |
| 工作树 | 未提交（C2/C2-B 改动及本 C3 改动均在 worktree 中） |
| C3 基线 | 由 HEAD + C2-A/C2-B/C2-B-rework diff 在 `/tmp/c3_pre` 重建；6 个文件 sha256 与 `start_state.log` 完全一致 |
| 前置 | C2-B rework 01 证据 `evaluation/virtual_a_corridor/C2B_REWORK_01/20261117T151325Z/`；用户 2026-11-17 明确下令 “进入C3任务” |
| Codex ACCEPTED | 本工作区无 ACCEPTED 产物；Claude 不声称已获 ACCEPTED |

本阶段未新建分支、未 commit、未 push。

## 3. 变更范围（C3-only diff）

基线 `/tmp/c3_pre` 的六项 sha256 已核对等于 `start_state.log`；
`source.diff`（1306 行）只含以下 5 个文件。

| 文件 | 行数变化 | 内容 |
|---|---|---|
| `.../formation_continuous_a/pipeline.py` | +328 −111 | `MAX_RECOVERIES=1` 替换 `MAX_COLLISION_RETRIES=2`；候选筛选/直连 LOS；单路线逐轮求解；一次恢复动作选择；预算检查；search 记录 |
| `.../formation_continuous_a/optimizer.py` | +51 −10 | `make_initial_guesses` 每路线 1 个 + endpoint-only 形状；`initial_count` 记录 |
| `.../formation_continuous_a/contracts.py` | +3 −0 | `GeometryResult.initial_count` 记录字段 |
| `tests/test_virtual_a_pipeline.py` | +16 −2 | 三层成功断言加入 C3 上限与 search 字段 |
| `tests/test_virtual_a_corridor_recovery.py` | +406 −0（新） | 10 个 C3 调度/恢复测试 |

当前源码 sha256（完整值见 `source_hashes.txt` / `results.json`）：

- `pipeline.py` `7e825526653adb50824a11ac08bd030f8c3a74dd6cbf9f655d61f4d977bcb4b7`
- `optimizer.py` `25b6dc3cc3185435e30de7570a96eba2f5d9e5339999bd4c79f3ab6b4ef255be`
- `contracts.py` `bf46feceb404022f023f6b50a41928b22e23a5ace963071f53e0800b434f1a51`
- `test_virtual_a_pipeline.py` `0620f9da5fad6cb602d7ffb690d70b14d2e0726c453651d7ba406899a4e4649e`
- `test_virtual_a_corridor_recovery.py` `924ee5d49f8fb2ae6a755d0b2f9cdfddd3b9fc9de38a0dd7be3c4afb4c547332`

## 4. 实施合同逐条对照

1. **候选 ≤3 / 初值 ≤6 / 60 s / 单 solve 30 s**
   `RouteGuideConfig.max_routes=3`；`MAX_INITIALS=6`（`make_initial_guesses`
   再取 `min`）；`G4_TOTAL_SOFT_BUDGET_S=60.0`；每次 solver 调用取
   `min(planner_config.max_seconds, remaining())`，即 ≤30 s。route 生成、
   corridor build/guide/allocation、solve、containment、retime、dynamics、
   collision 之前都有 `remaining()` 检查；超时如实记录，库不能立即中断，
   `hard_real_time_guarantee=False`。IPOPT success 不直接胜出：必须通过
   三层审核；已有 audited 候选不会被后续更低目标值/未审候选否决
   （由 C2 逻辑沿用，`test_audit_rejection_survives_later_solver_failure`、
   `test_timeout_preserves_only_a_previously_audited_candidate` 复核）。
2. **开阔直连与路线多样性**
   `_routes_shortcut_equivalent` 用简化路线顶点两两精确
   `segment_is_free`（C1 LOS）做 shortcut 等价筛查；仅当两两互相可视才
   视为同一候选。障碍分离的通道（单盒左右两侧）因存在不可视交叉对而
   保留。仅当 `not blocked and distance>0 and
   segment_is_free(start,goal)` 时添加 `open_direct_los`；它只是拓扑候选，
   不是三车证书。共享 `max_routes` 容量，不新增 A*；轻微狗腿被合并。
   明示：该筛查不是严格同伦分类，不声称完备。
3. **六初值分配**
   有 route 时 `make_initial_guesses`：每条不同 route 至多 1 个
   （重复 guide 不占槽），然后从 `DEFAULT_INITIAL_LENGTH_MULTIPLIERS[:2]`
   （1.2、1.8）× 两个方向补 endpoint 形状，去重 (长度, 方向)；
   route 初值不会被 endpoint 形状挤掉。无 route 分支保持历史 G2 六初值
   （三种长度 × 两方向），不硬套走廊。新 endpoint 初值不带
   `route_index/route_source/route_grid_sha256`；corridor 模式下只有
   `map_route_guide` 初值获得 `route_points` 高权重目标，endpoint 初值
   只受硬半空间约束（专项 fake-solver 参数记录测试）。
4. **唯一一次恢复，替换旧重试**
   `MAX_RECOVERIES=1`，原 `MAX_COLLISION_RETRIES=2` 删除；没有旧循环外的
   第二层。按当前首次失败类别只选一种动作：
   - 数值不收敛（`SOLVER_NOT_CONVERGED` / `ALL_ATTEMPTS_NUMERICAL_FAILURE`）：
     优先换尚未试的已有 route；无剩余 route 时花费同一次恢复切到真正
     endpoint-only 初值族（legacy 下即 G2 无路线六初值；corridor 下仍保留
     硬 halfspaces，只丢 guide 高权重目标）。
   - C 退化（corridor build/guide/allocation/chain audit 失败）：
     换一条已生成的 route；没有剩余 route 则 fail closed 返回具体原因，
     不放宽 allocation/interval/audit。单次 seed 切分未实现（见 §8）。
   - 固定 σ 失败：未实现相邻 portal 重分配（`corridor.py` 不在 C3 允许
     文件内）；失败仍以原审核原因 fail closed。
   - OBB 失败：优先换尚未试的不同 route；无剩余 route 时只用真实碰撞
     位置 `location.qf`（无位置则 `blocked_points` 为空，不恢复）生成
     0.20 m 局部封堵，source 明示
     `collision_recovery_local_block_heuristic`；重新生成 guide 并重新
     通过全部审核。`route_weight_scale=1.0`，不再使用旧“第二次重试
     放宽”逻辑。
   - 预算耗尽：直接返回 `BUDGET_EXHAUSTED/SOFT_BUDGET_EXHAUSTED`。
   timeout 不是数值失败，不触发恢复/回退。
5. **solver=auto 与候选排序**
   沿用 C0 的一次 IPOPT→SLSQP 规则，计入同一 attempts 池；timeout 后
   不启动 SLSQP（fake-solver 测试以 `AssertionError` 断言未调用）。
   完整三层审核通过才进入 executable；失败结果不携带 plan/path。
6. **search 记录**
   `search` 含 `route_count`、`initial_count`、`recovery_count`、
   `first_failure_layer`、`last_reason`、`recoveries[]`、`rounds[]`、
   `budget{total_soft_budget_s,max_recoveries,planner_config_max_seconds,
   route_config,hard_real_time_guarantee}`；`timings` 含
   `route_generation_s / geometry_solve_s / retime_s / dynamics_audit_s /
   collision_audit_s / result_assembly_s`，corridor 另含
   `corridor_build_s / corridor_guide_s / corridor_allocation_s /
   corridor_containment_s`。沿用既有 `attempts` 结构，无多级异常体系。
   ROS 失败的不发布/清理语义未改动（本阶段未运行 ROS）。

## 5. 测试结果（全部真实执行）

| 命令 | 起止 UTC | exit | 结果 |
|---|---|---|---|
| `test_virtual_a_corridor_recovery.py` | 16:48:03–16:48:27 | 0 | 10 tests, OK, 24.358 s |
| `test_virtual_a_optimizer.py` | 16:48:27–16:49:37 | 0 | 29 tests, OK, 69.726 s |
| `test_virtual_a_pipeline.py` | 16:49:37–16:52:53 | 0 | 8 tests, OK, 195.462 s |

日志在 `test_logs/`，时间/退出码在 `test_logs/run_meta.log`；旧日志未覆盖。
`test_virtual_a_pipeline.py` 的六冻结场景 S1/S1_mirror/S2/S2_mirror/S3/
S3_mirror 全部三层审核成功；其中 S3 使用一次 endpoint-only 恢复
（成功允许，上限仍为 1）。

任务要求的测试覆盖映射：

| 合同要求 | 测试 |
|---|---|
| 单盒左右拓扑 + 开阔狗腿去重 | `test_single_box_keeps_both_obstacle_separated_sides`、`test_open_grid_merges_slight_doglegs_into_one_candidate` |
| endpoint 不强贴 route0 | `test_corridor_endpoint_initials_are_not_pulled_to_route_zero`、`test_initial_slots_are_one_per_route_plus_endpoint_shapes` |
| 伪求解成功但 audit 失败 | `test_fake_solver_success_but_audit_rejection_spends_no_recovery` |
| 模拟 timeout 不 fallback | `test_simulated_timeout_never_starts_the_slsqp_fallback` |
| OBB 失败只换一次路线 | `test_obb_failure_switches_to_one_untried_route_then_stops` |
| 恢复仍失败无 plan | 上述 OBB 测试 + `test_numerical_failure_spends_one_endpoint_only_recovery` |
| 已全审候选在后续超时仍保留 | 既有 `test_timeout_preserves_only_a_previously_audited_candidate`（optimizer 套件） |
| fake solver 计数预算边界 | `test_budget_exhaustion_between_route_generation_and_solve` |
| 一个真实成功 + 一个真实失败请求 | `test_real_success_and_real_failure_requests` |

未反复运行全套 ROS 测试。

## 6. C2 对 C3 同输入变化（每场景一次）

输入同一性：`request_sha256`、`grid_sha256`、planner config、guidance mode
均已逐项核对为相同（`corridor_cases/C2_vs_C3.json` 中四个 same 标志全为
true）。C2 行取自已交付的 C2-B 原始证据文件，本轮**未重新执行 C2 代码**。

| 场景 | C2 | C3 | 变化 |
|---|---|---|---|
| S1 | `GEOMETRY_FAILED/CORRIDOR_AUDIT_WIDTH_LIMIT`，36.175 s，4 attempts（3 audited + 1 timeout） | 同状态/原因，35.255 s，4 attempts 全部 audited；routes 3，initials 5，recovery 0 | 最终层级结果不变；C3 初值分配使第 4 次由 timeout 变为 audited，但 containment 仍拒绝 |
| S2 | `BUDGET_EXHAUSTED/SOFT_BUDGET_EXHAUSTED`，35.686 s，2 attempts（failed + timeout） | 同状态/原因，35.925 s，2 attempts；routes 3，initials 5，recovery 0 | 不变 |
| S3 | `COLLISION_FAILED/MAP_OBB_COLLISION`，64.938 s，7 attempts，2 轮 OBB 碰撞 | `BUDGET_EXHAUSTED/SOFT_BUDGET_EXHAUSTED`，61.533 s，7 attempts；routes 1，initials 5，recovery 1 `collision_local_block`；第 0 轮真实碰撞 `vehicle 3`、`station 4.103667`、`sample 124`、`segment 2`、`t=11.4856 s`；第 1 轮几何可行但预算在 containment 前耗尽 | 第 0 轮同样观测到真实 OBB 碰撞；C3 用真实碰撞位置做唯一一次 heuristic 局部封堵，第二轮未能在 60 s 软预算内完成 audit，因此最终状态变为预算耗尽。两者均不可执行、均无 plan/发布 |

补充正例（legacy 模式、同一 S3 输入、最终代码）：
`corridor_cases/S3_legacy_c3.json`，`SUCCESS`，49.817 s，routes 1，
初值 6，恢复 1（`endpoint_only_initial`，t=17.563 s），首失败层
`COLLISION`/`COLLISION_NUMERICAL_FAILURE`，11 attempts（9 audited +
2 solver failed），三层审核全通过，`checks=12110`，path 长 12.093 m；
`model_binding_ok=true`，plan/result 的 request、planning-context、model
三个 sha256 逐项相等（model `f489cdb63036bd87223262e467c438b6b7c8fc2270bbdf8d1a4fd26bc21f9300`）。
该运行是纯 Python，不涉及 ROS 发布。

## 7. 分层结果（求解 / 几何 / 动力学 / 碰撞 / 发布·显示）

| 运行 | solver | 几何 | 动力学 | 碰撞 | 发布/显示 |
|---|---|---|---|---|---|
| S1 corridor | 4/4 `SOLVER_SUCCESS_AUDITED` | 求解几何可行；连续走廊 containment 因 `CORRIDOR_AUDIT_WIDTH_LIMIT` 失败 | 未运行（containment 硬失败即返回） | 未运行 | executable=false，无 plan/path；ROS/显示 NOT_RUN |
| S2 corridor | 0 成功（1 `SOLVER_FAILED` + 1 `SOLVER_TIMEOUT`） | `BUDGET_EXHAUSTED/SOFT_BUDGET_EXHAUSTED` | 未运行 | 未运行 | 无 plan/path；ROS/显示 NOT_RUN |
| S3 corridor | 4/7 audited（其余 2 failed + 1 timeout） | 两轮均几何可行；第 0 轮 containment 通过，第 1 轮在 containment 前预算停止 | 第 0 轮通过；第 1 轮未运行 | 第 0 轮 `MAP_OBB_COLLISION`（vehicle 3）；第 1 轮未运行 | executable=false，无 plan/path；0 误发布；ROS/显示 NOT_RUN |
| S3 legacy（补充） | 9/11 audited | 可行；legacy 无 corridor containment 层 | 两轮均通过 | 第 0 轮 numerical failure（fail-closed），第 1 轮通过 | executable=true，plan/path 存在；未做 ROS 发布/显示（NOT_RUN） |

## 8. 失败、未运行与未实现（如实列出）

**失败/负结果**
- corridor 模式 C3 成功 0/3（S1 containment 宽限、S2 预算、S3 预算）；
  任务未要求成功率提升，未扩大预算。
- S3 C3 最终状态与 C2 不同（COLLISION_FAILED → BUDGET_EXHAUSTED），
  原因见 §6；不是审核放宽，也不是误发布。
- S2 两次 solver 尝试均未得到 audited 解（IPOPT 迭代上限、墙钟超时）。

**NOT_RUN**
- ROS 节点、ROS master、RViz/显示、adapter 发布：全部未运行。
- 真车/硬件：未运行。
- corridor 镜像场景（S1_mirror/S2_mirror/S3_mirror 的 corridor 版）：未运行；
  legacy 六冻结场景已由 pipeline 测试覆盖。
- corridor 模式下的 scipy solver 变体：未运行。
- C2 代码未在 C3 重跑；C2 行为引用既有证据文件。

**NOT_IMPLEMENTED（明确未做）**
- 固定 σ 失败的“一次相邻 portal 重分配”：`corridor.py` 不在 TASK_C3
  允许文件集，未修改；该失败路径保持 fail closed。
- C 退化的“单次 seed 切分”：未实现；已实现合同给出的另一选项
  “换另一条已有路线”（corridor build/guide/allocation/chain audit 失败时
  花费唯一恢复换 route）。
- corridor 硬 containment 失败按合同列出的恢复类别没有对应动作，
  因此直接 fail closed，不消耗恢复槽。

**实现解释点（请 Codex 重点核对）**
- endpoint-only 恢复轮不向 solver 传 route guide，因此进入
  `make_initial_guesses` 的“无路线”分支，使用历史 G2 六初值
  （1.2/1.8/2.6 × 两方向）。有 route 的轮次严格按“每路线 1 个 +
  ≤2 种 endpoint 形状”分配。若合同要求 endpoint-only 恢复也限制为
  ≤2 种形状，需要改这一分支（会同时影响 S3 legacy 正例）。
- `_routes_shortcut_equivalent` 是首版 shortcut 等价筛查，可能合并
  非严格同伦的路线；文档和注释均不声称完备。
- 候选排序/已审候选保护沿用 C2 代码，本阶段未改其实现，只用既有测试
  复核。

## 9. 关键数值与预算

- `total_soft_budget_s=60.0`；单 solve 上限 `min(planner max_seconds=30,
  remaining)`；`max_recoveries=1`；`hard_real_time_guarantee=false`。
- S3 恢复记录：`action=collision_local_block`，`recovery_index=1`，
  `from_route_index=0`，`reason=MAP_OBB_COLLISION`，
  `wall_time_s=35.968`（请求开始后）。
- S3 碰撞位置：`qf=(4.10366700250394, 6.222767787956905e-07,
  3.1625703640427405e-07)`，`station=4.103667002504015`，
  `vehicle_id=3`，`sample_index=124`，`segment_index=2`，
  `time_s=11.48564424435774`。
- S1/C3 每次尝试 wall：7.98 / 7.71 / 7.01 / 7.44 s；
  S2/C3：20.09 / 13.40 s；S3/C3：13.05 / 5.11 / 5.15 / 7.81（第 0 轮），
  12.76 / 5.15 / 5.14（第 1 轮）。
- 0 false publications（所有运行和测试）。

## 10. 复现命令

精确命令、时间戳、退出码与测量墙钟见 `commands.log`；日志见
`test_logs/`。核心三条：

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```

corridor 同输入案例：

```bash
python3 -B evaluation/virtual_a_corridor/C3/20261117T155212Z/run_corridor_case.py \
  S1 evaluation/virtual_a_corridor/C3/20261117T155212Z/corridor_cases/S1_corridor_c3.json
# S2/S3 同理；默认 solver=ipopt，guidance_mode=corridor
```

环境：Python 3.8.10，numpy 1.24.4，scipy 1.10.1，casadi 3.7.2（IPOPT 可用），
Linux 5.15.0-139；见 `environment.txt`。

## 11. 交付物清单

- `SUMMARY.md`（本文件）
- `results.json`：schema、基线、前置、上限、测试、C2/C3 对比、
  legacy 正例、源码 hash、false_publications、NOT_RUN/NOT_IMPLEMENTED、
  测试覆盖映射
- `commands.log`：精确命令、UTC 起止、exit、测量墙钟
- `source_hashes.txt`：任务合同、基线、当前源码/测试、证据文件 hash
- `source.diff`：C3-only diff（5 文件，1306 行）
- `diffstat.txt`：逐文件增删统计
- `start_state.log` / `end_state.log` / `git_state_final_pre_evidence.log`
- `environment.txt`
- `run_corridor_case.py`（证据 harness）
- `corridor_cases/S1_corridor_c3.json`、`S2_corridor_c3.json`、
  `S3_corridor_c3.json` 及对应 `.stdout.log`
- `corridor_cases/S3_legacy_c3.json` 及 `.stdout.log`（补充正例）
- `corridor_cases/C2_vs_C3.json`（同输入对比，含 C2 证据文件 sha256）
- `test_logs/`（三条测试日志 + `run_meta.log`）
- `assemble_results.py` + `assemble_results.stdout.log`
- `manifest.sha256` + `manifest_verification.log`（全部证据文件校验）

## 12. 停止声明

C3 证据已交付，按任务要求在此停止：不自动进入 C4、不提交 Git、
不启动 ROS/显示、不声称 Codex ACCEPTED。等待 Codex 审核与用户指令。