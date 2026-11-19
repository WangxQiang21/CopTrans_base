# TASK 01 Claude 开发交付

> 状态：开发进行中（每步完成后更新）。

## 1. 基线与环境

- BASE SHA：`77805cd46849ebadc374187f9b00fce0f7151f84`
- 分支：`feature/virtual-rigid-formation-a`
- 开发开始前 dirty state：`?? work_flow/`（用户已有的技术方案与执行方案输入，未被本任务修改）
- Python：3.8.10（`/usr/bin/python3`）
- CasADi：3.7.2
- ROS：Noetic setup 存在；`rospy` 在本机 Python 环境可导入（离线证据与 ROS 证据仍分别记录）
- 权威运行地图：由仓库 `maps/map05-p12-parking-row-env2/map.yaml` 经 `tools/scale_map_runtime.py` 按 2.625 重建；与历史 `/tmp/expv3-repro-map.t2Au9l/...` 的 YAML/PGM **逐字节一致**（见 `00_baseline/map_sha256.txt`）。

## 2. Step 0：冻结基线与复现输入

命令与原始输出：

| 产物 | 路径 |
|---|---|
| 默认 dry-run 输出 | `work_flow/TASK01_evidence/00_baseline/default_dry_run.log`（退出 0，0.38 s） |
| 600003/600004 复现 | `work_flow/TASK01_evidence/00_baseline/repro_u_turn.{json,log,time}`（退出 0，57.34 s） |
| 基线实验身份汇总 | `00_baseline/baseline_summary.json` |
| 旧实现航向/S 域 | `00_baseline/baseline_heading.json` |
| 地图 SHA | `00_baseline/map_sha256.txt` |

基线实测（**旧实现，不是新验收**）：

- 600003：`SUCCESS`，wall 18.970 s，attempt 0 `Solve_Succeeded` 41 it 9.033 s；base heading −3.126 rad；S ∈ [10.0187, 40.0560] m。
- 600004：`BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED / GEOMETRY`，wall 37.797 s；attempt 0 `Infeasible_Problem_Detected` 68 it 11.627 s；base heading +3.124 rad；S ∈ [9.5298, 38.5895] m。
- 默认 `run_demo.sh --dry-run` 仍解析为 `backend:=uniform_v02`（TASK 01 待修项）。

## 3. Step 1：Progress、五图层、HUD 和失败冻结

状态：**Claude 自测完成，等待 Codex 独立复核**（不声明 Codex PASS，不声明 TASK 01 验收通过）。
`PUBLISHED` 按用户勘误执行：pipeline 永不发出；仅当 ROS 节点在
`playback_plan_pub.publish(envelope)` 成功返回后，由节点发布 `PUBLISHED` 文档。

### 3.1 实际修改

| 文件 | 修改 |
|---|---|
| `formation_continuous_a/contracts.py` | 新增 `PROGRESS_SCHEMA`、`PROGRESS_STAGES`、`PROGRESS_STAGE_INDEX`、`PROGRESS_STATES`、`PROGRESS_FAILED_STAGE` 规范常量 |
| `formation_continuous_a/pipeline.py` | 可选 ROS-free `progress_callback`；`_ProgressEmitter` 只读事件、单次异常记录后禁用；在 A*/corridor/guide/NLP/三项 audit 的真实开始与完成处发事件（含真实 metrics、`diagnostic_display`、`diagnostic_path`）；不可执行/异常发 `FAILED`；拒绝发 `PUBLISHED` |
| `formation_continuous_a/ros_adapter.py` | `solve()` 与 hooks 仅透传 `progress_callback`；旧无 callback 调用不变 |
| `formation_uniform_planner_node.py` | worker 显式 keyword 传 callback；admission 后发 `REQUEST_ADMITTED`；渐进 debug 文档经身份/阶段/摘要校验后才发布；复用 `_display_from_corridor_round()`；图层键缺省=保留、`null`=清该层；`_finish_virtual_failure()` 发最终 `FAILED` 并冻结已完成层，不调用 `_clear_planning_debug()`；`_finish_virtual_success()` 在信封 publish 之后发 `PUBLISHED`；显示构造失败只记 `PROGRESS_DISPLAY_ERROR` 并清对应层，不改变规划结果 |
| `formation_playback.py`（ROS-free） | corridor 固定红色 fill/edge；A* 橙色虚线 LINE_LIST；guide 绿色；新增 candidate builders（F/三车中心/OBB/失败点，固定 namespace/颜色/DIAGNOSTIC 标签）；`CERTIFIED` 仅允许 PUBLISHED；HUD 增加 stage/step/route/round/elapsed/reason 行；扩展身份匹配到 request/grid/context/model 与 planning 三态 |
| `formation_playback.py`（ROS script） | 新增 `/formation/planning_candidates` publisher 与逐层分发；HUD signature 含 progress 字段；新 admission 清三诊断层；FAILED job/raw request/playback `_clear` 均不误清；无轨迹时仍刷新 progress HUD |
| `formation_demo.rviz` | A* namespace 默认开启；走廊改名 `Convex workspace corridors (F point only)`；新增 `Planning candidates (unverified until PUBLISHED)` MarkerArray（第 9 节审核修改后拆为 Path 话题 + `Candidate OBBs and labels (markers)`） |
| `tests/test_virtual_a_corridor_display.py` | 扩充 corridor/guide/candidate 断言；新增 progress 契约、渐进文档身份负例、失败冻结、逐点 `state_at_s`/刚性/OBB heading、发布器异常隔离、播放节点生命周期测试 |
| `tests/test_formation_hud.py` | 新增 progress HUD 文本测试 |
| `tests/test_virtual_a_ros_adapter.py` | 将旧 `_publish_corridor_debug` 测试迁移为新的 `_corridor_displays` 构造/畸形输入测试；成功信封测试改为验证 `["envelope", "planning"]` 发布顺序及 PUBLISHED 文档字段 |

### 3.2 测试命令与结果

运行环境（本机 ROS Noetic）：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

> 说明：§5.1 字面的 `export PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src`
> 会覆盖 ROS/helper 路径，导致 `visualization_msgs`、`fwis_v3_msgs` 等导入失败；
> 实际采用上面的增量前置。测试模块选择与 §5.1 一致。

| 组 | 模块 | 用例 | 时长 | 退出码 |
|---|---|---:|---:|---:|
| G1 | `test_virtual_a_corridor_display` + `test_formation_hud` | 66 | 21.044 s | 0 |
| G2 | `test_virtual_a_receiver` + `test_virtual_a_playback` | 49 | 59.966 s | 0 |
| G3 | `test_formation_demo_assets` | 21 | 1.811 s | 0 |
| G4 | `test_virtual_a_corridor_optimizer` + `test_virtual_a_optimizer` | 53 | 80.500 s | 0 |
| G5 | `test_virtual_a_pipeline` + `test_virtual_a_corridor_recovery` | 26 | 133.772 s | 0 |
| G6 | `test_virtual_a_geometry` + `test_virtual_a_profile` | 42 | 2.852 s | 0 |
| G7 | `test_virtual_a_integration_bounds` + `test_virtual_a_audit_evaluation_guard` + `test_virtual_a_audit` + `test_virtual_a_trajectory` | 97 | 7.105 s | 0 |
| G8（补充） | `test_virtual_a_ros_adapter` | 60 | 17.336 s | 0 |

七组共 354 例；含补充共 414 例，全部 `OK`。`py_compile` 对全部改动 Python 文件通过。

### 3.3 定向验收证据

- 真实 stage 顺序：首次出现的 `ASTAR_SEARCH → CORRIDOR_BUILD → PORTAL_GUIDE → NLP_SOLVE → GEOMETRY_AUDIT → DYNAMICS_AUDIT → OBB_COLLISION_AUDIT` 索引严格递增；后续路由回跳允许。pipeline 事件中无 `PUBLISHED`。
- 错误 plan/generation/grid/context/request/schema/state/stage/stage_index/snapshot 事件零发布（`ProgressiveDocumentTest`）。
- FAILED 冻结：保留 A*/corridor/candidate，`stage_index` 为最后真实阶段；`_clear("...")`、FAILED job、heartbeat、raw request 均不清诊断层。
- 新 admission（PLANNING 且 plan 变化）清除三层；busy reject 不清。
- candidate 点逐点等于 `path.state_at_s(s)`（≤1e-12 m）；三车中心距刚性 ≤1e-10 m；OBB 首角等于车辆实际 heading（≤1e-12 m）。
- 未认证 candidate 不显示 `CERTIFIED`、不触发 playback；`PUBLISHED` 文档与信封发布顺序为 `envelope → planning`。
- 显示构造失败只清对应层并计 `PROGRESS_DISPLAY_ERROR`，规划状态/审核结果不变。

### 3.4 证据路径

全部日志、退出码、时长与汇总见
`work_flow/TASK01_evidence/01_visualization/`（`summary.md`、`g*.log`、
`g*.exit_code`、`g*.duration`）。`00_baseline/` 未被修改。

## 4. Step 2：默认研究入口和 corridor/hard 透传

状态：**Claude 自测完成，等待 Codex 独立复核**（不声明 Codex PASS，不声明 TASK 01 验收通过）。
soft 仍可显式实验；默认 hard。`--single`、`uniform_v02`、`legacy` 行为未改。

### 4.1 实际修改

| 文件 | 修改 |
|---|---|
| `run_demo.sh` | formation 无显式 backend 时默认 `virtual_rigid_a`；仅对该 backend 扫描 `virtual_a_guidance_mode:=`、`virtual_a_corridor_enforcement:=`、`auto_goal:=`，缺省注入 `corridor`/`hard`/`false`；同名参数出现 2 次即退出 2。显式 `uniform_v02`/`legacy` 完全不注入 virtual 参数，沿用历史 `auto_goal` 行为。`--single` 分支未改。 |
| `run_virtual_a.sh` | 仍是薄 wrapper：只补缺失的 `backend:=virtual_rigid_a`、`virtual_a_guidance_mode:=corridor`、`virtual_a_corridor_enforcement:=hard`、`auto_goal:=false`，不复制 map/GUI 逻辑；显式值原样透传。 |
| `formation_demo.launch` | `backend` 默认 `virtual_rigid_a`；`virtual_a_guidance_mode` 默认 `corridor`；新增 `virtual_a_corridor_enforcement`（默认 `hard`）；两个参数均只在 `backend=='virtual_rigid_a'` 时写入 `/formation_uniform_planner`。 |
| `formation_uniform_planner_node.py` | `main()` 读取 `~virtual_a_guidance_mode`（缺省 `corridor`）与 `~virtual_a_corridor_enforcement`（缺省 `hard`），经 `research_planner_config()` 规范化并 validate，启动日志打印实际 `guidance_mode`/`corridor_enforcement`/`as_dict()`；未知值抛 `INVALID_CONFIG`，`logfatal` 后重新抛出，节点启动失败。 |
| `formation_continuous_a/contracts.py` | 新增研究工厂 `research_planner_config()`（strip+lower 后复用 `validate_planner_config`）；`GeometryResult` 增加 `corridor_halfspace_constraint_count`、`corridor_max_residual_m` 并写入 `as_dict()`；历史 `PlannerConfig` 默认值、`as_dict`、`from_dict` 未改。 |
| `formation_continuous_a/optimizer.py` | `_CorridorConstraint.halfspace_constraint_count = Σ len(limits)`（与 numeric residual、CasADi `g` 使用同一 `groups`）；`_result()` 透传两个新字段；`solve_open_path_guided()` 在 `_build_corridor_constraint()` 之后读取真实计数，成功时报告返回候选的 `corridor_residual_m`，失败时报告全部 attempt 的最大残差。 |
| `formation_continuous_a/pipeline.py` | `_corridor_evidence(geometry)` 只读取 optimizer 的真实字段（无 polygon 推算）；NLP_SOLVE complete 进度、search round record、成功/几何失败结果与 metrics 均带这两个字段。 |
| `tests/test_formation_demo_assets.py` | 默认入口、显式覆盖、重复参数、uniform/legacy 历史 auto_goal、launch 只向 virtual node 传参、node main 读取/validate 源码契约。 |
| `tests/test_virtual_a_corridor_optimizer.py` | 研究工厂只改两个选择且历史默认不变；launch XML 默认值 → 工厂 → `_guidance_mode` → `_build_corridor_constraint()` → CasADi `g` 行数 352 全链断言。 |
| `tests/test_virtual_a_ros_adapter.py` | settings → map request 的 `planner_config` 同一性与 corridor 字段；`main()` 规范化值到达 `hooks.settings.planner_config` 并打印；未知值 fail closed。 |
| `tests/test_virtual_a_corridor_display.py` | NLP_SOLVE 进度中的真实计数/残差等于 result.metrics 与 `search["rounds"][-1]`。 |

### 4.2 验收链证据（dry-run → launch → ROS param → settings → corridor_mode → constraint → CasADi）

1. **dry-run 参数**（`02_research_entry/dry_run/`，均含 stdout/stderr 与退出码）：
   - 默认：`backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false`，四个名字各出现 1 次（`default_map04`，rc 0）。
   - 显式覆盖：`legacy_route/soft/auto_goal:=true` 原样保留（`explicit_overrides`，rc 0）。
   - 重复：guidance、enforcement、auto_goal 各自重复即退出 2，且不打印 launch 行（`duplicate_*`）。
   - `uniform_v02`/`legacy`：输出无任何 `virtual_a_*`，`auto_goal:=true` 保持历史行为（`explicit_uniform`/`explicit_legacy`，rc 0）。
   - `--single`：仍为 `demo.launch` + `planning_backend:=mode_path_v2`，无 virtual 参数（`single_backend`，rc 0）。
   - wrapper：缺省注入 `corridor/hard`；显式 `soft` 保留（`wrapper_default`/`wrapper_explicit_soft`，rc 0）。
2. **launch XML → ROS param**：`roslaunch --nodes fwis_v3_demo formation_demo.launch` 解析出四个节点；`roslaunch --dump-params ... backend:=virtual_rigid_a` 中只有 `/formation_uniform_planner` 出现 `virtual_a_guidance_mode: corridor` 与 `virtual_a_corridor_enforcement: hard`；`backend:=uniform_v02` 的 dump 没有任何 `virtual_a_*` 参数（`roslaunch_nodes.log`、`roslaunch_dump_virtual.log`、`roslaunch_dump_uniform.log`）。
3. **ROS param → settings → corridor_mode**：`NodeMainResearchEntryTest` 断言 `" Corridor "`/`"HARD"` 规范化后进入 `hooks.settings.planner_config`；`RequestFromSnapshotTest.test_research_settings_reach_the_map_request_and_corridor_mode` 断言 map request 持有同一 config，pipeline 的 `corridor_mode` 由该 config 的 `guidance_mode=="corridor"` 决定。
4. **constraint → CasADi**：`test_launch_research_defaults_reach_the_casadi_halfspaces` 直接读取 launch XML 默认值，构造 `_CorridorConstraint` 后断言 `halfspace_constraint_count == 352`、CasADi `nlp_g` 的 corridor 区间行数 == 352、下界全 `-inf`、上界全 `0`。
5. **结果/进度字段来源**：`test_corridor_evidence_reaches_progress_and_result` 断言 NLP_SOLVE complete 的计数为整数且 >0、残差在 `[0, 1e-8]`，并与 `result.metrics` 和 `search["rounds"][-1]` 完全一致；计数只在真实 `_CorridorConstraint` 构造后产生，mock/无 corridor 时为 `None`。

### 4.3 测试命令与结果

运行环境（同 Step 1）：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

| 组 | 模块 | 用例 | 时长 | 退出码 |
|---|---|---:|---:|---:|
| G1 | `test_virtual_a_corridor_display` + `test_formation_hud` | 67 | 19.824 s | 0 |
| G2 | `test_virtual_a_receiver` + `test_virtual_a_playback` | 49 | 57.586 s | 0 |
| G3 | `test_formation_demo_assets` | 28 | 3.955 s | 0 |
| G4 | `test_virtual_a_corridor_optimizer` + `test_virtual_a_optimizer` | 55 | 75.773 s | 0 |
| G5 | `test_virtual_a_pipeline` + `test_virtual_a_corridor_recovery` | 26 | 123.732 s | 0 |
| G6 | `test_virtual_a_geometry` + `test_virtual_a_profile` | 42 | 2.413 s | 0 |
| G7 | `test_virtual_a_integration_bounds` + `test_virtual_a_audit_evaluation_guard` + `test_virtual_a_audit` + `test_virtual_a_trajectory` | 97 | 6.713 s | 0 |
| G8（补充） | `test_virtual_a_ros_adapter` | 63 | 18.297 s | 0 |

七组共 364 例；含补充共 427 例，全部 `OK`。本轮为全部测试编辑完成后的权威重跑；
`py_compile`（全部改动 Python 文件）、`bash -n`（`run_demo.sh`、`run_virtual_a.sh`）、
`git diff --check` 均通过。

### 4.4 证据路径

`work_flow/TASK01_evidence/02_research_entry/`（`commands.txt`、`g*.log`、
`g*.exit_code`、`g*.duration`、`dry_run/`、`summary.md`）。`00_baseline/` 与
`01_visualization/` 未被修改。

## 5. Step 3：候选级 winding

状态：**Claude 自测完成，等待 Codex 独立复核**（不声明 Codex PASS，不声明 TASK 01 验收通过）。
只尝试有限 `{k0-1,k0,k0+1}`；默认首选，唯一 recovery 最多前进一个候选；
碰撞、动力学、数值认证与走廊包含失败不触发 winding 猜测。

### 5.1 实际修改

| 文件 | 修改 |
|---|---|
| `formation_continuous_a/contracts.py` | 新增不可变 `HeadingBranch`（六字段 + `as_dict`）；`GeometryResult.goal_heading_branch_rad`；`AttemptRecord` 增加 `heading_base_rad/winding_k/guide_turn_rad`（仅非 `None` 序列化）。 |
| `formation_continuous_a/optimizer.py` | `_search_packet(request, config, heading_branch=None, guide_length_m=None)` 是 heading/S 域唯一来源；有 branch 用 unwrapped，否则逐字节保留历史 wrap。`solve_open_path_guided()` 每轮只建一次 packet，原样传给初值、route/endpoint guess、numeric 与 CasADi 等式、attempt metadata；新增 `INVALID_HEADING_BRANCH`/`INVALID_GUIDE_LENGTH` fail-closed。`initial_guess_key()` 在有 branch 时追加 `winding_k`；`GeometryResult` 记录 unwrapped 值。 |
| `formation_continuous_a/pipeline.py` | `_guide_segment_tangents()` 删除连续重复/近零段；`_heading_branch_candidates()` 由 `psi0`+有效段 `atan2`+`psig` 累加 `Theta_G`，`base=wrap(psig-psi0)`，`k0=round((Theta_G-base)/2π)`，有限集合按 unwrapped-turn 误差排序（距离按 1e-12 rad 归并，并列严格取较小 `|k|`、再较小 `k`）。每轮按 `(route_index, corridor_mode)` 建立当前 guide 的分支集；`search["winding"]`、round、NLP_SOLVE、attempt 记录 base/k/turn/rank/guide length。`_winding_retry_allowed()` 只允许 `SOLVER_NOT_CONVERGED`/`FIXED_ALLOCATION_LIMIT`，或最终决定 `AUDIT_REJECTED` 的最后一次拒绝为 endpoint 位置/航向错误；winding recovery 前进一个候选，受既有 `MAX_RECOVERIES=1`、request-wide `MAX_INITIALS`、soft budget >1 s、未用 initial 约束。 |
| `tests/test_virtual_a_corridor_recovery.py` | 7 例分支数学/合同测试（CCW/CW 环、重复/近零段、600004 型近并列、退化 guide、物理 goal 不变、不可变）；4 例调度测试（audit 成功但 endpoint 拒绝时只耗一次 winding recovery、solver 非收敛重试恰好一个分支且 initial 数 `[5,1]`/6 个唯一 key、非 endpoint 认证失败不猜、动力学失败不猜）；`_winding_retry_allowed` 对最后 concrete rejection 的定向断言。 |
| `tests/test_virtual_a_optimizer.py` | 一次 packet 到达全部 solver 调用、`GeometryResult.goal_heading_branch_rad` 为 unwrapped、attempt metadata 一致；route initial 身份随分支改变。 |
| `tests/test_virtual_a_corridor_optimizer.py` | `k=1` branch 同时进入 numeric/CasADi endpoint 等式，行值一致（≤1e-12）且为 `-2π`。 |
| `work_flow/TASK01_evidence/03_winding/probe_winding.py` | 离线证据脚本（无 ROS），复用 Step 0 地图/请求/`solve()`，导出 `search["winding"]`、round/attempt 的 branch 字段。 |

### 5.2 真实分支与预算证据

探针地图/请求与 Step 0 完全一致（PGM/YAML/grid sha 相同；600003/600004
request sha 分别为 `bc748779…852b`/`141decf7…e718`，与基线逐位一致）。
冻结配置 corridor/hard/8 intervals/6 initials/30 s：

| 项 | 600003 | 600004 |
|---|---|---|
| 结果 | `SUCCESS`，路径 21.7016012085 m | `BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED / GEOMETRY` |
| guide | 18.625827 m | 45.744425 m |
| `base` / `Theta_G` | −3.126 / −3.126 | +3.124 / −3.1591853071795875 |
| 候选 rank→k | (0→0), (1→−1), (2→+1) | (0→−1), (1→0), (2→−2) |
| 选中 | k=0，rank 0 | k=−1，rank 0；unwrapped −3.159185…（物理 +3.124） |
| attempts / recovery | 1 次 41 iter audited / `[]` | 3 次 solver 失败（2 Infeasible + 1 wall timeout）/ `[]` |

- 600003 的路径长度与 Step 0 基线逐位一致，证明 packet 重构与 branch 透传未改历史成功语义。
- 600004 的 `k=−1` 是公式首选；`k=0` 与 `k=−2` 数学上均距 `Theta_G` 2π，
  1e-12 rad 归并后才按“较小 `|k|`”得到 `[−1,0,−2]`，避免浮点 ulps 把 `k=−2`
  错排在前。
- 600004 在本步**没有** winding recovery：几何 reason 是
  `SOFT_BUDGET_EXHAUSTED`，不在允许集合，且 request-wide 60 s soft budget 已耗尽；
  90 s 补充探针（`winding_600004_90s.json`）仍受 `G4_TOTAL_SOFT_BUDGET_S=60 s`
  总闸限制，4 个 attempt 都停在 `k=−1`、`recoveries=[]`。600004 仍不可执行是
  Step 4 的 guide 相关 `S_upper` 与 Step 5 的 v3 证书待办，不是本步分支选错。

### 5.3 测试命令与结果

环境同前。权威重跑（全部 Step 3 改动完成后的最终代码）：

| 组 | 模块 | 用例 | 时长 | 退出码 |
|---|---|---:|---:|---:|
| G1 | `test_virtual_a_corridor_display` + `test_formation_hud` | 67 | 19.501 s | 0 |
| G2 | `test_virtual_a_receiver` + `test_virtual_a_playback` | 49 | 62.398 s | 0 |
| G3 | `test_formation_demo_assets` | 28 | 3.827 s | 0 |
| G4 | `test_virtual_a_corridor_optimizer` + `test_virtual_a_optimizer` | 58 | 78.841 s | 0 |
| G5 | `test_virtual_a_pipeline` + `test_virtual_a_corridor_recovery` | 37 | 126.904 s | 0 |
| G6 | `test_virtual_a_geometry` + `test_virtual_a_profile` | 42 | 2.440 s | 0 |
| G7 | `test_virtual_a_integration_bounds` + `test_virtual_a_audit_evaluation_guard` + `test_virtual_a_audit` + `test_virtual_a_trajectory` | 97 | 6.731 s | 0 |
| G8（补充） | `test_virtual_a_ros_adapter` | 63 | 15.850 s | 0 |

七组共 378 例；含 G8 共 441 例，全部 `OK`。相对 Step 2 新增 14 例
（G4 +3、G5 +11），未删除或弱化既有用例。`py_compile`、`git diff --check` 通过。

### 5.4 证据路径

`work_flow/TASK01_evidence/03_winding/`（`commands.txt`、`summary.md`、
`g*.log`、`g*.exit_code`、`g*.duration`、`probe_winding.py`、
`winding_600003_600004.json`、`winding_600004_90s.json` 及对应日志/退出码/时长）。
`00_baseline/`、`01_visualization/`、`work_flow/TASK01_技术方案.md`、
`work_flow/TASK01_Codex执行方案.md` 未被修改。

## 6. Step 4：Guide 相关有限 S 域

状态：**Claude 自测完成，等待 Codex 独立复核**（不声明 Codex PASS，不声明 TASK 01 验收通过）。
`1.15` 为技术方案批准的首轮系数，未调参；未实现受控二次扩域；每次上界有限，
solver/initial/soft wall/`MAX_INITIALS` 不变。

### 6.1 实际修改

| 文件 | 修改 |
|---|---|
| `formation_continuous_a/contracts.py` | `AttemptRecord` 增加 `distance_m/guide_length_m/s_lower_m/s_upper_m/s_solution_m/s_upper_active`，仅非 `None` 时序列化为 `distance_m/guide_length_m/S_lower_m/S_upper_m/S_solution_m/upper_active`。 |
| `formation_continuous_a/optimizer.py` | `_search_packet` 在同一 packet 计算 `kappa_ref_max=min(config.kappa_limit,1/r_min_ackermann)`、`S_lower=max(intervals*0.05,d,abs(unwrapped_heading)/kappa_ref_max)`、有 guide 的 `S_upper=max(S_lower+2,12,3d+10,1.15*guide_length)`（无 guide 去掉末项）；删除旧 `min(60,...)` 帽，新增非有限域/非法 guide 的 fail-closed。packet 记录 `distance/guide_length_m/S_lower/S_upper/S_upper_active_tol_m/S_solution/upper_active`；`_guess_for`/`_route_guess` metadata 记录域字段；`_attempt_record` 从同一 metadata 和成功向量 `z[0]` 记录六个 S 字段（超时 iterate 不算解）；`upper_active=abs(S_solution-S_upper)<=max(1e-6,100*配置求解容差)`（scipy 用 `scipy_tolerance`，auto 取较大），非字符串状态；成功返回前用选定 audited candidate 关闭 packet。 |
| `formation_continuous_a/pipeline.py` | 新增 `_s_domain_evidence(geometry)`（成功时匹配 solved S 等于返回路径长度的 audited attempt，否则最后一次 attempt）；NLP_SOLVE complete 进度与每个 `search["rounds"]` 记录六个 S 字段。 |
| `tests/test_virtual_a_optimizer.py` | `SDomainPacketTests` 6 例：公式、`1.15` 项、50/100 m 无旧帽、attempt 的 S_solution/容差判定、packet↔attempt 一致、病态域 fail closed。 |
| `tests/test_virtual_a_pipeline.py` | 1 例：attempt 与 search round 六字段一致；每轮上界公式与 `upper_active` 容差；winding 第二轮 unwrapped heading 改变 `S_lower`。 |
| `work_flow/TASK01_evidence/04_s_domain/probe_s_domain.py` | 无 ROS 探针：600003/600004 + 50/100 m 合成长直线；独立重算期望域并逐项比对。 |

### 6.2 真实证据

探针地图/请求与 Step 0 一致（request sha 未变）。配置 corridor/8 intervals/6 initials/30 s：

| 项 | 600003 | 600004 |
|---|---|---|
| 结果 | `SUCCESS`，路径 21.7016012085 m | `BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED / GEOMETRY` |
| guide | 18.625827 m | 45.744425 m |
| `S_lower` / `S_upper` | 10.018651 / 40.055953（旧值同） | 9.529830 / **52.606089**（旧 38.5895） |
| 解出 `S` | 21.701601208480245，`upper_active=False` | 48.436299 / 48.444182 / 48.444191，均 `upper_active=False` |
| attempt 结果 | 1 × audited success 41 it | 3 × `Solve_Succeeded`，均 `CANDIDATE_CONSTRUCTION_FAILURE` |

- 600003 的 `S_lower/S_upper`、路径长度与 Step 0 逐位一致，历史成功语义未变。
- 600004 新上界 52.606 m 覆盖技术方案预期的 ~48.44 m；solver 已能给出
  `S≈48.44 m` 向量且未贴住上界。失败原因是 v2 数值证书
  `BOUND_UNCONFIRMED: Simpson refinement would exceed 200000 subintervals`
  （107008 panels），留给 Step 5 的 v3 证书；随后预算到期返回
  `SOFT_BUDGET_EXHAUSTED`。本步未做二次扩域，也未把 certificate 失败误判为
  S 不足。

### 6.3 合成 50 m/100 m 长直线（无旧帽）

| 项 | 50 m | 100 m |
|---|---|---|
| `S_lower` / `S_upper` | 50.0 / **160.0** | 100.0 / **310.0** |
| 旧 60 m 帽会给出 | 60.0 | 60.0 |
| initial `S` | 60,60,90,90,130,130 | 120,120,180,180,260,260 |
| solver `S` | ≈50.0（6/6） | ≈100.0（4 个）；310.0000000097（2 个，`upper_active=True`） |
| 结束 | `NO_ACCEPTED_CANDIDATE`，v2 `BOUND_UNCONFIRMED` | 同左（100776 panels） |

上界与 initial 都明显超过旧 60 m 帽，且每次上界有限；两个构造都在 v2 证书处
fail closed（无 plan）。100 m 的 9.7e-9 m 上界偏差按 1e-6 m 容差判为
`upper_active=True`，验证了容差而非状态字符串的判定。

### 6.4 测试命令与结果

环境同前。最终代码的权威重跑：

| 组 | 模块 | 用例 | 时长 | 退出码 |
|---|---|---:|---:|---:|
| G1 | `test_virtual_a_corridor_display` + `test_formation_hud` | 67 | 19.066 s | 0 |
| G2 | `test_virtual_a_receiver` + `test_virtual_a_playback` | 49 | 55.072 s | 0 |
| G3 | `test_formation_demo_assets` | 28 | 3.704 s | 0 |
| G4 | `test_virtual_a_corridor_optimizer` + `test_virtual_a_optimizer` | 64 | 74.109 s | 0 |
| G5 | `test_virtual_a_pipeline` + `test_virtual_a_corridor_recovery` | 38 | 123.424 s | 0 |
| G6 | `test_virtual_a_geometry` + `test_virtual_a_profile` | 42 | 2.311 s | 0 |
| G7 | `test_virtual_a_integration_bounds` + `test_virtual_a_audit_evaluation_guard` + `test_virtual_a_audit` + `test_virtual_a_trajectory` | 97 | 6.391 s | 0 |
| G8（补充） | `test_virtual_a_ros_adapter` | 63 | 15.212 s | 0 |

七组共 385 例；含 G8 共 448 例，全部 `OK`。相对 Step 3 新增 7 例
（G4 +6、G5 +1）。`py_compile`、`git diff --check` 通过。

### 6.5 证据路径

`work_flow/TASK01_evidence/04_s_domain/`（`commands.txt`、`summary.md`、
`s_domain_600003_600004.json`、`s_domain.log`、`s_domain.exit_code`、
`s_domain.time`、`probe_s_domain.py`、`g*.log`、`g*.exit_code`、`g*.duration`）。
`00_baseline/`、`01_visualization/`、`02_research_entry/`、`03_winding/`、
`work_flow/TASK01_技术方案.md`、`work_flow/TASK01_Codex执行方案.md` 未被修改。

## 7. Step 5：生产 v3 数值证书（保留 v2 重放）

### 7.1 实际修改

- `formation_continuous_a/bounds.py`：`NUMERICS_V2` / `NUMERICS_V3` /
  `SUPPORTED_NUMERICS_VERSIONS` / `DEFAULT_NUMERICS_VERSION=V3`；
  `POSITION_BUDGET_M=1e-7`、角预算、roundoff guard、`MAX_PANELS` 未改。
- `formation_continuous_a/integration_bounds.py`：新增
  `production_evaluation_guard()`（逐项对应生产 cached-grid：Horner
  `psi_at`、缓存 sin/cos、Simpson weighted sum、piece position prefix、每
  piece 一个 local panel；每段 `12*n_j+64+16*Psi`，加
  `16*(segments+1)*ε*L*Psi` 与 `2*(segments+1)*ε*(|start|+L)`，全部 outward）。
  `panel_m4_bounds()` / `segment_budget()` / audit 的 `evaluation_guard()`
  未改。
- `formation_continuous_a/geometry.py`：版本集合校验；`from_knots` /
  `VirtualAPath` / `VirtualAPath.hold` 的显式/默认版本规则；`_plan_integration_v2()`
  保留原字节语义（网格构建提取为 `_build_grids()`，算术不变）；新增
  `_plan_integration_v3()`：逐完整 panel M4 + 一个 local panel、
  production evaluation guard、`E_position=sqrt2*(E_quad+E_eval+E_angle)`、
  严格下降且 quadrature 为主因的细分；不可消除项或不降立即
  `BOUND_UNCONFIRMED`。
- `formation_continuous_a/trajectory.py`：model/profile/budget 版本声明必须
  受支持且一致；从文档声明版本重建（旧 v2 用 v2），不支持版本 fail-closed。
- `formation_continuous_a/audit.py`：`_audit_bind_plan` 新增路径/profile/预算
  版本一致性检查；raw-knot 独立重建与 5e-8 阈值未改。
- tests：`test_virtual_a_profile.py` 保留原 v2 用例并 pin `NUMERICS_V2`，
  新增 `V3NumericsTests`；`test_virtual_a_trajectory.py` 新增
  `VersionedNumericsModelTests`（历史 v2 fixture 字节重放 + v3 声明 + 跨版本
  篡改）；`test_virtual_a_audit.py` 新增 `VersionedAuditBindingTests`。
  新 fixture `tests/fixtures/virtual_a/v2_model_600000.json` 是历史
  `model_json` 原文（9283 chars，sha `df146b9b…af8`）。
- `tools/virtual_a_benchmark.py` 未改（其 `_integration_budget_audit` 仍是
  v2 公式；当前测试只用 `_differential_evidence`/`quadrature_position`）。

### 7.2 v3 证书公式（单坐标米）

```text
E_quad_coord  = Σ_seg [ Σ_{完整 Simpson panel} H^5 * M4_panel / 2880
                        + 一个任意前缀 local panel（该段最大 M4） ]
E_eval_coord  = production_evaluation_guard（float64 Horner psi/cached sin-cos/
                weighted Simpson sum/position prefix/每 piece 一个 local panel）
E_angle_coord = S * E_psi
E_position    = sqrt(2) * (E_quad_coord + E_eval_coord + E_angle_coord)
```

`M4_panel` 由 `panel_m4_bounds(c0,c1,c2,c3,h,n)` 对每完整 panel 求
outward 上界；无 `coefficients` 的诊断 analytic profile 用区间包络
（`analytic_interval_envelope_per_panel`）。细分只对最大 quadrature
contribution 段加倍，且需新 total 严格下降、quadrature 仍 ≥
max(eval, angle)；否则立即 `BOUND_UNCONFIRMED`。任意 `state_at_s()` 查询
（complete panels + 最多一个 local panel）由同一证书覆盖。

### 7.3 真实证据

- 历史 v2 model（G5 envelope 600000）重建字节一致：
  `df146b9bb4710132adeacd4d718d5a840c0f0e3bce353d06331a84142c2eaaf8`，
  v2 bound `3.3986e-8`，8×66 subintervals；同一 knots 用 v3 重建
  `CONFIRMED`，bound `4.0433e-9`，528 subintervals。
- 600003：`SUCCESS`，21.7016012085 m，v3 bound 5.1247e-9，1088
  subintervals，三项 audit True；model sha
  `e830730a…75ce`（v3 三处声明一致）。
- 600004：`SUCCESS`，48.4362986116 m，v3 bound 1.1170e-8，2432
  subintervals，三项 audit True；model sha `924beced…1540`。同一 knots 的
  v2 重建立即 `BOUND_UNCONFIRMED`（`total_panels=107008`），说明 Step 4
  的阻塞是 v2 证书边界而非几何/动力学。
- 50 m / 100 m 直线：solver-only `GEOMETRY_FEASIBLE`，v3 bound
  `1.13769e-8` / `2.28712e-8`，2512 / 5008 subintervals；v2 对照分别在
  101422 / 100160 处 `BOUND_UNCONFIRMED`。100 m 的第 5/6 个候选
  （S=310 m）被 `SOLVER_SUCCESS_AUDIT_REJECTED`（`BOUND_UNCONFIRMED`），
  未进入播放。
- 篡改拒绝：v3 model 改 numerics/profile 为 v2 → `MODEL_NUMERICS_MISMATCH`；
  改未知版本 → `UNKNOWN_NUMERICS_VERSION`；内存模型篡改经独立 audit →
  `UNKNOWN_NUMERICS_VERSION` / `UNSUPPORTED_PROFILE`（scope DYNAMICS）。
- 旧 v2 预期 fixture 未被修改；新 v3 model SHA、planning context 和证书是
  自然变化，已记录（600003 context `7449ee16…b8cb`，600004
  `f5c759a8…b70e`）。

### 7.4 测试命令与结果

`run_step5_tests.sh` 每组独立进程，原始 stdout/stderr、退出码、时长在
`05_numerics/tests/`：

| 组 | 命令 | 退出码 | 时长 | 结果 |
|---|---|---:|---:|---|
| G1 | `python3 -m unittest tests.test_virtual_a_corridor_display tests.test_formation_hud` | 0 | 19.70 s | 67 tests OK |
| G2 | `python3 -m unittest tests.test_virtual_a_receiver tests.test_virtual_a_playback` | 0 | 57.05 s | 49 tests OK |
| G3 | `python3 -m unittest tests.test_formation_demo_assets` | 0 | 3.82 s | 28 tests OK |
| G4 | `python3 -m unittest tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer` | 0 | 77.12 s | 64 tests OK |
| G5 | `python3 -m unittest tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery` | 0 | 128.00 s | 38 tests OK |
| G6 | `python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_profile` | 0 | 2.91 s | 49 tests OK |
| G7 | `python3 -m unittest tests.test_virtual_a_integration_bounds tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit tests.test_virtual_a_trajectory` | 0 | 7.76 s | 103 tests OK |

另有：

- `python3 work_flow/TASK01_evidence/05_numerics/verify_numerics_evidence.py`
  → exit 0，独立重算 5 份 v3 证书的 `sqrt2` 合成、每段 quad/eval/angle、
  panel 计数/最大 M4、v2 字节重放和 tamper 原因。
- `python3 -m py_compile` 于 `bounds.py`、`geometry.py`、
  `integration_bounds.py`、`trajectory.py`、`audit.py` 和三个测试文件 →
  exit 0。`git --no-pager diff --check` → exit 0。

### 7.5 证据路径

`work_flow/TASK01_evidence/05_numerics/`（`probe_numerics.py`、
`numerics_probe.json`、`numerics_probe.stdout`、`numerics_probe.stderr`、
`verify_numerics_evidence.py`、`verify_numerics_evidence.stdout`、
`run_step5_tests.sh`、`summary.md`、`tests/`）。
`00_baseline/`、`01_visualization/`、`02_defaults/`、`03_winding/`、
`04_s_domain/`、`evaluation/`、`TASK01_技术方案.md`、
`TASK01_Codex执行方案.md` 未被修改。

## 8. Step 6：完整回归与 ROS/RViz 原子发布验证

### 8.1 实际修改（仅一处生产代码）

- `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`：修复
  Step 1 引入的 transport preview 回归。虚拟后端 `on_request` 提前返回前
  恢复 preview 语义：当 `_job_state` 不是 `PLANNING/PLAYING`，或请求的
  `plan_id` 与已准入 plan 相同时置 `_previewing = False`；对另一个已准入
  plan 的 busy 请求保持 preview。`on_playback_plan` 在 preview 丢弃信封时
  增加 5 s 节流 warning。未触碰 trajectory/hold/`_active_plan`/audit/
  阈值/走廊或 optimizer 数学。
- `tests/test_virtual_a_receiver.py::VirtualTransportPreviewTest`：新增 3 个
  用例（IDLE 请求结束 preview、匹配已准入请求结束 preview、busy 请求保持
  preview）。
- 其余为证据脚本与记录，未改生产语义。

### 8.2 构建、离线全回归与入口

顺序按 §5.3：先离线全回归，再 `catkin_make`，再六个 dry-run，最后 ROS。
证据在 `work_flow/TASK01_evidence/06_regression/`（修复前）与
`06_regression/postfix/`（修复后复跑）。

| 项 | 修复前 | 修复后（postfix） |
|---|---|---|
| 全量 `unittest discover` | 1047 tests，549.586 s，exit 1，1046 通过 | 1050 tests（+3 preview 用例），550.550 s，exit 1，1049 通过 |
| 唯一错误 | `test_continuous_retiming...KeyError: 'dynamic_peaks'`（既有） | 同一错误，位置不变 |
| `catkin_make` | exit 0（3.57 s 预构建；正式 0.37 s） | exit 0，`elapsed=0.35 s`，87 行日志无 error |
| 六个 dry-run | 全部 exit 0 | 全部 exit 0（0.20–0.35 s） |

两次回归测试都改写了 `evaluation/formation_refactor_v02/` 下 26 个文件的
计时/路径/内存字段；清单与 diffstat 保存在 `regenerated/` 与
`postfix/regenerated/`（665 insertions / 665 deletions），核对后已全部
恢复，`git status --porcelain -- evaluation` 为空。没有修改
`00_baseline/`…`05_numerics/` 或历史证据。

### 8.3 ROS/RViz 实机结果

全部会话使用任务私有 master `http://127.0.0.1:11633`，结束后只剩
`/rosout`；另一进程的 11411 master / roslaunch pid 4042262 未被触碰。
详细逐项证据见 `07_ros_rviz/summary.md`。

**字面 map05（scale 2.625，corridor/hard）**

- 无效起点 `(0,0,0)`：plan 600000 `NO_ROUTE_GUIDE`，collision audit
  `MAP_OBB_COLLISION`（static_start, vehicle 3）；失败占位信封
  `model_json=0`、samples 0，可执行 0；证据保留在 `map05_full_3/`。
- 有效起点 reset `(2,2,0)`、goal `(8,2,0)`：plan 600000，712 samples，
  `T_end=14.203125000000316 s`，70 checks 全过；
  request `e69d068b…`、context `06aed480…`、model `e6b6e55d…`、
  grid `64eef089…`；certification `virtual_a_geometry_dynamics_collision/v1`，
  envelope numerics `g1.v3`（position bound `1.3589865e-9`）。
- 进度 12 条：`REQUEST_ADMITTED → ASTAR_SEARCH → CORRIDOR_BUILD →
  NLP_SOLVE×2 → GEOMETRY_AUDIT×2 → DYNAMICS_AUDIT×2 →
  OBB_COLLISION_AUDIT×2 → PUBLISHED`；每条事件带
  plan/request/context/grid/model/stage_index 键。
- 三项 audit 全过；collision 3740 checks；S `6.0/6.0/28.0`；
  `corridor_halfspace_constraint_count=352`、`corridor_max_residual_m=0.0`；
  attempt residual `1.78e-15`；candidate `CERTIFIED`；winding 选中 k=0。
- `/formation/playback_plan` 原始 2 条（启动占位 plan 0 不可执行、
  plan 600000 可执行 1 条：18921 B / 712）；可执行信封到达时间
  `1795029032.541922` 早于 `PUBLISHED` 事件 `1795029032.559619`，符合
  §2.2 的 errata。播放 `PLAYING → HOLD_END`，`duration=14.203125001 s`，
  19 个 marker、5 个 namespace，terminal 误差 0。
- 计数口径：失败 plan 仍由 `_publish_no_envelope()` 发布显式无效占位
  （空 model/trajectory/certification，用于覆盖 latched 旧信封），所以
  raw topic 为 1 条、**可执行信封 0 条**；成功 plan 可执行恰好 1 条。
  若按 Step 6 原文要求失败 plan 的 raw 计数也为 0，则该要求与当前的
  安全占位行为不同，需 Codex 判定；此处如实并列两个口径。

**S1 场景（冻结 manifest，corridor/hard）**

- `s1_easy_corridor_fixed/`：显式小目标 `(5,0,0)`，83 checks、0 failed；
  plan 600000 可执行 1（18286 B / 593），HOLD_END；继续请求被拒绝
  `UNSUPPORTED_INITIAL_STATE`，无可执行信封。
- `s1_failure_corridor_fixed/`：straight 失败 plan 600001 `NO_ROUTE_GUIDE`，
  原始占位 1 条、可执行 0、无 `PUBLISHED`，播放 `CLEARED`；恢复 plan
  600002 可执行 1（19134 B / 593），HOLD_END。155 checks、0 failed。
  场景窗口内可执行合计 2；后续 11 条 600002 可执行记录是 tamper 诊断
  重放，不是 planner 再次发布。
- 修复前 `s1_easy_corridor/` 的 `PLAYBACK_START_TIMEOUT` 与修复后成功
  并列保留；根因与最小修复见
  `07_ros_rviz/diagnostics/README_playback_preview_regression.md`。
- S1 原目标（FREE）在 corridor/hard 默认入口的
  `CORRIDOR_AUDIT_WIDTH_LIMIT` 仍是既有认证边界：用 `git archive` 的
  HEAD 复现树逐位一致，不是 Step 1–5 回归；
  `s1_main_free/` 与 `diagnostics/README.md` 保留该失败，未修改阈值。

**后端 smoke 与单车**

- `--backend uniform_v02`：map16/3.75，自动目标 `(2.725,-11.688,1.571)`；
  plan 600000 `TIME_BUDGET_EXHAUSTED`（wall 30.9137 s ≥ 30 s），
  candidate 1665，`A_HEADING_REQUIRES_S_ALIGNMENT=1152`；占位信封
  不可执行（0/0）。
- `--backend legacy`：同一目标，`FORMATION_BUDGET_EXHAUSTED`
  （planner geometry search failed，expanded 50000 / generated 551756 /
  goal_candidates 0）；`/formation/timed_trajectory` 为空；没有
  `/formation/playback_plan` 等话题。该会话 playback/admission backend
  均为 `legacy`；`/formation_uniform_planner/backend=uniform_v02` 是上一
  会话遗留参数，不作为本次 backend 证据。
- `--single --map map05`：`mode_path_v2`，自动目标 `(6.324,4.576,1.571)`，
  plan 1，`nodes=38`、`duration=15.080 s`，`/plan/timed_trajectory`
  62,589 B；播放结束 HUD `center v=0.00 m/s`、位置 `(6.3255,4.5736)`。
  formation 栈未启动。

**时间轴 tamper 诊断**

7 个负例中 6 个拒绝并 `CLEARED`；中点 +1 ns 因落在验证器 1e-9 容差内
未被拒绝（局部差约 7.9e-10 m），已在
`07_ros_rviz/diagnostics/README_tamper_time_boundary.md` 说明边界；独立
1 ms 扰动被确定性拒绝（差 7.92e-4 > 1e-9），随后原始信封重放至
`HOLD_END`。未修改生产容差或测试 fixture，该 1 ns 用例不计为门禁失败。

### 8.4 证据与边界

- 证据路径：`work_flow/TASK01_evidence/06_regression/postfix/`（回归、
  构建、六个 dry-run、evaluation 再生清单）；
  `work_flow/TASK01_evidence/07_ros_rviz/`（`summary.md`、各 session 的
  `launch.log`/`publications.jsonl`/`report.json`/话题采样/截图、
  `diagnostics/`）。
- 截图规范名 `plan_<id>_<stage>_<result>.png` 已用于成功 plan；map05 的
  规范名是 `plan1_*` 截图的同内容副本（sha256 相同），原图保留。失败
  plan 无可播放轨迹时播放截图标 `UNAVAILABLE`，仅保留 HUD/图层截图。
- 未修改 `00_baseline/`…`05_numerics/`、`evaluation/`、技术方案或
  Codex 执行方案；未覆盖 G5 S1/G5 历史结果；未改 audit/阈值/三车模型/
  A*/corridor 算法。
- 全部 session 已停止（`stop.txt` 记录 pid）；会话停止后私有 master
  11633 只剩 `/rosout`，任务私有 roscore 随后也已停止并关闭端口。
  未 commit/push；未进入 TASK 02；未宣称 Codex PASS 或 TASK 01 验收。

## 9. 人工审核后的显示与重置修改（2026-11-19）

本节记录 Step 6 之后人工审核意见第 1、3 项的实现；第 2 项只写分析文档，
没有为此修改生产代码。人工审核明确要求 `Planning candidates`、
`A* raw route` 和 `Portal guide` 也必须是 `nav_msgs/Path`，因此第 1 项
覆盖全部路径类图层，而不只是最终认证路径。详细说明分别见
`work_flow/TASK01_evidence/08_manual_review/README_display_changes.md`
与 `README_corridor_containment_analysis.md`。

### 9.1 全部路径类图层改为可编辑 `nav_msgs/Path`

- ROS-free builder：`build_markers()` 不再生成最终路径的 LINE_STRIP；
  每车 marker 由 5 个降为 4 个（OBB、朝向箭头、已行驶历史、文本标签），
  总计由 20 降为 17。新增 `vehicle_path_samples()`，直接返回已验证信封
  的逐样本中心 `(x, y, theta)`，不插值。
- 新增 `build_guide_display()`：A* 原始路线和 Portal guide 逐点输出到
  `paths[("astar_raw", route_id)]` / `paths[("portal_guide", route_id)]`；
  marker 层只保留参考折线文本标签，不再生成 LINE_LIST/LINE_STRIP。
- 新增 `build_candidate_display()`：F 参考路径和每车中心路径输出到
  `paths["f_candidate"]` / `paths[("candidate_vehicle", vehicle_id)]`；
  marker 层保留标签和 OBB 闭合轮廓。OBB 不是路径，标签不是几何，故
  两者仍为 Marker。
- ROS 节点新增动态 latched Path 话题：
  `/formation/path/f_candidate`、
  `/formation/path/candidate_vehicle_{id}`、
  `/formation/path/astar_raw_{route_id}`、
  `/formation/path/portal_guide_{route_id}`；最终路径仍为
  `/formation/path/vehicle_{id}`。每次新 payload 先清空该层全部旧 route
  话题再发布，route 数减少时不会残留旧几何。诊断 Path 使用 identity
  orientation，最终路径仍保留逐样本 `theta`。
- `formation_demo.rviz` 共 13 个 `rviz/Path` 显示：3 个最终路径、F
  candidate、3 个 candidate centre、3 个 A* raw route、3 个 Portal guide。
  用户可在 Displays 面板逐话题修改 `Color` 和 `Line Width`。`/formation/guides`
  与 `/formation/planning_candidates` 更名为 `Guide labels (markers)` 和
  `Candidate OBBs and labels (markers)`。
- 取舍：Path 每话题只有一个显示级颜色/线宽，A* 不再保留橙色虚线样式，
  candidate 路径不再按 `A/O/S` 模式分段着色；身份仍由同一话题上的文本
  标签和 OBB 表达。若需要虚线或按模式分段，需要额外显示配置，不在本次
  最小修改范围内。

### 9.2 2D Pose Estimate 清除上一次实验的图层

`on_preview` 现在在设置新 hold 后调用 `_clear_diagnostics()`，立即清空
corridor、guide/candidate marker、所有诊断 Path 和最终路径 Path 话题，
再发布新的静态 hold；同时清空 `trajectory`、`_active_plan`，hold 原因置
`START`。`on_request` 的虚拟分支保持原设计：原始请求不是准入，不清除
当前 replay/hold/diagnostic 图层。

### 9.3 验证

| 项 | 结果 |
|---|---|
| 定向测试（playback/receiver/corridor display/assets） | `155 tests`，`81.384 s`，`OK`（`08_manual_review/logs/focused_after_review.log`） |
| `catkin_make -C catkin_ws -j4` | `exit 0`，`[ 97%] Built target formation_planner_node`、`[100%] Built target fwis_v3_planner_node`（`logs/catkin_make_after_review.log`） |
| 全量离线回归 | `1054 tests`，`557.418 s`，`FAILED (errors=1)`；唯一错误是既有 `test_continuous_retiming` 的 `KeyError: 'dynamic_peaks'`，该测试与 `continuous_retiming.py` 相对 `HEAD` 均无改动，且在 `git archive HEAD` 导出树上单独运行同样失败（`logs/full_regression_after_review.log`、`logs/head_regression_error_reproduction.log`） |
| ROS smoke（RViz 目视由人工审核执行，本次未启动 GUI） | `39 checks`，`38 ok`，`1 fail`：`marker_status_feasible` 缺少 `rigidity=`/`obstacle=`。原因是 legacy optimizer quality report 不含 `backend`/`model_sha256`，playback 的 identity 门控因此不合并 pipeline；用 `git archive HEAD` 单独构建并运行同一 smoke 得到逐字相同的失败，故非本次修改引入。未放宽断言。证据：`08_manual_review/logs/ros_smoke_after_review_*` 与 `ros_smoke_head_*` |

### 9.4 边界

- 未改 optimizer 数学、三车模型、A*/corridor 算法、audit 或阈值。
- 未修改 `00_baseline/`…`05_numerics/`、`evaluation/`、技术方案或
  Codex 执行方案；Step 6 的 07_ros_rviz 证据保持修改前状态（19 个
  marker / 5 个 namespace），未回写。
- 未 commit/push；未进入 TASK 02；未宣称 Codex PASS 或 TASK 01 验收。