# Step 6 证据：ROS 实机 / RViz 原子发布与显示验证

日期：2026-11-19。本目录是 `work_flow/TASK01_Codex执行方案.md` Step 6
的 ROS/RViz 部分。未 commit/push；本文件不宣称 Codex PASS 或 TASK 01
验收，也不进入 TASK 02。所有数值均来自本目录保存的日志、JSONL、话题
采样、`report.json` 与截图；不存在或未保存的字段写 `UNAVAILABLE`。

## 1. 会话、环境与脚本

- ROS master：任务私有的 `http://127.0.0.1:11633`（本任务全部会话使用；
  另一进程的 11411 master 与 roslaunch pid 4042262 未被触碰）。全部会话
  结束后 `rosnode list` 只剩 `/rosout`。
- 环境：`source /opt/ros/noetic/setup.bash`、`source catkin_ws/devel/setup.bash`；
  `DISPLAY=:1`。ROS 节点日志写入各 run 目录的 `nodes_raw/`。
- 启动包装：`start_launch.sh <run_dir> <run_demo 参数...>`（直接 `exec
  ./run_demo.sh`）；S1 场景另用 `start_session.sh <run_dir> <map_dir>
  <scale> <policy>`。停止脚本 `stop_session.sh <run_dir>` 只终止本目录
  `session.pid` 对应的 roslaunch，并把 pid 写入 `stop.txt`。
- 采样脚本：`capture_session.sh <run_dir> [tag]` 保存 backend/guidance/
  enforcement/goal_policy、节点表、话题表、`rostopic echo -n1` 采样与
  RViz 截图（`smoke.screenshot`，返回 `ok: True`）。
- 发布审计：`watch_publication.py <jsonl>` 订阅 `/formation/playback_plan`、
  `/formation/planning_debug`、`/formation/optimizer_status`，逐条记录
  原始发布（含 `model_json_length`/`sample_count`/`executable`）。
- 截图规范命名：`plan_<id>_<stage>_<result>.png`；`map05_full_4/images/`
  中 `plan_600000_{start,mid,end}_passed.png` 是同一会话 `plan1_*` 截图的
  同内容别名副本（`sha256` 相同），原始截图保留。

会话一览：

| 目录 | 用途 | 结果 |
|---|---|---|
| `s1_main_free/` | S1 原目标（FREE）在 corridor/hard 默认入口 | 失败：`CORRIDOR_AUDIT_WIDTH_LIMIT`，见 `diagnostics/README.md`；可执行信封 0 |
| `s1_easy_corridor/` | 显式小目标，修复预览回归前 | planner 已发布 1 个可执行信封，但播放未确认（`PLAYBACK_START_TIMEOUT`）；根因诊断保留 |
| `s1_easy_corridor_fixed/` | 同参数、修复后 | 成功，83 checks，可执行信封 1 |
| `s1_failure_corridor_fixed/` | S1 straight 失败 + 恢复 + tamper 诊断 | 155 checks；失败 plan 可执行 0，恢复 plan 可执行 1 |
| `map05_full_3/` | 字面 map05（2.625）无效起点 | 失败 plan 600000 无路由；可执行信封 0 |
| `map05_full_4/` | 字面 map05 有效起点 | 成功，70 checks，可执行信封 1 |
| `backend_uniform_v02/` | `--backend uniform_v02` smoke | plan 600000 `TIME_BUDGET_EXHAUSTED`；占位信封不可执行 |
| `backend_legacy/` | `--backend legacy` smoke | `FORMATION_BUDGET_EXHAUSTED`；无 playback topic |
| `single_smoke/` | `--single --map map05` smoke | `mode_path_v2` 成功，plan 1，15.080 s |
| `map05_full/`、`map05_full_2/` | 早期过程会话（被停止/中止） | 不作为 Step 6 结论，保留原始日志 |

## 2. 字面 map05 全流程（失败与成功）

启动参数（`launch.log` 原文）：
`map_scale=2.625`，`backend:=virtual_rigid_a auto_goal:=false rviz:=true
virtual_a_goal_policy:=free map_dir:=<运行时 map05> virtual_a_guidance_mode:=corridor
virtual_a_corridor_enforcement:=hard scene:=manual_map`。运行期栅格
489×326，res `0.064871624`，signature
`5a9ab3f50546275ad9b47ed0524e76431e2096c4c4f753dcf37243e4b293451f`，
grid SHA `64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321`。

### 2.1 无效起点（`map05_full_3/`，保留失败证据）

reset `(0,0,0)`、goal `(5,0,0)`：plan 600000 `NO_ROUTE_GUIDE`，
A* 无 guide；collision audit `MAP_OBB_COLLISION`（`static_start`,
vehicle 3，qf `[0,0,0]`），detail `static start pose failed the map/vehicle
check`。进度序列 15 条，最后 `FAILED`/`FAILED`，无 `PUBLISHED`。
`/formation/playback_plan` 原始 2 条：启动占位 plan 0（0/0）与失败占位
plan 600000（`model_json_length=0`, `sample_count=0`），**可执行 0**。
该会话随后在同进程内尝试 reset `(2,2,0)` 时收到上一请求的 latch，驱动
报 `plan1reset_no_request: 1` 并中止（`report.json`），故未在该会话取得
成功证据；成功证据用全新会话 `map05_full_4/`。无效起点截图仅
`images/launch.png`（无 plan 可播放，播放截图 `UNAVAILABLE`）。

计数口径说明：失败 plan 会由 planner 的 `_publish_no_envelope()` 发布
一条**显式无效占位**（`model_json`/trajectory/certification 全空，用于
覆盖 latched 的旧可执行信封，避免后启动的播放节点接受旧 plan）。因此
`/formation/playback_plan` 的**原始**记录数为 1，**可执行信封**计数为 0。
本文件的“失败不发布”一律指可执行信封计数；若评审按 Step 6 原文把
raw 计数也要求为 0，则该要求与本实现的安全占位行为不同，需由 Codex
判定，此处不隐瞒。

### 2.2 成功会话（`map05_full_4/`）

reset `(2,2,0)`（`rviz_reset`，context `virtual-a-reset-None-1`），
goal `(8,2,0)`。`report.json`：`ok=true`、70 checks、0 failed。

| 字段 | 值 |
|---|---|
| plan_id | 600000 |
| request_sha256 | `e69d068b1cae75fa7205a7530b129f16bf731dddce705e6d2ef8042fec07aeaf` |
| planning_context_sha256 | `06aed480a711839483c94e42648563b53c48eb0080f3c9dfe4e16daebe9414dd` |
| model_sha256 | `e6b6e55db62346d041e959b1955e031d33bd571995e53483b46c1c962635bcad` |
| grid_sha256 | `64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321` |
| sample_count | 712（interior 710） |
| T_end | `14.203125000000316` s |
| certification | `virtual_a_geometry_dynamics_collision/v1` |
| envelope numerics | `version=virtual_rigid_a.numerics.g1.v3`；`position_error_bound_m=1.3589865068749644e-09`；`angle_error_bound_rad=1.6000177635683942e-10` |
| terminal_qf | `[8.0, 1.9999999999999998, -3.944304526105059e-31]`；位置误差 `2.22e-16 m`，航向误差 `1.09e-15 rad` |

进度事件序列（`publications.jsonl`，plan 600000，共 12 条）：

```text
REQUEST_ADMITTED → ASTAR_SEARCH → CORRIDOR_BUILD → NLP_SOLVE(×2)
→ GEOMETRY_AUDIT(×2) → DYNAMICS_AUDIT(×2) → OBB_COLLISION_AUDIT(×2)
→ PUBLISHED
```

每条事件 document 均含 `plan_id / request_sha256 / planning_context_sha256 /
grid_sha256 / model_sha256 / stage_index / elapsed_s`（`document_keys` 逐条
记录）；最终 `PUBLISHED` 采样保存了实际值：plan 600000、
request `e69d068b…`、context `06aed480…`、grid `64eef089…`、model
`e6b6e55d…`，`elapsed_s=6.174505471950397`。前 11 条事件的各字段值与
elapsed 数值未逐条保存（仅记录键集合），标 `UNAVAILABLE`，不补造。

数值审核（来自 `sample_quality_report_plan.txt` 与
`sample_planning_debug_plan.txt`）：

- geometry / dynamics / collision 三项 audit 全部 `passed=true`；
  collision `check_count=3740`（static start/goal + 全闭合路径三车 OBB
  扫描），`first_failure=null`。
- solver `ipopt`，attempt `status=SOLVER_SUCCESS_AUDITED`，
  `constraint_residual=1.7763568394002505e-15`，
  `corridor_residual_m=0.0`。
- `S_lower_m=6.0, S_solution_m=6.0, S_upper_m=28.0`；`S_m=6.0`。
- `corridor_halfspace_constraint_count=352`；`corridor_max_residual_m=0.0`；
  containment `passed=true`、`certified=true`、8 个采样、
  `max_bound_residual_m=-1.6249999486400117`。
- corridor chain digest `e7f8568f2397a95f44a0dc2693a1fea4e8f6bd50635dd80ce0e32c56eee4b618`；
  guide `PL_GUIDE`（`[[2,2],[8,2]]`），`route_source=astar_primary`。
- candidate `classification=CERTIFIED`、`certified=true`。
- winding 候选排序：k=0（rank 0，selected）、k=−1（rank 1）、k=+1（rank 2）；
  三者均来自 `corridor_endpoint_polyline`，base heading 0。

发布与播放：

- `/formation/playback_plan` 原始 2 条：启动占位 plan 0（不可执行，0/0）
  与 plan 600000（`model_json_length=18921`、`sample_count=712`、
  `executable=true`）。**本 plan 可执行发布计数 = 1**。
- 可执行信封到达（wall `1795029032.541922`）早于 `PUBLISHED` 事件
  （`1795029032.559619`），符合 §2.2 的“先发布、后 PUBLISHED”。
  发布 stamp `1795029032.532143115`；quality `verification_to_publication_stamp_delay_s=0.9756836891174316`。
- 播放确认：job_state 序列 7×`PLANNING`、10×`PLAYING`、3×`HOLD_END`；
  playback 状态 `PLAYING → HOLD_END`，`duration_s=14.203125001`、
  `elapsed_s=14.21865201`，`terminal=true`，terminal center/heading error
  均为 0，`model_sha256` 与信封一致。
- 显示：hold `marker_count=19`，namespace 5 个：
  `formation/cargo_wedge/links`、`formation/cargo_wedge/reference`、
  `formation/cargo_wedge/vehicle_1/2/3`。driver 的 marker/TF/status
  位置、角度、采样数检查全部通过（70 checks 全过）。

截图：`images/launch.png`、`images/plan1_{start,mid,end}.png`、
`images/plan.png`；规范命名副本
`images/plan_600000_{start,mid,end}_passed.png`。

## 3. S1 场景（修复前诊断、修复后成功与失败恢复）

S1 使用冻结场景
`evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json`
（`scene_key=S1`，manifest SHA `f956e831…`，map SHA `054cf5c7…`，
runtime grid `6e94c751…`），corridor/hard，`auto_goal:=false`。

### 3.1 `s1_easy_corridor_fixed/`（成功）

launch：`backend:=virtual_rigid_a auto_goal:=false rviz:=true
virtual_a_goal_policy:=free map_dir:=<S1> virtual_a_guidance_mode:=corridor
virtual_a_corridor_enforcement:=hard scene:=manual_map`。显式小目标
`(5.0, 0.0, 0.0)`。结果：83 checks、0 failed。

- plan 600000：request `e453ff9d…`，model `4a3e180f…`，593 samples，
  `T_end=11.835937515 s`，certification `virtual_a_geometry_dynamics_collision/v1`，
  HOLD_END、19 markers。
- 12 条进度事件，序列与 §2.2 相同，`PUBLISHED` 在可执行信封之后
  （`1795026933.898` vs `1795026933.901`）。
- `/formation/playback_plan` 原始 2 条：plan 0 占位（不可执行）+ plan
  600000（18286 B / 593，可执行）；**可执行 1**。
- 继续请求 plan 600001 被拒绝 `UNSUPPORTED_INITIAL_STATE`
  （`free_continuationrejection_*` 检查通过），未发布可执行信封。
- 截图：`images/plan_600000_{start,mid,end}_passed.png`、
  `plan1_{start,mid,end}.png`。

### 3.2 `s1_failure_corridor_fixed/`（失败 + 恢复）

launch：同上，`virtual_a_goal_policy:=straight`。结果：155 checks、0 failed。

- 失败 plan 600001：`NO_ROUTE_GUIDE`，job FAILED；进度仅
  `REQUEST_ADMITTED → ASTAR_SEARCH → FAILED(×2)`，无 `PUBLISHED`。
  `/formation/playback_plan` 原始 1 条失败占位（plan 600001，
  `model_json_length=0`, `sample_count=0`，不可执行），**可执行 0**；
  playback `CLEARED`（marker_count 0）。
- 首个有效 plan 600000：可执行 1（19132 B / 593），HOLD_END，19 markers。
- 恢复 plan 600002：request `682108ac…`，model `013a9a44…`，593 samples，
  `T_end=11.835937507 s`，12 条进度事件、`PUBLISHED` 在信封后
  （`1795027351.153` vs `1795027351.157`），可执行 1（19134 B / 593），
  HOLD_END，19 markers。
- 场景窗口内原始计数：plan 0 占位 3 条、plan 600000 可执行 1、plan 600001
  失败占位 1、plan 600002 可执行 1；**可执行合计 2，失败 plan 可执行 0**。
  `publications.jsonl` 中 wall ≥ `1795027410` 的 11 条 plan 600002 可执行
  记录来自后续 tamper 诊断的原始信封重放，不是 planner 再次发布。
- 截图：`images/plan_600000_*_passed.png`、`plan_600001_failure_cleared.png`、
  `plan_600002_*_passed.png`、`recovery_{start,mid,end}.png`、
  `failure_cleared.png`、`plan1_*`。失败截图同时保留 HUD reason 与图层。
- 显示：driver 的 marker/TF/status 位置、角度、采样数检查全过
  （display 证据 `display/plan1_display.json`、`display/recovery_display.json`）。

## 4. 播放节点 preview 回归与最小修复

`s1_easy_corridor/`（修复前）记录：planner 已发布 1 个可执行信封
（18286 B / 593），但 5 s 内播放未确认，`REJECTED /
PLAYBACK_START_TIMEOUT`。根因是 Step 1 为虚拟后端新增的 raw-request
提前返回位于 `self._previewing = False` 之前，reset 后 transport preview
不结束，`on_playback_plan` 静默丢弃信封。

修复只改
`catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`：
虚拟 `on_request` 在非 `PLANNING/PLAYING` 或请求 plan 与已准入 plan
匹配时结束 preview，busy 请求保持 preview；`on_playback_plan` 对 preview
丢弃增加 5 s 节流 warning。未改 audit、阈值、走廊/optimizer 数学。
`tests/test_virtual_a_receiver.py` 新增 3 个用例。修复后
`s1_easy_corridor_fixed/` 达到 `PLAYING` 并由 planner 确认；
完整诊断见 `diagnostics/README_playback_preview_regression.md`。

## 5. 时间轴 tamper 诊断

见 `diagnostics/README_tamper_time_boundary.md` 与
`s1_failure_corridor_fixed/report_tamper*.json`、`diagnostics/tamper_time_1ms.json`。
7 个负例中 6 个被拒绝并 `CLEARED`、0 个 `PLAYING`；中点 +1 ns 的用例
未拒绝，因为该信封在采样点的局部变化率低于验证器 1e-9 容差（差约
7.9e-10 m），属于容差边界。独立 1 ms 诊断被确定性拒绝
（`differs ... by 0.000792079 > 1e-09`），随后原始信封重放至 `HOLD_END`。
未修改生产容差或测试 fixture；该边界不计为 TASK 01 门禁失败。

## 6. 后端 smoke（实际结果）

### 6.1 `backend_uniform_v02/`

launch：`map16-s09-underground-garage`，scale 3.75，
`backend:=uniform_v02 auto_goal:=true`。参数采样：backend `uniform_v02`、
guidance `corridor`、enforcement `hard`、goal_policy `free`。自动目标
`(2.725, -11.688, 1.571)`，plan 600000 **失败**：
`pipeline_status=BUDGET_EXHAUSTED`，`reason=TIME_BUDGET_EXHAUSTED`
（wall `30.9137105 s ≥ 30 s`），`executable=false`；search：candidate
1665、expansions 9、collision_s `30.1579`，rejection histogram
`A_HEADING_REQUIRES_S_ALIGNMENT=1152`。`/formation/playback_plan` 采样
533 B，`model_json`/trajectory/certification 为空（不可执行）；
`/formation/job_state` 542 B、`quality_report` 1785 B。节点：
`formation_goal_bridge / formation_playback / formation_uniform_planner /
fwis_map_publisher / rosout / rviz`。截图 `images/result.png`。

### 6.2 `backend_legacy/`

launch：同一地图/比例，`backend:=legacy auto_goal:=true`；自动目标相同。
`launch.log`：`[v3 auto goal] sent center goal (2.725, -11.688, 1.571)`，
随后 `WARN [formation planner] geometry search failed:
FORMATION_BUDGET_EXHAUSTED`。`/formation/status`：
`node_state=GEOMETRY_SEARCH_FAILED`，`reason=FORMATION_BUDGET_EXHAUSTED`，
`expanded_nodes=50000`、`generated_nodes=551756`、`goal_candidates=0`、
`trajectory_published=false`、`executable=false`。`/formation/timed_trajectory`
仅 header/空数组（207 B）。该会话**没有** `/formation/playback_plan`、
`/formation/planning_debug`、`/formation/job_state` 话题（采样 exit 124、
0 bytes，`UNAVAILABLE`）。`/formation/playback/backend` 与
`/formation_goal_bridge/admission_backend` 均为 `legacy`；
`rosparam get /formation_uniform_planner/backend` 返回 `uniform_v02` 是
上一会话遗留的 ROS 参数（legacy 会话未启动该节点），不能当作本次
backend；以 launch.log 的 `backend:=legacy` 与 playback/admission 参数为准。
节点：`/formation_planner /formation_optimizer /formation_goal_bridge
/formation_playback /fwis_map_publisher /rviz`。截图 `images/result.png`。

### 6.3 `single_smoke/`（`--single --map map05`）

launch：`demo.launch map_dir:=<map05> planning_backend:=mode_path_v2
scene:=manual_map auto_goal:=true`，地图比例 0.7。参数：
`/fwis_v3_planner/planning/backend`、`/fwis_v3_optimizer/planning/backend`、
`/fwis_v3_playback/planning/backend` 均为 `mode_path_v2`；`length=0.738`、
`time/dt=0.1`、`direction_policy=bidirectional`。自动目标
`(6.324, 4.576, 1.571)`；planner `search_success id=1`（60 actions，
125000 iterations，1.134 s）；optimizer `success id=1
source=mode_path_v2 nodes=38 time=2.35 s common_score=14.333704
validity=FEASIBLE duration=15.080`。`/plan/timed_trajectory` 62,589 B
（plan_id 1）；`/plan/status` 294 B、`/plan/quality_report` 836,919 B、
`/plan/search_status` 5,027 B。播放结束 `/robot/hud` 文本
`v3 #1 OBLIQUE center v=0.00 m/s`，位置 `(6.3255056, 4.5736068)`
与目标一致。该会话不启动 formation 栈，没有 `/formation/*` 证据；
`rosparam list` 中残留的 formation 参数（`/formation_uniform_planner/*`、
`/formation_playback/backend` 等）来自上一会话，本会话未使用，不能作为
本次 backend 证据。截图 `images/result.png`。

## 7. §6 必填字段位置对照

| 要求 | 位置 / 值 |
|---|---|
| progress 事件序列 | `map05_full_4/publications.jsonl`、S1 fixed `publications.jsonl`；§2.2/§3 |
| 每事件 identity | 每条 debug 记录 `document_keys` 含 plan/request/context/grid/model/stage_index；`PUBLISHED` 采样给出实际值 |
| Marker namespace/点数 | hold `marker_count=19`、5 namespaces（§2.2）；S1 hold 19 markers |
| candidate classification | `map05_full_4/sample_planning_debug_plan.txt`：`CERTIFIED`/`certified=true` |
| playback 发布计数 | 原始 vs 可执行逐条列于 §2–§3；失败 plan 可执行 0 |
| halfspace constraint count/residual | 352 / `0.0`（map05）；attempt residual `1.78e-15` |
| winding 候选排序 | k=0/rank0(selected)、k=−1/rank1、k=+1/rank2（§2.2） |
| S packet | `S_lower=6.0, S_solution=6.0, S_upper=28.0`；离线 S 包证据见 `../04_s_domain/s_domain_600003_600004.json` |
| v2/v3 certificate components | 信封 `numerics.version=virtual_rigid_a.numerics.g1.v3`、position/angle bounds；离线组件见 `../05_numerics/` |

## 8. 历史证据、停止与边界

- 未修改 `../00_baseline/`…`../05_numerics/`；`git status --porcelain --
  evaluation` 为空；06 仅新增 `postfix/` 与 §6 复跑记录。
- 修复前证据 `s1_main_free/`、`s1_easy_corridor/`、`map05_full/`、
  `map05_full_2/`、`map05_full_3/` 原样保留，未覆盖。
- 全部会话已停止，`stop.txt` 记录 roslaunch/session pid；会话停止后
  私有 master 11633 上只剩 `/rosout`，随后任务私有 roscore 也已停止，
  端口 11633 已关闭。11411 master 与 pid 4042262 未触碰。
- 未 commit/push；未进入 TASK 02；不宣称 Codex PASS 或 TASK 01 验收。