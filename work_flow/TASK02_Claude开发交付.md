# TASK02 Claude 开发交付

生成时间：2026-11-19（UTC）。任务：`work_flow/TASK02_技术方案.md` + `work_flow/TASK02_Codex执行方案.md` 的 TASK02。
基线 HEAD：`bee4e3aa0f86e24085a1840ee0581b2fca34add8`（`bee4e3a Separate RViz path categories and active route aliases`），分支 `feature/virtual-rigid-formation-a`。

本文件是 Claude 的实现和证据说明，不是 Codex 审核结论；不宣称 Codex PASS，也不宣称 TASK01/TASK02 验收通过。TASK02 完成后已停止：未 commit、未 push、未开始 TASK03。

## 1. 最终状态与保护约束核对

- 工作树：`/home/wq/whut_thesis/expv3_multi_virtual_a`（git worktree）。
- 代码未提交；`git status --short --branch` 显示分支相对 `origin/feature/virtual-rigid-formation-a` 领先 1 个既有提交，工作树为 TASK02 改动 + 既有 dirty。
- TASK02 开始前的 TASK01 dirty 两项未被修改，SHA-256 与基线逐字节一致（`baseline/task01_dirty_sha256.txt`）：
  - `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py`
    `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d`
  - `tests/test_virtual_a_geometry.py`
    `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7`
- 未修改历史 `TASK01_evidence/`、`evaluation/`、输入方案文档、模型 fixture、TASK01 已接受测试期望；未清理、未移动既有未跟踪文件和 `latest` 符号链接。
- 未修改 `run_demo.sh`、`formation_demo.launch`、launch 发布合同和默认 backend/guidance 路由；默认入口本来已是 `virtual_rigid_a/corridor/hard/auto_goal=false`（dry-run 与实跑参数见第 8 节）。
- 真实 ROS 只用任务私有 master `http://localhost:11633`；另一个会话的 `11411`（pid 3039917/3039925/4042262 系列）未被触碰。证据中 `nohup`/`setsid` 只用于本任务自己启动的 `run_demo.sh`。
- 最终 diff 统计（`source.diff`，TASK02 自身 9 个已跟踪文件 + 1 个新增测试；TASK01 dirty 单列不计入）：

```text
已跟踪 9 文件：9 files changed, 4027 insertions(+), 22 deletions(-)
新增 tests/test_virtual_a_formation_assessment.py：762 行（source.diff 共 10 个 diff 段）
```

`geometry.py`、`tests/test_virtual_a_geometry.py` 的既有修改完整保留，未混入 TASK02 diff。

## 2. 实际改动文件、函数与调用路径

| 文件 | 主要新增/修改 | 调用路径 |
|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/corridor.py` | Step B：完整 OBB 多边形/支持函数、单区与相邻并集完整覆盖、`_guide_region_violation()`；Step C：`formation_transition_windows()`、`assess_formation_guide()`、`assess_route_guide()`；Step D/返工 A：`select_guide_target_portal()`（基线风险 Portal 优先、无风险最小 Chebyshev 半径）、`plan_guide_candidates()`（≤5，含 ≤3 单 Portal 局部变体）、`corridor_guide_candidates()` 兼容包装；Step E/返工 B：`corridor_portal_diagnostics()`（只读、有界，`single` 与 `union` 分列） | `pipeline.plan_virtual_a()` 先评估 QP base，再用同一 baseline 生成候选；`planner_node._display_from_corridor_round()` 附加 `portal_diagnostics` |
| `.../formation_continuous_a/pipeline.py` | `_ProgressEmitter`（只发 `CORRIDOR_BUILD/PORTAL_GUIDE/ASTAR_SEARCH/NLP_SOLVE/FAILED`，禁止 `PUBLISHED`）；`_route_assessment_score()`/`_route_attempt_order()`；`untried_route_index()`；按候选评估的字典序选择；`corridor_round` 的 `guide_candidates`/`guide_selection`/`formation_assessment`；round 与 solver 记录 `route_index`/`corridor_round_index`/`chain_digest` | `plan_virtual_a()` 的 A* 后粗评估 → route 尝试顺序 → chain audit → `make_corridor_guide()` → `allocate_corridors()` → `solve_open_path_guided()` → 原有三段独立 audit |
| `.../formation_playback.py`（optimizer 显示模块） | `planning_diagnostics_view()`（从真实 progress 文档复制有界标量，缺失为 `None`）；`hud_text()` 追加两行 diagnostics；`PORTAL_WINDOW_NAMESPACE`/`MAP_COLLISION_SAMPLE_NAMESPACE`/`COVERAGE_GAP_NAMESPACE`、颜色、`_append_portal_diagnostics()`；`build_candidate_display()` 的 `DIAGNOSTIC_ONLY` 标签 | playback 节点 HUD 与 `/formation/guides`、`/formation/planning_candidates` MarkerArray |
| `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py` | `_record_planning_progress()` 同 plan/generation/round 保留诊断，新 round/新 plan 不继承；`on_planning_debug` 订阅队列 `1 → 50`（见第 8.4 节）；HUD 使用 retained view | ROS 回调层 |
| `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py` | `_display_from_corridor_round()` 调 `corridor_portal_diagnostics()`；`_progress_document()` 透传真实 `metrics`；`/formation/planning_debug` 发布队列 `1 → 20`（见第 8.4 节） | ROS 节点 `/formation/planning_debug` latched 发布 |
| `catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz` | 新增 `Portal diagnostics (markers)`，只打开 `formation/planning/{portal_window,map_collision_sample,coverage_gap}`；既有 `Guide labels (markers)` 保持 disabled、name 不变；Path 颜色未改 | RViz 配置 |
| `tests/test_virtual_a_formation_assessment.py`（新增） | Step C/D 的几何、采样状态、窗口、候选与 identity 测试 | 单元测试 |
| `tests/test_virtual_a_pipeline.py` | 追加 route 顺序/候选选择真实进入 solver 的 spy 测试与 round identity 断言 | 单元测试 |
| `tests/test_virtual_a_corridor_display.py` | 追加有界 diagnostics、marker/namespace/颜色、fail-closed、progress 透传测试 | 单元测试 |
| `tests/test_formation_hud.py` | 追加真实计数、缺失 unknown、fail-closed、round 保留的 HUD 测试 | 单元测试 |

关键源码位置（最终工作树）：`corridor.py` 的 `formation_transition_windows()` L841、`assess_formation_guide()` L1896、`select_guide_target_portal()` L3166、`plan_guide_candidates()` L3486、`corridor_portal_diagnostics()` L4290；`pipeline.py` 的 `_route_attempt_order()` L819、候选生成 L1778–1797；`formation_playback.py`（显示模块）的 `planning_diagnostics_view()` L2064、`_hud_diagnostics_line()` L2113；队列修正见第 8.4 节。

## 3. 算法与诊断边界

- 单区与相邻并集覆盖均使用**完整旋转矩形 OBB 多边形**和真实 `map_spatial_state()` / `FormationCollisionChecker.static_check()`，不是四角或中心采样；支持函数对照误差 `6.66e-16 m`（`geometry/stepB_geometry_results.json`）。
- 并集按 `C_j ∪ C_{j+1}` 的两种顺序独立检查，正反结果一致才判 `COVERED_BY_ADJACENT_UNION`；diamond 反例判 `GAP`，不会 false PASS（U1/U2 见 `geometry/stepB_geometry_results.json`）。
- 状态分离：
  - map：`CLEAR_AT_SAMPLES` / `COLLISION_OBSERVED`（`MAP_OBB_COLLISION`、`VEHICLE_OBB_COLLISION`）/ `UNCONFIRMED`；
  - coverage：`SINGLE_REGION` / `COVERED_BY_ADJACENT_UNION` / `GAP` / `UNCONFIRMED_NUMERICS`；
  - 任何缺 shape、采样上限、退化 Guide、checker 异常或超时都降级为 UNKNOWN，不伪造 clear/gap。
- 所有诊断固定：
  - `candidate_certification=DIAGNOSTIC_ONLY`
  - `continuously_certified=false`
  - `formation_obb_hard_constraints=false`
  - `certification_claim=NONE`
- 有界上限：Guide 候选 5（单 Portal 变体 3）；route 粗评估 ≤3；`PORTAL_DIAGNOSTIC_COLLISION_LIMIT=5`、`GAP_LIMIT=6`、`WINDOW_LIMIT=3`。marker 截断时 `counts` 仍保留完整 assessment 总数。
- `legacy_f_allocation` 始终提供既有 F-only halfspace 约束；`transition_windows` 只用于诊断，不替代 F-only 约束。
- 原有 `audit_corridor_chain`、连续 F containment、geometry/dynamics/collision 三段独立 audit、receiver identity/publish gate 的调用位置和阈值未改。

## 4. Route/候选如何真正进入 solver

1. A* 成功后，对 ≤3 条原始 route 用真实 model/shapes/grid 做粗评估，写入 `search['corridor']['route_assessments']`。
2. `_route_assessment_score` 固定字典序：
   `(map_status_rank, collision_count, unconfirmed_count, route_length_m, original_route_index)`；
   `CLEAR_AT_SAMPLES < COLLISION_OBSERVED < UNCONFIRMED/缺失`。
3. `route_attempt_order` 只排序**原始 A* index**；`routes` 元组、A* topic ID、active alias 保持原编号；首次 `route_index = route_attempt_order[0]`，`untried_route_index()` 供既有单次 recovery 使用。
4. chain audit 通过后 `make_corridor_guide()` 构造 base，先对 QP base 做一次真实三车 baseline assessment（复用于候选生成，不重复调用 checker）；返工 A 的 `select_guide_target_portal()` 按“最早 map collision/union gap/transition unknown → 无风险时最小 Chebyshev 半径 → Portal index 升序”选一个目标 Portal，再生成 ≤3 个只改该 Portal 点的局部变体，总计 ≤5 个候选；每个候选独立执行 `allocate_corridors()`，成功者再执行真实 `assess_formation_guide()`。
5. 只在 solver-compatible 候选中按固定字典序选择：
   `(assessment_available, has_collision, collision_count, has_invalid, invalid_count, gap_count, coverage_unknown_count, map_unknown_count, guide_rank, length_m, candidate_index)`。
   规则字符串为 `SINGLE_BASE_GUIDE` / `DIAGNOSTIC_LEXICOGRAPHIC_BEST` / `ASSESSMENT_UNAVAILABLE_BASE_FALLBACK`。
6. **选中候选的 Guide 点列、该候选自己的 allocation、原始 chain** 一起传入 `solve_open_path_guided()`；`tests/test_virtual_a_pipeline.py::RouteAndGuideSelectionTests` 用 solver spy 断言三者与 round 记录同源，且 `corridor_chain.digest == chain_digest`。候选不改变 executable flag，也不能直接发布 `/formation/playback_plan`。

## 5. 失败与 UNKNOWN 语义

- 评估异常：记为 `FORMATION_ASSESSMENT_UNAVAILABLE` / `CHECKER_UNAVAILABLE`，仍调用真实 solver；不可用候选排在可用候选之后，全部不可用时回退 base Guide。
- `allocate_corridors()` 不兼容：记录 `FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE` 并拒绝该候选；base 不兼容则保持既有 `CORRIDOR_ALLOCATION_FAILED`。
- 失败计划：保留本 plan 已发布的诊断图层，不把失败候选升级为 certified；`CLEARED`/pose reset/busy rejection 沿用 TASK01 生命周期。
- 诊断 marker 身份错误（schema、`chain_digest`、`guide_source`、claim、越界计数）只清 Guide 层，不清 corridor/其他层。

## 6. 与 TASK01 基线的差异

- 默认入口和算法安全链未变：`./run_demo.sh` 仍解析为 `backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false`；未新增隐藏参数。
- 新增的是 route 尝试顺序、Guide 候选选择、采样诊断、progress/HUD 有界字段和 RViz 诊断图层；`/formation/playback_plan` schema、播放时间轴、三车 certified 路径与 safety gate 未改。
- RViz 既有 Path 颜色/宽度、`Guide labels (markers)` 未改；只追加一个独立的 `Portal diagnostics (markers)` 显示项。
- TASK01 dirty 两项完整保留，未混入本任务 diff。

## 7. 测试与构建

环境（仓库根目录）：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
```

### 7.1 Step B 几何

| 命令 | 结果 | 耗时 | exit |
|---|---|---|---|
| `python3 -m unittest tests.test_virtual_a_formation_assessment`（Step B 初始 17 tests；该文件在 Step C/D 扩展到 32 tests） | Ran 17 tests, OK | 0.003 s | 0 |
| `geometry/stepB_corridor_regression.log`（回归 52 tests + 真实 50×50 chain 构建/审计计时） | Ran 52 tests, OK；`c1_50x50_build_s=0.014560 audit_s=0.004654` | 9.801 s | 0 |

`geometry/stepB_geometry_results.json` 给出 U1 正例（正反两序 `COVERED_BY_ADJACENT_UNION`，最大违例 `-0.6 m`）、U2 diamond `GAP`、O1 支持函数误差 `6.66e-16 m`。

### 7.2 Step C/D 定向测试

| 日志 | 命令（模块） | 结果 | 耗时 | exit |
|---|---|---|---|---|
| `candidate/step_c/tests_formation_assessment.log` | `tests.test_virtual_a_formation_assessment` | Ran 32 tests, OK | 0.260 s | 0 |
| `candidate/step_c/tests_pipeline.log` | `tests.test_virtual_a_pipeline`（Step C 版本 10 tests） | Ran 10 tests, OK | 112.883 s | 0 |
| `candidate/step_d/tests_pipeline.log` | `tests.test_virtual_a_pipeline`（最终 13 tests） | Ran 13 tests, OK | 119.670 s | 0 |
| `candidate/step_d/tests_corridor_recovery.log` | `tests.test_virtual_a_corridor_recovery` | Ran 29 tests, OK | 18.207 s | 0 |
| `candidate/step_d/tests_corridor_optimizer.log` | `tests.test_virtual_a_corridor_optimizer` | Ran 27 tests, OK | 9.322 s | 0 |

### 7.3 Step E 显示/HUD 测试

`candidate/step_e/tests_display.log`（Step E 原始）与 `tests_display_after_queue_fix.log`（队列修正后的最终工作树）均为：

```bash
python3 -m unittest \
  tests.test_virtual_a_corridor_display tests.test_formation_hud \
  tests.test_virtual_a_receiver tests.test_virtual_a_playback \
  tests.test_formation_playback tests.test_planning_display \
  tests.test_formation_demo_assets
```

| 日志 | 结果 | 耗时 | exit |
|---|---|---|---|
| `tests_display.log` | Ran 194 tests, OK | 78.408 s | 0 |
| `tests_display_after_queue_fix.log` | Ran 194 tests, OK | 80.160 s | 0 |

### 7.4 Step F 离线回归（最终源码）

每个模块单独运行并保存原始 stdout/stderr 与 exit：

| 日志 | 模块 | 结果 | 耗时 | exit |
|---|---|---|---|---|
| `candidate/step_f/tests_formation_assessment.log` | `test_virtual_a_formation_assessment` | Ran 32, OK | 0.250 s | 0 |
| `tests_corridor.log` | `test_virtual_a_corridor` | Ran 25, OK | 0.373 s | 0 |
| `tests_corridor_optimizer.log` | `test_virtual_a_corridor_optimizer` | Ran 27, OK | 9.436 s | 0 |
| `tests_pipeline.log` | `test_virtual_a_pipeline` | Ran 13, OK | 119.724 s | 0 |
| `tests_corridor_recovery.log` | `test_virtual_a_corridor_recovery` | Ran 29, OK | 18.234 s | 0 |
| `tests_corridor_display.log` | `test_virtual_a_corridor_display` | Ran 63, OK | 17.018 s | 0 |
| `tests_receiver.log` | `test_virtual_a_receiver` | Ran 21, OK | 17.281 s | 0 |
| `tests_geometry.log` | `test_virtual_a_geometry` | Ran 24, OK | 1.104 s | 0 |
| `tests_collision.log` | `test_virtual_a_collision` | Ran 17, OK | 2.775 s | 0 |
| `tests_optimizer.log` | `test_virtual_a_optimizer` | Ran 37, OK | 66.341 s | 0 |
| `tests_integration_bounds.log` | `test_virtual_a_integration_bounds` | Ran 6, OK | 0.686 s | 0 |
| `tests_playback.log` | `test_virtual_a_playback` | Ran 34, OK | 53.814 s | 0 |
| `tests_trajectory.log` | `test_virtual_a_trajectory` | Ran 26, OK | 0.829 s | 0 |
| `tests_formation_demo_assets.log` | `test_formation_demo_assets` | Ran 29, OK | 3.742 s | 0 |
| `tests_formation_hud.log` | `test_formation_hud` | Ran 23, OK | 0.004 s | 0 |

队列修正后的补充回归：

| 日志 | 模块集合 | 结果 | 耗时 | exit |
|---|---|---|---|---|
| `tests_after_queue_fix.log` | 107 tests（planner 队列修正后的显示/播放/接收回归批次） | OK | 33.800 s | 0 |
| `tests_after_playback_queue_fix.log` | 102 tests（`playback`、`formation_playback`、`planning_display`、`formation_hud`、`receiver`） | OK | 57.825 s | 0 |

### 7.5 构建与静态检查

| 命令 | 结果 | exit |
|---|---|---|
| `catkin_make`（`candidate/step_f/catkin_make.log`、`catkin_make_step_f2.log`） | 完成，`catkin_make.exit=0` | 0 |
| `bash -n run_demo.sh`（`bash_n_run_demo.log`） | 无语法错误 | 0 |
| `git diff --check`（`git_diff_check.log`） | 无 whitespace 错误 | 0 |

### 7.6 已知基线失败（原样保留，不归因 TASK02）

`test_continuous_retiming` 在 TASK02 基线中即有 18 tests / 1 error（`KeyError: 'dynamic_peaks'`），TASK01 开发交付和审核报告均记录为既有问题；TASK02 未修改该测试或实现，也未声称修复。Step F 未复跑该模块。

## 8. 真实 ROS/RViz 实跑

> 本节 8.1 的原始 HUD 文本产生于返工前，字段 `union covered=…` 把 `SINGLE_REGION` 与 `COVERED_BY_ADJACENT_UNION` 混在一起，是返工 B 要修正的旧语义。返工后的默认入口重跑与最终 HUD 语义见 §12；历史原始消息仍保留在 `candidate/step_f/` 中，未改写。

### 8.1 环境与命令

- master：任务私有 `http://localhost:11633`（`candidate/step_f/master_11633/roscore.pid`、`roscore.log`）。
- 默认命令（未补任何隐藏 backend/guidance 参数）：
  - map05：`./run_demo.sh --map map05 --scale 2.625`
  - S1：`./run_demo.sh --map evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S1 --scale 1.0`
- RViz：脚本自动 `rviz:=true`（GUI 可用）；`formation_demo.rviz` 由 `run_demo.sh` 复制到临时 runtime 目录后启动。
- 每次运行保存：`launch.log`、`params_launch.yaml`（`rosparam dump /`）、`nodes_launch.txt`、`topics_launch.txt`、`roslaunch_nodes.txt`、runtime map 复制、`nodes_raw/` 原始节点日志、`images/*.png`。
- 两个场景的请求都由驱动脚本按 RViz 用户操作语义发布：`/initialpose`（2D Pose Estimate）后再发 `/move_base_simple/goal`（2D Nav Goal）；不发布参考轨迹，不写 `/formation/playback_plan`。

map05 实跑（`candidate/step_f/map05_default/`）：

- 实际 launch 行：`map_dir:=/tmp/expv3-map-scale.QTEIpV/map05-p12-parking-row-env2 backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false scene:=manual_map rviz:=true`。
- 节点：`/fwis_map_publisher`、`/formation_uniform_planner`、`/formation_playback`、`/formation_goal_bridge`、`/rviz`。
- 地图：489×326，res `0.06487162411212921`，origin `(-8.3609838125, -5.573955875)`，runtime grid SHA-256 `64eef089eb2be976589f9b7a2b2f4aa889452c86a08b7a843f9402ffb73af321`（与 TASK01 map05 基线一致）。
- 请求：reset `(2, 2, 0)`（context `virtual-a-reset-None-1`），goal `(8, 2, 0)`，plan `600000`；`request_sha256=e69d068b…`，`planning_context_sha256=06aed480…`，`model_sha256=e6b6e55d…`。
- 终态：`HOLD_END`；`executable envelope` 计数 **1**，无 non-executable envelope。
- 三段独立 audit：HUD/质量报告均 `geometry=PASS dynamics=PASS collision=PASS`；`virtual_result.certification="virtual_a_geometry_dynamics_collision/v1"`，`checks=4804`。
- 质量报告：`route_count=1`，选中 route `0`，round `0`，chain `e7f8568f2397a95f44a0dc2693a1fea4e8f6bd50635dd80ce0e32c56eee4b618`；Guide `corridor_endpoint_polyline`，2 点，长度 `6.0 m`；候选 1/1，选中 `0`，规则 `SINGLE_BASE_GUIDE`；allocation 8 段 `[0,0.125,…,1.0]`。
- 真实 assessment：`samples=13`，`map_collisions=0`，每车 `single=13`（union covered 合计 39），`gap=0 unknown=0`，`CLEAR_AT_SAMPLES`/`SINGLE_REGION`，无 transition window。
- solver round：`corridor_round_index=0`、`route_index=0`、`guide_length_m=6.0`、`winding_k=0`、`GEOMETRY_FEASIBLE`。
- `/formation/planning_debug`：驱动收到 17 条文档（含 ASTAR/CORRIDOR/PORTAL_GUIDE 的 `complete`）；playback 节点日志收到 16 次 stage 记录（阶段 1–7 各 running+complete、阶段 0 与 8 各一次）。
- HUD 最终文本（`hud_texts.json`）：

```text
planning PUBLISHED | step 8/8 | route 0/1 | round 0 | elapsed 6.4 s
diagnostics | guide=corridor_endpoint_polyline | selected=0/1 | SINGLE_BASE_GUIDE
samples=13 | map_collisions=0 | union covered=39 gap=0 unknown=0 | status=CLEAR_AT_SAMPLES
```

S1 实跑（`candidate/step_f/s1_history/`）：

- 实际 launch 行：`map_dir:=/tmp/expv3-map-scale.8JKqoP/S1 backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false scene:=manual_map rviz:=true`。
- 地图：409×302，res `0.05`，origin `(-6.8, -4.8)`；runtime grid SHA `6e94c751a40ec94431b495fcbff002945f09c975429c493c89060f0e7e2aa2a4`；源 `map.pgm` SHA `0e975d9a…` 与 TASK02 基线记录一致。
- 请求：reset `(0, 0, 0)`，goal `(8.819581504169413, 3.4858722492682084, 1.5707963267948968)`，plan `600000`。
- 终态：`FAILED`，原因 **`CORRIDOR_AUDIT_WIDTH_LIMIT`**；`executable envelope` 计数 **0**，另有 1 条 non-executable envelope 记录（`sample_count=0`）。历史失败结论/原因保持，未通过降低阈值掩盖。
- 质量报告：`status=GEOMETRY_FAILED`、`first_failure_layer=GEOMETRY`、`route_count=3`、`route_attempt_order` 使先试 route 1，再试 route 0；两轮均 `map_sample_status=COLLISION_OBSERVED`、`coverage_sample_status=GAP`：
  - round 0 route 1：chain `0ddd65e0…`，solver `guide_length=10.152706189722945`；
  - round 1 route 0：chain `c2882091a085…`，solver `guide_length=9.839848804515219`；
  - round 1 的 `GEOMETRY_AUDIT` 以 `CORRIDOR_AUDIT_WIDTH_LIMIT` 终止整次规划（hard containment failure），因此第三条 route 未被尝试。
- HUD（`FAILED` 时保留最后 selected route/Guide 诊断）：

```text
planning FAILED | step 5/8 | route 0/3 | round 1 | elapsed 50.0 s | CORRIDOR_AUDIT_WIDTH_LIMIT
diagnostics | guide=corridor_candidate_chebyshev | origin=CHEBYSHEV_CENTRE_GUIDE | selected=1/2 | DIAGNOSTIC_LEXICOGRAPHIC_BEST
samples=56 | map_collisions=28 | union covered=35 gap=93 unknown=40 | status=COLLISION_OBSERVED
```

- 该 HUD 的 `samples=56 / map_collisions=28 / union covered=35 gap=93 unknown=40` 与同一 round 的 `PORTAL_GUIDE` complete progress 文档、quality report round 1 assessment 逐项一致。
- `/formation/guides` 诊断层真实出现：`formation/planning/map_collision_sample` 10 个、`formation/planning/coverage_gap` 6 个、`formation/planning/portal_window` 3 个；三类文本均不含 `CERTIFIED`（`guide_diagnostics_no_certified=true`，原始文本见 `capture_summary.json`）。
- 候选 marker 标签为 `DIAGNOSTIC_ONLY` / `UNVERIFIED_CANDIDATE`，符合失败请求的“候选风险/unknown”要求。

### 8.2 成功请求的 certified 链

map05 成功请求中，亮蓝 certified 路径仍只由原有 geometry/dynamics/collision 三段独立审核和原子 playback envelope 产生：`executable_envelope_count=1`，`virtual_result.executable=true`，`certification="virtual_a_geometry_dynamics_collision/v1"`；HUD 同时显示 `AUDIT geometry=PASS dynamics=PASS collision=PASS`。新增诊断层没有任何 `CERTIFIED` 文本、不能修改 executable flag、也不能直接发布计划。

### 8.3 截图

`candidate/step_f/map05_default/images/`：`00_idle.png`、`01_reset.png`、`02_terminal.png`、`03_terminal_zoom.png`（6× 放大 HUD）及 `live_01…live_11`（0.8–34 s）。
`candidate/step_f/s1_history/images/`：同类 15 张。截图同时包含 RViz 图层、A*（橙）/active Guide（绿）/certified（亮蓝）路径、候选 OBB 与标签、HUD；`03_terminal_zoom.png` 可读 HUD 的 route/Guide/样本计数。

### 8.4 Step F 现场发现并修正的队列问题（重要）

真实 ROS 运行发现 progress 文档在“阶段 complete 紧跟下一阶段 running”的突发发送中被丢弃，导致 HUD 在 `PUBLISHED` 仍显示 `guide=unknown`。逐层定位并修正两处本地 ROS 队列参数（不改变任何 admission/求解/审核/发布门控）：

| 位置 | 原值 | 新值 | 证据 |
|---|---|---|---|
| planner `/formation/planning_debug` publisher | `queue_size=1` | `queue_size=20` | 修正前 `map05_default_run1_pre_queue_fix`：驱动只收到 13 条文档（缺 ASTAR/CORRIDOR/PORTAL_GUIDE complete），HUD `guide=unknown`。 |
| playback `/formation/planning_debug` subscriber | `queue_size=1` | `queue_size=50` | 修正前 `map05_default_run2_planner_queue_only`：驱动已收到 17 条（含 complete），但 playback 日志仍只有 14 次 stage 记录，HUD 仍 `guide=unknown`。 |

对应最终源码位置：`formation_uniform_planner_node.py:525`、`fwis_v3_playback/scripts/formation_playback.py:321-322`。

修正后 `map05_default`：playback 日志 16 次 stage 记录，HUD/S1 均显示完整诊断。该问题是 TASK01 既有发送行为在真实运行中的暴露（TASK01 日志同样只有不完整事件），不是 TASK02 规划语义变化；修正仅扩大有界队列，不改变消息内容或顺序。

## 9. 未运行、未完成与已知限制

- `test_continuous_retiming` 的既有基线失败未复跑、未修复（见 7.6）。
- RViz 图层的最终人工目视验收仍由用户完成；本任务只提供自动截图与原始消息。
- map05 成功场景的 `/formation/guides` 诊断层没有实际 marker，因为该 assessment 的碰撞/gap/window 计数均为 0（HUD 显示 `0`）；诊断 marker 的真实存在与上限由 S1 实跑（10/6/3）和单元测试证明。这是“零风险不伪造 marker”，不是图层缺失。
- 驱动脚本的 `candidate_markers_no_certified` 检查在 map05 为 `false`：`/formation/planning_candidates` 的 F/三车候选标签是 TASK01 已接受的 `CERTIFIED`（成功请求的真实认证标签），不是新增诊断层。S1 同一检查为 `true`，标签为 `DIAGNOSTIC_ONLY` / `UNVERIFIED_CANDIDATE`。本任务新增的三个诊断 namespace 在两次运行中均无 `CERTIFIED`。
- `planner_params_final` 中 `intervals`/`solver` 等为 `null`：这些不是 ROS parameter server 参数，而是节点内已验证的 planner config（`launch.log` 打印完整值）；`backend`/`virtual_a_guidance_mode`/`virtual_a_corridor_enforcement`/`virtual_a_goal_policy` 实际非空，完整参数树见 `params_launch.yaml`。
- S1 只进行了 2 轮 corridor 尝试（route 1、route 0）：hard `CORRIDOR_AUDIT_WIDTH_LIMIT` 是当前管线的终止失败，不能继续试第三条 route；这与历史结论一致，未隐藏。
- 未测试其他地图、未做长时间 soak、未覆盖所有失败注入组合；未修改 `run_demo.sh`/launch 默认路由。

## 10. 证据目录与可复算性

根目录：`work_flow/TASK02_evidence/20261119T054527Z/`

| 路径 | 内容 |
|---|---|
| `baseline/` | Step A 基线：HEAD、TASK01 dirty diff/哈希、未跟踪文件清单、默认入口 dry-run、地图 SHA、14 个基线测试日志 |
| `geometry/` | Step B：`stepB_geometry_results.json`（O1/U1/U2）、corridor 回归日志、diff/哈希 |
| `candidate/step_c/`、`candidate/step_d/` | 三车/采样/窗口评估、route 顺序/候选选择、solver spy 测试 |
| `candidate/step_e/` | display/HUD/RViz 实现、194 tests、dry-run/dump、progress/HUD 真实样本；`source_hashes.txt`/`step_e.diff` 已按最终工作树刷新 |
| `candidate/step_f/` | 离线回归日志（17 模块 + 构建/静态检查）、map05 与 S1 真实 ROS/RViz 运行、pre-fix/run2 对照、截图、原始 `planning_debug`/quality/path/marker/HUD 消息 |
| `source.diff` | TASK02 自身 diff（9 个已跟踪文件 + 新增测试；TASK01 dirty 不在其中） |
| `source_hashes.txt` | 最终源码/测试/RViz 哈希 + TASK01 dirty 对照哈希 |
| `git_state.txt` | 最终 `git status --short --branch`、HEAD、diff stat、未跟踪文件数 |
| `manifest.sha256` | 覆盖本交付文档、全部日志/结果/截图/源码哈希文件；可用 `sha256sum -c` 独立复算 |

复算命令（仓库根目录；manifest 内路径均相对仓库根目录）：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
LC_ALL=C sha256sum -c work_flow/TASK02_evidence/20261119T054527Z/manifest.sha256
```

本次交付前的独立复算结果保存为 `work_flow/TASK02_evidence/20261119T054527Z/manifest_check.log`（历史日志，已不再作为最终结论）。返工后的最终独立复算保存为 `rework/manifest_check_final.log`（原始输出与退出码，见 §12.4）；为提高可复算性，最终 manifest 不列出 `manifest.sha256` 自身，也不列出该最终校验日志，其余 414 个实际文件全部覆盖，`LC_ALL=C sha256sum -c` 实际退出码 0、414/414 OK。

## 11. 未解决问题与风险

- progress 文档的传输队列现在是 planner 20 / playback 50。若未来有更多并发订阅者或更大文档，仍可能出现丢弃；但最新文档 latched，HUD/图层会以收到的最新完整文档为准。没有引入重传协议或持久化日志。
- 诊断只代表有限静态采样（≤64/≤32 stations、候选 ≤5），不是连续认证；`continuously_certified=false` 在所有输出中固定。
- S1 的失败原因与历史一致，但 TASK02 新增的多 route 尝试使终态耗时从 TASK01 的单轮约 10.7 s 变为约 50 s（2 轮 × 求解/审计）。这是新增候选诊断的既有行为，不是阈值放松。
- 未对 `test_continuous_retiming` 的基线失败做归因或修复。

## 12. 2026-11-19 返工（依据 `work_flow/TASK02_返工执行方案.md`）

本节记录返工 A/B/C 的最终实现、证据与复核结果；与本节冲突的旧 HUD 标签/旧 manifest 结论以本节为准，历史原始消息仍保留不改写。返工只改了允许范围内的 `corridor.py`、`pipeline.py`、`formation_playback.py`、4 个测试文件和本交付文档；未改 `geometry.py`、`formation_collision.py`、`optimizer.py` 约束/目标/域、launch、`run_demo.sh`、历史 evidence 或模型 fixture。

### 12.1 返工 A：基线驱动的目标 Portal 局部候选

实现（`formation_continuous_a/corridor.py`）：

- 先对 QP base Guide 做一次真实三车 baseline assessment；该结果同时用于选择目标 Portal 和候选排序，不重复调用 checker。
- `select_guide_target_portal()` 先从 baseline 的 `vehicle_map_collisions`、transition window 的 `collisions`/`PORTAL_TRANSITION_GAP`/`PORTAL_TRANSITION_UNCONFIRMED`、vehicle `gap`、coverage gap witness 中收集风险 Portal；按 `MAP_COLLISION < UNION_GAP < TRANSITION_UNKNOWN`、再按 Portal index 升序选最早风险 Portal。无风险时选 `_portal_bundle` Chebyshev 半径最小者，半径相同按 Portal index 升序。
- 目标 Portal 凸交内用 `z' = 0.5 z + 0.5 v`（`z` 为 Chebyshev 中心，`v` 按固定顺序遍历交点）构造 ≤4 个确定性内点，去重后只评估该 Portal 局部 transition stations（≤16），局部排序后保留 ≤3 个完整 Guide；每个候选只改目标 Portal 的 1 个点，start/goal 不动，并重新执行 `allocate_corridors()`。总候选 ≤5，全部为 `DIAGNOSTIC_ONLY`。
- 局部评估完整才记 `ASSESSED`；超预算/采样上限/不完整记 `UNKNOWN` 及明确原因（`SOFT_BUDGET`/`LOCAL_SAMPLE_CAP`/`LOCAL_ASSESSMENT_INCOMPLETE`）；任何不兼容候选记录原因并保留 QP fallback，不放松 F-only halfspace。

结果证据（`rework/selection/selection_summary.json`，由 `rework/selection/generate_selection_evidence.py` 可复算）：

- 后置 Portal 风险 fixture（4 个走廊，window 0/1 正常、window 2 为 `PORTAL_TRANSITION_GAP`）：风险 Portal 仅 `[3]`，`selection=BASELINE_RISK`、`target_portal_index=3`、`target_reason=UNION_GAP`；生成 5 个候选 `[base, CHEBYSHEV_CENTRE_GUIDE, ONE_PORTAL_VARIANT_2 ×3]`，3 个局部候选的 `changed_indices` 均为 `[3]`，start/goal/前一个 Portal 点全部不变；目标点对相邻两走廊最大违反分别为 `-0.75 m`、`-0.25 m`（负值表示在内部），5 个 Guide 的 `allocate_corridors()` 均非空（各 11 段）。
- 无风险时：`later_left=11.5` 的半径 `[0.5, 0.25]`，选中 Portal 2；`later_left=11.0` 半径并列 `[0.5, 0.5]`，按索引选中 Portal 1。
- 真实 `FormationCollisionChecker.static_check()` 改善 fixture：base 12 次 map collision，候选碰撞数 `[12, 13, 0, 0, 0]`，按 pipeline 同一字典序选中 index 2（`ONE_PORTAL_VARIANT_0`，0 collision、0 gap），`selected_guide_points=[[0,0],[5.5,0.5],[10,0]]`；candidate record 的 `candidate_id`、assessment `selected_guide_source`、`legacy_f_allocation` 与候选/重新分配同源，checker 实际调用 384 次。
- 测试 `tests/test_virtual_a_formation_assessment.py::GuideCandidateGenerationTests` 8 项覆盖：后置风险、最早风险优先、无风险最小半径/并列索引、候选点位于相邻走廊交、只改一个 Portal、每个候选 F-only 分配、真实 checker 改善与 solver spy 同源、候选上限/60 s 预算耗尽保留已算结果且其余 UNKNOWN、无 baseline 时只生成 canonical guides。`test_virtual_a_pipeline.py` 的 solver spy 进一步断言所有 solver 调用使用同一 planner config、未改变 `max_initials/max_iterations/max_seconds/guidance_mode` 与 audit 路径。

### 12.2 返工 B：`SINGLE_REGION` 与 `COVERED_BY_ADJACENT_UNION` 分列

实现（`corridor_portal_diagnostics()`、`pipeline._progress_assessment_summary()`、`formation_playback.planning_diagnostics_view()`/`_hud_diagnostics_line()`）：

- `single_region_total` 只累加 `counts.single`；`union_covered_total` 只累加 `counts.union`，不再包含单区覆盖；`union_gap_total`/`union_unknown_total` 仍对应 gap/unconfirmed。
- progress、HUD、marker diagnostic 共享同一字段语义；缺失/非法/旧 payload 仍为 `unknown`，不补默认数字。
- HUD 紧凑标签固定为 `single=… | union=… | gap=… | unknown=…`；不再出现误导性的 `union covered` 总数字。
- 未改规划候选评分、安全决策或 executable/publish 条件。

真实结果：

```text
map05: samples=13 | map_collisions=0 | single=39 | union=0 | gap=0 | unknown=0 | status=CLEAR_AT_SAMPLES
S1:    samples=61 | map_collisions=25 | single=35 | union=0 | gap=108 | unknown=40 | status=COLLISION_OBSERVED
```

map05 的 39 与 3 车 × 13 个 `SINGLE_REGION` 完全一致，union 实际为 0；S1 HUD 的四项与其 quality report round 1 的 `coverage_vehicles` 汇总（single 35、union 0、gap 108、unconfirmed 40）逐项一致。相关回归覆盖：仅单区样本、仅相邻并集样本、单区/并集/gap/unknown 混合、旧/非法 progress fail-closed、HUD 与 progress 一致；原先把 `single + union` 当作 `union_covered_total` 的断言已改为分列。两次 live run 的原始 HUD 与 quality 文档在 `rework/live/{map05_default,s1_default}/`。

### 12.3 返工后默认入口重跑（无隐藏 backend/guidance 参数）

命令与环境、reset/goal、原始日志和截图均在 `rework/live/`：

- `./run_demo.sh --map map05 --scale 2.625`：driver exit 0；`virtual_result.status=SUCCESS`，终态 `HOLD_END`，executable envelope 1；三段 audit PASS；HUD 为上方 map05 行。地图 runtime grid SHA 与 TASK01 map05 基线一致。
- `./run_demo.sh --map evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S1 --scale 1.0`：driver exit 0；真实终因仍为 `GEOMETRY_FAILED / CORRIDOR_AUDIT_WIDTH_LIMIT`（`first_failure_layer=GEOMETRY`，route_count=3，2 轮后 hard containment 终止），executable envelope 0；未降低阈值、未改写失败结论。该 run 的 baseline 风险 Portal 为 index 1（`MAP_COLLISION + UNION_GAP`），候选碰撞数 `[36,28,25,28,36]`，选中 index 2（`ONE_PORTAL_VARIANT_0`，source `corridor_candidate_portal_0`），HUD 为上方 S1 行。
- 两次 run 均使用任务私有 master `127.0.0.1:11633`；结束后只按本任务记录的进程组停止 run_demo/roslaunch/节点，再停 master。`ss` 确认 11633 关闭；另一个会话的 11411（pid 3039917/3039925/4042262）始终未被触碰并保持运行。
- 截图：两个 run 的 `images/00_idle.png`、`01_reset.png`、`02_terminal.png`、`03_terminal_zoom.png` 及 `live_*.png`。

### 12.4 返工 C：证据冻结与独立复算

- 生成最终证据前确认本任务无 ROS/RViz/驱动进程继续写入（`rework/live/*/stop.log`、`rework/live/master_11633/stop.log`；无 11633 进程/端口、无本任务 map-scale 进程）。
- 重新生成 `source.diff`（TASK02 的 9 个已跟踪文件 + 新增测试；TASK01 dirty 不在其中）、`source_hashes.txt`（最终源码/测试/RViz 哈希 + TASK01 对照）、`git_state.txt`（最终 status/HEAD/diff stat/未跟踪数）。
- 最终 manifest 由仓库根目录下实际文件清单生成，**不包含** `manifest.sha256` 自身和校验后写入的 `rework/manifest_check_final.log`；其余实际文件全部覆盖，共 **414 项**。
- 独立复算命令（仓库根目录）：

```bash
LC_ALL=C sha256sum -c work_flow/TASK02_evidence/20261119T054527Z/manifest.sha256
```

- 原始输出与退出码保存为 `work_flow/TASK02_evidence/20261119T054527Z/rework/manifest_check_final.log`：实际 **414/414 OK，exit 0**，无 FAILED。旧 `manifest_check.log` 是返工前历史日志，不作为最终结论。
- 返工后测试/构建实际退出码：`test_virtual_a_formation_assessment` 40 tests exit 0；`test_virtual_a_pipeline` 13 tests exit 0；`test_virtual_a_corridor_display + test_formation_hud` 92 tests exit 0；`test_virtual_a_corridor + test_virtual_a_collision` 42 tests exit 0；`catkin_make` exit 0；`git diff --check` exit 0；`bash -n run_demo.sh` exit 0。原始日志与 `.exit` 文件在 `rework/tests/`。
- 最终 TASK01 dirty 哈希仍为：`geometry.py f31b20bb…`、`tests/test_virtual_a_geometry.py 4f59e68f…`，与基线逐字节一致。
- 最终 TASK02 diff 统计：9 个已跟踪文件 `4768 insertions(+), 19 deletions(-)`；新增 `tests/test_virtual_a_formation_assessment.py` 1103 行；整棵工作树（含 TASK01 dirty）11 文件 `4860 insertions(+), 20 deletions(-)`，未跟踪条目 26 个（含本任务证据目录）。
- 本轮返工新增的所有证据均在 `work_flow/TASK02_evidence/20261119T054527Z/rework/`；未新增/修改 TASK01 evidence、`evaluation/` 或模型 fixture。

## 13. 停止声明

TASK02 的 A–F 内部步骤已完成并按上文自测；本任务到此停止。

- 未 commit、未 push、未 reset/clean 工作区。
- 未开始 TASK03。
- 不声明 Codex PASS，不声明 TASK01/TASK02 验收通过；等待 Codex 一次性独立验收，用户另行做最终 RViz 目视验收。