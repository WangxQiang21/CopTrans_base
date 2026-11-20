# TASK02 定向审核报告：三车候选碰撞与走廊失败复盘

日期：2026-11-19。仓库：`/home/wq/whut_thesis/expv3_multi_virtual_a`。
审查依据：用户提供的 `/home/wq/下载/TASK02_定向审核方案_交Codex.md`，以及当前 TASK02 方案、Claude 交付和冻结证据。

## 结论

**总分类：`INSUFFICIENT_EVIDENCE`（针对用户报告的 plan 600001、600002）。**

我没有把同号的历史请求拼成这两个新失败。缺少这两次运行的冻结地图/比例、请求 SHA、grid SHA、planning context SHA、有效 launch 参数、对应 `quality_report` 和 `planning_debug`，无法独立重建两条候选选择链或归因其 `CORRIDOR_AUDIT_WIDTH_LIMIT`。计划号不是场景身份。

在当前源码、独立测试和另一份已冻结 S1 运行中，**未确认 TASK02 候选生成、评分或 solver 传递存在实现缺陷**。该 S1 运行确实显示 TASK02 只提供有限候选诊断：选中的 Guide 仍有采样风险，后续连续 F 点认证/三车 OBB 终审会拒绝不可行解。这证明了该输入上的能力边界，不足以替代用户这两个场景的复现。未生成返修清单；当前结论也不能视为这两个 plan 已验收通过。

## 1. 审查身份与复现输入

| 项目 | 审查事实 |
|---|---|
| HEAD | `a3c709e57560b1060dcc422777f4131a3e558acf`，等于 `origin/feature/virtual-rigid-formation-a` |
| 工作区 dirty | 仅 TASK01 的 `geometry.py` 与 `tests/test_virtual_a_geometry.py`；SHA 与 TASK02 基线记录相同（见证据清单），未修改 |
| 活跃 ROS/RViz | 审查时未发现 `roscore/rosmaster/roslaunch/rviz` 进程；未启动新 master |
| 目标 plan 600001/600002 | 按审核方案要求的 exact request/map/context identity 均不可得；见 [`repro_manifest.json`](TASK02_evidence/codex_targeted_review/repro_manifest.json) |
| 最近的 TASK02 现场输入 | S1 plan 600000：request `591b4100…`、grid `6e94c751…`、context `8a545c3f…`；它与用户描述的计划号和 HUD 结果不同 |

同号历史记录彼此不一致：旧 S1 `plan 600001` 是 `NO_ROUTE_GUIDE`；旧 map05 U-turn `plan 600001` 是 `ENVELOPE_INVALID`；旧 S1 `plan 600002` 是 reset 后的成功请求。它们都不是本次报告中“两个 plan 均为 `CORRIDOR_AUDIT_WIDTH_LIMIT`”的输入。

`run_demo.sh:404-441` 在默认 `virtual_rigid_a` 路径补齐 `guidance=corridor`、`enforcement=hard`、`auto_goal=false`；launch 对应参数见 `formation_demo.launch:24,79-83`。这是源码合同。目标会话的**实际生效参数**仍未知。没有在猜测的地图上重跑 ROS/RViz。

## 2. A–E 定向检查

| 项 | 结论 | 依据与边界 |
|---|---|---|
| A 输入、版本、入口 | **INCONCLUSIVE（目标 plan）/ PASS（源码入口）** | HEAD、dirty 哈希及默认入口已核实；目标请求身份不存在，不能声称复现。 |
| B 三车几何、地图与覆盖诊断 | **PASS（实现/单测）；目标现场 NOT_RUN** | `corridor.py:545-640` 将完整 OBB 多边形对相邻凸区做双顺序裁剪；只在两种顺序均 COVERED 时报告 union 覆盖，均 GAP 才报告 gap，否则 UNKNOWN。`_evaluate_station()` 使用实际车型尺寸、`map_spatial_state()` 生成车体朝向并调用既有 `FormationCollisionChecker.static_check()`（`corridor.py:1315-1378`、`formation_collision.py:408`）。U2 diamond、单区/并集、map collision 与 coverage gap 分离测试通过。 |
| C 路线/Portal/Guide 与 solver identity | **PASS（源码/测试/历史 S1）；目标现场 INCONCLUSIVE** | route 风险顺序见 `pipeline.py:795-843`；Portal 目标及局部变体见 `corridor.py:3166-3300,3486-3628`；候选字典序评分见 `pipeline.py:846-906`；选中候选 state（Guide、allocation、assessment）在 `pipeline.py:1840-1895` 取自同一 `selected_index`，同一 chain/allocation/Guide 传入 `solve_open_path_guided()`，见 `2041-2051`。测试 `test_route_attempt_order_and_guide_selection_reach_the_solver` 通过；目标场景的真实分数、Portal 淘汰理由和 solver record 缺失。 |
| D `CORRIDOR_AUDIT_WIDTH_LIMIT` 根因 | **目标现场 INCONCLUSIVE；不同 S1 输入有可核实结果** | `optimizer.py:_build_corridor_constraint()` / `_build_casadi_nlp()` 仅将分配给参考点 F 的 corridor 半空间作为 NLP 约束；不包含三车完整 OBB 硬约束。`corridor.py:audit_path_in_corridors()` 对最终 C² 曲线逐区认证 F 点连续包含；`CORRIDOR_AUDIT_WIDTH_LIMIT` 表示区间上界在最小宽度处仍未闭合，单看该标签不等于车体碰撞。Task02 提交未改这段审核函数（与父提交函数体 SHA 相同）。 |
| E 安全链/旧实现保护 | **PASS（源码/提交差异）** | TASK02 提交没有改 `optimizer.py`、`collision.py` 或 `audit_path_in_corridors()` 函数体；管线仍按几何→动态→完整三车 OBB 审核。ROS 节点在 `result.executable` 为假时走失败路径并发失败占位信封；只有重新校验原子 envelope 后才发布并发 `PUBLISHED`，见 `formation_uniform_planner_node.py:1828-1867,1920-2002`。 |

### B 项的计数与 marker 解释

`_assessment_map_summary()` 的 `sampled`/`map_collision`/`vehicle_collision` 是**采样站点数**，每个站点由一次 `static_check()` 得到一个状态；HUD `map_collisions` 是 `map_collision + vehicle_collision` 的站点计数，不是碰撞车辆数或接触点数，详细类型需看 `map_sample_counts`/`vehicle_map_collisions`（`corridor.py:1530-1560,1710-1761`；HUD 取值见 `pipeline.py:743-779`）。显示的 `MAP_OBB_COLLISION (sample)` 是 Guide 评估站的状态标注，不是求出的墙面接触位置。`BOTH_ORDERS_GAP` 表示 OBB 多边形未被两相邻 corridor 的并集覆盖；这是人工走廊 coverage 缺口，**不是地图障碍碰撞**。

### HUD 字段勘误

`formation_playback.py:2113-2130` 直接显示 `selected_candidate_index`，索引是 **0-based**。所以 `selected=3/5` 表示质量报告中的 `selected_candidate_index=3`（5 条候选的第 4 条），不是自动表示第 3 条。`SINGLE_BASE_GUIDE` 在 `pipeline.py:1852-1853` 由 `len(candidate_records)==1` 得出，**不代表 `region_count==1` 或没有 Portal**。必须使用同请求的 quality/planning 文档确认候选身份与候选总数。

## 3. 可核验的最近 S1 对照（不是目标 plan）

证据：`TASK02_evidence/20261119T054527Z/rework/live/s1_default/quality_report.json`、`planning_debug.jsonl`、`hud_texts.json`。冻结地图 grid SHA `6e94c751…`；plan `600000`；request SHA `591b4100…`。

路线评估均为 12 个 sampled stations，其中各有 8 个碰撞站点、4 个 clear 站点；状态全为 `COLLISION_OBSERVED`。route 长度分别为 route 0 `9.744793 m`、route 1 `9.672027 m`、route 2 `9.864056 m`，故有界排序为 `[1,0,2]`。受一次 recovery 限制，实际尝试 route 1 后再试 route 0，不代表 route 2 已证明不可行。

| round / route | candidate index / origin | map/vehicle collision stations | coverage gap | coverage unknown | 被选中 |
|---|---|---:|---:|---:|---|
| 0 / route 1 | 0 base QP | 42 | 97 | 45 | 否 |
| 0 / route 1 | 1 Chebyshev | 25 | 104 | 44 | 是 |
| 0 / route 1 | 2 variant 0 | 31 | 110 | 39 | 否 |
| 0 / route 1 | 3 variant 0 | 31 | 107 | 36 | 否 |
| 0 / route 1 | 4 variant 0 | 30 | 101 | 38 | 否 |
| 1 / route 0 | 0 base QP | 36 | 101 | 48 | 否 |
| 1 / route 0 | 1 Chebyshev | 28 | 93 | 40 | 否 |
| 1 / route 0 | 2 variant 0 | 25 | 108 | 40 | 是 |
| 1 / route 0 | 3 variant 0 | 28 | 107 | 39 | 否 |
| 1 / route 0 | 4 variant 0 | 36 | 119 | 40 | 否 |

这是诊断字典序的结果：round 1 中 variant 0 的碰撞站点最少（25），虽 gap 数高于 Chebyshev 仍先按 collision 字段胜出。**选中并不等于无碰撞/覆盖通过。** quality report 中 round 0/1 的 `corridor_round_index` 分别为 0/1，`route_index` 与 corridor record 一致；源码 solver spy 也断言选中 Guide、对应 allocation 与 chain 同源。该输入上可复核候选进 solver，不可外推到用户截图。

该 S1 的两轮后续审核还说明采样诊断与最终认证是不同层级：

- round 0 / route 1：几何和时域审核通过，连续 F containment 通过（`corridor_containment_passed=true`）；最终完整 OBB sweep 报 `MAP_OBB_COLLISION`，vehicle 2，station `2.654537 m`、time `10.106617 s`。这是实际最终审查报告，不是 Guide sample marker。
- round 1 / route 0：求解器几何候选返回，但连续 F containment 失败；首次失败记录 `s=7.757789733 m`、corridor `0`、width `9.682635e-6 m`、depth `17`、point residual `-4.691765e-6 m`、bound residual `+2.059593e-7 m`。该失败发生在最小 width `1e-5 m`，因此没有继续 retime/dynamics/OBB 审核，也没有可执行发布。
- 同一 containment 汇总的 `max_point_residual_m=+2.850885e-4 m`，大于零，证明审核访问过一个 F 点违反其分配半空间的中点；但质量报告没有该最大值对应的 `s_m`。因此不能把这份 S1 的 `WIDTH_LIMIT` 简化为“路径肯定在区内、只是区间保守”，也不能把它说成车体撞墙。这个定位缺口在 TASK02 前已有的审核函数中；TASK02 没有修改该函数，建议在后续独立诊断中补上 max-residual 位置或明确真实点越界原因，**本次不列为 TASK02 返修项**。

## 4. 安全与发布状态

`formation_obb_hard_constraints=false` 是当前 corridor optimizer 的真实合同：IPOPT 对 F 点施加半空间约束，Guide 评估的 OBB/coverage 全部是诊断，不进入三车 OBB NLP 硬约束。之后的 `audit_path_in_corridors()` 是 F 连续 containment；成功后才进入 retime/geometry、dynamic 与 `audit_collision()` 全三车 OBB sweep（`pipeline.py:2378-2499`）。其中任何失败都会返回非 executable 结果。ROS planner 对失败调用 `_finish_virtual_failure()`，写 rejected status 和不可执行占位信封；不会播放旧的轨迹；发布成功 envelope 前还会做独立 verifier 检查。

TASK02 中 Guide 的 map collision、coverage GAP、最终 F containment、最终三车 OBB audit 保持分离，没有用一种状态冒充另一种，也没有发现降阈值/关闭审核。历史 S1 说明当前任务可筛选较好 Guide，但还不保证 NLP 找到满足连续 F containment、随后通过真实 OBB sweep 的曲线；这属于下一算法阶段需处理的规划能力边界。

## 5. 独立测试与证据

- 40 + 13 + 52 + 92 + 17 项定向测试全部通过，共 214 项；具体命令、退出码、耗时见 [`test_results.log`](TASK02_evidence/codex_targeted_review/test_results.log)。其中包含完整 OBB union diamond 反例、map/coverage 状态分离、Portal 选择、solver spy、认证失败无 executable、碰撞边界测试。
- `bash -n run_demo.sh`、`git diff --check` 均 exit 0。
- 从仓库根目录重算 TASK02 证据 manifest：414/414 项通过，exit 0。
- 本轮没有运行 `catkin_make`，也没有开 ROS/RViz：没有目标请求可合法重放；TASK02 历史证据已有构建结果。原始源码 SHA 及两个 TASK01 dirty 文件 SHA 见 `TASK02_evidence/codex_targeted_review/source_hashes.txt`。

## 6. 下一步与返修判断

**不创建 `TASK02_Codex返修清单.md`**：目前没有独立证据证明这两个目标运行发生了 TASK02 实现错误；擅自改候选评分、提高预算或放宽审计都没有依据。也不以本报告直接授权 TASK03 实现。

要完成目标归因，最小补充是分别导出 plan 600001、600002 的：

1. `plan_request` 与 job/admission 文档（含 request/context/grid/model SHA、formation offsets、shape、start/goal/headings、goal policy）；
2. 同一运行的 quality report 与完整 `planning_debug`（至少包含每个 route assessment、所有 guide candidates/selection、chain digest、allocation、solver round、containment first failure 和 max residual）；
3. 地图原始文件与 scale/runtime grid SHA、实际 `run_demo.sh` launch 行和 ROS master；
4. 若 max point residual 为正，保存其 `s_m` 与所在 corridor 的 residual；若是 width limit，保留 `first_failure` 全字段。

之后按 request/context/model/grid identity 对两个计划分别复核。报告原始测试与复现清单分别见 `TASK02_evidence/codex_targeted_review/`。本次结论仅为目标截图证据不足，不把它等同于 TASK02 算法缺陷。
