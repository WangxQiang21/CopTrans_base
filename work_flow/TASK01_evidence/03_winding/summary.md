# Step 3 候选级 winding 证据（有限分支、单 packet、单 recovery）

- 日期：2026-11-19
- BASE HEAD：`77805cd46849ebadc374187f9b00fce0f7151f84`（未提交、未 push）
- 工作目录：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- Python：3.8.10；ROS Noetic；CasADi 3.7.2
- 状态：Claude 自测完成；不声明 Codex PASS，不声明 TASK 01 验收通过

## 1. 运行环境与命令

执行方案 §5.1 的字面 `export PYTHONPATH=.../src` 会覆盖 ROS/helper 路径；采用增量前置：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

命令清单见 `commands.txt`。本轮是全部 Step 3 改动（含两处严格性修正）完成后的
**权威重跑**：每组 stdout/stderr、退出码、墙钟分别保存在 `g*.log` /
`g*.exit_code` / `g*.duration`。`py_compile`（contracts/optimizer/pipeline、三个
测试文件、探针）与 `git --no-pager diff --check` 均通过。

## 2. §5.1 权威结果

| 组 | 命令（测试模块） | 用例数 | 日志内时长 | 墙钟 | 退出码 | 日志 |
|---|---|---:|---:|---:|---:|---|
| G1 | `tests.test_virtual_a_corridor_display tests.test_formation_hud` | 67 | 19.501 s | 20.18 s | 0 | `g1_corridor_display_hud.log` |
| G2 | `tests.test_virtual_a_receiver tests.test_virtual_a_playback` | 49 | 62.398 s | 63.05 s | 0 | `g2_receiver_playback.log` |
| G3 | `tests.test_formation_demo_assets` | 28 | 3.827 s | 3.89 s | 0 | `g3_demo_assets.log` |
| G4 | `tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer` | 58 | 78.841 s | 79.24 s | 0 | `g4_corridor_optimizer_optimizer.log` |
| G5 | `tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery` | 37 | 126.904 s | 127.32 s | 0 | `g5_pipeline_recovery.log` |
| G6 | `tests.test_virtual_a_geometry tests.test_virtual_a_profile` | 42 | 2.440 s | 2.74 s | 0 | `g6_geometry_profile.log` |
| G7 | `tests.test_virtual_a_integration_bounds tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit tests.test_virtual_a_trajectory` | 97 | 6.731 s | 7.03 s | 0 | `g7_integration_audit_trajectory.log` |
| G8（补充） | `tests.test_virtual_a_ros_adapter` | 63 | 15.850 s | 16.46 s | 0 | `g8_ros_adapter_supplemental.log` |

七组共 378 例；含 G8 共 441 例，全部 `OK`。Step 2 的七组为 364 例，本步新增
14 例（G4 +3、G5 +11），无既有用例被删除或弱化。

## 3. 实际修改

| 文件 | 修改 |
|---|---|
| `formation_continuous_a/contracts.py` | 新增不可变 `HeadingBranch`（`base_heading_rad, winding_k, unwrapped_heading_rad, guide_turn_rad, source, candidate_rank`，`as_dict()` 恰好六字段）；`GeometryResult.goal_heading_branch_rad` 记录 unwrapped 值；`AttemptRecord` 增加 `heading_base_rad/winding_k/guide_turn_rad` 并只在非 `None` 时序列化；`__all__` 导出。 |
| `formation_continuous_a/optimizer.py` | `_search_packet(request, config, heading_branch=None, guide_length_m=None)` 成为 heading/S 域唯一来源：有 branch 时用 `unwrapped_heading_rad`，无 branch 时逐字节保留历史 `wrap(psig-psi0)`；packet 记录 branch 与 guide length。`solve_open_path_guided()` 每次求解只构造一次 packet，原样传给 `make_initial_guesses()`、route/endpoint guess、numeric 与 CasADi endpoint 等式；新增 `INVALID_HEADING_BRANCH` / `INVALID_GUIDE_LENGTH` fail-closed。`initial_guess_key()` 在 guess 带 branch 时追加 `winding_k`（同几何初值在不同 unwrapped heading 上是不同发射身份）。`_attempt_record()` 从同一 guess metadata 记录 base/k/guide turn。 |
| `formation_continuous_a/pipeline.py` | `WINDING_GUIDE_SEGMENT_EPS_M=1e-6`、`WINDING_NEIGHBOUR_BRANCHES=1`、`WINDING_RANK_TIE_DIGITS=12`；`_guide_segment_tangents()` 删除连续重复/近零段；`_heading_branch_candidates()` 以 `psi0`+有效段 `atan2`+`psig` 累加 wrapped turn 得 `Theta_G`，`base=wrap(psig-psi0)`，`k0=round((Theta_G-base)/2π)`，有限集合 `{k0-1,k0,k0+1}` 按 `|base+2πk-Theta_G|` 排序；距离先按 1e-12 rad 归并，使数学并列严格走“较小 `|k|`、再较小 `k`”，避免浮点 ulps 改变并列次序。无可靠 guide 只给 `k=0`。每轮按 `(route_index, corridor_mode)` 建立当前 route/guide 的分支集，默认只用首选；`search["winding"]`、round record、NLP_SOLVE 进度与 attempt 均记录 base/k/turn/rank/guide length。`_winding_retry_allowed()` 只接受 `SOLVER_NOT_CONVERGED` / `FIXED_ALLOCATION_LIMIT`，或 `AUDIT_REJECTED` 且**最终决定该 reason 的最后一次** audit 拒绝为 `ENDPOINT_POSITION_ERROR`/`ENDPOINT_HEADING_ERROR`；碰撞、动力学、数值认证（如 `CERTIFICATE_ERROR`）、走廊包含失败永不触发。winding recovery 只前进一个候选，计入现有 `recovery_count` 与 request-wide initial pool，并受 `can_recover()`（soft budget >1 s、未用 initial、`MAX_RECOVERIES=1`）约束。 |
| `tests/test_virtual_a_corridor_recovery.py` | 新增 `HeadingBranchCandidateTests`（7 例）：CCW/CW 环、重复/近零段、600004 型近并列、退化 guide 只 `k=0`、物理 goal 不变、六字段不可变；新增 4 例调度：假 solver 成功但 endpoint 审计拒绝时只消耗一次 winding recovery 且永不认证、solver 非收敛恰好重试一个下一分支（初始数 `[5,1]`、6 个唯一 initial、packet/attempt 均带 branch）、非 endpoint 认证拒绝不猜 winding、动力学失败不进入 winding；另加 `_winding_retry_allowed` 对“最后 concrete rejection”的定向断言。历史 endpoint-only 测试用单分支退化 guide 保持其语义。 |
| `tests/test_virtual_a_optimizer.py` | 新增 `HeadingBranchPacketTests`：一次 `_search_packet` 调用、同一 packet 对象到达全部 solver 调用、`GeometryResult.goal_heading_branch_rad==unwrapped`、attempt metadata 与 packet 一致；route initial 身份随 winding 分支改变。 |
| `tests/test_virtual_a_corridor_optimizer.py` | 新增：带 `k=1` branch 的 packet 同时进入 numeric 与 CasADi endpoint 等式，两侧行值一致（≤1e-12）且等于 `-2π`。 |
| `work_flow/TASK01_evidence/03_winding/probe_winding.py` | 离线证据脚本（不启动 ROS）：复用 Step 0 的 `repro_u_turn` 地图/请求/`solve()` 路径，只读取并导出 `search["winding"]`、round/attempt 的 branch 字段。 |

未修改：`work_flow/TASK01_技术方案.md`、`work_flow/TASK01_Codex执行方案.md`、
`TASK01_evidence/00_baseline/`、`TASK01_evidence/01_visualization/`、三项独立
audit、`MAX_RECOVERIES`、`MAX_INITIALS`、几何/动力学/碰撞阈值与优化器数学。

## 4. 真实 600003/600004 分支证据

探针使用 Step 0 的同一运行地图与冻结请求（`probe_winding.py`，无 ROS）：

- 地图：`/tmp/expv3-task01-map.sax8HJ/map05-p12-parking-row-env2/map.yaml`
- PGM sha256 `0856ad0a…ba718`、YAML sha256 `e4ee76a1…345e0`、
  grid sha256 `55da861b…457`，与 `00_baseline/map_sha256.txt` 一致。
- 600003 request sha256 `bc748779…852b`、600004 `141decf7…e718`，
  与 `00_baseline/baseline_summary.json` 完全一致。
- 配置：corridor / hard / 8 intervals / 6 initials / 30 s（冻结的 Step 0 复现参数）。

| 项 | 600003 | 600004 |
|---|---|---|
| 结果 | `SUCCESS`，可执行 | `BUDGET_EXHAUSTED` / `SOFT_BUDGET_EXHAUSTED` / GEOMETRY |
| guide | `corridor_qp_polyline`，18.625827 m | `corridor_qp_polyline`，45.744425 m |
| `base = wrap(psig-psi0)` | −3.126 rad | +3.124 rad |
| `Theta_G` | −3.126 rad | −3.1591853071795875 rad |
| 有限候选（rank→k） | (0→0), (1→−1), (2→+1) | (0→−1), (1→0), (2→−2) |
| 选中 | k=0，rank 0 | k=−1，rank 0 |
| `unwrapped_heading_rad` | −3.126 | −3.1591853071795866 |
| rounds | 1 × route_guided / GEOMETRY_FEASIBLE | 1 × route_guided / BUDGET_EXHAUSTED |
| initials | 5 构造，1 新发射 | 5 构造，3 新发射 |
| attempts | 1 × `Solve_Succeeded`，41 iter，`SOLVER_SUCCESS_AUDITED` | 3 × `SOLVER_FAILED`（Infeasible 60；Infeasible 107；Max_WallTime 13） |
| recoveries | `[]` | `[]` |
| 路径 | 21.701601208480245 m / 63.64682553904238 s | 无 plan |

要点：

1. **600003 分支与历史结果一致**：首选 `k=0`，路径长与 Step 0 基线
   `21.701601208480245 m` 逐位一致，attempt 仍为 route guide 41 iter。说明
   `_search_packet` 重构与 branch 透传没有改变历史成功路径的语义。
2. **600004 首选分支由公式得到 `k=−1`**：`base−2π = −3.159185…` 与物理目标
   `+3.124 rad` 相差 2π，即同一物理朝向；`candidate_rank` 严格按 unwrapped-turn
   误差排序。若直接按浮点 ulps 比较，`k=−2` 会错排在 `k=0` 前；1e-12 rad 归并
   后顺序为 `[−1, 0, −2]`，符合方案“并列时较小 `|k|`、再较小 `k`”。
3. **600004 未发生 winding recovery**：该轮几何 reason 是
   `SOFT_BUDGET_EXHAUSTED`，不在允许集合；且 request-wide soft budget 已耗尽，
   `can_recover()` 为假。因此即使有 3 个有限候选，也没有越权尝试下一分支。
4. **90 s 补充探针**（`winding_600004_90s.json`）：把 `max_seconds` 提到 90 s
   后，pipeline 的 `G4_TOTAL_SOFT_BUDGET_S=60 s` 仍是总闸；该轮 4 个 attempt
   全部 `k=−1`，仍以 `SOFT_BUDGET_EXHAUSTED` 结束、`recoveries=[]`。它证明的是
   “请求 soft budget 已耗尽”这一门槛，而不是配置文件数字可绕过预算。
5. 600004 的失败仍是 **S 上界不足** 与后续证书问题（当前 `S_upper` 仍未使用
   `1.15*guide_length`；45.744 m 的 guide 对应未来 Step 4 上界约 52.6 m，且历史
   50 m 探针会在 Step 5 证书处 `BOUND_UNCONFIRMED`）。Step 3 只负责选择正确的
   有限 heading 分支，不声称 600004 在本步可执行。

## 5. 定向验收链对应

| 方案要求 | 证据 |
|---|---|
| `HeadingBranch` 六字段、不可变 | `HeadingBranchCandidateTests.test_heading_branch_is_immutable_and_serialises_six_fields` |
| 删除连续重复/近零 guide 段 | `test_duplicate_and_near_zero_segments_are_removed`（1e-9 段被删，仅剩 2 个有效切向） |
| 切向序列含 `psi0`/段 `atan2`/`psig`，累加 `Theta_G` | `_heading_branch_candidates`；CCW/CW 环测试 `±2π` |
| `base`/`k0`/有限集合/排序与并列规则 | CCW `[1,0,2]`、CW `[−1,0,−2]`、近并列 `[−1,0,−2]`；真实 600003/600004 探针 |
| 无 guide/退化只给 `k=0` | `test_degenerate_guide_offers_only_the_shortest_wrap_branch` |
| 不改物理 goal | `test_physical_goal_pose_is_never_rewritten`；request sha 与 Step 0 一致 |
| 一个 packet 同时供初值/route guess/NLP/numeric/CasADi/metadata | `HeadingBranchPacketTests.test_one_packet_reaches_initials_solver_and_attempt_metadata`；`CorridorOptimizerC2BTests.test_branch_heading_reaches_numeric_and_casadi_equalities` |
| route initial 身份含分支 | `test_route_initial_identity_changes_with_the_winding_branch` |
| 默认只试首选；最多下一个分支；计入 `MAX_RECOVERIES`/`MAX_INITIALS` | `test_solver_non_convergence_retries_exactly_one_next_branch`（2 rounds、k `[0,−1]`、initials `[5,1]`、6 唯一 initial、`recovery_count=1`）；真实 600004 首选-only |
| 碰撞/动力学/数值认证失败不猜 winding | `test_non_endpoint_audit_rejection_never_guesses_winding`、`test_dynamics_failure_never_enters_the_winding_retry`、`test_winding_retry_uses_the_concrete_audit_rejection`；OBB/route-switch 历史测试保持 |
| `GeometryResult.goal_heading_branch_rad` 为 unwrapped | `HeadingBranchPacketTests` 断言 `2π`；attempt/search 记录 base/k/turn |
| audit 终点仍用 wrapped 物理角误差 | `audit.py` 未改（`_wrap_abs(endpoint.psi − goal.psi)`）；G7 audit 全套 97 例通过 |

## 6. 边界说明

- 未进入 Step 4/5/6；`S_upper` 仍是历史 `min(60, max(12, 3d+10))`，未实现
  guide 相关有限域或 v3 证书。
- 未修改 optimizer 19 变量模型、A*/corridor 算法、三车模型、audit 阈值。
- 未 commit/push；本文件不构成 Codex 复核结论或 TASK 01 验收结论。