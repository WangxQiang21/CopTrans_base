# Step 4 guide 相关有限 S 域证据

- 日期：2026-11-19
- BASE HEAD：`77805cd46849ebadc374187f9b00fce0f7151f84`（未提交、未 push）
- 工作目录：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- Python：3.8.10；ROS Noetic；CasADi 3.7.2
- 状态：Claude 自测完成；不声明 Codex PASS，不声明 TASK 01 验收通过

## 1. 运行环境与命令

执行方案 §5.1 的字面 `export PYTHONPATH=.../src` 会覆盖 ROS/helper 路径；采用与
Step 1–3 相同的增量前置：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

命令清单见 `commands.txt`。本目录是全部 Step 4 改动（含有限性 fail-closed 修正）
完成后的**最终权威重跑**：每组 stdout/stderr、退出码、墙钟分别保存在 `g*.log` /
`g*.exit_code` / `g*.duration`；探针为 `s_domain.{log,exit_code,time}`。
`py_compile`（contracts/optimizer/pipeline、两个测试文件、探针）与
`git --no-pager diff --check` 均通过。

## 2. §5.1 权威结果

| 组 | 命令（测试模块） | 用例数 | 日志内时长 | 墙钟 | 退出码 | 日志 |
|---|---|---:|---:|---:|---:|---|
| G1 | `tests.test_virtual_a_corridor_display tests.test_formation_hud` | 67 | 19.066 s | 19.68 s | 0 | `g1_corridor_display_hud.log` |
| G2 | `tests.test_virtual_a_receiver tests.test_virtual_a_playback` | 49 | 55.072 s | 55.66 s | 0 | `g2_receiver_playback.log` |
| G3 | `tests.test_formation_demo_assets` | 28 | 3.704 s | 3.77 s | 0 | `g3_demo_assets.log` |
| G4 | `tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer` | 64 | 74.109 s | 74.54 s | 0 | `g4_corridor_optimizer_optimizer.log` |
| G5 | `tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery` | 38 | 123.424 s | 123.82 s | 0 | `g5_pipeline_recovery.log` |
| G6 | `tests.test_virtual_a_geometry tests.test_virtual_a_profile` | 42 | 2.311 s | 2.58 s | 0 | `g6_geometry_profile.log` |
| G7 | `tests.test_virtual_a_integration_bounds tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit tests.test_virtual_a_trajectory` | 97 | 6.391 s | 6.66 s | 0 | `g7_integration_audit_trajectory.log` |
| G8（补充） | `tests.test_virtual_a_ros_adapter` | 63 | 15.212 s | 15.80 s | 0 | `g8_ros_adapter_supplemental.log` |

七组共 385 例；含 G8 共 448 例，全部 `OK`。相对 Step 3 新增 7 例
（G4 +6、G5 +1），无既有用例被删除或弱化。

## 3. 实际修改

| 文件 | 修改 |
|---|---|
| `formation_continuous_a/contracts.py` | `AttemptRecord` 增加 `distance_m, guide_length_m, s_lower_m, s_upper_m, s_solution_m, s_upper_active`，只在非 `None` 时以 `distance_m/guide_length_m/S_lower_m/S_upper_m/S_solution_m/upper_active` 序列化；历史 attempt payload 不变。 |
| `formation_continuous_a/optimizer.py` | `_search_packet(request, config, heading_branch, guide_length_m)` 在同一个 packet 内计算 `kappa_ref_max = min(config.kappa_limit, 1/request.limits.r_min_ackermann)`、`S_lower = max(intervals*0.05, d, abs(unwrapped_heading)/kappa_ref_max)`、有 guide 时 `S_upper = max(S_lower+2, 12, 3d+10, 1.15*guide_length)`、无 guide 时去掉末项；`GUIDE_S_UPPER_FACTOR=1.15` 为技术方案批准的首轮系数。删除旧 `min(60, ...)` 总帽，但显式拒绝非有限 `lower/upper`（`INVALID_SEARCH_DOMAIN`）与非法 guide（`INVALID_GUIDE_LENGTH`）。packet 记录 `distance, guide_length_m, S_lower, S_upper, S_upper_active_tol_m, S_solution, upper_active`。`_guess_for`/`_route_guess` metadata 记录 `endpoint_distance_m, guide_length_m, S_lower_m, S_upper_m, S_upper_active_tol_m`。`_attempt_record` 从同一 metadata 和成功 solver 向量 `z[0]` 记录六个 S 字段；`S_solution` 只取 `success=True` 的向量，超时 iterate 不算解。`upper_active = abs(S_solution-S_upper) <= max(1e-6, 100*配置求解容差)`（scipy 用 `scipy_tolerance`，auto 取两者较大），不是字符串状态。成功返回前用选定 audited candidate 关闭 packet 的 `S_solution/upper_active`。solver/initial/soft wall/`MAX_INITIALS` 未改。 |
| `formation_continuous_a/pipeline.py` | 新增 `_s_domain_evidence(geometry)`：从 attempt 记录取 `distance_m, guide_length_m, S_lower_m, S_upper_m, S_solution_m, upper_active`；成功时优先匹配 solved S 等于返回路径长度的 audited attempt，否则取最后一次 attempt。NLP_SOLVE complete 进度和每个 `search["rounds"]` 记录均带这六个字段。 |
| `tests/test_virtual_a_optimizer.py` | 新增 `SDomainPacketTests` 6 例：无 guide 公式；guide 的 `1.15` 项；50 m/100 m 直线构造（上界 160/310，均越过旧 60 m 帽且 initial 全部在域内）；attempt 的 S_solution 与容差 `upper_active`（含失败/超时不记解）；packet 与 audited attempt 对选定 S 一致；病态 domain（`kappa_ref_max<=0`、非有限上界、非法 guide）fail closed。 |
| `tests/test_virtual_a_pipeline.py` | 新增 1 例：attempt 与 `search["rounds"]` 的六个 S 字段一致；每轮上界严格等于 `max(S_lower+2,12,3d+10[,1.15*guide])`，`upper_active` 严格等于 `|S_solution-S_upper|<=1e-6`；真实 winding recovery 的第二轮使用不同 unwrapped heading 的 `S_lower`。 |
| `work_flow/TASK01_evidence/04_s_domain/probe_s_domain.py` | 离线证据脚本（不启动 ROS）：Step 0 地图/请求/`solve()` 的 600003/600004，加 50 m/100 m 合成长直线（同一冻结 solver 策略、无 guide）；独立重算期望域并与 attempt/round 逐项比对。 |

未修改：`work_flow/TASK01_技术方案.md`、`work_flow/TASK01_Codex执行方案.md`、
`TASK01_evidence/00_baseline/`、`01_visualization/`、`02_research_entry/`、
`03_winding/`、三项独立 audit、`MAX_RECOVERIES`、`MAX_INITIALS`、几何/动力学/碰撞
阈值、优化器 19 变量模型与 A*/corridor 算法。

## 4. 真实 600003/600004 证据

探针地图/请求与 Step 0 完全一致（同一 YAML/PGM/grid/request sha；600003/600004
request sha 仍为 `bc748779…852b`/`141decf7…e718`）。配置：corridor / 8 intervals /
6 initials / 30 s / IPOPT。结果见 `s_domain_600003_600004.json`。

| 项 | 600003 | 600004 |
|---|---|---|
| 结果 | `SUCCESS`，可执行 | `BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED / GEOMETRY`，无 plan |
| distance `d` | 10.018651056903819 | 9.52983048117856 |
| guide length | 18.62582715074392 | 45.74442520707356 |
| unwrapped heading | −3.126（k=0） | −3.159185…（k=−1；物理 +3.124） |
| `kappa_ref_max` | 0.5524861878453039 | 0.5524861878453039 |
| `S_lower` | 10.018651056903819（d 主导） | 9.529830481178559（d 主导；转向需求 5.718 m） |
| `S_upper` | 40.055953170711454（`3d+10` 主导） | **52.606088988134594**（`1.15*guide` 主导） |
| 旧实现 `S_upper` | 40.0560（未变） | 38.5895（本步提高；旧 60 m 帽未触及） |
| attempts | 1 × `Solve_Succeeded` 41 it，`SOLVER_SUCCESS_AUDITED` | 3 × `Solve_Succeeded`，均 `CANDIDATE_CONSTRUCTION_FAILURE` |
| `S_solution` | 21.701601208480245 | 48.436298611556154 / 48.44418169867016 / 48.444190637681004 |
| `upper_active` | False | False（三者都远离 52.606，失败不是 S 上界活跃） |
| 路径 | 21.701601208480245 m / 63.64682553904238 s | 无 |
| 域一致性 | 5 项 checks 全 True | 4 项域 checks True；`path_length_equals_solution=False`（无路径） |

要点：

1. **600003 未回归**：`S_lower/S_upper` 与旧实现逐位一致（guide 项 21.42 m 低于
   `3d+10`），路径长度与 Step 0 基线 `21.701601208480245 m` 逐位一致，仍 41 it、
   audit 通过。新增 S 字段全部等于独立重算值。
2. **600004 的失败不再是 S 上界不足**：新上界 52.606 m 已覆盖技术方案预计的
   ~48.44 m；本探针最终三次 solver 成功向量 `S≈48.436/48.444 m` 均
   `upper_active=False`，说明它们没有贴住上界。失败点是
   `BOUND_UNCONFIRMED: Simpson refinement would exceed 200000 subintervals`
   （v2 数值证书，107008 panels），属于 Step 5 的 v3 证书范围；随后 budget
   到期使整轮返回 `SOFT_BUDGET_EXHAUSTED`。本步没有实施二次扩域，也没有把该
   certificate 失败误判为 S 不足。
3. **上界有限**：600004 的 52.606 m < 60 m，不是旧帽问题；旧公式的 38.59 m 才是
   此前的限制。600003/600004 的所有 attempt/round `S_lower/S_upper` 与探针独立
   重算完全一致（≤1e-9）。

## 5. 合成 50 m / 100 m 直线构造（无旧帽）

同一 600003 起点、沿 +x 的直线，legacy（无 guide）冻结 solver 策略；
`s_domain_600003_600004.json` 中 plan 900001/900002。

| 项 | 50 m | 100 m |
|---|---|---|
| distance | 50.0 | 100.0 |
| `S_lower` | 50.0 | 100.0 |
| `S_upper` | **160.0** | **310.0** |
| 旧 60 m 帽会给出 | 60.0 | 60.0 |
| 是否越过旧帽 | 是（160 > 60） | 是（310 > 60） |
| 构造的 initial `S` | 60, 60, 90, 90, 130, 130 | 120, 120, 180, 180, 260, 260 |
| solver 结果 `S` | ≈50.0（6/6） | ≈100.0（前 4 个）；后 2 个 `310.00000000973154` |
| `upper_active` | 全 False | 前 4 个 False；后 2 个 **True**（|310.0000000097−310|≈9.7e-9 ≤1e-6） |
| 结束状态 | `NO_ACCEPTED_CANDIDATE / CANDIDATE_CONSTRUCTION_FAILURE` | 同左 |
| 失败原因 | v2 `BOUND_UNCONFIRMED`（100480 panels） | v2 `BOUND_UNCONFIRMED`（100776 panels） |

要点：

1. 50 m/100 m 的初始值（最高 130/260 m）与上界（160/310 m）都明显超过旧 60 m
   帽，直接证明 `S_upper` 已无总帽；同时每次上界仍有限。
2. 100 m 最后两个 attempt 的 `S` 略高于上界 9.7e-9 m，按 1e-6 m 求解容差判为
   `upper_active=True`，验证了“容差而非字符串状态”的判断方式；其余 attempt
   为 False。
3. 两个合成长直线都在 v2 数值证书处 fail closed（无 plan、无执行轨迹），符合
   “病态/预算反例仍 fail closed”；它们不是 S 域不足，不能在本步通过调大上界
   或放宽证书“修好”，留给 Step 5 的 v3 证书。

## 6. 定向验收链对应

| 方案要求 | 证据 |
|---|---|
| `kappa_ref_max = min(kappa_limit, 1/r_min_ackermann)` | `test_finite_s_domain_formula_without_guide`；真实探针 0.5524861878453039 |
| `S_lower = max(intervals*0.05, d, abs(unwrapped_heading)/kappa_ref_max)` | 单测公式；测试中 winding 第二轮（k=−1）的 `S_lower=11.372565...`（2π/0.5525）由 `SDomainSearchEvidenceTests` 断言 |
| `S_upper = max(S_lower+2, 12, 3d+10[, 1.15*guide])` | `test_guide_length_raises_the_finite_upper_bound_with_approved_factor`；600004 52.606089=1.15×45.744425；50/100 m=160/310 |
| 取消 60 m 总帽 | 50/100 m 合成构造 160/310，initial 130/260；旧帽均会截为 60 |
| 每次上界仍有限 | 显式 `INVALID_SEARCH_DOMAIN`（非有限 lower/upper、非正 kappa_ref_max）；`test_pathological_domains_fail_closed` |
| `1.15` 为批准的首轮实验系数，不自行调参 | 常量 `GUIDE_S_UPPER_FACTOR=1.15` 集中定义；未改任何其他系数 |
| packet/guess/attempt/search 记录六个字段 | packet 键、两种 guess metadata、`AttemptRecord`、`search["rounds"]` 与 NLP_SOLVE complete metrics；`test_attempt_and_search_round_share_the_same_finite_domain` |
| `upper_active` 用求解容差而非字符串 | `_attempt_record`：`abs(S_solution-S_upper) <= max(1e-6, 100*tol)`；单测覆盖 True/False；100 m 的 310.0000000097 判 True |
| 不实现受控二次扩域 | 无二次扩域代码；600004 证书失败不在本步重试或放大域 |
| solver/initial/soft wall/MAX_INITIALS 不变 | 仅改 packet 上界公式；`max_seconds/max_initials/max_iterations` 与 Step 0 冻结值一致 |
| 病态/预算反例 fail closed | 非法 guide/非有限域→`INVALID_REQUEST`；600004 与 50/100 m 均无 plan；G5 budget 测试仍通过 |

## 7. 边界说明

- 未进入 Step 5/6；未实现 v3 生产积分证书或 v2 重放。600004 与 50/100 m 的
  `BOUND_UNCONFIRMED` 是当前 v2 证书的已知边界，Step 5 才处理。
- 未修改优化器 19 变量模型、A*/corridor 算法、三车模型、audit 阈值与预算。
- 未 commit/push；本文件不构成 Codex 复核结论或 TASK 01 验收结论。