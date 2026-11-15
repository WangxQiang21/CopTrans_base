# FR04-R1 返工报告：独立审核绑定原始请求

日期：2026-11-14。工作区：`/home/wq/whut_thesis/expv3_multi_v02`（V0.2 worktree）。
依据：`evaluation/formation_refactor_v02/FR04/codex_acceptance_20261114T033314Z/ACCEPTANCE.md`
唯一阻塞项 FR04-R1。未提交 Git，未进入 FR05，未运行 ROS/RViz。

## 1. 变更范围

只修改：

| 文件 | 变更 |
|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_pipeline.py` | 新增请求/权威边合同构建、plan 合同校验、结构化失配结果；`_build_and_audit` 不再使用 `UniformAuditRequest.from_plan`。 |
| `tests/test_uniform_pipeline.py` | 独立复审改为“原请求 + 权威边”；新增 F13–F19。 |
| `evaluation/formation_refactor_v02/FR04/rework_01/` | 本轮证据（本报告、日志、逐例结果、探针、基线核验、失败轮次）。 |

未修改 FR01/FR02/FR03 已验收源码与测试、`tests/test_uniform_search.py`、冻结文档、
车辆/轮系/碰撞/安全限值、搜索原语库/去重/预算/限值，也未改 `uniform_search.py`。
未触碰原仓库 `/home/wq/whut_thesis/expv3_multi`，未创建 worktree，未使用 Git 提交/reset/stash。

## 2. 根因与修复

### 2.1 根因

原 `_build_and_audit(edges, request)` 先调用

```python
UniformAuditRequest.from_plan(plan)
```

再把该自描述请求交给 FR02。于是三次审核虽然能验证 plan 的内部几何、动力学和碰撞，
但请求 ID、起点、终点、模型、车形、限值、`allow_reverse` 全部由待审 plan 回填。
在第二次轨迹重建返回值注入另一条自身可行的轨迹时，管道仍返回
`SUCCESS / executable=True / FEASIBLE`。这是管道认证边界缺口。

### 2.2 修复后的顺序

`_build_and_audit` 现在严格按以下顺序执行：

1. `_expected_contract(request, edges, zero_motion)`：在构造前对 `edges` 做 `tuple` 快照。
   - 非空：每条边必须是 `UniformMotionEdge`，offset 与请求模型一致；第一条边起点/初始车头
     必须等于请求起点/初始车头；相邻边位置/车头连续；最后一条边的精确终点和车头成为
     期望终点/终端车头。
   - 零运动：不允许任何边，期望终点就是请求起点，期望终端车头就是请求初始车头。
2. 另用原请求的冻结容差检查期望终点：`goal_xy_tol_m=0.02 m`、`goal_psi_tol_rad=1°`。
   不把 `request.goal_qf` 原样塞进 FR02 的 1e-7 精确比较。
3. 仅当 `goal_heading_free=False` 且请求显式给出 `goal_headings` 时，才检查最后一条边的
   终端车头是否在 `goal_heading_tol_rad=1°` 内；默认自由车头不增加任何相等要求。
4. 用**原请求**构造 `UniformAuditRequest`：`plan_id`、`formation_id`、`model`、
   `vehicle_shapes`、`start_qf`、`start_headings`、`timing_limits`、`allow_reverse`
   全部来自请求；`goal_qf/goal_headings` 使用第 1 步固定的精确边终点（保持用请求起点）。
5. 用同一合同参数调用 `UniformTimedPlan.from_edges`：显式绑定请求起点/初始车头/期望终点。
6. `_verify_plan` 校验构造结果仍属于该请求和该边序列：
   - `plan_id`、`formation_id`、offset、车形、`timing_limits`；
   - 精确起点/初始车头、精确终点/终端车头；
   - 边数相等，且逐条 `plan.edges[i] == authoritative_edges[i]`（dataclass 全字段相等，
     不是只比较端点）。
7. 只有以上全部通过，才 `sample()` 并调用真实 `audit_uniform_timed_plan(contract.audit_request,
   plan, projection, grid)`。

终端候选审核、最终重建审核、零运动保持都走同一个 `_build_and_audit`，合同一致。

### 2.3 失配的返回方式

合同失配抛内部 `_ContractError`，`_build_and_audit` 捕获后返回
`(None, None, FormationEvaluation(REJECTED, feasible=False, reason=<合同码>))`。
它不会抛未处理异常，也不会修改模式/车头/路径去“匹配”，不会放宽阈值。

| 合同码 | 含义 |
|---|---|
| `REQUEST_GOAL_TOLERANCE_EXCEEDED` | 期望边终点超出冻结 0.02 m / 1° |
| `REQUEST_TERMINAL_HEADINGS_MISMATCH` | 请求关闭自由终端车头时的终端车头超差 |
| `REQUEST_START_EDGE_MISMATCH` | 第一条边起点/初始车头不等于请求 |
| `REQUEST_EDGE_SEQUENCE_DISCONTINUITY` | 权威边序列自身不连续 |
| `REQUEST_EDGE_MODEL_MISMATCH` | 权威边 offset 不等于请求模型 |
| `PLAN_IDENTITY_MISMATCH` | plan ID/编队/模型/车形/限值与请求不符 |
| `PLAN_START_MISMATCH` | plan 起点/初始车头不是请求起点 |
| `PLAN_GOAL_MISMATCH` | plan 终点/终端车头不是权威边终点 |
| `PLAN_EDGE_SEQUENCE_MISMATCH` | plan 不对应权威边序列（含同端点换路径） |
| `PLAN_CONSTRUCTION_INVALID` | 构造/采样以 FR02 校验错误失败 |

最终重建阶段失配时，`plan_uniform` 返回 `AUDIT_REJECTED`、`feasible=False`、
`executable=False`、`plan=None`、layer `audit`。终端候选阶段失配则记为一次终审拒绝，
搜索继续；若没有其他分支，返回 `NO_SOLUTION`，绝不返回未审核成功。

## 3. 定向测试

最终套件 19 项全部通过（44.782 s）：

| 用例 | 场景 | 结果 |
|---|---|---|
| F01 | 精确保持 | `NO_MOTION`，可执行，独立复审 `FEASIBLE` |
| F02–F05 | 前/后/横/斜平移 | `SUCCESS`，3/3/9/6 次扩展，独立复审 `FEASIBLE` |
| F06/F07 | 原地 ±90° | `SUCCESS`，27/27 次扩展，各 2 次动态拒绝后找到可行弧 |
| F08 | 平移+旋转 | `SUCCESS`，31 次扩展，2 次动态拒绝后成功 |
| F09 | 围困障碍 | `NO_SOLUTION`，搜索层 |
| F10 | 越界目标 | `REQUEST_REJECTED / GOAL_CENTER_OUT_OF_BOUNDS` |
| F11 | 单次扩展预算 | `BUDGET_EXHAUSTED / EXPANSION_BUDGET_EXHAUSTED` |
| F12 | 真实 FR02 调用次数 | 恰 2 次（终端候选 + 最终重建） |
| **F13** | 目标距实际终点 0.0111803 m、货物角 0.5°；`goal_headings` 占位 π/2 但默认自由 | `SUCCESS`，plan 终点在容差内，终端车头保持 0，未被误拒 |
| **F14** | 目标 `(10.005,10.005,0.5°)`；默认自由车头 | `NO_MOTION` 保持通过，终点精确为请求起点 |
| **F15** | 第二次重建替换为自身可行的 `(0,0,0)→(1,0,0)` 轨迹 | `AUDIT_REJECTED / PLAN_START_MISMATCH`，不可执行，`plan=None` |
| **F16** | 第二次重建 plan_id=116，请求为 16 | `AUDIT_REJECTED / PLAN_IDENTITY_MISMATCH` |
| **F17** | 同端点但用一条 3 m 直行替代权威 3×1 m | `AUDIT_REJECTED / PLAN_EDGE_SEQUENCE_MISMATCH` |
| **F18** | 第二次重建用 `v_max=0.7` 的自洽 plan，请求默认 0.8 | `AUDIT_REJECTED / PLAN_IDENTITY_MISMATCH` |
| **F19** | 同端点、同 3 条边数，但用 O 平移替代权威 A 直行 | `AUDIT_REJECTED / PLAN_EDGE_SEQUENCE_MISMATCH` |

F15–F19 的替换 plan 都先用真实 FR02 自审为 `FEASIBLE`，证明拒绝来自请求/边序列绑定，
而不是替换轨迹本身不可行。测试保留真实 FR02，只注入轨迹构造输出或输入合同。

独立复审现在调用 `expected_audit_request(request, authoritative_edges, zero_motion)`，
其字段来自原请求和 `result.search.edges`（保持为 `()`），不使用
`UniformAuditRequest.from_plan`。记录同时保存 `edge_sequence_matches` 和
`goal_within_request_tolerance`。F02–F08、F12、F13 的复审均为 `FEASIBLE`，
刚性残差 ≤2.3e-15 m，有限差分 ≤2.6e-5，动态峰值比 0.9868–0.9950。

### 3.1 保留的失败轮次

| 轮次 | 现象 | 处置 |
|---|---|---|
| `iteration_02/` | F18 首次用低 `v_max` 定时但沿用原 MotionLimits 的边构造替换计划，其自审被 FR02 以 `UNIFORM_MODEL_MISMATCH edge0.limits.target_v` 拒绝，不能作为“自身可行”样本 | 改为用 `timing_motion_limits(0.7)` 同步构造边和 plan，自审 `FEASIBLE`；日志保留 |
| `iteration_03/` | 合同曾要求每条边的 `allow_reverse` 等于请求值；F06/F07/F08 所有终审候选因此被拒，搜索耗尽 30 s 时间预算 | 删除该过度约束。`allow_reverse` 是请求许可，S 边可携带 `False`；FR02 已同时检查边标志和请求许可。`iteration_04/` 重跑恢复 27/27/31 次扩展 |
| `previous_runs/final_before_docstring_edit/` | 最终 19 项已通过，但随后修改了测试文件 docstring，源码哈希失效 | 归档整轮；`final/` 用最终源码重跑 19/19 |

## 4. 回归与基线

| 项目 | 结果 |
|---|---|
| FR04 `test_uniform_pipeline.py` | 19/19 通过，44.782 s |
| FR03 `test_uniform_search.py` | 12/12 通过，158.544 s |
| 十个既有相关套件 | 145/145 通过（17+15+14+28+22+14+13+9+5+8） |
| 基线核验 | `all_accepted_match=true`、`all_frozen_match=true`、`fr03_prefix_match=true` |

FR01/FR02/FR03 源码与测试、冻结文档的 SHA-256 与已验收记录一致；`uniform_search.py`
去掉两条 FR04 import 和追加段后仍精确还原为 `982b2f96…f96e83`。

## 5. Codex 探针的修复后结果

原探针的故障注入不变：只在第二次轨迹重建返回注入一条真实、自身 `FEASIBLE` 的
`(0,0,0)→(1,0,0)` 计划。由于预期拒绝不再有 `result.plan`，本目录中的适配版
`codex_probe/adapted_independent_probes.py` 只保存拒绝结果，不再无条件读取
`result.plan.start_qf/goal_qf`。真实输出：

| 探针场景 | 结果 |
|---|---|
| 注入最终重建 | `AUDIT_REJECTED`，`executable=False`，`plan_present=false`，`PLAN_START_MISMATCH`；注入计划自审 `FEASIBLE` |
| 同一注入计划交给绑定原请求的 FR02 | `REJECTED / UNIFORM_MODEL_MISMATCH`，`plan.start_qf.x actual=0 expected=10 error=10 > 1e-07` |
| 同端点不同路径注入 | `AUDIT_REJECTED`，`PLAN_EDGE_SEQUENCE_MISMATCH`，不可执行 |
| 真实最终审核被拒 | `AUDIT_REJECTED`，`executable=False` |
| 失败后新请求 ID=402、目标 x=11 | `SUCCESS`，`executable=True`，plan_id=402，目标 x=11 |

原始 Codex 探针文件未被修改；本适配版位于 `rework_01/codex_probe/`。

## 6. 证据

- `final/pipeline_results.json`：19 例请求/地图/配置哈希、预算、路径、计数、拒绝分布、
  model、样本、计时、T_end、独立复审（含 `edge_sequence_matches`）；
- `final/cases/F01…F19.json`：逐例记录；
- `final/test_uniform_pipeline.log`、`fr03_tests.log`、`regression/*.log`、
  `logs_regression_suites.log`；
- `fr03_tests.log`：FR03 12/12；
- `codex_probe/adapted_independent_probes.{py,json,log}`；
- `baseline_verification.json`、`baseline_verification.log`、`verify_baseline_rework.py`；
- `iteration_02/`、`iteration_03/`、`iteration_04/`、`previous_runs/`：失败与中间轮次；
- `pre_rework_evidence_manifest.json`、`pre_rework_command_manifest.json`：返工前索引备份。

### 6.1 意外覆盖披露（如实保留）

在编写修复前的冒烟运行中，曾漏设 `FR04_ARTIFACT_DIR/FR04_RESULTS_PATH`，导致
`FR04/pipeline_results.json` 与 `FR04/cases/F01_hold.json`、`F02_forward.json`
被一次仅含 F01/F02 的运行覆盖。被覆盖后产生的 2 例文件已移入
`accidental_overwrite/`，未删除。原 12 例最终 `pipeline_results.json` 的
SHA-256 仍保存在 `pre_rework_evidence_manifest.json`，但其字节内容无法从
本机恢复；Codex 的 `codex_acceptance_20261114T033314Z/fr04_results_path.json`
是 Codex 自己的独立复跑，不是本轮原文件，不能冒充恢复。其余 F03–F12 逐例文件、
所有日志、失败轮次和清单均未被覆盖。本轮修复后的完整 19 例结果在
`rework_01/final/`，可重新生成全部数值。

## 7. 未变限制与停止点

- 默认固定 ±10° 圆弧的 jerk 保守界仍可能返回 `UNIFORM_DYNAMIC_UNCONFIRMED`；
  本轮未改 FR02、未放宽动态界。
- 加权启发不构成最优性/完备性声明；`NO_SOLUTION` 只针对该地图、候选库和预算。
- 仅离线 FR02 审核通过；未验证 ROS 发布、RViz 回放、控制器、动力学或实车。
- 未提交 Git，未进入 FR05。到此停止，等待 Codex 独立复核。