# E04 证据拆分：live 失败关闭 vs. 离线 OBB NLP 负例

日期：2026-11-20
依据：`work_flow/TASK03_Codex返工执行方案.md` §5 第 158–161 行

> E04 报告需拆成两个结论：
> - 已有 live E04：Guide assessment 在 NLP 前发现地图碰撞，`formation_obb_constraint_count=null`；只证明 fail closed 与无旧轨迹发布。
> - OBB NLP 负例：离线 solver/integration test 必须证明实际 OBB rows 数量大于零，F-only rows 可行而 OBB rows 有正残差时不能成为 accepted candidate。若另构造 live 版本，也必须证明它确实进入了 NLP。

本报告分别给出这两类证据，不把前者当作后者。

## 结论 1（live E04，只证明 fail closed）

证据目录（原始 TASK03，未修改）：
`work_flow/TASK03_evidence/20261119T133018Z/step6/live/negative_e04_map05_175/`

请求：map05，scale 1.75，现成 formation start `(1.51, 2.5, −π/2)` → goal `(1.51, 0.5, −π/2)`
（`capture_summary.json` → `request`；`plan_id=600000`）。

观测：

| 项 | 值 | 来源 |
|---|---|---|
| HUD 诊断 | `samples=5 \| map_collisions=3 \| single=3 \| union=0 \| gap=0 \| unknown=12 \| status=COLLISION_OBSERVED` | `hud_texts.json[0]` |
| 失败层 | `PLANNING`，`pipeline_status=NO_ACCEPTED_CANDIDATE`，`reason=FORMATION_CORRIDOR_UNCONFIRMED` | `quality_report.json` |
| `formation_corridor_audit` | `status=TEMPLATE_BUILD_FAILED`，`scope=three_vehicle_union_v1`，`detail="assessment station sigma=0.5 map status='COLLISION_OBSERVED'"` | 同上 |
| `formation_obb_constraint_count` | **null** | 同上 |
| `reference_constraint_count` / `formation_reference_constraint_count` | **null / null** | 同上 |
| `checks` / `certification` | `0` / `NONE` | 同上 |
| `attempts` | `[]`（空） | 同上 |
| `trajectory_published` / `playback_confirmed` / `publication` | `false / false / null` | 同上 |
| 接收到的 envelope | `executable_envelope_count=0`；两条非 executable 状态 envelope 均 `sample_count=0`、`model_sha256=""`、`formation_binding_present=false`、`formation_binding_length=0` | `capture_summary.json` |
| 状态序列 | `IDLE → PLANNING → REJECTED`，最后一帧 `trajectory_published=false` | `capture_summary.json → statuses` |
| RViz 截图 | `images/live_01_00.8s.png` … `live_05_07.0s.png`；HUD `AUDIT geometry=UNKNOWN dynamics=UNKNOWN collision=UNKNOWN` | 同目录 |
| 规划 debug | 只有 `REQUEST_ADMITTED`、`ASTAR_SEARCH`、guide 诊断事件，没有任何 `CORRIDOR_BUILD`/`SOLVE`/`CERTIFICATE` 阶段 | `planning_debug.jsonl` |

**能证明**：Union scope 在该输入上 fail closed；Guide assessment 在模板构建/NLP 之前发现
`car2` 的 3 个采样站地图 OBB 碰撞（s=1.00/1.50/2.00，`MAP_OBB_COLLISION`）；没有轨迹、
没有可执行 envelope、没有复用旧轨迹。

**不能证明**：它没有进入 NLP，`formation_obb_constraint_count=null`，因此不能作为
“OBB rows 拒绝了 F-only 可行轨迹”的证据。

## 结论 2（离线 OBB NLP 负例，证明 rows 真实且拒绝）

测试：`tests/test_virtual_a_formation_corridor.py::
FormationSampleAuditStep3Tests::test_e04_offline_obb_rows_are_real_and_reject_f_only_feasible`
（本次返工新增）。探针脚本：`e04_offline_probe.py`（本目录），输出
`e04_offline_probe.out` + `.meta`（exit=0，wall 11.26 s）。

场景：`_step3_scene(y_half=1.1, max_seconds=3.0)`，短直线走廊，两侧墙在 y=±1.1 m；
模板 `_step3_scene` 的 single-region 模板。该场景中三车完整 OBB 的角点超出墙内约
0.15 m，但 F 参考点仍有 >1 m 余量（F-only 可行）。

实测数值（探针输出逐字）：

| 项 | 值 |
|---|---|
| `constraint.sample_count`（旧 F-only corridor groups） | 0（union 取代） |
| `formation.f_scalar_row_count`（模板 F rows） | 324 |
| `formation_obb_constraint_count`（本字段只计 OBB rows） | 3888 |
| `rows_by_vehicle` | `{1:1296, 2:1296, 3:1296}` |
| `f_region_by_segment` | `[0]` |
| 固定决策向量 `_z(0)` 下 F rows 最大值 | **−1.09999**（F 参考点位于墙内 1.1 m） |
| 同一向量下 OBB rows 最大值 | **+0.15001 m**（完整 OBB 越界 0.15 m） |
| numeric 向量总长 = `corridor_end` | 4486 = 旧 rows 0 + F 324 + OBB 3888 + 等式/符号行 |
| 求解入口 | `solve_open_path_guided(..., formation_template=..., planning_context_sha256=CONTEXT_SHA)` |
| 返回 `formation_obb_constraint_count` | 3888（非 null，证明进入 NLP 且携带真实 OBB rows） |
| 返回 status / reason | `BUDGET_EXHAUSTED` / `SOFT_BUDGET_EXHAUSTED`（IPOPT 尝试 `SOLVER_FAILED`） |
| `result.path` | `None` |
| accepted attempt（`SOLVER_SUCCESS_AUDITED`） | **false** |

**能证明**：在真实的 solver 调用中（不是早退、不是标记层）：

1. OBB rows 数量为 3888，按车辆分布 1296/1296/1296，全部为正；
2. 同一固定决策向量上 F rows 全部 ≤ −1.09999（F-only 约束满足），而 OBB rows 最大
   +0.15001 m（正残差），因此“F-only 可行 + OBB 正残差”确实出现且被拒绝；
3. 返回结果带 `formation_obb_constraint_count=3888`，无 path、无 `SOLVER_SUCCESS_AUDITED`；
4. 与 `test_f_feasible_but_obb_infeasible_solve_is_never_accepted` 一起覆盖“同一场景
   F 可行而 OBB 正残差不得 accepted”。

**没有 live E04 版本**：本次未另构造进入 NLP 的 live OBB 负例；结论 1 保持其原有语义。
若需要 live 版，应在交付报告中明确标 `NOT_RUN`（见交付文档）。

## 数值一致性

`test_f_rows_shared_by_numeric_casadi_and_scipy_fallback`（同一次返工）在 row-level fixture
上证明：同一决策向量下 CasADi `g`、numeric 向量、SciPy fallback 约束函数一致，
`np.testing.assert_allclose(symbolic, numeric, atol=1e-8, rtol=0.0)`；F 行区间在
`g` 中为 `(-inf, 0]`，SciPy 侧 `corridor_values[-obb_count - f_count:-obb_count]` 与
numeric F 行完全相等（`atol=0.0`）。该测试属于 Step B/C 证据，日志见 `../step_bc/`。

## 复现

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD:$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e04/e04_offline_probe.py
python3 -m unittest \
  tests.test_virtual_a_formation_corridor.FormationSampleAuditStep3Tests.test_e04_offline_obb_rows_are_real_and_reject_f_only_feasible
```