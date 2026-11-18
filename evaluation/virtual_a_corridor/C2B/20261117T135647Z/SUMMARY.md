# C2-B 阶段总结：走廊半空间进 NLP、连续包含审核与 pipeline 接入

**状态：C2-B 代码、定向测试和证据已完成，工作区未提交，等待 Codex 独立验收。不提交 Git、不推送、不启动 ROS/RViz、不进入 C3。**

## 1. 基线、前置与工作区

| 项目 | 值 |
|---|---|
| 阶段 | C2-B |
| UTC run | `20261117T135647Z` |
| 任务书 | `plan/virtual_a_corridor_v01/TASK_C2.md` |
| 任务书源码基线 | `23fd16227e75a68f03f3ac7483555063c8189313` |
| 证据生成时 HEAD | `9a19fd3e22d33188b13795f695757c3e4a36909f`（C1 提交） |
| 分支 | `feature/virtual-rigid-formation-a` |
| 用户指令 | "v0 v1审核通过，然后推进c2 b"（V0/V1 已由用户确认通过） |
| Python / NumPy / SciPy / CasADi | 3.8.10 / 1.24.4 / 1.10.1 / 3.7.2（IPOPT 插件可用，见 `environment.txt`） |
| 源码目录 P | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a` |

### 1.1 Git 状态

C2-B 增量只涉及任务书允许的文件：

| 文件 | C2-B 增量（相对 pre-C2B 基线） | 相对 HEAD 的工作区 diff |
|---|---:|---:|
| `P/contracts.py` | +188 / −15（`source_diff_c2b_contracts.diff`） | +204 / −18（含 C2-A 的 +16/−2） |
| `P/corridor.py` | +331 / −3（`source_diff_c2b_corridor.diff`） | +870 / −5（含 C2-A 与 V1 显示改动） |
| `P/optimizer.py` | +493 / −61（`source_diff_c2b_optimizer.diff`，相对 HEAD） | 同左 |
| `P/pipeline.py` | +183 / −20（`source_diff_c2b_pipeline.diff`，相对 HEAD） | 同左 |
| planner node `main()` | +8 / −2（`source_diff_c2b_node.diff`） | +82 / −5（含 V1/Cargo HUD 改动） |
| `formation_demo.launch` | +3（`source_diff_c2b_launch.diff`） | +22（含 V1/Cargo HUD 改动） |
| `tests/test_virtual_a_corridor_optimizer.py` | +379（`source_diff_c2b_tests.diff`） | 新文件 723 行（C2-A 344 + C2-B 379） |

pre-C2B 基线重建方法：`contracts.py`/`corridor.py` 用 C2A 证据目录的 `pre_c2a_v1_corridor.py` + `source_diff_c2a_*.diff` 重建，SHA-256 与 C2A `source_hashes.txt` 完全一致（`383e87cd…`、`0172b0ff…`）；node/launch 用 HEAD + V1 `v0_v1_changes.patch` + Cargo HUD `cargo_hud.patch` 重建；`optimizer.py`/`pipeline.py`/测试 C2-A 前与 HEAD 相同，故直接用 `git diff HEAD`。`head_equality.txt` 逐行确认：**geometry.py、timing.py、audit.py、collision.py、trajectory.py、ros_adapter.py 六个禁改文件与 HEAD 完全一致（exit 0）**；`ros_adapter.py` 本阶段零改动（guidance_mode 经 `VirtualABackendSettings.planner_config` 传递，该字段早已存在，无需改 ros_adapter）。

工作区另有先前未提交的 V0/V1/Cargo HUD 改动（`formation_playback.py`、playback 脚本、rviz 配置、相关测试），不属于 C2-B 增量。`git_state.log` 记录完整 status/diff stat。**本阶段没有新建分支、commit 或 push。**

## 2. 实现

### 2.1 PlannerConfig 单一来源（contracts.py）

- `guidance_mode=legacy_route|corridor`，**默认 legacy_route**；`corridor_enforcement=hard|soft`，默认 hard；共 14 个走廊字段（margin、margin_portal_fraction、kappa/guide/clearance 权重、clearance_target、quadrature_points、audit 5 参、box_padding）。
- `as_dict()` 默认仍是**原 16 键**（任何走廊字段等于默认值都不出现），非默认时追加走廊键进 context 配置摘要（可重现）；`planner_config_from_dict` 对省略键回填默认、未知键报 `INVALID_PLANNER_CONFIG`。测试：默认编码逐键相等、round-trip、部分 payload、未知键拒绝（`test_legacy_config_encoding_and_corridor_opt_in`）。

### 2.2 走廊半空间进两个 solver（optimizer.py）

- `solve_open_path_guided(..., corridor_chain=None, corridor_allocation=None)` 关键字扩展，旧调用不变。
- `_build_corridor_constraint`：把世界系 `A p ≤ b−ε` 变换到 request-start 局部系（`A_local=(A0 c+A1 s, −A0 s+A1 c)`，`b_local=b−ε−A0·start.x−A1·start.y`），每段 Hermite 用 `_piece_position_samples(..., quadrature_count=9)` 在分配 σ 区间 `[σa,σb]` 采样 σa、9 点 Gauss 映射、σb；q=0 精确取段起点。有效 margin `ε=min(corridor_margin_m, portal_fraction·min_portal_radius)`，默认 1e-5 m。
- **数值 SLSQP**：`_numeric_constraint_vector` 追加走廊行（`-corridor_values` 进不等式），`_constraint_value_offsets` 给出 `(equality_end, signed_end, corridor_end)`；**CasADi/IPOPT**：`_build_casadi_nlp` 在曲率行后追加同序走廊行（下界 −inf、上界 0）。测试证明两 solver 走廊切片数值 `allclose(atol=1e-12)`（`test_halfspace_residual_is_identical_in_numeric_and_casadi_nlp`）。
- 残差诊断：接受门检查走廊切片，>1e-8 拒绝 `CORRIDOR_HALFSPACE_RESIDUAL`，残差不可得拒绝 `CONSTRAINT_RESIDUAL_FAILURE`（保留历史 "constraint residual unavailable" 措辞）。所有 attempt 失败且 corridor 模式下报 `FIXED_ALLOCATION_LIMIT`（optimizer.py:2070）。
- 初值语义：corridor 模式下 guide 只作用于 map_route_guide 初值的弱偏好（`corridor_guide_weight`）；automatic endpoint 初值**不被拉回旧 route0**，可行性来自硬半空间。
- 输入校验：legacy 模式带走廊输入 → `CORRIDOR_INPUTS_IN_LEGACY_MODE`；corridor 模式 >1 guides → `CORRIDOR_GUIDE_AMBIGUOUS`；缺 chain/allocation → `CORRIDOR_INPUTS_REQUIRED`。

### 2.3 连续包含审核（corridor.py `audit_path_in_corridors`）

- 真模型在区间中点求值，保守界 `n·p(mid) + Δs/2 + ε_total ≤ b`，`ε_total = position_budget_m + path.position_error_bound_m + 1e-12`。
- 切分集合：allocation 边界 ∪ Hermite 结点 ∪ profile domain segments；自适应二分，深 20 / 总 20000 / 宽 1e-5 m 触限即 `UNCONFIRMED`/对应 reason。
- hard 模式未确认不可发布该模式成功；soft 模式只诊断不认证。`is_three_vehicle_certificate: False`，不是三车证书。

### 2.4 pipeline 接入（pipeline.py）

- 顺序：路线生成 → 走廊 build（C1 链）→ guide（C2-A QP）→ allocation → 几何求解（两 solver 半空间）→ **连续包含（hard 门）** → retime → dynamics → collision 终审。
- hard 包含失败 → `GEOMETRY_FAILED`，无 path 无 plan（`_empty_result` 且 `VirtualAResult.__post_init__` 强制失败结果不带 path/plan）。碰撞重试沿 G4 既有逻辑（总预算 60 s、`MAX_COLLISION_RETRIES=2`）。
- `result.search["corridor"]["rounds"]` 记录 chain digest、region 行数、allocation、有效 margin、chain audit、guide（source/points/quality）、containment 全量证据。

### 2.5 ROS/launch 接线（允许范围内最小改动）

- launch 新增显式 `virtual_a_guidance_mode:=legacy_route|corridor`，**默认 legacy_route**，仅 `backend==virtual_rigid_a` 时下发同名 param。
- planner 节点 `main()`：`VirtualABackendSettings(planner_config=PlannerConfig(guidance_mode=param))`，无其他节点改动，不改状态机/backend 默认。**本阶段未启动 ROS。**

## 3. 测试

- `tests/test_virtual_a_corridor_optimizer.py` 现共 **20 个测试**：C2-A 13 个 + C2-B 7 个（配置编码、两 solver 半空间一致性、走廊解过独立审核、世界旋转/平移、分配切换映射、连续审核拒绝样本间越界、模式输入校验）。
- 定向套件（五模块一起）：**105 个测试全部通过，exit 0**（`test_logs/targeted_suites.log`）。
- 任务书列出的四条 `unittest discover -p` 命令逐条运行：corridor_optimizer 20/29/23/8 个测试全 OK，exit 0 全记录（`test_logs/*.exit`）。
- 旧 `test_virtual_a_optimizer`/`trajectory`/`pipeline` 无回归（legacy 路径未动其行为）。

## 4. pipeline 证据（19 个用例，全部 exit 0）

### 4.1 正例：door_circumcircle 非平凡走廊解（A/B 同配置）

| 项 | corridor (IPOPT) | corridor (SLSQP) | legacy_route (IPOPT) |
|---|---|---|---|
| 状态 | SUCCESS | SUCCESS | SUCCESS |
| 路径长 | 10.500000000000004 m | 10.5 m | 10.500557699942512 m |
| 连续包含 | `CORRIDOR_CONTAINMENT_VERIFIED`，8 样本，max_point −0.700 m，max_bound −0.0437 m，ε=7.97e-8 | 同左（独立重跑） | —（无走廊） |
| κ 峰值（车 1/2/3） | 5.2e-12 / 3.4e-12 / 3.4e-12 | 5.5e-6 / 3.6e-6 / 3.6e-6 | 0.1756 / 0.0778 / 0.0673 |
| 几何/动态/碰撞三层 | 全过 | 全过 | 全过 |
| 墙钟 | 43.6 s | 15.6 s | 22.3 s |

door 场景 corridor 与 legacy 的 `planning_context_sha256` 不同（7edaaef… vs 85af00…），request/grid SHA 相同——配置差异只有 `guidance_mode` 及其走廊默认值。**至少一条非平凡新走廊解连续包含 + 三层通过：按任务书字面成立**（合成门场景、双 solver 各一份、连续包含认证）。但必须如实指出：该解 κ 峰值仅 ~1e-12（近直线），几何难度低；它是当前唯一 corridor 成功样本，六个真实冻结场景为 0/6，见 §6.5。两 solver 一致有效：SLSQP 也独立给出走廊解并通过同一连续包含与三层审核。

### 4.2 反例（无效输入零误发布，全部无 path 无 plan）

| 场景 | 状态 | 关键证据 |
|---|---|---|
| S1 / S1_mirror corridor (IPOPT 与 SLSQP) | `GEOMETRY_FAILED / CORRIDOR_AUDIT_WIDTH_LIMIT` | 求解器本身成功（走廊残差 9.98e-9 ≤ 1e-8），但连续包含无法闭合界：451 样本、深度 17、区间宽 9.68e-6 m、max_bound_residual 0.59 m。fail-closed，无发布 |
| S2 corridor (IPOPT 30 s / SLSQP 40 s) | `BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED` | 软预算耗尽前无已审候选；SLSQP 版 40 s 内 6 次 attempt 全部失败（走廊残差 1.8e-3–1.8e-2 m），如实报预算与求解失败，不发布 |
| S2_mirror corridor (SLSQP 40 s) | `NO_ACCEPTED_CANDIDATE / FIXED_ALLOCATION_LIMIT` | **固定 8 区间分配无法覆盖该走廊链**，6 次 attempt 全部失败（`Positive directional derivative for linesearch` / 迭代上限，走廊残差 1.8e-3–1.2e-2 m）；这是任务书要求如实上报的固定分配不适用路径，无发布 |
| S3 / S3_mirror corridor (IPOPT 与 SLSQP) | `COLLISION_FAILED / MAP_OBB_COLLISION` | **中心安全但车体碰撞被拒**：连续包含 `CORRIDOR_CONTAINMENT_VERIFIED`（120 样本、max_point −0.175 m），但参考点安全不等于三车安全——OBB 终审在 station 4.104 m 处车 3 撞地图，两轮重试后失败，无发布 |

**`false_publications = 0`（results.json 机器判定）：12 个 corridor 用例（六冻结场景 IPOPT 6 个 + SLSQP 5 个 + door 双 solver）中 10 个拒绝全部无 path 无 plan；door 两份成功均为完整三层审核 + 连续包含认证。**

### 4.3 legacy_route 六冻结场景无回归

六场景 legacy_route 全部 SUCCESS（S1 10.4996 m / S2 / S3 11.8599 m 等），与 G4 冻结结果一致；corridor 六场景如实逐项报告（不强求全成功，见 4.2）。

### 4.4 旧模型 JSON 往返

六场景 frozen request payload 经 `request_from_payload → request_payload → JSON` 往返逐字节相等（`model_roundtrip.log`，SHA-256 前 16 位逐一相同）。

## 5. TASK_C2 C2-B 验收对照

| 要求 | 证据 |
|---|---|
| `guidance_mode` 默认 legacy_route | contracts 默认值 + launch 默认 + 节点 get_param 默认；六场景 legacy run 无走廊键 |
| 走廊参数单一来源、固定常数入 context | `PlannerConfig` 14 字段；`as_dict` 非默认进 context；planning_context SHA 随 mode 区分 |
| 扩 `solve_open_path_guided` 关键字、旧接口不变 | 新关键字默认 None；旧调用签名保持（测试未改旧调用） |
| 两 solver 约束与残差同时扩，不能只 IPOPT | SLSQP 数值向量 + CasADi 均带走廊行；`allclose(atol=1e-12)` 测试；S1/S3/door 的 scipy 端到端用例 |
| 9 点 Gauss + 两端采样、世界坐标 A p≤b−ε | `_piece_position_samples(quadrature_count=9)`；旋转/平移测试 residual ≤1e-9 |
| 端点型初值无旧 route0 强贴线；guide 仅初值/弱偏好 | corridor 模式下 automatic endpoint 初值不挂 route guidance（optimizer.py 注释与实现） |
| `audit_path_in_corridors` 中点界 + 二分 + 触限 UNCONFIRMED | 实现 + 样本间越界负例测试（arc κ=0.5 L=8，真峰值 4.0 vs 采样 3.9928） |
| soft 只诊断不认证；hard 未确认不可发布 | S1 hard 拒绝即证据；soft 分支带 diagnostic 不带证书字段 |
| pipeline 几何后时间前连续包含，再三层终审 | pipeline.py 顺序与 `search_rounds` 记录 |
| 中心安全但车体碰撞必须拒绝 | S3 corridor：containment VERIFIED 但 `MAP_OBB_COLLISION`，无发布 |
| 固定分配不适用报 `FIXED_ALLOCATION_LIMIT` | optimizer.py:2070；**S2_mirror corridor (SLSQP) 端到端触发**：`NO_ACCEPTED_CANDIDATE / FIXED_ALLOCATION_LIMIT`，6 次 attempt 全失败、无 path 无 plan |
| legacy 六冻结场景无新增回归 | 六场景 SUCCESS + 三个回归测试模块全 OK |
| 至少一条非平凡走廊解 + 三层通过 | door_circumcircle corridor（IPOPT 与 SLSQP 双份） |
| 无效反例 0 误发布 | `false_publications: 0` |
| numpy/CasADi 半空间残差 ≤1e-8 m | door/S1/S3 全部 attempt `corridor_residual_m ≤ 1e-8`（多数 0） |
| 配点间越界拒绝 | `test_continuous_audit_rejects_an_exit_between_corridor_samples` + S1 端到端 |
| 交付 A/B 同配置、guide 来源、mapping、包含与三层审核、长度/κ 峰值 | §4.1 表；`corridor_rounds` guide source（endpoint_polyline/qp_polyline）、allocation、containment 全记录在 JSON |
| 旧模型 JSON 往返不变 | `model_roundtrip.log` |
| 不改审核物理门槛/时间律/旧模型/legacy 默认 | `head_equality.txt` 六文件 exit 0 |

## 6. 限制、差异与遗留问题

1. **S2 corridor 未产生已审候选**：IPOPT 30 s 软预算耗尽；SLSQP 40 s 内 6 次 attempt 全部失败（`Positive directional derivative for linesearch` / 迭代上限，走廊残差 1.8e-3–1.8e-2 m）。这不是误发布（`BUDGET_EXHAUSTED` 如实报告），但说明该场景在当前走廊链 + 固定 8 区间分配下求解困难。
2. **`FIXED_ALLOCATION_LIMIT` 已由 S2_mirror corridor (SLSQP) 端到端触发**（optimizer.py:2070）：6 次 attempt 全失败后报 `NO_ACCEPTED_CANDIDATE / FIXED_ALLOCATION_LIMIT`，无 path 无 plan；IPOPT 侧同场景仍走预算路径。该状态不再是"仅单测覆盖、本 run 未触发"。
3. **door corridor 解 κ 峰值 ~1e-12/1e-6（近直线）**：走廊解选择了直线方案，这是该开门场景几何的真实最优附近；κ 峰值对比已如实给出（legacy 因路线引导有小弯）。若 Codex 需要带弯走廊正例，可换 S3 调 corridor_margin 重试——但 S3 的真实几何是"中心安全但 OBB 撞"，正例与反例二选一时本阶段选择了把 fail-closed 证据做满。
4. **S3 corridor 在 60 s 总预算内两轮碰撞重试后失败**——这是任务书要的反例本身（中心安全但 OBB 碰撞必须拒绝），不是缺陷。
5. **六冻结场景 corridor 通过率 0/6 是真实能力边界，不是审核误报**。默认配置下 S1/S1_mirror 连续包含宽度触限，S3/S3_mirror 中心安全但 OBB 碰撞，S2/S2_mirror 预算/固定分配失败。诊断探针（不属于 17/19 用例证据矩阵，只用于定位原因）显示：S1 把 `corridor_margin_m` 调到 1e-3 后可过连续包含，但三轮碰撞重试均在 station≈2.25 m 由 2 号车 OBB 撞上侧墙；再把 `corridor_clearance_weight` 提到 5e4、`corridor_clearance_target_m` 提到 0.25/0.6/1.0、收紧 SLSQP 容差、`corridor_box_padding_m` 调 0.3/0.5/1.0，碰撞点仅小幅移动，始终未消除。根因是 C1 走廊链按“最远可见点”简化路线构建（S1 被简化成对角线），走廊只约束参考点；C2-A 的 body-clearance proxy 在 corridor 模式默认权重只有 1.0（legacy 5e4），求解器拿不到足够梯度把编队车体推离墙面。door_circumcircle 满足任务书“至少一条非平凡走廊解”的字面要求，但走廊模式在真实冻结场景上没有任何一条可发布解；提高覆盖率需要后续阶段改走廊构建/引导（例如不做最远可见点简化、或把车体膨胀纳入走廊链），不属于 C2-B 允许改动的范围。
6. **`trajectory.py` 兼容报告**：未发现具体冲突（`head_equality.txt` exit 0 + trajectory 测试 23/23 OK + 往返逐字节相等），故未触碰。
7. **前置**：V0/V1 用户已确认通过（本轮用户指令原文）；C2-A Codex 独立验收产物仍未出现，本报告不声称 C2-A 已被 Codex 验收。
8. **工作区含先前未提交的 V0/V1/Cargo HUD 改动**，与 C2-B 增量区分以 §1.1 表和七个 `source_diff_c2b_*.diff` 为准。
9. **历史 `timing_continuous` `KeyError: dynamic_peaks` 未修复**：本阶段未触碰 timing，保持原状。
10. **未启动 ROS/RViz**：launch/node 接线已写并有配置测试，真实 master 上的发布显示 NOT_RUN（属交互演示阶段，且需用户私有 master 约定）。

## 7. 分栏状态

| 分栏 | C2-B 状态 |
|---|---|
| 求解成功 | door corridor IPOPT 43.6 s / SLSQP 15.6 s `SOLVER_SUCCESS_AUDITED`；S1 corridor 求解成功但被连续包含拒绝 |
| 几何 | 走廊半空间两 solver 一致（1e-12）；door 连续包含认证；S1 宽度触限拒绝 |
| 动态 | door corridor dynamics 审核通过（`dynamic_audit_passed: true`）；动力学代码零改动 |
| 碰撞 | S3 corridor `MAP_OBB_COLLISION` 拒绝（中心安全不等于车体安全）；door 6428 检查通过 |
| 发布/显示 | pipeline 内 path/plan 发布门全 fail-closed 验证；ROS/RViz 真实发布 NOT_RUN |
| Codex 独立验收 | **PENDING**：本报告只是 Claude 侧证据 |

## 8. 证据文件

| 文件 | 用途 |
|---|---|
| `SUMMARY.md` / `results.json` | 本文件 / 机器可读 19 用例 + false_publications=0 + 测试记录 |
| `commands.log` | 精确命令与 exit code（测试四条任务书命令 + 管线三脚本） |
| `environment.txt` | Python/NumPy/SciPy/CasADi/IPOPT 探测 |
| `source_hashes.txt` / `git_state.log` / `head_equality.txt` | 哈希、HEAD 状态、禁改文件逐行 exit 0 |
| `source_diff_c2b_*.diff`（7 份） | contracts / corridor / optimizer / pipeline / node / launch / tests 的 C2-B 权威增量 |
| `run_pipeline_case.py` 等三个 `.sh` + `assemble_results.py` | 证据 harness 与批跑脚本 |
| `*_corridor.json` 等 19 个 JSON | 每用例完整记录（含 corridor_rounds/attempts/containment/κ 峰值） |
| `test_logs/` | 定向套件 105 全 OK + 四条任务书命令逐条 OK |
| `sample_inputs.json` | 六冻结场景 request/planner payload + door 合成场景输入 |
| `model_roundtrip.log` | 旧模型 JSON 往返逐字节相等 |

## 9. 停止点

C2-B 完成。**在此停止，等待 Codex 独立验收与用户下达 C3；不提交 Git、不推送、不启动 ROS/RViz。**
