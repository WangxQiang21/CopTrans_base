# C2-B 返工 01 总结：clearance 世界坐标修正与自主弯曲正例

**状态：返工代码、定向测试与证据已完成，工作区未提交，等待 Codex 复验。不提交 Git、不推送、不启动 ROS/RViz、不进入 C3。**

## 1. 范围、基线和工作区

| 项目 | 值 |
|---|---|
| 阶段 | C2-B 返工 01 |
| 任务书 | `plan/virtual_a_corridor_v01/CLAUDE_C2B_REWORK_01.md` |
| UTC run | `20261117T151325Z` |
| 证据生成时 HEAD | `9a19fd3e22d33188b13795f695757c3e4a36909f`（C1 提交） |
| 分支 | `feature/virtual-rigid-formation-a` |
| 原 C2-B 证据 | `evaluation/virtual_a_corridor/C2B/20261117T135647Z/`（只读，未改动） |

开始状态（`start_state.log`，2026-11-17T15:13:25Z）：HEAD 同上，`git status --short` 与结束状态一致（仅多出本返工证据目录），返工前 `optimizer.py` SHA-256 `6de24f4d…`、测试 SHA-256 `8cece474…`，原 C2-B `SUMMARY.md` `bf7f1b5d…`、`results.json` `1424f52b…`。结束状态（`end_state.log`）：HEAD 不变，优化器 `2cac388a…`、测试 `0f8a3bc3…`。**未新建分支、未 commit、未 push。**

实际修改文件只有任务书允许的两个：

| 文件 | 本次返工增量 |
|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py` | `source.diff` 前半：新增 `_clearance_world_body`，改写三处 clearance 路径 |
| `tests/test_virtual_a_corridor_optimizer.py` | `source.diff` 后半：新增 4 个定向测试 |

`source.diff` 只含本次返工增量：优化器以 `HEAD + C2-B source_diff_c2b_optimizer.diff` 重建，测试以 `C2-A new_test_c2a.diff + C2-B source_diff_c2b_tests.diff` 重建；两个重建基线的 SHA-256 与 `start_state.log` 的返工前值逐字节相同。C2-A/C2-B/V0/V1/Cargo HUD 的既有未提交改动没有并入本 diff，也没有被 reset/checkout/覆盖。

## 2. 问题与修复

优化器决策变量 `(x, y, psi)` 是起点局部编队量，而 clearance field、`direction_at` 和 CasADi 插值都定义在世界坐标。返工前 `_clearance_proxy_angles`、`_proxy_numeric_penalty`、`_build_casadi_nlp` 直接拿局部量查询世界地图，Codex 用起点 `(3,-2,0.7)` 与局部零状态复现出最大 `4.04907279485009 m` 的车心偏差。

统一世界坐标公式（三处路径完全一致）：

\[
p_F^W=p_0+R(\psi_0)p_F^L,\qquad \psi_F^W=\psi_0+\psi_F^L
\]

\[
p_i^W=p_F^W+R(\psi_F^W)r_i,\qquad
\theta_i^W=\psi_F^W+\operatorname{atan2}(a_i\kappa,1-b_i\kappa)
\]

实现方式：

- `_clearance_world_body(request, x, y, psi, kappa, offset)` 是唯一的小型内部辅助函数，供 `_clearance_proxy_angles` 与 `_proxy_numeric_penalty` 两个 NumPy 路径复用；
- `_build_casadi_nlp` 在 clearance 分支中用 CasADi 标量写同一公式，先做世界变换，再执行原有地图边界 clamp 与插值；clamp、权重、target、归一化和支撑距离公式均未改；
- 走廊半空间已有的世界→局部变换未改动；`kappa`、`a_i`、`b_i` 仍是编队局部切向量。

## 3. R3 定向测试与数值证据

`tests/test_virtual_a_corridor_optimizer.py` 新增 4 个测试（总数 20 → 24）：

1. `test_clearance_body_centres_use_world_frame`：起点 `(3,-2,0.7)`、局部零状态、三车偏移，独立手算世界车心与 fallback 世界航向，误差门 `1e-12 m`；
2. `test_numeric_and_casadi_clearance_objectives_agree_off_origin`：非零平移/姿态、非对称 32×36 field、非零曲率向量，NumPy 目标与 CasADi `f` 对比，门 `1e-9`（绝对或相对），并断言 clearance 项实际起作用；
3. `test_local_clearance_query_is_detectably_wrong`：保留返工前的局部坐标公式作对照，断言错误查询产生明显不同的惩罚和明显不同的查询点，防止测试复制同一错误；
4. `test_origin_start_clearance_semantics_are_unchanged`：起点 `(0,0,0)` 时新公式与旧局部公式逐点一致，惩罚与手算一致。

独立测量脚本 `measure_clearance_rework.py`（不属于生产代码）给出本次实际数值：

| 量 | 实测值 | 门槛 |
|---|---:|---:|
| 世界车心最大手算误差 | `0.0 m`（3 车全部精确一致） | `<= 1e-12 m` |
| fallback 世界航向最大误差 | `0.0 rad` | `<= 1e-12 rad` |
| 障碍方向存储语义 | 记录 field 返回方向，误差 `0.0` | 精确一致 |
| NumPy 目标 | `95.49606317619846` | — |
| CasADi 目标 | `95.49606317619858` | — |
| 目标绝对差 / 相对差 | `1.1369e-13` / `1.1905e-15` | `<= 1e-9`（容差 `9.5496e-8`） |
| clearance 项贡献 | `92.7341`（去掉 proxy 后目标 `2.7620`） | `> 1e-6` |
| 局部坐标负例惩罚差 | `0.242750` | `> 1e-3` |
| 局部/世界查询点最大分离 | `2.02485 m` | `> 1.0 m` |
| 原点回归车心/惩罚误差 | `0.0 / 0.0` | `<= 1e-12` |

世界车心实测查询点：`(3.7648421873, -1.3557823128)`、`(2.3557823128, -1.2351578127)`、`(3.6442176872, -2.7648421873)`，与任务书给出的 Codex 复现值一致（舍入前）。全部 9 个 gate 通过，详见 `clearance_measurements.json` 的 `gates`。

## 4. R4 自主弯曲端到端正例

请求完全由证据脚本构造后交给生产 `plan_virtual_a`，没有注入路径或手工 solver 成功：

- 世界起点 `(2.0, 0.0, 0.6)`；局部目标 `(8 m, 2 m, 0 rad)`；转换后世界目标 `(7.4733999724, 6.1678110170, 0.6)`；
- 三车初始航向均等于起点编队姿态 `0.6 rad`；
- 400×300 空旷栅格，0.05 m 分辨率，原点 `(-2,-5)`，`grid_sha256=6844baec…`；
- `guidance_mode=corridor`，其余生产车辆/时间/动态/碰撞/审核合同与默认值一致（`PlannerConfig` 摘要见 `curved_case_*.json`）。

两个 solver 各自独立跑完整 pipeline，结果一致：

| 分栏 | scipy (SLSQP) | ipopt |
|---|---|---|
| 求解器成功 | `status=SUCCESS`，6 次 attempt 均 `SOLVER_SUCCESS_AUDITED`，走廊残差 `0.0` | 同左 |
| 连续走廊包含 | `CORRIDOR_CONTAINMENT_VERIFIED`，8 样本、ε=`7.4824e-8 m`、max_point `-2.2954 m`、max_bound `-1.7728 m` | 同左 |
| 几何审核 | 通过（155 checks） | 通过（155 checks） |
| 动态审核 | 通过（909 checks） | 通过（909 checks） |
| 碰撞审核 | 通过（6482 checks） | 通过（6482 checks） |
| 发布资格 | `feasible=true, executable=true`，path+plan 均存在，`publication_eligible_by_evidence_gate=true` | 同左 |
| 路径长度 | `8.3620379168 m` | `8.3620378880 m` |
| 参考曲率峰值 | `0.3315314602 m^-1`（≥0.05） | `0.3315313764 m^-1`（≥0.05） |
| 三车曲率峰值 | `0.41290 / 0.24899 / 0.49596 m^-1` | `0.41289 / 0.24899 / 0.49596 m^-1` |
| 端点位置误差 | `1.2318e-10 m`（≤0.005） | `1.1033e-10 m`（≤0.005） |
| 端点姿态误差 | `2.9770e-12 rad`（≈`1.7e-10°`，≤0.2°） | `0.0 rad` |
| 刚性误差（最大成对） | `1.1102e-15 m`（≤1e-6） | `8.8818e-16 m`（≤1e-6） |
| 墙钟 | `13.22 s` | `30.69 s` |

走廊链为 `astar_primary` 单走廊（面积 96.32 m²），guide 为 `corridor_endpoint_polyline`（2 点），固定 8 区间分配全部落在走廊 0。该结果是**真正的弯曲方案**：横向 2 m 位移要求非零曲率，峰值 0.3315 m^-1 远高于 0.05 m^-1 数值噪声门，且双 solver 独立复现。完整请求、guide/chain/allocation、all attempts、containment、三层审核、端点和哈希见 `curved_case_scipy.json` / `curved_case_ipopt.json`。

失败路径未被关闭：本返工只有这一条新正例；历史 C2-B 的 S1/S2/S2_mirror/S3 等失败仍以原证据为准（无 path 无 plan），生产 pipeline 的 fail-closed 逻辑未改，管道测试 `test_fake_solver_success_never_becomes_an_executable_candidate` 等仍通过。

## 5. 测试命令（真实运行）

环境：仓库根目录，`source /opt/ros/noetic/setup.bash`、`source catkin_ws/devel/setup.bash`，`PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1`，`PYTHONPATH` 含源码/tests/仓库根。

| 命令 | 数量 | 耗时 | 退出码 |
|---|---:|---:|---:|
| `test_virtual_a_corridor_optimizer.py` | 24 | 5.765 s | 0（OK） |
| `test_virtual_a_optimizer.py` | 29 | 65.795 s | 0（OK） |
| `test_virtual_a_trajectory.py` | 23 | 0.458 s | 0（OK） |
| `test_virtual_a_pipeline.py` | 8 | 190.497 s | 0（OK） |

`test_virtual_a_pipeline.py` 包含 `test_six_frozen_scenes_pass_all_three_audits_autonomously`，继续证明 legacy_route 六冻结场景无回归。所有日志在 `test_logs/`，无 skip、无 FAIL、无 ERROR。未运行会覆盖历史输出的全量 runner。

## 6. 修正后的表述（R5）

- 原 C2-B 的 door corridor 是**有效的直线/近直线正例**（连续包含与三层审核通过，κ 峰值约 `1e-12`）；本次不再称它为“非平凡弯曲解”。
- 本次新增的世界起点 `(2,0,0.6)` → 世界目标 `(7.4734,6.1678,0.6)` 横向位移请求，才是 **C2-B 的自主弯曲正例**（κ 峰值 `0.3315 m^-1`，双 solver）。
- 六个冻结 corridor 场景仍为原报告的 **0/6**；本返工只修 clearance 坐标系并补正例，**没有声称任何成功率改善**，也没有重跑 19 用例矩阵。
- clearance 坐标错误的影响是**优化能力**（错误的梯度/车心查询会改变解），不是已证实的**安全漏检**；OBB 终审始终 fail-closed，原 C2-B 的失败用例均无 path 无 plan。
- `false_publications` 保持为 **0**：本返工 2 条 positive 均有完整 path/plan + 三层审核 + 连续包含；无任何“有 path/plan 但不满足发布门”的结果。

## 7. 原 C2-B 证据保护

`C2B/20261117T135647Z/` 共 74 个文件，全部 mtime `<= 2026-11-17T22:59:31+0800`，早于返工开始 `23:13:25+0800`；`find -newermt` 返回空；`SUMMARY.md`、`results.json` 的 SHA-256 与 `start_state.log` 逐字节一致。完整清单在 `original_c2b_manifest.sha256`。

## 8. 限制与 NOT_RUN

- **ROS/RViz 真实启动与发布显示：NOT_RUN**（本返工禁止启动；属交互演示阶段）。
- **原 C2-B 19 用例长矩阵：NOT_RUN**（任务书明确：只改 `optimizer.py` 与其测试时无需重复）。
- **C3 恢复策略：NOT_RUN**（禁止进入）。
- 弯曲正例使用空旷栅格，不证明窄道通过率；六冻结场景 0/6 仍是真实能力边界，本返工不修复、不进入 C3。
- 历史 `timing_continuous` `KeyError: dynamic_peaks` 未触碰，保持原状。
- 本报告只是 Claude 侧证据，**不声称 Codex `ACCEPTED`**。

## 9. 证据文件

| 文件 | 用途 |
|---|---|
| `SUMMARY.md` | 本文件 |
| `results.json` | 机器可读结果：测试计数/退出码、世界坐标误差、NumPy/CasADi 差、弯曲正例、false_publications |
| `commands.log` | 实际命令、UTC 开始/结束、退出码 |
| `source.diff` | 仅本次返工增量（optimizer.py + 测试） |
| `source_hashes.txt` | 最终源码/测试/脚本/请求/地图/结果哈希 |
| `curved_case_scipy.json` / `curved_case_ipopt.json` | 弯曲用例完整请求、guide/chain/allocation、attempts、containment、三层审核、端点/刚性/曲率、墙钟 |
| `clearance_measurements.json` | R1/R3 手算、目标一致性、负例、原点回归与 gates |
| `measure_clearance_rework.py` / `run_curved_case.py` | 只读证据脚本（非生产代码） |
| `test_logs/` | 四条任务书命令真实日志 |
| `start_state.log` / `end_state.log` | 开始/结束 HEAD、git status、关键哈希 |
| `original_c2b_manifest.sha256` | 原 C2-B 目录 74 文件清单 |

## 10. 停止点

C2-B 返工 01 完成。**在此停止，等待 Codex 复验；不提交 Git、不推送、不启动 ROS/RViz、不进入 C3。**