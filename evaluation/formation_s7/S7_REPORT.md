# S7 阶段审查报告：固定模式结构联合连续优化

## 结论

**S7 完成：新增 ROS 无关的 `formation_optimizer.py`，在 S6 审核通过的 `FormationContinuousPlan` 上固定每个 block 的 S5 mode-direction word，只优化低维 block 边界参数（平移运动角/长度、纯旋转 `dpsi_F`）。所有车辆中心继续由 `q_F + R(psi_F) r_i` 消元派生，不作为自由变量；A 平移块的 `alpha` 由车体航向精确消去，避免重复等式使 NLP 过约束；求解器返回后必须重新运行 S5 枚举/独立审核与 S6 计划审核，只有两道审核都通过才输出 `KINEMATIC_VALID`，否则拒绝或只保留重新审核过的 seed。9 项 S7 定向测试全部通过；S5 15/15、S6 15/15；完整 Python 回归 263 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误，没有新增失败。本阶段没有时间参数化、速度/加速度/jerk、驻留事件、碰撞证书或 `FEASIBLE`，没有修改 C++、单车 `ContinuousPlan`、控制器、NMPC、执行器或任何单车消息。**

## 任务编号

S7：固定模式结构的联合连续优化。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L733-L777)，阶段表见同文档 [S7 行](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L402)。

## Git 检查点

- S6 已完成提交：`408c283`（`feat/formation-continuous` 分支 HEAD）
- S7 分支：`feat/formation-nlp`（从 S6 提交创建）
- S7 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| S6 连续计划 API | 已核对 `FormationContinuousPlan`/`FormationBlock`/`FormationPathControl` 的字段、`qf_at`、`body_headings_at`、`control.start_qf/end_qf/pivot_*` 与失败语义 |
| S5 模式 API | 复用 `FormationKinematicBlock.from_primitive`、`enumerate_mode_assignments`、`audit_mode_assignment`；重构后先按固定 mode word 选择 assignment，再独立审核 |
| 刚体消元 | 中心仍由 `plan.offsets` 与 `q_F` 派生；NLP 变量中没有 `p_i`、没有第二套中心曲线 |
| 单车 `ContinuousPlan` | 未导入、未修改 `ContinuousPlan`/`MotionBlock`/`retime_continuous` 的结构或语义；仅参考 `continuous_optimizer.py` 的 CasADi/IPOPT 失败处理方式 |
| 既有消息/节点 | 未修改任何 `.msg`、ROS 节点、launch 或 YAML；S7 不发布话题 |
| CasADi/IPOPT | 本机可用；IPOPT 实际返回 `Solve_Succeeded`，NLP 变量与约束规模在报告中记录 |
| 地图版本/扫掠证书 | 沿用 S6 的 `map_version_checked=false` 限制；S7 的碰撞预筛不是证书，`collision_certificate=false` |

## 修改文件

新增：

1. [formation_optimizer.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_optimizer.py)：`FormationOptimizerConfig`、`FormationOptimizationReport`、`FormationOptimizationResult`、固定模式结构 NLP、求解、独立审核与 seed 回退。
2. [test_formation_optimizer.py](../../tests/test_formation_optimizer.py)：9 项 ROS 无关测试。
3. [logs/formation_optimizer_tests.txt](logs/formation_optimizer_tests.txt)
4. [logs/formation_kinematics_tests.txt](logs/formation_kinematics_tests.txt)
5. [logs/formation_geometry_tests.txt](logs/formation_geometry_tests.txt)
6. [logs/unit_tests.txt](logs/unit_tests.txt)
7. [logs/py_compile.txt](logs/py_compile.txt)
8. [logs/git_diff_check.txt](logs/git_diff_check.txt)
9. [S7_REPORT.md](S7_REPORT.md)

未修改任何既有文件；没有 C++ 或消息变更，因此本阶段不需要 catkin 构建。

## 固定模式结构 NLP

### 决策变量

每个 block 只暴露一个低维表示：

- 平移 block：formation 运动角 `alpha` 与正长度 `L`；`qf_next = qf + L (cos alpha, sin alpha)`。
- 纯旋转 block：绕显式 formation-frame pivot 的有符号 `dpsi_F`；端点由 pivot 的刚体旋转重算，`psi_F_next = psi_F + dpsi_F`。

变量初值全部取 S6 seed；变形预算：

- `alpha` 相对 seed 的角度带 `±translation_angle_bound_rad`（默认 `π/2`）；
- `L` 在 `[max(min_block_length_m, seed_L*length_lower_factor), seed_L*length_upper_factor]`；
- `dpsi_F` 相对 seed 的带 `±rotation_angle_bound_rad`（默认 `π/3`），并保持 seed 的符号，不允许把旋转方向翻号。

`θ_i(λ)`、`κ_i(λ)`、`φ_i(λ)` 不作为独立控制节点：它们由固定 S5 mode word 与 block 恒定控制派生。这是本阶段相对文档变量表的**有意收窄**，见“未执行项目与边界”。

### 消元与硬约束

1. **A 平移角消元**：若平移 block 中存在 Ackermann segment，则 `alpha = θ_i + offset + branch` 精确成立；`alpha` 不再是变量，而是（可能依赖先前 `dpsi` 的）表达式。多个 A 车辆必须在 seed 上给出同一角度（模 `2π`），否则 `INCONSISTENT_A_TRANSLATION_ANGLES`。消元后只保留长度变量，避免“两个变量、四个等式”的伪过约束。
2. **O 平移斜角**：`direction=+1` 时硬约束 `cos(alpha-θ_i) >= cos(phi_max)-1e-10`；`direction=-1` 时 `-cos(alpha-θ_i) >= cos(phi_max)-1e-10`。反向 segment 的 signed direction 与 `allow_reverse` 由 S5 审核再次确认。
3. **纯旋转 A**：`θ_i = tangent_base + psi_F + offset`（forward/reverse 分支按 seed 选择）；每个 block 只加一次等价切线等式。所有车辆航向与 `psi_F` 共享同一组 `dpsi` 增量，等式在 seed 上先验证；不一致时 `A_ROTATION_TANGENT_SEED_MISMATCH`。
4. **纯旋转 S**：半径必须在 `pivot_tolerance_m` 内，方向严格等于 `sign(dpsi_F)`；非 pivot 车伪 Spin 在构建阶段拒绝。
5. **曲率**：旋转非 pivot A 的 `|κ| = |dpsi_F| / radius` 必须不超过 `1/r_min_ackermann`；不满足时 `CURVATURE_LIMIT`。
6. **终端**：`final_x`、`final_y`、`final_psi`（按 seed 的 `2π` 分支取最近等价角）是硬等式。
7. **终端车体航向**：每个 S5 旋转 segment 对每车航向的增量都等于 `dpsi_F`，因此 `θ_i(final)` 与 `psi_F(final)` 只差一个常量。`final_psi` 加上 seed 审核已确认的常量关系已经固定所有非 `goal_heading_free` 的终端 `θ_i`；若再加每车一条等式只会复制同一约束。构建时改为显式一致性检查 `FINAL_HEADING_GOAL_INCONSISTENT`，不添加重复等式。`goal_heading_free` 仍然只释放该车终端航向。
8. **中心刚性**：NLP 中不存在车辆中心变量，`p_i` 永远从 `q_F` 与 offset 重新计算；求解结果重构时也不独立移动任何车辆中心。

### 目标函数

```text
path_measure = Σ 平移 block 长度 + Σ sign(seed_dpsi) * dpsi_F
adjustment   = Σ (alpha-alpha_seed)^2 + Σ ((L-L_seed)/L_seed)^2
             + Σ (dpsi_F-dpsi_F_seed)^2
objective    = objective_length_weight * path_measure
             + objective_adjustment_weight * adjustment
```

默认 `length_weight=1.0`、`adjustment_weight=0.10`；两者均为代理目标，不是时间、净空或安全最优证明。目标中不含障碍物 soft surrogate，S7 不把任何数值目标当安全证书。

### 求解器

```text
opt.solver("ipopt", {"print_time": False, "expand": True},
           {"print_level": 0,
            "max_iter": max_iterations,          # 默认 120
            "max_wall_time": time_limit_s,       # 默认 1.0 s
            "tol": 1e-9,
            "constr_viol_tol": 1e-9,
            "dual_inf_tol": 1e-9,
            "hessian_approximation": "limited-memory"})
```

`opt.minimize(objective)` 在构建阶段显式调用。求解异常、非收敛或硬残差超限时返回结构化 `FORMATION_NLP_INFEASIBLE`/`NLP_HARD_RESIDUAL_EXCEEDED`；默认 `allow_seed_fallback=True` 时只有重新审核通过的 seed 可以作为 `SEED_RETAINED` 返回。

### 求解后独立审核

`_reconstruct_plan()` 不信任任何求解器数值：

1. 用求解值重建 `FormationPathControl` 与 S5 `FormationKinematicBlock`。
2. 每个 block 重新调用 `enumerate_mode_assignments`，在成功 assignment 中选择与 seed 完全相同的 `(vehicle_id, mode, direction)` word；找不到则 `FIXED_MODE_SCHEDULE_MISSING`。
3. 对被选 assignment 调用 `audit_mode_assignment`，重算中心、航向、曲率、斜角、reverse 和终点；失败则 `MODE_AUDIT_FAILED`。
4. 组装新的 `FormationContinuousPlan`，调用 S6 `audit_continuous_plan`：重算刚体中心、逐 block 审核、采样刚性残差、终端和可选地图扫掠初筛。
5. 只有 S6 审核成功才把 certification 从 `GEOMETRY_VALID` 替换为 `KINEMATIC_VALID`；否则不输出求解候选。

因此求解器 `Solve_Succeeded` 永远不会直接成为最终结果；失败时：
- `allow_seed_fallback=True`：重新审核 seed 后返回 `SEED_RETAINED`（`status=KINEMATIC_VALID`，`seed_retained=true`，`optimized_candidate_accepted=false`）；
- `allow_seed_fallback=False`：返回 `REJECTED`、`plan=None`、`certification=NONE`。

### 状态语义

`FormationOptimizationResult` 与 `FormationOptimizationReport` 恒有：

```text
executable=false
trajectory_published=false
independent_timing_audit=false
feasible=false
collision_certificate=false
time_parameterization=false
centers_eliminated=true
mode_structure_fixed=true
final_swept_audit_required=true
```

没有 `FEASIBLE` 状态；`FORMATION_NLP_INFEASIBLE` 是拒绝原因而不是可行声明。

## 本阶段自检发现的实现问题

以下问题在写测试前的数值核查中发现并修正，属于 S7 代码本身的缺陷，不是既有模块回归：

1. **目标函数未接入求解器**：`_build_nlp` 计算了 `objective`，但最初没有调用 `opt.minimize(objective)`，IPOPT 实际求解的是零目标可行性问题，返回点与迭代初值/求解器选项相关。已显式调用 `opt.minimize(objective)`；测试用“目标改善量必须大于 0.5”固定该行为。
2. **A 平移重复等式导致 NLP 过约束**：原实现为每个 A 车辆各加一条 `alpha-θ_i-offset-branch=0`，两个变量四个等式使 IPOPT 报 `Not_Enough_Degrees_Of_Freedom`。现按块消元 `alpha`，只保留长度变量；纯旋转的等价切线等式也只加一次。
3. **终端航向等式与 `final_psi` 重复**：每车终端航向变化与 `psi_F` 变化来自同一组 `dpsi_F`，再加每车一条等式会复制约束。现构建时验证 seed 终端航向与 goal 一致，由 `final_psi` 固定全部非释放航向。

上述修正不改变 S5/S6 的审核契约，只修正 S7 的 NLP 构建。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `/usr/bin/time -v env OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_optimizer.py' -v` | 9/9 通过；测试计时 0.408 s；墙钟 0.60 s；最大 RSS 207484 KiB | 0 |
| 同上，`-p 'test_formation_kinematics.py'` | 15/15 通过；0.008 s；墙钟 0.07 s；最大 RSS 18548 KiB | 0 |
| 同上，`-p 'test_formation_geometry.py'` | 15/15 通过；0.074 s；墙钟 0.24 s；最大 RSS 56836 KiB | 0 |
| 同上，`-p 'test_*.py'` | 263 项，262 通过、1 个既有错误；8.155 s；墙钟 8.45 s；最大 RSS 233856 KiB | 1 |
| `python3 -m py_compile ...` | 优化器、测试及 S5/S6 模块编译成功 | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 263 tests in 8.155s
FAILED (errors=1)
```

263 = S6 的 254 项 + S7 新增 9 项。没有新增跳过、弱化或删除任何既有测试；未修改 `test_continuous_retiming.py` 或相关重定时模块。

## 定向测试与验收对应

| 文档验收要求 | 实际测试 | 结果 |
|---|---|---|
| 硬约束残差数值报告 | `test_nlp_improves_objective_and_returns_kinematic_only` 断言 `hard_residual<=1e-6`；实际成功场景为 `1.3e-15`、`6.9e-11`、`1.8e-14` 量级 | 通过 |
| 各车曲率/斜角合法 | 求解值先进入 S5 `enumerate_mode_assignments` 与 `audit_mode_assignment`；O 不等式和旋转曲率在 NLP 中为硬约束，审核再次重算 | 通过 |
| 几何独立审核通过 | 每个成功候选都调用 S6 `audit_continuous_plan`；测试断言 `audit.success` | 通过 |
| 固定模式结构 | `test_rotation_translation_chain_keeps_fixed_mode_word` 断言 seed 与候选的 `(vehicle_id, mode, direction)` word 完全相同，反向 O 仍为 `direction=-1` | 通过 |
| A 平移块不产生伪过约束 | `test_ackermann_translation_angle_is_eliminated_not_overconstrained` 断言变量数为 1、`Solve_Succeeded`、无 DoF 错误 | 通过 |
| 纯旋转结构可处理 | `test_pure_rotation_structure_is_supported_and_reaudited`：pivot `S`、非 pivot `A`，`rotation` block 通过 S5/S6 审核 | 通过 |
| 求解器成功 ≠ 最终可行 | `test_post_solve_collision_audit_rejects_solver_candidate`：求解器 `Solve_Succeeded` 且 `converged=true`，但候选在障碍物地图上 S6 审核失败；默认返回 `SEED_RETAINED`，`allow_seed_fallback=False` 返回 `REJECTED`/`plan=None` | 通过 |
| seed 自身失败不启动求解器 | `test_seed_audit_failure_never_runs_the_solver`：地图障碍使 seed 审核失败，报告 `attempted=false`、`solver_status=not_run`、`iterations=0` | 通过 |
| 不支持结构 fail-closed | `test_unsupported_fixed_mode_structure_fails_closed`：平移 `S`、平移中改航向均被 `_check_seed_structure` 拒绝 | 通过 |
| 配置/类型失败闭合 | `test_invalid_inputs_and_configs_fail_closed`：错误 plan/config/limits、越界角度/长度因子/布尔标志/负权重全部抛 `FormationValidationError` | 通过 |
| 不输出 `FEASIBLE` | 所有成功/失败 status 断言 `executable=false`、`trajectory_published=false`、`independent_timing_audit=false`、`feasible=false`；`test_status_names_are_explicit_and_never_feasible` | 通过 |
| 单车版本仍运行 | 完整回归无新增失败；既有 `dynamic_peaks` 错误保留 | 通过 |

### 程序化场景结果摘录

| 场景 | seed 审核 | 求解器 | 目标 | 结果 |
|---|---|---|---|---|
| 两平移 detour，heading 0.3 | 通过 | `Solve_Succeeded`，8 次迭代，4 变量 | 3.1241 → 2.1530，改善 0.9711 | `OPTIMIZED`，S5/S6 审核通过 |
| 两平移 ±0.5，heading 0 | 通过 | `Solve_Succeeded`，6 次迭代，4 变量 | 2.2361 → 2.0380，改善 0.1981 | `OPTIMIZED`，候选通过审核 |
| 单段 A 平移 2 m | 通过 | `Solve_Succeeded`，0 次迭代，1 变量 | 2.0 → 2.0 | `SEED_RETAINED` / `NO_IMPROVEMENT`（无回退时 `REJECTED`） |
| 纯旋转 0.5 rad | 通过 | `Solve_Succeeded`，0 次迭代，1 变量 | 0.5 → 0.5 | `SEED_RETAINED` / `NO_IMPROVEMENT` |
| 旋转 0.3 + 两平移（含反向 O） | 通过 | `Solve_Succeeded`，8 次迭代，5 变量 | 2.3881 → 2.3145，改善 0.0736 | `OPTIMIZED`，固定 mode word 保持 |
| 同 detour + 障碍物 | 通过 | `Solve_Succeeded` 后候选审核失败 | — | `SEED_RETAINED` 或 `REJECTED`，`COLLISION_PRECHECK_FAILED` |

## 未执行项目与边界

1. **变量范围窄于文档**：只优化 block 边界参数；没有把 `x_F(λ), y_F(λ), ψ_F(λ)`、`θ_i(λ)`、`κ_i(λ)`、`φ_i(λ)` 展开为离散控制节点。A/O/S 控制仍为 block 恒定常数，`θ_i` 由固定模式 word 传播。
2. **A 平移角不可调**：A 必须精确对齐车体航向，消元后 `alpha` 只随先前 `dpsi_F` 变化；不释放 A 的微小转向偏差。
3. **没有时间参数化**：没有 `λ(t)`、速度、加速度、jerk、转向速率/加速度、轮速、模式切换 dwell 或统一时间网格；S8 负责。
4. **没有碰撞证书**：NLP 不含障碍物约束或 surrogate；求解后最多执行 S6 采样扫掠初筛，`collision_certificate=false`，S9 必须独立审核。
5. **局部求解**：IPOPT 只证明一阶 KKT 与硬残差；不证明全局最优、整条 primitive 链完备性或跨模式组合最优。
6. **变形预算是有界启发式**：角度/长度因子、时间/迭代预算、`adjustment_weight` 都不是安全参数；候选可能因预算被保守拒绝。
7. **seed 回退不是新候选**：`SEED_RETAINED` 只重新审核原 seed，其 `dynamic_profile` 保持 S6 值；报告明确 `optimized_candidate_accepted=false`。
8. **没有地图版本/动态障碍/地图更新协议**：沿用 S6 的 `map_version_checked=false` 限制；障碍扫掠只针对调用方提供的一次性地图。
9. **没有 ROS/回放/RViz**：S7 是纯 Python 库，不发布话题、不写轨迹、不执行。
10. **审核失败只返回首个失败项**：与 S5/S6 一致，尚未聚合全部 block/车辆失败。
11. **`goal_heading_free` 只覆盖终端航向**：不释放中心、`psi_F` 或中间 block 航向；由 S5/S6 契约继续约束。
12. **NLP 常量约束检查依赖 `hard_residual_tolerance`**：常量违反会以 `NLP_CONSTANT_CONSTRAINT_VIOLATION` 拒绝；该容差不是执行安全容差。

## 安全性审计

- **中心唯一来源**：NLP 和重构均无车辆中心变量；所有 `p_i` 从 `q_F + R(psi_F) r_i` 与固定 offset 派生。
- **不默认 `theta_i=psi_F`**：每车航向按 S5 固定 mode word 传播；旋转 A 要求切线对齐，S 要求半径为零；`theta_i` 与 `psi_F` 的常量关系在 seed 上显式验证，不是默认相等假设。
- **A/O/S 不混淆**：A 平移角消元、O 正/反向 cosine 约束、S 半径/方向检查均在 NLP 中；重建时 S5 枚举和独立审核再次检查。
- **反向不伪装**：`direction=-1` 保留，O 反向使用 `-cos(alpha-theta)`；测试在旋转+平移链中断言 `(vehicle_id=1, direction=-1)` 仍存在。
- **求解器不越权**：`Solve_Succeeded` 后必须通过 S5 assignment 选择/审核和 S6 计划审核；候选失败时不会进入结果，只有重新审核的 seed 可回退。
- **失败闭合**：非法输入、错误配置、无模式 schedule、模式 word 不匹配、常量约束违反、硬残差超限、非收敛、审核失败均返回结构化拒绝。
- **不冒充可执行**：结果最多 `KINEMATIC_VALID`；所有 status 有 `executable=false`、`trajectory_published=false`、`independent_timing_audit=false`、`feasible=false`；没有 `FEASIBLE`。
- **不污染单车**：未修改 `core.py`、`continuous_geometry.py`、`continuous_retiming.py`、`quality.py`、单车 `.msg`、C++ 或 ROS 节点；只参考 `continuous_optimizer.py` 的 CasADi 使用方式。
- **测试可复现**：成功/失败场景的 seed、偏移、heading、障碍物坐标和预算均写死在测试文件中；碰撞拒绝测试先断言 seed 审核通过，再断言求解成功后候选被拒。

## 存在风险

1. **block 边界参数化不覆盖连续控制形状**：即使 S6 几何审核通过，S8 仍可能在时间参数化时发现恒定控制段的速度/加速度/转向速率不可行；S7 的 `KINEMATIC_VALID` 不是时间可行证明。
2. **A 消元使 A 段完全刚性对齐**：若真实执行需要 A 段微小转角修正，S7 不会搜索该方向；需要 S8/S9 或后续控制节点扩展。
3. **终端航向由 `final_psi` 隐含固定**：该推理依赖 S5 旋转 segment 对每个车辆航向施加同一 `dpsi_F` 的结构；若未来引入每车独立旋转增量，必须恢复显式终端航向约束。
4. **NLP 非凸且依赖初值/预算**：不同 IPOPT 版本、线性求解器或时间预算可能返回不同局部点；硬残差和独立审核会拦截不可行点，但不能保证每次都找到改善解。
5. **障碍物不在 NLP 中**：优化路径可能主动穿过障碍物，只能在 S6 扫掠初筛中被拒绝；默认 seed 回退会放弃改善，不提供绕障优化。
6. **采样扫掠的离散 padding 不是证书**：窄通道、薄障碍物或动态障碍可能漏检；S9 必须独立、更细地审核。
7. **变形预算固定**：角度/长度带和 `adjustment_weight` 是启发式；更大编队或多 block 链可能因预算过紧而只能 `SEED_RETAINED`。
8. **没有多候选联合优化**：S7 一次只优化传入的一个 S6 plan；S6 master 的其余候选仍保持原几何，未做跨候选比较。
9. **`FIXED_MODE_SCHEDULE_MISSING` 覆盖不足**：正常 S6 seed 的 A/O/S word 在预算内通常可重建；该分支主要由防御性测试覆盖，真实触发场景需要更复杂的几何/模式组合。
10. **报告统计有限**：`FormationOptimizationReport` 记录单次求解的变量数、迭代、耗时、残差和目标改善，但没有跨调用累计成功率/耗时统计；S9/S10 再做实验汇总。

## 完成判据与 S8 前置条件

- 固定模式结构 NLP：**已实现并测试**；模式 word 不变量由 S5 枚举/审核和测试双重确认。
- 中心消元与硬约束：**已实现**；NLP 无中心变量，A/O/S、曲率、斜角、终端 `q_F` 为硬约束。
- 求解失败/残差超限策略：**已实现**；结构化拒绝或重新审核 seed，绝不降低车间距硬约束。
- 求解后独立审核：**已实现并测试**；碰撞拒绝场景证明 `Solve_Succeeded` 不是最终可行。
- 硬约束残差、耗时、变量数记录：**已实现**；报告字段可序列化。
- 只输出 GEOMETRY/KINEMATIC、无 FEASIBLE/时间轨迹：**已由代码和状态测试确认**。
- 单车回归：**无新增失败；既有 `dynamic_peaks` 错误保留**。
- S8 前置条件：必须只接受 S7 输出的 `KINEMATIC_VALID`（或明确标记的 `SEED_RETAINED`）几何；必须建立单一公共 `λ(t)`、全车速度/加速度/jerk/转向速率/轮速上限、模式切换全停与静止事件、共享 `times_from_start`；S8 不得把 S7 的几何审核当成时间可行证明；碰撞仍不是证书，S9 必须独立扫掠；不得修改单车 retiming 契约。