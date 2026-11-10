# S6 阶段审查报告：Python 编队连续几何 + 候选管理

## 结论

**S6 完成：新增 ROS 无关的 `FormationContinuousPlan`、`FormationBlock`、`FormationPathControl` 和 `FormationModeMaster`。所有车辆中心只从同一编队参考曲线 `q_F(lambda)` 与固定 offset 派生；每个 block 仍调用 S5 枚举并逐方案独立审核；终端拟合只修改最后一段 `q_F` 端点，纯旋转可延长 `dpsi_F`，但不独立移动任何车辆中心；局部优化前加入多车 OBB 扫掠初筛；候选 master 固定 K、按完整 key 去重、保留 incumbent 并记录拒绝原因。15 项 S6 定向测试全部通过，S5 修正后 15/15 通过；完整 Python 回归 254 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误，没有新增失败。**本阶段没有 NLP、时间参数化、速度/加速度/jerk、驻留事件、碰撞认证或 `FEASIBLE`；没有修改 C++、单车 `ContinuousPlan`、控制器、NMPC、执行器或单车消息。

## 任务编号

S6：Python 连续几何 + 候选管理。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L690-L729)，阶段表见同文档 [S6 行](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L401)。

## Git 检查点

- S5 已完成提交：`3952484da36508af57350f100a7a3ca4554530c7`
- S6 分支：`feat/formation-continuous`（从 S5 提交创建）
- S6 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| S4 种子 | `FormationPlanSeed` 含 `plan_id`、`formation_id`、`formation_start/goal`、`primitives`；`FormationPrimitive` 的 `TRANSLATION/ROTATION/COMPOSITE` 与显式 pivot 已冻结 |
| S5 模式 API | 已核对 `FormationKinematicBlock.from_primitive`、`enumerate_mode_assignments`、`audit_mode_assignment`、`VehicleModeSegment` 的字段和独立审核语义 |
| S1 刚体 API | 复用 `FormationPose`、`VehicleOffset`、`FormationModel.centers()`；S6 没有新增中心自由度或第二套刚体公式 |
| S2 碰撞 API | 复用 `FormationCollisionChecker.swept_check`、`FormationState`、`SweepPolicy`；S6 只做预筛，不把预筛当安全证书 |
| 单车 `ContinuousPlan` | 未导入、未修改 `ContinuousPlan`/`MotionBlock`/`retime_continuous` 的结构或语义 |
| 既有消息/节点 | 未修改任何 `.msg`、ROS 节点、launch 或 YAML；S6 不发布话题 |
| 地图版本字段 | `FormationPlanSeed.msg` 当前没有地图版本/签名；S6 支持 duck-typed `map_signature` 并在预期签名缺失时明确记录 `map_version_checked=false`，不伪造检查通过 |

## 修改文件

新增：

1. [formation_geometry.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_geometry.py)：`FormationPathControl`、`FormationBlock`、`FormationContinuousPlan`、S4 种子构建、终端拟合、独立计划审核、多车碰撞初筛。
2. [formation_master.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_master.py)：`FormationModeMaster`、固定 K、候选去重、incumbent 保留、全部拒绝原因。
3. [test_formation_geometry.py](../../tests/test_formation_geometry.py)：15 项 ROS 无关测试。
4. [logs/formation_geometry_tests.txt](logs/formation_geometry_tests.txt)
5. [logs/formation_kinematics_tests.txt](logs/formation_kinematics_tests.txt)
6. [logs/unit_tests.txt](logs/unit_tests.txt)
7. [logs/py_compile.txt](logs/py_compile.txt)
8. [logs/git_diff_check.txt](logs/git_diff_check.txt)
9. [S6_REPORT.md](S6_REPORT.md)

修改：

10. [formation_kinematics.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py)：修正非零 `psi_F` 下平移运动角和纯旋转 tangent 的世界航向偏移；见下文“S5 集成修正”。
11. [test_formation_kinematics.py](../../tests/test_formation_kinematics.py)：新增非零 `psi_F` 的平移/倒车/旋转回归测试。
12. [S5_REPORT.md](../formation_s5/S5_REPORT.md)：追加 S6 集成修正说明，保留 S5 原始证据。

未修改任何既有 `.msg`、C++、ROS 节点、launch、YAML、单车优化器或单车测试；没有 C++/消息变更，因此本阶段不需要 catkin 构建。

## 连续几何数据结构

### `FormationPathControl`

一个 block 只保存一个结构化编队参考控制：

- `TRANSLATION`：`parameter_length = ||(dx,dy)||`，`qf(lambda)` 在 formation frame 中线性插值，`psi_F` 恒定。
- `ROTATION`：`parameter_length = |dpsi_F|`，`pivot_x/pivot_y` 是显式 formation-frame pivot；`qf(lambda)` 由 pivot 的刚体旋转重新计算，端点和 pivot 必须一致。
- `dqf_dlambda()` 给出精确一阶导；平移为常向量，旋转为 `-(dpsi/L)R(psi_F)Jp`（含展开的 `psi_F`，不先 `wrap`）。S6 测试用中心差分核对。

### `FormationBlock`

```text
geometry_type, parameter_length,
formation_controls = (FormationPathControl,),
vehicle_mode_segments = 每车一个 S5 VehicleModeSegment
```

构造时 fail-closed 校验：

- 只允许 `translation`/`rotation`；`composite` 不支持。
- 控制长度必须等于 block 长度；每车 segment 起终 station 必须覆盖 `[0,L]`。
- 车辆 ID 唯一，所有 segment 的 mode/direction/control/theta 保留 S5 结果。
- 不保存独立车辆中心曲线；`qf_at(lambda)` 和 `body_headings_at(lambda)` 是唯一连续真值。

### `FormationContinuousPlan`

保存 `plan_id`、`formation_id`、显式 `start/goal`、固定 `offsets`、每车起终 `theta`、blocks、`goal_heading_free`、`allow_reverse`、`dynamic_profile` 和 `certification`。构造时校验：

- `plan_id` 是 uint32，`formation_id` 非空，offset 由 `FormationModel` 校验并排序。
- 起终航向映射必须覆盖且只覆盖当前车辆 ID。
- 首 block 起点等于 `plan.start`，相邻 block 的 `q_F` 端点在 `1e-7` 内连续。
- `certification` 只允许 `GEOMETRY_VALID` 或 `KINEMATIC_VALID`。
- `executable=False`、`trajectory_published=false`。

## 从 S4 种子构建

`build_continuous_plans(snapshot, seed, ...)` 的顺序：

1. 校验 `seed.plan_id`/`formation_id` 与请求一致；header `frame_id=="map"`、stamp 非零；种子起终 `q_F` 与请求在 `1e-7 m`/`1e-7 rad` 内一致。
2. 校验 primitive 非空；拒绝 `COMPOSITE`；跳过零长度平移/旋转；拒绝非有限值。
3. 从请求起点按顺序调用 `FormationKinematicBlock.from_primitive`，每个 primitive 的终点成为下一个起点，block 间不重置 `psi_F`。
4. 每个 block 调用 S5 `enumerate_mode_assignments`，对每个候选调用 `audit_mode_assignment`；只有审核成功的 S5 方案才变成 block choice。
5. 用有界 beam 组合各 block 的 S5 方案，最多保留 `max_schedule_combinations` 个组合；初始阶段不把终点航向硬塞给最后一个 block，因为纯旋转终端拟合会改变 `dpsi_F` 和每车终端 `theta`。
6. 对每个组合只拟合最后一个 block：
   - 若已精确到终点：保留原 block 的 S5 方案；
   - 最后是平移：保持 `psi_F` 不变，把端点吸收到 `goal`，长度重新计算；
   - 最后是旋转：保持显式 pivot，把 `dpsi_F` 延长/缩短到 `goal.psi`，并要求按 pivot 算出的端点确实命中 `goal`；
   - 每次几何改变后重新枚举并独立审核 S5 模式，不独立移动任何车中心。
7. 若终点航向不匹配且不能通过最后一段几何改变修复，返回 `GOAL_HEADING_MISMATCH`；若 pivot 不能达到终点，返回 `TERMINAL_FIT_FAILED`；若没有审核通过的终端模式，返回 `MODE_ASSIGNMENT_FAILED`。
8. 结果最多 K 个唯一 `FormationContinuousPlan`，按 `plan_candidate_score` 排序；初始认证固定为 `GEOMETRY_VALID`。

地图版本说明：当前 `FormationPlanSeed.msg` 没有地图版本字段。S6 支持调用方传入 `expected_map_signature` 以及 seed 对象可选的 `map_signature`；若 seed 没有签名，则构建可以继续，但 `map_version_checked=false` 并写入 detail；若两者都存在且不同，返回 `MAP_VERSION_MISMATCH`。S6 不会把“没有字段”说成“检查通过”。

## 独立计划审核

`audit_continuous_plan(plan, snapshot, ...)` 不信任保存的 centers、segment 端点或 terminal 状态：

1. 复核 `plan_id`、`formation_id`、offset 版本、起点/终点、起终航向、`allow_reverse` 和 `goal_heading_free` 与请求完全一致。
2. 按 blocks 顺序从 `q_F` 重建 `FormationKinematicBlock`；检查 block 车辆集合、station 覆盖和 `q_F` 连续性。
3. 把保存的每车 segments 重新包装为 `FormationModeAssignment`，从上一 block 的终点航向传播本 block 起始 `theta`，逐车调用 S5 `audit_mode_assignment`。这会重新检查 A 的 `κ`、O 的 `φ`、S 中心不动、reverse 策略、曲率、切线对齐和终点航向。
4. 每个 block 在 17 个 `lambda` 采样上从 `q_F` 重新计算所有车辆中心，检查 `FormationModel.rigidity_residual` 不超过 `geometry_tolerance_m`；S6 不存储第二套中心曲线。
5. 检查每个 segment 的 `theta_start` 与本 block 起点航向、`theta_end` 与枚举终点一致；block 间不做隐式 `theta_i=psi_F` 替换。
6. 检查最后一段 `q_F` 与请求 goal 的 `xy/yaw` 容差；非 `goal_heading_free` 车辆的终端 `theta` 必须匹配请求，`goal_heading_free` 只跳过该车终端航向，不跳过中心。
7. 若有地图，则执行多车扫掠初筛；失败时保留失败 block、车辆、碰撞原因、formation `qf` 和采样上下文。
8. 成功仍只标记 `KINEMATIC_VALID`，detail 为 `KINEMATIC_ONLY_UNCERTIFIED_SAFETY`；`collision_prechecked` 只表示做过预筛，`independent_timing_audit=false`。

当前审核在首个失败项返回，不做所有车辆/block 的失败聚合；拒绝记录仍保留 context。

## 多车扫掠初筛

`precheck_plan_collision()`：

- 对每个 block 按保守角点位移上界决定采样数：
  - 平移用 `L`；
  - 旋转用 `(max ||offset|| + max 车体半对角) * |dpsi|`。
- 每 block 至少包含首尾，最多 `max_samples_per_block`（默认 20000）；超预算 fail-closed。
- 每个样本从 block 的 `qf_at(lambda)` 和 `body_headings_at(lambda)` 生成 `FormationState(station, qf, body_headings)`；station 是全局单调 block 进度，不是车辆弧长。
- 调用 S2 `FormationCollisionChecker.swept_check`；触碰、地图 OBB、车车 OBB 都按碰撞处理。
- 失败时记录 `failing_block_index`、车辆、原因、`qf` 和采样；报告 reason 可能是 `MAP_OBB_COLLISION` 或 `VEHICLE_OBB_COLLISION`。

该初筛是采样 + S2 离散 padding 的保守预检，不是 S9 的最终独立碰撞证书，也不是时间/执行器安全证明。

## `FormationModeMaster`

`FormationModeMaster` 采用与单车 `ModePathMaster` 相同的原则：

- 配置固定 `max_candidates`（默认 K=4）、组合上限、每 block schedule 上限、margin、扫掠间距和采样上限。
- `consider(plan, snapshot, ...)`：
  1. 计算 candidate key；重复则记 `DUPLICATE_CANDIDATE`；
  2. 独立 `audit_continuous_plan`；失败记审核 reason；
  3. 若配置要求碰撞预筛而地图缺失，拒绝；
  4. 只有审核成功的 plan 才可替换为 `KINEMATIC_VALID` 并成为 `FormationCandidate`；
  5. 满 K 时保留 score 更小的 incumbent，淘汰最差候选并记录 `EVICTED_BY_BETTER_CANDIDATE`；不更优则记 `CANDIDATE_LIMIT_REACHED`。
- `consider_build` 同时记录 build 阶段的全部 rejection（terminal/mode/seed），不只记录最终候选失败。
- `result()` 只在有候选时返回 `KINEMATIC_VALID`；无候选返回 `REJECTED`/`NONE`，不返回 `FEASIBLE`。
- 去重 key 包含：编队 block 几何、每车 mode/direction/control/theta/travel、起终航向、offset 实际值、`goal_heading_free`、`allow_reverse`、`dynamic_profile`；不包含单车 `Action` signature，也不包含可变的 source 文本。
- 所有 candidate/result status 恒有 `executable=false`、`trajectory_published=false`、`independent_timing_audit=false`。

## S5 集成修正

S6 第一次用非零 `psi_F` 的 S4 种子做整链测试时，`MODE_DISPLACEMENT_MISMATCH` 暴露了 S5 的 4 处世界航向偏移：

- 平移 `motion_angle` 原为 formation-frame `atan2(local_dy,local_dx)`，没有加 `psi_F`；
- 旋转 forward/reverse tangent 原为 formation-frame `atan2(rx,-ry)`，没有加 `psi_F`；
- 独立审核的旋转 tangent 同样缺少 `psi_F`。

修正只把运动方向从 formation frame 转回世界系；A/O/S 方程、signed direction、曲率、reverse 和 `goal_heading_free` 契约不变。新增 `test_world_headings_include_the_formation_heading_offset` 覆盖正/倒车平移与纯旋转，S5 定向测试 15/15 通过。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `/usr/bin/time -v env OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_geometry.py' -v` | 15/15 通过；测试计时 0.081 s；墙钟 0.28 s；最大 RSS 56628 KiB | 0 |
| `/usr/bin/time -v env OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_kinematics.py' -v` | 15/15 通过；0.005 s；墙钟 0.06 s；最大 RSS 18304 KiB | 0 |
| `/usr/bin/time -v env OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 254 项，253 通过、1 个既有错误；8.183 s；墙钟 8.43 s；最大 RSS 211564 KiB | 1 |
| `python3 -m py_compile ...` | 新模块、S5 修正和测试编译成功 | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 254 tests in 8.183s
FAILED (errors=1)
```

254 = S5 的 238 项 + S5 新增 1 项 + S6 新增 15 项。没有新增跳过、弱化或删除任何既有测试；未修改 `test_continuous_retiming.py` 或相关重定时模块。

## 定向测试与验收对应

| 文档验收要求 | 实际测试 | 结果 |
|---|---|---|
| 各车辆轨迹从 `q_F` 精确推导 | `test_straight_translation_reconstructs_rigid_centers_and_audits`；审核在 17 个 `lambda` 样本重算刚性残差 | 通过 |
| 每个姿态满足相对位置不变量 | `audit_continuous_plan` 的 `max_rigidity_error_m`；测试断言 `<1e-9` | 通过 |
| 纯旋转无数值崩溃 | `test_pure_rotation_uses_spin_pivot_and_ackermann_ring`；pivot Spin、非 pivot A、中心/航向重算 | 通过 |
| 斜移段无数值崩溃 | `test_two_block_chain_propagates_body_headings`；旋转后接 O 斜移，φ=-π/2 | 通过 |
| 逆向段无数值崩溃 | `test_reverse_translation_is_signed_and_policy_checked`；未授权 reverse 拒绝、授权后 signed direction=-1 | 通过 |
| 终端拟合只改 `q_F` | `test_terminal_translation_fit_changes_only_last_endpoint`、`test_terminal_rotation_fit_extends_dpsi_without_moving_centers`；中心仍从 `q_F` 派生 | 通过 |
| 每个保存模式可独立重算 | `test_audit_rejects_tampered_center_heading_and_modes`；篡改中心/航向/模式均被 S5 审核拒绝 | 通过 |
| `goal_heading_free` 只释放终端 `theta` | `test_goal_heading_free_is_explicit_and_does_not_move_centers`；中心端点仍精确 | 通过 |
| 候选去重 key | `test_master_retains_fixed_k_deduplicates_and_reports_rejections`；重复候选被拒 | 通过 |
| 固定 K、incumbent 与拒绝原因 | 同上；K=2 时只保留 2 个，记录 `CANDIDATE_LIMIT_REACHED`/`DUPLICATE_CANDIDATE` | 通过 |
| 局部优化前多车扫掠初筛 | `test_dense_collision_precheck_rejects_mid_path_obstacle`；起终可清、弧中障碍失败，审核记录 block 与 `qf` | 通过 |
| 种子 ID/起终/动作类型检查 | `test_composite_and_seed_identity_fail_closed`；plan_id/start/composite 失败闭合 | 通过 |
| 地图版本缺失不伪造 | `test_map_version_is_explicitly_unchecked_when_seed_has_no_signature`；`map_version_checked=false`，签名不匹配则拒绝 | 通过 |
| 只输出 GEOMETRY/KINEMATIC，不输出 FEASIBLE | `test_status_is_geometry_or_kinematic_only`、`test_master_never_admits_failed_audit_or_claims_feasible` | 通过 |
| 单车 `ContinuousPlan` 无退化 | 完整回归无新增失败；既有 `dynamic_peaks` 错误保留 | 通过 |

## 未执行项目与边界

1. 没有 S7 联合 NLP：当前 block 仍是分段常值参考平移/旋转，没有可优化控制节点、自由 `theta_i(lambda)` 或 `kappa_i/phi_i` 节点。
2. 终端拟合只改最后一段；更早 block 的残余几何不能回修，纯旋转终端只有在显式 pivot 确实能到达请求 goal 时才成功。
3. `FormationPlanSeed.msg` 没有地图版本字段；S6 只能对 duck-typed 可选签名做检查，真实种子通常记录 `map_version_checked=false`。
4. 碰撞初筛是稠密采样 + S2 padding；不证明连续时间、执行器跟踪、动态障碍或真实传感器安全，S9 必须独立审核。
5. 没有全车统一时间轴、速度/加速度/jerk、驻留、模式切换 dwell 或 `FormationTimedTrajectory`；S8 负责。
6. 没有 ROS 节点、话题、RViz、回放或真实地图集成；S6 是纯 Python 库。
7. 组合搜索是有界 beam，不证明整条 primitive 链的全局最优或完备性；固定 K 只保留评分较小的 audited 候选。
8. `FormationBlock` 只支持 translation/rotation；`COMPOSITE` 继续拒绝，避免把未定义曲线冒充连续几何。
9. 审核失败只返回首个失败项；多车/多 block 失败集合尚未聚合。
10. precheck 的采样数上限、margin 和间距是配置值；不是最终安全参数，S9 需要自己的策略和独立检查。
11. 没有处理偏心 pivot 搜索、动态障碍、地图更新或候选失效协议；S4/S9 范围。
12. S5 修正只覆盖世界航向偏移；没有改变 S5 的其它限制，例如未检查 `dpsi_F` 绝对转角上限、Spin 角速度或 dwell。

## 安全性审计

- **中心唯一来源**：所有中心由 `q_F + R(psi_F)r_i` 重算；plan 不存第二套中心曲线，终端拟合不独立移动车辆。
- **不默认 `theta_i=psi_F`**：每车 mode segment 保存独立 `theta`；S5 审核从 block 起点和模式重算；S6 插入 block 时不重置 `psi_F` 或车体航向。
- **S 只允许真 pivot**：S5 审核仍拒绝非 pivot Spin；S6 不把绕 pivot 旋转的非 pivot 车改标为 S。
- **reverse 不伪装**：`allow_reverse` 由请求/plan 双双校验；S5 保留 signed `direction=-1` 和 O 的 `wrap(delta+π)` 归一化。
- **终端释放有限**：`goal_heading_free` 只跳过该车终端 `theta`；S6 仍要求中心精确到请求 goal，且不释放 `psi_F`/pivot 的刚性关系。
- **失败闭合**：非法 seed 字段、composite、零长度、未知车辆、审计失败、碰撞初筛失败、K 满、重复候选都拒绝；没有候选不会返回 `KINEMATIC_VALID`。
- **不冒充可执行**：候选最多 `KINEMATIC_VALID`；所有 status 有 `executable=false`、`trajectory_published=false`、`independent_timing_audit=false`；没有 `FEASIBLE`。
- **不污染单车**：未修改 `core.py`、`continuous_geometry.py`、`continuous_retiming.py`、`quality.py`、单车 `.msg`、C++ 或 ROS 节点。
- **S5 修正有回归**：非零 `psi_F` 的翻译/倒车/旋转测试覆盖修正；原本 `psi_F=0` 的 S5 测试仍全部通过。

## 存在风险

1. 有界 beam 可能漏掉更优的跨 block 模式组合；固定 K 只是预算策略，不是完备性声明。
2. 终端拟合只修改最后一段，复杂种子可能在最后一段无法修复，从而被保守拒绝。
3. 地图版本无法从真实 `FormationPlanSeed` 读取，`map_version_checked=false` 是明确限制而非安全证明。
4. 碰撞初筛依赖采样和 S2 padding；窄通道/薄障碍物可能被保守拒绝或需要 S9 更细的独立审核。
5. `plan_candidate_score` 是启发式（路径长度、O 惩罚、模式切换惩罚），不是时间或净空最优目标。
6. 18 个刚性采样点（17 个间隔）只验证刚体不变量，不证明模式微分方程在每个连续点的执行器可行性。
7. S5 的世界航向修正会改变非零 `psi_F` 下的模式选择；S5 旧报告已追加说明，但任何下游若缓存旧模式结果需重新审核。
8. 没有地图更新/候选失效协议；master 的 K 候选只在一次调用内有效。
9. 没有真实地图、RViz、回放或 ROS 多进程验证；`KINEMATIC_VALID` 不是执行认证。
10. `precheck_plan_collision` 的 `failing_block_index` 根据 sweep state-pair index 映射；边界样本可能归到相邻 block，但保留 `qf` 和车辆上下文。

## 完成判据与 S7 前置条件

- 连续 `FormationBlock`/`FormationContinuousPlan` 与 `q_F(lambda)` 派生中心：**已实现并测试**。
- 纯旋转、斜移、逆向段无模式/数值崩溃：**已实现并测试**。
- 终端拟合只修改 `q_F` 最后一段、不独立移动中心：**已实现并测试**。
- S5 每 block 枚举 + 独立审核：**已实现；每个 S5 候选先审核再进入候选**。
- 候选 dedup、固定 K、incumbent、拒绝原因：**已实现并测试**。
- 多车扫掠初筛与失败上下文：**已实现并测试**。
- 只输出 GEOMETRY/KINEMATIC、无 FEASIBLE/时间轨迹：**已由代码和状态测试确认**。
- 单车回归：**无新增失败；既有 `dynamic_peaks` 错误保留**。
- S7 前置条件：必须以 S6 audited 候选为固定模式结构输入；NLP 只能优化 `q_F/theta_i/kappa_i/phi_i` 的连续参数，不能把车辆中心或刚体关系当自由变量；求解器 `success=True` 后必须重新运行 S6 独立几何/模式审核，禁止把 solver candidate 直接当最终可行；地图版本缺失和扫掠初筛非证书的限制必须继续显式保留。