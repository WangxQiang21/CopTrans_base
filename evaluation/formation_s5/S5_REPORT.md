# S5 阶段审查报告：每车 A/O/S 模式分配与局部航向可行性

## 结论

**S5 完成：新增 ROS 无关的编队局部运动学模块，按 block-local 混合进度 `lambda` 为每辆车枚举 A/O/S 分段，独立审核函数从 `q_F(lambda)`、固定 offset 和显式车体航向重算模式方程、中心位置、曲率、斜角和 Spin 中心位移；14 项定向测试全部通过。**完整 Python 回归为 238 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误；没有新增失败。本阶段没有生成连续几何计划、时间轨迹、碰撞认证或 `FEASIBLE`，也没有修改 C++、单车 `ContinuousPlan`、控制器、NMPC 或执行器。

## 任务编号

S5：每车 A/O/S 模式分配与局部航向可行性。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L635-L699)。

## Git 检查点

- S4 已完成提交：`17ef75f1ccd84b480d2ad98cbaba510f40902c94`
- S5 分支：`feat/formation-modes`（从 S4 提交创建）
- S5 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| S4 primitive | `TRANSLATION(dx,dy)` 与显式 pivot 的 `ROTATION(dpsi)` 已冻结；`COMPOSITE` 仍未定义 |
| S4 航向语义 | S4 的 `theta_i(q_F)=start_theta_i+(psi_F-start_psi_F)` 只是碰撞包络约定，S5 没有沿用为模式真值 |
| 单车模式约定 | 核对 `core.Action`、`MotionBlock`、`continuous_geometry._derivative` 与 `retiming.wheel_configuration`：A 为`direction*[cos θ, sin θ, κ]`，O 为`direction*[cos(θ+φ), sin(θ+φ), 0]`，S 中心不动、航向精确旋转 |
| 限制来源 | `retiming.MotionLimits.r_min_ack=0.810`、O 角上限 `π/2`、`allow_reverse` 对 A/O 的负方向限制；S5 复制为独立 `KinematicLimits`，没有依赖或修改重定时器 |
| S1 刚体 API | 复用 `FormationPose`、`VehicleOffset`、`FormationModel.centers()`；没有新增中心自由度 |
| 单车 `ContinuousPlan` | 未导入、未修改 `ContinuousPlan`/`MotionBlock` 的结构或语义 |
| 既有消息/节点 | 未修改任何 `.msg`、ROS 节点、launch 或 YAML |

## 修改文件

新增：

1. [formation_kinematics.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py)：`FormationKinematicBlock`、`VehicleModeSegment`、联合模式枚举、独立审核与失败诊断。
2. [test_formation_kinematics.py](../../tests/test_formation_kinematics.py)：14 项 ROS 无关测试。
3. [logs/formation_kinematics_tests.txt](logs/formation_kinematics_tests.txt)
4. [logs/unit_tests.txt](logs/unit_tests.txt)
5. [S5_REPORT.md](S5_REPORT.md)

未修改任何既有文件；没有 C++ 或消息变更，因此本阶段不需要新增 catkin 目标。

## 局部进度与数据结构

S5 明确区分两种参数：

- `start_station`/`end_station`：block-local 混合进度 `lambda`，平移 block 取 `[0, ||Δp_F||]`，纯旋转 block 取 `[0, |dpsi_F|]`。纯旋转阶段不把 `lambda` 当作平移弧长，也不对 `s=0` 做除法。
- `travel_distance`：该车中心实际弧长。平移 A/O 取 `parameter_length`；旋转非 pivot A 取 `radius * |dpsi_F|`；S 恒为 0。

`VehicleModeSegment` 保存 `mode`、signed `direction`、`theta_start/theta_end`、连续控制量 `control`（A 为 signed curvature κ，O 为 signed oblique angle φ，S 为 0）、`travel_distance`、起终中心坐标和显式 pivot。所有字段在构造时 fail-closed 校验；`travel_distance=0` 只允许 S。

每个成功 assignment 还记录：

- 最多 K 个联合模式方案，按 `(joint mode cost, mode word)` 排序；默认 `K=4`，组合数上限 4096。
- `stop_required_at_boundaries=true`：首版保守要求整编队参考平移和所有车体旋转速度降为零后才能换模，具体 dwell 由 S8 生成。
- `certification=KINEMATIC_CANDIDATE`、`executable=false`；未审核候选不能直接进入发布或控制器。

## 模式判定

### 平移 block

- A forward：`motion_angle` 与 `theta_i` 在 `1e-6 rad` 内一致；`direction=+1`、`κ=0`、车体航向恒定。
- A reverse：`motion_angle` 与 `theta_i+π` 一致且 `allow_reverse=true`；`direction=-1`、`κ=0`。
- O forward：`φ=wrap(motion_angle-theta_i)`，要求 `|φ|<=π/2`；车体航向恒定。
- O reverse：`φ=wrap(motion_angle-theta_i+π)`，要求 `|φ|<=π/2` 且 `allow_reverse=true`；路径实际方向为 `theta_i+φ+π`，没有把倒车伪装成大于 `π/2` 的 forward 斜角。
- 精确 A 对齐时优先 A；否则选择 O。若唯一可行方向是 reverse 而请求禁止 reverse，返回 `REVERSE_NOT_ALLOWED`。

### 纯旋转 block

- 半径 `radius=||offset-pivot||`。
- `radius<=1e-9 m`：只允许 S；`direction=sign(dpsi_F)`，中心位移必须恒为零，航向精确增加 `dpsi_F`。
- 半径非零：只允许 A。车体起始航向必须等于该车辆的 forward tangent `atan2(rx,-ry)` 或 reverse tangent `+π`；否则 `HEADING_BOUNDARY_MISMATCH`。
  - `κ=sign(dpsi_F)*direction/radius`，因此 `dtheta/d(travel)=sign(dpsi_F)/radius` 与 `direction*κ` 一致；`|κ|<=1/r_min_ack`。
  - reverse tangent 需要 `allow_reverse=true`，否则 `REVERSE_NOT_ALLOWED`。
  - 纯旋转下 O 不可能保持车体航向且走直线，S5 明确返回 `ROTATION_OBLIQUE_UNSUPPORTED`。
- 若非 pivot 车被保存为 S，审核函数直接返回 `NON_PIVOT_SPIN`，不因中心绕 pivot 旋转而误判为原地 Spin。

## 独立审核

`audit_mode_assignment()` 不信任保存的 segment 字段：

1. 从 `block.start_qf/end_qf/pivot` 重算 `qf(0)`、`qf(L)` 和所有车辆刚性中心。
2. 逐车重算 `theta_start/theta_end` 及旋转/平移边界连续性。
3. 平移 A 重新检查 `κ=0`、直线上位移等于 `L*direction*e_theta`；O 重新检查 `|φ|<=π/2` 和 `L*direction*e_(theta+φ)`。
4. 旋转 A 重新检查切线对齐、`κ=sign(dpsi)*direction/radius`、`|κ|<=1/Rmin`、中心弦长与圆弧弦长一致；旋转 S 重新检查半径为零、中心起终一致、方向等于 `sign(dpsi)`、travel 为零。
5. 重新检查 `allow_reverse`；审核函数默认 false，即使保存结构伪造 reverse 也会被拒绝。
6. 若提供显式终点航向，则逐车检查非 `goal_heading_free` 的车辆；`goal_heading_free` 只释放该车终点航向，不释放中心位置或刚体关系。
7. 记录最大中心误差、最大航向误差、最大曲率、最大斜角和最大 Spin 中心位移；审核通过仍只标记 `KINEMATIC_AUDITED`，`executable=false`，不输出 `FEASIBLE`。

审核失败保留拒绝车辆、原因和残差；当前实现拒绝首个失败项，不做多失败聚合。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_kinematics.py' -v` | 14/14 通过；测试计时 0.004 s；墙钟 0.07 s；最大 RSS 21100 KiB | 0 |
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 238 项，237 通过、1 个既有错误；7.734 s；墙钟 7.96 s；最大 RSS 240396 KiB | 1 |
| `python3 -m py_compile ...` | 新模块和新测试编译成功 | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 238 tests in 7.734s
FAILED (errors=1)
```

238 = S4 的 224 项 + 本阶段新增 14 项。没有新增跳过、弱化或删除任何既有测试；未修改 `test_continuous_retiming.py` 或相关重定时模块。

## 定向测试与验收对应

| 文档验收要求 | 实际测试 | 结果 |
|---|---|---|
| 每车 A/O/S 与局部航向可行性 | `test_translation_a_forward_and_reverse_are_signed_consistently`、`test_oblique_translation_holds_body_heading_and_uses_signed_phi`、`test_oblique_reverse_is_not_hidden_in_a_huge_forward_angle` | 通过 |
| 纯旋转非 pivot 车不能分配 S | `test_rotation_pivot_is_spin_and_nonpivot_vehicles_use_a`、`test_nonpivot_spin_assignment_is_rejected_by_independent_audit` | 通过 |
| A 曲率 `|dtheta/darc|<=1/Rmin` | `test_rotation_radius_below_rmin_is_rejected`；审核同时重算 `κ` 和 `1/r_min_ack` | 通过 |
| 航向边界连续性 | `test_rotation_requires_heading_to_match_a_tangent`、`test_independent_audit_recomputes_centers_heading_and_control` | 通过 |
| 倒车方向与 `allow_reverse` | `test_translation_a_forward_and_reverse_are_signed_consistently`、`test_rotation_reverse_travel_requires_allow_reverse`；审核另拒绝未授权 reverse | 通过 |
| 终点 `theta_i` 与 `goal_heading_free` | `test_goal_heading_constraint_is_explicit_and_heading_free_only_skips` | 通过 |
| 每个保存模式可独立重算 | `test_independent_audit_recomputes_centers_heading_and_control`、`test_spin_requires_an_identically_fixed_center` | 通过 |
| 最多 K 个联合模式方案 | `test_joint_enumeration_retains_k_and_all_are_auditable` | 通过 |
| 不冒充可执行结果 | `test_status_never_claims_execution`；所有 status 恒有 `executable=false` | 通过 |

## 未执行项目与边界

1. 没有把 S4 `FormationPlanSeed` 直接接入 S5；S6 才负责 primitive 链、连续 `q_F(lambda)` 和候选管理。
2. 没有做多 block 模式排序；当前最多枚举单 block 的 K 个联合方案，S6 需按整条 primitive 序列选择。
3. 没有做模式切换 dwell 的数值时长；S5 只标记首版必须在静止时换模，时长和静止事件由 S8 实现。
4. 没有检查车轮转角、轮速、转向速率/加速度或 A 的详细轮胎非侧滑；S8/S9 必须独立计算。
5. 没有做碰撞检查、连续几何拟合、终端拟合、NLP 或时间参数化。
6. 未处理偏心 pivot 之外的特殊转向机动；S4 仍只生成编队原点 pivot 的旋转。
7. 对多 block 的航向边界，S5 逐 block 接收上一 block 的终点航向；若旋转后接平移且车体航向与新运动方向不兼容，当前保守失败，未插入额外的静止航向事件。
8. 没有真实地图、RViz、回放或多进程测试；`KINEMATIC_AUDITED` 不是执行认证。
9. 审核失败当前只返回首个失败项，不保留全部车辆/block 失败集合。
10. `KinematicLimits` 是 S5 写死时值的副本；S8 必须显式传入自己的限制，不能用本模块默认值替代动态审核。

## 安全性审计

- **不混淆参数**：`lambda` 是 block-local 混合进度，A/O 方程使用 `travel_distance`；纯旋转没有用零平移弧长做除零。
- **不默认 `theta_i=psi_F`**：所有模式都使用显式车体航向；旋转 A 必须与切线对齐，S 航向精确跟随 `dpsi_F`。
- **S 只允许真 pivot**：半径大于 `1e-9 m` 的车辆保存为 S 会被独立审核拒绝；S 的起终中心位移必须不超过几何容差。
- **倒车不伪装**：A/O reverse 都要求 `allow_reverse` 且保持 signed `direction=-1`；O reverse 的 φ 通过 `wrap(delta+π)` 归一化到 `±π/2`。
- **审核不信任保存字段**：中心、航向、曲率、travel、控制量均从 `q_F`、offset、pivot 和模式重算；篡改测试会失败。
- **失败闭合**：非法 block、composite、零长度 primitive、未知模式、重复/缺失车辆、非有限值均拒绝；不返回可行模式。
- **不冒充可执行**：成功 assignment 只标 `KINEMATIC_CANDIDATE`，审核只标 `KINEMATIC_AUDITED`；没有 `FEASIBLE`、没有轨迹、没有 `executable=true`。
- **不污染单车**：未修改 `core.py`、`continuous_geometry.py`、`continuous_retiming.py`、`quality.py`、任何 `.msg`、C++ 或 ROS 节点。

## 存在风险

1. 单 block 贪心枚举没有证明整条 primitive 链存在联合可行模式；S6 必须逐 block 传播航向并对整链重审。
2. 航向边界容差 `1e-6 rad` 对 S4 的 `0.16 rad` 几何终点容差并不等价；S4 种子端点航向不精确时 S5 会保守拒绝。
3. 旋转 A 的 `τ=radius*lambda` 近似假定 constant curvature 和车辆中心严格绕 pivot；没有建模轮胎侧偏或执行器跟踪误差。
4. 旋转 S 的方向严格等于 `sign(dpsi_F)`；如果未来允许 pivot 车反向 Spin 来缩短时间，需要新的模式/时间策略，不能直接放宽本检查。
5. K 个联合方案只按简单 mode cost 排序；没有使用净空、切换次数或预计时间，S6 需要更完整的候选排序。
6. 未检查 `dpsi_F` 的绝对转角上限、Spin 角速度/角加速度/jerk 或 dwell；这些留给 S8。
7. `KINEMATIC_AUDITED` 只证明局部 A/O/S 方程和边界结构，不证明整条轨迹连续、无碰撞或时间可行。
8. 审核只检查首项失败；调试多车失败时需要 S6/S7 扩展诊断聚合。
9. 没有修改 C++ 前端做可行性反馈；S6 的 Python 候选管理负责筛除 S4 不可行种子。
10. 复杂编队（多车非共线、旋转后接斜移）可能因显式航向边界不匹配被保守拒绝，尚无可行的静止航向事件插入。

## 完成判据与 S6 前置条件

- 每车 A/O/S 模式与局部航向可行性：**已实现并测试**。
- `ROTATE` 非 pivot 车辆 S 拒绝：**已实现并由独立审核测试确认**。
- A 曲率、O signed angle、reverse 方向与 `allow_reverse`：**已实现并测试**。
- 独立重算保存模式结构：**已实现并测试**。
- 联合方案保留 K 个与简单排序：**已实现并测试**。
- 不输出 `FEASIBLE` 或时间轨迹：**已由代码和状态测试确认**。
- 单车回归：**无新增失败；既有 `dynamic_peaks` 错误保留**。
- S6 前置条件：必须从 S4 种子逐 primitive 构建 `FormationBlock`，在每个 block 边界传入上一段终点 `theta_i`；必须把 S5 返回的每个联合方案分别审核后才可加入候选；必须在整链终端处理显式终点航向与 `goal_heading_free`，不能把 S5 的单 block 候选当作整链可行；不得修改单车 `ContinuousPlan`。