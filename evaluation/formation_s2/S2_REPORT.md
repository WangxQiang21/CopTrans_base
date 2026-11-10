# S2 阶段审查报告：多车 OBB 与扫掠碰撞

## 结论

**S2 完成：新增逐车 OBB 地图/车对碰撞检查，以及共同全车队形状态的保守扫掠检查；17 项定向测试全部通过，N=1 与既有单车审核有 100 个随机位姿 golden 对照一致。**完整 Python 回归为 194 项，仍只有 S0 已记录的同一个 `dynamic_peaks` 错误。本阶段未进行路径搜索、模式分配、同步重定时、ROS 消息或任何多车 `FEASIBLE` 发布。

## 任务编号

S2：多车 OBB 与扫掠碰撞。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L499-L541)。

## Git 检查点

- S1 已完成提交：`773826b5f8e0e492b0d5f1a6203fc5a58b00d89e`
- S1 分支：`feat/formation-model`
- S2 分支：`feat/formation-collision`
- S2 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程；两个原有未跟踪 Markdown 文件未清理、覆盖或提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| `core.Grid` | 使用 `length/width/margin`，但 `margin` 语义是**总尺寸增量、每侧一半**；`pose_clear` 做车体 OBB 对占用栅格的 SAT 检查 |
| `quality.FootprintDistance` | 接收实际车长/车宽，不含 margin；返回 OBB 到占用栅格及地图边界的精确欧氏距离，并可传入 `obstacle_index` 复用障碍物边界 KD-tree |
| 单车审核的 margin | [optimizer_node.py:2728](../../catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py#L2728) 明确发布 `total_dimension_increment_half_per_side` |
| 方案接口 | `FormationCollisionChecker(grid, model, vehicle_shapes, margin_m)`、`static_check()`、`swept_check(states, policy)` |
| 依赖边界 | 新模块复用 `core.Grid` 和 `quality.FootprintDistance`，不依赖 ROS 节点、RViz、C++ 或地图 launch |

## 修改文件

仅新增本阶段文件：

1. [formation_collision.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_collision.py)：车体形状、全车状态、扫掠策略、结构化碰撞报告和检查器。
2. [test_formation_collision.py](../../tests/test_formation_collision.py)：17 项碰撞及输入失败测试。
3. [logs/formation_collision_tests.txt](logs/formation_collision_tests.txt)：定向测试完整日志。
4. [logs/unit_tests.txt](logs/unit_tests.txt)：完整 Python 回归及资源统计。
5. [logs/catkin_build.txt](logs/catkin_build.txt)：catkin 构建日志。
6. [logs/collision_call_probe.txt](logs/collision_call_probe.txt)：N=3 多段扫掠耗时和查询次数。
7. [S2_REPORT.md](S2_REPORT.md)：本报告。

未修改 S1 的 `formation.py`、单车算法、消息、YAML、launch、CMake、C++ 或既有测试。

## 实现的碰撞合同

### 车辆形状与安全边距

- `VehicleShape(length_m, width_m)` 必须是有限且严格为正的实际车体尺寸。
- `vehicle_shapes` 为 `vehicle_id -> VehicleShape` 映射，也接受 `(length, width)` 或带 `length/width`、`length_m/width_m` 字段的车辆规格。
- ID 集合必须与 `FormationModel` 完全一致；缺车、额外车辆、重复 ID、零/负尺寸、负 margin 或错误 grid 类型均在构造阶段失败。
- `margin_m` 沿用现有单车的 `total_dimension_increment_half_per_side`：每个车体长宽各增加 `margin_m`，每侧增加 `margin_m/2`。因此地图净空必须严格大于 `margin_m/2`，两个物理车体之间的净空必须严格大于 `margin_m`。
- 地图占用格与地图边界触碰均判为失败；车间 OBB 接触也判为失败，不使用“刚好相等即安全”的宽松边界。

### 静态检查

`static_check(qf, body_headings)` 严格按以下顺序执行：

1. 校验 `FormationPose`、完整车辆 ID 集合和所有独立车体航向的有限性。
2. 由 S1 `FormationModel.vehicle_poses()` 恢复每辆车的真实中心与独立车体航向。
3. 每辆车调用现有 `FootprintDistance.distance()`，检查真实 OBB 与地图占用栅格/边界；只检查单车中心点会直接失败。
4. 对所有车辆对执行 2D OBB 分离轴检查，轴集合包含两车各自的长轴和短轴共四轴。
5. 使用物理尺寸加 margin 的 OBB，不以凸包或中心距离捷径替代。

失败返回 `CollisionReport`，包含 `reason`、`vehicle_id`、车对失败时的 `other_vehicle_id`、`segment_index`、`sample_index`、可选 `qf` 和上下文 `detail`。碰撞原因目前包括：

- `MAP_OBB_COLLISION`
- `VEHICLE_OBB_COLLISION`
- `INVALID_FORMATION_POSE`
- `VEHICLE_ID_SET_MISMATCH`
- `NONFINITE_VALUE`
- 其他输入/策略错误码

### 共同状态与扫掠

`FormationState(station, qf, body_headings)` 表示同一时刻/同一进度的全车队形状态。`swept_check(states, policy)` 仅接受严格递增 `station` 的有序状态，并执行：

1. 空状态、非法对象、缺字段、NaN/Inf、非单调 station、车辆 ID 不一致或车体航向缺失/额外均失败。
2. 所有状态必须包含同样且完整的车辆集合。
3. 初始状态先以 sweep padding 检查。
4. 每对相邻状态计算连续位移上界：

```text
D_bound = ||Δp_F||
        + Rmax * |Δψ_F|
        + max_i(rho_i * |Δθ_i|)
```

其中角度差使用**原始展开差**，不调用 `wrap()`；`psi` 和每辆车 `theta_i` 分别计入。`rho_i` 使用含 margin 的车辆角点半径，略保守于文档的物理半径公式。
5. 采样数 `ceil(D_bound / max_corner_step_m)`，多段共享同一连续插值规则；超过 `max_samples_per_segment` 时失败，不截断后继续。
6. 在 `q_F` 平移、`ψ_F` 与全部独立 `θ_i` 的共同进度分数处构造中间状态，所有车辆使用同一 fraction。
7. 每个扫掠样本按 `max_corner_step_m` 膨胀车体，再执行地图 OBB 和全车对 OBB 检查。该膨胀对应相邻采样间的最大车体点位移，用于覆盖采样之间的运动，属于现有 `action_clear()` 同类保守机制。
8. 所有中间失败都带 `reason + vehicle_id + segment_index + sample_index`；车对失败另带 `other_vehicle_id`。

`swept_check()` 是保守几何检查，不是最终高密度独立审核，也不构成可执行轨迹证书。文档要求的最终独立审计仍未实现。

### 查询统计

检查器提供累积 `statistics()`，字段包括：

- `static_checks`
- `swept_checks`
- `segments_checked`
- `states_checked`
- `swept_samples_checked`
- `map_queries`
- `obb_pair_queries`

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_collision.py' -v` | 17/17 通过；测试计时 0.030 s，墙钟 0.22 s；最大 RSS 56228 KiB | 0 |
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 194 项，193 通过、1 错误；测试计时 7.789 s，墙钟 7.98 s；最大 RSS 237984 KiB | 1 |
| `source /opt/ros/noetic/setup.bash; catkin_make -C .../catkin_ws -j1` | 构建系统检查与增量 `make` 完成；墙钟 2.14 s；最大 RSS 17084 KiB | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归错误与 S0 完全相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 194 tests in 7.789s
FAILED (errors=1)
```

194 = S1 的 177 项 + 本阶段新增 17 项。新增测试无 failure/error；既有错误未修复。`catkin_make` 为增量调用，不能据此推断全量构建耗时。

## N=3 扫掠探针

空 6 m × 6 m、0.05 m 栅格，三车 `0.738 × 0.500 m`，margin 0.04 m，两段共同状态，`max_corner_step_m=0.005`：

| 指标 | 实际值 |
|---|---:|
| 初始静态检查 | 成功；3 次地图查询、3 次车对查询 |
| 扫掠结果 | 成功 |
| 扫掠墙钟 | 0.020621888 s |
| 每段参考平移 | 0.353553391 m / 0.353553391 m |
| 扫掠采样数 | 455 |
| 累计状态检查数 | 456（含初始） |
| 累计地图查询 | 1368 |
| 累计 OBB 车对查询 | 1368 |

这些数字是单次本机合成地图探针，不是 ROS 或真实地图性能，也不是统计分位数。完整原始记录见 [collision_call_probe.txt](logs/collision_call_probe.txt)。

## 定向测试对应验收项

| 验收要求 | 测试 |
|---|---|
| N=1 与旧单车一致 | 100 个确定性随机位姿与 `grid.pose_clear()` 完全一致 |
| 车 0 安全、车 1 撞墙 | 失败，`vehicle_id=1`，`MAP_OBB_COLLISION` |
| 两个 OBB 旋转后重叠 | 失败，`VEHICLE_OBB_COLLISION` |
| 中心间距不变但朝向导致重叠 | 中心距离 0.5 m > 车宽 0.2 m 仍由 OBB 正确判失败 |
| 端点安全、中间旋转撞墙 | 端点静态均通过，扫掠中途失败，带 segment/sample |
| 端点安全、中间两车 OBB 重叠 | 端点静态均通过，扫掠中途车对失败 |
| 地图边界触碰 | 恰好触碰失败，向内侧 1 mm 通过 |
| 窄门按 margin 判定 | 0.60 m 门、0.50 m 车：margin 0.08 通过，扩宽后恰好 0.60 m 失败 |
| 空地图多段直线 | N=3、两段、共同 heading，通过并记录查询次数 |
| 空/不连续/NaN 状态 | 空列表、非单调 station、缺车、非法对象、NaN station/heading 均失败 |
| N=1/2/3 | 单车间距/边界、双车 OBB、三车多段均覆盖 |
| 展开角超过 π | `ψ_F` 和 `θ_i` 从 0 到 `2π` 时仍执行多采样，不被误判为零动作 |

## 未执行项目与边界

1. 未进入地图搜索；没有路径、primitive 或前端候选。
2. 未实现逐车 A/O/S 模式分配、共同时间轴、同步重定时或独立高密度轨迹审核。
3. 未创建 S3 ROS 消息、话题、launch 或请求适配器。
4. 未实现 C++ `formation_collision.hpp`；按文档在 S4 接入。
5. 未请求真实地图的静态或扫掠验证，未启动 ROS 节点、RViz 或回放。
6. 未修改或放宽单车审核、扫掠间距、碰撞 margin、终点 0.02 m / 1° 等合同。
7. `swept_check` 的保守采样不被宣称为最终连续安全证明，不能据此发布多车可执行结果。

## 安全性审计

- 地图碰撞是逐车真实 OBB，不是中心点、外接圆或凸包捷径。
- 车对碰撞是完整 SAT，包含两个 OBB 的四个候选分离轴；中心距离不构成充分条件。
- 地图/占格/车对/地图边界触碰均采用失败优先语义。
- 所有车辆使用同一个状态 fraction，禁止逐车独立采样后直接拼接。
- `ψ_F` 与各车 `θ_i` 分开传播和插值。
- 扫掠用连续位移上界决定采样密度，并按同一上界膨胀采样车体；不是只检查端点或静态采样点。
- 所有实际碰撞失败都带车辆和 segment/sample 上下文。
- 输入异常、超采样预算和非法策略全部 fail-closed。

## 存在风险

1. 完整 2π 旋转若在上游同时被编码成完全相同的已 wrap `ψ/θ` 数值，S2 无法恢复“转过一圈”的历史；`FormationState` 必须由上游保存展开角。当前测试使用显式 `2π` 差值证明本模块不会主动 wrap，但不能修复已经丢失的上游信息。
2. `swept_check` 假设相邻状态的 `q_F` 平移、`ψ_F` 和全部 `θ_i` 按线性共同进度连续插值。若真实执行采用其他参数化，必须提供对应连续模型或由独立审核覆盖。
3. margin 沿用单车的“总尺寸增量、每侧一半”语义；若调用者误把它当作地图的完整单侧净空要求，会低估地图净空。
4. 保守采样和膨胀尚未与最终独立高密度执行扫掠对照；不能发布 `FEASIBLE`。
5. 合成地图探针只测一次本机耗时，不能代表真实地图、CPU 负载或大 N 的复杂度表现。

## 完成判据与 S3 前置条件

- S2 模块与 17 项定向测试：**完成**。
- 单车 OBB golden、逐车碰撞、车对 OBB、扫掠中间碰撞、展开角：**已验证**。
- 旧回归新增失败：**无**；既有 `dynamic_peaks` 错误仍保留。
- S3 前置条件：冻结 `FormationState`、`CollisionReport` 和 `SweepPolicy` 的字段语义；ROS 请求重建后必须再次构造 `FormationModel`/`vehicle_shapes` 并校验车辆 ID，不能信任消息内容。S3 不应把本阶段几何通过解释为可执行轨迹通过。
