# S4 阶段审查报告：C++ 编队前端初始几何搜索

## 结论

**S4 完成：新增 ROS 无关的刚性编队模型、OBB/扫掠碰撞检查、仅搜索 `q_F=(x_F,y_F,psi_F)` 的启发式几何前端和只发布几何种子的 ROS 节点；13 项 C++ 几何测试、6 项既有规划策略测试、30 项消息/请求合同测试全部通过，catkin 构建退出码 0。真实隔离 ROS 探针只发布 `/formation/plan_seed` 和 `/formation/status`，成功结果严格标记为 `GEOMETRY_ONLY_UNCERTIFIED`，没有时间轨迹或 `FEASIBLE`。**完整 Python 回归仍为 224 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误；没有新增回归失败。

本阶段没有进入 S5：没有逐车 A/O/S 分配，没有车体航向规划，没有共同时间轴/重定时，没有控制器、NMPC、执行器或实车闭环修改。

## 任务编号

S4：C++ 编队前端（先几何，再运动学）。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L569-L632)，阶段表见同文档 [S4/S5 行](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L399-L400)。

## Git 检查点

- S3 已完成提交：`9f38755fdbf4fb16579a8254acedb5104cc9f996`
- S4 分支：`feat/formation-frontend`（从 S3 提交创建）
- S4 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| 单车规划器 | 已阅读现有 `planner_node.cpp` 的 Pose、Action、Key、碰撞和 A* 段；S4 没有复用单车 `Action[]`、`ModePrimitive` A/O/S 编号或 `minimum_time` 目标来冒充多车结果 |
| S1/S2 合同 | 已核对 Python `FormationModel` 的 `p_i=p_F+R(psi_F)r_i` 与 S2 OBB/保守扫掠语义；C++ 版本独立实现，但保持相同的刚体位置公式、严格失败闭合和触碰即碰撞语义 |
| S3 消息/请求 | 已核对 7 个编队消息、请求校验顺序、`goal_heading_free` 只释放终端车体航向而不释放中心刚体位置、`plan_id`/`formation_id` 独立 |
| `fwis_v3_msgs` | 7 个消息已在 `add_message_files` 中；S4 只迁移 `FormationPrimitive` 的初版字段，未新增依赖 |
| `fwis_v3_planner` | 原 `CMakeLists.txt` 只有单车节点和既有策略测试；S4 将本地 `include/` 加入 include 路径，并增加一个编队可执行目标和一个 gtest 目标；原单车目标未改 |
| 单车入口 | `fwis_v3_demo/launch/demo.launch`、`params.yaml`、`params_restricted.yaml`、单车 RViz/回放未修改 |
| 地图输入 | `/map` 为 `nav_msgs/OccupancyGrid`；S4 节点拒绝旋转或非有限 origin、非正分辨率、尺寸/数据不一致地图 |
| 消息消费面 | 仓库检索未发现 S3 之后、S4 之外的 `FormationPrimitive` 消费者；旧 `amount/control` 字段没有编队侧生产者或消费者 |

## 修改文件

新增 C++ 前端：

1. [formation_model.hpp](../../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_model.hpp)：ROS 无关的 `FormationPose`、显式 primitive、刚体积分与请求校验。
2. [formation_collision.hpp](../../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_collision.hpp)：栅格地图、车对车/车对地图 OBB SAT、保守扫掠。
3. [formation_search.hpp](../../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_search.hpp)：搜索配置、结果结构和入口声明。
4. [formation_search.cpp](../../catkin_ws/src/fwis_v3_planner/src/formation_search.cpp)：`q_F` 几何搜索、候选重建、预算和失败原因。
5. [formation_planner_node.cpp](../../catkin_ws/src/fwis_v3_planner/src/formation_planner_node.cpp)：`formation_planner` ROS 节点，只发布几何种子和状态。
6. [test_formation_geometry.cpp](../../catkin_ws/src/fwis_v3_planner/tests/test_formation_geometry.cpp)：13 项模型、字段拒绝、航向独立、碰撞和搜索测试。

新增入口与配置：

7. [formation_params.yaml](../../catkin_ws/src/fwis_v3_demo/config/formation_params.yaml)：独立搜索/代价参数；不修改单车参数。
8. [formation_demo.launch](../../catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch)：只启动地图发布器、编队前端和可选 RViz；不启动单车规划器/优化器/回放。

修改：

9. [FormationPrimitive.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationPrimitive.msg)：删除模糊的 `amount/control`，改为显式 `dx/dy/dpsi/pivot_x/pivot_y`。
10. [fwis_v3_planner/CMakeLists.txt](../../catkin_ws/src/fwis_v3_planner/CMakeLists.txt)：把本地 `include/` 加入 include 路径，新增 `formation_planner_node` 和 `test_formation_geometry`；原单车目标保留。
11. [test_formation_contract.py](../../tests/test_formation_contract.py)：冻结迁移后的 primitive slot 类型与 pivot 序列化。
12. [S3_REPORT.md](../formation_s3/S3_REPORT.md)：追加 S4 消息迁移说明，保留 S3 原始证据。

证据日志：

13. [logs/catkin_build.txt](logs/catkin_build.txt)
14. [logs/formation_geometry_tests.txt](logs/formation_geometry_tests.txt)
15. [logs/planner_policy_tests.txt](logs/planner_policy_tests.txt)
16. [logs/formation_contract_tests.txt](logs/formation_contract_tests.txt)
17. [logs/unit_tests.txt](logs/unit_tests.txt)
18. [logs/ros_node_probe.txt](logs/ros_node_probe.txt)
19. [logs/launch_config_check.txt](logs/launch_config_check.txt)
20. [logs/ros_message_schema.txt](logs/ros_message_schema.txt)
21. [logs/single_vehicle_message_hashes.txt](logs/single_vehicle_message_hashes.txt)
22. [S4_REPORT.md](S4_REPORT.md)

未修改任何既有单车 `.msg`、S1/S2 Python 模块、单车 `planner_node.cpp`、单车优化器、`demo.launch` 或单车参数。

## `FormationPrimitive` 消息迁移

S4 开始搜索前先定稿第一版 primitive，避免把旋转 pivot 藏进两个模糊数值：

```text
uint8   geometry_type   # TRANSLATION=0, ROTATION=1, COMPOSITE=2
float64 dx              # 当前编队坐标系平移，psi_F 不变
float64 dy
float64 dpsi            # 绕显式 pivot 的编队参考转角
float64 pivot_x         # 当前编队坐标系中的旋转中心
float64 pivot_y
```

- `TRANSLATION` 的 `(dx,dy)` 在积分时刻的当前编队坐标系中解释；积分后所有车辆中心得到同一个世界平移向量。
- `ROTATION` 的 pivot 是合同字段，必须显式给出；积分把 pivot 与每个车辆中心作同一刚体旋转。`ROTATION` 只是编队几何旋转候选，不等于每车 Spin 可行。
- `COMPOSITE` 保留常量但在 S4 中禁用；S6 定义连续参数化后再扩展。
- Python 合同测试把 slot 类型冻结为 `["uint8","float64","float64","float64","float64","float64"]`，并验证 pivot 非零的种子在 ROS 序列化/反序列化后保持不变。
- 仓库内没有旧 `amount/control` 编队消费者；五个单车 `.msg` 的 SHA-256 与 S3 前完全一致，见 [single_vehicle_message_hashes.txt](logs/single_vehicle_message_hashes.txt)。

## C++ 模型与请求校验

[formation_model.hpp](../../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_model.hpp) 明确区分编队参考航向 `psi_F` 与车体航向 `theta_i`：

- 请求仍使用 `p_i = p_F + R(psi_F) r_i`，`R` 是行列式 `+1` 的正常二维旋转；没有反射、镜像或 `theta_i` 替代 `psi_F`。
- `validate_formation_request()` 独立复核 S3 语义：`frame_id=="map"`、非零 header stamp、`nsecs ∈ [0,1e9)`、非空 `formation_id`、非空车辆列表、唯一 uint32 `vehicle_id`、有限值、`offset_z==0`、严格正车体尺寸、无重合 offset。
- `start_center`/`goal_center` 与 `p_F + R(psi_F) r_i` 的误差超过默认 `1e-6 m` 即拒绝。`goal_heading_free=true` 只释放终端车体航向约束，**不释放中心位置刚体约束**。
- 成功校验按 `vehicle_id` 排序车辆；`plan_id` 与 `formation_id` 始终是独立字段。

`FormationModel` 保存每辆车的 `start_theta` 与 `goal_theta`。S4 搜索只改变 `q_F`，因此几何碰撞包络采用明确的刚性航向约定：

```text
theta_i(q_F) = start_theta_i + (psi_F - start_psi_F)
```

这不是 S5 的模式分配结果，也不是“默认 `theta_i=psi_F`”。如果请求的初始车体航向不同，它们保持差值并随编队航向整体旋转；终点静态检查则使用请求显式给出的 `goal_theta_i`。逐车 A/O/S 航向和是否可 Spin 必须由 S5 独立判定。

刚体积分见 [formation_model.hpp:78-108](../../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_model.hpp#L78-L108)：

- `TRANSLATION`：`p_F' = p_F + R(psi_F)(dx,dy)`，`psi_F` 不变。
- `ROTATION`：pivot 先转换到世界系，`psi_F' = psi_F + dpsi`，`p_F'` 绕世界 pivot 旋转。
- `fraction_primitive()` 只按同一比例缩放平移/转角，扫掠采样与消息积分共用同一刚体定义。

## 碰撞检查

[formation_collision.hpp](../../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_collision.hpp) 不依赖 ROS 消息：

- 地图把非零 cell 和 unknown cell 都视为占用；每行合并连续占用 cell 为矩形段，再做 OBB-vs-AABB 的完整 SAT。边界、未知值和地图外都失败闭合。
- 触碰与 `1e-12` 内近触碰按碰撞处理，不把“刚好贴上”当作安全。
- 车对车使用四个候选轴的二维 OBB SAT；`start_clear()` 使用请求起点车体航向，`goal_clear()` 使用请求显式终点车体航向，`rigid_bodies(qf)` 使用上面的 S4 航向约定。
- 平移扫掠位移上界为 `||(dx,dy)||`。
- 旋转扫掠逐车取 `|r_i-pivot| + 0.5*hypot(L_i,W_i)` 的最大值，乘以 `|dpsi|` 得角点位移上界。采样数为 `max(1, ceil(D_bound/max_corner_step_m))`，所有车在共同分数处检查，并把车体按 `D_bound/(2*samples)` 膨胀。最远采样间角点位移不超过该膨胀量，因此这是保守几何包络。
- `SweepReport` 记录采样数、步长上限和 padding；`COMPOSITE` 或非法 pivot/非有限值不支持，返回失败。

这部分只证明“在栅格占用和给定几何包络下不碰撞”，不证明时间参数化、轮级运动学、执行器或真实传感器安全。

## `q_F` 搜索

[formation_search.cpp](../../catkin_ws/src/fwis_v3_planner/src/formation_search.cpp) 只搜索编队参考位姿 `q_F=(x_F,y_F,psi_F)`：

- 起点/终点先做完整静态 OBB 检查，失败分别返回 `START_FORMATION_COLLISION`、`GOAL_FORMATION_COLLISION`。
- 状态键为 `round(x_F/xy_res), round(y_F/xy_res), round(psi_F/yaw_res)`；`psi_F` 保持展开历史，防止绕圈后与短角路径错误合并。键不包含任何车的 A/O/S 真值。
- 启发函数为 `h=hypot(goal_xy-current_xy)`，代码注释明确它只是几何引导，没有声明 A* 最优性或可采纳时间下界。
- 代价为平移长度、旋转角、cell-distance clearance risk 和 primitive 类型切换惩罚的加权和；没有继承单车 `minimum_time` 评分。
- 初版 primitive：8 个等距编队平移方向、一个朝终点的截断平移、朝终点的最短方向旋转一个步长、两个固定方向旋转探针。初版旋转 pivot 固定为编队当前原点 `(0,0)`；消息和积分支持任意显式 pivot，测试也覆盖非零 pivot，但搜索尚未生成偏心 pivot。
- 每条边都做保守共同扫掠；接近终点后重建整条 primitive 链并重新做完整扫掠，只有重检通过才接受。
- 预算来自独立 YAML：`max_expansions`、`max_candidates`、`max_wall_time_s`；触顶返回 `FORMATION_BUDGET_EXHAUSTED`，没有路径返回 `FORMATION_NO_GEOMETRIC_PATH`。
- 成功时内部 `reason="OK"`、`detail="GEOMETRY_ONLY_UNCERTIFIED"`、`geometry_only=true`。`OK` 只表示几何搜索返回了候选，不是可执行认证。

## ROS 节点与状态

[formation_planner_node.cpp](../../catkin_ws/src/fwis_v3_planner/src/formation_planner_node.cpp)：

- 订阅：`/map`（`nav_msgs/OccupancyGrid`）、`/formation/plan_request`（`FormationPlanRequest`）。
- 发布：latch `/formation/plan_seed`（`FormationPlanSeed`）、latch `/formation/status`（`std_msgs/String` JSON）。
- 不发布 `FormationTimedTrajectory`、`TimedTrajectory`、控制量或任何可执行轨迹；状态恒有 `trajectory_published=false`、`executable=false`。
- 状态 schema 为 `formation_frontend_status/v1`，含 `valid`、`reason`、`field`、`vehicle_id`、`detail`、`plan_id`、`formation_id`、`node_state`、`certification`、`search_started`、`requests_seen`、`expanded_nodes`、`generated_nodes`、`goal_candidates`、`primitive_count`、`geometry_only`。
- 节点状态与认证：

| 情况 | `node_state` | `certification` | 是否发布种子 |
|---|---|---|---:|
| 请求结构/刚体关系非法 | `REJECTED` | `NONE` | 否 |
| 无地图 | `MAP_UNAVAILABLE` | `NONE` | 否 |
| 地图 frame 不是 `map` | `MAP_FRAME_MISMATCH` | `NONE` | 否 |
| 起点/终点/搜索几何失败或预算耗尽 | `GEOMETRY_SEARCH_FAILED` | `NONE` | 否 |
| 找到几何候选 | `GEOMETRY_ONLY_UNCERTIFIED` | `GEOMETRY_ONLY` | 是 |

- 种子 header frame 固定为 `map`，复制请求的 `plan_id`/`formation_id`，`formation_start/goal.theta` 是 `psi_F`，primitive 发布显式 `dx/dy/dpsi/pivot_x/pivot_y`。
- 非法请求不会清空或替换以前 latch 的旧种子；下游必须按 `plan_id`、`formation_id`、状态和重新验证决定是否消费。S4 没有实现种子失效协议。
- 搜索在请求回调中同步执行；单次预算上限为 8 s，期间该节点不处理新的地图/请求回调。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `source /opt/ros/noetic/setup.bash; catkin_make -C catkin_ws --pkg fwis_v3_planner -j1` | 强制重编 `formation_planner_node.cpp` 和 `formation_search.cpp`；墙钟 7.59 s；最大 RSS 490104 KiB；无 warning/error | 0 |
| `make -C catkin_ws/build test_formation_geometry -j1` | 重编测试源和 `formation_search.cpp`，链接成功 | 0 |
| `catkin_ws/devel/lib/fwis_v3_planner/test_formation_geometry --gtest_color=no` | 13/13 通过；gtest 51 ms；墙钟 0.05 s；最大 RSS 6176 KiB | 0 |
| `catkin_ws/devel/lib/fwis_v3_planner/test_planner_policy_history --gtest_color=no` | 6/6 通过；5 ms | 0 |
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_contract.py' -v` | 30/30 通过；0.006 s | 0 |
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 224 项，223 通过、1 个既有错误；7.747 s；墙钟 7.97 s；最大 RSS 239780 KiB | 1 |
| `roslaunch --nodes fwis_v3_demo formation_demo.launch` | 仅 `/fwis_map_publisher`、`/formation_planner` | 0 |
| `xmllint` + YAML 解析 | XML 合法；YAML 参数与文件一致 | 0 |
| 隔离 ROS 探针（端口 11413） | 成功/非法 frame/终点碰撞三种状态；`PROBE_OK`；清理后端口关闭 | 0 |
| 单车消息 SHA-256 与 `git diff` | 5 个既有消息字节不变 | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 224 tests in 7.747s
FAILED (errors=1)
```

这不是 S4 引入的失败，本阶段未修改 `test_continuous_retiming.py` 或相关重定时模块，也未以跳过/弱化测试的方式绕过。

## 验收矩阵

| 文档验收要求 | 实际测试/证据 | 结果 |
|---|---|---|
| 空地直线 | `FormationSearch.EmptyMapStraightLineSucceedsAsGeometryOnly`；种子 `geometry_only=true`、`detail=GEOMETRY_ONLY_UNCERTIFIED` | 通过 |
| 障碍物绕行 | `FormationSearch.ObstacleDetourGeometricallySucceeds`；路径避开障碍且全段重检 | 通过 |
| 终点不可行 | `FormationSearch.GoalInsideObstacleFailsStatically`；ROS 探针 `GOAL_FORMATION_COLLISION`，不发布种子 | 通过 |
| 起点不可行 | `FormationSearch.StartInsideObstacleFailsStatically` | 通过 |
| 旋转扫掠区分成功/失败 | `FormationCollision.RotationSweepDistinguishesMidArcObstacle`（起终静态可清、弧中间撞障碍必须失败）；`FormationSearch.ExplicitPivotRotationReachesRotatedGoal`（显式非零 pivot 的旋转终点） | 通过 |
| OBB 触碰/重叠严格失败 | `FormationCollision.SatTreatsTouchingBodiesAsCollision`、`DetectsVehicleOverlapAndMapObstacle` | 通过 |
| 固定 offset/镜像/换标 | 模型测试拒绝破坏刚体中心与重复 ID；S3 合同测试继续拒绝镜像目标 | 通过 |
| `theta_i` 不默认等于 `psi_F` | `FormationCollision.StartGoalAndRigidHeadingsRemainExplicit` 使用非零车体航向差，分别核对起点、终点和随 `psi_F` 刚性旋转后的车体航向 | 通过 |
| 关键字段失败闭合 | `FormationModel.RejectsMalformedRequestFieldsFailClosed` 覆盖非零 offset z、错误 frame、零 stamp、非正车体尺寸 | 通过 |
| 显式 pivot 合同 | 消息 slot 测试和 `PrimitiveIntegrationUsesFormationFrameAndExplicitPivot` | 通过 |
| 仅 `GEOMETRY_ONLY`，无可执行时间轨迹 | 所有搜索成功结果的 `geometry_only=true`；节点状态 `GEOMETRY_ONLY_UNCERTIFIED`；探针发布话题无任何 trajectory | 通过 |
| 原单车策略不回退 | 既有 `test_planner_policy_history` 6/6；单车 `.msg` 哈希不变；`demo.launch`/参数未改 | 通过 |

## ROS 节点探针

探针使用 100×100、0.1 m 分辨率的合成空地图和三车楔形编队，offset 为 `(0,0)`、`(0,1.2)`、`(0,-1.2)`，`q_F` 从 `(2,2,0.2)` 到 `(5,3,-0.1)`；实际输出见 [logs/ros_node_probe.txt](logs/ros_node_probe.txt)：

- 成功请求：`valid=true`、`node_state=GEOMETRY_ONLY_UNCERTIFIED`、`certification=GEOMETRY_ONLY`、8 个 primitive、`expanded_nodes=59`、`generated_nodes=646`、`goal_candidates=1`；`trajectory_published=false`、`executable=false`。种子 `plan_id=41`、`formation_id=s4_probe_wedge`、`header.frame_id="map"`。
- 非法 `frame_id="odom"`：`REJECTED`、`INVALID_REQUEST`、`field=header.frame_id`，`search_started=false`，不发布种子。
- 终点被障碍覆盖：`GEOMETRY_SEARCH_FAILED`、`GOAL_FORMATION_COLLISION`，不发布种子。
- 已发布话题仅为 `/formation/plan_request`、`/formation/plan_seed`、`/formation/status`、`/map`、`/rosout`、`/rosout_agg`；没有任何 trajectory/control 话题。
- 探针退出码 0；节点与 master 清理后端口 11413 关闭。

这是单进程合成探针，不是真实地图、RViz、回放、动态障碍或多请求并发性能结论。

## 未执行项目与边界

1. 未执行 S5：没有逐车 A/O/S 模式分配、没有每车 `theta_i` 可行性、没有对 `ROTATE` 非 pivot 车辆强制 Spin 的检查。
2. 未生成时间参数化、速度/加速度、曲率、模式切换 dwell 或共同时间轴；没有 `TimedTrajectory`/`FormationTimedTrajectory`。
3. 没有调用 Python 后端优化、NMPC、控制器、执行器或实车状态闭环。
4. 没有发布或声称 `FEASIBLE`；`GEOMETRY_ONLY` 不是可执行认证。
5. `allow_reverse` 只被请求结构携带，S4 搜索未解释，也没有倒车几何分支。
6. `goal_heading_free` 未被 S4 搜索用于释放终点朝向；终点静态碰撞仍采用请求显式 `goal_theta_i`，这是保守处理，语义留待 S5。
7. 旋转 primitive 只生成编队原点 pivot；没有偏心 pivot 的转向机动搜索。
8. `max_candidates` 目前限制的是接近终点的失败重检次数；一次成功即返回并发布一个种子，没有多候选排序或候选列表。
9. clearance risk 使用 cell-distance 场，只是代价项；安全门仍是精确栅格 OBB SAT。
10. 地图为静态最新 `/map`，没有地图版本、动态障碍、时序地图或多地图一致性检查。
11. 没有真实地图、RViz 可视化标记、回放或多进程负载测试。

## 安全性审计

- **严格失败闭合**：非法请求、无效地图、起点/终点碰撞、任意扫掠碰撞、预算耗尽都返回失败；失败请求不发布种子。
- **刚体约束**：车辆中心必须等于 `p_F+R(psi_F)r_i`；`goal_heading_free` 不能放宽中心位置；重复 ID、重合 offset、非有限值全部拒绝。
- **不混淆航向**：`psi_F` 与 `theta_i` 是不同字段；S4 搜索只改 `q_F`，逐车航向使用显式刚性约定，未默认 `theta_i=psi_F`。
- **碰撞门**：非零/unknown cell 均为占用，OBB SAT 完整，触碰即碰撞，旋转与平移都做保守共同采样和膨胀检查。
- **不冒充可执行**：搜索结果和节点状态标注 `GEOMETRY_ONLY`/`GEOMETRY_ONLY_UNCERTIFIED`，状态恒有 `executable=false`、`trajectory_published=false`，没有任何时间轨迹发布者。
- **不污染单车**：单车 `.msg`、`planner_node.cpp`、优化器、`demo.launch`、`params*.yaml` 和单车审核容差未改；5 个消息 SHA-256 不变。
- **不跳过独立审核**：S4 只做几何候选，不声称运动学、时间同步、执行器或真实世界安全；只有后续完整独立审核才能发布 `FEASIBLE`。

## 存在风险

1. 搜索是几何启发式，状态键取整可能合并不同连续历史，启发函数未证明可采纳；不能声称最优或完备。
2. 第一版只搜索编队原点 pivot，窄空间中的原地转向机动可能找不到可用几何路径。
3. 同步搜索最多可占用请求回调 8 s；多请求/大地图下应考虑独立工作线程和取消协议，但 S4 未实现。
4. `allow_reverse`、`goal_heading_free` 的完整语义和偏心 pivot 候选均未进入 S4，S5/S6 必须补齐。
5. 静态地图和时间轨迹缺失意味着 S4 种子不能直接交给控制器或回放执行。
6. 旋转扫掠的保守膨胀基于角点位移上界；它不建模轮胎、执行器跟踪误差或真实定位误差。
7. 节点 latch 种子没有失效/版本协议；下游必须用 `plan_id`、`formation_id` 和重新校验阻止旧种子误用。
8. 合成探针只验证单请求成功/拒绝路径，不代表真实地图性能或任意 N 的可靠性。

## 完成判据与 S5 前置条件

- C++ 编队模型、请求校验、OBB/扫掠碰撞：**已完成并测试**。
- 仅 `q_F` 几何搜索、起点/终点静态检查、障碍绕行、旋转扫掠区分、预算和失败原因：**已完成并测试**。
- 独立 `formation_params.yaml` 和单独 `formation_demo.launch`：**已完成；单车入口未改**。
- 只发布 `GEOMETRY_ONLY_UNCERTIFIED` 种子，无时间轨迹/`FEASIBLE`：**已由代码、测试和 ROS 探针确认**。
- 单车消息与策略回归：**无新增失败；既有 `dynamic_peaks` 错误保留**。
- S5 前置条件：必须重新校验 `FormationPlanSeed` 的 `plan_id`、地图、起点/终点和 primitive 合同；必须独立分配并审核每车 A/O/S 与 `theta_i`，不能把 S4 的刚性航向约定当作模式可行；`ROTATE` 中非 pivot 车辆不能仅因中心绕转就标为 Spin；纯旋转 block 需要 S6 的混合进度参数，不能拿平移弧长 `s=0` 做除零或伪参数化。
- S5/S6 若需要更多候选，应把 `max_candidates` 扩展为显式候选保存/反馈机制，而不是把单次成功当作唯一解。
- 未启动 S5；下一步需用户明确给出新的阶段推进指令。