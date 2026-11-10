# S3 阶段审查报告：ROS 多车消息与输入契约

## 结论

**S3 完成：新增 7 个编队 `.msg`、ROS 无关的请求/轨迹结构校验器和一个只验证、不搜索的 ROS 节点；30 项定向测试全部通过，catkin 构建退出码 0，新旧消息均可导入和序列化/反序列化，真实 ROS 探针确认只发布 `/formation/status`。**完整 Python 回归为 224 项，仍只有 S0 已记录的同一个 `dynamic_peaks` 错误。本阶段未启动路径搜索、模式分配、同步重定时、碰撞发布或任何多车 `FEASIBLE` 结论。

## 任务编号

S3：ROS 多车消息、输入与发布契约。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L545-L566)。

## Git 检查点

- S2 已完成提交：`23873510f3d3210f7fec43af5576e6f175a69774`
- S2 分支：`feat/formation-collision`
- S3 分支：`feat/formation-msgs`
- S3 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程；两个原有未跟踪 Markdown 文件未清理、覆盖或提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| `fwis_v3_msgs/CMakeLists.txt` | 原为单行 `add_message_files(FILES ...)`；`generate_messages(DEPENDENCIES std_msgs geometry_msgs)` 与 `catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)` 已覆盖新消息 |
| `fwis_v3_msgs/package.xml` | 新消息只使用已有 `std_msgs`、`geometry_msgs`；无新增依赖，未修改 |
| 既有单车消息 | 5 个 `.msg` 文件 SHA-256 与修改前完全一致；未改一字节 |
| 单车话题 | `/plan/*` 订阅/发布、`PlanSeed`、`TimedTrajectory`、`TimedPoint` 未修改 |
| 现有节点风格 | 订阅回调 + latch `std_msgs/String` 状态发布；限定 `~` 私有参数 |
| S1/S2 合同 | `FormationModel`、`FormationPose`、`VehicleOffset`、`VehicleShape` 均复用；S1/S2 源文件未改 |

## 修改文件

新增：

1. [FormationVehicleSpec.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationVehicleSpec.msg)
2. [FormationPlanRequest.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationPlanRequest.msg)
3. [FormationPrimitive.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationPrimitive.msg)
4. [FormationPlanSeed.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationPlanSeed.msg)
5. [FormationTimedPoint.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationTimedPoint.msg)
6. [FormationVehicleTrajectory.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationVehicleTrajectory.msg)
7. [FormationTimedTrajectory.msg](../../catkin_ws/src/fwis_v3_msgs/msg/FormationTimedTrajectory.msg)
8. [formation_request.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_request.py)：ROS 无关的严格结构校验与 `FormationModel`/`VehicleShape` 重建。
9. [formation_optimizer_node.py](../../catkin_ws/src/fwis_v3_optimizer/scripts/formation_optimizer_node.py)：只验证并发布结构化状态的 ROS 适配器。
10. [test_formation_contract.py](../../tests/test_formation_contract.py)：30 项定向测试。
11. [logs/formation_contract_tests.txt](logs/formation_contract_tests.txt)
12. [logs/unit_tests.txt](logs/unit_tests.txt)
13. [logs/catkin_build.txt](logs/catkin_build.txt)
14. [logs/ros_message_import.txt](logs/ros_message_import.txt)
15. [logs/ros_node_probe.txt](logs/ros_node_probe.txt)
16. [logs/single_vehicle_msg_hashes.txt](logs/single_vehicle_msg_hashes.txt)
17. [S3_REPORT.md](S3_REPORT.md)

修改：

1. [fwis_v3_msgs/CMakeLists.txt](../../catkin_ws/src/fwis_v3_msgs/CMakeLists.txt)：仅把 7 个新文件加入原 `add_message_files(FILES ...)`。
2. [fwis_v3_optimizer/CMakeLists.txt](../../catkin_ws/src/fwis_v3_optimizer/CMakeLists.txt)：把新节点加入原 `catkin_install_python(PROGRAMS ...)`。

未修改任何既有 `.msg`、`package.xml`、单车节点、launch、YAML、S1/S2 模块、C++ 或既有测试。

## 消息合同

七个消息按 §5.2 字段顺序建立，关键语义冻结在 `.msg` 注释中：

- `FormationVehicleSpec`：`offset_in_formation.z` 恒为 0；`start_center.theta` 和 `goal_center.theta` 是车体航向 `θ_i`，不是 `ψ_F`；`goal_heading_free` 仅释放终端 `θ_i`，不释放车辆中心刚体位置。
- `FormationPlanRequest`：`plan_id` 是请求 ID，`formation_id` 是固定 offset 配置身份；`formation_start/goal` 的 `theta` 是 `ψ_F`；`allow_reverse` 本阶段只携带，不解释。
- `FormationPrimitive`：常量 `TRANSLATION=0`、`ROTATION=1`、`COMPOSITE=2`；它是编队参考运动 primitive，不复用单车 `ModePrimitive` 的 A/O/S 语义。
- `FormationPlanSeed`：几何-only 种子，不是可执行时间轨迹。
- `FormationTimedPoint`：`center_pose.theta` 是 `θ_i`；模式常量与 `ModePrimitive` 的 A/O/S 数值一致。
- `FormationVehicleTrajectory`：`vehicle_id` 绑定该车点列。
- `FormationTimedTrajectory`：`formation_poses[i].theta` 是 `ψ_F`；`times_from_start` 是唯一共同时间网格；每个 `points[j].time_from_start` 必须与该索引网格完全一致。

ROS 生成结果在 [logs/ros_message_import.txt](logs/ros_message_import.txt) 中逐字段展示；Python `_slot_types` 与 C++ 头文件存在性由定向测试冻结。

## 输入校验合同

### 请求结构

`validate_formation_request()` 只接受 ROS 消息或字段结构相同的只读对象，失败返回 `RequestValidation(reason="INVALID_REQUEST")`，带 `field`、可选 `vehicle_id` 和 `detail`。严格检查顺序：

1. `header.frame_id == "map"`；`header.stamp` 非零且 `nsecs ∈ [0, 1e9)`。
2. `plan_id` 为 uint32；`formation_id` 为非空字符串；二者保持独立字段。
3. `formation_start/goal` 的 `x/y/theta` 全部有限。
4. 车辆列表非空；每辆车的 `vehicle_id` 为 uint32 且不重复。
5. 每个 offset 有限且 `z == 0`。
6. 每辆车 `start_center`/`goal_center` 的 `x/y/θ` 全部有限；`body_length_m`/`body_width_m` 严格为正且有限；`goal_heading_free` 必须是真正的 `bool`。
7. 用消息内容重建 `FormationModel`，由它拒绝重复 offset/重合 offset；用消息内容重建 `VehicleShape`。
8. 逐车检查 `start_center` 和 `goal_center` 的 `x/y` 是否等于 `q_F 位置 + R(ψ_F) r_i`，默认容差 `1e-6 m`。即使 `goal_heading_free=true`，中心仍必须满足该刚体关系。
9. 成功时返回 `FormationRequestSnapshot`，其中包含 `model`、`vehicle_shapes`、按 ID 排序的车辆、独立 `start_headings`/`goal_headings`、`goal_heading_free` 和 `allow_reverse`。

因此镜像目标、车辆 ID 换标、起点/终点中心偏移错误、`goal_heading_free` 误用、重复 ID、重合 offset、非有限值和错误 frame 都在搜索前明确失败。

### 时间轨迹结构

`validate_formation_timed_trajectory()` 返回 `reason="INVALID_TIMED_TRAJECTORY"`，检查：

- `header`、`plan_id`、非空 `formation_id` 与请求同规则。
- 车辆规格非空、ID 不重复、offset 可重建 `FormationModel`。
- `times_from_start` 非空、非负、严格递增。
- `formation_poses` 数量等于时间网格数量，且每项有限。
- `trajectories` 数量等于车辆数量；轨迹 `vehicle_id` 不重复并与车辆 ID 集合完全一致。
- 每条轨迹的 `points` 数量等于共同时间网格数量。
- 每个点的 `time_from_start`（`secs/nsecs`）与该索引的全局时间完全相等。
- 每个点的 `center_pose`、速度、角速度、`mode_control` 有限；`mode` 仅为 0/1/2。

该函数是结构合同，不是运动学、碰撞、同步重定时或独立执行审核；`as_status()` 明确给出 `executable=false`、`independent_audit=false`。

### ROS 节点

`formation_optimizer_node.py`：

- 订阅：`/formation/plan_request`（`FormationPlanRequest`）。
- 发布：`/formation/status`（latch `std_msgs/String`，JSON）。
- 状态字段：`schema`、`valid`、`reason`、`field`、`vehicle_id`、`detail`、`plan_id`、`formation_id`、`node_state`、`requests_seen`，并恒有 `search_started=false`、`trajectory_published=false`、`executable=false`。
- 合法请求状态为 `VALIDATED_NOT_PLANNED`；非法请求为 `REJECTED`。
- 不发布 `FormationPlanSeed` 或 `FormationTimedTrajectory`；没有可清理的旧成功轨迹。
- 不读取地图、不调用 S2 碰撞检查、不创建搜索线程。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `source /opt/ros/noetic/setup.bash; source catkin_ws/devel/setup.bash; OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_contract.py' -v` | 30/30 通过，0 skipped；测试计时 0.005 s，墙钟 0.19 s；最大 RSS 57904 KiB | 0 |
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 224 项，223 通过、1 错误；测试计时 7.842 s，墙钟 8.05 s；最大 RSS 211452 KiB | 1 |
| `source /opt/ros/noetic/setup.bash; catkin_make -C .../catkin_ws -j1` | 构建系统检查与增量 `make` 完成；墙钟 2.44 s；最大 RSS 16960 KiB | 0 |
| `python3 -c 'from fwis_v3_msgs.msg import FormationPlanRequest, FormationTimedTrajectory, ...'` | 新旧消息全部导入成功 | 0 |
| `rosmsg show fwis_v3_msgs/FormationPlanRequest` / `FormationTimedTrajectory` / `FormationVehicleSpec` | 字段与 §5.2 一致 | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归错误与 S0 完全相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 224 tests in 7.842s
FAILED (errors=1)
```

224 = S2 的 194 项 + 本阶段新增 30 项。30 项定向测试在完整回归中全部执行、无 skip；这是因为测试会在未 source ROS 时显式加入已构建的 devel/ROS Python 路径。新增测试无 failure/error；既有错误未修复。`catkin_make` 为增量调用，不能据此推断全量构建耗时。

## ROS 节点探针

在独立端口 11411 上启动隔离 `rosmaster`、运行新节点，用探针发布一条合法 N=3 请求和一条 `frame_id="odom"` 的非法请求。实际输出见 [logs/ros_node_probe.txt](logs/ros_node_probe.txt)：

- 收到 2 条状态；合法状态 `valid=true`、`node_state=VALIDATED_NOT_PLANNED`、`search_started=false`、`trajectory_published=false`、`executable=false`。
- 非法状态 `valid=false`、`reason=INVALID_REQUEST`、`field=header.frame_id`、`node_state=REJECTED`，同样不启动搜索、不发布轨迹。
- 节点进程的已发布话题只有 `/formation/status`（`std_msgs/String`）和 `/rosout`；没有 `/plan/*`、没有 `FormationPlanSeed` 或 `FormationTimedTrajectory`。
- 探针退出码 0；清理后端口 11411 关闭，无残留 `rosmaster`/节点进程。

这是单机合成探针，不是真实地图、RViz、回放或端到端规划性能结论。

## 定向测试对应验收项

| 验收要求 | 测试 |
|---|---|
| 原 `PlanSeed/TimedTrajectory` 仍正常导入 | `test_single_vehicle_messages_still_import_unchanged` |
| 单车消息文件一字节不改 | `test_existing_single_vehicle_message_bytes_are_untouched`（SHA-256） |
| 新消息可序列化/反序列化 | `test_request_roundtrip_and_validator_accept_real_ros_message`、`test_timed_trajectory_roundtrip_and_validator_accept_ros_message`、`test_seed_and_primitive_roundtrip` |
| 新消息 Python/C++ 类型存在 | `test_generated_slot_types_are_frozen`、`test_generated_cpp_headers_exist_after_build` |
| 常量值不漂移 | `test_constant_values_are_frozen` |
| 请求反序列化后重建 `FormationModel` | `test_request_roundtrip_and_validator_accept_real_ros_message`、`test_valid_request_rebuilds_s1_s2_objects_and_both_ids` |
| `goal_heading_free` 不放松中心刚体约束 | `test_goal_heading_free_never_relaxes_center_rigidity` |
| 起点/终点 offset 错误 | `test_start_and_goal_centers_enforce_fixed_offsets` |
| 镜像目标/换标 | `test_mirrored_goal_positions_are_rejected_against_rigid_model` |
| 空车列表、重复 ID、非法 ID | `test_duplicate_empty_and_bad_vehicle_ids_fail` |
| 重合 offset | `test_coincident_offsets_are_rejected_by_rebuilt_model` |
| `plan_id`/`formation_id` 不混淆 | `test_plan_id_and_formation_id_remain_distinct_fields` |
| 必填/数值/布尔/时间戳严格性 | `test_header_and_spec_fields_are_strict`、`test_missing_fields_and_none_request_fail_closed`、`test_formation_start_and_goal_must_be_finite` |
| 路径长度不同 | `test_different_path_lengths_fail` |
| 时间网格非单调/重复/负值 | `test_time_grid_must_be_nonnegative_and_strictly_monotonic` |
| 逐点时间戳与共同网格不一致 | `test_every_point_timestamp_must_equal_the_common_grid` |
| 轨迹与车辆 ID 集合不一致 | `test_vehicle_and_trajectory_id_sets_must_match` |
| `formation_poses` 网格不一致 | `test_formation_pose_grid_must_match_the_common_times` |
| 模式与数值字段非法 | `test_point_modes_and_numeric_fields_are_strict` |
| 轨迹 header/空时间网格 | `test_header_and_empty_time_grid_fail` |

## 未执行项目与边界

1. 未进入地图搜索；没有 primitive、候选、代价或启发函数。
2. 未实现逐车 A/O/S 模式分配、共同时间轴、同步重定时或独立轨迹审核。
3. 未调用 S2 地图/车对 OBB 碰撞检查；S3 只校验请求结构，不判断起点或终点是否撞墙、车体是否重叠。
4. 未创建 `formation_demo.launch`、`formation_params.yaml`、RViz 配置或回放脚本；这些属于后续阶段。
5. 未实现 C++ `formation_model.hpp`/前端；消息已冻结，等待 S4 使用。
6. 未发布 `FormationPlanSeed` 或 `FormationTimedTrajectory`；没有 `GEOMETRY_ONLY` 或 `FEASIBLE` 输出。
7. `allow_reverse` 仅被保存和回显，未参与任何可行性判断。
8. `formation_id` 仅校验非空字符串，没有配置注册表，因此尚不能断言它与 offset 集合来自同一已登记配置。
9. `goal_heading_free=true` 时仍要求 `goal_center.theta` 有限；自由的是终端约束，不是数值合法性。
10. 时间轨迹校验只检查结构同步，不证明点间连续、不碰撞、运动学可实现或轮级可执行。

## 安全性审计

- 所有请求字段 fail-closed：缺失、NaN/Inf、错误布尔类型、错误 frame、零 stamp、空列表均明确拒绝。
- 车辆 ID 集合、offset 刚体关系和车体尺寸由消息内容重建的 S1/S2 对象决定，不信任消息中的派生位置。
- `goal_heading_free` 不能放宽中心刚体位置；镜像目标会在搜索前被拒绝。
- `plan_id` 与 `formation_id` 在快照和状态中分别保留，没有互相替代。
- 时间轨迹拒绝不同路径长度、重复/缺失车辆、非严格递增网格和逐点时间戳不一致。
- 节点状态恒标 `search_started=false`、`trajectory_published=false`、`executable=false`；合法请求也只到 `VALIDATED_NOT_PLANNED`。
- 没有修改、放宽或绕过单车审核、margin、扫掠或终点容差合同。

## 存在风险

1. 1e-6 m 是请求消息的数学一致性容差，不是定位、安全或碰撞容差；真实安全仍取决于地图、OBB 和独立审核。
2. 零 `header.stamp` 被拒绝，上游必须显式写入共同生成时间或轨迹起始时间；若沿用默认零值会得到明确失败。
3. 状态是 `std_msgs/String` JSON，不是独立类型化消息；下游必须按 `schema` 字符串和固定字段解析，消息演化需另立迁移任务。
4. 消息没有版本号字段；字段语义已由源码、测试和 SHA-256 冻结，但仍应避免无迁移的直接改字段。
5. `mode ∈ {0,1,2}` 只是结构检查，不能证明车轮级运动学或执行器可实现。
6. 结构校验接受任意 N>=1，但第一可运行版本目标仍是 N=1 和 N=3；不能据此宣称任意 N 已可靠支持。
7. 未在真实地图、真实消息回放或多进程负载下测试；ROS 探针只在本机隔离 master 上执行一次。

## 完成判据与 S4 前置条件

- 7 个新消息、消息构建、新旧导入、序列化/反序列化：**完成**。
- 请求重建 `FormationModel`/`VehicleShape`、ID 集合、刚体起终中心、`goal_heading_free` 语义：**已验证**。
- 时间网格/路径长度/逐点时间戳合同：**已验证**。
- 单车消息字节不变：**已验证**。
- 只验证不搜索的 ROS 节点与结构化状态：**已完成并通过探针**。
- 旧回归新增失败：**无**；既有 `dynamic_peaks` 错误仍保留。
- S4 前置条件：直接使用 `validate_formation_request()` 返回的 `FormationRequestSnapshot`（含 `model`/`vehicle_shapes`），不要再次从原始消息自行解释位置；S4 的 C++ 前端必须与这些字段语义一致。S4 的第一版 `FormationPrimitive` 若需要 pivot，必须在开始搜索前完成消息迁移，不能把 pivot 藏进 `amount/control`。S4 仍只能发布几何-only 结果，不能把 `VALIDATED_NOT_PLANNED` 或结构通过解释为可执行轨迹通过。
