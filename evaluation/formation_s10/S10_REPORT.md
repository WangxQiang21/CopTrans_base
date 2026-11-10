# S10 阶段审查报告：RViz 回放、端到端 smoke 与固定用例回归

## 结论

**S10 完成：新增 ROS 无关的 `formation_playback.py` 回放核心与 ROS 回放节点，所有车辆只从 `FormationTimedTrajectory.header.stamp` 和一个共享 `time_from_start` 轴插值；Marker namespace 含 `formation_id` 与 `vehicle_id`，TF frame 按编队唯一；升级 `formation_demo.launch` 为编队演示 launch，并新增 `formation_demo.rviz`、`ros_formation_smoke.py`、`run_formation_regression.py`、`formation_eval_cases.yaml`。回放只有收到匹配 `plan_id` 且 `feasible/executable/trajectory_published` 的 S9 状态时才显示 `FEASIBLE`，否则显示 `AUDIT_PENDING` / `GEOMETRY_ONLY`。**

**固定用例 E01–E20 在 3 次重复下 20/20 通过声明期望，75/75 个有效 outcome 符合预期；无 Spin 消融 72/75（E03 三次均 `MODE_DISABLED`），无 O 消融 66/75（E04/E07/E08 三次均 `MODE_DISABLED`）。ROS smoke 在私有 master 11399 上 39/39 检查通过（10.56 s），覆盖 296 个共同时钟样本、终端精确保持、全车 Marker/TF、空结果清理和 `GEOMETRY_ONLY` 标签。定向测试 42/42 通过；完整 Python 回归 358 项，只有 S0 已记录的同一个 `dynamic_peaks` 错误，没有新增失败。`py_compile` 与 `git diff --check` 均通过。**

本阶段没有修改控制器、NMPC、执行器、实车接口、闭环状态仿真、既有 `.msg`、单车 `demo.launch`、`run_demo.sh` 或 `archive/`。按约束没有修改 `catkin_ws/build/` 与 `catkin_ws/devel/`，因此没有执行 `catkin_make` 和 `roslaunch` 实跑；smoke 直接启动源码节点，launch/RViz 资产由静态测试验证，详见“未执行项目与边界”。

## 任务编号

S10：RViz/脚本/性能实验。依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L869-L995)，阶段表见同文档 [S10 行](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L405)。

## Git 检查点

- S9 已完成提交：`8e726bb2faab2285b9140fb41567ea320829abf9`（`feat/formation-audit`）
- S10 分支：`feat/formation-eval`（从 S9 提交创建）
- S10 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 未推送远程
- 两个原有未跟踪 Markdown 文件 `CURRENT_REPOSITORY_STATUS.md` 与 `多车4WIS刚性编队协同规划_详细技术实施文档.md` 未清理、未覆盖、未提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| S8/S9 时间与状态契约 | `FormationTimedTrajectory` 由 S8 生成共享 `times_from_start` 与逐点 `time_from_start`；S9 只在独立审核通过且事务发布成功后给出顶层 `FEASIBLE`、`executable=true`、`trajectory_published=true` |
| 既有消息定义 | 未修改任何 `.msg`；回放使用现有 `FormationTimedTrajectory`、`FormationTimedPoint` 和 `FormationVehicleSpec` 字段 |
| 话题边界 | 回放读取 `/formation/timed_trajectory`、`/formation/optimizer_status`、`/formation/quality_report`，发布 `/formation/markers` 与 `/formation/playback_status`；C++ S4 前端继续使用 `/formation/status`，没有被覆盖 |
| S9 发布语义 | 优化器只发布 `FEASIBLE` 轨迹；失败/拒绝通过空 `vehicles` 消息清除旧轨迹，因此失败轨迹不会伪装成可执行轨迹进入回放 |
| 单车兼容 | 未修改 `playback.py`、`demo.launch`、`run_demo.sh`、单车话题或单车 `TimedTrajectory`；`fwis_v3_playback/CMakeLists.txt` 仍安装旧 `scripts/playback.py` |
| 构建边界 | 未修改 `catkin_ws/build/`、`catkin_ws/devel/`、`archive/`；未执行 `catkin_make` |
| 用户文件 | `CURRENT_REPOSITORY_STATUS.md` 和技术文档保持未跟踪且内容未变 |

## 修改文件

新增：

1. [formation_playback.py（回放核心）](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py)：`PlaybackVehicleSpec`、`PlaybackPoint`、`FormationPlaybackTrajectory`、`PlaybackFrame`、`PlaybackMarkerOptions`、`MarkerSpec`、`trajectory_from_fields`、`frame_at`、`terminal_errors`、`build_markers`、`clear_marker_specs`、`status_document`、`normalize_formation_id`、`vehicle_namespace`、`tf_frame_prefix`。
2. [formation_playback.py（ROS 节点）](../../catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py)：`fields_from_message`、`FormationPlaybackNode`、共同时钟 timer、TF broadcaster、空轨迹清理。
3. [formation_demo.rviz](../../catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz)：固定 `map` 帧，显示 `/map`、`/formation/markers` 和 TF；不引用任何单车显示话题。
4. [ros_formation_smoke.py](../../tests/ros_formation_smoke.py)：私有 ROS master、源码节点启动、消息/回放/Marker/TF/清理/`GEOMETRY_ONLY` 共 39 项检查、严格 JSON 报告。
5. [run_formation_regression.py](../../tools/run_formation_regression.py)：固定用例加载、几何/重定时/独立审核/节点事务直驱、3 次重复、CPU/内存/分位数/拒绝原因/失败保留。
6. [formation_eval_cases.yaml](../../tools/formation_eval_cases.yaml)：E01–E20 固定用例，schema `formation_eval_cases/v1`，SHA256 `6b239f5d30905cd0a325fe2ed9cd08d406057309db25f358a6dd8b4d3ee98b8f`。
7. [test_formation_playback.py](../../tests/test_formation_playback.py)：回放核心的共同时钟、终端保持、Marker/TF 命名、`GEOMETRY_ONLY`、清理和严格校验测试。
8. [test_formation_regression.py](../../tests/test_formation_regression.py)：用例冻结、字段变异、失败保留、统计分组与 CLI 约束测试。
9. [test_formation_demo_assets.py](../../tests/test_formation_demo_assets.py)：launch 唯一进程、禁止单车节点、RViz 话题与 CMake 安装检查。
10. [logs/](logs/)：本阶段全部实际命令日志与 ROS smoke 节点日志。
11. [regression/summary.json](regression/summary.json)、[ablation_no_spin/summary.json](ablation_no_spin/summary.json)、[ablation_no_oblique/summary.json](ablation_no_oblique/summary.json)、[smoke/ros_formation_smoke.json](smoke/ros_formation_smoke.json)。
12. [S10_REPORT.md](S10_REPORT.md)。

修改：

- [formation_demo.launch](../../catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch)：从 S4 的 geometry-only 前端 launch 升级为 S10 演示 launch。新增 `start_map/frontend/optimizer/playback/enable_nlp/max_candidates/optimizer_time_limit/require_collision_precheck/playback_rate/playback_publish_tf` 参数；最多一个地图发布器、一个 C++ 编队前端、一个 Python 编队优化器、一个编队回放；RViz 默认 `false`，使用 `formation_demo.rviz`；不启动任何单车规划/优化/回放节点。
- [formation_optimizer_node.py](../../catkin_ws/src/fwis_v3_optimizer/scripts/formation_optimizer_node.py)：新增 `~allowed_modes` 参数与阶段耗时记录（`build_s/master_s/nlp_s/retiming_s/selection_s/audit_s/total_s`），把 `allowed_modes` 贯穿 S6/S7 和管线；默认 `None` 保持既有行为。
- [formation_kinematics.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py)：新增 `normalize_allowed_modes` 与模式过滤；A/O/S 消融只会产生显式 `MODE_DISABLED`，不会悄悄换成另一模式。
- [formation_geometry.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_geometry.py)、[formation_optimizer.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_optimizer.py)：把 `allowed_modes` 传入候选构建/优化；默认行为不变。
- [test_formation_kinematics.py](../../tests/test_formation_kinematics.py)：新增允许模式过滤/禁用旋转、A/O 模式与非法参数校验的测试。
- [fwis_v3_playback/CMakeLists.txt](../../catkin_ws/src/fwis_v3_playback/CMakeLists.txt)、[package.xml](../../catkin_ws/src/fwis_v3_playback/package.xml)：安装 `scripts/formation_playback.py`，新增 `tf2_ros` 依赖；旧 `playback.py` 保留。

## 接口变化

新增（仅 `/formation/*` 与编队 TF）：

| 接口 | 类型/参数 | 说明 |
|---|---|---|
| `/formation/markers` | `visualization_msgs/MarkerArray` | 编队回放 Marker；不 latch，25 Hz 发布 |
| `/formation/playback_status` | `std_msgs/String`，latch | schema `formation_playback/v1`，含 state/时间/终端误差/认证字段 |
| `formation_<safe_id>_reference` | TF frame | 编队参考点 |
| `formation_<safe_id>_vehicle_<id>_base_link` | TF frame | 每车车体位姿 |
| `~frame_id` | playback param | 默认 `map`，必须与消息 `header.frame_id` 一致 |
| `~rate` | playback param | 默认 25 Hz，必须为正有限值 |
| `~publish_tf` | playback param | 默认 true |
| `~allowed_modes` | optimizer param | `None` 表示 A/O/S 全开；仅测试/消融使用 |
| `~band_width/history_width/body_width/heading_width/link_width/reference_scale/text_size/status_text_size/text_height/arrow_head_length` | playback params | Marker 几何尺寸 |

`formation_optimizer_status/v1`、`formation_quality_report/v1`、`formation_pipeline_result/v1` 等既有 schema 未破坏；回放只读取，不写回。没有修改任何 `.msg`、单车话题、单车 launch 或 C++ 接口。不兼容变化：无。

## 回放实现

### 共同时钟与插值

`FormationPlaybackTrajectory` 只接受一个显式非零 `header.stamp`、非空车辆集合、全部车辆共享的严格递增 `times_from_start`，以及每车与共享时间轴等长的点序列。`frame_at` 用 `bisect_right` 找区间，在线性插值中心与航向（航向走最短角差）；没有任何“收到消息时开始”的时钟：

```text
elapsed = now - trajectory.header.stamp
state   = HOLD_START (elapsed < 0)
          PLAYING    (0 <= elapsed < T_end)
          HOLD_END   (elapsed >= T_end)
clamped_elapsed = clamp(elapsed, 0, T_end)
```

`HOLD_END` 时强制返回最后一个共享样本，`fraction=0`，不做外推。`terminal_errors` 再独立比较回放终端与消息最后样本的中心/航向，smoke 中为 0。

### Marker 与 TF 合同

- 每车 namespace：`formation/<formation_id>/vehicle_<vehicle_id>`。
- 编队共享 namespace：`formation/<formation_id>/reference`、`/links`、`/status`。
- 每车 Marker：车体 OBB 闭合 LINE_STRIP（5 点）、航向 ARROW、完整路径 LINE_STRIP（逐样本 A/O/S 颜色）、已行驶历史 LINE_STRIP、模式/控制 TEXT。
- 共享 Marker：车间连线 LINE_LIST、编队参考点 SPHERE + 参考轨迹 + `F` 标签、状态 TEXT_VIEW_FACING。
- Marker ID 在整个 `MarkerArray` 内唯一；smoke 实测 20 个 Marker、20 个唯一 ID。
- TF frame：`formation_<safe_formation_id>_reference`、`formation_<safe_formation_id>_vehicle_<id>_base_link`。`normalize_formation_id` 对非 TF 安全字符追加确定性 SHA1 短后缀，避免两个不同 `formation_id` 静默共用一个 frame。
- 清空使用 `DELETEALL`，namespace 为 `formation`，并发布 `CLEARED` 状态。

### 状态显示与 `GEOMETRY_ONLY`

`_status_label` 只有在以下全部成立时才显示 `FEASIBLE`：

1. 收到 `formation_optimizer_status/v1`；
2. 状态的 `plan_id` 等于当前轨迹 `plan_id`；
3. `feasible=true`；
4. `executable=true`；
5. `trajectory_published=true`。

否则显示 `AUDIT_PENDING`（无匹配状态）或 `GEOMETRY_ONLY`（有状态但未认证/未发布）。回放本身从不写 `feasible`、`executable`、`collision_certificate` 或 `trajectory_published`。smoke 用一条合成的、无优化器状态的两点轨迹验证了 `GEOMETRY_ONLY`/`AUDIT_PENDING` 显示，并在空轨迹后清除。

状态文本显示 `T_end`、`rigidity`、`obstacle`、`intervehicle`、`dynamic_ratio`、`mode_switch`。优化器状态本身不含 `pipeline` 指标，回放节点在 `plan_id` 匹配时把 latched 的 `formation_quality_report/v1` 的 `pipeline` 块合并进显示副本，不改动任何认证字段。smoke 中终端文本为：

```text
plan 424242 | HOLD_END | FEASIBLE
T_end=5.889 s | FEASIBLE / independent audit and transactional publication
T_end=5.89 rigidity=4.44e-15 obstacle=4.55 intervehicle=0.7 dynamic_ratio=0.995 mode_switch=0
```

## 演示与实验资产

### launch 与 RViz

`formation_demo.launch` 的节点数量由 `test_formation_demo_assets.py` 解析 XML 验证：

- `fwis_map_publisher` 恰好 1 个（可关闭）；
- `formation_planner`（`formation_planner_node`）恰好 1 个；
- `formation_optimizer`（`formation_optimizer_node.py`）恰好 1 个；
- `formation_playback`（`formation_playback.py`）恰好 1 个；
- 禁止 `optimizer_node.py`、`playback.py`、`collect_node.py` 等单车节点；
- `rviz` 默认 `false`，默认加载 `formation_demo.rviz`。

RViz 配置固定 `map` 帧，显示 Map `/map`、MarkerArray `/formation/markers`、TF；测试还断言其中不含 `/plan/markers`、`/plan/mode_strip`、`/robot/heading_marker`、`/plan/timed_trajectory` 等单车话题。

### ROS smoke

`tests/ros_formation_smoke.py` 的行为：

1. 拒绝受保护端口 11311/11312/11320；默认使用私有 11399，并先检查端口空闲。
2. 自己 source `/opt/ros/noetic/setup.bash` 和 `catkin_ws/devel/setup.bash`，为子进程补齐 `PYTHONPATH`，设置独立 `ROS_HOME`/`ROS_LOG_DIR` 和单线程 BLAS。
3. 只启动 `roscore`、一个 `formation_optimizer` 源码节点、一个 `formation_playback` 源码节点；不启动任何单车 planner。
4. 发布 320×240、0.05 m、origin `(-8,-6)` 的空 `/map`，请求 `plan_id=424242`、`formation_id=smoke_wedge`、3 车 offset `(0,0)/(0,1.2)/(0,-1.2)`、目标平移 2.5 m，并发布匹配 seed。
5. 订阅优化器状态、质量报告、时间轨迹、MarkerArray、回放状态和 TF；检查 39 项。
6. 发送非法请求，检查空轨迹、`INVALID` 状态、回放 `CLEARED` 和 `DELETEALL`。
7. 发送一条合成的无状态两点轨迹，检查 `GEOMETRY_ONLY` 标签，再发空轨迹清理。
8. SIGINT 节点、确认子进程退出码 0、私有 master 端口释放。

check 列表覆盖：私有端口；单优化器+单回放；`FEASIBLE` 发布；`map_ready`/地图签名；独立审核与碰撞证书；轨迹身份；296 个共享时间样本；有限值与 A/O/S；车辆 ID/offset；起终刚性；回放接受计划；共同时钟样本；回放刚性；终端保持与消息一致；Marker frame/namespace/唯一 ID/OBB/航向/模式色带/历史/连线/参考点/状态指标；TF 唯一 frame 与车体位置；非法请求拒绝与清空；`GEOMETRY_ONLY`；空轨迹清理；进程停止；端口释放。

### 固定用例与回归 harness

`formation_eval_cases.yaml` 定义 E01–E20，默认地图固定为 320×240、0.05 m、origin `(-8,-6)`；默认 `max_candidates=2`、`max_schedule_combinations=8`、`max_schedules_per_block=2`、`planning_dt=0.02`、`enable_nlp=false`、`require_collision_precheck=true`，质量参数为 margin 0.05 m、sweep 0.01 m、publication dt 0.02 s、最多 200000 采样/段。

`run_formation_regression.py` 直接驱动生产 Python 层：请求校验 → S6 连续几何/候选 → 可选 S7 NLP → S8 共同时钟重定时 → S9 独立审核/选解 → 节点事务核心。每个 case 至少 3 次重复。失败行原样保留；`all_pass` 只表示声明的期望是否满足，不是“所有轨迹都成功”。`expanded_nodes` 明确记为 `null`，因为该 Python 后端消费直接 seed、不执行图搜索，不伪造节点数。

#### 用例与结果

| ID | 用例 | 类型 | 3 次重复结果 | 失败码/说明 |
|---|---|---|---|---|
| E01 | 单车基线直线 | pipeline/success | 3/3 FEASIBLE | `T_end=4.711 s` |
| E02 | 三车横向楔形直线 | pipeline/success | 3/3 FEASIBLE | `T_end=5.889 s`，车间净空 0.70 m |
| E03 | V 形三车刚体旋转 | pipeline/success | 3/3 FEASIBLE | Spin + 非中心 A，`T_end=2.744 s` |
| E04 | 宽弯平移-旋转-平移 | pipeline/success | 3/3 FEASIBLE | O 参与，4 次换模，`T_end=17.246 s` |
| E05 | 不可通过 0.8 m 门洞 | pipeline/must_fail | 3/3 REJECTED | `COLLISION_PRECHECK_FAILED`，候选 0 |
| E06 | 1.2 m 门洞特定队形 | pipeline/success | 3/3 FEASIBLE | `T_end=7.066 s`，障碍净空 0.35 m |
| E07 | 90° 急弯折线 | pipeline/success | 3/3 FEASIBLE | 4 次换模，`T_end=19.464 s` |
| E08 | S 形窄通道 | pipeline/success | 3/3 FEASIBLE | 障碍净空 0.15 m，车间净空 0.10 m |
| E09 | 起点 OBB 重叠 | evaluator/must_fail | 3/3 REJECTED | `START_FORMATION_COLLISION` |
| E10 | 中段扫掠撞障碍 | evaluator/must_fail | 3/3 REJECTED | `FORMATION_SWEEP_COLLISION` |
| E11 | 起点中心不满足刚体约束 | request_invalid | 3/3 INVALID | `INVALID_REQUEST`，`start_center` |
| E12 | 目标队形镜像 | request_invalid | 3/3 INVALID | `INVALID_REQUEST`，`vehicles[1].goal_center` |
| E13 | A 模式控制量篡改 | evaluator/must_fail | 3/3 REJECTED | `KINEMATIC_INFEASIBLE` |
| E14 | O 模式车体航向篡改 | evaluator/must_fail | 3/3 REJECTED | `KINEMATIC_INFEASIBLE` |
| E15 | S 模式中心速度篡改 | evaluator/must_fail | 3/3 REJECTED | `KINEMATIC_INFEASIBLE` |
| E16 | 共同时间轴篡改 | evaluator/must_fail | 3/3 INVALID | `INVALID_TIMED_TRAJECTORY` |
| E17 | 严格驻留下换模 | evaluator/must_fail | 3/3 REJECTED | `MODE_SCHEDULE_INFEASIBLE` |
| E18 | NaN/空列表/旧 `plan_id` | node/scenario_pass | 3/3 清理 | `INVALID_REQUEST`、`STALE_PLAN` |
| E19 | N=1/2/3 统计 | group/statistical | 9/9 子运行 FEASIBLE | `N1_N2_N3` |
| E20 | 队形尺寸/障碍密度 | group/statistical | 12/12 子运行 FEASIBLE | 4 组 × 3 次 |

E12 的 `goal_mirror` 保留声明的非对称 offset `(0,0)/(0.6,0.4)`，但把第二辆车目标中心放到 `q_goal + (0.6,-0.4) = (2.6,-0.4)`；请求校验在 `vehicles[1].goal_center` 失败，证明目标队形镜像不会被当作有效刚体目标。重复/错误车辆 ID 另由 `test_formation_model.py` 覆盖。

#### 成功轨迹指标摘录

| 用例 | 检查数 | 采样点 | 最大刚性误差 | 最小障碍净空 | 最小车间净空 | 动态峰值比 | 换模数 |
|---|---:|---:|---:|---:|---:|---:|---:|
| E01 | 9215 | 237 | 3.77e-15 m | 5.631 m | —（N=1） | 0.9950 | 0 |
| E02 | 30199 | 296 | 4.44e-15 m | 4.550 m | 0.700 m | 0.9950 | 0 |
| E03 | 15007 | 139 | 1.69e-15 m | 4.057 m | 1.009 m | 0.9851 | 0 |
| E04 | 52590 | 867 | 1.85e-15 m | 4.550 m | 1.900 m | 0.9868 | 4 |
| E06 | 23377 | 355 | 4.88e-15 m | 0.350 m | 0.262 m | 0.9950 | 0 |
| E07 | 60302 | 977 | 3.56e-15 m | 3.550 m | 1.900 m | 0.9950 | 4 |
| E08 | 30612 | 474 | 2.59e-15 m | 0.150 m | 0.100 m | 0.9901 | 0 |

所有成功用例的 `collision_certificate=true`、`independent_audit=true`，`dynamic_peak_ratio <= 1`，最大刚性误差均在 1e-14 m 量级。E01 无车间距，因此不填车间净空。

#### 性能与环境

环境：

- CPU：AMD Ryzen 7 7840H with Radeon 780M Graphics，16 逻辑核；
- OS：Linux 5.15.0-139-generic，x86_64，glibc 2.29；
- Python 3.8.10，NumPy 1.24.4，SciPy 1.10.1，CasADi 3.7.2，ROS Noetic；
- `OPENBLAS_NUM_THREADS=1`、`OMP_NUM_THREADS=1`；
- 进程峰值 RSS 81.64 MB；单次 Python `tracemalloc` 峰值最大 4.59 MB（E07）。

完整回归（`enable_nlp=false`，3 次/用例）：

| 指标 | 数值 |
|---|---:|
| 声明期望通过 | 20/20 |
| 有效 outcome | 75/75 |
| 最大单次墙钟 | 11.488 s（E20） |
| 最大单次 CPU | 11.478 s（E20） |
| 20 个 case 的中位数墙钟之和 | 24.873 s |
| 20 个 case 的中位数 CPU 之和 | 24.852 s |
| E20 墙钟 p95 | 11.488 s |

阶段拆分为 `build_s`（S6 几何含模式分配）、`master_s`（候选碰撞预检）、`nlp_s`（S7，默认关闭故为 0）、`retiming_s`（S8）、`selection_s`/`audit_s`（S9 审核与选解）、`total_s`。各阶段中位数之和不严格等于 `total_s`（存在未单独计时的请求校验等外层开销）。**未单独测量 C++ S4 前端与 ROS 传输时间**；该回归直接调用 Python 后端。ROS smoke 中从请求校验到 `FEASIBLE_PUBLISHED` 的日志时间约 0.55 s（3 车、2 候选、空地图），仅作端到端参考，不是正式基准。

#### 消融

| 配置 | 允许模式 | 声明期望通过 | 有效 outcome | 最大单次墙钟 | 中位数墙钟之和 | 关键拒绝 |
|---|---|---|---:|---:|---:|---:|---|
| 基线 | A/O/S | 20/20 | 75/75 | 11.488 s | 24.873 s | — |
| 禁用 Spin | A/O | 20/20 | 72/75 | 11.761 s | 24.383 s | E03 3/3 `MODE_DISABLED`；E15 也转为 `MODE_DISABLED` |
| 禁用 O | A/S | 20/20 | 66/75 | 5.824 s | 9.941 s | E04/E07/E08 各 3/3 `MODE_DISABLED`；E14/E17 也转为 `MODE_DISABLED` |

`no_oblique` 的总墙钟明显更低，主要因为 E04/E07/E08 在候选/模式阶段就失败，**不能**解读为“禁用 O 后成功路径更快”。`no_spin` 只损失 E03，总时间接近基线。消融为 record-only：成功类用例允许失败并记录，must-fail 用例仍必须按声明拒绝；失败样本全部保留在 summary 中。

## 本阶段自检发现并修正的问题

1. **首次 `/map` 不产生状态**：`FormationNodeCore.on_map` 对没有错误、快照、pending seed 或地图变化的首张地图返回 `None`，smoke 最初等待“地图状态”会超时。改为先发布地图并等待 0.3 s，再发请求/seed，最后在 `FEASIBLE` 状态上检查 `map_ready`/`map_signature`。
2. **私有 master 清理不彻底**：调试用的 setsid 进程在父 job 退出后仍占用 11399，导致下一次 smoke 拒绝启动。改为按实际 PID 清理，并在每次 smoke 结束（包括没有启动任何节点的情况）都执行 `_wait_port_closed`。
3. **回放共同时钟样本窗口过窄**：最初只等到 4 个 `PLAYING` 样本，跨度 0.076 s，低于 0.08 s 阈值而误报。改为等待至少 4 个样本且跨度 ≥0.12 s，稳定验证了共享时钟推进。
4. **Marker 误差列表真值判断错误**：OBB/航向检查把含近零浮点数的非空列表直接当作失败。改为分别检查类型错误为空且数值最大值 ≤1e-6。
5. **终端 Marker 检查重复/陈旧**：删除终端前的重复 `check_markers` 调用，避免同名 check 被覆盖、并把旧的历史错误带入终端结果。
6. **状态 Marker 缺约束指标**：优化器状态不含 `pipeline` 块，状态文本曾显示“no independent metrics reported”。回放节点在 `plan_id` 匹配时合并 latched quality report 的 `pipeline`，仅用于显示；smoke 新增 `rigidity=`/`obstacle=` 断言。
7. **车间连线检查的类型混用**：link 条件错误和数值误差混合后可能对字符串调用 `max()`。拆分 fatal 条件与数值误差列表。
8. **轨迹排序假设错误**：smoke 不再用 `trajectories[vehicle_id-1]`，改为按 `vehicle_id` 建字典。
9. **回放刚心检查初始化错误**：修正一个自抵消的表达式，改用第一辆车中心作为参考并遍历所有 offset。
10. **E12 覆盖不足**：原 E12 只是单车终点偏移，没有覆盖文档要求的“目标队形镜像/车辆 ID 错误”。改为两车非对称 offset 的 `goal_mirror` 变异，现命中 `vehicles[1].goal_center`；重复 ID 由既有模型测试覆盖。
11. **launch 末尾缺换行**：补齐末尾换行，`git diff --check` 保持无输出。

以上修正只影响 S10 回放、smoke、回归 harness 和 E12 用例本身，不改变 S5–S9 的安全阈值或既有单车行为。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 tools/run_formation_regression.py --cases-file tools/formation_eval_cases.yaml --output-dir evaluation/formation_s10/regression --repeats 3` | 20/20 通过；75/75 outcome；最大墙钟 11.488 s | 0 |
| 同上 `--output-dir evaluation/formation_s10/ablation_no_spin --ablation no_spin` | 20/20 通过；72/75 outcome；E03 `MODE_DISABLED` | 0 |
| 同上 `--output-dir evaluation/formation_s10/ablation_no_oblique --ablation no_oblique` | 20/20 通过；66/75 outcome；E04/E07/E08 `MODE_DISABLED` | 0 |
| `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest -v tests.test_formation_playback tests.test_formation_regression tests.test_formation_demo_assets` | 42/42 通过；3.606 s | 0 |
| `OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 358 项，1 个既有错误；15.103 s | 1 |
| `/usr/bin/python3 tests/ros_formation_smoke.py` | PASS；39/39 检查；10.56 s；端口 11399 释放 | 0 |
| `python3 -m py_compile <S10 changed/new Python files>` | `COMPILE_OK` | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归唯一错误仍与 S0 相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 358 tests in 15.103s
FAILED (errors=1)
```

358 = S9 的 312 项 + S10 新增 46 项（回放 21、回归 15、演示资产 6、运动学模式消融 4），没有新增跳过、弱化或删除任何既有测试；未修改 `tests/test_continuous_retiming.py` 或单车重定时模块。

日志：

- [logs/regression_full.txt](logs/regression_full.txt)
- [logs/regression_no_spin.txt](logs/regression_no_spin.txt)
- [logs/regression_no_oblique.txt](logs/regression_no_oblique.txt)
- [logs/s10_targeted_tests.txt](logs/s10_targeted_tests.txt)
- [logs/unittest_full.txt](logs/unittest_full.txt)
- [logs/ros_smoke.txt](logs/ros_smoke.txt)
- [logs/py_compile.txt](logs/py_compile.txt)
- [logs/git_diff_check.txt](logs/git_diff_check.txt)
- [logs/ros_smoke/](logs/ros_smoke/)（roscore/optimizer/playback 节点日志）

## 未执行项目与边界

1. **`catkin_make -C catkin_ws -j1` 未执行**：本任务的长期约束是不得修改 `catkin_ws/build/` 与 `catkin_ws/devel/`，因此没有重新生成 devel 空间。
2. **`roslaunch fwis_v3_demo formation_demo.launch` 未执行**：同样受 build/devel 约束。`catkin_install_python` 已包含 `scripts/formation_playback.py`，`package.xml`/`CMakeLists.txt` 已加 `tf2_ros`；但在执行 `catkin_make` 之前，`devel/lib/fwis_v3_playback/formation_playback.py` 不会出现，`roslaunch` 找不到该节点。**这是本阶段唯一未完成的集成步骤，原因是约束，不是测试通过后的遗漏。**
3. **没有启动 RViz GUI**：`formation_demo.rviz` 和 launch 仅做静态资产测试；Marker/TF 语义由 ROS smoke 在消息层验证，不等于屏幕像素检查。待重建后在 `roslaunch ... rviz:=true` 中做人工目视确认。
4. **smoke 不启动 C++ S4 前端**：它直接发布 `/map`、请求和 seed，隔离验证 S9/S10 的 Python 边界；C++ 前端另有 S4 的 13 项几何 gtest 与 ROS 探针覆盖。launch 资产测试确认 launch 中编队前端恰好一个。
5. **没有真实车辆/控制器/NMPC/执行器/闭环仿真**：S10 只显示轨迹，不闭环执行；这些接口按任务约束没有修改。
6. **没有动态障碍、通信延迟、执行器带宽或轮系响应**：所有地图是静态 `OccupancyGrid`，所有时间是理想共同时钟。
7. **没有 `rosbag` 回放或真实地图场景目视**：固定用例使用 320×240 空/门洞/弯道/通道的确定性栅格，不是实际地下车库地图。
8. **`expanded_nodes` 为 null**：直接 Python 后端没有图搜索，记录为 null，不以 0 或虚构数字冒充。
9. **前端耗时未单独测量**：回归直接驱动后端；`build_s` 包含 S6 几何与 S5 模式分配，未再拆 S5/前端/ROS 传输。
10. **性能数值不是实时性保证**：单线程、`OPENBLAS=1` 的桌面 CPU 结果；ROS 回调中同步规划，未做异步 worker/取消/超时压力测试。

## 安全性审计

- **共同时钟不可绕过**：回放从 `FormationTimedTrajectory.header.stamp` 计算 elapsed；`trajectory_from_fields` 拒绝零 stamp、不同车辆时间轴、非严格递增、非有限时间、非零 `nsecs>=1e9`。没有逐车收到消息时刻的入口。
- **刚性不被显示层改写**：回放不重新拟合队形，只插值消息中的中心/航向；smoke 在运动样本和终端样本重新计算 `p_i - p_F - R psi r_i` 与 pairwise 距离，误差为 0。
- **Marker 唯一性**：namespace 含 formation_id/vehicle_id，Marker ID 全数组唯一；TF frame 含 sanitize 后的 formation_id 和 vehicle_id，`base_link` 裸名不会冲突。
- **A/O/S 不混淆**：颜色严格按消息 mode 映射；smoke 检查完整路径每个颜色对应的 mode 都正确；模式/控制 TEXT 显示每车当前 mode 与 control。
- **失败不冒充成功**：状态标签只有在 `plan_id` 匹配且 `feasible/executable/trajectory_published` 全为 true 时显示 `FEASIBLE`；否则 `AUDIT_PENDING`/`GEOMETRY_ONLY`。smoke 用合成无状态轨迹验证。
- **空结果清理**：空 `vehicles`/`trajectories` 触发 `_clear`；发布 `DELETEALL` 与 `CLEARED`；smoke 检查旧 Marker 不残留。
- **终端保持**：`HOLD_END` 精确返回最后一个共享样本，不外推；终端中心/航向误差独立计算，smoke 为 0。
- **固定用例可复现**：地图尺寸/原点/障碍、body 尺寸、offsets、时间戳、seed、预算全部写在 YAML；每个 case 至少 3 次；失败行保留。
- **消融不偷偷放宽**：`allowed_modes` 只过滤模式；被禁模式的候选返回显式 `MODE_DISABLED`；不换模式、不降级成几何-only 成功。
- **统计诚实**：`all_pass` 是“声明期望是否满足”，不是成功率；成功率、分母和拒绝原因逐行记录；E19/E20 的统计组不要求全成功，但记录每个子运行。
- **不污染既有系统**：未修改单车 `playback.py`、`demo.rviz`、`demo.launch`、`run_demo.sh`、单车消息/话题、C++、控制器、NMPC、执行器、闭环仿真、`archive/`、`build/`、`devel/`。
- **进程与端口卫生**：smoke 使用私有 11399，拒绝受保护端口；结束后 SIGINT 子进程、确认退出码 0、端口释放；不批量 `pkill` 无关 ROS/Python。

## 存在风险

1. **回放插值假设**：回放用线性中心插值和最短角差航向插值；S9 的发布采样证书也按线性插值重建。若下游使用不同插值（多项式、不同角度展开），实际轨迹可能偏离显示/证书。
2. **显示层不是安全证明**：`FEASIBLE` 只是把 S9 的认证结果映射到屏幕；RViz 上的绿色/状态文本不能替代 S9 审核，也不证明实车可执行。
3. **几何-only 轨迹当前只由 smoke 合成**：S9 优化器不发布失败轨迹；若未来前端把 geometry-only 轨迹发布到 `/formation/timed_trajectory`，必须复用本回放的 `GEOMETRY_ONLY` 逻辑并保证不带 `trajectory_published`。
4. **Marker 性能未压测**：每车完整路径 + 历史 + 每帧 OBB/TF，在 N=3、296 样本下 25 Hz 正常；大队形/长轨迹/高频率可能造成 MarkerArray 过大，未测带宽/延迟。
5. **ROS 传输时序**：轨迹与状态是不同话题，回放可能先收到轨迹后收到状态，短时间内显示 `AUDIT_PENDING`；这是安全默认，但不是原子显示。若状态先到、轨迹后到，同样等待匹配 plan_id。
6. **`formation_id` sanitize 的碰撞概率**：SHA1 前 8 位用于区分被改写的 id，理论碰撞概率极低但非零；TF 名称安全不是安全认证。
7. **固定用例是确定性栅格近似**：E06/E08 的净空由栅格膨胀与 OBB 检查决定；真实地图的未知栅格、地图更新、定位误差不在内。
8. **统计样本量小**：每 case 3 次、E19/E20 子运行 9/12 次，给出的是可复现实验记录，不是统计显著的性能结论；没有置信区间。
9. **未执行真实 launch/RViz**：构建/devel 未重建前，`roslaunch` 会找不到回放脚本；这是明确的待办，不是已完成项。
10. **前向兼容**：`formation_playback/v1` 和 `formation_optimizer_status/v1` 是字符串 schema；后续新增字段必须保持向后兼容或升版本，回放对未知字段宽容但不校验其语义。
11. **单 pending seed 与同步回调**：S9 节点的单 pending seed/同步规划限制仍在；S10 没有引入异步或取消，长管线会阻塞 ROS 回调。

## 完成判据

| 文档要求 | 结果 |
|---|---|
| 新增回放/演示/ smoke/回归/YAML/RViz 文件 | **已完成**；`formation_demo.rviz` 为新建，未覆盖 `demo.rviz` |
| 一次一个编队 planner/optimizer，不复用三份单车 planner | **launch 静态验证唯一节点；smoke 只启动一个优化器和一个回放** |
| Marker namespace 含 formation/vehicle id，ID 唯一；TF 唯一 | **已完成并由 smoke 检查（20 Marker/20 唯一 ID；4 个唯一 TF frame）** |
| 显示参考点/OBB/航向/模式色带/历史/连线/指标/失败轨迹 | **已完成**；失败轨迹由合成无状态轨迹验证 `GEOMETRY_ONLY` |
| 共同 header.stamp + 相同 time_from_start | **已完成**；296 个共享样本，终端精确保持 |
| smoke 检查消息/回放采样/空结果清理/全车 Marker/终端误差/独立审核 | **39/39 检查通过** |
| 固定地图/种子/预算至少 3 次，记录 CPU/版本/分位数/成功率/拒绝原因，不删失败 | **E01–E20 3 次重复；summary 记录环境、CPU、RSS、分位数、outcome 定义和失败码；失败行保留** |
| E01–E20 每行至少一个固定 YAML case | **已覆盖**；E12 为两车目标镜像，E19/E20 为统计组 |
| 消融（禁用 Spin/O、N=1/2/3、队形/障碍密度） | **no_spin/no_oblique 已跑；N 与密度在 E19/E20 统计组** |
| 既有单车回归 | **358 项，仅既有 `dynamic_peaks` 错误，无新增失败** |
| 不修改控制器/NMPC/执行器/闭环仿真/C++/消息 | **已确认** |
| `catkin_make` / `roslaunch` 实跑 | **未执行**：受 build/devel 不可修改约束；待重建环境后补做 |
| 人工 RViz 目视 | **未执行**：同上 |

**结论：S10 的代码、固定用例、消融、smoke 和审查证据已完成；代码层与消息层验收通过。唯一遗留的集成动作是在允许重建 `catkin_ws/build`/`devel` 的环境执行 `catkin_make`，随后 `roslaunch fwis_v3_demo formation_demo.launch rviz:=true` 做一次真实 launch/RViz 目视确认。由于本任务明确禁止修改 build/devel，该动作不能在本阶段声称已通过。**

## 下阶段前置条件与项目收尾

S10 是实施文档的最后一个阶段，没有 S11。项目收尾需要：

1. 在允许写 `catkin_ws/build`/`devel` 的环境执行 `catkin_make -C catkin_ws -j1`，确认 `devel/lib/fwis_v3_playback/formation_playback.py` 生成。
2. 用固定地图执行 `roslaunch fwis_v3_demo formation_demo.launch rviz:=true`，确认 RViz 显示编队参考点、OBB、航向、模式色带、历史路径、车间连线与状态文本；失败/几何-only 轨迹必须显示 `GEOMETRY_ONLY`。
3. 保持本阶段的私有 master smoke 作为回归门：任何回放/状态/清理改动后重跑 `tests/ros_formation_smoke.py`。
4. 若未来接入实车/控制器，必须把 `FormationTimedTrajectory` 的共同时间轴和 S9 认证状态一并传入，不得用回放节点的显示状态替代控制器的安全门。
5. 保持 `tests/test_continuous_retiming.py:271` 的既有错误原样；修复需单独授权，不能把它混入 S10 提交。