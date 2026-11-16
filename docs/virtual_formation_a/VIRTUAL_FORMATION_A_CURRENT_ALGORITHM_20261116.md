# virtual_rigid_a 当前算法说明书

- 文档版本：2026-11-16（与仓库当前工作树一致）
- 适用代码：`feature/virtual-rigid-formation-a` 工作树
- 适用场景：三车货物中心楔形编队 `cargo_wedge`
  - car1 偏移 `(1.0, 0.0) m`
  - car2 偏移 `(0.0, 1.0) m`
  - car3 偏移 `(0.0, -1.0) m`
  - 单车车体 `0.738 m × 0.500 m`
- 本文目的：
  1. 说明当前 `virtual_rigid_a` 算法相对基线 `uniform_v02` 的全部功能性修改；
  2. 说明算法从 RViz 输入到轨迹发布的完整链路；
  3. 复现并解释 2026-11-16 在 map05（`--scale 2.0`）上观察到的两个问题：
     - 空旷区域出现不该有的转向、路径歪扭；
     - 空旷回头弯（U-turn）规划失败，报
       `NO_ACCEPTED_CANDIDATE / Maximum_Iterations_Exceeded / GEOMETRY`。
- 重要说明：
  - 本文记录的是**问题诊断**，不是“问题已修复”的验收报告。
  - 当前工作树的默认后端仍然是 `uniform_v02`，`virtual_rigid_a` 必须显式选择。
  - 两个问题都已在当前代码上复现；原因已用独立诊断实验定位到具体阈值和代码路径。

---

## 0. 结论摘要

### 0.1 回头弯规划失败：不是几何不可行，而是审计数值阈值把可行解全部拒绝

在 map05 `--scale 2.0` 上，用用户日志中的同一组起点/终点复现：

```text
start = (12.570, 10.320, -0.0297339)
goal  = ( 9.694,  2.854,  3.132)
policy = FREE
```

实际结果：

- 3 条 A* 引导路线全部生成；
- 6 个初值中，前 5 个 IPOPT **都求解成功**：
  - `Solve_Succeeded`
  - 20 / 22 / 20 / 25 / 23 次迭代
  - 目标函数约 `1.52e4 ~ 3.26e4`
- 但 5 个候选全部被独立几何审计拒绝，失败点完全相同：
  - 约束：`AUDIT_INTEGRATION_REMAINDER`
  - 实际值：`9.67e-8 ~ 1.01e-7 m`
  - 阈值：`5e-8 m`
  - 细化步数：`64`
  - 拒绝类别：`BOUND_UNCONFIRMED`
- 第 6 个初值 `length_multiplier=1.8; shape=left` 在 IPOPT 中达到
  `max_iter=120`，状态 `Maximum_Iterations_Exceeded`；
- 管线最终把**最后一个 attempt 的失败状态**当作整体 reason，因此日志显示
  `Maximum_Iterations_Exceeded`，掩盖了前 5 个“数值成功但审计拒绝”的真实原因。

独立诊断：只把 `GeometryAuditConfig.audit_position_budget_m` 从 `5e-8`
改成 `1e-7`（`1e-7` 本来就是生产路径的 `integration_budget_m` 合同值），
同一张图、同一请求、同一 `PlannerConfig(max_iterations=120)`：

```text
STATUS SUCCESS
attempt 0..5 全部 Solve_Succeeded（18/22/18/26/25/23 次）
geometry_audit passed=True
dynamic_audit  passed=True
collision_audit passed=True
wall ≈ 16.6 s
```

因此：

> 该回头弯在几何、动力学、碰撞三个真实审计层都是可行的；
> 失败发生在“独立积分数值证书”这一层。`5e-8 m` 的审计预算对这条
> 长约 15.56 m、曲率变化大的路径不可达；Simpson 余项 + 浮点 guard
> 在 64 次细化后停在约 `9.9e-8 m`。

这不是地图边界、车辆尺寸、碰撞、ROS 通信或“车转不过来”导致的失败。

### 0.2 路径歪扭/不必要转向：主路线候选被 IPOPT 迭代上限丢弃，只剩绕行替代路线可选

用一个完全空旷、本应近似直线的 3.93 m 东向移动复现：

```text
start = (12.570, 10.320, -0.0297339)
goal  = (16.500, 10.320, -0.0297339)
policy = FREE
```

默认 `solver="ipopt"`、`max_iterations=120` 时：

- 主路线 `route_guide=0; source=astar_primary`：
  - 目标函数 `2160.011`
  - IPOPT `Maximum_Iterations_Exceeded`，120 次
  - 残差 `6.66e-7`（未达到 1e-8）
  - 没有被审计，直接丢弃
- 替代路线 1：同样 `Maximum_Iterations_Exceeded`，目标 `11121.529`
- **替代路线 2**：`Solve_Succeeded`，34 次，目标 `10696.138`
  - 是唯一通过审计的候选
  - 被选中
- 最终路径：
  - 长度 `3.952 m`
  - 最大曲率 `|κ| = 0.5475 1/m`（半径 `1.83 m`）
  - 最大前轮转角约 `15.13°`
  - 曲率符号变化 5 次
  - 曲率总变差 `2.8983`
- 而替代路线 2 本身的 9 节点曲率是：
  - `[0.0, -0.702, 0.0, 0.0, 0.0, 0.0, 0.0, -0.7798, 0.0]`
  - 即路线在起点和终点附近各有一个“向外拐再回来”的 dogleg。

进一步实验：

- 只保留主路线，把 IPOPT `max_iterations` 提到 **2000**：
  - 仍然 `Maximum_Iterations_Exceeded`
  - 目标 `2142.308`
  - 残差 `5.67e-7`
  - 说明不是简单的“120 太小”，IPOPT 在这个强路线引导目标上本身收敛困难。
- 把 `solver` 改成 `"auto"`，让 SLSQP fallback 生效：
  - IPOPT 仍在 300 次后失败；
  - SLSQP 在 **55 次**迭代后成功；
  - 残差 `1.57e-10`，目标 `2079.771`；
  - 路径最大 `|κ| = 0.1902`（半径 `5.26 m`）
  - 最大前轮转角 `5.37°`
  - 曲率总变差 `0.5309`
  - 路径到主路线平均距离 `0.0033 m`。

因此：

> 空旷区域“本不该大转向”的根因不是几何约束本身，而是：
> 1. 生产配置 `solver="ipopt"` 使 `allow_scipy_fallback=True` 实际失效
>    （fallback 只在 `solver=="auto"` 时执行）；
> 2. IPOPT 在主路线候选上频繁 `Maximum_Iterations_Exceeded`；
> 3. 失败的主路线候选被丢弃，候选选择只能接受“唯一通过审计但目标更差”的
>    替代路线；
> 4. 替代路线由 A* 在惩罚带约束下生成，天然带起点/终点 dogleg；
> 5. 优化目标中路线位置权重 `5.0e5` 远大于曲率相关权重，路径被迫贴合
>    这种带 dogleg 的离散 A* 引导线；
> 6. 曲率只有硬上限 `kappa_limit = 1/(1+0.810) ≈ 0.5525 1/m`，
>    没有一个足够强的“尽量小曲率/小转角”软目标。

### 0.3 当前代码状态

- 两个问题均**未修改代码**。
- 本文给出建议修复方向，但任何阈值/消息/默认值改动都会影响已完成的 G5
  验收证据，因此没有在未获确认前直接改生产默认。
- 复现证据保存在：
  `docs/virtual_formation_a/diagnostics_20261116/`

---

## 1. 复现环境与操作方式

### 1.1 软件与 ROS

- Ubuntu + ROS Noetic
- Python 3
- CasADi 3.7.2 + IPOPT
- 私有 ROS master：`http://127.0.0.1:11587`
  - 仅用于隔离本次诊断，不与用户已有 ROS 节点串线；
  - 单次运行时 `roslaunch` 也会自动使用 11311，私有端口不是算法要求。

### 1.2 地图

- 地图：`map05-p12-parking-row-env2`
- 用户确认使用放大 2 倍的图：`--scale 2.0`
- 原始 map05：

```yaml
resolution: 0.024713
origin: [1.4578, 0.9719, 0.0]
image: map.pgm
```

- `--scale 2.0` 后运行时地图（`run_demo.sh` 生成的 `/tmp/expv3-map-scale.*` 副本）：

```text
width  = 489
height = 326
resolution = 0.049426 m/cell
origin = (-4.5845285, -3.056319)
extent ≈ x[-4.58, 19.58], y[-3.06, 13.06]
occupied_cells = 37838 / 159414
occupancy_sha256 = e5f91759835d6b0a2fbad3bc46da8fe21b4153c4680a90b1082c51529bcf0a7a
```

### 1.3 启动命令（诊断用）

```bash
export ROS_MASTER_URI=http://127.0.0.1:11587
roscore -p 11587

cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_virtual_a.sh --map map05 --scale 2.0 rviz:=false
```

`run_virtual_a.sh` 是新增的便捷包装脚本，只注入：

```text
backend:=virtual_rigid_a
auto_goal:=false
```

它**不改变** `run_demo.sh` 的既有默认后端 `uniform_v02`。

### 1.4 发布 reset 与 goal 的正确顺序

`virtual_rigid_a` 要求显式 reset：

1. 等待 `/initialpose` 与 `/move_base_simple/goal` 的订阅者连接建立；
2. 发布 `/initialpose`；
3. 等待 `/formation/job_state` 出现：
   - `reason == "RESET_CONFIRMED"`
   - `initial_reset_ready == true`
4. 再发布 `/move_base_simple/goal`。

本次诊断驱动脚本在发布前等待连接数：

```text
init=2 goal=1
```

否则第一条 `/initialpose` 会在 ROS 连接建立前丢失，表现为一直不确认 reset。

---

## 2. 基线 vs 当前实现

### 2.1 基线：`uniform_v02`（V0.2 统一编队规划器）

基线入口是 `formation_uniform_planner_node.py`，调用
`fwis_v3_optimizer.uniform_pipeline.plan_uniform`：

- 输入：`FormationPlanRequest`，三车规格、货物起点/终点、地图；
- 编队被当作一个统一的刚性编队整体规划；
- 轨迹由统一的编队运动 + FR02 审计产生；
- 发布原子 `FormationPlaybackPlan`；
- 车体航向与货物/编队航向绑定；
- 只有 `uniform_v02` 一个自主规划后端，另有旧 `legacy` C++/Python 对照链路。

基线没有：

- 三车独立车体航向；
- FREE / STRAIGHT 终点航向策略；
- 连续曲率 `κ(s), η(s), u(s), g(s)` 的 C2 Hermite 端点 NLP；
- 独立几何/动力学/碰撞三层 G4 审计与 `virtual_a_geometry_dynamics_collision/v1`
  证书；
- `virtual_rigid_a` 的 reset 上下文确认；
- 虚拟轨迹回放前的证书强校验。

### 2.2 当前：`virtual_rigid_a`

当前实现不是改坏 `uniform_v02`，而是在同一节点中增加一个**显式选择的**
后端钩子。总体链路：

```text
RViz 2D Pose Estimate
  -> /initialpose
  -> formation_goal_bridge（记录 manual start，不消费 busy 状态）
  -> formation_uniform_planner_node.on_initial_pose
       -> VirtualAInitialContext(source="rviz_reset")
       -> /formation/job_state: RESET_CONFIRMED
       -> /formation/optimizer_status: initial_reset_ready=true

RViz 2D Nav Goal
  -> /move_base_simple/goal
  -> formation_goal_bridge
       -> FormationPlanRequest
       -> /formation/plan_request

formation_uniform_planner_node（virtual_rigid_a hooks）
  -> request_from_snapshot(...)
       - 检查 reset 位姿与车体航向
       - 检查 FREE / STRAIGHT
       - 生成 VirtualAMapRequest
  -> plan_virtual_a / solve
       1. A* 生成最多 3 条引导路线（仅拓扑提示）
       2. 生成最多 6 个初值
       3. IPOPT（可选 SLSQP）求解 κ(s) 参数
       4. 独立几何审计
       5. 公共时间 quintic retiming + 独立动力学审计
       6. 三车 OBB 扫掠 + 地图碰撞审计
  -> 只有三层审计全部通过才输出 executable 结果
  -> FormationPlaybackPlan（含 certification）
  -> /formation/playback_plan

formation_playback
  -> verify_virtual_a_playback_plan
       - 重新验证 backend、certification、model_sha256、
         request_sha256、planning_context_sha256
       - 重新计算初始状态/轨迹
  -> TF / markers / 状态
```

关键原则：

- 路线只是 guide，不是可执行路径；
- 求解器的 `Solve_Succeeded` 不是证书；
- 只有三层独立审计全部通过，才允许 `executable=true`；
- 失败结果绝不发布可执行 envelope；
- `uniform_v02` 与 `legacy` 的代码路径保留。

### 2.3 所有功能性修改清单

#### 2.3.1 `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch`

新增/修改：

1. 后端说明新增 `backend:=virtual_rigid_a`；明确任一时刻只启动一个发布后端。
2. 新增参数：
   - `virtual_a_goal_policy`，默认 `free`；
   - `virtual_a_planning_budget_s`，默认 `60.0`；
   - `virtual_a_planning_timeout_s`，默认 `75.0`。
3. `formation_uniform_planner` 的启动条件从
   `backend == uniform_v02` 扩展为
   `backend == uniform_v02 or backend == virtual_rigid_a`。
4. 向 planner 传 `~backend`、`~max_publication_samples`；
   仅在 `virtual_rigid_a` 时传 `~virtual_a_goal_policy` 与预算参数。
5. 向 goal bridge 传 `~admission_backend` 与 `~virtual_a_goal_policy`。
6. `uniform_v02` 和 `legacy` 的行为不变。

为什么：

- 让同一个 ROS 节点根据显式 `backend` 加载不同纯逻辑 hooks；
- 避免复制节点；
- 避免新后端偷偷变成默认。

#### 2.3.2 `catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py`

修改：

1. 新增 `VIRTUAL_BACKEND = "virtual_rigid_a"`，
   `ADMISSION_BACKENDS = ("legacy", "uniform_v02", "virtual_rigid_a")`。
2. `~admission_backend` 校验加入 `virtual_rigid_a`。
3. 仅当后端是 `virtual_rigid_a` 时读取/校验 `~virtual_a_goal_policy`
   （`free` / `straight`）。
4. `_ensure_job_state()` 与 `_admission_busy()` 对 `virtual_rigid_a` 启用，
   与 `uniform_v02` 一样拒绝 `PLANNING/PLAYING` 期间的新输入。
5. `_build_and_publish_request()`：
   - `goal_heading_free = bool(self.goal_heading_free)`；
   - 若 `virtual_a_goal_policy` 已设置：
     `goal_heading_free = (policy == "free")`；
   - STRAIGHT 时仍把 cargo goal heading 写进每条 vehicle spec，
     由 adapter 再检查所有 heading 一致，不在这里偷改坐标系。
6. 增加 `virtual_a_goal_policy` 和 `goal_headings` 日志。

为什么：

- 保持消息类型 `FormationPlanRequest` 不变；
- FREE/STRAIGHT 是启动时确定的实验策略，不是每个 goal 的隐含行为；
- 让适配层可以独立复核策略和航向，而不是盲信 bridge。

#### 2.3.3 `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`

这是 G5 的主入口改动（新增约 638 行）。

修改：

1. 构造函数接受 `backend_hooks`。
2. 新增 `virtual_rigid_a` 专属状态与参数：
   - `virtual_a_planning_budget_s`
   - `virtual_a_planning_timeout_s`
   - reset context 计数
3. 新增 `/initialpose` 订阅与 `on_initial_pose()`：
   - 仅在 `virtual_rigid_a` 下启用；
   - 拒绝 busy（`PLANNING/PLAYING`）；
   - 验证 frame、平面位姿、有限值；
   - 创建 `VirtualAInitialContext(source="rviz_reset", context_id=...)`；
   - 发布 `RESET_CONFIRMED` / `initial_reset_ready=true`；
   - 不在这里生成 goal。
4. `on_request()` 在 virtual 后端下：
   - 拒绝 busy，返回 `BUSY_REJECTED`；
   - 调用 `request_from_snapshot`；
   - 失败时发布 `REQUEST_REJECTED`、状态 `REJECTED` 和原因。
5. 新增虚拟 planning worker：
   - `_admit_virtual()`；
   - `_virtual_plan_worker()`；
   - `_finish_virtual_failure()`；
   - `_finish_virtual_success()`。
6. 成功时构造 `FormationPlaybackPlan`：
   - `backend=virtual_rigid_a`
   - `certification=virtual_a_geometry_dynamics_collision/v1`
   - 包含 `model_json`、`trajectory`、请求/上下文指纹；
   - 发布前用 `verify_envelope_fields` 复核。
7. 失败时：
   - 状态 `FAILED`
   - `first_failure_layer`、`wall_time_s`、`virtual_result` 原样发布；
   - **不发布** playback plan。
8. job state 增加 `initial_context_*`、`virtual_a_goal_policy`、`request_sha256` 等。
9. `main()` 在 `backend == virtual_rigid_a` 时构造
   `VirtualABackendSettings` / `VirtualABackendHooks`；
   `uniform_v02` 仍走原 `plan_uniform`。

为什么：

- ROS 层负责准入、线程、发布；
- 纯算法留在 `formation_continuous_a`，可无 ROS 测试；
- reset 与 goal 分离，避免 RViz 0.5 s 竞态；
- 失败必须保留证据但绝不生成“看似可执行”的轨迹。

#### 2.3.4 `catkin_ws/src/fwis_v3_optimizer/setup.py`

把新包加入 catkin Python 安装列表：

```python
packages=[
    "fwis_v3_optimizer",
    "fwis_v3_optimizer.formation_continuous_a",
]
```

为什么：否则 `rosrun`/devel 空间无法导入新模块。

#### 2.3.5 `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py`

新增约 800 行，核心是**回放侧的独立验证**：

1. `virtual_a_request_from_raw_request(...)`：
   - 从 playback envelope 中的原始请求 JSON 重建 `VirtualARequest`；
   - 不信任 planner 的摘要字段。
2. `verify_playback_plan(*, backend, **fields)`：
   - 按 backend 分派；
   - `uniform_v02` 走原 `verify_uniform_playback_plan`；
   - `virtual_rigid_a` 走 `verify_virtual_a_playback_plan`。
3. `verify_virtual_a_playback_plan(...)`：
   - 校验 `backend`、`certification`、`semantic_version`；
   - 校验 `model_sha256`、`request_sha256`、`planning_context_sha256`；
   - 重建并验证初始状态与轨迹；
   - 校验每车共享时间、样本数、时间网格、终点；
   - 校验 `goal policy` 与 termination。
4. `_virtual_frame_at(...)`：从权威 timed model 解析回放帧；
   采样轨迹只是投影，不是权威。
5. `build_markers` / `status_document` 增加 virtual 分支。

为什么：

- planner 发布的 envelope 仍然可能在传输/序列化中被篡改或损坏；
- 回放端必须能独立重算，而不是“信 JSON”；
- 这也解释了用户日志中认证为空时的拒绝行为。

#### 2.3.6 `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`

修改：

1. 引入 `verify_playback_plan`、virtual request 重建、virtual frame 评估。
2. 记录原始 `FormationPlanRequest` 与 `/formation/job_state`，用于在回放时
   复核虚拟 envelope。
3. `_verify_virtual_envelope()`：
   - 要求 `certification == virtual_a_geometry_dynamics_collision/v1`；
   - 若为空或不匹配，拒绝并清理：
     `unsupported certification ''; expected ...`。
4. `_check_virtual_initial()`：核对 reset context 与请求起点/航向。
5. `_virtual_frame_at`：用权威 timed model 生成显示帧。
6. `_publish_hold(clear=...)`：失败/清理时显式发布不可执行状态。
7. busy 状态与 HOLD_END 反馈逻辑保持原语义。

为什么：

- 失败的 planner 不会发布 envelope，但回放节点可能仍持有旧状态；
- 必须拒绝无证书/错证书的 plan，并清除，避免“旧轨迹冒充新结果”。

#### 2.3.7 `run_demo.sh`

修改：

1. 帮助文本和 `--backend` 校验加入 `virtual_rigid_a`。
2. 仍保留默认 `uniform_v02`。
3. 新增 `run_virtual_a.sh` 作为便捷入口（不在 `run_demo.sh` 内改默认）。

为什么：

- G5 任务明确要求不改变旧默认；
- 多车实验需要简单启动方式，因此用外层 wrapper 注入两个参数；
- `run_demo.sh` 仍是原验收接口。

#### 2.3.8 新包 `formation_continuous_a/`（约 15,300 行）

| 文件 | 作用 |
|---|---|
| `contracts.py` | 不可变数据合同、状态码、配置校验、JSON/指纹/上下文、`VirtualARequest` / `VirtualAMapRequest` / `VirtualAResult` |
| `geometry.py` | C2 Hermite 曲率剖面、`VirtualAPath`、位置积分与积分预算、车辆空间状态 |
| `optimizer.py` | 端点 NLP、A* 引导采样、初值、IPOPT/SLSQP、目标函数、候选记录 |
| `audit.py` | 独立几何审计、独立动力学审计、证书生成 |
| `bounds.py` / `integration_bounds.py` | 区间算术、Simpson 余项、M4 包络、浮点/角度 guard |
| `collision.py` | 地图加载/哈希、解析扫掠段、旧 OBB/SAT 碰撞引擎适配、误差包络 |
| `timing.py` | 全路径公共时间 quintic retiming |
| `trajectory.py` | 权威 timed model、采样投影、反序列化 |
| `pipeline.py` | A* 路线、初值、求解、三层审计、候选选择、结果组装 |
| `ros_adapter.py` | 无 ROS 的 G5 hooks：snapshot→map request、solve、发布采样、envelope 验证 |

#### 2.3.9 测试、工具、文档

新增：

- `tests/test_virtual_a_{audit,collision,geometry,integration_bounds,optimizer,pipeline,playback,profile,receiver,ros_adapter,timing,trajectory}.py`
- `tests/virtual_a_g4_support.py`、`tests/virtual_a_g5_support.py`
- `tests/ros_virtual_a_smoke.py`
- `tools/virtual_a_{baseline,benchmark,scenes}.py`
- `docs/virtual_formation_a/` 全部任务书、返工单、决策记录
- `evaluation/virtual_formation_a/` G0-G5 证据

这些不是算法运行时依赖，但支撑了 G0-G5 的独立审计与回归。

---

## 3. 当前算法详细说明

### 3.1 输入合同

`VirtualARequest` 的核心字段：

- `plan_id`, `formation_id`
- `model`：三车偏移与车辆 ID
- `vehicle_shapes`：`0.738 × 0.500`
- `start_qf = (x_F, y_F, ψ_F)`：货物中心起点与编队航向
- `start_headings = {1: θ1, 2: θ2, 3: θ3}`：每车真实车体航向
- `goal_qf = (x_F, y_F, ψ_F)`
- `initial = InitialAssumption(stationary=True, mode="A", wheels_straight=True, source=...)`
- `goal_policy ∈ {FREE, STRAIGHT}`
- `limits = FormationTimingLimits()`

`VirtualAMapRequest` 额外绑定：

- 权威 `Grid`
- `PlannerConfig`
- `TimingConfig`（内含 `GeometryAuditConfig`）
- `CollisionConfig`

所有指纹：

- `request_sha256`
- `planning_context_sha256`（请求 + 栅格 + 三配置）
- `grid_sha256`
- `model_sha256`

任何一层不一致，回放端拒绝。

### 3.2 reset 合同

`rviz_reset` 要求：

- `/initialpose` 的 F 位姿与请求 `start_qf` 的位置误差
  `<= 1e-6 m`；
- 航向误差 `<= 1e-6 rad`；
- 三车真实车体航向与 reset F 航向平行，误差
  `<= 1e-7 rad`；
- 不是 `BUSY` 状态；
- `initial_context_id` 非空，且被本请求消费一次。

不满足则：

- `INITIAL_RESET_NOT_CONFIRMED`
- `NONPARALLEL_INITIAL_HEADINGS`
- `UNSUPPORTED_INITIAL_STATE`

### 3.3 FREE / STRAIGHT 终点策略

- `FREE`：
  - 每条 vehicle spec 的 `goal_heading_free=true`；
  - 优化器允许各车终点航向自由（但仍受车体/编队几何与审计约束）。
- `STRAIGHT`：
  - bridge 把所有 `goal_heading_free=false`；
  - 所有车终点航向 = cargo goal `ψ`；
  - adapter 要求所有 fixed heading 与 cargo `ψ` 一致（`1e-9 rad`）；
  - NLP 增加终端等式 `κ_F = 0`、`η_F = 0`。

用户本次测试全部使用：

```text
virtual_a_goal_policy=free
```

### 3.4 地图与坐标

- 地图来自 `/map`（`nav_msgs/OccupancyGrid`）；
- planner 使用栅格 `resolution`、`origin[0:2]`；
- **忽略 `origin` 的 yaw**；当前设计不接受旋转地图；
- 灰度/未知像素被 map server 发布为 `-1`，planner 把任何非零值视为占用；
- 地图边界视为硬墙；
- `--scale 2.0` 时，`run_demo.sh` 以地图中心为固定点放大分辨率/原点；
  栅格宽高不变，但世界尺寸翻倍；
- 规划器看到的 map05 2× 栅格摘要见第 1.2 节。

### 3.5 A* 引导路线

`pipeline.make_route_guides()`：

1. 用欧氏距离变换（EDT）生成 clearance field；
2. `blocked_radius_m=0.15`；
3. A* 8 邻域，禁止斜穿角落；
4. 步长代价：

   ```text
   step_cost = step * (1 + clearance_weight *
       max(0, clearance_cap_m - clearance) / clearance_cap_m)
   ```

   当前 `clearance_weight=2.0`, `clearance_cap_m=2.0`；
5. 最多 `max_routes=3` 条；
6. 生成替代路线时，在已选路线周围 ±2 cell 加惩罚带
   `alternative_penalty=2.0`；
7. 返回的每条路线只是 cell-center 折线：
   - 8 邻域离散；
   - 不做曲率平滑；
   - 不可执行；
   - 附带 `grid_sha256`。

### 3.6 初值生成

`make_initial_guesses()` 最多 6 个：

1. 每条 A* 路线一个 `map_route_guide` 初值；
2. 剩余名额用自动端点形状：
   - `DEFAULT_INITIAL_LENGTH_MULTIPLIERS`（1.2、1.8 ...）
   - `shape_sign = +1 / -1`
   - 标签如 `length_multiplier=1.2;shape=left`

路线初值：

- 把路线按弧长等分为 `intervals=8` 段 → 9 个节点；
- 节点位置映射到起点局部坐标；
- `_node_route_curvature()` 用三点外接圆估计节点曲率；
- 曲率按 `route_length / S_guess` 缩放；
- 裁剪到 `kappa_limit`；
- `_project_etas()` 做 clamped cubic spline C2 投影，保证
  `η_0 = η_N = 0` 与 C2 连续性。

注意：9 个节点、A* 折线、三点曲率，这三个因素共同决定了初值的
“路线形状”。A* 的起点/终点 dogleg 会被放大成节点曲率。

### 3.7 NLP 变量与约束

设计变量：

```text
z = [S, κ_0 ... κ_8, η_0 ... η_8]
```

- `S`：编队弧长；
- `κ_j`：节点曲率；
- `η_j = dκ/ds` 节点曲率变化率；
- 每段用三次 Hermite 插值 `κ(s)`：

  ```text
  c0 = κ_j
  c1 = h η_j
  c2 = 3(κ_{j+1}-κ_j) - h(2η_j+η_{j+1})
  c3 = 2(κ_j-κ_{j+1}) + h(η_j+η_{j+1})
  ```

- 通过 `ψ(s) = ψ_0 + ∫ κ ds` 与 `∫ exp(iψ) ds` 重建 F 点位置；
- 三车中心：

  ```text
  p_i(s) = p_F(s) + R(ψ(s)) r_i
  ```

约束：

1. 每段 C2 连续（`u` 连续）；
2. 终点 `x,y,ψ` 等于 `goal_qf`；
3. STRAIGHT 时 `κ_N=0, η_N=0`；
4. `|κ| <= kappa_limit = 1/(1+0.810) ≈ 0.5525`；
5. `|η| <= eta_limit = 2.0`；
6. 三车车体曲率约束：
   - 对每个偏移 `(a,b)`：
   - `A = 1 - bκ`, `B = aκ`, `D = A²+B²`
   - `A>0`, `D>0`
   - `(κD + aη)/D^{3/2}` 在采样点满足界；
7. 车体刚性与航向由 `map_spatial_state` 从 F 点恢复，不独立优化。

端点位置/航向的数值残差目标为 `1e-8` 级别。

### 3.8 目标函数

IPOPT/SLSQP 使用同一个目标（数值路径略去等价项）：

```text
J = S/l0
  + 0.02 * l0^3 * ∫ η² ds
  + ROUTE_GUIDANCE_WEIGHT * mean(||p_node - p_route_node||²)
  + ROUTE_GUIDANCE_WEIGHT * mean(||p_collocation - p_route_collocation||²)
  + ROUTE_CURVATURE_WEIGHT * mean((κ_node - κ_route_node)²)
  + CLEARANCE_PROXY_WEIGHT * mean(hinge(clearance_target - clearance)²)
```

当前权重：

```text
ROUTE_GUIDANCE_WEIGHT   = 5.0e5
ROUTE_CURVATURE_WEIGHT  = 100.0
CLEARANCE_PROXY_WEIGHT  = 5.0e4
CLEARANCE_PROXY_TARGET_M = 0.05
```

解释：

- `S/l0`：偏好短路径；
- `0.02 l0³ ∫η²`：偏好曲率变化小（平顺转向）；
- 路线位置项：5e5，**远大于**基础项；
- 路线曲率项：仅 100，**远小于**位置项；
- clearance proxy：5e4，软避障，不是碰撞证书。

因此当 A* 引导线本身有 dogleg 时：

- 路径位置被 5e5 强拉向 dogleg；
- 曲率项 100 几乎没有力量去“拉直”；
- 基础 `∫η²` 项相对可忽略；
- 最终路径必然复制引导线的拐弯形状。

用户感觉“曲率限制不够”，准确说是：

> 只有硬上限，没有足够强的软曲率/转角偏好；
> 路径曲率由引导线决定，而不是由“最小转向”决定。

### 3.9 求解器配置

`PlannerConfig` 默认：

```text
intervals = 8
max_initials = 6
max_seconds = 30.0
max_iterations = 120
solver = "ipopt"
allow_scipy_fallback = True
scipy_maxiter = 120
ipopt_tolerance = 1e-8
scipy_tolerance = 1e-8
curvature_margin = 0.02
eta_limit = 2.0
kappa_limit = 1/(1+0.810) = 0.552486...
```

IPOPT 选项：

```text
max_iter = 120
tol = 1e-8
max_wall_time = remaining budget
```

成功判定：

```text
Solve_Succeeded
Solved_To_Acceptable_Level
```

SLSQP fallback 的现实问题：

```python
if config.allow_scipy_fallback and solver_choice == "auto":
```

生产配置是 `solver="ipopt"`，不是 `"auto"`，所以：

> `allow_scipy_fallback=True` 实际不会被触发。
> IPOPT 一旦 `Maximum_Iterations_Exceeded`，这个候选就结束。

### 3.10 三层独立审计

#### 几何审计 `audit_geometry`

- 不调用优化器的残差函数；
- 从原始 knots 独立重建 Hermite 系数；
- 检查：
  - C0/C1/C2 连续性；
  - `|g|` 界；
  - 曲率/η 界；
  - 车体曲率；
  - 独立 Simpson 积分端点；
  - 区间算术；
  - Simpson 余项 + 浮点/角度 guard；
  - 5e-8 m 的 `audit_position_budget_m`。
- 关键失败点：

  ```text
  AUDIT_INTEGRATION_REMAINDER
  total_budget = remainder_euclidean + floating_guard
  阈值 = audit_position_budget_m = 5e-8
  细化上限 = 64 步
  ```

- 对长路径/高曲率路径，余项下降但 guard 增长；64 步后可能停在
  `9e-8 ~ 1e-7`，永远达不到 `5e-8`。
- 这是本次 U-turn 失败的直接原因。

#### 动力学审计 `audit_dynamics`

- 独立重建 timed model；
- 检查 v/a/jerk/横向加速度/转向角/转向速率/横摆率/横摆加速度；
- 使用区间包络，不信任采样最大值；
- 失败会给出具体层和约束。

本次 U-turn 在 audit budget 放宽后通过：
`dynamic_audit passed=True`。

#### 碰撞审计 `audit_collision`

- `collision.py` 把 C2 路径转成 `AnalyticSweepSegment`；
- 每段用区间算术给出 per-lambda 界；
- 独立验证界后调用旧 OBB/SAT 引擎；
- 起点/终点静态检查 + 全路径扫掠；
- 误差包络增加边距；
- 失败会给出车辆、站点、原因。

本次 U-turn 在 audit budget 放宽后通过：
`collision_audit passed=True`。

#### 证书

只有三层全部通过，结果才是：

```text
certification = virtual_a_geometry_dynamics_collision/v1
```

### 3.11 候选选择

`optimizer.solve_open_path_guided()`：

1. 遍历最多 6 个初值；
2. 每个初值：
   - IPOPT（或 SLSQP）；
   - 若失败且 `solver=="auto"`，尝试 SLSQP；
   - 成功则构造候选路径；
   - 几何审计；
   - 审计通过则与当前 best 比较目标函数；
3. 循环结束：
   - 有 best → 返回可执行结果；
   - 无 best → `NO_ACCEPTED_CANDIDATE`；
   - `reason = attempts[-1].rejection_reason or "NO_AUDITED_CANDIDATE"`。

这就是日志里 `Maximum_Iterations_Exceeded` 的来源：

> 它不是“所有候选都因迭代上限失败”，
> 而是“最后一个 attempt 恰好是迭代上限失败”；
> 前 5 个“求解成功但审计拒绝”的记录保存在 `attempts[]` 中，
> 但没有进入顶层 `reason`。

### 3.12 retiming 与回放

- 路径是几何 `κ(s)`；
- 公共时间进度是单段 rest-to-rest quintic；
- 无逐节点停顿、无隐藏 setup；
- 动力学限制来自 `FormationTimingLimits`；
- 总时长由全区间包络 + 安全系数选择；
- 采样投影按整数纳秒时间网格；
- 权威模型是 timed model JSON，不是采样点；
- 回放端重算每个显示帧。

---

## 4. 问题复现

### 4.1 问题二：空旷回头弯规划失败

#### 4.1.1 用户原始日志

截图见：

- `diagnostics_20261116/user_map05_plan600006_failure.png`

关键行：

```text
[INFO] ... explicit virtual experiment reset recorded at F=(12.5697, 10.3204, -0.0297339);
       this is a declared A-straight stationary initial state, not a physical wheel measurement

[INFO] ... plan 600006: start=(12.570, 10.320, -0.030) goal=(9.694, 2.854, 3.132) vehicles=3

[INFO] ... plan 600006 virtual_a_goal_policy=free
       goal_headings={1: 3.131593, 2: 3.131593, 3: 3.131593}

[INFO] ... plan 600006 admitted for virtual_rigid_a:
       start=(12.570, 10.320, -0.030) goal=(9.694, 2.854, 3.132)
       initial=rviz_reset/virtual-a-reset-600005-10 policy=FREE

[ERROR] ... virtual plan 600006 not executable:
        status=NO_ACCEPTED_CANDIDATE reason=Maximum_Iterations_Exceeded
        layer=GEOMETRY wall=19.082s

[WARN] ... plan 600006 has no executable envelope (Maximum_Iterations_Exceeded)
[WARN] ... plan 600006 status=REJECTED reason=Maximum_Iterations_Exceeded

[ERROR] ... playback rejecting playback plan 600006:
        unsupported certification ''; expected 'virtual_a_geometry_dynamics_collision/v1'

[INFO] ... playback cleared: invalid playback plan:
       unsupported certification ''; expected 'virtual_a_geometry_dynamics_collision/v1'
```

#### 4.1.2 本机复现

命令：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11587
roscore -p 11587

cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_virtual_a.sh --map map05 --scale 2.0 rviz:=false

# 另一个终端
python3 /tmp/repro_uturn.py /tmp/repro_uturn_out
```

驱动等待 `RESET_CONFIRMED` 后发布：

```text
/initialpose = (12.570, 10.320, -0.0297339)
/move_base_simple/goal = (9.694, 2.854, 3.132)
```

驱动日志（`repro_uturn_driver.log`）：

```text
[job] t=0.02 state=IDLE reason= ready=False
connections init=2 goal=1
publish initialpose (12.57, 10.32, -0.0297339)
[job] t=0.13 state=IDLE reason=RESET_CONFIRMED ready=True
...
reset confirmed= True
publish goal (9.694, 2.854, 3.132)
[job] t=2.36 state=PLANNING reason=planning ready=True
...
[status] t=19.73 state=REJECTED reason=Maximum_Iterations_Exceeded
         detail=solver attempts produced no independently audited candidate
[job] t=19.73 state=FAILED reason=Maximum_Iterations_Exceeded ready=True
```

planner ROS log（`repro_ros_planner.log`）：

```text
[rosout][INFO] ... [uniform planner] explicit virtual experiment reset recorded at
F=(12.57, 10.32, -0.0297339); ...
[rosout][INFO] ... [uniform planner] plan 600000 admitted for virtual_rigid_a:
start=(12.570, 10.320, -0.030) goal=(9.694, 2.854, 3.132)
initial=rviz_reset/virtual-a-reset-None-3 policy=FREE
[rosout][ERROR] ... [uniform planner] virtual plan 600000 not executable:
status=NO_ACCEPTED_CANDIDATE reason=Maximum_Iterations_Exceeded
layer=GEOMETRY wall=17.367s
```

结论一致：用户 19.082 s，本机 17.367 s，差异只是机器负载。

#### 4.1.3 6 个 attempt 的真实记录

来自 `repro_uturn_summary.json`：

| attempt | 初值 | solver status | 迭代 | 目标 | 审计 | 失败值 |
|---:|---|---|---:|---:|---|---|
| 0 | `route_guide=0; astar_primary` | `Solve_Succeeded` | 20 | 15605.46 | 拒绝 | `AUDIT_INTEGRATION_REMAINDER = 9.923477e-8 > 5e-8` |
| 1 | `route_guide=1; astar_alternative_1` | `Solve_Succeeded` | 22 | 32584.62 | 拒绝 | `1.013824e-7 > 5e-8` |
| 2 | `route_guide=2; astar_alternative_2` | `Solve_Succeeded` | 20 | 17543.93 | 拒绝 | `9.665780e-8 > 5e-8` |
| 3 | `length_multiplier=1.2; shape=left` | `Solve_Succeeded` | 25 | 15218.31 | 拒绝 | `9.923364e-8 > 5e-8` |
| 4 | `length_multiplier=1.2; shape=right` | `Solve_Succeeded` | 23 | 15759.10 | 拒绝 | `9.922589e-8 > 5e-8` |
| 5 | `length_multiplier=1.8; shape=left` | `Maximum_Iterations_Exceeded` | 120 | 15251.46 | 未审计 | solver 迭代上限 |

共同审计信息：

```text
constraint = AUDIT_INTEGRATION_REMAINDER
limit = 5e-08
refinement_steps = 64
value ∈ [9.6658e-8, 1.0138e-7]
detail = "audit Simpson refinement did not reach 5e-08"
```

说明：

- 前 5 个解不是几何不可行；
- 它们的端点误差、残差都在 1e-8 量级；
- 统一失败在独立积分的**保守余项预算**；
- 第 6 个的 `Maximum_Iterations_Exceeded` 只是最后一个 attempt 的状态，
  被顶层 reason 选中，造成误导。

#### 4.1.4 只改审计预算的诊断实验

脚本：`/tmp/repro_audit_budget.py`
唯一改动：

```python
GeometryAuditConfig(audit_position_budget_m=1e-7)
```

其他保持不变：

```text
map05 scale 2.0
start=(12.570,10.320,-0.0297339)
goal=(9.694,2.854,3.132)
FREE
PlannerConfig(max_iterations=120)
```

输出（`repro_uturn_budget_1e7.log`）：

```text
STATUS SUCCESS   wall 16.6266 s

attempt 0 map_route_guide ipopt Solve_Succeeded iters 18 obj 15708.3304 status SOLVER_SUCCESS_AUDITED audit True
attempt 1 map_route_guide ipopt Solve_Succeeded iters 22 obj 32319.0002 status SOLVER_SUCCESS_AUDITED audit True
attempt 2 map_route_guide ipopt Solve_Succeeded iters 18 obj 21894.2703 status SOLVER_SUCCESS_AUDITED audit True
attempt 3 automatic_endpoint_shape ipopt Solve_Succeeded iters 26 obj 15303.1672 status SOLVER_SUCCESS_AUDITED audit True
attempt 4 automatic_endpoint_shape ipopt Solve_Succeeded iters 25 obj 15897.3907 status SOLVER_SUCCESS_AUDITED audit True
attempt 5 automatic_endpoint_shape ipopt Solve_Succeeded iters 23 obj 15373.6219 status SOLVER_SUCCESS_AUDITED audit True

geometry_audit passed True
dynamic_audit passed True
collision_audit passed True
```

这是排他性证据：

> 同一路径、同一地图、同一车辆、同一求解器配置；
> 只改变独立几何审计的数值预算；
> 结果从“全部拒绝”变成“全部通过三层审计”。

因此 U-turn 的几何/动力学/碰撞本身没有问题，失败是审计数值证书的
**假阴性**。

#### 4.1.5 为什么 64 步细化仍达不到 5e-8

几何审计预算：

```text
total_budget = Simpson_remainder_euclidean + floating_guard
guard = POSITION_ROUNDOFF_GUARD_M * (simpson_panels + 8)
      + length * angle_bound
```

当前：

```text
POSITION_ROUNDOFF_GUARD_M = 1e-10
AUDIT_REFINEMENT_LIMIT = 64
audit_position_budget_m = 5e-8
```

- 对 15.56 m、曲率起伏大的 U-turn：
  - Simpson 余项随面板数下降；
  - 但 guard 随 `simpson_panels` 线性增长；
  - 两者相加在 64 步后约停在 `9.9e-8`；
- 提高面板数会继续增加 guard，不一定能到 5e-8；
- 这就是“细化 64 步仍未达到”的数值原因。

生产合同 `integration_budget_m=1e-7` 本身可以容纳该路径；
`audit_position_budget_m=5e-8` 是更严格的审计门限，当前对长路径过紧。

### 4.2 问题一：空旷区域不必要转向/路径歪扭

#### 4.2.1 用户的直观现象

截图：

- `diagnostics_20261116/user_map05_plan600002.png`
- `diagnostics_20261116/user_map05_plan600004.png`
- `diagnostics_20261116/user_map05_plan600005.png`

现象：

- 蓝线在开阔区域出现 S 形波动；
- 有些只需轻微转向的地方出现明显大转角；
- 曲率沿路径反复变化，看起来“歪歪扭扭”。

#### 4.2.2 最小复现：4 m 直线

选择完全空旷的东向移动：

```text
start = (12.570, 10.320, -0.0297339)
goal  = (16.500, 10.320, -0.0297339)
```

几何上这是：

- 位移 `3.93 m`；
- 航向变化 `0`；
- 无障碍；
- 期望路径：近似直线，曲率接近 0。

实际默认配置：

```text
solver = ipopt
max_iterations = 120
max_initials = 6
policy = FREE
audit_position_budget_m = 1e-7（诊断时放宽，以免审计假阴性干扰）
```

结果（`repro_straight_ipopt120.log`）：

```text
STATUS SUCCESS
attempt 0 route_guide=0;astar_primary
  ipopt Maximum_Iterations_Exceeded iters=120 obj=2160.011 residual=6.6568e-7
attempt 1 route_guide=1;astar_alternative_1
  ipopt Maximum_Iterations_Exceeded iters=120 obj=11121.529 residual=1.8125e-11
attempt 2 route_guide=2;astar_alternative_2
  ipopt Solve_Succeeded iters=34 obj=10696.138 residual=9.9942e-9 audit=True
attempt 3 length_multiplier=1.2;shape=left
  ipopt Maximum_Iterations_Exceeded iters=120 obj=2191.902 residual=6.0780e-6
attempt 4 length_multiplier=1.2;shape=right
  ipopt Maximum_Iterations_Exceeded iters=120 obj=2227.635 residual=1.1603e-4
attempt 5 length_multiplier=1.8;shape=left
  ipopt Maximum_Iterations_Exceeded iters=120 obj=2250.780 residual=3.4961e-4
```

只有 attempt 2 被审计通过，因此被选中。

#### 4.2.3 选中路径的曲率

```text
PATH length = 3.952016 m

knots s      = [0.000, 0.494, 0.988, 1.482, 1.976,
                2.470, 2.964, 3.458, 3.952]
knots kappa  = [0.0000, 0.3835, 0.0060, -0.3362, 0.0131,
                -0.0034, -0.3567, 0.0560, 0.5475]
knots eta    = [0.0000, 0.3112, -1.2081, 0.1511, 0.6469,
                -0.7181, -0.0200, 1.1588, 0.8754]

max |kappa| = 0.5475 1/m  -> 转弯半径 1.83 m
max steer   = 15.13 deg
curvature sign changes = 5
total variation of kappa = 2.8983
```

一段本应近似直线的 4 m 路径，出现了：

- 半径 1.83 m 的弯；
- 15.1° 前轮转角；
- 5 次曲率换向。

这与用户“不用那么大转向却发生大转向、路径歪歪扭扭”的描述完全一致。

#### 4.2.4 三条 A* 路线的形状

```text
ROUTE 0 astar_primary:
  points=82, length=3.935 m
  9-node kappa = [0.0, 0.0273, 0.0, -0.0, 0.0, -0.0, 0.0, 0.0275, 0.0]
  max|k|=0.0275, sign changes=6, TV=0.1097

ROUTE 1 astar_alternative_1:
  points=87, length=4.202 m
  9-node kappa = [0.0, 0.7576, 0.0, -0.0, 0.0, -0.0, 0.0, 0.8401, 0.0]
  max|k|=0.8401, sign changes=6, TV=3.1953

ROUTE 2 astar_alternative_2:
  points=87, length=4.202 m
  9-node kappa = [0.0, -0.702, -0.0, 0.0, 0.0, -0.0, 0.0, -0.7798, 0.0]
  max|k|=0.7798, sign changes=6, TV=2.9637
```

关键观察：

- 主路线 0 基本直，节点曲率只有 `0.0275`；
- 替代路线 1/2 在起点附近和终点附近各有约 `0.7~0.84` 的节点曲率；
- 这不是真实障碍绕行，而是 A* 替代路线为了“离开主路线惩罚带、
  再从精确端点重新汇入”形成的 dogleg；
- 选中路径到 route 2 平均距离 `0.0515 m`，说明它确实贴着这条 dogleg；
- 选中路径到主路线 0 平均距离 `0.0928 m`，最大值 `0.1562 m`，
  正是这段不该有的横向摆动。

#### 4.2.5 IPOPT 真的不能收敛主路线吗？

只保留主路线（`max_initials=1`），把 `max_iterations` 提到 2000：

```text
STATUS NO_ACCEPTED_CANDIDATE
reason Maximum_Iterations_Exceeded
attempt 0 route_guide=0;astar_primary
  ipopt Maximum_Iterations_Exceeded iters=2000
  obj=2142.308
  residual=5.6726e-7
wall=57.338 s
```

2000 次仍不收敛，残差停在 `5.67e-7`：

- 不是“120 稍微不够”；
- 是这个强路线引导目标让 IPOPT 在该实例上收敛困难；
- 残差距离 1e-8 还差近两个数量级。

#### 4.2.6 SLSQP fallback 能救主路线

设置：

```text
solver = "auto"
max_iterations = 300
max_initials = 1
policy = FREE
audit_position_budget_m = 1e-7
```

结果（`repro_straight_slsqp_auto.log`）：

```text
STATUS SUCCESS wall=13.298 s

attempt 0 route_guide=0;astar_primary
  ipopt Maximum_Iterations_Exceeded iters=300 obj=2116.592 residual=5.0630e-6
  scipy_slsqp Optimization terminated successfully iters=55 obj=2079.771
    residual=1.5743e-10 audit=True

PATH length = 3.930330 m
max |kappa| = 0.1902 1/m  -> 半径 5.26 m
max steer   = 5.37 deg
sign changes = 4
TV kappa    = 0.5309
path-to-route mean = 0.0033 m
```

对比：

| 指标 | 默认 ipopt（选中替代路线） | solver=auto（主路线 + SLSQP） |
|---|---:|---:|
| 目标函数 | 10696.14 | 2079.77 |
| max κ | 0.5475 | 0.1902 |
| 最小半径 | 1.83 m | 5.26 m |
| max 前轮转角 | 15.13° | 5.37° |
| κ 总变差 | 2.8983 | 0.5309 |
| 路径到主路线平均距离 | 0.0928 m | 0.0033 m |

结论：

> 不是没有更平滑的解；
> 是默认 solver 配置把更平滑的主路线解丢掉了，
> 只留下唯一通过审计的替代路线解。

### 4.3 两个问题的共同链路

```text
A* 路线是离散、非曲率感知的
  -> 替代路线带端点 dogleg
  -> 路线位置权重 5e5 强制路径贴合
  -> 曲率软权重只有 100，无法拉直

IPOPT 在主路线/强引导目标上频繁 max_iter
  -> 生产 solver="ipopt" 禁用 SLSQP fallback
  -> 主路线候选被丢弃
  -> 只剩替代路线候选可审计
  -> 选中歪扭但“可审计”的路径

几何审计 audit_position_budget_m=5e-8
  -> 长路径 U-turn 的 Simpson 余项+guard 约 9.9e-8
  -> 所有可行候选被拒绝
  -> 顶层 reason 取最后一个 attempt 的 max_iter
  -> 日志显示 Maximum_Iterations_Exceeded，误导为“优化器没解出来”
```

---

## 5. 原因分层

### 5.1 直接原因

1. **U-turn 失败**
   - 直接原因：`AUDIT_INTEGRATION_REMAINDER` 实际值 `9.67e-8 ~ 1.01e-7`
     大于 `audit_position_budget_m=5e-8`；
   - 64 步细化后仍不可达；
   - 前 5 个候选求解成功，但全部被此检查拒绝；
   - 第 6 个是 IPOPT max_iter，成为顶层 reason。

2. **路径歪扭**
   - 直接原因：主路线和自动初值候选大多 `Maximum_Iterations_Exceeded`；
   - 唯一可审计候选来自 A* 替代路线 2；
   - 该路线起点/终点 dogleg 的 9 节点曲率约 `0.7~0.84`；
   - 路径位置权重 `5e5` 强制贴合；
   - 最终曲率最大 `0.5475`、前轮 15.13°。

### 5.2 配置原因

1. `solver="ipopt"` 使 `allow_scipy_fallback=True` 形同虚设：
   - fallback 条件要求 `solver=="auto"`；
2. `max_iterations=120` 偏低；
   - 但单纯提高到 2000 仍不能解决主路线 IPOPT 收敛；
3. `max_seconds=30.0` 对多初值、多路线偏紧；
4. `ROUTE_GUIDANCE_WEIGHT=5e5` 与 `ROUTE_CURVATURE_WEIGHT=100`
   比例极端；
5. `kappa_limit=0.5525` 只是硬上限，不是“最小转向”目标；
6. `max_initials=6` 的排序让 3 条路线 + 3 个自动初值先占满，
   没有额外的“主路线专用平滑重试”。

### 5.3 算法/实现原因

1. A* 8 邻域 cell-center 折线没有平滑；
2. 替代路线的惩罚带机制天然产生“离开再汇入”的 dogleg；
3. 9 节点参数化对端点 dogleg 过于敏感；
4. 三点外接圆曲率会放大离散折线的局部拐点；
5. 候选选择只考虑“通过审计的候选”，不考虑失败候选的更低目标；
6. 顶层失败原因取 `attempts[-1]`，没有聚合/优先展示审计拒绝；
7. 几何审计的 `audit_position_budget_m` 与生产
   `integration_budget_m` 不一致（5e-8 vs 1e-7），对长路径过紧。

### 5.4 不是原因的部分

以下均已排除：

- 不是地图边界/未知栅格导致（同一图在 1e-7 下 SUCCESS）；
- 不是三车尺寸/横向 footprint 不可行（collision audit 通过）；
- 不是动力学超限（dynamic audit 通过）；
- 不是 ROS 通信丢失（reset 已确认，goal 已 admitted）；
- 不是 `initialpose`/goal 顺序问题（驱动等待了 RESET_CONFIRMED）；
- 不是地图比例错误（使用 `--scale 2.0`，栅格摘要与用户日志一致）。

---

## 6. 建议修复方向（尚未实施）

> 以下按优先级排列。任何改动都需要重新跑 G4/G5 回归与验收，
> 不能只改阈值让测试“变绿”。

### P0：修正 U-turn 假阴性

1. 将 `GeometryAuditConfig.audit_position_budget_m` 从 `5e-8`
   调整为与生产合同一致的 `1e-7`；
   - 已验证：同一请求在 1e-7 下三层审计全部通过；
   - 这是最小、最直接的修复；
   - 需要重新生成 G4/G5 证据并说明阈值变化。
2. 或者保留 5e-8，但改进 Simpson 余项/guard：
   - 研究 guard 随面板数增长的原因；
   - 对长路径使用更紧的 M4 包络或分段误差分配；
   - 目标是在不提高阈值的前提下达到 5e-8。
3. 增加明确的审计失败状态：
   - 例如 `AUDIT_NUMERICAL_BUDGET`；
   - 不要把数值证书失败归类为 `BOUND_UNCONFIRMED` 后又被 max_iter 覆盖。

### P0：修正原因聚合

1. `NO_ACCEPTED_CANDIDATE` 的顶层 `reason` 不应直接取
   `attempts[-1].rejection_reason`；
2. 建议优先级：
   - 有审计拒绝 → 报审计拒绝的约束和最佳候选；
   - 全部 solver 失败 → 报最常见的 solver 状态；
   - 混合 → 在 detail 中列出每个 attempt 的状态计数。
3. 日志应直接打印：
   ```text
   6 attempts: 5 solver-success/audit-rejected (AUDIT_INTEGRATION_REMAINDER),
   1 solver-max-iter
   ```
   而不是只显示最后一个。

### P1：修正路径歪扭

1. 让 SLSQP fallback 在 `solver="ipopt"` 时也可用：
   - 把条件从 `solver_choice == "auto"` 改为
     `allow_scipy_fallback and solver_choice in ("auto","ipopt")`；
   - 或者把生产默认改为 `solver="auto"`；
   - 已验证：主路线 + SLSQP 得到 max κ 0.19、5.37° 转角。
2. 对 A* 路线做曲率感知平滑/重采样：
   - 去除 cell-center 阶梯；
   - 起点/终点 dogleg 平滑；
   - 再用平滑后的路线做位置/曲率引导。
3. 调整目标权重：
   - 大幅提高 `ROUTE_CURVATURE_WEIGHT`（当前仅 100）；
   - 或降低 `ROUTE_GUIDANCE_WEIGHT`（当前 5e5）；
   - 增加显式 `∫|κ|² ds` 或转向角速率软惩罚；
   - 需要重新标定窄通道通过性，避免为了平滑牺牲间隙。
4. 增加候选质量门槛：
   - 若唯一可审计候选的目标函数远高于失败的主路线候选，
     应拒绝发布并报“主路线求解失败”，而不是接受歪扭替代路线；
   - 例如要求 accepted objective 不超过主路线失败 objective 的
     某个比例，或至少记录该比较。
5. 增加“直行/小转角优先”的重试：
   - 主路线失败时，先用平滑初值 + 更多迭代/SLSQP 重试；
   - 再考虑替代路线。

### P1：参数与审计一致性

1. `max_iterations` 与 `scipy_maxiter` 分离调参；
2. `max_seconds` 与 `virtual_a_planning_budget_s` 的关系要明确：
   - 当前 30 s 求解预算、60 s 作业预算、75 s 超时；
3. 将 `PlannerConfig` 的关键字段暴露为 launch 参数，便于复现实验，
   但保持默认不变；
4. 在 quality report 中输出每个 attempt 的 objective、residual、
   audit 约束，而不是只在失败时藏在 `virtual_result`。

### P2：测试与回归

1. 新增回归用例：
   - 4 m 空旷直线：要求 `max|κ| <= 0.2`、`max steer <= 6°`；
   - 长 U-turn：要求 `SUCCESS` 且三层审计通过；
   - 替代路线 dogleg：要求不得成为唯一可接受候选。
2. 对审计预算做边界测试：
   - 路径长度 5/10/15/20 m；
   - 曲率幅度 0.1/0.3/0.5；
   - 记录 64 步后的 `total_budget`，确认阈值可达性。
3. 保留 G5 的 `timing_continuous` 已知失败，不掩盖。

---

## 7. 复现命令汇总

```bash
# 1. 启动私有 master
export ROS_MASTER_URI=http://127.0.0.1:11587
roscore -p 11587

# 2. 启动 map05 2x，virtual 后端，headless
cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_virtual_a.sh --map map05 --scale 2.0 rviz:=false

# 3. U-turn 复现（另一个终端）
export ROS_MASTER_URI=http://127.0.0.1:11587
python3 /tmp/repro_uturn.py /tmp/repro_uturn_out

# 4. 只改审计预算的诊断
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
python3 /tmp/repro_audit_budget.py \
  /tmp/expv3-map-scale.*/map05-p12-parking-row-env2/map.yaml \
  1e-7 120

# 5. 直线曲率复现
python3 /tmp/repro_path_metrics.py \
  /tmp/expv3-map-scale.*/map05-p12-parking-row-env2/map.yaml \
  12.570,10.320,-0.0297339 \
  16.500,10.320,-0.0297339 \
  120 1e-7

# 6. 主路线 + SLSQP fallback
python3 /tmp/repro_path_metrics.py \
  /tmp/expv3-map-scale.*/map05-p12-parking-row-env2/map.yaml \
  12.570,10.320,-0.0297339 \
  16.500,10.320,-0.0297339 \
  300 1e-7 1 auto
```

注意：`/tmp/expv3-map-scale.*` 是 `run_demo.sh` 运行时生成的临时目录，
进程退出后可能被清理；复现时请使用当前运行实例打印的真实路径。

---

## 8. 证据文件

目录：`docs/virtual_formation_a/diagnostics_20261116/`

| 文件 | 内容 |
|---|---|
| `user_map05_plan600002.png` | 用户测试：plan 600002，开阔区域 S 形路径 |
| `user_map05_plan600004.png` | 用户测试：plan 600004，另一条 S 形路径 |
| `user_map05_plan600005.png` | 用户测试：plan 600005，明显弯道 |
| `user_map05_plan600006_failure.png` | 用户测试：U-turn 失败完整终端日志 |
| `repro_uturn_driver.log` | 本机 U-turn 复现驱动日志 |
| `repro_ros_planner.log` | 本机 planner ROS 日志 |
| `repro_ros_bridge.log` | 本机 bridge ROS 日志 |
| `repro_uturn_summary.json` | 6 个 attempt 的结构化摘要 |
| `repro_uturn_budget_1e7.log` | 只改审计预算为 1e-7 后 SUCCESS 的日志 |
| `repro_straight_ipopt120.log` | 4 m 直线默认配置的曲率/路线记录 |
| `repro_straight_primary2000.log` | 主路线 IPOPT 2000 次仍失败 |
| `repro_straight_slsqp_auto.log` | 主路线 + SLSQP fallback 成功且平滑 |

---

## 9. 关键参数速查

### 9.1 编队/车辆

```text
formation = cargo_wedge
offsets   = {1:(1.0,0.0), 2:(0.0,1.0), 3:(0.0,-1.0)} m
body      = 0.738 x 0.500 m
wheelbase = 0.494 m
track     = 0.364 m
r_min_ackermann = 0.81 m
v_max = 0.8 m/s
a_max = 0.6 m/s²
jerk_max = 2.0 m/s³
lateral_accel_max = 0.6 m/s²
yaw_rate_max = 0.8 rad/s
yaw_accel_max = 0.8 rad/s²
yaw_jerk_max = 3.0 rad/s³
steer_angle_max = 1.570796 rad
steer_rate_max = 0.8 rad/s
steer_accel_max = 1.5 rad/s²
```

### 9.2 规划

```text
intervals = 8
max_initials = 6
max_seconds = 30.0
max_iterations = 120
scipy_maxiter = 120
solver = ipopt
allow_scipy_fallback = True（但仅 solver=auto 生效）
ipopt_tolerance = 1e-8
scipy_tolerance = 1e-8
curvature_margin = 0.02
eta_limit = 2.0
kappa_limit = 0.552486...
```

### 9.3 路线

```text
max_routes = 3
max_expansions = 200000
max_seconds = 8.0
clearance_weight = 2.0
clearance_cap_m = 2.0
blocked_radius_m = 0.15
alternative_penalty = 2.0
```

### 9.4 目标权重

```text
ROUTE_GUIDANCE_WEIGHT = 5.0e5
ROUTE_CURVATURE_WEIGHT = 100.0
CLEARANCE_PROXY_WEIGHT = 5.0e4
CLEARANCE_PROXY_TARGET_M = 0.05
```

### 9.5 审计

```text
audit_panel_width_m = 0.02
audit_max_subintervals_per_segment = 4096
audit_max_total_subintervals = 20000
AUDIT_REFINEMENT_LIMIT = 64
audit_position_budget_m = 5e-8
integration_budget_m = 1e-7
POSITION_ROUNDOFF_GUARD_M = 1e-10
ANGLE_ROUNDOFF_GUARD_RAD = 1e-11
```

---

## 10. 最终判断

### 10.1 对问题二（回头弯）

- **可以规划**：几何、动力学、碰撞均可行；
- 当前失败是**审计数值预算 5e-8 对长 U-turn 路径不可达**；
- 不是“空旷区域没有可行解”；
- 修复优先级最高的是审计预算/余项算法，以及失败 reason 聚合。

### 10.2 对问题一（歪扭/大转向）

- **不是曲率约束不够紧**，而是没有足够强的曲率软目标；
- 主路线候选被 IPOPT 迭代上限丢弃；
- 生产配置实际上禁用了 SLSQP fallback；
- 唯一可审计的替代路线带端点 dogleg；
- 5e5 的路线位置权重把路径强拉向 dogleg；
- 100 的路线曲率权重无法拉直；
- 最终出现半径 1.83 m、15.1° 前轮转角的“不必要转向”。

### 10.3 建议的下一步

如果用户确认要修：

1. 先做 P0（审计预算 1e-7 + reason 聚合 + 回归）；
2. 再做 P1（fallback 生效 / 路线平滑 / 权重重标定）；
3. 每一项都要重新跑 G4/G5 和地图实测；
4. 不应直接改 `run_demo.sh` 默认后端，也不应通过放宽碰撞/动力学限制
   来“让测试通过”。

---

## 附录 A：本次复现使用的文件

- 驱动：`/tmp/repro_uturn.py`
- 审计预算诊断：`/tmp/repro_audit_budget.py`
- 路径曲率诊断：`/tmp/repro_path_metrics.py`
- 日志/截图：`docs/virtual_formation_a/diagnostics_20261116/`

## 附录 B：两个关键代码位置

- 审计预算与细化：
  `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/audit.py`
  - `_INTEGRATION_METHOD`
  - `_AUDIT_REFINEMENT_LIMIT = 64`
  - `_integration_plan()` 中 `threshold = config.audit_position_budget_m`
- 候选失败原因聚合：
  `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py`
  - `solve_open_path_guided()`
  - `status = "NO_ACCEPTED_CANDIDATE"`
  - `reason = attempts[-1].rejection_reason or "NO_AUDITED_CANDIDATE"`
- fallback 条件：
  `optimizer.py`
  - `if config.allow_scipy_fallback and solver_choice == "auto":`
- 路线位置/曲率权重：
  `optimizer.py`
  - `ROUTE_GUIDANCE_WEIGHT = 5.0e5`
  - `ROUTE_CURVATURE_WEIGHT = 100.0`
- A* 替代路线惩罚带：
  `pipeline.py`
  - `make_route_guides()`
  - `penalty[...] += config.alternative_penalty`
- 9 节点曲率估计：
  `optimizer.py`
  - `_node_route_curvature()`
  - `_route_guess()`

---

（完）