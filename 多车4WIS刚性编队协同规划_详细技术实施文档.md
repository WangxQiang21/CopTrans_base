# 单车 4WIS 到多车刚性编队 4WIS 协同规划：详细技术实施文档

> 版本：v1.0（2026-11-10）  
> 实施对象：`WangxQiang21/CopTrans_base`，分支 `continuous-mode-path`，已记录基线 SHA `956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee`。  
> 工作范围：**仅规划**（几何模型、搜索、模式可行性、连续几何、时序、独立审核、规划输出与回放）。**不实施控制器、NMPC、执行器、车辆状态闭环仿真改造**。  
> 面向读者：需要按任务单逐步改代码的编程 AI、项目开发者、毕业论文实验实施者。  
> 状态约定：`[现状已核实]` = 来自给定源码/两份文件；`[设计决策]` = 本文建议采用的实现合同；`[待确认]` = 必须经项目负责人确认，不能擅自变更论文目标；`[验收]` = 必须能以测试/运行日志证明。  
> **文档不是已实现成果证明**。本次未在 ROS1 Noetic 环境重新构建或运行完整规划测试；性能和通过率不得按本文推断。

---

## 0. 执行规范（必须先读）

### 0.1 执行总原则

1. **先读源码，再改代码**；文件路径不存在或函数签名不同，先记录差异，再修改任务单，不能自行编造等效接口。
2. **一次只做一个小任务/一个提交**；每次修改后运行最小单测，再运行关联回归；提交前 `git diff --check`。
3. **保持单车可运行**。新的多车功能通过新节点/新消息/新类和功能开关进入，不直接改变单车 `/plan/*` 的原有行为；不得大范围重写 `core.py`、`mode_master.py` 和 `optimizer_node.py`。
4. 轨迹只有满足**几何 + 碰撞 + 运动学 + 同步时间 + 动态约束 + 发布采样**全部检查，才能标记 `FEASIBLE`。几何可行不可冒充可执行轨迹。
5. **刚性队形是硬约束**，不允许使用大权重 `formation_cost` 来替代；也不允许优化失败时微调车间距“修复”。
6. 多车所有输出需要同一 `plan_id`、同一 `map` 坐标系、同一时间原点、同一时间轴；多车协同时间为联合决策，不可简单把各车独立重定时输出拼在一起。
7. 不要先做多车控制。规划输出接口必须为未来的 4WIS 轨迹跟踪保留每车 `mode`、车体航向、时间和速度信息。
8. 若遇到 `FAIL_*`，应返回结构化失败结果、保留诊断，不应输出旧规划的“成功轨迹”。

### 0.2 论文问题与不可偷偷放松的边界

研究对象：狭窄通行环境下，多台 4WIS 车辆保持固定的相对几何关系，通过静态受限地图的协同规划。车辆支持 A（Ackermann）、O（Oblique）和 S（Spin）三类运动模式，允许根据单车模型分配不同的模式。

**“多车间距要相同”有两种不等价的解释：**

- **解释 A：固定刚性队形（本文的默认软件抽象，但须确认）**。任意车辆对的距离 `d_ij(t) = d_ij(0)`，不同车辆对的数值可以不同。例如楔形队形两条斜边与后排横边不必等长。
- **解释 B：所有两两距离都相等**。3 辆车中心在平面上构成等边三角形（退化情形除外），这是额外的特殊队形约束。对 4 辆及以上互不重合的二维车中心，不能全部两两等距。

**执行规则**：把队形作为可配置 `offsets` 输入，而不是写死楔形或等边。论文用哪种队形，须由负责人确认；在确认之前，先完成“任意固定偏移”几何与单测。

**非目标**：非刚性编队变形、队形切换、分散后再集合、动态障碍物重规划、车货力学、控制稳定性证明、实车执行器控制。若完全刚性的编队因横向占用无法通过窄门，应该返回 `FORMATION_NO_GEOMETRIC_PATH`，而不是改变间距。

---

## 1. 代码现状与真实依赖

### 1.1 输入文件依据

1. `CURRENT_REPOSITORY_STATUS.md`：2026-11-09 对当前 `continuous-mode-path` 分支的静态源码梳理；其中明确：ROS1 Noetic，7 个活跃包，当前主后端 `mode_path_v2`，仍为**单车规划**。
2. `多车协同控制算法框架详解.md`：旧实验是 3 车楔形、领航预测共享、速度协同、分车 NMPC；**不是多车协同规划器**，没有联合避碰硬约束，使用普通自行车模型，不能替代本次 4WIS 规划。
3. `毕业论文项目先验知识_精简版.md`：论文核心是固定相对几何关系的多车 4WIS 在狭窄/急弯/障碍物环境中规划与控制；运动学/货物模型不应过早锁死。
4. `RANGER MINI 2.0 用户手册.pdf` 及仓库 `fwis_common/config/ranger_mini_2.yaml`：需在真正接入运动学时再次核验机械/驱动合同，尤其最大轮角、最大轮速、底盘坐标定义和模式转换要求。当前 YAML 不是完整实车动态标定。
5. GitHub 代码路径：`https://github.com/WangxQiang21/CopTrans_base/tree/continuous-mode-path`。现状核验以**指定提交实际源码**优先于历史 `plan/` 或 `archive/`。

### 1.2 单车主链（[现状已核实]）

```text
/map + /start_pose + /move_base_simple/goal
   ↓
fwis_v3_planner/src/planner_node.cpp (C++; PlannerNode)
   ├─ pose_clear(): 单车 OBB—栅格 SAT
   ├─ action_clear(): 单车动作扫掠
   ├─ oblique_route()/sparse_mixed_route()/search(): 多种 A/O/S 种子
   └─ /plan/seed 及 /plan/seed_candidates
   ↓
fwis_v3_optimizer/scripts/optimizer_node.py
   └─ run_mode_path() → ModePathMaster.solve(...)
      ├─ continuous_geometry.py: ContinuousPlan/MotionBlock/ControlKnot
      ├─ continuous_optimizer.py: 固定结构局部连续优化
      ├─ continuous_retiming.py: 时间缩放、模式事件
      └─ quality.py: TrajectoryEvaluator 独立审核
   ↓
/plan/timed_trajectory (TimedTrajectory) + /plan/path
   ↓
fwis_v3_playback/scripts/playback.py （开环回放，并非闭环控制）
```

**重要的具体实现事实**：

- `catkin_ws/src/fwis_v3_msgs/msg/ModePrimitive.msg`：`mode:uint8`、`amount:float64`、`control:float64`，A=0、O=1、S=2。A：`amount` 是有符号移动弧长、`control` 是中心曲率；O：有符号平移量、斜移方向角；S：有符号自转角。
- `PlanSeed.msg` 目前只有 `header, plan_id, start_center, goal_center, primitives[]`；`TimedTrajectory.msg` 只有 `header, plan_id, points[]`。**没有 vehicle_id，不能直接表达联合多车请求。**
- `TimedPoint.msg` 字段为 `time_from_start`、`center_pose`、`v`、`mode`；车体坐标点是**前后轴中点/车辆中心**。当前 `v` 为中心平移带方向语义；Spin 时平移速度为 0。
- C++ 前端的 `PlannerNode::on_goal()` 是规划入口；`pose_clear()` 使用旋转整车 OBB SAT 检查，`action_clear()` 执行扫掠；`sparse_mixed_route()` 负责稀疏路线图混合动作候选；`search()` 执行带模式、倒车和切换历史状态的搜索。**当前前端不仅是普通 Hybrid A\***。
- Python `core.py` 定义基础 `Pose/Action`、动作传播、碰撞等；`continuous_geometry.py` 定义 `ControlKnot/MotionBlock/ContinuousPlan`；`mode_master.py` 定义 `ModePathMaster`；`quality.py` 定义 `TrajectoryEvaluator`。
- `ModePathMaster(evaluator, config).solve(seed, auxiliary_seeds=...)` 处理候选；`TrajectoryEvaluator.evaluate_continuous_geometry(plan, timed=...)` 审核连续几何、完整动态/时间等。当前 evaluator 绑定**一车**的起终点和尺寸。
- `fwis_v3_playback/scripts/playback.py` 用固定 Marker ns/id，直接起三个实例会覆盖，必须添加多车独立 namespace/id。
- `run_demo.sh` 当前固定 `mode_path_v2` 后端；`archive/` 不参与默认构建，`plan/` 是历史资料，不能作为当前实现的事实来源。

### 1.3 当前单车参数基线（仅供回归对照，来自 `CURRENT_REPOSITORY_STATUS.md`）

| 参数 | 基线值 | 说明 |
|---|---:|---|
| 车长/车宽 | 0.738 / 0.500 m | `ranger_mini_2.yaml` |
| 轴距/轮距 | 0.494 / 0.364 m | `ranger_mini_2.yaml` |
| Ackermann 最小半径 | 0.810 m | 单车 A 可行性 |
| 规划步长 | 0.20 m | 前端默认 |
| 搜索 XY / yaw 分辨率 | 0.10 m / 10° | 前端默认 |
| 前端搜索预算 | 450000 iterations | 当前 YAML（勿用旧值） |
| `quality.search.budget_s` | 8 s | 协作预算，不是硬实时 |
| 最终目标审核 | 0.02 m / 1° | 单车已定义的最终审核合同 |
| `quality.cost.sweep_spacing` | 0.005 m | 单车独立扫掠 |
| 模式切换驻留 | 总 0.3 s（设置时间 0.15 s 已包含） | 示例 profile |
| 默认速度/加速度/jerk | 2 m/s、0.6 m/s²、2 m/s³ | 现有时间 profile；不是实车保证 |
| Spin 角速度/角加速度/角 jerk | 0.8 rad/s、0.8 rad/s²、3 rad/s³ | 当前 profile |

**禁止**为提高新多车算法成功率直接放宽上述单车最终审核阈值。新算法确有新增公差时，应独立设置并说明来源。

### 1.4 旧控制实验哪些可借鉴（[现状已核实]）

- 可借鉴：三车固定队形偏移表示、领航/跟随几何关系、车 ID 分离、轨迹发布/显示和评价方式。
- 不可直接复用：用 `Path_pub.py` 对同一 `ref_path` 生成偏移路径的做法；该操作本身**不验证整车碰撞/每车 4WIS 可行性**。
- 旧三车默认控制期望偏移是 `r0=(0,0)`、`r1=(-3,+2)m`、`r2=(-3,-2)m`；但旧的**全局 Path 偏移**为 `car0=(0,0)`、`car1=(0,+2)m`、`car2=(0,-2)m`，两者不能混为一谈。旧仿真初始跟车横向偏移也不完全匹配期望偏移。
- 旧 NMPC 是每车独立求解并使用编队误差**软代价**；不保证刚性距离精确恒定，也没有完整多车地图/车间碰撞硬约束。

---

## 2. 数学模型与约束

### 2.1 坐标和队形数据合同

世界坐标系 `map`；编队坐标系 `F`；车体坐标系 `B_i`。编队参考位姿 `q_F=(x_F,y_F,ψ_F)`，第 `i` 车相对于 `F` 的固定中心偏移 `r_i=(a_i,b_i)`。每辆车车体航向 `θ_i` **不默认等于** `ψ_F`。

定义：

\[
R(\psi)=\begin{bmatrix}\cos\psi&-\sin\psi\\\sin\psi&\cos\psi\end{bmatrix},\quad
J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}.
\]

固定刚性位置合同：

\[
\boxed{p_i(t)=p_F(t)+R(\psi_F(t))r_i.}
\]

立刻有 `||p_i-p_j|| = ||r_i-r_j||`。**几何硬约束**：对所有时间、全部车对成立；不能仅靠代价惩罚实现。

微分：

\[
\dot p_i=\dot p_F+\omega_F R(\psi_F)Jr_i, \quad \omega_F=\dot\psi_F.
\]

若进一步需要车体相对朝向固定，增加（**可选，待确认**）：`wrap(θ_i−ψ_F−α_i)=0`。强制该约束可能大幅缩小运动可行集；第一轮采用**固定中心队形、独立车体航向**，每次更改此假设都必须记录。

### 2.2 每车三模式可行性（必须按真实模型验证）

令车体航向单位向量 `e(θ)=(cosθ,sinθ)`，左法向 `n(θ)=(-sinθ,cosθ)`。设 `v_i` 是车中心**有符号平移速度**，`ω_i=dot θ_i`。

**A：Ackermann**（当前单车中心曲率 `κ_i` 版本）

\[
\dot p_i=v_i e(\theta_i),\quad \dot\theta_i=v_i\kappa_i,
\quad |\kappa_i|\le 1/R_{\min}.
\]

当 `v_i≠0`：要求 `dot p_i` 与 `e(θ_i)` 共线（可正向或倒车）；由 `ω_i/v_i` 得到曲率并检查限制。`v_i=0` 时不能靠 Ackermann 产生非零中心运动或非零原地角速度。

**O：Oblique**（当前单车斜移方向 `φ_i` 版本）

\[
\dot p_i=v_i e(\theta_i+\phi_i),\quad \dot\theta_i=0,
\quad |\phi_i|\le\pi/2
\]

实际限制必须服从仓库 `quality.py` 的合法性审核、斜行角度/倒车判定和具体轮系的约束。特别注意：O 轨迹的车体航向在每个 O 段应不变；速度向量方向可以因 `φ` 变化，但能否动态平滑必须在连续几何和转向速率规则中检查。

**S：Spin**

\[
\dot p_i=0,\quad \dot\theta_i=\omega_i.
\]

**S 并非编队绕其参考中心旋转。** 当 `ω_F≠0` 且 `r_i≠0`，通常 `dot p_i≠0`，不能标记该车辆为 S。对于旋转中心恰好位于某辆车中心的情况，该车可能使用 S，其他车辆必须运动并由 A/O 等模式满足对应速度。

### 2.3 真正的难点与不可行处理

给定编队参考曲线 `q_F(t)` 后，`p_i(t)` 已由刚性关系完全决定，但每车 `θ_i(t)`、`mode_i(t)` 必须满足前述模式运动学。**不是任意光滑编队曲线都能由 N 辆 4WIS 车实现。** 例如：某车中心在运动、却被分配 S；O 段要求车体航向转动；A 段要求非零法向速度；A 段局部曲率超上限；模式切换未停车。这些情况都必须拒绝该候选。

处理优先级：

1. 更改本段允许的每车模式序列或车体航向方案；
2. 调整编队整体的连续几何/曲率；
3. 返回高层搜索，探索其他通道、方向或旋转中心；
4. 若仍无解，输出 `KINEMATIC_INFEASIBLE` 或 `MODE_SCHEDULE_INFEASIBLE`，**不得改变刚性距离**。

**设计决策：两级可行性**。第一层 `geometry_only` 仅用于剔除地图碰撞；第二层 `kinematic_feasible` 才能参加时间求解；通过完整联合审核后才是 `FEASIBLE`。不要把 `geometry_only` 返回结果当作可执行计划。

### 2.4 起终状态的必要校验

输入可以采用 1 个编队位姿 + 固定偏移 + 每车体航向，也可以采用 N 辆车的绝对位姿反求编队位姿，但**不得混用而不验证**。

- 若输入 N 辆车绝对中心：检查任意车对距离与 `||r_i−r_j||` 的误差在配置容差内；还应检查它们可由**同一个**平面刚性变换拟合（只有 pairwise 距离检查可能遗漏镜像/车辆编号交换）。
- 若输入 `q_F` + `r_i`：直接生成所有车中心，再逐车检查地图边界、障碍物占用、车间物理碰撞。
- 每车初始/终端车体航向如果指定，必须参与 A/O/S 终端可行性检查；不能直接忽略。若未指定，消息中必须显式标记 `heading_free`，不能凭空假定等于 `ψ_F`。
- 目标形状如果在几何上无法放置或车间车体互撞，立即拒绝，不进入搜索。

---

## 3. 架构方案比较与选型

| 方案 | 优点 | 致命问题/代价 | 决策 |
|---|---|---|---|
| N 次独立单车搜索后拼接 | 易复用现成单车规划器 | 不保证固定队形、时间同步，可能各车独立可行但共同不可行 | **拒绝作为主算法**；仅可作实验对照 |
| 一条编队参考几何 + 每车模式可行性 + 联合时间规划 | 刚性关系天然成立；搜索维度较低；沿用现有前后端架构 | 对模式/车体航向分配需独立解决，分层方法不保证全局完备 | **推荐主线** |
| N 车全联合状态搜索 | 表达能力强，理论上可覆盖更多可行组合 | 维度随 N 激增、状态哈希及运动原语设计复杂、性能风险大 | 后续对照/增强，不作为第一版本 |

**推荐技术路线（[设计决策]）**：`FormationModel → FormationCollisionChecker → 编队参考前端候选 → 每车模式/航向可行性 → FormationContinuousPlan → 联合局部优化 → 共同时间缩放 → 独立 FormationEvaluator → 同步 ROS 发布`。

**限定策略（第一可运行版本）**：先支持 `N=1` 和 `N=3`、静态地图、同车型、固定 offset；先做队形平移、有限旋转、在整体静止时换模；逐步添加不同车型及更灵活的非同步模式变化。虽然数据结构必须支持 `N>=1`，**不要谎称最初版本已经可靠支持任意 N**。

---

## 4. 模块与文件实施蓝图

> 下列相对路径以仓库根目录为基准。`[新建]` 均为**建议文件，现状中不存在**；实施前需通过 `find`/`git ls-files` 再核实。老模块只增适配层，优先复用公共几何函数，不复制大段代码。

```text
catkin_ws/src/
  fwis_common/
    config/ranger_mini_2.yaml                   [保留现有]
    include/fwis_common/vehicle.hpp             [只读/核验]
    include/fwis_common/kinematics.hpp          [只读/核验，注意是历史兼容头文件]
  fwis_v3_msgs/
    msg/ModePrimitive.msg                       [保持原接口]
    msg/PlanSeed.msg                            [保持原接口]
    msg/TimedPoint.msg                          [保持原接口]
    msg/TimedTrajectory.msg                     [保持原接口]
    msg/FormationVehicleSpec.msg                [新建]
    msg/FormationPlanRequest.msg                [新建]
    msg/FormationPrimitive.msg                  [新建]
    msg/FormationPlanSeed.msg                   [新建]
    msg/FormationTimedPoint.msg                 [新建]
    msg/FormationVehicleTrajectory.msg         [新建]
    msg/FormationTimedTrajectory.msg            [新建]
  fwis_v3_planner/
    src/planner_node.cpp                         [保留单车]
    include/fwis_v3_planner/formation_model.hpp  [新建]
    include/fwis_v3_planner/formation_collision.hpp [新建]
    include/fwis_v3_planner/formation_search.hpp [新建]
    src/formation_planner_node.cpp               [新建]
    src/formation_search.cpp                     [新建]
    CMakeLists.txt                               [只增新可执行目标]
  fwis_v3_optimizer/
    scripts/optimizer_node.py                    [尽量保留单车]
    scripts/formation_optimizer_node.py          [新建]
    src/fwis_v3_optimizer/core.py                [原则上保持原有]
    src/fwis_v3_optimizer/formation.py           [新建]
    src/fwis_v3_optimizer/formation_collision.py [新建]
    src/fwis_v3_optimizer/formation_geometry.py  [新建]
    src/fwis_v3_optimizer/formation_kinematics.py [新建]
    src/fwis_v3_optimizer/formation_master.py    [新建]
    src/fwis_v3_optimizer/formation_optimizer.py [新建]
    src/fwis_v3_optimizer/formation_retiming.py  [新建]
    src/fwis_v3_optimizer/formation_quality.py   [新建]
  fwis_v3_demo/
    config/params.yaml                           [保持单车]
    config/formation_params.yaml                 [新建]
    launch/demo.launch                           [保持单车]
    launch/formation_demo.launch                 [新建]
  fwis_v3_playback/
    scripts/playback.py                          [保持单车]
    scripts/formation_playback.py                [新建]
tests/
  test_formation_model.py                        [新建]
  test_formation_collision.py                    [新建]
  test_formation_kinematics.py                   [新建]
  test_formation_geometry.py                     [新建]
  test_formation_quality.py                      [新建]
  test_formation_retiming.py                     [新建]
  test_formation_contract.py                     [新建]
  ros_formation_smoke.py                         [新建]
tools/
  run_formation_regression.py                    [新建]
  formation_eval_cases.yaml                      [新建]
```

**关于模型双份实现风险**：C++ 前端和 Python 后端对坐标、OBB 几何、动作可行性必须**同一合同**，不能分别使用不同膨胀边界；通过跨语言 golden tests 校验。同一车体真实碰撞始终由 Python 独立审核做最终决定，C++ 前端只是候选筛选。对单车 `fwis_common/kinematics.hpp` 内的历史 `Lateral/Diagonal` 枚举，不得臆断是当前 v2 的三个动作的唯一权威实现。优先以 `planner_node.cpp`、`core.py`、`quality.py`、消息定义的当前逻辑为准。

---

## 5. 消息、状态、单位和接口合同（先冻结设计，再修改代码）

### 5.1 ID / 时间 / 状态通用规范

- `formation_id`：编队配置身份，可复用字符串名，如 `wedge_3`，**不是**规划请求 ID。
- `plan_id`：每次规划请求的递增 `uint32`；同一请求下所有车辆都使用同一个值。
- `vehicle_id`：不重复的标识，建议 `uint32` 取 `0..N-1`；和 `offsets` 顺序一一对应。
- `header.frame_id = "map"`；`header.stamp` 为统一生成时间或共同轨迹起始时间（发布端必须单独规定并固定）。
- 所有平移单位 m；航向/斜角/旋转量 rad；线速度 m/s；角速度 rad/s；时间 s（消息使用 `duration`）；曲率 1/m。
- `ψ_F` 表示编队参考方位，`θ_i` 表示车体航向，二者是不同字段。
- 离散 A/O/S 模式按当前 `ModePrimitive` 的常量 `0/1/2`，不新增无法对应的神秘模式整数。
- 失败请求必须发布 `plan_id` 相关的失败状态并清空旧成功结果；旧 `plan_id` 的结果不可覆盖新请求。

### 5.2 建议 ROS `.msg` 定义（[设计决策]；不是仓库现有字段）

```text
# FormationVehicleSpec.msg
uint32 vehicle_id
geometry_msgs/Point offset_in_formation  # z 恒为 0, x/y 单位 m
geometry_msgs/Pose2D start_center       # 含车辆车体 θ_i，不是 ψ_F
geometry_msgs/Pose2D goal_center        # 若终端航向自由，见下方标志
bool goal_heading_free
float64 body_length_m
float64 body_width_m
```

```text
# FormationPlanRequest.msg
std_msgs/Header header
uint32 plan_id
string formation_id
geometry_msgs/Pose2D formation_start    # x_F,y_F,ψ_F
geometry_msgs/Pose2D formation_goal
fwis_v3_msgs/FormationVehicleSpec[] vehicles
bool allow_reverse
```

```text
# FormationPrimitive.msg
uint8 TRANSLATION=0
uint8 ROTATION=1
uint8 COMPOSITE=2
uint8 geometry_type
float64 amount                           # 平移 m / 旋转 rad；复合由下方控制解释
float64 control                          # 首版定义由 formation_geometry_contract 文档具体细化
```

`FormationPrimitive` **不能直接复用**单车 `ModePrimitive` 的 `amount/control` 语义，因为编队参考运动的平移/转动，与每辆车的 A/O/S 模式不同。首版可仅实现 `TRANSLATION` 和 `ROTATION`；`COMPOSITE` 未定义前必须拒绝，不能默默当平移。

```text
# FormationPlanSeed.msg
std_msgs/Header header
uint32 plan_id
string formation_id
geometry_msgs/Pose2D formation_start
geometry_msgs/Pose2D formation_goal
fwis_v3_msgs/FormationPrimitive[] primitives
```

```text
# FormationTimedPoint.msg
 duration time_from_start
 geometry_msgs/Pose2D center_pose
 float64 center_speed_mps
 float64 center_yaw_rate_radps
 uint8 mode
 float64 mode_control                  # A: κ[1/m]；O: φ[rad]；S: ω 控制另见约定
```

```text
# FormationVehicleTrajectory.msg
 uint32 vehicle_id
 fwis_v3_msgs/FormationTimedPoint[] points
```

```text
# FormationTimedTrajectory.msg
 std_msgs/Header header
 uint32 plan_id
 string formation_id
 fwis_v3_msgs/FormationVehicleSpec[] vehicles
 geometry_msgs/Pose2D[] formation_poses    # 对齐共同时间网格
 duration[] times_from_start             # 唯一共同时间网格
 fwis_v3_msgs/FormationVehicleTrajectory[] trajectories
```

**一致性规则**：所有 `trajectories[i].points` 长度与 `times_from_start` 长度一致；每个 `time_from_start` 严格一致；模式切换前后可以有同位置时间样本，但时间必须单调递增；车体航向和参考方位应按连续展开规则进行审核。消息里 `mode_control` 不能单独证明车轮级可执行，必须由离线审核和后续控制接口补充。

**需要二次确认的字段**：`FormationTimedPoint` 目前没有记录各车轮速/轮角；如果后续控制侧要求实际执行器指令，应另建控制命令消息，不要把执行器输出放入本期规划器。`formation_poses[]` 的 `Pose2D.theta` 是 `ψ_F`。真实建消息时检查 ROS `message_generation` / `message_runtime` 依赖及所有 `.msg` 导入。

### 5.3 参考 YAML（[设计建议]，不得当作仓库默认现状）

```yaml
formation:
  enabled: true
  formation_id: wedge_3
  reference_frame: map
  semantics: fixed_offsets    # 待确认；不是 all_pairs_equal
  vehicles:
    - {id: 0, offset_x: 0.0, offset_y: 0.0}
    - {id: 1, offset_x: -3.0, offset_y: 2.0}
    - {id: 2, offset_x: -3.0, offset_y: -2.0}
  strict_position_tolerance_m: 1.0e-6
  relative_heading_policy: independent
  mode_switch_policy: all_vehicles_stopped
  max_vehicle_count_for_v1: 3
  require_complete_certification: true
  publication_frame_id: map
```

这套偏移源自旧控制实验，只是第一批联调样例；**要先检查地图是否足以容纳这么大的队形**，否则性能测试毫无意义。首批碰撞/算法单测应另外使用小尺寸合理队形进行验证，不能把不可通过地图误判为算法错误。

---

## 6. 分阶段实现总表与完成定义

| 阶段 | 本阶段唯一交付物 | 依赖 | 验收标准 | Git 检查点 |
|---|---|---|---|---|
| S0 | 冻结单车基线和运行证据 | 无 | 现有单测及固定地图结果有日志；无法运行时如实记录 | `baseline/single-v2` |
| S1 | FormationModel + 输入校验 | S0 | 刚性坐标变换、距离保持、起终校验单测通过 | `feat/formation-model` |
| S2 | 多车 OBB / 扫掠碰撞 | S1 | 车障碍/车间碰撞、旋转扫掠、地图边界测试通过 | `feat/formation-collision` |
| S3 | ROS 多车消息与接口契约 | S1 | catkin 构建、序列化/时间网格合同测试通过 | `feat/formation-msgs` |
| S4 | C++ 编队前端初始几何搜索 | S2,S3 | 输出整编队不碰撞种子，严格区分几何-only | `feat/formation-frontend` |
| S5 | A/O/S 模式可行性与航向分配 | S4 | 拒绝不合法模式；通过纯平移及旋转可行案例 | `feat/formation-modes` |
| S6 | Python 连续几何 + 候选管理 | S5 | 不破坏刚性位置、不引入 O 段航向转动 | `feat/formation-continuous` |
| S7 | 固定模式结构联合连续优化 | S6 | 硬约束残差/几何/模式审计通过 | `feat/formation-nlp` |
| S8 | 全车统一 retiming | S6,S7 | 公共时间网格、速度/加速度/jerk/驻留全通过 | `feat/formation-time` |
| S9 | FormationEvaluator 独立审核及发布 | S8 | 失败不冒充成功；全部约束逐车逐段可复核 | `feat/formation-audit` |
| S10 | RViz/脚本/性能实验 | S9 | 端到端回归及对照数据，有失败诊断 | `feat/formation-eval` |

**推荐实施策略**：S0–S3 完成后先演示“给定编队路径的多车碰撞检测”；S4 完成后可显示 `GEOMETRY_ONLY` 候选，但**绝不能输出可执行轨迹**；S5–S9 完成后才称完整多车 4WIS 规划器。S7 若短期困难，可以用候选结构 + 可行性审核 + retiming 形成**无连续优化版基线**，但要明确该版本没有执行 S7，且所有轨迹仍必须完整审计。

---

## 7. S0：基线冻结（先执行，不改算法）

**输入**：当前仓库和 ROS1 Noetic 环境。**输出**：基线记录、冻结 commit、回归日志。

1. `git status --short`；若有用户未提交更改，禁止 `reset --hard`、禁止清理用户文件，先记录并选择新工作区或分支。
2. `git rev-parse HEAD` 与 `git branch --show-current`；检查 SHA 是否等于文档顶部；不等时记录真实 SHA 并重新审查差异。
3. 阅读 `CURRENT_REPOSITORY_STATUS.md`、`README.md`、`run_demo.sh`、`catkin_ws/src/fwis_v3_demo/launch/demo.launch`、车辆 YAML、算法 YAML；确认本项目实际运行命令。
4. 在**当前工作目录下**冻结用于复现的脚本命令/设备信息，不要复制文档里的个人主机路径。
5. 尝试：

```bash
# 以下命令需在含 catkin_ws、run_demo.sh 的仓库根目录执行
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'
# 有 ROS Noetic 时：
source /opt/ros/noetic/setup.bash
catkin_make -C catkin_ws -j1
# 固定地图示例（须检查 map04 运行配置是否存在）
bash run_demo.sh --map map04 rviz:=false
```

6. 记录 `environment.md`：Python、NumPy、SciPy、CasADi、IPOPT、ROS 版本；`baseline_cases.csv`：地图、比例、起终点、是否成功、规划耗时、执行时长、模式次数、碰撞/终点审核状态。
7. 新建开发分支，不要在当前能复现单车成果的分支上实施大规模改造。

**[验收]**：基线单测真实运行并记录结果；如 ROS 不可用，也必须保留未验证状态而不是写“已通过”。`git diff --check` 干净。

---

## 8. S1：FormationModel 与严格输入校验

**文件**：新增 Python `formation.py`、`tests/test_formation_model.py`；C++ `formation_model.hpp` 在 S4 前引入以免两套初期实现同步难以维护。

**数据类设计**：

```python
# formation.py: 设计骨架。实际导入当前 core.Pose 时核验字段 x/y/th。
from dataclasses import dataclass
from math import cos, sin, hypot

@dataclass(frozen=True)
class VehicleOffset:
    vehicle_id: int
    x: float
    y: float

@dataclass(frozen=True)
class FormationPose:
    x: float
    y: float
    psi: float

@dataclass(frozen=True)
class VehiclePose:
    vehicle_id: int
    x: float
    y: float
    theta: float  # 来自独立车体航向；不得自动写成 psi

class FormationModel:
    def __init__(self, offsets):
        # TODO: 检查 N>0、ID 不重复、偏移有限、零距离碰撞等
        ...

    def centers(self, qf):
        c, s = cos(qf.psi), sin(qf.psi)
        return {
            r.vehicle_id: (qf.x+c*r.x-s*r.y, qf.y+s*r.x+c*r.y)
            for r in self.offsets
        }

    def validate_centers(self, qf, given_centers, tol):
        # TODO: 每个 vehicle_id 的刚性重建残差、完整性、重复 ID
        ...
```

**逐项执行**：

1. 读 `core.Pose` 是否有 `th` 属性，统一 Python 内部对 `theta` 的命名转换，不改现有类字段。
2. `offsets` 按 `vehicle_id` 排序，对重复、非有限、缺失 ID 立即抛带上下文错误。
3. 编队位姿有限性检查：所有 `x,y,ψ` 必须有限；航向角存储允许展开，但算旋转矩阵无需预先 wrap。
4. 实现 `centers(q_F)`；实现通过一组指定 `body_headings` 构造 `VehiclePose[]`；**不能让 center 计算函数负责选择车体航向**。
5. 实现 `pairwise_reference_distances()`、`rigidity_residual()`、`validate_vehicle_centers()`；输入中任一车辆缺失或额外车辆即失败。
6. 实现可选 `fit_formation_pose()`（若需要从 N 车位姿反求）：用二维刚性 Procrustes/最小二乘，**禁止反射变换**，检查重建误差；若 1 个车或几何退化导致 `ψ_F` 不唯一，则必须由请求显式给出 `ψ_F`。
7. 添加测试：`ψ=0, π/2, -π/2, π`、`N=1/2/3`、非零参考偏移、重复 ID、NaN、反射队形、换 ID 后失败、刚性距离误差小于 `1e-10`（纯浮点计算单测建议阈值）。

**[验收]**：所有单测通过，`FormationModel` 不依赖 ROS 节点即可 import；数学映射与反变换的数值误差有断言。**不进入地图搜索**。

---

## 9. S2：多车 OBB 与扫掠碰撞

**文件**：新增 `formation_collision.py`、`test_formation_collision.py`。C++ 对应 `formation_collision.hpp` 在 S4 接入。优先调用已有 `core.Grid` / `quality.FootprintDistance` 的公开能力，但必须先检查其参数解释与缓存约束。

### 9.1 API 合同

```python
class FormationCollisionChecker:
    def __init__(self, grid, formation_model, vehicle_shapes, margin_m): ...
    def static_check(self, qf, body_headings):
        # 返回 CollisionReport(success, reason, vehicle_i, vehicle_j, ...)
        ...
    def swept_check(self, states, policy):
        # states: 按弧长或时间有序的全车联合状态，不是单车 centers
        ...
```

`static_check()` 必须先做：车辆真实中心恢复 → 每车 OBB 与地图/边界碰撞 → 所有车对 OBB 碰撞；可对接 `FootprintDistance` 的 per-vehicle 查询，但不得仅检查车中心点。**车间碰撞依据物理尺寸与可配置安全边距**，不能把 `||center_i-center_j|| > width` 当作所有航向条件下的充分条件。

### 9.2 连续扫掠安全策略

对相邻 `q_F^a,q_F^b`，定义 `Δtranslation=||p_F^b-p_F^a||`、`Δψ=wrap(ψ_b−ψ_a)` 的实际**连续旋转增量**（长转角需保存展开值，不能简单用最短角）。令 `Rmax = max_i ||r_i||`，每车角点到中心的最大半径 `ρ_i = 0.5 hypot(length_i,width_i)`。保守的单区间车体角点运动位移上界可取：

`D_bound = Δtranslation + Rmax * |Δψ| + max_i(ρ_i * |Δθ_i|)`。

将该量与空间检查步长相比较，细分动作；每段采样检验每车障碍物以及**车间** OBB。静态样本点清晰不意味着采样间必然清晰，必须采用类似现有 `action_clear()` 的运动保护膨胀/加密机制。**此位移上界只是采样密度控制，不单独构成连续安全证明**；最终仍由高密度独立审计确认。注意 `ψ_F` 和 `θ_i` 各有变化量，不能把 `Δψ_F` 当作 `Δθ_i`。

**局部测试（至少 12 个）**：

1. N=1 与旧单车相同位姿的碰撞结果一致。
2. N=3，车0安全但车1撞墙 → 失败且指出 id=1。
3. 车间两个 OBB 旋转后重叠 → 失败。
4. 车中心间距没变但车体朝向使 OBB 重叠 → 失败。
5. 所有端点可行、旋转中间某角度与墙碰撞 → 扫掠失败。
6. 所有端点可行、旋转途中两车车体发生重叠 → 扫掠失败。
7. 地图边界触碰 → 保守失败。
8. 宽度刚好等于需要净空的窄门 → 按安全边距合同判定，不采用“中心可过”捷径。
9. 空地图下的多段直线 → 成功。
10. 轨迹采样空/不连续/包含 NaN → 失败。
11. 不同 N=1/2/3 → 能处理。
12. `Δψ` 超过 π 时使用展开角实现正确扫掠，不把整周旋转误判为零动作。

**[验收]**：与现有单车 OBB 结果的 golden 对照一致；每个失败都有 `reason + vehicle_id + segment_index`；计算量和碰撞调用次数记录到日志。不得以 `convex_hull` 外包络替代逐车 OBB 作为唯一模型。

---

## 10. S3：ROS 多车消息、输入与发布契约

**文件**：新增 §5.2 所列 `.msg`、修改 `fwis_v3_msgs/CMakeLists.txt`（以及确有依赖变化时的 `package.xml`）、新建 `test_formation_contract.py`。

1. 先打开当前 `fwis_v3_msgs/CMakeLists.txt`，核对 `add_message_files(...)`、`generate_messages(...)`、`catkin_package(...)`；按照现有风格逐个加入新消息。
2. 确认 `FormationVehicleSpec` 中 `goal_heading_free` 语义：仅终端 `θ` 可忽略，车辆中心目标必须遵守 rigid offset。
3. 检查 ROS 生成的 Python/C++ 消息类型可以导入，并在请求消息中反序列化重建 `FormationModel`。
4. 检查 `plan_id` 与 `formation_id`、车辆 ID 不混淆；断言没有重复 ID、空车列表、路径长度不同、时间网格不单调、时间戳不一致。
5. 单车消息文件**一个字节也不改**（如果必须变更，另开任务，写迁移方案）。
6. 初期建立一个轻量 `formation_request_validator`（可放于新 `formation_optimizer_node.py`）只做验证并发布结构化失败，暂不启动规划搜索。

**构建命令**（有 ROS Noetic 环境才运行）：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C catkin_ws -j1
source catkin_ws/devel/setup.bash
python3 -c 'from fwis_v3_msgs.msg import FormationPlanRequest, FormationTimedTrajectory; print("ok")'
```

**[验收]**：原 `PlanSeed/TimedTrajectory` 仍正常导入；新消息可序列化/反序列化；缺少必填规范参数时以明确错误拒绝，不触发搜索。

---

## 11. S4：C++ 编队前端（先几何，再运动学）

**文件**：新增 `formation_model.hpp/formation_collision.hpp/formation_search.hpp/formation_search.cpp/formation_planner_node.cpp`，修改 `fwis_v3_planner/CMakeLists.txt`；新增单独的 `formation_demo.launch` 入口（不动单车 `demo.launch`）。

### 11.1 搜索状态（第一版）

```cpp
struct FormationSearchNode {
    FormationPose qf;           // x_F, y_F, psi_F
    int mode_hint;              // 仅候选几何类型提示，不是任何车的 A/O/S
    int gear_hint;
    int parent;
    double g;
    FormationPrimitive action;
    // TODO: S5 将 per-vehicle heading/mode/gear 绑定到可行候选
};
```

严格注明：仅 `q_F` 空间进行几何搜索，得到的是**未验证 4WIS 运动学的几何候选**；不可直接复用原单车的 `Action[]` 冒充多车可执行轨迹。

### 11.2 初版 primitive 及后续扩展

- `TRANSLATE(dx,dy)`：`ψ_F` 保持不变，整个队形平移。所有车的中心速度相同。车体航向暂不分配；S5 检验可用 O/A。
- `ROTATE(dψ, pivot)`：整个队形围绕选定参考中心运动，各车中心按 rigid transform 计算；只能标记为**队形几何旋转候选**，不等价于各车 S。
- `COMPOSITE(curve)`：初版禁用；S6 引入连续参数化后再扩展并逐车验证。

**重要**：如果 primitive 需要旋转中心 `pivot`，不能只靠 `amount/control` 两个模糊数值隐藏 pivot；可将 primitive 明确定义为 `start_qf/end_qf` 或增加 `pivot_x/pivot_y` 并同步更新消息。S3 的消息冻结必须在 S4 开始前根据实际第一版 primitive 设计定稿。

### 11.3 搜索伪代码

```text
function formation_search(request, map, config):
    validate_request_and_formation(request)
    if not FormationCollisionChecker.static_check(start):
        return fail("START_FORMATION_COLLISION")
    if not FormationCollisionChecker.static_check(goal):
        return fail("GOAL_FORMATION_COLLISION")
    open := priority_queue()
    best_cost := hash_table()
    push(start_qf, cost=0)
    while open not empty and within explicit budget:
        node := pop_min_f()
        if near_goal(node.qf):
            candidate := reconstruct(node)
            if full_formation_sweep_geometry_clear(candidate):
                save_as_geometry_seed(candidate)
        for primitive in generate_formation_primitives(node.qf):
            next_qf := rigid_integrate(node.qf, primitive)
            if not complete_formation_sweep_clear(node, primitive): continue
            if no_improved_discrete_key(next_qf, mode_hint, ...): continue
            push(next_qf)
    if geometry_seeds.empty(): return fail("FORMATION_NO_GEOMETRIC_PATH")
    publish_geometry_seed_with_status("GEOMETRY_ONLY_UNCERTIFIED")
```

### 11.4 状态离散化、启发与成本

- 状态键：`round(x_F/xy_res), round(y_F/xy_res), round(ψ_F/yaw_res)`；实际代码沿用单车 `Key` 思路，但不要直接使用单车 `Key` 中的 gear/控制编号作为编队模式真值。
- 对长角度路径保留角度展开或旋转方向历史，防止 `ψ` wrap 后与“绕了一圈”的候选错误合并。
- 初期启发：`h = hypot(goal_xy−current_xy)` 是几何引导，不可将其视为考虑队形旋转/模式切换的完备时间下界；若用 A* 最优性声明须证明 admissibility，未证明则只称启发搜索。
- 代价建议：`w_trans * length + w_rot * |Δψ| + w_clearance * risk + w_switch * mode_event_estimate`；禁止直接继承单车 `minimum_time` 评分声称联合时间已最优。
- 搜索节点预算、候选数预算、总墙钟预算均从独立 `formation_params.yaml` 配置；不要让多车默认篡改单车全局参数。

**S4 局限**：几何搜索可能找到没有任何可行 A/O/S 模式的路径；S5 必须筛除。**[验收]**：空地直线、障碍物绕行、终点不可行、旋转扫掠能区分成功/失败；只有 `GEOMETRY_ONLY` 结果，没有可执行时间轨迹。

---

## 12. S5：每车 A/O/S 模式分配与局部航向可行性

**文件**：新增 `formation_kinematics.py`、`test_formation_kinematics.py`；S4 新前端通过可行性反馈重新选候选；可视性能将候选 mode assignment 放在 Python 后端，避免 C++ 直接复制一整套动力学库。

### 12.1 核心数据

```python
@dataclass(frozen=True)
class VehicleModeSegment:
    vehicle_id: int
    start_station: float
    end_station: float
    mode: int                # 0 A, 1 O, 2 S
    direction: int           # +1/-1 或 Spin 的旋转方向
    theta_start: float
    theta_end: float
    # 实际控制参数可用连续表示，不强制 constant-control
```

**需要明确路径参数 `s` 的单位**：如果是编队参考平移弧长，纯旋转区间长度为 0，不能用它作严格递增参数；应使用**混合几何进度 `λ`**（对平移取弧长、对纯旋转取旋转角的有界标度）并对每个 block 分开定义 local parameter，再组成全局阶段顺序。不要在 `s=0` 的纯旋转阶段对 `dp_i/ds` 做除零运算。

### 12.2 候选模式判定算法（建议）

```text
for each formation block B:
  compute q_F(λ), d q_F/dλ
  for each vehicle i:
    derive p_i(λ) and d p_i/dλ
    enumerate admissible mode A/O/S and heading boundary candidates
    A feasible iff trajectory tangent aligns with body heading (forward/reverse)
        and |dtheta/darc_i| <= 1/Rmin,
        plus actual transition/steering constraints
    O feasible iff body heading stays constant on the O segment,
        phi(λ)=atan2(p'_iy,p'_ix)-theta_i within allowed signed-angle convention
    S feasible iff center displacement over entire block is identically zero
        and body heading changes consistently with allowed spin direction
  intersect vehicle-specific valid mode choices and global stop/switch policy
  if none: reject candidate with per-vehicle reason
  else retain up to K best joint schedules under explicit mode-cost ranking
```

**关键实现细节**：

- `atan2` 切向方向与 signed `v_i` 的反向约定要按当前 `core.advance()` / `quality.py` 的 `a.amount<0` 检查保持一致；不能用一个无符号方向角把倒车伪装成超大 O 角。
- 对 A：需考虑反向倒车使 `κ` 与 `θ_dot/v` 的符号一致；弧长为零时不计算除零。
- 对 O：使用展开 `φ` 和明确的合法范围；不应在同一 O 段产生非零 `theta_dot`。
- 对 S：仅中心确实不动时允许；需要单独检查 `θ_i` 的转角和角速度/加速度约束。
- 对模式切换：首版保守要求**整个编队参考平移和所有车体旋转速度都降为零**后再重新配置，记录 mode switch dwell；不得允许某车尚未转向完成时整编队起步。
- 允许某车在同一几何区间不动而其他车辆运动的情况，但其模式应符合该车中心零速和可实现的车体航向，不一定必须是 S。

**[验收]**：每个最终保存的模式结构都能被独立运动学审核函数从几何和体航向重算验证；`ROTATE` 中非旋转中心车辆分配 S 必须被拒绝；所有车辆速度方向、模式、航向边界连续性一致。

---

## 13. S6：Python 编队连续几何 + 候选管理

**文件**：新增 `formation_geometry.py`、`formation_master.py`、`test_formation_geometry.py`；现有 `continuous_geometry.py` 仅复用安全接口，不直接改其单车 `ContinuousPlan` 结构。

### 13.1 状态表示

```python
@dataclass(frozen=True)
class FormationBlock:
    geometry_type: str             # translation / rotation / composite
    parameter_length: float        # 本段局部 λ 的范围
    formation_controls: tuple      # 结构化曲线控制节点
    vehicle_mode_segments: tuple   # 每车合法 mode 结构

@dataclass(frozen=True)
class FormationContinuousPlan:
    plan_id: int
    formation_id: str
    start: FormationPose
    goal: FormationPose
    offsets: tuple
    vehicle_start_headings: tuple
    vehicle_goal_headings: tuple
    blocks: tuple
```

若某段 `q_F(λ)` 允许连续改变 `ψ_F`，则每车 `p_i(λ)` **每次都从 rigid formula 派生**，不得作为另一条独立可优化自由曲线存储。`θ_i(λ)` 是另外的连续函数，必须受模式约束。

### 13.2 候选构造与局部修复

1. 接收 S4 `FormationPlanSeed`，检查 `plan_id`、地图版本、起终几何和动作类型。
2. 将分段 geometric primitive 变成连续 `FormationBlock`，计算 `q_F(λ)`；保证端点连续；初期允许仅分段常值参考平移/旋转。
3. 对每段生成 `p_i(λ)` 和所需速度切向，调用 S5 模式赋值。
4. 终端拟合只修改 `q_F(λ)` 和允许的 `θ_i` 连续参数，不可独立移动车中心。
5. 局部优化前做多车扫掠初筛；失败时保留失败车辆、块 index、障碍物位置等上下文。
6. 候选去重 key 至少包含：编队几何块、每车模式序列、每车起终航向、offset 版本、动态 profile；不可仅复用现有单车 `Action` signature。
7. 参照 `ModePathMaster` 的预算、incumbent 保留和“完全审核后选解”；构建 `FormationModeMaster`；初期最多固定 K 候选，记录全部拒绝原因。
8. 输出 `FormationContinuousPlan` 只能为 `GEOMETRY_VALID` / `KINEMATIC_VALID`；等待 S8–S9 完成后才升级为 `FEASIBLE`。

**[验收]**：各车辆轨迹从 `q_F` 精确推导、每个姿态满足相对位置不变量；纯旋转、斜移、逆向段没有数值崩溃；无任何改动导致单车 `ContinuousPlan` 单测退化。

---

## 14. S7：固定模式结构的联合连续优化

**文件**：新增 `formation_optimizer.py` 和单独测试；先检查现有 `continuous_optimizer.py` 的 CasADi/IPOPT 导入、优化变量定义与失败处理，复用局部数值辅助而不是原样照搬。

### 14.1 决策变量与约束优先级

可优化变量（每次结构固定）：

- 编队曲线的控制节点 `x_F(λ), y_F(λ), ψ_F(λ)`（或低维参数化的等效控制）；
- 每车独立航向节点 `θ_i(λ)`；
- 每车 A 曲率 `κ_i(λ)`、O 斜角 `φ_i(λ)`；
- 段长度/控制节点位置的安全可优化参数（实际是否放开需确保 mode 结构固定）；
- **时间变量不在本阶段必需**，S8 单独处理公共 retiming。

**必须为硬约束/消元关系**：

1. `p_i(λ)=p_F(λ)+R(ψ_F(λ))r_i` 由构造消元，非 penalty。
2. 分段 A/O/S 微分等式（依第 2 节）。
3. 各车 `|κ_i|<=1/Rmin`、O `|φ_i|` 合法、S `dp_i/dλ=0`；各车模式/航向边界连续性。
4. 起终 `q_F` 与每车 `θ_i` 的硬约束（允许 heading_free 时按约定释放）。
5. 障碍物与车间距离在 NLP 中可采用可微 surrogate 提供引导，但**不能用 soft surrogate 作为最终安全证书**，S9 必须独立扫掠。

软目标可选：路径/旋转总量、参考净空、控制变化、模式切换次数（模式固定时该项是常数）、预计联合时间代理。

### 14.2 优化伪代码与求解失败策略

```text
function optimize_joint(seed_plan):
    assert all geometry/mode contracts validated
    X := pack_formation_and_heading_controls(seed_plan)
    build_NLP(fixed_mode_structure, rigid_centers_derived_from_formation)
    add_hard_kinematic_and_terminal_constraints()
    add_soft_clearance_and_smoothness_objectives()
    solve_with_budget()
    if solver failed or hard residual > tolerance:
        return REJECTED_SOLUTION_WITH_DIAGNOSTIC
    reconstruct_continuous_plan()
    run independent mode + collision + endpoint audit
    if fail: reject (never emit solver candidate as final)
    return geometry_candidate_for_retiming
```

若固定结构 NLP infeasible，不能降低车间距硬约束；可以保留原可行 seed（需重新审核）或返回 S5/S4 改模式/路线。**求解器的 `success=True` 不等于真实可行**，须独立逐项验证。

**[验收]**：硬约束残差数值报告、各车曲率/斜角合法，几何独立审核通过；记录 NLP 耗时和可用/失败数量；单车版本仍运行。

---

## 15. S8：全车同步时间参数化

**文件**：新增 `formation_retiming.py`、`test_formation_retiming.py`，借鉴现有 `continuous_retiming.py` 的 quintic profile 和 `mode_switch_timing.py` 事件逻辑。

### 15.1 一条公共时钟而非 N 条独立时钟

每个编队 block 有局部参数 `λ∈[0,L]`。**所有车辆共享一个**单调进度 `λ(t)`，不同车辆的几何轨迹由 `p_i(λ)` 给出；同时保留 `θ_i(λ)`：

\[
\dot p_i = p_i'(\lambda)\dot\lambda,\quad
\ddot p_i=p_i''(\lambda)\dot\lambda^2+p_i'(\lambda)\ddot\lambda.
\]

\[
\dot\theta_i=\theta_i'(\lambda)\dot\lambda,\quad
\ddot\theta_i=\theta_i''(\lambda)\dot\lambda^2+\theta_i'(\lambda)\ddot\lambda.
\]

对于有单独 pure Spin 的车体航向事件，其该车中心可不动，但整个编队可能需要停住；**进度停滞时可另引入独立的“静止动作事件”**（具有明确持续时间和每车航向插值），不能通过对 `p_i'` 除零得到速度。

设 `γ_i(λ)=||p_i'(λ)||`，则线速度大小：

\[
|v_i(t)|=\gamma_i(\lambda)|\dot\lambda|,
\quad |\dot\lambda|\le\min_{i:\gamma_i>0}\frac{v_{i,\max}}{\gamma_i}.
\]

这只给出**速度必要约束**；转角、角速度、加速度、jerk、轮速/转向变化需要用上面的全微分和轮系映射逐车、逐段检查。别忘记车辆 A 倒车的 signed speed 和 O 的方向约定。

### 15.2 推荐实现顺序

1. 从连续几何分段取 `q_F(λ), θ_i(λ)`，求导得到各车速度、角速度放大比。
2. 计算所有车辆共有的允许进度速度包络；若某个约束导致非法/不存在公共进度，返回 `FORMATION_RETIMING_INFEASIBLE`。
3. 首版采用同单车近似的分段五次进度规律、起终速度为零、模式切换全体停住；对于某车必须转轮但无法在移动段满足换模规则，插入整编队静止事件。
4. 动态限制通过全车最大值迭代延长阶段时长，绝不能只使用编队参考点的速度限制。
5. 定义单一 `times_from_start[k]`，为全部车辆在相同 `t_k` 采样 `center_pose, θ_i, v_i, yaw_rate_i, mode_i`。
6. 必须审核每车持续的 A/O/S 模式、驻留时间、方向换向和无时间倒退。
7. 生成 `FormationTimedTrajectory` 前调用 S9 独立 evaluator；第一轮若审核未完成，仍只能输出 `GEOMETRY_ONLY`。

**[验收]**：同一时刻所有 `p_i(t)` 满足刚性变换；各车速度/加速度/jerk/Spin angular dynamics 和模式驻留符合选定 profile；失败正确报告导致最紧约束的车辆和时段。

---

## 16. S9：独立 FormationEvaluator、选解和发布

**文件**：新增 `formation_quality.py`、`test_formation_quality.py`、`formation_optimizer_node.py`，新增计划发布/质量报告逻辑；单车 `quality.py` 的 `TrajectoryEvaluator` 继续作为单车独立审核器和交叉对照基线。

### 16.1 最终独立审核必须完全重算

审核器输入 `map + request + FormationContinuousPlan + timed`。不要信任前端提供的 `clear=True`、NLP 内部软距离约束或 retiming 内部峰值报告。

依次检查：

1. **请求合同**：唯一 `plan_id`、车辆数/ID、偏移一致、地图签名、`frame_id`、时间戳、数值有效。
2. **轨迹合同**：`times_from_start` 严格递增、全车同长度同时间、起终正确、`θ_i` 展开与模式一致。
3. **刚性几何**：每个位置来自共同 `q_F(t)`；对 `p_i−p_j−R(ψ)(r_i−r_j)` 检查最大绝对残差；对连续 block 也按充分加密采样或区间界检测。
4. **独立碰撞**：每车 OBB vs 地图与地图外边界，每两车 OBB；对相邻状态扫掠，并计算**最小障碍物净空**和**最小车间净空**。
5. **运动学**：A 的车体非侧滑关系、曲率，O 的车体航向恒定与斜角，S 的车中心固定；起终方向和倒车策略。
6. **时间/动态**：全车速度、加速度、jerk、yaw、Spin、模式切换驻留；严格审查整个时间曲线，不仅节点采样最大值。
7. **显示/发布采样**：按照实际用于 ROS 发布、RViz 插值的时间点重新扫掠；不能以稀疏显示点代替安全检查。
8. **状态**：只有所有子项通过才 `FEASIBLE`；中途任何一步失败均 `INVALID/REJECTED`；几何可行、时间不可行必须 `GEOMETRY_ONLY_NOT_EXECUTABLE`。

### 16.2 失败码（建议统一字典）

| 失败码 | 触发条件 | 诊断字段 |
|---|---|---|
| `INVALID_REQUEST` | ID/offset/时间戳/车辆规格非法 | `field`, `vehicle_id` |
| `START_FORMATION_COLLISION` | 起点撞墙/车间重叠 | `vehicle_id`, `pair_id` |
| `GOAL_FORMATION_COLLISION` | 目标撞墙/车间重叠 | `vehicle_id`, `pair_id` |
| `FORMATION_NO_GEOMETRIC_PATH` | 预算内未找整队几何通道 | `expanded`, `budget` |
| `FORMATION_SWEEP_COLLISION` | 运动中碰撞 | `segment`, `station`, `vehicle_id` |
| `KINEMATIC_INFEASIBLE` | A/O/S 模型矛盾 | `segment`, `mode`, `vehicle_id`, `residual` |
| `MODE_SCHEDULE_INFEASIBLE` | 无合法同步换模/航向序列 | `segment`, `vehicle_id`, `reason` |
| `FORMATION_NLP_INFEASIBLE` | 联合连续优化失败或残差超限 | `solver_status`, `residual` |
| `FORMATION_RETIMING_INFEASIBLE` | 无合法统一时间曲线 | `constraint`, `vehicle_id`, `segment` |
| `FORMATION_RIGIDITY_VIOLATION` | 位置不符合固定 offset | `max_error`, `pair_id` |
| `PUBLICATION_SWEEP_INVALID` | 实际采样/回放插值存在风险 | `time_interval`, `vehicle_id` |

**发布事务性**：同一个 `plan_id` 的 `FormationTimedTrajectory` 只能在 `FEASIBLE` 后发布；新请求开始后要撤销上次有效显示/发布状态（或明确状态版本控制），失败时不保留旧 `FEASIBLE` 假象。

### 16.3 选解目标

第一版优先以**完整审核通过**为首要筛选；在全部可行候选中比较共同总执行时间 `T_end`，在时间接近时比较净空、模式切换数、平滑性。不要使用一个车辆的时间代替整个编队完成时间。记录 `candidate_id / feasibility_status / duration / rejection_reason`。

**[验收]**：独立构造的非法轨迹（尤其“中间碰撞、间距漂移、S 车中心移动、各车时间轴不一致”）全部被拒绝；只有 `FEASIBLE` 输出完整多车轨迹；原单车验收仍通过。

---

## 17. S10：RViz、端到端实验与回归

**文件**：新增 `formation_playback.py`、`formation_demo.launch`、`ros_formation_smoke.py`、`run_formation_regression.py`、`formation_eval_cases.yaml`。修改 RViz 配置时新建 `formation_demo.rviz`，避免覆盖单车显示。

1. 一次只运行一个新的编队 planner、一个新的编队 optimizer；/map 可复用单一静态地图。不能启动三份单车 planner 代替编队规划器。
2. 每辆车的 Marker 命名空间必须包含 `formation_id` 和 `vehicle_id`（例如 `formation/wedge_3/vehicle_1`），ID 唯一；发布 TF 时也必须唯一 frame，避免多个 `base_link` 冲突。
3. 显示：编队参考点、每车车体 OBB、车体航向、A/O/S 模式色带、历史路径、车间连线、约束指标、失败轨迹（必须标 `GEOMETRY_ONLY`）。
4. 从一个 `FormationTimedTrajectory` 的共同 `header.stamp` 与相同 `time_from_start` 插值所有车，禁止每车各自从收到消息的当前时刻启动。
5. `ros_formation_smoke.py` 检查消息、实际回放采样、空结果清理、全车 Marker、终端误差和独立审核报告，而不仅是屏幕“看着对”。
6. 对固定地图/种子/预算进行至少三次重复测试，写入 CPU、版本、耗时分位数、成功率和拒绝原因；不能把失败场景删除以美化成功率。

### 17.1 实验矩阵（每行至少一个固定 YAML case）

| 编号 | 环境 | 期望结论 | 重点检查 |
|---|---|---|---|
| E01 | 空地图，N=1 直线 | 成功 | 与单车几何结果兼容，模式约束 |
| E02 | 空地图，N=3 刚性平移 | 成功 | 两两距离恒定、共同时间轴 |
| E03 | 空地图，N=3 全队旋转 | 条件可行 | 非中心车辆不可全用 S；模式组合 |
| E04 | 宽弯道 | 条件可行 | 每车曲率/斜移角/航向 |
| E05 | 小于编队最小投影宽度的窄门 | 必须失败 | 不得变形队形 |
| E06 | 可容纳但仅特定方向能过的窄门 | 条件可行 | 编队旋转与扫掠 |
| E07 | 90° 急弯/折线走廊 | 条件可行 | 候选多样性/换模 |
| E08 | S 型通道 | 条件可行 | 非同构车速、转向连续 |
| E09 | 两车中心不撞但车体 OBB 重叠 | 必须失败 | 两车碰撞审核 |
| E10 | 端点安全、中间扫掠撞障碍 | 必须失败 | 连续扫掠 |
| E11 | 起点不符合固定 offsets | 必须失败 | 请求校验 |
| E12 | 目标队形镜像/车辆 ID 错误 | 必须失败 | 刚体拟合与 ID |
| E13 | 某车 A 曲率超限 | 必须拒绝该候选 | 运动学检查 |
| E14 | 某车 O 段航向变化 | 必须拒绝该候选 | 运动学检查 |
| E15 | 某车 S 段中心移动 | 必须拒绝该候选 | 运动学检查 |
| E16 | 某车提前结束或时间戳不对齐 | 必须失败 | 同步时间合同 |
| E17 | 换模未静止/驻留不足 | 必须失败 | mode-switch 合同 |
| E18 | 含 NaN、空列表或旧 plan_id | 必须失败并清旧图 | 错误处理 |
| E19 | 相同地图 N=1/2/3 | 统计不要求全成功 | 规划复杂度和内存 |
| E20 | 不同队形尺寸与障碍密度 | 统计不要求全成功 | 成功率/耗时/净空 |

注：`条件可行` 只有在**具体几何尺寸和逐车运动学允许**时才应成功；若物理上不可行，正确结果是明确失败，而不是必须通过。实验地图必须在 case 配置中固定，不能凭描述自动推测。

### 17.2 指标及计算方法

- `success_rate = FEASIBLE_count / valid_request_count`；统计分母和失败类别必须公开。
- `plan_wall_time_s`：从收到完整有效请求到报告审核结果的墙钟耗时；单独拆前端、模式分配、NLP、retiming、audit。
- `T_end`：全部车辆共同的最终时间（s）。
- `rigidity_error_max_m = max_{t,i<j} | ||p_i(t)-p_j(t)||-||r_i-r_j|| |`；还需输出 `max ||p_i-p_F-Rr_i||`，防止仅有距离正确但整体刚体关系错误。
- `minimum_obstacle_clearance_m`：所有车对地图的最小 OBB 净空。
- `minimum_intervehicle_clearance_m`：所有车对的最小车体净空。
- `peak_A_curvature`, `peak_O_phi`, `mode_switch_count`：分别对每辆车统计，并记录全队汇总。
- `dynamic_peak_ratio`：实际速度/加速度/jerk/角速度等峰值与限制之比；应小于等于 1（按审计容差）。
- `memory_peak_MB`, `expanded_nodes`, `candidate_count`, `n_kinematic_reject`, `n_audit_reject`。

### 17.3 对照实验

1. 单车原始规划（N=1）：确认新体系单车边界兼容。
2. 单一路径偏移生成 N 车（旧控制实验思路）：**仅作无碰撞/运动学保证的对照**，不得与完整可执行轨迹成功率直接混比而不注明认证级别。
3. 编队几何搜索（S4）与完整多模式规划（S9）：量化几何-only 候选中被运动学/时间审核剔除的比例。
4. 多车规划禁用 Spin 或禁用 O 的消融：观察可行率、总时间、模式切换与规划耗时差异。

---

## 18. 兼容性、回归命令和提交规范

### 18.1 不可破坏的单车接口

`/plan/seed`、`/plan/seed_candidates`、`/plan/path`、`/plan/timed_trajectory`、`/plan/quality_report`、单车 `TimedPoint/TimedTrajectory`、单车 `demo.launch`、`run_demo.sh` 当前主后端 `mode_path_v2` 选择必须继续有效。新多车数据默认放在 `/formation/*` 命名空间。

### 18.2 每个阶段必执行的检查命令

```bash
# 1. 核实改了哪些文件
git status --short
git diff --stat
git diff --check

# 2. 单车回归，必须保存完整日志
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'

# 3. 仅运行本阶段相关测试（例：S1）
python3 -m unittest discover -s tests -p 'test_formation_model.py' -v

# 4. ROS 构建（有环境才运行）
source /opt/ros/noetic/setup.bash
catkin_make -C catkin_ws -j1

# 5. 提交前证据：不要声称未运行项“已通过”
git diff --check
```

若项目测试依赖特殊 `PYTHONPATH`、ROS 环境或需要跳过不相关昂贵用例，应先核实 README 和现有测试配置，记录实际完整命令与差异，而不是把测试脚本改到“永远通过”。

**Git 提交要求**：每阶段提交只包含对应模块、相关测试和最小配置；提交信息例如 `feat(formation): add rigid geometry model and tests`。禁止一次性格式化几千行原单车源码、禁止删除历史 `archive/`、禁止覆盖现有用户修改、禁止改 `catkin_ws/build` / `catkin_ws/devel`。

### 18.3 每阶段必须提供的审查报告模板

```text
任务编号：Sx
基线 SHA：...
本次新 SHA：...
已查看源码：路径 + 关键函数签名
修改文件：逐个路径 + 原因
接口变化：新增/不兼容之处
执行测试：逐条真实命令 + 退出码 + 关键日志
未执行测试：列出原因（不得写通过）
安全性审计：刚性/OBB/模式/时间检查是否覆盖
存在风险：具体触发条件
完成判据：通过 / 未通过
下阶段前置条件：...
```

---

## 19. 风险登记与止损规则

| 风险 | 触发方式 | 避免与处理 |
|---|---|---|
| 方向/车体航向与编队方向混淆 | 将 `θ_i` 自动设为 `ψ_F` | 所有结构分别存储，专门测试反例 |
| 车辆间距 soft-only | 加大 cost 权重但仍有残差 | 用硬构造/硬等式，独立审计 |
| 各车时间独立 | “先跑完的一车停车等”导致相对位置漂移 | 统一 `λ(t)` 与全车阶段事件 |
| 所有车同时 Spin | 旋转时非中心车辆中心运动 | S5 模式判定必须拒绝 |
| Python/C++ 几何不一致 | 前端说 clear，后端说撞墙 | 统一坐标/膨胀合同 + golden 对照 |
| 编队外接框过度保守 | 明明可通过的旋转窄门失败 | 以各车真实 OBB 并集为核心 |
| 车间 OBB 只检查离散节点 | 旋转过程中发生擦碰 | 自适应扫掠和独立采样认证 |
| NLP 将 soft clearance 当安全证明 | 数值优化成功但地图碰撞 | 候选逐项独立审核后才能发布 |
| Spin 与 O/A 的静止换模缺失 | 执行器未完成转向就移动 | 初版全队停车换模，并记 dwell |
| 旧消息污染 | 旧 plan_id 覆盖新结果 | 事务性发布与版本检查 |
| 大队形在给定地图物理不可通行 | 旧楔形尺寸大于门宽 | 预先判几何不可行，换地图验证算法 |
| 急于优化性能导致质量退化 | 取消扫掠/放宽终点 | 改缓存/候选排序；不放宽质量合同 |

**止损规则**：某阶段有未解决的**安全/运动学/时间**失败，不能跳过并进入“成功发布”；若性能不可接受，先记录瓶颈和可行/不可行样本，再做局部性能改进，不能删审核。

---

## 20. 交付给代码 AI 的总任务提示词（可直接复制）

```text
你负责在 CopTrans_base / continuous-mode-path 基础上执行多车 4WIS 刚性编队规划迭代。
请严格遵守《单车 4WIS 到多车刚性编队 4WIS 协同规划：详细技术实施文档》：
1) 当前只能执行我指定的一个阶段（例如 S1），禁止提前实现下一阶段，禁止改旧控制器或 NMPC。
2) 先检查 git status、真实分支、SHA，再读取本阶段涉及的真实源码、msg、YAML、CMake。函数与文档不一致时，列出差异并改写局部任务安排，不能编造接口。
3) 优先新建多车类/新节点/新消息；不删除、复制或重写大段单车核心；不得损坏单车回归。
4) 固定相对几何关系是硬约束；每车车体航向与编队参考方位分开；S 仅允许车辆中心不动；全车统一时间基准。
5) 写本阶段单测和反例测试；实测运行相关命令、git diff --check、旧单车回归。有 ROS 时编译，有障碍时如实说明环境缺失，不要捏造成功。
6) 只提交完成本阶段的改动。最后给出：改动文件、关键数据合同、实际测试命令和退出码、失败原因、Git SHA、下阶段前置条件。
如果某要求无法安全实现，不要将不可行候选误标为可执行；返回结构化失败和阻碍说明。
现在执行阶段：{S0/S1/.../S10}
```

### 20.1 分阶段执行提示词（一次选一条）

**S0**：`只执行 S0。核实当前 SHA，冻结单车测试和样例，产出环境与运行证据。不得改规划算法。`  
**S1**：`只执行 S1。新建 formation.py 与测试，包含固定 offset、N 车刚体变换、唯一 ID、起终队形校验。实现后跑相应单测及旧回归。`  
**S2**：`只执行 S2。新建 formation_collision.py；复用现有 OBB 并增加任意两车 OBB、连续扫掠、旋转中途碰撞反例。`  
**S3**：`只执行 S3。先定稿并新增 Formation*.msg，严格保持原消息兼容，修 CMake、编译并做 ROS 序列化/时间网格测试。`  
**S4**：`只执行 S4。新建 C++ 编队几何搜索，不复制单车 planner；只发布 GEOMETRY_ONLY 编队候选，不发布 FEASIBLE 时间轨迹。`  
**S5**：`只执行 S5。新建 formation_kinematics.py；逐车验证 A/O/S、反向语义、车体航向、全队停车换模，构造转动期间非中心车辆错误 Spin 反例。`  
**S6**：`只执行 S6。新建 FormationContinuousPlan、FormationModeMaster；各车中心始终从同一编队曲线导出，测试端点、模式与刚性误差。`  
**S7**：`只执行 S7。固定模式结构内联合连续优化，编队相对位置消元/硬约束，逐车运动学硬约束，独立复核求解器结果。`  
**S8**：`只执行 S8。实现单一共同进度 λ(t)、全车速度/加速度/jerk 限制、静止换模事件与共享时间网格。`  
**S9**：`只执行 S9。独立 FormationEvaluator 审核并事务性发布，只有 FEASIBLE 才发可执行多车时间轨迹；覆盖所有负例。`  
**S10**：`只执行 S10。多车回放/RViz、20 个固定实验用例、性能回归与消融，不更改已冻结的安全阈值。`

---

## 21. 用户/导师必须确认的设计选择

1. **“间距相同”具体指固定 offset/固定 pairwise 距离，还是三车所有两两距离数值完全一致？**（本文软件兼容两者，但示例默认固定 offset。）
2. 初始目标队形是**两车、三车楔形、三车横排、三车纵列或其他**？车间允许的真实几何净空是多少？
3. 论文的“刚性”只保证车**中心位置关系**，还是每辆车车体航向相对编队也必须固定？后者会显著影响 A/O/S 可行性。
4. 所有车辆是否都是相同 Ranger Mini 2.0？是否允许各车 mode 不同、个别车静止而其他车运动？
5. 编队参考点取领航车中心、车队几何质心、货物中心，还是任意虚拟点？本文用可配置虚拟点，不假定存在真实货物刚性连接。
6. 是否要求终点每辆车车体航向达到指定角度？若不要求，终端消息须明确 `heading_free`。
7. 是否允许后续研究**非零速度下换模**？第一可用版本推荐全队静止换模；真实平台切换时间及方向限制需现场验证。
8. 是否有必须使用的固定地图/工作目录和实验硬件资源？在确认前不得宣称某地图对旧楔形物理可通行。

**未确认时的安全默认**：任意固定 offset、独立车体航向、仅静态障碍、全队停车换模、N≤3 为首个交付范围、失败即拒绝可执行发布；这些是工程初版假设，不是论文最终定论。

---

## 22. 最终验收清单（必须逐项打钩）

- [ ] 已冻结当前单车 SHA、单测、ROS 用例及基线记录。
- [ ] 配置 `N` 辆车及固定 offsets；`formation_id/plan_id/vehicle_id` 语义清晰。
- [ ] 所有时刻以共同 `q_F` 生成中心；独立 `θ_i`；不靠 formation-cost 维持刚性。
- [ ] 每车 OBB 对障碍物/边界和每对车体 OBB 静态、连续扫掠全部通过。
- [ ] 起终姿态/队形、镜像与 ID 交换有检测。
- [ ] 搜索区分编队几何候选与 4WIS 可执行轨迹。
- [ ] 每车 A/O/S 运动学、倒车、曲率、斜移角、Spin 中心不动、换模驻留可审计。
- [ ] 连续优化不改变相对距离，不丢失模式/航向连续性。
- [ ] 全车只有一个共同时间轴；所有动态限制按每车实际速度/角速度审核。
- [ ] `FormationEvaluator` 不信任前端/NLP，独立重算且拒绝全部负例。
- [ ] 只有完整审核通过才发布 `FormationTimedTrajectory(FEASIBLE)`。
- [ ] 旧单车 `demo.launch`、`run_demo.sh` 和旧消息不受破坏。
- [ ] 多车 Marker/TF/ROS 话题互不覆盖。
- [ ] 实验覆盖直线、弯道、窄门、旋转、S 型、非法请求与不同 N 的性能压力。
- [ ] 规划耗时、运行总时间、成功率、最小净空、模式切换和最大刚性误差可复现。
- [ ] 每阶段独立提交、真实测试日志、未验证项明确标记。

---

## 23. 源码核查路径索引

- C++ 主规划前端：`catkin_ws/src/fwis_v3_planner/src/planner_node.cpp`。关键点：`PlannerNode::on_goal`、`pose_clear`、`action_clear`、`sparse_mixed_route`、`oblique_route`、`search`。检索时以当前实际签名为准。
- 单车消息：`catkin_ws/src/fwis_v3_msgs/msg/{ModePrimitive,PlanSeed,PlanSeedCandidates,TimedPoint,TimedTrajectory}.msg`。
- Python 后端入口：`catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py` 的 `OptimizerNode.on_seed()`、`run_mode_path()`。
- 单车基础几何：`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py`。
- 单车候选管理：`.../fwis_v3_optimizer/mode_master.py` 的 `ModePathMaster.solve()`。
- 单车连续几何：`.../fwis_v3_optimizer/continuous_geometry.py` 的 `ContinuousPlan`、`MotionBlock`、`sample_block` 等。
- 单车局部优化：`.../fwis_v3_optimizer/continuous_optimizer.py`。
- 单车时间缩放：`.../fwis_v3_optimizer/continuous_retiming.py` 和 `mode_switch_timing.py`。
- 独立安全审核：`.../fwis_v3_optimizer/quality.py` 的 `TrajectoryEvaluator.evaluate_continuous_geometry()`、`FootprintDistance`。
- 公共车辆参数：`catkin_ws/src/fwis_common/config/ranger_mini_2.yaml`。
- 集成配置：`catkin_ws/src/fwis_v3_demo/{launch/demo.launch,config/params.yaml}`。
- 回放：`catkin_ws/src/fwis_v3_playback/scripts/playback.py`。
- 现有回归：`tests/test_core.py`、`tests/test_quality.py`、`tests/test_continuous_geometry.py`、`tests/test_continuous_retiming.py`、`tests/test_segment_timing_cache.py`、`tests/ros_spin_smoke.py`、`tests/ros_oblique_smoke.py` 等。

GitHub 原始分支：<https://github.com/WangxQiang21/CopTrans_base/tree/continuous-mode-path>；当前交接报告：`CURRENT_REPOSITORY_STATUS.md`；旧控制实验：`多车协同控制算法框架详解.md`。

**最终执行顺序：S0→S1→S2→S3→S4→S5→S6→S7→S8→S9→S10。不得跳过 S5 的每车模式可行性，也不得跳过 S9 的独立审核。**
