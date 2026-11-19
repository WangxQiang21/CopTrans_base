# TASK 01｜研究默认入口、RViz 阶段可视化与连续规划正确性修复

**文档类型**：用户与 ChatGPT 确认的技术方案／交付 Codex 的需求输入（不是 Claude 执行文档）  
**任务状态**：方案讨论通过，待 Codex 核对工作区并编写详细执行计划  
**适用项目**：狭窄通行环境下三辆 4WIS 车的刚性编队连续规划与控制  
**代码仓库**：<https://github.com/WangxQiang21/CopTrans_base>  
**目标分支**：`feature/virtual-rigid-formation-a`  就是当前仓库
**本次远端核查提交**：`77805cd46849ebadc374187f9b00fce0f7151f84`（2026-11-18）  
**用户本地项目根目录**：`/home/wq/whut_thesis/expv3_multi_virtual_a`  
**本地文档归档目录**：`/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow`  

> **交付指令**：请将本文件由用户手动保存为 `work_flow/TASK01_技术方案.md`。Codex 必须以用户本地真实工作区的最新 HEAD 和实际文件为准复查，不得把上面的远端 SHA 当作当前工作区一定一致。本文件已包含必要背景和完整要求，Codex 不需要知道其他聊天中的 D01、D02 等内部编号。**Codex 第一轮只写详细执行方案，不直接改源码；Claude 依据经用户确认的执行方案开发；Claude 提交开发交付报告；Codex 做独立审核并写审核报告；用户通过本机 RViz 再验收。**

---

## 0. 本次只做 TASK 01，不越界

### 0.1 总体目标

现有三车 `virtual_rigid_a` 后端以虚拟参考点 F 生成连续阿克曼路径。任务 1 的目标不是改造多车集群优化模型，而是：

1. **先把可观察性建好**：通过默认 `run_demo.sh` 直接在 RViz 看见 A* 原始路线、当前二维凸安全走廊、Portal 几何 Guide、优化出的 F 参考点连续曲线、从真实候选路径推导的三车中心轨迹及车体 OBB，且右上角 HUD 显示当前计算阶段与失败原因；失败时保留已有诊断。
2. **真正启用当前研究算法**：无显式参数时，用户运行仓库根目录 `./run_demo.sh` 的编队流程就使用 `backend=virtual_rigid_a`、`virtual_a_guidance_mode=corridor`、`corridor_enforcement=hard`、手动 RViz 选起终点；不能悄悄回退 `uniform_v02` / `legacy_route`。
3. **消除三个已确认的错误拒绝源**：候选绕行方向与终点航向 winding 分支不一致、绕障路径长度 S 的搜索上界过窄、长路径生产积分的浮点保护项随 Simpson 面板数无意义地膨胀。
4. **每一部分都真实可测**：默认脚本应在本机真正执行 ROS 规划；测试覆盖成功与失败，展示实际效果；既有几何、动力学和原地图三车 OBB 连续碰撞终审及原子发布条件不能削弱。

**范围边界**：此任务中 `corridor` **仍然是 F 点二维凸走廊及其半空间硬约束**，三车旋转 OBB 进入 NLP、相邻区域并集的完整覆盖/Portal 配置通行、三车连续集群走廊证书、边界软代价归 TASK 02 / TASK 03。不能在 TASK 01 的报告中宣称已实现这些能力。当前仅使用车体几何；未明确建模的货物 footprint 不得冒充已完成碰撞认证。维持单车 A/O/S、FR06、`uniform_v02` 和显式 `legacy` 对照入口。

### 0.2 开发流程及工作目录（强制）

**本地只使用一个 `work_flow` 目录归档文档和每项任务证据，不要求另建复杂管理系统：**

```
/home/wq/whut_thesis/expv3_multi_virtual_a/
├── run_demo.sh                       # 用户每次测试使用的默认脚本，Claude 直接改仓库内此文件
└── work_flow/
    ├── TASK01_技术方案.md             # 本文件：ChatGPT 与用户确定、供 Codex 编写执行计划
    ├── TASK01_Codex执行方案.md         # Codex 编写，用户确认后交给 Claude
    ├── TASK01_Claude开发交付.md         # Claude 开发结束的真实 diff、命令、结果与证据
    ├── TASK01_Codex审核报告.md         # Codex 独立复核，给 PASS/REWORK/BLOCKED
    └── TASK01_验收记录.md             # 用户在 RViz 手动检查后记录；可用文字+截图路径
```

实际日志、截图和 JSON 若较多，可在 `work_flow/TASK01_evidence/` 下归档；**不要**复制整个仓库、制造第二套执行目录或把大量中间缓存提交到 Git。测试日志可按项目 `.gitignore`、实验归档惯例处理；报告保留可追溯路径。

角色与阶段：

- **用户 + ChatGPT（已完成此轮）**：定目标、数学原则、工程边界、验收要求；本文件交付 Codex。
- **Codex 设计**：只读检查最新 HEAD / dirty worktree / 实际调用链，提出文件级和函数级详细执行步骤、接口契约、测试命令与验收机制，写 `TASK01_Codex执行方案.md`，**此阶段不修改生产源码**。不可执行的假设要在文档中显式标出。
- **用户确认 Codex 执行方案**。
- **Claude 开发**：按经确认的执行方案改源码、写必要测试、运行可运行的离线与 ROS 测试、记录实际环境与结果；输出 `TASK01_Claude开发交付.md`。如果环境不具备 GUI/ROS，应明确 BLOCKED 并给用户本机复现命令和证据字段，不得声称已实测。
- **Codex 审核**：读取真实 diff、测试、运行日志、数值证明、启动生效配置、图层消息及失败案例；独立复跑可复跑测试，输出 `TASK01_Codex审核报告.md`。未过退回同一任务 Claude 返修，直至通过，**不拆新任务**。
- **用户 RViz 验收**：在自己的 Linux 图形环境使用默认 `run_demo.sh` 选起终点，观察五类图层、HUD、失败保留与三车 Playback，填写 `TASK01_验收记录.md`。**Codex PASS 不替代用户的 RViz 验收。**

---

## 1. 已核查的代码事实与关键风险

以下路径相对于仓库根目录。统一规划包前缀：

`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`

| 当前源码 | 已确认事实／必须核查点 |
|---|---|
| `run_demo.sh` | 编队启动缺省在 shell 中设置 `formation_backend="uniform_v02"`，随后显式注入 `backend:=uniform_v02`；仅改 launch 不够。对于含 `scene_meta.yaml` 的地图，默认可能自动目标。脚本会按地图名做运行时比例缩放；该行为不在本任务随意改动。 |
| `run_virtual_a.sh` | 自动追加 `backend:=virtual_rigid_a`、`auto_goal:=false`，未强制 guidance；要保持为同一研究后端的便捷薄封装，不复制主逻辑。 |
| `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch` | `backend` 缺省 `uniform_v02`；`virtual_a_guidance_mode` 缺省 `legacy_route`；已有 `rviz_config`、HUD 和 ROS 节点 wiring。当前**没有明确暴露** `virtual_a_corridor_enforcement` ROS 参数，需要核查后新增最小透传，不能只写 shell 参数而没有消费者。 |
| `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py::main` | 对 `virtual_rigid_a` 构造 `PlannerConfig(guidance_mode=rospy.get_param("~virtual_a_guidance_mode", "legacy_route"))`，`corridor_enforcement` 目前依赖类默认 `hard`。 |
| `contracts.py::PlannerConfig`、`planner_config_from_dict` | `guidance_mode="legacy_route"` 是历史默认；`as_dict()` 通过 `_PLANNER_CORRIDOR_DEFAULTS` **条件序列化**走廊字段；直接改变 dataclass 默认值会影响旧 payload 反序列化、规划上下文 SHA 与历史证据，必须先设计兼容处理。 |
| `pipeline.py::plan_virtual_a` | 当前有 A* 路线、走廊构建/Guide/分配、IPOPT、retime、动力学/OBB 审核、软预算、结果搜索记录；但没有稳定、显式的实时进度回调接口。 |
| `optimizer.py::_build_casadi_nlp` | corridor 模式下半空间确实约束 **F 点采样位置**，非三车 OBB；必须通过实际约束数量/激活证据检验生效。 |
| `formation_uniform_planner_node.py::_virtual_plan_worker` | 通常同步等待 `self._backend_hooks.solve(map_request)` 返回后处理；成功后 `_publish_corridor_debug`，失败路径 `_finish_virtual_failure` 会 `_clear_planning_debug`，造成失败时图层被抹除。 |
| `formation_playback.py`（ROS 节点脚本） | 已订阅 `/formation/planning_debug`；有 `_publish_hud`、`on_planning_debug`、`_clear_corridors/_clear_guides`；当前 `on_planning_debug` 通常只接受 `PLANNING/PLAYING` 阶段；需要支持本次失败诊断冻结。 |
| `fwis_v3_optimizer/formation_playback.py`（ROS-free 库） | 已有 `hud_text`、`build_corridor_markers`、`build_guide_markers`、MarkerSpec 等构造函数，尽量复用和扩展。 |
| `catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz` | 已有 `/formation/corridors`、`/formation/guides`、`/formation/markers`、`/formation/hud` 显示；其中 A* 的某些 namespace 当前被默认关闭，必须改为用户指定的显示规范。 |
| `tools/configure_formation_view.py` | 运行时读取 RViz 模板并修改地图视角，默认视图配置应改模板而非每次临时补；不得破坏运行时视角缩放。 |
| `optimizer.py::_search_packet` | `heading=_wrap(goal.psi-start.psi)`、`S_upper=min(60,max(12,3*d+10))`；`_route_guess` 的 `S_guess` 会被该上界截断。 |
| `geometry.py::_plan_integration`、`bounds.py::position_guard` | 生产路径采用复合 Simpson 积分；位置预算 `1e-7 m`，roundoff 项 `1e-10 m*(simpson_panels+8)` 使长路径的 guard 随面板数增长。 |
| `audit.py::_integration_plan`、`integration_bounds.py::evaluation_guard` | **已经实现**逐面板四阶导数界和与具体 float64 运算相关的积分求值误差界；应评估移植数学方法到生产积分，保持独立审核自己重建，不另造第三套积分认证系统。 |
| `collision.py` | 已有独立三车 OBB 解析扫掠碰撞审核；不得删除、放松、依靠 RViz 判安全。 |

仓库既有诊断：`evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z/SUMMARY.md`，连同相邻 `probe_*.json`、`cert_guard.json` 作为 TASK 01 的既有复现实验输入。Codex 应先确认文件与对应复现命令，**这些不是本轮新实验结果**。

---

## 2. 本任务最终要做到的用户可见结果（固定 RViz 规范）

### 2.1 五类必需显示（颜色、几何和语义统一）

| 图层 | 显示数据源 | 固定样式 | 用途和安全含义 |
|---|---|---|---|
| **A* 原始路线** | 当前候选 A* 的**未简化** F 点格网路线 | **橙色虚线**，清晰但不盖住墙 | 仅拓扑候选 `DIAGNOSTIC_ONLY`，不是车辆安全证明；A* 尚未产出时不显示 |
| **凸安全走廊** | 实际用于当前轮的已审核工作空间凸多边形 `C_j` | **红色半透明区域**，边界略深、alpha 低于车体 | 只代表 F 点空间走廊，不称三车 OBB 安全走廊；真实 polygon vertices 原样显示，不重新拟合 |
| **Portal 几何引导** | 当前 Portal SLSQP/回退折线 Guide | **绿色折线**；Portal 位置可用小点 | Guide 是初值参考，不能发布为可执行轨迹；区分不存在 Guide 和算法失败 |
| **最终虚拟参考点轨迹** | 通过候选重建的连续 F 曲线 `q_F(s)` 的**真实采样** | **黄色实线** | IPOPT 成功生成但尚未全审者必须标记 `UNVERIFIED_CANDIDATE`；仅全部审核通过才可标记 `CERTIFIED`。无真实曲线则不画 |
| **三车中心轨迹及车体** | 与上述同一条真实曲线、同一 `request.model`、`vehicle_shapes` 推导的三车中心、车体角度和 OBB | **蓝色中心线 + 蓝色 OBB 轮廓**（适度透明，可稀疏采样） | 三车轨迹是实际几何派生，不得画固定同向矩形；未终审时是诊断，不是 Playback；通过后保留现有真正 Playback 动画 |

固定类别不要改成其他颜色。因为红色走廊是二维工作空间图层，界面文字建议写 **`Convex workspace corridors (F point only)`**，避免误导为三车安全证书。

**附加但最小的失败诊断**：如果算法报告带有真实世界坐标或可靠的 `failure_s` 且候选路径可查询，可画紫红色失败标记；若只有失败代码没有可信位置，仅 HUD 显示原因，**不得编造失败坐标**。最小净空位置只有已有实际审核/计算给出可靠数据才显示（可归 TASK 03 丰富），不以此阻塞 TASK 01。

**简化路线**：可以在 RViz 上做低饱和细线默认关闭，或者仅作为 debug 数据保留，以免与指定的五色图层混淆；如实现，必须与 A* 原始路线区别命名。

### 2.2 右上角 HUD 进度显示

维持已有 `jsk_rviz_plugins/OverlayText`，**只在现有 HUD 上扩展**，不新建窗口/节点。至少显示：

- `Plan ID`、`backend/guidance_mode/enforcement`、`State=PLANNING/FAILED/PUBLISHED/PLAYING`；
- 当前**实际执行阶段**（例如 `Step 4/8 NLP_SOLVE`），当前 route/round（有则显示），已耗时（若可可靠计时）；
- 当前阶段生成的真实数据：A* 路线数、区域数、Guide 长度、IPOPT 状态/次数、路径 S、审核结果；**尚无结果写 `-` 或 `PENDING`，不能用示例值**；
- 失败时冻结 `First failure layer`、`Reason`，如有可靠 `s` 和车辆 ID 再显示；成功时显示 `PUBLISHED` 与 geometry/dynamics/collision 三项审核。

建议阶段表（内部数值只是**阶段序号**，不是伪造的完成百分比；候选重试可以回到先前阶段）：

| 阶段号 | `stage` | HUD 文本 |
|---:|---|---|
| 0 | `REQUEST_ADMITTED` | 请求已接纳，身份及地图锁定 |
| 1 | `ASTAR_SEARCH` | 正在搜索路线 / 路线完成 |
| 2 | `CORRIDOR_BUILD` | 构造/验证凸走廊 |
| 3 | `PORTAL_GUIDE` | Portal / Guide / allocation |
| 4 | `NLP_SOLVE` | 数值优化进行中（仅能可靠取得迭代信息时显示迭代数） |
| 5 | `GEOMETRY_AUDIT` | 独立几何/连续走廊及数值认证 |
| 6 | `DYNAMICS_AUDIT` | 共同时间律和动力学审核 |
| 7 | `OBB_COLLISION_AUDIT` | 原地图三车 OBB 连续碰撞终审 |
| 8 | `PUBLISHED` / `FAILED` | 最终结果，失败信息保持至下次接纳 |

> **进度语义**：`Step 4/8` 表示“当前阶段”，**不表示已完成 50% 时间**。预算时间不具实时硬保证；计时必须与真实 monotonic 墙钟相符。候选 1 失败进入候选 2 时，阶段允许回跳，同时显示 `route=2/3` 或 `round`，不能把回跳误当作协议错误。IPO​​PT 内部每次迭代的实时值若目前接口拿不到，只显示 `NLP_SOLVE / running`，在退出后给真实迭代数；不要为“进度感”冒险更换求解器。

示意（仅展示版式，数值不是本轮实测）：

```text
Virtual Rigid A | plan 600004
Backend: virtual_rigid_a / corridor / hard
State: PLANNING | Step 4/8: NLP_SOLVE
Route: 1/3 | Corridor round: 0
Guide length: 45.74 m | winding k: -1
IPOPT: running | S: --
Elapsed: 12.84 s
Audit: PENDING (DIAGNOSTIC_ONLY)
```

失败时用 `FAILED / GEOMETRY / BOUND_UNCONFIRMED` 等**实际证据**更新状态，而不是清空面板；成功后按原状态机展示已认证 Playback 信息。

### 2.3 数据与刷新策略（轻量且安全）

- **事件驱动**：算法到达/完成实际阶段时推送一次状态；不新设独立的轮询服务器。已有 HUD 10 Hz 刷新可利用同一状态和真实 elapsed 计时，无需每次重发大多边形 JSON。
- **单请求身份**：事件必须绑定 `plan_id`、`formation_id`、`generation`、`backend`、`grid_sha256`、`request_sha256`、`planning_context_sha256`；如尚无 `model_sha256`，显式为 `null`，不能伪造。若某个已有 matcher 对该字段的语义与 PLANNING 不兼容，Codex 必须设计向后兼容的身份过渡，并用负例测试。
- **图层渐进更新**：A* 完成出现橙线；走廊完成出现红区；Guide 完成出现绿线；NLP 有真实候选才出现黄线/蓝车。某阶段失败时已完成图层保留，后续尚不存在图层保持空白。
- **不要用过期结果覆盖新请求**：ROS worker 进度回调在处理前验证当前 `generation + plan_id + map/context`；地图版本变化、过期回调、忙碌拒绝、重复 ID 不得覆盖显示或 Playback。新请求**成功接纳**时清旧诊断；请求没有接纳不能随意清掉上一轮认证 Playback。
- **诊断与执行彻底分离**：所有未审核候选只走 `/formation/planning_debug` 和已有 Marker topic，明示 `DIAGNOSTIC_ONLY`；**绝不**进入 `/formation/playback_plan`，不通过改字段伪造审核证书，不能让 RViz 绘制行为改变算法结果、哈希或时间预算。
- **量控**：对连续曲线采样画线时限制点数（沿用已有发布采样限制或设置合理的专用渲染上限）；固定跳帧绘制车体 OBB；不在每次 IPOPT 内部迭代序列化完整栅格、整个搜索树或 20 万点曲线。可视化失败不得使本来可行的规划失败，且不得阻塞下游必要安全审核。

### 2.4 建议接入位置（Codex 需细化）

最小调用链：

```
ROS request admission
  -> formation_uniform_planner_node._admit_virtual
  -> worker _virtual_plan_worker
  -> VirtualABackendHooks.solve(map_request, progress_callback=...)
  -> ros_adapter.solve(...) / pipeline.plan_virtual_a(...)
  -> progress_callback(actual_stage_event)  # 仅报告真实阶段产物
  -> planner node: check generation/map/request -> /formation/planning_debug
  -> playback.on_planning_debug -> existing markers + HUD progress
  -> independent audits -> only certified result may publish /formation/playback_plan
```

具体签名不必须与示意逐字相同，但 Codex 要证明 hook 边界、worker 生命周期与现有测试 mock 兼容；**不要用“捕获所有 TypeError 后忽略 progress 参数”**隐式降级。事件只能单向只读、不能反向改变求解优化目标。

建议优先修改：

1. `pipeline.py`：在 A*、corridor、Guide、NLP、三项审核前后发布小型事件，尽量复用现有 `search/rounds/timings`；失败事件包含第一失败层和真实 reason。
2. `ros_adapter.py::VirtualABackendHooks.solve`：将可选回调透传至 pipeline，保持离线 `solve(map_request)` 旧调用形式可用。
3. `formation_uniform_planner_node.py`：worker 注册回调并校验身份；从“成功后一次性导出”提升为“已完成即发布”；失败路径改为**冻结最后一次诊断**，不要执行清空。
4. `formation_playback.py` ROS 节点：支持 `FAILED` 状态保留当前图层、已接纳新请求才清旧图，右上角 HUD 接收阶段事件。
5. `fwis_v3_optimizer/formation_playback.py` ROS-free 库：尽量扩展现有 `MarkerSpec`/guide/corridor 构造函数；加 A* 虚线、连续 F、三车中心/OBB；render namespace 与 topic 必须稳定，并与认证 Playback 使用不同 namespace 避免相互清理。
6. `formation_demo.rviz` 和 `tools/configure_formation_view.py`：前者设置五类图层可见及颜色/透明度，后者保持视角功能不被破坏；若代码已有图层与新图层重叠，优先复用旧 topic，少加新 topic。

---

## 3. 研究默认入口：用户只运行 `./run_demo.sh` 就使用当前算法

### 3.1 已批准的默认行为

在用户本机项目根目录（**不是** `work_flow` 目录）运行：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_demo.sh --map map05 --scale 2.625
```

**在有 ROS GUI 的编队研究模式下**，应自动解析为：

```text
backend = virtual_rigid_a
virtual_a_guidance_mode = corridor
virtual_a_corridor_enforcement = hard   # 须实现 shell -> launch -> node -> PlannerConfig 透传
rviz = true                            # 有图形桌面时
                                        # 无 GUI 时允许 false，但要显式提示
改为手动起终点：auto_goal = false
```

用户通过 RViz `2D Pose Estimate` 设置编队起点，`2D Nav Goal` 设置终点。遵循已有虚拟后端 reset/初始状态确认规则，不能为方便测试绕过 admission。

**不要将演示运行结果或示例数字写死到脚本**。本任务默认研究配置应始终追踪当前分支的真实实现；后续 TASK 02、TASK 03、TASK 04 完成后，Claude 仍必须核查并更新这个脚本的帮助说明、配置显示和当前研究阶段身份，保证普通命令没有跑回旧算法。

### 3.2 需要一起核查的所有默认值

1. `run_demo.sh` formation 分支：`formation_backend` 默认改成 `virtual_rigid_a`，默认 `virtual_a_guidance_mode:=corridor`、hard；**尊重用户显式参数**，不能因注入默认值造成同名 roslaunch 参数重复或覆盖。优先只在 `virtual_rigid_a` 模式注入研究相关参数；显式 `uniform_v02`、`legacy` 保留历史行为。
2. `formation_demo.launch`：无上层脚本直连启动时的研究默认 `backend`、`virtual_a_guidance_mode`；增加并透传 `virtual_a_corridor_enforcement`（若源码已有相应机制则直接用现有）。
3. `formation_uniform_planner_node.py::main`：读取 `~virtual_a_guidance_mode`、`~virtual_a_corridor_enforcement`，明确验证后写入 `PlannerConfig`；禁止 ROS 参数被忽略却仍打印 `hard`。
4. `run_virtual_a.sh`：保持对 `run_demo.sh` 的薄包装，缺省同为 `corridor/hard`；不需要再出现第二套默认算法分配逻辑。
5. `contracts.py::PlannerConfig`、`planner_config_from_dict()`：不允许离线新研究调用静默默认旧 guidance；但**不能破坏历史缺省 payload 语义与 request/planning-context digest**。建议 Codex 优先考虑“旧字节 payload 用显式旧默认反序列化，新构造调用使用最新研究默认”的兼容实现；若不可行，提出可验证的版本演进/迁移方案并获得用户确认，绝不能直接删 `_PLANNER_CORRIDOR_DEFAULTS`。
6. `run_demo.sh --single` 继续跑原单车路径；显式旧编队后端仍可用；地图缩放/场景目录索引/`--dry-run`/编队尺寸配置保持已有契约。

### 3.3 默认模式验收不是看脚本打印，而是看实际 NLP

必须证明：

`./run_demo.sh` → launch 有效参数 → node 实际 `PlannerConfig` → `pipeline` 的 corridor 分支 → corridor_chain / allocation → `optimizer._build_corridor_constraint` → `_build_casadi_nlp` 中确实存在 F 点半空间约束 → 原有独立审核 → 可执行时才播放。

建议证据字段：有效 `backend/guidance_mode/corridor_enforcement`、`plan_id`、request/map/context hash、corridor chain digest、当前轮的半空间约束条数、IPOPT 约束残差、真实失败层；**不能仅凭看到红色走廊或绿色 Guide 就声称硬约束已生效。**

---

## 4. 正确性修复 A：终点累计航向 winding 分支

### 4.1 现有问题

`optimizer.py::_search_packet` 当前：

```python
heading = _wrap(goal.psi - start.psi)
```

但曲率轨迹的终点累计转角是：

\[
\Delta\psi = \int_0^S\kappa(s)\,ds
\]

物理终点朝向相同的数值展开分支有：

\[
\boxed{\Delta\psi_k = \operatorname{wrap}(\psi_g-\psi_0)+2\pi k,\quad k\in\mathbb Z}
\]

当前 NLP 却只把 wrap 后的主值作为严格终点等式右侧。当绕障路线要顺时针绕行而主值要求逆时针展开时，可能把可行候选误拒。

已存诊断 `600004`：主值约 `+3.124 rad`，Guide 约 `-2.951 rad`，合适的候选展开分支约 `-3.159 rad`。`600003` 短 U-turn 已有已认证路径约 `21.70 m`；不能因修复而退化。

### 4.2 数学方案（原则确定，阈值交由 Codex 验证）

1. 对当前**候选** Guide 连续展开各折线段切向角 `phi_j=atan2(dy_j,dx_j)`，估算累计转角：

\[
\Theta_G=\sum_j \operatorname{wrap}(\phi_{j+1}-\phi_j).
\]

2. 选有限的候选 winding 集合 `K`，先选与 Guide 方向一致且累计转角差最小的展开分支：

\[
\boxed{k^*=\arg\min_{k\in\mathcal K}\left|\Delta\psi_k-\Theta_G\right|}
\]

3. 需要把起点和终点要求切向一起纳入可靠性判断：折线切向与真实平滑曲线不完全相同；对短折返、折线退化或 Guide 不可信时，不得盲选唯一分支。Codex 应给出有限的邻近候选尝试规则和对应预算（例如只优先选择 1 个、仅明确失败且预算允许时考虑最多少数邻近分支），不做无界 winding 枚举。
4. 对同一候选：`_search_packet`、`_route_guess`、`make_initial_guesses`、NLP 终点约束、曲率积分重建、诊断字段使用**同一个** `k` 和 `Δψ_k`；请求中的物理 goal 姿态不能被改写。终审仍检查模 `2π` 的物理朝向，同时记录所用展开分支，不能把角度差 `2πk` 误当物理朝向误差。
5. 终点约束仍保持等式：

\[
\int_0^S\kappa(s)ds=\Delta\psi_k.
\]

不将严格终点条件改成松软目标，也不引入整数混合规划。

### 4.3 推荐代码位置

- `pipeline.py`：当前 route/Guide 的 winding 估计和候选身份，避免把一个候选的分支套到另一个候选。
- `optimizer.py`：`_search_packet` 的分支输入、自动初值、实际 solve 调用、约束右端与 metadata；处理无路线/endpoint-only 路径时的显式受限策略。
- `contracts.py` / `audit.py`：如增加诊断字段与序列化身份，保护历史请求结构与最终审查；只修改必要契约。

### 4.4 验收

`600003` 不回归；`600004` 不再因唯一错误航向主值而失败；单元用例覆盖 `ψ_g≈±π`、同物理姿态差 `2π` 的输入、顺时针/逆时针绕障及无可靠 Guide。记录 `base_heading, winding_k, unwrapped_heading, guide_turn, selected_candidate`。

---

## 5. 正确性修复 B：路径长度 S 搜索上界与 Guide 绕障长度关联

### 5.1 现有问题

当前 `optimizer.py::_search_packet`：

```python
lower = max(intervals * 0.05, distance)
upper = min(60.0, max(12.0, 3.0 * distance + 10.0))
```

其中 `distance=||goal-start||`。这不是绕行实际需要的路径长度：`600004` 的直线距离约 `9.53 m`、Guide 长约 `45.74 m`、原 `S_upper≈38.59 m`；将上界增大到 `50 m` 且修正航向后，既有实验 IPOPT 可给出约 `48.44 m` 候选，**但此时仍没有通过数值认证与完整终审**。不能把 IPOPT 成功写成整机规划成功。

### 5.2 数学方案

保持路径长 S 为现有 19 个变量中的首变量，不改轨迹模型。令所选 winding 的展开角为 `Δψ_k`、欧氏端点距为 `d`、候选 Guide 长为 `L_G`、F 点曲率的安全上界为 `κ_max`，则必要下界：

\[
\boxed{S_{\min}\ge \max\left(d,\frac{|\Delta\psi_k|}{\kappa_{\max}},S_{\mathrm{num}}\right)}
\]

其中 `S_num` 保留原分段数和数值域要求；若该式的曲率界对当前三车配置不适用，Codex 应以实际约束推导更严格或正确的 F 点有效界。它是**必要条件**，不能保证物理可行。

初始上界依赖 Guide：

\[
\boxed{S_{\max}^{(0)}=\max\left(S_{\min}+\delta,12,3d+10,\alpha L_G,\beta\frac{|\Delta\psi_k|}{\kappa_{\max}}\right)}.
\]

`α≈1.15`、`β>=1` 可以作为**实验起始建议，尚未冻结**；对 `600004`，`1.15×45.74≈52.60 m`。取消没有物理依据的硬编码 `60m` 总上限；但**每轮求解都保持有限上界、有限迭代和真实墙钟预算**。Guide 折线长度不是严格下界，不能设置 `S_min=L_G`。

可选受控扩域：仅当观察到可信的上界活动/边界残差且时间和初值预算允许时，在同一请求中有限增加 S 域；IPOPT 的一般 `Infeasible_Problem_Detected` **不得**被视为“必然 S 不够”。具体扩域次数、触发条件由 Codex 根据已知诊断设计并给证据；禁止无限重试、无限扩域或把预算耗尽当物理无解。

### 5.3 工程要求

- `_search_packet` 产出候选独有且一致的 `S_lower/S_upper/Δψ_k`，不得多个函数各算一套；
- `_route_guess` 的 S 初值不会被过小域截断，`make_initial_guesses` 与最终 IPOPT 变量上下界完全一致；
- `endpoint-only` 旧初值仍可按明确的有限规则生成，保持 fallback 的公开身份、原有 `MAX_INITIALS` 和请求预算，不因新域产生重复求解；
- 记录 `distance, guide_length, S_lower, S_upper_before_after, S_solution, upper_active_flag, attempt_count`。

### 5.4 验收

分别测试短直线、长绕行、`600003/600004`、50 m 以上直线与 100 m 量级轨迹构造；**不能**因固定 60 m 上界拒绝。预算耗尽、上界受限、求解未收敛、下游认证失败须给不同原因。

---

## 6. 正确性修复 C：长路径数值积分认证不再被浮点 guard 误拒

### 6.1 已确认的问题

轨迹空间模型：

\[
\frac{dx}{ds}=\cos\psi(s),\quad \frac{dy}{ds}=\sin\psi(s),\quad
\frac{d\psi}{ds}=\kappa(s),\quad s\in[0,S].
\]

`optimizer.py` 使用 `N=8` 段三次 Hermite 曲率，默认决策变量：

\[
\boxed{z=(S,\kappa_0,\ldots,\kappa_8,\eta_0,\ldots,\eta_8)^T\in\mathbb R^{19}}
\]

每段 `h=S/N`、`q∈[0,1]`：

\[
\begin{aligned}
\kappa_j(q)&=c_0+c_1q+c_2q^2+c_3q^3,\\
c_0&=\kappa_j,\quad c_1=h\eta_j,\\
c_2&=3(\kappa_{j+1}-\kappa_j)-h(2\eta_j+\eta_{j+1}),\\
c_3&=2(\kappa_j-\kappa_{j+1})+h(\eta_j+\eta_{j+1}).
\end{aligned}
\]

`geometry.py::_plan_integration()` 当前按 `MAX_PANEL_WIDTH_M≈0.02m` 初始化 Simpson 子区间，`bounds.py::position_guard()` 中：

```python
roundoff = POSITION_ROUNDOFF_GUARD_M * (simpson_panels + 8)
angle = length_m * angle_bound_rad
# POSITION_ROUNDOFF_GUARD_M = 1e-10 m, POSITION_BUDGET_M = 1e-7 m
```

固定每面板 `1e-10m` 的保护会随路径长度/面板数量近似线性增长，旧诊断指出约 `36.8m` 起仅 guard 已可能耗尽预算，继续细分反而更不利。对于 `600004`，修正航向与 S 域之后 IPOPT 可收敛，但 `BOUND_UNCONFIRMED` 阻止数值轨迹认证。

**特殊事实**：`audit.py::_integration_plan` 和 `integration_bounds.py` 已有逐面板 `M4`、独立 float64 evaluation guard；任务 1 要利用它们的思路，**不是重建第三套认证框架**。

### 6.2 保持数值精度目标，改变误差估计的失真来源

对 `f(s)=e^{i\psi(s)}`，Simpson 面板宽 `H`，截断余项：

\[
\boxed{E_{\mathrm{panel,coord}}\le\frac{H^5}{2880}M_4},\quad
M_4\ge\max_{s\in\mathrm{panel}}|f^{(4)}(s)|.
\]

记 `κ=ψ'`, `η=κ'`, `u=κ''`, `g=κ'''`，则：

\[
\boxed{|f^{(4)}|=\sqrt{(\kappa^4-4\kappa u-3\eta^2)^2+(g-6\kappa^2\eta)^2}}.
\]

用现有 `integration_bounds.panel_m4_bounds` 在每个完整 Simpson 面板生成向外包围的 M4，保留任意弧长局部查询的一个附加面板界。分段及全局相加得到截断误差 `E_quad`。

总位置误差按来源拆分：

\[
E_{\mathrm{position}}\le E_{\mathrm{quad}}+E_{\mathrm{float}}+E_{\mathrm{angle}}.
\]

在**明确单坐标与欧氏误差单位**前提下，一种与现有独立审核器一致的保守组合是：

\[
\boxed{E_{\mathrm{position}}\le\sqrt{2}\left(E_{\mathrm{quad,coord}}+E_{\mathrm{eval,coord}}+S E_\psi\right)}.
\]

保持 `POSITION_BUDGET_M=1e-7 m`、已有航向预算和审核门槛，不得通过简单放宽阈值制造通过。实际实现需要将三项的向外舍入、三角函数 libm 假设、前缀累加、坐标原点量级与 Simpson 查询方式逐一说明。

现有 `integration_bounds.evaluation_guard()` 给出的**审核器特定**浮点界形式包括：

\[
E_{\mathrm{eval,seg}}\sim \epsilon\sum_j h_j(8n_j+28+16\Psi)
\]

和 `ψ` 前缀累加、位置前缀累加两项。该函数针对审核器自身计算路径设计，**不能原封不动套在生产 `geometry.py`**：Codex 必须核对生产计算是否同样的 Horner/Simpson 算法、是否缓存 `sin/cos`、是否使用相同求和路径，必要时建立生产侧有根据的独立误差界。若重用基础多项式上界代码，生产与 audit 仍各自对同一 raw knots 计算，不直接信任对方 `position_error_bound_m`。

### 6.3 细分停止规则

现有算法一旦预算不满足，就选最大余项段继续细分，而 guard 随面板数增长，可能出现无意义迭代。建议依据**下一步细分后的保守总误差预测**决定：

- 预算满足即接受，保存每项组成及证明假设；
- 若截断误差主导、细分可净降低总误差，有限细分高误差段；
- 若浮点/航向不可消除项已耗尽预算或继续细分使总界更差，停止并报告 `BOUND_UNCONFIRMED`，附余项与无法降低的保护项；
- 仍有最大面板数/段数/墙钟保护；任何数值不确定不是物理碰撞，也不能标为通过。

任意位置查询 `state_at_s(s)` 必须与端点一样受控：连续 OBB 和几何审核依赖整段路径，不仅仅是终点。原地图 OBB 误差包络不能降低。

### 6.4 建议代码位置

- `geometry.py::_plan_integration`：误差预算、面板细分、证书字段及任意 `s` 查询一致性；
- `bounds.py`：替换生产轨迹与其实际浮点操作相关的粗糙固定每面板 guard，保留通用向外区间运算；
- `integration_bounds.py`：复用逐面板 M4 基础，必要时为生产路径扩展局部误差界，但不新建全新审查框架；
- `audit.py`：维护独立重建/检查；调整数值契约只在必要范围内，历史模型证据变化显式记录版本。

### 6.5 验收

- 在保存的 600004 曲率候选上复算：新生产积分误差证据满足预算时通过构造，独立审核另行通过；若不能满足，给可核查 `BOUND_UNCONFIRMED`，不能伪造全流程成功。
- 50m 以上直线、100m 量级具有良好条件的轨迹不再被固定 guard 无端拒绝；有界/退化反例仍然拒绝；
- 数值界说明包含：面板宽度、M4 局部界、Simpson 截断、浮点 eval、角度、总误差、向外舍入、`state_at_s` 查询；
- 不改变原始三车曲率约束、动力学约束、独立 OBB 审核判据。

---

## 7. 建议的 Claude 实施顺序（属于**同一个** TASK 01）

Codex 的执行方案可将以下作为内部开发步骤，不拆成新的管理任务、PR 或第二套系统：

| 内部顺序 | 改动目标 | 每一步做完必须证明什么 |
|---:|---|---|
| 1 | 冻结 HEAD/原始默认行为及 `600003/600004` 基线 | 有可复现命令、原始失败层和旧耗时；源文件/地图身份已锁定 |
| 2 | 接通 RViz 实时进度/五图层/HUD/失败冻结 | 默认可视化基座先能观察当前**旧结果**；失败时 A*/走廊/Guide 保留；五色规范生效 |
| 3 | 修复同一个 `run_demo.sh` 默认研究入口，透传 `corridor/hard` | 普通脚本确实运行 `virtual_rigid_a/corridor/hard`、真实 F 半空间约束进入 NLP；旧后端显式入口可用 |
| 4 | winding 有限分支选择 | `600004` 不再被错误主值分支预先阻塞；短 U-turn 和物理终点姿态不回归 |
| 5 | Guide 相关的 S 搜索域 | 45m 级 Guide 允许在合理有限域里生成 48m 级候选；无固定 60m 上限；预算/初值计数仍受控 |
| 6 | 生产积分误差 guard/细分修复 | 可审查的保守误差界；长候选不因旧每面板守卫系统性拒绝；独立审核仍独立 |
| 7 | 统一全链路回归与真实 ROS/RViz 实测 | 成功/失败行为、五层显示、HUD、真实求解、三项审核与原子 Playback 证据齐全 |

**注意**：先完成可视化不意味着要对没有路径的失败画不存在的黄线或蓝车。只有当前真实算法生成的中间产物才能显示。

---

## 8. 测试矩阵与可量化验收门槛

### 8.1 对外可见 GUI 验收（用户关心的主门槛）

1. Linux GUI 环境直接运行 `./run_demo.sh --map map05 --scale 2.625`，在启动日志/实际 ROS 参数/HUD 可见 `virtual_rigid_a / corridor / hard`；不手动加 `backend:=...`、`virtual_a_guidance_mode:=...`。
2. RViz 手动 `2D Pose Estimate` + `2D Nav Goal` 触发规划：
   - A* **橙色虚线**在 A* 完成后即时出现；
   - 凸区 **红色半透明**在构造并验证后出现；
   - Portal Guide **绿色折线**在生成后出现；
   - 有真实优化连续曲线时出现 **黄色实线**；
   - 有同一曲线的真实三车几何时出现 **蓝色车中心线及 OBB**；
   - 在整个过程中 HUD 能显示 `Step / actual stage / plan / backend / candidate`，失败时显示真实 `reason`，不是静态的“正在规划”。
3. 故意选择一个失败用例：保留已生成的 A*/走廊/Guide 与失败原因，直到**下一条已接纳的新请求**；对尚未生成的层不画假数据。
4. 调整地图、连发请求、使用重复或迟到消息：旧图层不覆盖新请求，忙碌拒绝不清除当前已认证 Playback；规划未认证结果不会出现在 `/formation/playback_plan`。
5. 用旧算法显式启动做对照；`--single` 继续跑原单车流程，默认研究启动不影响它。

### 8.2 算法正确性验收

| 场景 | 应验证的结果 |
|---|---|
| `600003` 已成功短 U-turn | 保持可复现的已认证能力，不因 winding/S/数值修复回归 |
| `600004` 长 U-turn | 三个既知阻塞点不再构成无端失败；若真正因碰撞、动力学或超时失败，给明确第一失败层；不要求通过伪造证书产生“SUCCESS” |
| `ψ_goal≈+π/-π`、表示差 `2π` | 物理终点相同，候选选择合适的累计展开分支，NLP 等式与初值一致 |
| 顺时针和逆时针绕障及无可信 Guide | 分支选择有依据；无 guide 保留有界工程策略，不错误宣布无路 |
| `S>50m` 直线、`S≈100m` 已知可构造路径 | 搜索与认证没有无物理依据固定 60m/固定 guard 天花板；仍使用相同严格数值预算 |
| 故意数值病态轨迹 | `BOUND_UNCONFIRMED` 等明确失败，绝不软化阈值或绕过认证 |
| F 点半空间、三车运动学曲率与 OBB 原地图检查 | 既有约束与独立审查未被破坏，NLP 收敛不被误写成全部认证通过 |

### 8.3 需要新增或修改的最少回归测试

仓库现有 `tests/` 下含以下相关脚本（以最新工作区实际存在为准）：

- `tests/test_virtual_a_corridor_display.py`：已含 corridor/Guide Marker 与 ROS planner/playback 测试基础；新增五类图层、进度/失败保留、身份负例。
- `tests/test_formation_hud.py`、`tests/test_virtual_a_receiver.py`、`tests/test_virtual_a_playback.py`：HUD 不同阶段、请求身份、Playback 严格分离。
- `tests/test_virtual_a_optimizer.py`、`tests/test_virtual_a_pipeline.py`、`tests/test_virtual_a_corridor_optimizer.py`：winding、S 参数包、实际 corridor 约束使用、保留旧初值池/预算。
- `tests/test_virtual_a_geometry.py`、`tests/test_virtual_a_integration_bounds.py`、`tests/test_virtual_a_audit_evaluation_guard.py`、`tests/test_virtual_a_audit.py`：数值误差、连续审查、边界反例。
- `tests/test_formation_demo_assets.py`：shell/launch/RViz 配置默认值、参数传递、旧模式。
- `tests/ros_virtual_a_smoke.py`：真实 ROS 烟雾测试；Codex 要先检查输入及 CLI，不能假设可任意运行。

可供 Codex **核对后采用**的离线命令示例：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
python3 -m pytest -q \
  tests/test_virtual_a_corridor_display.py \
  tests/test_formation_hud.py \
  tests/test_virtual_a_optimizer.py \
  tests/test_virtual_a_pipeline.py \
  tests/test_virtual_a_geometry.py \
  tests/test_virtual_a_integration_bounds.py \
  tests/test_virtual_a_audit_evaluation_guard.py \
  tests/test_virtual_a_playback.py \
  tests/test_formation_demo_assets.py
```

> 部分 ROS test 需 `source /opt/ros/noetic/setup.bash` 和 `source catkin_ws/devel/setup.bash`、`catkin_make` 之后才能导入。**以上是候选命令，不声称本次已经运行通过**；Codex 应基于环境给出可执行的最终步骤。不要依赖截图代替数值测试，也不能只做 dry-run。

启动与诊断的**候选**命令：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_demo.sh --map map05 --scale 2.625 --dry-run
./run_demo.sh --map map05 --scale 2.625
# 在另一终端执行（需 ROS 环境）：
rostopic echo /formation/planning_debug
rostopic echo /formation/job_state
rostopic echo /formation/quality_report
# 对照（只在独立实验中单独启动）：
./run_demo.sh --map map05 --scale 2.625 --backend uniform_v02
./run_demo.sh --map map05 --scale 2.625 backend:=virtual_rigid_a virtual_a_guidance_mode:=legacy_route
```

不能把 `--dry-run` 成功当成 ROS 真正运行成功；每次测试要标注图形环境、启动命令、地图比例、起点/终点、生成的 plan_id 与最终状态。

### 8.4 每次实验最少记录哪些字段

```
HEAD; dirty state; datetime; command; ROS/Python/CasADi versions;
map path/scale + grid_sha256; request_sha256; planning_context_sha256;
plan_id/formation_id/generation/backend/guidance_mode/enforcement;
route index/source/count; raw route length; Guide length; corridor chain digest;
selected winding k and DeltaPsi_k; S_lower/S_upper/S_solution;
IPOPT status/iterations/residuals; numerical trunc/eval/angle/total budgets;
stage timings; first failure layer/reason; final status/executable/published;
geometry audit; dynamics audit; original-map OBB audit;
planning_debug event snapshots; RViz screenshots or screen-record filenames.
```

对于 IPOPT 失败或仅部分计算结果，未获取的数据必须写 `UNAVAILABLE`，不能按另一次成功实验补值。

---

## 9. 最小代码修改范围与明确的禁止事项

### 9.1 允许修改的核心文件（不是要求每个都大改）

- `run_demo.sh`、`run_virtual_a.sh`、`catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch`、`catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz`、必要的 `tools/configure_formation_view.py`
- `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`
- `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py`
- `formation_continuous_a/` 下的 `pipeline.py`、`ros_adapter.py`、`optimizer.py`、`geometry.py`、`bounds.py`、`integration_bounds.py`、必要的 `audit.py` 与 `contracts.py`
- 现有 `tests/test_virtual_a_*`、HUD/ROS/node/launch 测试及明确需要的少量 fixtures

### 9.2 不做

- 不引入另一个 planning manager、event bus、额外 ROS 可视化节点、前端网页或另一个整套测试框架。
- 不重写二维 A* / Portal 几何/集群配置搜索；这些由 TASK 02 实现。
- 不在 TASK 01 把三车 OBB 硬约束塞进 IPOPT；这是 TASK 03。
- 不换成 iLQR、DDP、混合整数优化，不重做现有五次时间律，也不擅自改车辆尺寸或编队偏移。
- 不增加数值容差、不删除 guard、不跳过独立几何/动力学/原地图 OBB 安全审查、不借可视化修改可执行证书。
- 不使用“超时就标无路”或“优化器收敛就可播放”的错误状态判断。
- 不为了凑 GUI 效果画虚假的路线、OBB、失败点或伪造进度百分比。
- 不偷偷把 `legacy_route` 作为 corridor 模式失败后的自动 fallback；旧算法仅供明确选择做对照。

---

## 10. Codex 现在需要编写的详细执行方案（唯一正式交付请求）

**请 Codex 先阅读本 TASK01 技术方案，在本地工作区核对最新代码，输出**：

`/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/TASK01_Codex执行方案.md`

此文档必须是可交给 Claude **逐项实施、逐项自测**的开发计划，内容至少包括：

1. **真实基线**：`pwd`, `realpath run_demo.sh`, `git status --short`, `git rev-parse HEAD`, 当前分支；具体文件/函数与现状证据；若代码已变更，明确与本文件差异。
2. **精确依赖与改动顺序**：先可视化+HUD→默认入口→winding→S→数值 guard→全回归，列出每项修改的最小函数签名、调用方及数据传递方式。
3. **逐文件修改规格**：每个触及的文件写清楚改哪一段、输入/输出和不变量、原行为如何兼容；**特别处理** progress callback 的线程和身份、FAILED 图层不清除、Marker namespace、HUD stage、Python 历史 config 的反序列化/sha。
4. **数学正确性**：winding 候选估计、无 Guide 时有限兜底；S 上界与扩域触发；Simpson 局部 M4、生产 float64 error guard 与任意弧长查询的保守性。公式和各个误差项的单位、假设必须可审核；疑难数值参数不得随便拍常数。
5. **真实可执行测试命令**：逐步的单测、离线重现 `600003/600004`、`run_demo.sh`/ROS/RViz 手工测试、老入口/安全回归；区分机器可自动执行与必须用户本机 GUI 的项目。
6. **具体日志/证据路径与样例**：证据命名、字段、截图位置、如何比对同地图/请求、如何证明 corridor constraint 确实进入 NLP、如何保证没有计划发布绕过安全门。
7. **Claude 交付模板**：代码提交 SHA、实际改动清单、测试原始命令与退出码、无法运行的环境障碍、已知问题、RViz 观察截图、未通过项、是否已更新默认 `run_demo.sh`。
8. **Codex 独立审核清单**：技术目标逐条 PASS/FAIL/BLOCKED，解释复现证据、回归差异、数值认证是否可靠、是否需要返修。不能只写“代码符合任务要求”。
9. **回滚/兼容策略**：配置和历史模型身份、ROS publication/admission、Marker 旧图层及单车/旧编队回归失败时怎么定位和撤销。
10. **明确开发边界**：不拆出数十个独立任务，不新增冗余框架；确有必要做重大模型调整时，Codex **先列出需用户批准的设计变更，不让 Claude 擅自扩大开发范围**。

**Codex 写好并经用户确认后才交 Claude。** Claude 完工再生成 `TASK01_Claude开发交付.md`；Codex 依据实际代码与证据生成 `TASK01_Codex审核报告.md`；用户最后用默认脚本和 RViz 验收 TASK01。只有 TASK01 的这一整套闭环通过，才进入 TASK02 正式实施。

---

## 11. TASK 01 最终通过的判定

该任务最终完成必须**同时**满足：

- [ ] 用户普通运行 `./run_demo.sh`（编队研究模式）就进入 `virtual_rigid_a/corridor/hard`，ROS 内实际生效；旧算法显式可用、`--single` 不回归。
- [ ] 用户 RViz 看到橙色虚线 A*、红色半透明凸走廊、绿色 Portal Guide、存在真实连续候选时黄色 F 曲线、蓝色三车中心/真实 OBB；颜色、语义、更新时序和失败保留都正确。
- [ ] HUD 右上角显示实际阶段、当前候选/状态、真实失败原因，无虚假百分比；旧进度/新请求身份隔离。
- [ ] `600003/600004` 与正反 winding/长 S/数值边界回归运行并保存证据，三个已知实现缺陷不再导致系统性错误拒绝；任何仍失败的结果诚实分类。
- [ ] 完整几何/动力学/原地图三车 OBB 安全终审及原子 `/formation/playback_plan` 发布闸门不被绕过；未认证预览绝不被执行。
- [ ] Claude 写交付文档，Codex 出独立审核文档（PASS 或明确的返修项），用户最后按 RViz 实测完成验收记录。

> **声明**：截至 2026-11-18，本方案的状态是“详细技术需求已编写、尚未开发”。上述路径、函数和基线来自 `feature/virtual-rigid-formation-a` 远端核查；本文未修改用户本机 `/home/wq/` 下的源码，也未声称运行了新的 ROS/RViz 实验。
