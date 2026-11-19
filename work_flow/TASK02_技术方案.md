# TASK 02｜面向狭窄通行的三车集群走廊与 Portal 候选重构——提交 Codex 的详细技术方案 V1.0

> **文件用途**：这是**用户 + ChatGPT 已确定技术方向的需求/算法设计文件**。下一步由 **Codex** 在用户本地最新仓库中核查源码、完成具体接口与文件级实施设计，生成 `TASK02_Codex执行方案.md`，然后交 **Claude** 按文档实施。本文**不授权 Codex 直接修改生产代码**，不等于 Claude 的执行方案，也不代表任何算法已验收。
>
> **本地存放路径（由用户自行下载放入）**：`/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/TASK02_技术方案.md`。此处不要求 ChatGPT 修改用户本机目录。
>
> **仓库**：`https://github.com/WangxQiang21/CopTrans_base`  
> **开发分支**：`feature/virtual-rigid-formation-a`  
> **编写本稿时已核对的 HEAD**：`222b919329bf487db1ee096e2902d753385a3527`，2026-11-19。Codex 开始之前**必须重新记录本机 HEAD、`git status` 和 RViz 配色修改是否已落地**；本稿以核对时源码为事实起点，不能假设后续提交内容已经存在。
>
> **研究对象**：三辆已处于给定几何编队关系的 4WIS 车辆，在 **A 模式（前后阿克曼）** 下协同通过狭窄环境。规划对象是虚拟编队参考点 F 的连续曲率轨迹；每车实际中心及车身朝向由同一参考状态和固定编队偏移确定。本任务不研究编队形成、不新增载荷实体模型、不开发 O/S 模式或控制器。

---

## 0. 本任务与现有四任务体系的关系

| 已有/后续工作 | 本任务承接或交付什么 | 禁止误认为已完成什么 |
|---|---|---|
| TASK 01（最新分支已有实现） | `virtual_rigid_a/corridor/hard` 默认研究入口；winding 分支、候选长度域、生产 `g1.v3` 数值证书；分阶段 ROS/RViz 进度、候选 F/车辆轨迹可视化 | 不重新设计 winding、积分误差、不放松已有连续认证；任务 1 是否最终被 Codex 审核通过应单独确认 |
| **TASK 02（本文）** | 保留二维 A*；建立三车配置/地图预评估、相邻走廊并集的完整 OBB 覆盖语义、有限 Portal 过渡候选、集群感知 Guide、实际候选合同和 RViz 显示 | **不声称已建立 IPOPT 的完整三车 OBB 硬约束或连续安全证书** |
| TASK 03 | 根据 TASK 02 的候选和跨 Portal 语义，让三车 OBB 进入 IPOPT，加入有意义的净空软代价，建立集群连续走廊认证 | TASK 03 不重新开发一套 A*/Portal/Guide，也不偷换成“三车同时在同一个凸区” |
| TASK 04 | 基于真实 profile 进行 A*/EDT/IPOPT、调度及总预算优化，最终全面回归 | TASK 02 不为猜测的性能收益提前增加通用缓存框架或重写整个搜索器 |

### 0.1 用户已批准的两个架构决策——不可由 Codex 自行改回

1. **保留现有二维 8 邻域 A***：用它提供有限拓扑路线，之后才叠加三车姿态/曲率/OBB 的几何预评估。**不默认重写全图 SE(2) 或 `(x,y,ψ,κ)` 搜索**；只有可复现实验表明二维 A* 无法提供必要拓扑时，才在后续提出局部扩展。
2. **Portal 以相邻自由凸区并集为目标空间语义**：允许三车分别位于 `C_j`、`C_{j+1}`，也允许**单辆完整 OBB**跨越二者的人工分割边界，只要该 OBB 整体位于 `C_j ∪ C_{j+1}`。**不能只检查车中心、四角点，也不能强制三车同时完整处于同一个 `C_j`。**

### 0.2 核心实施原则

- **更强的几何感知，但不伪造证书**：当前阶段允许有限静态采样的候选几何检查，结果只能表述为“这些样本已验证/观察到风险/无法判定”，不能宣称整段连续路径安全。
- **修改运行中的主算法，而不是旁路演示**：所有新候选评估、Portal 数据和 Guide 决策必须在**默认 `run_demo.sh → virtual_rigid_a → corridor/hard → plan_virtual_a`** 的真实调用链中使用；写一段离线函数、画好看 marker 不等于任务完成。
- **不把局部走廊不足当物理无路**：地图 实际 自由空间可能大于人工构造的凸区并集；当 `B_i ⊄ C_j ∪ C_{j+1}`，若原地图静态检查未发现碰撞，应标记 `CORRIDOR_COVERAGE_GAP` 或 `UNCONFIRMED`，不直接得出 `PHYSICALLY_INFEASIBLE`。
- **尊重现有发布闸门**：不删减几何、动力学、原地图三车 OBB 连续碰撞审核，也不改变 `/formation/playback_plan` 原子发布身份验证。未经完整终审的图层一律 `DIAGNOSTIC_ONLY`。
- **节制开发**：以现有 `pipeline.py/corridor.py/contracts.py` 为主，扩展已有 `GuideRoute/CorridorChain` 和 `search['corridor']['rounds']`；必要时只新增一个简洁不可变候选对象，不开发新 manager/factory/全局规划框架。

---

## 1. 当前代码的真实实现与任务 2 要解决的缺口

### 1.1 已核对源码位置（相对仓库根目录）

源码前缀统一为：

`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`

| 当前文件/函数 | 当前事实 | TASK 02 的具体关注点 |
|---|---|---|
| `pipeline.py::make_route_guides()`、`_astar_route()` | 最多三条 8 邻域 F 点 A* 路线，配置 `RouteGuideConfig.blocked_radius_m=0.15` m；路线可被 LOS 简化；多个备选按带状惩罚重新搜索 | 维持搜索职责；在简化后评估三车占地及候选风险；不能把 0.15 m 当成三车的外接半径 |
| `route_processing.py::simplify_route()` | 基于 F 点地图 LOS 的简化，非三车车辆路径 | 继续用于拓扑/种子；不把简化线变成车队已经可以通过的证明 |
| `corridor.py::_build_region()`、`build_corridor_chain()`、`audit_corridor_chain()` | 从简化路线线段种子生成经栅格障碍分离的凸区，核验工作空间区域；`ConvexCorridor` 保存单位外法向 `A`、`b`、顶点 | 复用二维 C 区域，不重写现有半空间生成器；加上**局部三车配置可行性信息** |
| `corridor.py::_portal_bundle()` | Portal 来自两相邻区域半空间交集的 Chebyshev 中心及内切圆 | 此处仅能证明**F 点空间重叠**；不能作为三车 Portal 通过证明 |
| `corridor.py::make_corridor_guide()` | 产生端点及 Portal 内部点的折线/QP Guide，F 点沿 Guide 在相应区域里；没有连续曲率/三车可行证明 | 保留此初值，再做有限集群感知评估和局部 Guide 改进 |
| `corridor.py::allocate_corridors()` | 沿 Guide 长度分配 `σ=s/S` 的区域索引，给现有 F 点走廊约束消费 | **保留作为任务 2 的 F 点兼容分配**；另给 TASK 03 导出跨区窗口及三车覆盖语义，不能仅靠此 index 表达单车跨 Portal |
| `optimizer.py::_build_corridor_constraint()`、`_build_casadi_nlp()` | NLP `corridor` 分支对 **F 点**采样位置加入半空间约束；已有三车 EDT 净空软代理 | TASK 02 保留 F-only 的原硬约束；只做必要的 Guide/候选输入接口配合。完整三车硬约束属于 TASK 03 |
| `geometry.py::map_spatial_state()` | 从 F 位姿、曲率及曲率导数映射三车中心、实际车身航向、车辆曲率及空间导数 | 复用现有物理变换及正常分支检查，避免自己另写一套矛盾公式 |
| `collision.py::audit_collision()` | 与既有 `FormationCollisionChecker`/原地图占据栅格联动的独立连续 OBB 终审 | 保持不动；TASK 02 可复用同底层的**静态**几何查询作为预评估，不能直接把终审 report 当候选证书 |
| `pipeline.py::_ProgressEmitter` 及 `ros_adapter.py` | TASK 01 已有只读 `progress_callback`，会携带 route/round、当前候选及身份 | 追加必要评估 metrics/diagnostic；不新建第二套 ROS 消息总线 |
| `formation_playback.py`（ROS-free）、`fwis_v3_playback/scripts/formation_playback.py`、`formation_demo.rviz` | 目前已有 A*/Guide、F 候选/车中心和候选 OBB 等图层，候选仅诊断 | 沿用已确定的四类路径颜色，增加少量必要的 Portal/集群预评估图层，避免密集 OBB/标签 |

**特别注意当前形状契约**：`VirtualARequest.vehicle_shapes` 由 `validate_virtual_a_request()` 验证，当前科研场景有固定的三辆车和尺寸要求。实现仍应**从请求/编队模型读取** offset、shape 和限制，而不是在新增算法里再次硬编码 `(1,0),(0,1),(0,-1)` 或 738×500 mm；固定尺寸仅可用于指定测试 fixture。

### 1.2 当前失败样例的重要事实

TASK 01 的旧/新同输入对照已经证明，S1 的 `corridor/hard` 失败 `CORRIDOR_AUDIT_WIDTH_LIMIT` 不是 TASK 01 新引入的回归。相关材料：

- `work_flow/TASK01_evidence/07_ros_rviz/diagnostics/README.md`
- `work_flow/TASK01_evidence/08_manual_review/README_corridor_containment_analysis.md`
- ROS/RViz 用户截图中 `plan 600000` 和 `plan 600001` 的失败表现。

该样例证明了两个**不同层面**的问题：

1. 原 F 点走廊并不要求偏移约 1 m 的车辆中心/OBB 也落在同一个凸区。蓝色候选出 F 走廊并不能单独证明撞真实墙，必须检查原始占据栅格。
2. F 点 NLP 在少量配点满足半空间（既有试验 4642 条约束、最大残差约 `9.99e-9 m`），连续 F 曲线仍出现亚毫米凸出，最终 fail-closed。这属于**现有连续 F 走廊约束和认证间隙**，主要由 TASK 03 处理；TASK 02 不得通过放宽 `corridor_audit_*`、`corridor_margin_m` 来冒充解决。

**用例约束**：TASK 02 要保留 S1 失败作为对照，并额外计算“原地图三车 OBB 静态样本是否碰撞”和“当前人工凸区并集覆盖是否充分”。不能根据屏幕颜色推断真实碰撞；也不能因 TASK 02 的静态预评估认为全程安全。

---

## 2. 任务目标、交付效果和非目标

### 2.1 核心目标

**O1——三车几何进入候选决策。** A* 仍产生有限 F 点拓扑路线，但正式进入 NLP 之前有可复现的编队位置/姿态/曲率估计、三车 OBB 与原地图静态检查、局部区域覆盖评估和候选优先级。至少能指出是哪辆车、在哪个候选弧长/路线站点、因何原因存在风险。

**O2——Portal 不被错误理解为单车矩形都必须完整进入同一块凸区。** 建立 `U_j = C_j ∪ C_{j+1}` 的有限过渡窗口表达，能判断固定状态下**完整旋转 OBB**是否由并集覆盖；并保留跨区域的车辆归属/覆盖见证，供 TASK 03 使用。

**O3——Guide 更有利于三车实际通过。** 在既有 QP Guide 基础上，利用三车几何与曲率估计做有限位置/姿态候选评估，输出连续展开航向建议、局部曲率建议、winding、路径长度 hint 和候选净空/覆盖质量，不盲目追求 F 点 Portal 几何中心。

**O4——研究默认入口立即可见、可测。** 完成后使用用户日常 `run_demo.sh`，能够看到新增评估数据用于**本次实际候选选择和求解初值**。失败时保留相同 plan 的 A*/Guide/集群诊断；历史模式显式可用。

### 2.2 本任务不做的内容（验收边界）

- **不做**三车 OBB 与相邻区域并集的光滑 NLP 硬约束；不新增三车连续走廊证书；不把 `is_three_vehicle_certificate` 改成 `true`。这两项属于 TASK 03。
- **不做**广域 SE(2) / 四维离散搜索、混合整数优化、通用路线管理器、新的车辆运动学模型、额外的货物 footprint、控制器改造。
- **不做** A* 启发式量纲修复、EDT/Ipopt 大规模缓存或性能基准竞争；那是 TASK 04。仅记录本任务新增耗时，防止严重拖慢。
- **不动** TASK 01 的 `g1.v2/v3` 历史模型重放、`S` 搜索、winding 的已批准语义、安全审查阈值、Playback 公共合同；不通过调低测试条件制造成功。

---

## 3. 数学设计：三车编队的真实几何与工作空间

### 3.1 参考状态与几何变换

F 的曲率路径按弧长 `s ∈ [0,S]`：

\[
\frac{dx_F}{ds}=\cos\psi_F,\quad
\frac{dy_F}{ds}=\sin\psi_F,\quad
\frac{d\psi_F}{ds}=\kappa_F.
\]

编队模型中车辆 `i` 的固定偏移 `r_i=(a_i,b_i)^T`。中心：

\[
\boxed{p_i = p_F + R(\psi_F)r_i}
\]

令 `A_i=1-b_i κ_F`、`D_i=A_i²+(a_iκ_F)²`，必须沿用当前模型的**正常分支条件** `A_i>0, D_i>0`。非退化时的实际车身方向为：

\[
\boxed{\theta_i=\psi_F+\operatorname{atan2}(a_i\kappa_F,1-b_i\kappa_F)}
\]

如果有 `η_F=dκ_F/ds`，车辆曲率由已有模型给出：

\[
\boxed{\kappa_i=\frac{\kappa_F+a_i\eta_F/D_i}{\sqrt{D_i}}.}
\]

本式依赖**曲率导数**，因此仅从折线的三点曲率估计出 `κ_F`，不能宣称已验证车辆曲率极限。`η_F` 未可靠估计时，对车辆曲率明确记录 `NOT_ESTIMATED/UNCONFIRMED`，不得无依据设为 0 后给出 `PASS`。

车辆完整旋转矩形为：

\[
B_i(q_F)=\{p_i+R(\theta_i)(u,v)^T:\ |u|\le L_i/2,\ |v|\le W_i/2\},
\quad B(q_F)=\bigcup_{i=1}^3B_i(q_F).
\]

几何基本条件是 `B(q_F) ∩ O = ∅`，并且所有 OBB 均在地图边界内，车辆彼此不重叠且运动学有效。`O` 是权威占据栅格转成的障碍集合，**unknown/gray 按既有碰撞规则处理，不得作为自由空间**。

### 3.2 现有凸区域 C 与配置可行域 K 的区别

当前 `ConvexCorridor`：

\[
C_j=\{p\in\mathbb R^2:A_jp\le b_j\}
\]

这里每行 `A_j` 已是单位外法向。构造 `C_j` 所用的自由空间审核和 `grid_sha256` 仍是权威事实。

配置空间仅作**隐式条件表达**，不建立全图四维网格：

\[
\mathcal K=\{(p_F,\psi_F,\kappa_F,\eta_F):
B_i(q_F)\subseteq\mathcal F\ \forall i,
A_i>0,\ D_i>0,\ \text{车辆曲率/运动学限制成立}\}.
\]

注意 `η_F` 对车辆曲率有影响。TASK 02 用当前 Guide 对若干固定配置状态做**近似的局部证据**，不是“已经精确求出了整个 `K`”。

### 3.3 单个凸区内的完整 OBB 半空间包含（用于单区见证）

对单位外法向 `n=(n_x,n_y)`，车身切向 `e_i=(cosθ_i,sinθ_i)`、横向 `e_i^⊥=(-sinθ_i,cosθ_i)`，其沿法向的支持半宽：

\[
h_i(n,\theta_i)=\frac{L_i}{2}|n^Te_i|+\frac{W_i}{2}|n^Te_i^\perp|.
\]

若**第 i 辆车单独归属 C_j**，完整 OBB 含入的充分且对矩形凸体准确的半空间条件为：

\[
\boxed{n^Tp_i+h_i(n,\theta_i)\le b-m\quad \forall (n,b)\in C_j.}
\]

`m` 为明确的候选几何裕量，不得自动套用任务 3 未确定的边界软代价参数。此式**按车辆 i 分别**判断，**不得推导成所有车辆在同一 `C_j`**。

### 3.4 相邻两块凸区的并集覆盖语义（本任务重点）

定义相邻区域局部自由并集：

\[
\boxed{U_j=C_j\cup C_{j+1}.}
\]

过渡处希望固定配置满足：

\[
\boxed{B_i(q_F)\subseteq U_j\quad(i=1,2,3).}
\]

这允许：

- 车 1 完全属于 `C_j`，车 3 完全属于 `C_{j+1}`，车 2 跨人工边界；
- **一辆车的完整 OBB** 同时占用两块区域的不同部分，只要其面积/边界没有越出并集；
- 编队参考点 F 不必在两个凸区的交集 Portal 中停留到“三车全部进入”为止。

**重要限制**：`U_j ⊆ 实际地图自由空间`，但局部并集往往 **小于**真实自由空间。因此 `OBB ⊄ U_j` 只能表示当前局部走廊的覆盖不足，不等于 OBB 与真实障碍相交。

---

## 4. 非凸并集下如何对固定 OBB 进行“完整覆盖”判断

这一节写给 Codex 是**必须实现的具体几何方法要求**，不能改成只测中心/角点。由于 `C_j∪C_{j+1}` 可为非凸，**矩形四角都在并集内，不推出矩形整体在并集内**。

### 4.1 推荐采用逐半空间裁剪的构造性覆盖检查

设车体矩形的世界坐标四边形 `P=B_i`，两个邻区分别为：

\[
C_L=\bigcap_{k=1}^m H_k,\qquad
H_k=\{x:n_k^Tx\le b_k\},\qquad C_R.
\]

从 `R₀=P` 开始，对 `C_L` 的每个半空间，按顺序计算：

\[
Q_k=R_{k-1}\cap\overline{H_k},\qquad
R_k=R_{k-1}\cap H_k.
\]

于是（把公共边界视为可覆盖，忽略重复边界的计数）：

\[
P\setminus C_L = \bigcup_{k=1}^{m}Q_k.
\]

每个具有实质外侧面积的 `Q_k` 是**凸多边形**，可使用既有 `_clip_polygon` 的同类半空间裁剪方法实现。若每个这样的 `Q_k` 的**所有顶点**均满足 `C_R` 的全部半空间（考虑保守数值误差），则：

\[
P\setminus C_L\subseteq C_R\Rightarrow
\boxed{P\subseteq C_L\cup C_R.}
\]

这个方法是**整个矩形的覆盖**，不是“只看原始四个角”。若 `P⊆C_L`，直接通过；若 `P⊆C_R` 同理通过；否则执行完整并集覆盖判定。

也可使用数学等价、可审查的稳定凸多边形布尔运算。但 Codex 不得以“对 OBB 四角分别判断 `inside(C_L) or inside(C_R)`”代替此条件。

**数值要求**：输入凸多边形/OBB 均必须有限且 CCW 合法，裁剪为仅有公共边界（零面积）的 `Q_k` 不能误当成实质外侧，退化及边界接触需有明确容差和 `UNCONFIRMED` 返回；仅用浮点面积误差 `area(P)-area(P∩C_L∪C_R)≈0` 不足以在微小切边处宣称**严格认证**。TASK 02 输出是固定样本几何评估，TASK 03 仍需独立、连续区间认证。

### 4.2 必须设计的四个几何单测（可构造解析真值）

1. **跨区可覆盖**：`C_L=[-2,0.3]×[-1,1]`，`C_R=[-0.3,2]×[-1,1]`，`P=[-0.5,0.5]×[-0.2,0.2]`。`P` 不完全属于任一个凸区，但完全被并集覆盖；应返回 `COVERED_BY_UNION`。
2. **四角误判反例**：`C_L=[-2,0]×[-2,2]`，`C_R=[-2,2]×[-2,0]`，P 为顶点 `(0,1),(1,0),(0,-1),(-1,0)` 的旋转正方形。四个原始角均属于并集（边界），但 P 在右上象限有内部未覆盖部分，必须返回 `COVERAGE_GAP`。
3. **单区包含**：P 完全位于 `C_L`；应成功且记录单区 witness，不必执行昂贵并集裁剪。
4. **边界/退化**：P 与两个区边界恰好相切、某一多边形裁剪为线段或面积接近零；不能用不受控的正容差导致漏检。可保守返回 `UNCONFIRMED_NUMERICS`，不能假装严格 PASS。

上述测试是**几何库的构造性例子**。实际规划链仍须由权威地图网格额外检查三车静态碰撞和地图边界。

---

## 5. 从二维 A* 到集群感知候选：不另起搜索器

### 5.1 保留现有拓扑路线生成

固定保留：

`make_route_guides() → A* 2D → up to 3 routes → simplify_route() → build_corridor_chain() → audit_corridor_chain()`。

当前 `RouteGuideConfig.max_routes=3`、`blocked_radius_m=0.15` 仅作为参考点路线搜索参数。不得在 TASK 02 把 `blocked_radius_m` 直接放大为“编队最大外接半径”：会过度保守，窄通道本来可由变姿态编队通过却被 A* 直接拒绝。

### 5.2 在**原有路由与走廊构造之间/之后**做有限两层评估

**预评估 A：原始/简化路线的粗风险。** 读取有序 `GuideRoute` 和 `simplify_route`，在端点、拐弯位置及均匀弧长站点估算 `ψ_F^g, κ_F^g`。计算三车 OBB 与原地图的静态关系；粗评估不能决定“该拓扑物理不可行”，只用于初步排序和风险提示。

**预评估 B：已有走廊和 Portal Guide 上的具体构型/覆盖。** 先按原有 `C_j` 构造和自由空间审核得到有效链、用原有 QP Guide 得到初值，再在每个 Portal 附近增加有限过渡站点，对 `B_i⊆C_j`、`B_i⊆C_{j+1}`、`B_i⊆C_j∪C_{j+1}` 及原地图 OBB 进行分开判断。此阶段可有限调整 Guide 的 Portal 点并重新评估，且不能破坏与当前 F-only NLP 的兼容。

这样避免为了“车辆几何”重复实现一个第二套 A*；候选处理应沿用 pipeline 的 route/round、预算、错误恢复和进度回调。

### 5.3 航向与曲率提示怎样获得

对非退化折线点 `p_0,…,p_M`，计算每条边方向：

\[
\phi_j=\operatorname{atan2}(y_{j+1}-y_j,x_{j+1}-x_j),
\quad \psi^g \text{沿路线连续展开}.
\]

必须接入 TASK 01 已有的 `HeadingBranch`/`_heading_branch_candidates`，**不再单独实现一个 winding 规则**。起终点的物理朝向要包含在可靠性判断里；短边、重合点、栅格阶梯转角和突变必须标记 `HEADING_HINT_UNRELIABLE`，不能作为高可信车体姿态。

局部曲率建议可优先复用 `optimizer.py::_node_route_curvature()` 的三点有向圆弧近似：

\[
\kappa_j^g\approx
\frac{2[(p_j-p_{j-1})\times(p_{j+1}-p_j)]}
{\|p_j-p_{j-1}\|\,\|p_{j+1}-p_j\|\,\|p_{j+1}-p_{j-1}\|}.
\]

退化三点或尖角不输出有效 `κ`；**本值只是 Guide hint，不是连续曲率路径**，不可以将数值跳变点直接作为一个瞬时可执行转向状态。对 `η` 的估计若不可靠，则对车辆实际曲率上限判断返回 `UNCONFIRMED` 而非 `PASS`。

### 5.4 OBB 原地图检查必须与“走廊覆盖”分开

每个样本站点至少输出**两组**状态：

1. `map_obstacle_state`：根据权威 `grid.occ`/地图边界，使用可复用的 `FormationCollisionChecker` 静态检查能力或等价的底层 OBB/占据栅格几何查询。必须同时考虑**车辆-地图**和既有**车-车**静态重叠规则。`MAP_COLLISION_OBSERVED` 说明该 Guide **此样本构型**有碰撞，不说明整条拓扑无解；`MAP_CLEAR_AT_SAMPLES` 只表示已检查站点清楚。
2. `corridor_coverage_state`：当前车体是否属于某一个 `C_j` 或合法相邻并集；若并集没有覆盖完整 OBB，标记 `COVERAGE_GAP`。若原地图同时显示该样本无碰撞，应输出 `CORRIDOR_COVERAGE_INSUFFICIENT`，而不是 `MAP_OBB_COLLISION`。

目前 `collision.py` 的终审器使用 `FormationCollisionChecker.static_check()`，且通过 `audit_collision()` 做完整模型、地图和数值闭合。TASK 02 可以复用**静态查询能力**或最小只读适配，但**不能调用完整 timed-plan 审核并把结果套到折线 Guide 上**；不能复制一套全局连续碰撞审查来替代它。

### 5.5 预评估结果的最小枚举与语义

建议使用**相互独立的字段**，而不是只有一个布尔 `feasible`：

| 字段 | 值/示例 | 严格含义 |
|---|---|---|
| `map_sample_status` | `CLEAR_AT_SAMPLES / COLLISION_OBSERVED / UNCONFIRMED` | 只针对已生成的有限候选姿态站点 |
| `coverage_sample_status` | `SINGLE_REGION / COVERED_BY_ADJACENT_UNION / GAP / UNCONFIRMED` | 是否被当前局部凸区（或并集）覆盖，不代表地图全局可行 |
| `kinematic_hint_status` | `NORMAL_BRANCH_OK / NORMAL_BRANCH_INVALID / CURVATURE_HINT_UNCONFIRMED` | 只对有充分信息的运动学性质做判断 |
| `guide_hint_status` | `VALID_HINT / LOW_CONFIDENCE / DEGENERATE` | 折线位姿/曲率提示的可信性 |
| `candidate_certification` | **固定为 `DIAGNOSTIC_ONLY`** | TASK 02 从不输出连续安全证书 |

风险记录包含 `vehicle_id`、`station_index`、`s_hint_m`、`pose/κ_hint`、`corridor_pair`、`nearest_obstacle_distance_estimate`（如存在）、`reason`。不存在的距离必须 `null`，不可用 0 或随意估算冒充真实净空。

**候选剔除原则**：一个有采样碰撞的 Guide 不一定意味着该 A* 拓扑不可行；最多降低候选优先级，或在有其它候选时延后尝试。除非存在**有严格证明**的结构性不可行条件，不得把有限采样碰撞变成“永远禁止该路线”；保留有限恢复和 `UNCONFIRMED` 语义。

---

## 6. Portal 过渡窗口与三车覆盖见证

### 6.1 过渡不是 Portal 内切圆半径

原 `P_j=C_j∩C_{j+1}` 只是二维 F 点位置上的交叠；内切半径只能反映点的通过余量。TASK 02 的真实过渡对象是 `U_j=C_j∪C_{j+1}` 对**三辆 OBB**的覆盖。

对于一个候选，在与相邻区域 `j,j+1` 相关的**有限 Guide 站点**上，计算：

- 每车 `single_region_ids`（可为一个、两个或空）；
- 每车 `union_cover`（`COVERED / GAP / UNCONFIRMED`）；
- 覆盖见证：由上述裁剪获得的 `Q_k` 是否全落 `C_{j+1}`，可记录必要的分片/索引或最坏违反量，而不存无限顶点数据；
- 参考点的 `portal_seed_point`、`sigma_hint` 和有限 `transition_window=[σ_a,σ_b]`；
- 三车**同时**完成静态覆盖的站点集合与具有风险的站点。

**一辆车既不完整属于左区、也不完整属于右区，但完整属于二者并集**，是一项必须支持的一级结果，不可把它压成 `NONE`。

### 6.2 推荐的有限窗口构建（任务 2 内可落地）

1. 使用既有 Guide 的分段弧长和 `allocate_corridors()` 给出的 `σ` 作为窗口**初始提示**，不重新增加一组自由优化变量。
2. 仅围绕每个实际相邻 Portal 建立有限站点/局部窗，并在这些站点对三车 OBB 做单区/并集和地图检查。
3. 根据相邻站点的覆盖结果，对窗口端点做**有限的局部细化/二分或固定细分**，只做候选表示而不是连续路径认证。
4. 保留 `source_region_index`、`target_region_index`、`guide_station_range`、`sigma_window`、`per_vehicle_witness`、`map/coverage statuses`、`uncertainty`；给 TASK 03 导出一致的数据合同。
5. 若窗口在有限复杂度内无法构造，返回明确的 `PORTAL_TRANSITION_UNCONFIRMED`；若地图静态安全、局部并集覆盖不足，返回 `CORRIDOR_COVERAGE_INSUFFICIENT`。**不得把未知推断成地图无路。**

**本阶段不要求**证明所有 `s ∈ [σ_a S,σ_b S]` 的 OBB 都已安全；这一连续验证属于 TASK 03。窗口的名字不能叫 `certified_portal`，避免误导。

### 6.3 如果现有凸区漏掉了地图自由空间

优先推荐的最小行为：

- 使用**地图 OBB 静态结果**把“真正与障碍冲突”和“只是人工走廊漏覆盖”分开，并将其作为诊断及后续候选选择依据；
- 可在同一候选中尝试**最多一次、可明确回滚的局部走廊补充/调整**，前提是复用现有 `_build_region`/`audit_corridor_chain` 且有地图自由空间依据；不能为了补覆盖直接放宽原工作空间障碍分离条件；
- 如果简单、可审核的局部补充不能成功，就保存 `COVERAGE_GAP` 并交给 TASK 03/后续研究，不无限自愈、不新建 corridor manager。

这里不要求一定引入新的多边形修补算法；Codex 应根据实际调用点说明**采用“纯诊断 + 备选候选排序”还是“一次局部补充”**，把成本/风险写清楚。两种都不得扭曲已批准的并集语义。

### 6.4 与现有 F 点 `allocation` 的兼容要求

**重要**：当前 `optimizer.py::_build_corridor_constraint()` 仍然把**每个 F 点 NLP 配点**固定归属一个 `C_j`。如果任务 2 产生的 Guide 只在 `U_j` 有效、F 本身不属于旧 `allocation` 对应的单区，直接将它喂给现有 solver 可能导致更严重的不可行。必须把两种信息分开：

- `legacy_f_allocation`：仍由 `allocate_corridors()` 返回，供 TASK 02 阶段当前 F-only solver 使用；
- `formation_transition_windows`：真实跨区候选/车辆覆盖信息，供后续 TASK 03 使用。

在 TASK 02 的运行中，新集群评分应当真正影响**候选排序和有限初值选择**；但若改进的 Guide 不满足 F 点现有半空间合同，必须明确回退到**同一个 `corridor` 模式的、经过原 F Guide 校验的初值**，并记录 `FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE`，绝不静默切换 `guidance_mode=legacy_route`。不允许为通过回归删除 F 点走廊硬约束。

---

## 7. 集群感知 Guide：有意义、有限且兼容现有曲率优化器

### 7.1 保留既有 Guide 的优势

`make_corridor_guide()` 目前通过相邻 Portal 内部点构造折线并提供 QP 引导。其位置线段处于对应凸区，因此是现有 F-only NLP 的良好初值。**不能为了集群指标把它整体替换为大规模非线性搜索器。**

### 7.2 推荐的局部改进方法

对每个 Portal 的原有点 `z_j`，沿该 Portal 交集/局部走廊允许的有限方向尝试若干**数量受限的候选位置**（例如基于内切中心与边界内侧的 3–5 个候选，不把这个数字认作性能承诺）：

1. 构造端点/Portal 折线候选，并检查 F Guide 每段仍满足既有 `C_j` 的凸性/半空间位置要求；
2. 估算各候选的 `ψ_F^g`、`κ_F^g`，检查正常分支并计算三车的实际 `θ_i`；
3. 对相应真实地图静态 OBB 与相邻并集覆盖做有限评估；
4. 先按风险类别排序，再比较归一化长度、局部地图净空、覆盖缺口和曲率 hint；
5. 保留当前 QP 原结果作为有来源的初值/对照。如果新候选不可靠或与现有 F-only allocation 不兼容，明确保留原 Guide，并输出原因。

**排序不建议一开始建立 10 个可调权重**。推荐先用可解释的多级顺序：

`已观察到真实地图碰撞?` → `正常分支风险?` → `走廊覆盖/不确定程度?` → `最小净空估计?` → `Guide 长度/局部曲率提示?`。

即使有碰撞样本，该路线**也可作为低优先级**尝试，因为具体 Guide 坏不等于拓扑路线绝对无解。为避免误拒，不能把上述排序字段当作正式硬安全约束。

若需要一个可测的辅助代价，可作为建议而不要求所有项同时落地：

\[
J_g=w_L\frac{L_g}{L_0}
+w_c\sum_{i,k}[\max(0,d_\text{pref}-\widehat d_{ik})]^2
+w_\kappa\sum_k\text{hintRisk}(\kappa_k,\eta_k)
+w_p\,\text{coverageRisk}.
\]

其中 `d_hat` 是有明确来源的**静态几何估计**，不是原地图全程精确最小净空。各项需单位归一化，权重不得拷贝 TASK 03 未来硬约束/软代价的参数。更小的词典序实现若达到上述候选选择目标则优先采用。

### 7.3 输出 Guide 提示，不增加一个权威“轨迹”

在既有位置 `GuideRoute.points` 之外，建议为当前候选保留：

- `heading_branch`：直接关联 TASK 01 已选分支（不可再算一套）；
- `heading_samples_rad`：路径上的**连续展开**角度提示及置信标记；
- `curvature_samples_1pm`、必要时 `eta_samples`：可用时填写，不可靠必须 `null/UNCONFIRMED`；
- `guide_length_m`、`S_upper_hint`：TASK 01 搜索域可以使用已有长度 hint 逻辑，不能引入另一套不一致的 S 截断；
- `sampled_vehicle_map_clearance`、`portal_union_coverage`：排序/诊断信息，不具备连续证书属性；
- `source_guide`、`selected_guide`、`guide_selection_reason`：使消融对照可追溯。

### 7.4 候选真正进入求解器

需要证明本轮“集群感知评估”不是只写诊断：

- route/Guide 的**尝试顺序或选定初值**必须至少有一处由真实的三车评估结果驱动；
- route 数量与 `max_initials` / 墙钟预算仍有原有上界，不进行无界 Guide 扩展；
- 给 `solve_open_path_guided()` 的位置 Guide 必须来自**这条候选**且可与旧 F-only constraint 使用；
- 记录 `route_index` 原始索引、实际 `attempt_order`、`round_index`、Guide 来源和评分原因，不能重新排序路线列表后让 RViz 显示索引错乱；
- 新候选在 NLP 中**没有三车硬约束**时，日志/HUD 必须清楚显示 `formation_obb_hard_constraints=false`，禁止虚假宣称已完成任务 3。

---

## 8. 建议的最小数据合同（TASK 02 → TASK 03）

### 8.1 优先扩展现有 round，不新建大框架

当前 `pipeline.py` 已在 `search['corridor']['rounds']` 中存：

`route_index, route_source, simplified_route, chain, chain_digest, chain_audit, guide, allocation, effective_margin_m`。

建议在这一轮下面额外附一份小型 `formation_assessment`，而非平行建一条算法流水线。推荐格式（示意，字段名由 Codex 核对现有 serializer 后定稿，含义不得丢失）：

```json
{
  "schema": "formation_corridor_assessment/v1",
  "candidate_id": "deterministic-id",
  "route_index": 1,
  "round_index": 0,
  "chain_digest": "<existing CorridorChain.digest>",
  "request_sha256": "<actual request digest>",
  "grid_sha256": "<actual grid digest>",
  "planning_context_sha256": "<actual context digest>",
  "guide_source": "original_qp_or_formation_refined",
  "guide_selection_reason": "...",
  "heading_branch_k": -1,
  "guide_length_m": 0.0,
  "legacy_f_allocation": [[0.0, 1.0, 0]],
  "transition_windows": [
    {
      "left_region": 0,
      "right_region": 1,
      "sigma_hint": 0.5,
      "sigma_window": [0.45, 0.56],
      "sample_witnesses": [
        {
          "sigma_hint": 0.5,
          "vehicle_id": 2,
          "coverage": "COVERED_BY_ADJACENT_UNION",
          "map_sample_status": "CLEAR_AT_SAMPLE"
        }
      ],
      "continuously_certified": false
    }
  ],
  "risk_summary": {
    "map_sample_status": "CLEAR_AT_SAMPLES",
    "coverage_sample_status": "COVERED_BY_ADJACENT_UNION",
    "kinematic_hint_status": "CURVATURE_HINT_UNCONFIRMED"
  },
  "candidate_certification": "DIAGNOSTIC_ONLY",
  "formation_obb_hard_constraints": false,
  "is_three_vehicle_certificate": false
}
```

**示例数字不是测试结果；示例摘要/ID/窗口不得硬编码。** 文档要求的是对应字段语义和验证边界。`candidate_id` 必须由本轮 request/route/round/chain/Guide 等规范化数据**确定性**构造，或使用现有可唯一追踪的 route/round 身份；不能以时间戳/随机数作为算法身份，不复写原模型 SHA。

### 8.2 数据一致性不变量（Codex 必须落实到验证）

1. `formation_assessment.request_sha256/grid_sha256/context_sha256` 与当次 admitted request 完全匹配；地图或模型变化即失效。
2. `candidate.route_index` 与 `routes[]`、RViz 选中的 raw A*/Guide、实际交给 solver 的 Guide 来源一致；选择/重试之后不能错把 0 当正在求解路线。
3. `transition_windows` 只能引用 `chain.regions` 中**相邻**的有效索引；`σ` 有限、有序、在 `[0,1]`；不可指向与当前 chain digest 不一致的区域。
4. 对每个 sample witness，必须可用请求模型重建 OBB 并再次计算对应静态覆盖结果；无 `θ_i` 来源不能标已验证。
5. `legacy_f_allocation` 仍由原 `validate_corridor_allocation()` 验证；新增 transition 不覆盖它的含义。
6. 旧的 `GuideRoute`/`CorridorChain` payload 和 `VirtualATimedPlan` 认证序列化保持兼容；新增候选诊断不能未经正式版本设计就被塞进模型 hash、旧 `v2/v3` 证书或 Playback 负载。
7. 可序列化诊断的采样数、marker 数、字符串长度应有**明确小上界**；不得把所有 OBB 面板/全局栅格作为每次 progress JSON 发布。

---

## 9. 工程接入范围和实施安排（给 Codex 定稿执行步骤的依据）

### 9.1 修改位置与期望接口

| 文件 | TASK 02 必要改动 | 刻意不做 |
|---|---|---|
| `formation_continuous_a/corridor.py` | 复用已有凸区半空间/顶点；新增**完整矩形在单区与相邻并集内的覆盖判定**、Portal 固定构型/有限窗口评估、必要的有限 Guide 候选计算；继续用已审核的链 | 不更改 `_audit_region` 的自由空间判据，不把并集假称凸区，不改 `audit_path_in_corridors` 的连续判据 |
| `formation_continuous_a/pipeline.py` | 在当前 A*/简化/chain/Guide/`allocate_corridors` 主流程中插入三车分析；评估用于排序和初值选择；把当前 route/round/候选评估挂到 `search`、现有 progress 回调；保持预算/失败恢复 | 不另写规划总入口、不引入自动 fallback 到 `legacy_route`、不无界候选循环 |
| `formation_continuous_a/contracts.py` | 必需时新增**一个**轻量不可变 `FormationCandidateAssessment` 或等价小合同、状态枚举和严格验证；保持 `GuideRoute`、`CorridorChain` 及旧反序列化语义 | 不改变物理 `VirtualARequest`/已认证 timed model 的 hash 契约 |
| `formation_continuous_a/geometry.py` | 原则上只复用 `map_spatial_state()` 及基础几何；确需纯函数时最小扩展 | 不改 Hermite 积分证书、`g1.v3` 数值语义 |
| `formation_continuous_a/optimizer.py` | **仅必要**的 Guide/排序 hint 消费与候选元信息记录；确保 TASK 01 的 winding 和 S 上界在新 Guide 下保持一致 | **不能**提前把三车走廊硬约束伪装为 TASK 02 已交付；不要重写 19 维 IPOPT |
| `formation_continuous_a/collision.py` / `formation_collision.py` | 尽可能不改；若静态 OBB checker 接口不好直接复用，允许增加小型只读 wrapper 且测试和终审行为不变 | 不动解析扫掠终审、Margin/容差和碰撞认证 |
| `formation_continuous_a/ros_adapter.py`、`formation_uniform_planner_node.py` | 沿用任务 1 的 progress callback，在现有 `planning_debug` 中携带限量可视化字段/状态 | 不创建新消息总线或改变可执行 envelope |
| `fwis_v3_optimizer/formation_playback.py`、`fwis_v3_playback/scripts/formation_playback.py`、`.rviz` | 仅为三车覆盖与 Portal 诊断添加必要 marker/标签，并复用当前计划身份/清图语义；尊重单类路径单颜色的已讨论规范 | 不新建第二个 RViz 节点，不把候选显示到 Certified path |
| 根目录 `run_demo.sh` / `formation_demo.launch` | **每项强制核查**默认仍为 `virtual_rigid_a/corridor/hard`，新增候选真实运行；只有实际新增参数才最小修改并保持研究默认启用 | 不改变单车 `--single`、旧显式 backend；不以临时手动参数掩盖默认未生效 |
| `tests/test_virtual_a_corridor*.py`、`tests/test_virtual_a_pipeline.py`、`tests/test_virtual_a_corridor_display.py` 等 | 加精准的覆盖/跨 Portal/错误语义/排序/身份/RViz 负例；利用现有 `tests/fixtures` 和 `evaluation` 对照 | 不建立平行测试框架或改历史期望数据来掩盖失败 |

### 9.2 推荐的**一个任务内**实施顺序

**Step A｜冻结基线（不改数学）**：记录最新 HEAD、工作区、既有 TASK 01 测试/审核状态、RViz 美化方案是否已合入、默认 run_demo 的真实参数、已知 S1 走廊失败。对当前测试地图生成输入 SHA 与旧行为报告。

**Step B｜几何核心及独立单测**：先做 OBB 顶点/支持函数/凸区和相邻并集的固定构型覆盖检查，验证四角误判、跨区、地图边界/退化；复用已有几何模型。先让这些测试可靠再接主 pipeline。

**Step C｜候选及 Portal 合同**：实现固定有限站点预评估、过渡窗口/多车覆盖见证、严格的状态/身份合同和日志，不碰求解器硬约束。测试模型/地图不匹配与安全状态不会被夸大。

**Step D｜Guide/实际调度接入**：有限集群评分/局部改进真正影响已有候选/初值选择，兼容当前 F-only `allocation`；记录原始 route index、attempt 顺序、winding、长度 hint。不能因 Guide 低可信就硬判无路。

**Step E｜RViz/HUD 与日常脚本**：只读进度显示已检查的实际三车几何、Portal 覆盖及风险/未知；不让旧 marker 与新 plan 混叠；检查并按需改 `run_demo.sh` 的默认实际算法。

**Step F｜离线回归 + ROS 实跑**：用固定正常场景、狭窄墙体反例、跨 Portal 正例、用户 S1 截图复现和旧模式对照完成验证；不允许只有 mock、单测或 `--dry-run`。

上面是**内部实施顺序**，不是另开 6 个独立开发任务。Codex 在 `TASK02_Codex执行方案.md` 中应把每一步细化到**修改函数、参数、数据结构、伪代码、测试命令和停工条件**，Claude 按此执行，不要求 Claude 自己重选数学方向。

---

## 10. RViz 实际效果：复用现有图层，保留用户已选配色

用户之前已经选定：**A* 橙 `#F97316`，Portal Guide 绿 `#22C55E`，未认证候选黄 `#FACC15`，Certified 最终轨迹亮蓝 `#38BDF8`**；相同算法类别三车同色，不用三个车号三种线色。视觉规范文件为：

`work_flow/RVIZ_路径分类配色与图层精简配置方案_V1.0.md`（若用户尚未放入仓库，Codex 应向用户报告，不得假定已经执行）。

### 10.1 TASK 02 必须增加而不改变路径配色语义的诊断

1. **三车候选 OBB**：在有真实候选位置/曲率来源的少量站点展示灰蓝细轮廓，每车默认少量代表姿态；不再满屏绘制 80 个密集车框。绝不能把候选车辆框绘制成“已认证”蓝线。
2. **Portal 过渡位置**：用小而清晰的点/短线或低透明区域显示 `C_j`/`C_{j+1}` 交集和有限过渡范围；只标当前 route/round，必要时一条短文 `Portal j→j+1 | union-cover at samples`，不在图中铺满顶点编号。
3. **真实地图碰撞样本/走廊覆盖缺口**：若有明确车辆、站点与地图静态碰撞证据，洋红叉标 `MAP COLLISION (sample)`；仅走廊覆盖不足则用**不同状态词**标注 `CORRIDOR COVERAGE GAP`，绝不把两者共用 `MAP_OBB_COLLISION` 文案。
4. **HUD**：在当前 `CORRIDOR_BUILD`/`PORTAL_GUIDE` 阶段追加 `formation samples`, `map collision samples`, `union-covered portal`, `coverage gap`, `current route/round` 等有限真实指标。没完成的阶段不填虚构百分比/安全结果；不必新建大量阶段枚举。
5. **身份与生命周期**：保留任务 1 的 plan/generation/request/grid/context SHA 校验，拒绝旧消息；新准入清旧图，FAILED 保留本 plan 的最新诊断，2D Pose Estimate/reset 清历史图层；busy reject 不清当前计划。

### 10.2 默认用户操作

用户实际命令保持简单（地图可按当前需求替换）：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_demo.sh --map map05 --scale 2.625
```

正常情况下不需要再手动补 `backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard rviz:=true auto_goal:=false`。**Codex 必须核查当前脚本真实启动语义**；若 RViz 在当前环境需要显式 `rviz:=true` 才开启，说明并在用户确认后最小改研究默认配置，不能在测试时隐蔽增加参数却声称默认满足。

**验收时要能亲眼看到**：A* 橙、Guide 绿、候选黄、完成全审才出现的亮蓝 Certified；三车候选 OBB、Portal 过渡覆盖/未知；右上角当前阶段、路线 ID、样本风险；失败时保留证据。不要求强迫每个测试场景都成功发布一条最终路径。

---

## 11. 验证与验收：至少覆盖六类固定反例/回归

**测试组织原则**：离线几何/接口/ROS mock 测试由 Claude 先跑；GUI/ROS 真实实验必须在用户本机完成并留下真实终端日志、topic 样本、屏幕图。Codex 必须在执行方案中给出具体可运行命令及所需 ROS 环境。**任何不能运行的项标 `NOT_RUN` 并解释，不能标 PASS。**

| 编号 | 场景 | 要求证明的点 | 合格证据/预期 |
|---|---|---|---|
| G1 | 宽敞直线、三车几何不受限 | 不能因新预评估无故变慢或拒绝；Guide 与原 solver 兼容 | `MAP_CLEAR_AT_SAMPLES`、有效 `candidate_id`、正常 F-only 求解；有真实 run_demo 轨迹时完整终审仍生效 |
| G2 | F 点路线可过，但某车 OBB 对原地图的静态样本明显撞墙 | 新算法能区分 F 净空和**实际车身**净空；风险精确定位到车辆与站点 | 不应把这条候选标集群安全；可降低优先级/尝试另一 Guide；没有合格候选时不发可执行信封 |
| G3 | 两相邻区的并集覆盖矩形，但任一区单独都不覆盖 | 能正确处理单辆车跨人工边界；三车可分别归属不同区域 | 合理 `union_cover=true` + per-vehicle witness；不强制三车同属一个区 |
| G4 | 四角都在并集、但 OBB 内部有缺口（第 4.2 节具体反例） | 不能用 4 角检查当完整覆盖 | 必须 `GAP/UNCONFIRMED`，不能误 `COVERED` |
| G5 | U-turn/急弯 + 大偏移 | `θ_i` 根据 `ψ_F,κ_F,a_i,b_i`；车辆曲率条件依赖 `η_F` | 几何单测/数值对照：与 `map_spatial_state` 在相同状态下相符；低可信曲率提示不得被标安全认证 |
| G6 | 走廊并集没覆盖车体，但权威地图静态 OBB 实际无碰撞 | “人工区域漏覆盖”和“真实障碍碰撞”分开 | `CORRIDOR_COVERAGE_INSUFFICIENT` 而非 `PHYSICALLY_INFEASIBLE`；候选身份/route 仍可追溯 |
| G7 | S1 旧问题 `plan 600000`/用户截图反例 | 核查任务 1 已出现的 F-only 与车体外伸、离散配点与连续审核间隙；不得靠降阈值美化 | 对照输出新 `formation_assessment`，保留旧 `CORRIDOR_AUDIT_WIDTH_LIMIT` 如仍存在，不声称 Task 02 已修复连续认证 |
| G8 | `600003 / 600004` U-turn 和旧后端 / 单车入口 | TASK 01 winding/S/数值证书不回归；显式 `legacy_route`/`uniform_v02` 与 `--single` 可用 | 同一请求/地图和配置的完整回归，若性能/求解状态变化需说明原因，不能只报一个 SUCCESS |

### 11.1 必备的反例和属性测试细则

- `geometry`: OBB 支持函数与显式四角最大投影一致；单位外法向旋转/不同车身角度均成立；三车中心固定偏移满足刚性关系；归一化、边界接触和无效 `A_i` 分支检查。
- `union`: 第 4.2 节四个解析用例 + 顺序对调 `C_L/C_R` 应等价、所有车独立 witness 和一辆车跨两区、三车分处两区；纯数值退化不得静默误 PASS。
- `integration`: 新增风险信息真实改变某次候选排序或选中的 Guide，而非仅写入日志；route-index/active A*/Guide 和 solver 输入来源一致；同一候选多次运行 deterministic；`max_routes/max_initials/soft budget` 无突破。
- `negative`: grid SHA、request SHA、context SHA、generation、route 索引、Portal 相邻关系、样本站点越界/NaN、非法 OBB 尺寸、过时 progress callback、busy request；不得污染当前计划或已认证播放。
- `safety`: 诊断层状态再好，也不能绕过既有独立几何、动力学、解析连续 OBB 终审；故意制造错误候选，不能出现 `PUBLISHED`。
- `compatibility`: 旧 `GuideRoute`、`CorridorChain`、`PlannerConfig()` 历史默认/序列化与 TASK 01 的 `g1.v2/v3` 重放不回归；若诊断 schema 新版需独立版本，不能修改原 request/model sha 定义。

### 11.2 仓库现有回归入口（Codex 逐项核对实际环境）

在本地仓库中（示意，执行前需 source ROS Noetic 对应 workspace）：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"

# 优先针对性执行，Codex 可按新增测试类拆分
python3 -m unittest tests.test_virtual_a_corridor tests.test_virtual_a_corridor_optimizer
python3 -m unittest tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery
python3 -m unittest tests.test_virtual_a_corridor_display tests.test_virtual_a_receiver
python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_collision
python3 -m unittest tests.test_virtual_a_optimizer tests.test_virtual_a_integration_bounds
python3 -m unittest tests.test_virtual_a_playback tests.test_virtual_a_trajectory

# 编译与入口校验
catkin_make -C catkin_ws -j4
./run_demo.sh --map map05 --scale 2.625 --dry-run
# 上面 dry-run 不是 ROS 真机验收；下列才是 GUI 现场入口
./run_demo.sh --map map05 --scale 2.625
```

**历史已知测试失败不要掩盖**：TASK 01 开发交付中全量回归曾报告非本次改动的 `test_continuous_retiming` `KeyError: 'dynamic_peaks'`；人工审核后 legacy smoke 曾有 `marker_status_feasible` 条件失败。Codex 应以本地当前 HEAD 重新核对，记录“旧存在/本次新增”的证据，**不得自行更改无关测试或降低安全断言**。

### 11.3 必须保留的实验证据

每个场景最少：

`git HEAD + 工作区状态 + 地图实际 SHA + request/context SHA + backend/guidance/enforcement + candidate/route/round/chain digest + 原始/改进 Guide + 样本站点 + OBB map/union 风险摘要 + 优化器真实消费的 Guide/排序信息 + NLP 状态/约束计数 + geometry/dynamics/collision audit + 发布时间/是否 executable + 各阶段 wall`。

如有真实 ROS：再保存 `/formation/planning_debug`、`/formation/job_state`、`/formation/quality_report`（实际存在的话题名称以 `rostopic list` 为准）、相关 path/Marker topic 样本、原始 `run_demo.sh` 命令、RViz 截图。候选失败时**可执行信封 0**；如果保留用于清理 latched topic 的**空占位信封**，记录它和可执行信封是不同计数，不能因为 raw topic 有占位而说安全发布了候选。

**与基线比较**：成功案例报告 `runtime/route_count/attempts/S/OBB`；失败案例给最早实质失败层和预算终因。没有充分实测不能写“成功率提高 X%”或“已连续认证”。

---

## 12. 开发与审核闭环：本任务必须产出的四份文件

**统一目录**：`/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/`

| 角色 | 产物 | 内容 |
|---|---|---|
| 用户 + ChatGPT（本文件） | `TASK02_技术方案.md` | 固定算法目标、数学/数据语义、范围和验收原则 |
| **Codex（第一轮，只设计不改代码）** | `TASK02_Codex执行方案.md` | **直接给 Claude 的详细执行合同**：最新 HEAD/差异核查；函数级接口与数据结构；精确伪代码；改动顺序/位置；单测/场景构造；ROS/RViz 操作；交付证据格式；每步停工/回滚条件 |
| Claude（按方案实施） | `TASK02_Claude开发交付.md` | 实际提交 SHA、按文件 diff 说明、真实运行环境/命令/退出码、测试通过/失败、每类反例日志、截图与产物索引、未解决风险；不能只说“全部实现” |
| **Codex（第二轮，独立审核）** | `TASK02_Codex审核报告.md` | 复核 Claude 源码与测试，按本技术方案逐项打 `PASS/FAIL/BLOCKED`；检查 OBB 并集完整覆盖、有效候选/Guide 进入默认 solver、回归、未认证隔离、实际证据；失败则同一任务返修 |

此外用户可自行写 `TASK02_RViz验收记录.md`（可选第五份**个人验收记录**，不是第五个开发任务）。本任务的机器/GUI 证据统一放：

`work_flow/TASK02_evidence/`

推荐少量逻辑子目录 `baseline/`, `geometry/`, `candidate/`, `ros_rviz/`, `review/`，不强制几十个冗余阶段目录。禁止覆盖 `TASK01_evidence/` 或历史 `evaluation/`。

### 12.1 交给 Codex 时的明确工作指令

> 你现在承担 **TASK 02 的执行方案设计者**，而不是直接开发者。请先阅读本文件、最新 `work_flow/TASK01_*`、现有 `TASK01_evidence` 和实际 Git HEAD；必要时核查用户是否已并入 RViz 配色方案。
>
> 先确认项目的代码、测试、启动脚本、Portal/Guide/F-only NLP 接口，并特别审查“完整旋转 OBB 属于相邻两个自由凸区的并集”的几何算法和退化数值条件。按本文已批准方向形成**一份独立、可直接交给 Claude 的 `TASK02_Codex执行方案.md`**；写出具体函数签名、数据合同、伪代码、调用位置、严格失败/未知状态、测试 fixture 构造及命令、验收证据结构、回滚策略。要足够详细，Claude 不需要知道我们此前对话。
>
> **目前只输出设计文档，不要修改生产代码，不要开始 TASK 03，不要自行降低硬约束/认证阈值。** 如果某项数学要求与当前 F-only solver 存在冲突，必须显式设计过渡兼容和留给 TASK 03 的接口，不能偷偷改成“各车完整位于同一 C”，也不能把未认证静态样本标为连续安全。
>
> 用户批准你的执行方案后，才由 Claude 实施，并由 Codex 最后独立审核。用户会在真实 RViz 中对实际运行效果再做人工验收。

### 12.2 Codex 执行方案必须回答的十个具体问题

1. **几何接口**：具体用哪个已有函数构造 `p_i/θ_i/OBB`？如何与 `map_spatial_state()` 一致、如何处理 `η` 未知和正常分支失败？
2. **完整并集判定**：第 4 节裁剪算法的函数签名、输入、容差、退化处理、复杂度、对照测试是什么？如何保证不是只测四角？
3. **原地图检查**：复用哪个实际静态碰撞查询接口？如何区分车辆-墙、车辆-车、地图外和局部走廊漏覆盖？
4. **有限采样**：在哪些站点检查、怎样覆盖 Portal 和拐弯、如何限制复杂度和在 UNKNOWN 处停止？
5. **过渡数据**：`transition_windows` 与原 `allocation` 的区别是什么？如何表示单辆车**跨越两个区**而不被迫归属一个区？
6. **Guide 优化**：有限几何扰动/排序如何保证原 `GuideRoute` 合法？哪一个实际决策因此改变，如何证明不是只增加日志？
7. **求解器兼容**：新候选不能满足旧 F-only `allocation` 时怎样做？如何保留 `corridor/hard` 和原终审而不静默变回 `legacy_route`？
8. **身份与版本**：如何保持 request/map/context、route/round、候选 ID、诊断消息、历史模型 hash 的一致性？
9. **可视化**：怎样在不新增 UI 框架的条件下显示三车样本、Portal 并集覆盖和风险/未知；如何与用户四类路径配色一致？
10. **真实验收**：在哪几个确定性测试 fixture 和 ROS 地图上复现，如何证明新候选真实进入默认 `run_demo.sh` 的求解和候选选择、仍不宣称 TASK 03 的三车硬约束？

---

## 13. 任务完成条件（不可降级）

只有同时满足以下条件，Codex 才能建议本任务通过自己的代码审核：

1. **静态几何能力存在且可测**：所有车辆几何从统一模型读取，OBB/支持函数与模型一致；相邻凸区并集覆盖有完整多边形判断，四角误判的反例不再漏过。
2. **候选真正集群感知**：已有二维 A* 保留、Route/Guide 不被替换成另一个大搜索器；三车地图风险和 Portal 并集覆盖实际影响至少一项有限候选/Guide 决策，并可由日志、测试和 ROS 现场证明。
3. **Portal 合同不偷换**：允许三车位于相邻区、单车完整跨人工边界；新合同与旧 F-only allocation 分开，能给 TASK 03 直接使用。
4. **失败语义正确**：真实地图 OBB 冲突、走廊覆盖缺口、低可信航向/曲率和数值未知分开；不能把采样安全称为连续安全/物理必然可通行。
5. **默认运行可观察**：用户直接启动 `./run_demo.sh --map map05 --scale 2.625` 等固定入口，实际是最新 `virtual_rigid_a/corridor/hard`、显示新三车评估与 Portal 信息；不只是 `--dry-run` 或离线 demo。
6. **安全不回归**：现有连续 F 走廊认证、几何/动力学/三车原地图 OBB 审核、Playback 信封身份、TASK 01 winding/S/数值版本与历史旧后端均保留。
7. **交付闭环真实可核验**：Claude 的完整交付文档 + 原始证据，Codex 的**独立**审核报告；未运行标 NOT_RUN，已知旧失败不悄悄删除，用户最后有 RViz 人工验收机会。

**本文件的核心结论**：TASK 02 的成果是**三车几何感知的候选空间、相邻自由并集 Portal 表达和更优 Guide 初值**，必须运行于用户实际默认研究路径，但**绝不是**已经完成三车 OBB NLP 硬约束或连续可执行认证。后者属于 TASK 03。只有保住这个边界，四任务研发才能持续可对照、可复现且不会靠“画得更好看”误认为算法已安全。
