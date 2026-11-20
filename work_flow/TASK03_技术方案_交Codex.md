# TASK03 技术方案——三车 OBB 约束、Portal 跨区通行与连续轨迹认证（提交 Codex）

> **版本**：V1.0 技术需求稿（用户 + ChatGPT → Codex）。**不是 Claude 直接开始修改代码的执行指令**。Codex 必须先读最新源码，编写 `work_flow/TASK03_Codex执行方案.md`；该方案由用户确认后交 Claude 执行，Claude 输出 `work_flow/TASK03_Claude开发交付.md`，Codex 再输出 `work_flow/TASK03_Codex审核报告.md`，最终用户在 RViz 进行人工验收。
>
> **前置门槛**：先完成 `work_flow/TASK02_定向审核方案_交Codex.md` 规定的独立审查。Task 2 有实现缺陷则先返修、复审；证据不足先补足必要事实。**Task 3 不承担替 Task 2 修评分错误或补伪证据的责任**。
>
> **本地仓库**：`/home/wq/whut_thesis/expv3_multi_virtual_a`；分支 `feature/virtual-rigid-formation-a`；本文核对远程 HEAD `a3c709e57560b1060dcc422777f4131a3e558acf`。Codex 编写执行方案前重新记录本地 `HEAD`、dirty 状态与已合并的 Task 2 修改，不得覆盖用户本地未提交文件。
>
> **统一目录**：全部技术/执行/交付/审核文件放 `work_flow/`；本任务运行输出放 `work_flow/TASK03_evidence/`，不要污染 TASK01/TASK02 冻结证据。

---

## 1. 最终研发目标和工程边界

### 1.1 具体要解决的问题

目前连续路径优化仍然只把虚拟参考点 **F** 的有限离散位置约束在二维凸走廊里。Task 2 增加了真实三车 OBB 的有限静态检查、基于碰撞的路线与 Guide 排序以及相邻 Portal 并集评估，但**没有让三车几何约束进入 IPOPT**。因此出现：

- Guide 有 `MAP_OBB_COLLISION` 风险标签，优化器仍有可能生成车辆压到真实墙上的**未认证候选**。
- 车辆可以合理分布在相邻两块凸自由区域中，但原 F-only 单区分配及连带审核可能拒绝合法跨区运动。
- NLP 离散配点满足走廊硬约束，**中间连续弧长**仍可能越界，导致 `CORRIDOR_AUDIT_WIDTH_LIMIT` / `CORRIDOR_CONTAINMENT_VIOLATION`。
- 已有原地图三车连续 OBB 终审只在上游其他审核通过后发生，优化器本身尚未主动满足完整三车几何可通行约束。

**Task 3 要使三车几何通行成为真正的优化约束，使优化后的连续曲线通过独立几何证明和原地图终审；不能再仅依赖 Guide 采样净空或最后碰撞筛选。**

### 1.2 固定边界

- 研究模式：已成刚性编队的 **三辆 4WIS 车辆，A（前/后阿克曼）连续曲率规划**；不开发形成编队、车货动力学、其它 O/S 模式，也不重写下位控制。
- 保留 **现有 IPOPT/CasADi、8 段 Hermite 曲率（先不改变 19 变量）**、任务 1 winding / S 搜索域 / 位置数值证书、车辆曲率和现有时间参数化。只有固定反例证明现有分段/分配不足时才能做有限局部调整。
- 保留 Task 2 的二维 A*、工作空间 `ConvexCorridor` 链、完整 OBB 静态评估、相邻并集 Portal 语义及候选生成；不重写 SE(2) 搜索。
- 原有独立几何、动力学、**真实地图三车连续 OBB** 终审及 `/formation/playback_plan` 身份与认证门控不得降低、不得绕过。
- 不把未知载荷 footprint 当作已检查对象。已知车体模型以权威请求和审核模型为准。
- 不做未经证明的全局混合整数规划、无限候选枚举、通用缓存/manager/factory、新规划器、单纯放宽阈值。

### 1.3 完成后必须同时成立

1. **实际 NLP 的 `g(z)` 中存在每辆车完整 OBB 的可计算安全约束**（或可证明充分的覆盖模板），而非仅 F 点。
2. 跨 Portal 时允许车辆属于不同相邻区域，以及单辆车跨人工分割边；人工边界不得与真实墙面混淆。
3. 独立于有限 NLP 配点，对**整条连续弧长轨迹**核验配置空间通行，无法证明则返回未认证，不能发布。
4. 原地图独立三车连续 OBB 终审仍然是必须通过的最后安全关口。
5. 默认 `./run_demo.sh` 真实走研究版新算法；RViz/HUD 能区分旧 F-only、新三车集群硬约束、候选、连续审核与真实碰撞状态。
6. 失败有确定性原因及可复现证据；不为了制造成功而隐藏碰撞或降低审核精度。

---

## 2. 当前源码定位（Codex 必须按最新 HEAD 重新核对）

核心目录：

`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`

| 文件与函数 | 当前事实 | Task 3 的职责 |
|---|---|---|
| `optimizer.py::_build_corridor_constraint()` | 从 `CorridorChain` + 固定 F-only `allocation` 生成 `groups[j]`，配点只约束 F 位置 | 生成新的**集群约束编译结果**：车辆/区域/过渡模板/采样对应关系，保留旧模式以便对照 |
| `optimizer.py::_build_casadi_nlp()` | 现有 `corridor_constraints` 为 `n·p_F-b`；另有三车曲率约束及 EDT 净空**软**代价 | 使用同一 `p_F,ψ_F,κ_F` 计算每辆车 OBB 顶点/支持函数，在 CasADi `g(z)` 中追加硬约束；计算可解释的新软净空目标 |
| `optimizer.py::solve_open_path_guided()` | 按给定 chain/Guide/allocation 建图、求解、存残差与候选 | 接受同源的有限集群模板、控制有限切换/切约束重求解，并完整记录约束来源/残差 |
| `corridor.py::formation_transition_windows()` / `assess_formation_guide()` / `obb_coverage_in_adjacent_union()` | 提供局部双区 `C_j∪C_{j+1}` 的**离散采样、完整 OBB 静态覆盖**诊断；未做连续证明 | 作为模板候选与其初始几何事实输入；**不能**直接把采样诊断标为证书 |
| `corridor.py::allocate_corridors()` / `audit_path_in_corridors()` | F 点固定 sigma 单区分配与 F 点连续审核；后者可能返回 `CORRIDOR_AUDIT_WIDTH_LIMIT` | 保留历史审查能力；新增/显式采用版本化的集群连续审核；明确新研究模式的门控语义 |
| `pipeline.py::_plan_virtual_a_impl()` | 组装路线、Guide、NLP、F containment、几何/动力学/真实 OBB 终审；已有有限恢复/预算 | 把**同一个候选**的集群模板送入求解，审查未通过则有限调整；记录失败层、关联 identity；不暗中切回 legacy |
| `audit.py` / `collision.py` | 数值/动力学与真实地图连续 OBB 独立终审 | 保持原有判定与证书；新集群走廊证书独立实现，不替代原地图检测 |
| `contracts.py` | request、配置、Route/Guide/CorridorChain 契约及历史 SHA | 最少必要的新模式、模板和认证结果契约；不得无声修改旧模型、旧序列化语义 |
| `formation_playback.py`、ROS planner/playback node、`.rviz` | Task 1/2 的 A*/Guide/候选/Portal 风险/HUD 已有 | 标明当前求解器约束版本、离散/连续审核结论和真实失败位置，不增加另一个可视化系统 |
| `run_demo.sh`、`formation_demo.launch` | 研究入口默认 `virtual_rigid_a / corridor / hard / auto_goal=false` | 每次开发完成验证**新模式真的生效**，不允许默认仍跑 Task 2 的仅采样诊断模式 |

任务 2 源码的 `formation_obb_hard_constraints=False` 当前是正确的历史语义；Task 3 只有**真正生成/求解并完成对应验证**才能把新模式的该字段置为 true。

---

## 3. 数学模型：从 F 点轨迹到三车完整 OBB

### 3.1 轨迹参数化

连续路径以弧长 `s∈[0,S]` 为参数：

\[
\frac{dp_F}{ds}=
\begin{bmatrix}\cos\psi_F\\\sin\psi_F\end{bmatrix},
\qquad \frac{d\psi_F}{ds}=\kappa_F(s),
\qquad \frac{d\kappa_F}{ds}=\eta_F(s).
\]

保留 8 段三次 Hermite 曲率和现有 `z=[S,κ_0…κ_8,η_0…η_8]`。计算 `x_F(s),y_F(s),ψ_F(s),κ_F(s)` 复用 optimizer 的**同一条**积分实现，禁止为 OBB 另做一套不一致的轨迹近似。

### 3.2 三车位置及车身朝向

对车辆 `i` 固定偏移 `r_i=(a_i,b_i)`，令

\[
A_i=1-b_i\kappa_F,\quad B_i=a_i\kappa_F,\quad D_i=A_i^2+B_i^2.
\]

\[
\boxed{p_i=p_F+R(\psi_F)r_i},\qquad
\boxed{\theta_i=\psi_F+\operatorname{atan2}(B_i,A_i)}.
\]

必须校验 `D_i` 在使用区间内具有安全正下界；如果速度切向退化 / 车身方向无法定义，则按照当前运动学合同报告数值或运动学不可确认，不允许 `atan2(0,0)` 继续生成假角度。

车辆 OBB：

\[
\boxed{
\mathcal B_i(q_F)=\{p_i+R(\theta_i)u:\ |u_x|\le L_i/2,\ |u_y|\le W_i/2\}
}
\]

`L_i,W_i,r_i` 全部来自与独立审核一致的模型/形状；在实现前，Codex 需核对生产 collision checker 是否使用预设研究尺寸并与 `request.vehicle_shapes` 验证一致。

### 3.3 真实安全、自由走廊与 Portal 语义

地图自由集：`𝓕 = map_domain \ obstacles`；`C_j={p : A_j p ≤ b_j}` 是已通过地图无障碍检查的二维凸自由区，局部双区并集：

\[
U_j=C_j\cup C_{j+1}\subseteq\mathcal F.
\]

期望的集群通行条件为

\[
\boxed{\forall s\in[0,S],\quad \mathcal B_i(q_F(s))\subseteq U_{j(s)},\ \ i=1,2,3}
\]

（非过渡段可以只有一个 `C_j`；另需满足各车曲率限制。）对于 F 参考点，可单独要求处于相应工作空间自由覆盖中，但**不能要求所有三车同时进入同一个凸区**。

**关键**：一般来说 `U_j` 非凸，`B_i⊂U_j` 不能靠各角点属于并集证明，更不能将 `C_j` 和 `C_{j+1}` 的所有半空间一起相交施加给整辆车。Task 2 的完整 OBB/并集静态检查只提供候选与 witness，不是可直接用于 IPOPT 的可微 hard constraints。

---

## 4. 核心优化手段 A：单区域的三车 OBB 硬约束

对单位外法向 `n=(n_x,n_y)`，车辆方向 `θ_i` 的 OBB 支持半宽为

\[
\boxed{
h_i(n,\theta_i)=\frac{L_i}{2}|n^T(\cos\theta_i,\sin\theta_i)|+
\frac{W_i}{2}|n^T(-\sin\theta_i,\cos\theta_i)|
}
\]

若车 i 完整归属凸区 `C_j`，其整车包含充分且必要条件为

\[
\boxed{n^Tp_i(s)+h_i(n,\theta_i(s))\le b-m_i
\quad\text{对 }C_j\text{ 的全部半空间}}.
\]

实现注意：

- `fabs` 在零投影处不可导，不能未经处理直接假定 IPOPT 总是稳定。**推荐用四角顶点约束**代替绝对值支持函数：对车辆四角 `u_v∈{(±L_i/2,±W_i/2)}`，逐个添加
  \[
  \boxed{g_{i,j,v,n}(z)=n^T[p_i(z)+R(\theta_i(z))u_v]-b+m_i\le0.}
  \]
  对凸多边形这与完整 OBB 包含等价，`sin/cos` 可微；代价是约束条数增加。Codex 应给出两种表达的约束数、建图耗时和导数行为对照，**默认优先有限角点法**。
- 保留已有 `F` 的位置/航向曲率连续性、终点/车辆曲率约束；新约束不能代替它们。
- 安全裕量 `m_i` 必须有明确的几何基线、数值误差闭合与单位；不擅自通过减少 margin 让求解成功。
- 每个约束必须可追溯：`vehicle_id`, `region_id`, `halfspace_id`, `vertex_id`, `sigma`, `curve_piece`, `template_id`，及 `actual_nlp_constraint_count`。

---

## 5. 核心优化手段 B：跨 Portal 的有限安全覆盖模板

### 5.1 不采用错误的两种做法

- **错误 1：**所有车必须完整属于同一个 `C_j`。这会误拒不同车分布在相邻区的安全过渡。
- **错误 2：**只要四角属于 `C_j∪C_{j+1}` 就视为安全。并集可能非凸，矩形边/内部可能在并集之外。

### 5.2 推荐主实现：有限 OBB 凸分片 + 外部离散归属

由于 `U_j` 非凸，推荐通过**有限个已验证的充分约束模板**让 IPOPT 仍处理连续变量，而不是把整套离散区域选择放到求解器里。

1. 在车辆车体局部坐标系，把完整矩形 `B_i^local` 拆成有限个**覆盖整个矩形、互不遗漏内部**的凸片 `T_{i,k}`。最简单的模板是沿矩形两种对角线分别拆成两个三角形；对特殊跨区场景可以构造少量固定 strip/triangle 模板。模板均为**真实完整覆盖的分片**，不是车辆角点稀疏抽样。
2. 对每个固定归属模板 `τ`，在某个连续弧长段上为**每辆车的每个凸片**指定一个相邻自由凸区 `C_j` 或 `C_{j+1}`；不同车辆、同车不同凸片允许不同归属。
3. 在 NLP 的每个相关约束配点 `s_k`，对属于片 `T_{i,k}` 的全部顶点 `u_v` 强制：

\[
\boxed{
\forall n_h\in A_{a(i,k)}:\quad
n_h^T[p_i(s_k)+R(\theta_i(s_k))u_v]\le b_{a(i,k),h}-m_i.
}
\]

由于 `T_{i,k}` 与 `C_{a(i,k)}` 都凸，**每片所有顶点在其归属凸区内，保证整片在内**；所有片覆盖整车，因此严格推出：

\[
\mathcal B_i(s_k)\subseteq C_j\cup C_{j+1}.
\]

4. **离散模板选择在 IPOPT 外面**：根据 Task 2 的真实 OBB 并集覆盖、车辆中心/角点 witness、Portal 过渡站点构造一个小的、确定性排序的模板候选集；每次 IPOPT 只接受**一个固定模板**（或每个连续段一个固定模板）。模板的每个约束仍为平滑的 CasADi 三角函数表达；不给 IPOPT 新增整数决策变量。
5. 对模板归属切换的弧长边界：必须在约束和连续审核上准确切段；变换前后两个归属模板都在公共端点检查。不能靠固定 Guide 预估的粗 `sigma` 切换后假定两边自然连续安全。
6. **模板不足不等于物理无路**：一个真实 `B_i⊂C_j∪C_{j+1}` 的矩形，可能无法被当前有限三角/strip 归属充分模板表示。有限模板都失败时，保留 `COVER_TEMPLATE_INSUFFICIENT` / `FORMATION_CORRIDOR_UNCONFIRMED`，可有限追加新的局部模板、细分或补充经地图认证的凸区；不能宣布 `MAP_OBB_COLLISION`，更不能将不存在的安全解标作已求得。

这是**可求解的保守充分条件实现**，不是一般非凸集合包含的完全等价解析表达。Codex 的执行方案必须写清其保守性、固定模板上限、反例以及何时不能证明。不得把它写成“无损把非凸约束线性化”。

### 5.3 模板搜索与预算

- 从**单凸区完整 OBB 归属**开始（最简单、最少约束）；Portal 段才启用 `C_j∪C_{j+1}` 分片模板。
- 同一区间内三个车辆可各自选区域或跨区模板，不能生成整个路径/每车/每片全部组合的指数笛卡尔积。优先利用 Task 2 的固定 Guide/witness 缩小局部选择；只允许少量局部修正。
- 现有每次求解的 finite initial/recovery 和 soft wall clock 是上界；新增模板尝试必须计入这些上界，不得偷偷再加一整套无限重复 IPOPT。
- 设计模板 `template_id` 的确定性 hash（含 chain digest、guide candidate ID、各车/片/区归属、弧长切段以及数值配置版本）并传到求解诊断与独立证书。
- 对同一路线多个可信模板，先按样本真实风险、最小有效 margin、模板约束规模排序；有限预算耗尽就报告未完成。

### 5.4 任务 2 `transition_windows` 的准确定位

Task 2 的 `s_window_m/σ_window` 是由 Guide 弧长与车体 reach 计算的**候选采样窗口**，不是优化曲线的严格证书，也不是物理时间窗口。

Task 3 可以将它作为初始模板切段的搜索范围，但不能直接把 `s_window_m` 当成最终 `S` 下的固定真实位置。使用 **归一化弧长 `σ=s/S`** 必须基于选中 Guide 的位置映射，在变化后的曲线上重新检验，模板切段/端点必须符合最终曲线与相关区域的实际连续约束。

---

## 6. 核心优化手段 C：离散 NLP 约束与连续认证构成闭环

### 6.1 为什么有限配点不够

NLP 在 `σ_k` 满足约束只说明这些离散位置合格，不能推出：

\[
\forall s\in[0,S],\quad\mathcal B_i(q_F(s))\subseteq U_{j(s)}.
\]

因此 Task 3 需要**独立的连续集群走廊审核**。现有 `corridor.py::audit_path_in_corridors()` 只验证 F 点在一组固定分配的单凸区里，不能冒用它的 `passed` 作为三车证明。

### 6.2 推荐连续审核界：各车顶点的弧长 Lipschitz 界

对车辆 i：

\[
\frac{dp_i}{ds}=R(\psi_F)\begin{bmatrix}1-b_i\kappa_F\\ a_i\kappa_F\end{bmatrix},
\qquad \left\|\frac{dp_i}{ds}\right\|=\sqrt{D_i}.
\]

当 `D_i>0`：

\[
\boxed{\frac{d\theta_i}{ds}=\kappa_F+\frac{a_i\eta_F}{D_i}}.
\]

对车体局部顶点 `u`，世界位置 `v_i(s)=p_i(s)+R(θ_i(s))u`：

\[
\left\|\frac{dv_i}{ds}\right\|\le
\sqrt{D_i}+\|u\|\left(|\kappa_F|+\frac{|a_i\eta_F|}{D_i}\right).
\]

对一个待验证弧长区间 `I=[s_0,s_1]`，以接受的数值区间/多项式界获得 `κ,η,D` 的**保守上下界**，于是有 `L_{i,u,I}`。令中点 `s_c`，单位法向 `n`：

\[
\boxed{
\max_{s\in I}(n^Tv_i(s)-b)
\le n^Tv_i(s_c)-b+L_{i,u,I}\frac{s_1-s_0}{2}
+\varepsilon_{i,u,I}.
}
\]

`ε` 覆盖路径位置、航向、相对车身角度、曲率求值等**有据可依**的数值误差。仅当每辆车、每个分片顶点、每个已归属区的半空间均证明 `≤-m_i`，当前区间才被认证。

不能直接使用采样点实测导数最大值当作连续上界；应该利用 Hermite 多项式包络、现有 `bounds.py` / `integration_bounds.py` 和必要的区间算术。不能仅凭较小步长宣称安全。

### 6.3 认证时必须切开的区间

- 曲率 Hermite 段边界；
- 已有路径数值积分域边界和真正的模型分段；
- **模板归属切换边界**（不同车辆/三角片改变归属时）；
- Portal 过渡窗口边界与生效的 allocation 断点；
- 必要的 kinematic denominator 下界边界。

对切换点应同时检验切换前/后可行性，避免断点仅一边通过。

结果至少三类：

| 类别 | 严格含义 | 后续行为 |
|---|---|---|
| `FORMATION_CORRIDOR_VERIFIED` | 全路径、所有车辆、全部被选中的分片归属约束都已连续认证，且数值界闭合 | 允许进入后续独立真实地图 OBB 终审，**不是直接发布** |
| `FORMATION_CORRIDOR_VIOLATED` | 发现明确的几何硬约束越界/witness，或经可靠几何检查的实际违规 | 拒绝该候选，允许预算内针对失败位置返修/重新求解 |
| `FORMATION_CORRIDOR_UNCONFIRMED` | 达到区间细分、精度、模板、数值或预算上限但未能证明，且没有可靠违规证据 | 不发布；报告未认证及准确阻塞原因，不冒充物理无路 |

必须记录：`vehicle_id, template_id, region_id, halfspace_id, vertex_id, s_m, Δs, point_residual_m, bound_residual_m, error_budget_m, min_margin_m, max_depth, interval_count, reason`。

### 6.4 正确处理既有 `CORRIDOR_AUDIT_WIDTH_LIMIT`

Task 3 不能只新增三车证书，却继续要求所有路径通过原 **F 点单凸区** 的过度保守分配，否则会把跨 Portal 合法轨迹再次误拒。

要求 Codex 对**新研究版集群模式**明确设计版本化的门控规则：

1. 原 `audit_path_in_corridors()` **保留**，供 F-only 对照、回归与诊断；不能篡改其结果或改容差来伪装通过。
2. 新研究模式必须建立 F 点在其**有效自由覆盖**（单区或合法局部并集）的连续证明，同时对三车完整 OBB 做上述覆盖证明。**证明几何域不得比实际交给 NLP 的域更宽且未经审核。**
3. 对新模式可将“原 F-only 单区审核通过”从**旧模式的硬门槛**改为**有记录的历史诊断**，但替代它的必须是有严格定义、经 Codex 独立验证的**F + 三车连续覆盖强证书**和原地图 OBB 终审；这个门控迁移必须在执行方案中逐条件证明为何不会削弱实际安全，不允许静默跳过审核。
4. 若 Codex 无法给出数学上完整的替代安全合同，则暂不改变原 hard gate，明确 `F_ONLY_GATE_BLOCKED` / `CORRIDOR_AUDIT_WIDTH_LIMIT` 的继续存在，并把门控迁移作为阻塞问题提交用户确认；**不得为了任务完成自行把失败改成 PASS**。
5. 对真实 `F` 路径点在走廊外、因固定 `σ` 切换而失败、以及只是区间界过保守导致 `UNCONFIRMED`，分别给出测试。

**注意**：Task 3 的关键不是“把所有失败变成成功”，而是“新约束与新认证语义一致；安全时能有机会通过，不安全时可靠拒绝”。

### 6.5 有限约束生成 / 定向重新求解

为避免一开始塞入过多约束，推荐采用**有限的约束补点闭环**：

1. 用基础 Gauss 节点、Hermite 端点、Portal/template 交界与必要安全裕量构造约束，运行 IPOPT。
2. 用**不依赖 NLP 配点**的独立连续集群审核寻找首个失败区间或最差残差。
3. 如果是可修复的配点间越界且预算尚有、模板本身有效：在对应**车辆/片/半空间/弧长位置**添加约束点，基于现有解重求解；不得只对 F 加点却忽略问题车辆。
4. 每轮要完整记录补点、约束数、最大残差、审核残差、求解时间；有限轮数、有限总配点数、有限总 wall。超过上限返回 `UNCONFIRMED`，不能发布。
5. 如果是模板覆盖不足或归属错误：只允许有限局部模板修正/已审核走廊补充，不把明确不可行的固定模板反复送进 IPOPT。
6. 每次新增约束必须仍然使用与原始优化器相同的连续曲率状态及确切模型；对 cache/solver build 的影响应有 profile，不提前重构所有 CasADi 构图结构。

**这部分必须证明**：加点约束进入 CasADi `g`，而不只是生成一份审查日志。已有 `CORRIDOR_AUDIT_WIDTH_LIMIT` 必须用于定向修复与诊断，但数值未确认不一定存在单个可靠违规点，需要保守处理。

---

## 7. 核心优化手段 D：真实障碍净空软目标（不替代硬约束）

### 7.1 现有能力与问题

`optimizer.py` 已实现基于地图 EDT 的 `clearance_proxy` 软项：三车中心的 EDT + OBB 支撑半径估计；它不提供可验证的硬安全界，并且原代码有依据初值确定的法向代理。Task 3 应**复用并有针对性地改进它**，而不是另建第二套障碍距离场。

### 7.2 建议的目标函数

三车真实障碍净空估计 `\tilde d_i(s)`，与目标偏好距离 `d_pref`（注意它只是软偏好，不是碰撞安全 margin）。使用可微 hinge / softplus 近似 `\phi_\delta(z)`：

\[
\boxed{
J_{\mathrm{clear}}=\frac{w_c}{3S}
\sum_{i=1}^3\int_0^S
\phi_\delta(d_{\mathrm{pref}}-\tilde d_i(s))^2\,ds.
}
\]

更简单、足够有效的离散积分在现有 Gauss 节点进行，数值随 S、采样数有合理归一化。`\tilde d_i` 可以保留经过验证的 EDT 支撑代理或在可靠梯度区域更新障碍法向，但必须独立于新硬约束的证书；方向不可靠时降级为 soft estimate/unknown，而不是制造硬判据。

**强制区分边界来源**：

- **真实墙/障碍/地图边界**：可以产生净空软偏好；
- **人造凸走廊边/Portal 切分边**：不能无差别当作墙，否则鼓励车辆远离合法的跨 Portal 区域，引起额外绕行。

如果需要根据走廊边做额外软项，必须先有可验证的“源于真实障碍的边”标签。当前 `ConvexCorridor` 使用精简后的 `edge_constraints`，**不能仅靠一条半空间的数值猜测它是真实墙**。没有可靠边界 provenance 时，优先仅用现有真实地图 EDT 净空代理。

### 7.3 消融测试

在完全相同的硬约束、地图、起终点、预算下对照：`w_c=0` 与启用软项，记录真正认证后的最小障碍净空/保守下界、轨迹长、曲率、成功率、总 wall 和 IPOPT 收敛情况。不允许只凭 RViz 看上去更居中就宣布优化有效。

---

## 8. 最小数据合同与 ROS/RViz 输出

### 8.1 研究配置

保持研究入口 `backend=virtual_rigid_a`、`guidance_mode=corridor`、`enforcement=hard`，为 Task 3 的新语义增加**最小、显式、版本化的**约束 scope，例如 `formation_corridor_scope=three_vehicle_union_v1`（最终名称由 Codex 校验现有 config 并明确），旧模式 `f_only` 保留为对照。

默认配置只在 Task 3 的完整三车约束接入并经过可复现验证后切换到新 scope，**不是仅修改字符串**。新 scope 的每次规划必须保存有效 scope/模板/审核版本；不影响 `uniform_v02`、`legacy_route`、单车 A/O/S、FR06 的历史入口。

不要为了加一个参数在全系统复制另一份 PlannerConfig；优先在 `research_planner_config`、ROS 入口和现有 `contracts.py` 最小扩展。与旧 config/request/model SHA 存在语义变化时显式版本化，不能悄悄复用旧证书/模型序列化标签。

### 8.2 统一候选身份

当前 Task 2 已有 `candidate_id/selected_guide_sha256/chain_digest/request_sha/grid_sha/context_sha`。Task 3 生成的 `template_id` 必须绑定这些字段。要求：**相同 candidate 的 Guide、allocation、template、NLP 实际约束及独立证书具有相同身份**；不允许从候选 A 取 Guide、候选 B 取 region 分配，或者复用不同地图的模板。

### 8.3 最少必须公开的求解/审核字段

```text
backend, guidance_mode, enforcement, formation_corridor_scope
plan_id, generation, request_sha256, grid_sha256, planning_context_sha256
route_index, round_index, guide_candidate_index, guide_source
chain_digest, template_id, template_count, template_validated_at_seed
actual_F_constraint_count, actual_vehicle_OBB_constraint_count
constraints_by_vehicle_and_region, maximum_NLP_inequality_residual_m
minimum_sampled_OBB_margin_m, constraint_refinement_rounds
F_only_legacy_containment_state, F_union_containment_state
formation_continuous_corridor_state, continuous_certificate_reason
first_failure_vehicle_id, first_failure_s_m, region/halfspace/template
geometry_audit, dynamics_audit, original_map_OBB_audit
executable, published, first_failure_layer, terminal_reason
```

以上字段需按实际得到的数据填值；未计算用 `null/UNKNOWN`，绝不填假成功数字。生产性能计时优先复用现有 `timings`、`search.rounds`、progress，不建新事件总线。

### 8.4 RViz 固定视觉语义

继续遵循用户已经确定的分类颜色：A* 橙 `#F97316`，Guide 绿 `#22C55E`，未认证候选黄 `#FACC15`，最终认证轨迹亮蓝 `#38BDF8`，凸工作空间区域柔红低透明度，三车 OBB 用低饱和轮廓。颜色属于**轨迹算法阶段**，不按车辆 ID 染成三种路径颜色。

增加/更新现有诊断层的最小能力：

- Task 2 的 Guide 地图碰撞样本与 **Task 3 最终优化路径的真实失败位置** 使用不同 marker namespace、文字和透明度，防止用户误认为两者是同一事件。
- 显示最终优化解中三车 OBB/中心轨迹与真实墙面的关系；OBB 只稀疏绘图，**不影响实际连续碰撞认证分辨率**。
- 高亮第一个违反的车辆、当前弧长、归属区域/Portal、`VIOLATED/UNCONFIRMED` 类型，能够在失败后保留图层。
- 右上角 HUD 显示已启用的三车硬约束 scope、实际 OBB 约束数、连续认证状态和明确终因；禁止进度百分比伪造。
- 新诊断 layer 仍为 `DIAGNOSTIC_ONLY`，不得进入 `/formation/playback_plan`；认证轨迹只有真正发布后才能显式显示为 `Certified`。

### 8.5 默认脚本每次交付都必须生效

Codex 在执行方案中列出脚本/launch/ROS 参数的真实完整链路。**Claude 完成 Task 3 后**必须可用正常用户命令，例如：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
./run_demo.sh --map map05 --scale 2.625
```

进入新约束 scope，不用手动补充隐藏 `backend` 或 `guidance`。若默认配置已包含目标算法且文件无需编辑，必须通过 `--dry-run` 和真实参数/quality report **提供证据**，而非为改动而改动脚本。旧算法需能显式选择用于论文对照。

---

## 9. 单任务内部实施安排（由 Codex细化为交 Claude 的执行方案）

这些是 **TASK03 一个任务内的工作步骤**，不是拆成多个独立项目：

| 实施顺序 | 核心工作 | 主要代码 | 完成证据 |
|---|---|---|---|
| Step 0 | 读取 Task 2 Codex 审核结论；冻结最新 HEAD/模型/地图/两张截图请求与 S1、map05；测原有成功率/失败层/耗时 | 现有测试、`pipeline.py`、ROS | 输入 manifest、旧版结果和性能基线；无可复现输入必须标明 |
| Step 1 | 单区域车体支持/角点约束、退化角度守卫、模板/约束编译数据合同；几何一致性单测 | `optimizer.py`、`contracts.py`、少量 `corridor.py` | 对真实三车 OBB 与独立静态模型的精确对照；导数/约束计数正确 |
| Step 2 | Portal 双区凸片覆盖模板、有限归属枚举和确定性选择，接入 Guide/chain/allocation 同源路径 | `corridor.py`、`pipeline.py`、`optimizer.py` | 非凸并集反例、三车不同区域反例、单车跨 Portal 正例、无模板 UNKNOWN 反例 |
| Step 3 | 真实 CasADi `g(z)` 硬约束、约束残差/导数检查、有限局部模板或补点重新求解 | `optimizer.py`、`pipeline.py` | 证明数值上确实进入 NLP；失败位置和相关变量可复算；保持 19 维基础轨迹模型 |
| Step 4 | 独立 F+三车连续走廊审核、数值误差界、门控迁移论证；保留原地图 OBB 终审 | `corridor.py`、`audit.py`、`pipeline.py` | 连续安全 vs 采样安全反例；未确认不发布；旧 F-only 审核仍可复算 |
| Step 5 | 真实墙净空软目标（复用 EDT）、消融与性能数据，调整必要尺度，不放松硬约束 | `optimizer.py` 和现有清空代理 | 开/关软项对照、合理梯度、真实净空/代价数据 |
| Step 6 | 默认脚本、ROS/HUD/RViz、离线与真 ROS 全回归，冻结证据 | 已有 ROS 节点/配置/`.rviz`、`run_demo.sh` | 默认入口真的运行三车硬约束；用户能看到失败/成功；历史模式不回归 |

Codex 对每一步都要列出**实际函数级修改顺序、输入输出结构、约束索引方案、数值稳定性处理、单测命令、完成条件和 Claude 交付证据**。没有必要重复实现 Task 1/2 的功能。

---

## 10. 必须设计和运行的实验矩阵

| 编号 | 输入/场景 | 必须验证 | 可接受结果 |
|---|---|---|---|
| E01 | map05 宽敞直线 / Task 2 已通过实际端到端例 | 默认研究入口、三车约束确实进入 IPOPT、原有认证全部保留 | 成功并发布时证书全 PASS；较旧版不无故回归 |
| E02 | 用户 `plan 600001` 同一地图/起终点的冻结复现 | Task 2 五候选正确衔接，新约束如何处理碰墙样本与 Portal gap | 得到真实连续认证安全的曲线，或明确失败/未认证；绝不发布撞墙曲线 |
| E03 | 用户 `plan 600002` 同一冻结输入 | 单 base Guide 的可行性与失败原因，不把样本碰撞写作物理无路 | 同上；不能仅靠改阈值通过 |
| E04 | Task 2 固定反例：F 自由但车 2/3 OBB 碰真实墙 | 三车 hard geometry 真正起作用 | 优化解规避；否则可靠失败不发布 |
| E05 | 跨 Portal：车 1 完全在 C1，车 3 在 C2 | 不强制所有车完整位于同一个凸区 | 存在 Task 3 覆盖模板能正确表达的正例并通过证书 |
| E06 | 一辆完整 OBB 横跨人工分界，完整属于 `C1∪C2` | 三角片/strip 模板支持跨区，不能只测四角 | 至少一个可表达的正例真实求解+认证通过；不能表达时准确报 TEMPLATE_INSUFFICIENT |
| E07 | OBB 四角均在非凸并集内但内部穿出并集 | 不能用角点属并集误判覆盖 | 证书拒绝或 UNCONFIRMED，不能 PASS |
| E08 | 所有 NLP 配点安全，中间曲线车体越界 | 连续审核 + 有限加点闭环 | 连续审核发现风险，重求解成功或拒绝；不能仅凭配点 PASS |
| E09 | F 点旧单区审核失败但真正 F+三车安全跨双区 | 版本化门控语义 + 连续强证书 | 若新认证完整则允许新模式通过（且原独立终审通过）；未证明时仍拒绝 |
| E10 | 地图真实可行但有限模板不能表达 | 区分人工覆盖不足和物理不通 | `TEMPLATE_INSUFFICIENT/UNCONFIRMED`，不是物理无路 |
| E11 | 急弯、偏移大、`D_i` 接近退化 | 车身方向、曲率导数/连续 Lipschitz 界与数值退化 | 远离退化时正确；近退化 fail-closed |
| E12 | `600003/600004` U-turn，`ψ≈±π`，长路径 | Task 1 winding/S/积分证书不回归 | 正确失败层或安全成功；不因 Task 3 打破旧数值合同 |
| E13 | 旧 `uniform_v02` / `legacy_route` / FR06 / 单车 A/O/S | 历史入口、证书和发布身份不变 | 原有回归 PASS；若基线本就 FAIL，分别保留事实 |
| E14 | ROS 旧 generation、重复 plan ID、地图变化、任务取消、诊断失败 | 安全身份与图层生命周期 | 旧请求不能污染新请求；未认证绝不可播放 |

场景中的 `600001/600002` **只是截图上显示的 plan ID**，它们不自动携带完整复现输入。缺地图/pose 时可先复现同类风险，但必须明确是“类似 fixture”，不能冒充同一场景。

### 10.1 有效对照指标

- **安全相关**：三车独立真实地图 OBB 连续碰撞最终状态、最小真实障碍净空或保守界、集群连续走廊认证通过率、未认证原因与首个失败 s/车号。
- **优化相关**：实际三车硬约束标量数、最大残差、模板数/切换数、补点轮数、IPOPT 迭代数/收敛状态、轨迹长度/曲率/车辆最大曲率、几何/动力学审查状态。
- **性能相关**：同样地图/请求/硬件上的可执行成功率、全流程 wall P50/P90、走廊/模板建图/NLP/连续审查/真实 OBB 审核耗时、预算耗尽比例。
- **可视化相关**：A* 橙/Guide 绿/候选黄/认证蓝/OBB 低密度、候选身份和求解阶段、失败位置、未认证与认证不能混淆。

无相同条件的实测不宣称性能提升。任何 `UNCONFIRMED` / `timeout` 不允许混入成功率分子。

---

## 11. Task 3 的 Codex 执行文档必须达到的具体深度

最终输出 `work_flow/TASK03_Codex执行方案.md`，**不仅是上述任务的转述**。至少包含：

1. **当前 HEAD 事实与差异**：Task 2 审核结果、真实函数/调用顺序、现有 PlannerConfig/hash 兼容风险、ROS 发布链、旧单区门控与新的覆盖语义冲突点。
2. **数据接口最终设计**：哪一个现有 dataclass 扩展、是否需要一个轻量不可变 `FormationCorridorTemplate`、其字段/`as_dict`/validation/digest/版本、怎么从 Task 2 的 `assess_formation_guide` 映射到模板。
3. **给 Claude 的关键伪代码**：如何生成完整 OBB 凸片、赋予 `region_id`、检查模板初始有效性、构建 CasADi 配点约束、复制到数值残差、获得连续导数界、做独立审核、补点/有限重求解、更新失败状态。
4. **数学证明与保守性清单**：单区域四角约束等价于 OBB 包含；凸片归属是并集包含的充分条件；模板可能有漏解；连续 Lipschitz 误差界的前提和退化处理；原 F-only 审核如何版本化保留。
5. **变更清单**：每个文件/函数的输入输出、与已有 19 维变量、`ca.nlpsol` 求解器、schema/model SHA 的交互；强制不修改哪些模块。
6. **求解与预算设计**：最多几个模板、每模板几个约束/采样、最多几轮补点、如何与已有总预算/initial/recovery 共享上限；若推荐参数需要调整，列依据和成本，不拍脑袋用无限重试。
7. **测试执行方式**：已有 unittest 命令、必要新增用例、`catkin_make`、纯 Python 离线可复现例、真实 `./run_demo.sh` ROS/RViz 演示、比较脚本与生成期望的证据文件；命令须在本地实际验证可运行。
8. **Claude 交付清单**：最终 diff、SHA/dirty 状态、测试与 exit code、原始日志、NLP `g` 的约束实际数量与 residual、独立连续证书、每个风险场景的原始报告和截图、默认入口 dry-run+实跑、已知失败。
9. **Codex 独立审核清单**：不只查代码，还用独立 fixture / solver spy / 重算区间界与原地图 OBB 终审、ROS 身份/发布负例、真实截图核对；不能拿 Claude 自报 PASS 代替独立验算。
10. **风险与回滚**：错误覆盖模板/非凸并集漏检、角度退化、求解无界时间、提前放宽 F-only 审核、旧序列化证书失效等分别如何识别，回滚只允许回到安全旧模式且需显式配置；不得静默降级运行旧算法骗过验收。

Codex 发现研究级设计分歧（特别是**有限模板是否足以覆盖用户期望 Portal 场景**、**新连续强证书能否安全接替旧单区 F-only 硬门槛**）时，需要给出数学反例、可实现备选与自己的推荐，提交用户 + ChatGPT 确认，不能将未证明的假设写成已经冻结的正确性结论。

---

## 12. Claude / Codex / 用户的正式交付与验收链

| 顺序 | 负责人 | 文件或动作 | 必须达到的要求 |
|---|---|---|---|
| 前置 | Codex | `TASK02_Codex审核报告.md` | 两张截图定向核查；实现缺陷修复后复审；Task 3 进入条件明确 |
| 1 | 用户 + ChatGPT | `TASK03_技术方案.md`（本文件） | 明确算法目的、数学模型、开发边界与验收指标 |
| 2 | Codex | `TASK03_Codex执行方案.md` | 文件/函数级详细执行说明，含必要证明、伪代码、测试和证据合同；此时不修改生产代码 |
| 3 | 用户 | 审查并批准 Codex 执行方案 | 特别确定跨 Portal 模板和连续门控语义 |
| 4 | Claude | 执行代码修改、自测、`TASK03_Claude开发交付.md`、`TASK03_evidence/` | 源码、命令、日志、证书、实际默认脚本/RViz 截图；不因失败造成功状态 |
| 5 | Codex | `TASK03_Codex审核报告.md` | 独立跑测试、检查 diff 和数学证书；FAIL 在同一 TASK03 返工 |
| 6 | 用户 | RViz 人工验收、`TASK03_验收记录.md` | 新模式默认运行，路径颜色/进度正确、碰撞风险及失败点可信、通过轨迹真实认证 |

**任务成功标准不是“用户截图再没有粉色风险字”，而是“优化器主动满足完整三车几何安全约束，连续审查严格可靠，可执行轨迹通过全部独立终审，失败不会被伪装成成功”。**

---

## 13. 可直接交给 Codex 的任务指令

> 请从最新本地分支出发，首先阅读 `work_flow/TASK02_定向审核方案_交Codex.md` 与最新 Task 2 审核结论。如果 Task 2 还没有获得结论，请先完成定向审查，不得跳过。随后阅读 `work_flow/TASK03_技术方案_交Codex.md`，结合实际源码和旧实验，编写**可直接交给 Claude Code 的 `work_flow/TASK03_Codex执行方案.md`**。要求函数级修改说明、完整数学约束和连续证明、Portal 非凸并集的有限可求解模板、任务 1/2 兼容与历史审核/安全门控、具体可运行测试与证据文件组织。不要改代码；若发现新旧审核硬门槛存在数学冲突或安全证明缺口，先把该设计决策作为阻塞项提交用户讨论，不能擅自放宽阈值。完成后停止，等待用户批准并交给 Claude。
