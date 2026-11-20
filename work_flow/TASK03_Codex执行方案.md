# TASK03 Codex 执行方案

版本：V1.0  
分支：feature/virtual-rigid-formation-a  
核对 HEAD：a3c709e57560b1060dcc422777f4131a3e558acf  
状态：**方案待用户确认；Claude 尚未获准实施。**

## 1. 结论、前置审核与工作区保护

TASK03 可在现有 virtual_rigid_a / FR06 架构上增量实现，不需要重建搜索、时间规划或 ROS 播放系统。当前限制是：TASK02 的三车 OBB/Portal 评估用于离散 Guide 诊断和排序，IPOPT 仍只约束参考点 F。最终真实地图三车 OBB 审核会拒绝不安全结果，但不能让优化器主动避开车辆占用冲突。

TASK02 定向审查已完成，报告为 work_flow/TASK02_Codex审核报告.md。plan 600001、600002 的结论为 **INSUFFICIENT_EVIDENCE**：缺少同次 request/map/context identity 和运行日志，不能用旧 plan ID 记录拼接复现。当前没有证实 TASK02 的实现缺陷；这不等于两个用户场景已复现或通过。

因此可以编制 TASK03 方案，但 Step 0 必须冻结真实请求和地图。若原始输入拿不到，E02/E03 标为 NOT_REPRODUCIBLE；可以使用冻结 S1 或相似 fixture 检查同类能力，但不得声称复现了原计划。

本次工作区已有修改：formation_continuous_a/geometry.py、tests/test_virtual_a_geometry.py；已有未跟踪材料包括 TASK02 审核报告/证据和用户提供的 TASK03 技术方案。Claude 开始时必须记录 HEAD、remote、完整 git status 和这些文件的 SHA-256。禁止 reset、checkout、clean、覆盖、移动既有内容；TASK01/TASK02 冻结证据不许改写。新增 TASK03 证据统一放 work_flow/TASK03_evidence/<UTC时间戳>/。

**不做：**更改 A-only/正向/刚性编队模型；三个独立 planner；O/S；底层控制/CAN/Gazebo/实车；重写 A*、Task2 Guide/Portal；改变 8 段 Hermite/19 变量、Task1 winding/S 域/积分证书、时间规划；降低审核阈值或 margin；改旧测试预期；增加无限候选或无证据的复杂 fallback；修改旧 FR06、uniform_v02、legacy、单车 A/O/S。

## 2. 当前调用链和复用矩阵

### 2.1 真实入口

~~~text
./run_demo.sh
→ catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch
→ scripts/formation_uniform_planner_node.py
→ VirtualABackendHooks.solve
→ formation_continuous_a.pipeline.plan_virtual_a
→ _plan_virtual_a_impl: A* routes → corridor chain → Guide/Portal candidates
→ solve_open_path_guided → 8 Hermite segments / 19 variables
→ F-only continuous corridor audit
→ existing timing → geometry/dynamics audit → original-map OBB audit
→ atomic playback envelope verifier/publisher
~~~

run_demo.sh 对多车默认使用 virtual_rigid_a，并注入 corridor guidance、hard enforcement、auto_goal=false；uniform_v02/legacy 是独立后端。formation_demo.launch 传参给 formation_uniform_planner_node.py；其 main 使用 research_planner_config。pipeline.py::_plan_virtual_a_impl 以同一个 selected_index 取 Guide、allocation、assessment，并将相同 chain/allocation 传给 solve_open_path_guided。失败请求沿用现有 failure/empty-envelope 流程，只有 executable 结果可形成轨迹发布。

### 2.2 按函数复用，不建平行系统

| 代码 | 已有能力与 TASK03 用法 |
|---|---|
| contracts.py：PlannerConfig、research_planner_config、planning_context | 配置和 request/grid/context digest。加 scope，但 f_only 默认序列化和旧 context hash 必须保持不变。 |
| contracts.py：GuideRoute、ConvexCorridor、CorridorChain | 原 Guide/半空间/chain digest。继续复用，不复制数据模型。 |
| corridor.py：audit_corridor_chain | 原始占用 cell SAT、地图边界、半空间一致性、Portal 审核。模板只能用其已审核 region。 |
| corridor.py：formation_transition_windows、assess_formation_guide、obb_coverage_in_adjacent_union | 生成 Portal 候选/witness；只能作离散模板候选，不是连续证书。 |
| corridor.py：allocate_corridors、audit_path_in_corridors | F-only 固定 sigma 分配和连续审核。保留作 f_only gate/新 scope 诊断。 |
| optimizer.py：_piece_position_samples | 现有 Hermite-piece prefix integral，三车 corner 必须使用相同 F state。 |
| optimizer.py：_numeric_constraint_vector、_build_casadi_nlp | 当前 numeric 和 CasADi 约束入口；两边都要加相同车辆 corner rows。 |
| optimizer.py：_clearance_proxy、_proxy_numeric_penalty | 原图 EDT/OBB 支撑软代价，复用为 soft preference，绝不当硬安全。 |
| optimizer.py：solve_open_path_guided | 现有 endpoint/C2/曲率约束、IPOPT、初值池、geometry audit。新 scope 传固定模板并共用预算。 |
| geometry.py：C2CurvatureProfile.interval_bounds、VirtualAPath.state_at_s、map_spatial_state | Hermite outward bounds、权威路径求值、三车实际空间运动学。当前用户 dirty geometry.py 只读。 |
| bounds.py、integration_bounds.py | outward-rounded 区间与积分误差，优先复用，不顺便重写。 |
| collision.py::audit_collision、FormationCollisionChecker | 原地图、边界、车辆间及连续 OBB sweep；仍是最终必要门槛。 |
| timing.py、trajectory.py | 公共时间参数化、权威路径和 timed model，不能另做平行实现。 |
| pipeline.py::_plan_virtual_a_impl | 候选、求解、F audit、后续审查/预算；接入模板和强证书，保留同一 identity。 |
| ros_adapter.py、planner node、formation_playback.py | 现有发布、progress/quality、HUD/marker；只最小扩展 scope/证书/最终失败位置。 |

当前 request 已将车型限制为 0.738 m × 0.500 m，偏移为 (1,0)、(0,1)、(0,-1) m。collision.py 使用同一研究车型并计算自己的误差 margin。NLP 不得另造车型尺寸，也不得以较小的 corridor margin 替代最终 collision margin_used_m。

### 2.3 代码事实

- optimizer.py::_build_corridor_constraint 当前只将 F 半空间加入约束；numeric residual 和 CasADi g 同样是 F-only。
- 已有三车曲率硬约束，EDT clearance_proxy 是软目标，没有三车完整 OBB 硬几何约束。
- pipeline 在约 2159 行调用 corridor.py::audit_path_in_corridors；该函数只认证 F，并明确标记不是三车证书。
- 通过 geometry 后仍运行 timing、audit_dynamics、collision.py::audit_collision。三者及发布身份 verifier 均须保持。
- PlannerConfig.as_dict 对默认 corridor 字段做省略以保持历史 hash；新 scope 必须沿用此兼容策略。

## 3. 数学合同与建议接口

### 3.1 三车轨迹及 OBB

保留现有 F 弧长轨迹：

$$
\frac{dp_F}{ds}=(\cos\psi_F,\sin\psi_F),\quad
\frac{d\psi_F}{ds}=\kappa_F,\quad
\frac{d\kappa_F}{ds}=\eta_F.
$$

对 offset r_i=(a_i,b_i)，定义：

$$
A_i=1-b_i\kappa_F,\quad B_i=a_i\kappa_F,\quad
D_i=A_i^2+B_i^2,
$$
$$
p_i=p_F+R(\psi_F)r_i,\quad
\theta_i=\psi_F+\operatorname{atan2}(B_i,A_i),
$$
$$
\|dp_i/ds\|=\sqrt{D_i},\quad
d\theta_i/ds=\kappa_F+a_i\eta_F/D_i.
$$

A-only forward 要求 A_i>0、D_i>0。区间不能证明正下界时，返回 normal-branch failure 或 UNCONFIRMED；禁止 atan2(0,0) 生成伪航向。OBB 局部四角 u_v=(±L_i/2,±W_i/2)，世界角点：

$$v_{i,v}=p_F+R(\psi_F)r_i+R(\theta_i)u_v.$$

### 3.2 单 region OBB hard rows

每个已审核凸区为 C_j={p:A_jp≤b_j}。OBB 是四角的凸包，因此四角都满足该 region 所有 halfspaces，等价于完整 OBB 在该凸区内。NLP 约束：

$$g_{i,v,h}=n_h^Tv_{i,v}-b_h+m_i\le0.$$

采用四角 sin/cos 约束，不依赖 abs 支持函数的非光滑点。m_i 是有明确 provenance 的 OBB buffer，不得低于 CollisionConfig.baseline_margin_m；数值 enclosure error 在 continuous audit 中另闭合。不可降低最终 collision audit 自己计算的 margin。

### 3.3 Portal finite templates

U_j=C_j∪C_{j+1} 一般非凸；四角在 union 不说明矩形内部也在 union。

首版固定模板：

1. whole-OBB：整个矩形归属一个 convex region。
2. Portal diagonal-0/diagonal-1：沿矩形对角线拆成两个闭三角形，每片分配到 C_j 或 C_{j+1}。
3. 每片所有顶点在所分配 region，因两者均凸，推出整片在内；所有片精确覆盖矩形，故推出 OBB 完全在 U_j。
4. 不同车辆和不同片可落在不同相邻 region；region 选择在 IPOPT 外固定，不加整数变量。
5. 这是保守充分条件，不是 union containment 的完整表达。模板无法表示的安全姿态返回 COVER_TEMPLATE_INSUFFICIENT/UNCONFIRMED，不能称物理无路或 MAP_OBB_COLLISION。

每个 selected Guide 最多上述三个模板。只有 E06 的真实安全 witness 证明它们不够，才讨论增加有限 strip；不得枚举三车所有区域组合。

### 3.4 Template identity

建议增加一个轻量 immutable FormationCorridorTemplate，字段包括 schema/version、request/grid/context SHA、chain digest、route/candidate/guide/allocation identity、车型和偏移 identity、decomposition kind、完整 sigma segments、F region、每车凸片顶点及 region assignment、margin/compiler/certificate version、template_id。

template_id 是规范化有限数 JSON 的 SHA-256，不含自身 digest。构造时检查 sigma 覆盖 [0,1]、无缺口/倒序、每片完整覆盖原 OBB、region 合法且仅为单区/相邻 pair、Guide/allocation/assessment/Chain/request/grid/context 完全同源。Portal window 只产生初始候选；优化后必须按实际 VirtualAPath 的 S 重新验证。

### 3.5 NLP row compiler

对全部 Hermite endpoints、当前 Gauss nodes、template/allocation boundaries 编译 rows。sigma 映射到 s=sigma*S 及对应 Hermite piece/q，F 状态调用现有 _piece_position_samples()；之后按 3.1 算三车 corners。

每个 row 的 provenance 至少是：

~~~text
vehicle_id, piece_id, region_id, halfspace_id, vertex_id
sigma, curve_piece, template_id, source, g_index
~~~

_numeric_constraint_vector、_constraint_value_offsets、_build_casadi_nlp、failure residual 和 SciPy fallback 必须消费同一 row 顺序/公式。actual OBB constraint count 只能统计真实 append 到 g 的 scalar rows，不可从 marker 或 polygon 数推断。新 scope 禁止 solver success 但无 hard rows。

模板候选和解算的最小控制流：

~~~text
for template in deterministic_order(templates_from_same_selected_candidate):
    if deadline_expired or no_unused_initial:
        record NOT_TRIED_BUDGET; break
    verify request/grid/context/chain/guide/allocation identity
    rows = compile_F_and_vehicle_corner_rows(template, endpoints, Gauss, cuts)
    geometry = solve_open_path_guided(..., fixed_template=template, rows=rows,
                                     max_new_initials=1)
    if no independently geometry-audited candidate:
        record actual solver reason; continue
    old_F = audit_path_in_corridors(...)  # diagnostic in union scope
    certificate = audit_formation_path_in_corridors(...)
    if certificate is VIOLATED and refinement_count == 0:
        add rows at exact witness for its vehicle/piece/region/halfspaces
        solve once with same template and a new unused initial identity
        recertify the returned path
    if certificate is not VERIFIED:
        record VIOLATED/UNCONFIRMED/TEMPLATE_INSUFFICIENT; continue
    run existing timing, geometry/dynamics, original-map OBB and envelope gates
    only if every gate passes return executable; otherwise retain real failure
return non-executable result with first/terminal failure and spent budget
~~~

连续审核按模板段和模型段切好 interval 后，对每个 F/OBB halfspace 计算 outward upper bound。upper≤0 时该 row 在当前 interval 认证；只有包含误差后的明确正 point witness 才记 VIOLATED；否则在预算内二分。达到 width/depth/interval/time 上限仍不闭合则 UNCONFIRMED。不可用采样点导数最大值替代连续上界。

### 3.6 独立连续证书

有限 NLP 配点不代表整条路径安全。连续 audit 复用 C2CurvatureProfile.interval_bounds()/bounds.segment_jet_bounds() 得到 outward k/eta 区间界。对区间 I=[s0,s1] 先证明 A_i>0 和 D_i≥D_min>0，corner 速度上界：

$$
L_{i,u,I}\ge\sqrt{D_{i,\max}}+
\|u\|\left(\max_I|\kappa_F|+
\frac{|a_i|\max_I|\eta_F|}{D_{i,\min}}\right).
$$

中点残差 r_c=nᵀv(s_c)-b+m_i 的区间上界：

$$
\max_{s\in I}r(s)\le r_c+L_{i,u,I}|I|/2+\epsilon_{i,u,I}.
$$

epsilon 覆盖 path position/heading error、offset 和 corner lever arm 放大、curvature evaluation error、outward rounding。至少验证：

$$
\epsilon_{i,u}\ge E_p+\|r_i\|E_\psi+
\|u\|\left(E_\psi+\frac{|a_i|}{D_{i,\min}}E_\kappa\right)+E_{\rm round}.
$$

不能给出 E_kappa 就不得设零，返回 UNCONFIRMED。切分 Hermite/profile domain、template assignment、Portal/allocation 和 denominator branch 边界；切点前后都检查。全部 halfspace upper residual≤0 才 VERIFIED。

| 状态 | 含义 |
|---|---|
| FORMATION_CORRIDOR_VERIFIED | F 与三车完整 OBB 全路径连续包含已证明；只允许继续原地图最终审核 |
| FORMATION_CORRIDOR_VIOLATED | 有包含 error budget 的明确越界 witness；拒绝，可用一次定向 cut |
| FORMATION_CORRIDOR_UNCONFIRMED | 数值、模板、细分或预算不足以证明；拒绝发布，不冒称碰撞 |

证书要记录 request/grid/context/model/path/chain/template identity、首个失败车/片/region/halfspace/vertex、station、width、point/bound residual、error/margin、depth、interval count 和 reason。

### 3.7 F-only gate 迁移证明

旧 audit_path_in_corridors 保持原样，f_only scope 继续使用为硬门槛。新 union scope 可将旧固定 sigma 单 region 结果降为诊断，前提：

1. audit_corridor_chain 对同一个 Chain 的引用 region 仍通过原始 map cell SAT、地图边界和半空间一致性。
2. 新模板只能引用这个已审核 Chain 的单区或真实相邻 Portal pair。
3. 新证书同时连续证明 F 点和三辆车所有完整 OBB 凸片属于该相同 allowed set。
4. 证书绑定 path/request/grid/context/Chain/template，身份不匹配 fail closed。
5. 原 audit_geometry、audit_dynamics、collision.py::audit_collision 和 envelope verifier 仍是发布必要条件。

已审核 region 属于原地图自由空间。片顶点全程满足 convex halfspaces 推出完整片和 OBB 在该 region/相邻并集内；F 也单独审查。这个 F+OBB 连续证明在同一已审域内替代更严格的 F 固定单区条件；原地图 collision audit 仍必须通过。因此仅 union scope 可迁移，旧 f_only 不变。任何条件未落地时保留旧 gate，新 scope 返回 F_ONLY_GATE_BLOCKED/UNCONFIRMED，不许静默放行。

### 3.8 有限 refinement 和预算

仅当独立连续审核给出可靠 VIOLATED station 才加对应车辆/片/region/halfspace/vertex 的真实 hard rows；UNCONFIRMED 不盲目补点。每模板最多一次 refinement solve，新增 solve 共享现有初值预算。

上限：selected Guide 模板≤3；全 request solver initial launch≤现有 max_initials≤6；route≤3、recovery≤1、G4 soft budget≤60s。每个模板先用一个尚未启动的 initial identity，refinement 再消耗一个；预算不足的模板标 NOT_TRIED_BUDGET。不得暗加重试或总时长。

## 4. 配置、结果和文件改动

### 4.1 Versioned scope

PlannerConfig 加 formation_corridor_scope，合法值 f_only / three_vehicle_union_v1。Python 默认保持 f_only；as_dict 在 f_only 省略新字段以保持旧 config/context SHA；union scope 明确加入 config 和 context SHA；历史 payload 缺字段时回到 f_only，未知值拒绝。开发先显式启用 union；全部验收后才改 run_demo.sh 默认。uniform_v02、legacy、legacy_route 和单车路径不受影响。

VirtualAResult 建议加入可选 scope 和 formation_corridor_audit。union SUCCESS/NO_MOTION 必须带 VERIFIED 且 SHA 全匹配的证书；无证书、非 VERIFIED 或 digest mismatch 不得 executable。新 scope 的 certificate/template/path identity 由已有 quality/progress/envelope plumbing 校验；失败沿用原不可执行 placeholder，不发布旧轨迹。f_only 旧序列化保持兼容。

### 4.2 必要生产改动

| 文件 | 修改职责 |
|---|---|
| formation_continuous_a/contracts.py | 新 scope、配置兼容、template 合同/digest、VirtualAResult executable invariant |
| formation_continuous_a/corridor.py | 从同一 candidate assessment/witness 构造 ≤3 固定模板；做分片和 region assignment 预检；不改旧 corridor 审核 |
| formation_continuous_a/optimizer.py | 新 constraint compiler；三车 corner rows 进入 CasADi/numeric/SciPy；输出真实 count/residual |
| formation_continuous_a/audit.py 或 corridor.py | 新独立 continuous audit（二选一，先决定职责避免重复），复用已有区间和路径合同 |
| formation_continuous_a/pipeline.py | 同一 guide/allocation/chain/template identity；有限模板与cut预算；强证书门控和准确失败 |
| formation_continuous_a/ros_adapter.py、scripts/formation_uniform_planner_node.py | scope/证书绑定、quality/progress 和 envelope identity verifier |
| formation_playback.py | 复用现有 marker/HUD，新增最终优化路径 OBB 与 failure/certificate 状态；sample 与最终失败分层 |
| catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch、run_demo.sh | 显式 scope 参数；所有验收完成后再把默认切到 union；保留 f_only |

### 4.3 测试文件和保护范围

新增 tests/test_virtual_a_formation_corridor.py；扩展 test_virtual_a_corridor_optimizer.py、test_virtual_a_pipeline.py；必要时扩展 test_virtual_a_ros_adapter.py、test_virtual_a_corridor_display.py、test_formation_demo_assets.py。复用现有 Task2 union/Guide tests，不复制整套测试。

不得修改 geometry.py、bounds.py、integration_bounds.py、timing.py 的数值语义和 collision.py 最终审核/margin/sweep policy。若新证书确实需扩展 trajectory serialization，先说明必要字段和旧 payload 兼容方案。不得碰 C++ FR06、uniform_v02、legacy optimizer、单车 A/O/S、历史测试期望或冻结证据。无确切必要不改 CMake/msg。

## 5. Claude 单 TASK03 内部步骤

Claude 可在每个内部 Step 完成后自行测试、自审并记录；用户要求 Codex 审核 TASK03 最终整体交付，不必每个小步停下。TASK03 全部结束后冻结等 Codex；未 ACCEPTED 前不切默认、不启动 TASK04、不 commit/push。

### Step 0：冻结基线

记录 HEAD/remote/status、既有 dirty/untracked SHA；冻结 map05 scale 2.625、S1 和空旷直线路径的 request/map/context/model/config 以及旧结果；查找用户 600001/600002 原始输入，不存在则标 NOT_REPRODUCIBLE。只运行旧默认 dry-run，不改生产代码。

~~~bash
git status --short
git rev-parse HEAD
sha256sum catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py tests/test_virtual_a_geometry.py work_flow/TASK02_Codex审核报告.md
./run_demo.sh --map map05 --scale 2.625 --dry-run
~~~

验收：既有 dirty SHA 前后一致，request identity 真实可追；缺失输入不假复现。

### Step 1：合同和单凸区 OBB

先验证旧 config/context hash；新增 scope/template digest；完成 single-region whole OBB compiler，再接 CasADi/numeric g。

用例：至少四种 heading（含非零/负）及正负曲率；corner 与 map_spatial_state/静态 checker 几何一致；支持函数与四角投影最大值误差≤1e-10m；A/D退化 fail closed；四角约束凸包正确；numeric/CasADi g 最大差≤1e-8m；Jacobian 有限差分相对误差≤1e-5，近零项另报绝对差；solver spy 确认每车 hard rows 真正进 g。

~~~bash
python3 -m unittest test_virtual_a_formation_corridor
python3 -m unittest test_virtual_a_corridor_optimizer
python3 -m unittest test_virtual_a_collocation
~~~

### Step 2：Portal 固定有限模板

复用 formation_transition_windows 和同一 selected assessment/witness。sigma partition 来源于 allocation/Portal candidate cuts；每车最多 whole + 两个对角线模板。所有入口只能来自同一 candidate state。

用例：三车分属相邻 regions；单车两个三角片分别在两区；四角位于两个分离凸条而矩形内部穿 gap；模板无法表示但 map geometry 未证伪的 UNKNOWN；sigma 全覆盖、切点双侧检查；同输入 digest 稳定，换 guide/chain/grid/model 即 digest 变化；漏片/错 region/伪造 hash 失败。

~~~bash
python3 -m unittest test_virtual_a_formation_corridor
python3 -m unittest test_virtual_a_formation_assessment
python3 -m unittest test_virtual_a_corridor test_virtual_a_corridor_optimizer
~~~

### Step 3：NLP 硬约束和一次 cut

初始 sigma 包含所有 Hermite endpoints、当前 Gauss nodes、template/allocation cuts。CasADi、numeric、SciPy 和 residual 同序；count 由真实 g append 取得。独立 audit 给 definite violation 才加点；最多一次 refinement，UNCONFIRMED 不乱补，timeout 不启动额外 solver。

验收：g size 等于 equality + 旧 signed constraints + F rows + OBB rows；每个 vehicle 有真实 rows；固定 z 独立重算 residual，numeric/CasADi≤1e-8m；接受候选最大 hard residual≤1e-8m；F可行但车辆角点出界的负例失败；spy证明第二轮 cut rows 真的进 g；仍保留19 variables和8段。

~~~bash
python3 -m unittest test_virtual_a_formation_corridor
python3 -m unittest test_virtual_a_corridor_optimizer
python3 -m unittest test_virtual_a_optimizer
~~~

### Step 4：continuous certificate 和门控

复用 VirtualAPath/profile interval bounds，覆盖 F 和三车所有 corner/piece；按必需边界切段，连续 upper bound 未闭合一律 UNCONFIRMED。运行旧 F-only 审核作对照；union scope 只有强证书 VERIFIED 后进入原 timing/geometry/dynamics/map collision。

用例：配点安全但中间角点越界；全程有余量的证书正例；Hermite/template/Portal切点双侧；interval保守不能闭合；A/D退化；NaN及深度/宽度/interval/time耗尽；tamper request/grid/context/path/model/template；旧 F-only 结果不变；thin obstacle、真实车辆航向、边界及碰撞 margin 回归。

验收：VERIFIED 所有 outward upper residual≤0；VIOLATED 有可信正 witness；UNCONFIRMED 无 playable envelope；union result 缺证书或 hash 不同不得 SUCCESS/executable。

~~~bash
python3 -m unittest test_virtual_a_formation_corridor
python3 -m unittest test_virtual_a_audit_evaluation_guard
python3 -m unittest test_virtual_a_collision
python3 -m unittest test_virtual_a_audit test_virtual_a_integration_bounds
~~~

### Step 5：复用 EDT soft clearance 并消融

不建第二距离场，使用 CachedClearanceField/_clearance_proxy。真实障碍只来自 map occupancy/map boundary；人工 corridor 边不能当墙。soft 项仅作偏好，不能影响 certificate；开关在同一 request/grid/context、相同硬约束/预算下比较。权重 0 精确恢复无 soft term；不得因失败调低 margin 或随意改默认 target/weight。

记录 final certificate clearance、路径长、曲率、solver status/iteration、全审计、总耗时。无数据支持不声称优化有效。

~~~bash
python3 -m unittest test_virtual_a_corridor_optimizer
python3 -m unittest test_virtual_a_optimizer
~~~

### Step 6：默认入口、HUD、ROS/RViz 和回归

先显式 union scope 贯通 launch→settings→pipeline→g→certificate→result→verifier。复用现有 progress/quality/marker；Guide sample risk 与最终 path failure 分不同 namespace。跑离线测试、catkin build、dry-run、实际 ROS/RViz。全部接受后才切换默认 scope，并保留 f_only 对照。

~~~~bash
python3 -m unittest test_virtual_a_pipeline
python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback
python3 -m unittest test_virtual_a_corridor_display test_formation_hud
python3 -m unittest test_formation_demo_assets
catkin_make -C catkin_ws -j4
bash -n run_demo.sh
./run_demo.sh --map map05 --scale 2.625 --dry-run
./run_demo.sh --map map05 --scale 2.625
~~~~

dry-run 不等于算法/RViz 验收。现场需核 scope、实际 OBB g count>0、certificate VERIFIED、geometry/dynamics/original-map OBB PASS、envelope verifier PASS；再跑 E04 负例确认不可播放、无旧轨迹残留。保存 ROS params/topic、quality/debug、日志和截图。

## 6. 实验矩阵与量化口径

| ID | 场景 | 可接受结果 |
|---|---|---|
| E01 | map05宽敞直线/冻结 TASK02 正例 | 新 OBB g 运行，证书及所有旧审核通过才发布 |
| E02/E03 | 用户600001/600002原始输入 | 缺输入为 NOT_REPRODUCIBLE；有输入后安全通过或如实失败 |
| E04 | F可行但车2/3 OBB碰墙 | 约束主动避开或拒绝 |
| E05 | car1在Cj、car3在Cj+1 | 允许不同 adjacent region 并正确认证 |
| E06 | 单车 OBB 横跨且完整属于 Portal union | 模板通过或明确 TEMPLATE_INSUFFICIENT |
| E07 | 四角属于非凸 union、矩形内部穿 gap | 拒绝/UNCONFIRMED |
| E08 | collocation 安全、中间曲线角点越界 | continuous audit 找到；cut进g，失败不发布 |
| E09 | 旧 F-only 单区 gate 失败、F+OBB在 audited union | 新强证书可通过，原地图 audit 仍必须通过 |
| E10 | map可行但模板无法表达 | TEMPLATE_INSUFFICIENT/UNCONFIRMED，不称物理无路 |
| E11 | 急弯、A/D临近退化 | 正下界正确处理，退化 fail closed |
| E12 | S2/S3 U-turn、heading近±π | winding/S/积分/19维不回归 |
| E13 | uniform_v02/legacy/FR06/单车/legacy_route | 历史行为不回归，已有失败如实披露 |
| E14 | 旧 generation/重复ID/map变化/取消/失败 | 身份不串扰、失败不发布旧轨迹或误标 Certified |

E07 的基础几何反例固定为真实车体尺寸 0.738×0.500 m，center=(0,0)、theta=π/2，OBB 为 x∈[-0.25,0.25]、y∈[-0.369,0.369]。设 C_upper=[-0.30,0.30]×[0.32,0.45]、C_lower=[-0.30,0.30]×[-0.45,-0.32]。四个车角落在两块 region，但 (0,0) 位于 union 外，因此角点属于 union 不能证明矩形包含。此 fixture 仅测试 nonconvex union 几何谓词，不冒称是有效 Chain Portal；E05/E06 另使用 audit_corridor_chain 通过的真实相邻区域。

记录安全指标（certificate、真实 map sweep/margin、first failure）、优化指标（g count/residual、每车/region、模板/refinement/IPOPT状态）、轨迹指标（长度/曲率/geometry/timing/dynamics）、性能指标（同机同输入各阶段耗时及 P50/P90）。只有 VERIFIED 且 geometry/dynamics/map collision 全 PASS 才计规划成功；UNCONFIRMED/timeout 不计入成功率。没有同条件基线不宣称性能提升。沿用 G4 总 soft 60 s、initial≤6、route≤3、recovery≤1。

## 7. 已验证的命令方式

仓库根目录、ROS Noetic 环境：

~~~bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
~~~

TASK02 独立日志中已成功运行过的调用方式：

~~~bash
python3 -m unittest test_virtual_a_formation_assessment
python3 -m unittest test_virtual_a_pipeline
python3 -m unittest test_virtual_a_corridor test_virtual_a_corridor_optimizer
python3 -m unittest test_virtual_a_corridor_display test_formation_hud
python3 -m unittest test_virtual_a_collision
bash -n run_demo.sh
catkin_make -C catkin_ws -j4
./run_demo.sh --map map05 --scale 2.625 --dry-run
~~~

每条命令保存 stdout/stderr、exit code、开始/结束时间。依赖缺失标 NOT_RUN；全量 discover 不得覆盖或代替定向失败。

## 8. Claude最终交付及完整任务提示词

交付 work_flow/TASK03_Claude开发交付.md 和 work_flow/TASK03_evidence/<UTC时间戳>/，包括 manifest、源码 diff/hash、dirty 状态、冻结输入 identity、测试 logs、CasADi真实g rows/count/residual、template digest/assignment、独立 certificate interval/error bound、cut记录、E01-E14状态、消融、dry-run和真ROS/RViz参数/日志/截图、已知失败。不能运行要标 NOT_RUN；600001/600002输入缺失标 NOT_REPRODUCIBLE。

~~~text
请在当前 feature/virtual-rigid-formation-a 工作区执行单一任务 TASK03。先阅读 work_flow/TASK03_技术方案_交Codex.md 与 work_flow/TASK03_Codex执行方案.md。

开始时记录 HEAD、remote、完整 git status 和既有 dirty/untracked 文件 SHA；不得 reset、checkout、clean、覆盖 TASK01/TASK02 工作或证据；不得建分支、commit、push。

按 Step 0–6 实现。保持三车 A-only 模型、8段 Hermite/19变量、Task2 A*/Guide/Portal、timing、geometry/dynamics 和 original-map OBB audit。车辆完整 corner hard rows 必须进入 CasADi g、numeric residual 和 SciPy fallback。Portal 仅做有限固定模板；不能证明则报告 TEMPLATE_INSUFFICIENT/UNCONFIRMED。

Continuous certificate 必须独立于 collocation，覆盖全 F 与三车完整 OBB。只有 VERIFIED 且原 geometry/dynamics/map OBB、publication verifier 全通过才 executable。缺证书、identity不符、VIOLATED、UNCONFIRMED、timeout均不发布；不能播放旧轨迹。不得降低margin/旧阈值或隐藏增加预算。

每个内部 Step 由你自测自审并记录，TASK03全做完后冻结，交付 work_flow/TASK03_Claude开发交付.md 和带 manifest/hash/log 的 work_flow/TASK03_evidence/<UTC时间戳>/，停止等待 Codex 整体审核，不自动开始 TASK04。

实现保持直接简单，优先复用现有模块，不加泛化 manager/factory/wrapper，不重复实现map/collision/timing/publisher。运行定向测试、catkin、dry-run和真实ROS/RViz；没运行就标NOT_RUN，dry-run不能冒充真算法验收。
~~~

## 9. Codex最终独立验收清单

Codex 在 Claude 冻结完整交付后整体验收，不以每个 Step 自述替代：

1. 复核 HEAD、dirty/untracked SHA、manifest；用户已有 geometry/test 与历史 evidence 不变。
2. 独立验证旧 config/context hash、新 scope hash、未知 scope fail-closed。
3. 重算 template 的 convex pieces 是否完整覆盖 OBB、region 是否合法相邻、sigma 是否连续；复跑 union corner/gap 反例。
4. 通过 solver spy/实际 CasADi 输出确认三车 corner rows 真进 g；独立重算 g、residual、gradient、cut 第二轮。
5. 独立重算代表 path 的 outward k/eta/D/L/error bound、必要切点和 upper residual；只有闭合才认可 VERIFIED。
6. 尝试用缺证书/错误 path、model、grid、context、template SHA 构造 SUCCESS/envelope，必须拒绝；检查失败没有旧轨迹发布。
7. 确认 geometry/dynamics/map collision 仍必需，margin/sweep 未放宽。
8. 检查 600001/600002 原始 identity；缺失只能认可 NOT_REPRODUCIBLE。
9. 独立跑定向测试、catkin、dry-run；条件允许时检查真实 ROS/RViz scope、HUD、marker、verifier。Mock/offline/dry-run 不替代现场验证。
10. 核对 E01-E14、成功率分子、耗时口径；不认可未实测性能承诺。
11. 任一非 VERIFIED 被标 executable/published、hard rows 未进 g 或原 collision audit 被绕过，TASK03 FAIL 并给返工。

## 10. 需用户/ChatGPT确认的研究决策

### A. 是否接受 finite template 漏解

有限两个对角线模板只提供充分条件。存在 OBB 完全属于 Cj∪Cj+1、但当前三种模板无法表达的情况。选项：

1. **推荐：**先做 whole-region + 两个对角线模板；表示不了就安全拒绝并报告 TEMPLATE_INSUFFICIENT。只有 E06 有具体安全 witness 时再讨论少量 strip。
2. 初版增加多种 strip/triangle，覆盖可能更高，但增加候选、约束和排序复杂度，仍不完整。
3. 求一般非凸 union/MIP，超出现有连续 NLP 阶段范围。

建议选 1。请确认可以接受“首版模板不足时安全拒绝”的边界。

### B. 新 scope 是否迁移 F-only hard gate

本方案给出数学条件：只在 three_vehicle_union_v1，允许 F+全 OBB 连续证书替代 F 固定单区门槛，旧 f_only 和旧 audit 函数保持。若不接受，旧 gate 继续阻止部分 Portal path，新 scope 必须 fail closed。建议按上述 scope 限定迁移，请确认。

### 其他技术风险

- finite templates 可能把物理可行 path 归为 TEMPLATE_INSUFFICIENT。
- 三车 corner/halfspace rows 可能较多，需实测 build/solve，不能承诺更快。
- interval bound 保守可能 UNCONFIRMED，不能改称 collision 或 PASS。
- 最终 collision audit 的误差 margin 可能大于 NLP buffer，仍以原 audit 为准。
- 600001/600002 输入缺失，具体现场原因待补。

## 11. 文档自审与停止点

| 自审 | 结果 |
|---|---|
| TASK02 结果如实承接 | 目标运行保留 INSUFFICIENT_EVIDENCE，不冒称复现/通过 |
| 源码接口落地 | 给出配置、candidate、optimizer、audit、collision、pipeline、ROS入口 |
| 数学合同一致 | 同一 F path 恢复三车位置/heading/OBB；凸片仅作并集充分证明 |
| 配点是否被误作连续安全 | 否；独立 interval certificate 才决定放行 |
| 旧独立安全门 | 保留 geometry/dynamics/original-map OBB/envelope gate |
| 搜索、模板和 solver retry 是否有上限 | 模板≤3，initial≤现有6，route≤3，recovery≤1，总预算≤60s |
| 用户 dirty 文件和历史证据 | 要求前后 SHA 保护 |
| 未验证性能承诺 | 无；要求同条件实测 |
| 用户需决策 | finite template 漏解接受度、union scope 下门控迁移 |

**停止条件：**本轮只新增本执行方案，不修改生产/测试/ROS配置，不运行实际开发。用户确认第10节决策后才把第8节任务书交 Claude；Claude 完整交付冻结后由 Codex 独立验收。在验收通过前不切默认、不启动 TASK04、不提交 Git。
