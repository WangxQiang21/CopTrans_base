# 虚拟集群凸走廊实施架构与决策 V0.1

编制日期：2026-11-17。用途：Claude 分阶段开发、Codex 增量验收；本文不代表实现完成。

## 基线与证据等级

【代码事实】本次本地核对 HEAD `23fd16227e75a68f03f3ac7483555063c8189313`，分支 `feature/virtual-rigid-formation-a`，origin 为 `git@github.com:WangxQiang21/CopTrans_base.git`，核对前工作区干净。没有联网核实远端 tip，不能称为远端最新提交。当前提交已将 G0–G5 和证据纳入 Git。候选分支 `feature/virtual-rigid-a-corridor` 本轮不创建；未来工作区由 Codex 准备，Claude 只在被交付的工作区开发。

【实验记录】`docs/virtual_formation_a/diagnostics_20261116/repro_uturn_summary.json` 中有五次 IPOPT success 后被独立积分审核拒绝，最后一次迭代超限覆盖了最终原因。本轮只读分析，不重跑实验、不将旧记录视为新方案效果。

所有下面的新增接口、默认试验参数、任务验收阈值均为【设计合同/尚未验证】，现有 API 见复用矩阵。保留旧 FR06 和现有 A-only 后端；C0 修复后的旧强引导作为正式 A/B 的 A 组。

## 架构与改动边界

`plan_virtual_a(map_request)` 保持唯一入口。增加显式 `PlannerConfig.guidance_mode='legacy_route'|'corridor'`，首版默认 legacy_route；C0 不加该开关、不改权重。新模式顺序：现有 A* → 精确 LOS → 分离半空间走廊 → portal QP → 几何引导 → 现有曲率 NLP → 走廊连续包含检查 → 原几何审核 → 原公共时间律 → 原动态审核 → 原三车连续 OBB 终审 → 原信封发布/回放。

只新增 `route_processing.py`（LOS/简化）、`corridor.py`（凸几何/portal/中线/包含检查）、必要测试文件、一个小型比较实验入口；不另建走廊 service、manager、validator 包。独立生成和审核采用不同几何算法，放同一模块的两个直接函数即可。ROS 显示仍由现有 planner/playback 节点承担。

算法真实设置放 PlannerConfig，走廊数据留在 `VirtualAResult.search`。仅非默认新设置才进入 `PlannerConfig.as_dict()` 的新增项，旧默认序列化保持原样；旧模型 JSON 不重写。新配置参与已有 planning_context 哈希；轨迹的运动学、时间律、模型版本和消息 MD5 不变。新走廊不是轨迹权威模型，不将整链塞进 model_json。若实测 loader 无法支持额外配置键，先提交最小兼容说明并停止相关接入，不能静默丢配置或改旧版本语义。

## 固定数学合同

偏移由现有运输合同读取，r1=(1,0)、r2=(0,1)、r3=(0,-1)m。全车正向统一 A，初态静止/车头平行/理想直行轮系。FREE 目标仅约束 F 的位姿；STRAIGHT 额外 κ(S)=η(S)=0。FREE 非兼容终态仍必须拒绝普通续接。

p_i=p_F+R(ψ)r_i；p_F'=(cosψ,sinψ)，ψ'=κ，κ'=η。
A_i=1-b_iκ>0，D_i=A_i²+(a_iκ)²>0；θ_i=ψ+atan2(a_iκ,A_i)；C_i=(κD_i+a_iη)/D_i^(3/2)。v_i=√D_i·s_dot，ω_i=(κ+a_iη/D_i)·s_dot。κ:[m^-1]、η:[m^-2]、u=κ'':[m^-3]。

曲率三次 Hermite 每段 h=S/N，c0=κj，c1=hηj，c2=3(κj+1−κj)−h(2ηj+ηj+1)，c3=2(κj−κj+1)+h(ηj+ηj+1)。内部 C2： (2c2_left+6c3_left)/h²=2c2_right/h²，κ和η由共用节点连续。N=8 首版保持，第三导数分段有界。三车曲率、轮系、完整二维动态限制及数值证书全部保留。

C0 配点使用段起点的前缀积分 p_j(q)=p_j(0)+h q Σw_l e(ψ_j(q ξ_l))；完整段末积分单独求，不可把完整 Gauss 权重的累计和当作配点位置。Numpy/CasADi 同用 ξ,w 和解析 ψ 多项式。局部坐标状态做世界坐标变换后才进入地图/走廊，避免旋转起点错误。

## 走廊合同

原始 occupied/unknown 栅格是闭正方形，接触视为碰撞；地图外为障碍。距离场仅作代价。A* 8 邻域路线本身仍须精确 LOS 检验，不能假定对角线不会切角。

每段 S 与原始栅格方块 O 精确最近点求单位 n，h=max_{s∈S}n·s、l=min_{o∈O}n·o；取 β=(h+ρ+l−d)/2，要求 h+ρ<l−d。区域 C=B∩{n·p≤β}。首版 d=0（仅几何自由，不能冒充车辆 .05m 安全余量），ρ=min(.05m,最小正分离距离/4)，数值保守内缩1e-8m。所有与 B⊕d 相交的占据方块必须覆盖；未处理的远障碍需用包围盒不相交证明。首版逐 cell 分离；不跨空隙取障碍凸包。

共同端点附近 B 应包含小圆盘；独立 polygon-cell SAT 审核全部 C 的原始栅格相交。portal=C_j∩C_j+1，单位法向 Chebyshev LP max r s.t. A g+ r≤b。非退化验收 r≥min(.005m,.1·地图分辨率)，area>1e-10m²。小于门槛标 CORRIDOR_DEGENERATE，不声称物理无路。

## Portal 与 8 段映射（重要修订）

M 个区域，M−1 个 portal 点 x_j；x0=start、xM=goal，每个 [x_j,x_j+1]⊂C_j。采用冻结正长度尺度 ℓ_j=max(种子段长,.1m)，不以优化中点距作除数。QP 用 d_j=(x_j+1−x_j)/ℓ_j：Σ||x_j+1−x_j||²/L0² + .1Σ||d_j−d_j−1||² + .01Σ||x_j−center(P_j)||²/L0²；首版固定权重是试验初值。受 portal 线性约束，SciPy SLSQP 解此小凸 QP，独立检查 primal residual，失败保留已验证 Chebyshev 折线。无需新 QP 依赖。

中线首版是安全折线；圆角二次 Bezier 是 C2 子验收的可选增量，仅当三个控制点均属于同一分配 C 时利用凸包性质认可整段。不能把几何平滑误称为可执行轨迹。点积负仅用于标记，不凭点积删真实回头弯。删点/shortcut 都必须重新证明所分配走廊和原始地图安全。

中线段长累计归一化为 τ_j∈[0,1]。冻结走廊占用区间 [τ_j,τ_j+1]，曲率坐标 σ=s/S。用所有 j/N 和 τ_j 的并集分割约束区间；走廊切换允许落在一段 Hermite 内，切换点同时满足两个半空间集合。每个区间布置9点Gauss及两端，追加 C2 NLP 半空间约束。N不增加，h_j不变成变量。若映射不适合真实曲线，报告 FIXED_ALLOCATION_LIMIT，不宣称不可行；C3 至多一次边界重分配。软模式只用于诊断，默认 corridor_hard。

连续包含证明：对单位 n、区间[a,b]，中点 s_c，n·p(s)≤n·p(s_c)+(b−a)/2+ε_position（因||p'_F||=1）。满足 b_plane 则通过；否则二分，最多深度20/总区间20000/最小宽1e-5m，触限 UNCONFIRMED，不能以稀疏点通过代替。配点可加入小于 portal 净空的可配置内缩提高可证性；未证实不发布 corridor_hard 成功。这只证明 F，在原三车 OBB 终审之前不称运输安全。

## 目标函数与有限恢复

J=wL S/L0+wκ L0∫κ²ds+wη L0³∫η²ds+wu L0⁵∫u²ds+wg mean(||p−guide||²/L0²)+wc Jclear；L0=max(端点距离,1m)，先试(1,.01,.02,.002,.1)，wc首试1，Jclear=mean(max(0,target+support−clearance)^2/L0²)，target/field/support沿用既有身体代理。以上待实验，不能预设提升；旧组维持原5e5/100。新组不得继续叠加强跟踪。

最多3条路线、6个初值、60s请求软预算、原30s求解配置；回退只 auto+allow 且主求解非超时失败+剩余预算>0时允许一次 SLSQP。C3 只允许一次按类型恢复，替换原最多2次碰撞重试，不新增叠加恢复循环。保留已通过三层审核候选，未审候选不能胜出。时间律仍全路径五次，不开发时间最优。

## 可视化与工程约束

现有 build_markers/build_hold_markers 与 ROS playback 接入 HUD。OverlayText 本机 /opt/ros/noetic/share 未发现，V0 必须实际 rospack/rosmsg 查证；首版固定标准视口像素left/top，提供参数并保存截图尺寸，不声称窗口缩放自动右对齐。不开发自定义插件。跟车仅car id/A/v，静止无法确认模式时写STATIC；全局长文字改HUD，F地标保留。

走廊展示只消费算法实际数据，debug String 负载复用 result.search，包含plan/formation/backend/grid/context/route/corridor数据摘要；执行轨迹仍原消息。不能为每层造新schema。无model时不能显示FEASIBLE，新admit清旧图层，busy拒绝保持当前合法播放。V3复用当前generation/admission，不新增状态机。

## 每阶段的简洁性要求

每个新增抽象/检查要指出实际复现问题。边界验证一次，内部用已有类型；不增无意义wrapper/manager/factory/重复validator/通用注册器；不加无证据fallback/宽泛吞错/无限重试/海量日志；不做无关重构。不可删除积分证书、曲率/轮系限制、连续OBB、plan/context/model绑定和失败禁止误播。各任务书再次规定此约束。
