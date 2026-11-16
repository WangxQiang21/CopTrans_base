# 虚拟运输集群连续 A 模式：V1 决策与硬合同

日期：2026-11-15。状态：**任务文档已编制，G0–G5 均未启动**。本文取代原审核稿中“仍需确认”的同名技术选项，不改写历史审核和 FR06 证据。

## D01 基线、所有权和阶段门禁

- 正式基线：`snapshot/formation-v02-fr06-20261115`，`567bdecba9676313da3bc9a97b82f57b5b569a2b`。
- 未来分支：`feature/virtual-rigid-formation-a`；建议未来隔离目录 `/home/wq/whut_thesis/expv3_multi_virtual_a`。
- **本轮只编制 Markdown，不创建分支或 worktree，不实施任何阶段。** 以后用户授权准备工作区时由 Codex 负责保存旧状态、创建和核验隔离 worktree，并复制本目录及源审核资料；Claude 不操作原仓库或旧 worktree。
- Claude 只执行当次被明确下达的一阶段任务；Codex 独立审查差异、重跑必要检查并写 `CODEX_ACCEPTANCE.json`。Claude 的自述、测试全绿或自写 ACCEPTED 不构成通过。
- 顺序为 `G0 → G1 → G2 → G3 → G4 → G5`。前一阶段存在与实际源码/证据哈希绑定的 Codex ACCEPTED，且用户明确下达下一阶段后，才可继续。
- 未额外授权时不提交或推送 Git。不重置、清理、覆盖、stash 或丢弃已有修改。遇到不明修改、分支冲突或保护哈希漂移，停止相关操作并报告。

## D02 研究对象和不变几何

- 三辆 Ranger Mini 2.0；以现有 `transport_formation.yaml` 为配置真源，车辆 ID/偏移为 `1:(1,0), 2:(0,1), 3:(0,-1)`，单位 m。车体 `0.738 × 0.500 m`。
- 直接复用 `FormationModel`、`FormationPose`、`VehicleOffset`、`VehicleShape`。新后端校验配置属于本研究构型，不另建一套编队参数源。
- `p_i=p_F+R(psi_F)r_i` 是构造性硬约束。刚性指车辆中心，不要求三车车头一致，不证明运输连接机构的受力或跟踪能力。
- F 前向为局部 +x、左侧为 +y，逆时针为正。各车车体航向由偏移曲线切向恢复，不是三个独立自由优化变量。

## D03 模式、初态和目标策略

- 只研究统一 A、正向、F 沿集群局部前向的子模型。O、S、倒车、自动 S 对齐均不实施；同一 A 内部控制结点不是停车点。
- 默认初态：全车静止、`theta_i=psi_F`、理想 A 直行轮系，故 `kappa_F(0)=eta_F(0)=0`。输入航向必须逐车读取并验证，不得先覆盖成 psi 再验证。
- 旧请求没有真实初始速度、模式和轮系状态。离线请求必须显式带 `InitialAssumption(stationary=True, mode='A', wheels_straight=True, source=...)`，这是实验初态声明，不是传感器测量。
- `source`首版只允许`offline_fixture`、`rviz_reset`、`verified_terminal`；附可选`context_id:str=''`。离线测试用offline_fixture，ROS两种来源必须有本会话可核对的非空context_id并进入请求摘要，不能由调用者随意自报而绕过节点上下文检查。
- ROS 新实验的手工 2D Pose Estimate 表示显式重置虚拟实验；普通后续目标必须继承权威终态。即使三车终点车头恰好平行，也须检查 `kappa_i(T)`/理想轮系是否为零，不能仅凭 TF 车头推断已回直。没有明确重置且终态不兼容时，拒绝 `UNSUPPORTED_INITIAL_STATE`，不加 S、不瞬变车头或轮系。
- 默认 `GoalPolicy.FREE`：只固定 F 的目标位置/方向，不约束终端 `kappa_F/eta_F` 为零；三车终端车头按模型恢复。
- 显式实验选项 `GoalPolicy.STRAIGHT`：额外约束 `kappa_F(S)=eta_F(S)=0`。这是用户选定的更小可行域，须写入原请求摘要、模型和结果。
- 首版不支持任意固定/混合自由标志的三车目标车头；适配层须显式拒绝 `UNSUPPORTED_GOAL_POLICY`。不能将旧请求的固定车头静默改为自由，也不能自动把“固定平行车头”升级为“轮系回直”。
- 精确相同起终位姿且初态合法：允许显式 `NO_MOTION`，长度/时长均为零，静止 A 直行轮系。位置相同但目标方向不同不属于零运动，找不到候选时不是物理无解证明。

## D04 数学模型和正常分支

`s` 为 F 弧长 m；`kappa[1/m]`、`eta=kappa'[1/m²]`、`u=kappa''[1/m³]`、`g=kappa'''[1/m⁴]`。prime 均对 s。

\[
p_F'=(\cos\psi,\sin\psi),\quad\psi'=\kappa,\quad\kappa'=\eta,\quad\eta'=u.
\]

\[
A_i=1-b_i\kappa,\quad B_i=a_i\kappa,\quad D_i=A_i^2+B_i^2,\quad Q_i=\sqrt{D_i},
\]
\[
\theta_i=\psi+\operatorname{atan2}(B_i,A_i),\quad H_i=\theta_i'=\kappa+a_i\eta/D_i,
\quad C_i=\kappa_i=H_i/Q_i.
\]

连续展开角度；只在比较误差时 wrap。必须满足正常前向分支 `A_i>0`、`D_i>0` 和每车实际曲率限制；不以任意 epsilon 截断分母后继续认证。

当前构型 `ell=1 m`、`r_min=0.810 m` 给出 `|kappa|<=1/(ell+r_min)=0.5524861878 1/m`。另须检查前车 eta 区间或通用 `|C_i|<=1/r_min`，不能只检查侧车/参考点曲率。

单车 A 使用现有 `wheel_configuration(Action(0,0,C_i), MotionLimits)` 对应的中心零侧偏、对称反相前后转向理想模型，不泛化为任意中心侧偏的 4WS 模型。

## D05 曲率表示和 C2

- 优先使用现有 `ControlKnot(s,value,derivative)` 和 `MotionBlock(mode=0,direction=1,length=S,knots=...,slope_policy='fixed')` 的 Hermite 数学；这只是 F 的内部标量曲率组件，不是三车规划结果。
- 在新 wrapper 中解析三次系数、三阶导数、左右极限和 C2 连接；不修改旧 `continuous_geometry.py` 的语义，不使用会重算导数的 monotone 策略。
- 对段 j，`q=(s-s_j)/h_j`，`k(q)=c0+c1*q+c2*q²+c3*q³`：
  `c0=k_j; c1=h_j*eta_j; c2=3*(k_{j+1}-k_j)-h_j*(2*eta_j+eta_{j+1}); c3=2*(k_j-k_{j+1})+h_j*(eta_j+eta_{j+1})`。
- C2 等式：`(2*c2_j+6*c3_j)/h_j² = 2*c2_{j+1}/h_{j+1}²`。共享 k/eta 结点已保证 C0/C1；`g=6*c3/h³` 每段有界，可以跨结点跳变。
- 内部左右 k/eta/u 容差分别 `1e-9 1/m`、`1e-9 1/m²`、`1e-8 1/m³`。不强制端点 u=0；端点静止并不需要该额外条件。G1 sin³ 基准恰有端点 u=0。
- 只有证据表明表达能力不足，才提交五次/其他表示的变更提案，不自行切换模型或放宽连续性。

## D06 理论车辆限制及参数假设

直接复用现有 `FormationTimingLimits`。下列为基线默认值，运行时须序列化实际值：

| 量 | 值 | 语义 |
|---|---:|---|
| v_max | 0.8 m/s | 每车中心速度 |
| a_max | 0.6 m/s² | 每车二维中心加速度模长，非仅纵向标量 |
| jerk_max | 2.0 m/s³ | 每车二维中心 jerk 模长 |
| yaw_rate / yaw_accel / yaw_jerk | 0.8 / 0.8 / 3.0 | rad/s、rad/s²、rad/s³ |
| lateral_accel_max | 0.6 m/s² | `abs(C_i)*v_i²` |
| wheel_speed_max | 1.0 m/s | 理想轮接地点速度 |
| steer_angle_max | pi/2 rad | 当前软件参数，不是已校准物理转角 |
| steer_rate / steer_accel | 0.8 / 1.5 | rad/s、rad/s²，规划层理想轮系变化 |
| wheelbase / track | 0.494 / 0.364 m | 中心轮系几何 |
| r_min_ackermann | 0.810 m | 单车中心最小半径的工作假设，厂商参考点未标定 |

- 不把 CAN ±40°当单轮角限制，不做硬件标定、轮角命令、控制器或 Gazebo。
- 原 `max_block_duration_s=120`、mode setup 等字段继续保留在旧限值类型；新 A 全路径不把 120 s 当物理硬限制。新工程预算单列为 TimingConfig，超预算报告失败，不降低车辆约束。
- 碰撞基准裕量 `0.05 m`、扫掠角点步长 `0.01 m`，与旧基线一致；数值误差需额外保守计入，不得从 0.05 m 中扣掉。

## D07 自主求解和时间律

- G1：解析双移线和镜像是人工数学基准，不是规划器；`K=.30 1/m,L=6 m` 正例，`L=.5 m` 前车超限反例必测。
- G2：真正的端点自主优化，默认 8 个等长 Hermite 区间、总长 S 和结点 k/eta 为变量；有限自动多初值，三车硬约束和 C2 等式，独立审核后才 `GEOMETRY_FEASIBLE`。该状态永不发布为可执行。
- 不在运行请求中传可行参考、G1 曲线或每车独立路径。固定参考只供测试证明存在解，求解器不得读取它。
- G3：`s=S*(10*tau³-15*tau⁴+6*tau⁵), tau=t/T`。全路径一条公共时间律，调整 T 满足三车全部限制；不做时间最优、不逐结点停车。初态已为合法直行 A，无初始模式切换或 wheel setup，首版额外启动驻留固定为 0。
- G4：少量地图路线只作拓扑/初值引导，优化与终审恢复真实三车。G2/G4 优化器 success 仅是候选，绝非认证。

## D08 权威模型、数值证明和安全分层

- 新 backend：`virtual_rigid_a`。几何模型：`virtual_rigid_a.path.v1`；时间模型：`virtual_rigid_a.timed.v1`；播放信封语义：`virtual_rigid_a.playback.v1`。旧字符串完全不改含义。
- 权威内容：原请求/目标策略、formation 和形状、实际限值、曲率结点/段长、积分版本及误差预算、统一 T、模型身份和摘要。位置样本、绘图、缓存不是权威数据。
- 求值管线只有 `t -> s(t) -> q_F(s) -> p_i/theta_i/C_i -> v_i/omega_i`；不得独立线性插值三车中心，不得把单车 `ContinuousPlan` 当三车证书。
- x/y 需数值积分；使用可复现积分和有依据的余项界，明确浮点舍入/超越函数误差假设，不声称形式化验证了所有 libm 运算。只比较两次网格差或使用 quad 的估计误差不是连续域证明。
- 在相应数学/浮点假设下，最终曲率和动态检查用带外扩的区间/多项式包络，不能仅取稠密采样峰值。不能确认时 `BOUND_UNCONFIRMED`/`AUDIT_BUDGET_EXHAUSTED`，不可发布。
- 证据等级独立：`MATH_VALIDATED`、`GEOMETRY_FEASIBLE`、`DYNAMIC_FEASIBLE`、`COLLISION_FEASIBLE`、`ROS_PLAYBACK_VERIFIED`；不得互相替代。地图未检查不能输出完整 FEASIBLE/EXECUTABLE。
- 程序报告必须绑定 request/path/model/limits/grid 摘要。JSON 中 `passed:true` 和字符串 certification 本身不是可信证明，Codex须重新计算。
- request_sha256固定表示物理请求；G4/G5另用planning_context_sha256绑定地图和planner/timing/collision配置。完整上下文在timed model及相关审核构建前固定，禁止审核后修改模型再沿用原报告。

## D09 ROS 和基线保护

- 保留 `uniform_v02` 和 `legacy`；默认后端不变。单次启动最多一个轨迹发布后端，串行、不抢占、不动态重规划。
- 优先复用现有 `FormationPlanRequest`、`FormationPlaybackPlan` 消息字段；新请求摘要和初态声明放新 model JSON。首版不改消息字段/MD5，也不借用 Header.seq 传业务状态。
- G5 才允许在原节点插入有限后端 hook，扩展 launch/run_demo、bridge、模型验证和帧求值分派；不复制 1200 行规划节点，不建立泛化插件框架。
- 收到新的合法请求并开始规划时清理旧可执行计划；busy 拒绝保留当前合法计划。失败/错误摘要/未知版本/过期结果不可播放。普通续接使用真实终态，必要时要求显式重置新实验。
- 冻结 `evaluation/formation_refactor_v02/**`、旧测试预期、旧 maps 和车辆配置。新结果只写 `evaluation/virtual_formation_a/Gx/<run-id>/`，run-id 必须唯一；旧测试默认写历史目录的路径通过已有环境变量重定向。

## D10 扩展边界和变更纪律

暂不实现：O/S/倒车、非零初始轮系、任意固定终点车头、时间最优、NLP+搜索通用平台、在线抢占、动态障碍、Gazebo、控制/CAN、学习算法、实车验证。

保留的扩展入口是 `InitialAssumption`、`GoalPolicy`、版本化 profile 类型、TimingConfig、PlannerConfig、模型/播放分派；首版只接受本文枚举值，未知值显式失败，不为未来先写空实现。

若需要改变几何构型、运动方向、初态域、连续性、车辆限制、认证含义或模型版本，停止相关开发，提交“事实—冲突—最小替代—影响测试”给用户/ChatGPT；普通实现缺陷在既定合同内修复。

新颖性和性能尚未验证。旧 FR06 宽通道超时/混合模式成功不是新 A 的能力或性能保证。比较须披露共同可行域和全部失败，不用强制零切换作为唯一优越性结论。

各阶段完整任务见 `CLAUDE_G0_TASK.md` 至 `CLAUDE_G5_TASK.md`；主方案见 `VIRTUAL_FORMATION_A_IMPLEMENTATION_PLAN_FOR_CLAUDE.md`。
