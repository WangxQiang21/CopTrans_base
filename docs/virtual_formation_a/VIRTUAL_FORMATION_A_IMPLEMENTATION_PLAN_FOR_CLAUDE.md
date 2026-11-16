# 虚拟运输集群连续 A 模式：Claude 分阶段实施主方案

2026-11-15；修订版 V1。**本文件是开发任务合同，不表示 G0–G5 已开始或验收。**

## 1. 来源、效力和本轮边界

已完整阅读原稿 `/home/wq/下载/Codex_Review_Plan_Formation_FR06_Base.md` 与[独立审核报告](../VIRTUAL_FORMATION_A_FR06_TECHNICAL_REVIEW_20261115.md)。本方案落实用户随后确认的十项决策，取代原稿中的待选项；保留原稿和审核报告不改写，以便追溯。

正式基线为 `snapshot/formation-v02-fr06-20261115`，提交 `567bdecba9676313da3bc9a97b82f57b5b569a2b`。当前编制工作区仍为 `/home/wq/whut_thesis/expv3_multi_v02`。未来开发分支 `feature/virtual-rigid-formation-a`，建议未来 worktree `/home/wq/whut_thesis/expv3_multi_virtual_a`；**本轮不创建它们、不运行阶段、不改代码或实验资料、不提交**。

使用方式：先读[决策合同](VIRTUAL_FORMATION_A_DECISIONS.md)，只将本次被授权阶段的 `CLAUDE_Gx_TASK.md` 交给 Claude。Claude完成后停止，Codex检查差异和真实证据。Codex ACCEPTED 加用户明确下达下一阶段，是继续的共同条件。文档中的新函数都是拟新增接口，不能假装当前已存在。

## 2. 已有能力、复用边界和目标

【代码事实】当前 FR06 是 Python 统一模式有限原语搜索、逐边 rest-to-rest 公共时间、独立审核和权威模型回放。单车 V2 Hermite/IPOPT/SciPy 基础已在同一分支，不必切回单车分支。

【历史实验事实】2.8 m 冻结拐角三次成功、3.2 m 三次预算耗尽；参考含 O/S/A，不能当新 A-only 可行解。旧证据仍是独立基线。

【尚未验证】新 A-only 的自主求解成功率、窄道性能和运行时间。下文预算是工程边界/验收目标，不是性能承诺。

目标系统：

```text
现有三车请求 + 配置 + 地图
  -> 新后端初态/目标策略校验
  -> G2 自动多初值集群曲率优化（G4加入少量地图路线引导）
  -> 独立几何审核
  -> G3 全路径一条五次公共时间律
  -> 独立三车动态审核 + G4 真实 OBB 保守扫掠
  -> 绑定原请求/地图/限值的完整权威模型
  -> G5 现有 ROS 原子消息与公共时钟三车回放
```

不使用三个独立规划器，不开发控制、CAN、Gazebo、实车跟踪。车头允许不同，但由集群路径恢复，不是三个独立优化自由度。

### 2.1 真实源码复用表

路径前缀 `P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`；行号对应精确基线。

| 源码/入口 | 复用或保护方式 |
|---|---|
| `P/formation.py`：`FormationModel.centers:240`、`vehicle_poses:255`、`rigidity_residual:290` | 直接复用模型和映射，不复制偏移合同。 |
| `P/transport_contract.py`：`contract_from_mapping:75`；`transport_formation.yaml` | 直接读取权威 ID、offset、形状、策略；不修改默认旧实验配置。 |
| `P/formation_request.py`：`validate_formation_request:372`、`FormationRequestSnapshot:169` | 复用规范化请求，再做新能力域检查。 |
| `P/continuous_geometry.py`：`ControlKnot:19`、`MotionBlock.control:101`、`control_integral:90`、`sample_block:287` | 复用 Hermite/积分数学；新 wrapper 提供 C2、第三导数和误差界。旧 `ContinuousPlan` 不作三车结果。 |
| `P/continuous_geometry.py`：`fit_continuous_endpoint:436` | 参考块长/结点终点拟合方法；它没有三车限制，不直接承担新认证。 |
| `P/continuous_optimizer.py`：`_optimize_continuous_geometry_ipopt:347`、`optimize_continuous_geometry:780` | 借用 CasADi/SciPy 求解经验；现有函数是给定拓扑局部形变，不是新自主入口。 |
| `P/formation_collision.py`：`static_check:408`、`AnalyticSweepSegment:89`、`swept_check_analytic:601` | 原样调用真实 OBB/SAT/保守覆盖；新增曲线回调/导数上界适配。 |
| `P/retiming.py`：`wheel_configuration:101`、`quintic:127`；`P/formation_retiming.py`：`FormationTimingLimits:119` | 复用理想轮系、基础多项式、实际动态限值；不沿用常控制导数和逐块停车。 |
| `P/continuous_retiming.py`：`_quintic_coefficients:177`、`_quintic_state:189` | 可参考基础算子；其 `_lookup_pose:652` 的位置线性插值不作新权威模型。 |
| `P/uniform_quality.py`：`audit_uniform_timed_plan:3378`；`P/uniform_trajectory.py`：`state_at:743` | 保留旧实现和类型；借用审核分层/身份绑定原则，新模型独立实现。 |
| `P/formation_playback.py`：`verify_uniform_playback_plan:542`、`frame_at:818`、`build_markers:1159` | G5扩展有限模型分派，旧验证函数与 Marker 行为保留。 |
| ROS `formation_uniform_planner_node.py`：`on_request:630`、`_finish_success:918`、`_finish_failure:887` | G5插入有限后端 hook，复用串行状态机，不复制整节点。 |
| `catkin_ws/src/fwis_v3_optimizer/setup.py` | 当前只显式登记根包；G1必须加登记新子包，防止仅 PYTHONPATH 测试通过而 catkin 运行导入失败。 |

### 2.2 文件责任与阶段白名单

新包 `P/formation_continuous_a/`，不预建空功能：

| 文件 | 首次阶段 | 唯一职责 |
|---|---|---|
| `__init__.py`、`contracts.py` | G1 | 新模型有限数据合同/失败枚举；复用旧 FormationModel/limits。 |
| `geometry.py` | G1 | C2 Hermite wrapper、F积分、三车空间 jets、诊断 profile。 |
| `bounds.py` | G1 | 新曲率模型所需的局部多项式/区间及积分余项界；不是通用证明框架。 |
| `optimizer.py` | G2 | 有限自动初值、端点 NLP，后续可接路线约束；没有 ROS。 |
| `audit.py` | G2 | G2几何、G3动态、G4碰撞分层独立重建和绑定；不信任求解器/采样字段。 |
| `trajectory.py`、`timing.py` | G3 | 完整权威时间模型/序列化；统一 T 的选择。 |
| `collision.py`、`pipeline.py` | G4 | 调用旧碰撞引擎的适配；少量路线引导和完整可执行流水线。 |
| `ros_adapter.py` | G5 | 无 ROS 的新后端请求/字段转换 hook，使用原消息形状。 |

工具：G0 `tools/virtual_a_baseline.py`；G1起 `tools/virtual_a_benchmark.py` 按阶段增加子命令；G4另加 `tools/virtual_a_scenes.py`。新测试 `tests/test_virtual_a_*.py`，G5 `tests/ros_virtual_a_smoke.py`。这些均为未来新增，不是当前存在的工具。

G1–G4只允许新包、新工具、新测试及新证据；G1另允许 `setup.py` 子包登记。G5才允许改原 ROS 节点、播放模块、bridge、launch、run_demo 和安装登记的有限接入点。旧 `uniform_*` 核心、`formation_geometry/kinematics/retiming/quality/collision.py`、单车核心、旧测试、车辆 YAML、旧地图和 `evaluation/formation_refactor_v02/**` 全程保护。

若需要白名单外修改，先给 Codex说明真实调用冲突及最小差异，不能先改后补理由。

## 3. 公共数据合同和错误语义

下面是建议签名，阶段实现时按本名创建；若普通语言类型细节需调整，先更新接口清单，保持后续调用一致。

```python
InitialAssumption(stationary: bool, mode: str, wheels_straight: bool,
                  source: str, context_id: str='')     # 明确实验声明来源
GoalPolicy = FREE | STRAIGHT
CurvatureJet(kappa: float, eta: float, u: float, g: float)
VehicleSpatialJet(vehicle_id, x, y, theta, curvature,
                  p_s, p_ss, p_sss, theta_s, theta_ss, theta_sss,
                  curvature_s, curvature_ss)           # 单位按 s 的阶数
FormationSpatialState(s, qf: FormationPose, curvature: CurvatureJet,
                      vehicles: tuple[VehicleSpatialJet, ...],
                      position_error_bound_m, angle_error_bound_rad)

VirtualARequest(plan_id, formation_id, model: FormationModel,
                vehicle_shapes: Mapping[int, VehicleShape],
                start_qf: FormationPose, start_headings: Mapping[int,float],
                goal_qf: FormationPose, initial: InitialAssumption,
                goal_policy: GoalPolicy, limits: FormationTimingLimits)

AuditReport(scope, passed, reason, failure_location, checks, metrics,
            request_sha256, path_sha256=None, model_sha256=None,
            limits_sha256=None, grid_sha256=None,
            planning_context_sha256=None, assumptions=())
GeometryResult(status, path, request_sha256, audit, attempts, wall_time_s)
VirtualAResult(status, reason, detail, first_failure_layer,
               request_sha256, plan, projection, evaluation,
               wall_time_s, attempts, planning_context_sha256=None, search=None)
```

- 所有 identity/映射按 vehicle_id 排序；保留 formation_id/plan_id，不依赖输入列表顺序。JSON拒绝 NaN、重复ID、非有限值、未知版本、未知枚举和不合法多余字段。
- 初态source只允许`offline_fixture/rviz_reset/verified_terminal`；后两者需节点已核对的非空context_id。不能在ROS输入中接受任意自报offline_fixture作为已观测轮系的替代。
- `request_fingerprint(request)`：canonical JSON、排序 key、UTF-8、SHA-256；包含真实车头、目标策略、初态声明、编队/形状和实际限制。schema/version进入摘要；排除墙钟耗时和绘图缓存。
- G4/G5另计算`planning_context_sha256=SHA256({request_sha256,grid_signature,planner_config,timing_config,collision_config})`。物理request摘要不因加入地图而改变，完整规划上下文另绑定，防止G2/G3/G4同名字段含义漂移。模型与对应阶段报告同时带两者。
- path 摘要绑定几何及物理request摘要；timed model 摘要再绑定 T、数值求值版本和planning_context。G4须在构造timed model及动态审核**之前**固定地图/配置上下文，不能通过审核后才给JSON加grid导致模型摘要变化。证书绑定最终摘要，禁止重用另一地图/限值/目标策略的审核。
- `GeometryResult.status`：`GEOMETRY_FEASIBLE | NO_MOTION | INVALID_REQUEST | NO_ACCEPTED_CANDIDATE | BUDGET_EXHAUSTED | NUMERICAL_FAILURE`。几何结果从不 executable。
- 完整 `VirtualAResult.executable` 只在 G4后，对匹配原请求和地图的几何/动态/碰撞三个报告均 passed 且有权威 timed plan 时为真。G3只允许离线 `DYNAMIC_FEASIBLE`，ROS不得发布。
- 常见 reason：`UNSUPPORTED_INITIAL_STATE`、`UNSUPPORTED_GOAL_POLICY`、`FORMATION_MISMATCH`、`CURVATURE_LIMIT`、`NON_C2_PROFILE`、`BOUND_UNCONFIRMED`、`AUDIT_BUDGET_EXHAUSTED`、`MAP_OBB_COLLISION`、`DYNAMIC_LIMIT`、`MODEL_BINDING_MISMATCH`。
- 不向目标误差/碰撞/动力限制注入不同“宽松测试模式”。不可确认不等于已违反，更不等于物理无解；报告必须区分。

## 4. 统一数学实现规范

### 4.1 集群、三车和初终边界

初态为静止 A 直行，真实 `theta_i≈psi_F`，`kappa(0)=eta(0)=0`。新构型和单车理想参数见决策 D02–D06。

\[
p_i=p_F+R(\psi)r_i,\quad A_i=1-b_i\kappa,\ B_i=a_i\kappa,
\quad D_i=A_i^2+B_i^2,\ Q_i=\sqrt{D_i},
\]
\[
\theta_i=\psi+\operatorname{atan2}(B_i,A_i),\quad
H_i=\kappa+a_i\eta/D_i,\quad C_i=H_i/Q_i.
\]

所有车 `A_i>0`，`D_i>0`，`|C_i|<=1/.810`；本构型可先利用 `|kappa|<=.5524861878` 快检，但前车 eta 约束仍须检查。

内部 `kappa/eta/u` 连续，g 分段有界。端点 `GoalPolicy.FREE` 不加 k/eta=0；STRAIGHT显式增加二式。目标 F 位置/方向始终是硬约束。

### 4.2 Hermite 和 C2 的可直接编码表达

对段 `[s_j,s_{j+1}]`，h为段长，q为归一化进度：

```text
c0=k0; c1=h*eta0
c2=3*(k1-k0)-h*(2*eta0+eta1)
c3=2*(k0-k1)+h*(eta0+eta1)
k=c0+c1*q+c2*q²+c3*q³
eta=(c1+2*c2*q+3*c3*q²)/h
u=(2*c2+6*c3*q)/h²; g=6*c3/h³
C2_j: (2*c2_j+6*c3_j)/h_j² - 2*c2_next/h_next² = 0
```

复用 `MotionBlock.control/control_integral` 并对照独立系数计算；不能用其默认“C1”描述推断已经满足C2。导数在节点提供 `side='left'|'right'`，默认内部取右、终端取左；报告同时检查左右。

### 4.3 三车空间导数

在 F 局部坐标中写三个向量，实际世界向量是 R(psi) 乘之：

```text
z1=(1-b*k, a*k)
z2=(-a*k²-b*eta, k-b*k²+a*eta)
z3=(-k²+b*k³-3*a*k*eta-b*u,
     eta-3*b*k*eta+a*u-a*k³)
p_s=R*z1; p_ss=R*z2; p_sss=R*z3
D_s=2*((a²+b²)*k-b)*eta
D_ss=2*((a²+b²)*eta²+((a²+b²)*k-b)*u)
Q_s=D_s/(2*Q)
Q_ss=D_ss/(2*Q)-D_s²/(4*Q³)
H_s=eta+a*(u/D-eta*D_s/D²)
H_ss=u+a*(g/D-2*u*D_s/D²-eta*D_ss/D²+2*eta*D_s²/D³)
C_s=H_s/Q-H*Q_s/Q²
C_ss=H_ss/Q-2*H_s*Q_s/Q²-H*Q_ss/Q²+2*H*Q_s²/Q³
```

这些是新几何公式，不调用旧常 pivot `_vehicle_derivatives` 替代。保守动态模长可在局部坐标计算，避免为区间模长额外引入三角函数依赖。

### 4.4 G2 优化问题和初值

默认 N=8等长区间，变量 `S,k_0..k_N,eta_0..eta_N`，`h=S/N`；先固定N，不同时开发自适应网格。初始 `k0=eta0=0`，内部C2，目标F三项约束，STRAIGHT另两式，三车曲率和前向分支硬约束。

默认搜索域：`S∈[max(N*.05,d), min(60,max(12,3*d+10))] m`、`|eta_j|<=2 1/m²`、k受侧车界约束。它们是有限数值搜索域，不是物理可行域的完备描述；超域不得宣称数学无解。d为端点距离，d=0非零转向问题仍可尝试S>0。

目标无量纲化：令 `l0=max(1,d)`，默认最小化 `S/l0 + .02*l0³*∫eta² ds + .002*l0⁵*∫u² ds`。实际权重配置入证据，不能为单个测试偷偷调整。

最多6个自动初值，使用S长度倍率及左右形状分支。可用 `CubicSpline`/线性等式投影让初值先满足C2和初始夹持；不让被测优化器读取存在性参考曲率。初值投影、求解器及最终积分统一单位/角度分支。

默认总软预算30 s/请求、120次迭代/初值；回调和初值边界检查deadline，记录实际超时。不可将不能中断的单次库调用冒称硬实时保证。CasADi/IPOPT为优先环境，缺失/插件不可用明确记录；SciPy SLSQP可作为显式标注后备，不能吞掉任意异常后假成功。

优化内先用固定Gauss积分和每段至少9个配点，候选再用独立更严格积分、C2及整段界审核。初始版本不开发三车位姿联合配点、时间变量联合NLP或FR06搜索器重写。

### 4.5 区间和位置积分误差：不留空白“自行认证”任务

`bounds.py`只需支持归一化三次多项式及其导数的区间Horner/二分、外扩基本运算和非负平方根。遇到跨零分母先细分，仍无法确认则失败；不得用`max(sample)`当sup。G2用它界定D和C；G3加入H/Q、角/轮系动态表达；通用算子可共享，独立审核重新从原始结点生成系数。

位置积分保留现有Gauss数学用于快路径；最终权威模型固定为可重建的分片求积。最小可执行的误差方案是分段复合Simpson加四阶余项界。对 `f=exp(i*psi)`：

\[
|f^{(4)}|\le G+4KU+3E^2+6K^2E+K^4=M_4,
\]

K/E/U/G分别是该子区间 `|k/eta/u/g|` 的上界。宽H的Simpson面板每个坐标余项 `<=H^5*M4/2880`；二维位置取两坐标合成的保守界。不能跨Hermite节点套一个光滑面板；任意s查询采用完整前缀面板加该面板内的有界部分积分，缓存不是权威几何。

默认位置总误差预算 `1e-7 m`、角误差预算 `1e-9 rad`，含舍入与超越函数数值保护。默认支持连续展开 `|psi|<=100 rad` 的计算域；更大输入明确拒绝数值域，不擅自wrap整条路径。外扩和libm误差假设写入`numerics_version`，测试用独立严格quad/解析基准核查保护量；不把经验差值称为严格浮点形式化证明。若实现无法给出上述余项/保护依据，返回`BOUND_UNCONFIRMED`，交Codex审核最小修正，不能降级为“采得够密”。

误差配置初值：`position_roundoff_guard_m=1e-10`、`angle_roundoff_guard_rad=1e-11`，每次最终加法/区间运算向外扩；需将结点系数运算误差及sin/cos调用误差计入保护量，不能把这两个经验常数本身当证明。G1报告必须说明实际累积误差模型和支持范围；若目标误差预算容纳不了有效保护量，失败并报告，不缩小保护量求通过。

几何曲率极值可用区间C直接细分确认，或检查 `(k*D+a*eta)^2-kmax²*D³<=0` 的多项式包络。默认区间最大深度20、总叶数200000；耗尽可失败，不提高预算掩盖病态。先保守后局部细分收紧，不要求开发全功能符号证明系统。

### 4.6 G3 公共时间和轮系

\[
s=S h(\tau),\quad h=10\tau^3-15\tau^4+6\tau^5,\quad\tau=t/T.
\]
\[
w=Sh'/T,\quad a=Sh''/T^2,\quad j=Sh'''/T^3,
\quad v_i=Q_iw,\ \omega_i=H_iw.
\]

```text
velocity=p_s*w
acceleration=p_ss*w²+p_s*a
jerk=p_sss*w³+3*p_ss*w*a+p_s*j
yaw_rate=H*w
yaw_accel=H*a+H_s*w²
yaw_jerk=H*j+3*H_s*w*a+H_ss*w³
lateral_accel=abs(C)*(Q*w)²
```

每个理想轮中心 `(x_w,y_w)=(±.247,±.182)`，令 `m=1-y_w*C,n=x_w*C,Fw=sqrt(m²+n²)`，轮角`delta=atan2(n,m)`，轮速`Fw*v_i`。规划约束导数：

```text
Ew=m²+n²; Ew_C=-2*y_w+2*(x_w²+y_w²)*C
delta_s=x_w/Ew*C_s
delta_ss=x_w/Ew*C_ss-x_w*Ew_C/Ew²*C_s²
delta_dot=delta_s*w
delta_ddot=delta_ss*w²+delta_s*a
```

轮角/轮速由旧`wheel_configuration`交叉核对；只输出规划层诊断，不生成执行命令。

以T0=1 s计算各动态量的全域上界U。按阶数选择 `T >= max(U_speed/limit, sqrt(U_acc/limit), cbrt(U_jerk/limit), ...)`，再乘1.01数值安全系数；静态轮角/曲率违规不能靠加T修复。独立审核按时间区间重算，必要时二分收紧。`TimingConfig.max_total_duration_s=600` 是新工程预算，超限报告`TIMING_BUDGET_EXCEEDED`；不是物理无解。零运动另有显式hold，不应用S=0运动公式。

### 4.7 G4 保守扫掠

对 `[s_a,s_b]`，`s=s_a+Delta_s*lambda`，lambda在[0,1]：

```text
center_speed_bound_m_per_lambda = Delta_s
formation_rate_bound_rad_per_lambda = Delta_s * sup(abs(k))
body_rate_bounds_rad_per_lambda[i] = Delta_s * sup(abs(H_i))
state_at(lambda) -> FormationState(s, qf, actual_body_headings)
time_at(lambda) -> inverse_of_global_s_of_t(s)
```

直接构造旧`AnalyticSweepSegment`交给`FormationCollisionChecker.swept_check_analytic`，不使用旧样本chord API。全局时间律逆映射用单调二分、端点直接赋值，不伪造时间。位置/角误差对车体角点位移的包络上界至少包含 `eps_xy+(|r_i|+inflated_body_radius_i)*eps_psi+inflated_body_radius_i*eps_relative_heading`；作为额外checker裕量取全车最大值，原0.05 m不减少。

优化内距离场是引导，终审是真实三车OBB/SAT/保守覆盖。外接圆仅作安全充分条件快通，不作为唯一硬排斥。图引导只生成少数路线初值，不能把折线作为执行结果。

## 5. 阶段分解、门禁和量化标准

| 阶段/子任务顺序 | 交付能力 | 允许进入下一步的依据 |
|---|---|---|
| G0-A 基线/工作区核验 → G0-B 安全回归入口 | 保护清单、重定向测试输出、旧参考可重审 | G0 Codex ACCEPTED；不实施几何 |
| G1-A 公式/初态 → G1-B C2 Hermite/积分界 → G1-C 双移线/负例 | 数学模型及生产曲率组件 | 数学证据，不能声称自主规划 |
| G2-A 整段几何审核 → G2-B 自动初值/NLP → G2-C 冻结端点实验 | 起终点自主几何路径 | 独立GEOMETRY_FEASIBLE，无动态/地图认证 |
| G3-A 时间求值 → G3-B 动态界/选T → G3-C 序列化/篡改 | 任意时刻三车同步动态模型 | DYNAMIC_FEASIBLE，仍不可ROS发布 |
| G4-A A-only场景冻结 → G4-B 路线/碰撞 → G4-C 全流水线 | 地图上完整可执行且审核的计划 | 三层报告齐全且摘要一致 |
| G5-A 有限hook → G5-B bridge/播放 → G5-C真实ROS及旧回归 | RViz交互、同步播放和失败安全 | ROS证据加新旧回归，阶段完成后停止 |

子任务是阶段内顺序检查点，先小范围测试再继续；出现失败先修本子任务，不一次改全系统。整阶段通过后仍等待Codex和用户，不自动跨阶段。

统一量化合同：初态heading误差`<=1e-7 rad`；刚性误差`<=1e-6 m`；双移线端点误差`<=1e-6 m/rad`；独立曲率差分`<=1e-5 1/m`、航向差分`<=1e-6 rad`；自主终点`<=.005 m/.2°`；序列化/任意时刻状态一致`<=1e-7`相应单位，时间一致`<=1e-9 s`。C2公差见D05。全域限制须在明确误差包络后满足，不另加“允许超限百分比”。

独立微分采用高精度位置oracle并记录步长/误差放大；不得把默认1e-7m积分预算直接作二/三阶差分仍要求同一数值公差。G3独立速度/加速度/jerk差分建议门槛分别1e-5/1e-4/1e-2相应单位，详见G3任务书；这不替代解析全域动态审核，也不放宽roundtrip一致性。

相位边界采用左右极限，g可以跳变但保持有界；不把有限jerk跳变误写为jerk必须C0。精度变差/求解失败时不改旧测试预期、形状或限制求通过。

## 6. 具体端点、场景和回归策略

### 6.1 G1强制样例

双移线`K=.30,L=6`、镜像`K=-.30`，F起点(0,0,0)。第一次移线`(5.8222549849,1.1256889273,0)`，完整终点`(11.6445099698,0,0)`。F四符号`+,-,-,+`；前车不得要求同符号。`L=.5`前车峰值约4.34822应拒绝；侧车峰值约.428571仍正常，必须定位前车/eta原因。

### 6.2 G2六个非平凡可行端点

下表的存在性参考为`k(s)=alpha*(s/S)^2`、`eta=2*alpha*s/S²`、`psi=alpha*s³/(3*S²)`。它是C2且起始k/eta=0的二次曲率，可被8段Hermite精确表示；**只供离线测试确认存在解，不传给初值生成器**。正式输入仅表中端点和固定初态/配置，精确端点由独立quad生成，不以表格舍入作为唯一真值。

| S/alpha（仅存在性） | 输入goal x/y/psi，m/rad |
|---|---|
| 6 / +.1 | `(5.982887884033, .299200999308, .2)` |
| 6 / -.1 | `(5.982887884033, -.299200999308, -.2)` |
| 6 / +.2 | `(5.931919086525, .593631911497, .4)` |
| 6 / -.2 | `(5.931919086525, -.593631911497, -.4)` |
| 8 / +.3 | `(7.644636301724, 1.533083640872, .8)` |
| 8 / -.3 | `(7.644636301724, -1.533083640872, -.8)` |

另测(6,0,0)直线、整体平移旋转等价、精确零运动、非平凡显式STRAIGHT端点。STRAIGHT可用夹持CubicSpline结点`s=0..6`、`k=[0,.10,.20,0,-.20,-.10,0]`、两端eta=0独立生成参考端点并验证曲率；参考需在求解前冻结，不能根据求解结果倒换测试。

### 6.3 G4基准

使用G1完整双移线、上述C2平滑转弯参考及含局部避让的路线构造三类A-only场景及镜像。地图为真实占用栅格；扫掠参考验证通过后冻结其地图/端点，但正式规划器只能读取地图/端点，不得读取参考模型。具体宽度由独立参考的三车矩形包络和0.05 m裕量确定，不预先保证旧2.8 m/3.2 m恰好可行。详细构造、反例和命令见G4任务书。

### 6.4 保护旧测试输出

新工具`virtual_a_baseline.py`须提供`--output DIR --regression LEVEL`，使用subprocess逐条运行旧测试并注入：

```text
FR01_RESULTS_PATH
FR02_TIMING_RESULTS_PATH
FR02_TRAJECTORY_RESULTS_PATH
FR02_QUALITY_RESULTS_PATH
FR02_ARTIFACT_DIR
FR03_RESULTS_PATH
FR03_ARTIFACT_DIR
FR04_RESULTS_PATH
FR04_ARTIFACT_DIR
```

全部指向本次新run目录的独立子目录。原测试有其他输出时，先读源码识别，不能盲跑全discover重写旧证据。重定向后再做before/after历史哈希核验。旧测试日志和非零退出码全部保留；未执行、环境缺失、权限拒绝一律NOT_RUN/BLOCKED，不算PASS。

分层：G0只旧参考审核和依赖检查；G1新数学测试+旧geometry/Hermite轻回归；G2新几何/NLP测试；G3新动态/序列化+旧时间关键回归；G4新地图集成；G5完整相关FR01–FR05回归、FR06至少一条原参考及一次冻结窄道自主搜索、原单车受影响测试及真实ROS。不要求每阶段重跑六次FR06搜索。

## 7. G5有限接入接口

现有节点并非通用后端。保留`UniformPlannerNode(plan=plan_uniform)`的旧注入签名/默认行为，增设可选`backend_hooks=None`，只有选择新backend时启用。建议在新`ros_adapter.py`提供有限hook对象：

```python
VirtualABackendHooks.request_from_snapshot(snapshot, grid, settings, initial_context)
VirtualABackendHooks.solve(request) -> VirtualAResult
VirtualABackendHooks.publication_fields(snapshot, result, stamp, dt) -> dict
VirtualABackendHooks.verify_envelope_fields(envelope_fields) -> VerifiedPlayback
VirtualABackendHooks.result_summary(result) -> dict
```

旧路径可保留原代码分支，不强制重构成通用插件或复制节点。节点状态机、map变更拒绝、generation过期防护、busy和发布后回执继续复用；在原始调用点分派请求构造、成功信封/采样、结果摘要以及版本常量。

播放模块新增`verify_playback_plan(...)`分派器：旧backend仍调用原`verify_uniform_playback_plan`；新backend调用`verify_virtual_a_playback_plan`。新模型保持独立字段`virtual_a_plan`或明确的typed模型槽，不把新对象塞入旧`uniform_plan`。`frame_at`新分支从`state_at(t)`生成旧`PlaybackFrame/PlaybackVehicleState`，Marker和命名空间共用。

`FormationPlaybackPlan.msg`字段/MD5不变，新model JSON包含request/hash/limits/map/audit绑定。原`certification`字段只是标签；节点发布前完整审核且收端重新检查版本、模型摘要、样本和几何/动态合同，不依赖字符串宣称通过。默认launch仍`uniform_v02`，新后端单独显式选择。

## 8. 阶段交付与Codex独立验收

所有运行证据写唯一`evaluation/virtual_formation_a/Gx/<run-id>/`：

```text
COMMANDS.md               实际cwd、解释器、环境、命令、退出码
environment.json          依赖/硬件/ROS环境及NOT_RUN原因
source_manifest.json       HEAD、dirty diff摘要、源码/测试/配置哈希
requests/ models/ audits/  本阶段实际产生的数据，未产生的不造文件
results.json              每条用例及负例的期望、实际和误差
logs/                     stdout/stderr及求解迭代失败
CLAUDE_REPORT.md           修改摘要、公式核对、结果与阻塞，不能自封ACCEPTED
```

每阶段新增工具必须默认拒绝覆盖现有run目录，不自动更新时间戳或源证据。Codex另写`CODEX_ACCEPTANCE.json/md`，字段含`stage/status/reviewed_source_sha/evidence_manifest/independent_commands/findings`。基线后有未提交开发时绑定文件哈希+diff，不仅绑定HEAD。

Codex检查：允许文件差异、独立数学/差分/区间界、原请求而非被审模型自报端点、负例、证据哈希、序列化和随机非样本时刻、历史保护、未执行项；按各任务书详细清单验收。任一必要项缺失为REWORK/BLOCKED，不因Claude称“差不多通过”进入下一阶段。

## 9. 文档自审及P1落实追踪

| 原问题 | 本方案落实位置 | 验证阶段 |
|---|---|---|
| P1-1 初态不兼容/无轮系字段 | 决策D03；本方案3/4.1/7；ROS保留终态轮系语义 | G1拒绝负例，G5普通续接与显式reset |
| P1-2 C1不足 | D05；4.2/4.3，显式C2等式和g侧极限 | G1生产profile，G2硬约束，G3节点动态 |
| P1-3 完整动态 | D06；4.3/4.6含二维jerk、角与轮系 | G3独立全程界及负例 |
| P1-4 旧类型/版本冲突 | D08/D09；2.2/3/7 | G3新序列化，G5分派与旧回归 |
| P1-5 整段/积分/扫掠 | 4.5/4.7可实现余项与归一化上界 | G1/G2边界，G3动态，G4扫掠 |
| P1-6 真正自主规划 | 4.4/6.2，独立参考隔离 | G2读取禁止/初值来源测试 |
| P1-7 新模型已知可行地图 | 6.3及G4-A先冻结后求解 | G4参考审查与seed隔离 |

依赖自审：G1不依赖NLP/时间/地图，G2不依赖G3，G3不需要ROS，G4依赖G3的逆时间及完整模型，G5才改共享接入；零运动、自由目标和初始轮系在各层使用同一合同。G1解析profile为测试诊断，生产序列化只支持已验收的C2 Hermite或显式NO_MOTION hold（profile=None），不能混装版本或用重复站点冒充零长Hermite。

风险仍在：有限初值局部NLP可能失败、保守区间可能过松、全局五次T可能偏大、A-only窄道可能无解、数值库和ROS环境需实际核实。这些通过失败记录和阶段停止处理；没有写成已实现的性能优势。当前未发现需要重新选择整体研究路线的阻塞决策。

本目录八份文档为统一交付；逐阶段的完整可执行说明见对应任务书。本轮到此只形成文档，等待用户确认及后续工作区准备授权。

### 9.1 本次文档自审记录

| 自审项 | 结果及处理 |
|---|---|
| 七项P1是否全部落实 | 已逐条映射到上表，并在阶段任务中有具体正反例/失败条件；不是只在摘要承诺。 |
| 阶段依赖是否冲突 | 已检查；补齐显式零运动hold、G4在动态审核前冻结上下文、G5才接ROS的边界。 |
| 数学/接口/公差是否一致 | 已检查；统一profile字段名、初态source/context、C2等式、两个摘要及单侧导数；差分门槛与位置积分误差预算分开。 |
| 是否仍有无法直接执行的笼统任务 | 各阶段都有新文件/白名单、签名、具体输入、命令和交付。未来run-id/ACCEPTED证据路径须由执行时读取，不能在当前虚构。数值界/求解遇阻按明确失败路径上报，不自行降级安全语义。 |
| 是否可能破坏FR06 | G1–G4隔离新包，G5仅有限共享hook且强制旧回归；旧测试输出重定向，原历史目录哈希保护。风险仍需实际差异验收，文档不冒称已证明未来不会破坏。 |
| 人工基准是否误写为自主结果 | G1sin³、G2存在性参考、G4场景参考均与正式输入/生产导入隔离，设patch与无reference目录验证。 |
| 是否承诺未验证性能 | 无性能提升/成功率/创新已成立的表述。30s/60s/600s分别为软求解/工程时长预算，固定集合全部通过是待实现验收目标。 |

本轮已做Markdown文件集合、代码围栏、数学分隔符、内部链接和Git差异检查；八个目标文件齐全。既有FR06验收前哈希快照中146个源码/测试文件重算均匹配，tracked/staged diff为空；只新增本方案Markdown，原审核报告保持不动。未运行G0–G5工具、算法测试、ROS或正式实验，也未安装依赖、创建分支/worktree或提交Git。
