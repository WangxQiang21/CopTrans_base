# Claude G3：全路径公共时间、三车完整动态、权威模型和独立审核

## 0. 上下文与门禁

只执行本阶段，须G2有Codex ACCEPTED且用户明确下达G3。工作分支`feature/virtual-rigid-formation-a`，祖先`567bdecba9676313da3bc9a97b82f57b5b569a2b`，仅隔离worktree。不得改旧FR06、提交Git或进入G4。

固定offset为ID1:(1,0)、2:(0,1)、3:(0,-1)m；统一A正向、F沿psi。静止初态三车车头与psi平行、理想A直行，k0=eta0=0。默认FREE不强制终端回直，STRAIGHT才kT=etaT=0。生产曲率为已验收C2 Hermite，g分段有界。

G2只证明几何；本阶段证明公共时间和动态，不涉及地图安全或ROS播放。当前没有连续A时间最优任务。

## 1. 目标和文件白名单

新增：

- `P/formation_continuous_a/trajectory.py`：VirtualATimedPlan、TimeState/VehicleTimeState、任意时刻求值、规范序列化/摘要。
- `P/formation_continuous_a/timing.py`：全路径五次、总时长选择、单调逆映射与工程预算。
- `tests/test_virtual_a_timing.py`、`tests/test_virtual_a_trajectory.py`。

可改：新`contracts.py`的时间/报告类型，`audit.py`增加独立动态检查，`bounds.py`增加当前表达式需要的区间操作，`tools/virtual_a_benchmark.py`增加G3，`tests/test_virtual_a_audit.py`增加动态负例；新证据目录。

`P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。允许因真实接口缺陷修G1/G2新源码，但要单列、重跑其旧新测试，不改旧测试预期/限值。

保护全部旧源码、ROS、YAML/地图和`evaluation/formation_refactor_v02/**`。

直接复用`formation_retiming.FormationTimingLimits:119`、`retiming.MotionLimits:17/wheel_configuration:101/quintic:127`；可参考`continuous_retiming._quintic_state:189`。不得调用旧`uniform_timing.schedule_uniform_edges:675`把曲率结点变成停车边，也不能用旧固定pivot的导数审核新几何。

## 2. 本阶段接口和数据语义

```python
TimingConfig(max_total_duration_s=600.0, safety_factor=1.01,
             max_rescalings=8, ...)
def retime_path(request: VirtualARequest, path: VirtualAPath,
                config: TimingConfig, *, planning_context=None) -> TimingResult: ...

class VirtualATimedPlan:
    @property
    def duration_s(self) -> float: ...
    def state_at(self, t: float, side: str='auto') -> VirtualATimeState: ...
    def time_at_s(self, s: float) -> float: ...
    def sample(self, times: tuple[float,...]) -> VirtualAProjection: ...
    def to_model_dict(self) -> dict: ...
    def to_model_json(self) -> str: ...
    def model_sha256(self) -> str: ...
    @classmethod
    def from_model_json(cls, payload: str, *, expected_request=None): ...

def audit_dynamics(request: VirtualARequest, plan: VirtualATimedPlan,
                   config: DynamicAuditConfig, *, expected_planning_context=None) -> AuditReport: ...
```

TimeState含`t,s,w,a,j,qf,mode='A'`以及按ID三车`x,y,theta,C_i,v_i,omega_i,velocity,acceleration,jerk,yaw_accel,yaw_jerk`；理想轮角、轮速、设置速率/加速度仅诊断字段，不是控制指令。

Projection含同一组times、对应完整states和model摘要；不能作为权威模型。state_at在[0,T]连续求值，不对三车样本各自线性插值。t<0保持初态、t>T保持终态，位置/车头/终端轮系不改，速度/角速度为0；时域外hold和边界单侧导数明确区分。

内部节点auto右、终点auto左，left/right都可独立审核。g不连续处允许有界jerk/设置加速度跳变，但加速度和设置速率应连续。起终速度和加速度为零，不额外要求运动单侧jerk也为零。

精确NO_MOTION采用显式hold模型，S=T=0、只有一个t=0样本，不走除T/反求s的运动公式；车头/轮系A直行，身份/摘要/目标照常审核。

## 3. 公共时间律和完整动态公式

### 3.1 时间律

```text
tau=t/T
h=10*tau³-15*tau⁴+6*tau⁵
h1=30*tau²*(1-tau)²
h2=60*tau-180*tau²+120*tau³
h3=60-360*tau+360*tau²
s=S*h; w=S*h1/T; a=S*h2/T²; j=S*h3/T³
```

h1在0<tau<1严格为正。每个空间控制结点只对应一个普通运动时刻，不加零速/驻留/setup。初态已明确A直行，首版额外启动驻留固定0；不开发模式重配置或补S。FREE终端轮系可以非零，不能为了好看在hold时瞬间设为直行。

`time_at_s`用h单调性二分，端点直接0/T；误差<=1e-9s，非有限/域外输入拒绝；零运动只接受s=0。后续G4扫掠需用这个真实映射。

### 3.2 从G1空间jets恢复动态

令Q=sqrt(D)，H=theta_s，C=H/Q，空间导数由已验收G1提供：

```text
v=Q*w; omega=H*w
velocity=p_s*w
acceleration=p_ss*w²+p_s*a
jerk=p_sss*w³+3*p_ss*w*a+p_s*j
yaw_accel=H*a+H_s*w²
yaw_jerk=H*j+3*H_s*w*a+H_ss*w³
lateral_accel=abs(C)*v²
```

每车`omega=C*v`。`a_max`和`jerk_max`限制二维向量模长，不是只限制纵向`dot(v)`。默认逐车限制：v .8、二维a .6、二维jerk2、yaw .8/.8/3、横向a .6、轮速1、轮角pi/2、设置速率.8、设置加速度1.5（对应m/s阶次、rad/s阶次），实际值用原`FormationTimingLimits`并写摘要，禁止放宽。

独立审核须从原始profile重新计算这些量，不直接信plan.state_at内的派生字段；通用区间算子可复用，导数表达和原请求绑定独立重算，再用差分核对生产求值器。

### 3.3 理想轮系限制

轮坐标`xw=±wheelbase/2,yw=±track/2`，对每车实际曲率C而非F曲率：

```text
m=1-yw*C; n=xw*C; Ew=m²+n²
delta=atan2(n,m); factor=sqrt(Ew); wheel_speed=factor*v
Ew_C=-2*yw+2*(xw²+yw²)*C
delta_s=xw/Ew*C_s
delta_ss=xw/Ew*C_ss-xw*Ew_C/Ew²*C_s²
delta_dot=delta_s*w
delta_ddot=delta_ss*w²+delta_s*a
```

从G1原始profile独立恢复C_s/C_ss：

```text
D_s=2*((aoff²+boff²)*k-boff)*eta
D_ss=2*((aoff²+boff²)*eta²+((aoff²+boff²)*k-boff)*u)
Q_s=D_s/(2*Q); Q_ss=D_ss/(2*Q)-D_s²/(4*Q³)
H_s=eta+aoff*(u/D-eta*D_s/D²)
H_ss=u+aoff*(g/D-2*u*D_s/D²-eta*D_ss/D²+2*eta*D_s²/D³)
C_s=H_s/Q-H*Q_s/Q²
C_ss=H_ss/Q-2*H_s*Q_s/Q²-H*Q_ss/Q²+2*H*Q_s²/Q³
```

其中aoff/boff为车辆偏移，避免与时间加速度a混淆。轮速/轮角与旧`wheel_configuration(Action(0,0,C),motion_limits)`交叉核对。纯几何角超限不能靠增加T修复；不要开发CAN/轮角命令输出。

## 4. T选择和全程独立审核

### 4.1 时长选择

先通过几何审核。T0=1s时计算三车、各轮各项动态的保守全域上界U；使用同一s路径和tau区间，动态按T阶数缩放：

- v/omega/轮速/设置速率为T^-1；
- 二维a/yaw_accel/横向a/设置加速度为T^-2；
- 二维jerk/yaw_jerk为T^-3。

取所有`(U/limit)^(1/order)`最大值作为T，再乘1.01。独立audit若因界过松未确认，先细分相关区间；如需再放大T，最多8次。T>600s返回`TIMING_BUDGET_EXCEEDED`；600s是新工程预算，不复用旧max_block_duration_s=120来限制整路径，不代表物理无解。

### 4.2 连续域检查

按Hermite段和对应时间节点切分，用G1外扩区间界k/eta/u/g、D/Q/H及轮系量，五次时间多项式用区间界w/a/j。组合得到各向量模长和角/轮系上界，不能以采样峰值冒充全域峰值。

独立audit从T和原始knots重算，遍历节点左右，差分plan.state_at的x/y/theta；不直接接受retimer提供的peaks。默认最大深度20/叶数200000，无法确认给`BOUND_UNCONFIRMED`或预算失败，不输出DYNAMIC_FEASIBLE。不能关闭审核、删难指标、放宽限值或只检查“最慢一车”。

## 5. 新权威模型与版本

- backend=`virtual_rigid_a`，timed model=`virtual_rigid_a.timed.v1`，path=`virtual_rigid_a.path.v1`。
- JSON核心字段：版本、plan/formation ID、原request_inputs及request_sha、实际model/形状/limits、goal_policy/initial声明、原始profile结点、numerics_version/积分容差和误差假设、统一T，G4加入地图摘要。
- request_sha仅绑定物理请求；planning_context另绑定request_sha、地图及planner/timing/collision设置。G3可为空，G4必须在retime_path构建模型之前传入冻结上下文，并在audit_dynamics传expected上下文核对，不能审核后再改变模型摘要。
- 不序列化三车独立路径作为权威，不把位置缓存/绘图样本当模型。运动只支持生产C2 Hermite，NO_MOTION用profile=None的显式hold kind；G1 sin³保留数学工具用途，不新增第二个正式运动profile类型。
- 模型摘要为有限canonical JSON核心SHA-256。模型不内嵌自指的`model_sha256`或绑定自身摘要的完整audit报告，避免循环；审核报告在结果/证据中单独绑定最终模型摘要。
- from_model_json默认只恢复并验证结构/数学，不因JSON自报passed就“已认证”。传expected_request时逐字段比原请求；动态审核仍单独执行。拒绝未知版本、额外不支持字段、NaN、重复ID、错误/非单调域、非C2、修改初态/限值/目标。
- FREE在终点hold保留路径终端实际C/车头/理想轮角；不能v=0时用omega/v算C或把C清零。

## 6. 测试清单与期望

1. G2实际成功的六个端点路径、(6,0,0)直线、STRAIGHT路径，逐车时间化且独立审核。
2. 至少100个确定性非均匀tau，以及所有结点对应t和左右极限；数值比对速度/角速度/二维加速度/jerk，差分步长与误差表记录。
3. 解析h1证明内部>0，数值确认所有内部profile节点w>0；近端点浮点极小速度不当成额外停车。
4. 直线对照：Q=1,H=0，各车v相同、omega=0、中心向量动态相同，轮速=v；两端v/omega/a为0。
5. 人为将合法T缩小到.1倍，必须有至少一项实际动态超限，独立audit定位量/车/时间；不是只检查retimer状态。
6. C1但非C2的真实profile在进入时间化前拒绝；g跳变但C2合法的profile不能因有限jerk左右不同而误拒绝。
7. 构造前车角动态最不利、侧车速度最不利、理论轮速/设置速率或加速度最不利的不同负例，不能只覆盖center v。
8. 模型JSON roundtrip：未采样t的全部状态差<=1e-7相应单位，时间逆映射<=1e-9s；零运动roundtrip无除零。
9. 篡改T、一个profile值、一个车头/速度、实际limits、request/goal_policy/hash/版本，必须被对应审核或绑定检查拒绝。样本篡改不改变权威模型，也不能被接受。
10. FREE路径终端非零曲率时hold不回直；续接合法性标志准确交给G5，不能宣称任意FREE终点都可当新首版初态。

差分口径：对同一原始profile用独立更严格积分生成位置oracle，选`dt=.005/.01s`且不跨g跳变或时域hold边界的stencil，分别比生产解析velocity/omega、accel/yaw_accel、jerk/yaw_jerk，建议误差门槛依次`1e-5/1e-4/1e-2`相应单位；必须同时记录截断和位置误差放大诊断。默认1e-7m生产积分误差不能无条件经三阶差分仍要求1e-7一致。序列化/同一t状态roundtrip的1e-7标准不因此放宽。边界用解析左右极限，不用跨不光滑处stencil制造假超限。

## 7. 子任务和命令

G3-A先实现五次、逆映射、点态state_at及直线测试；G3-B再实现全程界、T选择和动态反例；G3-C完成序列化和身份/样本篡改。每个子任务通过再继续，暂不加地图/ROS。

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G3/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$VA_RUN/logs"
set -o pipefail
python3 -B -m unittest discover -s tests -p 'test_virtual_a_timing.py' -v 2>&1 | tee "$VA_RUN/logs/timing.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_trajectory.py' -v 2>&1 | tee "$VA_RUN/logs/trajectory.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v 2>&1 | tee "$VA_RUN/logs/audit.log"
python3 -B tools/virtual_a_benchmark.py --stage G3 --output "$VA_RUN/benchmark"
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN/legacy" --regression timing
```

保留G1/G2受影响回归。旧时间测试必须经G0runner设置FR02_*输出重定向，不能默认写旧验收目录。环境/权限未运行项如实标注。

## 8. 验收、证据和Codex检查

量化要求：全部固定正例动态审核通过，原限值一项不漏；刚性<=1e-6m；roundtrip状态<=1e-7、时间<=1e-9s；无内部强制停车；起终速度/角速度/加速度零；必要反例全部正确拒绝。全域上界通过而非仅峰值采样。位置积分和C2精度继续遵守G1。

交付完整模型JSON、规范摘要、原请求、T选择/最不利车辆/约束/缩放阶数、节点左右动态、区间界及误差假设、独立差分、100个非均匀时间对照、所有篡改/超限记录、保护哈希、实际命令/退出码和`CLAUDE_REPORT.md`。标`DYNAMIC_FEASIBLE`，`collision_checked=false`，`executable=false`，不能造地图/ROS证据。

Codex独立检查：

- 手推T缩放及二维加速度/jerk、角动态、轮系导数，核查不是沿用旧常控制公式。
- 独立从原profile/T数值求导，定位节点左/右；检查端点jerk语义没有不必要放宽/加强。
- 重建model JSON后选全新非样本t，核对所有车；检查request/limits绑定与无循环摘要。
- 缩短T、改limits/策略/样本并重审，确认审核器独立于retimer状态。
- 确认默认FREE终点保留轮系，内部不停，无隐式setup/S，旧代码和历史数据不变。

**停止：G3提交报告后等待Codex ACCEPTED与用户下达G4；不做地图、ROS或时间最优，不提交Git。**
