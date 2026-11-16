# Claude G2：端点驱动的自主连续 A 几何规划与独立审核

## 0. 上下文、前置和停止边界

本任务仅G2；须用户明确下达，G0/G1有Codex ACCEPTED并与实际源码/证据匹配。正式祖先 `567bdecba9676313da3bc9a97b82f57b5b569a2b`，仅在Codex准备的 `feature/virtual-rigid-formation-a` worktree实施。不得创建分支、操作旧worktree、提交Git或自动做G3。

固定三车：ID 1:(1,0)、2:(0,1)、3:(0,-1)m，车体.738×.500m；统一A、正向、F速度沿psi；初始全车静止、theta_i=psi、理想A直行轮系，k0=eta0=0。不支持初态不能靠S/覆盖车头修复。默认终点车辆车头自由，STRAIGHT才额外kT=etaT=0。

目的是真正输入端点自动求解。G1人工sin³路径只证明数学，不得直接返回或注入G2；本阶段输出仅`GEOMETRY_FEASIBLE`，没有时间/动态/地图/ROS认证。

## 1. 范围与文件白名单

`P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

新增：

- `P/formation_continuous_a/optimizer.py`：自动初值、端点NLP与候选选择。
- `P/formation_continuous_a/audit.py`：独立几何审核；G3/G4再扩展，不预建空实现。
- `tests/test_virtual_a_optimizer.py`、`tests/test_virtual_a_audit.py`。
- `tests/fixtures/virtual_a/open_requests.json`：冻结端点；独立存在性参考另存fixture或证据，不在生产代码读取。

可改：新包`contracts.py`增加本阶段request/config/report，`geometry.py/bounds.py`仅必要的参数/整段接口；`tools/virtual_a_benchmark.py`增加G2；新G2证据目录。

不得修改已验收G1测试预期、初态/C2/车辆限制；若发现真实G1缺陷，单列返工差异、回归全部G1并让Codex确认，不混作“提高成功率”。旧FR06、单车、ROS、配置、碰撞代码和历史证据全保护。

复用真实源码：旧`ControlKnot/MotionBlock.control/control_integral`；`continuous_optimizer.py:347`的CasADi组织和`continuous_geometry.py:436 fit_continuous_endpoint`的终点拟合方法仅供借鉴，它们没有三车约束。复用G1实际交付`C2CurvatureProfile/VirtualAPath/map_spatial_state`，不要假设其它接口已经存在。

## 2. 本阶段接口与返回合同

```python
VirtualARequest(plan_id, formation_id, model, vehicle_shapes,
                start_qf, start_headings, goal_qf,
                initial: InitialAssumption, goal_policy: GoalPolicy,
                limits: FormationTimingLimits)
PlannerConfig(intervals=8, max_initials=6, max_seconds=30.0,
              max_iterations=120, solver='ipopt', ...)

def make_initial_guesses(request, config) -> tuple[InitialGuess,...]: ...
def solve_open_path(request: VirtualARequest,
                    config: PlannerConfig) -> GeometryResult: ...
def audit_geometry(request: VirtualARequest, path: VirtualAPath,
                   config: GeometryAuditConfig, *, planning_context=None) -> AuditReport: ...
def request_fingerprint(request) -> str: ...
```

`GeometryResult`包含status、path或None、原请求摘要、audit、各次attempt和wall_time_s。status仅`GEOMETRY_FEASIBLE/NO_MOTION/INVALID_REQUEST/NO_ACCEPTED_CANDIDATE/BUDGET_EXHAUSTED/NUMERICAL_FAILURE`；没有executable=True。每个attempt保存初值来源、solver、状态、迭代、目标/约束残差、耗时、候选审核及拒绝原因。

AuditReport记录scope=`GEOMETRY`、通过/原因、首个failure位置(s/interval/vehicle/constraint)、原请求/路径/限制摘要、全段界、积分/C2误差及检查数。审核从原请求验证起终点，不能只比较path自己携带的goal。

request_sha256只表示物理请求，G4另用planning_context_sha256绑定地图/规划设置，不能到G4重新改变request_sha256含义。G2的planning_context可为空；预留该keyword用于后续同一几何在明确地图上下文下审核，不在G2实现地图。

拒绝未知GoalPolicy、非有限值、错ID/构型、非静止/非A/非直行声明和不兼容真实车头。精确相同start/goal允许显式NO_MOTION几何hold；不能用位置接近但方向不同的请求走零运动快捷分支。

## 3. 具体NLP规范

### 3.1 参数化、约束和单位

默认N=8等长区间，变量`S,k_0..k_N,eta_0..eta_N`，站点s_j=j*S/N。h=S/N，k单位1/m，eta单位1/m²。固定初态k0=eta0=0。

每段复用G1Hermite：

```text
c0=k_j; c1=h*eta_j
c2=3*(k_next-k_j)-h*(2*eta_j+eta_next)
c3=2*(k_j-k_next)+h*(eta_j+eta_next)
u_left_end=(2*c2+6*c3)/h²
u_right_start=2*c2_next/h²
C2_j: u_left_end-u_right_start=0
```

终点积分`x'=cos(psi),y'=sin(psi),psi'=k`满足原goal三项；角度按目标相对于start的连续分支处理，首版使用最短wrap分支并记录，不宣称覆盖所有绕圈解。FREE不约束k_N/eta_N；STRAIGHT明确加两式。

每辆车、每个配点及最终整段：

```text
A_i=1-b_i*k > 0; D_i=A_i²+(a_i*k)² > 0
C_i=(k*D_i+a_i*eta)/D_i**1.5
abs(C_i)<=1/r_min_ackermann
```

可用侧车界`abs(k)<=1/(1+.810)`先限制，但不代替前车eta硬约束。初始真实车头误差<=1e-7rad。刚性直接由FormationModel映射，无需在NLP中再优化三套x/y后加距离罚项。

### 3.2 搜索域、目标和积分

端点距离d，`S_lower=max(N*.05,d)`，`S_upper=min(60,max(12,3*d+10))`；零位移非零转向允许S>0。eta结点范围±2 1/m²；S和eta搜索域是数值工程限制，不是物理无解条件。下限>=上限时返回明确搜索域原因。

`l0=max(1,d)`，默认无量纲目标：

`J=S/l0 + .02*l0³*∫eta² ds + .002*l0⁵*∫u² ds`。

积分项可用多项式精确积/Gauss，NLP终点用可微固定Gauss；最终权威路径由G1有界积分恢复，不能将优化配点积分终点冒充最终误差。每段至少9个配点，候选最终独立整段审核；发现段内问题可增加本候选约束重解，但不能只加采样就声称证明。

### 3.3 自动初值和有限求解

最多6个、固定顺序，由端点自动生成：长度倍率(1.2,1.8,2.6)乘`max(d,1)`，各配左右曲率形状；利用目标方向差给净曲率积分偏置，利用横向误差给左右形状；通过夹持三次样条/线性C2投影使初值先满足C2和k0/eta0。每个实际初值须可重建并写日志。

不得按测试ID/特定goal查表选择参考曲线，不从fixtures/evaluation或G1构造器导入种子。可参考现有优化技术，但不直接把旧局部形变器当自主求解入口。

优先CasADi/IPOPT；G0确认依赖。如果使用显式SciPy SLSQP后备，标记solver和原因，不吞任意数值错误。总软预算30s，单初值最多120迭代，deadline在迭代/初值边界检查；库调用超时如实记录，非硬实时。找到独立审核通过的最好候选再返回；solver success本身不够。

## 4. 独立几何审核具体内容

不调用优化器的残差函数作为唯一验证；从原始结点独立重建c0..c3，重新检查：

1. request身份、初态、目标策略、模型/形状/限值未被改变。
2. 正h、结点域、C0/C1/C2左右极限，g分段有界。
3. 更严格位置积分及其余项预算；与原请求的终点差。
4. 全区间A/D正则、实际C_i和理论静态轮角限制；不是只有结点。
5. 刚性、切向/实际车头及`theta_s=C_i*Q_i`。

使用G1通用外扩算子，但独立生成系数和表达式。可检查`(k*D_i+a_i*eta)^2-kmax²*D_i³<=0`并保持D>0，或直接细分C_i区间。默认最大深度20/总叶数200000，无法确认则`BOUND_UNCONFIRMED`/`AUDIT_BUDGET_EXHAUSTED`，不可GEOMETRY_FEASIBLE。数学实数余项与浮点保护假设要保留，不将高密度网格作sup。

## 5. 测试输入和预期

### 六个非平凡端点：默认FREE

全部start=(0,0,0)、真实车头0、初始A静止直行，默认车辆参数。输入：

| ID | goal(x,y,psi)，m/rad |
|---|---|
| E1 | `(5.982887884033, .299200999308, .2)` |
| E2 | `(5.982887884033, -.299200999308, -.2)` |
| E3 | `(5.931919086525, .593631911497, .4)` |
| E4 | `(5.931919086525, -.593631911497, -.4)` |
| E5 | `(7.644636301724, 1.533083640872, .8)` |
| E6 | `(7.644636301724, -1.533083640872, -.8)` |

存在性参考仅在测试/证据中：`k(s)=alpha*(s/S)²`，(S,alpha)依次(6,.1),(6,-.1),(6,.2),(6,-.2),(8,.3),(8,-.3)。该二次profile属于生产C2空间；先用独立积分/曲率界确认并冻结精确端点，正式solve调用只传request，不传S/alpha/profile。采用patch参考构造器一旦被调用就抛错，以及检查生产import/文件读操作，证明隔离；不能把patch目标没被使用当完整证明，Codex还读数据流。

其它必测：

- (6,0,0)直线、精确零运动、任一请求整体旋转.7rad并平移(2,-3)后的等价性。
- STRAIGHT：独立夹持CubicSpline站点0..6、k=[0,.10,.20,0,-.20,-.10,0]、端点eta=0，先审核可行并冻结积分终点。优化只收目标和STRAIGHT策略。
- 构造终端k或eta非零但合法的路径：FREE审核通过，STRAIGHT拒绝；不要求自主最优结果一定非零，只检语义没有隐含回直。
- 段内超限但结点合格的Hermite反例必须被独立界拒绝；由固定fixture给出系数和超限位置，不只mock审核返回false。
- 非C2、错误初态car1=.1rad、伪造goal/request摘要、错shape/limit、NaN、solver失败、极小总预算。
- 假solver success但终点错误/曲率超限：必须无GEOMETRY_FEASIBLE路径。
- G1完整双移线终点在无障碍下允许直线，不写成“必须自主双移线”的错误测试。

## 6. 子任务、命令和定量标准

G2-A先完成audit_geometry及其真实正反例；G2-B再做初值+NLP，先直线和E1/E2；G2-C运行全部冻结端点和隔离/失败测试。单个任务失败先定位，不同时改时间/地图。

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G2/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$VA_RUN/logs"
set -o pipefail
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v 2>&1 | tee "$VA_RUN/logs/audit.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v 2>&1 | tee "$VA_RUN/logs/optimizer.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v 2>&1 | tee "$VA_RUN/logs/g1_geometry.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v 2>&1 | tee "$VA_RUN/logs/g1_profile.log"
python3 -B tools/virtual_a_benchmark.py --stage G2 --output "$VA_RUN/benchmark"
```

验收要求：

- 六个非平凡端点及直线、冻结STRAIGHT目标均有实际求解且独立几何审核通过，使用同一冻结默认配置/预算。达不到则G2返工，不把局部失败改写为物理无解或删用例。
- 最终位置误差<=.005m、方向<=.2°，刚性<=1e-6m；C2与积分预算符合G1合同。
- 所有曲率/理论静态轮系限制在全段包络后满足；必要负例全部拒绝且原因正确。
- 30s为总软预算记录实际wall及超时点，不承诺每次严格30秒返回。无法在此预算达到最小集合须报告，不自动增大预算或加段数。
- 不对运行时间改善、全局收敛和完整通行能力作未经验证的结论。

## 7. 交付与Codex独立验收

交付源码/测试、冻结requests和独立存在性资料、每次初值/solver日志、全部attempt、原请求/配置/hash、实际profile/端点/C2/全段界、每条审核、失败原因、保护哈希和`CLAUDE_REPORT.md`。本阶段不造timed model或ROS证据。

Codex独立：

- 从原请求重建终点和真实初态检查，不信path自报goal。
- 阅读生产数据流，确认只有端点/设置入solve，参考曲线没有以warm start/fixture/cache形式泄漏。
- 自己选一对镜像请求和变换请求重跑；用不同积分/数值差分复核结果及段内极值。
- 检查FREE未强制kT/etaT=0，STRAIGHT约束明确入request摘要。
- 用真实超限/非C2/假success篡改检查拒绝，检查不可确认状态不被发布为合格。
- 确认原FR06、旧测试预期、G1限制和历史证据未改；差异只在白名单。

**停止：交付G2后等待Codex ACCEPTED和用户下达G3。不得以演示名义加时间、地图或ROS，不提交Git。**
