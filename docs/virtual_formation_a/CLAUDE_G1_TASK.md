# Claude G1：解析三车运动学、C2曲率组件和双移线数学基准

## 0. 上下文和执行门禁

你负责实现，Codex独立验收。本任务仅G1；须已有G0与真实源码哈希绑定的Codex ACCEPTED，并收到用户明确执行G1的指令。只在Codex准备的 `feature/virtual-rigid-formation-a` 隔离worktree工作，正式祖先 `567bdecba9676313da3bc9a97b82f57b5b569a2b`。不操作原两个worktree，不提交/推送，不自动进入G2。

本研究固定三车ID/offset `1:(1,0),2:(0,1),3:(0,-1) m`，车体`.738×.500 m`，统一A、正向、F沿集群前向。静止初态三车车头平行psi且理想A直行轮系，所以kappa(0)=eta(0)=0；输入不兼容必须拒绝。航向由偏移曲线恢复，不增加三个规划器。默认终点自由，回直是显式选项。

本目录决策和主方案为共同合同；本任务包含G1全部关键公式。若前阶段没有实际API，不可编造它已存在，应先报告。

## 1. 目标和非目标

交付三车解析几何/空间导数、可复用的C2三次Hermite wrapper、可核查的位置积分误差和完整双移线/镜像/超限负例。证明数学模型正确，不证明自主规划、动态限速或碰撞安全。

不实现NLP、地图、正式s(t)、ROS、模式切换、控制命令，不为本阶段建立通用机器人框架。

## 2. 文件清单和已有复用

`P=catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

允许新增：

- `P/formation_continuous_a/__init__.py`：仅导出已实现对象。
- `contracts.py`：`InitialAssumption`、`GoalPolicy`、CurvatureJet、空间state/jet、结构化失败；复用旧Formation类型。
- `geometry.py`：C2CurvatureProfile、VirtualAPath、map_spatial_state、validate_initial_state。
- `bounds.py`：局部多项式区间包络、生产积分余项预算；不做未使用的通用符号系统。
- `tests/test_virtual_a_geometry.py`、`tests/test_virtual_a_profile.py`。
- `tools/virtual_a_benchmark.py`：本阶段只实现 `--stage G1 --output DIR`，诊断曲线构造器仅在工具/测试中，不能进入未来自主初值入口。
- 新 `evaluation/virtual_formation_a/G1/<run-id>/**`。

唯一允许修改旧文件：`catkin_ws/src/fwis_v3_optimizer/setup.py`，仅将`fwis_v3_optimizer.formation_continuous_a`加入packages登记，保留根包。

直接复用：`formation.py:240 FormationModel.centers`、`vehicle_poses:255`、`rigidity_residual:290`；`transport_contract.contract_from_mapping`及权威YAML；`continuous_geometry.ControlKnot:19`、`MotionBlock.control:101/control_integral:90`；`retiming.wheel_configuration:101`作为理想轮系对照；旧`uniform_motion.a_arc_edge:1586`仅用于常曲率特例交叉对照。

严禁改旧`uniform_*`、旧formation核心、`continuous_geometry.py`、YAML、旧测试/地图/验收证据。不要复制FormationModel或重新硬编码一份运行配置。

## 3. 需要交付的具体接口

以下均为本任务新增建议签名，不是基线已有API：

```python
@dataclass(frozen=True)
class CurvatureJet: kappa: float; eta: float; u: float; g: float

class C2CurvatureProfile:
    @classmethod
    def from_knots(cls, knots: tuple[ControlKnot,...]) -> 'C2CurvatureProfile': ...
    def jet_at(self, s: float, side: str='auto') -> CurvatureJet: ...
    def interval_bounds(self, s0: float, s1: float) -> dict: ...
    # 包含每段c0..c3/h以及k/eta/u左右极限；所有s单位m

class VirtualAPath:
    # start_qf: FormationPose, profile: C2CurvatureProfile, model: FormationModel
    # 暴露length_m/start_qf/profile/model只读属性
    @classmethod
    def hold(cls, start_qf: FormationPose, model: FormationModel) -> 'VirtualAPath': ...
    def state_at_s(self, s: float, side: str='auto') -> FormationSpatialState: ...
    def endpoint(self) -> FormationPose: ...

def map_spatial_state(qf: FormationPose, jet: CurvatureJet,
                      model: FormationModel, *, s: float=0.0) -> FormationSpatialState: ...
def validate_initial_state(start_qf, start_headings, initial_assumption,
                           kappa0=0.0, eta0=0.0) -> ValidationResult: ...
def check_vehicle_curvatures(state, limits) -> ValidationResult: ...
```

空间state按ID包含`x/y/theta/C_i,p_s,p_ss,p_sss,H_i,H_i_s,H_i_ss,C_i_s,C_i_ss`和位置/角误差界。未来时间规划直接链式求导，不再从离散点估计。

实例字段统一采用主方案名称`curvature,theta_s,theta_ss,theta_sss,curvature_s,curvature_ss`；C_i/H_i仅为公式别名。hold是显式零运动kind、profile=None、length_m=0，只接受s=0并返回零曲率/直行A状态；不能伪造两个相同s的Hermite结点。

InitialAssumption字段为`stationary,mode,wheels_straight,source,context_id=''`；source首版仅`offline_fixture/rviz_reset/verified_terminal`，G1测试用offline_fixture。ROS来源的非空context和上下文绑定由G5节点核对，G1不能把任意字符串当作真实状态证明。

`side='auto'`内部节点取右，终点取左；显式left/right可检查g跳变。域外s、NaN、重复/乱序结点、非正段长、未知mode、非C2均结构化拒绝；不得无声clip域外s。常曲率profile本身可用于数学特例，但不因此通过初始直行请求检查；“曲线可计算”和“首版请求可接受”分开。

## 4. 数学实现规范

### 4.1 位置、航向和曲率

prime对F弧长s；`k[1/m],eta[1/m²],u[1/m³],g[1/m⁴]`。

```text
p_i=p_F+R(psi)*r_i
A=1-b*k; B=a*k; D=A²+B²; Q=sqrt(D)
theta_i=psi+atan2(B,A)
H_i=theta_i_s=k+a*eta/D
C_i=kappa_i=H_i/Q
```

初态真实车头与psi误差`<=1e-7 rad`、声明静止/统一A/直行轮系、k0=eta0=0才接受。未知声明source/非有限值拒绝；不可先替换输入车头再校验。

当前构型正常分支`A_i>0,D_i>0`；曲率上限`1/.810`，参考点侧车界`|k|<=1/(1+.810)`，前车还受eta影响。即使只有F转弯曲率符号，前车可以不同号；统一A不要求统一曲率符号。

### 4.2 空间jets

令`D_s=2*((a²+b²)*k-b)*eta`、`D_ss=2*((a²+b²)*eta²+((a²+b²)*k-b)*u)`：

```text
Q_s=D_s/(2*Q); Q_ss=D_ss/(2*Q)-D_s²/(4*Q³)
H_s=eta+a*(u/D-eta*D_s/D²)
H_ss=u+a*(g/D-2*u*D_s/D²-eta*D_ss/D²+2*eta*D_s²/D³)
C_s=H_s/Q-H*Q_s/Q²
C_ss=H_ss/Q-2*H_s*Q_s/Q²-H*Q_ss/Q²+2*H*Q_s²/Q³
z1=(1-b*k,a*k)
z2=(-a*k²-b*eta,k-b*k²+a*eta)
z3=(-k²+b*k³-3*a*k*eta-b*u,eta-3*b*k*eta+a*u-a*k³)
p_s=R*z1; p_ss=R*z2; p_sss=R*z3
```

g可跳变，k/eta/u在内部节点必须连续。不要求初终u为零；G1解析基准恰有该性质。

### 4.3 生产C2 Hermite

优先复用`MotionBlock(mode=0,direction=1,slope_policy='fixed')`，新wrapper独立生成归一化系数并验证，不修改旧组件：

```text
c0=k0; c1=h*eta0
c2=3*(k1-k0)-h*(2*eta0+eta1)
c3=2*(k0-k1)+h*(eta0+eta1)
k=c0+c1*q+c2*q²+c3*q³
eta=(c1+2*c2*q+3*c3*q²)/h
u=(2*c2+6*c3*q)/h²; g=6*c3/h³
C2: (2*c2_left+6*c3_left)/h_left² == 2*c2_right/h_right²
```

C0/C1/C2容差分别1e-9、1e-9、1e-8，单位依次1/m、1/m²、1/m³。`monotone`会改变导数，不使用。优先验证三次方案，不先升级五次。

### 4.4 积分、误差和区间基础

psi由曲率多项式精确积分；x/y为`∫cos(psi),∫sin(psi)`，不是折线插值。权威积分版本固定，任意s由前缀加局部积分恢复。

实现外扩区间Horner/二分界k/eta/u/g，不能取采样最大值冒充sup。生产积分至少提供主方案的复合Simpson余项方案：

```text
M4=G+4*K*U+3*E²+6*K²*E+K⁴
单面板宽H，每坐标积分余项<=H⁵*M4/2880
总预算=各面板余项之和+浮点/超越函数保护
```

K/E/U/G来自本面板全域界，面板不跨Hermite节点；二维误差保守合成。目标总位置误差`<=1e-7 m`、角误差`<=1e-9 rad`；支持`|psi|<=100 rad`，记录舍入/libm假设，不声称形式化浮点证明。Gauss/quad可快算或独立对照，quad估计误差/两次网格差不能单独作全域证明。达不到界时明确`BOUND_UNCONFIRMED`，不能忽略误差继续认证。

本阶段建立局部边界算子及余项核验，完整GEOMETRY证书在G2实现。

## 5. 强制测试输入和独立验证

### G1-P1 完整双移线与镜像

第一段`k=K*sin³(2*pi*s/L)`，第二段`k=-K*sin³(2*pi*(s-L)/L)`。K=.30 1/m，L=6 m；镜像K=-.30。工具提供解析eta/u/g，起点F=(0,0,0)。这是工具/测试的解析profile，不转换成拟合Hermite后冒充解析真值，也不添加到生产规划请求。

每段局部`t=c*s_local,c=2*pi/L`、符号sigma=+1/-1时，`eta=sigma*3*K*c*sin²(t)*cos(t)`，`u=sigma*3*K*c²*(2*sin(t)*cos²(t)-sin³(t))`，`g=sigma*3*K*c³*(2*cos³(t)-7*sin²(t)*cos(t))`；内部拼接按左右表达分别取极限。

独立闭式psi：第一段`psi=(K/c)*(2/3-cos(c*s)+cos³(c*s)/3),c=2*pi/L`，第二段镜像；用严格quad积分位置，然后五点差分三车位置恢复切线和曲率，对照生产映射。

期望：

- F曲率四符号`+,-,-,+`；镜像相反。
- `p_F(L)=(5.8222549849,1.1256889273)`，`p_F(2L)=(11.6445099698,约0)`，两处psi=0。
- `max|eta|≈.3627598728`；前车峰值约.526313、侧车约.428571；不得把峰值采样等同严格极值。
- s=0,L/2,L,3L/2,2L检查k/eta/u及三车状态左右极限；刚性d12=d13=sqrt(2)、d23=2。
- 镜像后car1曲率取反，car2/car3按ID互换并取反；位置/车头也作对应映射。
- 可用固定w=.5 m/s核对`v_i=Q_i*w,omega_i=H_i*w`，仅诊断，不宣称动态审核通过。

### G1-N1 前车eta超限

同K=.30、L=.5 m。前车曲率峰值约4.34822>1/.810，侧车仍约.428571；必须`CURVATURE_LIMIT,vehicle_id=1`，附s和实际值。不能仅断言“抛了任意异常”。

### 其他必须用例

1. 直线k=eta=u=g=0：三车车头=psi，三车速度比例1。
2. 常曲率k=.2、eta=u=g=0：与合法旧固定pivot(0,5)圆弧比较实际中心/车头/曲率；该特例不要求符合首版初态直行。
3. 生产C2 profile：结点s=(0,1,2)，k=.01*s²、eta=.02*s，u=.02连续，与旧Hermite数值一致。
4. 非C2反例：结点`(s,k,eta)=[(0,0,0),(1,.1,0),(2,.3,0)]`，必须拒绝并定位内部结点。
5. 非零g跳变但C2合法profile，验证节点侧别；不得因g不连续而额外拒绝。
6. s域外、零/负h、乱序/重复结点、NaN、错ID/offset、侧车A=0和正常分支外、真实车头car1偏差.1rad、声明非静止/非A/非直行。
7. 对独立解析二次/三次曲率核对积分余项包络，误差不会通过查询顺序、cache有无、节点侧别而超预算。

差分用h=.001和.0005 m，选121个避开不光滑g边界的内部点；边界另用解析左右极限。位置二阶差分存在消减误差，不强制缩h必然单调改善。

用于检验映射公式的差分位置由独立闭式psi+严格quad生成，而不是把默认允许1e-7m误差的生产位置直接做二阶差分后仍要求1e-5曲率误差。生产VirtualAPath的位置另按已报告积分误差包络与独立oracle比较；两类测试分开，不能用过粗位置积分导致的差分误差要求放宽运动学公式。

## 6. 子任务顺序和命令

G1-A先实现点态公式/输入拒绝并运行geometry测试；G1-B再实现C2wrapper/积分界并运行profile测试；G1-C最后构造双移线、镜像、负例和图表。不在同一变更里加入G2优化器。

未来实际命令：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src${PYTHONPATH:+:$PYTHONPATH}"
VA_RUN="evaluation/virtual_formation_a/G1/$(date -u +%Y%m%dT%H%M%SZ)"
mkdir -p "$VA_RUN/logs"
set -o pipefail
python3 -B -m unittest discover -s tests -p 'test_virtual_a_geometry.py' -v 2>&1 | tee "$VA_RUN/logs/geometry.log"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_profile.py' -v 2>&1 | tee "$VA_RUN/logs/profile.log"
python3 -B tools/virtual_a_benchmark.py --stage G1 --output "$VA_RUN/benchmark"
python3 -B tools/virtual_a_baseline.py --output "$VA_RUN/legacy" --regression geometry
```

不得直接discover全部旧测试污染FR01–FR04输出。环境/权限未执行项记录NOT_RUN，不计通过。setup.py登记后核对包解析；完整catkin/ROS构建留G5，不声称G1已完成ROS验证。

## 7. 量化验收和交付

- 刚性中心残差<=1e-6m，双移线端点/方向误差<=1e-6m/rad。
- 差分航向误差<=1e-6rad，曲率误差<=1e-5 1/m，保存每种h结果。
- C2公差及积分预算满足，节点位置/车头连续，g左右有界且语义一致。
- 两个解析正例和全部负例符合指定reason/vehicle/location；未知输入不能产出“数学通过”。
- 不输出GEOMETRY_FEASIBLE/EXECUTABLE，不声称实际硬件已标定可行。

交付新源码/测试、`COMMANDS.md`、参数和初态声明、数值CSV/JSON、双移线/镜像轨迹与各车heading图、k/eta/各车曲率图、积分余项/差分/C2证据、负例详情、保护哈希前后、`CLAUDE_REPORT.md`。图用现有绘图工具，不为G1另建可视化服务。

## 8. Codex独立验收

- 独立读公式/单位，手算侧车和前车eta项；核查旧常pivot导数没有混用。
- 自己用闭式psi和不同积分/差分恢复三车曲率，不以生产map函数结果当expected。
- 检查L=.5反例确为前车超限，而不是任意不相关异常。
- 从原始knots独立重建C2左右极限和积分余项，不只看profile自报bound。
- 核查反向/奇异分支和真实初态拒绝，没有输入覆盖；确认解析sin³不进入自主后端。
- 检查setup登记、导入依赖方向、新工具不会写旧证据、所有保护文件不变。

**停止：完成G1后等待Codex验收和用户下达G2。不得为“顺便演示”增加NLP/ROS或提交Git。**
