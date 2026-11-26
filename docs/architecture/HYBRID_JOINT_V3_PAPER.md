# CopTrans V3 核心规划与控制算法

本文描述 `hybrid-joint-v3` 分支算法快照 `22a1230991ef751f388154cb815212de7f201540`，用于撰写方法章节、核对公式和复现实验。默认单车与三车后端分别为 `single_hybrid_joint_cpp` 和 `formation_hybrid_joint_cpp`。两者共用样条运动模型、混合模式搜索、非线性优化和连续轨迹认证，执行侧使用 C++ 轨迹跟踪器。三车 CTS 在运输前运行一轮集结规划和停靠检查。默认参数取自 [统一测试配置](../../ros2_ws/src/fwis_bringup/config/test_defaults.yaml)。

## 1. 问题定义与系统结构

给定静态占据栅格 $\mathcal{M}$、参考起终位姿、车体初始航向、允许的运动模式、车体及货物包络和运动上限，寻找一条有限时长的分段轨迹。单车以底盘中心为参考点。CTS 运输以编队坐标系 $F$ 为参考，每辆车的几何中心对应同一个固定挂点。挂点允许车身绕竖直轴相对于货物旋转，三车在同一参考时间下跟踪各自的位姿和速度。

规划输入还包含逐车终点航向是否约束、是否允许倒挡、任务时长上限，以及是否尝试终点车身对齐。CTS 默认约束货物终点位置和朝向，车辆在运输终点的朝向由轨迹切向或运动模式决定，随后在有空间时对齐。单车的目标位姿包含自身终点航向。

```mermaid
flowchart LR
    M["占据栅格及起终点"] --> P["C++ 混合模式规划"]
    D["CTS 集结与挂点分配"] --> P
    P --> V["连续轨迹认证"]
    V --> E["带时间戳和摘要的轨迹模型"]
    E --> A["C++ 协调器复核和统一准入"]
    E --> T["各车 C++ 跟踪器"]
    A --> T
    T -->|车体 Twist| G["Gazebo 底盘模型"]
    G -->|里程计和模式反馈| T
    G -->|三车位姿| C["货物位姿拟合与误差测量"]
```

规划器求解名义运动，协调器负责身份匹配和同步许可，各车进行本地闭环跟踪。货物位姿和编队误差可供实验监测和停车判定。默认运输控制律采用各车局部误差，车辆之间的几何协调来自联合参考轨迹和共同时间。 [规划核接口](../../ros2_ws/src/fwis_hybrid_joint/include/fwis_hybrid_joint/model.hpp) · [规划节点](../../ros2_ws/src/fwis_hybrid_joint_ros/src/planner.cpp) · [执行协调器](../../ros2_ws/src/fwis_hybrid_joint_ros/src/executor.cpp)

## 2. 三车与货物的几何耦合

定义 $p_F=[x_F,y_F]^{\mathsf T}$，$\psi$ 为编队朝向，$o_i$ 为第 $i$ 辆车在编队系内的挂点偏置，$o_C$ 为货物几何中心偏置。$R(\psi)$ 是二维旋转矩阵，$J=\begin{bmatrix}0&-1\\1&0\end{bmatrix}$。令 $r_i=R(\psi)o_i$，则

$$
p_i=p_F+r_i,\qquad p_C=p_F+R(\psi)o_C.
$$

以相位的归一化曲线参数 $s\in[0,1]$ 求导，撇号表示对 $s$ 的导数。

$$
p_i'=p_F'+\psi'Jr_i,\qquad
p_i''=p_F''+\psi''Jr_i-(\psi')^2r_i,\qquad
\dot p_i=p_i'\dot s .
$$

因此，同一条货物参考曲线会产生三条速度、曲率各异的挂点轨迹。货物中心限速施加在 $\|\dot p_C\|$ 上。编队名义车间距满足 $\|p_i-p_j\|=\|o_i-o_j\|$。车身航向 $\theta_i$ 独立于货物朝向 $\psi$，碰撞检查按照各自的真实模型朝向放置车体包络。

CTS 楔形挂点为 $(1,0)$、$(0,1)$、$(0,-1)$ m，车辆与挂点的对应关系由集结任务确定。货物平面尺寸 $1.95\times2.70$ m，$o_C=(0.50,0)$ m，车体平面尺寸 $0.738\times0.500$ m。编队规划通常在车体和货物矩形每侧加 $0.05$ m 的碰撞裕量。RViz 和用例配置中的货物中心位置 $p_C$ 进入求解前转换为 $p_F=p_C-R(\psi)o_C$。速度统计、终点位置误差和挂点误差分别按各自的参考点计算。单车取 $o_1=0$ 并省略货物，模型输出的参考航向等于车身航向。 [运动学实现](../../ros2_ws/src/fwis_hybrid_joint/src/model.cpp) · [请求和包络参数](../../ros2_ws/src/fwis_hybrid_joint_ros/src/request.cpp) · [货物配置](../../ros2_ws/src/ranger_mini_gz/config/cargo_geometry.yaml) · [CTS 用例](../../ros2_ws/src/fwis_bringup/config/cts_long_routes.yaml)

### 2.1 三种运动模式

记 $g_i\in\{-1,+1\}$ 为第 $i$ 辆车的方向分支，$\det(a,b)=a_xb_y-a_yb_x$。同一运输运动相位共享模式和时间律，车辆可具有不同的方向分支和车身姿态。允许倒挡的开关作用于平移相位各车的方向分支。

| 模式 | 位姿与控制量 |
| --- | --- |
| A · Ackermann | $\theta_i=\operatorname{atan2}(g_i p'_{iy},g_i p'_{ix})$，要求挂点切向非退化。$v_{x,i}=g_i\lVert p_i'\rVert\dot s$，$v_{y,i}=0$，$\omega_i=\det(p_i',p_i'')\dot s/\lVert p_i'\rVert^2$。有符号曲率 $\kappa_i=g_i\det(p_i',p_i'')/\lVert p_i'\rVert^3$。同一相位允许连续变化曲率。 |
| O · Oblique，斜行或横移 | 货物朝向和各车车身航向在相位内保持常量。$\left[v_{x,i},v_{y,i}\right]^{\mathsf T}=R(-\theta_i)p_i'\dot s$，$\omega_i=0$。相位可沿曲线平移，等效舵角取平移切向在车体坐标系中的方向并按 $g_i$ 选支。 |
| S · Spin | 编队参考和车辆中心固定。$\theta_i(s)=\theta_{i,0}+s(\theta_{i,1}-\theta_{i,0})$，$v_{x,i}=v_{y,i}=0$，$\omega_i=(\theta_{i,1}-\theta_{i,0})\dot s$。原地转向改变各车车身航向，货物朝向保持原值。 |

模式之外还有静止的 `Prepare` 和 `Hold` 相位。前者为轮端转向与反馈确认预留时间，后者保持位姿。论文统计模式切换时，可分别记录 A/O/S 的切换和同模式相邻运动相位之间的停车次数。 [相位定义与解析采样](../../ros2_ws/src/fwis_hybrid_joint/include/fwis_hybrid_joint/model.hpp) · [采样公式](../../ros2_ws/src/fwis_hybrid_joint/src/model.cpp)

## 3. 连续轨迹、时间律及约束

### 3.1 空间轨迹与时间参数化

一个运动相位有 $n$ 个等参数跨度的五次 Bernstein 曲线片。对 $f\in\{x_F,y_F,\psi\}$，第 $j$ 片的曲线写成

$$
f_j(z)=\sum_{k=0}^{5} b_{j,k}^{f}\binom{5}{k}z^k(1-z)^{5-k},
\qquad z=ns-j,\qquad z\in[0,1].
$$

编队航向系数沿连续角度分支表达。求解器按具体候选使用显式 $C^3$ 连接约束，或用五次 B 样条的双重内结点表达相同的连续函数空间，再经结点插入转换为 Bernstein 系数。同一运动相位内各片共享单一时钟，拼接处保持位置、切向和曲率连续，车辆可以持续运动。

对相位时长 $T_k$，令 $u=(t-t_k)/T_k$，再取 $s=\sigma(u)$。静止起停的五次时间律为

$$
\sigma(u)=10u^3-15u^4+6u^5,\qquad
\dot s=\sigma'(u)/T_k,\qquad
\ddot s=\sigma''(u)/T_k^2 .
$$

A/O 还可使用首尾三次速度坡与中段等归一化速度相接的时间律。设单侧速度坡宽度为 $r$，峰值为 $1/(1-r)$，在前坡令 $z=u/r$，则 $\sigma'(u)=(3z^2-2z^3)/(1-r)$，后坡镜像对称。实现支持 $r\in\{1/64,1/32,1/16,1/8\}$。比 $1/8$ 更窄的曲线与 $1/8$ 宽坡按 $7:1$ 混合。这里的恒定量是 $\dot s$，车体实际速度还随 $\|p_i'(s)\|$ 变化。运动相位端点的速度和加速度为零，加加速度在分段边界按两侧的界检查。整个任务时间为运动、准备和静止相位的时长之和。 [时间律实现](../../ros2_ws/src/fwis_hybrid_joint/src/clock.hpp) · [空间参数化及 NLP](../../ros2_ws/src/fwis_hybrid_joint/src/casadi_solver.cpp)

### 3.2 运动学与轮端约束

每一运动相位检查各车的平移速度、沿路径加速度和加加速度、横向加速度，以及航向角速度、角加速度和角加加速度。A 模式还要求 $|\kappa_i|\le1/R_{\min}$。O 模式约束舵角和舵角变化，S 模式约束原地转动及对应轮端速度。货物中心限速作为单独约束。

设某轮在车体坐标系的位置为 $(x_w,y_w)$。车体 Twist 给出的车轮位置处平面速度为 $(v_x-y_w\omega,\ v_y+x_w\omega)$。A 模式中当 $1-y_w\kappa>0$ 时，模型使用

$$
\delta_w=\operatorname{atan2}(x_w\kappa,1-y_w\kappa),\qquad
|v_w|=|v_x|\sqrt{(1-y_w\kappa)^2+(x_w\kappa)^2}.
$$

O 模式的舵向由车体平移方向得到。S 模式的舵向与以车体中心为圆心的转动切向一致，轮速模为 $|\omega|\sqrt{x_w^2+y_w^2}$。证书检查轮速、舵角、舵角速率与加速度。以轮间最大舵角变化量 $\Delta\delta$ 计算准备时间下限

$$
T_{\mathrm{prepare}}=T_0^{\mathrm{prep}}+10^{-4}
+\max\left\{\frac{1.875\,\Delta\delta}{\dot\delta_{\max}},\
\sqrt{\frac{5.774\,\Delta\delta}{\ddot\delta_{\max}}}\right\}.
$$

标准配置的协议准备基线 $T_0^{\mathrm{prep}}=0.70$ s。应用层发送车体 Twist，轮端状态在此用于计算约束及准备时长。 [独立运动与轮端证书](../../ros2_ws/src/fwis_hybrid_joint/src/certificate.cpp) · [准备相位构造](../../ros2_ws/src/fwis_hybrid_joint/src/search.cpp)

## 4. 混合模式规划算法

将离散结构 $q$ 定为运动模式、方向分支和连接顺序。在给定 $q$ 的候选上优化曲线系数、相位时长以及允许调整的连接状态。外层在有限墙钟预算内搜索多个 $q$ 和几何初值，连续子问题由 CasADi 的符号自动微分与 Ipopt 求解。段间静止、挂点、端点条件和配点约束组成固定候选的非线性规划，最终可执行性由独立证书判定。 [候选搜索](../../ros2_ws/src/fwis_hybrid_joint/src/search.cpp) · [NLP 实现](../../ros2_ws/src/fwis_hybrid_joint/src/casadi_solver.cpp) · [Ipopt 原论文](https://doi.org/10.1007/s10107-004-0559-y)

### 4.1 生成初值与选择边界条件

1. 生成直连、正车和倒车候选。存在障碍时，从 8 邻域占据栅格搜索几何通道，禁止对角穿角。以 $d$ 表示栅格近似净空、$R_{\mathrm{env}}$ 表示外包范围，边代价含行进距离、近障碍惩罚 $1.5(R_{\mathrm{env}}/\max(0.15,d))^3$，宽通道种子可增加起始切向惩罚。
2. 在通道参考曲线上构造货物朝向初值。用货物多圆盘近似和车体外接圆估计通道净空，在离散朝向上做动态规划，以同时考虑净空、朝向偏移和相邻朝向变化。需要时对姿态变化区间作有约束的样条预修复。初值仍按矩形车体和货物重新认证。
3. 检查端点 A 模式的挂点切向兼容性。对三车共同的未知 $[\dot x_F,\dot y_F,\dot\psi]^{\mathsf T}$ 的方向约束矩阵，第 $i$ 行可写为

$$
\left[-\sin\theta_i,\ \cos\theta_i,\
      \sin\theta_i\,r_{iy}+\cos\theta_i\,r_{ix}\right].
$$

   A 模式需要可使各挂点沿各自车身方向运动的非零切向。实现对精确边界的秩及端点航向容差邻域分别检查。只有对容差邻域仍成立的满秩证据才用于剪去相应边界的 A 候选。
4. 优先检查连续 A。固定空间曲线先尝试可认证的时间律；有限预算内还搜索斜行、转向和换挡连接，允许插入、删除、合并和替换运动模式子序列。单车另有终端切向候选。 [通道航向初值](../../ros2_ws/src/fwis_hybrid_joint/src/channel_spline.hpp) · [姿态修复](../../ros2_ws/src/fwis_hybrid_joint/src/channel_repair.hpp) · [搜索和端点约束](../../ros2_ws/src/fwis_hybrid_joint/src/search.cpp)

### 4.2 数值求解、分段恢复与结果选择

候选的硬约束包含样条连接、端点条件、相位时长与轮端准备、配点动力学和由障碍与车体生成的局部分离条件。数值目标含时间、路程及平滑项，部分约束按数值求解需要收紧。认证失败后可根据见证区间添加碰撞分离条件或局部曲率条件。速度、舵速及加速度类失败可尝试扩时或更换时间律。迭代达到上限的候选也可进入独立认证，交付以认证和结果状态为准。

CTS 的局部恢复处理两类典型困难边界。起步航向有偏差且落在共同容差内时，可先让货物以当前姿态短距离直行，停稳后再求主段。狭窄终点可预留一段保持目标货物朝向的直线入位，在进入该段前于可行空间完成转向。较急的离场可包含停稳后的 Spin，亦可从终点反向构造候选再恢复实际方向。对快检尚无连续方案且存在相应几何迹象的任务，提前尝试分段；分段失败后继续原候选。前段、后段及准备相位拼接后，按**同一个原始请求**重新认证。末端车身对齐也是一次附加并重证的 Spin，空间受限时保存通过认证的运输计划。 [分段与终点对齐](../../ros2_ws/src/fwis_hybrid_joint/src/search.cpp)

最终策略在已经认证的候选中优先选择无辅助运动块的轨迹。然后比较辅助代价 $J_{\mathrm{aux}}$，在比较容差内再按执行代价 $J_{\mathrm{exec}}$ 排序。定义 $R_{\min}$ 为参考半径、$T_*=R_{\min}/v_{\max}$，$L_O$ 与 $L_F$ 分别为 O 相位和整轨参考点的累计长度，$N_{\mathrm{aux}}$ 为辅助块数，则

$$
\begin{aligned}
J_{\mathrm{aux}}&=0.5N_{\mathrm{aux}}+L_O/R_{\min}
  +0.25\sum_{\mathrm{S},i}|\Delta\theta_i|/(1\,\mathrm{rad}),\\
J_{\mathrm{exec}}&=T/T_*+0.10L_F/R_{\min}
  +0.02E_{\mathrm{steer}}+0.02E_{\mathrm{jerk}}+0.04E_{\mathrm{clear}}.
\end{aligned}
$$

各 $E$ 为归一化的转向速率、加加速度和净空代价。评分按每个几何片 24 个区间累积转向速度平方，以局部速度的二阶差分估计加加速度，Spin 对偏航速度做相应计算。额外净空从零到 $0.20$ m 分为 8 级，按欠缺量的平方赋予软代价。整个轨迹的硬碰撞检查在认证器中进行。论文可把这种策略描述为**有限预算下的 Ackermann 优先可行规划**。 [候选评分代码](../../ros2_ws/src/fwis_hybrid_joint/src/policy.cpp) · [权重配置](../../ros2_ws/src/fwis_hybrid_joint/config/policy.json)

### 4.3 连续轨迹认证

1. 检查参数、相位结构、端点及相位连续性。平面端点位置容差为 $0.02$ m，受约束的参考或车身朝向容差为 $1^\circ$。准备和保持相位须满足静止条件及准备时间要求。
2. 对每个空间曲线片覆盖其归一化时间区间。在区间中点按原始障碍栅格检查货物与地图、车辆与地图、车与车之间的 OBB 分离。未知占据按障碍处理。
3. 用 Bernstein 包络和向外舍入的区间算术约束整段导数与轮端状态。设 $\rho_C$、$\rho_i$ 为矩形外接圆半径，区间内的角点速度上界可分别取

$$
B_C\ge\|\dot p_F\|+(\|o_C\|+\rho_C)|\dot\psi|,
\qquad
B_i\ge\|\dot p_i\|+\rho_i|\omega_i|.
$$

   取这些上界的最大值 $B$。中点 OBB 按 $B\Delta t/2$ 加数值重构裕量外扩，覆盖该时间片内的扫掠。若包络过宽或约束界不明确，细分时间片继续检查。判定失败保留相位、车辆索引及区间位置；超过深度或墙钟预算而无法证成时返回相应拒绝原因。
4. 输出认证结论供规划器、离线复核和执行协调器使用。初值净空、Ipopt 返回状态和显示采样各有用途，可执行性统一使用连续证书。 [区间认证实现](../../ros2_ws/src/fwis_hybrid_joint/src/certificate.cpp) · [栅格与 SAT 内核](../../ros2_ws/src/fwis_planning_core/include/fwis_planning_core/collision.hpp)

### 4.4 计算复杂度与本版内核优化

运行耗时依赖可达栅格数、通道朝向样本数、候选数、样条跨度、非线性迭代和证书细分深度。数值迭代与碰撞复核的计算量随障碍和几何约束变化。当前实现按栅格复用净空代价，在同一候选内复用未改变的分离约束，在相同采样位置共享参考与挂点符号导数。货物姿态修复的 Jacobian 仅在各控制点的 B 样条局部支撑范围内计算采样导数。认证器按所需导数阶数计算区间值，并复用逆时标搜索中的相同求值。  [搜索复用](../../ros2_ws/src/fwis_hybrid_joint/src/search.cpp) · [符号导数复用](../../ros2_ws/src/fwis_hybrid_joint/src/casadi_solver.cpp) · [局部支撑](../../ros2_ws/src/fwis_hybrid_joint/src/channel_repair.hpp) · [认证实现](../../ros2_ws/src/fwis_hybrid_joint/src/certificate.cpp)


## 5. CTS 集结、定时与运输接续

CTS 由三辆车从场景预设车位驶向货物挂点。货物的二维平面包络进入运输规划。集结阶段对三车参加者和场景障碍使用单车轨迹与车间扫掠检查，模拟货物位于底盘上方的停靠过程。

1. 根据输入货物姿态计算三个挂点目标。自动分配时最多求解 $3\times3$ 个车辆到挂点的独立单车任务，指定分配时求解对应的三条。每条轨迹生成后单独认证。单车求解最多并行三个工作进程。直接求解失败时可用离散停靠引导生成其他初值，候选仍由原生内核认证。
2. 在具备可行单车路径的分配中枚举挂点排列。对每个分配枚举发车优先顺序，按零时刻、已有车辆结束时刻和离散秒级延时检查冲突，选择已评估方案中最早完成的一组。第 $i$ 辆车延时为 $d_i$、自身轨迹时长为 $T_i$，评估目标为 $\min\max_i(d_i+T_i)$。该枚举限制在已生成的轨迹和延时选择内。
3. 对每对车辆检查移动、等待和停靠阶段的矩形扫掠。在候选时间片中点求 SAT 净距，用车辆中心速度与最大角点速度推导区间下界。等待或已停车的车辆贡献零速上界。无法证成时继续细分，在 $8$ ms 以内仍无法证成则拒绝。若所有时间安排都受已停靠车辆阻挡，生成考虑停靠区预留的备选路径，再用原始地图和车间扫掠复核。
4. 实际停靠同时检查 Gazebo 位姿和里程计，车中心位置误差上限 $0.06$ m，车身真值航向误差上限 $0.10$ rad，并要求三车在 A 模式停稳保持 $0.15$ 个仿真秒。集结中监测车间间隙和局部跟踪异常，部分故障可在停稳后从最新反馈重新规划一次。

从实测车中心 $\tilde p_i$ 与已知 $o_i$ 做二维刚体最小二乘拟合。以 $\bar p,\bar o$ 表示质心，拟合公式为

$$
\begin{aligned}
\hat\psi
&=\operatorname{atan2}\!\left(
  \sum_i \det(o_i-\bar o,\tilde p_i-\bar p),
  \sum_i (o_i-\bar o)^{\mathsf T}(\tilde p_i-\bar p)\right),\\
\hat p_F&=\bar p-R(\hat\psi)\bar o.
\end{aligned}
$$

运输请求采用这个拟合参考位姿。发送给规划器的车中心按挂点几何重建，逐车初始航向取停稳时的里程计航向。这样每次 Gazebo 闭环都从当轮实测停靠条件开始。默认离线回归中的两条历史失败用例使用归档分配及停靠真值复现原运输问题。 [集结规划与调度](../../ros2_ws/src/fwis_bringup/fwis_bringup/assembly_plan.py) · [集结执行](../../ros2_ws/src/fwis_bringup/fwis_bringup/assembly_execution.py) · [刚体拟合](../../ros2_ws/src/fwis_bringup/fwis_bringup/formation_route.py) · [运输请求构造](../../ros2_ws/src/fwis_bringup/scripts/run_experiment)

## 6. C++ 分模式闭环控制

### 6.1 参考与反馈控制律

跟踪器每周期以当前 ROS 时间减共同启动时刻 $t_0$，在 `HybridTimedPlan/v2` 模型中直接解析求值本车的参考位姿和 Twist。参考来自与规划认证一致的 Bernstein 系数和相位时标。RViz 路径与显示采样用于观察。位置误差表达在实际车体坐标系中

$$
\begin{bmatrix}e_x\\e_y\end{bmatrix}
=R(-\theta)(p_r-p),\qquad
e_\theta=\operatorname{wrap}(\theta_r-\theta).
$$

默认 C++ 控制律为非线性速度反馈，另可显式选择 PID。设参考车体系 Twist 为 $(v_{x,r},v_{y,r},\omega_r)$。A 模式采用 Kanayama 型跟踪，其中 $v_r=v_{x,r}$ 带正倒车符号

$$
v_x^*=v_r\cos e_\theta+k_x e_x,\qquad
v_y^*=0,\qquad
\omega^*=\omega_r+k_yv_r e_y+k_\theta|v_r|\sin e_\theta.
$$

默认增益 $(k_x,k_y,k_\theta)=(1.35,9.0,6.0)$。在理想单轨运动学、连续 A 相位且限幅未激活的条件下，可取 $V=(e_x^2+e_y^2)/2+(1-\cos e_\theta)/k_y$，得到

$$
\dot V=-k_xe_x^2-(k_\theta/k_y)|v_r|\sin^2e_\theta\le0 .
$$

该关系解释连续 A 反馈项的作用。完整系统还包含停稳、确认、限幅、离散采样和仿真传输延迟，闭环结论以实测为依据。 [控制律代码](../../ros2_ws/src/fwis_control/src/tracking_model.cpp) · [Kanayama 等，1990](https://doi.org/10.1109/ROBOT.1990.126006)

O 模式先把参考平动换算到实际车体系，再加二维位置误差

$$
\begin{bmatrix}v_x^*\\v_y^*\end{bmatrix}
=R(e_\theta)\begin{bmatrix}v_{x,r}\\v_{y,r}\end{bmatrix}
+1.65\begin{bmatrix}e_x\\e_y\end{bmatrix},\qquad
\omega^*=0 .
$$

S 模式使用 $v_x^*=v_y^*=0$ 和 $\omega^*=\omega_r+2.0e_\theta$。反馈控制之后才生成受执行条件约束的三轴 Twist。 [解析参考和预检](../../ros2_ws/src/fwis_control/src/hybrid_tracking.hpp) · [反馈和执行限幅](../../ros2_ws/src/fwis_control/src/tracking_model.cpp) · [默认控制增益](../../ros2_ws/src/fwis_control/config/ranger_tracking.yaml)

### 6.2 Ackermann 模式边界与切换

底盘命令为 $(\mathrm{linear.x},\mathrm{linear.y},\mathrm{angular.z})$。当前 Gazebo 模拟解码依次处理三种情况。非零横向速度编码 O 并令偏航速度为零。横向为零且 $|\omega|<10^{-6}$ rad/s 时按 A 直行处理。其余横向为零的命令中，$|v_x|<0.47644|\omega|$ 时解码为 S 并令平动为零，其余解码为 A。数值 $0.47644$ m 是该仿真配置的模式边界。 [Gazebo Twist 分类](../../ros2_ws/src/ranger_mini_gz/ranger_mini_gz/body_sim.py) · [实车接口背景](../ros2_migration/RANGER_TWIST_CONTROL.md)

当前规划与反馈用两级半径约束留出余量

$$
R_{\mathrm{plan}}=0.90\ \mathrm{m},\qquad
R_{\mathrm{guard}}=\max(0.810,0.47644)+0.02=0.83\ \mathrm{m},\qquad
|\omega_{\mathrm{cmd}}|\le |v_{x,\mathrm{cmd}}|/R_{\mathrm{guard}}.
$$

其中 $0.810$ m 为当前配置采用的底盘机械半径参数。在 A 相位内，控制器先去掉横向命令，限制纵向指令方向及可达转弯半径，再应用速度、角速度和变化率限制。接近零速时允许的偏航速度同步减小，为 A 到 S 的被动分类边界保留死区。正倒车换向先经过零速度。有需要时在进入高转向区前先限速。对最大平动变化和最大偏航变化使用同一个可达插值比例

$$
\gamma=\min\left(1,\frac{a_{\max}\Delta t}{\|v^*-v_{\mathrm{prev}}\|},
\frac{\alpha_{\max}\Delta t}{|\omega^*-\omega_{\mathrm{prev}}|}\right).
$$

对应分母为零的项取 $1$。O 模式守住参考横移侧。轴向附近可使用 $10^{-5}$ m/s 量级的非零横向编码来保持分类。连续斜行参考穿过零横向分量时，控制器根据纵向前进一致性和本周期的加速度可达性选择过渡，真实反向仍执行过零停车。 [曲率、方向与变化率护栏](../../ros2_ws/src/fwis_control/src/tracking_model.cpp) · [控制参数](../../ros2_ws/src/fwis_control/config/ranger_tracking.yaml)

计划切换通过静止的 Prepare 相位完成。控制器先制动并确认里程计低于 $0.012$ m/s 和 $0.020$ rad/s，保持稳定 $0.08$ s，随后发送仿真模式请求并等待反馈，确认期限 $0.30$ s，确认后再稳定 $0.08$ s。切换超期进入停车，三车按照相同的计划时钟执行。仿真模式请求话题与 Twist 一起用于模式协调，真实底盘协议应按相应硬件与固件另行验证。 [切换执行状态机](../../ros2_ws/src/fwis_control/src/tracking_execution.cpp) · [底盘仿真模式确认](../../ros2_ws/src/ranger_mini_gz/ranger_mini_gz/body_sim.py)

### 6.3 同步许可和运行异常

规划器发布带未来启动时刻的权威轨迹模型。模型封装包含请求身份、地图校验、运动限制、阶段系数、求解器构建指纹和模型摘要。协调器使用自己接收的地图、请求和限制重做轨迹认证，并收齐各车对应同一模型和时刻的预检应答。默认在发布 ROS 时间后 $1.25$ s 启动，准入截止为启动前 $0.20$ s。协调器复核预算至多 $0.40$ s，并受启动剩余时间约束。满足条件后周期发布共同执行许可。

跟踪控制周期按 $25$ Hz ROS 时间配置。运行侧用单调墙钟监控反馈和许可，里程计及模式反馈超时窗为 $0.20$ s。模型或许可身份不匹配、反馈超时、时钟回退、切换失败和计划撤销会触发停止。编队中活动成员的故障可撤销全组许可。共享启动时间给出统一参考相位，实际回调和数据传输的时延仍需单独测量。 [权威模型编码](../../ros2_ws/src/fwis_hybrid_joint_ros/src/codec.cpp) · [统一执行协调器](../../ros2_ws/src/fwis_hybrid_joint_ros/src/executor.cpp) · [跟踪节点及 watchdog](../../ros2_ws/src/fwis_control/src/tracking_controller_node.cpp)

## 7. 默认部署、话题及参数

| 数据 | 单车 | 三车 CTS 运输 |
| --- | --- | --- |
| 地图 | `/map` | `/map` |
| 规划输入 | `/start_pose` 或 `/initialpose`，加 `/move_base_simple/goal` | `/formation/plan_request` |
| 权威轨迹 | `/hybrid_joint/plan` | `/formation/playback_plan` |
| 执行许可 | `/hybrid_joint/execution_permit` | 同一话题发送三车公共许可 |
| 车体命令与反馈 | 车辆命名空间下的 `cmd_vel`、`odom`、`motion_state` | 各车分别接收命令并回传 |
| 运输求解进度 | `/plan/status`、`/plan/search_status` | `/formation/optimizer_status`、`/formation/uniform_progress` |
| 车与货物的观测 | 单车轨迹及误差 | 三车中心用来拟合货物参考，发布 `/cargo/odom` 和编队误差 |

`/cargo/odom` 的位姿表示拟合的编队参考点，速度按带偏置的货物几何中心计算。出图时应用 $o_C$ 显式换算，并在图注标明观察的是货物中心还是参考点。前端脚本、集结编排、结果归档和可视化主要使用 Python，单车/三车的主要规划模型、轨迹认证、运输准入及跟踪器使用 C++。 [ROS 请求和话题](../../ros2_ws/src/fwis_hybrid_joint_ros/include/fwis_hybrid_joint_ros/inputs.hpp) · [统一实验接口](../../ros2_ws/src/fwis_bringup/scripts/run_experiment)

| 量 | 默认 standard 原生配置 |
| --- | --- |
| 规划墙钟预算 | 单车 $8$ s，编队运输 $12$ s |
| 名义任务时长上限 | 默认 $300$ s，可按请求设定 |
| A 规划最小半径 | 单车和编队均为 $0.90$ m |
| 名义平动速度上限 | 单车车体 $1.50$ m/s，编队每车 $0.90$ m/s，货物几何中心另限 $0.80$ m/s |
| 高转角降速 | 舵角超过 $20^\circ$ 时，默认规划车体速度上限 $0.72$ m/s，默认 native 控制及仿真上限为 $0.75$ m/s |
| 名义参考配置 | 单车平动加速度上限 $0.42$ m/s²，编队 $0.36$ m/s²，轮距 $0.364$ m，轴距 $0.494$ m |
| 车体控制限幅 | 默认 native 速度 $1.50$ m/s，偏航速度 $0.55$ rad/s，平动加速度 $0.60$ m/s²，偏航加速度 $0.70$ rad/s² |

配置按场景及速度档覆盖，以每轮存档的 resolved config 和规划器参数为准。规划墙钟时间、轨迹计划时长、实际仿真时间、控制回调计算时间分别记录。 [统一测试配置](../../ros2_ws/src/fwis_bringup/config/test_defaults.yaml) · [规划器的限制解析](../../ros2_ws/src/fwis_hybrid_joint_ros/src/request.cpp) · [跟踪限制](../../ros2_ws/src/fwis_control/config/ranger_tracking.yaml)

## 8. 实验口径与论文表述范围

### 8.1 长路径规划对照

CTS 室内试验占据栅格为 $30\times24$ m，分辨率 $0.05$ m，包含 $2.60$ m 宽的 CTS 通道。下表为相同机器、输入、地图和 $12$ s 预算的三次中位数。计时为原生求解器墙钟耗时，包含内部认证。轨迹长度为 A 段的参考曲线累计长度。旧版为 `753f24a`，新版为 `22a1230`。本地证据编号 `20261126_175945_UUrhXv` 下的 `kernel_latency.json` 保存每次原始计时、源码身份、独立认证和路径差异。 [场景几何](../../ros2_ws/src/ranger_mini_gz/scenarios/factory_indoor_01.yaml) · [固定输入](../../ros2_ws/src/fwis_bringup/config/cts_long_routes.yaml)

| CTS 用例 | A 段长度（m） | 旧版求解（s） | 新版求解（s） | 降幅 |
| --- | ---: | ---: | ---: | ---: |
| `20261126_161917_u046w8` | 33.45 | 2.203 | 1.848 | 16.1% |
| `20261126_162144_qqdYea` | 37.38 | 1.280 | 1.050 | 18.0% |
| `CTS_S1` | 40.68 | 1.683 | 1.459 | 13.3% |
| `CTS_S3` | 39.26 | 2.600 | 2.236 | 14.0% |
| `CTS_E_TO_S3` | 38.87 | 2.677 | 2.225 | 16.9% |

两版在同一批九组归档输入上均独立认证通过，规划模式序列一致。另有三条本轮未用于调优的长窄通道，分别为单车一条、编队两条，起终点直距 $22.11$–$23.38$ m，均通过独立认证。这十二组比较中的轨迹控制点最大差异约 $2.03\times10^{-8}$ m。用这些数据可说明本轮计算复用降低了受测输入的求解耗时。

### 8.2 闭环指标与版本

取 $p_i^m(t)$ 为仿真实测车中心，$\hat p_F,\hat\psi$ 为同一时刻的三车刚体拟合结果。采样挂点误差和车间距误差定义为

$$
e_{\mathrm{mount}}(t)=\max_i\lVert p_i^m(t)-\hat p_F(t)-R(\hat\psi(t))o_i\rVert ,
$$

$$
e_{\mathrm{spacing}}(t)=\max_{i<j}
\left|\lVert p_i^m(t)-p_j^m(t)\rVert-\lVert o_i-o_j\rVert\right|.
$$

报告最大值取运输阶段的采样时间。货物最小净空采用模拟真值 OBB 的离散采样。终点误差比较最终拟合的编队参考和请求终点参考。以下结果各对应一次 Gazebo 闭环，可用于展示系统行为。 [实验指标实现](../../ros2_ws/src/fwis_bringup/fwis_bringup/run_metrics.py) · [闭环验收条件](../../ros2_ws/src/fwis_bringup/scripts/run_experiment)

| 本地实验编号 | 源码快照 | 任务 | 求解墙钟（s） | 计划运输（s） | 最大挂点误差（mm） | 终点位置误差（mm） |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `20261126_230038_Y1Tdbt` | `22a1230` | `CTS_S3` | 2.248 | 101.833 | 7.62 | 1.90 |
| `20261126_170430_m7MkZ3` | `bf8c701` | 归档起步用例闭环 | 6.430 | 92.725 | 6.01 | 1.51 |
| `20261126_170722_d0bjzr` | `bf8c701` | 归档入位用例闭环 | 9.608 | 114.399 | 18.02 | 12.71 |

三次均完成三车跟踪。前一行属于当前数值内核，后二行用于说明对应历史版本的实际闭环，与上表的规划配对统计各有自己的输入和版本。`reports/hybrid_v3` 保留较早冻结矩阵的摘要和版本清单，可追溯其场景和测试范围。原始逐轮日志保存在本地 `reports/runs/`，原始验收收据位于本地 `reports/acceptance/`。 [历史验收索引](../../reports/hybrid_v3/README.md) · [版本清单](../../reports/hybrid_v3/manifest.json)

### 8.3 假设、贡献和复现

可在论文中将系统概括为静态占据地图上以连续曲率 Ackermann 为优先的 A/O/S 联合运动规划。核心方法包括固定挂点的参考轨迹耦合、离散模式与连续样条的分层搜索、逐区间可执行性认证，以及共同时钟下的分布式跟踪。适用边界如下。

- 搜索的完备性和全局最优性未获证明。超时表示在指定时间内未找到合格候选。工作空间、足迹和限制属于给定请求。
- 这里的通道搜索用于产生初值和局部碰撞分离条件。GCS 的定义是在凸集图上联合优化离散路径与连续顶点变量，本文实现使用离散栅格初值和非凸样条 NLP。 [Marcucci 等，Shortest Paths in Graphs of Convex Sets](https://arxiv.org/abs/2101.11565)
- 时间分配在有限组可微时标与相位时长上求解并认证。TOPP-RA 的可达集和可控集递推属于另一类路径参数化算法。 [Pham 与 Pham，A New Approach to Time-Optimal Path Parameterization based on Reachability Analysis](https://arxiv.org/abs/1707.07239)
- 认证针对静态栅格、名义几何和运动学。Gazebo 通过受加速度限制的车体 Twist 模型积分底盘平面位姿，货物姿态由三车中心拟合并用于可视化。当前实验未验证轮胎侧滑、载荷力分配、挂点柔性、真实轮端闭环或动态障碍，也未建立端到端硬实时保证。 [底盘模型](../../ros2_ws/src/ranger_mini_gz/ranger_mini_gz/body_sim.py) · [场景货物更新](../../ros2_ws/src/ranger_mini_gz/scripts/scene_map)

从仓库根目录可复跑完整 CTS 离线测试。附加 `--case CTS_S3` 可选择一组。闭环示例会自动执行集结、运输规划与控制，仿真一次运行一条。

```bash
./scripts/test.sh --planner-only
./scripts/test.sh --task multi --case CTS_S3 --headless
```

离线日志的 `transport_s` 是求解墙钟耗时，`transport_duration_s` 是规划轨迹时长。闭环日志的 `result.yaml` 记录求解、完成、终点和挂点误差。建议将 Git 提交、模型与地图摘要、求解预算、模式序列、样本数和计时定义一并写入论文实验表。 [测试命令](../../README.md) · [离线测试与日志写入](../../ros2_ws/src/fwis_bringup/fwis_bringup/cts_offline.py)
