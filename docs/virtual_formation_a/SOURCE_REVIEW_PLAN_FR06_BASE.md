# 从 FR06 多车快照派生虚拟运输集群连续 A 模式规划：实施方案及 Codex 审核清单

**状态：仅供设计审核（Review Only），未改仓库；G1 已更新为双移线主验收**  
**日期：2026-11-15**  
**仓库：** `WangxQiang21/CopTrans_base`  
**建议基线：** `snapshot/formation-v02-fr06-20261115`  
**本轮远端比对得到的基线提交：** `567bdecba9676313da3bc9a97b82f57b5b569a2b`（执行前重新检查）  
**基线与 `continuous-mode-path` 的关系：** FR06 快照从单车 V2 分支延伸，GitHub compare 显示增加了 19 个提交；不应从单车 V2 重新建设多车底座。  
**建议新分支：** `feature/virtual-rigid-formation-a`（审批后再创建）。  
**本轮任务：** 请 Codex 阅读实际源码进行严格审核；**不要创建分支、修改文件、执行重构或提交代码**。

## 0. 核心决议：修改旧计划的基线与复用策略

本方案替代《Codex_Review_Plan_Virtual_Formation_A_From_V2.md》的**分支基线、模块复用、工作量与验收路径**；数学公式和已讨论的 A-only 研究边界仍然有效。

- **保留 FR06 已存在的三车多车骨架：** `formation.py`、`transport_contract.py`、`formation_collision.py`、`formation_playback.py`、`formation_request.py`、`formation_quality.py`、`formation_retiming.py` 和现有 ROS/RViz 节点、消息、配置、验收证据。先逐函数核对复用边界，禁止从单车 V2 重建平行模块。
- **新增连续 A 模式几何/优化后端：** 以虚拟集群参考路径的曲率 `kappa_F(s)`、曲率导数 `eta_F(s)` 为规划量，在同一参考路径上解析恢复 3 辆车中心位置、各车航向与曲率，保持现有编队刚性硬约束。
- **不沿用 FR06 现有固定 pivot 原语作为新 A 的唯一路径空间：** `uniform_motion.py` 中 A 直线/固定旋转中心圆弧以及 `uniform_search.py` 的有限候选库保留作基线/对照，但新 A 后端必须允许无须 S 对齐的渐进曲率变化。
- **不沿用现有逐 edge/逐 block rest-to-rest 作为完整连续 A 的默认时间律：** `uniform_timing.py`、`formation_retiming.py` 有可复用公共动态审核概念，但其源文件注明的分段停车逻辑必须经适配；同一 A 运动内部曲率控制结点不必停车。
- **保留 FR06 已有的三车足迹、真实航向、公共时钟和独立审核语义：** 不要把单车 V2 的 `TimedTrajectory`、`quality.py` 的单车认证直接冒充三车证书。
- **首版范围：** 三车相同 A 模式、正向运动，集群参考点沿编队局部前向。O/S/倒车先不实施；统一模式切换语义和独立初始航向仍保留接口兼容性。
- **不实施底层控制：** 不开发 CAN、轮角命令生成、车辆控制器或 Gazebo。规划层返回各车 `(x_i,y_i,theta_i,v_i,omega_i)`；底盘接受 `v_i,omega_i` 的接口留给未来。理论前后阿克曼轮系限制可作为模型约束，但不代表已校准执行器。

## 1. 实际代码基线核验结果（Codex 必须逐项重新确认）

### 1.1 Git 版本与权威配置

- `snapshot/formation-v02-fr06-20261115`：三车 FR01–FR06 快照；GitHub compare 与 `continuous-mode-path` 的共同祖先正是单车 V2 提交 `956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee`；快照位于其后 19 个提交。
- `catkin_ws/src/fwis_v3_demo/config/transport_formation.yaml`：编队 ID、车辆 IDs、固定偏移 `[(1,0),(0,1),(0,-1)] m`、车体 `0.738 x 0.500 m`、初始航向策略、终点航向自由策略。此文件为**唯一权威编队合同**。
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation.py`：`FormationModel`、`FormationPose`、`VehiclePose` 与刚性中心映射；车体航向独立于编队姿态。
- `.../transport_contract.py`：编队配置合同解析及初始/连续目标航向语义。
- `.../formation_request.py` 和 `catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py`：ROS 请求、真实初始/连续目标衔接，不得隐式覆写实际车辆航向。

### 1.2 FR06 已经存在、应优先复用的能力

| 文件/入口 | 已有角色 | 本方案处理原则 |
|---|---|---|
| `formation.py` | 三车刚性中心几何、独立航向 | 直接复用，禁止平行复制另一套编队合同 |
| `transport_contract.py` + YAML | 配置、IDs、车长宽、航向策略 | 复用与扩展，不硬编码偏移 |
| `formation_collision.py` | 三车 OBB、成对 SAT 与连续扫掠 | 复用 `FormationCollisionChecker` 思路，新增曲率轨迹的保守速度/角度上界适配 |
| `formation_playback.py` / ROS `formation_playback.py` | 三车共同时间轴同步显示 | 复用，需检查新轨迹格式兼容性 |
| `formation_quality.py` / `uniform_quality.py` | 多车独立质量/最终审核 | 新路径必须独立审查；优先做认证接口适配，不能借用旧边型认证 |
| `uniform_pipeline.py` | 请求→搜索→时间→终审 | 保留旧 FR04 基线，为新 A 后端建立并行入口 |
| `uniform_motion.py` | FR01 A 直线和固定 pivot 弧、O/S 几何 | **不要**把这些原语当成新连续 A 的唯一空间；作为旧版对照 |
| `uniform_search.py` | FR03 有限标签搜索、离散 A/O/S 继承 | 首版不直接扩展搜索状态；未来只借用全局引导思想 |
| `uniform_timing.py` / `uniform_trajectory.py` | FR02 公共分段时间与权威解析轨迹 | 复用时间约束、模型序列化与审计原则，不能保留每段零速默认 |
| `formation_geometry.py` / `formation_kinematics.py` | 旧多车 S5/S6 几何与模式分配 | 审核通用几何和导数可否复用；不让旧 A 弧域阻止变曲率 |
| `formation_retiming.py` | 多车公共进度、时间约束、分段停车 | 保留公共时间思路，新的同 A 路径实现非停车段连接 |
| 单车 V2 `continuous_geometry.py` | C1 Hermite 曲率及一二阶导数工具 | 可复用数学插值和积分组件，不可直接把单车姿态/可行性用于三车 |
| 单车 V2 `continuous_optimizer.py` | 连续路径 NLP 技术 | 仅作为求解器、控制结点与线性化思路参考；主约束需以三车重定义 |
| `formation_uniform_planner_node.py` | 现有多车 ROS 入口 | 检查能否通过新增后端 selector 实现并行接入，不破坏 FR06 |

**已核对的 FR06 事实：** `evaluation/formation_refactor_v02/FR06/REPORT.md` 报告宽度 2.8m 的三个尝试成功、3.2m 的三个尝试达到预算、两者参考路线均通过审核；`FR06/CODEX_ACCEPTANCE.md` 记录 2026-11-14 Codex 验收通过。这是仓库保存的历史证据，**不是本轮重新运行得到的结果**。需要 Codex 对本地当前 HEAD 重新验证。

## 2. 数学模型：集群而不是三个单车规划器

### 2.1 模型假设及边界

- 取货物/编队参考点 `F`，局部前向 `+x`，局部左侧 `+y`；各车中心固定偏移 `r_i=(a_i,b_i)`。
- 保持车体航向自由的**中心刚性编队**，不要求三车车头始终相同。
- 首版 A-only 子模型选择参考点切向与编队姿态一致，仅正向行驶：

\[
 x_F'=\cos\psi_F,\quad y_F'=\sin\psi_F,\quad
 \psi_F'=\kappa_F,\quad \kappa_F'=\eta_F,\quad\eta_F'=u_F,
\]

其中 `'` 是关于参考点路径弧长 `s` 求导。状态 `z_F=(x_F,y_F,psi_F,kappa_F,eta_F)`，空间控制 `u_F=kappa_F''`。不宣称覆盖所有可能的三车 A 模式运动。

### 2.2 集群→车辆解析映射

\[
 p_i=p_F+R(\psi_F)r_i,\quad
 D_i=(1-b_i\kappa_F)^2+(a_i\kappa_F)^2,
\]

\[
 p_i'=R(\psi_F)
 \begin{bmatrix}1-b_i\kappa_F\\a_i\kappa_F\end{bmatrix},
 \qquad
 \theta_i=\psi_F+\operatorname{atan2}(a_i\kappa_F,1-b_i\kappa_F),
\]

\[
 \kappa_i=\frac{\kappa_FD_i+a_i\eta_F}{D_i^{3/2}},
 \quad v_i=\dot s\sqrt{D_i},\quad
 \omega_i=\left(\kappa_F+\frac{a_i\eta_F}{D_i}\right)\dot s.
\]

符号、尺寸和单位必须单测；要求 `D_i>0`，不要在速度零/奇异时盲算中心曲率。起终真实车体航向/零轮角必须校验，不能“覆盖”请求。

当前 \(\ell=1\,\mathrm m\)，偏移 `r1=(ell,0), r2=(0,ell), r3=(0,-ell)`，正常分支 `|ell*kappa_F|<1` 时：

\[
 \kappa_1=\frac{\kappa_F(1+(\ell\kappa_F)^2)+\ell\eta_F}{(1+(\ell\kappa_F)^2)^{3/2}},\quad
 \kappa_2=\frac{\kappa_F}{1-\ell\kappa_F},\quad
 \kappa_3=\frac{\kappa_F}{1+\ell\kappa_F}.
\]

\[
 v_1=\dot s\sqrt{1+(\ell\kappa_F)^2},\quad
 v_2=\dot s(1-\ell\kappa_F),\quad
 v_3=\dot s(1+\ell\kappa_F),
\]

\[
 \omega_1=\left(\kappa_F+\frac{\ell\eta_F}{1+(\ell\kappa_F)^2}\right)\dot s,\quad
 \omega_2=\omega_3=\kappa_F\dot s.
\]

### 2.3 集群可行域

暂以 `r_min_ack=0.810 m` 作**单车中心最小半径工作假设**，必须注明该半径的厂商参考点尚未精确校准。

\[
 |\kappa_i(\kappa_F,\eta_F)|\le \kappa_{\max}=1/0.810,
\]

对该构型正常左/右弯的两侧车，可得 `|kappa_F| <= 1/(1+0.810) ≈ 0.5525 1/m`；车1还给出有关 `eta_F` 的允许区间：

\[
 \frac{-\kappa_{\max}D_1^{3/2}-\kappa_FD_1}{\ell}
 \le \eta_F \le
 \frac{\kappa_{\max}D_1^{3/2}-\kappa_FD_1}{\ell},
 \quad D_1=1+(\ell\kappa_F)^2.
\]

**内侧车决定固定半径的限制，不等于充分验证连续变曲率弯道。** 在路径产生阶段应严格审核三车的 `kappa_i`；若在数学模型层面需要理想前后轮几何轮角限幅，可增加诊断性可行性检查，但不实现控制指令映射，且不得把厂商 CAN ±40°直接当单轮物理角限值。

若起点三车车头全部与编队 `psi_F` 平行、且轮系对应直行，则首版设置 `kappa_F(0)=eta_F(0)=0`；终点是否置零按**用户真实目标语义**决定：有明确三车平行/轮系回正要求则置零；若仅货物位姿受限且目标车头自由，可优先使用首版明确的停车/回正目标策略并标注其额外约束，不能混淆“目标车头自由”与“模型自动强制为0”。

## 3. 规划问题和解算器

### 3.1 G1 数学验证——以“双移线”作为主验收（没有地图/ROS）

**用户确认的 G1 主任务：** 用纯解析参考曲率使三车刚性集群完成一次标准双移线：**从原直道向左横移到相邻通道，再向右返回原通道**。它至少经历“左转→右转→右转→左转”的曲率符号顺序，即 `+ → − | − → +`，且全程为正向 A 模式。须增加完全镜像的“先右再左”案例，避免只测试左转符号分支。这里的“移线”是**通过前进和转弯得到的横向位移**，绝不是使用 O 模式横移。

**G1 不引入优化器、地图、ROS、模式切换或完整时间规划。** 首先手工指定**解析曲率函数**，由已有/新增集群数学模块积分恢复虚拟参考轨迹，再解析导出三车中心、各车航向、实际中心曲率，验证正确性。此项只证明**模型有表达与审核能力**，不冒充 G2 的自主路径规划。

#### A. 建议的可复现双移线解析基准

令单次移线的参考弧长为 `L > 0`，目标曲率振幅为 `K > 0`（单位 m^-1），定义 `f(ξ)=sin^3(2πξ)`（ξ 无量纲）。将两次方向相反、等长度的 S 形移线连续拼接：

\[
\kappa_F(s)=
\begin{cases}
 K\sin^3(2\pi s/L),&0\le s\le L,\\
 -K\sin^3(2\pi (s-L)/L),&L\le s\le 2L.
\end{cases}
\]

其一阶曲率导数可以直接解析求出：

\[
\eta_F(s)=\frac{d\kappa_F}{ds}
=\pm\frac{6\pi K}{L}\sin^2(2\pi\xi)\cos(2\pi\xi),
\]

其中 `±` 与各段 `K` 的正负号对应，`ξ` 为该段局部弧长归一化值。对应二阶导数 `u_F=κ_F''` 也可以解析给出或通过符号/自动微分获得。函数在起点、移线连接点和终点均满足 `κ_F=η_F=u_F=0`，因此至少具有本测试所需的连续接续性质。**不可把 `sin^3` 解析曲率误写为底盘角度控制指令。**

由于每次 S 形移线的净航向变化 `∫ κ_F ds=0`，两次形状相同且方向镜像，理想积分时满足：

- 起点 `ψ_F(0)=0`、三车 `θ_i(0)=0`、所有实际中心曲率为零；
- 首次移线后 `ψ_F(L)=0`，获得**非零左向侧移**，三车再次平行、曲率与曲率导数为零；
- 第二次镜像移线后 `ψ_F(2L)=0`、`y_F(2L)=y_F(0)`，集群回到原通道，三车再次平行、曲率与曲率导数为零；
- 四个弯曲子阶段包含左右两种曲率符号，各车包括内外侧身份交换，因而可以交叉验证曲率映射、速度比例及角速度正负号。

**建议基准参数（工作性测试，不是硬件标定）：** `K=0.30 m^-1`、`L=6.0 m`；起点 `q_F=(0,0,0)`。以高密度数值积分得到单次左向移线约 `1.126 m`、首段前进约 `5.822 m`；完成两段后总前进约 `11.645 m`、最终横向位移约 `0 m`、航向回到 `0`。在这一基准下 `max |η_F|≈0.363 m^-2`，三车中心曲率峰值约：`max |κ_1|≈0.526 m^-1`，`max |κ_2|≈max |κ_3|≈0.429 m^-1`，均低于暂定工作性上限 `1/0.810≈1.235 m^-1`。这些数值应由独立脚本重新计算，不要作为被测算法的硬编码真值，也**不能据此直接认定真实底盘执行器已校准可行**。

#### B. G1 具体实施与硬验收

1. 新增纯数学的“双移线参考曲率构造器”，支持 `K、L、首移方向`；同时保留直线与单次平滑圆弧过渡作为单元用例。
2. 数值积分 `x_F'=cos ψ_F, y_F'=sin ψ_F, ψ_F'=κ_F`，在参考轨迹上按固定偏移恢复三车位置与切线航向；分别用几何数值微分重新计算 `κ_i`，对照解析公式，不允许仅复用同一公式再自检。
3. 每个弧长位置审核 `D_i>0`、`|κ_i|≤κ_max`、理论 A 无侧滑切向相容性、刚性中心距离；记录曲率符号、最不利车辆及约束裕度。理想轮角仅用于规划模型约束诊断，不编写轮角控制映射。
4. 检查起点、中点 `s=L`、终点 `s=2L` 的 `κ_F、η_F` 和 `θ_i、κ_i` 连续性，特别检查 `κ_F` 过零、内外车互换和前方车受 `η_F` 影响的情况。
5. 同样运行 `K→-K` 镜像案例：横向位移反号、曲率/角速度相关符号对称、两次移线后同样回到原车道。对称性不能只靠绘图目测，应比较数值残差。
6. 可以使用任意固定正参考速度 `\dot s` 输出 `v_i、ω_i` **诊断曲线**，核实正负号、速度比例及同步公式；**G3 才开发正式公共时间规划器**。
7. 保存可复现的离线轨迹图（集群与三车中心、车体朝向）、`κ_F/η_F` 与三车 `κ_i` 曲线、约束与数值误差 JSON/CSV，以及固定随机种子（若使用）的测试命令。必要时给 G1 增加一个极端 `K/L` 反例，保证非法曲率变化确实被拒绝。

**G1 通关判据：** 基准与镜像双移线都由**一个连续集群曲率函数**产生三车轨迹，初末三车同向且原车道返回；中途左/右弯均通过中心曲率与刚性硬约束，数值微分残差在明确容差内，所有拐弯方向和前车受 `η_F` 影响的情况均得到测试证据。若车辆中心曲率或其他必要几何约束违规，则拒绝测试候选并报告具体位置、车辆与原因。正式自行求解 `K、L、终点` 的任务**仍属于 G2**，不能把 G1 的人工参数化曲线视为规划成功。

### 3.2 M2 空旷环境 A-only 自主规划

采用分段连续曲率表示，优先复用现有 `ControlKnot` 与 Hermite 插值和积分能力，评估是否要升级分段五次曲率函数。必须保证 `kappa_F`、`eta_F` 至少连续；若需要进一步控制导数跳变，保证 `kappa_F''` 连续。非凸优化变量可以是各段控制点的曲率/导数以及段长；通过集群方程积分恢复最终位姿。

\[
 \min\; w_LL+w_\eta\int_0^L\eta_F^2 ds+w_u\int_0^L u_F^2ds
\]

约束：起终货物位姿、真实初始航向、三车中心曲率可行、各段接续连续、段长正值；使用多初值，不允许把人工构造参考路径当求解器输出。优化返回 SUCCESS 必须经独立审核。**先不要把 FR06 `uniform_search.py` 的固定 pivot A successor 原样搬入新后端。**

### 3.3 M3 全路径公共时间参数化

只规划一条 `s(t)`，从同一个参考轨迹求所有车辆 `(p_i,theta_i,v_i,omega_i)`；确保同一连续 A 段的每个内部数值控制节点**不强制停车**。利用现有 `formation_retiming.py` 动态限值及诊断体系，但需要适配其每 block rest-to-rest 语义。可用 `b(s)=dot(s)^2` 来施加中心速度、加速度与已配置的角速度约束：

\[
 v_i=\sqrt{D_i}\dot s,\quad
 \dot v_i=\sqrt{D_i}\ddot s+\frac{D_i'}{2\sqrt{D_i}}\dot s^2,\quad
 \ddot s=\tfrac12 b'(s).
\]

不得把旧逐原语 zero-speed 当作 A 内部节点的真实停车。当前不进行 CAN 或轮角控制输出。

### 3.4 M4 狭窄环境自主规划与碰撞

在 M2 成立后，使用地图搜索作为**路线/拓扑初值**，而不是直接采用二维折线作为可执行轨迹；优先复用 FR06 真实 OBB 检查和保守扫掠审核。三车 OBB 旋转方向必须使用每辆车的 `theta_i`，而不是全车统一的 `psi_F`。外接圆只作为快速安全充分条件，不能唯一硬膨胀排除宽度约 2.8m 的可行通道。

重视 FR06 报告披露的热点：3.2m 宽道多次预算耗尽，碰撞占大部分时间。优化内环采用廉价预检查和距离场；末端必须独立、可保守证明的三车扫掠审核。无法证明无碰撞的候选不可发布 `FEASIBLE`。保留 FR06 2.8m/3.2m 场景作为**参考比较**，但旧有 A/O/S 混合模式参考解**不等于**新的 A-only 解必然存在；要另外准备已知 A-only 可行参考集。

### 3.5 M5 ROS/RViz 集成

新增后端 selector/ROS 适配层，保留既有多车请求解析、统一车辆 ID、货物/车体航向语义与 `formation_playback.py` 的公共时间轴。旧 `uniform_v02` 原有启动路径必须保持可用。新发布必须带清晰的 `backend/model_version/plan_id/request_hash/certification`，不能让旧回放器把新几何-only 候选当作通过终审的执行轨迹。失败清除旧可执行轨迹、保留起点静止模型与失败原因。

## 4. 推荐的新增模块边界（与 FR06 并行而不是替换整个项目）

建议在现有包 `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/` 下新增：

```text
formation_continuous_a/
  __init__.py
  state.py                 # 集群路径状态与真实请求/目标边界合同
  kinematics.py            # 三车偏移曲线、曲率、航向、速度映射
  curvature_profile.py     # 曲率 Hermite/五次段、弧长积分
  feasibility.py           # 三车中心曲率与可选理想轮角约束
  seed.py                  # 无障碍多初值
  optimizer.py             # 受限连续曲率 NLP
  common_timing.py         # 全 A 连续非零速时间律
  trajectory.py            # 权威解析路径与共同时间、版本化模型
  audit.py                 # 与现有 S9 兼容但独立的三车终审适配
```

说明：这是**建议模块边界**，并非强制每个文件都新建。Codex 必须先指出哪些能够通过现有函数直接复用；若 `formation_kinematics.py` 或 `formation_geometry.py` 已有合适通用函数，应在既有模块中复用/小范围扩展，避免第二套重复数据合同。现有 `formation_collision.py`、`formation_playback.py`、`transport_contract.py`、`formation_goal_bridge.py` 等优先不复制。

**禁止设计错误：** 不要仅把现有单车 V2 `ContinuousPlan` 解释成货物轨迹后，再在 RViz 中给三个车复制三份相同 `theta`。三车真实 `theta_i` 和 `kappa_i` 需要由偏移曲线解析恢复，且应进入碰撞/时间/终审全过程。

## 5. 里程碑、交付物与硬验收

| 阶段 | 交付物 | 首要验收/阻塞条件 |
|---|---|---|
| G0 基线审计 | 精确 HEAD、工作区状态、函数依赖图、FR06 测试清单 | 旧 FR06 可重放；不得无意改动历史功能 |
| G1 数学核心 | 纯 Python 三车映射 + 双移线解析基准与镜像验证 | 左→右→右→左的 A-only 双移线、原车道返回、数值微分、刚性距离、车1变曲率峰值及奇异点审核合格 |
| G2 空旷自主规划 | 非凸连续曲率求解器、多初值、独立几何审核 | 多组已知可行起终点无需人工喂轨迹，可满足终端及整段硬约束 |
| G3 公共速度 | 连续 A 专用时间律与动态审核 | 普通曲率控制结点无零速；速度/加速度/角速度受限，三车同步 |
| G4 地图避障 | 路线引导、三车 OBB 保守扫掠、新路径独立证书 | 已知 A-only 可行窄道通过；不误称预算失败为物理无解 |
| G5 ROS 集成 | 多车消息/适配、状态可视化、回归报告 | 起终请求一致、失败清旧结果、同一时间轴发布和回放 |

**持续必跑：** FR01–FR06 已有测试和回归、现有单车 V2 测试；每阶段保存 Git commit SHA、参数快照、测试命令、地图哈希、耗时和独立审核结果。禁止只凭 RViz 动画验收。

## 6. 推荐的安全建分支方法（必须经用户单独同意后执行）

```bash
cd /path/to/CopTrans_base
git status --short
git fetch origin
# 首先检查远端快照 SHA，确认仍是本次审查过的提交：
git rev-parse origin/snapshot/formation-v02-fr06-20261115
# 本轮观察到的提交：567bdecba9676313da3bc9a97b82f57b5b569a2b
# 如工作区有未提交工作，应停止并确认，不要自动覆盖。
git switch --detach origin/snapshot/formation-v02-fr06-20261115
git switch -c feature/virtual-rigid-formation-a
```

如需保留当前工作区，优先使用 `git worktree add -b ...` 从远端快照建立独立目录。不得把 `continuous-mode-path` 当基线，不得在快照分支提交算法改动，也不得无评审整体合并 `multimodal-hybrid-astar`。

## 7. 交给 Codex 的审核问题及期待输出

请按优先级回答：

1. **阻塞性数学问题：** 本文 `kappa_i(kappa_F,eta_F)`、`theta_i`、`v_i/omega_i` 在现有 `FormationModel` 坐标/符号规范下是否正确；特殊点 `D_i=0`、零速、起终车头/轮角、单位是否覆盖？需自行做数值微分对照。
2. **实际复用边界：** 按函数名、行号逐个核对 `formation.py`、`formation_geometry.py`、`formation_kinematics.py`、`formation_collision.py`、`formation_retiming.py`、`uniform_quality.py`、`formation_playback.py` 与单车 V2 `continuous_geometry.py`。明确“直接复用/适配/新建/禁用”。
3. **连续性等级：** C1 Hermite 的曲率连续及其导数在节点处的情况；是否需要切换五次，还是在新 A 模块中施加足够的节点约束即可？给最小实现方案，不为形式堆叠多项式阶次。
4. **时间规划语义：** `formation_retiming.py` 和 `uniform_timing.py` 中哪些 rest-to-rest 语义阻碍同 A 连续转弯？怎样最小改动实现 `s(t)` 而不损坏旧 FR06？
5. **碰撞审核成本：** 如何接入 `formation_collision.py` 的可证明保守扫掠，怎样设计优化内环快速筛选？证明 2.8m/3.2m 场景不会因集群外接圆硬膨胀造成错误否决。
6. **系统集成：** 对现有 `formation_goal_bridge.py`、`formation_uniform_planner_node.py`、播放与消息接口的兼容方案。如何在新模型中表达 `goal_heading_free=true` 与实际初始车体航向？
7. **G1 双移线核验：** 检查建议的 `sin^3` 双移线曲率构造是否满足双移线起终航向、原车道返回、`κ/η/u` 过零连续、三车中心曲率工作约束与左右镜像；核对数值示例并给出独立微分对照，不得将此归类为 G2 自主求解。
8. **可复现验收：** FR06 的代码和历史证据能否在本地按冻结参数重跑？输出当前测试命令、运行结果、失败归因和耗时，不能把历史报告写成新运行。
9. **代码/研究成本：** 首版最小改动量、最容易冲突的模块、可独立合入的 PR 阶段和预计新增测试类别。

**期望报告格式：** A. 审查结论（通过/有条件通过/不通过）；B. P0/P1/P2 问题及证据；C. 文件与函数级复用矩阵；D. 数学公式核验；E. 最小实现顺序；F. 回归测试与未知前提；G. 建议修改后的执行方案。**本轮只有审查权限，不创建分支、不写代码。**

## 8. 文献方法定位（不要误报新颖性）

- Fagninou 等，2026 RA-L，*Differential-Flatness-Based Path Planning for Semi-Rigid Formations of Ackermann-Steered Vehicles With Obstacle Avoidance*，DOI `10.1109/LRA.2026.3742267`：编队偏移曲率映射与几何路径优化，是最直接的相关工作；本方案须明确区别其对象、约束及验证。
- Fraichard & Scheuer，2004 TRO，*From Reeds and Shepp's to Continuous-Curvature Paths*，DOI `10.1109/TRO.2004.833789`：连续曲率规划基础。
- Banzhaf 等，2018 IV，*From G2 to G3 Continuity: Continuous Curvature Rate Steering Functions for Sampling-Based Nonholonomic Motion Planning*，DOI `10.1109/IVS.2018.8500653`：曲率变化率连续连接思路。
- 文献仅提供思路与基准；不要声称这些方法已直接解决当前三车 Ranger 前后阿克曼 + FR06 代码约束的全部规划问题。
