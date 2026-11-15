# 《4WIS 刚性编队规划重构方案 V0.1》技术审核

审核日期：2026-11-12。仓库：`expv3_multi`，分支 `feat/formation-eval`，HEAD `6227833`，含未提交 T5 改动。

审查稿：[下载目录中的 V0.1](</home/wq/下载/4WIS_刚性编队规划重构方案_V0.1_Codex审核稿.md>)。本报告对照当前源码、消息、配置及现有实验记录，不以历史 AI 完成声明代替证据。

**范围：最终交付仅为 RViz 交互输入、规划轨迹生成与三车同步回放。** 本轮没有修改代码、测试或配置，没有运行规划实验，没有继续 T5/T6，没有提交 Git；下文是架构审查及能力层面的可行路线，不是正式实施任务单。

标记：**【推导】**为本报告数学结论；**【代码】**为当前源码可核实；**【记录】**为已有实验产物支持；**【待验证】**为尚无实现或实测证据的判断。

## 1. 审核结论

**总体结论：方向有条件成立，当前仓库需要有边界的核心改造，不需要重写整个 ROS/RViz 系统。**

1. 【推导】刚性中心几何、车体独立航向、全车统一 A/O/S、公共停稳切换可以同时成立。这里的“独立航向”是允许不同，不是任意选择后都能满足当前模式；A 中航向受中心速度切线约束。
2. 【推导】统一 A 并不阻止协同转弯。固定瞬时旋转中心的刚体圆弧可给出解析可行解；必须检查每车半径/曲率，不能只检查货物参考轨迹。
3. 【推导】统一 O 在轮角允许随时间变化时，确实可能使编队转动。首版仅提供整体平移是合理的受限原语选择，不能写成 O 运动学的必然限制。
4. 【代码】现有 BODY_SPIN 是“旋转车 S、未旋转车 A 保持”，且验证器拒绝零角度 S；它不符合新稿的统一 S，不能直接复用为已合规实现。
5. 【代码】现有前端节点只按货物位姿去重，现有种子只携带几何动作，不保存搜索选定的模式/各车航向边界。只在前端加几次运动学判断而保留原去重和交接合同，不能实现方案目标。
6. 【代码】公共时钟、逐车 OBB、扫掠、请求/目标、TimedTrajectory、Marker/TF 基础均可复用；需要统一新模式合同、搜索边表示和实际回放插值。NLP 可暂缓，不是首版 RViz 功能的必要前置条件。

**不建议直接把 V0.1 的完整语义状态 `(q_F,theta_1,theta_2,theta_3,m_F,...)` 全部离散成高分辨率网格，也不建议先开发一般 O 转弯、连续变曲率联合优化或在线抢占。** 先确定一组可解析核验的原语、有限航向标签和无损交接表示，再评估是否需要扩展。

## 2. 数学模型审查

### 2.1 刚性几何与独立航向：定义正确，但不能把“独立”误解为不受约束

【推导】固定货物坐标偏移：

\[
p_i=p_F+R(\psi)r_i,\qquad
\dot p_i=\dot p_F+\omega R(\psi)Jr_i,\quad \omega=\dot\psi.
\]

它严格保持完整有向构型和两两距离。`theta_i` 不必等于 `psi`，也不必彼此相同。当前三车到货物均为 1 m，三条边长为 `√2,√2,2 m`；货物并非车辆质心。

定义货物坐标系中的速度 `u=R(-psi) dot(p_F)`，则每车速度在该坐标系中为：

\[
w_i=u+\omega Jr_i,\qquad \dot p_i=R(\psi)w_i.
\]

【推导】在 A 模式且 `w_i≠0` 时，选择该车正反向符号 `sigma_i∈{+1,-1}`：

\[
v_i=\sigma_i\|w_i\|,\qquad
\theta_i=\psi+\operatorname{atan2}(w_{iy},w_{ix})
+\begin{cases}0,&\sigma_i=+1\\\pi,&\sigma_i=-1\end{cases}
\pmod{2\pi}.
\]

这说明 A 中的车体朝向由刚体速度场与方向分支决定。统一 A 允许不同车速、不同朝向、不同曲率，亦不自动要求所有车辆同为前进或同为倒车；是否再限制方向一致，应单独定义。当前 `allow_reverse=true` 没有要求全车方向一致。

【推导】对一般光滑运动，且方向分支不在非零速度时跳变，有：

\[
\dot\theta_i=\omega+
\frac{w_{ix}\dot w_{iy}-w_{iy}\dot w_{ix}}{\|w_i\|^2},
\quad \dot w_i=\dot u+\dot\omega Jr_i,\qquad
\kappa_i=\frac{\dot\theta_i}{\sigma_i\|w_i\|}.
\]

因此一般 A 可行性涉及速度场的变化，不是仅检查瞬时 `q_F`、货物曲率或三车中心位置即可确定。初版使用固定几何参数、只改变公共时间进度的圆弧可避免立即求解此一般问题。

【代码】当前 `formation_kinematics._rotation_choices()` 已按实际中心导数判方向、按 `theta'/v` 审核曲率；这部分思想可保留。它当前针对固定枢轴旋转块，不应直接宣传为任意曲线的判定器。

### 2.2 统一 A 的解析原语与逐车半径条件

【推导】取固定的货物坐标枢轴 `c`，令一段运动的刚体几何满足：

\[
u=-\omega Jc,\quad w_i=\omega J(r_i-c),\quad
\rho_i=\|r_i-c\|.
\]

枢轴世界位置 `C=p_F+R(psi)c` 固定，货物和各车绕同一个世界点转动；这包含货物参考点边移动边转向的圆弧，不限于“货物原地转角”。

对 `rho_i>0`，车头切向/反切向、`theta_i` 随 `psi` 同增量变化，且：

\[
|v_i|=|\omega|\rho_i,\quad
\kappa_i=\frac{\operatorname{sign}(\omega)}{\sigma_i\rho_i},\quad
\rho_i\ge R_{min,i}.
\]

还要检查轮系角度/轮速限制；最小半径只是必要检查之一。当前默认 A 最小半径为 0.810 m。

**明确反例：【推导】** 取正确构型和 `c=(0,1.5)`。货物转弯半径 1.5 m 大于 0.810 m，但 car2 半径只有 0.5 m，需要 `|kappa_2|=2 m⁻¹`，超出当前上限约 `1.2346 m⁻¹`。该统一 A 原语不可行，不能通过放慢时间修复几何曲率。

**明确可行例：【推导】** `c=(0,0)` 时三车半径均为 1 m，正转时正向切向航向为 `(psi+π/2,psi+π,psi)`，`|kappa_i|=1 m⁻¹`。符合当前半径上限，前提是起始航向接续、轮系与扫掠也合法。

**真正的协调行进转弯例：【推导】** `c=(0,2)` 时货物走半径 2 m 的圆弧，三车半径为 `(√5,1,3) m`；逆时针正向初始相对航向可取 `(atan2(1,2),0,0)`，速度比 `√5:1:3`，曲率分别 `1/√5,1,1/3 m⁻¹`。这证明统一 A 可以实现参考点移动的协调转弯，不必增加新运动模式。

【代码】`FormationPrimitive.ROTATION` 和 `FormationPathControl` 已有显式 `pivot_x/pivot_y`，几何积分可表达这些例子；但 C++ `generate_primitives()` 当前主要只发 `c=0`，`formation_spin.rotation_schemes()` 也按 `r_i` 而非 `r_i-c` 求切向。**几何表示支持非零枢轴，不等于候选生成、航向重配置、审核全链已正确支持。**

### 2.3 A 的零速与枢轴车辆：应明确处理，不可套用除零公式

【推导】A 应同时满足：
\[
\dot p_i=v_i e(\theta_i),\qquad
|\dot\theta_i|\le |v_i|/R_{min,i}.
\]
当 `v_i=0` 时不能凭 A 转动车身。共同停稳边界的切向应由即将开始的路径单侧极限判断；不能因瞬间速度为零就把任意起始车头视为已经切向，也不能直接改写车体初值。

【推导】若某个旋转枢轴恰好等于一辆车中心，该车在一整段内可以 **A 零速保持且车头固定**，其他车沿合法 A 弧绕它运动；并非必然需要该车进入 S。此时该车的 `theta_i` 不随 `psi` 增加。反之，若要求它同时跟随 `psi` 转动车身，就违反 A。

【代码冲突】当前 `_rotation_choices()` 将零半径车辆分配为 S，旋转块又倾向令每车航向增加同一个 `dpsi`，不能直接拿来表示上面的统一 A 退化情况。

**建议：【技术判断】** 最小原语库可明确排除“枢轴落在任一车中心及其附近”的特殊分支，仅保留所有 `rho_i≥0.810` 的候选；这是降低实现成本的原语域限制，不应写成数学上所有零半径 A 都不可行。是否支持驻车枢轴车辆留作方案决策。

### 2.4 O 模式可以转动编队，当前实现却不能直接承接

【推导】O 的方程为 `dot(theta_i)=0`、`dot(p_i)=v_i e(theta_i+phi_i)`。若 `phi_i(t)` 可以变化且满足轮角/速率限制，并不推出 `omega=0`。

一个局部反例足以说明：货物固定位置绕原点转 `psi:0→π/4`，三车半径 1 m；车体航向固定为 `(π/2,π,0)`，令三车 `phi_i(t)=psi(t)`、`v_i(t)=dot(psi(t))`，则正转中每车中心速度正好为 `dot(psi)R(psi)Jr_i`，所有车同为 O，车体没有自转。

该例需限制 `phi` 范围、轮角变化速率/加速度、中心加速度等，并在两端停稳。不能将它推广为 O 可不受限制完成任意多圈旋转；跨越轮角边界的 `phi±π` 重编码和速度符号变化也不能伪装成无时间代价的实际轮角跳变。

【代码】当前 `VehicleModeSegment.control`、O 平移上下文和 `wheel_configuration(Action(...))` 按块使用固定斜移角；时间层通常把块内转向速率视为零。它不能直接审核上例的 `phi_i(t)`。若增加一般 O 转动，需要新控制函数及轮角动态计算，不是改一下模式标签。

**建议：【技术判断】** V0.1 首版仅实现 O 整体平移非常合理；应写成已选受限能力集，保留一般 O 推导，不必为 RViz 首版立即实现它。

### 2.5 S 与统一停稳切换：自洽，但代码和标签合同必须调整

【推导】所有中心固定时，对任意两个不同偏移：
\[
0=\dot p_i-\dot p_j=\omega R(\psi)J(r_i-r_j)
\Rightarrow \omega=0\Rightarrow\dot p_F=0.
\]
所以 S 中货物位姿固定正确；各车可独立改变航向，也可为零角速度。非退化三车构型完全满足该推导条件。

【代码冲突】`formation_spin.make_spin_block()` 把未转动车设成 A；`audit_mode_assignment()` 的 BODY_SPIN 分支对 S 的零角变化返回 `ZERO_BODY_SPIN`。新方案下应允许“**S 模式内静止保持**”；不能只把显示标签改成 S 而保留构造、轮系、时长和审核的旧逻辑。不转动车也应具有 S 的轮系设置状态，可能增加进入/退出驻留需求。

【推导】共同 S 运动可用 `theta_i=theta_i0+Δtheta_i h(t/T)`，其中各自 `Δtheta_i` 可以为零，T 由最紧车辆约束决定。停车切换段则要求货物/各车中心/各车车体角全部固定，仅转向轮系重配置；两者不能合并为一个“停几秒即完成车体旋转”的事件。

V0.1 还应补清两点：

- **模式在切换驻留区间如何标记。** 可采用统一的旧/新模式标签加独立 `mode_switch` 相位和 `from/to`，定义半开区间与边界值；轮系过渡过程中不应宣称始终处在某个纯模式的规范轮角。无需为了静止阶段引入第四种运动能力。
- **初始模式与轮角从何而来。** 起点车体角已有确定策略，但首段轮系设置不能在时间和审核中凭空省略。可以规定在起点静止完成初始设置或指定初始模式/规范轮角；作为回放合同记录即可，不涉及底盘控制。

【代码】S8 每块起终速度/角速度为零的五次进度、公共驻留和 `mode_switch_timing` 已有基础；S9 `_check_mode_schedule()` 当前主要按车检查驻留/边界，**没有等价于“全时域三车模式相等”的全局合同**。需要在块、采样与相位区间检查统一模式，不能只在几个时间点比标签。

### 2.6 停稳不等于不需要车轮状态，时间也不能修复一切

【技术判断】首版所有运动边都起停，可以避免把车辆速度作为完整搜索网格维度，但轮角设置时间仍依赖上条边的曲率/斜移角/模式与下一条边的设置。若精确计该代价，需携带上一原语/轮系配置标签；若不携带，须明确采用保守统一设置时间，不能仍声称代价精确。

【代码】当前 `formation_retiming._boundary_event()` 逐车比较 `wheel_configuration()` 的角差，并取共同所需驻留。此能力可复用；A→A 曲率变化、O→O 斜移角变化也可能需要停稳调整，不能只对 `m_F` 改变才计设置时间。

【技术判断】当前 A/O/S 在直行和静止时存在几何重叠，例如 A 的零曲率和 O 的零斜角；新模式标签是操作/约束状态，不能从轨迹外观唯一反推。所有层必须保留搜索所选模式，而不是后端按更低罚项重新分类。

## 3. 原语和搜索架构：合理部分与过度设计风险

### 3.1 最小原语是否足够

【推导】在无障碍空地、允许必要 S 重配置和足够停车空间时，O 平移＋S 调整车头＋A 绕货物原点旋转已能改变任意货物位置和方向。它具备基本自由空间位姿可达能力。

**但这不是狭窄环境的完备性证明。** 固定 `c=0` 的货物原地旋转可能要求过大的局部扫掠空间；通道内可行的非零枢轴弧可能更合适。不同固定枢轴 A 弧的端点车头未必接续，不能把任意两条合法圆弧直接拼接为合法 A 轨迹；可先停稳进入统一 S 调整，若该处没有 Spin 空间则仍不可行。一般连续变曲率 A 路径的能力大于这组有限圆弧。

【技术判断】首版可以保守，但必须至少验证一个**货物位置确实移动的 A 协调圆弧**，否则“阿克曼协同转弯”可能只展示了绕原地货物转圈。窄拐角应有同一受限原语/模式合同下的可审核参考解，否则无法区分原语不足和搜索不足。

### 3.2 状态压缩：可以不建六维细网格，但不能仅按 q_F 去重

【代码】当前 `formation_search.cpp::SearchKey/key_of` 只用 `(x,y,psi)`。相同货物位姿、不同车体航向可能具有不同 OBB 占用、Spin 可用空间和下一条 A 边；低 g 值并不自动支配另一种航向标签。模式不同、轮角设置不同也可能改变后继和成本。

【技术判断】较简单的可行表达是：货物位姿网格＋模式＋有限航向方案/上一原语标签，节点保存精确航向值；A 的航向从所选固定枢轴及方向分支生成，O 保留已有航向，S 只调整至有限后继候选所需的航向。S 的连续角度无须独立穷举三个高分辨率网格。

有限标签的去重仍需说明角度分箱、成本和状态等价规则；最终验证使用精确值。不得因两个节点在同一 q_F 网格就丢弃另一套合法车头。也不必首版引入未经证明的激进支配或高维启发式。

### 3.3 搜索边必须保留一套权威的连续几何、模式与航向

【代码冲突】`FormationPlanSeed` 只有货物起终点与 `FormationPrimitive[]`；后者仅含几何类型、dx/dy/dpsi/pivot，没有 `m_F`、各车边界航向、方向分支，也不能直接表达内部 BODY_SPIN。当前后端 `build_continuous_plans()` 会重新枚举模式与航向。

**所以不能仅在旧 C++ 前端检查统一模式后，仍发旧几何种子，让后端自由重分配。** 那样搜索碰撞姿态和最终轨迹可能不是同一条方案。

合理的两种交接方式：

| 方式 | 收益 | 代价/风险 |
|---|---|---|
| 保留 C++ 前端，扩展有明确版本的模式/航向边界种子合同 | 延续现有部署和 C++ 搜索性能基础 | C++/Python 均需理解同一原语与事件；消息生成、转换、审核要联动 |
| 在一个规划进程中生成权威的内部块序列，直接交已有 S8/S9 | 避免不必要的中间消息迁移与重复构造，较易复用 Python 旋转/Spin/审核代码 | 搜索运行效率待验证；必须有明确入口，避免同时启动两个规划发布者 |

【技术判断】这属于需要选择的接口架构，不是必须先重写全部 ROS。只为保留旧种子格式而增加旁路字典、隐藏状态或分散的补丁，通常比一次清晰的内部/消息合同更复杂。

### 3.4 搜索目标、优化与验证顺序

【技术判断】首版可用可解释的非负几何/重配置近似成本找可行解，最终用 S8/S9 检查并报告 T_end；若要声称时间更优，就应使搜索排序和真实 Spin/驻留成本尽量一致。不应将米、弧度、经验罚项直接称为秒。

【代码】现有局部 NLP `_block_parameters()` 只支持 translation/rotation，遇到 BODY_SPIN 可回退保留审核 seed；其目标主要是几何长度/转角与偏移罚项，没有完整障碍净空优化。它不能直接覆盖 V0.1 的全部统一模式序列。

【技术判断】把该 NLP 置于非首阶段阻塞项是正确的。可以先直接重定时和审核搜索结果；后续应依据具体质量问题决定扩展 Spin 边界、固定枢轴参数还是连续避障约束，而不是提前开发通用混合优化器。

航向一致性软项初期作为分析指标合理。对必需的切向差异施加强罚可能促使算法选择大半径绕行，甚至压制窄处唯一方案；应保持其优先级低于可行性，不能通过极大权重形成隐含同航向硬约束。

## 4. 当前模块复用和冲突映射

| 模块/接口 | 判断 | 必要边界 |
|---|---|---|
| `transport_formation.yaml`、`transport_contract.py` | 保留 | 正确偏移、尺寸和初始航向策略可复用；统一模式是新增合同，不能假装现已有 |
| `formation.py::FormationModel` | 保留 | 刚性中心派生适用；货物不是实体碰撞模型，V0.1 排除实体载荷建模可接受 |
| `formation_search.cpp`、`formation_search.hpp` | 核心改造或替代搜索入口 | 新边必须有运动学与真实车头，去重需保留后继相关标签 |
| C++ `formation_collision.hpp` | SAT/地图部分保留，姿态输入改造 | `rigid_bodies()` 固定相对车头假设与独立航向冲突；`goal_clear()` 不能用自由终端车头的占位角做唯一判定 |
| Python `FormationCollisionChecker` | 大部分保留 | 真实 q_F、各车 theta 输入与扫掠可复用；仍需逐边及最终时间采样验证 |
| `formation_kinematics` 的 A/O 微分、方向、曲率审核 | 条件复用 | 增加统一模式与零速边界；当前模式笛卡尔枚举可产生混合，不能只在截断候选后过滤 |
| `formation_spin.make_spin_block/rotation_schemes` | 改造 | 未转动车必须允许 S 保持；非零 pivot 的切向从 r_i-c 计算；角度选择保留连续语义 |
| `formation_geometry`、`FormationContinuousPlan/FormationBlock` | 多数内部表示可复用 | 后端必须保留搜索已选边，不再任意改模式；现有内部 BODY_SPIN 可扩成统一 S 事件 |
| `FormationPlanSeed/FormationPrimitive` | 现接口不足 | 需权威内部路径或扩展合同；不能把 S 假装成零角 ROTATION/未定义 COMPOSITE |
| `formation_retiming` 与 `mode_switch_timing` | 条件复用 | 固定曲率弧、固定斜移角、共同 Spin、分段起停可复用；补统一模式、零 Spin 车、初始化/切换相位；一般 O 转动不可直接套用 |
| `formation_quality::_IndependentEvaluator` | 框架保留，审核合同改造 | 加全区间模式一致性、S 零角保持、搜索边界一致、真实插值一致；不能仅信任构造器声明 |
| `FormationTimedTrajectory/FormationTimedPoint` | 大体足够 | 已含共同时间、q_F、每车 pose/mode/control，可表达统一模式；一致性须检验，未必需要增加重复 m_F 字段 |
| `formation_optimizer.py` | 首版旁路、后续按需扩展 | 暂不阻塞搜索→时间→审核→回放；不能把 fallback 成功算成 NLP 已支持新结构 |
| launch、run_demo、map_publisher、RViz 配置、TF/Marker | 保留 | 接入一个权威规划源；修正时间/插值和串行忙状态，不引入 Gazebo/控制链 |
| T5 工作树 | 保留记录、逐项决定是否采用 | 失败终态/锁可能复用；末端两顺序几何补丁不等于新的统一模式搜索，不宜作为其架构前提 |

【记录】T1–T4 有正确构型、平移、切向旋转、独立车体 Spin、同步时间和审核的支持；T4 原始 smoke 的 Spin 阶段仍属于旧混合模式合同。因此旧测试通过只能证明可复用基础，不能证明新“同一时刻三车模式完全相同”已经满足。

证据：[T3 两种 NLP 下基本任务](../evaluation/wedge_fix_2026-11-11/T3/T3_REPORT.md)、[T4 实现与测试范围](../evaluation/wedge_fix_2026-11-11/T4/T4_REPORT.md)。T4 正转有真实 ROS 支持，负转主要是内部管线测试；真实窄拐角、统一模式路径尚无本轮证据。

【记录】T5 四轮 smoke 顶层均失败。最新 `_callback_lock` 与测试中“新请求已 VALIDATED 后”的判定有调整，未见后续整轮通过；不能直接认定当前所有晚到消息仍然有错，也不能当成已解决。采用用户接受的单请求串行可显著缩小该问题范围。

## 5. RViz 最终功能：现有基础、缺口和最小路线

### 5.1 起终点和初始航向

【代码】`formation_goal_bridge.on_initial_pose/on_goal` 已分别接收 `/initialpose` 与 `/move_base_simple/goal`，输入对应货物参考点及 `psi_F`；配置生成三车偏移和初始航向。`initial_heading_policy=cargo_heading` 可直接作为首版确定策略，静态预览让用户看到初始三车朝向；起点也可用已有切向策略，但必须显示和记录。

不需要新增三套鼠标输入。初始全同向可以成立，但开始 A 协调弧前可能需要一段统一 S 对齐；不能在规划后悄悄把起点车头改成弧线切向。

【代码】现有终点 `goal_heading_free=true` 释放各车终端航向，仍要求货物终点位置和 `psi_F`；符合稿件目标。若以后要求终端车头整齐，应作为另一项约束/可选末端 S，不能混入“货物终点箭头”的语义。

### 5.2 轨迹与播放

【代码】现有 `FormationTimedTrajectory`、共享 header/time grid、每车 timed point、Marker/TF 及终端保持均可继续使用。`start_on_publication` 已将求解耗时与播放时钟分离；执行时长 T_end 继续取 timed.duration，求解耗时另计墙钟。

必要修正是 `formation_playback.frame_at()`：先按合同插值货物位姿，再计算 `p_F+R(psi_F)r_i`，各车 theta 单独连续插值；当前逐车中心线性插值给出圆弦而不是刚性圆弧。S9 `_state_at/_check_publication` 与显示端必须采用一致语义。

“同一重构公式”还不意味着恢复了任意连续控制律。若仍采用离散采样插值，须明确该插值就是显示轨迹并审核它的 OBB 扫掠；不能一边审核理想曲线、一边显示另一条曲线。跨 ±π/多圈场景必须规定展开角或明确角差策略；首版小角原语可限制范围以降低代价。

停车/切换位置及相位可通过当前 plan_id 对应的质量/轨迹元数据提供，无需先做复杂 RViz 面板。静态路径、三车车体、航向箭头、统一模式和状态文字已能承载核心观察目标。

### 5.3 单请求串行足够，但需要明确忙状态

【代码】bridge 当前没有完整的 BUSY 拒绝/排队协议，回调锁也不等于产品层串行准入。最小协议可从接受请求到播放结束占用一个 job；忙时明确拒绝新目标/起点重置或规定排队一种行为，结束/失败后释放。

这可避免首版同时处理取消、旧 TF、旧 latched 消息、新起点和规划抢占。仍需按 plan_id 核对状态/轨迹，失败清空旧可执行路径并保留静止车体；不需要为此实现在线重规划系统。

### 5.4 最小可行实施路线（能力依赖建议，非正式任务拆解）

**先固定一套小型权威原语合同。** 候选能力为统一 O 固定方向平移、统一 S 独立车体调整、统一 A 固定枢轴圆弧与直行/倒车，所有边起停。用现有半径/轮系/OBB/S8/S9 检查；统一 S 与搜索路径交接先解决，NLP 暂不进入关键链。

**再让一个模式感知搜索结果无损进入现有播放管线。** 搜索保留必要航向/模式标签，边的几何、车头、方向和模式保持不变地交给公共时间规划及独立审核；输出现有 TimedTrajectory。沿用现有 launch/目标桥接/Marker/TF，修正刚性插值并采用串行准入。

**最后以很小的能力集合确认交付边界。** 空地 A 前后行、O 横/斜移、含零角保持车的统一 S、非零枢轴 A 行进转弯、一个经参考解审核的窄拐角，以及一个明确拒绝目标；检查节点与区间内部的刚性、统一模式、停稳、OBB、耗时与 T_end。这里仅说明应该看见的能力，不生成本轮实施清单或要求执行上一轮所有 T0–T7。

【待验证】受限原语在具体窄拐角中的成功率、有限标签数量和搜索时间尚未知；应先用一个有参考可行解的目标判断，再增加原语或网格维度。上述路线不承诺已有代码只改配置即可完成，也不要求重写整个项目。

## 6. 仍需讨论的关键技术决策（最多五项）

### D1：统一 A 的首版原语域有多大？

- **A：** 货物原点旋转＋直线，只用已明确的半径 1 m 几何。复用最多，空地能力清晰；狭窄转弯表达保守。
- **B：** 再加入少量非零枢轴协调圆弧，排除枢轴落车中心的特殊分支。表达真正行进转弯，需同步扩展切向、扫掠和边界；仍比一般变曲率联合优化小。
- **审核建议：** 先确认 A 的解析合同，再把 B 中至少一类行进弧作为研究方案候选。需要用户/ChatGPT 判断：选定窄拐角是否必须依赖该弧，参考解是否属于所选原语族；不要用自由空间可达替代窄通行能力结论。

### D2：O 是否暂限整体平移？

- **A：** 固定斜移角的刚性平移。现有表示、轮系计算和时间层最易复用。
- **B：** 引入随时间变化的 phi，使 O 能执行某些刚体转动。可扩大可行集，但须改控制函数、转向速率/加速度和独立审核。
- **审核建议：** RViz 首版采用 A，B 留为后续比较。需要确认这是主动限制原语，而非论文中宣称 O 原理上不能转动编队。

### D3：S 的航向候选和搜索去重采用何种粒度？

- **A：** S 只连接有限后继运动原语要求的车头方案，节点携带方案/精确航向；去重保留模式和后继相关标签。
- **B：** 三车角度分别做网格或作为连续自由优化变量。覆盖更广，但组合规模、等价状态和求解稳定性代价更大。
- **审核建议：** 先 A；所有车辆全程 S 标签、零角保持和真实 Spin 扫掠必须完整。需判断有限航向库是否已覆盖首版场景，及是否允许因库不足返回受限搜索失败。

### D4：首版停车与轮系边界的合同是什么？

- **A：** 所有原语停稳，携带上一原语/规范轮系标签，按最大逐车轮角变化计公共驻留；明确初始设置及切换区间标签。
- **B：** 第一版用保守统一驻留近似，或进一步支持同模式不停驶接续。前者简化状态但代价不精确，后者需更多连续边界/转向动态状态。
- **审核建议：** 分段停稳与明确的公共相位足以首版交付；不应把几何、时间精确性与不停驶性能一次全部立项。需确认使用实际设置代价还是保守代价，以及相应结论范围。

### D5：模式感知搜索如何与现有后端交接？

- **A：** 保留 C++/Python 边界，扩展带模式和车头边界的种子消息。
- **B：** 在一个规划进程内直接构建权威 FormationContinuousPlan/事件，再复用 S8/S9 和既有 ROS 输出；旧几何前端保留为历史基线。
- **审核建议：** 以减少重复原语实现和无损交接为主要标准；B 值得优先评估工程成本，但 Python 搜索效率须实测，不能先宣称足够快。若选 A，消息扩展是必要边界改造，不能靠保留旧种子字段自动解决。两者都不要求改 RViz 用户输入或 TimedTrajectory 基本表示。

## 7. 审核依据与状态说明

主要源码：

- [formation_search.cpp](../catkin_ws/src/fwis_v3_planner/src/formation_search.cpp)：`SearchKey/key_of/generate_primitives/search_formation_geometry`。
- [formation_collision.hpp](../catkin_ws/src/fwis_v3_planner/include/fwis_v3_planner/formation_collision.hpp)：`rigid_bodies/goal_clear/sweep_clear`。
- [formation_kinematics.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py)：`_translation_choices/_rotation_choices/audit_mode_assignment`。
- [formation_spin.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_spin.py)：`make_spin_block/rotation_schemes`。
- [formation_geometry.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_geometry.py)：`FormationPathControl/FormationBlock/build_continuous_plans`。
- [formation_retiming.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_retiming.py)：`_context_for/_analyze_block/_boundary_event/retime_formation`。
- [retiming.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/retiming.py)：`wheel_configuration`；[mode_switch_timing.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/mode_switch_timing.py)：轮角调整与设置时间。
- [formation_quality.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_quality.py)：`_check_block_kinematics/_check_mode_schedule/_state_at/_check_publication`。
- [formation_optimizer.py](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_optimizer.py)：`_block_parameters/_build_nlp/optimize_continuous_plan`。
- [消息目录](../catkin_ws/src/fwis_v3_msgs/msg)：`FormationPlanSeed/FormationPrimitive/FormationTimedTrajectory/FormationTimedPoint`。
- [bridge](../catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py)、[回放核心](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py)、[回放 ROS 节点](../catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py)。

既有记录边界：[T4 真实入口记录](../evaluation/wedge_fix_2026-11-11/T4/smoke_nlp_off_r2.json) 为旧合同空地、NLP 关闭的成功；[T5 第四轮记录](../evaluation/wedge_fix_2026-11-11/T5/smoke_nlp_off_r4.json) 顶层失败。新统一模式原语、统一 S 和窄拐角效果本轮均未实验验证。历史完整 Python 回归有 `dynamic_peaks` 错误，不报告为全绿。

本报告的 O 转动、A 枢轴与曲率结论来自明确方程推导，不将其当成已有代码实验结果；也没有以笼统“Hybrid A*”“联合优化”名称代替具体状态、原语和合同。本轮未开展外部文献系统综述，不据现有材料宣称方法新颖性或对文献方法优越。

**审核在此停止。以上尚未转为正式实施任务或代码变更，待用户与 ChatGPT 确认模型和架构选择后再进行工程任务拆解。**
