# Chang et al. 2026 论文复现与实验计划

**论文**：Yuan Chang et al., “Generic motion planning for multi-locomotion autonomous vehicles: A spatio-temporal decoupling framework to accommodate multiple maneuvering modes,” *Information Sciences* 754 (2026), 123675. [DOI: 10.1016/j.ins.2026.123675](https://doi.org/10.1016/j.ins.2026.123675)。全文副本位于 [`papers/`](papers/) 。

**逐阶段执行细节**：见 [`CHANG_2026_EXECUTION_RUNBOOK.md`](CHANG_2026_EXECUTION_RUNBOOK.md)，其中列有入口条件、任务步骤、实验矩阵、交付和 go/no-go。

**定位**：该计划把论文方法结构复现与 Ranger Mini 适配分开。论文未公开源码、原始数据和全部算法参数；凡原文未给出的值均标注为复现假设，不能声称逐值复现。

> **当前实施范围（用户确认）**：现阶段只实现/验证几何路径与运动模式序列规划，假设底层控制完美跟踪；不实现轮级控制、执行器动态、速度/加速度调度、换模瞬态或闭环跟踪验证。这些工作留给后续控制阶段。论文忠实复现部分仍记录其完整时空方法，但当前 Ranger 规划实验不以控制层完成为前置条件，也不把几何可行性宣称为实体可执行性。

## 1. 研究问题与假设

论文关注多模式车辆在低速、狭窄环境中的规划：模式选择和路径几何相互影响；轮向/运动状态在换模期间会经历瞬态；若全局路径、离散模式、轮级控制和时序一起优化，问题过大。它将任务拆成空间层与时空层，但仍在空间层联合优化模式和几何。

需要验证的假设：

- H1：空间层联合优化模式序列与几何，比单模式规划和“先几何、后规则分配模式”更易行、路线更直接。
- H2：按模式分段做轨迹优化，比单个长时域问题更容易求解，且轨迹/控制更平滑。
- H3：切换点附近 ROI 瞬态优化，相较常转弯半径加 B 样条轮角插值，能扩展可行终点范围并降低横向速度变化代价。
- H4：STC 能有效限制优化空间，但不能取代高精度整车连续扫掠验证。

## 2. 论文方法复原

### 2.1 总体流水线

1. `CoarsePathSearch`：改进 A* 多模式扩展，生成空间引导路径和模式初值；终点条件放宽，启发权重 `κ>1`，此层优先速度，不保证最优。
2. `ConstructSafeCorridor`：沿粗路径生成 safe travel corridor (STC)，用覆盖车身的 feature-point discs 限制车辆外形。
3. `UpdateBounds`：由粗路径与信任范围 `β=[βx,βy,βφ]` 更新状态界和 Big-M bounds。
4. `PathModeOptimization`：空间层联合优化模式二元变量、行驶方向、连续状态与几何控制量，满足理想模式动力学、手册支持的高层模式范围、STC、起终约束；实体轮控制约束延期。
5. `PiecewiseTrajectoryGen`：按模式序列切段，每段固定模式优化时序轨迹；段边界状态由空间解给定，论文还设置段端控制为零。
6. `SwitchingPlan`：依据模式对和运动条件触发；在切换点两侧 ROI 优化中间瞬态。未触发/失败时退回 stop-and-switch。
7. `SynthesizeSegments`：拼接各段与切换轨迹。

**数学规划类型必须先核实**：论文称空间问题为 MIP，但其车辆运动学含 `sin/cos/tan`、控制量与状态乘积。实现前逐式确认变换后是否线性或 MIQP；若仍为一般非线性约束，实际是 MINLP，需选匹配求解器或用有误差界的分段线性近似。不要把论文术语当成求解器可直接接收的模型形式。

### 2.2 空间层模型

每个离散点恰选一种模式（one-hot `Σαᵢ(k)=1`）。模式状态按对应离散运动学传播；用 Big-M 激活/屏蔽模式约束。整数变量包括模式和方向，连续变量包括位姿和控制。主要约束：

- 起始 pose 固定，目标允许落在泊车可行区域和航向容差内。
- 模式专属理想运动学和手册支持的高层运动范围。
- perfect-tracking 假设；实体轮逆解和机械执行器限制留到控制阶段。
- 覆盖车身的 feature points 留在各自 STC。
- 各步唯一模式；可加模式聚类、最小段长、方向切换削减等剪枝。

论文通过 `vᵤ(k)ΔT'(k)=p(k)Δd` 把时间和速度从空间层消去，令空间步长固定。目标可写作：

`J₁ = η_length J_length + η_steer J_steer + η_switch J_switch + η_direction J_direction`

即路径长度、模式内转向平滑、模式切换次数、方向反转代价。**空间层不是直接最小化实际总行驶时间**；总时间在后续轨迹层形成。

### 2.3 分段时序轨迹

每个固定模式段独立求解：段内模式动力学、段首尾位姿/状态、段端控制、STC 和实体轮角限位。论文建议的 `J₂` 可组合总时间、加速度平方与转向输入变化平方。段内可增加节点以描述更多运动细节。

复现时要逐式确认：段端虚拟输入归零是否映射为实体轮角归零；相邻模式段的 wheel configuration 如何衔接；不同模式控制量不可直接作同一个 `δ` 的差值惩罚。

### 2.4 ROI 瞬态换模

论文提出两种中间运动模型：虚拟前/后轮单轨控制，或用侧偏角 `β` 与 ICR 辅助参数 `θ` 描述运动，避免平行移动时 ICR 无穷远的奇异问题。围绕切换点按前后轨迹弧长建立 ROI；其端点 pose 和输入与相邻段匹配。优化包括中间运动学、端点状态/控制、STC 和实体轮角限位。示例代价综合 yaw rate 平方、横向速度变化平方和换模时间。

触发逻辑不是所有模式通用。论文重点处理同向 ASM↔DMM；不符合条件则 stop-and-switch。ASM↔ZRM 可能因 ICR 从车体外转入车体内且轮角受机械限制，需要先停车重构。

### 2.5 论文公开实验

- 仿真：Python，Intel i5-11260H @ 2.60GHz；Gurobi 求空间 MIP，IPOPT 求 NLP。
- 仿真车：轴距 2.91m、轮距 1.675m、CoG 到前/后轴 1.25/1.66m、前/后悬各 0.5m。
- 全局案例：上方障碍物阻挡的平行泊车；起点附近再加非法停车障碍；窄通道垂直泊车。
- Table 6 proposed 总耗时分别 3.32/3.72/2.56s；Baseline 1 单模式 Hybrid A* + trajectory optimization 为 11.38/8.85/8.22s。作者机器结果只作计时分项/趋势复核，不作数值目标。
- 终端约束对照：无 heading 约束、`[-π/8,π/8]`、精确 `0`；论文称有界约束首次可行时间和 gap 降至 0.3 的时间综合较好。
- 换模对照：constant-radius + B-spline，切换时间 1.9s、固定速度 1m/s；另测试 0.5–1.5m/s 变化速度。平滑性示例权重 `η_yaw=1, η_lateral=5`。
- 实车原型另有轴距 0.89m、轮距 0.82m、前/后悬 0.165/0.29m，ROS + pure pursuit；两种场景测 ASM/DMM。因机械风险未实测 ZRM，场景使用 ZRM 属模型假设。
- 数据声明按请求提供；未找到公开代码仓库。

## 3. Ranger Mini 适配边界

当前工作区与论文的源码级差异、参数冲突和 solver 环境审计见 [`CHANG_MODEL_AUDIT.md`](CHANG_MODEL_AUDIT.md)。以下复现阶段的 Phase 0 执行时先按该审计的 P0 项清理。

Chang 仿真车与 Ranger Mini 尺寸相差显著。当前手册/K01 记录 Ranger Mini 轴距约 0.494m、轮距 0.364m、车身 0.738×0.500m；Chang 仿真车轴距 2.91m。不可直接比较路径长度、成功率或运行时。需要建立两份独立配置，先在论文尺度验证算法，再用 S09 验证本车适配。

Chang 的 ASM/DMM/ZRM 不与本项目 Ackermann/Diagonal/Lateral/Spin 一一对应。DMM（车身航向不变、速度方向可变）可能覆盖 Diagonal，Lateral 可能是其边界情况；Spin/ZRM 需确认四轮角和驱动实现。先从驱动/CAN 映射核实，不能按名称直接合并。

旧 K01 记录 `r_min_ack≈0.810m`、等效转角口径（约 ±40° 或用 0.494m 轴距反推约 ±31.4°）、Diagonal/Lateral 等效模型、保守 Spin 角速度；但手册未给完整内部轮角映射和伺服响应时间。因此先作为规划假设，不能宣称实体可行。历史状态有 S09、ROS 接口、CasADi 后端和整车 OBB 连续碰撞闸门；新优化没有通过该 validator 前保持 `lock_verified_geometry` 安全策略。

## 4. 两条实验轨道

### A. 论文结构复现

按论文三个场景拓扑复建环境；实现 coarse A*、STC、空间联合模式/路径优化、分段轨迹、ROI transient；复现三类比较方法与 terminal-heading / switching 对照。以结构、趋势、分项耗时和失败模式为复现目标，不追求原作者绝对时间。

### B. 本车适配验证

替换成 Ranger Mini 参数和核实过的轮级映射；使用 S09 七个 preset 和障碍/起点扰动。仅启用物理核实的模式；Spin 未核实时只算模型仿真。结论限定为仿真/硬件在环/实体试验中的实际等级，不把运动学闭环仿真写成真车验证。

## 5. 分阶段实施计划

### Phase 0：规格与模型冻结

1. 对照论文 PDF 图 2 和式 (1)–(7)、(11)–(20)、(33)–(49)，记录状态参考点、坐标方向、轮序、符号与变量维数。PDF 的文本抽取会破坏公式，公式以页面图像为准。
2. 建立各模式 `state/control → four wheel steering/speed` 映射及反解；覆盖正/负曲率、倒车、侧移、旋转和限位边缘。
3. 查驱动/CAN；若可读取舵角，低速记录目标角、反馈角、变化率与响应时间。没有硬件依据的参数明确标 `planning-equivalent`。
4. 单独维护 Chang 车辆与 Ranger 参数 YAML。
5. 将 trig、乘积、Big-M 分别登记，决定 MILP/MIQP 近似或 MINLP 求解器，并确认许可/部署约束。

**交付**：变量/公式字典、车辆参数配置、模式转换表、求解器建模说明。**门槛**：各单模式模型通过传播对照，实体轮逆解不超限，每对模式被标为 continuous-transient / stop-reconfigure / forbidden。

### Phase 1：运动学、footprint、STC 单元验证

- 独立实现单模式直线/转弯/侧移/旋转传播，与小步积分交叉核对。
- 随机采样位姿和控制，检查轮级角度、速度映射与倒车符号。
- 验证 STC 圆盘集合覆盖车辆真实矩形在多朝向旋转下的完整外形；对照精确 OBB 碰撞。
- 建立窄口、贴柱、障碍边原地转向、车位边界场景，测试连续扫掠碰撞。

**门槛**：最终 validator 零漏报；STC 可保守误报但记录保守率；未通过者不得进入优化层集成。

### Phase 2：固定模式序列后端

先固定合法模式序列，不开放 MIP 模式选择。粗路径生成 STC，对每段做连续优化和时序化。

诊断地图：空旷直达、单弯、窄口、平行车位、立柱侧绕、U 形障碍。每类固定 20 组 pose/地图种子。扫 STC 信任域 `β`、空间步长 `Δd`、corridor 松紧，记录可行率、求解时间、长度、最小净空、偏离粗路径量。

**门槛**：所有已接受轨迹通过 OBB 连续扫掠；小规模非退化样例可行率达到预设工程目标（建议 ≥95%）。失败先查模型/走廊，不能靠碰撞 slack 放过。

### Phase 3：空间模式/路径联合优化

实现步骤：

1. 先做 ASM+DMM 两模式小问题，再加入经核实的其他模式。
2. 建立 one-hot 模式、方向、各模式运动学、所有轮角限位、STC、初末状态/区域和目标项。
3. coarse A* 模式结果作 warm start；用局部几何推导紧 Big-M；比较无 warm start、松/紧界与剪枝设置。
4. 对 trig/bilinear 项做求解器可行性审计；若 PWA 近似，给出误差界并扫分段数量。
5. 加模式聚类/最短段约束、steer smoothness、切换和方向反转项；每次单独消融。
6. 求解失败需存 status、冲突/不可行解释、gap 和 incumbent，不允许静默输出未验粗路。

场景：重建论文 3 类全局案例；S09 七个 preset 加窄度、起点姿态、障碍位置扰动。先每场景 20 个固定样本，最后主场景扩到 50 次。Baselines：A) Ackermann-only Hybrid A*+同后端；B) 普通 A* + fuzzy/规则分配模式 + 同后端；C) 多模式粗搜但固定后处理模式；Ours) 空间联合优化。所有方法共用 footprint、安全余量、终端区域、优化容差和 validator。

**交付**：联合求解器、场景数据、运行日志、CSV、失败案例。联合方法不要求所有指标第一，但需证明模式由联合优化选出，且在某类窄场景有可重复收益。

### Phase 4：分段轨迹和切换 ROI

**4A 分段优化**：先无切换单段，再两段 stop-and-switch；对照整体长 NLP。报告边界连续、耗时、加速度和转向变化。

**4B 瞬态换模**：先实现无障碍同向 ASM↔DMM ROI，比较论文两种中间模型中的一种；另一种作独立核对。再加入障碍、方向改变。按论文触发条件执行；不触发时 stop-and-switch。

**4C 对照**：复建论文固定端点/轮角切换簇、自由终点及 point-to-point；与 constant-radius+B-spline 比较。先用论文设置 `T=1.9s`、`v=1m/s`、`η_yaw=1`、`η_lateral=5` 复现该子实验，再扫 0.5–1.5m/s；这些数不能直接当作 Ranger 默认值。

指标：成功/可达终点面积、端点 pose/control 误差、切换时间、`Σω²`、横向速度增量平方和、轮角/速率峰值、最小净空、OBB 碰撞、求解时间与失败率。

### Phase 5：端到端 S09 与鲁棒性

每个 S09 preset 先做 20 个固定起点/障碍扰动，完整方法最终不少于每场景 50 次。初始 pose 不 free 的样本在规划前标注并单列，不从统计里悄悄删掉。对照旧 planner（固定 commit/config）、Ackermann-only、空间联合优化无 transient、完整方法。若无真车，结果明确标为运动学仿真。

消融：无/有 STC；feature-point 数和安全 margin；warm start；Big-M tightness；有/无模式聚类 cuts；三种 terminal-heading 约束；切换代价分项；整体/分段 NLP；无 transient/stop switch/ROI transient。每次仅改一项并用相同种子。

## 6. 指标与统计规范

**安全/可行**：成功率、MIP/NLP infeasible/timeout、最终 validator 拒绝率、最小车体间距、碰撞数（所有接受轨迹必须为零）、轮角/轮速/转向速率/加速度违约、端点误差。

**质量/效率**：路径长度、总执行时间（包含降速停车和舵轮重构）、倒车距离、方向/模式切换数、轮角总变差、最大 yaw rate、横向速度变化、最小净空、偏离粗搜量；分别报告 coarse/STC/MIP/segment/transient/validator 的 p50/p90/p95 和最大时间。

成功率给 Wilson 95% 区间；连续值给 median/IQR，必要时补 mean±SD；按场景类型分层。保留 timeout、失败轨迹和全部配置，不只展示成功截图。Table 6 的 3.32/3.72/2.56s 与 11.38/8.85/8.22s 只作为论文作者机器的参考趋势，不作为我们验收阈值。

## 7. 复现性目录与运行记录

建议新增：

```text
experiments/
  configs/                 # vehicle, scenario, solver, planner
  scenarios/chang/         # 三个论文场景及尺度差异
  scenarios/s09/           # preset + 扰动场景
  scripts/                 # batch, aggregate, plot, validate
  results/raw/<run_id>/    # 每次输入/输出/solver log
  results/summary/         # CSV、统计、图
  decisions/               # 原文未明确项与工程决策
```

每次记录 `manifest.json`：git commit、车辆/地图 hash、随机种子、软件/求解器版本、参数和容差、场景编号、模块耗时、solver status/gap/incumbent、validator 结果。未公开参数建立 `paper_unknowns.md`：原文是否给值、我们采用值、理由、敏感性。预留不参与调权重的固定验证集。

## 8. 风险与 go/no-go

| 风险 | 处理 |
|---|---|
| 论文称 MIP 但模型有 trig/bilinear | 明确归类 MINLP 或做有界 PWA 近似；量化误差与求解时间 |
| Ranger 轮级映射未知 | 查驱动并低速标定；未核实模式禁止作为可执行模式 |
| STC 离散约束不保证连续扫掠 | feature-disc 覆盖检查 + OBB swept validator 独立否决 |
| Big-M 太松/β 太紧 | 物理/地图推导变量界，做 gap、可行率、时间对照 |
| MIP 太慢或许可不可部署 | 小规模 go/no-go；失败则启用有限拓扑 k-best 图作为替代和对照 |
| 论文参数没披露 | 标明复现假设并做敏感性，不声称逐值复现 |
| ZRM 未实车验证 | 仿真和实车结论分开，先做低速单模式/切换台架验证 |

**首个 go/no-go**：在小规模双模式问题上，空间联合优化是否稳定找到通过精确 validator 的解，并在可接受时间内返回？若否，先检查动力学表达、corridor 和 Big-M/非线性近似；之后仍不满足，再转有限候选图方案。

## 9. 阶段交付和完成标准

交付车辆模型/模式关系表、coarse search、STC builder、空间联合求解器、分段 trajectory optimizer、transient optimizer、独立 validator、论文场景与 S09 测试集、全量日志/CSV/图和复现差异表。

完成时要求：固定配置可批量重跑；所有成功轨迹通过连续整车安全和轮级可行检查；三个论文案例和 S09 均有基线、消融及失败统计；论文复现与本车适配分开报告；未实测、solver 近似和模型局限均可追溯。
