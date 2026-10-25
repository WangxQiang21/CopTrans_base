# Chang 2026 × 当前 Ranger Mini 代码：Phase 0 模型审计

审计对象：当前可见工作区源码与参数文件。`/home/wq/whut_thesis/exp` 不是 Git 仓库，因此无法记录 commit hash；历史 `current_status.md` 不能代替当前源码状态。

## 1. 当前代码实际上是什么

| 组件 | 当前实现 | 与 Chang 2026 的差别 |
|---|---|---|
| 前端候选 | `planner.cpp` 默认 `use_macro_frontend=true`，先搜索 clearance-aware 2D 通道，再将每段平移连接成 Ackermann 标签，并插入原地 Spin 旋转 | Chang 粗搜产生多模式路径/模式初值；这里的 macro path 不是多模式运动学搜索，更没有模式/几何联合决策 |
| 旧 Hybrid A* | 同文件保留 A/L/P/S 离散扩展、显式零位移换模边和 RS shot，但默认 macro 前端走另一分支 | 可作为 baseline/引导器参考，不是当前默认输出 |
| OCP 状态 | `casadi_safe_corridor_backend.py` 优化 `(x,y,theta,u,v,r)` 与加速度，运动是 body twist 连续模型 | 没有 mode binary、没有 Chang 的 switched mode dynamics；优化可混合纵横向/角速度，再在消息序列化时由 twist 阈值回判 mode |
| 几何自由度 | `lock_verified_geometry=true` 时逐点锁定 x/y/yaw；因此 OCP 不能平滑路径几何 | 可做时间/速度层脚手架，但未实现 Chang 的空间 path-mode optimizer，也没有几何优化效果 |
| STC | C++ 生成前/后轴中心走廊，重采样 0.1m；python OCP 各对两个轴中心施加走廊约束；末端节点独立 OBB swept validator | 与论文按车身 feature points/discs 构建 STC 的想法接近，但现有数目、覆盖性和约束实现不同，需做覆盖证明/数值检查 |
| 终端条件 | 当前 ROS 消息起终 pose 固定；空间 MIP 尚不存在 | Chang 优化可用泊车区域 + 航向容差，另有终端约束 MIP gap 对照 |
| 模式切换 | 旧搜索边记软成本；vel QP 硬设切换点速度为零；macro 路径以 Spin 作为 heading change | 没有切换 ROI 轨迹，没有 mode-pair transient planner，也没有轮角重构状态机 |

关键源码：

- `catkin_ws/src/fwis_common/include/fwis_common/kinematics.hpp`：A/L/P/S 位姿步进模型。
- `catkin_ws/src/fwis_common/include/fwis_common/modes.hpp`：模式枚举及定义。
- `catkin_ws/src/fwis_common/include/fwis_common/vehicle.hpp`：几何和限位参数默认值。
- `catkin_ws/src/fwis_planner_hybrid_astar/config/params.yaml`：宏观前端默认开启。
- `catkin_ws/src/fwis_planner_hybrid_astar/src/planner.cpp`：topology path 转 A/Spin 序列；旧 Hybrid A* 与 Shot 仍在同文件。
- `catkin_ws/src/fwis_common/include/fwis_common/safe_corridor.hpp`：C++ corridor 生成。
- `catkin_ws/src/fwis_backend_qp/scripts/casadi_safe_corridor_backend.py`：连续 body-twist OCP，锁几何开关，wheel-rate OCP 默认关闭。
- `catkin_ws/src/fwis_backend_qp/config/backend_params.yaml`：OCP 配置。

## 2. 参数与模型冲突

### P0：轮距冲突已在标准 ROS launch 中统一

AGILE-X 手册给出轮距 364 mm、轴距 494 mm；backend 原先误设为 430 mm。现新增 `catkin_ws/src/fwis_common/config/ranger_mini_2.yaml`，作为标准 launch 的车辆参数入口：planner、playback 和 safe-corridor node 都加载这份配置；backend 的重复车身/轮距字段已从 `backend_params.yaml` 移除，safe node 改读 `~vehicle/*`；`fwis_common` 安装该配置目录，backend 声明对 common package 的依赖。规划器和 Python 参数代码仍保留同值 fallback，以便节点在未使用标准 launch 时有默认值，实际运行请使用标准 launch 并以 ROS 参数服务器为准。

按用户确认，本研究现阶段只做几何规划并假设控制器完美跟踪，控制速率和轮级逆解留到下一阶段。Ranger 手册给出前后阿克曼命令范围 ±40°、斜移范围 ±90°（90°为横移），以及自旋模式；这些是高层命令范围，不等于已验证的实体轮角映射。手册具体规格表列 5.4 km/h，另一张参数表列 2.6 m/s；当前空间规划不依赖速度上限。现有 `max_wheel_steer_rate=2.50 rad/s` 仅为未核实配置假设，不进入当前阶段约束。此次源码和 launch 配置修改尚未编译或运行 ROS 节点。

### P0：现有 OCP 的 mode 后分类不等于离散模式约束

后端优化 `(u,v,r)` 连续 body twist，`u,v,r` 可同时非零；随后才从轨迹速度估计 A/L/P/S 标签。因此，即使标签只出现四种模式，轨迹本身也不保证落在各模式运动学可行集合。Chang 的 `α_i(k)` 在优化中 one-hot 激活单一模式动力学。不能直接把当前 OCP 解释成其分段模式 OCP。

### P0：Ackermann 当前是等效自行车假设

`kinematics.hpp` 采用 `dθ = ds tan(δ)/wheelbase`；`planner.hpp` 由 `r_min_ack=0.81` 反推 `δ_max=atan(wheelbase/r_min)`。这满足半径边界，但并未实现手册未公开的四轮转角/轮速精确映射。K01 另有 `L_eff=0.680m, δ_eq=40°` 等效口径。两者会给相似最小半径，却不是经实车验证的 wheel-level model。Chang 明确要求把 master steering 映射到每个实体轮并约束每轮限位。

### P1：Chang 坐标参考点需转换

Chang 4WID-4WIS 运行示例以虚拟后轮/后轴为参考描述部分控制式，正文也允许选择特定轴心；当前搜索和 footprint 碰撞以车身中心为参考。移植时必须从论文原式确定其每个公式的 reference point，并将后轴中心状态变换到车身中心：`p_body = p_rear + R(θ)[Lr,0]`（若该论文具体模式采用该定义），终端车位和 STC feature points 用统一参考系。不能把等式直接贴入中心点状态。

### P1：模式集合映射不可按标签硬套

当前枚举：Ackermann、Lateral（纯侧向）、Diagonal（body yaw 不变的 crab/平行运动）、Spin。Chang 例子：ASM、DMM（等效单轨控制的对角/平移模式）、ZRM（原地旋转）。初步语义对应可考虑 ASM↔Ackermann，DMM↔Diagonal（Lateral 是边界转角的特殊情况），ZRM↔Spin；但是否可把 Lateral 并入 DMM、是否允许连续角度范围，取决于底盘固件、轮角和轮速约束，当前资料不足以认定。

### P1：转向速率是占位配置

backend 设 `max_wheel_steer=1.570rad`, `max_wheel_steer_rate=2.5rad/s`，但 `enforce_wheel_rate_in_ocp=false`。规划器/消息里也未携带四轮实际角度状态。因此 Chang 的 transient endpoint control matching 目前没有可校准的真实边界值。需先确认轮角反馈、舵机速率、舵角 wrap/reversible-drive 语义，再考虑启用约束。

### P2：可行域与 safety margin 多处独立配置

规划器 footprint margin、backend safety margin、车宽长度、STC map resolution 等配置并非同一配置入口。论文式 STC 和最终 OBB validator 需用一致的障碍膨胀/边界规则，避免“走廊判安全、发布 validator 判碰撞”或反过来。

## 3. 当前环境的求解器状态

只做了模块导入检查（不是运行/算法测试）：当前 Python 环境可导入 CasADi 3.7.2 与 SciPy 1.10.1；未发现 `gurobipy`、Pyomo、CVXPY。故 Chang 空间优化求解器尚无可用实现入口。论文中 Gurobi 的精确 formulation、license、模型类型需先解决；CasADi/IPOPT 已有，可继续承担连续 NLP，但不能处理二元模式选择本身。

推荐开发决策顺序：

1. 把论文变量/约束做成独立小模型，确认真实数学类型（MILP/MIQP/MINLP）。
2. 选择可部署的混合整数求解器；若用分段近似，先对单模式离散模型做动力学误差对比。
3. 仅在 toy map 上测可行解时延和 gap，再决定是否扩到 S09。
4. 若 solver/license 或时间门槛不满足，启用 reproduction plan 定义的 finite-topology k-best 备用架构，不把问题伪装成已有 OCP。

执行中已补充 [`experiments/decisions/optimization_formulation.md`](experiments/decisions/optimization_formulation.md) 和 [`experiments/specs/paper_equations.md`](experiments/specs/paper_equations.md)。原文虽称 spatial optimization 为 MIP 且使用 Gurobi，但精确三角运动学与 binary activation 通常构成 MINLP；需根据原 PDF 中的实际松弛/离散化澄清后再选求解器。`pdftotext` 中 Eq. (5)/(11)/(12)/(21) 公式布局有歧义，当前已显式列为视觉核对项。

## 4. Phase 0 必须先处理的决策

**已完成：** standard launch 的几何参数统一入口与轮距冲突修正。**尚未完成：** 全部 margin 定义去重、solver 选型、真实轮级映射和 mode-pair 状态表。

| 顺序 | 决策/材料 | 完成判据 | 状态 |
|---|---|---|---|
| 1 | 统一 vehicle params；修正 0.364/0.430m track 冲突，统一 footprint 和 margins | 搜索、STC、wheel IK、validator 从同一份配置读取 | 标准 ROS launch 已统一几何；margin 去重待办 |
| 2 | 规划级车辆参数与模式语义 | 车身尺寸、轴距/轮距、阿克曼/斜移/自旋高层范围有手册来源或明确标成假设 | Ranger 手册信息已录入；转弯半径参考点和 CoG 位置待明确 |
| 3 | 逐模式空间动力学核对 | 有状态参考点、符号、变量范围、离散传播误差和倒车约定的表格 | 论文原式已部分目视核对；ASM 平移项语义疑点待裁定 |
| 4 | 模式对转换表 | 当前空间规划标瞬时 mode switch；执行器 transient 属后续控制阶段 | 当前采用 perfect-control 理想化；ROI transient 延期 |
| 5 | Chang formulation 类型 | 把 trig/bilinear 与 binary coupling 全列出，确认实际 solver formulation、许可、近似误差 | 方程规格初稿；MIP/MINLP关系与论文ASM公式疑点待核 |
| 6 | STC footprint 评审 | corridors 保守覆盖整车与 margin，且相同 validator 独立重检 | 待办 |
| 7 | 当前规划基线快照 | 保存参数、七个 preset、输出 CSV/失败图；后续比较用同一地图与起点 | 待办 |

## 5. 与复现计划衔接

- 在 `CHANG_2026_REPRODUCTION_PLAN.md` 中，Phase 0 加入本审计作为起始清单。
- Phase 1 先复用已存在 OBB swept checker 验证 STC 覆盖，不重复实现另一套碰撞语义。
- Phase 2 固定模式的 segmented NLP 可以部分复用 CasADi/IPOPT 管线，但必须移除连续 body twist 的自由模式混合，改成固定模式运动学并显式传输边界 wheel/control state。
- Phase 3 新建空间联合模式优化模块；不要把旧 Hybrid A* 当成 Chang MIP 已复现，也不要把当前 macro topology frontend 当作多模式粗搜。
- 现有旧 Hybrid A*、topology frontend、continuous twist OCP 均保留为不同 baseline，不先删代码。
