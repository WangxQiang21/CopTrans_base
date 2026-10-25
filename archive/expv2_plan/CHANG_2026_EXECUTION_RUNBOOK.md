# Chang 2026 各阶段执行手册

本手册把 [`CHANG_2026_REPRODUCTION_PLAN.md`](CHANG_2026_REPRODUCTION_PLAN.md) 的 Phase 0–6 拆成可执行工作包。每一阶段都有入口条件、任务顺序、交付物和 go/no-go；未过门槛时停在当前阶段，不用后阶段参数去掩盖前阶段模型错误。

## 项目目录与记录规范

建议在 `/home/wq/whut_thesis/expv2/plan/experiments/` 新建：

```text
experiments/
  specs/                    # 论文方程、车辆参数、mode-pair 表
  configs/{chang,s09}/      # 场景、车体、求解器、算法配置
  scenarios/{chang,s09}/    # YAML/地图/车位和碰撞几何
  src/                      # 独立可测模块；ROS 集成层最后接入
  scripts/                  # batch, validate, summarize, plot
  results/raw/<run_id>/     # 输入快照、输出、日志、solver stats
  results/summary/          # CSV/图/统计表
  decisions/                # 未知参数、solver 选择和偏差记录
```

每次规划统一序列化：`run_id, scenario_id, seed, vehicle_config_hash, map_hash, planner_config_hash, solver/version, start, goal_region, status, failure_stage, module runtimes, incumbent, bound, gap, path length, total time, mode sequence, switch count, minimum clearance, validator status`。失败也保存完整样本。不要在 CSV 汇总阶段过滤 timeout/infeasible。

## 阶段关系

```text
Phase 0 规格/车辆/求解器冻结
   ↓
Phase 1 单模式模型 + footprint + STC 验证
   ↓
Phase 2 固定模式序列连续轨迹优化
   ↓
Phase 3 空间层模式/路径联合优化
   ↓
Phase 4 分段时序 + ROI 换模瞬态
   ↓
Phase 5 Chang 三场景复现 / Ranger S09 适配
   ↓
Phase 6 消融、鲁棒性、统计与论文图表
```

当前源码对照和 P0 风险先看 [`CHANG_MODEL_AUDIT.md`](CHANG_MODEL_AUDIT.md)。

当前已有 ideal Ranger 多模式粗搜索、feature-disc STC、固定模式连续几何优化和独立 OBB validator。几何差分 pilot 在 5000 个随机位姿和 1000 组短扫掠中修正采样问题后未见规划漏检；S09 六个有效目标的固定模式优化均通过动作回放、终点容差和 4 cm 总车身余量 OBB 扫掠。结果见[固定模式连续优化报告](experiments/results/summary/固定模式连续优化_阶段结果.md)。当前可以开始有限模式拓扑候选与连续优化的联合选择原型；Phase 2 扰动矩阵和 STC 统一余量仍待完成，Phase 3 的 Chang 精确 MIP/MINLP 仍受公式和求解器决策阻塞。用户确认当前只做几何路径/模式序列规划，轮级执行、速率、瞬态切换和闭环测试属于后续控制阶段。

---

## Phase 0 — 规格、车辆模型与求解器决策

**目标**：任何规划代码动工前，固定坐标、规划级几何参数、模式含义、求解器数学类型和唯一配置来源。完美控制为当前用户确认的抽象，不要求先实现控制器。

### 执行清单

1. 建立 `specs/paper_equations.md`。按 Chang 原 PDF 逐页抄录图 2 和 Eq. (1)–(7)、(11)–(20)、(21)–(32)、(33)–(49)：状态变量参考点、输入、单位、角度正方向、离散步长、边界条件。OCR 文本仅定位，不作为公式证据。
2. 建立 `specs/vehicle_params_chang.yaml` 和 `specs/vehicle_params_ranger.yaml`。至少分开记录 `length,width,wheelbase,track,Lf,Lr,overhangs,steer limits,steer-rate,drive-speed,acceleration,frame_origin`，每值加来源、可信级别和单位。
3. 建立 `specs/mode_mapping.csv`：Chang ASM/DMM/ZRM 对 Ranger A/L/P/S 候选映射；标 `confirmed / candidate / unsupported`，并列出四轮角、四轮轮速与每个模式的控制自由度。默认只有 confirmed 模式允许进入后续实车适配。
4. 将实体轮角读回、角度 wrap、轮速符号、每轮限幅、转向速率、模式重构时间列入后续控制阶段；当前不作为空间规划前置条件。
5. 修订统一参数入口设计：搜索、STC、OCP、最终 OBB validator 共用同一 vehicle config。解决目前 `track=0.364m` 与 safe-corridor backend `track_width=0.430m` 的冲突；当前统一几何安全余量，跟踪误差 margin 留待控制阶段。
6. 生成 `decisions/optimization_formulation.md`：列出每个优化项的数学类型。特别检查空间传播中的 trig、连续控制乘积、二元激活、轮角映射。比较选项：
   - 可接受的 PWA/分段仿射近似（明确区间和最大状态误差）；
   - 支持原约束的 MINLP 形式；
   - 若两者均不能满足资源，采用有限拓扑候选图作为非 MIP 主线替代。
7. 清点实际可用的 solver binary/Python binding/license/runtime，不因本机没发现 `gurobipy` 就直接假定实验机器也一样。写一个最小 solver smoke specification（仅变量/约束类型，不做大规模规划），记录 solver 入口、版本、license 状态及部署条件。
8. 把 S09 的七个 preset 和一组 Chang 三场景重建需求写入 `scenarios/index.yaml`；规定起点、终端可行域、安全 margin、地图 frame、障碍边界。

### 交付与通过门

- 参数审计表无单位/参考点空白；矛盾参数全部有唯一来源或被标为未决。
- 每个 mode pair 有 `continuous-transient / stop-reconfigure / forbidden / unknown` 结论和证据级别；unknown 不允许自动运动切换。
- 优化数学类型和求解器部署路径已确认；若还未确认，Phase 3 不开始。
- 碰撞 margin 的所有来源有唯一计算链，不存在重复膨胀。

### 不通过时

先补车型数据/驱动映射或缩减模式集；不以拟合论文轮距、提高 solver tolerance、弱化碰撞约束跳过门槛。

---

## Phase 1 — 单模式运动学、footprint 与 STC

**目标**：确认预测车辆怎么动、车辆外形怎么被走廊覆盖、最终验证器和 STC 语义一致。

### 模块与步骤

1. 实现独立 `src/kinematics/`，不依赖 ROS：
   - `propagate(mode, state, control, ds_or_dt, vehicle)`；
   - Ranger ideal Ackermann、oblique/diagonal/lateral、spin 几何传播；
   - 车身参考点/轴心点坐标变换；
   - 每步输出运动学残差和几何约束余量。轮级逆解留到控制阶段。
2. 为每种启用模式建立解析检查：直线、正/负最大可行曲率、倒车、DMM 纯平移、Lateral 边界、Spin 正/反旋转。使用独立小步积分器与解析传播比对，写明离散误差随步长变化。
3. 实现单一 `footprint_collision(pose,map)` 与 `swept_collision(pose_a,pose_b,map)`。碰撞边界处理、map origin/resolution、unknown cells 行为都显式配置。
4. 实现 STC builder：从粗中心线重采样；选车辆 feature points/discs；沿每个点的切向/法向生长 corridor；记录每个 corridor 的障碍净距。对随机朝向和边界姿态，采样车身多边形边界点，验证所有 footprint 点落入对应 feature discs 所代表的集合。
5. 构造 6 类极小碰撞地图：单障碍贴边、直窄口、斜窄口、凸角、贴柱原地旋转、终点边缘。分别比较 STC 可行判定和 OBB sweep 判定。
6. 输出可视化：地图、车辆矩形、feature points/discs、STC polygons、被判冲突的 pose/segment；不要只输出 bool。

### 实验矩阵

- 每种模式：至少 100 个固定 seed 随机合法状态/控制点，另有极限值附近样本。
- Footprint：至少 10,000 个随机 `(x,y,yaw)` 与障碍格布局组合；边界方向集中采样角点。
- Sweep：每类 100 个 pose pair，重点包含大 yaw 变化、短平移原地转向和 cell boundary grazing。
- STC：`feature_count × disc_radius × map_resolution × safety_margin` 做小矩阵，保存保守率和漏检数。

### 交付与通过门

- `src/kinematics/`, `src/collision/`, `src/stc/`；`results/summary/phase1_*.csv`；覆盖检查图和误差图。
- 最终 sweep validator 对生成的场景样本零漏检；STC 可以保守拒绝，但 false reject 率报告清楚。
- 每种规划模式的解析不变量和几何边界满足 Phase 0 阈值；低层轮级可执行性留待控制阶段评估。

---

## Phase 2 — 固定模式序列的分段连续优化

**目标**：先证明连续优化和安全走廊本身可靠，暂不把离散模式选择混入求解。

### 执行清单

1. 抽象 `SegmentProblem(mode, start_state, end_state, corridor, initial_guess)`，一个 segment 内只有一种动力学，不允许 `(u,v,r)` 任意混成不受模式约束的 body twist。
2. 固定模式构造 6 种小案例（直、弧、倒车、crab、Lateral、Spin）；然后构造两段/三段路径，切换暂时采用硬 stop-and-reconfigure。
3. 用粗路径形成 STC；初期锁定拓扑 corridor，但允许位置/航向在 corridor 内调整。对每个优化节点约束车体 feature points，不用仅限制 CoG 的轴对齐 box。
4. 同时保存初始轨迹、优化轨迹、solver return status、最大约束残差、每轮碰撞余量；不收敛解必须通过 validator 后才可用于诊断，绝不当成功解发布。
5. 扫描离散间距 `Δd`、STC 余量、初始路径扰动和 optimizer tolerance。由 coarse/fine 两组步长量化计算时间—可行率—误差折衷。
6. 和当前 `lock_verified_geometry=true` 的 OCP 对照：一个为纯时间/速度参数化基线，一个为真几何自由优化。核实自由优化后的轨迹仍通过全车连续 sweep。

### 实验矩阵

- 6 个基础单段案例 × 3 个步长 × 3 个 corridor margin × 20 个起终扰动。
- 两段路径覆盖 A→DMM、A→Spin、DMM→A；Phase 0 判为 forbidden 的组合不运行。
- 比较 segmented NLP 与单个全长 NLP；相同起终状态、地图、初值、约束容差。

### 通过门与失败处理

- 有限几何自由度的优化确实能改变路径且不改变拓扑；所有成功结果 sweep 零碰撞。
- 失败按 `bad_initial_guess / corridor_infeasible / model_infeasible / solver_failure / validator_reject` 分类。
- 若 safety margin 缩小时 validator 出现漏检，回到 Phase 1；若 NLP 常因 corridor 无解，重构粗路径或 corridor，不加碰撞 slack 伪造成功。

---

## Phase 3 — 空间层模式/路径联合优化

**目标**：复现论文最核心的贡献，模式二元变量和连续路径一起求解。

### 建模顺序

1. 先在无障碍 toy map 上建两模式、短 horizon 的模型。状态统一在车身中心或后轴中心；转换全部通过明确的 frame transform。
2. 明确变量表：`x,y,theta`、各模式控制、方向 `p`、模式 `α_i`、状态分解变量、terminal slack（若有）。写出每变量 bounds 和单位。
3. 加入 one-hot、离散动力学、手册支持的高层模式范围、方向逻辑和路径级切换代价；先不加 STC，验证纯几何模式切换结果。完美控制假设下，切换暂按采样点瞬时完成，不优化执行器瞬态。
4. 加入 STC/feature disc 碰撞约束；将粗搜输出对齐 MIP sample indices，给出状态 Big-M bounds。Big-M 必须从粗路径局部 bounds 推出，禁止任意大 M。
5. 加 `J_length/J_steer/J_switch/J_direction`；逐个新增目标项做同一场景比较，权重归一化到同一数量级并保留单位解释。
6. 用 coarse search 解作 warm start；添加模式聚类/最短段约束；对照无 warm start/无 cuts/不同 β 界。
7. 在 toy map 检查完整变量值、运动学 residual 和 STC，随后才迁移到 Chang 三场景；轮级可执行性留待后续控制阶段。
8. 对 solver 的 `optimal / feasible-timeout / infeasible / unbounded / numerical-error` 分别实现日志和返回策略；仅 `optimal` 或满足预设 gap 的 `feasible-timeout` 可进入下游，后者需清楚标注非最优。

### 逐步扩展矩阵

| 子实验 | 模式/地图 | 主要查什么 | 成功条件 |
|---|---|---|---|
| 3.1 | ASM 单模式、无障碍 | 参考点和曲率方程 | 末态误差及动力学残差达 Phase 0 容差 |
| 3.2 | ASM+DMM、无障碍 | one-hot 与模式切换 | 输出不是单步 mode flicker，理想几何传播残差达标 |
| 3.3 | ASM+ZRM、开阔区 | 旋转动力学与终端域 | 原地旋转整车 footprint 安全 |
| 3.4 | 两模式+单墙/窄口 | STC 与 Big-M | solver 解通过精确 OBB sweep |
| 3.5 | Chang 场景 1–3 | 方法整体效应 | 所有统计和失败原因可复现 |
| 3.6 | S09 子集 | Ranger 车辆适配 | 仅对已确认模式报告可执行性 |

### Baselines

- B1 单 Ackermann Hybrid A* + 同样的 STC/连续后端。
- B2 几何 2D A* + fuzzy/规则 mode selector + 相同分段后端。
- B3 多模式 coarse search + 固定其 mode 序列 + 相同后端。
- Ours 空间联合优化模式和几何。

所有 baseline 必须共用同一地图、footprint、安全 margin、终端区域和 validator；否则结果不可归因。

### 通过门

- toy map 模型残差全部在阈值内；solver 正常返回且 MIP/MINLP 状态记录齐全。
- 每个接受解都通过全车 sweep 与理想运动学检查；轮级检查留到后续控制阶段。
- 联合方法必须有可重复案例显示模式与几何共同决策带来收益；若没有收益，先分析 MIP corridor/代价/终端可行域，不直接宣称复现成功。

---

## Phase 4 — 时序分段与模式切换瞬态（后续控制阶段）

**当前状态：延期。** 用户明确当前仅做规划并假设完美控制；因此当前版本不实现速度/加速度曲线、执行器连续性或模式切换瞬态。完美控制下空间规划的 mode switch 暂作瞬时切换。以下保留作为后续控制阶段的实验方案，不是当前 Phase 0–3 的验收要求。

### 4A：分段轨迹生成

1. 从 Phase 3 解按连续 mode label 切 segment，边界写入共享 pose 和 mode-specific control state。
2. 固定一段和多段两类问题；比较论文 `J₂` 不同项：时间、加速度、转向变化。每项先单独加，再组合。
3. 对照一个整体时序 NLP，记录问题变量数、迭代数、时延、状态/控制连续性和可行率。
4. 明确约束点：端点控制是否为零、段与段之间的 wheel-angle initialization、反向行驶下速度符号。

### 4B：切换 ROI optimizer

1. 按 mode-pair matrix 只实现允许连续 transient 的组合；其余一律 stop-switch。
2. 由前后段轨迹按弧长截取 ROI；导出两端 pose/velocity/control；ROI 端点不可随优化漂移。
3. 首版用 Chang 虚拟前/后轮模型；第二实现可用 ICR/β 参数化交叉核对。输入映射到四轮后检查机械 limits。
4. 目标项按论文拆分 `yaw_rate²`, `Δv_lateral²`, `T_switch`；统一尺度/权重归一化。禁止只看总目标值，分别输出三个分项。
5. 实现触发条件、ROI infeasible 分类以及退回 stop-switch。fallback 失败时拒绝整条路径。
6. 生成 constant-radius + B-spline baseline，保证两种方法采用相同端点、起末轮角、速度/时间约束。

### 4C：切换实验矩阵

- 论文 Table 4/5 初始 pose、实体轮角、1.9s、1m/s 和 0.5–1.5m/s 条件作为 Chang vehicle reproduction 参数集。
- 端点模式：自由终点、固定 baseline endpoint；至少 100 组初始 steering/control 条件采样以近似估可行 reach set。
- 输出可达终点散点/占据网格、轮角时序、pose/yaw、lateral velocity、switch duration 和失败率。
- Ranger 参数单独重跑，不混用论文车尺寸/舵机时间。

### 通过门

- ROI 两端 pose/control 连续；碰撞/轮角/速率约束过 validator；切换 fallback 明确。
- 可行域比较使用同一采样边界条件；平滑指标按相同积分时长/归一化定义。
- 若 transient model 无法映射到实体四轮，论文方法仿真可单独保留，但不得进入 Ranger 可执行轨迹链。

---

## Phase 5 — 批量对比：论文三场景与 S09（空间规划阶段）

### 场景构建

1. `scenarios/chang/` 重建三类拓扑：被上侧障碍限制的平行泊车；平行泊车加起点附近违停障碍；窄通道垂直泊车。保留尺寸来源/估读项，参数化缩放到 Chang simulation vehicle。
2. `scenarios/s09/` 引用现有 S09 地图和 7 个 preset；附加窄度、起点姿态和障碍位置扰动，不覆盖原始 S09 基线。
3. 每种场景先人工核查起点、目标区域、车辆 footprint 可行，再放入 batch；无法起步的 initial state 标 `invalid_initial` 并保存。

### Batch 执行顺序

- Smoke batch：每场景 1 个案例，确认消息、输入/输出文件、solver log、validator report 链路完整。
- Pilot batch：每场景固定 20 seeds；查失败分类与耗时长尾；在看最终验证集之前冻结配置。
- Final batch：论文三场景及 S09 主任务至少各 50 seeds；需要复现稳定性曲线时增加至 100 seeds。每个 baseline 和 Ours 用完全相同的 seed。
- 执行顺序随机打散或轮换方法顺序，避免机器热状态/后台负载固定偏向某一方法；记录机器和资源使用。

### 统一报告

- 成功率 + Wilson 95% CI；median/IQR path length、total time；solver p50/p90/p95/max；collision rejection 率；mode sequence 频数。
- 论文方法分项时间和 Baseline 1 分项时间；本机时间不与作者 i5 的绝对秒数硬比较。
- 每个场景各放一组典型成功、边界成功和失败图；所有图都能回链 `run_id`。
- 论文尺度结果单独章节；Ranger/S09 结果标为 adaptation study。

### 通过门

- 所有接受轨迹均由同一独立 validator 判定无几何碰撞/理想运动学违规；不声称控制跟踪或轮级执行已验证。
- 原始日志能从配置和 seed 重建；成功率分母、invalid-initial 处理和 timeout 规则明确。
- Ours 对 baseline 有统计可见且可解释的收益，或诚实报告未优于基线；不通过手挑样本调整结论。

---

## Phase 6 — 消融、鲁棒性和最终复现报告

### 消融矩阵（固定 seed 与其它设置）

1. STC off/on；feature-point 数与圆盘半径；不同 safety margin。
2. coarse warm start off/on；Big-M loose/tight；模式聚类 cuts off/on。
3. terminal heading 无约束 / ±π/8 / 精确目标角；记录首次可行时间与 MIP gap 曲线。
4. `J_steer`, `J_switch`, `J_direction` 单独移除；统计路径、切换数和倒车。
5. full horizon NLP vs segmented NLP；fixed stop-switch vs ROI transient。
6. vehicle/map perturb：车体尺寸、定位误差、yaw 偏差、障碍边界、grid resolution、安全 margin。

一次只变一项；对主结论至少每组 30–50 seeds，pilot 可先每组 10 seeds。

### 结果检查

- 成功率按场景与方法分层；所有失败按阶段分类。
- 画 terminal-heading 的 incumbent/bound/gap vs wall time；solver timeout 仍作为样本报告。
- 对 reach set 用相同网格和相同边界采样；报告覆盖面积和 sampling uncertainty。
- 检查模型近似误差与碰撞安全的关系：近似状态误差不得吃掉 safety margin。
- 汇总论文一致项/偏差项、Ranger 额外工程限制、未实车验证内容。

### 最终交付与停止定义

1. 固定参数可一键重跑 Chang 与 S09 两条轨道。
2. 原始日志、summary CSV、绘图脚本、轨迹文件与报告数字一致。
3. 所有成功轨迹通过最终 validator；未通过轨迹只作为失败样本保存。
4. 论文复现结果和 Ranger 适配结果分开书写；不把作者结果当成本项目成果。
5. 形成复现差异表和复现实验章节初稿，说明无法复现的设定、求解器近似和统计置信度。

## 执行顺序速查

| 顺序 | 阶段 | 继续条件 |
|---|---|---|
| 1 | Phase 0 车辆/公式/solver | 参数源一致、mode pair 已判定、优化类型已选 |
| 2 | Phase 1 运动学/STC/validator | 单模式误差合格，零漏检 |
| 3 | Phase 2 固定模式 NLP | 几何能优化且安全、失败可诊断 |
| 4 | Phase 3 空间联合求解 | 小模型可行，gap/status 正确，S09 外形验证通过 |
| 5 | Phase 4 分段和 transient | 边界连续、fallback 安全 |
| 6 | Phase 5 批量主结果 | 配置冻结、同 seed、日志完整 |
| 7 | Phase 6 消融与报告 | 结果可重跑、统计透明、结论不过度外推 |
