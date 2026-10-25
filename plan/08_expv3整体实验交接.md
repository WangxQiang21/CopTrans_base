# expv3 整体实验交接文档

**2026-10-10 最新入口：** [R05 剩余重构与联合优化详细执行方案](15_R05_%E5%89%A9%E4%BD%99%E9%87%8D%E6%9E%84%E4%B8%8E%E8%81%94%E5%90%88%E4%BC%98%E5%8C%96%E8%AF%A6%E7%BB%86%E6%89%A7%E8%A1%8C%E6%96%B9%E6%A1%88.md)。当前默认 `mode_path_v2`，目标速度 2.0 m/s、横向加速度上限 2.0 m/s²，连续时间已允许跨几何节点非零加速度；倒车尽量留到后段，极端情况允许提前及全程倒车。R06 已用同一规划源码完成六个新 profile 锚点各三次共 18 次运行，全部通过独立检查，34 项定向测试已有原件；N03 / P12_env4 净空约 3.0 / 2.8 cm，连续净空 NLP、约束走廊、多初值银行和 v2 段级 MINLP 仍待推进。详见新计划及 [R05 原件汇总](../relust/R05_target2_late_reverse_20261010/comparison.json)。以下各阶段历史合同与耗时按日期阅读。

更新时间：2026-10-06  
工作区：`/home/wq/whut_thesis/expv3`  
关联 ROS underlay：`/home/wq/whut_thesis/exp/catkin_ws`

> 本文是新一轮对话/调试的项目总入口，覆盖研究目标、当前实现、场景与结果、运行方式、已知问题和下一步。下一阶段交给 GPT Luna 的详细执行任务书见 `plan/10_下一阶段详细推进计划与执行方案_Luna.md`。`06_当前算法进度与调试快照.md` 是 2026-10-02 的详细实现快照；`07_P16_narrow_maze性能优化交接.md` 是 2026-10-03 的 P16 专项记录。若本文与当前代码参数不一致，以代码和 `catkin_ws/src/fwis_v3_demo/config/params.yaml` 为准。

## 1. 研究目标、用户约束与范围

目标是研究狭窄/受限环境下的 4WIS（四轮独立转向）车辆多模式几何规划：给定占据栅格、起始与目标车辆中心位姿，自动联合选择可行路径几何及运动模式，安全地通过窄道并达到终点位姿。模式选择与轨迹形状应尽可能协同优化；不能退化成“遇到某地图/某种泊车场景就执行固定模式序列”的状态机。

重要边界：

- 车库只是典型窄空间基准，不是唯一目标；倒车不是成功标准，也不要求规划器为了倒车入库单独写策略。
- 希望减少无意义的频繁转向；小曲率/巡航可以由 Ackermann 完成，大转向由 Spin 是否可行、代价是否更优来竞争决定。不是一概偏好 Ackermann、斜移或 Spin。
- 地图的相对障碍内容和轮廓按论文图数字化。用户指出的问题是车辆与空隙比例不协调，修正方式是统一缩放整张地图的米制分辨率/原点；不要局部挪障碍、人工压窄通道或把轮廓规整成另一张地图。
- 当前只做规划研究和 ROS/RViz demo，不做轮速解算、控制器设计或实车闭环。
- 用户明确要求持续改进当前有效算法；不要把工作转成维护一个无进展的 baseline demo。基线只作为同条件对照实验。

## 2. 坐标、车辆模型和输出合同

规划参考点、ROS 起终点、footprint 与轨迹统一使用车辆中心，即前后轴中心中点。车辆严格前后对称，参数来自 `papers/4wis车辆参数.pdf`：车长 `0.738 m`，车宽 `0.500 m`，轴距 `0.494 m`，轮距 `0.364 m`，最小 Ackermann 转弯半径约 `0.810 m`；前后悬各约 `0.122 m`。

连续平面模型（`v` 可带符号表示前进/倒车）：

```text
A：xdot = v cos(theta), ydot = v sin(theta), thetadot = v kappa
   前后轴反向转向的 4WS 双 Ackermann 等效模型，kappa = 2 tan(delta_v) / L

O：xdot = v cos(theta + phi), ydot = v sin(theta + phi), thetadot = 0
   车身航向不变的斜移；横移是 phi = ±90° 的特例

Spin：xdot = 0, ydot = 0, thetadot = omega
      车辆中心原地旋转，必须对整车旋转扫掠做碰撞检查
```

ROS 规划结果输出严格递增时间戳的车辆中心序列 `(t, [x,y,v,theta])`；`v` 是中心平动速度，Spin 时为 0。模式标签作为轨迹元数据与显示信息。不得把输出点解释成后轴中心轨迹。

## 3. 当前真实算法架构（不是目标架构的全部）

### 3.1 实际运行主链

```text
OccupancyGrid / 起终点
  → 距离场与多 clearance 权重二维 8 邻接 A*，生成拓扑候选
  → 按局部转向自适应间距取路线锚点，再跑 O/Spin DP + 稀疏 A/O/Spin 图
  → 候选作为 incumbent，限额 A/O/Spin Hybrid A* 尝试降 cost
  → Python 固定模式事件序列连续 NLP
  → 锚点 DAG / 末端局部候选 / 最多两轮交替模式—几何提议与 NLP
  → 精确运动学重放、中心终点容差、整车 OBB swept-SAT
  → 安全时间化，再独立扫掠复验，发布 ROS Path、轨迹、模式、HUD 与 RViz marker
```

2D A* 只找空间拓扑，不包含车辆航向、档位或模式，路线本身不是车辆可执行轨迹。稀疏候选图通过同一代价竞争 A/O/Spin：状态保留路线锚点、到达航向、上一模式、方向和离散控制历史；A 用有限范围 Reeds–Shepp 型弧连接，O 为斜移原语，Spin 为同中心姿态旋转；所有候选边扫掠验车体。Hybrid A* 仍保留，用可行候选做上界并进一步改进，或在候选图没有 incumbent 时搜索可行性。

### 3.2 连续优化和所谓“联合优化”的实际含义

Python 后端先沿粗解给定的模式/事件序列做连续 NLP；随后锚点 DAG 和有限末端候选尝试改变局部连接、几何或模式切换点。默认最多两轮交替执行“在当前最佳安全几何周围提议离散模式/锚点变化 → 对一个候选做固定序列 NLP”。只接受共同 cost 严格下降、终点满足约束、独立 swept-SAT 通过的候选；低于 `0.25%` 的一轮仍可保留改进，但会跳过后续轮次。

这是有限候选上的局部离散—连续协调，不是走廊内完整模式整数变量与连续轨迹变量的全局 MINLP，也没有全局最优保证。Bonmin `joint.enabled` 当前关闭，困难案例会失败或 `LIMIT_EXCEEDED`。不能称为“已接入最先进算法”；它是结合文献思想和当前可运行约束形成的阶段性工程算法。目标走廊内完整联合模型及拓扑补图仍是后续研究方向，详见 `plan/05_决策与轨迹联合优化算法分析.md`。

### 3.3 代价与约束

当前 cost 以中心行驶距离为基础，并加 Spin 角度、模式切换、低曲率处 A/O 切换、倒车/换向、Ackermann 曲率与控制变化、O 斜移角变化/斜移使用量及低角度 O 巡航惩罚。参数见 `catkin_ws/src/fwis_v3_demo/config/params.yaml`。核心当前值包括：`w_switch=2.00`、`w_low_curvature_switch=0.35`、`switch_turn_threshold=0.35 rad`、`w_reverse=1.00`、`w_direction=20.0`、`w_steer_tv=0.40`、`w_curvature=0.01`、`w_oblique_tv=1.50`、`w_oblique_event=1.50`、`w_oblique_use=0.55`、`w_oblique_cruise=0.45`。Spin 系数默认按 `v_demo/omega_demo` 换算。

这些是工程代理权重，不是测得的真实耗时、能耗或跨场景最优权重。目前尚未完成多地图 Pareto/权重扫描及严格 Ackermann-only 配对基线。成功与否必须首先看完整路径安全、末端精度、路径连续/曲率、净空和时延；模式是否包含倒车不构成好坏判据。

## 4. 安全、时间化和可视化

- footprint 使用矩形 OBB；车辆长宽及 `collision/margin=0.04 m`、`sweep_guard=0.005 m` 配置见参数文件。A 图边默认 40 mm、路线 O 边默认 10 mm；样本按运动角点位移的一半扩张。时间化轨迹最终使用更密的 swept 检查（当前 `timed_sweep_corner_step=0.0025 m`）。安全结论以静态 occupancy + 理想运动学 + 独立整车扫掠复验为限，不代表实车安全。
- 平移演示参数 `v_demo=2.0 m/s`，Spin `omega_demo=2.0 rad/s`，`dt=0.1 s`；Smoothstep 平移组平均约 `1.33 m/s`。这是输出/RViz 演示时间标度，不是控制器指令或实车速度承诺。
- RViz 分开展示粗解和最终解；Ackermann/Oblique/Spin 分别青蓝/橙/紫，HUD 有当前模式，ego 有航向箭头。走廊框按参考航向，不是世界轴对齐；颜色表达最终局部模式，不代表已生成三套模式专属走廊。point fallback 用琥珀色警示，不作为安全证明。

## 5. 当前实验与结果证据

### 5.1 S09 预设目标

`relust/report.md` 与 `relust/metrics.csv` 记录 2026-10-02 的七个预设目标结果；配图为 `relust/s09_presets_1_2_3_4_5_6_7_modes.png` 和 `relust/s09_presets_1_2_3_4_5_6_7_costs.png`。该次 7/7 规划结果通过终点/时间化扫掠检查，结果包括 A/O/Spin 混合解，也有按 cost 选择的 O/Spin 或 Spin/O 解；不得为满足“必须多模式/必须倒车”篡改目标。

该次各目标墙钟约 `0.45–4.91 s`，粗搜索 `0.13–1.41 s`，优化器 `0.27–4.19 s`。交替阶段的报告 cost 改善范围约 `0.01–19.51%`，但这是单次运行内部阶段对比，不是新旧版本配对消融，也不说明跨场景普遍提升。#5 后端耗时较高，交替阶段和末端候选有开销；更多场景及消融仍欠缺。

### 5.2 P16_narrow_maze 最新卡点（2026-10-03）

`07_P16_narrow_maze性能优化交接.md` 记录专项复现。P16 来源于 Zhu 论文 Figure 19，地图已按论文车辆宽度统一缩放（约 0.25 倍）；起终点不在地图文件中。专项测试用近似坐标 `(12,12,0) → (18,13.75,0)`，对应约 `(3,3) → (27,10)` 的缩放结果，精确车头朝向未确认。

早期无 incumbent 测试曾让两轮 Hybrid A* 各消耗 180k 次扩展，失败耗时约 `155.13 s`。后续 C++ 增加按栅格行合并占用 cell 后的矩形段 SAT 查询，并令引导搜索和 fallback 共用 `max_iter=180000` 总预算，各 90k；同一近似姿态的早期复测约 `3.16 s` 失败。再后续诊断、clearance-floor 候选和 0.5 倍 fallback 结果见下方当前状态及专项文档；不要将早期 `no_route_candidate` 摘要当成最新诊断。

**当前 P16 状态（2026-10-03）：** `search/enable_route_heuristic` 已恢复为 `false`。基于近似姿态 `(12,12,0) → (18,13.75,0)` 的多轮复测仍未找到安全路线；增加 clearance-floor 路线候选和 0.5 倍细栅格 fallback 后仍在 180k 搜索预算内失败。最新诊断显示候选路线可到达 waypoint 约 8/37，稀疏混合图没有触预算上限但前沿停在路线中段。地图没有标注起终点，当前姿态不能作为论文场景的确定失败结论；不再针对这组估计坐标追加地图专用规则。

**本轮代码进展与验证（2026-10-03）：** 路线阶段诊断已补充有效/重复 2D 路线数、候选边碰撞类别、waypoint DP 最远可达点、混合图调用/扩展/预算命中及分阶段耗时。路线候选无解时会尝试更细的 fallback lattice；仅在普通候选无 incumbent 时增加 clearance-floor 备选。`catkin_make -C catkin_ws -j4` 通过，`run_demo.sh` 启动期间也重复完成 overlay 构建。

**多地图验证状态：** 通用压力地图 N01–N06 均用各自 `scene_source.yaml` 的起终点跑通粗搜和优化，且最终时间轨迹通过独立 5 mm 定时扫掠 SAT 检查（6/6）。逐图的端点、expanded/action 数、cost、时间、终点误差和探针 JSON 见 `relust/map_probe_report.md`。这些是通用算法压力图，不是论文数字化图结果。

**全局 MINLP 初步评估：** 当前默认局部锚点/模式交替优化（最多两轮）已经有多个地图的接受记录；它不等同于 Bonmin 全局模式 MINLP。Bonmin 试验中，N04 的 29 knots 默认超限；上限临时设为 32 后报 `LIMIT_EXCEEDED`，约 27 s 后退回局部 incumbent。N02 的 21 knots 也超出默认上限。N05 的 B-BB 尝试会返回超时、起点零动作或 swept-SAT 碰撞候选；所有危险候选都被拒绝。参数保持 `enabled:false,max_knots:18`。当前不建议直接全图启用 Bonmin；先解决可行 incumbent 和 swept 碰撞约束，再用相同 seed 比较可行率、cost、延迟和安全性。

**2026-10-04 迭代：** 固定模式 NLP 与实验 MINLP 增加可配置的末端位置/航向软精度项，默认 `w_terminal_xy=8,w_terminal_yaw=2`；终点容差仍为硬约束，所有候选仍需精确重放和独立 swept-SAT。N01–N06 复跑保持 6/6，通过后最大位置误差降至 0.043 m；N06 航向误差由 0.135 rad 降至 0.099 rad。N04 cost 从 35.390 降至 33.181，其余地图 cost 小幅增加约 0.2–1.2%。Bonmin 增加 `joint/max_translation=3.0` 以将 N05 整数 horizon 从约 11 knots 压至约 8 knots，但仍 `LIMIT_EXCEEDED`，说明当前瓶颈不只是 knot 数量；默认仍关闭。

**2026-10-05 全局 MINLP 迭代：** A/O/S one-hot 编码减少为 A/O 两个整数变量、Spin 由约束推导，连同 gear 共 3 个整数变量/knot。碰撞走廊由 8 个小圆盘改为 4 个沿车长分段且覆盖全车身的圆盘；每段再按最多约 0.75 m 间距约束解析中间状态。Bonmin 使用可配置的 `most-fractional` 整数分支选择，独立 `bonmin.opt` 中对子问题限迭代数和 wall time；默认配置仍是 `joint.enabled:false`。

N05 knot-only MINLP 曾找到终点合格但 swept-SAT 碰撞的候选，现被安全校验拒绝。`most-fractional` 在原半步模型中 12 s 内找到整数候选，但第 4、6 段 swept-SAT 碰撞；将走廊间距收紧到 0.75 m 后，20 s 内仍返回第 4、6、7 段碰撞候选。约 0.5 m 间距的 8-knot 模型在 20 s 限时内未形成 incumbent，返回起点零动作，位置差 11.816 m；短暂将单 NLP 子问题上限放到 6 s 也无可接受解。临时放宽 `max_translation` 到 4 m 得到 7 knots，12 s 同样失败。未文档化的 `fix_and_solve_heuristic` 在 N05 原生段错误，已移除。N05 seed 与各组直接 Bonmin 参数/结果记录在 `relust/N05_minlp_20261005_seed_and_trials.json`。这些早期试验的结论是走廊约束退化后无法挡住段内碰撞。

同日全 ROS N05 标准探针在全局 MINLP 关闭时，由现有 `iterated_joint` 接受 1 个 NLP 提案：cost `23.667`、优化器耗时 `2.16 s`、位置误差 `0.0404 m`、航向误差近 0；5 mm 独立 timed swept-SAT 通过（3,185 samples）。与旧单次 cost `23.869` 的差异约为 -0.85%，只是单次运行间波动，不能当作版本收益；原始记录为 `relust/N05_oct05_iterated_probe.json`。

**2026-10-05 cutoff 与碰撞反馈配对试验（全局退化走廊版本）：** 给 `joint_refine` 增加 `use_seed_cutoff` 开关，按 seed 的 MINLP 初始目标值设置 Bonmin `cutoff`。在同一 N05 seed、0.75 m 走廊和 12 s 时限下，开/关 cutoff 都约 12.1 s 后 `LIMIT_EXCEEDED`，返回零动作且末端差 11.816 m；20 s 开 cutoff 得到的整数候选与此前无 cutoff 的 20 s 结果相同，精确 swept-SAT 仍拒绝第 4、6、7 段。Bonmin 接受该选项，但配对结果没有可见收益。

碰撞反馈重试也未恢复可行解：分半预算首轮约 11.09 s 就返回零动作；每次尝试给满 20 s 时，不带分离平面总耗时 40.28 s，带 SAT 分离平面和 no-good schedule 总耗时 45.43 s，第二轮都 `LIMIT_EXCEEDED`。所以 `collision_retries` 默认设为 0。随后通过局部保留圆盘 clearance 找到 N05 安全候选，细节见下文；单纯增加 Bonmin 时限没有质量收益。试验细节记录在 `relust/N05_minlp_20261005_seed_and_trials.json`。

**N05 局部走廊退化修正及 MINLP 首个安全候选：** 诊断发现 33 个 seed 走廊采样站 × 4 个圆盘特征中，只有 2 个位置未达到圆盘 clearance（分别短 `0.011 m`、`0.015 m`），旧逻辑却将整条路径退化为 point-only corridor。现改为逐 feature station 局部 point fallback，其他 130 个位置保留圆盘约束。相同 N05 seed 下，20 s cutoff 开/关都稳定返回同一候选（`10011111`），common score `23.66664`，相较 seed `24.16200` 降 `2.05%`；位置误差 `0.00949 m`、航向误差 `0.0200 rad`。测试侧独立 5 mm swept-SAT 检查通过（3,160 samples）。其 cost 与既有 `iterated_joint` ROS 结果 `23.667` 基本相同，目前没有证明 MINLP 优于现有优化器。该结果仅覆盖 N05 单 seed，不能外推到 N01–N06；全局 MINLP 默认仍关闭。

**N01/N03 跨图 MINLP 检查：** N01 seed 折叠后仍有 13 knots，模型走廊全程满足圆盘 clearance，但 20 s Bonmin 时限实际约 29.0 s 后没有整数 incumbent，返回零动作并被终点检查拒绝。增加一次 root NLP 重解或 fractional dive 启发式后，仍分别约 25.0 s、24.1 s 返回零动作。N03 seed 为 9 knots，20 s 内找到的候选在第 1 段被 exact swept-SAT 拒绝；再加一轮 SAT 分离面和 no-good schedule 后，总耗时约 36.3 s，仍因 `LIMIT_EXCEEDED` 无可用候选。将 N03 走廊间距收紧到 0.5 m（6 substeps）后，实际约 38.4 s 仍没有 incumbent；把 `max_translation` 设为 4 m（8 knots）后约 26.1 s 仍无 incumbent，设为 6 m（7 knots）后约 12.8 s 返回 `INFEASIBLE`。两图原 ROS seed 都通过 timed swept-SAT，MINLP 失败后仍保留原基线。结果分别在 `relust/N01_minlp_seed_probe.json` 和 `relust/N03_minlp_seed_probe.json`。目前仅 N05 有安全 MINLP 候选，不能启用全图。

将 N03 现有 `iterated_mode_geometry_refine` 输出作为更好的连续 warm start，common score 为 `21.3543` 且 swept-SAT 安全，但末端误差 `0.0736 m / 0.0538 rad`。默认 MINLP 末端容差下初始赋值不可行；放宽至规划容差 `.16 m / .14 rad` 后，Bonmin 仍在超出 20 s 配置时限的约 28.5 s 后没有 incumbent。另一个 fixed-mode NLP warm start 在更密的 5 mm sweep 下被拒绝。root 重解和 fractional dive 也没有改善 N01 可行率。下一步关键点是向 Bonmin 注入可行整数 incumbent，并妥善处理更紧的末端投影；`set_initial` 本身不足以保证 Bonmin 保留 seed 作为 incumbent。

2026-10-05 N03 局部走廊/搜索策略迭代：seed 第 1 段 20 个圆盘 clearance 采样中只有一个站短 `0.0022 m`，exact swept-SAT 碰撞也位于该段。把该段走廊局部加密到 8 substeps 后，约 `22.79 s` 仍 `LIMIT_EXCEEDED` 并返回被终点检查拒绝的零动作，因此新 `adaptive_collision_refinement` 默认关闭。SAT 平面反馈重试时保留原 mode/gear schedule、没有立刻加 no-good cut，第二轮仍超时，总耗时约 `50.47 s`。关闭两种 feasibility pump 后，约 `16.24 s` 返回与原试验相同的 `111100000` 碰撞候选；B-OA 约 `4.23 s` 报 `INFEASIBLE`。这些策略均无可用安全解，不能据此启用全局 MINLP。细节已写入 `relust/N03_minlp_seed_probe.json`。

**2026-10-05 全局 MINLP ROS 配对复验：** `joint_refine` 对 Bonmin mode/gear schedule 追加 SAT 分离面并由 IPOPT 固定 schedule 修复；所有接纳轨迹仍经过独立 5 mm timed swept-SAT。N03 同一起终点 matched-off 为 `21.476`（1.47 s）。两次 global-enabled ROS 运行都由 Bonmin 返回 `LIMIT_EXCEEDED` 下的 `111100000` schedule，经固定 schedule 修复得到约 `19.540` 候选，最终 `iterated_joint` 分别为 `19.354` 和 `19.537`；相较 paired baseline 改善 `9.88%`、`9.03%`，两条最终轨迹均通过独立 swept-SAT。第三次把 warm-start SAT retry 上限增至 16 后，虽然固定 seed warm start 可行，但 Bonmin 没有 terminal incumbent，最终 fallback 得分 `20.896`、规划时间 `57.36 s`；上限因此恢复为 2 次。

N01 matched-off 为 `31.960`（1.72 s）。global-enabled 最终为 `31.098`，改善 `2.70%`，5 mm timed swept 通过；Bonmin `LIMIT_EXCEEDED` 未返回 terminal incumbent，收益来自 seed schedule IPOPT fallback（`31.451`）及之后的 `iterated_joint`，总规划时间 `25.75 s`。打开 fractional dive 未改变状态或分数，耗时升至 `29.99 s`。N05 matched-off `23.667`；最初 MINLP 尝试被一个过严的初值诊断门槛提前拒绝（走廊约束残差仅 `1.93e-5`）。将该诊断容差由 `1e-5` 改为 `1e-4` 后，N05 Bonmin 返回 `SUCCESS` 候选 `23.666639`，其模式序列种子经 `iterated_joint` 得到最终 `22.984`（改善 `2.89%`），5 mm timed swept 通过，规划时间 `16.92 s`。该容差只控制是否在求解前拒绝微小的初值数值残差；原约束未放宽，最终 swept 校验不变。

当前 N03 两次重复与 N05 的新 N05 结果构成多图端到端收益证据；N01 体现失败时安全 fallback。但 N01 没有 Bonmin 终端 incumbent，N03 仅两次拿到同一有限 incumbent，求解时延仍高，故还不能宣称跨图稳定的直接 MINLP 成功。默认保持 `joint.enabled:false`、`time_limit:1.5`、`random_seed:0`。逐次 N01/N03/N05 probe 与汇总见 `relust/global_minlp_20261005_paired_probes.json`。

Bonmin 的 B-BB 对这类非凸模型仍是启发式搜索，不能提供全局最优保证；官方选项表列有 fractional dive 启发式，但本轮 N01 对照未见收益（[COIN-OR Bonmin 介绍](https://coin-or.github.io/Bonmin/Intro.html)，[Bonmin options list](https://coin-or.github.io/Bonmin/options_list.html)）。

### 5.3 地图数据状态

- 正式论文数字化地图：`maps/paper_digitized/` 下 P11 三图、P12 四图、P16 两图，共九图。原始论文图片不是作者 occupancy grid，属于按图提取的近似场景；起终点通常需在新缩放坐标系手动设置。
- 通用算法压力场景：`maps/map10-*` 至 `maps/map15-*`，用于窄门、交错障碍、斜向挤压等几何通行测试。
- S09 车库预设地图来自 exp ROS 场景，入口/姿态在 underlay 场景配置中。
- 局部改过障碍内容的旧 tight 版本已放入 `maps/archive/scale_content_variants_20261002/`，不应作为当前论文图结果；`run_demo.sh --list-maps` 不列它们。
- 用户对地图的最终要求是比例正确、车辆和间隙关系协调、保持论文原场景几何。修改尺寸时全图等比例调 `map.yaml`，不对局部障碍做手术。

## 6. 代码和资料导航

| 内容 | 文件/目录 |
|---|---|
| ROS 一键启动、地图选择 | `run_demo.sh` |
| 规划器 C++、模式图、Hybrid A*、SAT 粗搜 | `catkin_ws/src/fwis_v3_planner/src/planner_node.cpp` |
| 连续轨迹模型、cost、碰撞与优化内核 | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py` |
| ROS 优化流程、候选图、交替迭代、时间化/markers | `catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py` |
| ROS 参数 | `catkin_ws/src/fwis_v3_demo/config/params.yaml` |
| ROS 消息合同 | `catkin_ws/src/fwis_v3_msgs/msg/` |
| 论文算法综述（16 篇） | `plan/01_论文编号与算法综述.md` |
| exp/expv2 流程 | `plan/02_exp与expv2算法流程总结.md` |
| 方案框架（含车辆模型与推荐实现） | `plan/03_expv3推荐算法框架.md` |
| ROS 移植接口 | `plan/04_ROS接口移植与优化接入方案.md` |
| 决策与规划/联合优化分析 | `plan/05_决策与轨迹联合优化算法分析.md` |
| 10-02 实现快照、S09 实验和详细回归 | `06_当前算法进度与调试快照.md` |
| P16 本轮性能专项 | `07_P16_narrow_maze性能优化交接.md` |
| cost 分项、图片与轨迹 CSV/JSON | `relust/` |
| 纯算法单测及 ROS smoke | `tests/` |
| 论文地图比例脚本与场景来源信息 | `tools/digitize_paper_maps.py`、`maps/paper_digitized/` |

主要 ROS 话题：粗搜索 `/plan/search_status`、优化状态 `/plan/status`、粗路径 `/plan/path_coarse`、最终 `/plan/path`、粗/最终模式色带 `/plan/mode_strip_coarse` 与 `/plan/mode_strip`、走廊 `/plan/corridor_markers`、HUD `/robot/hud`、航向箭头 `/robot/heading_marker`。具体类型可看 launch、message 定义和 RViz 配置。

## 7. 启动、构建和测试

从 expv3 目录可启动 S09 默认场景：

```bash
cd /home/wq/whut_thesis/expv3
./run_demo.sh
```

论文图列表和手动选图：

```bash
./run_demo.sh --list-maps
./run_demo.sh --map P16_narrow_maze rviz:=false
```

手动选图时用 RViz `2D Pose Estimate` 设置起点，`2D Nav Goal` 设置终点。headless 时向 `/start_pose`（或 `/initialpose`）和 `/move_base_simple/goal` 发 `map` frame 姿态，随后读取 `/plan/search_status`、`/plan/status`。P16 当前近似复测姿态和 ROS 消息例子见专项交接 `07`。

常用验证：

```bash
cd /home/wq/whut_thesis
python3 -m unittest discover -s expv3/tests -p test_core.py -v
source /opt/ros/noetic/setup.bash
source exp/catkin_ws/devel/setup.bash
catkin_make -C expv3/catkin_ws -j4
```

S09 完整预设结果采集见 `tests/capture_s09_results.py`；独立 ROS 终点/swept 检查见 `tests/ros_parking_goals_smoke.py` 与 `tests/ros_s09_smoke.py`。冒烟通过只证明被测姿态/参数，不代表算法对其它地图普遍可靠。机器上此前发现多个遗留 ROS master/planner（含 11311/11312/11320 会话）；新开 ROS 测试前检查 `ROS_MASTER_URI` 和 `rosnode list`，不要不辨会话地杀旧节点。

## 8. 当前主要卡点与下一步

建议下一步按此顺序推进：

1. 将 N01–N06 六图结果固化为基线，在其中挑选 N02（长交错门）和 N04（混合模式/高切换）做重复运行，核实随机性、优化器耗时和 cost 方差。
2. 先解决 P16 起终点/朝向从论文图精确映射的问题，再决定是否继续拓扑候选覆盖；近似坐标失败不视为地图级失败。
3. 做 C++ row-run SAT 等价性核验，并把 S09、至少一张 P11/P12 数字化图加入同一探针/报告链路。
4. Bonmin 已减少每 knot 整数变量，并加入 0.75 m 中间位姿约束。N03 matched-off `21.476`；此前两次全局接入经固定 schedule + SAT/IPOPT 修复得到 `19.354`、`19.537`。本轮批量 SAT 面在修复预算 6 s/8 s 下再次得到 `19.537`；将 Bonmin 总预算设为 15 s 后，最终分数进一步到 `19.460`，低于 off 9.39%，5 mm sweep 通过、耗时 53.43 s。总预算 12 s 也找到同一 schedule，最终 `19.468`、58.46 s，说明名义时限没有带来单调的 wall-time 变化。4 s 修复在终点误差门槛附近超时。关闭 N03 warm start 反而耗时 80.07 s，最终 `20.863`，故 N03 opt-in 配置应保留暖启动。N05 此前有一次 `22.984`（低 2.89%）的安全结果，本轮复验为 `23.529`（低 0.58%），Bonmin 未返回 terminal incumbent，反映候选可用率仍不稳定。N01 从 `31.960` 降至 `31.098`，但收益来自安全 seed fallback 和局部迭代。继续查 Bonmin/Ipopt 节点 NLP 限时在当前调用路径中的实际效果，并提高跨图 incumbent 可行率；默认仍关闭全局 MINLP。数据见 `relust/global_minlp_20261005_paired_probes.json` 与 `relust/map_probe_report.md`。
5. 再做跨图代价权重/交替阶段开关和 Ackermann-only 同条件对照。结果评价包括独立扫掠、终点误差、搜索与优化时延、cost、模式切换和净空，不以指定模式或模式数作为目标。

算法迭代继续围绕“低维拓扑候选 + 安全稀疏多模式图 + 连续优化 + 局部补图”的通用架构推进；不为 P16、车库或单个图添加模式状态机，不通过放松 footprint/SAT 来换取速度。每次报告需区分：编译/单测、粗搜成功、优化器成功、最终时间轨迹独立扫掠通过；没有运行的环节明确标未验证。

## 9. 2026-10-05 SAT 批量修复与全局 MINLP 复验

本轮在 `core.py` 中把碰撞段的 SAT 切面从逐轮单面改为一次最多收集 4 个、沿路径至少间隔 0.2 m 的 seed-preserving 面；guard 不超过安全种子在该分离轴上的实际余量。固定 schedule 修复 NLP 的 wall budget 使用 `max(subproblem_time_limit, collision_repair_time_limit)`，Bonmin 根节点 NLP 上限不变。模型仍以精确车辆回放和 swept-SAT 决定最终候选是否可用。

- N03 修复预算 4 s 时两次都没接受全局候选，分别在 40.60 s、40.19 s 后保留安全 baseline `21.476`。其中第二次末端位置误差 `0.00519 m`，航向误差 `0.02044 rad`，略超严格终点门槛。
- 将修复预算提到 8 s 后，`111100000` 的固定 schedule IPOPT 候选 `19.539704` 被接受，端到端 `19.537`、耗时 `54.55 s`；6 s 复验也得到同一分数，耗时 `56.16 s`。两次均比 off 低 9.03%，独立 5 mm timed swept-SAT 通过。
- 将 Bonmin 总预算从 20 s 降到 15 s、修复预算保持 6 s，仍返回 `111100000`，修复候选 `19.539766`，端到端 `19.460`、耗时 `53.43 s`（比 off 低 9.39%）；降到 12 s 也返回该 schedule，端到端 `19.468`、耗时 `58.46 s`。这说明当前总耗时受节点 NLP/求解器调用时限影响，尚未随 Bonmin nominal time limit 单调变化。
- 关闭 N03 warm start 的消融耗时 `80.07 s`，没有 Bonmin terminal incumbent，安全 fallback 后得分 `20.863`，5 mm sweep 通过。对 N03 应保留 warm start；warm-start retry cap 仍维持 2。
- N05 用 6 s 修复预算复验耗时 `30.85 s`，Bonmin 无 terminal incumbent，fallback 后 final `23.529`，相对 off `23.667` 低 0.58%，5 mm sweep 通过；此前较强的 `22.984` 未能在本轮重现。

本轮没有理由打开全局 MINLP 默认开关。Ipopt 官方选项把 `max_wall_time` 定义为单个 NLP 的 wall-clock 上限，并在收敛检查时终止（[Ipopt options](https://coin-or.github.io/Ipopt/OPTIONS.html)）；下一步核实 Bonmin/CasADi 当前运行时是否将该限制按预期传入节点求解，再处理 N05 的 incumbent 波动，之后做 N02/N04 或论文数字化图配对复验。探针原件与配置对照列于 `relust/global_minlp_20261005_paired_probes.json`；所有 ROS 探针只用隔离 master 11324，结束后已关闭，11311/11312/11320 未触碰。


## 10. 2026-10-05 SAT 重试预算与 Bonmin 限额诊断

本轮定位并修复 warm-start 碰撞失败日志的格式化错误：原 `joint_solution_swept_sat_collision` 参数顺序会把 `repair_failure=None` 格式化为浮点数，产生 TypeError 并掩盖真实 swept-SAT 失败。修复后日志指出种子 schedule 在碰撞段仍需额外 SAT 面；warm-start 未成功不等同于 retry cap 本身无效。

N03 的 paired-off 为 `21.476`。固定 Bonmin 配置为 15 s、根节点 NLP 4 s、SAT repair 6 s，warm-start retry 2：

- 修复重试上限 4：安全 fallback `21.476`，28.26 s；SAT 阶段 10.28 s，段 1 仍碰撞。
- 上限 8：安全 fallback `21.476`，34.18 s；累计 9 个切面后仍检测到新碰撞。
- 上限 12：安全 fallback `21.476`，88.66 s；Bonmin 43.08 s，无 terminal incumbent。
- 上限 16：错误报告修复后得到 `19.539766` 固定 schedule 候选；最终 `19.520`、56.70 s，较 off 低 9.11%，5 mm timed swept-SAT 通过。SAT 修复占 38.17 s，后续局部迭代将候选再降约 0.020。
- `bonmin_iteration_limit=600` + 上限 16：最终 `19.521`、56.50 s，SAT 阶段 38.25 s；相较未设累计迭代上限没有实际提速。该参数仅作为 opt-in 诊断保留，默认 0。Bonmin 文档称 `iteration_limit` 累计限制节点 NLP 迭代、`time_limit` 是全局计算限制（[Bonmin options](https://coin-or.github.io/Bonmin/options_list.html)）。
- SAT 每段一次尝试最多 8 面、0.10 m 间距、修复重试 4：27.20 s 后仍回退；固定 NLP 初值有两条新切面违反，最大 `0.00925`，IPOPT 返回 `Restoration_Failed`。恢复四面、0.20 m 间距。
- 把递归 SAT 修复改成共享 6 s wall budget：候选未接受，Bonmin 主求解实测 47.36 s；已撤回共享预算代码。

结论是低于 16 次的配置目前没有复现 N03 全局候选收益，16 次可以得到约 9.1% 安全收益但 SAT 阶段仍慢。增加切面密度导致连续修复初值不可行；Bonmin 600 次累计迭代上限未减少端到端耗时。保留 exact swept-SAT、原始四面/0.20 m 批量规则、`joint.enabled:false` 与参数默认值。下一步要让每轮切面避开已有冗余面，并构造切面加入后的可行初值；同时查清 15 s Bonmin 设置为何个别运行超时至 43–47 s。

源码与参数通过 `py_compile` 和 `catkin_make -j4`。所有 ROS 探针只使用隔离 master 11324，启动后已关闭；配置文件已恢复，11311/11312/11320 未触碰。探针明细和原始 JSON 名称见 `relust/global_minlp_20261005_paired_probes.json` 及 `relust/map_probe_report.md`。


## 11. N05 post-diagnostic-fix paired复验

在 N05 原起终点下，以 `time_limit=20 s`、warm-start retry 2、SAT 修复 16 次/6 s、`bonmin_iteration_limit=0` 复验：warm-start 使用成功（2.49 s），但 Bonmin 无 terminal incumbent；安全 seed fallback `23.875295` 后局部迭代得 `23.529`，总耗时 `30.02 s`。相较 matched-off `23.667` 低 `0.58%`，独立 5 mm sweep 的 3,163 个样本通过。它重复了较弱的 N05 结果，没有重现此前 `22.984`，因此跨图 incumbent 可用率仍未稳定。探针为 `relust/N05_global_minlp_postformatfix_iterlimit0_time20_repair6_probe.json`。


## 12. 2026-10-05 ranked SAT cut 与 N05 时域安全复验

将切面发现从每个碰撞采样 2.5 mm 改为 1 cm，保留接受检查的 2.5 mm exact sweep；对碰撞样本计算分离面违反量并优先选择最严重的样本，去掉已有切面后最多选 4 面、沿同一原语至少间隔 0.20 m。N03 的 matched-off 为 `21.476`：repair retry 4 在 28.88 s 后仍碰撞并回退；retry 8 在 31.17 s 得到安全候选 `19.540826`，端到端 `19.541`、降低 9.02%；retry 16 得到同分同候选，耗时 31.67 s，没有相对 8 次的收益。5 mm timed swept-SAT 都通过。该排序切面方式缩短了修复阶段，但低于 8 次目前没有复现收益。

N05 matched-off 为 `23.667`。20 s、random seed 1、feasible warm start 开启时，warm start 成功用时 2.50 s，但 Bonmin 20.92 s 后 `LIMIT_EXCEEDED` 且无终端 incumbent，最终安全 score `23.529`，耗时 28.59 s。把总限时增加至 30 s、seed 0 后，Bonmin `SUCCESS`（19.24 s）并产生 `23.666639` 的 `101` mode-run 候选；最终 `23.667` 与 off 持平，耗时 23.90 s，5 mm sweep 通过（3,334 samples，位置/航向误差 `0.00949 m / 0.02000 rad`）。同一次优化的局部交替候选 score `22.991710` 更低，但被 `safe_time_sample` 以 `timed_sat_collision_after_adaptive_sampling` 拒绝。由此 N05 不再只是“整数 incumbent 随机缺失”：低分连续候选也可能因时间重采样 swept safety 失败而失去收益。保留严格安全接受条件，下一轮需提高连续 refinement 的时域净空裕度。

seed 1 的首个诊断探针误漏开 warm start（`N05_global_minlp_ranked_planes_seed1_probe.json`），只用于记录 no-warm 诊断，不作为配对结果；匹配的 warm-start 重跑在 `N05_global_minlp_ranked_planes_seed1_warm_probe.json`。本轮其他原始数据为 `N03_global_minlp_ranked_planes_sample10mm_retry{4,8,16}_probe.json` 与 `N05_global_minlp_ranked_planes_time30_probe.json`，完整 ledger 在 `relust/global_minlp_20261005_paired_probes.json`。

配置已恢复默认 `joint.enabled:false,time_limit:1.5,random_seed:0,feasible_seed_warm_start:false`；隔离 ROS master 11324 已关，11311/11312/11320 未触碰。下一步保持 N03 ranked-cut 方案，修复 N05 time-resampled trajectory 的 clearance robustness，并用 matched-off 重新核验 N05 和 N03；在跨图可重复收益出现前，不启用默认 MINLP。


## 13. 2026-10-05 N05 timed-sweep 候选恢复器试验

N05 30 s / seed 0 / warm-start 配置的两次复跑分叉：一次 Bonmin `SUCCESS`、最终 `23.667`；另一次没有 terminal incumbent、最终 `23.529`、约 37 s。成功运行曾有 `22.991710` 的 `iterated_joint` 候选，但它因 `timed_sat_collision_after_adaptive_sampling` 被严格 swept 检查拒绝。新增一个只在该类 timed-sweep 拒绝后运行的固定 mode-sequence IPOPT 恢复器，逐步尝试 1、2.5、5、10 mm 额外 vehicle margin；候选需对原始 margin 再过精确 timed sweep，且分数低于其余安全候选。恢复 NLP 可选择允许输入 seed 轻微违反额外 margin，以免在入口校验处直接放弃，但这不改变最终验收条件。

首个 5/10 mm 版本在进入 IPOPT 前发现加严 margin 下第 14 段 seed 已碰撞。允许 infeasible initial seed 并加入 1/2.5 mm 细级别后，两次 30 s N05 重跑都没有产生可供该分支修复的低分 MINLP 提案，最后安全 fallback `23.529`；因此新修复器效果尚未验证，也可能增加 opt-in 的最坏时延。不能据此宣称 N05 改善。日志与探针：`N05_global_minlp_ranked_planes_time30_clearance_repair_probe.json`、`N05_global_minlp_ranked_planes_time30_clearance_fine_probe.json`、`N05_global_minlp_ranked_planes_time30_clearance_unsafe_seed_probe.json`。

在跨图重复收益和 repair 分支成功之前保持 `joint.enabled:false`。源码已通过 `py_compile` 与 `catkin_make -j4`；所有本轮 ROS 实验使用隔离 master 11324，参数恢复默认，11311/11312/11320 未触碰。


## 14. 2026-10-05 N03 同配置复验波动

当前代码下再次运行 N03：`time_limit=15 s`、seed 0、warm start、SAT repair retry 8 / 6 s。结果走 `joint_seed_fallback`，score `20.826`、耗时 `66.86 s`；位置/航向误差 `0.00760 m / 0.02000 rad`，独立 5 mm sweep 通过（3,006 samples）。此前 ranked-cut retry 8/16 两次为 `19.541`、约 31 s；新结果没有复现该收益，并出现较长时延。故当前 N03 有两次强结果和一次明显较弱 fallback，不能称为稳定 9% 提升。探针为 `N03_global_minlp_ranked_planes_sample10mm_retry8_clearance_patch_probe.json`。

继续保持 `joint.enabled:false`。下一步优先解决 Bonmin terminal incumbent 的可重复获取及 wall-time 长尾，再单独验证 N05 timed-clearance recovery；不要通过扩大候选门槛或放宽 swept-SAT 接受条件制造收益。完整 recheck 写入 `relust/global_minlp_20261005_paired_probes.json`。


## 15. 2026-10-05 Bonmin 节点预算与 N05 time-clearance 复验

本轮在 `core.py` 接入 `bonmin_subproblem_iteration_limit` 参数（`params.yaml` 默认 600），并限制无 Bonmin terminal incumbent 时的 seed-schedule fallback SAT 重试数为 `min(4, collision_repair_iterations)`。N03 的 matched-off 为 `21.476`；15 s Bonmin 限时、每节点 240 次迭代、warm-start 开启时，两次均由固定 schedule + SAT/IPOPT 修复得到 `19.541`，墙钟时间 `33.43/33.32 s`，独立 5 mm timed swept-SAT 均通过（各 3,138 samples），改善 9.02%。每节点 120 次没有 incumbent；240 次下将全局限时降到 12 s 也失去该候选，回到 `21.476`。因此 N03 当前观察到的点是 240/15 s，但仍是 opt-in 且显著慢于 off。

N05 原起终点 matched-off `23.667`。fractional dive 开启、每节点 600 次、45 s 上限、warm-start 开启、seed 0 的五次结果均得到 Bonmin `SUCCESS`/`101` 模式候选；端到端分数 `23.045, 23.667, 23.063, 23.208, 23.210`，4/5 低于 off（1.93–2.63%），一次持平，全部通过独立 5 mm sweep。35 s 和 40 s 限时都未得到整数 incumbent，安全回退到 `23.529`；总 wall 分别 `42.67/44.59 s`。每节点 240 次、30 s 和每节点 600 次、20 s 也没有终端 incumbent，均得到 `23.529`。N05 global 收益现在在相同 seed0/45 s 配置下多次出现，但还不足以降低 33–46 s 的规划延迟。

为解决 `22.991710` 低分局部候选的时间扫掠碰撞，恢复分支将额外 margin 改成 1.0–2.5 mm、0.25 mm 步进，每次 IPOPT 至少 300 次迭代；最终仍只接受通过原始碰撞 margin 下完整 timed swept-SAT 的轨迹。seed 1 一次运行在 1.25 mm 额外 margin 下接受 `22.884203`，相对 off 低 3.31%，时间 40.74 s、独立 5 mm sweep 3,341 samples 通过；同配置复跑所有恢复档位失败并回到 `23.667`。之后增加了从同 mode-run Bonmin 安全几何出发的备用恢复 seed，但对应试验没有同时满足“出现 Bonmin incumbent且局部候选触发恢复”的条件，尚未验证其效果。

N05 将局部 alternating iterations 从 2 增加到 4 的 matched-off 仍为 `23.529`、2.59 s；global 为 `23.210`、45.18 s。日志显示低增益早停仍发生在第 2 轮，增加轮数没有收益。

**当前判断：** 全局 MINLP 接入已在 N03/N01/N05 多图上产生安全端到端改善证据；N03 约 9% 且重复两次，N05 seed0/45 s 为 4/5 改善、1/5 持平，最好一次净空恢复为 3.31% 但未重复。不能把最佳单次 N05 得分当成稳定结果，也不打开默认开关。下轮优先稳定 incumbent 与 timed-clearance 恢复、压缩 33–46 s wall，而不是继续降低 safety gate。全部原始探针和逐次配置记录在 `relust/global_minlp_20261005_paired_probes.json`。

ROS 探针均使用隔离 master 11324，结束后已关闭；`params.yaml` 已恢复默认 `joint.enabled:false,time_limit:1.5,random_seed:0,dive_fractional:false,feasible_seed_warm_start:false,alternating_iterations:2`。用户已有 ROS masters 11311/11312/11320 未触碰。


## 16. 2026-10-05 N05 Bonmin 单节点 wall cap 优化

固定 N05 matched-off `23.667`、seed 0、warm start、fractional dive、每节点 IPOPT 迭代上限 240、Bonmin 总时限 45 s 和修复预算 16/6 s，比较单个节点 NLP 的 wall cap：

| 单节点 NLP wall cap | Bonmin | Bonmin solve | 最终 score | 总 wall | 5 mm timed sweep |
|---:|---|---:|---:|---:|---|
| 4 s | `SUCCESS`，候选 `23.666639`，mode-run `101` | 40.89 s | 22.984 | 45.10 s | 通过，3,330 samples |
| 2 s，试验 1 | `SUCCESS`，候选 `23.666639`，mode-run `101` | 28.68 s | 22.984 | 32.78 s | 通过，3,330 samples |
| 2 s，重复 | `SUCCESS`，候选 `23.666639`，mode-run `101` | 27.54 s | 22.984 | 31.96 s | 通过，3,330 samples |

两次 2 s 运行都由后续 `iterated_joint` 阶段将全局候选从 `23.666639` 降至 `22.984`，相较 matched-off 改善 2.89%；相较每节点 4 s，端到端耗时下降约 27–29%。把全局 Bonmin 限时压到 40 s（节点 wall cap 4 s）时，没有 terminal incumbent，Bonmin 耗时 38.81 s，最终回退至 `23.529`，总 wall 46.55 s。因此局部节点 wall cap 2 s 是当前 N05 上重复成功的更快配置；40 s 的 nominal global cap 仍不能可靠地确保提前拿到可用 incumbent。

这是 N05 的两次重复结果，仍需在 N03/N01 或其他场景配对验证，不能直接推广为全局默认。`joint.enabled` 继续关闭。ROS 探针只使用隔离 master 11324，结束后已关闭；`params.yaml` 已恢复 `enabled:false,time_limit:1.5,subproblem_time_limit:4.0,bonmin_subproblem_iteration_limit:600,random_seed:0,dive_fractional:false,feasible_seed_warm_start:false,collision_repair_iterations:12,collision_repair_time_limit:2.0,alternating_iterations:2`。逐次 JSON 在 `relust/global_minlp_20261005_paired_probes.json`，原始探针为 `N05_global_minlp_nodeiter240_dive_time45_probe.json`、`N05_global_minlp_node240_dive45_subproblem2_probe.json`、`N05_global_minlp_node240_dive45_subproblem2_repeat_probe.json` 和 `N05_global_minlp_nodeiter240_dive_time40_probe.json`。


## 17. N03 单节点 NLP 限时边界复验

在 N03 原始起终点、15 s Bonmin 总限时、每节点迭代上限 240、warm-start 参数开启下，逐档下调单节点 NLP wall cap：

| 单节点 cap | Bonmin/安全结果 | final score | wall | 独立 5 mm sweep |
|---:|---|---:|---:|---|
| 2 s | 无终端 schedule；seed schedule 第 2 段碰撞，回退 | 21.476 | 33.78 s | 通过，3,026 samples |
| 3 s | 无终端 schedule；同一 seed 段碰撞，回退 | 21.476 | 40.79 s | 通过，3,026 samples |
| 3.5 s | 无终端 schedule；同一 seed 段碰撞，回退 | 21.476 | 36.43 s | 通过，3,026 samples |
| 3.75 s | 返回 `LIMIT_EXCEEDED` mode schedule；固定 schedule 修复 `19.540826`，随后 timed-clearance repair 接受 `19.526901` | 19.527 | 45.10 s | 通过，3,140 samples |
| 4 s（此前两次） | 固定 schedule + SAT/IPOPT 修复，重复 `19.541` | 19.541 | 33.43/33.32 s | 通过，各 3,138 samples |

N03 的可用 schedule 门槛位于 3.5–4 s 之间。3.75 s 虽比 matched-off `21.476` 降 9.08%，但局部低分候选 `19.359789` 被严格 timed swept 检查拒绝，time-clearance 恢复耗时 14.58 s，整体比 4 s 配置慢约 12 s，且最终收益只多约 0.06 个百分点。N05 的 2 s cap 两次均成功、缩短到约 32 s；N03 的 2/3/3.5 s 档均无 terminal schedule。由此确认应按 MINLP 子问题实际维度/难度继续校准，而不能把 N05 参数直接设为全局通用值。未改默认配置，`joint.enabled:false`。

相关原始探针：`N03_global_minlp_node240_subproblem2_probe.json`、`N03_global_minlp_node240_subproblem3_probe.json`、`N03_global_minlp_node240_subproblem3_5_probe.json`、`N03_global_minlp_node240_subproblem3_75_probe.json`。配置现已恢复默认；隔离 master 11324 关闭，11311/11312/11320 未触碰。


## 18. 单模式 seed 的 Bonmin 节点限时自适应试验

基于当前探针添加 opt-in 参数 `joint/subproblem_time_adaptive`，默认 `false`。开启后，若规划器 seed 没有任何模式切换，Bonmin 单节点 NLP wall cap 使用 `min(configured_limit, 2.0 s)`；seed 已含模式切换则继续用配置 cap。日志打印有效 cap、seed mode switch 数和开关值。该判断仅依赖输入轨迹的模式切换结构，不匹配地图名称；全局 MINLP 默认开关仍为 false。

| 地图 | 输入 seed | 有效节点 cap | Bonmin / final | wall | 独立 5 mm sweep |
|---|---|---:|---|---:|---|
| N01，显式固定 cap 对照 | 13 primitives，0 次切换 | 2 s | 无 terminal incumbent；fallback `31.450815` 后 final `31.098`，相对 off `31.960` 改善 2.70% | 23.67 s（此前 cap 4 s/总预算20 s 为25.75 s） | 通过，3,701 samples |
| N05，自适应开启 | 7 primitives，0 次切换 | 2 s | `SUCCESS`/`101`，Bonmin seed `23.666639`，final `23.080`，较 off `23.667` 改善 2.48% | 43.18 s | 通过，3,345 samples |
| N03，自适应开启 | 10 primitives，2 次切换 | 4 s | `LIMIT_EXCEEDED` 但返回 mode schedule；固定 schedule repair `19.540826`，time-clearance repair final `19.526901`，较 off `21.476` 改善 9.08% | 45.51 s | 通过，3,140 samples |

自适应分支在 N05/N03 日志中分别明确打印 `node_nlp_wall_cap=2.00s/4.00s`，说明 cap 选择符合规则。结果仍有波动：N05 早先显式 cap2 两次得到 `22.984`、31.96/32.78 s；本次自适应为 `23.080`、43.18 s。N03 本次命中 timed clearance recovery（14.65 s），比此前 4 s cap 的两次 `19.541`/约33.4 s 慢。故该策略尚未证明能稳定降低 wall，也不是全局参数默认值；保留为 `subproblem_time_adaptive:false`，继续需要更多跨图重复和对 timed recovery 的效率优化。

探针：`N01_global_minlp_node240_subproblem2_probe.json`、`N05_global_minlp_adaptive_nodecap_probe.json`、`N03_global_minlp_adaptive_nodecap_probe.json`。源码在 `optimizer_node.py`；`params.yaml` 默认仍是 `joint.enabled:false,time_limit:1.5,subproblem_time_limit:4.0,subproblem_time_adaptive:false,bonmin_subproblem_iteration_limit:600,random_seed:0,dive_fractional:false,feasible_seed_warm_start:false`。ROS 只使用已关闭的隔离 master 11324；11311/11312/11320 未触碰。


## 19. 2026-10-05 timed-clearance alternate geometry 直接探测

针对 N03 的恢复阶段连续尝试多个相邻余量档造成的延迟，在 optimizer_node.py 调整为：同模式序列的 Bonmin swept-safe 几何优先只探测 2.25/2.5 mm；若两档失败，再从低分 iterated_joint 几何运行完整 1–10 mm 余量梯度。最终分数仍须优于其余安全候选，且通过原始 occupancy grid 的 timed swept-SAT。

N03 配置固定为总限时 15 s、节点 wall cap 4 s、每节点迭代 240、warm-start、随机 seed 0、collision repair 16/6 s。首个探针沿用 Bonmin 的同模式修复几何，在 2.25 mm 首档接受 19.526901，相较 matched-off 21.476 改善 9.08%；总 wall 32.95 s，timed-check 1.02 s，较此前相同分数但需 6.09–7.53 s timed-check 的运行更快，独立 5 mm sweep 3,140 samples 通过。立即同配置复跑 Bonmin LIMIT_EXCEEDED 且无 terminal schedule，最终安全回退 21.476，总 wall 32.67 s、5 mm sweep 3,026 samples 通过。这说明优化器分支的恢复成本降低，但 Bonmin incumbent 波动尚未解决。

N05 seed 1、总限时 45 s、节点 cap 2 s、每节点迭代 240、fractional dive 与 warm-start 下，Bonmin SUCCESS/101，最终 23.086，相较 matched-off 23.667 改善 2.46%，总 wall 28.21 s，独立 5 mm sweep 3,351 samples 通过。该运行没有触发本次改动的 timed-clearance 恢复分支。

构建与 py_compile 均通过；参数文件恢复默认 joint.enabled:false,time_limit:1.5,subproblem_time_limit:4.0,bonmin_subproblem_iteration_limit:600,random_seed:0,dive_fractional:false,feasible_seed_warm_start:false,collision_repair_iterations:12,collision_repair_time_limit:2.0,alternating_iterations:2。隔离 ROS master 11324 已关闭，11311/11312/11320 仍运行且未触碰。下一轮优先压低 Bonmin 无可用 mode schedule 的发生率与 SAT 修复耗时；全局 MINLP 暂不设为默认。


## 20. 2026-10-06 转向自适应路线锚点与上游 S-O-S 后缀重规划

路线图锚点现在按原始 2D A* 折线路线的局部转向变化自适应：查看前后约 `0.45 m` 的方向变化，达到 `0.80 rad` 时用 `0.30 m` 间距，其余位置用 `0.50 m`。锚点图边长度仍限制在 `3.0 m`，车辆扫掠碰撞检查不变。此阈值只控制锚点加密；A/O 切换代价用独立的 `cost/switch_turn_threshold=0.35 rad` 和 `cost/w_low_curvature_switch=0.35`。检查时发现两处原先共用了路线加密阈值，导致抬高加密阈值也同时放松切换惩罚；现已修正为各自读取独立参数。这个间距是路线图采样，不是 NLP 积分步长或车辆原语长度。

保留上游 S-O-S 重规划原型：扫描模式/档位变化、Spin 与换模式相邻、以及转向量集中的热点；在热点上游找有空间的位置 P，将 P 精确切分到已有动作内，尝试 `Spin(P) → Oblique → Spin(Q)`，再从 Q 重搜后缀。复杂度比较现在使用 Q 后等长的旧/新后缀，避免把宏动作本身的切换也算进后缀复杂度。若新后缀有可测的简化，总代价最多可放宽 `1.1`；没有简化时必须直接降低共同代价。所有候选还须满足终点约束并通过整条轨迹 swept-OBB 检查。配置在 `endpoint_replan/sos_*`。

| 场景与配置 | 粗搜索 | 连续最终解 | 安全/末端 |
|---|---|---|---|
| N04，旧记录 | `0.370 s`，route cost `39.494`，8 次 A/O 切换 | score `35.390`，总优化 `5.46 s` | 独立 5 mm sweep 通过（4,938 样本）；末端误差 `0.0473 m / 0.0829 rad` |
| N04，当前：`0.30/0.50 m`，加密阈值 `0.80 rad` | `0.378 s`，route cost `39.504`，锚点 `36/35/45/50`，仍为 8 次 A/O 切换 | score `31.317`，35 nodes，总优化 `4.87 s` | 独立 5 mm sweep 通过（4,898 样本）；末端误差 `0.0756 m / 0.1334 rad`，均在容差内 |
| N06，旧记录 | `0.271 s`，route cost `21.183`，4 次 A/O 切换 | score `14.322`，总优化 `2.75 s` | 旧记录独立 5 mm sweep 通过 |
| N06，当前：`0.30/0.50 m`，加密阈值 `0.80 rad` | `0.273 s`，route cost `21.763`，锚点 `25/25/27/28`，4 次 A/O 切换 | score `15.288`，总优化 `2.18 s`，优化结果 1 次模式切换 | 独立 5 mm sweep 通过（2,997 样本）；末端误差 `0.1494 m / 0.0748 rad`，均在容差内 |

N04 最终分数比旧记录低约 11.5%，总优化时间略短；N06 总规划时间少约 21%，但 score 比旧记录高约 6.7%。N06 新目标多了 `0.35` 的低曲率切换代价；去掉该项后约为 `14.938`，仍比旧分数高约 4.3%。因此目前可确认 N04 有收益、N06 时间缩短但几何/代价未完全恢复，不能概括成所有地图都提升。两图的粗搜耗时变化都很小（约 `0.25–0.37 s`），固定 `0.5 m` 改自适应间距没有证明是系统总耗时的主瓶颈；NLP 和后续锚点优化占了大部分端到端时间。

对 N06 的消融曾观察到：`0.30/0.70 m` 的稀疏间距把最终 score 从旧记录附近推高到约 `19`；将切换惩罚置零变化很小，改回固定 `0.50 m` 后 score 回到 `14.413`。这表明主要退化来自锚点布局改变了离散路径/候选，而不是切换惩罚本身。当前把直线路段稀疏间距收至 `0.50 m`，只在明显急转弯加密至 `0.30 m`，并修正加密阈值与切换惩罚阈值的耦合。这个保守设置保住了 N06 结果附近，并在 N04 上取得较低分。

S-O-S 在 N04 尝试 2 个上游宏动作（1 次后缀搜索成功），其 Q 后等长窗口的最佳动作数与转向量下降均为 0，所以候选被复杂度筛选拒绝；还没走到全路径碰撞检查。N06 本次没有热点触发。两图都没有接受 S-O-S 候选，说明“提前调姿态再重规划后缀”已接入，但真实地图收益仍未证实。合成空地图能生成安全候选，只证明生成器可工作。探针：`relust/N04_turn_adaptive_sos_probe.json`、`relust/N06_turn_adaptive_probe.json`。

本轮 C++ 构建通过：`source /opt/ros/noetic/setup.bash && catkin_make -C catkin_ws --pkg fwis_v3_planner fwis_v3_optimizer`。N04/N06 在隔离 ROS master 上实跑，独立 swept 检查通过；隔离 master 11338 已关闭，11311/11312/11320 未触碰。全局 MINLP 仍为 `joint.enabled:false`。

下一轮先在同一地图上保留“固定 `0.50 m` 锚点图”和“急转弯加密图”两种离散候选，让它们以同一 cost 和 swept-SAT 竞争，避免单一锚点布局丢掉质量较高的路线；同时检查 N06 的末端位置误差余量。之后再用确有揉库/大航向变化的场景改善 S-O-S 热点触发和上游姿态候选。


## 21. 2026-10-06 锚点重复计算与全局 MINLP 种子顺序复验

### 粗解图搜索

固定基础图与转向自适应图仍作为两套独立候选；若两套 waypoint 的坐标逐点相同，跳过第二次相同 DP 和混合图搜索，并在 `route2d` 诊断中记录 `duplicate_waypoint_variants`。N03 当前粗搜产生 4 条 clearance 路线，其中 3 条的两套 waypoint 完全相同；跳重后 route-DP 为 `0.168–0.175 s`，整个 route2d 为 `0.282–0.289 s`，mixed graph 调用从重复执行降为 2 次。当前最优安全连续轨迹为 `22.176`，5 mm timed swept 检查通过，末端误差 `0.0372 m / 0.0582 rad`。这次改动省掉了重复离散计算，不改变独立候选的择优规则。

以默认 `joint.enabled:false` 在 N06 实跑复验：4 条 clearance 路线中跳过 2 张重复图，route2d 为 `0.268 s`、route-DP 为 `0.173 s`；最终仍为 `15.288`，总优化 `2.10 s`，5 mm swept 2,997 个样本通过，末端误差 `0.1494 m / 0.0748 rad`。去重没有改变 N06 的路径与 score，确认它只省掉重复搜索；N06 相对旧 `14.322` 的质量退化仍要单独处理。探针在 `relust/N06_dedup_current_default_probe.json`。

N04/N06 的已记录效果仍有差异：N04 的转向自适应候选为 `31.317`，比旧记录 `35.390` 低约 11.5%；N06 为 `15.288`，比旧记录 `14.322` 高约 6.7%（去除新增的 `0.35` 低曲率切换项后约 `14.938`）。所以转向加密与双图保底还没有在所有场景同时恢复旧质量。

试过把基础 waypoint 改为沿路径按精确弧长插值，目的是避免从栅格路径上越过 `0.50 m` 阈值后取折点造成间距漂移。N04 最终 score 变为 `39.807`，优化总时长 `20.95 s`，其中 timed-check 长尾 `15.48 s`；虽然 5 mm sweep 安全，但质量和时间都退化。该采样版本已撤回，保留此前验证过的栅格折点锚点。实验记录为 `relust/N04_exact_arc_sampling_rejected_probe.json`。

### 全局 MINLP

为试验用更强连续轨迹给 Bonmin 做 seed，曾把全局 MINLP 移到固定 NLP、锚点图和局部模式/几何迭代之后，并尝试将最佳候选投影到 MINLP 的 `0.015 m / 0.02 rad` 末端约束。N05 的候选末端误差约 `0.0577 m / 0 rad`，同模式紧投影未通过整轨 swept/末端验收；N03 的局部候选为 `0.0372 m / 0.0582 rad`，也无法在原模式序列下投影到根节点精度。直接传入局部候选时，MINLP 在建根初值阶段报 `joint_initial_infeasible`；回到粗 seed 的补试仍出现 `INFEASIBLE` 或无可接受 terminal schedule。N03 的后置 MINLP 分支最终分别回到 `22.176`，耗时 `21.54 s` 和 `45.83 s`。这说明当前 NLP 的普通规划容差解不能自动充当严格 MINLP root seed。

恢复全局 MINLP 在固定 NLP/锚点优化之前读取粗搜索 seed 的调用顺序：粗 route seed 保留模式切换结构，局部候选在其后作为安全竞争解。以 N03 当前粗 seed 复验的一次运行中，Bonmin 在 `LIMIT_EXCEEDED` 下仍返回 mode schedule，经固定 schedule/SAT 修复得到 `19.7468`，time-clearance 修复后最终 `19.725`；相较历史 matched-off `21.476` 低约 8.2%，总时长 `55.42 s`，独立 5 mm sweep 通过。相同方向的 N05 探针最终为 `23.770`、`35.23 s`，比历史 off `23.667` 略差，Bonmin 未形成可接受 incumbent。结合 N03 的回退复验，当前只能把 N03 的低分视为一次可用结果，不能据此推广 MINLP 收益或默认启用。

相关探针：`relust/N03_global_minlp_fixed_tight_seed_probe.json`、`relust/N03_global_minlp_best_continuous_seed_probe.json`、`relust/N03_global_minlp_best_seed_retry_probe.json`、`relust/N03_global_minlp_bestconfig_postlocal_probe.json`、`relust/N05_global_minlp_fixed_tight_seed_probe.json`。后置最优连续 seed 的 dispatch 方案已经从代码中撤回；当前 MINLP 保持粗 seed 入口，且 `joint.enabled:false`。

本轮 `catkin_make -C catkin_ws --pkg fwis_v3_planner fwis_v3_optimizer` 与 Python `py_compile` 均通过。N03/N04/N05 ROS 探针只使用隔离 master `11343`，已停止；`11311/11312/11320` 未触碰。`params.yaml` 已恢复默认 `joint.enabled:false,time_limit:1.5,subproblem_time_limit:4.0,bonmin_subproblem_iteration_limit:600,dive_fractional:false,feasible_seed_warm_start:false`。

下一步粗解侧先用同一 N06 起终点和参数做配对：保留旧基础图作为硬兜底，只调急转弯触发和加密范围，防止 N04 收益掩盖 N06 退化。全局 MINLP 侧的首要问题是构造同时满足紧末端约束与整车 swept 安全的混合模式 seed；先评估离散阶段输出的其他安全模式候选是否有可行投影，再让 MINLP 改动模式和几何，而不是直接拿普通容差的连续最优解强塞给 root。


## 25. 2026-10-06 行驶方向硬约束与 A-first 模式策略

### 规划规则

用户明确要求禁止“前进—倒车—再前进”的揉库行为，并要求能由 A 模式通过的路径优先使用 A；O 模式只有在 A/Spin 搜索未找到可行路径时才开放。此前仅给反向距离 `0.25`、方向切换 `0.25`、O 角度总变化 `0.35/rad` 和事件 `0.04` 的软代价，规划器可以用少量分数换取频繁换向与 O 转角调整，这是“可通行但动作奇怪”的直接原因。

现改成两层规则：

1. 一条路线一旦开始平移，所有 A/O 平移段必须保持同一 gear。粗搜索的主格点、稀疏 A/O/S 图、route-waypoint DP、Python bounded replan 和 MINLP 都执行该约束；Spin 不改变 gear。候选验收再检查一次整条动作序列，过滤任何 gear 来回切换。粗搜索里专门构造的倒车入位终端动作已删除。纯倒车路线仍可能作为没有前进解时的候选，但反向距离附加代价改为 `1.0/m`。
2. route-waypoint 图额外先搜索 A+Spin 子图。若得到经过整车扫掠检查的可行路线，主格点搜索与后续优化都禁止引入 O；若该子图没有候选，才运行完整 A/O/Spin 搜索。这个策略按路线可行性切换，不看地图名、不含任何逐地图状态机。

同时将共享模式切换代价调为 `1.00`，O 角度总变化调为 `1.50/rad`，每次明显 O 角度调整事件调为 `0.80`，O 使用/低角度巡航附加代价分别为 `0.85/m`、`0.80/m`。A 转向仍通过曲率变化表达，留在 A 子图里的频繁转向不会触发 O 的角度变化惩罚。全局 MINLP 继续默认关闭；这些规则也约束其可行域。

### 跨场景结果

| 场景 | A+Spin 图候选 | 最终模式与共同分数 | 方向 / O 转向 | 独立 5 mm swept-SAT |
|---|---|---|---|---|
| N03 | 找到 | A+Spin，`17.331`；此前本轮首版混合路线为 `19.744` | 0 次换向；最终无 O | 通过，3,384 samples；末端误差 `0.1104 m / 0.0745 rad` |
| N01 | 未找到，开放 O 回退 | O+Spin，`44.392` | 0 次换向；角度调整沿长段分布 | 通过，4,360 samples；末端误差 `0.0596 m / 0.0384 rad` |
| N05 | 未找到，开放 O 回退 | O，`32.971` | 0 次换向；最终 O updates 为 3，`<=5°` 微调为 0 | 通过，3,206 samples；末端误差 `0.0697 m / 0 rad` |
| P12_env2，运行比例 0.8 | 找到 | A，`10.384` | 0 次换向；无 O | 通过，2,293 samples；末端误差 `0.1574 m / 0.0384 rad` |

P12_env2 的起终点按用户截图方向估取（`(10.0,3.8,π/2)` 到 `(4.0,7.0,π)`），属于算法行为探针，不是既定论文基准。N01/N05 的“A+Spin 图未找到路线”也只是当前稀疏 waypoint/候选图内未找到，不是对连续几何问题无解的数学证明；回退 O 后路径仍必须通过同一 swept-SAT。

N03 的 A-first 路线在 `max_iter_with_incumbent=1500` 下重复得到相同 `25.153` 粗分数和 `17.331` 最终分数。临时将该搜索预算提至 10,000 次没有改变 N05 的 7 段 O 路线，因此参数已恢复 `1500`。`joint.enabled:false` 与 `root_resolves:0` 均未改。

验证：`catkin_make -C catkin_ws --pkg fwis_v3_planner fwis_v3_optimizer -j4`、Python `py_compile` 通过；N01/N03/N05/P12_env2 的运行探针均得到 swept-safe timed trajectory。原有 ROS master `11311/11312/11320` 未触碰；本轮隔离 master `11343` 已关闭。

原始探针：`relust/behavior_policy_20261006_N03_final.json`、`relust/behavior_guard_20261006_N01_ack_first.json`、`relust/behavior_guard_20261006_N05_ack_first.json`、`relust/behavior_guard_20261006_P12_env2.json`。预算消融和首次行为结果另见同前缀 `behavior_guard_20261006_*.json`。

下一步：用相同 A-first 策略复验 P11_case_b/P11_case_c/P12_env1/P12_env3 的正式起终点，特别检查窄场景是否因禁用换向而失去可行性；再把 O steering 调整次数作为模式图的独立指标，确认 A/O 混合场景中 O 只留在 A/Spin 子图确实无法覆盖的局部段。保持无地图编号分支。


## 22. 2026-10-06 MINLP 初值门槛与无整数解回退复查

### 查到的问题

`joint_refine()` 原先在建模后检查初值约束残差；只要粗 seed 有一项超过 `1e-4`，就直接抛 `joint_initial_infeasible`，Bonmin 完全没有机会修正末端或走廊变量。另一个问题是，即使初值不可行，代码仍用它的目标值设置 Bonmin `cutoff`，相当于拿一个不可行点限制可行解搜索。

修复后，残差只记录为诊断；只有 warm-start 已提供可行解或初值本身满足模型约束时，才设置 seed cutoff。输出仍须满足紧末端约束、精确整车 swept 检查和最终 timed swept 检查。日志把根节点违反项单独保留，避免固定模式修复器的局部残差覆盖根节点信息。

N03 复验中 warm-start 投影失败后，Bonmin 仍启动并返回了可修复的 mode schedule。固定模式与 SAT/IPOPT 修复分数为 `19.540826`，随后 time-clearance 恢复最终得 `19.526901`；相较 matched-off `21.476` 低约 `9.1%`，总耗时 `32.05 s`，独立 5 mm swept 检查通过（3,140 samples）。这确认原来的初值门槛确实能拦掉有用的搜索。

### 仍在阻止全局提升的问题

N05 同配置出现了另一种失败：Bonmin 根初值可行，约束违反数为 0、最大数值残差 `3.85e-8`，warm-start 已开启，但 B-BB 仍返回 `INFEASIBLE`。同一配置关闭 seed cutoff 后仍是 `INFEASIBLE`，说明这次不是 cutoff 导致。Bonmin 没有带来路径收益：最终 `23.770` 与当前代码的 MINLP-off 配对分数 `23.770` 相同；开 MINLP 总耗时约 `32.07 s`，关闭时为 `1.71 s`。

代码原先会丢弃已经通过末端和 swept 检查的固定模式 warm-start，再从粗 seed 重跑同一模式。现在 Bonmin 无终端整数候选时直接保留已验证的 warm-start。N05 总耗时从重复 fallback 前约 `35.90 s` 降至 `32.07 s`；固定模式 warm-start 候选为 `23.838554`，共同打分更低的局部解 `23.770` 仍胜出。这降低了失败成本，没有改变该图的最终质量。

另外试了 Bonmin `B-iFP`：120 s 探针期限内没有返回终态，结束进程时 Bonmin 抛出 `TNLPSolver::UnsolvedError`，没有可用路径。当前模型含三角运动学与整数模式耦合，属于非凸 MINLP；Bonmin 文档说明非凸情形下算法是启发式，而当前 B-BB 配置会在 NLP 子问题未解出时 fathom 节点。因此 `INFEASIBLE` 需结合节点 NLP 日志理解，不能直接当成已证明的几何无解。下一步优先解决可行模式解如何作为真实 incumbent 保留，以及根 NLP 失败时的分支恢复，再调整 mode 搜索。Bonmin 官方说明见 [options list](https://coin-or.github.io/Bonmin/options_list.html) 与 [Bonmin overview](https://coin-or.github.io/Bonmin/)。

逐次结果：`relust/N03_minlp_initial_gate_fix_probe.json`、`relust/N05_minlp_initial_gate_fix_probe.json`、`relust/N05_minlp_initial_gate_nocutoff_probe.json`、`relust/N05_minlp_warm_fallback_probe.json`、`relust/N05_minlp_matched_off_after_fix_probe.json`、`relust/N05_minlp_bifp_probe.json`。汇总见 `relust/global_minlp_20261006_failure_analysis.json`。

本轮 `py_compile` 与 optimizer 包编译通过。所有试验配置使用 `/tmp` 临时 launch/YAML，仓库 `params.yaml` 保持 `joint.enabled:false`；隔离 ROS master `11343` 已关闭，原有 `11311/11312/11320` 未触碰。

## 23. 2026-10-06 cutoff 写入修正与 root-resolve 配对

### 新发现与修复

`joint_refine()` 的 CasADi 参数字典此前只会在 `cutoff_eligible` 时设置 `bonmin.cutoff`，但每次生成的临时 `bonmin.opt` 仍只看 `use_seed_cutoff`。当 warm-start 失败且根初值不可行时，第二条配置通路会把不可行粗解目标值传给 Bonmin，可能把可行空间错误剪掉。现将两条通路统一为 `use_seed_cutoff and cutoff_eligible`。本轮 N03 实验没有走到“warm-start 与根初值都不可行、配置开关仍开启”的分支，因此该特定分支尚待专门触发验证；语法检查通过。

### 求解器配对结果

- N05、root resolve 1、cutoff 关闭：Bonmin 从 `INFEASIBLE` 变为 `SUCCESS`，得到全 O 模式候选，分数 `23.973683`；候选进入当前安全筛选，最终局部解 `23.770` 胜出。Bonmin 约 `25.47 s`，端到端约 `31.25 s`，较 MINLP-off `1.71 s` 没有质量收益。最终发布轨迹通过独立 5 mm swept-SAT（3,670 samples）。数据：`N05_minlp_rootresolve1_probe.json`。
- N05、root resolve 1、cutoff 开启：Bonmin 仍返回 `INFEASIBLE`，使用可行 seed fallback `23.838554`，本地解 `23.770` 胜出；端到端约 `23.70 s`。该配对说明当前 cutoff 会挡住这组 N05 搜索里的整数候选，但尚未改善最终路径。数据：`N05_minlp_rootresolve1_cutoff_probe.json`。
- N03、root resolve 0、cutoff 开启复验：固定 schedule/SAT 修复 `19.746776`，最终 `19.725`，对照 matched-off `21.476` 改善约 `8.2%`，耗时 `36.11 s`，5 mm swept-SAT 通过（3,144 samples）。数据：`N03_minlp_cutoff_eligibility_fix_probe.json`。
- N03、root resolve 1、cutoff 关闭：Bonmin `LIMIT_EXCEEDED` 但仍返回同一可修复 A/O schedule；固定 schedule/SAT 修复 `19.540826`，time-clearance 后最终 `19.526901`，比 matched-off 改善约 `9.1%`，总耗时 `31.20 s`，5 mm swept-SAT 通过（3,140 samples）。Bonmin 阶段约 `11.33 s`。该结果与先前最好 N03 分数一致；root resolve / cutoff 的净收益仍需同一 warm-start 结果下重复配对。数据：`N03_minlp_rootresolve1_nocutoff_probe.json`。

### 当前判断与下一步

N03 的混合模式 MINLP 仍有可复现的明显路径收益；N05 单 O 路线没有收益且会多花约 20–30 秒。root resolve 在 N05 改变了 Bonmin 终态，在 N03 一次运行里也把可用 schedule 阶段压到约 11.3 秒，但 warm-start 成功与否会改变下游分支和总耗时，所以不能据此全局打开 root resolve 或 cutoff。继续工作应优先用 matched N03/N05 配置各重复一次，记录完整 schedule、Bonmin 状态与根可行性；全局 `joint.enabled:false` 保持不变。所有探针使用隔离 master `11343`，现已关闭；`11311/11312/11320` 未触碰。


## 24. 2026-10-06 通用锚点消融、S-O-S 评价窗口与 root-resolve 配对

### 粗路径锚点采样消融

本轮只对所有地图共享的转弯阈值和间距公式做消融，没有按地图名添加逻辑。把转弯加密阈值从 `0.80 rad` 降到 `0.60 rad` 后，N03 final `21.684`（原基线 `21.476`），N04 分数不变但 route 搜索由 `0.358 s` 增至 `0.603 s`，N05 只改善 `0.020` 分，N06 从 `15.288` 退化到 `16.377`。随后试了按转向需求在 `0.30–0.50 m` 间连续插值：N03 退化到 `22.176`，N04 不变但更慢，N05 仅改善 `0.062`，N06 退化到 `20.072` 且优化耗时升至 `14.63 s`。两种改法都撤回，恢复转弯阈值 `0.80 rad` 的通用稀疏/密集二档；planner 编译通过，配置未改。

### 上游 S-O-S 重规划评价窗口

原复杂度筛选只比较 Q 点之后的后缀，没有统计候选实际替换掉的 P–Q 原路径。现在用“原 P–Q 段剩余部分 + 下游窗口”与“S-O-S 宏动作 + 重规划后缀”比较模式/档位变化和同模式转向变化。N01 纯 O 路线的 5 个候选测得最佳局部转向变化减少 `1.462 rad`，但候选 score `34.130` 高于安全 incumbent `31.406` 达 `2.724`，超过 `1.1` 的统一 score slack，因此全部拒绝、最终轨迹不变；独立 5 mm sweep 通过（3,762 samples）。这修正了候选评价漏算替换段的问题，没有构成该地图的路径收益。N03 本轮未触发 S-O-S 候选，最终 `22.176`、5 mm sweep 通过（3,026 samples）。

### 全局 MINLP root-resolve 同配置对照

N03 固定粗 seed、`time_limit=15 s`、节点 wall cap `4 s`、节点迭代 `240`、cutoff 开启、无 warm-start、SAT repair `16 × 6 s`。root resolve `0` 时 Bonmin 返回不可投影的全 spin 点，最终安全局部候选 `22.176`，端到端 `35.14 s`；root resolve `1` 两次都返回同一 `O4/A5` schedule，固定 schedule 修复约 `19.541`，最终分别 `19.372` 和 `19.527`，耗时 `33.41 s` 和 `26.20 s`，5 mm timed swept-SAT 均通过。相对本轮相同 seed 的 root-resolve-0 对照，两个结果降低约 `12.6%` 和 `12.0%`。第二次的局部候选因 timed sweep 碰撞被拒，2.25 mm clearance 修复后得到 `19.527`。

root resolve `1` / cutoff 关闭的既有 N03 探针也得到 `19.527`；与本轮 cutoff 开启的重复结果相同，尚无 cutoff 带来稳定增益的证据。N05 root resolve `1` 的既有配对中，cutoff 关闭得到全 O 候选但局部 incumbent 胜出，cutoff 开启则 Bonmin `INFEASIBLE`；最终均为 `23.770`。因此 root resolve 对混合模式 N03 有明确候选可用性收益，对单 O N05 没有路径质量收益，`joint.enabled:false` 与 `root_resolves:0` 默认保持不变。

同时把 Bonmin 解出的模式/档位 schedule 写入所有直接结果及无 incumbent 回退的 metadata，避免只在 SAT repair 分支记录 schedule。验证命令：Python `py_compile` 和 optimizer catkin build 均通过。探针：`N01_sos_gate_before_probe.json`、`N01_sos_window_fix_probe.json`、`N03_sos_window_fix_probe.json`、`N03_minlp_root1_cutoff_on_sosdiag_probe.json`、`N03_minlp_root1_cutoff_on_sosdiag_repeat_probe.json`、`N03_minlp_root0_cutoff_on_sosdiag_probe.json`；采样消融探针以 `N03/N04/N05/N06_turn_threshold060_probe.json` 和 `N03/N04/N05/N06_continuous_turn_spacing_probe.json` 命名。

全局 MINLP 与 `root_resolves` 的配置均来自 `/tmp` 临时 YAML/launch；仓库默认 `joint.enabled:false`、`root_resolves:0` 未变。所有 ROS 实跑仅使用隔离 master `11343`，已停止；`11311/11312/11320` 未触碰。没有增加按地图编号分支或地图状态机。


## 26. 2026-10-07 qna 请求复查：禁倒车与 A/Spin 模式优先

### 请求中的现象与日志判断

- `qna/request.md` 提到 P11_case_a 绕行多转一次、P11 障碍边缘锯齿。多转来自离散锚点/可行性图给出的路线候选，并不证明那条路线是连续几何意义上的最短路径；既有数字化地图还残留了来源栅格的局部毛刺。本轮保留通用曲率采样，不加地图编号分支；数字化脚本已对障碍掩膜做中值清理和闭运算，并修整 P11_case_a 两个直边障碍面。
- P12_case_b 中“一种起终姿态成功、附近姿态失败”不能仅由截图判定为几何无解。旧 ROS 日志里有一组附近姿态以 `search_budget_exhausted iterations=180000` 结束，`route2d` 没有候选；这说明当前离散搜索/预算未找到路线，不是 endpoint 碰撞证明。该日志案例与截图编号的对应关系需要按实际起终点核对。
- 两条全倒车路线在旧日志中的倒车长度分别为 `18.5397 m` 和 `14.4818 m`。这是方向自由度被当成可用路线选择，而软倒车代价不足以阻止规划器选择它们。
- A-O-A 问题不只在粗规划：即使粗规划找到 A/Spin 路线，Python optimizer 的锚点重规划、SOS 与 MINLP 候选仍可重新引入 O。此前“粗解优先 A”没有贯穿候选最终筛选。

### 本轮策略修正

1. 粗规划中所有平移 gear 固定为正向；倒车入位终端动作、负向 O 段和负向 RS 段均不可生成。Python bounded replan、候选验收和全局 MINLP seed/变量约束继续执行同一规则。无正向解时返回搜索失败，不再输出纯倒车或揉库路线。
2. **已撤销的错误方案**：曾建议发现安全 A/Spin 路线后让 guided/fallback 禁用 O，并记录 `mode_policy=A_spin_only|mixed_fallback`。这会错误地排除能局部降曲率、少转向的 O。
3. **已撤销的错误方案**：曾建议 Python optimizer 继承粗解模式族、阻止 A/Spin seed 变成含 O 的路线。现在粗规划、SOS、锚点图与 MINLP 都让 A/O/Spin 通过同一代价竞争；见第 27 节。
4. planner、Python optimizer 与 Python 代价函数的 `w_switch` 默认统一为 `2.0`，降低无必要的模式切换；O 转角总变差与事件惩罚保持既有设置。

`tools/digitize_paper_maps.py` 的障碍边缘清理及生成地图已在前一轮完成。本轮 planner/optimizer catkin build 与 Python `py_compile` 均通过；未重启 ROS 或重放截图案例，因此场景效果仍待复验。下一步用请求中每张图的确切 start/goal 做隔离 ROS 配对：检查 P11_case_a 是否减少多余转弯、P12_case_b 的失败状态与 expansion 分布、倒车距离恒为零，并确认局部 O 能否降低 A 转向，同时减少 O 角反复调整。若相邻起始航向仍耗尽预算，应优先改进通用航向格点与扩展预算，不按地图名硬编码。


## 27. 2026-10-07 撤销 A-only 硬门控与 P12 地图调整

用户指出 A/Spin 可行时仍应允许有益的局部 O 段。第 26 节的 A-only 门控是错误策略，现已撤销：planner 将 A/Spin probe 作为普通可行候选，与混合模式候选按同一完整分数竞争；guided/fallback search 始终开放 O。Python optimizer、SOS、锚点图和 MINLP 的 O 候选也不再按粗路线是否含 O 过滤，统一按 `sequence_cost` 排序。无倒车约束继续保留。第 26 节中“探针发现 A/Spin 后关闭 O”的建议作废。

O 模式现在可以为减少 A 曲率/转向而在局部使用。共享评分与 NLP 对 O 的连续角变化计总变差，非零角度变化还收取事件费；恒定 O 角度不付事件费。O 的逐米使用费当前为 `w_oblique_use=0.55`、`w_oblique_cruise=0.45`，让 O 在局部转向收益足够时竞争，但避免它轻易替代整条 A/Spin 路线。A/O 模式切换代价仍为 `2.0`。

`run_demo.sh --map P12_env1` 默认全图比例为 `0.5`；`--map-scale` 仍可覆盖。用户此前附图的障碍布局与论文 P12_env4 obstacle-field 图一致，而 P12_env1 是方形迷宫。两图已从轴对齐障碍矩形重建边界；另将 P12_env1/P12_env4 占据栅格采样提高到 3 倍，不改地图实际米制范围，用于降低 RViz 放大时的格子感。

复验状态：上一轮 map generator、planner/optimizer catkin build 和 Python `py_compile` 均通过。下一步对 P12_env1 和附图对应的 P12_env4 用明确的中心起终位姿跑配对：检查 O 段是否因减少 A 转向而获选、O 控制角更新次数/总变差是否下降、无倒车、以及两张地图清理后的拓扑与细小障碍保留情况。

## 28. 2026-10-07 O 模式代价与 P12 栅格分辨率复调

用户进一步明确：O 可用于局部降低路径曲率/减少 Ackermann 转向；要压制的是连续使用 O 时反复改变侧移角。未增加模式族硬门控，所有模式仍通过共同目标函数竞争。试验将 `w_oblique_use/w_oblique_cruise` 从 `0.85/0.80` 降到 `0.25/0.15` 后，N03 从 A/Spin 变成全程 O，说明这组值压得过低；4°事件死区还使 N05 出现 13 次 O 角更新、11 次低于 5°，因此移除了死区并将逐米费用恢复到中间值 `0.55/0.45`。O 角总变差按 `w_oblique_tv=1.50` 计，任何非零相邻 O 角改变还收 `w_oblique_event=1.50`。同一目标函数规则同步进入粗规划、共享评分和两个 NLP 目标，并为固定序列 NLP 补入原先漏掉的 O 巡航项。

P12_env1 默认 runtime map scale 已是 ×0.5（`run_demo.sh --map-scale` 可覆盖）。为减小可视栅格单元，P12_env1 与此前截图对应的 P12_env4 占据图像导出像素增至 3 倍，`resolution` 同比减小，地图米制边界不变。生成 PGM 中矩形障碍边缘是直线；若 RViz 仍显示明显锯齿，应再核实实际加载的 map topic/目录是否为这两个 digitized map。

中间费用组在 N03 同起终姿态配对为 A15/O1/Spin1，O 只在一段局部出现；优化后 0 次 O 角更新，独立 swept-SAT 通过，总耗时 6.66 s。较低 O 费用组的全 O 路线则有 6 次角更新、3 次低于 5°，并触发多次时序碰撞修复，总耗时 25.87 s，已弃用。N05 中间费用组仍由粗规划选择全 O（该起终姿态下 Ackermann probe 未找到候选），优化后保留 2 次 O 角更新、0 次小于 5°，倒车为 0，独立 swept-SAT 通过，总耗时 4.61 s。

N03 中间权重同参数的局部基线为 score `23.509`、`6.51 s`；把 MINLP knot 上限临时放宽到 24 后，20-knot seed 进入 Bonmin，但 Bonmin 在 4.17 s 后 `LIMIT_EXCEEDED` 且没有整数 incumbent。其安全固定模式 warm-start fallback 以相同 `Spin-A-O` 序列得到 score `20.458`（约降 13.0%），端点误差 `0.0076 m / 0.0205 rad`，独立 swept-SAT 通过；总耗时 `17.72 s`。这次显著改善来自安全连续 fallback 的几何优化，不能归因于 Bonmin 找到了新的整数模式。把 warm-start 预算从 6 s 缩到 3 s 后，该 20-knot seed 初始 NLP 不可行，没有 fallback，因此不采用更短预算。

N05 中间权重同参数的局部基线为 score `28.548`、`4.61 s`；单 seed MINLP fallback 得到 `28.432`（约降 0.4%），端点误差 `0.0105 m`，swept-SAT 通过，但耗时 `16.17 s`，Bonmin 同样无终端整数 incumbent。低 O 费用的双 seed N05 试验未改善，合计 `19.01 s`；已把 MINLP 输入限制为最低分的单个安全本地 seed，低费用单 seed 试验缩到 `12.52 s`。因此虽 N03 得到明显 safe fallback 改善，Bonmin 可用整数候选率仍为 0，且两场景启用 MINLP 的时延显著高于局部优化，整体目标尚未达成。


## 29. 2026-10-07 分离 warm-start 预算并按安全增益早停

### 配置问题与修正

`joint_refine()` 此前把固定模式 seed 投影的时限取自 `collision_repair_time_limit`。这两个阶段职责不同：前者是 Bonmin 启动前的可行连续投影，后者是整数候选碰撞后的 SAT 修复预算。N03 将修复预算设为 3 s 时，warm-start 也被压到 3 s；记录到的 fixed-schedule 求解因 `Maximum_WallTime_Exceeded` 结束，末端位置误差 `0.04184 m`，超过 MINLP 的 `0.0155 m` 可行门槛，因而不能作为安全 fallback。

现在新增 `joint/warm_start_time_limit`（默认 `6.0 s`），与 SAT 修复预算分开。另新增默认关闭的 `joint/warm_start_early_accept_improvement`：当固定模式投影已经通过末端和 swept 检查、且相对输入 seed 的共同分数改善超过配置阈值时，可以直接返回该安全候选，省去后续 Bonmin；默认 `0.0` 表示总是继续尝试 Bonmin。这个分支只接受固定模式连续解，不把它标记成 MINLP 整数改进。

### N03 同 seed 配对

N03 起终点 `(0.85,1.25,0) → (11.05,6.75,0)`，20 knots，warm-start `6 s`，Bonmin 总限时 `8 s`，节点 NLP 时限 `1.25 s`，root resolve `1`：

- 关闭早停时，Bonmin `LIMIT_EXCEEDED` 且无整数 incumbent；安全 fixed-schedule fallback 分数 `20.457805`，输入 seed `23.509489`，端点误差 `0.00759 m / 0.02050 rad`，独立 timed swept-SAT 通过（3,399 samples），端到端 `17.55 s`。
- 临时启用 `5%` 早停阈值时，获得完全相同的分数、端点和 swept 结果；端到端降为 `13.17 s`，节省 `4.38 s`（约 `25%`）。此配置在达到阈值后跳过 Bonmin，所以这仍是连续几何改进，不是模式整数解的收益。

N05 用 `0.3%` 阈值复测后也提前接受了固定模式候选：score `28.432270`，相较输入 `28.548232` 降约 `0.4%`，与原 MINLP fallback 分数一致；端点误差 `0.01053 m`，独立 timed swept-SAT 通过（3,206 samples）。端到端从原来的 `16.17 s` 降为 `6.09 s`，约节省 `62%`。这证明该早停对 N03 和 N05 两类模式结构都能保留既有安全连续收益；依然没有证明 Bonmin 找到整数候选。仓库阈值保持默认 `0.0`。探针：`relust/N03_mode_cost_minlp_v25_probe.json`、`relust/N03_mode_cost_minlp_v26_probe.json`、`relust/N05_mode_cost_early_accept_v33_probe.json`。Python `py_compile` 与 optimizer catkin build 通过。探针使用隔离 master `11344`，已关闭；`11311/11312/11320` 未触碰。

### Bonmin 根节点诊断与参数消融

将节点顺序从 `depth-first` 改为 `best-bound` 后，N03 仍返回相同 fixed-schedule fallback，score `20.457805`，耗时 `17.27 s`；未提高整数候选率。为复现与诊断，新增加可配置的 `joint/node_comparison`，默认仍为 `depth-first`。

同 seed 下把总时限增至 `15 s`、关闭 cutoff 后，Bonmin 约 `4.15 s` 报 `INFEASIBLE`。新增日志显示根初值 `root_feasible=True`、约束违反 `0`、最大残差 `5.74e-8`；因此这次 `INFEASIBLE` 不能解释成 warm-start 根点违反了 MINLP 约束，也不能归因于 cutoff。每节点 NLP 放宽至 `3.5 s / 600 iterations` 后状态变成 `LIMIT_EXCEEDED`，仍没有整数 incumbent；再放宽至 `8 s / 1200 iterations`、取消 root resolve，Bonmin 阶段耗时约 `24.36 s`，仍无整数候选。三组最终都保留 score `20.457805` 的安全 fixed-schedule fallback，未优于 13% 的既有连续几何收益。

Bonmin 文档说明 `nlp_failure_behavior=fathom` 会在 Ipopt 未能满足最优性容差时丢弃该搜索节点，使 B-BB 变成启发式搜索；本模型当前正使用该设置。[Bonmin options list](https://coin-or.github.io/Bonmin/options_list.html)。下一步要针对“根点确实可行，但非凸根 NLP 未能解出整数候选”改进可行解注入或节点失败后的搜索恢复，而不是继续单纯增加总时限。

相关探针：`relust/N03_mode_cost_minlp_v27_probe.json`（best-bound）、`relust/N03_mode_cost_minlp_v28_probe.json`（总时限 15 s）、`relust/N03_mode_cost_minlp_v29_probe.json`（关闭 cutoff）、`relust/N03_mode_cost_minlp_v31_probe.json`（节点 NLP 3.5 s）、`relust/N03_mode_cost_minlp_v32_probe.json`（节点 NLP 8 s）。根点诊断来自 ROS optimizer 日志；完整探针时间可能超过配置的 Bonmin 时限，因为 warm-start、NLP 建模和 Bonmin 节点边界都会额外耗时。

## 30. 2026-10-07 B-Hyb 崩溃诊断与隔离

在 N03 同一起终点上试跑 Bonmin `B-Hyb`（总限时 15 s、节点 NLP 3.5 s、600 次迭代、warm-start 开启）。粗搜索正常得到 A15/O1/Spin1 的正向路线；optimizer 进入 joint MINLP 后，Bonmin 抛出 `Bonmin::TNLPSolver::UnsolvedError*`，进程以 exit code `-6` 退出，未发布优化轨迹，也没有可供独立 swept-SAT 检查的结果。探针日志保存在 `relust/N03_mode_cost_minlp_v34_bhyb_probe.json`。

Bonmin 官方选项表将 `nlp_failure_behavior` 列为 B-BB 可用、B-Hyb 不可用；该选项正是当前 B-BB 用于在 Ipopt 节点解失败时继续搜索的恢复机制。结合本次非凸轨迹 NLP 的进程退出实测，`joint_refine()` 现在在建模前拒绝 B-Hyb，并返回普通 Python 配置错误，让 optimizer 保留已有安全的本地候选，而不是让 C++ 异常终止 ROS 进程。仓库默认仍是 B-BB，本次不改默认求解器。源码语法检查与 optimizer catkin build 通过；该保护分支没有做第二次 ROS 复跑。

### B-BB 未解节点随机重启消融

另加了 `joint/bonmin_unsolved_nlp_retries`，默认 `0`、上限 `3`，只请求 Bonmin 对未解节点尝试有限次随机起点。N03 同 seed 将它设为 `1`，其余沿用 15 s / 节点 NLP 3.5 s / 600 次迭代 / root resolve 1 / cutoff 关闭：Bonmin 仍为 `INFEASIBLE`、无整数 incumbent；root 初值仍可行，最大残差 `5.74e-8`。安全 fixed-schedule 候选仍为 `20.457805`（seed `23.509489`），独立 5 mm swept-SAT 通过（3,399 samples），末端 `0.00759 m / 0.02050 rad`；端到端 `21.26 s`。随机重启没有新增模式改进或质量收益，所以该选项保留为默认关闭的诊断开关，不推荐在常规运行中开启。逐次结果：`relust/N03_mode_cost_minlp_v35_nlp_retry1_probe.json`。

随后仅把 B-BB `random_seed` 从 0 改为 1、关闭节点随机重启，其他设置不变复跑：仍无整数 incumbent，状态 `LIMIT_EXCEEDED`；安全 fixed-schedule 候选仍为 `20.457805`，独立 5 mm swept-SAT 通过（3,399 samples），端到端 `24.13 s`。同一启动的自动首轮也得相同分数，耗时 `24.66 s`。说明随机种子本身没有复现此前的模式改进，不能作为稳定方案。数据：`relust/N03_mode_cost_minlp_v36_seed1_probe.json`。

再将变量选择改为 Bonmin `nlp-strong-branching`，其余恢复 seed 0 配置：仍 `LIMIT_EXCEEDED`、无整数 incumbent，safe fallback 仍为 `20.457805`，独立 5 mm sweep 通过（3,399 samples），端到端 `23.96 s`。因此目前 depth-first/best-bound、seed 0/1、普通最分数/ NLP strong branching、节点随机重启均没有改变 N03 的整数候选结果；不继续堆这些参数。结果：`relust/N03_mode_cost_minlp_v37_nlp_strong_branching_probe.json`。

下一步回到可控的 B-BB 分支：不再单纯加长节点 NLP 时限、改变随机种子/重启或分支变量规则，转向从安全的固定模式解生成少量几何驱动的邻近模式组合，逐个固定 schedule 连续精修，再由统一分数与 swept-SAT 筛选；比较时仍要单独报告 Bonmin 整数 incumbent 与这些邻域候选，避免把连续几何收益误记为全局模式优化收益。

### 当前结论

这次迭代修正了 warm-start 与碰撞修复预算的错误耦合，并验证了固定模式投影已有明显收益时提前返回可以省下求解时间，同时保留路径质量和安全结果。尚未解决的核心问题不变：Bonmin 可用整数 incumbent 率仍低，当前 N03 的约 `13%` 路径改善来自 fixed-schedule 连续几何优化。后续应继续改善分支搜索如何利用已验证的固定模式可行点，之后再用 N03/N05 配对检验是否真正增加整数 incumbent 与模式质量。

## 31. 2026-10-08 MINLP 运动学松弛拆分与 seed 来源对照

### 运动学表达式

把 A/O 两类平移距离分成 `ds_ack` 和 `ds_obl`，分别用对应的模式二进制量做上下界门控；状态转移直接累加 A 弧线位移与 O 斜向位移，航向仅由 A 曲率和 Spin 角度增量更新。整数 one-hot 模式下与原运动学等价，但根松弛不再出现“分数模式变量乘整个位姿状态”的项。初版还保留总距离 `ds_f` 并加等式 `ds_f=ds_ack+ds_obl`，它增加冗余 NLP 变量/等式：N03 warm-start 根点出现一条 `0.0171` 的走廊违约，N05 没有超过本地解的候选。已删除 `ds_f` 和该等式，距离目标改成 `ds_ack+ds_obl`；整数运动学不变，N03/N05 根点随后均可行。optimizer 对“候选未改善”路径也补充打印 Bonmin 终态、回退类型、根可行性与求解时间，避免漏掉失败诊断。

### 配对数据

| 场景 / seed | Bonmin 结果 | 安全连续回退 | 根点 | 时间 |
|---|---|---|---|---|
| N03 best-local，20 knots | `LIMIT_EXCEEDED`，无整数 incumbent | `23.509489 → 20.457800`；5 mm sweep 通过，末端 `0.00759 m / 0.02050 rad` | 可行，0 项违约，最大残差 `2.42e-6` | warm `6.39 s` + Bonmin `10.78 s`；全流程 `24.10 s` |
| N05 best-local，8 knots | `LIMIT_EXCEEDED`，无整数 incumbent | `28.548232 → 28.432269`；5 mm sweep 通过，末端 `0.01053 m / 0 rad` | 可行，0 项违约，最大残差 `2.35e-8` | warm `6.03 s` + Bonmin `4.74 s`；全流程 `15.56 s` |
| N03 coarse，15 knots | `INFEASIBLE`，无整数 incumbent | `25.979580 → 20.473667`；5 mm sweep 通过，末端 `0.00760 m / 0.01519 rad` | 可行，0 项违约，最大残差 `9.67e-8` | warm `1.87 s` + Bonmin `10.69 s`；全流程 `19.45 s` |

N03 选粗 seed 后端到端快约 `4.65 s`，最终分数只比 best-local 回退高 `0.0159`；减少的是 seed 投影与节点数，仍没得到整数模式收益。N05 固定模式回退分数复现既有早停结果，但本次关闭早停以测 Bonmin，耗时 `15.56 s`，早停配对是 `6.09 s` 且会跳过 Bonmin。故当前只能说运动学拆分恢复了根初值可行性，粗 seed 降低了 N03 的 MINLP 时延；没有提高 Bonmin 可用整数解率，也没有改善最优安全最终路径分数。全局 MINLP 继续保持 `joint.enabled:false`。

N03/N05 独立 ROS probe 与 solver 指标合并在 `relust/global_minlp_disaggregated_motion_20261008.json`；逐次原始数据见 `relust/N03_mode_cost_minlp_disagg_reduced_probe.json`、`relust/N05_mode_cost_minlp_disagg_reduced_probe.json` 和 `relust/N03_mode_cost_minlp_coarse_seed_probe.json`。第一版冗余表达式诊断见同前缀未带 `reduced` 的探针文件。代码新增实验参数 `joint/minlp_seed_source`，默认 `best_local`；`coarse` 只用于此轮隔离配对，尚未设成仓库默认。

下一步停止继续堆 Bonmin 时限/分支选项；在通用高曲率或转向变化区域生成少量模式邻域方案，逐个做固定 schedule 连续精修并用共同代价、整车 swept-SAT 筛选。同时检查怎样把已验证的整数可行点作为真正 incumbent 提交给 Bonmin；报告时把 Bonmin 新整数 schedule 与固定模式连续 fallback 分开统计。全程不按地图编号加状态机。

本轮 py_compile、optimizer 包构建及 N03/N05 ROS 探针通过；隔离 master `11344` 已关闭，原有 `11311/11312/11320` 保持运行。

### Coarse seed / cutoff 单变量复验

在 N03 disaggregated model 上先用粗路线作 Bonmin seed、cutoff 关闭，再保持同一参数只打开 cutoff。两次均为根点可行、15 knots，Bonmin 均 `INFEASIBLE` 且无整数 incumbent；安全 fixed-schedule fallback 都是 `20.473667`。端到端分别 `19.45 s` 与 `19.37 s`，差异在单次运行抖动范围，cutoff 没有改变可用候选或分数。该结果没有复现旧配置曾返回 `O4/A5` 的现象；旧探针的粗模式 seed 与代价参数不同，不能直接视为同配置对照。本轮配对数据：`relust/N03_mode_cost_minlp_coarse_seed_probe.json`、`relust/N03_mode_cost_minlp_coarse_seed_cutoff_probe.json`。


## 32. 2026-10-08 Bonmin 全 Spin 非 incumbent 诊断

Bonmin 的限时返回矩阵在 N03/N05 两个独立场景都变成全 Spin：N03 为 20 个 `2`，N05 为 8 个 `2`。这些序列不能改变位置，且不满足末端位置约束，因此是 `LIMIT_EXCEEDED` 时暴露的非可行迭代点，不能称作整数 incumbent。`core.py` 现在会在终端单步投影失败后识别该模式字，跳过必然失败的固定序列 NLP，并记录 `bonmin_schedule_has_no_translation`；其它有平移的不同模式序列仍会尝试固定序列修复，最终仍须通过终端约束和精确 swept-SAT。

N03 用同一 seed 复跑得到安全 fixed-schedule fallback `20.457805`（seed `23.509489`），root 初值可行、0 项违约，warm `6.40 s`、Bonmin `10.79 s`、joint `17.67 s`、planner `24.25 s`。之前一次多做全 Spin fixed repair 的运行花了约 `2.8 s` 并以相同 score 回退；当前单次对比支持跳过该无效 NLP，但延迟收益还需重复测量。N05 同样全 Spin，安全 fallback `28.432269`（seed `28.548232`），root 初值可行，planner `15.76 s`；该分数与早先早停运行一致。

另将 N03 的 `joint/min_round_improvement` 临时从 `0.0025` 设为 `0.0`：交替搜索接受 2 轮，最终 local score `23.255`、耗时 `22.59 s`，但这一路径无法生成满足严格末端盒的 MINLP seed，`joint_refine` 以 `INFEASIBLE` 退出（初始末端违约 `0.009919 m`），全局候选没有产生。保留原早停阈值；不早停虽然改善了局部候选，却损失 `20.457805` 的安全 fixed-schedule fallback。

还试过按端点位移推导 A/O knot 数下界的线性强化。N03 仍返回全 Spin；root 初始违约变为 `0.0171`，且 warm-start 回退丢失，因此已撤回。逐次结果见 `relust/N03_bonmin_allspin_and_noearlystop_probe_20261008.json`。本轮 `py_compile` 与 `fwis_v3_optimizer` catkin build 均通过；隔离 ROS master `11344` 已关闭，原有 `11311/11312/11320` 保持运行。全局 MINLP 继续关闭；固定模式回退收益不能记作 Bonmin 模式改进。


## 33. 2026-10-08 通用固定模式邻域精修与 MINLP seed 保留

### 算法调整

把短窗口 A/O 模式邻域精修从 Bonmin 后置分支提升为独立的外层阶段，放在可行局部候选收集之后、全局 MINLP 之前。该阶段先对当前最佳安全 incumbent 做固定模式 IPOPT 精修，再生成最多两个通用短窗口候选：端点航向恢复的 A 段可以换成 O 直移；O 段可以换成端点完全匹配的 A 直移或双弧 S 形段。候选先通过 5 mm 整车 swept 检查，再固定模式做连续精修，以统一代价与当前 incumbent 比较。

这使其在 `joint.enabled:false` 时也运行；若用户显式开启 global MINLP，外层候选会进入 MINLP 的 seed 比较。模式邻域候选与 Bonmin 返回的整数解分别记账。仍不允许倒车，不含地图编号状态机；没有更好安全候选时保留当前 incumbent。

### joint 关闭时的场景配对结果

| 场景 | 局部 incumbent | 邻域精修后 | 改善 | 用时 | 检查 |
|---|---:|---:|---:|---:|---|
| N03 staggered islands | 23.509489 | 17.380742 | 26.1% | 11.34 s | terminal 与 timed swept 通过，倒车 0 m |
| N05 central chicane | 28.548232 | 26.472407 | 7.3% | 9.23 s | terminal 与 timed swept 通过，倒车 0 m |
| N04 pinched diagonal corridor | 33.941 | 未接受替换 | 0 | 6.85 s | incumbent 保留，terminal 与 timed swept 通过 |

N03 选中末端 `O → A` 等价替换，N05 选中窗口 `13:19` 的 `O → A` 替换。两条成功路径均由固定模式连续精修得到，不是 Bonmin 整数候选。N04 的模式邻域没有通过共同代价筛选，系统保留原安全 incumbent；该场景约增加 1.95 s 邻域阶段开销。

### 全局 MINLP 接入结果与边界

N03 继续开启 MINLP 后，外层邻域候选为 `17.380742`。Bonmin 状态为 `INFEASIBLE`，未产生整数 incumbent；但模型的 root 初始点可行、0 项违反、最大残差 `3.42e-7`。保留的安全 fixed-schedule seed 经连续精修后为 `17.207491`，planner 总用时 `22.90 s`，timed swept 检查通过。相比 joint 关闭的 `17.380742` 约再降 1.0%，但这仍是固定模式连续优化收益，不能记作全局 MINLP 找到了更好的模式序列。

对新 seed 把 root resolve 从 1 加到 3，Bonmin 仍为 `INFEASIBLE`；打开 RINS 也未改变结果。不同单次求解的局部路径分数略有波动，最佳仍是外层候选 `17.215614`，全局 MINLP 没有产生整数 incumbent。因此 RINS 保持关闭、`root_resolves:0` 与 `joint.enabled:false` 默认不变。Bonmin 官方选项说明 `nlp_failure_behavior=fathom` 会在节点 NLP 未解出时丢弃节点并继续，使非凸搜索成为启发式；该机制没有在本轮找到新的 schedule。[Bonmin options](https://coin-or.github.io/Bonmin/options_list.html)

`core.py` 另加了已验证整数 seed 的保留逻辑：若传入 seed 通过严格终端盒、精确整车 swept 检查，且 CasADi 模型的初始赋值也满足全部约束，则它直接作为安全 fallback 和可选 cutoff 上界；不再要求对同一非凸固定模式问题重新跑 IPOPT 后才允许保留。该路径不是向 Bonmin 注册的分支定界 incumbent，状态和 fallback 元数据明确标出这一点。

逐次指标与限定见 `relust/global_minlp_mode_neighborhood_20261008.json`。本轮 Python `py_compile` 与 `catkin_make -C catkin_ws --pkg fwis_v3_optimizer -j4` 通过。ROS 实验使用隔离 master `11344`，已停止；预存的 `11311/11312/11320` 未触碰。

### 下一步

外层固定模式邻域已经在 N03/N05 两个场景带来明显、可复现类型的质量改善，N04 没有候选时保持 incumbent。Global MINLP 仍未给出可验证的整数 incumbent；下一步应解决可行整数 seed 如何作为真实 incumbent 交给 Bonmin，以及未解 root/node NLP 的恢复与日志诊断。保持全局 joint opt-in，不再堆叠没有 matched gain 的 RINS 或额外 root starts。


## 34. 2026-10-08 MINLP 可达性下界、终点容差与节点预算配对

### 模型与配置调整

在 MINLP 中增加必要平移长度下界：

`sum_k(ds_ack[k] + ds_obl[k]) >= max(0, ||goal_xy - start_xy|| - terminal_xy_tol - solver_slack)`。

A/O 的弧长总和必然不小于起终点平移距离；自旋不产生位移。该式只缩紧松弛，不限制 A/O/Spin 的可选组合，返回轨迹仍逐条通过精确运动学、终点约束和整车 swept-SAT。首次实现误用了 CasADi `sum1`，对 `1 x n` 行向量沿行求和，造成每个 knot 都被要求覆盖全程位移；N03 初值因此有 21 项违约。首跑保存为 `relust/N03_translation_distance_cut_shape_bug_probe_20261008.json`，随后改为沿 knot 方向使用 `sum2`，错误结果不纳入对比。

探针还发现 MINLP 终点盒默认 `0.015 m / 0.02 rad`，比规划器实际验收的 `0.16 m / 0.14 rad` 严约一个数量级。正常局部路线可能已满足规划器标准，却无法成为 MINLP 可行初值。配置已将 MINLP 终点盒与规划器验收范围对齐，并保留较大的终点误差权重 `100 / 10`；最终发布仍须经过独立 timed swept 检查。Bonmin 的单节点 NLP 上限从 `4.0 s` 降为 `0.75 s`，总 MINLP 限时仍为 `1.5 s`。全局开关仍是 `joint.enabled:false`。

### 配对结果

| 地图/设置 | Bonmin 结果 | 安全最终代价 | 最终来源 | Planner 用时 | timed sweep |
|---|---|---:|---|---:|---|
| N03，严格终点盒 | `INFEASIBLE`；初值终点约束违约 `0.01705 m` | — | 保留原局部轨迹 | — | — |
| N03，规划器终点盒 | `INFEASIBLE`；root 初值 0 项违约，最大残差 `3.43e-7`；返回 10 个 Spin 的非可行点 | `17.188003` | `mode_neighborhood` | `24.89 s` | 通过，3,395 samples |
| N05，总限时 12 s / 节点 3.5 s | `LIMIT_EXCEEDED`；root 可行，返回 8 个 Spin 的非 incumbent | `24.844` | `joint_mode_neighborhood` | `31.32 s` | 通过 |
| N05，总限时 4 s / 节点 1.5 s | `LIMIT_EXCEEDED`；无整数 incumbent | `25.076` | `joint_mode_neighborhood` | `19.79 s` | 通过 |
| N05，总限时 1.5 s / 节点 0.75 s | `LIMIT_EXCEEDED`；root 0 项违约，Bonmin 约 `2.37 s` 后仍无整数 incumbent | `24.844` | `joint_mode_neighborhood` | `18.81 s` | 通过，3,647 samples |

N05 最近的 `joint.enabled:false` 邻域基线为 `26.472407`、`9.23 s`。最短预算下结果 `24.844`，共同代价降低约 `6.15%`，终点误差 `0.01495 m / 0.10618 rad`，倒车距离为 0。这个改善来自 Bonmin 后的固定模式邻域搜索；Bonmin 自身仍没有返回可行整数 incumbent。N03 同样由安全模式邻域得到 `17.188003`，略低于上一轮局部基线 `17.380742`，但多花约 13.6 s，不能记为全局 MINLP 的模式优化增益。

这些配对说明：对齐终点验收范围能让 N03/N05 的初始赋值满足 MINLP 约束；缩短节点预算保留了 N05 的最佳邻域结果并减少了总用时。但 Bonmin 的整数候选率仍为 0，N03 返回 `INFEASIBLE`，N05 返回 `LIMIT_EXCEEDED`，全 Spin 模式字均是无效限时点。结果与配置完整记录在 `relust/global_minlp_reachability_cut_20261008.json`；上一节记录增加了该文件链接。

### 当前判断

保留必要平移下界、规划器范围的终点盒和 `0.75 s` 节点 NLP cap，MINLP 继续默认关闭。N05 的端到端规划质量在 MINLP 开启后进一步下降，但收益由通用固定模式邻域后处理产生，耗时仍高于关闭 MINLP 的基线；因此这只是集成流水线的配对结果，不是已证明的 Bonmin 搜索收益。下一步继续解决向 Bonmin 注入真实可行整数 incumbent 的接口，并定位初始赋值可行但根 NLP 仍无 incumbent 的原因；不加地图专用规则，也不把安全回退分数记为 MINLP 命中。

本轮 `py_compile` 与 `catkin_make -C catkin_ws --pkg fwis_v3_optimizer -j4` 通过。隔离 ROS master `11344` 已停止；既有 `11311/11312/11320` 保持运行。


## 35. 2026-10-08 CBC 可行 seed incumbent 注入验证

用临时 CasADi 3.7.2 `bonminseed` 插件验证真正的 CBC incumbent 入口：CasADi 的 `set_initial()` 只给 NLP 起点；在注册的 `CbcHeuristic::setModel()` 中调用 `CbcModel::setBestSolution()`，能在 Bonmin 根分支搜索前将已通过完整约束与轨迹安全检查的 seed 提交给 CBC。N03 10 knots 与 N05 8 knots 都触发了注入日志，toy MINLP 也确认该钩子在根搜索前执行。首次接口验证在 `/tmp` 完成后，已将它做成版本锁定的可选工程组件：补丁 `catkin_ws/src/fwis_v3_optimizer/patches/casadi-0e672301-bonminseed.patch` 与构建脚本 `catkin_ws/src/fwis_v3_optimizer/tools/build_bonminseed_plugin.sh`。运行开关 `joint/use_cbc_seed_incumbent` 默认为 false；正式调用还要求 `CASADIPATH` 指向构建目录，并通过完整 seed 可行性门控。

| 场景 / 配置 | Bonmin 终态 | Bonmin 返回候选共同分 | 输入 seed 分 | 后置模式邻域 | Planner 总用时 |
|---|---|---:|---:|---:|---:|
| N03，10 knots，12 s | `SUCCESS`，模式字 `2000000000` | 17.187999 | 17.187999 | — | 21.12 s |
| N05，8 knots，12 s | `LIMIT_EXCEEDED`，模式字 `11110011` | 26.402998 | 26.402998 | 25.073796 | 29.28 s |
| N05，8 knots，45 s / 节点 2 s / 240 iter / fractional dive | `SUCCESS`，模式字 `11110011` | 26.405007 | 26.405007 | 25.075632 | 35.93 s |

N05 的本次最终路线通过 planner timed-sweep 检查。N05 后置模式邻域相对同一次 MINLP seed 约降低 5.0%，但 Bonmin 自身都只回到 seed 同分数；不能把这部分质量变化记为全局 MINLP 收益。较长预算虽然得到 `SUCCESS`，没有进一步降低共同轨迹代价。

N05/N03 正式 ROS 接入复核：构建脚本成功针对 wheel revision `0e672301aa9046162d311f69421f6375805f4fca` 生成插件；两个场景均在根搜索前分别提交了 147/183 维 seed。N05 Bonmin `SUCCESS`、返回 `11110011`，但 `26.404921` 与 seed 持平；随后通用模式邻域在窗口 `2:4` 将 O 换为 A，得到 `24.533158`，比 MINLP seed 低 7.09%，timed sweep 通过，planner 用时 `24.01 s`。N03 Bonmin `SUCCESS`，返回 `2000000000`，`17.188004` 与 seed 持平，planner 用时 `20.84 s`。因此真正的 CBC incumbent 接入已跨两图验证，但 Bonmin 本身尚未体现共同评分改进，N05 的额外收益仍来自其后的通用模式邻域。插件和开关均保持 opt-in，`joint.enabled:false` 默认不变。后续 N06 外推、目标对齐调整、迭代重规划与 cutoff/seed 对照见第 36 节。

原始逐次记录：`relust/N05_bonminseed_incumbent_probe_20261008.json`。临时 ROS master `11345` 已停止；预存的 `11311/11312/11320` 未触碰。

## 36. 2026-10-08 全局 MINLP 复种迭代与代价对齐复核

### 通用调整

在 `core.py` 的 MINLP 目标中补上共同评分里的低曲率 A/O 切换代价：若 A 与 O 之间切换时运动切线变化小于阈值，原 `sequence_cost` 会增加 `low_curvature_switch`，MINLP 以前漏掉了这一项。现在用窄 sigmoid 近似该阈值指标，保持 NLP 可微。该项主要约束“切模式却没有明显改变行驶方向”的情况，不限制为了减小局部弯曲而使用 O。

修正 MINLP 后模式邻域的接收门槛：邻域连续优化结果必须同时优于当前安全 incumbent 和它自己的候选 seed；避免把比自身 seed 更差、但略优于早期 incumbent 的解写进候选池。新增 opt-in 参数 `joint/global_rounds`（默认 `0`，最多 `2`），在一个安全邻域解改善共同分数后，用它重建走廊并重跑 MINLP；迭代 seed 继续拒绝倒车或方向切换。

### N05 配对结果

| 配置 | Bonmin 结果 | MINLP seed → 返回共同分 | 后处理邻域 | Planner 总时间 | timed sweep |
|---|---|---:|---:|---:|---|
| best-local，12 s，CBC seed，1 个额外 global round | 首轮 `LIMIT_EXCEEDED`；第二轮 `SUCCESS` | 第二轮 `24.626115 → 24.626115`，模式字 `11000011` 不变 | 最终 `24.626115` | `36.52 s` | 通过 |
| best-local，45 s，关闭 seed cutoff，CBC seed | `SUCCESS`，14 nodes / 1,545 iterations，best incumbent 与 bound 均为 `26.524247` | `26.297802 → 26.297802`，返回 `11110011` | 窗口 `2:4` 得到 `24.626156` | `53.13 s` | 通过 |
| coarse seed，45 s，CBC seed | `SUCCESS`，4 nodes / 501 iterations，best incumbent 与 bound 均为 `33.189866` | `33.189866 → 33.189866`，返回 `1111111` | 邻域得到 `25.917261` | `35.82 s` | 通过 |

额外 global round 确实以改进后的安全邻域轨迹作为新 seed 重新建走廊，但没有进一步降低共同分数。关闭 cutoff 扩展了节点树，也没有让 Bonmin 产生更低共同评分；粗路线作 seed 时 Bonmin 保留原全 O 粗路线，最终由后置模式邻域降到 `25.917261`，仍不如 best-local 路线邻域的 `24.5–24.6` 区间。故这些结果支持“安全邻域重规划可以带来收益”，不支持“当前 Bonmin MINLP 已经提高了结果”。Bonmin 使用 `nlp_failure_behavior=fathom` 丢弃未解的非凸节点 NLP；`SUCCESS` 或本次树搜索中的 incumbent/bound 相等，不构成原非凸问题的严格全局最优证明。`joint/global_rounds` 保持默认关闭，MINLP 不改为默认启用。

另以 N06 做接入外推：CBC 在 147 维模型上收到 seed；Bonmin `SUCCESS` 返回 `00000000`，共同分 `13.776025` 与 seed 持平；planner 总用时 `9.84 s`，timed sweep 通过。连同 N03/N05，已在三类地图上接通 CBC incumbent，但尚未看到 Bonmin 独立降低共同分数。

本轮诊断文件：`relust/N05_bonminseed_incumbent_probe_20261008.json`。本节实验使用隔离 master `11352`–`11355`，均已停止；预存 `11311/11312/11320` 未触碰。下一步优先扩大可行自由空间探索（走廊/多 seed 的通用策略）并区分“模型 bound 变好”和“可验证轨迹共同分变好”；当前走廊内 Bonmin 多次保留原 seed，继续单纯加时限意义有限。


## 37. 2026-10-08 N03/N05 CBC seed 可用率与 MINLP 时限配对

### 匹配实验

在 N03 staggered islands 与 N05 central chicane 上固定上游模式邻域时限 `2.0 s`、节点 NLP cap `0.75 s`、最大 knots `24`、碰撞离散间距 `0.75 m`，比较 joint 关闭、CBC seed 注入开关、Bonmin 总预算 `1.5/0.5 s`、MINLP 后固定模式邻域预算 `1/2 s`，以及粗模式邻域候选数 `1/2/4`。所有完成运行均通过独立整车 timed swept-SAT：沿轨迹段以 `5 mm` 采样完整足迹，并检查末端误差；无倒车和地图专用状态机。逐次结果与参数见 [`relust/N03_N05_minlp_latency_candidate_match_20261008.json`](../relust/N03_N05_minlp_latency_candidate_match_20261008.json)。

| 场景 / 配置 | Bonmin 返回 | 最终共同分 / 来源 | Planner 用时 | Independent swept-SAT |
|---|---|---:|---:|---|
| N03，joint 关闭 | — | `17.307` / mode neighborhood | `14.39 s` | 通过，3,421 samples |
| N03，1.5 s，CBC 关闭，后邻域 1 s | 全 Spin 非 incumbent，Python 安全回退 | `17.368` / mode neighborhood | `17.10 s` | 通过，3,457 samples |
| N03，1.5 s，CBC 开，后邻域 1 s | `LIMIT_EXCEEDED`；安全 seed 候选 `17.306855`，未改进 seed | `17.307` / joint MINLP | `16.51 s` | 通过，3,402 samples |
| N03，1.5 s，CBC 开，后邻域 2 s | `LIMIT_EXCEEDED`；安全 seed 候选与 seed 持平 | `17.307` / joint MINLP | `17.69 s` | 通过，3,418 samples |
| N03，0.5 s，CBC 开，后邻域 1 s（重复两次） | 两次均注入并返回 seed，`LIMIT_EXCEEDED` | `17.368` / joint MINLP | `15.67 / 15.59 s` | 两次通过，3,438 samples |
| N03，1.5 s，CBC 开，粗模式邻域候选数 1 | `LIMIT_EXCEEDED`；安全 seed 候选 `17.306921`，未改进 seed | `17.306921` / joint MINLP | `14.63 s` | 通过，3,403 samples |
| N03，0.5 s，CBC 开，粗模式邻域候选数 1 | `LIMIT_EXCEEDED`；返回 seed `17.368135` | `17.368135` / joint MINLP | `13.02 s` | 通过，3,438 samples |
| N05，joint 关闭 | — | `26.298` / mode neighborhood | `11.63 s` | 通过，3,541 samples |
| N05，1.5 s，CBC 关闭，后邻域 1 s | 全 Spin 非 incumbent，Python 安全回退 | `24.716` / joint mode neighborhood | `16.43 s` | 通过，3,656 samples |
| N05，1.5 s，CBC 开，后邻域 1 s | `LIMIT_EXCEEDED`；返回 `11110011`，与 seed 同分 | `24.645` / joint mode neighborhood | `16.64 s` | 通过，3,648 samples |
| N05，1.5 s，CBC 开，后邻域 2 s | `LIMIT_EXCEEDED`；Bonmin 与 seed 同分 | `24.626` / joint mode neighborhood | `17.25 s` | 通过，3,644 samples |
| N05，0.5 s，CBC 开，后邻域 1 s | `LIMIT_EXCEEDED`；返回 `11110011`，与 seed 同分 | `24.464` / joint mode neighborhood | `15.79 s` | 通过，3,631 samples |
| N05，1.5 s，CBC 开，粗模式邻域候选数 1 | `LIMIT_EXCEEDED`；Bonmin 与 seed 同分 | `24.626` / joint mode neighborhood | `14.23 s` | 通过，3,644 samples |
| N05，1.5 s，CBC 开，粗模式邻域候选数 4 | `LIMIT_EXCEEDED`；Bonmin 与 seed 同分 | `24.626` / joint mode neighborhood | `20.96 s` | 通过，3,644 samples |
| N05，0.5 s，CBC 开，粗模式邻域候选数 1 | `LIMIT_EXCEEDED`；Bonmin 与 seed 同分 | `24.639` / joint mode neighborhood | `13.29 s` | 通过，3,648 samples |

### 结论与下一步

CBC 注入把当前匹配的 Bonmin 可用安全候选率从 `0/2` 提高到 `2/2`；共 18 次 CBC 开启运行都返回了安全 seed，但没有一次找到优于最佳输入 seed 的整数模式序列。N05 最终比 joint 关闭低约 `6.0–7.0%`，改进来自 MINLP 后的通用固定模式邻域（O→A，窗口 `2:4`），不是 Bonmin。N03 在 1.5 s 下与 joint 关闭几乎持平；0.5 s 两次候选数为 2 的复跑及一次候选数为 1 的运行均为 `17.368135`，比本轮 joint 关闭高约 `0.35%`。所以整体集成在 N05 有改善，但尚未证明 MINLP 本身跨场景提升质量。

粗模式邻域候选数 `2 → 1` 后，N03/N05 最终分分别变化 `-0.00008 / -0.019`，粗邻域阶段分别省约 `2.59 / 2.26 s`，Planner 总时延省约 `1.88 / 2.41 s`。加到 `4` 时，N05 粗邻域阶段从约 `7.08 s` 增到 `11.60 s`，最终分没有改善。故将 `mode_master/schedule_neighbor_candidates` 的 demo 参数和节点兜底默认值设为 `1`；这是减少前置固定模式邻域耗时，不是 Bonmin 的搜索收益。

将 MINLP 后邻域限时从 `2 s` 降至 `1 s`，N03 共同分基本不变，省 `1.18 s`；N05 的分数差 `0.019`，省 `0.61 s`。据此把 `joint/mode_neighborhood_time_limit` 的 opt-in 默认值和 demo 参数从 `2.0` 调为 `1.0`。把前置粗邻域候选数设为 1 后，再叠加 Bonmin `0.5 s`，Planner 用时为 N03 `13.02 s`、N05 `13.29 s`，两图均拿到 CBC seed 且 swept-SAT 通过。N05 最终 `24.639`；N03 为 `17.368135`，比 joint 关闭高约 `0.35%`，因此 Bonmin 总预算 `1.5 s` 另保留。`joint.enabled:false`、CBC incumbent 注入仍关闭，全局 MINLP 仍 opt-in。

下一轮应把已通过独立扫掠的最佳通用模式邻域解提前作为 MINLP 的可行 seed/incumbent，并判断 CBC/Bonmin 是否能在它之上发现严格更低的共同轨迹分。继续分别记录“Bonmin 新 schedule 改善”与“MINLP 后固定模式邻域改善”，不能把后处理收益记作全局 MINLP 搜索收益。本轮隔离 ROS master `11356`–`11366` 已清理；既有 `11311/11312/11320` 保持运行。


## 38. 2026-10-08 Bonmin distinct-seed 多起点消融

为提高全局 MINLP 搜索覆盖，临时将每个场景传给 Bonmin 的安全起点从 1 个增为 2 个，并要求模式字不同。N03 起点为 `17.368135 / mode 20` 与 `20.296384 / mode 201`；Bonmin 分别返回各自 seed，最终 `17.368135`，用时从单起点候选数 1 的 `14.63 s` 增至 `19.40 s`。N05 起点为 `26.297802 / mode 101` 与 `28.548232 / mode 1`；第二次 Bonmin 返回 `28.547422`（仍比最佳 seed 差），最终仍由 MINLP 后邻域得到 `24.626115`，总用时从单起点候选数 1 的 `14.23 s` 增至 `22.17 s`。两图独立 timed swept-SAT 均通过。

增加 distinct seed 没有改善 Bonmin 相对最佳输入 seed 的整数候选，反而多耗时 `4.8–7.9 s`。因此已撤销该临时多起点代码和参数，保留单起点行为。细节保存在上述 JSON 的 `experimental_distinct_minlp_seeds2` 两次运行记录中。下一步聚焦走廊/目标与最佳安全 seed 的匹配，让一个 seed 本身覆盖更有希望的自由空间；不继续堆叠 Bonmin 启动次数。


## 39. 2026-10-08 MINLP 走廊 clearance slack 隔离实验

先把原 `joint/corridor_clearance_slack` 临时设为 `0.05 m` 做全链路实验。该参数也传给前置 fixed-schedule 邻域，因此它同时改变了局部候选和 Bonmin 走廊：N05 的前置邻域候选数从 1 变成 0，最终分 `26.923`，高于 joint 关闭基线 `26.298`。这个结果说明不能将整条链路的变化归因于 MINLP 自身。

随后新增 `joint/minlp_corridor_clearance_slack`，只传给 Bonmin MINLP 与显式 global round；前后 fixed-mode 优化仍使用原 slack。把该值设为 `0.05 m` 后，N03/N05 都通过独立 swept-SAT，Bonmin 仍返回 seed：N03 为 `17.368135`，N05 最终仍为 `24.626115`，Planner 时间分别 `14.51 / 14.87 s`。N05 的 Bonmin bound 从常规 slack 的约 `20.73` 提到 `23.87`，但 incumbent 与输出模式字未变，且用时多约 `0.64 s`。不把 slack 提升视为轨迹质量提升；正式新参数保持 `0.0`，留作后续受控实验接口。逐次数据见第37节链接的 JSON。

本轮 `optimizer_node.py` Python 语法检查通过。所有本轮完成运行的独立 `5 mm` swept-SAT 均通过；隔离 master `11365/11366` 已关闭，预存 `11311/11312/11320` 未触碰。下一步应比较模型目标与共同轨迹分在连续状态、低曲率切换、走廊位置惩罚上的差别，并以独立安全候选作 Bonmin incumbent；当前只提高 CBC 安全候选可用性与后处理质量，还没有 Bonmin 新模式带来的分数下降。


## 40. 2026-10-08 按 seed 模式切换数分配 MINLP 预算

### 调整与配对结果

给 global MINLP 增加通用预算策略：输入 seed 恰有一次模式切换时请求 `4.0 s`；零次或多次切换仍请求基础 `1.5 s`。规则同时用于首次 MINLP 与显式 global round，不依赖地图编号。`joint/adaptive_time_limit` 由 demo 参数开启；`joint.enabled:false` 仍保持关闭，因此常规规划不受影响。

固定混合模式切换目标、CBC seed 注入、3.0 s 节点 cap、关闭两类 feasibility pump，在 N03/N05 配对运行：

| 场景 | seed 切换数 / 请求预算 | Bonmin 返回 / 共同分 | 最终结果 | Planner 用时 | 独立扫掠检查 |
|---|---|---|---|---:|---|
| N03 staggered islands | 1 / 4.0 s | `LIMIT_EXCEEDED`，返回全 A；`18.508122 → 15.384002` | `joint_minlp` | `22.20 s` | 通过，3,440 samples；末端误差 `0.0080 m / 0.0544 rad` |
| N05 central chicane | 2 / 1.5 s | `LIMIT_EXCEEDED`，Bonmin 未改 seed `26.297802` | 后置固定模式邻域得到 `24.626` | `16.00 s` | 通过，3,644 samples；末端误差 `0.0174 m / 0.0689 rad` |

N03 的 Bonmin 新 schedule 相对输入 seed 降分约 `16.9%`，且没有倒车；这是本轮确实由全局模式搜索带来的质量改善。N05 仍由后置固定模式邻域贡献最终改善，不能算作 Bonmin 增益。相较 N05 也请求 4 秒的前一轮混合目标运行，这次在 1.5 秒请求下最终分数相同，Bonmin wall time 从约 `9.14 s` 降到 `3.99 s`。

需要注意，`time_limit` 不是严格 wall-clock 上限：N03 请求 4 秒但 Bonmin 实际运行 `10.35 s`；N05 请求 1.5 秒但实际运行 `3.99 s`。更长预算在单次切换 N03 上带来了明显分数下降，也显著拉长了求解时间。独立 5 mm 整车 swept-SAT 对两条最终轨迹均通过。逐项参数与日志数据保存在 [`relust/adaptive_minlp_budget_by_switch_count_20261008.json`](../relust/adaptive_minlp_budget_by_switch_count_20261008.json)。

### N03 预算档位探针与下一步

同一 N03 流程另测请求 `2.0 s` 与 `3.0 s`。2 秒运行从 12-knot、共同分 `18.508122`、模式字 `20` 的 seed 出发，Bonmin 约 `3.99 s` 后仍返回同一模式和分数；4 秒运行从同分同 knots 的 seed 出发，返回全 A，降到 `15.384002`。3 秒运行从不同的 9-knot seed（分数 `17.383088`）出发，Bonmin 约 `3.46 s` 未改善，因此不能当作干净的预算剂量对照。独立扫掠检查三次均通过。该结果支持暂时保留 4 秒档，但 Bonmin wall time 超额仍需解决，且需在更多地图验证。

已完成 N05 global round：把后置邻域 `24.626115` 的安全轨迹以模式字 `11000011` 重新交给 Bonmin。第二轮仍返回同模式、同共同分，增加约 `3.95 s` planner 时间；独立扫掠通过。因此 `joint/global_rounds` 继续默认 `0`。N06 外推中，seed 零切换，按基础 `1.5 s` 请求；Bonmin `SUCCESS`，返回同样的全 A 模式，common score 从 `13.776025` 降到 `13.707373`（约 `0.50%`），planner 用时 `5.23 s`，独立扫掠通过。这是连续状态改进，不是模式组合改进。

三图结果目前显示：N03 有明显模式组合收益；N06 有小幅连续精修收益；N05 的收益仍来自 MINLP 后固定模式邻域，Bonmin 与额外 global round 都未降低分数。下一步在 N04 做同配置外推，并处理 Bonmin 请求限时与实际 wall time 超额问题。全局 MINLP 继续 opt-in，独立记账模式 schedule 与连续状态改进。隔离 master `11386/11387/11388/11389/11390` 均已关闭；既有 `11311/11312/11320` 未触碰。


## 41. 2026-10-08 N04 knot 合并、可行 seed warm start 与预算诊断

### N04 维度消融

N04 的 MINLP 输入有 4 次模式切换、21 knots、339 个模型变量。默认合并阈值与温和 `0.05 1/m / 5°` 均保持 21 knots，Bonmin 共同分 `32.543`。阈值改为 `0.10 1/m / 7.5°` 后减到 20 knots / 323 variables，分数仅变为 `32.531`。中等阈值 `0.15 1/m / 8.5°` 减至 18 knots / 291 variables，但折叠 seed 的 root 初值有 9 项约束违反、最大 `0.0279`；Bonmin 未产生 terminal integer incumbent。固定模式安全回退得到 `31.906`，独立 5 mm swept-SAT 通过（5,609 samples），因此这 `1.96%` 改善不能归因于全局模式搜索。

### Warm start 与预算语义

把 opt-in `feasible_seed_warm_start` 打开后，warm projection 耗时约 `3.58 s`。在总 MINLP 请求 `1.5 s` 下，模型 setup 占 `4.07 s`，Bonmin 只剩 `0.05 s`；即使把总预算提到 `6 s`、实际给 Bonmin `1.94 s`，也未找到优于 seed 的 mode schedule，最终分数约 `31.911`、Planner 用时 `17.62 s`。此开关仍保持关闭。

本轮发现 seed 几何安全与终点合格，并不保证压缩动作后的离散 MINLP 初始点满足所有模型约束。因此 `validated_seed_candidate` 现在检查实际 collapsed `initial_actions`；显式启用 `feasible_seed_warm_start` 时会执行固定模式投影，而不再只在几何/终点检查失败时跳过式触发。预算日志也已修正：固定模式回退会保留外层 Bonmin 的 requested/setup/solver/node/pump 信息。N04 回归日志验证为 `requested=1.50 s, setup=0.49 s, solver_limit=1.01 s`，而非内部固定模式 repair 使用的 `2.0 s`。两项 Python 文件语法检查通过。

### 结论

N04 的 knot 合并降低了模型大小，并让固定模式安全回退找到更低共同分，但中等强度合并仍让初始 MINLP 约束不可行；强制 warm start 没解决模式搜索，也会挤占全局预算。默认合并阈值与 `feasible_seed_warm_start:false` 保持不变；仅保留一般性的可配置实验参数。逐次记录见 [`relust/adaptive_minlp_budget_by_switch_count_20261008.json`](../relust/adaptive_minlp_budget_by_switch_count_20261008.json)。隔离 master `11391`–`11399` 均已关闭；既有 ROS master `11311/11312/11320` 未触碰。


## 42. 2026-10-08 MINLP 参考走廊对齐与候选后处理时延复核

### 初值与走廊参考对齐

`_collapse_for_joint()` 返回的参考点来自折叠前分组端点；当合并容差改变动作长度时，它们可能不再落在 MINLP 实际初值轨迹上。现在动作序列确定后，用 `replay(start, initial_actions)` 重建 `refs/seed_poses`，使走廊中心与模型的状态、控制初值来自同一条轨迹。该修正不依赖地图编号，也不改变模式规则。

配对结果：N03 的 10-knot 输入根点可行（0 项违反，最大 `2.97e-7`），Bonmin `SUCCESS` 将 seed `17.380962` 的单次切换模式字改为全 A，common score 降至 `15.210291`，Bonmin 用时 `1.76 s`、Planner `13.50 s`；独立 5 mm swept-SAT 通过（3,410 samples，终点 `0.00496 m / 0.02825 rad`）。这次是当前证据中由 Bonmin 改变模式字并降低共同分数的明确样例。

N05 根点也可行（0 项，最大 `3.9e-7`），Bonmin 保持 `26.297802` 的输入分数；后续固定模式邻域得到 `24.626121`，Planner `16.23 s`，独立扫掠通过（3,644 samples）。这部分收益仍属于 MINLP 后处理。N04 在 `.15 1/m / 8.5°` 合并配置下，根点违反为 0、最大残差 `7.33e-5`；1.5 秒请求运行的模式结构仍为 `20102`，共同分数 `32.600266`，独立扫掠通过（5,690 samples）。较早 9 项违反的 N04 运行使用不同上游 seed，因此不能把两次差异当成严格因果对照。

N04 同配置把请求从 1.5 秒加到 4 秒，最终模式字和共同分数均未变化；Bonmin 实际用时从 `3.37 s` 增到 `9.39 s`，Planner 从 `12.34 s` 增到 `18.28 s`。多切换 seed 保持 1.5 秒档，不再为 N04 追加预算。

### 后置邻域候选数与异常目标值

把 `joint/mode_neighborhood_candidates` 从 2 降到 1 后做重复配对。N03 两次都由 Bonmin 得到全 A、共同分 `15.207418`（seed `17.383056`），Planner 用时 `13.16 / 13.06 s`，低于双候选运行的 `13.50 s`；两次扫掠均通过。N05 两次后处理得分为 `24.721292 / 24.626115`，双候选结果为 `24.626121`；Planner 用时 `15.12 / 15.05 s`，比双候选运行的 `16.23 s` 少约 `1.15 s`。观察到的最高分差为 `0.095`（约 `0.38%`），第二次复跑达到与双候选相同的得分。综合延迟与质量，已将 demo 配置和节点兜底默认值设为 1；它只限制 MINLP 后固定模式邻域的候选数，`joint.enabled:false` 仍不变。

一次 N03 候选数为 1 的运行曾记录异常 `model_obj=-1.38e11`，但共享轨迹分正常。已给 MINLP 日志增加求解后约束残差计数与最大值；相同设置连续两次复跑均得到 `model_obj=15.221708`、0 项约束违反，Bonmin `SUCCESS`，独立扫掠均通过（各 3,406 samples）。异常暂按孤立求解值异常记录并继续监测；候选仍按统一轨迹分、终点验收和独立整车 swept-SAT 判断，不用该异常值宣称优化收益。

配置与逐次数据见 [`relust/adaptive_minlp_budget_by_switch_count_20261008.json`](../relust/adaptive_minlp_budget_by_switch_count_20261008.json)。Python 语法检查通过；本轮 ROS 使用隔离 master `11403`–`11408`，均已停止；预存 `11311/11312/11320` 保持运行。


## 43. 2026-10-08 按模式切换数与 knot 数设置 Bonmin 预算

将节点 NLP wall cap 与总请求预算按 seed 结构分档：零切换最多 `2.0 s`；多于一次切换仍请求基础 `1.5 s`，每个节点 NLP cap 为 `0.75 s`；恰有一次切换且估算折叠 horizon 不超过 10 knots 时请求 `4.0 s`、节点 cap 使用基础 `3.0 s`；单切换但超过 10 knots 时回到 `1.5 s` 基础请求。规则只依赖模式切换数和折叠 knot 数，不使用地图编号。demo 配置和节点兜底默认值已同步，`joint.enabled:false` 与 CBC 插件 opt-in 不变。

N05 matched 配对里，输入 seed、单个后置邻域候选数和其它 MINLP 参数保持一致。Bonmin 实际用时从 base-cap `3.0 s` 的 `3.97 s` 降至 adaptive `0.75 s` 的 `2.41 s`；Planner 从 `15.05 s` 降至 `13.67 s`，最终共同分两次均为 `24.626115`，独立 5 mm swept-SAT 均通过。两次都插入安全 CBC seed，Bonmin 返回的模式字与 seed 一致；分数改善仍来自 MINLP 后固定模式邻域，不能记作 Bonmin 新模式收益。

N03 在本轮 12-knot 单切换 seed 上回到基础 `1.5 s` 请求，Bonmin wall `3.27 s`，仍返回全 Spin 非候选点，安全保留 seed `17.542490`，Planner `15.13 s`，独立扫掠通过（3,433 samples）。另一条 12-knot seed `17.411225` 请求 `4.0 s` 后同样未找到更好模式，Bonmin 用时 `7.04 s`、Planner `19.15 s`。两次上游 seed 分数不同，故只能说明高 knot 单切换 case 值得少花预算，不能当作严格匹配的时延对照。此前 10-knot N03 seed 曾在 4 秒档找到全 A；保留 `<=10` knots 的长预算分支。

N04 的 20-knot / 4-switch seed 使用 `0.75 s` 节点 cap，Bonmin wall `2.66 s`、Planner `11.70 s`，折叠模式仍为 `20102`、共同分 `32.600908`，没有离散模式收益；独立扫掠通过（5,692 samples）。返回模型残差有 4 项超过 `1e-5`，最大 `7.35e-5`，仍低于初值可行性诊断的 `1e-4` 阈值；应继续观察求解后残差。

当前采用此分档，原因是 N05 同分少用约 `1.38 s`，N04 也降低了求解耗时；N03 高 knot 单切换 seed 则不再请求长预算。该结果改善的是求解超时控制，不是多图 Bonmin 质量提升。后续重点应使 N03 类单切换模式收益对上游 seed 更稳定，并在 N04/N05 验证多切换档位。逐项日志在 [`relust/adaptive_minlp_budget_by_switch_count_20261008.json`](../relust/adaptive_minlp_budget_by_switch_count_20261008.json)。Python 语法检查通过；隔离 master `11409`–`11411` 均已停止，预存 `11311/11312/11320` 保持运行。


## 44. 2026-10-08 冻结 MINLP seed 的配对合并实验

### 可复用 seed 与平局顺序

为避免把不同上游路径误当成 MINLP 参数差异，新增两个 opt-in 诊断参数：`joint/seed_trace_file` 将实际选中的 MINLP 输入动作、起终点和栅格/车体签名写成 JSON；`joint/seed_replay_file` 在下一次运行重放该 seed。重放前要求地图签名、起终点完全匹配，并重新验证无倒车、无平移方向切换、终点容差和 `5 mm` 整车扫掠安全。两参数默认空，不改变常规规划。

粗 A*、混合路线图和二维路线图的优先队列此前都只按 `f` 排序。本轮补上 `id` 或 `(g,id)` 次序作为平局规则。N03 的粗搜索摘要保持 17 个动作、guided cost `25.979580`；此单图未观察到路线代价变化。显式规则方便后续复现实验，不代表随机波动已被证明来自队列。

### 同 seed 合并阈值配对

用上述 trace/replay 固定 seed，配对比较严格合并（A 曲率 `1e-5 1/m`、O 角度 `0.25°`）与温和合并（A 曲率 `0.1 1/m`、O 角度 `7.5°`）：

| 场景 | 固定 seed | 严格合并 | 温和合并 | 结果来源与扫掠 |
|---|---:|---|---|---|
| N03，有利 seed | `17.383056` / 11 actions | 10 knots，全 A，`15.207418` | 10 knots，全 A，`15.207418` | 两次均独立扫掠通过，3,406 samples；该 seed 不受合并阈值影响 |
| N03，高 horizon seed | `23.509489` / 20 actions | 20 knots，Bonmin 保持 seed | 13 knots，Bonmin `23.503046` | 最终 `17.411` 两次都来自 MINLP 后固定模式邻域；扫掠通过，3,420 samples |
| N04 | `32.679779` / 33 actions | 33 knots，超过 `max_knots=24`，Bonmin 未运行 | 22 knots，Bonmin 得到 `32.601110` | 根点 0 项违反、最大残差 `7.35e-5`；最终扫掠通过，5,697 samples |
| N05 | `26.297802` / 8 actions | 8 knots，Bonmin 保持 seed | 8 knots，Bonmin 保持 seed | 两次最终均 `24.626115`，来自后置模式邻域；扫掠通过，3,644 samples |

N04 是采用温和合并的主要理由：同一个 33-action 输入在严格阈值下无法进入 MINLP；温和阈值减到 22 knots 后，Bonmin 得到比 seed 低 `0.078669` 的共同分，而且终轨迹通过独立扫掠。正式 demo 原有 `max_knots=18` 仍会挡住这 22-knot 输入，因此同步把 MINLP 上限提高为 `24`；本次已用 24 上限跑通该 seed。N03 高 horizon seed 减少 7 knots、共同分低 `0.006443`，最终大幅下降仍属于 MINLP 后模式邻域。N05 和 N03 有利 seed 没有分数损失。由此将 demo/node 的 MINLP 专用默认值改成 `0.1 1/m / 7.5°` 与 `max_knots=24`；`joint.enabled:false` 保持不变，`core.py` 内固定模式优化的独立默认值也未改。

另对 N03 同一 seed 将 MINLP track weight 从 `0.015` 降为 `0`：共同分为 `15.207865`，略差于 `15.207418`；修复前 Bonmin 曾报告 `model_obj=-2581.99`。第 48 节随后确认，非激活模式绝对值辅助变量缺少模式上界会制造虚假负目标；因此该旧值不再解释为随机求解器异常。保留 track weight `0.015`，候选仍统一按共同轨迹分、模型残差和独立安全验证接收。

本轮 Python 语法检查与 catkin overlay 构建通过；N03/N04/N05 的所有最终 timed swept-SAT 均通过。实验使用的隔离 master `11413`–`11425` 均已清理。`11311/11312/11320` 保持运行且未触碰。完整配对数据在 [`relust/adaptive_minlp_budget_by_switch_count_20261008.json`](../relust/adaptive_minlp_budget_by_switch_count_20261008.json)。

### 下一步

保留 MINLP opt-in，先用新默认在 N01/N02/N06 和 P12 代表图复测 horizon、root 可行率与独立扫掠；重点确认更多高 knot seed 能否降到 `max_knots` 内。继续分别记录 Bonmin 的离散 schedule 收益、连续共同分改进，以及 MINLP 后固定模式邻域收益，不把三者合并记账。

## 45. 2026-10-08 N02/N06 冻结 seed 下的 CBC incumbent 与预算配对

本轮围绕 N02 重放实际 MINLP 输入 seed，拆分比较 `max_knots`、Bonmin 时间预算、CBC incumbent 注入与 seed cutoff。N02 seed 来自 `iterated_joint`，39 个动作全为 O；按 `0.1 1/m / 7.5°` 合并后只有 12 knots。18 和 24 knots 两档都能覆盖这个输入，因此 knot 上限并非当前瓶颈。两档 Bonmin 都返回全 Spin 无平移点，系统安全回退到共同分 `71.089430` 的 seed；把请求时限提高至 5 s 仍没改变候选。

启用已构建的 `bonminseed` CBC incumbent 插件后，Bonmin 在同一冻结 seed 上稳定返回有效全 O 候选：1.5 s 请求重复两次共同分均为 `69.680436`，比 seed 低 `1.408994`（约 1.98%）；0.5 s 也得到同分候选。5 s 与关闭 seed cutoff 都没有再降低分数，只增加用时。返回模式字仍是全 O，故目前证实的是 CBC 注入让求解器能够从可行整数初值开始并改善连续控制，不是离散模式词变化。无 CBC 时 0.5/1.5/5 s 均没有终点可行整数 incumbent。N02 每次最终路径均通过独立整车 swept-SAT，终点误差约 `0.0039 m / 0.000004 rad`。

再用 N06 同一冻结 seed 比较 0.5 与 1.5 s：16 个全 A 动作折叠成 8 knots。0.5 s 候选与 seed 同分 `13.776024`；1.5 s 返回 `SUCCESS` 候选 `13.707435`，连续代价下降约 0.50%。两次都通过独立扫掠。N02 的全 O seed 在 0.5 s 已得到完整改进，N06 全 A seed 则需要 1.5 s，因此不按模式切换数统一压低全局预算。

本轮决定：`joint.enabled:false` 继续保持；暂不把 0.5 s 或 5 s 设为通用时限。CBC 可行 incumbent 是 N02 候选可用的必要差异，N03/N05/N06 既有配对也表明它能提高候选可行率；N01 又验证了 best-local seed 碰撞后可回退到安全 coarse seed。`run_demo.sh` 现在会在插件已构建时将其目录加入 `CASADIPATH`。P12_env2 的首轮 0.8 尺度配对显示约 0.77% 分数收益，但模型目标曾出现负值；第 48 节已定位并修复这个模型漏洞，修复后的对照仍复现同一收益，规划额外用时约 0.52 s。N01/N02/N06/P12_env2 当前 Bonmin 模式字均与 seed 一致。

逐次设置、分数、耗时与扫掠采样见 [`relust/N02_global_minlp_frozen_seed_cbc_probe_20261008.json`](../relust/N02_global_minlp_frozen_seed_cbc_probe_20261008.json)。本轮只用隔离 master `11426`–`11437`，均已清理；保护端口 `11311/11312/11320` 未触碰。

## 46. 2026-10-08 N01 best-local seed 碰撞后的通用粗 seed 回退

N01 的默认 `best_local` MINLP 输入在 `joint_refine` 的 seed swept 检查处触发 `joint_seed_sat_collision`，Bonmin 未启动。随后本地候选修复仍返回安全轨迹，但 timed-check/clearance 阶段约耗 13.75 s。显式选择通用 `minlp_seed_source=coarse` 后，规划器已有的粗路线经相同终点、方向与整车扫掠验证，12 个全 O 动作折叠为 10 knots；CBC 提交 163 维 seed，Bonmin 返回共同分 `54.149886`，低于 coarse seed `55.844636` 约 3.03%。模式字仍是全 O，属于同一模式下的连续改进。最终路线仍选择共同分 `52.832` 的安全 fixed 候选，所以这次没有端到端路线分数收益。

已在 `optimizer_node.py` 增加通用回退：当默认 best-local seed 因 `joint_seed_sat_collision` 被拒、没有 frozen replay 且 coarse route 通过安全/终点门控时，把 coarse seed 加入尝试队列。成功的 best-local seed 不会触发第二次 MINLP；显式 `coarse` 与 frozen replay 路径不变。N01 自动回退复跑得到同一个 Bonmin 分数 `54.149886`，最终仍是 fixed `52.832`，独立 timed swept-SAT 通过（3,792 samples）。该单次结果提高了 MINLP candidate availability，但观察到的 Planner 总用时 `29.97 s`，高于回退前单次 `19.18 s`；由于局部求解与 clearance 修复耗时也有变化，暂不把差值全部归因于新增 Bonmin 求解。全局开关仍默认关闭，CBC 注入仍默认关闭。

为让以后显式启用 `joint/use_cbc_seed_incumbent` 时找到插件，`run_demo.sh` 在 `libcasadi_nlpsol_bonminseed.so` 已构建时自动将其目录加入 `CASADIPATH`；插件构建仍由 `tools/build_bonminseed_plugin.sh` 提供，本轮未改变 MINLP 默认参数。Python 语法检查和 optimizer catkin build 通过；run_demo shell 语法检查通过。

N01 对照原件：[`relust/N01_global_minlp_safe_coarse_fallback_20261008.json`](../relust/N01_global_minlp_safe_coarse_fallback_20261008.json)。P12_env2 已完成 0.8 尺度匹配实验，结果见第 47、48 节；N01 的 coarse fallback 仍只提高了 Bonmin 自身候选分，没有改善最终选择。后续继续用其他地图检查修正后的目标项与候选可用率，默认开关暂不调整。隔离 master `11438`–`11440` 已停止，`11311/11312/11320` 未触碰。

## 47. 2026-10-08 P12_env2 0.8 尺度的 MINLP / CBC 配对

按现有地图脚本将 P12_env2 以 0.8 比例绕地图中心缩小，使用固定起终点 `(10, 3.8, 1.5708) -> (4, 7, 3.14159)`。捕获的 seed 来自 `fixed_schedule_refine`，18 个 A 动作折成 7 knots，score `10.312117`。首轮配对和后续修正重跑都保存在 [`relust/P12_env2_global_minlp_cbc_matched_20261008.json`](../relust/P12_env2_global_minlp_cbc_matched_20261008.json)。

| 配置 | Bonmin | 共同分 | Planner 用时 | 独立 swept-SAT |
|---|---|---:|---:|---|
| joint-off 局部基线 | 未运行 | `10.311`（日志精度为 0.001） | `4.22 s` | 通过，2,285 samples |
| CBC seed 注入开启（修复前） | `SUCCESS` | `10.232058` | `5.79 s` | 通过，2,256 samples |
| CBC seed 注入关闭，冻结 seed（修复前） | `SUCCESS` | `10.232058` | `5.75 s` | 通过，2,256 samples |
| CBC seed 注入关闭，冻结 seed（修复后） | `SUCCESS` | `10.232058` | `4.71 s` | 通过，2,256 samples |
| CBC seed 注入开启，冻结 seed（修复后） | `SUCCESS` | `10.232058` | `4.76 s` | 通过，2,256 samples |

修复后 MINLP 相对 seed 降低 `0.080059`（约 0.776%），相对 joint-off 最终分数约降低 0.77%；规划用时增加约 0.52 s。返回的 7 位模式字仍为 `0000000`，即全 A，没有离散模式变化。CBC-on/off 的候选完全相同，因此这张图不支持把 CBC 注入作为普遍候选可用率提升的充分条件。

修复前的负模型目标并非 Bonmin/CasADi 返回值错位，而是双线性 O 使用项允许数值漏洞：非激活 O knot 的 `phi_abs` 没有随 O 模式关闭的上界，`ds_obl` 可能在容差内略小于 0，求解器便把无界的 `phi_abs` 放大，以 `ds_obl*phi_abs` 取得虚假的大负值。第 48 节为 `spin_abs`、`kappa_abs`、`phi_abs` 增加与对应模式绑定的上界。修复后 P12 CBC 开/关的 `model_obj=10.235648`，目标重构核验通过，最大约束残差约 `1e-8`；共同轨迹分和 swept-SAT 结果不变。之前负目标值运行保留为历史记录，但不再用于评价求解器目标质量。

继续保持 `joint.enabled:false`、CBC 注入默认关闭；下一步用不同模式组成的场景检查模型约束和候选可用率。P12 首轮使用的隔离 master `11441`–`11443` 已关闭；`11311/11312/11320` 未触碰。

## 48. 2026-10-08 修复 MINLP 绝对值辅助变量的无界漏洞，并复验 P12/N05

P12 目标分项审计定位到负值来源：目标含双线性项 `ds_obl * phi_abs`，但 `phi_abs` 只有 `phi_abs >= ±phi`，缺少 `phi_abs <= (π/2) * mode_O`。在 A/Spin knot 上 `phi_abs` 因而无上界；Bonmin 又允许 `ds_obl` 落在约 `-1e-8` 的可行性容差内，两个变量相乘可造出约 `-99.84` 的假收益。为三个绝对值 epigraph 都补上物理范围与模式联动的上界：`spin_abs <= max_spin * mode_S`、`kappa_abs <= (1/0.810) * mode_A`、`phi_abs <= (π/2) * mode_O`。这些约束与各自控制量的已有模式界一致，不限制任何有效解，也消除非激活模式辅助变量的漂移。

P12_env2 同一冻结 seed、CBC 开/关修复后均返回全 A、共同分 `10.232058`，比 seed `10.312117` 低约 0.776%。Bonmin 模型目标 `10.235648` 与独立目标重构一致，残差最多约 `9.92e-9`；CBC 插件提交的最终 incumbent 目标 `10.235647727` 通过一致性检查。Planner 用时约 `4.71–4.76 s`，相较 joint-off `4.22 s` 多约 `0.52 s`。两次独立 swept-SAT 均通过，各检查 2,256 samples。修复前得到的负目标旧运行仍保存在 P12 JSON 中，并已标为不应用于求解器质量评估。

再在 O/A 混合比例较高的 N05 复验冻结 seed（6 个 O、2 个 A，8 knots）。CBC 插件将 131 维 seed 作为 incumbent 提交，Bonmin 因短预算返回 `LIMIT_EXCEEDED`，模式字仍为 `11110011`；模型目标为正的 `26.524241`，CBC 内部目标复核一致，模型残差最多 `3.9e-7`。Bonmin 共同分与 seed 持平 `26.297803`，所以不记作 MINLP 收益。通用 `joint_mode_neighborhood` 后处理将最终共同分降到 `24.588`，较冻结 seed 低约 6.50%；最终轨迹无倒车、O 角更新为 0，独立 swept-SAT 通过（3,638 samples，终点误差 `0.017418 m / 0.068929 rad`）。这确认新增上界没有阻止该混合模式 seed 的复现与后续安全改进，但短预算下 N05 的收益来自后处理，不是 Bonmin 本身。

修复位于 [`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py`](../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py)，`py_compile` 与 optimizer catkin build 通过。P12 数据见 [`relust/P12_env2_global_minlp_cbc_matched_20261008.json`](../relust/P12_env2_global_minlp_cbc_matched_20261008.json)，N05 混合模式复验见 [`relust/N05_global_minlp_mode_bounds_replay_20261008.json`](../relust/N05_global_minlp_mode_bounds_replay_20261008.json)。当前仍保持 MINLP 与 CBC 默认关闭；下一步继续对 N02/N03/N06 等冻结 seed 做修复后重放，重点记录 Bonmin 真正降低共同分的场景、候选可用率和新增的规划时延。隔离 ROS master `11445`–`11450` 已清理，保护端口 `11311/11312/11320` 未触碰。

## 49. 2026-10-08 N03 在目标修复后的全局 MINLP 复验

使用 N03 正式起终点 `(0.85, 1.25, 0) -> (11.05, 6.75, 0)`，在第 48 节绝对值辅助变量模式上界修复后重新运行。进入 MINLP 的通用模式邻域 seed 共同分为 `17.387425`，10 个动作折成 9 knots，只有一次模式切换。CBC 在搜索前成功提交 147 维可行整数 seed；Bonmin `SUCCESS`，返回 9 位全 A 模式字 `000000000`，共同分降到 `15.202620`，降低 `2.184805`（约 12.565%）。模型目标为正的 `15.218131`，CBC 与 TMINLP 最终 incumbent 目标一致，模型约束残差 0 项超过门限、最大约 `9.85e-9`。

最终来源为 `joint_minlp`，Planner 用时 `12.67 s`，其中 Bonmin `1.62 s`、整个 joint 调用 `1.99 s`。最终路线无倒车、无 O 角更新；独立 5 mm 整车 swept-SAT 通过（117 segments / 3,400 samples），终点误差 `0.004963 m / 0.028199 rad`。这说明目标漏洞修复后，N03 的明显收益仍然成立，而且属于 Bonmin 把混合模式改为全 A 的离散模式收益。

同轮第一次临时探针误把 N03 目标航向写成 `-0.35 rad`，checker 却按正式目标 `0 rad` 验收，因此产生 `yaw=0.336776` 的 checker failure；该结果属于实验输入错误，已从算法结论中剔除。正确复验记录见 [`relust/N03_global_minlp_mode_bounds_replay_20261008.json`](../relust/N03_global_minlp_mode_bounds_replay_20261008.json)。隔离 master `11451/11452` 已关闭，保护端口未触碰。

## 50. 2026-10-08 WP0：统一 case 清单与可复现 ROS runner

新增 [`tools/expv3_eval_cases.yaml`](../tools/expv3_eval_cases.yaml)，收录 N01–N06 的正式 `scene_meta.yaml` 起终点，以及 P12_env2 ×0.8 已验证探针起终点。新增 [`tools/run_expv3_probe.py`](../tools/run_expv3_probe.py)：从同一 case 读取 publisher 和独立 checker 的输入，支持隔离 ROS master、joint/CBC 开关、已有参数覆盖、seed capture/replay、地图缩放、地图栅格签名预检、配置与代码哈希、完整日志和 JSON 结果。Replay seed 会复制进本次结果目录。`demo.launch` 增加可选 `config_file` 参数，默认仍加载正式参数文件；每轮实验只加载输出目录中的配置副本。独立探针增加 case 终点容差、单调时间、方向速度和模式序列记录，并把最终终点与 5 mm swept-SAT 纳入退出门槛。optimizer 的候选日志增加共同分、模型目标重构误差和各目标分项，便于后续目标审计；不改变求解行为。

N03 在相同 N03 case、joint on、CBC on、相同默认参数下完成两次 seed capture 和两次 frozen-seed replay。两次 capture 和 replay 的 Bonmin 状态均为 `SUCCESS`，seed score 均为 `17.387425`，Bonmin 候选共同分均为 `15.202620`，模型目标为 `15.218131`，最大模型残差为 `9.85e-9`，全 A 模式字 `000000000`。每次最终轨迹均无倒车、终点误差约 `0.004963 m / 0.028199 rad`，独立 swept-SAT 均通过（117 segments / 3,400 samples）。runner 记录的 Planner 时间为 `11.85–12.05 s`，Bonmin 时间为 `1.43–1.51 s`。模型分项与完整启动配置保存在每轮 `run.json` 和 `roslaunch.log` 中。

有效结果目录：[`N03_default_on_cbcon_20261008T071449Z`](../relust/probes/N03_default_on_cbcon_20261008T071449Z)、[`N03_default_on_cbcon_20261008T071515Z`](../relust/probes/N03_default_on_cbcon_20261008T071515Z)、[`N03_default_on_cbcon_20261008T071732Z`](../relust/probes/N03_default_on_cbcon_20261008T071732Z)、[`N03_default_on_cbcon_20261008T072014Z`](../relust/probes/N03_default_on_cbcon_20261008T072014Z)。首轮 runner 集成运行发现 roslaunch 子节点会脱离启动进程组，因此曾错误报告清理失败；runner 已改为按本轮 `ROS_MASTER_URI` 记录并关闭子节点，后续四次结果均记录 6 个 ROS 节点、roslaunch、roscore 全部停止且端口已释放。该故障运行未纳入复现次数。

同一轮还发现 11 个更早的 optimizer 孤儿进程，其 ROS master URI 分别为已关闭的隔离端口 `11441–11452`，没有对应活动 master；已只对这些精确进程发送 SIGINT 并确认全部退出。保护端口 `11311/11312/11320` 未触碰。完整 `catkin_make`、Python `py_compile` 和 launch XML 检查通过。WP0 验收完成；下一步按 WP1 用冻结 seed 复验 N02、N04、N05、N06、P12_env2，并把 Bonmin 模型目标重构、分项、约束残差与最终安全检查汇总到运行记录。

## 51. 2026-10-08 WP1：修复后 MINLP 目标审计与 CBC 配对

使用 WP0 runner 对 N02/N03/N04/N05/N06 与 P12_env2 ×0.8 完成 CBC on/off 配对。N03 的主要配对使用同一冻结 seed；N02/N04/N06 从 CBC-on 捕获的 seed 复制到 CBC-off replay；N05/P12 使用同一冻结 seed。汇总和逐轮原件见 [`relust/expv3_wp1_mode_bounds_replay_20261008.json`](../relust/expv3_wp1_mode_bounds_replay_20261008.json)。

| 场景 | seed 共同分 | CBC-on Bonmin 候选 / 模型目标 | 候选最大约束残差 | CBC-on 最终分 / 来源 | CBC-off 最终分 / 来源 | 安全检查 |
|---|---:|---:|---:|---|---|---|
| N02 | 71.089430 | 69.680436 / 69.681940 | `8.67e-19` | 69.680 / `joint_minlp` | 71.089 / `iterated_joint`（没有模型目标候选） | 5,252/5,294 样本均通过；无倒车；末端 `0.00388 m / 0 rad` |
| N03 | 17.387425 | 15.202620 / 15.218131 | `9.85e-9` | 15.203 / `joint_minlp` | 15.203 / `joint_minlp` | 两边均 3,400 样本通过；全 A；末端 `0.00496 m / 0.0282 rad` |
| N04 | 32.543069 | 32.530724 / 32.874580 | `6.76e-5` | 32.531 / `joint_minlp` | 32.543 / `fixed_schedule_refine`（没有模型目标候选） | 两边均通过；模式序列 `S-A-O-A-S`；末端 `0.0523 m / 0.0827 rad` |
| N05 | 26.297803 | 26.297803 / 26.524241 | `3.9e-7` | 24.588 / `joint_mode_neighborhood` | 24.588 / `joint_mode_neighborhood` | 两边均 3,638 样本通过；无倒车；末端 `0.0174 m / 0.0689 rad` |
| N06 | 13.776024 | 13.707394 / 13.739963 | `9.95e-9` | 13.707 / `joint_minlp` | 13.707 / `joint_minlp` | 两边均 2,973 样本通过；全 A；末端 `0.00858 m / 0.0502 rad` |
| P12_env2 ×0.8 | 10.312117 | 10.232058 / 10.235648 | `9.92e-9` | 10.232 / `joint_minlp` | 10.232 / `joint_minlp` | 两边均 2,256 样本通过；全 A；末端 `0.00500 m / 0.00894 rad` |

所有有 Bonmin 目标记录的候选，其目标均有限且非负，模型目标重构误差不超过 `7.11e-15`。WP1 对候选收益采用保守 `1e-5` 约束残差审阅门槛；N04 最大残差 `6.76e-5`，所以它的候选不用于 MINLP 收益结论。代码里的 `1e-4` 是输入初值诊断容差，不是候选接受门槛。N04 同一 frozen seed 的加时诊断将请求从 `1.5 s` 提到 `6.0 s`、子问题预算提到 `4.0 s`，结果、目标和残差完全没变，状态由 `LIMIT_EXCEEDED` 变成 `SUCCESS`；因此延长预算没有修复这个约束残差。

可归因结果：N03 的 seed→Bonmin 候选下降约 `12.57%`，模式字从混合 seed 变为全 A，且 CBC on/off 均重现；这是已复现的离散模式收益。P12 与 N06 的改善约 `0.78%`、`0.50%`，保持全 A，属于连续优化收益。N05 最终从 `26.298` 降到 `24.588`，但 Bonmin 候选与 seed 相同，且 CBC on/off 最终都由 `joint_mode_neighborhood` 得到该结果；收益来自后置邻域，不归因于全局 MINLP。N02 的 CBC-on 候选低于 seed，CBC-off 未返回模型候选，因此只能确认该次候选的连续改善，不能宣称 CBC 消融等价。N04 的小幅候选差也因残差门槛排除。

六个正式配对的最终轨迹都通过终点容差、严格递增时间、无倒车和独立 5 mm swept-SAT。每次隔离 master、roslaunch、已启动 ROS 节点均清理完毕，端口释放；未触碰 `11311/11312/11320`。WP1 验收完成，N04 残差保留为显式问题。下一步进入 WP2，先处理重复模式邻域和可用候选上的阶段调度，保留当前安全回退。

## 52. 2026-10-08 WP2：MINLP 后处理去重与条件调度

在 optimizer 里给前/后模式邻域共用一个折叠 schedule key：记录模式、档位、方向、5 cm 量化的 knot 边界以及 0.01 量化控制量。前置邻域确实求解过的候选，后置阶段不再重复做固定 schedule NLP。Bonmin 相对 seed 的共同分改善达到门槛、模型目标和约束审计通过、模式复杂度不增加时，跳过后置邻域。默认相对改善门槛设为 `0.5%`；对没有显著 MINLP 收益的候选，后置邻域仍保留。

端点 S-O-S 增加通用热点评分门槛 `1.5`。仅在局部模式/档位事件或转向变化达到该门槛时生成宏候选；还须有至少一个上游无碰撞 S-O-S 窗口，才进入后缀重规划。Profile 新增热点分数/门槛、净空窗口数、前/后邻域 schedule 生成/求解/接受数、重复跳过数、跳过理由码和阶段时长。N05 的低强度热点分数为 `1.088`，原算法因此启动 10 个宏尝试和一次约 `2.1 s` 的后缀搜索但没有安全改进；门槛将该场景的端点 S-O-S 搜索跳过。N03 分数 `2.0`，仍保留原来的四个净空窗口尝试；P12 分数 `2.0`，仍保留一个窗口。没有地图名或编号条件。

WP2 还发现局部模式邻域先对当前 schedule 额外做一次固定模式 NLP，而前一阶段已对同一 incumbent 做过连续优化。MINLP 开启时默认跳过这个冗余 base NLP，保留实际 A/O/Spin 邻域候选生成和精修；MINLP 关闭时仍走原 base refinement 路径，避免改变用户当前默认 opt-out 行为。N05 冻结 seed 配对和正常 seed capture 都保持 `24.588`。

| 场景 | WP1 Planner 时间 | WP2 时间 | 时间变化 | 共同分 | 最终模式 / 安全 |
|---|---:|---:|---:|---:|---|
| N03 | `11.99 s` | `8.35 s` | `-30.4%` | `15.202620` 不变 | 全 A；3,400 点 5 mm sweep 通过；无倒车 |
| N05 | `10.84 s` | `7.71 s`（捕获 seed `7.67 s`） | `-28.9%` | `24.588440` 不变 | O-A-O；3,638 点通过；无倒车 |
| P12_env2 ×0.8 | `4.60 s` | `2.42 s` | `-47.4%` | `10.232058` 不变 | 全 A；2,256 点通过；无倒车 |

N03 的后置邻域因 Bonmin 改善 `12.565%` 且复杂度从 `(1 次模式切换, 0 次 O 航向更新, 1 个 Spin)` 降至 `(0,0,0)` 而跳过；离散 MINLP 收益保留。P12 连续改善约 `0.776%` 且复杂度相同，因此在 `0.25%/0.5%` 档跳过；`1%` 档不跳过，不过没有额外邻域候选，最终分数不变。N05 Bonmin 候选与 seed 相同，所以后置邻域继续运行并接受一个候选；最终 `24.588` 收益来自 `joint_mode_neighborhood`，不归因于 MINLP。

对 `0.25%/0.5%/1.0%` 三档完成了 N03/N05/P12 冻结 seed sweep；三档的分数、终点和安全状态一致，`0.5%` 作为默认值。消融前置模式邻域导致 N05 退化至 `26.356`，不采用；减少 alternating rounds 导致正常 seed capture 的 N05 退化约 `0.74%`，不采用；把后置 NLP 时限缩到 `0.25 s` 得到 `24.972`，不采用；`0.5 s` 虽回到 `24.588`，Planner 时间没有稳定下降，保留 `1.0 s`。

逐轮配置、seed、目标、残差、profile 与独立 checker 结果在 [`relust/expv3_wp2_schedule_profile_20261008.json`](../relust/expv3_wp2_schedule_profile_20261008.json)。WP2 的 score 与安全门槛、N03/N05 至少 `20%` 时延下降均通过；WP2 完成。下一步进入 WP3：实现曲率与 clearance 驱动的连续自适应 route graph spacing，先以 N03/N04/N05/N06 冻结 seed 配对。


## 53. 2026-10-08 WP3：曲率 / 净空连续自适应 route graph 锚点

在 `planner_node.cpp` 中为 route graph 增加连续 spacing 分支，需求为曲率压力与保守净空压力的较大值，按 `spacing = sparse - u*(sparse-dense)` 插值。用 90°作为满曲率需求，避免把栅格路径常见的 45°过渡判为满需求；曲率率峰值做空间非极大抑制。路径起终点保留，显著原始拐角、曲率符号变化、曲率极值、曲率率极值和净空局部极小作为强制锚点。模式候选仍由现有统一碰撞可达性和共同代价竞争，没有按地图或曲率门控模式。新增了路径长度、锚点密度分类、强制锚点原因、DP 全状态数、DP 候选边 / 可行边，以及 route graph wall time 字段。旧二档分支仍可通过参数复现。

初轮将 45°栅格转向直接除以旧 `0.8 rad` 密集阈值，造成过采样（N03 自适应锚点最多 62 个，曲率率锚点占绝大多数）；这版已丢弃。改为满量程 90°并抑制相邻曲率率峰值后，采用 `dense=0.30 m / sparse=1.00 m / continuous=true`。按相同代码、冻结 seed 和优化配置做二档基线 / 连续自适应配对，另外检查 P12_env2 ×0.8。

| 场景 | 粗图 wall time | DP 候选边 | 最终共同分 | 搜索模式切换 | 最终模式序列 | 5 mm sweep / 终点 |
|---|---:|---:|---:|---:|---|---|
| N03 | `0.130→0.112 s`（-13.9%） | `49,140→30,960`（-37.0%） | `15.203→15.203` | `2→0` | A→A | 通过 / 通过 |
| N04 | `0.226→0.213 s`（-5.7%） | `57,933→40,950`（-29.3%） | `32.531→29.335` | `4→2` | S-A-O-A-S→S-A-S | 通过 / 通过 |
| N05 | `0.127→0.100 s`（-21.6%） | `36,684→18,063`（-50.8%） | `24.588→24.588` | `0→0` | O-A-O→O-A-O | 通过 / 通过 |
| N06 | `0.155→0.124 s`（-19.9%） | `18,117→15,912`（-12.2%） | `13.707→13.707` | `0→0` | A→A | 通过 / 通过 |
| P12_env2 ×0.8 | `0.092→0.094 s`（+1.8%） | `23,562→18,666`（-20.8%） | `10.232→10.232` | `0→0` | A→A | 通过 / 通过 |

N03/N04/N05/N06 的候选边下降中位数为 `33.2%`，超过 WP3 的 `25%` 门槛；各场景中位共同分变化为 `0%`，没有增加模式切换或最终 O 航向更新。粗图耗时中位下降 `16.9%`；P12 没有耗时收益，但边数减少、质量不退化。所有 run 的独立整车 swept-SAT、终点、无倒车和 ROS 清理都通过。新默认参数做了无 override 的 N03 replay，仍为全 A、score `15.202620`、独立 sweep 通过。

逐场景 route 详情、完整 run 目录、冻结 seed、参数和代码哈希见 [`relust/expv3_wp3_adaptive_spacing_20261008.json`](../relust/expv3_wp3_adaptive_spacing_20261008.json)。二档对照与连续候选均使用隔离 ROS master `11494–11510` 中实际启用的端口；全部停止并释放，没有触碰保护端口 `11311/11312/11320`。WP3 通过；下一步执行 WP4：把净空纳入 hotspot，检查并完善在上游宽阔区提前调航向后重规划后缀的通用候选。


## 54. 2026-10-08 WP4：上游调姿后缀重规划首轮多场景诊断

WP4 已把终端热点判断改为通用几何/行为特征：近端净空、Ackermann 曲率总变差、O 航向总变差与更新数、模式/Spin 事件、剩余路程和目标航向需求。只有热点达到门槛且存在上游净空窗口时，才尝试保留原路线尾段的局部重规划。候选包含纯 A、固定 A-O-A、提前 Spin+A；有独立精确 swept-SAT、正向运动、终点和共同分准入。阈值、窗口和候选序列均不依赖地图编号。

当前参数为 hotspot 分数门槛 `1.0`、目标航向需求只看末端 `8 m`、热点净空 `0.585 m`、上游参考净空 `0.54 m`、最多 2 个窗口与 3 次搜索。首轮结果：

| 场景 | hotspot | 重规划情况 | 最终共同分 / 模式 | 5 mm sweep |
|---|---:|---|---|---|
| P12_env2 ×0.8 | `0.866` | 未触发，低于门槛 | `10.232` / 全 A | 2,256 点通过 |
| N03 | `0.652` | 未触发，低于门槛 | `15.203` / 全 A | 3,400 点通过 |
| N05 | `0.597` | 未触发，低于门槛 | `24.588` / O-A-O，O 航向无额外更新 | 3,638 点通过 |
| N04 | `1.330` | 找到一个上游窗口；纯 A、A-O-A、Spin+A 均做候选搜索 | `29.335` / S-A-S | 5,872 点通过 |

N04 的末端热点确有较高曲率和 `0.372 m` 局部净空，但上游窗口净空只有约 `0.552–0.553 m`。纯 A 后缀候选可生成，然而共同分高于当前路线而被拒；A-O-A 与提前 Spin+A 没有更好的可行候选。N04 最终行为、安全和分数都没有退化，但本轮没有得到提前调姿收益，不能标记 WP4 验收通过。下一轮要改进候选窗口长度和上游姿态需求预测，并观测搜索成本；继续保留共同分和 swept-SAT 双门槛。

原件见 `relust/probes/N04_default_on_cbcon_20261008T083351Z`、`relust/probes/P12_env2_x0p8_on_cbcon_20261008T081821Z`、`relust/probes/N03_default_on_cbcon_20261008T081911Z`、`relust/probes/N05_default_on_cbcon_20261008T081943Z`。本轮 ROS 端口均由 runner 记录清理并释放。


## 55. 2026-10-08 WP5：Bonmin 前压缩 seed 安全预检

MINLP seed 现在先按 Bonmin 实际使用的动作合并阈值压缩，再逐个检查前向方向、最大 knot 数、终点误差和全车 `2.5 mm` swept-SAT。只有压缩后仍安全的候选才参加共同分排序；Bonmin 每轮只接收一个已审计 seed。冻结 seed replay 同样执行该审计；无效 replay 会退回到压缩后安全的本地候选。CBC incumbent 注入仍只提交最终选出的安全 seed。

| 场景 | 预检候选 | 压缩碰撞拒绝 | 选中 seed | Bonmin / 最终结果 | 最终安全 |
|---|---:|---:|---|---|---|
| N01 | 8 项，5 项本地候选合格 | 2 | `anchor_graph_compressed`，10 knots / 4 switches | Bonmin 返回与 seed 同分 `46.587758`；最终 `45.790` 来自 `mode_master` | 3,779 点通过；终点、正向通过 |
| N03 | 5 项，4 项合格 | 0 | 13 knots / 0 switches | 全 A，共同分 `15.203` | 3,400 点通过 |
| N05 | 6 项，5 项合格 | 0 | 7 knots / 2 switches | Bonmin 保持 seed；最终 `24.588` 来自模式邻域 | 3,638 点通过 |
| P12_env2 ×0.8 | 8 项，5 项合格 | 0；另有 2 项终点拒绝 | 8 knots / 0 switches | 全 A，共同分 `10.232` | 2,256 点通过 |

N01 验证了关键安全性质：2 个 seed 在压缩后被碰撞预检挡住，没有进入 Bonmin；有效 seed 只启动一次 Bonmin，轨迹仍由全流水线共同分、安全和终点检查决定。N03/N05/P12 的模式和安全验收保留。WP5 完成。逐轮计数位于各 `run.json` 的 optimizer profile；N01 原件为 `relust/probes/N01_default_on_cbcon_20261008T083813Z`。


## 56. 2026-10-08 WP6：MINLP 目标与共同分同 seed 排序审计

为每个 Bonmin 候选日志补记 `seed_model_obj`，probe runner 解析为 `seed_model_objective`，从而能在同一个优化模型里比较 seed 和候选的模型目标下降方向，再与共同分变化方向对照。冻结 seed 配对使用 N03、N05、N06 和 P12_env2 ×0.8；另对每个完全相同的 seed 运行 `minlp_track_weight=0` 对照。

| 场景 | seed 共同分 / 模型目标 | 默认候选共同分 / 模型目标 | track=0 候选共同分 / 模型目标 | 模式 |
|---|---|---|---|---|
| N03 | `17.387425 / 17.423312` | `15.202620 / 15.218131` | `15.202541 / 15.212942` | 全 A；有明确离散改善 |
| N05 | `26.297803 / 26.524241` | `26.297803 / 26.524241` | `26.297803 / 26.524241` | O-A-O；Bonmin 持平 |
| N06 | `13.776024 / 13.809621` | `13.707394 / 13.739963` | `13.707375 / 13.739897` | 全 A；小幅连续改善 |
| P12_env2 ×0.8 | `10.310796 / 10.316188` | `10.232058 / 10.235648` | `10.231977 / 10.235257` | 全 A；小幅连续改善 |

四组候选中，模型目标与共同分相对各自 seed 的变化方向一致：3 组共同改善，N05 两者持平，没有观察到模型更优而共同分更差的排序冲突。将 track 正则从 `0.015` 置零没有改变 Bonmin 模式字；共同分最大变化只有 `0.000081`，模型目标变化最大 `0.005189`，均未改变候选接受或排序结论。因此保留 `minlp_track_weight=0.015`，不根据无显著收益的消融改目标。

四组默认和 track=0 运行都通过终点、无倒车和独立 swept-SAT；模型重构误差在浮点精度内，候选最大残差低于 `1e-5`。逐轮目标、目标分项、配置、地图签名和清理信息见 [`relust/expv3_wp6_objective_alignment_20261008.json`](../relust/expv3_wp6_objective_alignment_20261008.json)。WP6 的这批 seed/candidate 对比已完成；仍需在后续候选丰富后记录多候选的完整排序冲突率。


## 57. 2026-10-08 WP7：走廊几何剪枝试验与交互回归

MINLP 新增三种独立 opt-in 几何剪枝，均由当前 MINLP 的局部 swept-feature corridor 推导，不使用地图名称：

1. `joint/geometry_yaw_bounds`：利用车体前后外侧特征圆的走廊差集，收紧每个 knot 必要的 body yaw 范围。
2. `joint/geometry_spin_pruning`：相邻中心走廊 AABB 不相交时，排除无法保持中心位置不变的 Spin 模式。
3. `joint/geometry_oblique_bounds`：利用相邻中心走廊位移方向，收紧 O 的航向角范围；若该走廊锥与可行 yaw/phi 范围不相交，则排除该 knot 的 O 模式。

候选仍需模型残差审阅、exact replay、终点和独立 swept-SAT。所有参数在正式配置默认关闭。N03 冻结 seed 配对结果：

| 几何界组合 | Bonmin 候选共同分 | 模式字 | 结果 |
|---|---:|---|---|
| 全关 | `15.202620` | `0`（全 A） | 保留基准离散收益 |
| yaw 单项 | `15.202620` | `0` | 保留收益 |
| Spin 单项 | `15.202620` | `0` | 保留收益；一次运行 Planner `3.30 s`，暂不把单次时延差当成稳定收益 |
| O 方向单项 | `15.202620` | `0` | 保留收益 |
| yaw+Spin | `15.202620` | `0` | 保留收益 |
| yaw+O 方向 | `15.202620` | `0` | 保留收益 |
| Spin+O 方向 | `17.387425` | `20`（Spin-A） | 退回 seed，丢失全 A 改善 |
| 三项全开 | `17.387425` | `20`（Spin-A） | 退回 seed，丢失全 A 改善 |

交互结果表明这些约束单项数学上由当前走廊推导，但同时改变 Bonmin 的非凸分支/连续子问题搜索行为；尤其 Spin+O 方向组合会使 N03 失去已复现模式收益。N04 全开或只开 Spin 时，MINLP 共同分均与 seed `32.530724` 持平；模型最大残差 `6.76e-5`，超过 `1e-5` 收益审阅门槛，不能记 MINLP 收益。两种配置的最终共同分仍为 `29.335`，来源于 `iterated_joint`，独立扫掠通过 5,872 点。N05 只开 O 方向界时保留 O-A-O seed 与 `24.588` 最终结果；其改善仍来自 `joint_mode_neighborhood`，不是 Bonmin。

所有几何界默认关闭，WP7 验收尚未满足：除 N03 外还没有场景出现新的、更优离散模式字。N01 冻结 seed 上把 `joint.mode_neighborhood_candidates` 从 1 提到 3，Bonmin 仍返回 seed `46.587758`，最终仍为 `45.790`；新增后邻域生成 6 项、求解 3 项、接受 0 项，Planner 从基线 `17.70 s` 到 `19.49 s`。最终路线仍是 O-A-O-A-O，因而不采用更大候选数。下一步不增大 Bonmin 时限，也不重复加邻域候选；改为把已通过安全检查但没有胜出的固定模式路线保留为一次 MINLP 的初值提示，输出仍与已有共同分 incumbent 竞争。先在 N01 做同 seed 配对，再决定是否进入 N04/N05。分项实验、几何界宽度、模式、共同分和安全清理见 [`relust/expv3_wp7_geometry_bounds_20261008.json`](../relust/expv3_wp7_geometry_bounds_20261008.json) 和 `relust/probes/WP7_N01_more_mode_candidates_20261008`。

## 58. 2026-10-09 核心重构 R01：新评价、时间规划与模式/路径主问题

用户已明确先定义好轨迹标准、再重构模式决策核心，后续以 [11_好轨迹评价规范与Cost设计.md](11_%E5%A5%BD%E8%BD%A8%E8%BF%B9%E8%AF%84%E4%BB%B7%E8%A7%84%E8%8C%83%E4%B8%8ECost%E8%AE%BE%E8%AE%A1.md) 为评价合同，不继续将旧 seed 邻域上的参数迭代当作全部研究方向。

已新增独立 `mode_path_v1` 后端：`quality.py` 提供整车距离与统一质量验收，`retiming.py` 提供有限加减速/jerk/角运动/轮速及转向速率的声明仿真 profile，`mode_master.py` 搜索模式结构并用全路径自由中间姿态 NLP 和独立评分反馈。现有 `legacy` 默认保留作对照，R01 分支不串接旧优化流水线，也不调用 Bonmin。

16 项新合同测试通过。N01–N06 与 P12_env2 ×0.8 七场景的冻结代码首轮运行全部通过独立扫掠、终点、正向位移和发布采样动态检查。新 cost 相对同一粗 seed 在相同能力 profile 下重定时分别降低约 57.0%、48.7%、21.2%、7.4%、47.5%、13.2%、18.8%；不能与历史等效距离分数直接比较。N01 O 调角 9→1，N02 13→4，N05 6→0。后端内部首个合格候选约 0.20–0.51 s，后端总时延约 7.50–7.80 s，仍需统一 C++ 与后端的端到端预算。

所有隔离端口 11580–11586 和子节点已清理，保护端口未触碰。原件：[relust/quality_refactor_R01_20261009/summary.json](../relust/quality_refactor_R01_20261009/summary.json)，使用方法、分项和归因见 [12_核心重构R01_实现与验证.md](12_%E6%A0%B8%E5%BF%83%E9%87%8D%E6%9E%84R01_%E5%AE%9E%E7%8E%B0%E4%B8%8E%E9%AA%8C%E8%AF%81.md)。早期试验的 N01 最佳单次 O 调角为零，不替代冻结版该轮为 1 的结果。

R01 未完成区域图、多拓扑初值银行、航向可达集合传播或连续曲率段。目前不同恒定 A 控制仍通过停车重配置执行，会高估一些真实平滑 A 路线的执行成本，影响 A/O 比较。下一步优先实现连续控制段及时间规划，再补全成对标定、区域/姿态表示和段级 MINLP 对照。轨迹净空极小值当前为 0.05 m 测度下的采样估计；动态参数为仿真配置，不是实车标定。不得宣称完整重构、稳定跨图全局最优或真实车辆执行验证已经完成。


## 59. 2026-10-09 R02：正向优先、局部倒车恢复与上游航向重规划

按用户最新要求更新方向合同：先找不含倒车的 A/O/Spin 解；预算内未找到合格正向解时，允许局部倒车恢复；全程倒车始终拒绝。新后端命令为 `./run_demo.sh --map P12_env1 planning_backend:=mode_path_v1 allow_reverse_fallback:=true`，回退开关默认 true，设为 false 禁止恢复。参数同时接到 C++ planner 和 Python optimizer，legacy 仍作为历史纯正向对照。

C++ 增加独立有界恢复搜索及前进历史状态；Python 正向阶段保留独立初始化器，只有零正向合格候选才接受混合方向初值，随后搜索通用上游调整点/航向和稳定 A/O 尾段，再做自由中间姿态 NLP 与真实质量验收。加入倒车距离和换向代价，速度保留负号且具有有限减速、停车和加速。独立检查不再直接拒绝所有负速度，而是核对方向符号、全程倒车、开关及恢复来源。没有地图状态机或地图专用权重；失败仅表示预算/离散表示下未找到，不作为连续不可行证明。

23 项合同测试、catkin 构建、参数展开检查通过。窄死胡同接宽房间的默认参数 ROS 配对：关闭回退不进入倒车搜索；打开回退通过整车扫掠、终点、符号及动态检查。上游结构选择将全 A 粗解变为 A-Spin-稳定 O，新 cost 8.870→8.080，倒车 3.313→2.200 m，A 曲率控制总变差 6.173→0.617；执行时间 19.740→23.323 s，增加约 18.2%，因此不能宣称所有指标变好。它是结构提议与真实评分的收益，不是全局 MINLP 收益。原件：[方向合同汇总](../relust/reverse_contract_R02_20261009/summary.json)。

最终冻结运行源码的 N01–N06 和 P12_env2×0.8 七场景均保持纯正向、未激活回退、独立安全/终点/动态/方向检查通过；各 run.json 源码哈希一致。后端约 7.50–7.70 s，无预算超出，隔离端口和所有子节点已清理。N01 当前仍有 2 次 O 调角、N02 4 次，不以历史 R01 更佳单次替代本轮数据；原件：[冻结七场景回归](../relust/quality_reverse_R02_frozen_20261009/summary.json)。

详见 [12_核心重构R01_实现与验证.md](12_%E6%A0%B8%E5%BF%83%E9%87%8D%E6%9E%84R01_%E5%AE%9E%E7%8E%B0%E4%B8%8E%E9%AA%8C%E8%AF%81.md) 第 8 节及 [评价规范 v1.1](11_%E5%A5%BD%E8%BD%A8%E8%BF%B9%E8%AF%84%E4%BB%B7%E8%A7%84%E8%8C%83%E4%B8%8ECost%E8%AE%BE%E8%AE%A1.md)。下一项仍优先完成连续 A 曲率/转向与速度时间化：当前不同 A 控制之间停车会扭曲 A/O/Spin 的执行时间比较，需纠正后再标定权重和接入段级 MINLP。上游调整点目前是有限初值银行，区域图和可达航向传播尚未完成。


## 2026-10-10 R04 手测低速问题修复补记

默认入口为 mode_path_v2。本轮从实际 ROS 日志冻结 P12_env4/P12_env2 起终点，确认并修复毫米级末端转向污染长区间限速、结点二阶导数侧别、整块统一速度缩放和重复计算；新增保持模式/方向词的连续控制平滑候选并恢复最终走廊显示。同输入 ROS 最终轨迹时长分别 18.488/24.854 s，后端状态耗时 4.74/6.09 s，纯 A、无中途零速点；整车扫掠/动态/终点验收通过。N03 补查通过，23 项定向测试通过。全部隔离 master 和子节点已清理。

详见 [R04 修复记录](14_R04_%E4%BD%8E%E9%80%9F%E5%BC%82%E5%B8%B8%E4%B8%8E%E8%B5%B0%E5%BB%8A%E6%98%BE%E7%A4%BA%E4%BF%AE%E5%A4%8D%E8%AE%B0%E5%BD%95.md)。安全走廊目前是最终路径包络，尚未接入 v2 连续 NLP；仿真限值未标定，时间支撑点仍强制零加速度，P12_env4 最小车身净空约 4.2 cm。下一步继续时间模型、净空成本、联合候选与连续走廊约束；其它手测地图问题、地图锯齿和全局 MINLP 尚未完成。
