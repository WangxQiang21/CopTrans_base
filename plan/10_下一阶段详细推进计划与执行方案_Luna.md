> **最新执行入口（2026-10-09）：** 请按 [R03 测试问题修复与连续转向重构执行方案](13_R03_%E6%B5%8B%E8%AF%95%E9%97%AE%E9%A2%98%E4%BF%AE%E5%A4%8D%E4%B8%8E%E8%BF%9E%E7%BB%AD%E8%BD%AC%E5%90%91%E9%87%8D%E6%9E%84%E6%89%A7%E8%A1%8C%E6%96%B9%E6%A1%88_GPT6-Luna.md) 执行。用户最新规则为同模式连续转向/连续速度、切模式才静止重配置、合理倒车正常竞争及严格终点。下方 R02 的倒车门槛和本文件旧 WP0–WP8 按历史记录理解；R03 目前是待执行方案，不代表代码已实现。

# expv3 下一阶段详细推进计划与执行方案（交给 GPT Luna）

> 2026-10-09 更新：方向合同以评价规范 v1.1 和 R01/R02 实现文档第 8 节为准。优先正向，无合格正向解才允许局部倒车回退，始终拒绝全程倒车；本文下方的旧全程正向要求按历史版本理解。下一项优先连续 A 转向与受约束时间规划。

更新时间：2026-10-08

适用工程：`/home/wq/whut_thesis/expv3`

总交接文档：`08_expv3整体实验交接.md`

**2026-10-08 最新方向：** 用户要求先明确“好轨迹”标准，再重构模式决策与路径优化核心。已建立[好轨迹评价规范与 Cost 设计](11_%E5%A5%BD%E8%BD%A8%E8%BF%B9%E8%AF%84%E4%BB%B7%E8%A7%84%E8%8C%83%E4%B8%8ECost%E8%AE%BE%E8%AE%A1.md)：全程正向为硬约束，在合理效率代价内优先净空和少调整，保留必要/综合更好的 Spin，完整评价包含有约束的时间规划。本文件 WP0–WP8 保留为此前迭代计划与执行记录；后续重构实施安排应以新规范为评价合同，不能将继续在固定 seed 周围增加局部修改当作完成核心重构。新规范的研究权重尚未标定，当前实现也尚未满足完整动态评价。

**2026-10-09 R01 进度：** 已建立独立 `mode_path_v1` 后端，模块为 `quality.py`、`retiming.py`、`mode_master.py`；16 项合同测试与 7 个正式 case 的独立扫掠/终点/正向/发布采样动态检查通过。新 cost 对照对象是同一 C++ seed 在同仿真 profile 下重新时间化，不能直接和旧 score 比。后续优先增加连续曲率段及其时间规划，再做成对权重标定、区域/航向集合和统一全程预算；详细数据与未完成边界见[核心重构 R01 实现与验证](12_%E6%A0%B8%E5%BF%83%E9%87%8D%E6%9E%84R01_%E5%AE%9E%E7%8E%B0%E4%B8%8E%E9%AA%8C%E8%AF%81.md)。新后端暂未调用 Bonmin，也未建立完整区域图。

当前用户目标：继续从“粗解优化”和“全局 MINLP 接入”两个方向迭代，在多个地图上稳定得到自然、无倒车、少模式切换、少 O 模式调角且 swept-safe 的路径；全局 MINLP 接入后应当带来可复现的端到端改善。禁止为特定地图编写状态机或地图编号分支。

---

## 1. Luna 开始工作前必须先读的文件

按下面顺序阅读，不能只读摘要：

0. `11_好轨迹评价规范与Cost设计.md`
   - 这是最新用户偏好和后续核心重构的评价合同。
1. `08_expv3整体实验交接.md`
   - 重点阅读第 27–49 节。
   - 第 48 节记录 MINLP 负目标漏洞及修复。
   - 第 49 节是修复后 N03 的最新正确结果。
2. `qna/request.md`
   - 这是用户对 P11/P12 路径行为的原始问题。
3. `catkin_ws/src/fwis_v3_demo/config/params.yaml`
   - 当前正式参数来源。
4. `catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py`
   - 当前候选生成、候选排序、MINLP 调用和后处理流水线。
5. `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py`
   - 重点函数见第 4 节。
6. `catkin_ws/src/fwis_v3_planner/src/planner_node.cpp`
   - 重点阅读 route graph 采样、模式 DP、粗路径生成和状态日志。
7. 最新实验原件：
   - `relust/N03_global_minlp_mode_bounds_replay_20261008.json`
   - `relust/P12_env2_global_minlp_cbc_matched_20261008.json`
   - `relust/N05_global_minlp_mode_bounds_replay_20261008.json`
   - `relust/N02_global_minlp_frozen_seed_cbc_probe_20261008.json`
   - `relust/N01_global_minlp_safe_coarse_fallback_20261008.json`
   - `relust/adaptive_minlp_budget_by_switch_count_20261008.json`

阅读完成后，先用一段简短文字复述：当前各阶段的数据流、最终候选如何选择、哪些结果属于 Bonmin、哪些属于固定模式 NLP 或模式邻域。确认归因无误后再改代码。

---

## 2. 不可违反的设计约束

### 2.1 行为约束

1. 正式规划路线不得含倒车或前后反复换向。
2. O 模式可以用于局部降低 A 曲率、减少 Ackermann 转向，不能采用“A/Spin 优先失败后才开放 O”的硬门控。
3. 应重点惩罚连续 O 模式中反复改变侧移角，而不是简单惩罚 O 出现。
4. 可直接用 A 平滑通过的区域，不应无原因生成 A-O-A。
5. 末端若需要频繁调航向，应优先在前方宽阔净空区域提前完成航向调整。
6. 不允许按 `N03`、`P12_env2` 等地图名或编号编写算法分支、参数分支或状态机。
7. 所有行为判断必须来自通用几何量：曲率、曲率变化、净空、终点航向误差、模式切换、转向总变差、O 角变化和安全扫掠。

### 2.2 安全约束

1. 最终候选必须通过现有车辆全外形 swept-SAT。
2. 独立 checker 使用 5 mm 级采样，不得以中心点碰撞检查替代。
3. 不得放宽最终安全验收来制造分数收益。
4. MINLP 走廊只是求解约束，最终 exact replay、终点误差和 swept-SAT 才是发布门槛。
5. 任何不满足安全门槛的候选只能记录为诊断，不能进入最终候选列表。

### 2.3 工程约束

1. `joint.enabled` 继续默认 `false`，直到第 13 节的启用标准全部满足。
2. `joint.use_cbc_seed_incumbent` 继续默认 `false`；实验配置可显式开启。
3. 不得触碰已有 ROS master 端口 `11311/11312/11320`。
4. 每次实验使用独立 ROS master，并在实验后确认端口与子进程全部关闭。
5. 不得只把实验脚本放在 `/tmp` 后就丢失。可复现的 runner、场景清单和结果解析器必须提交到工程 `tools/` 或 `tests/`。
6. 不得把 `LIMIT_EXCEEDED` 简单等同于失败；若有合法 incumbent，需要继续检查模型残差、共同分、终点和 swept-SAT。
7. Bonmin 当前求解的是非凸 MINLP，不能宣称严格全局最优。报告中使用“全局联合模型”“Bonmin MINLP 候选”或“离散模式改善”，不能写“已证明全局最优”。

---

## 3. 当前基线与正确结论

以下数据是下一阶段的比较基线，不能用更早的旧日志覆盖：

| 场景 | MINLP 输入 | Bonmin/最终结果 | 已确认收益来源 | 独立安全 |
|---|---:|---:|---|---|
| N03 | `17.387425` | `15.202620`，全 A | Bonmin 离散模式收益，约 12.565% | PASS，3,400 samples |
| N02 | `71.089430` | `69.680436`，全 O | CBC 使候选可用，同模式连续改善约 1.98% | PASS |
| N06 | `13.776025` | `13.707435`，全 A | 同模式连续改善约 0.50% | PASS |
| P12_env2 ×0.8 | `10.312117` | `10.232058`，全 A | 同模式连续改善约 0.776% | PASS，2,256 samples |
| N05 | `26.297803` | Bonmin 与 seed 持平；最终 `24.588` | 约 6.50% 来自后置固定模式邻域 | PASS，3,638 samples |
| N01 | coarse `55.844636` | Bonmin `54.149886`；最终仍为本地 `52.832` | MINLP 候选改善，但无端到端收益 | PASS |

最新目标模型修复：`core.py` 已为 `spin_abs/kappa_abs/phi_abs` 增加模式绑定上界。不得删除这些约束。修复前出现的负 `model_obj` 数据只能用于解释历史漏洞，不能再作为算法收益依据。

同轮有一次 N03 临时发布脚本把目标航向误写为 `-0.35 rad`，checker 按正式 `0 rad` 验收，产生假失败。该运行已剔除。以后 runner 必须从同一个场景清单同时生成 publisher 和 checker 目标，禁止手写两份坐标。

---

## 4. 当前架构和关键函数

### 4.1 粗规划器

文件：`catkin_ws/src/fwis_v3_planner/src/planner_node.cpp`

当前关键参数：

- `search/route_graph_dense_spacing = 0.30 m`
- `search/route_graph_sparse_spacing = 1.00 m`
- `search/route_graph_continuous_adaptive = true`
- `search/route_graph_turn_window = 0.45 m`
- `search/route_graph_turn_threshold = 0.80 rad`
- `search/route_graph_max_edge_length = 3.0 m`

当前采用连续曲率/保守净空需求插值 spacing，并保留曲率率极值作为强制锚点。WP3 已在 N03/N04/N05/N06 与 P12_env2 ×0.8 完成配对；见交接第 53 节。

### 4.2 Python 局部优化和模式候选

文件：`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py`

关键函数：

- `sequence_cost()`：所有候选最终共同评分的权威实现。
- `mode_schedule_neighbor_candidates()`：通用短窗口 A/O 模式替换。
- `upstream_sos_replan_candidates()`：在热点前方尝试 S-O-S 重规划。
- `anchor_mode_sequences()`：基于安全路径锚点生成模式序列。
- `iterated_mode_geometry_refine()`：模式/几何交替局部优化。
- `_collapse_for_joint()`：将 seed 压缩为 MINLP knots。
- `joint_refine()`：固定模式 NLP 或 Bonmin MINLP 主入口。
- `bounded_research()`：局部后缀搜索。

### 4.3 流水线调度

文件：`catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py`

关键流程位于 `on_seed()`：

1. 固定模式连续优化；
2. anchor graph / iterated joint；
3. endpoint replan 和 upstream S-O-S；
4. MINLP 前模式邻域；
5. 选择一个 MINLP seed；
6. 可选 Bonmin；
7. MINLP 后模式邻域；
8. 最终共同分排序、终点检查、timed trajectory 和 swept-SAT。

当前问题是第 3、4、6、7 步可能重复探索同一局部结构，Bonmin 自身耗时并非总时延的唯一主要来源。

---

## 5. 总体推进原则

下一阶段按以下顺序推进，不能跳过前面的可复现性和归因工作：

1. 建立持久化、单一真值源的实验框架；
2. 修复后统一重建 baseline；
3. 减少重复优化和无效 S-O-S 时延；
4. 把粗路径二档采样改为曲率/净空/航向需求驱动的连续采样；
5. 将末端调航向重规划前移到宽阔区；
6. 改进 MINLP seed 排序和走廊适配；
7. 对齐 MINLP 主目标与共同评分；
8. 最后才继续调整 Bonmin 搜索策略；
9. 在 N01–N06 和论文地图上统一配对；
10. 达到启用标准后再讨论默认打开 global MINLP。

每个阶段只改变一类机制。若一个实验同时改变锚点、代价、Bonmin 时限和后处理，就无法归因，结果作废。

---

## 6. WP0：建立持久化实验框架

### 6.1 目标

消除临时脚本坐标不一致、日志散落、端口残留和不同运行无法匹配的问题。

### 6.2 建议新增文件

1. `tools/expv3_eval_cases.yaml`
2. `tools/run_expv3_probe.py`
3. `tools/collect_expv3_probe.py`，若 runner 内部解析足够清晰可合并到 runner。

### 6.3 `expv3_eval_cases.yaml` 建议结构

每个 case 至少包含：

```yaml
N03_default:
  map_dir: maps/map12-n03-staggered-islands
  map_scale: 1.0
  start: [0.85, 1.25, 0.0]
  goal: [11.05, 6.75, 0.0]
  xy_tol: 0.16
  yaw_tol: 0.14
  tags: [builtin, mixed_mode, minlp_discrete_gain]
```

先录入：

- N01–N06 的 `scene_meta.yaml` 起终点；
- P12_env2 ×0.8 当前已验证起终点；
- P11/P12 其他案例只有在能从现有记录确认起终点时才录入；不能凭截图随意猜坐标。

publisher、checker、结果摘要必须都读取同一个 case 对象。禁止分别手写 goal。

### 6.4 runner 必须支持

- 指定 case；
- 指定隔离 ROS master 端口；
- joint on/off；
- CBC on/off；
- capture/replay frozen seed；
- 指定覆盖参数；
- 超时后终止整个进程组；
- 采集 planner、optimizer、CBC、独立 checker 日志；
- 输出一个 JSON；
- JSON 中保存实际启动配置、地图 SHA256、起终点、代码时间戳或文件哈希。

### 6.5 runner 启动前检查

1. 目标端口未监听；
2. 端口不在 `11311/11312/11320`；
3. 地图目录存在 `map.yaml`；
4. map scale 为正有限数；
5. start/goal 数值有限；
6. replay seed 的地图签名、start、goal 完全匹配；
7. CBC 开启时插件文件和 `CASADIPATH` 可见。

### 6.6 runner 结束后检查

1. roscore、rosmaster、roslaunch 和子节点全部退出；
2. 端口释放；
3. JSON 能通过 `python3 -m json.tool`；
4. 若 planner 成功，独立 swept checker 必须有明确 PASS/FAIL；
5. 不允许只有 RViz 截图而没有数值结果。

### 6.7 WP0 验收条件

- N03 同一 case 连续跑两次，不再出现 publisher/checker 目标不一致；
- runner 能自动清理端口；
- 输出 JSON 包含完整配置和安全检查；
- 不修改算法行为。

---

## 7. WP1：修复后基线重建与目标审计

### 7.1 目标

确认第 48 节的绝对值辅助变量修复没有对不同模式族产生副作用，并把修复前可能受负目标漏洞影响的结果重新定级。

### 7.2 必跑场景

按 frozen seed 重放：

1. N02：全 O、12 knots；
2. N03：单切换 seed、9–10 knots；已经正确跑过一次，再重复一次确认确定性；
3. N04：高 horizon、温和合并后约 22 knots；
4. N05：O/A 混合、8 knots；已正确跑过一次；
5. N06：全 A、8 knots；
6. P12_env2 ×0.8：全 A、7 knots；已完成 CBC 开/关配对。

### 7.3 每次必须记录的三套分数

1. `seed_common_score`：进入 MINLP 的安全 seed 共同分；
2. `model_objective`：Bonmin 内部目标；
3. `candidate_common_score`：回放 Bonmin 输出后由 `sequence_cost()` 计算的共同分。

同时记录模型目标分项：

- distance；
- spin；
- curvature；
- switch；
- oblique_use；
- oblique_cruise；
- steer_tv；
- oblique_tv；
- oblique_event；
- low_curvature_switch；
- terminal_xy/yaw；
- track_position/yaw。

### 7.4 强制诊断门槛

出现以下任一情况时，候选不得用于算法收益结论：

- `model_objective` 非有限；
- 模型目标小于 `-1e-6`；
- 目标重构差大于 `1e-5`；
- 模型约束最大残差大于当前接受门槛；
- 终点不合格；
- exact replay 不一致；
- 独立 swept-SAT 失败。

### 7.5 WP1 验收条件

- N02/N03/N04/N05/N06/P12 的模型目标全部有限且重构一致；
- 合法 O 模式没有因为新增上界被错误关闭；
- N03 离散收益和 P12 连续收益至少各复现一次；
- 输出一份新的汇总 JSON，不覆盖旧历史原件。

---

## 8. WP2：流水线调度与时延优化

### 8.1 当前问题

N03 最新正确运行中：

- Bonmin：`1.62 s`；
- joint 调用：`1.99 s`；
- Planner 总用时：`12.67 s`；
- endpoint S-O-S/后缀相关阶段约 `4.2 s`；
- MINLP 后模式邻域约 `3.7 s`（该临时实验配置显示 2 个候选，正式默认仍为 1）。

因此，首先优化阶段调度，不要继续只压 Bonmin 时限。

### 8.2 目标流水线

建议改为：

```text
粗解
  -> 固定模式连续优化
  -> 一次通用模式邻域
  -> seed 可行性/安全性排序
  -> 可选 MINLP
  -> 条件触发后置邻域
  -> 条件触发提前调航向重规划
  -> 最终共同分与 swept-SAT
```

### 8.3 去重规则

1. 前置模式邻域生成过的 schedule，用规范化 key 保存：
   - mode；
   - amount 符号；
   - 控制量量化；
   - 合并后的 knot 边界。
2. MINLP 后不得再次求解完全相同的固定 schedule。
3. 如果 Bonmin 相对 MINLP seed 的共同分改善达到阈值，并且模式复杂度没有变差，则跳过后置模式邻域。
4. 如果 Bonmin 只返回 seed、无 terminal incumbent 或改善很小，再运行一次后置模式邻域。
5. 后置邻域只从“Bonmin 输出”和“进入 Bonmin 的最佳 seed”中尚未探索的一方生成。

建议初始跳过阈值：相对共同分改善 `>= 0.5%`。这是调度阈值，不是候选接受阈值；候选接受仍保留任何安全的真实改善。必须做 `0.25%/0.5%/1.0%` 三档配对后再决定正式值。

### 8.4 endpoint S-O-S 条件触发

当前 `upstream_sos_replan_candidates()` 不应对所有路线无条件消耗完整预算。增加通用 hotspot 判定：

- 末端若干米内模式事件数量；
- 末端 Spin 总角度；
- O 控制角总变差；
- A 曲率总变差；
- 终点航向误差在路径上的集中程度；
- 前方是否存在足够净空的候选重规划窗口。

只有至少一项行为指标超过阈值，并且存在安全净空窗口时才运行 S-O-S。阈值必须由几何量配置，不得按地图名控制。

### 8.5 新增 profile 字段

至少记录：

- `pre_neighbor_schedules_generated/solved/accepted`；
- `post_neighbor_schedules_generated/solved/accepted/skipped_duplicate`；
- `post_neighbor_skip_reason`；
- `sos_hotspot_detected`；
- `sos_clear_window_count`；
- `sos_skip_reason`；
- 每个阶段 wall time。

### 8.6 配对实验

固定 seed 比较旧调度与新调度：

- N03：必须保持约 12.5% 的 MINLP 离散收益；
- N05：必须保留 `24.588–24.63` 量级的后置邻域结果；
- P12_env2：必须保留 `10.232058` 量级结果；
- N06：必须保留约 0.5% 连续收益。

### 8.7 WP2 验收条件

- 以上场景共同分退化不超过 `0.2%`；
- N03/N05 Planner 总用时相对当前对应基线至少下降 `20%`，否则说明调度修改价值不足；
- 所有路线无倒车且独立 swept-SAT 通过；
- profile 能解释省下的时间来自哪一阶段。

---

## 9. WP3：粗搜索的连续自适应锚点

### 9.1 目标

解决固定或二档步长带来的两类问题：

1. 平直区域锚点太多、模式组合爆炸；
2. 曲率/净空快速变化处锚点不足，模式边界被迫落在不合适位置。

### 9.2 修改位置

主要文件：`catkin_ws/src/fwis_v3_planner/src/planner_node.cpp`

当前 route graph 逻辑在约 840–880 行附近，根据 turn demand 在 dense/sparse 两档间选择。

### 9.3 新的连续需求量

对路线弧长位置 `s` 计算归一化需求 `u(s) ∈ [0,1]`：

```text
u_curvature = normalized absolute heading change in a lookahead window
u_curvature_rate = normalized change of local curvature
u_clearance = 1 - normalized free-space clearance
u_goal_yaw = propagated terminal heading demand near the final approach
u = max(weighted curvature terms, clearance pressure, goal-yaw demand)
```

第一版不要一次引入大量自由参数。建议：

- 先实现 `u = max(u_curvature, u_clearance)`；
- 验证后再加入 `u_goal_yaw`；
- 曲率变化率用于强制保留局部极值锚点，不一定直接进入加权和。

### 9.4 spacing 公式

将二档判断改为连续插值：

```text
spacing(s) = clamp(sparse_spacing
                   - u(s) * (sparse_spacing - dense_spacing),
                   dense_spacing,
                   sparse_spacing)
```

建议初始实验范围：

- `dense_spacing = 0.20–0.30 m`；
- `sparse_spacing = 0.80–1.00 m`；
- 仍保留原 `0.30/0.50 m` 作为 baseline。

### 9.5 必须强制保留的锚点

无论 spacing 如何，下列位置不能被稀疏采样跳过：

- 曲率符号改变；
- 曲率或曲率变化率局部极值；
- 净空局部极小；
- 路径与终点接近时的航向需求边界；
- 原始路线拐点；
- 起点和终点。

### 9.6 模式候选原则

自适应锚点用于减少候选数量和移动模式边界，不得变成硬模式门控：

- A/O/Spin 仍由统一几何可达性和共同代价竞争；
- 低曲率区域可以出现 O，但只有当它确实减少 A 曲率/转向或改善路径长度时才会胜出；
- 不能写成“低曲率只允许 A”或“只有 A 失败才允许 O”。

### 9.7 日志指标

新增：

- 总路径长度；
- anchor 总数；
- dense/intermediate/sparse anchor 数；
- 强制锚点数量及原因；
- DP 状态数；
- 模式候选边数；
- route graph wall time；
- 最终模式事件数。

### 9.8 配对场景

- N03：不能丢掉全 A 优解入口；
- N04：高 horizon 应明显减少 knots 或候选数；
- N05：减少无效 O 角变化，但不能破坏可行性；
- N06：避免过去质量退化；
- P11_case_a：检查是否消除起点附近多余转弯；
- P12_env2：检查是否减少末端揉库式调姿；

### 9.9 WP3 验收条件

- route graph 状态或候选边数量下降至少 `25%`，或粗搜索时间下降至少 `20%`；
- 最终共同分中位数不退化超过 `0.5%`；
- 不增加模式切换和 O 角更新；
- 所有已有可行场景保持可行；
- 无地图专用条件。

---

## 10. WP4：在宽阔区提前调航向的通用重规划

### 10.1 目标

将用户提出的思路落实为通用算法：如果末端或窄区需要多次 Spin、连续 O 调角或频繁模式切换，则向路径前方回溯，在净空充足的位置提前完成航向调整，再重规划后缀。

这里的 S-O-S 指 Spin–Oblique–Spin 的前向/横移组合，不允许加入前后倒车揉库。

### 10.2 hotspot 定义

沿路径建立滑动窗口，对每个窗口计算：

- 模式切换次数；
- Spin 总角度；
- A 曲率总变差；
- O 角总变差；
- O 非零角更新次数；
- 路径切向与目标航向的剩余差；
- 窗口最小净空。

热点是“行为复杂度高且下游净空趋紧”的位置。阈值必须参数化，并记录触发原因。

### 10.3 上游窗口选择

从热点向前回溯，寻找：

1. 最小净空大于车辆包络和 margin；
2. 可容纳一次 Spin 和一次 O 平移的扫掠；
3. 与热点距离不超过配置的 upstream distance；
4. 不穿越已经确认的窄门或拓扑瓶颈。

候选窗口按以下顺序排序：

1. 更大净空；
2. 更靠近热点但仍有充分操作空间；
3. 更少新增路径长度；
4. 更低的预计模式事件。

### 10.4 候选宏动作

至少比较：

- 保持原路线；
- 单次提前 Spin + 后续 A；
- S-O-S；
- A-O-A；
- 纯 A 后缀重新拟合。

这些是候选生成模板，不是地图状态机。每个模板必须通过实际几何拟合和 swept-SAT。

### 10.5 接受条件

候选只有同时满足以下条件才接受：

1. 无倒车和方向切换；
2. 终点合格；
3. exact swept-SAT 通过；
4. 共同分降低，或在极小分数 slack 内显著降低行为复杂度；
5. 若使用 score slack，必须减少至少一个模式事件，或显著降低 steering/O-angle TV；
6. 不能只因路线视觉上更平滑就绕过共同代价。

### 10.6 复杂度比较

建议使用字典式指标辅助接受：

```text
(reverse_events,
 direction_switches,
 mode_switches,
 oblique_angle_updates,
 spin_total,
 steering_tv,
 common_score)
```

`reverse_events` 和 `direction_switches` 必须为 0；其余仍以共同分为主，复杂度用于 score slack 和候选排序。

### 10.7 WP4 验收条件

- P12_env2/P11/P12 末端案例中，不出现前后倒车；
- 至少一个原末端复杂场景减少模式事件或 O 角更新；
- N03/N05 等非末端热点场景不得因该模块无条件增加时延；
- 无热点时 S-O-S 阶段应快速跳过，并在 profile 中说明原因。

---

## 11. WP5：MINLP seed 排序与走廊适配

### 11.1 当前问题

现在通常按共同分从候选中选择第一个无倒车 seed。N01 说明最低分本地候选可能在压缩或联合模型入口处发生 swept 碰撞，而较弱 coarse seed 反而能进入 Bonmin。

### 11.2 seed 预验证

在启动 Bonmin 前，对每个候选计算：

- 共同分；
- 无倒车/无方向切换；
- `_collapse_for_joint()` 后 knot 数；
- 压缩动作 exact replay 终点误差；
- 压缩动作 swept-SAT；
- MINLP 初始赋值约束违反数和最大值；
- 最小净空；
- 模式切换数；
- 预计模型变量数。

### 11.3 seed 排序

使用分层排序，不要把不同量纲直接拍脑袋加权：

1. 压缩后安全且终点合格；
2. MINLP 初值可行；
3. knot 数不超过上限；
4. 更低共同分；
5. 更高最小净空；
6. 更少 knot 和模式切换。

若最低共同分 seed 不满足前 3 项，直接跳到下一个，不要先让 `joint_refine()` 抛异常再回退。

### 11.4 单次 Bonmin 原则

已有 distinct multi-seed 实验表明，多次启动 Bonmin 只增加 `4.8–7.9 s`，没有改善最终结果。因此正式路径继续只选一个最优可用 seed。其他 seed 只用于诊断或构建更合适的走廊，不逐个启动 Bonmin。

### 11.5 走廊改进顺序

先做 seed 选择，再考虑走廊：

1. 确保走廊 reference 来自压缩后实际 `initial_actions` replay；当前已修复，不能回退。
2. 记录每个 knot 走廊宽度、覆盖类型和最小剩余自由度。
3. 如果某些模式变化被单条 seed 走廊完全锁死，尝试把邻近安全候选的局部 corridor box 合并为有限候选区域。
4. 合并走廊必须保持凸/可表达，或明确作为有限局部 disjunction；不要直接把大范围自由空间塞给非凸 MINLP。
5. 所有输出仍做 exact swept-SAT。

### 11.6 WP5 验收条件

- N01 不再先尝试已知压缩后碰撞的 best-local seed；
- seed 选择日志能解释为何选 local 或 coarse；
- 不增加 Bonmin 启动次数；
- N03/N05/P12 现有收益不退化。

---

## 12. WP6：MINLP 目标与共同评分对齐

### 12.1 目标

让 Bonmin 优化方向尽可能接近最终候选选择方向，避免 model objective 改善但 common score 不改善。

### 12.2 当前差异

MINLP 目标与 `sequence_cost()` 的主要差异包括：

- track position/yaw 正则只存在于 MINLP；
- 部分绝对值采用 epigraph 或平滑表达；
- 低曲率 A/O 切换使用 sigmoid 近似；
- 终点误差权重和最终硬容差同时存在。

### 12.3 执行步骤

1. 为每个候选同时输出 `model_objective_components` 和 `common_score cost_parts`。
2. 在 frozen seed 上计算两种目标的排序一致性。
3. 若 track 项经常改变排序，将它从主目标降为极小 tie-break，或改为 trust-region/走廊约束。
4. 不要直接将 track 权重设为 0 后凭单次结果决定；至少在 N03/N05/N06/P12 做同 seed 配对。
5. cutoff 必须与 Bonmin 实际模型目标一致；最终接受必须与 common score 一致。
6. 如果 model objective 更低但 common score 更高，候选不能替代 incumbent，并记录 `objective_rank_mismatch`。

### 12.4 O 模式代价原则

保持用户要求：

- `oblique_use` 和 `oblique_cruise` 不能高到让 O 只在 A 失败后出现；
- 连续 O 的角度变化由 `oblique_tv` 和 `oblique_event` 重点惩罚；
- 恒定角度的局部 O 可以与高曲率 A 竞争；
- A/O 低曲率切换继续收取额外代价，防止无意义 A-O-A。

### 12.5 WP6 验收条件

- model/common 排序冲突率明确下降；
- N03 全 A 离散收益保留；
- N05 不出现长距离 O 反复调角；
- P12 不出现为了 track 正则而增加路径行为复杂度。

---

## 13. WP7：全局 MINLP 搜索本身的进一步优化

只有 WP0–WP6 完成后才进入本阶段。不要继续无目标地扫 Bonmin 参数。

### 13.1 保留配置

- A/O 两个整数 bit，Spin 为补变量；
- 全程正向平移硬约束；
- CBC 可行 seed 注入；
- mode-scaled absolute epigraph 上界；
- 三类走廊几何剪枝开关 `geometry_yaw_bounds`、`geometry_spin_pruning`、`geometry_oblique_bounds` 默认均关闭；WP7 配对证实组合剪枝会影响 N03 已有离散收益，暂不作为默认行为；
- exact replay 和 swept-SAT 后验；
- `max_knots=24`；
- 温和合并 `0.1 1/m / 7.5°`；
- 单切换且不超过 10 knots 时 4 s 请求，其余基础 1.5 s；除非新配对证据推翻。

### 13.2 优先尝试的搜索改进

1. 将前置模式邻域中可行但未选中的局部模式替换，转换为 branching priority 或变量初值信息，而不是第二次 Bonmin。
2. 根据几何可达性删除不可能的 mode/knot 组合。
3. 对明显直线 knot 增加强有效界，而不是硬固定 A。
4. 对 Spin knot 使用必要角度范围，避免无意义大角度。
5. 对 O knot 使用由 corridor 和端点几何推导的 `phi` 有效范围。
6. 继续记录 root relaxation、incumbent、bound、节点数和 node NLP 失败原因。

### 13.3 不优先继续的方向

以下方案已有负面或无收益证据，除非新诊断给出明确原因，不要优先重复：

- 盲目增加 Bonmin 总时限；
- 多 seed 多次启动 Bonmin；
- 随机改变 branching seed；
- RINS；
- 多次 root resolve；
- 强制 fixed-schedule warm start 挤占总预算；
- 把 corridor slack 当成质量提升。

### 13.4 WP7 验收条件

- 至少在 N03 之外再有一个不同结构场景产生新的、更优离散模式字；
- 该收益相对进入 Bonmin 的最佳安全 seed 计算；
- 不能把固定模式连续改善或后置邻域收益记作 MINLP 离散收益；
- 求解时延没有不可接受增长。

---

## 14. WP8：论文地图行为验证

### 14.1 地图清单

`maps/paper_digitized/` 当前包括：

- P11_case_a/b/c；
- P12_env1/2/3/4；
- P16_complex_parking；
- P16_narrow_maze。

`run_demo.sh` 当前默认：

- P12_env1：scale `0.5`；
- P12_env2：scale `0.8`；
- 可用 `--map-scale` 覆盖。

### 14.2 先建立正式 case

对每张论文图明确：

- 地图目录；
- scale；
- start/goal 中心位姿；
- 位姿来源；
- 期望方向；
- 是否属于用户截图同一案例。

如果缺少确定起终点，不得凭视觉猜测后宣称算法成功或失败。可把 case 标为 `pending_pose_confirmation`。

### 14.3 用户问题对应的验证指标

#### P11_case_a

- 起步是否出现多余向上转弯；
- A 曲率 TV；
- 模式切换数；
- 与用户期望直接路径的共同分比较；
- 地图障碍边缘是否为实际 occupancy 锯齿，还是 RViz 放大显示。

#### P11_case_b / P11_case_c

- 红色目标箭头附近真实车辆包络净空；
- 地图边缘直线性；
- 失败属于几何碰撞、终点包络不可行还是搜索预算耗尽；
- 不允许仅凭 `search_budget_exhausted` 宣称几何无解。

#### P12_env1

- 使用默认 scale 0.5；
- 检查细小障碍和锯齿；
- 检查缩放后 start/goal 是否与地图中心缩放一致。

#### P12_env2

- 使用 scale 0.8；
- 比较末端是否仍出现揉库式行为；
- 检查提前调航向模块是否把复杂动作移到宽阔区；
- 已知 MINLP 连续收益约 0.776%，作为回归基线。

#### P12_env3 / P12_env4

- 核对用户截图实际对应哪张图；
- 检查 ego 左侧地图边界锯齿；
- 检查是否出现全程倒车或连续 O 调角。

### 14.4 地图锯齿诊断顺序

1. 确认实际加载的 `map_dir`；
2. 读取 `map.yaml` resolution 和 PGM 尺寸；
3. 判断锯齿来自障碍轮廓、缩放采样还是 RViz 显示；
4. 若障碍本身锯齿，修改 `tools/digitize_paper_maps.py` 的通用几何清理/高分辨率生成；
5. 不在运行时规划器中按地图修补 occupancy；
6. 重生成后检查拓扑和狭窄间隙没有被形态学处理封死。

### 14.5 WP8 验收条件

- 每张已确认位姿的地图都有 joint-off/on 配对；
- 所有成功路线独立 swept-SAT 通过；
- 所有路线 reverse distance 为 0；
- 报告模式切换、O 角更新、A steering TV、共同分和时延；
- 截图只作为辅助，结论以数值和路径数据为准。

---

## 15. 统一实验矩阵

### 15.1 每个 case 的最低配置

1. `joint=false`：当前局部算法基线；
2. `joint=true, CBC=true`：候选全局联合配置；
3. 若分析 CBC 贡献，再加 `joint=true, CBC=false`；
4. frozen-seed 对照必须使用完全相同 seed；
5. 修改单个模块时，其余参数保持一致。

### 15.2 必须输出的质量指标

- `common_score` 和所有 cost parts；
- 路径长度；
- A/O/Spin 距离或角度；
- 模式切换次数；
- reverse distance；
- direction switches；
- A curvature TV；
- O angle TV；
- O angle update count；
- 小于 5° 的 O micro update count；
- Spin 总角度；
- 终点位置/航向误差；
- 最小安全余量，若现有 checker 可可靠提供；
- independent sweep samples 和 PASS/FAIL。

### 15.3 必须输出的性能指标

- planner 总时间；
- coarse route graph 时间；
- fixed NLP 时间；
- anchor/mode graph 时间；
- endpoint/S-O-S 时间；
- pre-neighborhood 时间；
- MINLP setup/Bonmin/warm/repair 时间；
- post-neighborhood 时间；
- timed check 时间；
- knot 数、模型变量数、seed mode switches。

### 15.4 必须输出的归因字段

- final source；
- seed source；
- Bonmin returned mode word；
- seed mode word；
- 是否发生离散模式变化；
- score gain 分类：
  - coarse search；
  - fixed schedule continuous；
  - local mode neighborhood；
  - Bonmin continuous；
  - Bonmin discrete；
  - SAT repair；
  - endpoint/upstream replan。

---

## 16. 默认启用 global MINLP 的标准

在满足以下条件前，`joint.enabled` 必须保持默认 `false`：

### 16.1 硬门槛

1. 所有纳入验证的场景无倒车、无方向切换；
2. 所有成功结果独立 swept-SAT 通过；
3. 无非有限或负模型目标；
4. 无地图编号状态机；
5. 失败时总能安全回退到 joint-off 可用候选；
6. 不因为开启 MINLP 让原本可行场景丢失结果。

### 16.2 质量门槛

建议至少满足：

- N01–N06 中至少 4 个场景不劣于 joint-off `0.5%`；
- 至少 2 个不同结构场景有 `>=2%` 的端到端共同分改善；
- 至少一个场景是 Bonmin 离散模式改善，且不能只有 N03；
- 论文地图中至少 3 个已确认案例表现不退化；
- O angle updates 和模式切换的中位数不增加。

### 16.3 时延门槛

建议：

- 普通场景额外中位时延不超过 joint-off 的 `50%`；
- 单切换复杂场景可使用更长预算，但必须有显著质量收益；
- 无收益场景应通过调度快速退出，不得固定支付完整 Bonmin + post-neighborhood 成本。

阈值可根据最终论文实验口径调整，但调整必须在统一配对表中说明，不能事后为某张图单独放宽。

---

## 17. 每阶段的构建和验证命令

在工程根目录执行：

```bash
python3 -m py_compile \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py \
  catkin_ws/src/fwis_v3_optimizer/scripts/optimizer_node.py

catkin_make -C catkin_ws --pkg fwis_v3_planner fwis_v3_optimizer -j4

bash -n run_demo.sh
```

JSON 结果检查：

```bash
python3 -m json.tool relust/结果文件.json >/dev/null
```

端口清理检查示例：

```bash
ss -ltnp | rg '实验端口|11311|11312|11320' || true
```

手动运行论文地图示例：

```bash
./run_demo.sh --map P12_env1
./run_demo.sh --map P12_env2
./run_demo.sh --map P12_env2 --map-scale 0.85
```

正式自动探针应使用 WP0 runner，而不是长期依赖手工 RViz 操作。

---

## 18. 每轮修改后的报告模板

每完成一个工作包，在 `08_expv3整体实验交接.md` 追加一节，并保存独立 JSON。报告必须包含：

### 修改

- 改了哪些文件和函数；
- 新增/删除哪些参数；
- 为什么该修改是通用算法规则；
- 是否影响默认路径。

### 配对

- 场景、地图签名、scale、start/goal；
- baseline 与 candidate 的完整配置差异；
- frozen seed 是否一致；
- 至少列出共同分、模式字、时延和 swept-SAT。

### 归因

- 收益属于哪个阶段；
- 是否发生模式字变化；
- 是否只是连续状态改善；
- 是否来自 fallback 或后处理。

### 决策

- 参数是否保留；
- 默认值是否改变；
- 回退路径是否完整；
- 下一步只解决一个明确问题。

---

## 19. 推荐的实际执行顺序

### 第一轮：实验基础和修复后基线

1. 完成 WP0 runner 和 case manifest；
2. 用 runner 重放 N03 正确 case，复现 `15.202620` 附近结果；
3. 重放 N02/N04/N06；
4. 输出修复后统一 baseline；
5. 不改算法参数。

### 第二轮：调度减时

1. 增加 schedule 去重；
2. 增加 Bonmin 强改善后跳过 post-neighborhood；
3. 增加 S-O-S hotspot gate；
4. N03/N05/P12 配对；
5. 达不到 20% 时延下降则继续查 profile，不进入粗锚点改造。

### 第三轮：连续自适应锚点

1. 只加入 curvature + clearance 连续 spacing；
2. N03/N04/N05/N06 配对；
3. 通过后加入 goal-yaw propagation；
4. 再跑 P11/P12。

### 第四轮：提前调航向

1. 建立末端行为 hotspot；
2. 上游净空窗口选择；
3. S-O-S/A-O-A/纯 A 候选；
4. 先在 P12_env2 验证；
5. 再扩展 P11/P12 其他正式 case。

### 第五轮：seed 与目标对齐

1. seed 预验证和分层排序；
2. N01 验证不再浪费一次碰撞 seed；
3. model/common objective 对齐实验；
4. N03/N05/N06/P12 配对。

### 第六轮：MINLP 搜索增强和默认开关评估

1. 利用几何可达性收紧 mode/control bounds；
2. 至少找到第二个离散模式改善场景；
3. 完成 N01–N06 + 论文地图总表；
4. 按第 16 节决定是否默认开启。

---

## 20. Luna 的首个具体任务

Luna 接手后的第一个实现任务应限定为：

> 建立 `tools/expv3_eval_cases.yaml` 和持久化 ROS probe runner，使 publisher、checker、seed trace/replay 和结果 JSON 全部读取同一个 case；用它在独立 ROS master 上复现 N03 正确终点的修复后结果。不得同时修改规划算法。

首任务交付物：

1. case manifest；
2. runner；
3. N03 新 JSON；
4. 独立 swept-SAT PASS；
5. 端口清理证明；
6. 对比第 49 节结果，解释数值差异；
7. 在交接文档追加执行记录。

## 21. 执行进度（2026-10-08）

- WP0：完成。统一 case 清单和可复现 ROS probe runner 已落地；N03 修复后结果可重复，见交接第 50 节。
- WP1：完成。N02/N03/N04/N05/N06/P12_env2 完成模型目标、残差和 CBC 配对审计；N04 的 `6.76e-5` 约束残差超过 `1e-5` 收益审阅线，因此不用于 MINLP 收益结论。见交接第 51 节和 `relust/expv3_wp1_mode_bounds_replay_20261008.json`。
- WP2：完成。N03、N05、P12_env2 ×0.8 保持共同分和安全，Planner 时间分别下降约 `30.4%`、`28.9%`、`47.4%`。默认后置邻域跳过门槛为 `0.5%`；S-O-S hotspot score 门槛 `1.5`；joint MINLP 开启时跳过重复的 base schedule NLP。见交接第 52 节和 `relust/expv3_wp2_schedule_profile_20261008.json`。
- WP3：完成。采用连续曲率/净空需求，90°满曲率量程与曲率率峰值空间抑制，默认 `0.30/1.00 m`。N03/N04/N05/N06 的 DP 候选边数中位下降 `33.2%`，共同分中位变化 `0%`，无新增模式切换 / O 航向更新，安全与终点均通过；无 override 的 N03 默认配置 replay 通过。详见交接第 53 节与 `relust/expv3_wp3_adaptive_spacing_20261008.json`。
- WP4：本轮通用 hotspot 和上游后缀重规划已实现并完成首轮多场景诊断，验收未通过：P12/N03/N05 没有触发额外搜索；N04 触发纯 A、A-O-A、Spin+A 候选，但没有安全且低于 incumbent 的替代路线。保留安全拒绝逻辑，继续改进宽阔区位置、窗口跨度和对转向复杂度的预测，不加地图特例。见交接第 54 节。
- WP5：完成。Bonmin 前对每个压缩 seed 执行方向、knots、终点和 2.5 mm swept-SAT 预检；N01 有 2 个压缩后碰撞 seed 被拒，安全 seed 才进入唯一一次 Bonmin。N03/N05/P12 行为和安全门槛保留。见交接第 55 节。
- WP6：完成审计，不改目标权重。N03/N05/N06/P12 四组同 seed 比较中，Bonmin 模型目标相对 seed 的变化方向与共同分一致；track 权重从 `0.015` 设为 `0` 时，共同分最大变化 `0.000081`、模式字不变，因此保留 `0.015`。见交接第 56 节和 `relust/expv3_wp6_objective_alignment_20261008.json`。
- WP7：进行中。新增必要的 yaw、Spin、O 方向几何界并拆成独立 opt-in 参数；N03 单项剪枝保留已复现全 A 收益，但 Spin+O 组合会让 Bonmin 留在 seed，所有开关默认关闭。N04/N05 未出现新的 MINLP 离散收益。N01 把模式邻域候选数从 1 加到 3 后仍是 `45.790`、Bonmin seed 同分，后邻域多求解 3 项、Planner 到 `19.49 s`；不采用更大候选数。WP7 验收仍未满足。下一步把安全但未胜出的模式邻域路线保留为单次 MINLP 的初值提示，并以 N01 冻结 seed 配对验证。见交接第 57 节和 `relust/expv3_wp7_geometry_bounds_20261008.json`。
- WP8：尚未开始。WP7 至少在一个 N03 以外场景获得新的、共同分更低且独立扫掠安全的离散模式字后，再进行论文地图批量行为表。
