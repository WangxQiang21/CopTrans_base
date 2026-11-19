# TASK02 执行方案：三车几何感知的走廊、Portal 与 Guide

**用途：**可直接交给 Claude Code 执行的单任务实施合同。A–F 是 Claude 在同一个 TASK02 内依次完成和自审的内部步骤，**步骤之间不等待 Codex 验收或授权**。TASK02 全部交付后停止，由 Codex 对整体代码、测试、证据和真实入口做一次独立验收。用户仍单独进行最终 RViz 目视验收。

**本文件只规划实施，不授权本轮修改生产代码。** 本轮没有修改算法、测试、ROS 配置或实验记录。

## 1. 已核对的开发基线

- 仓库目录：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- 分支：`feature/virtual-rigid-formation-a`
- 本次核对的 HEAD：`bee4e3aa0f86e24085a1840ee0581b2fca34add8`
- 本地分支比 `origin/feature/virtual-rigid-formation-a` 超前 1 个提交；该提交是 RViz 路径分类与 active route alias 改动。
- TASK01 的代码审核报告已记为返工复核 PASS，但报告里的 HEAD `222b919...` 已过期；TASK02 应以实际 checkout 状态为准。
- TASK01 返工审核记录了 156 项定向数值/几何/审核测试通过，并独立复核了原 v2 重放与新 v3 负曲率误差证据；这属于已有证据，不替代 Claude 在当前工作区开始 TASK02 前的基线复跑。
- **当前工作区并非干净。** TASK01 已接受返工仍有未提交修改：
  - `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py`
  - `tests/test_virtual_a_geometry.py`
  此外还有多份既有未跟踪的 TASK01/RViz 文档、证据目录和历史证据链接。它们均不属于 TASK02 的修改。

**Claude 的开始门槛：**先保存 `git status --short --branch`、`git diff --` 上述两个 TASK01 文件、各文件 SHA-256 和实际 HEAD。不得 reset、checkout、clean、覆盖、移动或删除这些既有内容。后续 TASK02 变更应优先落在与这两个 TASK01 文件不同的文件中。若实现确实必须改其中之一，停止并报告原因，先由用户/Codex 明确处理基线；不得把 TASK01 未提交内容悄悄并入 TASK02 diff。不得在 TASK02 阶段提交或推送 Git，除非用户另行授权。

TASK02 技术方向以 `work_flow/TASK02_技术方案.md` 为准；TASK01 API 与证据以当前源码、`work_flow/TASK01_Claude开发交付.md`、`work_flow/TASK01_Codex审核报告.md` 及 `work_flow/TASK01_evidence/` 为准。若历史报告中的 SHA、状态与当前仓库冲突，以当前源码与 `git status` 为准并记录差异。

TASK01 对照证据重点位置：`work_flow/TASK01_evidence/07_ros_rviz/`（不同 backend、map05、S1、single 现场记录）、`08_manual_review/README_corridor_containment_analysis.md`（S1 的 F-point 与 OBB 诊断边界）、`09_rework_01/`（v3 负曲率误差界修复证据）。复用前先核对文件/hash；历史临时 map/runtime 路径不存在时不得声称成功复现。

## 2. 源码现状、真实调用链与插入点

下列路径均相对仓库根目录。TASK02 应顺着现有调用链增量实现，不能新增旁路 planner。

| 源码位置 | 当前职责与可复用接口 | TASK02 接入点/限制 |
|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py::make_route_guides()`、`_astar_route()` | 二维 8 邻域 A*，最多 3 条 `GuideRoute`，不同路线保留原始索引 | 保留路线生成和路线对象。A* 完成后、当前 `route_index=0` 和 corridor 构建前，对全部已生成路线做有界粗评估，生成**尝试顺序**；不得重排 `routes` 本身或改写 A* 路径 ID |
| 同文件 `_plan_virtual_a_impl()` | 唯一主规划流程；当前依序 `simplify_route → build_corridor_chain → audit_corridor_chain → make_corridor_guide → allocate_corridors → solve_open_path_guided`。route/round、预算、恢复、审查都在此调度 | 在 A* 结果后插入路线粗排序；在现有 `make_corridor_guide()` 后插入有限 Guide/Portal 评估；被选择的 Guide、它自己的 allocation、同一 chain 必须实际传入下面的 solver 调用。`untried_route_index()` 读取新的排序索引，但返回的仍是原 A* route index |
| `pipeline.py::_heading_branch_candidates()`、`_progress_result_view()`、`_ProgressEmitter` | TASK01 winding 候选；已有进度身份和阶段回调；display view 直接读 `search/routes` | 复用 winding helper 并对最终选中的 Guide 重新生成 `HeadingBranch`；扩展现有 `search['corridor']` 和 progress metrics/display，不另建 ROS 消息总线 |
| `formation_continuous_a/corridor.py::_clip_polygon()`、`_canonical_vertices()`、`_portal_bundle()`、`_guide_region_violation()` | 已有凸多边形半空间裁剪、Portal 交集及 Guide 约束工具 | 复用裁剪和 canonicalization，增加 OBB 单区/相邻并集完整覆盖判定。并集保持为 `C_j ∪ C_{j+1}`，不得把它替换成凸包 |
| `corridor.py::build_corridor_chain()`、`audit_corridor_chain()` | 从简化 A* seed 建链，并独立审核自由空间区域；chain digest 绑定原地图 hash | 不更改 `_build_region`、区域障碍分离、安全条件、digest 语义。TASK02 不为填补覆盖缺口而放宽区域构造或 audit |
| `corridor.py::make_corridor_guide()`、`allocate_corridors()` | 当前 QP Portal Guide 与 F 点单区分配 | 原 QP Guide 必须作为第一候选和回退；所有新 Guide 都独立调用 `allocate_corridors()`。`allocation` 仍只代表 NLP 的 F 点单区约束，不等同于车体过渡窗口 |
| `formation_continuous_a/contracts.py::VirtualARequest`、`GuideRoute`、`ConvexCorridor`、`CorridorChain` | request 持有真实 `model`、`vehicle_shapes`、起终点和规划上下文；`GuideRoute.quality` 可选、`compare=False`；凸区 halfspaces `A p≤b`，单位外法向；chain digest 绑定区域、route source 和 map SHA | 从 request/model 读取车辆 ID、偏移、尺寸；不得再硬编码车辆配置。新增数据只放诊断合同，不改 `VirtualARequest`、`VirtualATimedPlan`、模型 hash 或历史序列化 |
| `formation_continuous_a/geometry.py::map_spatial_state()` | 按已审核的模型映射三车中心、航向、曲率，并检查正常分支 `A_i>0,D_i>0` | 只读复用。当前 geometry 文件有 TASK01 未提交的已接受修改；TASK02 不修改它。`η` 未知时不得以 `η=0` 得出车辆曲率限制通过 |
| `formation_continuous_a/optimizer.py::_node_route_curvature()`、`solve_open_path_guided()`、`_build_corridor_constraint()` | 三点圆弧曲率 hint；solver 消费 `route_guides`、`corridor_chain`、`corridor_allocation`、`heading_branch`、`guide_length_m`；硬约束仍为 F-only | 首选不改 optimizer。候选只经现有接口进入 guide/初值，保留 winding、S 域、CasADi/NLP 约束和审核。禁止 TASK02 加三车 OBB NLP 硬约束或连续证书 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_collision.py::FormationCollisionChecker.static_check()` | 一次静态检查使用 `model.vehicle_poses()`，依次查车辆地图 OBB，再查车间 OBB SAT；真实碰撞原因含 `MAP_OBB_COLLISION`、`VEHICLE_OBB_COLLISION` | 每个规划请求创建/复用一个 checker，形状直接来自 request，margin 用当前 `collision_config.baseline_margin_m`。这只是预评估静态样本；不得替代 `collision.py::audit_collision()` 的完整时空终审 |
| `formation_continuous_a/ros_adapter.py`、`scripts/formation_uniform_planner_node.py::_progress_document()`、`_display_from_corridor_round()` | settings/request 透传；TASK01 progress 通过 request/grid/context/generation 身份检查；复用 search corridor round 构造显示 | 仅当现有 display view 不传递新诊断时最小扩展。过时或身份不符事件必须拒绝；显示失败不得改变规划、安全审核或发布状态 |
| `formation_playback.py::build_guide_display()`、`build_candidate_display()`；`fwis_v3_playback/scripts/formation_playback.py`；`fwis_v3_demo/rviz/formation_demo.rviz` | 已有 Path/MarkerArray、候选、清图和认证状态边界；`.rviz` 已有 A* 橙、Portal Guide 绿、候选黄、最终 certified 亮蓝图层 | 扩展已有诊断层或 MarkerArray，不新建 RViz 节点/消息协议。未认证 OBB/Portal 不得进入 certified Path。`nav_msgs/Path` 是实线，不复制一套线条冒充虚线 |
| 根目录 `run_demo.sh`、`catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch` | 默认编队 backend 已是 `virtual_rigid_a`，脚本注入 `corridor/hard/auto_goal=false`；有桌面时默认启 RViz，无桌面自动 `rviz=false` | 默认入口必须实际经过本次新评估。只有经测试证明现有 launch 丢弃必要字段时才允许最小改动；保留 `uniform_v02`、`legacy`、`--single` |

**实际 solver 调用**（`pipeline.py`）：

```python
solve_open_path_guided(
    request, geometry_config,
    route_guides=(selected_guide,),
    obstacle_context=obstacle_context,
    geometry_audit_config=audit_config,
    corridor_chain=chain,
    corridor_allocation=selected_allocation,
    heading_branch=selected_heading_branch,
    guide_length_m=selected_guide.length_m,
    excluded_initial_keys=tuple(used_initial_keys),
    max_new_initials=remaining_initial_slots(),
    stop_after_route_audit=True)
```

以上是现有入口的形态；Claude 应按当前源码的真实签名传参，不得照抄后假设 keyword 已存在。`endpoint_only` recovery 仍只把 route guide 置空，**chain/allocation 的硬约束继续保留**。

新增 route/Guide 评估只插入现有 `corridor_mode` 分支。精确 hold（起终点相同）仍直接走既有 hold/static audit，不触发 A* 或 route assessment；`legacy_route`、显式 `uniform_v02`、`legacy` 和 `--single` 流程不变。不要为让 TASK02 生效而偷偷把配置切成 corridor。

## 3. 已冻结的数学合同和任务边界

### 3.1 编队映射

以集群参考点状态 `q_F=(x_F,y_F,ψ_F)`、集群曲率 `κ_F` 及固定偏移 `r_i=(a_i,b_i)`：

\[
p_i=p_F+R(ψ_F)r_i,\quad A_i=1-b_iκ_F,\quad D_i=A_i^2+(a_iκ_F)^2,
\]
\[
θ_i=ψ_F+\operatorname{atan2}(a_iκ_F,1-b_iκ_F),\quad
κ_i=\frac{κ_F+a_iη_F/D_i}{\sqrt{D_i}}.
\]

中心和航向按 `map_spatial_state()` 的实现作为唯一权威。必须满足正常分支 `A_i>0`、`D_i>0`。Guide 只有折线三点曲率近似，故 `κ_F` 只是提示、`η_F` 默认未估计；车辆曲率极限字段必须标 `CURVATURE_HINT_UNCONFIRMED`，不得用占位 `η_F=0` 声称约束通过。低可信几何提示不能成为硬拒绝整条拓扑路线的理由。

车辆真实矩形 `B_i` 从 `request.vehicle_shapes[vehicle_id]` 获取尺寸，从 `request.model` 获取偏移和车辆 ID。对于半空间单位外法向 `n`，完整旋转矩形的支持半宽为：

\[
h_i(n,θ_i)=\frac{L_i}{2}|n^Te_i|+\frac{W_i}{2}|n^Te_i^\perp|,
\quad e_i=(\cos θ_i,\sin θ_i),\ e_i^\perp=(-\sin θ_i,\cos θ_i).
\]

单区完整包含检查为 `n·p_i + h_i ≤ b` 对该区所有半空间成立。单区不包含时，必须按整个 OBB 多边形判断 `B_i⊆C_j∪C_{j+1}`；不准只看中心、四角归属或 OBB 凸包。

### 3.2 任务 2 的确定输出含义

TASK02 是**有限候选的静态诊断和排序**，不是可行性证明：

- `map_sample_status`: `CLEAR_AT_SAMPLES | COLLISION_OBSERVED | UNCONFIRMED`
- `coverage_sample_status`: `SINGLE_REGION | COVERED_BY_ADJACENT_UNION | GAP | UNCONFIRMED`
- `kinematic_hint_status`: `NORMAL_BRANCH_OK | NORMAL_BRANCH_INVALID | CURVATURE_HINT_UNCONFIRMED`
- `guide_hint_status`: `VALID_HINT | LOW_CONFIDENCE | DEGENERATE`
- `candidate_certification`: 固定 `DIAGNOSTIC_ONLY`
- `continuously_certified`: 固定 `false`
- `formation_obb_hard_constraints`: 固定 `false`

`MAP_OBB_COLLISION` 是地图上某一静态样本撞障碍；`VEHICLE_OBB_COLLISION` 是两车某一静态样本互撞。`COVERAGE_GAP` 只说明 OBB 未被当前人工构造的局部并集覆盖。地图样本清楚但 coverage 有缺口时写 `CORRIDOR_COVERAGE_INSUFFICIENT`，不能写物理无路。任何 `UNCONFIRMED` 不得暗示安全。

Portal 目标语义固定为 `U_j=C_j∪C_{j+1}`。各车分别判断，允许不同车辆落在不同 region，也允许**单辆车完整 OBB 跨人工边界**但由并集完整覆盖。新 `transition_windows` 与 optimizer 使用的旧 F-only `allocation` 是不同字段、不同含义；不能把并集伪装成凸区，也不能要求三车同时完整进入同一个 region。

**不做：**二维 A* 重写、SE(2)/4D 搜索、TASK03 OBB NLP 约束、连续 OBB/Portal 证书、NLP 净空软代价改造、控制器/CAN/Gazebo/实车、TASK04 性能系统优化、对旧阈值/审核器/`/formation/playback_plan` 身份闸门的放宽。原连续 F 走廊审核、几何/动力学/原地图三车 OBB 终审和认证发布规则必须原样保留。

## 4. 推荐数据接口和几何实现

### 4.1 完整 OBB 与相邻并集几何

在 `formation_continuous_a/corridor.py` 增加少量纯函数，名称可按风格调整，但行为和返回语义必须一致：

```python
def obb_polygon(center_xy, theta_rad, shape):
    """Return four CCW world-frame corners from request VehicleShape."""

def obb_coverage_in_corridor(obb_vertices, corridor, tol_m):
    """Return {status, region_index?, max_violation_m, witness?}."""

def obb_coverage_in_adjacent_union(obb_vertices, left, right, tol_m):
    """Return status in COVERED_BY_ADJACENT_UNION/GAP/UNCONFIRMED_NUMERICS."""
```

`obb_polygon` 负责世界系 OBB 角点和 CCW/闭合规范；静态 map collision 仍通过 `FormationCollisionChecker`，不要再写第二套 grid 栅格碰撞器。`obb_coverage_in_corridor` 使用支持函数或等价的四角最大投影，对完整矩形检查所有 halfspaces；记录最差余量及 region index。

**并集覆盖伪代码（核心验收点）：**

```text
coverage(P, C_left, C_right):
    validate P, polygons, finite values, CCW, halfspace normals
    if robustly all vertices of P satisfy C_left: return SINGLE_LEFT
    if robustly all vertices of P satisfy C_right: return SINGLE_RIGHT

    classify_one_order(P, A=C_left, B=C_right):
        remaining = canonical(P)
        pieces = []
        for (n, b) in A.halfspaces:       # A is {x | n·x <= b}
            outside = clip(remaining, -n, -b)  # n·x >= b
            inside  = clip(remaining,  n,  b)
            inspect outside:
                if it is robustly positive-area:
                    if every vertex robustly satisfies every B halfspace:
                        pieces.append(COVERED_BY_B)
                    elif a positive-area outside piece has a vertex
                         violating B by > tol:
                        return GAP with halfspace/vertex witness
                    else:
                        remember NUMERICALLY_AMBIGUOUS
            remaining = inside
            if remaining is empty: break
        if no substantial outside piece: return SINGLE_A
        if every outside piece was robustly inside B: return COVERED
        return UNCONFIRMED if only near-boundary/degenerate evidence remains

    forward = classify_one_order(P, C_left, C_right)
    reverse = classify_one_order(P, C_right, C_left)
    if both orders prove covered: return COVERED_BY_ADJACENT_UNION
    if both orders prove a robust gap: return GAP
    otherwise return UNCONFIRMED_NUMERICS
```

对凸多边形 `Q_k`，只要每个顶点满足另一凸区的全部线性半空间，就证明整个 `Q_k` 被覆盖（凸性）。如果某个外侧分片具有实质面积且顶点对另一凸区有严格正的违反量，则邻近其外侧内部仍存在未覆盖点，可作为 GAP 见证。落在容差带、顺序对调结果不一致、异常小裁片、零长度/近共线边、NaN/Inf、非法朝向或多边形时返回 `UNCONFIRMED_NUMERICS`，不得以容差“扩张”自由区来 PASS。

建议距离容差：`tol_m=max(1e-9, 64*sys.float_info.epsilon*scale)`，`scale=max(1, |obb/corridor coordinates|, |b|)`，兼容当前 ROS Noetic Python 运行时。面积退化阈值按量纲取 `area_tol=tol_m*max(1, perimeter(P))`；小裁片若是否实质外侧无法稳定判定，整体结果为 `UNCONFIRMED_NUMERICS`，不得静默丢弃。保留 witness 的最小信息：区索引、半空间索引、vertex 或最差违反量；不把所有裁剪顶点无限塞入 progress JSON。

必须顺序交换 left/right 再判断，只有两种顺序都 robustly covered 才返回并集覆盖；若结果冲突，返回 UNKNOWN。不得计算 `convex_hull(C_left∪C_right)` 后测试，因为它会填掉非凸并集中的障碍缺口。

### 4.2 Guide 有限候选和 Portal 窗口

一个 route 的候选上限为 **5 个完整 Guide**：

1. 原始 `make_corridor_guide()` 的 QP 结果（必保留、第一回退）。
2. 由已有 `_portal_bundle()` 中 Chebyshev 中心组成的已验证折线（若与 QP 不重复）。
3. 至多 3 个单 Portal 局部变体：选一个需要改善的 Portal（优先基线 assessment 中最早风险/覆盖缺口对应的 Portal；没有风险时取最小 Chebyshev 半径 Portal，索引稳定打破平局），调用已有 `_portal_measure(left, right)` 获取 `C_j∩C_{j+1}` 的 CCW 顶点，在其内生成至多 4 个确定性内部点（以中心到极值顶点的凸组合 `z'=0.5 z+0.5 v`，按固定方向顺序去重），先用局部过渡站评估挑出最多 3 个，再做完整 Guide 评估。每个变体只改一个 Portal，不做多 Portal 笛卡尔积。

每个变体必须保持起点/终点等于请求、点有限、Portal 点在 `C_j∩C_{j+1}`、每段 F 线段满足原来对应凸区归属（凸性 + `_guide_region_violation()`），并调用 `allocate_corridors(chain, guide, intervals)` 重新得到其 F-only allocation。若任一条件失败，丢弃该变体并保留失败理由；不能改成 `legacy_route`。不改 `CorridorChain.digest`，不把 guide quality 写入 timed model hash。`GuideRoute.quality` 可用于记录 `candidate_id`、来源和有限评分，但不能使 Guide 变成可执行轨迹。

**Portal transition window：**对每个相邻 `(j,j+1)`，用现有 QP/所选 Guide 的累计折线长度确定 `s_j` 和 `σ_j=s_j/L_g`；`L_g≤1e-9 m` 则 UNKNOWN。构造仅供候选采样的 reach hint：

\[
ρ=\max_i\left(\|r_i\|_2+\tfrac12\sqrt{L_i^2+W_i^2}\right),\quad
W_j=[\max(0,(s_j-ρ)/L_g),\min(1,(s_j+ρ)/L_g)].
\]

该窗口不是 Portal 内切圆，也不是连续覆盖证明。采样必须包含窗口两端、`σ_j`、窗口内所有 Guide 顶点和固定间距不大于 `0.10 m` 的站点；每个窗口最多 2 层局部二分，总 Guide 最多 64 个不同站点。若站点上限无法覆盖端点/Portal/局部变化，标 `PORTAL_TRANSITION_UNCONFIRMED` + `SAMPLE_CAP`，不能下采样后标 PASS。为避免过量开销，单 route 最多 5 个完整候选，A* 最多 3 route；诊断达到当前 60 s request soft budget 时停止后续诊断，保留当前已评估结果并将未评估项设 UNKNOWN，不延长预算、不削减 solver 或安全审核预算。

每个窗口最少保留：`source_region_index=j`、`target_region_index=j+1`、`portal_seed_point`、`sigma_hint`、`sigma_window`、`station_count`、每车 `single_region_ids`、每车 `union_cover_status`、每车最小覆盖余量/必要 witness、地图与车间碰撞样本、`continuously_certified=false`。独立保存的 `legacy_f_allocation` 仍是 optimizer 唯一 F 点区间分配。

### 4.3 几何采样与地图预评估

复用 `optimizer.py::_node_route_curvature()` 得到折线三点曲率 hint，复用 pipeline 的 `_heading_branch_candidates()` 确定该 Guide 的 winding 选择。沿弧长构造有限 route stations，必须包含起终点、Guide 顶点/转角、Portal 窗口站及均匀站点。方向使用已有 guide segment tangent；端点使用真实请求航向。角点、短边、曲率突变、首末 tangent 与请求 heading 明显不一致时置 `guide_hint_status=LOW_CONFIDENCE`，不可将折线尖角误称为连续曲率。

每个样本通过 `map_spatial_state(FormationPose(x,y,psi), CurvatureJet(kappa_hint,0,0,0), request.model)` 取得真实 offset 下的 `vehicle centers/body headings`。这里 `eta=0` 只为调用需要的曲率导数参数，并且必须设置 `vehicle_curvature_status=CURVATURE_HINT_UNCONFIRMED`；`κ_i` 结果不能进入车辆曲率 PASS 判定。若正常分支校验失败，记录 `NORMAL_BRANCH_INVALID` 的 hint 和具体车/s，不伪造该样本 OBB 朝向，不让未估曲率的路线因此被认定真实无解。

以 `FormationCollisionChecker(grid, request.model, request.vehicle_shapes, collision_config.baseline_margin_m)` 检查静态地图 OBB 与车间 SAT。只针对固定样本；根据其真实 `CollisionReport.reason/vehicle_id/other_vehicle_id` 分类 `MAP_OBB_COLLISION`、`VEHICLE_OBB_COLLISION`、边界/输入错误。不要用 corridor coverage 代替 map 检查，也不要对 Guide 调用完整 `audit_collision()`。

**三车样本评估伪代码：**

```text
assess_guide(route_id, guide, chain, checker, request):
    stations = bounded_stations(guide, endpoints, corners, portal_windows,
                                uniform_ds<=0.10m, cap=64)
    if a required endpoint/portal/corner is dropped by cap:
        return missing stations as UNKNOWN with SAMPLE_CAP
    for station s in stations:
        interpolate x,y on the polyline by cumulative Euclidean arc length
        psi = request start/goal heading at endpoints; otherwise local tangent
        kappa_hint = _node_route_curvature at nearest valid guide vertex;
                     use 0 on a straight edge; degenerate => UNKNOWN
        mark corner/tangent discontinuity as LOW_CONFIDENCE
        try mapped_state = map_spatial_state(FormationPose(x,y,psi),
                              CurvatureJet(kappa_hint,0,0,0), request.model)
        except NORMAL_BRANCH_VIOLATION:
            store vehicle/s witness; OBB pose at this station is UNKNOWN
            continue
        headings = {v.vehicle_id: v.theta for v in mapped_state.vehicles}
        report = checker.static_check(FormationPose(x,y,psi), headings)
        for each model vehicle_id:
            corners = obb_polygon(mapped_center, mapped_theta,
                                  request.vehicle_shapes[vehicle_id])
            test full OBB in individual region(s)
            at transition stations test full OBB in adjacent-region union
        store map collision and region coverage as independent results
    aggregate finite statuses only; never return a path certificate
```

Guide station `s` 属于线段 `[p_k,p_{k+1}]` 时，`t=(s-s_k)/||p_{k+1}-p_k||`，位置是 `p_k+t(p_{k+1}-p_k)`。内部非顶点 heading hint 用该段 tangent；顶点在 incoming/outgoing tangent 非退化且不近反向时用单位方向和作为 bisector，否则为 `LOW_CONFIDENCE/UNKNOWN`。短段 `≤1e-8 m` 或切向和范数 `≤1e-6` 视为退化；首末 tangent 与请求 heading 差 `>20°` 只标低可信（诊断阈值，不是车辆安全限值）。端点 pose 使用 request 的真实起终点 heading。`_node_route_curvature()` 对 Guide 顶点给有向三点曲率 hint（其分母 `≤1e-12` 按退化处理）；线段内部的 `κ=0` 只代表 PL edge hint，不能声称顶点曲率连续。全局 endpoint winding 必须由 `_heading_branch_candidates()` 提供，不新建 branch 枚举规则。

**Guide 候选和 Portal window 伪代码：**

```text
build_guides(chain, base_qp):
    candidates = [base_qp]
    cheb = [start] + each _portal_bundle(...).center + [goal]
    add cheb only if distinct and it passes existing F-region guide check
    baseline = assess_guide(base_qp)
    j = earliest risk portal in baseline; else minimum-radius portal
    (radius, area, vertices) = _portal_measure(C_j, C_j+1)
    points = up to 4 stable unique midpoint(center_j, extreme vertices)
    locally score those points; keep up to 3 deterministic alternatives
    for point in alternatives:
        variant = base_qp with only portal j replaced
        require portal membership and existing Guide region validity
        allocation = allocate_corridors(chain, variant, intervals)
        require validate_corridor_allocation(chain, allocation)
        assess the complete variant; add only if distinct
    choose stable lexicographic best; absent/invalid proposals keep base_qp
    return selected guide, its own allocation, bounded assessment list

for each adjacent pair (C_j,C_j+1):
    s_j = cumulative Guide distance at that portal; sigma_j=s_j/L_guide
    rho = max_i(norm(r_i)+body_half_diagonal_i)
    window = clamp([s_j-rho, s_j+rho], [0,L_guide])
    stations = endpoints + sigma_j + Guide vertices + ds<=0.10m stations
    cap each Guide at 64 stations; preserve required Portal/end stations
    assess each vehicle's map state, single-region and full-union coverage
    add at most two bisections around sampled status changes
    record evidence with continuously_certified=false
```

这里 `rho` 只设定有限候选采样窗口，不是扫描连续过渡的证明。progress JSON 不存所有逐点多边形：每个 window 存 station 总数、各类状态计数以及每车最多 3 个 witness（首个 GAP、最差 slack、首个 UNKNOWN）；每个 route 的风险列表最多 8 条。RViz 复用现有 `CANDIDATE_OBB_LIMIT=5`（每车最多 5 个 OBB）并且最多新增 32 个 Portal/风险 marker；超限时按风险优先再按 station/portal 索引稳定截断，同时显示 omitted count，不改变规划结果。

### 4.4 推荐的诊断合同

不增加通用 validator/manager/factory。优先用既有 `search['corridor']` 和 round 字典。字段示例：

```python
search["corridor"] = {
    "guidance_mode": "corridor",
    "enforcement": "hard",
    "route_attempt_order": [1, 0],       # original route IDs
    "route_assessments": [...],          # <=3 coarse assessments
    "rounds": [...],                     # existing authoritative route/solver rounds
}
round["formation_assessment"] = {
    "schema": "virtual_a_formation_assessment_v1",
    "candidate_id": "<stable sha256>",
    "route_index": 1,                    # original A* index, never rank
    "round_index": 0,
    "request_sha256": "...",
    "grid_sha256": "...",
    "planning_context_sha256": "...",
    "chain_digest": "...",
    "selected_guide_source": "...",
    "selected_guide_sha256": "...",
    "legacy_f_allocation": [...],
    "transition_windows": [...],
    "map_sample_status": "CLEAR_AT_SAMPLES",
    "coverage_sample_status": "COVERED_BY_ADJACENT_UNION",
    "kinematic_hint_status": "CURVATURE_HINT_UNCONFIRMED",
    "guide_hint_status": "LOW_CONFIDENCE",
    "candidate_certification": "DIAGNOSTIC_ONLY",
    "formation_obb_hard_constraints": False,
    "continuously_certified": False,
}
```

Use deterministic candidate ID：SHA-256 输入包括 schema、request SHA（已绑定实际 request/model）、grid SHA、context SHA、原始 route index、round index、chain digest、Guide 点、候选来源。规划期间现有 `_progress_result_view()` 的最终轨迹 `model_sha256` 是 `None`，不得伪造；candidate ID 也不进入 physical model hash、认证轨迹 hash 或 playback envelope。最终 plan 成功后仍用原 envelope verifier 产生并核验真实 model SHA。

候选排序用有限、稳定的词典序，不堆可调权重：

1. map 样本：clear 优先，其次 unknown，最后 collision-observed；**碰撞只降序，不硬删除整个 route**。
2. 正常分支：可靠 hint 的 OK 优先；unknown 次之；可靠地 invalid 最后。低可信折线曲率不能造成 hard prune。
3. Portal 覆盖：已采样单区/并集完整覆盖优先；unknown 次之；已发现 coverage gap 最后。
4. 最后才按 Guide 长度、稳定来源顺序、原始 route/variant index 排序。

缺少 map/shape、超预算或数值退化一律 UNKNOWN；诊断缺失不能拦截原 QP Guide。若所有候选都观察到 collision/gap，仍保留最优排序的原 QP 或候选走现有 solver/既有审核流程，除非当前已有明确的原始输入校验拒绝条件；日志不得称“物理不可行”。

## 5. TASK02 内部实施步骤 A–F

这些步骤属于**一个 TASK02**，不是让 Claude 开始 TASK03。Claude 对每步的实现、测试和本步验收条件自行检查，通过后直接进入下一步；不需要把步骤交给 Codex 分段审核。普通实现缺陷由 Claude 在 TASK02 范围内修复后继续。只有发现必须改变研究模型、碰撞认证、F-only 硬约束，或必须改动 TASK01 已接受文件/破坏受保护基线时，才停下并报告用户，不得自行换路线。完成 F 的整体任务后停止，提交完整交付物，由 Codex 做一次最终独立验收。

### Step A — 冻结基线与建立可复现输入

**目标与改动：**只写入 `work_flow/TASK02_evidence/<timestamp>/baseline/`。记录真实 HEAD、分支、dirty diff、TASK01 两个修改文件哈希、相关未跟踪文件清单、地图/请求/context SHA、默认启动参数、历史已知失败。基线证据不修改源代码/旧证据。

**基线至少核对：**

1. `git status --short --branch`、`git rev-parse HEAD`、`git diff --stat`、对 dirty geometry/test 的完整 diff + SHA-256。
2. `TASK01_Codex审核报告.md` 与当前 HEAD 差异；标清文件 `work_flow/TASK01_evidence/` 中现存的独立证据，不宣称历史临时文件仍可用。
3. `./run_demo.sh --map map05 --scale 2.625 --dry-run` 输出的 backend、guidance、enforcement、auto_goal；核实启动脚本在当前 GUI 环境下的 RViz 行为。
4. 当前测试基线中已知的 `test_continuous_retiming` `KeyError: 'dynamic_peaks'`、legacy marker smoke 旧问题：先复跑与 TASK02 有关的定向集，记录本地真实结果，不改无关测试预期。

**Step A 自检/停止条件：**证据可重算、工作区既有修改仍存在且哈希匹配。若无法区分 TASK01 dirty diff 与 TASK02 变更、或基线文件已被外部改变，停止，不建 branch、不清理工作区。

### Step B — OBB 与凸区并集完整覆盖几何

**允许修改：**`formation_continuous_a/corridor.py`，新增 `tests/test_virtual_a_formation_assessment.py`（几何用例可集中在该文件）。必要时只对既有测试做直接相关的少量扩展；不改 geometry、collision checker 或 optimizer。

**实现顺序：**先做 `obb_polygon()` 与单区支持函数，再做双向半空间裁剪 union 判定，最后落下固定反例。禁止一开始接 ROS/pipeline。

**定向几何用例和预期：**

1. 正例：`C_L=[-2,0.3]×[-1,1]`、`C_R=[-0.3,2]×[-1,1]`、`P=[-0.5,0.5]×[-0.2,0.2]`，P 不属于任何单区但完整属于并集，正序/反序都 `COVERED_BY_ADJACENT_UNION`。
2. 四角陷阱：`C_L=[-2,0]×[-2,2]`、`C_R=[-2,2]×[-2,0]`、diamond P 顶点 `(0,1),(1,0),(0,-1),(-1,0)`。原始四角均在并集，右上内部缺口，必须 `GAP` 或保守 UNKNOWN，绝不 COVERED。
3. P 完全在单个左区/右区：返回 `SINGLE_REGION` 和对应 witness。
4. P 整体跨区，但两个单区都不包含：返回 union witness；不得把 vehicle 归属压成 NONE。
5. 尺寸/heading 不同的旋转矩形；支持函数与显式四角投影最大值差 ≤`1e-12 m`（double fixture 尺度在 10 m 内）。
6. 两区交换、顶点旋转起点、CCW 合法顶点顺序不改变确定状态；无效 clockwise/NaN/Inf/短边或近退化裁剪 fail closed。
7. 精确相切、容差内擦边、零面积裁片：只允许 `UNCONFIRMED_NUMERICS` 或由严格间隙证明的 GAP，不允许靠正容差扩大可行区后 PASS。

**Step B 验收：**解析几何四例全过；反例证明使用全多边形而不是四角；顺序交换一致；不改 `_audit_region()`、`audit_corridor_chain()`、`audit_path_in_corridors()`。保存 diff、哈希、测试命令/退出码/原始日志及 geometry results JSON。任何不确定结果都按 UNKNOWN 处理。

### Step C — 三车姿态、地图静态样本与 Portal 过渡数据

**允许修改：**`corridor.py`、`pipeline.py`、最多一个轻量数据 contract（确有必要才改 `contracts.py`）、新测试文件。`geometry.py` 和 `formation_collision.py` 均为复用模块，原则上不修改。

**实现：**

1. 建立一次 request 级 checker，直接使用真实 `request.model`、`request.vehicle_shapes`、grid 和现有 baseline margin。
2. 路线粗采样最多 32 站/route；Guide 完整评估最多 64 unique stations/候选。采样含端点、折线转角、Portal 窗口点及弧长均匀点。每次静态检查同时得到三车地图 OBB 与车间 SAT。
3. 在 `map_spatial_state()` 中复用实际模型偏移，使用 guide heading 与三点 `κ` hint；凡 `η` 未有连续估计，车辆曲率始终 unknown。
4. 按第 4 节构造 transition windows；每车分别记录 single-region 与 union coverage，三车可有不同区域状态。
5. 在 `search['corridor']['route_assessments']` 记录 A* route 的有限粗评估；round 中记录所选 Guide 的详细 `formation_assessment`。保留 route 原始索引、chain digest、request/grid/context identity。JSON 限制：route ≤3、Guide 候选 ≤5/route、transition window 数 ≤region pair 数、站点 ≤64/Guide；marker/日志只发必要摘要。

**负例测试：**

- F 点清楚而一个车辆 OBB 实际撞静态墙：`COLLISION_OBSERVED`，有 vehicle ID/站点/reason，不能成为 clear。
- 车间 SAT 重叠与地图障碍分别保留不同 reason。
- map OBB 清楚但局部 `C_j∪C_{j+1}` 不覆盖：`CORRIDOR_COVERAGE_INSUFFICIENT`，不是 `MAP_OBB_COLLISION` 或 `PHYSICALLY_INFEASIBLE`。
- 走廊覆盖成功但 map static collision：两个字段各自准确呈现，不能相互覆盖。
- `A_i≤0`、非有限数、缺 shape、缺车辆 ID、grid/request 不匹配、窗口越界、单窗口样本超限：相应 INVALID/UNKNOWN；绝不通过更改 shape/offset 获得成功。
- `eta=0` diagnostic placeholder 不得使 vehicle curvature status 变 PASS；`continuously_certified=false`、`candidate_certification=DIAGNOSTIC_ONLY` 永远成立。

**Step C 验收：**直接调用现有 `FormationCollisionChecker.static_check()`，不得新增等价独立 raster/SAT。冻结 request/map/context/chain identity 可被核对；所有 map、coverage、kinematic 三类状态独立。限额下单次评估耗时可测，未评估候选保留 UNKNOWN。

### Step D — Guide 候选排序真正接入主 solver

**允许修改：**`pipeline.py`、`corridor.py`、相关 pipeline/recovery tests；只有要把数值 JSON 安全传进 progress 才最小改 `formation_uniform_planner_node.py`。优先不修改 `optimizer.py`。

**具体流程：**

```text
A* returns routes[0:n], n<=3 (keep original tuple and IDs)
for every route: simplify once; coarse-assess map OBB and hint status
route_attempt_order = stable lexicographic sort(coarse assessment, length, ID)
route_index = route_attempt_order[0]

when current pipeline builds that route's chain and passes audit:
    base = make_corridor_guide(chain, request start, request goal)
    guides = bounded candidates(base, Chebyshev center guide,
                                <=3 one-portal variants)
    for guide in guides:
        allocation = allocate_corridors(chain, guide, intervals)
        validate assigned F segments with existing guide/chain checks
        assess OBB map + each adjacent union at bounded stations
    selected = deterministic lexicographic best
    if assessment/candidate generation is unavailable:
        selected = original base QP guide, mark UNKNOWN/fallback reason
    recompute _heading_branch_candidates(request, selected.points, source)
    pass selected guide, its own allocation, original chain to solver
```

必须修改真实调度的 `route_index=0` 初始化和 `untried_route_index()`，使后续 route recovery 按 `route_attempt_order` 选**未尝试的原 route index**；`routes` 列表、A* topic ID、active route alias 都不改成排名后的虚假索引。相同 request 多次运行的 route order、candidate ID、Guide choice 必须 deterministic。

为避免重复简化或不同候选使用不同 seed，A* 返回后保存 `simplified_by_route[original_index]`；粗评估和后续 `build_corridor_chain()` 共用这份结果。route 粗评估仅决定 attempt order，不提前声称尚未尝试的 route corridor 已构造或审核；Portal/Guide 详细评估仍按当前 route/round 按需构造。

如果新 Guide 与原 F-only allocation/chain 校验不兼容，改选原 QP Guide，记录 `FORMATION_GUIDE_NOT_SOLVER_COMPATIBLE`；始终 `guidance_mode=corridor`、`enforcement=hard`。如果全部候选 map-collision/coverage-gap/unknown，仍按风险顺序尝试原 QP 与可用路线，不把有限样本变成物理不可行，也不得削减 `MAX_ROUTES=3`、`MAX_INITIALS=6`、`MAX_RECOVERIES=1` 或 60 s soft budget。

**必须证明候选真影响算法：**新增一个确定 pipeline 单测，造 route0 的局部三车 OBB map risk、route1 的 clear case，让 route attempt order 为 `[1,0]`，但 A* `routes` 仍保持原 `[0,1]`；spy `solve_open_path_guided()` 断言该 round 的原始 `route_index==1`，实参 `route_guides` 精确等于 route1 chain 所选 Guide，`corridor_chain` 和 `corridor_allocation` 与该 Guide 一致，并验证 `heading_branch`/`guide_length_m` 来自该 Guide。solver 收到的是 corridor Guide，不是 raw A* `GuideRoute`；测试应检查来源关系而非要求传 raw route。再加“全 UNKNOWN/全观察到碰撞仍继续原 solver”用例。只断言 diagnostics 中出现了 assessment 不算通过。

另加一个 Guide 选择集成例：固定同一有效 chain，使原 QP Guide 的某一 Portal transition 样本有真实 map OBB collision 或 union gap，而同一 Portal 的一个候选位置在真实 checker 上状态改善；断言选择器选择该候选（或当 fixture 中不存在安全改进时明确保留 QP），并由 solver spy 检查传入点列、chain、allocation 与 assessment 的 `candidate_id` 完全同源。若用 stub evaluator 隔离选择排序，至少再有一个真实 `FormationCollisionChecker` 几何 fixture 证明样本状态来自真实 OBB/map 查询；不允许只 mock 出“候选更好”就通过。

如果任何 local Guide variant 的分数更优但其 F-only allocation 不能通过既有验证，必须选兼容 QP；不得改变 `_build_corridor_constraint()` 的区分配或 halfspace/margin。候选 selection 的详细 evidence 进入 `corridor_round` 与相应 NLP attempt 记录，使 solver 实际消费关系可追溯。

### Step E — 默认入口、HUD 和 RViz 诊断

**允许修改：**优先扩展现有 `pipeline.py` progress payload、`formation_uniform_planner_node.py::_progress_document()`/`_display_from_corridor_round()`、`formation_playback.py::build_guide_display()`/既有 candidate builders、`fwis_v3_playback/scripts/formation_playback.py` 及 `formation_demo.rviz`。只有默认运行链被源码/测试证明确实未透传时，才动 `run_demo.sh` 或 launch。

**显示要求：**

- A* route 仍橙 `#F97316`，实际选中的 Portal Guide 绿 `#22C55E`，未认证 candidate/OBB 黄 `#FACC15` 系或已存在灰蓝 OBB，Certified 最终轨迹亮蓝 `#38BDF8`；同类三车同色。
- map collision 样本用洋红标记且标签含 `(sample)`；corridor gap 用独立状态/图形/标签，不能伪装为地图碰撞。
- 只画当前 plan/当前 route/round 的少量代表 OBB 和 Portal 窗口。Marker 数有硬上限，不绘制全部采样密集车框；无关旧图层按既有 lifecycle 清理。
- HUD 显示 route index、round、候选来源、静态样本数、map collision 数、union covered/gap/unknown 数；字段来自真实 assessment。未进行的指标显示空/unknown，不编百分比。
- 新字段保留现有 `plan_id, generation, request_sha256, grid_sha256, planning_context_sha256` admission 检查；`request_sha256` 绑定本次实际 model/request。规划期间最终 trajectory `model_sha256` 按当前源码保持 unavailable/null，不伪造；仅最终成功 envelope 继续走原 model digest verifier。失败时可保留本 plan 诊断，但不会把失败候选变 certified；过时 event、pose reset、busy rejection 生命周期与 TASK01 保持一致。
- Certified Path 只有原成功审核后原子 `/formation/playback_plan` 发布链能产生。候选路径和 Portal diagnostics 不得改执行消息。

**默认入口校验：**当前 `run_demo.sh` 已设置 formation backend 为 `virtual_rigid_a`，缺省补 `virtual_a_guidance_mode:=corridor`、`virtual_a_corridor_enforcement:=hard`、`auto_goal:=false`；有图形桌面时默认开启 RViz，无桌面自动 `rviz:=false`。保留这一行为，不用测试时偷偷补 backend 参数来证明默认成功。

**Step E 验收：**离线 display tests 覆盖正确颜色/图层/身份拒绝/失败清图/不 certified；实际 ROS progress document 中能看到被选择 route、Guide 来源及有限诊断字段。默认脚本 dry-run 及 launch dump 证明 route 经过 `virtual_rigid_a/corridor/hard`。

### Step F — 回归、ROS/RViz 实跑与冻结交付

不增加功能，不自动进入 TASK03。Claude 按第 7–9 节自行完成离线和真实 ROS 验收；所有失败/未运行项原样报告。F 不是额外的 Codex 中间验收点。TASK02 整体交付后 Claude 停止，不提交、不推送，等待 Codex 一次性审核。

## 6. 需要新增/修改/保护的文件清单

### 6.1 预期主要改动

| 文件 | 预期工作 |
|---|---|
| `.../formation_continuous_a/corridor.py` | OBB polygon/support；单区和双向相邻 union 完整覆盖；有限 Portal guide variant 生成/校验；不改区域障碍分离与连续 F audit |
| `.../formation_continuous_a/pipeline.py` | route 粗评估排序；按实际 route/round 做 Guide/Portal 评估和选中；真实 solver 接入；transition/assessment/search/progress 摘要；保持 budgets/recovery/audits |
| `.../formation_continuous_a/contracts.py` | **可选**：只有普通 dict 无法确保 schema 不变量时增加一个轻量 immutable diagnostic contract；不改物理 request/hash/旧 plan |
| `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py` | **条件改动**：若目前 `_progress_document()` 不透传实际 selection/assessment，只增加 JSON-safe 字段，不改 admission/publish gate |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py` | 现有 guide/candidate builder 加少量 OBB、Portal/witness/status marker，严格遵循认证和 identity lifecycle |
| `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`、`catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz` | 仅在现有 marker/path dispatch 缺少表达能力时小改；复用既有 topics/namespaces/palette |
| `tests/test_virtual_a_formation_assessment.py` | 新增专门几何、sample-status、transition-window、诊断 identity 测试 |
| `tests/test_virtual_a_pipeline.py`、`tests/test_virtual_a_corridor_display.py`、`tests/test_virtual_a_receiver.py` | 只追加 route/guide 真正影响求解和 progress/display lifecycle 直接测试 |
| `work_flow/TASK02_evidence/` | 新建本任务证据，不覆盖 TASK01 或 evaluation 历史目录 |
| `work_flow/TASK02_Claude开发交付.md` | 按本任务报告实际实现、diff、测试、ROS/RViz、证据/未完成项 |

### 6.2 原则上不改

- `geometry.py`（特别是 `g1.v2/v3`、数值证书；当前还有 accepted dirty fix）
- `formation_collision.py`、`collision.py`（静态 checker 和独立完整审查均只复用）
- `optimizer.py` 的约束、目标、求解域、数值积分、initial 数量、S/winding；若没有无法通过现有接口解决的实证问题，不改文件
- `ros_adapter.py`、launch、`run_demo.sh` 的发布合同/default routing；不需新 param 就不改
- 任何历史 `TASK01_evidence/`、`evaluation/`、model fixtures/已验收测试预期
- `/formation/playback_plan` schema、播放时间轴、三车认证路径和 safety gate

所有计划范围外修改必须先停下来解释。不得用大范围格式化、类重命名、兼容 wrapper、manager/factory 新层、parallel validator system 或清理历史文件夹扩大 diff。

## 7. 测试命令与场景验收

### 7.1 Python 环境

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
```

测试只写新证据目录或系统临时目录，不覆盖任何既有 evidence。

### 7.2 必跑定向模块

```bash
python3 -m unittest tests.test_virtual_a_formation_assessment
python3 -m unittest tests.test_virtual_a_corridor tests.test_virtual_a_corridor_optimizer
python3 -m unittest tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery
python3 -m unittest tests.test_virtual_a_corridor_display tests.test_virtual_a_receiver
python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_collision
python3 -m unittest tests.test_virtual_a_optimizer tests.test_virtual_a_integration_bounds
python3 -m unittest tests.test_virtual_a_playback tests.test_virtual_a_trajectory
python3 -m unittest tests.test_formation_demo_assets tests.test_formation_hud
catkin_make -C catkin_ws -j4
bash -n run_demo.sh
git diff --check
```

TASK01 dirty geometry change is baseline; rerun `test_virtual_a_geometry` and report that it passes without altering its expected values. Do not use full `unittest discover` as the only signal: known `test_continuous_retiming` `dynamic_peaks` KeyError is a documented baseline issue. If encountered in a broader run, capture it, compare against baseline, do not “fix” or suppress it inside TASK02.

### 7.3 必须通过的场景集合

| ID | 最小 fixture / 操作 | 量化或明确预期 |
|---|---|---|
| U1 | §5 Step B 两区重叠矩形 | 两单区各自不含 P；union 双顺序 robust PASS；证明全 polygon |
| U2 | diamond 四角反例 | 输出 GAP 或数值 UNKNOWN；绝不 COVERED |
| U3 | 单区与相切/退化 | 单区 witness 正确；边界/数值不明 UNKNOWN；不放宽 halfspace |
| O1 | 旋转 OBB 支持函数对照 | `h(n,θ)` 与显式四角投影最大值误差 ≤`1e-12 m`（坐标尺度 ≤10 m） |
| M1 | 合成 grid 墙体切到 car OBB、F 参考点路线仍通 | 静态 checker 找到真实 vehicle ID，reason 为 map collision；candidate 不进入 clear rank |
| M2 | 车辆对车辆静态重叠 | `VEHICLE_OBB_COLLISION`、两个 ID 均可追踪，与墙体碰撞不同字段 |
| C1 | 改窄人工 corridors、map 保持空闲 | coverage GAP、map CLEAR 两字段并存；不称物理无路 |
| C2 | 三车在相邻区分布 + 一车跨区被 union 覆盖 | 单区 ID 可不同/为空；跨区 OBB 仍得到 union witness；不强制共用 C |
| K1 | U-turn、大 offset、非零 κ hint | 车中心/heading 与 `map_spatial_state()` 同状态对照 ≤`1e-12`；`η` 未知时曲率 status UNKNOWN |
| P1 | 两条合成 A* routes，route0 有采样 map 风险、route1 清楚 | `route_attempt_order=[1,0]`；原 routes 顺序不动；首次真实 solver 调用收到 route1 对应 Guide/chain/allocation/HeadingBranch |
| P1-G | 同一 chain 的 QP Guide 与单 Portal variant 评分不同 | 真实 OBB/map 或 union coverage 证据改变选中 Guide；solver 实参/`candidate_id`/guide/allocation 与被选 variant 一致 |
| P2 | route/guide 全 UNKNOWN 或有限样本都有碰撞 | base QP 保留；仍调用原 solver/已有审核，不把未知/样本碰撞变物理无解 |
| P3 | 改 Portal candidate 使 `allocate_corridors` 或 Guide halfspace 不合法 | 变体拒绝且明确 reason；同 corridor/hard 下用原 QP，不进入 legacy route |
| I1 | request/map/context/generation/route 错身份、非有限坐标、超 station/marker cap | fail closed/UNKNOWN；过时 progress 不改变当前 plan/图层/认证播放 |
| S1 | TASK01 历史窄走廊样例 | 保留原 hard 失败（如仍失败）；新增 map-vs-coverage 诊断；不得放宽 `CORRIDOR_AUDIT_WIDTH_LIMIT` |
| T1 | 已冻结的 `600003/600004` TASK01 数值/绕行案例 | 现存 map/request 可重建时比较旧证据；数值模型、winding、S 域、几何/动态/OBB audit 不回归；不可重建则 `NOT_RUN` |

**M1 fixture 构造：**从现有 `VirtualARequest`/vehicle-shape 测试 fixture 取得 `model` 和 shape；构造分辨率 `0.05 m`、边界至少覆盖 `[-4,4]×[-4,4] m` 的空 `Grid`（优先复用 `test_virtual_a_collision.py` 的 grid builder）。取 `q_F=(0,0,0)`、`κ_F=0`，由 `map_spatial_state()` 取得车辆中心，选一辆车中心所在 cell 标为 occupied，断言 F 原点 cell 仍 free、其他车 OBB 没有被同一格覆盖。用实际 request shapes 与 baseline margin 调 `static_check()`，期待 `MAP_OBB_COLLISION` 且 vehicle id 是该车。若当前 Grid builder 的边界约定导致 cell 归属不同，在测试里显式断言 cell center 和 footprint report，不通过改车尺寸/offset来凑结果。

**P1-G fixture 构造：**在测试中构造已通过 Guide 几何校验的两 region chain，利用 `make_corridor_guide()` 的 QP Portal 点和 `_portal_measure()` 的候选点生成 QP/variant 两条合法 Guide；用真实固定模型和实际 OBB footprint，在测试 Grid 中把一个障碍只放在 QP Guide 的对应 Portal 窗口车体 footprint，variant window 留空。先单独断言 `FormationCollisionChecker.static_check()` 的两组样本分别 collision/clear，再执行实际 candidate selector 和 pipeline solver spy。若该布置无法做到只影响 QP，不可伪造 evaluator 结果；改用直接构造状态的 union-gap vs map-clear case 验证覆盖评分，再在报告中明确没有达到 map-collision guide-improvement 用例。

### 7.4 主入口与回归兼容

```bash
./run_demo.sh --map map05 --scale 2.625 --dry-run
roslaunch --nodes fwis_v3_demo formation_demo.launch
roslaunch --dump-params fwis_v3_demo formation_demo.launch backend:=virtual_rigid_a
roslaunch --dump-params fwis_v3_demo formation_demo.launch backend:=uniform_v02
./run_demo.sh --map map05 --scale 2.625
```

期待 dry-run 默认命令包含且仅一次：`backend:=virtual_rigid_a`、`virtual_a_guidance_mode:=corridor`、`virtual_a_corridor_enforcement:=hard`、`auto_goal:=false`；GUI 存在时 RViz 自动开启；无桌面时脚本打印并设置 `rviz:=false`。`uniform_v02`/`legacy` 不应收到新 virtual 参数，`--single` 仍走原单车入口。

ROS 现场运行必须使用最后一条**不补隐藏 backend/guidance 参数**的默认命令，并完成：

1. 启动后保存实际 `roslaunch` 参数、节点列表、地图 SHA 与 runtime map 路径；在 RViz 用 `2D Pose Estimate` 和 `2D Nav Goal` 建立 request。
2. 运行 `rostopic list`，然后保存 `/formation/planning_debug`、`/formation/job_state`、`/formation/quality_report`（若 topic 名不同，以实际列表为准）的消息；验证 route index 是原 A* ID，Guide source/candidate ID、chain SHA、allocation 与 solver round 一致。
3. 分别保存 `/formation/path/astar_active`、`/formation/path/portal_guide_active`、`/formation/planning_candidates` 的消息样本；截图需同时看清 A*、实际 Guide、候选风险/unknown、HUD；不得让诊断图层带 `CERTIFIED`。
4. 成功请求时检查原 geometry、dynamics、collision 三项审核和原子 playback envelope 仍是唯一产生亮蓝 certified 路径的链；失败请求检查可执行 envelope 数为 0。若发布清空用的空 tombstone，单独记录为非 executable。
5. 在合成集成测试中证明 route1/局部 Guide variant 可作为被选候选；真实 map05 若恰好选 route0，不得据此省略 P1 spy 测试。
6. 以同一默认脚本检查一次宽松可行场景和 S1 窄通道历史场景。S1 保留旧失败结论/原因；如地图或起终点不能重现，记 `NOT_RUN` 并说明，不伪造等价场景。

仅 `--dry-run`、launch dump、ROS mock 或截取某个 Marker 都不能单独通过主入口验收。真实 GUI 不可用时记录 `NOT_RUN`，TASK02 不得报完整通过，交 Codex 判断是否可条件验收。

## 8. 默认运行接线与安全不变量

Codex 验收必须同时找到：

1. route coarse assessment 位于 `make_route_guides()` 成功之后、第一次 route solver 调用之前，并真实改变 `route_attempt_order`/首次 route 尝试。
2. Portal/Guide assessment 位于已 audit 的 chain 和 `make_corridor_guide()` 之后；它真实选择 Guide variant 或保留 QP，并把**精确选中 points**及独立重算 allocation 传入 `solve_open_path_guided()`。
3. NLP round/attempt 记录同一 `route_index`（原始 ID）、`round_index`、`chain_digest`、`candidate_id`、guide SHA/source、winding 与长度 hint；RViz active route 使用同一 ID。
4. `legacy_f_allocation` 始终给既有 F-only halfspace constraint；`transition_windows` 只给 diagnostic/future handoff。两者不可互相覆盖。
5. 原 `audit_corridor_chain`、F-point 连续 containment、几何 audit、动态 audit、`audit_collision` 及 receiver identity/publish gate 调用位置和阈值不被删改。candidate statuses 永不能修改 executable flag 或直接发布 `/formation/playback_plan`。
6. 所有新增 status 和 Marker 只展示静态采样结果，文案含 `sample`/`diagnostic`/`unconfirmed`；不得叫 `certified portal`、`continuous OBB safe`、`physically infeasible`。

## 9. Claude 交付物与证据要求

目录：`work_flow/TASK02_evidence/<UTC timestamp>/`，推荐仅用 `baseline/`, `geometry/`, `candidate/`, `ros_rviz/`, `regression/`。不得覆盖 `TASK01_evidence/` 或 `evaluation/`。

必须交：

1. `work_flow/TASK02_Claude开发交付.md`：实际改动文件/函数和调用路径；算法状态与诊断边界；候选怎么进入 solver；失败与 UNKNOWN 语义；与 TASK01 基线差异；每个测试真实命令、exit code、数量/耗时；ROS/RViz 完成/未完成项；未解决问题。
2. `source.diff`、`source_hashes.txt`、`git_state.txt`：diff 必须把原有 dirty TASK01 文件单列对照，不能把旧 dirty 误报为 TASK02；哈希覆盖源码、测试、RViz 配置、任务证据摘要。
3. 每条测试命令原始 stdout/stderr、exit code、起止时间；几何反例的实际 result JSON；P1 solver spy 的原始测试日志；真实地图请求输入 identity。
4. ROS/RViz 的原始命令、node/param/topic 摘要、`planning_debug`/job/quality/path/marker 原始消息（限必要片段）、截图、认证发布计数；测试 ROS 与 GUI 不可运行则列 `NOT_RUN` 和具体环境阻碍。
5. `manifest.sha256` 覆盖交付文档、日志、结果和截图；独立重算全部匹配。不能只给 Claude 摘要文本或哈希清单。
6. 明确最终 Git state。不得 commit/push/reset/clean；不得删除或修改与 TASK02 无关的未跟踪文件。Task02 完成后停止，不自动开始 TASK03。

## 10. Codex 独立验收清单

Codex 不直接相信 `TASK02_Claude开发交付.md` 的通过声明；按源码重新核对：

### 数学/几何

- [ ] 读取实际使用 request model/shapes，没有新增硬编码车身尺寸/offset。
- [ ] OBB 构造、支持函数、正常分支和 `map_spatial_state` 对照正确。
- [ ] 完整并集算法处理多边形，不是四角/中心测试；U1 正例通过，diamond U2 不会 false PASS。
- [ ] 单区、并集、gap、numeric unknown、地图 collision 是分离语义；并集检查正反顺序稳定。
- [ ] `eta` 未知未被伪装成车辆曲率通过；所有 diagnostics 都是 `DIAGNOSTIC_ONLY`。

### 主算法实际使用

- [ ] 仍为二维 8 邻域 A*、最多 3 route；`routes` 元组和 RViz ID 保持原索引，只有尝试顺序改变。
- [ ] route assessment 实际进入 `untried_route_index()`/首次尝试排序；不能只在 log/search 记录。
- [ ] 被选的 Portal Guide 和它自己的 `allocation` 作为实参进入真实 `solve_open_path_guided()`；单元 spy 与源码一致。
- [ ] Guide 改进变体通过既有 corridor/guide 检查；不兼容则明确回退 QP，同 `corridor/hard`；endpoint-only 继续保留 chain/allocation hard constraint。
- [ ] route/Guide variant 数、stations、soft budget、initial/recovery cap 都有明确小上界；预算用尽不触发隐藏 solver 或改变现有策略。

### 安全、回归、可视化

- [ ] 没有触碰/放宽 TASK01 数值证书、连续 F containment、geometry/dynamics/OBB 终审、身份绑定和发布闸门；TASK01 dirty fix 未丢失。
- [ ] Guide/Portal diagnostic 不能置 `CERTIFIED` 或直接发布计划；失败不能回放旧轨迹。
- [ ] display 拒绝错身份/过时 generation；marker 上限与清图生命周期正确；颜色符合当前 RViz 方案。
- [ ] 狭窄历史 S1 失败没有通过降低阈值掩盖；T1 数值与旧 backend/单车回归证据诚实标记。
- [ ] 默认 `./run_demo.sh --map map05 --scale 2.625` 实跑到新诊断与 solver，不是额外参数/离线 demo；如果 GUI 未运行，按未完成列出。
- [ ] 证据 diff/hash/manifest 可独立复算；TASK02 diff 不含原始 TASK01 dirty change 或无关 untracked 文件。

Codex 总结只用 `PASS / CONDITIONAL PASS / FAIL`，列出阻塞点、源码行号、复现命令和最小返工项。存在“主规划器没有消费候选”“OBB union false PASS”“放松任何独立认证”“默认入口未集成”任一项时必须 FAIL，不能用测试总数或截图替代。

## 11. 直接交 Claude Code 的任务说明

> 在当前 `feature/virtual-rigid-formation-a` worktree 中，按 `work_flow/TASK02_技术方案.md` 和 `work_flow/TASK02_Codex执行方案.md` 完成一个 TASK02。按 Step A–F 顺序执行；每个内部步骤由你自行测试、自审并在满足条件后继续，**不要逐步等待 Codex 验收**。当前 HEAD 是 `bee4e3aa0f86e24085a1840ee0581b2fca34add8`，但工作区含已经 Codex 接受的 TASK01 dirty 修改 `geometry.py` 与 `tests/test_virtual_a_geometry.py` 及其他用户文档/证据；先记录并保护这些内容，禁止 reset/clean/覆盖。只做最小必要改动，复用现有 A*、corridor、`map_spatial_state`、`FormationCollisionChecker.static_check`、F-only NLP、独立审核和 progress/RViz 层。严格执行相邻凸区并集完整 OBB 语义、UNKNOWN/采样-only 语义、硬预算和原始 route ID。必须证明 route/Guide 选择真正进入默认 `run_demo.sh → virtual_rigid_a/corridor/hard → plan_virtual_a → solve_open_path_guided`。不要实施 TASK03，不改模型/安全阈值/旧测试期望，不 commit/push。完成所有离线与实际 ROS/RViz 项后，整理第 9 节交付证据；整个 TASK02 交付后停止等待 Codex 一次性独立验收。只有涉及研究路线、独立认证或 TASK01 accepted file 的阻塞问题才暂停并报告用户。>

