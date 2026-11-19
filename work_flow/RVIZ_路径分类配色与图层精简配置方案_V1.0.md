# CopTrans_base RViz 路径分类配色与图层精简配置方案 V1.0

> **用途**：用户与 ChatGPT 确定的独立可视化调整需求，交给 Claude 实施；不涉及轨迹优化、碰撞判定或数学认证。该文件是**配置及最小显示代码修改方案**，不是已执行的修改记录。配套图片为视觉示意，不是程序实测截图。
>
> **仓库**：`https://github.com/WangxQiang21/CopTrans_base`；分支 `feature/virtual-rigid-formation-a`；核对 HEAD `222b919329bf487db1ee096e2902d753385a3527`（2026-11-19）。
>
> **用户本地目标路径**：`/home/wq/whut_thesis/expv3_multi_virtual_a/work_flow/RVIZ_路径分类配色与图层精简配置方案_V1.0.md`。由用户下载后自行放入；**不需要 ChatGPT 修改本地仓库**。

## 1. 用户目标与方案总原则

**核心原则：一类路径只对应一种颜色，不再按车辆 ID 为同类路径配三种颜色。** A*、Portal Guide、未认证候选、最终已认证路径各用固定颜色；车辆中心与虚拟参考点 F 属于同一类别时只以线宽和可见性区分。安全状态不靠颜色单独表达，保留明确的 `UNVERIFIED/DIAGNOSTIC_ONLY` 和 `PUBLISHED/CERTIFIED` 字样。

现有 RViz 默认打开了 3 条 A*、3 条 Guide、3 条候选车辆中心、3 条最终车辆中心、F 候选，以及最多 80 帧 × 3 辆车的候选 OBB。多个图层同时绘制，蓝色 OBB 密度过大、轨迹与标签交错，遮盖地图。目标是实现**默认清爽查看，按需打开详细诊断**；并保留当前路径的 RViz `nav_msgs/Path` 颜色和线宽可编辑能力。

## 2. 配色与线型（正式配置表）

| 算法/几何类别 | 颜色 | RGB | 线宽/样式（m） | 默认 | 语义 |
|---|---|---|---|---|---|
| A* 原始路径 | **橙 `#F97316`** | `249; 115; 22` | `0.025`，细实线 | 当前使用路线 ON，其余 OFF | 拓扑候选，不是可执行轨迹 |
| Portal Guide | **绿 `#22C55E`** | `34; 197; 94` | `0.035`，中实线 | 当前使用 Guide ON，其余 OFF | 优化初值，不是可执行轨迹 |
| F 点候选轨迹 | **黄 `#FACC15`** | `250; 204; 21` | `0.047`，中实线 | ON | IPOPT 候选，未通过全部审核时仅诊断 |
| 三辆车候选中心轨迹 | **黄 `#FACC15`** | `250; 204; 21` | `0.024`，细实线 | 默认 OFF（手动诊断开启） | 与 F 候选属于**同一类别**，不因车号分色 |
| 三辆车最终已认证轨迹 | **亮蓝 `#38BDF8`** | `56; 189; 248` | `0.065`，粗实线 | ON | 只在合格证书发布后表示最终可执行轨迹 |
| 二维 F 工作空间走廊填充 | **柔红 `#E87986`** | `232; 121; 134` | alpha `0.09` | ON | 不是三车认证自由域 |
| 走廊外边框 | **深玫红 `#B45362`** | `180; 83; 98` | `0.013`，alpha `0.55` | ON | 提供走廊边界，不盖住轨迹 |
| 候选车辆 OBB | **灰蓝 `#94A3B8`** | `148; 163; 184` | `0.010`，alpha `0.38` | 稀疏 ON | 车辆的候选几何快照，不是最终安全认证 |
| 当前正在播放的车身 OBB | **保持已有运动模式颜色** | — | 原有宽度或轻微减小 | ON | 不改变车辆 A/O/S 模式显示语义 |
| 失败位置/文字 | **洋红 `#EC4899`** | `236; 72; 153` | 十字标记 + 短标签 | 有真实失败位置时 ON | 失败位置；不可用时不要画假失败点 |

**线型限制**：仓库路径目前用 `nav_msgs/Path` + RViz `rviz/Path`，其 `Line Style: Lines` 不直接支持虚线。为保持路径可编辑，A* 先使用细橙色**实线**，**不为虚线再增建 Marker 的重复轨迹**。

**颜色约束**：`vehicle_1/2/3` 必须完全相同的 `#38BDF8`；`candidate_vehicle_1/2/3` 与 `f_candidate` 必须完全相同的 `#FACC15`；不同车辆靠线条空间位置、车身 OBB 和按需标签区分，不靠线色区分。

## 3. 默认图层清单与切换策略

| 图层/话题 | 默认显示 | 说明 |
|---|---|---|
| `/map` | ON | 原始地图 |
| `/formation/path/astar_active`（建议新增显示别名） | ON | 实际当前候选使用的 A* 路线，橙色 |
| `/formation/path/portal_guide_active`（建议新增显示别名） | ON | 与同一候选绑定的 Guide，绿色 |
| `/formation/path/astar_raw_{0,1,2}` | OFF | 保留三路独立调试话题，不删除 |
| `/formation/path/portal_guide_{0,1,2}` | OFF | 同上 |
| `/formation/path/f_candidate` | ON | 黄线，当前候选；即使失败也可保留供诊断 |
| `/formation/path/candidate_vehicle_{1,2,3}` | OFF | 三条黄线，用户需要查看三车候选时手动开启 |
| `/formation/path/vehicle_{1,2,3}` | ON | 三条亮蓝粗线；只显示真实已认证结果 |
| `/formation/corridors` | ON | 红色半透明走廊，**F only** |
| `/formation/planning_candidates` | ON（仅稀疏 OBB 和失败 marker） | 不再画 80 个密集 OBB |
| `/formation/guides` 标签 | 默认 OFF（需要时可开） | 减少车旁重复文字 |
| `/formation/markers` | ON | 当前车辆、方向箭头、必要的短标签；播放行为不变 |
| `/formation/hud` | ON | 当前步骤、状态、失败原因；此次主要不改 HUD 业务逻辑 |
| `/formation/TF` | 推荐默认 OFF | 可手动用于坐标排错，避免全图坐标轴和箭头拥挤 |

### 当前候选自动选择必须真实准确

现有 `_0/_1/_2` 话题名表示候选下标，不保证求解器正在使用 0。**禁止仅关闭路线 1/2，假装路线 0 是当前真实 Guide**。推荐在现有 playback 节点中增加两个轻量别名 Path publisher：`/formation/path/astar_active` 和 `/formation/path/portal_guide_active`，它们从**同一份已接收、已通过身份验证的 planning_debug** 以及本轮真实 `route_index` / corridor round 选择对应的现有路径点。只复制数据显示，不重新拟合、不平滑、不重采样、不运行优化。

- stage 进行中、尚未得到 `route_index`：可以暂时不显示 active Guide；显示当前已明确选择的路线之前不要把 route 0 冒充为 active。
- 候选路线/round 发生变化：先清空旧 active Path，再显示新路径；不留跨代图层。
- 失败：保留该 plan 最后有效的诊断 Path 和失败位置，直到新的**已接纳**规划请求或 2D Pose Estimate/reset 清理。
- `PUBLISHED`：最终蓝线仅从原有 verified envelope 派生，不能把黄色 candidate 改色当作蓝色最终轨迹。
- 如果现有代码已能在不加 topic 的条件下可靠识别并只画当前路线，可选择**更小的等效实现**；Codex/Claude 必须说明真实 route_id 映射，不得硬编码 0。

## 4. 降低 OBB、标签和遮挡密度

### 4.1 Candidate OBB

当前 `formation_playback.py`：`CANDIDATE_OBB_LIMIT=80`，`sample_candidate_path()` 从真实 `state_at_s()` 采样；三车同时绘制会形成满屏栅栏。

**新默认：每辆车最多 5 个全路径代表性 OBB**（建议弧长比例 `0%、25%、50%、75%、100%`），只用于显示；三车总数约 15 个。若真实失败位置具有 `s_m`，允许为失败车辆添加**一个**就近特殊快照，并使用洋红失败标记突出位置。采样需从实际 `state_at_s()` 提取 `theta_i`，**绝不使用 `psi_F` 代替车辆车身朝向**。保持可切换的详细诊断选项是加分项，但**不要求**为此开发配置管理框架。

- 候选 OBB：灰蓝淡线（alpha 0.38），不再使用高饱和亮蓝。
- 当前播放的三车 OBB：仍保持清楚可见；不要受候选采样限制影响。
- 不修改生产轨迹、NLP 采样、数值认证与碰撞审查的任何分辨率或阈值。仅控制**显示量**。

### 4.2 文字标签

- 关闭默认全部 Guide Route 的文本标签；必要时只展示当前所选路线名称/编号。
- 候选中心每辆车的中点 `carX centre | DIAGNOSTIC_ONLY` 大标签默认关闭或合并到 HUD；保留 `F candidate` 简短单行，建议缩小到 `0.14 m`。
- 候选失败点仅显示一条短标签，例如 `CORRIDOR_AUDIT_WIDTH_LIMIT @ s=7.76m`；详细异常堆栈留终端，不挤进地图。
- 播放中的车辆编号允许保留，但标签字号和高度应与地图比例协调。

### 4.3 层叠和视觉优先级

视觉优先：**已认证蓝色轨迹 > 黄色候选 > 绿色 Guide > 橙色 A* > 红色走廊填充**。不更改 x/y 位置；如 Path 因 z-fighting 不清晰，只允许对独立**显示消息**增加很小的 z 排序偏移，不能改变原始轨迹、车辆高度或模型签名。使用 `rviz/Path` 的显示级线宽，不叠加同一条 `Marker LINE_STRIP`。

## 5. 工程修改位置（根据 HEAD 222b919 核对）

| 文件 | 具体工作 | 注意 |
|---|---|---|
| `catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz` | 将所有 `rviz/Path` 颜色统一为四类；设置线宽、默认可见性；若采用 active alias，新增两个 `rviz/Path` display；调整 MarkerArray 和 TF 的默认开关 | 当前确有 13 个路径显示，颜色按车辆混用 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py` | 调整 `CORRIDOR_FILL_COLOR/ALPHA`、`CORRIDOR_EDGE_COLOR/ALPHA`、`CANDIDATE_OBB_COLOR`、默认 OBB 数量/采样站点、候选标签密度 | 保持当前抽样来自真实 `path.state_at_s()`，不改安全/发布逻辑 |
| `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py` | 如采用 `*_active`，在现有 `on_planning_debug`、路径发布和清理函数中**最小增加**两条 Path publisher 及正确 route 选择；处理失败保持、新请求/reset 清理、过期代数拒绝 | 必须沿用现有 `corridor_display_matches()` 身份检查；禁止跨 plan 混叠 |
| `tests/test_virtual_a_corridor_display.py`、`tests/test_formation_demo_assets.py` 等现有测试 | 更新默认图层、颜色、OBB 个数/值、active route 选择与清除负例 | **不要**顺带重写测试框架 |

**不要动的范围**：`optimizer.py` 的 NLP 数学、`pipeline.py` 的候选算法、`corridor.py` 的几何构造与审计判据、三车碰撞和动力学检查、`/formation/playback_plan` 发布条件、旧 v2 认证模型与历史哈希。

`run_demo.sh` 已是用户唯一默认研究启动入口，此次只需确保仍加载更新后的 `formation_demo.rviz`（目前脚本通过 `tools/configure_formation_view.py` 从模板自动生成地图自适应配置），**不需要把显示调色重构成新脚本**。如果运行时 RViz 意外仍加载旧颜色，先检查用户自定义 `rviz_config:=` 覆盖项及旧进程。

## 6. Claude 实施顺序（一次小型显示调整，不拆开发任务）

1. 记录当前 HEAD 和 `git status`，核实默认 `run_demo.sh --dry-run` 的研究后端及 RViz 模板来源。
2. 先更新 `.rviz`：统一四类配色和开关，不动几何计算；打开任一已保存成功/失败会话确认颜色和图层是否正确。
3. 降低 candidate OBB 密度和文字标注，确保采样数据仍来自**相同真实候选状态**。
4. 为当前真实 route/guide 增加最小 active 显示（若采用本方案中的两个 alias）；检查当前 route 为 1 或 2 时不会显示 route 0。
5. 运行现有定向单测及 ROS/RViz 实机：成功案例、失败案例、跨 route 切换、手动 2D Pose Estimate 重置、过期请求拒绝。保留实际截图及执行记录。
6. 只提交显示相关文件与测试；写简短交付文档。Codex 对 diff、身份、安全隔离和实测截图进行独立审核。

## 7. 强制验收（用户打开 RViz 一眼判断）

- **配色**：橙=A*、绿=Guide、黄=Candidate、亮蓝=Certified；三辆车最终蓝色完全一致，三辆车候选黄色完全一致；走廊仅作为低透明红色背景。
- **清晰度**：默认只突出正在使用的 A*/Guide；候选 OBB 不再形成数十个密集蓝框；默认无满屏重复的大段文字。
- **真实性**：active route/guide 来源和求解器使用的候选对应；未认证黄色路径绝不变成 `/formation/path/vehicle_{i}` 上的蓝色认证轨迹。
- **失败场景**：保留本轮诊断，HUD 显示失败阶段/原因，蓝色认证路径为空或保持符合既有明确规则的状态；新的已接纳请求/reset 清除旧路径。
- **运行**：用户通过 `/home/wq/whut_thesis/expv3_multi_virtual_a/run_demo.sh` 默认启动即可看见新配置。至少给出 U-turn 失败现场和一个已认证成功场景的截图。
- **代码**：原三车动力学、数值、碰撞审核全部未修改；仅显示/显示派生数据发生变化；相关单测通过。若环境无法运行 RViz，要在交付中标注 `NOT_RUN`，不谎报截图验收。

## 8. 最终交付材料

Claude 在 `work_flow/` 内提供：

- `RVIZ_配色调整_Claude交付.md`：变更 commit、文件列表、实测场景、颜色/图层、功能对比、已知限制、截图目录。
- `RVIZ_配色调整_evidence/`：至少两张不裁剪真实运行状态的 RViz 截图、真实运行日志、最小定向测试结果。

Codex 审核产物：`RVIZ_配色调整_Codex审核.md`，按上述强制验收逐项标记 PASS / FAIL / NOT_RUN。**这是一项显示改造，不增设新的算法开发任务，不改变 TASK02–TASK04 算法研发边界。**

---

### 用于效果预览的声明

配套图示只展示期望的颜色分层、线宽、OBB 稀疏程度和 HUD 相对布局；它不是从真实 `run_demo.sh` 渲染的输出。最终以 Claude 修改后的真实 RViz 截图为准。
