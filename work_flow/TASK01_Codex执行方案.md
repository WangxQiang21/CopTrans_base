# TASK 01｜Codex 详细执行方案（交 Claude Code）

> 状态：Step 0 已由 Codex 独立核验通过；后续 Step 1–6 由 Claude 在用户明确下达后实施。本文件不授权自动进入 TASK 02。  
> 范围：同一个 TASK 01 内依次完成可视化、默认入口、winding、S 域、生产积分证书和回归；每一步有 Claude 自检检查点，TASK 01 全部完成后再由 Codex 做一次整体独立验收。  
> 禁止：未认证候选进入 Playback、放宽安全阈值、改三车模型、改 A*/走廊算法、增加新管理框架。

## 1. 真实基线与审查结论

审查时工作区：

```text
pwd=/home/wq/whut_thesis/expv3_multi_virtual_a
realpath run_demo.sh=/home/wq/whut_thesis/expv3_multi_virtual_a/run_demo.sh
branch=feature/virtual-rigid-formation-a
HEAD=77805cd46849ebadc374187f9b00fce0f7151f84
git status --short=?? work_flow/
```

`work_flow/TASK01_技术方案.md` 是审查前已有的未跟踪输入。Claude 不得清理、覆盖或把它误报为自己的开发改动。开始开发时记录新的真实 HEAD；若不是上述 SHA 的后代，或存在不能隔离的生产代码修改，停止并报告。

已核实的实现事实：

- `run_demo.sh:393-401` 默认仍为 `uniform_v02`；`run_virtual_a.sh` 只注入 backend/auto_goal。
- `formation_demo.launch:15,79,115-125` 默认仍为 `uniform_v02/legacy_route`，没有 enforcement 参数。
- `formation_uniform_planner_node.py:1303-1308` worker 同步等待 `hooks.solve()`；`1339-1375` 失败后清空 planning debug。
- node 只在结果返回后由 `_publish_corridor_debug()` 导出当前最后一轮走廊；pipeline 没有 progress callback。
- playback 的 `on_planning_debug()` 只接受当前 job 的 `PLANNING/PLAYING`，失败时不能冻结图层；现有 `/formation/guides` 已有橙/绿 namespace，A* 默认关闭且不是虚线；走廊颜色当前按 route 变化，不是固定红色。
- `AttemptRecord.candidate_path` 已保留真实候选对象。TASK 01 的黄线、蓝色三车中心和 OBB 必须从该对象或最终 `result.path` 的 `state_at_s()` 派生，不新增独立插值轨迹。
- `_search_packet()` 仍使用 shortest wrap 和 `min(60, 3d+10)`；同一 packet 被初值、NLP 和残差使用，修复必须集中到这里。
- `VirtualAPath._plan_integration()` 仍使用每 Simpson accumulation panel `1e-10 m` 的 guard。独立 audit 已有逐面板 `panel_m4_bounds()` 和其自身 `evaluation_guard()`。
- 旧序列化模型重建会逐字比较 `integration_budget`。改变生产证书后不能继续冒用 `virtual_rigid_a.numerics.g1.v2`，否则旧模型无法重建且新旧语义混淆。
- 既有 U-turn 复现材料存在于 `evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z/`；其 `/tmp/expv3-repro-map.t2Au9l/.../map.yaml` 当前存在，但证据不得依赖临时路径长期存活，运行时优先从仓库 `maps/map05-p12-parking-row-env2/` 按原比例流程生成或记录实际 grid SHA。

本机审查环境：Python 可导入 CasADi 3.7.2；当前未 source ROS，直接 Python 无法导入 `rospy`。因此 Claude 必须分别记录离线测试和 source ROS 后的测试，不能把 dry-run 当 ROS 实测。

## 2. 冻结的实现合同

### 2.1 Progress 事件

不新增 event bus、manager 或 ROS 消息。使用现有 `/formation/planning_debug` JSON topic，并增加一个轻量、ROS-free 的回调：

```python
# pipeline.py
def plan_virtual_a(map_request, progress_callback=None): ...

# ros_adapter.py
def solve(request, progress_callback=None): ...

class VirtualABackendHooks:
    def solve(self, request, progress_callback=None): ...
```

pipeline 内部事件是只读 dict，最少字段：

```text
schema=virtual_a_progress/v1
stage=REQUEST_ADMITTED|ASTAR_SEARCH|CORRIDOR_BUILD|PORTAL_GUIDE|
      NLP_SOLVE|GEOMETRY_AUDIT|DYNAMICS_AUDIT|OBB_COLLISION_AUDIT|
      PUBLISHED|FAILED
stage_index=0..8
state=PLANNING|FAILED|PUBLISHED
elapsed_s, route_index, route_count, round_index
request_sha256, grid_sha256, planning_context_sha256
metrics={}                 # 只含已真实得到的数据
diagnostic_path=<进程内对象或 None>  # 绝不直接 JSON 序列化
```

`generation/backend/formation_id/plan_id` 由 node 的 admission closure 加入，不能由 pipeline 猜测。`model_sha256` 在权威模型尚不存在时必须为 `null`。node 发布前验证 closure 中的 generation、当前 state、map signature、request/grid/context SHA；迟到或不匹配事件直接忽略，不清图层。

新增 `UniformPlannerNode._on_virtual_progress(generation, plan_id, snapshot, event)`。它在 worker 线程中获取现有 `self._lock`，只做身份检查、有限采样和一次 ROS publish，不反向改变 solver、预算或 `search`。callback 异常必须被显式记录为 `PROGRESS_DISPLAY_ERROR` 并停用该请求后续显示回调，但规划和安全审核继续；禁止用捕获 `TypeError` 的方式静默回退旧 solve 签名。

pipeline 的 `_emit_progress()` 只负责验证阶段名、真实 monotonic elapsed 和调用 callback。事件允许 route/round 重试导致 stage 回跳；`stage_index` 不是百分比。

### 2.2 Planning debug 与 Marker 分层

保留输入 topic `/formation/planning_debug`，输出：

- `/formation/corridors`：固定红色半透明 F 点二维凸多边形及深红边界；
- `/formation/guides`：橙色虚线 raw A*、绿色 Portal guide；
- 新增且仅新增一个 `/formation/planning_candidates` MarkerArray：黄色 F 连续候选、蓝色三车中心线/稀疏 OBB、可靠失败点；
- `/formation/markers` 继续只画已验证 Playback，不承载未认证候选。

稳定 namespace：

```text
formation/planning/astar_raw
formation/planning/corridor_f_only/fill
formation/planning/corridor_f_only/edge
formation/planning/portal_guide
formation/planning/f_candidate
formation/planning/car{1,2,3}_center_candidate
formation/planning/car{1,2,3}_obb_candidate
formation/planning/failure
```

A* 虚线仅为显示样式：沿原始相邻格网线段生成 `LINE_LIST` 的交替短段，不改变原 polyline 支撑集，也不回写规划输入。candidate 采样上限固定为一个显示常量（建议 400 个 F/中心点、最多 80 组 OBB）；若不足以覆盖全长则等弧长采样并包含 `s=0,S`。每个点通过同一个真实 `path.state_at_s(s)` 求值；OBB 使用该状态中的各车中心/独立车体航向和 `request.vehicle_shapes`，不得固定为 `psi_F`。

候选分类：

- solver 构造出 `candidate_path` 但三项审核未全过：`UNVERIFIED_CANDIDATE / DIAGNOSTIC_ONLY`；
- 三项审核全过且原子信封发布成功：`CERTIFIED`，但执行仍只来自 `/formation/playback_plan`；
- 没有真实 path：不画黄线/蓝车；
- failure 只有实际 `first_failure` 世界坐标，或可靠 `failure_s` 且同一 candidate 可求值时才画。

### 2.3 图层生命周期

- `_admit_virtual()` 成功增加 generation 后清空上一请求诊断，再发 `REQUEST_ADMITTED`；未接纳请求和 busy reject 不清除当前诊断或已认证 Playback。
- 每个新事件替换同 generation 的相应层，但保留前面阶段已生成层。
- `_finish_virtual_failure()` 发布最终 `FAILED` progress 文档并冻结最后层，不调用 `_clear_planning_debug()`。
- playback matcher 接受当前身份的 `PLANNING/FAILED/PUBLISHED`；FAILED 不 DELETEALL。只有新 admission 的 `CLEARED` 清除全部三类诊断 marker。
- `on_request()` 对 virtual backend 不再以“收到原始消息”等同“已接纳”；清除动作移至匹配 generation 的 `on_job_state(PLANNING)`/planning debug admit 事件。uniform/legacy 原行为保持。

### 2.4 配置兼容

保持 `PlannerConfig()`、`_PLANNER_CORRIDOR_DEFAULTS` 和缺字段历史 payload 的 `legacy_route/hard` 语义不变，避免历史 planning-context SHA 漂移。新增一个明确的研究工厂而不是改变 dataclass 隐式默认：

```python
def research_planner_config(**overrides):
    # 固定 guidance_mode='corridor', corridor_enforcement='hard'
```

ROS 当前研究入口、TASK 01 离线复现和以后新研究调用使用该工厂；历史 fixture 与 `planner_config_from_dict()` 继续使用旧默认。工厂仍调用 `validate_planner_config()`，不产生第二套配置类。

## 3. 同一 TASK 01 的实施顺序

#### 验收与推进规则

- Step 0 已完成并经 Codex 核验通过。Claude 从 Step 1 开始，不重复运行 Step 0，也不覆盖 `TASK01_evidence/00_baseline/` 中已核验的证据。
- 每个 Step 是 TASK 01 内部实施步骤，不是独立管理任务或 Codex 验收门。Claude 完成该 Step 后，运行该步定向测试、核对本步停止条件，并在 `work_flow/TASK01_Claude开发交付.md` 记录命令、退出码、结果和证据；自检通过即可继续下一个 Step，无需等待 Codex 逐步审核。
- 某 Step 自检失败时，Claude 先在该 Step 范围内修复并重跑相关检查，不带着已知失败进入依赖步骤。若触发第 9 节的研究决策边界，停止相关实现并报告，不自行改变合同。
- Step 1–6 全部完成、最终回归与可运行的 ROS/RViz 检查完成或如实标记受环境阻塞后，Claude 更新交付报告并停止。届时 Codex 才依据完整 diff、逐步自检证据和最终测试结果，对整个 TASK 01 做独立验收；Codex 未给出 PASS 前不得宣称 TASK 01 验收通过，也不得进入 TASK 02。
- Claude 的步骤自检不能替代 Codex 的最终独立验收；Codex 的最终 PASS 也不替代用户的 RViz 手动验收。

### Step 0：冻结基线和复现输入

只记录，不改算法：

1. 保存 HEAD、dirty state、Python/CasADi/ROS 版本、map YAML SHA、grid SHA。
2. 将旧 `run_demo.sh --map map05 --scale 2.625 --dry-run` 输出保存为 `work_flow/TASK01_evidence/00_baseline/default_dry_run.log`。
3. 使用 UTURN_DIAG 的原脚本各运行一次 600003/600004，输出到新 evidence，不覆盖历史 JSON；如果临时地图与仓库重建地图 grid SHA 不同，停止并先说明，不能横向比较。
4. 记录旧结果的 base heading、S 上下界、first failure、wall time。旧结果不是新实现验收。

本 Step 已完成并经 Codex 核验通过。本次执行只核对并沿用已有证据，不重跑、不覆盖。

已核验停止条件：基线 SHA、仓库重建地图身份及 600003/600004 复现输入均已锁定。若 Claude 开始 Step 1 前发现 HEAD、工作区或证据文件与核验结果不一致，停止并报告，不重做或覆盖证据。

### Step 1：Progress、五图层、HUD 和失败冻结

修改：

- `pipeline.py`：增加上述可选 callback；在真实操作开始/完成处发阶段事件。A* 后带 raw route 数量/长度；corridor 后带真实 chain/audit/digest；guide 后带 guide/quality/length/allocation；NLP 前为 running，返回后带 attempts/status/iterations/S/constraint residual，并从 `geometry.path` 或最后一个非空 `AttemptRecord.candidate_path` 提供真实诊断 path；三项 audit 前后分别发实际 report 摘要；失败路径发 `FAILED`。pipeline 不得在仅构造成功结果时发 `PUBLISHED`。
- `ros_adapter.py`：只透传 callback，旧 `solve(request)` 仍合法。
- `formation_uniform_planner_node.py`：worker 用显式 keyword 传 callback；新增身份检查和渐进 debug document 构造；复用 `_display_from_corridor_round()`，不要复制走廊几何；失败不清空。
- ROS-free `formation_playback.py`：扩展现有 MarkerSpec builders，新增 `build_candidate_markers()` 和 progress HUD 文本；将 corridor 固定为红色；A* 改橙色虚线，Guide 绿色；验证所有数值有限、身份一致、诊断标签不会升级为证书。
- playback ROS script：增加一个 candidate publisher、clear 函数和 planning debug 分发；HUD signature 加 `stage/route/round/reason`，10 Hz 只更新时间，不重发大几何。
- RViz：A* namespace 默认开启；走廊名称改为 `Convex workspace corridors (F point only)`；新增 candidate MarkerArray display。颜色由 Marker 固定，RViz 不覆盖。

不得改 optimizer 数学。可视化构造失败只能清该诊断层并报告，不能使规划失败或发布候选。

定向验收：真实 stage 顺序/允许回跳；错误 plan/generation/grid/context/request 事件零发布；FAILED 保留；新 admission 清除；busy reject 不清；candidate 点逐点等于 `state_at_s`；三车刚性距离误差 ≤ `1e-10 m`；OBB 角等于实际车辆 heading；未认证 path 从不触发 playback publisher。

`PUBLISHED` 只由 ROS node 在通过既有三项独立 audit、原子 playback envelope 成功发布之后发出；因此它表示真实发布完成，不表示 pipeline 求解器返回 success 或成功对象已构造。失败、审核未通过或 envelope 发布失败时不得发 `PUBLISHED`。

### Step 2：默认研究入口和 corridor/hard 透传

修改：

- `run_demo.sh`：formation 无显式 backend 时改为 `virtual_rigid_a`。对该 backend 分别扫描 guidance/enforcement/auto_goal 是否已显式给出；缺省注入 `corridor/hard/false`，同名重复或冲突明确退出 2。显式 `uniform_v02`/`legacy` 不注入 virtual 参数，沿用原 auto_goal 行为。`--single` 完全不变。
- `run_virtual_a.sh`：仍为薄 wrapper，只补缺失的 backend/guidance/enforcement/auto_goal，不复制 map/GUI 逻辑。
- launch：默认 `backend=virtual_rigid_a`、guidance `corridor`，新增 `virtual_a_corridor_enforcement=hard` 并只对 virtual node 透传。
- node `main()`：读取并规范化两参数，使用 `research_planner_config()`；启动日志打印实际 validate 后配置。soft 仍允许显式实验，但默认 hard；未知值启动失败。
- `contracts.py`：增加研究工厂；不改历史 dataclass/as_dict/from_dict 默认。

验收必须贯通：dry-run 参数 → launch XML → ROS param → hooks.settings.planner_config → pipeline `corridor_mode` → `_build_corridor_constraint()` → CasADi `g` 半空间数量。新增结果/进度字段 `corridor_halfspace_constraint_count` 和最大残差，计数必须来自实际 `_CorridorConstraint`，不能根据红色 polygon 猜。

### Step 3：候选级 winding

增加最小不可变合同 `HeadingBranch`（可放 `contracts.py`）：

```text
base_heading_rad, winding_k, unwrapped_heading_rad,
guide_turn_rad, source, candidate_rank
```

计算：删除连续重复/近零 guide 段；令切向序列包含起始前向切向 `psi0`、各有效 guide 段 `atan2`、终点切向 `psig`，逐项累加 `wrap(phi[j+1]-phi[j])` 得 `Theta_G`。`base=wrap(psig-psi0)`，首选 `k0=round((Theta_G-base)/(2π))`，按 `|base+2πk-Theta_G|` 排序有限集合 `{k0,k0-1,k0+1}`。同分时确定性选择较小 `|k|`、再较小 k。退化/no-guide 只给 `k=0`。

pipeline 对每条 route/guide 建立自己的 branch，不改物理 goal。默认只尝试首选；仅在该分支没有 audited candidate、原因属于 solver non-convergence/endpoint constraint，且现有请求 soft budget、未用 initial 和唯一 recovery 均允许时，最多尝试排序中的下一个分支。该尝试计入现有 `MAX_RECOVERIES=1` 和 request-wide `MAX_INITIALS`，不得额外叠加无限分支。碰撞/动力学/数值认证失败不触发 winding 猜测。

重构 `_search_packet(request, config, heading_branch=None, guide_length_m=None)`，先生成一次 packet，再传给初值、route guess、NLP、numeric/CasADi residual 和 candidate metadata。禁止 `make_initial_guesses()` 与 solve 各算一份 heading/S 域。`GeometryResult.goal_heading_branch_rad` 记录 unwrapped 值；Attempt/search 同时记录 base/k/guide turn。audit 终点继续用 wrap 后物理角误差。

### Step 4：Guide 相关有限 S 域

在同一个 packet 中计算：

```text
kappa_ref_max = min(config.kappa_limit,
                    1/request.limits.r_min_ackermann)
S_lower = max(intervals*0.05, d, abs(unwrapped_heading)/kappa_ref_max)
S_upper = max(S_lower + 2.0, 12.0, 3*d + 10.0,
              1.15*guide_length_m)       # 有可信 guide
S_upper = max(S_lower + 2.0, 12.0, 3*d + 10.0)  # 无 guide
```

`1.15` 是技术方案批准的首轮实验系数，必须用 600003/600004 和合成长直线记录验证；若证据表明不足，Claude 停止并报告，不自行调参。本 TASK 01 **不实现受控二次扩域**，避免把普通 IPOPT infeasible 错判为 S 不足。取消 60 m 总帽，但每次上界仍有限，solver/initial/soft wall/MAX_INITIALS 不变。

packet/guess/attempt/search 记录 `distance, guide_length, S_lower, S_upper, S_solution, upper_active`；`upper_active` 用求解容差而非字符串状态判断。50 m/100 m 直线构造证明没有旧帽；病态/预算反例仍 fail closed。

### Step 5：生产积分 v3 证书，保留 v2 重放

不要删除 `bounds.POSITION_BUDGET_M=1e-7`、角预算或独立 audit。实现版本化：

- `SUPPORTED_NUMERICS_VERSIONS={g1.v2,g1.v3}`，新建路径默认 v3；从旧 model JSON 重建时显式选择 v2，并保留现有 `_plan_integration_v2()` 字节语义。
- `C2CurvatureProfile.from_knots(..., numerics_version=None)`、`VirtualAPath(..., numerics_version=None)` 只接受支持集合；trajectory/audit 从序列化字段传版本，禁止“未知版本自动用最新”。
- 新 `_plan_integration_v3()` 使用 `integration_bounds.panel_m4_bounds()` 对每完整 Simpson panel 求 outward M4；`segment_budget()` 加一个任意前缀 local panel。
- 新增 `production_evaluation_guard()`，逐项对应 geometry 的 Horner `psi_at`、缓存 sin/cos、Simpson weighted sum、piece position prefix 和最多一个 local panel；可以复用 outward primitives，不能直接信任 audit 的最终 guard。独立 audit 仍从 raw knots 自己重建。

单位统一为单坐标米，最终：

```text
E_quad_coord = outward sum(full panel H^5*M4/2880 + one local panel/piece)
E_eval_coord = production float64/libm/prefix guard
E_angle_coord = S * E_psi
E_position = sqrt(2) * (E_quad_coord + E_eval_coord + E_angle_coord)
```

证书列出每段 subinterval、panel M4、quadrature/evaluation/angle/total、向外舍入方法和版本。细分时先试算“仅将最大 quadrature contribution 段 subinterval×2”的新总界；只有新 total 严格下降且截断项仍是可改善主因才接受。不可消除项已耗尽预算或新界不降，立即 `BOUND_UNCONFIRMED`，不循环到 MAX_PANELS。任意 `state_at_s()` 的 complete panels + one local panel 必须被同一证书覆盖。

旧 v2 仅用于重建/核验旧模型，不用于新规划；新 v3 model SHA、planning context 和证书自然变化，必须记录，不能篡改旧验收 fixture 的预期来“通过”。

### Step 6：全回归、ROS 与 RViz

完成所有离线测试后再 build/ROS。失败用例和成功用例都必须验证原子发布：未认证候选只出现 debug topic，`/formation/playback_plan` 计数为 0；成功仅在三项 audit 全过后发布一次。

## 4. 逐文件修改矩阵

| 文件 | 最小修改 | 不变量 |
|---|---|---|
| `run_demo.sh` | 默认与参数去重/注入/help | 单车、map scale、显式旧后端、dry-run |
| `run_virtual_a.sh` | 薄 wrapper 补 corridor/hard | 不复制主逻辑 |
| `formation_demo.launch` | 默认及 enforcement 透传、candidate topic无需 remap | 恰好一个 planner/backend |
| `formation_demo.rviz` | 开 A*、固定名称、新 candidate display | map frame、HUD、视角 |
| `tools/configure_formation_view.py` | 原则上不改；只有模板字段被脚本丢失时最小修复 | 只调整视角/输出，不改算法层 |
| `contracts.py` | research factory、HeadingBranch、必要诊断字段/版本集合 | 历史 PlannerConfig payload 和 digest |
| `pipeline.py` | progress callback、branch/packet 调度、真实阶段事件 | 原三项 audit 和总预算 |
| `optimizer.py` | packet 单一来源、branch/S 域、真实约束计数 | 19变量模型、终点等式、曲率约束 |
| `geometry.py` | v2/v3 生产积分与 state_at_s 证书 | path 方程、预算阈值 |
| `bounds.py` | 版本常量/低级 outward 运算；保留 v2 guard | 不放宽预算 |
| `integration_bounds.py` | production guard 的有依据扩展 | audit guard 独立调用 |
| `audit.py`、`trajectory.py` | 仅为版本化重建/独立检查所需 | 未知版本 fail closed |
| `ros_adapter.py` | callback 透传 | 旧无 callback 调用可用 |
| planner node | ROS 参数、worker callback、身份/失败冻结 | admission、原子 envelope |
| ROS-free playback | builders/HUD/matcher | Marker 不具执行权 |
| playback node | candidate publisher、生命周期 | Playback 只收验证信封 |
| tests | 在现有相关文件补测试；最多新增一个 progress contract test | 不复制测试框架 |

## 5. 测试命令和门槛

### 5.1 每步快速测试

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src

python3 -m unittest tests.test_virtual_a_corridor_display tests.test_formation_hud
python3 -m unittest tests.test_virtual_a_receiver tests.test_virtual_a_playback
python3 -m unittest tests.test_formation_demo_assets
python3 -m unittest tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer
python3 -m unittest tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery
python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_profile
python3 -m unittest tests.test_virtual_a_integration_bounds \
  tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit \
  tests.test_virtual_a_trajectory
```

每条单独保存 stdout/stderr、退出码和时长。不能用一次 `discover` 的总退出码掩盖具体失败。核心门槛：身份负例 0 marker/0 playback；candidate 与 path 差 ≤ `1e-10 m`；corridor hard residual ≤现有 `1e-8 m`；数值预算仍 ≤`1e-7 m`；旧 v2 model fixture 可重建，新 v3 篡改版本拒绝。

### 5.2 U-turn 和长路径

不要覆盖旧证据：

```bash
export PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src
MAP=$PWD/maps/map05-p12-parking-row-env2/map.yaml
OLD=$PWD/evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z
OUT=$PWD/work_flow/TASK01_evidence/05_algorithm
mkdir -p "$OUT"
python3 "$OLD/repro_u_turn.py" --map-yaml "$MAP" \
  --plan 600003 --plan 600004 --output "$OUT/repro_u_turn.json"
```

先核对 map/grid SHA；若旧临时缩放地图才是权威输入，则由现有 `run_demo.sh --scale 2.625` 生成运行时地图并记录路径，不能把未缩放 map 替代。修改 repro 脚本只允许适配新公开 API，不能 monkeypatch 新算法结果。

新增单测构造：`S=50 m`、`S=100 m` 直线；±π 两侧、CW/CCW guide、退化 guide；600003 应保持三项认证能力。600004 的目标是三个错误拒绝源消失，最终仍可诚实失败于碰撞/动力学/预算；只有三项审核都过才报告 SUCCESS。

### 5.3 构建和 ROS（Claude 有 ROS 环境时必须执行）

```bash
source /opt/ros/noetic/setup.bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws
catkin_make
source devel/setup.bash
cd ..
./run_demo.sh --map map05 --scale 2.625 --dry-run
./run_demo.sh --map map05 --scale 2.625
```

另开终端记录：

```bash
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
rosparam get /formation_uniform_planner/backend
rosparam get /formation_uniform_planner/virtual_a_guidance_mode
rosparam get /formation_uniform_planner/virtual_a_corridor_enforcement
rostopic echo /formation/planning_debug
rostopic echo /formation/job_state
rostopic echo /formation/quality_report
rostopic echo /formation/playback_plan
```

然后分别运行 `--backend uniform_v02`、`--backend legacy` 和 `--single` smoke。`tests/ros_virtual_a_smoke.py` 必须先阅读其 CLI，使用实际 scene manifest、独立 ROS master 和新输出目录；不得复用/覆盖 G5 历史结果。GUI 不可用时 Claude 标记 `RVIZ_MANUAL=BLOCKED`，但仍完成 build、headless ROS 和用户复现命令。

## 6. 证据目录和必填字段

统一写入：

```text
work_flow/TASK01_evidence/
  00_baseline/
  01_visualization/
  02_defaults/
  03_winding/
  04_s_domain/
  05_numerics/
  06_regression/
  07_ros_rviz/
```

只保存必要 JSON、命令日志和少量截图，不复制源码。每个实验 JSON 至少包含技术方案第 8.4 节字段；不存在写 `UNAVAILABLE`。额外保存：progress 事件序列、每事件 identity、Marker namespace/点数、candidate classification、playback 发布计数、halfspace constraint count/residual、winding 候选排序、S packet、v2/v3 certificate components。

截图命名：`plan_<id>_<stage>_<result>.png`；失败截图必须同时能看到 HUD reason 和已保留图层。截图只证明显示，不替代数值审核。

## 7. Claude 交付文档模板

写 `work_flow/TASK01_Claude开发交付.md`：

1. BASE SHA、最终 HEAD（未提交则写 `NOT_COMMITTED`）、branch、最终 dirty state。
2. 按上述 Step 0–6 列实际改动文件/函数和未实施项。
3. 配置有效链：shell→launch→ROS param→PlannerConfig→corridor constraint 数量/残差。
4. winding 与 S 的实际 600003/600004 数据，不得引用旧数字冒充新结果。
5. v3 数值证书公式、实现操作计数、50/100 m 结果、v2 重放结果。
6. 每条测试原始命令、退出码、时长、日志路径；NOT_RUN/BLOCKED 明示。
7. ROS plan_id、身份 SHA、事件序列、成功/失败 publication 数；RViz 截图路径。
8. 已知问题和范围外事项；不得宣称 F 走廊是三车 OBB 走廊或货物已认证。
9. 明确 `run_demo.sh` 默认是否已经更新并实测。
10. 停止，不提交/推送，除非用户另行授权；不进入 TASK 02。

## 8. Codex 独立审核清单

Codex 不依赖 Claude 自述，逐项给 `PASS/REWORK/BLOCKED`：

- diff 是否只在允许文件且无新框架/平行轨迹；历史 fixture 是否被篡改；
- 默认入口四层实际一致，显式旧后端和单车仍可用；
- progress callback 线程安全、身份 fail closed、回跳合法、callback 失败不改变规划；
- 五色/namespace/失败冻结符合规范，所有 candidate 点来自真实 path；
- 未认证 candidate 对 playback publisher 为 0，成功信封仍经原三项 audit 和 verifier；
- winding 的 branch 在 packet/guess/NLP/residual/audit metadata 一致，物理 goal 未改；
- S 域无 60 m 帽、有限且与 guide 绑定，MAX_INITIALS/recovery/budget 未扩大；
- v3 M4/float/angle/total 公式与实际操作一致，细分可单调改善，50/100 m 正例和病态负例真实；
- v2 历史模型重放、未知版本和篡改证书负例；
- 独立复跑定向测试、600003/600004、至少一次 headless ROS；GUI 结论与用户手验分开。

任何一项安全门被关闭、阈值放宽、旧轨迹在失败后误播放、或用 G1 人工/旧 JSON 冒充新规划，直接 REWORK。

## 9. 回滚和重大设计变更

实现按 Step 保持可按文件/步骤识别的 diff；本任务当前未获提交授权，不得自行 commit/push。回归定位顺序：显示层→默认参数→branch packet→S packet→numerics version。不得用 `git reset --hard`；只撤销本任务明确文件。

以下情况 Claude 必须停止请求用户/ChatGPT 决策：需要改变 19 变量曲率模型、增加 winding 整数优化、超过一次 recovery、改变 `1e-7 m` 数值预算、改变车辆/编队几何、将 OBB 放入 NLP、无法在保留 v2 重放时实现 v3、或需要修改独立碰撞审核语义。普通接口/测试修复不升级。

本方案经用户确认后才开始开发。TASK 01 整体通过前，不自动开始 TASK 02。
