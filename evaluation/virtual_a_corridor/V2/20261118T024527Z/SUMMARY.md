# V2 A*、中线与最终三车轨迹分层 — 证据摘要

Run: `evaluation/virtual_a_corridor/V2/20261118T024527Z`（UTC，2026-11-18）
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
分支: `feature/virtual-rigid-formation-a`
任务指定基线: `23fd16227e75a68f03f3ac7483555063c8189313`
当前 HEAD: `9a19fd3e22d33188b13795f695757c3e4a36909f`

**V2 改动未提交。** 当前 HEAD 还包含后来验收的 C0-A/C0-B
`70a2336` 与 C1 `9a19fd3`，任务基线是 HEAD 的祖先；HEAD 中没有任何 V2
提交。未创建分支、未 commit、未 push。工作区另有此前阶段的未提交改动，
因此 `git diff --stat` 是累计值（13 files, 3860+/178-），**不是 V2 净改动**。

## 1. 改动边界与净 diff

任务要求只改既有 V1 debug 导出/显示函数、`formation_demo.rviz` 和
`tests/test_virtual_a_corridor_display.py`；未新增 topic、schema、ROS 节点，
未改 solver/audit。

V2 净 diff 由重建的 pre-V2 已验收快照生成（`/tmp/v2_pre_20261118`）。该快照
由 HEAD + 已验收 V0/V1/C2-A/C2-B/C3 补丁重建；六个 C3_REWORK_02 源码 hash
逐一相等，重建过程见 `pre_v2_reconstruction.log`。

| 文件 | 增/删 |
| --- | --- |
| `formation_continuous_a/corridor.py` | +136 |
| `scripts/formation_uniform_planner_node.py` | +62 |
| `fwis_v3_optimizer/formation_playback.py` | +187 |
| `fwis_v3_playback/scripts/formation_playback.py` | +53 |
| `fwis_v3_demo/rviz/formation_demo.rviz` | +17/−19 |
| **合计** | **5 files, 436 insertions, 19 deletions** |

`v2_source_changes.patch`（sha256 `fc0afabd…97c3ada`）即上述净 diff。
`tests/test_virtual_a_corridor_display.py`（938 行）在 V2 前未被 git 跟踪，
不能作为普通 diff 计算，故单独全文保存在本目录。完整源码 hash 见
`source_hashes.txt`。

实现要点：

- `corridor_display_fields(...)` 在原有 `planning_debug` payload 中增加
  `raw_route_points`、`guide_points`、`guide_quality`、`route_id`、
  `corridor_used`、`chain_sha256`；只复制实际 `result.search` 走廊轮次和实际
  `result.routes[route_index]` 的点，不做插值、重采样或示意数据。
- planner 节点构造 V2 display 时核对 grid/context/model、route index、
  guide dict 和 chain payload；任何一处不符就清空，不拿旧几何顶替。
- 库 `formation_playback.py` 新 `build_guide_markers`：
  - A* 路线：`formation/guides/astar`，橙色 `(1.0,0.55,0.0,0.95)`，宽
    0.018 m，z=0.012 m，默认隐藏，246 点；
  - C2-A 中线：`formation/guides/centerline`，绿色
    `(0.05,0.82,0.25,0.95)`，宽 0.03 m，z=0.012 m，默认显示，3 点；
  - 标签 `A* ROUTE | REFERENCE_ONLY`、`PL_GUIDE | REFERENCE_ONLY`；
  - 不合格 guide 标 `UNVALIDATED_GUIDE | PL_GUIDE | REFERENCE_ONLY`，黄色；
  - 未使用路线放 `*_candidate` namespace，带 `| CANDIDATE`，默认隐藏；
  - 标签不含 SAFE/CERTIFIED/VERIFIED/EXECUTABLE/FEASIBLE/PASSED 等安全词。
- playback 节点新增独立 `/formation/guides` 发布；与 `/formation/corridors`
  同一 stamp 原子替换，异常/CLEARED/新 request 同时清空两层。
- `/formation/markers` 继续复用既有 `build_markers`，保留 MODE_COLORS 与
  0.05 m 粗线；RViz 图层改名为 `Final certified vehicle paths`。
- RViz 新增 guides 图层，默认开关：
  `astar=false`、`centerline=true`、`astar_candidate=false`、
  `centerline_candidate=false`；corridors 图层名改为
  `Corridor candidates (reference point only, low alpha)`。

## 2. 真实 ROS 端到端结果

私有 master `127.0.0.1:11585`；真实
`formation_uniform_planner_node` + `formation_playback` + RViz
（窗口 `155189262`）。驱动只发布合成 OccupancyGrid、`/initialpose` 和普通
`FormationPlanRequest(plan_id=92001, formation_id="synthetic_g4")`；所有
plan/route/guide/corridor/final marker 都来自两个真实节点。

场景：半径 7.0 m、半宽 3.0 m 的窄弯环带；分辨率 0.05 m、520×520、
origin (−13,−13)；start `(3.5, −6.06218, 0.523599)`、goal
`(3.5, 6.06218, 2.618034)`；三车偏移 `(1,0)/(0,1)/(0,−1)`，车体
0.738×0.5 m。

实际 solver/plan 结果：

| 项 | 值 |
| --- | --- |
| pipeline / planner | `SUCCESS` / `FEASIBLE`，`executable=true`，`feasible=true` |
| plan_id / formation | 92001 / `synthetic_g4`，generation 1 |
| routes | 1 条，`route_id=0`，`route_source=astar_primary`，`corridor_used=true` |
| raw A* 点 | 246 |
| C2-A guide | 3 点，2 regions，chain digest/sha `8775e3d1…538f3183` |
| guide quality | stage `C2_A`, `rounding=PL_ONLY_NO_BEZIER`, `smoothness_claim=NONE`, `is_executable_path=false`, `qp_used=true`, `qp_status=SLSQP_SUCCESS`, warnings `[]` |
| 段长 / guide 长 | `[9.81968713822653, 2.5092987395753643]` / `12.328985877801895` m |
| T_end / 采样 | `39.602577282743304 s` / 1982 |
| 路径长 S | `14.255361542330647 m` |
| model / request / context / grid | `ea346350…23b651f` / `e6812ca7…fce4675987` / `ec12268f…bdaaeee3` / `ec821c01…899fa9` |

playback 到窄弯中点时捕获：

- `state=PLAYING`，`progress=0.4637708187210357`
  （`clamped_elapsed_s / duration_s`），实际时刻
  `18.36651969 s`，**不是发布采样点**（最近采样差
  `0.006519690000001077 s`，`lower=918`, `upper=919`）；
- 该时刻 `executable=true`、`certification=FEASIBLE`、
  `audit_status=FEASIBLE`、`geometry_only=false`、`marker_count=19`；
- 实时状态三车中心/朝向对 `frame_at` 的误差均为 **0.0**（阈值 `1e-7`）；
- payload sha256（`planning_debug`+`corridors`+`guides`，去掉 marker
  header stamp 后 canonical JSON）
  `3c4ebe3cfdbdf3ba272a0b83f505363c764d8267fbb39db7b8b087a3e7c18b68`；
- markers sha256 `dfd97484…b34c631`；
- 原截图 `images/screenshot_mid_curve.png`，1920×1043，104649 bytes，
  sha256 `61ddfbd47e3524052f1a94e61eedb84368658e877caa1aa16a17178f63fe2ad4`；
  带注释副本 `screenshot_mid_curve_annotated.png`（含 payload hash、
  progress、图层事实），原图保持不变。

捕获到的三层标记：

| 层 | 收到的 MarkerArray |
| --- | --- |
| guides | DELETEALL + A* line 246 点 + A* text + centerline line 3 点 + centerline text |
| corridors | DELETEALL + 2 个 fill TRIANGLE_LIST + 2 个 edge LINE_STRIP（低 alpha） |
| markers | 19 个最终标记：每车 body OBB、heading、1982 点 mode-A 蓝色粗路径、910 点 history、label；links/reference |

## 3. 离线独立验证

`verify_v2_display.py`（不启动 ROS，不信任驱动摘要）重算 payload hash、
用生产 `verify_virtual_a_playback_plan` 重建 plan，并逐层比较坐标：
**68/68 checks 通过**，输出 `verification.json`。

最大误差：

| 比较 | 最大误差 |
| --- | --- |
| A* marker vs 实际 route | `0.0 m` |
| centerline marker vs 实际 guide | `0.0 m` |
| corridor fill / edge vs 实际 regions | `0.0 m` |
| final path vs 解析 plan | `8.881784197001252e-16 m` |
| OBB / live status 位置、朝向 | `0.0` |
| final path MODE_COLORS | `1.1920928966180355e-08` |

最后一项是 ROS `ColorRGBA` float32 量化，不是坐标；坐标阈值仍为
`1e-9 m`。负例也通过：未使用路线落到 candidate namespace/`CANDIDATE`
标签；篡改 `guide_quality.warnings` 后标成 `UNVALIDATED_GUIDE` 且变黄；
错 plan_id 被 `PlaybackValidationError` 拒绝；RViz 默认开关逐项相等。

`verify_audits.py` 另行重建权威 OccupancyGrid（summary 与 SHA-256 必须等于
admission 记录），重建同一 plan，再调用生产 audit 函数，**11/11 checks
通过**，输出 `audit_verification.json`：

| 审核 | 结果 | checks | 备注 |
| --- | --- | --- | --- |
| geometry | PASS | 159 | 绑定同一 request/context/grid（path audit 不带 model 字段） |
| dynamics | PASS | 913 | `collision_checked=false` 是 dynamics scope 的设计值；不单独宣称 executable |
| collision | PASS | 12116 | 8 sweep segments、2014 states、2 static、6042 map + 6042 OBB-pair queries；margin closure gap `2.99e-17 m` |

碰撞审核是**对本次保存 plan 的独立离线复核**，不是 V2 新写的 solver；
V2 没有改 collision/audit 代码。运行时 planner 的
`pipeline_status=SUCCESS`、`status=FEASIBLE` 与 playback
`audit_status=FEASIBLE` 是本次真实运行的状态证据。

## 4. 定向测试与退出码

| 测试 | 数量 | 结果 |
| --- | --- | --- |
| `test_virtual_a_corridor_display.py` | 30 | OK, exit 0 (17.053 s) |
| `test_virtual_a_playback.py` | 34 | OK, exit 0 (53.384 s) |
| `test_virtual_a_ros_adapter.py`（额外，因 V2 触及 ros adapter 文件） | 60 | OK, exit 0 (15.078 s) |
| `test_formation_demo_assets.py`（额外，RViz 资产） | 21 | OK, exit 0 (1.422 s) |

精确命令与退出码见 `commands.log`，原始日志在 `logs/`。

## 5. 过程中发现并修复的真实集成缺陷

首轮 attempt1 中，真实 playback 拒绝了 V2 走廊/guide payload：

```
ignoring corridor payload for plan 92001: identity does not match the current admission
```

原因是 `_record_virtual_job` 在 PLANNING 阶段保存的 job 记录 `model_sha256`
仍为 `None`，而 `corridor_display_matches` 要求它等于 envelope digest；
envelope 发布与 job heartbeat 之间存在时序窗口。修复位于 playback
`on_planning_debug`：**仅当** job 的该字段缺失、且本 receiver 已验证同一个
plan 的 envelope 时，才用已验证 digest 补齐；不覆盖任何已报告 digest，
也不跨 plan 填充。新增正例
`test_missing_job_digest_is_filled_only_from_verified_envelope` 与负例
`test_other_plans_verified_digest_never_fills_job_identity`。修复后真实日志：

```
corridor/guide display plan=92001 generation=1 routes=1 regions=2 state=CANDIDATE_REFERENCE_POINT_ONLY
```

同类单元测试原先伪造了匹配的 job 记录，因此没有暴露该真实 ROS 缺陷；这是
本轮最重要的发现。attempt2/attempt3 随后失败于把
`PlaybackStatus.fraction`（采样点之间的局部插值比例，非曲线进度）当成曲线
进度，已改为 `clamped_elapsed_s / duration_s`；三次失败尝试完整保存在
`attempt1/`、`attempt2/`、`attempt3/`。

## 6. 已知细节、NOT_RUN 与 NOT_APPLICABLE

- **marker 与 status 的 tick 不完全同一时刻**：捕获的
  `/formation/markers` 是最新 10 Hz 发布（其 OBB 对应 status tick
  `18.166410685 s`），捕获的 `playback_status` 是稍后的
  `18.36651969 s`。最终路径数组静态不变；验证脚本用 OBB 中心匹配到实际
  status tick 后比较 `frame_at`（误差 0），并单独检查捕获时刻的实时 status
  （误差 0）。这两个事实都保留在 `verification.json`，没有伪称同一原子快照。
- **Bezier = NOT_APPLICABLE**：代码库没有 Bezier 实现；C2-A guide 是
  PL 折线（3 点），quality 明确 `PL_ONLY_NO_BEZIER`、`smoothness_claim=NONE`。
  因此“Bezier 采样点 vs 解析点”不适用，未伪造该测试。
- **多路线 candidate 层的真实 ROS 展示 = NOT_RUN**：本场景只有一条实际
  使用路线。candidate namespace/默认隐藏/`CANDIDATE` 标签由单元测试和离线
  负例覆盖，没有在真实 ROS 中制造第二条路线。
- **优化失败生命周期 = 未在本 V2 真实 ROS session 重跑**：V1 的单身份过滤与
  CLEARED 生命周期未改；唯一例外是 §5 的缺失 digest 窄修复。V3 负责生命周期
  清理。单元测试覆盖了缺 corridor round、非法 guide、CLEARED 时同时清空
  两层且不残留旧 ADD marker。
- **图层开关不影响 planning/playback/TF = 仅在单元测试验证**（任务要求的
  `test_layer_toggles_do_not_change_planning_playback_or_frames`），没有在
  真实 ROS 中做开关前后计时对比。
- playback 日志在 envelope 刚被接受时有一条瞬时 warning：
  `ignoring audit status for plan 92001: identity differs ...`，发生在
  捕获前约 17 s；捕获时刻的 status 是匹配的 `FEASIBLE`，该 warning 未影响
  捕获数据，原样保留在 `logs/playback.stdout.log`。
- `commands.log` 中 `verify_v2_display.py` 的检查数已按实际结果写为 68；
  草稿曾写 76，已更正，未把草稿数字留在交付物中。

## 7. 停止状态

- 本轮只完成 V2，未进入 V3，未自动推进下一阶段。
- 未创建分支/commit/push；V2 源码与测试仍是未提交改动。
- 未声称 Codex ACCEPTED；等待 Codex/用户复核。
- 私有 ROS session 已按自己的 PID 清理，端口 11585 已释放；用户
  11411 master 全程未触碰。清理记录在 `logs/cleanup.log`。

关键产物：`results.json`、`verification.json`、`audit_verification.json`、
`run_result.json`、`display_messages.jsonl`、`v2_source_changes.patch`、
`v2_display.rviz`、`images/`、`logs/`、`MANIFEST.sha256`。