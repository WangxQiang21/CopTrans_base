# 人工审核分析：为什么候选路径看起来在走廊外，以及为什么该次规划被拒绝

日期：2026-11-19。分析对象是 Step 6 已完成会话
`../07_ros_rviz/s1_main_free/`；本文件只解释已有证据，**没有修改**
planner、optimizer、corridor/audit 数学或任何阈值，也没有改生产代码。
本文件不宣称 Codex PASS 或 TASK 01 验收。

## 0. 结论（先看这四条）

1. 粉色的 `Convex workspace corridors (F point only)` 是**队形参考点 F
   的工作空间**，不是三辆车车体或车辆中心的 containment 区域。图层名、
   走廊 payload 的 `state=CANDIDATE_REFERENCE_POINT_ONLY` 都明确这一点。
2. 截图里跑到走廊外的蓝色/黄色线主要是 `Planning candidates` 图层中的
   **车辆中心候选路径和 OBB 候选轮廓**（诊断预览），不是最终认证路径。
   这些候选的中心按固定队形偏移 `(1,0),(0,1),(0,-1)` 生成，天然会落在
   F 走廊之外；这不表示最终发布路径在走廊外。
3. 该次 F 参考路径本身基本在走廊内：205 个采样点中只有 2 个在走廊外，
   最大外伸 **0.0005538 m**（0.554 mm）；连续审计也在路径模型上采到了
   正残差。真正拒绝的原因是**连续 containment 的保守宽度下限**：
   `CORRIDOR_AUDIT_WIDTH_LIMIT`，失败点仍在走廊内 4.69 µm，但
   `0.5×区间宽度 + epsilon` 已大于这点余量，审计无法证明整个小区间都在
   走廊内，于是 fail-closed。
4. 这不是“规划器故意跑到走廊外”。NLP 只在每个 allocation 段的左端、
   9 个 Gauss 点和右端（共 11 个 station）约束走廊 halfspace；全部
   4642 条 halfspace 约束的残差只有 `9.9858e-09 m`。采样 station 之间
   的连续路径仍可以有亚毫米级凸出，连续审计因此拒绝。该差异是
   **离散约束与连续认证之间的间隙**，属于保守认证行为，不是导航错误。

## 1. 证据来源与本次会话的身份

| 项 | 值 | 来源 |
|---|---|---|
| plan_id | 600000 | 两份 YAML |
| request_sha256 | `591b4100355ab991d3b7dd5afeb95e203343ec185bf7187d025e2662b4afdd3b` | quality report |
| planning_context_sha256 | `8a545c3f4c794f9f04c57f453c454612c54ca2d3fe31e4c6b887037d2d36604b` | quality report |
| grid_sha256 | `6e94c751a40ec94431b495fcbff002945f09c975429c493c89060f0e7e2aa2a4` | quality report |
| 最终状态 | `status=REJECTED`, `pipeline_status=GEOMETRY_FAILED`, `reason=CORRIDOR_AUDIT_WIDTH_LIMIT`, `executable=false` | quality report |
| 失败层 | `first_failure_layer=GEOMETRY` | quality report |
| 候选分类 | `classification=UNVERIFIED_CANDIDATE`, `status=GEOMETRY_FAILED`, `diagnostic_label=DIAGNOSTIC_ONLY`, `certified=false` | planning debug |
| 走廊状态 | `state=CANDIDATE_REFERENCE_POINT_ONLY`, `corridor_used=true`, `guide_label=PL_GUIDE` | planning debug |
| 发布 | `publication=null`, `trajectory_published=false`, `playback_confirmed=false` | quality report |

输入文件：
`../07_ros_rviz/s1_main_free/live_formation_planning_debug.yaml`（最终
`FAILED` 文档，205 个 F 采样点、每车 205 个中心点、80 个 OBB 轮廓）与
`../07_ros_rviz/s1_main_free/live_formation_quality_report.yaml`
（`virtual_result.search.corridor.rounds[0]` 的 allocation、containment、
attempt 和 planner config）。本文件所有数值都取自这两个文件；没有使用
截图估读值。

需要特别说明：人工审核附图中同时出现了 `COLLISION MAP_OBB_COLLISION`
和 `INVALID ... PREVIEW ONLY` 标签，而 `s1_main_free/` 的 plan 600000
失败原因是 `CORRIDOR_AUDIT_WIDTH_LIMIT`，不是碰撞。这些标签来自其它
失败计划留在 RViz 中的旧图层（即第 3 项报告的“2D Pose Estimate 后旧
marker 不消失”）。因此截图不能按图层逐条归因到同一个 plan；下面的
数值分析只使用 plan 600000 的两份 YAML。第 3 项的清除修复会在手动
重置时移除这些旧层，避免后续截图再混叠。

## 2. 三个图层不是同一个几何对象

| RViz 图层 | 内容 | 是否认证 | 几何约束对象 |
|---|---|---|---|
| `Convex workspace corridors (F point only)` | 2 个凸走廊多边形（`region 0/1`） | 否，显示层 | 仅 F 参考点 |
| `Planning candidates (unverified until PUBLISHED)` | 黄色 F 候选、蓝色车辆中心候选、蓝色 OBB 候选 | 否，`UNVERIFIED_CANDIDATE` / `DIAGNOSTIC_ONLY` | 求解器诊断预览 |
| `Vehicle bodies, headings, history and labels (markers)` | 当前帧 OBB、箭头、已行驶历史、标签 | 仅在有独立 FEASIBLE 状态时有意义 | 当前显示帧 |
| `Certified path vehicle 1/2/3 (editable)` | 已验证信封的完整中心路径（`nav_msgs/Path`） | 是，仅 `PUBLISHED` 后可执行 | 各车辆中心 |

本次会话没有 `PUBLISHED`，因此没有任何 `Certified path` 可显示；地图上
看到的蓝/黄线全部来自第二行，是**失败计划的诊断候选**。候选 payload
的 `source` 为 `plan 600000 virtual_rigid_a`，来自
`_progress_diagnostic_path()` 返回的实际 solver 候选，而不是事后插值的
假路径。

`A* raw route and Portal guide` 图层也不是认证轨迹：A* raw route 是搜索
原始折线，本身不承诺在 corridor 内；corridor 是围绕
`corridor_qp_polyline` 生成的 Portal guide 构建的。把 raw route、guide、
corridor 和 solver candidate 当成同一条“规划路径”会误判越界来源。

> 图层名称与消息类型在本次审核的第 1 项修改后有所变化：候选 F/中心路径、
> A* raw route 和 Portal guide 改为独立的 `nav_msgs/Path` 话题，candidate
> OBB 与文本标签仍在 `/formation/planning_candidates`，guide 的 marker 层
> 只剩标签；最终路径话题不变。本文件的几何归因与“越界来自 F-only corridor
> 加候选/OBB 偏移”的结论不受显示类型修改影响。

走廊 payload 的 `state` 是 `CANDIDATE_REFERENCE_POINT_ONLY`，与图层名
`F point only` 相互印证。走廊是围绕 F 的引导折线生成的候选凸区域，不是
三车 OBB 的安全区域。

## 3. 数值：F 路径基本在走廊内，车辆中心/OBB 本来就在走廊外

按弧长把每个候选采样映射到 `allocation` 的对应 corridor，再计算该
region 的所有 halfspace `A·p - b` 的最大值（正数表示在对应走廊外，单位
为米）。同时给出“到两个走廊多边形并集”的残差，它更接近 RViz 里的
视觉判断（在任一粉色多边形内就算没出界）。

| 对象 | 采样数 | 并集最大外伸 | 并集外点数 | 对应走廊最大外伸 | 对应走廊外点数 |
|---|---:|---:|---:|---:|---:|
| F 参考路径 | 205 | 0.0005538 m | 2 | 0.0005538 m | 2 |
| car1 中心 | 205 | 0.1387 m | 18 | 0.4116 m | 36 |
| car2 中心 | 205 | 0.9441 m | 205 | 0.9441 m | 205 |
| car3 中心 | 205 | 0.9374 m | 205 | 1.0006 m | 205 |
| car1 OBB 角点 | 400 | 0.3661 m | 71 | 0.7865 m | 98 |
| car2 OBB 角点 | 400 | 1.3007 m | 356 | 1.3007 m | 362 |
| car3 OBB 角点 | 400 | 1.1416 m | 375 | 1.2691 m | 377 |

F 路径的两个外点出现在索引 156、157：

```text
index 156  p=(7.475482838409877, 1.7068018238218357)  residual=+0.0001238957 m
index 157  p=(7.522672479611136, 1.72261470500096)   residual=+0.0005538389 m
```

这只是亚毫米级凸出，而且发生在两个采样点之间（索引 158 又回到走廊内，
残差 `-9.125e-05 m`）。它说明 F 路径“基本在走廊内”而不是“逐点严格
在走廊内”；连续审计正是为了捕捉这种采样点之间/连续模型上的偏差。

车辆中心/OBB 的数值不能用来判断走廊约束是否满足：车辆中心在 F 基础上
有固定偏移 `(1,0),(0,1),(0,-1)`，而走廊只约束 F。例如起点处
F=(0,0)，car2=(0,1)、car3=(0,-1)，仅偏移就已经超出 region 0 在原点附近
的上下边界（约 `-0.775 m` 到 `+0.0014 m`）。这些蓝色轮廓在地图上“出
走廊”是预期显示，不是坐标系错误，也不是规划器把车开出了走廊。

## 4. 为什么连续审计拒绝：`CORRIDOR_AUDIT_WIDTH_LIMIT`

### 4.1 NLP 只约束 Gauss 点

本次 `planner_config`：

```text
corridor_enforcement = hard
corridor_quadrature_points = 9
corridor_margin_m = 1e-05
corridor_clearance_target_m = 0.05
corridor_audit_position_margin_m = 5e-08
corridor_audit_min_width_m = 1e-05
corridor_audit_max_depth = 20
corridor_audit_max_intervals = 20000
```

attempt 0 的结果：

```text
status = SOLVER_SUCCESS_AUDITED
solver = ipopt
iterations = 31
constraint_residual = 9.985823057334642e-09
corridor_residual_m = 9.985823057334642e-09
```

走廊 0 的 43 个 halfspace 和走廊 1 的 78 个 halfspace 在每个 station
上重复施加：8 个走廊 0 allocation 段 × 11 station × 43 = 3784，
1 个走廊 1 allocation 段 × 11 station × 78 = 858，合计 4642 条约束，
与 quality report 的 `corridor_halfspace_constraint_count=4642` 一致。
也就是说，**在优化器实际采样的 station（左端、9 个 Gauss 点、右端）**
上，走廊 halfspace 约束满足到约 `1e-8 m`。优化器没有“选择”走廊外的
解；它只能保证这些离散采样点。C2 分段多项式在相邻 station 之间仍可能
向外凸出，尤其当路径贴着边界、曲率较大时。

### 4.2 连续审计的判据

`audit_path_in_corridors()` 对每个弧长区间取中点，计算
`point_residual = max_i(A_i·p - b_i)`，并要求：

```text
bound_residual = point_residual + 0.5 * width + epsilon + guard <= 0
```

其中 `width` 是该弧长区间长度，`epsilon = position_budget_m +
path_position_error_bound_m`，`guard = 1e-12`。审计自适应二分；当
`width <= min_width_m` 时：

- 若 `point_residual > 0`，报 `CORRIDOR_CONTAINMENT_VIOLATION`；
- 否则报 `CORRIDOR_AUDIT_WIDTH_LIMIT`（路径可能仍在内部，但区间宽度
  下限已无法继续缩小以证明整个区间都在内部）。

本次 containment：

```text
passed = false
reason = CORRIDOR_AUDIT_WIDTH_LIMIT
epsilon_total_m = 5.6406434245933675e-08
position_budget_m = 5e-08
path_position_error_bound_m = 6.40643424593368e-09
min_width_m = 1e-05
max_depth_reached = 17
sample_count = 453

first_failure:
  s_m = 7.757789761470737
  corridor_index = 0
  width_m = 9.682634578567217e-06
  depth = 17
  point_residual_m = -4.692437438724717e-06   # 仍在走廊内
  bound_residual_m = +2.0528728480482543e-07 # 但界无法闭合
```

算术核对：

```text
-4.692437438724717e-06
+ 0.5 * 9.682634578567217e-06   (+4.841317289283609e-06)
+ 5.6406434245933675e-08        (+5.6406434245933675e-08)
+ 1e-12
= +2.0528728480482543e-07
```

失败点的路径仍在 corridor 内 4.69 µm；但半区间宽 4.84 µm 加 epsilon
5.64e-8 m 已经超过这点余量，因此无法用该区间宽度证明 containment。
审计继续二分会低于 `min_width_m=1e-5`；在宽度下限处按自身规则返回
`CORRIDOR_AUDIT_WIDTH_LIMIT`。这是**保守、fail-closed 的认证结果**：
它没有说“路径在走廊外”，而是说“在给定 epsilon 和宽度下限下，无法
证明这一段完全在走廊内”。

同一轮 containment 的其它字段：

```text
max_point_residual_m = +2.850879913934845e-04 m
max_bound_residual_m = +0.5905181591590959 m
max_depth_reached = 17
base_interval_count = 9
```

`max_point_residual_m` 为正，说明连续审计已经采到过路径在走廊外的点
（+0.285 mm 量级）；`max_bound_residual_m` 更大是因为它包含宽区间的
`0.5 * width`，不能把它读成路径外伸 0.59 m。审计在深度优先遍历中遇到
第一个宽度下限失败就停止，因此 `max_point_residual_m` 是“已检查到的
最大值”，不是全路径最大违反量。结合第 3 节的采样路径最大外伸
0.554 mm，可以判断：**路径在亚毫米尺度上确实越过了 halfspace，连续
审计拒绝认证是正确的；报告的首个失败标签是宽度下限，因为失败中点本身
仍在内部。**

### 4.3 为什么不是 `CORRIDOR_CONTAINMENT_VIOLATION`

失败中点 `point_residual_m=-4.69e-6`，是负数。按上述判据，宽度降到
`9.68e-6 m <= min_width_m` 时，`point_residual <= 0`，所以分类为
`CORRIDOR_AUDIT_WIDTH_LIMIT` 而不是 containment violation。若该中点
当时已经在走廊外，才会报 `CORRIDOR_CONTAINMENT_VIOLATION`。这解释了
HUD/quality report 的 reason 字面含义，也说明拒绝的门槛是“无法证明”，
不是“已证明越界”。

## 5. 为什么没有通过 fixed-sigma reallocation 恢复

`virtual_result.search.reallocation`：

```text
attempted = true
available = false
reason = fixed_sigma_reallocation_unavailable
detail = no eligible adjacent allocation segment with a non-degenerate
         local overlap; the fixed-sigma failure is not at a switch
failure_sigma = 0.7640900611877441
failure_s_m = 7.757789761470737
failure_corridor_index = 0
old_boundary_sigma = null
new_allocation = null
```

`allocation` 的 9 段中，走廊 0 占 `[0, 0.9213301232564699]`，走廊 1 占
`[0.9213301232564699, 1.0]`；切换点在 sigma 0.9213。失败 sigma 0.7641
位于走廊 0 的内部段，不在切换点附近。`reallocate_adjacent_portal_once()`
只允许在切换点两侧、且有非退化局部重叠时做一次 fixed-sigma 调整；这次
不符合条件，因此没有候选。这是既定 guard 的行为，不是异常分支，也不
应为了通过而放宽。该次会话随后以 `GEOMETRY_FAILED` 结束，没有发布任何
可执行信封。

## 6. 对“规划应该在走廊内进行”的准确表述

- 对**F 参考点**：设计意图确实是 F 路径在 F 走廊内，而且该次候选路径
  在视觉上和数值上都基本如此（205 点中 2 个亚毫米级外点）。但“基本
  在内部”不等于“连续认证通过”；硬约束只在 Gauss 点采样，连续审计
  使用 `epsilon + 0.5*width` 的保守界，因此边界附近的小凸出会导致
  fail-closed。
- 对**车辆中心/OBB**：当前走廊图层明确只覆盖 F，车辆中心/OBB 不在
  该走廊的约束对象内。蓝色候选中心/OBB 在走廊外是队形偏移和诊断显示
  的正常结果；不能用它们来判断 F 走廊约束是否被满足。
- 对**最终认证路径**：本次没有 `PUBLISHED`，也没有可执行的
  `nav_msgs/Path` 可看。只有在 planner 完成几何/动力学/碰撞审计并发布
  原子信封后，`/formation/path/vehicle_*` 才会出现；失败时该话题为空。

## 7. 复现与核对方法

两份输入都是 ROS 采样保存的 YAML（`rostopic echo -n1` 包装成
`data:` 字符串）。核对步骤：

```python
import bisect, json, math, yaml

base = "work_flow/TASK01_evidence/07_ros_rviz/s1_main_free/"
debug = json.loads(next(yaml.safe_load_all(
    open(base + "live_formation_planning_debug.yaml")))["data"])
quality = json.loads(list(yaml.safe_load_all(
    open(base + "live_formation_quality_report.yaml")))[0]["data"])
round0 = quality["virtual_result"]["search"]["corridor"]["rounds"][0]

regions = round0["chain"]["regions"]
allocation = round0["allocation"]
f_points = debug["candidates"]["f_points"]

def residual(point, region):
    return max(row[0] * point[0] + row[1] * point[1] - limit
               for row, limit in zip(region["A"], region["b"]))

def union_residual(point):
    return min(residual(point, region) for region in regions)

cumulative = [0.0]
for first, second in zip(f_points, f_points[1:]):
    cumulative.append(cumulative[-1] + math.hypot(
        second[0] - first[0], second[1] - first[1]))
length = cumulative[-1]
starts = [item[0] for item in allocation]

def allocated_region(index):
    sigma = cumulative[index] / length
    segment = allocation[min(
        len(allocation) - 1,
        max(0, bisect.bisect_right(starts, sigma) - 1))]
    return segment[2]

print(max(union_residual(point) for point in f_points))
print([(index, union_residual(f_points[index]))
       for index in (156, 157)])
for vehicle in debug["candidates"]["vehicles"]:
    print(vehicle["vehicle_id"], max(
        union_residual(point) for point in vehicle["center_points"]))
```

该片段只读取证据，不修改任何生产文件；F 路径并集最大外伸
`0.000553838880274049`，外点索引 156/157，与第 3 节一致。车辆中心和
OBB 的“对应走廊”数值需要用 `allocated_region()` 的映射；截图的视觉
判断对应 `union_residual()`。

## 8. 边界与未做的事

- 本文件只分析 `s1_main_free/` 这一次观测，不外推到所有地图/所有
  起点；其它会话（例如 `map05_full_4/` 的 `CORRIDOR_CONTAINMENT_VERIFIED`）
  有不同的 corridor/路径几何。
- 该失败不是 Step 1–5 修改引入的：Step 6 已用 `git archive` 导出的 HEAD
  复现树在同一输入上得到逐位一致的 `CORRIDOR_AUDIT_WIDTH_LIMIT`（见
  交付文档 §8.3 与 `07_ros_rviz/diagnostics/README.md`）。本文件只解释
  这个既有认证边界的机制。
- 没有为了“让路径通过”而修改 `corridor_margin_m`、
  `corridor_audit_min_width_m`、`corridor_audit_position_margin_m`、
  `corridor_quadrature_points` 或 audit 逻辑；没有删除或弱化任何检查。
- 没有修改 `00_baseline/`…`05_numerics/` 或 `evaluation/`；没有
  commit/push；没有进入 TASK 02；没有宣称 Codex PASS 或 TASK 01 验收。
- 若后续需要让该类边界路径通过，只能走经批准的方案（例如增加
  Gauss 点密度、对连续段加 interval-safe 约束、或在明确的容差设计下
  调整 margin/audit 配置），并重新跑完整的几何/动力学/碰撞审计。本
  文件不建议、也没有实施任何此类改动。