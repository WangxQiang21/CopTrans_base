# C1 阶段总结：精确 LOS 与二维分离超平面凸走廊

**状态：C1 实现、审核收紧与定向测试完成，等待 Codex 独立验收。本文生成时工作区尚未提交；C1 提交只能在其后产生，因此提交 SHA 见交接消息和最终 `git log`，不能写进本文件自身。不进入 C2，不推送远程，不启动 ROS/RViz/发布进程。**

## 1. 基线与工作区

| 项目 | 值 |
|---|---|
| 阶段 | C1 |
| UTC run | `20261117T073007Z` |
| 基线 commit | `23fd16227e75a68f03f3ac7483555063c8189313` |
| 证据生成时 HEAD | `23fd16227e75a68f03f3ac7483555063c8189313` |
| 分支 | `feature/virtual-rigid-formation-a` |
| 证据生成时工作区 | 未提交：C0-A/C0-B 前置改动 + C1 新增/修改文件 |
| Python / NumPy / SciPy | 3.8.10 / 1.24.4 / 1.10.1 |
| 源码目录 P | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a` |
| C1 提交 | 证据之后执行；SHA 在交接消息中给出 |

`git_state.txt` 如实记录证据生成时的未提交状态：`contracts.py` 以及 C0 前置的 `audit.py`、`integration_bounds.py`、`optimizer.py`、`tests/test_virtual_a_optimizer.py` 为已修改文件，C1 的 `route_processing.py`、`corridor.py`、`tests/test_virtual_a_corridor.py` 和证据目录为未跟踪文件。C1 提交只包含本阶段内容；C0 前置内容在单独的前置提交中，使 C1 提交依赖可复现。

上一轮中间证据 `../20261117T071317Z/` 生成于审核收紧之前，已原样保留并用 `SUPERSEDED.md` 标明，不得作为最终验收证据。本 run 的所有日志、JSON 和哈希都对应最终代码。

环境变量和所有命令原文见 `commands.log`；本次没有启动 ROS master、规划节点、RViz、控制器或发布进程，ROS setup 仅用于导入现有离线依赖。

## 2. 范围与非声明

### 2.1 本阶段实现

- `route_processing.segment_is_free(grid, p0, p1, margin_m=0.0)`：原闭栅格上的精确 segment/AABB 视线测试。
- `route_processing.simplify_route(grid, route)`：先验证原 A* 相邻段，再做最远可视点贪心简化；保留端点、`source`、`grid_sha256`。
- `contracts.ConvexCorridor`、`contracts.CorridorChain`：不可变几何合同和确定性摘要。
- `corridor.build_corridor_chain(grid, guide, *, box_padding_m=2.0)`：为每个 guide seed 构建分离超平面凸走廊，含一次 midpoint split。
- `corridor.audit_corridor_chain(grid, chain)`：不依赖生成超平面的独立审核，含原 occupied cell 的 polygon-cell SAT、精确地图包含、seed、存储半空间与多边形一致性、portal、面积/半径检查。

### 2.2 明确未实现/未声明

- 没有接入 NLP、曲率规划或走廊引导；没有执行 C2。
- 没有修改三车运动学、动态、连续 OBB、时间和求解器配置。
- 没有真实 ROS/RViz 运行或 publish/display 证据。
- 走廊只证明参考点几何自由，`d_safe = 0`；不声称三车 swept OBB 安全，不包含 `FEASIBLE` 字段。
- 不把 `GuideRoute` 或 `CorridorChain` 当成可执行轨迹；两者都带 `is_executable_path: False` 语义。

## 3. 修改摘要

| 文件 | 状态 | 行数/差异 | 内容 |
|---|---|---|---|
| `P/contracts.py` | 修改 | +136 / −1 | 只新增 `ConvexCorridor`、`CorridorChain` 并登记 `__all__`；原有合同字段、校验和语义未改 |
| `P/route_processing.py` | 新增 | 246 行 | 精确 LOS + 贪心简化 |
| `P/corridor.py` | 新增 | 697 行 | 凸走廊构建 + 独立审核（含审核收紧） |
| `tests/test_virtual_a_corridor.py` | 新增 | 504 行 / 25 tests | 7 类小地图、反例、合同、独立 SAT、确定性、50×50 计时 |

C1 源码哈希（详见 `source_hashes.txt`）：

```
852de175c211b33fdef07df006bf7f569aa8053c30a9f1545c18025abe6f62ef  contracts.py
31a959eef95e209cd068813b82fa458e82c838819a7b843e410763cfc5163921  route_processing.py
d816aba3ea694c273acb656ec7d2b11240e450a082f32982dbb836236e41948c  corridor.py
d29a633626ac4bf44dffae2bebd1200a582b0d3bc916dae0041834d75fb8dc4c  tests/test_virtual_a_corridor.py
```

**未修改证据**：`head_equality.txt` 用 `git diff --quiet HEAD -- <file>` 记录以下文件相对基线 HEAD 完全未变：`geometry.py`、`collision.py`、`pipeline.py`、`timing.py`、`trajectory.py`、`bounds.py`、`ros_adapter.py`、`__init__.py`，以及 `tests/test_virtual_a_audit.py`、`test_virtual_a_profile.py`、`test_virtual_a_geometry.py`、`test_virtual_a_timing.py`、`test_virtual_a_trajectory.py`。

C0-A/C0-B 前置文件在 C1 中未改：`audit.py`、`integration_bounds.py`、`optimizer.py`、`tests/test_virtual_a_optimizer.py` 的 SHA-256 与 C0-B 证据 `source_hashes.txt` 一致。C1 没有修改 `optimizer`、`timing`、现有审核器或 ROS 配置。

## 4. 算法与实现

### 4.1 精确 LOS

`segment_is_free` 使用权威 `Grid.occ`，不读取距离场：

1. 端点必须在闭地图内；unknown/地图外按障碍处理。
2. 线段到地图边界的最近距离必须严格大于 `margin_m`；地图边界本身按闭障碍处理。
3. 枚举线段包围盒按 `margin_m` 膨胀后再向外多取一圈的 occupied cell，不做未经证明的距离跳过。
4. 每个 cell 先做闭区间 slab 的 segment/AABB 相交测试；相交即距离 0。
5. 不相交时，最近点对只可能出现在“线段端点到方块四条边”或“方块四角到线段投影”，两种都计算后取最小。
6. 只要距离 `<= margin_m + 1e-12` 就拒绝；零长度线段按点处理。
7. 地图边界最近距离同样用“端点到边界边 + 边界角到线段投影”计算。

穿 cell 角、贴边、margin 严格性和地图外反例都在 `LosTests` 中。

### 4.2 路线简化

`simplify_route`：

- 验证对象是 `GuideRoute`、`grid_sha256` 与权威栅格一致。
- **先验证原 A* 所有相邻段**；任一段被闭 cell 阻挡（包括斜向切角）就抛 `ROUTE_SEGMENT_BLOCKED`，不假设 A* 天然合法。
- 然后从当前点向后找最远仍精确可见的原路线点，跳过中间点。
- 结果保留首尾点、`source`、`grid_sha256`。
- 对角障碍切角的 `ROUTE_SEGMENT_BLOCKED` 反例在 `RouteSimplificationTests`。

### 4.3 凸走廊构建

对每个 seed 线段 `(s, e)`：

- `B = seed bbox 向外扩 box_padding_m（默认 2.0 m）`，再裁到地图内缩 `1e-8 m` 的矩形；裁后为空或面积非正即退化。
- 枚举所有与 `B` 相交或被 B 安全距离膨胀范围触及的原始 occupied cell；索引包围盒故意再向外多取一圈，`_occupied_cells` 不做未证明的距离跳过。
- 对每个 occupied cell 求 seed 与闭 cell 的精确最近点对：
  - `n = (o - s) / dist`，其中 `s` 是 seed 上的最近点，`o` 是 cell 上的最近点；`n` 是单位法向。
  - `h = max(n·seed_start, n·seed_end)`。
  - `l = min(n·cell 四角)`。
  - 要求 `gap = l - h > 0`，否则显式退化。
- 所有障碍的 `gap` 取最小值：`rho = min(0.05 m, min_gap / 4)`。
- 每个障碍的半空间为 `n·p <= beta`，其中
  `beta = (h + rho + l) / 2 - 1e-8`。
- 用 B 的四个边界半空间和全部障碍半空间依次裁剪 B（Sutherland–Hodgman），要求最终至少 3 个顶点、正面积 `> 1e-10 m²`，并把顶点规范为 CCW、字典序最小顶点开头。
- 再检查 seed 两端对所有存储半空间的违约 `<= 1e-9 m`；超过即退化。
- `d_safe = 0`，只处理参考点；不声称车辆外形安全。

单盒手算样例：cell 顶边 `[20,40)×[20,40)` 的顶边 `n=(0,-1)`、`h=-3.0`、`l=-2.0`、`rho=0.05 m`，得到
`beta = (-3.0 + 0.05 - 2.0)/2 - 1e-8 = -2.47500001`。
`sample_chain.json` 中 `region0` 的最小 y 和 `region1` 的最小 x 都是 `2.47500001`；同一平面在 A/b 中有 20 行完全匹配手算值。

### 4.4 Portal

相邻走廊的 portal 是两凸多边形的交集：

- 构建侧在存储半空间 `A,b` 上用 Chebyshev LP
  `max r s.t. A x + ||A_row|| r <= b`，`scipy.optimize.linprog(method="highs")`。
- 范数列使用字面 `||A_row||`（即 `math.hypot(a, b)`），不是硬编码 1。
- 认证半径不直接取 `-result.fun`：先用 LP 返回的圆心 `(x, y)` 代入每条约束，
  取 `(b_i - A_i·x) / ||A_i||` 的最小值作为半径。这样即使 HiGHS 的原始可行性
  容差让目标值略微乐观，审核/构建使用的是由实际 slack 认证的半径。
- 要求：
  - `r >= min(0.005 m, 0.1 * resolution)`；
  - portal 面积 `> 1e-10 m²`；
  - seed 违约 `<= 1e-9 m`。
- 失败时对涉及的问题 seed 至多做一次 midpoint split，绝不递归；仍失败则抛 `CORRIDOR_DEGENERATE`，并明确说明这是走廊构造退化，不是物理无解。
- 审核侧不使用构建侧存储的 portal 结果，而是从两个 `vertices` 重建边半空间、求交集并重新算面积/Chebyshev 半径。

### 4.5 独立审核

`audit_corridor_chain` 不靠生成侧半空间来证明“走廊不含障碍”：

1. 对每个区域，遍历**原始 occupied cell**，范围为多边形 bbox（含 1e-9 m 保守触边容差），逐 cell 做 polygon/AABB SAT；任一轴投影间隙不大于 `1e-9 m` 就按接触/重叠拒绝，记录首个 `{region, cell}`。
2. 另查地图包含、seed 两端在闭多边形内（及存储半空间内）、顶点满足存储半空间、CCW/正面积/凸性、单位法向、A/b 长度。
3. **存储半空间与多边形一致性**：每条多边形边必须落在某条存储半空间边界上（`1e-9 m` 匹配容差），且所有顶点满足全部存储半空间。两者合起来证明
   `{p : A p <= b}` 与审核多边形是同一区域。否则 C2 消费的 `A,b` 可能比 SAT 审核的多边形更松，从而绕开障碍审核。
4. 地图包含使用闭地图的**精确**比较，不给 `1e-9 m` 容差；顶点只要超出地图（哪怕 `5e-10 m`）就拒绝。
5. 检查区域面积、portal 面积/半径、相邻区域 seed 连续性。
6. 结果字段包括 `passed`、`reason`、`detail`、`region_count`、逐区域 metrics、`portals`、`min_region_area_m2`、`max_seed_violation_m`、`min_portal_radius_m`、`min_portal_area_m2`、`first_collision`、`thresholds`；**没有** `feasible`/`FEASIBLE`。

开发中修正的真实问题（均有回归测试）：

- **SAT 分离判据错误**：最初把“多边形最大值大于 cell 最小值”当作分离，几乎任何走廊都会被误判为分离，手工构造的含 `[20,20]` cell 的假走廊也会通过审核。修正为
  `min(polygon) > max(cell) + tol` 或 `min(cell) > max(polygon) + tol` 才分离；现在该负例明确返回 `CORRIDOR_CELL_OVERLAP` 和 `first_collision={"region":0,"cell":[20,20]}`。
- **存储 `A,b` 比多边形松**：假走廊顶点 `[3,4]²`、存储 `A,b` 为 `[0,4]²` 时，旧审核会通过，但 `{p: A p <= b}` 覆盖了 occupied cell `[0,1]²`，C2 会消费这个更松的区域。新增边支撑检查后返回 `CORRIDOR_INVALID`。
- **地图包含容差过松**：旧审核允许 `1e-9 m` 越界；新增 `vertex (-5e-10, 0.5)` 反例，现返回 `CORRIDOR_MAP_BOUNDARY`。
- **Portal LP 目标值可能乐观**：改为字面范数列 + slack 认证半径。

## 5. 结果与关键数值

### 5.1 七类小地图（全部由独立审核通过）

| 类别 | 区域数 | 最小区域面积 m² | 最小 portal 半径 m | 最小 portal 面积 m² | seed 最大违约 m | 结果 |
|---|---:|---:|---:|---:|---:|---|
| 空场 | 3 | 9.999999935 | 1.249999995 | 9.999999935 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |
| 单盒 | 2 | 6.0999998895 | 0.76249999 | 2.325624939 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |
| L 角 | 3 | 3.8749998845 | 0.38749999 | 1.375624949 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |
| 凹 U | 2 | 4.1506030038 | 0.47499999 | 2.161154651 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |
| 窄缝 | 2 | 0.4196619515 | 0.09374999 | 0.0446619915 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |
| 地图边界 | 3 | 8.0999999195 | 1.012499995 | 4.1006249595 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |
| B 内远障碍 | 1 | 11.89999989025 | 无 portal | 无 | 0.0 | `CORRIDOR_CHAIN_VERIFIED` |

所有类别 `first_collision = null`。窄缝区域的 y 范围为 `1.95625001 .. 2.14374999`（在自由通道 `1.9 .. 2.2` 内）。B 内远障碍样例的走廊最大 x 为 `2.97499999 = 2.975 - 1e-8`，证明 padding box 内但不在 seed 附近的障碍没有被跳过。

### 5.2 显式失败/反例

| 场景 | 结果 |
|---|---|
| 线段穿 cell 角 | `segment_is_free = False` |
| 对角 A* 切角 | `ROUTE_SEGMENT_BLOCKED` |
| B 内远障碍 | 被分离平面裁掉，走廊最大 x 比障碍近侧少 `1e-8` |
| seed 触障碍 | 先失败，split 一半仍触障碍 → `CORRIDOR_DEGENERATE` |
| seed 在地图外 | `segment_is_free = False`，build → `CORRIDOR_DEGENERATE` |
| 点接触 portal | `CORRIDOR_DEGENERATE`，`portal 0 is empty or infeasible`，`min_portal_radius_m = null` |
| 假走廊含 occupied cell `[20,20]` | 独立 SAT → `CORRIDOR_CELL_OVERLAP`，首碰 cell 正确 |
| 存储 `A,b` 比多边形松 | `CORRIDOR_INVALID`，`polygon edge is not on a stored halfspace boundary` |
| 顶点越地图 `5e-10 m` | `CORRIDOR_MAP_BOUNDARY` |

### 5.3 确定性与摘要

- 相同输入两次构建：CCW 规范顶点、`CorridorChain.digest`、`as_dict()` 和独立审核 dict 完全一致（`test_empty_field_chain_is_verified_and_deterministic`）。
- 单盒手算样例链摘要：
  `d3e838f164a1a1bd32fa6d859da4ab0c5f302a3d0feccbfe13d84a10f6e6e801`。
- `sample_chain.json` 是给 V1 的最小交接输入：grid（80×80、0.05 m、400 occupied cells）、route、链的 A/b/vertices/digest、完整审核和手算 plane 对照。

### 5.4 耗时

50×50（resolution 0.1 m、8 个 route 点、7 个区域）单次样例：

```
build  0.02323722594883293 s
audit  0.006887699011713266 s
total  0.030124924960546196 s
```

测试日志另一次运行的打印为 `build=0.022110 s, audit=0.006292 s`，属于同一台机器的单样本波动。**这不是性能承诺**；C1 尚未把走廊构建时间纳入请求预算，C2 必须把该耗时计入请求预算并单独测量。

## 6. 测试与回归

| 测试 | 结果 | 耗时 | 说明 |
|---|---|---|---|
| `test_virtual_a_corridor.py` | 25 tests OK | 0.617 s | C1 新增，7 类 + 反例 + 合同 + 独立 SAT + 两个审核收紧负例 + 确定性 |
| `test_virtual_a_pipeline.py` | 8 tests OK | 192.880 s | 现有真实求解、三车几何/动态/连续 OBB 审计与三个冻结场景 |
| `test_virtual_a_optimizer.py` | 29 tests OK | 66.202 s | 现有求解器回归 |
| `test_virtual_a_audit.py` | 64 tests OK | 4.650 s | 现有独立审核回归 |
| `test_virtual_a_collision.py` | 17 tests OK | 2.587 s | 现有连续 OBB 碰撞回归 |

全部使用 `python3 -B -m unittest discover -s tests -p '<单文件模式>' -v`，输出重定向到对应日志；退出码均为 0。**没有运行全量 `unittest discover`**，避免覆盖历史证据输出。

分栏状态：

| 分栏 | C1 状态 |
|---|---|
| 求解成功 | C1 不调用 NLP，走廊引导求解 **NOT_RUN**；现有 pipeline 真实求解回归 8/8 通过，只证明未破坏既有求解，不等于 C1 走廊已接入 solver |
| 几何 | 7 类走廊 + 反例 + LOS 全部通过；参考点几何 |
| 动态 | C1 未改动态代码；现有 pipeline 动态审计回归通过；C1 自身不产生动态结论 |
| 碰撞 | C1 使用独立 polygon-cell SAT；现有连续 OBB 碰撞测试 17/17 通过；未改三车碰撞逻辑 |
| 发布/显示 | **NOT_RUN**，不启动 ROS/RViz，不声称发布成功 |

## 7. 验收对照（TASK_C1）

| 要求 | 证据 |
|---|---|
| `segment_is_free` 精确 LOS、unknown/地图外障碍、触边拒绝、零段按点、距离场不作认证 | `route_processing.py`；`LosTests` 3 tests；`segment_is_free` 只读 `Grid.occ` |
| `simplify_route` 最远可视点、先验原 A* 相邻段、切角拒绝、保留端点/source/grid | `route_processing.py`；`RouteSimplificationTests` 3 tests |
| 两个合同 + `build_corridor_chain` + `audit_corridor_chain` | `contracts.py`、`corridor.py`；`ContractTests` 2 tests；`CorridorCategoryTests` 8 tests；`AuditTests` 7 tests |
| B 扩 2 m、地图内缩 1e-8 m、全 occupied cell 覆盖、精确最近点对、`h/l/gap/rho/beta` | `corridor._build_region`；单盒手算测试；B 内远障碍反例 |
| 走廊含 seed、正面积、portal LP、r/area/seed 阈值、退化显式失败、一次 split | `corridor.build_corridor_chain`；点接触 portal、seed 触障碍、地图外反例 |
| 独立 polygon-cell SAT、地图/seed/portal/面积/首碰 cell、无 FEASIBLE | `corridor.audit_corridor_chain`；假走廊 `[20,20]` 反例 |
| 存储 `A,b` 必须与审核多边形一致（C2 消费 `A,b`） | 边支撑 + 顶点检查；`test_audit_rejects_halfspaces_looser_than_polygon` |
| 地图包含无容差放宽 | 精确闭地图比较；`test_audit_rejects_vertex_just_outside_map` |
| 7 类小 grid、100% 独立审核通过、确定性、50×50 计时 | `measurements.json`、`results.json`、`test_virtual_a_corridor.log` |
| 保存 grid/route/A,b/vertices/portal/SAT JSON 供 V1 | `sample_chain.json`（90 KB） |

## 8. 证据文件

| 文件 | 用途 |
|---|---|
| `SUMMARY.md` | 本文件 |
| `results.json` | 机器可读的阶段结论、测试计数、范围、NOT_RUN、声明 |
| `measurements.json` | 七类结果、显式拒绝、环境、50×50 计时 |
| `sample_chain.json` | V1 消费的 grid/route/A,b/vertices/portal/审核 JSON |
| `commands.log` | 精确命令、环境、退出码、说明 |
| `git_state.txt` | 证据生成时 HEAD/branch/status/diff stat（如实未提交） |
| `head_equality.txt` | 受保护文件相对 HEAD 未变；C0/C1 哈希 |
| `source_hashes.txt` | 源码、测试、计划文档、脚本、日志、JSON 的 SHA-256 |
| `source_diff_c1_contracts.diff` | C1 对 `contracts.py` 的增量 |
| `source_diff_c0_context.diff` | C0-A/C0-B 既有改动（非 C1） |
| `new_route_processing.diff` | 新文件全文 diff |
| `new_corridor.diff` | 新文件全文 diff |
| `new_test_diff.diff` | 新测试全文 diff |
| `test_virtual_a_corridor.log` 等 5 个 | 定向测试日志 |
| `collect_c1_evidence.py/.log` | 生成 sample/measurements 的证据脚本与输出 |
| `assemble_results.py/.log` | 生成 results/git_state 的脚本与输出 |
| `../20261117T071317Z/SUPERSEDED.md` | 标明上一轮中间证据已被本 run 取代 |

## 9. 失败、NOT_RUN 与遗留问题

1. **C2/NLP 未开始**：没有走廊引导 NLP、曲率坐标、软硬模式或请求预算接入；**NOT_RUN**。
2. **真实 ROS/RViz/发布未执行**：没有 ROS master、节点、topic、显示或 publish 证据；**NOT_RUN**。本次仅用 ROS overlay 的 Python 环境跑离线测试。
3. **全量 `unittest discover` 未执行**：避免覆盖历史证据；只跑定向模式。**NOT_RUN**。
4. **三车安全未由本阶段证明**：`d_safe=0`，走廊不含车辆外形/扫掠 OBB；三车连续 OBB 和独立安全审核仍由既有、未修改的最终审计负责。C1 不产生 `FEASIBLE`。
5. **精确地图边界点保守拒绝**：`segment_is_free` 把地图边界当闭障碍，route 点恰好落在地图边界（或 build 时被 1e-8 内缩裁掉）会显式失败（LOS false / `CORRIDOR_DEGENERATE`）。边界类测试使用 0.025 m 内侧点验证 B 内缩；若 C2 的真实请求允许边界起点/终点，必须由 C2 单独定义并验证，不能在 C1 里静默放宽。
6. **首版保守参数**：`box_padding_m=2.0`、`rho_max=0.05`、`d_safe=0` 是首版几何选择，未做最优性声称；复杂狭缝可能因一次 split 仍失败，返回 `CORRIDOR_DEGENERATE`，这只说明构造器退化，不代表物理解不存在。
7. **Portal LP 数值**：认证半径由 LP 圆心处的实际 slack 重新计算，范数列为 `||A_row||`；这消除了“信任 HiGHS 目标值”的乐观误差。审核侧仍从多边形顶点独立重建 portal，不直接信任构建侧 portal 数值。构建/审核都要求 LP `success`。
8. **审核收紧后的额外语义**：存储 `A,b` 现在必须与多边形同区域（`1e-9 m` 边匹配容差）；这会让任何多出冗余但更松的半空间集合被拒绝，即使多边形本身无碰撞。这是为 C2 消费 `A,b` 而定的必要约束，不是过防御。
9. **50×50 耗时为单样本**，尚未进入请求预算；C2 需要把 build+audit 时间纳入预算并重新测量。
10. **历史 `timing_continuous` 的 `KeyError: dynamic_peaks` 仍未修复**：C1 未触碰 timing，历史失败保持原状，不能作为 C1 回归通过的反证。
11. **C0 前置与 C1 的提交分开**：证据生成时全部未提交；C0-A/C0-B 前置文件先单独提交，C1 文件和本证据随后提交。C1 提交 SHA 在交接消息给出。

## 10. 停止点

C1 几何门完成。**在此停止，等待 Codex 独立验收；不自动进入 C2，不推送远程，不启动 ROS 实验。**