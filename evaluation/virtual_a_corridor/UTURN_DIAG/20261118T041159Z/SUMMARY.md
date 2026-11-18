# 回头弯 600003 成功 / 600004 失败的复现与根因分析

- 日期：2026-11-18
- 现场会话：`/tmp/expv3-test-ros.ASVoGG`，私有 master `127.0.0.1:11590`，运行时段 11:58–12:07
- 现场地图：`map05-p12-parking-row-env2`，编队尺度 2.625（489×326，分辨率 0.064871625 m，origin `[-8.3609838125, -5.573955875]`）
- 对应关系（按你的描述：第二张图能规划、第一张不能）：**第二张图 = plan 600003（成功）**，**第一张图 = plan 600004（失败）**
- 结论一句话：两处几何确实是同一个回头弯、同一条走廊同伦；但 600004 的两段长直线把路线从 18.6 m 拉到 45.7 m，依次撞上了**端点航向分支**、**S 搜索域**、**认证积分长度上限**三层独立的硬门。任何一层单独修掉都不够。

本文所有数字都来自离线复现和隔离实验，证据文件见第 10 节。生产代码一行未改。

---

## 0. 结论（TL;DR）

| # | 阻塞 | 机制 | 600003 | 600004 |
|---|------|------|--------|--------|
| A | 端点航向分支与走廊同伦不一致 | `_wrap(goal.psi - start.psi)` 选最短弧，但走廊路线的绕行方向是另一支 | 目标 −3.126 rad，路线 −2.286 rad，同支，差 0.58 rad | 目标 **+3.124 rad**，路线 **−2.951 rad**，异支，差 **6.107 rad ≈ 350°** |
| B | S 搜索域装不下走廊路线长度 | `S_upper = min(60, max(12, 3·d + 10))` 只用起终点直线距离 d | d=10.019 → 上界 40.056 > 路线 18.626，能装下 | d=9.530 → 上界 **38.589 < 路线 45.744**，装不下 |
| C | 认证积分的长度天花板 | 初始 Simpson 面板数 ≈ S/0.02，guard ≈ 2.66e-9·S，S 超过约 **36.8 m** 后 guard 单项就超过 `POSITION_BUDGET_M=1e-7`，且细化只会让 guard 更大 | 实际路径 21.702 m → guard 5.947e-8，认证通过 | 唯一可行的走廊解 48.436 m → guard 1.31e-7，认证必然失败 |

三层的依赖关系：

1. 只修 A（把 600004 的目标分支改到 −3.159 rad）：NLP 仍不可行（S 被 38.589 卡死，端点 x 差 3.43 m，走廊在 region-1 入口超界 3.08 m）。
2. 修 A + B（分支对齐、`S_upper=50`）：IPOPT 24 次迭代收敛到 S=48.436 m，端点误差 2.8e-13、走廊残差 −1.5e-9，**但候选构造被认证层拒绝**：`BOUND_UNCONFIRMED`。
3. 修 A + B + C 才可能产出可执行解；C 属于数值契约层改动，本次未做。

---

## 1. 复现方法与保真度

### 1.1 现场事实（planner 日志原文）

日志：`/tmp/expv3-test-ros.ASVoGG/log/3f276a5e-.../formation_uniform_planner-2.log`

```text
49: 12:02:52,708 plan 600003 admitted ... start=(10.841, 12.071, 0.000) goal=(10.153, 2.076, -3.126)
                              initial=rviz_reset/virtual-a-reset-600002-6 policy=FREE
50: 12:03:13,608 plan 600003 published: T_end=63.656 s samples=3184 wall=17.828 s
54: 12:04:17,191 plan 600003 HOLD_END; busy released
57: 12:04:22,641 plan 600004 admitted ... start=(-3.625, 11.771, -0.022) goal=(-2.490, 2.309, 3.102)
                              initial=rviz_reset/virtual-a-reset-600003-8 policy=FREE
58: 12:05:03,413 [ERROR] virtual plan 600004 not executable:
                              status=BUDGET_EXHAUSTED reason=SOFT_BUDGET_EXHAUSTED layer=GEOMETRY wall=40.768s
59: 12:05:03,423 [WARNING] plan 600004 has no executable envelope (SOFT_BUDGET_EXHAUSTED)
```

### 1.2 离线复现结果

复现脚本用生产同一套函数重建地图与请求：

```text
load_map -> validate_formation_request -> request_from_snapshot -> solve
```

地图由原始 `map05-p12-parking-row-env2` 按尺度 2.625 重建，起终点位姿取自日志打印值（3 位小数）。

| 指标 | 600003 现场 | 600003 离线 | 600004 现场 | 600004 离线 |
|------|-------------|-------------|-------------|-------------|
| 状态 | published | **SUCCESS** | BUDGET_EXHAUSTED | **BUDGET_EXHAUSTED** |
| reason / layer | — | — | SOFT_BUDGET_EXHAUSTED / GEOMETRY | **SOFT_BUDGET_EXHAUSTED / GEOMETRY** |
| wall | 17.828 s | 16.988 s | 40.768 s | 40.811 s |
| T_end | 63.656 s | 63.647 s | — | — |
| 路径长度 | — | 21.702 m | — | — |
| attempts | — | 1 次（41 it，7.41 s） | — | 3 次（68/120/45 it） |

状态、失败层、耗时都对得上（wall 差 0.84 s / 0.043 s），可以认为复现保真。

600004 的三次尝试：

| attempt | 初值 | 求解器状态 | 迭代 | 耗时 | 残差 |
|---------|------|-----------|------|------|------|
| 0 | `route_guide=0;source=corridor_qp_polyline` | `Infeasible_Problem_Detected` | 68 | 10.02 s | 5.138 |
| 1 | `length_multiplier=1.2;shape=left` | `Maximum_Iterations_Exceeded` | 120 | 14.58 s | 26.558 |
| 2 | `length_multiplier=1.2;shape=right` | `Maximum_WallTime_Exceeded` | 45 | 7.52 s | — |

三次尝试合计 32.1 s，加上路线/走廊构建，超过 GEOMETRY 层 30 s 的软预算（`pipeline.py` 中 `replace(planner_config, max_seconds=min(max_seconds, remaining()))`），所以对外表现为 `SOFT_BUDGET_EXHAUSTED`。但这只是表层症状，下面是 A/B/C 三层。

---

## 2. 几何对比：确实是同一个回头弯

| 量 | 600003 | 600004 |
|----|---------|---------|
| start (x, y, ψ) | (10.841, 12.071, 0.000) | (−3.625, 11.771, −0.022) |
| goal (x, y, ψ) | (10.153, 2.076, −3.126) | (−2.490, 2.309, 3.102) |
| 起终点直线距离 d | 10.0187 m | 9.5298 m |
| A* 原始路线 | 22.5685 m | 49.3651 m |
| 简化路线 | 18.9227 m | 46.0010 m |
| 走廊 guide | 18.6258 m | 45.7444 m |
| 走廊种子长度 | [8.2037, 6.0103, 4.7087] | [21.8207, 6.5857, 17.5945] |
| 三个区域段长 | [8.2009, 4.8934, 5.5315] | [21.7914, 6.2064, 17.7467] |
| 区域面积 | [44.829, 32.832, 29.280] m² | [88.281, 34.495, 72.073] m² |
| 入口 portal 半径 | [0.8321, 1.1609] m | [0.8076, 1.1542] m |
| guide 折线累计转角 | −2.2863 rad（−131.0°） | −2.9514 rad（−169.1°） |
| 走廊审计 | 通过 | 通过 |
| allocation（区域 1 区间） | 0.4403–0.7030 | 0.4764–0.6120 |

两者都从同一堵墙的上方出发、从墙的**右端**绕过去：先向东、再向南、再向西，同一顺时针（CW）同伦。U 形弯本身几乎一样，真正的差别只有两段直线：600003 的直段约 8.20 + 5.53 m，600004 是 **21.79 + 17.75 m**，路线总长 2.46 倍。"只是直线部分长了一些"在物理上完全准确。

---

## 3. 根因 A：端点航向分支与走廊同伦不一致

### 3.1 代码事实

`optimizer.py:147`：

```python
heading = _wrap(goal.psi - start.psi)
```

`_wrap` 是 `atan2(sin, cos)`（`contracts.py:90`），始终取最短弧。但 NLP 的端点等式（`optimizer.py:1306-1309`）要求**展开后的航向角**精确等于这个值：

```python
x - packet["dx"], y - packet["dy"], psi - packet["heading_change_rad"]
```

也就是说，端点朝向虽然物理上只约束到 mod 2π，数值上却绑死在一个具体分支上。走廊路线由 A* 同伦固定，它的累计转角可能落在另一个分支上。

### 3.2 数值

| 量 | 600003 | 600004 |
|----|---------|---------|
| 生产目标 `_wrap(goal−start)` | −3.1260 rad（−179.1°） | **+3.1240 rad（+179.0°）** |
| guide 累计转角 | −2.2863 rad（−131.0°） | **−2.9514 rad（−169.1°）** |
| `k = round((turn − target)/2π)` | 0 | −1 |
| 与走廊同分支的目标 `target + 2πk` | −3.1260 | **−3.1592 rad** |
| 生产 route 初值积分到端点的 ψ | −2.5431 | −2.9827 |
| ψ_end − 生产目标 | **+0.5829 rad（+33.4°）** | **−6.1067 rad（≈−350°）** |
| ψ_end − 同分支目标 | +0.5829 rad | **+0.1765 rad（+10.1°）** |

600003 的目标与路线同支，初值偏差 33°；600004 的目标与路线差整整 2π，初值偏差 350°——但**换成同分支目标后初值只差 10.1°，比 600003 还小**。

物理上两个 goal 的朝向只差 3.2°（−179.1° vs +177.7°），恰好一个落在 ±π 切割线的两侧，`atan2` 把 600004 的最短弧翻到了 CCW 一侧。这是 A 被触发的偶然因素，但不是根因——根因是端点等式不允许换支。

### 3.3 隔离实验

| 实验 | 目标分支 | S_upper | 结果 |
|------|---------|---------|------|
| 600003 生产 | −3.126（k=0） | 40.056 | **SUCCESS**，16.99 s，41 it |
| 600003 强行反向分支 | +3.1572（k=+1） | 40.056 | **失败** `BUDGET_EXHAUSTED` 39.01 s；attempt 0 `Infeasible` 109 it，残差 4.969 |
| 600004 生产 | +3.124（k=0） | 38.589 | 失败 40.81 s（10.02 / 14.58 / 7.52 s） |
| 600004 修正分支（k=−1） | −3.1592 | 38.589 | **仍失败** 40.95 s；attempt 0 `Infeasible` 60 it，残差 3.433 |
| 600004 修正分支 + 放宽 S | −3.1592 | 50 | 三次求解**都** `Solve_Succeeded`，但候选构造被认证拒绝 |

600003 反向分支一改就失败，说明这个机制是双向真实的；600004 修了分支仍失败，说明 A 是必要但不充分条件。

---

## 4. 根因 B：S 搜索域装不下走廊路线

### 4.1 代码事实

`optimizer.py:150`：

```python
upper = min(60.0, max(12.0, 3.0 * distance + 10.0))
```

上界只由起终点直线距离 d 决定，与走廊路线实际长度无关。`_route_guess`（`optimizer.py:373`）随后把初值长度夹到上界：

```python
S_guess = min(S_upper, max(S_lower, route_length))
```

600004：`S_guess = min(38.589, 45.744) = 38.589`，同时 `kappas *= route_length / S_guess = 1.185`，曲率被额外放大。

### 4.2 在 S 被卡住时的可行点分解

用 `probe_infeasible.py` 直接调用生产 `_solve_with_ipopt`（分支已修正，`S_upper` 仍为 38.589），取 IPOPT 最优努力点：

| 量 | 值 |
|----|-----|
| 求解器状态 | `Infeasible_Problem_Detected`，60 it，11.63 s |
| 端点 y 误差 | 2.5e-12 m |
| 端点 ψ 误差 | 4.6e-13 rad |
| **端点 x 误差** | **3.4326 m** |
| **走廊最大残差** | **3.0846 m**（+3.08 m 越过半空间） |
| 走廊正残差样本占比 | 2.14%（1540 个样本中约 33 个） |
| 最差样本位置 | allocation entry 0→1 的入口 portal，σ=0.4764，piece 3，q=0.811（region 1） |
| 曲率最大 |c| | 0.631（界 1.210，未违反） |

解释：ψ 和 y 都能精确满足，说明约束体系本身没写错；但 38.589 m 的长度不足以沿走廊走到终点，IPOPT 只能把 x 尽量往前推，代价是把车压出 region-1 入口 3.08 m。要同时满足走廊和端点，路径必须更长。

### 4.3 放宽 S 后立刻收敛

同一初值、只把 `S_upper` 提到 50（诊断进程内 monkeypatch，不落生产）：

| 量 | 值 |
|----|-----|
| 状态 | `Solve_Succeeded`，24 it，7.32 s |
| 求解得到的 S | **48.4363 m** |
| 端点 x / y / ψ 误差 | 2.7e-13 m / 8.0e-14 m / 4.4e-16 rad |
| 走廊最大残差 | **−1.49e-9 m** |
| 曲率最大 |c| | 0.281（界 1.210） |

也就是说，仅长度变量一项就足以让 NLP 从"不可行"变成"精确可行"。这是 B 的直接证据。

补充：只放宽 S 到 60、但不修分支（`diag_600004_supper60.json`）仍然失败（attempt 0 `Infeasible` 50 it，残差 5.093），说明 B 与 A 必须同时处理。

---

## 5. 根因 C：认证积分的长度天花板

### 5.1 代码事实

`geometry.py:499`：每段的初始 Simpson 子区间数按 `MAX_PANEL_WIDTH_M = 0.02` 定：

```python
"panels": _even_at_least(h / MAX_PANEL_WIDTH_M)
```

`bounds.py:252-267` 的浮点/角度 guard：

```python
roundoff = POSITION_ROUNDOFF_GUARD_M * (simpson_panels + 8)      # 1e-10 / panel
angle    = length_m * angle_bound_rad
```

`geometry.py:511-541` 的接受条件是 `euclidean_remainder + guard <= POSITION_BUDGET_M = 1e-7`（`bounds.py:17`）；不满足就把最差段的面板数翻倍，最多 64 轮；若下一次翻倍会使 `total_panels > MAX_PANELS/2 = 100000`，抛 `BOUND_UNCONFIRMED`（`bounds.py:21`）。

关键点：**guard 随面板数单调增加**。初始面板数已经 ≈ S/0.02，于是 guard ≈ `1e-10·(25S + n + 8) + S·angle_bound`，在 n=8 时约 **2.66e-9·S**。细化只会让 guard 变大，所以一旦 guard 单项超过 1e-7，任何细化都不可能成功。

### 5.2 guard 随 S 的数值（`cert_guard.json`）

| S (m) | n | 每段面板 | Simpson 面板 | guard (m) | ≤ 1e-7 |
|-------|---|---------|--------------|-----------|--------|
| 18.6258 | 8 | 118 | 480 | 5.1781e-8 | 是 |
| **21.7016（600003 实际解）** | 8 | 136 | 552 | **5.9473e-8** | **是** |
| **38.5895（600004 的 S_upper）** | 8 | 242 | 976 | **1.0458e-7** | **否** |
| **45.7444（600004 走廊长度）** | 8 | 286 | 1152 | **1.2332e-7** | **否** |
| 48.4363（600004 可行解） | 8 | 304 | 1224 | 1.3095e-7 | 否 |
| 38.5895 | 16 | 122 | 992 | 1.0926e-7 | 否 |
| 45.7444 | 16 | 144 | 1168 | 1.2858e-7 | 否 |
| 48.4363 | 16 | 152 | 1232 | 1.3563e-7 | 否 |

> 表值按 `guard = 1e-10·(full+local+8) + S·angle_budget(n, π + S·κ_limit)` 计算（κ_limit=0.5525，|ψ_start|≤π 取保守值），与生产实测 600003 的 guard 5.947e-8 **逐位吻合**。

由二分法，仅 guard 一项允许的最大长度：

- n=8：**36.8 m**
- n=16：**35.2 m**

600004 的 `S_upper` 本身（38.589）就已经超过这个天花板；它唯一可行的走廊解（48.436）更是超出 31%。

### 5.3 生产中的失败形态

| 实验 | 结果 |
|------|------|
| 600004 修正分支 + `S_upper=50`，n=8 | 三个候选都 `Solve_Succeeded`，但都在候选构造时抛 `BOUND_UNCONFIRMED: Simpson refinement would exceed 200000 subintervals [('total_panels', 107008)]`；整轮 38.78 s 后 `BUDGET_EXHAUSTED` |
| 同上，n=16 | attempt 0 求解成功但认证 `BOUND_UNCONFIRMED: Simpson refinement did not converge`；attempt 1 超时；总 48.29 s |

两种错误信息是同一个根因的两种收尾方式：

- n=8：细化把总子区间推到 107008，下一次翻倍会超过 200000，触发面板上限；
- n=16：局部面板更多、guard 更高（1.356e-7），64 轮细化耗尽仍未满足预算，报"did not converge"。

无论哪种，IPOPT 已经把 NLP 解出来了（端点、走廊、曲率都合格），**卡死在认证层**——这就是"NLP 有解但产不出可执行轨迹"。

### 5.4 成功样本的认证预算（对照组）

600003 的 `integration_budget`：

| 量 | 值 |
|----|-----|
| 路径长度 | 21.7016 m |
| 总子区间 / 每段面板 | 1088 / 136 |
| 细化步数 | 0 |
| floating point guard | 5.947e-8 m |
| Simpson 余项（m4≈0.012） | ≈1.2e-10 m |
| position_error_bound | **5.959e-8 m ≤ 1e-7** |

也就是说，600003 是"一次就过"，而且离预算还有 40% 的余量；600004 是"怎么细化都过不了"。

---

## 6. 为什么 600003 可以规划出来

三个条件同时成立：

1. **长度在认证天花板以内**：实际解 21.702 m < 36.8 m，guard 5.947e-8，0 次细化即通过。
2. **长度在搜索域以内**：`S_upper = 3·10.019 + 10 = 40.056 m > 路线 18.626 m`，初值不需要夹紧、曲率不需要放大。
3. **航向分支与走廊一致**：`goal−start = −3.126 rad` 与 guide 的 −2.286 rad 同支，初值偏差 0.583 rad，IPOPT 41 次迭代收敛。

缺任何一个都会失败：第 3 节把 600003 翻到反向分支后它同样 `BUDGET_EXHAUSTED`。

---

## 7. 对"同样是回头弯，只是直线部分长了一些"的解释

这个观察是准确的，而且正是失败的本质：

1. **同伦与曲率需求相同**：两者绕同一堵墙的同一端、同一方向，portal 半径几乎一样（0.83/1.16 vs 0.81/1.15 m），U 弯段本身对曲率的要求没有区别。600004 的曲率最大 |c| 只有 0.281，远低于界 1.210，几何上完全转得过来。
2. **长度是两把硬尺子的自变量**：
   - `S_upper` 只看直线距离 d，长直线绕行会让实际路线远大于 `3d+10`；
   - 认证 guard 只看总长度 S，每米约 2.66e-9 m 的固定成本，超过 36.8 m 就必然超预算。
   
   600004 多出来的约 27 m 直线，恰好同时越过这两条线。换句话说，**不是弯转不过来，是路太长**：在当前数值契约下，任何超过约 36.8 m 的路径都无法被认证，而这条走廊的唯一同伦需要 45.7 m 以上。
3. **±π 的航向表示是第三个独立触发点**：两个 goal 的物理朝向只差 3.2°，但一个落在 `atan2` 切割线的这一侧、一个在另一侧，使得 600004 的最短弧目标与走廊路线差 2π。即使长度问题不存在，这一点也会让 NLP 不可行。

所以"同一个弯、直线长一点"之所以结果完全不同，是因为直线长度同时决定了搜索域、可行同伦长度和认证预算，而这三个门限都在 36–38 m 附近；600003 在 21.7 m 处舒适通过，600004 在 45.7 m 处三线全撞。

---

## 8. 失败链条（600004 生产运行）

```text
route_guide initial, S 被夹到 38.589 m
  ├─ 航向目标 +3.124 rad 与走廊 −2.951 rad 差 2π   → NLP 不可行（68 it，10.02 s）
  ├─ 形状初值 1.2/left、1.2/right 继承同一错误分支 → 120 it 迭代超限 / 45 it 墙钟超时
  └─ GEOMETRY 层 30 s 软预算耗尽                    → BUDGET_EXHAUSTED / SOFT_BUDGET_EXHAUSTED

即使修掉分支：
  ├─ S ≤ 38.589 装不下走廊 → 端点 x 差 3.43 m、走廊超界 3.08 m（入口 portal）
  └─ S = 48.436 可解       → 认证 guard 1.31e-7 > 1e-7，候选构造 BOUND_UNCONFIRMED
```

对照实验排除了其他解释：

| 变量 | 结果 | 说明 |
|------|------|------|
| `guidance_mode=legacy_route` | `NO_ACCEPTED_CANDIDATE / SOLVER_NOT_CONVERGED`，32.45 s，6 个初值全败 | 不是走廊引导独有的问题 |
| `intervals=16` | attempt 0 111 it / 39.57 s 超时，总 48.45 s | 提高离散分辨率不解决 A/B/C |
| `solver=scipy` | `FIXED_ALLOCATION_LIMIT`，11.54 s，5 个初值全 `Positive directional derivative for linesearch` | 不是 IPOPT 调参问题 |
| 仅放宽 `S_upper=60` | 仍失败 40.95 s | 只修 B 不够 |

---

## 9. 如果要修（仅方向建议，本次未实现）

1. **A（端点分支）**：端点航向应按走廊同伦选 2π 分支（例如用 guide 累计转角确定 k），或把端点等式改为 mod 2π；但验收仍要用 `_wrap` 检查物理朝向，不能削弱审计。注意这会改变 600003/600004 之外的路径语义，需要契约评审。
2. **B（S 搜索域）**：把走廊路线长度纳入 `S_upper`，例如 `max(3d+10, route_length × 1.1)` 并保留 60 m 上限；S_lower 仍按原式。
3. **C（认证长度上限）**：这是最深的一层。当前 `MAX_PANEL_WIDTH_M=0.02` + 每面板 1e-10 的 guard 使可认证长度上限只有 36.8 m，本质上把"长路径"排除在可认证空间之外。可选方向：按段/局部误差重新分配面板（而不是全段 0.02 m）、提高 `POSITION_BUDGET_M`、或重新推导 guard 的每面板成本。这三种都改动数值契约，必须走 Codex 复审，不能顺手改。

即使三层都修，仍需重新跑三车连续 OBB、动力学与审计，本文不声称修复后一定可执行。

---

## 10. 证据清单与复现命令

目录：`evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z/`

| 文件 | 内容 |
|------|------|
| `repro_u_turn.py` | 复现入口（重建地图 → 校验请求 → `solve`） |
| `repro_30s.json` | 两条 plan 的完整复现结果（状态、attempt、走廊、metrics） |
| `probe_branch.py` | 重建走廊链、计算 guide 转角/分支、评估生产初值、可选的诊断性分支求解 |
| `probe_infeasible.py` | 复刻 attempt 0 的 IPOPT 调用，分解连续性/端点/曲率/走廊残差 |
| `probe_600003_evaluate.json` / `probe_600004_evaluate.json` | 初值评估（S、ψ_end、分支、走廊与曲率） |
| `probe_600004_branch.json` | 仅修分支，仍失败 |
| `probe_600004_branch_s50.json` | 修分支 + `S_upper=50`：求解成功、认证失败（n=8） |
| `probe_600004_branch_s50_n16.json` | 同上，n=16 |
| `probe_600003_wrong_branch.json` | 600003 反向分支失败 |
| `probe_600004_infeasible_branch.json` | S 卡死时的残差分解 |
| `probe_600004_conv_branch_s50.json` | S 放宽后的收敛点 |
| `cert_guard.json` | guard 随 S 的数值与 36.8/35.2 m 天花板（二分） |
| `diag_600004_legacy.json` / `diag_600004_n16.json` / `diag_600004_scipy.json` / `diag_600004_supper60.json` | 对照实验 |

复现命令（在 `/home/wq/whut_thesis/expv3_multi_virtual_a` 下）：

```bash
export PYTHONPATH=catkin_ws/src/fwis_v3_optimizer/src
MAP=/tmp/expv3-repro-map.t2Au9l/map05-p12-parking-row-env2/map.yaml
OUT=evaluation/virtual_a_corridor/UTURN_DIAG/20261118T041159Z

python3 $OUT/repro_u_turn.py --map-yaml $MAP --plan 600003 --plan 600004 \
        --output $OUT/repro_30s.json

python3 $OUT/probe_branch.py --map-yaml $MAP --plan 600004 --mode evaluate \
        --output $OUT/probe_600004_evaluate.json

python3 $OUT/probe_branch.py --map-yaml $MAP --plan 600004 --mode solve-branch \
        --relax-s-upper 50 --output $OUT/probe_600004_branch_s50.json

python3 $OUT/probe_infeasible.py --map-yaml $MAP --plan 600004 --branch-shift \
        --output $OUT/probe_600004_infeasible_branch.json
```

`probe_branch.py --mode solve-branch` 与 `probe_infeasible.py --branch-shift` 只在诊断进程内 monkeypatch `optimizer._search_packet`，不改生产文件；`cert_guard.json` 由生产 `bounds` 函数直接计算。

---

## 11. 边界声明

- 本文只做复现、隔离实验与原因分析；**未修改任何生产代码**，未生成 600004 的可执行轨迹。
- 诊断进程只使用私有 master，未干扰用户的 ROS 会话；未提交、未推送 git。
- 未宣称 Codex ACCEPTED，未进入 V3 或下一阶段。
- 文中"600004 唯一走廊同伦需要 ≥45.7 m"来自走廊硬约束下的数值证据（S=38.589 时最优努力点仍超界 3.08 m；S=48.436 时精确可行），不是形式化最短性证明。