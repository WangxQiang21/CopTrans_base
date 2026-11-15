# V0.2 编队规划 RViz 现象诊断报告

- 日期：2026-11-15
- 代码基线：`/home/wq/whut_thesis/expv3_multi_v02`
- `git rev-parse HEAD`：`6227833011e2d92b711179bfdec979bb0b878b29`
- 工作区状态：分析开始前已有大量未提交的 V0.2 / FR06 变更；本报告**没有修改任何代码、参数、阈值或证据文件**，只新增了本 Markdown 文档。
- 分析对象：`uniform_v02` 后端，`formation_demo.launch`
- 在线实验：
  - map04：`/tmp/expv3-test-ros.BBc1tI/log/latest/formation_uniform_planner-2.log`
  - map05：`/tmp/expv3-test-ros.XljlqJ/log/f7f2f834-e041-11f1-a57b-1b4c90db486b/`
  - 临时目录可能被清理；本报告已把用到的关键行固定在正文中。

> 本文的目的不是直接给出补丁，而是把三个 RViz 现象还原为可复现的机制链，供 Codex 做独立的代码级分析和方案决策。文中“已验证”与“推断”会明确区分。

---

## 0. 结论摘要

### 0.1 三个现象对应三个不同层

| 现象 | 直接原因 | 主要代码位置 |
| --- | --- | --- |
| 车辆一顿一顿 | 每个搜索边都是独立的 rest-to-rest 五次多项式块；搜索步长固定 1 m；块边界速度强制为 0；换轮时再插入静止 Setup | `uniform_search.py:123`、`uniform_timing.py:591`、`formation_retiming.py:1-8`、`retiming.py:127-140` |
| 简单开放区域规划不出来 | 每次扩展要 sweep 约 48 个 S 对齐自转 + 9 个 O 候选；开阔地图里每次扩展 2.4–3.0 s，30 s 只够约 12 次扩展；同时加权 A* 在“位置已到、ψ 未到”处形成局部极小，S 不改变 ψ，永远排不到 | `uniform_search.py:703-761`、`uniform_search.py:936-997`、`uniform_search.py:1168-1174`、`uniform_search.py:1237-1247` |
| 不用 A、改用 O、模式切来切去 | A straight 只沿“当前到目标的瞬时方向”生成，且要求三车 heading 精确匹配到 `1e-9`；默认 `initial_heading_policy: cargo_heading` 把 heading 设成 ψ，通常不匹配；要用 A 必须先 S 对齐，S 换轮 Setup 为 2.343–3.031 s；O 首块只付 0.3 s，于是 O 严格更便宜 | `uniform_search.py:82`、`uniform_search.py:818-826`、`uniform_search.py:873-934`、`transport_formation.yaml:31`、`mode_switch_timing.py:5-60` |

### 0.2 当前能力上限（经验值）

在本次两套地图和默认 30 s 预算下：

- 约 **7–8.4 m、几乎笔直、ψ 不变的路径可以成功**，但已经消耗 25–26 s wall、9 次扩展。
- 约 **5.9 m + 88° 转向** 的开放区域请求，30 s 失败；放宽到 90 s、35 次扩展仍然失败。
- 约 **10.8 m、隔墙、且终点 ψ 转 90°** 的请求，30/90/180 s 均失败。
- 30 s 预算下，搜索实际只能做 **10–12 次扩展**；`max_expansions=10000` 从未成为约束。

---

## 1. 复现方法与输入

### 1.1 地图与缩放

`run_test.sh` 的编队默认比例是“单车默认 × 3.75”；`run_demo.sh` 通过 `tools/scale_map_runtime.py` 缩放地图的分辨率和原点，栅格本身不重采样（`run_test.sh:59-70`、`run_demo.sh:327`、`tools/scale_map_runtime.py:67-75`）。机器人尺寸保持 0.738 m × 0.500 m。

| 地图 | 原始分辨率 | 原始 origin | 运行缩放 | 运行分辨率 | 运行 origin |
| --- | --- | --- | --- | --- | --- |
| map04-p12-maze-env1 | 0.011272 | (1.9352, 1.9352) | 1.875 | 0.021135 | (-5.1217765, -5.1217765) |
| map05-p12-parking-row-env2 | 0.024713 | (1.4578, 0.9719) | 2.625 | 0.064871625 | (-8.3609838, -5.5739559) |

因此在线日志里 map04 的 `res=0.021`、`1431x1431` 是缩放后的地图，不是原始地图。

### 1.2 场景起终点

map04 `scene_meta.yaml`：

```
start: x=1.4195060000000002, y=22.596776, theta=0.0
goal:  x=15.9603485,          y=17.5454735, theta=0.0
```

map05 `scene_meta.yaml`：

```
start: x=-5.7336816875, y=9.638438875, theta=0.0
goal:  x=3.0888566875000008, y=3.4107641875000003, theta=1.570796
```

后续在线点击产生的请求（日志中的实际值）：

| 请求 | start | goal | 说明 |
| --- | --- | --- | --- |
| map04 600000 | (1.420, 22.597, 0.000) | (15.960, 17.545, 0.000) | 自动起点，静态检查失败 |
| map04 600001 | (4.704, 22.243, -0.016) | (11.763, 22.226, -0.011) | 7.06 m 直线 |
| map04 600002 | (11.763, 22.226, -0.016) | (20.151, 22.179, -0.016) | 8.39 m 直线 |
| map04 600003 | (20.151, 22.179, -0.016) | (22.086, 16.640, -1.547) | 5.87 m + 87.7° 转向 |
| map05 600000 | (-5.734, 9.638, 0.000) | (3.089, 3.411, 1.571) | 10.80 m，隔墙，终点转 90° |
| map05 600001 | (-5.734, 9.638, 0.000) | (-0.284, 12.016, 0.044) | 5.95 m，终点转 23.6° |

### 1.3 在线启动参数

两次运行都是：

```
map_dir:=<缩放后的地图目录>
backend:=uniform_v02
scene:=manual_map
auto_goal:=true
rviz:=true
```

默认参数（`formation_demo.launch:52-61`）：

```
max_expansions=10000
uniform_max_seconds=30.0
uniform_translation_step=1.0
uniform_collision_margin=0.05
uniform_sweep_spacing=0.01
uniform_goal_xy_tol=0.02
uniform_goal_psi_tol=0.017453292519943295  # 1 deg
playback_rate=25.0
```

### 1.4 离线复现方式

在 `/tmp/fr_analysis_repro/` 下用与在线节点相同的 `plan_uniform`、同一缩放地图、同一 `SuccessorConfig()`、同一 `GraphSearchConfig(max_seconds=30)` 复现：

- `repro_request.py`：通用地图/起终点复现，输出 status、wall、expansions、candidates、accepted、collision、rejection histogram、path、phase table。
- `repro_map05.py`：map05 专用，带 pop 日志 monkeypatch。
- 读取 PGM 时按 `map_publisher.py` 的方式 `np.flipud`，占用阈值与 `map.yaml` 的 `occupied_thresh=0.65` / `free_thresh=0.196` 一致。

离线与在线的一致性：

| 请求 | 在线 | 离线复现 |
| --- | --- | --- |
| map04 600001 | SUCCESS，T_end 26.236 s，wall 24.940 s，1320 samples | SUCCESS，T_end 26.2166 s，wall 24.889 s，9 expansions |
| map04 600002 | SUCCESS，T_end 28.657 s，wall 26.039 s，1442 samples | SUCCESS，T_end 28.6508 s，wall 25.723 s，9 expansions |
| map04 600003 | 30 s `BUDGET_EXHAUSTED`，wall 32.055 s | 90 s `BUDGET_EXHAUSTED`，wall 91.591 s，35 expansions |
| map05 600000 | 30 s `BUDGET_EXHAUSTED`，wall 32.352 s | 30 s `BUDGET_EXHAUSTED`，wall 32.187 s，11 expansions |
| map04 自动起点 | `REQUEST_REJECTED / MAP_OBB_COLLISION`，0.027 s | 同样 0.0255 s，detail=`vehicle_id=2` |

map04 的 heading 用日志精确值 `-0.016215` 时，离线 T_end 与在线差 < 0.02 s，说明路线与时序模型一致。

---

## 2. 现象 1：为什么车辆一顿一顿

### 2.1 结论

不是播放器刷新问题，也不是 RViz 插值问题。运动模型本身就是**逐块 rest-to-rest**：每个搜索边单独做一次“静止 → 加速 → 匀速 → 减速 → 静止”的五次多项式，块与块之间不允许速度连续；搜索步长又是固定 1 m。因此一条 8.39 m 的直线被拆成 9 个运动块，停 9 次。

### 2.2 机制链

1. **搜索步长固定 1 m**：`SuccessorConfig.translation_step_m = 1.0`（`uniform_search.py:123`），O 平移候选按 1 m 生成（`uniform_search.py:788-816`）。
2. **每个边单独定时**：`uniform_timing.py` 把每个 FR01 边转成一个 `FormationBlock`，用 `_analyze_block` 单独求公共运动时长，然后加初始/边间静止换轮（`uniform_timing.py:1-10`、`uniform_timing.py:591-680`）。
3. **块边界强制零速**：`formation_retiming.py:1-8` 明确写着每个 block 是一个 rest-to-rest quintic progress phase，块边界永远是 zero-speed point；若轮角变化，整个编队在前面插入一个静止 reconfiguration event。
4. **五次多项式本身零端点速度**：`retiming.py:127-140` 的 `quintic(tau)` 在 `tau=0,1` 速度为 0；`bounded_duration` 用单位 quintic 的精确峰值 `1.875`、`10/√3`、`60` 反推时长。
5. **1 m 块的实际时长**：jerk 上限 2.0 m/s³ 给出

   ```
   T = (60 × 1 / 2.0)^(1/3) × 1.005 = 3.1227686685 s
   峰值速度 = 1.875 / T = 0.6004287 m/s
   峰值加速度 = (10/√3) / T² = 0.5920519 m/s²
   峰值 jerk = 60 / T³ = 1.9702975 m/s³
   ```

   即每一块都是 0 → 0.60 → 0 m/s 的“起步—走一米—刹停”。

### 2.3 证据：在线计划的相位表

map04 600002（离线复现，在线 T_end 一致）：

```
initial_setup 0.000–0.300  0.300 s
O motion      0.300–3.423  3.122769 s
O motion      3.423–6.546  3.122769 s
O motion      6.546–9.668  3.122769 s
O motion      9.668–12.791 3.122769 s
O motion     12.791–15.914 3.122769 s
O motion     15.914–19.037 3.122769 s
O motion     19.037–22.159 3.122769 s
O motion     22.159–25.282 3.122769 s
setup        25.282–26.355 1.072853 s
O motion     26.355–28.651 2.295814 s
```

- 9 个运动块 = 8 个 1 m 块 + 1 个 0.397 m 块。
- 9 次运动 = 9 次停车；T_end 28.651 s。
- 换轮 Setup 1.073 s 是静止的。
- 末块 0.397 m 用 2.296 s，峰速 `1.875×0.397/2.296 = 0.325 m/s`，比 1 m 块还慢。

map04 600001 更极端：

```
7 × 1 m O    : 7 × 3.122769 = 21.859 s
setup        : 2.550564 s（静止）
0.112 m O    : 1.506613 s（峰速 0.139 m/s）
总 T_end     : 26.216557 s
```

即“先沿 ψ 漂 7 m → 原地停 2.55 s 换轮 → 以 0.14 m/s 爬 11 cm → 停”。

### 2.4 为什么不是播放器的锅

- 播放器定时器是 `1/playback_rate`（`fwis_v3_playback/scripts/formation_playback.py:157`），默认 25 Hz；每次 tick 用公共时钟 `elapsed` 和解析 `frame_at` 求当前状态（同文件 `465-515`、`571`）。
- `playback_rate` 只是显示刷新频率，不是时间倍率。FR06 在线回放把 `playback_rate` 设到 100，T_end 仍是 33.024 s。
- 所以“每米停一次”来自模型相位表，不来自 ROS 发布节奏。

### 2.5 影响

- 停止次数 ≈ 运动边数 ≈ 路径米数（1 m 步长）。
- 每次停车都伴随加速度/速度归零；视觉上就是“一顿一顿”。
- 换轮 Setup 是额外的“完全静止”段：0.3 s（模式切换但轮角不变）、2.343 s（S 轮角翻转）、3.031 s（S→90° A 弧）、1.648 s（A→O）等。
- FR06 tight 2.8 m 走廊：6 个运动块、5 次普通停车、4 个 Setup（1 个初始 0.3 s + 3 个边间共 7.022 s），T_end 33.024 s；其中 S 块 2.111 s、90° A 弧 11.100 s。

---

## 3. 现象 2：为什么简单开放区域规划不出来

### 3.1 结论

有两个独立但叠加的原因：

1. **吞吐**：每次扩展要 sweep 约 48 个 S 对齐自转 + 9 个 O 候选。开阔地图里每次扩展 2.4–3.0 s，30 s 只够 10–12 次扩展。`max_expansions=10000` 从未生效。
2. **搜索局部极小**：加权 A* 的启发式只看 cargo ψ 的旋转，不看 body heading；S 自转不改变 ψ，所以 S 不减少 h，却增加 g。搜索必须先接受一次 f 上升，才能解锁随后的 A 弧（f 大幅下降）。在“位置已到、ψ 未到”处，大量 O 状态的 f 更低，S 永远排不到。搜索于是在目标附近来回振荡。

### 3.2 候选分解：185 = 9 O + 8 A straight + 120 A arc + 48 S

这是理解吞吐的关键。以 map04 600003 起点为例，`SuccessorGenerator.generate` 每次生成：

| 候选族 | 数量 | 是否 sweep |
| --- | --- | --- |
| O：8 个 cargo 方向 + 1 个 goal 方向 | 9 | 是 |
| A straight：1 个方向 × 8 个 gear 组合 | 8 | 仅在 heading 精确匹配时 |
| A arc：15 个 scheme × 8 个 gear 组合（5 pivot × ±10°，加 5 个 crop 角） | 120 | 仅在 heading 精确匹配时 |
| S：A scheme 要求的 heading 集合，去重后 | 48 | 是 |
| 合计 | **185** | |

起点 `headings = ψ = -0.016215`，所有 A scheme 都要求别的 heading，因此：

```
128 个 A 候选 → 立即 A_HEADING_REQUIRES_S_ALIGNMENT（不 sweep）
9 个 O + 48 个 S = 57 个候选 → 真正 sweep
实测：43 个 MAP_OBB_COLLISION + 14 个 accepted = 57
```

这与 `generate` 返回的 histogram 完全吻合：

```
candidate_count = 185
rejection_histogram = {A_HEADING_REQUIRES_S_ALIGNMENT: 128,
                       MAP_OBB_COLLISION: 43}
accepted = 14
```

后续每次扩展只要 heading 仍是 ψ（O 模式不改变 heading），这个 128/48/9 的结构就保持不变。实测 map04 600003 的 12 次扩展：

```
12 × 128 = 1536  A_HEADING   （histogram 实测 1536）
12 × 185 = 2220  candidates  （实测 2220）
12 次扩展的 collision = 30.248 s
```

map04 600001 的 9 次扩展是 **1665 = 9 × 185**，与上面的 185 分解完全一致（该请求目标 ψ 比起点多 0.005 rad，crop 弧仍在，A 候选为 128）。26.216 s 的总时长主要花在 7 个 1 m O 块上，搜索本身的 collision 为 21.922 s。

map04 600002 则更有说服力：它的目标 ψ 与起点 ψ **完全相同**（都是 -0.016），crop 弧角为 0，于是 5 个 crop scheme 被 `a_arc_schemes` 跳过，A 候选从 128 降到 88（8 straight + 80 fixed arc），但 **S 对齐目标仍是 48 个**：

```
每次扩展：9 O + 48 S = 57 个 sweep
          8 A straight + 80 fixed A arc = 88 个 A 候选，全部 heading 拒绝
          合计 145 个候选
9 次扩展：9 × 145 = 1305 candidates（实测 1305）
          9 × 88  = 792 A_HEADING（实测 792）
          9 × 57  = 513 sweeps；collision 22.448 s → 43.8 ms/sweep
```

即：**这个请求完全不需要转向，crop 弧根本不存在，搜索仍然每次扩展 sweep 48 个 S 对齐自转。** 这是“简单直线也慢”的直接原因。

**结论：每次扩展真正昂贵的是 57 个 sweep（9 O + 48 S），而不是候选总数。** 48 个 S 中，绝大多数是为固定 ±10° 弧服务的；真正对当前目标有用的通常只有 1–2 个（crop 弧）。在开阔地图里，每个 sweep 约 43–52 ms，57 个就是 2.4–3.0 s/扩展。这个“57 sweep/扩展”模型在多次运行中高度一致：

| 请求 | expansions | 57×exp | collision | ms/sweep |
| --- | --- | --- | --- | --- |
| map04 600001 | 9 | 513 | 21.922 | 42.7 |
| map04 600002 | 9 | 513 | 22.448 | 43.8 |
| map04 600003（30 s） | 12 | 684 | 30.248 | 44.2 |
| map05 600000（30 s） | 11 | 627 | 31.486 | 50.2 |
| FR06 wide #1 | 10 | 570 | 29.543 | 51.8 |
| FR06 tight #1 | 6 | 342 | 5.490 | 16.1（窄走廊早退） |

tight 走廊里 S 很早就碰撞退出，所以 FR06 tight 的 collision 只有 0.915 s/扩展；开阔地图则是完整 sweep，接近 2.5 s/扩展。

### 3.3 吞吐与预算

| 请求 | 预算 | 结果 | wall | expansions | candidates | accepted | collision | collision/exp | collision/wall |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| map04 600001 | 30 s | SUCCESS | 24.889 | 9 | 1665 | 252 | 21.922 | 2.436 | 88.1% |
| map04 600002 | 30 s | SUCCESS | 25.723 | 9 | 1305 | 267 | 22.448 | 2.494 | 87.3% |
| map04 600003 | 30 s | BUDGET | 31.009 | 12 | 2220 | 409 | 30.248 | 2.521 | 97.5% |
| map04 600003 | 90 s | BUDGET | 91.591 | 35 | 6473 | 1002 | 89.374 | 2.554 | 97.6% |
| map05 600000 | 30 s | BUDGET | 32.187 | 11 | 1869 | 412 | 31.486 | 2.862 | 97.8% |
| map05 600000 | 90 s | BUDGET | ≈91.5 | 30 | 4645 | 973 | 90.800 | 3.027 | ≈99% |
| map05 600000 | 180 s | BUDGET | ≈181 | 58 | 8677 | 1701 | 176.430 | 3.042 | ≈97% |
| FR06 wide 3.2 #1 | 30 s | BUDGET | 30.272 | 10 | 1832 | 476 | 29.543 | 2.954 | 97.6% |
| FR06 tight 2.8 #1 | 30 s | SUCCESS | 8.637 | 6 | 1024 | 48 | 5.490 | 0.915 | 63.6% |

说明：

- 30 s 预算的检查在循环顶部（`uniform_search.py:1237-1247`），一次扩展 2–3 s，所以 wall 会冲到 32 s 左右（在线 32.055 / 32.352，FR06 wide 32.769）。
- 10000 次扩展预算从未接近；真正约束是墙钟。
- 8.4 m 直线用 9 次扩展、25.7 s；有效上限就是“几乎笔直、8–9 m、ψ 不变”。
- FR06 tight 是 2.8 m 走廊、6 次扩展成功，说明短距离窄走廊可以过；一旦需要转向或在开阔地图扫描大量 S，预算就不够。

### 3.4 关键案例：map04 600003 的局部极小

请求：

```
start (20.151, 22.179, -0.016215)
goal  (22.086, 16.640, -1.547)
欧氏距离 5.867 m
ψ 变化 -1.530785 rad = -87.7°
```

30 s 运行的 12 次 pop（monkeypatch 记录，按 f 排序）：

| # | mode | qf (x,y,ψ) | g | h | f = g + 3h |
| --- | --- | --- | --- | --- | --- |
| 1 | A | (20.151, 22.179, -0.016) | 0.000 | 13.075 | 39.224 |
| 2 | O | (21.151, 22.163, -0.016) | 1.550 | 12.742 | 39.777 |
| 3 | O | (21.846, 21.444, -0.016) | 4.791 | 11.753 | 40.051 |
| 4 | O | (21.896, 20.446, -0.016) | 7.877 | 10.503 | 39.387 |
| 5 | O | (21.946, 19.447, -0.016) | 9.127 | 9.253 | 36.887 |
| 6 | O | (21.996, 18.448, -0.016) | 10.377 | 8.003 | 34.387 |
| 7 | O | (22.046, 17.449, -0.016) | 11.627 | 6.753 | 31.887 |
| 8 | O | (22.029, 16.449, -0.016) | 13.531 | 5.989 | 31.498 |
| 9 | O | (21.980, 17.448, -0.016) | 12.281 | 6.759 | 32.559 |
| 10 | O | (21.930, 18.447, -0.016) | 11.031 | 8.008 | 35.053 |
| 11 | O | (21.914, 17.447, -0.016) | 12.281 | 6.772 | 32.597 |
| 12 | O | (21.897, 16.447, -0.016) | 13.531 | 6.078 | 31.764 |

观察：

- 12 次 pop **全部是 O 或初始 A，没有一次 S**。
- pop 8 已经到达目标 xy 附近（0.199 m），但 ψ 仍是 -0.016，旋转项让 h 保留 5.742。
- 之后在 y≈16.44 与 y≈17.45 之间来回，f 在 31.5–35 之间，直到 30 s 预算用完。

为什么 S 永远排不到？在 pop 8 的状态上：

```
当前 O 状态：g = 13.531，h = 5.989，f = 31.498

S 前置（对齐 crop 弧所需 heading）：
  setup 2.343 + 自转 proxy 3.927 = 6.270
  S 后 g = 19.801，ψ 不变，h 仍 5.989
  f = 19.801 + 3×5.989 = 37.768   （f 上升 +6.27）

A crop（pivot (0,0)，-1.5308 rad）：
  cost 4.944
  A 后 g = 24.745，ψ = goal，h = 0.249
  f = 24.745 + 3×0.249 = 25.492   （f 下降 12.28）
```

即：搜索必须先接受一次 **+6.27 的 f 上升**，才能解锁随后 **−12.28 的 f 下降**。但 open set 里始终有大量 O 状态 f≈31.5，S 状态 f≈37.8 永远不是最小值。这就是加权 A* 的局部极小，而不是“没有可行路径”。

在起点，S 候选的 f 同样高于 O：

```
最便宜的 S：cost 2.923 → f = 2.923 + 39.224 = 42.147
解锁 crop 的 S：cost 6.270 → f = 45.494
O 第一步：cost 1.550 → f = 39.777
```

所以从第一次扩展开始，S 就被 O 压住。

### 3.5 已验证：转向前缀本身是可行的

用同一 generator 直接构造并 sweep，不经过搜索：

```
S_to_A_arc_p0_0_pos_gear--+        setup+spin = 4.307 s
A_arc_crop_p0_0_neg_gear++-        setup+arc  = 4.944 s
结果 qf = (20.151, 22.179, -1.547)，即 ψ 已到目标
S sweep：OK
A sweep：OK
```

也就是说，从起点出发，**S + 90° crop 弧的两条边都是无碰撞的**，总 proxy g = 9.251。搜索没有走到它，不是几何不可行，而是 f 排序把它饿死。

转向后，剩余 5.867 m 的 O 平移只有部分方向可行（旋转后的编队 footprint 在该走廊里更宽）：

```
A 后状态 generate：
  accepted 54，collision 3.790
  可行 O：O_cargo_02（cost 5.769，含 4.519 s 换轮）
  O_goal 被 MAP_OBB_COLLISION 拒绝（vehicle 1, station 0.287）
  O_cargo_00/01/07 也被拒绝
```

这说明“先转再走”不一定直接可行，转向的时机/位置需要搜索；但搜索连一次转向都没有展开。

### 3.6 map05 的失败模式

map05 的第一个目标与起点之间隔着一道中央墙。沿起终点直线采样（编队中心点，401 个采样）：

```
距离 10.799 m，方位 -0.615 rad
占用采样 109/401
第一个占用点：距起点 3.321 m，坐标 (-3.0208, 7.7234)
以中心点为圆心、半径 0.5/0.9/1.25 m 内有障碍的采样：
  42/81、54/81、62/81
```

所以它不是“直线开放区域”，而是一个需要绕墙、且终点还要转 90° 的请求。

复现结果：

- 30 s：11 次扩展、1869 候选、412 accepted、collision 31.486 s，无解。
- 90 s：30 次扩展、4645 候选、973 accepted、collision 90.800 s，无解。
- 180 s：58 次扩展、8677 候选、1701 accepted、collision 176.430 s，无解。

90 s 的 pop 日志中，pop 8 已经出现 ψ=1.571 的状态：

```
POP (-5.73, 7.64, 1.571) g=14.64
h = 12.22，f = 51.33
```

起点 f = 58.17。旋转后 f 下降，说明搜索确实尝试了旋转；但随后仍在起点 2 m 范围内反复，g≈20，无法建立通往目标的走廊。这是同一类局部极小/排序问题的另一种表现：O 链先绕墙，f 升高后 S 才有机会 pop；旋转后也没有稳定的 A/O 序列。

### 3.7 其它会放大“规划不出来”观感的因素

1. **目标桥在 PLANNING 时拒绝新目标**：`BUSY_STATES = ("PLANNING", "PLAYING")`（`formation_goal_bridge.py:43`），`_refuse_when_busy` 直接拒绝并打印 `refusing ... job ... is PLANNING`（`formation_goal_bridge.py:196-212`）。一次失败要等约 32 s 才释放，用户中途点的 2D Pose / 2D Nav Goal 会被丢弃，表现就是“点了没反应”。
2. **map04 的自动起点本身非法**：`(1.419506, 22.596776, 0)` 的静态检查在 0.0255 s 内失败，`detail=start static check failed: vehicle_id=2`。这不是搜索失败，而是场景输入与编队 footprint 不匹配。
3. **播放器对拒绝信封报 ERROR**：规划失败时节点仍发一个空 envelope，播放器在 `formation_playback.py:573` 抛 `model_sha256 must be 64 hex digits`。这是日志噪音，不代表多失败了一次。
4. **预算检查粒度粗**：只在循环顶部检查，最后一次扩展会超出 30 s 约 2–3 s；这不是主因，但会拉长“点了没反应”的等待。

---

## 4. 现象 3：为什么不用 A、为什么 O 切来切去

### 4.1 A straight 在当前实现里几乎不可用

`_a_straight_displacements` 只生成 **一个** 候选：沿“当前到目标的瞬时方向”，长度 `min(1 m, remaining)`（`uniform_search.py:818-826`）。然后每个 gear 组合要求三车 heading 精确等于该方向（或反向），容差是 `HEADING_MATCH_TOL_RAD = 1e-9`（`uniform_search.py:82`、`_headings_match` 548-551、`_add_a_straight` 873-902）。

默认起点 heading 来自 `initial_heading_policy: cargo_heading`（`transport_formation.yaml:31`），桥节点把 `theta_i(0) = psi_F(0)`（`formation_goal_bridge.py:306`）。于是：

| 请求 | 起点 ψ | 到目标方位 | 偏差 | A straight |
| --- | --- | --- | --- | --- |
| map04 600001 | -0.016215 | -0.002408 | 0.013807 rad = 0.791° | 全部拒绝 |
| map04 600002 | -0.016215 | -0.005603 | 0.010612 rad = 0.608° | 全部拒绝 |
| map04 600003 | -0.016215 | -1.234709 | 1.218494 rad = 69.8° | 全部拒绝 |
| map05 600000 | 0.000 | -0.614663 | 0.614663 rad = 35.2° | 全部拒绝 |

map04 600002 的 1305 个候选里，**792 个**是 `A_HEADING_REQUIRES_S_ALIGNMENT`。这不是“部分扩展改变了 heading”，而是因为该请求的目标 ψ 与起点 ψ 相同（均为 -0.016），crop 弧角为 0，5 个 crop scheme × 8 gear = 40 个 A 候选被跳过，于是每次扩展的 A 候选是 8 straight + 80 fixed arc = 88，9 × 88 = 792。map04 600003 的 90 s 运行里 6473 个候选中有 **4474 个** A_HEADING（约 69%）。

关键点：**0.6° 的偏差也足以让 A 完全不合法**。当前没有“沿 ψ 的 A straight”，也没有“带小曲率修正的 A straight”，所以只要初始 heading 不是目标方位，A 就必须先 S。

### 4.2 用 A 的代价：S 换轮 2.34–3.03 s

轮角配置（用 `wheel_configuration` 实测）：

```
A straight:  [0.0, 0.0, -0.0, -0.0]
O (phi=0):   [0, 0, 0, 0]        # 与 A 相同
S:           [-0.935769591, 0.935769591, 0.935769591, -0.935769591]
```

`mode_switch_event`（`mode_switch_timing.py:5-60`）对 S 轮角翻转 0.93577 rad 给出：

```
duration = max(0.3, 0.15 + steering_min)
steering_min = max(trapezoid, quintic)
             = 2.1932 s
duration     = 2.3432 s
```

FR06 的实测相位：

| 切换 | 实测 duration |
| --- | --- |
| O→S（→S 自转） | 2.343 s |
| S→A（90° crop 弧） | 3.031 s |
| A→O | 1.648 s |
| 初始 A→O（轮角不变，仅模式变化） | 0.300 s |

因此：

- **O 第一块**：0.3 s Setup + 1.25 s proxy = 1.55 s（与 600001/600002 的 g 完全一致：1.55 → 2.80 → 4.05 → …）。
- **S 第一段**：2.343 s Setup + 自转 proxy；最便宜 2.923 s，解锁 90° crop 的是 6.270 s。
- **A 弧真正运动之前**，已经付了 2.343 s（O→S 换轮）+ 2.111 s（S 自转）+ 3.031 s（S→A 换轮）≈ 7.49 s，其中静止换轮 2.343 + 3.031 = 5.374 s。

在搜索自己的 g 里，O 严格更便宜，所以选 O 不是“优化器随便偏袒”，而是当前原语/代价定义下的理性选择。

### 4.3 O 链的“生硬”：沿 ψ 漂移 + 末端横向修正

O 提供 8 个 cargo 方向 + 1 个 goal 方向（`uniform_search.py:788-816`）。沿 ψ 的 `O_cargo_00` 与目标方位只差 0.6–0.8°，且轮角不变（phi=0），每一步 Setup=0，cost=1.25 s；`O_goal` 的方向随位置变化，oblique 角变化会触发 Setup。因此搜索偏好沿 ψ 的 O 链，横向误差慢慢积累，最后再一次性修正。

map04 600001 的末端：

```
最后一段起点 (11.703104, 22.131005)
终点           (11.763000, 22.226000)
剩余位移       (0.059896, 0.094995)，长度 0.112301 m
世界方向       1.008240 rad（57.77°）
ψ              -0.016215 rad
oblique phi    1.024455 rad（58.70°）
```

O 的轮角就是 oblique phi；从 phi=0 到 1.0245 rad，`mode_switch_event`：

```
steering_min = 2.4011 s（rate-limited quintic）
duration     = 0.15 + 2.4011 = 2.5511 s
```

与实测 2.55056 s 一致。然后 0.112 m 的块又因为 rest-to-rest 用了 1.5066 s，峰速 0.139 m/s。

map04 600002 的末端：

```
剩余位移 (0.389024, 0.080995)，长度 0.397366 m
世界方向 0.205267 rad（11.76°）
phi      0.221482 rad（12.69°）
duration 1.0733 s（实测 1.07285）
```

所以“生硬”的精确机制是：**横向漂移只有几厘米到十几厘米，但末端剩余位移方向与 ψ 的夹角可以很大（600001 达 58.70°，600002 为 12.69°），O 的轮角因此被拉到 58.7°/12.7°，触发 2.55 s/1.07 s 的静止换轮，再以很慢的短块爬完最后一点。** 这不是路径长度问题，而是 O 原语的轮角 = 平移方向相对 heading 的夹角，小位移 + 大夹角会被转向速率放大成很长的 Setup。

### 4.4 转弯为什么必然出现 S/A 切换

V0.2 的三个原语：

- **O**：编队整体平移，ψ 和所有 body heading 不变。
- **A straight**：沿 heading 平移，ψ 和 heading 不变。
- **A arc**：绕固定 pivot 旋转，ψ 改变，body heading 必须等于弧切线。
- **S**：三车原地自转，body heading 改变，**ψ 不变**（`s_spin_edge` 的 start_qf = end_qf）。

因此：

1. 只要目标 ψ 与当前 ψ 不同，**必须用 A arc**；S 单独无法改变 ψ。
2. A arc 的起点 heading 必须精确等于弧切线；当前 heading 是 ψ，通常不等于切线，所以必须先 S。
3. 当前弧库是 5 个固定 pivot × ±10° + 5 个 crop 角（`a_arc_schemes`，`uniform_search.py:593-627`），换 pivot 或换方向就要重新对齐。
4. 搜索的 S 库是“所有 A scheme 的 required heading 集合”，去重后 48 个；每个都是一次三车自转 sweep。绝大多数与当前目标无关。

FR06 tight 的路线是 `O → O → S → A(+π/2) → O → O`，正是这套原语的必然结果：

```
setup 0–0.3          0.300
O     0.3–3.423      3.123
O     3.423–6.546    3.123
setup 6.546–8.889    2.343
S     8.889–11.000   2.111
setup 11.000–14.031  3.031
A     14.031–25.131  11.100
setup 25.131–26.779  1.648
O     26.779–29.902  3.123
O     29.902–33.024  3.123
```

6 个运动块、4 个 Setup、5 次普通停车，总 T_end 33.024 s；其中 7.022 s 是边间静止换轮。90° A 弧真实 11.1 s，而 `motion_proxy_s` 只按“最大中心行程 / v_max”估：

```
pivot (0,2)，三车半径最大 3 m
中心行程 = 3 × π/2 = 4.712 m
proxy = 4.712 / 0.8 = 5.890 s
```

即 proxy 低估了约 1.9 倍。proxy 的 docstring 也明确承认它忽略加速度、jerk、轮速和 quintic，既不是 S8 时长，也不是可采纳下界（`uniform_search.py:391-410`）。

### 4.5 用户期望的“三辆车一起 A 过去”为什么没有发生

- 如果三车 heading 已经等于目标方位，A straight 是可以用的；但默认 `cargo_heading` 让 heading=ψ，而目标方位通常不等于 ψ。
- 如果允许 A 从任意 heading 起步（带一点转向修正），或者允许任意曲率的 A 弧，就不需要 S 对齐；但当前 V0.2 原语库没有这两个能力。
- 如果 S 能改变 ψ，就不需要 A 弧；但 V0.2 定义 S 不改变 ψ，改这个等于改契约。
- 所以“直接 A 过去”在当前原语/契约下不是搜索偏好问题，而是**能力缺失**。

---

## 5. 代码索引

路径均相对仓库根目录。

| 主题 | 文件:行 | 说明 |
| --- | --- | --- |
| 搜索步长 | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py:123` | `translation_step_m: float = 1.0` |
| A heading 精确容差 | 同上 `:82` | `HEADING_MATCH_TOL_RAD = 1e-9` |
| 状态 key | 同上 `:305-340` | xy 0.2 m / ψ 10° / heading 10° + wheel label + a_labels |
| Setup 代价 | 同上 `:357-388` | `setup_demand_s`，与 FR02 `_make_setup` 同规则 |
| 运动 proxy | 同上 `:391-410` | `max(中心行程/v_max, 偏航/ω)`，明确非 S8、非下界 |
| heading 匹配 | 同上 `:548-551` | 1e-9 |
| A 弧库 | 同上 `:593-627` | 5 pivot × ±10° + crop |
| 候选生成总入口 | 同上 `:703-761` | O / A straight / A arc / S 的顺序 |
| 单候选 cost+sweep | 同上 `:711-737` | `consider()` 里立即 sweep |
| 目标判定 | 同上 `:765-785` | xy 0.02 m、ψ 1°、body heading 可自由 |
| O 候选 | 同上 `:788-816` | 8 cargo + goal |
| A straight 候选 | 同上 `:818-826` | 只有 goal 方向，1 m |
| O 构造 | 同上 `:841-871` | oblique 分支 |
| A straight 构造/拒绝 | 同上 `:873-902` | `A_HEADING_REQUIRES_S_ALIGNMENT` |
| A arc 构造/拒绝 | 同上 `:904-934` | 同上 |
| S 对齐目标 | 同上 `:936-977` | 所有 A scheme 的 required headings |
| S 构造 | 同上 `:979-997` | 原地自转 |
| 搜索预算 | 同上 `:1046-1049` | 10000 / 30 s / w=3 / rotation=3 |
| 启发式 | 同上 `:1168-1174` | `distance/v_max + 3×rotation/yaw_max` |
| 预算检查 | 同上 `:1237-1247` | 只在循环顶部 |
| f 入堆 | 同上 `:1332` | `g + 3×h` |
| A straight required headings | `.../uniform_motion.py:388` | 精确切线/反切线 |
| A arc required headings | `.../uniform_motion.py:416` | 精确弧切线 |
| 边构造 | `.../uniform_motion.py:1557,1586,1629,1661` | A straight / A arc / O / S |
| 公共定时 | `.../uniform_timing.py:1-10,591-680` | 每边一个 block + 静止 Setup |
| rest-to-rest 声明 | `.../formation_retiming.py:1-8` | 块边界零速 |
| block 时长 | `.../formation_retiming.py:798-913` | max(rate, sqrt(accel), cbrt(jerk)) × 1.005 |
| quintic | `.../retiming.py:127-140` | 端点零速 |
| 模式切换事件 | `.../mode_switch_timing.py:5-60` | 0.3 s 基线、转向率/加速度限制 |
| 管线入口 | `.../uniform_pipeline.py:566-...` | 静态检查、初始状态、搜索、重建/审计 |
| 目标桥 busy 拒绝 | `catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py:43,196-212` | PLANNING/PLAYING 拒绝新目标 |
| heading 初始化 | 同上 `:306` + `config/transport_formation.yaml:31` | `cargo_heading` → θ_i=ψ |
| 启动参数 | `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch:52-61` | 30 s / 1 m / 0.01 m / 0.02 m / 1° |
| 播放器 tick | `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py:157,465-515` | 25 Hz，解析求值 |
| 空 envelope 报错 | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py:573` | `model_sha256 must be 64 hex digits` |

---

## 6. 已验证、推断与竞争假设

### 6.1 已验证

- 1 m O 块 = 3.122769 s，峰速 0.600 m/s，端点零速；离线/在线 T_end 一致。
- 600001 末端 0.112 m 块 + 2.55056 s Setup；600002 末端 0.397 m 块 + 1.07285 s Setup；Setup 角度和时长可由 `mode_switch_event` 精确复算。
- 600003 的 12 次 pop 全是 O；S 候选存在但 f 更高；S+A crop 前缀无碰撞且能把 ψ 转到目标。
- 600003 的 185 = 9 O + 8 A straight + 120 A arc + 48 S 分解与 histogram 逐项吻合（12×128=1536 A_HEADING、12×185=2220 candidates）；600002 的 145 = 9 O + 8 A straight + 80 fixed A arc + 48 S 分解也吻合（9×88=792 A_HEADING、9×145=1305 candidates）。
- map04 自动起点因 vehicle 2 静态碰撞被拒绝。
- map05 直线采样有 109/401 占用，第一处占用在 3.321 m；不是无墙直线。

### 6.2 推断（需要 Codex 代码级确认）

- 48 个 S sweep 是 collision 时间的主要来源：每次扩展共 57 个 sweep（9 O + 48 S），S 占 84%。按 44–52 ms/sweep 计，600003 的 S 部分约 25–26 s（总 collision 30.2 s），600002 约 18.8 s（总 22.4 s），map05 30 s 约 26.4 s（总 31.5 s）。但具体每个 sweep 的采样数取决于碰撞早退位置，需要 instrument 确认。
- 加权 A* 的 f 局部极小是 600003/600002 转向失败的充分解释；是否存在其他低 f 路径可以在不经过 S 的情况下改变 ψ，需要更完整的可达性分析。
- map05 的旋转发生在 pop 8；后续 thrash 的具体 open-set 组成未逐条记录，只能从 30/90/180 s 的扩展数和 pop 摘要推断。
- 真实硬件上，Setup 和 rest-to-rest 的“顿挫”可能被控制器平滑；本文只分析规划/定时模型输出的理想轨迹。

### 6.3 已排除/不成立的解释

- **“RViz 刷新率太低”**：25 Hz 足够；`playback_rate=100` 也不改变 T_end。
- **“碰撞检查误报导致 A 弧不可行”**：600003 的 S+A crop 前缀直接 sweep 为 OK。
- **“地图缩放太小/机器人太大”**：运行时地图被放大 1.875×/2.625×，自由空间相对机器人更大；且 map04 直线成功。
- **“目标桥完全不工作”**：空闲时能正常接收；只是在 PLANNING 的 30 s 内拒绝新点击。
- **“`max_expansions=10000` 太小”**：从未接近；约束是 30 s 墙钟。

---

## 7. 建议 Codex 重点分析/决策的问题

以下只是分析方向，不代表已实现或已批准。

### 7.1 契约层（会影响 FR01/FR02/FR05/FR06 证据，必须先决策）

1. **rest-to-rest 是否是冻结契约？** `formation_retiming.py:1-8` 明确块边界零速。如果这是 S8/FR02 的冻结行为，那么“顿挫”是设计后果，只能在搜索/后处理层合并连续同向块，不能改定时层。
2. **`initial_heading_policy: cargo_heading` 是否是冻结行为？** 它系统性地让 A straight 不可用（除非目标方位恰好等于 ψ）。如果改成 `tangent`，A 可能可用，但会改变所有已验收实验的初始状态。
3. **`HEADING_MATCH_TOL_RAD=1e-9` 是数值精确性要求还是建模选择？** 0.6° 的偏差是 0.0106 rad，1e-3 rad 的容差仍然通不过；要覆盖 0.6–0.8° 至少需要约 0.014 rad（0.8°）。若放宽到 0.02 rad（约 1.15°），两个直线请求会通过，但这等于允许 A 的运动方向与 heading 有偏差，需要转向修正或新的原语，不能只改容差。
4. **A straight 是否只允许 goal 方向？** 当前 `_a_straight_displacements` 只给一个方向；若允许 8 个 cargo 方向或任意方向，A 的可用性会大幅提高，但候选数×8。
5. **A 弧是否必须固定 5 pivot / 10°？** 当前库很粗；允许任意曲率/任意 pivot 可以减少 S 对齐次数，但会改变“固定库”的冻结声明。
6. **S 是否允许改变 ψ？** 若允许，S 可直接完成转向，不再需要 A 弧；但这会改变 V0.2 原语定义。

### 7.2 搜索层（可能不动契约）

1. **S 宏动作**：把“S + 随后的 A”作为一个 macro-action 计 cost，让 f 看到净收益，而不是先付 S 的 f 上升。或者给 S 一个“对齐潜力”启发项。
2. **只生成目标相关的 S**：当前每次扩展 48 个 S，绝大多数为固定 ±10° 弧服务。若只对 crop 弧、或对当前最可能的 A 方向生成 S，sweep 数可从 48 降到个位数。
3. **惰性 sweep / 粗筛**：A 候选先做粗筛，S 先做 cheap clearance，只有可能被 pop 的候选才做 0.01 m sweep。
4. **g 用真实 S8 时长**：`motion_proxy_s` 低估 1 m 块 2.5×、90° 弧 1.9×；用真实时长会改变 O/A 的相对代价，但也会让所有 g 变大，需要重新评估预算。
5. **启发式**：`goal_heuristic_s` 明确不是下界；`heuristic_weight=3`、`rotation_weight=3` 把旋转项放大 9 倍。是否改用可采纳/一致启发式，或把“body heading 对齐”纳入状态/启发式。
6. **目标附近的两阶段策略**：先规划 xy，再在终点附近单独规划 ψ；或者把“终点 ψ 旋转”作为一个显式子目标。
7. **tie-break**：8 个 cargo 方向 f 几乎相同，沿 ψ 的 O_cargo_00 因为无 Setup 总是赢；是否显式偏好 goal 方向/减少横向漂移。
8. **预算**：调大 30 s 只能缓解吞吐，不能解决 600003 的 f 局部极小（90 s/35 次扩展仍失败）。需要和 7.2.1/7.2.2 一起做。

### 7.3 定时/轨迹层（若允许改 FR02）

1. **合并连续同向块**：8 个 1 m O 块合并成一个 8 m 块，消除 7 次停车；但需要保持审计/相位表兼容。
2. **非零边界速度**：在非模式切换处允许速度连续；会改变 rest-to-rest 契约和所有 T_end。
3. **更大的 `translation_step`**：只能减少块数，不能消除每块 rest-to-rest 的本质；且会降低避障分辨率。
4. **Setup 与运动重叠**：当前 Setup 完全静止；若允许换轮与运动重叠，会缩短 T_end，但改变 FR02 的静止重配置语义。

### 7.4 交互/场景层（低风险）

1. 修正 map04 自动起点（vehicle 2 碰撞），或让自动起点选择做静态检查后回退。
2. PLANNING 时排队/替换目标，而不是直接拒绝；至少给用户“正在规划”的反馈。
3. 规划失败时不要发空 playback envelope，消除 `model_sha256` ERROR 噪音。

---

## 8. 附录

### 8.1 关键在线日志摘录（map04）

```
plan 600000 admitted: start=(1.420, 22.597, 0.000) goal=(15.960, 17.545, 0.000)
plan 600000 not executable: status=REQUEST_REJECTED reason=MAP_OBB_COLLISION layer=request wall=0.027s
plan 600001 admitted: start=(4.704, 22.243, -0.016) goal=(11.763, 22.226, -0.011)
plan 600001 published: T_end=26.236 s wall=24.940 s samples=1320
plan 600002 admitted: start=(11.763, 22.226, -0.016) goal=(20.151, 22.179, -0.016)
plan 600002 published: T_end=28.657 s wall=26.039 s samples=1442
plan 600003 admitted: start=(20.151, 22.179, -0.016) goal=(22.086, 16.640, -1.547)
plan 600003 not executable: status=BUDGET_EXHAUSTED reason=TIME_BUDGET_EXHAUSTED layer=budget wall=32.055s
```

### 8.2 关键在线日志摘录（map05）

```
planner ready: backend=uniform_v02 budget=10000 expansions / 30.0 s
plan 600000 admitted: start=(-5.734, 9.638, 0.000) goal=(3.089, 3.411, 1.571)
plan 600000 not executable: status=BUDGET_EXHAUSTED reason=TIME_BUDGET_EXHAUSTED wall=32.352s
plan 600001 admitted: start=(-5.734, 9.638, 0.000) goal=(-0.284, 12.016, 0.044)
plan 600001 not executable: status=BUDGET_EXHAUSTED reason=TIME_BUDGET_EXHAUSTED wall=32.281s
goal bridge: refusing 2D Pose Estimate: job 600000 is PLANNING
goal bridge: refusing 2D Nav Goal: job 600000 is PLANNING
playback: model_sha256 must be 64 hex digits
```

### 8.3 600003 的 S+A crop 前缀复算命令（只读）

```bash
OPENBLAS_NUM_THREADS=1 \
PYTHONPATH=/home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/lib/python3/dist-packages \
python3 - <<'PY'
# 加载缩放后的 map04
# 构造 FormationModel / VehicleShape / FormationCollisionChecker
# s0 = initial_state(..., goal_qf=(22.086,16.640,-1.547))
# 直接取 S_to_A_arc_p0_0_pos_gear--+ 与 A_arc_crop_p0_0_neg_gear++-
# 对两条边做 checker.swept_check_analytic
# 结果：两条边都 OK，A 后 qf=(20.151,22.179,-1.547)
PY
```

完整可运行脚本在 `/tmp/fr_analysis_repro/repro_request.py` 和本次分析时的一次性脚本中；`/tmp` 可能被清理，正文已记录关键数值。

### 8.4 参数速查

```
SuccessorConfig:
  xy_bin 0.20 m
  psi_bin 10°
  heading_bin 10°
  control_bin 0.01
  arc_pivots (0,0), (0,±2), (±2,0)
  arc_step 10°
  translation_step 1.0 m
  allow_reverse True
  goal_heading_free True
  goal_xy_tol 0.02 m
  goal_psi_tol 1°
  sweep_spacing 0.01 m
  collision_margin 0.05 m

GraphSearchConfig:
  max_expansions 10000
  max_seconds 30.0
  heuristic_weight 3.0
  rotation_weight 3.0

FormationTimingLimits:
  v_max 0.8
  a_max 0.6
  jerk_max 2.0
  yaw_rate_max 0.8
  yaw_accel_max 0.8
  yaw_jerk_max 3.0
  wheel_speed_max 1.0
  steer_rate_max 0.8
  steer_accel_max 1.5
  mode_setup_time 0.15
  mode_switch_duration 0.3
  min_mode_dwell_s 0.3
  r_min 0.81
```