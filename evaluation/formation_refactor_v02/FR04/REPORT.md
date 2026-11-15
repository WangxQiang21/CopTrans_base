# FR04 有预算自主规划基线：实施报告

日期：2026-11-14。工作区：`/home/wq/whut_thesis/expv3_multi_v02`（V0.2 worktree）。本阶段只做离线自主搜索＋FR02 时间/审核，不涉及 ROS/RViz、NLP、控制器或动力学，不提交 Git，不进入 FR05。

## 1. 范围与文件

| 文件 | 状态 | 说明 |
|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py` | 修改（仅追加） | 新增 `import heapq`/`import time` 与 FR04 搜索段：`GraphSearchConfig`、`UniformGraphSearchResult`、`reconstruct_path`、`path_records`、`goal_heuristic_s`、`UniformGraphSearch`。FR03 已验收字节（`982b2f96…f96e83`）是当前文件的精确前缀；见第 8 节。 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_pipeline.py` | 新增 | 薄管道：请求校验 → 保持/搜索 → FR02 终端审核 → 重建路径再审核。 |
| `tests/test_uniform_pipeline.py` | 新增 | F01–F12，共 12 项。 |

未改动 FR01/FR02 源码与测试、`tests/test_uniform_search.py`、冻结文档、车辆/轮系/碰撞/安全限值，也未新增后端模式重选、参考 seed、坐标特判或每用例枢轴表。

## 2. 搜索实现

- 状态：直接使用 FR03 `UniformSearchState`（精确 `q_F`、精确车头、mode、上一轮系签名、`a_labels`、父边）。不吸附、不瞬移。
- 代表表：每个请求/目标/配置一个 `RepresentativeTable`；完整 key 相同才比较 g，且只在同一目标下使用。
- open/closed：`heapq`，优先级 `f = g + heuristic_weight·h`，并列用 g 与插入序号打破；弹出时用 `table.representative(key) is not state` 丢弃过期条目并计数 `stale_heap_entries`。
- 扩展：全部来自 `SuccessorGenerator.generate(state, goal_qf)`（FR03 后继），每条边是 FR01 权威 A/O/S 单边，含 mode、车头、设置签名与实际解析扫掠。
- 启发：`h = XY 距离/v_max + rotation_weight·|wrap(goal_psi - psi)|/yaw_rate_max`；默认 `heuristic_weight=3.0`、`rotation_weight=3.0`，全请求共用，不按用例调整。**不是可采纳下界**，加权 A* 与旋转权重都不构成最优性声明。旋转权重只用于避免必要的货物转向被大量零位移动的 S/航向替代长期压后。
- 预算：默认 `max_expansions=10000`、`max_seconds=30.0`（`GraphSearchConfig`），每次循环检查；终审通过后还检查终审完成时刻是否已超时，超时不得以“审核已通过”绕过时间预算。
- 终端：`terminal_validator(edges)` 由管道注入真实 FR02（`UniformTimedPlan.from_edges` + `sample` + `audit_uniform_timed_plan`）。被拒路径写入 `terminal_audits` 与 `terminal_rejection_histogram`，搜索继续；无 validator 时返回 `AUDIT_REJECTED`，绝不返回未审核的几何终点。
- 状态：`SUCCESS` / `NO_MOTION` / `NO_SOLUTION` / `BUDGET_EXHAUSTED` / `AUDIT_REJECTED`，另有管道层 `REQUEST_REJECTED`。
- 冗余消除：跳过连续 S→S。理由不是新增限制：S 目标只由 `q_F` 与目标决定，FR01 可直接从第一段车头集合构造一条到第二目标的 S 边（含零角车），所以 S→S 链不会解锁单条 S 解锁不了的 A 能力。所有被跳过的边计入 `redundant_s_pruned`。

## 3. 管道实现

`UniformPlanningRequest` 固化 grid、model、vehicle shapes、精确起点/车头、货物目标、可选目标车头、plan/formation ID、`SuccessorConfig` 与 `GraphSearchConfig`；`request_fingerprint` 对栅格占用字节、分辨率/原点、offset、车形、位姿/车头与两份配置做规范 JSON 哈希。

`plan_uniform(request)` 顺序固定：

1. 起点静态 OBB/map 检查（失败 → `REQUEST_REJECTED`，layer `request`）。
2. 任一刚体车辆中心越出地图 → `REQUEST_REJECTED / GOAL_CENTER_OUT_OF_BOUNDS`，不进入搜索。
3. 若显式给出目标车头，再对目标做静态检查；默认目标车头自由，车体终端不施加占位角。
4. `initial_state(..., goal_qf=实际目标)` 正确生成同目标 `a_labels`，不跨目标复用代表表。
5. 搜索；`SUCCESS` 时按父链重建边序列并**再次**构造、采样、审核；两次审核都通过才 `feasible/executable=True`。`NO_MOTION` 时构造显式空边保持（时长 0、一个 t=0 样本）并交 FR02 审核。

## 4. 八项规定请求结果

地图 30 m×30 m、0.05 m、原点 (-15,-15)、空图；起点 `(10,10,0)`，三车初始车头均为 0。默认预算 10000/30 s。`T_end` 是 FR02 轨迹时长，不是求解墙钟。

| 用例 | 请求 | 状态 | 扩展 | 求解墙钟 s | T_end s | 路径原语（含枢轴/方向要点） |
|---|---|---|---|---|---|---|
| F01 | 原地保持 | `NO_MOTION` | 0 | 0.000012 | 0.0 | 空边保持，1 个 t=0 样本 |
| F02 | `(13,10,0)` | `SUCCESS` | 3 | 1.157 | 9.368306005451 | A 直行×3 |
| F03 | `(7,10,0)` | `SUCCESS` | 3 | 1.050 | 9.368306005451 | A 直行×3，三车 direction=-1 |
| F04 | `(10,13,0)` | `SUCCESS` | 9 | 2.773 | 13.199859896376 | O 平移×3 |
| F05 | `(13,12,0)` | `SUCCESS` | 6 | 1.949 | 18.551871641156 | O 平移×5 |
| F06 | 原地 `+90°` | `SUCCESS` | 27 | 8.088 | 30.441497053344 | S(0.4636) → A 弧 pivot `(0,-2)`，+π/2 → O×3 |
| F07 | 原地 `-90°` | `SUCCESS` | 27 | 8.242 | 30.441497053344 | S(0.4636) → A 弧 pivot `(0,2)`，−π/2 → O×3 |
| F08 | `(13,10,π/2)` | `SUCCESS` | 31 | 9.577 | 39.809803058795 | A 直行×3 → S → A 弧 pivot `(0,-2)`，+π/2 → O×3 |

全部 8 例的独立复审核均为 `FEASIBLE`：

- 刚性最大误差 ≤ 2.3e-15 m；
- 内部几何/状态误差 ≤ 5.4e-15 m 或 rad；
- 模式归一化残差 0；有限差分最大 2.6e-5（阈值 1e-4）；
- 动态峰值比 0.9868–0.9950（≤ 1+1e-3）。

F12 用同一前进请求包裹管道的审核入口，确认 `audit_uniform_timed_plan` 恰被调用 2 次：1 次终端候选审核 + 1 次重建路径终审；两者均接受。

## 5. 终端拒绝与负例（如实保留）

### 5.1 原地旋转的零枢轴直达路径被拒

F06/F07 在找到可行绕行前各有 2 次终端审核拒绝（候选路径 `edge_count=2`）；F08 有 2 次（`edge_count=5`，A 直行×3 + S + 零枢轴 A 弧）。失败码与数值相同：

```
UNIFORM_DYNAMIC_UNCONFIRMED / constraint=acceleration
candidate peak 0.594044701864 < limit 0.6
adopted full-interval bound 0.602047757911 > limit
analytic 1.13642756147, refined 0.6020477579113579,
max adjacent time gap 0.00493921822021 s
```

审核器明确写明“这不是已证明的物理超限”。FR04 **没有**修改 FR02、没有放宽 0.6 m/s²、没有把该拒绝隐藏为成功；搜索记录拒绝后继续，改走非零枢轴 A 弧 + O 回移，最终路径通过完整审核。

### 5.2 障碍无解

F09 在起点周围三簇障碍（中心 `(11,10)`、`(10,11)`、`(10,9)`，各 ±0.45/±0.35 与四个对角点）下：起点/目标静态通过，第一次扩展 143 个候选、0 个被接受，拒绝分布 `MAP_OBB_COLLISION=56`、`A_HEADING_REQUIRES_S_ALIGNMENT=87`，open 集耗尽 → `NO_SOLUTION`，layer `search`。这是有限候选库在该图中的真实无解结论，不冒充全图数学无解。

### 5.3 越界拒绝

F10 目标 `(16,10,0)` 超出 30 m 地图，管道在任何搜索前返回 `REQUEST_REJECTED / GOAL_CENTER_OUT_OF_BOUNDS`，无 plan、无 search 结果。

### 5.4 故意预算耗尽

F11 前进请求将 `max_expansions` 显式降为 1：`BUDGET_EXHAUSTED / EXPANSION_BUDGET_EXHAUSTED`、layer `budget`、扩展数 1、无 plan。该例只用于验证预算状态，不计为物理无解；默认 10000/30 s 未被改动。

### 5.5 预算瓶颈诊断（按计划 §8 要求）

成功用例的计数如下，未出现超时或预算耗尽：

| 用例 | 扩展 | 候选 | 接受后继 | 同键拒绝 | S→S 剪枝 | 过期堆条目 | 生成 s | 扫掠 s |
|---|---|---|---|---|---|---|---|---|
| F02 | 3 | 429 | 168 | 0 | 0 | 0 | 0.214 | 0.786 |
| F04 | 9 | 1302 | 510 | 0 | 0 | 0 | 0.532 | 2.081 |
| F06 | 27 | 4367 | 600 | 11 | 780 | 0 | 1.680 | 5.898 |
| F07 | 27 | 4390 | 616 | 18 | 780 | 0 | 1.712 | 6.015 |
| F08 | 31 | 5208 | 684 | 101 | 780 | 0 | 2.006 | 6.709 |

热点是解析扫掠（约占搜索墙钟一半以上）与 S 方案候选量；没有状态爆炸、没有错误的终端条件导致的空转。单次终端审核耗时 0.08–0.49 s（F06/F07 拒绝约 0.08 s、接受约 0.34 s；F08 约 0.18/0.49 s；F02 约 0.16 s），不是搜索墙钟的主要部分。默认预算下最慢成功例（F08）9.6 s 搜索墙钟、31 次扩展，距 30 s/10000 次仍有余量；本轮**没有**为通过而调整预算、模型或审核。

## 6. 已知限制（不隐藏、不顺手修）

1. FR03 记录的默认固定 ±10° 非零枢轴 A 弧仍会在 FR02 返回 jerk 类 `UNIFORM_DYNAMIC_UNCONFIRMED`。本轮成功路径使用的是 ±90° 非零枢轴弧与 O/A 直行，未触发该限制；该限制依然存在，FR04 没有处置，也没有据此放宽限值。
2. 上述零枢轴 ±90° 直达路径的加速度全区间保守界拒绝只说明“未确认”，不是物理超限证明；采样仍是有限点，不是连续证明。
3. 加权启发与旋转权重只用于排序，不构成最优性、完备性或最短路径声明；`NO_SOLUTION` 只针对该地图、该候选库与该预算。
4. A-label 的有限能力证明仍只覆盖默认 `allow_reverse=True`；本轮沿用默认配置。
5. 成功只证明离线 FR02 审核通过；没有 ROS 发布、RViz 图形回放、消息封装、控制器或动力学验证。
6. 终端审核被拒的终点状态不会继续扩展：搜索从其余 open 状态继续，且同 key 代表规则也通常会阻止以更大 g 回到同一 key。因此“到达审核被拒的终点、再离开并从另一前驱边返回”这类解不在本轮保证范围内，不能把本实现当作完备枚举。F06/F07/F08 的可行解来自其他中间状态，不依赖扩展被拒终点。
7. 目标由 FR03 验收合同判定为 `0.02 m / 1°` 容差内，管道报告的是构图目标的位姿；不是“精确等于请求坐标”的更强证明。

## 7. 测试与回归

| 项目 | 结果 |
|---|---|
| FR04 `test_uniform_pipeline.py` | 12/12 通过，38.3 s |
| FR03 `test_uniform_search.py` | 12/12 通过，159.7 s |
| 十个既有相关套件 | 145/145 通过（17+15+14+28+22+14+13+9+5+8） |

首次运行失败（12 项均因 `FormationModel.centers()` 返回字典被误按序列解包）与一次测试断言错误（把倒车误写为负参数长度）的原始日志保存在 `failures/`，未被覆盖或删除。第一次 12/12 通过但尚未把终审耗时计入时间预算的整轮结果保存在 `previous_runs/round1_pre_terminal_budget_fix/`；源码哈希键改为仓库相对路径前的整轮结果保存在 `previous_runs/round2_final_before_path_keys/`。所有轮次目录保留原始 `cases/`、`pipeline_results.json` 与日志。

环境：Linux 5.15.0-139，Python 3.8.10，numpy 1.24.4，scipy 1.10.1，16 核；所有计时为本机单进程、`OPENBLAS_NUM_THREADS=1`。

## 8. 基线与证据

`verify_baseline.py` 直接比对 FR01/FR02/FR03 验收记录中的源码哈希与冻结文档哈希，并验证 FR03 的 `uniform_search.py` 在当前文件中仍可按“去掉两条 FR04 import + 追加段”精确重建：

- `all_accepted_match=true`、`all_frozen_match=true`；
- 重建 SHA-256 = `982b2f96e1ab6dfcb3008d1e7232432bd861ebe8c8b9376e65bfbd96f8f96e83`；
- 当前文件相对 FR03 只多 2 条 import、2 行分隔空行和 315 行追加段。

证据文件：

- `pipeline_results.json`：12 例的请求/地图/配置哈希、预算、路径原语、搜索计数与拒绝分布、model、样本、墙钟、T_end、独立复审核；
- `cases/*.json`：逐例同样的完整记录（便于单独审阅）；
- `baseline_verification.json`、`verify_baseline.py`；
- `diff/uniform_search.py.diff`、`pre_fr04/`（重建的已验收 FR03 源及其哈希说明）；
- `regression/*.log`、`logs_fr03_tests.log`、`logs_regression_suites.log`、`test_uniform_pipeline.log`；
- `failures/`、`previous_runs/`；
- `COMMANDS.md`、`command_manifest.json`、`evidence_manifest.json`。

本轮到此停止，等待 Codex 独立验收；未提交 Git，未开始 FR05。

## 9. FR04-R1 返工（2026-11-14）

Codex 复核指出：`_build_and_audit` 用 `UniformAuditRequest.from_plan(plan)`，审核请求的
ID、起终点、模型、车形、限值和 `allow_reverse` 全部由待审 plan 回填，第二次重建被替换成
另一条自身可行轨迹时仍会给出 `SUCCESS/executable`。本轮只改
`uniform_pipeline.py`、`tests/test_uniform_pipeline.py` 和 FR04 证据。

修复后先按原请求与权威边快照建立合同，再构造/审核：请求字段全部来自原请求；最后一条边
的精确终点先按冻结 0.02 m/1° 容差核对原请求，再作为独立 `UniformAuditRequest` 的精确终点；
逐条比较 `plan.edges` 与权威边序列，禁止同端点换路径；默认自由终端车头不加相等要求。
失配返回结构化 `REJECTED`，不抛未处理异常。

新增 F13–F19：容差内目标+自由车头的正例、容差内零运动保持正例，以及错误起终点重建、
plan ID 错配、限值错配、同端点不同路径、同边数不同控制五个负例。最终 19/19 通过（44.782 s），
FR03 12/12（158.544 s），十个回归套件 145/145，基线核验全为 true。Codex 探针修复后为
`AUDIT_REJECTED / PLAN_START_MISMATCH`、不可执行；失败后新请求 ID=402 正常成功。

完整返工报告、原始日志、失败轮次与探针输出见 `rework_01/REWORK_REPORT.md`。
需要说明：修复前一次漏设证据目录的冒烟运行覆盖了本目录 `pipeline_results.json` 与
F01/F02 两个逐例文件；被覆盖后生成的 2 例文件保存在 `rework_01/accidental_overwrite/`，
原文件哈希仍在 `rework_01/pre_rework_evidence_manifest.json`，但其字节内容无法恢复。
其余既有逐例结果与日志未覆盖；修复后的完整 19 例结果在 `rework_01/final/`。

本轮未提交 Git，未进入 FR05，到此停止等待 Codex 复核。