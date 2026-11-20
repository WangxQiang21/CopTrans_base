# TASK03 返工 02 Claude 交付：E09 门控迁移验收

**结论：`BLOCKED_DESIGN_DECISION`。E09 = `NOT_REPRODUCIBLE`（在方案规定的 30 个冻结组合内）。**
本返工未修改任何生产代码、测试代码或默认配置；未 commit、未 push；未开始 TASK04。

- 基线 HEAD：`a3c709e57560b1060dcc422777f4131a3e558acf`
- 分支：`feature/virtual-rigid-formation-a`
- 证据根目录：`work_flow/TASK03_REWORK_02_evidence/20261120T025148Z/`
- 机器可读结果：`results.json`；摘要：`SUMMARY.md`；矩阵：`scan/scan_matrix.json` / `.md`

---

## 1. 代码事实（本次复用的是既有实现，未改动）

1. 生产入口 `plan_virtual_a()` 在 union scope 下依次执行：A* route → `simplify_route()` → `build_corridor_chain()` → `audit_corridor_chain()` → `make_corridor_guide()` → `allocate_corridors()` → `assess_formation_guide()` → `build_formation_template_candidates()` → NLP → `certify_formation_path()` → 旧 `audit_path_in_corridors()` → migration 分支。
2. `assess_formation_guide()` 要求每个 station 的 `map_status == CLEAR_AT_SAMPLES`；任一 station 为 `COLLISION_OBSERVED`，模板构建直接抛 `FORMATION_CORRIDOR_UNCONFIRMED`，不会进入模板/NLP（`formation_corridor.py:1766-1770`）。
3. 三车模型（`tests/virtual_a_g4_support.py`）：`V1` 偏移 `(1.0, 0)`，`V2` `(0, +1.0)`，`V3` `(0, -1.0)`，每车 OBB `0.738 × 0.500 m`（半长 0.369、半宽 0.250）。编队整体绕 F 的包络约 2.7 m × 2.5 m。map 碰撞 margin 为 `CollisionConfig.baseline_margin_m = 0.05`，不得放宽。
4. 路线搜索参数在 `pipeline.py` 内固定（`G4_ROUTE_CLEARANCE_WEIGHT=2.0`、`blocked_radius_m=0.15` 等），本次未改。

## 2. 实验事实（Step 1 失败层诊断）

**扫描预算：30/30 个冻结确定性组合全部用满**（外加 2 次同组合的重复诊断 s02b/s03c，不计为新组合；manifest 见 `scan/combos.json`、`scan/batch_0{1..4}_*.json`）。每个组合都走完整生产 `plan_virtual_a`；扫描器只包裹 `certify_formation_path` / `assess_formation_guide` / `make_route_guides` 观测真实输入输出，不改变任何返回对象、模板、allocation、证书或审核结果。最终结论不依赖任何 monkeypatch。

| 失败层 | 组合数 |
|---|---|
| 路线生成失败（`NO_ROUTE_GUIDE`） | 1（s01，大网格 A* 超限） |
| 单 region chain（无相邻区） | 2（s04、s06） |
| **两个真实相邻 region + Portal 审核通过，但 assessment 三车地图 OBB 碰撞** | **27** |
| 模板构建成功 | **0** |
| NLP 求解 / 候选捕获 | **0** |
| 连续证书 | **0** |
| 旧 allocation audit 对最终路径失败 | **0**（不存在最终优化路径） |
| `f_only_gate_migrated=true` | **0** |

- 27 个双 region 组合的 allocation 切点在 `0.4591–0.9357`，chain/Portal 均通过既有 `audit_corridor_chain()`。
- 24 个组合带完整 instrumentation，assessment 碰撞 station 数为 13–190；全部在 `TEMPLATE_BUILD_FAILED` 停止。
- 全部 30 个组合最终 `NO_ACCEPTED_CANDIDATE / FORMATION_CORRIDOR_UNCONFIRMED`，`executable=false`。
- 关键几何：所有能诱导出真实双 corridor 的障碍布局，都会让编队前车 V1 在转弯/跨越阶段扫入障碍；降低墙高（最低到 0.5 m）、改变墙宽（0.1–3.0 m）、改变转角（0°/10°/20°）均未改变这一结论。代表结果：s30 的 V1 在 `sigma=0.3171` 处压墙；s19 的 V1 在 `sigma=0.3488` 起连续压墙 117 个 station。

**独立 witness 复算（真实碰撞，不是 margin 误报）**：`scan/witness_s30.json` 不使用生产碰撞代码，直接用三车模型偏移与该冻结地图的墙格做 SAT：
- station `sigma=0.317073`，`s_m=6.407694`，F 位姿 `(6.387246, 0.511506, 0.079912)`；
- V1 中心 `(7.3841, 0.5913)`，对墙体**裸 OBB 穿透 0.1077 m**；
- V2/V3 在该 station 无碰撞。

因此 assessment 的拒绝是真实安全违规。按返工方案 §4 Step 3，此类候选"确实不安全，不得修证书或调 margin 使其通过"，只能改 fixture/route 几何；而 30 个冻结组合已用尽。

## 3. 技术推断（未作为结论上报）

1. 现有模板对 F region 的约束是**单侧包含**：允许但不要求优化路径离开旧 allocation region；本次连模板构建层都未到达，因此无法验证"优化器是否会自然迁移"。
2. 在当前编队尺寸（±1.0 m 横向、+1.0 m 纵向）下，要让平滑 F 路径在 allocation 切点前穿出旧 region 且全程满足三车 OBB，需要比本轮所有障碍布局更大的净空或不同的切换语义（例如按实际路径 crossing 定义切点）。这支持方案 §7 中列出的设计决策方向，但本报告不替用户/Codex 选择。
3. 由于 30/30 无正例，E09 的 8 条合同（尤其是第 3–7 条：接受模板的 region 迁移、旧 audit 对最终路径失败、F/OBB rows 真实进入 NLP、连续证书 VERIFIED、`f_only_gate_migrated=true`）均未得到演示；任何 PASS 声明都会与证据不符。

## 4. 为什么停在 BLOCKED_DESIGN_DECISION

方案 §7 规定：30 个确定性 fixture 仍无符合合同的场景，或只能靠改目标函数/allocation 语义才能得到迁移时，必须交付 `BLOCKED_DESIGN_DECISION` 并停止，不得自行选择研究路线。本轮正好触发该停止条件：

- 30/30 用尽，0 个场景进入模板/NLP/证书/迁移层；
- 失败原因是真实三车 OBB 碰撞，放宽它会违反禁用条款；
- 继续制造正例只能改目标函数、allocation 切点或 region 语义——三者均须用户与 ChatGPT 决策。

**需要用户与 ChatGPT 决策的最小选项（Claude 未选择）：**

1. 保留当前 allocation 定义，通过有研究依据的初值/目标改进让合法相邻模板可被优化器找到；
2. 改为按实际优化路径的 region crossing 定义切换时刻（会改变 allocation 语义并涉及模板/优化耦合）；
3. 保留旧 F-only hard gate，明确放弃本阶段 gate migration/E09 目标。

## 5. 交付物清单

`work_flow/TASK03_REWORK_02_evidence/20261120T025148Z/`（143 个文件，manifest 校验通过）：

- `SUMMARY.md`、`results.json`（机器可读结论与失败层计数）
- `scan/combos.json`（30 组合冻结清单）、`scan/batch_0{1..4}_*.json`
- `scan/scan_e09.py`（生产管线扫描器）、`scan/run_batch_0{1..4}.sh`
- `scan/s01..s30.log` + `*.meta`（每次运行的 SUMMARY/退出码/耗时）
- `scan/s03c.assessments.json`、`scan/s07..s30.assessments.json`、`*.route_sets.json`
- `scan/scan_matrix.json` / `scan_matrix.md`（30 行失败层矩阵）
- `scan/commands.json`（逐次 argv/exit/elapsed）、`commands.log`
- `scan/verify_witness.py` + `scan/witness_s30.json`（独立碰撞复算）
- `scan/summarize_scan.py`
- `start/`（HEAD/分支/status、19 个 TASK03 源文件 SHA、保护文件 SHA、源码快照）
- `protected_checks.txt`（返工前后保护哈希核对）、`git_state.txt`、`source.diff`（0 行）
- `make_manifest.sh`、`manifest.sha256`、`manifest_summary.json`

**未交付**：`tests/test_virtual_a_e09.py`、E09 fixture、ROS/RViz E09 live 证据。原因：不存在满足合同的 E09 输入；方案禁止用离线单测冒充 live 通过，也禁止伪造正例。REWORK_01 的 live 证据保持原样未动。

## 6. 完整性与范围核对

- `source.diff` 为空（0 行）：未改任何生产/测试源码。
- 19 个 TASK03 源文件 `sha256sum -c` 19/19 OK。
- `geometry.py` = `f31b20bb…7efc3d`、`tests/test_virtual_a_geometry.py` = `4f59e68f…b4c7`，与保护值一致。
- `evaluation/` git tree = `c2fa3efa…d5ae1`；TASK01/02/03 与 REWORK_01 evidence tree 哈希全部与 start 一致。
- `git status --porcelain` 仍为 32 条既有条目（未新增/暂存/提交）；未触碰 `11411` master；未运行会覆盖历史 evidence 的 discover。
- 未 commit、未 push、未开始 TASK04。

**停止点**：等待 Codex 独立验收，以及用户/ChatGPT 对第 4 节三个方向的决策。