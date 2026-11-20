# E09 端到端迁移场景：不可复现报告（NOT_REPRODUCIBLE）

状态：**NOT_REPRODUCIBLE（按返工方案 §4 Step D / 原方案 line 129 停止该步骤）**
日期：2026-11-20（UTC 证据目录 20261119T180735Z）
工作树：`/home/wq/whut_thesis/expv3_multi_virtual_a`

## 1. 要求

`work_flow/TASK03_Codex返工执行方案.md` §4 Step D 要求：一次小规模、确定性、真实审计的相邻
`CorridorChain` 场景中，形成一条**真实几何可行路径**，使 F 在旧 `allocation` 固定指定的某段
region 之外，但始终属于该段允许的相邻已审核 Chain region；旧
`audit_path_in_corridors()` 明确失败并保存 failure station/region/residual；模板 F 行、三车 OBB
行与 continuous certificate 全部通过并绑定 request/grid/context/path/chain/template；其余
几何/动力学/原始地图 OBB/播放包络审计通过；最终 `SUCCESS`/`executable=true` 且
`f_only_gate_migrated=true`；`f_only` 在同一输入下保持旧行为。

line 129 同时规定：**如果无法构造这种场景，必须报告实际阻碍并停止该步骤；不得把当前
`f_only_gate_migrated=false` 的直线路径、模板早期拒绝或人为令旧 audit 失败作为 E09 通过
证据。**

本报告按此要求给出构造尝试、观测结果与结构性阻碍，并明确不伪造通过证据。

## 2. 场景搜索方法与证据

- 脚本：`e09_profiling.py`（本目录）。它只通过 `unittest.mock` 把
  `build_formation_template_candidates` 过滤到指定 profile，并捕获
  `certify_formation_path` / `_formation_sample_audit` 的输入输出用于观测；不修改生产
  约束、代价、阈值或模板构建逻辑。
- 场景入口：同一 `VirtualARequest`/`Grid`/`Route`/`GuideRoute` 由
  `make_route_guides` 的固定返回提供，`build_corridor_chain`、`audit_corridor_chain`、
  `make_corridor_guide`、`allocate_corridors`、`plan_virtual_a` 全部为生产实现。
- 矩阵：`e09_matrix.sh` 顺序运行 14 个确定性场景（`matrix/` 同名日志），再加 2 个障碍强制
  场景（`attempt_block_*.log`）。汇总见 `matrix_summary.md` / `matrix_summary.json`。
- 参数覆盖：45°/60°/90°/120° 转角、腿长比 8:20、12:24、18:18、20:8、24:12、18:30、
  padding 2.5/3.0/3.5/4.0、起始航向 0/45°、目标航向沿腿/0°、allocation/forward/backward
  三个模板 profile（forward/backward 为主，allocation 无迁移窗口）。

## 3. 观测结果（全部日志可复核）

| 项目 | 结果 |
|---|---|
| 捕获路径的 `WORST r_alloc`（F 相对**旧 allocation region** 的逐点残差最大值） | 有候选路径的 11 个场景全部 **≤ −1.255 m**（负值 = 在 region 内部），即 F 从未离开旧 allocation region；m08/m12/m14 未产生候选路径 |
| 旧 `audit_path_in_corridors()` | 所有捕获到路径的场景（11 个）全部 **passed=True / CORRIDOR_CONTAINMENT_VERIFIED**；`first_failure=None`；m08/m12/m14 无路径可审 |
| union certificate / sample audit | 没有任何一个场景产生 VERIFIED：UNCONFIRMED（区间未闭合）或 DEFINITE_VIOLATION（OBB 越界） |
| pipeline 最终状态 | 全部 `NO_ACCEPTED_CANDIDATE`；无 `SUCCESS`，无 `executable=true` |
| 障碍场景 `block(19.5,2.0)` | 链表可审核，但 adjacent region 被半空间收窄到三车足印无法覆盖：`COVER_TEMPLATE_INSUFFICIENT` |
| 障碍场景 `block(21.0,2.0)` | 有候选轨迹，但 F 相对 allocation 的残差仍为 −1.257 m，旧 audit PASS；certificate `DEFINITE_VIOLATION` |
| 唯一可复现的 union `SUCCESS` | `tests/test_virtual_a_pipeline.py::G4PipelineTests::test_union_scope...`（扩展网格直线 6 m）与 TASK03 Step 6 live `union_map05_2625`：**chain 只有 1 个 region**（allocation 全 0，模板单段 f=0），结构上不存在相邻 region，也不存在迁移窗口；live 记录 `f_only_gate_migrated=false` |
| 证书失败量级 | m01：区间未闭合，upper residual 7.4e-6 m；m05 两条候选的 definite OBB 越界为 8.9e-3 m 与 2.5e-4 m。连“不迁移的 union 路径”都未被证书接受 |

原始日志：`m01..m14`、`attempt_block_19p5_2p0.log`、`attempt_block_21p0_2p0.log`；逐行数据见
`matrix_summary.md`。

## 4. 结构性阻碍分析

### (a) 模板约束是单侧包含，不可能**强制**迁移

`FormationCorridorConstraint` 的每一行都是“该车完整 OBB（或 F 参考点）位于某个凸 region 内”
的合取。相邻 region 由已审核 Chain 提供，且 `R_alloc ∩ R_adj` 非空（portal 审核要求半径 >
阈值）。因此存在这样的轨迹类：把模板中分配到 `R_adj` 的车辆改为留在 `R_alloc ∩ R_adj` 内，
其余约束不变。对这类轨迹，所有模板行仍然满足，而 F 也不会离开 `R_alloc`。换言之：
**约束集允许迁移，但从不要求迁移**；是否迁移完全由代价函数/初值决定。

中点凸性引理：`F = (V2 + V3) / 2`。凸集同时包含 V2、V3 的中心则包含 F；所以 F 离开
`R_alloc` 的必要条件是至少一辆横向车的 OBB 离开 `R_alloc`。而模板对车辆的声明只是“在
`R_adj` 内”，并不排斥其同时位于 `R_alloc` 内。布局级 fixture 的 mixed-cover（V2∈R0,
V3∈R1）只在**窗口内**成立，仍不强制 F 越过 R0 的远边界。

### (b) allocation 切点与路径出口的次序

`allocate_corridors()` 的 region 切换发生在 guide 第一段长度占比
`tau = |start→portal| / L_guide`（再与 `k/8` 切点合并取区间中点判定）。guide 的 portal 是
相邻 region 交集的 Chebyshev 中心，经 QP 拉向 start-goal 弦，位于 `R_alloc` 的边界上或内部，
且沿 guide 一定在 `R_alloc` 出口之前。

长度/曲率最优的 C2 平滑路径在拐角处内切：它比 guide 更靠 `R_alloc` 内部，到达 `R_alloc`
边界时的归一化弧长晚于 `tau`。用观测数据量化（`L_path/L_guide` 为路径相对 guide 的长度比）：

- m01：`L_guide=46.085`，`L_path=49.541`，比 1.075；allocation 在 `tau=0.3842`
  切到 region 1。路径直到 `sigma=0.3607`（切点前）仍在 `R0`（y≤3）内
  2.281 m 深处（F=(17.833, 0.719)）；要在此前离开 `R0`，路径必须再向外多走
  约 2.3 m 的纵向行程，而长度/曲率代价正阻止这种外摆。
- m03：`L_guide=30.594`，`L_path=36.798`，比 1.203；`tau=0.5`。整条路径相对
  “当前 sigma 的 allocation region” 的最大残差为 −1.257 m（发生在 `sigma=0.6393`，
  F=(19.743, 4.955)）；即 F 从未接近离开其 allocation region 的边界。
- 实测所有产生路径的场景 `WORST r_alloc ≤ −1.255 m`，说明路径从未接近 `R0` 边界，更未穿越。

若为了把路径“外摆”到 `R_adj` 而增加长度，长度/曲率/参考项会反对；而且这些拐角场景在
OBB 掠界（1e-6..1e-4 m）处**先**被 continuous certificate 拒绝，根本到不了“accepted
path”阶段。

### (c) 障碍分离半空间对两个 region 等量收窄

`_build_region()` 对每个占用格加入半空间，其界限位于 seed 投影与障碍投影的中点。因此，
把 `R_alloc` 切到足以让 route 提前离开的障碍，同时把 `R_adj` 收窄同样的支撑距离。实测：

- 障碍 `(19.5, 2.0)`：`R1` 被切到 x≤18.65，三车侧向足印（x±1.31）无法覆盖，模板构建返回
  `COVER_TEMPLATE_INSUFFICIENT`；
- 障碍 `(21.0, 2.0)`：三车可覆盖，但 `R0` 的切点在 route 下游（F 残差 −1.257 m），旧 audit
  仍 PASS，且 certificate 因 OBB 掠界 `DEFINITE_VIOLATION`。

因此障碍强制无法在“模板可覆盖 + 证书可验证”的前提下把出口提前到 `tau` 之前。

### (d) 布局级 fixture 的窗口在生产 pipeline 中不可达

`_portal_scene(overlap_half=0.3)` 的 forward 模板 F cut = 0.369 < allocation cut 0.5062，
但直线 guide/路径在 x=0.3 处的 `R0` 出口在 sigma≈0.634；要让路径在 0.5062 前离开 `R0`，
必须由轨迹优化器主动外摆到 overlap 的外侧。6 个 row-level 测试证明 union 行确实按模板 F
region 编译、而不是按 allocation（`tests/test_virtual_a_formation_corridor.py::
FormationFRegionReworkTests`），但生产代价不会选择这种外摆路径。

## 5. 结论与处置

1. **E09 无法在本次返工授权的改动范围内构造**：不是搜索不足，而是当前 corridor builder +
   轨迹优化器 + 证书判据的组合不产生“证书 VERIFIED 且旧 audit 失败”的路径。任何进一步
   尝试都需要改变代价/参考/初值或 allocation 语义，属于设计变更，超出返工方案允许的
   “不新增搜索、不改阈值/边距”的固定契约。
2. 按 line 129，E09 记为 **NOT_REPRODUCIBLE**，该步骤停止。
3. 不提交伪造证据：本目录不含任何 `f_only_gate_migrated=true` 的运行；live union
   `union_map05_2625` 为单 region，`f_only_gate_migrated=false`；未把模板早期拒绝
   （`COVER_TEMPLATE_INSUFFICIENT`）或人为构造的旧 audit 失败包装成 E09 通过。
4. Step B/C 的代码与 row-level 证据仍然完整保留（union NLP 源自模板 F region、continuous
   certificate 同一模板、`f_only` 兼容性），供 Codex 独立判断；若需真正的 E09，建议下一步
   先由用户/Codex 决策是否允许改变代价或 allocation 语义（例如：将 allocation 由 guide
   portal 占比改为“路径自身区域切换点”或引入区域切换阶段的显式外摆参考）。

## 6. 复现方式

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD:$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e09/e09_profiling.py \
  --a 18 --b 18 --deg 90 --pad 3 --only forward
# 14 场景顺序重跑：
bash work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e09/e09_matrix.sh
python3 work_flow/TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e09/summarize_matrix.py
```

注：`e09_matrix.sh` 中的输出目录是 `/tmp/e09_sweep2`（诊断脚本的原始路径）；若在原处重跑，
新日志写入 `/tmp/e09_sweep2`，本目录保存的是本次运行的逐字日志。