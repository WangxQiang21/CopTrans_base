# TASK03 返工方案 02：完成 E09 门控迁移验收

**执行人：Claude Code**  
**独立验收：Codex**  
**工作区：** `/home/wq/whut_thesis/expv3_multi_virtual_a`  
**分支 / 基线 HEAD：** `feature/virtual-rigid-formation-a` / `a3c709e57560b1060dcc422777f4131a3e558acf`

本任务只处理 TASK03 的 E09 未完成项。不得重做已修复的航向区间和 F 区域模板，不得进入 TASK04，不得提交或推送 Git。开始前核对 HEAD、分支和工作区；现有未提交改动及历史证据均须原样保留。

## 1. 当前问题与证据

### 已确认事实

1. **P0 航向区间代码已修复。** `formation_certificate.py::_heading_interval()` 用向外舍入的区间乘法 `[κ] × [0, Δs]` 包含整段部分曲率积分，避免同号曲率时漏掉区间起点。
2. **P1 F 区域归属已经接入。** 模板段携带 `f_region_index`；union 优化器根据模板生成 F 硬约束，连续证书依据同一字段验证 F。旧 `f_only` 约束语义保留。
3. **E09 仍未验收。** `TASK03_REWORK_01_evidence/20261119T180735Z/step_d_e09/matrix_summary.md` 中，尝试的 14 个确定性场景及 2 个障碍场景均未产生 E09 所需结果。已捕获的 11 条候选轨迹都通过旧 F-only containment；连续证书为 `UNCONFIRMED` 或 `VIOLATED/DEFINITE_VIOLATION`。没有最终 `SUCCESS`、`executable=true` 或 `f_only_gate_migrated=true`。
4. 现有成功 union 直线例只有一个 corridor region，没有相邻区域和可迁移窗口，不能作为 E09 证据。

### 当前技术判断

【代码事实】现有 `allocation / forward / backward` 三种有限模板 profile 能选择相邻 F region；NLP 和连续证书能按模板 region 约束 F。相关实现位于 `formation_corridor.py::build_formation_template_candidates()`、`optimizer.py::_build_corridor_constraint()` 和 `formation_certificate.py`。

【实验事实】当前试过的路径没有离开其旧 allocation 区域，且证书没有接受候选。障碍强制场景分别出现模板覆盖不足或真实 OBB 违规。

【技术推断】模板目前“允许”F 在部分 sigma 段归属相邻 region，但并不要求优化后的 F 离开旧 allocation region。模板由 Guide witness 生成，实际优化轨迹仍可能留在两区重叠部分；现有目标函数也没有专门奖励跨出旧 region。该推断解释了当前矩阵，但**不能证明**所有满足现有合同的场景都无法产生迁移。

【尚未验证】是否能在不改目标函数和安全阈值的前提下，通过一个真正受障碍约束的双 corridor 场景，使生产优化自然产生并连续认证 E09 路径。上一轮有限参数扫描不足以排除这种可能。

## 2. 返工目标

构造并保存**一个**可复现的 E09 端到端正例，证明新 union 合同能安全接受一条旧固定 allocation 审核会拒绝的真实规划轨迹；同时用相同输入证明 `f_only` 仍保留旧语义。

E09 是行为验收，不是放宽安全门槛的指标。任何硬约束、碰撞 margin、连续证书界、时间/动态门槛、发布门槛都不得为制造正例而放宽。

## 3. 必须满足的 E09 验收合同

正例必须由真实生产调用链生成，不得人工改写审核结果或伪造轨迹。至少满足：

1. 使用同一 `VirtualARequest`、Grid/map、起终点、车辆模型及除 `formation_corridor_scope` 外完全相同的规划参数；分别运行 `three_vehicle_union_v1` 和 `f_only`。scope 是显式版本化 context 输入，所以两种运行的 `planning_context_sha256` 可以且通常应不同；不得伪称 hash 相同。两边各自绑定正确 context，并记录 route/guide/chain/allocation 的实际身份；为使对照有效，应由相同生产入口确定相同 route/guide/chain，若 scope 导致不同则需解释并证明差异不是通过更换场景制造。
2. Chain 至少有两个真实相邻 region，Portal 通过现有 `audit_corridor_chain()` 审核；地图、车辆模型、起终状态及 margin 均为有效输入。
3. union 的生产模板集合中确实包含相邻 F region 赋值。最终被接受模板的某段 `f_region_index` 与同一 sigma 的旧 allocation region 不同；不得通过 mock 过滤模板、人工替换模板或手写 cert 来达成。
4. 对**最终优化路径**原样运行 `audit_path_in_corridors()`：结果必须为失败，并保存首个失败的 `sigma/s_m`、allocation region、半空间和 residual。不得 monkeypatch audit、伪造失败结果或只对 Guide/初值做审计。
5. 同一条路径的 union F rows 和三车完整 OBB rows 均实际进入求解约束；F rows、OBB rows 数量分别大于零，数值残差最大值 `<= 1e-8 m`。CasADi / 数值 / SciPy fallback 对同一决策向量的约束值差异不超过 `1e-8 m`。
6. 独立连续 F+OBB 证书状态为 `VERIFIED`，所有闭合区间 upper residual `<= 0`、未闭合区间数为 0。template、request、grid、context、path、chain 身份绑定一致。
7. 后续 geometry、dynamics、原地图连续 OBB collision、timing 和 playback envelope 独立验证全部通过；最终 union 结果为 `SUCCESS`、`executable=true`、`f_only_gate_migrated=true`，并记录失败的旧审核与通过的新证书。
8. 相同研究输入的 `f_only` 运行不得消费相邻 union 模板来绕过旧 gate；其默认值、模型/context identity 和历史序列化合同不变。union 使用自身包含 scope 的 context identity。保存两种 scope 的完整对照结果。

若没有全部满足，E09 状态必须为 `NOT_REPRODUCIBLE` 或具体失败状态，不得报告 PASS。

## 4. 实施步骤

### Step 0：冻结与保护

- 保存 `git rev-parse HEAD`、分支、`git status --porcelain`、相关源码和测试 SHA-256。
- 记录 TASK01 的 `geometry.py`、`test_virtual_a_geometry.py`、TASK02/TASK03 历史证据和 `evaluation/` 状态；不得覆盖。
- 返工证据写入新的 `work_flow/TASK03_REWORK_02_evidence/<UTC时间戳>/`。

### Step 1：按实际失败阶段诊断，不先改算法

对上一轮场景及新场景，为每个模板候选保存一行机器可读记录：

`route/guide/template id → f_region_by_segment → F rows / OBB rows → solver 状态及残差 → 连续证书状态/首个 witness → 旧 allocation audit → map OBB → timing → playback → 最终状态`。

判明候选停在哪一层：没有相邻 F profile、候选未被尝试、NLP 未找到可行解、连续证书确认为违规、证书仅未闭合、地图碰撞、动态/时间拒绝或 playback binding 拒绝。日志应使用现有 search/result 结构或测试 harness；不新增常驻冗长生产日志。

### Step 2：构造最小真实 E09 场景

优先只改/新增确定性测试 fixture，不改生产优化目标：

1. 在小型 OccupancyGrid 中构造一段相邻双 corridor 与可审核 Portal；起终点、连接段及三车完整 OBB 都留有明确正净空。
2. 让 Guide allocation 的 region 切换发生在已审核 Portal 的 sigma 处；在切换前通过真实障碍布局使最短、平滑且动力学可行的 F 路径必须进入相邻 region。不要把单车通路等同于三车可行，先以现有模板的 whole/diagonal covers 核验三车实际 OBB 可覆盖。
3. 控制 fixture 尺寸，减少不必要弯道和搜索规模；目标是单个可复现正例，不是随机场景成功率基准。
4. 调试时可对有限的地图尺寸/障碍位置做确定性网格扫描，但最终通过例必须调用完整生产规划管线、使用全部正常模板候选、真实 solver 和审核。最终用例不得 monkeypatch route/guide/template builder、优化器、证书、旧 audit、碰撞审核或发布状态。
5. 每次候选扫描记录输入 hash、allocation sigma、Portal overlap、三车覆盖余量和失败层。将搜索限定为最多 30 个冻结参数组合；到达 30 个仍无候选时停止，不扩大到随机/无界搜索。

### Step 3：仅按诊断结果实施最小修正

先证明有一个**真实几何可行且符合现有合同**的 E09 场景，再决定是否需要生产代码改动。禁止先改代价或门槛再找场景。

- **如果模板生成阶段缺少合法相邻 F profile：**只修正 `formation_corridor.py` 中基于现有 Guide witness、相邻 region 和 Portal 双侧切点的有限 profile 生成。保持最多 3 个模板、不枚举组合、不允许非相邻 region；新增一个 F profile 正例和非相邻/Portal 不重叠反例。
- **如果合法 profile 已生成但没有被尝试：**检查 `pipeline.py` 的现有模板排序、预算和失败状态。只在当前模板数/soft budget 上限内调整确定性顺序；不得增加隐式重试或放宽 60 s 请求预算。
- **如果 solver 始终停在旧 region，且证据证明 fixture 的相邻模板存在可行解：**优先复用现有初值生成能力，最多替换现有 6 个 initial 中的一个为 template-aware 的连续曲率初值。初值必须满足原起终边界、曲率连续合同；最终仍须由相同 NLP、独立连续证书及后续审核验收。先对 frozen fixture 证明该初值可收敛；不调低硬约束、不增加 initial 上限。
- **如果路径实际违反三车 OBB、证书给出 `VIOLATED` 或地图碰撞：**该候选确实不安全，不得修证书或调 margin 使其通过。改 fixture/route 几何；只有发现可独立复算的证书实现错误时，才提交最小数学修复及专门反例。
- **如果只有 `UNCONFIRMED`：**保存未闭合区间、误差项和最大 bound。不得把 `UNCONFIRMED` 改为通过；只有能证明区间实现漏界/错界时才修实现。若是正常保守性，则换一个净空更大的 fixture。
- **如果在不改目标函数/区域语义的情况下始终无法产生迁移：**停止开发并提交设计决策，不得自行引入 migration reward、改 allocation 切点或改变区域定义。原因是这些会改变规划偏好或数学语义，须由用户和 ChatGPT 决定。

### Step 4：完整正负与回归验证

至少新增一个独立测试文件（建议 `tests/test_virtual_a_e09.py`）及冻结小地图 fixture；如确需生产修正，只改 `formation_corridor.py`、`optimizer.py`、`pipeline.py` 中有直接证据的最小部分。回归至少包括：

- E09 union 正例：上述 8 项逐项断言；不能只检查最终 `SUCCESS`。
- 同请求 `f_only` 对照：旧 allocation hard gate 和默认 identity 未变化。
- 安全负例：去掉相邻 region 覆盖/收窄 Portal 后拒绝；保留真实 OBB 越界例；template 非相邻 F region、伪造迁移字段、篡改 certificate/template payload 均拒绝。
- 原有航向区间正、负、跨零曲率与 OBB 边界反例保持通过。
- TASK03 既有 formation corridor、optimizer、pipeline、playback/receiver、audit/integration bounds、collision 回归。
- 至少一次真实 ROS/RViz union 正例与失败清屏负例；若 E09 能在本地 ROS 环境运行，则正例必须使用该 E09 输入，并核对 receiver 重建 binding 后才显示/播放。若环境阻塞，准确记录，不可用离线单测代替并声称通过。

建议命令（仓库根目录）：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 -m unittest test_virtual_a_e09
python3 -m unittest test_virtual_a_formation_corridor
python3 -m unittest test_virtual_a_corridor_optimizer
python3 -m unittest test_virtual_a_pipeline
python3 -m unittest test_virtual_a_audit test_virtual_a_integration_bounds test_virtual_a_collision
python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback test_virtual_a_receiver
```

另按环境条件重跑既有 `catkin_make -C catkin_ws -j4`、`bash -n run_demo.sh`、dry-run 和指定 ROS/RViz smoke；所有命令保存退出码、耗时、stdout/stderr。不得运行会覆盖历史 evidence 的全量 discover。

## 5. 禁止事项与范围

- 禁止更改 `f_only` 默认值/行为，禁止混淆旧版 hard gate 的诊断结果。
- 禁止关闭、跳过或放宽三车 OBB 约束、连续证书、原地图碰撞审核、动态/时间审核、playback verifier。
- 禁止篡改旧 `audit_path_in_corridors()` 的返回值、容差或执行次序来制造迁移。
- 禁止手工写入 `f_only_gate_migrated=true`，不得让 pipeline 以证书未 `VERIFIED` 的结果进入发布。
- 禁止为 E09 改写起终点/模型身份后仍称为“同一输入”；union/f_only 对照必须有同一输入 manifest。
- 禁止为通过 E09 改历史测试预期、禁用测试、删减证据或覆盖 TASK01/TASK02/TASK03 原 evidence。
- 禁止更改 geometry/model/timing/collision 基础合同、ROS message schema、旧 FR06 或其他规划后端，除非具体缺陷经证据证明且先停下提交 Codex 审核。
- 不做全局搜索器重构、不引入新的规划框架、不提高路线/模板/initial/recovery/时间预算上限。

## 6. 量化验收与 Codex 独立检查

### 必须全部通过

1. E09 的最终 union result 满足 `SUCCESS && executable`，并有 `f_only_gate_migrated=true`。
2. 旧 allocation containment 对**同一接受路径**明确失败，存在可复算的违反 witness；union F 与三车 OBB 全域证书为 `VERIFIED`。
3. 最终 union F/OBB 数值硬残差均 `<=1e-8 m`；CasADi、数值和 SciPy fallback 固定向量比较差 `<=1e-8 m`；连续证书所有 upper residual `<=0`、未闭合区间为 0。
4. map OBB continuous collision、geometry、dynamics、timing、playback envelope/receiver 验证全部通过；无旧轨迹或失败轨迹发布。
5. `f_only` 同输入对照保留原 gate、模型/context hash 与序列化语义，不得带 union binding。
6. 新增正例与负例测试、指定回归、构建/脚本检查通过；保护文件和历史 evidence 前后哈希一致；`git diff --check` 通过。

### Codex 验收时独立复算

- 对照 fixture manifest，重新从 request/map/route/guide 得到输入身份；确认没有被 mock 的生产 gate、solver、证书或 route generator。
- 从保存的最终路径独立重跑旧 F-only containment，复算失败 residual；确认不是只在 Guide 或初值上失败。
- 重新检查接受 template 的每段 F region、allocation 邻接关系及双侧 Portal cut；重建 template/certificate identity。
- 复核实际 NLP F/OBB row count、数值残差、连续证书和后续安全审计状态；确认通过结果和发布的是同一 path hash。
- 检查 `f_only_gate_migrated` 的赋值只由真实旧审核失败与先前 `VERIFIED` 证书路径触发。
- 重跑 E09 定向测试及关键回归；不能以 Claude 报告代替实测。

## 7. 交付物与停止点

Claude 交付：

- 最终开发报告：`work_flow/TASK03_REWORK_02_Claude交付.md`，区分代码事实、实验事实和推断；
- 新测试及冻结 fixture（若成功找到 E09）；
- 新证据根目录，至少含 `SUMMARY.md`、机器可读 `results.json`、场景 manifest、失败层/模板矩阵、commands 与测试日志、最小 `source.diff`、源码/保护文件 hashes、`git_state.txt`、`manifest.sha256`；
- 如真实 ROS/RViz 已运行，附 launch 参数、topic/check summary 和截图/录屏索引。证据必须匹配同一 E09 输入和最终代码。

完成 E09 或触及本方案停止条件后立即停下，等待 Codex 独立验收；不得继续 TASK04，不得提交/推送。

如果最多 30 个确定性 fixture 仍无符合合同的场景，或发现只能靠改目标函数/allocation 语义才能得到迁移，交付 `BLOCKED_DESIGN_DECISION` 报告，附失败层和最小决策选项后停止。此时不得把返工记为整体通过。需由用户和 ChatGPT 在下列方向中决策后再另立任务：

1. 保留当前 allocation 定义，通过有研究依据的初值/目标改进让合法相邻模板可被优化器找到；
2. 改为按实际优化路径的 region crossing 定义切换时刻（会改变 allocation 语义并涉及模板/优化耦合）；
3. 保留旧 F-only hard gate，明确放弃本阶段 gate migration/E09 目标。

不得由 Claude 或 Codex 在本返工中自行选择上述研究路线。
