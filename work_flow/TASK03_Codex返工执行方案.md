# TASK03 返工执行方案 01

**执行对象：Claude Code**  
**最终验收：Codex 独立复核**  
**工作区：** `/home/wq/whut_thesis/expv3_multi_virtual_a`  
**分支 / 基线 HEAD：** `feature/virtual-rigid-formation-a` / `a3c709e57560b1060dcc422777f4131a3e558acf`

本任务只修复 TASK03 验收发现的问题。Claude 完成内部步骤后自行检查；整个返工任务完成后停止，由 Codex 对完整差异和证据验收。不得自动开始 TASK04。

## 1. 返工结论与范围

TASK03 当前不能验收通过，原因有两项：

1. **P0，连续证书的航向区间没有包含区间起点。** [formation_certificate.py](/home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/formation_certificate.py:386) 以 `kappa * width` 作为整段航向增量区间。若曲率区间同号，该区间不包含零增量。例如曲率 `[0.4, 0.6]`、宽度 `0.1` 时，代码得到约 `[0.04, 0.06]`，而所有部分区间积分的安全外包络至少应为 `[0, 0.06]`。OBB 姿态和角点区间可能漏掉真实路径姿态，因此不能据此签发 `VERIFIED`。
2. **P1，union scope 的 F 参考点仍受旧固定 allocation 限制，门控迁移目前没有实际放宽能力。** `FormationCorridorTemplateSegment` 只有车辆 `covers`，没有 F 区域归属；优化器仍按 `corridor_allocation` 编译旧 F-only rows；连续证书也按 `_allocations_at()` 检查 F。pipeline 虽在最终旧审核失败时设置 `f_only_gate_migrated` 并继续，但候选在更早的 NLP 和证书阶段仍被要求留在 allocation 指定的单个 region。因此，方案要求的 E09“旧 F-only 单区 gate 失败、F+OBB 在已审计相邻 union 内通过”尚未实现。任务计划 §3.4、§3.7 及 E09 明确要求该能力。交付报告也将 E09 标为 `NOT_RUN`。

本返工的主要目标是修复上述两项，保留 TASK03 已实现的三车 OBB rows、碰撞审核、时间规划、ROS envelope 和 RViz 交付。E09 必须成为真实的端到端回归证据。E04 的现有 RViz 负例在模板构建前已因 Guide assessment 发现地图碰撞而停止；该结果证明失败时不发布轨迹，但不证明 OBB NLP rows 主动拒绝了该场景。返工报告须准确区分这两类证据。

## 2. 固定技术合同

返工不得改变以下决策：

- 保留 `f_only` 为 Python、launch 和 `run_demo.sh` 默认值；不得在本次返工切换默认 scope。
- `three_vehicle_union_v1` 只在同一个已审核 `CorridorChain` 的相邻区域之间使用有限、确定性的赋值。禁止非相邻 region 跳转、全组合枚举、整数规划或新增搜索器。
- 每个连续 sigma 段的 F 参考点和每辆车的完整 OBB 都必须有明确的凸区域归属。完整 OBB 仍使用现有 whole / diagonal convex-piece 模板；每个 piece 的全部顶点通过半空间约束，凸性推出整片包含。
- 段切换处必须进行左右两侧检查；F 区域切换也必须在切点满足两侧 region 约束。
- union scope 的连续证书只有在 F 与三车 OBB 全域 `VERIFIED` 后才可进入原 geometry、dynamics、map OBB collision、timing 与 publication verifier。不得关闭或放宽任何既有审核或 margin。
- `f_only` 的序列化、规划输出、默认参数和历史 model/context hash 保持兼容。所有新约束行必须在 CasADi `g`、数值 residual 和 SciPy fallback 中使用一致顺序与公式。
- 保持现有限额：模板最多 3 个，solver initial 最多 6 个，routes 最多 3 个、recovery 最多 1 次、请求 soft budget 最多 60 s。不得增加暗中重试。
- 不修改 TASK01/TASK02 的历史证据、TASK03 原交付证据、冻结 fixture 或 `evaluation/` 历史记录。所有返工证据写入新目录。

## 3. 工作区保护与允许修改范围

### 3.1 开始前必须记录

Claude 在现有 worktree 中执行，不得切分支、创建 worktree、reset、checkout、clean 或覆盖现存文件。先保存：

- `git rev-parse HEAD`、分支名、remote 状态、`git status --porcelain`；
- 当前所有 TASK03 生产和测试文件 SHA-256；
- 保护文件 SHA-256：`formation_continuous_a/geometry.py`、`tests/test_virtual_a_geometry.py`；
- TASK01/TASK02 证据目录及 `evaluation/` 的开始状态。

开始状态应与 TASK03 交付根 `work_flow/TASK03_evidence/20261119T133018Z/final/manifest.json` 对照。当前 HEAD 仍是 `a3c709e57560b1060dcc422777f4131a3e558acf`，TASK03 改动尚未提交；不得将本次工作误写为从干净 HEAD 开始。

### 3.2 允许的生产代码范围

- `formation_continuous_a/formation_certificate.py`：修复部分路径上的航向积分区间。
- `formation_continuous_a/contracts.py`：为模板 sigma 段加入 F 区域赋值及合同校验；只在确有必要时更新模板 schema/compiler 版本。
- `formation_continuous_a/formation_corridor.py`：从同一 Guide、assessment、allocation 和 Chain 构造有限的 F 区域赋值；在既有模板中加入 F rows。
- `formation_continuous_a/optimizer.py`：union scope 使用模板 F rows 替代旧 allocation 固定 F rows；保持 OBB rows 不变，并使 numeric/CasADi/SciPy 一致。
- `formation_continuous_a/pipeline.py`：接入 F-template identity、独立证书和准确迁移状态；保留旧最终地图 collision gate。
- `formation_playback.py`、planner node 或 playback receiver：仅当模板 payload 字段变化导致现有 binding 重建不兼容时做最小适配。
- `tests/test_virtual_a_formation_corridor.py`、`tests/test_virtual_a_corridor_optimizer.py`、`tests/test_virtual_a_pipeline.py`，必要时的 playback/receiver 测试。

无需更改 ROS message schema：现有 `formation_binding_json` 已承载模板 JSON。未经证据不得修改 `bounds.py`；应优先复用 `formation_continuous_a.bounds.imul()` 的向外舍入区间乘法。不得修改 `geometry.py`、`integration_bounds.py`、`timing.py`、`collision.py`、旧 FR06、uniform_v02、legacy 或单车规划器。

## 4. 实施步骤

### Step A：修正航向增量的区间外包络

在 `formation_certificate._check_interval()` 中，设 `kappa = [k_lo, k_hi]` 是 profile 在 `[a,b]` 上的向外曲率界，`width=b-a`。对任意 `s∈[a,b]`：

\[
\Delta\psi(s)=\int_a^s\kappa(u)\,du,\quad s-a\in[0,width].
\]

必须使用外向舍入的区间乘法：

\[
\Delta\psi([a,b])\subseteq [k_{lo},k_{hi}]\,[0,width]
=[\min(0,k_{lo}width),\max(0,k_{hi}width)].
\]

建议直接使用 `bounds.imul(kappa, (0.0, width))`，再通过现有 `bounds.iadd()` 加到 `(qf.psi,qf.psi)`，并保留现有 `2 * angle_error_rad` 外扩。不可只把端点 `kappa * width` 改成普通 `min/max` 后丢失 outward rounding。

测试至少包括：

1. 正曲率区间 `[0.4,0.6]`、宽度 `0.1`，航向增量下界须不大于 0、上界须不小于 0.06。
2. 负曲率区间 `[-0.6,-0.4]`、宽度 `0.1`，下界须不大于 -0.06、上界须不小于 0。
3. 跨零区间 `[-0.2,0.3]`、宽度 `0.1`，所得区间须包含 `[-0.02,0.03]`。
4. 用正、负、跨零曲率的 `VirtualAPath` 在区间内密集取样，独立计算 `state_at_s(s).qf.psi - state_at_s(a).qf.psi`，样本都必须落入用于证书的航向区间；比较误差按已有 integration error bound 处理。
5. 至少一个靠近凸走廊边界的 OBB 反例：真实路径某个中间状态违反半空间时，证书不得返回 `VERIFIED`；允许 `VIOLATED` 或由于证据/误差不足返回 `UNCONFIRMED`。同时保留全程有正安全余量的 `VERIFIED` 正例。

### Step B：为每个 union 模板段补齐 F 区域归属

当前模板段的 `covers` 只描述车辆 OBB，不能代表 F。为每个 `FormationCorridorTemplateSegment` 增加一个确定的 `f_region_index`（或等价的单 region 点覆盖字段），并将它纳入模板 canonical payload、template id 和 verifier 重建。

模板构造要求：

1. 赋值只来自本次已审核 Chain 与当前 selected Guide/assessment；不从其他 route 或历史 candidate 借 region。
2. 每个 sigma 段的 F 允许归属当前 allocation region，或当前 allocation/Portal window 中与之直接相邻的 Chain region；禁止跳过 region、跨多个 Portal 或使用 Chain 外的区域。
3. 在 Guide/assessment 的各 witness station 上，确定 F region 候选；优先保留旧 allocation region，其次按确定顺序尝试合法相邻 region。只生成现有 3 个模板候选额度内的结果，不对 F 与三车全部组合做笛卡尔积。
4. 如果单个 region 赋值不能连续覆盖，则在现有 Portal/allocation cuts 或必要 witness station 形成的切点处分段。切点两侧 F 必须同时满足左右 region 的 halfspaces；不能仅因两个半空间集合拼接就声称 F 位于 union。
5. 所有 F region 必须属于通过 `audit_corridor_chain()` 审核的 Chain。缺失、非相邻或 station 无合法赋值时，返回 `COVER_TEMPLATE_INSUFFICIENT` / `UNCONFIRMED`，不回退到 f_only。
6. 若新增字段改变 payload，更新 schema/compiler 身份，保证旧模板 payload 不会被静默当作新合同接受。默认 `f_only` 旧 payload key set 与 hash 不变。

### Step C：使 NLP 和连续证书消费同一个 F assignment

**优化器：**

- `f_only` 沿用原 `corridor_allocation` F-only constraints，行为与 row 顺序保持不变。
- union scope 使用 template 每段的 `f_region_index` 编译 F-reference-point halfspace rows，替代旧固定 allocation F rows。禁止 union 中同时添加固定 allocation 的 F-only hard rows，否则 E09 仍不能通过。
- 三车完整 OBB 的现有 rows 保留。F rows 与 OBB rows 分开记录数量和 provenance；`formation_obb_constraint_count` 仍只统计真正 append 到 CasADi `g` 的 OBB scalar rows。
- numeric residual、CasADi `g`、SciPy fallback、failure residual 和 row count 共享相同 station、region、半空间顺序与符号。hard residual 验收门槛沿用 `<=1e-8 m`。

**连续证书：**

- 对每个开放 interval 根据 template segment 选择 F 的 `f_region_index`；车辆 OBB 继续按其完整 convex piece region assignments 检查。
- interval cuts 包含 Hermite/profile/template/F-region/Portal/allocation 边界。每个内部切点左右分别审核，F region 切换时也必须左右都闭合。
- 证书中的 F 和 OBB region 全部来自同一已审核 Chain。旧 `audit_path_in_corridors()` 在 union scope 作为 F-only 对照诊断保留，不再暗中作为前置硬约束；`f_only` 仍以它作为硬门槛。
- 只有连续 F+OBB 证书 `VERIFIED` 且既有 geometry、dynamics、原地图 OBB collision、timing 与 envelope verifier 全部通过，才能置 `SUCCESS` / `executable=true`。

**Pipeline 与回放：**

- `f_only_gate_migrated=true` 仅在旧 F-only containment 实际失败、union F+OBB certificate `VERIFIED`、后续所有既有安全审核通过时设置；其他情况下必须是 false，并报告准确原因。
- pipeline、`VirtualAResult`、`formation_binding_json`、playback verifier 必须使用同一模板序列化和 template id。缺字段、旧 schema、digest mismatch、证书非 `VERIFIED` 均拒绝发布/回放。
- 模板无法表达时明确拒绝，不将有限模板失败描述为物理不可行。

### Step D：E09 端到端回归与结果报告

新增一个小型、确定性的、真实有效相邻 Chain 场景。构造时先单独通过 `audit_corridor_chain()`。E09 必须满足：

1. 使用 union scope 的同一请求、Grid、Chain、Guide 和 context；不得伪造/monkeypatch F-only audit 结果。
2. 形成一条真实几何可行路径，使 F 在旧 allocation 固定指定的某段 region 之外，但始终属于该段允许的相邻已审核 Chain region；旧 `audit_path_in_corridors()` 明确失败并保存 failure station/region/residual。
3. 同一候选的 F-template rows、三车 OBB rows 和连续证书全部通过。证书须绑定 request/grid/context/path/chain/template identities。
4. 既有 geometry、dynamics、原地图连续 OBB collision 和 playback envelope verifier 全通过；最终 result 是 `SUCCESS` 且 `executable=true`，并显示 `f_only_gate_migrated=true`。
5. 用 `f_only` 对同一输入运行对照，必须保留旧行为：不得以新模板的相邻 region 放行；旧 F-only context/model identity 和序列化兼容测试通过。

如果无法构造这种场景，必须报告实际阻碍并停止该步骤；不得把当前 `f_only_gate_migrated=false` 的直线路径、模板早期拒绝或人为令旧 audit 失败作为 E09 通过证据。

## 5. 必须运行的定向验证

在仓库根目录设置 ROS/Python 环境：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
```

返工后执行并保存退出码、stdout/stderr、开始结束时间和 wall time：

```bash
python3 -m unittest test_virtual_a_formation_corridor
python3 -m unittest test_virtual_a_corridor_optimizer
python3 -m unittest test_virtual_a_pipeline
python3 -m unittest test_virtual_a_audit test_virtual_a_integration_bounds
python3 -m unittest test_virtual_a_collision
python3 -m unittest test_virtual_a_ros_adapter test_virtual_a_playback test_virtual_a_receiver
python3 -m unittest test_formation_demo_assets
catkin_make -C catkin_ws -j4
bash -n run_demo.sh
./run_demo.sh --map map05 --scale 2.625 --dry-run
```

另外需真实 ROS/RViz 重跑至少一个 union 正例与失败清屏负例，确认新模板 binding 能被 receiver 重建和验证。E09 端到端 pipeline 回归必跑；若用 ROS/RViz 展示 E09，保存完整 params/topic/log/capture summary。`f_only` 使用已保存历史 model/context hash 和定向 pipeline regression 核对；不得为了生成证据覆盖或重写旧 `evaluation/` 数据。

E04 报告需拆成两个结论：

- 已有 live E04：Guide assessment 在 NLP 前发现地图碰撞，`formation_obb_constraint_count=null`；只证明 fail closed 与无旧轨迹发布。
- OBB NLP 负例：离线 solver/integration test 必须证明实际 OBB rows 数量大于零，F-only rows 可行而 OBB rows 有正残差时不能成为 accepted candidate。若另构造 live 版本，也必须证明它确实进入了 NLP。

不要求无目的重跑 617 项全套历史测试；若返工触及与上述调用链无关的模块，先在交付报告说明原因，再由 Codex 判断是否补充范围测试。

## 6. 量化验收条件

### 数学证书

- 正、负、跨零曲率的部分积分区间均向外包含真实增量；采样 oracle 不得发现漏界。
- 旧区间实现的反例必须在新增测试中失败；修复后真实越界用例绝不返回 `VERIFIED`。
- `VERIFIED` 证书的每个闭合 interval outward upper residual `<=0`，unclosed interval 数为 0；`VIOLATED` 需要包含 error budget 后仍为正的 witness；其余为 `UNCONFIRMED`。

### F region/template 与约束

- 每个 union template 段恰有一个合法 F convex-region assignment；不得缺段、倒序、region 越界或非相邻跳转。
- F region/template boundaries 与 OBB piece boundaries 都有双侧检查。
- union NLP 中每个 F 和三车均有真实 hard rows；numeric/CasADi 同一固定决策向量 residual 最大差 `<=1e-8 m`；每个被接受候选最大硬 residual `<=1e-8 m`。
- `formation_obb_constraint_count` 仅计 OBB rows；证据必须同时列出 union F rows、OBB rows 和实际 NLP `g` 总数。

### 行为和安全门

- E09 按 §4 Step D 全部满足，包括旧 gate 失败、union certificate 通过、旧 map collision 通过和最终可执行。
- E04 OBB solver 负例不得 accepted；失败结果无 path/plan/envelope，不复用旧轨迹。
- 所有 union `SUCCESS` 的 template/certificate/request/grid/context/path/chain identities 一致；播放 verifier 接受；任一 identity 被篡改均拒绝。
- `f_only` 基线兼容、默认scope、工作区保护文件 SHA 前后不变。所有预算上限与返工前相同。

## 7. Claude 交付物与停止条件

在不覆盖旧交付的前提下新增：

- `work_flow/TASK03_REWORK_01_Claude交付.md`：逐项记录修复、实际函数和数据结构、E04/E09 结果、未完成项与停止状态。
- `work_flow/TASK03_REWORK_01_evidence/<UTC时间戳>/`：开始/结束 Git 状态、返工增量 diff（相对于本返工开始状态）、源码及受保护文件 SHA、测试日志、E09 完整输入与结果、证书/template/binding identities、ROS/RViz 证据及 manifest。

返工报告必须如实记录：未运行项标 `NOT_RUN`；不可复现输入保持 `NOT_REPRODUCIBLE`；不能把 optimizer `success` 等同于安全通过；不能把 E04 early rejection 说成 OBB NLP 负例；不能把 E09 未通过或未运行标为通过。

完成后停止，等待 Codex 独立验收。不得：

- 切分支、建 worktree、reset/checkout/clean、覆盖或删除任何旧文件；
- 修改冻结的 TASK01/TASK02/TASK03 原始证据或 `evaluation/` 历史数据；
- 切换 `f_only` 默认、修改旧门槛、关闭 collision/certificate/playback 审核；
- 开始 TASK04、提交 Git、推送远端。

Codex 只在复核上述数学外包络、F assignment→NLP→certificate→binding 调用链、E09、E04 证据口径、回归测试与 Git 保护状态后，决定 TASK03 是否 ACCEPTED。
