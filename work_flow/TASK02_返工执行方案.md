# TASK02 返工执行方案（Codex 独立验收后）

**执行人：Claude Code**  
**返工验收人：Codex（一次性整体复核）**  
**工作区：** `/home/wq/whut_thesis/expv3_multi_virtual_a`  
**分支：** `feature/virtual-rigid-formation-a`  
**基线 HEAD：** `bee4e3aa0f86e24085a1840ee0581b2fca34add8`

## 1. 任务目标与边界

修复 TASK02 整体验收中发现的三项问题：

1. Guide 局部变体必须优先围绕 assessment 指出的风险 Portal 生成；无风险时选择 Chebyshev 半径最小的 Portal。
2. HUD 和诊断数据必须区分 `SINGLE_REGION` 与 `COVERED_BY_ADJACENT_UNION`，不能把单区覆盖计入并集覆盖。
3. 停止本任务 ROS 进程后重新冻结证据，确保 manifest 全部通过校验。

本返工仅完成以上三项。不得实施 TASK03，不扩大搜索算法，不更改安全阈值或既有独立认证。

## 2. 工作区保护要求

- 开始前记录 `git status --short --branch`、HEAD 与 TASK02 当前差异。
- 保留工作区现有全部内容。不得执行 `git reset`、`git clean`、覆盖、删除或重排历史文件。
- TASK01 已接受的 dirty 文件必须逐字节保留：
  - `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py`
  - `tests/test_virtual_a_geometry.py`
- 不提交、不推送、不切换分支。
- 允许修改范围仅限：
  - `formation_continuous_a/corridor.py`、`formation_continuous_a/pipeline.py`
  - `formation_playback.py`（仅在 HUD 需要读取新增的区分计数时）
  - `tests/test_virtual_a_formation_assessment.py`
  - `tests/test_virtual_a_pipeline.py`
  - `tests/test_virtual_a_corridor_display.py`
  - `tests/test_formation_hud.py`
  - 本任务交付文档及 `work_flow/TASK02_evidence/` 中的返工证据
- 不修改 `geometry.py`、`formation_collision.py`、`optimizer.py`、`run_demo.sh`、launch、`/formation/playback_plan` 合同或 TASK01 历史证据。若确有接口阻塞，先停止并报告，不自行扩大范围。

## 3. 返工 A：风险 Portal 优先的有界 Guide 变体

### 3.1 当前问题

现有 `corridor_guide_candidates()` 按 Portal 索引生成至多三个局部变体，没有读取基线 assessment；Portal 数量超过三个时，后续风险 Portal 可能没有对应变体。这不符合 `TASK02_Codex执行方案.md` §4.2 的候选选择规则。

### 3.2 实现合同

保留现有 QP Guide 与 Chebyshev-center Guide。局部候选按以下步骤生成：

1. 先对 QP Guide 做一次基线三车 assessment。
2. 选择一个目标 Portal：优先选基线 assessment 中最早出现的 map collision、union gap 或 transition unknown 对应 Portal；无风险时选择 Chebyshev 半径最小者；平局按 Portal 索引升序。
3. 对选中 Portal 的凸交集，最多构造 4 个确定性内部点。可复用 `z' = 0.5 z + 0.5 v`，其中 `z` 为 Portal Chebyshev 中心，`v` 按固定次序取交集顶点。去重后，只对局部 transition 站进行有限评估，保留最多 3 个局部候选。
4. 每个完整 Guide 只能改动目标 Portal 一个点；请求起终点必须保持不变，Guide 的 F-only 区域约束与 `allocate_corridors()` 验证必须通过。
5. 完整候选总数仍不超过 5。候选及其 assessment 仍为 `DIAGNOSTIC_ONLY`，只用于排序/初值；不得据此宣称安全、可执行或连续覆盖。
6. 复用已经计算的 QP 基线 assessment，避免无谓重复调用 checker。受现有 60 秒 soft budget 约束；预算耗尽时保留已算结果，其余候选标为 UNKNOWN，不延长预算。
7. 任何候选不兼容时记录明确原因并保留原 QP 回退；不能通过放松 F-only corridor 约束让候选入选。

可调整内部函数签名，但不要新增 manager、factory 或第二套 assessment 管线。候选生成和排序须保持确定性。

### 3.3 必须新增或修正的测试

- 多 Portal 固定 chain：把风险放在索引靠后的 Portal，证明局部候选确实修改该 Portal，而不是只修改前三个 Portal。
- 基线首个风险 Portal 的优先级高于更早但无风险的 Portal；无风险时验证最小 Chebyshev 半径和平局索引规则。
- 局部候选点都位于相邻 corridor 交集；只改一个 Portal；每个候选重新分配并通过 F-only allocation 验证。
- 构造一个真实 `FormationCollisionChecker.static_check()` 结果改善的候选，证明 assessment 排序会选择它，并由 solver spy 断言传入的 Guide、allocation、chain 和 candidate identity 同源。
- 所有候选均为 collision/gap/unknown 时仍调用原 solver；不减少原有搜索/初值/recovery 上限，不更改审核路径。
- 候选数上限、60 秒预算与预算耗尽行为有测试覆盖。

## 4. 返工 B：区分单区覆盖与相邻并集覆盖

### 4.1 当前问题

`corridor_portal_diagnostics()` 当前将每车 `counts.single + counts.union` 写入 `union_covered_total`。因此 map05 的 39 个单区样本显示为 `union covered=39`，语义错误。

### 4.2 实现合同

- `single_region_total` 只累计 `counts.single`。
- `union_covered_total` 只累计 `counts.union`，不得包含单区覆盖。
- `union_gap_total` 和 `union_unknown_total` 继续分别对应 gap、unconfirmed。
- progress、HUD、marker 诊断摘要使用同一字段语义；缺失/未计算仍显示 `unknown`，不得生成默认数值。
- HUD 可以紧凑显示为 `single=… | union=… | gap=… | unknown=…`。若因现有空间限制不能加字段，应明确选择准确标签，不能保留误导性的 `union covered` 合计。
- 不得改变规划候选评分、安全判定或可执行发布条件。

### 4.3 必须新增或修正的测试

- 仅 `SINGLE_REGION` 的样例：single 计数非零、union 计数为 0。
- 仅 `COVERED_BY_ADJACENT_UNION` 的样例：union 计数精确反映样本数。
- 混合 single/union/gap/unknown 样例：四类计数互不混合，HUD 与 progress 保持一致。
- 缺字段、无效字段和旧 progress payload 仍 fail-closed / unknown。
- 更新现有把 `single + union` 当作 `union_covered_total` 的断言，不得仅改显示文案来掩盖底层字段语义错误。
- map05 预期值：交付报告中的 13 站、三车单区覆盖对应 `single=39, union=0`（如果实际最终 station 数变化，以冻结 assessment 原始字段逐项核对并解释）。

## 5. 返工 C：证据停止后重冻结

### 5.1 当前问题

Codex 从仓库根目录独立执行 manifest 校验时，314 项中有 4 项失败：

- `candidate/step_f/master_11633/ros_home/log/09d4bc28-e3ed-11f1-a57b-1b4c90db486b/master.log`
- `candidate/step_f/master_11633/ros_home/log/09d4bc28-e3ed-11f1-a57b-1b4c90db486b/roslaunch-wq-ubuntu-708899.log`
- `candidate/step_f/master_11633/ros_home/log/09d4bc28-e3ed-11f1-a57b-1b4c90db486b/rosout-1-stdout.log`
- `candidate/step_f/master_11633/roscore.log`

Codex 核查时没有发现仍运行的 ROS/RViz 进程；当前清单与日志内容不一致。

### 5.2 重冻结步骤

1. 只检查并停止本返工自己启动的 ROS/RViz 进程；不得触碰其他 master、用户节点或其他会话。
2. 确认本任务没有仍会写入证据目录的进程后，再生成最终 evidence 文件清单与 SHA-256。
3. 从仓库根目录运行：

   ```bash
   LC_ALL=C sha256sum -c work_flow/TASK02_evidence/20261119T054527Z/manifest.sha256
   ```

4. 将最终原始校验输出和退出码保存到返工证据。避免清单自引用：manifest 不要列出 `manifest.sha256` 自身或会在校验后再改写的最终校验日志；清单项目数量以实际文件数为准。
5. 重新核对 TASK02 源码哈希、TASK01 两个 dirty 文件哈希、最终 `git status`、差异统计及证据文件列表。报告里只写实际校验数量与退出码，不沿用旧的 `314/314` 声明。

## 6. 独立验证命令

在仓库根目录使用项目已有环境：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 -m unittest test_virtual_a_formation_assessment
python3 -m unittest test_virtual_a_pipeline
python3 -m unittest test_virtual_a_corridor_display test_formation_hud
python3 -m unittest test_virtual_a_corridor test_virtual_a_collision
catkin_make
git diff --check
bash -n run_demo.sh
```

之后用未经隐藏 backend/guidance 参数覆盖的默认入口重新验证：

```bash
./run_demo.sh --map map05 --scale 2.625
./run_demo.sh --map evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S1 --scale 1.0
```

确认 map05 的规划、认证和播放仍通过；S1 如仍失败，记录真实终因，不通过降低阈值改写结论。保存实际命令、退出码、关键 progress/quality 数据及必要截图。若当前环境不允许安全启动私有 ROS master，标 `NOT_RUN` 并说明原因，不影响证据完整性修复。

## 7. 验收门槛与停止条件

Claude 完成后交付：

- 修改摘要、最终 diff、最终 Git 状态；
- 新增/修正测试及每条命令的退出码和原始日志；
- 后置 Portal 风险选择的结果证据；
- single/union 分离计数的 progress/HUD 样例；
- map05/S1 默认入口验证结果（或明确 `NOT_RUN`）；
- 重新生成并成功独立校验的 manifest；
- TASK01 dirty 文件前后哈希相同的证据。

只有同时满足以下条件，TASK02 才可重新提交 Codex 整体验收：

1. 后续高风险 Portal 可被局部变体选择，全部变体仍通过 F-only corridor 校验。
2. `single_region_total` 和 `union_covered_total` 语义、HUD 文案与原始 assessment 一致。
3. 保留 route/Guide 实际进入 solver 的证据；所有独立安全审核与 publish gate 未改、未放宽。
4. 定向测试、构建、静态检查通过；默认入口结果如实记录。
5. manifest 根目录复算无失败，Task01 基线内容未丢失。

完成上述工作后立即停止，等待 Codex 独立整体复核；不进入 TASK03，不提交、不推送。
