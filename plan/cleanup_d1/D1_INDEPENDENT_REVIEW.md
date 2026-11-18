# CopTrans_base D1 精简重构独立审核

审核日期：2026-11-18  
仓库：`/home/wq/whut_thesis/expv3_multi_virtual_a`  
分支：`feature/virtual-rigid-formation-a`  
审核 HEAD：`af7c9fd6246e6c2c1c7b8416d4d094e47f0adc26`

## 1. 结论

**有条件通过，但必须显著缩小 D1。** 原稿的保护原则正确，候选范围却混合了三类性质不同的对象：运行源码、验收证据、无效运行时残留。当前证据只足以批准删除 3 个失效符号链接；**没有任何生产源码文件、函数或代码区间达到安全删除标准**。

本次不应以“目录很大”为理由处理 `evaluation/virtual_formation_a/`（约 427 MiB）和 `evaluation/formation_refactor_v02/`（约 377 MiB）。二者内部存在跨阶段清单、源码快照、保护哈希和验收索引。逐文件清理会破坏历史证据；整包外移又属于独立的证据归档任务，不是无行为变化的源码精简。

## 2. 最终删除白名单

以下 3 项为 **SAFE_TO_REMOVE**：

1. `evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt1/ros_log/latest`
2. `evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt2/ros_log/latest`
3. `evaluation/virtual_a_corridor/V2/20261118T024527Z/attempt3/ros_log/latest`

依赖证据：三者均为已跟踪的绝对路径符号链接，当前目标不存在；全仓精确路径检索没有引用；实际 V2 数据不位于链接目标中。删除不改变源码、ROS/CMake 入口、测试输入或实验原始文件。它们只占目录项，不会明显缩小仓库体积。

**批准删除的源码文件：无。**  
**批准删除的函数或代码区间：无。**

## 3. 必须保留的候选

| 对象 | 结论 | 依赖证据 |
|---|---|---|
| `fwis_v3_optimizer/planning_display.py` | KEEP | `fwis_v3_playback/scripts/playback.py:12,26,38-50,109` 运行时直接导入并用于拒绝过期/不可执行轨迹；`tests/test_planning_display.py` 验证其合同；`optimizer_node.py:831` 和离线探针也将其纳入身份清单。删除会破坏单车回放和失败后不播放旧轨迹的安全语义。 |
| `tests/test_planning_display.py` | KEEP | 是上述运行时状态机的直接合同测试，不是孤立旧测试。 |
| `formation_continuous_a/optimizer.py` | KEEP | 当前连续 A NLP、初值、走廊约束、求解与候选审核主链；`solve_open_path_guided()`/`solve_open_path()` 被 pipeline 和测试使用。未发现可独立证明不可达的顶层符号。 |
| `formation_continuous_a/pipeline.py` | KEEP | `plan_virtual_a()` 是当前地图规划入口；A*、恢复、审核结果编排被 ROS adapter 和多组测试直接使用。 |
| `formation_continuous_a/corridor.py` | KEEP | C1/C2 走廊构造、包含性审核、恢复分配和 RViz 显示均有直接调用；`formation_playback.py:28` 还读取显示状态。 |
| `formation_continuous_a/ros_adapter.py` | KEEP | 请求绑定、发布准入、模型/地图/上下文哈希与 fail-closed 发布主链；`formation_uniform_planner_node.py:625,661,872,1183,1938` 等位置动态导入。 |
| `legacy_route` 分支 | KEEP | `contracts.py:780,821` 仍是默认合同，`formation_uniform_planner_node.py:1943` 默认读取该值，`optimizer.py:1757` 明确处理其输入约束。 |
| FR04/FR05/FR06 的 43 个 `reviewed_source/` 文件 | KEEP | 多个 `audit_manifest.json`、`CODEX_ACCEPTANCE.json`、`before_hashes.json` 和验收说明按相对路径及 SHA-256 引用；仅删快照会使证据自校验失效，且总量约 1.7 MiB，收益很小。 |
| `qna/` 全部 24 张 PNG 与 3 个 Markdown | KEEP | `request.md` 引用 `image.png`～`image-7.png`，`test_error.md` 引用 `image-8.png`～`image-23.png`；`plan/13_R03...md:93` 将其列为必读复现材料，定时任务脚本使用第三个 Markdown。不存在孤立截图。 |
| 历史 `.pid`、仍有效的 `latest` 链接 | KEEP（D1） | 它们不是运行输入，但已进入 FR/G 阶段的 source/protected manifests 和哈希文件。单独删除会破坏历史清单；应随完整证据包归档，而非在 D1 零散清理。 |
| `G0/.../runner_fixtures/history_link` | KEEP | G0/G1/G3/G5 的 fixture、source manifest 和保护快照明确记录它；不是普通 ROS 残留链接。 |
| 单车、旧 formation、`uniform_v02`/legacy 节点和模块 | KEEP | CMake 安装 `optimizer_node.py`、`formation_optimizer_node.py`、`formation_uniform_planner_node.py`；`demo.launch` 与 `formation_demo.launch` 分别启动单车和多车后端；playback/demo CMake 也安装相应脚本。 |

## 4. 需要证据、不得纳入 D1 的对象

- `evaluation/virtual_formation_a/`、`evaluation/formation_refactor_v02/` 中所谓“重复日志/报告/JSON”：**NEEDS_EVIDENCE**。现有阶段间保护清单会记录早期目录，不能凭时间戳认定旧运行可删。
- 有效的 ROS `latest` 绝对链接和所有 PID 文件：**NEEDS_EVIDENCE**。它们没有运行价值，但属于已签名/已哈希证据内容。若要移除，必须把完整父证据包原样归档并提供可验证恢复流程。
- 四个连续 A 核心文件中的私有函数：**NEEDS_EVIDENCE**。名称前导下划线或实现相似不等于不可达；当前静态引用、测试和动态 ROS 路径不足以支持删减。
- 注释、诊断文本和失败原因：**KEEP**。这些内容用于区分求解失败、审核拒绝、预算耗尽和发布禁止，删除收益不足以抵消诊断风险。

建议另立“证据冷归档”任务：以完整阶段包为最小单位，生成清单与 SHA-256，在外部归档可恢复验证通过后再决定是否移出主分支。不要和 D1 源码重构混做。

## 5. 原稿遗漏与范围修正

1. 原稿低估了 `planning_display.py` 的运行时作用；它不是仅供旧 HUD 使用，而是单车 playback 的发布状态闸门。
2. 原稿没有说明 `reviewed_source` 与验收清单是相对路径绑定；只保存哈希文本不能替代被审核字节。
3. `qna` 图片全部有 Markdown 引用，不存在可直接删除的“未引用截图”。
4. 真正明确的冗余只有 3 个断链；仓库未跟踪 `__pycache__`、`.pyc`、编辑器备份等常见垃圾文件。
5. D1 不应允许 Claude 在执行时自行扩展删除范围。发现新候选只能记录，不能顺手删除。

## 6. 行为保护与验收原则

D1 实施后，Git diff 只能包含本报告批准的 3 个符号链接删除和 D1 交付报告；不得修改 Python/C++、测试、launch、CMake、消息、配置或现有验收证据。由于没有代码变化，无须以大规模规划实验证明等价；应以精确 diff、引用复查和工作树边界检查验收。若实施者希望删除任何源码或更多历史数据，必须停止并重新申请范围审核。
