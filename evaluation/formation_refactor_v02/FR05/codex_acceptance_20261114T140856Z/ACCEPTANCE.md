# FR05 最终独立验收：ACCEPTED

2026-11-14，Codex。工作区`/home/wq/whut_thesis/expv3_multi_v02`，分支`refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。

**FR05验收通过。FR05-R1、FR05-R2均关闭，无剩余已确认阻塞。** 本轮严格限定为上轮要求的发布前空摘要守卫和合法反馈回归，不重新扩大任务范围，不自动进入FR06。

## 1. 改动及合同核实

【代码事实】与上一轮174项源码清单逐项比较，仅`catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`和`tests/test_uniform_feedback_identity.py`改变。节点实际差异只有`on_playback_status()`约1087行新增的确认类反馈守卫（含注释18行），原摘要/编队/backend匹配逻辑保留。无搜索、时间规划、解析回放、launch默认值或数学模型变动。

【代码事实】活动摘要在新请求准入时清空，在审核成功、构造及自检完成并准备发布时建立；相关修改及回调由同一锁保护。现在对HOLD_START/PLAYING/HOLD_END首先要求活动摘要非空，因此缺失或显式null不能再利用`None==None`获得发布前确认。

【代码事实】没有笼统禁止PLANNING状态的反馈：已发布但尚未首次确认的正常请求、以及直接到HOLD_END的零运动保持仍按原流程处理。CLEARED/INVALID失败处理未改。符合上轮规定的最小修复范围。

## 2. 本轮独立执行的验证

【实验事实】本轮使用最终源码实际运行：

| 验证 | 结果 |
|---|---|
| `tests.test_uniform_feedback_identity` | 10/10通过，真实回调，无ROS节点启动 |
| 上轮Codex原脚本、原5项输入 | 5/5符合预期 |
| 额外发布后阶段正例 | 4/4通过 |
| 新提交源码清单 | 175项全部哈希匹配 |

原探针结果：已发布错误摘要HOLD_END保持PLAYING、无事件；匹配摘要HOLD_END正常到终态；发布前缺摘要HOLD_END、显式null HOLD_END均保持PLANNING、seen=False、无发布事件；发布前缺摘要HOLD_START也不确认。上轮复现的所有违规结果均消失。

额外正例直接调用真实回调：已发布身份正确时，PLANNING收到HOLD_START或PLAYING正常进入PLAYING；PLANNING收到HOLD_END正常完成；NO_MOTION直接终态也正常完成。单测另核对NO_MOTION状态词保留，未误标成普通运动。

完整证据：`feedback_units.log`、`feedback_phase_boundary_probe.py/json`、`probe_expectations.json`、`extended_feedback_probe.py/json/log`、`command_results.json`。

【代码事实，测试证据说明】两项名称带“missing_digest”的新单测调用`_deliver()`时没有显式删除默认摘要字段，因此这两项本身不能作为“字段确实缺失”的证据。显式null单测以及Codex原探针真实缺字段输入均已独立通过；本次以原探针作为缺失字段验证依据。实现合同已有实际验证覆盖，该单测命名/fixture细节不构成本轮阻塞，也不据此追加返工阶段。

## 3. 继承证据及验收范围

【实验事实】继承上一轮已经核实的182项回归、冻结参数的真实近目标ROS反例，以及`rework_20261114T095659Z`的main112项、near-goal8项、failure57项、tamper14项和可见RViz回放。上述本轮未重跑，不累加为本轮新执行。与上轮批准的守卫级验收范围一致。

【代码事实】本轮只新增拒绝“没有已发布模型”的非法确认，不改变已有合法模型的搜索、轨迹、时间轴、消息、Marker/TF或用户交互路径；正例已专门检查发布后PLANNING及零运动确认。因此没有证据要求重复完整30秒负例、全部ROS场景或GUI操作。

【实验事实】175项清单相符，上一轮Codex目录182个证据文件与其清单一致。旧原始实验、失败轮次及验收报告未被覆盖。FR01–FR04引用的12份已验收源码/测试哈希匹配；原仓库673个tracked、43个untracked、60个T5含忽略文件条目与保护快照相符（排除准备阶段已授权修订的三份设计文档），tracked二进制diff未变。

【尚未验证】本次没有新增狭窄环境成功率、全局最优性/完备性、闭环控制或动力学结论；固定10°圆弧动态界限制和启动阶段显示tick余量沿用既有披露。FR05通过表示冻结基线的ROS/RViz交互和同轨迹同步回放已达本阶段标准，不表示FR06已完成。

## 4. 最终状态与停止点

- FR05-R1：冻结目标容差与预算已恢复，关闭。
- FR05-R2：显示认证/反馈摘要绑定及发布前空摘要守卫均验证，关闭。
- FR05当前状态：**ACCEPTED**。
- FR01–FR04原验收保留；具备准备FR06窄拐角实验任务的前置能力，FR06未启动。

本轮源码快照在`reviewed_source/`，清单原文在`submitted_SOURCE_MANIFEST.sha256`，旧当前验收索引逐字节归档至`previous_acceptance/`。审查期间已有源码、原测试、配置、任务文档和提交证据无变化，见`after_checks.json`、`historical_protection.json`。

Codex只新增验收报告和独立输出，未修改实现/原测试/配置，未提交Git。完成验收后停止。
