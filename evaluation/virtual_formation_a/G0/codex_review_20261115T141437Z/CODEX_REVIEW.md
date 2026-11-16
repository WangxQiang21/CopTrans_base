# G0 独立验收：REWORK

审查时间：2026-11-15。审查方：Codex。实现方：Claude Code。

**结论：基线、参考审核和依赖核验成立，但安全回归入口存在两项已确认的问题，G0暂不通过，禁止进入G1。** 本轮没有替Claude修改实现，没有提交Git，没有启动新算法或ROS。

## 1. 审查对象和边界

- 工作区 `/home/wq/whut_thesis/expv3_multi_virtual_a`。
- 分支 `feature/virtual-rigid-formation-a`。
- HEAD `567bdecba9676313da3bc9a97b82f57b5b569a2b`。
- Claude交付目录：`evaluation/virtual_formation_a/G0/20261115T140936Z/`。
- 工具SHA-256：`77d280314198debca090dea9472df69f7a0d9b0817e88b2fd511234b38ba8dcb`。
- 接口说明SHA-256：`97c5c411eead4c21abfbd2c70aa6c1af338ef2ce69f8ca6df9f9ef540b8917d3`。

独立核验脚本为本目录 `verify_submission.py`；只新增本次验收证据，Claude提交文件前后17项摘要均未改变。没有沿用此前被移出活动worktree的Codex提前产物作为Claude本次交付。

## 2. 已通过的部分

| 检查 | 本轮独立结果 |
|---|---|
| 提交source_manifest 16项文件摘要 | 全部匹配 |
| 新worktree原tracked文件对Git blob核对 | 3,550项全部匹配 |
| 原T5文件/状态 | 823项核对无变化，包括两份忽略的研究JSON |
| 原V0.2文件/状态 | 3,559项无变化，包括九份未提交Markdown |
| Claude工具重新CLI执行baseline | PASS；新结果单独保存于本目录fresh_baseline |
| 不调用提交helper、自己构造原请求复审参考 | FEASIBLE，13,390次检查，轨迹时长29.95529475329162s |
| 模型语义摘要 | `1da16fa1bdbe93e1bd3e55855b8fd900b6c4770f66a7ccbe8ae7eb368a5affde` |
| 非空输出、历史目录CLI输出、错误哈希、缺失文件/模型 | 按预期拒绝，未破坏旧资料 |
| 六个会写历史证据的旧测试模块输出常量 | 实际导入核对后均指向本次隔离目录；没有执行TestCase |

Claude保护清单为3,120项，独立Git blob检查补充核对了其范围外的原tracked文件。没有将清单条数不同直接当成数据丢失。

当前解释器 `/usr/bin/python3` 为Python3.8.10；numpy1.24.4、scipy1.10.1、casadi3.7.2、PyYAML5.3.1、Pillow10.4.0可用，实际IPOPT标量求解通过。rospy能否直接导入取决于是否加载ROS环境；新worktree尚无devel构建。本轮不验证G5，不将未运行ROS或旧测试写成通过。

## 3. R1：工作区门禁无法兼容后续已授权阶段（阻塞）

【代码事实】`tools/virtual_a_baseline.py`：

- `_classify_untracked:169`仅认识设计文档、G0工具/接口文档和新证据，其余一律unknown。
- `collect_manifest:223`把全部tracked差异列为unknown_modifications。
- `_workspace_gate:663`无阶段参数或已批准路径机制，存在任一上述差异即拒绝。
- `main:852`对baseline/geometry/timing/full采用同一门禁。

而已确认G1任务要求新增 `formation_continuous_a`、新测试/工具，并修改`setup.py`登记子包；任务书还要求在未提交状态执行：

`python3 -B tools/virtual_a_baseline.py --output ... --regression geometry`

所以实现G1之后，即使没有任何越界修改，该命令也会因其授权改动失败。新增包/测试会被标为unknown，`setup.py`也会触发相同问题。

【独立验证】本轮仅在内存manifest中模拟允许的`setup.py`差异，实际源码未动。门禁确实返回：

`UNKNOWN_WORKSPACE_MODIFICATIONS: [' M catkin_ws/src/fwis_v3_optimizer/setup.py']`

证据：`future_stage_gate.json`。这是已确认的后续接口冲突，不是要求提前实施G1。

**最小修复：** 保持G0 baseline严格，给后续回归提供有限、显式的阶段授权路径识别方式，与现有任务书白名单一致；允许已声明的新增文件和必要旧文件变化，同时仍拒绝未知/越界修改。可以采用小型阶段白名单或显式已批准清单，不需要通用权限框架。不得简单关闭门禁、允许所有dirty、自动提交或自动清理来绕过。

补充正反例：G0遇未知变更拒绝；G1允许的新增路径及setup修改被识别；相同阶段修改车辆YAML/旧算法仍拒绝。使用临时fixture、manifest或mock验证，不要实际实施G1或改变真实旧文件。

## 4. R2：公共run_regression没有输出保护（阻塞）

【代码事实】`run_regression:559`仅resolve输出路径，直接构造重定向目录、启动子进程，最后在`logs/<case>.stdout.log`等位置写入。它没有调用或共享`_prepare_output:679`中的路径归属检查，也没有拒绝已有case输出。

CLI main的正常路径有保护，这部分已通过；但任务书明确提供的公共函数没有相同保护。直接调用例如：

`run_regression(repo_root, repo_root/'evaluation/formation_refactor_v02/FR06', 'geometry')`

不会拒绝，其重定向和日志会落到历史FR06目录。重复调用同一输出位置还可能覆盖case日志和测试数据。

【独立验证】使用mock替代子进程与写操作，真实旧测试执行数为0、历史目录实际写入数为0；函数仍准备写入历史FR06的四个日志路径。证据：`public_runner_guard.json`。

**最小修复：** 公共runner必须在任何子进程/写操作前验证输出归属、拒绝历史/其它worktree目录及已有case输出覆盖；与CLI共用一致路径规则。注意main调用runner时output已含manifest等本次元数据，不能简单要求整个output完全为空而破坏正常CLI。可以检查独立的regression_artifacts/case及对应日志，不覆盖已有case结果。

补充正反例：直接公共调用的历史/外部路径、已有case日志/artifact必须在子进程启动前拒绝；唯一新目录正常；正常CLI仍通过。不要实际向历史目录试写，继续用mock验证拒绝发生在前。

## 5. 非阻塞说明

- 当前full列表覆盖FR01–FR04主要单元项，未包含FR05的播放/反馈单元测试；G5最终验收仍需按G5任务书补齐这些测试及真实ROS。不能把本工具full的PASS冒称完整G5通过。本次不要求启动ROS或新增G5能力。
- 1,018行工具是否需要精简不作为本轮返工目标；不要为了风格重写工具。只修R1/R2及其必要验证/说明。
- 当前没有发现需要改变研究模型、参数或整体算法方向的问题。

## 6. 返工范围和停止条件

允许：`tools/virtual_a_baseline.py`、必要的`G0_INTERFACE_CHECK.md`说明更新，以及唯一的新G0返工证据目录。保护已提交证据原件，保留本轮失败结果。

禁止：G1实现、新规划包、算法/测试/ROS配置修改、旧限制放宽、Git提交、删除历史证据。无需重跑六次FR06搜索、完整旧测试或ROS。

返工后交付两项修复的代码差异、明确正反例和真实输出、重新baseline审核及保护清单，停止并交Codex复验。Codex本轮不替实现方修改工具，也不签发ACCEPTED。

**G0 = REWORK；下一阶段授权 = 不允许。**
