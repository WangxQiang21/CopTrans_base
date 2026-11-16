# G0 返工独立复验：REWORK（仅剩R1的一条白名单漏项）

日期：2026-11-15。实现方Claude Code；独立审查Codex。

**R2已关闭；R1的G1门禁冲突已修复，但G2任务书必需的一个fixture路径仍被拒绝。G0暂不签发ACCEPTED。** 这不是新增研究任务，而是上次R1“识别既有任务书授权路径”的一个遗漏。

## 1. 当前提交与审查范围

- 工作区：`/home/wq/whut_thesis/expv3_multi_virtual_a`。
- 分支：`feature/virtual-rigid-formation-a`；HEAD仍为`567bdecba9676313da3bc9a97b82f57b5b569a2b`。
- 本次Claude交付：`evaluation/virtual_formation_a/G0/20261115T143321Z_rework/`。
- 工具SHA-256：`4bf3c9468d09c7554d9006361d5c94081b3c8ba2b941eca3edc92208dcbfb92e`。
- 接口文档SHA-256：`0abad6582d3bf53e2454264bf3ad0bee72e161812569298c7d04d32519a739bd`。

已阅读全文中新增的阶段名单、分类、门禁、公共runner预检与CLI调用分派，核查返工证据25项文件摘要，无不匹配。本目录保存了本次实际代码和接口文档副本，避免后续仅剩摘要而无法复核版本。

## 2. R2已关闭的独立依据

公共runner与CLI现在共用`_resolve_evidence_output`，启动子进程前用`_guard_regression_outputs`预检所有case。

Codex另外构造9项验证：历史FR06目录、外部目录、证据根、已有case artifact、已有case日志、第二个case已有输出、指向历史目录的符号链接、含main元数据的新run目录、全新run目录。

- 7个拒绝例均在子进程/写入前拒绝，mock调用次数为0。
- 2个正例可正常构造两个几何case，不因main已有manifest元数据误拒。
- 实际旧测试执行数为0，历史目录实际写入数为0；正例子进程和写入均为mock，不冒称真实回归通过。

证据：`independent_runner_guards.json`。本轮不再要求修改R2。

## 3. R1剩余项：必需的G2 fixture没有进入授权集合

【既有合同】`docs/virtual_formation_a/CLAUDE_G2_TASK.md:20`明确新增：

`tests/fixtures/virtual_a/open_requests.json`

【代码事实】`tools/virtual_a_baseline.py:125-130`的`STAGE_NEW_FILES['G2']`仅有optimizer、audit及两个测试文件，没有这个JSON；`STAGE_NEW_PREFIXES:151`也只允许G4的maps子目录。

【独立复现】将该路径放入纯内存manifest（未创建真实G2文件），用后续`level='timing'`调用门禁，仍得到：

```text
UNKNOWN_WORKSPACE_MODIFICATIONS: ['tests/fixtures/virtual_a/open_requests.json']
```

所以G2按任务书交付后，后续G3规定的timing回归会被合法fixture阻塞；full的累计名单也缺该路径。这与上次R1是同一问题。

本轮逐项检查G1–G5任务书中26条明确新增路径，**其余25条均按对应累计阶段允许，唯一漏项就是上述JSON**。另独立核对setup允许、baseline严格拒绝、旧算法/车辆YAML/删除操作拒绝、G5节点只在full允许，均符合要求。

证据：`authorized_fixture_probe.json`、`task_path_matrix.json`、`gate_positive_negative.json`。

### 唯一剩余修正

把`tests/fixtures/virtual_a/open_requests.json`加入G2明确新增文件集合即可，不要泛化放开整个fixtures目录。

补定向测试：

- baseline/G0拒绝该G2路径；
- geometry/G1拒绝；
- timing/G3接受该已授权G2路径；
- full/G5接受；
- 未授权的相邻fixture仍拒绝。

这些只用manifest模拟，不创建G1/G2实现。R2和其它阶段名单没有本轮新增修改要求。

## 4. 基线和既有能力继续有效

| 本轮复核 | 结果 |
|---|---|
| 工具重新CLI执行baseline | PASS，单独保留fresh_baseline，不覆盖Claude交付 |
| 参考轨迹 | FEASIBLE，13,390次检查，T=29.95529475329162s |
| IPOPT标量依赖探测 | PASS；不等于新NLP已实现 |
| 非空run拒绝与manifest复核 | 符合预期 |
| 原tracked文件对Git blob核对 | 3,550项无变化 |
| 原T5工作区 | 823项文件与状态无变化 |
| 原V0.2工作区 | 3,559项文件与状态无变化 |
| 当前Claude提交文件 | 验收前后摘要无漂移 |

没有修改实现、旧算法、测试、车辆参数、地图、历史证据；没有提交Git、运行旧完整测试、FR06搜索、ROS或启动G1。未执行项不标通过。

## 5. 返工与停止

仅补上述一条G2白名单、相应定向模拟证据及新run的baseline/保护清单。不改变任务书、算法路线、安全限制或R2；不重跑大规模实验。

保留本轮及此前所有证据。完成后交Codex复验，未经G0 ACCEPTED及用户明确下达，不进入G1。

**状态：G0=REWORK；R2=CLOSED；R1=一条既有授权路径待补。**
