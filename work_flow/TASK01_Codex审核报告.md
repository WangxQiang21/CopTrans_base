# TASK 01 独立代码审核

日期：2026-11-19  
结论：**PASS（返工 01 已通过独立复核）。**

## 审核范围

- 基线：`77805cd46849ebadc374187f9b00fce0f7151f84`
- TASK 01 代码提交：`91dd71f619d968099622570f669a78f9865c7118`
- 当前 HEAD：`222b919329bf487db1ee096e2902d753385a3527`
- 用户已授权提交；提交本身不作为审核问题。
- 审核聚焦 TASK 01 新增的 v3 数值证书、规划进度/回放接线、研究入口、winding 与 S 域，不重复审查 FR06 历史模块。未修改代码。

## 返工问题及复核结果

原问题位于 `geometry.py` v3 数值计划：全负曲率段把带符号上界当作绝对曲率上界，可能低估角度误差界。

[geometry.py](/home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py:739) 现以 `max(abs(k_lower), abs(k_upper))` 更新 `max_kappa`。只修改 v3；v2 计算代码未改。

新增 `test_virtual_a_geometry.py` 四项：纯负曲率、负向占主导的多段曲率、v2 字节级重放、超过角度预算时 fail closed。还提供了修复前负向对照：三项负曲率断言按预期失败；恢复修复后四项通过。

独立复核确认 v3 使用绝对包络，v2 fixture 的 model SHA 和序列化字节保持一致。修复前后新旧数值证据均保留；返工后的 600003/600004 v3 角度界、位置界均在原阈值内，三项审核通过。

## 已核验通过项

- v3 分段 M4 与生产浮点运算预算按真实 Simpson 网格计算；模型版本显式绑定，v2 重建路径保留。
- 搜索进度与候选可视化走诊断通道；执行轨迹仍只从三项审核通过后的原子 playback envelope 发布。失败路径写入空 tombstone，避免迟到订阅者读取旧成功轨迹。
- winding 候选有界，分支进入同一 search packet；S 上界有限且纳入 guide 长度，不再使用旧 60 m 截断。
- 工作区基线至代码提交的差异通过 `git diff --check`；用户已有 v2 numerics、retiming 等基线文件未被修改。

## 独立验证

返工前独立运行的 4 组定向测试共 **452 项通过**。返工后，我重新运行受影响的数值/路径/审核测试，共 **156 项全部通过**：

- `test_virtual_a_geometry`、`test_virtual_a_profile`、`test_virtual_a_integration_bounds`、`test_virtual_a_trajectory`、`test_virtual_a_audit_evaluation_guard`、`test_virtual_a_audit`。

返工后另外独立运行原 Step 5 verifier 与返工 verifier，均核验 5 份文档通过；`git diff --check` 通过。独立测试命令退出码 0，156 项全部 `OK`。全量 unittest discover 的既有 `dynamic_peaks` KeyError 仍是基线问题，不归因于本次返工，也不记作通过。

返工证据：`work_flow/TASK01_evidence/09_rework_01/`，包含摘要、工作区状态、完整源码差异、哈希、正负对照测试日志、修复后数值探针及 verifier 输出。

## 非阻塞验收边界

- 现有证据支持离线测试和 ROS 冒烟结果；最终 Path 诊断显示改动后未保存一次完整虚拟 A GUI 回放证据。因此代码合入状态不等同于用户 RViz 视觉验收。
- 失败时发布空 playback tombstone 是一条 ROS 消息，但不含可执行 envelope；统计“发布次数”时应区分消息数与 executable envelope 数。

本结论是代码与现有离线/ROS 证据审核通过；RViz 最终 Path 诊断层的用户目视确认仍单独进行。返工改动尚未提交；此次验收不自动授权开始 TASK 02。
