# Claude G2 返工 03：恢复成功求解的约束残差记录

## 复核结论

G2 第二次返工已修复 `auto` 模式 IPOPT 超时后错误回退的问题。Codex 独立复现确认：IPOPT 超时时返回 `BUDGET_EXHAUSTED`，记录 `SOLVER_TIMEOUT`，不会启动 SLSQP，也不会审核超时迭代值。

但这次改动引入一处证据记录回退：`optimizer.py:solve_open_path` 现在只对失败的 primary/fallback 求解结果计算 `_constraint_residual`，对成功的 `candidate_result` 没有计算；`_attempt_record` 因而记录 `constraint_residual=None`。在 `evaluation/virtual_formation_a/G2/20261115T172337Z/analysis/attempts.json` 中，**48 次成功尝试的约束残差全部为 null**。这不证明几何结果有误，但不符合 G2 原任务书第 2、7 节“每次 attempt 记录目标/约束残差及完整求解证据”的要求。因此本次暂不验收 G2。

## 最小返工范围

工作区 `/home/wq/whut_thesis/expv3_multi_virtual_a`，分支 `feature/virtual-rigid-formation-a`，HEAD `567bdecba9676313da3bc9a97b82f57b5b569a2b`。仅修复成功候选的约束残差记录及必要测试和新证据；保持已有独立审核、数学模型、车辆限制、软时限语义和旧 FR06，不修改已通过测试预期，不进入 G3，不提交 Git。保留全部既有 G2 证据。

## 实现要求

1. 对每个可用且有限的成功 solver 决策向量，包括 IPOPT 直接成功及 SLSQP 合法回退成功，在记录 attempt 时计算并保存 `_constraint_residual`；记录数值应有限、非负、可独立复算。该数值为 NLP 配点约束残差诊断，**不代替独立全段几何审核**。
2. 对求解失败、超时、没有可用向量或向量非有限的情况，可以使用 null，但保留明确状态及原因；超时迭代仍不得作为成功候选或传给独立审核。残差计算出现异常或非有限时要失败关闭并如实记录，不能伪装为 0 或让整个规划器抛出未处理异常。
3. 增加测试：真实 IPOPT 成功和真实或受控的 SLSQP 回退成功均有有限残差，并与单独调用 `_constraint_residual` 的结果一致；无向量及非有限向量不误报数值；原有超时分支不变。

## 验证与交付

- 重跑 `test_virtual_a_optimizer.py`、`test_virtual_a_audit.py` 和 G2 benchmark；保留六个冻结端点、直线和 STRAIGHT 的原配置与门槛。已有 G1 代码和测试未改时，保留并引用上一轮已通过的 G1 回归证据；若改动 G1 相关文件，必须重跑 G1 两组测试并单列说明。
- 在新的 `evaluation/virtual_formation_a/G2/<唯一run-id>/` 中保存完整 attempt 证据，逐项说明残差、solver、状态、初值和原请求；列出成功尝试总数、残差非 null 且有限的数量，并留存代码、结果、日志 SHA-256 和旧证据保护差异。
- 记录完整执行命令、退出码及 `CLAUDE_REPORT.md`。完成后停止，等待 Codex 独立复核；不自动进入 G3。

## Codex 验收要点

独立检查 IPOPT 和 SLSQP 成功路径的残差计算及数值、残差异常失败关闭、超时不得被审核或触发回退，并核对冻结实验、旧证据和保护范围。只有上述合同及既有 G2 要求均满足，才记录 G2 ACCEPTED。
