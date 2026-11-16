# Claude G2 返工 02：auto 求解器超时后错误回退

## 结论与范围

G2 返工 01 已修复 NaN 审核放行，并补充独立 Simpson 积分误差预算；定向审核测试和 G1 回归测试通过。但 Codex 发现 `solver='auto'` 的超时处理仍存在遗漏，G2 本次 **NOT ACCEPTED**。

只修复本文件的软时限控制和相关测试、证据，不修改数学模型、物理限制、原有 G1 测试预期或旧 FR06，不进入 G3，不提交 Git。工作区 `/home/wq/whut_thesis/expv3_multi_virtual_a`，分支 `feature/virtual-rigid-formation-a`，HEAD `567bdecba9676313da3bc9a97b82f57b5b569a2b`。保留 G2 原运行及返工 01 运行，使用新的唯一证据目录。

## 已确认问题

`formation_continuous_a/optimizer.py` 的 `solve_open_path` 在 `solver='auto'` 且 `allow_scipy_fallback=True` 时，先按 `not result['success']` 进入 SLSQP 回退，之后才检查 `result['soft_deadline_triggered']`。因此 IPOPT 返回 `Maximum_WallTime_Exceeded` 后，代码仍会尝试备用求解器；如果备用求解器成功，则覆盖原结果、遗漏 IPOPT 超时记录，最终可能返回 `GEOMETRY_FEASIBLE` 且 `deadline_exhausted=False`。

Codex 用真实 `STRAIGHT_LINE` 请求和默认车辆模型，mock 两个 solver 的边界结果进行确定性复现：IPOPT 返回 `success=False, soft_deadline_triggered=True, solver_status='Maximum_WallTime_Exceeded'`，SLSQP 返回可通过真实独立审核的 6 m 直线解；实际结果为：

```text
fallback_called,remaining=29.999...
status=GEOMETRY_FEASIBLE deadline_exhausted=False trigger=''
attempt=SOLVER_SUCCESS_AUDITED solver=scipy
```

此复现使用 mock 精确模拟求解器状态，证明控制流缺陷；它不声称默认 `solver='ipopt'` 的冻结实验出现过该问题，也不声称已审核的回退路径几何错误。现有超时测试只覆盖默认 IPOPT 分支，未覆盖 auto 分支。代码同时可能在 IPOPT 异常后已无剩余预算时仍启动备用求解器。

## 修改要求

1. 在任何回退之前处理超时状态；若 IPOPT 声明软时限触发，立即保留该 solver 状态及触发原因，记录 `SOLVER_TIMEOUT`，不启动 SLSQP，不审核其超时迭代结果。无先前审核通过的候选时返回 `BUDGET_EXHAUSTED`；有先前已审核候选时允许按既有合同保留它，同时明确记录触发时限。
2. 普通求解失败或异常允许按原合同回退，但启动备用求解器之前必须重新检查整个请求的剩余软预算；无剩余预算则记录真实触发位置并停止，不得启动新求解器。不得吞掉普通非超时求解异常的原因。
3. 备用求解器本身若触发超时，也应完整记录且不得审核其超时迭代值；状态、来源、超时点及已有候选语义应一致。保持软时限的非硬实时性质，不承诺库能即时中断。
4. 若普通 IPOPT 失败后 SLSQP 成功，允许采用经独立审核的回退结果，但日志应保留 IPOPT 原失败及回退信息，不得把失败重写为纯成功历史。

## 必须增加的测试

- `auto` + IPOPT 超时：mock IPOPT 超时，并让任何 SLSQP 调用即抛错；断言 SLSQP 绝不执行、审核绝不接收超时迭代、返回 `BUDGET_EXHAUSTED`、超时来源/状态正确。
- `auto` + 首个已审核候选后 IPOPT 超时：保留已审核候选，断言第二次不触发 fallback，记录 `deadline_exhausted=True` 及 `SOLVER_TIMEOUT`。
- `auto` + IPOPT 异常后预算耗尽：使用可控时钟或等价确定性方法，断言 SLSQP 不执行，真实记录异常和预算耗尽。
- `auto` + 非超时失败且仍有预算：证明合法 SLSQP 回退和独立审核仍能工作，并保存前序失败原因。
- `auto` + SLSQP 超时：不得审核超时迭代；正确报告触发来源。
- 保留原有真实 IPOPT 超时、真实 SLSQP 回调和超时后已审核候选测试；mock 控制流测试不能替代已有真实求解器测试。

## 验证与交付

重跑 `test_virtual_a_optimizer.py`、`test_virtual_a_audit.py`、G1 geometry/profile 回归和完整 G2 benchmark；保持六个冻结端点、直线及 STRAIGHT 的原配置及验收门槛。记录命令、退出码、触发路径、实际 solver 状态、所有 attempt、源码哈希及 G1/旧 FR06/既有 G2 证据保护差异。为本返工生成新的 `CLAUDE_REPORT.md` 和独立证据目录。

完成后停止，等待 Codex 复验。Codex 将独立复现上述 auto 分支，检查正常 fallback、双求解器超时和预算耗尽路径，以及冻结实验和保护证据。不得自行进入 G3。
