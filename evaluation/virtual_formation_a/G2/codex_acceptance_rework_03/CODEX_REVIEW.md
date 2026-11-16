# G2 第三次返工独立验收

结论：**ACCEPTED**。验收范围是连续 A 模式、端点驱动的自主几何规划及独立几何审核；不包含时间参数化、动态约束、障碍碰撞、ROS 回放、硬件执行或全局完备性。

## 复核依据

- 审查 `optimizer.py`：成功的 IPOPT 和 SLSQP 回退候选均在构造路径及审核前计算 `_constraint_residual_diagnostic`；无效向量、异常或非有限残差产生失败记录，未作为候选送审。残差仅是 NLP 配点诊断，独立全段审核仍是几何通过条件。
- 独立读取返工 03 `analysis/attempts.json`：48 个 `SOLVER_SUCCESS_AUDITED` 记录均有有限非负约束残差，最大 `4.707345624410664e-14`。先前 null 缺陷已关闭。测试包括真实 IPOPT 和 SLSQP 回退残差复算及异常负例。
- 复查返工 02 的超时控制：primary 超时在回退前处理；SLSQP 前再次检查预算；超时向量不送审。返工 02 中独立重现的 auto 超时反例已拒绝并正确记录。求解超时仍是软限制，不保证硬实时。
- 独立重跑：`test_virtual_a_optimizer.py` 26 项、`test_virtual_a_audit.py` 31 项、G1 `test_virtual_a_geometry.py` 20 项、`test_virtual_a_profile.py` 22 项全部通过。G2 运行记录含八个冻结请求及 124 项 benchmark 检查通过；没有将 solver success 当作独立审核通过。
- 返工 01 的 NaN 配置反例已独立复现为 `INVALID_AUDIT_CONFIG`，独立积分 Simpson 余项曾按四段记录重新计算，预算与记录一致。该预算含文档化浮点假设，不构成形式化 IEEE 浮点证明。
- 重新运行返工 03 的证据清单校验，48 次尝试、124 项 benchmark、源码及 G0/G1 保护快照检查通过。原 G2 历史运行和 G1 已验收源码的保护差异记录无未授权修改；Git 未提交。

具体代码和日志绑定见 `evaluation/virtual_formation_a/G2/20261115T174526Z/source_manifest.json`；本次验收绑定该清单的 SHA-256。测试是有限案例证据，不表示任意端点必然有解或规划结果具备动态、碰撞及执行可行性。下一阶段须等待用户明确授权。
