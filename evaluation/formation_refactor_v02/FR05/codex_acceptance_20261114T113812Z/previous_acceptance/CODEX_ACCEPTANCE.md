# FR05 当前验收：CHANGES_REQUESTED

2026-11-14，Codex独立审核。168项回归通过，五份保存信封解析一致性复核通过；实际ROS反例和两个身份探针确认两项阻塞：

1. **FR05-R1：ROS默认合同漂移。** 目标容差变成0.16m/0.16rad、预算变成50000次/20秒。真实请求相差0.1m/0.1rad却以零时长NO_MOTION、可执行、HOLD_END发布。
2. **FR05-R2：认证/终态未绑定模型摘要。** 同plan_id不同摘要的模型可借用旧审核标为FEASIBLE，错误摘要的HOLD_END也会释放busy。

两项均限于FR05接口层修复，不要求重构算法。另有两份源码与提交清单哈希不符，修复后须提供与最终源码对应的验证证据。

完整复现、审查范围与可直接交给Claude的最小返工任务见 [本轮报告](codex_acceptance_20261114T093751Z/ACCEPTANCE.md)；机器结论见 [CODEX_ACCEPTANCE.json](CODEX_ACCEPTANCE.json)。

Codex未修改算法/测试/配置，未提交Git；原T5及FR01–FR04验收基线保留，未进入FR06。
