# Claude G0 剩余返工：仅补一条既有路径

R2已经独立通过，不再改动。R1仅剩一个遗漏：G2任务书第20行明确要求的`tests/fixtures/virtual_a/open_requests.json`没有进入`STAGE_NEW_FILES['G2']`，当前timing/full仍会拒绝它。

请：

1. 将这个精确路径加入G2新增文件集合，不放开整个fixtures目录。
2. 用内存manifest验证：baseline/geometry拒绝，timing/full接受；未授权相邻fixture仍拒绝。不要创建真实G1/G2文件。
3. 新建唯一G0返工证据目录，保留旧证据；重新baseline与保护清单，保存本次差异、工具/说明哈希及实际命令结果。

除这一行名单及对应验证/说明外，不改其它实现，不进入G1，不修改算法、原测试/配置，不提交Git。完成后停止，交Codex复验。
