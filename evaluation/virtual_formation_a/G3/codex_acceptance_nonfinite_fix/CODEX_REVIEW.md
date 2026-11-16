# G3 非有限值修复独立验收

结论：**ACCEPTED**。本验收对 G3/G4 既有验收增加新源码哈希绑定；不覆盖历史验收记录。

- 审阅 `audit.py:_audit_actual_finite`、`_audit_actual_metrics` 和 `_audit_check_actual_metrics`：实际指标及轮角在比较前检查有限性，`C_i` 在转换轮角前检查；拒绝使用 `NONFINITE_AUDIT_VALUE` 和车辆/轮/指标定位，非有限值以文本描述，有限超限选择仍沿用最不利利用率。
- 独立重现 `wheel_speed=NaN/+inf/-inf`，三者均返回 `NONFINITE_AUDIT_VALUE`，定位车辆 1、轮 0、`wheel_speed`。独立重跑 `test_virtual_a_audit.py` 64 项、`test_virtual_a_collision.py` 17 项通过；测试包含受控注入下完整动态审核拒绝路径。
- 运行返工证据 manifest 校验通过；返工记录还包含 timing 18、trajectory 23、pipeline 8 项及 G4 冻结 S1 无参考输入端到端回归通过，绑定同一请求、地图、上下文与模型摘要。这些额外测试以返工日志为依据，本次未重复完整重跑。
- 保护记录对照 G4 已验收清单，除授权的 `audit.py`、`tests/test_virtual_a_audit.py` 外其余被绑定源文件未改变；G3/G4 历史验收文件保留。原 legacy `timing_continuous` 既有 `KeyError: dynamic_peaks` 本次未运行、未修复，不计为通过。

本次验收未证明真实运动曾自然产生非有限值，也不改变 G3/G4 的实验范围。G5 已由用户明确授权，依原任务书及配套基线补充文件执行，完成后停止等待独立验收。
