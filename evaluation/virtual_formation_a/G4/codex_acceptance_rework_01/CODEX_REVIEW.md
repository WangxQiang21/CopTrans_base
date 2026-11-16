# G4 返工 01 独立验收

结论：**ACCEPTED**。范围为六个冻结 A-only 场景的编队地图引导、公共时间轨迹和几何/动态/保守扫掠碰撞审核；不表示一般地图的完备性、实时性或实车可执行性。

## 审核依据

- 对照旧 `formation_collision.py` 的 `(shape_dimension + margin_m)/2` 半尺寸语义审查新 `collision.py:error_envelope`：数值包络按每侧增量覆盖，并用实际 margin 对应的半对角线求固定点；扫掠运动 padding 保持单独作用。独立用 `Fraction` 重算六个场景各自的参考和自主结果，共 12 个，实际 checker 每侧半尺寸增量均不小于对应数值包络。
- `integration_bounds.py` 从 `f=exp(iψ)` 推导 `f''''/f = (k⁴-4ku-3eta²)+i(g-6k²eta)`，按 Simpson 完整面板使用向外区间运算和独立积分余项，保留既有角度/浮点保护假设。独立读取 12 个参考/自主记录，其审核预算均不超过 `5e-8 m`；S2 参考为 `4.535685842243642e-8 m`。全域余项依据区间运算，采样只作对照。
- 独立重跑 `test_virtual_a_collision.py` 17 项、`test_virtual_a_integration_bounds.py` 6 项、`test_virtual_a_pipeline.py` 8 项（含六个场景的自主端到端求解）、G3 `test_virtual_a_audit.py` 51 项及 `test_virtual_a_timing.py` 18 项，均通过。返工证据记录 G4 benchmark 84 项通过、六场景三层审核通过；本次未单独重跑完整 benchmark 或全部 G1–G3 套件。
- 独立运行 `analysis/verify_source_manifest.py`：源码和运行证据哈希、冻结场景与保护清单校验通过，旧 checker 和历史数据在记录的保护范围内未修改。旧 G3 legacy timing 回归中 `timing_continuous` 的 `KeyError: dynamic_peaks` 为既有失败，不计为通过。

G3 记录的 `_audit_check_actual_metrics` 非有限值被跳过的问题仍未修复；此前未发现它单独绕过全段证书使结果通过的端到端反例，继续作为非阻塞维护问题记录。积分与角度保护包含文档化浮点假设，并非形式化浮点证明。`executable` 是当前规划模型中的通过状态，不代表控制器、ROS 回放或真实硬件已验证。G5 须用户明确授权。
