# G5 执行基线补充：G3 非有限值修复已验收

用户已明确授权进入 G5。请与 `CLAUDE_G5_TASK.md` 一起阅读并按其完成 G5；开发由 Claude Code 执行，完成后停止等待 Codex 独立验收。保留原有 G5 范围、测试和停止要求，不增加控制、Gazebo、实车或动态重规划。

G0–G4 原验收记录不变。G4 ACCEPTED 基线文件为 `evaluation/virtual_formation_a/G4/codex_acceptance_rework_01/CODEX_ACCEPTANCE.json`；原 G4 运行及冻结场景清单为 `evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json`，SHA-256 `f956e8316ebf1e72102d89338b88bbcabe48bae5aa52977deef3670abe24240d`。实际地图与请求取清单对应的 `scenes/inputs/<scene>/`，并按 G5 任务书核对哈希，不能使用参考曲线替代自主规划。

G4 原验收之后，仅 `formation_continuous_a/audit.py` 和 `tests/test_virtual_a_audit.py` 经用户授权修复 G3 的非有限值问题，已由 Codex 以 `evaluation/virtual_formation_a/G3/codex_acceptance_nonfinite_fix/CODEX_ACCEPTANCE.json` 补充验收。当前两文件 SHA-256 分别为：

```text
audit.py                    54dda61ffc273e61be31eeeba3232dd896f00605c864a1b05056fc2de6620e5d
test_virtual_a_audit.py     7b7baa9a780de5c166f3abb37f134efdf4a5e4de81a57adf75668a00a04b33a7
```

G5 开始前同时核验 G4 原清单和上述补充验收：原清单中这两个文件按补充验收哈希核对，其他文件仍按原清单核对；不得覆盖原验收或历史证据。将新的设计文档和验收补充视作已授权文件，而不是删除它们以通过静态白名单。

G5 按原任务书的 G5-A → G5-B → G5-C 执行。真实 ROS 或 GUI 因环境/权限无法执行时如实记录 `NOT_RUN` 及原因，不能声称真实验证通过；不用模拟数据替代。历史 legacy `timing_continuous` 的 `KeyError: dynamic_peaks` 是既有未修复失败，不得记为通过，也不能擅自修改旧测试掩盖。不得提交 Git，完成后停止等待 Codex 验收。
