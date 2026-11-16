# G3 返工 01 独立验收

结论：**ACCEPTED**，范围为固定几何路径的公共五次时间律和三车动态审核；不包含地图碰撞、ROS 回放、硬件执行或时间最优。

## 复核依据

- 审查 `timing.py:_vehicle_bounds` 和 `audit.py:_audit_interval_vehicle`：三阶空间导数 x 分量已改为 `-k²+b*k³-3*a*k*eta-b*u`，与独立推导和点态几何一致。独立重算原反例，真实范数 `0.10677546534667971`，两套 jerk 上界均为 `0.10677546534667996`，覆盖真实值。
- 返工记录包含窄区间、不同曲率符号及三车分量覆盖、镜像 C2 路径、节点左右极限、缩短时间的真实 jerk 超限和无法确认时失败关闭等验证。数值采样用于对照，不能替代区间上界。
- 本次独立重跑 G3 timing 18 项、trajectory 23 项、audit 51 项，以及 G1 geometry 20 项、profile 22 项，全部通过。返工记录中 G2 optimizer 26 项及 G3 benchmark 151 项通过；本次未独立重跑完整 benchmark 或 G2 优化套件。
- 独立运行返工证据清单校验通过，包含 9 个模型、151 个 benchmark 检查、31 个源码文件和旧 G2 证据；记录的旧版保护差异无未授权删除。历史 legacy timing 总回归为 **FAIL**，其中 `timing_continuous` 的 `KeyError: dynamic_peaks` 在 pristine HEAD 探针也已复现，不能将其计作通过。原始 gate 因白名单拒绝；披露过的 wrapper 执行了回归。

## 非阻塞缺陷与限制

`audit.py:_audit_check_actual_metrics` 对非有限 metric 使用 `if not math.isfinite(value) or value <= limit: continue`，直接向该内部函数传入 NaN 或 infinity 可返回 True；此处应改为明确拒绝非有限值并添加测试。调用点位于区间不能确认时的见证/细分路径，未发现它单独绕过整段证书、使此缺陷导致 `DYNAMIC_FEASIBLE` 的端到端反例；现有正常有限值实验及本次 jerk 上界修复不依赖它。此项记录为后续维护问题，不将该内部函数的非有限输入行为视为已验证正确。

本验收基于有限测试与记录的区间运算、浮点保护假设，不是硬件执行或形式化浮点证明。G4 仍须用户明确授权。
