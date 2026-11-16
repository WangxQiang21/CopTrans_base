# Claude G2 返工 01：独立审核的失败关闭与积分误差证据

## 结论与范围

Codex 对 G2 首次交付的结论为 **NOT ACCEPTED，需返工**。G2 已实现端点驱动的 NLP、独立审核及规定场景的实验，但独立审核存在可复现的错误放行，且独立积分缺少任务书要求的余项预算。仅处理本文件问题，继续遵守 `CLAUDE_G2_TASK.md` 和已验收 G1 合同。不得进入 G3、修改旧 FR06、放宽物理限制、删除负例或提交 Git。

工作区：`/home/wq/whut_thesis/expv3_multi_virtual_a`；分支：`feature/virtual-rigid-formation-a`；基线 HEAD：`567bdecba9676313da3bc9a97b82f57b5b569a2b`。保留 `evaluation/virtual_formation_a/G2/20261115T161608Z/` 的原始证据，新建唯一返工运行目录，禁止覆盖历史证据。

## P0：审核配置非有限值可使错误轨迹通过

`formation_continuous_a/audit.py:941` 的 `audit_geometry` 只检查部分配置字段是否为 `int/float` 并比较大小；没有全面检查 `math.isfinite`，也没有覆盖 `c2_value_tol`、`c2_slope_tol`、`c2_second_tol` 等字段。NaN 的大小比较不成立，导致对应的拒绝判断失效。

Codex 独立复现：从 `tests/test_virtual_a_audit.py` 的 helper 构造真实请求，目标 `(10,0,0)`；使用长度 6 m 的合法直线路径，绑定该请求指纹；调用 `audit_geometry(request, path, GeometryAuditConfig(position_tolerance_m=float('nan')))`，结果 `passed=True`，同时独立终点误差报告为 **4.0 m**。这不是求解器成功率问题，而是审核合同被绕过。

**要求：**

1. 在任何使用配置或执行 hold 分支前统一验证 `GeometryAuditConfig` 的每个字段：数值必须有限，域合法；整数计数必须是合法整数而非 bool；C2、终点、刚性、积分和角度容差及第三导数上界须有明确正值/非负域。无效配置一律返回 `INVALID_AUDIT_CONFIG`，不能抛出未捕获异常或通过。
2. 增加 NaN、正负无穷、非法类型、负值和非法计数的参数化负例，覆盖所有相关字段，尤其 `position_tolerance_m`、`heading_tolerance_rad`、`rigidity_tolerance_m`、`integration_budget_m`、`angle_budget_rad` 和三个 C2 容差。对非有限中间结果同样失败关闭。
3. 固定上述 **6 m 路径 / 10 m 目标 / NaN 容差**真实反例，断言审核拒绝。正常默认配置下同样路径须因终点误差拒绝；正常合法路径须继续通过。不得仅 mock 审核结果。
4. 检查调用链中的配置转换，确保不能通过求解配置把 NaN 或非法容差带入审核。保留原任务的数值标准，不能以扩大默认容差来通过。

## P1：独立终点积分没有自身余项预算

`audit.py:_integrate_xy` 每段固定使用 32 点 Gauss-Legendre；`_endpoint_audit` 读取的是 `path.position_error_bound_m`（G1 路径自己的预算），然后以 `config.integration_budget_m + 1e-9` 比较两者。代码没有计算或记录 Gauss 审核积分自身的截断余项上界，因而与 G2 任务书第 4 节“更严格位置积分及其余项预算”不一致。32 点数值通常很准确，但点数和两套积分接近，不能单独构成其误差上界。

**要求：**

1. 基于从原始结点独立重建的曲率多项式，给审核侧积分提供明确、可复核的截断误差预算，包含采用的方法、每段导数界/面板数量、每坐标及二维合成值和浮点/角度保护假设。可以复用 G1 的通用外扩算子和 Simpson 余项工具，但不能直接复制路径自报预算当作审核预算；可以选择另一种有数学依据的积分及上界。
2. 按任务书实现比 G1 原 `1e-7 m` 合同更严格的审核预算，并在证据中记录固定阈值、实际值及其依据；若无法确认或超过审核预算，返回 `BOUND_UNCONFIRMED` 或明确失败，不得标为合格。数值检查中计入 G1 轨迹和审核积分各自的误差预算，不以采样差充当数学余项证明。明确浮点保护仍为数值假设。
3. 给出非零四阶导数的合法 C2 曲率路径和多段路径，用独立更高精度积分进行数值对照；覆盖误差预算可确认、细分以及资源限制下不可确认的负例。检查最终与原始请求目标的误差，以及返回路径与独立积分一致性。不得放宽 G1 的位置和角度预算。

## P1：软时限的求解器迭代边界检查

G2 原合同要求在初值和迭代边界检查 30 s 软预算；当前 `optimizer.py:671` 只在一次 solver 调用前后检查，IPOPT 内未检查。报告已如实说明没有轮询，但实现尚未符合任务书。

请为可支持的求解器增加有明确行为的剩余软预算检查，优先使用其可用的迭代回调或求解器时间限制，并记录触发位置、超时与已取得候选的处理；不要声称硬实时或保证库调用一定即时中断。SLSQP 若可通过 callback 检查也应覆盖。构造可重复测试，证明触发后无未经审核的成功结果，且已审核候选的处理、超时状态和日志真实；若底层接口确实无法支持，提供具体实验与限制，停止并交 Codex 判断，不得自行更改合同。

## 允许修改和交付

- 可修改 G2 的 `formation_continuous_a/audit.py`、`optimizer.py`、`contracts.py`，以及 `tests/test_virtual_a_audit.py`、`tests/test_virtual_a_optimizer.py`、G2 benchmark 与新的 G2 证据；若 `bounds.py` / `geometry.py` 必须修改，单列理由和差异，重跑全部 G1 测试并接受 Codex 对 G1 合同的重新检查。不得改变 G1 已验收测试预期。
- 必须重跑原 G2 的审核、优化、G1 geometry/profile 和完整 G2 benchmark。六个冻结非平凡端点、直线及冻结 STRAIGHT 均使用相同默认配置并通过；负例必须保留。留存命令、退出码、日志、完整审核预算、各初值/solver/审核记录、源码与证据 SHA-256、G1 和旧版保护差异。
- 报告须区分：有限值反例、独立积分余项推导与数值对照、软时限检查、尚未证明的浮点假设。不得把未经运行的测试报告为通过。
- 返工完成后停止并等待 Codex 独立验收和用户授权，不进入 G3。

## Codex 复验条件

1. 独立重现 4 m 终点偏差反例，验证 NaN 配置明确拒绝；检查全部审核字段和非有限中间值失败关闭。
2. 独立检查审核积分误差公式及与 G1 预算的合成，复算至少一个非平凡例；验证资源耗尽不放行。
3. 审查并复测软时限触发、状态和留痕；重核 G2 所有原定实验与 G1 基线。
4. 以真实源码/证据哈希和 Git 差异为依据，确认旧 FR06 与历史证据未修改；通过后才记录 G2 ACCEPTED。
