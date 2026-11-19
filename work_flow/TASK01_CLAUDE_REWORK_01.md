# TASK 01 返工 01：修正 v3 全负曲率角度误差界

**执行人：Claude Code**  
**范围：仅修复 TASK 01 v3 数值证书的曲率绝对值上界，并补齐针对性测试与证据。**  
**停止条件：本返工通过自测后停止，交 Codex 复核；不得自行开始 TASK 02。**

## 1. 基线与保护要求

- 开发分支：`feature/virtual-rigid-formation-a`。
- 已审核代码提交：`91dd71f619d968099622570f669a78f9865c7118`；当前工作区另有方案/证据提交。开始时记录真实 `HEAD` 和 `git status`。
- 用户已授权此前提交；本返工不要求重置或改写提交历史。
- 不覆盖 `work_flow/TASK01_evidence/00_baseline/`、既有 Step 5 数值证据或 ROS/RViz 证据；所有返工日志与新证据写入新目录：`work_flow/TASK01_evidence/09_rework_01/`。
- 不清理工作区现存未跟踪的 `latest` 证据链接，不执行 `git reset`、`git checkout --` 或其他会覆盖用户数据的操作。

## 2. 已确认的问题

文件：

`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py`

在 `_plan_integration_v3()` 中，当前代码用：

```python
max_kappa = max(max_kappa, piece_bounds["k"][1])
```

当某段曲率区间完全小于零时，`piece_bounds["k"][1]` 仍是负数；从 `max_kappa=0.0` 开始取最大值会把这段曲率当成零。后续角度误差界按 `abs(start_psi) + length * max_kappa` 计算，可能低估全负或负向占主导路径的界。

独立审核器 `formation_continuous_a/audit.py::_integration_plan()` 已使用区间端点绝对值计算曲率包络。不得因此删除或放宽审核器。

## 3. 实现要求

1. 在 `_plan_integration_v3()` 中，对每个 piece 使用曲率区间的最大绝对值更新全局 `max_kappa`：

   ```python
   kappa_envelope = max(abs(k_lower), abs(k_upper))
   max_kappa = max(max_kappa, kappa_envelope)
   ```

   `k_lower`、`k_upper` 应取自现有 `piece_bounds["k"]`。如已有 `kappa_envelope` 计算，可直接复用，避免重复实现。

2. 修复仅作用于 v3 数值计划。保留 `_plan_integration_v2()` 的计算与 v2 模型逐字节重放语义。

3. 不改变角度/位置容差、浮点 guard、积分策略、碰撞审核、优化约束、规划预算或失败策略；不添加新的 fallback、wrapper 或抽象层。

4. 保持现有 v3 数值模型字段结构。若修复使既有冻结 v3 输出不同，保留原证据不覆盖；在新证据中记录差异，并重新生成本返工后的 v3 证书。不要静默改写历史证据。若发现必须改变数值版本才能维持已声明的序列化合同，停止并向 Codex 报告理由，不自行扩大范围。

## 4. 必须新增的针对性测试

在现有 `test_virtual_a_geometry.py` 或最贴近的数值测试文件中增加简洁测试，覆盖：

1. **纯负曲率段**：构造有限、有效的全负曲率 profile/path；验证生产 v3 的 `angle_error_bound_rad` 等于按
   `bounds.angle_budget(segment_count, abs(start_psi) + S * max_abs_kappa)`
   得到的界，且大于错误的零曲率包络结果。
2. **负向占主导的多段路径**：至少一段为正曲率、另一段负曲率绝对值更大；验证全局包络取到较大的负曲率绝对值，而非只取最大的带符号上界。
3. **v2 兼容**：重建现有 v2 冻结模型仍逐字节一致；不得通过修改旧测试预期使其通过。
4. **端到端拒绝/审核边界**：确认使用正确包络后，角度界超过 `ANGLE_BUDGET_RAD` 的合成输入按既有方式 fail closed；不得因此放宽角度上限。

测试断言应来自公式和真实 profile 区间界，不要只检查源码字符串，也不要只复用生产函数计算期望值。可使用 `bounds.angle_budget()` 计算公式的最终舍入值，但测试需独立从构造的曲率输入推导 `max_abs_kappa`。

## 5. 验证命令

在仓库根目录执行并保存完整 stdout/stderr 与退出码：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_profile tests.test_virtual_a_integration_bounds tests.test_virtual_a_trajectory tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit
python3 work_flow/TASK01_evidence/05_numerics/verify_numerics_evidence.py
git diff --check
```

若现有 v3 冻结证书按原语义无法在修复后通过重建，应在新返工证据目录重新运行 Step 5 numerics probe，生成修复后的输出并由 verifier 核验；旧 Step 5 原始文件保持不变，并在报告中标记为修复前证据。不要运行会覆盖历史 `evaluation/formation_refactor_v02/` 数据的全量 unittest discover。

## 6. 允许修改范围

- 生产代码：仅 `formation_continuous_a/geometry.py` 的 v3 曲率包络计算。
- 测试：优先仅修改 `tests/test_virtual_a_geometry.py`；若必须新增文件，说明原因，且只新增上述数值边界测试。
- 证据：仅新增/写入 `work_flow/TASK01_evidence/09_rework_01/`。
- 不修改 `audit.py`、`bounds.py` 常量、v2 代码、序列化结构、优化器、pipeline、ROS 节点、RViz 配置及无关测试。

## 7. 交付与验收标准

返工结束时在 `09_rework_01/` 保存：

- `SUMMARY.md`：根因、修复点、v2 保持方式、命令/退出码、测试结果、证据是否因修复变化。
- `source.diff`、修改文件 SHA-256、`git_state.txt`。
- 测试日志；如重跑数值探针，保存独立的新输出及 verifier 结果。

通过条件：

- 全负及负向占主导测试通过，生产角度界覆盖由绝对曲率包络得到的参考界。
- v2 冻结模型仍逐字节重放。
- v3 profile/model 重建、独立 audit 与数值 verifier 通过；所有既有安全阈值保持不变。
- 修改仅在允许文件内，`git diff --check` 通过；未覆盖历史证据，未继续 TASK 02。

任一条件未满足、序列化身份需要变更或发现第二个数学合同问题时，停止并将具体证据交 Codex，不自行扩大返工范围。
