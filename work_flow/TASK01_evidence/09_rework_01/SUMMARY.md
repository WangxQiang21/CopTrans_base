# TASK 01 返工 01 自测摘要（交 Codex 复核）

日期：2026-11-19  
分支：`feature/virtual-rigid-formation-a`  
开始 HEAD：`222b919329bf487db1ee096e2902d753385a3527`  
状态：**Claude 自测完成，未提交、未推送；不代表 Codex 审核通过或 TASK 01 验收通过。**

## 1. 根因

`formation_continuous_a/geometry.py::_plan_integration_v3()` 原先用

```python
max_kappa = max(max_kappa, piece_bounds["k"][1])
```

更新角度误差界所需的全局曲率包络。`piece_bounds["k"][1]` 是区间**带符号上界**；当某段曲率区间完全为负时该上界仍为负数，而 `max_kappa` 从 `0.0` 起算，于是该段被当成零曲率，角度界被低估。全负路径与负向占主导路径均受影响。

## 2. 修复

仅改 v3 生产计划中的这两行（v3 原本已为 M4 计算 `kappa_envelope`，此处直接复用）：

```python
kappa_envelope = max(
    abs(piece_bounds["k"][0]), abs(piece_bounds["k"][1]))
max_kappa = max(max_kappa, kappa_envelope)
```

未改 `_plan_integration_v2()`：v2 仍使用 `piece_bounds["k"][1]` 与 `max(0.0, ...)` 的 M4 计算，冻结模型逐字节重放语义不变。未改 `audit.py`、`bounds.py` 常量、角度/位置预算、guard、积分策略、失败策略、优化器、pipeline、ROS 或 RViz。

## 3. 新增测试（仅 `tests/test_virtual_a_geometry.py`）

新增 `V3CurvatureEnvelopeTests` 四个测试，期望值由构造的曲率输入手算，生产函数只用于 `bounds.angle_budget()` 的最终舍入：

1. **纯负曲率**：`k=-0.05`、`S=2 m`、`psi0=0`。生产角度界 `9.000195399252335e-11` 等于 `angle_budget(1, 2*0.05)`，且大于错误的零包络结果 `9.000177635683941e-11`。
2. **负向占主导**：曲率从 `+0.1` 线性降到 `-0.3`，两段区间为 `[-0.1, +0.1]` 与 `[-0.3, -0.1]`。生产角度界 `1.0000284217094304e-10` 等于使用 `max|k|=0.3` 的公式值，而不是使用带符号上界 `0.1` 的错误值 `1.0000213162820728e-10`。
3. **v2 兼容**：`v2_model_600000.json` 经 `VirtualATimedPlan.from_model_json()` 重建后版本为 v2、`model_sha256()` 为 `df146b9bb4710132adeacd4d718d5a840c0f0e3bce353d06331a84142c2eaaf8`，且 `to_model_json()` 与读入字节完全一致。
4. **fail closed**：`k=-300000`、`S=2 m` 时按既有方式抛 `BOUND_UNCONFIRMED`，`angle_bound_rad=1.1558158799969943e-09 > ANGLE_BUDGET_RAD=1e-9`；未放宽上限。

**负向对照**：临时恢复修复前的 v3 两行并运行同一测试类，得到 `exit=1`、`FAILED (failures=2, errors=1)`（两个角度界断言失败；fail-closed 用例因路径被错误接受而 `KeyError: 'angle_bound_rad'`）。随后按 SHA-256 校验恢复修复文件，再运行得到 4/4 通过。证明测试确实覆盖该回归。

## 4. 验证命令与退出码

工作目录：仓库根目录。环境：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

| 命令 | 退出码 | 结果 |
|---|---|---|
| `python3 -m unittest tests.test_virtual_a_geometry tests.test_virtual_a_profile tests.test_virtual_a_integration_bounds tests.test_virtual_a_trajectory tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit` | 0 | `Ran 156 tests in 10.557s`, `OK` |
| `python3 work_flow/TASK01_evidence/05_numerics/verify_numerics_evidence.py` | 0 | 原 Step 5 证据仍通过：5 个文档、v2 重放字节一致、5 个 v3 证书在 1e-7 内 |
| `python3 work_flow/TASK01_evidence/09_rework_01/verify_numerics_evidence_rework.py` | 0 | 修复后新探针通过：5 个文档、v2 重放字节一致、5 个 v3 证书在 1e-7 内 |
| `python3 work_flow/TASK01_evidence/05_numerics/probe_numerics.py --map-yaml /tmp/expv3-task01-map.sax8HJ/map05-p12-parking-row-env2/map.yaml --output work_flow/TASK01_evidence/09_rework_01/numerics_probe_rework.json` | 0 | v2 重放一致；600003/600004 成功；两个合成直线场景仍为 `GEOMETRY_FEASIBLE` |
| `git diff --check` | 0 | 无输出 |

未运行会覆盖历史 `evaluation/formation_refactor_v02/` 数据的全量 `unittest discover`。

在仅显示改造提交 `bee4e3aa0f86e24085a1840ee0581b2fca34add8`（不包含
`geometry.py`）之后，于同一工作区复跑：六个模块
`Ran 156 tests in 10.294s`、退出码 0；两个 numerics verifier 均退出码 0。
日志为 `rework_tests_final_after_rviz_commit.log`、
`verify_numerics_evidence_final.log`、
`verify_numerics_evidence_rework_final.log`；两文件 SHA-256 仍为
`f31b20bb…` 与 `4f59e68f…`。

## 5. 数值证据变化

修复会改变真实曲线路径的 v3 证书与模型哈希，这是预期行为。**原 `05_numerics/` 文件未被覆盖**；修复前后的完整输出分别保存在旧目录与本目录，差异摘要见 `probe_delta.json`。

| 场景 | 字段 | 修复前 | 修复后 |
|---|---|---|---|
| 600003 | `angle_error_bound_rad` | `1.6000177635683942e-10` | `1.600121333890662e-10` |
| 600003 | `position_error_bound_m` | `5.124679957475981e-09` | `5.124997822032041e-09` |
| 600003 | model `sha256` | `e830730a…d875ce` | `b4f16583…7b2bda` |
| 600004 | `angle_error_bound_rad` | `1.600067585199066e-10` | `1.6002476702682593e-10` |
| 600004 | `position_error_bound_m` | `1.1170049229353544e-08` | `1.117128279893865e-08` |
| 600004 | model `sha256` | `924beced…c01540` | `959d5bb4…99e7e5` |

- 两个真实计划的 v3 角度界与位置界仍分别低于 `1e-9` 与 `1e-7`；`geometry_audit`、`dynamic_audit`、`collision_audit` 均 `passed=True`、`reason=""`。
- 两个合成直线场景（曲率为零）的证书字段不变，仅运行时间不同。
- v2 冻结模型重放仍逐字节一致，SHA-256 仍为 `df146b9bb4710132adeacd4d718d5a840c0f0e3bce353d06331a84142c2eaaf8`。
- v3 模型 JSON 的键集合、类型与数组长度未变；变化仅为证书数值字符串及其哈希/长度。未改序列化结构，也未改数值版本。

说明：`verify_numerics_evidence_rework.py` 是复制到本证据目录的新文件，仅改 `JSON_PATH` 指向 `numerics_probe_rework.json`，并把两个真实计划的漂移守卫常量更新为上述修复后数值（文件中注明旧值）。原 `05_numerics/verify_numerics_evidence.py` 未修改。

## 6. 修改文件与哈希

仅两个受跟踪文件被修改：

| 文件 | SHA-256 |
|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py` | `f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d` |
| `tests/test_virtual_a_geometry.py` | `4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7` |

`source.diff` 为这两文件的完整工作区 diff。临时负向对照前后的生产文件 SHA-256 相同，见 `geometry_before_negative_control.sha256`、`geometry_after_restore.sha256` 及 `sha256sum -c` 结果。

## 7. 证据文件

- `SUMMARY.md`（本文件）
- `source.diff`、`modified_files_sha256.txt`、`git_state_start.txt`、`git_state.txt`
- `rework_tests.log`（要求的 6 个测试模块，156 个测试）
- `rework_tests_new_class.log`（4 个新增测试，详细）
- `rework_tests_pre_fix_negative_control.log`、`rework_tests_post_restore.log`
- `geometry_before_negative_control.sha256`、`geometry_after_restore.sha256`
- `verify_numerics_evidence_after_fix.log`（原始 verifier 对旧证据）
- `numerics_probe_rework.json`、`numerics_probe_rework.stdout`、`numerics_probe_rework.stderr`
- `verify_numerics_evidence_rework.py`、`verify_numerics_evidence_rework.stdout`、`verify_numerics_evidence_rework.stderr`
- `probe_delta.json`（修复前后证书差异摘要）
- `git_diff_check.log`

## 8. 停止点

返工 01 自测到此停止，交 Codex 复核。未开始 TASK 02，未提交或推送本次修改。