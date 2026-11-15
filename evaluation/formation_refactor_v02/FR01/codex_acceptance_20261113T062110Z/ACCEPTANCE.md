# FR01 返工独立验收：通过

验收人：Codex。日期：2026-11-13。任务：FR01 / rework_01。结论：**ACCEPTED**。

本轮 R1–R3 返工已达到 FR01 阶段合同，无剩余阻塞项。允许将本记录作为后续 FR02 的前置验收依据；**本次没有授权或开始 FR02 实现**。本结论仅覆盖几何运动原语与本阶段校验，不代表时间、碰撞、搜索或 RViz 已实现或通过。

## 1. 验收对象与修改边界

- 工作区：`/home/wq/whut_thesis/expv3_multi_v02`。
- 分支：`refactor/formation-v02`。
- HEAD：`6227833011e2d92b711179bfdec979bb0b878b29`，与准备基线一致。
- `uniform_motion.py` sha256：`1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2`。
- `tests/test_uniform_motion.py` sha256：`7c25e85b14d8c11ecb460daf76eead60d82270dd9ad74b0d2531f490b98f360a`。

【代码事实】独立按 AST 比较返工前后：实现只改变 `UniformEdgeValidation` 指标载体和 `validate_uniform_edge()`；既有四类工厂、几何积分、进度参数和导数实现未改。测试仅增加 `test_n01_n07_independent_validator_negatives`，原有 P01–P12 与全部原阈值保持不变。

【实验事实】tracked/index 无差异；冻结输入与 provenance 哈希一致；Claude 返工目录的26份清单文件逐项核验一致；重建的返工前两个源码副本与上轮 Codex 记录的哈希完全一致。原仓库分支/HEAD 未变、T5 binary diff 与保护快照逐字节一致，原 tracked 文件、其余原始 untracked 内容及 T5 含忽略日志的历史记录均保持完整。

## 2. R1–R3 复核结论

### R1：A 圆弧实际车头和运动学关系——已关闭

【代码事实】校验在实际采样车头和中心导数上计算有符号进度速度、切向残差及曲率关系：

```text
v_lambda_i = sigma_i * ||p_i'||
p_i' = v_lambda_i * [cos(theta_i), sin(theta_i)]
theta_i' = kappa_i * v_lambda_i
```

同时保留枢轴几何的独立核对。上述量均为几何进度导数，没有引入真实时间。归一化模式阈值保持 `1e-6`。

【实验事实】N01 的一致车头偏移 `0.2 rad` 现被明确拒绝；正转、顺时针合法倒车及直线倒车正例均通过。合法圆弧切向残差约 `2.99e-16`，曲率关系残差约 `1.11e-16`。

### R2：统一模式结构及 S 首末规范轮系——已关闭

【代码事实】已核对全局 mode/primitive/geometry 配对与逐车 segment 的 ID、模式和方向；首末轮角与几何因子均按轮位置独立计算期望。

【实验事实】N02（零角 car2 仅末端轮系变成 A）、N05（全局 mode 与原语冲突）、N06（全局 S 内零角车标 A）均明确拒绝。正常零角 S 车首末均保持规范 S，四轮角绝对值约 `0.9357695914 rad`，因子约 `0.3068110168 m`；相对 A 的设置角需求没有丢失。真实设置时长仍留待 FR02。

### R3：限值与非有限拒绝——已关闭

【代码事实】审核所用航向、方向、轮系及运动参数增加结构/有限性前置检查；以本边限值检查逐车半径、曲率和首末轮角。没有依赖调用者读取超限指标后自行决定。

【实验事实】N03（实际轮角大于0.1 rad限值）、N04（半径1 m低于2 m限值）、N07（NaN轮角）均明确拒绝；错误分别为 `EDGE_VALIDATION_FAILED`、`EDGE_VALIDATION_FAILED`、`NONFINITE_VALUE`。

## 3. Codex 独立运行结果

以下均为 Codex 本轮实际运行，不以 Claude 的 PASS 声明替代：

| 验证 | 结果 | 退出码 |
|---|---|---|
| FR01 `test_uniform_motion.py` | 13/13 通过；原 P01–P12 与新增 N01–N07 | 0 |
| 原样复制上轮 Codex 独立探针 | 24条圆弧正例通过，7/7非法对象拒绝 | 0 |
| `test_formation_model.py` | 26/26 通过 | 0 |
| `test_formation_geometry.py` | 15/15 通过 | 0 |
| `test_formation_kinematics.py` | 28/28 通过 | 0 |
| `git diff --check` | 无输出；不据此宣称未跟踪新文件完成空白检查 | 0 |

24条固定种子圆弧包含正负转、逐车不同前后方向、非零世界位姿与多种非零枢轴。使用实际采样位置的中心数值差分：最大中心误差 `8.88e-16 m`，最大差分误差 `8.88e-10`，最大横向残差 `2.89e-10`，均满足 FR01 合同。

命令、环境覆盖、退出码见 [test_runs.json](test_runs.json)；完整输出保存为同目录各 `*.log`。独立复跑结果写入 [rerun_primitive_results.json](rerun_primitive_results.json)，没有覆盖 Claude 的原始结果。独立反例及指标见 [independent_probes.json](independent_probes.json)，探针源码与上一轮 sha256 一致。

## 4. 仍未验证的范围与后续准入

【尚未验证】FR02 的公共时间、全车停车、实际轮系设置驻留、真实速度/动态限值及连续轨迹审核仍未实现；完整扫掠碰撞、搜索和 RViz 回放也不在 FR01 验收内。不能把本次几何阶段通过解释为完整规划可用。

【实验事实】上轮已复现的旧 `test_continuous_retiming.py:271` 中 `KeyError: 'dynamic_peaks'` 本轮未修复、未重复运行；原测试/实现仍为 T4，不能将该旧时间套件标为通过，也不将其追加为 FR01 返工范围。

FR01 已具备可供下一阶段使用的统一模式几何边、规范首末轮系与阶段内数学检查。后续若进入 FR02，继续按既定合同处理权威边到时间序列、零角 S 兼容及公共设置时间，不需要重新选择 V0.2 路线。

## 5. 证据与历史保护

- [baseline_verification.json](baseline_verification.json)：文件/工作区/原 T5 保护、AST差异与提交哈希核验。
- [test_runs.json](test_runs.json)：本轮所有实际运行记录。
- [previous_acceptance/](previous_acceptance/)：2026-11-12 未通过报告和 JSON 的逐字节归档及哈希清单；上轮独立证据目录保持不变。
- `final_verification.json`：验收结束时源码、Claude 结果与报告未被 Codex 修改的核验。

本轮 Codex 只增加验收证据，并将 FR01 根目录的 `CODEX_ACCEPTANCE.md/json` 更新为当前结论；未修改实现或测试、未启动 ROS、未提交 Git、未继续 FR02。此前 CHANGES_REQUESTED 记录完整保留。
