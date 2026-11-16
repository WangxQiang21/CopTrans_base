# Claude 修复任务：G3 动态见证检查的非有限值处理

## 任务背景和边界

G3、G4 已分别通过 Codex 验收，但验收报告均记录了一项未修复的非阻塞问题：动态审核内部的实际指标检查可能跳过 `NaN` 或无穷大。本任务只修复该问题并验证相关路径，完成后停止等待 Codex 复核，**不得自动进入 G5**。

工作区：`/home/wq/whut_thesis/expv3_multi_virtual_a`  
分支：`feature/virtual-rigid-formation-a`  
基线 HEAD：`567bdecba9676313da3bc9a97b82f57b5b569a2b`

不得修改旧 FR06 源码、旧地图、ROS 配置或历史实验与验收证据；不得重置工作区、提交 Git、进入 G5。保留现有未提交改动。G3/G4 原验收记录绑定旧源码哈希，不修改或覆盖这些记录；本次交付新补丁和独立证据，待 Codex 在新源码上重新核验其适用性。

## 已确认问题

在 `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/audit.py` 的 `_audit_check_actual_metrics(actual, request)` 中，当前对每车每轮动态指标使用：

```python
if not math.isfinite(value) or value <= limit:
    continue
```

这会把 `NaN`、`+inf` 和 `-inf` 当作无需处理，函数可返回 `True`。Codex 已用一条包含 `wheel_speed=NaN` 或 `wheel_speed=+inf` 的实际指标记录直接复现。该函数主要用于区间审核无法立即确认时的实际点见证与细分路径；**尚未证明该问题曾使完整 `audit_dynamics` 把无效轨迹判为通过**。不要把它描述为已发生碰撞或已证明 G3/G4 实验结果错误。

## 实现要求

1. 对所有 `_DYNAMIC_METRICS` 涉及的三车四轮指标，遇到非有限值必须立即失败关闭，不能继续、替换为 0 或当成合法值。返回已有 `_AuditFailure` 机制支持的明确原因，例如 `NONFINITE_AUDIT_VALUE`，并包含车辆 ID、轮号、指标名和适当的约束位置；如果现有公共报告需要 JSON 安全，请避免把 NaN/Infinity 作为可序列化数值输出，可以用字符串描述。
2. 检查同一函数的 `wheel_angle_rad`，确保非有限值也被明确拒绝并有定位；有限值超过上限仍按原有 `DYNAMIC_LIMIT` / `WHEEL_ANGLE_LIMIT` 语义处理。正常有限值及原有“最不利超限车辆/轮”选择规则不得改变。
3. 审查 `_audit_actual_metrics` 生成标量的路径：若向量、轮速或转向动态在计算中产生非有限值，确保不能经该函数跳过。只做为此必要的局部修复；不要改物理限制、区间证书、积分预算、碰撞 margin 或规划算法。
4. 确认异常通过 `_audit_certificate`、`audit_dynamics` 和相关返回链转换为失败报告，不能抛出未处理异常或产生 `DYNAMIC_FEASIBLE`。保持区间无法证明时原有失败关闭行为。

## 允许修改

- `formation_continuous_a/audit.py`：上述非有限检查及必要的局部报告处理。
- `tests/test_virtual_a_audit.py`：新增该缺陷的正反例与调用链测试。
- 新建 `evaluation/virtual_formation_a/G3/<唯一run-id>/` 存放本次日志与证据。

如发现必须修改上述范围以外的文件，先记录具体原因和最小差异；若涉及改变核心数学合同、车辆限制、G4 碰撞语义或 G5 方案，停止并报告 Codex，不擅自扩展任务。

## 必须测试

1. **直接反例**：为每项动态指标分别构造 `NaN`、`+inf`、`-inf`，覆盖至少不同车辆与轮号；分别检查拒绝原因、车辆、轮和指标名。也覆盖非有限 `wheel_angle_rad`。直接覆盖函数的 `_DYNAMIC_METRICS` 全集合，不能只测 `wheel_speed`。
2. **有限值正反例**：所有有限值低于限值时通过；构造两项有限超限，仍报告原合同要求的最不利利用率；轮角有限超限仍按原原因拒绝。
3. **审核调用链**：在真实合法 C2 路径与 `VirtualATimedPlan` 上，使用受控注入使区间需要见证/细分时某实际指标变为非有限；通过 `audit_dynamics` 验证返回失败、有明确定位、不产生 `DYNAMIC_FEASIBLE`、不泄漏未处理异常。注明这是注入测试，不能声称实际运动已自然产生该值。
4. **现有能力回归**：运行 `test_virtual_a_audit.py`、`test_virtual_a_timing.py`、`test_virtual_a_trajectory.py` 和 `test_virtual_a_collision.py`。G4 核心验收不可仅靠旧哈希：至少用已冻结的无 reference 输入重新运行一条非平凡 G4 端到端场景，核对几何、动态、碰撞审核及 request/model/grid 绑定；可复用现有 pipeline 测试中的指定场景，不必为本项重跑六场景完整 benchmark。
5. 如果历史 G3 legacy timing 回归未重新运行，就注明未运行；它此前的 `timing_continuous` `KeyError: dynamic_peaks` 是已记录的旧基线失败，不得写为通过。若运行旧回归，使用安全 runner 的输出重定向，禁止覆盖旧证据。

参考测试命令：

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B -m unittest discover -s tests -p 'test_virtual_a_audit.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_timing.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_trajectory.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_collision.py' -v
```

单场景端到端的具体命令与输入路径写入实施报告，确保只读取 map/request，不读取存在性参考作为规划输入。所有测试日志写入新唯一 run 目录；记录实际命令、退出码和环境。

## 交付与验收

- 新证据目录包含 `CLAUDE_REPORT.md`、测试日志、非有限负例和真实场景回归结果、修改文件的 SHA-256、原 G3/G4 验收记录与历史证据保护检查、当前 Git 状态。
- 报告区分：已实际运行并通过、失败、未运行；保留完整原因和定位。不得自称 Codex ACCEPTED。
- 完成后停止。Codex 将独立复现直接反例、审查报告链并核对场景/哈希；确认通过后再等待用户决定是否进入 G5。
