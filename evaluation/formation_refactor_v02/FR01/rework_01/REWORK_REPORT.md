# FR01 返工报告 R1–R3

任务：FR01 rework_01。日期：2026-11-13。状态：**R1–R3 修补完成，定向测试与独立探针全部通过，等待 Codex 再验收**。未开始 FR02，未启动时间规划/搜索/ROS，未提交 Git。

本报告只回应 [CODEX_ACCEPTANCE.md](../CODEX_ACCEPTANCE.md) 的有限返工要求；不改变 V0.2 模型、A/O/S 原语族、进度参数合同或既有算法。

## 1. 结论摘要

- 修补 `validate_uniform_edge()`：A 圆弧按采样处**实际车头**检查无侧滑、前后方向与曲率关系；核对全局及逐车模式结构；独立核对首末两套规范轮系；以本边限值拒绝半径/曲率/轮角超限，并在比较前拒绝非有限值、非法 ID/枚举/方向与结构错误。
- `tests/test_uniform_motion.py` 新增 N01–N07 定向负例，并为每个负例保留合法正例；P01–P12 全部继续通过。
- 最终定向测试：**13/13 PASSED，退出码 0**（原 12 项 + N01–N07 测试方法）。
- 复制 Codex `independent_probes.py` 后原样运行：**24 条随机 A 圆弧正例全部通过，N01–N07 共 7/7 明确拒绝，退出码 0**。
- 三个旧几何/模型套件复跑：model 26/26、geometry 15/15、kinematics 28/28，均退出码 0。未运行、未修改 `test_continuous_retiming.py`，旧 `dynamic_peaks` 失败保持原状。

## 2. 工作区与边界

| 项目 | 记录 |
|---|---|
| 工作区 | `/home/wq/whut_thesis/expv3_multi_v02` |
| 分支 | `refactor/formation-v02` |
| HEAD | `6227833011e2d92b711179bfdec979bb0b878b29`（未变） |
| tracked/index | 无 tracked diff、无 staged diff；两个允许源码/测试文件仍为 untracked |
| `git diff --check` | 退出码 0，无输出（不覆盖未跟踪文件内容） |
| 修改范围 | 仅 `uniform_motion.py`、`tests/test_uniform_motion.py`，以及本 `rework_01/` 新证据 |
| 未修改 | 旧算法、旧测试、配置、冻结任务文档、Codex `codex_acceptance_20261112T081727Z/` 证据 |

未执行 `git add/commit/merge/push/stash/reset/clean`；未访问原仓库；未创建或清理 worktree；未运行 ROS/RViz/catkin 构建；未新增依赖；未扩大回归。

## 3. 文件与哈希

验收基线取自 `CODEX_ACCEPTANCE.md` 引用的两个哈希。为生成精确返工 diff，`baseline/` 中的副本由会话记录中的原始 Write/Edit 操作回放到本次返工前时点得到，并逐一核对哈希一致（无尾随换行，故 `wc -l` 少 1）。

| 文件 | 返工前 | 返工后 | 行数/字节（后） |
|---|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_motion.py` | `d75e469007172b7c0da9a706190effb8a3afc8d9fe20771391a7d528ed1726a5` | `1df4140684c155ea5dc5533c13278c8e4cf8e590b49e77b0360194695aa333b2` | 2422 / 107699 |
| `tests/test_uniform_motion.py` | `a30b740bbeedfc3d141696d93db23aa047f93cf0952c6af779bebf425938d4ba` | `7c25e85b14d8c11ecb460daf76eead60d82270dd9ad74b0d2531f490b98f360a` | 1730 / 75904 |

- 基线哈希复核：`baseline/uniform_motion.py` = `d75e4690…726a5`；`baseline/test_uniform_motion.py` = `a30b740b…4d4ba`。见 [hashes.json](hashes.json)。
- 返工 diff：[diff/uniform_motion.py.rework.diff](diff/uniform_motion.py.rework.diff)、[diff/test_uniform_motion.py.rework.diff](diff/test_uniform_motion.py.rework.diff)。
- 复制的 Codex 探针与本目录副本逐字节一致，sha256 `116e4c32…c64e6`；未修改探针内容。

## 4. 修补内容

### R1：A 圆弧实际车头、无侧滑、方向与曲率

`validate_uniform_edge()` 现在在每个采样进度处读取 `state_at()` 的**实际车头**和 `derivative_at()` 的中心导数，按合同方程计算：

```text
speed_i      = ||dp_i/dlambda||
v_lambda_i   = sigma_i * speed_i
r_tangent_i  = ||dp_i/dlambda - v_lambda_i [cos(theta_i), sin(theta_i)]||
               / max(speed_i, |v_lambda_i|, 1e-12)
r_curv_i     = |dtheta_i/dlambda - kappa_i v_lambda_i|
               / max(|dtheta_i/dlambda|, |kappa_i v_lambda_i|, 1e-12)
```

同时检查存储的 `signed_progress_speeds` 是否等于 `sigma_i * speed_i`。A 直线与 A 圆弧都执行该检查；A 圆弧保留并加强由枢轴公式独立重算的中心导数检查。新增审核指标 `max_a_tangent_residual`、`max_a_curvature_relation_residual`；阈值仍为 `1e-6`，没有放宽容差。

N01 注入后，独立的实际横向残差为 `0.19866933079506133 m/rad`（`sin(0.2)`），审核器明确拒绝，不再返回 `max_mode_equation_residual=0`。

### R2：全局/逐车模式结构与首末规范轮系

- 在任何数值误差比较前核对：`edge.mode`、`edge.primitive`、`_PRIMITIVE_MODE` 配对、`control.geometry_type`、`limits` 类型、`allow_reverse` 布尔性。
- 核对 `segments` 数量、车辆 ID 顺序、segment 模式（`A/O/S` 对应的 legacy 枚举）、方向、起止站与起止展开车头；全局 S 边内任何零角车辆的 segment 仍必须为 S。
- 对 `wheel_angles_start/factors_start` 与 `wheel_angles_end/factors_end` **两套边界**分别按轮位置独立重算规范轮角和几何因子并比较。零角 S 车在首末两端都必须保持规范 S，不能退化为 A 零曲率配置；新增指标 `max_wheel_angle_start_rad`、`max_wheel_angle_end_rad`。

N02（只改零角 car2 的末端 A 轮系）、N05（全局 mode 改为 S）、N06（只把零角 car2 的 segment 改为 A）现均明确拒绝。

### R3：有限性、ID/枚举/方向、结构与限值

- 所有被审核消费的字段先经过有限性与结构验证：`_finite`、`_validate_sorted_pair_field`、`_validate_direction_field`、`_validate_wheel_field`；控制、段、轮系、航向、曲率、斜移角、速度、限值及采样状态/导数值均不允许 NaN/Inf 绕过 `abs(actual-expected)>tol`。
- 以本边 `limits` 显式检查：
  - A 圆弧每车半径 `rho >= r_min_ack`（数值容差沿用既有 `EDGE_FIELD_TOL=1e-9`）；
  - 曲率 `|kappa| <= 1/r_min_ack`；
  - 首末轮角 `max|angle| <= steer_angle_max`；
  - 半径、曲率或轮角超限直接追加拒绝原因并抛出 `EDGE_VALIDATION_FAILED`，不只返回一个超限指标。
- 保留既有阈值：端点 qf/航向 `1e-7`、刚性 `1e-6`、归一化模式方程 `1e-6`、独立有限差分 `1e-4`。

N03、N04、N07 分别由轮角限值、半径/曲率限值和非有限前置检查拒绝。

## 5. N01–N07 结果

完整机器结果见 [primitive_results.json](primitive_results.json) 的 `N01-N07` case（含合法正例与每个注入说明）。

| 编号 | 注入 | 结果 | 明确原因/关键数值 |
|---|---|---|---|
| N01 | A 圆弧全部车头+segment 车头 `+0.2 rad`，路径/曲率/方向不变 | REJECTED | `EDGE_VALIDATION_FAILED`；独立横向残差 `0.19866933079506133`；审核归一化模式残差超 `1e-6` |
| N02 | 零角 S 车 car2 仅末端轮角/因子改为 A 零曲率 | REJECTED | `EDGE_VALIDATION_FAILED`；`end wheel_angles` 期望 `±0.9357695914 rad`、因子 `0.3068110168` |
| N03 | 合法 A 圆弧边限值改为 `steer_angle_max=0.1` | REJECTED | `EDGE_VALIDATION_FAILED`；实际最大轮角 `0.2932503134605156 rad` |
| N04 | 同一边限值改为 `r_min_ack=2.0` | REJECTED | `EDGE_VALIDATION_FAILED`；三车半径均 `1.0 m`，`|kappa|=1 > 0.5` |
| N05 | A_ARC 边全局 `mode='S'` | REJECTED | `MODE_PRIMITIVE_MISMATCH`；`mode=S` 与 `primitive=A_ARC` 不一致 |
| N06 | 全局 S 边仅零角 car2 `segment.mode=0(A)` | REJECTED | `EDGE_VALIDATION_FAILED`；segment mode 实际 0、期望 2（SPIN） |
| N07 | car1 一个起点轮角设为 NaN | REJECTED | `NONFINITE_VALUE`；在误差比较前拒绝 |

对应合法正例均在同一测试中先行通过：

| 正例 | 内容 | 关键指标 |
|---|---|---|
| L01 | A 圆弧正转，`direction=+1` | 切向残差 `2.99e-16`；曲率关系残差 `1.11e-16` |
| L02 | A 圆弧顺时针反转，`direction=-1`，`allow_reverse=True` | 切向残差 `2.48e-16`；曲率关系残差 `1.11e-16` |
| L03 | A 直线合法倒车，car2 `direction=-1` | 切向残差 `1.22e-16`；曲率关系残差 0 |
| L04 | 零角 S car2 首末规范轮系 | 首末角 `(-0.9357695914, +0.9357695914, +0.9357695914, -0.9357695914)`；因子均 `0.3068110167513546` |

## 6. 运行命令与结果

工作目录：`/home/wq/whut_thesis/expv3_multi_v02`。完整命令、环境、退出码、日志路径同时写入 [test_runs.json](test_runs.json)。

| 运行 | 命令 | 结果 |
|---|---|---|
| FR01 定向测试 | `OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 FR01_RESULTS_PATH=<rework_01>/primitive_results.json python3 -m unittest discover -s tests -p 'test_uniform_motion.py' -v` | `Ran 13 tests in 0.027s`，`OK`，退出码 0 |
| Codex 探针副本 | `OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 evaluation/formation_refactor_v02/FR01/rework_01/independent_probes.py` | 24 正例；7/7 拒绝；`unexpected_acceptances=0`；退出码 0 |
| 旧 model 套件 | `python3 -m unittest discover -s tests -p 'test_formation_model.py' -v` | `Ran 26 tests`，`OK`，退出码 0 |
| 旧 geometry 套件 | `python3 -m unittest discover -s tests -p 'test_formation_geometry.py' -v` | `Ran 15 tests`，`OK`，退出码 0 |
| 旧 kinematics 套件 | `python3 -m unittest discover -s tests -p 'test_formation_kinematics.py' -v` | `Ran 28 tests`，`OK`，退出码 0 |
| 空白检查 | `git diff --check` | 无输出，退出码 0 |

探针正例指标：最大中心误差 `8.88e-16 m`，最大数值差分误差 `8.88e-10`，最大横向残差 `2.89e-10`。探针 JSON 与日志：[independent_probes.json](independent_probes.json)、[logs/independent_probes.log](logs/independent_probes.log)。

## 7. 证据清单

| 文件 | 说明 |
|---|---|
| [REWORK_REPORT.md](REWORK_REPORT.md) | 本报告 |
| [primitive_results.json](primitive_results.json) | 最终 13 项测试机器结果；N01–N07 子例与合法正例均带指标 |
| [test_runs.json](test_runs.json) | 命令、环境、退出码、日志、结果摘要 |
| [hashes.json](hashes.json) | 基线/返工后源码哈希与探针副本一致性 |
| [baseline/](baseline/) | 返工前两个文件的精确副本，用于可复核 diff（哈希与验收文件一致） |
| [diff/](diff/) | 两个聚焦返工 diff（基线 → 返工后） |
| [independent_probes.py](independent_probes.py) | Codex 探针原样副本 |
| [independent_probes.json](independent_probes.json) | 探针最终机器结果 |
| [logs/](logs/) | 定向测试、探针、三个旧套件、`git diff --check`、哈希与开发中间失败摘录 |
| [rework_manifest.json](rework_manifest.json) | 本目录文件清单（生成于最后） |

## 8. 失败与开发中间状态

正式最终运行没有失败。开发 N01–N07 测试方法时出现过两次**测试脚手架**错误，已原样摘录于 [logs/development_failure_excerpts.log](logs/development_failure_excerpts.log)：

1. `KeyError: 'legal_positives'`：记录结构未初始化，补上 `case["legal_positives"] = []` 后修复。
2. `INVALID_PAIR_FIELD: wheel_angles_end must be an iterable of (vehicle_id, four values) pairs`：N02/N07 注入列表的条件表达式漏写非目标车的 `(vehicle_id, values)` 包装，修正后修复。

两次错误都不是算法/审核器行为失败；正式结果未删除、未改写。旧 `test_continuous_retiming.py` 的 `dynamic_peaks` 错误未运行、未修改、未修复，也不在本返工范围内。

## 9. 未覆盖与停止声明

- 未实现或修改时间参数化、真实速度/角速度、碰撞、设置驻留、搜索、ROS/RViz 或 FR02 转换。
- 未修改旧 `FormationBlock` 适配、`_sample_motion()` 的 `speed_signed` 语义或其他旧模块；FR02 适配点仍按原报告保留。
- 未提交 Git；未进入 FR02。

按返工边界：**完成后停止，等待 Codex 再验收**。