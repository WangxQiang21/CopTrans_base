# T2：旋转方向、带符号速度与曲率的独立微分合同（报告）

日期：2026-11-11。工作目录：`/home/wq/whut_thesis/expv3_multi`。
T2 起始 HEAD：`32d77da`（T1 提交）。本报告只覆盖 T2；T3–T7 仍未完成。

## 1. 结论

T2 完成标准全部满足：

- 正/负旋转（CCW/CW）均通过独立一阶微分验证，覆盖正向/反向、切向/反向切向、
  非零初始货物航向（跨 ±π）与非零枢轴；
- 故意篡改 `direction`、`signed speed` 或曲率均被拒绝；
- 同一独立合同已应用到 S5（`audit_mode_assignment`）、S8
  （`_context_for`/`_quantities`/`_sample_motion`）与 S9
  （`_check_block_kinematics`/`_dynamic_values`），审计不再直接信任构造器
  写入的 `direction`/`control`。

T2 定向测试 **64/64 通过**；全量 Python **392 tests**，仅保留 1 个 T0 已记录的
历史错误（`dynamic_peaks`，非 T2 缺陷）；C++ 规划器几何 **14/14 通过**。
真实 `run_demo.sh` 入口回归冒烟 **PASS**：33/33 检查、91.901 s，T1 的 CCW
旋转用例 E05 仍为 `FEASIBLE / OK`。

## 2. 改动清单

| 文件 | 作用 |
|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py` | S5：`_rotation_choices` 用 `sign(dpsi)` 决定切向方向标签、`kappa=theta'/signed_speed`；审计从块几何独立计算中心速度、方向符号、A 侧向残差、`theta'/v`、O 斜移角；平移审计也改为从块 `q_F` 位移独立求速度 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_retiming.py` | S8：`_context_for` 独立校验方向/曲率/O 角；`_quantities` 用 `dot(p_dot, u(theta))` 生成带符号速度；`_sample_motion` 保存带符号速度；A 旋转零投影速度 fail closed |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_quality.py` | S9：`_check_block_kinematics` 独立微分校验 direction/lateral/`theta'/v`/O 角；`_dynamic_values` 与样本比较改用实际车身航向的 dot/cross |
| `tests/test_formation_kinematics.py` | 新增 9 项：CW 切向反向、CW 反向切向正向、CW 未授权反向、篡改方向/曲率/行程、跨 π、非零枢轴、平移方向独立性 |
| `tests/test_formation_retiming.py` | 新增 CW 纯旋转：负的带符号中心速度、正的轮速幅值、曲率 `+1/1.2` |
| `tests/test_formation_quality.py` | 新增 4 项：S9 CW 可行、S9 篡改 direction/curvature/signed speed 被拒 |
| `tools/verify_rotation_contract.py` | 只读工具：7 个解析用例 + 3 个篡改用例，输出 `rotation_contract.json` |

未修改任何阈值（碰撞 0.02 m、航向 1°）、配置文件、launch、相邻仓库或用户文件。

## 3. 独立微分合同

对每个候选，中心导数由块几何直接计算，不读取构造器的 `direction`：

```
p_i(λ) = p_F(λ) + R(ψ_F(λ)) r_i
dp_i/dλ = dp_F/dλ + (dψ_F/dλ) R(ψ_F) J r_i
```

- A：`dp_i/dλ = v_i [cos θ_i, sin θ_i]`，`dθ_i/dλ = v_i κ_i`；
- O：`dp_i/dλ = v_i [cos(θ_i+φ_i), sin(θ_i+φ_i)]`，`dθ_i/dλ = 0`；
- S：`dp_i/dλ = 0`，航向可独立变化。

统一判据：

```
signed_speed = dot(dp_i/dλ, u(θ_i))
lateral      = cross(dp_i/dλ, u(θ_i))
direction    = sign(signed_speed)
A curvature  = θ_i' / signed_speed
O φ          = wrap(atan2(lateral, signed_speed) - π·[direction<0])
```

旧实现的问题（已由本阶段修复）：`_rotation_choices` 对切向车无条件写
`direction=+1`，CW 时实际中心速度投影为负，曲率用
`sign(dpsi)·direction/radius` 得到 `-1/r`，与 `theta'/v=+1/r` 矛盾；S8/S9
又把带符号速度写成 `direction*|p_dot|`，掩盖了该错误。

### 3.1 解析基线（`tools/verify_rotation_contract.py`）

货物原点、半径 1 m、`ψ_F=0`、切向航向 `(π/2, π, 0)`、`κ=+1 m⁻¹`
（默认 `r_min=0.810 m` 内）。结果（`rotation_contract.json`）：

| 用例 | dψ | 航向 | 允许倒车 | direction | signed speed | κ 存储/重算 |
|---|---:|---|---:|---:|---:|---:|
| `ccw_tangent_forward` | +0.5 | 切向 | 否 | +1 | +1.000000 | +1.000000 / +1.000000 |
| `cw_tangent_reverse` | −0.5 | 切向 | 是 | −1 | −1.000000 | +1.000000 / +1.000000 |
| `cw_opposite_forward` | −0.5 | 反向切向 | 否 | +1 | +1.000000 | −1.000000 / −1.000000 |
| `ccw_opposite_reverse` | +0.5 | 反向切向 | 是 | −1 | −1.000000 | −1.000000 / −1.000000 |
| `ccw_nonzero_pivot` | +0.4 | 切向 | 否 | +1 | +1.005/+0.900/+1.100 | 与 `1/半径` 一致 |
| `cw_nonzero_pivot_reverse` | −0.4 | 切向 | 是 | −1 | −1.005/−0.900/−1.100 | 与 `1/半径` 一致 |
| `ccw_cross_pi` | +0.6 | 切向，ψ₀=2.9 | 否 | +1 | +1.000000 | +1.000000 / +1.000000 |

最大残差：方向 0，侧向 `4.16e-16 m`，曲率 `4.44e-16 m⁻¹`。

### 3.2 篡改拒绝（S5）

| 篡改 | 审计结果 |
|---|---|
| CCW 切向段 `direction` 翻号 | `DIRECTION_SIGN_MISMATCH` |
| CCW 切向段 `control` 翻号 | `MODE_CURVATURE_MISMATCH` |
| CW 切向段改用固定正切向曲率 `−1/r` | `MODE_CURVATURE_MISMATCH` |

另：平移段 `direction` 翻号由块 `q_F` 位移独立求得正投影后拒绝
（`DIRECTION_SIGN_MISMATCH`）。

## 4. S8 / S9 应用

### 4.1 S8 重定时

`test_clockwise_rotation_reports_reverse_signed_speeds`：

- CW 纯旋转，切向车 `mode=0`（A），`direction=-1`；
- 所有非零采样 `center_speed_mps < 0`（实际中心速度沿车身航向为负）；
- `wheel_speed_mps >= 0`（轮速是幅值，符号语义分离）；
- 车辆中心 `y` 从 0 降到 `−0.575`，与 CW 几何一致；
- 段 `control=+1/1.2` 等于独立 `theta'/v`。

`_context_for` 对 `direction`、A 曲率、O φ 做独立校验；若 A 旋转的
`signed_speed` 为零则抛 `direction_sign`，不会被除零绕过。

### 4.2 S9 评估

`test_clockwise_rotation_is_feasible_with_reverse_speeds`：CW 旋转完整通过
S9 独立审核，采样带符号速度、曲率、轮速与解析链式法则一致。

篡改用例（为证明 S9 自身检查，测试中用 mock 绕过 S4 的
`audit_continuous_plan`，生产路径不会绕过）：

| 篡改 | 结果 | 字段 |
|---|---|---|
| 计划段 `direction` 翻号 | `KINEMATIC_INFEASIBLE` | `direction` |
| 计划段 `control` 翻号 | `KINEMATIC_INFEASIBLE` | `curvature` |
| 采样 `center_speed_mps` 减半（同号） | `FORMATION_RETIMING_INFEASIBLE` | `center_speed_mps` |
| 采样 `center_speed_mps` 翻号 | `KINEMATIC_INFEASIBLE` | `center_speed_mps` |

O 模式的存储量是沿车身航向的带符号分量；S9 用独立侧向分量重建全速度后再与
`|v|` 比较，避免把 `|v|cosφ` 误当作全速度。S9 的 `_dynamic_values` 与采样
比较均使用 `block.body_heading_at(...)`，不再使用编队航向或 `direction*|v|`。

## 5. 真实入口回归

沿用 T1 的 `tests/ros_cargo_smoke.py`（私有 master 11447、30×30 空地图、
`run_demo.sh --scale 1.0 auto_goal:=false rviz:=false enable_nlp:=false`）：

- 33/33 检查通过，耗时 91.901 s；
- `run_demo` / `roscore` 退出码 0，端口已释放；
- 权威 YAML sha256 仍为
  `21b9ff574bbfe2e190958a415ae0232a4d9df0146165960aa66f00f9e3f9ca92`；
- E05（货物 +90°、三车切向航向 `(π/2, π, 0)`）仍 `FEASIBLE / OK`，
  终端航向 `{1: π, 2: 3π/2, 3: π/2}`；
- 连续目标、手动重置、E06 预期拒绝与失败后恢复均保持 T1 行为。

本阶段没有新增 CW 的真实入口用例，CW 的端到端回放留给 T5/T7。

## 6. 可复现命令

```bash
cd /home/wq/whut_thesis/expv3_multi

# 解析微分证据（7 用例 + 3 篡改）
python3 tools/verify_rotation_contract.py \
  --out evaluation/wedge_fix_2026-11-11/T2/rotation_contract.json

# T2 定向测试（64）
OPENBLAS_NUM_THREADS=1 python3 -m unittest -v \
  tests.test_formation_kinematics tests.test_formation_retiming \
  tests.test_formation_quality

# 全量 Python（392；保留 1 个历史错误）
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'

# C++ 规划器几何（14）
catkin_ws/devel/lib/fwis_v3_planner/test_formation_geometry

# 真实入口回归冒烟
OPENBLAS_NUM_THREADS=1 python3 tests/ros_cargo_smoke.py \
  --output evaluation/wedge_fix_2026-11-11/T2/smoke.json \
  --log-dir evaluation/wedge_fix_2026-11-11/T2/runtime
```

## 7. 机器可读结果

```json
{
  "stage": "T2",
  "pass": true,
  "rotation_contract": {"cases": 7, "all_pass": true,
    "tamper": 3, "tamper_rejected": 3,
    "max_direction_error": 0.0,
    "max_lateral_speed_m": 4.163336342344337e-16,
    "max_curvature_error_1_per_m": 4.440892098500626e-16},
  "s9_evaluation": {
    "clockwise_rotation": "FEASIBLE",
    "tampered_direction": "KINEMATIC_INFEASIBLE/direction",
    "tampered_curvature": "KINEMATIC_INFEASIBLE/curvature",
    "tampered_signed_speed_magnitude": "FORMATION_RETIMING_INFEASIBLE/center_speed_mps",
    "tampered_signed_speed_sign": "KINEMATIC_INFEASIBLE/center_speed_mps"},
  "tests": {"targeted_t2": {"tests": 64, "failures": 0},
    "full_python_suite": {"tests": 392, "errors": 1,
      "preserved_historical_error": "tests/test_continuous_retiming.py:271 KeyError 'dynamic_peaks'"},
    "planner_cpp_tests": {"tests": 14, "failures": 0}},
  "real_entry_smoke": {"checks": 33, "failed": 0, "duration_s": 91.901,
    "run_demo_exit": 0, "roscore_exit": 0, "port_released": true}
}
```

完整内容见 `T2.json`、`rotation_contract.json`、`smoke.json` 与
`runtime/*.log`。

## 8. 失败原因

- 全量套件的唯一错误仍是 T0 已记录的
  `tests/test_continuous_retiming.py:271 KeyError: 'dynamic_peaks'`。
  按指示保留、未修复、未隐藏；因此全量命令退出码为 1，但 T2 定向测试与
  其余 391 项通过。
- 未放宽任何碰撞/航向/动力学阈值；未修改历史失败用例。
- 调试中修正的问题：
  - S9 旧的 `abs(center_speed_mps)` 与 `|v|` 比较在 O 模式把
    `|v|cosφ` 误当全速度，导致链式用例误报；改为用独立侧向分量重建
    全速度（`formation_quality.py`）。
  - S5 旧测试 `test_independent_audit_recomputes_centers_heading_and_control`
    期望篡改 O φ 报 `MODE_DISPLACEMENT_MISMATCH`；独立 φ 校验现在更早、
    更准确地报 `OBLIQUE_ANGLE_MISMATCH`，已更新该断言。

## 9. 已验证 / 未验证范围

已验证：

- CCW/CW、正向/反向、切向/反向切向、跨 ±π、非零枢轴的 S5 解析微分；
- S5/S8/S9 对 direction、signed speed、A 曲率、O 斜移角的独立校验；
- 篡改 direction/曲率/带符号速度在 S5 与 S9 均被拒绝；
- 真实入口的 T1 用例回归（E05 CCW、连续目标、重置、E06）。

未验证（留给后续阶段）：

- T3：平移/倒车/横移/斜移/零运动的完整任务级验收与动态上限；
- T4：货物静止时车体独立 Spin；
- T5：CW 轨迹在回放/RViz 中的插值、轮速与带符号速度语义；
- T6：连续目标事务、晚到消息、重置竞争；
- T7：真实地图 map04/map16、RViz 图形流程、×3.75 尺度重新验收；
- 用户原始失败尚未复现（T0 已声明）。

## 10. 运行清理

冒烟脚本只清理自己启动的 `run_demo.sh`、`roscore` 与进程组；未使用全局
`pkill`，未占用 11311/11312/11320。`cleanup.exit_codes.run_demo=0`、
`roscore=0`、`port_released=true`。