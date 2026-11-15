# FR02-A 返工报告（R1/R2）

- 任务依据：`docs/CLAUDE_TASK_02.md`、`evaluation/formation_refactor_v02/FR02/A/CODEX_ACCEPTANCE.md`
- 工作区：`/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`
- 范围：仅返工 FR02-A 的 R1（近终点进度稳定）与 R2（S 逐轮有符号滚动速度）
- 结论：R1/R2 已修复；原 A01–A10、FR01 与四个旧套件继续通过；Codex 探针原有的 29 个合法近终点异常和 4 个 S 轮速不一致全部消除。
- 未进入 FR02-B，未实现碰撞/S9、搜索或 ROS；未提交 Git，未访问原仓库 `/home/wq/whut_thesis/expv3_multi`。
- 本报告与全部新证据位于 `rework_01/`，原 `REPORT.md`、原结果、原 artifacts 与 Codex 验收目录均未覆盖。

## 1. 变更文件

仅修改：

| 文件 | 变更 |
|---|---|
| `P/uniform_trajectory.py` | 局部稳定五次时间求值；S 逐轮滚动速度改为有符号轮平面投影 |
| `tests/test_uniform_trajectory.py` | 增加 R1/R2 定向测试；原 A06–A10 用例、阈值和负例保留 |

`P/` = `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

未修改 FR01 `uniform_motion.py`/`test_uniform_motion.py`，未修改三个已通过局部适配的共享文件，未修改 `retiming.py`、`formation_quality.py`、消息、控制/动力学或 ROS 文件。

## 2. R1：合法近终点时间不再触发进度越界

### 2.1 原因与修复

原实现只在时间恰好等于相位端点时固定 `tau=0/1`；内部五次式在 `tau -> 1` 时发生消去误差，导致 `lambda` 略大于 `L`，被 FR01 严格进度域拒绝。

修复位于 `P/uniform_trajectory.py`：

- 新增 `_quintic_basic()`/`_stable_quintic()`；
- 后半段使用对称式 `s(tau)=1-s(1-tau)`，导数使用对应符号：
  - `s'(tau)=s'(1-tau)`
  - `s''(tau)=-s''(1-tau)`
  - `s'''(tau)=s'''(1-tau)`
- 端点显式返回 `(0/1, 0, 0, 60)`，保留原端点高阶导数；
- `_motion_state()` 的归一化 `tau` 仅在输入 `t` 已通过 `_phase_at()` 严格域检查后，对浮点除法本身产生的 `[-ulp, 1+ulp]` 舍入做归位；不对任意越界 `t` 做吸附，也未放宽 FR01 进度检查；
- `_setup_state()` 与运动相位共用同一稳定五次式，保持 setup 轮角曲线一致；
- 未修改只读的 `retiming.quintic()`。

### 2.2 定向测试

新增 `test_R1_stable_near_endpoint_progress_and_roundtrip`，覆盖：

- `straight`、`mixed_sign_spin`、`negative_spin`、手工 `S -> A 非零 pivot 弧 -> S -> O`；
- 每条 motion 的首点、末点、左右两侧；
- 末点前驱浮点数；
- 终点前相对间隔 `10^-5` 至 `10^-15`；
- 首点后相对间隔 `10^-5` 至 `10^-15`；
- 原模型的序列化往返模型，在同一时刻逐状态比较；
- 每个 setup 相位的近终点轮角；
- 每个 plan 最后一个 motion 近终点的 `sample()` 投影；
- 平面终点后一个 ULP 与非负时间域前一个 ULP 仍按 `TIME_OUT_OF_RANGE` 拒绝；
- 逐状态有限性、`lambda in [0,L]`、单调性、中心误差、刚性和独立弧闭式误差；
- 用 `Fraction` 精确计算 `(t-t0)/duration` 与五次式，独立核对 `progress` 和 `progress_rate`，而不是只核对“可求值且不越界”；
- 用 `Fraction` 独立核对 setup 近终点轮角 `start + delta*s(tau)`。

一个请求时刻因本身舍入到精确终点而被排除，明细保存在 `artifacts/R1_near_endpoint.json` 的 `skipped_boundary_requests`；这与 Codex 探针的筛选规则一致，不把“请求时刻舍入成终点”误报为内部失败。

另用 1e-3 的 `tau` clamp 做负向对照：该实现被新断言拒绝（`progress_rate` 与精确五次式不符），证明“以较大 ε 提前结束运动”不能再通过本测试；对照脚本与输出在 `checks/mutation_controls/`。

### 2.3 R1 数值结果

- 近端点合法时刻检查：`164` 个；
- 相位边界左右侧检查：`28` 个；
- 最大 `lambda-L` 越界：`0.0`；
- 最小 `lambda`：`0.0`；
- 最大单调下降：`0.0`（容差 `1e-12 * max(1,L)`）；
- 最大状态往返误差：`8.881784197001252e-16`；
- 最大 `progress` 对精确时间->五次式的绝对误差：`6.310887241768095e-30`；
- 最大 `progress_rate` 对精确五次式导数的绝对误差：`1.3390192960855274e-20`；
- 最大中心误差：`3.1554436208840472e-30 m`；
- 最大刚性误差：`4.440892098500626e-16 m`；
- 非零 pivot 弧独立闭式误差：`0.0`（在独立 `progress` 处求值）；
- setup 近终点轮角越界：`0.0 rad`；
- setup 轮角对独立精确时间映射的最大误差：`1.1102230246251565e-16 rad`；
- `sample()` 近终点状态误差：`0.0`。

## 3. R2：S 逐轮有符号滚动速度

### 3.1 原因与修复

FR01 的 S 轮因子是速度幅值；规范 S 轮角折入 `[-pi/2, pi/2]` 后，部分轮子必须反向滚动。原实现将全部 S 轮写成 `wheel_factor * yaw_rate`，导致输出字段 `wheel_rolling_speeds_mps` 与轮平面速度向量矛盾。

修复仅作用于 `_motion_state()` 的 S 分支：

```text
v_wheel = (-omega * y_w, omega * x_w)
u_w = v_wheel dot [cos(delta_w), sin(delta_w)]
     = (-omega*y_w)*cos(delta_w) + (omega*x_w)*sin(delta_w)
```

- 轮坐标顺序与 `retiming.wheel_configuration()` 完全一致；
- 保留 FR01 规范轮角与幅值因子语义；
- `wheel_speed_mps` 仍为 `max(abs(u_w))`，因此既有 S8 轮速峰值计算不变；
- A/O 分支未修改。

### 3.2 定向测试

新增 `test_R2_signed_spin_wheel_rolling_vectors`，覆盖：

- 混合正负 S 增量 `(pi/2, 0, -pi/4)`；
- 负向 S 增量 `(-pi/2, -pi/3, 0)`；
- 每个运动相位的 `0.05/0.25/0.5/0.75/0.95` 归一化时刻；
- 四轮逐轮独立向量检验；
- 折叠与未折叠轮；
- 零角车辆保持规范 S 轮角且全部滚动速度为 `0`；
- 参考角速度不取实现的状态字段，而由每个 segment 的 `theta_end-theta_start`、站距和精确五次式导数独立计算；
- 状态报告的 `yaw_rate_radps` 与独立参考角速度比较；
- 状态报告的 `wheel_angles` 与 `retiming.wheel_configuration()` 的规范折叠角逐轮比较；
- 每车必须恰有 4 个轮角和 4 个滚动速度，拒绝 `zip` 截断；
- 每个滚动速度与独立参考角速度、规范轮角的投影比较；
- `wheel_speed_mps` 与独立幅值 `max(canonical_factors)*abs(omega_ref)` 比较，并在半周期峰值时刻与既有 `motion.peaks[...]["wheel_speed"]` 比较；
- 序列化前后完整状态比较。

另做 4 个 R2 负向对照：滚动速度整体反号、角速度与滚动速度同时反号、未折叠轮角、只返回两个轮速；4 个变异均使测试失败，未变异基线通过。输出在 `checks/mutation_controls/mutation_controls.json`。

### 3.3 R2 数值结果

- 逐轮行数：`120`；
- 折叠轮样本/未折叠轮样本：`60/60`；
- 零角车辆样本：`10`；
- 最大轮平面向量残差：`5.551115123125783e-17 m/s`（阈值 `1e-7`）；
- 最大角速度参考误差：`3.469446951953614e-18 rad/s`；
- 最大规范轮角误差：`0.0 rad`；
- 最大轮速参考误差：`5.551115123125783e-17 m/s`；
- 最大轮速与既有峰值差：`8.326672684688674e-17 m/s`；
- 序列化前后状态最大误差：`0.0`。

## 4. Codex 探针原样复跑

`codex_probe/independent_probes.py` 为 Codex 原探针的逐字节副本，在新证据目录运行；原 `codex_acceptance_20261113T105416Z/independent_probes.py/json/log` 未覆盖。

| 项目 | Codex 原结果 | 返工后 |
|---|---:|---:|
| 正常内部/往返状态 | 66 | 66 全部通过 |
| 合法边界/近端点求值失败 | 29 | 0 |
| 边界求值总数 | 71 | 71 全部通过 |
| S 逐轮失败 | 4 | 0 |
| S 逐轮总数 | 12 | 12 全部通过 |
| 最大 S 轮向量误差 | `0.48845535005190793 m/s` | `5.551115123125783e-17 m/s` |

探针进程退出码：`0`。输出：`codex_probe/independent_probes.json`、`codex_probe/independent_probes.log`、`codex_probe/independent_probes.exit_code`。

## 5. 回归

全部在 `OPENBLAS_NUM_THREADS=1` 下、使用本 worktree 源码运行。

| 套件 | 结果 | 退出码 |
|---|---:|---:|
| `test_uniform_timing.py` | 5/5 OK | 0 |
| `test_uniform_trajectory.py` | 7/7 OK（A06–A10 + R1/R2） | 0 |
| FR01 `test_uniform_motion.py` | 13/13 OK | 0 |
| `test_formation_geometry.py` | 15/15 OK | 0 |
| `test_formation_kinematics.py` | 28/28 OK | 0 |
| `test_formation_retiming.py` | 14/14 OK | 0 |
| `test_formation_hold.py` | 14/14 OK | 0 |

合计 `96` 项通过。

当前被测文件哈希：

- `P/uniform_trajectory.py`：`d8c26261e64e1ab742aa9223b3c920316341984c6dc56755e6f69833edfaf014`
- `tests/test_uniform_trajectory.py`：`cc334d75c3ddae77feb564aefe5d5c683e6c844bc84aa9dd1b10b4a37ae01874`

未运行、未宣称修复旧 `test_continuous_retiming.py:271 dynamic_peaks`；该问题不属本次返工范围。

## 6. 证据索引

- `pre_rework/`：返工前源码/测试副本、绝对/相对哈希、HEAD/分支/Git 状态和 diff 统计；`pre_rework/REPORT.md`、`pre_rework/CODEX_ACCEPTANCE.md` 为原报告副本。
- `final/shared/`、`final/final_hashes.sha256`、`final/absolute_hashes.sha256`、`final/hashes_before_after.json`：返工后副本与哈希。
- `diff/`：两个修改文件相对 `pre_rework` 的完整 unified diff；`diff/*.exit_code` 记录退出码。
- `codex_probe/`：原探针逐字节副本、原探针结果副本、返工后探针 JSON/log/退出码。
- `logs/`：7 个正式套件完整输出与退出码。
- `results/`：`timing_results.json`、`trajectory_results.json`、`fr01_primitive_results.json`；A01–A10、R1/R2 全部 `PASSED`。
- `artifacts/R1_near_endpoint.json`、`artifacts/R2_spin_wheel_vectors.json`：R1/R2 机器证据，包含独立精确对照误差和逐样本记录。
- `artifacts/` 其余文件为 A06–A10 重新生成的原 artifacts，未覆盖原目录。
- `checks/self_check.json`：最终机械自检，`overall: PASSED`，无失败项；`checks/generate_evidence.py`：证据生成脚本。
- `checks/mutation_controls/`：R1 大 ε clamp 与 R2 四种状态的负向对照脚本、日志和 JSON；`all_controls_matched: true`。
- `command_manifest.json`、`environment.json`、`evidence_manifest.json`、`scope_verification.json`、`probe_comparison.json`、`regression_summary.json`。

## 7. 边界

- 本返工不宣称碰撞通过或 S9 独立审核通过；证书边界仍为 `GEOMETRY_ONLY_NO_S9_NO_COLLISION`。
- 独立审查另发现一项返工前已存在的解码容差信息：`from_model_dict()` 接受不超过 `TIME_CONTINUITY_TOL_S=1e-9` 的相位间隙，而 `state_at()` 在该间隙内可能抛出 `PHASE_TIME_DISCONTINUITY`。该代码路径本次未修改，不属 R1/R2 范围，未修复，也未据此宣称整体通过。
- 独立数值审查还覆盖了极端浮点 `duration`：当 `duration` 小到与 `t0` 相加后仍等于 `t0`（约小于半个 ULP）时，解码器可接受该相位，但 `_motion_state()` 先命中 `t0` 分支，导致 `state_at(plan.duration_s)` 返回进度 `0` 而非 `L`；当存储的 `duration` 极小（约 `<1e-102 s`）时，合法时刻的导数还会因 `duration**2`/`duration**3` 下溢触发 `ZeroDivisionError`，或因 `progress_rate**3` 溢出产生 `OverflowError`/非有限值。两者均只通过 `from_model_dict()` 的非物理存储时长触发；工厂调度时长（`>=0.3 s`）未复现，且返工前相同代码路径已存在。不属 R1/R2 范围，未修复；正常尺度 `duration` 的近端点行为由 R1 精确检查覆盖。
- 未进行 FR02-B、搜索、ROS/RViz、消息定义、控制/动力学工作。
- 未创建 worktree、未提交 Git、未操作原仓库。
- 完成后停止，交给 Codex 再验收。