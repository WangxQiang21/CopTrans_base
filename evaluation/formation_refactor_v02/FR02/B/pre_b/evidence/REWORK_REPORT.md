# FR02-A 返工报告（R3 解码时轴有效性）

- 任务依据：`docs/CLAUDE_TASK_02.md`、`evaluation/formation_refactor_v02/FR02/A/CODEX_ACCEPTANCE.md`、`evaluation/formation_refactor_v02/FR02/A/codex_acceptance_20261113T115526Z/ACCEPTANCE.md`
- 工作区：`/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`
- 范围：仅返工 FR02-A 的 R3（解码时轴有效性）。不涉及 R4、FR02-B、动态可行性、碰撞/S9、搜索或 ROS。
- 结论：R3 已修复。Codex 的两个解码反例在修复前均可逐字节复现；修复后在 `from_model_dict()` 解码阶段以结构化原因拒绝，不延迟为求值异常。R1/R2 修复保持，97 项套件全部通过，Codex 独立探针 66/71/12 行仍全绿。
- 未修改 FR01 两个文件、S8/共享补丁、原数学阈值、历史证据或原仓库；未提交 Git；本报告与全部新证据位于 `rework_02/`。

## 1. 变更文件

仅修改以下两个文件：

| 文件 | 修复前 SHA-256 | 修复后 SHA-256 | 差异 |
|---|---|---|---|
| `P/uniform_trajectory.py` | `d8c26261e64e1ab742aa9223b3c920316341984c6dc56755e6f69833edfaf014` | `a93c2b2faba8bda7ac4592888cd502ea3850e6138505df851a12f44a5e86b579` | 36 行新增 / 4 行删除，4018 字节 |
| `tests/test_uniform_trajectory.py` | `cc334d75c3ddae77feb564aefe5d5c683e6c844bc84aa9dd1b10b4a37ae01874` | `713374e57559a1388d5940bf1fcc1aff6e255dd32644a6cede293272f08a03c2` | 344 行新增 / 2 行删除，18844 字节 |

`P/` = `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

未修改：

- FR01：`uniform_motion.py`、`test_uniform_motion.py`；
- 共享/时序：`formation_geometry.py`、`formation_kinematics.py`、`formation_retiming.py`、`uniform_timing.py`、`test_uniform_timing.py`；
- 原 `REPORT.md`、`CODEX_ACCEPTANCE.md/json`、原结果、原 artifacts、`rework_01/`；
- 冻结任务书、决策、阈值、车辆尺寸/offsets/r_min/轮角/轮速/安全阈值。

`diff/uniform_trajectory.py.diff`、`diff/test_uniform_trajectory.py.diff` 与 `diff/complete.diff` 保存了完整差异；`final/hashes_before_after.json` 与 `original_evidence_comparison.json` 保存逐文件前后哈希。R1/R2 的源码与测试未被回退或重写，只在其后追加 R3 检查。

## 2. R3 缺陷与修复前复现

### 2.1 容差内的真实缝隙

原 `_validate_plan_structure()` 对相邻相位使用：

```text
abs(phase.t0_s - cursor) <= TIME_CONTINUITY_TOL_S   # 1e-9
```

但 `_phase_at()` 按真实半开区间 `[t0, end_time)` 查找。因此，小于等于 `1e-9` 的正缝隙会通过解码，却在 `[0, T_end]` 内部存在无定义时间。

Codex 反例（`codex_decoder_probe/original/subtolerance_gap_input.json`）：仅将 motion 的 `t0` 后移 `5e-10 s`，同时缩短 `duration_s`，保持原 end 与总时长。

```text
前相位 end = 2.3432099798544908 s
后相位 t0  = 2.3432099803544910 s
实际缝隙   = 5.000000413701855e-10 s
缝隙内 t   = 2.3432099801044908 s
修复前：from_model_dict() ACCEPTED；state_at(t) -> PHASE_TIME_DISCONTINUITY
```

修复前探针退出码 `1`，两个用例均为 `FAILED`；其 JSON/log 的 SHA-256 为 `77843fc15348413409c7714211c655a4dd100e7fcd0f450645b52236608ebb73`，与 Codex 原证据逐字节一致。

### 2.2 浮点坍缩的正 duration

同一模型将 motion 的 `duration_s` 设为 `1e-20 s`。该值有限且为正，但在 `t0 = 2.3432099798544908 s` 的浮点尺度下：

```text
motion.t0 == motion.end_time == 2.3432099798544908 s
duration_s = 1e-20 s > 0
state_at(T_end).progress = 0.0
实际边 L = 1.5707963267948966 rad
```

修复前 `from_model_dict()` 接受该模型，终点返回起点；这不是求值舍入，而是存储器把正区间表达成零宽度。Codex 反例在 `codex_decoder_probe/original/unrepresentable_motion_interval_input.json` 中逐字节保存。

### 2.3 极端 duration 的数值支持域

R3 还要求明确在 A 阶段可稳定求值的时钟范围。A 求值器形成：

```text
progress_rate          = L * first / duration
progress_acceleration  = L * second / duration**2
progress_jerk          = L * third / duration**3
chain jerk             contains progress_rate**3
```

当 duration 远小于 `1e-100 s` 时，`duration**3` 进入次正规/下溢区，链式项无法稳定表示；当 duration 远大于 `1e100 s` 时，`duration**3` 及逆量接近/超过双精度有限范围。A 不判断物理动态可行性（那是 FR02-B），但必须在入口拒绝无法稳定表达的输入。

本返工显式定义数值支持域：

```text
MIN_STABLE_PHASE_DURATION_S = 1e-100
MAX_STABLE_PHASE_DURATION_S = 1e100
```

该支持域作用于存储的 motion `duration_s` 与 setup `steering_duration_s`。它不改变任何原数学阈值，也不把物理时长审核前移。

## 3. 修复内容

### 3.1 持久化时轴严格连续

`_validate_plan_structure()` 现在对版本化持久化时轴执行三条严格规则：

1. 首相位 `t0_s` 必须精确等于 `0.0`；
2. 相邻相位 `t0_s` 必须精确等于前一相位的 `end_time`（`difference != 0.0` 即拒绝）；
3. 最末 `end_time` 必须精确等于 `plan.duration_s`。

违反第 1/2 条时抛出：

```text
PHASE_TIME_DISCONTINUITY
```

违反第 3 条时抛出：

```text
PLAN_DURATION_MISMATCH
```

错误消息保留方向与差值（`gap`/`overlap`、实际秒数）及 `phase_id`。原 `TIME_CONTINUITY_TOL_S = 1e-9` 仅继续用于 setup 分解与采样网格，不再用于持久化相位边界；没有放宽或改窄原几何比较容差。

这是“严格拒绝”，不是重新定时：

- `_phase_at()` 未改，仍按真实半开区间查找；
- 解码器不移动任何 `t0_s` 或 `duration_s`，也不把缝隙归给前/后相位；
- 当前调度器本来就用上一相位 `end_time` 作为下一相位 `t0_s`，序列化写入同一 double，JSON 往返不引入新舍入；严格相等不会拒绝合法 S8 模型。

修复后同一 Codex 缝隙反例在 `from_model_dict()` 抛出 `PHASE_TIME_DISCONTINUITY`，`state_at()` 不再有机会看到无定义区间。

### 3.2 正时长可表示性

`_validate_plan_structure()` 对每个相位执行：

```text
end_time = phase.end_time
if not math.isfinite(end_time) or end_time <= phase.t0_s:
    -> PHASE_TIME_UNREPRESENTABLE
```

因此 `duration_s > 0` 但 `t0_s + duration_s == t0_s` 的坍缩区间在解码阶段被拒绝。`duration_s` 为 0 或负数时先由支持域检查拒绝（`PHASE_DURATION_UNSUPPORTED`）。错误保留 `phase_id`、`t0`、`duration` 与计算出的 `end`。

修复后 Codex 坍缩反例抛出 `PHASE_TIME_UNREPRESENTABLE`，不再返回 `progress=0` 的错误终端状态。

### 3.3 数值支持域入口检查

新增 `_stable_phase_duration()`：

```text
result = _finite(value, context)
if not MIN_STABLE_PHASE_DURATION_S <= result <= MAX_STABLE_PHASE_DURATION_S:
    -> PHASE_DURATION_UNSUPPORTED
```

调用点：

- `_validate_plan_structure()`：motion/setup 的 `duration_s`；
- `_validate_setup()`：`steering_duration_s`（setup 求值器的 `first/duration`、`second/duration**2` 时钟）。

边界值 `1e-100` 与 `1e100` 被接受并可稳定求值；`1e-101`、`1e101` 在解码入口被拒绝。该检查只针对不支持的数值尺度，不对合法物理时长作额外限制；合法零运动 hold（`duration_s=0.0`、无相位）仍按原合同接受。

## 4. 定向测试

新增 `test_R3_decoded_time_axis_is_contiguous_and_representable`，位于 `tests/test_uniform_trajectory.py:1558`。测试不缩小缝隙、不吸附时间、不把失败转为跳过；每个拒绝用例都断言具体结构化原因，并验证解码器未修改输入 JSON。

### 4.1 缝隙与重叠（8 例）

全部满足 `abs(actual_delta) <= 1e-9`，修复前均为容差内可接受、修复后均为 `PHASE_TIME_DISCONTINUITY`：

| 用例 | 实际 delta (s) | 相对旧容差 |
|---|---:|---|
| `codex_motion_gap_5e-10` | `+5.000000413701855e-10` | 内 |
| `motion_gap_one_ulp` | `+4.440892098500626e-16` | 内 |
| `motion_gap_just_below_tolerance` | `+9.999991945619513e-10` | 内 |
| `first_phase_gap_equal_tolerance` | `+1e-9` | 等于 |
| `motion_overlap_5e-10` | `-5.000000413701855e-10` | 内 |
| `motion_overlap_one_ulp` | `-4.440892098500626e-16` | 内 |
| `motion_overlap_just_below_tolerance` | `-9.999991945619513e-10` | 内 |
| `first_phase_overlap_equal_tolerance` | `-1e-9` | 等于 |

覆盖了 motion 前边界与首相位边界，正负两个方向，以及 ULP、`5e-10`、接近 `1e-9` 和恰等于 `1e-9` 四种尺度。

### 4.2 坍缩区间（1 例）

`codex_motion_collapsed_1e-20`：`t0=2.3432099798544908`、`duration=1e-20`、`end==t0`，断言 `PHASE_TIME_UNREPRESENTABLE`。

### 4.3 总时长与末端不一致（3 例）

| 用例 | 存储总时长与末端差 | 原因 |
|---|---:|---|
| `total_duration_short_one_ulp` | `-8.881784197001252e-16 s` | `PLAN_DURATION_MISMATCH` |
| `total_duration_short_5e-10` | `-5.000000413701855e-10 s` | `PLAN_DURATION_MISMATCH` |
| `total_duration_long_5e-10` | `+5.000000413701855e-10 s` | `PLAN_DURATION_MISMATCH` |

三个用例都只改 `duration_s`，相位边界不动；错误在解码阶段抛出，`phase_id` 为 `None`。

### 4.4 支持域边界与拒绝

- 接受：`1e-100 s`、`1e100 s`；在 `0, 0.25, 0.5, 0.75, 1` 归一化时刻全部状态字段有限，终端 `progress == L == 2.0 rad`；
- 拒绝：motion `1e-101`/`1e101`、setup steering `1e-101`/`1e101`，四项均为 `PHASE_DURATION_UNSUPPORTED`。

### 4.5 合法零运动 hold 与正常往返

- 合法零运动 hold：`duration=0.0`、0 相位、采样 `[0.0]`、`stationary`、全部速度和轮速为 0，序列化往返后保持；
- `straight`、`spin`、`complex` 三个 S8 生成模型：序列化往返后逐相位边界精确成链，首边界 `0.0`，末边界等于 `duration_s`，末运动终端 `progress` 等于对应边 `L`，终端 `qf` 与目标差在 `1e-12` 内，原/复原状态最大绝对误差 `0.0`。

`artifacts/R3_decoder_time_axis.json` 保存全部行与精确值，供独立复算。

## 5. 回归与独立探针

### 5.1 七个套件

| 套件 | 通过数量 | 退出码 |
|---|---:|---:|
| `test_uniform_timing` | 5 | 0 |
| `test_uniform_trajectory`（含 A06–A10、R1、R2、R3） | 8 | 0 |
| `test_uniform_motion`（FR01） | 13 | 0 |
| `test_formation_geometry` | 15 | 0 |
| `test_formation_kinematics` | 28 | 0 |
| `test_formation_retiming` | 14 | 0 |
| `test_formation_hold` | 14 | 0 |
| **合计** | **97** | 全部 0 |

A01–A05 与 A06–A10、R1、R2 结果分别保存在 `results/timing_results.json`、`results/trajectory_results.json`，全部 `PASSED`。97 = 上轮 96 项全部保留 + 1 项 R3 定向测试。

### 5.2 Codex 独立探针

原 `independent_probes.py`（SHA-256 `7b8224267c3c05872e030b2eb8fa34347d4a45a6ebfe462269bb88e38244c7ac`）逐字节复制，修复前/后均退出码 0：

- 66 个内部/往返状态：全部通过，往返正向计数 66；
- 71 个边界/近端状态：0 失败；
- 12 个显式 S 逐轮向量行：0 失败，最大向量误差 `5.551115123125783e-17 m/s`。

修复前后 JSON/log 哈希相同（`ddb3a886...`/`2c8c0eac...`），说明 R3 检查没有改变合法模型的求值结果。

### 5.3 Codex 解码探针

`decoder_time_probes.py`（SHA-256 `145dece772539464ac1f70bf7a85168788fbb8362feb6da31f9aeb40273b8f06`）逐字节复制：

| 阶段 | 退出码 | subtolerance_gap | unrepresentable_motion_interval |
|---|---:|---|---|
| 原证据 | 1 | `ACCEPTED` / `FAILED` | `ACCEPTED` / `FAILED`，`progress=0.0` |
| 修复前复跑 | 1 | 同上，逐字节一致 | 同上，逐字节一致 |
| 修复后 | 0 | `REJECTED` / `PASSED`，`PHASE_TIME_DISCONTINUITY` | `REJECTED` / `PASSED`，`PHASE_TIME_UNREPRESENTABLE` |

修复后两个用例均在解码阶段拒绝，未调用 `state_at()` 延迟报错。

## 6. 证据与自检

`checks/generate_evidence.py` 重新生成并核对本目录：

- `checks/self_check.json`：`PASSED`，41 项检查，0 失败；
- 检查覆盖：七套件退出码/数量、97 项合计、A01–A10/R1/R2/R3、FR01 13 项、R1/R2 数值断言、R3 8/1/3/2/4 行、合法 hold、三模型往返/终端、探针 66/71/12、解码探针 pre/post、原证据未覆盖、仅两文件变更、共享/FR01/文档未变、HEAD 未变、无 Git 提交；
- `scope_verification.json`：`tracked_modified_only_expected=true`，仅既有三个共享适配文件处于修改状态；本返工变更只有 `uniform_trajectory.py` 与 `test_uniform_trajectory.py`；未进入 FR02-B、未做 ROS/搜索、未修改原仓库、未提交 Git；
- `original_evidence_comparison.json`：17 个原证据/上轮返工文件全部未变；
- `command_manifest.json`：9 条运行命令及退出码；`environment.json`：Python 3.8、Linux x86_64、无 `math.nextafter`（测试使用 `struct` 浮点步进）；
- `evidence_manifest.json`：报告写入后以 `--manifest-only` 刷新，共 104 个文件（不含清单自身），含本报告。

证据目录：

| 路径 | 内容 |
|---|---|
| `pre_rework/` | 修复前源码/测试副本、绝对哈希、HEAD/分支/git 状态、上轮报告副本、17 个原证据哈希与复制说明 |
| `codex_decoder_probe/` | Codex 解码探针脚本、两个完整输入模型、original/pre_fix/post_fix 的 JSON/log/退出码 |
| `codex_probe/` | Codex 独立探针脚本及 original/pre_fix/post_fix 输出 |
| `logs/` | 七个套件日志与退出码 |
| `results/` | timing、trajectory、FR01 机器结果 |
| `artifacts/` | A01–A10 原 artifacts、R1、R2、R3 定向 artifacts |
| `diff/` | 两个文件及完整的 unified diff、命令与退出码 |
| `final/` | 最终源码/测试副本、最终/绝对/FR01 哈希、前后哈希、环境、git 状态/HEAD/分支 |
| `checks/` | 证据生成器、日志、41 项自检 |
| 顶层 | `command_manifest.json`、`environment.json`、`regression_summary.json`、`probe_comparison.json`、`decoder_probe_comparison.json`、`scope_verification.json`、`original_evidence_comparison.json`、`evidence_manifest.json` |

## 7. 边界与遗留

- 本返工只关闭 R3 的**结构可求值性**：时轴连续、区间可表示、总时长一致、时钟在 A 数值支持域内。
- 不判断速度/加速度/加加速度是否物理可行，不判断动态时长下限；这些仍属于 FR02-B。
- 数值支持域 `[1e-100, 1e100] s` 是 A 求值算术的支持边界，不是物理时长政策。合法 S8 模型的时长均远在域内，未受影响。
- 零运动 hold 的总时长 `0.0` 且无相位，不属于“正时长相位”，继续合法。
- 严格相等只适用于版本化持久化时轴。`TIME_CONTINUITY_TOL_S` 仍服务于 setup 事件分解与采样，不改原几何容差。
- 未开发完整新审核器，未进入 FR02-B、碰撞/S9、搜索、ROS/RViz、消息、控制或动力学；未提交 Git。
- 按验收要求，R3 完成后停止，交回 Codex 复验。

## 8. 复现

在 `/home/wq/whut_thesis/expv3_multi_v02`：

```bash
env OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 evaluation/formation_refactor_v02/FR02/A/rework_02/checks/generate_evidence.py
```

仅刷新报告后的文件清单：

```bash
python3 evaluation/formation_refactor_v02/FR02/A/rework_02/checks/generate_evidence.py --manifest-only
```

直接复跑两个解码反例：

```bash
env OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
  python3 evaluation/formation_refactor_v02/FR02/A/rework_02/codex_decoder_probe/post_fix/decoder_time_probes.py
```