# G3 动态见证检查非有限值修复

## 结论与范围

本报告处理 `CLAUDE_G3_NONFINITE_FIX_TASK.md` 指出的已确认缺陷：G3
`audit.py` 的 `_audit_check_actual_metrics(actual, request)` 对每车每轮动态
指标使用 `if not math.isfinite(value) or value <= limit: continue`，会把
`NaN`、`+inf`、`-inf` 当作无需处理，函数可以返回 `True`；非有限
`wheel_angle_rad` 也没有明确的原因和定位。

修复后：

- 所有 `_DYNAMIC_METRICS` 的三车四轮指标先经过
  `_audit_actual_finite`，非有限值立即以 `NONFINITE_AUDIT_VALUE` 失败，包含
  `vehicle_id`、`wheel_index`、`quantity`、`constraint`，不进入限值比较。
- 非有限 `wheel_angle_rad` 以同一原因拒绝，`wheel_index=None`；有限超限仍按
  原 `WHEEL_ANGLE_LIMIT` 拒绝。
- `_audit_actual_metrics` 在调用 `_static_wheel_angle` 前检查原始 `C_i`，
  防止 `atan2` 把非有限曲率变成有限轮角（`NaN -> NaN` 比较被跳过、
  `±inf -> 有限角度`）。
- 失败经 `_audit_certificate` -> `audit_dynamics` 的既有 `_AuditFailure`
  路径转换为失败报告，不产生 `DYNAMIC_FEASIBLE`，不泄漏未处理异常；详细
  描述把非有限值写成文本，位置字段不含 NaN/Infinity 数值，报告可用
  `json.dumps(..., allow_nan=False)` 序列化。

本任务只修改了允许的两个文件并新建本证据目录。未修改旧 FR06 源码、旧地图、
ROS 配置、G3/G4 验收记录或历史实验/证据；未 reset、未提交 Git、未进入 G5。
G3/G4 原验收记录仍绑定修复前哈希，本补丁不声称已被 Codex ACCEPTED，需在新
源码上复审。

## 缺陷复现与修复语义

修复前同一函数的两个分支是：

```python
value = record[name]
if not math.isfinite(value) or value <= limit:
    continue
...
if not math.isfinite(angle) or angle > angle_limit:
    ...
```

非有限动态指标被直接跳过；非有限角度虽然可能进入 `worst`，但原因仍是
`WHEEL_ANGLE_LIMIT`，且 `NaN` 会进入位置/详情。修复：

```python
def _audit_actual_finite(value, name, vehicle_id, wheel_index):
    try:
        finite = math.isfinite(value)
    except (TypeError, ValueError, OverflowError):
        finite = False
    if finite:
        return
    try:
        number = float(value)
        if math.isnan(number):
            description = "NaN"
        elif number == math.inf:
            description = "+Infinity"
        elif number == -math.inf:
            description = "-Infinity"
        else:
            description = "non-finite"
    except (TypeError, ValueError, OverflowError):
        description = "not a finite real ({})".format(type(value).__name__)
    raise _AuditFailure(
        "NONFINITE_AUDIT_VALUE",
        "vehicle_id={} wheel={} {} is non-finite ({})".format(
            vehicle_id, wheel_index, name, description),
        vehicle_id=vehicle_id, wheel_index=wheel_index,
        quantity=name, constraint="NONFINITE_AUDIT_VALUE")
```

`_audit_check_actual_metrics` 对每个 `(vehicle, metric, wheel)` 先调用该函数，
再执行原来的 `value <= limit`；角度先检查 `wheel_angle`，再执行原来的
`angle > angle_limit`。有限值的循环顺序、最不利利用率 `value/limit`（限值为
0 时 `math.inf`）和 `DYNAMIC_LIMIT` / `WHEEL_ANGLE_LIMIT` 原因均未改变。

`_audit_actual_metrics` 增加：

```python
_audit_actual_finite(vehicle["C_i"], "C_i", vehicle_id, None)
angle = _static_wheel_angle(
    float(vehicle["C_i"]), request.limits.wheelbase, request.limits.track)
```

这是因为 `atan2(inf, inf)` 返回有限的 `pi/4`，`atan2(nan, nan)` 之后的
`angle > maximum` 比较为假并返回 `0.0`，会在生成标量时把非有限输入藏掉。
其余向量、轮速、转向动态的非有限值经 `abs`/`hypot` 后仍是非有限，
`_audit_actual_finite` 会在限值比较前拒绝。

## 直接负例证据

`analysis/run_nonfinite_probe.py` 直接调用私有函数，使用冻结直线 request，
覆盖 `_DYNAMIC_METRICS` 全集合：

| 覆盖 | 结果 |
|---|---|
| 10 项指标 × {NaN, +Infinity, -Infinity} = 30 例 | 全部 `NONFINITE_AUDIT_VALUE` |
| 车辆覆盖 | 1、2、3 |
| 轮号覆盖 | 0、1、2、3 |
| 非有限 `wheel_angle_rad` | 3 例，`wheel_index=None`，明确拒绝 |
| 非有限 `C_i` 生成路径 | 3 例，在 `atan2` 前拒绝 |
| 原始向量/轮速/转向源非有限 | 10 例，标量生成后全部拒绝 |
| 有限值全低于限值 | `_audit_check_actual_metrics` 返回 `True` |
| 两项有限超限（较大在后） | 较早 speed 1.2 先违规，较晚 jerk 2.0 胜出，报告车辆 3/轮 2/jerk |
| 两项有限超限（较大在前） | 较早 speed 2.0 胜出，较晚 jerk 1.2 较小，报告车辆 1/轮 0/speed |
| 非有限优先 | 较早有限 speed 1.2 超限后遇到较晚 `jerk=NaN`，报告 `NONFINITE_AUDIT_VALUE` |
| 非有限轮角优先 | 较早有限 speed 1.2 超限后遇到本车/后车非有限轮角，仍报告 `NONFINITE_AUDIT_VALUE` |
| 有限轮角超限 | 原 `WHEEL_ANGLE_LIMIT`，车辆 2，`wheel_index=None` |

每个非有限例都断言原因、车辆、轮号、指标名、约束位置；位置字段严格 JSON
可序列化且不含 `value` 数值字段，非有限值只在 `detail` 中以文本描述。有限
超限例仍按原合同保留 `value`/`limit` 数值字段。探针输出 `status=PASS`。

## 测试

`tests/test_virtual_a_audit.py` 从 51 项增至 64 项，新增 13 项：

- 全部 10 项指标 × 3 种非有限值，跨车辆/轮号；
- 非有限轮角；
- 非有限 `C_i` 不能被 `atan2` 隐藏；
- 10 个原始向量/轮速/转向源非有限，经 `_audit_actual_metrics` 生成标量后仍被拒绝；该测试会在生成阶段把非有限值清洗成有限值的实现上失败；
- 原始轮速非有限时，标量生成后的 `wheel_speed` 仍被拒绝，不会因 `abs`/标量提取而被隐藏；
- 非有限值优先于较早出现的有限超限，不会退化为 `DYNAMIC_LIMIT`；同车和后车两种位置均覆盖；
- 非有限轮角不会被较早的有限动态超限抢先报告，同车和后车两种位置均覆盖；
- 有限全低于限值通过；
- 较早发生但利用率较小（speed 1.2）与较晚发生但利用率较大（jerk 2.0）两项有限超限，必须报告后者；按首次违规返回的实现会失败；
- 较早发生且利用率较大（speed 2.0）与较晚发生且利用率较小（jerk 1.2）两项有限超限，必须报告前者；按最后一次违规覆盖的实现会失败；
- 有限轮角超限保持原原因；
- 真实合法 C2 路径与真实 `VirtualATimedPlan` 上的两个受控注入调用点：
  `interval_subdivisions_per_segment=1, max_depth=20` 的粗区间先确认
  `leaves>3`（需要实际见证/细分），未注入时 `audit_dynamics` 通过。
  (1) 在 `_audit_actual_metrics` 返回的车辆 1/轮 0 `wheel_speed` 注入
  `NaN`，覆盖超限见证分支，`audit_dynamics` 返回失败
  `NONFINITE_AUDIT_VALUE`，定位 `vehicle_id=1, wheel_index=0,
  quantity=wheel_speed`。(2) 强制 `_audit_interval_leaf` 失败以进入区间
  包络见证分支，并在其真实 `_audit_dynamic_at_time` 生成结果中分别注入原始
  `v_i=NaN` 与 `C_i=NaN`；两种原始源都使实际指标生成/检查立即以
  `NONFINITE_AUDIT_VALUE` 失败，定位分别为 `vehicle_id=1, wheel_index=0,
  quantity=speed` 与 `vehicle_id=1, wheel_index=None, quantity=C_i`，证明新
  `C_i` 守卫异常也能转换为失败报告。三个案例均无 `DYNAMIC_FEASIBLE`、无
  未处理异常，`as_dict()` 严格 JSON 序列化成功，位置字段不含 `value`。

这些注入是受控测试输入，不声称真实运动自然产生了非有限值。

全部套件实际运行并通过：

| 套件 | 测试数 | 退出码 |
|---|---:|---:|
| `test_virtual_a_audit.py` | 64 | 0 |
| `test_virtual_a_timing.py` | 18 | 0 |
| `test_virtual_a_trajectory.py` | 23 | 0 |
| `test_virtual_a_collision.py` | 17 | 0 |
| `test_virtual_a_pipeline.py` | 8 | 0 |
| 合计 | 130 | 全部 0 |

日志均以 `OK` 结束；见 `logs/` 与 `logs/test_exit_codes.log`。

## 冻结 G4 场景端到端回归

使用已验收 G4 运行冻结的 S1 无 reference 输入
`evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S1/`
（仅 `map.pgm`、`map.yaml`、`request.json`）。脚本先读取验收 `scene_manifest`
和 G4 benchmark 结果仅用于期望哈希，再在 `Path.read_text` /
`Path.read_bytes` / `json.load` 均被 guard 禁止调用的条件下运行
`plan_virtual_a`。

实际结果：

```text
status = SUCCESS
executable = True
checks = 10664
geometry audit = passed
dynamics audit = passed
collision audit = passed
request_sha256 = d44f9efeb70f636a1e0f24dbc367e7869366ce344d0d88bba020d4ad355c8a1a
grid_sha256 = 054cf5c77079536f9a098e33232732d6c181f0f624f07e3a446ebfab0266a10e
planning_context_sha256 = 80f625e28f50901991661a97f90e55364c8f4bd8f3fe0d8e7eb7ea7e79106282
model_sha256 = 1e8f1a3dcd16b728fcc0c68380096d8365a8479226ac8f2af7b9e830fde9c589
```

三个 digest 与验收 G4 scene manifest 一致；plan 的 request/context/model
digest 与结果一致；collision grid digest 与结果一致；模型哈希与验收 G4 S1
benchmark 完全相同。输入包无 reference 文件，规划期间 guard 活跃。该结果只
证明这一个已知可行 A-only 场景，不声明成功率、完备性或实时性。

## 保护检查

`analysis/verify_protection.py` 结果 `PASS`：

- G3 验收记录 SHA-256 `adea82755b9fbaec7c589ff411d9b6a06b7cb23d7a89866c0edb4e5d882bff9d`；
  G4 验收记录 SHA-256 `413f5f4cc1965bb982be8771d0ea94c41e240a0ac5c934d20812fbeef638a17a`；
  两者仍为 `ACCEPTED`、HEAD 为基线、`next_stage_authorized=false`、绑定各自
  接受的 source manifest。
- 与 G4 验收 source manifest 的 73 个源文件比较，当前树只有两个授权文件
  改变：`formation_continuous_a/audit.py` 与
  `tests/test_virtual_a_audit.py`；其余 71 个文件哈希不变，无文件缺失。
- 四个带 source manifest 的历史运行（G3 `023528Z`、G3 `031558Z`、
  G4 `055008Z`、G4 `081458Z`）逐文件复核 473 个文件、106,501,508 字节，
  全部大小与 SHA-256 一致，`UNCHANGED`。无 manifest 的更早运行未逐个哈希
  核验，报告不声称它们被哈希绑定。
- Git：HEAD `567bdecba9676313da3bc9a97b82f57b5b569a2b`，分支
  `feature/virtual-rigid-formation-a`，暂存区 0 字节，无提交/push/新分支。

修改文件 SHA-256：

```text
audit.py (修复后)              54dda61ffc273e61be31eeeba3232dd896f00605c864a1b05056fc2de6620e5d
test_virtual_a_audit.py (修复后) 7b7baa9a780de5c166f3abb37f134efdf4a5e4de81a57adf75668a00a04b33a7
audit.py (G4 验收绑定)          7549420ea2319aa6d153592e7b6a9b205018352969891214a97111cb3875e604
test_virtual_a_audit.py (G3 验收绑定) 7708c3b51404d810d6c3c055ee5979cd5401def04a892021f97c1b0267eb582f
```

## 证据与复验入口

- [results.json](results.json)，SHA-256 `b436906decdf29a4b7707f43230447d35e1f5e20b621da2fdbbb756f2d734e4b`，
  状态 `PASS_PENDING_CODEX_RECHECK`、`codex_acceptance=NOT_RUN_NOT_CLAIMED`。
- [analysis/nonfinite_probe.json](analysis/nonfinite_probe.json)，SHA-256
  `8f33b8cfd8b7bf1133214f38da35017912b97fccf0037a51aee89ecdddeba10f`。
- [analysis/scene_regression.json](analysis/scene_regression.json)，SHA-256
  `327ec388e326b79a08c473863350e0e5e3ae88fae0830b7533e6c11f4ef9c1d2`。
- [analysis/protection_check.json](analysis/protection_check.json)，SHA-256
  `6ab43aa326111e7487039ade8dd4870e9e88e5d826dca1e418e2f30524b9bdf4`。
- [environment.json](environment.json)，SHA-256
  `22d7b911e88288f74061f55a1e58161347fbeb1bcf22ba8758d0fb61e0b868e1`。
- 命令与真实退出码：[COMMANDS.md](COMMANDS.md)、
  [logs/test_exit_codes.log](logs/test_exit_codes.log)、
  [logs/command_exit_codes.log](logs/command_exit_codes.log)。最后一轮命令
  从文档 preamble 开始重跑；命令日志由实际 `rc=$?` 写入，包含
  `environment`、五个测试套件、`nonfinite_probe`、`scene_regression`、
  `protection_check`、`assemble_evidence` 的退出码，全部为 0。
  `environment.json` 也在同一 preamble 下重新生成，记录
  `PYTHONDONTWRITEBYTECODE=1`、`OPENBLAS_NUM_THREADS=1` 及仓库与 ROS 的
  完整 `PYTHONPATH`，不再是最早的陈旧环境快照。
- [source_manifest.json](source_manifest.json) 绑定 2 个授权源文件、7 个任务/
  验收/历史源清单文件和本目录其余 26 个文件（逐个大小/SHA 记录在清单中；
  生成/验证日志与清单自身按设计排除以免自引用）。
  [logs/source_manifest_verification.log](logs/source_manifest_verification.log)
  显示 `status=VERIFIED`、HEAD/分支/暂存区与基线一致。清单校验是本地完整性
  证据，不是 Codex 验收。

## 未运行与已知问题

- 历史 legacy `timing_continuous` 回归**未在本任务重新运行**。此前记录的
  旧基线 `KeyError: 'dynamic_peaks'`（`tests/test_continuous_retiming.py:271`）
  仍未修复，也不写为通过。
- 未运行 G3/G4 完整 benchmark（本任务只要求单场景端到端）；已运行上表五个
  测试套件和一条冻结 S1 场景。
- 没有 ROS、Gazebo、硬件、地图修改、碰撞 margin 修改、规划器修改或 G5 工作。
- 非有限直接案例和调用链注入均为可控构造，不证明真实运动中曾自然出现非有限值，
  也不证明此前发生过错误放行、碰撞或 G3/G4 实验结果错误。

## 非声明

- 本报告不声称 Codex ACCEPTED；G3/G4 原验收记录绑定修复前源码，本次补丁的
  适用性需 Codex 在新源码上独立复核。
- 不声明一般成功率、完备性、最优性、实时性或形式化浮点证明。
- 不修改或覆盖任何旧验收记录、旧地图、旧实验和旧证据。

完成后停止，等待 Codex 复核；不自动进入 G5，不提交 Git。