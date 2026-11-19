# 1 ns 时间轴篡改用例的容差边界说明

日期：2026-11-19。未 commit/push；本文件不宣称 Codex PASS 或 TASK 01 验收。
未修改任何生产代码、单元测试、阈值或 audit。

## 1. 场景与命令

在 `s1_failure_corridor_fixed` 会话仍运行时，以该会话最新 `HOLD_END`
plan 600002 的信封为基线运行了补充 tamper 负例（会话最后已停止）：

```bash
python3 tests/ros_virtual_a_smoke.py \
  --scene-manifest evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S1 --scenario tamper \
  --baseline-envelope work_flow/TASK01_evidence/07_ros_rviz/s1_failure_corridor_fixed/envelopes/envelope_00600002.json \
  --output work_flow/TASK01_evidence/07_ros_rviz/s1_failure_corridor_fixed/report_tamper.json
```

结果保存在该目录的 `report_tamper.json`（同时复制为
`report_tamper_1ns_time_boundary.json`）、`report_tamper.stdout`、
`report_tamper.stderr`。

## 2. 结果

7 个负例中 6 个被明确拒绝并发布 `CLEARED` 状态、0 个 `PLAYING`：

| 用例 | 结果 | 拒绝原因（播放节点日志） |
|---|---|---|
| `unknown_semantic_version` | 拒绝 | `unsupported playback semantic_version 'virtual_rigid_a.playback.v0'` |
| `unknown_model_version` | 拒绝 | `unsupported model_version 'virtual_rigid_a.timed.v0'` |
| `wrong_digest` | 拒绝 | `model_sha256 mismatch` |
| `tampered_position` | 拒绝 | 样本 x 差 0.001 > 1e-09 |
| `tampered_heading` | 拒绝 | 样本 theta 差 0.001 > 1e-09 |
| `tampered_speed` | 拒绝 | 样本 center speed 差 0.001 > 1e-09 |
| `tampered_time`（中点 +1 ns） | **未拒绝** | 播放节点接受了该信封 |

`tampered_time` 用例使驱动在 `note()` 处抛出并中止，因此
`tamper_results.json` 未写出，最后的 genuine replay 检查也未执行。
这不是 `playback_plan` 可执行计数或失败用例原子发布门槛的失败：
失败 plan 600001 的可执行信封计数仍为 0（见该目录 `report.json`
的 `failure_no_executable_envelope` 与 `publications.jsonl`）。

## 3. 原因：1 ns 恰好等于验证容差

`verify_virtual_a_playback_plan` 对每个字段使用 `1e-9` 容差
（位置/角度/速度），并且把传输时间轴作为 `state_at(time_s)` 的查询时间：

```python
times = trajectory.times_from_start
for index, time_s in enumerate(times):
    state = plan.state_at(time_s)
    _check_close(float(sample[1]), vehicle.x, ..., position_tolerance)
```

驱动只把中点样本的时间轴与所有车辆该采样点 `time_from_start` 一起加
1 ns。此时模型在 `t+1 ns` 与原始样本的差约为 `v * 1e-9`。本信封
sample 296 的 `center speed ≈ 0.792 m/s`，差约 `7.92e-10 m`，
小于 `1e-9`，其余字段（含该点的 `mode_control`）变化也低于容差，
因此验证通过。G5 的 S2 信封在同一驱动下通过，是因为其 sample 364
的 `mode_control` 变化率为 `1.821e-9 > 1e-9`——即该用例是否捕获
取决于基线信封该点的局部变化率，1 ns 扰动本身就是容差边界上的
非确定性负例。

时间轴本身的检查只要求：整数纳秒、严格递增、末端在
`T_end ± 2e-9` 内（本用例只改中点，不触碰末端），没有逐点与模型
规范时间轴比较；因此 1 ns 的偏移不会被时间轴元数据检查单独拒绝。

## 4. 原子门不被削弱；1 ms 扰动被拒绝

为区分“测试扰动太小”与“验证器不绑定时间轴”，用同一
`envelope_00600002.json` 做了一次独立诊断（驱动未修改，仅用其
`build_envelope_message`/`Session`）：把同一采样点偏移 **1 ms**，
保持单个 publisher 全程注册以避免短命 publisher 的连接竞态。

结果见 `../s1_failure_corridor_fixed/diagnostics/tamper_time_1ms.json`：

```text
state=CLEARED
reason=invalid playback plan: formation_poses[296].x=2.5024009870970683
  differs from the model 2.5016089081831678 by 0.000792079 > 1e-09
tampered_playing=[]          # 无 PLAYING/HOLD_START
genuine_restored=HOLD_END    # model_sha256=013a9a44...4109fdd 匹配
```

即：超过容差的时间轴篡改被拒绝并清空，随后原始信封可正常重放至
`HOLD_END`。1 ns 用例的接受是容差内（<1e-9 s 等效）的偏移，不是
对未认证信封的放行。

## 5. 边界与后续建议

- 本任务没有修改 `tests/ros_virtual_a_smoke.py` 的 1 ns fixture，也没有
  修改生产验证容差；G5 历史证据未触碰。
- 若评审希望 tamper 负例成为确定性门禁，建议另行评审将
  `tamper_time` 的扰动量从 1 ns 提升到 1 ms（仍远小于约 20 ms 的
  采样间隔）；该改动属于测试灵敏度修复，不应与生产容差混同。
- 本文件不把 1 ns 用例计为 TASK 01 的失败门槛；相关结论只用于说明
  证据边界，不宣称验收通过。