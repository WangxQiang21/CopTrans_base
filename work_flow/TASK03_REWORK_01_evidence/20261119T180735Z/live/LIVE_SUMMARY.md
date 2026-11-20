# TASK03 返工 live ROS/RViz 证据

日期：2026-11-20
工具：`tools/run_rework_live.sh`（自 TASK03 Step 6 driver 复制，仅改证据输出根目录）
+ `tools/ros_step6_capture.py`（未改逻辑；`ROOT` 仍解析到工作树根）。
ROS master：任务私有 `11663`；`ROS_HOME`/`ROS_LOG_DIR` 均在各自 run 目录内；
从未接触 `11411`（另一会话）或其他 roslaunch 进程。

## 1. Union 正例：`union_map05_2625_rework`

UTC：01:59:06 → 02:00:01
命令：

```bash
./run_demo.sh --map map05 --scale 2.625 \
  virtual_a_formation_corridor_scope:=three_vehicle_union_v1
# 输入（与用户操作相同的话题）：reset 2,2,0；goal 8,2,0
```

`capture_summary.json`：`passed=true`，**29/29 checks ok**，`executable_envelope_count=1`，
状态序列 `IDLE → PLANNING → FEASIBLE → FEASIBLE`，HUD 终态
`job HOLD_END | playback HOLD_END | planning PUBLISHED | audit FEASIBLE |
AUDIT geometry=PASS dynamics=PASS collision=PASS`。

`quality_report.json`（`virtual_result`）：

| 项 | 值 |
|---|---|
| `status` / `executable` / `formation_corridor_scope` | `SUCCESS` / `true` / `three_vehicle_union_v1` |
| `formation_obb_constraint_count` | **3888** |
| `formation_corridor_audit.status` | `VERIFIED` |
| `template.schema_version` / `compiler_version` / `certificate_version` | `formation_corridor_template.v2` / `formation_corridor_rows.v2` / `formation_corridor_certificate.v1` |
| `template.segments`（sigma_start, sigma_end, f_region_index） | `[(0.0, 1.0, 0)]`（单 region chain，无迁移窗口） |
| `allocation` | 8 个区间全部 index 0 |
| `f_only_gate_migrated` | `false`（单 region，符合语义） |
| `certificate.status` / `max_outward_residual_m` / `unclosed_interval_count` | `VERIFIED` / `−0.37498999716394005` / `0` |
| `certificate.intervals_verified` / `intervals_checked` | `16` / `24`（其余区间以闭合子区间证明） |
| `certificate.certificate_sha256` | `a36ca37a4b374e95c7d351a5572dff317114ec79ca35ec47bbe54731401b6c0a` |
| `template_id`（= binding 中 template_id） | `382adc91b8176dda8e58e3c33db2ebb27560852cc9f76c23dc9ed7cb41ff89b2` |
| `chain_digest` | `e7f8568f2397a95f44a0dc2693a1fea4e8f6bd50635dd80ce0e32c56eee4b618` |
| request / grid / context | `e69d068b…7aeaf` / `64eef089…af321` / `9655b79f…80fc` |
| `attempts` | `[(0, SOLVER_SUCCESS_AUDITED, ipopt)]` |
| certificate path length / `path_sha256` | 6.0 m / `6775dda4…48ced`（route/guide length 为 6.079790116 m，二者不同） |

Receiver 重建（关键）：`capture_summary.checks` 中

- `envelope_carries_quality_binding`：`ok=true`，carried object length 11989；
- `envelope_verifier`：`ok=true`，`expected_formation=true`，712 samples；
- `union_binding_complete`：`complete=true`，template_id `382adc91…`；
- `envelope_digest` / `envelope_request_identity`：`ok=true`。

验证路径是生产代码
`formation_playback._verify_union_formation_binding`：它调用
`corridor_chain_from_payload(binding["chain"])`、
`formation_template_from_payload(binding["template"])`（v2，含每段
`f_region_index`）、`verify_formation_certificate(...)`，并重新执行
`audit_geometry` / `audit_dynamics`。因此该 live 结果直接证明：新 v2 模板 payload
能在 receiver/播放 verifier 中重建并通过证书验证，而不是仅靠单测。

执行层审计（`checks` 明细）：geometry 155 checks、dynamics 909 checks、collision
3740 checks，全部 `passed=true`。

截图：`images/live_01_00.8s.png` … `images/live_11_34.0s.png`、`02_terminal.png`、
`03_terminal_zoom.png`（`screenshots_ok=true`）。

## 2. 失败清屏负例：`negative_e04_map05_175_rework`

UTC：02:00:33 → 02:01:00
命令：

```bash
./run_demo.sh --map map05 --scale 1.75 \
  virtual_a_formation_corridor_scope:=three_vehicle_union_v1
# reset 1.51,2.5,-1.5707963267948966；goal 1.51,0.5,-1.5707963267948966
```

`capture_summary.json`：`passed=true`，**22/22 checks ok**，`executable_envelope_count=0`；
`negative_no_playback_plan`、`negative_no_stale_playback`（0 stale events）、
`negative_hud_not_certified`、`negative_candidates_not_certified`、
`negative_no_certificate` 全部 true。

`quality_report.json`：

| 项 | 值 |
|---|---|
| `status` / `executable` / `reason` | `NO_ACCEPTED_CANDIDATE` / `false` / `FORMATION_CORRIDOR_UNCONFIRMED` |
| `formation_corridor_audit` | `TEMPLATE_BUILD_FAILED`，`detail="assessment station sigma=0.5 map status='COLLISION_OBSERVED'"` |
| `formation_obb_constraint_count` / `formation_reference_constraint_count` | `null` / `null` |
| `checks` / `attempts` | `0` / `[]` |
| `trajectory_published` / `playback_confirmed` / `publication` | `false` / `false` / `null` |
| request / grid / context | `e5b6e715…165d` / `fcf003c3…5215` / `0d87586b…7673`（与返工前 live E04 完全一致） |

HUD 终态：`job FAILED | playback FAILED | planning FAILED | FORMATION_CORRIDOR_UNCONFIRMED |
map_collisions=3 (car2 at s=1.00/1.50/2.00, MAP_OBB_COLLISION) | audit AUDIT_PENDING`。

该负例只作为“失败清屏、无旧轨迹/无 envelope”证据；`formation_obb_constraint_count=null`
说明它在 NLP 之前停止，不能当作 OBB NLP 负例。OBB NLP 负例见
`../step_d_e04/E04_SPLIT.md` 结论 2。

截图：`images/live_01_00.8s.png` … `images/live_05_07.0s.png`（`screenshots_ok=true`）。

## 3. 进程隔离与清理

每次 run 由 driver 创建独立 `roscore -p 11663` 与 `setsid ./run_demo.sh` 会话，结束后
`kill -INT -- -<pgid>`，最后停止 master。`command.txt` 记录了 PID/PGID 与 UTC 起止。
运行前后 `ps` 仅见另一会话的 `roscore -p 11411`，未被触碰。