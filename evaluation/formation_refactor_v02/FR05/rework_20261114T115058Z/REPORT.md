# FR05-R2 发布前空摘要守卫：返工报告

2026-11-14。工作区 `/home/wq/whut_thesis/expv3_multi_v02`（worktree）。
本轮只按 `codex_acceptance_20261114T113812Z/ACCEPTANCE.md` §3 的最小范围
补齐 FR05-R2 的发布前边界，**未提交 Git，未进入 FR06**。

## 1. 结论

- 唯一源码改动：`formation_uniform_planner_node.py:on_playback_status()`，
  在接受 `HOLD_START/PLAYING/HOLD_END` 前要求本 job 已有非空
  `_active_model_sha256`。
- 反馈身份单测由 6 项扩展到 10 项，最终源码新运行 `EXIT=0`，
  覆盖 ACCEPTANCE 要求的全部负例与两个正例。
- Codex 探针原脚本、原输入复制运行，五项输出全部符合合同；
  新增断言检查 `EXIT=0`。
- 未改模块（搜索、时间规划、回放算法、launch 默认值）不动；相应 ROS/GUI
  证据明确继承上一轮 `rework_20261114T095659Z`，本轮没有重跑，也不冒称
  为本轮新记录。

## 2. 源码改动

文件：`catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`
（`on_playback_status()`，1070–1117 行；守卫在 1087–1104 行）。

生命周期事实：`on_request()` 在 760 行把 `_active_model_sha256` 置 `None`；
`_finish_success()` 在 990 行、发布信封（999 行）之前才写入审核通过的
`verified_digest`。因此 PLANNING 阶段存在“已有请求、尚无模型摘要”的真实
窗口。原身份比较 `report.get("model_sha256") != self._active_model_sha256`
在该窗口把缺失摘要和 `None` 判为相等（`None == None`），会提前解除 busy。

新增守卫（在状态白名单之后、原身份比较之前）：

```python
if state in ("HOLD_START", "PLAYING", "HOLD_END"):
    if not self._active_model_sha256:
        rospy.logwarn_throttle(
            5.0,
            "[uniform planner] ignoring playback feedback for "
            "plan %s: no model has been published for this job "
            "yet", plan_id)
        return
```

设计取舍：

- 只拦截三个“确认类”状态，`CLEARED/INVALID` 的失败路径保持原行为，
  不改变失败场景的 job_state 迁移。
- 判据是“是否已有发布摘要”，不是 `_state == PLANNING`。信封发布后、
  首次回放确认前 job 仍可处于 PLANNING；合法零时长保持也可能直接反馈
  HOLD_END。这两种已发布且身份正确的情况继续成功（见 3.2 正例）。
- 守卫之后仍走原有 digest/formation_id/backend 比较，没有新增握手或状态
  系统。

## 3. 测试

### 3.1 运行结果

`python3 -m unittest -v tests.test_uniform_feedback_identity`，最终源码，
`EXIT=0`，`Ran 10 tests in 0.004s / OK`。完整输出：
`logs/units_feedback_identity.log`。

| # | 测试 | 场景 | 期望 |
|---|---|---|---|
| 1 | `test_matching_identity_terminal_releases_busy` | 已发布、摘要匹配 HOLD_END | HOLD_END，status 可执行 |
| 2 | `test_wrong_digest_terminal_does_not_release_busy` | 已发布、错误摘要 HOLD_END（保留） | 保持 PLAYING，无输出 |
| 3 | `test_missing_identity_terminal_does_not_release_busy` | 已发布、缺任一身份字段（保留） | 保持 PLAYING，无输出 |
| 4 | `test_wrong_formation_or_backend_terminal_is_ignored` | 已发布、错误编队/backend（保留） | 保持 PLAYING |
| 5 | `test_wrong_digest_hold_start_does_not_confirm_playback` | 已发布、错误摘要 HOLD_START（保留） | 不置 `_playback_seen` |
| 6 | `test_prepublication_missing_digest_terminal_is_ignored` | 发布前、缺摘要 HOLD_END | 保持 PLANNING，seen=False，无事件 |
| 7 | `test_prepublication_explicit_null_digest_terminal_is_ignored` | 发布前、显式 `model_sha256=null` HOLD_END | 同上 |
| 8 | `test_prepublication_missing_digest_start_does_not_confirm` | 发布前、缺摘要 HOLD_START | 保持 PLANNING，seen=False，无事件 |
| 9 | `test_published_planning_terminal_releases_busy` | 已发布、身份正确、PLANNING→HOLD_END | HOLD_END，executable/published |
| 10 | `test_published_zero_motion_terminal_releases_busy` | 已发布、状态词 NO_MOTION、PLANNING→HOLD_END | HOLD_END，status 词为 NO_MOTION |

负例断言“无事件”同时覆盖：不发布 `job_state`（不解除 busy）、不发布
`feasible/executable/published` 状态。正例使用与真实节点一致的非空
`_published_wall`，走真实回调而非复制逻辑。

测试文件：`tests/test_uniform_feedback_identity.py`（改动：增加
`PLANNING` 导入、`NULL` 显式空值标记、`_prepublication_node()` fixture；
`_base_status` 桩保留位置参数以断言 NO_MOTION；新增 5 项测试）。
既有 5 项测试语义未改。

### 3.2 Codex 阶段边界探针（原输入、原脚本）

脚本从 `codex_acceptance_20261114T113812Z/` 逐字节复制
（`cmp` 相同，哈希 `accbeaed…`），在复制目录运行以保护 Codex 原始输出。
输入五项与 Codex 完全一致。

| case | 发布前/后 | Codex 原始结果 | 本轮结果 |
|---|---|---|---|
| `original_wrong_digest` | 已发布 | PLAYING, seen=False, 0 事件 | 相同 |
| `published_matching_digest` | 已发布 | HOLD_END, seen=True, 2 事件 | 相同 |
| `prepublication_missing_digest` | 发布前 | **HOLD_END, seen=True, 2 事件（违规）** | **PLANNING, seen=False, 0 事件** |
| `prepublication_null_digest` | 发布前 | **HOLD_END, seen=True, 2 事件（违规）** | **PLANNING, seen=False, 0 事件** |
| `prepublication_missing_start_digest` | 发布前 | PLANNING, **seen=True（违规）**, 0 事件 | **PLANNING, seen=False, 0 事件** |

原始输出保留在 `codex_acceptance_20261114T113812Z/feedback_phase_boundary_probe.json`
（未被覆盖，哈希 `d80749…`）；本轮输出 `feedback_phase_boundary_probe.json`，
断言检查 `check_probe_expectations.py` → `PROBE_ASSERTIONS_OK`，
日志 `logs/probe_expectations_check.log`。

## 4. 哈希与清单

`SOURCE_MANIFEST.sha256` 共 175 项（上一轮 174 项逐项重新哈希 + 复制的探针
脚本），`sha256sum -c` 全部成功（`logs/manifest_check.log`）。

与上一轮清单相比仅两处变化：

| 文件 | 上一轮 | 本轮 |
|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py` | `3175e62d…` | `dc7c036d352aed89d42b88a66b181237454cb2b12f90113e1a12136ee31f200b` |
| `tests/test_uniform_feedback_identity.py` | `4bf36e7c…` | `64721ce8e7ae9c7ce0dcf52ebeb3ecdf80f897ce73145b56a0b20b47bda62bb8` |

新增证据文件：

| 文件 | SHA-256 |
|---|---|
| `feedback_phase_boundary_probe.py`（复制，与 Codex 原文件相同） | `accbeaed65cb1c6455e2bb040f56093a8bc29ce91af818fd81b712160d13ad5e` |
| `feedback_phase_boundary_probe.json`（本轮输出） | `7430778ac8267c869569789d62d02e907643fdf7fdf60391450eee483d38bab7` |
| `check_probe_expectations.py` | `4a8b88e249167e4a3811488622756a31bea668b52f4a698c8d02d679fbf880fe` |
| `logs/units_feedback_identity.log` | `7d9cbc043812b02f15d64e1f6dae50bb9320eab420c62714a6d15ab462eba0a9` |

## 5. 证据来源划分（继承 vs 本轮新运行）

**本轮由最终源码新运行：**

- 反馈身份套件 10 项（`logs/units_feedback_identity.log`）；
- Codex 阶段边界探针五项（`feedback_phase_boundary_probe.json`）及断言；
- 清单 175 项校验。

**继承上一轮 `rework_20261114T095659Z`，本轮未重跑，不作为本轮新记录：**

| 证据 | 位置（上一轮） | 结果 |
|---|---|---|
| main ROS 场景 | `ros/main_report.json` | ok=True，112/112 检查 |
| near-goal 30 s 负例 | `ros_near_goal/near_goal_report.json` | 诚实 `BUDGET_EXHAUSTED`，无有效信封 |
| failure 场景 | `ros_failure/` | 57/57 |
| tamper 场景 | `ros_tamper/` | 14/14 |
| 同 ID 不同摘要探针 | `ros_probes/` | `GEOMETRY_ONLY`，executable=false |
| RViz 可见回放（round 3） | `rviz/round1/` | plan 600003，`T_end=6.244301 s`，PLAYING/HOLD_END 可执行 |
| 相关回归 134 项 | `logs/units_related_full.log` | Ran 134 tests，OK |
| 上一轮 182 项整体记录 | 见该目录 REPORT.md | 来自对应提交/运行，不在本目录重复引用为新运行 |

上述未改路径不含 `on_playback_status()` 的确认分支，守卫不影响它们；因此
按 ACCEPTANCE §3 不重跑四套 ROS 场景、30 秒近目标与 GUI。

## 6. 范围与保护

- 修改文件仅两份允许文件，其余为新建本证据目录；所有旧证据目录
  （含 `codex_acceptance_20261114T113812Z/`）保持原样。
- 未改 FR01–FR04 实现、冻结计划/决策、旧测试期待、车辆尺寸/offset/r_min/
  轮角/轮速/安全阈值、搜索/时间规划/回放算法、launch 默认值。
- 未操作原仓库，未创建 worktree，未 `git add/commit`，未进入 FR06。
- 本轮未启动任何 ROS master 或节点。

完成后停止，交 Codex 复核。