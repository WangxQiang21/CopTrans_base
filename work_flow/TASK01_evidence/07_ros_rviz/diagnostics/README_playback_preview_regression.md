# 播放节点 transport preview 回归（Step 1 引入）诊断与最小修复

日期：2026-11-19。未 commit/push；不宣称 Codex PASS 或 TASK 01 验收。
不涉及任何 audit、阈值、optimizer 或走廊数学修改。

## 1. 症状（s1_easy_corridor，修复前）

显式小目标 `(5.0, 0.0, 0.0)` 在 corridor/hard 下三项 audit 均通过，
planner 成功构造、复验并发布了可执行原子信封：

```text
02:27:23,240 [uniform planner] virtual plan 600000 published:
  T_end=11.836 s model_sha256=4a3e180f... samples=593 wall=9.458 s
  verification_to_stamp_delay=0.8517 s
02:27:28,476 [uniform planner] plan 600000: playback did not confirm
  the atomic envelope within 5.0 s
```

`publications.jsonl`、`report.json`、`watch_publication.py` 记录：
`/formation/playback_plan` 该 plan 的可执行计数为 1（model_json_length
18286、sample_count 593），但驱动等待 `PLAYING` 超时；optimizer_status 为
`REJECTED / PLAYBACK_START_TIMEOUT / playback_confirmed=false`。

## 2. 根因

播放节点日志（`nodes_raw/*/formation_playback-3.log`）中**没有**
`accepted ... plan 600000`，也**没有** `rejecting playback plan 600000`。
即 `on_playback_plan` 在验证之前直接返回。其第一个检查是：

```python
if self._previewing:
    return
```

`_previewing` 由 `/formation/display_pose` 的 `on_preview` 置为 True。
goal bridge 在手动 reset（`on_initial_pose`）时 latching 发布该位姿
（日志 02:27:12,322），因此 reset 后 `_previewing=True`。

HEAD 版本中，随后的 `/formation/plan_request` 会走到
`self._previewing = False`（HEAD 脚本第 794 行）。Step 1 为虚拟后端新增了
“raw request 不是 admission”的提前返回：

```python
if self.backend == VIRTUAL_PLAYBACK_BACKEND:
    ...
    return
```

该分支位于 `_previewing = False` 之前，于是虚拟后端的原始请求不再结束
transport preview。之后 planner 发布的可执行信封被 `on_playback_plan`
无条件静默丢弃，planner 在 5 s 后判 `PLAYBACK_START_TIMEOUT` 并发布失败
占位，job_state FAILED，播放状态回到 `STATIC / FAILED: planning`。

这是当前工作树 Step 1 改动引入的回归，而不是 TASK 01 的 audit 或走廊
认证问题；`s1_main_free` 的 `CORRIDOR_AUDIT_WIDTH_LIMIT` 诊断不受影响。

## 3. 最小修复

`catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`：

1. 虚拟后端 `on_request` 提前返回前，恢复 HEAD 的 preview 语义：当
   `_job_state` 不是 `PLANNING/PLAYING`，或请求的 `plan_id` 正是已准入
   plan 时，置 `_previewing = False`；对另一个已准入 plan 的 busy 请求
   保持 preview（与 HEAD 的 busy guard 顺序一致）。不触碰 trajectory、
   hold、诊断层或 `_active_plan`。
2. `on_playback_plan` 在 preview 丢弃信封时增加 5 s 节流 warning，避免
   再次出现无日志的静默丢弃；行为本身不变。

没有修改任何 audit、阈值、走廊/optimizer 数学、消息定义或准入规则。

## 4. 单元测试

`tests/test_virtual_a_receiver.py::VirtualTransportPreviewTest` 新增 3 个
用例：IDLE 请求结束 preview、匹配已准入 plan 的请求结束 preview、busy
请求保持 preview。2026-11-19 运行：

```bash
PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src python3 \
    -m pytest tests/test_virtual_a_receiver.py -q
# 18 passed
```

## 5. 修复后现场验证

同参数新 session `s1_easy_corridor_fixed/`，仍为 corridor/hard、显式
`(5.0, 0.0, 0.0)`，不覆盖本目录的修复前证据。验收要求：三项 audit 通过、
可执行信封恰好 1 次、播放状态 `PLAYING` 且由 planner 确认、
`/formation/playback_plan` 可执行计数为 1。结果写入
`s1_easy_corridor_fixed/` 与 `summary.md`。

修复前证据保留在 `s1_easy_corridor/`，不删除、不改写。