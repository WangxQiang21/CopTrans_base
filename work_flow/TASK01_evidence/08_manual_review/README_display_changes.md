# 人工审核修改：路径类图层改为 Path 与 2D Pose Estimate 重置

日期：2026-11-19。对应人工审核建议的第 1、3 项：

1. 路径图层改成 `nav_msgs/Path`，以便在 RViz 中修改宽度和颜色；
2. 规划后点击 2D Pose Estimate，上一次的 marker 等仍留在地图上。

人工审核进一步明确：需要改成 Path 的不只是最终认证路径，还包括
`Planning candidates`、`A* raw route` 和 `Portal guide`。本文按该范围
记录实际代码修改、取舍、测试与证据；不宣称 Codex PASS 或 TASK 01 验收。

第 2 项（走廊外规划原因）不改代码，单独见
`README_corridor_containment_analysis.md`。

## 1. 路径类图层全部改为 `nav_msgs/Path`（第 1 项）

### 1.1 修改前

- 最终认证路径：`build_markers()` 在 MarkerArray 中为每车生成一条
  `LINE_STRIP`（`/formation/markers`）。
- Planning candidates：F 参考点路径和每车中心路径均为
  `LINE_STRIP`（`/formation/planning_candidates`）。
- A* raw route：橙色虚线由交替 `LINE_LIST` 段表示；Portal guide：
  `LINE_STRIP`（两者在 `/formation/guides`）。

Marker 的 `color`/`scale` 虽然存在，但 RViz Marker 显示没有对整条路径
生效的显示级颜色/线宽编辑入口；人工审核无法直接调整。

### 1.2 修改后

ROS-free builder
（`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py`）：

- `build_markers()` 删除最终路径的 LINE_STRIP，保留 OBB、朝向箭头、
  已行驶历史和文本标签；每车 4 个 marker，总计 17 个（12 车 + 1 links +
  3 reference + 1 status）。新增 `vehicle_path_samples(trajectory)`，直接
  从已验证信封的 `trajectory.points[vehicle_id]` 提取
  `(center_x, center_y, theta)`，不插值、不重采样。
- 新增 `build_guide_display(document)`，返回 `(markers, paths)`。
  `paths` 的键为 `("astar_raw", route_id)` 和
  `("portal_guide", route_id)`，值为原 A* 路线/guide 的逐点 `(x, y)`，
  不重采样。marker 层只保留 `A* ROUTE ...` 和 `PL_GUIDE ...` 文本标签；
  不再生成 A* 或 guide 的 LINE_LIST/LINE_STRIP。原
  `build_guide_markers()` 保留为只返回 marker 的兼容包装。
- 新增 `build_candidate_display(document)`，返回 `(markers, paths)`。
  `paths` 的键为 `"f_candidate"` 和
  `("candidate_vehicle", vehicle_id)`；marker 层只保留 F/中心文本标签和
  `CANDIDATE_OBB_NAMESPACE` 的 OBB 闭合轮廓。原
  `build_candidate_markers()` 保留为只返回 marker 的兼容包装。
- 删除仅用于 A* 虚线的 `_astar_dashed_segments()`、`ASTAR_DASH_ON_M`、
  `ASTAR_DASH_GAP_M` 和两个线宽常量；A* 路线仍逐点复制，只是显示方式
  改由 Path 话题承担。

ROS 节点
（`catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`）：

- 最终路径继续使用 `/formation/path/vehicle_{id}`。
- 新增动态 latched Path 话题：
  - `/formation/path/f_candidate`
  - `/formation/path/candidate_vehicle_{id}`
  - `/formation/path/astar_raw_{route_id}`
  - `/formation/path/portal_guide_{route_id}`
- A* 和 guide 可能有多条 route，不能共享一个 `nav_msgs/Path`，因此按
  `route_id` 分话题；candidate 中心按 `vehicle_id` 分话题。
- `_publish_guide_paths()` / `_publish_candidate_paths()` 在每次新 payload
  到达时先清空该层所有已创建的 latched Path 话题，再发布当前 payload
  中的路径。这样 route 数变少时，不会留下上一轮 route 的旧几何。
- 诊断 Path 的 `PoseStamped` 使用 identity orientation（`w=1`），因为
  候选中心/A*/guide 是参考折线，不声明车辆姿态；最终认证路径仍保留
  逐样本 `theta`。
- `_clear_guides()` / `_clear_candidates()` 同时清空各自 Path 注册表；
  `_clear_diagnostics()` 仍覆盖 corridor、guide、candidate 和最终路径。

RViz
（`catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz`）：

- 新增 10 个 `rviz/Path` 显示：F candidate、3 个 candidate centre、3 个
  A* raw route、3 个 Portal guide；连同原 3 个最终路径显示，共 13 个。
  默认颜色/线宽：F 黄 `255;217;13`/0.045，candidate 蓝
  `26;115;255`/0.03，A* 橙 `255;140;0`/0.026，guide 绿
  `13;209;64`/0.032。用户可在 Displays 面板逐话题修改 `Color` 与
  `Line Width`。
- `/formation/guides` 图层更名为 `Guide labels (markers)`；
  `/formation/planning_candidates` 更名为
  `Candidate OBBs and labels (markers)`。两图层仍启用，但只承载文本标签
  和 candidate OBB，不再承载路径线。
- `/formation/markers` 图层继续承载最终路径车辆的 OBB、朝向、历史和
  标签。

### 1.3 取舍

- `nav_msgs/Path` 的显示属性是**每话题一个**颜色/线宽：
  - A* 原来的橙色虚线样式不再保留，变成实线；但路线点列不变，且用户
    可以自行调色/调宽。若仍需虚线，可用 RViz 的 `Line Style` 或后续
    自定义显示实现，不在本次最小修改范围内。
  - candidate 路径不再按 `A/O/S` 模式在 marker 中分段着色；其
    `UNVERIFIED_CANDIDATE` / `DIAGNOSTIC_ONLY` 身份仍由同一话题上的
    文本标签表达，OBB 也继续用 candidate 颜色绘制。
- Candidate OBB 是闭合车体轮廓，不是一条路径，仍保留为 Marker；文本
  标签不是几何，无法用 `nav_msgs/Path` 表达。
- 多个 A*/guide route 分别占用独立 Path 话题；当前 RViz 资产预置
  route 0/1/2。若 planner 未来产生更多 route，需要相应增加显示项，节点
  仍会按 `route_id` 自动创建并发布话题。

## 2. 2D Pose Estimate 清除旧图层（第 3 项）

### 2.1 原因

`on_preview` 收到 `/initialpose`（RViz 2D Pose Estimate）时，原实现只
更新 hold、清空 trajectory 和 marker 待清除标志，没有清 corridor、
guide、candidate、诊断 Path 和最终 Path 话题。上一次 `REJECTED` 计划的
诊断候选/走廊/引导，以及上一次成功计划的 latched Path，会一直保留，
直到下一次 job admission 或 CLEARED planning payload 才调用
`_clear_diagnostics()`。

### 2.2 修改

`on_preview` 现在在设置新 hold 后立即调用 `_clear_diagnostics()`：

```python
self.hold = (contract.formation_id, (p.x, p.y, yaw), vehicles)
self.trajectory = None
self._clear_markers_pending = False
self._previewing = True
self._active_plan = None
self._hold_reason = "START"
self._clear_diagnostics()   # corridor + guide/candidate markers and Paths
self._publish_hold(clear=True)
```

`_clear_diagnostics()` = `_clear_corridors()` + `_clear_guides()` +
`_clear_candidates()` + `_clear_paths()`；每层向 latched 话题发布
DELETEALL 或空 `Path` 消息。`_clear_guides()` 与 `_clear_candidates()`
现在还会清空各自动态创建的 A*/guide/candidate Path 话题。这样手动
2D Pose Estimate 会立即移除上一次实验的全部显示层，新规划从干净画面
开始。

`on_request` 的虚拟分支**没有**增加清层：原始请求不是准入，按既有
设计不得清除当前 replay/hold/diagnostic 图层；它只在非 busy 或
plan 匹配时结束 preview。这一边界在
`07_ros_rviz/diagnostics/README_playback_preview_regression.md` 和
Step 1 的 preview 修复中已经说明，本次未改变。

## 3. 测试与构建

修改文件：

```text
catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py
catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py
catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz
tests/test_formation_playback.py
tests/test_virtual_a_playback.py
tests/test_virtual_a_receiver.py
tests/test_virtual_a_corridor_display.py
tests/test_formation_demo_assets.py
tests/ros_formation_smoke.py
```

定向测试：

```text
python3 -m unittest tests.test_formation_playback \
  tests.test_virtual_a_playback tests.test_virtual_a_receiver \
  tests.test_virtual_a_corridor_display tests.test_formation_demo_assets
Ran 155 tests in 81.384s
OK
```

日志：`logs/focused_after_review.log`。

新增/覆盖点：

- `test_guide_paths_copy_route_and_centreline_without_resampling`：A* 和
  Portal guide 的 Path 点列与真实 route/guide 逐点一致；marker 层不再有
  LINE_LIST/LINE_STRIP。
- `test_unverified_candidate_uses_stable_namespaces_and_labels`：F 和
  3 条 candidate 中心 Path 与 payload 逐点一致；文本标签/ OBB 仍在
  marker 中，路径线不再由 marker 承载。
- `test_candidate_payload_publishes_paths_and_keeps_obb_markers`：
  `on_planning_debug` 收到 candidate payload 后发布 F + 3 条中心 Path，
  marker 层保留 OBB 且不再有 candidate 路径线。
- `test_diagnostic_paths_clear_stale_routes_before_replacement`：新
  payload 前先清空全部旧 route 话题；route 变少后旧 topic 为空 Path。
- `test_all_path_layers_use_adjustable_rviz_path_displays`：13 个
  `rviz/Path` 话题、`Color`、`Line Width > 0`，并检查两个 marker 图层
  的新名称。
- `test_2d_pose_estimate_clears_every_old_display_layer`：2D Pose Estimate
  触发 corridor/guide/candidate 各一个 DELETEALL、最终 Path 与诊断 Path
  全部空消息，并结束旧 trajectory/active plan、hold 变为 START。

构建：

```text
catkin_make -C catkin_ws -j4
exit 0
[ 97%] Built target formation_planner_node
[100%] Built target fwis_v3_planner_node
```

日志：`logs/catkin_make_after_review.log`。

全量离线回归（在本次修改后运行）：

```text
python3 -m unittest discover -s tests -p 'test_*.py'
Ran 1054 tests in 557.418s
FAILED (errors=1)

ERROR: test_short_translation_has_finite_acceleration_and_zero_end_speed
       (test_continuous_retiming.ContinuousRetiming)
  File ".../tests/test_continuous_retiming.py", line 271
    self.assertLessEqual(timed.phases[0]["dynamic_peaks"]["acceleration"],
KeyError: 'dynamic_peaks'
```

日志：`logs/full_regression_after_review.log`。该失败与本次显示修改无关：
`tests/test_continuous_retiming.py` 和
`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/continuous_retiming.py`
相对 `HEAD` 均无改动；在 `git archive HEAD` 导出树
`/tmp/task01_head_smoke` 中单独运行该测试同样以
`KeyError: 'dynamic_peaks'` 失败（`logs/head_regression_error_reproduction.log`）。全量回归会改写
`evaluation/formation_refactor_v02/` 下部分计时字段；跑完后已用
`git restore --worktree -- evaluation/formation_refactor_v02` 恢复，
`git status --porcelain -- evaluation` 为 0，不触碰 `00_baseline/`…
`05_numerics/` 的历史证据。

### 3.1 ROS smoke（39 项中 38 项通过）

```text
tests/ros_formation_smoke.py --port 11399 \
  --output /tmp/task01_review_smoke_summary.json \
  --log-dir /tmp/task01_review_smoke_logs
SMOKE_EXIT=1
ros_formation_smoke: FAIL (39 checks, ...)
[FAIL] marker_status_feasible: status text='plan 424242 | HOLD_END | FEASIBLE
  T_end=5.889 s | FEASIBLE / independent audit and transactional publication
  no independent metrics reported'
```

证据：`logs/ros_smoke_after_review_summary.json`、
`logs/ros_smoke_after_review_stdout.log`、
`logs/ros_smoke_after_review_logs/`。其余 38 项通过，包括本次显示修改
直接相关的 `marker_frame`（17 个 marker）、`marker_unique_ids`、
`marker_obb_and_heading`、`marker_history_contains_all_shared_samples` 和
`path_topics_contain_all_shared_samples`（三个 `nav_msgs/Path` 覆盖全部
公共时间样本）。smoke 未通过的是状态文字里缺少 `rigidity=`/`obstacle=`，
本文件没有放宽该断言，也没有修改检查代码。

原因已定位，且**不是本次显示修改引入**：

1. smoke 使用 legacy 栈 `formation_optimizer_node.py` +
   `formation_playback.py`；legacy 的 `_quality()` 从来不在 quality report
   顶层写 `backend` 或 `model_sha256`（该文件相对 `HEAD` 无改动）。
2. playback 的 `_status_for_trajectory()` 仅在
   `quality_identity_matches(identity, quality)` 为真时合并
   `quality["pipeline"]`；`quality_identity_matches()` 要求和
   `backend`、`model_sha256` 完全匹配（该函数与
   `_status_for_trajectory()` 相对 `HEAD` 逐字节相同）。
3. 对本次 smoke 的实际 quality report 直接调用
   `quality_identity_matches()` 返回 `False`（`backend=None,
   model_sha256=None`），因此状态行落到
   `_metrics_text()` 的 “no independent metrics reported”。
4. 用 `git archive HEAD` 导出 `HEAD=77805cd` 到
   `/tmp/task01_head_smoke` 并单独 `catkin_make`（exit 0），运行同一
   smoke：同样 39 项、同样只有 `marker_status_feasible` 失败，状态文字
   逐字相同（见 `logs/ros_smoke_head_stdout.log`、
   `logs/ros_smoke_head_summary.json`、`logs/ros_smoke_head_build.log`）。
   HEAD 的 marker 计数是 20（5/车），当前是 17（4/车），正好对应本次
   Path 改造，但二者失败状态文字一致。
5. 历史 `evaluation/formation_s10/logs/ros_smoke.txt` 中通过的
   `rigidity=` 记录来自 2026-11-10（commit `d3dd1ac`），早于
   2026-11-18 的 commit `17a91e0`；后者才引入 quality identity 门控，
   因此该历史记录不能作为当前 `HEAD` 的通过依据。

## 4. 证据与边界

- 本文件与 `README_corridor_containment_analysis.md` 位于
  `work_flow/TASK01_evidence/08_manual_review/`。
- Step 6 的 `07_ros_rviz/` 证据记录的是修改前的 19 个 marker（5/车）和
  路径线 marker；本次修改后最终车辆 marker 为 17 个（4/车），指南层只剩
  文本标签，候选层只剩标签和 OBB，路径几何在 13 个 Path 话题上。
  历史会话证据没有回写或重跑，计数差异属于本次审核修改。
- 未修改 optimizer 数学、三车模型、A*/corridor 算法、audit/阈值。
- 未修改 `00_baseline/`…`05_numerics/`、`evaluation/`、技术方案或
  Codex 执行方案。
- 未 commit/push；未进入 TASK 02；未宣称 Codex PASS 或 TASK 01 验收。