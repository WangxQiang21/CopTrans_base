# V0 跟车标签与屏幕 HUD — 证据摘要

Run: `evaluation/virtual_a_corridor/V0/20261117T105425Z` (UTC, 2026-11-17)
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
Base commit: `9a19fd3e22d33188b13795f695757c3e4a36909f`
Current SHA: 同上（**未提交**，工作区含 V0+V1 改动；见 `git_state.log`）

## 交付内容

| 项 | 位置 |
| --- | --- |
| 紧凑标签 `car1 \| A \| v=1.23 m/s`、静态/无模型 `car1 \| STATIC \| v=0.00 m/s`、保留短 `F` | `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py` |
| 纯函数 `hud_text(playback_status, job_state, quality)` | 同上 |
| 审核列 `UNKNOWN/PENDING/PASS/FAIL`，仅同身份三层报告可 PASS | 同上 |
| quality `pipeline` 合并需 plan/formation/backend/model 同身份（virtual 另需 request/context） | 同上 |
| `/formation/hud` OverlayText 发布（10 Hz/状态变化，`hud_enabled/hud_left/hud_top/width/height`） | `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py` |
| demo 关闭长全局文字与 Cargo reason；库默认保留历史显示 | `formation_demo.launch`、`formation_demo.rviz` |
| 可选 HUD 运行依赖 | `catkin_ws/src/fwis_v3_playback/package.xml` (`jsk_rviz_plugins` exec_depend) |

标准 1280×720 默认：left=900, top=20, width=360, height=230。

## 验证结果

| 检查 | 结果 |
| --- | --- |
| `test_formation_hud`（15 项：标签、静态速度、审计列、身份门、display 开关） | **PASS** |
| `test_formation_demo_assets`（21 项：launch/RViz/run_demo/package.xml） | **PASS** |
| `test_virtual_a_ros_adapter`（60 项，含 HUD/质量身份发布回归） | **PASS** |
| `rospack find jsk_rviz_plugins` | **失败/未安装**（exit 1） |
| `rosmsg show jsk_rviz_plugins/OverlayText` | **失败/未安装**（exit 1） |
| `rospack plugins --attrib=plugin rviz \| grep overlay` | **无输出**（exit 1） |
| `/opt/ros/noetic/share/jsk_rviz_plugins` | **不存在**（exit 2） |
| 真实 RViz HUD 显示（开始/播放/失败截图） | **NOT_RUN** |

包状态：`ros-noetic-jsk-rviz-plugins` 未安装，apt 候选
`2.1.10-1focal.20250521.014225`。缺少该包时节点把
`hud_enabled` 置 False 并记录安装提示（`OverlayText` import 失败），
因此本轮**没有真实 `/formation/hud` 消息**，也没有 HUD 截图。
V0 真实 GUI 门为 **NOT_RUN**，不能判 V0 完整 PASS；纯单元/回归部分通过。

## 命令与退出码

见 `commands.log`；原始测试日志在 `test_logs/`；源码 hash 在
`source_hashes.txt`；完整增量 diff 在 `v0_v1_changes.patch`。

## 说明与遗留

- 本证据目录在 V1 实现完成后生成，测试针对同一未提交 V0+V1 工作区；
  source hash 与 V1 run `20261117T105607Z` 一致。
- identity 门使 legacy quality 报告的 `pipeline` 指标不再合并（legacy
  quality 无 backend/model）；这是 V0 “只有同身份三层报告可 PASS”的有意
  结果，需 Codex 在审 diff 时确认；历史轨迹包/模型/审核/消息 MD5 未改。
- 安装 `ros-noetic-jsk-rviz-plugins` 后需另跑真实 GUI 门：
  `run_demo.sh --backend virtual_rigid_a` 单次 S1，保存开始/播放/失败三张
  HUD 截图，并按本目录命令复核 OverlayText 字段与尺寸。
- 未提交 Git；未启动 C2；历史 `timing_continuous` 的
  `KeyError: dynamic_peaks` 仍未处理（本阶段未触碰）。