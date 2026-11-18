# V1 实际凸走廊可视化 — 证据摘要

Run: `evaluation/virtual_a_corridor/V1/20261117T105607Z` (UTC, 2026-11-17)
Worktree: `/home/wq/whut_thesis/expv3_multi_virtual_a`
Base commit: `9a19fd3e22d33188b13795f695757c3e4a36909f`
Current SHA: 同上（**未提交**；工作区含 V0+V1 改动，diff
`9 files changed, 907 insertions(+), 23 deletions(-)` + 2 个新测试/证据目录）

## 1. 输入与身份

- C1 输入（原样复制）：`input_sample_chain.json`
  - 来源 `evaluation/virtual_a_corridor/C1/20261117T073007Z/sample_chain.json`
  - sha256 `088a7cfe23174c50636fcf923e95760d14e744e164850a4df21c798f6752a734`
  - chain digest `d3e838f164a1a1bd32fa6d859da4ab0c5f302a3d0feccbfe13d84a10f6e6e801`
  - grid sha256 `97dd06c8c06614a5e9a961f7f643a4b08f30f95db6d3403ba10d1f88f4057734`
- **未发现 Codex 的 C1 ACCEPTED 独立验收产物**；本轮把该 C1 JSON 视为已审计输入
  使用，但 C1 验收状态本身是证据缺口，不由本摘要声称通过。
- 显示状态固定 `CANDIDATE_REFERENCE_POINT_ONLY`；`used_by_optimizer` 不出现，
  走廊显示不等于三车安全认证。
- 本 run 是 **C1 保存 JSON 直接发布**的显示测试（任务允许的 debug case），
  不是 planner→playback 的自主端到端；该端到端属 C2-B/V3。

## 2. 实现位置（未提交）

- `formation_continuous_a/corridor.py`：`corridor_display_fields(chain, identity)`
  直接导出原 `A/b/vertices`，身份/网格校验，chain digest。
- `formation_playback.py`（库）：`corridor_display_matches` 与
  `build_corridor_markers`；扇形三角化、边线、颜色、原子替换。
- `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`：消费
  `/formation/planning_debug`（latched JSON），发布 `/formation/corridors`；
  `_job_state_plan` 门拒绝旧 plan；admission/fail/clear 清空。
- `formation_uniform_planner_node.py`：单个 latched debug 发布，携带
  plan/formation/backend/grid/context/generation/route/digest/regions；
  `model_sha256=null` 时如实为 null；无 C1 链时只发 CLEARED。
- 车辆 body markers 仍走 `/formation/markers`，不混入走廊层。

## 3. 真实 ROS 显示证据（私有 master 127.0.0.1:11585）

session：playback `_backend:=virtual_rigid_a _hud_enabled:=false
_publish_tf:=false _transport_ui:=false`，独立 recorder，RViz
`v1_display.rviz`（窗口 155189265）。recorder 共 4 条消息：

| # | 事件 | 内容 |
| --- | --- | --- |
| 1 | playback 启动清空 | 1×DELETEALL |
| 2 | plan 91001 gen 1 route 0，完整 C1 链 | DELETEALL + 2 区（每区 fill+edge）= 5 markers |
| 3 | plan 91002 gen 2 route 3，仅 C1 区 0 | DELETEALL + 1 区（fill+edge）= 3 markers |
| 4 | 匹配 CLEARED（plan 91002 gen 2） | 唯一一条消息，唯一 DELETEALL |

- 每条替换都在 **同一个 MarkerArray** 内先 DELETEALL 再 ADD；新小链没有
  残留旧 route_0 id（消息 3 只有 route_3 的 2 个 ADD marker）。
- 截图（原图保留）：
  `screenshot_first.png`、`screenshot_second.png`、
  `screenshot_cleared.png`；带 digest/身份/数值注释：
  `*_annotated.png`。首图两区 fills 与边线均按原坐标显示；替换图只剩
  一个区；清空图走廊层为空。首轮 run `20261117T105033Z` 因 TRIANGLE_LIST
  `Marker.scale` 默认 0.1 曾出现缩小填充，本 run 已用 `scale=(1,1,1)` 修正
  （见该目录 `SUPERSEDED.md`）。

## 4. 发布几何与清理核对（`verify_corridor_markers.py`，exit 0）

`marker_verification.json` 直接对比 recorder 收到的实际 point 与 C1 导出：

- 消息 2 两区：fill 点数 6/9、edge 点数 5/6；所有 `point_xyz` 与导出顶点
  扇形/闭合边 **完全相等**；fill `scale=[1,1,1]`，edge `scale=[0.03,0,0]`；
  fill z=0.005 alpha=0.15，edge z=0.01 alpha=0.95（float32 传输值）。
- 面积：区 0 相对误差 `2.9e-16`，区 1 `0.0`。
- 消息 3 单区与 C1 区 0 顶点逐点相等，digest
  `1093ccc5a1ff02e3d235229a558eeebf65fd508ce8714be6ffe175894bdacdf8`。
- 负例：旧 plan 91001 gen 1 只发 debug → recorder 4→4；当前 plan 91002
  但 generation 99 → 4→4。playback 日志分别记录
  `current job is plan 91002` 与 `identity does not match the current admission`。
- 颜色由 route_id 确定性生成（route 0 红、route 3 品红），同输入重复一致。

## 5. 定向测试（同一源码 hash，禁止全量 discover）

| 模块 | 数量 | 结果 |
| --- | --- | --- |
| test_virtual_a_corridor_display | 15 | PASS |
| test_virtual_a_ros_adapter | 60 | PASS |
| test_virtual_a_receiver | 15 | PASS |
| test_virtual_a_playback | 34 | PASS |
| test_formation_playback | 22 | PASS |
| test_formation_hud | 15 | PASS |
| test_formation_demo_assets | 21 | PASS |
| test_virtual_a_corridor | 25 | PASS |
| test_formation_optimizer_node | 23 | PASS |
| test_virtual_a_pipeline | 8 | PASS（从 `tests/` 运行，192.19 s） |

`test_virtual_a_pipeline` 从仓库根首次运行 exit 1，原因是测试助手
`virtual_a_g4_support` 只在 `tests/` 路径下可导入；从 `tests/` 重跑通过。
错误与修正日志都保留在 `test_logs/`，未掩盖。

## 6. 失败、NOT_RUN 与说明

- **V0 真实 HUD GUI = NOT_RUN**：`jsk_rviz_plugins` 未安装；`OverlayText`
  import 失败时 playback 将 `hud_enabled` 置 False。详见
  `evaluation/virtual_a_corridor/V0/20261117T105425Z/`。
- **V1 自主 plan→display 端到端 = NOT_RUN**：本 run 使用 C1 JSON 直接
  发布；需要 C2-B/V3。
- 本 run 第二次 publish 的 `plan_request 91002` 被 playback 以
  “ignoring busy plan_request ... while job 91001 is PLANNING” 拒绝；显示
  身份来自随后匹配的 `/formation/job_state` PLANNING 记录（现有 playback
  路径，非 V1 新增状态机）。日志保留该 warning，未宣称 request 被 admit。
- 实时负例覆盖旧 plan 与错 generation；错 map/context/model 的拒绝由
  `test_virtual_a_corridor_display` 覆盖。fixture 的 context sha 为
  `"c"*64`，在 request/job/debug 三处一致使用，是显示测试的本地身份值，
  不声称是真实地图 context 摘要；`model_sha256=null` 如实表示无模型。
- 本 run 第一次启动尝试背景作业未继承 ROS 环境（playback
  `ModuleNotFoundError: fwis_v3_msgs`，RViz 连不上 master），未发布任何
  走廊消息；随后按 `commands.log` 的环境重启，本目录日志来自成功 session。
- recorder 在场景完成后对 SIGINT/SIGTERM 仍停在 `rospy.sleep`，最终 SIGKILL；
  4 条消息已逐条 flush，JSONL 完整。`pids/roscore.pid` 记录的是启动包装
  shell（66215），实际 rosmaster 66288 由 `ros_log/.../master.log` 定位并
  停止。失败首启还遗留 recorder 66330（其 pid 文件随后被成功 session
  覆盖）；最终排查时发现并 SIGKILL，它未写入任何消息（JSONL 仍 4 条）。
  端口 11585 已释放；用户 11411 session 全程未触碰。
- 走廊层不依赖 `jsk_rviz_plugins`：它只使用标准 `visualization_msgs`
  MarkerArray，因此本轮走廊显示不受缺失 HUD 包影响。
- 未修改轨迹包/模型/审核/消息 MD5；三车运动学、动态、连续 OBB、连续时间
  认证未削弱；portal/debug normals 默认不显示。
- 未提交 Git、未建分支、未 push、未进入 C2；历史
  `timing_continuous KeyError: dynamic_peaks` 仍未处理。

## 7. 文件

`commands.log`（精确命令/退出码）、`results.json`、`source_hashes.txt`、
`v0_v1_changes.patch`、`git_state.log`、`corridor_messages.jsonl`（含全部
point/scale/color）、`marker_verification.json`、`first/second/clear/stale/
wrong_generation_fixture.json`、`ros_log/`、`ros_home/`、`pids/`、
`cleanup.log`、三张原始与三张注释截图。