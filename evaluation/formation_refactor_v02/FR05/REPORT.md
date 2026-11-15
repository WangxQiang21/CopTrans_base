# FR05 报告：真实 ROS/RViz 交互与同轨迹回放

日期：2026-11-14  
工作区：`/home/wq/whut_thesis/expv3_multi_v02`（独立 worktree，未提交 Git）  
私有 master：`http://127.0.0.1:11411`  
状态：实现与 headless 验收完成；可见图形 RViz 一轮已完成并留有截图/动图，但**尚未取得真人目视确认**（见第 8 节）。

## 1. 结论摘要

- 新增原子封装 `fwis_v3_msgs/FormationPlaybackPlan`，内嵌原 `FormationTimedTrajectory` 与完整
  版本化 `UniformTimedPlan` 模型 JSON/摘要；旧消息定义未改。
- `/formation/playback_plan` 上**只有一个发布者**：`/formation_uniform_planner`；回放节点是唯一
  订阅者。`formation_demo.launch` 用 `backend` 条件保证旧 C++ 前端/旧优化器与 uniform_v02
  不会同时启动；RViz run 的 `rosnode list`/launch 日志证实了这一点。
- 回放不再线性插值点列，而是用 FR02 的 `UniformTimedPlan.sample/state_at` 解析采样；每车中心由
  `q_F` 与 offsets 派生，非零 pivot 弧（s_to_a 的 `A_ARC`，pivot=(0,2)，Δψ=0.6）和零角车的
  `S_SPIN` 都按模型回放。
- 墙钟 header 在正式发布前统一设置（planner 在 provisional stamp 上先自检，再重打信封和
  内嵌 header 的共同时间）；所有状态共享 job/plan_id/model_sha256。
- 单 job 忙准入：PLANNING/PLAYING 期间 Goal/InitialPose 被明确拒绝且不消费 start override；
  busy 只在回放到达真实 HOLD_END 后释放（实测 job 落后 playback 0.28–0.77 s），不以“轨迹已发布”
  当到达。无抢占。
- 主场景（100 Hz，两轮）全部检查通过：前进、O 横移、含零角车的统一 S→A、非零 pivot 回放、
  busy 拒绝、回放结束后第二/第三条实验。失败场景 49 项、篡改场景 9 项全部通过。
- 可见图形：DISPLAY=:1 上真实启动 RViz，实际点击工具条 2D Pose Estimate / 2D Nav Goal 并拖拽，
  在 root 窗口连续截图；plan 600002 从 (7.994,8.011) 到 (11.990,10.010)，T_end=21.672 s，
  回放 HOLD_END 后保持静止。截图中车辆位置由起点移动到目标（对应 offsets 的 (12.99,10.01)、
  (11.99,11.01)、(11.99,9.01)），见 `rviz/rviz_play_start.png`、`rviz_play_mid.png`、
  `rviz_play_end.png`、`rviz_playback.gif`。

## 2. 改动边界

新增：

- `catkin_ws/src/fwis_v3_msgs/msg/FormationPlaybackPlan.msg`
- `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py`
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_{motion,search,timing,trajectory,pipeline,quality}.py`
- `tests/ros_uniform_playback_smoke.py` 及 uniform 单元测试
- `evaluation/formation_refactor_v02/FR05/` 全部证据

修改（FR05 相关部分）：

- `formation_demo.launch`：`backend` 单一后端选择；frontend/legacy optimizer 仅在
  `backend=='legacy'` 启动，`formation_uniform_planner` 仅在 `backend=='uniform_v02'` 启动；
  playback/goal_bridge 接收同一 backend。
- `run_demo.sh`：`--backend`/`backend:=...` 统一解析，默认 `uniform_v02`，冲突/未知值退出 2，
  仅编队模式可用。
- `fwis_v3_msgs/CMakeLists.txt`：注册新 msg；旧 msg 列表不动。
- `fwis_v3_optimizer/CMakeLists.txt`：`catkin_install_python(... formation_uniform_planner_node.py)`。
- `fwis_v3_playback/scripts/formation_playback.py`：订阅 `/formation/playback_plan`，校验信封后
  以模型采样驱动 Marker/TF/status；失败时清空并以冻结静态 hold 停住。回放每帧在
  [formation_playback.py:894-923](catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py#L894-L923)
  用 `uniform_plan.state_at(clamped)` 解析求值（含 q_F、S theta、setup 与逐车中心），不是点列
  线性插值；展示侧每 tick 调用 `frame_at`（[formation_playback.py:456](catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py#L456)、
  [554](catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py#L554)）。
- `fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py`：信封自检
  `verify_uniform_playback_plan`（版本、digest、模型重建、offsets/中心/航向、每个样本 vs
  `state_at(t)`，时长/位置容差 1e-9）。
- `formation_goal_bridge.py`：busy 拒绝、manual override → 最新回放 TF（刚性/新鲜度检查）→
  配置 fallback 的起点选择；`initial_plan_id=600000`。
- `tests/test_formation_demo_assets.py`：新增 backend 选择/安装/run_demo 解析断言。

未改动：FR01 两个已验收文件、旧消息定义、车辆尺寸/offsets/r_min/轮角/轮速/安全阈值、旧实验与
Codex 证据。未提交 Git。

## 3. 接口与状态机

`FormationPlaybackPlan.msg`：`header, plan_id, formation_id, backend, semantic_version,
model_version, model_sha256, model_json, certification, FormationTimedTrajectory trajectory`。
`semantic_version="uniform_playback_plan/v1"`；信封 header stamp 必须等于内嵌 trajectory stamp。

回放状态：`HOLD_START → PLAYING → HOLD_END`；无效/失败信封发布
`state=CLEARED, cleared=True, executable=False, marker_count=0`，随后保持冻结静态 hold
（MarkerArray、TF、`state=STATIC`、reason "FAILED: planning"），不继续旧动画；冻结位置由
失败检查实测（第 5 节）。
planner/job：`IDLE → PLANNING → PLAYING → HOLD_END`，失败为 `FAILED`；忙碌时
`/formation/plan_request` 返回 `BUSY_REJECTED`，bridge 拒绝 Goal/InitialPose 且保留 override。
信封的 `certification` 是模型里的 `GEOMETRY_ONLY_NO_S9_NO_COLLISION`，只作显示；独立
FEASIBLE 审核状态到达前回放 status 的 `executable=False`，几何显示不被当作成功证书
（篡改场景基线即 `certification=GEOMETRY_ONLY, audit_status=AUDIT_PENDING`）。

## 4. 主场景（真实 run_demo，100 Hz，空地图）

命令见 `COMMANDS.md`；原始报告 `ros/main_report.json`（83 项检查全部 ok），封装原文在
`ros/envelopes/envelope_00600004..6.json`。同一场景在 100 Hz 下连续通过两轮
（`ros_attempts/main_100hz_pass1/main_report.json`，计划 600004–6；`pass2`，计划
600007–9；均为 83 项检查 ok）；更早的 50 Hz 与失败尝试也在 `ros_attempts/`，未覆盖删除。

| 实验 | plan | 起点 → 目标 | 原语 | T_end (s) | 规划墙钟 (s) | 样本 | 信封自检最差位置/角度 |
|---|---|---|---|---|---|---|---|
| forward | 600004 | (10,10,0) → (12,10,0) | A_STRAIGHT ×2 | 6.245537 | 0.8246（请求→发布 1.3076） | 316 | 2.90e-10 m / 0 |
| lateral | 600005 | (12,10,0) → (10,12,0) | O_TRANSLATION ×4 + 2 setup + 初始 | 15.429103 | 1.8698（2.4632） | 778 | 5.68e-10 m / 0 |
| s_to_a | 600006 | (10,10,0) → (12,11,0.6) | O, S_SPIN, A_ARC(pivot=(0,2),Δψ=0.6), O | 21.443573 | 15.3999（16.0633） | 1078 | 7.7e-11 m / 1.39e-10 rad |

每相位内部时刻（模型样本数，要求 ≥11）：

- forward：motion_0000=157，motion_0001=158。
- lateral：initial_setup=15，motion=157/157/157/109，setup=100/82。
- s_to_a：initial_setup=15，motion=157/106/212/138，setup=118/152/179。

回放显示侧每个相位内部 tick 数（100 Hz，Marker/status/TF）：

- forward：300/297（三路一致）；lateral 最少 14（initial_setup marker），其余 ≥104；
  s_to_a 最少 11（initial_setup marker），其余 ≥76。
- 显示最差误差：forward Marker/TF 5.86e-10 m，角度 0；lateral 5.73e-10 m；
  s_to_a Marker 6.19e-10 m / 2.90e-10 rad，TF 4.62e-10 m / 2.06e-10 rad。
  检查共同阈值 1e-3 m / 1e-3 rad；轨迹外壳/样本自检阈值 1e-9。

模型/采样/TF/Marker 一致性：全部状态（planner status、job_state、playback_status）同
plan_id 与 model_sha256；每车中心 = `q_F + R(φ_F)·offset`（`sample_centers_derived_from_qf`
误差 0），S 相位零角车 heading 保持（`s_phase_heading_deltas motion_0001 =
{1: 0.4636, 2: 0.0, 3: 0.0}`），非零 pivot 弧由解析模型采样。

共同墙钟：信封 stamp 在发布时统一写入；`stamp→receipt` 9.6/23.4/37.3 ms，回放终态相对 stamp
6.265/15.556/21.512 s（≈T_end+启动开销）。planner 记录 `verification_to_stamp_delay`
0.070–0.247 s。

### 4.1 busy 拒绝与第二请求

- 600004 PLAYING 期间注入假 plan 请求 1100004（Goal + InitialPose）：planner 返回
  `BUSY_REJECTED`（reason BUSY，`requests_seen` 不增加），bridge 记录 0 个新 plan 请求；
  job 保持 600004/PLAYING，假 plan_id 没有信封（`busy_planner_no_envelope` 检查通过）。
- busy 释放以真实回放终态为准：forward job HOLD_END 落后 playback HOLD_END 0.279 s，
  lateral 0.766 s，s_to_a 0.721 s。
- 回放 HOLD_END 后同一 run_demo 依次接受 lateral、s_to_a（start 来自上一终态），三者都完成
  HOLD_END；主报告 `forward/lateral/s_to_a` 的 job_states 序列均以 HOLD_END 结束。

## 5. 失败场景（blocked_map）

报告 `ros_failure/failure_report.json`（49 项检查全部 ok）。

- 基线 600010：(10,10)→(11,10)，T_end=3.122769 s，回放 HOLD_END，busy 释放（落后 0.099 s）。
- 失败 600011：目标 (13.5,10)；planner 墙钟 20.647 s ≥ 20 s 预算 →
  `TIME_BUDGET_EXHAUSTED`，`first_failure_layer=budget`，status REJECTED，
  `trajectory_published=False`，job FAILED，`model_sha256=null`；搜索统计
  expansions=6、candidates=858、collision_s=20.26、rejection_histogram
  {A_HEADING_REQUIRES_S_ALIGNMENT:517, MAP_OBB_COLLISION:21}。未把预算失败写成物理无解。
- 失败后回放：收到 CLEARED，`cleared=True`、`executable=False`、`marker_count=0`、
  latched 信封无效（`model_sha256` 为空/0 样本）。旧动画停止检查：最后 24 条 TF 消息、
  各 child 的位置 spread = 0，Marker 位置 spread = 0；冻结 hold 在 (11,10)，与基线终态误差 0。
- 失败后接受新实验：重试 600012 从冻结 hold (11,10) 起步，目标 (10,10)，T_end=3.122769 s、
  159 样本、每相位 158 内部样本，回放完成 HOLD_END（`failure_retry_completed=600012`）。

## 6. 篡改/未知版本（真实回放节点）

报告 `ros_tamper/tamper_report.json`（9 项检查全部 ok）。先用真实信封 600004 建立可播放基线，
再逐个发布 6 种变体，全部被拒：

| 用例 | 拒绝原因（片段） | cleared | executable | markers |
|---|---|---|---|---|
| unknown_semantic_version | unsupported playback semantic_version 'uniform_playback_plan/v0' | True | False | 0 |
| unknown_model_version | unsupported model_version 'uniform_timed_plan.v0' | True | False | 0 |
| digest_mismatch | model_sha256 mismatch | True | False | 0 |
| tampered_model | model_json 重建失败：REQUESTED_POSE_MISMATCH | True | False | 0 |
| tampered_sample | vehicle 1 sample 158 x 差 0.001 > 1e-9 | True | False | 0 |
| stamp_mismatch | envelope header stamp differs from embedded trajectory stamp | True | False | 0 |

每个拒绝后发布 cleared status（plan_id 为空、marker_count=0、executable=False）。晚到的旧
`FormationTimedTrajectory` 之后仍收到 79 条文档，但回放状态保持 PLAYING（`late_legacy`
`states_after_publish=['PLAYING']`），没有切换到旧轨迹。

## 7. 可见图形 RViz 验收（DISPLAY=:1）

环境：双显示器 3840×1080，GNOME；RViz 窗口 `0x900000e` 位于 (0,37)，1920×1043。
视图 TopDownOrtho 下地图原点在内容坐标 (1133.5,537)，28 px/m；屏幕 = 内容 + (0,37)。

真实 GUI 操作（xdotool 在 RViz 工具条点击/拖拽，非直接发 topic）：

- 第 1 轮：Pose (10.010,10.080) → Goal (11.957,10.080)，plan 600000，T_end=6.188735 s，
  wall=0.812 s，samples=313，hash `6207…4852`，PLAYING→HOLD_END。
- 第 2 轮（首次与 RViz 交互时点到 Nav Goal 按钮）：Goal (7.994,8.011)，起点取当前回放
  TF (11.957,10.080)，plan 600001，T_end=21.845508 s，wall=2.643 s，samples=1100，
  hash `f65e…059c`，HOLD_END。
- 第 3 轮（完整 Pose+Goal）：Pose (7.994,8.011) → Goal (11.990,10.010)，planner 日志
  `start from manual start`，plan 600002，T_end=21.672367 s，wall=2.651 s，samples=1092，
  hash `be64…dcdc`；`/initialpose` 与 `/move_base_simple/goal` 由 `click_probe2.json`
  记录（RViz 发布值 7.9936/8.0114/0 与 11.9903/10.0098/0）。PLAYING→HOLD_END 21.68 s。

证据：`rviz/rviz_round2_pose_goal.json`（两次真实工具发布）、`rviz/run_demo_rviz.log`
（节点/参数/plan 行）、`rviz/node_topic_inventory.txt`（live rosnode/rostopic），以及 root
窗口截图（含窗口标题与 RViz 画面）：

- `rviz/rviz_play_start.png`（t+5.6 s，起点附近）、`rviz_play_mid.png`（t+16.3 s）、
  `rviz_play_end.png`（t+25.5 s，回放已到 HOLD_END 的终态）、`rviz_hold_end.png`（t+45 s，
  保持静止）；
- `rviz/rviz_zoom_{start,mid,end}.png`：地图局部 2× 放大，可见三车从起点移动到目标；
- `rviz/rviz_playback.gif`：t+2.6…27.0 s 的 17 帧（1.5 s/帧）动图。
- 逐帧像素差：地图区域从 t+2.6 到 t+27.0 s 持续变化，t+30 s 之后只剩 RViz 状态栏文本变化，
  与回放 HOLD_END 时间一致。

注意：RViz run 使用 `playback_rate:=30`，因此“每相位至少 11 个内部显示时刻”的量化检查是在
100 Hz headless 主场景完成的；30 Hz 可见轮只做定性图形验收，未套用 11 tick 阈值。

## 8. 未验证项与限制（如实记录）

1. **真人目视确认未取得**：RViz 操作由自动化鼠标事件在真实窗口完成，截图/动图证明图形可见、
   规划—回放—停车流程完整；但没有真人手眼确认记录。计划要求“真人/可见图形”，本报告只声称
   可见图形一轮，请用户/Codex 复核截图后再作最终人工验收。
2. **未使用独立 ROS_HOME**：roscore 与 ROS 内部日志使用默认 `~/.ros/log/<uuid>`；仅端口
   11411 私有、每个 run 的 stdout 单独保存。计划 4.2 节的独立 ROS_HOME 未实现。
3. **未构建 install 空间**：本工作区使用 devel overlay（`run_demo.sh` 自动 `catkin_make`）；
   `catkin_install_python` 规则已加且 devel relay 存在，但 `catkin_ws/install/` 未生成。
4. **可见轮无定量 tick/误差断言**：30 Hz RViz 轮未附加 smoke 驱动器，故第 7 节只报告真实
   plan/回放状态与截图；定量一致性与 11 tick 检查来自第 4–6 节的 100 Hz headless 轮。
5. **未涉及**：控制器/Gazebo/新大面板、抢占、连续优化、FR06；旧 T5 r4 未作为 FR05 验收。

## 9. 定向回归

- `tests.test_formation_demo_assets, test_uniform_trajectory, test_uniform_timing,
  test_uniform_motion, test_uniform_playback_plan`：**61 tests OK**（日志
  `logs/unit_61.log`）。
- `tests.test_uniform_pipeline, test_uniform_search, test_uniform_quality,
  test_formation_playback, test_formation_hold, test_transport_contract`：**85 tests OK**，
  212.9 s（日志 `logs/unit_85.log`）。
- `tests.test_formation_optimizer_node`：**23 tests OK**（日志 `logs/unit_optimizer_node.log`）。

## 10. 证据索引（本目录）

```
COMMANDS.md                          启动/构建/测试/图形命令与环境
REPORT.md                            本报告
SOURCE_MANIFEST.sha256               关键源码/配置哈希（22 个文件）
ros/main_report.json                 主场景 83 项检查 + 原始状态/轨迹摘要
ros/envelopes/envelope_00600004..6.json  正式主场景三份真实信封（model_json + samples）
ros/envelopes/                           另保留早期 600001–3 的信封原文，未删除
ros_failure/failure_report.json      失败场景 49 项检查
ros_failure/envelopes/               失败基线/重试信封
ros_tamper/tamper_report.json        6 个篡改负例 + 基线 + 晚到旧轨迹
logs/                                定向回归与 run_demo 日志
rviz/                                真实 RViz 操作、截图、动图、节点清单
ros_attempts/                        保留的失败/重复尝试（main 50/100 Hz、failure 1–4、tamper 1–2）
tools/make_maps.py                   空地图/阻塞地图生成
empty_map/, blocked_map/             FR05 专用地图
```

## 11. 停止状态

FR05 已按计划完成实现、headless 验收和一轮可见图形验收；未提交 Git，未进入 FR06，未做全局
ROS 清理（私有 roscore 仍保留在 11411，供复核）。等待 Codex 验收。