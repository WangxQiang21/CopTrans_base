# RViz 配色与图层调整：证据目录

日期：2026-11-19。对应实施提交：`bee4e3aa0f86e24085a1840ee0581b2fca34add8`
（基线 `222b919329bf487db1ee096e2902d753385a3527`）。本目录是显示改造的
实测与测试证据，不是 Codex 审核结论，也不宣称 TASK 01 验收通过。

## 目录内容

| 文件/目录 | 内容 |
|---|---|
| `s1_failure_live/` | 任务私有 ROS master `127.0.0.1:11633` 上的 S1 “成功 → NO_ROUTE_GUIDE 失败 → 2D Pose Estimate 复位恢复”实机会话 |
| `s1_failure_live/report.json` | `ros_virtual_a_smoke.py` 的 155 项检查，0 failed；含 plan/信封/显示/重置/清除证据 |
| `s1_failure_live/report.stdout` / `.stderr` | 实机驱动输出；stdout 为 `G5 virtual smoke PASS: 155 checks, 0 failed` |
| `s1_failure_live/launch.log` | `run_demo.sh` 启动日志（实际 `map_dir`、后端、corridor/hard、straight 策略） |
| `s1_failure_live/images/` | 真实 RViz 窗口截图，原尺寸 1920×1080，未裁剪 |
| `s1_failure_live/nodes_raw/` | 本次 roslaunch 的全部节点日志（planner/playback/goal bridge/map） |
| `s1_failure_live/stop.txt`、`nodes_after_stop.txt` | 停止记录；停止后仅 `/rosout`，私有 master 已关闭 |
| `uturn_failure_live/` | map05 真实 U-turn 补充会话：现场 600000（对应历史 600003）认证成功 → 2D Pose Estimate 复位 → 现场 600001（对应历史 600004）失败；4 张未裁剪截图、原始 job/optimizer/planning_debug 采样、节点日志与 `report.json` |
| `uturn_failure_live/failure_optimizer_status.yaml` | 失败时 `/formation/optimizer_status` 原始消息；实际为 `ENVELOPE_INVALID`，不是历史 `SOFT_BUDGET_EXHAUSTED`，报告未改写原因 |
| `active_aliases_live/paths.json` | `plan 600002` 的 `astar_active` / `portal_guide_active` 与 `raw_0` / `guide_0` 的逐点 SHA-256、首尾点、点数；`route_index=0`、`route_count=1` |
| `active_aliases_live/planning_debug.yaml` | 同一时刻 `/formation/planning_debug` 原样采样 |
| `active_aliases_live/sample_active_aliases.py` | 只订阅证据脚本；不发布任何规划/重置/轨迹消息 |
| `runtime_formation.rviz` | 本次 S1 地图的实际运行时 RViz 配置（由 `tools/configure_formation_view.py` 从模板生成） |
| `targeted_tests.log` | 定向测试 102 项全部通过，含 active route 1/2 切换与“不回退到 route 0”用例 |
| `display_source.diff` | 仅显示相关源码与测试的改动（不含 `geometry.py` 返工） |
| `display_commit.txt`、`display_commit_show.txt`、`display_commit_check.log`、`display_files_sha256.txt` | 显示提交、提交内容与 whitespace 检查、6 个显示文件的 SHA-256 |
| `git_head.txt`、`git_status.txt`、`git_diff_stat.txt`、`git_diff_check.log` | 提交后的工作区状态；`git diff --check` 退出码 0 |
| `git_head_before_display_commit.txt`、`git_status_before_display_commit.txt`、`git_diff_stat_before_display_commit.txt` | 提交前的工作区快照 |
| `screenshot_hashes.txt` | 截图 SHA-256 |

## 实机场景

启动参数（`launch.log`）：

```text
backend:=virtual_rigid_a auto_goal:=false rviz:=true
virtual_a_goal_policy:=straight
map_dir:=<S1 冻结地图的运行时副本>
virtual_a_guidance_mode:=corridor
virtual_a_corridor_enforcement:=hard
scene:=manual_map
rviz_config:=<由 formation_demo.rviz 生成的运行时配置>
```

S1 地图来自
`evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/maps/S1.pgm` 与
`S1.yaml`（map SHA-256
`054cf5c77079536f9a098e33232732d6c181f0f624f07e3a446ebfab0266a10e`；
运行时 grid SHA-256
`6e94c751a40ec94431b495fcbff002945f09c975429c493c89060f0e7e2aa2a4`）。
驱动：

```bash
ROS_MASTER_URI=http://127.0.0.1:11633 DISPLAY=:1 \
python3 tests/ros_virtual_a_smoke.py \
  --scene-manifest evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S1 --scenario failure --goal-policy straight \
  --valid-goal 5.0,0.0,0.0 --failure-goal 10.0,0.0,0.0 \
  --planning-timeout 120 \
  --output work_flow/RVIZ_配色调整_evidence/s1_failure_live/report.json \
  --capture-dir work_flow/RVIZ_配色调整_evidence/s1_failure_live/images
```

结果：

| 阶段 | plan | 结果 |
|---|---|---|
| 首次成功 | 600000 | executable=true，593 samples，HOLD_END；蓝线来自认证信封 |
| S1 无可行路径失败 | 600001 | `NO_ROUTE_GUIDE`；失败占位信封 `model_json_length=0, sample_count=0`，可执行信封 0 |
| 2D Pose Estimate 复位 | — | `initial_source=rviz_reset`，context `virtual-a-reset-600001-3`；旧动画清除 |
| 复位后恢复 | 600002 | executable=true，593 samples，HOLD_END；16 markers |

所有 155 项 checks 中，`failure_marker_clear` 记录 16→0 的 marker 清理；
`failure_clear_screenshot.ok=true`；`failure_clears_old_animation` 为
`new experiment 600001 admitted`；`plan1reset_no_request=0` 与
`recoveryreset_no_request=0` 证明复位未携带上一请求。截图：

- `images/plan1_start.png`、`plan1_mid.png`、`plan1_end.png`：已认证成功现场；
- `images/failure_cleared.png`：S1 的 `NO_ROUTE_GUIDE` 失败现场（不是历史
  `UTURN_DIAG` 的 600004），HUD 明确显示
  `job FAILED | playback FAILED`、`NO_ROUTE_GUIDE`，无伪造失败点；
- `images/recovery_start.png`、`recovery_mid.png`、`recovery_end.png`：
  2D Pose Estimate 后的新实验。

## map05 U-turn 实机补充

S1 失败不是历史 `UTURN_DIAG` 的 map05 U-turn 600004。为直接覆盖“U-turn
失败现场”验收，另用 `maps/map05-p12-parking-row-env2`、绝对比例 `2.625`、
`guide=corridor/enforcement=hard/auto_goal=false` 在同一私有 master
`11633` 上运行 `uturn_failure_live/run_uturn_live.py`。命令与会话记录见
`uturn_failure_live/run_command.txt`、`launch.log`、`nodes_raw/` 和
`screenshot_hashes.txt`。

| 阶段 | plan | 实测结果 |
|---|---|---|
| U-turn 成功 | 现场 plan 600000（对应历史 600003） | `FEASIBLE and published`，3184 samples，`T_end=63.647 s`，`HOLD_END`；蓝线来自认证信封 |
| 2D Pose Estimate 复位 | — | 起点 `(-3.625, 11.771, -0.022)`；`virtual-a-reset-600000-3` |
| U-turn 失败 | 600001 | job `FAILED`；HUD `ENVELOPE_INVALID`，`route 0/3`，`step 7/8`；错误 `PLAYBACK_VERIFICATION_FAILED: sample 3201 time 64.019999999999996 is not an integer-nanosecond time`；无可执行信封 |

失败现场截图 `uturn_failure_live/images/uturn_failure_cleared.png` 保留本轮
真实 A* 橙线、Guide 绿线、黄色候选、走廊填充和稀疏 OBB；HUD 明确显示
`job FAILED | playback FAILED`。成功现场为
`images/uturn_success_start.png`、`uturn_success_mid.png`、
`uturn_success_end.png`。`uturn_failure_live/report.json` 的 12 项检查全部
通过，且没有把失败原因改写成历史 `SOFT_BUDGET_EXHAUSTED`。

补充采样 `plan 600001`（失败时路由仍保留在显示层）：

- `planning_debug`：`route_index=0`、`route_count=3`、`state=FAILED`；
- `astar_active` 与 `astar_raw_0`：均为 739 点，逐点 SHA-256 均为
  `9756027d927b1944991cf550145789c54c2e1370d505560bf75a1fd68cbf06af`；
- `portal_guide_active` 与 `portal_guide_0`：均为 4 点，逐点 SHA-256 均为
  `0dc4288a0565718a1363f945b9e7b3696a23d12ae86619f497db5b9df366410d`；
- `astar_raw_1/2` 为空，`portal_guide_1/2` 不存在；active 没有回退到 0。

**该补充会话的边界**：它运行在当前工作区，包含未提交的 TASK01
`geometry.py` 返工，因此二进制度与 `bee4e3a` 不完全相同；它只用于显示
验收的额外真实现场，不用来宣称历史 600004 的 `SOFT_BUDGET_EXHAUSTED`
仍可复现。该现场选中的仍是 route 0（3 条路线中的第 0 条），未在实机诱发
route 1/2。

## active alias 的实测身份

停止前采样 `plan 600002`：

- `planning_debug`：`route_index=0`、`route_count=1`、`state=PUBLISHED`；
- `astar_active` 与 `astar_raw_0`：均为 103 点，逐点序列
  SHA-256 均为 `95ff2d69d8d2c9fc76a9e382fa9bbe80a5f51aaf3d8a101b8a2c6a076c189c18`；
- `portal_guide_active` 与 `portal_guide_0`：均为 2 点，逐点序列
  SHA-256 均为 `cfcb86ebceeb8c0231776ada537313e9fe43f6bf761051f919735319c5601dbb`。
- `raw_1/2`、`guide_1/2` 在 `route_count=1` 的该场景中不存在，采样超时；
  这不是回退到 0：active topic 仍与真实 `raw_0/guide_0` 完全一致。

**live 限制（不隐瞒）**：本次 real S1 场景实际选中的是 route 0，未能
在现场诱发一个求解器实际选择的 route 1/2。route 2 选中、切换到 route
1、越界 `route_index=3` 清空、以及“active 缺失时绝不回退 route 0”的
确定性用例在 `targeted_tests.log` 中全部通过；没有把测试数据冒充现场
route 1/2。若要现场复现，需要构造让 route 0 的走廊/求解失败并触发
既有 route switch 的场景，本次没有为此改地图或改算法。

## 截图与配置说明

- 截图来自真实 RViz 窗口，`1920×1080` 原始窗口抓取，未裁剪，未重新着色；
- 运行时 `runtime_formation.rviz` 中 `rviz/Path` 的颜色/线宽采用
  `A* #F97316`、`Guide #22C55E`、`Candidate #FACC15`、`Certified #38BDF8`；
  active alias 两条 display 默认 ON，raw `_0/_1/_2` 默认 OFF；
- 候选 OBB 每车最多 5 个，来自真实 `state_at_s()`；F 标签高度 0.14 m；
  candidate center 标签与 guide 标签默认关闭；TF 默认关闭；
- 走廊填充 `#E87986` alpha 0.09、边缘 `#B45362` alpha 0.55、宽 0.013；
  失败色 `#EC4899` 仅在真实失败位置存在时绘制，`NO_ROUTE_GUIDE`
  无 s 坐标时不绘制假失败点；
- 分辨率、轨迹、碰撞/动力学/NLP 数值均未改动；本次提交只涉及显示派生
  数据与 RViz 配置。`geometry.py` 的 TASK01 返工不包含在 `bee4e3a` 中。

## 复现

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
python3 -m unittest -v tests.test_formation_demo_assets \
  tests.test_virtual_a_corridor_display tests.test_virtual_a_receiver
```

`targeted_tests.log` 的结果为 `Ran 102 tests in 37.910s`、`OK`。