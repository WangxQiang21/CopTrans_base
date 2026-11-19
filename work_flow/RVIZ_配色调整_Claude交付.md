# RViz 路径分类配色与图层精简：Claude 交付

日期：2026-11-19。实施对象：`work_flow/RVIZ_路径分类配色与图层精简配置方案_V1.0.md`。
本文件只说明显示改造的完成情况与实测证据，不宣称 Codex PASS 或 TASK 01
验收，也不进入 TASK 02。所有未实际运行的项在“已知限制”中明确标注。

## 1. 变更范围

| 项 | 值 |
|---|---|
| 基线 HEAD | `222b919329bf487db1ee096e2902d753385a3527` |
| 显示改造提交 | `bee4e3aa0f86e24085a1840ee0581b2fca34add8` |
| 提交内容 | 仅显示配置、显示派生数据和对应测试 |
| 未包含 | `formation_continuous_a/geometry.py` 的 TASK01 返工；该文件仍在工作区未提交，不混入本提交 |
| 推送 | **未 push**；仅本地 commit |
| 算法影响 | 未改 NLP 数学、pipeline 候选算法、corridor 几何/审计、碰撞/动力学阈值、`/formation/playback_plan` 发布条件、v2 历史模型与哈希 |

提交文件：

```text
catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz
catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py
catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py
tests/test_formation_demo_assets.py
tests/test_virtual_a_corridor_display.py
tests/test_virtual_a_receiver.py
```

`git diff --check` 退出码 0；变更差异保存在
`work_flow/RVIZ_配色调整_evidence/display_source.diff`。

## 2. 配色与默认图层

| 类别 | 颜色（RViz `rviz/Path`） | 线宽 | 默认 |
|---|---|---|---|
| A* 原始路径 | `249; 115; 22`（`#F97316`） | `0.025` | 当前 active alias ON；`astar_raw_0/1/2` OFF |
| Portal Guide | `34; 197; 94`（`#22C55E`） | `0.035` | 当前 active alias ON；`portal_guide_0/1/2` OFF |
| F 点候选 | `250; 204; 21`（`#FACC15`） | `0.047` | ON |
| 三车候选中心 | `250; 204; 21`（`#FACC15`） | `0.024` | OFF |
| 三车最终已认证轨迹 | `56; 189; 248`（`#38BDF8`） | `0.065` | ON，三条完全同色 |
| 走廊填充 | `232; 121; 134`（`#E87986`）alpha `0.09` | — | ON |
| 走廊边缘 | `180; 83; 98`（`#B45362`）alpha `0.55` | `0.013` | ON |
| 候选 OBB | `148; 163; 184`（`#94A3B8`）alpha `0.38` | `0.010` | 稀疏 ON，每车最多 5 个全路径代表性 OBB |
| 失败位置 | `236; 72; 153`（`#EC4899`） | 十字标记 + 短标签 | 仅真实失败位置有 `s_m` 时绘制；无位置不画假点 |
| 播放中车身 | 保持原有 A/O/S 运动模式颜色 | 原宽度 | ON |

`/formation/markers`、地图、HUD、走廊保持 ON；Guide 标签、candidate center
标签、TF 默认 OFF；F 候选标签高度缩至 `0.14 m`。候选 OBB 的抽取仍是
`path.state_at_s()` 的真实状态，使用各车自己的 `theta`，没有用 `psi_F`
代替车身朝向。

## 3. active route/guide 的真实选择

在 `formation_playback.py` 增加了两个 latched `nav_msgs/Path` 发布器：

- `/formation/path/astar_active`
- `/formation/path/portal_guide_active`

选择规则由 `active_route_index(document)` 实现，来源是同一条已通过
`corridor_display_matches()` 身份检查的 planning 文档：

1. `route_index`、`route_count` 必须都是非 bool 的整数；
2. `route_count >= 1` 且 `0 <= route_index < route_count`；
3. 合法时仅复制该 `route_id` 已有的 A* 点列和 Guide 点列，不拟合、不
   平滑、不重采样、不优化；
4. 字段缺失、类型错误或越界时两个 active 话题保持空，**绝不默认 route 0**；
5. `_publish_guide_paths` 每次先清空 path registry，再写 raw 路径与 active
   alias；2D Pose Estimate/reset、身份不匹配和失败清理沿用原有清理路径。

如果 `route_index` 合法但对应 A* 或 Guide 不存在，对应 active 话题为空。
`PUBLISHED` 之后的蓝色轨迹仍只从 verified envelope 派生，不会把黄色
candidate 改色成蓝色。

## 4. 测试结果

命令与完整输出见 `work_flow/RVIZ_配色调整_evidence/targeted_tests.log`：

```text
Ran 102 tests in 37.910s
OK
exit=0
```

覆盖的关键用例：

- `test_all_path_layers_use_adjustable_rviz_path_displays`：15 个
  `rviz/Path` 显示，active alias 存在且默认启用；类别颜色/线宽和默认
  开关精确匹配；
- `test_active_aliases_follow_real_route_index`：`route_index=2` 时 active
  为 route 2；切到 route 1 时先清空旧 active；`route_index=3` 越界时清空；
- `test_active_alias_never_falls_back_to_route_zero`：只有 raw route 0 数据、
  但真实选择是 2 时，active 必须为空，raw 0 仍可独立显示；
- `test_active_guide_follows_corridor_route_id`：三套 corridor 中只显示
  `route_index=2` 对应的 A* 与 Guide；
- 2D Pose Estimate reset 清理用例：active alias 与旧 marker 一并清空；
- candidate OBB 每车 ≤5、颜色/线宽、F 标签 0.14 m、center/guide 标签默认
  关闭；
- corridor fill/edge 的颜色、alpha、边缘宽度。

## 5. 实机 ROS/RViz 场景

私有 master：`http://127.0.0.1:11633`。`11411` 的另一会话与 `11311` 的
既有进程均未触碰。场景为冻结 S1 地图
`evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/maps/S1.*`，
逐字实际参数见
`work_flow/RVIZ_配色调整_evidence/s1_failure_live/launch.log`。驱动结果：

```text
scenario=failure, goal_policy=straight
checks=155, failed_checks=[], result PASS
plan 600000: executable=true, 593 samples, HOLD_END
plan 600001: NO_ROUTE_GUIDE, 失败占位信封 model_json_length=0/sample_count=0
2D Pose Estimate reset: context virtual-a-reset-600001-3, initial_source=rviz_reset
plan 600002: executable=true, 593 samples, HOLD_END
```

其中：

- `failure_marker_clear`：旧 marker 序列 16→0，显示层已清空；
- `failure_clears_old_animation`：`new experiment 600001 admitted`；
- `failure_clear_screenshot.ok=true`；
- `plan1reset_no_request=0`、`recoveryreset_no_request=0`；
- 可执行失败信封为 0；失败场景的蓝色认证轨迹为空/已清除，HUD 显示
  `job FAILED | playback FAILED` 与 `NO_ROUTE_GUIDE`。

实际截图（1920×1080，RViz 窗口原尺寸，未裁剪）：

| 文件 | 用途 |
|---|---|
| `s1_failure_live/images/plan1_mid.png` | 已认证成功；蓝线、active A*/Guide、黄候选、稀疏 OBB、走廊背景 |
| `s1_failure_live/images/plan1_start.png` | 成功起点/图层默认状态 |
| `s1_failure_live/images/failure_cleared.png` | S1 的 `NO_ROUTE_GUIDE` 失败现场（不是历史 600004）：HUD 失败原因，无蓝色认证轨迹/假失败点 |
| `s1_failure_live/images/recovery_start.png` | 2D Pose Estimate 复位后的新实验 |
| `s1_failure_live/images/recovery_mid.png`、`recovery_end.png` | 复位恢复后的成功播放 |

截图 SHA-256 见 `work_flow/RVIZ_配色调整_evidence/screenshot_hashes.txt`。

### map05 U-turn 补充现场

S1 的失败原因是 `NO_ROUTE_GUIDE`，不是历史 `UTURN_DIAG` 的 map05 U-turn
600004。为直接覆盖“U-turn 失败现场”，另在同一私有 master `11633` 上用
`maps/map05-p12-parking-row-env2`、绝对比例 `2.625`、corridor/hard 运行
`uturn_failure_live/run_uturn_live.py`。实际结果：

```text
plan 600000 (对应历史 600003): FEASIBLE and published, 3184 samples,
  T_end=63.647 s, HOLD_END
2D Pose Estimate reset: start=(-3.625, 11.771, -0.022),
  context=virtual-a-reset-600000-3
plan 600001: job FAILED, HUD ENVELOPE_INVALID, route 0/3, step 7/8,
  layer=envelope, no executable envelope
```

`plan 600001` 的具体错误为
`PLAYBACK_VERIFICATION_FAILED: sample 3201 time 64.019999999999996 is not an integer-nanosecond time`。
这是本次真实观测到的失败，不是历史 600004 的 `SOFT_BUDGET_EXHAUSTED`；
交付没有把原因改写。失败现场截图保留了橙色 A*、绿色 Guide、黄色候选、
走廊填充和稀疏 OBB，HUD 明确显示 `job FAILED | playback FAILED`：

- `uturn_failure_live/images/uturn_success_start.png`、`uturn_success_mid.png`、
  `uturn_success_end.png`：U-turn 成功现场；
- `uturn_failure_live/images/uturn_failure_cleared.png`：U-turn 失败现场；
- `uturn_failure_live/report.json`：12 项检查全部通过；`planning_debug`
  记录 `route_index=0/route_count=3`，`astar_active` 与 `astar_raw_0` 同为
  739 点、同一 SHA-256，`portal_guide_active` 与 `portal_guide_0` 同为
  4 点、同一 SHA-256；`astar_raw_1/2` 为空，active 未回退到 0；
- 节点日志、`failure_job_state.yaml`、`failure_optimizer_status.yaml`、
  `failure_planning_debug.yaml`、`failure_playback_plan.yaml` 和停止记录在
  `uturn_failure_live/`。

该补充会话运行在当前工作区，包含未提交的 `geometry.py` 返工；它与
`bee4e3a` 的二进制不完全相同，只作为显示验收的额外真实现场，不用来证明
历史 600004 的 `SOFT_BUDGET_EXHAUSTED` 仍可复现。

### active alias 的现场采样

停止前采样 `plan 600002`：

```text
/formation/planning_debug: route_index=0, route_count=1, state=PUBLISHED
/formation/path/astar_active: 103 points, sha256 95ff2d69...
/formation/path/astar_raw_0:  103 points, sha256 95ff2d69...  (相同)
/formation/path/portal_guide_active: 2 points, sha256 cfcb86e...
/formation/path/portal_guide_0:      2 points, sha256 cfcb86e...  (相同)
```

`route_count=1` 时 `raw_1/2` 不存在，采样超时；active 没有因缺失而
伪造或回退 route 0。逐点记录见
`work_flow/RVIZ_配色调整_evidence/active_aliases_live/paths.json` 与
`live_summary.json`。

### 已知限制（不谎报）

1. 本次 real S1 实际选择为 route 0；没有在实机现场诱发 route 1/2 的真实
   选择。route 2、切换 route 1、越界清空和“不回退 0”由确定性单测覆盖，
   见 `targeted_tests.log`。没有把测试数据当作现场 route 1/2 证据。
2. 过期 planning 请求/身份不匹配拒绝由既有 receiver 单测覆盖；本次 RViz
   现场脚本没有重复构造 stale-request 实验。2D Pose Estimate 复位是实机
   运行的。
3. 截图拍摄时 RViz 窗口为原尺寸；颜色/线宽由配置和测试精确断言。实际
   屏幕像素受线宽、地图比例和抗锯齿影响，截图中部分线条较细，这不是
   重新着色或裁剪。
4. 未改算法、审计、阈值、发布条件或历史 v2 文件；`geometry.py` 的返工
   仍独立在未提交工作区，不包含在 `bee4e3a` 中。
5. map05 U-turn 补充现场实际失败在 `ENVELOPE_INVALID`（时间样本非整数
   纳秒），不是历史 600004 的 `SOFT_BUDGET_EXHAUSTED`；该现场选中 route
   0/3，未在实机诱发 route 1/2。S1 与 U-turn 的截图均为真实运行，未裁剪、
   未重新着色；U-turn 补充会话包含未提交 `geometry.py` 返工。

## 6. 复现命令

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH

python3 -m unittest -v tests.test_formation_demo_assets \
  tests.test_virtual_a_corridor_display tests.test_virtual_a_receiver
```

用户默认入口仍为：

```bash
./run_demo.sh
```

`run_demo.sh` 会在运行时用
`catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz` 生成地图自适应配置；
若指定了旧的 `rviz_config:=` 覆盖项，需要移除后重启才能看到新配置。

## 7. 交付状态

- 代码：本地 commit `bee4e3aa0f86e24085a1840ee0581b2fca34add8`，未 push；
- 证据：`work_flow/RVIZ_配色调整_evidence/`；
- 真实 RViz 截图：S1 7 张 + map05 U-turn 4 张，成功/失败/复位恢复均有；
- Codex 审核：尚未进行，本文件不代替 `RVIZ_配色调整_Codex审核.md`；
- TASK 02：未开始。