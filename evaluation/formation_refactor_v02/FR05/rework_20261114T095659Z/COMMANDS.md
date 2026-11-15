# FR05 最小返工：真实命令与退出码

所有命令都在 worktree `/home/wq/whut_thesis/expv3_multi_v02` 下执行；
`ENV` 指本目录的 `env.sh`（私有 master 11472、专用 ROS_HOME、
`OPENBLAS_NUM_THREADS=1`、同时 source `/opt/ros/noetic` 与本 worktree 的
`catkin_ws/devel`）。

```bash
export ENV=/home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR05/rework_20261114T095659Z/env.sh
export R=/home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR05/rework_20261114T095659Z
cd /home/wq/whut_thesis/expv3_multi_v02
```

## 1. 私有 master 与 overlay 构建

```bash
source "$ENV"
nohup /opt/ros/noetic/bin/roscore -p 11472 > "$R/logs/roscore.log" 2>&1 &
```

`run_demo.sh` 每次启动都会先构建 overlay（脚本内第 488 行）：

```bash
catkin_make -C /home/wq/whut_thesis/expv3_multi_v02/catkin_ws -j2 >/dev/null
source /home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/setup.bash
```

端口保护：smoke 拒绝 11311 / 11312 / 11320（`PROTECTED_PORTS`）。

实测（`logs/run_demo_main3.log` 47–68 行）：

```
NODES
  / formation_goal_bridge (fwis_v3_demo/formation_goal_bridge.py)
    formation_playback (fwis_v3_playback/formation_playback.py)
    formation_uniform_planner (fwis_v3_optimizer/formation_uniform_planner_node.py)
    fwis_map_publisher (fwis_map_server/map_publisher.py)
ROS_MASTER_URI=http://127.0.0.1:11472

/formation_uniform_planner/max_expansions: 10000
/formation_uniform_planner/max_seconds: 30.0
/formation_uniform_planner/goal_xy_tol: 0.02
/formation_uniform_planner/goal_psi_tol: 0.017453292519943295
/formation_uniform_planner/goal_heading_tol: 0.017453292519943295
/formation_uniform_planner/translation_step: 1.0
/formation_uniform_planner/collision_margin: 0.05
/formation_uniform_planner/sweep_spacing: 0.01
/formation_uniform_planner/publication_dt: 0.02
/formation_uniform_planner/frame_id: map
/formation_playback/rate: 100
/formation_playback/vehicle_ids: [1, 2, 3]
```

## 2. 主场景（empty_map，100 Hz 定量）

```bash
source "$ENV" && ./run_demo.sh \
  --map evaluation/formation_refactor_v02/FR05/empty_map --scale 1.0 \
  backend:=uniform_v02 rviz:=false playback_rate:=100 \
  > "$R/logs/run_demo_main3.log" 2>&1 &
# 等 demo 就绪后，启动 smoke（attach 到已运行的 launch，不自己拉起节点）：
source "$ENV" && python3 tests/ros_uniform_playback_smoke.py \
  --scenario main --output "$R/ros/main_report.json" \
  > "$R/logs/main_smoke3.log" 2>&1
echo EXIT=$?     # EXIT=0；ok=True；checks=112/112
```

结果：`ros/main_report.json`（ok=True，112 项检查全部为 true），
`ros/envelopes/envelope_00600000..00600003.json`。

## 3. near-goal 负例（同一主场景 demo 会话）

```bash
source "$ENV" && python3 tests/ros_uniform_playback_smoke.py \
  --scenario near_goal --output "$R/ros_near_goal/near_goal_report.json" \
  --planning-timeout 150 > "$R/logs/near_goal_smoke2.log" 2>&1
echo EXIT=$?     # EXIT=0；ok=True；(10,10,0)->(10.1,10,0.1)
```

结果：plan 600005 诚实失败，`status=BUDGET_EXHAUSTED
reason=TIME_BUDGET_EXHAUSTED layer=budget wall=30.0479 s`，预算 10000 / 30 s
未改。

## 4. failure 场景（blocked_map，100 Hz）

```bash
source "$ENV" && ./run_demo.sh \
  --map evaluation/formation_refactor_v02/FR05/blocked_map --scale 1.0 \
  backend:=uniform_v02 rviz:=false playback_rate:=100 \
  > "$R/logs/run_demo_failure.log" 2>&1 &

source "$ENV" && python3 tests/ros_uniform_playback_smoke.py \
  --scenario failure --output "$R/ros_failure/failure_report.json" \
  > "$R/logs/failure_smoke.log" 2>&1
echo EXIT=$?     # EXIT=0；ok=True；checks=57/57
```

## 5. tamper 场景（planner/bridge 关闭，重放基线 envelope）

```bash
source "$ENV" && ./run_demo.sh \
  --map evaluation/formation_refactor_v02/FR05/empty_map --scale 1.0 \
  backend:=uniform_v02 rviz:=false playback_rate:=100 \
  planner:=false goal_bridge:=false \
  > "$R/logs/run_demo_tamper.log" 2>&1 &

source "$ENV" && python3 tests/ros_uniform_playback_smoke.py \
  --scenario tamper --output "$R/ros_tamper/tamper_report.json" \
  --baseline-envelope "$R/ros/envelopes/envelope_00600000.json" \
  > "$R/logs/tamper_smoke.log" 2>&1
echo EXIT=$?     # EXIT=0；ok=True；checks=14/14
```

## 6. 可见 RViz 往返（GUI 定性）

```bash
source "$ENV" && export DISPLAY=:1 && ./run_demo.sh \
  --map evaluation/formation_refactor_v02/FR05/empty_map --scale 1.0 \
  backend:=uniform_v02 rviz:=true playback_rate:=30 \
  > "$R/logs/run_demo_rviz.log" 2>&1 &
```

真实 `xdotool` 点击（窗口 150994958，2D Pose Estimate → 2D Nav Goal，
逐帧 `import -window root` 截图）。以下为实际执行的命令：

```bash
# round 1（未缩放）：pose (1200,600)，goal (1448,313) -> plan 600000
export DISPLAY=:1
xdotool windowactivate --sync 150994958; sleep 0.3
xdotool mousemove --sync 376 81 click 1; sleep 0.5          # 2D Pose Estimate
xdotool mousemove --sync 1200 400; xdotool mousedown 1
xdotool mousemove --sync 1240 400; xdotool mouseup 1; sleep 1
# （随后用 1395,313 重新设置 pose，记录为 02_pose_set.png）
xdotool mousemove --sync 376 81 click 1; sleep 0.5
xdotool mousemove --sync 1395 313; xdotool mousedown 1; sleep 0.2
xdotool mousemove --sync 1435 313; xdotool mouseup 1; sleep 1.5
xdotool mousemove --sync 503 81 click 1; sleep 0.5          # 2D Nav Goal
xdotool mousemove --sync 1448 313; xdotool mousedown 1; sleep 0.2
xdotool mousemove --sync 1488 313; xdotool mouseup 1
for i in $(seq -w 1 44); do import -window root "$R/rviz/round1/play_$i.png"; sleep 0.25; done

# round 2（缩小比例后）：pose (1395,313) -> (11.401,9.670)
xdotool mousemove --sync 376 81 click 1; sleep 0.4
xdotool mousemove --sync 1395 313; xdotool mousedown 1; sleep 0.15
xdotool mousemove --sync 1435 313; xdotool mouseup 1; sleep 1.2
xdotool mousemove --sync 503 81 click 1; sleep 0.4          # goal (1435,313)->(1475,313)
xdotool mousemove --sync 1435 313; xdotool mousedown 1; sleep 0.15
xdotool mousemove --sync 1475 313; xdotool mouseup 1; sleep 1
xdotool mousemove --sync 1420 313; xdotool click --repeat 4 --delay 150 4   # zoom

# 缩放后（calibration 后的坐标）：pose (1184,322)，goal (1418,322)
xdotool windowactivate --sync 150994958; xdotool mousemove --sync 376 81 click 1; sleep 0.5
xdotool mousemove --sync 1184 322; xdotool mousedown 1; sleep 0.2
xdotool mousemove --sync 1224 322; xdotool mouseup 1; sleep 1.5
xdotool mousemove --sync 503 81 click 1; sleep 0.5
xdotool mousemove --sync 1418 322; xdotool mousedown 1; sleep 0.2
xdotool mousemove --sync 1458 322; xdotool mouseup 1
for i in $(seq -w 1 60); do import -window root "$R/rviz/round1/run3_$i.png"; sleep 0.25; done
```

第一次缩放后的同坐标点击（18:18:00 左右）因 plan 600002 仍在 PLANNING 被
bridge 拒绝；作业 FAILED 后原样重放（18:19:28 前后），得到 plan 600003。

记录：`rviz/round1/{00_initial,01_resized,02_pose_set,play_*.png,
zoom_*.png,run3_*.png}`、`clicks.json`（/initialpose 与
/move_base_simple/goal 的原始世界坐标）、`run3_meta.json`、
`NOTE_stray_round.md`（意外远目标 plan 600002 的诚实 30 s 预算失败）。

停止 demo（对真正的 roslaunch 进程发 SIGINT，不用 pgrep 模糊匹配）：

```bash
kill -INT <roslaunch fwis_v3_demo 的 python PID>
# 校验：无 formation_*/fwis_map_publisher/rviz 进程残留
```

## 7. 状态探针（统一审计状态与零运动认证）

```bash
source "$ENV" && python3 "$R/ros_probes/status_probe.py" \
  12.0 10.0 0.0 22 "$R/ros_probes/status_probe_1.json"
```

参数：目标 x y theta、采样时长、输出文件。结果：plan 600001，
`REQUEST_VALIDATED -> PUBLISHED -> HOLD_END`（optimizer，同一
`model_sha256=b0c895…`），playback `HOLD_END executable=True`，
`hold_end_certified=[True,True,True,True,True,True]`。该探针在私有 master
上的同一套代码/worktree 运行，时间戳早于主场景，属于补充探针，不计入上面
四个受控场景。

## 8. 相关单元回归

```bash
source "$ENV" && python3 -m unittest -v \
  tests.test_uniform_pipeline tests.test_uniform_search \
  tests.test_uniform_quality tests.test_formation_playback \
  tests.test_formation_hold tests.test_transport_contract \
  tests.test_uniform_playback_plan tests.test_uniform_feedback_identity \
  tests.test_formation_demo_assets \
  > "$R/logs/units_related_full.log" 2>&1
echo EXIT=$?     # EXIT=0；Ran 134 tests in 214.821s，OK
```

（`test_uniform_playback_plan.AuditIdentityBindingTest` 覆盖同 plan_id
不同摘要的两个各自自洽 envelope、缺失身份字段、正确摘要正例、
foreign-digest 拒绝、verified envelope 权威摘要、零运动保持与 legacy；
`test_uniform_feedback_identity` 直接驱动真实
`UniformPlannerNode.on_playback_status`；`test_formation_demo_assets`
锁定 launch 冻结参数与节点回退。）

## 9. 冻结与清单

```bash
{ find catkin_ws/src tests tools -type f \
    ! -path '*__pycache__*' ! -name '*.pyc' -print0 | sort -z \
    | xargs -0 sha256sum; sha256sum run_demo.sh; } > "$R/SOURCE_MANIFEST.sha256"
sha256sum -c "$R/SOURCE_MANIFEST.sha256"   # 全部 OK
```

`SOURCE_MANIFEST.sha256` 共 174 条目；git HEAD
`6227833011e2d92b711179bfdec979bb0b878b29`（worktree 为
`6227833-dirty`，本轮不提交）。

## 10. 保留的失败尝试（命令与退出码）

| 尝试 | 命令（相对上面章节） | 退出码 | 结果 |
|---|---|---|---|
| 主场景 attempt 1 | main 场景，同一命令，输出 `ros_attempts/main_attempt1/` | 1 | `s_to_a_interior_ticks_per_phase`：`initial_setup_0000` 只有 9 个内部显示时刻（阈值 11），在 `note()` 抛出 `SmokeError` |
| 主场景 attempt 2 | main 场景，同一命令，输出 `ros_attempts/main_attempt2/` | 1 | `playback_terminal_is_certified_for_that_digest: [False, True, True]`：playback 缓存的审计状态滞后于发布，首个 HOLD_END tick 仍为 AUDIT_PENDING；随后同一摘要认证到达 |
| near-goal attempt 1 | near_goal 场景，输出 `ros_attempts/near_goal_attempt1/` | 1 | `SmokeError('envelope_certification: ')`：smoke 手工轮询把 planner 在诚实预算失败后发布的 *无效占位* envelope 当成了可执行计划 |
| RViz 意外远目标 | 真实 GUI 点击（非受控），见 `rviz/round1/NOTE_stray_round.md` | — | plan 600002：`BUDGET_EXHAUSTED ... wall=30.180 s`，playback `CLEARED`、0 marker；忙时拒绝后续点击 |

每次失败都已原样保存（报告、日志、envelope、NOTE），未覆盖、未删除。