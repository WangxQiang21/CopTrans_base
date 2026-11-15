# FR05 启动命令与环境记录

日期：2026-11-14  
工作区：`/home/wq/whut_thesis/expv3_multi_v02`（独立 worktree，未提交 Git）  
私有 master：`http://127.0.0.1:11411`（未使用 11311/11312/11320，未做全局清理）

## 1. 环境

`/tmp/fr05_run/env.sh`：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11411
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1
export PYTHONUNBUFFERED=1
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/setup.bash
```

私有 roscore（PID 3039917）：

```bash
source /opt/ros/noetic/setup.bash
ROS_MASTER_URI=http://127.0.0.1:11411 ROS_HOSTNAME=127.0.0.1 ROS_IP=127.0.0.1 \
  /opt/ros/noetic/bin/roscore -p 11411
```

说明：本轮未设置独立 `ROS_HOME`；ROS 内部日志位于 `~/.ros/log/a100fd5c-dff4-11f1-a57b-1b4c90db486b/`，
每个 run 的 stdout/stderr 单独保存在 `/tmp/fr05_run/` 并复制到本目录证据中。这是与计划
4.2 节“独立 ROS_HOME/日志”的一处偏差，已如实记录；端口仍是私有 11411，未触碰其它实验
的 master。

## 2. overlay 构建

`run_demo.sh` 自己构建并 source 本 worktree 的 overlay（`run_demo.sh:487-490`）：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -C /home/wq/whut_thesis/expv3_multi_v02/catkin_ws -j2
source /home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/setup.bash
```

`FormationPlaybackPlan.msg` 已加入 `fwis_v3_msgs/CMakeLists.txt` 的
`add_message_files`，生成物 `catkin_ws/devel/lib/python3/dist-packages/fwis_v3_msgs/msg/_FormationPlaybackPlan.py`
存在。`formation_uniform_planner_node.py` 已加入 `catkin_install_python` 的
`PROGRAMS`；devel relay `catkin_ws/devel/lib/fwis_v3_optimizer/formation_uniform_planner_node.py`
存在。本工作区使用 devel 空间，未构建 `catkin_ws/install/`。

## 3. 无图形（headless）验收

主场景（空地图，前进 A / O 横移 / 含零角车统一 S→A / busy 拒绝 / 终态后第二请求）：

```bash
cd /home/wq/whut_thesis/expv3_multi_v02
export ROS_MASTER_URI=http://127.0.0.1:11411 ROS_HOSTNAME=127.0.0.1 ROS_IP=127.0.0.1
export OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1
./run_demo.sh --map evaluation/formation_refactor_v02/FR05/empty_map --scale 1.0 \
  backend:=uniform_v02 rviz:=false playback_rate:=100 \
  > /tmp/fr05_run/run_demo_main.log 2>&1 &
python3 tests/ros_uniform_playback_smoke.py --scenario main \
  --output evaluation/formation_refactor_v02/FR05/ros/main_report.json
```

失败场景（blocked_map；先完成一条可达轨迹，再让同一 job 因 20 s 预算失败，然后从冻结
hold 接受第二条实验）：

```bash
./run_demo.sh --map evaluation/formation_refactor_v02/FR05/blocked_map --scale 1.0 \
  backend:=uniform_v02 rviz:=false playback_rate:=100 \
  > /tmp/fr05_run/run_demo_blocked.log 2>&1 &
python3 tests/ros_uniform_playback_smoke.py --scenario failure \
  --output evaluation/formation_refactor_v02/FR05/ros_failure/failure_report.json
```

篡改/未知版本场景（回放节点单独运行，直接向 `/formation/playback_plan` 重放真实封装）：

```bash
./run_demo.sh --map evaluation/formation_refactor_v02/FR05/empty_map --scale 1.0 \
  backend:=uniform_v02 rviz:=false playback_rate:=100 \
  planner:=false goal_bridge:=false \
  > /tmp/fr05_run/run_demo_tamper.log 2>&1 &
python3 tests/ros_uniform_playback_smoke.py --scenario tamper \
  --baseline-envelope evaluation/formation_refactor_v02/FR05/ros/envelopes/envelope_00600004.json \
  --output evaluation/formation_refactor_v02/FR05/ros_tamper/tamper_report.json
```

主场景在 100 Hz 回放率下连续通过两轮；第一轮报告保存在
`ros_attempts/main_100hz_pass1/`，第二轮在 `ros_attempts/main_100hz_pass2/`，当前
`ros/main_report.json` 为 600004–600006 的正式证据。失败/篡改场景的失败尝试分别保留在
`ros_attempts/failure_attempt1..4/`、`ros_attempts/tamper_attempt1..2/`。

## 4. 可见图形 RViz 验收（DISPLAY=:1）

```bash
DISPLAY=:1 ./run_demo.sh --map evaluation/formation_refactor_v02/FR05/empty_map --scale 1.0 \
  backend:=uniform_v02 rviz:=true playback_rate:=30 \
  > /tmp/fr05_run/run_demo_rviz.log 2>&1 &
```

RViz 窗口：`0x900000e`（150994958），client 绝对位置 (0,37)，大小 1920×1043。
视图中地图原点在窗口内容坐标 (1133.5, 537)，比例 28 px/m（30 m 地图 840 px；10 m
Grid 每格 28 px），屏幕坐标 = 内容坐标 + (0,37)。

真实工具条点击 + 拖拽（不是直接发 topic）：

```bash
export DISPLAY=:1
xdotool windowmap 150994958
xdotool windowactivate --sync 150994958
# 2D Pose Estimate：内容 (362,41)，拖拽方向 = 车头方向
xdotool mousemove --sync 362 80 click 1
xdotool mousemove --sync 1358 350; xdotool mousedown 1
xdotool mousemove --sync 1400 350; xdotool mouseup 1
# 2D Nav Goal：内容 (486,41)
xdotool mousemove --sync 486 80 click 1
xdotool mousemove --sync 1470 294; xdotool mousedown 1
xdotool mousemove --sync 1512 294; xdotool mouseup 1
```

（上表坐标对应起车 (8,8)、目标 (12,10)；`xdotool` 的 y 是屏幕坐标，已含窗口 37 px
offset。`rviz_round2_pose_goal.json` 记录了 RViz 实际发布的 `/initialpose`
(7.9936, 8.0114, 0) 与 `/move_base_simple/goal` (11.9903, 10.0098, 0)。）

可见回放截图使用 root 抓取，避免 `import -window` 的合成缓存导致重复画面：

```bash
for i in $(seq -w 1 50); do
  import -window root /tmp/fr05_run/visible/round2_$i.png
  sleep 0.5
done
```

关键帧与动图已保存到 `FR05/rviz/`（见 REPORT 第 7 节）。

## 5. 定向回归

```bash
cd /home/wq/whut_thesis/expv3_multi_v02
python3 -m unittest tests.test_formation_demo_assets tests.test_uniform_trajectory \
  tests.test_uniform_timing tests.test_uniform_motion tests.test_uniform_playback_plan -v   # 61 tests OK
python3 -m unittest tests.test_uniform_pipeline tests.test_uniform_search \
  tests.test_uniform_quality tests.test_formation_playback tests.test_formation_hold \
  tests.test_transport_contract -v                                                          # 85 tests OK (212.9 s)
python3 -m unittest tests.test_formation_optimizer_node -v                                  # 23 tests OK
```