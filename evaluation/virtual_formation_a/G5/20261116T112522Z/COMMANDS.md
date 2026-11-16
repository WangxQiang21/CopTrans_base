# Claude G5 Command Record

- 证据根：`/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z`
- 基线 HEAD：`567bdecba9676313da3bc9a97b82f57b5b569a2b`
- 分支：`feature/virtual-rigid-formation-a`
- 记录日期：2026-11-16
- Codex 独立验收：**NOT_RUN / NOT_CLAIMED**

本文件记录 G5 实际执行的命令。只读查询（`grep`、`sed`、`ls`、`ps`、`rosnode list` 等）除关键校验外不逐条列入；所有日志、报告和失败 run 均保留在证据根。命令中的 `$R`、`$E` 为下列固定路径的简写，实际执行时也使用过等价的绝对路径：

```bash
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E=$R/evaluation/virtual_formation_a/G5/20261116T112522Z
```

## 0. 通用环境

```bash
source /opt/ros/noetic/setup.bash
source "$R/catkin_ws/devel/setup.bash"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
```

新后端真实 ROS 使用私有 master `http://127.0.0.1:11585`；旧后端使用另一个私有 master `http://127.0.0.1:11586`。两者从未同时作为同一后端启动，也未使用默认端口 11311 或无关的既有 11411 master。

`run_demo.sh --scale 1.0` 会把 G4 场景地图复制/缩放到 `/tmp/expv3-map-scale.*`；各 run 的 `logs/launch*.log` 第一行记录了实际 `map_dir`。这些 `/tmp` 目录是临时输入，不是证据本体。

## 1. 构建

```bash
cd "$R/catkin_ws" && {
  source /opt/ros/noetic/setup.bash && catkin_make;
} > "$E/logs/catkin_make_final_verify.log" 2>&1
rc=$?
echo "catkin_make_rc=$rc" | tee -a "$E/logs/catkin_make_final_verify.log"
exit $rc
```

实际结果：`catkin_make_rc=0`。

## 2. G5-A：纯 Python / hook 测试

```bash
R=/home/wq/whut_thesis/expv3_multi_virtual_a
L="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/logs"
cd "$R"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1

for p in test_virtual_a_ros_adapter.py test_virtual_a_playback.py \
         test_virtual_a_receiver.py test_uniform_feedback_identity.py \
         test_uniform_playback_plan.py; do
  echo "=== $p"
  python3 -B -m unittest discover -s tests -p "$p" -v > "$L/$p.log" 2>&1
  rc=$?
  tail -3 "$L/$p.log"
  echo "RC=$rc"
done

for p in test_formation_playback.py test_formation_demo_assets.py; do
  echo "=== $p"
  python3 -B -m unittest discover -s tests -p "$p" -v > "$L/$p.log" 2>&1
  rc=$?
  tail -3 "$L/$p.log"
  echo "RC=$rc"
done
```

测试数：adapter 58、playback 34、receiver 15、old feedback 10、uniform playback plan 26、formation playback 22、formation demo assets 18，全部 OK。

## 3. G5-B：新后端真实 ROS/RViz

### 3.1 master

```bash
# 启动时尚在本 G5 的早期工作目录；该 master 随后被全部最终 run 复用。
VA_RUN=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T111547Z
cd "$R"
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$VA_RUN/ros_new"
mkdir -p "$ROS_LOG_DIR"
setsid roscore -p 11585 > "$VA_RUN/logs/roscore_11585.log" 2>&1 < /dev/null &
echo $! > "$VA_RUN/logs/roscore_11585.pid"      # wrapper PID 3938556
ROS_MASTER_URI=http://127.0.0.1:11585 rosparam list >/dev/null 2>&1 && echo ROSCORE_READY
```

master/rosout 实际 PID 为 `3938581`/`3938606`（见 `$VA_RUN/logs/roscore_11585.log` 与最终 `logs/final_process_cleanup.txt`）。`VA_RUN` 是 G5 的早期工作目录，不是最终证据根；最终 run 的 launch/driver 只改写各自 `ROS_LOG_DIR`。

### 3.2 launch

```bash
# FREE — G4 S1
cd "$R"
source /opt/ros/noetic/setup.bash
source "$R/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/final_main_free_6/nodes_raw"
"$R/run_demo.sh" \
  --map "$R/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S1" \
  --scale 1.0 backend:=virtual_rigid_a auto_goal:=false rviz:=true \
  virtual_a_goal_policy:=free > "$E/final_main_free_6/logs/launch.log" 2>&1
```

```bash
# STRAIGHT — G4 S2
cd "$R"
source /opt/ros/noetic/setup.bash
source "$R/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/final_main_straight_3/nodes_raw"
"$R/run_demo.sh" \
  --map "$R/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S2" \
  --scale 1.0 backend:=virtual_rigid_a auto_goal:=false rviz:=true \
  virtual_a_goal_policy:=straight > "$E/final_main_straight_3/logs/launch.log" 2>&1
```

```bash
# failure — G4 S2（attempt2 为最终证据；attempt1 目录保留）
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/final_failure_straight_2/nodes_raw"
/home/wq/whut_thesis/expv3_multi_virtual_a/run_demo.sh \
  --map /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S2 \
  --scale 1.0 backend:=virtual_rigid_a auto_goal:=false rviz:=true \
  virtual_a_goal_policy:=straight > "$E/final_failure_straight_2/logs/launch_attempt2.log" 2>&1
```

```bash
# tamper — G4 S2，独立单后端会话；未发送目标/重置请求
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/tamper_session_2/nodes_raw"
/home/wq/whut_thesis/expv3_multi_virtual_a/run_demo.sh \
  --map /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S2 \
  --scale 1.0 backend:=virtual_rigid_a auto_goal:=false rviz:=true \
  virtual_a_goal_policy:=straight > "$E/tamper_session_2/logs/launch.log" 2>&1
```

实际 `/tmp` 地图目录（launch log 原文）：FREE `ODl2nX/S1`；STRAIGHT `oRGQNe/S2`；failure `KTqpfp/S2`；tamper `3qowGR/S2`。节点 PID：FREE 3990067–3990071；STRAIGHT 3988402–3988406；failure 3984534–3984538；tamper 3986228–3986232。

### 3.3 smoke driver

```bash
# FREE：S1 main / free；83 checks
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/final_main_free_6
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/nodes_raw"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
set -o pipefail
python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tests/ros_virtual_a_smoke.py \
  --scene-manifest /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S1 --scenario main --goal-policy free \
  --output "$E/report.json" --capture-dir "$E/images" --planning-timeout 120 \
  2>&1 | tee "$E/logs/driver.log" | tail -25
```

```bash
# STRAIGHT：S2 main / straight；154 checks
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/final_main_straight_3
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/nodes_raw"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
set -o pipefail
python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tests/ros_virtual_a_smoke.py \
  --scene-manifest /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S2 --scenario main --goal-policy straight \
  --output "$E/report.json" --capture-dir "$E/images" --planning-timeout 120 \
  2>&1 | tee "$E/logs/driver.log" | tail -25
```

```bash
# failure：S2 failure / straight；155 checks
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/final_failure_straight_2
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/nodes_raw"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
set -o pipefail
python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tests/ros_virtual_a_smoke.py \
  --scene-manifest /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S2 --scenario failure --goal-policy straight \
  --output "$E/report.json" --capture-dir "$E/images" --planning-timeout 120 \
  2>&1 | tee "$E/logs/driver_attempt2.log" | tail -25
```

```bash
# tamper 主会话：S2 main / straight；154 checks
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/tamper_session_2
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/nodes_raw"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
set -o pipefail
python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tests/ros_virtual_a_smoke.py \
  --scene-manifest /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S2 --scenario main --goal-policy straight \
  --output "$E/report_main.json" --capture-dir "$E/images" --planning-timeout 120 \
  2>&1 | tee "$E/logs/driver_main.log" | tail -25
```

```bash
# tamper 负例：重放 plan 600001 baseline envelope；27 checks
E=/home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G5/20261116T112522Z/tamper_session_2
test -s "$E/envelopes/envelope_00600001.json" || { echo MISSING; exit 2; }
cd /home/wq/whut_thesis/expv3_multi_virtual_a
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11585
export ROS_LOG_DIR="$E/nodes_raw"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export DISPLAY=:1
set -o pipefail
python3 -B /home/wq/whut_thesis/expv3_multi_virtual_a/tests/ros_virtual_a_smoke.py \
  --scene-manifest /home/wq/whut_thesis/expv3_multi_virtual_a/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json \
  --scene-key S2 --scenario tamper \
  --baseline-envelope "$E/envelopes/envelope_00600001.json" \
  --output "$E/report_tamper.json" --planning-timeout 120 \
  2>&1 | tee "$E/logs/driver_tamper.log" | tail -25
```

首次 tamper 使用 `envelope_00600000.json` 时 driver 等待超时（会话实际在 plan 600001 `HOLD_END`），已保留为 `report_tamper_attempt1.json`；最终证据使用 `envelope_00600001.json`。

### 3.4 截图

截图由 driver 内部完成：`--capture-dir "$E/images"` 会在 start/mid/end 时刻调用 `xdotool` 聚焦/最大化 RViz、关闭 “End-of-Life” 对话框，并用 `import -window <RViz窗口>` 捕获真实窗口；失败只记为 capture 结果，不影响数值检查。环境为 `DISPLAY=:1`（`XAUTHORITY=/run/user/1000/gdm/Xauthority`）。没有用 mock 图片替代数值证据。

## 4. G5-C：旧后端与旧证据

### 4.1 G0 未修改 gate（被拒绝，保留原日志）

```bash
R=/home/wq/whut_thesis/expv3_multi_virtual_a
cd "$R"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
set -o pipefail
python3 -B tools/virtual_a_baseline.py \
  --output evaluation/virtual_formation_a/G5/20261116T112522Z/legacy \
  --regression full \
  2>&1 | tee evaluation/virtual_formation_a/G5/20261116T112522Z/logs/g0_runner_attempt1.log | tail -40
echo "RC=${PIPESTATUS[0]}"
```

实际以 `UNKNOWN_WORKSPACE_MODIFICATIONS` 拒绝；日志随后保留为 `logs/g0_runner_unmodified_gate.log`，未修改 runner 或其 allowlist。

### 4.2 G0 披露 wrapper（实际生成本次 legacy 结果）

```bash
R=/home/wq/whut_thesis/expv3_multi_virtual_a
B="$R/evaluation/virtual_formation_a/G5/20261116T112522Z"
cd "$R"
source /opt/ros/noetic/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
set -o pipefail
python3 -B "$B/baseline_runner_disclosed_allowlist.py" \
  --output "$B/legacy" --regression full \
  2>&1 | tee "$B/logs/g0_runner_disclosed_full.log" | tail -40
echo "RC=${PIPESTATUS[0]}"
```

wrapper 只 import 未修改的 `tools/virtual_a_baseline.py`，在内存中分类已存在/已授权/任务要求的项；详细清单见 `CLAUDE_REPORT.md` §5.1。`legacy/results.json` 的 `overall_status=FAIL` 原样保留，原因 `old_regression_tests` 的 G0 设计检查（G0 不应执行旧测试）。

### 4.3 旧单元回归（由 G0 runner 子进程执行）

runner 记录在 `legacy/regression_results.json` 的 `records` 中；每个套件的实际命令均为 `/usr/bin/python3 -B -m unittest discover -s tests -p <pattern> -v`，cwd 为仓库根：

| 套件 | pattern | 结果 |
|---|---|---|
| geometry_continuous | `test_continuous_geometry.py` | PASS 18 |
| geometry_formation | `test_formation_geometry.py` | PASS 15 |
| timing_continuous | `test_continuous_retiming.py` | **FAIL 18/1 error（已知 `KeyError: 'dynamic_peaks'`）** |
| timing_formation | `test_formation_retiming.py` | PASS 14 |
| fr01_uniform_motion | `test_uniform_motion.py` | PASS 13 |
| fr02_a_timing | `test_uniform_timing.py` | PASS 5 |
| fr02_a_trajectory | `test_uniform_trajectory.py` | PASS 8 |
| fr02_b_quality | `test_uniform_quality.py` | PASS 9 |
| fr03_uniform_search | `test_uniform_search.py` | PASS 12 |
| fr04_uniform_pipeline | `test_uniform_pipeline.py` | PASS 19 |

runner 对每个子进程设置了 `FR01_RESULTS_PATH`、`FR02_ARTIFACT_DIR`、`FR02_QUALITY_RESULTS_PATH`、`FR02_TIMING_RESULTS_PATH`、`FR02_TRAJECTORY_RESULTS_PATH`、`FR03_ARTIFACT_DIR` 等输出重定向；具体值见 `legacy/regression_results.json` 的 `environment_delta`。

### 4.4 FR06 2.8 m 无 seed 自主搜索

```bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a
D=evaluation/virtual_formation_a/G5/20261116T112522Z/legacy/fr06_fresh_search_2_8m
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src"
set -o pipefail
python3 -B "$D/run_fresh_search.py" 2>&1 | tee "$D/search.log"
echo "RC=${PIPESTATUS[0]}"
```

入口脚本 `run_fresh_search.py` 只 import 冻结的 `fr06_scene` 和原 `run_experiments._run`，并把 `runner.HERE` 重定向到本目录；不传入参考路径、边、seed 或轨迹。结果：`SUCCESS`，`feasible=true`，`executable=true`，T_end `33.02449442722614 s`，search wall `7.960148098994978 s`，总 wall `9.582085570087656 s`。

### 4.5 旧后端真实 ROS

```bash
# 旧 master（独立端口 11586）
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
mkdir -p "$E/main" "$E/failure" "$E/tamper" "$E/logs" \
         "$E/nodes_main" "$E/nodes_failure" "$E/nodes_tamper"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/logs/ros_main_logs"
roscore -p 11586 > "$E/logs/roscore.log" 2>&1
```

```bash
# main：空地图，四个计划
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
source /opt/ros/noetic/setup.bash
source "$R/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/nodes_main"
cd "$R"
./run_demo.sh --map "$R/evaluation/formation_refactor_v02/FR05/empty_map" \
  --scale 1.0 backend:=uniform_v02 rviz:=false playback_rate:=100 \
  > "$E/logs/launch_main.log" 2>&1
```

```bash
# failure：阻塞地图
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
source /opt/ros/noetic/setup.bash
source "$R/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/nodes_failure"
cd "$R"
./run_demo.sh --map "$R/evaluation/formation_refactor_v02/FR05/blocked_map" \
  --scale 1.0 backend:=uniform_v02 rviz:=false playback_rate:=100 \
  > "$E/logs/launch_failure.log" 2>&1
```

```bash
# tamper：空地图，显式关闭 planner/bridge
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
source /opt/ros/noetic/setup.bash
source "$R/catkin_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/nodes_tamper"
cd "$R"
./run_demo.sh --map "$R/evaluation/formation_refactor_v02/FR05/empty_map" \
  --scale 1.0 backend:=uniform_v02 rviz:=false playback_rate:=100 \
  planner:=false goal_bridge:=false > "$E/logs/launch_tamper.log" 2>&1
```

```bash
# main driver：112 checks
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
cd "$R"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/nodes_main"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
set -o pipefail
python3 -B tests/ros_uniform_playback_smoke.py --scenario main \
  --output "$E/main/main_report.json" \
  2>&1 | tee "$E/logs/driver_main.log" | tail -25
```

```bash
# failure driver：57 checks
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
cd "$R"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/nodes_failure"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
set -o pipefail
python3 -B tests/ros_uniform_playback_smoke.py --scenario failure \
  --output "$E/failure/failure_report.json" \
  2>&1 | tee "$E/logs/driver_failure.log" | tail -25
```

```bash
# tamper driver：14 checks，baseline 为 main 的 plan 600003
R=/home/wq/whut_thesis/expv3_multi_virtual_a
E="$R/evaluation/virtual_formation_a/G5/20261116T112522Z/legacy_ros"
test -s "$E/main/envelopes/envelope_00600003.json" || { echo MISSING; exit 2; }
cd "$R"
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11586
export ROS_LOG_DIR="$E/nodes_tamper"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
set -o pipefail
python3 -B tests/ros_uniform_playback_smoke.py --scenario tamper \
  --baseline-envelope "$E/main/envelopes/envelope_00600003.json" \
  --output "$E/tamper/tamper_report.json" \
  2>&1 | tee "$E/logs/driver_tamper.log" | tail -25
```

旧后端 launch 的 PID：main `3993613/3993614/3993616/3993619`（master/rosout `3992803/3992844`）；failure `3995715`–`3995718`；tamper `3997396/3997397`。

## 5. 进程清理

```bash
# 新后端 launch 与 master
kill -INT <final_run_launch_pid>
kill -TERM <final_run_launch_pid>
kill 3938581 3938606   # 本任务创建的 11585 rosmaster / rosout

# 旧后端 launch 与 11586 master 在 legacy 证据完成后停止
kill -INT <legacy_launch_pid>
kill -TERM <legacy_master_pid>
```

最终核对：

```bash
ps -eo pid,args | grep -E \
  'rosmaster --core -p 11585|rosmaster --core -p 11586|roslaunch .*formation_demo|formation_uniform_planner_node|formation_playback.py|formation_goal_bridge.py|fwis_map_publisher|rviz' \
  | grep -v grep || true
ps -eo pid,args | grep 'rosmaster --core -p 11411' | grep -v grep || true
```

结果写入 `logs/final_process_cleanup.txt`：无剩余任务 ROS 进程；无关的 11411 master（PID 3039925）未触碰。

## 6. Git、保护树与证据清单

```bash
cd "$R"
git status --short > "$E/logs/final_git_status.txt"
git rev-parse HEAD >> "$E/logs/final_git_status.txt"
git branch --show-current >> "$E/logs/final_git_status.txt"
git diff --stat
git status --short -- evaluation/formation_refactor_v02
```

保护树核对（`legacy/protected_before.json` / `protected_after.json`）：

```bash
python3 - <<'PY'
import json
B="evaluation/virtual_formation_a/G5/20261116T112522Z/legacy"
b=json.load(open(B+"/protected_before.json"))
a=json.load(open(B+"/protected_after.json"))
print("protected_file_count_before",b["protected_file_count"])
print("protected_file_count_after",a["protected_file_count"])
print("protected_tree_sha256_before",b["protected_tree_sha256"])
print("protected_tree_sha256_after",a["protected_tree_sha256"])
print("protected_unchanged",b["protected_tree_sha256"]==a["protected_tree_sha256"])
print("missing_protected_paths",a["missing_protected_paths"])
print("unknown_modifications_before",b["unknown_modifications"])
print("unknown_modifications_after",a["unknown_modifications"])
print("design_documents_unchanged",b["design_document_sha256"]==a["design_document_sha256"])
print("authorized_stage_tracked_changes_unchanged",
      b["authorized_stage_tracked_changes"]==a["authorized_stage_tracked_changes"])
PY
```

实际：3183 个保护文件；before/after tree SHA 均为 `de8e609c5cc348d996dbe3dd42a7eaede580b647fda190cf8af61adfa674b8a4`；missing/unknown 均为空；`evaluation/formation_refactor_v02` 当前 2844 个历史文件、`git status` 干净。

最终证据清单（不含清单自身）：

```bash
cd "$E"
find . -type f ! -name 'final_evidence_sha256.txt' ! -name '*.tmp' -print0 \
  | sort -z | xargs -0 sha256sum > /tmp/g5_final_evidence_sha256.txt
mv /tmp/g5_final_evidence_sha256.txt "$E/logs/final_evidence_sha256.txt"
wc -l "$E/logs/final_evidence_sha256.txt"
```

清单路径为证据根相对路径；写入后再核对关键报告/信封 SHA 与 `CLAUDE_REPORT.md` §6.2 一致。

## 7. 保留的失败/被替代命令记录

以下命令或目录曾执行但未作为最终通过证据，均保留：

- `final_main_free`、`final_main_free_2`～`final_main_free_5`：`ros_virtual_a_smoke.py --scene-key S1 --scenario main --goal-policy free`；其中 `final_main_free_3` 记录 pre-PLAYING HOLD 导致的 TF `1.821726266111803e-07` 失败。
- `final_main_straight`、`final_main_straight_2`：`--scene-key S2 --scenario main --goal-policy straight`。
- `final_failure_straight`、`final_failure_straight_2_attempt1_harness_scope`：`--scenario failure`；attempt1 恢复显示曾 `3.0158157137236544e-07`，因 driver 在 playback 仍 HOLD 时开始比较。
- `tamper_session`、`report_tamper_attempt1.json`：`--scenario tamper --baseline-envelope envelope_00600000.json` 超时；最终改用 `envelope_00600001.json`。
- `ros_main_free`、`ros_main_straight`：早期不带 `--capture-dir` 的 S1/S2 运行，保留原始节点日志与报告。

上述失败/替代 run 的存在不影响最终证据，也不被改写为 PASS。