# FR06 — commands and environment

All commands were run from the worktree root
`/home/wq/whut_thesis/expv3_multi_v02` unless a `cd` is shown.
No command touched the original repository, the FR01/FR05 evidence or any
Codex acceptance directory. No Git commit was made.

## Environment

`evaluation/formation_refactor_v02/FR06/env.sh`:

```
export ROS_MASTER_URI=http://127.0.0.1:11483
export ROS_HOSTNAME=127.0.0.1
export ROS_IP=127.0.0.1
export ROS_HOME=/home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR06/ros_home
export OPENBLAS_NUM_THREADS=1
export PYTHONUNBUFFERED=1
export PYTHONDONTWRITEBYTECODE=1
export DISPLAY=:1
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/setup.bash
```

Ports 11311, 11312 and 11320 were not used. Only 11483 was opened for this
round, and it was closed afterwards.

## 1. Freeze the scene maps

```
source evaluation/formation_refactor_v02/FR06/env.sh
OPENBLAS_NUM_THREADS=1 python3 \
  evaluation/formation_refactor_v02/FR06/make_scene.py \
  | tee evaluation/formation_refactor_v02/FR06/logs/make_scene.log
```

Writes `maps/corridor_w{2.6,2.8,3.2}/` and `scene_frozen.json`. Re-running
leaves all six map files byte-identical (`sha256sum` compared).

## 2. Freeze and FR02-audit the reference

```
source evaluation/formation_refactor_v02/FR06/env.sh
OPENBLAS_NUM_THREADS=1 python3 \
  evaluation/formation_refactor_v02/FR06/freeze_reference.py \
  | tee evaluation/formation_refactor_v02/FR06/logs/freeze_reference.log
```

Writes `reference/reference_model.json`, `reference_sample_projection.json`,
`audit_{wide,tight,exploratory}.json`, `reference_summary.json`. The script
exits non-zero if the wide or tight reference audit is not feasible.

## 3. Six autonomous attempts plus the unacceptable goal

```
source evaluation/formation_refactor_v02/FR06/env.sh
OPENBLAS_NUM_THREADS=1 python3 \
  evaluation/formation_refactor_v02/FR06/run_experiments.py \
  | tee evaluation/formation_refactor_v02/FR06/logs/run_experiments.log
```

Writes `runs/*.json` (all six attempts plus `runs/collision_goal.json`),
`plans/tight_2.8m_attempt{1,2,3}_plan620{1,2,3}.json` and
`experiment_summary.json`. The tight runs succeed; the wide runs end with
`BUDGET_EXHAUSTED`, which is stored as-is.

## 4. Real ROS/RViz replay (tight 2.8 m)

```
cd /home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR06
source env.sh
roscore -p 11483 > logs/roscore.log 2>&1 &
echo $! > logs/roscore.pid
nohup ./run_demo.sh --map "$PWD/maps/corridor_w2.8" --scale 1.0 \
  backend:=uniform_v02 rviz:=true playback_rate:=100 \
  > logs/run_demo_rviz.log 2>&1 &
echo $! > logs/demo.pid
```

`run_demo.sh` is the repository's demo runner at the worktree root; the
`formation_demo.launch` defaults it starts are listed in the log. The
`scene_meta.yaml` of the 2.8 m map supplies the auto goal: start (6, 6, 0)
and goal (10, 10, 1.571). One plan (600000) was published and replayed;
state transitions and the model digest are in `logs/run_demo_rviz.log`
and in the node logs under `ros_home/log/`.

Screenshots (DISPLAY=:1):

```
mkdir -p rviz/round1
for i in $(seq -w 1 30); do
  import -window root "rviz/round1/auto_${i}.png"
  sleep 0.5
done
```

Frame crops: `convert rviz/round1/auto_XX.png -crop 2010x1011+330+75 +repage rviz/round1/frames/auto_XX.png`.
Metadata: `rviz/round1/run_meta.json`.

Shutdown (performed after capture):

```
kill -INT <roslaunch pid>      # nodes stopped
kill -INT <roscore pid>        # private master 11483 stopped
pgrep -af "rosmaster --core -p 11483|roslaunch fwis_v3_demo"  # empty
```

## 5. Manifest check

```
cd /home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR06
sha256sum -c SOURCE_MANIFEST.sha256
```

Passing output is recorded in `logs/manifest_check.log`.

## Recorded environment

* host: Linux-5.15.0-139-generic x86_64, AMD Ryzen 7 7840H
* Python 3.8.10, ROS Noetic
* experiment config sha256:
  `3020fe9a3a08383839cd824a19e3fb64dc56383467e5b1473dcb201b44c26229`
* reference model sha256:
  `1da16fa1bdbe93e1bd3e55855b8fd900b6c4770f66a7ccbe8ae7eb368a5affde`
* RViz replay plan digest:
  `a39818b8125ce7d3cd212e16108f7e91b849c952f534dfcf3b559e4bac0cc967`