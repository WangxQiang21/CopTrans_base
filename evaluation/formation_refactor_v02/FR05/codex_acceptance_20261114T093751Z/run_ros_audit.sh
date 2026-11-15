#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash
source /home/wq/whut_thesis/expv3_multi_v02/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11461 ROS_HOSTNAME=127.0.0.1 ROS_IP=127.0.0.1
export ROS_HOME="/home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR05/codex_acceptance_20261114T093751Z/ros_home" OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
mkdir -p "$ROS_HOME"
python3 "/home/wq/whut_thesis/expv3_multi_v02/evaluation/formation_refactor_v02/FR05/codex_acceptance_20261114T093751Z/run_ros_audit.py"
