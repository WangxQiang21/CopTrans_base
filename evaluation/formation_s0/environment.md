# S0 单车基线环境记录

- 记录日期：2026-11-10
- 工程目录：`/home/wq/whut_thesis/expv3_multi`
- 原始分支：`continuous-mode-path`
- 已核实算法基线 SHA：`956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee`
- 基线提交标题：`Unify vehicle execution and add human-labeled ROS data collection`
- S0 新建分支：`baseline/single-v2`，创建时指向上述基线 SHA
- 算法后端：`mode_path_v2`（由 [run_demo.sh](../../run_demo.sh) 和 [params.yaml](../../catkin_ws/src/fwis_v3_demo/config/params.yaml) 核实）

## 工作区已有文件

切换到 S0 分支前，Git 显示两个已有未跟踪文件：

1. [CURRENT_REPOSITORY_STATUS.md](../../CURRENT_REPOSITORY_STATUS.md)
2. [多车4WIS刚性编队协同规划_详细技术实施文档.md](../../多车4WIS刚性编队协同规划_详细技术实施文档.md)

本阶段未清理、覆盖或提交这两个用户文件，未执行 `git reset --hard`。

## 操作系统与硬件

| 项目 | 本次命令实测值 |
|---|---|
| 主机/内核 | `Linux wq-ubuntu 5.15.0-139-generic #149~20.04.1-Ubuntu SMP x86_64` |
| CPU | AMD Ryzen 7 7840H，8 核 16 线程 |
| 内存 | 总计 30 GiB；检查时可用 7.7 GiB |
| Swap | 总计 2.0 GiB；检查时可用 562 MiB |
| 工程所在文件系统可用空间 | 109 GiB |

资源数值是检查时快照，不是测试期间峰值。完整 Python 回归最大 RSS 为 231408 KiB；catkin 构建最大 RSS 为 586040 KiB；C++ 测试目标构建及运行最大 RSS 为 707528 KiB。

## Python 与数值依赖

以下结果来自工程目录内、未使用 `-I` 隔离用户 site-packages 的实际 Python 环境：

| 组件 | 实测版本/状态 |
|---|---|
| Python | 3.8.10，可执行文件 `/usr/bin/python3` |
| NumPy | 1.24.4 |
| SciPy | 1.10.1 |
| CasADi | 3.7.2 |
| PyYAML | 5.3.1 |
| CasADi IPOPT plugin | `casadi.has_nlpsol("ipopt") == True` |

注意：使用 `python3 -I` 会隐藏本环境的用户级 SciPy/CasADi，不能据此判断项目实际依赖缺失。

## ROS 与构建工具

| 组件 | 实测状态 |
|---|---|
| ROS | `/opt/ros/noetic/setup.bash` 存在 |
| `ROS_DISTRO` | `noetic` |
| `ROS_VERSION` | `1` |
| `rosversion -d` | `noetic` |
| ROS launch 显示的 `rosversion` | 1.17.4 |
| `catkin_make` | `/opt/ros/noetic/bin/catkin_make` |
| `python3-catkin-pkg` | 1.0.0-100 |
| `python3-rospkg` | 1.6.0-100 |

单独执行 `catkin_make --version` 会把 `/home/wq/whut_thesis` 当作工作区并报告缺少 `src`；该调用没有构建本工程。实际构建命令显式使用 `catkin_make -C /home/wq/whut_thesis/expv3_multi/catkin_ws -j1`，退出码为 0。

## 固定 ROS 案例配置

| 项目 | 固定值 |
|---|---|
| 地图 | [map04-p12-maze-env1](../../maps/map04-p12-maze-env1/) |
| 启动方式 | `bash run_demo.sh --map map04 rviz:=false` |
| 地图比例 | 0.5，围绕地图中心缩放，栅格不重采样 |
| 原始栅格分辨率 | 0.011272 m |
| 原始地图原点 | `[1.9352, 1.9352, 0.0]` |
| 原始场景起点 | `(5.423884, 16.718428, 0.0)` |
| 原始场景目标 | `(13.179, 14.0244, 0.0)` |
| **实际缩放后起点** | `(7.7121, 13.359372, 0.0)` |
| **实际缩放后目标** | `(11.589658, 12.012358, 0.0)` |
| 位姿含义 | 单车中心；位置单位 m，航向单位 rad |
| 后端 | `mode_path_v2` |
| 最终目标审核 | 0.02 m / 1°，由实际质量报告 `selected.goal_tolerance` 核实 |
| RViz | 显式关闭 |
| ROS Master | `http://127.0.0.1:11410`，与受保护端口隔离 |

原始场景位姿来自 [scene_meta.yaml](../../maps/map04-p12-maze-env1/scene_meta.yaml)，实际缩放后位姿由本次前端日志中的 `dp_x/dp_y` 和自动目标日志核实。两套坐标不得混用。

## 实际验证结果索引

| 项目 | 结果 | 证据 |
|---|---|---|
| Python 完整回归 | 151 项，150 通过、1 错误；退出码 1 | [unit_tests.txt](logs/unit_tests.txt) |
| catkin 构建 | 7 个包构建成功；退出码 0 | [catkin_build.txt](logs/catkin_build.txt) |
| C++ 前端测试 | 6/6 通过；退出码 0 | [cpp_planner_tests.txt](logs/cpp_planner_tests.txt) |
| map04 一键 Demo | `FEASIBLE`，几何/动态/时间/显示扫掠通过 | [map04_demo.txt](logs/map04_demo.txt)、[map04_summary.json](map04_summary.json) |
| ROS 清理 | 本次节点已停止，11410 端口已释放 | [S0_REPORT.md](S0_REPORT.md) |

约 1.8 MB 的完整原始质量报告保存在仓库外的 [实验归档](../../../expv3_experiment_archive/formation_s0_956bc64/map04_quality_report.yaml)，仓库内保留紧凑摘要，避免将大规模原始轨迹产物提交进源码仓库。

本记录只证明所列命令和固定案例的本次结果，不代表全地图通过率、严格实时能力或实车安全性。
