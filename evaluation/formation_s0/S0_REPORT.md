# S0 阶段审查报告：单车基线冻结

## 结论

**S0 的基线记录与冻结任务已完成，但当前单车基线不是“全测试通过”。**本次真实结果为：Python 150/151 通过、1 项错误；C++ 6/6 通过；catkin 构建成功；固定 map04 案例发布 `FEASIBLE` 轨迹并通过其质量审核。未进入 S1，未修改任何规划算法。

## 任务编号

S0：冻结单车基线和运行证据，不修改规划算法。

## Git 基线

- 算法基线 SHA：`956bc64ead93f38e8f5632738ceb0f5e6cb7f4ee`
- 原分支：`continuous-mode-path`
- S0 分支：`baseline/single-v2`
- 本阶段提交：仅包含 `evaluation/formation_s0/` 下的基线证据；最终 SHA 以提交后的 `git rev-parse HEAD` 和阶段交付回复为准
- 远程推送：未执行

## 已查看源码和配置

| 路径 | 核实内容 |
|---|---|
| [CURRENT_REPOSITORY_STATUS.md](../../CURRENT_REPOSITORY_STATUS.md) | 单车架构、测试入口、配置层次和多车扩展边界 |
| [README.md](../../README.md) | 当前分支后端、测试命令、地图和启动参数 |
| [run_demo.sh](../../run_demo.sh) | 固定 `mode_path_v2`、map04 默认比例 0.5、catkin 构建及 roslaunch 调用 |
| [demo.launch](../../catkin_ws/src/fwis_v3_demo/launch/demo.launch) | planner、optimizer、playback、地图和自动目标节点组合 |
| [params.yaml](../../catkin_ws/src/fwis_v3_demo/config/params.yaml) | 搜索预算 450000、基础后端预算 8 s、扫掠配置和方向策略 |
| [ranger_mini_2.yaml](../../catkin_ws/src/fwis_common/config/ranger_mini_2.yaml) | 0.738 × 0.500 m 车体、0.810 m 最小半径和共享动态 profile |
| [scene_meta.yaml](../../maps/map04-p12-maze-env1/scene_meta.yaml) | 原始场景起终点 |
| [map.yaml](../../maps/map04-p12-maze-env1/map.yaml) | 地图分辨率、原点和阈值 |
| [test_core.py](../../tests/test_core.py) | 测试从真实源码路径导入后端模块 |
| [test_continuous_retiming.py:263-272](../../tests/test_continuous_retiming.py#L263-L272) | 本次 Python 基线错误所在断言 |
| [fwis_v3_planner/CMakeLists.txt](../../catkin_ws/src/fwis_v3_planner/CMakeLists.txt) | C++ `test_planner_policy_history` 测试目标 |
| [run_expv3_probe.py](../../tools/run_expv3_probe.py)、[ros_map_probe.py](../../tests/ros_map_probe.py) | 项目已有的隔离 ROS、独立检查和清理机制，用于核对运行合同 |
| [.gitignore](../../.gitignore) | build/devel、原始实验输出和日志忽略规则 |

本阶段没有修改上述源码、消息、YAML、launch 或测试。

## 修改文件

仅新增本阶段记录和证据：

1. [environment.md](environment.md)：环境、依赖和固定案例坐标合同。
2. [baseline_cases.csv](baseline_cases.csv)：固定案例的实际指标。
3. [map04_summary.json](map04_summary.json)：从实际质量报告提取的紧凑指标摘要。
4. [logs/unit_tests.txt](logs/unit_tests.txt)：完整 Python 回归及资源统计。
5. [logs/catkin_build.txt](logs/catkin_build.txt)：catkin 构建日志。
6. [logs/cpp_planner_tests.txt](logs/cpp_planner_tests.txt)：C++ 测试构建与运行日志。
7. [logs/map04_demo.txt](logs/map04_demo.txt)：一键 Demo、实际规划结果及关闭日志。
8. [S0_REPORT.md](S0_REPORT.md)：本审查报告。

完整原始质量报告约 1.8 MB，已移至仓库外的 [实验归档](../../../expv3_experiment_archive/formation_s0_956bc64/map04_quality_report.yaml)。未新增多车算法代码，未进入 S1。

## 接口变化

无。单车消息、话题、后端、预算、审核阈值和启动脚本均未改变。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 151 项，150 通过、1 错误；测试计时 7.605 s，墙钟约 7.94 s | 1 |
| `source /opt/ros/noetic/setup.bash; catkin_make -C /home/wq/whut_thesis/expv3_multi/catkin_ws -j1` | 7 个 ROS 包构建成功，墙钟约 16.85 s | 0 |
| `catkin_make -C /home/wq/whut_thesis/expv3_multi/catkin_ws -j1 run_tests_fwis_v3_planner` | 2 个测试套件、6 项 C++ 测试全部通过 | 0 |
| `ROS_MASTER_URI=http://127.0.0.1:11410 ROS_IP=127.0.0.1 bash run_demo.sh --map map04 rviz:=false` | `plan_id=1`，前端搜索成功，后端 `FEASIBLE` | 长驻进程，记录后主动停止 |
| `rostopic echo -n 1 /plan/quality_report` | 收到同一 `plan_id=1` 的完整 `FEASIBLE` 报告 | 0 |
| 对本次实际 `roslaunch` PID 发送 `SIGINT` | 节点关闭；随后无匹配 ROS 进程，11410 无监听 | 0 |
| `git diff --check` | 已执行，无输出 | 0 |

Python 命令和 catkin 命令通过 `/usr/bin/time -v` 记录资源，完整输出见对应日志。Demo 外层曾使用 180 s 保护性 timeout；规划结论只取自实际 ROS 状态和质量报告，不取 timeout 或包装进程退出码。

### Python 基线错误

唯一错误为：

```text
test_short_translation_has_finite_acceleration_and_zero_end_speed
KeyError: 'dynamic_peaks'
tests/test_continuous_retiming.py:271
Ran 151 tests in 7.605s
FAILED (errors=1)
```

该结果表明测试访问的首个 phase 没有 `dynamic_peaks` 字段。本阶段只冻结事实，未将其擅自称作算法缺陷或测试缺陷，也未修改源码/测试使其通过。后续若修复，应作为独立任务评估，不能混入多车 S1。

## map04 固定案例结果

实际缩放后起点为 `(7.7121, 13.359372, 0)`，目标为 `(11.589658, 12.012358, 0)`；不是原始 [scene_meta.yaml](../../maps/map04-p12-maze-env1/scene_meta.yaml) 中未经缩放的坐标。

| 指标 | 本次结果 |
|---|---:|
| 状态 | `FEASIBLE` |
| 前端搜索耗时 | 2.068315 s |
| 后端 `ModePathMaster` 总耗时 | 8.246575552 s |
| 优化器回调墙钟 | 9.773910589 s |
| 前端 + 优化器回调（派生值） | 11.842225589 s |
| 最终执行时长 | 40.746479533 s |
| 前端动作/换模数 | 135 / 14 |
| 最终轨迹换模数 | 8 |
| 最终路径长度 | 24.621388143 m |
| 最终倒车长度 | 0 m |
| 最小物理车体净空 | 0.023401429 m |
| 最小安全尺寸净空 | 0.003401429 m |
| 终点位置误差 | 0.001613806 m |
| 终点航向误差 | 6.43e-16 rad |
| 几何 / 动态 / 时间审核 | PASS / PASS / PASS |
| 实际显示采样扫掠 | `execution_display_sweep_clear=true` |
| 最终获选来源 | `combined_spin_A_connections` |
| NLP 调用次数 | 0 |

派生的 11.842225589 s 是前端 `seconds=2.068315` 与质量报告 `optimizer_callback_wall_s=9.773910589` 之和，不包含进程启动，也不是一次独立端到端单调时钟测量。基础后端预算为 8 s，但高换模种子触发 15 s 自适应扩展，实际预算是 23 s；报告为 `budget_exceeded=false`。因此不能将本案例描述为严格 8 s 硬实时规划。

## 未执行项目

1. 未运行全部地图或全部 ROS smoke/probe；只有固定 map04 的一次结果，不能推断全地图成功率或耗时分位数。
2. 未启动 RViz 人工交互，未做控制器、NMPC、执行器、实车或闭环状态反馈测试。
3. 未运行多车测试；本阶段没有多车实现。
4. 未修复已记录的 Python 基线错误，避免超出 S0 范围。
5. 未推送远程分支。

## 安全性审计

本阶段未改变任何安全或运动学行为。map04 的实际报告满足当前单车几何、动态、时间和显示采样合同；但其 `hardware_certified=false`，默认 `mode_switch_only` profile 下部分转向及平移派生需求仅作诊断，不能据此宣称实车全动态安全。固定刚性队形、多车 OBB、逐车模式、共同时间轴和 FormationEvaluator 尚未实现。

## 存在风险

1. 当前基线存在一项可复现的 Python 回归错误；后续阶段必须同时运行同一回归，区分既有失败与新增失败。
2. map04 仅运行一次，性能值受本机负载、依赖和搜索预算影响，不是统计性能保证。
3. 最小安全尺寸净空约 3.4 mm，后续修改不能通过放大扫掠步长或减小 margin 来伪造改善。
4. 原始场景坐标和实际缩放后坐标不同，后续案例配置必须明确使用哪一套。

## 完成判据与下阶段前置条件

- S0 记录与冻结：**完成**；测试失败已如实保留，未伪称全绿。
- 单车算法源码保持基线：**确认**。
- 下一阶段：**未开始**。只有用户另行指定“执行 S1”后才实施 `FormationModel` 与输入校验。
- S1 开始前应保留本阶段同一组回归命令，并将上述 Python 错误标记为既有基线失败，不得为多车功能擅自放宽单车审核合同。
