# Phase 6 三车走廊与共同时间轴

`--multi-convex-corridor` 启用带货凸区域图引导，`--multi-reachable-clock` 启用共同 jerk 受限时间轴。两项功能可以单独启用，并使用同一套编队参数和联合认证。标准仿真档保持 0.50 m/s 每车线速度、0.36 m/s² 每车线加速度和 0.90 m 的规划最小转弯半径。

## 走廊与联合搜索

- 为货物航向和三车车身航向组合构造保守的编队配置栅格。车体、偏置货物、障碍、未知格和地图边界均参与。自由格按连续行段合并成矩形凸区。相邻区域共享有正宽度的边，正角点接触视为不连通。
- 图搜索先选择门户序列。门户折线可在所选凸区并集内缩短，每条连线用线段裁剪核对整个区间的覆盖。候选方向需允许三车同时采用对应的 A 或 O 运动。原门户折线和快捷折线均重建为联合运动边，再按包含停稳、转向和运动的代价排序。
- 搜索保持编队位姿、三车独立车身航向、齿向、控制和轮姿。在不同航向图层之间，既有共享转轴 A 弧和定点 S 准备边参与连接。每条边执行车体与货物扫掠检查，终点候选用独立的连续几何和动力学审计复核。走廊候选失败时用剩余预算运行 `uniform_v02` 原搜索。
- RViz 的 `/formation/convex_corridor_guide` 显示搜索使用的凸区和途经点。完整和独立认证的执行轨迹继续通过 `/formation/playback_plan` 发布。

[凸区域和门户](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_convex_corridor.py) · [三车联合搜索](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_search.py) · [独立审计和回退](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_pipeline.py)

## 共同时间轴与准入

原几何先通过独立 FR02 审计。新时间轴在每个 A、O 或 S 运动相位计算三车平动和航向随进度的导数、轮速系数和侧向加速度约束。共同进度求解器相交全部限值，经可达速度上界和 jerk 受限的七段时间律生成候选。弧线还计入向心加速度及随速度变化的三阶导数。各相位用解析上界复核所有车辆和从静止到静止的边界，B 段沿用原停车和确认时间。只有确实缩短且通过复核的相位才采用新时间律。

候选沿相同连续几何、相同方向和相同端点顺序单调扫过每条运动边，沿用绑定地图、车辆和偏置货物的原连续碰撞证明。发布 `uniform_trajectory.reachable_jerk/v1` 时同时携带原计划和按相位排序的时间参数。发布端、Playback 和执行路由各自重建模型，校验计划 ID、版本、SHA-256 及共享投影。执行端重采样至相位对齐的控制时间网格，等待三车预检和共同许可。

[共同进度求解](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_progress_clock.py) · [时间律认证与版本模型](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_reachability.py) · [模型接收复验](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/formation_playback.py) · [执行路由](../../ros2_ws/src/fwis_bringup/fwis_bringup/formation_route.py)

## 固定几何的共同时间参数对照

相同原路径和标准仿真限值，每个样本预热 5 次、测量 100 次。参考时长改善比例的中位数为 31.4%，五个案例的解析动力学上界最高使用配置限值的 99.53%。生成时间含新模型及投影，P95 为 2.69–5.50 ms，P99 为 2.88–5.79 ms。 [逐次数据](../../reports/architecture_optimization/formation_clock_ab.json)

| 纯 A 路径 | 原参考时长（s） | 共同 jerk 时长（s） | 缩短 |
| --- | ---: | ---: | ---: |
| 直线 1 m | 4.025 | 3.579 | 11.1% |
| 直线 4 m | 15.075 | 9.609 | 36.3% |
| 直线 8 m | 30.150 | 17.649 | 41.5% |
| 左转共享中心弧 | 11.358 | 8.174 | 28.0% |
| 右转共享中心弧 | 12.727 | 8.737 | 31.4% |

## 走廊对照

相同速度档、输入地图和货物参数下，六张生成地图均得到认证解。`portal_detour` 的两组走同一条路线。`staggered_portals` 预热 5 次后，短接走廊 30/30 可执行，原路线在 30/30 次试验中用完 40 次扩展预算。封闭墙的两组都拒绝发布。室内临界通道 30/30 得到认证解，首证 P50 为 0.682 → 0.578 s，P95 为 0.717 → 0.612 s。货物路线从 45.59 m 降至 29.36 m，模式切换从 5 次降至 2 次，参考时长从 582.04 s 降至 385.53 s。`too_narrow` 变体绕行时切换数从 5 次增加到 8 次，参考时长缩短 10.0%。 [九场景对照](../../reports/architecture_optimization/convex_corridor_final_ab.json) · [临界重复试验](../../reports/architecture_optimization/convex_corridor_final_critical.json) · [交错窄门重复试验](../../reports/architecture_optimization/convex_corridor_final_staggered.json)

## Gazebo 与 RViz 闭环

| 室内任务 | 原标准档参考时长 → 新参考时长 | 货物最小采样净距 | 最大装载支点偏差 | 编队终点误差 |
| --- | ---: | ---: | ---: | ---: |
| 短弯，共同时钟 | 14.466 → 12.324 仿真秒 | 0.559 m | 0.96 mm | 19.41 mm / 0.329° |
| 全程，凸走廊与共同时钟 | 139.827 → 87.257 仿真秒 | 0.125 m | 12.70 mm | 16.29 mm / 0.515° |

两轮均由 RViz Playback 重建并接受最新模型。三台原生跟踪器使用同一共同启动时间，通过预检，在 0 s 起步偏差下完成，并在终点输出零 Twist。全程从请求到首条内部认证 0.554 s，到收到轨迹信封 1.271 s，到复核并构造控制路线 1.697 s，到共同许可 1.799 s。 [闭环证据与分段时序](../../reports/architecture_optimization/formation_clock_closed_loop.json)

最终安装版额外通过三车带货直行，参考时长 9.207 仿真秒，货物最小采样净距 0.250 m，最大支点偏差 0.224 mm，共同起步偏差 0 s。 [直行最终复验](../../reports/architecture_optimization/formation_clock_straight_final.json)

## 复现

```bash
cd /home/wq/Whut/CopTrans_base
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 reports/architecture_optimization/bench_formation_clock.py --output /tmp/formation-clock.json
python3 reports/architecture_optimization/bench_convex_corridor.py --output /tmp/corridor-matrix.json
python3 reports/architecture_optimization/bench_convex_corridor.py \
  --case factory_indoor_01_critical --pairs 30 --warmups 5 --output /tmp/corridor-critical.json

./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment short_turn \
  --architecture-profile optimized --multi-native-frontier --multi-reachable-clock \
  --tracker-implementation cpp --no-build
./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment full_route \
  --architecture-profile optimized --multi-native-frontier --multi-convex-corridor \
  --multi-reachable-clock --tracker-implementation cpp --no-build
```

撤掉两个实验参数可恢复原联合搜索和时间律。旧速度档继续用 `--speed-profile baseline`，可用于同条件算法比较。
