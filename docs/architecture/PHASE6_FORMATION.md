# 三车走廊与共同时间轴验收

三车规划保持编队刚性挂点、每辆车独立航向和 A／O／S 模式，货物以带偏置矩形参与每条边的碰撞检查。`--multi-convex-corridor` 启用矩形走廊引导，`--multi-reachable-clock` 启用共同 jerk 受限时间轴。两个开关可独立复测。

## 算法口径

| 项目 | 论文中的方法 | CopTrans 中运行的方法 |
| --- | --- | --- |
| GCS 与路径选择 | 图的顶点带凸集合，边代价是端点连续变量的凸函数。混合整数凸优化同时选择边和连续变量，松弛给出目标下界。轨迹版本也把区域内的 Bézier 曲线参数纳入问题。 | 每个固定航向层先把带车体和货物膨胀的空闲栅格合并为矩形。用正宽门户建立邻接关系，以矩形中心距离做 A*，再在已选矩形并集内用线段裁剪短接。引导结果按联合运动边的代价排序。验收对象是有界预算内生成的候选和独立证书。 |
| TOPP-RA 与固定路径计时 | 将固定路径的二阶约束写成进度加速度和速度平方的线性条件。在离散路径站点用小型线性规划作向后可控集传播和向前提速。 | 标量原型先用区间导数上界得到速度上限和加速度区间，以闭式递推计算可刹停包络。执行时间轴在既有运动相位上使用七段、静止到静止的 jerk 律，并逐车认证。它比较相同路径的参考时长和限值利用率。 |

论文依据包括 GCS 最短路、曲线 GCS 和 TOPP-RA 的原作者论文。[Marcucci 等，GCS 最短路](https://doi.org/10.1137/22M1523790) · [Marcucci 等，曲线 GCS](https://arxiv.org/abs/2205.04422) · [Pham 与 Pham，TOPP-RA](https://arxiv.org/abs/1707.07239)

走廊把未知格和地图边界按占据处理。车辆航向、齿向、转向准备、轮速和共同运动约束由 `uniform_v02` 应用。搜索可用共享转轴 A 弧、O 平移和定点 S 准备连接不同航向层。候选需通过各车与偏置货物的整段扫掠及终点连续几何、动力学审计。走廊预算耗尽或候选认证失败时，联合搜索使用剩余预算。预算耗尽记录为本次搜索未找到可执行解。RViz 的 `/formation/convex_corridor_guide` 用于查看引导，受独立认证的执行模型通过 `/formation/playback_plan` 发布。[凸区域门户](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_convex_corridor.py) · [联合搜索](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_search.py) · [终点审计和回退](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_pipeline.py)

共同时间轴每个运动相位沿原几何、同方向和同端点顺序单调推进。直线进度用米，圆弧进度可用弧度。三辆车的线速度、角速度、轮速、侧向加速度和 jerk 共同约束进度。只采用更短且通过连续解析上界复核的候选时间律，B 相位继续保留停车和确认时间。长 C² Ackermann 段的候选按整段上界缩放，窄弯的转向较紧时会同时降低该段直行速度。单车的分段速度平方原型用于研究对照，其加速度接缝与已认证的 jerk 执行律有不同的约束。[标量递推](../../ros2_ws/src/fwis_backends/fwis_backends/path_reachability.py) · [单车原型和七段律](../../ros2_ws/src/fwis_backends/fwis_backends/single_continuous_mode_path/ackermann_reachability.py) · [三车相位认证](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_reachability.py) · [C² 长段计时](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/uniform_curve_clock.py)

发布 `uniform_trajectory.reachable_jerk/v1` 时携带原计划及替换相位的参数。发布端、Playback 和执行路由分别校验计划 ID、模型版本、SHA-256 和共享投影。执行路由在相位对齐的控制网格重采样，等待三车预检和共同许可。[Playback 复验](../../ros2_ws/src/fwis_backends/fwis_backends/multi_4wis/formation_playback.py) · [执行路由](../../ros2_ws/src/fwis_bringup/fwis_bringup/formation_route.py)

## 固定几何计时对照

五个纯 A 案例保持路径、地图、货物和运动限值一致。每项预热 5 次，实测 100 次。参考时长缩短中位数 **31.4%**，各案例的连续动力学认证最大使用配置限值的 **99.53%**。计时包含新模型及投影，P95 为 3.81–6.77 ms，P99 为 3.85–6.91 ms。原始样本、参数、源码 SHA 和工作树补丁在本地 `reports/acceptance/20261125_132106_algorithm_clock100/`。

| 纯 A 路径 | 原参考时长（s） | 共同 jerk 时长（s） | 缩短 |
| --- | ---: | ---: | ---: |
| 直线 1 m | 4.025 | 3.579 | 11.1% |
| 直线 4 m | 15.075 | 9.609 | 36.3% |
| 直线 8 m | 30.150 | 17.649 | 41.5% |
| 左转共享中心弧 | 11.358 | 8.174 | 28.0% |
| 右转共享中心弧 | 12.727 | 8.737 | 31.4% |

## 窄通道验收

离线对照固定地图、起终点、三车和货物尺寸、原生 frontier 以及扩展预算，只切换凸走廊。对照为 0.16 m/s 规划线速度档，各场景预热 5 对、实测 30 对。Gazebo 闭环使用 0.50 m/s 标准档。原始离线样本和输入指纹在本地 `reports/acceptance/20261125_132054_algorithm_corridor30/`。

| 离线场景 | 原搜索／凸走廊可执行次数 | 同预算下观察 |
| --- | ---: | --- |
| `factory_indoor_01/wide` | 30／30 | 3.60 m 口宽。首证 P95 0.501 → 0.529 s。 |
| `factory_indoor_01/critical` | 30／30 | 3.00 m 口宽。首证 P95 0.517 → 0.423 s，模式切换 5 → 2。 |
| `factory_indoor_01/too_narrow` | 30／30 | 2.60 m 口宽侵入 2.70 m 货物的原参考姿态。首证 P95 0.474 → 0.579 s，模式切换 5 → 8。 |
| `portal_detour` | 30／30 | 两组得到同一路线，首证 P95 0.159 → 0.161 s。 |
| `staggered_portals` | 0／30 | 两个交错窄门。原搜索用完 40 次扩展预算，凸走廊首证 P95 0.272 s。 |
| `closed_wall` | 0／0 | 两组在给定预算内均无可执行轨迹。 |

Gazebo 验收核对 `result.yaml` 和 `timing_trace.jsonl` 中的独立执行证书、地图一致、三车共同生效时刻、实际三车及货物最小净空为正、车辆和编队终点误差在 0.10 m／0.10 rad 内、最大挂点误差在 0.10 m 内、终点输出零 Twist。控制时序由单独的控制验收使用墙钟核对。连续曲率和绕行结论同时查看归档的 `trajectories/` 与 `metrics/motion.jsonl.gz`。[闭环代码](../../ros2_ws/src/fwis_bringup/scripts/run_experiment)

两次室内闭环的起终点、货物和控制参数相同。`critical` 的货物中心在柜体中心线 `x=12.1 m` 处以 `y≈5.54 m` 穿过通道。`too_narrow` 在同一截面以 `y≈20.08 m` 从北侧绕过。两次都匹配生成地图，起步偏差为 0 s，三车全部完成并输出零 Twist。

| Gazebo 复测 | 室内变体 | 规划到下发墙钟（s） | 仿真行驶（s） | 货物／车辆最小采样净空（m） | 最大挂点误差（mm） | 编队终点误差 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| `20261125_132051_mGuofk` | `critical` | 8.694 | 89.692 | 0.101／0.211 | 4.68 | 16.06 mm／0.507° |
| `20261125_132257_XKwYLG` | `too_narrow` | 9.266 | 134.124 | 0.112／0.220 | 10.48 | 15.98 mm／0.512° |

默认地图上另有单车过西侧限宽口 `20261125_120341_hdrAoM` 和三车窄口出口附近连续转弯 `20261125_120655_6xtU9x` 两次已通过的本地归档。室内实跑验证闭环安全与跟踪，离线交错窄门对照验证凸走廊对搜索结果的独立贡献。

## 复现

先构建工作区，再在同一终端依次运行。脚本提供 Gazebo、RViz 和本地实验归档。

```bash
cd /home/wq/Whut/CopTrans_base
(
  set -e
  for layout in critical too_narrow; do
    ./scripts/test.sh --task multi --scenario factory_indoor_01 \
      --scene-variant "$layout" --experiment full_route \
      --architecture-profile optimized --multi-native-frontier \
      --multi-convex-corridor --multi-reachable-clock \
      --tracker-implementation cpp --no-build
  done
)
```

离线证据走同一个归档入口。不同重复试验各使用新的本地目录。

```bash
cd /home/wq/Whut/CopTrans_base
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1
python3 scripts/record_acceptance.py \
  --output "reports/acceptance/$(date +%Y%m%d_%H%M%S)_algorithm_corridor30" \
  --input ros2_ws/src/ranger_mini_gz/scenarios/factory_indoor_01.yaml \
  --input ros2_ws/src/ranger_mini_gz/config/cargo_geometry.yaml \
  --input ros2_ws/src/fwis_bringup/config/experiments.yaml -- bash -c '
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash &&
    python3 reports/architecture_optimization/bench_convex_corridor.py --pairs 30 --warmups 5 \
      --case factory_indoor_01_wide --case factory_indoor_01_critical \
      --case factory_indoor_01_too_narrow --case portal_detour --case staggered_portals \
      --case closed_wall --output "$COPTRANS_EVIDENCE_DIR/corridor_30.json"'
python3 scripts/record_acceptance.py \
  --output "reports/acceptance/$(date +%Y%m%d_%H%M%S)_algorithm_clock100" \
  --input ros2_ws/src/fwis_bringup/config/test_defaults.yaml \
  --input ros2_ws/src/fwis_backends/multi_4wis/config/transport_formation.yaml -- bash -c '
    source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash &&
    python3 reports/architecture_optimization/bench_formation_clock.py --samples 100 --warmups 5 \
      --output "$COPTRANS_EVIDENCE_DIR/formation_clock_100.json"'
```

`--scene-variant` 适用于新实验。`--replay` 使用归档中的原始布局和地图指纹。测试比较显式开关时使用相同场景、速度档和跟踪器。
