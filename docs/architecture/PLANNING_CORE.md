# 共享 C++ 规划核与三车搜索

## Phase 2 共享内核

`fwis_planning_core` 是 C++17 数学和计算库，导出 `fwis_planning_core::fwis_planning_core`。库内部使用 double，固定坐标系位置和尺寸用 m，方向用 rad，局部轨迹采样时间用 s。库可使用独立 CMake 构建。ROS 消息转换由调用方负责。

| 接口 | 参数和输出 |
| --- | --- |
| `geometry.hpp` | 解析 A/O/S 常控制原语推进、角度 wrap、货物参考位姿到车体支点的刚体变换。A/O 的 `amount` 单位为 m，可带倒车符号。S 用 rad，车位保持不动。O 保持原姿态角。A 曲率分支阈值为 1e-10 1/m。 |
| `trajectory.hpp` | 局部五次时间律和解析单原语采样，输出时间、固定位姿和车体坐标系 Twist。采样在闭区间内，时间律的输入是无量纲的归一化时间。 |
| `collision.hpp` | 行优先、轴对齐的不可变占据栅格，非零值含未知占据。固定编队的车体与货物包络通过 SAT 检查，保留边界接触、安全余量、失败见证和统计顺序。解析扫掠离散化用每段角点位移上界计算采样区间与外扩量。上界由权威模型推导后提供。 |
| `fwis_planning_core_native` | `advance_batch`、`quintic_batch`、`trajectory_samples`、`CollisionKernel.check_batch` 和 `sweep_discretization`。NumPy 批量计算在 C++ 循环内完成，计算期间释放 GIL，调用方保持数组只读。 |

`GridSnapshot` 保存原点、分辨率、占据及调用方提供的版本/摘要。`CollisionKernel` 构造时拥有独立的占据行索引、车体尺寸和货物包络，查询可并发。静态查询先检测货物与栅格，再检测车辆与栅格，最后检测车体两两重叠。车辆顺序决定失败报告中的索引。

`uniform_v02` 的 Python 解析轨迹、连续扫掠和最终独立认证继续执行。`_formation_collision_native.CollisionKernel` 保持原构造和返回接口，内部调用共享库的 SAT。pybind11 未安装时，现有 Python 碰撞参考实现仍可用。原 OBB 算术次序和闭合边界保留。`-ffp-contract=off` 保持接触判断次序，`geometry.cpp` 使用独立 sin/cos 调用，因为编译器合并为 sincos 后，近直行曲率阈值处的相消误差最高约 1.1 µm。

### 验证与性能

- GoogleTest 覆盖 9 个案例，包括 A/O/S 正反向、`nextafter` 接触边界、未知栅格、货物与车体顺序、两端无碰撞而内部碰撞、静止支点 Spin 扫掠、扫掠预算和输入异常。
- 100,000 个随机 A/O/S 推进结果在本次 x86-64/GCC 环境逐分量精确一致，包括 1e-10 曲率阈值两侧。五次时间律差分绝对误差阈值 1e-14，定时位姿与 Twist 阈值 1e-12。
- 100,000 个三车联合静态 OBB 样本覆盖有货和无货、三档外扩、正负地图原点、旋转车体、障碍和未知栅格。报告分类、碰撞车辆索引、查询计数一致，漏检 0。线段分析扫掠额外对照首个失败采样。
- ASan 和 UBSan 下核心 9 个测试通过。沙箱采用 ptrace，LeakSanitizer 禁用。
- CPU 12 固定亲和力、powersave governor、Release 构建。5 次预热后 30 次交错配对，20,000 条同输入状态推进的墙钟中位数由 **33.196 ms** 降至 **1.072 ms**，下降 **96.8%**。P95 从 36.808 ms 降至 1.140 ms，同批输出精确相等。该结果衡量批量推进和数组生成。当前完整规划耗时另测。

[固定输入的 Python/C++ 原始样本](../../reports/architecture_planning_core/batch_parity_latency.json) · [Google Benchmark 原始样本](../../reports/architecture_planning_core/google_benchmark.json)

## Phase 3 三车 C++ 搜索内核

`fwis_multi_planner_cpp` 提供 FR03 候选和代价，以及 FR04 的有限标签键、代表表、优先队列、运行预算和认证回调桥接。键覆盖 XY、编队航向、模式、各车的模式与轮控标签、车身航向与行进方向、当前目标下的 A 能力集合。平分点按 Python ties-to-even 规则量化。相同键仅严格更低代价替代，出队顺序遵循 rank、g 和入队序号。被替换的条目出队时判为过期。

候选库在 C++ 生成八个编队坐标系 O 方向、目标方向的 A/O 平移，以及配置的共享转动中心和固定/目标截断 A 弧。C++ 处理各车正倒分支、弧线最小半径、当前航向对齐、S 目标去重和自环剔除。直连 A 弧按最大及总旋转量对车体方向组合稳定排序。代理代价仍由各车轮角的静止准备上界和最慢车辆的平移/转动代价组成。

直线、O 平移、共享中心弧和 S 的搜索扫掠都在 C++ 沿精确几何采样并执行联合 SAT。原保守采样数量、起点检测、外扩量和早失败顺序保留。失败样本的站位、时间、姿态和车辆归属由同一解析边恢复。保留逐点 trace 和旧碰撞后端。C++ 预算按取消、扩展上限和墙钟截止顺序检查，通过回调执行独立终点认证并核对完成时刻。

规划适配器继续持有解析状态、父边、FR01 边工厂和整条路径复审。连接器拓扑选择和高层搜索编排仍由 `uniform_v02` 的 Python 层负责。

统一测试入口的 `optimized` 配置默认加载 `fwis_multi_frontier_native` 并报告 `frontier_backend=cpp`。用 `--no-multi-native-frontier` 单独回退搜索前沿，或用 `--architecture-profile baseline` 回退整套规划配置。缺少原生绑定时会给出诊断。

### 验证与性能

- 7 个 C++ 用例在 Release 和 ASan/UBSan 下通过，覆盖平分点、gear 与 A 能力键、稳定优先级、过期条目，以及搜索预算、候选分支和代价。
- 合计 35 项 pytest 通过。覆盖 2,000 组状态键、原始候选次序和拒绝原因、代价及解析边等价、A/O/S 首个联合碰撞样本、五个固定规划场景、取消/预算、无终点验证器和终点审计被拒绝或过期。固定测试还覆盖地图贯通墙的不可行请求。
- 使用同一 Python SAT 回退时，2,000 次固定入队及出队的合成任务中位数从 **17.483 ms** 降至 **4.864 ms**，而带货 A 路线的完整搜索约 502 ms，说明只迁移前沿影响有限。
- 启用相同 native SAT 的配对基准执行了 5 次预热和 30 次正式测量。固定带货 A 路线触发两次格点扩展，完整搜索中位数从 **446.292 ms** 降至 **205.680 ms**（**2.17 倍**），规划至认证总墙钟中位数从 **549.113 ms** 降至 **309.160 ms**。两组模型摘要和认证结果一致。此夹具达到 1.5 倍搜索目标。ROS 发布时延和广泛场景的尾延迟由系统级测试独立验收。
- 最新全原语 C++ 基准同样使用 5 次预热、30 次固定配对和相同 SAT。完整搜索中位数由 **275.563 ms 降至 106.183 ms**，达到 **2.60 倍**。首条认证 P95 由 **297.422 ms 降至 115.905 ms**。规划与独立终审的中位数由 **341.452 ms 降至 173.245 ms**。两次格点扩展、最终模型摘要和认证结果一致。这组离线测量以包含实际格点扩展的带货 A 路线为输入。
- 使用相同生成栅格、场景配置和实际货物矩形执行八项离线对照。室内短弯、直线、室内/港口/室外长路线及 indoor 的 wide、critical、too_narrow 变体均得到相同的摘要、模式序列和规划结果。too_narrow 标记特定瓶颈的宽度，整张图还有可行线路。

[全原语 C++ 的 30 组配对样本](../../reports/architecture_planning_core/phase3_full_successor_latency.json) · [八项生成地图的最新等价对照](../../reports/architecture_planning_core/phase3_full_successor_scene_parity.json) · [前沿对照](../../reports/architecture_planning_core/frontier_parity_latency.json) · [加速 S 扫掠阶段的对照](../../reports/architecture_planning_core/frontier_spin_parity_latency.json)

## 构建与复测

从仓库根目录运行。Google Benchmark 的构建使用 CMake 可解析的 `benchmark` 包。

```bash
cmake -S ros2_ws/src/fwis_planning_core -B /tmp/fwis-core -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/tmp/fwis-core-install
cmake --build /tmp/fwis-core --parallel 2
ctest --test-dir /tmp/fwis-core --output-on-failure
cmake --install /tmp/fwis-core

cmake -S ros2_ws/src/fwis_multi_planner_cpp -B /tmp/fwis-frontier -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/tmp/fwis-core-install
cmake --build /tmp/fwis-frontier --parallel 2
ctest --test-dir /tmp/fwis-frontier --output-on-failure

export PYTHONPATH="/tmp/fwis-frontier:/tmp/fwis-core:$PWD/ros2_ws/src/fwis_backends"
python3 -m pytest -q ros2_ws/src/fwis_planning_core/test/test_python_differential.py \
  ros2_ws/src/fwis_multi_planner_cpp/test/test_native_frontier.py
python3 ros2_ws/src/fwis_planning_core/benchmark/compare_batch.py --output /tmp/fwis-batch.json
python3 ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_frontiers.py --output /tmp/fwis-frontier.json
```

在已重新构建当前 ROS 2 工作区的新 shell 中，使用安装后的两个 C++ 扩展复现完整搜索和地图对照。

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python3 ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_frontiers.py --output /tmp/fwis-native-search.json
python3 ros2_ws/src/fwis_multi_planner_cpp/benchmark/compare_scene_matrix.py --output /tmp/fwis-native-scenes.json
```

```bash
cmake -S ros2_ws/src/fwis_planning_core -B /tmp/fwis-core-san -DCMAKE_BUILD_TYPE=Debug \
  -DFWIS_PLANNING_CORE_PYTHON=OFF -DFWIS_PLANNING_CORE_SANITIZERS=ON
cmake --build /tmp/fwis-core-san --parallel 2
ASAN_OPTIONS=detect_leaks=0 UBSAN_OPTIONS=halt_on_error=1 \
  ctest --test-dir /tmp/fwis-core-san --output-on-failure
```
