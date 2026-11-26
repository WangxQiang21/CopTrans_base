# V3 联合规划与闭环验收

## 2026-11-26 定向修复

提交 `ffd3135` 和 `0d5777d` 让搜索与证书逐车执行 `allow_reverse=false`。该限制覆盖 Ackermann 和斜行的倒挡，原地旋转仍可通过认证。3 个定向测试全部通过，覆盖单车、三车、正向斜行和原地旋转。[定向结果](../acceptance/hybrid_v3/forward_only_c10)

新版通过一次三车 Gazebo 长途复验。与 `20261125_115519_smXjHv` 使用相同地图、起终点、载荷和速度档，实走 27.79 m，规划时间从 6.06 s 降至 1.74 s，每车切换从 3 次降至 0 次，最大挂点误差从 12.88 mm 降至 0.09 mm。[闭环结果](../runs/20261126_012635_ixqyqR/result.yaml)

按要求停止整套矩阵重跑，回环网络批次停在 36/100 轮。单车、三车和 RViz 交互入口已按要求将修复版设为本地仿真的默认规划器，完整验收仍暂停。曲线斜行的舵角恰好达到 ±90° 时，连续证书仍可能保守拒绝合法轨迹。[边界复现](../acceptance/hybrid_v3/core_quality_c9/signed_oblique_review/README.md)

## 2026-11-26 闭环验收进度

候选 `109943b` 已通过 17/21 项验收。核心和开发集 384/384、冻结留出集 480/480、历史路线 52/52。原生隔离负载台在持续并发规划下完成单车和三车各 605 秒，每车 15,126 个周期，命令提交零超期。[阶段验收清单](../acceptance/hybrid_v3/c9_loopback_in_progress/acceptance.json)

旧 Gazebo 批次归档 85 轮，84 轮符合预期。第 85 轮三车低速辅助模式发生 Wi-Fi 链路超时，Gazebo 物理仍在推进，ROS 时钟和里程计传输中断。控制器均发出零速度指令，停稳状态无法确认。旧数据和失败记录均保留。单机仿真改用 `GZ_IP=127.0.0.1` 后，同一工况独立复验三次全部到达。正式矩阵正在回环网络上重采。 [传输中断诊断](../acceptance/hybrid_v3/incident_c9_network_20261125_232513_98838f/README.md)

## 2026-11-25 长途复测进度

19:46 更新。候选 `70be94d` 在 384 个核心和开发请求里通过 379 个。余下 5 个用例在先尝试曲线斜行时耗尽了搜索预算。候选 `109943b` 调整了简单直线斜行和曲线斜行的顺序，并在保留初始 Ackermann 搜索的前提下给备选验证和模式复查预留时间。这 5 个用例的两个速度档已通过 10/10 次独立认证。整批核心、开发和封存留出集正在依次复验。[核心与开发原始记录](../acceptance/hybrid_v3/release_c8) · [双档独立复测](../acceptance/hybrid_v3/search_budget_c9_repeat)

在 `70be94d` 上，单车和三车各测 605 秒，每车 15,126 个周期，命令提交零超期。三车长途和注入停车试跑均通过时钟记录复核。修复版在两张原始窄通道地图的标准档和低速档完成四次 Gazebo 闭环，实际行驶 29.21–29.32 m，每车零模式切换，最大挂点误差 3.64–11.71 mm。最终版本的完整闭环和默认入口复验仍待通过。[原生负载记录](../acceptance/hybrid_v3/control_605_c8) · [时钟试跑复核](../acceptance/hybrid_v3/clock_c8_pilot_receipts.json) · [两张窄通道的闭环记录](../acceptance/hybrid_v3/gazebo_history_narrow_32e)

18:16 的验收复算通过 **16/21** 项。单车和三车各完成 605 秒原生负载测试，每车采集 15,126 个周期。单车第 5,200 周期晚唤醒 43.13 ms，跟踪计算耗时 0.0036 ms，超过 40 ms 的命令提交期限。三车全程零超期。余下五项为原生控制零超期、Gazebo 时钟、完整闭环、默认入口切换和最终证据复核。[本机逐项验收](../acceptance/hybrid_v3/c7_after_native/acceptance.json)

候选 `70be94d` 在 13 条历史路线的原图、起终点和航向上完成 **52/52** 次盲解和独立连续认证。单车及三车各 26 次，覆盖标准、低速两档，全部使用一个连续 Ackermann 运动段。相同 52 份请求在 C6、`7cdf518`、`da85bef` 上分别有 37、46、46 次达到策略验收要求。`da85bef` 另有两次在搜索预算到期时只交付了安全证书。三车长途 `004851_QZBuhe` 的最终结果采用连续倒车，标准和低速档分别用 2.36 s 和 2.32 s 求解，规划时长为 75.42 s 和 224.18 s。两条窄通道 `132051_mGuofk`、`132257_XKwYLG` 两档也均通过。这些是规划结果；实车体挂点跟踪误差需查相应 Gazebo 闭环轮次。[逐请求同图对照](historical_routes.json)

在旧 ROS1 停车场原图和起终点上，候选 `bf2e5e1` 使用当前货物包络和固定终点朝向，标准档在 2.10 s 内求出 51.38 m 的连续 Ackermann 路线。低速档在 12 s 求解预算内超时。若沿用标准档的几何路径，最远的一辆车要行驶 52.77 m，以低速档上限 0.16 m/s 行驶至少需要 329.8 s，已经超出该任务 300 s 的执行上限。这组数据来自原生求解和独立认证。[本机原图及请求复测](../acceptance/hybrid_v3/map05_ros1_c8/summary.json)

## 运行

在 CopTrans_base 仓库目录运行。每条仿真命令单独使用一个 Gazebo，关闭后再执行下一条。

```bash
./scripts/interactive.sh --planner-backend hybrid_joint
./scripts/test.sh --task single --planner-backend hybrid_joint --experiment short_turn
./scripts/test.sh --task multi --planner-backend hybrid_joint --experiment full_route
./scripts/test.sh --task multi --planner-backend hybrid_joint --scenario v3_multibend --experiment full_route
./scripts/test.sh --task single --planner-backend hybrid_joint --scenario v3_auxiliary --experiment full_route --speed-profile baseline
./scripts/test.sh --task multi --planner-backend legacy --experiment full_route
```

交互窗口使用 RViz 的 **2D Pose Estimate** 和 **2D Goal Pose**。终端和本地 `reports/runs/<实验编号>/experiment.json` 记录实际后端。V3 单车和三车的标识分别为 `single_hybrid_joint_cpp`、`formation_hybrid_joint_cpp`。可用 `./scripts/interactive.sh --dry-run` 查看当前无参入口的配置。

## 求解和执行

单车和三车共享 C++17 搜索、CasADi C++/Ipopt 数值求解和独立连续认证。候选包含连续 Ackermann（A）、斜行（O）、原地旋转（S）以及停稳和舵角准备阶段。搜索从多组几何和方向初值出发，可删除、替换模式子序列，改变连接状态，调整曲线和时长。优先使用时长上限内的纯 A 解。需要辅助模式时，先按连续辅助块数、斜行距离和各车旋转总角度计分，再按无量纲时长、路程、转向变化率、加加速度和障碍物净距排序。权重来自随源码构建的 `fwis_hybrid_joint/config/policy.json`。受限数值搜索无解或超时会单独记录，不能据此推断全局无解。

三车模型使用编队参考位姿和三个固定挂点。每辆车有自己的航向，通过同一分段时间律计算车体及轮端状态。单个 A 阶段可含多个连续曲率曲线段，在段间可以保持运动。认证器按区间细分检查车体、偏置货物、车间碰撞及动态约束。控制接口为车体 Twist。地图、起终点、限制、轨迹系数和共同起跑时刻绑定到权威模型。C++ 协调器在本地重新认证，收齐对应模型的全部控制器预检应答后发许可。

ROS 核心路径为请求 → C++ 规划节点 → 权威模型 → C++ 执行协调器及 C++ 跟踪器 → 各车 cmd_vel → Gazebo。里程计与模式反馈回到跟踪器。Python 用于实验、场景、可视化和离线验收。实时性结论限定在本机原生软实时负载台和静态地图仿真。

## 复算

从仓库根目录执行。重新编译需要 ROS 2 Humble 和可供 CMake 发现的 CasADi C++ SDK。本机已有 `../.coptrans-python/casadi`。其他安装位置可用 `COPTRANS_CASADI_ROOT` 指定。原生求解运行时加载 `libcasadi` 及 Ipopt 插件。下面的独立构建会保留复验所用的文件。

```bash
bash scripts/hybrid_v3/build_snapshot.sh /tmp/coptrans-v3-recheck HEAD
source /opt/ros/humble/setup.bash
source /tmp/coptrans-v3-recheck/ros2_ws/install/setup.bash
python3 scripts/hybrid_v3/scene_benchmark.py --help
python3 scripts/hybrid_v3/benchmark.py --help
```

```bash
python3 scripts/accept_hybrid_v3.py --compact
python3 scripts/accept_hybrid_v3.py
```

第一条命令使用随代码保存的逐次测量摘要。第二条重新读取本机原始试验。两者均对照同一份冻结规范，有缺项或失败时退出码为 1。结果见 `acceptance.json`，每项包含实测值和门槛。`case_manifest.json` 与 `case_manifest_baseline.json` 保存两档每个请求、地图、约束、参考轨迹的摘要。`runs.csv` 保存候选结果、原生后端、源版本、耗时和轨迹指标。原始 ROS 日志、轨迹与控制逐周期采样位于本机 `reports/runs/`、`reports/acceptance/`，仓库忽略这两个目录。

冻结数据由独立构造器产生，验证参考与盲解使用不同的请求操作。第三批冻结留出集每档、每任务有 70 个已知纯 A 可行、40 个其他可行和 10 个构造无效的请求。旧规划器和 V3 的比较按同一请求、地图及速度限制逐条匹配。`v3_multibend` 要求实测参考点依次通过高、低、高三个开口。`v3_auxiliary` 的单车窄槽由侧向位移上界排除纯 A，三车初始挂点约束满秩，起步必须使用辅助模式。

### 从冻结输入重新求解

下面的命令使用新的本地目录保存结果。默认入口的验收结果、原生软件版本和实际二进制摘要见 `manifest.json`。两档速度的留出集使用 cohort 3。每个请求分别发送给参考轨迹认证器和不接收参考轨迹的求解器。

```bash
root=$PWD
raw=$(mktemp -d "$root/reports/acceptance/v3-recheck.XXXXXXXX")
revision=$(python3 -c 'import json; print(json.load(open("reports/hybrid_v3/manifest.json"))["numeric_and_control_sources"]["system_commit"])')
bash scripts/hybrid_v3/build_snapshot.sh "$raw/system" "$revision"
bash scripts/hybrid_v3/full_benchmark.sh "$raw/system" "$raw/batch" "$raw/witness" "$raw/binary" 3
```

要复测编队多弯窄通道的真实闭环，接着运行以下命令。输出的 `run_id` 对应本机 `reports/runs/` 子目录。`--profile baseline` 和 `--profile standard` 分开测试。加 `--headless` 可保留物理仿真和 ROS 通信，仅关闭窗口。

```bash
source /opt/ros/humble/setup.bash
source "$raw/system/ros2_ws/install/setup.bash"
python3 scripts/hybrid_v3/closed_loop.py --task formation --families multibend \
  --profile standard --repetitions 3 --output "$raw/gazebo"
```

### 按历史实验编号复测

在已加载对应系统快照的终端启动一个 Gazebo 和 RViz。以下命令取回第一次手动选点的地图与位姿，用该快照的三车规划器重新求解，并为这次运行分配新编号。

```bash
python3 scripts/hybrid_v3/closed_loop.py --from-run 20261125_115519_smXjHv --task formation \
  --profile standard --repetitions 1 --output "$raw/gazebo_history"
```

地图像素、分辨率、原点、实际选定起终点与原始实验逐项核对。单车用 `--task single`，另一速度档用 `--profile baseline`。原始四个长途闭环来自三车 standard。完整 GUI 矩阵用两个速度档分别运行 `--task both --repetitions 3`，省略 `--families` 即运行四条历史路线、全程路线、多弯窄道、必要辅助和注入停车八组。再用 `--families long --task both --repetitions 1 --headless` 分别运行两个速度档。每组串行执行。

## 实测与已知故障


本次自动验收尚未完成，共 15/21 项通过。机器可读结果见 [acceptance.json](acceptance.json)。

本表的离线求解快照为 `c09357a32cea`，Gazebo 与控制验证使用系统快照 `a3f5c3581715`。原生规划库 SHA 与逐试次身份见 [manifest.json](manifest.json)。

该批验收时，默认单车规划器为 `state_lattice`，三车为 `multi_4wis`，跟踪器为 `cpp`。

| 任务 | 速度档 | 核心可行通过 | 留出可行通过 | 留出纯 A | 旧版同限制可行通过 | 易 / 难规划 P95（秒） |
|---|---|---:|---:|---:|---:|---:|
| 单车 | standard | 33/33 | 110/110 | 70/70 | 16/110 | 0.162 / 0.644 |
| 单车 | baseline | 33/33 | 110/110 | 70/70 | 16/110 | 0.159 / 0.725 |
| 三车 | standard | 33/33 | 110/110 | 70/70 | 73/110 | 0.192 / 0.411 |
| 三车 | baseline | 33/33 | 110/110 | 70/70 | 70/110 | 0.132 / 0.333 |

旧版在完全相同的核心请求上测得的政策合规 P95 如下。每格对应 5 次预热及 30 次正式请求。门槛内成功数不足以观测 P95 时单独标出。

| 任务 | 速度档 | 简单直线（秒） | 困难多弯（秒） |
|---|---|---:|---:|
| 三车 | baseline | 0.063 | 2.864 |
| 三车 | standard | 0.054 | 2.589 |
| 单车 | baseline | 未达到（0/30） | 未达到（0/30） |
| 单车 | standard | 7.985 | 未达到（0/30） |

按冻结轨迹重算了 1288 条原生实验及 44 个场景请求。已知纯 Ackermann 可行的留出任务中，280/280 个请求返回通过独立认证的纯 Ackermann 计划。原生各组共检出 0 次 A 段内部停车。逐组的辅助段位置、曲率跳变和各轮转角峰值见 [behavior.json](behavior.json)。

冗余辅助模式专项已从 40 个预置序列中完整删除 40 个。核心集的纯 A 判定要求内部停车次数为零。必要辅助使用窄槽侧向位移上界和编队挂点满秩反例检验。

当前批次有 3 轮已归档的真实 Gazebo 闭环，其中 2 轮正常完成，0 轮执行了注入停车。正常完成轮次的最大车辆终点误差为 2.28 mm，最大编队挂点误差为 12.42 mm。各场景的全部试次与失败原因保存在逐轮记录中。

该批 3 个 GUI 请求中，2 个实际发布了可供复算的权威模型，2 个为纯 Ackermann。最长连续 Ackermann 段为 43.04 m，各模型内部停车合计 0 次。

### 历史长途复测

直距取实际选定的起终点，实走里程按单车里程计或三车货物参考点的连续采样累加。表内里程为米，耗时为秒，挂点误差为毫米。失败轮保留已走里程。

| 历史编号 | 地图 SHA 前八位 | 起点 → 终点（x, y, yaw） | 直距 | 旧实走 | 旧结果 |
|---|---|---|---:|---:|---|
| 20261125_115519_smXjHv | 6cc440ef | (3.866, 5.650, 0.000) → (22.893, 18.211, -0.019) | 22.80 | 29.94 | passed · all_trackers_complete |
| 20261125_120655_6xtU9x | f5d20632 | (3.866, 5.650, 0.000) → (22.893, 18.211, -0.019) | 22.80 | 30.66 | passed · all_trackers_complete |
| 20261125_004431_1tfhhn | 6cc440ef | (2.429, 3.706, 0.502) → (13.438, 21.892, 3.131) | 21.26 | 31.31 | failed · formation_tracking_or_clearance_error |
| 20261125_015146_OS7Yaz | 6224ae2b | (2.913, 8.183, -0.011) → (32.268, 28.381, 0.065) | 35.63 | 0.15 | cancelled · launch_shutdown_before_completion |

下表逐次列出同图、同起终点、同载荷几何、同物理参数和同速度档的三车运行。旧计划三列切换数按三辆车的 CDR 采样模式分别统计，新计划按共同运动阶段统计。实测切换按各车已记录的模式反馈统计，采样间隔以内的瞬时切换可能漏计。

| 历史编号 | 闭环试次 | 结果 | 实走里程 | 规划耗时 | 计划时长 | 最大挂点误差 | 计划切换 | 实测切换（各车） |
|---|---|---|---:|---:|---:|---:|---:|---:|
| 20261125_115519_smXjHv | 历史三车 standard | passed | 29.94 | 6.060 | 83.87 | 12.88 | 3/3/3 | 3/3/3 |
| 20261125_120655_6xtU9x | 历史三车 standard | passed | 30.66 | 7.337 | 114.98 | 7.34 | 0/0/0 | 0/0/0 |
| 20261125_120655_6xtU9x | 20261125_173635_b7f539 | passed · all_trackers_complete | 29.12 | 10.074 | 70.76 | 0.05 | 0 | 0/0/0 |
| 20261125_004431_1tfhhn | 历史三车 standard | failed | 31.31 | 1.356 | 92.72 | 246.71 | 3/3/3 | 3/3/3 |
| 20261125_004431_1tfhhn | 20261125_173803_14ad19 | failed · NO_CERTIFIED_SOLUTION_WITHIN_BUDGET A_SOLVE_TIMEOUT | 0.00 | 12.401 | 待测 | 0.00 | 待测 | 0/0/0 |
| 20261125_015146_OS7Yaz | 历史三车 standard | cancelled | 0.15 | 6.871 | 127.84 | 1.39 | 8/8/8 | 4/4/4 |
| 20261125_015146_OS7Yaz | 20261125_173822_017394 | passed · all_trackers_complete | 43.04 | 4.518 | 109.72 | 12.42 | 0 | 0/0/0 |

冻结历史离线请求的独立审计已记录 52 次，结果为 CERTIFIED_POLICY_COMPLIANT 37 次、NO_CERTIFIED_SOLUTION_WITHIN_BUDGET 15 次。单车和 baseline 为新增复测，旧归档只提供三车 standard 原始对照。逐条 SHA、状态和失败原因见 [measurements.json](measurements.json)。

保留此前 C6 的 38 个已归档 GUI 试次，其中 32 次正常到达。长途验收另计 96 个 GUI 试次及 4 个无界面试次。

正常 Gazebo 试次的分阶段墙钟 P95，单位为毫秒。构图、求解和认证按每次请求内的全部候选累积，编辑可能嵌在候选处理内。各列分别取分位数。

| 任务及速度档 | 样本 | 初值 | 结构编辑 | NLP 构图 | NLP 求解 | 独立认证 | 模型编码 | 显示轨迹采样 | DDS 发布调用 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 三车 standard | 2 | 429.09 | 0.04 | 2616.84 | 7081.20 | 98.82 | 0.27 | 0.73 | 0.46 |

真实 GUI 下共 13536 个车次跟踪周期，最大回调 0.584 ms。按消息值和时间作唯一匹配的 13345 个命令中，最慢从控制节点提交到 Gazebo 消费用了 13.855 ms。各次跟踪的实时因子为 0.998–0.999。

实际闭环还测得 2 个从权威模型打包到整组准入的样本，P95 53.2 ms、最大 53.2 ms。该数据包含模型大小与 ROS 调度的影响，单独于固定短路径基准报告。

短路径 DDS 准入测试每组 5 次预热后正式测 30 次。三车 baseline 的 P95 为 47.4 ms。三车 standard 的 P95 为 51.3 ms。单车 baseline 的 P95 为 48.1 ms。单车 standard 的 P95 为 52.1 ms。

待通过门项为 CONTROLLER_COMPUTE (NOT_RUN)、NATIVE_CONTROL_DEADLINES (NOT_RUN)、GAZEBO_CLOCK_SCOPE (FAIL)、CLOSED_LOOP (FAIL)、DEFAULT_INTEGRATION (FAIL)、REPRODUCIBLE_EVIDENCE (FAIL)。

开发期间真实复现过启动时两个节点的仿真时钟错序、接收端尚未收到首个时钟、三车 S→A 的末时刻浮点残差、DDS 发送回调长尾。失败试次在本机原始目录保留。旧版单车用采样 Twist 反推运动模式有歧义，表中旧版列仅统计同请求下的认证成功数。TSan 在本机进入测试主函数前因内存映射失败，ASan、UBSan 以及主动执行求解器的测试结果单独列出。硬件状态为 `HARDWARE_NOT_VERIFIED`。
