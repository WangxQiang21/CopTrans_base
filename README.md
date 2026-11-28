# CopTrans 测试方法

在终端进入仓库。下面的命令都从这个目录执行。

```bash
cd /home/wq/Whut/CopTrans_base
```

## 1. 跑默认 CTS 测试集

**11 月 27 日论文场景**共 12 条离线用例。每张地图各有三个起点、一个共同终点。正反向交换完整货物中心位姿。下面的命令使用当前 C++ 后端，依次运行集结、运输规划和轨迹认证。

```bash
./scripts/test.sh --planner-only --suite paper
./scripts/test.sh --planner-only --case INDOOR_S1_E --case YARD_E_S2
```

室内用例以 `INDOOR_` 开头，厂外装卸场用例以 `YARD_` 开头。后缀取 `S1_E`、`S2_E`、`S3_E`、`E_S1`、`E_S2` 或 `E_S3`。用例编号会自动选中对应地图。每个点的坐标和航向见 [论文场景起终点](ros2_ws/src/fwis_bringup/config/cts_paper_routes.yaml)。

无参数运行原有的 7 条离线回归用例。两个测试集都可在关闭 Gazebo 和 RViz 的情况下运行。

```bash
./scripts/test.sh --planner-only
```

用 `--case` 选一条。用例编号区分大小写，`--planner-only` 放在第一个参数。

```bash
./scripts/test.sh --planner-only --case CTS_S3
```

一条命令测多条时，重复写 `--case`。

```bash
./scripts/test.sh --planner-only --case CTS_S3 --case CTS_E_TO_S3
```

| 用例编号 | 离线测试内容 |
| --- | --- |
| `CTS_S1` | 三车集结规划和 CTS_S1 到 CTS_E 的运输规划 |
| `CTS_S2` | 三车集结规划和 CTS_S2 到 CTS_E 的运输规划 |
| `CTS_S3` | 三车集结规划和 CTS_S3 到 CTS_E 的运输规划 |
| `CTS_S4` | 三车集结规划和 CTS_S4 到 CTS_E 的运输规划 |
| `CTS_E_TO_S3` | 三车集结规划和 CTS_E 到 CTS_S3 的反向路线 |
| `20261126_161917_u046w8` | 用归档的挂点和实测停靠位姿复测运输规划 |
| `20261126_162144_qqdYea` | 用归档的挂点和实测停靠位姿复测运输规划 |

每条用例的货物起点、终点和航向见 [CTS 用例配置](ros2_ws/src/fwis_bringup/config/cts_long_routes.yaml)。只想从已挂载状态测试运输规划时，加 `--transport-only`。

终端最后会打印 `Offline results` 和 `summary.json` 的路径。每条用例的 `status` 为 `passed` 表示通过，`reason` 可查结果原因，`transport_s` 是运输求解耗时，`transport_duration_s` 是计划行驶时长。查看 `assembly_cruise` 可逐车核对集结模式。查看 `transport_cruise` 中的 `switches`、`enroute_spins` 和 `longest_a_m`，可分别核对运输模式切换次数、途中 Spin 段数和最长连续 Ackermann 路程。切换次数含终点对齐，同模式换挡单独体现于原始相位的 `gear`。进入运输求解后，规划结果和认证记录保存在同目录的 `用例编号/transport.json.gz`。

## 2. 在 Gazebo 和 RViz 跑指定用例

把 `CTS_S3` 换成上表或论文场景中的任一编号，例如 `YARD_S1_E`。脚本打开 Gazebo 和 RViz，载入该用例的地图和固定起终点，自动执行三车集结、运输规划和闭环跟踪。

```bash
./scripts/test.sh --task multi --case CTS_S3
```

需要后台运行闭环时，加 `--headless` 关闭 Gazebo 和 RViz 的图形窗口。

```bash
./scripts/test.sh --task multi --case CTS_S3 --headless
```

每次启动一条闭环，等当前进程退出后再跑下一条。闭环结果在终端 `Run directory` 指向的目录中，打开 `result.yaml` 看结果、失败原因、规划耗时、逐车集结时序和最大挂点误差。默认按认证可行路径的里程自动分配挂点。加 `--dock-assignment 1,3,2` 可让 `robot1`、`robot2`、`robot3` 分别使用 1、3、2 号挂点。RViz 中可勾选或关闭「货物包络轨迹」，查看规划路径上货物的外廓和朝向。

默认使用当前 V3 C++ 联合规划器，闭环使用 C++ 跟踪器。首次测试或更新代码后直接运行上述命令。已构建且源码一致时可加 `--no-build`。如果提示安装过期或校验失败，去掉 `--no-build` 再运行。

## 3. 在 RViz 手动选择起点和终点

```bash
./scripts/interactive.sh
```

`./scripts/test.sh --task multi` 和无参数的 `./run_demo.sh` 也会进入交互测试。可用 `./scripts/test.sh --list-scenarios` 查看场景，加 `--scenario port_01` 切换到港口。

1. 等「规划与执行」面板提示选择起点。点击 **2D Pose Estimate**，在地图上按住左键拖出货物中心的起点和朝向。
2. 点击 **2D Goal Pose**，拖出货物中心的终点和朝向。两点选齐后，三车开始集结，全部到位停稳后进入 CTS 运输。
3. 在面板查看结果。按 **Ctrl+C** 关闭，再运行脚本开始下一轮。

## 4. 查日志和复跑

终端和 RViz 会显示实验编号，交互模式的「规划与执行」面板可以复制编号。日志保存在本地 `reports/runs/实验编号/`。`experiment.json` 记录起终点、结果和轨迹文件位置，`environment.txt` 记录源码及安装指纹。反馈问题时附上实验编号和现象。

```bash
id=20261126_161917_u046w8  # 换成本机已有的实验编号
cat "reports/runs/$id/experiment.json"
```

使用原实验归档的规划器和参数复现。

```bash
./scripts/test.sh --task multi --replay "$id"
```

使用当前默认规划器复测同一张地图及原起终点和朝向。

```bash
./scripts/test.sh --task multi --retest "$id"
```

## 5. 其他固定用例

```bash
./scripts/test.sh --task single --experiment full_route
./scripts/test.sh --task multi --experiment full_route
```

`short_turn` 测短弯，`short_straight` 测直行，`full_route` 测预设全程。单车可省略 `--experiment full_route`。加 `--scene-variant critical` 测临界窄通道，加 `--scene-variant too_narrow` 测绕行。

回退到旧规划器和 Python 跟踪器。

```bash
./scripts/test.sh --task multi --experiment full_route --planner-backend legacy --architecture-profile baseline --tracker-implementation python
```

## 6. 看双车风机叶片场景

从仓库根目录选一条运行。Gazebo 打开后，两辆 Ranger 和 15.375 m 的刚性叶片静置在起点。按 **Ctrl+C** 关闭当前场景后再换下一张。

```bash
./scripts/test.sh --scene-only blade_mountain_01
./scripts/test.sh --scene-only blade_right_angle_01
./scripts/test.sh --scene-only blade_roundabout_01
```

山区可看 S 弯和回头弯。直角街巷由东行、北行和西行三段组成，两处直角之间相隔 28 m。环岛的任务路线从西进入，经南侧和东侧后向北驶出。在街巷的两个弯中查看叶片与两车的挂点可运行下列命令。

```bash
./scripts/test.sh --scene-only blade_right_angle_01 --placement key_pose --view supports
./scripts/test.sh --scene-only blade_right_angle_01 --placement second_bend --view supports
```

`--placement` 可选 `start`、`key_pose`、`goal`。直角街巷还可选 `second_bend`。`--view` 可选 `overview`（全景）、`top`（垂直俯视）、`detail`（运输位姿）和 `supports`（低机位看挂点）。这个入口用于静态场景检查。双车闭环仍待规划器接入。

只生成地图和世界时加 `--generate-only`。产物在 `ros2_ws/generated/scenes/<场景名>/`，其中 `map.yaml` 为 0.05 m 栅格，`map_0025.yaml` 为 0.025 m 栅格。街巷地图因新增第二个直角而更新。可用下面的命令复核环岛与第一版的文件哈希、占用像素和 YAML 元数据。

```bash
./scripts/test.sh --scene-only blade_roundabout_01 --generate-only
baseline='/home/wq/Whut/design/双车叶片_V1原始地图基线_直角与环岛/v1_map_baseline'
(cd "$baseline" && sha256sum -c SHA256SUMS.txt)
python3 scripts/compare_blade_maps.py "$baseline" ros2_ws/generated/scenes --scene blade_roundabout_01
```

## 7. 跑双车叶片离线规划和 Gazebo 闭环

```bash
./scripts/test.sh --blade-plan
```

这条命令在离线状态下按三张地图的预设起终点求解。只测一张时，在 `--blade-plan` 后加地图名，例如 `blade_roundabout_01`。规划器优先考虑两辆车都能连续走的 Ackermann 轨迹，统一从货物位姿推导两个中心挂点，再为两车各自计算航向、速度和角速度。环岛额外检查南侧和东侧的有序通行区。

收窄后的三张地图用 `./scripts/test.sh --blade-plan --hard`。弓形街巷两处直角净宽 6.15 m，环岛环道 5.6 m、进口 6.0 m，山区主路 5.8 m 并缩紧回头弯和回程 S 弯。三张加严地图沿用原尺寸叶片和原起终点。可以再加 `--ackermann-only` 单独对照 Ackermann 搜索结果。

终端打印实验目录 `reports/runs/<实验编号>/`。每张地图的子目录包含 `offline.json`（求解时间、模式、扫掠净距和挂点验收）、`plan.npz`（轨迹及速度）和 `planned_footprint.png`（俯视包络图）。

离线通过后，把终端打印的实验目录和地图名拼成完整路径，传给 `--blade-run`。加 `--rviz` 同时显示 Gazebo 和 RViz，沿用原三车实验的规划与执行面板、状态卡、误差曲线和货物包络。只看 Gazebo 时用 `--gui`。

```bash
trial='/home/wq/Whut/CopTrans_base/reports/runs/<实验编号>/blade_roundabout_hard'
./scripts/test.sh --blade-run "$trial" --rviz
```

一次运行一张地图。每次执行会在对应子目录下生成 `gazebo_*/result.json`（闭环结论、扫掠净距与误差）、`tracking.npz`（Gazebo 实测位姿、指令与参考值）及日志。闭环从已挂载状态起步，控制器根据 Gazebo 回读位姿分别给两辆车发 Twist。仿真用理想平面约束维持挂点间距，速度投影残差反映两车指令与刚性约束的相容性。

### 对比窄弯中的可倒车 Ackermann 与多模式

从仓库根目录运行。以下命令沿用各地图的原起终点。山区保留道路中心线，只在指定弯段沿原路平顺收窄。局部重规划后对两车和刚性叶片做全程扫掠检查。

```bash
./scripts/test.sh --blade-bends street --width 6.10 --spacing .63 --reserve .075 --tries A R.35+R.63 O.52
./scripts/test.sh --blade-bends mountain --bend return --width 3.80 --spacing .43 --initial clearance --guide-map grid --reserve .017 --tries A R.36+R.64 O.38
./scripts/test.sh --blade-bends mountain --bend hairpin --width 4.90 --spacing .57 --reserve .023 --tries A R.33+R.65 O.46
```

`--width` 指弯中收窄后的目标净宽，单位为米。`A` 在每个求解区间均允许前进和倒车。`R.35+R.63` 额外尝试在弯段进度约 35% 和 63% 处倒车。`O` 和 `S` 分别尝试斜移和原地调整车头。数字是离线求解中候选动作所在的区间，实际切换坐标由求解确定。

终端打印本地实验目录。`comparison.json` 记录通过的候选，按模式切换、换向优先以及按车程优先分别列出结果。`results.json` 记录切换、两车实际换向、最小净距和求解时间。地图在 `maps.tar.gz`。将下例中的目录换成回程 S 弯那次的输出，即可画出两条轨迹及包络。

```bash
trial='reports/runs/<实验编号>'
python3 scripts/blade_bends/plot.py "$trial/A.npz" "$trial/O.38.npz" "$trial/comparison.png"
```

## 8. 首次配置环境

新机器先准备 ROS 2 Humble 和 Colcon。闭环测试还需要 Gazebo Harmonic、对应的 ROS 桥和 RViz。缺少 CasADi C++ SDK 时可执行 `python3 -m pip install --user --no-deps casadi==3.7.2`。已有独立 SDK 时，先设置 `COPTRANS_CASADI_ROOT` 为实际的 SDK 目录。首次运行脚本会编译 Release。

双车场景生成使用 Shapely 2.1.2，安装命令为 `python3 -m pip install --user 'shapely==2.1.2'`。双车规划和包络图使用 SciPy、Matplotlib。地图对照脚本使用 Pillow 读取 PGM。
