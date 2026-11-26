# CopTrans 测试方法

在终端进入仓库。下面的命令都从这个目录执行。

```bash
cd /home/wq/Whut/CopTrans_base
```

## 1. 跑默认 CTS 测试集

一次跑完 7 条离线用例。离线测试只运行规划和轨迹认证，无需启动 Gazebo 或 RViz。

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

终端最后会打印 `Offline results` 和 `summary.json` 的路径。每条用例的 `status` 为 `passed` 表示通过，`reason` 可查结果原因，`transport_s` 是运输求解耗时，`transport_duration_s` 是计划行驶时长。进入运输求解后，规划结果和认证记录保存在同目录的 `用例编号/transport.json.gz`。

## 2. 在 Gazebo 和 RViz 跑指定用例

把 `CTS_S3` 换成上表中任一编号。脚本打开 Gazebo 和 RViz，载入用例的固定起终点，自动执行三车集结、运输规划和闭环跟踪。

```bash
./scripts/test.sh --task multi --case CTS_S3
```

需要后台运行闭环时，加 `--headless` 关闭 Gazebo 和 RViz 的图形窗口。

```bash
./scripts/test.sh --task multi --case CTS_S3 --headless
```

每次启动一条闭环，等当前进程退出后再跑下一条。闭环结果在终端 `Run directory` 指向的目录中，打开 `result.yaml` 看结果、失败原因、规划耗时和最大挂点误差。默认自动分配挂点。加 `--dock-assignment 1,3,2` 可让 `robot1`、`robot2`、`robot3` 分别使用 1、3、2 号挂点。

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

## 6. 首次配置环境

新机器先准备 ROS 2 Humble 和 Colcon。闭环测试还需要 Gazebo Harmonic、对应的 ROS 桥和 RViz。缺少 CasADi C++ SDK 时可执行 `python3 -m pip install --user --no-deps casadi==3.7.2`。已有独立 SDK 时，先设置 `COPTRANS_CASADI_ROOT` 为实际的 SDK 目录。首次运行脚本会编译 Release。
