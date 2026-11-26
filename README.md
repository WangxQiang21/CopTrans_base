# CopTrans 仿真使用方法

## 1. 启动交互仿真

```bash
cd CopTrans_base
./scripts/interactive.sh
```

`./scripts/test.sh --task multi` 和无参数的 `./run_demo.sh` 也进入同一交互测试。脚本会增量构建并打开 Gazebo 和 RViz，等待你选定起点和终点。使用 `--no-build` 时，脚本会核对安装与源码是否一致。默认是室内厂房，西侧靠墙有一处只允许单车通过的 1.10 m 限宽口。可用 `--list-scenarios` 查看场景，加 `--scenario port_01` 切换到港口。

新机器先准备 ROS 2 Humble、Gazebo Harmonic、对应的 ROS 桥、RViz 和 Colcon。若缺少 CasADi C++ SDK，可执行 `python3 -m pip install --user --no-deps casadi==3.7.2`。已有独立 SDK 时，可先执行 `export COPTRANS_CASADI_ROOT=/实际路径/casadi`。首次启动会编译 Release；旧安装或文件校验失败时，去掉 `--no-build` 再运行。

单车、三车和 RViz 交互仿真默认使用 V3 C++ 联合规划器和 C++ 跟踪器。终端启动时显示所用的规划器、跟踪器和版本。接收计划后，`experiment.json` 的 `planner_source` 为 `hybrid_joint`。源码及安装指纹可在同目录的 `environment.txt` 查询。

## 2. 在 RViz 设置起点和终点

1. 等待左侧「规划与执行」面板提示选择起点。点击 **2D Pose Estimate**，在地图上按住左键并拖出朝向。Gazebo 会把三辆车和货物放到选定位置。
2. 等面板确认三车已停稳，点击 **2D Goal Pose**，在地图上拖出终点和朝向。松开鼠标后自动规划，三车通过预检后同步执行。两次点选设置的都是整个编队的位姿。
3. 在「规划与执行」面板查看结果。完成或失败后窗口会保留。按 **Ctrl+C** 退出，再运行脚本开始下一轮。

## 3. 查日志或复跑

RViz 右下角和终端都会显示实验编号。交互模式也可在「规划与执行」面板点 **复制编号**。反馈问题时附上编号和看到的现象。每轮日志自动保存在本地 `reports/runs/实验编号/`。

```bash
id=20261125_000000_aB1cDe  # 换成面板上的实验编号
cat "reports/runs/$id/experiment.json"
./scripts/interactive.sh --replay "$id" --no-build
```

`experiment.json` 可查起终点、结果、失败原因和轨迹文件位置。`--replay` 使用归档的规划器和参数，适合复现旧问题。`--retest` 用当前默认规划器复跑同一张地图和同一组 RViz 起终点及朝向，新日志的 `retest_of` 会记录原实验编号。

```bash
./scripts/test.sh --task multi --retest 20261125_115519_smXjHv
```

## 4. 测试三车集结和 CTS 运输

```bash
./scripts/interactive.sh --scene-variant cts_assembly
```

用 RViz 点选货物中心的起点和终点。三车在两点选定后从 S1–S3 出发，全部到位并停稳后进入运输。默认自动分配挂点。加 `--dock-assignment 1,3,2` 会让 `robot1`、`robot2`、`robot3` 分别使用 1、3、2 号挂点。

```bash
./scripts/test.sh --planner-only                       # 五条长途离线规划
./scripts/test.sh --planner-only --case CTS_S3         # 指定一条离线用例
./scripts/test.sh --task multi --case CTS_S3 --headless # Gazebo 服务端闭环
```

离线规划与闭环都会显示实验编号。闭环省略 `--headless` 会打开 Gazebo 和 RViz。一次运行一个闭环实例。用例编号为 `CTS_S1`、`CTS_S2`、`CTS_S3`、`CTS_S4` 和 `CTS_E_TO_S3`。

## 5. 自动跑其他固定用例

```bash
./scripts/test.sh --task single --experiment full_route
./scripts/test.sh --task multi --experiment full_route
```

加 `--experiment full_route` 后会使用预设起终点自动跑完全程。无参数时默认自动跑单车厂房全程。终端会显示本轮实际加载的规划器、跟踪器和版本。

`short_turn` 是短弯，改成 `short_straight` 测直行，改成 `full_route` 测全程。加 `--scene-variant critical` 测临界窄通道，加 `--scene-variant too_narrow` 测绕行。

需要回退到旧规划器和 Python 跟踪器时，显式选择旧后端。

```bash
./scripts/test.sh --task multi --experiment full_route --planner-backend legacy --architecture-profile baseline --tracker-implementation python --no-build
```
