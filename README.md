# CopTrans 仿真使用方法

## 1. 启动交互仿真

```bash
cd CopTrans_base
./scripts/interactive.sh
```

脚本会增量构建并打开 Gazebo 和 RViz，`./run_demo.sh` 无参数时也进入同一测试。首次运行或代码变更后直接运行脚本。使用 `--no-build` 时，脚本会核对安装与源码是否一致。默认是室内厂房，西侧靠墙有一处只允许单车通过的 1.10 m 限宽口。可用 `--list-scenarios` 查看场景，加 `--scenario port_01` 切换到港口。

新机器先准备 ROS 2 Humble、Gazebo Harmonic、对应的 ROS 桥、RViz 和 Colcon。若缺少 CasADi C++ SDK，可执行 `python3 -m pip install --user --no-deps casadi==3.7.2`。已有独立 SDK 时，可先执行 `export COPTRANS_CASADI_ROOT=/实际路径/casadi`。首次启动会编译 Release；旧安装或文件校验失败时，去掉 `--no-build` 再运行。

要测试 V3 原生联合求解器，在仓库目录执行下面任意一条。终端启动时显示选定的规划器、跟踪器、源码和安装指纹。三车交互求解时，RViz 左侧面板显示规划器上报的实际后端和构建编号。收到原生计划后，本轮 `experiment.json` 的 `planner_source` 为 `hybrid_joint`，`model_sha256` 标识收到的模型。源码及安装指纹可在同目录的 `environment.txt` 查询。

```bash
./scripts/interactive.sh --planner-backend hybrid_joint
./scripts/test.sh --task single --planner-backend hybrid_joint --experiment short_turn
./scripts/test.sh --task multi --planner-backend hybrid_joint --experiment full_route
```

## 2. 在 RViz 设置起点和终点

1. 等待左侧「规划与执行」面板提示选择起点。点击 **2D Pose Estimate**，在地图上按住左键并拖出朝向。Gazebo 会把三辆车和货物放到选定位置。
2. 等面板确认三车已停稳，点击 **2D Goal Pose**，在地图上拖出终点和朝向。松开鼠标后自动规划，三车通过预检后同步执行。两次点选设置的都是整个编队的位姿。
3. 在「规划与执行」面板查看结果。完成或失败后窗口会保留。按 **Ctrl+C** 退出，再运行脚本开始下一轮。

## 3. 查日志或复跑

在面板点 **复制编号**。反馈问题时附上编号和看到的现象。每轮日志自动保存在本地 `reports/runs/实验编号/`。

```bash
id=20261125_000000_aB1cDe  # 换成面板上的实验编号
cat "reports/runs/$id/experiment.json"
./scripts/interactive.sh --replay "$id" --no-build
```

`experiment.json` 可查起终点、结果、失败原因和轨迹文件位置。复跑使用归档中的地图版本、起终点和参数，自动重新规划，并生成新的实验编号。用当前地图重新测试时，直接启动交互仿真，再选择相同的起终点。

## 4. 自动跑固定用例

```bash
./scripts/test.sh
./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment short_turn --no-build
```

无参数默认跑单车 State Lattice 短弯。三车默认启用 `uniform_v02` 的原生前沿、凸走廊和共同时间轴，两者都用 C++ 跟踪器。终端会显示本轮实际加载的配置，单车的 `experiment.json` 还会记录选中的轨迹来源。格点规划器在长距离可选用重新认证的连续候选，来源显示为 `continuous_portfolio`。

`short_turn` 是短弯，改成 `short_straight` 测直行，改成 `full_route` 测全程。加 `--scene-variant critical` 测临界窄通道，加 `--scene-variant too_narrow` 测绕行。

需要回退到基础规划配置和 Python 跟踪器时，运行同一测试并加上两个参数。

```bash
./scripts/test.sh --task multi --architecture-profile baseline --tracker-implementation python --no-build
```
