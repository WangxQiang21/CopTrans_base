# CopTrans 仿真使用方法

## 1. 启动交互仿真

```bash
cd /home/wq/Whut/CopTrans_base
./scripts/interactive.sh
```

脚本会构建程序并打开 Gazebo 和 RViz。已有最新构建时，用 `./scripts/interactive.sh --no-build` 直接启动。默认是室内厂房，西侧靠墙有一处只允许单车通过的 1.10 m 限宽口。可用 `./scripts/interactive.sh --list-scenarios` 查看场景，加 `--scenario port_01` 切换到港口。

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
./scripts/test.sh --task multi --scenario factory_indoor_01 --experiment short_turn \
  --architecture-profile optimized --multi-native-frontier --multi-convex-corridor \
  --multi-reachable-clock --tracker-implementation cpp --no-build
```

`short_turn` 是短弯，改成 `short_straight` 测直行，改成 `full_route` 测带货全程。脚本自动发目标，终端会打印结果和日志目录。
