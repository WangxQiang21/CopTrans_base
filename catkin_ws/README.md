# expv3 ROS1 Demo（车辆中心轨迹）

**R19 多拐角纯 A 修复（2026-10-21）：** 用户截图对应输入执行时间由 34.14 s 降至 14.95 s，6 次换模降为 0；完成三轮原始初值对照和四例实际 ROS 发布审核。见 [R19 记录](../relust/R19_multi_corner_A_20261021/report.md)。

**R18 搜索热循环提速（2026-10-21）：** 三组用户输入的三轮串行对照，前端计算耗时分别缩短 18.5%、23.8%、19.7%，主路径、反向辅助候选及展开次数一致；三个输入已完成当前源码 ROS 复现及发布轨迹审核。见 [R18 记录](../relust/R18_search_hotpath_20261021/report.md)。

**R17 搜索提速（2026-10-19）：** 三组固定输入的三轮后端计时，缩短幅度中位数 53.1%，最终轨迹与修改前一致；三个用户输入已完成 ROS 联调及发布轨迹审核。实际求解时间、继续搜索门槛和验证范围见 [R17 记录](../relust/R17_search_speed_20261019/report.md)。

**R16（2026-10-19）：** 默认改为严格认证候选中按执行时间最短选解，增加纯 A 转弯位置候选，并修正 0.55 倍 P12_env1 图1搜索预算分配。三个用户输入已离线严格审核；C++ 构建通过，ROS 联调因自动审批服务容量错误尚未执行。详见 [具体原因与结果](../relust/R16_minimum_time_20261019/report.md)。

**R14 求解加速（2026-10-19）：** 默认已启用批量检查、等价栅格行合并、重复候选去重和有可行解后的提前结束。实际前后耗时及验收见 [R14 报告](../relust/R14_solve_speed_20261018/report.md)。

2026-10-18 默认 profile 为 `kinematic_sim_mode_switch_only_v2`：目标速度 2.0 m/s，每次切模式静止 0.3 秒，同模式内不因曲率/转向需求主动降速。起终点及模式边界保留加减速；短路段可能无法达到目标速度。可传 `target_v:=数值`。原受限动态对照使用 `config_file:=/home/wq/whut_thesis/expv3/catkin_ws/src/fwis_v3_demo/config/params_restricted.yaml`。当前 profile 是运动学仿真，横向、轮速和转向需求记录为诊断；Spin 保留角速度限值。复测见 [R13 修复记录](../relust/R13_switch_speed_map_20261018/report.md)。

当前 `run_demo.sh` 与默认 ROS 参数使用 `mode_path_v2`（R03 连续控制/连续时间原型）；`planning_backend:=legacy` 可显式运行旧后端对照，`mode_path_v1` 是 R02 恒定原语后端。R03 跨场景验收和 cost 校准仍未完成，此默认切换用于当前测试，不代表算法验收完成。`/plan/quality_report` 保存候选、能力 profile、时间相位和评分。下面的旧耗时和 cost 记录属于历史版本，详见 [R01 实现与验证](../plan/12_%E6%A0%B8%E5%BF%83%E9%87%8D%E6%9E%84R01_%E5%AE%9E%E7%8E%B0%E4%B8%8E%E9%AA%8C%E8%AF%81.md)。

R04 历史低速与走廊修复见 [修复记录](../plan/14_R04_%E4%BD%8E%E9%80%9F%E5%BC%82%E5%B8%B8%E4%B8%8E%E8%B5%B0%E5%BB%8A%E6%98%BE%E7%A4%BA%E4%BF%AE%E5%A4%8D%E8%AE%B0%E5%BD%95.md)：该阶段按用户日志输入复测，P12_env4/P12_env2 均为连续纯 A，中途无零速点，后端约 4.74/6.09 s。R05 的当前数据见 [汇总](../relust/R05_target2_late_reverse_20261010/comparison.json)。走廊从最终连续几何构造并显示，连续优化安全验收仍为独立整车扫掠。

R02 增加 `allow_reverse_fallback:=true/false`，仅作用于 `mode_path_v1`；默认 true，但先搜索正向解，无合格正向解才尝试局部倒车，全程倒车始终禁止。负平移段的 `TimedPoint.v` 为负，仍受加减速约束。可用 `./run_demo.sh --map P12_env1 planning_backend:=mode_path_v1 allow_reverse_fallback:=false` 关闭回退。

该工作空间包含 Demo 所需的 `fwis_common`、`fwis_map_server` 和 expv3 算法包，可直接基于 ROS Noetic 构建，不依赖 `exp/catkin_ws`。地图与场景定义统一位于仓库根目录 `maps/`：每张地图使用 `maps/mapNN-特点/` 独立目录，内含 `preview.png`、`map.pgm`、`map.yaml`、来源说明和对应场景配置；目录索引见 `maps/README.md`。规划状态和 ROS 起终点均为前后轴中心的中点；最终输出 `/plan/timed_trajectory` 中逐点的 `(time_from_start, [x,y,v,theta])`，数值 `theta` 按时间连续展开。A 使用前后反向转向的中心曲率和精确圆弧传播，O 含斜移/横移，S 保持中心位置不动且 `v=0`。运行环境需 ROS Noetic、Python 3、NumPy、SciPy 和 CasADi/IPOPT；本机已具备这些依赖。

## 构建与运行

一键启动（推荐，有图形桌面时自动打开 RViz，并自动规划 S09 默认目标）：

```bash
cd /home/wq/whut_thesis/expv3
./run_demo.sh
```

选择地图时先查看名称，再传地图名或包含 `map.yaml` 的目录。所有可运行地图都带项目预设起终点；选图后默认使用预设起点并发送预设目标。论文图的预设位姿是项目为测试选取的，不是论文中的原始位姿。若要手动设置，在命令后加 `auto_goal:=false`，再在 RViz 中使用 `2D Pose Estimate`、`2D Nav Goal` 设置车辆中心起终点：

```bash
./run_demo.sh --list-maps
./run_demo.sh --map P12_env1
./run_demo.sh --map /path/to/map_directory
./run_demo.sh --map P12_env1 --map-scale 0.9
```

手动选图默认比例为：map01 `0.75`、map03 `0.7`、map04/P12_env1 `0.5`、map05/P12_env2 `0.7`、map07 `0.8`、map08 `1.2`、map15 `0.8`；map02、map06、map09、map16 为 `1.0`。可用 `--map-scale`、`--scale` 或 `-scale` 指定其他正比例。map08、map09 的论文栅格已使用 5 像素中值滤波平滑边缘；P12_env2 停车排障碍已按原直边重建并去除毛刺和缺口。每次启动都从原始地图生成临时缩放副本，停止 Demo 后自动清理。

所有可运行地图都提供 `scene_meta.yaml` 预设起点和终点。用 `run_demo.sh --map mapNN` 启动时默认载入起点并自动发送终点；传 `auto_goal:=false` 后可在 RViz 手动设置。论文地图的这些测试位姿由项目选取，不代表论文作者原始实验位姿。

原有直接传 ROS 参数的方式仍兼容，例如 `./run_demo.sh map_dir:=/path/to/map_directory auto_goal:=false scene:=manual_map`。无图形桌面时运行 `./run_demo.sh rviz:=false`；脚本在未检测到显示环境时也会自动关闭 RViz。按 `Ctrl+C` 停止。RViz 中灰色 `/plan/path_coarse` 与浅色 `/plan/path` 分别是粗解和优化几何；暗色 `/plan/mode_strip_coarse` 与亮色 `/plan/mode_strip` 显示粗解/优化解的连续模式段，A 青色、O 橙色、Spin 紫色。车辆当前模式显示在 `/robot/hud`，红色箭头 `/robot/heading_marker` 指示车体航向。走廊沿路径离散节点构造并以地面线框矩形显示；有效走廊按最终局部模式着色，point-fallback 琥珀色（仅作优化引导，不是安全保证）。蓝色矩形是按时间回放的车辆。Spin 的车辆中心不移动，紫色小弧仅作模式符号。

启动后，在另一个已加载 ROS 环境的终端可发送 S09 预设车位，例如：

```bash
source /opt/ros/noetic/setup.bash
rostopic pub -1 /preset_goal std_msgs/Int32 'data: 7'
```

`7` 对应顶部斜车位；预设编号 `1–7` 标在 RViz 地图上。规划器会按成本和碰撞约束自行选模式，不保证预设标签所提示的模式一定被采用。

手动构建/启动方式（用于开发调试）：

```bash
cd /home/wq/whut_thesis/expv3
source /opt/ros/noetic/setup.bash
catkin_make -C catkin_ws -j1
source catkin_ws/devel/setup.bash
roslaunch fwis_v3_demo demo.launch
```

RViz 中使用 `2D Pose Estimate` 设置车辆中心起点、`2D Nav Goal` 设置车辆中心目标。默认 S09 起点 `(13.2,2.25,π)`；可直接使用已有场景预设目标。无图形环境可运行 `roslaunch fwis_v3_demo demo.launch rviz:=false`，另开终端在同样 source 后分别运行：

```bash
python3 -m unittest discover -s tests -p test_continuous_geometry.py -v
python3 -m unittest discover -s tests -p test_continuous_retiming.py -v
python3 tests/ros_s09_smoke.py
python3 tests/ros_spin_smoke.py
python3 tests/ros_oblique_smoke.py
python3 tests/ros_failure_smoke.py
```

低内存环境串行运行针对性测试。旧 `test_core.py` 曾有高内存运行问题，完整执行前先审查受影响用例。隔离探针使用空闲非保护端口；不要占用或清理 11311、11312、11320。详细探针命令及本轮验收矩阵见第 15 文档第 6 节。

## 数据流和安全边界

- `/map`、`/start_pose`、`/initialpose`、`/move_base_simple/goal`：沿用 exp 的地图及 RViz 入口，位姿均指车辆中心。
- `/plan/path_coarse` 与 `/plan/seed`：多条 2D 拓扑路线、稀疏 A/O/Spin 航点连接图与限额 Hybrid A* 竞争后的精确原语和粗解。前端已使用 OBB–占据格 SAT，而非 exp 的点采样 OBB。
- `/plan/path`、`/plan/timed_trajectory`：仅当后端固定模式优化、终点约束、原语连续扫掠及时间重采样 SAT 全部通过时发布。
- `/plan/mode_strip_coarse`、`/plan/mode_strip`、`/plan/corridor_markers`、`/robot/marker`、`/robot/heading_marker`、`/robot/hud`：两阶段模式色带、走廊、车体航向箭头和带时间的车辆中心回放；`TimedPoint.mode` 提供与时间采样对齐的当前模式。
- `/plan/search_status`、`/plan/status`：搜索和后端的成功/失败原因；新目标会清除旧结果。

完整特征圆 STC 不可行时，后端会降级到特征点走廊。**点走廊不提供整车安全保证**；候选必须通过独立的整车 SAT 扫掠，且状态显示 `corridor=point_fallback`。优化候选碰撞时用原语空间回溯寻找非零安全改动，找不到就拒绝，不会把碰撞解发布为成功。

## 当前范围

已完成 S09 默认目标、原地 Spin、纯横移 O 和碰撞目标清除旧结果的端到端 headless 测试及基础单元测试。最新默认目标中，C++ 粗解为 A/O 两模式，连续优化调整成 O/A/O 三段；TimedTrajectory 有逐点模式标签，最终独立 sweep 通过。向量化 SAT 后，粗搜约 `0.48s`、优化发布约 `0.87s`；固定 refine 约 `0.80s`（NLP 求解约 `0.62s`），锚点候选约 `0.06s`，时间扫掠约 `0.01s`。连续优化的 O 角调整由 4 次降到 2 次，其中小于等于 5° 的调整由 2 次降到 1 次，共同分 `12.853→12.503`。窄位揉库目标 `(3.387,4.950,1.567)` 粗搜约 `0.68s`、优化约 `0.76s`，独立扫掠通过。400 个随机位姿 SAT 与标量参考一致，单测 20 项；另有 Spin、纯斜移、碰撞清除等 ROS smoke。加大 knot 间距或将 IPOPT 上限降到 80 次会损害揉库目标质量，已撤回；目前 NLP 的 100 次迭代仍触及上限，是后续热点。这些测量仅代表当前 S09 场景。

粗解来自多条 2D 拓扑路线上的稀疏 A/O/Spin 候选图，再由有 incumbent 限额的 Hybrid A* 竞争改善；已有安全解后的 Hybrid A* 改进预算为 1500 次，首次找解仍可用 180000 次。昂贵的混合 A/O/Spin 图仅在净空权重 0 和 1.5 路线运行，四条拓扑路线仍参加便宜 O/Spin DP。Python 后端以同一加权代价精修固定动作序列，并在粗解锚点 DAG 的邻近最多 3 个锚点间拟合 A/O/Spin 候选；每条边先做全车 swept SAT，路径可作为安全候选，最多精修一个有竞争力的提议。优化离散步长最多 1m/30°，IPOPT 上限 100 次迭代；上限返回仍要通过精确重放、终点容差、共同代价不变差和独立 swept SAT。S09 六个有效预设、两个额外揉库目标和一个预期碰撞目标已端到端回归；8 条 timed trajectory 共 23,156 个独立扫掠样本通过。粗搜约 0.13–1.39s，后端约 0.43–2.82s。锚点/几何候选在揉库场景将共同代价由 16.445 降至 16.213（O/Spin/O），在顶部斜车位由 14.944 降至 14.673（A/O/Spin）；另一个揉库目标仍选择固定序列解 15.916（锚点候选 16.038）。500 次改进预算在顶部斜车位退化，1500 保留更优结果。Bonmin 全局 MINLP 虽已修正代价项，难例仍约 8s 超限，保持关闭。验证只覆盖当前 S09 地图，不代表其他场景的成功率或时延保证；静态地图与完美控制假设不变，不输出轮速或执行真实车辆控制。
