# P16_narrow_maze 性能优化交接

更新时间：2026-10-03  
工作区：`/home/wq/whut_thesis/expv3`

## 当前任务与约束

本轮目标是继续优化 expv3 的通用多模式规划算法，重点排查 `P16_narrow_maze` 求解慢的问题。不是要为 P16 写专用状态机，也不是要改地图障碍布局。P16 是从论文 Figure 19 数字化后按车辆宽度统一缩放的地图；地图元数据写明缩放因子约 0.25、车宽映射为 0.5 m。论文图没有机器可读的起终点姿态，测试姿态只能作为复现近似。

## 本轮做了什么

### 1. 定位到粗搜索阶段

此前对 P16 做过一次 headless ROS 复现。使用的近似测试姿态为：

- 起点：`(12.0, 12.0, 0.0)`
- 终点：`(18.0, 13.75, 0.0)`

它们近似对应论文坐标 `(3,3)` 到 `(27,10)` 在以 `(15,15)` 为中心、缩放 0.25 后的坐标；论文原图中的精确朝向未确认。该次规划没有得到粗解，优化器没有拿到 seed，因此耗时热点不在 NLP/联合优化阶段。

修改前状态：

```text
search_budget_exhausted
iterations=360000
guided_iterations=180000
fallback_iterations=180000
route2d=unavailable
route2d_reason=no_route_candidate
seconds=155.129955
```

### 2. 栅格碰撞检测加速

在 `catkin_ws/src/fwis_v3_planner/src/planner_node.cpp` 中，给每个栅格行预先合并连续占用单元为水平矩形段；`pose_clear()` 对这些矩形段做 OBB-SAT，而不再对车体包围盒中的每一个占用栅格逐格做 SAT。合并表示的是同一行相邻占用栅格的几何并集，未缩小车体、未放松障碍判定。查询范围还额外覆盖相邻栅格，SAT 再排除不相交项。

### 3. 修复两轮搜索重复烧满预算

之前 `search/max_iter=180000` 实际被引导搜索和无 incumbent 的 fallback 各使用一次，单目标最多扩展 360000 次。现将无 incumbent 时的引导搜索上限设为 90000 次，fallback 只使用总预算 `180000 - guided_iterations` 的余量；带可行 incumbent 时仍受 `max_iter_with_incumbent` 限制。搜索函数增加了每次调用的节点预算参数。

修改后同一组 P16 测试：

```text
search_budget_exhausted
iterations=180000
guided_iterations=90000
fallback_iterations=90000
route2d=unavailable
route2d_reason=no_route_candidate
route2d_seconds=0.608233
guided_seconds=1.351439
fallback_seconds=1.204486
seconds=3.164167
```

这次结果把失败场景从约 155 秒降到约 3.16 秒（约 49 倍），并将扩展数减半；但**目前只是解决了慢，不代表 P16 已规划成功**。还需要在其他地图验证路径质量、模式使用和耗时是否回退。

### 4. 最新 P16 诊断与后续限制

当前 `search/enable_route_heuristic` 已恢复为 `false`。增加 clearance-floor 备选路线（常规候选无 incumbent 后尝试）和 0.5 倍细栅格 fallback 后，同一近似起终点仍未找到解。最近一次记录约 180k 扩展、总耗时约 3.0 s；2D 路线数 6，DP 最远到达约 8/37 waypoint，稀疏混合图 4 次调用共 575 次扩展、未触 8000 上限。失败前沿约在 `(15.91,12.33)`，现有证据不支持单纯增加混合图预算。

这仍然只是 `(12,12,0) → (18,13.75,0)` 近似姿态的失败。P16 地图缺少机器可读起终点和朝向；进一步专项优化前，应先从论文图复核车体箭头和缩放坐标。

## 当前规划器结构（本轮涉及部分）

1. 在地图上运行多组 clearance 权重的二维 8 邻接 A*，生成拓扑路径。
2. 尝试快速 Oblique 路径、沿路线 waypoint 的模式/航向 DP，以及稀疏混合模式图，为 Hybrid A* 提供可行候选。
3. 若候选可行，运行带障碍距离场启发的 A/O/Spin Hybrid A* 做改进；未成功时可用剩余扩展预算运行无引导 fallback。
4. 只有粗解成功后，优化器才会做连续轨迹优化、模式候选/联合迭代和最终碰撞回验。

因此 P16 当前已知线索是：2D 轨迹存在，但候选轨迹在路线中段受障碍扫掠约束阻断；增加 clearance-floor 备选仍只把 waypoint DP 前沿推进到约 8/37，稀疏混合图没有触预算上限。下一步优先校准端点/朝向，再针对路线拐点附近的候选航向和安全转接检查做通用改进。

## 改动文件与验证状态

- `catkin_ws/src/fwis_v3_planner/src/planner_node.cpp`
  - 增加按行合并占用栅格的 SAT 查询。
  - 增加搜索调用的节点预算，并让 fallback 使用总预算余量。
  - 失败状态增加 route、guided、fallback 分阶段耗时。
  - **本轮新增：** 路线诊断计数有效/重复 2D 路线、单朝向候选碰撞类别、waypoint DP 最远可达点和末端状态、稀疏图调用/成功/扩展/预算命中及细分耗时。
- `catkin_ws/src/fwis_v3_demo/config/params.yaml`
  - 新增 `search/guided_iter_without_incumbent: 90000`。
  - `search/enable_route_heuristic: false`。
- `joint.enabled: false, joint.max_knots: 18`。Bonmin A/B 中 N04 29 knots、N02 21 knots 被默认上限拒绝；N04 临时提高至 32 后约 27 s 返回 `LIMIT_EXCEEDED`，N05 在上限内也返回 `LIMIT_EXCEEDED`。配置已恢复默认，详见 `relust/map_probe_report.md`。
- 编译：旧版 C++ 代码曾通过 `catkin_make -C catkin_ws -j4`；本轮新增诊断代码也已通过该构建，新二进制包含诊断字段。
- P16：增强候选和细栅格 fallback 后仍失败；精确起终点待确认。
- ROS：N01–N06 六张通用压力图已通过粗搜、优化和独立 5 mm 定时 swept-SAT（6/6）；详见 `relust/map_probe_report.md`。
- Bonmin 全局 MINLP：默认局部锚点/模式交替不等于全局联合优化；N04/N05 的 opt-in Bonmin 没产生可用或优于 incumbent 的方案，且 N05 延迟增加。
- 临时 ROS master 已通过对应 roslaunch 会话 Ctrl-C 关闭；没有清理其他 ROS 会话的 planner/master。
- 当前工作目录不是 Git 仓库根目录，`git status` 在 expv3 下不可用；场景指标与完整 JSON 列在 `relust/map_probe_report.md`。

## 新对话建议从这里继续

1. 复核 P16 起终点与朝向。地图没有预置姿态，当前姿态只是论文坐标近似；不能把近似姿态失败作为 Figure 19 的最终结论。
2. 以 `relust/map_probe_report.md` 中 N01–N06 为压力图基线，增加重复运行、S09 与至少一张 P11/P12 数字化图回归。
3. 针对 Bonmin 修正 wall-clock 超时行为和 knot 规模预检；先在小规模 seed 上与默认局部交替做配对比较，再决定是否扩展到多图。
4. 对 row-run SAT 做独立等价核验，覆盖贴障碍、斜向姿态和地图边界。
5. 按跨图安全、末端误差、时延、cost、曲率/模式切换和净空比较消融，不添加地图名称专用状态机。

## 可直接复现的操作

```bash
cd /home/wq/whut_thesis/expv3
./run_demo.sh --map P16_narrow_maze rviz:=false
```

无 RViz 时发布 `map` frame 的 `/start_pose` 和 `/move_base_simple/goal`。若要重用本文的粗测姿态，坐标见前文。规划状态读取：

```bash
rostopic echo -n 1 /plan/search_status
```

由于机器上存在多个遗留 ROS master，复测前先检查 `ROS_MASTER_URI`、`rosnode list` 和目标节点所属会话，避免把新目标发到旧 planner。
