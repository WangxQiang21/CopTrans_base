# 诊断计数与 marker 语义

- `map_collisions` 是 `map_collision_count + vehicle_collision_count`，统计在 Guide 采样站点上被 `FormationCollisionChecker.static_check()` 报为地图 OBB 或车车 OBB 碰撞的站点数；每个采样站点是一条 checker 状态。它不是碰撞车辆数，也不是接触点数。细分类型看 `map_sample_counts` 和 `vehicle_map_collisions`。
- `MAP_OBB_COLLISION (sample)` marker 定位到 Guide 的评估采样，不是求出的墙面接触位置。采样碰撞证明该被检查状态碰撞；没有采样碰撞不证明连续 Guide 安全。
- `BOTH_ORDERS_GAP` 表示 OBB 多边形未被指定的两个相邻凸 corridor 区域并集覆盖（双顺序裁剪均发现 gap）。它不是地图占据栅格中的障碍碰撞。顺序不一致/边界或退化情形会回报 UNKNOWN。
- 最终 `audit_path_in_corridors()` 检查连续路径的 F 参考点相对 allocation 指定的单个 region 的包含性；最终 `audit_collision()` 检查三车完整 OBB 的扫掠。二者都不由 Guide sample marker 代替。
- RViz marker 会受诊断上限截断；HUD 计数来自完整 assessment summary。

源码：`corridor.py` 的 `obb_coverage_in_adjacent_union()`、`_evaluate_station()`、`_assessment_map_summary()`、`_collect_collisions()`；`pipeline.py` 的 `_progress_assessment_summary()`。目标 plan 的具体 marker 仍需同一 request identity 的 `planning_debug` 和 quality report。
