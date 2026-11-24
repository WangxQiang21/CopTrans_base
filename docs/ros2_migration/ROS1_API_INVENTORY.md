# ROS1 接口清单

| 入口 | 输入 | 输出 |
|---|---|---|
| 单车 `planner_node.cpp` | `/map` OccupancyGrid，`/start_pose` PoseStamped，`/initialpose` PoseWithCovarianceStamped，`/move_base_simple/goal` PoseStamped | `/plan/path_coarse` Path，`/plan/seed` 与 `/plan/request` PlanSeed，`/plan/seed_candidates` PlanSeedCandidates，`/plan/search_status` String，`/plan/markers` MarkerArray |
| 单车 `optimizer_node.py` | 地图、种子、候选、搜索状态 | `/plan/path` Path，`/plan/timed_trajectory` TimedTrajectory，`/plan/mode_strip`、`/plan/mode_strip_coarse`、`/plan/corridor_markers` MarkerArray，`/plan/status`、`/plan/quality_report` String |
| 单车 `playback.py` | 轨迹、起点、诊断 | `/robot/marker`、`/robot/heading_marker`、`/robot/hud` Marker |
| 多车 `formation_uniform_planner_node.py` | `/map`、`/formation/plan_request` FormationPlanRequest、播放状态，虚拟刚体路径监听 `/initialpose` | `/formation/playback_plan` FormationPlaybackPlan，`/formation/optimizer_status`、`/formation/quality_report`、`/formation/job_state`、`/formation/planning_debug` String |
| 多车 `formation_playback.py` | 播放模型或时间轨迹、作业状态和诊断 | `/formation/markers`、`/formation/path/vehicle_ID`、调试路径、回放状态、可选 TF |
| 多车 `formation_goal_bridge.py` | RViz 起点/目标、地图、作业状态、播放 TF | `/formation/endpoints`、`/formation/plan_request`，通过 Trigger 服务取消待执行规划 |
| 多车 legacy 前端和优化节点 | `/map`、`/formation/plan_request`、`/formation/plan_seed` | FormationPlanSeed、FormationTimedTrajectory 与状态 |

搜索与优化配置为节点私有参数，包含 `search/*`、`planning/*`、`vehicle/*`、`collision/*`、`quality/*` 和 `time/*`。多车参数详见 `formation_demo.launch`、`formation_params.yaml` 与 `transport_formation.yaml`。规划帧为 `map`，位置以车体中心表达。单车使用 tf 四元数转换；多车播放还发布自身的编队与车辆变换。仿真模型位姿与规划回放须使用不同的 TF 子帧。

C++ 依赖 NodeHandle、Publisher/Subscriber、ROS 时间和日志。Python 依赖 rospy 参数、发布/订阅、Timer、Service/ServiceProxy、时钟和日志。单车及多车均使用 ROS1 XML Launch，许多输出使用 latch。ROS2 迁移保留消息与话题语义，持久输出采用 transient local。
