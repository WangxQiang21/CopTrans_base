# 真实源码复用矩阵

核对HEAD：`23fd16227e75a68f03f3ac7483555063c8189313`。以下为真实定义行号；新增接口另在阶段任务书标注。P=`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`。

| 真实位置 | 定义 | 处理 |
|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:369` | `def _guess_route_guidance(guess, request, route_guides, n, collocation=None,                           obstacle_context=None):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:429` | `def _collocation_states(vector, packet, collocation):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:550` | `def make_initial_guesses(request, config, *, route_guides=(),                          obstacle_context=None):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:590` | `def _numeric_constraint_vector(z, request, config, packet, collocation):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:646` | `def _numeric_objective(z, request, config, packet, collocation,                        route_points=None, route_weight=0.0,                        route_kappas=None, route_curvature_weight=0.0,                        route_collocation_points=None,                        route_collocation_weight=0.0,                        clearance_proxy=None):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:806` | `def _build_casadi_nlp(request, config, packet, wall_limit_s=None,                       route_points=None, route_weight=0.0,                       route_kappas=None, route_curvature_weight=0.0,                       route_collocation_points=None,                       route_collocation_weight=0.0,                       clearance_proxy=None):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py:1273` | `def solve_open_path_guided(request, config, *, route_guides=(),                            obstacle_context=None,                            geometry_audit_config=None):` | C0配点修复/C2约束/C3初值；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py:135` | `def _astar_route(grid, allowed, cost_field, start, goal, config,                  deadline):` | C1/C2/C3接入现有唯一管线；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py:225` | `def make_route_guides(grid, start_qf, goal_qf, config=None, *,                       blocked_points=()):` | C1/C2/C3接入现有唯一管线；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py:349` | `def _collision_retry_context(report, plan, grid_sha256, route, intervals,                             retry_index=1):` | C1/C2/C3接入现有唯一管线；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/pipeline.py:420` | `def plan_virtual_a(map_request):` | C1/C2/C3接入现有唯一管线；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:757` | `class PlannerConfig` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:1006` | `class GeometryAuditConfig` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:1753` | `class RouteGuideConfig` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:1834` | `class GuideRoute` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:2122` | `class VirtualAMapRequest` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:2173` | `def planning_context(request, grid, planner_config, timing_config,                      collision_config):` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/contracts.py:2198` | `class VirtualAResult` | 小结构/新设置入context；旧默认保护；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py:114` | `class C2CurvatureProfile` | 直接复用，不重写模型；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py:389` | `class VirtualAPath` | 直接复用，不重写模型；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py:681` | `def map_spatial_state(qf, jet, model, *, s=0.0, position_error_bound_m=0.0,                       angle_error_bound_rad=0.0):` | 直接复用，不重写模型；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/audit.py:340` | `def _integration_plan(pieces, start_psi, config):` | 复用；C0仅有推导支持的积分局部修正；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/audit.py:1343` | `def audit_geometry(request, path, config=None, *, planning_context=None):` | 复用；C0仅有推导支持的积分局部修正；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/audit.py:3857` | `def audit_dynamics(request, plan, config=None, *,                    expected_planning_context=None):` | 复用；C0仅有推导支持的积分局部修正；中 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/collision.py:160` | `def load_map(yaml_path):` | 直接复用，不降低证书/余量；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/collision.py:351` | `def make_sweep_segments(plan, config=None):` | 直接复用，不降低证书/余量；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/collision.py:414` | `def verify_sweep_segments(plan, segments, config=None, *,                           state_samples=5):` | 直接复用，不降低证书/余量；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/collision.py:660` | `def error_envelope(plan, config=None):` | 直接复用，不降低证书/余量；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/collision.py:1005` | `def audit_collision(request, plan, grid, config=None):` | 直接复用，不降低证书/余量；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/ros_adapter.py:243` | `class VirtualABackendSettings` | 复用请求/信封，必要设置传递；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/ros_adapter.py:371` | `def request_from_snapshot(snapshot, grid, settings, initial_context):` | 复用请求/信封，必要设置传递；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/ros_adapter.py:734` | `def publication_fields(snapshot, result, stamp, dt, settings=None):` | 复用请求/信封，必要设置传递；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/ros_adapter.py:895` | `def verify_envelope_fields(envelope_fields):` | 复用请求/信封，必要设置传递；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/ros_adapter.py:1008` | `def result_summary(result):` | 复用请求/信封，必要设置传递；低 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/ros_adapter.py:1096` | `class VirtualABackendHooks` | 复用请求/信封，必要设置传递；低 |

## ROS、回放和实验

| 真实文件 | 实际函数/职责 | 改动 |
|---|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py` | `frame_at/_virtual_frame_at/_status_label/status_document` | 直接复用权威轨迹和身份，不另做插值 |
| 同上 | `_mode_text/build_markers/build_hold_markers/PlaybackMarkerOptions` | V0文本/选项，保持旧默认与返回值 |
| `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py` | `on_job_state/on_quality/on_playback_plan/_publish_tick/_publish_hold/_clear` | V0 HUD和V3清理；中 |
| `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py` | `_finish_virtual_success:1254`、`_finish_virtual_failure:1217`、`_publish_job_state:653` | V1实际result.search图层，非新规划节点 |
| `catkin_ws/src/fwis_v3_demo/rviz/formation_demo.rviz`、`launch/formation_demo.launch` | MarkerArray/TF/initialpose/goal/backend | V0/V1参数和图层；低 |
| `catkin_ws/src/fwis_v3_playback/package.xml` | rospy、geometry、visualization、std、tf依赖 | V0登记实际jsk依赖 |
| `formation_collision.py` | `FormationCollisionChecker._obb_overlap/static_check/swept_check_analytic` | 复用SAT/原车体扫掠；不复制车辆审核 |
| `formation_continuous_a/timing.py/trajectory.py` | `quintic_progress/VirtualATimedPlan` | 原样复用，旧JSON往返/摘要回归 |
| `tools/virtual_a_benchmark.py/virtual_a_scenes.py` | G1–G4基准/场景 | 复用fixture输入，C5用小runner避免旧大脚本扩张 |

## 代码与现有记录揭示的限制

1. `_collocation_states/_numeric_objective/_build_casadi_nlp`都在Gauss循环累计完整权重后用于配点残差，是同一错误；`_numeric_constraint_vector`只用段末端，不能机械当bug修改。
2. `_guess_route_guidance`把非route初值也绑定index=0；C0保留隔离变量，C2/C3取消新模式强绑定。
3. `solve_open_path_guided`只有auto+allow且非超时失败才回退；ipopt+True不回退。现有超时不回退应保留。
4. `audit._integration_plan`阈值比较Simpson余项+position_guard；更多panel不保证总界单调降低。历史uturn summary的五次求解成功被约9.7e-8–1.01e-7总预算拒绝，阈值5e-8；最终原因又被最后一次迭代超限覆盖。
5. `VirtualAMapRequest`当前只有request/grid/planner_config/timing_config/collision_config；没有corridor输入。`VirtualAResult.search`可复用作诊断，不再建平行Result。
6. trajectory.py严格检查顶层keys，走廊debug不能加进model_json顶层。设置应进现有planning_context，实际链在search中。
7. 本机标准ROS目录没找到jsk文件，未核对所有overlay，V0实际rospack/rosmsg后决定安装，不声称已可用。
8. playback节点`_status_for_trajectory`对quality.pipeline只比plan_id后合并；HUD不能把它直接当完整身份绑定质量，V0应在显示边界复用完整身份字段核对。
9. 旧单车timing_continuous的dynamic_peaks KeyError在既有记录中未修；本轮不能冒称所有旧测试通过。

## 优先复用的现有测试

`tests/test_virtual_a_optimizer.py`、`test_virtual_a_pipeline.py`、`test_virtual_a_integration_bounds.py`、`test_virtual_a_trajectory.py`、`test_virtual_a_ros_adapter.py`、`test_virtual_a_playback.py`、`test_virtual_a_receiver.py`、`test_formation_playback.py`、`test_formation_demo_assets.py`、`ros_virtual_a_smoke.py`。只定向运行，带证据输出的测试先核查输出路径；禁止全量discover覆盖历史。
