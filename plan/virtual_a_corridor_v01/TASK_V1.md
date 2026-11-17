# V1：实际凸走廊可视化

## 独立执行上下文与边界

源码基线`23fd16227e75a68f03f3ac7483555063c8189313`，原分支feature/virtual-rigid-formation-a；只在Codex准备并明确交付的工作区执行。记录HEAD/status/前置ACCEPTED，不覆盖现有数据，不自行建分支、提交或推送。P=`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/`。新增接口均为待实现建议，不能假定已存在。

三车偏移(1,0)/(0,1)/(0,-1)m从已有合同取，全车正向统一A，p_i=pF+Rψ r_i，车头独立；保留FREE/STRAIGHT、reset、公共时间律、积分证书、三车运动学/动态、连续OBB和plan/context/model绑定；失败不得误播旧成功。旧FR06和历史证据保护。

**本阶段禁止过度防御性编程**：复用既有模块/类型，输入边界必要验证一次，内部使用已验证合同。不增无意义wrapper/validator/adapter/manager/factory/多层抽象，不加无证据fallback、宽泛吞异常、无限重试、重复日志测试或无关重构。每项新增检查/抽象说明对应哪个可复现错误，否则不加；必要数学、安全、身份检查不能因此删除。

Claude实施时仓库根测试环境：
```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:${PYTHONPATH:-}"
```
若ROS overlay未准备，先纯Python，真实ROS标NOT_RUN。只跑定向测试，禁止全量discover覆盖历史输出。

## 前置与文件

C1 ACCEPTED，V0可独立。读C1实际CorridorChain、VirtualAResult.search、ros_adapter.result_summary、现有planner virtual结果路径及playback callbacks。允许planner现有脚本、P/ros_adapter.py的结果摘要、P/corridor.py纯导出函数、ROS playback脚本、formation_demo.rviz，拟新增tests/test_virtual_a_corridor_display.py；不改轨迹信封/模型/审核、不得加新规划节点。

## 数据和接口

C1尚未接优化器时，先用C1生成并独立审核的保存JSON在私有ROS测试中发布显示（显示测试内增加一个直接发布case，不新建生产节点），显示状态必须CANDIDATE_REFERENCE_POINT_ONLY；C2后才可标used_by_optimizer。拟`corridor_display_fields(chain,identity)->dict`直接导出原A/b/vertices，放result.search，并由planner通过单个`/formation/planning_debug` std_msgs/String latched发布。字段复用plan_id/formation_id/backend/grid_sha256/planning_context_sha256和已有generation，加route_id/chain digest/regions；没有model时model_sha=null。链摘要为规范JSON的SHA，不造版本registry；此载荷仅显示，不能赋执行认证。

C2-B负责将构造链写入result.search，V1不在旧legacy管线重新生成另一条链。C1阶段fixture发布case同时提供真实测试请求/admission身份，不能忽略身份检查；自主请求自动显示的端到端门在C2-B/V3补验。playback消费匹配当前admission的载荷，在`/formation/corridors` MarkerArray画TRIANGLE_LIST扇形三角化CCW凸多边形（z=.005、alpha=.15）和LINE_STRIP边（z=.01），稳定route/region namespace/id。portal/debug法向默认关闭。不可重新平滑/扩大区域，图就是算法输出；generation/plan/map/context不匹配直接忽略。V1阶段C1独立测试链可发布显式debug请求身份，不能伪造自主优化已消费。

每次替换同topic先DELETEALL再ADD同一个MarkerArray，避免已有queue_size丢单独清理包；body markers保持独立topic，不误删车辆。已有admit/fail/clear事件调用清空，不新状态机。V3集中全生命周期验收。

## 测试与门槛

导出后每顶点/平面与C1原值≤1e-9m，重建三角形总面积相对≤1e-9，凸面顺序正确；两route不同颜色确定。错plan/map/generation载荷不显示，新小链替换旧大链无多余id。RViz截图含实际算法JSON hash；C1几何自由不能被标三车安全。

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_display.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_ros_adapter.py' -v
```
首项拟新增。真实ROS只需一个C1链显示+替换截图，可与V0同session复用。Codex比vertices和hash及Marker清理，不能仅看好看的图片。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
