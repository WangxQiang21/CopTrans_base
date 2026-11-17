# V2：A*、中线与最终三车轨迹分层

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

C2-B/V1 ACCEPTED。读C2 result.search的实际route/guide/allocation，V1 debug消费，库build_markers与权威frame_at。允许仅改V1已有debug字段导出/显示函数、formation_demo.rviz、tests/test_virtual_a_corridor_display.py；不另造topic/schema/ROS节点，不改求解器/审核。

## 实施和接口

沿同一planning_debug载荷加raw_route_points、guide_points、guide_quality、route_id、corridor_used、chain_sha256，都是算法实际消费的数据，禁止显示另一条人工示意线。保留V1单次身份过滤和namespace生命周期。

独立MarkerArray话题`/formation/guides`：A*橙细线默认隐藏，中线绿细线默认显示，未完整验证guide标UNVALIDATED_GUIDE，几何折线写PL_GUIDE而非SMOOTH_PATH；Bezier显示采样只作绘制，安全来自其控制凸包证书。最终三车路径复用原/formation/markers保持原模式颜色和加粗，不为了“蓝线”覆盖模式语义；在RViz图层名说明Final certified vehicle paths。走廊面低alpha，图层均可开关。

只显示当前route所用链；可选其它route默认关且标candidate。优化失败时不显示旧最终轨迹为当前成功；失败诊断可留当前candidate但必须有FAIL/REFERENCE_ONLY标识，V3统一按生命周期清理。

## 测试/验收

同一plan三层点坐标对真实route/guide/final模型误差≤1e-9m（绘制Bezier采样对解析点）；candidate与final身份不一致不混合。未审核guide和假audit success不得使用安全标签。图层启闭不影响规划、播放时间或TF。保存一张窄弯中部图与其载荷hash，非样本时刻frame仍满足现有1e-7模型一致标准。

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_display.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_playback.py' -v
```
复用已存在/前置新增测试，不写重复全套；Codex抽对guide是否真实消费、route_id和最终digest。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
