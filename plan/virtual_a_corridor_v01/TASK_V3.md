# V3：状态生命周期与ROS/RViz回归

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

V0/V1/V2/C3 ACCEPTED，C4非前置。读现有planner admission/generation/watchdog及playback on_job_state/on_request/on_playback_plan/_clear，G5 smoke driver。仅允许上述节点的显示生命周期接线、tests/ros_virtual_a_smoke.py增量观察HUD/debug/markers、相关显示测试；禁止重写原串行状态机/identity验证，禁止改规划物理算法。

## 状态合同

新合法请求admit：HUD清除旧PASS/model，旧corridor/guide/final executable清理，展示当前PLANNING。busy拒绝：维持当前合法计划/图层，不消费新reset。预admission不兼容FREE初态拒绝：不能用旧admission字段给新plan认证，HUD顶层新失败，旧终态仅HOLD/previous标识。地图更新/generation过期：清不匹配诊断与执行数据。成功时对应context/model匹配的三层报告才PASS；HOLD_END/NO_MOTION真实保持。失败/坏信封清理HUD执行状态和三图层，禁止latched旧消息重现。停止节点后重启接收端也要匹配新的job/admission，不沿用旧成功。

使用原plan_id/formation/backend/context/model/generation；显示没有model阶段用当前admission键，不能只按plan_id赋PASS。同topic DELETEALL+ADD原子发送；不同topic以同身份事件清理，检查最终没有旧namespace，不能声称跨topic原子事务。stage状态仅来自真实已有事件，未知显示UNKNOWN，不让UI推算审核。

## 实跑与命令

先在一个未占用私有master，例如11637，确认不属于其他工作。以下示例需三个已source环境终端，`SCENE_MANIFEST`用`$PWD/evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/scene_manifest.json`（先核存在及SHA），`RUN`为新证据目录；不要使用受保护默认master：
```bash
ROS_MASTER_URI=http://127.0.0.1:11637 roscore -p 11637
# 另一终端，同ROS_MASTER_URI
./run_demo.sh --map evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/inputs/S2 --scale 1.0 backend:=virtual_rigid_a auto_goal:=false rviz:=true virtual_a_goal_policy:=straight
# C2-B新增并验收后，在上述launch命令末尾加 virtual_a_guidance_mode:=corridor；原G5无此参数
python3 -B tests/ros_virtual_a_smoke.py --scene-manifest "$SCENE_MANIFEST" --scene-key S2 --scenario failure --goal-policy straight --output "$RUN/report.json"
```
现有driver main/failure/tamper和baseline-envelope参数真实存在。同master确保launch/driver都设URI；S2路径存在性先核，若将来不同从fixture manifest定位，不生成参考解替代规划。S1 FREE另一次main可复用相同源码已有证据，source/身份有变才重跑。tamper用本session真实envelope并指定--baseline-envelope，不手写假模型。uniform_v02单独关闭新节点后启一次关键main，复用原driver，不在同master双后端发布。

新显示拟纯单元命令：
```bash
python3 -B -m unittest discover -s tests -p 'test_formation_hud.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_display.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_receiver.py' -v
```

量化：全部状态序列0误PASS/0旧namespace残留/0失败误播；收到admit或clear后两个tick内（10Hz则≤.2s，不含消息传输延迟）完成本节点显示更新，传输延迟另记；playback非样本模型误差≤1e-7、刚性≤1e-6m。截图起点/弯中/终点/失败各一张，日志只存必要事件和原始信封。ROS/插件权限阻塞标NOT_RUN，不能完整验收。Codex亲触一个真实新请求及失败/身份负例，查旧默认uniform_v02仍可用。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
