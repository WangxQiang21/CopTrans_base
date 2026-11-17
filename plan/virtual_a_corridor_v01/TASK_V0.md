# V0：跟车标签与屏幕HUD

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

## 前置与允许文件

基于现有G5可独立于C0进行，按文件不重叠交付；读`formation_playback.py`库的_mode_text/build_markers/build_hold_markers/_status_label/status_document和ROS playback脚本on_job_state/on_quality/_publish_tick/_publish_hold/_clear。允许上述两文件、fwis_v3_playback/package.xml、formation_demo.launch/rviz、test_formation_playback.py/test_formation_demo_assets.py；拟新增tests/test_formation_hud.py。不改规划/审核/时间/消息MD5。

## 实施

1. 实际查`rospack find jsk_rviz_plugins`、`rosmsg show jsk_rviz_plugins/OverlayText`、插件描述文件；本轮未找到标准安装，不能假定可用。若缺少，报告包/ROS版本与安装需求，用户允许安装后再GUI验证；纯单元可先做，真实HUD门为NOT_RUN。只用现有ROS1插件，不自制图形框架。
2. 车辆标签缩为car1 | A | v=.xx m/s，来自当前frame，不是均匀索引/估算速度。静止无模型时写STATIC/v=0，不能假装确认轮系；F简短地标保留。默认不显示ω，现有PlaybackVehicleState没有ω字段，若用户要再由权威plan.state_at取，首版不扩大字段。
3. 库拟`hud_text(playback_status,job_state,quality)->str`一个纯格式函数，复用_status_label的plan/formation/backend/model身份认证，不再写第二套validator。注意现有_status_for_trajectory对quality.pipeline合并仅比plan_id；HUD读取该pipeline前还需比formation/backend/model/context（该边界一次），否则旧质量metrics可能混入新HUD。复用已有身份字段，不重构原质量系统。audit三栏UNKNOWN/PENDING/PASS/FAIL，只有真实匹配三层报告为PASS；IPOPT success不能FEASIBLE。planning stage当前若无内部阶段事件写PLANNING，不虚构实时NLP/audit进度。
4. 现有playback节点发布`/formation/hud` OverlayText，10Hz或状态变化发布，复用已有tick。参数hud_enabled/hud_left/hud_top/width/height，默认标准1280×720**渲染视口**下width=360、left=900、top=20；需按实际视口调整，面板宽不能当渲染宽。保存截图实际尺寸和参数。插件未证实自动右锚，首版固定标准窗口+可调像素，不加窗口监听器。
5. 通过PlaybackMarkerOptions显示开关和build_hold_markers可选参数在真实demo禁用全局长status/Cargo reason，函数旧默认保留便于旧接口回归；HUD不可用且显式disabled时保留原简短地图提示，不建自动多框架fallback。uniform_v02同样可用HUD但状态按其原身份规则。

## 验收

成功→新plan PLANNING→拒绝→STATIC/HOLD_END/NO_MOTION测试，旧digest/错plan质量不得显示PASS。标签位置跟frame中心误差≤1e-9m、速度格式误差≤.005m/s；标准窗口HUD完整在右上角无遮挡三车，拖相机不移动HUD，缩窗口只要求可配置、不承诺自动对齐。

```bash
rospack find jsk_rviz_plugins
rosmsg show jsk_rviz_plugins/OverlayText
python3 -B -m unittest discover -s tests -p 'test_formation_hud.py' -v
python3 -B -m unittest discover -s tests -p 'test_formation_playback.py' -v
python3 -B -m unittest discover -s tests -p 'test_formation_demo_assets.py' -v
```
HUD测试拟新增，真实ROS用现有run_demo新backend S1单次并保存开始/播放/失败截图，不重跑所有G5。Codex核实际msg字段/插件尺寸，错身份反例。缺插件不能将V0完整验收PASS。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
