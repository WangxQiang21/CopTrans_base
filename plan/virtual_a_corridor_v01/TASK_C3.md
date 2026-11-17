# C3：多通道选择与有限失败恢复

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

C2-B ACCEPTED。仅改善候选调度/错误分类/有限恢复，读P/pipeline.py make_route_guides/_collision_retry_context/plan_virtual_a和optimizer.py make_initial_guesses/_guess_route_guidance/solve_open_path_guided；允许这些函数、contracts.py少量记录字段及现有相关tests；拟新增tests/test_virtual_a_corridor_recovery.py。不改模型、审核器、时间律，不预先实施C4，不加全局搜索框架。

## 实施合同

1. 保持最多3条A*候选、6初值和60s请求软预算，单solve设置30s；corridor/QP/包含/audit同样计入。每次开始操作检查剩余预算，库不能立即中断如实记overshoot。禁止以IPOPT success直接胜出。
2. 开阔直连只在C1精确LOS成立时加入候选；其仍不是三车证书。路线多样性首版用简化路线互相可视的shortcut等价筛查和受障碍分离的通道差异，非严格同伦分类，不声称完备。轻微狗腿被合并，单盒左右两侧须保留；共享guide容量，不额外加无限A*。
3. 6初值分配最多每有效路线1个，至少1个真正endpoint-only；其余择最多2个不同长度/曲率形状，去重同向同长度。新模式的endpoint初值不得用route0高权重目标；可以受当前走廊约束。若无路线不硬套不相干走廊。
4. 只一次恢复，**替换**当前最多2次碰撞重试，不能在旧循环外再加一层：数值不收敛换剩余一个初值；C退化选另一已有路线或单次seed切分；固定σ失败仅一次相邻portal范围内重分配；OBB失败优先换尚未试的不同路线，几何局部封堵只能用真实碰撞位置并明确仍启发式；预算耗尽直接返回。选择当前首次失败类别的一种动作，不能依次全做。
5. solver=auto时沿C0一次IPOPT→SLSQP规则，算入同一总尝试池；timeout不回退。候选排序先可行后代价；只有完整三层审核通过才进入executable集合，已有有效解不能被未审目标值更低的解否决。
6. search保存route_count/initial_count/recovery_count/first_failure_layer/last_reason/各段耗时，用现有attempts，不增多级异常体系。ROS失败保持已有不发布/清理语义。

## 测试与标准

单盒左右拓扑+开阔三条狗腿去重；endpoint不强贴route0；伪求解成功但audit失败；模拟timeout不fallback；OBB失败只换一次路线；恢复仍失败无plan；已全审candidate在后续超时仍可保留（前提原预算合同允许）。用fake solver计数测试预算边界，附一个真实成功和一个真实失败请求，不反复全套ROS。

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_recovery.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```
首项拟新增。验收routes≤3、initials≤6、恢复≤1、预算耗尽不开始新solver、0误发布；记录C2对C3同输入各一次变化，成功率提升不设必达门槛。若需要增加预算/初值/全局框架才有效，停并提交证据，不自行扩大范围。Codex计数和审accepted选择代码，重放一个预算反例。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
