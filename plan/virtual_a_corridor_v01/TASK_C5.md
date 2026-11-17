# C5：公平性能实验与最终回归

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

C3/V3 ACCEPTED，C4可SKIPPED。本阶段冻结算法，不借实验改权重/限制/审计；问题定位后另报返修。允许拟新增tools/virtual_a_corridor_compare.py、少量tests/fixtures/virtual_a_corridor/cases.json及新实验数据；若C0已建输入直接扩展，复用G4场景和现有工具，禁止修改历史记录和生产算法。拟新增runner不是新规划器。

## 实验输入与接口

拟CLI `python3 tools/virtual_a_corridor_compare.py --cases <json> --variants legacy_route,corridor --repeats 3 --output <new-dir>`。仅调用现有VirtualAMapRequest/plan_virtual_a，按PlannerConfig显式模式；不导入任何参考路径作为warm start，不从scene ID选答案。

A=C0后旧强引导，B=C3二维走廊，C仅C4实施后另标签。使用同一个C0批准积分审核合同、同地图/车型/偏移/初终态、N=8、max_initials≤6、60s请求软预算、依赖/CPU线程/机器；公平记录每组特有设置（guidance权重/筛查等），不能假称设置完全相同。确定性求解也重复3次测wall波动，交替AB/BA次序；不借重复挑最好一条。

冻结至少12个请求：精确直线、3.93m历史近直线、单盒左右2、完整双移线及镜像2、L窄道及镜像2、凹U、历史长回头弯、几何无路、F有路但三车不可行反例。历史地图若缺，明确缺测而非替代冒充；G4已知可行样例只提供请求/地图，参考解不入规划。每组36次，C4不实施时72次离线总量；这仅C5运行，本轮不执行。每次预算耗尽保留失败，不自动补跑到成功。

每请求保存输入/配置/源码/地图hash、三层审核、是否executable、route/corridor/centerline/NLP/各audit分项wall、S、max|C_i|、∫κ²ds、∫η²ds、有效曲率变号数（|κ|<1e-5m^-1忽略）、区域数/面积/portal半径/包含违约。曲率/代价积分用现有profile求值；max峰值区分采样值和认证上界。最小OBB净空若checker无真实欧氏距离，不拿distance-field代理充数：记录现有保守下界且标方法，或N/A，不新开发净空系统。

主离线指标=三层审核且executable数/全部请求；端到端发布率只对实际ROS请求单独统计，不能把离线executable写成发布成功。报告成功率计数及区间、wall中位/p95和失败类别，全分母含预算/无路；可解和已知反例分层另报，避免无路负例污染可解任务成功率。

## 验收/回归命令

```bash
python3 tools/virtual_a_corridor_compare.py --cases tests/fixtures/virtual_a_corridor/cases.json --variants legacy_route,corridor --repeats 3 --output evaluation/virtual_a_corridor/C5/NEW_RUN
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_ros_adapter.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_playback.py' -v
python3 -B -m unittest discover -s tests -p 'test_uniform_playback_plan.py' -v
```
CLI和cases拟新增，NEW_RUN由本次时间生成且必须不存在。旧FR06选择既有已知plan复审/一个uniform_v02 ROS smoke，不每次跑全FR01–FR06；测试输出重定向新目录，保留旧timing_continuous已知失败事实。V3已完成相同源码ROS证据可复用；有代码漂移只重跑受影响用例。

验收：72条输入输出一一对应/失败不丢分母、反例0误认证、审核/模型身份不退化、预算overshoot如实披露、所有新增阶段回归完成。成功率无需提升固定百分比；不提升也可验收实验执行，但必须写出哪些场景变差、耗时瓶颈和算法结论边界。Codex独立复算随机一对A/B及一个失败分母，抽核真实ROS发布记录，不能把solver_success当最终成功。

## 交付、Codex验收和停止

证据放`evaluation/virtual_a_corridor/本阶段/UTC-run/`：一份SUMMARY.md、results.json、精确命令/退出码、源码hash及必要小输入/模型/图片。交接列base/current SHA（未提交如实写）、diff stat/文件、关键数值、失败/NOT_RUN和未完成项；求解成功、几何、动态、碰撞、发布/显示分栏。

Codex审增量diff/数学、核对输入和原始证据，独立复现一个关键正例及反例；不依赖Claude口头完成声明。验收后由用户明确下达下一阶段。本阶段完成即停止，不自动推进、不提交Git。任务引用的主文档均在`plan/virtual_a_corridor_v01/`，不依赖聊天历史。
