# FR05 独立审核：CHANGES_REQUESTED

日期：2026-11-14（Asia/Shanghai）。Codex独立审核。工作区 `/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。

**FR05 暂不通过。确认两项阻塞：ROS 默认参数偏离冻结合同，及回放认证/完成反馈未绑定模型摘要。** 修复范围均为FR05边界接入，不要求重构搜索、修改FR01–FR04或新增优化器。另须使最终源码与验证证据重新对应。

## 1. 已实现且有依据的部分

【代码事实】实际新增原子 `FormationPlaybackPlan.msg`，包含原TimedTrajectory、完整解析model JSON、版本和摘要。旧消息未修改。launch按backend互斥启动uniform或legacy；run_demo解析backend并构建本worktree overlay。

【代码事实】`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py:verify_uniform_playback_plan()`核对模型、摘要、身份和采样字段；`_uniform_frame_at()`（890行）调用`UniformTimedPlan.state_at()`产生任意时刻车身姿态，中心由刚性模型派生。不是各车中心独立线性插值，也不是非零pivot货物圆弧的弦线插值。展示节点将同一帧传给Marker与TF。

【代码事实】实际有串行job状态、bridge忙准入、原子信封失效清理、终态后第二请求。实现没有引入控制器、Gazebo、连续优化或FR06。

【实验事实】本轮独立执行168项定向测试全部通过：61项demo/assets与FR01/FR02/新信封测试，84项相关功能回归，23项旧optimizer节点回归。FR03 S09的200状态性能测量未重复执行，已验收源码/测试未变；不将其计成本轮执行。

【实验事实】用当前解析回放库重新验证五份真实已保存信封（600004/5/6、600010/12），包含A直行、O平移、含零角车的S和非零pivot A弧。每相位11个内部时刻，共209个时刻，解析frame对模型的位置/车头误差与独立刚性残差均为0（本轮浮点计算）。这证明解析显示路径一致性，不替代这些轨迹在原地图上的重新S9审核。见`saved_envelope_reverification.py/json`。

【实验事实】提交的主场景83项、失败场景49项、篡改场景9项记录均标记通过；参数快照均为0.16m/0.16rad、50000次/20秒。它们不能证明冻结阈值下通过。因后述两份源码哈希不对应，也不能把旧完整ROS运行自动归给当前全部源码。

【实验事实】我目视核对了`rviz_zoom_start/mid/end.png`三张保存图，能看到三车及路径、位置随时间变化。结合工具发布记录、完整截图和日志，已有可见图形证据；不因没有单独的“真人确认签字”新增阻塞。本轮Codex未在GUI中重新鼠标操作，不将图像复核写成自己的现场交互实验。

## 2. 阻塞 FR05-R1：ROS 默认容差及预算改写冻结合同

**位置：**

- `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py:UniformPlannerNode.__init__()`（约318–355行）。
- `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch`（约46–57行及对应param）。
- `tests/ros_uniform_playback_smoke.py:scenario_main()`把`rviz_style_achieved_goal_inside_frozen_tolerance`写成≤0.16m。
- 依据：冻结DECISIONS公共阈值、FR03默认配置和实施计划FR04固定预算；FR05职责是接入同一合同。

【代码事实】ROS默认XY、货物角、终端车头容差均为0.16；冻结标准为0.02m、1°（约0.0174533rad）。此外默认扩展预算改为50000、时间改为20秒，原基线为10000/30秒。不是物理轮系限值变动，但确实改变了到达判定与实验预算。新增smoke甚至将0.16误称为冻结容差，故测试全绿没有覆盖住这一偏差。

【实验事实】本轮实际启动`run_demo.sh`，在全新私有master `http://127.0.0.1:11461`、独立ROS_HOME、空地图上，通过标准InitialPose/Goal话题输入：

```
起点 (10,10,0)
目标 (10.1,10,0.1)
实际终点 (10,10,0)，T_end=0
XY误差 0.1 m，角误差 0.1 rad≈5.73°
planner: PUBLISHED / NO_MOTION, feasible=true, executable=true
playback: HOLD_END, executable=true
```

没有注入假规划结果或伪造审核。这是一条正常请求在真实ROS流程中的误到达判定，已超冻结阈值。FR04实现本身未变，其使用传入配置；问题在FR05把更宽配置传给它。证据`live_contract_probe.json/log`、`run_demo.log`、`ros_commands.json`。

**最小修复：** ROS节点默认参数及launch默认恢复冻结值，优先引用已有配置默认而不是另复制一组数；测试直接检查冻结值，不能以配置恰好相等证明研究合同正确。恢复10000/30秒预算。可以为预算负例显式使用较小预算并记录，但不得用加宽目标容差或更大扩展数使正例过关。近目标反例不能原地就获“已到达”：应真正达到冻结容差，或如实返回失败，不要求为这一反例新增算法能力。

## 3. 阻塞 FR05-R2：认证和完成反馈仅匹配plan_id

**位置：**

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py:_status_label()`（1057–1075行）；`build_markers()`、`status_document()`共同使用它。
- `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py:_status_for_trajectory()`（430行附近）和`on_playback_plan()`。
- `catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py:on_playback_status()`（1063–1114行）。

【代码事实】信封内部有模型摘要校验，但“信封内部一致”与“该模型取得当前审核”是两件事。显示认证只匹配plan_id及三个布尔字段，没有核对model_sha256/编队身份；规划节点的播放确认也只按plan_id匹配，错误摘要的HOLD_END仍会释放busy并发布可执行状态。冻结要求是同一job/plan_id/hash贯穿规划、审核与回放。

【实验事实】定向调用真实显示库：A模型直行终点(11.5,10)，B模型终点(12,10)，plan_id同为700124，两个摘要不同。B信封自身版本/摘要/样本校验通过；将A的合成FEASIBLE审核状态交给B的frame，实际返回：

```
status_document.executable=true, certification=FEASIBLE
Marker executable=true, audit_state=FEASIBLE
```

见`audit_digest_binding_probe.py/json`，输出包含两份完整摘要。没有修改显示函数或伪造函数返回值；审核状态是测试输入。这是已复现的身份检查漏口，**不是声称正常流程已自动发生跨模型串台**。

【实验事实】调用真实`UniformPlannerNode.on_playback_status()`，active digest为a×64，输入同ID、digest为b×64的HOLD_END（且executable=false），节点仍从PLAYING改为HOLD_END，并发布feasible/executable=true。该探针只模拟反馈输入和输出publisher，不启动ROS。见`feedback_digest_probe.py/json`。

**最小修复：** uniform_v02分支使用已存在的plan_id、formation_id、backend、model_sha256绑定“显示模型—审核状态—已发布job—完成反馈”。缺失/不符不得获得FEASIBLE或解除busy；正确身份和零运动保持仍须正常完成。保留legacy原行为。不要新增互相等待的latched状态基础设施，也不必要求回放先收到审核标签才能向planner报告几何终态；关键是反馈确实属于该planner已发布的模型，且不能借另一摘要的证书标为可执行。

## 4. 证据对应问题与未验证范围

【实验事实】提交的`SOURCE_MANIFEST.sha256`共有22项，其中两项不匹配：

| 文件 | 清单前缀 | 当前前缀 |
|---|---|---|
| formation_uniform_planner_node.py | f292e8ac… | 07c44bc2… |
| ros_uniform_playback_smoke.py | 38c3aaab… | 3bb4e277… |

当前smoke已包含额外rviz_style实验，正式报告仍描述三条主实验。本轮已保存清单原文及当前源码快照；审核期间两份文件没有再变化。修复后应冻结源码、更新哈希、重跑必要ROS流程并使报告一致，不能只改清单宣布旧测试覆盖了新代码。此项并入返工证据交付，不另开算法任务。

【实验事实】首次私有ROS启动因沙箱禁止本地socket而失败；取得执行许可后在11461端口完成了上述真实复现。只启停本轮创建的进程组，专用ROS_HOME在本验收目录；未清理全局ROS，也未访问/停止Claude保留的11411 master。

【尚未验证】本轮没有重新执行完整main/failure/tamper三套ROS脚本，也未构建install空间或重做一轮GUI点击。现有可见图形和旧ROS记录已分析；本轮独立实测范围是定向真实ROS反例、168项回归、两个身份探针和保存信封复核。已发现阻塞后不以重复旧正例替代修复。未进行FR06、动力学或控制验证。

## 5. 可直接交给Claude的最小返工任务

> 在V0.2 worktree仅修FR05-R1/R2；允许修改uniform ROS节点、uniform相关launch参数、播放库与播放节点的身份绑定分支、对应FR05测试和本阶段证据。必要的demo资产测试可以调整，但不得降低已冻结阈值。禁止修改FR01–FR04实现及旧测试预期、车辆/轮系限值、搜索原语/去重，不新增NLP、抢占、消息架构或FR06。
>
> 1. 恢复默认目标0.02m/1°、终端车头1°及10000扩展/30秒；统一node与launch，删除smoke里把0.16称为冻结值的验收逻辑。保留“发布的模型终点可以是冻结容差内实际达到的终点”，但必须对原请求检查严格容差，不把消息goal改为实际终点就视为满足原目标。
> 2. 把现有model摘要贯穿uniform审核标签、Marker/status、job反馈；仅ID一致而摘要不同或缺失时不能标FEASIBLE/可执行，不能以其HOLD_END释放当前job；正确身份、消息先后到达、零运动保持与legacy行为需保持正常。复用现有身份字段，不新增并行状态系统。
> 3. 增加定向正负例：本报告真实近目标误判；同ID不同摘要的自洽信封/审核状态；同ID错误摘要的HOLD_END；正确摘要的正例；合法容差内保持、正常busy拒绝与结束后第二请求。不能仅用摘要错误的“不自洽信封”代替“两个各自自洽但身份不匹配”的反例。
> 4. 运行相关单元回归；用私有master和专用ROS_HOME执行冻结默认参数下的必要main/failure/tamper验证与至少一轮可见RViz流程。保留每相位11个内部显示时刻和既有位置/角阈值；可使用原100Hz定量检查，GUI低频轮可定性。预算负例的较小预算须显式记录，不改默认合同。若严格参数下遇到规划失败，先提供固定输入和首个失败层，不能顺手改搜索/加宽阈值。
> 5. 每轮结果保存到新的rework目录，保留本轮原始证据、所有失败尝试和Codex探针；源码冻结后生成清单及报告。输出真实命令/退出码、overlay、节点清单、参数/模型/原始状态及图形证据。不得用只改哈希清单补验收。完成后停止，不提交Git、不进入FR06。

两项修复均是FR05原合同的落实，不要求大规模重构。默认10°圆弧动态界限制仍按原阶段说明保留，不在本轮扩展范围。

## 6. 基线保护与最终状态

【实验事实】FR01/FR02/FR03/FR04 ACCEPTED记录及12份被这些记录引用的源码/测试哈希匹配；原FR04独立审查的168份证据匹配。原仓库673个tracked、43个untracked、60个T5含忽略文件条目与保护快照核对无差异（排除准备阶段已授权修改的三份设计文档），原二进制diff未变。

本轮检查后已有源码、测试、配置、任务文档及提交证据均未改变；仅新增Codex验收文件和本轮隔离运行输出。详见`after_checks.json`、`historical_protection.json`。

**FR05=CHANGES_REQUESTED；FR01–FR04验收不撤销。Codex未修改实现/测试/配置，未提交Git，未进入FR06。**
