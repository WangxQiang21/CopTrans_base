# FR05返工独立复核：CHANGES_REQUESTED（仅剩R2发布前边界）

2026-11-14，Codex。工作区`/home/wq/whut_thesis/expv3_multi_v02`，分支`refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。

**FR05-R1已关闭；FR05-R2已修好发布后的身份不匹配，仍缺“模型尚未发布时禁止确认”的守卫。FR05暂不通过，只需一次回调级最小补齐。** 不要求重构规划、重做消息架构或重复整轮GUI测试。

## 1. 已核实通过的修复

【代码事实】ROS节点默认引用`SuccessorConfig`和`GraphSearchConfig`，launch恢复0.02m/1°、10000扩展/30秒，smoke也按冻结值断言；FR01–FR04实现未变。

【实验事实】本轮独立运行182项测试，全部通过：75项demo/时间/轨迹/信封/反馈身份测试，84项相关功能回归，23项旧optimizer节点回归。FR03 S09性能测量未重复，不计入本轮数量。输出在`unit_61.log`（名称沿用运行器，实际75项）、`unit_84.log`和`unit_node.log`，实际命令/退出码见`test_commands.json`。

【实验事实】在独立master `11461`、专用ROS_HOME重新启动真实run_demo，经标准InitialPose/Goal话题复跑原请求`(10,10,0)→(10.1,10,0.1)`：约30秒后`FAILED / TIME_BUDGET_EXHAUSTED`，有效信封数0，没有任何可执行状态。该结果符合上轮要求的“真正到达或诚实失败”，不再误报零运动到达。见`live_contract_probe.json/log`、`ros_commands.json`。本次只启停自己创建的进程，未碰Claude的11472或其他master。

【实验事实】原同ID不同摘要显示探针现返回`GEOMETRY_ONLY`、`executable=false`；已发布状态下的错误摘要HOLD_END保持PLAYING且无输出；匹配摘要的HOLD_END正常完成。说明原R2的发布后漏洞已经修复。见`audit_digest_binding_probe.json`及下述反馈探针前两项。

【实验事实】新清单174项逐项哈希匹配，上一轮源码/清单不对应问题已解决。提交的main112项、near-goal8项、failure57项、tamper14项检查均标记true，参数记录均为冻结值；失败尝试保留。Codex分析这些原始记录，没有将它们冒称本轮亲自重跑的四套完整ROS实验。

【实验事实】目视核对本轮保存的`run3_08.png`和`run3_45.png`，可以观察三车从PLAYING移动到HOLD_END。结合点击记录和ROS日志，认可已有可见图形验证证据。本轮未再次鼠标操作GUI。

## 2. 唯一剩余阻塞：FR05-R2发布前空摘要被当作相同身份

**位置：**`catkin_ws/src/fwis_v3_optimizer/scripts/formation_uniform_planner_node.py:on_playback_status()`，约1070–1102行；同时参考`on_request()`初始化活动摘要和`_plan_worker()`对状态的检查。

【代码事实】新请求进入PLANNING时，`_active_model_sha256=None`，尚无已发布轨迹。当前身份门只比较：

```python
report.get("model_sha256") != self._active_model_sha256
```

反馈缺失摘要或显式给出null时，左边同样是None，比较结果为相等。只要plan_id、formation_id和backend一致，就会继续接受HOLD_END。代码没有先要求活动模型摘要已经存在。

【实验事实】独立调用真实回调，状态严格对应实际发布前阶段：

```
_state=PLANNING
_active_model_sha256=None
_published_wall=None
```

输入同ID/编队/backend、缺失model_sha256的HOLD_END，结果：

```
_state=HOLD_END
_playback_seen=True
发布 feasible=True、executable=True、published=True
```

显式`model_sha256=null`同样复现。缺摘要的HOLD_START也会错误设置`_playback_seen=True`。五项完整输入/输出及复现脚本保存在`feedback_phase_boundary_probe.py/json`：两项已发布正负例正确，三项发布前反例违反合同。

该探针使用真实`on_playback_status()`，只模拟反馈与输出publisher，不修改回调结果。这是已确认的防错检查遗漏；**没有在正常GUI运行中观察到回放节点自行产生这种缺摘要HOLD_END**，不能把受控输入复现描述成正常实验必然串台。

【技术推断】除提前释放busy和错误标可执行之外，真实工作线程之后返回时，`_plan_worker()`会因状态已不再是PLANNING而丢弃结果。问题仍属于原R2“仅确认本节点已发布模型”的合同，不是新的算法研究任务。

现有反馈单测的fixture始终为PLAYING且有非空活动摘要，因此没有覆盖这一阶段边界。把两个缺失值当作相等，不足以证明存在已发布身份。

## 3. 可直接交给Claude的最小补齐任务

> 仅补齐FR05-R2的发布前空摘要守卫。允许修改`formation_uniform_planner_node.py:on_playback_status`、`tests/test_uniform_feedback_identity.py`和新的FR05返工证据目录；其他已通过模块、搜索、时间规划、回放算法、launch默认值不改。
>
> 在接受HOLD_START/PLAYING/HOLD_END反馈前，明确要求当前已有有效的已发布模型身份，不能把`None==None`当作匹配。现有实际摘要只在通过审核、准备发布时建立，最小做法可以是先拒绝没有活动摘要的状态，再做现有身份比较。根据代码生命周期选择简单守卫，不新增握手或状态系统。
>
> **不能笼统拒绝所有PLANNING状态反馈**：信封发布后、回放首次确认前，job仍可能处于PLANNING；合法零时长保持也可能直接反馈HOLD_END。已发布且身份正确的这两种情况必须继续成功。
>
> 增加发布前缺摘要/显式null的HOLD_END和缺摘要HOLD_START负例：保持PLANNING，不设置playback_seen，不发布可执行或已发布状态，不解除busy。保留已发布后的错误摘要负例，增加/保留已发布且身份正确的PLANNING→HOLD_END正例以及零运动终态正例。测试直接驱动真实回调即可。
>
> 运行反馈身份套件和本次`feedback_phase_boundary_probe.py`（可改为加入预期断言，但保留原输入），保存真实输出。守卫以外源码不变时，**不要求重跑整套main/failure/tamper、30秒近目标和GUI流程，也不要求重跑所有182项**；这些未改路径已有本轮与提交证据。若实现意外影响其他路径，再有针对性扩测。
>
> 更新本次变更文件的哈希、清单及报告，明确哪些ROS/GUI证据继承前轮、哪些回调测试由最终源码新运行，不能冒称全部旧ROS记录来自新源码。保留所有旧目录，完成后停止交Codex验收；不提交Git、不进入FR06。

范围就是一个回调守卫和阶段边界测试，不要求新增其他防御能力。下一次复核也以此范围和正确反馈正例为准。

## 4. 基线与验收范围

【实验事实】174项提交源码清单匹配，上一轮Codex验收目录原200个文件均匹配；FR01–FR04引用的12份已验收源码/测试哈希匹配。原仓库673个tracked、43个untracked、60个T5含忽略文件条目符合保护快照（排除准备时已授权改动的三份设计文档），原tracked二进制diff未变。

【实验事实】本次审核期间既有源码、测试、配置、任务文档和提交证据均未改变。只新增验收报告、源码快照及隔离运行输出。见`baseline_checks.json`、`historical_protection.json`、`after_checks.json`。

【尚未验证】本轮未再跑全部四套ROS脚本、未重新GUI点击、未做FR06。启动阶段显示tick余量偏小以及固定10°圆弧动态界限制沿用已披露范围，本轮不追加返工。近目标诚实失败不是“近目标规划能力已成功”的证明。

**最终：FR05仍为CHANGES_REQUESTED；R1关闭，R2仅剩上述发布前边界；FR01–FR04验收不撤销。Codex未修改实现/原测试/配置，未提交Git，未进入FR06。**
