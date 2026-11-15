# FR02-B 独立验收：主体已验证，动态放行状态需局部修正

日期：2026-11-13。验收人：Codex。结论：**CHANGES_REQUESTED**。本轮只验收FR02-B，不进入FR03或ROS。

144项提交测试/回归全部通过，解析扫掠、几何/轮系篡改和投影一致性已有实际支持。**唯一算法阻塞是：动态尚未确认的结果仍通过FEASIBLE成功路径放行。** 不需要重构A，也不需要建立新的优化器或通用区间验证平台；下文给出现有数据即可完成的局部上界方案。

## 1. 基线、范围与真实复用

【代码事实】V0.2工作区、分支 `refactor/formation-v02` 和HEAD `6227833011e2d92b711179bfdec979bb0b878b29` 均保持。FR02-A验收仍为ACCEPTED；A的uniform_motion/timing/trajectory、三份共享适配源码、旧测试和冻结任务书逐字节未变。B执行前副本与其哈希清单核验一致，暂存区为空。

【代码事实】B新增 `P/uniform_quality.py` 与 `tests/test_uniform_quality.py`，修改任务允许的 `formation_collision.py`、`formation_quality.py`。P指 `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

【代码事实】碰撞复用原OBB/地图/车间核，通过新增解析回调取得真实姿态。审核复用S9独立动态核 `_dynamic_values`，以及S8分析/几何导数与现有数据类型。需如实说明：新 `_UniformAuditor` 约3248行，覆盖了自己的完整审核流程；继承旧 `_IndependentEvaluator` 不等于直接复用了旧完整审核入口。B新增的 `_check_block_kinematics(...,uniform_v02=...)` 参数目前未在新审核链调用。本轮不因行数要求整体重写，也不把该未调用扩展算作已验证的新链能力；后续不要继续复制平行基础设施。

## 2. Codex独立复跑结果

【实验事实】全部命令使用本worktree导入路径与独立结果目录，未覆盖Claude证据。

| 套件 | 通过 |
|---|---:|
| uniform_quality | 8 |
| formation_collision | 17 |
| formation_quality | 22 |
| formation_hold | 14 |
| formation_geometry | 15 |
| formation_kinematics | 28 |
| formation_retiming | 14 |
| FR01 uniform_motion | 13 |
| uniform_timing | 5 |
| uniform_trajectory | 8 |

合计144项、10条命令全部退出码0。B01–B08的PASSED为当前测试断言通过，不自动等于完整阶段验收。

【实验事实】Codex另补测了提交测试缺少的S自旋中途碰撞：地图分辨率0.05m，原点(-10,-10)，单占用cell(row=209,col=221)，其余为空。S边的起终OBB均安全；审核在car1、t=4.088966537921189s、相位进度0.4718309859处拒绝，返回 `FORMATION_SWEEP_COLLISION`。这证明当前实现已具备该能力，但应加入正式回归而非只依靠本次Codex探针。

【实验事实】同一复合模型，在每个S/A/O运动相位内部取0.371相对时间，把它同时加入0.02s与0.05s投影，两投影与解析状态完全一致。原B07的共同非网格时刻0.137/0.419/0.777s却全位于initial_setup，不能单独证明运动相位的跨采样率一致性；本次独立检查已补足功能证据，返工时补成持久测试即可。

## 3. B-R1：动态未确认却返回完整成功（阻塞）

### 表现与证据

【代码事实】`uniform_quality.py:_check_dynamic_peaks()` 在保守上界超过限值时仅设置 `dynamic_continuous_certificate=False`。`_uniform_success()` 随后仍返回 `FormationEvaluation(status=FEASIBLE,feasible=True)`、`certification=FEASIBLE`。原 `FormationEvaluation.executable` 直接由feasible决定；现有 `FormationPublicationGate.offer()` 也只依赖feasible，不检查这条诊断指标。

【实验事实】非零枢轴A弧和复合场景均复现：

```text
vehicle=3, constraint=acceleration
候选网格峰值=0.5796584150 m/s²
现有保守上界=0.7893281465 m/s²
限值=0.6 m/s²
dynamic_continuous_certificate=False
结果却为 FEASIBLE / feasible=True / executable=True
```

Codex在纯Python进程调用已有publication gate，它返回 `publish=True`。此调用只是结果对象语义核查，没有运行ROS或真实发布。

【技术判断】**这不证明轨迹实际超限。** 上界过松只意味着现判据尚不能确认。报告主动披露这一点是正确的；问题在于未确认状态仍被成功API放行。允许数值诊断并记录限制，与允许完整FEASIBLE证书是两件事。当前任务要求独立审核与安全轨迹一致，不能依赖下游主动读取一个可选metrics字段才避免误用。

### 最小解决方案：不新增算法平台

可以继续使用现有803点网格，但需给网格最大值加有根据的漏峰误差界，而不是仅继续加密网格。

对本次唯一上界未过的加速度，现有代码已提供全区间有效jerk上界J。若候选网格包含端点、最大相邻物理时间间隔为h，则：

```text
sup_t ||a(t)|| <= max_grid ||a(t_k)|| + J*h/2
```

依据是 `| ||a(t)||-||a(t_k)|| | <= ||a(t)-a(t_k)|| <= J|t-t_k|`，选最近网格点即可。可与原三角上界取较小值，前提是J确为同一相位全区间保守界。

【独立计算】当前A弧car3：

```text
J=1.9332470673 m/s³
h=0.0052998046875 s
修正上界=0.5796584150+J*h/2=0.5847813310 m/s² < 0.6
```

因此本例无需延长A轨迹、修改车辆限制或新建优化器即可获得足够紧的依据。计算见 `acceleration_bound_review.json`。具体实现可选等价的闭式极值或保守界，但仅覆盖现有原语和当前缺口，不建立通用平台。

**验收要求：** 所有返回完整FEASIBLE的结果，其采用的全区间动态判据均须完成；若仍不能确认则返回明确非放行的未确认结果，不能把“未确认”说成“已证实超限”，也不能继续 `feasible=True`。成功直线/自旋/正常非零pivot弧/复合场景保持可验证通过。新增测试明确断言完整成功的动态确认标记；注入仍未确认的情形时，不能进入成功或现有gate放行路径。对误差界保存端点覆盖、h、J、候选峰值、修正上界与限值，不只输出布尔标志。

## 4. 附带补齐的正式测试（不新增功能）

1. B06保留现有直线和A弧碰撞用例，补一条S自旋起终安全、中途车体碰撞用例。本次Codex地图可直接借鉴。
2. B07保留setup比较，但共同非网格时间还要包含S、非零枢轴A及O运动内部，断言相位类型确为motion，避免又只测静止驻留。保留弦插值拒绝。

这两项是任务书原有验收要求的回归覆盖补全，独立补测表明功能已存在，不要求重做碰撞或采样系统。

## 5. 可直接交给Claude的有限返工

> 只返工FR02-B的动态放行闭合和上述测试覆盖。默认只改 `P/uniform_quality.py`、`tests/test_uniform_quality.py`，新证据写入 `evaluation/formation_refactor_v02/FR02/B/rework_01/`。A已验收源码、FR01、三个旧共享补丁、旧测试/限制/历史证据均只读，不进入FR03/ROS。
>
> 使用现有候选网格和保守界补足当前加速度确认缺口，优先采用本报告给出的jerk误差界或等价局部方法。不新增优化器、区间平台或第二套时间规划；不改A的几何/时间模型来凑通过。未确认结果必须不放行，完整成功必须有相应动态依据。保留“未确认”和“实际超限”的区别。
>
> 新增对完整成功动态确认的断言与未确认不放行负例，补S中途碰撞以及运动内部跨采样率比较。不得只将原合法A弧的成功期望改为失败来消除问题，也不得放宽限值或把候选全过直接标成连续证书。
>
> 复跑本报告10个定向套件，保存原始输入、候选/保守/修正界及h/J、逐项结果、拒绝原因、真实命令/环境/退出码和完整diff。报告中按真实调用列出S8/S9/碰撞核复用，不把未调用的扩展写成已接入能力。无需为源文件行数整体重构。
>
> 保存当前源码副本/哈希及旧证据。Codex本轮探针复制到新目录后运行，不覆盖本次输出；它会检查两例未确认FEASIBLE是否仍存在，以及Spin碰撞和运动采样一致性。完成后停止交Codex验收，不提交Git、不继续下一阶段。

## 6. 证据和验收边界

本轮证据：[codex_acceptance_20261113T144933Z](codex_acceptance_20261113T144933Z/)。含基线/提交清单核验、10套件实际日志和机器结果、独立probe源码与JSON、加速度误差界复算、最终文件未修改核验。

独立probe退出码1仅表示发现两条“动态尚未确认却FEASIBLE”的结果；Spin碰撞与运动投影检查通过。原动态限制已主动记录，不因报告诚实披露而判为实际物理失败。

Codex本轮未修改任何实现/测试，未运行ROS、未提交Git。FR02-A仍保持ACCEPTED；FR02-B本轮暂不通过。旧单车dynamic_peaks问题未重跑，未计通过，不追加为返工任务。
