# FR02-A 返工复验：R1/R2关闭，尚有解码时轴阻塞

日期：2026-11-13。验收人：Codex。范围：FR02-A / rework_01。

**本次指定的R1/R2修复验收通过；FR02-A整体仍为CHANGES_REQUESTED，唯一剩余阻塞为R3解码时轴有效性。暂不放行FR02-B。** R3由Claude本次报告主动披露，Codex独立复现；不是本轮返工引入的问题，也不是要求开发FR02-B动态审核。

## 1. 已通过的复核

【代码事实】工作区、分支和HEAD仍为既定V0.2基线。此次只修改 `P/uniform_trajectory.py` 和 `tests/test_uniform_trajectory.py`（P指包源码目录）。FR01两个文件、`uniform_timing.py`、三个共享文件的前轮补丁、冻结任务书均未变化。用AST比较确认原有测试方法与阈值未改；原证据未被覆盖。

【代码事实】R1使用后半段对称五次式和稳定因式分解导数；设置相位同样复用新求值。输入t仍先严格检查，没有放宽FR01进度域。R2按规范折叠轮角对轮平面速度投影，保留幅值因子及最大绝对轮速语义，A/O分支未改。

【实验事实】Codex重新运行7个套件，全部退出码0：

| 套件 | 通过数量 |
|---|---:|
| uniform_timing | 5 |
| uniform_trajectory（含R1/R2） | 7 |
| FR01 uniform_motion | 13 |
| formation_geometry | 15 |
| formation_kinematics | 28 |
| formation_retiming | 14 |
| formation_hold | 14 |

合计96项。输出写入本Codex目录，与Claude结果分开。

【实验事实】上轮Codex探针逐字节复制后复跑，退出码0：66个内部/往返状态、71个边界/近端状态、12个S逐轮向量全部通过。原29次合法近终点异常和4个滚动速度错误均消除。R1/R2新增测试还覆盖正负S、零角车、精确有理时间参考及真正越界拒绝。

## 2. R3：解码器接受了无法完整求值的时轴

### 2.1 小于容差的真实缝隙仍然是无定义区间

【代码事实】`P/uniform_trajectory.py:_validate_plan_structure` 使用 `abs(phase.t0_s-cursor)<=1e-9` 接受时轴接续；`_phase_at` 却按实际半开区间查找，不覆盖被接受的缝隙。

【实验事实】用合法S模型，仅将motion的起点后移5e-10秒，同时缩短其duration、保持原末点及总时长。`from_model_dict()`接受。随后：

```text
前相位end = 2.3432099798544908 s
后相位t0  = 2.3432099803544910 s
缝隙内部t = 2.3432099801044908 s
state_at(t) -> PHASE_TIME_DISCONTINUITY
```

该t在解码模型声明的[0,T_end]内。此问题不同于已经修好的五次式舍入上溢：模型的相位覆盖本身存在缺口。

### 2.2 存储duration为正，但浮点区间已坍缩

【实验事实】同一模型将motion的duration设为 `1e-20 s`，端点和总时长按实值计算。解码器接受：

```text
motion.t0 == motion.end_time == 2.3432099798544908 s
state_at(T_end).progress == 0
实际边L == 1.5707963267948966 rad
```

duration是正数，但不能在该t0的浮点尺度下表达运动区间，终点返回了起点。该时长当然不物理可行，物理时长审核属于B；A仍必须在解码时拒绝不可表示的结构，不能返回一个连终端几何都错误的解析对象。

### 为什么仍属于A

【合同事实】`CLAUDE_TASK_02.md` 的A10要求时间缝隙/重叠明确拒绝，3.3要求合法模型对[0,T_end]任意有限时间可求值。数值比较容差不能变成允许存在真实无定义区间。这里不要求在A执行速度/加速度等动态可行性审核；正但过短、尚可数值表示和求值的时长仍可留给B拒绝。

## 3. 最小修复与验收要求（仅R3）

优先在解码/结构校验集中处理：

1. 对正时长phase要求有限且 `end_time>t0`，明确拒绝浮点坍缩区间。
2. 不接受实际相位缝隙或重叠。最小方案是对版本化持久化时轴要求相邻边界严格一致，仍保留原几何比较容差。当前调度器使用上一end作为下一t0，JSON往返可保留同一double，不需重新定时。若选择其他方案，必须说明其保证无缝完整覆盖的机制，不能仅改查找器把缺口归给前/后相位或偷偷移动输入时间。
3. 验证最末边界与总时长一致、全相位覆盖[0,T_end]；两例都须在解码阶段以结构化原因拒绝，不能延迟为求值异常/错误终端。

允许仅修改 `P/uniform_trajectory.py` 和 `tests/test_uniform_trajectory.py`，新证据放 `FR02/A/rework_02/`。不修改FR01、S8、共享补丁、原数学阈值，不开发B，也不以本次问题引入完整新审核器。保留R1/R2修复及所有历史记录。

测试必须包含小于/等于原1e-9容差的正缝隙和负重叠、坍缩时间区间、总时长与末端不一致、合法零运动hold、正常模型严格往返与终端到达；合法模型仍通过原A01–A10、R1/R2、上轮探针和本轮96项套件。物理可行时长由S8产生的模型不应受到影响。

若采用严格拒绝规则，用新定向反例证明；不得把模型静默重新定时，不得靠缩小测试中的缝隙或隐藏失败通过。报告中的更极端duration导致算术下溢/溢出问题，应明确数值支持域，无法表达/稳定求值的输入在入口拒绝；不要求把物理动态审核前移至A。

**交给Claude：只补齐上述R3解码时轴检查和定向测试，完成后停止待Codex复验，不进入FR02-B、不提交Git。**

## 4. 独立证据和历史

本目录保存：`baseline_verification.json`、`test_runs.json`、各套件日志/机器结果、原样 `independent_probes.py/json/log`，以及新增 `decoder_time_probes.py/json/log` 和两个完整输入模型。新解码探针退出码1，对应上述两个已确认缺陷。它不调用S9或碰撞模块。

复现：在V0.2目录以 `OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 evaluation/formation_refactor_v02/FR02/A/codex_acceptance_20261113T115526Z/decoder_time_probes.py` 运行。返工时复制到新证据目录，保留本次失败输出。

此前未通过报告/JSON已逐字节保存在 `previous_acceptance/`；当前根验收记录更新为本次结论。Codex未修改实现或测试，没有提交Git。FR02-B/S9/碰撞/ROS及旧dynamic_peaks问题均不计为本轮已验证。
