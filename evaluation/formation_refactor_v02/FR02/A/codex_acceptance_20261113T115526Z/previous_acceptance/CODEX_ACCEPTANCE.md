# FR02-A 独立验收：需有限返工

日期：2026-11-13。验收人：Codex。范围仅为 **FR02-A**。结论：**CHANGES_REQUESTED**，暂不放行 FR02-B。

公共时间、初始/相邻设置调度、正常内部时刻解析求值与序列化往返已有实际支持；发现两个集中于新求值器的问题。只修这两个问题及定向测试，不改变 V0.2 路线、不重写 S8/S9、不扩大至 FR02-B。

## 1. 已核验的交付与通过项

【代码事实】工作区 `/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。共享文件只修改了任务允许的 `formation_geometry.py`、`formation_kinematics.py`、`formation_retiming.py`，合计27行新增、23行删除，均为零角S局部适配。新增两个时间/轨迹模块与两个测试文件，未开发B。

【代码事实】实际复用 S8 `_analyze_block`、高阶几何导数及 `mode_switch_event`；没有复制完整峰值求解系统。FR01源码/测试、冻结设计输入及 `CLAUDE_TASK_02.md` 哈希未变。修改前的共享文件副本与HEAD blob相同，暂存区无改动。

【实验事实】Codex独立复跑：

| 套件 | 结果 |
|---|---|
| `test_uniform_timing.py` | 5/5通过 |
| `test_uniform_trajectory.py` | 5/5通过 |
| FR01 `test_uniform_motion.py` | 13/13通过 |
| `test_formation_geometry.py` | 15/15通过 |
| `test_formation_kinematics.py` | 28/28通过 |
| `test_formation_retiming.py` | 14/14通过 |
| `test_formation_hold.py` | 14/14通过 |

共94项通过。实际包路径来自当前worktree。所有复跑JSON和artifacts写入独立Codex目录，没有覆盖Claude结果。

【实验事实】额外对直线、S和S→A弧→S→O手工场景的各相位共66个内部状态进行往返及刚性核对，均通过。正常S零角保持、共同停车和设置驻留、O横移带符号中心速度已由定向用例验证。上述通过项不消除下列边界/逐轮问题。

## 2. R1：合法近终点时间会触发进度越界（高优先级，阻塞）

【代码事实】`P/uniform_trajectory.py:791–807` 的 `_motion_state()` 只在时间**恰好等于**相位首末点时固定tau。内部五次多项式在接近1时存在舍入上溢，导致λ>L，随后被FR01的严格进度检查拒绝。P指 `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/`。

【实验事实】最简单2m直线，运动区间 `[0,4.710937499999999] s`：

```text
t = 4.7109327890624995 s  （严格在合法时间区间内）
state_at(t) -> PROGRESS_OUT_OF_RANGE
progress = 2.0000000000000009，L = 2
```

在直线、S和复杂场景各motion终点的前驱浮点数及相对终点距离10^-5至10^-15测试中，共29个合法时刻报错。详细输入和异常在独立probe JSON中。探针中因请求时刻舍入成终点的项已排除，不把这些项误报为内部失败。

【技术推断】这会使任意时刻解析求值、投影采样或未来播放在正常近终点时刻报错；违背A的核心合同。不是输入非法，也不应通过放宽FR01进度域来处理。

**最小返工：** 在新轨迹模块内部稳定求值五次进度与导数。例如用 `s(tau)=1-s(1-tau)` 的对称式处理后半段，导数使用稳定因式分解。输入t仍先严格检查，FR01保持只读。可处理已验证合法t在内部运算产生的舍入误差，不得把任意越界t吸附进区间或以较大ε提前结束运动。setup的进度求值若共享同一计算也应保持一致，不改只读的旧 `retiming.py`。

**验收：** 原A01–A10继续通过；直线、非零pivot弧、正/负S、非零t0的运动及序列化后模型，在首末点、两侧、终点前驱浮点数及近端相对间隔10^-5至10^-15处均能求值。有限、λ∈[0,L]、单调、几何误差/刚性与原阈值不变；真正越界时间仍拒绝。Python3.8可用struct位表示或现有NumPy取前驱浮点数，不假设有 `math.nextafter`。

## 3. R2：S逐轮有符号滚动速度与规范轮角不一致（中优先级，本轮修正）

【代码事实】`P/uniform_trajectory.py:839–844` 给全部S轮子使用 `wheel_factor * yaw_rate`。FR01的factor是**幅值**；规范S轮角折入[-π/2,π/2]时，部分轮子需反向滚动。新状态明确输出 `wheel_rolling_speeds_mps`，其中包含正负数，所以这些量不能按同符号幅值处理。

【实验事实】标准 `(π/2,0,-π/4)` S边中点，car1的 `omega=0.7960199004975126 rad/s`：

```text
第0轮位置 (x,y)=(0.247,0.182)m，规范轮角=-0.9357695914rad
代码输出滚动速度：+0.2442276750 m/s
由轮位置与自旋速度投影得到：-0.2442276750 m/s
```

car1、car3各有两轮方向错误，car2零角轮速正确为0。最大轮平面速度向量误差0.4884553501m/s。

【技术推断】目前车体解析轨迹及 `wheel_speed_mps=max(abs(...))` 的限值计算不受该符号错误影响；但权威状态中的逐轮数据自相矛盾，不能留给下一阶段作为正确轮系运动解释。此修复是新输出字段的几何一致性，不涉及控制器或实车。

**最小返工：** 对轮体坐标 `(x_w,y_w)`，计算车体自旋产生的轮平面速度 `(-omega*y_w, omega*x_w)`，在规范轮角 `delta_w` 上投影：

```text
u_w = (-omega*y_w)*cos(delta_w) + (omega*x_w)*sin(delta_w)
u_w * [cos(delta_w),sin(delta_w)] = [-omega*y_w,omega*x_w]
```

保留FR01规范转角和幅值因子，不改旧helper语义；轮速上限仍取绝对值。补充正/负自旋、折叠/未折叠轮、零角保持的四轮独立向量测试。向量残差≤1e-7m/s，零角车轮速为0；最大轮速与既有峰值限值一致，序列化前后相同。

## 4. 可直接交给Claude的返工范围

> 只返工FR02-A的R1/R2。读本报告、原 `docs/CLAUDE_TASK_02.md`，在现有V0.2 worktree完成新轨迹求值器数值稳定性与S有符号轮速修正。默认只改 `P/uniform_trajectory.py`、`tests/test_uniform_trajectory.py`；如需在已有 `tests/test_uniform_timing.py` 增加定向用例可以，但不删改原正例/负例和阈值。不要修改FR01、三个已通过局部适配的共享文件或其他旧模块。
>
> 先保存当前新文件与哈希，新增 `evaluation/formation_refactor_v02/FR02/A/rework_01/` 证据，保留本次所有源码/报告/输出及Codex证据。将Codex探针原样复制到新证据目录运行，不覆盖原probe JSON/log。原探针引用现有测试fixture；fixture名保持可用。
>
> A01–A10、FR01以及本报告列出的四个旧套件均须继续通过。增加R1的近端和前驱浮点数用例、R2逐轮独立向量用例；原probe中的29次合法时刻异常及4个S轮速不一致须全部消除，66个正常内部/往返状态继续通过。保留真实越界拒绝及原数学阈值。
>
> 输出完整diff、实际命令/环境/退出码/日志、机器结果及修复后的状态/误差；所有输出指向rework_01，不覆盖原结果。不得通过放宽FR01进度检查、轮速限制或关闭测试通过。完成后停止，由Codex再验收；不做FR02-B、碰撞/S9扩展、搜索或ROS，不提交Git。

## 5. 证据与本轮边界

证据目录 [codex_acceptance_20261113T105416Z](codex_acceptance_20261113T105416Z/)：基线核验、7套测试的命令/日志/JSON、独立 `independent_probes.py/json/log` 与源码/结果未修改的最终核验。

探针脚手架首次因一个“内部时间被舍入为精确终点”的断言而停止，该次输出另存 `independent_probe_harness_initial.log`；修正探针只筛选实际位于开区间的时间后，最终退出码1对应上述29个求值错误和4个轮速错误。未修改被验收代码或用例来生成这些失败。

【尚未验证】FR02-B/S9、碰撞、搜索、ROS/RViz均不在本次验收中；旧单车时间测试 `dynamic_peaks` 问题未重跑，未计通过。不要求本次返工顺带解决这些事项。

Codex本轮只做审查、定向运行和验收证据生成；未修改实现、测试、配置或历史实验，未提交Git。
