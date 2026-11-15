# FR02-B rework_01 独立验收：ACCEPTED

日期：2026-11-13。验收人：Codex。**B-R1已关闭，要求补齐的正式测试已完成，FR02-B无剩余阻塞项。FR02-A保持ACCEPTED。** 本次不授权或开始FR03。

## 1. 变更边界与已验收基线

工作区 `/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。

【代码事实】本返工仅修改 `P/uniform_quality.py`、`tests/test_uniform_quality.py`（P为optimizer包源码）。`formation_collision.py`、`formation_quality.py`的B阶段改动未变化；A的源码/测试、FR01、三个旧共享适配、冻结任务书均保持。返工前副本与上轮Codex源指纹相同，全部提交证据清单核验一致。

当前全部B相关源码指纹见 `baseline_verification.json` 和根 `CODEX_ACCEPTANCE.json`。新审核器复用已有S9动态核、候选集、S8分析与解析碰撞内核，此次没有增加时间规划器、优化器或通用区间平台。此前未调用的S9零角参数扩展未被当作新链已验证能力，报告已如实注明。

## 2. 动态放行闭合：已通过

【代码事实】候选峰值现在与同相位导数保守上界构成漏峰误差界，所采用的上界为原解析界与修正界中的较小者。速度/加速度、yaw和轮速等使用相应已知导数界；jerk缺少snap上界时保留原解析界，不捏造导数证据。候选网格保持803点，包含端点；记录最大相邻物理时间间隔h、J、误差项、采用界与限值。

```text
sup ||q(t)|| <= max_grid ||q(t_k)|| + J*h/2
+```

【实验事实】Codex未仅复用待测审核器的确认布尔值，而按A圆弧的独立闭式重新计算car3加速度候选：`rho*sqrt(lambda_ddot²+lambda_dot⁴)`，并从rho、L、T与五次导数上界重新算J和h：

```text
rho = 3 m
candidate_acceleration = 0.5796584150289499 m/s²
J = 1.9332470672747744 m/s³
h = 0.005299804687500357 s
candidate + J*h/2 = 0.5847813309635694 m/s² < 0.6 m/s²
```

该结果与审核输出一致；A弧和复合场景均保持FEASIBLE，现具有完成的动态确认，而不是把原合法场景改成失败来“通过”。

【代码事实/实验事实】所有完整成功均断言全区间采用判据已完成；若未确认，返回 `UNIFORM_DYNAMIC_UNCONFIRMED`、`feasible=False`、`executable=False`。成功出口再次检查完成标志。没有修改publication gate。

Codex另创建几何与轮系不变、边和时间配置一致的更严格 `jerk_max=1.5` 负例：候选jerk约1.4170小于限值，保守界约1.9332仍超过限值。结果明确为未确认而非已证实物理超限，gate返回 `publish=False`。因此未确认不能再走完整成功路径。详见 `confirmation_probe.py/json/log`。

## 3. 独立运行与覆盖

【实验事实】本轮实际运行10个定向套件，全部退出码0：

| 套件 | 通过数 |
|---|---:|
| uniform_quality（B01–B09） | 9 |
| formation_collision | 17 |
| formation_quality | 22 |
| formation_hold | 14 |
| formation_geometry | 15 |
| formation_kinematics | 28 |
| formation_retiming | 14 |
| FR01 uniform_motion | 13 |
| uniform_timing | 5 |
| uniform_trajectory | 8 |

共145项。额外原样复跑上轮Codex探针，straight/arc/complex全部成功且动态确认完成，没有未确认FEASIBLE；S起终安全但中途碰撞仍拒绝，四个S/A/S/O运动内部的跨采样率状态仍一致。

【代码事实/实验事实】正式B06新增S中途碰撞，B07新增运动相位内部非网格时间并断言phase_kind/mode；B09检查正例误差界和未确认不放行负例。原直线/弧碰撞、弦插值拒绝及旧审核测试均保持通过。

自旋碰撞证据沿用单cell(row209,col221)、0.05m分辨率和(-10,-10)原点：起终OBB安全，car1在t≈4.08897s的运动中途被检测为碰撞。

## 4. 文件和证据保护

【实验事实】Codex运行前后所有被验收源码和Claude返工证据哈希相同；全部tracked补丁保持原样、索引为空。原仓库文件及T5含忽略日志的历史记录与保护清单一致，T5 binary diff逐字节相同。

本目录保存 `baseline_verification.json`、`test_runs.json`、10套件日志及复跑机器结果、`independent_probes.py/json/log`、`confirmation_probe.py/json/log`、`final_verification.json`。全部结果路径独立于Claude原输出。

辅助确认探针首次把边的jerk限值保留为2、仅将时间限值改1.5，审核正确先拒绝配置不一致；修正探针为边/时间一致后，复跑证实上述未确认负例。初次脚手架输出单独保留为 `confirmation_probe_harness_initial.log`，没有删改待测代码。

上一轮未通过报告和JSON逐字节归档于 `previous_acceptance/`；根记录更新为本次ACCEPTED。

## 5. 完成能力与停止范围

FR02-A/B现已完成：统一模式边的公共时间与初始/相邻轮系设置、权威解析求值/序列化、独立数学和动态确认、模型/投影一致性及静态地图上的解析保守扫掠审核。无法确认的动态输入明确不放行；审核通过关联具体模型、地图、形状、限值和误差标准，不表示任意输入都可解。

【尚未实施】自主搜索FR03/后续阶段、ROS/RViz接入、连续优化均未在本轮开展；仍不涉及闭环控制、Gazebo或实车。旧单车dynamic_peaks失败未重跑，不计通过，也不是本阶段新增返工项。

Codex未修改实现或测试、未提交Git、未启动ROS。FR02-B验收结束，停止等待下一阶段指令。
