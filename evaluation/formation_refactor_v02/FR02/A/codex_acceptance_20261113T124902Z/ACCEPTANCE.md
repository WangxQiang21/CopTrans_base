# FR02-A rework_02 独立验收：ACCEPTED

日期：2026-11-13。验收人：Codex。本轮仅复验FR02-A剩余R3及回归。**R1/R2/R3全部关闭，FR02-A达到任务合同，无剩余阻塞项。** 本结论可作为FR02-B前置依据，不授权自动开始B。

## 1. 实际对象和范围

工作区 `/home/wq/whut_thesis/expv3_multi_v02`，分支 `refactor/formation-v02`，HEAD `6227833011e2d92b711179bfdec979bb0b878b29`。

本轮源码指纹：

- `uniform_trajectory.py`：`a93c2b2faba8bda7ac4592888cd502ea3850e6138505df851a12f44a5e86b579`。
- `tests/test_uniform_trajectory.py`：`713374e57559a1388d5940bf1fcc1aff6e255dd32644a6cede293272f08a03c2`。

【代码事实】相对上轮仅修改上述两个文件；`uniform_timing.py`、三个旧共享文件的局部补丁、FR01源码/测试与冻结任务书均未变化。测试AST比较确认原A用例和R1/R2方法未改，新增R3方法。原源码/测试的返工前副本与上轮Codex哈希相同，提交证据清单逐项核验一致。

## 2. R3复核：已关闭

【代码事实】`uniform_trajectory.py:_validate_plan_structure()` 对持久化时轴要求首t0=0、相邻t0严格等于前end、最后end严格等于总时长。真实gap/overlap不再因为小于1e-9被接受；几何误差阈值未改。解码不挪动或重新定时输入相位。

【代码事实】每个正时长相位还要求有限且end>t0，拒绝浮点尺度下坍缩的区间。motion时钟和setup转向时钟使用明确数值支持域 `[1e-100,1e100] s`；这是求值器算术范围，不是修改车辆动态限值。无相位的零时长hold保持合法。正常S8输出时长远在此范围内。

【实验事实】Codex原解码探针逐字节复制并实际运行，两个输入现在均在`from_model_dict()`阶段拒绝：

| 原反例 | 本轮结果 |
|---|---|
| 相邻motion后移5e-10秒的真实缝隙 | `PHASE_TIME_DISCONTINUITY` |
| t0≈2.3432、duration=1e-20导致end==t0 | `PHASE_TIME_UNREPRESENTABLE` |

独立探针退出码0，不再延迟为state_at失败或错误终端。R3新增定向测试还覆盖ULP尺度/旧容差尺度的缝隙和重叠、总时长不匹配、数值支持边界、正常往返、hold及终端到达。

## 3. 回归与原R1/R2

【实验事实】Codex本轮实际运行：

| 套件 | 通过 |
|---|---:|
| uniform_timing | 5 |
| uniform_trajectory | 8 |
| FR01 uniform_motion | 13 |
| formation_geometry | 15 |
| formation_kinematics | 28 |
| formation_retiming | 14 |
| formation_hold | 14 |

合计97项全部通过。另原样运行两个Codex探针，全部9条命令退出码0。

【实验事实】原R1/R2独立探针：66个正常内部/往返状态、71个边界/近端状态、12个S轮平面速度向量全部通过；原29次近端求值异常与4个S轮速错误没有回退。真正越界拒绝、公共设置与零角S轮系、O横移速度语义仍通过原定向测试。

所有运行的结果/日志写入本目录，FR01_RESULTS_PATH及FR02输出路径均独立设置，没有覆盖Claude或上轮Codex结果。详见 `test_runs.json`、各套件log、`independent_probes.json`、`decoder_time_probes.json`、复跑artifacts。

## 4. 保护及证据

【实验事实】最终再次核对，被验收源码和Claude的rework_02全部证据在Codex运行前后哈希一致；暂存区为空，三个共享文件补丁未变。原仓库tracked文件、其余原始untracked文件和T5历史含忽略日志保持完整，T5 binary diff与保护快照逐字节相同。

- `baseline_verification.json`：基线、原测试方法与变更边界。
- `test_runs.json`：实际命令、输出路径、环境覆盖与退出码。
- `independent_probes.py/json/log`：R1/R2原样探针。
- `decoder_time_probes.py/json/log` 与输入模型：R3原样探针。
- `final_verification.json`：验收结束一致性检查。
- `previous_acceptance/`：上一轮未通过报告/JSON逐字节归档与哈希。

## 5. 验收边界与停止

FR02-A已交付公共时间、initial/setup/motion相位、可任意时刻求值的解析模型、规范轮系输出、版本化往返和采样投影。

【尚未验证】FR02-B完整独立S9审核、真实解析扫掠碰撞、采样/模型篡改安全证书尚未交付；本次ACCEPTED不将 `GEOMETRY_ONLY_NO_S9_NO_COLLISION` 升格为碰撞通过或可发布。旧单车时间套件的dynamic_peaks问题未重跑，未计为通过。搜索、ROS/RViz也未在本阶段验收。

本轮只新增验收材料并更新当前结论；没有修改算法/测试，没有提交Git，没有开始FR02-B。等待用户交付下一验收点。
