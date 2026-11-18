# Claude C2-B 返工 01：修正 clearance 世界坐标并补齐弯曲正例

## 结论、工作区与停止边界

Codex 对 C2-B 的首次独立验收结论为 **FAIL**。走廊半空间、双 solver、一致残差、连续包含审核、三车 OBB 终审和失败禁止发布均已通过定向核验；当前只返修以下两个问题：

1. clearance 代理把起点局部坐标直接用于世界地图查询；
2. 原 C2-B 报告把曲率近零的 door 直线路径称为“非平凡走廊解”，没有提供任务书要求的自主弯曲正例。

工作区固定为 `/home/wq/whut_thesis/expv3_multi_virtual_a`，分支 `feature/virtual-rigid-formation-a`，审核时 HEAD 为 `9a19fd3e22d33188b13795f695757c3e4a36909f`。当前工作区同时包含未提交的 C2-A、C2-B、V0/V1/Cargo HUD 改动。不得 reset、checkout、覆盖、清理或重新生成这些既有改动；开始前记录 HEAD、`git status --short` 和本任务涉及文件的 SHA-256。

本返工不进入 C3，不提高六个冻结窄道场景的成功率，不修改走廊构造、固定分配、连续包含、碰撞审核、车辆限制、时间律、ROS/RViz 状态机或旧 `legacy_route` 行为。不提交、不推送、不建分支、不启动 ROS/RViz。完成后停止，等待 Codex 复验。

保留原证据目录：

`evaluation/virtual_a_corridor/C2B/20261117T135647Z/`

不得修改其中任何文件。新证据写入：

`evaluation/virtual_a_corridor/C2B_REWORK_01/<唯一 UTC run-id>/`

## 已确认问题与复现证据

优化变量给出的编队参考位置和姿态是起点局部量：

\[
p_F^L=(x,y),\qquad \psi_F^L=\psi.
\]

clearance field 来自原始栅格地图，查询参数必须是世界坐标。当前以下三个执行路径却直接用局部 `x,y,psi` 计算车辆中心并查询世界 field：

- `optimizer.py::_clearance_proxy_angles`；
- `optimizer.py::_proxy_numeric_penalty`；
- `optimizer.py::_build_casadi_nlp` 中的 clearance 目标。

Codex 用起点 `(3,-2,0.7)` 和一个局部零状态独立复现：实际 field 查询车心为 `(1,0)/(0,1)/(0,-1)`，正确世界车心为 `(3.764842,-1.355782)/(2.355782,-1.235158)/(3.644218,-2.764842)`，最大误差 `4.04907279485009 m`。

最终 OBB 审核仍会失败关闭，因此这不是已确认的误发布；但错误 clearance 梯度会改变优化结果，不能带入 C3 成功率与恢复研究。

## 唯一允许的生产代码修改

允许修改：

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/optimizer.py`

允许修改测试：

- `tests/test_virtual_a_corridor_optimizer.py`

允许在新的返工证据目录新增只读运行脚本、JSON、日志和报告。除非遇到可复现且无法绕开的接口冲突，禁止修改 `contracts.py`、`corridor.py`、`pipeline.py`、`geometry.py`、`trajectory.py`、`timing.py`、`audit.py`、`collision.py`、ROS 节点、launch、RViz 和其他测试。若确有接口冲突，停止并报告，不自行扩大范围。

不得新增通用 wrapper、adapter、manager、factory 或平行坐标系统。优先在现有三个 clearance 计算入口中用一个简单、可测试的局部到世界变换；若提取一个小型内部辅助函数，必须同时供 NumPy 初始化/目标复用，并保持 CasADi 表达式使用完全相同的公式。

## 数学合同

令请求起点为：

\[
p_0=(x_0,y_0),\qquad \psi_0.
\]

优化器局部状态转换为世界状态：

\[
p_F^W=p_0+R(\psi_0)p_F^L,
\qquad
\psi_F^W=\psi_0+\psi_F^L.
\]

对车辆偏移 `r_i=(a_i,b_i)`：

\[
p_i^W=p_F^W+R(\psi_F^W)r_i,
\]

\[
\theta_i^W=\psi_F^W+
\operatorname{atan2}(a_i\kappa,1-b_i\kappa).
\]

必须使用 `p_i^W` 查询 `field.direction_at` / `field.value_at` / CasADi interpolant；障碍方向属于世界坐标，支撑距离使用 `theta_i^W-direction_i^W`：

\[
support_i=\frac{L_i}{2}|\cos(\theta_i^W-d_i^W)|+
\frac{W_i}{2}|\sin(\theta_i^W-d_i^W)|.
\]

clearance 代价保持原合同，不改权重、target、field、clamp 或归一化：

\[
J_{clear}=\operatorname{mean}
\left(\frac{\max(0,target+support-clearance)^2}{L_0^2}\right).
\]

世界变换必须同时用于：

1. 每个初值冻结最近障碍方向；
2. NumPy/SLSQP 数值目标；
3. CasADi/IPOPT 符号目标。

走廊半空间已有独立的正确世界到局部变换，不得改动。曲率 `kappa`、偏移 `a,b` 仍定义在编队局部切向坐标中，刚体旋转不会改变其数值。

## 实施步骤

### R1：修复初始化方向查询

修改 `_clearance_proxy_angles`，将每个局部参考状态转换为世界参考状态，再计算三车世界中心、世界车体航向和障碍方向。不得通过假设起点为 `(0,0,0)` 规避变换。

### R2：修复两个目标实现

在 `_proxy_numeric_penalty` 中应用相同变换。在 `_build_casadi_nlp` clearance 分支中用 CasADi 标量表达同一变换，之后再执行既有地图边界 clamp 和插值。不得只修 NumPy 或只修 CasADi。

### R3：增加高价值定向测试

至少新增以下测试，避免大量重复用例：

1. **世界车心手算测试**：请求起点至少包含非零平移和非零旋转，例如 `(3,-2,0.7)`；对局部零状态和三车偏移，独立手算 field 查询点。三车坐标误差均 `<=1e-12 m`，障碍方向和世界车辆航向语义一致。
2. **NumPy/CasADi 目标一致性**：使用非零平移、非零起始姿态、非对称 clearance field 和非零曲率决策向量，分别计算数值目标与 CasADi `f`，绝对或相对误差 `<=1e-9`。不能只使用常量 field、原点起点或零曲率，因为这些输入无法暴露坐标错误。
3. **等变或独立公式检查**：对一个平移旋转后的等价请求/地图，手工世界公式与实现一致；至少断言错误的“局部坐标直接查询”会产生明显不同结果，防止测试同时复制同一个错误。
4. **原点回归**：起点 `(0,0,0)` 的既有 clearance 行为和旧 `legacy_route` 测试保持不变。

不得修改旧测试期望、降低容差、关闭 clearance 或将权重设为零来通过测试。

### R4：补一条真正弯曲的端到端证据

新增一个独立证据用例，不必加入长期单元测试。建议使用足够大的空旷栅格，局部任务为：

- 起点世界位姿含非零平移和旋转；
- 局部目标相对起点约 `(8 m, 2 m, 0 rad)`，转换后形成世界目标；
- 三车初始航向均等于起点编队姿态；
- `guidance_mode=corridor`，其余生产车辆、时间、动态、碰撞与审核合同不变。

该请求必须自主经过 `plan_virtual_a`，不得直接注入人工路径或手工构造 solver 成功结果。保存完整请求、地图摘要、PlannerConfig、guide/chain/allocation、全部 attempts、连续包含、三层审核、路径长度、曲率峰值、刚性误差、端点误差、上下文/模型哈希和墙钟时间。

验收门：

- `status=SUCCESS`、`feasible=true`、`executable=true`；
- path 和公共时间 plan 均存在；
- `CORRIDOR_CONTAINMENT_VERIFIED`；
- geometry、dynamics、collision 三层均通过；
- 参考曲率峰值 `>=0.05 m^-1`，证明不是近直线数值噪声；
- 端点位置误差 `<=0.005 m`；
- 端点姿态误差 `<=0.2°`；
- 刚性误差 `<=1e-6 m`；
- 规划失败时仍必须无 path、无 plan，不能为了正例关闭审核。

若该建议场景因确定的数值原因不收敛，可以使用同等难度的平滑横向位移场景，但必须保持非零世界平移/旋转、明显非零曲率和三层通过。不得回退到 door 直线或 G1 人工曲率路径冒充自主正例。

### R5：修正证据表述

不要改写旧 `C2B/20261117T135647Z` 历史文件。在新返工 `SUMMARY.md` 中明确：

- 原 door corridor 是有效的直线/近直线正例，不再称为“非平凡弯曲解”；
- 新增的横向位移请求才是 C2-B 自主弯曲正例；
- 六冻结 corridor 场景仍为原报告的 `0/6`，本返工没有声称改善；
- clearance 坐标错误的影响是优化能力，不是已证实的安全漏检；
- `false_publications` 保持为 0。

## 必须运行的验证

从仓库根目录设置：

```bash
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export PYTHONPATH="$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PWD/tests:$PWD:${PYTHONPATH:-}"
```

运行：

```bash
python3 -B -m unittest discover -s tests -p 'test_virtual_a_corridor_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_optimizer.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_trajectory.py' -v
python3 -B -m unittest discover -s tests -p 'test_virtual_a_pipeline.py' -v
```

只跑定向测试，不运行会覆盖历史输出的全量 runner。`test_virtual_a_pipeline.py` 必须继续证明 legacy 六冻结场景无回归。若只修改 `optimizer.py` 和本阶段测试，无需重复 ROS/RViz 真实启动或 19 个 C2-B 长耗时矩阵；只需运行一条新的弯曲端到端证据。若实际修改了其他文件，则按影响补跑对应测试并说明原因。

## 交付物与证据

在新目录保存：

- `SUMMARY.md`：问题、公式、改动、修正后的表述、测试和限制；
- `results.json`：测试退出码、世界坐标手算误差、NumPy/CasADi 目标误差、弯曲正例量化结果和 `false_publications`；
- `commands.log`：实际命令、开始结束时间、退出码；
- `source.diff`：只包含本次返工增量，不混入 C2-A/C2-B/V0/V1；
- `source_hashes.txt`：最终源码、测试、请求、地图及结果哈希；
- 弯曲用例完整 JSON；
- 开始和结束的 `git status --short`、HEAD、相关文件哈希；
- 原 C2-B 证据目录未改变的核验记录。

报告必须区分：求解器成功、连续走廊包含、几何审核、动态审核、碰撞审核和发布资格。测试未运行就标 `NOT_RUN`，不得用旧日志冒充本次结果。

## 禁止事项

- 不修改或放宽车辆曲率、速度、加速度、jerk、轮系或碰撞限制；
- 不降低连续包含、几何、动态或 OBB 审核门槛；
- 不关闭 clearance 代价或把权重设为零掩盖错误；
- 不修改 C1 路线简化、走廊几何或 C2-A 分配；
- 不修复六冻结场景 `0/6`，不进入 C3 恢复策略；
- 不修改旧 C2-B 证据，不覆盖历史实验；
- 不提交、不推送、不启动下一阶段。

## Claude 完成后必须停止并汇报

汇报以下内容后停止：

1. 实际修改文件和逐文件改动摘要；
2. 三处 clearance 路径采用的统一世界坐标公式；
3. 世界车心最大手算误差和 NumPy/CasADi 目标差；
4. 弯曲端到端正例的曲率峰值、端点误差、刚性误差、三层审核及连续包含结果；
5. 四条测试命令的真实数量、耗时和退出码；
6. 新证据目录、HEAD、Git 状态、未提交/未推送声明；
7. 尚未解决的问题和所有 `NOT_RUN`。

不得自行声称 Codex `ACCEPTED`，不得自动进入 C3。

## Codex 复验门

Codex 将独立检查：

1. 三个 clearance 执行路径是否都使用同一世界坐标语义；
2. 数值目标和 CasADi 目标在非零起点、非对称 field、非零曲率下是否一致；
3. 错误局部坐标负例是否确实能被测试识别；
4. 新正例是否由真实 pipeline 自主生成且曲率峰值 `>=0.05 m^-1`；
5. 连续包含和三层审核是否全部通过，失败结果是否仍无 path/plan；
6. legacy 六场景、旧模型/context 及历史证据是否未被破坏。

只有以上全部满足，C2-B 才可改判 `ACCEPTED` 并进入 C3。
