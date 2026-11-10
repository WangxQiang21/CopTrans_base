# S1 阶段审查报告：FormationModel 与严格输入校验

## 结论

**S1 完成：新增纯 Python 刚性编队几何模型与 26 项定向测试，全部通过；完整 Python 回归仍只有 S0 已记录的同一个 `dynamic_peaks` 错误，没有新增失败。**本阶段未进入地图搜索、未实现多车碰撞检查、未创建 ROS 消息或时间轨迹，也没有发布任何多车 `FEASIBLE` 结论。

## 任务编号

S1：`FormationModel` 与严格输入校验。实现依据为 [实施文档](../../多车4WIS刚性编队协同规划_详细技术实施文档.md#L438-L496)。

## Git 检查点

- S0 已完成提交 SHA：`1610268e9850a0d4fb164a7ad9729fa5d80405c1`
- S0 基线分支：`baseline/single-v2`
- S1 分支：`feat/formation-model`
- S1 提交 SHA：以本阶段提交后的实际 `git rev-parse HEAD` 和交付回复为准
- 远程推送：未执行
- 两个原有未跟踪 Markdown 文件仍未清理、覆盖或提交

## 核实结果

| 项目 | 核实结果 |
|---|---|
| [core.py:33-38](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/core.py#L33-L38) | 现有单车位姿是 `Pose(x, y, th)`，没有 `psi` 字段 |
| 航向命名处理 | 新增 `FormationPose.from_core()` 显式读取 `th` 并映射到 `psi`；未修改 `core.Pose` |
| 依赖边界 | 新模块只使用 Python 标准库；未 import ROS、NumPy、SciPy、CasADi 或单车地图代码 |
| 导入验证 | `python3 -I` 下导入成功，退出码 0；catkin devel 环境也可导入 |
| 旧测试起点 | S1 修改前 `test_core.py` 为 31/31 通过，退出码 0 |

## 修改文件

仅新增本阶段文件：

1. [formation.py](../../catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation.py)：数据类、刚性中心映射、反求与校验。
2. [test_formation_model.py](../../tests/test_formation_model.py)：26 项纯几何定向测试。
3. [logs/formation_model_tests.txt](logs/formation_model_tests.txt)：定向测试完整输出与资源统计。
4. [logs/unit_tests.txt](logs/unit_tests.txt)：完整 Python 回归真实输出与资源统计。
5. [logs/standalone_import.txt](logs/standalone_import.txt)：隔离导入结果。
6. [logs/catkin_build.txt](logs/catkin_build.txt)：catkin 构建真实输出与资源统计。
7. [S1_REPORT.md](S1_REPORT.md)：本报告。

没有修改单车算法、消息、YAML、launch、CMake、C++ 或既有测试。

## 实现的几何合同

### 数据与输入

- `VehicleOffset(vehicle_id, x, y)`：编队坐标系下的车辆中心偏移。
- `FormationPose(x, y, psi)`：编队参考位姿，`psi` 是编队参考方位，不是任何车辆的车体航向。
- `VehiclePose(vehicle_id, x, y, theta)`：车辆中心与**独立输入**的车体航向。
- 所有坐标和角度强制为有限值；`psi` 允许展开，不做预先 wrap。
- `vehicle_id` 必须是不可为 `bool` 的 `int`，范围 `0..2^32-1`。
- `FormationModel` 拒绝空队形、重复 ID、重复或完全重合的中心偏移，并按 `vehicle_id` 稳定排序。
- 重合偏移检查覆盖任意一对车辆，不局限于排序后的相邻 ID。

### 正变换

对每一个 ID 计算：

```text
p_i = q_F.位置 + R(q_F.psi) r_i
```

`centers(qf)` 只返回车辆中心。它没有 `body_headings` 参数，因此不可能隐式选择或写入车体航向。

`vehicle_poses(qf, body_headings)` 才将刚体中心与独立车体航向组合。`body_headings` 必须是精确 ID 集合到有限角度的映射；缺车、额外车辆或不可用值立即失败。输出顺序与排序后的 `offsets` 一致。

### 参考距离、残差与完整校验

- `pairwise_reference_distances()` 返回按 ID 对升序排列的参考距离副本，调用者不能改写模型内部参考值。
- `rigidity_residual(centers)` 先严格检查 ID 集合和所有中心有限性，再返回任意车辆对距离的最大绝对误差；N=1 时为 0。
- `validate_vehicle_centers(qf, centers, tolerance_m)` 同时检查：
  1. 车辆 ID 精确完整，重复、缺失、额外均失败；
  2. 所有两两距离相对参考队形的残差；
  3. 每辆车相对由同一个 `q_F` 重建中心的残差。
- 第三项是必要的：仅检查两两距离不能识别反射；当等边队的车辆标签互换时，两两距离集合甚至可能完全不变，而逐 ID 重建仍能识别换标。
- 输入中心支持两种形式：`vehicle_id -> (x, y)` 映射，或带 `vehicle_id` 的 `VehiclePose` 序列。序列在转换为字典前先检查重复 ID，避免重复信息被静默覆盖。
- 设计骨架中的 `validate_centers()` 与逐项清单中的 `validate_vehicle_centers()` 均已提供，后者是主实现，前者是签名一致的兼容入口。
- 容差必须是有限且非负；默认 `1e-6` m，纯浮点数学测试使用 `1e-10` m。

### 可选位姿反求

`fit_formation_pose(centers, psi=None, tolerance_m=1e-6)`：

- 使用二维相对 Procrustes 最小二乘求解平移与旋转，旋转矩阵行列式固定为 +1，不使用反射解。
- 若未显式给出 `psi` 且方位可观测，则用 `atan2(cross, dot)` 求唯一最优旋转。
- N=1 或几何退化（方向判别量相对尺度不大于 `1e-12`）时抛 `FORMATION_ORIENTATION_UNOBSERVABLE`，要求调用者显式提供 `psi`。
- 反求得到的 `q_F` 必须再次通过 `validate_vehicle_centers` 的两两距离和逐 ID 重建检查，否则不返回；因此错误队形、拉伸和反射原因会以带 pair/vehicle 上下文的错误暴露。

## 实际测试命令与退出码

| 实际命令 | 结果 | 退出码 |
|---|---|---:|
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_formation_model.py' -v` | 26/26 通过；测试计时 0.002 s，墙钟 0.15 s；最大 RSS 51884 KiB | 0 |
| `python3 -I -c '<独立导入 formation>'` | 输出 `standalone_import_ok FormationModel` | 0 |
| `OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'` | 177 项，176 通过、1 错误；测试计时 7.639 s，墙钟 7.81 s；最大 RSS 209268 KiB | 1 |
| `source /opt/ros/noetic/setup.bash; catkin_make -C .../catkin_ws -j1` | 构建系统检查与增量 `make` 完成；墙钟 2.18 s；最大 RSS 16972 KiB | 0 |
| catkin devel 环境下 Python 导入 | 成功解析到实际源码文件的 `FormationModel` | 0 |
| `git diff --check` | 无输出 | 0 |

完整回归的 1 项错误与 S0 完全相同：

```text
tests/test_continuous_retiming.py:271
KeyError: 'dynamic_peaks'
Ran 177 tests in 7.639s
FAILED (errors=1)
```

177 = S0 的既有 151 项 + 本阶段新增 26 项。新增测试无 failure、无 error；既有 `dynamic_peaks` 错误未在本阶段修复。`catkin_make` 本次为增量调用，没有据此推断全量编译耗时或重新构建数量。

## 定向测试覆盖

| 合同 | 覆盖 |
|---|---|
| `ψ = 0, π/2, -π/2, π` | 手算期望中心，12 位小数检查 |
| N=1/2/3 | 正变换、完整校验和反求往返 |
| 非零参考平移/偏移 | q_F 平移非零、车辆偏移非零，反求后位姿一致 |
| 展开航向 | `ψ = 16π + 0.2` 调用的中心与 `ψ = 0.2` 一致 |
| 刚性距离误差 | 正逆映射残差均小于 `1e-10` m |
| 重复 ID | 偏移构造、中心序列输入分别失败并带 `vehicle_id` |
| 任意重合偏移 | 非相邻 ID 的重合偏移也被拒绝 |
| 非有限值 | 偏移、编队位姿、中心、车体航向、容差和显式 `psi` |
| 缺车/额外车辆 | 错误信息分别列出 missing 和 extra ID |
| 反射 | 两两距离残差可接近 0，但给定 `q_F` 重建失败，反求也失败 |
| 车辆 ID 互换 | 使用所有两两距离相等的等边队形，距离残差为 0，但逐 ID 重建失败 |
| 非刚体拉伸 | 反求后重建检查失败 |
| 方位退化 | 单车无 `psi` 时拒绝；提供 `psi` 后通过；中心坍缩时要求显式 `psi` 且仍因重建失败被拒 |
| 车体航向独立 | 车体航向与 `q_F.psi` 不同且原样保留；没有根据编队方位自动赋值 |
| 不可变性 | 冻结偏移不可修改，`offsets` 属性不可重新赋值，参考距离返回副本 |

## 未执行项目与边界

1. 未进入地图搜索，未计算任何地图上的多车起点、终点或路径。
2. 未实现 S2 的车辆 OBB、车辆间碰撞、扫掠或地图碰撞检查。S1 仅拒绝**中心完全重合**的偏移；中心不重合不代表车体不重叠。
3. 未创建 S3 ROS 消息、话题、launch 或消息适配器。
4. 未引入 C++ `formation_model.hpp`；按文档在 S4 前再引入，避免两套实现同步。
5. 未实现共同时间轴、同步重定时、逐车模式或 FormationEvaluator。
6. 未启动 ROS 节点、RViz、回放或地图 Demo；未运行 C++ 回归（本阶段没有 C++ 改动）。
7. 未删除或覆盖用户文件，未推送远程，未执行 `git reset --hard`。

## 安全性审计

- 本阶段不产生可执行轨迹，不能给出 `FEASIBLE`、硬件安全或实时性声明。
- 车体航向不能被编队方位替代；API 中不存在隐式 `theta_i = psi_F` 的路径。
- 反求禁止反射；退化方位必须显式指定，不能由优化器随意猜测。
- 严格 ID 集合和逐 ID 重建检查避免缺车、重复车以及换标车辆被当作同一队形。
- 反求的最终接受条件是对象级容差检查，而不是“最小二乘已经收敛”本身。

## 存在风险

1. 两个已标记点的镜像与旋转在二维中不可区分；N=2 无法凭几何唯一判断标签是否对应镜像布局，必要时仍需由上游提供明确语义。N>=3 的非退化队形才能通过符号形状检查。
2. 反求的退化阈值是相对尺度判断，不是安全阈值；任何真实规划输入仍应经过显式 ID、地图碰撞和独立轨迹审核。
3. 当前容差默认 `1e-6` m；不能把纯数学 `1e-10` 测试阈值解释成车辆定位、碰撞或执行安全容差。
4. 本阶段未验证任何真实多车场景，也没有性能或地图成功率数据。

## 完成判据与 S2 前置条件

- `FormationModel` 与严格输入校验：**完成**。
- 定向测试全部通过，模块可脱离 ROS 导入：**确认**。
- 旧回归新增失败：**无**；既有 `dynamic_peaks` 错误仍保留。
- S2 前置条件：在 `formation_collision.py` 中调用本模型作为车辆中心的唯一几何来源；同时检查每辆车的 OBB、编队扫掠与 margin，不能把 S1 的中心不重合判据当作碰撞安全证明。
