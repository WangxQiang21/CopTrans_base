# T4：货物固定时车辆独立车体 Spin（报告）

日期：2026-11-12。工作目录：`/home/wq/whut_thesis/expv3_multi`。
T4 起始 HEAD：`4be42e5`（T3 提交）。本报告只覆盖 T4；T5–T7 仍未完成。

## 1. 结论

T4 完成标准全部满足：

- 初始三车航向均为 0、货物原地 `+90°`（`(10,10,0)→(10,10,π/2)`）经真实入口
  `run_demo.sh` 返回 `FEASIBLE` 并发布；
- 同一场景的 `−90°` 在解析链路（S6 候选 → S8 时间 → S9 独立审核）完成；
- 整个 Spin 事件中 `q_F` 与三车中心固定，逐采样刚性误差为 0；
- 车辆 1 以 SPIN 原地转向 `π/2`，中心误差 0、中心速度 0；随后进入绕货物的
  A 运动（中心行程 `1.414 m`）；
- 至少一个测试明确断言“车辆自身 Spin → A 绕货物”，且 ±90° 两个方向都覆盖；
- S9 独立重算固定 `q_F`、固定中心、逐车连续航向、模式方程、车体旋转扫掠碰撞
  与动力学；篡改航向、篡改中心、把绕行车误标为 Spin 均被拒绝。

最新真实入口冒烟 `smoke_nlp_off_r2.json` 顶层 **40/40 检查通过**，退出码 0，
端口已释放。第一轮失败 `smoke_nlp_off.json` 原样保留；其失败是**测试端**沿用了
T4 之前“E06 失败后仍停在起点”的预期，E06 本身的全部 T4 检查在该轮已通过。

## 2. 改动清单

| 文件 | 作用 |
|---|---|
| `formation_spin.py`（新） | 内部 `BODY_SPIN` 事件：`SpinScheme`、`make_spin_block`、`heading_only_targets`、`heading_reconfiguration_needed`、`build_heading_only_plan`、`rotation_schemes`。`q_F` 固定，旋转车用 SPIN 与方向符号，不转车用零控制、零行程的 A 保持；事件长度 = 最大绝对航向变化 |
| `formation_kinematics.py` | 增加 `BODY_SPIN` 常量；构造/`qf_at`/`audit_mode_assignment` 接受并验证固定 `q_F`、固定中心、零控制/行程、SPIN 方向；旋转分支继续拒绝绕行车的 `NON_PIVOT_SPIN` |
| `formation_geometry.py` | `FormationPathControl`/`FormationBlock` 支持 `BODY_SPIN`；零运动种子在显式航向仍有差异时不再误判保持；`_fit_terminal_geometry` 支持先 Spin 再拟合终端刚体块；新增 `_audited_block_choices`/`_block_options`，在平移前比较 A/O 与切向/反切向 Spin 方案（上限 16）；航向-only 分支生成单个 `BODY_SPIN` 计划；Spin 扫掠采样按角点半径与最大角步长计算 |
| `formation_retiming.py` | Spin 的 `q_F` 导数恒为 0、`psi_F` 固定；逐车上下文验证固定中心、零控制/行程、SPIN 非零航向与方向、A 保持无航向；中心速度置 0，跳过 A 曲率检查；S8 用逐车角速度/角加速度/角急动度约束求公共事件时长 |
| `formation_quality.py` | S9 独立评估支持 `BODY_SPIN`：重算固定 `q_F`、固定中心、A 保持零控制/行程、SPIN 航向符号；车体旋转用扫掠 OBB 碰撞；速度按 0 处理 |
| `tests/test_formation_t4_spin.py`（新） | 10 项：航向-only 不是保持；+90/−90 Spin 后 A 环绕；公共事件时长由最大航向变化约束；驻留与 Spin 时间分离；回放公共时钟与独立航向；Spin 中途扫掠碰撞；独立审核篡改拒绝；中心从 `q_F` 重算；绕行车误标 Spin 拒绝 |
| `tests/test_formation_optimizer_node.py` | 新增 1 项端到端节点测试：航向全 0 的货物旋转依次产生 `body_spin` 与 `rotation`，车辆 1 原地转向后沿 A 弧绕行 |
| `tests/ros_cargo_smoke.py` | E06 从“预期 `HEADING_BOUNDARY_MISMATCH` 失败”改为验收独立车体 Spin：初始航向、可行、独立审核/碰撞证书、公共时间网格、逐采样刚性、静止 Spin、后续 A 环绕、逐车终端航向、终端货物位姿；E06 后的桥接用例改为从 `(10,10,π/2)` 继续 |
| `README.md` | 删除“三车航向 0 时返回 `HEADING_BOUNDARY_MISMATCH`、T4 未实现”的过期说明，改为 T4 已验证行为与报告链接 |

本阶段未修改 C++ 规划器、ROS 消息、配置文件、launch、碰撞/终端阈值或地图；
未使用外部未定义的 COMPOSITE 种子；未修改 `archive/`、`catkin_ws/build/`、
`catkin_ws/devel/`（构建产物除外）、相邻仓库或用户文件。

## 3. 事件合同（`body_spin`）

| 项目 | 约束 |
|---|---|
| 内部类型 | `geometry_type="body_spin"`，不是外部种子 primitive，不静默复用 COMPOSITE |
| 货物位姿 | `start_qf == end_qf`（x/y/psi 均固定，容差 `1e-7`） |
| 车辆中心 | 每车 `start_center == end_center`，且由 `q_F + R(psi_F) r_i` 重算 |
| 旋转车辆 | `mode=SPIN`，`control=0`，`travel=0`，航向变化非零且方向符号与最短弧一致 |
| 不转车辆 | `mode=ACKERMANN` 保持，`control=0`，`travel=0`，航向不变 |
| 事件长度 | `max_i |wrap(target_i - start_i)|`，不是各车角度之和，也不是零长度平移 |
| 公共时长 | S8 在一条时间轴上按逐车角速度/角加速度/角急动度求最小可行公共时长 |
| 模式切换驻留 | Spin 运动之后单独的 stationary `mode_switch` 事件；不计入 Spin 运动时长，也不重复计费 |
| 碰撞 | 车体旋转的连续 OBB 扫掠（地图与其他车辆），不仅检查首末位姿 |
| 发布 | 一条公共时间轴，逐车独立 `theta_i`，无边界跳变；回放未到终端前不得提前进入下一段 |

`FormationTimedPoint.theta` 始终是车体航向，不是 `psi_F`；回放按最短弧插值。

## 4. 解析验收（±90° 与 Spin→A）

`tests/test_formation_t4_spin.py` 的 `_check_cargo_rotation` 对
`+π/2` 与 `−π/2` 都执行：

1. `build_continuous_plans` 成功，首个 `BODY_SPIN` 块前后 `q_F` 与所有中心相同；
2. 至少一辆车为 SPIN，且下一块是 ROTATION（由 A 弧实现绕货物转动），
   存在行程 `>0.1 m` 的 A 段；
3. `audit_continuous_plan` 通过；
4. S8/S9 选出可行轨迹，终端货物位姿误差 `≤1e-6`；
5. 所有车辆共用严格递增的同一时间轴；Spin 段内中心固定、航向连续变化，
   相邻采样航向差不超过 `yaw_rate_max·0.02`；
6. 逐采样中心 = `q_F + R(psi_F) r_i`，误差 `≤1e-7`。

`test_heading_only_request_is_a_body_spin_not_a_hold` 进一步验证公共事件时长：
车辆 1 转 `π/2`、车辆 2 转 `−π/4`、车辆 3 保持；三车 yaw rate、
yaw acceleration、yaw jerk 均在限制内，且车辆 1 的 yaw rate 是约束上界，
证明时长由最大航向变化决定，而不是各车分别计时。

`test_spin_time_and_mode_switch_dwell_are_separate` 验证：Spin 运动相位
`>0`、模式切换驻留 `>0`、驻留起点严格等于 Spin 结束、SPIN 模式区间只含
实际 Spin 时长，随后 A 区间也满足最小驻留。

## 5. 碰撞与 S9 独立审核

- `test_body_spin_obb_sweep_rejects_obstacle_outside_both_endpoints`：墙体障碍
  放在仅扫掠中途相交的位置，首末位姿均无碰撞；旧的两端静态检查会放过，
  新扫掠检查拒绝（`precheck.samples > 1`）。
- `test_tampered_spin_target_is_rejected_by_independent_audit`：篡改 Spin 终端
  航向后 S9 拒绝。
- `test_stored_spin_centres_are_recomputed_from_qf`：改写存储中心但不改 `q_F`，
  S9 以 `CONSTRUCTION_MISMATCH` 拒绝，证明中心是重算而非信任存储。
- `test_orbiting_vehicle_mislabelled_as_spin_is_rejected`：把绕货物运动的车辆
  标成 SPIN，S9 以 `NON_PIVOT_SPIN` 拒绝，未把 S 当成普通转向。

## 6. 真实入口冒烟（`smoke_nlp_off_r2.json`）

命令入口：`run_demo.sh`，私有 master 11448，30×30 m、0.05 m 空地图，
`rviz:=false`、`auto_goal:=false`、`enable_nlp:=false`。顶层检查 **40/40**，
总耗时 124.691 s，`run_demo`/`roscore` 退出码 0，端口已释放；配置仍读取
`transport_formation.yaml`（sha256 `21b9ff…ca92`）。

E06 结果：

| 项目 | 值 |
|---|---|
| 初始三车航向 | `[0.0, 0.0, 0.0]` |
| 状态 | `FEASIBLE` / `OK`，规划 22.4368 s |
| 独立审核 / 碰撞证书 | `true` / `true` |
| 公共采样数 | 1130 |
| 逐采样刚性误差 | 0 m |
| Spin 车辆 / 航向变化 | 1 / `1.570796 rad` |
| Spin 中心误差 / 中心速度 | 0 m / 0 m/s |
| Spin 后 A 中心行程 | `1.414 m` |
| 终端逐车航向 | `{1: π, 2: π/2, 3: π/2}` |
| 终端 `psi_F` / 货物 TF | `π/2` / `(10,10,π/2)` |

同一轮还回归了 E05（航向与 `psi_F` 不同的旋转）、连续目标真实航向、
E01/E02 桥接、标记与 TF 一致性；全部通过。

## 7. 测试结果

| 测试 | 结果 | 日志 |
|---|---:|---|
| T4 定向（`test_formation_t4_spin` 10 + `test_formation_optimizer_node` 23） | 33/33 通过，11.383 s | `runtime/targeted_t4_tests.log` |
| 全量 Python（`unittest discover`） | 427 项，1 error，其余通过 | `runtime/python_tests.log` |
| C++ `test_formation_geometry` | 14/14 通过 | `runtime/planner_cpp_tests.log` |
| C++ `test_planner_policy_history` | 6/6 通过 | 同上 |

全量套件唯一错误仍是 T0 已记录的历史失败
`tests/test_continuous_retiming.py:271 KeyError: 'dynamic_peaks'`，按要求保留、
未修复、未隐藏，因此全量命令退出码为 1。

## 8. 可复现命令

```bash
cd /home/wq/whut_thesis/expv3_multi

# T4 定向单元测试（33）
OPENBLAS_NUM_THREADS=1 python3 -m unittest \
  tests.test_formation_t4_spin tests.test_formation_optimizer_node

# 全量 Python（427；保留 1 个历史错误）
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'

# C++ 规划器测试
catkin_ws/devel/lib/fwis_v3_planner/test_formation_geometry
catkin_ws/devel/lib/fwis_v3_planner/test_planner_policy_history

# 真实入口冒烟（E06 + 既有回归检查）
OPENBLAS_NUM_THREADS=1 python3 tests/ros_cargo_smoke.py \
  --port 11448 \
  --output evaluation/wedge_fix_2026-11-11/T4/smoke_nlp_off_r2.json \
  --log-dir evaluation/wedge_fix_2026-11-11/T4/runtime_nlp_off_r2
```

冒烟脚本只清理自己启动的 `run_demo.sh`、`roscore` 与进程组，拒绝占用
11311/11312/11320。

## 9. 机器可读结果

`T4.json` 汇总验收运行、保留失败轮次、E06 指标、解析验收与测试计数。摘要：

```json
{
  "stage": "T4",
  "stage_pass": true,
  "accepted_runs": [{
    "file": "smoke_nlp_off_r2.json",
    "pass": true, "checks_ok": 40, "checks_total": 40,
    "e06": {"status": "FEASIBLE", "independent_audit": true,
            "collision_certificate": true, "common_samples": 1130,
            "max_rigidity_error_m": 0.0, "spin_vehicle": 1,
            "spin_heading_change_rad": 1.5707963267948966,
            "orbit_centre_travel_m": 1.4142135623730951}
  }],
  "preserved_failed_rounds": [{
    "file": "smoke_nlp_off.json", "pass": false,
    "failure": "E01_forward_request_start expected the pre-T4 start",
    "product_fault": false
  }],
  "tests": {
    "targeted_t4_and_node": {"tests": 33, "failures": 0},
    "full_python": {"tests": 427, "errors": 1, "failures": 0},
    "planner_cpp": {"tests": 20, "failures": 0}
  }
}
```

## 10. 失败原因

- 第一轮冒烟 `smoke_nlp_off.json`：E06 的全部 T4 检查通过，唯一失败为
  `E01_forward_request_start` 仍期待旧失败路径的起点 `(10,10,0)`；实际回放
  已到 E06 终端 `(10,10,π/2)`。修复仅在测试桥接预期，生产代码未改。原始
  失败文件保留、未覆盖。
- 历史失败：`tests/test_continuous_retiming.py:271 KeyError: 'dynamic_peaks'`
  为 T0 已记录旧问题，本阶段未修改该测试或数据结构。
- 未放宽碰撞裕量、终端阈值（0.02 m / 1°）、动力学或轮速限制。

## 11. 已验证 / 未验证范围

已验证：

- 三车初始航向 0 时，货物 `+90°` 经真实入口规划、S9 审核、事务发布与回放；
- 同一场景 `−90°` 经 S6/S8/S9 解析链路完成；
- Spin 期间固定 `q_F`、固定三车中心、逐车独立航向、公共时间轴与无跳变；
- Spin 扫掠碰撞（墙体和车辆）、模式方程、动力学峰值与驻留分离；
- S9 对篡改航向、存储中心与错误模式标签的独立拒绝。

未验证（留给后续阶段）：

- `−90°` 的真实 ROS 入口冒烟（当前为解析链路验收；T7 可纳入完整矩阵）；
- T4 在默认 NLP 开启下的表现（本阶段冒烟为 `enable_nlp:=false`）；
- T5：混合平移+货物旋转、失败终态、无解/连续目标、候选预算与事务竞态；
- T6：RViz 图形插值、连续运行与旧消息生命周期；
- T7：map04/map16 真实地图、×3.75 尺度、完整验收矩阵与 NLP on/off 对比；
- 真实车辆动力学/载荷模型：本阶段仍是几何运输抽象，不代表实物载荷验证。