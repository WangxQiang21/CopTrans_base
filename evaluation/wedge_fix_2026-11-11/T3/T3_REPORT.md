# T3：基本平移、零运动任务与独立审核/回放（报告）

日期：2026-11-12。工作目录：`/home/wq/whut_thesis/expv3_multi`。
T3 起始 HEAD：`3b6d678`（T2 提交）。本报告只覆盖 T3；T4–T7 仍未完成。

## 1. 结论

T3 完成标准全部满足：

- 30×30 m、0.05 m 空地图、正确楔形构型、货物起点 `(10,10,0)`、三车初始航向 0、
  终端航向自由、允许倒车；
- E00–E04 全部经真实入口 `run_demo.sh` 规划、S9 独立审核、事务发布与回放；
- 零运动为明确的 `NO_MOTION` 审计保持，一条公共 `t=0` 采样，不伪造非零路径；
- 关闭 NLP：最新一轮 **15/15 attempt 通过**（3 次 × 5 任务）；
- 开启默认 NLP：**15/15 attempt 通过**，E01/E02/E04 与关闭 NLP 结果一致，
  E03 仅时长缩短（26.371 s → 24.044 s），已有可行解未被破坏；
- 独立审核、回放状态、标记、TF、终端位姿逐 attempt 校验通过；
- 定向测试 63/63、C++ 规划器测试 20/20 通过；全量 Python 416 项仅保留
  1 个 T0 已记录的历史错误（`dynamic_peaks`，非 T3 缺陷）。

第一轮 NLP-off 冒烟（`smoke_nlp_off.json`）13/15 通过，暴露的两处**测试端**
等待竞态已修复并用第二轮重新验收；该失败轮次按要求原样保留，未删除。

## 2. 改动清单

| 文件 | 作用 |
|---|---|
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_geometry.py` | `_validate_seed` 增加 `allow_empty`；新增 `validate_zero_motion_seed`：仅当起终点位姿相同且所有 primitive 长度为零（空序列可）时判为零运动候选，否则交回正常 S6 路径 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_hold.py`（新） | `NO_MOTION`、`HOLD_CONTRACT`、`build_hold_timed_trajectory`（单采样静态保持）、`evaluate_formation_hold`（独立静态审核）、`hold_failure`、`zero_motion_seed` |
| `catkin_ws/src/fwis_v3_optimizer/scripts/formation_optimizer_node.py` | 空/零长度种子先尝试零运动审计；`NO_MOTION` 进入事务 gate 发布；pipeline/status 增加 `executable`、`collision_certificate`、`no_motion`、`hold_contract`；新增 `NO_MOTION_PUBLISHED` 事件与 `hold` 拒绝阶段 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py` | 可执行状态为 `NO_MOTION` 时发布 `NO_MOTION` 认证标签，单采样轨迹在 `t≥0` 即进入静态 `HOLD_END` 终端 |
| `catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py` | 接受并记录 `NO_MOTION` 的成功发布语义（不再当作失败或未知状态） |
| `tests/test_formation_hold.py`（新） | 14 项：静态保持可行；非零速度/偏移中心/错误航向/伪 Spin/多采样/地图签名不符/起点碰撞均被拒；空种子与零长度种子识别；非零 primitive、不同终点、种子身份或起点不符均不判保持 |
| `tests/test_formation_t3_missions.py`（新） | 5 项解析验收：E00–E04 经 S6 build → S8 retiming → S9 select，检查模式、带符号速度、终端位姿、逐采样刚性与终端零速 |
| `tests/test_formation_optimizer_node.py` | 新增 4 项：零运动 pipeline 审计保持、不同终点的空种子不判保持、碰撞起点被拒、`NO_MOTION` 经 gate 发布静态保持 |
| `tests/test_formation_playback.py` | 新增 1 项：单采样保持即静态终端状态，状态文档含 `NO_MOTION` 认证 |
| `tests/ros_t3_smoke.py`（新） | 真实入口冒烟：私有 master、`run_demo.sh`、3 次 × E00–E04，逐 attempt 验证状态、质量报告、轨迹、回放、标记、TF 与终端位姿 |

本阶段未修改 C++ 规划器、消息定义、配置文件、launch、阈值或地图。未修改
`archive/`、`catkin_ws/build/`、`catkin_ws/devel/`（构建产物除外）、相邻仓库或用户文件。

## 3. 零运动合同（`NO_MOTION`）

判定（`validate_zero_motion_seed`）：

- 请求起终点 formation pose 相同（xy/yaw 均 ≤ `1e-6`）；
- 种子身份/头/起终点/地图签名通过正常 S6 校验；
- 所有 primitive 长度为零（空序列视为零长度）。

审计（`evaluate_formation_hold`）：

| 检查 | 要求 |
|---|---|
| 公共时间轴 | 恰好一个采样 `t=0`，各车采样数均为 1 |
| 构型 | 每车中心 = `q_F(0)+R(ψ_F(0))r_i`，误差 ≤ 位置容差 |
| 航向 | 每车 `theta_i` = 请求初始航向，误差 ≤ 航向容差 |
| 运动 | 带符号中心速度、yaw rate、轮速均为 0；`stationary=True` |
| 模式 | A，`control=0`（禁止用伪 Spin 表示静止） |
| 碰撞 | 静态 OBB/地图证书，使用请求的精确地图签名与 S9 碰撞裕量 |

`HOLD_CONTRACT` 原文字符串写入优化器状态与质量报告，回放状态文档
`certification=NO_MOTION`。保持轨迹本身仍是 `GEOMETRY_ONLY`，只有
`evaluate_formation_hold` 返回 `FEASIBLE` 后才允许发布。

## 4. 任务级解析验收（`tests/test_formation_t3_missions.py`）

| 用例 | 目标 | S6/S8/S9 结果 | 模式与带符号速度 | 终端 |
|---|---|---|---|---|
| E00 | `(10,10,0)` | 空种子 `INVALID_SEED/EMPTY_PRIMITIVE_SEQUENCE`，零运动审计 `FEASIBLE/NO_MOTION` | 单采样，速度/控制为 0 | 保持起点，未伪造路径 |
| E01 | `(13,10,0)` | `FEASIBLE`，全部候选可行 | A，运行段 `speed>0` | Δx=3，θ≡0 |
| E02 | `(10,13,0)` | `FEASIBLE`，全部候选可行 | O，`|φ|=π/2`，车体 θ≡0；90° 横移的有符号前向速度恒为 0 | Δy=3，Δx=0 |
| E03 | `(13,12,0)` | `FEASIBLE` | O，`φ=atan2(2,3)`，运行段 `speed>0`，θ≡0 | Δx=3，Δy=2 |
| E04 | `(7,10,0)` | `FEASIBLE`，全部候选可行 | A，运行段 `speed<0` | Δx=−3，θ≡0 |

所有用例逐采样检查中心 = `q_F+R(ψ_F)r_i`，`max_rigidity_error_m ≤ 1e-6`；
终端 formation pose 与三车中心误差 ≤ `1e-6`，终端带符号速度 ≈ 0。

说明：E02 是纯 90° 横移，`dot(center_velocity, body_heading)=0`，因此 `direction=±1`
在带符号前向速度上不可观测；S5 可标为正向或反向 O，二者 `|φ|=π/2` 且车体不转。
测试与冒烟按 `|φ|=π/2` 验收，不把符号当作独立可观测量。

## 5. 真实入口冒烟

命令入口为 `run_demo.sh`（私有 master 11447，`rviz:=false`，
`auto_goal:=false`，30×30 空地图），每个任务连续 3 次，逐 attempt 验证：

- 终端状态：`status`、`plan_id`、`feasible`、`executable`，E00 另验
  `NO_MOTION` 标志、原因与合同；
- `/formation/quality_report`：`independent_audit=true`、
  `collision_certificate=true`、`pipeline.status`；
- 轨迹：公共时间轴从 0 严格递增、车辆身份/偏移、逐采样刚性 ≤ `1e-6` m、
  点数一致、E00 单采样零速、E01 A 正速、E02 O 且带符号速度 0 且 Δy>2.9、
  E03 O 正速且 Δx=3/Δy=2、E04 A 负速且 Δx<−2.9；
- 回放：等待与期望认证匹配的可执行状态文档，验 `plan_id`/终端/时长/采样数；
- 标记：每车 OBB、航向、状态文本；终端中心与目标 ≤ `1e-4` m；
- TF：参考位姿与三车中心 ≤ `1e-3` m，帧时间新鲜。

### 5.1 NLP 关闭（验收轮，`smoke_nlp_off_r2.json`）

| 用例 | 成功 | T_end (s) | 采样数 | 总检查数/attempt | 回放升级延迟 (s) |
|---|---:|---:|---:|---:|---:|
| E00 | 3/3 | 0.0 | 1 | 72 | 0.000 / 0.014 / 0.027 |
| E01 | 3/3 | 14.871258808 | 745 | 4534 | 0.0 |
| E02 | 3/3 | 14.871258808 | 745 | 4537 | 0.0 |
| E03 | 3/3 | 26.371248392 | 1324 | 8011 | 0.0 |
| E04 | 3/3 | 14.871258808 | 745 | 4537 | 0.0 |

总耗时 247.561 s；顶层 7 项检查全通过（真实入口启动、NLP 参数生效、
5 个任务 3/3）；`run_demo`/`roscore` 退出码 0，端口已释放；权威 YAML
sha256 仍为 `21b9ff574bbfe2e190958a415ae0232a4d9df0146165960aa66f00f9e3f9ca92`
（以 `config.sha256` 为准）。

### 5.2 默认 NLP（`smoke_nlp_on.json`）

| 用例 | 成功 | T_end (s) | 采样数 |
|---|---:|---:|---:|
| E00 | 3/3 | 0.0 | 1 |
| E01 | 3/3 | 14.871258808 | 745 |
| E02 | 3/3 | 14.871258808 | 745 |
| E03 | 3/3 | 24.044066379 | 1211 |
| E04 | 3/3 | 14.871258808 | 745 |

总耗时 253.644 s。E00/E01/E02/E04 与关闭 NLP 相同；E03 的 O 分段更短、
φ 值不同，时长由 26.371 s 降为 24.044 s，终端 `(13,12,0)`、模式 O、正速、
刚性与回放仍全部通过。
未出现 NLP 失败导致可行解被丢弃的情况。

### 5.3 保留的失败轮次（`smoke_nlp_off.json`）

- 12/15 attempt 通过，E00 1/3、E03 2/3；
- E00 attempt 2/3：`playback_executable: False` —— 回放第一个 `HOLD_END` 在
  优化器状态到达前被标为 `GEOMETRY_ONLY`，属测试等待时序；
- E03 attempt 2：`marker_center_1` 偏差 4.648 mm —— 读到终端前的旧标记数组。

修复仅在冒烟等待逻辑：要求与期望认证匹配的可执行状态文档，以及中心与终端
一致的最新标记数组。生产节点、阈值与轨迹未因此改变。原始文件保留未改。

## 6. 测试结果

| 测试 | 结果 | 日志 |
|---|---:|---|
| T3 定向（`test_formation_t3_missions` / `test_formation_hold` / `test_formation_optimizer_node` / `test_formation_playback`） | 63/63 通过，4.99 s | `runtime/targeted_t3_tests.log` |
| 全量 Python（`unittest discover`） | 416 项，1 error，其余通过 | `runtime/python_tests.log` |
| C++ `test_formation_geometry` | 14/14 通过 | `runtime/planner_cpp_formation_geometry.log` |
| C++ `test_planner_policy_history` | 6/6 通过 | `runtime/planner_cpp_policy_history.log` |

全量套件唯一错误仍是 T0 已记录的历史失败
`tests/test_continuous_retiming.py:271 KeyError: 'dynamic_peaks'`，按要求保留、
未修复、未隐藏，因此全量命令退出码为 1。

## 7. 可复现命令

```bash
cd /home/wq/whut_thesis/expv3_multi

# T3 定向单元测试（63）
OPENBLAS_NUM_THREADS=1 python3 -m unittest \
  tests.test_formation_t3_missions tests.test_formation_hold \
  tests.test_formation_optimizer_node tests.test_formation_playback

# 全量 Python（416；保留 1 个历史错误）
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'

# C++ 规划器测试
catkin_ws/devel/lib/fwis_v3_planner/test_formation_geometry
catkin_ws/devel/lib/fwis_v3_planner/test_planner_policy_history

# 真实入口冒烟，NLP 关闭（3 次 × 5 任务）
OPENBLAS_NUM_THREADS=1 python3 tests/ros_t3_smoke.py \
  --port 11447 --enable-nlp false --attempts 3 \
  --output evaluation/wedge_fix_2026-11-11/T3/smoke_nlp_off_r2.json \
  --log-dir evaluation/wedge_fix_2026-11-11/T3/runtime_nlp_off_r2

# 真实入口冒烟，默认 NLP（3 次 × 5 任务）
OPENBLAS_NUM_THREADS=1 python3 tests/ros_t3_smoke.py \
  --port 11447 --enable-nlp true --attempts 3 \
  --output evaluation/wedge_fix_2026-11-11/T3/smoke_nlp_on.json \
  --log-dir evaluation/wedge_fix_2026-11-11/T3/runtime_nlp_on
```

冒烟脚本只清理自己启动的 `run_demo.sh`、`roscore` 与进程组，拒绝占用
11311/11312/11320。

## 8. 机器可读结果

`T3.json` 汇总三次冒烟、逐 attempt 状态/T_end/采样/模式/质量报告、测试计数与
失败原因。摘要：

```json
{
  "stage": "T3",
  "date": "2026-11-12",
  "stage_pass": true,
  "accepted_runs": ["smoke_nlp_off_r2.json", "smoke_nlp_on.json"],
  "preserved_failed_rounds": ["smoke_nlp_off.json"],
  "runs": {
    "nlp_off_run2": {"pass": true, "attempts_succeeded": 15, "attempts_total": 15,
      "duration_s": 247.561},
    "nlp_on": {"pass": true, "attempts_succeeded": 15, "attempts_total": 15,
      "duration_s": 253.644},
    "nlp_off_run1_failed": {"pass": false, "attempts_succeeded": 12,
      "attempts_total": 15, "duration_s": 246.623}
  },
  "tests": {
    "targeted_t3": {"tests": 63, "failures": 0},
    "full_python": {"tests": 416, "errors": 1, "failures": 0,
      "preserved_historical_error": "tests/test_continuous_retiming.py:271 KeyError: 'dynamic_peaks'"},
    "planner_cpp": {"formation_geometry": {"tests": 14, "failures": 0},
                    "policy_history": {"tests": 6, "failures": 0}}
  }
}
```

完整逐 attempt 数据见 `T3.json`；原始 ROS 输出见三个 `smoke_*.json` 与
`runtime_nlp_*/`。按 `evaluation/README.md` 的仓库瘦身约定与“不操作相邻归档
目录”的约束，原始大文件保留在工作树，不复制进仓库历史。

## 9. 失败原因

- 历史失败：`tests/test_continuous_retiming.py:271 KeyError: 'dynamic_peaks'`
  为 T0 已记录的旧问题，本阶段未修改该测试或数据结构。
- 第一轮 NLP-off 冒烟的 2 个失败 attempt 是**测试端**等待竞态（回放状态升级、
  旧标记数组），不是规划或审核失败；修复等待条件后第二轮 15/15。
- E02 的 O 方向符号不可观测，已在测试注释与本报告第 4 节说明；未通过修改
  S6 打分或调度键来强行统一符号（超出 T3 范围，留给后续阶段评估）。
- 未放宽碰撞裕量、终端阈值（0.02 m / 1°）、动力学或轮速限制。

## 10. 已验证 / 未验证范围

已验证：

- E00 零运动在真实 C++ 前端空种子、Python 审计、事务发布、回放、桥接全链路成功；
- E01 直行、E02 横移（车体不转）、E03 斜移、E04 倒车的模式、带符号速度、
  终端位姿、刚性、碰撞证书与回放；
- NLP 关闭与默认 NLP 两种配置下 3 次 × 5 任务均通过；
- 实车尺寸楔形构型、空地图、固有阈值下的逐 attempt 独立审核。

未验证（留给后续阶段）：

- T4：货物静止时车辆独立 Spin（含 Spin 后 A 环绕）与 Spin 扫掠碰撞；
- T5：失败终态、候选预算、`connect_exact_goal` 两种顺序与事务竞态；
- T6：RViz 图形插值、连续目标、重置与旧消息生命周期；
- T7：map04/map16 真实地图、×3.75 尺度、用户脚本与 RViz 图形验收；
- 真实车辆动力学/载荷模型：本阶段仍是几何运输抽象，不代表实物载荷验证；
- E02 90° 横移的方向标签仍不可由带符号速度独立观测。