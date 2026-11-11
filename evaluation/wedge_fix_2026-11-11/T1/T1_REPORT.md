# T1：统一正确队形与航向合同（报告）

日期：2026-11-11。工作目录：`/home/wq/whut_thesis/expv3_multi`。
T1 起始 HEAD：`58dc57f`（T0 提交）。本报告只覆盖 T1，不将 T2–T7 标记为完成。

## 1. 结论

T1 完成标准全部满足：

- 启动配置、请求、Marker、TF、面积工具五处读取同一份权威配置；
- 含三车不相同 `theta_i` 的合法请求通过 S9 审核并发布；
- 默认实验不再出现 `CARGO_HEADING_MISMATCH` 或
  `CARGO_ROTATION_REQUIRES_GENERAL_4WIS`。

真实 `run_demo.sh` 入口冒烟测试 **PASS**：33/33 检查，91.321 s，
`run_demo`/`roscore` 均退出 0，私有端口已释放（`smoke.json`）。

## 2. 改动清单

### 2.1 T1 本阶段新增/修改

| 文件 | 作用 |
|---|---|
| `catkin_ws/src/fwis_v3_demo/config/transport_formation.yaml` | 唯一权威配置：ids、offsets、车身尺寸、`allow_reverse`、`goal_heading_free`、初始航向策略 |
| `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/transport_contract.py` | 严格解析/校验合同；`cargo_heading` 与 `tangent` 两种初始策略 |
| `catkin_ws/src/fwis_v3_demo/launch/formation_demo.launch` | bridge/playback 均 `rosparam command="load"` 同一 YAML；删除内联 offsets；`require_cargo_heading` 默认 false |
| `catkin_ws/src/fwis_v3_demo/scripts/formation_goal_bridge.py` | 读取合同；新实验按策略初始化 `theta_i(0)`；连续目标从一次同步播放 TF 快照读取起点与三车实际航向 |
| `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py` | 静态预览/失败保持使用合同 offsets 与航向；`transport_ui` 状态 |
| `run_demo.sh` | 编队入口帮助文本改为新构型；×3.75 明确标注为旧策略、待 T7 重新验收 |
| `tests/test_transport_contract.py` | 合同字段、距离/半径、策略、非法值拒绝、launch 唯一加载 |
| `tests/test_cargo_transport.py` | 足迹数值、独立切向航向合法性、旧锁仅作显式受限实验、静态预览 |
| `tests/test_formation_demo_assets.py` | 断言 launch 不再有内联 offsets，且加载权威 YAML |
| `tests/ros_cargo_smoke.py` | 真实入口端到端冒烟：五处一致性、E05/E06、连续目标、手动重置、失败后恢复 |
| `tools/transport_footprint.py` | 只读同一 YAML 计算足迹与比例因子候选，输出 sha256 |
| `README.md` | 当前多车入口说明改为 T1 事实（新 offsets、独立航向、已验证/未实现范围） |

### 2.2 随 T1 一并提交的 T0 前遗留支撑改动

以下文件在 T0 快照前已修改、T0 未提交；T1 的真实运行时和测试依赖它们，
因此随 T1 提交，使提交树与已验收工作树一致。它们不代表 T2/T3 已完成：

- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py`
  （`require_cargo_heading` 字段、默认 false、旧锁分支限定）
- `catkin_ws/src/fwis_v3_optimizer/scripts/formation_optimizer_node.py`
  （`start_on_publication`：只平移共享时间原点）
- `catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_playback.py`
  （`build_hold_markers`：失败/静态三车显示）
- `catkin_ws/src/fwis_v3_planner/src/formation_search.cpp`
  （终端同时闭合位置与航向）
- `catkin_ws/src/fwis_v3_planner/src/formation_planner_node.cpp`
  （地图晚到后重放挂起请求）
- `catkin_ws/src/fwis_v3_planner/tests/test_formation_geometry.cpp`
  （近终点必须精确闭合位姿与航向）

### 2.3 未提交

- 用户文件：`CURRENT_REPOSITORY_STATUS.md`、两份用户 Markdown、
  `evaluation/cargo_transport/`、`evaluation/formation_s10/screenshots/`、
  `tools/configure_formation_view.py`。
- 未执行 `git reset --hard`、`git clean`、删除或丢弃任何未跟踪/已修改内容。

### 2.4 新增证据

- `smoke.json`（33 项检查、观察值、清理结果）
- `footprint.json`（足迹工具机器可读输出）
- `runtime/run_demo.log`、`roscore.log`、`ros_logs/`
- `runtime/targeted_tests.log`（47 tests）
- `runtime/python_tests.log`（全量 378 tests，保留历史错误）
- `runtime/catkin_make.log`（构建退出 0）
- `runtime/planner_cpp_tests.log`（14 tests）

## 3. 权威合同与五处一致性

权威 YAML sha256：
`21b9ff574bbfe2e190958a415ae0232a4d9df0146165960aa66f00f9e3f9ca92`。

| 项 | 值 |
|---|---|
| ids | `1, 2, 3` |
| offsets (m) | `(1,0)`, `(0,1)`, `(0,-1)` |
| 车身 | `0.738 × 0.500 m` |
| 相邻中心距 | `d12=d13=√2 m`, `d23=2 m`；每车中心距货物 1 m |
| `allow_reverse` | `true` |
| `goal_heading_free` | `true` |
| `initial_heading_policy` | `cargo_heading` |
| `require_cargo_heading` | 默认 `false`（launch 参数显式传入） |

一致性核对：

1. **YAML**：`transport_contract.py` 严格解析；非法 ids/offsets/类型被拒绝。
2. **launch**：bridge 与 playback 各只有一条 `rosparam command="load"`，
   均指向 `$(arg transport_config)`；launch 中无第二份 offsets。
3. **请求**：E01、E02、E05、连续目标的 `/formation/plan_request` 均被冒烟脚本
   逐字段比对 ids/offsets/尺寸/策略，全部一致。
4. **Marker/TF**：手动起点 `(10,10,0)` 的静态预览为
   `(11,10)/(10,11)/(10,9)`、航向 0；Marker 与配置最大误差
   `4.44e-16`，TF 与 Marker 误差 0。
5. **面积工具**：`transport_footprint.py` 读取同一 sha256 的 YAML，
   AABB `[1.738, 2.5] m`、凸包 `3.345 m²`、保守旋转包络 `6.5662 m²`、
   轴向比例候选 `5.0`（见 `footprint.json`）。

## 4. 验收用例（真实入口、私有 master）

运行环境：`run_demo.sh --map .../maps/empty30 --scale 1.0
auto_goal:=false rviz:=false enable_nlp:=false`；私有 master `11447`；
30×30 m、0.05 m 空地图。全部输入/观察保存在 `smoke.json` 的 `observed`。

### 4.1 静态手动起点

`/initialpose (10,10,0)` 后显示三车
`[[11,10],[10,11],[10,9]]`，航向均为 0（`cargo_heading` 策略），
旧可执行轨迹不存在。该手动起点随后被第一个 bridge 目标消费，避免
直接注入 E05 后连续目标仍从旧配置起点出发。

### 4.2 E05：合法的不相同 `theta_i`（T1 核心）

直接向 `/formation/plan_request` 注入 plan 700000：

- 货物：`(10,10,0) → (10,10,π/2)`；
- 起点车体航向：`θ=(π/2, π, 0)`（ψ=0 时的正向切向）；
- `goal_heading_free=true`，三车目标中心由刚性合同给出；
- 结果：`FEASIBLE / OK`，`trajectory_published=true`；
- 终端航向：`{1: π, 2: 3π/2, 3: π/2}`（与解析切向一致）；
- 终端货物 TF：`(10.000000, 10.000000, 1.570796)`。

相比 T0 的 `CARGO_ROTATION_REQUIRES_GENERAL_4WIS`，默认实验已可通过。

### 4.3 连续目标读取真实回放航向

E05 完成后，经 bridge 发 `2D Nav Goal` 到 `(10,13,π/2)`：

| 项 | 值 |
|---|---|
| 请求起点 | `[10.0, 10.0, 1.5707963267948963]`（= 已显示终端，不是配置起点） |
| 起点航向 | `{1: π, 2: -π/2, 3: π/2}`（= 回放实际值，不是 cargo 策略的全体 π/2） |
| 结果 | `FEASIBLE / OK` |
| 终点航向 | 与起点相同，平移不重置车体航向 |
| 终端货物 TF | `(10.000000, 13.000000, 1.570796)` |

bridge 在一次 TF 查询中取同一时间戳的三车帧，拒绝超过 `0.5 s`
的快照和刚性残差超过 `1e-3 m` 的快照；发布轨迹不被当作到达。

### 4.4 手动重置是新实验

`/initialpose (10,10,0)` 后回放为 `STATIC / START`、`executable=false`，
三车重新按 `cargo_heading` 初始化为航向 0（最大误差 0），旧轨迹不可执行。

### 4.5 E06：当前预期的拒绝

手动重置后，bridge 发 `(10,10,π/2)`，请求起点航向全 0：

- 优化器：`REJECTED / HEADING_BOUNDARY_MISMATCH`，
  detail=`vehicle_id=1 body heading 0 is neither the forward tangent
  1.57079632679 nor the reverse tangent -1.57079632679`；
- 回放：`STATIC / REJECTED: HEADING_BOUNDARY_MISMATCH`，
  三个静态 OBB，无新路径；
- 所有优化器状态中不存在 `CARGO_HEADING_MISMATCH` 或
  `CARGO_ROTATION_REQUIRES_GENERAL_4WIS`。

这是 T4（货物静止时独立 Spin）尚未实现时的预期行为，不是 T1 回归。

### 4.6 失败后恢复

E06 后经 bridge 依次执行 E01 `(13,10,0)` 与 E02 `(13,13,0)`（货物位置横移）：

| 用例 | 结果 | 三车终端中心 (m) |
|---|---|---|
| E01 前进 | `FEASIBLE / OK` | `[[14,10],[13,11],[13,9]]` |
| E02 横移 | `FEASIBLE / OK` | `[[14,13],[13,14],[13,12]]` |

起点由回放实际位姿读取，请求与 YAML 一致，终端中心误差 0。

## 5. 可复现命令

```bash
cd /home/wq/whut_thesis/expv3_multi

# 端到端冒烟（脚本自建 11447 master，自启 run_demo.sh，结束后清理）
OPENBLAS_NUM_THREADS=1 python3 tests/ros_cargo_smoke.py \
  --output evaluation/wedge_fix_2026-11-11/T1/smoke.json \
  --log-dir evaluation/wedge_fix_2026-11-11/T1/runtime

# 足迹机器可读输出
python3 tools/transport_footprint.py \
  --out evaluation/wedge_fix_2026-11-11/T1/footprint.json

# T1 相关单测（47）
OPENBLAS_NUM_THREADS=1 python3 -m unittest \
  tests.test_transport_contract tests.test_cargo_transport \
  tests.test_formation_demo_assets tests.test_formation_playback

# 全量 Python 单测（378；保留 1 个历史错误）
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p 'test_*.py'

# C++ 规划器几何测试（14）
catkin_ws/devel/lib/fwis_v3_planner/test_formation_geometry

# 增量构建
source /opt/ros/noetic/setup.bash
catkin_make -C catkin_ws -j2
```

## 6. 机器可读结果

```json
{
  "stage": "T1",
  "pass": true,
  "config_sha256": "21b9ff574bbfe2e190958a415ae0232a4d9df0146165960aa66f00f9e3f9ca92",
  "smoke": {"checks": 33, "failed": 0, "duration_s": 91.321,
            "run_demo_exit": 0, "roscore_exit": 0, "port_released": true},
  "targeted_tests": {"tests": 47, "failures": 0},
  "planner_cpp_tests": {"tests": 14, "failures": 0},
  "full_python_suite": {"tests": 378, "errors": 1,
    "preserved_historical_error": "tests/test_continuous_retiming.py:271 KeyError 'dynamic_peaks'"},
  "footprint": {"aabb_m": [1.738, 2.5], "hull_m2": 3.345,
    "heading_free_envelope_m2": 6.5662084606402,
    "axis_scale_factor": 5.0, "disk_scale_factor": 5.782856135530851}
}
```

证据文件：`smoke.json`、`footprint.json`、`runtime/*.log`、`runtime/ros_logs/`。

## 7. 失败原因

### 7.1 仍存在的产品限制（不属于 T1 缺陷）

- E06（三车航向 0、货物原地 +90°）返回 `HEADING_BOUNDARY_MISMATCH`：
  当前模型没有“货物与车中心固定、车体独立 Spin”的合法块，T4 才能支持。
- 全量测试中 `test_continuous_retiming.py:271` 的
  `KeyError: 'dynamic_peaks'` 是 T0 已记录的历史失败，按指示保留，
  未修复、未隐藏。

### 7.2 调试中修正的测试/时序问题

- 一次性请求在订阅者连接前发布导致超时 → 等待
  `request_pub.get_num_connections() >= 4`。
- E05 直接注入绕过 bridge，旧 `/initialpose` 覆盖未被消费，连续目标从
  `(10,10,0)` 出发 → 先用 bridge E01 消费覆盖，再注入 E05。
- 优化器先发布带同一 plan_id 的空事务清除轨迹 → 轨迹等待同时要求
  `vehicles` 与 `trajectories` 非空。
- `STATIC` 回放状态没有 `vehicles` 字段 → 静态预览改用 Marker 与 TF 校验。
- Marker `ADD` 常量为 0（不是 2）→ 修正等待条件。
- 单测曾断言 car3 的切向航向必须非零，但其合法值就是 0 →
  改为断言三个航向两两不同。

## 8. 已验证 / 未验证范围

已验证：

- 真实 `run_demo.sh` 入口、真实 devel 节点、私有 master 的完整
  S4→S9→发布→回放链；
- 五处配置一致性、静态预览 Marker/TF、手动重置；
- 含不相同 `theta_i` 的合法请求（E05）及连续目标保留真实航向；
- E06 的预期拒绝与失败后 E01/E02 恢复；
- 构建、47 项 T1 单测、14 项 C++ 几何测试、378 项全量 Python 测试。

未验证（留给后续阶段）：

- T2：正/负旋转符号、曲率与独立微分审计；
- T3：平移/倒车/横移/斜移/零运动全部任务、动态上限、NLP 默认对照；
- T4：车体独立 Spin 与 ±90° 原地转向；
- T5：前端/后端反馈与 RViz 插值刚性；
- T6：连续目标事务、晚到消息、重置竞争；
- T7：真实地图 map04/map16、RViz 图形流程、×3.75 尺度重新验收；
- 用户原始失败尚未复现（T0 已声明）；
- `require_cargo_heading=true` 仅单测覆盖，未做端到端受限实验。

## 9. 运行清理

冒烟脚本只清理自己启动的 `run_demo.sh`、`roscore` 与进程组；
未使用全局 `pkill`，未占用 11311/11312/11320。
`cleanup.exit_codes.run_demo=0`、`roscore=0`、`port_released=true`。