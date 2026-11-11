# T0：冻结现状与最小失败复现（报告）

日期：2026-11-11。工作目录：`/home/wq/whut_thesis/expv3_multi`。
本报告只覆盖 T0（复现与归因），未修改规划实现。

## 1. 现状快照（已冻结，未清理）

| 项 | 值 |
|---|---|
| 分支 | `feat/formation-eval` |
| HEAD | `620f5fd5b556e350c5889b994453621ddc8bb774` |
| 未提交修改 | 12 个文件，`git diff --binary` 共 985 行 |
| 未跟踪 | `evaluation/cargo_transport/`、`evaluation/formation_s10/screenshots/`、`tests/ros_cargo_smoke.py`、`tests/test_cargo_transport.py`、`tools/transport_footprint.py`、`tools/configure_formation_view.py`、`CURRENT_REPOSITORY_STATUS.md` 及两份用户文档 |
| 源码哈希 | 114 个源文件的 sha256 在 `snapshot/source_sha256.txt` |
| 快照文件 | `snapshot/git_status.txt`、`git_head.txt`、`git_diff_stat.txt`、`working_tree.diff`、`ros_overlay.txt` |

`git diff --check` 无空白错误。**未执行** `git reset --hard`、`git clean`、删除未跟踪文件或任何丢弃改动操作。

## 2. 运行环境（真实用户入口）

| 项 | 值 |
|---|---|
| 入口 | `bash run_demo.sh --map <empty30> --scale 1.0 auto_goal:=false rviz:=false` |
| ROS master | `http://127.0.0.1:11407`（私有；11311/11312/11320 未占用） |
| 地图 | `evaluation/wedge_fix_2026-11-11/maps/empty30`，600×600、0.05 m、origin(0,0)、30×30 m，PGM sha256 `d888e9…b8d9` |
| 尺度 | 显式 `--scale 1.0`（绝对比例），记录于 `runtime/` |
| 优化器 | `enable_nlp=true`、`max_candidates=4`、`optimizer_time_limit=1.0`（均为入口默认值） |
| 节点来源 | devel 包装器 relay 到工作区源码（`devel/lib/.../formation_goal_bridge.py` 为 catkin relay 脚本） |
| 运行参数 | `runtime/param_goal_bridge.yaml`、`param_optimizer.yaml`、`param_playback.yaml` |

实测生效参数（失败根因的直接证据）：

```yaml
# /formation_goal_bridge
offsets: [[0.0, 1.0], [-1.0, 0.0], [1.0, 0.0]]   # 旧构型
goal_heading_free: false
# /formation_optimizer
kinematic_limits:
  require_cargo_heading: true                      # 旧航向锁
enable_nlp: true
max_candidates: 4
```

## 3. 用例结果（空地图、正确入口、真实规划链）

每例独立 `2D Pose Estimate` 起点 `(10,10,0)`；`2D Nav Goal` 由 `tools/formation_case_probe.py`
按 RViz 工具同样的话题与消息类型发布，不依赖鼠标。全部证据在 `cases/<id>/`：
`inputs.json`（起终点/重放输入）、`topics.jsonl`（全话题原始记录）、`verdict.json`（归因）、
`trajectory.json`（成功轨迹）。运行时参数快照在 `runtime/param_*.yaml`（由 `rosparam dump` 保存）。

| 用例 | 货物目标 | 结果 | 首个失败阶段 | 耗时 |
|---|---|---|---|---|
| E01 前进 | (13,10,0) | **FEASIBLE_PUBLISHED** | — | 3.36 s |
| E02 横移 | (10,13,0) | **FEASIBLE_PUBLISHED** | — | 3.91 s |
| E06 原地 +90° | (10,10,π/2) | REJECTED | 后端模式分配 | 1.05 s |
| E09 平移+转向 | (13,10,π/2) | REJECTED | 后端模式分配 | 1.05 s |
| E00 零运动 | (10,10,0) | REJECTED | `INVALID_SEED` | 1.05 s |
| E06 重放 | 由 `inputs.json` 重发 | REJECTED（同因） | 后端模式分配 | 1.05 s |

失败原因与阶段：

- **E06/E09：`CARGO_ROTATION_REQUIRES_GENERAL_4WIS`**
  - 前端**已成功**搜索并发布几何种子：`primitive_count=9`、`GEOMETRY_ONLY_UNCERTIFIED`。
  - 后端拒绝，`detail` 原文：
    `A/O/S cannot rotate this rigid formation while keeping every body aligned with cargo: an off-axis vehicle needs simultaneous lateral translation and body yaw`
  - 触发点：`catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_kinematics.py:802`
  - 归因：**不是**前端无解、不是地图太窄、不是碰撞；是旧"车货同航向"锁（`require_cargo_heading=true`）在模式枚举阶段直接排除所有转向实现。
- **E00：`INVALID / INVALID_SEED / EMPTY_PRIMITIVE_SEQUENCE`**
  - 前端对起终点相同返回 0 段 primitive；后端 `_validate_seed` 以空序列判非法。
  - 归因：缺少 `NO_MOTION`/保持语义（合同 T3 要求）。
- 失败画面：三例 `marker_count=12`，回放保持静止编队并显示 `STATIC + REJECTED: <reason>`，**不是空画面**。

## 4. 独立核对（不依赖构造器公式）

### 4.1 已发布轨迹的航向独立性

`tools/formation_trajectory_audit.py` 对 E01/E02 的 745 个共同时钟采样点复核：

| 指标 | E01 | E02 |
|---|---|---|
| `max |θ_i − ψ_F|` | 0.0 rad | 0.0 rad |
| 任两车航向不同 | 否 | 否 |
| 节点刚性残差 | 0.0 m | 0.0 m |
| 显示插值刚性残差 | 0.0 m | 0.0 m |
| 车间距离漂移 | 0.0 m | 0.0 m |
| 距离 d12/d13/d23 | 1.4142/1.4142/2.0 m | 同左 |

结论：**当前管线能产生的全部成功轨迹中，三车航向始终相同且恒等于货物航向**（`θ_i≡ψ_F`）。
合同 6.1 要求"至少一条成功用例出现 `θ_i≠ψ_F` 且车辆间航向不同"，当前实现**一次也产生不了**。
这不是测试遗漏，是构造侧的硬约束（见 §5）。

### 4.2 旋转区间的显示刚性（源码确认 + 数值验证）

`formation_playback.frame_at()` 分别线性插值各车中心与货物位姿，未按 `p_F+R(ψ_F)r_i` 重算。
用合同构型 (1,0)/(0,1)/(0,-1)、半径 1 m、`publication_dt=0.02` 的合成旋转轨迹实测
（`probe/rotation_interpolation_probe.json`）：

| 角速度 | 每采样角差 | 解析中点误差 `1−cos(Δψ/2)` | 实测最大显示刚性误差 |
|---:|---:|---:|---:|
| 0.2 rad/s | 0.004 rad | 2.0×10⁻⁶ m | 2.0×10⁻⁶ m |
| 0.5 rad/s | 0.010 rad | 1.25×10⁻⁵ m | 1.25×10⁻⁵ m |
| 1.0 rad/s | 0.020 rad | 5.0×10⁻⁵ m | 5.0×10⁻⁵ m |

三档均超过合同建议的 1×10⁻⁶ m。当前两条可行轨迹是纯平移（ψ_F 恒定），所以尚未暴露；
一旦转向打通，该缺陷会直接出现在 RViz 显示与发布采样中。

### 4.3 顺时针旋转方向（文档 P0-3 探针）

`probe/negative_rotation_probe.txt`，正确构型、切向初值 `(π/2,π,0)`：

| `dpsi` | 枚举 `direction` | 实际中心速度沿车头投影 | 曲率 | S5 审核 |
|---:|---:|---:|---:|---|
| +0.5 | +1 | +1.0 | +1.0 | 通过 |
| −0.5 | **+1** | **−1.0** | −1.0 | **仍通过** |

负旋转被标成正向，S5 审计未发现。与文档 P0-3 一致。

## 5. 与合同不符点（T0 确认）

| 编号 | 证据 |
|---|---|
| P0-1 运行时与合同矛盾 | launch 两处旧 offsets（:67、:85-89）、`require_cargo_heading=true`（:55）、`goal_heading_free=false`（:81）；bridge 旧 fallback（:149-150）；playback 预览旧 fallback；footprint 工具旧 offsets+×3.75 |
| P0-1 bridge 重置航向 | `formation_goal_bridge.py:190-201` 三条起点路径都返回空航向字典；`:259-263` 每次 goal 都令 `body_headings={所有车: 货物航向}`；`_latest_body_headings()` 存在但**无任何调用** |
| P0-2 无独立航向重配置 | `formation_kinematics.py:654-667` 旋转要求各车处于切向；`:412-425` 拒绝零长度平移/零角度旋转；无"货物与各车中心固定、车体自转"的合法块 |
| P0-3 负旋转符号 | §4.3 |
| P1-1 前端航向/碰撞语义 | `formation_collision.hpp:299` 全程按 `theta_i=start_theta_i+(ψ_F−start_ψ_F)`；`:412` `goal_clear()` 用占位 `goal_theta` 且不读 `goal_heading_free` |
| P1-2 显示插值破坏刚性 | §4.2 |
| P1-3 测试范围 | `tests/ros_cargo_smoke.py` 固化旧 offsets（:110）、`θ_i≡ψ_F`（:114-116）、转向必失败（:140-143）；`run_demo.sh` 入口未被测试覆盖 |
| P1-4 尺度/动态 | `run_demo.sh:277-279` 自动 ×3.75（map16 默认即 3.75）；`transport_footprint.py:11-14` 旧构型与 ×3.75 |

## 6. 代码审计（只读）说明

并行只读审计覆盖 7 个模块。**7 个读取阶段完成，6 个交叉复核因 API 配额（429）未执行**，
因此复核结论仅 `config_entry` 模块有；上表 P0 项均已由独立探针/实测独立确认，不依赖该审计。
原始输出：`audit/read_pass_results.json`。

各模块关键结论（读取阶段）：

- `config_entry`：旧 offsets 出现在 6 处；同航向文案 2 处；`run_demo.sh` 默认 map16 尺度 3.75。
- `frontend_cpp`：搜索状态无独立 `θ_i`，无静止车体自转事件；`goal_clear()` 忽略 `goal_heading_free`。
- `s5_kinematics`：旋转切向要求、零长度/零角拒绝、负旋转方向由 `dpsi` 未参与选择。
- `geometry_candidates`：终端拟合只能改最后一段，无法补车体自转重配置。
- `retiming_quality`：无静止车体自转事件；S9 用 `hypot(p_dot)` 与 `direction*speed`，未做车头投影/侧向残差；NLP 曲率用 `abs(delta)/radius`。
- `tests_baseline`：16 处旧合同断言/文档（清单见 `read_pass_results.json`）。
- `playback_display`：**读取阶段因配额缺失**，但 P1-2 已由源码阅读 + §4.2 数值探针确认。

## 7. 可复现性

每个用例可从记录文件重放，已验证：

```bash
cd /home/wq/whut_thesis/expv3_multi
source /opt/ros/noetic/setup.bash && source catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://127.0.0.1:11407 ROS_LOG_DIR=/tmp/wedge_t0/logs
python3 tools/formation_case_probe.py --out /tmp/replay_e06 \
  --replay evaluation/wedge_fix_2026-11-11/T0/cases/E06_inplace_rotation/inputs.json
```

重放结果：`REJECTED reason=CARGO_ROTATION_REQUIRES_GENERAL_4WIS`，与原始运行一致（见 `cases/E06_replay/`）。

## 8. 已验证 / 未验证范围

已验证：
- 真实 `run_demo.sh` 入口 + 真实 devel 节点 + 私有 master 上的完整 S4→S9→回放链；
- 前进、横移成功；原地转向、平移+转向、零运动失败及其首个失败阶段；
- 失败可从文件重放；失败时车辆保持可见（12 markers）；
- 负旋转符号问题与旋转显示刚性误差的数值复现。

未验证：
- **用户原始失败尚未复现**：未提供当时地图、坐标与日志；本报告是新最小失败集，不声称已复现该次；
- RViz 图形流程、真实地图（map04/map16）、默认 NLP 下的完整矩阵（T7）；
- `enable_nlp=false` 与默认 NLP 的对照（T3/T7）；
- 前端失败后的终态缺失、连续目标事务、地图更新失效等（T5/T6）。

## 9. 运行清理

T0 演示（roslaunch 树、master 11407、5 个节点）已按 PID 逐个停止，仅清理本次启动的进程；
端口 11407/11311/11312/11320 均空闲。未触碰其他会话。