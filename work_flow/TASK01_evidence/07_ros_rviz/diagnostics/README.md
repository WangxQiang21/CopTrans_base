# S1 corridor/hard 失败的独立诊断

日期：2026-11-19。本目录只记录诊断，不修改代码、配置或阈值。
未 commit/push；不宣称 Codex PASS 或 TASK 01 验收。

## 1. 现场症状

`07_ros_rviz/s1_main_free/` 的 `--scenario main`（S1 FREE，目标
`(8.819581504169413, 3.4858722492682084, pi/2)`）在 plan 600000 超时。
latched 状态与 `publications.jsonl` 显示：

- stage 序列：`REQUEST_ADMITTED → ASTAR_SEARCH → CORRIDOR_BUILD →
  PORTAL_GUIDE → NLP_SOLVE → GEOMETRY_AUDIT → FAILED`；
- pipeline `status=GEOMETRY_FAILED`，`first_failure_layer=GEOMETRY`，
  `reason=CORRIDOR_AUDIT_WIDTH_LIMIT`；
- `s_m=7.757789761470737`，`corridor_index=0`，`width_m=9.682634578567217e-06`，
  `point_residual_m=-4.692437438724717e-06`，
  `bound_residual_m=2.0528728480482543e-07`；
- geometry audit `passed=true`；dynamics/collision 尚未运行（`None`）；
- `/formation/playback_plan` 原始消息 2 条（plan 0 启动占位、plan 600000
  失败占位），**可执行信封 0 条**（两条 `model_json` 长度和
  `trajectory` 采样数均为 0）。

## 2. 离线复现

用冻结 S1 场景（`evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/
inputs/S1/map.yaml` + `request.json`，grid `6e94c751…`）离线调用
`plan_virtual_a`，planner config 仅把 guidance/enforcement 设为
`corridor`/`hard`，其他字段保持场景 payload：

```bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src
python3 repro_s1_corridor.py --guidance corridor \
    --output s1_corridor_worktree_run.json
```

当前工作树结果（`s1_corridor_worktree_run.json`；命令行/stdout/stderr
另存为同名 `.stdout/.stderr`）：

```text
status=GEOMETRY_FAILED reason=CORRIDOR_AUDIT_WIDTH_LIMIT
s_m=7.757780009545941 width=9.68263449241391e-06
point_residual=-4.893289646812349e-06
bound_residual=4.435030596203895e-09
geometry_status=GEOMETRY_FEASIBLE → containment 拒绝
```

## 3. 与 HEAD 对照

从 `HEAD=77805cd` 用 `git archive` 导出独立源码树
`/tmp/task01_head_tree`（未使用 worktree，未触碰共享 stash），运行同一个
场景与脚本（`--code-root /tmp/task01_head_tree`）。HEAD 结果
（`s1_corridor_head_run.json`；另存 `s1_corridor_head_run.stdout/.stderr`）：

```text
status=GEOMETRY_FAILED reason=CORRIDOR_AUDIT_WIDTH_LIMIT
s_m=7.757780009545941 width=9.68263449241391e-06
point_residual=-4.893289646812349e-06
bound_residual=4.1183355077337625e-08
```

- 失败位置、走廊索引、区间宽度、点残差**逐位相同**；
- 只有 `epsilon_total_m`/bound residual 不同（当前 5.64e-8 vs HEAD 9.32e-8），
  这是 Step 5 的 v3 数值证书改变了路径 `position_error_bound_m`，不是求解
  轨迹改变；
- `corridor.py` 在 `git diff HEAD` 中没有修改；`corridor_margin_m=1e-5`、
  `corridor_audit_position_margin_m=5e-8`、`corridor_audit_min_width_m=1e-5`
  的默认值在 HEAD 与当前工作树一致。

**结论：该失败不是 TASK 01 Step 1–5 引入的回归。** 它是 corridor/hard
在 `min_width_m=1e-5` 停止细分时无法用区间界证明约 `4.9e-6 m` 点间隙的
既有认证边界。Step 2 把默认入口从 `legacy_route` 改为 `corridor/hard` 后，
这条既有边界才在 demo 默认入口上暴露。

机制（未改任何数学）：solver 半空间约束使用
`corridor_effective_margin=1e-5` 内缩，而该路径由于离散配点之间的偏差在
该点只剩约 `4.9e-6 m` 点间隙；连续审计的半宽界为 `0.5*9.68e-6`，在
`width <= min_width_m` 时停止并 fail closed。没有放宽阈值、没有改
optimizer/corridor 代码。

## 4. 全场景探测（未修改代码）

`scene_probe_head_worktree.txt`：S1、S1_mirror、S2、S2_mirror、S3、
S3_mirror 在 corridor/hard 下的结果。S1/S1_mirror 是
`CORRIDOR_AUDIT_WIDTH_LIMIT`；S2/S2_mirror 是 `SOFT_BUDGET_EXHAUSTED`
（30 s solver 预算，5 个 initial 无 audited candidate）；S3/S3_mirror 是
`NO_ACCEPTED_CANDIDATE`/`FIXED_ALLOCATION_LIMIT`。同一 S1/S2 场景在
`legacy_route` 下分别为 SUCCESS（S=10.49956869991864 / S=11.998777606015235）。

## 5. 本目录未做的事

- 未修改 `corridor_margin_m`、`corridor_audit_min_width_m`、
  `corridor_audit_position_margin_m` 或任何 audit/optimizer 代码；
- 未把失败场景改小目标来冒充同一场景成功；
- 未删除或覆盖现场失败证据 `s1_main_free/`；
- 未把 HEAD 复现树写回工作树。

后续 Step 6 的成功发布证据使用**显式更小目标**在 corridor/hard 下的另一个
session 取得，并在交付文档中与本节失败并列报告；不把该成功解释为
原 S1 目标已经通过 corridor/hard。