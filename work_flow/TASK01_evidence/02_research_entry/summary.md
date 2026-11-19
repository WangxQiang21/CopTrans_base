# Step 2 研究入口证据（默认 virtual_rigid_a / corridor / hard）

- 日期：2026-11-19
- BASE HEAD：`77805cd46849ebadc374187f9b00fce0f7151f84`（未提交、未 push）
- 工作目录：`/home/wq/whut_thesis/expv3_multi_virtual_a`
- Python：3.8.10；ROS Noetic；CasADi 3.7.2
- 状态：Claude 自测完成；不声明 Codex PASS，不声明 TASK 01 验收通过

## 1. 运行环境

执行方案 §5.1 的字面 `export PYTHONPATH=$PWD/catkin_ws/src/fwis_v3_optimizer/src`
会覆盖 ROS/helper 路径；实际采用增量前置：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH
```

测试模块选择与 §5.1 一致。本轮是**全部测试编辑完成后的权威重跑**（早前后台运行
早于最后一次测试修正，未采用）。命令清单见 `commands.txt`；每组 stdout/stderr、
退出码、墙钟时长分别保存在 `g*.log` / `g*.exit_code` / `g*.duration`。

## 2. §5.1 七组结果

| 组 | 命令（测试模块） | 用例数 | 时长 | 退出码 | 日志 |
|---|---|---:|---:|---:|---|
| G1 | `tests.test_virtual_a_corridor_display tests.test_formation_hud` | 67 | 19.824 s | 0 | `g1_corridor_display_hud.log` |
| G2 | `tests.test_virtual_a_receiver tests.test_virtual_a_playback` | 49 | 57.586 s | 0 | `g2_receiver_playback.log` |
| G3 | `tests.test_formation_demo_assets` | 28 | 3.955 s | 0 | `g3_demo_assets.log` |
| G4 | `tests.test_virtual_a_corridor_optimizer tests.test_virtual_a_optimizer` | 55 | 75.773 s | 0 | `g4_corridor_optimizer_optimizer.log` |
| G5 | `tests.test_virtual_a_pipeline tests.test_virtual_a_corridor_recovery` | 26 | 123.732 s | 0 | `g5_pipeline_recovery.log` |
| G6 | `tests.test_virtual_a_geometry tests.test_virtual_a_profile` | 42 | 2.413 s | 0 | `g6_geometry_profile.log` |
| G7 | `tests.test_virtual_a_integration_bounds tests.test_virtual_a_audit_evaluation_guard tests.test_virtual_a_audit tests.test_virtual_a_trajectory` | 97 | 6.713 s | 0 | `g7_integration_audit_trajectory.log` |

补充（Step 2 修改了 adapter/node main）：

| 组 | 命令 | 用例数 | 时长 | 退出码 | 日志 |
|---|---|---:|---:|---:|---|
| G8 | `tests.test_virtual_a_ros_adapter` | 63 | 18.297 s | 0 | `g8_ros_adapter_supplemental.log` |

七组合计 364 例；含补充共 427 例，全部 `OK`。
`py_compile`（全部改动 Python 文件）、`bash -n run_demo.sh run_virtual_a.sh`、
`git --no-pager diff --check` 均通过。

## 3. dry-run 参数证据（`dry_run/`）

| 场景 | 关键输出 | 退出码 | 文件 |
|---|---|---:|---|
| 默认（`--map map04`） | `backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false`，四名各 1 次 | 0 | `default_map04.log` |
| 显式覆盖 | `legacy_route` / `soft` / `auto_goal:=true` 原样保留 | 0 | `explicit_overrides.log` |
| guidance 重复 | `virtual_a_guidance_mode:= 只能指定一次`，无 launch 行 | 2 | `duplicate_guidance.log` |
| enforcement 重复 | `virtual_a_corridor_enforcement:= 只能指定一次` | 2 | `duplicate_enforcement.log` |
| auto_goal 重复 | `auto_goal:= 只能指定一次` | 2 | `duplicate_auto_goal.log` |
| 显式 `uniform_v02` | 无任何 `virtual_a_*`，`auto_goal:=true`（历史行为） | 0 | `explicit_uniform.log` |
| 显式 `legacy` | 无任何 `virtual_a_*`，`auto_goal:=true`（历史行为） | 0 | `explicit_legacy.log` |
| `--single` | `demo.launch` + `planning_backend:=mode_path_v2`，无 virtual 参数 | 0 | `single_backend.log` |
| `run_virtual_a.sh` 默认 | 补 `corridor`/`hard`/`auto_goal:=false` | 0 | `wrapper_default.log` |
| `run_virtual_a.sh` 显式 `soft` | `soft` 保留，其余补缺省 | 0 | `wrapper_explicit_soft.log` |

## 4. launch XML → ROS param 证据（`dry_run/`）

- `roslaunch --nodes fwis_v3_demo formation_demo.launch`：4 个节点
  （`/fwis_map_publisher`、`/formation_uniform_planner`、`/formation_playback`、
  `/formation_goal_bridge`），退出码 0（`roslaunch_nodes.log`）。
- `roslaunch --dump-params ... backend:=virtual_rigid_a`：仅
  `/formation_uniform_planner` 出现
  `virtual_a_guidance_mode: corridor`、`virtual_a_corridor_enforcement: hard`
  （`roslaunch_dump_virtual.log`）。
- `roslaunch --dump-params ... backend:=uniform_v02`：所有节点均**没有**
  `virtual_a_*` 参数（`roslaunch_dump_uniform.log`）。

## 5. Step 2 验收链对应

| 链路环节 | 证据 |
|---|---|
| dry-run 参数 → launch 行 | §3 的 10 个场景 |
| launch XML → ROS private param | §4 的 `--nodes` / `--dump-params` |
| ROS param → `hooks.settings.planner_config` | `NodeMainResearchEntryTest.test_main_builds_the_validated_research_config`（`" Corridor "`/`"HARD"` → `corridor`/`hard`，日志参数一致）；`test_main_fails_closed_on_unknown_values` |
| settings → map request → `corridor_mode` | `RequestFromSnapshotTest.test_research_settings_reach_the_map_request_and_corridor_mode`；pipeline 在 `pipeline.py:799` 从同一 `planner_config.guidance_mode` 判定 `corridor_mode` |
| `_build_corridor_constraint()` → CasADi `g` | `test_launch_research_defaults_reach_the_casadi_halfspaces`：launch XML 默认值 → `research_planner_config()` → `_guidance_mode(config)=="corridor"` → `halfspace_constraint_count==352` → `nlp_g` corridor 区间 352 行（下界全 `-inf`、上界全 `0`）；`test_halfspace_residual_is_identical_in_numeric_and_casadi_nlp` 断言 numeric 与 CasADi 残差 ≤1e-12 |
| 新进度/结果字段 | `ProgressCallbackContractTest.test_corridor_evidence_reaches_progress_and_result`；实跑值见 `progress_corridor_evidence.txt`：成功 corridor 用例 `count=1001`、`residual=9.736952577554803e-09`，progress / `result.metrics` / `search["rounds"][-1]` 三处一致；pipeline 未发 `PUBLISHED` |
| 计数必须来自实际约束 | 计数为 `_CorridorConstraint.halfspace_constraint_count = Σ len(limits)`，与 numeric residual、CasADi `g` 共用同一 `groups`；无约束（mock/非 corridor）时为 `None`，不从 polygon 推算 |
| 历史默认不变 | `test_research_entry_factory_changes_only_the_two_choices`：直接 `PlannerConfig()` 仍为 `legacy_route`、`as_dict()` 不输出该字段；工厂只改两个选择并完成 `from_dict` 往返 |
| uniform/legacy/`--single` 不受影响 | `test_explicit_uniform_or_legacy_keeps_historical_auto_goal`、`test_cli_and_launch_arg_forms_select_the_same_backend`、`test_backend_is_formation_only`、`single_backend.log` |

## 6. 边界说明

- soft 仍允许显式实验（wrapper/dry-run/node 均保留），默认 hard；未知值启动失败。
- 未进入 Step 3 及后续；未修改 `00_baseline/`、`01_visualization/`、
  `work_flow/TASK01_技术方案.md`、`work_flow/TASK01_Codex执行方案.md`。
- 未 commit/push；本文件不构成 Codex 复核结论。