# Step 6 证据：构建、离线全回归与入口参数

日期：2026-11-19。本目录是 `work_flow/TASK01_Codex执行方案.md` Step 6
的离线部分。未 commit/push；本文件不宣称 Codex PASS 或 TASK 01 验收。
ROS/RViz 实机验证见 `../07_ros_rviz/`。

## 1. 顺序

按 §5.3，先在 Step 5 已完成 §5.1 七组离线测试，再完成本目录的
`discover` 全量离线回归；随后重新执行 `catkin_make`，再重跑六个
`--dry-run` 入口，最后才启动 ROS 会话。构建前的预构建日志保留为
`build_precheck.log/.time`（3.57 s，exit 0），本目录的 `build.log/.time`
是离线测试后的正式构建。

## 2. 构建

```bash
source /opt/ros/noetic/setup.bash
cd /home/wq/whut_thesis/expv3_multi_virtual_a/catkin_ws
catkin_make
source devel/setup.bash
```

| 项 | 值 |
|---|---|
| 正式构建退出码 | 0（`build.exit_code`） |
| 正式构建耗时 | `elapsed=0.37 s`（`build.stderr`） |
| 预构建（离线回归前） | exit 0，`elapsed=3.57 s`（`build_precheck.*`） |
| 构建日志 | `build.log`（87 行，无 error） |

## 3. 离线全量回归

命令（工作树根目录，执行环境与 §5.1 相同）：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH=$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:${PYTHONPATH:-}
python3 -m unittest discover -s tests -p 'test_*.py'
```

| 项 | 值 |
|---|---|
| 测试总数 | 1047 |
| 耗时 | 549.586 s（`elapsed=550.49 s`） |
| 退出码 | 1 |
| 通过 | 1046 |
| 错误 | 1（见 §3.1） |
| 失败 | 0 |

原始 stdout/stderr、退出码：`full_regression.stdout`、
`full_regression.stderr`、`full_regression.exit_code`。

### 3.1 唯一错误：既有 `test_continuous_retiming` 用例

```text
ERROR: test_short_translation_has_finite_acceleration_and_zero_end_speed
(tests.test_continuous_retiming.ContinuousRetiming)
KeyError: 'dynamic_peaks'  (tests/test_continuous_retiming.py:271)
```

该用例的输入构造（`Action(1, .08, .15)`）产生两个 phase，`phases[0]`
是 `mode_switch`（键集合含 `steering_profile` 等，无 `dynamic_peaks`），
而测试假定 `phases[0]` 一定是 `translation`。单独重跑得到同一错误：

- `isolated/continuous_retiming_41.stdout/.stderr/.exit_code`（exit 1）
- `isolated/continuous_retiming_41_no_devel.*`（去掉 ROS devel 环境，仍 exit 1）

判定为**既有失败，不是本任务引入**，依据：

- `git diff HEAD` 对 `tests/test_continuous_retiming.py`、
  `continuous_retiming.py`、`continuous_geometry.py`、`core.py`、
  `retiming.py`、`tests/__init__.py` 全为空（这些是执行该用例的全部代码）；
- 失败形态与 Step 1–5 的 virtual A 代码无导入关系；
- 该测试文件的最后一次提交是 `4523e0f`（共享 CopTrans 基线导入），
  与本任务分支的 virtual A 修改无关。

没有修改该既有失败用例，也没有把它计为本任务的门禁失败。§5.1 的七组
virtual A/formation 测试仍全部通过（见 `../05_numerics/summary.md` §8）。

### 3.2 测试对 `evaluation/` 的再生与恢复

`test_uniform_*`/`test_formation_regression` 在缺省环境下会重写
`evaluation/formation_refactor_v02/` 下的墙钟/探针字段与
`repo_root`/`package_src` 路径。本次回归确实产生这些写入；为遵守
“不得修改历史证据”，在记录差异后已全部恢复到 HEAD：

- `regenerated/evaluation_modified_by_tests.txt`（26 个被测试改写的文件）
- `regenerated/evaluation_diffstat_by_tests.txt`（+/- 行数）

恢复后 `git status --porcelain -- evaluation` 为空。差异内容只有
`wall_time_s`/`collision_s`/`generation_s`/`audit_wall_time_s`/
`search_elapsed_at_audit_s` 等计时值和本工作树路径，没有结论/门禁值
变化；恢复动作不影响本任务任何证据。

## 4. dry-run 入口

离线全回归后重跑，全部 exit 0；原始输出与退出码在
`dry_run_*.stdout/.stderr/.exit_code`：

| 入口 | 关键参数（stdout 原文） | 退出码 |
|---|---|---:|
| `--map map05 --scale 2.625` | `backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false scene:=manual_map rviz:=true` | 0 |
| `--map map05`（默认比例） | 同上，地图默认比例 0.7×3.75=2.625 | 0 |
| `--backend uniform_v02` | `backend:=uniform_v02 scene:=manual_map auto_goal:=true` | 0 |
| `--backend legacy` | `backend:=legacy scene:=manual_map auto_goal:=true` | 0 |
| `--single` | `demo.launch ... planning_backend:=mode_path_v2 ... auto_goal:=true` | 0 |
| 覆盖 `virtual_a_guidance_mode:=legacy_route virtual_a_corridor_enforcement:=soft` | 显式值保留（未注入 corridor/hard） | 0 |

`uniform_v02`/`legacy` 保留历史 `auto_goal:=true`；`virtual_rigid_a`
默认注入 `auto_goal:=false`，且显式覆盖不被改写。与 Step 2 的 dry-run
证据一致。

## 5. 边界

- 本目录不包含 ROS 节点/话题/截图证据；这些在 `../07_ros_rviz/`。
- 未修改 `00_baseline/`…`04_s_domain/` 与 `evaluation/`（见 §3.2）。
- 未 commit/push；未进入 TASK 02；未宣称 Codex PASS 或 TASK 01 验收。

## 6. 预览回归修复后的复跑（postfix）

ROS 证据阶段发现并修复了 Step 1 引入的 playback transport preview 回归
（症状、根因与最小修复见
`../07_ros_rviz/diagnostics/README_playback_preview_regression.md`；修复
只在 `catkin_ws/src/fwis_v3_playback/scripts/formation_playback.py`，另在
`tests/test_virtual_a_receiver.py` 新增 3 个 preview 用例）。修复后按同一
顺序重新执行离线全回归、正式构建和六个 dry-run，证据在 `postfix/`。

| 项 | 值 | 证据 |
|---|---|---|
| 测试总数 | 1050（此前 1047 + 3 个新用例） | `postfix/full_regression.stderr` |
| 耗时 | `Ran 1050 tests in 550.550s`（墙钟 551 s） | `postfix/full_regression.time/.wall` |
| 退出码 | 1 | `postfix/full_regression.exit_code` |
| 通过 | 1049 | `postfix/full_regression.stderr` |
| 错误 | 1，仍为同一个既有 `dynamic_peaks` KeyError（`tests/test_continuous_retiming.py:271`） | 同上 |
| 失败 | 0 | 同上 |
| 正式构建 | exit 0，`elapsed=0.35 s`，`build.log` 87 行、无 error | `postfix/build.*` |
| 六个 dry-run | 全部 exit 0（map05×2、uniform_v02、legacy、single、覆盖项） | `postfix/dry_runs/*.exit_code` |

dry-run 耗时（`postfix/dry_runs/*.time`）：map05 scale2625 `0.28 s`、
map05 默认 `0.29 s`、uniform_v02 `0.35 s`、legacy `0.34 s`、single
`0.20 s`、覆盖项 `0.29 s`。single 输出与修复前基线一致：地图比例 0.7，
`planning_backend:=mode_path_v2`。其余入口的注入参数与 §4 相同。

测试同样改写了 `evaluation/formation_refactor_v02/` 下 26 个文件；
`postfix/regenerated/evaluation_modified_by_tests.txt` 记录清单，
`evaluation_diffstat_by_tests.txt` 显示 `26 files changed, 665 insertions(+),
665 deletions(-)`。核对后已全部恢复，`git status --porcelain -- evaluation`
为空；差异仍只是计时/路径/内存等再生字段，结论值未变。历史证据目录
`00_baseline/`…`05_numerics/` 未被本目录改写。