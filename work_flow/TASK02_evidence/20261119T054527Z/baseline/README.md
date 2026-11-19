# TASK02 Step A 基线证据

生成时间：2026-11-19T05:45:27Z（UTC）。目录：`work_flow/TASK02_evidence/20261119T054527Z/baseline/`。

本目录只冻结 TASK02 开始前的工作区事实，不修改源代码、历史证据或既有测试预期；它不是 Codex 审核结论，也不宣称 TASK01/TASK02 验收通过。

## 1. 真实仓库状态

| 项 | 值 |
|---|---|
| 工作树 | `/home/wq/whut_thesis/expv3_multi_virtual_a`（git worktree） |
| 分支 | `feature/virtual-rigid-formation-a`，相对 `origin/feature/virtual-rigid-formation-a` 领先 1 个提交 |
| HEAD | `bee4e3aa0f86e24085a1840ee0581b2fca34add8` |
| HEAD 提交 | `bee4e3a Separate RViz path categories and active route aliases` |
| 父提交 | `222b919 Add TASK 01 work-flow plans and evidence` |
| TASK01 代码提交 | `91dd71f Add virtual-A corridor pipeline and editable Path playback` |
| TASK01 审核报告基线 | `77805cd46849ebadc374187f9b00fce0f7151f84` |

工作区存在两处 TASK01 已接受返工的未提交修改（不是 TASK02 变更）：

```text
 M catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/formation_continuous_a/geometry.py
 M tests/test_virtual_a_geometry.py
```

SHA-256（`task01_dirty_sha256.txt`）：

```text
f31b20bbd6333c44f12be4064c069e11143c870ab02b99d4839b9778317efc3d  .../formation_continuous_a/geometry.py
4f59e68ff5dc26468d42e131836078b0d2eef06853e998bfca5e168c77ddb4c7  tests/test_virtual_a_geometry.py
```

完整 diff 见 `task01_dirty.diff`，差异统计 `2 files changed, 92 insertions(+), 1 deletion(-)`。另有 114 个既有未跟踪文件（`untracked_files_head50.txt` 只列前 50 个），包含 TASK01/RVIZ 文档、证据目录和 `latest` 符号链接；本任务不清理、不移动、不删除它们。

## 2. 文件清单

| 文件 | 内容 |
|---|---|
| `git_state_start.txt` | `git status --short --branch`、`git rev-parse HEAD`、分支名、`git log -3` |
| `git_status_porcelain.txt` | 完整 `git status --porcelain` |
| `git_diff_stat.txt` | 工作区 diff 统计 |
| `task01_dirty.diff` | 两个 TASK01 修改文件的完整 diff |
| `task01_dirty_sha256.txt` | 上述两个文件的 SHA-256 |
| `worktree_full.diff` | `git diff HEAD` 全文 |
| `untracked_files_head50.txt` | 未跟踪文件前 50 行（总 114 个） |
| `codex_report_comparison.md` | TASK01 审核报告 HEAD 与实际 HEAD 的差异核对 |
| `run_demo_dry_run.txt` | 默认入口 dry-run 实际输出 |
| `maps_sha256.txt` | map05 与冻结 S1 地图/配置的 SHA-256、分辨率与 origin |
| `tests/*.log` | 14 个定向测试模块的原始输出 |
| `tests/test_continuous_retiming.log` | 已知基线失败的复跑记录（18 tests，1 error） |
| `tests_summary.txt` | 14 个模块的耗时汇总 |

## 3. 测试基线（本工作区实际复跑）

```text
test_formation_demo_assets        Ran 29 tests, OK
test_formation_hud                Ran 19 tests, OK
test_virtual_a_collision          Ran 17 tests, OK
test_virtual_a_corridor_display   Ran 52 tests, OK
test_virtual_a_corridor           Ran 25 tests, OK
test_virtual_a_corridor_optimizer Ran 27 tests, OK
test_virtual_a_corridor_recovery  Ran 29 tests, OK
test_virtual_a_geometry           Ran 24 tests, OK
test_virtual_a_integration_bounds Ran  6 tests, OK
test_virtual_a_optimizer          Ran 37 tests, OK
test_virtual_a_pipeline           Ran  9 tests, OK
test_virtual_a_playback           Ran 34 tests, OK
test_virtual_a_receiver           Ran 21 tests, OK
test_virtual_a_trajectory         Ran 26 tests, OK
```

合计 356 项通过。`test_virtual_a_geometry` 24 项通过是在包含 TASK01 dirty 返工的工作区上得到的；未修改其期望值。

已知基线失败（原样保留、不修复、不抑制）：

```text
test_continuous_retiming: 18 tests, 1 error
ERROR test_short_translation_has_finite_acceleration_and_zero_end_speed
KeyError: 'dynamic_peaks'
exit=1
```

该失败在 TASK01 开发交付与 TASK01 审核报告中均记为既有问题；TASK02 不归因于本次工作，也不修改该测试。

复跑命令（在仓库根目录，source ROS 环境后）：

```bash
source catkin_ws/devel/setup.bash
export PYTHONPATH="$PWD/tests:$PWD/catkin_ws/src/fwis_v3_optimizer/src:$PYTHONPATH"
python3 -m unittest tests.test_virtual_a_corridor   # 其余模块同理
```

## 4. 默认入口与地图

`./run_demo.sh --map map05 --scale 2.625 --dry-run` 的实际 roslaunch 行（`run_demo_dry_run.txt`）：

```text
roslaunch fwis_v3_demo formation_demo.launch map_dir:=/tmp/expv3-map-scale.O4uf9u/map05-p12-parking-row-env2 backend:=virtual_rigid_a virtual_a_guidance_mode:=corridor virtual_a_corridor_enforcement:=hard auto_goal:=false scene:=manual_map rviz_config:=/tmp/expv3-map-scale.O4uf9u/formation.rviz rviz:=true
```

即默认 backend/guidance/enforcement/auto_goal 分别为 `virtual_rigid_a` / `corridor` / `hard` / `false`；当前 GUI 环境默认 `rviz:=true`。`map_dir` 和 `rviz_config` 是运行时生成的临时路径，不能假设下次仍存在；复现时应重新运行脚本。

地图 SHA-256（`maps_sha256.txt`）：

- map05 `maps/map05-p12-parking-row-env2/map.pgm`
  `0856ad0af795b09e17e01e41bbf7c0f819170f53c93c6df718ed8342749ba718`，
  `map.yaml` `96f12a5355be92fffdd114df5944eb592fa96db6e875a9e124949a7d92c3d8ee`，
  `resolution: 0.024713`，`origin: [1.4578, 0.9719, 0.0]`；
- 冻结 S1 `evaluation/virtual_formation_a/G4/20261116T081458Z/scenes/maps/S1.pgm`
  `0e975d9a49a2e1f2d6fc64f68f0536def44880587e0df4ff33f5fe4eac5175b8`，
  `S1.yaml` `6975052b78f654502e8311bfa894240a3fa4d53dc6807e8d06d5e9d17c95cfc4`。

## 5. 保护约束（本任务内生效）

- 不 reset/checkout/clean/覆盖/移动/删除上述 TASK01 dirty 内容和既有未跟踪文件；
- 不修改 `geometry.py`、`tests/test_virtual_a_geometry.py`；若确需修改，先停止并报告；
- 不修改历史 `TASK01_evidence/`、`evaluation/`、输入方案文档、模型 fixture 或已接受测试期望；
- 不 commit/push；TASK02 完成后停止，不进入 TASK03。