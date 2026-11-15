# FR04 实际命令与退出码

工作目录：`/home/wq/whut_thesis/expv3_multi_v02`。所有 Python 命令均为本机单进程；未启动 ROS，未访问原仓库，未提交 Git。

## 1. 编译检查

```bash
python3 -m py_compile \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_search.py \
  catkin_ws/src/fwis_v3_optimizer/src/fwis_v3_optimizer/uniform_pipeline.py \
  tests/test_uniform_pipeline.py
```

退出码 0。

## 2. FR04 测试（最终轮）

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \
  -p 'test_uniform_pipeline.py' -v \
  > evaluation/formation_refactor_v02/FR04/test_uniform_pipeline.log 2>&1
```

退出码 0；`Ran 12 tests in 38.334s ... OK`。日志：`test_uniform_pipeline.log`。

首轮（接口错误）与第二轮（一项断言错误）分别退出码 1，见第 6 节。

## 3. FR03 已验收套件

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \
  -p 'test_uniform_search.py' -v \
  > evaluation/formation_refactor_v02/FR04/logs_fr03_tests.log 2>&1
```

退出码 0；`Ran 12 tests in 159.665s ... OK`。

## 4. 十个相关回归套件

```bash
for t in test_formation_collision test_formation_geometry test_formation_hold \
         test_formation_kinematics test_formation_quality test_formation_retiming \
         test_uniform_motion test_uniform_quality test_uniform_timing \
         test_uniform_trajectory; do
  OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests -p "$t.py" -v \
    > "evaluation/formation_refactor_v02/FR04/regression/$t.log" 2>&1
  echo "$t exit=$?"
done
```

每次退出码 0；合计 145 项：

| 套件 | 项数 |
|---|---|
| test_formation_collision | 17 |
| test_formation_geometry | 15 |
| test_formation_hold | 14 |
| test_formation_kinematics | 28 |
| test_formation_quality | 22 |
| test_formation_retiming | 14 |
| test_uniform_motion | 13 |
| test_uniform_quality | 9 |
| test_uniform_timing | 5 |
| test_uniform_trajectory | 8 |

汇总日志：`logs_regression_suites.log`。

## 5. 基线核验

```bash
python3 -I evaluation/formation_refactor_v02/FR04/verify_baseline.py
```

退出码 0；输出 `all_accepted_match=true`、`all_frozen_match=true`、`fr03_prefix_match=true`。结果：`baseline_verification.json`。

## 6. 保留的失败轮次

| 轮次 | 命令 | 退出码 | 现象 | 处置 |
|---|---|---|---|---|
| 首轮 | 同第 2 节 | 1 | 12 项均在搜索前 `TypeError: cannot unpack non-iterable int object`（`model.centers()` 返回字典被按序列解包） | 改为遍历 `.values()`；日志归档 `failures/test_uniform_pipeline_initial_center_iteration_error.log` |
| 第二轮 | 同第 2 节 | 1 | 11/12 通过；F03 断言误把倒车写成负参数长度 | 改为检查逐车 `direction=-1`；整轮日志与 JSON 归档 `failures/reverse_assertion_run/` |
| 第三轮 | 同第 2 节 | 0 | 12/12 通过，但终审耗时尚未计入时间预算、终审拒绝尚无分布字段 | 修正搜索后整轮归档 `previous_runs/round1_pre_terminal_budget_fix/` |
| 第四轮 | 同第 2 节 | 0 | 12/12 通过，但证据中源码哈希键为裸文件名 | 改为仓库相对路径后归档 `previous_runs/round2_final_before_path_keys/`；最终轮见第 2 节 |

失败轮次的 `cases/` 与 `pipeline_results.json` 一并保留，未被最终结果覆盖。

## 7. 证据清单生成

```bash
python3 -I evaluation/formation_refactor_v02/FR04/make_manifest.py
```

退出码 0；写出 `evidence_manifest.json` 与 `command_manifest.json`。

## 8. 未运行内容

- 未运行 ROS/RViz、消息发布、控制器、Gazebo 或任何动力学仿真。
- 未运行全地图随机基准、最优性/完备性实验或 C++ 后端对照。
- 未修改、未重跑原仓库 `/home/wq/whut_thesis/expv3_multi`。
- 未执行任何 Git 提交、reset、stash 或 worktree 操作。

## 9. FR04-R1 返工命令（2026-11-14）

所有新证据写入 `FR04/rework_01/`，使用环境变量避免覆盖旧结果：

```bash
FR04_ARTIFACT_DIR="$PWD/evaluation/formation_refactor_v02/FR04/rework_01/final" \
FR04_RESULTS_PATH="$PWD/evaluation/formation_refactor_v02/FR04/rework_01/final/pipeline_results.json" \
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \
  -p 'test_uniform_pipeline.py' -v \
  > evaluation/formation_refactor_v02/FR04/rework_01/final/test_uniform_pipeline.log 2>&1
```

退出码 0；`Ran 19 tests in 44.782s ... OK`。

```bash
OPENBLAS_NUM_THREADS=1 python3 -m unittest discover -s tests \
  -p 'test_uniform_search.py' -v \
  > evaluation/formation_refactor_v02/FR04/rework_01/fr03_tests.log 2>&1
```

退出码 0；`Ran 12 tests in 158.544s ... OK`。

十个回归套件同第 4 节循环，日志写入 `rework_01/regression/`，汇总于
`rework_01/logs_regression_suites.log`；10 个退出码均为 0，合计 145 项。

```bash
OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 python3 \
  evaluation/formation_refactor_v02/FR04/rework_01/codex_probe/adapted_independent_probes.py \
  > evaluation/formation_refactor_v02/FR04/rework_01/codex_probe/adapted_independent_probes.log 2>&1
```

退出码 0；注入计划为 `AUDIT_REJECTED / PLAN_START_MISMATCH`、不可执行、无 plan；
同端点换路径为 `PLAN_EDGE_SEQUENCE_MISMATCH`；失败后新请求 ID=402 为 `SUCCESS`。

```bash
python3 -I evaluation/formation_refactor_v02/FR04/rework_01/verify_baseline_rework.py
```

退出码 0；输出写入 `rework_01/baseline_verification.json`，未覆盖原
`baseline_verification.json`。

### 9.1 返工中的失败/中间轮次

| 轮次 | 命令 | 退出码 | 现象 | 处置 |
|---|---|---|---|---|
| `iteration_02/` | 单跑 F18 | 1 | 替换 plan 用低 `v_max` 定时但边沿用原 MotionLimits，自审 `UNIFORM_MODEL_MISMATCH` | 同步构造边与 plan，重跑通过 |
| `iteration_03/` | 19 项全跑 | 1 | 过度要求每条边 `allow_reverse` 等于请求，F06/F07/F08 终审全拒并耗尽 30 s | 删除该过度约束；`iteration_04/` 恢复 27/27/31 扩展 |
| `previous_runs/final_before_docstring_edit/` | 19 项全跑 | 0 | 19/19 通过，但随后测试 docstring 修改导致源码哈希失效 | 归档整轮，最终源码重跑 `final/` |

另有一次修复前冒烟运行漏设 `FR04_ARTIFACT_DIR/FR04_RESULTS_PATH`，覆盖了
`FR04/pipeline_results.json` 与 F01/F02 两个逐例文件。被覆盖后产生的 2 例文件保存在
`rework_01/accidental_overwrite/`；原文件哈希见 `pre_rework_evidence_manifest.json`，
字节内容无法恢复。其余 F03–F12 文件与全部日志未覆盖。详见
`rework_01/REWORK_REPORT.md` 第 6.1 节。