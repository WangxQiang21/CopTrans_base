# G0 真实接口与安全入口核对

日期：2026-11-15。范围：**仅 G0**。本文记录实现时读取到的实际接口，不构成 Codex 验收，也不表示 G1–G5 已实施。

## 1. 工作区与保护范围

- 实现目录：`/home/wq/whut_thesis/expv3_multi_virtual_a`。
- 分支：`feature/virtual-rigid-formation-a`。
- HEAD：`567bdecba9676313da3bc9a97b82f57b5b569a2b`，与指定 FR06 基线一致。
- 开始前仅有准备说明中列出的未跟踪设计文档，没有 tracked/staged 修改。
- 工具递归保护 `catkin_ws/src`、`tests`、旧 `tools`、`maps`、`run_demo.sh`、`run_test.sh` 和整个 `evaluation/formation_refactor_v02`。新工具自身及 Python 缓存明确排除；新证据只写入 `evaluation/virtual_formation_a/`。
- 原 T5 工作区、原 V0.2 工作区和永久备份不由本工具写入。本阶段不运行 Git 写操作。

## 2. 运行时 API 核对

以下签名由当前解释器通过 `inspect.signature` 读取；最终机器可读快照见本次 run 的 `api_interfaces.json`。

| 接口 | 实际签名/位置 | G0 用法 |
|---|---|---|
| `VehicleOffset` | `(vehicle_id: int, x: float, y: float)`；`formation.py:58` | 独立构造 `1:(1,0), 2:(0,1), 3:(0,-1)`。 |
| `FormationPose` | `(x: float, y: float, psi: float)`；`formation.py:74` | 独立构造原起点与目标。 |
| `FormationModel` | `(offsets: Iterable[VehicleOffset])`；`formation.py:128` | 直接复用旧模型，不建立新几何类型。 |
| `VehicleShape` | `(length_m: float, width_m: float)`；`formation_collision.py:57` | 三车均为 `0.738 × 0.500 m`。 |
| `Grid` | `(occupied, resolution, origin_x, origin_y, length=0.738, width=0.5, margin=0.04)`；`core.py:428` | 使用 `Grid(occupied, 0.05, 0.0, 0.0)`。 |
| `FormationTimingLimits` | 实际默认字段见 `formation_retiming.py:119-144` | 显式使用 `FormationTimingLimits()` 并序列化全部实际值。 |
| `UniformAuditRequest` | `(plan_id, formation_id, model, vehicle_shapes, start_qf, start_headings, goal_qf, goal_headings, timing_limits=<factory>, allow_reverse=False)`；`uniform_quality.py:219` | 显式传 `allow_reverse=True`，不从被审 plan 构造请求。 |
| `UniformTimedPlan.from_model_json` | `(payload)`；`uniform_trajectory.py:1158` | 只读冻结模型，身份字段仅取 `plan_id/formation_id`。 |
| `UniformTimedPlan.sample` | `(self, times=None)`；`uniform_trajectory.py:1014` | 从冻结模型重新生成投影，不读取历史投影作为审核输入。 |
| `audit_uniform_timed_plan` | `(request, plan, projection, grid, config=None)`；`uniform_quality.py:3378` | 以独立请求、新投影和 PGM 重建地图重新审核。 |

任务书所列接口均存在，未发现需要修改旧源码的调用冲突。`UniformTimedPlan.from_model_json` 的实际定义行是 1158；任务书未给该函数固定行号，因此这不是接口变更。

## 3. FR06 参考审核输入隔离

`reaudit_fr06_reference` 直接写入以下来源常量：

- offsets：`1:(1,0), 2:(0,1), 3:(0,-1)`；
- shapes：三车 `(0.738,0.500)`；
- `start_F=(6,6,0)`，初始车头均为 0；
- `goal_F=(10,10,pi/2)`；
- 终端车头 `{1:pi/2+atan2(1,2), 2:pi/2, 3:pi/2}`；
- `FormationTimingLimits()` 与旧参考合同 `allow_reverse=True`。

被审模型只提供身份并作为审核对象，不提供审核请求的起点、目标、车头或限值。地图从 `FR06/maps/corridor_w2.8/map.pgm` 读取，按 `flipud(image < 127)` 构造占据数组。工具不导入或执行历史 `verify_fr06.py`、`run_experiments.py`、`freeze_reference.py`，也不进行新搜索。

## 4. 安全回归清单

工具只按文件名逐条调用 `python3 -B -m unittest discover -s tests -p PATTERN -v`，不使用全目录 discover，不调用历史实验脚本。

| level | 显式测试文件 |
|---|---|
| `baseline` | 无旧测试子进程；仅依赖检查与 FR06 冻结参考复审。 |
| `geometry` | `test_continuous_geometry.py`、`test_formation_geometry.py`。 |
| `timing` | `test_continuous_retiming.py`、`test_formation_retiming.py`、`test_uniform_timing.py`、`test_uniform_trajectory.py`、`test_uniform_quality.py`。 |
| `full` | geometry/timing 全部，加 `test_uniform_motion.py`、`test_uniform_search.py`、`test_uniform_pipeline.py`。 |

G0 只运行 `baseline`。`full` 是安全旧单元测试入口，不冒称完整 G5：新 FR06 自主搜索和真实 ROS 仍需后续明确授权及专门证据，本工具不会启动它们。

## 5. 输出重定向核对

逐文件阅读上述会写证据的旧测试后，确认所有 FR01–FR04 长期结果均读取下列环境变量。runner 为**每个测试用例**设置独立目录 `regression_artifacts/<case_id>/`，并为每个子进程同时设置全部九个变量：

| 变量 | 每个 case 下的重定向目标 |
|---|---|
| `FR01_RESULTS_PATH` | `FR01/primitive_results.json` |
| `FR02_TIMING_RESULTS_PATH` | `FR02/A/timing_results.json` |
| `FR02_TRAJECTORY_RESULTS_PATH` | `FR02/A/trajectory_results.json` |
| `FR02_QUALITY_RESULTS_PATH` | `FR02/B/uniform_quality_results.json` |
| `FR02_ARTIFACT_DIR` | `FR02/artifacts` |
| `FR03_RESULTS_PATH` | `FR03/successors.json` |
| `FR03_ARTIFACT_DIR` | `FR03` |
| `FR04_RESULTS_PATH` | `FR04/pipeline_results.json` |
| `FR04_ARTIFACT_DIR` | `FR04` |

因此 FR02-A 的 timing/trajectory 与 FR02-B 的 quality 不共享 artifact 目录，即使用户在以后分别运行它们也不会互相覆盖。geometry 和未写长期文件的 timing 测试也获得同样的隔离环境，避免后续测试增加输出时漏配。子进程环境通过 `subprocess.run(env=...)` 传递，不经 shell 拼接。

每个子进程保存完整 argv、cwd、解释器、环境增量、stdout、stderr、退出码和墙钟；权限错误记录 `NOT_RUN_PERMISSION`，启动失败记录 `NOT_RUN`，非零退出记录 `FAIL`，只有退出码 0 记录 `PASS`。

## 6. 阶段授权门禁（R1 修复）

门禁与回归级别一一绑定：`baseline→G0`、`geometry→G1`、`timing→G3`、`full→G5`。门禁**重新读取原始 `git status` 行并按当前阶段重新判定**，不信任 manifest 中已有的 `unknown_modifications` 或“已允许”标签。

| 阶段 | 允许新增 | 允许修改的旧文件 |
|---|---|---|
| G0 | 仅 G0 工具、接口说明及 `evaluation/virtual_formation_a/**` | 无 |
| G1 | G0 全部；`formation_continuous_a/{__init__,contracts,geometry,bounds}.py`、两个 G1 测试、`tools/virtual_a_benchmark.py` | `catkin_ws/src/fwis_v3_optimizer/setup.py` |
| G3 | G1 全部；`optimizer.py`、`audit.py`、`trajectory.py`、`timing.py`、对应测试 | 同 G1 |
| G5 | G3 全部；`collision.py`、`pipeline.py`、`ros_adapter.py`、`virtual_a_scenes.py`、G4/G5 测试、`tests/fixtures/virtual_a/maps/**` | G1 的 `setup.py`，加 G5 任务书列出的节点、播放、bridge、launch、`run_demo.sh`、优化器 `CMakeLists.txt` |

规则：

- 只接受 `A/M` 型 tracked 变更；删除、重命名、类型变化一律拒绝。
- 新增文件必须精确命中清单或声明前缀；`tests/fixtures/virtual_a/maps/**` 只在 G4 起开放。
- 车辆 `transport_formation.yaml`、旧 `uniform_*`/formation 核心、旧测试预期、旧地图、历史证据均不在任何阶段清单内，仍拒绝。
- 这是**路径授权**，不证明内容合规；每个实际 diff 仍须 Codex 按对应任务书核对最小性与范围。
- 未知路径仍以 `UNKNOWN_WORKSPACE_MODIFICATIONS` 拒绝；未关闭门禁，也不允许任意 dirty。

## 7. 公共 runner 输出保护（R2 修复）

`run_regression` 与 CLI 共用 `_resolve_evidence_output`：

- 输出必须在 `evaluation/virtual_formation_a/` 下，且不能是证据根目录本身；符号链接在 resolve 前拒绝。
- 在**启动任何子进程或写任何文件之前**，预检该 level 全部 case 的 `regression_artifacts/<case_id>/` 与 `logs/<case_id>.{stdout,stderr}.log`；任一已存在即 `REGRESSION_OUTPUT_EXISTS` 整体拒绝。
- 预检同时核对每个目标 resolve 后仍位于 run 目录内，防止符号链接逃逸。
- CLI 正常运行不受影响：`main` 先建立的 manifest/环境元数据不在这些 case 路径内。

## 8. CLI 与失败语义

```bash
python3 -B tools/virtual_a_baseline.py --output DIR --regression baseline
python3 -B tools/virtual_a_baseline.py --verify-manifest FILE
```

- 已存在且非空的输出目录以 `OUTPUT_EXISTS_NONEMPTY` 拒绝，返回非零。
- `--verify-manifest` 按清单内 `workspace_authorization_stage` 复核分类，只读、不创建报告；哈希或路径不一致返回非零。
- 分支/基线错误、阶段外修改、缺失保护路径均在写证据前停止。
- before/after 对整个保护集合重新计算 SHA-256，而不只比较 Git tracked 文件。
- 当前解释器及 numpy/scipy/casadi/PyYAML/Pillow/rospy、独立 CasADi/IPOPT 标量求解、ROS setup 文件和未执行项以本次 run 的 `environment.json` 为准。

## 9. 本阶段边界

没有新增 `formation_continuous_a` 包，没有实现新几何、优化、时间或地图算法，没有运行 FR06 搜索、ROS/Gazebo、G1–G5 测试，没有修改旧测试预期、车辆配置或历史证据。R1/R2 验证使用内存 manifest 与 mock，未实际实施 G1。完成后停止，等待 Codex 复验。
